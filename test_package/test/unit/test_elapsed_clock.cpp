// Conan::ImportStart
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

#include <gtest/gtest.h>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace {

namespace core = foc::core;

constexpr float kDt = 2.5e-5F;           // a 40 kHz control period
constexpr long kTicksTo512S = 20480000L; // 512 s / 25 us

} // namespace

TEST(ElapsedClock, APeriodIsQuantisedToWholeNanoseconds) {
  // The clock's unit, asserted at the boundary rather than assumed: a period is
  // converted once per sample, so its rounding is the only error the clock can
  // make.
  // 时钟的单位，在边界上断言而不是假定：周期每个采样转换一次，所以它的舍入是时钟唯一可能犯的错。
  EXPECT_EQ(25000U, core::ElapsedClock::to_nanos(2.5e-5F));
  EXPECT_EQ(1000U, core::ElapsedClock::to_nanos(1.0e-6F));
  EXPECT_EQ(1000000000U, core::ElapsedClock::to_nanos(1.0F));
  // Anything not positive adds nothing: negative periods, zero, NaN.
  // 非正值都不累加：负周期、零、NaN。
  EXPECT_EQ(0U, core::ElapsedClock::to_nanos(0.0F));
  EXPECT_EQ(0U, core::ElapsedClock::to_nanos(-1.0F));
  EXPECT_EQ(0U, core::ElapsedClock::to_nanos(
                    std::numeric_limits<float>::quiet_NaN()));
  // And an absurd recipe saturates instead of wrapping -- a wrap would turn a
  // long timeout into an immediate one.
  // 荒谬的配方是饱和而不是回绕——回绕会把长超时变成立刻超时。
  EXPECT_EQ(
      core::ElapsedClock::kMaxNanos,
      core::ElapsedClock::to_nanos(std::numeric_limits<float>::infinity()));
  EXPECT_EQ(core::ElapsedClock::kMaxNanos,
            core::ElapsedClock::to_nanos(1.0e30F));

  core::ElapsedClock clock;
  clock.add_sample(0.0F);
  clock.add_sample(-1.0F);
  EXPECT_EQ(0U, clock.nanos()) << "a non-positive period advanced the clock";
  // A positive period always advances it, however small: a period the clock
  // silently ignores is the defect.
  // 正周期一定推进它，无论多小：一个会被时钟静默忽略的周期，正是那个缺陷。
  clock.add_sample(1.0e-12F);
  EXPECT_GT(clock.nanos(), 0U);
  clock.reset();
  EXPECT_EQ(0U, clock.nanos());
}

TEST(ElapsedClock, TheSumOfTwentyMillionPeriodsIsFiveHundredAndTwelveSeconds) {
  // L3: the same sum in float next to the same sum in the clock. This is the
  // falsifier for the whole class -- if a float accumulator were good enough,
  // these two columns would agree.
  // L3：同一个累加，浮点一列、时钟一列。这是整类缺陷的反证——如果浮点累加器够用，两列会一致。
  core::ElapsedClock clock;
  float as_float = 0.0F;
  long clock_at_512 = -1;
  long float_at_512 = -1;
  for (long tick = 0; tick < kTicksTo512S; ++tick) {
    clock.add_sample(kDt);
    as_float += kDt;
    if ((float_at_512 < 0) && (as_float >= 512.0F)) {
      float_at_512 = tick + 1;
    }
    if ((clock_at_512 < 0) && clock.reached(512.0F)) {
      clock_at_512 = tick + 1;
    }
  }
  std::printf("[clock] 512 s was reached at tick %ld by the float sum (%.3f s "
              "of real time) and at tick %ld by the "
              "clock (%.3f s)\n",
              float_at_512,
              static_cast<double>(float_at_512) * static_cast<double>(kDt),
              clock_at_512,
              static_cast<double>(clock_at_512) * static_cast<double>(kDt));

  // Exact, and that is the point: 20480000 x 25000 ns is 512 s to the
  // nanosecond, and it is reached on the last tick and not before.
  // 精确，而且这正是重点：20480000 × 25000 ns 就是 512 s，而且它是在最后一个
  // tick 到达的，不是更早。
  EXPECT_EQ(static_cast<std::uint64_t>(kTicksTo512S) * 25000U, clock.nanos());
  EXPECT_FLOAT_EQ(512.0F, clock.seconds());
  EXPECT_EQ(kTicksTo512S, clock_at_512)
      << "the clock did not take exactly the ticks its own arithmetic says";

  // The falsifier. The float sum reaches 512 s **early** -- measured at tick 17
  // 866 579, i.e. 446.66 s of real time, 12.8 % fast -- because past `2^23*dt`
  // each addition advances the sum by an ULP rather than by `dt`. It then sits
  // there: by the end of the loop it still reads 512, which is the freeze. A
  // timer built on that number fires a 512 s timeout 65 s early, and a 520 s
  // timeout never fires at all. 反证。浮点累加**提前**到达 512 s——实测第 17 866
  // 579 tick，即真实 446.66 s，快 12.8%——因为越过 `2^23*dt`
  // 之后每次加法推进的是一个 ULP 而不是
  // `dt`。然后它就停在那里：循环结束时读数仍是 512，这就是冻结。建在这个
  // 数上的定时器会让 512 s 的超时早 65 s 触发，而 520 s 的超时永不触发。
  ASSERT_GT(float_at_512, 0) << "the float mirror never reached 512 s, so this "
                                "case stopped discriminating";
  EXPECT_LT(float_at_512, (kTicksTo512S * 88L) / 100L)
      << "the float mirror no longer runs fast, so it can no longer "
         "show what the clock fixes";
  EXPECT_FLOAT_EQ(512.0F, as_float)
      << "the float sum should have frozen at 512 s, not passed it";

  EXPECT_FALSE(clock.reached(513.0F)) << "the clock fired before its own limit";
  EXPECT_TRUE(clock.reached(512.0F))
      << "the clock did not reach its own limit at 512 s";

  // And it carries on past the old freeze point: 520 s is reachable, where the
  // float sum could not advance at all. 而且它能越过旧的冻结点：520 s
  // 可达，而浮点和在那里根本加不动。
  while (!clock.reached(520.0F)) {
    clock.add_sample(kDt);
  }
  std::printf("[clock] after carrying past the old floor: %.6f s\n",
              static_cast<double>(clock.seconds()));
  EXPECT_FLOAT_EQ(520.0F, clock.seconds());
}
