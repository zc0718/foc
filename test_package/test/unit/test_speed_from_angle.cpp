// Speed from an angle: the two config clamps, the reset, and the ticks that ask for nothing.
//
// `SpeedFromAngle` was exercised only through SIL runs, which always hand it a sane config and never reset it
// mid-run. That left the defensive branches -- the ones that decide what a *wrong* config does -- unexecuted.
// A clamp that is never run is a clamp nobody has checked, and the divisor is the first place a zero pole count
// would show up as `inf`.
//
// 由角度求速度：两处配置夹取、reset、以及"什么都没要"的 tick。
// `SpeedFromAngle` 此前只在 SIL 里跑，配置永远合理、也从不中途 reset，于是那些决定"配置错了会怎样"的防卫分支
// 从未被执行。没跑过的夹取等于没验证过；而极对数取 0 时，除法的第一个后果就是 `inf`。
//
// The criteria, fixed before the numbers below were measured:
//   C1  `pole_pairs = 0` is treated as 1: a 100 rad/s electrical ramp reads 100 rad/s mechanical, finite. Left
//       unclamped it would divide by zero.
//   C2  `window_ticks = 0` is treated as the documented minimum (2): not ready after one sample, ready after two.
//   C3  a `window_ticks` above `kMaxWindowTicks` neither writes past the buffer nor stops converging: after the
//       maximum number of samples it is ready and reports the true speed.
//   C4  `reset()` returns to the unprimed state, so the sample count starts again -- and a sample far from the
//       pre-reset angle does not produce a spike, because there is no previous angle to subtract.
//   C5  a tick with dt <= 0 is ignored rather than accumulated: an estimate that was ready and correct stays so.
//
// 判据写在量之前：C1 极对数 0 视作 1（100 rad/s 电→100 rad/s 机械，且有限）；C2 窗口 0 视作下限 2（1 拍未就绪、
// 2 拍就绪）；C3 窗口超过上限时既不越界也能收敛；C4 reset 回到未就绪状态，且重置后远离原角度的采样不产生尖峰；
// C5 dt<=0 的 tick 被忽略而不是累积。

#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/speed_from_angle.hpp>

namespace {

namespace core = foc::core;

constexpr float kDt = 0.001F;        // 1 kHz
constexpr float kDelta = 0.1F;       // electrical radians per tick -> 100 rad/s
constexpr float kTrueSpeed = kDelta / kDt;

/**
 * @brief One sample sequence at a constant electrical rate, starting from `from` so a reset can be made visible.
 */
auto feed(core::SpeedFromAngle& estimator, int count, float from = 0.0F) -> float {
  float angle = from;
  for (int i = 0; i < count; ++i) {
    estimator.update(angle, kDt);
    angle += kDelta;
  }
  return angle;
}

} // namespace

TEST(SpeedFromAngle, C1_AZeroPoleCountIsOnePoleNotADivision) {
  core::SpeedFromAngle estimator{core::SpeedFromAngle::Config{4U, 0U}};
  feed(estimator, 8);
  ASSERT_TRUE(estimator.ready());
  EXPECT_TRUE(std::isfinite(estimator.mech_rad_s()));
  EXPECT_NEAR(estimator.mech_rad_s(), kTrueSpeed, 1.0F) << "a zero pole count must not divide by zero";
}

TEST(SpeedFromAngle, C2_AWindowBelowTwoIsTheMinimumWindow) {
  core::SpeedFromAngle estimator{core::SpeedFromAngle::Config{0U, 1U}};
  estimator.update(0.0F, kDt);
  EXPECT_FALSE(estimator.ready()) << "one sample cannot span an interval";
  estimator.update(kDelta, kDt);
  ASSERT_TRUE(estimator.ready());
  EXPECT_NEAR(estimator.mech_rad_s(), kTrueSpeed, 1.0F);
}

TEST(SpeedFromAngle, C3_AWindowAboveTheMaximumStillConverges) {
  const std::uint16_t huge = static_cast<std::uint16_t>(core::SpeedFromAngle::kMaxWindowTicks * 4U);
  core::SpeedFromAngle estimator{core::SpeedFromAngle::Config{huge, 1U}};
  feed(estimator, static_cast<int>(core::SpeedFromAngle::kMaxWindowTicks) - 1);
  EXPECT_FALSE(estimator.ready()) << "the clamped window is the maximum, so it fills at the maximum";
  feed(estimator, 1, (core::SpeedFromAngle::kMaxWindowTicks - 1U) * kDelta);
  ASSERT_TRUE(estimator.ready());
  EXPECT_NEAR(estimator.mech_rad_s(), kTrueSpeed, 1.0F);
}

TEST(SpeedFromAngle, C4_ResetStartsTheCountAgainWithoutASpike) {
  core::SpeedFromAngle estimator{core::SpeedFromAngle::Config{4U, 1U}};
  feed(estimator, 8);
  ASSERT_TRUE(estimator.ready());

  estimator.reset();
  EXPECT_FALSE(estimator.ready());
  EXPECT_FLOAT_EQ(estimator.mech_rad_s(), 0.0F);

  // A sample far from the pre-reset angle: without a cleared previous angle this is a 5 rad jump in one tick,
  // i.e. ~5000 rad/s. With the reset it is just the first sample of a new window.
  float angle = feed(estimator, 1, 5.0F);
  EXPECT_FALSE(estimator.ready());
  for (int i = 0; i < 8; ++i) {
    estimator.update(angle, kDt);
    angle += kDelta;
  }
  ASSERT_TRUE(estimator.ready());
  EXPECT_NEAR(estimator.mech_rad_s(), kTrueSpeed, 1.0F) << "reset must not leave a stale previous angle";
  EXPECT_FALSE(estimator.lost_count());
}

TEST(SpeedFromAngle, C5_ATickWithNoTimeIsIgnored) {
  core::SpeedFromAngle estimator{core::SpeedFromAngle::Config{4U, 1U}};
  feed(estimator, 8);
  ASSERT_TRUE(estimator.ready());
  const float settled = estimator.mech_rad_s();

  for (const float dt : {0.0F, -0.001F}) {
    estimator.update(settled + 1.5F, dt);   // a large angle at zero time would be an enormous speed
    EXPECT_TRUE(estimator.ready());
    EXPECT_NEAR(estimator.mech_rad_s(), settled, 1.0e-3F) << "dt = " << dt << " must not be accumulated";
  }
}
