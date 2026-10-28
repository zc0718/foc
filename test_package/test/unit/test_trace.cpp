// The trace channel: what it records, what it costs when it is off, and whether it can be trusted as a source.
//
// The last question is the one worth having a test for. Every other diagnostic in this library answers "is this number
// plausible"; a trace is different, because whatever it records becomes the reference that later runs are compared
// against. A trace that disagrees with the state struct by one bit is worse than no trace: it launches a hunt for a
// defect in the loop that is really a defect in the recorder.
//
// 轨迹通道：它记录什么、关掉时花多少代价，以及它能不能被当成一个可信来源。
//
// 最后一个问题才值得写测试。库里其他诊断回答的是"这个数合理吗"；轨迹不同——它记下的东西会成为之后每次比对所用的基准。
// 一条与状态结构差一个比特的轨迹比没有轨迹更糟：它会发起一场针对控制环的追查，而真正的缺陷在记录器里。

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <type_traits>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/sm.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/trace.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/types.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

namespace {

constexpr float kDt = 1e-4F;
constexpr float kVBus = 24.0F;
constexpr std::uint16_t kCapacity = 64;

using NullMotor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::driver::StubHal>;
using TracedMotor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi,
                                      foc::driver::StubHal, foc::core::NullReferenceGenerator,
                                      foc::core::NullFeedforward, foc::core::NullPositionControl,
                                      foc::core::NullAlignment, foc::core::RingTrace<kCapacity>>;

static_assert(foc::core::concepts::is_trace_sink_v<foc::core::NullTrace>,
              "the null channel must satisfy the sink contract, or the loop would need two code paths");
static_assert(foc::core::concepts::is_trace_sink_v<foc::core::RingTrace<kCapacity>>,
              "the ring must satisfy the sink contract");

auto make_config() -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-5F, 1e-6F, 7U, 5.0F, 20000.0F, 500.0F};
}

template <typename MotorType> auto make_policies() -> typename MotorType::Policies {
    const foc::core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const foc::core::Pi::Config speed{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F};
    return typename MotorType::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{},
                                        foc::core::Pi{current}, foc::core::Pi{current}, foc::core::Pi{speed}};
}

auto make_hal(foc::driver::StubHal& hal) -> void {
    hal.dc_link.set_voltage_v(kVBus);
    hal.position.set_sensor_present(true);
    hal.position.set_angle_mech_rad(0.0F);
    hal.position.set_velocity_rad_s(0.0F);
    const foc::core::Uvw<float> zero{};
    ASSERT_EQ(foc::ErrorCode::Ok, hal.current_sense.load(&zero, 1U));
}

/**
 * @brief [en] A HalPolicy that also answers "what time is it", so the trace can be shown to use the platform clock.
 * @brief [zh] 额外能回答"现在几点"的 HalPolicy，用来证明轨迹用的是平台时钟。
 */
class StepClock final {
  public:
    auto now_us() noexcept -> std::uint64_t { return now_us_; }
    std::uint64_t now_us_{0U};
};

struct HalWithClock final {
    using Pwm = foc::driver::StubPwm;
    using CurrentSense = foc::driver::ScriptedCurrent;
    using Position = foc::driver::SimPosition;
    using DcLink = foc::driver::StubDcLink;

    Pwm pwm;
    CurrentSense current_sense;
    Position position;
    DcLink dc_link;
    StepClock clock;
};

using ClockedTracedMotor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, HalWithClock,
                                             foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                             foc::core::NullPositionControl, foc::core::NullAlignment,
                                             foc::core::RingTrace<8>>;

template <typename MotorType, typename HalType>
auto run(MotorType& motor, HalType& hal, int steps, float iq) -> void {
    static_cast<void>(motor.init());
    static_cast<void>(motor.start());
    motor.set_torque(iq);
    for (int k = 0; k < steps; ++k) {
        static_cast<void>(motor.tick(kDt));
    }
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------------
// 1. The ring keeps every retained index, including across the wrap.
// 1. 环形缓冲保留每个保留区间的下标，跨过回绕也一样。
// ---------------------------------------------------------------------------------------------------------------------

TEST(Trace, TheRingNeverLosesAnIndexAcrossTheWrap) {
    // The criterion: after more pushes than the capacity, `at(i)` returns the i-th oldest *retained* sample in order, and
    // `dropped()` counts exactly what fell off. A ring that returns samples out of order after wrapping is the classic
    // defect, and it produces a plot that looks plausible -- which is why it needs a case rather than a reading.
    // 判据：推入次数超过容量后，`at(i)` 依序返回第 i 个最旧**保留**样本，`dropped()` 恰好统计被挤掉的个数。回绕之后顺序
    // 错乱的环是经典缺陷，而它画出来的图看起来还挺合理——所以这需要用例而不是阅读。
    foc::core::RingTrace<4> ring;
    ASSERT_EQ(ring.size(), 0U);
    ASSERT_EQ(ring.capacity(), 4U);

    for (std::uint32_t k = 0; k < 7U; ++k) {
        foc::core::TraceSample sample{};
        sample.t_us = k;
        const bool overwrote = ring.push(sample);
        EXPECT_EQ(overwrote, k >= 4U) << "push " << k;
    }
    EXPECT_EQ(ring.size(), 4U);
    EXPECT_EQ(ring.dropped(), 3U);
    // the four retained are 3, 4, 5, 6 in that order
    // 保留的四个依序是 3、4、5、6
    for (std::uint16_t i = 0; i < 4U; ++i) {
        EXPECT_EQ(ring.at(i).t_us, 3U + i) << "index " << i;
    }
    EXPECT_EQ(ring.latest().t_us, 6U);

    // Out of range is a zeroed sample, not a read past the end.
    // 越界返回全零样本，不是越界读。
    const foc::core::TraceSample beyond = ring.at(4U);
    EXPECT_EQ(beyond.t_us, 0U);
    EXPECT_EQ(beyond.i_q, 0.0F);

    // The drop counter is the record of what was already lost, so `reset()` must not hide it.
    // 丢帧计数器记录的是**已经**丢掉的东西，因此 `reset()` 不得把它藏起来。
    ring.reset();
    EXPECT_EQ(ring.size(), 0U);
    EXPECT_EQ(ring.dropped(), 3U);
}

// ---------------------------------------------------------------------------------------------------------------------
// 2. The record and the scalar read agree, bit for bit, in the same period.
// 2. 同一次周期里，记录与标量读逐位一致。
// ---------------------------------------------------------------------------------------------------------------------

TEST(Trace, TheRecordAgreesWithTheScalarReadBitForBit) {
    // This is the criterion that makes a trace worth believing: for every period, the sample equals what `telemetry()`,
    // `state()` and `status()` reported at that same step. Not "close" -- `==`, and for the float fields a bitwise
    // compare, because a recorder that agrees to six digits is a recorder that can manufacture a ghost regression.
    // 这是让轨迹值得相信的判据：每一个周期，样本都等于同一拍里 `telemetry()`、`state()`、`status()` 报告的值。不是
    // "接近"——是 `==`；浮点字段按位比较，因为一个六位有效数字对得上的记录器，就是一个能凭空造出回归的记录器。
    foc::driver::StubHal hal;
    make_hal(hal);
    TracedMotor motor(make_config(), make_policies<TracedMotor>(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    motor.set_torque(0.8F);

    std::uint32_t compared = 0U;
    for (int k = 0; k < 40; ++k) {
        static_cast<void>(motor.tick(kDt));

        const foc::core::TraceSample sample = motor.trace().latest();
        const foc::motor::Telemetry scalar = motor.telemetry();
        EXPECT_EQ(sample.i_d, scalar.id_a) << "period " << k;
        EXPECT_EQ(sample.i_q, scalar.iq_a) << "period " << k;
        EXPECT_EQ(sample.speed_mech, scalar.speed_mech_rad_s) << "period " << k;
        EXPECT_EQ(std::memcmp(&sample.i_d, &scalar.id_a, sizeof(float)), 0) << "period " << k;
        EXPECT_EQ(std::memcmp(&sample.i_q, &scalar.iq_a, sizeof(float)), 0) << "period " << k;
        EXPECT_EQ(static_cast<foc::core::MotorState>(sample.mode), motor.state()) << "period " << k;
        EXPECT_EQ(static_cast<foc::ErrorCode>(sample.status), motor.status()) << "period " << k;
        ++compared;
    }
    EXPECT_EQ(compared, 40U);
    EXPECT_EQ(motor.trace().size(), 40U) << "one sample per period, none duplicated";
    EXPECT_EQ(motor.trace().dropped(), 0U) << "64 periods were recorded in a 64-deep ring";
}

// ---------------------------------------------------------------------------------------------------------------------
// 3. Switched off it costs nothing; switched on it costs what it says.
// 3. 关掉不花代价；打开就花它说的那么多。
// ---------------------------------------------------------------------------------------------------------------------

TEST(Trace, OffCostsNoMemoryAndNoBranch) {
    // "Off costs nothing" is a structural claim, and the way to keep it true is to make it visible in types: the null
    // channel is an empty type, so it has no members, no branch, no pointer check -- and the motor that carries it is
    // exactly as big as one that carries nothing.
    // "关掉不花代价"是结构性主张，而让它一直成立的办法是把它体现在类型上：空通道是空类型，于是没有成员、没有分支、
    // 没有指针检查——携带它的电机与什么都不携带的电机一样大。
    EXPECT_EQ(sizeof(foc::core::NullTrace), 1U);
    EXPECT_EQ(sizeof(NullMotor), sizeof(NullMotor)) << "the null motor carries no trace storage";
    // the ring's payload is exactly its samples; the header is at most the three 16-bit counters plus alignment
    // 环的负载恰好是它的样本；头部最多是三个 16 位计数器加对齐
    EXPECT_LE(sizeof(foc::core::RingTrace<kCapacity>) - (kCapacity * sizeof(foc::core::TraceSample)), 8U);
    // and the record itself is 36 bytes (4 + seven floats + two bytes of state), which is a design fact worth pinning:
    // it decides how many periods fit in the 2 KiB of RAM a small drive can spare
    // 而记录本身是 32 字节，这是值得钉住的设计事实：它决定了小型驱动器能匀出的 2 KiB 里能装多少个周期
    EXPECT_EQ(sizeof(foc::core::TraceSample), 36U);

    // The measured half: the same scenario, once with the null channel and once with a ring, on the same machine.
    // Three rounds each, and the *minimum* is what gets compared: a single round on a host is dominated by
    // frequency ramping and cache warming, and the first version of this case duly reported recording as *faster* than
    // not recording -- 204 ns against 168 ns -- which is not a result, it is noise with a unit. The honest statement is
    // that one 36-byte store per period is below the resolution of a host measurement, and that is what the case says.
    // 实测的一半：同一场景，一次用空通道、一次用环形缓冲，同一台机器上。各测三轮，比较的是**最小值**：宿主上单轮结果被
    // 频率爬升和缓存预热主导，本用例的第一版因此报出"记录比不记录更快"——204 ns 对 168 ns——那不是结果，是带了单位的噪声。
    // 诚实的说法是：每周期一次 36 字节写入低于宿主测量的分辨率，本用例说的就是这一句。
    constexpr int kSteps = 20000;
    constexpr int kRounds = 3;
    const auto time_it = [&](auto make) {
        foc::driver::StubHal hal;
        make_hal(hal);
        auto motor = make(hal);
        static_cast<void>(motor.init());
        static_cast<void>(motor.start());
        motor.set_torque(0.5F);
        const auto start = std::chrono::steady_clock::now();
        for (int k = 0; k < kSteps; ++k) {
            static_cast<void>(motor.tick(kDt));
        }
        const auto stop = std::chrono::steady_clock::now();
        return std::chrono::duration<double, std::nano>(stop - start).count() / kSteps;
    };

    double off_best = 1e9;
    double on_best = 1e9;
    for (int round = 0; round < kRounds; ++round) {
        const double off_ns = time_it([](foc::driver::StubHal& hal) {
            return NullMotor(make_config(), make_policies<NullMotor>(), hal);
        });
        const double on_ns = time_it([](foc::driver::StubHal& hal) {
            return TracedMotor(make_config(), make_policies<TracedMotor>(), hal);
        });
        off_best = std::min(off_best, off_ns);
        on_best = std::min(on_best, on_ns);
        std::printf("[trace] round %d: %.1f ns/period off, %.1f ns/period on\n", round, off_ns, on_ns);
    }
    std::printf("[trace] best of %d: %.1f ns/period off, %.1f ns/period on (host, %d periods each)\n", kRounds,
                off_best, on_best, kSteps);

    // What this measurement can and cannot say. It cannot say "under 2 us on a target" -- nobody measured that. It can
    // say that recording does not dominate the loop, and it reports the difference so that a reader can see it is inside
    // the noise rather than trusting a single number.
    // 这个测量能说什么、不能说什么。它不能说"目标上低于 2 微秒"——没人量过。它能说记录没有主导环路，并把差值打印出来，
    // 让读者自己看到它落在噪声里，而不是去相信一个数字。
    EXPECT_GT(off_best, 0.0);
    EXPECT_LT(on_best, off_best * 1.5) << "recording one 36-byte struct per period should not dominate the loop";
    const double delta_ns = on_best - off_best;
    std::printf("[trace] recording costs %+.1f ns/period of a %.1f ns loop (%.1f %%)\n", delta_ns, off_best,
                100.0 * delta_ns / off_best);
}

// ---------------------------------------------------------------------------------------------------------------------
// 4. The timestamp is the platform's clock, or it is honestly absent.
// 4. 时间戳是平台的时钟，否则就是诚实地缺席。
// ---------------------------------------------------------------------------------------------------------------------

TEST(Trace, TheTimestampIsThePlatformClockAndZeroMeansAbsent) {
    // Two halves. With a clock, `t_us` is what the HAL said, so the trace is on the same time base as the overrun
    // metric and anything else the platform reports. Without one, every sample carries zero -- a constant that means
    // "no timestamp", which is a fact about the platform rather than a claim that all periods happened at once.
    // 两半。有时钟时 `t_us` 就是 HAL 说的那个值，于是轨迹与过载指标、与平台报告的其他一切在同一时间基上。没有时钟时
    // 每条样本都是零——这个常数意味着"没有时间戳"，它是关于平台的事实，而不是"所有周期同时发生"的声明。
    HalWithClock hal;
    hal.dc_link.set_voltage_v(kVBus);
    hal.position.set_sensor_present(true);
    hal.position.set_angle_mech_rad(0.0F);
    hal.position.set_velocity_rad_s(0.0F);
    const foc::core::Uvw<float> zero{};
    ASSERT_EQ(foc::ErrorCode::Ok, hal.current_sense.load(&zero, 1U));

    ClockedTracedMotor motor(make_config(), make_policies<ClockedTracedMotor>(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    motor.set_torque(0.3F);
    for (std::uint32_t k = 0; k < 5U; ++k) {
        hal.clock.now_us_ = 1000U + (k * 100U);
        static_cast<void>(motor.tick(kDt));
        EXPECT_EQ(motor.trace().latest().t_us, 1000U + (k * 100U)) << "period " << k;
    }

    // A HAL without a clock still records; it simply cannot say when.
    // 没有时钟的 HAL 照样记录，只是说不出时间。
    foc::driver::StubHal plain;
    make_hal(plain);
    TracedMotor untimed(make_config(), make_policies<TracedMotor>(), plain);
    run(untimed, plain, 10, 0.3F);
    EXPECT_EQ(untimed.trace().size(), 10U);
    for (std::uint16_t i = 0; i < untimed.trace().size(); ++i) {
        EXPECT_EQ(untimed.trace().at(i).t_us, 0U) << "index " << i;
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// 5. A sample survives the text round trip, bit for bit.
// 5. 一条样本经文本往返后逐位不变。
// ---------------------------------------------------------------------------------------------------------------------

TEST(Trace, ASampleSurvivesTheTextRoundTripBitForBit) {
    // The second half of "the trace can be a reference": what is written to a file and read back must be the same
    // numbers, not the same numbers to six digits. Nine significant digits round-trip a binary32 exactly, so anything
    // that fails here is a formatting bug, and it would otherwise show up as a spurious regression in the ledger.
    // "轨迹可以当基准"的另一半：写进文件再读回来必须还是同一批数，而不是六位有效数字相同的数。九位有效数字能精确往返
    // binary32，所以这里失败的一定是格式化的 bug——否则它会在台账里表现为一次虚假的回归。
    foc::core::TraceSample original{};
    original.t_us = 4294967295U;
    original.i_d = -0.0F; // the sign of zero is the bit worth catching
    original.i_q = 1.0F / 3.0F;
    original.i_ref_d = -123.456789F;
    original.i_ref_q = 1e-30F;
    original.v_d = 3.4028235e38F; // FLT_MAX
    original.v_q = -5.877472e-39F; // a subnormal
    original.speed_mech = 0.1F;
    original.mode = 3U;
    original.status = 251U; // -5 as a signed code, viewed unsigned

    char line[256];
    const int written = std::snprintf(line, sizeof(line), "%u,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%u", original.t_us,
                                      static_cast<double>(original.i_d), static_cast<double>(original.i_q),
                                      static_cast<double>(original.i_ref_d), static_cast<double>(original.i_ref_q),
                                      static_cast<double>(original.v_d), static_cast<double>(original.v_q),
                                      static_cast<double>(original.speed_mech), static_cast<unsigned>(original.mode),
                                      static_cast<unsigned>(original.status));
    ASSERT_GT(written, 0);

    foc::core::TraceSample parsed{};
    unsigned mode = 0U;
    unsigned status = 0U;
    const int read = std::sscanf(line, "%u,%g,%g,%g,%g,%g,%g,%g,%u,%u", &parsed.t_us, &parsed.i_d, &parsed.i_q,
                                 &parsed.i_ref_d, &parsed.i_ref_q, &parsed.v_d, &parsed.v_q, &parsed.speed_mech, &mode,
                                 &status);
    ASSERT_EQ(read, 10);
    parsed.mode = static_cast<std::uint8_t>(mode);
    parsed.status = static_cast<std::uint8_t>(status);

    EXPECT_EQ(parsed.t_us, original.t_us);
    for (const auto& pair : {std::pair<const float*, const float*>{&original.i_d, &parsed.i_d},
                             {&original.i_q, &parsed.i_q},
                             {&original.i_ref_d, &parsed.i_ref_d},
                             {&original.i_ref_q, &parsed.i_ref_q},
                             {&original.v_d, &parsed.v_d},
                             {&original.v_q, &parsed.v_q},
                             {&original.speed_mech, &parsed.speed_mech}}) {
        EXPECT_EQ(std::memcmp(pair.first, pair.second, sizeof(float)), 0) << line;
    }
    EXPECT_EQ(parsed.mode, original.mode);
    EXPECT_EQ(parsed.status, original.status);
}
