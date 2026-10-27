// S13 - inverter dead time. This scenario exists for one reason: the PRD's U1 said "dead-time
// distortion cannot be verified in SIL", and that was a statement about the *plant*. With a
// dead-time model the distortion becomes a number CI can print, which is what turns the known gap
// into a measured one.
// S13 - 逆变器死区。本场景只为一件事存在：PRD 的 U1 说"死区畸变在 SIL 里无法验证"，而那是一句关于
// **被控对象**的话。加上死区模型之后，畸变变成 CI 能打印出来的数字——这才把"已知缺口"变成"已测缺口"。

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"



namespace {

constexpr float kTwoPi = 6.28318530718F;
constexpr float kSineHz = 1.0F;
constexpr float kAmplitudeRadS = 2.0F; // the amplitude where the distortion is largest, see below
constexpr float kRunSeconds = 2.0F;
constexpr float kSettleSeconds = 0.25F;
constexpr float kPwmHz = 20000.0F;
constexpr float kDeadtimeS = 1.0e-6F;

/// The voltage a dead time of `kDeadtimeS` injects per phase at the given bus voltage:
/// `(t_dead / T_pwm) * v_bus`, i.e. the missing volt-seconds spread over the switching period.
/// 给定母线电压下，`kDeadtimeS` 死区每相注入的电压：`(t_dead / T_pwm) * v_bus`。
auto deadtime_voltage(float v_bus) -> float { return (kDeadtimeS * kPwmHz) * v_bus; }

struct SineResult final {
    float worst_error_rad_s;
    float measured_amplitude_rad_s;
};

/// S3's 1 Hz tracking scenario at a low amplitude, with the dead time switched on or off.
/// S3 的 1 Hz 跟踪场景（低幅值），可开关死区模型。
auto run_low_amplitude_sine(float v_dead) -> SineResult {
    foc::sil::Bench bench(foc::sil::standard_bench_config());
    bench.hal().set_deadtime_voltage(v_dead);
    bench.motor().set_speed(0.0F);
    static_cast<void>(bench.init_and_start());
    bench.run(2e-3F);

    const int steps = static_cast<int>((kRunSeconds / bench.config().dt_ctrl_s) + 0.5F);
    const int skip = static_cast<int>((kSettleSeconds / bench.config().dt_ctrl_s) + 0.5F);

    std::vector<foc::sil::Sample> trace;
    trace.reserve(static_cast<std::size_t>(steps - skip));

    SineResult result{0.0F, 0.0F};
    for (int i = 0; i < steps; ++i) {
        const float t = bench.elapsed_s();
        const float ref = kAmplitudeRadS * std::sin(kTwoPi * kSineHz * t);
        bench.motor().set_speed(ref);
        bench.tick_once();
        if (i >= skip) {
            trace.push_back(bench.sample());
            const float error = std::fabs(trace.back().speed - ref);
            result.worst_error_rad_s = (error > result.worst_error_rad_s) ? error : result.worst_error_rad_s;
        }
    }

    const std::vector<float> speed = foc::sil::series_of(trace, &foc::sil::Sample::speed);
    result.measured_amplitude_rad_s = 0.5F * (foc::sil::max_of(speed) - foc::sil::min_of(speed));
    return result;
}

} // namespace



TEST(SilS13Deadtime, TheModelInjectsTheVoltSecondsTheDeadTimeTakes) {
    // 1 us at 20 kHz is 2 % of the switching period; at a 24 V bus that is 0.48 V per phase, which is
    // the quantity every dead-time compensation scheme has to reconstruct.
    // 20 kHz 下 1 us 占开关周期的 2%；24 V 母线上就是每相 0.48 V——这正是所有死区补偿方案必须重建的量。
    EXPECT_NEAR(0.48F, deadtime_voltage(24.0F), 1e-3F);

    foc::sil::PmsmPlant plant(foc::sil::nominal_plant_params());
    foc::sil::PlantHal hal(plant, 24.0F);
    EXPECT_FLOAT_EQ(0.0F, hal.deadtime_voltage());
    hal.set_deadtime_voltage(deadtime_voltage(24.0F));
    EXPECT_NEAR(0.48F, hal.deadtime_voltage(), 1e-5F);
    // A negative setting is not "reverse dead time": it clamps to off.
    // 负的设定不是"反向死区"：它会被钳到关闭。
    hal.set_deadtime_voltage(-1.0F);
    EXPECT_FLOAT_EQ(0.0F, hal.deadtime_voltage());
}



TEST(SilS13Deadtime, LowCurrentTrackingDegradesOnceDeadTimeIsModelled) {
    // The distortion is *not* monotonic in amplitude, and the sweep below is why this test
    // uses 2 rad/s rather than "as slow as possible" -- measured worst-error ratio with 1 us
    // of dead time:
    //   1 rad/s -> x1.00, 2 rad/s -> x1.48, 5 rad/s -> x1.13, 10 rad/s -> x1.03
    // At large amplitudes the resistive and back-EMF voltages dwarf the 0.48 V the dead time
    // removes; at the very lowest amplitudes this model's `sign(0) = 0` makes the
    // zero-crossing neighbourhood *better* than hardware -- exactly the idealisation U1 warns
    // about.
    // 畸变对幅值**不是单调的**，所以本用例取 2 rad/s 而不是"越慢越好"——1 us 死区下实测最差
    // 误差比值：1 rad/s → x1.00，2 rad/s → x1.48，5 rad/s → x1.13，10 rad/s → x1.03。
    // 幅值大时电阻压降与反电动势远大于死区拿走的 0.48 V；幅值极低时本模型的 `sign(0) = 0`
    // 让零电流附近比真实硬件**更好**——这正是 U1 所警告的那层理想化。
    const SineResult ideal = run_low_amplitude_sine(0.0F);
    const SineResult with_deadtime = run_low_amplitude_sine(deadtime_voltage(24.0F));

    // The ideal case tracks tightly -- this is the control the library already promises.
    // 理想情况跟踪得很紧——这是库本来就承诺的控制质量。
    EXPECT_LE(ideal.worst_error_rad_s, 0.05F * kAmplitudeRadS) << "ideal worst error " << ideal.worst_error_rad_s;

    // The distortion is real and measurable: the worst error grows, while the loop still tracks.
    // 畸变是真实的、可测的：最差误差变大，而环路仍然在跟踪。
    EXPECT_GT(with_deadtime.worst_error_rad_s, 1.3F * ideal.worst_error_rad_s)
        << "ideal " << ideal.worst_error_rad_s << " vs dead time " << with_deadtime.worst_error_rad_s;
    EXPECT_GE(with_deadtime.measured_amplitude_rad_s, 0.9F * kAmplitudeRadS) << "the loop must not collapse";

    // Recorded, not asserted: the ratio is the size of the gap U1 describes, and it is what a
    // dead-time compensation stage would have to remove.
    // 只记录不断言：这个比值就是 U1 所描述的缺口大小，也是死区补偿环节需要消掉的东西。
    std::printf("[S13] worst error: ideal %.5f rad/s, with 1us dead time %.5f rad/s (x%.2f)\n",
                static_cast<double>(ideal.worst_error_rad_s), static_cast<double>(with_deadtime.worst_error_rad_s),
                static_cast<double>(with_deadtime.worst_error_rad_s / ideal.worst_error_rad_s));
}
