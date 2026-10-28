// S7 - multi-rate. The speed loop is decimated by elapsed time, not by a tick count, so a 10x or
// 20x slower outer loop must still meet the speed-step acceptance thresholds. This is the closed-loop
// counterpart of Motor.SpeedLoopDecimationKeepsTheIntegralOverTheSameElapsedTime: that one proves the
// integral is preserved, this one proves the loop still behaves.
// S7 - 多速率。速度环按累计时间降采样，而不是按 tick 计数，因此外环慢 10 倍或 20 倍时仍须满足 S2
// 的验收阈值。这是 Motor.SpeedLoopDecimationKeepsTheIntegralOverTheSameElapsedTime 的闭环对照：
// 那条证明积分守恒，这条证明环路行为不变。

#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"



namespace {

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 4;
constexpr int kPolePairs = 7;
constexpr float kVBus = 24.0F;
constexpr float kOmegaCurrent = 2000.0F;
constexpr float kOmegaSpeed = 250.0F;
constexpr float kTargetSpeed = 100.0F;
constexpr float kRunSeconds = 0.40F;
constexpr float kSettleBand = 0.02F;
constexpr float kSettleFloorS = 0.05F;

struct Outcome final {
    float overshoot_pct;
    float settle_s;
    float steady_err_pct;
    foc::ErrorCode status;
};

/**
 * @brief [en] One speed step with the outer loop running every `speed_loop_period_s` seconds (0 = every tick).
 * @brief [zh] 以 `speed_loop_period_s` 秒为周期（0 = 每 tick）跑一次速度阶跃。
 */
auto step_speed(float speed_loop_period_s) -> Outcome {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal>;
    foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
    mc.speed_loop_period_s = speed_loop_period_s;

    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
    foc::sil::PlantHal hal(plant, kVBus);
    const typename MotorT::Policies policies{foc::core::BackEmfObserver({mc.rs, mc.ld}),
                                             foc::core::Svpwm{},
                                             foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                             foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                             foc::core::Pi{foc::speed_pi_config(mc, kOmegaSpeed)}};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_speed(kTargetSpeed);
    static_cast<void>(motor.start());

    float peak = 0.0F;
    float settle = 0.0F;
    const int steps = static_cast<int>(kRunSeconds / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        const float t = static_cast<float>(i + 1) * kDt;
        const float w = motor.telemetry().speed_mech_rad_s;
        if (w > peak) {
            peak = w;
        }
        if ((t > kSettleFloorS) && (std::fabs(w - kTargetSpeed) > (kSettleBand * kTargetSpeed))) {
            settle = t; // last moment it was outside the band
        }
    }

    const float final_speed = motor.telemetry().speed_mech_rad_s;
    return Outcome{100.0F * (peak - kTargetSpeed) / kTargetSpeed, settle,
                   100.0F * std::fabs(final_speed - kTargetSpeed) / kTargetSpeed, motor.status()};
}

} // namespace



/**
 * @brief [en] A decimated outer loop must keep the speed-step acceptance thresholds, and must not change the
 *             closed-loop response by more than a fixed factor. The speed-loop bandwidth here is 250 rad/s (a 4 ms time
 *             constant), so 500 us and even 1 ms remain oversampled -- that is exactly why the decimation is allowed to
 *             be this coarse, and the test pins that reasoning.
 * @brief [zh] 降采样的外环必须保持 S2 的验收阈值，且不能把闭环响应改变超过一个固定倍数。这里的速度环 带宽是 250 rad/s（
 *             时间常数 4 ms），因此 500 us 甚至 1 ms 仍然是过采样——这正是降采样可以 这么粗的理由，用例把这条推理钉住。
 */
TEST(SilMultirate, DecimatedSpeedLoopKeepsTheS2Thresholds) {
    const Outcome every = step_speed(0.0F);
    const Outcome dec10 = step_speed(10.0F * kDt);
    const Outcome dec20 = step_speed(20.0F * kDt);

    for (const Outcome& outcome : {every, dec10, dec20}) {
        EXPECT_EQ(foc::ErrorCode::Ok, outcome.status);
        EXPECT_LE(outcome.overshoot_pct, 10.0F);  // S2: overshoot <= 10%
        EXPECT_LE(outcome.steady_err_pct, 1.0F);  // S2: steady-state error <= 1%
        EXPECT_GT(outcome.settle_s, 0.0F);        // it did leave the band and come back
    }

    // The response may shift slightly, but not by an order of magnitude.
    EXPECT_NEAR(every.settle_s, dec10.settle_s, 0.5F * every.settle_s);
    EXPECT_NEAR(every.settle_s, dec20.settle_s, 0.5F * every.settle_s);
    EXPECT_NEAR(every.overshoot_pct, dec20.overshoot_pct, 1.0F);
}
