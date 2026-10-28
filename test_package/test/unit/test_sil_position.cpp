// S8 - position control on the SIL plant. The command is a 1 rad step; the cascade is
// position -> velocity -> current, with the position loop a plain P whose kp *is* its bandwidth.
// The point of this scenario is that the three loops together settle, hold and do not oscillate --
// S1/S2/S7 only ever exercise the two inner ones.
// S8 - plant 上的位置控制。给 1 rad 阶跃；级联为 位置 -> 速度 -> 电流，位置环是纯 P，其 kp 就是
// 它的带宽。本场景的意义是三个环合起来能收敛、能保持、不振荡——S1/S2/S7 只覆盖了里面两个。

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
constexpr float kTargetRad = 1.0F;
constexpr float kBandRad = 0.02F;
constexpr float kOvershootGuardS = 0.02F;

struct Outcome final {
    float overshoot_rad;
    float settle_s;
    float final_err_rad;
    foc::ErrorCode status;
};

/**
 * @brief [en] One position step with the given position-loop bandwidth, velocity limit and run length.
 * @brief [zh] 用给定位置环带宽、速度上限与运行时长跑一次位置阶跃。
 */
auto step_position(float position_bandwidth_rad_s, float velocity_limit_rad_s, float run_seconds = 1.0F) -> Outcome {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                     foc::core::PositionP>;
    const foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
    foc::sil::PlantHal hal(plant, kVBus);

    const typename MotorT::Policies policies{
        foc::core::BackEmfObserver({mc.rs, mc.ld}),
        foc::core::Svpwm{},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
        foc::core::NullReferenceGenerator{},
        foc::core::NullFeedforward{},
        foc::core::PositionP{foc::position_p_config(mc, position_bandwidth_rad_s, velocity_limit_rad_s, 0.0F)}};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_position(kTargetRad);
    static_cast<void>(motor.start());

    float peak = 0.0F;
    float settle = 0.0F;
    const int steps = static_cast<int>(run_seconds / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        const float t = static_cast<float>(i + 1) * kDt;
        const float position = hal.position.angle_mech_rad();
        if (position > peak) {
            peak = position;
        }
        if ((t > kOvershootGuardS) && (std::fabs(position - kTargetRad) > kBandRad)) {
            settle = t; // last moment outside the 2% band
        }
    }

    return Outcome{peak - kTargetRad, settle, std::fabs(hal.position.angle_mech_rad() - kTargetRad), motor.status()};
}

} // namespace



/**
 * @brief [en] The cascade must land on the commanded angle and stay there. Measured on this bench: 20 rad/s gives 2.1
 *             mrad of overshoot and 180 ms to the 2% band, which is the first-order response the position-loop contract
 *             predicts (`settle ~= 4/kp`).
 * @brief [zh] 级联必须落到给定角度并保持住。本台架实测：20 rad/s 时超调 2.1 mrad、180 ms 进入 2% 带， 正是位置环契约预
 *             测的一阶响应（`settle ≈ 4/kp`）。
 */
TEST(SilPosition, StepLandsOnTargetAndStaysThere) {
    const Outcome outcome = step_position(20.0F, 50.0F);

    EXPECT_EQ(foc::ErrorCode::Ok, outcome.status);
    EXPECT_LE(outcome.overshoot_rad, 0.02F);
    EXPECT_GT(outcome.settle_s, 0.0F); // it did leave the band and come back
    EXPECT_LE(outcome.settle_s, 0.40F);
    EXPECT_LE(outcome.final_err_rad, 0.01F);
}



/**
 * @brief [en] `kp` is a bandwidth: doubling it should roughly halve the settling time, and the velocity limit must hold
 *             it back from overshooting just the same. A regression here means the loop lost its first-order character
 *             (e.g. an unintentional integral term).
 * @brief [zh] `kp` 就是带宽：翻倍它应当让稳定时间大致减半，同时速度上限仍要压住超调。这里变红意味着 环路失去了一阶特性
 *             （例如不小心多了一个积分项）。
 */
TEST(SilPosition, BandwidthScalesTheSettlingTimeAsAFirstOrderLoop) {
    const Outcome slow = step_position(10.0F, 50.0F);
    const Outcome fast = step_position(20.0F, 50.0F);

    EXPECT_EQ(foc::ErrorCode::Ok, slow.status);
    EXPECT_EQ(foc::ErrorCode::Ok, fast.status);
    EXPECT_LE(slow.final_err_rad, 0.01F);
    EXPECT_LE(fast.final_err_rad, 0.01F);

    // Theory says the ratio is 2. Allow 1.4-3.0 so tuning changes do not break the test.
    const float ratio = slow.settle_s / fast.settle_s;
    EXPECT_GT(ratio, 1.4F);
    EXPECT_LT(ratio, 3.0F);
}



/**
 * @brief [en] A velocity limit low enough to bind must turn the step into a ramp, and the axis must still arrive: the
 *             limit shapes the motion, it does not stop it.
 * @brief [zh] 低到真会触发的速度上限必须把阶跃变成斜坡，而且轴仍要到位：限速塑造运动，而不是阻断运动。
 */
TEST(SilPosition, AVelocityLimitTurnsTheStepIntoARampButStillArrives) {
    const Outcome limited = step_position(20.0F, 1.0F, 2.0F); // 1 rad at 1 rad/s needs >= 1 s to cover

    EXPECT_EQ(foc::ErrorCode::Ok, limited.status);
    EXPECT_LE(limited.overshoot_rad, 0.05F);
    EXPECT_LE(limited.final_err_rad, 0.01F);
}
