// S11 - per-move limits. The same joint, the same distance, two different acceleration allowances
// carried by the move itself: the tighter one has to be visibly gentler and still stop where it was
// told. This is the scenario that proves "limits belong to the move, not only to the mechanism".
// S11 - 按次限制。同一个关节、同样的距离，两种由运动本身携带的加速度许可：更紧的那次必须明显更
// 温和，同时仍然停在被告知的位置。这个场景证明"限制属于运动，而不只属于机构"。

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/motion_profile.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_bench.hpp"
#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"



namespace {

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 4;
constexpr float kVBus = 24.0F;
constexpr float kDistanceRad = 0.30F;
constexpr float kMaxVelocity = 5.0F;  // the mechanism is allowed this much
constexpr float kMaxAccel = 50.0F;    // and this much
constexpr float kGain = 20.0F;

using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                 foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                 foc::core::MotionProfile>;

struct MoveResult final {
    float peak_speed_rad_s;
    float final_error_rad;
    float overshoot_rad;
    float travelled_rad;
};

auto nominal_config() -> foc::motor::MotorConfig { return foc::sil::nominal_motor_config(); }

/// Run one move with the given per-move limits and report what the rotor did.
auto run_move(float accel_limit_rad_s2, float duration_s) -> MoveResult {
    const foc::motor::MotorConfig mc = nominal_config();
    foc::sil::PmsmPlant plant(foc::sil::plant_params_from(foc::sil::nominal_spec()));
    foc::sil::PlantHal hal(plant, kVBus);

    const typename MotorT::Policies policies{
        foc::core::BackEmfObserver({mc.rs, mc.ld}),
        foc::core::Svpwm{},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
        foc::core::NullReferenceGenerator{},
        foc::core::NullFeedforward{},
        foc::core::MotionProfile{foc::core::MotionProfile::Config{kGain, kMaxVelocity, kMaxAccel}}};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    static_cast<void>(motor.start());

    // The move carries its own ceiling; the mechanism's configuration stays as it was.
    // 这次运动自带上限；机构配置保持原样。
    motor.set_command(foc::motor::Command{kDistanceRad, 0.0F, 0.0F, 0.0F, accel_limit_rad_s2, 0.0F, 0.0F, 0.0F});

    MoveResult result{0.0F, 0.0F, 0.0F, 0.0F};
    const int steps = static_cast<int>((duration_s / kDt) + 0.5F);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        result.peak_speed_rad_s = std::max(result.peak_speed_rad_s, std::fabs(motor.telemetry().speed_mech_rad_s));
    }

    result.travelled_rad = hal.position.true_angle_mech_rad();
    result.final_error_rad = kDistanceRad - result.travelled_rad;
    result.overshoot_rad = std::max(0.0F, result.travelled_rad - kDistanceRad);
    return result;
}

} // namespace



TEST(SilPerMoveLimits, ATighterPerMoveAccelLimitIsVisiblyGentlerAndStillArrives) {
    // The same distance, the same gain, the same mechanism: only the move's acceleration allowance
    // differs. The gentler move peaks at half the speed -- which is what "half this value" means
    // physically -- and still lands on target without overshooting.
    // 同样的距离、同样的增益、同样的机构：只有运动的加速度许可不同。更温和的那次峰值速度约为一半
    // ——这正是"这个值减半"的物理含义——并且照样到位、不冲过头。
    const MoveResult brisk = run_move(kMaxAccel, 1.0F);
    const MoveResult gentle = run_move(5.0F, 2.0F);

    EXPECT_GT(brisk.peak_speed_rad_s, 0.5F) << "the brisk move has to actually move";
    // For an accel-limited move of distance d at acceleration a, the peak speed is sqrt(a*d).
    // 加速度受限、距离为 d、加速度为 a 的运动，峰值速度是 sqrt(a*d)。
    EXPECT_NEAR(std::sqrt(kMaxAccel * kDistanceRad), brisk.peak_speed_rad_s, 0.25F * brisk.peak_speed_rad_s);
    EXPECT_NEAR(std::sqrt(5.0F * kDistanceRad), gentle.peak_speed_rad_s, 0.25F * gentle.peak_speed_rad_s);

    EXPECT_LT(gentle.peak_speed_rad_s, brisk.peak_speed_rad_s) << "the tighter limit must be slower";
    EXPECT_NEAR(0.0F, gentle.final_error_rad, 5e-3F) << "and it must still arrive";
    EXPECT_LT(gentle.overshoot_rad, 5e-3F) << "with no overshoot: the braking curve knew the limit";
    EXPECT_NEAR(0.0F, brisk.final_error_rad, 5e-3F);
}



TEST(SilPerMoveLimits, AZeroPerMoveLimitFallsBackToTheMechanismConfiguration) {
    // 0 means "no override". The move is then shaped by `MotionProfile::Config`, so its peak speed is
    // the one the *mechanism* allows, not the gentle one.
    // 0 表示"不覆盖"。此时运动由 `MotionProfile::Config` 塑造，于是峰值速度是**机构**允许的那个，
    // 而不是温和的那个。
    const MoveResult configured = run_move(0.0F, 1.0F);
    EXPECT_GT(configured.peak_speed_rad_s, 1.0F);
    EXPECT_NEAR(0.0F, configured.final_error_rad, 5e-3F);
}



TEST(SilPerMoveLimits, APerMoveVelocityCeilingHolds) {
    // A long move with a low velocity ceiling: the rotor must never exceed it, which is observable
    // because the ceiling is well below what the mechanism would otherwise allow.
    // 长距离运动配上低速度上限：转子绝不得超过它，这可以观测，因为该上限远低于机构本来的许可。
    const foc::motor::MotorConfig mc = nominal_config();
    foc::sil::PmsmPlant plant(foc::sil::plant_params_from(foc::sil::nominal_spec()));
    foc::sil::PlantHal hal(plant, kVBus);

    const typename MotorT::Policies policies{
        foc::core::BackEmfObserver({mc.rs, mc.ld}),
        foc::core::Svpwm{},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
        foc::core::NullReferenceGenerator{},
        foc::core::NullFeedforward{},
        foc::core::MotionProfile{foc::core::MotionProfile::Config{kGain, kMaxVelocity, kMaxAccel}}};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    static_cast<void>(motor.start());

    constexpr float kCeiling = 0.25F;
    motor.set_command(foc::motor::Command{2.0F, 0.0F, 0.0F, kCeiling, 0.0F, 0.0F, 0.0F, 0.0F});

    float peak = 0.0F;
    const int steps = static_cast<int>(1.0F / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        peak = std::max(peak, std::fabs(motor.telemetry().speed_mech_rad_s));
    }

    // Allow the inner loop's own tracking error, not a policy of exceeding the ceiling.
    // 允许内层环的跟踪误差，而不是允许超上限这件事本身。
    EXPECT_LT(peak, kCeiling * 1.1F) << "the per-move ceiling must hold on the plant";
}
