// S9 - accel-limited position motion. S8 uses a bare P position loop, whose velocity command jumps
// with the error; this scenario puts `core::MotionProfile` in the same slot and checks the invariants
// that matter on a joint: the command never exceeds the velocity limit, never changes faster than the
// acceleration limit, and still arrives.
// S9 - 加速度受限的位置运动。S8 用的是纯 P 位置环，其速度指令随误差跳变；本场景把
// `core::MotionProfile` 放进同一个槽位，检查关节真正在意的不变式：指令不越速度上限、变化不快于
// 加速度上限，而且照样到位。

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/motion_profile.hpp>
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
constexpr float kVelocityLimit = 2.0F;
constexpr float kAccelLimit = 10.0F;

struct Trace final {
    float worst_velocity;
    float worst_accel;      ///< [en] largest per-step velocity change divided by dt. [zh] 单步速度变化除以 dt 的最大值。
    float final_err_rad;
    float overshoot_rad;
    foc::ErrorCode status;
};

} // namespace



/**
 * @brief [en] The profile shapes the velocity command; the plant must show the same shape. The
 *        measured acceleration is bounded by the configured limit (the inner loop tracks the command
 *        much faster than the profile changes it), which is the whole point of putting the profile
 *        in front of the loop instead of inside it.
 *        [zh] 规划器塑造的是速度指令；plant 上量到的也必须是同样的形状。实测加速度受配置上限约束
 *        （内层环跟踪指令远比规划器改变指令快得多）——这正是把规划器放在环**前面**而不是里面的意义。
 */
TEST(SilS9Profile, AccelLimitedMoveRespectsItsLimitsAndStillArrives) {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                     foc::core::MotionProfile>;
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
        foc::core::MotionProfile{foc::core::MotionProfile::Config{20.0F, kVelocityLimit, kAccelLimit}}};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_position(kTargetRad);
    static_cast<void>(motor.start());

    Trace trace{0.0F, 0.0F, 0.0F, 0.0F, foc::ErrorCode::Ok};
    float previous_speed = 0.0F;
    float peak = 0.0F;
    const int steps = static_cast<int>(1.0F / kDt);
    for (int i = 0; i < steps; ++i) {
        trace.status = motor.tick(kDt);
        hal.advance(kDt, kSubsteps);

        const float speed = motor.telemetry().speed_mech_rad_s;
        const float position = hal.position.angle_mech_rad();
        trace.worst_velocity = std::max(trace.worst_velocity, std::fabs(speed));
        trace.worst_accel = std::max(trace.worst_accel, std::fabs(speed - previous_speed) / kDt);
        peak = std::max(peak, position);
        previous_speed = speed;
    }

    trace.overshoot_rad = peak - kTargetRad;
    trace.final_err_rad = std::fabs(hal.position.angle_mech_rad() - kTargetRad);

    EXPECT_EQ(foc::ErrorCode::Ok, trace.status);

    // The velocity limit binds but is not exceeded (the inner loop regulates it, so allow a little).
    EXPECT_LE(trace.worst_velocity, 1.05F * kVelocityLimit);
    EXPECT_GE(trace.worst_velocity, 0.90F * kVelocityLimit); // ...and it really did reach the limit

    // The acceleration limit holds on the plant, not only inside the profile.
    EXPECT_LE(trace.worst_accel, 1.5F * kAccelLimit);

    EXPECT_LE(trace.overshoot_rad, 0.02F);
    EXPECT_LE(trace.final_err_rad, 0.01F);
}



/**
 * @brief [en] The same move without the profile: the bare P loop turns the step into a velocity step,
 *        which is exactly the acceleration the profile exists to remove. This is the negative control
 *        that proves the test above is measuring something real.
 *        [zh] 同一次运动去掉规划器：纯 P 环把位置阶跃变成速度阶跃，而这正是规划器要消掉的加速度。
 *        这条反向对照证明上面那条用例量的是真东西。
 */
TEST(SilS9Profile, WithoutTheProfileTheSameMoveIsAVelocityStep) {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, foc::core::NullFeedforward, foc::core::PositionP>;
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
        // No acceleration limit: a bare P loop, which is the point of this control.
        // 不给加速度限制：纯 P 环，这才是这条对照的意义。
        foc::core::PositionP{foc::position_p_config(mc, 20.0F, kVelocityLimit, 0.0F)}};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_position(kTargetRad);
    static_cast<void>(motor.start());

    float worst_accel = 0.0F;
    float previous_speed = 0.0F;
    const int steps = static_cast<int>(1.0F / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        const float speed = motor.telemetry().speed_mech_rad_s;
        worst_accel = std::max(worst_accel, std::fabs(speed - previous_speed) / kDt);
        previous_speed = speed;
    }

    EXPECT_GT(worst_accel, 3.0F * kAccelLimit);
}
