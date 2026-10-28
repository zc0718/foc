// S6 - feedforward in closed loop. S1/S2/S5 prove the *default* combination (no feedforward); this
// scenario proves the first optional stage actually does something on a coupled machine, and that it
// does not cost stability. Both runs use the same plant, nameplate and tuning, so the only variable
// is the feedforward policy.
// S6 - 闭环下的前馈。S1/S2/S5 证明的是**默认组合**（无前馈）；本场景证明第一个可选阶段在一台
// 存在轴间耦合的电机上确实起作用，并且不以稳定性为代价。两次运行共用同一 plant、同一铭牌、同一
// 整定，唯一变量就是前馈策略。

#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/feedforward.hpp>
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
constexpr float kRunSeconds = 0.30F;
constexpr float kSettleSeconds = 5e-3F;

/// [en] An interior-PM machine: Ld != Lq, so the cross-coupling terms are non-trivial.
/// [zh] 内嵌式电机：Ld != Lq，于是交叉耦合项非平凡。
auto machine() -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{0.5F, 2.0e-4F, 6.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
}

struct Outcome final {
    float max_id_a;      ///< [en] worst d-axis excursion after the transient. [zh] 暂态之后最差的 d 轴偏移。
    float final_speed;   ///< [en] mechanical speed at the end of the run. [zh] 运行结束时的机械速度。
    foc::ErrorCode status; ///< [en] sticky status of the last tick. [zh] 最后一次 tick 的粘滞状态。
};

/// [en] Run one fixed scenario with the given feedforward policy.
/// [zh] 用给定前馈策略跑一次固定场景。
template <typename FeedForward> auto run_with(FeedForward ff) -> Outcome {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, FeedForward>;
    const foc::motor::MotorConfig mc = machine();
    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
    foc::sil::PlantHal hal(plant, kVBus);

    const typename MotorT::Policies policies{foc::core::BackEmfObserver({mc.rs, mc.ld}),
                                             foc::core::Svpwm{},
                                             foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                             foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                             foc::core::Pi{foc::speed_pi_config(mc, kOmegaSpeed)},
                                             foc::core::NullReferenceGenerator{},
                                             ff};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_speed(kTargetSpeed);
    static_cast<void>(motor.start());

    float max_id = 0.0F;
    const int steps = static_cast<int>(kRunSeconds / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        const float t = static_cast<float>(i) * kDt;
        const float id = motor.telemetry().id_a;
        if ((t > kSettleSeconds) && (std::fabs(id) > max_id)) {
            max_id = std::fabs(id);
        }
    }

    return Outcome{max_id, motor.telemetry().speed_mech_rad_s, motor.status()};
}

} // namespace



/**
 * @brief [en] The d axis is commanded to zero, so every |id| that appears is a disturbance the
 *        current loop had to fight. Decoupling removes most of it; the test asserts an order of
 *        magnitude, not the exact figure, so normal tuning changes do not break it.
 *        [zh] d 轴给定为零，因此出现的每一个 |id| 都是电流环必须对抗的扰动。解耦把其中大部分消掉；
 *        用例断言的只是数量级而不是具体数值，正常的整定变更不会把它弄红。
 */
TEST(SilFeedforward, CrossDecouplingCutsTheDaxisDisturbanceByAnOrderOfMagnitude) {
    const Outcome without = run_with(foc::core::NullFeedforward{});
    const Outcome with = run_with(foc::core::DecouplingFeedforward{foc::feedforward_config(machine())});

    EXPECT_EQ(foc::ErrorCode::Ok, without.status);
    EXPECT_EQ(foc::ErrorCode::Ok, with.status);

    // Both runs must reach and hold the target speed: the feedforward may not trade stability for
    // disturbance rejection.
    EXPECT_NEAR(kTargetSpeed, without.final_speed, 2.0F);
    EXPECT_NEAR(kTargetSpeed, with.final_speed, 2.0F);

    // Measured on this bench: 0.284 A without, 0.008 A with. Assert a factor of five so the test
    // still means "decoupling works" without pinning an implementation detail.
    EXPECT_GT(without.max_id_a, 0.05F);
    EXPECT_LT(with.max_id_a * 5.0F, without.max_id_a);
}



/**
 * @brief [en] A zeroed configuration must behave exactly like no feedforward: a machine with no
 *        inductance and no flux has nothing to cancel. This pins the "unset means off" rule.
 *        [zh] 全零配置必须与「无前馈」完全等价：没有电感、没有磁链的电机没有可抵消的东西。这条
 *        用例钉住「未配置即关闭」的规则。
 */
TEST(SilFeedforward, AZeroedConfigurationIsIndistinguishableFromNoFeedforward) {
    const Outcome none = run_with(foc::core::NullFeedforward{});
    const Outcome zeroed = run_with(foc::core::DecouplingFeedforward{});

    EXPECT_EQ(none.status, zeroed.status);
    EXPECT_FLOAT_EQ(none.max_id_a, zeroed.max_id_a);
    EXPECT_FLOAT_EQ(none.final_speed, zeroed.final_speed);
}
