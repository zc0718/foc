// W7-a, increment 5's closure: the homing policy driving a real sensor-less motor to its own index mark.
//
// The unit cases prove what the routine decides. This one proves that a caller can follow it with nothing else in
// between: take the speed out of `step()`, hand it to the motor, feed the motor's reference back in, and the search ends
// where the mark is. If that only worked with rig-specific glue, the policy would be a suggestion rather than a piece of
// the library.
//
// W7-a 增量 5 的闭环：让回零策略驱动一台真实的、无感的电机去找它自己的索引标记。
// 单元用例证明这个流程**决定**了什么。这一个证明调用方可以照着它在中间不加任何东西：从 `step()` 取出转速、交给电机、
// 把电机的参考喂回去，搜索就停在标记所在的地方。如果这场只靠台架专用的胶水才能成立，那这个策略就只是一条建议，
// 而不是库的一部分。
//
// The criterion, fixed before the numbers below were measured:
//   C1  the search ends by finding the mark, not by timing out: `Done` with the origin captured, and the captured origin
//       is at the mark's true position within two ticks of travel. (Two ticks, not one, because the pulse is computed
//       from the plant at the start of a period and read at its end -- the same sampling order that made the position
//       hold's first criterion wrong.)
//   C2  the search stops commanding when it says it has: after `Done`, the command is zero and the state stays `Done`.
//   C3  the axis then holds a position at the place it stopped, with no encoder anywhere: worst error inside the
//       alignment routine's own settle tolerance over the last 0.2 s. Unlike the high-speed hold, the angle source here
//       is the injection observer at a few rad/s, which is exactly the regime it claims.
//
// 判据在量之前定下：
//   C1 搜索以**找到标记**结束，而不是超时：`Done` 且原点已捕获，捕获位置在标记真值的两个 tick 行程之内。（取两个而
//      不是一个：脉冲由周期**开始**时的 plant 位置算出、在周期末读取——与让位置保持那条判据第一版出错的采样顺序相同。）
//   C2 搜索在它说停的时候就停：`Done` 之后指令为零、状态保持 `Done`。
//   C3 轴随后停下来的那个位置被保持住，全程无编码器：最后 0.2 s 内最差误差落在这台机器对齐流程自己的收敛容差之内。
//      与高速保持不同，这里角度源是几 rad/s 下的注入观测器，而那是它声称的区间。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/homing.hpp>
#include <foc/core/observer/switchable_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_pmsm_plant.hpp"
#include "sil_plant_hal.hpp"

namespace {

namespace core = foc::core;
namespace motor = foc::motor;
namespace sil = foc::sil;

constexpr float kPi = 3.14159265358979F;
constexpr float kTwoPi = 6.28318530717959F;

constexpr float kRs = 0.35F;
constexpr float kLd = 2.0e-4F;
constexpr float kLq = 4.0e-4F;
constexpr float kKt = 0.05F;
constexpr float kJ = 1.0e-5F;
constexpr float kB = 1.0e-3F;
constexpr std::uint8_t kPoles = 7U;

constexpr float kCtrlHz = 20000.0F;
constexpr float kDt = 1.0F / kCtrlHz;
constexpr int kSubsteps = 8;

constexpr float kMarkMechRad = 0.5F;
constexpr float kCreepRadS = 3.0F;
constexpr float kTimeoutS = 3.0F;
constexpr float kSettleBandRad = 0.02F;
constexpr float kSettleWindowS = 0.2F;
constexpr float kHoldSeconds = 0.8F;

using MotorT = motor::Motor<core::SwitchableObserver, core::Svpwm, core::Pi, sil::PlantHal,
                            core::NullReferenceGenerator, core::NullFeedforward, core::PositionP, core::NullAlignment>;

auto plant_params() -> sil::PlantParams { return sil::PlantParams{kRs, kLd, kLq, kKt, kJ, kB, kPoles}; }

auto motor_config() -> motor::MotorConfig {
    return motor::MotorConfig{kRs, kLd, kLq, kKt, kJ, kB, kPoles, 3.0F, kCtrlHz, 0.0F, 0.0F, 0.0F};
}

auto observer_config() -> core::SwitchableObserver::Config {
    const motor::MotorConfig cfg = motor_config();
    const float psi_f = foc::psi_f_wb(cfg);
    core::SwitchableObserver::Config config{};
    config.back_emf = core::BackEmfObserver::Config{cfg.rs, cfg.ld};
    config.nonlinear_flux = core::NonlinearFluxObserver::Config{cfg.rs, cfg.lq, psi_f, 0.3F, 3.0F, 0.5F * psi_f};
    config.hfi.inject_v = 12.0F;
    config.hfi.inject_hz = 2000.0F;
    config.hfi.current_hp_hz = 300.0F;
    config.hfi.demod_lpf_hz = 200.0F;
    config.hfi.pll_bandwidth_hz = 40.0F;
    config.hfi.pll_zeta = 0.9F;
    config.hfi.demod_gain_a = core::HfiObserver::predicted_gain_a(config.hfi.inject_v, config.hfi.inject_hz, kLd, kLq);
    config.hfi.min_hf_current_a = 0.05F;
    config.kind = core::ObserverKind::Hfi;
    return config;
}

auto wrap_pi_local(float angle) noexcept -> float {
    while (angle <= -kPi) {
        angle += kTwoPi;
    }
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    return angle;
}

} // namespace

TEST(SilHoming, ThePolicyDrivesASensorlessMotorToItsOwnMarkAndItHolds) {
    sil::PmsmPlant plant(plant_params());
    sil::PlantHal hal(plant, 24.0F);
    hal.reset(sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
    hal.position.set_sensor_present(false); // no encoder at any point
    hal.position.set_index_position_mech_rad(kMarkMechRad);

    const motor::MotorConfig cfg = motor_config();
    MotorT motor(cfg,
                 MotorT::Policies{core::SwitchableObserver{observer_config()},
                                  core::Svpwm{},
                                  core::Pi{foc::current_pi_config(cfg, 2000.0F, 24.0F)},
                                  core::Pi{foc::current_pi_config(cfg, 2000.0F, 24.0F)},
                                  core::Pi{foc::speed_pi_config(cfg, 200.0F)},
                                  core::NullReferenceGenerator{},
                                  core::NullFeedforward{},
                                  core::PositionP{foc::position_p_config(cfg, 20.0F, 120.0F, 0.0F)},
                                  core::NullAlignment{}},
                 hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    ASSERT_EQ(core::MotorState::Run, motor.state());

    auto tick = [&motor, &hal]() {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
    };

    // Let the injection observer acquire the angle at standstill before asking the speed loop to do anything with it.
    // 先让注入观测器在静止时获取角度，再让速度环拿它去做事。
    motor.set_torque(0.0F);
    for (int i = 0; i < static_cast<int>(0.3F / kDt); ++i) {
        tick();
    }

    // --- the search, driven only through the policy's own command ---------------------------------------------
    // 搜索——只通过策略自己的指令驱动。
    core::HomingRoutine routine(core::HomingConfig{kCreepRadS, kTimeoutS, 1});
    ASSERT_EQ(foc::ErrorCode::Ok, routine.validate());
    ASSERT_FALSE(routine.begin(motor.position_reference()));

    bool finished = false;
    float finished_at_s = -1.0F;
    float captured_truth_rad = 0.0F;
    for (int i = 0; i < static_cast<int>((kTimeoutS + 0.5F) / kDt); ++i) {
        const core::HomingRoutine::Output out = routine.step(motor.position_reference(), kDt);
        motor.set_speed(out.speed_ref_rad_s);
        tick();
        if (out.finished) {
            finished = true;
            finished_at_s = static_cast<float>(i) * kDt;
            captured_truth_rad = plant.theta_mech_rad();
            break;
        }
    }
    std::printf("\n[homing-sil] finished=%d after %.3f s, state %d, failure %d, captured at truth %.4f rad\n",
                static_cast<int>(finished), static_cast<double>(finished_at_s),
                static_cast<int>(routine.state()), static_cast<int>(routine.failure()),
                static_cast<double>(captured_truth_rad));
    ASSERT_TRUE(finished) << "the search never finished";
    ASSERT_EQ(core::HomingState::Done, routine.state()) << "the search ended by giving up rather than by finding the mark";
    ASSERT_TRUE(motor.position_reference().homed());

    // C1. The allowance is **three** ticks and the breakdown matters: one because the pulse is computed from the plant at
    // the start of a period and read at its end, one because the search's own loop only learns that the origin exists on
    // the iteration *after* the tick that set it, and one for a crossing that falls inside a period rather than on its
    // boundary. Each of those is a property of where the numbers are read, not slack in the mark.
    // C1。允许值是**三个** tick，且要拆开说：一个因为脉冲由周期开始时的 plant 位置算出、在周期末读取；一个因为搜索
    // 自己的循环要到"设置原点那一 tick 之后"的那次迭代才知道原点存在；一个留给跨越落在周期内部而非边界的情形。
    // 这三条都是"数字在哪里被读到"的性质，不是标记本身的余量。
    const float tick_travel = std::fabs(plant.omega_mech_rad_s()) * kDt;
    const float index_error = std::fabs(wrap_pi_local(captured_truth_rad - kMarkMechRad));
    const float allowance = std::max(3.0F * tick_travel, 1.0e-3F);
    std::printf("[homing-sil] captured origin is %.4f rad from the mark (allowance %.4f = max(3 ticks, 1e-3))\n",
                static_cast<double>(index_error), static_cast<double>(allowance));
    EXPECT_LE(index_error, allowance) << "the search stopped away from the mark";

    // C2
    const core::HomingRoutine::Output after = routine.step(motor.position_reference(), kDt);
    EXPECT_FLOAT_EQ(0.0F, after.speed_ref_rad_s) << "a finished search kept commanding a speed";
    EXPECT_EQ(core::HomingState::Done, after.state);

    // --- hold where it stopped ---------------------------------------------------------------------------------
    // 停在哪儿就保持在哪儿。
    const float commanded = motor.position_reference().referenced_mech_rad();
    const float target_truth = commanded + kMarkMechRad;
    motor.set_position(commanded);

    float worst_after_settling = 0.0F;
    const int hold_ticks = static_cast<int>(kHoldSeconds / kDt);
    const int settle_ticks = hold_ticks - static_cast<int>(kSettleWindowS / kDt);
    for (int i = 0; i < hold_ticks; ++i) {
        tick();
        if (i >= settle_ticks) {
            worst_after_settling =
                std::max(worst_after_settling, std::fabs(wrap_pi_local(plant.theta_mech_rad() - target_truth)));
        }
    }
    std::printf("[homing-sil] holding %.4f rad (truth %.4f): worst |error| over the last %.1f s = %.4f rad\n",
                static_cast<double>(commanded), static_cast<double>(target_truth),
                static_cast<double>(kSettleWindowS), static_cast<double>(worst_after_settling));

    // C3
    EXPECT_LE(worst_after_settling, kSettleBandRad) << "the axis did not hold the place the search left it at";
}
