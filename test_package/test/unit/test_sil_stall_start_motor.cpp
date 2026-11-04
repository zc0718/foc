// `Motor::begin_stall_start`, measured: a stuck rotor breaks loose at a torque the caller can name, and
// the value the sequence reports is the one the speed loop has to take over on.
//
// Three cases, and the third is what keeps the first two honest:
//
//   1. a load the drive can beat: the phase walks Ramping -> Released, the reported hand-over torque is
//      the load it actually beat (not the recipe's estimate), and the rotor turns afterwards;
//   2. a load above the sequence's own ceiling: Ramping -> Failed, `FaultCode::Stall` is latched and the
//      bridge is off. `Failed` is a first-class answer -- a sequence that ramps forever instead of
//      saying so leaves an operator watching a silent axis;
//   3. the counterfactual: the same bench, the same ticks, the only difference is that the sequence is
//      never begun. If the rotor moved anyway, cases 1 and 2 would be statements about the plant rather
//      than about the sequence, and nothing in the numbers would say so.
//
// The load is the plant's own `set_load_torque`, so the "stiction" is a real torque the machine has to
// beat rather than a flag: the rotor stays where it is because the drive cannot produce more than the
// load, which is the same reason a real stuck axis stays put.
//
// 负载用的是 plant 自己的 `set_load_torque`，所以这个"静摩擦"是机器真要战胜的力矩而不是一个标志位：
// 转子不动的原因是驱动产生的力矩不如负载，与真机上卡住的轴不动的原因完全相同。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/alignment.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/position.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>

#include "sil_bench.hpp"
#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 4;
constexpr float kVBus = 24.0F;

// `NullAlignment` because this test is about the breakaway sequence, not about finding the sensor
// offset: `begin()` answers false, which is Motor's signal to go straight from Align to Run.
using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi,
                                 foc::sil::PlantHal, foc::core::NullReferenceGenerator,
                                 foc::core::NullFeedforward, foc::core::PositionP, foc::core::NullAlignment>;

/**
 * @brief [en] What one breakaway run produced, including the phase transitions.
 * @brief [zh] 一次起步运行产出的东西，含阶段迁移。
 */
struct Outcome final {
    int ticks_to_release{-1};
    float handover_torque_nm{0.0F};
    float speed_at_release_rad_s{0.0F};
    float speed_after_rad_s{0.0F};
    float travelled_rad{0.0F};
    bool failed{false};
    int fault_code{0};
    float peak_torque_nm{0.0F};
};

auto recipe(float breakaway_nm, float max_torque_nm, float ramp_nm_per_s = 0.4F,
            float give_up_s = 0.15F) -> foc::core::StallStartConfig {
    foc::core::StallStartConfig config{};
    config.breakaway_nm = breakaway_nm;
    config.max_torque_nm = max_torque_nm;
    config.ramp_nm_per_s = ramp_nm_per_s;
    config.release_speed_rad_s = 1.0F;
    config.give_up_s = give_up_s;
    return config;
}

/**
 * @brief [en] One run on a plant whose rotor is held by a load torque.
 * @brief [zh] 在转子被负载转矩按住的 plant 上跑一次。
 * @param stiction_nm [en] the torque the rotor has to beat before it can move.
 * @param stiction_nm [zh] 转子动起来之前要战胜的力矩。
 * @param use_sequence [en] when false the sequence is never begun: the counterfactual case.
 * @param use_sequence [zh] 为假时从不启动序列：反事实那一例。
 * @param ticks [en] how long to run.
 * @param ticks [zh] 跑多久。
 */
auto run_case(float stiction_nm, const foc::core::StallStartConfig &config, bool use_sequence, int ticks) -> Outcome {
    const foc::sil::MachineSpec spec = foc::sil::nominal_spec();
    // Stiction, not a load torque: `set_load_torque` applies a torque to the shaft, which accelerates a
    // rotor the drive is not holding. The breakaway case needs a torque the rotor has to *beat* -- the
    // plant models it as `t_static_nm`, with Coulomb friction once it is moving.
    foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    params.t_static_nm = stiction_nm;
    params.t_coulomb_nm = 0.8F * stiction_nm;
    params.omega_eps_rad_s = 0.5F;
    foc::sil::PmsmPlant plant(params);
    foc::sil::PlantState initial{};
    initial.theta_mech = 0.0F;
    plant.reset(initial);

    foc::sil::PlantHal hal(plant, kVBus);
    const foc::motor::MotorConfig motor_config = foc::sil::nominal_motor_config();
    const foc::core::Pi::Config current =
        foc::current_pi_config(motor_config, foc::sil::standard_bench_config().omega_current_rad_s, kVBus);
    const foc::core::Pi::Config speed =
        foc::speed_pi_config(motor_config, foc::sil::standard_bench_config().omega_speed_rad_s);

    MotorT motor(motor_config,
                 MotorT::Policies{foc::core::BackEmfObserver(foc::core::BackEmfObserver::Config{spec.rs, spec.ld}),
                                  foc::core::Svpwm{}, foc::core::Pi{current}, foc::core::Pi{current},
                                  foc::core::Pi{speed}, foc::core::NullReferenceGenerator{},
                                  foc::core::NullFeedforward{}, foc::core::PositionP{},
                                  foc::core::NullAlignment{}},
                 hal);
    Outcome out{};
    if ((motor.init() != foc::ErrorCode::Ok) || (motor.start() != foc::ErrorCode::Ok)) {
        return out;
    }
    if (use_sequence && (motor.begin_stall_start(config) != foc::ErrorCode::Ok)) {
        return out;
    }

    const float start_angle = plant.theta_mech_rad();
    for (int tick = 0; tick < ticks; ++tick) {
        const foc::ErrorCode rc = motor.tick(kDt);
        out.peak_torque_nm = std::fmax(out.peak_torque_nm, std::fabs(motor.stall_start_torque_nm()));
        if ((out.ticks_to_release < 0) && (motor.stall_start_phase() == foc::core::StallStartPhase::Released)) {
            out.ticks_to_release = tick;
            out.handover_torque_nm = motor.stall_start_torque_nm();
            out.speed_at_release_rad_s = plant.omega_mech_rad_s();
            // The sequence hands the axis back: without taking over in the same period the breakaway
            // torque keeps being applied, which is what this call prevents.
            motor.set_speed(20.0F);
        }
        hal.advance(kDt, kSubsteps);
        if (rc == foc::ErrorCode::Fault) {
            out.failed = true;
            out.fault_code = static_cast<int>(motor.fault_code());
            break;
        }
    }
    out.speed_after_rad_s = plant.omega_mech_rad_s();
    out.travelled_rad = plant.theta_mech_rad() - start_angle;
    return out;
}

/**
 * @brief [en] Does the rotor break loose, at the torque the sequence says it did?
 * @brief [zh] 转子挣脱了吗？是在序列所说的那个力矩上吗？
 */
TEST(SilStallStartMotor, TheRotorBreaksLooseAtTheTorqueTheSequenceReports) {
    const foc::core::StallStartConfig config = recipe(0.030F, 0.060F);
    const Outcome out = run_case(0.030F, config, true, 20000);

    ASSERT_GE(out.ticks_to_release, 0) << "the sequence never reached Released on a load it can beat";
    EXPECT_FALSE(out.failed);
    // The hand-over torque is a measurement: the ramp crosses the load and the rotor answers on the
    // next period, so the reported value sits just above the load rather than exactly on it.
    EXPECT_GT(out.handover_torque_nm, 0.028F);
    EXPECT_LT(out.handover_torque_nm, 0.038F);
    // 0.030 N*m at 0.4 N*m/s is 75 ms, i.e. 1500 periods, plus the load's own margin.
    EXPECT_GT(out.ticks_to_release, 1000);
    EXPECT_LT(out.ticks_to_release, 2500);
    // And it actually turned afterwards: the sequence's job is to hand over a moving axis, not a torque.
    EXPECT_GT(std::fabs(out.travelled_rad), 0.05F);
    std::printf("[stall-start] released after %.1f ms at %.4f N*m, travelled %.3f rad\n",
                static_cast<double>(out.ticks_to_release) * kDt * 1e3, static_cast<double>(out.handover_torque_nm),
                static_cast<double>(out.travelled_rad));
}

/**
 * @brief [en] A load above the sequence's ceiling is a reported failure, not a silent ramp.
 * @brief [zh] 超出序列上限的负载是一个被报告的失败，而不是一条沉默的斜坡。
 */
TEST(SilStallStartMotor, ALoadAboveTheCeilingFailsLoudly) {
    const foc::core::StallStartConfig config = recipe(0.030F, 0.060F, 0.4F, 0.10F);
    const Outcome out = run_case(0.090F, config, true, 20000);

    EXPECT_TRUE(out.failed) << "the drive could not move the load and said nothing";
    EXPECT_EQ(out.fault_code, static_cast<int>(foc::core::FaultCode::Stall));
    EXPECT_LT(out.ticks_to_release, 0);
    // The ramp reaches its own ceiling and stops there: a sequence that kept climbing would be a
    // torque command nobody bounded.
    EXPECT_GT(out.peak_torque_nm, 0.055F);
    EXPECT_LT(out.peak_torque_nm, 0.065F);
}

/**
 * @brief [en] The counterfactual: without the sequence the same bench does not move the rotor.
 * @brief [zh] 反事实：没有序列，同一个台架不会让转子动。
 * @note [en] If this case passed with the rotor moving, the two cases above would be measurements of the plant rather
 *            than of the sequence -- and no number in them would say so. This is the E4 shape the plan asks for, on a
 *            capability that just became reachable.
 * @note [zh] 如果这一例在转子会动的情况下也通过，上面两例就成了关于 plant 而不是关于序列的测量——而它们里面没有任何数字
 *            会说出这件事。这就是计划要求的 E4 形态，落在刚刚变为可达的那个能力上。
 */
TEST(SilStallStartMotor, WithoutTheSequenceTheSameBenchDoesNotMoveTheRotor) {
    const foc::core::StallStartConfig config = recipe(0.030F, 0.060F);
    const Outcome out = run_case(0.030F, config, false, 20000);

    EXPECT_LT(out.ticks_to_release, 0);
    EXPECT_EQ(out.peak_torque_nm, 0.0F) << "the sequence reported a torque it was never asked for";
    EXPECT_LT(std::fabs(out.travelled_rad), 1e-3F) << "the rotor moved without the sequence commanding anything";
}
} // namespace
