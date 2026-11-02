// Scenario harness for the independent reference in `plant_reference.py`. Not part of the library and never installed:
// it exists so that a float64 solver in another language can be handed exactly what the C++ side computed.
//
// The design point is what gets dumped. Recording the *state* alone would only let the reference check the scenarios it
// can re-run itself, i.e. open loop, which is the case least in need of checking. Recording the **applied voltage**
// alongside the state makes the plant's integration auditable under any controller at all: the reference replays each
// control period from the dumped state with the dumped duty and asks whether it lands on the dumped next state. That is
// what lets the closed-loop scenario -- the real PositionP -> PI speed -> PI current -> SVPWM cascade -- be checked
// without re-implementing a single controller line in Python, so a disagreement is unambiguously a plant disagreement.
//
// 为什么要导出「实际施加的电压」而不只是状态：只记状态，参考实现就只能查它自己能重跑的场景（开环），而那恰好是最
// 不需要查的那一个。把**施加电压**与状态一起记下来，plant 的积分就能在**任何控制器**下受审：参考端用导出的状态和
// 导出的占空比，重放每一个控制周期，问它是否落在导出的下一个状态上。这就是闭环场景——真正的
// PositionP → PI 速度 → PI 电流 → SVPWM 级联——能被查出来的原因，而 Python 里一行控制器代码都不用复刻，于是任何
// 分歧都无可争辩地是 plant 的分歧。

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <vector>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/position.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>

#include <sil_bench.hpp>
#include <sil_pmsm_plant.hpp>
#include <sil_plant_hal.hpp>

namespace {

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 4;
constexpr float kVBus = 24.0F;
constexpr float kPositionBandwidth = 20.0F;
constexpr float kVelocityLimit = 50.0F;

using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                 foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                 foc::core::PositionP>;

/// The scenario's machine, emitted ahead of the data. The reference reads these instead of restating them: a parameter
/// copied into two languages is a parameter that will disagree eventually, and the disagreement would look like a plant
/// defect. The *equations* stay independently written; only the numbers are shared, because they are the scenario's input.
void emit_machine(const foc::sil::PmsmPlant& plant) {
    const foc::sil::PlantParams& p = plant.params();
    std::printf("# machine,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%u,%.9g,%d\n", static_cast<double>(p.rs),
                static_cast<double>(p.ld), static_cast<double>(p.lq), static_cast<double>(p.kt),
                static_cast<double>(p.j), static_cast<double>(p.b), static_cast<unsigned>(p.pole_pairs),
                static_cast<double>(kDt), kSubsteps);
}

void emit(int step, const foc::sil::PmsmPlant& plant, foc::sil::PlantHal& hal) {
    const foc::sil::PlantState st = plant.state();
    const foc::core::Uvw<float> duty = hal.pwm.duty();
    const float v_bus = hal.dc_link.voltage_v();
    std::printf("%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g\n", step, static_cast<double>(st.id_a),
                static_cast<double>(st.iq_a), st.omega_mech, st.theta_mech, static_cast<double>(duty.u * v_bus),
                static_cast<double>(duty.v * v_bus), static_cast<double>(duty.w * v_bus));
}

// Held-voltage, open loop. The shortest path to "does the integrator integrate", with nothing in between.
int run_openloop() {
    constexpr float v_bus = kVBus;
    foc::sil::PmsmPlant plant(
        foc::sil::PlantParams{0.5F, 4.0e-4F, 8.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, 7U, 0.0F, 0.0F, 0.05F});
    foc::sil::PlantHal hal(plant, v_bus);
    plant.reset(foc::sil::PlantState{0.5F, 1.0F, 5.0F, 0.3F});
    // Three phase voltages chosen directly rather than inverted from a dq request, so that the reference has no
    // modulation step to share a mistake with. Clarke is immune to common mode, so the zero sequence does not matter to
    // the plant and a balanced set is enough to exercise a rotating dq.
    const foc::core::Uvw<float> v_uvw{2.0F, -1.0F, -1.0F};
    static_cast<void>(hal.pwm.enable());
    hal.pwm.write_duty(foc::core::Uvw<float>{v_uvw.u / v_bus, v_uvw.v / v_bus, v_uvw.w / v_bus});
    emit_machine(plant);
    std::printf("step,id,iq,omega,theta,vu,vv,vw\n");
    emit(0, plant, hal);
    for (int i = 1; i <= 2000; ++i) { // 100 ms
        hal.advance(kDt, kSubsteps);
        emit(i, plant, hal);
    }
    return 0;
}

// The real cascade, position step. Same construction as `test_sil_position_budget.cpp` so a disagreement here is about
// the plant and not about a different tuning.
int run_closedloop() {
    const foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, 7U, 3.0F, 20000.0F, 0.0F};
    foc::sil::PmsmPlant plant(
        foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, 7U, 0.0F, 0.0F, 0.05F});
    foc::sil::PlantHal hal(plant, kVBus);
    plant.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
    const typename MotorT::Policies policies{
        foc::core::BackEmfObserver({mc.rs, mc.ld}),
        foc::core::Svpwm{},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
        foc::core::NullReferenceGenerator{},
        foc::core::NullFeedforward{},
        foc::core::PositionP{foc::position_p_config(mc, kPositionBandwidth, kVelocityLimit, 0.0F)}};
    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_position(1.0F);
    static_cast<void>(motor.start());
    emit_machine(plant);
    std::printf("step,id,iq,omega,theta,vu,vv,vw\n");
    const int steps = 24000; // 1.2 s
    // The dump pairs a state with the voltage that will move it, so a replay is `state[k] + v[k] -> state[k+1]`.
    // Emitting after `advance` instead would pair each state with the voltage of the *previous* period: the two columns
    // straddle a period boundary, and a replay built on them disagrees by a whole period while looking like a defect.
    // 导出把「状态」与「将要推动它的电压」配在一起，于是重放就是 `state[k] + v[k] -> state[k+1]`。若在 `advance` 之后
    // 导出，每个状态就会配上**上一个**周期的电压：两列跨在一个周期边界两侧，用它做重放会整整差一个周期，而看起来
    // 却像一个缺陷。
    for (int i = 1; i <= steps; ++i) {
        static_cast<void>(motor.tick(kDt)); // writes the duty this period will apply
        emit(i - 1, plant, hal);            // state before the advance, duty during the period
        hal.advance(kDt, kSubsteps);
    }
    emit(steps, plant, hal); // final state; its duty column is the last one already spent
    return 0;
}

// One positioning move, parameterised from the command line, printing only where the rotor ended up.
//
// This scenario exists for claims rather than for the integrator, and it dumps the final state alone on purpose. The
// positioning questions -- ISO 230-2's accuracy, repeatability and reversal value, and the distribution of a quantised
// sensor's residual -- are answered by sweeping the *same* move many times with one variable changed: which side the
// axis approached from, where the target falls inside a count, whether there is stiction. Sixty dense trajectories of
// 24000 rows each would be 140 MB of CSV to compute a single number per run, and the audits that need dense data are
// the ones that run on `openloop` and `closedloop`.
//
// 本场景是为**主张**而不是为积分器存在的，且故意只导出末态。定位类问题——ISO 230-2 的精度/重复性/反转值，以及量化
// 传感器残差的分布——由同一个动作反复扫出来，每次只改一个变量：从哪一侧逼近、目标落在一个计数内的哪里、有没有静
// 摩擦。六十条每条 24000 行的密集轨迹只为一个数，是 140 MB 的 CSV；需要密集数据的审计跑在 `openloop`/`closedloop`
// 上。
int run_positioning(int argc, char **argv) {
    if (argc < 7) {
        std::fprintf(stderr, "positioning <start> <target> <lsb> <friction_nm> <seconds>\n");
        return 2;
    }
    const float start_rad = static_cast<float>(std::atof(argv[2]));
    const float target_rad = static_cast<float>(std::atof(argv[3]));
    const float lsb_rad = static_cast<float>(std::atof(argv[4]));
    const float friction_nm = static_cast<float>(std::atof(argv[5]));
    const float seconds = static_cast<float>(std::atof(argv[6]));

    const foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, 7U, 3.0F, 20000.0F, 0.0F};
    foc::sil::PmsmPlant plant(
        foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, 7U, friction_nm, friction_nm, 0.05F});
    foc::sil::PlantHal hal(plant, kVBus);
    hal.position.set_resolution_rad(lsb_rad);
    plant.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, start_rad});
    const typename MotorT::Policies policies{
        foc::core::BackEmfObserver({mc.rs, mc.ld}),
        foc::core::Svpwm{},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
        foc::core::NullReferenceGenerator{},
        foc::core::NullFeedforward{},
        foc::core::PositionP{foc::position_p_config(mc, kPositionBandwidth, kVelocityLimit, 0.0F)}};
    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_position(target_rad);
    static_cast<void>(motor.start());
    const int steps = static_cast<int>(seconds / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
    }
    emit_machine(plant);
    const foc::sil::PlantState st = plant.state();
    std::printf("final,%.9g\n", st.theta_mech);
    return 0;
}

} // namespace

int main(int argc, char **argv) {
    const char *scenario = (argc > 1) ? argv[1] : "openloop";
    if (std::strcmp(scenario, "openloop") == 0) {
        return run_openloop();
    }
    if (std::strcmp(scenario, "closedloop") == 0) {
        return run_closedloop();
    }
    if (std::strcmp(scenario, "positioning") == 0) {
        return run_positioning(argc, argv);
    }
    std::fprintf(stderr, "unknown scenario '%s' (openloop|closedloop|positioning)\n", scenario);
    return 2;
}
