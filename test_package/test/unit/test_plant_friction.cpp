// The physics 3.2 needs, with its own criteria -- written before any capability uses it, because a control law tested
// against a model without stiction is tested against a machine that always creeps, and "it works" would mean nothing.
// 3.2 需要的物理，连同它自己的判据——在任何能力使用它之前写，因为用"没有静摩擦"的模型去测一个控制律，等于对着一台
// 永远会蠕动的机器测，"它能工作"也就没有意义。

#include <cmath>
#include <cstddef>

#include <gtest/gtest.h>

#include "sil_bench.hpp"   // nominal_plant_params()
#include "sil_pmsm_plant.hpp"


namespace {

constexpr float kDt = 6.25e-5F;

/**
 * @brief A machine with friction big enough to see and inertia small enough to move.
 */
auto friction_rig() -> foc::sil::PlantParams {
    foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    params.b = 1.0e-3F;         // viscous: 0.05 N*m at 50 rad/s, so the numbers below are readable
    params.t_static_nm = 0.020F; // breakaway
    params.t_coulomb_nm = 0.010F;
    params.omega_eps_rad_s = 0.5F;
    return params;
}

/**
 * @brief The voltage that makes **both** current derivatives zero at the state given: a decoupled current source. Nulling `diq` alone was not enough and the first two runs of this file measured it: with `vd = 0` the d current drifts to `we*lq*iq/Rs` (0.21 A here), and that drift removes a `we*ld*id` term from `diq`, bleeding the q current by 5 % -- 35.4 rad/s against the 40.0 the arithmetic promises. The cross-coupling term on the d axis is what keeps the d current where it was put. 只把 `diq` 置零不够，本文件前两次运行的实测说明了这一点：`vd = 0` 时 d 轴电流会漂到 `ωe·lq·iq/Rs`（这里 0.21 A），而这个漂移从 `diq` 里拿掉了一项 `ωe·ld·id`，把 q 轴电流抽走 5%——35.4 rad/s 对算术承诺的 40.0。d 轴上的交叉耦合项才是把 d 轴电流留在原处的东西。
 */
auto decoupled_source(const foc::sil::PlantParams &params, float omega_mech, float iq_setpoint, float psi_f)
    -> foc::core::Dq<float> {
    const float omega_e = omega_mech * static_cast<float>(params.pole_pairs);
    // From the *setpoints*, not from the measured currents. The first version nulled the derivative at the current the
    // plant happened to have, which makes the current a free integrator: it holds wherever the transient left it
    // (measured: iq settled at 0.90988 A against a 1.0 A setpoint, and the case read 35.49 rad/s against the 40.00 the
    // arithmetic promises). Regulating a derivative is not regulating a value.
    // 用**给定值**而不是实测电流。第一版在 plant 恰好所处的电流上把导数置零，这使电流变成一个自由积分器：它停在
    // 暂态把它留下的位置（实测：1.0 A 的给定下 iq 稳在 0.90988 A，用例读出 35.49 rad/s，而算术承诺 40.00）。
    // 调节导数不是调节数值。
    return foc::core::Dq<float>{-(omega_e * params.lq * iq_setpoint), (params.rs * iq_setpoint) + (omega_e * psi_f)};
}

/**
 * @brief Hold a fixed q current by nulling `diq` every step, so the mechanical law is the only thing under test. The plant integrates the currents from the applied voltage, so "hold iq" means feeding the voltage that makes `diq = 0` at the current speed: `vq = Rs*iq + we*psi_f`. That is a current-regulated source, which is what the mechanical equation is meant to be driven by here. plant 从施加电压积分电流，因此"保持 iq"就是喂那个让当前转速下 `diq = 0` 的电压：`vq = Rs·iq + ωe·ψf`。这就是一个电流源，也正是力学方程在这里应当被驱动的方式。
 */
auto run_open_loop(const foc::sil::PlantParams &params, float iq_a, float seconds) -> foc::sil::PlantState {
    foc::sil::PmsmPlant plant(params);
    plant.reset(foc::sil::PlantState{0.0F, iq_a, 0.0F, 0.0F});
    const float psi_f = params.kt / (1.5F * static_cast<float>(params.pole_pairs));
    const int ticks = static_cast<int>(seconds / kDt);
    for (int i = 0; i < ticks; ++i) {
        // `diq = 0` exactly, which needs the d-q cross-coupling term as well: leaving it out leaves a few
        // hundred mA of d current to bleed the q current by 5 %, and the first run of this case measured 37.85 rad/s
        // against the 40.00 the arithmetic promises.
        // 精确 `diq = 0`，这还要求 dq 交叉耦合项：漏掉它会留下几百毫安 d 轴电流把 q 轴电流抽走 5%，本用例第一版就
        // 实测 37.85 rad/s，而算术承诺的是 40.00。
        const auto now = plant.state();
        const float omega_e = now.omega_mech * static_cast<float>(params.pole_pairs);
        plant.set_voltage_dq(decoupled_source(params, now.omega_mech, iq_a, psi_f));
        plant.advance(kDt);
    }
    return plant.state();
}

} // namespace



TEST(PlantFriction, BelowTheBreakawayTorqueTheRotorDoesNotCreepAtAll) {
    // Without a stiction term the rotor creeps at every non-zero torque, however small: `T - T_coulomb` is still
    // positive. A stall-start law cannot be tested against that machine, because there is no stall to escape.
    // 没有静摩擦项时，任何非零力矩下转子都会蠕动，再小也会：`T − T_coulomb` 仍然是正的。带载起动的控制律没法对着那样
    // 的机器测，因为根本不存在"要挣脱的静止"。
    const auto params = friction_rig();
    const float kt = params.kt;
    const float below_a = 0.9F * params.t_static_nm / kt;
    const float above_a = 1.1F * params.t_static_nm / kt;

    const auto stuck = run_open_loop(params, below_a, 0.5F);
    const auto moving = run_open_loop(params, above_a, 0.5F);
    std::printf("\n[3.2-p] breakaway %.3f N*m = %.2f A: below -> %.6f rad/s, above -> %.3f rad/s\n",
                static_cast<double>(params.t_static_nm), static_cast<double>(params.t_static_nm / kt),
                static_cast<double>(stuck.omega_mech), static_cast<double>(moving.omega_mech));

    EXPECT_FLOAT_EQ(0.0F, stuck.omega_mech) << "below the breakaway torque the rotor must not move at all";
    EXPECT_GT(moving.omega_mech, 1.0F) << "above it the rotor must break away";
}


TEST(PlantFriction, OnceMovingTheSteadySpeedIsTheTorqueMinusCoulombOverViscous) {
    // The kinetic branch has an arithmetic answer: `T = T_coulomb + b*w`, so `w = (T - T_c) / b`. If the Coulomb term
    // were folded into the stiction term, or forgotten once moving, this number moves -- and no other case would say so.
    // 动摩擦分支有算术答案：`T = T_coulomb + b·ω`，所以 `ω = (T − T_c)/b`。若把库仑项并进静摩擦项、或在运动后漏掉，
    // 这个数就会变——而其他任何用例都不会说出来。
    const auto params = friction_rig();
    const float drive_nm = 0.050F;             // well above breakaway
    const float expected = (drive_nm - params.t_coulomb_nm) / params.b;
    const auto state = run_open_loop(params, drive_nm / params.kt, 2.0F);
    std::printf("[3.2-p] steady speed %.3f rad/s against %.3f predicted by (T - T_c)/b; id %.5f iq %.5f tau %.6f\n",
                static_cast<double>(state.omega_mech), static_cast<double>(expected), static_cast<double>(state.id_a),
                static_cast<double>(state.iq_a),
                static_cast<double>(1.5F * 7.0F * ((params.kt / (1.5F * 7.0F)) * state.iq_a)));
    EXPECT_NEAR(expected, state.omega_mech, 1.0F);
}


TEST(PlantFriction, ALoadStepChangesTheSteadySpeedByTheLoadOverViscous) {
    // The third piece of 3.2's physics -- load torque -- already existed; this asserts it *quantitatively*, which no case
    // did: a step of `dT` at steady state must move the speed by `dT / b`.
    // 3.2 物理的第三块——负载力矩——本来就有；本用例断言它**定量**成立，而此前没有用例这么做：稳态下 `ΔT` 的阶跃必须让
    // 转速变化 `ΔT / b`。
    const auto params = friction_rig();
    const float drive_nm = 0.050F;
    const float load_nm = 0.010F;

    const float iq_a = drive_nm / params.kt;
    const float psi_f = params.kt / (1.5F * static_cast<float>(params.pole_pairs));
    foc::sil::PmsmPlant plant(params);
    plant.reset(foc::sil::PlantState{0.0F, iq_a, 0.0F, 0.0F});
    const auto settle = [&](float seconds) {
        const int ticks = static_cast<int>(seconds / kDt);
        for (int i = 0; i < ticks; ++i) {
            const auto now = plant.state();
            plant.set_voltage_dq(decoupled_source(params, now.omega_mech, iq_a, psi_f));
            plant.advance(kDt);
        }
        return plant.state().omega_mech;
    };

    const float before = settle(1.0F);
    plant.set_load_torque(load_nm);
    const float after = settle(1.0F);
    const float expected = before - (load_nm / params.b);
    std::printf("[3.2-p] load step %.3f N*m: %.1f -> %.1f rad/s, predicted %.1f\n", static_cast<double>(load_nm),
                static_cast<double>(before), static_cast<double>(after), static_cast<double>(expected));
    EXPECT_NEAR(expected, after, 1.0F);
}


TEST(PlantFriction, WithTheFrictionTermsAtZeroThePlantIsExactlyWhatItWas) {
    // The physics is default-off, and that has to be *measurable*: every existing case and every recorded window in the
    // ledger was produced by the plant without friction, so a stray non-zero default would move all of them.
    // 该物理默认关闭，而这必须能被**测量**：所有既有用例与台账里每一个窗口都由无摩擦的 plant 产出，因此一个多余的非零
    // 默认值会把它们全部挪动。
    const auto nominal = foc::sil::nominal_plant_params();
    EXPECT_FLOAT_EQ(0.0F, nominal.t_static_nm);
    EXPECT_FLOAT_EQ(0.0F, nominal.t_coulomb_nm);
    EXPECT_FLOAT_EQ(0.0F, nominal.omega_eps_rad_s);

    const auto state = run_open_loop(nominal, 0.5F, 0.5F);
    EXPECT_GT(state.omega_mech, 0.0F) << "and 0.5 A still moves the rotor, so the rig itself is not the reason";
}
