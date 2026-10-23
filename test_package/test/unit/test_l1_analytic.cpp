// L1 - analytic reference (hard gate per PRD 8.5). The closed loops of the bench are compared with
// their closed-form step responses, not with another simulation.
// L1 - 解析解对照（PRD 8.5 的硬门禁）。台架的闭环响应与闭式解比较，而不是与另一次仿真比较。

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"



namespace {

using Sample = foc::sil::Sample;

constexpr float kTolerance = 0.03F; // absolute error against the closed form

/**
 * [en] Closed form of the PI + R-L current loop for the critically damped tuning.
 *      With `ki = wn^2 L` and `kp = 2 wn L - R` the loop is
 *      `(kp s + ki) / (L (s + wn)^2)`, whose step response is
 *      `1 - e^(-wn t) * (1 - (wn - R/L) t)`; the numerator zero is what makes it faster than the
 *      textbook `1 - e^(-x)(1 + x)`.
 *      [zh] 临界阻尼整定下 PI + R-L 电流环的闭式解：`ki = wn^2 L`、`kp = 2 wn L - R` 时环路为
 *      `(kp s + ki) / (L (s + wn)^2)`，阶跃响应为 `1 - e^(-wn t) * (1 - (wn - R/L) t)`；
 *      分子上的零点正是它比教科书式 `1 - e^(-x)(1 + x)` 更快的原因。
 */
auto current_loop_closed_form(float omega_n, float r_over_l, float t_s) -> float {
    const float x = omega_n * t_s;
    return 1.0F - (std::exp(-x) * (1.0F - ((omega_n - r_over_l) * t_s)));
}

/// [en] First-order step response: 1 - e^(-wn t).
/// [zh] 一阶阶跃响应：1 - e^(-wn t)。
auto first_order(float omega_n, float t_s) -> float { return 1.0F - std::exp(-omega_n * t_s); }

/// [en] Sample of the trace closest to `t_s`, measured from the first sample.
/// [zh] 轨迹中最接近 `t_s` 的采样（相对首个采样）。
auto at_time(const std::vector<foc::sil::Sample>& trace, float Sample::* field, float t_s) -> float {
    const float t0 = trace.front().t_s;
    std::size_t best = 0U;
    float best_gap = 1e9F;
    for (std::size_t i = 0U; i < trace.size(); ++i) {
        const float gap = std::fabs((trace[i].t_s - t0) - t_s);
        if (gap < best_gap) {
            best_gap = gap;
            best = i;
        }
    }
    return trace[best].*field;
}

} // namespace



TEST(SilL1Analytic, CurrentLoopMatchesTheCriticallyDampedClosedForm) {
    foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
    // A control period 12.5 times shorter than the loop time constant makes the ZOH / forward-Euler
    // discretisation negligible, so the continuous closed form is the honest reference.
    cfg.dt_ctrl_s = 5e-6F;
    const foc::motor::MotorConfig mc = foc::sil::nominal_motor_config();

    // A locked rotor removes the mechanical dynamics, so the q axis is exactly the SISO loop
    // Kt-free R-L plant driven by a PI controller.
    foc::sil::PlantParams locked = foc::sil::nominal_plant_params();
    locked.j = 1e6F;

    foc::sil::Bench bench(cfg, locked);
    bench.motor().set_torque(0.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());
    bench.run(1e-3F);

    bench.motor().set_torque(1.0F);
    std::vector<Sample> trace;
    bench.run(0.01F, [&trace](const foc::sil::Bench& b) { trace.push_back(b.sample()); });
    ASSERT_FALSE(trace.empty());

    // Closed-loop poles from the gains themselves: ki = wn^2 L, kp = 2 wn L - R  =>  zeta = 1.
    const foc::core::Pi::Config pi = foc::sil::current_pi_config(mc, cfg.omega_current_rad_s, cfg.v_bus_v);
    const float omega_n = std::sqrt(pi.ki / mc.ld);
    const float zeta = (mc.rs + pi.kp) / (2.0F * omega_n * mc.ld);
    ASSERT_NEAR(1.0F, zeta, 1e-3F) << "the tuning must be critically damped for this closed form";

    for (const float multiples : {1.0F, 2.0F, 4.0F}) {
        const float t_s = multiples / omega_n;
        const float measured = at_time(trace, &Sample::iq_a, t_s);
        const float expected = current_loop_closed_form(omega_n, mc.rs / mc.ld, t_s);
        EXPECT_NEAR(expected, measured, kTolerance) << "t = " << t_s;
    }

    // The integral term removes the steady-state error completely.
    const std::vector<float> iq = foc::sil::series_of(trace, &Sample::iq_a);
    EXPECT_NEAR(1.0F, foc::sil::tail_mean(iq, 0.2F), 0.005F);
    EXPECT_LE(foc::sil::overshoot_fraction(iq, 1.0F, 0.0F), kTolerance);
}



TEST(SilL1Analytic, SpeedLoopMatchesTheFirstOrderClosedForm) {
    const foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
    const foc::motor::MotorConfig mc = foc::sil::nominal_motor_config();

    foc::sil::Bench bench(cfg);
    bench.motor().set_speed(0.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());
    bench.run(1e-3F);

    // 5 rad/s keeps iq at 2.5 A, inside the 3 A limit, so the loop stays linear.
    const float target = 5.0F;
    bench.motor().set_speed(target);
    std::vector<Sample> trace;
    bench.run(0.05F, [&trace](const foc::sil::Bench& b) { trace.push_back(b.sample()); });
    ASSERT_FALSE(trace.empty());

    // ki/kp = B/J cancels the mechanical pole, leaving omega_loop/s: a first-order closed loop.
    const foc::core::Pi::Config pi = foc::sil::speed_pi_config(mc, cfg.omega_speed_rad_s);
    ASSERT_NEAR(mc.b / mc.j, pi.ki / pi.kp, 1e-3F) << "the zero must cancel the mechanical pole";
    const float omega_c = pi.kp * mc.kt / mc.j;
    EXPECT_NEAR(cfg.omega_speed_rad_s, omega_c, 1.0F);

    for (const float multiples : {1.0F, 2.0F, 3.0F}) {
        const float t_s = multiples / omega_c;
        const float measured = at_time(trace, &Sample::speed, t_s) / target;
        const float expected = first_order(omega_c, t_s);
        EXPECT_NEAR(expected, measured, kTolerance) << "t = " << t_s;
    }

    const std::vector<float> speed = foc::sil::series_of(trace, &Sample::speed);
    EXPECT_NEAR(target, foc::sil::tail_mean(speed, 0.2F), 0.01F * target);
    EXPECT_LE(foc::sil::overshoot_fraction(speed, target, 0.0F), kTolerance);
}



TEST(SilL1Analytic, OpenLoopElectricalTimeConstantMatchesTheAnalyticSolution) {
    // Pure RL step without the controller: the plant itself must reproduce the closed form.
    const foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    foc::sil::PmsmPlant plant(params);
    plant.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
    plant.set_voltage_dq(foc::core::Dq<float>{1.0F, 0.0F});

    const float tau = params.ld / params.rs;
    const float target = 1.0F / params.rs;
    const float dt = 5e-6F;

    float elapsed = 0.0F;
    for (const float multiples : {0.5F, 1.0F, 3.0F}) {
        const float until = multiples * tau;
        const int steps = static_cast<int>(((until - elapsed) / dt) + 0.5F);
        for (int i = 0; i < steps; ++i) {
            plant.advance(dt);
        }
        elapsed = until;
        EXPECT_NEAR(target * (1.0F - std::exp(-elapsed / tau)), plant.current_dq().d, 0.01F * target)
            << "t = " << elapsed;
    }
}
