// S1 - current step in torque mode (speed loop open), per PRD section 8.4.
// S1 - 转矩模式下（速度环开环）的电流阶跃，对应 PRD 8.4。

#include <vector>

#include <gtest/gtest.h>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"



namespace {

using foc::sil::Bench;
using foc::sil::Sample;

auto make_bench() -> foc::sil::BenchConfig {
    foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
    cfg.v_bus_v = 24.0F;
    return cfg;
}

} // namespace



TEST(SilS1CurrentStep, TracksTheReferenceWithoutOvershoot) {
    Bench bench(make_bench());
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());

    bench.motor().set_torque(0.0F);
    bench.run(1e-3F); // settle at zero current

    bench.motor().set_torque(1.0F);
    std::vector<Sample> trace;
    trace.reserve(400);
    bench.run(0.02F, [&trace](const Bench& b) { trace.push_back(b.sample()); });

    ASSERT_FALSE(trace.empty());
    const std::vector<float> iq = foc::sil::series_of(trace, &Sample::iq_a);

    // PRD 8.4: overshoot <= 10 %.
    EXPECT_LE(foc::sil::overshoot_fraction(iq, 1.0F, 0.0F), 0.10F);

    // PRD 8.4: 2 % settling. The literal formula "5*dt_ctrl*20" resolves to 5 closed-loop time
    // constants (PRD erratum D26); the current loop is critically damped at 2 krad/s, so 2.5 ms.
    const float settled = foc::sil::settling_time_s(trace, &Sample::iq_a, 1.0F, 0.02F);
    ASSERT_GE(settled, 0.0F) << "iq never settled";
    EXPECT_LE(settled, 5.0F / bench.config().omega_current_rad_s);

    // PRD 8.4: no sustained oscillation - peak-to-peak over the last 20 % below 2 % of the step.
    EXPECT_LE(foc::sil::tail_peak_to_peak(iq, 0.2F), 0.02F);
}



TEST(SilS1CurrentStep, HoldsTheDaxisCurrentAtZero) {
    Bench bench(make_bench());
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());

    bench.motor().set_torque(1.0F);
    std::vector<Sample> trace;
    bench.run(0.02F, [&trace](const Bench& b) { trace.push_back(b.sample()); });

    const std::vector<float> id = foc::sil::series_of(trace, &Sample::id_a);
    // The d axis has no reference: it must stay near zero while the rotor accelerates.
    EXPECT_LE(foc::sil::max_abs_error(id, 0.0F), 0.02F * 1.0F);
}



TEST(SilS1CurrentStep, NegativeReferenceIsSymmetric) {
    Bench bench(make_bench());
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());

    bench.motor().set_torque(-1.0F);
    std::vector<Sample> trace;
    bench.run(0.02F, [&trace](const Bench& b) { trace.push_back(b.sample()); });

    const std::vector<float> iq = foc::sil::series_of(trace, &Sample::iq_a);
    // The rotor keeps accelerating, so the back EMF ramps; a type-1 current loop then trails by
    // EMF_rate/ki (here ~1 % of 1 A). Exact zero steady-state error is asserted in the locked-rotor
    // L1 case instead.
    EXPECT_NEAR(-1.0F, foc::sil::tail_mean(iq, 0.2F), 0.02F);
    EXPECT_GE(foc::sil::min_of(iq), -1.1F); // overshoot <= 10 % below the target
}
