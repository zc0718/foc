// S2 - speed step 0 -> 100 rad/s with the speed loop closed, per PRD section 8.4.
// S2 - 速度环闭环的速度阶跃 0 -> 100 rad/s，对应 PRD 8.4。

#include <vector>

#include <gtest/gtest.h>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"
#include "sil_speed_step_case.hpp"



TEST(SilSpeedStep, ReachesTheReferenceWithinTheThresholds) {
    const foc::sil::SpeedStepResult r = foc::sil::run_speed_step(foc::sil::nominal_plant_params(), 100.0F);

    // PRD 8.4: overshoot <= 10 %.
    EXPECT_LE(r.overshoot, 0.10F);
    // PRD 8.4: steady-state error <= 1 %.
    EXPECT_LE(r.steady_error_fraction, 0.01F);
    // No sustained oscillation over the last 20 %.
    EXPECT_LE(r.tail_peak_to_peak, 0.02F * 100.0F);
    EXPECT_GE(r.settling_s, 0.0F) << "speed never settled";
    EXPECT_GT(r.speed_final, 0.0F);
}



TEST(SilSpeedStep, DirectionIsReversible) {
    const foc::sil::SpeedStepResult r = foc::sil::run_speed_step(foc::sil::nominal_plant_params(), -100.0F);

    EXPECT_LE(r.steady_error_fraction, 0.01F);
    EXPECT_LT(r.speed_final, 0.0F);
    EXPECT_GE(r.settling_s, 0.0F) << "speed never settled";
}



TEST(SilSpeedStep, CurrentStaysInsideTheNameplateLimit) {
    const foc::sil::SpeedStepResult r = foc::sil::run_speed_step(foc::sil::nominal_plant_params(), 100.0F);

    EXPECT_LE(r.peak_iq, foc::sil::nominal_spec().i_max_a + 1e-3F);
}
