// S4 - saturation and recovery, per PRD section 8.4. The bus voltage is deliberately too low to
// hold the first command, so the Vdq limit is hit; the reference then drops back.
// S4 - 限幅与恢复，对应 PRD 8.4。母线电压刻意不足以支撑首个指令，从而触发 Vdq 限幅；随后给定回落。

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/transforms.hpp>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"



namespace {

constexpr float kBusV = 12.0F;
constexpr float kAxisLimitV = 0.5F * kBusV; // per-axis voltage clamp of the current PI
constexpr float kSaturatedRefRadS = 300.0F;
constexpr float kRecoveredRefRadS = 100.0F;

auto make_config() -> foc::sil::BenchConfig {
    foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
    cfg.v_bus_v = kBusV;
    return cfg;
}

/// [en] Magnitude of the stationary voltage vector currently applied by the inverter.
/// [zh] 逆变器当前施加的静止电压矢量幅值。
auto applied_v_ab(foc::sil::PlantHal& hal) -> float {
    const foc::core::Uvw<float> duty = hal.pwm.duty();
    const foc::core::Ab<float> v_ab =
        foc::core::clarke(foc::core::Uvw<float>{duty.u * kBusV, duty.v * kBusV, duty.w * kBusV});
    return std::sqrt((v_ab.alpha * v_ab.alpha) + (v_ab.beta * v_ab.beta));
}

} // namespace



TEST(SilSaturationLimit, SaturatesTheModulatorAndRecoversWithinTwoHundredMilliseconds) {
    foc::sil::Bench bench(make_config());
    bench.motor().set_speed(0.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());
    bench.run(2e-3F);

    // 1. Ask for far more speed than the bus can deliver -> Vdq limiting.
    bench.motor().set_speed(kSaturatedRefRadS);
    std::vector<foc::sil::Sample> saturated;
    bool hit_limit = false;
    bench.run(0.3F, [&bench, &saturated, &hit_limit](const foc::sil::Bench&) {
        saturated.push_back(bench.sample());
        // The current PI clamps each axis at v_bus/2; reaching it proves the Vdq limit is active.
        if (applied_v_ab(bench.hal()) >= 0.95F * kAxisLimitV) {
            hit_limit = true;
        }
    });

    const std::vector<float> speed_during = foc::sil::series_of(saturated, &foc::sil::Sample::speed);
    EXPECT_TRUE(hit_limit) << "the modulator never reached its linear limit";
    EXPECT_LT(foc::sil::max_of(speed_during), 0.8F * kSaturatedRefRadS) << "the bus cannot deliver this speed";
    float peak_iq = 0.0F;
    for (const foc::sil::Sample& s : saturated) {
        const float magnitude = std::fabs(s.iq_a);
        peak_iq = (magnitude > peak_iq) ? magnitude : peak_iq;
    }
    EXPECT_LE(peak_iq, foc::sil::nominal_spec().i_max_a + 1e-3F); // clamped, not diverging

    // 2. Drop back to a speed the bus can hold: the loop must recover inside 200 ms.
    bench.motor().set_speed(kRecoveredRefRadS);
    std::vector<foc::sil::Sample> recovery;
    recovery.reserve(4200);
    bench.run(0.2F, [&recovery](const foc::sil::Bench& b) { recovery.push_back(b.sample()); });

    const std::vector<float> speed = foc::sil::series_of(recovery, &foc::sil::Sample::speed);

    // PRD 8.4: back inside the S2 metrics within 200 ms of leaving the limit.
    EXPECT_LE(std::fabs(foc::sil::tail_mean(speed, 0.1F) - kRecoveredRefRadS), 0.01F * kRecoveredRefRadS);
    EXPECT_LE(foc::sil::tail_peak_to_peak(speed, 0.2F), 0.02F * kRecoveredRefRadS);

    // PRD 8.4: no reverse overshoot above 15 % caused by windup.
    EXPECT_GE(foc::sil::min_of(speed), 0.85F * kRecoveredRefRadS);
}



TEST(SilSaturationLimit, RecoversToTheTwoPercentBandInsideTwoHundredMilliseconds) {
    foc::sil::Bench bench(make_config());
    bench.motor().set_speed(0.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());
    bench.run(2e-3F);

    bench.motor().set_speed(kSaturatedRefRadS);
    bench.run(0.3F);

    bench.motor().set_speed(kRecoveredRefRadS);
    std::vector<foc::sil::Sample> recovery;
    recovery.reserve(4200);
    bench.run(0.2F, [&recovery](const foc::sil::Bench& b) { recovery.push_back(b.sample()); });

    const float settled =
        foc::sil::settling_time_s(recovery, &foc::sil::Sample::speed, kRecoveredRefRadS, 0.02F * kRecoveredRefRadS);
    ASSERT_GE(settled, 0.0F) << "the speed never returned to the 2 % band";
    EXPECT_LE(settled, 0.2F);
}
