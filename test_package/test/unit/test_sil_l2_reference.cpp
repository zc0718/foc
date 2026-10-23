// L2 - discretisation arbiter (hard gate per PRD 8.5): the C++ closed loop is compared with the
// reference trace produced by .github/misc/offline_arbiter.py (scipy, independent integrator).
// L2 - 离散化仲裁（PRD 8.5 硬门禁）：C++ 闭环与 .github/misc/offline_arbiter.py（scipy、独立积分器）
// 生成的参考轨迹比对。

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"



namespace {

constexpr float kThreshold = 0.02F;   // PRD 8.5: normalised RMSE <= 2 %
constexpr float kTargetRadS = 100.0F; // the S2 step the reference was generated for
constexpr float kHoldSeconds = 0.5F;
constexpr std::size_t kWarmupSamples = 40U; // 2 ms of 0 rad/s before the step, as in the arbiter
constexpr std::size_t kDecimation = 5U;     // the arbiter stores every fifth control period

struct Row final {
    float t_s;
    float speed;
    float id_a;
    float iq_a;
};

auto load_csv(const std::string& path) -> std::vector<Row> {
    std::vector<Row> rows;
    std::ifstream in(path);
    if (!in) {
        return rows;
    }

    std::string line;
    std::getline(in, line); // header
    while (std::getline(in, line)) {
        if (line.empty()) {
            continue;
        }
        std::stringstream ss(line);
        std::string field;
        Row row{0.0F, 0.0F, 0.0F, 0.0F};
        if (std::getline(ss, field, ',')) {
            row.t_s = std::stof(field);
        }
        if (std::getline(ss, field, ',')) {
            row.speed = std::stof(field);
        }
        if (std::getline(ss, field, ',')) {
            row.id_a = std::stof(field);
        }
        if (std::getline(ss, field, ',')) {
            row.iq_a = std::stof(field);
        }
        rows.push_back(row);
    }
    return rows;
}



/// [en] Largest absolute value of a channel. [zh] 通道绝对值的最大值。
auto peak_of(const std::vector<float>& y) -> float {
    float peak = 0.0F;
    for (const float v : y) {
        const float magnitude = std::fabs(v);
        peak = (magnitude > peak) ? magnitude : peak;
    }
    return peak;
}



/// [en] Normalised root-mean-square deviation: RMS(error) / scale.
/// [zh] 归一化均方根偏差：RMS(误差) / 归一化尺度。
auto normalised_rmse(const std::vector<float>& actual, const std::vector<float>& reference, float scale) -> float {
    if (actual.size() != reference.size() || actual.empty() || !(scale > 0.0F)) {
        return 1e9F;
    }
    float sum = 0.0F;
    for (std::size_t i = 0U; i < actual.size(); ++i) {
        const float error = actual[i] - reference[i];
        sum += error * error;
    }
    return std::sqrt(sum / static_cast<float>(actual.size())) / scale;
}

} // namespace



TEST(SilL2Reference, SpeedStepMatchesTheOfflineArbiter) {
#ifndef RESOURCES_PATH
    GTEST_SKIP() << "RESOURCES_PATH is not defined in this build";
#else
    const std::vector<Row> reference = load_csv(std::string(RESOURCES_PATH) + "/ref_s2_speed.csv");
    ASSERT_GT(reference.size(), kWarmupSamples + 100U) << "missing or truncated ref_s2_speed.csv";

    // Replay the very same run the arbiter models: 2 ms at standstill, then the step.
    foc::sil::Bench bench(foc::sil::standard_bench_config());
    bench.motor().set_speed(0.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());

    std::vector<foc::sil::Sample> trace;
    bench.run(kWarmupSamples * bench.config().dt_ctrl_s,
              [&trace](const foc::sil::Bench& b) { trace.push_back(b.sample()); });
    bench.motor().set_speed(kTargetRadS);
    bench.run(kHoldSeconds, [&trace](const foc::sil::Bench& b) { trace.push_back(b.sample()); });

    // Both traces start at power-on and advance by one control period per row, so row i is tick i.
    const std::size_t count =
        (reference.size() * kDecimation < trace.size()) ? reference.size() : (trace.size() / kDecimation);
    ASSERT_GT(count, kWarmupSamples + 100U);
    std::vector<float> ref_speed;
    std::vector<float> ref_id;
    std::vector<float> ref_iq;
    std::vector<foc::sil::Sample> aligned;
    for (std::size_t i = 0U; i < count; ++i) {
        ref_speed.push_back(reference[i].speed);
        ref_id.push_back(reference[i].id_a);
        ref_iq.push_back(reference[i].iq_a);
        aligned.push_back(trace[(i * kDecimation) + (kDecimation - 1U)]);
    }
    const std::vector<float> speed = foc::sil::series_of(aligned, &foc::sil::Sample::speed);
    const std::vector<float> id = foc::sil::series_of(aligned, &foc::sil::Sample::id_a);
    const std::vector<float> iq = foc::sil::series_of(aligned, &foc::sil::Sample::iq_a);

    // Speed is normalised by its own peak; currents by the nameplate limit, because their
    // reference is near zero at standstill and a self-normalised metric would explode there.
    const float rmse_speed = normalised_rmse(speed, ref_speed, peak_of(ref_speed));
    const float rmse_id = normalised_rmse(id, ref_id, foc::sil::nominal_spec().i_max_a);
    const float rmse_iq = normalised_rmse(iq, ref_iq, foc::sil::nominal_spec().i_max_a);
    std::printf("[l2] normalised RMSE speed=%.4f%% id=%.4f%% iq=%.4f%% (limit %.1f%%)\n",
                static_cast<double>(100.0F * rmse_speed), static_cast<double>(100.0F * rmse_id),
                static_cast<double>(100.0F * rmse_iq), static_cast<double>(100.0F * kThreshold));

    EXPECT_LE(rmse_speed, kThreshold) << "speed";
    EXPECT_LE(rmse_id, kThreshold) << "id";
    EXPECT_LE(rmse_iq, kThreshold) << "iq";
#endif
}



TEST(SilL2Reference, EndsWhereTheArbiterEnds) {
#ifndef RESOURCES_PATH
    GTEST_SKIP() << "RESOURCES_PATH is not defined in this build";
#else
    const std::vector<Row> reference = load_csv(std::string(RESOURCES_PATH) + "/ref_s2_speed.csv");
    ASSERT_FALSE(reference.empty());

    const Row& last = reference.back();
    // The reference settles just above the target because the speed loop is still unwinding the
    // windup of the ramp; the C++ side must land on the same operating point.
    EXPECT_NEAR(last.speed, kTargetRadS, 0.02F * kTargetRadS);
    EXPECT_NEAR(0.0F, last.id_a, 0.01F);
    EXPECT_NEAR(last.iq_a, foc::sil::nominal_spec().b * last.speed / foc::sil::nominal_spec().kt, 0.02F);
#endif
}
