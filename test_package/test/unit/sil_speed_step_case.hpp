// Conan::ImportStart
#pragma once

#include <vector>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"
// Conan::ImportEnd

namespace foc {

namespace sil {



/**
 * @brief [en] Outcome of one S2 speed-step run, in the terms of PRD 8.4.
 *        [zh] 一次 S2 速度阶跃的结果，按 PRD 8.4 的口径给出。
 *
 * @since 0.1.0
 */
struct SpeedStepResult final {
    float overshoot;             ///< [en] overshoot as a fraction of the step. [zh] 超调占阶跃的比例。
    float steady_error_fraction; ///< [en] steady-state error over the target. [zh] 稳态误差占目标的比例。
    float tail_peak_to_peak;     ///< [en] ripple over the last 20 %. [zh] 末 20% 的峰峰值。
    float settling_s;            ///< [en] 2 % settling time, -1 when never. [zh] 2% 调节时间，未收敛为 -1。
    float speed_final;           ///< [en] tail mean of the speed. [zh] 速度末段均值。
    float peak_iq;               ///< [en] largest |iq| over the run. [zh] 全程 |iq| 最大值。
};



/**
 * @brief [en] Run the S2 scenario: hold 0 rad/s, then step to `target_rad_s`.
 *        [zh] 运行 S2 场景：先稳住 0 rad/s，再阶跃到 `target_rad_s`。
 *
 * @param plant [en] plant parameters under test (S5 passes perturbed ones). [zh] 被控 plant 参数（S5 传入摄动值）。
 * @param target_rad_s [en] speed reference in rad/s. [zh] 速度给定，rad/s。
 * @param hold_s [en] how long to run after the step. [zh] 阶跃后运行时长。
 * @since 0.1.0
 */
inline auto run_speed_step(const PlantParams& plant, float target_rad_s, float hold_s = 0.5F) -> SpeedStepResult {
    Bench bench(standard_bench_config(), plant);
    bench.motor().set_speed(0.0F);
    static_cast<void>(bench.init_and_start());
    bench.run(2e-3F); // let the current loop settle at zero speed

    bench.motor().set_speed(target_rad_s);
    std::vector<Sample> trace;
    trace.reserve(static_cast<std::size_t>(hold_s / bench.config().dt_ctrl_s) + 1U);
    bench.run(hold_s, [&trace](const Bench& b) { trace.push_back(b.sample()); });

    const std::vector<float> speed = series_of(trace, &Sample::speed);
    const float magnitude = (target_rad_s > 0.0F) ? target_rad_s : -target_rad_s;
    const bool forward = target_rad_s > 0.0F;

    // Mirror a negative step so the same "fraction of the step" metric applies to both.
    std::vector<float> normalised;
    normalised.reserve(speed.size());
    float peak_iq = 0.0F;
    for (std::size_t i = 0U; i < speed.size(); ++i) {
        normalised.push_back(forward ? speed[i] : -speed[i]);
        const float current = trace[i].iq_a;
        const float current_abs = (current > 0.0F) ? current : -current;
        peak_iq = (current_abs > peak_iq) ? current_abs : peak_iq;
    }

    SpeedStepResult r{};
    r.speed_final = tail_mean(speed, 0.1F);
    r.overshoot = overshoot_fraction(normalised, magnitude, 0.0F);
    const float error =
        (r.speed_final > target_rad_s) ? (r.speed_final - target_rad_s) : (target_rad_s - r.speed_final);
    r.steady_error_fraction = error / magnitude;
    r.tail_peak_to_peak = tail_peak_to_peak(speed, 0.2F);
    r.settling_s = settling_time_s(trace, &Sample::speed, target_rad_s, 0.02F * magnitude);
    r.peak_iq = peak_iq;
    return r;
}

} // namespace sil

} // namespace foc
