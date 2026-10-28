// Conan::ImportStart
#pragma once

#include <cstddef>
#include <vector>

#include "sil_bench.hpp"
// Conan::ImportEnd

namespace foc {

namespace sil {



/// [en] Extract one channel of a trace. [zh] 取出轨迹中的一个通道。
inline auto series_of(const std::vector<Sample>& trace, float Sample::* field) -> std::vector<float> {
    std::vector<float> out;
    out.reserve(trace.size());
    for (const Sample& s : trace) {
        out.push_back(s.*field);
    }
    return out;
}



/// [en] Largest value of a channel. [zh] 通道的最大值。
inline auto max_of(const std::vector<float>& y) -> float {
    float v = y.empty() ? 0.0F : y.front();
    for (const float e : y) {
        v = (e > v) ? e : v;
    }
    return v;
}



/// [en] Smallest value of a channel. [zh] 通道的最小值。
inline auto min_of(const std::vector<float>& y) -> float {
    float v = y.empty() ? 0.0F : y.front();
    for (const float e : y) {
        v = (e < v) ? e : v;
    }
    return v;
}



/**
 * @brief [en] Overshoot of a step response as a fraction of the commanded step.
 * @brief [zh] 阶跃响应的超调，按给定阶跃的百分比表示。
 * @param y [en] response channel.
 * @param y [zh] 响应通道。
 * @param target [en] final value.
 * @param target [zh] 终值。
 * @param y0 [en] starting value.
 * @param y0 [zh] 起始值。
 * @return [en] (max - target) / (target - y0), 0 when there is no overshoot.
 * @return [zh] (max - target) / (target - y0)；无超调时为 0。
 * @since 0.1.0
 */
inline auto overshoot_fraction(const std::vector<float>& y, float target, float y0) -> float {
    const float step = target - y0;
    if (!(step > 0.0F)) {
        return 0.0F;
    }
    const float over = max_of(y) - target;
    return (over > 0.0F) ? (over / step) : 0.0F;
}



/// [en] Highest absolute deviation from a value. [zh] 相对某值的最大绝对偏差。
inline auto max_abs_error(const std::vector<float>& y, float target) -> float {
    float v = 0.0F;
    for (const float e : y) {
        const float d = (e > target) ? (e - target) : (target - e);
        v = (d > v) ? d : v;
    }
    return v;
}



/// [en] Mean of the last `fraction` of a channel. [zh] 通道末尾 `fraction` 段的均值。
inline auto tail_mean(const std::vector<float>& y, float fraction) -> float {
    if (y.empty()) {
        return 0.0F;
    }
    std::size_t count = static_cast<std::size_t>(static_cast<float>(y.size()) * fraction);
    if (count == 0U) {
        count = 1U;
    }
    const std::size_t begin = y.size() - count;

    float sum = 0.0F;
    for (std::size_t i = begin; i < y.size(); ++i) {
        sum += y[i];
    }
    return sum / static_cast<float>(count);
}



/// [en] Peak-to-peak of the last `fraction` of a channel. [zh] 通道末尾 `fraction` 段的峰峰值。
inline auto tail_peak_to_peak(const std::vector<float>& y, float fraction) -> float {
    if (y.empty()) {
        return 0.0F;
    }
    std::size_t count = static_cast<std::size_t>(static_cast<float>(y.size()) * fraction);
    if (count == 0U) {
        count = 1U;
    }
    const std::size_t begin = y.size() - count;

    float hi = y[begin];
    float lo = y[begin];
    for (std::size_t i = begin; i < y.size(); ++i) {
        hi = (y[i] > hi) ? y[i] : hi;
        lo = (y[i] < lo) ? y[i] : lo;
    }
    return hi - lo;
}



/**
 * @brief [en] Time after which the response stays inside `band` of the target for good.
 * @brief [zh] 响应此后一直落在目标 ± `band` 之内的时刻。
 * @return [en] seconds relative to the first sample, or -1 when it never settles.
 * @return [zh] 相对首个样本的时间，秒；始终未进入带内返回 -1。
 * @since 0.1.0
 */
inline auto settling_time_s(const std::vector<Sample>& trace, float Sample::* field, float target, float band)
    -> float {
    const std::vector<float> y = series_of(trace, field);
    if (y.empty()) {
        return -1.0F;
    }

    // Walk backwards to the last sample outside the band: O(n) instead of O(n^2).
    std::size_t last_outside = y.size();
    for (std::size_t i = y.size(); i > 0U; --i) {
        const float d = (y[i - 1U] > target) ? (y[i - 1U] - target) : (target - y[i - 1U]);
        if (d > band) {
            last_outside = i - 1U;
            break;
        }
    }

    if (last_outside == y.size()) {
        return 0.0F;
    }
    if (last_outside + 1U >= y.size()) {
        return -1.0F;
    }
    return trace[last_outside + 1U].t_s - trace.front().t_s;
}

} // namespace sil

} // namespace foc
