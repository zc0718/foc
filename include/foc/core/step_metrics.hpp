// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstddef>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] What a recorded step response actually did, in numbers a tuning decision can rest on.
 *
 * @brief [zh] 一段记录下来的阶跃响应到底做了什么，用可直接据以整定的数说出来。
 *
 *     Exists so the loop can measure itself instead of asking a human to eyeball a plot. The two quantities that carry
 *     a tuning decision are `bandwidth_rad_s` -- the -3 dB point of the first-order model fitted to the approach, which
 *     the gain law in `motor_config.hpp` claims to place at the requested bandwidth -- and `settling_s`, which is the
 *     claim a customer actually makes ("it holds within 2 % in 70 ms").
 *
 *     `status` is the diagnostic channel, and it is not decoration: `Ok` means a decay was fitted, `State` means the
 *     window carried no usable decay (fewer than three samples strictly between the start and the target), `Param`
 *     means the arguments cannot describe a step at all. A bandwidth of 0 with status `Ok` is impossible by
 *     construction, so "zero" cannot be mistaken for "not measured" -- which is the failure mode the capability matrix
 *     exists to prevent.
 *
 *     存在的意义是让环路自己测自己，而不是让人去看图。承载整定决策的两个量是 `bandwidth_rad_s`——拟合到逼近段的一阶模型
 *     的 -3 dB 点，`motor_config.hpp` 的增益律声称把它放在请求的带宽上——和 `settling_s`，后者正是客户真正会说的话（"70
 *     ms 内稳定在 2% 带内"）。
 *
 *     `status` 是诊断通道，不是装饰：`Ok` 表示拟合到了衰减，`State` 表示窗口里没有可用的衰减（严格落在起点与目标之间的
 *     样本少于三个），`Param` 表示参数根本描述不了一个阶跃。构造上不可能出现 `status == Ok` 而带宽为 0，因此"零"不会被
 *     误读成"没测"——而这正是能力矩阵存在的理由。
 * @since 0.1.8
 * @exporter
 */
struct StepMetrics {
    ErrorCode status; ///< [en] Ok / State (no usable decay) / Param. [zh] Ok / State（无可用衰减）/ Param。
    /**
     * @brief [en] -3 dB of the fitted first-order loop, rad/s; 0 unless status is Ok.
     * @brief [zh] 拟合一阶环路的 -3 dB 点，rad/s；仅当 status 为 Ok 时非零。
     */
    float bandwidth_rad_s;
    float tau_s; ///< [en] fitted time constant, s; `1 / bandwidth_rad_s`. [zh] 拟合时间常数，s。
    /**
     * @brief [en] seconds from the first sample until the response stays inside the band; -1 when it never does, 0 when
     *             it starts inside.
     * @brief [zh] 从首个样本起，响应此后一直落在带内所需时间；永不进入为 -1，一开始就在带内为 0。
     */
    float settling_s;
    /**
     * @brief [en] peak beyond the target, as a fraction of the step; 0 when it never crosses.
     * @brief [zh] 峰值越过目标的部分，以阶跃幅度为单位；不越过为 0。
     */
    float overshoot_fraction;
    /**
     * @brief [en] mean of the last tenth minus the target, as a fraction of the step.
     * @brief [zh] 末尾十分之一的均值减目标，以阶跃幅度为单位。
     */
    float tail_error_fraction;
};


/**
 * @brief [en] Measure a recorded step response: fit a first-order decay to the approach and report the loop's numbers.
 *
 * @brief [zh] 测量一段记录下来的阶跃响应：对逼近段拟合一个一阶衰减，并给出环路的几个数。
 *
 *     The window must start at (or before) the step: `y[0]` is taken as the pre-step value and `target` as the
 *     post-step one, so a window that begins mid-transient measures the transient, not the step. That is a documented
 *     precondition rather than a guess about the caller's data.
 *
 *     The fit uses the **approach segment only**: every sample from the first movement up to the first arrival at the
 *     target, and nothing after it. An overshoot above the target would make `ln` of a non-positive number, and the
 *     ring-down past the crossing is a shape the first-order model does not describe -- including it pulls the time
 *     constant toward the oscillation period. A response that never gets closer than 10 % of the way to the target
 *     therefore reports `State` instead of a confidently wrong number, and a response that overshoots is measured on
 *     the part that is actually first-order.
 *
 *     `bandwidth_rad_s = 1 / tau`. For the speed loop in this library that is the whole claim: `speed_pi_config()`
 *     cancels the mechanical pole `B/J` and leaves `wb / s`, a first-order loop with time constant `1 / wb`, whose -3
 *     dB point is `wb`. Fitting `tau` from the trace and comparing it with the requested `wb` is therefore a
 *     measurement of whether the gain law did what its documentation says.
 *
 *     窗口必须从阶跃之前（或正好在阶跃处）开始：`y[0]` 被当作阶跃前的值、`target` 被当作阶跃后的值，因此从暂态中间开始
 *     的窗口测到的是暂态，不是阶跃。这是写下来的前提条件，不是对调用方数据的猜测。
 *
 *     拟合只取**逼近段**：从第一次移动起，到第一次到达目标为止，之后的一个都不用。越过目标的超调会让 `ln` 作用在非正数
 *     上；而越过之后的振荡下降是一阶模型描述不了的形状——把它纳入会把时间常数拉向振荡周期。"始终没能逼近目标 10% 以内"的
 *     响应因此报 `State` 而不是一个自信的错误数字，而有过冲的响应则在真正属于一阶的那一段上被测量。
 *
 *     `bandwidth_rad_s = 1 / tau`。对本库的速度环而言这就是那个断言的全部：`speed_pi_config()` 抵消机械极点 `B/J` 后留
 *     下 `wb / s`，即时间常数为 `1 / wb` 的一阶环，其 -3 dB 点就是 `wb`。因此从轨迹里拟合 `tau` 并与请求的 `wb` 比较，
 *     就是在测量增益律是否做到了它文档里说的事。
 *
 * @param y [en] response samples, oldest first, SI units; at least three.
 * @param y [zh] 响应样本，最早的在前，SI 单位；至少三个。
 * @param n [en] sample count.
 * @param n [zh] 样本数。
 * @param dt_s [en] spacing between samples, s (> 0).
 * @param dt_s [zh] 相邻样本的间隔，s（> 0）。
 * @param target [en] the commanded value the response is approaching.
 * @param target [zh] 响应正在逼近的目标值。
 * @param band [en] half-width of the settling band, in the same unit as `y` (>= 0).
 * @param band [zh] 稳定带半宽，与 `y` 同单位（>= 0）。
 * @return [en] the metrics; every field is written even when status is not Ok.
 * @return [zh] 指标；即使 status 不是 Ok，每个字段也被写入。
 * @context [en] Host-side analysis over a recorded window: noexcept, no allocation, O(n), no function pointer.
 * @context [zh] 对录制窗口的主机侧分析：noexcept、不分配、O(n)、无函数指针。
 * @since 0.1.8
 * @exporter
 */
inline auto step_metrics(const float* y, std::size_t n, float dt_s, float target, float band) noexcept -> StepMetrics {
    StepMetrics out{};
    out.status = ErrorCode::Param;
    out.bandwidth_rad_s = 0.0F;
    out.tau_s = 0.0F;
    out.settling_s = 0.0F;
    out.overshoot_fraction = 0.0F;
    out.tail_error_fraction = 0.0F;

    if ((y == nullptr) || (n < 3U) || !(dt_s > 0.0F) || !(band >= 0.0F)) {
        return out;
    }

    const float y0 = y[0];
    const float step = target - y0;
    const float step_abs = (step >= 0.0F) ? step : -step;
    if (!(step_abs > 0.0F)) {
        return out;
    }

    // Settling: the last sample outside the band decides it, so a response that leaves and re-enters is measured by
    // where it ends up rather than by where it first arrived.
    // 稳定时间由"最后一个落在带外的样本"决定，因此离开又重新进入的响应按最终位置算，而不是按第一次到达算。
    std::size_t last_outside = n;
    for (std::size_t i = n; i > 0U; --i) {
        const float d = (y[i - 1U] > target) ? (y[i - 1U] - target) : (target - y[i - 1U]);
        if (d > band) {
            last_outside = i - 1U;
            break;
        }
    }
    if (last_outside == n) {
        out.settling_s = 0.0F;
    } else if ((last_outside + 1U) >= n) {
        out.settling_s = -1.0F;
    } else {
        out.settling_s = static_cast<float>(last_outside + 1U) * dt_s;
    }

    float peak = y[0];
    double tail_sum = 0.0;
    const std::size_t tail_count = (n / 10U) == 0U ? 1U : (n / 10U);
    for (std::size_t i = 0; i < n; ++i) {
        if (y[i] > peak) {
            peak = y[i];
        }
        if (i >= (n - tail_count)) {
            tail_sum += static_cast<double>(y[i]);
        }
    }
    const float beyond = peak - target;
    out.overshoot_fraction = (beyond > 0.0F) ? (beyond / step_abs) : 0.0F;
    const double tail_mean = tail_sum / static_cast<double>(tail_count);
    out.tail_error_fraction =
        static_cast<float>((tail_mean - static_cast<double>(target)) / static_cast<double>(step_abs));

    // First-order fit through the origin: the fraction still to go is w_i = 1 - (y_i - y0)/step = exp(-t_i / tau), so
    // ln(w_i) = -t_i / tau. The least-squares slope through the origin is the mean of `t*ln(w)` over the mean of `t^2`;
    // accumulating in double keeps a 2000-sample window from losing the slope to float rounding. Note it is `w` and not
    // its complement -- fitting ln of the *progress* is the same mistake as fitting a different curve, and it fails
    // quietly: the first draft of this function did exactly that and reported a time constant 1722 times too large.
    // 过原点的一阶拟合：还差多少的比例 w_i = 1 - (y_i - y0)/step = exp(-t_i/tau)，于是 ln(w_i) = -t_i/tau。过原点最小
    // 二乘斜率等于 `t*ln(w)` 的均值除以 `t^2` 的均值；用 double 累加，避免 2000 个样本的窗口把斜率丢在 float 舍入里。
    // 注意是 `w` 而不是它的补——对"进度"取 ln 等于拟合另一条曲线，而且失败得很安静：本函数的第一版正是这样写的，
    // 报出的时间常数大了 1722 倍。
    double num = 0.0;
    double den = 0.0;
    std::size_t used = 0U;
    for (std::size_t i = 1U; i < n; ++i) {
        const double progress = (static_cast<double>(y[i]) - static_cast<double>(y0)) / static_cast<double>(step);
        if (progress <= 0.0) {
            continue; // no movement yet: carries nothing
        }
        if (progress >= 1.0) {
            break; // arrived: everything after this is overshoot or ring-down, not approach
        }
        const double w = 1.0 - progress;
        const double t = static_cast<double>(i) * static_cast<double>(dt_s);
        num += t * std::log(w);
        den += t * t;
        ++used;
    }

    if ((used < 3U) || !(den > 0.0)) {
        out.status = ErrorCode::State;
        return out;
    }

    const double slope = num / den;
    if (!(slope < 0.0)) {
        out.status = ErrorCode::State;
        return out;
    }
    const double tau = -1.0 / slope;
    out.tau_s = static_cast<float>(tau);
    out.bandwidth_rad_s = static_cast<float>(1.0 / tau);
    out.status = ErrorCode::Ok;
    return out;
}


} // namespace core

} // namespace foc
