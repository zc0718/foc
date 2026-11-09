// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file auto_tune.hpp
 *
 * @brief [en] The speed-loop tuner's contract: what it is asked for, what it reports, and what it is allowed to trust.
 * @brief [zh] 速度环整定器的契约：被要求什么、回报什么，以及它被允许相信什么。
 * @since 0.1.0
 */

namespace foc {

namespace core {

/**
 * @brief [en] What a speed-loop tuning pass is asked for, and what it is allowed to trust.
 *
 *   **The two numbers a speed loop actually needs are both measurable ratios**, and neither is `kt` or `J` on its own.
 *   From `J*dw/dt = kt*iq - b*w` the acceleration per amp is `kt/J` and the mechanical pole is `b/J`. The library's own
 *   `speed_pi_config()` cancels that pole and lands at `kp = bandwidth*J/kt`, `ki = bandwidth*b/kt` -- which, written
 *   in the ratios, is `kp = bandwidth/(kt/J)` and `ki = bandwidth*(b/J)/(kt/J)`. So a tuner never has to know `kt` or
 *   `J` to produce the right gains; it has to *measure the two ratios*. The nameplate enters only through `j_expected`,
 *   whose job is to answer a different question: is this the machine the caller thinks it is tuning?
 *
 * @brief [zh] 一次速度环整定被要求什么，以及它被允许相信什么。
 *
 *   **速度环真正需要的两个数都是可测的比值** ，而且都不是单独的 `kt` 或 `J`。由 `J·dω_m/dt = kt·iq − b·ω_m` 得，每安培
 *   加速度是 `kt/J`、机械极点是 `b/J`。库自己的 `speed_pi_config()` 抵消那个极点后落在 `kp = bandwidth·J/kt`、
 *   `ki = bandwidth·b/kt` ——写成比值就是 `kp = bandwidth/(kt/J)` 与 `ki = bandwidth·(b/J)/(kt/J)`。所以整定器根本不需要
 *   知道 `kt` 或 `J` 才能给出正确的增益；它需要 **量出这两个比值** 。铭牌只通过 `j_expected` 参与，而它的职责是回答另一
 *   个问题：这台机器是不是调用方以为自己在整定的那一台？
 * @since 0.1.0
 * @exporter
 */
struct SpeedTuneConfig final {
    float kt_nm_per_a{/**
         * @brief [en] nameplate torque constant, used only for the fallback gains and `J`.
         * @brief [zh] 铭牌转矩常数，只用于回退增益与 `J`。
         */
                      0.0F};
    float j_expected_kg_m2{/**
         * @brief [en] the inertia the caller believes in; `valid` is decided against it.
         * @brief [zh] 调用方相信的惯量； `valid` 对着它判定。
         */
                           0.0F};
    /**
     * @brief [en] the viscous coefficient the caller believes in.
     * @brief [zh] 调用方相信的粘性系数。
     */
    float b_expected_nm_s{0.0F};
    /**
     * @brief [en] requested closed-loop bandwidth, rad/s.
     * @brief [zh] 请求的闭环带宽，rad/s。
     */
    float bandwidth_rad_s{0.0F};
    /**
     * @brief [en] current ceiling for the produced `Pi` config, A.
     * @brief [zh] 产出的 `Pi` 配置的电流上限，A。
     */
    float i_max_a{0.0F};
    float tolerance{/**
         * @brief [en] how far the measured inertia may sit from `j_expected`, relative.
         * @brief [zh] 实测惯量与 `j_expected` 允许的相对偏差。
         */
                    0.25F};
    float min_current_a{/**
         * @brief [en] below this peak current the window carries no information.
         * @brief [zh] 峰值电流低于此值，窗口不携带信息。
         */
                        0.05F};
    float max_residual{/**
         * @brief [en] largest normalised fit residual that is still called a measurement.
         * @brief [zh] 仍算测量的最大归一化拟合残差。
         */
                       0.35F};
    /**
     * @brief [en] shortest window that can be called one.
     * @brief [zh] 能称为窗口的最短长度。
     */
    int min_samples{100};
};

/**
 * @brief [en] A tuning pass's answer, including whether it is an answer at all.
 * @brief [zh] 一次整定的答案，包括它到底算不算答案。
 * @note [en] When `valid` is false the gains in this struct are the **nameplate** ones -- the same values
 *            `speed_pi_config()` would have produced -- so a caller that applies whatever it gets applies nothing new.
 *            That is deliberate: a fallback of zero would be a drive that cannot produce torque, and a fallback of the
 *            measured value would be a silent re-tune to a machine nobody described.
 * @note [zh] `valid` 为假时，本结构里的增益是 **铭牌** 增益——即 `speed_pi_config()` 会给的同一组值——于是"拿到什么就用什
 *            么 "的调用方等于什么都没换。这是刻意的：回退成零会得到一个出不了力矩的驱动器，回退成实测值则是悄悄整定到一
 *            台没人描述过的机器上。
 * @since 0.1.0
 * @exporter
 */
struct SpeedTuneResult final {
    ErrorCode
        /**
         * @brief [en] `Ok` when the configuration could be evaluated, `Param` when it could not.
         * @brief [zh] 配置能评估时为 `Ok`，否则 `Param`。
         */
        status;
    bool
        /**
         * @brief [en] whether the measurement is a measurement of *this* machine.
         * @brief [zh] 这次测量是否是对 **这台** 机器的测量。
         */
        valid;
    /**
     * @brief [en] identified acceleration per amp, rad/s^2 per A.
     * @brief [zh] 辨识出的每安培加速度，rad/s²/A。
     */
    float kt_over_j;
    /**
     * @brief [en] identified mechanical pole, 1/s.
     * @brief [zh] 辨识出的机械极点，1/s。
     */
    float b_over_j;
    float
        /**
         * @brief [en] `kt / (kt/J)` from the nameplate `kt`, or 0 when `kt` was not supplied.
         * @brief [zh] 由铭牌 `kt` 得出的 `kt/(kt/J)`；没给 `kt` 时为 0。
         */
        j_kg_m2;
    /**
     * @brief [en] proportional gain to apply.
     * @brief [zh] 要施加的比例增益。
     */
    float kp_a_per_rad_s;
    /**
     * @brief [en] integral gain to apply.
     * @brief [zh] 要施加的积分增益。
     */
    float ki_a_per_rad_s2;
    float
        /**
         * @brief [en] normalised RMS residual of the fit, 0 = the model explains the window exactly.
         * @brief [zh] 拟合的归一化 RMS 残差，0 表示模型完全解释了该窗口。
         */
        residual;
    const char*
        /**
         * @brief [en] why `valid` is false, for a log; empty when it is true.
         * @brief [zh] `valid` 为假的原因，用于日志；为真时为空串。
         */
        reason;
    /**
     * @brief [en] how many periods were judged.
     * @brief [zh] 参与判定的周期数。
     */
    int samples;
};

/**
 * @brief [en] Identify the two ratios a speed loop needs from a torque-step window.
 *
 *   And refuse when the window is not a measurement.
 *
 *   **The fit.** `dw/dt = (kt/J)*iq - (b/J)*w` is linear in `(iq, w)`, so a two-parameter least squares over the window
 *   gives both ratios without ever assuming the speed held still: `kt/J` is the slope on `iq` and `b/J` is the slope on
 *   `w`, and the fit carries an **intercept** on purpose -- an unknown constant load lands there, which is how the pass
 *   can tell "the machine is heavier than you said" from "something is pulling on it".
 *
 *   **Refusals, each with its own reason.** Too short a window, a current that did not vary (no slope to measure), a
 *   saturated sample (the loop was not delivering what the reference asked), a residual too large (the model does not
 *   explain the window -- friction that changes with position, a cogging detent, a load that moves), a negative or
 *   implausible mechanical pole, and a measured inertia further than `tolerance` from `j_expected`. In every one of
 *   those the result carries the **nameplate** gains and `valid == false`, so a caller that applies what it gets
 *   applies nothing new.
 *
 * @brief [zh] 从一个转矩阶跃窗口辨识速度环需要的两个比值，并在窗口不构成测量时拒绝。
 *
 *   **拟合。** `dω_m/dt = (kt/J)·iq − (b/J)·ω_m` 对 `(iq, ω_m)` 是线性的，所以窗口上的两参数最小二乘同时给出两个比值，
 *   而不需要假设转速不变： `kt/J` 是对 `iq` 的斜率， `b/J` 是对 `ω_m` 的斜率。拟合 **刻意带截距** ——一个未知的恒定负载
 *   会落在那里，这是本次整定能区分"机器比你说的重"与"有东西在拉它"的方式。
 *
 *   **拒绝，各有理由。** 窗口太短、电流没有变化（没有斜率可测）、出现饱和样本（环路没在送出参考要的东西）、残差过大（模
 *   型解释不了这个窗口——随位置变化的摩擦、齿槽定位点、在动的负载）、机械极点不成立或为负、实测惯量离 `j_expected` 超过
 *   `tolerance`。以上每一种，结果里带的都是 **铭牌** 增益且 `valid == false`，于是"拿到什么就用什么"的调用方等于什么都
 *   没换。
 * @since 0.1.0
 * @exporter
 */
class SpeedTuner final {
  public:
    SpeedTuner() noexcept = default;

    explicit SpeedTuner(const SpeedTuneConfig& config) noexcept : config_(config) {}

    /**
     * @brief [en] Whether the configuration could ever produce an answer.
     * @brief [zh] 该配置是否有可能给出答案。
     */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode;

    /**
     * @brief [en] Throw away the window.
     * @brief [zh] 丢弃当前窗口。
     */
    void reset() noexcept;

    /**
     * @brief [en] Feed one control period of the torque-step window.
     * @brief [zh] 喂入转矩阶跃窗口的一个控制周期。
     * @param dt_s [en] period, s.
     * @param dt_s [zh] 周期，s。
     * @param iq_a [en] measured q-axis current, A.
     * @param iq_a [zh] 实测 q 轴电流，A。
     * @param omega_mech_rad_s [en] mechanical speed, rad/s.
     * @param omega_mech_rad_s [zh] 机械转速，rad/s。
     * @param saturated [en] whether the current loop was railing this period.
     * @param saturated [zh] 本周期电流环是否撞限。
     * @since 0.1.0
     */
    void accumulate(float dt_s, float iq_a, float omega_mech_rad_s, bool saturated) noexcept;

    /**
     * @brief [en] Periods accumulated so far.
     * @brief [zh] 目前累积的周期数。
     */
    [[nodiscard]] auto samples() const noexcept -> int;

    /**
     * @brief [en] Turn the window into gains, or into a refusal.
     * @brief [zh] 把窗口变成增益，或变成一个拒绝。
     */
    [[nodiscard]] auto finish() const noexcept -> SpeedTuneResult;

  private:
    [[nodiscard]] auto nameplate_gains() const noexcept -> SpeedTuneResult;

    SpeedTuneConfig config_{};
    double s_ii_{0.0};
    double s_iw_{0.0};
    double s_ww_{0.0};
    double s_ia_{0.0};
    double s_wa_{0.0};
    double s_a_{0.0};
    double s_aa_{0.0};
    double s_i_{0.0};
    double s_w_{0.0};
    float previous_iq_{0.0F};
    float previous_omega_{0.0F};
    bool previous_has_{false};
    float iq_min_{0.0F};
    float iq_max_{0.0F};
    bool saturated_{false};
    int samples_{0};
};

} // namespace core

} // namespace foc
