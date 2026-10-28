// Conan::ImportStart
#pragma once
#include <cmath>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Everything a reference generator may look at, filled once per current-loop step.
 *        [zh] 参考生成器可以看的全部输入，每个电流环周期填一次。
 *
 *        [en] `iq_command_a` carries the torque setpoint in torque mode and the speed loop's
 *        output in speed mode: the generator never has to know which mode is active, because by
 *        the time it runs the outer loop has already collapsed the command onto the q axis.
 *        [zh] `iq_command_a` 在转矩模式下是转矩给定，在速度模式下是速度环的输出：参考生成器
 *        不需要知道当前是哪种模式，因为它运行时外环已经把给定归约到 q 轴上了。
 *
 * @context [en] Current-loop ISR, read-only for the policy. [zh] 电流环 ISR；对策略只读。
 * @since 0.1.0
 * @exporter
 */
struct ReferenceInput final {
    float iq_command_a;  ///< [en] q-axis command in A. [zh] q 轴给定，A。
    float omega_e_rad_s; ///< [en] electrical angular speed in rad/s. [zh] 电角速度，rad/s。
    float v_bus_v;       ///< [en] DC link voltage in V, this step. [zh] 本周期母线电压，V。
    Dq<float> i_meas;    ///< [en] measured dq current in A. [zh] 实测 dq 电流，A。
};


/**
 * @brief [en] The default reference generator: d axis held at zero, q axis passed through.
 *        [zh] 默认参考生成器：d 轴恒为 0，q 轴直通。
 *
 *        [en] "d = 0" is optimal for a surface-mounted machine and nothing more: an interior-PM
 *        machine earns extra torque from a negative d current (MTPA), and any machine needs d
 *        current to run past its base speed (field weakening). That is why the split is a policy
 *        with an input struct, rather than a `0.0F` written into the current loop.
 *        [zh] “d = 0”只对表贴式电机最优，仅此而已：内嵌式电机靠负 d 轴电流拿到额外转矩（MTPA），
 *        任何电机超过基速后都要靠 d 轴电流做弱磁。所以这个“拆分”是一个带输入结构的策略，而不是
 *        写死在电流环里的 `0.0F`。
 *
 * @note [en] A reference generator must be default constructible: the five-policy aggregate form
 *       leaves this member defaulted, which is what keeps existing assembly points source-compatible.
 *       [zh] 参考生成器必须可默认构造：五元聚合初始化形式会把该成员留作默认值，现有装配点因此保持
 *       源码兼容。
 * @context [en] Current-loop ISR: noexcept, no allocation. [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class NullReferenceGenerator final {
  public:
    /**
     * @brief [en] Split the q-axis command into the dq reference.
     *        [zh] 把 q 轴给定拆分成 dq 参考。
     *
     * @param in [en] inputs of this control step. [zh] 本控制步的输入。
     * @return [en] the dq reference in A; `Motor` clamps it to the nameplate and drops non-finite
     *         values before it reaches the current loop.
     *         [zh] dq 参考，A；`Motor` 在送入电流环之前按铭牌限幅并丢弃非有限值。
     * @since 0.1.0
     */
    [[nodiscard]] auto generate(const ReferenceInput& in) noexcept -> Dq<float> {
        return Dq<float>{0.0F, in.iq_command_a};
    }
};

/**
 * @brief [en] MTPA: the d-axis current that gets the most torque per amp.
 *        [zh] MTPA：每安培转矩最大的那个 d 轴电流。
 *
 *        [en] Why a policy and not a constant. For a surface-mounted machine `i_d = 0` *is* MTPA, so the whole saliency
 *        story looks like a detail. For an interior-PM machine it is not a detail: the reluctance torque is worth a
 *        third of the total, and every amp spent on the wrong `i_d` is an amp not making torque. The gap between the
 *        two is the difference between needing a bigger drive and not.
 *        [zh] 为什么是策略而不是常数。表贴机上 `i_d = 0` **就是** MTPA，于是整个凸极性
 *        故事看起来像个细节。内嵌机上它不是细节：磁阻转矩占总量的三分之一，花在错误 `i_d` 上的每
 *        一安培都是没在做转矩的安培。两者之差就是"需要换更大驱动器"与"不需要"之差。
 *
 *        [en] The curve. Setting `dT/di_d = 0` on `T = 1.5*p*(psi_f*i_q + (Ld-Lq)*i_d*i_q)` subject to `i_d^2+i_q^2 =
 *        |i|^2` gives ``` i_d = (psi_f - sqrt(psi_f^2 + 8*(Lq-Ld)^2*i_q^2)) / (4*(Lq-Ld)) ``` which is negative for `Lq
 *        > Ld` and degenerate at `Lq = Ld` -- the degenerate case is handled as `i_d = 0` rather than by a tiny
 *        epsilon, because an epsilon would produce an `i_d` of order `psi_f/eps` and destroy the loop.
 *        [zh] 曲线。在 `i_d²+i_q²=|i|²` 约束下对 `T = 1.5·p·(ψf·iq + (Ld−Lq)·id·iq)` 令 `dT/di_d = 0`，得上式；`Lq >
 *        Ld` 时为负， `Lq = Ld` 时退化——退化情形按 `i_d = 0` 处理而不是用一个极小量，因为极小量会给出量级 `ψf/eps` 的
 *        `i_d`，把环直接毁掉。
 *
 * @context [en] Current-loop ISR, on the reference-generation path. [zh]
 * 电流环 ISR，位于参考生成路径上。
 * @since 0.1.0
 * @exporter
 */
class MtpaReference final {
  public:
    /**
     * @brief [en] The machine. [zh] 机器。
     * @since 0.1.0
     */
    struct Config final {
        float ld;      ///< [en] d-axis inductance, H. [zh] d 轴电感，H。
        float lq;      ///< [en] q-axis inductance, H. [zh] q 轴电感，H。
        float psi_f;   ///< [en] rotor flux linkage, Wb. [zh] 转子磁链，Wb。
        float i_max_a; ///< [en] per-axis current limit, A. [zh] 每轴电流限幅，A。
    };

    MtpaReference() noexcept = default;

    explicit MtpaReference(const Config& config) noexcept : config_(config) {}

    /// [en] Swap in a new machine without reconstructing. [zh] 不必重建就能换机器。
    void set_config(const Config& config) noexcept { config_ = config; }

    /**
     * @brief [en] The dq current reference for this step. [zh] 本步的 dq 电流参考。
     *
     * @param in [en] q command and the rest (unused here). [zh] q 给定及其余项（此处不用）。
     * @return [en] `{i_d, i_q}` in A, with `|i| <= i_max_a`. [zh]
     * `{i_d, i_q}`，A，且 `|i| <= i_max_a`。
     *
     * @note [en] When the request does not fit, the answer is **not** to scale
     * both components: that leaves the MTPA curve and gives up more torque than
     * necessary. The answer is to walk *down the curve* until the magnitude fits,
     * which keeps "most torque per amp" true at every point including the clamp.
     * The walk is a bisection on `i_q` because magnitude is monotone in `i_q` along
     * the curve, and 24 halvings is the same budget the field-weakening solver
     * uses.
     * [zh] 装不下时**不是**把两个分量一起缩小：那会离开 MTPA 曲线，多丢了本可以拿到的转矩。
     * 正确做法是**沿曲线往下走**直到幅值装得下，这样"每安培转矩最大"在包括限幅点的每一点都
     * 仍成立。走法是对 `i_q` 二分，因为沿曲线幅值对 `i_q` 单调，而 24 次折半正是弱磁求解器
     * 用的同一预算。
     * @since 0.1.0
     */
    [[nodiscard]] auto generate(const ReferenceInput& in) const noexcept -> Dq<float> {
        const float iq_request = in.iq_command_a;
        const float sign = (iq_request < 0.0F) ? -1.0F : 1.0F;
        const float iq_abs = std::fabs(iq_request);
        const float limit = (config_.i_max_a > 0.0F) ? config_.i_max_a : 0.0F;
        if (!(limit > 0.0F)) {
            return Dq<float>{0.0F, 0.0F};
        }
        // The unconstrained answer first; it is the common case.
        // 先给无约束的答案；那才是常见情形。
        if (magnitude(iq_abs) <= limit) {
            return Dq<float>{d_axis(iq_abs), sign * iq_abs};
        }
        // Then walk down the curve. `hi` is known not to fit, `lo` always fits.
        // 再沿曲线往下走。`hi` 已知装不下，`lo` 一定装得下。
        float lo = 0.0F;
        float hi = iq_abs;
        for (int k = 0; k < 24; ++k) {
            const float mid = static_cast<float>(0.5F * (lo + hi));
            if (magnitude(mid) <= limit) {
                lo = mid;
            } else {
                hi = mid;
            }
        }
        return Dq<float>{d_axis(lo), sign * lo};
    }

    /// [en] The config in force. [zh] 当前配置。
    [[nodiscard]] auto config() const noexcept -> const Config& { return config_; }

  private:
    /// [en] `i_d` on the MTPA curve at `|i_q|`, A. [zh] MTPA 曲线上 `|i_q|` 处的 `i_d`，A。
    [[nodiscard]] auto d_axis(float iq_abs) const noexcept -> float {
        const float saliency = config_.lq - config_.ld;
        if (!(std::fabs(saliency) > 1.0e-9F)) {
            // Surface-mounted: `i_d = 0` is MTPA, and an epsilon here would
            // divide by nothing meaningful.
            // 表贴式：`i_d = 0` 就是 MTPA，这里的极小量只会除一个没有意义的数。
            return 0.0F;
        }
        const float root =
            std::sqrt((config_.psi_f * config_.psi_f) + (((8.0F * saliency) * saliency) * (iq_abs * iq_abs)));
        return (config_.psi_f - root) / (4.0F * saliency);
    }

    /// [en] `|(i_d, i_q)|` on the curve, A. [zh] 曲线上 `|(i_d, i_q)|`，A。
    [[nodiscard]] auto magnitude(float iq_abs) const noexcept -> float {
        const float d = d_axis(iq_abs);
        return std::sqrt((d * d) + (iq_abs * iq_abs));
    }

    Config config_{};
};

} // namespace core

} // namespace foc