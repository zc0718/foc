// Conan::ImportStart
#pragma once
#include <cmath>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Everything a reference generator may look at, filled once per current-loop step.
 *
 *     `iq_command_a` carries the torque setpoint in torque mode and the speed loop's output in speed mode: the
 *     generator never has to know which mode is active, because by the time it runs the outer loop has already
 *     collapsed the command onto the q axis.
 *
 * @brief [zh] 参考生成器可以看的全部输入，每个电流环周期填一次。
 *
 *     `iq_command_a` 在转矩模式下是转矩给定，在速度模式下是速度环的输出：参考生成器 不需要知道当前是哪种模式，因为它运
 *     行时外环已经把给定归约到 q 轴上了。
 *
 * @context [en] Current-loop ISR, read-only for the policy.
 * @context [zh] 电流环 ISR；对策略只读。
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
 *
 *     "d = 0" is optimal for a surface-mounted machine and nothing more: an interior-PM machine earns extra torque from
 *     a negative d current (MTPA), and any machine needs d current to run past its base speed (field weakening). That
 *     is why the split is a policy with an input struct, rather than a `0.0F` written into the current loop.
 *
 * @brief [zh] 默认参考生成器：d 轴恒为 0，q 轴直通。
 *
 *     “d = 0”只对表贴式电机最优，仅此而已：内嵌式电机靠负 d 轴电流拿到额外转矩（MTPA）， 任何电机超过基速后都要靠 d 轴
 *     电流做弱磁。所以这个“拆分”是一个带输入结构的策略，而不是 写死在电流环里的 `0.0F`。
 *
 * @note [en] A reference generator must be default constructible: the five-policy aggregate form leaves this member
 *            defaulted, which is what keeps existing assembly points source-compatible.
 * @note [zh] 参考生成器必须可默认构造：五元聚合初始化形式会把该成员留作默认值，现有装配点因此保持 源码兼容。
 * @context [en] Current-loop ISR: noexcept, no allocation.
 * @context [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class NullReferenceGenerator final {
  public:
    /**
     * @brief [en] Split the q-axis command into the dq reference.
     * @brief [zh] 把 q 轴给定拆分成 dq 参考。
     * @param in [en] inputs of this control step.
     * @param in [zh] 本控制步的输入。
     * @return [en] the dq reference in A; `Motor` clamps it to the nameplate and drops non-finite values before it
     *              reaches the current loop.
     * @return [zh] dq 参考，A；`Motor` 在送入电流环之前按铭牌限幅并丢弃非有限值。
     * @since 0.1.0
     */
    [[nodiscard]] auto generate(const ReferenceInput& in) noexcept -> Dq<float> {
        return Dq<float>{0.0F, in.iq_command_a};
    }
};

/**
 * @brief [en] MTPA: the d-axis current that gets the most torque per amp.
 *
 *     Why a policy and not a constant. For a surface-mounted machine `i_d = 0` *is* MTPA, so the whole saliency story
 *     looks like a detail. For an interior-PM machine it is not a detail: the reluctance torque is worth a third of the
 *     total, and every amp spent on the wrong `i_d` is an amp not making torque. The gap between the two is the
 *     difference between needing a bigger drive and not.
 *
 *     The curve. Setting `dT/di_d = 0` on `T = 1.5*p*(psi_f*i_q + (Ld-Lq)*i_d*i_q)` subject to `i_d^2+i_q^2 = |i|^2`
 *     gives `i_d = (psi_f - sqrt(psi_f^2 + 8*(Lq-Ld)^2*i_q^2)) / (4*(Lq-Ld))`, which is negative for `Lq > Ld` and
 *     degenerate at `Lq = Ld` -- the degenerate case is handled as `i_d = 0` rather than by a tiny epsilon, because an
 *     epsilon would produce an `i_d` of order `psi_f/eps` and destroy the loop.
 *
 * @brief [zh] MTPA：每安培转矩最大的那个 d 轴电流。
 *
 *     为什么是策略而不是常数。表贴机上 `i_d = 0` **就是** MTPA，于是整个凸极性故事看起来像个细节。内嵌机上它不是 细节：
 *     磁阻转矩占总量的三分之一，花在错误 `i_d` 上的每一安培都是没在做转矩的安培。两者之差就是"需要换更大驱动器" 与"不需
 *     要"之差。
 *
 *     曲线。在 `i_d²+i_q² = |i|²` 约束下对 `T = 1.5·p·(ψf·iq + (Ld−Lq)·id·iq)` 令 `dT/di_d = 0`，得上式；`Lq > Ld` 时为
 *     负，`Lq = Ld` 时退化——退化情形按 `i_d = 0` 处理而不是用一个极小量，因为极小量会给出量级 `ψf/eps` 的 `i_d`，把环直
 *     接毁掉。
 *
 *     **This policy refuses to apply a d-axis current nobody can see.** The shipped single-`L` observer's flux target
 *     is a constant `psi_f`, so any `i_d` moves the flux it believes it is tracking, and at low speed the back-EMF is
 *     too small to outvote the error: measured on the nominal machine, `i_d = -1 A` reads 0.5079 rad at 200 rad/s
 *     against the published 0.15 rad ceiling, while the same current is harmless at 3200 rad/s. The allowance therefore
 *     scales with speed (`i_d_per_speed_a_s_per_rad`) and defaults to **zero**, which is the only rate that is safe
 *     with no further knowledge. A machine with a position sensor can raise it without limit; a sensorless one cannot.
 *
 *     **这个策略拒绝施加没人看得见的 d 轴电流。** 出厂单 `L` 观测器的磁链目标是常数 `ψf`，所以任何 `i_d` 都会挪动 它以
 *     为自己 在跟踪的那个磁链，而低速下反电势压不住这个误差：标称机上实测，`i_d = −1 A` 在 200 rad/s 读到 0.5079 rad，
 *     而公开天花板 是 0.15 rad；同一个电流在 3200 rad/s 却无害。因此许可量随转速线性放大 （
 *     `i_d_per_speed_a_s_per_rad`），默认**为零**——在没有任何其他知识时，零是唯一安全的比率。带位置传感器的 机器可以放
 *     开，无感的机器不行。
 *
 * @note [en] The cost of the guard is torque, and it is stated rather than hidden: clipping `i_d` toward zero leaves
 *            the MTPA curve, so the policy returns fewer newton-metres than the curve says. That is the trade: an
 *            accurate angle, or the reluctance torque, not both -- below the speed where the back-EMF carries the
 *            observer.
 * @note [zh] 这道门的代价是转矩，而且是被写明而不是被隐藏的：把 `i_d` 往零夹就是离开 MTPA 曲线，所以策略返回的 牛顿米少
 *            于曲线给出的值。这就是取舍：准确的角度，或磁阻转矩，二者不能兼得——在反电势撑起观测器的那个 转速以下。
 * @context [en] Current-loop ISR, on the reference-generation path.
 * @context [zh] 电流环 ISR，位于参考生成路径上。
 * @since 0.1.0
 * @exporter
 */
class MtpaReference final {
  public:
    /**
     * @brief [en] The machine.
     * @brief [zh] 机器。
     * @since 0.1.0
     */
    struct Config final {
        float ld;      ///< [en] d-axis inductance, H. [zh] d 轴电感，H。
        float lq;      ///< [en] q-axis inductance, H. [zh] q 轴电感，H。
        float psi_f;   ///< [en] rotor flux linkage, Wb. [zh] 转子磁链，Wb。
        float i_max_a; ///< [en] per-axis current limit, A. [zh] 每轴电流限幅，A。
        /// [en] How much d-axis current this machine may use per rad/s of electrical speed, A/(rad/s).
        /// Defaults to 0, which refuses d-axis current outright: that is the only value that is safe with no
        /// statement about how the angle is measured. Measured on the nominal machine, the published 0.15 rad ceiling
        /// holds up to `|i_d| = 0.04 A` at 100 rad/s, 0.12 A at 200, 0.29 A at 400 and 0.92 A at 800, so 3.5e-4 is a
        /// rate that is safe across that range; a machine with a position sensor can pass any large value.
        /// [zh] 这台机器每 rad/s 电角速度允许用多少 d 轴电流，A/(rad/s)。默认 0，即完全不施加 d 轴电流：在对角度怎么
        /// 测没有任何陈述时，零是唯一安全的值。标称机上实测，公开的 0.15 rad 天花板在 100 rad/s 时允许 `|i_d| =
        /// 0.04 A`、200 时 0.12 A、400 时 0.29 A、800 时 0.92 A，所以 3.5e-4 是在这一段上都安全的比率；带位置
        /// 传感器的机器可以传任意大值。
        float i_d_per_speed_a_s_per_rad{0.0F};
    };

    MtpaReference() noexcept = default;

    explicit MtpaReference(const Config& config) noexcept : config_(config) {}

    /// [en] Swap in a new machine without reconstructing. [zh] 不必重建就能换机器。
    void set_config(const Config& config) noexcept { config_ = config; }

    /**
     * @brief [en] The dq current reference for this step.
     * @brief [zh] 本步的 dq 电流参考。
     * @param in [en] q command and the rest (unused here).
     * @param in [zh] q 给定及其余项（此处不用）。
     * @return [en] `{i_d, i_q}` in A, with `|i| <= i_max_a`.
     * @return [zh] `{i_d, i_q}`，A，且 `|i| <= i_max_a`。
     * @note [en] When the request does not fit, the answer is **not** to scale both components: that leaves the MTPA
     *            curve and gives up more torque than necessary. The answer is to walk *down the curve* until the
     *            magnitude fits, which keeps "most torque per amp" true at every point including the clamp. The walk is
     *            a bisection on `i_q` because magnitude is monotone in `i_q` along the curve, and 24 halvings is the
     *            same budget the field-weakening solver uses.
     * @note [zh] 装不下时**不是**把两个分量一起缩小：那会离开 MTPA 曲线，多丢了本可以拿到的转矩。 正确做法是**沿曲线往
     *            下走**直到幅值装得下，这样"每安培转矩最大"在包括限幅点的每一点都 仍成立。走法是对 `i_q` 二分，因为沿曲
     *            线幅值对 `i_q` 单调，而 24 次折半正是弱磁求解器 用的同一预算。
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
        // The d-axis allowance for this speed. Clipping happens after the walk, never inside it: a
        // smaller `|i_d|` only shrinks the magnitude, so a reference that fitted the current limit
        // before the clip still fits after it.
        // 本转速下的 d 轴许可量。夹紧发生在走曲线**之后**，绝不在其中：`|i_d|` 变小只会让幅值更小，
        // 所以夹紧前装得下电流限幅的参考，夹紧后依然装得下。
        const float allowance = i_d_limit_a(in.omega_e_rad_s);
        // The unconstrained answer first; it is the common case.
        // 先给无约束的答案；那才是常见情形。
        if (magnitude(iq_abs) <= limit) {
            return Dq<float>{clipped(d_axis(iq_abs), allowance), sign * iq_abs};
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
        return Dq<float>{clipped(d_axis(lo), allowance), sign * lo};
    }

    /**
     * @brief [en] The d-axis current this policy will allow at that speed, A.
     * @brief [zh] 该转速下本策略允许的 d 轴电流，A。
     * @param omega_e_rad_s [en] electrical speed, rad/s; the sign does not matter.
     * @param omega_e_rad_s [zh] 电角速度，rad/s；符号无关。
     * @return [en] `i_d_per_speed_a_s_per_rad * |omega|`, which is 0 for the default config -- that is the guard, and
     *              it is answered here rather than hidden inside `generate()` so a caller can report it.
     * @return [zh] `i_d_per_speed_a_s_per_rad * |omega|`，默认配置下就是 0——这就是那道门，从这里而不是从 `generate()`
     *              内部回答，是为了调用方能把它报出去。
     * @since 0.1.0
     */
    [[nodiscard]] auto i_d_limit_a(float omega_e_rad_s) const noexcept -> float {
        const float rate = config_.i_d_per_speed_a_s_per_rad;
        if (!(rate > 0.0F)) {
            return 0.0F;
        }
        return rate * std::fabs(omega_e_rad_s);
    }

    /// [en] The config in force. [zh] 当前配置。
    [[nodiscard]] auto config() const noexcept -> const Config& { return config_; }

  private:
    /// [en] `d` held inside `[-allowance, 0]`. [zh] 把 `d` 夹在 `[-allowance, 0]` 内。
    [[nodiscard]] static auto clipped(float d, float allowance) noexcept -> float {
        return (d < -allowance) ? -allowance : d;
    }

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