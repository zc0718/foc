// Conan::ImportStart
#pragma once
#include <cmath>
#include <foc/core/machine_params.hpp>
#include <foc/core/reference.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] Whether field weakening is worth building for *this* machine, as numbers rather than an opinion.
 *
 *     The number that decides it is the **characteristic current** `i_char = psi_f/Ld`: the d-axis current that would
 *     null the rotor flux. If `i_char` is far above the current limit, the drive cannot null the flux, and field
 *     weakening buys only the sliver `i_max/i_char` of it -- measured on the SIL machine as 2910 -> 3850 rad/s (**+32
 *     %**) with torque collapsing to zero at the top. If `i_char` is below the current limit the machine has a real
 *     constant-power region: the same machine with ten times the inductance holds 36 % of its base torque at 8000
 *     rad/s.
 *
 * @brief [zh] 对**这台**机器来说弱磁值不值得做，用数说话，不用观点。
 *
 *     决定它的是**特征电流** `i_char = ψf/Ld`：把转子磁链削到零所需的 d 轴电流。 如果 `i_char` 远高于电流限幅，驱动器根
 *     本削不动磁链，弱磁只买到 `i_max/i_char` 那一点点—— SIL 标称机上实测是 2910 → 3850 rad/s（**+32 %**），顶端转矩塌
 *     到零。如果 `i_char` 低于电流 限幅，这台机器就有真正的恒功率区：同一台机器把电感放大十倍，8000 rad/s 处仍有基速转
 *     矩的 36 %。
 *
 * @note [en] The `speed_gain` here is the closed-form ceiling at `i_q = 0`: `psi_f/(psi_f - Ld*i_max)`. It is an upper
 *            bound -- carrying torque needs more voltage, so the measured envelope sits slightly below it -- which is
 *            exactly why it is safe to use as a *go/no-go* number and not as a target.
 * @note [zh] 这里的 `speed_gain` 是 `i_q = 0` 处的闭式上限： `ψf/(ψf − Ld·i_max)`。它是上界——带转矩要更多电压，所以实测
 *            包络略低于它——这正是它可以用来 做"做/不做"的门槛而不能当目标值的原因。
 * @since 0.1.0
 * @exporter
 */
struct FieldWeakeningAdvice final {
    float i_char_a; ///< [en] `psi_f/Ld`, A. [zh] `ψf/Ld`，A。
    /**
     * @brief [en] `i_char/i_max`; below 1 means the flux can be nulled inside the current limit.
     * @brief [zh] `i_char/i_max`；小于 1 表示在电流限幅内能把磁链削到零。
     */
    float ratio;
    /**
     * @brief [en] speed ceiling at `i_q = 0` relative to base, e.g. 1.34 = +34 %.
     * @brief [zh] `i_q = 0` 处的转速上限相对基速， 例如 1.34 = +34 %。
     */
    float speed_gain;
    /**
     * @brief [en] the verdict, with the rule stated in the docs of `is_field_weakening_worthwhile()`.
     * @brief [zh] 结论，判据写在 `is_field_weakening_worthwhile()` 的文档里。
     */
    bool worthwhile;
};

/**
 * @brief [en] The go/no-go test, from nameplate numbers alone.
 *
 *     The rule, and why each half of it exists:
 *
 *     - **`ratio < 0.6`**: the flux has to be substantially nullable inside the current limit, or the region is a
 *     sliver. - **`speed_gain >= 1.5`**: the product has to actually gain speed. A 32 % stretch is usually cheaper to
 *     buy with bus voltage, and raising the bus does not break `i_d ~ 0`, does not touch the observer, and does not
 *     spend current on heat. Either failing means "do not build it"; both passing means the machine was designed for
 *     it.
 *
 * @brief [zh] 只用铭牌数字做的"做/不做"判定。
 *
 *     规则，以及每一半为什么存在： - **`ratio < 0.6`**：磁链要在电流限幅内被大幅削弱，否则那个区只是一条缝。 -
 *     **`speed_gain >= 1.5`**：产品要真的因此提速。32 % 的延展通常用母线电压买更便宜， 而升母线不破坏 `i_d ≈ 0`、不碰观
 *     测器、也不需要拿电流去发热。 任一条不满足就是"不要做"；两条都满足说明这台机器本来就是为弱磁设计的。
 *
 * @param params [en] the machine, SI.
 * @param params [zh] 机器，SI。
 * @param i_max_a [en] the current limit the drive will actually use, A.
 * @param i_max_a [zh] 驱动器实际会用的电流限幅，A。
 * @return [en] the three numbers and the verdict.
 * @return [zh] 三个数与结论。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto is_field_weakening_worthwhile(const MachineParams& params, float i_max_a) noexcept
    -> FieldWeakeningAdvice {
    FieldWeakeningAdvice advice{0.0F, 0.0F, 1.0F, false};
    if (!(params.ld > 0.0F) || !(params.psi_f > 0.0F) || !(i_max_a > 0.0F)) {
        return advice;
    }
    advice.i_char_a = params.psi_f / params.ld;
    advice.ratio = advice.i_char_a / i_max_a;
    const float residual_flux = params.psi_f - (params.ld * i_max_a);
    // A machine whose current limit already nulls the flux has no ceiling; the
    // gain is reported as infinite rather than as a huge float, because a huge
    // float invites arithmetic on it.
    // 电流限幅本身就能把磁链削到零的机器没有转速上限；这里把增益报成无穷而不是一个巨大的浮点数，
    // 因为巨大的浮点数会诱使别人拿它去做算术。
    if (!(residual_flux > 0.0F)) {
        advice.speed_gain = INFINITY;
        advice.worthwhile = true;
        return advice;
    }
    advice.speed_gain = params.psi_f / residual_flux;
    advice.worthwhile = (advice.ratio < 0.6F) && (advice.speed_gain >= 1.5F);
    return advice;
}

/**
 * @brief [en] The flux magnitude the observer should pull towards when the drive is spending current on `i_d`.
 *
 *     `psi_f` alone is the *rotor* linkage. As soon as `i_d != 0` the stator flux vector is `psi_f + Ld*i_d` on the d
 *     axis, and telling a magnitude correction to aim at `psi_f` is telling it to aim at the wrong number: measured in
 *     SIL, at `i_d = -2.95 A` the true magnitude is 75 % of `psi_f` and the resulting angle error goes from 0.0467 rd
 *     to 0.0709 rd. That is still inside the 0.10 rd gate, which is why this is a *refinement* and not a prerequisite
 *     -- but it is a refinement with a measurable size.
 *
 * @brief [zh] 当驱动器把电流花在 `i_d` 上时，观测器该把幅值拉向多少。
 *
 *     单独的 `psi_f` 是**转子**磁链。只要 `i_d != 0`，定子磁链矢量在 d 轴上就是 `ψf + Ld·i_d`，而让幅值修正去瞄准
 *     `psi_f` 就是让它瞄错数：SIL 实测 `i_d = −2.95 A` 时 真实幅值是 `ψf` 的 75 %，角度误差从 0.0467 rad 走到 0.0709
 *     rad。那仍在 0.10 rad 闸内， 所以这是**refinement**而不是前置条件——但它是一个大小可测的 refinement。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] constexpr auto flux_reference_wb(float psi_f, float ld, float i_d) noexcept -> float {
    const float reference = psi_f + (ld * i_d);
    // Never hand a non-positive magnitude to a correction loop: it would pull
    // the estimate through zero and the angle with it.
    // 绝不把非正的幅值交给修正环：那会把估计拉过零点，角度跟着一起翻。
    return (reference > 1.0e-6F) ? reference : 1.0e-6F;
}

/**
 * @brief [en] The d-axis reference that keeps the drive inside its voltage ceiling by spending current on flux instead
 *             of on torque.
 *
 *     Drop-in for `NullReferenceGenerator`: same shape, same call site, which is the point -- `ReferenceInput` already
 *     carries `omega_e`, `v_bus` and the measured current, so field weakening needs **no change to `Motor`, no new
 *     template parameter on the existing instantiations, and no state**. Measured (SIL): the nominal machine's `i_d`
 *     goes to `-1.55 A` at the base speed and `-2.70 A` at 3500 rad/s, holding `i_q = 1.24 A` where `i_d = 0` would
 *     give zero torque.
 *
 * @brief [zh] 用电流去买磁链（而不是转矩），从而把驱动器留在电压上限以内的 d 轴参考。
 *
 *     `NullReferenceGenerator` 的直接替代：同样的形状、同样的调用点，这正是要点—— `ReferenceInput` 已经带着 `omega_e`、
 *     `v_bus` 和实测电流，所以弱磁**不需要改 `Motor`、 不需要给既有实例化加模板参数、也不需要状态**。实测（SIL）：标称
 *     机在基速处 `i_d` 到 `−1.55 A`，3500 rad/s 处到 `−2.70 A`，同时守住 `i_q = 1.24 A`，而 `i_d = 0` 在那里是零转矩。
 *
 * @note [en] The decision is solved **without `Rs`**, and then verified against the full voltage equation in the tests.
 *            That is deliberate: the resistor's contribution to the ellipse is small compared with `omega*L` in the
 *            region where this matters, the closed form is what makes the policy a dozen lines, and the test that
 *            measures `|v|` against `v_max` is what keeps the approximation honest instead of hidden.
 * @note [zh] 判定**不含 `Rs`**，然后在测试里对着完整电压方程验证。这是有意的：在真正需要它的区域， 电阻对椭圆的贡献相对
 *            `ωL` 很小，闭式解才让这个策略只有十几行，而"把 `|v|` 对着 `v_max` 量" 的那条测试让近似保持诚实而不是隐藏起
 *            来。
 * @since 0.1.0
 * @exporter
 */
class FieldWeakeningReference final {
  public:
    /**
     * @brief [en] The machine and the ceilings.
     * @brief [zh] 机器与上限。
     * @since 0.1.0
     */
    struct Config final {
        /**
         * @brief [en] unused by the decision, kept for symmetry with the other policies.
         * @brief [zh] 判定不用，保留以与其它策略对称。
         */
        float rs;
        float l;       ///< [en] single-L model, H. [zh] 单一 L 模型，H。
        float psi_f;   ///< [en] rotor flux linkage, Wb. [zh] 转子磁链，Wb。
        float i_max_a; ///< [en] per-axis current limit, A. [zh] 每轴电流限幅，A。
        /**
         * @brief [en] `v_max/v_bus`; `1/sqrt(3)` for SVPWM's linear range.
         * @brief [zh] `v_max/v_bus`；SVPWM 线性区取 `1/√3`。
         */
        float v_max_ratio;
    };

    FieldWeakeningReference() noexcept = default;

    explicit FieldWeakeningReference(const Config& config) noexcept : config_(config) {}

    /**
     * @brief [en] The dq current reference for this step.
     * @brief [zh] 本步的 dq 电流参考。
     * @param in [en] q command, speed, bus, measured current.
     * @param in [zh] q 给定、转速、 母线、实测电流。
     * @return [en] `{i_d, i_q}` in A, both inside the current limit.
     * @return [zh] `{i_d, i_q}`，A，两者都在电流限幅内。
     * @note [en] `i_d` is **continuous** in `omega_e` across the base speed: the ellipse solution grows away from zero
     *            as soon as the point no longer fits, so there is no step for an integrator to be shocked by. That
     *            continuity is what the continuity requirement is about -- a discontinuous reference would wind the
     *            current PI up and cost an overshoot on the way back down, which is the classic way a field-weakening
     *            implementation damages a machine it was supposed to help.
     * @note [zh] `i_d` 在基速处对 `omega_e` 是**连续**的：一旦工作点装不下，椭圆解就从零连续 长出来，所以积分器没有台阶
     *            可被震。这个连续性正是那条要求的来由——不连续的参考会把电流 PI 积分上去，回程时付出一个过冲，那正是弱磁
     *            实现伤到它本该帮助的机器时的经典方式。
     * @since 0.1.0
     */
    [[nodiscard]] auto generate(const ReferenceInput& in) noexcept -> Dq<float> {
        const float i_limit = (config_.i_max_a > 0.0F) ? config_.i_max_a : 0.0F;
        const float i_q_want = clamp_axis(in.iq_command_a, i_limit);
        if (!(config_.l > 0.0F) || !(config_.psi_f > 0.0F) || !(in.v_bus_v > 0.0F) ||
            !std::isfinite(in.omega_e_rad_s)) {
            return Dq<float>{0.0F, i_q_want};
        }
        const float omega = std::fabs(in.omega_e_rad_s);
        if (!(omega > 0.0F)) {
            return Dq<float>{0.0F, i_q_want};
        }
        const float v_max = config_.v_max_ratio * in.v_bus_v;

        // The voltage ellipse **with `Rs`**, as an exact quadratic in `i_d`:
        // `A*i_d^2 + B*i_d + C = 0` with `A = Rs^2 + x^2`, `B = 2*x*omega*psi_f`,
        // `C = (x*i_q)^2 + (Rs*i_q + omega*psi_f)^2 - v_max^2`. The root closest to
        // zero is the one the drive should sit on; the other is the far side of the
        // ellipse, where the voltage is in phase with the current and the drive is
        // over-excited for no reason.
        // 含 `Rs` 的电压椭圆，写成 `i_d` 的精确二次方程：`A·i_d² + B·i_d + C = 0`，
        // `A = Rs² + x²`、`B = 2·x·ωψf`、`C = (x·i_q)² + (Rs·i_q + ωψf)² − v_max²`。
        // 离零近的那个根才是驱动器该待的地方；另一个是椭圆远端，那里电压与电流同相，白白过励。
        //
        // Including `Rs` is not pedantry: at 24 V and 3200 rad/s, dropping it lets the
        // decision believe `i_q = 2.6 A` fits where the exact equation allows 1.98 A --
        // a 30 % optimism that a real drive pays for in voltage saturation.
        // 含 `Rs` 不是迂腐：24 V、3200 rad/s 下忽略它会让判定以为 `i_q = 2.6 A` 装得下，而精确
        // 方程只允许 1.98 A——30 % 的乐观，实机会用电压饱和来付账。
        const float x = omega * config_.l;
        const float e = omega * config_.psi_f;
        const float a = (config_.rs * config_.rs) + (x * x);
        const float b = 2.0F * x * e;
        const float rs = config_.rs;
        auto i_d_needed = [a, b, x, e, v_max, i_limit, rs](float i_q) noexcept -> float {
            const float g = (rs * i_q) + e;
            const float c = ((x * i_q) * (x * i_q)) + (g * g) - (v_max * v_max);
            const float discriminant = (b * b) - (4.0F * a * c);
            if (!(discriminant > 0.0F)) {
                // No real root: this `i_q` is out of reach at any `i_d`.
                // 无实根：这个 `i_q` 在任何 `i_d` 下都够不到。
                return -i_limit - 1.0F;
            }
            return ((-b) + std::sqrt(discriminant)) / (2.0F * a);
        };
        auto fits = [&](float i_q, float i_d) noexcept -> bool {
            return (i_d * i_d) + (i_q * i_q) <= (i_limit * i_limit) * 1.0001F;
        };

        // 1. Does the requested operating point fit as it stands?
        // 1. 要求的这个工作点本身就装得下吗？
        const float i_d_at_want = i_d_needed(i_q_want);
        if (!(i_d_at_want < 0.0F)) {
            // Still inside the voltage ceiling with `i_d = 0`: nothing to do.
            // 在 `i_d = 0` 下仍在电压上限以内：无事可做。
            return Dq<float>{0.0F, i_q_want};
        }
        if (fits(i_q_want, i_d_at_want)) {
            return Dq<float>{i_d_at_want, i_q_want};
        }

        // 2. It does not: trade `i_q` for `i_d` down the **current circle**, which
        // is the constant-power corner. Bisection because the feasible set is an
        // interval and 24 halvings of a 3 A span land inside float precision --
        // deterministic, no allocation, no table.
        // 2. 装不下：沿着**电流圆**把 `i_q` 换成 `i_d`，这就是恒功率拐角。用二分，因为可行集是一个
        // 区间，3 A 跨度上 24 次折半已经落到 float 精度之内——确定、不分配、不用表。
        float low = 0.0F;
        float high = i_q_want;
        if (!fits(0.0F, i_d_needed(0.0F))) {
            // Even at zero torque the speed is out of reach: give the flux
            // everything there is. The drive is asking for a speed it cannot
            // hold, and saying so with a saturated `i_d` is safer than pretending.
            // 连零转矩下这个转速都够不到：把全部电流给磁链。驱动器要的是一个它守不住的速度，用饱和的
            // `i_d` 说出这件事比假装安全。
            return Dq<float>{-i_limit, 0.0F};
        }
        for (int k = 0; k < 24; ++k) {
            const float mid = 0.5F * (low + high);
            if (fits(mid, i_d_needed(mid))) {
                low = mid;
            } else {
                high = mid;
            }
        }
        return Dq<float>{i_d_needed(low), low};
    }

    /**
     * @brief [en] The config in force.
     * @brief [zh] 当前配置。
     */
    [[nodiscard]] auto config() const noexcept -> const Config& { return config_; }

  private:
    /**
     * @brief [en] Symmetric clamp that also drops non-finite input.
     * @brief [zh] 对称限幅，同时丢弃非有限输入。
     */
    [[nodiscard]] static auto clamp_axis(float value, float limit) noexcept -> float {
        if (!std::isfinite(value)) {
            return 0.0F;
        }
        if (value > limit) {
            return limit;
        }
        return (value < -limit) ? -limit : value;
    }

    Config config_{};
};

} // namespace core

} // namespace foc
