// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace sil {

/**
 * @brief [en] A second, **independently derived** PMSM plant in phase variables,
 * used only to cross-check the dq plant. [zh]
 * 第二台**独立推导**的相变量 PMSM 被控对象，只用于与 dq plant 对拍。
 *
 *        [en] Why this exists, and what it substitutes for. With no motor expert to review the dq derivation, a sign or
 *        factor error in `sil_pmsm_plant.hpp` would make every SIL conclusion wrong in the same direction while staying
 * *internally* consistent -- nothing inside the project could see it. This model
 * never forms a dq quantity: it integrates the three phase fluxes directly from
 * `v = Rs*i + d(lambda)/dt` with
 * `lambda_p = (Ls - M)*i_p + lambda_m*cos(theta_p)`, and it derives torque from
 * co-energy as `Te = -p*lambda_m*sum(i_p*sin(theta_p))`. If the two models agree
 * at six operating points, both derivations are almost certainly right; if they
 * disagree, one is wrong and the disagreement points at which.
 *        [zh] 为什么要有它、它替代什么。没有电机专家来评审 dq 推导时， `sil_pmsm_plant.hpp`
 *        里一个符号或系数错误会让**所有** SIL 结论朝同一个方向错，而内部
 *        完全自洽——项目内部没有任何东西能看见它。本模型**从不构造 dq 量**：它直接按 `v = Rs·i + dλ/dt`
 *        积分三相磁链，`λ_p = (Ls − M)·i_p + λm·cos(θ_p)`， 并由磁共能给出转矩 `Te = −p·λm·Σ(i_p·sin
 *        θ_p)`。两个模型在六个工作点上一致，
 *        则两个推导几乎必然都对；若不一致，其中一个错了，而不一致的位置会指出是哪一个。
 *
 * @note [en] Scope, stated so nobody over-reads it. This is a **non-salient**
 * machine (`Ld = Lq`), because that is the case where the phase-variable form is
 * unambiguous; a salient abc model needs an angle-dependent inductance matrix and
 * would reintroduce the very transformation being checked. The mechanical
 * equation is deliberately frozen: mechanics is not where derivation errors hide,
 * and the dq plant already covers it.
 * [zh] 范围，写清楚以免被过度解读。这是**非凸极**机器（`Ld = Lq`），因为那正是相变量
 * 形式无歧义的情形；凸极的 abc 模型需要随角度变化的电感矩阵，会把正要检查的那个变换
 * 又引回来。机械方程故意冻结：推导错误不藏在力学里，而 dq plant 已经覆盖了它。
 *
 * @context [en] Host-side SIL only: never part of the shipped library. [zh]
 * 仅 host 侧 SIL：绝不进入发布库。
 * @since 0.1.0
 */
class AbcPlant final {
public:
    /**
     * @brief [en] The machine, plus the leakage/mutual split.
     *        [zh] 机器，外加漏感/互感的分法。
     * @note [en] `ls_minus_m` is the synchronous inductance `Ls - M`; with the
     * plant's `ld == lq == l`, that is exactly `l`.
     * [zh] `ls_minus_m` 是同步电感 `Ls − M`；在 plant 的 `ld == lq == l` 下，它正好等于 `l`。
     * @since 0.1.0
     */
    struct Params final {
        float rs;         ///< [en] stator resistance, ohm. [zh] 定子电阻，Ω。
        float ls_minus_m; ///< [en] synchronous inductance `Ls - M`, H. [zh]
                          ///< 同步电感 `Ls − M`，H。
        float lambda_m;   ///< [en] rotor flux linkage, Wb. [zh] 转子磁链，Wb。
        std::uint8_t pole_pairs; ///< [en] pole pairs. [zh] 极对数。
    };

    explicit AbcPlant(const Params& params) noexcept : params_(params) {}

    /// [en] Set the frozen electrical speed and the rotor angle. [zh]
    /// 设定冻结的电角速度与转子角。
    void set_state(float theta_e_rad, float omega_e_rad_s) noexcept {
        theta_e_ = theta_e_rad;
        omega_e_ = omega_e_rad_s;
    }

    /// [en] Line-to-neutral voltages, V. [zh] 相对中点电压，V。
    void set_voltage(const core::Uvw<float>& v) noexcept { v_ = v; }

    /// [en] Advance with RK4 on the two independent phase currents. [zh]
    /// 对两个独立相电流做 RK4 推进。
    void advance(float dt_s) noexcept {
        // `i_w = -i_u - i_v` from Kirchhoff, so two states are enough and the
        // constraint is satisfied by construction rather than by correction.
        // 由基尔霍夫 `i_w = −i_u − i_v`，两个状态就够，约束是构造上满足的，而不是靠事后修正。
        float iu = i_u_;
        float iv = i_v_;
        float theta = theta_e_;
        const float h = dt_s / 8.0F;
        for (int step = 0; step < 8; ++step) {
            const Derivatives k1 = derivative(iu, iv, theta);
            const Derivatives k2 = derivative(
                static_cast<float>(iu + (0.5F * h * k1.du)),
                static_cast<float>(iv + (0.5F * h * k1.dv)),
                static_cast<float>(theta + (0.5F * h * omega_e_)));
            const Derivatives k3 = derivative(
                static_cast<float>(iu + (0.5F * h * k2.du)),
                static_cast<float>(iv + (0.5F * h * k2.dv)),
                static_cast<float>(theta + (0.5F * h * omega_e_)));
            const Derivatives k4 = derivative(
                static_cast<float>(iu + (h * k3.du)),
                static_cast<float>(iv + (h * k3.dv)),
                static_cast<float>(theta + (h * omega_e_)));
            iu = static_cast<float>(iu + ((h / 6.0F) * ((k1.du + (2.0F * k2.du)) +
                                                        ((2.0F * k3.du) + k4.du))));
            iv = static_cast<float>(iv + ((h / 6.0F) * ((k1.dv + (2.0F * k2.dv)) +
                                                        ((2.0F * k3.dv) + k4.dv))));
            theta = static_cast<float>(theta + (h * omega_e_));
            theta_e_ = theta;
        }
        i_u_ = iu;
        i_v_ = iv;
    }

    /// [en] The three phase currents, A. [zh] 三相电流，A。
    [[nodiscard]] auto current() const noexcept -> core::Uvw<float> {
        return core::Uvw<float>{i_u_, i_v_, static_cast<float>(-i_u_ - i_v_)};
    }

    /// [en] Electromagnetic torque from magnetic co-energy, **without Park**: N*m.
    /// [zh] 由磁共能给出的电磁转矩，**不经 Park**：N·m。
    [[nodiscard]] auto torque() const noexcept -> float {
        const core::Uvw<float> i = current();
        float sum = 0.0F;
        for (int phase = 0; phase < 3; ++phase) {
            const float angle = static_cast<float>(
                theta_e_ - (kTwoThirdPi * static_cast<float>(phase)));
            sum = static_cast<float>(sum + (phase_current(i, phase) * std::sin(angle)));
        }
        return -static_cast<float>(params_.pole_pairs) * params_.lambda_m * sum;
    }

    /// [en] Mechanical power balance: `Te*omega_m = v.i - Rs*|i|^2` at steady
    /// state. Reported so the test can check the lossless case too. [zh]
    /// 机械功率平衡：稳态下 `Te·ω_m = v·i − Rs·|i|²`。一并给出，好让测试也能检查无损耗情形。
    [[nodiscard]] auto electrical_power() const noexcept -> float {
        const core::Uvw<float> i = current();
        return static_cast<float>((v_.u * i.u) + (v_.v * i.v) + (v_.w * i.w));
    }

    /// [en] The electrical speed the model is frozen at, rad/s. [zh]
    /// 模型冻结的电角速度，rad/s。
    [[nodiscard]] auto omega_e() const noexcept -> float { return omega_e_; }

private:
    struct Derivatives final {
        float du;
        float dv;
    };

    static constexpr float kTwoThirdPi = 2.0943951F;

    [[nodiscard]] static auto phase_current(const core::Uvw<float>& i,
                                            int phase) noexcept -> float {
        switch (phase) {
        case 0:
            return i.u;
        case 1:
            return i.v;
        default:
            return i.w;
        }
    }

    /// [en] `di/dt` per phase from `v = Rs*i + (Ls-M)*di/dt + d(lambda_m)/dt`,
    /// with the magnet term differentiated by hand: `d/dt cos(theta) = -omega*sin(theta)`.
    /// [zh] 每相的 `di/dt`，来自 `v = Rs·i + (Ls−M)·di/dt + dλm/dt`，磁钢项手写求导：
    /// `d/dt cos θ = −ω·sin θ`。
    [[nodiscard]] auto derivative(float iu, float iv, float theta) const noexcept
        -> Derivatives {
        const float back_u = params_.lambda_m * std::sin(theta);
        const float back_v = params_.lambda_m * std::sin(theta - kTwoThirdPi);
        const float scale = 1.0F / params_.ls_minus_m;
        return Derivatives{
            static_cast<float>((v_.u - (params_.rs * iu) + (omega_e_ * back_u)) * scale),
            static_cast<float>((v_.v - (params_.rs * iv) + (omega_e_ * back_v)) * scale)};
    }

    Params params_{};
    core::Uvw<float> v_{0.0F, 0.0F, 0.0F};
    float i_u_{0.0F};
    float i_v_{0.0F};
    float theta_e_{0.0F};
    float omega_e_{0.0F};
};

} // namespace sil

} // namespace foc
