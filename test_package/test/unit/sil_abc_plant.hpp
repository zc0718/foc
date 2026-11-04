// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

#include "sil_cogging_table.hpp"

namespace foc {

namespace sil {

/**
 * @brief [en] A second, **independently derived** PMSM plant in phase variables, used only to cross-check the dq plant.
 *
 *     Why this exists, and what it substitutes for. With no motor expert to review the dq derivation, a sign or factor
 *     error in `sil_pmsm_plant.hpp` would make every SIL conclusion wrong in the same direction while staying
 *     *internally* consistent -- nothing inside the project could see it. This model never forms a dq quantity: it
 *     integrates the three phase fluxes directly from `v = Rs*i + d(lambda)/dt` with `lambda_p = (Ls - M)*i_p +
 *     lambda_m*cos(theta_p)`, and it derives torque from co-energy as `Te = -p*lambda_m*sum(i_p*sin(theta_p))`. If the
 *     two models agree at six operating points, both derivations are almost certainly right; if they disagree, one is
 *     wrong and the disagreement points at which.
 *
 * @brief [zh] 第二台**独立推导**的相变量 PMSM 被控对象，只用于与 dq plant 对拍。
 *
 *     为什么要有它、它替代什么。没有电机专家来评审 dq 推导时， `sil_pmsm_plant.hpp` 里一个符号或系数错误会让**所有**
 *     SIL 结论朝同一个方向错，而内部 完全自洽——项目内部没有任何东西能看见它。本模型**从不构造 dq 量**：它直接按 `v =
 *     Rs·i + dλ/dt` 积分三相磁链，`λ_p = (Ls − M)·i_p + λm·cos(θ_p)`， 并由磁共能给出转矩 `Te = −p·λm·Σ(i_p·sin θ_p)`。
 *     两个模型在六个工作点上一致， 则两个推导几乎必然都对；若不一致，其中一个错了，而不一致的位置会指出是哪一个。
 *
 * @note [en] Scope, stated so nobody over-reads it. This is a **non-salient** machine (`Ld = Lq`), because that is the
 *            case where the phase-variable form is unambiguous; a salient abc model needs an angle-dependent inductance
 *            matrix and would reintroduce the very transformation being checked. The mechanical equation is
 *            deliberately frozen: mechanics is not where derivation errors hide, and the dq plant already covers it.
 * @note [zh] 范围，写清楚以免被过度解读。这是**非凸极**机器（`Ld = Lq`），因为那正是相变量 形式无歧义的情形；凸极的 abc
 *            模型需要随角度变化的电感矩阵，会把正要检查的那个变换 又引回来。机械方程故意冻结：推导错误不藏在力学里，而
 *            dq plant 已经覆盖了它。
 * @context [en] Host-side SIL only: never part of the shipped library.
 * @context [zh] 仅 host 侧 SIL：绝不进入发布库。
 * @since 0.1.0
 */
class AbcPlant final {
  public:
    /**
     * @brief [en] The machine, plus the leakage/mutual split.
     * @brief [zh] 机器，外加漏感/互感的分法。
     * @note [en] `ls_minus_m` is the synchronous inductance `Ls - M`; with the plant's `ld == lq == l`, that is exactly
     *            `l`.
     * @note [zh] `ls_minus_m` 是同步电感 `Ls − M`；在 plant 的 `ld == lq == l` 下，它正好等于 `l`。
     * @since 0.1.0
     */
    struct Params final {
        float rs; ///< [en] stator resistance, ohm. [zh] 定子电阻，Ω。
        /**
         * @brief [en] synchronous inductance `Ls - M`, H.
         * @brief [zh] 同步电感 `Ls − M`，H。
         */
        float ls_minus_m;
        float lambda_m;          ///< [en] rotor flux linkage, Wb. [zh] 转子磁链，Wb。
        std::uint8_t pole_pairs; ///< [en] pole pairs. [zh] 极对数。
    };

    explicit AbcPlant(const Params& params) noexcept : params_(params) {}

    /**
     * @brief [en] What turns the co-energy torque into motion: rotor inertia and viscous friction.
     * @brief [zh] 把磁共能转矩变成运动的东西：转子惯量与粘性摩擦。
     */
    struct Mechanics final {
        float j; ///< [en] rotor inertia, kg*m^2. [zh] 转子惯量，kg·m²。
        float b; ///< [en] viscous friction, N*m*s. [zh] 粘性摩擦，N·m·s。
    };

    /**
     * @brief [en] Let the model integrate mechanics instead of holding the speed it was handed.
     * @brief [zh] 让模型自己积分机械方程，而不是保持交给它的转速。
     * @param mechanics [en] inertia and damping of the same machine the parameters describe.
     * @param mechanics [zh] 参数所描述的那台机器的惯量与阻尼。
     * @note [en] **Off by default, and that matters.** With mechanics off the speed and angle are exactly what
     *            `set_state()` put there, which is the frozen-speed model the six-point cross-check was written
     *            against; turning it on is a different experiment and must not silently change that one. Why it exists
     *            at all: a *transient* exchange between mechanics and the electrical side is the one thing a
     *            frozen-speed model cannot witness, and it is where this project's open-loop start claim lives. Here it
     *            is computed without ever forming a d-q quantity, and without ever using `Kt`.
     * @note [zh] **默认关闭，这一点很重要。** 关闭时转速与角度完全等于 `set_state()` 放进去的值，也就是六点对拍所对着的
     *            冻结转速模型；打开它是另一个实验，不能悄悄把它改掉。存在的理由：机械侧与电气侧之间的**瞬态**交换，正是
     *            冻结转速模型无法见证的东西，而本项目的开环起动主张就住在那里。这里的计算从不构造 d-q 量，也从不使用
     *            `Kt`。
     * @since 0.1.0
     */
    void enable_mechanics(const Mechanics& mechanics) noexcept {
        mechanics_on_ = mechanics.j > 0.0F;
        mech_ = mechanics;
    }

    /** @brief [en] Load torque opposing positive rotation, N*m. [zh] 阻碍正向旋转的负载转矩，N·m。 */
    void set_load_torque(float torque_nm) noexcept { load_nm_ = torque_nm; }

    /**
     * @brief [en] The shared harmonic type, so `AbcPlant::CoggingHarmonic` keeps naming the same thing both plants
     *             inject. See `sil_cogging_table.hpp` for what an order means.
     * @brief [zh] 共享的谐波类型，于是 `AbcPlant::CoggingHarmonic` 仍然指两台 plant 注入的同一个东西。次数的含义见
     *             `sil_cogging_table.hpp`。
     * @since 0.1.9
     */
    using CoggingHarmonic = ::foc::sil::CoggingHarmonic;

    /** @brief [en] How many harmonics the injection can hold. [zh] 注入最多能装几条谐波。 */
    static constexpr int kMaxCogging = ::foc::sil::kMaxCoggingHarmonics;

    /**
     * @brief [en] Inject a cogging torque: a rotor torque that exists with no current, is periodic in the
     *             **mechanical** angle, and averages to zero over a revolution.
     *
     * @brief [zh] 注入齿槽转矩：一个**不通电流也存在**、随**机械**角周期变化、一转平均为零的转子转矩。
     *
     *     It enters the mechanical equation **additively** (`J*w' = Te - b*w - load + Tcog`), which fixes the sign a
     *     compensator must produce: to cancel it, a controller has to make `Te` carry `-Tcog`. Writing it as a load
     *     that happens to depend on angle would work arithmetically and would leave that sign to be rediscovered by
     *     whoever writes the compensator.
     *
     *     **Off by default (no harmonics), and bit-identical when off** -- the same rule the other injection ports
     *     follow: an error source that changes the frozen scenarios cannot be told apart from a regression. 它**加性**
     *     进入机械方程（`J·ω' = Te − b·ω − load + Tcog`），这就固定了补偿器必须产出的符号：要抵消它，控制器得让 `Te` 带
     *     上 `-Tcog`。若把它写成「恰好随角度变化的负载」，算术上等价，却把那个符号留给写补偿的人重新发现一遍。 **默认关
     *     闭（无谐波），关闭时逐位不变**——与其它注入端口同一条规矩：会改变冻结场景的误差源无法与回归区分。
     *
     * @param harmonics [en] the table, at most `kMaxCogging` entries.
     * @param harmonics [zh] 谐波表，最多 `kMaxCogging` 条。
     * @param count [en] how many entries are valid.
     * @param count [zh] 有效条数。
     * @since 0.1.9
     */
    void set_cogging(const CoggingHarmonic* harmonics, int count) noexcept {
        cogging_count_ = ::foc::sil::store_cogging(cogging_, kMaxCogging, harmonics, count);
    }

    /** @brief [en] How many harmonics are active. [zh] 当前生效的谐波条数。 */
    [[nodiscard]] auto cogging_harmonics() const noexcept -> int { return cogging_count_; }

    /**
     * @brief [en] The injected torque at the current mechanical angle, N*m -- the truth a compensator is measured
     *             against.
     * @brief [zh] 当前机械角处的注入转矩，N·m——补偿器被对照的那把尺。
     * @since 0.1.9
     */
    [[nodiscard]] auto cogging_torque_nm() const noexcept -> float {
        return ::foc::sil::evaluate_cogging(cogging_, cogging_count_, theta_mech_rad());
    }

    /** @brief [en] Rotor angle in mechanical radians. [zh] 转子机械角，rad。 */
    [[nodiscard]] auto theta_mech_rad() const noexcept -> float {
        return theta_e_ / static_cast<float>(params_.pole_pairs);
    }

    /** @brief [en] Rotor speed in mechanical rad/s. [zh] 转子机械转速，rad/s。 */
    [[nodiscard]] auto omega_mech_rad_s() const noexcept -> float {
        return omega_e_ / static_cast<float>(params_.pole_pairs);
    }

    /** @brief [en] The parameters this model was built with. [zh] 构造本模型所用的参数。 */
    [[nodiscard]] auto params() const noexcept -> const Params& { return params_; }

    /** @brief [en] The three phase currents, same accessor name as the d-q plant. [zh] 三相电流，访问名与 d-q plant 一致。 */
    [[nodiscard]] auto current_uvw() const noexcept -> core::Uvw<float> { return current(); }

    /**
     * @brief [en] Put the rotor at a mechanical angle and speed, currents to zero.
     * @brief [zh] 把转子放到某个机械角度与转速，电流归零。
     */
    void reset(float theta_mech_rad, float omega_mech_rad_s) noexcept {
        theta_e_ = static_cast<float>(params_.pole_pairs) * theta_mech_rad;
        omega_e_ = static_cast<float>(params_.pole_pairs) * omega_mech_rad_s;
        omega_m_ = omega_mech_rad_s;
        i_u_ = 0.0F;
        i_v_ = 0.0F;
    }

    /**
     * @brief [en] Set the frozen electrical speed and the rotor angle.
     * @brief [zh] 设定冻结的电角速度与转子角。
     */
    void set_state(float theta_e_rad, float omega_e_rad_s) noexcept {
        theta_e_ = theta_e_rad;
        omega_e_ = omega_e_rad_s;
        omega_m_ = omega_e_rad_s / static_cast<float>(params_.pole_pairs);
    }

    /**
     * @brief [en] Line-to-neutral voltages, V.
     * @brief [zh] 相对中点电压，V。
     */
    void set_voltage(const core::Uvw<float>& v) noexcept { v_ = v; }

    /**
     * @brief [en] Advance with RK4 on the two independent phase currents.
     * @brief [zh] 对两个独立相电流做 RK4 推进。
     */
    void advance(float dt_s) noexcept {
        // `i_w = -i_u - i_v` from Kirchhoff, so two states are enough and the
        // constraint is satisfied by construction rather than by correction.
        // 由基尔霍夫 `i_w = −i_u − i_v`，两个状态就够，约束是构造上满足的，而不是靠事后修正。
        float iu = i_u_;
        float iv = i_v_;
        float theta = theta_e_;
        const float h = dt_s / 8.0F;
        for (int step = 0; step < 8; ++step) {
            if (mechanics_on_) {
                // The mechanical half of the same sub-step: co-energy torque in, speed and angle out. It uses no `Kt`
                // and no Park transform, which is the whole reason this model exists -- if a law that carries `p*Kt`
                // comes out the same here, the derivation has a witness that never mentions that product.
                // 同一子步的机械半：磁共能转矩进，转速与角度出。它不使用 `Kt`、也不做 Park 变换，这正是本模型存在的理由
                // —— 若一条带 `p*Kt` 的规律在这里得到同样的结果，那个推导就有了一个根本不提这个乘积的证人。
                //
                // The injected cogging torque is added here and nowhere else: it is a rotor torque, so it belongs in the
                // mechanical equation and must NOT be folded into `torque_at()`, which is the electromagnetic torque the
                // cross-check and the compensation are both measured against.
                // 注入的齿槽转矩只加在这里，不加别处：它是转子转矩，属于机械方程，**不能**并进 `torque_at()`——后者是对拍与
                // 补偿共同对照的那个电磁转矩。
                const float te = torque_at(iu, iv, theta);
                const float tcog = ::foc::sil::evaluate_cogging(
                    cogging_, cogging_count_, static_cast<float>(theta) / static_cast<float>(params_.pole_pairs));
                omega_m_ =
                    static_cast<float>(omega_m_ + (h * ((((te + tcog) - (mech_.b * omega_m_)) - load_nm_) / mech_.j)));
                omega_e_ = static_cast<float>(params_.pole_pairs) * omega_m_;
            }
            const Derivatives k1 = derivative(iu, iv, theta);
            const Derivatives k2 =
                derivative(static_cast<float>(iu + (0.5F * h * k1.du)), static_cast<float>(iv + (0.5F * h * k1.dv)),
                           static_cast<float>(theta + (0.5F * h * omega_e_)));
            const Derivatives k3 =
                derivative(static_cast<float>(iu + (0.5F * h * k2.du)), static_cast<float>(iv + (0.5F * h * k2.dv)),
                           static_cast<float>(theta + (0.5F * h * omega_e_)));
            const Derivatives k4 =
                derivative(static_cast<float>(iu + (h * k3.du)), static_cast<float>(iv + (h * k3.dv)),
                           static_cast<float>(theta + (h * omega_e_)));
            iu = static_cast<float>(iu + ((h / 6.0F) * ((k1.du + (2.0F * k2.du)) + ((2.0F * k3.du) + k4.du))));
            iv = static_cast<float>(iv + ((h / 6.0F) * ((k1.dv + (2.0F * k2.dv)) + ((2.0F * k3.dv) + k4.dv))));
            theta = static_cast<float>(theta + (h * omega_e_));
        }
        theta_e_ = theta;
        i_u_ = iu;
        i_v_ = iv;
    }

    /**
     * @brief [en] The three phase currents, A.
     * @brief [zh] 三相电流，A。
     */
    [[nodiscard]] auto current() const noexcept -> core::Uvw<float> {
        return core::Uvw<float>{i_u_, i_v_, static_cast<float>(-i_u_ - i_v_)};
    }

    /**
     * @brief [en] Electromagnetic torque from magnetic co-energy, **without Park**: N*m.
     * @brief [zh] 由磁共能给出的电磁转矩，**不经 Park**：N·m。
     */
    [[nodiscard]] auto torque() const noexcept -> float { return torque_at(i_u_, i_v_, theta_e_); }

    /**
     * @brief [en] Mechanical power balance: `Te*omega_m = v.i - Rs*|i|^2` at steady state. Reported so the test can
     *             check the lossless case too.
     * @brief [zh] 机械功率平衡：稳态下 `Te·ω_m = v·i − Rs·|i|²`。一并给出，好让测试也能检查无损耗情形。
     */
    [[nodiscard]] auto electrical_power() const noexcept -> float {
        const core::Uvw<float> i = current();
        return static_cast<float>((v_.u * i.u) + (v_.v * i.v) + (v_.w * i.w));
    }

    /**
     * @brief [en] The electrical speed the model is frozen at, rad/s.
     * @brief [zh] 模型冻结的电角速度，rad/s。
     */
    [[nodiscard]] auto omega_e() const noexcept -> float { return omega_e_; }

  private:
    struct Derivatives final {
        float du;
        float dv;
    };

    static constexpr float kTwoThirdPi = 2.0943951F;

    /**
     * @brief [en] Co-energy torque at an explicit state, so the integrator can ask without mutating first.
     * @brief [zh] 给定显式状态的磁共能转矩，好让积分器不必先改状态再问。
     */
    [[nodiscard]] auto torque_at(float iu, float iv, float theta) const noexcept -> float {
        const float iw = static_cast<float>(-iu - iv);
        float sum = 0.0F;
        for (int phase = 0; phase < 3; ++phase) {
            const float angle = static_cast<float>(theta - (kTwoThirdPi * static_cast<float>(phase)));
            const float phase_i = (phase == 0) ? iu : ((phase == 1) ? iv : iw);
            sum = static_cast<float>(sum + (phase_i * std::sin(angle)));
        }
        return -static_cast<float>(params_.pole_pairs) * params_.lambda_m * sum;
    }

    /**
     * @brief [en] `di/dt` per phase from `v = Rs*i + (Ls-M)*di/dt + d(lambda_m)/dt`, with the magnet term
     *             differentiated by hand: `d/dt cos(theta) = -omega*sin(theta)`.
     * @brief [zh] 每相的 `di/dt`，来自 `v = Rs·i + (Ls−M)·di/dt + dλm/dt`，磁钢项手写求导： `d/dt cos θ = −ω·sin θ`。
     */
    [[nodiscard]] auto derivative(float iu, float iv, float theta) const noexcept -> Derivatives {
        const float back_u = params_.lambda_m * std::sin(theta);
        const float back_v = params_.lambda_m * std::sin(theta - kTwoThirdPi);
        const float scale = 1.0F / params_.ls_minus_m;
        return Derivatives{static_cast<float>((v_.u - (params_.rs * iu) + (omega_e_ * back_u)) * scale),
                           static_cast<float>((v_.v - (params_.rs * iv) + (omega_e_ * back_v)) * scale)};
    }

    Params params_{};
    Mechanics mech_{0.0F, 0.0F};
    core::Uvw<float> v_{0.0F, 0.0F, 0.0F};
    CoggingHarmonic cogging_[kMaxCogging]{};
    int cogging_count_{0};
    float i_u_{0.0F};
    float i_v_{0.0F};
    float theta_e_{0.0F};
    float omega_e_{0.0F};
    float omega_m_{0.0F};
    float load_nm_{0.0F};
    bool mechanics_on_{false};
};

} // namespace sil

} // namespace foc
