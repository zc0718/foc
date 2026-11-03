// Conan::ImportStart
#pragma once
#include <cstdint>

#include <foc/core/machine_params.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Dual-inductance flux observer: the same integrator as `BackEmfObserver`, but the rotor angle is recovered
 *             from the saliency instead of from a single `L`.
 *
 * @brief [zh] 双电感磁链观测器：与 `BackEmfObserver` 相同的积分器，但转子角从凸极性反解，而不是靠单一 `L` 减掉电流磁
 *             链。
 *
 *     The stator flux of a salient machine is a **position-dependent** quantity, not "rotor flux plus `L*i`":
 *
 *     psi = psi_f * e^(j*theta) + L0 * i + L1 * conj(i) * e^(j*2*theta), L0 = (Ld + Lq) / 2, L1 = (Ld - Lq) / 2
 *
 *     `BackEmfObserver` assumes `L1 = 0` and answers `theta = arg(psi - L*i)`, which is **exact only for Ld == Lq**. On
 *     a salient machine the neglected `L1` term is a second harmonic of the electrical angle whose amplitude is
 *     `|L1|*|i|` -- 8.5 % of the rotor flux at 100 A with the Prius pair (`Ld = 0.13 mH`, `Lq = 0.29 mH`, `psi_f =
 *     0.0947 Wb`) -- so the single-`L` answer is wrong by an angle that grows with current, and with it the whole
 *     reason `MtpaReference` may not be paired with a single-`L` observer. This observer keeps the same integrator (`d
 *     psi / dt = v - Rs * i`, with no inductance term at all) and inverts the relation above exactly: with `x =
 *     e^(j*theta)` the relation is a **quadratic in `x`**, so the answer is closed form and needs no iteration, no
 *     allocation and no tuning.
 *
 *     凸极电机的定子磁链是**与位置相关**的量，不是"转子磁链加上 `L·i`"：
 *
 *     psi = psi_f·e^(jθ) + L0·i + L1·conj(i)·e^(j2θ)，L0 = (Ld+Lq)/2，L1 = (Ld−Lq)/2
 *
 *     `BackEmfObserver` 假设 `L1 = 0`，给出 `θ = arg(psi − L·i)`，这只在 `Ld == Lq` 时精确。凸极机上被丢掉的 `L1` 项是
 *     电角度二次谐波，幅值 `|L1|·|i|`——用 Prius 那对电感（`Ld = 0.13 mH`、`Lq = 0.29 mH`、`psi_f = 0.0947 Wb`）在 100 A
 *     时达到转子磁链的 8.5%——所以单 `L` 的答案会有一个随电流增长的角度误差，而"`MtpaReference` 不能与单 `L` 观测器配对"
 *     正是因为这一点。本观测器保留同一个积分器（`dψ/dt = v − Rs·i`，完全不含电感项），并精确反解上式：令 `x = e^(jθ)`，
 *     上式就是关于 `x` 的**二次方程**，因此答案是闭式的，不需要迭代、不分配、也没有可调增益。
 *
 * @warning [en] Like the open-loop integrator it generalises, the **absolute** angle needs a seed: a pure integrator
 *               cannot know its integration constant. `seed_flux()` takes the full stator flux (not the rotor flux),
 *               and the two roots of the quadratic are disambiguated by continuity with the previous estimate -- on the
 *               first step after a seed, with the angle the seed implies.
 * @warning [zh] 与它所推广的开环积分器一样，**绝对角**需要播种：纯积分器无从知道积分常数。`seed_flux()` 接收的是完整定
 *               子磁链（不是转子磁链），二次方程的两个根靠"与上一次估计连续"来区分——播种后的第一步，则与种子隐含的角度
 *               连续。
 * @context [en] Current-loop ISR after construction: noexcept, no allocation, no iteration count that depends on data.
 * @context [zh] 构造之后的电流环 ISR：noexcept、不分配、迭代次数不依赖数据。
 * @since 0.1.11
 * @exporter
 */
class DualInductanceObserver final {
  public:
    /**
     * @brief [en] Electrical parameters, with the two inductances kept apart and the rotor flux made explicit.
     * @brief [zh] 电气参数：两个电感分开保留，转子磁链显式给出。
     * @since 0.1.11
     */
    struct Config final {
        float rs;    ///< [en] stator resistance in ohm, >= 0. [zh] 定子电阻，Ω，>= 0。
        float ld;    ///< [en] d-axis inductance in H, > 0. [zh] d 轴电感，H，> 0。
        float lq;    ///< [en] q-axis inductance in H, > 0. [zh] q 轴电感，H，> 0。
        float psi_f; ///< [en] rotor flux linkage in Wb, > 0. [zh] 转子磁链，Wb，> 0。
    };

    /**
     * @brief [en] Construct with the machine parameters.
     * @brief [zh] 用电机参数构造。
     * @param config [en] invalid values put the observer in Param status (update then holds 0).
     * @param config [zh] 非法值会让观测器进入 Param 状态（之后 update 保持 0）。
     * @since 0.1.11
     */
    explicit DualInductanceObserver(Config config) noexcept;

    /**
     * @brief [en] Seed the **stator** flux estimate.
     * @brief [zh] 播种**定子**磁链估计。
     * @param psi_ab [en] the full stator flux in Wb at the current instant -- `psi_f*e^(j*theta) + L0*i + L1*conj(i)
     *                    *e^(j*2*theta)`, not the rotor flux alone.
     * @param psi_ab [zh] 当前时刻的**完整**定子磁链，Wb——`psi_f·e^(jθ) + L0·i + L1·conj(i)·e^(j2θ)`，不是单独的转子磁
     *                    链。
     * @note [en] The seed does not need to be exact: it only breaks the tie between the two roots of the quadratic, and
     *            a wrong seed costs a fixed angle offset exactly as it does in the open-loop observer.
     * @note [zh] 种子不必精确：它只用于在二次方程的两个根之间定夺，种子有偏时带来的就是这个固定角度偏移——与开环观测器完
     *            全相同。
     * @since 0.1.11
     */
    void seed_flux(Ab<float> psi_ab) noexcept;

    /**
     * @brief [en] Take an updated machine description.
     * @brief [zh] 接受更新后的机器描述。
     * @param params [en] the machine as it actually is.
     * @param params [zh] 机器实际的样子。
     * @note [en] `ld`, `lq` and `psi_f` all come from `params`; this is the observer that has no "which inductance?"
     *            question to answer, which is the whole difference from `BackEmfObserver::apply_params`.
     * @note [zh] `ld`、`lq`、`psi_f` 都取自 `params`；这个观测器没有"用哪个电感"这个问题要回答，这正是它与
     *            `BackEmfObserver::apply_params` 的全部区别。
     * @since 0.1.11
     */
    void apply_params(const MachineParams& params) noexcept;

    /**
     * @brief [en] Advance the observer by one current-loop step.
     * @brief [zh] 推进一个电流环步。
     * @param v_ab [en] applied stator voltage in V, same sample instant as `i_ab`.
     * @param v_ab [zh] 施加的定子电压，V，与 `i_ab` 同一采样时刻。
     * @param i_ab [en] measured stator current in A.
     * @param i_ab [zh] 实测定子电流，A。
     * @param dt_s [en] step length in s; <= 0 is rejected (status Param, state holds).
     * @param dt_s [zh] 步长，s；<= 0 被拒绝（状态置 Param，输出保持）。
     * @return [en] estimated electrical angle and speed.
     * @return [zh] 估计的电角度与电角速度。
     * @note [en] `v_ab` must come from the *same* sample instant as `i_ab`, for the reason documented on
     *            `BackEmfObserver::update`.
     * @note [zh] `v_ab` 必须与 `i_ab` 取自同一采样时刻，理由见 `BackEmfObserver::update`。
     * @since 0.1.11
     */
    auto update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState;

    /**
     * @brief [en] What the last `update()` reported, without stepping again.
     * @brief [zh] 上一次 `update()` 报出的状态，不推进。
     * @return [en] The last `ObsState`; the initial one before any update.
     * @return [zh] 上一次的 `ObsState`；未推进过时是初始值。
     * @since 0.1.11
     */
    [[nodiscard]] auto state() const noexcept -> ObsState { return state_; }

    /**
     * @brief [en] Clear the integrator, the speed estimate and the root continuity.
     * @brief [zh] 清除积分器、速度估计与选根连续性。
     * @since 0.1.11
     */
    void reset() noexcept;

    /**
     * @brief [en] Ok, or Param after an invalid configuration or a non-positive dt.
     * @brief [zh] Ok，或非法配置 / 非正 dt 后的 Param。
     * @since 0.1.11
     */
    [[nodiscard]] auto status() const noexcept -> ErrorCode;

    /**
     * @brief [en] Current stator flux estimate in Wb (diagnostics).
     * @brief [zh] 当前定子磁链估计，Wb（诊断用）。
     * @since 0.1.11
     */
    [[nodiscard]] auto flux() const noexcept -> Ab<float>;

    /**
     * @brief [en] How the last step chose between the two roots of the quadratic.
     * @brief [zh] 上一步在二次方程的两个根之间是如何选择的。
     * @return [en] Angle gap in rad between the two candidates; large means the choice was not close.
     * @return [zh] 两个候选之间的角度间隔，rad；很大意味着这次选择并不"接近"。
     * @note [en] A diagnostic, not part of the estimate: the two roots are the two rotor positions consistent with the
     *            measured flux and current, and the gap between them is how much of the answer came from continuity
     *            rather than from the measurement. Near the q axis (where `i_d ~ 0` and the saliency carries almost no
     *            information) the gap is smallest, which is exactly where a caller should trust the angle least.
     * @note [zh] 这是诊断量，不是估计的一部分：两个根是"与实测磁链、电流一致的两个转子位置"，它们的间隔说明这个答案有多
     *            少来自连续性、多少来自测量。q 轴附近（`i_d ≈ 0`、凸极几乎不携带信息）间隔最小，而那正是调用方最不该相
     *            信这个角的地方。
     * @since 0.1.11
     */
    [[nodiscard]] auto root_gap_rad() const noexcept -> float { return root_gap_rad_; }

  private:
    Config config_;
    float l0_;      ///< [en] (ld + lq) / 2. [zh] (ld+lq)/2。
    float l1_;      ///< [en] (ld - lq) / 2; zero is the surface-mounted degeneracy. [zh] (ld−lq)/2；零即表贴式退化。
    Ab<float> psi_; ///< [en] stator flux estimate in Wb (the integrator state). [zh] 定子磁链估计，Wb（积分器状态）。
    float seed_theta_; ///< [en] angle the seed implied, for the first root choice. [zh] 种子隐含的角度，用于首次选根。
    ObsState state_;
    ErrorCode status_;
    bool theta_primed_;
    float root_gap_rad_;
};

} // namespace core

} // namespace foc
