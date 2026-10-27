// Conan::ImportStart
#pragma once
#include <foc/core/machine_params.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Back-EMF flux-linkage observer (open-loop integration), V1's "real" observer.
 *        [zh] 反电动势磁链观测器（开环积分），V1 的"真实可用"观测器。
 *
 *        Model: `psi_ab = integral(v_ab - Rs * i_ab) dt - L * i_ab`, angle = atan2(psi_beta, psi_alpha).
 *        Units are SI; the V1 build assumes `Ld == Lq` (surface-mounted PMSM).
 *        模型：`psi_ab = ∫(v_ab - Rs·i_ab)dt - L·i_ab`，角度 = atan2(psi_beta, psi_alpha)。
 *        SI 单位；V1 假设 `Ld == Lq`（表贴式 PMSM）。
 *
 * @warning [en] A pure integrator cannot know its integration constant, so the **absolute** angle
 *          needs a known initial flux. Seed it with `seed_flux()` during/after the align phase
 *          (the motor layer knows psi_m = Kt / (1.5 * pole_pairs) from the nameplate). Without
 *          seeding the estimate keeps a constant offset of `-psi(0)`; integrator drift over time
 *          still needs a PLL/SMO observer (P2).
 *          [zh] 纯积分器无法知道积分常数，**绝对角**需要已知初始磁链：在 Align 阶段/之后用
 *          `seed_flux()` 播种（电机层可从铭牌得到 psi_m = Kt / (1.5 * pole_pairs)）。不播种时
 *          估计值会带一个 `-psi(0)` 的恒定偏移；长期漂移仍需 PLL/SMO 观测器（P2）。
 *
 * @context [en] Current-loop ISR after construction: noexcept, no allocation.
 *           [zh] 构造之后的电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class BackEmfObserver final {
  public:
    /**
     * @brief [en] Electrical parameters of the machine model.
     *        [zh] 电机模型电气参数。
     *
     * @since 0.1.0
     */
    struct Config final {
        float rs; ///< [en] stator resistance in ohm, >= 0. [zh] 定子电阻，Ω，>= 0。
        float l;  ///< [en] stator inductance in H, > 0 (V1: Ld == Lq). [zh] 定子电感，H，> 0（V1：Ld == Lq）。
    };

    /**
     * @brief [en] Construct with the machine parameters.
     *        [zh] 用电机参数构造。
     *
     * @param config [en] invalid values put the observer in Param status (update then holds 0).
     *               [zh] 非法值会让观测器进入 Param 状态（之后 update 保持 0）。
     * @since 0.1.0
     */
    explicit BackEmfObserver(Config config) noexcept;

    /**
     * @brief [en] Seed the flux linkage estimate (align phase): the observer's residual integration
     *        constant is chosen so that the current estimate equals `psi_ab`.
     *        [zh] 播种磁链估计（对齐阶段）：观测器会把剩余积分常数选成"当前估计等于 `psi_ab`"。
     *
     * @param psi_ab [en] flux linkage in Wb at the current instant. [zh] 当前时刻的磁链，Wb。
     * @since 0.1.0
     */
    void seed_flux(Ab<float> psi_ab) noexcept;

    /**
     * @brief [en] Take an updated machine description.
     *        [zh] 接受更新后的机器描述。
     *
     * @param params [en] the machine as it actually is. [zh] 机器实际的样子。
     * @note [en] `l` becomes `params.lq`, not the average: this observer is used with `id ~ 0`, where the
     *       current sits on the q axis and the flux it produces is `Lq*i`. With field weakening that
     *       assumption stops holding -- which is one more reason field weakening needs a different
     *       observer rather than a parameter change.
     *       [zh] `l` 取 `params.lq` 而不是平均值：本观测器用于 `id ≈ 0`，此时电流坐在 q 轴上，它产生的
     *       磁链是 `Lq·i`。一旦进入弱磁这个假设就不成立——这也是弱磁需要换观测器而不是换参数的原因之一。
     * @since 0.1.0
     */
    void apply_params(const MachineParams& params) noexcept;

    /**
     * @brief [en] Advance the observer by one current-loop step.
     *        [zh] 推进一个电流环步。
     *
     * @param v_ab [en] applied stator voltage in V. [zh] 施加的定子电压，V。
     * @param i_ab [en] measured stator current in A. [zh] 实测定子电流，A。
     * @param dt_s [en] step length in s; <= 0 is rejected (status Param, state holds).
     *              [zh] 步长，s；<= 0 被拒绝（状态置 Param，输出保持）。
     * @return [en] estimated electrical angle and speed. [zh] 估计的电角度与电角速度。
     * @since 0.1.0
     */
    auto update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState;

    /**
     * @brief [en] Clear the integrator and the speed estimate.
     *        [zh] 清除积分器与速度估计。
     * @since 0.1.0
     */
    void reset() noexcept;

    /**
     * @brief [en] Ok, or Param after an invalid configuration or non-positive dt.
     *        [zh] Ok，或非法配置 / 非正 dt 后的 Param。
     * @since 0.1.0
     */
    [[nodiscard]] auto status() const noexcept -> ErrorCode;

    /**
     * @brief [en] Current flux linkage estimate in Wb (diagnostics).
     *        [zh] 当前磁链估计，Wb（诊断用）。
     * @since 0.1.0
     */
    [[nodiscard]] auto flux() const noexcept -> Ab<float>;

  private:
    Config config_;
    Ab<float> psi_;    ///< [en] flux linkage estimate in Wb (the integrator state). [zh] 磁链估计，Wb（积分器状态）。
    Ab<float> i_prev_; ///< [en] previous current sample; makes the L*di term exact. [zh] 上次电流采样，使 L·di 项精确。
    ObsState state_;
    ErrorCode status_;
    bool primed_;
    bool theta_primed_;
};

} // namespace core

} // namespace foc
