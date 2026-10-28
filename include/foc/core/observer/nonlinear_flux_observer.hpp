// Conan::ImportStart
#pragma once
#include <foc/core/machine_params.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Nonlinear flux observer: the same integral as `BackEmfObserver`, plus a term that pulls the
 *        estimated permanent-magnet flux magnitude back to the nameplate value.
 *        [zh] 非线性磁链观测器：与 `BackEmfObserver` 相同的积分，外加一项把估计的永磁磁链幅值拉回铭牌值的修正。
 *
 *        The one line that differs, in words: the open-loop observer integrates `v - Rs*i` and hopes the
 *        result stays a circle of radius `psi_f`. Nothing makes it -- a wrong `Rs` tilts and stretches that
 *        circle, an injected carrier wobbles it, and an integrator never forgets what it was given. This
 *        observer adds a correction along the flux vector that vanishes when `|eta| = psi_f` and pushes
 *        otherwise, so the circle has a restoring force.
 *        差别就一行，用文字说：开环观测器积分 `v − Rs·i`，然后**指望**结果停在半径 `psi_f` 的圆上。没有
 *        任何东西在保证这一点——`Rs` 偏了会把那个圆拉歪、载波会让它抖、而积分器从不忘记收到过什么。
 *        本观测器沿磁链矢量加一项修正：`|eta| = psi_f` 时为零，否则往回推，于是那个圆有了恢复力。
 *
 * @note [en] Structure follows the nonlinear observer for surface-mount PMSMs described by Lee, Hong, Nam,
 *       Ortega and Praly (*IEEE Transactions on Power Electronics*, 2010): a flux prediction, an `eta` that
 *       removes the current's own flux, and a correction proportional to `(psi_f^2 - |eta|^2) * eta`. The
 *       implementation here is written from that mathematical statement, not copied from any source.
 *       [zh] 结构来自 Lee / Hong / Nam / Ortega / Praly 关于表贴式 PMSM 非线性观测器的论文（*IEEE TPEL*，
 *       2010）：磁链预测、去掉电流自身磁链的 `eta`、以及正比于 `(ψf² − |η|²)·η` 的修正。此处按该数学表述
 *       独立实现，未拷贝任何源码。
 *
 * @warning [en] It needs a seed. With `eta = 0` the correction has no direction to push in, so an unseeded
 *          observer stays at zero exactly like the open-loop one -- and, like it, the absolute angle needs
 *          the seed the motor layer writes from the nameplate during start-up.
 *          [zh] 它需要播种。`eta = 0` 时修正没有可推的方向，因此未播种的观测器与开环观测器一样停在零——
 *          并且同样地，绝对角需要电机层在起动时按铭牌写入的种子。
 *
 * @context [en] Current-loop ISR after construction: noexcept, no allocation. [zh] 构造之后的电流环 ISR：
 *           noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class NonlinearFluxObserver final {
  public:
    /**
     * @brief [en] Machine numbers plus the correction gain.
     *        [zh] 机器数字加上修正增益。
     * @since 0.1.0
     */
    struct Config final {
        float rs; ///< [en] stator resistance in ohm, >= 0. [zh] 定子电阻，Ω，>= 0。
        float l;  ///< [en] stator inductance in H, > 0 (single-L model). [zh] 定子电感，H，> 0（单一 L 模型）。
        float
            psi_f; ///< [en] rotor flux linkage the correction pulls towards, Wb, > 0. [zh] 修正所拉向的转子磁链，Wb，> 0。
        float
            gain_ratio; ///< [en] correction rate as a fraction of the estimated electrical speed, > 0. [zh] 修正速率占估计电角速度的比例，> 0。
        float min_gain_rad_s; ///< [en] floor for that rate, > 0. [zh] 该速率的下限，rad/s，> 0。
        float
            min_flux_wb; ///< [en] below this estimated flux no answer is reported, > 0. [zh] 低于此估计磁链即不报答案，> 0。
    };

    /**
     * @brief [en] Construct and validate.
     *        [zh] 构造并校验。
     * @param config [en] invalid values put the observer in Param status; update() then holds.
     *               [zh] 非法值让观测器进入 Param 状态；此后 update() 保持。
     * @since 0.1.0
     */
    explicit NonlinearFluxObserver(Config config) noexcept;

    /**
     * @brief [en] Seed the flux estimate; also the only way an unseeded observer can start.
     *        [zh] 播种磁链估计；也是未播种观测器唯一可能的起点。
     * @param psi_ab [en] flux linkage in Wb at the current instant. [zh] 当前时刻的磁链，Wb。
     * @since 0.1.0
     */
    void seed_flux(Ab<float> psi_ab) noexcept;

    /**
     * @brief [en] Take an updated machine description.
     *        [zh] 接受更新后的机器描述。
     *
     * @param params [en] the machine as it actually is. [zh] 机器实际的样子。
     * @note [en] Unlike the open-loop observer this one does *not* need a re-seed to shed the error a wrong
     *       parameter left behind: the correction keeps pulling towards `psi_f`, so it converges on its own.
     *       That is the whole point of the extra term, and it is measured.
     *       [zh] 与开环观测器不同，这一个**不需要**重新播种来摆脱错误参数留下的误差：修正会持续把幅值拉向
     *       `ψf`，因此它自行收敛。这正是那一项存在的意义，有实测。
     * @since 0.1.0
     */
    void apply_params(const MachineParams& params) noexcept;

    /**
     * @brief [en] Advance one current-loop step.
     *        [zh] 推进一个电流环步。
     * @param v_ab [en] applied stator voltage in V. [zh] 施加的定子电压，V。
     * @param i_ab [en] measured stator current in A. [zh] 实测定子电流，A。
     * @param dt_s [en] step length in s; <= 0 is rejected (status Param). [zh] 步长，s；<= 0 被拒绝（状态 Param）。
     * @return [en] estimated electrical angle and speed. [zh] 估计的电角度与电角速度。
     * @since 0.1.0
     */
    auto update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState;

    /// [en] Clear the integrator, the speed estimate and the prime flag. [zh] 清除积分器、转速估计与 priming 标志。
    void reset() noexcept;

    /// [en] Ok, or Param after an invalid configuration or non-positive dt. [zh] Ok，或非法配置 / 非正 dt 后的 Param。
    [[nodiscard]] auto status() const noexcept -> ErrorCode;

    /// [en] Whether the estimated flux is large enough for the answer to mean anything. [zh] 估计磁链是否大到答案有意义。
    [[nodiscard]] auto locked() const noexcept -> bool;

    /// [en] Current flux estimate in Wb (diagnostics). [zh] 当前磁链估计，Wb（诊断用）。
    [[nodiscard]] auto flux() const noexcept -> Ab<float>;

    /// [en] The estimate as last published (diagnostics). [zh] 最近一次发布的估计（诊断用）。
    [[nodiscard]] auto state() const noexcept -> ObsState;

  private:
    Config config_;
    Ab<float> psi_;    ///< [en] PM flux estimate in Wb (the integrator state). [zh] 永磁磁链估计，Wb（积分器状态）。
    Ab<float> i_prev_; ///< [en] previous current sample; makes the L*di term exact. [zh] 上次电流采样，使 L·di 项精确。
    ObsState state_;
    ErrorCode status_;
    bool locked_;
    bool primed_;
    bool theta_primed_;
};


} // namespace core

} // namespace foc
