// Conan::ImportStart
#pragma once
#include <foc/core/filters.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Saliency-tracking observer: injects a high-frequency voltage on the estimated d axis and
 *        reads the rotor angle out of the current response. Works at standstill, where a back-EMF
 *        observer has nothing to integrate.
 *        [zh] 凸极跟踪观测器：在估计的 d 轴注入高频电压，从电流响应中读出转子角度。静止时即可工作，
 *        而这正是反电动势观测器无事可做的地方。
 *
 *        Why this cannot be a back-EMF variant: back-EMF scales with speed, so at zero speed it is
 *        zero and the angle is unobservable. Saliency does not scale with speed at all -- it is a
 *        property of the magnetic circuit. That is why the two are complements, not alternatives.
 *        为什么它不可能是反电动势观测器的变体：反电动势与转速成正比，零速时为零，角度不可观测。凸极
 *        与转速完全无关——它是磁路的属性。两者互补，不是二选一。
 *
 *        Signal chain, one current-loop step:
 *          v_inj = V_h * sin(w_h t) along the estimated d axis
 *          eps   = LPF2{ 2 * (i_ab . q_est) * cos(w_h t) } / demod_gain
 *          eps  ~= -sin(2 * angle_error)
 *        The error drives a type-2 PLL whose gains come from a bandwidth and a damping ratio, so the
 *        numbers a caller tunes are physical instead of machine-specific.
 *        信号链，每个电流环步：
 *          v_inj = V_h·sin(w_h t)，方向为估计 d 轴
 *          eps   = LPF2{ 2·(i_ab·q_est)·cos(w_h t) } / demod_gain
 *          eps  ≈ -sin(2·角度误差)
 *        该误差驱动二阶 PLL，其增益由带宽与阻尼比导出，于是调用方调的是物理量而非机器特定的数字。
 *
 * @warning [en] Three limitations that are properties of the method, not bugs to be found later:
 *          1. **pi/2 envelope.** `sin(2*dtheta)` only pulls correctly for `|dtheta| < pi/2`. The loop
 *             converges for an initial error inside that window and drifts to the `dtheta = pi`
 *             mirror outside it. Start it from an alignment pulse or from a previous estimate.
 *          2. **Magnet polarity is NOT resolved.** `dtheta = pi` is also a stable equilibrium (the
 *             d axis is right, the magnet is backwards, so the torque sign is inverted). Telling the
 *             two apart needs a separate pulse/saturation test, which is *not* implemented here.
 *          3. **Low speed only.** The fundamental current and back-EMF appear as low-frequency
 *             content that survives the demodulation filter, so above a few hertz electrical the
 *             signal drowns. Blending with a back-EMF observer is the usual answer and is *not*
 *             implemented here.
 *          [zh] 三条方法本身的限制，不是以后才会发现的 bug：
 *          1. **pi/2 包络**：`sin(2·dtheta)` 只在 `|dtheta| < pi/2` 内正确牵引。初值在此窗口内会收敛，
 *             窗口外会漂到 `dtheta = pi` 的镜像点。请用对齐脉冲或上一次估计作为起点。
 *          2. **不解决磁极极性**：`dtheta = pi` 也是稳定平衡点（d 轴对了、磁钢装反，于是转矩符号反）。
 *             区分二者需要单独的脉冲/饱和测试，**此处未实现**。
 *          3. **仅低速**：基频电流与反电动势经解调滤波后仍有低频残留，电频率超过几 Hz 后信号就被淹没。
 *             常规做法是与反电动势观测器混合，**此处未实现**。
 *
 * @context [en] Current-loop ISR after construction: noexcept, no allocation. [zh] 构造之后的电流环 ISR：
 *           noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class HfiObserver final {
  public:
    /**
     * @brief [en] Injection, demodulation and loop-shaping parameters.
     *        [zh] 注入、解调与环路整形参数。
     *
     * @note [en] Every field is a physical quantity. There is no `enabled` flag: an injection
     *       amplitude of zero is rejected rather than treated as "off", because an observer that
     *       silently does nothing is indistinguishable from one that is broken.
     *       [zh] 每个字段都是物理量。没有 `enabled` 开关：注入幅值 0 会被拒绝而不是当作“关闭”，
     *       因为一个静默无作为的观测器与一个坏掉的观测器无法区分。
     * @since 0.1.0
     */
    struct Config final {
        float inject_v;         ///< [en] injection amplitude, peak V, > 0. [zh] 注入幅值，峰值 V，> 0。
        float inject_hz;        ///< [en] injection frequency in Hz, > demod_lpf_hz. [zh] 注入频率，Hz，> demod_lpf_hz。
        float demod_lpf_hz;     ///< [en] demodulation filter cutoff in Hz. [zh] 解调滤波截止频率，Hz。
        float pll_bandwidth_hz; ///< [en] PLL natural frequency in Hz, > 0. [zh] PLL 自然频率，Hz，> 0。
        float pll_zeta; ///< [en] PLL damping ratio, > 0 (0.7..1.0 typical). [zh] PLL 阻尼比，> 0（常用 0.7..1.0）。
        float
            demod_gain_a; ///< [en] signed sensitivity, see predicted_gain_a(); != 0. [zh] 带符号灵敏度，见 predicted_gain_a()；!= 0。
        float
            min_hf_current_a; ///< [en] HF current amplitude below which "no lock" is reported, >= 0. [zh] 低于此高频电流幅值即报告“未锁定”，>= 0。
    };

    /**
     * @brief [en] Construct and validate.
     *        [zh] 构造并校验。
     *
     * @param config [en] invalid values put the observer in Param status; update() then holds its
     *               state instead of reporting an angle nobody measured.
     *               [zh] 非法值会让观测器进入 Param 状态；此后 update() 保持状态，而不是报一个没人测过
     *               的角度。
     * @since 0.1.0
     */
    explicit HfiObserver(Config config) noexcept;

    /**
     * @brief [en] The small-signal sensitivity of the demodulated signal: `V_h*(Ld-Lq)/(2*w_h*Ld*Lq)`
     *        in A. Signed, because the sign *is* the saliency direction: negative for a normal
     *        interior machine (`Lq > Ld`), positive when `Ld > Lq`.
     *        [zh] 解调信号的小信号灵敏度：`V_h·(Ld-Lq)/(2·w_h·Ld·Lq)`，单位 A。带符号，因为符号**就是**
     *        凸极方向：常规内置式（`Lq > Ld`）为负，`Ld > Lq` 时为正。
     *
     * @note [en] This is a *first-order* prediction: it ignores `Rs`, which is why it is only a
     *       starting value for Config::demod_gain_a. The error is `w_h*Ld/(Rs^2 + (w_h*Ld)^2)`-ish
     *       and shrinks as the injection frequency rises -- at `w_h*Ld = 7*Rs` the measured value was
     *       10% below this formula. Measure it when the machine is on the bench.
     *       [zh] 这是**一阶**预测：忽略了 `Rs`，因此只作 Config::demod_gain_a 的起点。误差量级约
     *       `w_h·Ld/(Rs² + (w_h·Ld)²)`，注入频率越高越小——在 `w_h·Ld = 7·Rs` 时实测比本式低 10%。
     *       有台架时实测为准。
     * @since 0.1.0
     */
    [[nodiscard]] static auto predicted_gain_a(float inject_v, float inject_hz, float ld, float lq) noexcept -> float;

    /**
     * @brief [en] Advance one current-loop step.
     *        [zh] 推进一个电流环步。
     *
     * @param v_ab [en] applied stator voltage in V. Deliberately *unused*: the observer knows the
     *               injection it asked for, and a saliency estimate that depended on the voltage
     *               command would inherit the current loop's error. Named for the contract.
     *               [zh] 施加的定子电压，V。**故意不用**：观测器知道自己请求的注入量，而依赖电压指令的
     *               凸极估计会继承电流环的误差。保留名字是为了契约。
     * @param i_ab [en] measured stator current in A -- the only input that carries the rotor angle.
     *               [zh] 实测定子电流，A——唯一携带转子角度信息的输入。
     * @param dt_s [en] step length in s; <= 0 is rejected (status Param, state holds).
     *              [zh] 步长，s；<= 0 被拒绝（状态置 Param，输出保持）。
     * @return [en] estimated electrical angle and speed. [zh] 估计的电角度与电角速度。
     * @since 0.1.0
     */
    auto update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState;

    /**
     * @brief [en] The voltage to add to the command for the period that starts now, oriented along
     *        the current estimate. The caller adds it; the observer never touches the modulator.
     *        [zh] 即将开始的那个周期要叠加到指令上的电压，方向取当前估计角。由调用方叠加；观测器不碰
     *        调制器。
     *
     * @note [en] Calling this is what advances the injection phase's *use*, but the phase itself
     *       advances inside update(). A caller that stops calling update() stops injecting instead
     *       of silently injecting a constant voltage.
     *       [zh] 调用它只是把注入相位“用掉”，相位本身在 update() 里推进。停止调用 update() 的调用方
     *       会停止注入，而不是悄悄注入一个直流电压。
     * @since 0.1.0
     */
    [[nodiscard]] auto injection_ab() const noexcept -> Ab<float>;

    /**
     * @brief [en] Clear the PLL, the filters and the injection phase.
     *        [zh] 清除 PLL、滤波器与注入相位。
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
     * @brief [en] Whether the HF current response is large enough for the estimate to mean anything.
     *        [zh] 高频电流响应是否大到足以让估计有意义。
     *
     * @note [en] `ObsState` has no validity channel, so an unlocked observer cannot say "unknown"
     *       through the contract: it holds the last angle and reports zero speed instead. This
     *       accessor is how a caller finds out that the number is a hold rather than a measurement.
     *       Adding a validity flag to `ObsState` would be the proper fix and changes every observer.
     *       [zh] `ObsState` 没有有效性通道，因此未锁定的观测器无法通过契约表达“未知”：它保持上一个
     *       角度并报零速。调用方靠这个访问器得知那个数字是保持值而非测量值。给 `ObsState` 加有效性
     *       标志才是正解，但那会改动每一个观测器。
     * @since 0.1.0
     */
    [[nodiscard]] auto locked() const noexcept -> bool;

    /**
     * @brief [en] Normalised error signal (diagnostics): about `-sin(2*angle_error)` when locked.
     *        [zh] 归一化误差信号（诊断用）：锁定时约为 `-sin(2·角度误差)`。
     * @since 0.1.0
     */
    [[nodiscard]] auto error() const noexcept -> float;

    /**
     * @brief [en] The estimate as last published (diagnostics; update() also returns it).
     *        [zh] 最近一次发布的估计值（诊断用；update() 也会返回它）。
     * @since 0.1.0
     */
    [[nodiscard]] auto state() const noexcept -> ObsState;

    /**
     * @brief [en] Demodulated HF current amplitude in A (diagnostics and the lock detector).
     *        [zh] 解调得到的高频电流幅值，A（诊断与锁定判据）。
     * @since 0.1.0
     */
    [[nodiscard]] auto hf_current_a() const noexcept -> float;

  private:
    auto configure_filters(float dt_s) noexcept -> ErrorCode;

    Config config_;
    FirstOrderLowPass demod_lpf_a_;
    FirstOrderLowPass demod_lpf_b_;
    FirstOrderLowPass amplitude_lpf_;
    ObsState state_;
    ErrorCode status_;
    float
        phase_applied_; ///< [en] injection phase of the voltage in effect during the last period. [zh] 上一周期实际生效电压的注入相位。
    float theta_applied_; ///< [en] estimate that oriented that voltage. [zh] 该电压所用的方向角。
    float omega_int_;     ///< [en] PLL integral state, electrical rad/s. [zh] PLL 积分状态，电角速度。
    float error_;         ///< [en] normalised error, diagnostics. [zh] 归一化误差，诊断用。
    float hf_amp_;        ///< [en] demodulated HF amplitude in A. [zh] 解调高频幅值，A。
    float dt_configured_; ///< [en] period the filters were built for. [zh] 滤波器配置所用的周期。
    bool primed_;
    bool locked_;
};


} // namespace core

} // namespace foc
