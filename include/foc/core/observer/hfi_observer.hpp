// Conan::ImportStart
#pragma once
#include <foc/core/filters.hpp>
#include <foc/core/machine_params.hpp>
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
 *        Signal chain, one current-loop step: v_inj = V_h * sin(w_h t) along the estimated d axis eps = LPF2{ 2 * (i_ab
 *        . q_est) * cos(w_h t) } / demod_gain eps ~= -sin(2 * angle_error) The error drives a type-2 PLL whose gains
 *        come from a bandwidth and a damping ratio, so the numbers a caller tunes are physical instead of
 *        machine-specific.
 *        信号链，每个电流环步： v_inj = V_h·sin(w_h t)，方向为估计 d 轴 eps = LPF2{ 2·(i_ab·q_est)·cos(w_h t) } /
 *        demod_gain eps ≈ -sin(2·角度误差) 该误差驱动二阶
 *        PLL，其增益由带宽与阻尼比导出，于是调用方调的是物理量而非机器特定的数字。
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
     *
     * @note [en] Sizing `inject_v`: an injected carrier is not free. Any *other* observer running
     *       alongside this one -- a back-EMF flux integrator, say -- cannot tell the carrier from real
     *       back-EMF, and whatever it leaves behind stays there because an integrator has no reset. The
     *       residue is about one carrier period of volt-seconds, `inject_v / (2*pi*inject_hz)` in Wb,
     *       which shows up as `inject_v / (2*pi*inject_hz*psi_f)` radians of angle bias. Measured on
     *       S16: 2 V at 2 kHz on a 4.8 mWb rotor gave 0.031 rad against 0.033 predicted. So the
     *       amplitude is a trade -- enough current to read the saliency, little enough that the bias it
     *       leaves elsewhere is under what the application tolerates.
     *       [zh] 定容 `inject_v`：注入载波不是免费的。与它并行的**任何其他**观测器——比如反电动势磁链
     *       积分器——都分不清载波与真正的反电动势，而积分器没有复位，留下的东西就一直留着。残余量约为
     *       一个载波周期的伏秒，即 `inject_v / (2π·inject_hz)`，单位 Wb；换算成角度偏差就是
     *       `inject_v / (2π·inject_hz·ψf)` rad。S16 实测：4.8 mWb 转子上 2 V / 2 kHz 给出 0.031 rad，
     *       预测 0.033。因此幅值是一笔交易——大到能读出凸极，小到它在别处留下的偏差在应用可接受范围内。
     *
     * @note [en] Why `current_hp_hz` exists: the demodulator multiplies the measured current by the
     *       carrier, so *any* DC content in the current leaks through the follow-on filter at the
     *       carrier frequency and becomes a standing bias in the error signal. A load step at
     *       standstill is exactly that: a DC current appearing from nowhere. Measured on the S16
     *       fixture, 0.5 A of q-axis current biased the error by 0.034 -- enough to push a start-up
     *       over the `pi/2` boundary and lock the estimate to the mirror. The high-pass removes the
     *       fundamental before demodulation, and because the filter primes on its first sample a
     *       constant current never reaches the multiplier at all. Set it above the highest electrical
     *       frequency at which HFI is meant to work and well below `inject_hz`.
     *       [zh] `current_hp_hz` 存在的理由：解调器把实测电流与载波相乘，于是电流中**任何**直流成分
     *       都会以载波频率漏过后级滤波器，变成误差信号上的固定偏置。静止时的负载阶跃正好如此：凭空
     *       出现一个直流电流。在 S16 台架上实测，0.5 A 的 q 轴电流使误差偏置 0.034——足以把一次起动
     *       推过 `pi/2` 边界、锁到镜像上。高通在解调前去掉基频，且由于滤波器在第一个采样上自播种，
     *       恒定电流根本到不了乘法器。取值应在“HFI 预期工作的最高电频率”之上、`inject_hz` 之下。
     * @since 0.1.0
     */
    struct Config final {
        float inject_v;  ///< [en] injection amplitude, peak V, > 0. [zh] 注入幅值，峰值 V，> 0。
        float inject_hz; ///< [en] injection frequency in Hz, > demod_lpf_hz. [zh] 注入频率，Hz，> demod_lpf_hz。
        float
            current_hp_hz; ///< [en] high-pass on the measured current in Hz, see the note below. [zh] 实测电流的高通截止频率，Hz，见下方注释。
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
     * @note [en] Also false for the first few milliseconds after construction or `reset()`: the
     *       demodulation filters need about ten time constants before their output means anything,
     *       and integrating before that turns the carrier switching on into a measurement. How long
     *       is derived from `demod_lpf_hz` rather than exposed, because it is a property of the
     *       filter, not a machine fact.
     *       [zh] 构造或 `reset()` 之后的最初几毫秒内同样为 false：解调滤波器需要约十个时间常数，
     *       其输出才有意义；在此之前积分，等于把"载波刚加上"当成一次测量。时长由 `demod_lpf_hz`
     *       导出而不单独暴露，因为它是滤波器的属性，不是机器的事实。
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
     * @brief [en] Correct the half-turn ambiguity with the answer from `MagnetPolarityTest`.
     *        [zh] 用 `MagnetPolarityTest` 的结论修正半圈歧义。
     *
     * @param reversed [en] true when the pulses showed the estimate is half a turn out; the estimate
     *                 then rotates by pi and the same object is correct from here on.
     *                 [zh] 脉冲显示估计差半圈时传 true；随后估计旋转 π，同一对象从此正确。
     * @note [en] A separate call rather than an argument to update(): the correction is a one-off
     *       commissioning fact, and threading it through the per-step contract would mean every step
     *       carries a decision that was already made once.
     *       [zh] 单独一次调用，而不是 update() 的参数：这个修正是调试期一次性的事实，把它穿进每步契约
     *       等于让每一步都携带一个早已做过的决定。
     * @since 0.1.0
     */
    void apply_polarity(bool reversed) noexcept;

    /**
     * @brief [en] Hand the loop a starting estimate, which must be inside `+-pi/2` of the rotor.
     *        [zh] 给环路一个起始估计，必须落在转子 ±π/2 之内。
     *
     * @param theta_e [en] electrical angle in rad. [zh] 电角度，rad。
     * @param omega_e [en] electrical speed in rad/s: the loop's integrator is preloaded with it, so it
     *                resumes tracking instead of having to catch up from zero. A loop that has to
     *                acquire a running rotor spends the catch-up as angle error, which is exactly the
     *                transient a handover must not produce.
     *                [zh] 电角速度，rad/s：环路积分器预置为它，于是环路是"继续跟踪"而不是"从零追赶"。
     *                让一个环路去追赶正在转的转子，追赶过程就是角度误差——而这正是交接绝不能产生的暂态。
     * @note [en] The loop only pulls inside that window (see the warning above), so an estimate that
     *       wandered while the carrier was off cannot be resumed -- it has to be told where the rotor
     *       is now. A blending observer calls this while the carrier is wound down, which is what makes
     *       the carrier able to come back at any time instead of only from a fresh start.
     *       [zh] 环路只在该窗口内牵引（见上方警告），因此"载波关掉期间漂走的估计"无法直接续用——必须
     *       告诉它转子现在在哪。混合观测器在载波收掉期间调用它，于是载波可以在任何时刻回来，而不是只能
     *       从全新状态开始。
     * @since 0.1.0
     */
    void seed_angle(float theta_e, float omega_e) noexcept;

    /**
     * @brief [en] Take an updated machine description, recomputing the modulation sensitivity from it.
     *        [zh] 接受更新后的机器描述，并据此重算解调灵敏度。
     *
     * @param params [en] the machine as it actually is (identified, or re-read after heating). [zh]机器实际的样子（辨识
     * 得到，或升温后重读）。
     * @note [en] Only `demod_gain_a` depends on the machine -- the loop shape and the injection settings do
     *       not -- so this is one recomputation, not a retune. Nothing is reset: the estimate carries on
     *       from where it was, which is what makes it usable mid-run.
     *       [zh] 只有 `demod_gain_a` 依赖机器——环路形状与注入设置不依赖——因此这是一次重算，不是重新整定。
     *       不重置任何状态：估计从原处继续，这正是它能在运行中使用的原因。
     * @since 0.1.0
     */
    void apply_params(const MachineParams& params) noexcept;

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
    FirstOrderLowPass current_hp_alpha_;
    FirstOrderLowPass current_hp_beta_;
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
    float
        blank_s_; ///< [en] start-up blanking time, derived from the demodulation filter. [zh] 起动屏蔽时长，由解调滤波器导出。
    float blank_left_s_; ///< [en] blanking still to run. [zh] 尚未走完的屏蔽时间。
    bool primed_;
    float no_signal_s_; ///< [en] how long the carrier response has been absent, s. [zh] 载波响应已缺失多久，s。
    bool locked_;
};


} // namespace core

} // namespace foc
