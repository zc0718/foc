// Conan::ImportStart
#pragma once
#include <cstdint>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] What a polarity pulse pair concluded.
 *        [zh] 一对极性脉冲得出的结论。
 *
 * @note [en] `Inconclusive` is the initial value and the safe default at once: "no answer yet" and
 *       "this machine cannot answer" are both "not Aligned/Reversed", so a caller that only tests
 *       for `Reversed` cannot be fooled by a run that has not finished.
 *       [zh] `Inconclusive` 同时是初值和安全默认值：“还没有答案”与“这台机器答不了”都是“既非
 *       Aligned 也非 Reversed”，于是只判断 `Reversed` 的调用方不会被一次还没跑完的测试骗到。
 * @since 0.1.0
 * @exporter
 */
enum class PolarityVerdict : std::int8_t {
    Inconclusive, ///< [en] no pulse pair has finished, or they showed no asymmetry. [zh] 还没跑完一对脉冲，或二者没有不对称。
    Aligned, ///< [en] the estimate's +d already is the magnet's +d. [zh] 估计的 +d 就是磁钢的 +d。
    Reversed, ///< [en] the estimate is half a turn out: add pi before closing the loop. [zh] 估计差半个电周期：闭环前加 π。
};


/**
 * @brief [en] Magnet polarity from a pair of d-axis voltage pulses.
 *        [zh] 用一对 d 轴电压脉冲判断磁钢极性。
 *
 *        Why a separate step rather than something the observer does: a saliency tracking loop has a
 *        stable zero at `dtheta = 0` **and** at `dtheta = pi`, because `sin(2*dtheta)` cannot tell the
 *        two apart. The information that separates them is not in the saliency -- it is in magnetic
 *        saturation: `+i_d` reinforces the rotor flux and `-i_d` opposes it, so the *same* volt-second
 *        pulse in the two directions produces two different current rises. The larger rise is the
 *        magnet's `+d`.
 *        为什么是一个单独步骤而不是观测器顺手做的事：凸极跟踪环在 `dtheta = 0` 与 `dtheta = pi` 都有
 *        稳定零点，因为 `sin(2·dtheta)` 区分不了两者。区分它们的信息不在凸极里，而在磁饱和里：
 *        `+i_d` 增强转子磁链、`-i_d` 削弱它，于是**相同**伏秒的脉冲在两个方向上产生不同的电流上升。
 *        上升更大的那个方向就是磁钢的 `+d`。
 *
 *        Why pulses along d and not q: a pulse on the d axis of a converged estimate produces no
 *        torque (to first order), so the rotor stays where the test found it. A q-axis pulse would
 *        move it and the second pulse would be measuring a different machine position.
 *        为什么沿 d 轴而不是 q 轴脉冲：在已收敛估计的 d 轴上脉冲，(一阶意义上)不产生转矩，转子留在
 *        测试发现它的位置。q 轴脉冲会把它推走，第二个脉冲测的就是另一个转子位置了。
 *
 * @warning [en] This injects a current pulse on purpose, so the caller must run it at standstill with
 *          the inverter under its own control -- not through a closed current loop that would fight
 *          the pulse. The documented order is: HFI converges, this test runs, `apply_polarity()` is
 *          handed the answer, and only then is the loop closed.
 *          [zh] 它故意注入电流脉冲，因此调用方必须在静止、且逆变器由自己直接控制的条件下运行它——不能
 *          经过一个会与脉冲对抗的闭环电流环。文档给的顺序是：HFI 收敛 → 跑本测试 →
 *          `apply_polarity()` 应用结论 → 之后才闭环。
 *
 * @context [en] Commissioning only: noexcept, no allocation. [zh] 仅调试/上电流程：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class MagnetPolarityTest final {
  public:
    /**
     * @brief [en] Pulse shape and decision threshold.
     *        [zh] 脉冲形状与判定阈值。
     * @since 0.1.0
     */
    struct Config final {
        float pulse_v;  ///< [en] d-axis pulse magnitude in V, > 0. [zh] d 轴脉冲幅值，V，> 0。
        float pulse_s;  ///< [en] duration of each pulse in s, > 0. [zh] 每个脉冲的时长，s，> 0。
        float settle_s; ///< [en] relaxation before each pulse in s, >= 0. [zh] 每个脉冲前的松弛时间，s，>= 0。
        float
            min_asymmetry_a; ///< [en] rise difference below which the answer is Inconclusive, >= 0. [zh] 上升差低于此值即判为 Inconclusive，>= 0。
    };

    /**
     * @brief [en] One step's worth of output.
     *        [zh] 单步输出。
     * @since 0.1.0
     */
    struct Output final {
        PolarityVerdict
            verdict;    ///< [en] Inconclusive until a pair has finished. [zh] 一对脉冲跑完前恒为 Inconclusive。
        Ab<float> v_ab; ///< [en] voltage the caller must apply this step. [zh] 本步调用方必须施加的电压。
        bool running;   ///< [en] false once the verdict is final. [zh] 结论确定后为 false。
        float
            delta_positive_a; ///< [en] signed current change during the +d pulse, A. [zh] +d 脉冲期间带符号的电流变化，A。
        float
            delta_negative_a; ///< [en] signed current change during the -d pulse, A. [zh] −d 脉冲期间带符号的电流变化，A。
    };

    /**
     * @brief [en] Construct and validate.
     *        [zh] 构造并校验。
     * @param config [en] invalid values put the test in Param status; step() then applies no voltage.
     *               [zh] 非法值让测试进入 Param 状态；此后 step() 不施加任何电压。
     * @since 0.1.0
     */
    explicit MagnetPolarityTest(Config config) noexcept;

    /**
     * @brief [en] Start the sequence along a given d-axis estimate.
     *        [zh] 沿给定的 d 轴估计开始序列。
     *
     * @param theta_e [en] the d-axis angle from the saliency loop, rad -- known to within +-pi/2,
     *                which is exactly the ambiguity this test resolves.
     *                [zh] 凸极环给出的 d 轴角度，rad——已知在 ±π/2 以内，而这正是本测试要解掉的歧义。
     * @since 0.1.0
     */
    void begin(float theta_e) noexcept;

    /**
     * @brief [en] Advance one step.
     *        [zh] 推进一步。
     *
     * @param dt_s [en] step length in s; <= 0 is rejected (status Param). [zh] 步长，s；<= 0 被拒绝（状态 Param）。
     * @param i_ab [en] measured stator current in A, in the stationary frame. [zh] 实测定子电流，A，静止系。
     * @return [en] the voltage to apply and, once finished, the verdict. [zh] 要施加的电压，以及跑完后的结论。
     * @since 0.1.0
     */
    auto step(float dt_s, Ab<float> i_ab) noexcept -> Output;

    /**
     * @brief [en] Ok, or Param after an invalid configuration or non-positive dt.
     *        [zh] Ok，或非法配置 / 非正 dt 后的 Param。
     * @since 0.1.0
     */
    [[nodiscard]] auto status() const noexcept -> ErrorCode;

    /**
     * @brief [en] The verdict as last decided. [zh] 最近一次判定的结论。
     * @since 0.1.0
     */
    [[nodiscard]] auto verdict() const noexcept -> PolarityVerdict;

  private:
    /// [en] Where the sequence is. [zh] 序列所处阶段。
    enum class Stage : std::int8_t { SettlePositive, PulsePositive, SettleNegative, PulseNegative, Done };

    auto decide() noexcept -> void;
    [[nodiscard]] auto output() const noexcept -> Output;

    Config config_;
    ErrorCode status_;
    Stage stage_;
    PolarityVerdict verdict_;
    float theta_e_;    ///< [en] d-axis angle the pulses are applied along. [zh] 脉冲施加的 d 轴角度。
    float stage_s_;    ///< [en] time spent in the current stage, s. [zh] 当前阶段已用时间，s。
    float i_before_a_; ///< [en] current along d at the end of the preceding settle. [zh] 前一段松弛结束时的 d 轴电流。
    float delta_positive_a_;
    float delta_negative_a_;
};


} // namespace core

} // namespace foc
