// Conan::ImportStart
#pragma once
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Per-call gain scaling: how hard the drive resists, for this call only.
 *
 *   A zero or negative scale means "no override" and becomes \f$1\f$, which is the same rule the rest of the project
 *   uses for per-call values; here it also keeps a defaulted aggregate from asking for zero stiffness. The scale
 *   multiplies the **configuration** and is never accumulated into it, so one compliant call cannot leave the
 *   controller detuned for the next stiff one.
 *
 * @brief [zh] 按次增益缩放：这一次驱动要抵抗多少。
 *
 *   零或负的缩放表示"不覆盖"，会被取成 \f$1\f$，与项目其余部分对按次传值的规则一致；在这里它还防止一个被值初始化的聚合
 *   体要求零刚度。缩放乘的是 **配置** 、绝不累加进配置，所以一次柔顺调用不会把控制器留在失调状态。
 * @see `test_pi.cpp` (`AZeroIntegralScaleMeansNoOverrideRatherThanAnEmptyWindow`)
 * @since 0.1.0
 * @exporter
 */
struct SpeedInput final {
    float target_rad_s; ///< [en] speed setpoint, rad/s. [zh] 速度给定，rad/s。
    float kp_scale;     ///< [en] proportional scale, 0 = configured. [zh] 比例缩放，0 = 用配置值。
    float ilimit_scale; ///< [en] integral-limit scale, 0 = configured. [zh] 积分限幅缩放，0 = 用配置值。
};


/**
 * @brief [en] One discrete PI step with the output and the integral both clamped.
 *
 *   The integral clamp **is** the anti-windup mechanism: there is no back-calculation and `step()` never sees the
 *   actuator's actual output. The integral is re-clamped against the **current** window before it is added to the
 *   proportional term, which is what makes a shrunken window release stored output at once:
 *
 *   \f[ e = r - y,\qquad i \leftarrow \mathrm{clamp}(i + k_i\,e\,\Delta t,\; i_{min},\; i_{max}),\qquad u =
 *   \mathrm{clamp}(k_p\,e + i,\; u_{min},\; u_{max}) \f]
 *
 *   Where \f$e\f$ is the control error, \f$u\f$ the output, \f$k_p\f$ the proportional gain (output units per error
 *   unit), \f$k_i\f$ the integral gain (output units per error unit per second) and \f$\Delta t\f$ the step length in
 *   seconds. Everything is in the caller's loop units; no machine quantity appears here, so nothing in this block needs
 *   the symbol table. Ledger `E-PI-1`.
 *
 *   Size the integral window to the loop's steady-state output range -- for a speed loop, the current that holds the
 *   top speed rather than `i_max` -- because the full output range costs a slow, visible recovery after saturation
 *   (measured in the current-step and saturation-limit scenarios). All state lives in the object: no globals, no
 *   allocation, one instance per axis.
 *
 * @brief [zh] 输出与积分都被限幅的离散 PI 一步。
 *
 *   积分限幅 **就是** 抗饱和手段：本实现没有 back-calculation，`step()` 也拿不到执行器实际施加的输出。积分在加到比例项
 *   **之前** 先按 **当前** 窗口重新夹紧，这正是窗口收窄时已存输出能立刻释放的原因：
 *
 *   \f[ e = r - y,\qquad i \leftarrow \mathrm{clamp}(i + k_i\,e\,\Delta t,\; i_{min},\; i_{max}),\qquad u =
 *   \mathrm{clamp}(k_p\,e + i,\; u_{min},\; u_{max}) \f]
 *
 *   其中 \f$e\f$ 是控制误差、\f$u\f$ 是输出、\f$k_p\f$ 是比例增益（输出单位/误差单位）、\f$k_i\f$ 是积分增益（输出单位/
 *   误差单位/秒）、\f$\Delta t\f$ 是步长，s。全部取调用方环路的单位；本段不出现机器量，因此不需要符号表。台账
 *   `E-PI-1`。
 *
 *   积分窗口请取成该环的 **稳态输出量级** ——速度环即“维持最高转速所需电流”，而不是 `i_max`——因为按满量程取会让饱和后出
 *   现明显的慢恢复（电流阶跃与饱和限幅两个场景有实测数字）。状态全部在对象内：无全局、不分配，每个轴一个实例。
 *
 * @warning [en] A negative gain is refused at construction and the controller latches `ErrorCode::Param`; a
 *               non-positive `dt` is refused at the step and the output holds at its previous value. A latched `Param`
 *               keeps returning that held value on every later step: it never resumes on its own.
 * @warning [zh] 负增益在构造时被拒绝、控制器锁存 `ErrorCode::Param`；非正 `dt` 在这一步被拒绝，输出停在上一拍的值。一旦
 *               锁存 `Param`，之后每一步都返回那个保持值：它不会自己恢复。
 * @see `foc/core/pi.hpp` claim row, `test_pi.cpp`, ledger `E-PI-1`
 * @context [en] Current-loop / speed-loop ISR: noexcept, no allocation.
 * @context [zh] 电流环/速度环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class Pi final {
  public:
    /**
     * @brief [en] Gains and limits, validated once at construction rather than on every step.
     *
     *   Valid when \f$k_p \ge 0\f$, \f$k_i \ge 0\f$, \f$u_{min} \le u_{max}\f$ and \f$i_{min} \le i_{max}\f$; anything
     *   else leaves the controller latched in `ErrorCode::Param`. The point of checking here is that a configuration
     *   which passes cannot make `clamp` receive an inverted window, which is the only way a clamp silently stops being
     *   a clamp. \f$k_p\f$ and \f$k_i\f$ are in output units per error unit (per second for \f$k_i\f$); the four limits
     *   are in output units and in the integral's own units.
     *
     * @brief [zh] 增益与限幅，在构造时校验一次，而不是每一步都校验。
     *
     *   合法条件是 \f$k_p \ge 0\f$、\f$k_i \ge 0\f$、\f$u_{min} \le u_{max}\f$、\f$i_{min} \le i_{max}\f$；其余情形让控
     *   制器锁存在 `ErrorCode::Param`。在这里校验的意义是：通过的配置不可能让 `clamp` 收到一个反向的窗口——而那是限幅悄
     *   悄不再是限幅的唯一途径。\f$k_p\f$ 与 \f$k_i\f$ 的单位是输出单位/误差单位（\f$k_i\f$ 再除一次秒）；四个限幅分别
     *   取输出单位与积分自身单位。
     *
     * @warning [en] A negative gain or an inverted window is refused at construction, and `reset()` re-runs the same
     *               check, so a latched controller cannot be revived by clearing its state alone.
     * @warning [zh] 负增益或反向窗口在构造时被拒绝；`reset()` 会重跑同一项检查，所以无法靠清状态把锁存的控制器救活。
     * @see `test_pi.cpp` (`NegativeGainIsRejectedAtConstruction`)
     * @since 0.1.0
     */
    struct Config final {
        float kp;           ///< [en] proportional gain, must be >= 0. [zh] 比例增益，必须 >= 0。
        float ki;           ///< [en] integral gain, must be >= 0. [zh] 积分增益，必须 >= 0。
        float out_min;      ///< [en] lower output clamp. [zh] 输出下限。
        float out_max;      ///< [en] upper output clamp. [zh] 输出上限。
        float integral_min; ///< [en] lower integral clamp. [zh] 积分下限。
        float integral_max; ///< [en] upper integral clamp. [zh] 积分上限。
    };

    /**
     * @brief [en] Construct from a validated configuration.
     * @brief [zh] 用配置构造。
     * @param config [en] gains and limits; invalid values put the controller in Param status.
     * @param config [zh] 增益与限幅；非法值会让控制器进入 Param 状态。
     * @since 0.1.0
     */
    explicit Pi(Config config) noexcept;

    /**
     * @brief [en] Advance one control step.
     * @brief [zh] 推进一步控制。
     * @param setpoint [en] reference.
     * @param setpoint [zh] 给定。
     * @param feedback [en] measured value.
     * @param feedback [zh] 反馈值。
     * @param dt_s [en] step length in s; <= 0 is rejected (status becomes Param, output holds).
     * @param dt_s [zh] 步长，单位 s；<= 0 被拒绝（状态置 Param，输出保持不变）。
     * @return [en] clamped controller output.
     * @return [zh] 限幅后的控制输出。
     * @context [en] Current-loop / speed-loop ISR.
     * @context [zh] 电流环/速度环 ISR。
     * @since 0.1.0
     */
    auto step(float setpoint, float feedback, float dt_s) noexcept -> float;

    /**
     * @brief [en] Advance one control step with per-call gain scaling.
     * @brief [zh] 带按次增益缩放地推进一步控制。
     * @param request [en] setpoint plus the two scales (0 = use the configuration).
     * @param request [zh] 给定以及两个缩放（0 = 用配置值）。
     * @param feedback [en] measured value.
     * @param feedback [zh] 反馈值。
     * @param dt_s [en] step length in s; <= 0 is rejected (output holds).
     * @param dt_s [zh] 步长，s；<= 0 被拒绝（输出保持）。
     * @return [en] clamped controller output.
     * @return [zh] 限幅后的控制输出。
     * @note [en] The configuration is never mutated: the scales are applied to the gains for this call only, so the
     *            next stiff call gets the configured gains back with no restore step. The integral term is re-clamped
     *            into the scaled window, which is what makes `ilimit_scale` release stored torque instead of holding it
     *            until it leaks away.
     * @note [zh] 配置从不被改动：缩放只在本次调用生效，下一次刚性调用直接拿回配置的增益，不需要恢复 步骤。积分项会被重
     *            新夹进缩放后的窗口——这正是 `ilimit_scale` 能 **释放** 已存转矩、而不是 等它慢慢漏掉的机制。
     * @since 0.1.0
     */
    auto step(const SpeedInput& request, float feedback, float dt_s) noexcept -> float;

    /**
     * @brief [en] Clear the integral term and any latched parameter error.
     * @brief [zh] 清零积分项与已锁存的参数错误。
     * @since 0.1.0
     */
    void reset() noexcept;

    /**
     * @brief [en] Current integral term.
     * @brief [zh] 当前积分项。
     * @since 0.1.0
     */
    [[nodiscard]] auto integral() const noexcept -> float;

    /**
     * @brief [en] Last computed output.
     * @brief [zh] 最近一次计算出的输出。
     * @since 0.1.0
     */
    [[nodiscard]] auto output() const noexcept -> float;

    /**
     * @brief [en] Ok, or Param after an invalid configuration or a non-positive dt.
     * @brief [zh] Ok，或非法配置 / 非正 dt 后的 Param。
     * @since 0.1.0
     */
    [[nodiscard]] auto status() const noexcept -> ErrorCode;

  private:
    [[nodiscard]] auto run(float setpoint, float feedback, float dt_s, float kp, float integral_min,
                           float integral_max) noexcept -> float;

    Config config_;
    float integral_;
    float output_;
    ErrorCode status_;
};

} // namespace core

} // namespace foc
