// Conan::ImportStart
#pragma once
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] The per-call gain scaling a teleoperation client sends: how hard to resist, for this call only.
 *
 *     **Equation.** `kp_eff = kp * s_kp`, `i-limits_eff = i-limits * s_i`, where `s = (scale > 0) ? scale : 1`.
 *
 *     **Notation.** `target_rad_s` the setpoint, `kp_scale` and `ilimit_scale` dimensionless multipliers.
 *
 *     **Guarantees.** A scale is applied to the **configuration** and never accumulated into it, so one compliant call
 *     cannot leave the controller detuned for the next stiff one.
 *
 *     **Refusals.** Nothing is refused here: a zero or negative scale means "no override" and becomes `1`, which is the
 *     same rule the rest of the project uses for per-call values.
 *
 *     **Evidence.** The `foc/core/pi.hpp` claim row, `test_pi.cpp`
 *     (`AZeroIntegralScaleMeansNoOverrideRatherThanAnEmptyWindow`).
 *
 * @brief [zh] 遥操作客户端发来的按次增益缩放：这一次要抵抗多少。
 *
 *     **Equation.** `kp_eff = kp · s_kp`，`积分限幅_eff = 积分限幅 · s_i`，其中 `s = (scale > 0) ? scale : 1`。
 *
 *     **Notation.** `target_rad_s` 给定；`kp_scale` 与 `ilimit_scale` 是无量纲乘数。
 *
 *     **Guarantees.** 缩放作用于**配置**、绝不累加进配置，因此一次柔顺调用不会把控制器留在失调状态等着下一次刚性调用。
 *
 *     **Refusals.** 这里不拒绝任何东西：零或负的缩放表示"不覆盖"，会被取成 `1`——这与项目其余部分对按次传值的规则一致。
 *
 *     **Evidence.** `foc/core/pi.hpp` 的 claim 行、`test_pi.cpp`（
 *     `AZeroIntegralScaleMeansNoOverrideRatherThanAnEmptyWindow`）。
 * @since 0.1.0
 * @exporter
 */
struct SpeedInput final {
    float target_rad_s; ///< [en] speed setpoint, rad/s. [zh] 速度给定，rad/s。
    float kp_scale;     ///< [en] proportional scale, 0 = configured. [zh] 比例缩放，0 = 用配置值。
    float ilimit_scale; ///< [en] integral-limit scale, 0 = configured. [zh] 积分限幅缩放，0 = 用配置值。
};


/**
 * @brief [en] One discrete PI step whose output and integral are both clamped, so a saturating command cannot wind the
 *             integrator up. All state lives in the object: no globals, no allocation, one instance per axis.
 *
 *     **Equation.** `e = r - y`; `i <- clamp(i + ki*e*dt, integral_min, integral_max)`; `u = clamp(kp*e + i, out_min,
 *     out_max)`; ledger `E-PI-1`. The integral is re-clamped against the **current** window before it is added to the
 *     proportional term, which is what makes a shrunken window release stored output at once.
 *
 *     **Notation.** `e` control error, `u` output, `kp` proportional gain (output units per error unit), `ki` integral
 *     gain (output units per error unit per second), `dt` the step length in s. Everything is in the caller's loop
 *     units; no machine quantity appears in this block, so nothing here needs the symbol table.
 *
 *     **Guarantees.** The output stays inside `[out_min, out_max]` and the integral inside `[integral_min,
 *     integral_max]`; both clamps are reached and held, never overshot. The integral clamp **is** the anti-windup
 *     mechanism: there is no back-calculation and `step()` never sees the actuator's actual output. Size the integral
 *     window to the loop's steady-state output range -- for a speed loop, the current that holds the top speed rather
 *     than `i_max` -- because the full output range costs a slow, visible recovery after saturation (measured in the
 *     current-step and saturation-limit scenarios).
 *
 *     **Refusals.** A negative gain is refused at construction and the controller latches `ErrorCode::Param`; a
 *     non-positive `dt` is refused at the step and the output holds at its previous value. A latched `Param` keeps
 *     returning the held output on every later step: it never resumes on its own.
 *
 *     **Evidence.** The `foc/core/pi.hpp` claim row, `test_pi.cpp`, ledger `E-PI-1`.
 *
 * @brief [zh] 输出与积分**都**被限幅的离散 PI 一步，于是打满输出的指令积不起积分。状态全部在对象内：无全局、不分配，每
 *             个轴各用一个实例。
 *
 *     **Equation.** `e = r - y`；`i ← clamp(i + ki·e·dt, integral_min, integral_max)`；`u ← clamp(kp·e + i, out_min,
 *     out_max)`；台账 `E-PI-1`。积分在加到比例项**之前**先按**当前**窗口重新夹紧——这正是窗口收窄时已存输出能立刻释放的
 *     原因。
 *
 *     **Notation.** `e` 控制误差、`u` 输出、`kp` 比例增益（输出单位/误差单位）、`ki` 积分增益（输出单位/误差单位/秒）、
 *     `dt` 步长，s。全部取调用方环路的单位；本段不出现机器量，因此不需要符号表。
 *
 *     **Guarantees.** 输出不越 `[out_min, out_max]`、积分不越 `[integral_min, integral_max]`；两个限幅都是到达并保持，
 *     不会越过。积分限幅**就是**抗饱和手段：本实现没有 back-calculation，`step()` 也拿不到执行器实际施加的输出。请把积
 *     分窗口取成该环的**稳态输出量级**——速度环即"维持最高转速所需电流"，而不是 `i_max`——因为按满量程取会让饱和后出现明
 *     显的慢恢复（电流阶跃与饱和限幅两个场景有实测数字）。
 *
 *     **Refusals.** 负增益在构造时被拒绝、控制器锁存 `ErrorCode::Param`；非正 `dt` 在这一步被拒绝，输出停在上一拍的值。
 *     一旦锁存 `Param`，之后每一步都返回那个保持值：它不会自己恢复。
 *
 *     **Evidence.** `foc/core/pi.hpp` 的 claim 行、`test_pi.cpp`、台账 `E-PI-1`。
 *
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
     *     **Equation.** Valid when `kp >= 0`, `ki >= 0`, `out_min <= out_max` and `integral_min <= integral_max`;
     *     anything else leaves the controller latched in `ErrorCode::Param`.
     *
     *     **Notation.** `kp`, `ki` gains in output-units-per-error-unit (per second for `ki`); the four limits are in
     *     output units and in the integral's own units.
     *
     *     **Guarantees.** A configuration that passes this check cannot make `clamp` receive an inverted window, which
     *     is the only way a clamp silently stops being a clamp.
     *
     *     **Refusals.** A negative gain or an inverted window is refused at construction, and `reset()` re-runs the
     *     same check, so a latched controller cannot be revived by clearing its state alone.
     *
     *     **Evidence.** The `foc/core/pi.hpp` claim row, `test_pi.cpp` (`NegativeGainIsRejectedAtConstruction`).
     *
     * @brief [zh] 增益与限幅，在构造时校验一次，而不是每一步都校验。
     *
     *     **Equation.** 合法条件：`kp >= 0`、`ki >= 0`、`out_min <= out_max`、`integral_min <= integral_max`；其余情形
     *     让控制器锁存在 `ErrorCode::Param`。
     *
     *     **Notation.** `kp`、`ki` 的单位是"输出单位/误差单位"（`ki` 再除一次秒）；四个限幅分别取输出单位与积分自身单
     *     位。
     *
     *     **Guarantees.** 通过这项检查的配置不可能让 `clamp` 收到一个反向的窗口——而那是限幅悄悄不再是限幅的唯一途径。
     *
     *     **Refusals.** 负增益或反向窗口在构造时被拒绝；`reset()` 会重跑同一项检查，所以无法靠清状态把锁存的控制器救
     *     活。
     *
     *     **Evidence.** `foc/core/pi.hpp` 的 claim 行、`test_pi.cpp`（`NegativeGainIsRejectedAtConstruction`）。
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
     *            新夹进缩放后的窗口——这正是 `ilimit_scale` 能**释放**已存转矩、而不是 等它慢慢漏掉的机制。
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
