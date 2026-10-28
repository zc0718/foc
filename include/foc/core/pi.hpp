// Conan::ImportStart
#pragma once
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] A PI step with per-call gain scaling: the knob teleoperation needs.
 *
 *     Pushing a joint by hand requires the drive to stop resisting, and the two knobs that decide how much it resists
 *     are the proportional gain (`kp_scale`) and how much torque the integrator is allowed to keep holding
 *     (`ilimit_scale`). Both are *per call*: a compliant approach and a stiff hold are the same controller, not two
 *     configurations.
 *
 * @brief [zh] 带按次增益缩放的 PI 一步：遥操作需要的就是这个旋钮。
 *
 *     用手推关节要求驱动不再抵抗，而决定"抵抗多少"的两个旋钮是比例增益（`kp_scale`）与 积分器被允许持续保持多少转矩（
 *     `ilimit_scale`）。两者都是**按次**的：柔顺接近与刚性保持 是同一个控制器，而不是两套配置。
 *
 * @note [en] `0` means "no override": the configured gain is used unchanged. That is the same rule the rest of the
 *            project uses for per-call values, and here it also protects a defaulted aggregate initialiser from
 *            silently asking for zero stiffness.
 * @note [zh] `0` 表示"不覆盖"：按配置的增益原样使用。这与项目其余部分对按次传值的规则一致；在这里 它还防止一个被值初始
 *            化的聚合体悄悄要求零刚度。
 * @since 0.1.0
 * @exporter
 */
struct SpeedInput final {
    float target_rad_s; ///< [en] speed setpoint, rad/s. [zh] 速度给定，rad/s。
    float kp_scale;     ///< [en] proportional scale, 0 = configured. [zh] 比例缩放，0 = 用配置值。
    float ilimit_scale; ///< [en] integral-limit scale, 0 = configured. [zh] 积分限幅缩放，0 = 用配置值。
};


/**
 * @brief [en] PI controller with output clamping and integral clamping (V1 anti-windup). All state lives in the object:
 *             no globals, no allocation, safe to use per axis.
 * @brief [zh] 带输出限幅与积分限幅（V1 抗饱和）的 PI 控制器。状态全部在对象内：无全局、不分配， 每个轴各用一个实例即
 *             可。
 * @note [en] The integral clamp IS the anti-windup mechanism: there is no back-calculation, and `step()` never sees the
 *            actuator's actual output. Size `integral_min`/`integral_max` to the steady-state output range of the loop
 *            (for a speed loop: the current that holds the top speed, not `i_max`), so a reference step that saturates
 *            the output cannot wind the integral up. Sizing them to the full output range costs a slow, visible
 *            recovery after saturation (measured in the current-step and saturation-limit scenarios).
 * @note [zh] 积分限幅**就是**抗饱和手段：本实现没有 back-calculation，`step()` 也拿不到执行器 实际施加的输出。请把
 *            `integral_min`/`integral_max` 取成该环的**稳态输出量级**（速度环 即"维持最高转速所需电流"，而不是
 *            `i_max`），这样给定阶跃打满输出时积分不会积起来； 若按满量程取，饱和后会出现明显的慢恢复（电流阶跃与饱和限
 *            幅两个场景有实测数字）。
 * @context [en] Current-loop / speed-loop ISR: noexcept, no allocation.
 * @context [zh] 电流环/速度环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class Pi final {
  public:
    /**
     * @brief [en] Gains and limits. See the constraints on each field.
     * @brief [zh] 增益与限幅。各字段的约束见注释。
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
