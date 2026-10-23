// Conan::ImportStart
#pragma once

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] PI controller with output clamping and integral clamping (V1 anti-windup).
 *        All state lives in the object: no globals, no allocation, safe to use per axis.
 *        [zh] 带输出限幅与积分限幅（V1 抗饱和）的 PI 控制器。状态全部在对象内：无全局、不分配，
 *        每个轴各用一个实例即可。
 *
 * @context [en] Current-loop / speed-loop ISR: noexcept, no allocation.
 *           [zh] 电流环/速度环 ISR：noexcept、不分配。
 * @since 0.1.0
 */
class Pi final {
  public:
    /**
     * @brief [en] Gains and limits. See the constraints on each field.
     *        [zh] 增益与限幅。各字段的约束见注释。
     *
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
     *        [zh] 用配置构造。
     *
     * @param config [en] gains and limits; invalid values put the controller in Param status.
     *               [zh] 增益与限幅；非法值会让控制器进入 Param 状态。
     * @since 0.1.0
     */
    explicit Pi(Config config) noexcept;

    /**
     * @brief [en] Advance one control step.
     *        [zh] 推进一步控制。
     *
     * @param setpoint [en] reference. [zh] 给定。
     * @param feedback [en] measured value. [zh] 反馈值。
     * @param dt_s [en] step length in s; <= 0 is rejected (status becomes Param, output holds).
     *              [zh] 步长，单位 s；<= 0 被拒绝（状态置 Param，输出保持不变）。
     * @return [en] clamped controller output. [zh] 限幅后的控制输出。
     * @context [en] Current-loop / speed-loop ISR. [zh] 电流环/速度环 ISR。
     * @since 0.1.0
     */
    auto step(float setpoint, float feedback, float dt_s) noexcept -> float;

    /**
     * @brief [en] Clear the integral term and any latched parameter error.
     *        [zh] 清零积分项与已锁存的参数错误。
     * @since 0.1.0
     */
    void reset() noexcept;

    /**
     * @brief [en] Current integral term.
     *        [zh] 当前积分项。
     * @since 0.1.0
     */
    [[nodiscard]] auto integral() const noexcept -> float;

    /**
     * @brief [en] Last computed output.
     *        [zh] 最近一次计算出的输出。
     * @since 0.1.0
     */
    [[nodiscard]] auto output() const noexcept -> float;

    /**
     * @brief [en] Ok, or Param after an invalid configuration or a non-positive dt.
     *        [zh] Ok，或非法配置 / 非正 dt 后的 Param。
     * @since 0.1.0
     */
    [[nodiscard]] auto status() const noexcept -> ErrorCode;

  private:
    Config config_;
    float integral_;
    float output_;
    ErrorCode status_;
};

} // namespace core

} // namespace foc
