// Conan::ImportStart
#pragma once
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Symmetric clamp. Returns `value` confined to [low, high].
 *        [zh] 对称/区间限幅：把 `value` 限制在 [low, high] 内返回。
 *
 * @pre [en] low <= high. [zh] low <= high。
 * @context [en] Current-loop ISR: constexpr, noexcept, no allocation. [zh] 电流环 ISR：constexpr、noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
template <typename T> constexpr auto clamp(T value, T low, T high) noexcept -> T {
    return (value < low) ? low : ((value > high) ? high : value);
}


/**
 * @brief [en] Slew-rate limiter: moves the output towards the target no faster than a
 *        configured rate per second (symmetric limit).
 *        [zh] 斜率限制器：输出以不超过配置速率的步进逼近目标（对称限制）。
 *
 * @context [en] Current-loop ISR: noexcept, no allocation. [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class SlewLimiter final {
  public:
    /**
     * @brief [en] Construct a limiter with a maximum slew rate.
     *        [zh] 用最大斜率构造限制器。
     *
     * @param rate_per_s [en] maximum change per second, must be > 0.
     *                   [zh] 每秒最大变化量，必须 > 0。
     * @since 0.1.0
     */
    explicit SlewLimiter(float rate_per_s) noexcept;

    /**
     * @brief [en] Advance one step towards the target.
     *        [zh] 向目标推进一步。
     *
     * @param target [en] desired value. [zh] 目标值。
     * @param dt_s [en] step length in s; non-positive means "no time passed".
     *              [zh] 步长，单位 s；非正值表示未经过时间。
     * @return [en] the limited value, which also becomes the new state.
     *         [zh] 限幅后的值，同时作为新的内部状态。
     * @context [en] Current-loop ISR. [zh] 电流环 ISR。
     * @since 0.1.0
     */
    auto step(float target, float dt_s) noexcept -> float;

    /**
     * @brief [en] Force the internal state (e.g. on mode change).
     *        [zh] 强制设置内部状态（例如模式切换时）。
     *
     * @param value [en] new state. [zh] 新状态。
     * @since 0.1.0
     */
    void reset(float value) noexcept;

    /**
     * @brief [en] Current limited output.
     *        [zh] 当前限幅输出。
     * @since 0.1.0
     */
    [[nodiscard]] auto value() const noexcept -> float;

  private:
    float rate_per_s_;
    float value_;
};

} // namespace core

} // namespace foc
