// Conan::ImportStart
#pragma once
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Symmetric clamp. Returns `value` confined to [low, high].
 * @brief [zh] 对称/区间限幅：把 `value` 限制在 [low, high] 内返回。
 * @pre [en] low <= high.
 * @pre [zh] low <= high。
 * @context [en] Current-loop ISR: constexpr, noexcept, no allocation.
 * @context [zh] 电流环 ISR：constexpr、noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
template <typename T> constexpr auto clamp(T value, T low, T high) noexcept -> T {
    return (value < low) ? low : ((value > high) ? high : value);
}


/**
 * @brief [en] Moves its output towards a target no faster than a configured rate.
 *
 *     Symmetrically in both directions.
 *
 *     The budget for one step, \f$\rho\,\Delta t\f$, is applied to the **state** rather than to the error:
 *
 *     \f[ y \leftarrow \mathrm{clamp}(r,\; y - \rho\,\Delta t,\; y + \rho\,\Delta t) \qquad \text{for } \Delta t > 0
 *     \f]
 *
 *     That is what makes this a rate limit rather than a first-order lag -- the limiter keeps no memory of how far
 *     behind it was. \f$y\f$ is the limited value (the object's state), \f$r\f$ the requested value, \f$\rho\f$ the
 *     rate per second and \f$\Delta t\f$ the step length. All in the caller's units; the rate carries them per second.
 *     Ledger `E-SLEW-1`.
 *
 * @brief [zh] 让输出以不超过配置速率的步进逼近目标，正反两个方向对称。
 *
 *     一步的预算 \f$\rho\,\Delta t\f$ 作用在**状态**上而不是误差上：
 *
 *     \f[ y \leftarrow \mathrm{clamp}(r,\; y - \rho\,\Delta t,\; y + \rho\,\Delta t) \qquad \Delta t > 0 \f]
 *
 *     这正是它是速率限制而不是一阶滞后的原因——限制器不记得自己落后多少。\f$y\f$ 是限幅后的值（对象状态）、 \f$r\f$ 是请
 *     求值、\f$\rho\f$ 是每秒变化量、\f$\Delta t\f$ 是步长。全部取调用方单位；速率自带“每秒”。台账 `E-SLEW-1`。
 *
 * @warning [en] Nothing is refused here, and two cases look like errors but are not. A non-positive \f$\Delta t\f$ is a
 *               step in which no time passed, so the value holds and the caller sees the same number as last time. A
 *               non-positive rate freezes the output rather than rejecting the configuration: `step()` only multiplies
 *               the rate and never validates it, so the layer that builds the limiter from a nameplate is where a bad
 *               rate belongs.
 * @warning [zh] 这里不拒绝任何东西，而有两种情形看起来像错误其实不是。非正 \f$\Delta t\f$ 是没有时间经过的一步，于是值
 *               保持不变、调用方看到上一拍的数。非正速率会让输出冻住，而不是配置被拒绝：`step()` 只乘速率、从不校验它，
 *               所以该拦下坏速率的是用铭牌构造限制器的那一层。
 * @see `foc/core/limits.hpp` claim row, `test_limits.cpp` (`RampsTowardsTargetAtConfiguredRate`, `NonPositiveDtHoldsValue`), ledger `E-SLEW-1`
 */
class SlewLimiter final {
  public:
    /**
     * @brief [en] Construct a limiter with a maximum slew rate.
     * @brief [zh] 用最大斜率构造限制器。
     * @param rate_per_s [en] maximum change per second, must be > 0.
     * @param rate_per_s [zh] 每秒最大变化量，必须 > 0。
     * @since 0.1.0
     */
    explicit SlewLimiter(float rate_per_s) noexcept;

    /**
     * @brief [en] Advance one step towards the target.
     * @brief [zh] 向目标推进一步。
     * @param target [en] desired value.
     * @param target [zh] 目标值。
     * @param dt_s [en] step length in s; non-positive means "no time passed".
     * @param dt_s [zh] 步长，单位 s；非正值表示未经过时间。
     * @return [en] the limited value, which also becomes the new state.
     * @return [zh] 限幅后的值，同时作为新的内部状态。
     * @context [en] Current-loop ISR.
     * @context [zh] 电流环 ISR。
     * @since 0.1.0
     */
    auto step(float target, float dt_s) noexcept -> float;

    /**
     * @brief [en] Force the internal state (e.g. on mode change).
     * @brief [zh] 强制设置内部状态（例如模式切换时）。
     * @param value [en] new state.
     * @param value [zh] 新状态。
     * @since 0.1.0
     */
    void reset(float value) noexcept;

    /**
     * @brief [en] Current limited output.
     * @brief [zh] 当前限幅输出。
     * @since 0.1.0
     */
    [[nodiscard]] auto value() const noexcept -> float;

  private:
    float rate_per_s_;
    float value_;
};

} // namespace core

} // namespace foc
