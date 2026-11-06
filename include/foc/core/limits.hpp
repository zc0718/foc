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
 * @brief [en] Moves its output towards a target no faster than a configured rate, symmetrically in both directions.
 *
 *     **Equation.** `y <- clamp(target, y - rate*dt, y + rate*dt)` for `dt > 0`, and `y` holds otherwise; ledger
 *     `E-SLEW-1`.
 *
 *     **Notation.** `y` the limited value (the object's state), `target` the requested value, `rate` per second, `dt`
 *     the step length in s. All in the caller's units; the rate carries them per second.
 *
 *     **Guarantees.** The change per step never exceeds `rate*dt`, in either direction, so a step command cannot
 *     produce a jump larger than the configured slope however far away the target is. The budget is applied to the
 *     state rather than to the error, which is what makes this a rate limit instead of a first-order lag: the limiter
 *     keeps no memory of how far behind it was.
 *
 *     **Refusals.** Nothing is refused. A non-positive `dt` is not an error here -- it is a step in which no time
 *     passed, so the value holds and the caller sees the same number it saw last time. A non-positive `rate` makes
 *     every step return the held value, which is a frozen output rather than a rejected configuration; the crate that
 *     builds the limiter from a nameplate is where a bad rate should be caught.
 *
 *     **Evidence.** The `foc/core/limits.hpp` claim row, `test_limits.cpp` (`RampsTowardsTargetAtConfiguredRate`,
 *     `ReachesTargetWhenInsideOneStep`, `LimitsNegativeDirection`, `NonPositiveDtHoldsValue`), ledger `E-SLEW-1`.
 *
 * @brief [zh] 让输出以不超过配置速率的步进逼近目标，正反两个方向对称。
 *
 *     **Equation.** `dt > 0` 时 `y ← clamp(target, y − rate·dt, y + rate·dt)`，否则 `y` 保持不变；台账 `E-SLEW-1`。
 *
 *     **Notation.** `y` 限幅后的值（即对象状态）、`target` 请求值、`rate` 每秒变化量、`dt` 步长，s。全部取调用方单位；
 *     速率自带“每秒”。
 *
 *     **Guarantees.** 每一步的变化不超过 `rate·dt`，两个方向都一样：无论目标多远，阶跃指令都不会造成大于配置斜率的跳
 *     变。预算作用在**状态**上而不是误差上，因此这是速率限制而不是一阶滞后——限制器不记得自己落后多少。
 *
 *     **Refusals.** 这里不拒绝任何东西。非正 `dt` 不是错误——那是没有时间经过的一步，于是值保持不变、调用方看到的是上一
 *     拍的数。非正 `rate` 会让每一步都返回保持值，也就是输出被冻住，而不是配置被拒绝；该在校验处拦下的是用铭牌构造限制
 *     器的那一层。
 *
 *     **Evidence.** `foc/core/limits.hpp` 的 claim 行、`test_limits.cpp`（`RampsTowardsTargetAtConfiguredRate`、
 *     `ReachesTargetWhenInsideOneStep`、`LimitsNegativeDirection`、`NonPositiveDtHoldsValue`）、台账 `E-SLEW-1`。
 *
 * @context [en] Current-loop ISR: noexcept, no allocation.
 * @context [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
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
