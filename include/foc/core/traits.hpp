// Conan::ImportStart
#pragma once
#include <foc/core/alignment.hpp>
#include <foc/core/feedforward.hpp>
#include <foc/core/machine_params.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/position.hpp>
#include <foc/core/reference.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/trace.hpp>
#include <foc/core/types.hpp>
#include <type_traits>
#include <utility>
// Conan::ImportEnd

namespace foc {

namespace hal {

namespace concepts {

/**
 * @brief [en] Compile-time contract checks for the foc_hal interfaces (C++17 detection idiom; the project targets
 *             C++17, so `concept`/`requires` are not available).
 *
 * @brief [zh] foc_hal 接口的编译期契约检查（C++17 检测惯用法；项目目标是 C++17，没有 concept/requires）。
 *
 *     Each trait answers two questions at once: does the member exist with the documented signature, and is it
 *     `noexcept`? A stub that forgets a member fails at compile time with the message from the caller's
 *     `static_assert`, never at run time. 每个 trait 同时回答两件事：成员是否以文档化的签名存在、是否 `noexcept`。 漏掉
 *     成员的桩会在编译期失败（报调用点的 static_assert 消息），不会拖到运行期。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_pwm : std::false_type {};

/**
 * @brief [en] `T` satisfies the foc::hal::Pwm contract.
 * @brief [zh] `T` 满足 foc::hal::Pwm 契约。
 * @since 0.1.0
 * @exporter
 */
template <typename T>
struct is_pwm<T, std::void_t<decltype(std::declval<T&>().configure(std::declval<float>(), std::declval<float>())),
                             decltype(std::declval<T&>().write_duty(std::declval<core::Uvw<float>>())),
                             decltype(std::declval<T&>().enable()), decltype(std::declval<T&>().disable()),
                             decltype(std::declval<const T&>().fault())>>
    : std::bool_constant<
          std::is_same_v<decltype(std::declval<T&>().configure(std::declval<float>(), std::declval<float>())),
                         ErrorCode> &&
          noexcept(std::declval<T&>().configure(std::declval<float>(), std::declval<float>())) &&
          std::is_same_v<decltype(std::declval<T&>().write_duty(std::declval<core::Uvw<float>>())), void> &&
          noexcept(std::declval<T&>().write_duty(std::declval<core::Uvw<float>>())) &&
          std::is_same_v<decltype(std::declval<T&>().enable()), ErrorCode> && noexcept(std::declval<T&>().enable()) &&
          std::is_same_v<decltype(std::declval<T&>().disable()), void> && noexcept(std::declval<T&>().disable()) &&
          std::is_same_v<decltype(std::declval<const T&>().fault()), bool> &&
          noexcept(std::declval<const T&>().fault())> {};

/**
 * @brief [en] `T` satisfies the foc::hal::CurrentSense contract.
 * @brief [zh] `T` 满足 foc::hal::CurrentSense 契约。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_current_sense : std::false_type {};

template <typename T>
struct is_current_sense<
    T, std::void_t<decltype(std::declval<T&>().read_uvw()), decltype(std::declval<T&>().calibrate_offset()),
                   decltype(std::declval<const T&>().sample_rate_hz())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<T&>().read_uvw()), core::Uvw<float>> &&
                         noexcept(std::declval<T&>().read_uvw()) &&
                         std::is_same_v<decltype(std::declval<T&>().calibrate_offset()), ErrorCode> &&
                         noexcept(std::declval<T&>().calibrate_offset()) &&
                         std::is_same_v<decltype(std::declval<const T&>().sample_rate_hz()), float> &&
                         noexcept(std::declval<const T&>().sample_rate_hz())> {};

/**
 * @brief [en] `T` satisfies the foc::hal::PositionSensor contract.
 * @brief [zh] `T` 满足 foc::hal::PositionSensor 契约。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_position_sensor : std::false_type {};

template <typename T>
struct is_position_sensor<
    T, std::void_t<decltype(std::declval<T&>().angle_mech_rad()), decltype(std::declval<T&>().velocity_rad_s()),
                   decltype(std::declval<const T&>().has_sensor())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<T&>().angle_mech_rad()), float> &&
                         noexcept(std::declval<T&>().angle_mech_rad()) &&
                         std::is_same_v<decltype(std::declval<T&>().velocity_rad_s()), float> &&
                         noexcept(std::declval<T&>().velocity_rad_s()) &&
                         std::is_same_v<decltype(std::declval<const T&>().has_sensor()), bool> &&
                         noexcept(std::declval<const T&>().has_sensor())> {};

/**
 * @brief [en] `T` satisfies the foc::hal::DcLink contract.
 * @brief [zh] `T` 满足 foc::hal::DcLink 契约。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_dc_link : std::false_type {};

template <typename T>
struct is_dc_link<T, std::void_t<decltype(std::declval<T&>().voltage_v())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<T&>().voltage_v()), float> &&
                         noexcept(std::declval<T&>().voltage_v())> {};

/**
 * @brief [en] `T` satisfies the foc::hal::Clock contract.
 * @brief [zh] `T` 满足 foc::hal::Clock 契约。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_clock : std::false_type {};

template <typename T>
struct is_clock<T, std::void_t<decltype(std::declval<T&>().now_us())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<T&>().now_us()), std::uint64_t> &&
                         noexcept(std::declval<T&>().now_us())> {};

/**
 * @brief [en] Convenience variable templates, e.g. `static_assert(hal::concepts::is_pwm_v<MyPwm>, "MyPwm must satisfy
 *             foc::hal::Pwm");`.
 * @brief [zh] 便捷变量模板，例如 `static_assert(hal::concepts::is_pwm_v<MyPwm>, "MyPwm must satisfy foc::hal::Pwm");`。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_pwm_v = is_pwm<T>::value;

template <typename T> inline constexpr bool is_current_sense_v = is_current_sense<T>::value;

template <typename T> inline constexpr bool is_position_sensor_v = is_position_sensor<T>::value;

template <typename T> inline constexpr bool is_dc_link_v = is_dc_link<T>::value;

template <typename T> inline constexpr bool is_clock_v = is_clock<T>::value;

} // namespace concepts

} // namespace hal

namespace core {

namespace concepts {

/**
 * @brief [en] `T` exposes the optional `seed_flux(Ab<float>)` hook of a flux-linkage observer. Motor uses it during
 *             start(): a sensorless setup seeds the flux from the nameplate.
 * @brief [zh] `T` 是否提供磁链观测器可选的 `seed_flux(Ab<float>)` 钩子。 Motor 在 start() 中使用它：无感场景用铭牌参数
 *             播种磁链。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct has_seed_flux : std::false_type {};

template <typename T>
struct has_seed_flux<T, std::void_t<decltype(std::declval<T&>().seed_flux(std::declval<Ab<float>>()))>>
    : std::true_type {};

/**
 * @brief [en] Convenience variable template for the optional seed hook.
 * @brief [zh] 可选播种钩子的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_seed_flux_v = has_seed_flux<T>::value;

/**
 * @brief [en] `T` (an observer) asks for a voltage of its own: a high-frequency injection that Motor has to add to the
 *             command on its behalf.
 * @brief [zh] `T`（观测器）要自己的一份电压：Motor 需代它把高频注入叠加到指令上。
 * @note [en] Detection is capability, not configuration. An observer that needs no injection is simply a type that does
 *            not have the method, so there is no flag to leave in the wrong state -- the same rule the modulator slot
 *            follows for `is_modulator_v`.
 * @note [zh] 探测即能力，不是配置。不需要注入的观测器就是没有这个方法的类型，于是不存在会被设错的开关——与调制器槽位对
 *            `is_modulator_v` 遵循同一条规则。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct has_injection : std::false_type {};

template <typename T>
struct has_injection<T, std::void_t<decltype(std::declval<const T&>().injection_ab())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<const T&>().injection_ab()), Ab<float>> &&
                         noexcept(std::declval<const T&>().injection_ab())> {};

/**
 * @brief [en] Convenience variable template for the injection hook.
 * @brief [zh] 注入钩子的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_injection_v = has_injection<T>::value;

/**
 * @brief [en] `T` (an observer) can report whether its estimate currently means anything.
 * @brief [zh] `T`（观测器）能报告其估计当前是否有意义。
 * @note [en] The observer contract has no validity channel, so an observer that can be temporarily blind (a saliency
 *            loop with no carrier response) exposes `locked()` beside it. A blending observer uses it to decide whether
 *            the low-speed half has anything to contribute; an observer without it is assumed to always have an answer.
 * @note [zh] 观测器契约没有有效性通道，因此会暂时"看不见"的观测器（无载波响应的凸极环）在契约旁边提供 `locked()`。混合
 *            观测器用它判断低速那一半是否有东西可贡献；没有它的观测器视为永远有答案。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct has_locked : std::false_type {};

template <typename T>
struct has_locked<T, std::void_t<decltype(std::declval<const T&>().locked())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<const T&>().locked()), bool> &&
                         noexcept(std::declval<const T&>().locked())> {};

/**
 * @brief [en] Convenience variable template for the lock hook.
 * @brief [zh] 锁定钩子的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_locked_v = has_locked<T>::value;

/**
 * @brief [en] `T` (an observer) can be handed a starting angle and speed.
 * @brief [zh] `T`（观测器）可以接受一个起始角度与转速。
 * @note [en] Needed by anything that switches an observer off and on again: an observer that stops tracking while it is
 *            not being used comes back with a stale estimate, and if its pull-in window is narrower than a full turn (a
 *            saliency loop's is half of one) it cannot re-acquire from there. Handing it the current best angle is the
 *            difference between "can resume" and "must start over".
 * @note [zh] 任何会把观测器关掉再打开的机制都需要它：不被使用期间停止跟踪的观测器回来时带着过时的估计，而若它的牵引窗口
 *            小于一整圈（凸极环只有半圈），就无法从中重新捕获。把当前最好的角度交给它，是 "可以继续"与"只能重来"之间 的
 *            区别。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct has_seed_angle : std::false_type {};

template <typename T>
struct has_seed_angle<
    T, std::void_t<decltype(std::declval<T&>().seed_angle(std::declval<float>(), std::declval<float>()))>>
    : std::bool_constant<noexcept(std::declval<T&>().seed_angle(std::declval<float>(), std::declval<float>()))> {};

/**
 * @brief [en] Convenience variable template for the seed-angle hook.
 * @brief [zh] 播种角度钩子的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_seed_angle_v = has_seed_angle<T>::value;

/**
 * @brief [en] `T` (a policy) can take an updated set of machine parameters at runtime.
 * @brief [zh] `T`（策略）可以在运行期接受一组更新后的机器参数。
 * @note [en] Needed because identification happens *after* the policies are built: an observer tuned for the nameplate
 *            has to be told what the machine turned out to be, without the caller rebuilding the whole motor. A policy
 *            without this hook simply keeps its construction-time numbers, which is the right answer for anything that
 *            does not model the machine at all.
 * @note [zh] 因为辨识发生在策略**构建之后**：按铭牌整定的观测器需要被告知这台机器实际上是什么样，而调用方不必重建整个电
 *            机。没有这个钩子的策略就保持构造时的数字——对任何不建模机器的组件来说，这就是正确答案。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct has_apply_params : std::false_type {};

template <typename T>
struct has_apply_params<T, std::void_t<decltype(std::declval<T&>().apply_params(std::declval<const MachineParams&>()))>>
    : std::bool_constant<noexcept(std::declval<T&>().apply_params(std::declval<const MachineParams&>()))> {};

/**
 * @brief [en] Convenience variable template for the parameter hook.
 * @brief [zh] 参数钩子的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_apply_params_v = has_apply_params<T>::value;

/**
 * @brief [en] `T` satisfies the observer policy contract (see observer_policy.hpp): it must expose `update(Ab<float>,
 *             Ab<float>, float) noexcept -> ObsState`.
 * @brief [zh] `T` 满足观测器策略契约（见 observer_policy.hpp）：必须提供 `update(Ab<float>, Ab<float>, float) noexcept
 *             -> ObsState`。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_observer : std::false_type {};

template <typename T>
struct is_observer<T, std::void_t<decltype(std::declval<T&>().update(
                          std::declval<Ab<float>>(), std::declval<Ab<float>>(), std::declval<float>()))>>
    : std::bool_constant<
          std::is_same_v<decltype(std::declval<T&>().update(std::declval<Ab<float>>(), std::declval<Ab<float>>(),
                                                            std::declval<float>())),
                         ObsState> &&
          noexcept(std::declval<T&>().update(std::declval<Ab<float>>(), std::declval<Ab<float>>(),
                                             std::declval<float>()))> {};

/**
 * @brief [en] Convenience variable template for the observer contract.
 * @brief [zh] 观测器契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_observer_v = is_observer<T>::value;

/**
 * @brief [en] `T` satisfies the reference-generator policy contract (see reference.hpp): it must expose `generate(const
 *             ReferenceInput&) noexcept -> Dq<float>`.
 * @brief [zh] `T` 满足参考生成器策略契约（见 reference.hpp）：必须提供 `generate(const ReferenceInput&) noexcept ->
 *             Dq<float>`。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_reference_generator : std::false_type {};

template <typename T>
struct is_reference_generator<T, std::void_t<decltype(std::declval<T&>().generate(std::declval<ReferenceInput>()))>>
    : std::bool_constant<
          std::is_same_v<decltype(std::declval<T&>().generate(std::declval<ReferenceInput>())), Dq<float>> &&
          noexcept(std::declval<T&>().generate(std::declval<ReferenceInput>()))> {};

/**
 * @brief [en] Convenience variable template for the reference-generator contract.
 * @brief [zh] 参考生成器契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_reference_generator_v = is_reference_generator<T>::value;

/**
 * @brief [en] `T` satisfies the feedforward policy contract (see feedforward.hpp): it must expose `generate(const
 *             FeedforwardInput&) noexcept -> Dq<float>`.
 * @brief [zh] `T` 满足前馈策略契约（见 feedforward.hpp）：必须提供 `generate(const FeedforwardInput&) noexcept ->
 *             Dq<float>`。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_feedforward : std::false_type {};

template <typename T>
struct is_feedforward<T, std::void_t<decltype(std::declval<T&>().generate(std::declval<FeedforwardInput>()))>>
    : std::bool_constant<
          std::is_same_v<decltype(std::declval<T&>().generate(std::declval<FeedforwardInput>())), Dq<float>> &&
          noexcept(std::declval<T&>().generate(std::declval<FeedforwardInput>()))> {};

/**
 * @brief [en] Convenience variable template for the feedforward contract.
 * @brief [zh] 前馈契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_feedforward_v = is_feedforward<T>::value;

/**
 * @brief [en] `T` (a HalPolicy) exposes an optional `clock` member satisfying `hal::Clock`.
 *
 *     Deliberately optional: a HAL without a clock is legal, and a member that happens to be named `clock` but does not
 *     satisfy the contract is treated as absent rather than as an error, so an unrelated field name never breaks a
 *     build.
 *
 * @brief [zh] `T`（HalPolicy）是否暴露了满足 `hal::Clock` 的可选成员 `clock`。
 *
 *     故意设计成可选：没有时钟的 HAL 是合法的；名字恰好叫 `clock` 但不满足契约的成员按 “不存在”处理而不是报错，于是无关
 *     字段名永远不会把构建弄红。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct has_clock : std::false_type {};

template <typename T>
struct has_clock<T, std::void_t<decltype(std::declval<T&>().clock)>>
    : std::bool_constant<hal::concepts::is_clock_v<decltype(std::declval<T&>().clock)>> {};

/**
 * @brief [en] Convenience variable template for the optional clock member.
 * @brief [zh] 可选时钟成员的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_clock_v = has_clock<T>::value;

/**
 * @brief [en] `T` satisfies the controller-slot contract: `step(setpoint, feedback, dt) noexcept -> float` plus
 *             `reset()`. The same shape fills all four controller slots (current d, current q, speed, position), which
 *             is why one trait covers them all.
 * @brief [zh] `T` 满足控制器槽位契约：`step(setpoint, feedback, dt) noexcept -> float` 与 `reset()`。四个控制器槽位（d
 *             轴电流、q 轴电流、速度、位置）形状相同，所以用一个 trait 覆盖全部。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_controller : std::false_type {};

template <typename T>
struct is_controller<
    T, std::void_t<decltype(std::declval<T&>().step(0.0F, 0.0F, 0.0F)), decltype(std::declval<T&>().reset())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<T&>().step(0.0F, 0.0F, 0.0F)), float> &&
                         noexcept(std::declval<T&>().step(0.0F, 0.0F, 0.0F)) && noexcept(std::declval<T&>().reset())> {
};

/**
 * @brief [en] Convenience variable template for the controller-slot contract.
 * @brief [zh] 控制器槽位契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_controller_v = is_controller<T>::value;

/**
 * @brief [en] `T` satisfies the *widened* modulator contract: `modulate(const ModulatorInput&) noexcept ->
 *             SvpwmResult`.
 * @brief [zh] `T` 满足**加宽后的**调制器契约：`modulate(const ModulatorInput&) noexcept -> SvpwmResult`。
 * @note [en] Separate from the two-argument form so every existing modulator keeps working: `Motor` asks the widened
 *            one when it exists and the narrow one otherwise. The difference is a capability, not a preference -- a
 *            modulator that never sees the phase currents cannot linearise the inverter, and saying so in the type
 *            system is cheaper than documenting it.
 * @note [zh] 与两参数形式分开，好让所有既有调制器继续工作：存在加宽形式时 `Motor` 用它，否则用窄的。 这是**能力**差异而
 *            非偏好——从不接触相电流的调制器无法线性化逆变器，把它写在类型系统里比写在文档里便宜。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_modulator : std::false_type {};

template <typename T>
struct is_modulator<T, std::void_t<decltype(std::declval<T&>().modulate(std::declval<ModulatorInput>()))>>
    : std::bool_constant<
          std::is_same_v<decltype(std::declval<T&>().modulate(std::declval<ModulatorInput>())), SvpwmResult> &&
          noexcept(std::declval<T&>().modulate(std::declval<ModulatorInput>()))> {};

/**
 * @brief [en] Convenience variable template for the widened modulator contract.
 * @brief [zh] 加宽调制器契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_modulator_v = is_modulator<T>::value;

/**
 * @brief [en] `T` (used in the speed slot) accepts a `SpeedInput`, i.e. it can scale its own gains for one call.
 * @brief [zh] `T`（用作速度槽位）能接收 `SpeedInput`，即可以按次缩放自身增益。
 * @note [en] Separate from `is_controller_v` so a plain three-argument controller still fills the speed slot -- it
 *            simply cannot do compliance. `Motor` reports `NotImpl` when a move asks for scaling that the slot cannot
 *            deliver, instead of ignoring the request.
 * @note [zh] 与 `is_controller_v` 分开，这样一个普通的三参数控制器仍能占速度槽位——只是做不到柔顺。当运动要求了槽位交付
 *            不了的缩放时，`Motor` 报 `NotImpl`，而不是把请求忽略掉。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_speed_control : std::false_type {};

template <typename T>
struct is_speed_control<T, std::void_t<decltype(std::declval<T&>().step(std::declval<SpeedInput>(), 0.0F, 0.0F)),
                                       decltype(std::declval<T&>().reset())>>
    : std::bool_constant<
          std::is_same_v<decltype(std::declval<T&>().step(std::declval<SpeedInput>(), 0.0F, 0.0F)), float> &&
          noexcept(std::declval<T&>().step(std::declval<SpeedInput>(), 0.0F, 0.0F)) &&
          noexcept(std::declval<T&>().reset())> {};

/**
 * @brief [en] Convenience variable template for the scaled speed-slot contract.
 * @brief [zh] 可缩放速度槽位契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_speed_control_v = is_speed_control<T>::value;

/**
 * @brief [en] `T` satisfies the *position* slot contract: `step(const PositionInput&, float, float) noexcept -> float`
 *             plus `reset()`.
 * @brief [zh] `T` 满足**位置**槽位契约：`step(const PositionInput&, float, float) noexcept -> float` 与 `reset()`。
 * @note [en] Deliberately *not* the same shape as the current and speed slots. Those three are regulators: a setpoint,
 *            a measurement, a step. The position slot is a trajectory shaper too, and a move has a velocity and an
 *            acceleration, not just an endpoint. Widening this one slot is what makes per-move limits possible without
 *            teaching every `Pi` about acceleration.
 * @note [zh] 与电流、速度槽位的形状**故意不同**。那三个是调节器：给定、实测、步长。位置槽位同时还是轨迹塑造器，而一次运
 *            动有速度和加速度，不只有一个终点。把这个槽位加宽，才能在不让每个 `Pi` 都去理解加速度的前提下支持按次限制。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_position_control : std::false_type {};

template <typename T>
struct is_position_control<T, std::void_t<decltype(std::declval<T&>().step(std::declval<PositionInput>(), 0.0F, 0.0F)),
                                          decltype(std::declval<T&>().reset())>>
    : std::bool_constant<
          std::is_same_v<decltype(std::declval<T&>().step(std::declval<PositionInput>(), 0.0F, 0.0F)), float> &&
          noexcept(std::declval<T&>().step(std::declval<PositionInput>(), 0.0F, 0.0F)) &&
          noexcept(std::declval<T&>().reset())> {};

/**
 * @brief [en] Convenience variable template for the position-slot contract.
 * @brief [zh] 位置槽位契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_position_control_v = is_position_control<T>::value;

/**
 * @brief [en] `T` satisfies the aligner-slot contract (see alignment.hpp): `validate()`, `begin(pole_pairs) -> bool`,
 *             `step(dt, angle) noexcept -> AlignOutput`, `reset()`.
 * @brief [zh] `T` 满足对齐槽位契约（见 alignment.hpp）：`validate()`、 `begin(pole_pairs) -> bool`、`step(dt, angle)
 *             noexcept -> AlignOutput`、`reset()`。
 * @note [en] `begin()` returns `bool` rather than `ErrorCode` on purpose: "nothing to do" is a normal answer (a build
 *            with a stored offset), while "your recipe is invalid" is a configuration error reported by `validate()`
 *            before the sequence starts. Folding both into one return value would let a typo in the recipe look like a
 *            machine that needs no alignment.
 * @note [zh] `begin()` 返回 `bool` 而非 `ErrorCode` 是故意的：“无事可做”是正常回答（偏置已存的 构建），而“配方非法”是配
 *            置错误，由 `validate()` 在流程开始前报出。把两者合成一个返回值，会让配方里的笔误看起来像一台不需要对齐的电
 *            机。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_aligner : std::false_type {};

template <typename T>
struct is_aligner<T, std::void_t<decltype(std::declval<T&>().validate()), decltype(std::declval<T&>().begin(0U)),
                                 decltype(std::declval<T&>().step(0.0F, 0.0F)), decltype(std::declval<T&>().reset())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<T&>().validate()), ErrorCode> &&
                         noexcept(std::declval<T&>().validate()) &&
                         std::is_same_v<decltype(std::declval<T&>().begin(0U)), bool> &&
                         noexcept(std::declval<T&>().begin(0U)) &&
                         std::is_same_v<decltype(std::declval<T&>().step(0.0F, 0.0F)), AlignOutput> &&
                         noexcept(std::declval<T&>().step(0.0F, 0.0F)) && noexcept(std::declval<T&>().reset())> {};

/**
 * @brief [en] Convenience variable template for the aligner contract.
 * @brief [zh] 对齐器契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_aligner_v = is_aligner<T>::value;

/**
 * @brief [en] `T` (a position sensor) accepts a mechanical offset and reports the one in force.
 * @brief [zh] `T`（位置传感器）可接受机械偏置并回报当前生效值。
 * @note [en] Optional by design. The HAL contract already says `angle_mech_rad()` is free of the mounting offset, so a
 *            sensor may own that correction; one that does not (a sensor whose offset is written by the platform at
 *            boot) simply lacks these members and `Motor` leaves it alone. The aligner still reports the offset either
 *            way, through `Motor::aligner()`.
 * @note [zh] 故意做成可选。HAL 契约已声明 `angle_mech_rad()` 已消除安装偏置，所以传感器可以自己 拥有这个校正；不具备这
 *            两个成员的传感器（例如偏置由平台在启动时写入的那种）就按“不存在” 处理，`Motor` 不会去动它。两种情况下对齐
 *            器都仍然通过 `Motor::aligner()` 报出偏置。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct has_mech_offset : std::false_type {};

template <typename T>
struct has_mech_offset<T, std::void_t<decltype(std::declval<T&>().set_offset_mech_rad(0.0F)),
                                      decltype(std::declval<const T&>().mech_offset_rad())>>
    : std::bool_constant<noexcept(std::declval<T&>().set_offset_mech_rad(0.0F)) &&
                         std::is_same_v<decltype(std::declval<const T&>().mech_offset_rad()), float> &&
                         noexcept(std::declval<const T&>().mech_offset_rad())> {};

/**
 * @brief [en] Convenience variable template for the optional mechanical-offset hook.
 * @brief [zh] 可选机械偏置钩子的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_mech_offset_v = has_mech_offset<T>::value;

/**
 * @brief [en] `T` (a position sensor) can report an index mark.
 * @brief [zh] `T`（位置传感器）能报告索引标记。
 * @note [en] Optional, and optional for a reason recorded in `hal/position.hpp`: a sensor without a mark is a perfectly
 *            ordinary sensor, it simply cannot establish an absolute position, and a caller that needs one has to be
 *            told rather than sold a number that means something else. Both members are required together, because a
 *            mark that can be asked about but never pulses is not a mark.
 * @note [zh] 可选，理由记在 `hal/position.hpp`：没有标记的传感器是完全正常的传感器，它只是无法建立绝对位置，而需要绝对
 *            位置的调用方必须**被告知**这件事，而不是拿到一个含义不同的数字。两个成员必须成对存在——能问"有没有标记"却永
 *            远不会给出脉冲的，不算标记。
 * @since 0.1.6
 * @exporter
 */
template <typename T, typename = void> struct has_index : std::false_type {};

template <typename T>
struct has_index<
    T, std::void_t<decltype(std::declval<const T&>().has_index()), decltype(std::declval<T&>().index_pulse())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<const T&>().has_index()), bool> &&
                         noexcept(std::declval<const T&>().has_index()) &&
                         std::is_same_v<decltype(std::declval<T&>().index_pulse()), bool> &&
                         noexcept(std::declval<T&>().index_pulse())> {};

/**
 * @brief [en] Convenience variable template for the optional index hook.
 * @brief [zh] 可选索引钩子的便捷变量模板。
 * @since 0.1.6
 * @exporter
 */
template <typename T> inline constexpr bool has_index_v = has_index<T>::value;

/**
 * @brief [en] `T` (an aligner) reports the captured offset via `offset_mech_rad() noexcept -> float`.
 * @brief [zh] `T`（对齐器）通过 `offset_mech_rad() noexcept -> float` 报出捕获到的偏置。
 * @note [en] Separate from `is_aligner_v` because the offset is the aligner's *result*, not part of the contract the
 *            motor needs to drive the sequence. An aligner that only diagnoses (or one that writes the offset itself)
 *            has nothing to hand back.
 * @note [zh] 与 `is_aligner_v` 分开：偏置是对齐器的**结果**，不是电机驱动流程所必需的契约。只做诊断、或自己写回偏置的对
 *            齐器没有东西要交回来。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct has_align_offset : std::false_type {};

template <typename T>
struct has_align_offset<T, std::void_t<decltype(std::declval<const T&>().offset_mech_rad())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<const T&>().offset_mech_rad()), float> &&
                         noexcept(std::declval<const T&>().offset_mech_rad())> {};

/**
 * @brief [en] Convenience variable template for the aligner offset result.
 * @brief [zh] 对齐器偏置结果的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_align_offset_v = has_align_offset<T>::value;

/**
 * @brief [en] `T` is a trace sink: something the control loop can hand one `TraceSample` per period.
 * @brief [zh] `T` 是轨迹接收端：控制环每周期可以交给它一个 `TraceSample`。
 * @note [en] Deliberately about what the loop *calls*, not about which class it is: `NullTrace` satisfies it (with
 *            no-ops) so the loop has one code path, and a test double that counts pushes satisfies it without being a
 *            ring at all. That is the difference between a policy and a special case.
 * @note [zh] 它描述的是环**调用**什么，而不是那个类是谁：`NullTrace` 满足它（全部空操作），于是环只有一条代码路径；一个
 *            只数调用次数的测试替身也满足它，而它根本不是环。这就是策略与特例的区别。
 * @since 0.1.8
 * @exporter
 */
template <typename T, typename = void> struct is_trace_sink : std::false_type {};

template <typename T>
struct is_trace_sink<
    T, std::void_t<decltype(std::declval<T&>().push(std::declval<const TraceSample&>())),
                   decltype(std::declval<const T&>().size()), decltype(std::declval<const T&>().dropped())>>
    : std::true_type {};

/**
 * @brief [en] Convenience variable template for the trace sink contract.
 * @brief [zh] 轨迹接收端契约的便捷变量模板。
 * @since 0.1.8
 * @exporter
 */
template <typename T> inline constexpr bool is_trace_sink_v = is_trace_sink<T>::value;

/**
 * @brief [en] `T` can say which observer mode it is currently running.
 * @brief [zh] `T` 能说出它当前在跑哪个观测器模式。
 * @note [en] `has_injection_v` answers a question about a *type*; a policy that can switch modes answers a different
 *            one at run time, and the two are not the same question. A slot holding an injector but running back-EMF
 *            injects nothing, so anything that has to decide "will this inject?" needs this trait to ask.
 * @note [zh] `has_injection_v` 回答的是关于**类型**的问题；能切换模式的策略在运行时回答的是另一个问题，两者并不相同。一
 *            个装着注入器却在跑反电势的槽位不注入任何东西，因此任何要判断"它会不会注入"的地方都需要这个特型来提问。
 * @since 0.1.8
 * @exporter
 */
template <typename T, typename = void> struct has_kind : std::false_type {};

template <typename T> struct has_kind<T, std::void_t<decltype(std::declval<const T&>().kind())>> : std::true_type {};

/**
 * @brief [en] Convenience variable template for "can report its mode".
 * @brief [zh] "能上报自身模式"的便捷变量模板。
 * @since 0.1.8
 * @exporter
 */
template <typename T> inline constexpr bool has_kind_v = has_kind<T>::value;

/**
 * @brief [en] `T` can say whether it is injecting *right now*, as opposed to whether it could.
 * @brief [zh] `T` 能说出它**此刻**是否在注入，而不是它是否具备注入能力。
 * @since 0.1.8
 * @exporter
 */
template <typename T, typename = void> struct has_injecting : std::false_type {};

template <typename T>
struct has_injecting<T, std::void_t<decltype(std::declval<const T&>().injecting())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<const T&>().injecting()), bool>> {};

/**
 * @brief [en] Convenience variable template for the run-time injection query.
 * @brief [zh] 运行时注入查询的便捷变量模板。
 * @since 0.1.8
 * @exporter
 */
template <typename T> inline constexpr bool has_injecting_v = has_injecting<T>::value;

} // namespace concepts

} // namespace core

} // namespace foc
