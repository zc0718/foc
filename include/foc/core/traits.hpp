// Conan::ImportStart
#pragma once
#include <type_traits>
#include <utility>
#include <foc/core/alignment.hpp>
#include <foc/core/feedforward.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/position.hpp>
#include <foc/core/reference.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace hal {

namespace concepts {


/**
 * @brief [en] Compile-time contract checks for the foc_hal interfaces (C++17 detection idiom;
 *        the project targets C++17, so `concept`/`requires` are not available).
 *        [zh] foc_hal 接口的编译期契约检查（C++17 检测惯用法；项目目标是 C++17，没有 concept/requires）。
 *
 *        Each trait answers two questions at once: does the member exist with the documented
 *        signature, and is it `noexcept`? A stub that forgets a member fails at compile time with
 *        the message from the caller's `static_assert`, never at run time.
 *        每个 trait 同时回答两件事：成员是否以文档化的签名存在、是否 `noexcept`。
 *        漏掉成员的桩会在编译期失败（报调用点的 static_assert 消息），不会拖到运行期。
 *
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_pwm : std::false_type {};


/**
 * @brief [en] `T` satisfies the foc::hal::Pwm contract.
 *        [zh] `T` 满足 foc::hal::Pwm 契约。
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
 *        [zh] `T` 满足 foc::hal::CurrentSense 契约。
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
 *        [zh] `T` 满足 foc::hal::PositionSensor 契约。
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
 *        [zh] `T` 满足 foc::hal::DcLink 契约。
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
 *        [zh] `T` 满足 foc::hal::Clock 契约。
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct is_clock : std::false_type {};

template <typename T>
struct is_clock<T, std::void_t<decltype(std::declval<T&>().now_us())>>
    : std::bool_constant<std::is_same_v<decltype(std::declval<T&>().now_us()), std::uint64_t> &&
                         noexcept(std::declval<T&>().now_us())> {};


/**
 * @brief [en] Convenience variable templates, e.g.
 *        `static_assert(hal::concepts::is_pwm_v<MyPwm>, "MyPwm must satisfy foc::hal::Pwm");`.
 *        [zh] 便捷变量模板，例如
 *        `static_assert(hal::concepts::is_pwm_v<MyPwm>, "MyPwm must satisfy foc::hal::Pwm");`。
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
 * @brief [en] `T` exposes the optional `seed_flux(Ab<float>)` hook of a flux-linkage observer.
 *        Motor uses it during start(): a sensorless setup seeds the flux from the nameplate.
 *        [zh] `T` 是否提供磁链观测器可选的 `seed_flux(Ab<float>)` 钩子。
 *        Motor 在 start() 中使用它：无感场景用铭牌参数播种磁链。
 *
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct has_seed_flux : std::false_type {};

template <typename T>
struct has_seed_flux<T, std::void_t<decltype(std::declval<T&>().seed_flux(std::declval<Ab<float>>()))>>
    : std::true_type {};


/**
 * @brief [en] Convenience variable template for the optional seed hook.
 *        [zh] 可选播种钩子的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_seed_flux_v = has_seed_flux<T>::value;


/**
 * @brief [en] `T` satisfies the observer policy contract (see observer_policy.hpp): it must expose
 *        `update(Ab<float>, Ab<float>, float) noexcept -> ObsState`.
 *        [zh] `T` 满足观测器策略契约（见 observer_policy.hpp）：必须提供
 *        `update(Ab<float>, Ab<float>, float) noexcept -> ObsState`。
 *
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
 *        [zh] 观测器契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_observer_v = is_observer<T>::value;


/**
 * @brief [en] `T` satisfies the reference-generator policy contract (see reference.hpp): it must
 *        expose `generate(const ReferenceInput&) noexcept -> Dq<float>`.
 *        [zh] `T` 满足参考生成器策略契约（见 reference.hpp）：必须提供
 *        `generate(const ReferenceInput&) noexcept -> Dq<float>`。
 *
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
 *        [zh] 参考生成器契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_reference_generator_v = is_reference_generator<T>::value;


/**
 * @brief [en] `T` satisfies the feedforward policy contract (see feedforward.hpp): it must
 *        expose `generate(const FeedforwardInput&) noexcept -> Dq<float>`.
 *        [zh] `T` 满足前馈策略契约（见 feedforward.hpp）：必须提供
 *        `generate(const FeedforwardInput&) noexcept -> Dq<float>`。
 *
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
 *        [zh] 前馈契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_feedforward_v = is_feedforward<T>::value;


/**
 * @brief [en] `T` (a HalPolicy) exposes an optional `clock` member satisfying `hal::Clock`.
 *        [zh] `T`（HalPolicy）是否暴露了满足 `hal::Clock` 的可选成员 `clock`。
 *
 *        [en] Deliberately optional: a HAL without a clock is legal, and a member that happens to be
 *        named `clock` but does not satisfy the contract is treated as absent rather than as an
 *        error, so an unrelated field name never breaks a build.
 *        [zh] 故意设计成可选：没有时钟的 HAL 是合法的；名字恰好叫 `clock` 但不满足契约的成员按
 *        “不存在”处理而不是报错，于是无关字段名永远不会把构建弄红。
 *
 * @since 0.1.0
 * @exporter
 */
template <typename T, typename = void> struct has_clock : std::false_type {};

template <typename T>
struct has_clock<T, std::void_t<decltype(std::declval<T&>().clock)>>
    : std::bool_constant<hal::concepts::is_clock_v<decltype(std::declval<T&>().clock)>> {};


/**
 * @brief [en] Convenience variable template for the optional clock member.
 *        [zh] 可选时钟成员的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_clock_v = has_clock<T>::value;


/**
 * @brief [en] `T` satisfies the controller-slot contract: `step(setpoint, feedback, dt) noexcept ->
 *        float` plus `reset()`. The same shape fills all four controller slots (current d, current q,
 *        speed, position), which is why one trait covers them all.
 *        [zh] `T` 满足控制器槽位契约：`step(setpoint, feedback, dt) noexcept -> float` 与
 *        `reset()`。四个控制器槽位（d 轴电流、q 轴电流、速度、位置）形状相同，所以用一个 trait
 *        覆盖全部。
 *
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
 *        [zh] 控制器槽位契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_controller_v = is_controller<T>::value;


/**
 * @brief [en] `T` (used in the speed slot) accepts a `SpeedInput`, i.e. it can scale its own gains
 *        for one call.
 *        [zh] `T`（用作速度槽位）能接收 `SpeedInput`，即可以按次缩放自身增益。
 *
 * @note [en] Separate from `is_controller_v` so a plain three-argument controller still fills the
 *       speed slot -- it simply cannot do compliance. `Motor` reports `NotImpl` when a move asks
 *       for scaling that the slot cannot deliver, instead of ignoring the request.
 *       [zh] 与 `is_controller_v` 分开，这样一个普通的三参数控制器仍能占速度槽位——只是做不到柔顺。
 *       当运动要求了槽位交付不了的缩放时，`Motor` 报 `NotImpl`，而不是把请求忽略掉。
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
 *        [zh] 可缩放速度槽位契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_speed_control_v = is_speed_control<T>::value;


/**
 * @brief [en] `T` satisfies the *position* slot contract: `step(const PositionInput&, float, float)
 *        noexcept -> float` plus `reset()`.
 *        [zh] `T` 满足**位置**槽位契约：`step(const PositionInput&, float, float) noexcept -> float`
 *        与 `reset()`。
 *
 * @note [en] Deliberately *not* the same shape as the current and speed slots. Those three are
 *       regulators: a setpoint, a measurement, a step. The position slot is a trajectory shaper too,
 *       and a move has a velocity and an acceleration, not just an endpoint. Widening this one slot
 *       is what makes per-move limits possible without teaching every `Pi` about acceleration.
 *       [zh] 与电流、速度槽位的形状**故意不同**。那三个是调节器：给定、实测、步长。位置槽位同时还
 *       是轨迹塑造器，而一次运动有速度和加速度，不只有一个终点。把这个槽位加宽，才能在不让每个
 *       `Pi` 都去理解加速度的前提下支持按次限制。
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
 *        [zh] 位置槽位契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_position_control_v = is_position_control<T>::value;


/**
 * @brief [en] `T` satisfies the aligner-slot contract (see alignment.hpp): `validate()`,
 *        `begin(pole_pairs) -> bool`, `step(dt, angle) noexcept -> AlignOutput`, `reset()`.
 *        [zh] `T` 满足对齐槽位契约（见 alignment.hpp）：`validate()`、
 *        `begin(pole_pairs) -> bool`、`step(dt, angle) noexcept -> AlignOutput`、`reset()`。
 *
 * @note [en] `begin()` returns `bool` rather than `ErrorCode` on purpose: "nothing to do" is a
 *       normal answer (a build with a stored offset), while "your recipe is invalid" is a
 *       configuration error reported by `validate()` before the sequence starts. Folding both
 *       into one return value would let a typo in the recipe look like a machine that needs no
 *       alignment.
 *       [zh] `begin()` 返回 `bool` 而非 `ErrorCode` 是故意的：“无事可做”是正常回答（偏置已存的
 *       构建），而“配方非法”是配置错误，由 `validate()` 在流程开始前报出。把两者合成一个返回值，
 *       会让配方里的笔误看起来像一台不需要对齐的电机。
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
 *        [zh] 对齐器契约的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool is_aligner_v = is_aligner<T>::value;


/**
 * @brief [en] `T` (a position sensor) accepts a mechanical offset and reports the one in force.
 *        [zh] `T`（位置传感器）可接受机械偏置并回报当前生效值。
 *
 * @note [en] Optional by design. The HAL contract already says `angle_mech_rad()` is free of the
 *       mounting offset, so a sensor may own that correction; one that does not (a sensor whose
 *       offset is written by the platform at boot) simply lacks these members and `Motor` leaves
 *       it alone. The aligner still reports the offset either way, through `Motor::aligner()`.
 *       [zh] 故意做成可选。HAL 契约已声明 `angle_mech_rad()` 已消除安装偏置，所以传感器可以自己
 *       拥有这个校正；不具备这两个成员的传感器（例如偏置由平台在启动时写入的那种）就按“不存在”
 *       处理，`Motor` 不会去动它。两种情况下对齐器都仍然通过 `Motor::aligner()` 报出偏置。
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
 *        [zh] 可选机械偏置钩子的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_mech_offset_v = has_mech_offset<T>::value;


/**
 * @brief [en] `T` (an aligner) reports the captured offset via `offset_mech_rad() noexcept -> float`.
 *        [zh] `T`（对齐器）通过 `offset_mech_rad() noexcept -> float` 报出捕获到的偏置。
 *
 * @note [en] Separate from `is_aligner_v` because the offset is the aligner's *result*, not part of
 *       the contract the motor needs to drive the sequence. An aligner that only diagnoses (or one
 *       that writes the offset itself) has nothing to hand back.
 *       [zh] 与 `is_aligner_v` 分开：偏置是对齐器的**结果**，不是电机驱动流程所必需的契约。只做
 *       诊断、或自己写回偏置的对齐器没有东西要交回来。
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
 *        [zh] 对齐器偏置结果的便捷变量模板。
 * @since 0.1.0
 * @exporter
 */
template <typename T> inline constexpr bool has_align_offset_v = has_align_offset<T>::value;

} // namespace concepts

} // namespace core

} // namespace foc
