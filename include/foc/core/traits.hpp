// Conan::ImportStart
#pragma once

#include <type_traits>
#include <utility>

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
 */
template <typename T, typename = void> struct is_pwm : std::false_type {};

/**
 * @brief [en] `T` satisfies the foc::hal::Pwm contract.
 *        [zh] `T` 满足 foc::hal::Pwm 契约。
 * @since 0.1.0
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
 */
template <typename T> inline constexpr bool is_pwm_v = is_pwm<T>::value;

template <typename T> inline constexpr bool is_current_sense_v = is_current_sense<T>::value;

template <typename T> inline constexpr bool is_position_sensor_v = is_position_sensor<T>::value;

template <typename T> inline constexpr bool is_dc_link_v = is_dc_link<T>::value;

template <typename T> inline constexpr bool is_clock_v = is_clock<T>::value;

} // namespace concepts

} // namespace hal

} // namespace foc
