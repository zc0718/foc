// Conan::ImportStart
#pragma once

#include <cmath>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] Transform constants, amplitude-invariant convention. Variable templates keep the
 *        literal at full precision for `double` users instead of truncating to `float`.
 *        [zh] 变换常量，幅值不变式约定。用变量模板是为了让 `double` 用户拿到全精度常量，
 *        而不是被截断成 `float`。
 *
 * @since 0.1.0
 */
template <typename T> inline constexpr T kPi = static_cast<T>(3.141592653589793238462643383279502884L);

template <typename T> inline constexpr T kTwoPi = static_cast<T>(2) * kPi<T>;

template <typename T> inline constexpr T kTwoThirds = static_cast<T>(2) / static_cast<T>(3);

template <typename T> inline constexpr T kInvSqrt3 = static_cast<T>(0.577350269189625764509148780501957456L);

template <typename T> inline constexpr T kSqrt3Over2 = static_cast<T>(0.866025403784438646763723170752936183L);

/**
 * @brief [en] Clarke transform: three-phase (abc) to stationary two-phase (alpha-beta).
 *        Amplitude invariant: a balanced set of amplitude A maps to a vector of amplitude A.
 *        [zh] Clarke 变换：三相 abc 到静止两相 alpha-beta。幅值不变式：幅值 A 的平衡三相映射为
 *        幅值 A 的矢量。
 *
 * @param uvw [en] phase quantities in SI units. [zh] 三相量，SI 单位。
 * @return [en] alpha-beta vector in the same unit. [zh] 同一单位的 alpha-beta 矢量。
 * @context [en] Current-loop ISR: noexcept, no allocation, no function pointer.
 *           [zh] 电流环 ISR：noexcept、不分配、无函数指针。
 * @since 0.1.0
 */
template <typename T> auto clarke(Uvw<T> uvw) noexcept -> Ab<T> {
    const T half = static_cast<T>(1) / static_cast<T>(2);

    return Ab<T>{static_cast<T>(kTwoThirds<T> * (uvw.u - half * (uvw.v + uvw.w))),
                 static_cast<T>(kInvSqrt3<T> * (uvw.v - uvw.w))};
}

/**
 * @brief [en] Inverse Clarke transform: stationary two-phase back to three-phase.
 *        [zh] 逆 Clarke 变换：静止两相回到三相。
 *
 * @param ab [en] alpha-beta vector. [zh] alpha-beta 矢量。
 * @return [en] balanced three-phase quantities. [zh] 平衡三相量。
 * @context [en] Current-loop ISR: noexcept, no allocation. [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 */
template <typename T> auto inv_clarke(Ab<T> ab) noexcept -> Uvw<T> {
    const T half = static_cast<T>(1) / static_cast<T>(2);

    return Uvw<T>{ab.alpha, static_cast<T>(-half * ab.alpha + kSqrt3Over2<T> * ab.beta),
                  static_cast<T>(-half * ab.alpha - kSqrt3Over2<T> * ab.beta)};
}

/**
 * @brief [en] Park transform: stationary alpha-beta to rotating d-q (d = flux, q = torque).
 *        [zh] Park 变换：静止 alpha-beta 到旋转 d-q（d = 磁链轴，q = 转矩轴）。
 *
 * @param ab [en] alpha-beta vector. [zh] alpha-beta 矢量。
 * @param theta_rad [en] electrical angle in rad. [zh] 电角度，单位 rad。
 * @return [en] d-q vector. [zh] d-q 矢量。
 * @context [en] Current-loop ISR: noexcept, no allocation. [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 */
template <typename T> auto park(Ab<T> ab, T theta_rad) noexcept -> Dq<T> {
    const T c = std::cos(theta_rad);
    const T s = std::sin(theta_rad);

    return Dq<T>{static_cast<T>(ab.alpha * c + ab.beta * s), static_cast<T>(-ab.alpha * s + ab.beta * c)};
}

/**
 * @brief [en] Inverse Park transform: rotating d-q back to stationary alpha-beta.
 *        [zh] 逆 Park 变换：旋转 d-q 回到静止 alpha-beta。
 *
 * @param dq [en] d-q vector. [zh] d-q 矢量。
 * @param theta_rad [en] electrical angle in rad. [zh] 电角度，单位 rad。
 * @return [en] alpha-beta vector. [zh] alpha-beta 矢量。
 * @context [en] Current-loop ISR: noexcept, no allocation. [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 */
template <typename T> auto inv_park(Dq<T> dq, T theta_rad) noexcept -> Ab<T> {
    const T c = std::cos(theta_rad);
    const T s = std::sin(theta_rad);

    return Ab<T>{static_cast<T>(dq.d * c - dq.q * s), static_cast<T>(dq.d * s + dq.q * c)};
}

} // namespace core

} // namespace foc
