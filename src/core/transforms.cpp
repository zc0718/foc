// Conan::ImportStart
#include <foc/core/transforms.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file transforms.cpp
 *
 * @brief [en] Instantiates the transform templates for `float`, so both the C++ and the C path link against symbols rather than inlining their own copy.
 * @brief [zh] 为 `float` 显式实例化变换模板，使 C++ 与 C 两条路径都链接到符号，而不是各自内联一份。
 * @since 0.1.0
 */

namespace foc {

namespace core {

/**
 * @brief [en] Stable ABI symbols for the V1 precision (ADR-005).
 *
 *   Even a consumer that never instantiates the templates itself links against these float specialisations.
 *
 * @brief [zh] V1 精度的稳定 ABI 符号（ADR-005）。
 *
 *   即使消费方自己从不实例化模板，也链接到这些 float 特化。
 */
template Ab<float> clarke<float>(Uvw<float>) noexcept;

template Uvw<float> inv_clarke<float>(Ab<float>) noexcept;

template Dq<float> park<float>(Ab<float>, float) noexcept;

template Ab<float> inv_park<float>(Dq<float>, float) noexcept;

} // namespace core

} // namespace foc
