// Conan::ImportStart
#include <foc/core/transforms.hpp>
// Conan::ImportEnd

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
