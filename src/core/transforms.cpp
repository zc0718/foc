// Conan::ImportStart
#include <foc/core/transforms.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

// Stable ABI symbols for the V1 precision (ADR-005): even a consumer that never instantiates the
// templates itself links against these float specialisations.
template Ab<float> clarke<float>(Uvw<float>) noexcept;

template Uvw<float> inv_clarke<float>(Ab<float>) noexcept;

template Dq<float> park<float>(Ab<float>, float) noexcept;

template Ab<float> inv_park<float>(Dq<float>, float) noexcept;

} // namespace core

} // namespace foc
