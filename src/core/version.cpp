// Conan::ImportStart
#include <foc/core/version.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

auto version_string() noexcept -> const char* { return foc_version_string(); }

} // namespace core

} // namespace foc
