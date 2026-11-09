// Conan::ImportStart
#include <foc/core/version.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file version.cpp
 *
 * @brief [en] Implements the C++ version query by forwarding to the C string the package defines.
 * @brief [zh] C++ 版本查询的实现：转发到包所定义的 C 字符串。
 * @since 0.1.0
 */

namespace foc {

namespace core {

auto version_string() noexcept -> const char* { return foc_version_string(); }

} // namespace core

} // namespace foc
