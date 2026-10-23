// Conan::ImportStart
#pragma once

#include <foc/version.h>
// Conan::ImportEnd

namespace foc {

/**
 * @brief [en] Core algorithms shared by both the C++ and the C API path.
 *        [zh] C++ 路径与 C API 路径共享的核心算法层。
 *
 * @since 0.1.0
 */
namespace core {

/**
 * @brief [en] Version string of the library, e.g. "0.1.0". Mirrors foc_version_string().
 *        [zh] 库的版本字符串，例如 "0.1.0"。与 foc_version_string() 一致。
 *
 * @return [en] Pointer to a static NUL-terminated buffer; never NULL, never freed by caller.
 *         [zh] 指向静态 NUL 结尾缓冲区的指针；永不为 NULL，调用方不得释放。
 * @context [en] Any context. noexcept, no allocation, ISR-safe.
 *           [zh] 任意上下文。noexcept、无内存分配、ISR 安全。
 * @since 0.1.0
 */
auto version_string() noexcept -> const char*;

} // namespace core

} // namespace foc
