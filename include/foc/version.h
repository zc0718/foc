// Conan::ImportStart
#pragma once
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file version.h
 *
 * @brief [en] The version string the package reports, as a query a C caller can make.
 * @brief [zh] 包所报告的版本字符串，以 C 调用方可发起的查询形式给出。
 * @since 0.1.0
 */


/**
 * @brief [en] Version string of the foc library, e.g. "0.1.0". Kept in sync with metadata.json.
 * @brief [zh] foc 库的版本字符串，例如 "0.1.0"。与 metadata.json 保持一致。
 * @return [en] Pointer to a static NUL-terminated buffer. Never NULL; the caller must not free it.
 * @return [zh] 指向静态 NUL 结尾缓冲区的指针。永不为 NULL；调用方不得释放。
 * @context [en] Any context. noexcept, reentrant, no allocation, ISR-safe.
 * @context [zh] 任意上下文。noexcept、可重入、无内存分配、ISR 安全。
 * @pre [en] None.
 * @pre [zh] 无。
 * @since 0.1.0
 * @exporter
 */
const char *foc_version_string(void);
