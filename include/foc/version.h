// Conan::ImportStart
#pragma once
// Conan::ImportEnd


/**
 * @brief [en] Version string of the foc library, e.g. "0.1.0". Kept in sync with metadata.json.
 *        [zh] foc 库的版本字符串，例如 "0.1.0"。与 metadata.json 保持一致。
 *
 * @return [en] Pointer to a static NUL-terminated buffer. Never NULL; the caller must not free it.
 *         [zh] 指向静态 NUL 结尾缓冲区的指针。永不为 NULL；调用方不得释放。
 * @context [en] Any context. noexcept, reentrant, no allocation, ISR-safe.
 *           [zh] 任意上下文。noexcept、可重入、无内存分配、ISR 安全。
 * @pre [en] None. [zh] 无。
 * @since 0.1.0
 * @exporter
 */
const char *foc_version_string(void);
