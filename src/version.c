// Conan::ImportStart
#include <foc/version.h>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file version.c
 *
 * @brief [en] Defines the version string the package reports, the one the tests compare against `metadata.json`.
 * @brief [zh] 定义包对外报告的版本字符串——也就是用例与 `metadata.json` 比对的那一个。
 * @since 0.1.0
 */


static const char kVersionString[] = "0.1.0";


const char *foc_version_string(void) { return kVersionString; }
