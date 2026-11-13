#pragma once

// The C surface as a C++ translation unit sees it in this tree.
//
// `foc/api/foc_api.h` carries no `extern "C"` guards of its own. The *installed*
// copy does, because `conanfile.py::_make_c_compatible()` rewrites every `.h` in
// the package and wraps it. So a case built against the package has always linked
// against `foc_api.c`, while the same case built against the source tree did not:
// its references were mangled, and every `foc_*` call was an undefined symbol
// (`mcdc` measured 237 of them, which is why these cases were excluded from the
// gate instead of fixed).
//
// Wrapping the include is the source-tree half of that same step, and it nests
// inside the installed header's own guard -- which is legal -- so one file cannot
// disagree with the other. A case that only *names* the C types does not need
// this header; one that *calls* the ABI does.
//
// 装机包里每个 `.h` 都被 `_make_c_compatible()` 包进 `extern "C"`，源码树里没有。
// 于是对着 **包** 编的用例一直链得上 `foc_api.c`，对着 **树** 编的同一条用例链不上：
// 引用是修饰名，每个 `foc_*` 调用都是未定义符号（`mcdc` 量到 237 个，这就是这些用例
// 当初被排除而不是被修好的原因）。把 include 包起来是那同一步的源码树版本，而且对装机
// 头是嵌套的（合法），所以两边不会各说各话。只 **用到** C 类型的用例不需要它；**调用**
// ABI 的才需要。
extern "C" {
#include <foc/api/foc_api.h>
}
