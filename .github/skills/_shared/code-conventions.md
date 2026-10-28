# fcpp C/C++ Code Conventions（模板私有代码规范）

> Fact source: module generation in `conanfile.py`, `.github/misc/.clang-format-c`, `.github/misc/.clang-format-cpp`, `.github/misc/.clang-tidy`, `.github/misc/Codegen-Starter.txt`. CI 调用的工具配置统一放 `.github/misc/`；根目录仅保留框架自动发现必需的文件（见 `.github/CONTRIBUTING.md` 布局节）。

## File Organization（文件组织）

1. **Pairing（配对约定）**: every `.h` ↔ `.c`, `.hpp` ↔ `.cpp` one-to-one. 一一对应。
2. **Suffix roles（后缀分工）**: `.h/.c` = C, `.hpp/.cpp` = C++, `.dox` = pure docs, `.cxx` = examples. 后缀分工。
3. Public headers in `include/`, implementations in `src/`, Python bindings in `api/python_bindings.cpp`.
4. **Doc-only suffixes（文档专属后缀）**: `.dox` and `.cxx` belong to the documenting system only. They live **exclusively** under `docs/doxygen/dox/` (`mainpage.dox`, `demos/*.dox`, `demos/*.cxx`) as hand-written standalone files, and are **never** written as a docstring inside a file under `include/` or `src/` — those two trees carry `.h/.c/.hpp/.cpp` only. `mainpage`/`demo` have no `include/`-or-`src/` counterpart by design. A complete doc set is a landing page plus a tutorial; one `.dox` may pull in 0..N `.cxx`, no pairing required (`het-docs`). 文档专属后缀只放 `docs/doxygen/dox/`，`include/`、`src/` 里不得出现；完备 = 主页 + 教程。

## Module Annotations（模块生成注解）

In Doxygen comments of `include/` + `src/`:
- **`@exporter`**: export this symbol into the generated C++ module. 导出符号。
- **`@attacher`**: attach to the module (not forced export). 附加符号。
```cpp
/**
 * @brief vector add
 * @exporter
 */
void fcpp_vec_add_f32(const float*, const float*, float*, uint32_t);
```

## Module Boundary Markers（模块边界标记）

Wrap the `#include` area with `// Conan::ImportStart` / `// Conan::ImportEnd`; these are converted to `import` when generating modules. 包裹 include 区，生成模块时转为 import。
```cpp
// Conan::ImportStart
#pragma once
#include <cstdint>
// Conan::ImportEnd
```

## Layout Rules（排版规则）

1. **2 blank lines between global objects** (module generation splits on this). 全局对象间强制 2 空行。
2. Multi-line Doxygen `/** ... */` before functions/classes/variables. 声明前用多行 Doxygen。
3. C headers are auto-wrapped with `extern "C"` by the recipe — do not hand-write. C 头自动包 extern "C"。

## Comment Language Spec（注释语言规范）

- Bilingual tags: `@brief [en] English` / `@brief [zh] 中文`. 双语标签。
- Version tag: `@since <version>` → filtered by docs/build.py per version. 版本标注。
- Active languages come from `doc_languages` in metadata. 生效语言由 metadata 决定。

### Tag shape（标签形状，照 `fcpp/include/doctest.hpp`）

The shape is part of the contract, not a style preference: `docs/build.py` splits a block by
marker, so a tag in the wrong place moves text into the wrong language version.
形状是契约的一部分，不是偏好：`docs/build.py` 按标记切块，标签放错位置就会把文字送进错误的
语言版本。

1. **One tag line per language group.** `@brief [en] …` and `@brief [zh] …` are two lines. Never
   two `[en]` groups in one `@brief`, never both tags on one line.
   **每个语言组一行标签**：`[en]`/`[zh]` 各占一行；一个 `@brief` 里不得出现两个 `[en]`，
   也不得两种标签挤在一行。
2. **Bodies carry no tag.** A second paragraph under a tag line is plain indented text — its
   language is the language of the tag line above it.
   **正文不带标签**：标签行下方的第二段就是缩进正文，语言由上面的标签行决定。
3. **Every field repeats per language**: `@param x [en] …` / `@param x [zh] …`,
   `@return [en]` / `[zh]`, `@note [en]` / `[zh]`, `@tparam T [en]` / `[zh]`.
   **每个字段按语言重复**。
4. **No tag = every language.** `@since`, `@see`, `@ingroup`, `@version` appear once, untagged.
   **不带标签 = 所有语言可见**。
5. **Indentation**: a wrapped line of a tag line aligns under that field's own text; a separate
   body paragraph uses a block indent five columns in from the star.
   **缩进**：标签行的折行续行对齐到该字段自身的文字之下；独立正文段落用星号后五格的块缩进。

`.github/misc/docfmt.py` enforces all five (7th gate, `python3 .github/misc/gates.py docs`). It
refuses to write a file whose prose characters would change — run `--check` first, `--fix` behind
it, and never hand-edit what it would reflow.
形状由 `docfmt.py` 强制（第 7 道门禁）。它会拒绝写入任何正文文字发生变化的文件——先 `--check`
再 `--fix`，不要手改它会被重排的地方。

### Trailing comments are not doc output（尾部注释不是文档产出）

`///<` says **what a field is**: unit, sign, range, one clause. It is not a place for rationale,
pitfalls or cross-cutting notes — a doc that needs a second line has become a paragraph, and a
paragraph in the trailing position is where the reasoning rots into `///< because the Rs fit needs
them.`

- **One line** → keep the `///<`.
- **More than one line** → promote it to a `/** ... */` block above the declaration, tagged once per
  language like any other block. `docfmt.py` does this automatically, so `--check` fails while a
  multi-line `///<` exists anywhere.
- **Why** (design reason, pitfall, constraint that spans fields) → `@note`/`@warning` in the owning
  type's block, where the reader deciding whether to use the type will actually see it.

Measured before writing this rule: of 444 `///<` lines, 431 are field descriptions, 11 mention a
reason at all, and 2 of those reasons genuinely belong at field level ("six or more, because the
`Rs` fit needs them"). The rationale this convention is about lives in block docstrings and in
`//` comments next to the code it explains — not in `///<`.

`///<` 只回答**这个字段是什么**：单位、符号、范围，一句话。它不是放缘由、坑、跨字段关注项的地方——
需要第二行的文档已经变成一段话，而尾部的段落正是推理烂成 `///< because the Rs fit needs them.` 的地方。

- **一行** → 保留 `///<`。
- **超过一行** → 提升为该声明上方的 `/** ... */` 块，与其它块一样每语言一个标签。`docfmt.py` 自动做这件事，
  因此只要仓库里还有多行 `///<`，`--check` 就会失败。
- **为什么**（设计缘由、坑、跨字段约束）→ 写进所属类型的块里作为 `@note`/`@warning`，那里才是决定
  "要不要用这个类型"的读者会看到的地方。

写这条规则前量过：444 行 `///<` 里 431 行是字段说明，11 行提到缘由，其中 2 行的缘由确实属于字段级
（"六个以上，因为 `Rs` 拟合需要"）。本条规则针对的推理，住在块注释和紧贴代码的 `//` 里，不在 `///<` 里。


## Format / Static Checks（格式化/静态检查）

- 语言族分流（私有约定，见 `conanfile.py` syntax guide #9）：`.c`/`.h` → C（`.github/misc/.clang-format-c`，`PointerAlignment: Right`）；`.hpp`/`.cpp` → C++（`.github/misc/.clang-format-cpp`，`PointerAlignment: Left`、`Standard: c++17`）。**clang-format 内置扩展映射把 `.h` 当 C++，门禁必须显式 `--style=file:` 把 `.h` 路由到 C 配置**。120 cols; no include sort; no comment reflow; `MaxEmptyLinesToKeep: 3` 保护空行规则。
- 空行分段：全局对象间 2 空行（`\n\n\n`），模块生成 `_get_export_objects` 按 `split('\n\n')` 切分（`conanfile.py`，含 TODO）；2~3 空行均兼容，超过 3 个是违规。
- 宏命名规则由 clang-tidy `cppcoreguidelines-macro-usage` 强制（`^[A-Z][A-Z0-9]*(_[A-Z0-9]+)*$`），不在 `.clang-format` 中。
- `.clang-tidy`: `bugprone-*` + `performance-*` + `cppcoreguidelines-avoid-magic-numbers`; **WarningsAsErrors: '*'**. warning 即失败；`HeaderFilterRegex: "^(include|src)/"` 锚定仓库路径。
- Local check（本地自查）: the gate body lives in `.github/misc/gates.py` (CI / pre-commit / editor share it). 门禁实现只有一份。
  ```bash
  python3 .github/misc/gates.py format   # clang-format --dry-run --Werror, dual C/C++ configs
  python3 .github/misc/gates.py tidy     # clang-tidy, standards read from metadata.json
  ```
- header-only（`metadata.is_header=true`，或 `src/` 无 TU 时）：`tidy` 为每个公开头合成一个 TU（`.h`→C、`.hpp`→C++），顺带验证头文件自包含；`format` 不变。

## Baremetal Constraints（裸机约束）

- Target-side (main package) deps filtered by `baremetal_white_list`（默认 etl/ArduinoJson）.
- OS-dependent libs (zlib/pcre2/eigen/gtest/pybind11) filtered in baremetal cross-build. 依赖 OS 的库被过滤。
- Guard desktop-only code with `#ifndef __ARM_EABI__` (see `cpptest.cpp`). 桌面专属代码加保护。

