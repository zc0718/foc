# 语言子集合规摘要

本文件由 `tools/compliance/subset_gate.py --summary` 从 `rules.tsv` 生成，请勿手改。

## 四个计数

| 状态 | 条数 | 含义 |
|---|---|---|
| `implemented` | **117** | 有机器执行的机制，`check` 列指名它是谁 |
| `not-applicable` | **28** | 接口/表/测试替身（`delta: none`），或该语言特性被编译期关掉（`delta: vacuous`） |
| `deviation` | **4** | 有 `deviation/<rule>.md`，五要素齐全 |
| `open` | **3** | 已知缺口，只减不增（基线 3） |
| **合计** | **152** | = `rules.tsv` 的行数（这条等式由门禁强制） |

## 按来源拆开

| 来源 | implemented | not-applicable | deviation | open |
|---|---|---|---|---|
| .github/misc/.clang-format-c / -cpp | 1 | 0 | 0 | 0 |
| .github/misc/.clang-tidy | 1 | 0 | 0 | 0 |
| .github/misc/.include-guard.txt | 1 | 0 | 0 | 0 |
| .github/project/deps.json + deps_gate.py | 1 | 0 | 0 | 0 |
| .github/project/forms.json + form_gate.py | 1 | 0 | 0 | 0 |
| .github/project/mcdc.json + mcdc_gate.py | 1 | 0 | 0 | 0 |
| A10 C++ Core Guidelines（免费） | 0 | 0 | 0 | 1 |
| A11 ISO/IEC 14882:2017 (N4659，dl/ 已存) | 1 | 0 | 0 | 0 |
| A12 ISO/IEC 9899:2011 (N1570，dl/ 已存) | 1 | 0 | 0 | 0 |
| A2 SEI CERT C++（免费，wiki） | 1 | 0 | 0 | 0 |
| A4 MISRA C++:2023（dl/ 已存，2023-10） | 0 | 0 | 0 | 1 |
| A9 cppcheck 手册（dl/ 已存）+ A5（未买） | 0 | 0 | 0 | 1 |
| FOC-LANG-EXC 的推论 | 0 | 1 | 0 | 0 |
| FOC-LANG-RTTI 的推论 | 0 | 1 | 0 | 0 |
| README claim 行 L1（终态） | 0 | 12 | 0 | 0 |
| README claim 行 L1（终态，HAL 组） | 0 | 7 | 0 | 0 |
| README claim 行 L1（终态，stub 组） | 0 | 7 | 0 | 0 |
| T0-language-subset.md §四（两个小洞之一） | 1 | 0 | 0 | 0 |
| api/fmu/fmu_wrapper.c + api/fmu/smoke.py | 1 | 0 | 0 | 0 |
| clang-tidy (pinned config) | 98 | 0 | 3 | 0 |
| clang-tidy（未启用）+ src/api/bridge.cpp | 0 | 0 | 1 | 0 |
| include/foc/api/foc_types.h FOC_WARN_UNUSED + .github/project/negative_compile.py | 1 | 0 | 0 | 0 |
| include/foc/core/trace.hpp + test_trace.cpp | 1 | 0 | 0 | 0 |
| metadata.json build_cppstd + .github/project/gates.py strict | 2 | 0 | 0 | 0 |
| test_package/test/unit/test_encoder_stream.cpp | 1 | 0 | 0 | 0 |
| test_package/test/unit/test_enum_names.cpp | 1 | 0 | 0 | 0 |
| test_package/test/unit/test_frame.cpp | 1 | 0 | 0 | 0 |
| tools/compliance/subset_gate.py + deviation/ | 1 | 0 | 0 | 0 |

## 能说的话与不能说的话

能写：

- "语言基线 C++17 / C11，由 `strict` 门逐头强制；子集登记的 152 条规则里 117 条有机器执行，4 条以偏差许可提交，28 条不适用并附理由。"

不能写（在补齐之前）：

- **不能写任何 MISRA C++:2023 的 N/M/K 计数**——规则编号映射还没做（`open` 里 `misra-rule-mapping` 一行就是它）。今天能引用的是**结构**（四类计数 + 偏差五要素），不是**数值**。
- 不能写"符合 MISRA C++:2023"或任何 ASIL / SIL / Class B 等级。

## 为什么三条 open 关不掉（这是本表最有信息量的部分）

`cert-*`、`cppcoreguidelines-*`、`-Wconversion` 三条都卡在同一个地方：
**启用点在 `.github/misc/.clang-tidy`，而那个文件被 `scaffold` 门字节钉住**。
所以它们的关闭方式不是"改代码"，是"上游补丁改一行配置"。把它们记成 `open` 而不是
`deviation`，是因为我们**打算**做，只是本地做不了——这个区别决定了下一个人该去改什么。
