# 偏差许可 `performance-enum-size`

| | |
|---|---|
| **规则** | `performance-enum-size`（clang-tidy，由 `.github/misc/.clang-tidy` 的 `performance-*` 启用） |
| **压制位置** | `include/foc/api/foc_types.h` 共 **10 处**（`:77` `:130` `:226` 等，逐处由 gen_subset 扫描得到） |
| **状态** | 已登记（`rules.tsv` 里这一行是 `deviation`） |

## 为什么

C 的 `enum` **没有固定底层类型**（C23 之前）。`foc_err_t`、`foc_motor_state_t`、`foc_fault_t`
这些类型的宽度由编译器按取值域决定，写 `enum : std::uint8_t` 是 C++ 的写法，在 C 里根本不成立——
而这个头必须同时被 C11 和 C++17 编过（`strict` 门两条腿都跑）。

更硬的一条理由：**这些数值是已冻结的 C ABI**。`foc::ErrorCode`、`foc::MotorState`、`foc::FaultCode`
与它们一一对应，改底层类型会改 `sizeof`，那就是 ABI 变更。`performance-enum-size` 的建议
（"把只用到 3–4 个值的枚举收窄成 `uint8_t`"）在这里**与冻结承诺直接冲突**，而承诺优先。

## 影响

- **运行时**：无。这条检查是"内存布局建议"，不是正确性检查；不改布局就没有风险面。
- **接口**：正相反——**遵守**这条建议才会引入 ABI 风险。
- **可移植性**：无。C 侧本来就不能指定底层类型。

## 替代控制

1. **取值域完整性由用例钉住**：`test_package/test/unit/test_enum_names.cpp` 把 `0..last`
   每个值走一遍（每个值都有自己的名字、域外回 `"?"`），并断言**每个 C 宏等于它镜像的 C++ 枚举值**。
   布局建议换不来这条证据，它换来的是"忘写一个值时不会静默"。
2. **ABI 稳定性由测试面**：`test_package/c_api_validation.c` 与 `test_api_c_smoke.cpp` 覆盖
   C 面的全部入口；任何让 `sizeof(foc_err_t)` 变化的改动都会在那里先炸。
3. 本偏差的**存在性**由 `subset_gate` 的两条腿强制：每处 `NOLINT` 必须有本文件（反之亦然），
   且本文件必须含五要素标题，否则门禁红。

## 批准人

`zc`（项目 owner）。批准依据：C 语言约束 + 已冻结 ABI，二者都不是"风格偏好"。

## 复查日期

**2027-05-06**（或更早，触发条件见下）。

**提前复查的触发条件**：① 项目把 C 面升到 C23（届时 `enum : uint8_t` 在 C 侧可表达）；
② 客户要求 `foc_err_t` 的宽度可配置；③ `foc_types.h` 的枚举数量或取值域变化到需要重新论证。
