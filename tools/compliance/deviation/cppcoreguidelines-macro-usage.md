# 偏差许可 `cppcoreguidelines-macro-usage`

| | |
|---|---|
| **规则** | `cppcoreguidelines-macro-usage`（clang-tidy，由 `.github/misc/.clang-tidy` 显式启用，`WarningsAsErrors: '*'`） |
| **压制位置** | `include/foc/api/foc_types.h` 共 **2 处**（`FOC_HAS_ATTRIBUTE` 的两个分支） |
| **状态** | 已登记（`rules.tsv` 里这一行是 `deviation`） |

## 为什么

这条检查的建议是"函数式宏改用 `constexpr` 模板函数"。**在 C 里这个建议不可实现**：
`foc_types.h` 必须被 C11 与 C++17 同时编过（`strict` 门两条腿都跑），而 `constexpr` 与模板
都不在 C 里。

更具体的一条：被压制的宏是 `__has_attribute(x)` 的**探测**包装。

```c
#if defined(__has_attribute)
#define FOC_HAS_ATTRIBUTE(x) __has_attribute(x)
#else
#define FOC_HAS_ATTRIBUTE(x) 0
#endif
```

它必须是函数式宏——`__has_attribute` 要一个属性名作实参。改成对象式宏（`#define FOC_HAS_ATTRIBUTE 1`）
表达不了"用哪个属性"，而这正是它存在的全部意义。

## 影响

- **运行时**：无。两个分支在预处理期就选定，各自展开成 `__has_attribute(x)` 或常量 `0`。
- **接口**：无。`FOC_HAS_ATTRIBUTE` 不在发布 ABI 里（它不产生符号），只是头文件内部的探测手段。
- **可读性**：正相反——把它换成别的形式会让"这个编译器认识该属性吗"这个问题变得隐晦。

## 替代控制

1. **它的效果有测试面**：`FOC_WARN_UNUSED` 展开成什么，由负例集的一条真编译用例钉住
   （`case:c_abi_dropped_error_code`：忽略 `foc_err_t` 必须编译失败）。也就是说，这个宏
   **不是"写了没验"**——它失败的样子被单独测过。
2. **两处都压，不留一半**：`#else` 分支同样压制，因为在不认识 `__has_attribute` 的编译器上
   它同样是个函数式宏；只压一处会让"换个编译器就红"，而那正是本库最不想留下的性质。
3. **宏的数量被封顶**：`foc_types.h` 的注释里写着"两个宏，不是一个可移植性库"，
   重开条件（需要写第四个宏且非按编译器分叉不可）也写在同一处。
4. 本偏差的**存在性**由 `subset_gate` 强制：这 2 处 `NOLINT` 缺本文件即红。

## 批准人

`zc`（项目 owner）。

## 复查日期

**2027-05-06**。

**提前复查的触发条件**：① 项目放弃 C 侧支持（届时可改成 `constexpr`/concepts）；
② 需要写第 4 个属性宏（按 `foc_types.h` 已写明的那条重开条件处理）；
③ clang-tidy 增加"仅 C++ 生效"的过滤选项，可以在 C 翻译单元里自然跳过这条检查。
