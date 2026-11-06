# 偏差许可 `cppcoreguidelines-pro-type-reinterpret-cast`

**完整论证在 `deviation/performance-no-int-to-ptr.md`。** 两份文件存在的原因与分工：

- `performance-no-int-to-ptr.md`：主论证（令牌式句柄、为什么两次转换都不涉及地址、替代控制、复查条件）。
- 本文件：让 `cppcoreguidelines-pro-type-reinterpret-cast` 这个**规则名**在偏差目录里有自己的落点，
  因为 `subset_gate` 要求"代码里每个 `NOLINT` 的规则名都有一个同名记录"。同一条 `NOLINTNEXTLINE`
  一次压制了两个规则名，所以两个名字各要一份。

## 为什么

见 `performance-no-int-to-ptr.md` 的「为什么」：`foc_motor_t` 是**令牌**而不是地址，两次转换
（签发时的 `uintptr_t -> 指针`、解析时的 `指针 -> uintptr_t`）都只在打包/拆包位域，**从不解引用**。

一条与前一份文件不同的补充事实：**这条检查今天并没有启用**（钉住的 `.clang-tidy` 只启用了两条
`cppcoreguidelines-*`，不含这一条）。所以：

- `rules.tsv` 里它的状态是 `not-applicable` / `delta: vacuous`——今天没有规则可违反；
- 但抑制**保留在代码里**，因为它是前向防护：谁哪天开启 `cppcoreguidelines-*` 全族，
  这两个转换点不应该成为"要让代码先红一次"的地方。

这正是本仓库对待"未启用但已知会冲突"的规则的一贯做法：**先写记录，再谈启用**，而不是等到红的那天
在压力下补理由。

## 影响

同 `performance-no-int-to-ptr.md`：「运行时」无、「接口」正相反（遵守建议才会引入 ABI 风险）、
「危险面」是"如果有人真的解引用令牌"。

## 替代控制

同 `performance-no-int-to-ptr.md` 的四条：唯一转换点、解引用禁令有测试面、代际计数器有覆盖、
存在性由 `subset_gate` 强制。

## 批准人

`zc`（项目 owner）。

## 复查日期

**2027-05-06**。提前复查条件同主论证；另加一条：**启用 `cppcoreguidelines-*` 全族时**，本行的
状态要从 `not-applicable` 改成 `deviation`（`gen_subset.py` 会自动这么做——它按抑制点判断）。
