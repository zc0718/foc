> **MERGED -- do not apply.** Upstream shipped this in `fcpp` v0.3.4: `f2519753 fix(:shield:): let cppcheck finish
> its analysis instead of going red`, and `98ea19a1 fix(:shield:): cover the C cppcheck axis too` went further than
> this patch asked (it also set `C_CPPCHECK_ARGUMENTS`, which `.h`/`.c` need because they are linted by the C
> descriptor). The file below is kept as the record of what was asked for; `scaffold.json` is pinned to v0.3.4 with
> `tolerated` empty.

# 0019 · cppcheck 用 `--check-level=exhaustive`，而不是让"分析受限"变成红

**一句话**：cppcheck 2.14 遇到分支多的函数会输出 `too many branches, analysis limited`
**提示信息**（`information` 级，不是缺陷），而 MegaLinter 用 `--error-exitcode=1` 调用它 ——
于是十一个头文件把 job 变红，背后一个缺陷都没有。

- 补丁：`0019-cppcheck-exhaustive.patch`（**1 文件 / +5−0 / 1 hunk**）
- 基线：`HeT-FTI/fcpp@v0.3.3` = `6d69c85`
- 目标仓库：`HeT-FTI/fcpp`（**上游**）

```bash
git apply --check workspace/scaffold/0019-cppcheck-exhaustive.patch
```

## 1. 事实（本仓库实测）

- 上游 `.github/misc/.mega-linter.yml` 里 cppcheck 只有默认参数。
- 本仓库 `CPP_CPPCHECK_ARGUMENTS: "--check-level=exhaustive"` 加上之后，
  `megalinter` 从红转绿，**没有屏蔽任何一条规则**：cppcheck 是"改做完整分析"，
  而不是"别提这件事"。
- 这一点值得强调，因为最省事的修法是 `--suppress=checkersReport` 之类：
  它让消息消失，也让那条检查从此在**任何地方**都不再起作用 ——
  正是本模板要防的失败模式（消掉告警 ≠ 通过标准）。

## 2. 为什么该上游

这不是本项目特有的配置口味，而是**任何有头文件的派生项目都会踩**：

1. 模板默认开 MegaLinter 的 cppcheck 轴；
2. cppcheck 的启发式限额与项目代码规模无关，只与单个函数的复杂度有关；
3. `--error-exitcode=1` 把 `information` 也当失败。

因此每个派生项目要么自己发现并加这一行（我们花了时间才定位到是 `information` 级），
要么就一直看着一个红 job 并逐渐学会忽略它 —— 后者危害更大。

## 3. 证据

| 项 | 值 |
|---|---|
| 触发文件数 | 11 个头文件 |
| cppcheck 版本 | 2.14 |
| 消息级别 | `information`（非 `error` / `warning` / `style`） |
| 加参数后 | `megalinter` 绿，规则无一被 suppress |
| `scaffold` 门 | 该文件曾是 12 个 fork 之一；本补丁上游化后 fork 归零 |

## 4. 替代方案（都更差）

- **压低单函数复杂度**：治不了根，新写一个长 `if` 链又回来，且把工具限额
  变成了代码风格约束。
- **`--suppress`**：让真缺陷也一起消失。
- **关掉 `CPP_CPPCHECK`**：整轴丢失。
- **在派生项目里单独维护**：就是本仓库此前的状态 —— 一个 4 行的 fork 要跟着
  上游每次发版复核，成本远高于收益。

## 5. 保护点

1. 允许 `information` 级消息通过，但**不降低** `error`/`warning`/`style` 的失败阈值。
2. 用"完成分析"而不是"抑制报告"消除该消息。
