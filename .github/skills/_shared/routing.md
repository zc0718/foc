# Routing Table（唯一路由表：意图 → 技能）

> **Single routing authority.** This file is the only complete routing table in the repository. `het-agent`
> (A1) is the only router agent; `het-guide` (S0) is onboarding and points here. Do not copy this table
> into another skill or entry file. 本文件是唯一路由表；`het-agent` 是唯一路由 Agent，`het-guide` 只做导航并指向本表。
>
> Machine-readable registry: `.github/skills/manifest.json`. 机器可读注册表见 manifest。

## Intent → Skill（意图 → 技能）

| User intent（用户意图） | Skill | What it does（作用） |
|---|---|---|
| Build / compile / test / coverage（构建/编译/测试/覆盖率） | `het-build` (S1) | Build, test, coverage |
| Release / new version / changelog（发布/新版本） | `het-release` (S2) | Release a new version |
| Docs / bilingual docs（文档/中英文文档） | `het-docs` (S3) | Auto docs |
| Quality / security / lint（质量/安全） | `het-quality` (S4) | Pass quality gates |
| Board / cross-compile / baremetal（上板/交叉编译/裸机） | `het-board` (S5) | Cross-compile & run on board |
| CI red / errors（CI 红了/报错） | `het-fix-ci` (S6) | Troubleshoot |
| Deps（依赖） | `het-deps` (N1) | Dependency governance |
| New module / API（新增模块/API） | `het-module` (N2) | Add a module/API |
| Environment / first build（环境搭建/首次构建） | `het-setup` (N3) | Environment setup |
| Generate tests（测试生成） | `het-testgen` (N4) | Test case generation |
| Commit（规范提交） | `het-commit` (N5) | Auto Conventional Commit |
| Audit（审计） | `het-audit` (N6) | Repository audit |
| Pre-release gate（发版门禁） | `het-preflight` (N7) | Pre-release gate |
| Patent mining（专利挖掘） | `het-patent` (N8) | Patent disclosure drafting |
| Onboarding / navigation（新手导航） | `het-guide` (S0) | Onboarding entry |
| Composite / ambiguous task（综合任务） | `het-agent` (A1) | The single router |

## Composition Examples（技能组合示例）

| User request（用户请求） | Chain（组合链） |
|---|---|
| "给 etl 模块补测试并提交" | `het-testgen` (N4) → `het-commit` (N5) |
| "审计仓库并把规范问题修掉" | `het-audit` (N6) → `het-commit` (N5) |
| "补测试 → 跑覆盖率 → 出文档 → 提交" | `het-testgen` (N4) → `het-build` (S1) → `het-docs` (S3) → `het-commit` (N5) |
| "新增模块并补测试" | `het-module` (N2) → `het-testgen` (N4) |
| "发版前全检查一遍" | `het-preflight` (N7) |
| "挖掘专利点并生成交底书" | `het-patent` (N8) |
| "上板前先审计裸机依赖" | `het-audit` (N6) → `het-board` (S5) |

Order by dependency: analysis → implementation → tests → build → docs → commit → release. 按依赖排序。

## Intent Shortcuts（意图快捷词）

| Phrase（说法） | Means（含义） | Where（去向） |
|---|---|---|
| 全量 / 全量测试 / 走一遍全量 / full sweep | every online pipeline except release (build + tests + docs + security + cross-compile) | commit form in `_shared/gitmoji.md`; if red → `het-fix-ci` |
| 只构建 / build only | the build pipeline alone | `het-build` (S1) |
| 发版 / release / 打 tag | semantic-release, needs `(:package:):` | `het-release` (S2) |
| 上板 / board / 烧录 | the self-hosted `:fire:` route | `het-board` (S5) |
| 覆盖率 / coverage | rates + the two-leg reconciliation | `het-build` (S1) |

A "全量" request is **not** a release request: never add `(:package:):` to satisfy it. 全量不含发版。

## Available Skills（可用技能范围）

- **Template built-in（模板内置）**: every `het-*` skill listed above, registered in `manifest.json`.
- **Public accessible（公开白名单）**: environment-provided generic skills named in
  `manifest.agent.public_skill_whitelist` (e.g. `project-setup-info-local`).
- Outside the whitelist: do not call unless the user explicitly asks. 白名单外默认不调用。

## Invocation（怎么加载）

- Native skill harness: load by **name** (`het-build`), or `/het-build` where the harness supports slash
  invocation. 原生技能 harness 按名加载；支持 `/` 的可用 `/het-build`。
- No native loader: read `.github/skills/<name>/SKILL.md` directly. 无原生加载器时直接读规范路径。
- Per-harness matrix: [`harness-adapters.md`](./harness-adapters.md). 各 harness 见适配矩阵。
