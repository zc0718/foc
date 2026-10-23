---
name: het-guide
description: 'Onboarding & routing entry for the fcpp template. Use when: new users ask what this project can do automatically, where to start, how CI triggers work, or which skill to use. 新手导航/入口：项目能自动做什么、从哪开始、CI 怎么触发、该用哪个技能。'
argument-hint: "Tell me what you want to do / 告诉我你想做什么"
user-invocable: true
license: Apache-2.0
---

# S0 · het-guide — IaC Commander / Onboarding（指挥官/入口）

> Plain language first, file paths second. 面向 CI/CD 新手：先大白话，再给路径。

## Mental Model（心智模型）

> "This is a C/C++ library template. Push code → the system reads the `metadata.json` switches and auto-runs build / test / docs / release. Put an emoji in the commit message to request a pipeline. You don't need to know workflow / orchestrator terms."
> 这是一个 C/C++ 库模板：提交代码后按 `metadata.json` 开关自动跑构建/测试/文档/发版；提交信息带 emoji 即可点名触发。

## Routing（路由：只指向唯一事实源）

- **Intent → skill table（意图→技能表）**: `.github/skills/_shared/routing.md` — the only complete routing table. 唯一完整路由表。
- **Composite tasks（综合任务）**: load `het-agent` (A1), the single router. 组合任务交给 `het-agent`。
- **Quick route（速记）**: build → `het-build`; release → `het-release`; docs → `het-docs`; quality → `het-quality`; board → `het-board`; CI red → `het-fix-ci`; dev tasks → `het-deps` / `het-module` / `het-setup` / `het-testgen` / `het-commit` / `het-audit` / `het-preflight` / `het-patent`.
- Do not add a second routing table here. 本文件不再维护第二份路由表。

## Trigger Cheat Sheet（触发速查表）

| I want（我想） | Put in commit message（提交信息里写） | It runs（它会） |
|------|------------|------|
| Build（构建） | `feat(:building_construction:): ...` | Build matrix |
| Tests（测试） | `test(:beer:): ...` | GTest + coverage |
| Release（发版） | `chore(:package:): ...` | Auto release + version bump |
| Docs（文档） | `docs(:book:): ...` | Bilingual docs |
| Quality / security（质量/安全） | `ci(:shield:): ...` | Native gates (format/tidy/gitleaks) + MegaLinter SAST — both block |
| Board（上板） | `feat(:fire:): ...` | Cross-compile + board transfer |

> Canonical form: `type(:emoji:): description`. Prerequisite: the matching `workflow_triggers.*` must be `true` in `metadata.json`. 规范格式 `type(:emoji:): 描述`；前提是对应开关为 true（模板默认 build/tests/docs/security_scan/cross_compile 已开启，release 需自行打开；cross_compile 可单独用 `:hammer_and_wrench:` 触发）。

## Newbie 3 Steps（新手三步走：从零到发版）

1. Open `metadata.json`, adjust `workflow_triggers.*` (build/tests/docs/security/cross_compile are on by default; enable release when needed). 按需调整开关（build/tests/docs/security 默认开启）。
2. Write code, commit with `feat(:building_construction:): description`. 用规范格式提交。
3. Check the GitHub **Actions** page; artifacts are in the bottom **Artifacts** area. 看 Actions 页，产物在 Artifacts。

## Invocation Across Harnesses（跨 Harness 调用）

- Load this skill by the exact name `het-guide`; `/het-guide` works where the harness supports slash invocation. 按名加载；支持 `/` 时也可 `/het-guide`。
- Harness matrix and generated views: `.github/skills/_shared/harness-adapters.md`. 各 harness 见适配矩阵。
- No native loader: read `.github/skills/het-guide/SKILL.md` directly. 无原生加载器直接读规范路径。

## Usage Principles（使用原则）

1. Read-only: this skill only navigates and explains. 只读导航，不做修改。
2. Explain what will happen before giving steps. 先讲清会发生什么，再给操作。
3. Any metadata / commit / release change needs a plan and confirmation first. 涉及修改先给计划并确认。

## Related Files（关联文件）

- Routing（唯一路由表）：`.github/skills/_shared/routing.md`
- Harness adapters（跨 harness 矩阵）：`.github/skills/_shared/harness-adapters.md`
- Fact sources（事实源）：`.github/skills/_shared/gitmoji.md`、`.github/skills/_shared/metadata-contract.md`
- Generation blueprint（生成蓝图）：`.github/misc/Codegen-Starter.txt`
- CI: `.github/workflows/ci-orchestrator.yml`、`metadata-controller.yml`
