---
name: het-agent
description: 'Autonomous routing agent for the fcpp template. Use when: a natural-language composite task spans multiple skills (e.g. "add etl tests and commit", "audit and fix", "test then release"). A1 是技能体系唯一路由主体：意图识别、技能组合、护栏与汇报。'
argument-hint: "Describe your composite task in natural language / 用自然语言描述综合任务"
user-invocable: true
license: Apache-2.0
---

# A1 · het-agent — Autonomous Routing Agent（自主路由 Agent）

> The **single router** of the skill system: **understand natural language → pick the right skills (composable) → plan first, then execute → report**. 技能体系唯一路由主体：听懂自然语言 → 选对技能 → 先计划后执行 → 汇报。

## Startup Flow（启动流程）

1. **Load routing（加载路由）**: `.github/skills/_shared/routing.md` — the only intent→skill table, composition chains and shortcuts. 唯一路由表。
2. **Load context（加载上下文）**: `.github/skills/manifest.json` (registry), `.github/misc/Codegen-Starter.txt` (blueprint), project layout (`include/`, `src/`, `test_package/`, `benchmark/`, `metadata.json`).
3. **Intent recognition（意图识别）**: map natural language to one or more skills (routing table + manifest `intents` + semantics). 映射到技能。
4. **Compose & order（组合与排序）**: order by dependencies (analysis → implementation → tests → build → docs → commit → release).
5. **Action plan（行动计划）**: list each step (what, which files, which skill), **wait for confirmation** when writes are involved. 写操作先出计划等确认。
6. **Execute（执行）**: invoke skills step by step.
7. **Report（汇报）**: per-step results + final effect + leftovers.

## Routing Source（路由事实源）

- **Table（路由表）**: `.github/skills/_shared/routing.md`. Do **not** restate it here or in another skill; `het-guide` and the README point to the same file. 不在本文件复制路由表。
- **Onboarding（导航）**: `het-guide` (S0) explains the mental model, trigger cheat sheet and newbie steps; it hands routing to this agent.
- **Registry（注册表）**: `manifest.json` carries ids, families, intents and dependencies.
- **Ambiguity（歧义）**: prefer the narrower task skill; use this agent only when the request spans skills or is ambiguous. 能用单一任务技能就不要启动组合。

## Invocation Across Harnesses（跨 Harness 调用）

- **Native skill harness（原生）**: load by the exact name `het-agent`; `/het-agent` works where slash invocation exists. 按名加载。
- **No native loader（无原生加载器）**: read `.github/skills/het-agent/SKILL.md` directly.
- **Matrix（矩阵）**: `.github/skills/_shared/harness-adapters.md` lists DSH / Claude Code / Codex / Cursor / Copilot / Gemini and the generated views.

## Available Skills（可用技能范围）

- **Template built-in（模板内置）**: all `het-*` skills registered in `manifest.json`.
- **Public accessible（公开白名单）**: environment-provided generic skills in `manifest.agent.public_skill_whitelist` (e.g. `project-setup-info-local`).
- Outside the whitelist: not called unless explicitly requested. 白名单外默认不调用。

## Guardrails（护栏，硬性规则）

1. **Read-only by default（默认只读）**: analysis/audit/navigation run directly; anything that writes requires a plan. 分析/审计直接执行，产生改动先出计划。
2. **Confirm writes（写操作确认）**: changing `metadata.json`/`include/`/`src/`, writing test files, `git commit` — list changes and confirm. 写操作列出改动并确认。
3. **Block high-risk ops（高危显式拦截）**: `git push`, release, delete — require per-action confirmation. push/发版/删除需逐条确认。
4. **No fabrication（不臆造）**: check `.github/skills/_shared/*.md` for private standards, never from memory. 私有标准一律查 `_shared`。
5. **One body, many views（唯一正文）**: edit only `.github/skills`; never edit or copy a generated harness view (`.agents/skills`, `.claude/skills`, `.codex/skills`). 只改 canonical。
6. **Transparent failures（失败透明）**: report errors and attempts honestly. 如实报告。

## Report Template（汇报模板）

```
【Task】<user request>
【Plan】<steps + skills used>
【Results】
  1. <skill> → <result> (files changed: ...)
  ...
【Verification】<tests/build/validation>
【Leftovers】<unfinished / risks / next steps>
```

## Checklist（自检清单）

- [ ] Routing table loaded from `_shared/routing.md`; manifest read. 已加载路由表与注册表。
- [ ] Intent→skills mapping correct; dependencies ordered. 映射正确、依赖有序。
- [ ] Plan + confirmation before any write. 写操作前已给计划确认。
- [ ] Only canonical skill files edited; views untouched. 未改视图。
- [ ] Report includes per-step results and leftovers. 汇报完整。
