---
name: het-audit
description: 'Repository audit for the fcpp template (engineering standards + functional review vs plan/tests). Use when: audit / review / check template conformance / compare plan vs implementation. 仓库审计：按模板工程标准与功能标准输出差距报告。可用 /project-audit。'
license: Apache-2.0
argument-hint: "Audit scope (optional) / 审计范围（可选）"
user-invocable: true
---

# N6 · het-audit — Repository Audit（仓库审计 /project-audit）

> For developers. Audits against the fcpp template's private standards; read-only by default. 以模板私有标准为基准，默认只读。

## Two Audit Axes（两个审计轴）

### Axis 1: engineering standards（轴 1：工程标准，以模板定义为准）

| Check（检查项） | Baseline（基准） | Verdict |
|------|------|------|
| metadata.json consistency | single source of truth, all consumers agree | pass/violate |
| 4-bucket dep semantics | GTest only in test_package; pybind11 gated; one package per bucket | pass/violate |
| baremetal_white_list | deps & requirements filter consistently, lowercase-normalized | pass/violate |
| module annotations | `@exporter`/`@attacher`, ImportStart/End markers | present/missing |
| paired naming | `.h`↔`.c`, `.hpp`↔`.cpp` one-to-one | pass/violate |
| comment spec | `[en]`/`[zh]`, `@since` | pass/missing |
| 2 blank lines | module generation splits on this | pass/violate |
| format/static | clang-format, clang-tidy WarningsAsErrors | pass/violate |
| commit history | Conventional Commits + canonical `type(:emoji:): description` | pass/violate |
| versioning | semantic-release only; no commit-base-versioning residue | pass/residue |

### Axis 2: functional standards（轴 2：功能标准，以计划/测试为目标）

**Basis（基准）**: plan doc (PRD/PlantUML/design) or the test suite (tests as target spec, e.g. `test_package/test/unit/`).
**Evidence（证据）**: actual implementation in `include/` + `src/`.

Judgments（逐项判定）: `implemented（已实现）` / `partial（部分）` / `missing（缺失）` / `mismatch（不一致）`.

Additional findings（附加发现）: public APIs without test coverage (→ suggest `het-testgen`), dead code, deviation from `.github/misc/Codegen-Starter.txt`.

## Output（输出格式）

**Report location（报告位置）**: all reports go under `workspace/` at repo root — if `workspace/` does not exist, create it first, then write inside. 报告统一放根目录 `workspace/` 下；若无该目录，先创建再写入。

1. **Markdown report** (default): `workspace/AUDIT-<date>.md` — scope, axis 1 (✅/⚠️/❌ + evidence file:line), axis 2 (feature | contract | implementation | verdict), prioritized suggestions. 默认 Markdown 报告。
2. **Optional JSON**: `workspace/AUDIT-<date>.json` for CI. 可选 JSON。
3. Issues map to the 12 repo labels (e.g. `:recycle: refactor`, `:test_tube: tests`). 问题可挂 labels。

## Workflow（工作流）

1. Confirm scope: whole repo / module / plan doc. 确认范围。
2. Read fact sources: `.github/skills/_shared/*.md`. 读取事实源。
3. Collect evidence: `include/`, `src/`, `test_package/`, `metadata.json`, git log. 收集证据。
4. Judge per axis. 逐轴判定。
5. Output report: check for `workspace/` at repo root — create it if missing (`mkdir -p workspace/` on POSIX,
   `New-Item -ItemType Directory -Force workspace` in PowerShell, or the editor's new-folder action), then write
   `AUDIT-<date>.md` inside it (source code untouched; only the report file is created). 输出报告：检查根目录 `workspace/`，不存在则先创建，再将 `AUDIT-<date>.md` 写入其中（不改动源码，仅生成报告文件）。
6. If user wants fixes/tests → then invoke `het-testgen` / other skills (write ops need confirmation). 需要补测/修复时再进入写操作。

## Checklist（自检清单）

- [ ] Baselines referenced from `_shared/`, not memory. 基准引用 _shared。
- [ ] Every conclusion has evidence (file:line or test name). 每条结论带证据。
- [ ] Engineering vs functional issues separated. 两类问题分开。
- [ ] Actionable suggestions with priority. 可执行建议 + 优先级。

