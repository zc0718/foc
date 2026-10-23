# fcpp Skills System（技能体系）

> 15 task skills + 1 routing agent, all under the canonical `.github/skills/` tree. DSH, Cursor,
> Gemini CLI and Copilot (VS 2026) read the default `.agents/skills` view; Claude Code and Codex are
> one optional command away. No root-level `AGENTS.md`/`CLAUDE.md`/`GEMINI.md` files. 15 个任务技能 +
> 1 个路由 Agent；canonical 在 `.github/skills/`，默认只生成一个 `.agents/` 目录。

## Canonical & Views（唯一正文与生成视图）

| Path | Kind | Read by |
|---|---|---|
| `.github/skills/` | **canonical, committed** | Copilot, and everyone via direct read |
| `.agents/skills` | link → `.github/skills` (git-ignored) | DSH, Cursor, Gemini CLI, Copilot (VS 2026) || `.claude/skills` | optional link → `.github/skills` (git-ignored) | Claude Code |
| `.codex/skills` | optional link → `.github/skills` (git-ignored) | Codex |

> **One body, many views.** Never copy a `SKILL.md` into a view and never edit a view. Edit
> `.github/skills` and run `python .github/skills/_shared/skillkit/cli.py sync`. 只改 canonical，视图由工具生成。

## Directory（目录结构）

```text
.github/skills/
├── README.md                      # this index（本索引）
├── manifest.json                  # registry（注册清单，机器可读）
├── _shared/                       # shared fact sources（共享事实源，引用不复制）
│   ├── routing.md                 # the ONLY complete routing table（唯一完整路由表）
│   ├── harness-adapters.md        # per-harness invocation matrix（跨 harness 矩阵）
│   ├── skillkit/                  # validate / sync / catalog tool（校验与视图生成）
│   ├── gitmoji.md                 # emoji trigger cheat sheet（触发速查）
│   ├── metadata-contract.md       # metadata.json contract（契约）
│   ├── code-conventions.md        # C/C++ code conventions（代码规范）
│   └── test-conventions.md        # testing conventions（测试规范）
├── het-agent/                     # A1 routing agent（唯一路由主体）
├── het-guide/                     # S0 onboarding（导航）
├── het-build/                     # S1 build/test/coverage
├── het-release/                   # S2 release
├── het-docs/                      # S3 docs
├── het-quality/                   # S4 quality/security
├── het-board/                     # S5 cross-compile/board
├── het-fix-ci/                    # S6 troubleshooting
├── het-deps/                      # N1 dependency governance
├── het-module/                    # N2 add module/API
├── het-setup/                     # N3 environment setup
├── het-testgen/                   # N4 test generation
├── het-commit/                    # N5 conventional commit
├── het-audit/                     # N6 repository audit
├── het-preflight/                 # N7 pre-release gate
└── het-patent/                    # N8 patent mining/disclosure
```

## Invocation（如何调用）

| Harness | Load by name | Slash / gesture | Discovery |
|---|---|---|---|
| DSH | `skill` tool with `het-build` | `/het-build` | `.agents/skills` |
| Claude Code | skill name | `/het-build` | `.claude/skills` (optional view) |
| Codex | `/use het-build` | — | `.codex/skills` (optional view) |
| Cursor | skill name | `/het-build` or `@het-build` | `.agents/skills` |
| Copilot | — | `/het-build` | `.github/skills` |
| Gemini CLI | `activate_skill` | — | `.agents/skills` alias |
| Generic | read `.github/skills/<name>/SKILL.md` | — | direct read / `catalog` |

Full matrix and sources: [`_shared/harness-adapters.md`](_shared/harness-adapters.md).

## Skill Index（技能索引）

> Source of truth for ids/families/intents/dependencies: `manifest.json`. This table is a human index.

| `/` command | Skill | Family | What it does |
|---|---|---|---|
| `/het-guide` | S0 | iac_usage | onboarding + trigger cheat sheet; routes to `het-agent` |
| `/het-build` | S1 | iac_usage | build / test / coverage |
| `/het-release` | S2 | iac_usage | release a version |
| `/het-docs` | S3 | iac_usage | docs generation |
| `/het-quality` | S4 | iac_usage | quality / security gates |
| `/het-board` | S5 | iac_usage | cross-compile / board |
| `/het-fix-ci` | S6 | iac_usage | CI troubleshooting |
| `/het-deps` | N1 | dev_automation | dependency governance |
| `/het-module` | N2 | dev_automation | add module / API |
| `/het-setup` | N3 | dev_automation | environment setup |
| `/het-testgen` | N4 | dev_automation | test generation |
| `/het-commit` | N5 | dev_automation | auto conventional commit |
| `/het-audit` | N6 | dev_automation | repository audit |
| `/het-preflight` | N7 | dev_automation | pre-release gate |
| `/het-patent` | N8 | dev_automation | patent mining / disclosure drafting |
| `/het-agent` | A1 | agent | autonomous routing agent（唯一路由主体） |

## Sync & Validate（生成与校验）

```bash
# default: one root directory, .agents/ (DSH / Cursor / Gemini / Copilot VS)
python .github/skills/_shared/skillkit/cli.py sync

# optional harness views
python .github/skills/_shared/skillkit/cli.py sync --harness claude
python .github/skills/_shared/skillkit/cli.py sync --harness codex
python .github/skills/_shared/skillkit/cli.py sync --harness all

python .github/skills/_shared/skillkit/cli.py sync --check
python .github/skills/_shared/skillkit/cli.py validate --strict
python .github/skills/_shared/skillkit/cli.py catalog --format markdown
```

## Usage Conventions（使用约定）

1. **Confirm writes（写操作需确认）**: read-only by default; file writes / push / release / delete need a plan + confirmation first. 默认只读，写操作先确认。
2. **Reference `_shared`, don't copy（引用 `_shared` 不复制）**: all fact standards come from `_shared/` to avoid drift. 避免口径漂移。
3. **Route first（路由优先）**: `het-agent` (A1) is the only router; `het-guide` (S0) is onboarding. A1 优先，S0 导航。
4. **Frontmatter `description`** is the discovery surface — bilingual, keyword-rich. description 用中英关键词，保证意图命中。
5. **Single agent body（唯一 Agent 主体）**: only `.github/skills/het-agent/SKILL.md`; no second real `het-agent/SKILL.md` anywhere. 禁止第二份 Agent 实体。
6. **Root hygiene（根目录最小化）**: no root-level agent entry files; only the generated `.agents/` directory by default. 根目录不新增 agent 文件。

## Adding a Skill（新增技能）

1. Create `.github/skills/<name>/SKILL.md` with `name`, `description`, `license`, `user-invocable`.
2. Add it to `manifest.json` (`id`, `family`, `intents`, `dependencies`, `read_only`) and to
   `_shared/routing.md`.
3. Run `python .github/skills/_shared/skillkit/cli.py sync` and `validate --strict`.
4. Do **not** copy it into `.agents/skills`, `.claude/skills` or `.codex/skills`; the views are links.
