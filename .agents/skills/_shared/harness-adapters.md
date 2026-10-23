# Harness Adapters（跨 Harness 调用矩阵）

> Fact source for "how does this harness see the skills?". Canonical skills are **only** in
> `.github/skills/`; every other root is a generated view. 唯一正文在 `.github/skills/`，其余根目录都是生成视图。
>
> **Root hygiene（根目录最小化）**: no root-level `AGENTS.md` / `CLAUDE.md` / `GEMINI.md` /
> `copilot-instructions.md`. Default `sync` creates exactly one root directory, `.agents/`.
> Claude Code / Codex views are optional, created only when asked. 根目录默认只多一个 `.agents/` 目录。

## Canonical & Views（规范路径与视图）

| Path | Kind | Read by | Committed? |
|---|---|---|---|
| `.github/skills/` | **canonical** | Copilot (native), everyone via direct read | yes |
| `.agents/skills` | link → `.github/skills` (template) / identical copy (instance) | DSH, Cursor, Gemini CLI, Copilot (VS 2026) | template: no (git-ignored); instance: yes |
| `.claude/skills` | optional link → `.github/skills` | Claude Code | no (git-ignored) |
| `.codex/skills` | optional link → `.github/skills` | Codex | no (git-ignored) |

Two view modes exist. `link` is the default and always preferred; `copy` exists
only so that a repository created from this template works with **zero setup**.
两种模式：`link` 默认；`copy` 只用于"模板派生仓库开箱即用"。

| Mode | Command | Why |
|---|---|---|
| link | `sync` | no duplication, cannot drift |
| copy | `sync --view-mode copy` | survives a plain `git clone` on Windows, where a committed symlink checks out as a text file |

Default (one new directory):

```bash
python .github/skills/_shared/skillkit/cli.py sync
```

Optional harness views:

```bash
python .github/skills/_shared/skillkit/cli.py sync --harness claude   # .claude/skills
python .github/skills/_shared/skillkit/cli.py sync --harness codex    # .codex/skills
python .github/skills/_shared/skillkit/cli.py sync --harness all      # all three
```

Validate / inspect:

```bash
python .github/skills/_shared/skillkit/cli.py sync --check
python .github/skills/_shared/skillkit/cli.py validate --strict
python .github/skills/_shared/skillkit/cli.py status
python .github/skills/_shared/skillkit/cli.py catalog --format markdown
```

Do **not** create `.dsh/skills`, `.cursor/skills`, or `.gemini/skills` by hand: DSH/Cursor/Gemini already
read `.agents/skills`, and a second root would surface the same skill twice. 不要手建 `.dsh`/`.cursor`/`.gemini`。

## Invocation Matrix（调用方式）

| Harness | Discovery root | Manual invocation | Automatic? |
|---|---|---|---|
| **DSH** | `.agents/skills` (rank 200) | `skill` tool with the exact name; `/het-build` gesture | yes, catalog at pre-step |
| **Claude Code** | `.claude/skills` (optional view) | `/het-build` | yes, by description |
| **Codex** | `.codex/skills` (optional view); `$CODEX_HOME/skills` user root | `/use het-build` | yes, by description |
| **Cursor** | `.agents/skills` (also `.claude`, `.codex` if present) | `/het-build` or `@het-build` | yes |
| **GitHub Copilot** | `.github/skills` (VS 2026 also `.claude`/`.agents`) | `/het-build` in chat | yes, by description |
| **Gemini CLI** | `.agents/skills` alias (priority over `.gemini/skills`) | `activate_skill` | yes |
| **Generic / other** | direct file read | read `.github/skills/<name>/SKILL.md` | depends on the harness |

Sources（2026-10）：

- DSH: built-in `@deepseek-ai/dsh-skill-filesystem` / `dsh-tool-skill` package docs.
- Claude Code: <https://code.claude.com/docs/en/skills>
- Cursor: <https://cursor.com/docs/skills>
- Copilot: <https://docs.github.com/zh/copilot/how-tos/copilot-on-github/customize-copilot/customize-cloud-agent/add-skills>
- Visual Studio 2026: <https://learn.microsoft.com/en-us/visualstudio/ide/copilot-agent-skills>
- Gemini CLI: <https://geminicli.com/docs/cli/skills/>
- Open standard: <https://agentskills.io/specification>

## Fallback（没有原生技能加载器时）

1. No root-level agent file is required; start from the canonical tree. 不依赖根级入口文件。
2. Load `het-agent` for routing, or `het-guide` for onboarding:
   - `.github/skills/het-agent/SKILL.md`
   - `.github/skills/het-guide/SKILL.md`
3. Read `.github/skills/_shared/routing.md` for the intent → skill table.
4. Read the selected `.github/skills/<name>/SKILL.md` and follow it.
5. Print a catalog for the model when it cannot discover files:

```bash
python .github/skills/_shared/skillkit/cli.py catalog --format markdown
```

## Single-Body Rule（唯一正文规则）

- `.github/skills/` is the only editable body. 只有它可以改。
- A view is either a **link** (symlink or Windows junction) or a **byte-identical copy**.
  Anything else is a second body. 视图要么是链接，要么逐字节相同。
- A copy view exists only so that an instance created from this template works on clone; it
  is produced by `sync --view-mode copy` and is rejected the moment it drifts.
  副本视图只为派生仓库开箱即用，漂移即报错。
- `het-agent` exists only at `.github/skills/het-agent/SKILL.md`; a second real `het-agent/SKILL.md`
  anywhere else is a contract violation.
- Edit the canonical file, not the view. 改 canonical，不改视图。
- `skillkit validate` fails on a copy view that differs from the canonical tree.

## Shell Portability（命令可移植性）

The quality gates are cross-platform (`python .github/misc/gates.py ...`); the remaining POSIX-only pieces are
`bash .github/misc/run-megalinter.sh` and the docs toolchain. On Windows use one of:

- **WSL** or **Git Bash** for the POSIX commands;
- the repository's `.devcontainer/` (recommended for a reproducible Linux toolchain);
- the equivalent editor task/command, or PowerShell (e.g. `Get-ChildItem` instead of `find`).

The Python and Conan commands (`python .github/skills/_shared/skillkit/cli.py ...`, `conan create ...`) are cross-platform.
