# Contributing to this project（贡献指南）

## 提交规范（Commit Conventions — canonical form）

提交信息是**双通道**：conventional 前缀驱动版本号（semantic-release），emoji 驱动附加 CI 流水线（见 `.github/skills/_shared/gitmoji.md`）。

**Canonical form**: `<type>(<emoji|scope>)?!?: <description>`

| type | 用途 | 示例 |
|---|---|---|
| `feat` / `fix` / `perf` | 版本号变更（minor/patch） | `feat(:fire:): cross-compile support` |
| `docs` / `test` / `build` / `ci` / `refactor` / `style` / `chore` | 不触发版本变更 | `chore(release): 1.0.0 [skip ci]` |

- 破坏性变更：`feat!:` 或正文含 `BREAKING CHANGE:` → major。
- **CI 门禁强制**：`ci-orchestrator.yml` 的 `commit-lint` job 在 push/PR 上校验；本地自查 `npx commitlint --config .github/misc/commitlint.config.js --from HEAD~1 --to HEAD`。
- ⚠️ `.github/misc/commitlint.config.js` 与 `.github/misc/.releaserc.json` 的 `headerPattern` 必须同步修改，否则会出现"门禁通过但版本不自增"。

## PR 流程

1. 从 `main` 拉分支，按 canonical form 拆分提交。
2. 保证 `commit-lint`、`schema-check`、`build`、`tests`（PR 上 shift-left 强制）全绿。
3. 填写 PR 模板自检清单；`CODEOWNERS` 评审人为必审。
4. 合并到 `main` 后，release 由 `(:package:):` + Release 配置 + 开关触发（semantic-release）。

## 本地开发（Local Setup）

```bash
# 构建环境
source ~/venv/build/bin/activate
# node 工具（commitlint/ajv/semantic-release）
npm ci --no-audit --no-fund
# 第一道闸（一次性）
pip install pre-commit && pre-commit install --config .github/misc/.pre-commit-config.yaml

# 质量自查（门禁命令只有一份实现：.github/misc/gates.py，CI/pre-commit/编辑器共用）
# 注：clang-format 需 >= 19，建议 `pip install clang-format==23.1.0`
npx ajv validate -s .github/misc/metadata.schema.json -d metadata.json --spec=draft7
python3 .github/misc/gates.py format   # clang-format --dry-run --Werror，.c/.h 走 C 配置
python3 .github/misc/gates.py tidy     # clang-tidy（WarningsAsErrors，标准取自 metadata.json）
# MegaLinter SAST（会拦 CI，镜像 megalinter job；需 Docker/Podman）
# 版本已固定为 v8.8.0，与 CI action（oxsecurity/megalinter@e08c2b05...）一致
bash .github/misc/run-megalinter.sh          # 或 npm run lint:megalinter；--fix 自动修复

# 构建 + 测试（此处 Debug 是本地复现基线：覆盖率在 Debug 下测量）
# CI 与发版跟随 metadata.build_type（metadata-controller 注入），两者不是同一个来源
conan create . -s build_type=Debug --build=missing
# 需要按项目声明的类型构建时：-s build_type=$(python3 .github/misc/meta.py build_type)
```

## 工程契约（Contracts）

- `metadata.json`：单一真源，schema 见 `.github/misc/metadata.schema.json`；字段语义见 `.github/skills/_shared/metadata-contract.md`。
- 代码规范：`.github/misc/.clang-format-c`（`.c/.h`）、`.github/misc/.clang-format-cpp`（`.hpp/.cpp`）、`.github/misc/.clang-tidy`（WarningsAsErrors）与 `.github/skills/_shared/code-conventions.md`。
- 测试体系：`test_package/test/{unit,stress}/`，GTest 只进 test_package（见 `test-conventions.md`）。

## 仓库布局（Repository Layout）

| 位置 | 内容 | 原因 |
|---|---|---|
| `.github/workflows/` | CI 流水线 | 所有 workflow |
| `.github/misc/` | CI 调用的工具配置（clang-format 双配置、clang-tidy、gitleaks、commitlint、releaserc、schema、mega-linter、pre-commit、labels）与本地/CI 共用的入口脚本（`gates.py` 门禁唯一实现、`meta.py` 读取 metadata） | 保持根目录干净；流水线显式传路径 |
| `.github/` | `CODEOWNERS`、`CONTRIBUTING.md`、PR 模板、dependabot | GitHub 约定位置 |
| 根目录 | `package.json`/`package-lock.json`（npm）、`conanfile.py`/`conandata.yml`/`metadata.json`（conan）、`CMakeLists.txt`（cmake）、`Doxyfile`（`docs/build.py` 硬引用）、`.editorconfig`/`.gitignore`（编辑器框架自动发现）、`.vscode/`、`.devcontainer/` | 框架/规范强制根目录 |

> 注：`.clang-format` 不再放在根目录（clang-format 内置映射无法让 `.h` 走 C）；编辑器经 `.vscode/settings.json` 显式指向 `.github/misc/.clang-format-cpp`，C 家族的正确性以 CI 门禁为准。`node_modules/` 是 `npm install`/`npm ci` 的本地依赖缓存目录，不入库（`.gitignore`）。

> `.vscode/` 的定位：只降低上手摩擦，不参与任何 CI 路径——可复现性由 conan + CMake + CI 保证，所以里面的任务只调用 `.github/misc/gates.py`，不复制命令本体；改门禁只改一处，三处消费者自动跟随。
> 取舍：`extensions.json` 同时推荐 `clangd` 与 `cpptools`，两者都提供 C/C++ IntelliSense，同时启用会得到双份诊断。这是刻意保留的"两条路"——默认走 clangd（`settings.json` 已配好 `--style` 等参数），需要 Microsoft 调试器/嵌入式调试的用户保留 cpptools；日常建议只启用其中一个的 IntelliSense，避免重复报错。
> `compile_commands.json` 不入库：它随 build_type 与工具链变化，由 conan 的 `cmake_layout` 在本地 `build/<build_type>/` 下生成；clangd 找不到时退化为启发式解析，要精确结果直接跑 `gates.py tidy`（用真实编译参数）。所以 `settings.json` 不写死 `--compile-commands-dir`，否则等于把某个 build_type 的路径固化进模板。

## 回退（Rollback）

- 撤销远端分支/tag：`git push origin --delete <branch> <tag>`
- 本地回退到整改前：`git checkout main && git branch -D <branch> && git tag -d <tag>`
- CI 红定位：Actions UI 或 `gh run list`（需 `gh auth login`）
