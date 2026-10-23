---
name: het-setup
description: 'Environment setup & first build for the fcpp template (conan/cmake/doxygen/sphinx/arm-toolchain). Use when: environment setup / install conan / first build / prerequisites / how to start. 环境搭建/首次构建：装工具链并完成首次构建与文档生成。'
license: Apache-2.0
argument-hint: "Target platform: desktop/cross (optional) / 目标平台（桌面/交叉）（可选）"
user-invocable: true
---

# N3 · het-setup — Environment Setup / First Build（环境搭建 / 首次构建）

> For CI/CD novices & newcomers: explain what to install and why, then give commands. 先把"装什么、为什么"讲清楚，再给命令。

## Mental Model（心智模型）

> "The project uses Conan 2 for deps, CMake for build, Doxygen+Sphinx for docs. Install once, then it's all `conan create`. For cross-compile to a board, Conan pulls `arm-toolchain` automatically — no manual GCC cross toolchain."
> Conan 管依赖、CMake 构建、Doxygen/Sphinx 出文档；装一次即可；交叉编译时 Conan 自动拉 arm-toolchain。

## Prerequisites（前提条件）

| Tool（工具） | Version | Purpose（用途） |
|------|------|------|
| Python | >= 3.10 | scripts & Conan |
| Conan | >= 2.0 | deps & build |
| CMake | >= 3.28 | auto-installed by Conan |
| Compiler | GCC / Clang / MSVC | build |
| Doxygen + Graphviz | — | docs (optional) |
| Sphinx + sphinx-intl + sphinx-rtd-theme + numpy | — | docs (optional) |

## Install（安装步骤）

```bash
pip install conan
conan profile detect --force
```
Docs tools (optional, Linux): `sudo apt install -y doxygen graphviz && pip install numpy sphinx sphinx-intl sphinx-rtd-theme`.

> Windows: use `py -m pip install conan` (or a venv) and run the POSIX commands in WSL, Git Bash or the
> `.devcontainer/`; `conan` / `python` commands are cross-platform. Windows 用 `py -m pip` 安装，POSIX 命令用 WSL/Git Bash/devcontainer。

## First Build（首次构建）

```bash
conan create . -s build_type=Debug --build=missing
```
> Builds fcpp_c/fcpp_cpp → runs test_package smoke test → packages into local cache. Success shows `CPP Compiler is ready!`. 构建库 + 跑冒烟测试 + 打包。

## Local Docs（本地出文档，可选）

```bash
python ./docs/build.py
```

## Cross-Compile to Board（交叉编译到板子）

```bash
python benchmark/script/run_bench.py --no-flash   # build only（只构建）
python benchmark/script/run_bench.py              # build + flash/deploy（构建+上板）
```
- Conan auto-installs `arm-toolchain`. 自动安装交叉工具链。
- Baremetal deps limited by `baremetal_white_list`（默认 etl/ArduinoJson）.
- Details → `het-board` (S5).

## Self-Help When Red（失败自救）

1. `conan` not found → `pip install conan` + reopen terminal. 重开终端。
2. `conan profile detect` error → check compiler in PATH. 检查编译器。
3. Slow first build → `--build=missing`; configure mirror/private server. 配镜像/私服加速。
4. Windows `conan create` odd → shared forced to static (expected). Windows 下 shared 回退是预期。

