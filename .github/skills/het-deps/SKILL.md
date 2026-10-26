---
name: het-deps
description: 'Dependency governance for the fcpp template (metadata 4-bucket spec, conan_targets mapping, switch gating, baremetal whitelist). Use when: add/remove a dependency, dependency config. 依赖治理：按四桶规范与开关门控正确增删依赖。'
license: Apache-2.0
argument-hint: "Dep to add/remove (optional) / 要增删的依赖（可选）"
user-invocable: true
---

# N1 · het-deps — Dependency Governance（依赖治理）

> For developers. Facts: `.github/skills/_shared/metadata-contract.md`、`_shared/gitmoji.md`。

## Mental Model（心智模型）

> "Adding a dependency touches two files and three consumers, plus one quality gate: `conandata.yml` (declare) → `metadata.json` `dependencies` (bucket + target mapping) → verify main recipe and test_package filter consistently → sync the clang-tidy gate's system-header mapping in `security-linters.yml`. A wrong bucket or switch pollutes downstream (benchmark); a missing gate mapping fails clang-tidy in CI."
> 加依赖要动两个文件、检查三个消费方、同步一处质量门禁；分错桶或漏开关，下游（benchmark）会被污染；漏同步门禁映射，clang-tidy 会 fail-loud。

## The 4-Bucket Spec（依赖四桶语义，不可违背）

| Bucket（桶） | Meaning（含义） | Example |
|------|------|------|
| `common` | shared by C/C++ targets | `{"ZLIB": ["ZLIB::ZLIB"]}` |
| `c` | C target only | `{"PCRE2": ["pcre2::pcre2"]}` |
| `cpp` | C++ target only | `{"Eigen3": ["Eigen3::Eigen"], "etl": ["etl::etl"]}` |
| `infra` | host-side infra layer | `{"GTest": [...], "pybind11": [...]}` |

**Hard rules（硬规则）**:
1. One package in **one bucket only**. 一个包只放一个桶。
2. **GTest belongs to test_package only** (host GTest), never in the main package's component requires. GTest 只归 test_package。
3. **pybind11 enters the main package only when `enable_python_bindings=true`**. pybind11 跟随开关。
4. Key case differs from conandata package name (`Eigen3` vs `eigen`) — always compare lowercase. 大小写不同，统一小写归一。
5. **Components are not dependencies**：`<name>_c` 与 `<name>_cpp` 之间的边写在 `metadata.json` 的 `component_requires`（`{from: [to]}`），不进四桶，也不改 `conanfile.py`。四桶只管**外部**包；组件图只管**包内**两块库。

## 3-Step Flow（增依赖三步流程）

**① `conandata.yml` — declare the version**
```yaml
requirements:
  - "mylib/1.2.3"
```
**② `metadata.json` `dependencies` — bucket + target mapping**
```json
"cpp": {"MyLib": ["mylib::mylib"]}
```
- If the CMake target name differs from the Conan package, add a mapping in `conanfile.py` `conan_targets` (e.g. `Eigen3::Eigen` → `eigen::eigen`). 目标名不同需补 `conan_targets` 映射。
- Evaluate baremetal availability: if OS-dependent or not cross-compilable, keep it out of `baremetal_white_list`. 评估裸机可用性。

**③ Verify the three consumers（验证三消费方口径一致）**
- Main `requirements()`: baremetal whitelist; gtest never required; pybind11 gated. 主配方过滤口径。
- Main `_preparing_deps_links()`: `infra` GTest popped unconditionally; pybind11 gated. 主配方 deps 口径。
- test_package: GTest kept; pybind11 gated. test_package 保留 GTest。

**④ Sync the clang-tidy quality gate（同步质量门禁）**
- `.github/workflows/security-linters.yml` 从 `conandata.yml` 推导系统头依赖：apt 映射 zlib→zlib1g-dev / pcre2→libpcre2-dev / eigen→libeigen3-dev；ETL 版本从 conandata 解析下载。
- 新依赖的头文件被 `src/` 引用时：有 apt 包 → 在 workflow 的 `mapping` 加一行；header-only 无 apt 包 → 参照 ETL 用固定版本源码包下载。
- 头文件搜索路径由 `.github/misc/gates.py tidy` 统一提供：`-Iinclude` + eigen（`EIGEN_INC` 可覆盖）+ `GATES_INCLUDE`（`os.pathsep` 分隔）+ `-isystem $ETL_INC`。装到标准前缀（`/usr/include`）的 apt 包无需额外配置；非标准前缀 → 在 workflow 里导出 `GATES_INCLUDE`。
- 不补映射 → clang-tidy 门禁报 file-not-found（fail-loud 是有意设计）。

## Switch Map（开关对照）

| Dep（依赖） | Controlled by（受控于） |
|------|------|
| gtest | test_package only (host) |
| pybind11 | `enable_python_bindings` |
| OS deps (zlib/pcre2/eigen/gtest/pybind11) | `baremetal_white_list`（默认 `["etl","ArduinoJson"]`） |

## Verify（验证）

```bash
conan create . -s build_type=Debug --build=missing
python benchmark/script/run_bench.py --no-flash   # baremetal cross check（裸机交叉验证）
```

## Self-Help When Red（失败自救）

1. "dep not found" → check `conandata.yml` version / private server. 检查版本与私服。
2. Downstream (benchmark) polluted → GTest/pybind11 wrongly in main requires. 检查是否把测试依赖错放进主包。
3. Baremetal build fails → check `baremetal_white_list`. 检查裸机白名单。
4. Case mismatch → normalize lowercase. 大小写统一小写。
5. clang-tidy gate file-not-found → extend the system-header mapping in `security-linters.yml` (step ④). 门禁报找不到头 → 补门禁依赖映射（步骤④）。

