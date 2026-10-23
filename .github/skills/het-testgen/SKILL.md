---
name: het-testgen
description: 'Test case generation for the fcpp template. Use when: generate/add tests, TDD, test-first from a PRD or PlantUML blueprint, HPO/model tuning. 测试生成：从代码或蓝图自动生成 GTest，支持测试先行与模型调参。'
license: Apache-2.0
argument-hint: "Mode/target module: A=code, B=blueprint / 模式/目标模块（A:代码 B:蓝图）"
user-invocable: true
---

# N4 · het-testgen — Test Case Generation（测试用例自动生成）

> For developers. Facts: `.github/skills/_shared/test-conventions.md`、`code-conventions.md`、`metadata-contract.md`。

## Two Input Modes（两种输入模式）

### Mode A: code-driven（模式 A：代码驱动 / 逆向生成）
**Input**: `include/` public headers + `src/` implementations
**Output**: matching GTest cases in `test_package/test/unit/*.cpp` (also the coverage target)

Steps（步骤）:
1. Scan `include/*.hpp` (and `.h`) for public API: signatures, classes, templates, params/returns. 提取公开 API。
2. Cross-check `src/` behavior (e.g. `etl.cpp` uses `etl::transform` for element-wise add). 对照实现确认行为。
3. Generate per API: positive + boundary (0-length/empty/extreme) + negative (if semantics allow). 正向 + 边界 + 负向。
4. Write to `test_package/test/unit/<module>_test.cpp`, follow GTest spec. 写入测试文件。
5. Verify: `conan create . -s build_type=Debug --build=missing` (with `activate_code_coverage` to see line coverage). 验证编译与覆盖率。

### Mode B: blueprint-driven / test-first（模式 B：蓝图驱动 / 测试先行）
**Input**: development blueprint (PRD / PlantUML architecture diagram)
**Output**:
1. Extract the input → output contract from the blueprint. 提取输入/输出契约。
2. **Write GTest cases first** as the contract (failing/not compiling is expected — implementation not written yet). 先写 GTest 契约，失败是预期。
3. Produce an **implementation plan**: which `include/*.hpp` + `src/*.cpp` to add, signatures, how tests drive them. 输出实现计划。
4. Agentic Coding implements until tests go green. 按契约实现直到跑绿。

**Extended use（扩展应用）**: model design / HPO — treat evaluation metrics + data pipeline as a fixed "verification contract" (benchmark cases or tuning harness). 模型设计/HPO：把评估指标固化为验证契约。

## Output Spec（输出规范）

- Location: `test_package/test/unit/` (unit), `test_package/test/stress/` (stress). 位置。
- Structure: `#include <gtest/gtest.h>` + `TEST(SuiteName, CaseName)`. GTest 结构。
- Naming: `<module>_test.cpp`; suite = module name; case describes behavior. 命名。
- Coverage: each public API >= 1 positive + >= 1 boundary. 覆盖目标。
- `main.cpp` entry auto-generated — do not hand-write. 入口自动生成。
- Turn on `trigger_tests=true` (plus `activate_code_coverage` as needed). 开开关。

## Quality Checklist（质量自检清单）

- [ ] Every public API has a case. 每个公开 API 至少一例。
- [ ] Boundary inputs included. 含边界输入。
- [ ] Cases compile (local `conan create` passes). 能编译。
- [ ] Coverage report produced when enabled. 覆盖率可出。
- [ ] No GTest written into `src/` or `benchmark/`. 不往主包/benchmark 写 GTest。

## Guardrails（边界与护栏）

- Mode A is read-only analysis + new test files; never modifies `include/`/`src/`. 模式 A 只读 + 新增测试文件。
- Mode B plans new implementation files — **present the plan and confirm before writing**. 模式 B 写文件前先给计划确认。
- Test-first failures are expected; do not revert the cases. 测试先行失败是预期。

