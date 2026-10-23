## 变更说明（What & Why）

<!-- 简述本次变更与动机 -->

## 自检清单（Checklist）

- [ ] 提交遵循 canonical form：`<type>(<emoji|scope>)?!?: <description>`（见 `.github/CONTRIBUTING.md`）
- [ ] `metadata.json` 通过 schema 校验：`npx ajv validate -s .github/misc/metadata.schema.json -d metadata.json --spec=draft7`
- [ ] 格式检查通过：C 家族（`.c/.h`）用 `.github/misc/.clang-format-c`，C++ 家族（`.hpp/.cpp`）用 `.github/misc/.clang-format-cpp`：`python3 .github/misc/gates.py format`
- [ ] 静态检查通过：`python3 .github/misc/gates.py tidy`
- [ ] 构建与测试通过：`source ~/venv/build/bin/activate && conan create . -s build_type=Debug --build=missing`
- [ ] 新公共 API 已有对应 GTest 用例（`test_package/test/unit/`）
- [ ] 若修改依赖，已核对 4-bucket 语义与 `baremetal_white_list`（见 `metadata-contract.md`）

## CI 说明（CI Notes）

<!-- 如需触发附加流水线，说明使用的 emoji（:shield:/:building_construction:/:beer:/:book:/:package:） -->
