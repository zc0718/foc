# 偏差许可 `performance-no-int-to-ptr` 与 `cppcoreguidelines-pro-type-reinterpret-cast`

| | |
|---|---|
| **规则** | `performance-no-int-to-ptr`（已启用）；`cppcoreguidelines-pro-type-reinterpret-cast`（**未启用**，见下） |
| **压制位置** | `src/api/bridge.cpp` 共 **8 处**（`:590` `:640` `:778` 等，逐处由 gen_subset 扫描得到） |
| **状态** | 前者 `deviation`；后者 `not-applicable` / `delta: vacuous` |

沙箱把它做成两份文件：`deviation/performance-no-int-to-ptr.md` 与
`deviation/cppcoreguidelines-pro-type-reinterpret-cast.md` 都指向本文件，因为**两处抑制写在同一个
`NOLINTNEXTLINE` 里**，理由也只有一条。两份文件各自很短，指向同一份论证，避免把同一段话抄两遍后开始分叉。

---

## 为什么

C ABI 的句柄 `foc_motor_t` 是 `struct foc_motor_s *`（不透明指针），但库**从不解引用它**：
它按 `magic | generation | slot` 打包成一个整数值，形式上用指针承载。这是刻意的设计，
头文件注释里记着原因：

> 早先的版本让句柄指向每个槽位的控制块；由于控制块会被下一次占用复用，**旧句柄会读到新代际**
> 从而通过校验。代际必须放在句柄值本身里。

于是这里有两次转换：`uintptr_t -> 指针`（签发）和 `指针 -> uintptr_t`（解析）。两条规则都说
"整数与指针之间不该来回转换"，它们是对的**当那个指针真的是地址**；这里它不是地址，是令牌。

判别点很清楚：**解引用了吗？** `resolve_in()` 在成员判定（高 32 位的 magic）之前不解引用任何
指针，所以野句柄不会触发访问异常——这正是把未定义行为变成 `FOC_ERR_PARAM` 的地方。
换成"真指针 + 控制块"的写法会让这条性质消失（上面那段历史就是它消失之后发生的事）。

## 影响

- **运行时**：无。转换本身不产生指令；打包/拆包是几十条位运算。
- **接口**：句柄的**表示**是 ABI 的一部分（`foc_motor_t` 的宽度），改它要改 ABI；本偏差正是在避免它。
- **危险面**：如果哪天有人真的解引用这个令牌，就是 UB。所以替代控制里第一条是硬要求。

## 替代控制

1. **`mint_handle()` / `resolve_in()` 是唯一两个转换点**，都在 `src/api/bridge.cpp` 内、都不导出。
   新增第三个转换点会在评审里显眼——它必须改本文件。
2. **解引用禁令有测试面**：`test_api_c_smoke.cpp` / `c_api_validation.c` 用**失效/野生句柄**调用
   每个入口，要求返回 `FOC_ERR_PARAM` 而不是崩溃；`resolve_in()` 的 `@post` 把"成员判定前不解引用"
   写成了契约。
3. **代际计数器**由 `test_enum_names.cpp` 之外的一组 API 用例覆盖（销毁-重建-旧句柄三连）。
4. 本偏差的**存在性**由 `subset_gate` 强制：8 处 `NOLINT` 缺记录即红。

## 批准人

`zc`（项目 owner）。

## 复查日期

**2027-05-06**。

**提前复查的触发条件**：① 出现第二个需要"令牌式句柄"的 ABI（届时该抽出公共工具并重新论证）；
② 有人提议把 `foc_motor_t` 改成整数句柄类型（`foc_motor_id_t`）——那是 ABI 变更，本偏差随之作废；
③ 编译器/检查器开始对 `reinterpret_cast` 给出无法用 `NOLINT` 抑制的诊断。
