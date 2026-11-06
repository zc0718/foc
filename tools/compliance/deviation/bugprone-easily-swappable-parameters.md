# 偏差许可 `bugprone-easily-swappable-parameters`

| | |
|---|---|
| **规则** | `bugprone-*.easily-swappable-parameters`（clang-tidy） |
| **压制位置** | 全库 **31 处**函数签名（清单见下），**不给任何一处加 `NOLINT`**：豁免登记在 `.github/project/enforce.json` 的 `except` 里，`enforce` 门按名单过滤，且**没有本文件即红** |
| **状态** | 已登记（`rules.tsv` 里这一行是 `deviation`） |

## 为什么

这条检查问的是："两个相邻参数类型相同或可互转，调用方会不会把顺序写反？" 在通用代码里这是个好问题。
在本库里它问的是**整个 API 的形状**：本库的每个环节都是一个纯函数，收两个标量、返回一个值——
`step(dt, angle)`、`park(alpha, beta)`、`reconstruct(a, b)`、`apply(omega_m, theta_mech)`。
要让它闭嘴只有两条路：

1. **把每对参数包成一个结构体。** 那意味着公开接口全改一遍（C++ 侧与 C ABI 侧都要动），
   而且与控制环的意图相反：本库刻意用 POD 标量，是因为 `-fno-exceptions -fno-rtti`、无分配、
   控制环内无虚调用，参数应当逐寄存器传递。
2. **把参数改成不同的类型**（强类型 `Meters`/`Radians` 之类）。本库确实在**真正会同名同型**的地方
   这么做了——`Ab<T>`/`Dq<T>`/`Uvw<T>` 就是为此存在的，凡是两个量属于**不同坐标系**或**不同物理量**
   的场合，签名里看到的是结构体而不是两个 `float`。剩下的 31 处，两个参数本来就属于同一类量
   （两个时间、两个角度、两个电流），包成结构体不会增加任何信息。

所以这是**规则不适用于本项目的接口风格**，不是"以后修"。差别在于：这不是"检查没开所以没有宾语"，
而是"检查开着、它说的每一处我都看过、并认为不该改"。

## 影响

- **运行时**：无（本许可不改变任何代码）。
- **接口**：无（正因为要保住接口形状才签的）。
- **可读性 / 可维护性**：**有一个真实代价**，写在这里而不是藏起来：写错顺序时编译器不会帮忙。
  本库对此的补偿见下一条，而不是"注意点就行"。

## 替代控制

1. **同名同型的"真容易搞混"处已经不在名单里**：`Ab<T>`/`Dq<T>`/`Uvw<T>` 把坐标系编码进类型，
   `park`/`inv_park` 收的是 `(Ab<float>, float)` 而不是三个 `float`；`set_dq(Dq<float>)```
   收结构体。名单上剩下的是同量纲、同单位、顺序由代数本身决定的场合。
2. **参数名在声明处就写出来**：31 处里绝大多数在头文件里带 `@param` 说明（`docshape`/`docvoice`
   管着这一层），而不是只有实现里的一行。
3. **数值行为有独立的用例兜底**：这些函数里的绝大多数被 857 条 GTest 与 SIL 基线覆盖，
   把参数顺序调换会改变数值结果，测试会红——也就是说错误是**可观测**的，而不是静默的。
   这是本许可能签的最实质的理由：交换参数不是"看起来对但算错"，它有输出。
4. **名单本身受门禁管**：`.github/project/enforce.json` 的 `except` 每一条都必须有本文件，
   `enforce` 门的 `evaluate()` 强制这一点；删掉本文件、或让名单里出现一个没有记录的规则，门禁立刻红。

## 批准人

`zc`（项目 owner）。

## 复查日期

**2027-05-06**。

**提前复查的触发条件**：① 本库决定给标量加强类型（那时这条检查应该被重新打开，名单应当缩短而不是消失）；
② clang-tidy 给这条检查加了可配置的参数名白名单，那时可以按名放行而不必整条豁免；
③ 名单上某处函数被改成"两个参数确实是不同物理量"，那处应当改签名而不是留在名单里。

## 名单（31 处）

| 文件 | 函数 | 相邻参数个数 |
|---|---|---|
| `include/foc/comm/encoder_stream.hpp` | `TurnAssembler` | 2 |
| `include/foc/comm/frame.hpp` | `encode_frame` | 2 |
| `include/foc/core/alignment.hpp` | `step` | 2 |
| `include/foc/core/board_check.hpp` | `sweep_sample_delay` | 2 |
| `include/foc/core/command_source.hpp` | `step` | 2 |
| `include/foc/core/commissioning.hpp` | `add` | 2 |
| `include/foc/core/commissioning.hpp` | `add`（第二个重载） | 2 |
| `include/foc/core/commissioning.hpp` | `axis_deadtime_v` | 2 |
| `include/foc/core/gear_ratio.hpp` | `apply` | 2 |
| `include/foc/core/hall.hpp` | `six_step_duty` | 2 |
| `include/foc/core/hall.hpp` | `update` | 2 |
| `include/foc/core/machine_map_text.hpp` | `read_float` | 2 |
| `include/foc/core/six_step.hpp` | `accept` | 2 |
| `include/foc/core/soft_limit.hpp` | `shape` | 2 |
| `include/foc/core/stall_start.hpp` | `step` | 2 |
| `include/foc/core/step_metrics.hpp` | `step_metrics` | 2 |
| `include/foc/core/trace_parse.hpp` | `trace_segment` | 2 |
| `include/foc/motor/torque_map.hpp` | `torque_command` | 2 |
| `src/api/bridge.cpp` | `foc_bridge_mtpa_reference` | 2 |
| `src/api/bridge.cpp` | `foc_bridge_torque_map_build` | 2 |
| `src/core/alignment.cpp` | `step` | 2 |
| `src/core/cogging_comp.cpp` | `table_torque_nm` | 2 |
| `src/core/limits.cpp` | `step` | 2 |
| `src/core/motion_profile.cpp` | `shape` | 2 |
| `src/core/observer/hfi_observer.cpp` | `predicted_gain_a` | 2 |
| `src/core/observer/hfi_observer.cpp` | `seed_angle` | 2 |
| `src/core/observer/ideal_observer.cpp` | `set_truth` | 2 |
| `src/core/pi.cpp` | `run` | 4 |
| `src/core/single_shunt.cpp` | `reconstruct` | 2 |
| `src/core/speed_from_angle.cpp` | `update` | 2 |
| `src/motor/motor_config.cpp` | `current_pi_config` | 2 |
