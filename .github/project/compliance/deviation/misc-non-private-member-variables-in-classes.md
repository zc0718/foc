# 偏差许可 `misc-non-private-member-variables-in-classes`

| | |
|---|---|
| **规则** | `misc-non-private-member-variables-in-classes`（clang-tidy） |
| **压制位置** | 全库 **47 处**成员（6 处文件，清单见下），**不给任何一处加 `NOLINT`**：豁免登记在 `.github/project/enforce.json` 的 `misc` 族 `except` 里，`enforce` 门按名单过滤，且**没有本文件即红** |
| **状态** | 已登记（`rules.tsv` 里 `misc-*` 不在收编范围，本族由 `enforce` 门在钉住配置之外启用） |

## 为什么

这条规则问的是："这个成员变量为什么要公开？" 它在**有行为、有不变式**的类里是个好问题：公开成员让人绕过不变式，
而类的意义就是守住不变式。

本库里被它点到的 47 个成员，没有一个是那种类。它们的宿主全部是**配置与测量聚合体**：

| 宿主 | 成员数 | 它是什么 |
|---|---:|---|
| `driver::callback_hal.hpp`（`HalCallbacks` 及其分项结构） | 19 | C 回调集在 C++ 侧的逐字段镜像；调用方填、驱动读，没有不变式可守 |
| `driver::stub_hal.hpp`（`StubPwm`/`StubCurrentSense`/…） | 15 | 测试替身暴露的写入点与观测点，公开**就是**它的接口 |
| `core::board_check.hpp`（`BoardReport`/`BoardItem`/`BoardLimits`） | 5 | 一张检查结果表：`items[]` 加 `count`，本身就是聚合 |
| `core::machine_map.hpp`（`MachineMapRow`、`MachineMap`） | 5 | 从文本表里读出来的行，逐字段搬运 |
| `comm::EncoderStream`（`Config`、`Status`） | 2 | 位宽配置与位域状态，`constexpr` 构造 |
| `src/api/bridge.cpp`（槽位环） | 1 | 桥接层内部的句柄池，无名命名空间内、不导出 |

它们共同满足三条：**按 `const&` 传、没有虚函数、不被继承**。规则的建议是"做成 class，把成员藏起来、给访问器"，
而那恰恰会：
1. 把每次字段读取变成一次调用（本库的形状是 POD 逐寄存器传递，`-fno-exceptions -fno-rtti`、无分配、控制环内无虚调用）；
2. 让 `constexpr` 聚合初始化写不出来（`HalCallbacks{...}`、`BoardReport{...}` 是现在的写法）；
3. 在 C 侧镜像结构上多一层壳——那份结构本来就要和 `foc_hal_callbacks_t` 逐字段对得上，多一层只会让两边更容易漂。

所以这是**规则不适用于本项目的值类型风格**，不是"以后修"。差别在于：检查开着、它点到的每一处都看过、
并认为不该改。

## 影响

- **运行时**：无（本许可不改变任何代码）。
- **接口**：无（正因为要保住聚合形状才签的）。
- **可维护性**：**有一个真实代价**，写在这里而不是藏起来：这些聚合体的成员可以被人直接改，
  而不变式（例如 `BoardReport::count <= kMaxItems`）只能靠写入方自觉。补偿见下一条。

## 替代控制

1. **越界的写入点被 `constexpr` 上限与断言夹住**：`BoardReport` 的 `add` 在 `count >= kMaxItems` 时停手
   （`board_check.hpp` 里的 lambda），宁可丢一条也不越界；`MachineMap` 的 `count` 只在 `push_row` 里自增。
2. **对外面暴露的那一层不是聚合**：C ABI 侧拿到的是 `foc_*_t` 透明句柄与显式函数，C++ 侧拿到的是
   `Motor<...>` 这个有大写方法、有不变式、成员私有的类。被点到的是两者之间的**数据搬运层**，它本来就没有不变式。
3. **这一族的另外三条真在跑**：`misc-definitions-in-headers`、`misc-use-internal-linkage`、
   `misc-const-correctness` 由 `enforce` 门强制执行（见 `.github/project/enforce.json` 的 `misc` 族），
   所以"因为豁免了这一条，整个 `misc-*` 就没人管了"这件事不成立。

## 名单（47 处）

| 文件 | 成员 |
|---|---|
| `include/foc/driver/callback_hal.hpp` | `HalCallbacks`: `user`、`pwm_write_duty`、`read_current`、`read_angle_mech_rad`、`read_voltage_v`、`read_velocity_rad_s`、`pwm_fault`、`read_drive_fault`、`pwm_enable`、`pwm_disable`、`now_us`、`position_has_index`、`position_index_pulse`、`pwm`、`current_sense`、`position`、`dc_link`、`clock`、`fault_input`（19） |
| `include/foc/driver/stub_hal.hpp` | `pwm`（3 个替身各一）、`current_sense`（3）、`position`（3）、`dc_link`（3）、`fault_input`（3）（15） |
| `include/foc/core/board_check.hpp` | `BoardReport::items`、`count`、`failures`、`BoardLimitHit::cause`、`cause_phase`（5） |
| `include/foc/core/machine_map.hpp` | `MachineMapRow::ld_h`、`lq_h`、`psi_f_wb`、`MachineMap::rows`、`count`（5） |
| `include/foc/comm/encoder_stream.hpp` | `Config::single_turn_bits`、`multi_turn_bits`（2） |
| `src/api/bridge.cpp` | 槽位环 `ring`（1） |

## 批准人

zc（2026-11-06，`enforce` 门启用 `misc-*` 时一并拍板）

## 复查日期

2027-11-06，或下一次有人要往这些聚合体里加**需要释放的成员**时——那一刻它就不再是 POD，
本许可的前提消失，这条豁免要重新论证。
