# L3 约定对齐表（任何比对前必须先对齐）

> 定位：PRD §8.5「约定对齐」。**这张表不是比对结果，是比对的前置条件**——角度、旋转方向、
> 坐标系、单位制、钳位方式任何一项不一致，后续的 RMSE/超调比对都会得出错误结论。
>
> 本表记录的是**本项目（`foc`）自身采用的口径**，以及和任何外部参考（当前是 SguanFOC，
> 见 §4 的状态说明）对齐时必须先确认的条目。

## 1. 本项目采用的约定（权威口径）

这些口径由 `include/foc/` 的实现**唯一决定**，任何参考实现都要向这里对齐，而不是反过来。

| 条目 | 本项目口径 | 证据（代码即规范） |
|------|-----------|-------------------|
| 角度单位 | rad（弧度） | `hal::PositionSensor::angle_mech_rad()` 返回 rad |
| 机械角零点 | 转子 d 轴与 A 相轴线重合时为 0；位置传感器须自行消掉安装偏置 | `hal::PositionSensor` 契约注释 |
| 电角度 | `theta_e = pole_pairs * theta_mech`，**不归一化**（`park`/`inv_park` 内部用 `cos/sin`，周期性天然成立） | `motor::Motor::tick()` |
| 旋转方向 | **逆时针（CCW）为正**：`park(v, θ)` 与 `inv_park(v, θ)` 互为逆运算，θ 增大时矢量随之正向旋转 | `core::park` / `core::inv_park` |
| Clarke 变换 | **幅值不变式**（amplitude invariant）：`alpha = (2a-b-c)/3`、`beta = (b-c)/√3`；三相正弦峰值为 `I` 时 `|i_ab| = I` | `core::clarke`，用例 `Transforms.ClarkeIsAmplitudeInvariant` |
| dq 约定 | **电动机惯例（motor convention）**：`iq > 0` 产生正向电磁转矩；d 轴对齐转子磁链 | `sil_pmsm_plant.hpp` 的转矩式与 `Motor` 的 `iq_ref` 符号 |
| 转矩常数 | `Kt`［N·m/A］与磁链 `psi_f`［Wb］的关系固定为 `psi_f = Kt / (1.5·po​le_pairs)` | `foc::psi_f_wb()`，PRD D24 |
| 单位制 | **SI 全量**，不用标幺（per-unit）；电压 V、电流 A、转速 rad/s（机械）、转矩 N·m、电感 H、惯量 kg·m² | `motor::MotorConfig` 字段注释 |
| 电流正方向 | 流入电机为正 | `hal::CurrentSense::read_uvw()` 契约 |
| SVPWM 约定 | 中点（min-max）注入的对称七段式；占空比 `duty = clamp(0.5 + (v_phase − v_mid)/v_bus, 0, 1)`；线性调制边界 `|v_ab| = v_bus/√3` | `core::Svpwm::modulate()`，PRD D12 |
| 母线电压 | 每个控制周期读一次 `hal::DcLink::voltage_v()`，SVPWM 用它做归一化；`v_bus <= 0` 输出 50% 占空比并置 `Param` | `core::Svpwm::modulate()` |
| PI 形式 | 位置式（并联）：`I += ki·e·dt`（前向欧拉）→ 积分钳位 → `out = clamp(kp·e + I)`；**抗饱和只有静态积分限幅**，无 back-calculation | `core::Pi::step()`，PRD D27 |
| 控制器更新率 | 电流环 = PWM 载波频率（SIL 台架 20 kHz / 50 µs）；速度环与电流环同频执行 | `sil_bench.hpp:standard_bench_config()` |
| 积分器（SIL） | C++ 侧：手写 RK4，每控制周期 4 个子步、电压矢量在**静止系**零阶保持；仲裁侧：scipy `solve_ivp`（RK45, rtol 1e-9），两条路径刻意不同源 | `sil_plant_hal.hpp`、`.github/misc/offline_arbiter.py` |
| 比对切面 | 机械转速（rad/s，机械）、`id`/`iq`（A）、时间从阶跃时刻起算；采样按控制周期对齐（L2 参考轨迹每 5 个周期抽稀一行） | `test_sil_l2_reference.cpp` |

## 2. 对齐时最容易出错的四条（踩过的坑）

1. **机械 vs 电角度**：`pole_pairs` 漏乘会让整个 dq 系以 `1/p` 的速度旋转，表现为"电流环
   怎么调都跟不上"。SIL 里 `PlantHal::advance()` 与 `PlantPosition` 都必须用同一套换算。
2. **幅值不变式 vs 功率不变式**：功率不变式 Clarke（带 `√(2/3)`）会让 `|i_ab|` 差 `√(3/2)`，
   于是"给定 1 A 实际 1.22 A"，而**闭环的积分器会把这个增益误差悄悄补掉，只有开环比对才看得见**。
3. **静止系保持 vs 旋转系保持**：真实逆变器在整个 PWM 周期内把电压矢量钉在**静止**系；
   若按 dq 系保持（每个子步重新 park），高速时等效于额外加了零阶保持误差。本项目 C++ 侧与
   仲裁侧都按静止系处理。
4. **稳态误差的"斜坡陷阱"**：转速斜坡上电流环必然有 `e = ĖMF / ki` 的跟随偏差（本地约 1%），
   这不是缺陷；判"稳态误差"必须先确认处于匀速段（PRD D28）。

## 3. 与外部参考比对时的额外检查项

| 条目 | 需要确认什么 |
|------|-------------|
| dq 约定 | 参考实现是 motor convention 还是 generator convention（`iq` 符号相反） |
| 旋转方向 | CW 还是 CCW 为正（`theta` 符号相反） |
| 单位制 | 标幺 vs SI；若是标幺，基准值（`V_base`/`I_base`/`ω_base`）各是多少 |
| PI 抗饱和 | 有无、何种方式（钳位 / back-calculation / conditional integration） |
| SVPWM | 七段式还是五段式；有无过调制；钳位是逐相还是矢量 |
| 采样与延迟 | 是否有 1 拍计算延迟、是否中点采样；控制器更新率与 PWM 频率的关系 |

## 4. SguanFOC（第三层参考）的状态

- **当前状态：未 vendor、未构建、不参与任何 CI**（PRD ADR-006 / D8）。
- 上游：`Sguan-ZhouQing/SguanFOC_Library`（个人主导项目，许可与长期维护性未经审计）。
- 按 PRD §12 T2.1，vendor 快照（锁定 commit）到 `third_party/sguanfoc/` 并写 `UPSTREAM.md`
  属于**要把第三方代码放进本库仓库**的决定，涉及许可、体积与后续维护，**属于架构级决策，
  留给仓库所有者裁决**；在裁决前本表 §1/§3 即为唯一有效口径。
- 即使将来 vendor，L3 也只用于**人工阅读特定算法思路**（LADRC、HFI 等），不产生比对数据、
  不进 CI；与它不一致时**默认不信任 SguanFOC**（PRD §8.5）。

## 5. 变更规则

- 本表任何一条口径发生变化（例如引入标幺、改 generator convention、给 PI 加 back-calculation），
  必须先改本表 → 再改代码 → 再重新生成 L2 参考轨迹（`.github/misc/offline_arbiter.py`）→
  最后才允许提交。顺序颠倒会让"门禁绿了但口径错了"。
- 新增比对场景时，追加到 §1 的"比对切面"行，并在 PRD §16 记一条 D 记录。
