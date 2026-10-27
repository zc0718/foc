# L3 · Convention table — read this before comparing with another FOC project

**Purpose / 用途**：almost every wrong conclusion about "their controller is better" comes from
comparing two numbers that were never in the same units, the same angle zero, or the same sign
convention. This table fixes ours and states how to convert theirs. **No cross-project comparison is
valid until this table has been consulted.**
"他们的控制器更好"这类结论，绝大多数来自比较了两个从未处于同一单位、同一角度零点或同一符号约定下的数字。
本表固定我们的口径，并写明如何换算别人的。**任何跨项目比对，先查这张表。**

---

## 1. Units — what the numbers mean / 单位口径

| Quantity / 量 | This library (SI) / 本库（SI） | VESC | ODrive | moteus | SimpleFOC |
|---|---|---|---|---|---|
| Angle / 角度 | **rad** (mechanical unless stated) / rad（未注明即机械角） | degree (some params) / 度 | **turn** (1 turn = 2π rad) | **rev** (= turn) | rad |
| Angular speed / 角速度 | **rad/s** mechanical / 机械 rad/s | **eRPM** (electrical RPM = mech RPM × pole pairs) | **turn/s** | **rev/s** | rad/s |
| Current / 电流 | **A**, per axis (`id`, `iq`) | A | A | A | A |
| Torque / 转矩 | **N·m** | A (current proxy) | N·m | N·m | A (current proxy) or V |
| Voltage / 电压 | **V** (`v_bus`, `v_d`, `v_q`) | V | V | V | V |
| Time / 时间 | **s** | s / ms mixed in params | s | s | s |

Conversions / 换算：

```
rad/s   = 2π × turn/s = 2π × rev/s
rad/s   = (eRPM / pole_pairs) × 2π / 60          # eRPM -> mechanical rad/s
eRPM    = rad/s × 60 / (2π) × pole_pairs
N·m     = Kt × A,  Kt = 1.5 × pole_pairs × psi_f   # this library's convention, see PRD D24
```

> **Trap / 陷阱**: `Kt` is *not* interchangeable with `psi_f`. This library's plants and its
> feed-forward use `psi_f` in the voltage equation and `Kt = 1.5·p·psi_f` in the torque equation;
> substituting one for the other is a dimensional error (PRD erratum D24).
> `Kt` 与 `psi_f` **不可互换**：本库的被控对象与前馈在电压方程里用 `psi_f`，在转矩方程里用
> `Kt = 1.5·p·psi_f`；两者互相代入是量纲错误（PRD 勘误 D24）。

## 2. Angle zero and who owns the offset / 角度零点与偏置归属

| Question / 问题 | This library's answer / 本库的答案 |
|---|---|
| Is `angle_mech_rad()` already free of the mounting offset? / 传感器读数是否已减去安装偏置 | **Yes.** The HAL contract says the sensor owns that correction (`include/foc/hal/position.hpp`). `Motor` writes it back through the optional `set_offset_mech_rad()` hook after alignment. / **是**。契约规定由传感器自己拥有该校正；对齐完成后 `Motor` 通过可选的 `set_offset_mech_rad()` 写回 |
| What does alignment resolve? / 对齐能解决什么 | **The electrical offset only, modulo one mechanical pole pitch (`2π/p`).** The rotor parks at the *nearest* equilibrium, so the capture cannot tell which pole it started from. Absolute multi-turn position still needs an index signal. See `core::AlignRoutine` and SIL S10. / **只解决电角度偏置，且只能定到 `2π/p` 的模**。转子停在**最近的**平衡点，因此捕获无法分辨它从哪一极出发。绝对多圈位置仍需索引信号 |
| Does a "position" command mean the same thing everywhere? / "位置"命令是否同义 | **No.** Ours is in the frame the sensor reports (after the offset). moteus/ODrive express position in rev/turn from *their* zero, which is set by their own homing/index procedure. / **不同**。本库用的是传感器汇报的坐标系（已扣偏置）；moteus/ODrive 用它们自己的回零/索引过程建立的零点 |

## 3. dq frame and signs / dq 坐标系与符号

| Item / 项 | This library / 本库 | Note / 说明 |
|---|---|---|
| d axis | rotor flux axis / 转子磁链轴 | `id` along it |
| q axis | 90° ahead of d in the direction of positive rotation / 沿正转方向超前 d 轴 90° | torque-producing axis / 产生转矩的轴 |
| Park convention | `d = α·cosθ + β·sinθ`, `q = −α·sinθ + β·cosθ` | see `src/core/transforms.cpp`; a sign error here shows up as a *negative* `Kt` |
| Torque sign | `Te = 1.5·p·(psi_f·iq + (Ld−Lq)·id·iq)` | positive `iq` → positive torque → positive speed |
| Back-EMF feed-forward | `v_q_ff = +ωe·psi_f` | measured in SIL S6: a wrong sign produced ~0.57 A of positive feedback |
| Current sign | positive current flows **out of** the inverter into the motor / 正电流由逆变器流向电机 | same convention as every driver we benchmarked, but verify with the vendor's own docs before trusting a comparison |

> Before comparing controllers, verify the sign empirically: command a small positive `iq` at
> standstill and confirm the rotor accelerates the way the vendor's documentation says it should.
> **A comparison that begins with an unverified sign convention measures the sign, not the controller.**
> 比对之前先用实验验证符号：静止时给一个小的正 `iq`，确认转子按对方文档的方向加速。
> **从"未验证的符号约定"出发的比对，量到的是符号，而不是控制器。**

## 4. Gains — "kp" means at least four different things / "kp" 至少有四种含义

| Where / 位置 | What "kp" multiplies / kp 乘的是什么 | Unit / 单位 | In this library / 本库对应 |
|---|---|---|---|
| Current loop | current error / 电流误差 | V/A | `core::Pi::Config::kp` |
| Speed loop | speed error / 速度误差 | A/(rad/s) | `speed_pi_config()` output, `kp = wn·J/Kt` |
| Position loop | position error → **velocity** command / 位置误差 → **速度**指令 | 1/s | `PositionP::Config::gain_1_s`, `position_p_config()` (`gain = bandwidth`) |
| Vendor "position gain" | often torque per rad / 常常是每弧度的转矩 | N·m/rad | not used here: our cascade puts the speed loop between position and torque |

**Why this matters / 为什么重要**：a vendor's `kp = 20` and our position gain `20 1/s` are not
comparable numbers even though they are the same digits. Our gain closes a *first-order* position
loop through the speed loop; theirs may command torque directly.
对方的 `kp = 20` 与本库位置增益 `20 1/s` 即便数字相同也不可比：本库的增益通过速度环闭成**一阶**位置
环，对方可能直接给转矩。

## 5. Gain scaling names / 增益缩放的名称

| Our field / 本库字段 | Scales / 缩放 | Vendor equivalent / 对方对应 |
|---|---|---|
| `kp_scale` | position loop gain / 位置环增益 | moteus `kp_scale` (same meaning) |
| `kd_scale` | speed loop proportional gain (the damping) / 速度环比例增益（阻尼） | moteus `kd_scale` |
| `ilimit_scale` | speed loop integral limit / 速度环积分限幅 | moteus `ilimit_scale` |

`0` in all three means **"no override, use the configured value"**, not "zero gain" — a project-wide
rule (0 is only ever a default where 0 is not a legal value). Measured behaviour and its limits are in
`test_sil_s12_compliance.cpp`.
三者的 `0` 都表示**"不覆盖，用配置值"**，不是"零增益"——这是全项目统一的规则（0 只在 0 本身不合法
的地方才表示默认）。实测行为与其边界见 `test_sil_s12_compliance.cpp`。

## 6. Timing and rates / 时序与频率

| Item / 项 | This library / 本库 |
|---|---|
| Control tick period | **given by the caller** (`tick(dt_s)`); the library never invents a time base / **由调用方给出**，库从不自造时基 |
| Decimation | by **elapsed time** (`speed_loop_period_s`, `position_loop_period_s`), not by a tick count, so a change of control frequency keeps the physical period / 按**累计时间**降采样，控制频率变化时物理周期不变 |
| Overrun detection | `Telemetry::max_period_us` — only when the HalPolicy exposes a `clock`; it is an indicator, **never** a scheduling input / 仅当 HAL 暴露 `clock`；它是指示器，**从不**参与调度 |
| Typical vendor rates | VESC `PID_RATE` 25 Hz–10 kHz; SimpleFOC `motion_downsample`; moteus 15–30 kHz — do not compare a control-loop bandwidth across different rates without normalising / 不同控制频率下的带宽不可直接比较，先归一化 |

## 7. Checklist / 检查单

- [ ] Units converted with the formulas in §1, and written down in the comparison.
      按 §1 换算，并把换算写进比对记录。
- [ ] Angle zero and offset ownership identified (§2).
      明确角度零点与偏置归属。
- [ ] Sign convention verified **empirically**, not assumed (§3).
      符号约定**实验**验证，不靠假设。
- [ ] "Gain" resolved to one of the four meanings in §4.
      把"增益"落到 §4 的四种含义之一。
- [ ] Control rates recorded next to every bandwidth number (§6).
      每个带宽数字旁边都记下控制频率。
