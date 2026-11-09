// Conan::ImportStart
#pragma once

#include <cmath>

#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

#include "sil_cogging_table.hpp"

namespace foc {

namespace sil {

using core::Dq;
using core::Uvw;

namespace {

constexpr float kOneAndHalf = 1.5F;

} // namespace

/**
 * @brief [en] Surface-mounted PMSM parameters used by the SIL plant, SI units.
 * @brief [zh] SIL 被控对象使用的表贴式 PMSM 参数，SI 单位。
 * @since 0.1.0
 */
struct PlantParams final {
    float rs;                ///< [en] stator resistance in ohm. [zh] 定子电阻，Ω。
    float ld;                ///< [en] d-axis inductance in H. [zh] d 轴电感，H。
    float lq;                ///< [en] q-axis inductance in H. [zh] q 轴电感，H。
    float kt;                ///< [en] torque constant in N*m/A. [zh] 转矩常数，N·m/A。
    float j;                 ///< [en] rotor inertia in kg*m^2. [zh] 转子惯量，kg·m²。
    float b;                 ///< [en] viscous friction in N*m*s. [zh] 粘性摩擦，N·m·s。
    std::uint8_t pole_pairs; ///< [en] pole pairs. [zh] 极对数。
    /**
     * @brief [en] breakaway (static) torque, N*m. The rotor does not move until the net torque exceeds it, and then it
     *             moves against the Coulomb term instead. 0 = no stiction, which is the behaviour every earlier case
     *             was written against.
     * @brief [zh] 起动（静）摩擦力矩，N·m。净力矩超过它之前转子不动，超过之后改为对抗库仑项。0 = 无静摩擦，也就是此前每
     *             一个用例所对着的行为。
     */
    float t_static_nm{0.0F};
    /**
     * @brief [en] Coulomb (kinetic) friction torque, N*m, opposing motion at any non-zero speed.
     * @brief [zh] 库仑（动）摩擦力矩， N·m，在任何非零速度下阻碍运动。
     */
    float t_coulomb_nm{0.0F};


    /**
     * @brief [en] speed below which the rotor counts as at rest for the stiction decision, rad/s. A small band is
     *             unavoidable: `sign(0)` is not defined, and a model that integrates through zero without one chatters
     *             across it.
     * @brief [zh] 判定"静止"的速度带宽，rad/s。这一带宽不可避免：`sign(0)` 无定义，而一个穿过零点却无带宽的模型会在零点
     *             两侧抖动。
     */
    float omega_eps_rad_s{0.0F};
    /**
     * @brief [en] Stribeck speed, rad/s: the width of the smooth dip between breakaway and kinetic friction. 0 = the
     *             degenerate case every earlier scenario was built on, where the transition is instantaneous.
     * @brief [zh] Stribeck 速度，rad/s：起动摩擦与动摩擦之间那段平滑下凹的宽度。0 = 此前所有场景所在的退化情形，即该过
     *             渡是瞬时的。
     * @note [en] Why this exists. The library's `FrictionComp` is a Coulomb-plus-viscous feedforward, and until now the
     *            plant's friction was Coulomb-plus-viscous too, so any check of the compensator was the model checking
     *            itself. The Stribeck dip is the published mechanism that a velocity-sign feedforward structurally
     *            cannot reach: it has no state and no velocity dependence beyond the viscous term, so whatever friction
     *            remains inside the dip stays. With `0` the moving branch returns exactly `t_coulomb_nm`, so every
     *            existing scenario is bit-for-bit unchanged.
     * @note [zh] 为什么有这一项。库里的 `FrictionComp` 是库仑加粘性前馈，而在此之前 plant 的摩擦也是库仑加粘性——于是对
     *            补偿器的任何检查都是 **模型检查自己** 。Stribeck 下凹是已发表的机理，而一个按速度符号给量的前馈在结构
     *            上够不着它：它没有状态，除粘性项外没有速度依赖，所以下凹里剩下的摩擦一点都消不掉。取 `0` 时运动分支原
     *            样返回 `t_coulomb_nm`，于是所有既有场景逐位不变。
     */
    float stribeck_speed_rad_s{0.0F};
};

/**
 * @brief [en] Integrator state of the plant (d-q currents, mechanical speed and angle).
 * @brief [zh] 被控对象的积分状态（d-q 电流、机械转速与机械角度）。
 * @since 0.1.0
 */
struct PlantState final {
    float id_a;        ///< [en] d-axis current in A. [zh] d 轴电流，A。
    float iq_a;        ///< [en] q-axis current in A. [zh] q 轴电流，A。
    double omega_mech; ///< [en] mechanical speed in rad/s. [zh] 机械转速，rad/s。
    double theta_mech; ///< [en] mechanical angle in rad. [zh] 机械角度，rad。
};

/**
 * @brief [en] Parameterised PMSM plant model for software-in-the-loop tests.
 *
 * @brief [zh] 用于 SIL 测试的参数化 PMSM 被控对象模型。
 *
 *   Equations (d-q, SI). Note the back-EMF term uses the rotor flux linkage `psi_f`, not the torque constant: `Kt = 1.5
 *   * pole_pairs * psi_f`, so `psi_f = Kt / (1.5 * pole_pairs)`. Using `Kt` directly in the voltage equation is
 *   dimensionally wrong (PRD erratum D24). 方程（d-q，SI）。注意反电动势项用的是转子磁链 `psi_f`，不是转矩常数： `Kt =
 *   1.5 * pole_pairs * psi_f`，即 `psi_f = Kt / (1.5 * pole_pairs)`。 在电压方程里直接用 `Kt` 量纲不对（PRD 勘误 D24
 *   ）。
 *
 *   did/dt = (vd - Rs*id + we*Lq*iq) / Ld diq/dt = (vq - Rs*iq - we*Ld*id - we*psi_f) / Lq Te = 1.5 * p * (psi_f*iq +
 *   (Ld - Lq)*id*iq) dwm/dt = (Te - B*wm - T_load) / J dthm/dt = wm, we = p * wm
 *
 * @context [en] Host-side SIL only: never part of the shipped library.
 * @context [zh] 仅 host 侧 SIL：绝不进入发布库。
 * @since 0.1.0
 */
class PmsmPlant final {
  public:
    /**
     * @brief [en] Construct from nameplate parameters.
     * @brief [zh] 用铭牌参数构造。
     * @param params [en] plant parameters; must be physically consistent (Rs, Ld, Lq, J > 0).
     * @param params [zh] 被控对象参数；需物理自洽（Rs、Ld、Lq、J > 0）。
     * @since 0.1.0
     */
    explicit PmsmPlant(PlantParams params) noexcept;

    /**
     * @brief [en] Reset the integrator state and the load torque.
     * @brief [zh] 复位积分状态与负载转矩。
     * @param initial [en] starting state.
     * @param initial [zh] 初始状态。
     * @since 0.1.0
     */
    void reset(PlantState initial) noexcept;

    /**
     * @brief [en] Apply a stator voltage in the rotor frame (hold until changed, like a real inverter between two
     *             control periods).
     * @brief [zh] 在转子系施加定子电压（保持到下次修改，等同真实逆变器在两个控制周期之间）。
     * @param v_dq [en] voltage in V.
     * @param v_dq [zh] 电压，V。
     * @since 0.1.0
     */
    void set_voltage_dq(Dq<float> v_dq) noexcept;

    /**
     * @brief [en] Set the mechanical load torque.
     * @brief [zh] 设置机械负载转矩。
     * @param torque_nm [en] load torque in N*m, opposing the rotation when positive.
     * @param torque_nm [zh] 负载转矩，N·m，正值时阻碍旋转。
     * @since 0.1.0
     */
    void set_load_torque(float torque_nm) noexcept;

    /**
     * @brief [en] The friction torque opposing motion or starting, N*m.
     * @brief [zh] 阻碍运动或起步的摩擦力矩，N·m。
     * @param omega_mech [en] current mechanical speed, rad/s. @param omega_mech
     * @param omega_mech [zh] 当前机械转速，rad/s。
     * @param driving_nm [en] the electromagnetic torque being produced, N*m.
     * @param driving_nm [zh] 正在产生的电磁力矩，N·m。
     * @return [en] the friction torque, signed so that it opposes the drive at rest and the motion otherwise.
     * @return [zh] 摩擦力矩，符号使其在静止时对抗驱动、其余时候对抗运动。
     */
    [[nodiscard]] auto friction_nm(float omega_mech, float driving_nm) const noexcept -> float;

    /**
     * @brief [en] Model d-axis magnetic saturation: `Ld(i_d) = Ld0 * (1 - depth * tanh(i_d / current_a))`. Off by
     *             default (depth = 0), because a nameplate gives one `Ld` at rated current and nothing about the shape
     *             of its collapse -- saturation is a modelling refinement, not a datasheet number.
     * @brief [zh] 模拟 d 轴磁饱和：`Ld(i_d) = Ld0·(1 - depth·tanh(i_d/current_a))`。默认关闭 （depth = 0），因为铭牌只
     *             给额定点的一个 `Ld`，不说它怎么塌——饱和是建模细化，不是铭牌数字。
     * @param depth [en] fractional fall of the incremental inductance at `i_d >> current_a`, 0..1.
     * @param depth [zh] `i_d >> current_a` 时增量电感的下降比例，0..1。
     * @param current_a [en] current at which saturation is substantially engaged, > 0.
     * @param current_a [zh] 饱和明显生效的电流，> 0。
     * @note [en] Why this exists: the magnet polarity is invisible without it. `+i_d` reinforces the rotor flux and
     *            `-i_d` opposes it, so the *same* volt-second pulse produces two different current rises -- which is
     *            the only thing `MagnetPolarityTest` has to measure. The model is a local fit near `+-current_a`, not a
     *            magnetics reference: the `dL/di` term in the voltage equation is deliberately ignored.
     * @note [zh] 它存在的理由：没有它就看不出磁钢极性。`+i_d` 增强转子磁链、`-i_d` 削弱它，于是 **相同** 的伏秒脉冲会产
     *            生两个不同的电流上升——这是 `MagnetPolarityTest` 唯一可测的现象。该模型是 `±current_a` 附近的局部拟合，
     *            不是磁学参考：电压方程里的 `dL/di` 项被故意忽略。
     * @since 0.1.0
     */
    void set_d_axis_saturation(float depth, float current_a) noexcept;

    /**
     * @brief [en] Incremental d-axis inductance at a given d-axis current, H. Equals `params().ld` when saturation is
     *             off.
     * @brief [zh] 给定 d 轴电流下的增量 d 轴电感，H。饱和关闭时等于 `params().ld`。
     * @param id_a [en] d-axis current in A to evaluate at.
     * @param id_a [zh] 求值点的 d 轴电流，A。
     * @since 0.1.0
     */
    [[nodiscard]] auto effective_ld(float id_a) const noexcept -> float;

    /**
     * @brief [en] q-axis saturation: a fraction of `Lq` lost as `i_q` grows.
     * @brief [zh] q 轴饱和：随 `i_q` 增大损失的 `Lq` 比例。
     * @param [en] [in] depth fraction lost at large `i_q`, 0 = ideal. [in] current_a current at which that loss is engaged.
     * @param [zh] 大 `i_q` 下的损失比例，0 = 理想。 该损失生效的电流。
     * @note [en] Separate from the d-axis knob because the two are physically different: d-axis saturation bites the
     *            *uphill* direction of the magnet, q-axis saturation bites the flux produced by the stator. A model
     *            with only the first can look complete while being blind to the second.
     * @note [zh] 与 d 轴旋钮分开，因为两者物理上不同：d 轴饱和咬的是磁钢的 **上坡** 方向，q 轴饱和 咬的是定子自己产生的
     *            磁通。只有前者的模型看起来完整，却对后者完全盲。
     * @since 0.1.0
     */
    void set_q_axis_saturation(float depth, float current_a) noexcept;

    /**
     * @brief [en] The incremental q-axis inductance at `i_q`: H.
     * @brief [zh] `i_q` 处的增量 q 轴电感：H。
     */
    [[nodiscard]] auto effective_lq(float iq_a) const noexcept -> float;

    /**
     * @brief [en] Iron loss as a shunt resistance across the air-gap voltage.
     * @brief [zh] 铁损建模为跨气隙电压的并联电阻。
     * @param [en] [in] rm_ohm shunt resistance, ohm; 0 = ideal (no iron loss).
     * @param [zh] 并联电阻，Ω；0 = 理想（无铁损）。
     * @note [en] This is not a torque loss term bolted onto the shaft -- it is the reason the *sensed* current is not
     *            the current that makes torque. That distinction is the whole point: a torque-loss term cannot perturb
     *            an observer, and this can.
     * @note [zh] 这不是往轴上贴一项转矩损失——它说明 **被传感的** 电流不是产生转矩的电流。这个区别 才是重点：转矩损失项
     *            扰动不了观测器，而这一项可以。
     * @since 0.1.0
     */
    void set_iron_loss(float rm_ohm) noexcept;

    /**
     * @brief [en] The current that actually produces torque, A.
     * @brief [zh] 真正产生转矩的电流，A。
     */
    [[nodiscard]] auto magnetizing_current_dq() const noexcept -> core::Dq<float>;

    /**
     * @brief [en] The current the shunt resistance diverts, A.
     * @brief [zh] 被并联电阻分走的电流，A。
     */
    [[nodiscard]] auto core_loss_current_dq() const noexcept -> core::Dq<float>;

    /**
     * @brief [en] Model winding heating: `Rs = Rs20 * (1 + alpha * (T - 20))`. Off by default.
     * @brief [zh] 模拟绕组发热：`Rs = Rs20·(1 + alpha·(T − 20))`。默认关闭。
     * @param temperature_c [en] winding temperature in degrees C.
     * @param temperature_c [zh] 绕组温度，摄氏度。
     * @param alpha_per_k [en] temperature coefficient, copper is about 3.93e-3 per K.
     * @param alpha_per_k [zh] 温度系数，铜约为3.93e-3 /K。
     * @note [en] Why this exists: a parameter-dentification result that only works at 20 C is a calculator, not an
     *            estimator. A drifting truth is how SIL can test the difference -- the plant heats up while the
     *            observer keeps believing the nameplate.
     * @note [zh] 它存在的理由：只在 20 °C 成立的参数辨识是计算器，不是估计器。真值漂移是 SIL 能测出这个 区别的方式
     *            ——plant 升温而观测器仍然相信铭牌。
     * @since 0.1.0
     */
    void set_winding_temperature(float temperature_c, float alpha_per_k) noexcept;

    /**
     * @brief [en] Parameters of the lumped thermal model: how hot the winding actually gets, not how hot the caller
     *             says it is.
     * @brief [zh] 集总热模型的参数：绕组 **实际** 会有多热，而不是调用方说它多热。
     * @note [en] `set_winding_temperature` above is the *constant* version of this -- a number, with no dynamics. A
     *            derating scheduler cannot be argued with against a constant: the whole claim is about where the
     *            temperature goes, so the plant has to be able to make it go somewhere.
     * @note [zh] 上面的 `set_winding_temperature` 是它的 **常数** 版本——一个数，没有动力学。降额调度无法对着常数论证：
     *            那条主张就是关于温度走向的，所以 plant 必须能让它走。
     * @since 0.1.0
     */
    struct ThermalConfig final {
        float rt_k_per_w{0.0F};   ///< [en] thermal resistance, K/W; <=0 = off. [zh] 热阻，K/W；<=0 = 关闭。
        float ct_j_per_k{0.0F};   ///< [en] thermal capacitance, J/K; <=0 = off. [zh] 热容，J/K；<=0 = 关闭。
        float t_ambient_c{20.0F}; ///< [en] ambient the heat leaks to, C. [zh] 散热到的环境温度，°C。
    };

    /**
     * @brief [en] Turn on a one-node thermal model: `Ct*dT/dt = P_cu - (T - T_amb)/Rt`, off by default.
     *
     * @brief [zh] 打开单节点热模型：`Ct·dT/dt = P_cu − (T − T_amb)/Rt`，默认关闭。
     *
     *   `P_cu = 1.5*(id^2 + iq^2)*Rs(T)` -- the 1.5 is the same amplitude-invariant convention the torque uses, and
     *   `Rs(T)` is this plant's own `effective_rs()`, so the model has the **positive feedback** a real winding has:
     *   hotter means more loss means hotter. That feedback makes the steady state non-linear rather than a
     *   proportionality, and above `Rt*P*alpha = 1` there is no steady state at all -- which is a property worth being
     *   able to see in SIL rather than discovering on a bench.
     *
     *   The step is the **exact** solution of the one-node equation with the loss held at the step's own state (`T <-
     *   T_eq + (T - T_eq)*exp(-dt/tau)`, `tau = Rt*Ct`), not an Euler step: a thermal time constant is seconds while
     *   the electrical step is microseconds, so an explicit form would be stable only by luck.
     *
     *   **Off by default, and bit-identical when off**: with `rt <= 0` or `ct <= 0` nothing is integrated and `temp_c_`
     *   stays exactly what `set_winding_temperature` put there.
     *
     *   `P_cu = 1.5·(id² + iq²)·Rs(T)`——那个 1.5 与转矩用的是同一个幅值不变约定，`Rs(T)` 就是本 plant 自己的
     *   `effective_rs()`，于是模型具备真实绕组具备的 **正反馈** ：更热→损耗更大→更热。这个反馈让稳态是 **非线性** 的而
     *   不是一个比例关系，而且当 `Rt·P·α = 1` 以上 **根本不存在稳态** ——这是一个应该在 SIL 里看得见、而不是在台架上才发
     *   现的性质。
     *
     *   步进用单节点方程的 **精确解** （损耗取该步自身状态）：`T ← T_eq + (T − T_eq)·exp(−dt/τ)`，`τ = Rt·Ct`，而不是欧
     *   拉步：热时间常数是秒、电步长是微秒，显式格式只能靠运气稳定。
     *
     *   **默认关闭，关闭时逐位不变** ：`rt <= 0` 或 `ct <= 0` 时什么都不积分，`temp_c_` 精确停在
     *   `set_winding_temperature` 放进去的那个值。
     *
     * @param config [en] the three parameters above; any non-positive `rt` or `ct` turns the model off.
     * @param config [zh] 上面三个参数；`rt` 或 `ct` 非正即关闭模型。
     * @since 0.1.0
     */
    void set_thermal(ThermalConfig config) noexcept;

    /**
     * @brief [en] Winding temperature the plant is actually at, C -- what `effective_rs()` is using.
     * @brief [zh] plant 实际所处的绕组温度，°C——也就是 `effective_rs()` 正在用的那个。
     * @since 0.1.0
     */
    [[nodiscard]] auto winding_temperature_c() const noexcept -> float;

    /**
     * @brief [en] Copper loss at the current state, W: `1.5*(id^2 + iq^2)*Rs(T)`.
     * @brief [zh] 当前状态下的铜损，W：`1.5·(id² + iq²)·Rs(T)`。
     * @since 0.1.0
     */
    [[nodiscard]] auto copper_loss_w() const noexcept -> float;

    /**
     * @brief [en] Thermal time constant `Rt*Ct` in s; 0 when the model is off.
     * @brief [zh] 热时间常数 `Rt·Ct`，s；模型关闭时为 0。
     * @since 0.1.0
     */
    [[nodiscard]] auto thermal_time_constant_s() const noexcept -> float;

    /**
     * @brief [en] Add sensor noise to the current readings: white, uniform in [-amplitude, +amplitude] amps, from a
     *             deterministic integer generator so a run is reproducible on any ISA. Off by default (amplitude 0).
     * @brief [zh] 给电流读数加传感器噪声：均匀分布在 [−amplitude, +amplitude] 安培，由确定性整数生成器 产生，使同一次运
     *             行在任何 ISA 上都可复现。默认关闭（幅值 0）。
     * @param amplitude_a [en] peak amplitude in A, >= 0.
     * @param amplitude_a [zh] 峰值幅值，A，>= 0。
     * @note [en] Determinism is not a nicety here: the PIL leg compares host against target, and an entropy-based
     *            generator would make that comparison meaningless the moment noise is on.
     * @note [zh] 确定性不是细节：PIL 腿要比对 host 与 target，一旦打开噪声，基于熵的生成器会让这个比对 失去意义。
     * @since 0.1.0
     */
    void set_current_noise(float amplitude_a) noexcept;

    /**
     * @brief [en] Give the machine a Stribeck dip of this width; 0 restores the instantaneous transition.
     * @brief [zh] 给机器一个该宽度的 Stribeck 下凹；0 恢复瞬时过渡。
     */
    void set_stribeck_speed(float speed_rad_s) noexcept;

    /**
     * @brief [en] Inject a cogging torque into the mechanical equation, as a table of harmonics (order per
     *             **mechanical** revolution, `amplitude * sin(order*theta_mech + phase)`, N*m).
     *
     * @brief [zh] 把齿槽转矩以谐波表注入机械方程（次数按 **每机械转** ，`amplitude · sin(order·θ_mech + phase)`，N·m
     *             ）。
     *
     *   Additive on the rotor: `J*dwm/dt = (Te + Tcog - B*wm - friction - load) / J`, and it is handed to the friction
     *   model as part of the driving torque because that is what a detent is -- at rest, cogging alone can move the
     *   rotor. Writing it as a load would have been the same arithmetic with the sign left to the compensator's author
     *   to rediscover.
     *
     *   **Off by default, and bit-identical when off**: no table means no term at all, so every scenario that predates
     *   this port integrates exactly what it integrated before. The mechanism is shared with `AbcPlant` through
     *   `sil_cogging_table.hpp`, so the two models cannot disagree about what a given table means. 加性作用在转子上：
     *   `J·dωm/dt = (Te + Tcog − B·ωm − friction − load)/J`，并且它作为驱动力矩的一部分交给摩擦模型 —— 因为定位点本来就
     *   是这样：静止时齿槽单独就能推动转子。写成"负载"是同一份算术，只是把符号留给写补偿的人重新发现。
     *   **默认关闭，关闭时逐位不变** ：没有表就完全没有这一项，所以本端口之前的所有场景积分的仍是原来那个东西。机制经
     *   `sil_cogging_table.hpp` 与 `AbcPlant` 共享，因此两台模型对同一张表的含义不可能分歧。
     *
     * @param harmonics [en] table, at most `kMaxCoggingHarmonics` entries; null clears.
     * @param harmonics [zh] 表，最多 `kMaxCoggingHarmonics` 条；空指针表示清空。
     * @param count [en] how many entries are valid.
     * @param count [zh] 有效条数。
     * @since 0.1.0
     */
    void set_cogging(const CoggingHarmonic* harmonics, int count) noexcept;

    /**
     * @brief [en] The injected cogging torque at the current mechanical angle, N*m -- the truth a compensator is
     *             measured against, and the only number that says what an injection of zero should have done.
     * @brief [zh] 当前机械角处的注入齿槽转矩，N·m——补偿器被对照的那把尺，也是"注入为零时应当如何"的唯一答案。
     * @since 0.1.0
     */
    [[nodiscard]] auto cogging_torque_nm() const noexcept -> float;

    /** @brief [en] How many harmonics are active. [zh] 当前生效的谐波条数。 */
    [[nodiscard]] auto cogging_harmonics() const noexcept -> int;

    /**
     * @brief [en] Stator resistance the plant is actually using, ohm (includes the temperature model).
     * @brief [zh] plant 实际使用的定子电阻，Ω（含温度模型）。
     * @since 0.1.0
     */
    [[nodiscard]] auto effective_rs() const noexcept -> float;

    /**
     * @brief [en] Advance the plant by one fixed step with classical RK4.
     * @brief [zh] 用经典 RK4 推进一个固定步长。
     * @param dt_s [en] step in s; <= 0 is ignored.
     * @param dt_s [zh] 步长，s；<= 0 被忽略。
     * @since 0.1.0
     */
    void advance(float dt_s) noexcept;

    /**
     * @brief [en] Current integrator state.
     * @brief [zh] 当前积分状态。
     * @since 0.1.0
     */
    [[nodiscard]] auto state() const noexcept -> PlantState;

    /**
     * @brief [en] Mechanical rotor angle in rad (unwrapped).
     * @brief [zh] 机械转子角度，rad（不做归一化）。
     * @since 0.1.0
     */
    [[nodiscard]] auto theta_mech_rad() const noexcept -> float;

    /**
     * @brief [en] Mechanical rotor speed in rad/s.
     * @brief [zh] 机械转速，rad/s。
     * @since 0.1.0
     */
    [[nodiscard]] auto omega_mech_rad_s() const noexcept -> float;

    /**
     * @brief [en] Override the mechanical speed and leave the electrical state alone -- a dyno bench. The torque
     *             equation keeps running and simply has nothing to say: with a large inertia the torque cannot move the
     *             speed between two calls, so the speed is what the caller put there.
     * @brief [zh] 覆盖机械转速、不动电学状态——测功机台架。转矩方程照常运行，只是无话可说：惯量取大之后转矩在两次调用之
     *             间推不动转速，于是转速就是调用方放进去的那个值。
     * @note [en] Not `reset()`: that rewrites the currents too, which throws away the intra-period high-frequency
     *            content -- the carrier itself. A rig that called `reset()` every step measured an observer with no HF
     *            current at all and reported it as "unlocked", which is how this member came to exist.
     * @note [zh] 不是 `reset()`：那会把电流一并重写，从而丢掉周期内的高频内容——也就是载波本身。每一步都调 `reset()` 的
     *            台架量到的是一个没有高频电流的观测器，并把它报成"未锁定"，这个成员就是这么来的。
     */
    void set_mechanical_speed(float omega_mech_rad_s) noexcept;

    /**
     * @brief [en] Phase currents reconstructed through inverse Park + inverse Clarke, i.e. what a current sensor would
     *             see. This is the signal the SIL harness feeds to the HAL stub.
     * @brief [zh] 经逆 Park + 逆 Clarke 重构的相电流，即电流传感器"看到"的信号。SIL 夹具把它喂给 HAL 桩。
     * @since 0.1.0
     */
    [[nodiscard]] auto current_uvw() const noexcept -> core::Uvw<float>;

    /**
     * @brief [en] d-q currents in A.
     * @brief [zh] d-q 电流，A。
     * @since 0.1.0
     */
    [[nodiscard]] auto current_dq() const noexcept -> core::Dq<float>;

    /**
     * @brief [en] The d-q voltage this plant is currently integrating, V.
     * @brief [zh] 本 plant 当前正在积分的 d-q 电压，V。
     * @note [en] A read-out, not an input: it is what the last `set_voltage_dq()` was given. It exists because the bus
     *            current is `1.5*(v_d*i_d + v_q*i_q)/v_bus` and a caller that has to keep its own copy of the voltage
     *            can pair it with a current from a different instant -- which is the one mistake that makes a power
     *            measurement silently wrong instead of loudly wrong.
     * @note [zh] 这是一个读数而不是输入：它就是上一次 `set_voltage_dq()` 被给到的值。它存在的理由是母线电流是
     *            `1.5*(v_d*i_d + v_q*i_q)/v_bus`，而一个必须自己另存一份电压的调用方，可能把电压和 **另一个时刻** 的电
     *            流配在一起——那正是让功率测量"悄悄错"而不是"明显错"的那个错误。
     * @since 0.1.0
     */
    [[nodiscard]] auto voltage_dq() const noexcept -> core::Dq<float>;

    /**
     * @brief [en] Electromagnetic torque in N*m.
     * @brief [zh] 电磁转矩，N·m。
     * @since 0.1.0
     */
    [[nodiscard]] auto torque_nm() const noexcept -> float;

    /**
     * @brief [en] Rotor flux linkage derived from the torque constant.
     * @brief [zh] 由转矩常数反推的转子磁链。
     * @since 0.1.0
     */
    [[nodiscard]] auto psi_f_wb() const noexcept -> float;

    /**
     * @brief [en] Nameplate parameters of this plant.
     * @brief [zh] 本 plant 的铭牌参数。
     */
    [[nodiscard]] auto params() const noexcept -> const PlantParams&;

  private:
    struct Derivative final {
        float did;
        float diq;
        double domega;
        double dtheta;
    };

    [[nodiscard]] auto derivative(const PlantState& s) const noexcept -> Derivative;
    [[nodiscard]] static auto offset(const PlantState& s, const Derivative& d, float h) noexcept -> PlantState;

    PlantParams params_;
    PlantState state_;
    core::Dq<float> v_dq_;
    float load_nm_;
    float psi_f_;
    float sat_depth_{
        0.0F}; ///< [en] fraction of Ld lost at heavy positive i_d; 0 = ideal. [zh] 强正 i_d 下 Ld 的损失比例；0 = 理想。
    float sat_current_a_{1.0F}; ///< [en] current at which that loss is engaged. [zh] 该损失生效的电流。
    /**
     * @brief [en] Winding temperature, C -- carried in `double` **because of how the node is integrated**, not for
     *             precision in general.
     *
     * @brief [zh] 绕组温度，°C——用 `double` **是因为节点的积分方式** ，不是一般意义上的精度考虑。
     *
     *   The exact exponential step adds `(T_eq - T)*(1 - exp(-dt/tau))` per step, which for a seconds-long thermal
     *   constant against a microsecond electrical step is about `1.25e-5` of the remaining gap. In `float`, one ulp at
     *   26 C is `1.9e-6`, so the update stops being representable while the gap is still **0.076 K** -- measured, and
     *   the node then sits at 25.92 C forever against a `T_eq` of 26.00. Nothing about that stall looks like a bug in a
     *   plot; it looks like a different (and plausible) steady state.
     *
     *   精确指数步每步加 `(T_eq − T)·(1 − exp(−dt/τ))`，对"秒级热时间常数配微秒级电学步长"来说约是剩余间隙的
     *   `1.25e-5`。在 `float` 里 26 °C 处一个 ulp 是 `1.9e-6`，于是当间隙还剩 **0.076 K** 时这个增量就已经无法表示—— 实
     *   测值，节点自此永远停在 25.92 °C，而 `T_eq` 是 26.00。这个停滞在图上完全不像 bug，它像一个不同的（而且合理的）稳
     *   态。
     */
    double temp_c_{20.0};
    float temp_alpha_{0.0F};  ///< [en] Rs temperature coefficient, 1/K; 0 = no drift. [zh] Rs 温度系数，1/K；0 = 不漂。
    ThermalConfig thermal_{}; ///< [en] one-node thermal model; off by default. [zh] 单节点热模型；默认关闭。
    float noise_a_{0.0F};     ///< [en] current-noise peak, A; 0 = ideal sensor. [zh] 电流噪声峰值，A；0 = 理想传感器。
    float qsat_depth_{
        0.0F}; ///< [en] fraction of Lq lost at heavy positive i_q; 0 = ideal. [zh] 强正 i_q 下 Lq 的损失比例；0 = 理想。
    float qsat_current_a_{1.0F}; ///< [en] current at which that loss is engaged. [zh] 该损失生效的电流。
    float rm_ohm_{0.0F};         ///< [en] iron-loss shunt, ohm; 0 = no iron loss. [zh] 铁损并联电阻，Ω；0 = 无铁损。
    CoggingHarmonic
        cogging_[kMaxCoggingHarmonics]{}; ///< [en] injected cogging, empty = off. [zh] 注入齿槽，空 = 关闭。
    int cogging_count_{0};                ///< [en] how many are active. [zh] 生效条数。
    mutable std::uint32_t noise_state_{
        0x9E3779B9U}; ///< [en] xorshift state; mutable because the noise belongs to the reading. [zh] xorshift 状态；mutable 因为噪声属于“读数”而非状态。
};


// Implementations are inline: the test harness compiles every test/unit/*.cpp as its own
// executable, so a plant with a separate .cpp would not link. Header-only keeps the SIL plant a
// single self-contained translation unit against the tests.
// 实现内联：测试框架把每个 test/unit/*.cpp 单独编成可执行文件，plant 若放独立 .cpp 会链不上。
// 头文件实现让 SIL plant 相对测试是自包含的。

inline PmsmPlant::PmsmPlant(PlantParams params) noexcept
    : params_(params), state_{0.0F, 0.0F, 0.0F, 0.0F}, v_dq_{0.0F, 0.0F}, load_nm_(0.0F), psi_f_(0.0F) {
    const float poles = static_cast<float>(params_.pole_pairs);
    psi_f_ = (poles > 0.0F) ? (params_.kt / (kOneAndHalf * poles)) : 0.0F;
}

inline void PmsmPlant::reset(PlantState initial) noexcept {
    state_ = initial;
    v_dq_ = core::Dq<float>{0.0F, 0.0F};
    load_nm_ = 0.0F;
}

inline void PmsmPlant::set_voltage_dq(core::Dq<float> v_dq) noexcept { v_dq_ = v_dq; }

inline void PmsmPlant::set_load_torque(float torque_nm) noexcept { load_nm_ = torque_nm; }

inline void PmsmPlant::set_cogging(const CoggingHarmonic* harmonics, int count) noexcept {
    cogging_count_ = store_cogging(cogging_, kMaxCoggingHarmonics, harmonics, count);
}

inline auto PmsmPlant::cogging_torque_nm() const noexcept -> float {
    return evaluate_cogging(cogging_, cogging_count_, state_.theta_mech);
}

inline auto PmsmPlant::cogging_harmonics() const noexcept -> int { return cogging_count_; }


auto PmsmPlant::friction_nm(float omega_mech, float driving_nm) const noexcept -> float {
    // Coulomb friction opposes motion; stiction opposes *starting*. The two are different mechanisms and a machine that
    // has only the first one always creeps when the torque is small -- which is exactly the behaviour a stall-start
    // control law has to be tested against, and it cannot be tested against a model that does not have it.
    //
    // 库仑摩擦阻碍 **运动** ，静摩擦阻碍 **起步** 。二者是不同机理，而只有前者的机器在小力矩下总会缓慢蠕动——那正是
    // "带载起动"控制律必须被检验的行为，而一个没有该现象的模型检验不出来。
    const float net_at_rest = driving_nm - load_nm_;
    if (std::fabs(omega_mech) <= params_.omega_eps_rad_s) {
        // At rest the drive must beat the breakaway torque before anything moves. Below it the friction term cancels
        // the drive exactly, so the acceleration is zero and the rotor stays put instead of creeping.
        // 静止时驱动必须超过起动摩擦才会动。低于它时摩擦项正好抵消驱动，加速度为零，转子保持不动而不是缓慢蠕动。
        if (std::fabs(net_at_rest) <= params_.t_static_nm) {
            return net_at_rest;
        }
        return (net_at_rest > 0.0F) ? params_.t_coulomb_nm : -params_.t_coulomb_nm;
    }
    // Moving. Without a Stribeck width this is the constant kinetic value every earlier scenario was built on; with one,
    // the level rises smoothly toward breakaway as the speed falls, which is the dip a sign-based feedforward cannot undo.
    // 运动中。没有 Stribeck 宽度时就是此前所有场景赖以建立的那个常数动摩擦；有宽度时，随速度下降平滑抬向起动值，
    // 这正是按符号给量的前馈消不掉的那个下凹。
    const float speed = std::fabs(omega_mech);
    float level = params_.t_coulomb_nm;
    if (params_.stribeck_speed_rad_s > 0.0F) {
        const float ratio = speed / params_.stribeck_speed_rad_s;
        level += (params_.t_static_nm - params_.t_coulomb_nm) * std::exp(-ratio * ratio);
    }
    return (omega_mech > 0.0F) ? level : -level;
}

inline void PmsmPlant::set_mechanical_speed(float omega_mech_rad_s) noexcept { state_.omega_mech = omega_mech_rad_s; }

inline void PmsmPlant::set_d_axis_saturation(float depth, float current_a) noexcept {
    sat_depth_ = (depth > 0.0F) ? ((depth < 1.0F) ? depth : 1.0F) : 0.0F;
    sat_current_a_ = (current_a > 0.0F) ? current_a : 1.0F;
}

inline auto PmsmPlant::effective_ld(float id_a) const noexcept -> float {
    if (!(sat_depth_ > 0.0F)) {
        return params_.ld;
    }
    return static_cast<float>(params_.ld * (1.0F - (sat_depth_ * std::tanh(id_a / sat_current_a_))));
}

inline void PmsmPlant::set_q_axis_saturation(float depth, float current_a) noexcept {
    qsat_depth_ = (depth > 0.0F) ? ((depth < 1.0F) ? depth : 1.0F) : 0.0F;
    qsat_current_a_ = (current_a > 0.0F) ? current_a : 1.0F;
}

inline auto PmsmPlant::effective_lq(float iq_a) const noexcept -> float {
    if (!(qsat_depth_ > 0.0F)) {
        return params_.lq;
    }
    return static_cast<float>(params_.lq * (1.0F - (qsat_depth_ * std::tanh(iq_a / qsat_current_a_))));
}

inline void PmsmPlant::set_iron_loss(float rm_ohm) noexcept { rm_ohm_ = (rm_ohm > 0.0F) ? rm_ohm : 0.0F; }

inline void PmsmPlant::set_winding_temperature(float temperature_c, float alpha_per_k) noexcept {
    temp_c_ = temperature_c;
    temp_alpha_ = (alpha_per_k > 0.0F) ? alpha_per_k : 0.0F;
}

inline void PmsmPlant::set_thermal(ThermalConfig config) noexcept {
    thermal_.rt_k_per_w = (config.rt_k_per_w > 0.0F) ? config.rt_k_per_w : 0.0F;
    thermal_.ct_j_per_k = (config.ct_j_per_k > 0.0F) ? config.ct_j_per_k : 0.0F;
    thermal_.t_ambient_c = config.t_ambient_c;
}

inline auto PmsmPlant::thermal_time_constant_s() const noexcept -> float {
    if ((!(thermal_.rt_k_per_w > 0.0F)) || (!(thermal_.ct_j_per_k > 0.0F))) {
        return 0.0F;
    }
    return thermal_.rt_k_per_w * thermal_.ct_j_per_k;
}

inline auto PmsmPlant::winding_temperature_c() const noexcept -> float { return static_cast<float>(temp_c_); }

inline auto PmsmPlant::copper_loss_w() const noexcept -> float {
    // `1.5` for the same reason the torque carries it: a d-q amplitude-invariant pair of currents whose three phase
    // currents each carry that much loss. Dropping it would make the machine look 33 % cooler than it is.
    // `1.5` 的理由与转矩相同：幅值不变的 d-q 电流对，三相各自带走这么多损耗。漏掉它会让机器看起来比实际凉 33 %。
    return kOneAndHalf * (((state_.id_a * state_.id_a) + (state_.iq_a * state_.iq_a)) * effective_rs());
}

inline void PmsmPlant::set_stribeck_speed(float speed_rad_s) noexcept {
    params_.stribeck_speed_rad_s = (speed_rad_s > 0.0F) ? speed_rad_s : 0.0F;
}

inline void PmsmPlant::set_current_noise(float amplitude_a) noexcept {
    noise_a_ = (amplitude_a > 0.0F) ? amplitude_a : 0.0F;
    noise_state_ = 0x9E3779B9U; // a fixed seed, so "same run" stays "same numbers"
}

auto PmsmPlant::effective_rs() const noexcept -> float {
    if (!(temp_alpha_ > 0.0F)) {
        return params_.rs;
    }
    return static_cast<float>(params_.rs * (1.0F + (temp_alpha_ * (temp_c_ - 20.0F))));
}

inline auto PmsmPlant::derivative(const PlantState& s) const noexcept -> Derivative {
    const float poles = static_cast<float>(params_.pole_pairs);
    const float omega_e = poles * s.omega_mech;
    // The incremental inductance at the current operating point. Without saturation this is exactly
    // `params_.ld`, so every scenario that predates this model is bit-for-bit unchanged.
    // 工作点处的增量电感。没有饱和时它就是 `params_.ld`，于是本模型之前的所有场景逐位不变。
    const float ld = effective_ld(s.id_a);
    // Same idea for the resistance: without a temperature model this is exactly `params_.rs`.
    // 电阻同理：没有温度模型时它就是 `params_.rs`。
    const float rs = effective_rs();

    const float lq = effective_lq(s.iq_a);
    // The shunt diverts current from the air-gap voltage. It is evaluated on the magnetizing current
    // (not the terminal one) so the flux equation stays acyclic; the dropped term is of order
    // `Rs/Rm` times the core current, i.e. 1e-5 here.
    // 并联支路从气隙电压取电流。用励磁电流（而非端电流）求值，好让磁链方程保持无环；丢掉的项是
    // `Rs/Rm` 乘铁损电流的量级，在这里是 1e-5。
    // The shunt sits across the magnetizing branch, i.e. across the *internal*
    // voltage `v - Rs*i`, which at steady state is the cross-coupling plus the
    // back-EMF: `-omega*lq*iq` on d and `omega*(ld*id + psi_f)` on q. Driving it
    // from `v - Rs*i` instead would give a numerator that is identically zero at
    // steady state -- the resistor would draw nothing and the model would be
    // silently inert. That mistake was made and caught by this test.
    // 并联支路跨在励磁支路上，即跨在 **内部** 电压 `v − Rs·i` 上，稳态时它就是交叉耦合加反电势：
    // d 上是 `−ω·lq·iq`，q 上是 `ω·(ld·id + ψf)`。若改成用 `v − Rs·i` 驱动，分子在稳态恒为零
    // ——电阻什么都不抽，模型静默失效。这个错犯过，并被本测试抓到。
    const float icd = (rm_ohm_ > 0.0F) ? ((-omega_e * lq * s.iq_a) / rm_ohm_) : 0.0F;
    const float icq = (rm_ohm_ > 0.0F) ? (((omega_e * ld * s.id_a) + (omega_e * psi_f_)) / rm_ohm_) : 0.0F;
    const float itd = s.id_a + icd;
    const float itq = s.iq_a + icq;

    const float did = static_cast<float>((v_dq_.d - (rs * itd) + (omega_e * lq * s.iq_a)) / ld);
    const float diq = static_cast<float>((v_dq_.q - (rs * itq) - (omega_e * ld * s.id_a) - (omega_e * psi_f_)) / lq);

    const float torque = kOneAndHalf * poles * ((psi_f_ * s.iq_a) + ((ld - lq) * s.id_a * s.iq_a));
    // The injected cogging torque, evaluated at the RK4 stage's own angle so the integrator sees its slope rather than a
    // value frozen at the start of the step. `friction_nm` receives the *driving* torque including cogging, because at
    // rest it is cogging that decides whether the rotor is held or moves -- that is what a detent is.
    // 注入的齿槽转矩在 RK4 各阶 **各自的** 角度上求值，好让积分器看到它的斜率而不是一步步冻结的值。`friction_nm` 收到的是
    // **含齿槽** 的驱动力矩，因为静止时决定转子被卡住还是动起来的正是齿槽——定位点就是这个意思。
    const float tcog = evaluate_cogging(cogging_, cogging_count_, static_cast<float>(s.theta_mech));
    const float driving = torque + tcog;
    const double domega =
        (static_cast<double>(driving) - (static_cast<double>(params_.b) * s.omega_mech) -
         static_cast<double>(friction_nm(static_cast<float>(s.omega_mech), driving)) - static_cast<double>(load_nm_)) /
        static_cast<double>(params_.j);

    return Derivative{did, diq, domega, s.omega_mech};
}

inline auto PmsmPlant::offset(const PlantState& s, const Derivative& d, float h) noexcept -> PlantState {
    const auto hd = static_cast<double>(h);
    return PlantState{static_cast<float>(s.id_a + (d.did * h)), static_cast<float>(s.iq_a + (d.diq * h)),
                      s.omega_mech + (d.domega * hd), s.theta_mech + (d.dtheta * hd)};
}

inline void PmsmPlant::advance(float dt_s) noexcept {
    if (!(dt_s > 0.0F)) {
        return;
    }

    const float half = dt_s / static_cast<float>(2);
    const Derivative k1 = derivative(state_);
    const Derivative k2 = derivative(offset(state_, k1, half));
    const Derivative k3 = derivative(offset(state_, k2, half));
    const Derivative k4 = derivative(offset(state_, k3, dt_s));

    const double w = static_cast<double>(dt_s) / static_cast<double>(6);
    const double two = 2.0;
    state_.id_a = static_cast<float>(
        static_cast<double>(state_.id_a) +
        (w * ((static_cast<double>(k1.did) + (two * (static_cast<double>(k2.did) + static_cast<double>(k3.did)))) +
              static_cast<double>(k4.did))));
    state_.iq_a = static_cast<float>(
        static_cast<double>(state_.iq_a) +
        (w * ((static_cast<double>(k1.diq) + (two * (static_cast<double>(k2.diq) + static_cast<double>(k3.diq)))) +
              static_cast<double>(k4.diq))));
    state_.omega_mech = state_.omega_mech + (w * ((k1.domega + (two * (k2.domega + k3.domega))) + k4.domega));
    state_.theta_mech = state_.theta_mech + (w * ((k1.dtheta + (two * (k2.dtheta + k3.dtheta))) + k4.dtheta));

    // The thermal node is stepped by its exact solution rather than by RK4: a winding time constant is seconds against
    // a microsecond electrical step, so the stiffness is in the ratio. The loss is evaluated once per step at the state
    // the electrical integration has just produced, and `T_eq` is where this step's loss alone would settle.
    // 热节点用精确解推进而不是 RK4：绕组时间常数是秒，电学步长是微秒，刚度全在这个比值里。损耗每步在电学积分刚产出的
    // 状态上求一次，`T_eq` 是「只有这一步的损耗」会收敛到的温度。
    const float tau = thermal_time_constant_s();
    if (tau > 0.0F) {
        const double t_eq = static_cast<double>(thermal_.t_ambient_c) +
                            (static_cast<double>(thermal_.rt_k_per_w) * static_cast<double>(copper_loss_w()));
        temp_c_ = t_eq + ((temp_c_ - t_eq) * std::exp(-static_cast<double>(dt_s) / static_cast<double>(tau)));
    }
}

inline auto PmsmPlant::state() const noexcept -> PlantState { return state_; }

inline auto PmsmPlant::theta_mech_rad() const noexcept -> float { return static_cast<float>(state_.theta_mech); }

inline auto PmsmPlant::omega_mech_rad_s() const noexcept -> float { return static_cast<float>(state_.omega_mech); }

inline auto PmsmPlant::magnetizing_current_dq() const noexcept -> core::Dq<float> {
    return core::Dq<float>{state_.id_a, state_.iq_a};
}

inline auto PmsmPlant::core_loss_current_dq() const noexcept -> core::Dq<float> {
    if (!(rm_ohm_ > 0.0F)) {
        return core::Dq<float>{0.0F, 0.0F};
    }
    const float poles = static_cast<float>(params_.pole_pairs);
    const float omega_e = poles * state_.omega_mech;
    const float ld = effective_ld(state_.id_a);
    const float lq = effective_lq(state_.iq_a);
    return core::Dq<float>{(-omega_e * lq * state_.iq_a) / rm_ohm_,
                           ((omega_e * ld * state_.id_a) + (omega_e * psi_f_)) / rm_ohm_};
}

inline auto PmsmPlant::voltage_dq() const noexcept -> core::Dq<float> { return v_dq_; }

inline auto PmsmPlant::current_dq() const noexcept -> core::Dq<float> {
    const core::Dq<float> core = core_loss_current_dq();
    return core::Dq<float>{state_.id_a + core.d, state_.iq_a + core.q};
}

inline auto PmsmPlant::current_uvw() const noexcept -> core::Uvw<float> {
    const float theta_e = static_cast<float>(params_.pole_pairs) * state_.theta_mech;
    const core::Dq<float> sensed = current_dq();
    const core::Uvw<float> true_current = core::inv_clarke(core::inv_park(sensed, theta_e));
    if (!(noise_a_ > 0.0F)) {
        return true_current;
    }
    // xorshift32 with integer arithmetic only: the same sequence on every ISA, which is what lets the
    // PIL leg compare host against target with noise switched on. The noise belongs to the *reading*,
    // not to the plant state, hence `mutable`.
    // 只用整数运算的 xorshift32：在任何 ISA 上序列都相同，这正是 PIL 腿可以在打开噪声后比对 host 与
    // target 的前提。噪声属于 **读数** 而非 plant 状态，故用 mutable。
    const auto next_sample = [this]() noexcept -> float {
        noise_state_ ^= noise_state_ << 13U;
        noise_state_ ^= noise_state_ >> 17U;
        noise_state_ ^= noise_state_ << 5U;
        // Top 24 bits scaled by 2^-23 gives [-1, 1) exactly; both factors are powers of two.
        // 取高 24 位再乘 2^-23 精确得到 [−1, 1)；两个因子都是 2 的幂。
        return (static_cast<float>(noise_state_ >> 8U) * (1.0F / 8388608.0F)) - 1.0F;
    };
    return core::Uvw<float>{static_cast<float>(true_current.u + (noise_a_ * next_sample())),
                            static_cast<float>(true_current.v + (noise_a_ * next_sample())),
                            static_cast<float>(true_current.w + (noise_a_ * next_sample()))};
}

inline auto PmsmPlant::torque_nm() const noexcept -> float {
    const float poles = static_cast<float>(params_.pole_pairs);
    const float ld = effective_ld(state_.id_a);
    const float lq = effective_lq(state_.iq_a);
    return kOneAndHalf * poles * ((psi_f_ * state_.iq_a) + ((ld - lq) * state_.id_a * state_.iq_a));
}

inline auto PmsmPlant::psi_f_wb() const noexcept -> float { return psi_f_; }

inline auto PmsmPlant::params() const noexcept -> const PlantParams& { return params_; }

} // namespace sil

} // namespace foc
