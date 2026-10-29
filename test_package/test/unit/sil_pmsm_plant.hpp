// Conan::ImportStart
#pragma once

#include <cmath>

#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

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
};

/**
 * @brief [en] Integrator state of the plant (d-q currents, mechanical speed and angle).
 * @brief [zh] 被控对象的积分状态（d-q 电流、机械转速与机械角度）。
 * @since 0.1.0
 */
struct PlantState final {
    float id_a;       ///< [en] d-axis current in A. [zh] d 轴电流，A。
    float iq_a;       ///< [en] q-axis current in A. [zh] q 轴电流，A。
    float omega_mech; ///< [en] mechanical speed in rad/s. [zh] 机械转速，rad/s。
    float theta_mech; ///< [en] mechanical angle in rad. [zh] 机械角度，rad。
};

/**
 * @brief [en] Parameterised PMSM plant model for software-in-the-loop tests.
 *
 * @brief [zh] 用于 SIL 测试的参数化 PMSM 被控对象模型。
 *
 *     Equations (d-q, SI). Note the back-EMF term uses the rotor flux linkage `psi_f`, not the torque constant: `Kt =
 *     1.5 * pole_pairs * psi_f`, so `psi_f = Kt / (1.5 * pole_pairs)`. Using `Kt` directly in the voltage equation is
 *     dimensionally wrong (PRD erratum D24). 方程（d-q，SI）。注意反电动势项用的是转子磁链 `psi_f`，不是转矩常数： `Kt
 *     = 1.5 * pole_pairs * psi_f`，即 `psi_f = Kt / (1.5 * pole_pairs)`。 在电压方程里直接用 `Kt` 量纲不对（PRD 勘误
 *     D24）。
 *
 *     did/dt = (vd - Rs*id + we*Lq*iq) / Ld diq/dt = (vq - Rs*iq - we*Ld*id - we*psi_f) / Lq Te = 1.5 * p * (psi_f*iq +
 *     (Ld - Lq)*id*iq) dwm/dt = (Te - B*wm - T_load) / J dthm/dt = wm, we = p * wm
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
     * @note [zh] 它存在的理由：没有它就看不出磁钢极性。`+i_d` 增强转子磁链、`-i_d` 削弱它，于是**相同** 的伏秒脉冲会产
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
     * @note [zh] 与 d 轴旋钮分开，因为两者物理上不同：d 轴饱和咬的是磁钢的**上坡**方向，q 轴饱和 咬的是定子自己产生的磁
     *            通。只有前者的模型看起来完整，却对后者完全盲。
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
     * @note [zh] 这不是往轴上贴一项转矩损失——它说明**被传感的**电流不是产生转矩的电流。这个区别 才是重点：转矩损失项扰
     *            动不了观测器，而这一项可以。
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
        float domega;
        float dtheta;
    };

    [[nodiscard]] auto derivative(const PlantState& s) const noexcept -> Derivative;
    [[nodiscard]] static auto offset(const PlantState& s, const Derivative& d, float h) noexcept -> PlantState;

    PlantParams params_;
    PlantState state_;
    core::Dq<float> v_dq_;
    float load_nm_;
    float psi_f_;
    float sat_depth_{0.0F};     ///< [en] fraction of Ld lost at heavy positive i_d; 0 = ideal. [zh] 强正 i_d 下 Ld 的损失比例；0 = 理想。
    float sat_current_a_{1.0F}; ///< [en] current at which that loss is engaged. [zh] 该损失生效的电流。
    float temp_c_{20.0F};       ///< [en] winding temperature, C. [zh] 绕组温度，°C。
    float temp_alpha_{0.0F};    ///< [en] Rs temperature coefficient, 1/K; 0 = no drift. [zh] Rs 温度系数，1/K；0 = 不漂。
    float noise_a_{0.0F};       ///< [en] current-noise peak, A; 0 = ideal sensor. [zh] 电流噪声峰值，A；0 = 理想传感器。
    float qsat_depth_{0.0F};    ///< [en] fraction of Lq lost at heavy positive i_q; 0 = ideal. [zh] 强正 i_q 下 Lq 的损失比例；0 = 理想。
    float qsat_current_a_{1.0F}; ///< [en] current at which that loss is engaged. [zh] 该损失生效的电流。
    float rm_ohm_{0.0F};        ///< [en] iron-loss shunt, ohm; 0 = no iron loss. [zh] 铁损并联电阻，Ω；0 = 无铁损。
    mutable std::uint32_t noise_state_{0x9E3779B9U}; ///< [en] xorshift state; mutable because the noise belongs to the reading. [zh] xorshift 状态；mutable 因为噪声属于“读数”而非状态。
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


auto PmsmPlant::friction_nm(float omega_mech, float driving_nm) const noexcept -> float {
    // Coulomb friction opposes motion; stiction opposes *starting*. The two are different mechanisms and a machine that
    // has only the first one always creeps when the torque is small -- which is exactly the behaviour a stall-start
    // control law has to be tested against, and it cannot be tested against a model that does not have it.
    //
    // 库仑摩擦阻碍**运动**，静摩擦阻碍**起步**。二者是不同机理，而只有前者的机器在小力矩下总会缓慢蠕动——那正是
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
    return (omega_mech > 0.0F) ? params_.t_coulomb_nm : -params_.t_coulomb_nm;
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

inline void PmsmPlant::set_iron_loss(float rm_ohm) noexcept {
    rm_ohm_ = (rm_ohm > 0.0F) ? rm_ohm : 0.0F;
}

inline void PmsmPlant::set_winding_temperature(float temperature_c, float alpha_per_k) noexcept {
    temp_c_ = temperature_c;
    temp_alpha_ = (alpha_per_k > 0.0F) ? alpha_per_k : 0.0F;
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
    // 并联支路跨在励磁支路上，即跨在**内部**电压 `v − Rs·i` 上，稳态时它就是交叉耦合加反电势：
    // d 上是 `−ω·lq·iq`，q 上是 `ω·(ld·id + ψf)`。若改成用 `v − Rs·i` 驱动，分子在稳态恒为零
    // ——电阻什么都不抽，模型静默失效。这个错犯过，并被本测试抓到。
    const float icd = (rm_ohm_ > 0.0F) ? ((-omega_e * lq * s.iq_a) / rm_ohm_) : 0.0F;
    const float icq = (rm_ohm_ > 0.0F)
                          ? (((omega_e * ld * s.id_a) + (omega_e * psi_f_)) / rm_ohm_)
                          : 0.0F;
    const float itd = s.id_a + icd;
    const float itq = s.iq_a + icq;

    const float did = static_cast<float>((v_dq_.d - (rs * itd) + (omega_e * lq * s.iq_a)) / ld);
    const float diq = static_cast<float>(
        (v_dq_.q - (rs * itq) - (omega_e * ld * s.id_a) - (omega_e * psi_f_)) / lq);

    const float torque = kOneAndHalf * poles * ((psi_f_ * s.iq_a) + ((ld - lq) * s.id_a * s.iq_a));
    const float domega =
        static_cast<float>((torque - (params_.b * s.omega_mech) - friction_nm(s.omega_mech, torque) - load_nm_) /
                           params_.j);

    return Derivative{did, diq, domega, s.omega_mech};
}

inline auto PmsmPlant::offset(const PlantState& s, const Derivative& d, float h) noexcept -> PlantState {
    return PlantState{static_cast<float>(s.id_a + (d.did * h)), static_cast<float>(s.iq_a + (d.diq * h)),
                      static_cast<float>(s.omega_mech + (d.domega * h)),
                      static_cast<float>(s.theta_mech + (d.dtheta * h))};
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

    const float w = dt_s / static_cast<float>(6);
    const float two = static_cast<float>(2);
    state_.id_a = static_cast<float>(state_.id_a + (w * (k1.did + (two * (k2.did + k3.did)) + k4.did)));
    state_.iq_a = static_cast<float>(state_.iq_a + (w * (k1.diq + (two * (k2.diq + k3.diq)) + k4.diq)));
    state_.omega_mech =
        static_cast<float>(state_.omega_mech + (w * (k1.domega + (two * (k2.domega + k3.domega)) + k4.domega)));
    state_.theta_mech =
        static_cast<float>(state_.theta_mech + (w * (k1.dtheta + (two * (k2.dtheta + k3.dtheta)) + k4.dtheta)));
}

inline auto PmsmPlant::state() const noexcept -> PlantState { return state_; }

inline auto PmsmPlant::theta_mech_rad() const noexcept -> float { return state_.theta_mech; }

inline auto PmsmPlant::omega_mech_rad_s() const noexcept -> float { return state_.omega_mech; }

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
    // target 的前提。噪声属于**读数**而非 plant 状态，故用 mutable。
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
