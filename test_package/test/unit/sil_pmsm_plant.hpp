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
 *        [zh] SIL 被控对象使用的表贴式 PMSM 参数，SI 单位。
 *
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
};

/**
 * @brief [en] Integrator state of the plant (d-q currents, mechanical speed and angle).
 *        [zh] 被控对象的积分状态（d-q 电流、机械转速与机械角度）。
 *
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
 *        [zh] 用于 SIL 测试的参数化 PMSM 被控对象模型。
 *
 *        Equations (d-q, SI). Note the back-EMF term uses the rotor flux linkage `psi_f`, not the
 *        torque constant: `Kt = 1.5 * pole_pairs * psi_f`, so `psi_f = Kt / (1.5 * pole_pairs)`.
 *        Using `Kt` directly in the voltage equation is dimensionally wrong (PRD erratum D24).
 *        方程（d-q，SI）。注意反电动势项用的是转子磁链 `psi_f`，不是转矩常数：
 *        `Kt = 1.5 * pole_pairs * psi_f`，即 `psi_f = Kt / (1.5 * pole_pairs)`。
 *        在电压方程里直接用 `Kt` 量纲不对（PRD 勘误 D24）。
 *
 *        did/dt  = (vd - Rs*id + we*Lq*iq) / Ld
 *        diq/dt  = (vq - Rs*iq - we*Ld*id - we*psi_f) / Lq
 *        Te      = 1.5 * p * (psi_f*iq + (Ld - Lq)*id*iq)
 *        dwm/dt  = (Te - B*wm - T_load) / J
 *        dthm/dt = wm,   we = p * wm
 *
 * @context [en] Host-side SIL only: never part of the shipped library.
 *           [zh] 仅 host 侧 SIL：绝不进入发布库。
 * @since 0.1.0
 */
class PmsmPlant final {
  public:
    /**
     * @brief [en] Construct from nameplate parameters.
     *        [zh] 用铭牌参数构造。
     *
     * @param params [en] plant parameters; must be physically consistent (Rs, Ld, Lq, J > 0).
     *               [zh] 被控对象参数；需物理自洽（Rs、Ld、Lq、J > 0）。
     * @since 0.1.0
     */
    explicit PmsmPlant(PlantParams params) noexcept;

    /**
     * @brief [en] Reset the integrator state and the load torque.
     *        [zh] 复位积分状态与负载转矩。
     *
     * @param initial [en] starting state. [zh] 初始状态。
     * @since 0.1.0
     */
    void reset(PlantState initial) noexcept;

    /**
     * @brief [en] Apply a stator voltage in the rotor frame (hold until changed, like a real
     *        inverter between two control periods).
     *        [zh] 在转子系施加定子电压（保持到下次修改，等同真实逆变器在两个控制周期之间）。
     *
     * @param v_dq [en] voltage in V. [zh] 电压，V。
     * @since 0.1.0
     */
    void set_voltage_dq(Dq<float> v_dq) noexcept;

    /**
     * @brief [en] Set the mechanical load torque.
     *        [zh] 设置机械负载转矩。
     *
     * @param torque_nm [en] load torque in N*m, opposing the rotation when positive.
     *                  [zh] 负载转矩，N·m，正值时阻碍旋转。
     * @since 0.1.0
     */
    void set_load_torque(float torque_nm) noexcept;

    /**
     * @brief [en] Model d-axis magnetic saturation: `Ld(i_d) = Ld0 * (1 - depth * tanh(i_d / current_a))`.
     *        Off by default (depth = 0), because a nameplate gives one `Ld` at rated current and
     *        nothing about the shape of its collapse -- saturation is a modelling refinement, not a
     *        datasheet number.
     *        [zh] 模拟 d 轴磁饱和：`Ld(i_d) = Ld0·(1 - depth·tanh(i_d/current_a))`。默认关闭
     *        （depth = 0），因为铭牌只给额定点的一个 `Ld`，不说它怎么塌——饱和是建模细化，不是铭牌数字。
     *
     * @param depth [en] fractional fall of the incremental inductance at `i_d >> current_a`, 0..1.
     *              [zh] `i_d >> current_a` 时增量电感的下降比例，0..1。
     * @param current_a [en] current at which saturation is substantially engaged, > 0.
     *                  [zh] 饱和明显生效的电流，> 0。
     * @note [en] Why this exists: the magnet polarity is invisible without it. `+i_d` reinforces the
     *       rotor flux and `-i_d` opposes it, so the *same* volt-second pulse produces two different
     *       current rises -- which is the only thing `MagnetPolarityTest` has to measure. The model is
     *       a local fit near `+-current_a`, not a magnetics reference: the `dL/di` term in the voltage
     *       equation is deliberately ignored.
     *       [zh] 它存在的理由：没有它就看不出磁钢极性。`+i_d` 增强转子磁链、`-i_d` 削弱它，于是**相同**
     *       的伏秒脉冲会产生两个不同的电流上升——这是 `MagnetPolarityTest` 唯一可测的现象。该模型是
     *       `±current_a` 附近的局部拟合，不是磁学参考：电压方程里的 `dL/di` 项被故意忽略。
     * @since 0.1.0
     */
    void set_d_axis_saturation(float depth, float current_a) noexcept;

    /**
     * @brief [en] Incremental d-axis inductance at a given d-axis current, H. Equals `params().ld`
     *        when saturation is off.
     *        [zh] 给定 d 轴电流下的增量 d 轴电感，H。饱和关闭时等于 `params().ld`。
     *
     * @param id_a [en] d-axis current in A to evaluate at. [zh] 求值点的 d 轴电流，A。
     * @since 0.1.0
     */
    [[nodiscard]] auto effective_ld(float id_a) const noexcept -> float;

    /**
     * @brief [en] Model winding heating: `Rs = Rs20 * (1 + alpha * (T - 20))`. Off by default.
     *        [zh] 模拟绕组发热：`Rs = Rs20·(1 + alpha·(T − 20))`。默认关闭。
     *
     * @param temperature_c [en] winding temperature in degrees C. [zh] 绕组温度，摄氏度。
     * @param alpha_per_k [en] temperature coefficient, copper is about 3.93e-3 per K. [zh] 温度系数，铜约为 3.93e-3 /K。
     * @note [en] Why this exists: a parameter-dentification result that only works at 20 C is a
     *       calculator, not an estimator. A drifting truth is how SIL can test the difference -- the
     *       plant heats up while the observer keeps believing the nameplate.
     *       [zh] 它存在的理由：只在 20 °C 成立的参数辨识是计算器，不是估计器。真值漂移是 SIL 能测出这个
     *       区别的方式——plant 升温而观测器仍然相信铭牌。
     * @since 0.1.0
     */
    void set_winding_temperature(float temperature_c, float alpha_per_k) noexcept;

    /**
     * @brief [en] Add sensor noise to the current readings: white, uniform in [-amplitude, +amplitude]
     *        amps, from a deterministic integer generator so a run is reproducible on any ISA.
     *        Off by default (amplitude 0).
     *        [zh] 给电流读数加传感器噪声：均匀分布在 [−amplitude, +amplitude] 安培，由确定性整数生成器
     *        产生，使同一次运行在任何 ISA 上都可复现。默认关闭（幅值 0）。
     *
     * @param amplitude_a [en] peak amplitude in A, >= 0. [zh] 峰值幅值，A，>= 0。
     * @note [en] Determinism is not a nicety here: the PIL leg compares host against target, and an
     *       entropy-based generator would make that comparison meaningless the moment noise is on.
     *       [zh] 确定性不是细节：PIL 腿要比对 host 与 target，一旦打开噪声，基于熵的生成器会让这个比对
     *       失去意义。
     * @since 0.1.0
     */
    void set_current_noise(float amplitude_a) noexcept;

    /**
     * @brief [en] Stator resistance the plant is actually using, ohm (includes the temperature model).
     *        [zh] plant 实际使用的定子电阻，Ω（含温度模型）。
     * @since 0.1.0
     */
    [[nodiscard]] auto effective_rs() const noexcept -> float;

    /**
     * @brief [en] Advance the plant by one fixed step with classical RK4.
     *        [zh] 用经典 RK4 推进一个固定步长。
     *
     * @param dt_s [en] step in s; <= 0 is ignored. [zh] 步长，s；<= 0 被忽略。
     * @since 0.1.0
     */
    void advance(float dt_s) noexcept;

    /**
     * @brief [en] Current integrator state.
     *        [zh] 当前积分状态。
     * @since 0.1.0
     */
    [[nodiscard]] auto state() const noexcept -> PlantState;

    /**
     * @brief [en] Mechanical rotor angle in rad (unwrapped).
     *        [zh] 机械转子角度，rad（不做归一化）。
     * @since 0.1.0
     */
    [[nodiscard]] auto theta_mech_rad() const noexcept -> float;

    /**
     * @brief [en] Mechanical rotor speed in rad/s.
     *        [zh] 机械转速，rad/s。
     * @since 0.1.0
     */
    [[nodiscard]] auto omega_mech_rad_s() const noexcept -> float;

    /**
     * @brief [en] Phase currents reconstructed through inverse Park + inverse Clarke, i.e. what a
     *        current sensor would see. This is the signal the SIL harness feeds to the HAL stub.
     *        [zh] 经逆 Park + 逆 Clarke 重构的相电流，即电流传感器"看到"的信号。SIL 夹具把它喂给
     *        HAL 桩。
     * @since 0.1.0
     */
    [[nodiscard]] auto current_uvw() const noexcept -> core::Uvw<float>;

    /**
     * @brief [en] d-q currents in A.
     *        [zh] d-q 电流，A。
     * @since 0.1.0
     */
    [[nodiscard]] auto current_dq() const noexcept -> core::Dq<float>;

    /**
     * @brief [en] Electromagnetic torque in N*m.
     *        [zh] 电磁转矩，N·m。
     * @since 0.1.0
     */
    [[nodiscard]] auto torque_nm() const noexcept -> float;

    /**
     * @brief [en] Rotor flux linkage derived from the torque constant.
     *        [zh] 由转矩常数反推的转子磁链。
     * @since 0.1.0
     */
    [[nodiscard]] auto psi_f_wb() const noexcept -> float;

    /// [en] Nameplate parameters of this plant. [zh] 本 plant 的铭牌参数。
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

    const float did = static_cast<float>((v_dq_.d - (rs * s.id_a) + (omega_e * params_.lq * s.iq_a)) / ld);
    const float diq = static_cast<float>(
        (v_dq_.q - (rs * s.iq_a) - (omega_e * ld * s.id_a) - (omega_e * psi_f_)) / params_.lq);

    const float torque = kOneAndHalf * poles * ((psi_f_ * s.iq_a) + ((ld - params_.lq) * s.id_a * s.iq_a));
    const float domega = static_cast<float>((torque - (params_.b * s.omega_mech) - load_nm_) / params_.j);

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

inline auto PmsmPlant::current_dq() const noexcept -> core::Dq<float> {
    return core::Dq<float>{state_.id_a, state_.iq_a};
}

inline auto PmsmPlant::current_uvw() const noexcept -> core::Uvw<float> {
    const float theta_e = static_cast<float>(params_.pole_pairs) * state_.theta_mech;
    const core::Uvw<float> true_current =
        core::inv_clarke(core::inv_park(core::Dq<float>{state_.id_a, state_.iq_a}, theta_e));
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
    return kOneAndHalf * poles * ((psi_f_ * state_.iq_a) + ((params_.ld - params_.lq) * state_.id_a * state_.iq_a));
}

inline auto PmsmPlant::psi_f_wb() const noexcept -> float { return psi_f_; }

inline auto PmsmPlant::params() const noexcept -> const PlantParams& { return params_; }

} // namespace sil

} // namespace foc
