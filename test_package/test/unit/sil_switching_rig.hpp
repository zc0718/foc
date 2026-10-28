// Conan::ImportStart
#pragma once

#include <cmath>

#include <foc/core/deadtime.hpp>
#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

#include "sil_bench.hpp"
#include "sil_discrete_inverter.hpp"
#include "sil_pmsm_plant.hpp"

namespace foc {

namespace sil {

/**
 * @brief [en] Where to put the machine, and what to lie to the observers about.
 *        [zh] 把机器放在哪里，以及对观测器谎报什么。
 *
 * @note [en] The *true* machine is always the nameplate one; only the belief
 * varies. That split is what makes an Rs-mismatch sweep meaningful -- heating
 * the machine and heating the belief together would measure nothing. [zh]
 * **真**机器永远是铭牌电机，只有信念在变。这个分离才让 Rs
 * 失配扫描有意义——把机器和信念一起 加热，什么都量不到。
 * @since 0.1.0
 */
struct SwitchingRigConfig final {
  float omega_e{
      200.0F};      ///< [en] electrical speed, rad/s. [zh] 电角速度，rad/s。
  float iq_a{0.5F}; ///< [en] load current, A. [zh] 负载电流，A。
  /// [en] d-axis current, A. Non-zero here is what makes a salient machine
  /// behave saliently: the model error a single-`L` observer makes is
  /// `(Ld - Lq)*i_d`, so with `i_d = 0` every saliency ratio looks identical.
  /// Defaults to 0, so every existing test keeps its meaning.
  /// [zh] d 轴电流，A。这里非零才是凸极机表现为凸极的条件：单 `L` 观测器的模型误差是
  /// `(Ld − Lq)·id`，所以 `i_d = 0` 时任何凸极比看起来都一样。默认 0，既有测试含义不变。
  float id_a{0.0F};
  float belief_rs{1.10F}; ///< [en] Rs multiplier the observers are told. [zh]
                          ///< 告诉观测器的 Rs 倍数。
  float noise_a{0.1F};    ///< [en] phase-current noise amplitude, A; 0 = clean.
                          ///< [zh] 相电流噪声幅值，A。
  float t_dead_s{1.0e-6F}; ///< [en] inverter dead time, s; 0 = ideal legs. [zh]
                           ///< 逆变器死区，s。
  std::int32_t sub_per_switch{
      40}; ///< [en] sub-steps per switching period. [zh] 每开关周期子步数。
  float v_bus_v{24.0F}; ///< [en] DC link, V. [zh] 母线电压，V。
  float seconds{0.4F};  ///< [en] run length; the second half is measured. [zh]
                        ///< 运行时长；只量后半段。
  /// [en] Dead-time volt-seconds to *compensate* for, V; 0 = no compensator in
  /// the loop. [zh] 要**补偿**的死区伏秒，V；0 = 环内没有补偿器。
  float compensate_v{0.0F};
  /// [en] The fade width handed to the compensator, A; used only when
  /// `compensate_v > 0`. [zh] 交给补偿器的淡出宽度，A；只在 `compensate_v > 0`
  /// 时使用。
  float i_fade_a{0.0F};
  /// [en] The machine this rig is built around. Defaults to the nominal one, so
  /// every existing test keeps its meaning; P5-f varies it to ask whether the
  /// tuning survives a different motor. [zh]
  /// 这个台架围绕的机器。默认标称机，所以既有测试的含义不变；P5-f
  /// 改变它，用来问"换一台 电机，整定还活不活"。
  MachineSpec spec = nominal_spec();
};

/**
 * @brief [en] Constant-speed machine driven through a switching inverter, with
 * the current sampled once per control period. [zh]
 * 经开关逆变器驱动的恒速电机，电流每个控制周期采一次。
 *
 * @note [en] The observers are handed the **commanded** vector and the
 * **sampled** current, which is all a real controller knows. Everything else --
 * ripple, dead-time volt-seconds, the fact that the current is not the average
 * -- is what they have to cope with. [zh]
 * 观测器拿到的是**指令**矢量和**采样**电流，也就是真实控制器知道的全部。其余的一切——纹波、
 *       死区伏秒、电流不等于平均值——都是它们必须自己应付的。
 * @since 0.1.0
 */
class SwitchingRig final {
public:
  explicit SwitchingRig(const SwitchingRigConfig &config) noexcept
      : config_(config), plant_(plant_params_from(config.spec)),
        inverter_(InverterConfig{20000.0F, config.t_dead_s, 0.0F,
                                 config.sub_per_switch}),
        compensator_(core::DeadTimeCompensator<core::Svpwm>::Config{
            config.compensate_v, 0.02F, config.i_fade_a}),
        steps_(static_cast<int>(config.seconds / kControlPeriod)) {
    if (config_.noise_a > 0.0F) {
      plant_.set_current_noise(config_.noise_a);
    }
    plant_.reset(PlantState{
        config.id_a, config.iq_a,
        config.omega_e / static_cast<float>(config.spec.pole_pairs), 0.0F});
    // Steady-state command for both axes, saliency included: the d-axis
    // voltage carries the `Lq*iq` cross-coupling and the q-axis voltage carries
    // `Ld*id`.
    // 两个轴的稳态指令，含凸极：d 轴电压带 `Lq·iq` 交叉耦合，q 轴电压带 `Ld·id`。
    v_d_ = (config_.spec.rs * config.id_a) - (config_.omega_e * config_.spec.lq * config_.iq_a);
    v_q_ = (config_.spec.rs * config_.iq_a) +
           (config_.omega_e * config_.spec.ld * config.id_a) + (config_.omega_e * machine_psi_f());
  }

  SwitchingRig(const SwitchingRig &) = delete;
  SwitchingRig &operator=(const SwitchingRig &) = delete;
  SwitchingRig(SwitchingRig &&) = delete;
  SwitchingRig &operator=(SwitchingRig &&) = delete;

  /// [en] Control period of this rig, s. [zh] 本台架的控制周期，s。
  static constexpr float kControlPeriod = 5.0e-5F;

  /// [en] The nominal machine's flux linkage. Kept static for the tests that
  /// name it directly. [zh] 标称机的磁链。保留 static 给直接引用它的测试。
  [[nodiscard]] static auto psi_f() noexcept -> float {
    return nominal_spec().kt /
           (1.5F * static_cast<float>(nominal_spec().pole_pairs));
  }

  /// [en] The flux linkage of the machine this rig was actually built with.
  /// [zh] 本台架实际所用机器的磁链。
  [[nodiscard]] auto machine_psi_f() const noexcept -> float {
    return config_.spec.kt /
           (1.5F * static_cast<float>(config_.spec.pole_pairs));
  }

  [[nodiscard]] auto true_angle_e() const noexcept -> float {
    return static_cast<float>(config_.spec.pole_pairs) *
           plant_.theta_mech_rad();
  }

  /// [en] The vector the controller asks for, in the stationary frame. [zh]
  /// 控制器请求的矢量，静止系。
  [[nodiscard]] auto command() const noexcept -> core::Ab<float> {
    return command_;
  }

  /// [en] The current the controller *sees*: one sample per control period.
  /// [zh] 控制器**看到**的电流：每 控制周期一个采样。
  [[nodiscard]] auto current() const noexcept -> core::Ab<float> {
    return current_;
  }

  /// [en] What an observer should be told the stator resistance is. [zh]
  /// 应该告诉观测器的定子电阻。
  [[nodiscard]] auto believed_rs() const noexcept -> float {
    return config_.belief_rs * config_.spec.rs;
  }

  [[nodiscard]] auto steps() const noexcept -> int { return steps_; }

  void step() noexcept {
    command_ = core::inv_park(core::Dq<float>{v_d_, v_q_}, true_angle_e());
    const core::Uvw<float> i_uvw = plant_.current_uvw();
    current_ = core::clarke(i_uvw);

    const core::ModulatorInput request{command_, config_.v_bus_v, i_uvw};
    // The compensator sits between the command and the legs when it is enabled;
    // the observer is still handed `command()`, because with the loss
    // compensated the delivered voltage *is* the command.
    // 补偿器启用时坐在指令与桥臂之间；观测器拿到的仍是 `command()`，因为损失被
    // 补掉后交付电压**就是**指令。
    const core::SvpwmResult modulated = compensator_.modulate(request);
    const int count =
        inverter_.begin_period(modulated.duty, config_.v_bus_v, kControlPeriod);
    const float h = inverter_.step_length_s();
    const auto poles = static_cast<float>(config_.spec.pole_pairs);
    for (int k = 0; k < count; ++k) {
      const core::Ab<float> v = inverter_.step(plant_.current_uvw());
      plant_.set_voltage_dq(core::park(v, poles * plant_.theta_mech_rad()));
      plant_.advance(h);
    }
  }

private:
  SwitchingRigConfig config_;
  PmsmPlant plant_;
  DiscreteInverter inverter_;
  core::DeadTimeCompensator<core::Svpwm> compensator_;
  int steps_;
  float v_d_{0.0F};
  float v_q_{0.0F};
  core::Ab<float> command_{0.0F, 0.0F};
  core::Ab<float> current_{0.0F, 0.0F};
};

/**
 * @brief [en] Runs one observer over a rig and returns the mean |angle error|
 * over the second half. [zh]
 * 在一个台架上跑一个观测器，返回后半段的角度误差绝对均值。
 * @since 0.1.0
 */
template <typename Update>
auto run_switched(const SwitchingRigConfig &config, Update &&update) -> float {
  SwitchingRig rig(config);
  float sum = 0.0F;
  int count = 0;
  const int total = rig.steps();
  for (int k = 0; k < total; ++k) {
    rig.step();
    const core::ObsState observed = update(rig);
    if (k >= (total / 2)) {
      float error = observed.theta_e - rig.true_angle_e();
      while (error <= -core::kPi<float>) {
        error += core::kTwoPi<float>;
      }
      while (error > core::kPi<float>) {
        error -= core::kTwoPi<float>;
      }
      sum += std::fabs(error);
      ++count;
    }
  }
  return (count > 0) ? (sum / static_cast<float>(count)) : 0.0F;
}

/**
 * @brief [en] A textbook sliding-mode observer, **a control and not a
 * candidate**: it exists so the gate has something that can fail. Current-error
 * sliding surface, `z = K * sat(s / layer)` injected as a voltage, and a
 * first-order filter on `z` to read out the equivalent control (the back-EMF).
 *        [zh]
 * 教科书式滑模观测器，**是对照而不是候选**：它的存在是为了让闸有一个能失败的对象。电流误差
 *        滑模面，`z = K·sat(s/layer)` 作为注入电压，再对 `z`
 * 做一阶滤波读出等效控制（反电动势）。
 *
 * @note [en] It is deliberately *sane*: it reaches ~0.03 rad on a plant with
 * neither impairment, so a failure here is about the structure under
 * disturbance, not about a broken straw man. [zh]
 * 它被刻意做得**靠谱**：在没有任何未建模效应的 plant 上它能到 ~0.03
 * rad，所以这里的失败是结构 在扰动下的问题，而不是稻草人写坏了。
 * @since 0.1.0
 */
class SmoControl final {
public:
  SmoControl(float rs, float l, float gain, float fc_hz, float layer,
             float dt_s = 5.0e-5F) noexcept
      : rs_(rs), l_(l), gain_(gain), layer_(layer), dt_(dt_s),
        alpha_(std::exp(-core::kTwoPi<float> * fc_hz * dt_s)) {}

  auto update(core::Ab<float> v, core::Ab<float> i) noexcept -> core::ObsState {
    const float s_a = static_cast<float>(i_hat_.alpha - i.alpha);
    const float s_b = static_cast<float>(i_hat_.beta - i.beta);
    const float z_a = gain_ * sat(s_a);
    const float z_b = gain_ * sat(s_b);
    i_hat_.alpha = static_cast<float>(
        i_hat_.alpha + (dt_ * ((v.alpha - (rs_ * i_hat_.alpha) - z_a) / l_)));
    i_hat_.beta = static_cast<float>(
        i_hat_.beta + (dt_ * ((v.beta - (rs_ * i_hat_.beta) - z_b) / l_)));
    e_.alpha =
        static_cast<float>((alpha_ * e_.alpha) + ((1.0F - alpha_) * z_a));
    e_.beta = static_cast<float>((alpha_ * e_.beta) + ((1.0F - alpha_) * z_b));

    // e = -omega*psi_f*sin(theta), omega*psi_f*cos(theta), so the angle is
    // atan2(-e_alpha, e_beta). e = −ω·ψf·sinθ, ω·ψf·cosθ，所以角度是
    // atan2(−e_alpha, e_beta)。
    float theta = std::atan2(-e_.alpha, e_.beta);
    while (theta < 0.0F) {
      theta += core::kTwoPi<float>;
    }
    state_.theta_e = theta;
    return state_;
  }

private:
  [[nodiscard]] auto sat(float value) const noexcept -> float {
    const float normalized = value / layer_;
    return (normalized > 1.0F) ? 1.0F
                               : ((normalized < -1.0F) ? -1.0F : normalized);
  }

  float rs_;
  float l_;
  float gain_;
  float layer_;
  float dt_;
  float alpha_;
  core::Ab<float> i_hat_{};
  core::Ab<float> e_{};
  core::ObsState state_{};
};

/**
 * @brief [en] The tuning a sliding-mode observer needs to survive the whole
 * speed range, as a *schedule* rather than a constant. [zh]
 * 滑模观测器要在整个速度范围活下来所需的调参——是一条**调度**，不是一个常数。
 *
 * @param omega_e [en] electrical speed in rad/s. [zh] 电角速度，rad/s。
 * @param psi_f [en] rotor flux linkage in Wb. [zh] 转子磁链，Wb。
 * @return [en] gain in V, filter corner in Hz, boundary layer in A. [zh]
 * 增益（V）、滤波拐点（Hz）、 边界层（A）。
 *
 * @note [en] Both numbers come from one fact: the sliding-mode gain must exceed
 * the back-EMF it is trying to reconstruct, `K > |omega_e| * psi_f`, and it
 * must not be much larger than that or the chattering it injects grows with it.
 * So `K` is proportional to speed with a floor. The equivalent-control filter
 *       has the mirror problem: its corner must be well above the signal it
 * must pass (lag) but low enough to reject the switching (chatter), so it
 * tracks speed too. [zh]
 * 两个数来自同一个事实：滑模增益必须大于它要重建的反电动势，`K >
 * |omega_e|·psi_f`；又不能比它大 太多，否则注入的抖振跟着涨。所以 `K`
 * 有下限且正比于转速。等效控制滤波器是镜像的问题：拐点要明显
 *       高于必须通过的信号（否则滞后），又要低到能压住开关（否则抖振），所以它也跟随转速。
 * @note [en] These constants are fitted to the S18 record, not derived: the
 * point of the bookkeeping is that a *single constant* provably cannot serve
 * the envelope, not that these particular numbers are optimal. [zh]
 * 这些常数是对着 S18
 * 的记录**拟合**的，不是推导出来的：这份记账要说明的是"单个常数**不可能**
 *       覆盖整个包络"，而不是这几个数字最优。
 * @since 0.1.0
 */
struct SmoTuning final {
  float gain;
  float fc_hz;
  float layer;
};

[[nodiscard]] inline auto scheduled_smo(float omega_e, float psi_f) noexcept
    -> SmoTuning {
  const float back_emf = std::fabs(omega_e) * psi_f;
  const float signal_hz = std::fabs(omega_e) / core::kTwoPi<float>;
  return SmoTuning{std::fmax(2.0F, 1.5F * back_emf),
                   std::fmax(500.0F, 30.0F * signal_hz), 1.0F};
}

} // namespace sil

} // namespace foc
