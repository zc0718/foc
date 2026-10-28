// Conan::ImportStart
#pragma once

#include <cstdint>

#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
#include <foc/hal/current_sense.hpp>
#include <foc/hal/dc_link.hpp>
#include <foc/hal/position.hpp>
#include <foc/hal/pwm.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_discrete_inverter.hpp"
#include "sil_pmsm_plant.hpp"
// Conan::ImportEnd

namespace foc {

namespace sil {

using core::Ab;
using core::Dq;
using core::Uvw;
using motor::MotorConfig;

/**
 * @brief [en] Inverter adapter: records the duty and lets the bench push it into the plant.
 *
 * @brief [zh] 逆变器适配层：记录占空比，由台架把它送进 plant。
 *
 *     The duty is held in the **stationary** frame for a whole control period (zero-order hold on the applied stator
 *     voltage vector), which is what a real inverter does. Disabled or not-yet-enabled states hold 50% duty, i.e. zero
 *     voltage. 占空比在整个控制周期内保持于**静止**坐标系（作用到定子电压矢量上的零阶保持），这才是真实 逆变器的行为。
 *     未使能/已关闭状态保持 50% 占空比，即零电压。
 * @since 0.1.0
 */
class PlantPwm final : public hal::Pwm {
public:
  PlantPwm() noexcept = default;

  auto configure(float freq_hz, float deadtime_ns) noexcept
      -> ErrorCode override;
  void write_duty(Uvw<float> duty) noexcept override;
  auto enable() noexcept -> ErrorCode override;
  void disable() noexcept override;
  [[nodiscard]] auto fault() const noexcept -> bool override;

  /// [en] Last written duty (50% when disabled). [zh]
  /// 最近写入的占空比（关闭时为 50%）。
  [[nodiscard]] auto duty() const noexcept -> Uvw<float>;

  /// [en] Carrier frequency in Hz. [zh] 载波频率，Hz。
  [[nodiscard]] auto freq_hz() const noexcept -> float;

  /// [en] Whether the stage is enabled. [zh] 输出级是否使能。
  [[nodiscard]] auto enabled() const noexcept -> bool;

private:
  Uvw<float> duty_{0.5F, 0.5F, 0.5F};
  float freq_hz_{0.0F};
  float deadtime_ns_{0.0F};
  bool enabled_{false};
};

/**
 * @brief [en] Current sensing adapter reading the plant's phase currents.
 * @brief [zh] 电流采样适配层，读取 plant 的相电流。
 * @since 0.1.0
 */
class PlantCurrentSense final : public hal::CurrentSense {
public:
  explicit PlantCurrentSense(const PmsmPlant &plant,
                             const PlantPwm &pwm) noexcept;

  auto read_uvw() noexcept -> Uvw<float> override;
  auto calibrate_offset() noexcept -> ErrorCode override;
  [[nodiscard]] auto sample_rate_hz() const noexcept -> float override;

private:
  const PmsmPlant *plant_;
  const PlantPwm *pwm_;
};

/**
 * @brief [en] Position adapter reporting the plant rotor state as a sensored encoder would.
 * @brief [zh] 位置适配层，按带编码器的电机那样汇报 plant 的转子状态。
 * @since 0.1.0
 */
class PlantPosition final : public hal::PositionSensor {
public:
  explicit PlantPosition(const PmsmPlant &plant) noexcept;

  auto angle_mech_rad() noexcept -> float override;
  auto velocity_rad_s() noexcept -> float override;
  [[nodiscard]] auto has_sensor() const noexcept -> bool override;

  /// [en] Model an encoder mounting error the driver has to find. [zh]
  /// 模拟驱动需要找出的编码器安装误差。
  void set_mount_error_rad(float error_rad) noexcept;

  /// [en] Model a sensor-less setup: has_sensor() starts returning false, so
  /// the angle has to come
  ///        from the observer. Same name as `driver::SimPosition` on purpose --
  ///        the scenarios read the same either way. [zh]
  ///        模拟无感场景：has_sensor() 开始返回
  ///        false，角度必须由观测器给出。故意与 `driver::SimPosition`
  ///        同名——两种台架读起来一样。
  void set_sensor_present(bool present) noexcept;

  /// [en] The correction offset handed over by alignment. [zh]
  /// 由对齐交给传感器的校正偏置。
  void set_offset_mech_rad(float offset_rad) noexcept;

  /// [en] Offset currently subtracted from the readings, rad. [zh]
  /// 当前从读数中减去的偏置，rad。
  [[nodiscard]] auto mech_offset_rad() const noexcept -> float;

  /// [en] Rotor angle the plant is actually at, offset removed: the truth a
  /// test compares against. [zh] plant
  /// 转子真实所在的角度（不含偏置）：供测试比对的真值。
  [[nodiscard]] auto true_angle_mech_rad() const noexcept -> float;

private:
  const PmsmPlant *plant_;
  float mount_error_rad_{0.0F};
  float offset_rad_{0.0F};
  bool sensor_present_{true};
};

/**
 * @brief [en] DC link adapter with a stiff (ideal) bus voltage.
 * @brief [zh] 母线适配层，理想（刚性）母线电压。
 * @since 0.1.0
 */
class PlantDcLink final : public hal::DcLink {
public:
  explicit PlantDcLink(float v_bus) noexcept;

  auto voltage_v() noexcept -> float override;

  void set_voltage_v(float v_bus) noexcept;

private:
  float v_bus_;
};

/**
 * @brief [en] Complete HalPolicy for the SIL bench: the plant replaces the hardware.
 * @brief [zh] SIL 台架用的完整 HalPolicy：plant 代替硬件。
 * @context [en] advance() owns the plant integration; the controller only sees the HAL contracts.
 * @context [zh] advance() 负责 plant 积分；控制器只看到 HAL 契约。
 * @since 0.1.0
 */
class PlantHal final {
public:
  PlantHal(PmsmPlant &plant, float v_bus) noexcept;

  PlantHal(const PlantHal &) = delete;
  PlantHal &operator=(const PlantHal &) = delete;
  PlantHal(PlantHal &&) = delete;
  PlantHal &operator=(PlantHal &&) = delete;

  /**
   * @brief [en] Integrate the plant over one control period with the held duty.
   * @brief [zh] 用保持的占空比把 plant 积分一个控制周期。
   * @param dt_ctrl_s [en] control period in s.
   * @param dt_ctrl_s [zh] 控制周期，s。
   * @param substeps [en] integration sub-steps per control period.
   * @param substeps [zh] 每控制周期的积分子步数。
   * @post [en] The applied vector is fixed in the stationary frame over the whole period.
   * @post [zh] 整个周期内作用矢量在静止坐标系里固定。
   * @since 0.1.0
   */
  void advance(float dt_ctrl_s, int substeps) noexcept;

  /// [en] Reset the plant and the inverter to the power-on state. [zh] 把 plant
  /// 与逆变器复位到上电态。
  void reset(PlantState initial) noexcept;

  /**
   * @brief [en] Model inverter dead time as a per-phase voltage loss: `v_applied = v_cmd - v_dead * sign(i_phase)`.
   * @brief [zh] 把逆变器死区建模为每相的电压损失：`v_applied = v_cmd - v_dead * sign(i_phase)`。
   * @param v_dead [en] the magnitude, in V, of `(t_dead / T_pwm) * v_bus`; 0 disables the model.
   * @param v_dead [zh] `(t_dead / T_pwm) * v_bus` 的量值，V；0 表示关闭该模型。
   * @note [en] What this model buys: without it, the PRD's U1 ("dead time cannot be verified in SIL") was a statement
   *            about the *plant*, not the library. With it, dead-time distortion becomes a measurable quantity in CI --
   *            which is the difference between "we know we do not compensate" and "we can say by how much". Honest
   *            limitation: `sign(0) = 0` here, so the zero-current neighbourhood is modelled as *better* than real
   *            hardware, where the transition is a finite slope with ringing. Numbers from this model are a lower bound
   *            on the real distortion.
   * @note [zh] 这个模型换来的是：没有它，PRD 的 U1（"死区在 SIL 里无法验证"）是一个关于**被控对象** 的陈述，而不是关于
   *            库的。有了它，死区畸变在 CI 里成为可测量的量——这正是"我们知道自己没 补偿"与"我们能说出差多少"之间的区
   *            别。 诚实的局限：这里 `sign(0) = 0`，因此零电流附近被建模得比真实硬件**更好**——真实的 过渡是一段有限斜率
   *            并伴随振荡。本模型给出的数字是真实畸变的**下界**。
   * @since 0.1.0
   */
  void set_deadtime_voltage(float v_dead) noexcept;

  /// [en] Width of the smooth dead-zone transition in A; 0 keeps the ideal
  /// `sign()` model.
  ///      [zh] 平滑死区过渡宽度，A；0 保留理想的 `sign()` 模型。
  void set_deadtime_transition(float width_a) noexcept;

  [[nodiscard]] auto deadtime_voltage() const noexcept -> float;

  /**
   * @brief [en] Hand the voltage path to a switching inverter model (nullptr = the mean-value path).
   * @brief [zh] 把电压通路交给开关逆变器模型（nullptr = 平均值通路）。
   * @param inverter [en] model owned by the caller, or nullptr to restore the legacy behaviour.
   * @param inverter [zh] 由调用方持有的模型；nullptr 恢复旧行为。
   * @note [en] While a model is attached, `set_deadtime_voltage()` / `set_deadtime_transition()` are **not consulted**:
   *            the dead-time effect comes out of the switching process instead of being added to it, and having two
   *            sources would make the numbers unreadable. If the model refuses the request (control period not a whole
   *            number of switching periods), `advance()` falls back to the mean-value path rather than quietly doing
   *            nothing.
   * @note [zh] 挂上模型之后 `set_deadtime_voltage()` / `set_deadtime_transition()` **不再生效**：死区 效应从开关过程里
   *            出来，而不是加到它上面；两个来源会让数字无法解读。 若模型拒绝该请求（控制周期不是开关周期的整数倍），
   *            `advance()` 退回平均值通路，而不是 悄悄什么都不做。
   * @since 0.1.0
   */
  void set_inverter(DiscreteInverter *inverter) noexcept;

  PlantPwm pwm;
  PlantCurrentSense current_sense;
  PlantPosition position;
  PlantDcLink dc_link;

private:
  PmsmPlant *plant_;
  DiscreteInverter *inverter_{nullptr};
  float v_dead_{0.0F};
  float transition_a_{
      /**
       * @brief [en] dead-zone transition width in A; 0 = ideal `sign()`.
       * @brief [zh] 死区过渡宽度，A；0 = 理想 `sign()`。
       */
      0.0F};
};

// PlantPwm

inline auto PlantPwm::configure(float freq_hz, float deadtime_ns) noexcept
    -> ErrorCode {
  if (!(freq_hz > 0.0F) || !(deadtime_ns >= 0.0F)) {
    return ErrorCode::Param;
  }
  freq_hz_ = freq_hz;
  deadtime_ns_ = deadtime_ns;
  return ErrorCode::Ok;
}

inline void PlantPwm::write_duty(Uvw<float> duty) noexcept { duty_ = duty; }

inline auto PlantPwm::enable() noexcept -> ErrorCode {
  enabled_ = true;
  duty_ = Uvw<float>{0.5F, 0.5F, 0.5F};
  return ErrorCode::Ok;
}

inline void PlantPwm::disable() noexcept {
  enabled_ = false;
  duty_ = Uvw<float>{0.5F, 0.5F, 0.5F};
}

inline auto PlantPwm::fault() const noexcept -> bool { return false; }

inline auto PlantPwm::duty() const noexcept -> Uvw<float> {
  return enabled_ ? duty_ : Uvw<float>{0.5F, 0.5F, 0.5F};
}

inline auto PlantPwm::freq_hz() const noexcept -> float { return freq_hz_; }

inline auto PlantPwm::enabled() const noexcept -> bool { return enabled_; }

// PlantCurrentSense

inline PlantCurrentSense::PlantCurrentSense(const PmsmPlant &plant,
                                            const PlantPwm &pwm) noexcept
    : plant_(&plant), pwm_(&pwm) {}

inline auto PlantCurrentSense::read_uvw() noexcept -> Uvw<float> {
  return plant_->current_uvw();
}

inline auto PlantCurrentSense::calibrate_offset() noexcept -> ErrorCode {
  return ErrorCode::Ok;
}

inline auto PlantCurrentSense::sample_rate_hz() const noexcept -> float {
  return pwm_->freq_hz();
}

// PlantPosition

inline PlantPosition::PlantPosition(const PmsmPlant &plant) noexcept
    : plant_(&plant) {}

inline auto PlantPosition::angle_mech_rad() noexcept -> float {
  // Two different quantities live here, and mixing them up is exactly the bug
  // this model has to be able to detect: `mount_error_rad_` is how badly the
  // encoder is installed (physics, set by the test), while `offset_rad_` is the
  // correction the motor's alignment writes through the optional HAL hook. The
  // reading is truth + mounting error - correction, wrapped into [0, 2*pi).
  // 这里有两个不同的量，把它们混同正是本模型必须能检出的那个错误：`mount_error_rad_`
  // 是编码器装错 了多少（物理量，由测试设定），`offset_rad_` 是电机对齐通过可选
  // HAL 钩子写入的校正量。读数为 真值 + 安装误差 - 校正量，并归一化到 [0, 2π)。
  const float two_pi = 6.28318530718F;
  float angle = plant_->theta_mech_rad() + mount_error_rad_ - offset_rad_;
  angle =
      angle - (two_pi * static_cast<float>(static_cast<int>(angle / two_pi)));
  if (angle < 0.0F) {
    angle += two_pi;
  }
  return angle;
}

inline void PlantPosition::set_mount_error_rad(float error_rad) noexcept {
  mount_error_rad_ = error_rad;
}

inline void PlantPosition::set_offset_mech_rad(float offset_rad) noexcept {
  offset_rad_ = offset_rad;
}

inline auto PlantPosition::mech_offset_rad() const noexcept -> float {
  return offset_rad_;
}

inline auto PlantPosition::true_angle_mech_rad() const noexcept -> float {
  return plant_->theta_mech_rad();
}

inline auto PlantPosition::velocity_rad_s() noexcept -> float {
  return plant_->omega_mech_rad_s();
}

inline auto PlantPosition::has_sensor() const noexcept -> bool {
  return sensor_present_;
}

inline void PlantPosition::set_sensor_present(bool present) noexcept {
  sensor_present_ = present;
}

// PlantDcLink

inline PlantDcLink::PlantDcLink(float v_bus) noexcept : v_bus_(v_bus) {}

inline auto PlantDcLink::voltage_v() noexcept -> float { return v_bus_; }

inline void PlantDcLink::set_voltage_v(float v_bus) noexcept { v_bus_ = v_bus; }

// PlantHal

inline PlantHal::PlantHal(PmsmPlant &plant, float v_bus) noexcept
    : pwm(), current_sense(plant, pwm), position(plant), dc_link(v_bus),
      plant_(&plant) {}

inline void PlantHal::advance(float dt_ctrl_s, int substeps) noexcept {
  if (!(dt_ctrl_s > 0.0F) || substeps <= 0) {
    return;
  }

  const float v_bus = dc_link.voltage_v();
  const Uvw<float> duty = pwm.duty();

  // The switching model, when one is attached: the period is walked sub-step by
  // sub-step and the plant is handed what the legs actually deliver at each
  // instant, including the ripple and the dead-time windows. See
  // sil_discrete_inverter.hpp for why this exists.
  // 挂上开关模型时：逐子步走完这个周期，把桥臂在该时刻实际交付的电压交给
  // plant，纹波与死区窗口都在里面。 存在的理由见 sil_discrete_inverter.hpp。
  if (inverter_ != nullptr) {
    const int model_steps = inverter_->begin_period(duty, v_bus, dt_ctrl_s);
    if (model_steps > 0) {
      const float h = inverter_->step_length_s();
      const auto poles = static_cast<float>(plant_->params().pole_pairs);
      for (int step = 0; step < model_steps; ++step) {
        const Ab<float> v_switched = inverter_->step(plant_->current_uvw());
        const float theta_e = poles * plant_->theta_mech_rad();
        plant_->set_voltage_dq(core::park(v_switched, theta_e));
        plant_->advance(h);
      }
      return;
    }
  }

  // The inverter holds one voltage vector in the stationary frame for the whole
  // period; Clarke of the phase voltages is immune to the common-mode injection
  // of SVPWM, so the duty maps back to v_ab exactly.
  Uvw<float> v_uvw{duty.u * v_bus, duty.v * v_bus, duty.w * v_bus};

  // Dead time makes the applied phase voltage depend on the sign of that
  // phase's current: during the blanking interval the phase is clamped to the
  // rail the current is coming from.
  // 死区让施加的相电压取决于该相电流的方向：在死区时间内，该相被钳位在电流来自的那条母线上。
  if (v_dead_ > 0.0F) {
    const Uvw<float> i_uvw = plant_->current_uvw();
    const auto phase_loss = [this](float i) noexcept -> float {
      // A real inverter does not switch polarity on a zero crossing; the clamp
      // at the rails is approached smoothly. `transition_a_ > 0` models that as
      // a linear ramp of width
      // +-transition, which is the unmodelled effect S13's `sign()` version
      // cannot see.
      // 真实逆变器不会在过零处瞬间换极性；钳位到母线是平滑趋近的。`transition_a_
      // > 0` 把它建模为 ±transition 宽的线性斜坡，这正是 S13 的 `sign()`
      // 版本看不见的未建模效应。
      if (transition_a_ > 0.0F) {
        const float normalized = i / transition_a_;
        return v_dead_ * ((normalized > 1.0F)
                              ? 1.0F
                              : ((normalized < -1.0F) ? -1.0F : normalized));
      }
      return v_dead_ * static_cast<float>((i > 0.0F) - (i < 0.0F));
    };
    v_uvw.u -= phase_loss(i_uvw.u);
    v_uvw.v -= phase_loss(i_uvw.v);
    v_uvw.w -= phase_loss(i_uvw.w);
  }

  const Ab<float> v_ab = core::clarke(v_uvw);

  const float dt_sub = dt_ctrl_s / static_cast<float>(substeps);
  const auto poles = static_cast<float>(plant_->params().pole_pairs);
  for (int i = 0; i < substeps; ++i) {
    const float theta_e = poles * plant_->theta_mech_rad();
    plant_->set_voltage_dq(core::park(v_ab, theta_e));
    plant_->advance(dt_sub);
  }
}

inline void PlantHal::set_inverter(DiscreteInverter *inverter) noexcept {
  inverter_ = inverter;
}

inline void PlantHal::set_deadtime_voltage(float v_dead) noexcept {
  v_dead_ = (v_dead > 0.0F) ? v_dead : 0.0F;
}

inline void PlantHal::set_deadtime_transition(float width_a) noexcept {
  transition_a_ = (width_a > 0.0F) ? width_a : 0.0F;
}

inline auto PlantHal::deadtime_voltage() const noexcept -> float {
  return v_dead_;
}

inline void PlantHal::reset(PlantState initial) noexcept {
  plant_->reset(initial);
  pwm.disable();
}

} // namespace sil

} // namespace foc
