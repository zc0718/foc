// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <foc/core/limits.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/sm.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
#include <foc/motor/foc_loop.hpp>
#include <foc/motor/motor_config.hpp>
// Conan::ImportEnd

namespace foc {

namespace motor {

using core::Ab;
using core::Dq;
using core::Uvw;


/**
 * @brief [en] Read-back of the last control step.
 *        [zh] 最近一个控制步的读回值。
 *
 * @since 0.1.0
 * @exporter
 */
struct Telemetry final {
    float speed_mech_rad_s; ///< [en] mechanical speed in rad/s. [zh] 机械速度，rad/s。
    float id_a;             ///< [en] measured d-axis current in A. [zh] 实测 d 轴电流，A。
    float iq_a;             ///< [en] measured q-axis current in A. [zh] 实测 q 轴电流，A。
};


/**
 * @brief [en] What the q-axis reference is driven by.
 *        [zh] q 轴给定由什么驱动。
 *
 * @since 0.1.0
 * @exporter
 */
enum class ReferenceMode : std::uint8_t {
    Torque = 0, ///< [en] iq is commanded directly. [zh] 直接给定 iq。
    Speed = 1,  ///< [en] the speed controller produces iq. [zh] 由速度环产生 iq。
};


/**
 * @brief [en] Policy-composed motor: no globals, no allocation, no virtual calls in the loop.
 *        All state lives in this object, so multiple instances are independent by construction.
 *        [zh] 策略组合的电机：无全局状态、不分配、控制环内无虚函数调用。状态全在对象内，
 *        因此多实例天然相互独立。
 *
 *        Binding contract:
 *        - `Observer` provides `update(Ab<float>, Ab<float>, float) noexcept -> ObsState`
 *          (and optionally `seed_flux(Ab<float>)`, used at start()).
 *        - `Modulator` provides `modulate(Ab<float>, float) noexcept -> SvpwmResult`.
 *        - `Controller` provides `step(float, float, float) noexcept -> float` and `reset()`.
 *        - `HalPolicy` exposes members `pwm`, `current_sense`, `position`, `dc_link`.
 *        绑定契约：见上（Observer 可选 seed_flux，HalPolicy 的成员名固定）。
 *
 * @context [en] tick() is the ISR entry: noexcept, no allocation, no I/O.
 *           [zh] tick() 是 ISR 入口：noexcept、不分配、无 I/O。
 * @since 0.1.0
 * @exporter
 */
template <typename Observer, typename Modulator, typename Controller, typename HalPolicy> class Motor final {
  public:
    /**
     * @brief [en] Policy instances owned by the motor (one per control axis plus the speed loop).
     *        [zh] 电机持有的策略实例（每个控制轴一个，外加速度环）。
     * @since 0.1.0
     */
    struct Policies final {
        Observer observer;
        Modulator modulator;
        Controller current_d;
        Controller current_q;
        Controller speed;
    };

    Motor(MotorConfig config, Policies policies, HalPolicy& hal) noexcept;

    /**
     * @brief [en] Validate the configuration and configure the inverter.
     *        [zh] 校验配置并配置逆变器。
     * @return [en] Ok, or Param/State from the PWM contract. [zh] Ok，或来自 PWM 契约的 Param/State。
     * @context [en] Initialisation only. [zh] 仅初始化阶段。
     * @since 0.1.0
     */
    auto init() noexcept -> ErrorCode;

    /**
     * @brief [en] Idle -> Align -> Run, enable the inverter, seed a flux observer if it takes one.
     *        [zh] Idle -> Align -> Run，使能逆变器；若观测器支持则为其播种。
     * @return [en] Ok, or the state-machine / PWM error. [zh] Ok，或状态机 / PWM 错误。
     * @since 0.1.0
     */
    auto start() noexcept -> ErrorCode;

    /**
     * @brief [en] Disable the inverter and return to Idle.
     *        [zh] 关闭逆变器并回到 Idle。
     * @since 0.1.0
     */
    auto stop() noexcept -> ErrorCode;

    /**
     * @brief [en] Enter the latched Fault state and disable the inverter.
     *        [zh] 进入锁存 Fault 状态并关闭逆变器。
     * @since 0.1.0
     */
    void fault() noexcept;

    /**
     * @brief [en] Leave Fault (only legal from Fault) and reset the controllers.
     *        [zh] 退出 Fault（仅能从 Fault 离开）并复位控制器。
     * @since 0.1.0
     */
    auto clear_fault() noexcept -> ErrorCode;

    /**
     * @brief [en] Torque reference: iq in A, clamped to the nameplate limit.
     *        [zh] 转矩给定：iq，A，按铭牌限幅。
     * @since 0.1.0
     */
    void set_torque(float iq_a) noexcept;

    /**
     * @brief [en] Speed reference: mechanical rad/s, fed into the speed loop.
     *        [zh] 速度给定：机械角速度 rad/s，送入速度环。
     * @since 0.1.0
     */
    void set_speed(float speed_mech_rad_s) noexcept;

    /**
     * @brief [en] One current-loop step: sense -> Clarke -> angle -> Park -> PI -> InvPark -> SVPWM.
     *        [zh] 一步电流环：采样 -> Clarke -> 角度 -> Park -> PI -> InvPark -> SVPWM。
     *
     * @param dt_s [en] step length in s; <= 0 is rejected (status Param).
     *              [zh] 步长，s；<= 0 被拒绝（状态置 Param）。
     * @return [en] Ok, Param, or State (not running). [zh] Ok、Param 或 State（未运行）。
     * @since 0.1.0
     */
    auto tick(float dt_s) noexcept -> ErrorCode;

    /**
     * @brief [en] Current mode state.
     *        [zh] 当前模式状态。
     * @since 0.1.0
     */
    [[nodiscard]] auto state() const noexcept -> core::MotorState;

    /**
     * @brief [en] Which reference drives the q axis.
     *        [zh] q 轴由哪种给定驱动。
     * @since 0.1.0
     */
    [[nodiscard]] auto mode() const noexcept -> ReferenceMode;

    /**
     * @brief [en] Telemetry of the last tick.
     *        [zh] 最近一次 tick 的遥测。
     * @since 0.1.0
     */
    [[nodiscard]] auto telemetry() const noexcept -> Telemetry;

    /**
     * @brief [en] Sticky status of the last operation.
     *        [zh] 最近一次操作的粘滞状态。
     * @since 0.1.0
     */
    [[nodiscard]] auto status() const noexcept -> ErrorCode;

    /**
     * @brief [en] Mutable access to the observer (used to seed it from the align routine or a test).
     *        [zh] 观测器的可变访问（供对齐流程或测试播种）。
     * @since 0.1.0
     */
    [[nodiscard]] auto observer() noexcept -> Observer&;

    /**
     * @brief [en] Nameplate parameters.
     *        [zh] 铭牌参数。
     * @since 0.1.0
     */
    [[nodiscard]] auto config() const noexcept -> const MotorConfig&;

  private:
    MotorConfig config_;
    Policies policies_;
    HalPolicy* hal_;
    core::StateMachine sm_;
    Telemetry telemetry_;
    ErrorCode status_;
    ReferenceMode mode_;
    float iq_ref_a_;
    float speed_ref_rad_s_;
    Ab<float> v_ab_prev_;
};

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
Motor<Observer, Modulator, Controller, HalPolicy>::Motor(MotorConfig config, Policies policies, HalPolicy& hal) noexcept
    : config_(config), policies_(policies), hal_(&hal), sm_(), telemetry_{0.0F, 0.0F, 0.0F}, status_(ErrorCode::Ok),
      mode_(ReferenceMode::Torque), iq_ref_a_(0.0F), speed_ref_rad_s_(0.0F), v_ab_prev_{} {}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
auto Motor<Observer, Modulator, Controller, HalPolicy>::init() noexcept -> ErrorCode {
    if (!(config_.rs >= 0.0F) || !(config_.pwm_freq_hz > 0.0F) || (config_.pole_pairs == 0U)) {
        status_ = ErrorCode::Param;
        return status_;
    }

    status_ = hal_->pwm.configure(config_.pwm_freq_hz, config_.deadtime_ns);
    return status_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
auto Motor<Observer, Modulator, Controller, HalPolicy>::start() noexcept -> ErrorCode {
    if (status_ != ErrorCode::Ok) {
        return status_;
    }

    ErrorCode rc = sm_.request(core::MotorState::Align);
    if (rc != ErrorCode::Ok) {
        return rc;
    }

    // Align: a sensored setup already knows the angle; a sensorless one needs the flux seeded from
    // the nameplate so the integrator's unknown constant stops mattering (see D18 in the PRD).
    if constexpr (core::concepts::has_seed_flux_v<Observer>) {
        policies_.observer.seed_flux(Ab<float>{psi_f_wb(config_), 0.0F});
    }

    rc = hal_->pwm.enable();
    if (rc != ErrorCode::Ok) {
        static_cast<void>(sm_.request(core::MotorState::Fault));
        status_ = rc;
        return rc;
    }

    return sm_.request(core::MotorState::Run);
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
auto Motor<Observer, Modulator, Controller, HalPolicy>::stop() noexcept -> ErrorCode {
    hal_->pwm.disable();
    return sm_.request(core::MotorState::Idle);
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
void Motor<Observer, Modulator, Controller, HalPolicy>::fault() noexcept {
    hal_->pwm.disable();
    static_cast<void>(sm_.request(core::MotorState::Fault));
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
auto Motor<Observer, Modulator, Controller, HalPolicy>::clear_fault() noexcept -> ErrorCode {
    const ErrorCode rc = sm_.clear_fault();
    if (rc == ErrorCode::Ok) {
        policies_.current_d.reset();
        policies_.current_q.reset();
        policies_.speed.reset();
        iq_ref_a_ = 0.0F;
        status_ = ErrorCode::Ok;
    }
    return rc;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
void Motor<Observer, Modulator, Controller, HalPolicy>::set_torque(float iq_a) noexcept {
    mode_ = ReferenceMode::Torque;
    iq_ref_a_ = core::clamp(iq_a, -config_.i_max_a, config_.i_max_a);
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
void Motor<Observer, Modulator, Controller, HalPolicy>::set_speed(float speed_mech_rad_s) noexcept {
    mode_ = ReferenceMode::Speed;
    speed_ref_rad_s_ = speed_mech_rad_s;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
auto Motor<Observer, Modulator, Controller, HalPolicy>::tick(float dt_s) noexcept -> ErrorCode {
    if (!(dt_s > 0.0F)) {
        status_ = ErrorCode::Param;
        return status_;
    }

    if (status_ != ErrorCode::Ok) {
        return status_;
    }

    if (sm_.state() != core::MotorState::Run) {
        // Not latched: "you have not started me" is a usage error the caller fixes by calling
        // start(), not a fault. Latching it would leave a C-API instance dead forever, because the
        // C surface has no clear_fault().
        return ErrorCode::State;
    }

    const Uvw<float> i_uvw = hal_->current_sense.read_uvw();
    const Ab<float> i_ab = core::clarke(i_uvw);

    // A sensor may report the angle but not the speed (`NaN` = not supported, see the HAL
    // contract): the angle then comes from the sensor and the speed from the observer policy.
    const bool has_sensor = hal_->position.has_sensor();
    const float sensor_speed = has_sensor ? hal_->position.velocity_rad_s() : 0.0F;
    const bool speed_from_observer = !has_sensor || std::isnan(sensor_speed);

    float theta_e = 0.0F;
    float speed_mech = 0.0F;
    if (speed_from_observer) {
        const core::ObsState observed = policies_.observer.update(v_ab_prev_, i_ab, dt_s);
        speed_mech = observed.omega_e / static_cast<float>(config_.pole_pairs);
        theta_e =
            has_sensor ? (hal_->position.angle_mech_rad() * static_cast<float>(config_.pole_pairs)) : observed.theta_e;
    } else {
        theta_e = hal_->position.angle_mech_rad() * static_cast<float>(config_.pole_pairs);
        speed_mech = sensor_speed;
    }

    if (mode_ == ReferenceMode::Speed) {
        iq_ref_a_ =
            core::clamp(policies_.speed.step(speed_ref_rad_s_, speed_mech, dt_s), -config_.i_max_a, config_.i_max_a);
    }

    const Dq<float> i_dq = measure_dq(i_uvw, theta_e);
    const float v_d = policies_.current_d.step(0.0F, i_dq.d, dt_s);
    const float v_q = policies_.current_q.step(iq_ref_a_, i_dq.q, dt_s);
    const Dq<float> v_dq{v_d, v_q};
    const Ab<float> v_ab = core::inv_park(v_dq, theta_e);
    const core::SvpwmResult modulated = policies_.modulator.modulate(v_ab, hal_->dc_link.voltage_v());

    hal_->pwm.write_duty(modulated.duty);
    v_ab_prev_ = v_ab;
    telemetry_ = Telemetry{speed_mech, i_dq.d, i_dq.q};

    if (modulated.status != ErrorCode::Ok) {
        status_ = modulated.status;
    }
    return status_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
auto Motor<Observer, Modulator, Controller, HalPolicy>::state() const noexcept -> core::MotorState {
    return sm_.state();
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
auto Motor<Observer, Modulator, Controller, HalPolicy>::mode() const noexcept -> ReferenceMode {
    return mode_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
auto Motor<Observer, Modulator, Controller, HalPolicy>::telemetry() const noexcept -> Telemetry {
    return telemetry_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
auto Motor<Observer, Modulator, Controller, HalPolicy>::status() const noexcept -> ErrorCode {
    return status_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
auto Motor<Observer, Modulator, Controller, HalPolicy>::observer() noexcept -> Observer& {
    return policies_.observer;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy>
auto Motor<Observer, Modulator, Controller, HalPolicy>::config() const noexcept -> const MotorConfig& {
    return config_;
}

} // namespace motor

} // namespace foc
