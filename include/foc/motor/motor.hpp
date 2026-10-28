// Conan::ImportStart
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <foc/core/alignment.hpp>
#include <foc/core/limits.hpp>
#include <foc/core/feedforward.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/position.hpp>
#include <foc/core/reference.hpp>
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
 * @brief [en] Read-back of the last control step, plus the worst tick interval seen so far.
 * @brief [zh] 最近一个控制步的读回值，外加迄今见过的最差 tick 间隔。
 * @note [en] `max_period_us` is 0 unless the HalPolicy exposes a `clock`: the library never invents a time base, it
 *            only measures the one the platform gives it. It is an overrun indicator, not a scheduling input --
 *            `tick(dt)` takes its period from the caller.
 * @note [zh] 除非 HalPolicy 暴露了 `clock`，`max_period_us` 恒为 0：库从不自造时基，只测量平台给出的 时基。它是过载指示
 *            器而非调度输入——`tick(dt)` 的周期来自调用方。
 * @since 0.1.0
 * @exporter
 */
struct Telemetry final {
    float speed_mech_rad_s;      ///< [en] mechanical speed in rad/s. [zh] 机械速度，rad/s。
    float id_a;                  ///< [en] measured d-axis current in A. [zh] 实测 d 轴电流，A。
    float iq_a;                  ///< [en] measured q-axis current in A. [zh] 实测 q 轴电流，A。
    std::uint32_t max_period_us; ///< [en] worst tick interval, us. [zh] 最差 tick 间隔，us。
};


/**
 * @brief [en] What the q-axis reference is driven by.
 * @brief [zh] q 轴给定由什么驱动。
 * @since 0.1.0
 * @exporter
 */
enum class ReferenceMode : std::uint8_t {
    Torque = 0,   ///< [en] iq is commanded directly. [zh] 直接给定 iq。
    Speed = 1,    ///< [en] the speed controller produces iq. [zh] 由速度环产生 iq。
    Position = 2, ///< [en] the position loop produces the speed setpoint. [zh] 由位置环产生速度给定。
};


/**
 * @brief [en] One move: where to go, how hard you may push, and what to add on top.
 * @brief [zh] 一次运动：去哪里、允许用多大力、以及额外叠加多少。
 * @note [en] The limit and scale fields default to 0, which means "use what the controllers were configured with". The
 *            mechanism's own limits stay in `core::MotionProfile::Config`; these are the *move's*, and a non-zero value
 *            replaces the configured one for this move only, without any restore step. The three scales are the
 *            compliance knobs. `kp_scale` sets how much the joint resists being moved; `ilimit_scale` sets how much
 *            torque the speed integrator may keep holding, so a joint pushed by hand gives way instead of fighting
 *            back; `kd_scale` sets the damping around it. They scale gains, never limits, so a compliant joint still
 *            cannot exceed `max_current_a` or the motion limits.
 *
 * @note [zh] 限制与缩放字段默认 0，含义是"用控制器自身配置的那个"。机构自己的限制仍在 `core::MotionProfile::Config`
 *            里；这些是**这次运动**的，非零值只在本次运动内替换配置值， 不需要任何恢复步骤。
 *
 *     三个缩放是柔顺旋钮。`kp_scale` 决定关节抵抗被推动的程度；`ilimit_scale` 决定速度 积分器被允许保有多少转矩，于是被
 *     手推的关节会退让而不是对抗；`kd_scale` 决定其周围的阻尼。 它们缩放增益而非限值，因此柔顺的关节仍然不能越过
 *     `max_current_a` 或运动限制。
 * @since 0.1.0
 * @exporter
 */
struct Command final {
    float position_rad;          ///< [en] target mechanical angle in rad. [zh] 目标机械角，rad。
    float max_current_a;         ///< [en] iq ceiling, 0 = nameplate. [zh] 本次 iq 上限，0 = 铭牌值。
    float feedforward_current_a; ///< [en] iq added after the speed loop. [zh] 速度环后叠加的 iq。
    float max_velocity_rad_s; ///< [en] per-move speed ceiling, 0 = controller's. [zh] 本次速度上限，0 = 用控制器配置。
    float max_accel_rad_s2;   ///< [en] per-move rate limit, 0 = controller's. [zh] 本次加速度上限，0 = 用控制器配置。
    float kp_scale;           ///< [en] stiffness scale, 0 = configured. [zh] 刚度缩放，0 = 用配置值。
    float kd_scale;           ///< [en] speed-loop gain scale, 0 = configured. [zh] 速度环增益缩放，0 = 用配置值。
    float ilimit_scale;       ///< [en] speed integral scale, 0 = configured. [zh] 速度环积分缩放，0 = 用配置值。
};


/**
 * @brief [en] Policy-composed motor: no globals, no allocation, no virtual calls in the loop. All state lives in this
 *             object, so multiple instances are independent by construction.
 *
 * @brief [zh] 策略组合的电机：无全局状态、不分配、控制环内无虚函数调用。状态全在对象内， 因此多实例天然相互独立。
 *
 *     Binding contract:
 *
 *     - `Observer` provides `update(Ab<float>, Ab<float>, float) noexcept -> ObsState` (and optionally
 *     `seed_flux(Ab<float>)`, used at start()). - `Modulator` provides `modulate(Ab<float>, float) noexcept ->
 *     SvpwmResult`. - `Controller` provides `step(float, float, float) noexcept -> float` and `reset()`. -
 *     `RefGenerator` provides `generate(const ReferenceInput&) noexcept -> Dq<float>` (default
 *     `core::NullReferenceGenerator` = d axis at zero). - `FeedForward` provides `generate(const FeedforwardInput&)
 *     noexcept -> Dq<float>` (default `core::NullFeedforward` = no term). - `PositionControl` provides `step(setpoint,
 *     feedback, dt) noexcept -> float` + `reset()` and commands a *velocity*; the default `core::NullPositionControl`
 *     makes position mode answer `ErrorCode::NotImpl` instead of pretending. - `Aligner` provides `validate()`,
 *     `begin(pole_pairs) -> bool`, `step(dt, angle_mech) noexcept -> AlignOutput` and `reset()`; the default
 *     `core::NullAlignment` answers `false` from `begin()`, so `start()` goes straight from Align to Run. - `HalPolicy`
 *     exposes members `pwm`, `current_sense`, `position`, `dc_link`. 绑定契约：见上（Observer 可选 seed_flux，HalPolicy
 *     的成员名固定）。
 *
 * @context [en] tick() is the ISR entry: noexcept, no allocation, no I/O.
 * @context [zh] tick() 是 ISR 入口：noexcept、不分配、无 I/O。
 * @since 0.1.0
 * @exporter
 */
template <typename Observer, typename Modulator, typename Controller, typename HalPolicy,
          typename RefGenerator = core::NullReferenceGenerator, typename FeedForward = core::NullFeedforward,
          typename PositionControl = core::NullPositionControl, typename Aligner = core::NullAlignment>
class Motor final {
  public:
    /**
     * @brief [en] Policy instances owned by the motor (one per control axis plus the speed loop).
     * @brief [zh] 电机持有的策略实例（每个控制轴一个，外加速度环）。
     * @since 0.1.0
     */
    struct Policies final {
        Observer observer;
        Modulator modulator;
        Controller current_d;
        Controller current_q;
        Controller speed;
        RefGenerator reference{};
        FeedForward feed_forward{};
        PositionControl position{};
        Aligner align{};
    };

    Motor(MotorConfig config, Policies policies, HalPolicy& hal) noexcept;

    /**
     * @brief [en] Validate the configuration and configure the inverter.
     * @brief [zh] 校验配置并配置逆变器。
     * @return [en] Ok, or Param/State from the PWM contract.
     * @return [zh] Ok，或来自 PWM 契约的 Param/State。
     * @context [en] Initialisation only.
     * @context [zh] 仅初始化阶段。
     * @since 0.1.0
     */
    auto init() noexcept -> ErrorCode;

    /**
     * @brief [en] Idle -> Align -> Run, enable the inverter, seed a flux observer if it takes one.
     * @brief [zh] Idle -> Align -> Run，使能逆变器；若观测器支持则为其播种。
     * @note [en] With an aligner that has a sequence to run, the machine stays in Align and the caller keeps calling
     *            `tick()` until `state()` becomes Run; with the default `core::NullAlignment` the transition is
     *            immediate, as before.
     * @note [zh] 对齐器有流程要跑时，机器会停在 Align，调用方继续 `tick()` 直到 `state()` 变成 Run；使用默认的
     *            `core::NullAlignment` 时迁移立即完成，与从前一致。
     * @return [en] Ok, or the state-machine / PWM error.
     * @return [zh] Ok，或状态机 / PWM 错误。
     * @since 0.1.0
     */
    auto start() noexcept -> ErrorCode;

    /**
     * @brief [en] Disable the inverter and return to Idle.
     * @brief [zh] 关闭逆变器并回到 Idle。
     * @since 0.1.0
     */
    auto stop() noexcept -> ErrorCode;

    /**
     * @brief [en] Enter the latched Fault state and disable the inverter.
     * @brief [zh] 进入锁存 Fault 状态并关闭逆变器。
     * @since 0.1.0
     */
    void fault() noexcept;

    /**
     * @brief [en] Leave Fault (only legal from Fault) and reset the controllers.
     * @brief [zh] 退出 Fault（仅能从 Fault 离开）并复位控制器。
     * @since 0.1.0
     */
    auto clear_fault() noexcept -> ErrorCode;

    /**
     * @brief [en] Torque reference: iq in A, clamped to the nameplate limit.
     * @brief [zh] 转矩给定：iq，A，按铭牌限幅。
     * @since 0.1.0
     */
    void set_torque(float iq_a) noexcept;

    /**
     * @brief [en] Speed reference: mechanical rad/s, fed into the speed loop.
     * @brief [zh] 速度给定：机械角速度 rad/s，送入速度环。
     * @since 0.1.0
     */
    void set_speed(float speed_mech_rad_s) noexcept;

    /**
     * @brief [en] Position reference: mechanical angle in rad, closed by the position loop.
     * @brief [zh] 位置给定：机械角，rad，由位置环闭合。
     * @param position_rad [en] target in the same frame as `PositionSensor::angle_mech_rad()`.
     * @param position_rad [zh] 目标值，与 `PositionSensor::angle_mech_rad()` 同一坐标系。
     * @note [en] Requires a compiled-in `PositionControl` and a sensor; without either, `tick()` returns
     *            `ErrorCode::NotImpl` (not latched).
     * @note [zh] 需要编入 `PositionControl` 且存在传感器；缺任一条件时 `tick()` 返回 `ErrorCode::NotImpl`（不锁存）。
     * @since 0.1.0
     */
    void set_position(float position_rad) noexcept;

    /**
     * @brief [en] Position reference with a per-move current ceiling and a feedforward torque.
     * @brief [zh] 带本次运动电流上限与前馈转矩的位置给定。
     * @param command [en] the move to execute.
     * @param command [zh] 要执行的运动。
     * @note [en] `feedforward_current_a` is where a gravity or friction term goes: it is added after the speed loop, so
     *            it does not fight the integrator, and it is clamped with everything else by the move's current
     *            ceiling.
     * @note [zh] `feedforward_current_a` 就是重力或摩擦项该去的地方：它加在速度环之后，所以不会和 积分器打架，并且和其
     *            余量一起受本次运动的电流上限约束。
     * @since 0.1.0
     */
    void set_command(const Command& command) noexcept;

    /**
     * @brief [en] One current-loop step: sense -> Clarke -> angle -> Park -> PI -> InvPark -> SVPWM.
     * @brief [zh] 一步电流环：采样 -> Clarke -> 角度 -> Park -> PI -> InvPark -> SVPWM。
     * @param dt_s [en] step length in s; <= 0 is rejected (status Param).
     * @param dt_s [zh] 步长，s；<= 0 被拒绝（状态置 Param）。
     * @return [en] Ok, Param, or State (not running).
     * @return [zh] Ok、Param 或 State（未运行）。
     * @since 0.1.0
     */
    auto tick(float dt_s) noexcept -> ErrorCode;

    /**
     * @brief [en] Current mode state.
     * @brief [zh] 当前模式状态。
     * @since 0.1.0
     */
    [[nodiscard]] auto state() const noexcept -> core::MotorState;

    /**
     * @brief [en] Which reference drives the q axis.
     * @brief [zh] q 轴由哪种给定驱动。
     * @since 0.1.0
     */
    [[nodiscard]] auto mode() const noexcept -> ReferenceMode;

    /**
     * @brief [en] Telemetry of the last tick.
     * @brief [zh] 最近一次 tick 的遥测。
     * @since 0.1.0
     */
    [[nodiscard]] auto telemetry() const noexcept -> Telemetry;

    /**
     * @brief [en] Sticky status of the last operation.
     * @brief [zh] 最近一次操作的粘滞状态。
     * @since 0.1.0
     */
    [[nodiscard]] auto status() const noexcept -> ErrorCode;

    /**
     * @brief [en] Mutable access to the observer (used to seed it from the align routine or a test).
     * @brief [zh] 观测器的可变访问（供对齐流程或测试播种）。
     * @since 0.1.0
     */
    [[nodiscard]] auto observer() noexcept -> Observer&;

    /**
     * @brief [en] Hand updated machine parameters to the policies that can take them.
     * @brief [zh] 把更新后的机器参数交给能接受的策略。
     * @param params [en] the machine as it actually is (identified at commissioning, or re-read later).
     * @param params [zh] 机器实际的 样子（调试时辨识得到，或之后重读）。
     * @note [en] Forwarded, not stored: `Motor` does not model the machine itself, so keeping a copy would create a
     *            second source of truth for numbers that belong to the policies. A slot without the hook simply keeps
     *            what it was built with.
     * @note [zh] 只转发、不保存：`Motor` 自己不建模机器，存一份会给本该属于策略的数字制造第二个真值来源。 没有该钩子的
     *            槽位保持构造时的值。
     * @since 0.1.0
     */
    void apply_params(const core::MachineParams& params) noexcept;

    /**
     * @brief [en] Mutable access to the reference generator (tuning, diagnostics, tests).
     * @brief [zh] 参考生成器的可变访问（整定、诊断、测试）。
     * @since 0.1.0
     */
    [[nodiscard]] auto reference() noexcept -> RefGenerator&;

    /**
     * @brief [en] Mutable access to the feedforward stage (tuning, diagnostics, tests).
     * @brief [zh] 前馈阶段的可变访问（整定、诊断、测试）。
     * @since 0.1.0
     */
    [[nodiscard]] auto feed_forward() noexcept -> FeedForward&;

    /**
     * @brief [en] Mutable access to the aligner, which is where the captured offset lives.
     * @brief [zh] 对齐器的可变访问，捕获到的偏置就在这里。
     * @since 0.1.0
     */
    [[nodiscard]] auto aligner() noexcept -> Aligner&;

    /**
     * @brief [en] Read-only view of the aligner (the captured offset lives here).
     * @brief [zh] 对齐器的只读视图（捕获到的偏置在这里）。
     * @since 0.1.0
     */
    [[nodiscard]] auto aligner() const noexcept -> const Aligner&;

    /**
     * @brief [en] Mutable access to the position slot (retuning without rebuilding the motor).
     * @brief [zh] 位置槽位的可变访问（不重建电机即可重新整定）。
     * @since 0.1.0
     */
    [[nodiscard]] auto position() noexcept -> PositionControl&;

    /**
     * @brief [en] Nameplate parameters.
     * @brief [zh] 铭牌参数。
     * @since 0.1.0
     */
    [[nodiscard]] auto config() const noexcept -> const MotorConfig&;

  private:
    /**
     * @brief [en] Keep one policy output inside the nameplate. A non-finite value is a policy bug, not a physical
     *             quantity: it collapses to 0 instead of poisoning the integrators.
     * @brief [zh] 把策略的单个输出限制在铭牌范围内。非有限值是策略的缺陷而非物理量：归零而不是让 它污染积分器。
     */
    [[nodiscard]] static auto sanitize_reference(float value, float i_max_a) noexcept -> float {
        return std::isfinite(value) ? core::clamp(value, -i_max_a, i_max_a) : 0.0F;
    }

    /**
     * @brief [en] Apply the nameplate clamp to both axes of a reference-generator result.
     * @brief [zh] 对参考生成器的结果在两个轴上应用铭牌限幅。
     */
    [[nodiscard]] static auto clamp_reference(Dq<float> ref, float i_max_a) noexcept -> Dq<float> {
        return Dq<float>{sanitize_reference(ref.d, i_max_a), sanitize_reference(ref.q, i_max_a)};
    }

    /**
     * @brief [en] Hand the captured offset to the sensor, when the sensor can take it.
     * @brief [zh] 把捕获到的偏置交给传感器——前提是传感器能接收。
     * @note [en] Both halves are optional and detected at compile time: an aligner that reports no offset, or a sensor
     *            whose offset is owned by the platform, simply does not get written to. Nothing is inferred and no
     *            default is invented; the value comes from the aligner's own capture.
     * @note [zh] 两半都是可选且编译期探测的：不报偏置的对齐器、或偏置由平台掌管的传感器，都不会被 写入。这里不做任何推
     *            断、不编造默认值；值就是对齐器自己捕获的那个。
     */
    void apply_alignment_offset() noexcept {
        if constexpr (core::concepts::has_mech_offset_v<decltype(hal_->position)> &&
                      core::concepts::has_align_offset_v<Aligner>) {
            hal_->position.set_offset_mech_rad(policies_.align.offset_mech_rad());
        }
    }

    MotorConfig config_;
    Policies policies_;
    HalPolicy* hal_;
    core::StateMachine sm_;
    Telemetry telemetry_;
    ErrorCode status_;
    ReferenceMode mode_;
    float iq_ref_a_;
    float speed_ref_rad_s_;
    float speed_elapsed_s_;
    bool speed_step_due_;
    float position_ref_rad_;
    Command command_;
    float speed_ref_from_position_rad_s_;
    float position_elapsed_s_;
    bool position_step_due_;
    std::uint32_t max_period_us_;
    std::uint64_t last_clock_us_;
    bool clock_seeded_;
    Ab<float> v_ab_prev_;
};

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner>::Motor(
    MotorConfig config, Policies policies, HalPolicy& hal) noexcept
    : config_(config), policies_(policies), hal_(&hal), sm_(), telemetry_{0.0F, 0.0F, 0.0F, 0U}, status_(ErrorCode::Ok),
      mode_(ReferenceMode::Torque), iq_ref_a_(0.0F), speed_ref_rad_s_(0.0F), speed_elapsed_s_(0.0F),
      speed_step_due_(true), position_ref_rad_(0.0F), command_{}, speed_ref_from_position_rad_s_(0.0F),
      position_elapsed_s_(0.0F), position_step_due_(true), max_period_us_(0U), last_clock_us_(0U), clock_seeded_(false),
      v_ab_prev_{} {}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::init() noexcept -> ErrorCode {
    if (!(config_.rs >= 0.0F) || !(config_.pwm_freq_hz > 0.0F) || (config_.pole_pairs == 0U) ||
        !(config_.speed_loop_period_s >= 0.0F) || !(config_.position_loop_period_s >= 0.0F)) {
        status_ = ErrorCode::Param;
        return status_;
    }

    status_ = hal_->pwm.configure(config_.pwm_freq_hz, config_.deadtime_ns);
    return status_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::start() noexcept -> ErrorCode {
    if (status_ != ErrorCode::Ok) {
        return status_;
    }

    // An unusable alignment recipe is a configuration error and must surface before the inverter is
    // enabled: starting a sequence with an invented current would pull the rotor with a number the
    // caller never chose. `NotImpl` is the third answer and means "no recipe was requested" -- the
    // machine then simply goes straight to Run and this whole feature stays inert.
    // 不可用的对齐配方是配置错误，必须在逆变器使能前暴露：用一个凭空造的电流去跑流程，等于用调用
    // 方从未选过的数字去拉转子。`NotImpl` 是第三种答案，含义是“没人请求过配方”——此时机器直接进
    // Run，整个特性保持静默。
    const ErrorCode recipe = policies_.align.validate();
    if ((recipe != ErrorCode::Ok) && (recipe != ErrorCode::NotImpl)) {
        status_ = ErrorCode::Param;
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

    // Phase the decimation so the first tick runs the speed loop immediately, instead of holding a
    // zero current reference for a whole period. The overrun statistic restarts with the motor.
    // 相位对齐：让第一个 tick 立刻跑速度环，而不是在整整一个周期里保持零电流给定。过载统计随电机
    // 启动重新开始。
    speed_elapsed_s_ = 0.0F;
    speed_step_due_ = true;
    position_elapsed_s_ = 0.0F;
    position_step_due_ = true;
    max_period_us_ = 0U;
    last_clock_us_ = 0U;
    clock_seeded_ = false;

    // Only an aligner with a sequence to run keeps the machine in Align. `begin()` answering false
    // is the "nothing to do here" case (absolute encoder, stored offset) and must leave the old
    // immediate Idle -> Align -> Run path intact.
    // 只有真有流程要跑的对齐器才把机器留在 Align。`begin()` 返回 false 就是“这里无事可做”（绝对值
    // 编码器、偏置已存），必须保留原来 Idle -> Align -> Run 一步到位的路径。
    if (!policies_.align.begin(config_.pole_pairs)) {
        return sm_.request(core::MotorState::Run);
    }
    return ErrorCode::Ok;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::stop() noexcept -> ErrorCode {
    hal_->pwm.disable();
    // An aborted alignment must not resume where it stopped: the rotor will have been moved by
    // whatever held it in between.
    // 中断的对齐不得从断点继续：期间转子的实际位置已被别的东西挪动过。
    policies_.align.reset();
    return sm_.request(core::MotorState::Idle);
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::fault() noexcept {
    hal_->pwm.disable();
    static_cast<void>(sm_.request(core::MotorState::Fault));
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::clear_fault() noexcept -> ErrorCode {
    const ErrorCode rc = sm_.clear_fault();
    if (rc == ErrorCode::Ok) {
        policies_.current_d.reset();
        policies_.current_q.reset();
        policies_.speed.reset();
        policies_.position.reset();
        policies_.align.reset();
        iq_ref_a_ = 0.0F;
        status_ = ErrorCode::Ok;
    }
    return rc;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner>::set_torque(
    float iq_a) noexcept {
    mode_ = ReferenceMode::Torque;
    iq_ref_a_ = core::clamp(iq_a, -config_.i_max_a, config_.i_max_a);
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner>::set_speed(
    float speed_mech_rad_s) noexcept {
    mode_ = ReferenceMode::Speed;
    speed_ref_rad_s_ = speed_mech_rad_s;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::set_position(float position_rad) noexcept {
    set_command(Command{position_rad, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F});
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::set_command(const Command& command) noexcept {
    mode_ = ReferenceMode::Position;
    command_ = command;
    position_ref_rad_ = command.position_rad;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner>::tick(
    float dt_s) noexcept -> ErrorCode {
    // Optional overrun measurement: only when the platform gave us a clock. It never feeds back into
    // the control law -- it exists so an ISR overrun becomes visible instead of silent.
    // 可选过载测量：仅当平台提供了时钟时启用。它从不参与控制律——存在的意义是让 ISR 过载可见而非
    // 静默。
    if constexpr (core::concepts::has_clock_v<HalPolicy>) {
        const std::uint64_t now_us = hal_->clock.now_us();
        if (clock_seeded_) {
            const std::uint64_t delta_us = now_us - last_clock_us_;
            const std::uint32_t clamped = (delta_us > UINT32_MAX) ? UINT32_MAX : static_cast<std::uint32_t>(delta_us);
            if (clamped > max_period_us_) {
                max_period_us_ = clamped;
            }
        }
        last_clock_us_ = now_us;
        clock_seeded_ = true;
    }

    if (!(dt_s > 0.0F)) {
        status_ = ErrorCode::Param;
        return status_;
    }

    // A hardware fault is the one condition this layer can neither repair nor ignore, and the only
    // one that must win over every other status: latch it before anything else can mask it.
    // 硬件故障是本层既修不好、也不能忽略的条件，也是唯一应压过其余状态的：先锁存，别的状态不得掩盖它。
    if (hal_->pwm.fault()) {
        fault();
        status_ = ErrorCode::Fault;
        return status_;
    }

    if (status_ != ErrorCode::Ok) {
        return status_;
    }

    const bool aligning = (sm_.state() == core::MotorState::Align);
    if (!aligning && (sm_.state() != core::MotorState::Run)) {
        // Not latched: "you have not started me" is a usage error the caller fixes by calling
        // start(), not a fault. Latching it would turn a calling-order mistake into a state the
        // caller has to be told how to clear.
        return ErrorCode::State;
    }

    const Uvw<float> i_uvw = hal_->current_sense.read_uvw();
    const Ab<float> i_ab = core::clarke(i_uvw);

    // One bus reading per step: the reference generator and the modulator must reason about the
    // same voltage, otherwise a sag is invisible to whichever one happened to read second.
    // 每步只读一次母线：参考生成器与调制器必须看到同一个电压，否则母线跌落对后读的那一方不可见。
    const float v_bus_v = hal_->dc_link.voltage_v();

    // A sensor may report the angle but not the speed (`NaN` = not supported, see the HAL
    // contract): the angle then comes from the sensor and the speed from the observer policy.
    const bool has_sensor = hal_->position.has_sensor();
    const float sensor_speed = has_sensor ? hal_->position.velocity_rad_s() : 0.0F;
    const float angle_mech = has_sensor ? hal_->position.angle_mech_rad() : 0.0F;
    const bool speed_from_observer = !has_sensor || std::isnan(sensor_speed);

    float theta_e = 0.0F;
    float speed_mech = 0.0F;
    Dq<float> i_dq{};
    Dq<float> i_ref{};
    Dq<float> v_ff{};
    bool alignment_finished = false;

    if (aligning) {
        // Alignment forces the angle. The field has to stay put in the *stator* frame while the
        // rotor travels to it, and the sensor angle is precisely the unknown being measured, so it
        // cannot be the input here. Regulating id against a wrong angle instead yields a constant
        // torque error of sin(offset) and spins the machine rather than aligning it.
        // 对齐必须强制角度。转子走过去的过程中磁场要固定在**定子**坐标系里，而传感器角度正是待测的
        // 未知量，不能作为这里的输入。改用错角度去调 id 会得到恒为 sin(offset) 的转矩误差，结果是把
        // 电机转起来而不是对齐。
        const core::AlignOutput align_out = policies_.align.step(dt_s, angle_mech);
        if (align_out.failed) {
            // Outputs off and latched: the rotor stopped somewhere nobody has measured, so continuing
            // to modulate would be guessing at the commutation angle.
            // 关输出并锁存：转子停在一个无人测量过的地方，继续调制等于猜换相角。
            fault();
            status_ = ErrorCode::Fault;
            return status_;
        }
        theta_e = align_out.theta_e_rad;
        i_ref = Dq<float>{sanitize_reference(align_out.id_a, config_.i_max_a), 0.0F};
        // The observer is not stepped while the angle is forced -- its input would be the forced
        // vector, not the rotor -- so its speed output would be fiction; report the sensor's.
        // 强制角度期间不推进观测器——它的输入是强制矢量而非转子，速度输出会是编的；改报传感器值。
        speed_mech = std::isnan(sensor_speed) ? 0.0F : sensor_speed;
        i_dq = measure_dq(i_uvw, theta_e);
        if (align_out.finished) {
            apply_alignment_offset();
            alignment_finished = true;
        }
    } else {
        if (speed_from_observer) {
            const core::ObsState observed = policies_.observer.update(v_ab_prev_, i_ab, dt_s);
            speed_mech = observed.omega_e / static_cast<float>(config_.pole_pairs);
            theta_e = has_sensor ? (angle_mech * static_cast<float>(config_.pole_pairs)) : observed.theta_e;
        } else {
            theta_e = angle_mech * static_cast<float>(config_.pole_pairs);
            speed_mech = sensor_speed;
        }

        // Position mode needs both a position loop and an angle to close it on. Either one missing is
        // a capability the build does not have, so it is reported as `NotImpl` and deliberately
        // *not* latched: a missing feature is not a fault the caller has to clear.
        // 位置模式既需要位置环，也需要可闭环的角度。缺任何一项都属于“这次构建没有这个能力”，因此报
        // `NotImpl` 且**故意不锁存**：缺功能不是调用方需要清除的故障。
        if (mode_ == ReferenceMode::Position) {
            if constexpr (std::is_same_v<PositionControl, core::NullPositionControl>) {
                return ErrorCode::NotImpl;
            } else {
                if (!has_sensor) {
                    return ErrorCode::NotImpl;
                }
                position_elapsed_s_ += dt_s;
                if (position_step_due_ || (position_elapsed_s_ >= config_.position_loop_period_s)) {
                    // The move's limits travel with the request, so the controller shapes this
                    // move without being rebuilt and without remembering anything for the next one.
                    // 本次运动的限制随请求一起传下去，于是控制器塑造这一次运动，既不必重建，也不为
                    // 下一次记住任何东西。
                    const core::PositionInput request{position_ref_rad_, command_.max_velocity_rad_s,
                                                      command_.max_accel_rad_s2, command_.kp_scale};
                    speed_ref_from_position_rad_s_ = policies_.position.step(request, angle_mech, position_elapsed_s_);
                    position_elapsed_s_ = 0.0F;
                    position_step_due_ = false;
                }
            }
        }

        if ((mode_ == ReferenceMode::Speed) || (mode_ == ReferenceMode::Position)) {
            // Decimated by elapsed time, not by a tick count: a divider only means something relative
            // to a tick rate, while a period is physical and survives a change of control frequency.
            // With a period of 0 the accumulator always clears, so every tick runs the loop exactly
            // as before.
            // 按累计时间降采样，而不是按 tick 计数：分频数只有相对 tick 频率才有意义，而周期是物理量，
            // 控制频率变了它依然成立。周期为 0 时累加器每次都清零，于是每个 tick 都跑环，行为与从前
            // 完全一致。
            speed_elapsed_s_ += dt_s;
            if (speed_step_due_ || (speed_elapsed_s_ >= config_.speed_loop_period_s)) {
                const bool position_mode = (mode_ == ReferenceMode::Position);
                const float speed_setpoint = position_mode ? speed_ref_from_position_rad_s_ : speed_ref_rad_s_;
                const float feedforward = position_mode ? command_.feedforward_current_a : 0.0F;
                const float ceiling = (position_mode && (command_.max_current_a > 0.0F))
                                          ? std::min(command_.max_current_a, config_.i_max_a)
                                          : config_.i_max_a;
                const bool scaling_requested =
                    position_mode && ((command_.kd_scale != 0.0F) || (command_.ilimit_scale != 0.0F));
                float demanded_iq = 0.0F;
                if constexpr (core::concepts::is_speed_control_v<Controller>) {
                    demanded_iq =
                        policies_.speed.step(core::SpeedInput{speed_setpoint, command_.kd_scale, command_.ilimit_scale},
                                             speed_mech, speed_elapsed_s_);
                } else {
                    // The slot cannot scale its gains. Saying so is the only honest option: silently
                    // ignoring the request would leave a joint that was told to be compliant and is
                    // not -- which is discovered by a human pushing against it.
                    // 该槽位无法缩放自身增益。明说出去是唯一诚实的做法：静默忽略请求，会留下一个被告知
                    // 要柔顺、实际却不柔顺的关节——而这件事只能靠人去推它才发现。
                    if (scaling_requested) {
                        return ErrorCode::NotImpl;
                    }
                    demanded_iq = policies_.speed.step(speed_setpoint, speed_mech, speed_elapsed_s_);
                }
                iq_ref_a_ = core::clamp(demanded_iq + feedforward, -ceiling, ceiling);
                speed_elapsed_s_ = 0.0F;
                speed_step_due_ = false;
            }
        }

        const float omega_e = speed_mech * static_cast<float>(config_.pole_pairs);
        i_dq = measure_dq(i_uvw, theta_e);
        const core::ReferenceInput ref_in{iq_ref_a_, omega_e, v_bus_v, i_dq};
        i_ref = clamp_reference(policies_.reference.generate(ref_in), config_.i_max_a);

        // The feedforward is a summation node after the PI, not part of it: that keeps
        // `Controller::step(ref, fb, dt)` honest and lets the two axes be decoupled without the PI
        // ever seeing the other axis. It is deliberately skipped during alignment: the flux terms it
        // carries are computed from a speed that is not meaningful while the angle is forced.
        // 前馈是 PI 之后的求和节点而非 PI 的一部分：`Controller::step(ref, fb, dt)` 因此保持诚实，
        // 两轴解耦也不需要 PI 看到另一轴。对齐期间故意跳过：它携带的磁链项依赖的速度在强制角度时
        // 没有意义。
        v_ff = policies_.feed_forward.generate(core::FeedforwardInput{omega_e, i_dq});
    }

    const float v_d = policies_.current_d.step(i_ref.d, i_dq.d, dt_s) + v_ff.d;
    const float v_q = policies_.current_q.step(i_ref.q, i_dq.q, dt_s) + v_ff.q;
    const Dq<float> v_dq{v_d, v_q};
    Ab<float> v_ab = core::inv_park(v_dq, theta_e);

    // An observer that injects a carrier owns the voltage it injects: adding it here, as the last
    // thing before modulation, is the only place where "what is applied this period" and "what the
    // observer will demodulate next period" are guaranteed to be the same number.
    // 注入载波的观测器拥有它注入的那份电压：在这里、调制前的最后一步叠加，是唯一能保证“本周期实际施加
    // 的量”与“观测器下一周期要解调的量”是同一个数字的地方。
    //
    // Gated on the same condition that gates the observer itself. While aligning the angle is forced
    // and update() is not called, so the injection phase would stop advancing and a frozen carrier
    // would be a DC bias on the alignment current.
    // 与观测器本身同一个门控条件。对齐期间角度被强制、不调用 update()，注入相位就不再推进，冻结的载波
    // 会变成对齐电流上的直流偏置。
    if constexpr (core::concepts::has_injection_v<Observer>) {
        if (!aligning) {
            const Ab<float> injected = policies_.observer.injection_ab();
            v_ab = Ab<float>{static_cast<float>(v_ab.alpha + injected.alpha),
                             static_cast<float>(v_ab.beta + injected.beta)};
        }
    }
    // The modulator is told what it is producing *and* what the phases are carrying: a dead-time
    // compensator in front of it needs the second half, and a plain SVPWM ignores it.
    // 调制器同时被告知"要产生什么"与"各相在流什么"：它前面的死区补偿器需要后半部分，而纯 SVPWM 会忽略。
    core::SvpwmResult modulated{};
    if constexpr (core::concepts::is_modulator_v<Modulator>) {
        modulated = policies_.modulator.modulate(core::ModulatorInput{v_ab, v_bus_v, i_uvw});
    } else {
        modulated = policies_.modulator.modulate(v_ab, v_bus_v);
    }

    hal_->pwm.write_duty(modulated.duty);
    v_ab_prev_ = v_ab;
    telemetry_ = Telemetry{speed_mech, i_dq.d, i_dq.q, max_period_us_};

    if (modulated.status != ErrorCode::Ok) {
        status_ = modulated.status;
    }
    // The capture tick also drove zero current at the forced angle, so the hand-over happens on a
    // tick whose output was already harmless; from here the sensor's angle is trustworthy.
    // 捕获那一 tick 已在强制角下给了零电流，因此交接发生在一个输出本就无害的 tick 上；从此刻起
    // 传感器角度可信。
    if (alignment_finished) {
        static_cast<void>(sm_.request(core::MotorState::Run));
    }
    return status_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner>::state()
    const noexcept -> core::MotorState {
    return sm_.state();
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner>::mode()
    const noexcept -> ReferenceMode {
    return mode_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner>::telemetry()
    const noexcept -> Telemetry {
    return telemetry_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner>::status()
    const noexcept -> ErrorCode {
    return status_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::observer() noexcept -> Observer& {
    return policies_.observer;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::reference() noexcept -> RefGenerator& {
    return policies_.reference;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::feed_forward() noexcept -> FeedForward& {
    return policies_.feed_forward;
}


template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::aligner() noexcept -> Aligner& {
    return policies_.align;
}



template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner>::aligner()
    const noexcept -> const Aligner& {
    return policies_.align;
}



template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl,
           Aligner>::position() noexcept -> PositionControl& {
    return policies_.position;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner>::config()
    const noexcept -> const MotorConfig& {
    return config_;
}

} // namespace motor

} // namespace foc
