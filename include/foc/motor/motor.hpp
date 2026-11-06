// Conan::ImportStart
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <foc/core/alignment.hpp>
#include <foc/core/feedforward.hpp>
#include <foc/core/limits.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/position.hpp>
#include <foc/core/position_reference.hpp>
#include <foc/core/protection.hpp>
#include <foc/core/reference.hpp>
#include <foc/core/sm.hpp>
#include <foc/core/stall_start.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
#include <foc/motor/foc_loop.hpp>
#include <foc/motor/motor_config.hpp>
#include <type_traits>
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
    /**
     * @brief [en] mechanical speed in rad/s.
     * @brief [zh] 机械速度，rad/s。
     */
    float speed_mech_rad_s;
    float id_a; ///< [en] measured d-axis current in A. [zh] 实测 d 轴电流，A。
    float iq_a; ///< [en] measured q-axis current in A. [zh] 实测 q 轴电流，A。
    /**
     * @brief [en] worst tick interval, us.
     * @brief [zh] 最差 tick 间隔，us。
     */
    std::uint32_t max_period_us;
};


/**
 * @brief [en] What the q-axis reference is driven by.
 * @brief [zh] q 轴给定由什么驱动。
 * @since 0.1.0
 * @exporter
 */
enum class ReferenceMode : std::uint8_t {
    Torque = 0, ///< [en] iq is commanded directly. [zh] 直接给定 iq。
    Speed = 1,  ///< [en] the speed controller produces iq. [zh] 由速度环产生 iq。
    /**
     * @brief [en] the position loop produces the speed setpoint.
     * @brief [zh] 由位置环产生速度给定。
     */
    Position = 2,
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
    /**
     * @brief [en] target mechanical angle in rad.
     * @brief [zh] 目标机械角，rad。
     */
    float position_rad;
    /**
     * @brief [en] iq ceiling, 0 = nameplate.
     * @brief [zh] 本次 iq 上限，0 = 铭牌值。
     */
    float max_current_a;
    /**
     * @brief [en] iq added to the speed loop's output, in the speed and position modes. Ignored in torque mode, where
     *             the command *is* the reference.
     * @brief [zh] 加到速度环输出上的 iq，在速度与位置模式下生效。torque 模式下忽略——那里指令本身就是参考。
     * @note [en] This is the slot a cogging, friction or gravity term goes into. It used to be honoured in position
     *            mode only while the documentation said "after the speed loop", which meant a speed-mode host had no iq
     *            offset at all and no way to find that out from the header. Torque mode is deliberately excluded:
     *            `set_torque()` sets the reference directly, so a second additive field there would be two ways to say
     *            the same thing.
     * @note [zh] 齿槽、摩擦、重力项就走这个槽。它原先只在位置模式生效，而文档写的是「速度环后」，也就是说速度模式的宿主
     *            根本没有 iq 偏置、而且从头上看不出来。torque 模式是有意排除的：`set_torque()` 直接设置参考，再加一个可
     *            叠加字段就等于同一件事有两个说法。
     * @since 0.1.0
     */
    float feedforward_current_a;
    /**
     * @brief [en] per-move speed ceiling, 0 = controller's.
     * @brief [zh] 本次速度上限，0 = 用控制器配置。
     */
    float max_velocity_rad_s;
    /**
     * @brief [en] per-move rate limit, 0 = controller's.
     * @brief [zh] 本次加速度上限，0 = 用控制器配置。
     */
    float max_accel_rad_s2;
    /**
     * @brief [en] stiffness scale, 0 = configured.
     * @brief [zh] 刚度缩放，0 = 用配置值。
     */
    float kp_scale;
    /**
     * @brief [en] speed-loop gain scale, 0 = configured.
     * @brief [zh] 速度环增益缩放，0 = 用配置值。
     */
    float kd_scale;
    /**
     * @brief [en] speed integral scale, 0 = configured.
     * @brief [zh] 速度环积分缩放，0 = 用配置值。
     */
    float ilimit_scale;
};


/**
 * @brief [en] Policy-composed motor: no globals, no allocation, no virtual calls in the loop. All state lives in this
 *             object, so multiple instances are independent by construction.
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
 *     exposes members `pwm`, `current_sense`, `position`, `dc_link`.
 *
 * @brief [zh] 策略组合的电机：无全局状态、不分配、控制环内无虚函数调用。状态全在对象内， 因此多实例天然相互独立。
 *
 *     绑定契约：见上（Observer 可选 seed_flux，HalPolicy 的成员名固定）。
 *
 * @context [en] tick() is the ISR entry: noexcept, no allocation, no I/O.
 * @context [zh] tick() 是 ISR 入口：noexcept、不分配、无 I/O。
 * @since 0.1.0
 * @exporter
 */
template <typename Observer, typename Modulator, typename Controller, typename HalPolicy,
          typename RefGenerator = core::NullReferenceGenerator, typename FeedForward = core::NullFeedforward,
          typename PositionControl = core::NullPositionControl, typename Aligner = core::NullAlignment,
          typename Trace = core::NullTrace>
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
        /**
         * @brief [en] The trace channel. Defaults to the switch that costs nothing: an empty type with no-op calls.
         * @brief [zh] 轨迹通道。默认是不花代价的那个开关：一个空类型，调用皆空操作。
         * @since 0.1.8
         */
        Trace trace{};
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
     * @brief [en] Enter the latched Fault state with a cause, and disable the inverter.
     * @brief [zh] 带成因进入锁存 Fault 状态，并关闭逆变器。
     * @param code [en] what went wrong; `FaultCode::None` when the caller has no word for it.
     * @param code [zh] 出了什么问题；调用方没有对应词条时用 `FaultCode::None`。
     * @note [en] A fault state without a cause is a support call with no first question, so every raise point here
     *            states one. This is the C++ half of the vocabulary the C API already names through
     *            `foc_bridge_fault_name()`.
     * @note [zh] 没有成因的 Fault 状态是"无从问起"的售后电话，因此这里的每个触发点都要报一个。它就是 C API 已经通过
     *            `foc_bridge_fault_name()` 给出名字的那套词条的 C++ 半边。
     * @since 0.1.6
     */
    void fault(core::FaultCode code) noexcept;

    /**
     * @brief [en] The cause recorded by the last `fault()`, or `None` when there was no fault.
     * @brief [zh] 最近一次 `fault()` 记录的成因；没有故障时为 `None`。
     * @note [en] `clear_fault()` resets it together with the controllers, so a stale cause can never outlive the state
     *            it explains.
     * @note [zh] `clear_fault()` 会连同控制器一起把它复位，因此陈旧的成因不会比它所解释的状态活得更久。
     * @since 0.1.6
     */
    [[nodiscard]] auto fault_code() const noexcept -> core::FaultCode;

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
     * @note [en] `feedforward_current_a` is where a gravity or friction term goes: it is added *after* the speed loop,
     *            so it does not fight the integrator, and it is clamped with everything else by the move's current
     *            ceiling. `set_command()` is the position-mode entry point, but the field itself is now honoured in
     *            speed mode too -- see `Command`.
     * @note [zh] `feedforward_current_a` 就是重力或摩擦项该去的地方：它加在速度环**之后**，所以不会和积分器打架，并且和
     *            其余量一起受本次运动的电流上限约束。`set_command()` 是位置模式的入口，但这个字段现在在速度模式下也生效
     *            ——见 `Command`。
     * @since 0.1.0
     */
    void set_command(const Command& command) noexcept;

    /**
     * @brief [en] Update the feedforward iq **without changing the mode**.
     *
     *     `set_command()` switches the axis to position mode, which is why a speed-mode host had no way to use this
     *     field at all: the only writer also changed what was being controlled. A cogging or friction compensator
     *     updates its offset every period while the axis keeps doing whatever it was doing, so the setter is separate
     *     from the command.
     *
     *     The value **persists** until it is written again -- the same rule the speed reference follows -- because a
     *     compensator that has to re-state a constant gravity term every tick would be a worse interface than one that
     *     says it once. It is deliberately *not* cleared by `set_speed()`: clearing it there would break the one-shot
     *     usage.
     *
     * @brief [zh] 更新前馈 iq，**不改变模式**。
     *
     *     `set_command()` 会把轴切到位置模式，这就是速度模式宿主完全没法用这个字段的原因：唯一的写入者也改变了被控对
     *     象。齿槽或摩擦补偿器每个周期更新它的偏置，而轴继续做它原本在做的事，所以这个设置器与指令分开。
     *
     *     该值**保持**到下一次写入——与速度参考同一条规矩——因为一个必须每拍重述恒定重力项的补偿器，是比只说一次的接口更
     *     差的接口。它有意**不**被 `set_speed()` 清零：在那里清零会破坏"设一次"的用法。
     *
     * @param iq_a [en] the offset in A; clamped with the reference by the current ceiling, not here.
     * @param iq_a [zh] 偏置，A；它随参考一起被电流上限约束，不在这里约束。
     * @since 0.1.9
     */
    void set_feedforward_current(float iq_a) noexcept;

    /**
   * @brief [en] Start a breakaway sequence: raise the torque on a known ramp, report the instant the rotor moves, and
   *             leave that hand-over torque readable, so the speed loop starts from a known current instead of a
   *             wound-up integrator.
   * @brief [zh] 开始一条起步序列：以已知斜率抬升力矩，报告转子动起来的那一刻，并把交接力矩留在可读处，于是速度环从一个
   *             已知电流起步，而不是从一个饱和的积分起步。
   * @param recipe [en] the machine's breakaway estimate and the drive's limits.
   * @param recipe [zh] 机器的起动点估计与驱动的限幅。
   * @return [en] Ok; Param when the recipe cannot work (`max_torque_nm <= breakaway_nm`); State when the axis is
   *              aligning, faulted or not running. The phase stays `Idle` on both refusals.
   * @return [zh] Ok；配方不可能成立时为 Param（`max_torque_nm <= breakaway_nm`）；正在对齐、已故障或未运行时为 State。
   *              两种拒绝都会让阶段停在 `Idle`。
   * @note [en] Why this is not "let the speed loop do it": a speed PI *does* break a stuck axis loose -- by winding its
   *            integral up until the torque passes the breakaway point -- and that stored torque is exactly what drives
   *            the rotor past the setpoint the instant it lets go. This sequence does the same thing on purpose and
   *            reports where it happened. The axis is in torque mode while the sequence runs, so `stall_start_phase()`
   *            is the only thing that says whether the commanded torque is the sequence's or the caller's. When it
   *            reaches `Released` the caller owns the axis again: call `set_speed()` (or `set_torque()`) in the same
   *            control period, or the breakaway torque keeps being applied. `Failed` latches `FaultCode::Stall` -- a
   *            load the drive cannot move at its limit is a fact about the machine, and it outlives the call that found
   *            it.
   * @note [zh] 为什么这不是“让速度环自己干”：速度 PI 确实能把卡住的轴弄动，办法是把积分顶到力矩越过起动点—— 而转子一松
   *            脱，正是这份存下来的力矩把它推过给定。本序列把同一件事有意地做，并报告它发生在哪里。序列运行期间轴处于转
   *            矩模式，因此只有 `stall_start_phase()` 能说明当前指令力矩是序列的还是调用方的。到达 `Released` 后控制权
   *            回到调用方：必须在同一控制周期内调 `set_speed()`（或 `set_torque()`），否则起动转矩会继续施加。 `Failed`
   *            会锁存 `FaultCode::Stall`——驱动在限幅内搬不动的负载是关于这台机器的事实，它比发现它的那次调用活得更久。
   * @since 0.1.9
   */
    auto begin_stall_start(const core::StallStartConfig& recipe) noexcept -> ErrorCode;

    /**
   * @brief [en] Where the breakaway sequence is, and what it last commanded.
   * @brief [zh] 起步序列处在哪一步，以及它上一次下发了多少力矩。
   * @note [en] Read them together: the phase says whether the torque below is still owned by the sequence.
   * @note [zh] 两个要一起读：阶段说明它下面那个力矩是否仍归序列所有。
   * @since 0.1.9
   */
    [[nodiscard]] auto stall_start_phase() const noexcept -> core::StallStartPhase;

    /**
   * @brief [en] The torque the sequence last commanded, N*m. After `Released` this is the hand-over value.
   * @brief [zh] 序列上一次下发的力矩，N·m。进入 `Released` 之后它就是交接值。
   * @since 0.1.9
   */
    [[nodiscard]] auto stall_start_torque_nm() const noexcept -> float;

    /**
   * @brief [en] Whether the breakaway ramp is sitting at its ceiling, N*m.
   * @brief [zh] 起步斜坡是否已经贴在它的上限上。
   * @note [en] It is the difference between "climbing" and "climbing and out of authority": at the ceiling the only
   *            outcomes left are motion or `Failed`.
   * @note [zh] 它是“还在爬”与“爬到顶、没有余地”的区别：贴在上限上时，剩下的结局只有动起来或 `Failed`。
   * @since 0.1.9
   */
    [[nodiscard]] auto stall_start_limited() const noexcept -> bool;

    /**
   * @brief [en] Abandon the sequence and zero the torque command; the mode is left as the caller had it.
   * @brief [zh] 放弃序列并把指令力矩清零；模式保持调用方当前的样子。
   * @since 0.1.9
   */
    void cancel_stall_start() noexcept;

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
     * @brief [en] The trace channel, for reading back what the loop recorded.
     * @brief [zh] 轨迹通道，用于读回控制环记录的内容。
     * @note [en] With the default `NullTrace` this returns an empty object whose `size()` is 0 -- the accessor exists
     *            in both cases so a template never has to ask which one it has.
     * @note [zh] 默认的 `NullTrace` 下它返回一个 `size()` 为 0 的空对象——两种情况都有这个访问器，于是模板不必去问它拿到
     *            的是哪一个。
     * @since 0.1.8
     */
    [[nodiscard]] auto trace() noexcept -> Trace& { return policies_.trace; }
    [[nodiscard]] auto trace() const noexcept -> const Trace& { return policies_.trace; }

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
     * @brief [en] The turn counter and index origin behind a sensor-less position.
     * @brief [zh] 无感位置背后的圈计数与索引原点。
     * @return [en] The reference this motor keeps; `homed()` says whether a position command can be honoured and
     *              `lost_count()` whether the reconstruction may be trusted.
     * @return [zh] 该电机持有的参考；`homed()` 说明位置指令能否被兑现，`lost_count()` 说明重建是否可被信任。
     * @note [en] Read-only on purpose. The origin is established by the index pulse arriving at `hal.position`, not by
     *            a caller writing an angle: a settable origin would let a wrong number look exactly like a measured
     *            one.
     * @note [zh] 故意只读。原点由 `hal.position` 收到的索引脉冲建立，而不是由调用方写一个角度——可写的原点会让一个错误的
     *            数字看起来和测量出来的完全一样。
     * @since 0.1.6
     */
    [[nodiscard]] auto position_reference() const noexcept -> const core::PositionReference&;

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
     * @brief [en] How long after a forced-angle hand-over the observer gets before its speed is compared with the
     *             commanded one. 20 ms is the window the hand-over case already measured convergence in.
     * @brief [zh] 强制角交接之后留给观测器的时间，之后才把它的转速与命令值比较。20 ms 是交接用例已经实测过收敛的窗口。
     */
    static constexpr float kHandoverCheckS = 0.02F;

    /**
     * @brief [en] Below this fraction of the hand-over speed the start is reported as `LostSync`. Half is not a tuned
     *             number: a caught rotor is at the commanded rate, one that was never caught is at a few percent.
     * @brief [zh] 低于交接速度的这个比例就报 `LostSync`。一半不是调出来的数：被抓住的转子就在命令速率上，而从未被抓住的
     *             只有几个百分点。
     */
    static constexpr float kLostSyncSpeedFraction = 0.5F;

    /**
     * @brief [en] Below this forced rate the hand-over is a *hold*, not a pull, and there is nothing to lose sync with:
     *             a plain alignment hands over at zero speed by design and must not be judged against a fraction of
     *             zero.
     * @brief [zh] 低于这个强制速率时，交接是"按住"而不是"拖动"，没有同步可失：纯对齐按设计就在零速交接，不能用零的几分
     *             之几去判它。
     */
    static constexpr float kLostSyncMinHandoverRadS = 1.0F;

    /**
     * @brief [en] How long the loop may sit at full current without the machine moving the way it was asked to before
     *             `Stall` latches. 100 ms is long enough that a speed step's own transient passes through, and short
     *             enough that a jammed axis is stopped before the winding heats.
     * @brief [zh] 环路可以顶在满电流、而机器没有按指令方向运动多久之后锁存 `Stall`。100 ms 足够让一次速度阶跃的暂态穿过
     *             去，又足够短，不至于让卡住的轴先把绕组烤热。
     */
    static constexpr float kStallWindowS = 0.1F;

    /**
     * @brief [en] The current has to be at this fraction of the ceiling for the loop to be out of authority. Below it
     *             the drive is choosing not to push harder, which is not a stall.
     * @brief [zh] 电流要到天花板的这个比例，才算环路已经用尽权限。低于它说明驱动是选择不再用力，那不是堵转。
     */
    static constexpr float kStallCurrentFraction = 0.9F;

    /**
     * @brief [en] Speed below this fraction of the command -- or of the wrong sign -- counts as "not moving the way it
     *             was asked to". Reversal counts: a machine driven backwards under load is more stuck than one that is
     *             merely slow.
     * @brief [zh] 转速低于命令的这个比例、或者符号相反，都算"没有按指令方向运动"。反向也算：负载下被推着倒转的机器比只
     *             是慢一点的机器更"卡"。
     */
    static constexpr float kStallSpeedFraction = 0.25F;

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
    /**
     * @brief [en] Will the observer inject a carrier right now -- answered on the *mode*, not on the type.
     * @brief [zh] 观测器此刻会注入载波吗——按**模式**回答，而不是按类型。
     * @note [en] The distinction is the whole reason this exists: a slot holding an injection observer but running
     *            back-EMF injects nothing, so a type-level trait answers "yes" about a situation that is "no". The
     *            first version of the admission rule in `init()` used the type-level answer and refused every motor
     *            whose observer could switch modes -- thirty-eight failing cases' worth of wrong.
     * @note [zh] 这个区分正是它存在的全部理由：一个装着注入观测器却在跑反电势的槽位不注入任何东西，于是类型级特型对一个
     *            实际是"否"的情形回答"是"。`init()` 里准入门槛的第一版用了类型级答案，拒绝了每一台观测器可切换模式的电
     *            机——那是三十八个用例的错。
     */
    [[nodiscard]] auto injecting_now() const noexcept -> bool {
        if constexpr (!core::concepts::has_injection_v<Observer>) {
            return false;
        } else if constexpr (core::concepts::has_injecting_v<Observer>) {
            return policies_.observer.injecting();
        } else {
            return true;
        }
    }

    Policies policies_;
    HalPolicy* hal_;
    core::StateMachine sm_;
    Telemetry telemetry_;
    ErrorCode status_;
    core::FaultCode fault_code_{core::FaultCode::None};
    ReferenceMode mode_;
    float iq_ref_a_;
    float speed_ref_rad_s_;
    // These three stay float on purpose, and they are a different thing from the `core::ElapsedClock` the routines
    // use: none of them only grows. `speed_elapsed_s_` and `position_elapsed_s_` are phase decimators, cleared every
    // time the loop they gate runs, so their magnitude is bounded by the configured period; `stall_elapsed_s_` is
    // bounded by `kStallWindowS` because reaching it faults. A float is exact enough for a value that never leaves
    // hundredths of a second -- but a *timeout* is not bounded that way, which is why the routines that own one no
    // longer accumulate it in float (F-105), and `gates.py clocks` holds the line.
    // 这三个故意留作 float，与各流程用的 `core::ElapsedClock` 不是一回事：它们都**不是只增不减**的。
    // `speed_elapsed_s_` 与 `position_elapsed_s_` 是分频相位，它们所门控的环每跑一次就清零，量级被配置的周期兜住；
    // `stall_elapsed_s_` 被 `kStallWindowS` 兜住（达到即故障）。一个永不超出百分之几秒的量，float 足够精确——
    // 但**超时**不是这样被兜住的，所以拥有超时的流程不再用 float 累加它（F-105），并由 `gates.py clocks` 守住。
    float speed_elapsed_s_;
    bool speed_step_due_;
    float position_ref_rad_;
    Command command_;
    float speed_ref_from_position_rad_s_;
    float position_elapsed_s_;
    bool position_step_due_;
    /**
     * @brief Kept always, not only in the sensor-less case: a position command may arrive at any time, and building the counter lazily would mean the turn count starts from wherever the first position command happened to land. 始终保留，而不只是无感时才建：位置指令随时可能到来，懒建会让圈计数从"第一条位置指令恰好落在哪里"开始。
     */
    core::PositionReference position_reference_;
    std::uint32_t max_period_us_;
    std::uint64_t last_clock_us_;
    bool clock_seeded_;
    Ab<float> v_ab_prev_;
    // Post-hand-over check: the open-loop start forces the angle and cannot see the rotor, so it cannot tell whether
    // the rotor was caught. The observer can, one window later. Kept as a countdown in seconds rather than ticks so a
    // variable control period cannot stretch the window.
    // 交接后检查：开环起动强制角度、看不见转子，因此它无法判断转子是否被抓住。观测器在一个窗口之后可以。
    // 用秒而不是 tick 做倒计时，变化的控制周期就不能把这个窗口拉长。
    float handover_omega_e_rad_s_{0.0F};
    float handover_check_s_{0.0F};
    // Stall detector: the loop is at full current and the machine is not moving the way it was asked to. Counted in
    // seconds for the same reason as the hand-over window -- a tick count means nothing across control rates.
    // 堵转检测：环路顶在满电流，而机器没有按指令方向运动。用秒计数，理由与交接窗口相同——tick 数跨控制频率没有意义。
    float stall_elapsed_s_{0.0F};
    // Breakaway sequence. A concrete member rather than an eleventh policy slot:
    // the sequence is stateful, it needs `dt` and the measured speed every
    // period, and it is the *torque source* for a while -- which is what the mode
    // already expresses. An eleventh template parameter would have churned every
    // instantiation in `src/` and every test that names one, for a policy nobody
    // would ever substitute.
    // 起步序列。具体成员而不是第十一个策略槽：序列是有状态的，每周期都要 `dt`
    // 与实测转速，且它在一段时间内就是
    // **转矩来源**——而这正是模式已经表达的事。加第十一个模板参数会弄脏 `src/`
    // 里每个实例化与每个点名它的用例， 而那个策略没有人会去替换。
    core::StallStart stall_start_{};
    core::StallStartPhase stall_phase_{core::StallStartPhase::Idle};
    float stall_torque_nm_{0.0F};
    bool stall_limited_{false};
};

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner, Trace>::Motor(
    MotorConfig config, Policies policies, HalPolicy& hal) noexcept
    : config_(config), policies_(policies), hal_(&hal), sm_(), telemetry_{0.0F, 0.0F, 0.0F, 0U}, status_(ErrorCode::Ok),
      mode_(ReferenceMode::Torque), iq_ref_a_(0.0F), speed_ref_rad_s_(0.0F), speed_elapsed_s_(0.0F),
      speed_step_due_(true), position_ref_rad_(0.0F), command_{}, speed_ref_from_position_rad_s_(0.0F),
      position_elapsed_s_(0.0F), position_step_due_(true),
      position_reference_(core::PositionReference::Config{config.pole_pairs, 0.0F}), max_period_us_(0U),
      last_clock_us_(0U), clock_seeded_(false), v_ab_prev_{} {}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::init() noexcept -> ErrorCode {
    if (!(config_.rs >= 0.0F) || !(config_.pwm_freq_hz > 0.0F) || (config_.pole_pairs == 0U) ||
        !(config_.speed_loop_period_s >= 0.0F) || !(config_.position_loop_period_s >= 0.0F)) {
        status_ = ErrorCode::Param;
        return status_;
    }

    status_ = hal_->pwm.configure(config_.pwm_freq_hz, config_.deadtime_ns);
    return status_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::start() noexcept -> ErrorCode {
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

    // Refuse to assemble a drive that cannot say whether its power stage is faulted.
    // 拒绝装配一个说不出自己功率级有没有故障的驱动。
    //
    // This is the one capability in the HAL whose *absence* is not allowed to be silent, and it is refused here rather
    // than documented at the call site because the failure mode is invisible: a motor that never asks is
    // indistinguishable, from the outside, from one that asked and was told "all clear". The only value a missing
    // fault input could stand for is the dangerous one, so it stands for nothing and the axis does not start. A host
    // with no such pin says so with `driver::StubFaultInput` and never raises it -- one line, in the integrator's own
    // code, where a reviewer can see it. See `hal::FaultInput`.
    // 这是 HAL 里唯一一个**缺席也不许静默**的能力，而且在这里拒绝而不是在调用点写文档，是因为失效是看不见的：一台从不
    // 提问的电机，从外面看与一台问过并被告知"一切正常"的电机是一样的。一个缺席的故障输入唯一可能代表的值恰好是危险的
    // 那一个，所以它什么也不代表，轴不会启动。没有这根引脚的主控方用 `driver::StubFaultInput` 明说这件事、并且从不拉高
    // ——一行，写在集成方自己的代码里，评审看得见。见 `hal::FaultInput`。
    if constexpr (!core::concepts::has_fault_input_v<HalPolicy>) {
        status_ = ErrorCode::NotImpl;
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
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::stop() noexcept -> ErrorCode {
    hal_->pwm.disable();
    // An aborted alignment must not resume where it stopped: the rotor will have been moved by
    // whatever held it in between.
    // 中断的对齐不得从断点继续：期间转子的实际位置已被别的东西挪动过。
    policies_.align.reset();
    return sm_.request(core::MotorState::Idle);
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::fault() noexcept {
    fault(core::FaultCode::None);
}


template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::fault(core::FaultCode code) noexcept {
    fault_code_ = code;
    hal_->pwm.disable();
    static_cast<void>(sm_.request(core::MotorState::Fault));
}


template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::fault_code() const noexcept -> core::FaultCode {
    return fault_code_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::clear_fault() noexcept -> ErrorCode {
    const ErrorCode rc = sm_.clear_fault();
    if (rc == ErrorCode::Ok) {
        policies_.current_d.reset();
        policies_.current_q.reset();
        policies_.speed.reset();
        policies_.position.reset();
        policies_.align.reset();
        iq_ref_a_ = 0.0F;
        status_ = ErrorCode::Ok;
        fault_code_ = core::FaultCode::None;
        handover_check_s_ = 0.0F;
        stall_elapsed_s_ = 0.0F;
        // The sequence is reset with everything else: a recipe half-way up a ramp
        // describes a torque that was applied to an axis the fault has already
        // switched off, and resuming it after `clear_fault()` would apply the rest
        // of that ramp to a rotor that may have moved.
        // 序列与其余状态一起复位：爬到一半的配方描述的是施加在一个已被故障关断的轴上的力矩，`clear_fault()`
        // 之后 接着爬，会把剩下的斜坡施加到一个可能已经动过的转子上。
        stall_start_.reset();
        stall_phase_ = core::StallStartPhase::Idle;
        stall_torque_nm_ = 0.0F;
        stall_limited_ = false;
    }
    return rc;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::set_torque(float iq_a) noexcept {
    mode_ = ReferenceMode::Torque;
    iq_ref_a_ = core::clamp(iq_a, -config_.i_max_a, config_.i_max_a);
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::set_speed(float speed_mech_rad_s) noexcept {
    mode_ = ReferenceMode::Speed;
    speed_ref_rad_s_ = speed_mech_rad_s;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::set_feedforward_current(float iq_a) noexcept {
    command_.feedforward_current_a = iq_a;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::set_position(float position_rad) noexcept {
    set_command(Command{position_rad, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F});
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::set_command(const Command& command) noexcept {
    mode_ = ReferenceMode::Position;
    command_ = command;
    position_ref_rad_ = command.position_rad;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::begin_stall_start(const core::StallStartConfig& recipe) noexcept -> ErrorCode {
    // Admission, not validation of the numbers alone: the sequence commands a
    // torque, so it can only run on an axis that is running and not being
    // aligned. Both refusals leave the phase `Idle` -- a refusal that leaves the
    // old state standing is the failure mode the aligner had (F-35).
    // 准入，而不只是数字校验：序列下发的是转矩，所以只能在一个正在运行、且不在对齐的轴上跑。两种拒绝都把阶段留在
    // `Idle`——拒绝却把旧状态留着，正是对齐器修掉的那个失效形态（F-35）。
    if (sm_.state() != core::MotorState::Run) {
        stall_phase_ = core::StallStartPhase::Idle;
        return ErrorCode::State;
    }
    stall_start_.set_config(recipe);
    if (!stall_start_.begin()) {
        stall_phase_ = core::StallStartPhase::Idle;
        return ErrorCode::Param;
    }
    stall_phase_ = core::StallStartPhase::Ramping;
    stall_torque_nm_ = 0.0F;
    stall_limited_ = false;
    mode_ = ReferenceMode::Torque;
    iq_ref_a_ = 0.0F;
    return ErrorCode::Ok;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::stall_start_phase() const noexcept -> core::StallStartPhase {
    return stall_phase_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::stall_start_torque_nm() const noexcept -> float {
    return stall_torque_nm_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::stall_start_limited() const noexcept -> bool {
    return stall_limited_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::cancel_stall_start() noexcept {
    stall_start_.reset();
    stall_phase_ = core::StallStartPhase::Idle;
    stall_torque_nm_ = 0.0F;
    stall_limited_ = false;
    iq_ref_a_ = 0.0F;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::tick(float dt_s) noexcept -> ErrorCode {
    // Optional overrun measurement: only when the platform gave us a clock. It never feeds back into
    // the control law -- it exists so an ISR overrun becomes visible instead of silent.
    // 可选过载测量：仅当平台提供了时钟时启用。它从不参与控制律——存在的意义是让 ISR 过载可见而非
    // 静默。
    std::uint64_t now_us = 0U;
    if constexpr (core::concepts::has_clock_v<HalPolicy>) {
        now_us = hal_->clock.now_us();
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

    // The power stage's own fault output, next to the inverter's flag and for the same reason. It carries a cause the
    // inverter flag does not: `DriveFault` says the *driver* tripped, which is hardware telling on itself rather than
    // the protector inferring from currents. The member is guaranteed to exist by the refusal in `start()`; the
    // `if constexpr` is what keeps this compilable for a HalPolicy that never got that far.
    // 功率级自己的故障输出，与逆变器标志并列、理由相同。它带着逆变器标志没有的成因：`DriveFault` 说的是**驱动器**跳了，
    // 即硬件自己开口，而不是保护器从电流里推出来。这个成员由 `start()` 里的拒绝保证存在；`if constexpr` 只是让一个还没
    // 走到那一步的 HalPolicy 仍然能编译。
    if constexpr (core::concepts::has_fault_input_v<HalPolicy>) {
        if (hal_->fault_input.faulted()) {
            fault(core::FaultCode::DriveFault);
            status_ = ErrorCode::Fault;
            return status_;
        }
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
            // Hand-over. The forced vector's own rate is the speed the rotor has to be at if it was caught, and it is
            // the only speed available here: the observer is not stepped while the angle is forced, so waiting for its
            // estimate is the only way to learn whether the rotor actually followed.
            // 交接。强制矢量自身的角速度就是"若转子被抓住则它应有的转速"，而这也是此处唯一可得的转速：强制角度期间观测器
            // 不被推进，因此要等它的估计才能知道转子是否真的跟上了。
            handover_omega_e_rad_s_ = align_out.handover_speed_e_rad_s;
            handover_check_s_ = kHandoverCheckS;
            apply_alignment_offset();
            alignment_finished = true;
        }
    } else {
        // The observer runs when the loop needs its angle *or* when it is injecting a carrier. The second half is not an
        // optimisation: an observer that is not stepped does not advance its carrier phase, so an injecting mode selected
        // on a sensored axis used to apply no carrier at all -- admitted, reported, and inert. Stepping it costs one
        // observer update and keeps the sensor as the commutation angle, which is exactly a hybrid drive: the sensor
        // owns the angle, the injection owns nothing but its own demodulation.
        // 观测器在两种情形下运行：环路需要它的角度，或它在注入载波。后半句不是为了优化：没被推进的观测器不推进载波相位，
        // 于是带传感器的轴上选中注入模式过去完全不施加载波——被准入、被上报、却毫无作用。推进它只多一次观测器更新，而
        // 换相角仍由传感器提供，这正是混合驱动的样子：传感器管角度，注入只管它自己的解调。
        if (speed_from_observer || injecting_now()) {
            const core::ObsState observed = policies_.observer.update(v_ab_prev_, i_ab, dt_s);
            // The published speed follows the same rule as the angle. Stepping the observer only to carry a carrier must
            // not change it: `speed_from_observer` is false on a sensored axis, so the sensor's speed is published --
            // except when the sensor reports no velocity at all, which is the case this line has always covered.
            // 发布转速与角度遵循同一条规则。仅为携带载波而推进观测器不得改变它：带传感器的轴上 `speed_from_observer` 为假，
            // 因此发布传感器转速——除非传感器根本不报速度，那正是这一行一直覆盖的情形。
            speed_mech = speed_from_observer ? observed.omega_e / static_cast<float>(config_.pole_pairs) : sensor_speed;
            theta_e = has_sensor ? (angle_mech * static_cast<float>(config_.pole_pairs)) : observed.theta_e;
            // The reference is fed from the observer's own angle, and the index pulse -- when the sensor offers one --
            // is what fixes its origin. Without a mark the reference still counts turns, it simply has no absolute
            // zero, which is exactly what `homed()` reports and why a position command must ask rather than assume.
            // 参考角由观测器自己的角度推进，而索引脉冲（传感器提供时）负责把它的原点钉住。没有标记时参考仍然在数圈，
            // 只是没有绝对零点——这正是 `homed()` 报告的，也是位置指令必须去问而不是假设的原因。
            if constexpr (core::concepts::has_index_v<decltype(hal_->position)>) {
                const bool pulse = hal_->position.has_index() && hal_->position.index_pulse();
                static_cast<void>(position_reference_.update(observed.theta_e, pulse));
            } else {
                static_cast<void>(position_reference_.update(observed.theta_e, false));
            }
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
                // A position command needs an *absolute* angle, which a sensor provides and a turn counter
                // provides once its origin is fixed. Requiring `has_sensor` was a consequence of `angle_mech` being a
                // constant zero without one, not a rule about what position control needs; with a reference the
                // requirement becomes what it always meant. `lost_count()` refuses too: a count that was lost is not
                // a count, and a loop closing on it would drive to a position one turn out.
                // 位置指令需要**绝对**角度：传感器给得出，圈计数在原点被钉住之后也给得出。原先要求 `has_sensor` 是
                // "无传感器时 `angle_mech` 恒为 0"的后果，而不是关于位置控制需要什么的规矩；有了参考，这条要求才回到
                // 它本来的含义。`lost_count()` 同样拒绝：丢掉的计数不算计数，闭环在它上面会把轴开到一个差一整圈的位置。
                const bool position_angle_available =
                    has_sensor || (position_reference_.homed() && !position_reference_.lost_count());
                if (!position_angle_available) {
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
                    const float position_angle_mech =
                        has_sensor ? angle_mech : position_reference_.referenced_mech_rad();
                    speed_ref_from_position_rad_s_ =
                        policies_.position.step(request, position_angle_mech, position_elapsed_s_);
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
                const float feedforward = command_.feedforward_current_a;
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

        // A breakaway sequence owns the torque command while it ramps, and it
        // *replaces* whatever the torque mode was holding: the point of the
        // sequence is that the commanded torque is known at every instant, which a
        // value the caller set by hand is not.
        // 起步序列在爬升期间拥有转矩指令，并**替换**转矩模式原本保持的值：序列的意义在于每一刻的指令力矩都是
        // 已知的，而调用方手设的值不是。
        if (stall_phase_ == core::StallStartPhase::Ramping) {
            const core::StallStartOutput out = stall_start_.step(dt_s, speed_mech);
            stall_phase_ = out.phase;
            stall_torque_nm_ = out.torque_nm;
            stall_limited_ = out.limited;
            if (out.phase == core::StallStartPhase::Failed) {
                iq_ref_a_ = 0.0F;
                fault(core::FaultCode::Stall);
                status_ = ErrorCode::Fault;
                return status_;
            }
            // Torque to current through the nameplate constant, clamped like any
            // other torque command. A machine with no `kt` cannot be given a torque,
            // and inventing one would be a number nobody measured.
            // 由铭牌常数把力矩换成电流，并像其他转矩指令一样限幅。没有 `kt`
            // 的机器给不出力矩，编一个就是没人测过的数。
            iq_ref_a_ =
                (config_.kt > 0.0F) ? core::clamp(out.torque_nm / config_.kt, -config_.i_max_a, config_.i_max_a) : 0.0F;
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
        // Gated on the observer being *stepped*, not on the motor merely lacking a sensor. An observer that is not updated
        // does not advance its carrier phase and a frozen phase sits at a zero crossing, so `injection_ab()` returns
        // exactly zero -- silently disarming a mode the caller asked for. Measured: with the step gate reverted and this
        // gate left open, the swing on a sensored axis is 0.0000. `speed_from_observer` alone is the wrong test: the
        // observer is now stepped whenever it injects (`injecting_now()`), including on a sensored axis where the sensor
        // keeps the commutation angle, so the carrier must be added on exactly the same condition.
        // 门控在"观测器是否在被推进"上，而不只是"电机没有传感器"。没有被推进的观测器不会推进载波相位，因此它的
        // `injection_ab()` 要么是零——静默解除了调用方要求的模式——要么从某个非零初相给出一个常矢量：电流上的直流偏置，
        // 与上面注释里对齐期间的那个隐患完全相同。单看 `speed_from_observer` 是错的判据：观测器只要在注入就会被推进
        // （`injecting_now()`），包括传感器保留换相角的带传感器轴，因此载波必须加在完全相同的条件上。实测：把推进
        // 门控回退、这个门控保持敞开，带传感器轴上的摆幅为 0.0000。
        if ((speed_from_observer || injecting_now()) && !aligning) {
            const Ab<float> injected = policies_.observer.injection_ab();
            v_ab = Ab<float>{static_cast<float>(v_ab.alpha + injected.alpha),
                             static_cast<float>(v_ab.beta + injected.beta)};
        }
    }
    // The window after a forced-angle hand-over, checked on the speed the loop *publishes* -- from the sensor or from
    // the observer, whichever owns the angle -- because the open-loop start cannot see the rotor at all. A ramp that
    // outran the rotor still reaches its hand-over speed: the routine only knows the angle it commands. Half the
    // commanded rate is the threshold, and it is stated rather than tuned: a caught rotor sits at the commanded rate,
    // one that was never caught sits at a few percent of it.
    // 强制角交接之后的窗口，检查的是环路**发布**的转速——不论角度归传感器还是归观测器——因为开环起动完全看不见转子。
    // 跑得比转子快的斜坡照样会到达交接速度：例程只知道它下发的角度。阀值取命令速率的一半，并且是写下来的而不是调出来的：
    // 被抓住的转子就坐在命令速率上，从未被抓住的只有它的几个百分点。
    if (handover_check_s_ > 0.0F) {
        handover_check_s_ -= dt_s;
        if (handover_check_s_ <= 0.0F) {
            handover_check_s_ = 0.0F;
            const float electrical_speed = std::abs(speed_mech) * static_cast<float>(config_.pole_pairs);
            if ((handover_omega_e_rad_s_ > kLostSyncMinHandoverRadS) &&
                (electrical_speed < (kLostSyncSpeedFraction * handover_omega_e_rad_s_))) {
                fault(core::FaultCode::LostSync);
                status_ = ErrorCode::Fault;
                return status_;
            }
        }
    }
    // Stall: the loop is asking for everything it has and the machine is still not moving the way it was told to. This
    // is the second fault code that had a definition and no producer. The condition is deliberately narrow -- full
    // current *and* the wrong speed *and* sustained -- because the alternative is a drive that trips on every large
    // speed step, and a protection nobody trusts gets disabled.
    // 堵转：环路已经在要求它能给出的全部，而机器仍没有按指令方向运动。这是第二个"有定义、没有生产者"的故障码。
    // 条件刻意写得窄——满电流**且**转速不对**且**持续——因为另一种写法会让每一次大的速度阶跃都跳闸，而没人信任的保护
    // 最终会被关掉。
    if (mode_ != ReferenceMode::Torque) {
        const float command = std::abs(speed_ref_rad_s_);
        const bool out_of_authority = std::abs(iq_ref_a_) >= (kStallCurrentFraction * config_.i_max_a);
        const bool not_moving = (command > 0.0F) && ((speed_mech * speed_ref_rad_s_) < 0.0F ||
                                                     (std::abs(speed_mech) < (kStallSpeedFraction * command)));
        if (out_of_authority && not_moving) {
            stall_elapsed_s_ += dt_s;
            if (stall_elapsed_s_ >= kStallWindowS) {
                fault(core::FaultCode::Stall);
                status_ = ErrorCode::Fault;
                return status_;
            }
        } else {
            stall_elapsed_s_ = 0.0F;
        }
    } else {
        stall_elapsed_s_ = 0.0F;
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

    // The trace records what the loop decided, after it decided it. Placed here rather than earlier so one sample is one
    // period: the currents the PI acted on, the voltage it asked for, and the state the period ended in. With the default
    // `NullTrace` this whole block is an inline no-op, which is the only way "off costs nothing" survives a code review:
    // there is no branch to predict, no pointer to check and no counter to increment.
    // 轨迹记录的是控制环的决策，且记录在决策之后。放在这里而不是更早，是为了让一条样本恰好对应一个周期：PI 作用其上的
    // 电流、它请求的电压、以及该周期结束时的状态。默认的 `NullTrace` 下这一整块是内联空操作，这是"关掉不花代价"能过
    // 代码评审的唯一方式：没有分支要预测、没有指针要检查、没有计数器要自增。
    if constexpr (core::concepts::is_trace_sink_v<Trace>) {
        core::TraceSample sample{};
        sample.t_us = static_cast<std::uint32_t>(now_us);
        sample.i_d = i_dq.d;
        sample.i_q = i_dq.q;
        sample.i_ref_d = i_ref.d;
        sample.i_ref_q = i_ref.q;
        sample.v_d = v_dq.d;
        sample.v_q = v_dq.q;
        sample.speed_mech = speed_mech;
        sample.mode = static_cast<std::uint8_t>(sm_.state());
        sample.status = static_cast<std::uint8_t>(status_);
        static_cast<void>(policies_.trace.push(sample));
    }

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
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::state() const noexcept -> core::MotorState {
    return sm_.state();
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::mode() const noexcept -> ReferenceMode {
    return mode_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::telemetry() const noexcept -> Telemetry {
    return telemetry_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::status() const noexcept -> ErrorCode {
    return status_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::observer() noexcept -> Observer& {
    return policies_.observer;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
void Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::apply_params(const core::MachineParams& params) noexcept {
    // Every slot that can take the numbers gets them; every slot that cannot is
    // left as it was built. Three explicit `if constexpr`s rather than a fold
    // over the policy tuple, because the slots are named members and a generic
    // walk would have to invent an ordering for a set of calls that has none.
    // 能接受这些数字的槽位都拿到它们；不能的保持构造时的样子。三个显式 `if
    // constexpr` 而不是对策略元组做折叠，因为
    // 槽位是有名字的成员，而通用遍历必须给一组本来没有次序的调用发明一个次序。
    if constexpr (core::concepts::has_apply_params_v<Observer>) {
        policies_.observer.apply_params(params);
    }
    if constexpr (core::concepts::has_apply_params_v<Controller>) {
        policies_.controller.apply_params(params);
    }
    if constexpr (core::concepts::has_apply_params_v<FeedForward>) {
        policies_.feed_forward.apply_params(params);
    }
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::reference() noexcept -> RefGenerator& {
    return policies_.reference;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::feed_forward() noexcept -> FeedForward& {
    return policies_.feed_forward;
}


template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::position_reference() const noexcept -> const core::PositionReference& {
    return position_reference_;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::aligner() noexcept -> Aligner& {
    return policies_.align;
}



template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::aligner() const noexcept -> const Aligner& {
    return policies_.align;
}



template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::position() noexcept -> PositionControl& {
    return policies_.position;
}

template <typename Observer, typename Modulator, typename Controller, typename HalPolicy, typename RefGenerator,
          typename FeedForward, typename PositionControl, typename Aligner, typename Trace>
auto Motor<Observer, Modulator, Controller, HalPolicy, RefGenerator, FeedForward, PositionControl, Aligner,
           Trace>::config() const noexcept -> const MotorConfig& {
    return config_;
}

} // namespace motor

} // namespace foc
