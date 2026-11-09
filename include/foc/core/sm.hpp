// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file sm.hpp
 *
 * @brief [en] The motor state machine: the states an instance may be in, and the transitions between them.
 * @brief [zh] 电机状态机：实例可能处于的状态及其之间的迁移。
 * @since 0.1.0
 */

namespace foc {

namespace core {


/**
 * @brief [en] Operating mode of a motor instance.
 * @brief [zh] 电机实例的运行模式。
 * @since 0.1.0
 * @exporter
 */
enum class MotorState : std::uint8_t {
    /**
     * @brief [en] outputs off.
     * @brief [zh] 输出关闭。
     */
    Idle = 0,
    /**
     * @brief [en] rotor alignment before closed loop.
     * @brief [zh] 闭环前的转子对齐。
     */
    Align = 1,
    /**
     * @brief [en] closed loop.
     * @brief [zh] 闭环运行。
     */
    Run = 2,
    /**
     * @brief [en] latched fault, only clear_fault() leaves it.
     * @brief [zh] 故障锁存，只能由 clear_fault() 退出。
     */
    Fault = 3,
};


/**
 * @brief [en] Mode state machine: `Idle -> Align -> Run` forwards, **back to `Idle` from `Align` or `Run`** (the
 *             abort), any live state -> `Fault`, and `Fault` -> `Idle` only through `clear_fault()`. That is seven
 *             legal edges and nothing else; every refusal is counted and ignored -- never a crash.
 * @brief [zh] 模式状态机： `Idle -> Align -> Run` 前进， **从 `Align` 或 `Run` 可以退回 `Idle`** （中止），任意活状态
 *             -> `Fault`，而 `Fault` -> `Idle` 只能经 `clear_fault()`。合法边只有这七条，其余一律拒绝；拒绝只计数并忽略
 *             —— 绝不崩溃。
 * @note [en] The seven edges are counted rather than described, because the description was wrong until it was counted:
 *            the line above used to say `Idle -> Align -> Run` and stop, which reads as "a running motor cannot be
 *            commanded back to `Idle`" -- so a caller obeying the documentation would have had to fault the drive to
 *            stop it. Measured, both abort edges are legal; `test_sm.cpp` now enumerates the whole relation and
 *            computes its graph properties (every state reachable from `Idle`, `Fault` the only sink) instead of
 *            trusting the sentence. A request for the state the machine is already in is **refused**, not ignored: a
 *            supervisor that re-asserts its target every cycle fills `illegal_transitions()` with attempts that mean
 *            nothing.
 * @note [zh] 这七条边是 **数出来** 的而不是描述出来的，因为在数之前那句描述是错的：上面那行原写作
 *            `Idle -> Align -> Run` 就结束了，读起来是"正在运行的电机无法被命令回 `Idle`"——照文档办事的调用方只能先让它
 *            故障才能停机。实测两条中止边都是合法的； `test_sm.cpp` 现在枚举整个关系并计算它的图性质（每个状态都可自
 *            `Idle` 到达、 `Fault` 是唯一汇），而不是相信那句话。请求机器 **已经处于** 的状态会被 **拒绝** ，而不是被忽
 *            略：每周期重申一次目标的管理层会把 `illegal_transitions()` 填满毫无意义的尝试。
 * @context [en] Called from the control loop or the command layer: noexcept, no allocation.
 * @context [zh] 由控制环或命令层调用：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class StateMachine final {
  public:
    StateMachine() noexcept = default;

    /**
     * @brief [en] Current state.
     * @brief [zh] 当前状态。
     * @since 0.1.0
     */
    [[nodiscard]] auto state() const noexcept -> MotorState;

    /**
     * @brief [en] Number of illegal transition attempts so far.
     * @brief [zh] 目前为止的非法迁移尝试次数。
     * @since 0.1.0
     */
    [[nodiscard]] auto illegal_transitions() const noexcept -> std::uint32_t;

    /**
     * @brief [en] Request a transition to `target`.
     * @brief [zh] 请求迁移到 `target`。
     * @param target [en] desired state.
     * @param target [zh] 目标状态。
     * @return [en] Ok when the transition was applied, State when it is illegal (state unchanged, counter incremented).
     * @return [zh] 迁移生效返回 Ok；非法时返回 State（状态不变、计数器自增）。
     * @note [en] Leaving Fault requires clear_fault(), a plain request to Idle is illegal.
     * @note [zh] 只有 clear_fault() 能离开 Fault，直接请求 Idle 属于非法迁移。
     * @since 0.1.0
     */
    auto request(MotorState target) noexcept -> ErrorCode;

    /**
     * @brief [en] Clear a latched fault and return to Idle.
     * @brief [zh] 清除锁存故障并回到 Idle。
     * @return [en] Ok when the machine was in Fault, State otherwise (counted as illegal).
     * @return [zh] 处于 Fault 时返回 Ok，否则返回 State（计入非法迁移）。
     * @since 0.1.0
     */
    auto clear_fault() noexcept -> ErrorCode;

  private:
    MotorState state_{MotorState::Idle};
    std::uint32_t illegal_{0};
};

} // namespace core

} // namespace foc
