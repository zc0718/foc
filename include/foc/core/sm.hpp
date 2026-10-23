// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Operating mode of a motor instance.
 *        [zh] 电机实例的运行模式。
 *
 * @since 0.1.0
 * @exporter
 */
enum class MotorState : std::uint8_t {
    Idle = 0,  ///< [en] outputs off. [zh] 输出关闭。
    Align = 1, ///< [en] rotor alignment before closed loop. [zh] 闭环前的转子对齐。
    Run = 2,   ///< [en] closed loop. [zh] 闭环运行。
    Fault = 3, ///< [en] latched fault, only clear_fault() leaves it. [zh] 故障锁存，只能由 clear_fault() 退出。
};


/**
 * @brief [en] Mode state machine: Idle -> Align -> Run, any state -> Fault, Fault -> Idle on an
 *        explicit clear. Illegal transitions are counted and ignored -- never a crash.
 *        [zh] 模式状态机：Idle -> Align -> Run，任意状态 -> Fault，Fault 显式清除后回到 Idle。
 *        非法迁移只计数并忽略——绝不崩溃。
 *
 * @context [en] Called from the control loop or the command layer: noexcept, no allocation.
 *           [zh] 由控制环或命令层调用：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class StateMachine final {
  public:
    StateMachine() noexcept = default;

    /**
     * @brief [en] Current state.
     *        [zh] 当前状态。
     * @since 0.1.0
     */
    [[nodiscard]] auto state() const noexcept -> MotorState;

    /**
     * @brief [en] Number of illegal transition attempts so far.
     *        [zh] 目前为止的非法迁移尝试次数。
     * @since 0.1.0
     */
    [[nodiscard]] auto illegal_transitions() const noexcept -> std::uint32_t;

    /**
     * @brief [en] Request a transition to `target`.
     *        [zh] 请求迁移到 `target`。
     *
     * @param target [en] desired state. [zh] 目标状态。
     * @return [en] Ok when the transition was applied, State when it is illegal (state unchanged,
     *         counter incremented).
     *         [zh] 迁移生效返回 Ok；非法时返回 State（状态不变、计数器自增）。
     * @note [en] Leaving Fault requires clear_fault(), a plain request to Idle is illegal.
     *       [zh] 只有 clear_fault() 能离开 Fault，直接请求 Idle 属于非法迁移。
     * @since 0.1.0
     */
    auto request(MotorState target) noexcept -> ErrorCode;

    /**
     * @brief [en] Clear a latched fault and return to Idle.
     *        [zh] 清除锁存故障并回到 Idle。
     *
     * @return [en] Ok when the machine was in Fault, State otherwise (counted as illegal).
     *         [zh] 处于 Fault 时返回 Ok，否则返回 State（计入非法迁移）。
     * @since 0.1.0
     */
    auto clear_fault() noexcept -> ErrorCode;

  private:
    MotorState state_{MotorState::Idle};
    std::uint32_t illegal_{0};
};

} // namespace core

} // namespace foc
