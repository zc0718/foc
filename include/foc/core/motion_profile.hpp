// Conan::ImportStart
#pragma once
#include <foc/core/position.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file motion_profile.hpp
 *
 * @brief [en] The profile that shapes a velocity command into something the mechanics can follow.
 * @brief [zh] 把速度指令塑造成机械跟得上的形状的运动规划。
 * @since 0.1.0
 */

namespace foc {

namespace core {


/**
 * @brief [en] A position controller that shapes its velocity command: proportional on the measured error, then
 *             rate-limited by a configured acceleration.
 *
 *   It fills the same controller slot as `Pi` (`step(setpoint, feedback, dt)` + `reset()`), but it answers a different
 *   question: not "how fast should I go to kill this error" alone, but "how fast may I go, and how quickly may that
 *   change". The output is a velocity command, so the inner speed loop still owns velocity tracking.
 *
 *   Why a filtered command rather than a trapezoid planner: a planner has to own a phase machine (accelerate / cruise /
 *   decelerate) and re-plan every time the target moves. This keeps no phases: it takes the smallest of the
 *   proportional demand, the physical braking curve `sqrt(2*a*|error|)` and the velocity limit, then rate-limits the
 *   result. That gives accel-limited motion with a stopping distance that matches the deceleration it is allowed,
 *   retargeting for free, and no overshoot when a limit binds. It is not time-optimal against a moving target, and it
 *   does not pretend to be.
 *
 *   That stopping distance is a property of the *command*, not of the axis. The axis reaches the deceleration threshold
 *   after covering `v^2/(2a)` plus whatever it travels while the inner loop catches up, so a plant that lags the
 *   command passes the threshold and overshoots by an amount this class cannot bound -- measured at 13.7 % of the
 *   threshold on the SIL axis. The command itself cannot overshoot: both ceilings fall monotonically in `|error|` and
 *   the sign follows the error. 那个制动距离是 **指令** 的属性，不是轴的属性。轴走完 `v^2/(2a)` 之后还要加上内层环追赶
 *   期间走过的距离，因此跟随滞后的 plant 会越过阈值，并超出这个类无法约束的量——在 SIL 轴上测得阈值的 13.7 %。指令本身不
 *   会过冲：两个上限都随 `|error|` 单调下降，符号跟着误差走。
 *
 * @brief [zh] 会塑造速度指令的位置控制器：对实测误差做比例，再用配置的加速度做速率限制。
 *
 *   它占用与 `Pi` 相同的控制器槽位（ `step(setpoint, feedback, dt)` + `reset()`），但回答的 是另一个问题：不只是“为了消
 *   掉这个误差我该跑多快”，而是“我准许多快、以及这个快能变多快”。 输出是速度指令，所以速度跟踪仍由内层速度环负责。
 *
 *   为什么用滤波而不是梯形规划器：规划器必须自持相位机（加速/匀速/减速），且目标一动就要 重规划。本实现不存相位：取“ 比
 *   例需求、物理制动曲线 `sqrt(2*a*|error|)`、速度上限”三者最小， 再对结果做速率限制。这样既得到加速度受限的运动，制动
 *   距离又与允许的减速度相匹配；中途改 目标天然支持，限幅生效时也不会冲过头。它对移动目标不是时间最优的，也没有假装是。
 *
 * @context [en] Called once per position-loop step: noexcept, no allocation.
 * @context [zh] 每个位置环周期调用一次：noexcept、不分 配。
 * @since 0.1.0
 * @exporter
 */
class MotionProfile final {
  public:
    /**
     * @brief [en] The three numbers a joint has to declare. None of them is derivable from the nameplate: they are
     *             application limits, so there is no defaulting helper on purpose.
     * @brief [zh] 关节必须自己声明的三个数。它们都无法从铭牌推导——是应用侧的限制，因此故意不提供 “自动取默认”的辅助函
     *             数。
     * @note [en] A zeroed configuration is inert (`step()` always answers 0), which is the safe answer for "nobody
     *            configured me".
     * @note [zh] 全零配置是惰性的（ `step()` 恒返回 0），这正是“没人配置过我”时应有的安全答案。
     * @since 0.1.0
     * @exporter
     */
    struct Config final {
        /**
         * @brief [en] velocity per rad of error, 1/s.
         * @brief [zh] 每弧度误差对应的速度，1/s。
         */
        float shaping_gain_rad_s;
        /**
         * @brief [en] ceiling for the velocity command, rad/s.
         * @brief [zh] 速度指令上限，rad/s。
         */
        float velocity_limit_rad_s;
        /**
         * @brief [en] how fast the command may change, rad/s^2.
         * @brief [zh] 指令允许的变化速率，rad/s^2。
         */
        float accel_limit_rad_s2;
    };

    MotionProfile() noexcept = default;

    explicit MotionProfile(const Config& config) noexcept;

    /**
     * @brief [en] Velocity command for this step, in rad/s.
     * @brief [zh] 本步的速度指令，rad/s。
     * @param setpoint [en] target mechanical angle in rad.
     * @param setpoint [zh] 目标机械角，rad。
     * @param feedback [en] measured mechanical angle in rad.
     * @param feedback [zh] 实测机械角，rad。
     * @param dt_s [en] step length in s; <= 0 is rejected.
     * @param dt_s [zh] 步长，s；<= 0 被拒绝。
     * @return [en] the rate-limited velocity command; a non-finite input holds the previous command instead of
     *              propagating the garbage.
     * @return [zh] 限速率后的速度指令；输入非有限时保持上一条指令，而不是把垃圾值传下去。
     * @since 0.1.0
     */
    [[nodiscard]] auto step(float setpoint, float feedback, float dt_s) noexcept -> float;

    /**
     * @brief [en] Velocity command for this step, honouring per-move limits when they are given.
     * @brief [zh] 本步的速度指令；给定了本次限制时按本次限制执行。
     * @param request [en] target plus optional per-move ceilings.
     * @param request [zh] 目标以及可选的本次上限。
     * @param feedback [en] measured mechanical angle in rad.
     * @param feedback [zh] 实测机械角，rad。
     * @param dt_s [en] step length in s; <= 0 is rejected.
     * @param dt_s [zh] 步长，s；<= 0 被拒绝。
     * @note [en] A non-zero per-move limit *replaces* the configured one for the duration of the call; the
     *            configuration is untouched, so the next move can ask for something else without any restore step.
     * @note [zh] 非零的本次限制在本次调用内 **替换** 配置里的那一个；配置本身不被改动，因此下一次运动 可以直接要求别的
     *            值，不需要任何恢复步骤。
     * @since 0.1.0
     */
    [[nodiscard]] auto step(const PositionInput& request, float feedback, float dt_s) noexcept -> float;

    /**
     * @brief [en] Forget the command and start from rest.
     * @brief [zh] 丢弃当前指令，从静止重新开始。
     * @since 0.1.0
     */
    void reset() noexcept;

    /**
     * @brief [en] The configuration in force.
     * @brief [zh] 当前生效的配置。
     * @since 0.1.0
     */
    [[nodiscard]] auto config() const noexcept -> const Config&;

    /**
     * @brief [en] The velocity command the last step produced.
     * @brief [zh] 最近一步产生的速度指令。
     * @since 0.1.0
     */
    [[nodiscard]] auto velocity_command_rad_s() const noexcept -> float;

  private:
    [[nodiscard]] auto shape(float target_rad, float feedback_rad, float dt_s, float velocity_limit_rad_s,
                             float accel_limit_rad_s2, float shaping_gain_rad_s) noexcept -> float;

    Config config_{};
    float velocity_rad_s_{0.0F};
};

} // namespace core

} // namespace foc
