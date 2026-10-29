// Conan::ImportStart
#pragma once

#include <cmath>
#include <cstdint>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] The window an axis is allowed to occupy, in mechanical radians.
 * @brief [zh] 轴允许占据的窗口，机械弧度。
 * @note [en] Two numbers and nothing else: a soft limit that also carries a slowdown zone, a stiffness or a ramp is a
 *            motion controller, and the axis already has one. What this cannot do -- keep an axis out of a wall it was
 *            already past -- needs a homing switch, not a wider window.
 * @note [zh] 只有两个数：一个还带减速区、刚度或斜坡的软限位就是一台运动控制器，而这条轴已经有了一个。它做不到的事—— 把
 *            一条**已经**越过去的轴挡在墙外——需要回零开关，而不是更宽的窗口。
 * @since 0.1.10
 */
struct SoftLimitConfig final {
    float min_rad; ///< [en] lower wall, rad, inclusive. [zh] 下墙，rad，含端点。
    float max_rad; ///< [en] upper wall, rad, inclusive. [zh] 上墙，rad，含端点。
};


/**
 * @brief [en] What the limit did to a target this period.
 * @brief [zh] 本周期限位对目标做了什么。
 * @since 0.1.10
 * @exporter
 */
struct SoftLimitOutput final {
    /**
     * @brief [en] `Ok` when the target was inside the window and is passed through unchanged; `Param` when it was
     *             outside and has been *refused* -- the previous target comes back instead, so nothing downstream ever
     *             sees a position it would drive into a wall.
     * @brief [zh] 目标在窗口内并原样通过时为 `Ok`；目标在窗口外并被**拒绝**时为 `Param`——回来的 是上一个目标，因此下游
     *             任何东西都不会看到一个会把它驱进墙里的位置。
     */
    ErrorCode status;
    float target_rad; ///< [en] the target to use this period. [zh] 本周期要用的目标。
    bool at_wall;     ///< [en] the accepted target is exactly on a wall. [zh] 被接受的目标正好在墙上。
};


/**
 * @brief [en] Keep an axis inside a window: refuse the targets outside it, and cap the speed near the walls.
 *
 * @brief [zh] 把轴留在窗口内：拒绝窗口外的目标，并在靠近墙时限制速度。
 *
 *     The two halves answer two different failures, and mixing them up is how soft limits get a bad name. Refusing a
 *     target outside the window fixes a *programming* fault -- a move that was never meant to be commanded -- and it is
 *     answered by status, not by clamping, because a clamp turns an impossible command into a plausible one and the
 *     caller never learns. Capping the speed near the wall fixes a *physical* fault -- the axis has momentum and needs
 *     distance to lose it -- and it is answered by an invariant that holds every period: `v^2 <= 2*a*(d - v*dt)`, with
 *     `d` the distance left to the wall, `a` the deceleration the axis really has, and `dt` the drive's own delay.
 *
 *     Why the invariant rather than a slowdown zone: a zone has a width, the width has to be right, and "right" depends
 *     on the speed. The invariant has no width -- at every speed and every position it says the same thing -- and it is
 *     checkable in the same period it is applied, so a case can assert it sample by sample instead of at the end.
 *
 *     The `v*dt` term is not a refinement, it is the difference between working and not working. On the SIL plant at 20
 *     rad/s toward a wall: without the term the axis crosses by **19 mm**, with it it stops **1.04 mm short**, and 19
 *     mm is exactly `v*dt` for the 1 ms this drive takes to deliver the current it was asked for. A ceiling that
 *     assumes instantaneous braking assumes infinite jerk, and no drive has it.
 *
 *     两半回答两种不同的失效，把它们混在一起正是软限位名声不好的原因。拒绝窗口外的目标修的是**编程**错误——一次本就不该
 *     下发的运动——而它用状态回答而不是钳位，因为钳位会把一条不可能的命令变成一条看起来合理的命令，调用方永远学不到。靠
 *     近墙时限制速度修的是**物理**错误——轴有动量，需要距离把它耗掉——而它用一个每周期都成立的守恒式回答： `v² <= 2·a·(d
 *     − v·dt)`，`d` 是到墙的剩余距离，`a` 是轴真实具备的减速度，`dt` 是驱动自身的延迟。
 *
 *     为什么用守恒式而不是减速区：区有宽度，宽度得对，而"对"取决于速度。守恒式没有宽度——在任何速度和任何位置它说的都是
 *     同一件事——而且它就在施加它的那个周期里可检，用例因此可以逐点断言，而不是等到最后。
 *
 *     `v·dt` 这一项不是精修，而是能不能用的分界。在 SIL plant 上以 20 rad/s 冲向一面墙：没有这一项时轴越过 **19 mm**，
 *     有它时停在墙前 **1.04 mm**，而 19 mm 恰好是这台驱动把电流送到位所需的 1 ms 里的 `v·dt`。假设瞬时制动的上限等于假
 *     设无限加加速度，而没有哪台驱动有这个。
 * @since 0.1.10
 * @exporter
 */
class SoftLimit final {
  public:
    SoftLimit() noexcept = default;
    explicit SoftLimit(const SoftLimitConfig& config) noexcept : config_(config) {}

    /**
     * @brief [en] Replace the window. The remembered target is clamped into the new window on the next `apply()`.
     * @brief [zh] 换窗口。记住的目标会在下一次 `apply()` 时被钳进新窗口。
     */
    void set_config(const SoftLimitConfig& config) noexcept { config_ = config; }

    [[nodiscard]] auto config() const noexcept -> const SoftLimitConfig& { return config_; }

    /**
     * @brief [en] `Ok` when the window describes a real one; `Param` when it is inverted or not finite.
     * @brief [zh] 窗口描述的是一段真实区间时为 `Ok`；反了或非有限时为 `Param`。
     */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode {
        if (!std::isfinite(config_.min_rad) || !std::isfinite(config_.max_rad)) {
            return ErrorCode::Param;
        }
        return (config_.max_rad >= config_.min_rad) ? ErrorCode::Ok : ErrorCode::Param;
    }

    /**
     * @brief [en] Forget the remembered target.
     * @brief [zh] 忘掉记住的目标。
     */
    void reset() noexcept {
        initialised_ = false;
        target_rad_ = 0.0F;
    }

    /**
     * @brief [en] Admit or refuse a target.
     * @brief [zh] 接纳或拒绝一个目标。
     * @param target_rad [en] the target the caller wants, rad. @param target_rad
     * @param target_rad [zh] 调用方想要的目标，rad。
     * @note [en] The walls are **inclusive**: a target exactly on the wall is accepted and reported through `at_wall`.
     *            An exclusive boundary would make the last usable position depend on the last digit of a float, and
     *            "move to the limit" is the one command a soft limit exists to make safe.
     * @note [zh] 墙是**含端点**的：正好在墙上的目标被接受，并通过 `at_wall` 报告。用排除边界会让最后一个可用位置取决于
     *            一个 float 的最后一位，而"走到限位"正是软限位存在的意义所在的那条命令。
     * @since 0.1.10
     */
    [[nodiscard]] auto apply(float target_rad) noexcept -> SoftLimitOutput {
        SoftLimitOutput out{validate(), target_rad_, false};
        if (out.status != ErrorCode::Ok) {
            return out;
        }
        if (!initialised_) {
            // The first target is admitted only if it is already inside; otherwise there is no legal position to hold
            // and answering with one would be inventing it.
            // 第一个目标只有在它本来就在窗口内时才被接纳；否则没有合法的位置可保持，而给出一个就是在编造它。
            if (target_rad < config_.min_rad || target_rad > config_.max_rad) {
                out.status = ErrorCode::Param;
                return out;
            }
            initialised_ = true;
            target_rad_ = target_rad;
        } else if (target_rad < config_.min_rad || target_rad > config_.max_rad) {
            out.status = ErrorCode::Param;
            return out;
        } else {
            target_rad_ = target_rad;
        }
        out.target_rad = target_rad_;
        out.at_wall = (target_rad_ <= config_.min_rad) || (target_rad_ >= config_.max_rad);
        return out;
    }

    /**
     * @brief [en] Cap a velocity command so the axis can still stop before the wall it is heading for.
     * @brief [zh] 限住速度指令，使轴仍能在它正驶向的那面墙之前停住。
     * @param velocity_command_rad_s [en] what the loop would like, rad/s. @param velocity_command_rad_s
     * @param velocity_command_rad_s [zh] 环路想要的速度，rad/s。
     * @param position_rad [en] where the axis is, rad. @param position_rad
     * @param position_rad [zh] 轴在哪，rad。
     * @param decel_rad_s2 [en] the deceleration the axis really has, rad/s^2 (> 0). @param decel_rad_s2
     * @param decel_rad_s2 [zh] 轴真实具备的减速度，rad/s²（> 0）。
     * @param delay_s [en] how long the drive takes to deliver a braking command, s (>= 0). @param delay_s
     * @param delay_s [zh] 驱动把制动指令送到位需要多久，s（>= 0）。
     * @return [en] a command whose magnitude never exceeds the ceiling `v^2 = 2*decel*(d - v*delay)` in the direction
     *              of the wall it is approaching, and the input unchanged otherwise.
     * @return [zh] 在它正接近的那面墙方向上，幅值不超过上限 `v² = 2·减速度·(d − v·延迟)` 的指令；其余情况原样返回。
     * @note [en] `decel_rad_s2` is the axis's own stopping capability and `delay_s` its own latency; neither is
     *            derivable here. A number taken from a datasheet or a commissioning run is the only honest source, so
     *            `decel <= 0` is refused (the command is passed through) instead of defaulted, and a negative delay is
     *            treated as zero -- the idealised drive, which is the assumption the `v*delay` term exists to correct.
     * @note [zh] `decel_rad_s2` 是轴自身的停车能力、`delay_s` 是它自身的延迟，两者都无法从这里推导。取自数据表或一次调
     *            试运行的数才是唯一诚实的来源，因此 `减速度 <= 0` 被拒绝（指令原样通过）而不是取默认；负的延迟当作零处
     *            理 ——那是理想化驱动，也正是 `v·延迟` 这一项存在的目的所要纠正的假设。
     * @since 0.1.10
     */
    [[nodiscard]] auto shape(float velocity_command_rad_s, float position_rad, float decel_rad_s2,
                             float delay_s) const noexcept -> float {
        if (!(decel_rad_s2 > 0.0F) || !std::isfinite(velocity_command_rad_s) || !std::isfinite(position_rad)) {
            return velocity_command_rad_s;
        }
        const float delay = (delay_s > 0.0F) ? delay_s : 0.0F;
        const float to_high = config_.max_rad - position_rad;
        const float to_low = position_rad - config_.min_rad;
        // The wall the command is heading for, and the distance left to it. A command toward a wall with no distance
        // left is removed entirely -- magnitude zero, not a reversed command: reversing would be the limit driving.
        // 指令正驶向的那面墙，以及到它的剩余距离。面对一面已经没有距离的墙的指令被整个去掉——幅值为零而不是反号：
        // 反号等于限位在驱动。
        if (velocity_command_rad_s > 0.0F && to_high <= 0.0F) {
            return 0.0F;
        }
        if (velocity_command_rad_s < 0.0F && to_low <= 0.0F) {
            return 0.0F;
        }
        // `d - |v|*delay` is the distance that is still braking distance rather than travel: at `v` the first `delay`
        // seconds are spent still moving at `v` while the current rises to the braking value.
        // `d − |v|·延迟` 才是仍然算作制动距离的那一段：速度为 `v` 时，头 `delay` 秒仍在以 `v` 前进，而电流正在爬到制动值。
        const auto ceiling_for = [&](float distance) -> float {
            const float travel =
                (velocity_command_rad_s < 0.0F ? -velocity_command_rad_s : velocity_command_rad_s) * delay;
            const float effective = distance - travel;
            return (effective > 0.0F) ? std::sqrt(2.0F * decel_rad_s2 * effective) : 0.0F;
        };
        if (velocity_command_rad_s > 0.0F) {
            const float ceiling = ceiling_for(to_high);
            return (velocity_command_rad_s > ceiling) ? ceiling : velocity_command_rad_s;
        }
        if (velocity_command_rad_s < 0.0F) {
            const float ceiling = ceiling_for(to_low);
            return (velocity_command_rad_s < -ceiling) ? -ceiling : velocity_command_rad_s;
        }
        return velocity_command_rad_s;
    }

    /**
     * @brief [en] The target currently in force.
     * @brief [zh] 当前生效的目标。
     * @since 0.1.10
     */
    [[nodiscard]] auto target_rad() const noexcept -> float { return target_rad_; }

  private:
    SoftLimitConfig config_{};
    float target_rad_{0.0F};
    bool initialised_{false};
};


} // namespace core

} // namespace foc
