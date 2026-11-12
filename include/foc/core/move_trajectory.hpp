// Conan::ImportStart
#pragma once
#include <algorithm>
#include <cmath>
#include <foc/core/types.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file move_trajectory.hpp
 *
 * @brief [en] The reference that says where to be on the way, not only where to end up.
 * @brief [zh] 每一拍都说出该在哪儿、而不只是终点在哪儿的参考。
 * @since 0.1.0
 */

namespace foc {

namespace core {


/**
 * @brief [en] The two numbers a move is allowed to use: how fast it may cruise and how quickly that may change.
 * @brief [zh] 一次运动允许用的两个数：能巡航多快，以及这个快能变多快。
 * @note [en] Application limits, not machine data: nothing on a nameplate says how fast a joint may be asked to move.
 *            Both must be positive for the trajectory to move at all.
 * @note [zh] 应用侧的限制，不是机器数据：铭牌上没有任何一项说这个关节可以被要求跑多快。两个都必须为正，轨迹才动。
 * @since 0.1.0
 * @exporter
 */
struct MoveLimits final {
    /**
     * @brief [en] the speed a cruise may hold, rad/s.
     * @brief [zh] 巡航段允许保持的速度，rad/s。
     */
    float velocity_rad_s;
    /**
     * @brief [en] how fast the reference velocity may change, rad/s^2.
     * @brief [zh] 参考速度允许的变化速率，rad/s^2。
     */
    float accel_rad_s2;
};


/**
 * @brief [en] Where the reference is now, how fast it is going there, and whether anything is still pending.
 * @brief [zh] 参考此刻在哪、以多快往那儿去，以及是否还有事没做完。
 * @since 0.1.0
 * @exporter
 */
struct TrajectoryPoint final {
    /**
     * @brief [en] the mechanical angle to hand to the position loop this tick, rad.
     * @brief [zh] 本拍要交给位置环的机械角，rad。
     */
    float position_rad;
    /**
     * @brief [en] how fast that position is travelling, rad/s. Signed.
     * @brief [zh] 这个位置正以多快移动，rad/s。有符号。
     */
    float velocity_rad_s;
    /**
     * @brief [en] nothing is pending: the reference is at its target and stays there until told otherwise.
     * @brief [zh] 没有待办：参考已经在目标上，在被告知别的之前都停在那儿。
     */
    bool done;
};


/**
 * @brief [en] A position reference that ramps, cruises and arrives, not only one that ends up there.
 *
 *   Why this is not `MotionProfile`. `MotionProfile` sits in the position *controller* slot and shapes a velocity
 *   command from the position error; it never emits a position, and it cannot say when it will be finished. A joint
 *   that has to be somewhere at a known time needs both, so this type produces a position per tick and reports when it
 *   has arrived. The measured consequence is the cruise gap -- how far the setpoint the loop was given is from where
 *   the axis is, while the axis travels at the ceiling: `0.49597 rad` with only a shaped velocity command, against
 *   `0.09881 rad` with a streamed reference and a plain proportional loop, on one bench for one move. That second
 *   number is not a tuning result -- a first-order loop following a ramp of `v` lags by `v/gain`, `2/20 = 0.1` here.
 *   And the reference has to be followed by a plain loop: put the shaper in the slot as well and the two spend the
 *   acceleration budget twice, so the axis peaks at `1.8902 rad/s` instead of `2` and never reaches the cruise.
 *
 *   One law, not a phase machine. Each step takes the smaller of the cruise ceiling and the fastest speed from which
 *   the distance can still be stopped on, then limits how fast the velocity may move toward that, at `a`. The three
 *   phases of a trapezoid -- ramp up, cruise, ramp down -- are what that one line does over time, rather than three
 *   states that could disagree with each other about which one is active. The deceleration therefore begins where
 *   stopping needs it to, not at a threshold someone tuned.
 *
 *   The braking speed is evaluated on the distance that will remain **after this step**, `|remaining| - |v|*dt`, and
 *   not on the distance now: the tick about to be taken consumes distance too. Without that, the switch into
 *   deceleration happens one sample late, and one sample of distance is not a small error at the end of a ramp -- it
 *   measured as landing **92 ticks (0.66 %) early** and dropping `0.045 rad/s` in a single step, because the remaining
 *   distance ran out while the velocity was still climbing down. With it, the same move lands within one tick of
 *   `D/v + v/a` and the velocity reaches zero on the tick it lands.
 *
 *   A new target mid-move is not a new plan. `move_to()` writes the target and touches neither the position nor the
 *   velocity, so the reference cannot jump: the value handed out on the tick before the call is the value handed out on
 *   the tick after it, and the motion continues from where it is -- braking first if the new target is behind. Steering
 *   the axis by streaming setpoints is therefore continuous by construction rather than by a clamp.
 *
 *   What it is **not**: it does not know the axis. It produces a reference the loop follows with its own lag, so the
 *   axis arrives later than the reference does, by an amount this type cannot bound. It is not time-optimal against a
 *   moving target, and it does not avoid anything: it goes to the target it was given.
 *
 * @brief [zh] 会加速、巡航、到位的 位置参考：交给环的是「该在哪儿」，而不只是「终点在哪儿」。
 *
 *   为什么它不是 `MotionProfile`。 `MotionProfile` 占的是位置 **控制器** 槽位，它用位置误差塑造速度指令；它从不输出位
 *   置，也说不出自己何时结束。一个必须在已知时刻到达某处的关节两样都要，因此本类型每拍给出一个位置，并报告自己已经到
 *   位。可测量的后果在巡航间隙上——轴以上限速度行进时，环被告知的设定点离它有多远：同一个台架、同一次运动，只有速度指令
 *   时是 `0.49597 rad`，而流式参考配一个朴素比例环时是 `0.09881 rad`。后一个数不是调出来的结果——一阶环跟一条 `v` 的斜
 *   坡，滞后就是 `v/gain`，这里 `2/20 = 0.1`。而且这条参考必须由朴素环来跟：把整形器也放进槽位，两个整形器花掉两遍加速
 *   度预算，轴峰值只到 `1.8902 rad/s` 而不是 `2`，压根进不了巡航段。
 *
 *   一条律，不是一台相位机。每拍取「巡航上限」与「从剩余距离还来得及停下的最快速度」中的较小者，再限制速度朝它变化的快
 *   慢（按 `a`）。梯形的三段——加速、巡航、减速——是这一行随时间的产物，而不是三个可能互相不同步的状态。于是减速恰好在「
 *   停下来需要的那段距离」上开始，而不是在某人调出来的阈值上开始。
 *
 *   制动速度算在 **这一拍走完之后** 剩下的距离上（ `|剩距| - |v|*dt` ），而不是现在的距离：即将走的这一拍本身也消耗距
 *   离。不这么做，切换进减速就晚了一个采样，而一个采样的距离在斜坡末尾一点也不小——实测是 **提前 92 拍（0.66 %）** 落
 *   地，并在一步之内丢掉 `0.045 rad/s`，因为速度还在往下爬的时候距离就没了。这么做了之后，同一次运动落在 `D/v + v/a` 的
 *   一拍之内，而且速度正好在落地那一拍归零。
 *
 *   中途换目标不是重新规划。 `move_to()` 只写目标，既不动位置也不动速度，因此参考不可能跳变：调用前那一拍交出去的值，就
 *   是调用后那一拍交出去的值，运动从它当时所在的地方继续——新目标在身后就先刹住。所以用流式设定点操纵轴，连续性来自构
 *   造，而不是来自某个夹紧。
 *
 *   它 **不是** 什么：它不认识轴。它产生的参考由环带着自己的滞后去跟随，因此轴比参考晚到，晚多少本类型约束不了。它对移
 *   动目标不是时间最优的，也不避让任何东西：给什么目标就去哪儿。
 *
 * @context [en] Position-loop ISR: noexcept, no allocation, one call per tick.
 * @context [zh] 位置环 ISR：noexcept、不分配、每拍一次调用。
 * @since 0.1.0
 * @exporter
 */
class MoveTrajectory final {
  public:
    /**
     * @brief [en] Take the two limits. A move with a limit that is not a positive number never leaves its start.
     * @brief [zh] 接收两个限值。限值不是正数的运动永远不会离开起点。
     * @param limits [en] the cruise ceiling and the rate at which the velocity may change.
     * @param limits [zh] 巡航上限，以及速度允许的变化速率。
     * @note [en] Nothing is refused here, because there is nothing to refuse at construction: `move_to()` answers
     *            `Param` and moves nothing, which is where the caller is looking anyway.
     * @note [zh] 这里不拒绝任何东西，因为构造时没有可拒的东西： `move_to()` 会答 `Param` 且什么都不动，而那才是调用方在
     *            看的地方。
     * @since 0.1.0
     */
    explicit MoveTrajectory(const MoveLimits& limits) noexcept : limits_(limits) {}

    /**
     * @brief [en] Aim at a new mechanical angle, from wherever the reference is right now.
     * @brief [zh] 从参考此刻所在的地方，瞄准一个新的机械角。
     * @param target_rad [en] the mechanical angle to end on, rad.
     * @param target_rad [zh] 要停在上面的机械角，rad。
     * @return [en] `Ok`, or `Param` for a target that is not a number or limits that cannot describe motion. On `Param`
     *              nothing changes -- not the target, not the position.
     * @return [zh] `Ok`；目标不是数、或限值描述不了运动时返回 `Param`。返回 `Param` 时什么都不改——目标不改、位置不改。
     * @post [en] `point().position_rad` and `point().velocity_rad_s` are bit-identical to what they were before the
     *            call. Re-aiming cannot move the reference; only `step()` does.
     * @post [zh] `point().position_rad` 与 `point().velocity_rad_s` 与调用前逐位相同。换目标不可能移动参考；只有
     *            `step()` 会。
     * @since 0.1.0
     */
    auto move_to(float target_rad) noexcept -> ErrorCode {
        if (!usable() || !std::isfinite(target_rad)) {
            return ErrorCode::Param;
        }
        target_rad_ = static_cast<double>(target_rad);
        point_.done = (position_rad_ == target_rad_);
        if (point_.done) {
            velocity_rad_s_ = 0.0;
            point_.velocity_rad_s = 0.0F;
        }
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] Advance one tick and answer where the reference is now.
     * @brief [zh] 推进一拍，并回答参考现在在哪。
     * @param dt_s [en] step length in s; `<= 0` or not a number is refused and changes nothing.
     * @param dt_s [zh] 步长，s； `<= 0` 或不是数会被拒绝且什么都不改。
     * @return [en] the point for this tick.
     * @return [zh] 本拍的点。
     * @post [en] the change in `velocity_rad_s` is at most `accel_rad_s2 * dt_s`, `|velocity_rad_s|` never exceeds the
     *            cruise ceiling nor `sqrt(2*a*(|target - position| + |velocity_rad_s|*dt_s))` -- the stopping distance
     *            counting the sample being taken -- and `position_rad` never passes the target: it lands on it, sets
     *            `done`, and then holds.
     * @post [zh] `velocity_rad_s` 每拍变化不超过 `accel_rad_s2 * dt_s`； `|velocity_rad_s|` 不超过巡航上限，也不超过
     *            `sqrt(2*a*(|目标 − 位置| + |velocity_rad_s|*dt_s))`（把正在取的那一个采样算进去的制动距离）；
     *            `position_rad` 不会越过目标——它落在目标上、置 `done`，然后保持。
     * @since 0.1.0
     */
    auto step(float dt_s) noexcept -> const TrajectoryPoint& {
        if (!usable() || !(dt_s > 0.0F)) {
            return point_;
        }
        if (point_.done) {
            velocity_rad_s_ = 0.0;
            point_.velocity_rad_s = 0.0F;
            return point_;
        }
        const double remaining_rad = target_rad_ - position_rad_;
        const double direction = (remaining_rad > 0.0) ? 1.0 : -1.0;
        const double accel = static_cast<double>(limits_.accel_rad_s2);
        // The fastest this may be going and still stop on the target, then the ceiling, then the ramp.
        // 还能停在目标上的最快速度，再取巡航上限，再限斜率。
        const double ahead_rad = std::fabs(remaining_rad) - (std::fabs(velocity_rad_s_) * static_cast<double>(dt_s));
        const double stop_speed_rad_s = std::sqrt(2.0 * accel * ((ahead_rad > 0.0) ? ahead_rad : 0.0));
        const double ceiling_rad_s = std::min(stop_speed_rad_s, static_cast<double>(limits_.velocity_rad_s));
        const double wanted_rad_s = direction * ceiling_rad_s;
        const double max_change_rad_s = accel * static_cast<double>(dt_s);
        double velocity_rad_s = velocity_rad_s_;
        if (wanted_rad_s > (velocity_rad_s + max_change_rad_s)) {
            velocity_rad_s += max_change_rad_s;
        } else if (wanted_rad_s < (velocity_rad_s - max_change_rad_s)) {
            velocity_rad_s -= max_change_rad_s;
        } else {
            velocity_rad_s = wanted_rad_s;
        }
        // Exact for a velocity that is linear in time across the step, which it is between two limits.
        // 对「步内速度随时间线性」的情形是精确的，而两个限值之间正是这种情形。
        double position_rad = position_rad_ + (0.5 * (velocity_rad_s_ + velocity_rad_s) * static_cast<double>(dt_s));
        if (((direction > 0.0) && (position_rad >= target_rad_)) ||
            ((direction < 0.0) && (position_rad <= target_rad_))) {
            position_rad = target_rad_;
            velocity_rad_s = 0.0;
            point_.done = true;
        }
        position_rad_ = position_rad;
        velocity_rad_s_ = velocity_rad_s;
        point_.position_rad = static_cast<float>(position_rad_);
        point_.velocity_rad_s = static_cast<float>(velocity_rad_s_);
        return point_;
    }

    /**
     * @brief [en] The point of the last step.
     * @brief [zh] 最近一步的点。
     * @return [en] the point; `{0, 0, true}` before the first `move_to()`, which is "nothing is pending".
     * @return [zh] 该点；第一次 `move_to()` 之前为 `{0, 0, true}`，意思是「没有待办」。
     * @since 0.1.0
     */
    [[nodiscard]] auto point() const noexcept -> const TrajectoryPoint& { return point_; }

    /**
     * @brief [en] What it is aiming at.
     * @brief [zh] 它正瞄准什么。
     * @since 0.1.0
     */
    [[nodiscard]] auto target_rad() const noexcept -> float { return static_cast<float>(target_rad_); }

    /**
     * @brief [en] Whether a move is still pending.
     * @brief [zh] 是否还有运动没做完。
     * @since 0.1.0
     */
    [[nodiscard]] auto done() const noexcept -> bool { return point_.done; }

    /**
     * @brief [en] The limits in force.
     * @brief [zh] 当前生效的限值。
     * @since 0.1.0
     */
    [[nodiscard]] auto limits() const noexcept -> const MoveLimits& { return limits_; }

    /**
     * @brief [en] Put the reference at rest on a given angle, ready for a new move.
     * @brief [zh] 把参考静止放在给定的角度上，准备下一次运动。
     * @param position_rad [en] where the reference should sit, rad; not a number leaves it where it is.
     * @param position_rad [zh] 参考应该待在的角度，rad；不是数时保持原处。
     * @post [en] the target becomes that angle, so a caller that resets to the measured position starts from rest
     *            without an immediate error.
     * @post [zh] 目标变成那个角度，因此把参考重置到实测位置的调用方是从静止开始的，没有立刻的误差。
     * @since 0.1.0
     */
    void reset(float position_rad) noexcept {
        if (!std::isfinite(position_rad)) {
            return;
        }
        position_rad_ = static_cast<double>(position_rad);
        velocity_rad_s_ = 0.0;
        target_rad_ = position_rad_;
        point_ = TrajectoryPoint{position_rad, 0.0F, true};
    }

  private:
    [[nodiscard]] auto usable() const noexcept -> bool {
        return std::isfinite(limits_.velocity_rad_s) && std::isfinite(limits_.accel_rad_s2) &&
               (limits_.velocity_rad_s > 0.0F) && (limits_.accel_rad_s2 > 0.0F);
    }

    MoveLimits limits_{0.0F, 0.0F};
    // The accumulation runs in `double` and only the answer is a `float`. Adding a hundred-thousandth to a number
    // near one, fourteen thousand times, is exactly where single precision loses the tail: in float this class
    // drifted 1.2e-4 rad ahead of its own plan, which at 2 rad/s is **92 ticks** of arrival time, and the last step
    // then had to drop 0.047 rad/s at once instead of one ramp step. Everything on the wire is still `float`.
    // 累加在 `double` 里跑，只有最后的答案才是 `float`。把「万分之一」加到一个接近 1 的数上、加一万四千次，正是单精度会
    // 丢掉尾巴的地方：用 float 时这个类比自己规划的位置多走了 1.2e-4 rad，在 2 rad/s 下就是 **92 拍** 的到达时间，
    // 而最后一步不得不一次丢掉 0.047 rad/s，而不是一个斜坡步。对外交出去的仍然是 `float`。
    double position_rad_{0.0};
    double velocity_rad_s_{0.0};
    double target_rad_{0.0};
    TrajectoryPoint point_{0.0F, 0.0F, true};
};

} // namespace core

} // namespace foc
