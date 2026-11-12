// The reference a move is followed through, rather than only its endpoint. The unit cases assert the three things a
// trajectory promises -- the cruise holds the ceiling bit-for-bit, the finish time converges on the closed form
// `D/v + v/a`, and a new target mid-move is a value that cannot jump -- and the SIL case measures the one consequence
// that tells it apart from a shaped velocity command: the gap between the setpoint the loop was given and the axis.
// 一次运动被跟随的那条参考，而不是只有终点。单元用例断言轨迹承诺的三件事——巡航逐位保持上限、终点时刻收敛到闭式
// `D/v + v/a`、中途换目标不可能跳变——SIL 用例则量出它与「速度塑造型指令」分开的那一个后果：环被告知的设定点与轴之间
// 的间隙。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>

#include <gtest/gtest.h>

#include <foc/core/motion_profile.hpp>
#include <foc/core/move_trajectory.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/position.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"


namespace {

constexpr float kDt = 5e-5F;
constexpr float kTargetRad = 1.0F;
constexpr float kVelocityLimit = 2.0F;
constexpr float kAccelLimit = 10.0F;

constexpr int kSubsteps = 4;
constexpr int kPolePairs = 7;
constexpr float kVBus = 24.0F;

/**
 * @brief [en] What an answer is allowed to be off by, because it is reported as a `float`.
 * @brief [zh] 答案允许的偏差，因为它是按 `float` 报出来的。
 * @note [en] Not a fudge: `TrajectoryPoint` is `float` like everything else on the wire, so a position near 1 rad
 *            carries about 6e-8 and a velocity near 2 rad/s about 1.2e-7, and a difference of two of them carries twice
 *            that. Every bound below is "the limit, plus what the resolution of the answer itself can add" -- without
 *            that, a case fails on the last bit of a float and says nothing about the trajectory.
 * @note [zh] 不是放水： `TrajectoryPoint` 和线上其他东西一样是 `float`，所以 1 rad 附近的位置带着约 6e-8、2 rad/s 附近
 *            的速度带着约 1.2e-7，而两个这样的数相减带着两倍。下面每个界都是「限值，加上答案自身分辨率可能加上的那一
 *            点」 ——否则用例会输在 float 的最后一个比特上，而那与轨迹毫无关系。
 */
constexpr float kSlackRad = 1.0e-6F;
constexpr float kSlackRadS = 1.0e-5F;

/**
 * @brief [en] The per-tick properties that must hold for *every* reference this file produces.
 * @brief [zh] 本文件产生的 **每一条** 参考都必须满足的单拍性质。
 * @note [en] `worst_stop_excess` is the *sampled* stopping test, not the plane one: a reference that advances in steps
 *            of `dt` has to be able to stop within `|remaining| + |v|*dt`, because the tick it is about to take also
 *            consumes distance. The plane curve `v <= sqrt(2*a*|remaining|)` is the `dt -> 0` limit of this, and
 *            asserting the plane form at a finite step fails by 2 % on a correct implementation.
 * @note [zh] `worst_stop_excess` 是 **采样版** 的「停得住」判据，不是平面版：按 `dt` 逐步推进的参考必须能在
 *            `|剩距| + |v|*dt` 内停下，因为它即将走的那一拍也消耗距离。平面曲线 `v <= sqrt(2*a*|剩距|)` 是它在
 *            `dt -> 0` 时的极限，在有限步长下断言平面形式会让一个正确的实现以 2% 的偏差失败。
 */
struct Invariants final {
    float worst_change_rad_s{0.0F}; ///< [en] largest |dv| in one tick. [zh] 单拍 |dv| 的最大值。
    float worst_stop_excess{0.0F};  ///< [en] largest v^2 / (2*a*(|remaining| + v*dt)). [zh] v^2 / (2*a*(|剩距|+v*dt)) 的最大值。
    float worst_jump_rad{0.0F};     ///< [en] largest single-tick |dposition|. [zh] 单拍 |dposition| 的最大值。
    float max_position_rad{-1.0e9F};
    bool monotone{true};
};

struct MoveResult final {
    Invariants invariants;
    int ticks{0};
    float finish_s{0.0F};
    float peak_velocity_rad_s{0.0F};
    int cruise_ticks{0};
    float cruise_travel_rad{0.0F};
    float cruise_min_rad_s{0.0F};
    float cruise_max_rad_s{0.0F};
    float final_position_rad{0.0F};
    float final_velocity_rad_s{0.0F};
    bool done{false};
};

/**
 * @brief [en] Run one move to completion, watching every tick.
 * @brief [zh] 把一次运动跑完，同时盯住每一拍。
 */
auto run_move(float dt_s, float target_rad, int max_ticks) -> MoveResult {
    foc::core::MoveTrajectory traj(foc::core::MoveLimits{kVelocityLimit, kAccelLimit});
    MoveResult result{};
    result.cruise_min_rad_s = 1.0e9F;
    result.cruise_max_rad_s = -1.0e9F;
    if (traj.move_to(target_rad) != foc::ErrorCode::Ok) {
        return result;
    }
    auto before = traj.point();
    while (!traj.done() && (result.ticks < max_ticks)) {
        const auto& now = traj.step(dt_s);
        const float remaining = std::fabs(target_rad - now.position_rad);
        const float stop_budget = 2.0F * kAccelLimit * (remaining + (std::fabs(now.velocity_rad_s) * dt_s));
        result.invariants.worst_change_rad_s =
            std::max(result.invariants.worst_change_rad_s, std::fabs(now.velocity_rad_s - before.velocity_rad_s));
        result.invariants.worst_stop_excess =
            std::max(result.invariants.worst_stop_excess,
                     (now.velocity_rad_s * now.velocity_rad_s) / std::max(stop_budget, 1.0e-12F));
        result.invariants.worst_jump_rad =
            std::max(result.invariants.worst_jump_rad, std::fabs(now.position_rad - before.position_rad));
        result.invariants.max_position_rad = std::max(result.invariants.max_position_rad, now.position_rad);
        if (now.position_rad < before.position_rad) {
            result.invariants.monotone = false;
        }
        if (now.velocity_rad_s == kVelocityLimit) {
            result.cruise_min_rad_s = std::min(result.cruise_min_rad_s, now.velocity_rad_s);
            result.cruise_max_rad_s = std::max(result.cruise_max_rad_s, now.velocity_rad_s);
            result.cruise_travel_rad += now.position_rad - before.position_rad;
            ++result.cruise_ticks;
        }
        result.peak_velocity_rad_s = std::max(result.peak_velocity_rad_s, now.velocity_rad_s);
        before = now;
        ++result.ticks;
    }
    result.finish_s = static_cast<float>(result.ticks) * dt_s;
    result.final_position_rad = traj.point().position_rad;
    result.final_velocity_rad_s = traj.point().velocity_rad_s;
    result.done = traj.done();
    return result;
}

struct CruiseGap final {
    float mean_rad{0.0F};
    float worst_jump_rad{0.0F};
    float final_err_rad{0.0F};
    float peak_speed_rad_s{0.0F};
    int cruise_ticks{0};
};

/**
 * @brief [en] Drive one move on the SIL bench and report the gap between the setpoint in force and the axis.
 * @brief [zh] 在 SIL 台架上跑一次运动，报告当时生效的设定点与轴之间的间隙。
 * @param mc [en] the machine, shared so that every row is the same bench.
 * @param mc [zh] 机器参数，共用以保证每一行是同一个台架。
 * @param position_control [en] the loop that follows the setpoint.
 * @param position_control [zh] 跟随设定点的那个环。
 * @param stream [en] `true` to hand the axis a new point every tick from a trajectory, `false` to hand it the endpoint
 *                    once.
 * @param stream [zh] `true` 表示每拍从轨迹取一个新点交给轴， `false` 表示只把终点交给它一次。
 */
template <typename PositionControlT>
auto measure_cruise_gap(const foc::motor::MotorConfig& mc, PositionControlT position_control, bool stream) -> CruiseGap {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, foc::core::NullFeedforward, PositionControlT>;
    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
    foc::sil::PlantHal hal(plant, kVBus);
    const typename MotorT::Policies policies{foc::core::BackEmfObserver({mc.rs, mc.ld}),
                                             foc::core::Svpwm{},
                                             foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
                                             foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
                                             foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
                                             foc::core::NullReferenceGenerator{},
                                             foc::core::NullFeedforward{},
                                             position_control};
    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());

    foc::core::MoveTrajectory traj(foc::core::MoveLimits{kVelocityLimit, kAccelLimit});
    float setpoint = 0.0F;
    if (stream) {
        static_cast<void>(traj.move_to(kTargetRad));
    } else {
        setpoint = kTargetRad; // no trajectory: the endpoint is the setpoint from the first tick
    }
    motor.set_position(setpoint);
    static_cast<void>(motor.start());

    CruiseGap gap{};
    float previous_setpoint = setpoint;
    const int ticks = static_cast<int>(1.0F / kDt);
    for (int i = 0; i < ticks; ++i) {
        if (stream) {
            setpoint = traj.step(kDt).position_rad;
            motor.set_position(setpoint);
        }
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        const float speed = std::fabs(motor.telemetry().speed_mech_rad_s);
        const float actual = hal.position.angle_mech_rad();
        gap.worst_jump_rad = std::max(gap.worst_jump_rad, std::fabs(setpoint - previous_setpoint));
        gap.peak_speed_rad_s = std::max(gap.peak_speed_rad_s, speed);
        if (speed >= 0.95F * kVelocityLimit) {
            gap.mean_rad += std::fabs(setpoint - actual);
            ++gap.cruise_ticks;
        }
        previous_setpoint = setpoint;
    }
    if (gap.cruise_ticks > 0) {
        gap.mean_rad /= static_cast<float>(gap.cruise_ticks);
    }
    gap.final_err_rad = std::fabs(hal.position.angle_mech_rad() - kTargetRad);
    return gap;
}

} // namespace


/**
 * @brief [en] A move long enough to cruise: the cruise holds the ceiling without moving at all, and the tick it lands
 *             on is the closed form `D/v + v/a` to within two ticks.
 *
 * @brief [zh] 一次长到足以巡航的运动：巡航段逐位保持上限、一动不动，而落地的那一拍就是闭式 `D/v + v/a`，误差不超过两
 *             拍。
 *
 *   The two numbers are not read off the configuration, they are derived from it: `D/v + v/a = 1/2 + 2/10 = 0.7 s` and
 *   the cruise covers `D - v^2/a = 0.6 rad`. Both are then compared against what the run did, so a change to the law
 *   that still "looks like a trapezoid" but moves either number fails here.
 *
 *   The residual is the sample, and this case proves that rather than asserting a tolerance: the same move is run again
 *   with the step four times shorter, and the error has to shrink with it. A law that was simply *wrong* keeps its
 *   error when the step is refined; a sampled law loses it in proportion.
 *
 * @note [en] Criterion, written before the run: the landing tick is within *two* ticks of `D/v + v/a` (one for the
 *            sample, one for the rounding of a float multiply), the cruise window is non-empty (otherwise the constancy
 *            assertion is vacuous), the cruise covers `D - v^2/a` within 1 mrad, and refining the step by 4x cuts the
 *            timing error by at least half.
 * @note [zh] 判据（写在本次测量之前）：落地那一拍与 `D/v + v/a` 相差不超过 **两** 拍（一拍给采样、一拍给 float 乘法的舍
 *            入）；巡航窗口非空（否则「恒定」这条断言是空的）；巡航走过的距离与 `D - v^2/a` 相差不超过 1 mrad；把步长细
 *            四倍，时间误差至少减半。
 */
TEST(MoveTrajectory, AFullTrapezoidCruisesAtTheCeilingAndLandsWhenTheFormulaSays) {
    const auto coarse = run_move(kDt, kTargetRad, 40000);
    const auto fine = run_move(0.25F * kDt, kTargetRad, 160000);

    const float predicted_s = (kTargetRad / kVelocityLimit) + (kVelocityLimit / kAccelLimit);
    const float cruise_distance = kTargetRad - ((kVelocityLimit * kVelocityLimit) / kAccelLimit);
    const float coarse_error = std::fabs(coarse.finish_s - predicted_s);
    const float fine_error = std::fabs(fine.finish_s - predicted_s);
    std::printf("\n[trapezoid] landed after %d ticks = %0.5f s, closed form %0.5f s, error %0.4f ms\n", coarse.ticks,
                static_cast<double>(coarse.finish_s), static_cast<double>(predicted_s),
                static_cast<double>(1.0e3F * coarse_error));
    std::printf("%-22s %12s %10s %12s\n", "cruise", "travelled", "ticks", "measured");
    std::printf("%-22s %12.5f %10d %12.5f\n", "predicted", static_cast<double>(cruise_distance), 6000,
                static_cast<double>(coarse.cruise_travel_rad));
    std::printf("%-22s %12s %10d %12.4f ms\n", "step / 4", "", fine.ticks, static_cast<double>(1.0e3F * fine_error));

    EXPECT_TRUE(coarse.done);
    // The reference lands on the target exactly, not near it, and it stops there.
    EXPECT_EQ(kTargetRad, coarse.final_position_rad);
    EXPECT_EQ(0.0F, coarse.final_velocity_rad_s);
    EXPECT_LE(coarse_error, 2.0F * kDt);
    EXPECT_GT(coarse.cruise_ticks, 0);
    // Bit-for-bit constant: a cruise whose speed wanders is not a cruise.
    EXPECT_EQ(coarse.cruise_min_rad_s, coarse.cruise_max_rad_s);
    EXPECT_EQ(kVelocityLimit, coarse.cruise_max_rad_s);
    EXPECT_NEAR(cruise_distance, coarse.cruise_travel_rad, 1.0e-3F);
    // Refining the step shrinks the error: what is left is the sample, not the formula.
    EXPECT_LT(fine_error, 0.5F * coarse_error);
    ASSERT_GT(fine.cruise_ticks, 0);
    EXPECT_EQ(fine.cruise_min_rad_s, fine.cruise_max_rad_s);
    // Every tick respected the limits it was given.
    EXPECT_LE(coarse.invariants.worst_change_rad_s, (kAccelLimit * kDt) + kSlackRadS);
    EXPECT_LE(coarse.invariants.worst_stop_excess, 1.0F + kSlackRadS);
    EXPECT_TRUE(coarse.invariants.monotone);
}


/**
 * @brief [en] A move too short to reach the ceiling is a triangle, and its peak and its duration are both closed forms
 *             that the geometry fixes rather than the limits alone.
 *
 * @brief [zh] 短到够不着上限的运动是个三角形：它的峰值与时长都是几何定下来的闭式，不由限值单独决定。
 *
 *   `D = 0.2 rad` against `v^2/a = 0.4 rad`, so the ceiling is never reached: the peak is `sqrt(a*D) = 1.41421 rad/s`
 *   and the move takes `2*sqrt(D/a) = 0.28284 s`.
 *   **A trajectory that only ever compared against the ceiling would still look correct here** -- it would ramp up, run
 *   out of distance and decelerate -- but the two numbers would be wrong, which is what these assertions catch.
 *
 * @note [en] Criterion, written before the run: the peak speed agrees with `sqrt(a*D)` within 1 %, the ceiling is never
 *            hit exactly, and the duration agrees with `2*sqrt(D/a)` within two ticks.
 * @note [zh] 判据（写在本次测量之前）：峰值速度与 `sqrt(a*D)` 相差 ≤1%，上限一次也没被精确达到，时长与 `2*sqrt(D/a)` 相
 *            差不超过两拍。
 */
TEST(MoveTrajectory, AShortMoveIsATriangleAndItsPeakIsTheQuadraticRootOfTheGeometry) {
    constexpr float kDistanceRad = 0.2F;
    const auto triangle = run_move(kDt, kDistanceRad, 40000);

    const float predicted_peak = std::sqrt(kAccelLimit * kDistanceRad);
    const float predicted_s = 2.0F * std::sqrt(kDistanceRad / kAccelLimit);
    std::printf("\n[triangle] peak %0.5f vs %0.5f rad/s, landed %0.6f vs %0.6f s\n",
                static_cast<double>(triangle.peak_velocity_rad_s), static_cast<double>(predicted_peak),
                static_cast<double>(triangle.finish_s), static_cast<double>(predicted_s));

    EXPECT_TRUE(triangle.done);
    EXPECT_NEAR(predicted_peak, triangle.peak_velocity_rad_s, 0.01F * predicted_peak);
    EXPECT_LT(triangle.peak_velocity_rad_s, kVelocityLimit);
    EXPECT_EQ(0, triangle.cruise_ticks);
    EXPECT_LE(std::fabs(triangle.finish_s - predicted_s), 2.0F * kDt);
    EXPECT_EQ(kDistanceRad, triangle.final_position_rad);
    EXPECT_LE(triangle.invariants.worst_change_rad_s, (kAccelLimit * kDt) + kSlackRadS);
    EXPECT_LE(triangle.invariants.worst_stop_excess, 1.0F + kSlackRadS);
}


/**
 * @brief [en] Re-aiming mid-cruise moves no reference: the value handed out before the call is the value handed out
 *             after it, bit for bit, and the motion continues from there.
 *
 * @brief [zh] 巡航途中换目标不动任何参考：调用前交出去的值与调用后交出去的值逐位相同，运动从那里继续。
 *
 *   This is the streaming-setpoint case: a caller that steers the axis by re-aiming every tick must not be able to
 *   produce a discontinuity in the reference, and here that is not a clamp but a consequence -- `move_to()` writes one
 *   field and no other. The jump detector below is deliberately dumb (a single-tick move larger than the ceiling
 *   allows) so that it catches a discontinuity wherever it comes from, including a future refactor that starts
 *   re-planning from the target instead of from the current point.
 *
 * @note [en] Criterion, written before the run: the position and velocity are equal before and after the re-aim, no
 *            tick moves the reference further than the ceiling allows, the reference never passes the new target, and
 *            it lands on it exactly.
 * @note [zh] 判据（写在本次测量之前）：换目标前后位置与速度相等；没有哪一拍把参考移动得超过上限允许的量；参考从不越过新
 *            目标；并且精确落在它上面。
 */
TEST(MoveTrajectory, ANewTargetMidCruiseIsANumberTheReferenceCannotJumpTo) {
    constexpr float kNewTargetRad = 0.7F;
    constexpr int kReaimTick = 6000; ///< [en] t = 0.3 s, which is 0.1 s into the cruise. [zh] t = 0.3 s，即巡航中的第 0.1 s。

    foc::core::MoveTrajectory traj(foc::core::MoveLimits{kVelocityLimit, kAccelLimit});
    ASSERT_EQ(foc::ErrorCode::Ok, traj.move_to(kTargetRad));
    for (int i = 0; i < kReaimTick; ++i) {
        static_cast<void>(traj.step(kDt));
    }
    const float position_before = traj.point().position_rad;
    const float velocity_before = traj.point().velocity_rad_s;
    ASSERT_GT(velocity_before, 0.0F);

    ASSERT_EQ(foc::ErrorCode::Ok, traj.move_to(kNewTargetRad));
    EXPECT_EQ(position_before, traj.point().position_rad);
    EXPECT_EQ(velocity_before, traj.point().velocity_rad_s);
    EXPECT_FALSE(traj.done());

    Invariants inv{};
    int ticks = 0;
    auto before = traj.point();
    while (!traj.done() && (ticks < 40000)) {
        const auto& now = traj.step(kDt);
        const float remaining = std::fabs(kNewTargetRad - now.position_rad);
        const float stop_budget = 2.0F * kAccelLimit * (remaining + (std::fabs(now.velocity_rad_s) * kDt));
        inv.worst_change_rad_s = std::max(inv.worst_change_rad_s, std::fabs(now.velocity_rad_s - before.velocity_rad_s));
        inv.worst_jump_rad = std::max(inv.worst_jump_rad, std::fabs(now.position_rad - before.position_rad));
        inv.worst_stop_excess =
            std::max(inv.worst_stop_excess, (now.velocity_rad_s * now.velocity_rad_s) / std::max(stop_budget, 1.0e-12F));
        inv.max_position_rad = std::max(inv.max_position_rad, now.position_rad);
        if (now.position_rad < before.position_rad) {
            inv.monotone = false;
        }
        before = now;
        ++ticks;
    }

    std::printf("\n[re-aim] from %0.5f rad at %0.4f rad/s, largest single-tick move %0.8f, lands after %d ticks\n",
                static_cast<double>(position_before), static_cast<double>(velocity_before),
                static_cast<double>(inv.worst_jump_rad), ticks);

    EXPECT_TRUE(traj.done());
    EXPECT_EQ(kNewTargetRad, traj.point().position_rad);
    EXPECT_EQ(0.0F, traj.point().velocity_rad_s);
    EXPECT_TRUE(inv.monotone);
    EXPECT_LE(inv.max_position_rad, kNewTargetRad);
    // No discontinuity anywhere in the streamed reference. This is the assertion criterion (3) is about.
    EXPECT_LE(inv.worst_jump_rad, (kVelocityLimit * kDt) + kSlackRad);
    EXPECT_LE(inv.worst_change_rad_s, (kAccelLimit * kDt) + kSlackRadS);
    EXPECT_LE(inv.worst_stop_excess, 1.0F + kSlackRadS);
}


/**
 * @brief [en] A target that appears behind the reference is reached by braking first: it travels one more `v^2/(2a)` in
 *             the old direction, reverses, and lands on the target.
 *
 * @brief [zh] 出现在参考身后的目标要先刹住才能到达：它沿旧方向再走 `v^2/(2a)`，然后反向，落在目标上。
 *
 *   It is worth being explicit about what this case is *not* claiming: the reference does not jump backwards, and it
 *   does not stop being a position. It travels `v^2/(2a) = 0.2 rad` past the point where it was re-aimed -- the same
 *   distance any axis needs to stop from 2 rad/s at 10 rad/s^2 -- and only then comes back. A trajectory that reversed
 *   instantly would satisfy "it arrives" and fail this.
 *
 * @note [en] Criterion, written before the run: the overshoot beyond the re-aim point agrees with `v^2/(2a)` within 2
 *            %, the velocity never changes faster than the limit, and it lands on the new target exactly.
 * @note [zh] 判据（写在本次测量之前）：越过换目标点的量与该刻速度的 `v^2/(2a)` 相差 ≤2%；速度变化从不快于限值；并且精确
 *            落在新目标上。
 */
TEST(MoveTrajectory, ATargetBehindTheReferenceIsReachedByBrakingFirst) {
    constexpr int kReaimTick = 6000; ///< [en] t = 0.3 s: at the ceiling, 0.4 rad travelled. [zh] t = 0.3 s：在上限上，已走 0.4 rad。
    foc::core::MoveTrajectory traj(foc::core::MoveLimits{kVelocityLimit, kAccelLimit});
    ASSERT_EQ(foc::ErrorCode::Ok, traj.move_to(kTargetRad));
    for (int i = 0; i < kReaimTick; ++i) {
        static_cast<void>(traj.step(kDt));
    }
    const float position_before = traj.point().position_rad;
    const float velocity_before = traj.point().velocity_rad_s;
    ASSERT_EQ(kVelocityLimit, velocity_before);

    ASSERT_EQ(foc::ErrorCode::Ok, traj.move_to(0.0F));
    float worst_change = 0.0F;
    float furthest = -1.0e9F;
    int ticks = 0;
    auto before = traj.point();
    while (!traj.done() && (ticks < 40000)) {
        const auto& now = traj.step(kDt);
        worst_change = std::max(worst_change, std::fabs(now.velocity_rad_s - before.velocity_rad_s));
        furthest = std::max(furthest, now.position_rad);
        before = now;
        ++ticks;
    }

    const float stopping_distance = (velocity_before * velocity_before) / (2.0F * kAccelLimit);
    const float overshoot = furthest - position_before;
    std::printf("\n[reverse] from %0.5f rad at %0.4f rad/s, went on to %0.5f (+%0.5f vs %0.5f predicted)\n",
                static_cast<double>(position_before), static_cast<double>(velocity_before),
                static_cast<double>(furthest), static_cast<double>(overshoot),
                static_cast<double>(stopping_distance));

    EXPECT_TRUE(traj.done());
    EXPECT_EQ(0.0F, traj.point().position_rad);
    EXPECT_NEAR(stopping_distance, overshoot, 0.02F * stopping_distance);
    EXPECT_LE(worst_change, (kAccelLimit * kDt) + kSlackRadS);
}


/**
 * @brief [en] Limits that cannot describe motion, and targets that are not numbers, are refused and change nothing.
 *
 * @brief [zh] 描述不了运动的限值、以及不是数的目标，会被拒绝，且什么都不改。
 *
 *   A trajectory is asked to move by one call and advanced by another, so "refused" has to mean *nothing moved*: not
 *   the target, not the position, and not the pending flag. The alternative -- silently holding the old target -- is
 *   the failure this case exists to prevent, because the caller's next log line would then be about a move it did not
 *   ask for.
 *
 * @note [en] `dt_s` of zero, negative or `NaN` is the same rule: refused, nothing changes. That is not pedantry; a zero
 *            period silently reported as a completed tick would make a caller's loop counter disagree with the
 *            reference.
 * @note [zh] `dt_s` 为零、为负或为 `NaN` 是同一条规矩：拒绝，什么都不改。这不是迂腐：把零周期静默当成完成了一拍，会让调
 *            用方的循环计数与参考对不上。
 */
TEST(MoveTrajectory, NumbersThatCannotDescribeAMoveAreRefusedAndNothingMoves) {
    const auto nan = std::nanf("");

    const foc::core::MoveLimits unusable[] = {
        {0.0F, kAccelLimit},             // no speed
        {kVelocityLimit, 0.0F},          // no acceleration
        {-kVelocityLimit, kAccelLimit},  // backwards
        {nan, kAccelLimit},
        {kVelocityLimit, std::numeric_limits<float>::infinity()},
    };
    for (const auto& limits : unusable) {
        foc::core::MoveTrajectory traj(limits);
        EXPECT_EQ(foc::ErrorCode::Param, traj.move_to(kTargetRad));
        static_cast<void>(traj.step(kDt));
        EXPECT_EQ(0.0F, traj.point().position_rad);
        EXPECT_EQ(0.0F, traj.point().velocity_rad_s);
        EXPECT_TRUE(traj.done());
    }

    foc::core::MoveTrajectory traj(foc::core::MoveLimits{kVelocityLimit, kAccelLimit});
    ASSERT_EQ(foc::ErrorCode::Ok, traj.move_to(kTargetRad));
    for (int i = 0; i < 100; ++i) {
        static_cast<void>(traj.step(kDt));
    }
    const float position = traj.point().position_rad;
    const float velocity = traj.point().velocity_rad_s;
    ASSERT_GT(velocity, 0.0F);

    // A target that is not a number leaves the old one in force.
    EXPECT_EQ(foc::ErrorCode::Param, traj.move_to(nan));
    EXPECT_EQ(kTargetRad, traj.target_rad());
    // A period that is not a positive number leaves the point where it was.
    for (const float dt : {0.0F, -kDt, nan}) {
        const auto& held = traj.step(dt);
        EXPECT_EQ(position, held.position_rad);
        EXPECT_EQ(velocity, held.velocity_rad_s);
        EXPECT_FALSE(held.done);
    }

    // reset() puts it at rest where it is told, and ignores what is not a number.
    traj.reset(position);
    EXPECT_EQ(position, traj.point().position_rad);
    EXPECT_EQ(0.0F, traj.point().velocity_rad_s);
    EXPECT_TRUE(traj.done());
    EXPECT_EQ(position, traj.target_rad());
    traj.reset(nan);
    EXPECT_EQ(position, traj.point().position_rad);

    // Re-aiming at where it already is completes immediately rather than after a full step.
    EXPECT_EQ(foc::ErrorCode::Ok, traj.move_to(position));
    EXPECT_TRUE(traj.done());
}


/**
 * @brief [en] The measured difference from a shaped velocity command: what the loop is told to reach during the cruise.
 *
 * @brief [zh] 与「速度塑造型指令」之间被量出来的差别：巡航期间环被告知要去哪儿。
 *
 *   Three runs on the same bench, same final target, same limits and the same gain, each with the position controller
 *   that goes with its setpoint: a constant endpoint with the library's own shaper (`MotionProfile`), the streamed
 *   reference with a plain proportional loop (the shaping has already been done by the generator), and -- so that the
 *   attribution is visible rather than assumed -- the streamed reference with the *same* shaper as the first run. The
 *   cruise is identified the same way in all three: ticks on which the axis is actually travelling at the ceiling.
 *
 *   Without a trajectory the setpoint is the endpoint for the whole move, so the gap during the cruise *is* the
 *   remaining distance. With one, the setpoint is a ramp the axis can be next to, and the same gap is a tracking lag.
 *   The third run is the interesting one: two shapers in series fight, because a controller that limits its velocity by
 *   `sqrt(2*a*error)` cannot follow a reference that is already spending the whole acceleration budget -- so the axis
 *   there crawls. That row is reported rather than asserted; it is the reason a trajectory wants a plain loop.
 *
 * @note [en] Criterion, written before the run: the `MotionProfile` mean cruise gap reproduces the recorded 0.496 rad
 *            within 30 % (if it does not, this comparison is against the wrong bench and means nothing), the streamed
 *            reference's mean cruise gap is at most half of it, and the streamed setpoint never jumps by more than the
 *            ceiling allows.
 * @note [zh] 判据（写在本次测量之前）： `MotionProfile` 侧的平均巡航间隙复现已记录的 0.496 rad 且偏差 ≤30%（若不能，这
 *            个比较就对着错的台架，毫无意义）；流式参考的平均巡航间隙不超过它的一半；并且流式设定点从未跳动得比上限允许
 *            的更多。
 */
TEST(MoveTrajectory, TheCruiseGapIsWhatTellsThisApartFromAShapedVelocityCommand) {
    constexpr float kGain = 20.0F;
    const foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
    const foc::core::MotionProfile profile{foc::core::MotionProfile::Config{kGain, kVelocityLimit, kAccelLimit}};
    const foc::core::PositionP proportional{foc::core::PositionP::Config{kGain, kVelocityLimit, kAccelLimit}};

    const CruiseGap endpoint = measure_cruise_gap(mc, profile, false);
    const CruiseGap streamed = measure_cruise_gap(mc, proportional, true);
    const CruiseGap fighting = measure_cruise_gap(mc, profile, true);
    std::printf("\n%-28s %12s %12s %10s %8s\n", "setpoint / position loop", "mean gap", "final err", "peak speed",
                "cruise");
    std::printf("%-28s %12.5f %12.5f %10.4f %8d\n", "endpoint + MotionProfile", static_cast<double>(endpoint.mean_rad),
                static_cast<double>(endpoint.final_err_rad), static_cast<double>(endpoint.peak_speed_rad_s),
                endpoint.cruise_ticks);
    std::printf("%-28s %12.5f %12.5f %10.4f %8d\n", "streamed + PositionP", static_cast<double>(streamed.mean_rad),
                static_cast<double>(streamed.final_err_rad), static_cast<double>(streamed.peak_speed_rad_s),
                streamed.cruise_ticks);
    std::printf("%-28s %12s %12.5f %10.4f %8d\n", "streamed + MotionProfile",
                (fighting.cruise_ticks > 0) ? "" : "no cruise", static_cast<double>(fighting.final_err_rad),
                static_cast<double>(fighting.peak_speed_rad_s), fighting.cruise_ticks);
    if (fighting.cruise_ticks > 0) {
        std::printf("%-28s %12.5f\n", "", static_cast<double>(fighting.mean_rad));
    }

    // Both sides of the comparison measured a cruise, or it is empty.
    ASSERT_GT(endpoint.cruise_ticks, 0);
    ASSERT_GT(streamed.cruise_ticks, 0);
    // The recorded 0.496 rad reproduces on this bench; otherwise the rows are not comparable.
    EXPECT_NEAR(0.496F, endpoint.mean_rad, 0.3F * 0.496F);
    // The claim: a position to be followed is a different thing from an endpoint to be chased.
    EXPECT_LE(streamed.mean_rad, 0.5F * endpoint.mean_rad);
    // And the streamed reference is continuous and the move still arrives.
    EXPECT_LE(streamed.worst_jump_rad, (kVelocityLimit * kDt) + kSlackRad);
    EXPECT_LE(streamed.final_err_rad, 0.02F);
    // Reported, not asserted as a target: two shapers in series cannot both spend the whole acceleration budget.
    EXPECT_LT(fighting.peak_speed_rad_s, streamed.peak_speed_rad_s);
}
