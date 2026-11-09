// S9 - accel-limited position motion. S8 uses a bare P position loop, whose velocity command jumps
// with the error; this scenario puts `core::MotionProfile` in the same slot and checks the invariants
// that matter on a joint: the command never exceeds the velocity limit, never changes faster than the
// acceleration limit, and still arrives.
// S9 - 加速度受限的位置运动。S8 用的是纯 P 位置环，其速度指令随误差跳变；本场景把
// `core::MotionProfile` 放进同一个槽位，检查关节真正在意的不变式：指令不越速度上限、变化不快于
// 加速度上限，而且照样到位。

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/motion_profile.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"



namespace {

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 4;
constexpr int kPolePairs = 7;
constexpr float kVBus = 24.0F;
constexpr float kTargetRad = 1.0F;
constexpr float kVelocityLimit = 2.0F;
constexpr float kAccelLimit = 10.0F;

struct Trace final {
    float worst_velocity;
    float worst_accel;      ///< [en] largest per-step velocity change divided by dt. [zh] 单步速度变化除以 dt 的最大值。
    float final_err_rad;
    float overshoot_rad;
    foc::ErrorCode status;
};

} // namespace



/**
 * @brief [en] The profile shapes the velocity command; the plant must show the same shape. The measured acceleration is
 *             bounded by the configured limit (the inner loop tracks the command much faster than the profile changes
 *             it), which is the whole point of putting the profile in front of the loop instead of inside it.
 * @brief [zh] 规划器塑造的是速度指令；plant 上量到的也必须是同样的形状。实测加速度受配置上限约束 （内层环跟踪指令远比规
 *             划器改变指令快得多）——这正是把规划器放在环 **前面** 而不是里面的意义。
 */
TEST(SilMotionProfile, AccelLimitedMoveRespectsItsLimitsAndStillArrives) {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                     foc::core::MotionProfile>;
    const foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
    foc::sil::PlantHal hal(plant, kVBus);

    const typename MotorT::Policies policies{
        foc::core::BackEmfObserver({mc.rs, mc.ld}),
        foc::core::Svpwm{},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
        foc::core::NullReferenceGenerator{},
        foc::core::NullFeedforward{},
        foc::core::MotionProfile{foc::core::MotionProfile::Config{20.0F, kVelocityLimit, kAccelLimit}}};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_position(kTargetRad);
    static_cast<void>(motor.start());

    Trace trace{0.0F, 0.0F, 0.0F, 0.0F, foc::ErrorCode::Ok};
    float previous_speed = 0.0F;
    float peak = 0.0F;
    const int steps = static_cast<int>(1.0F / kDt);
    for (int i = 0; i < steps; ++i) {
        trace.status = motor.tick(kDt);
        hal.advance(kDt, kSubsteps);

        const float speed = motor.telemetry().speed_mech_rad_s;
        const float position = hal.position.angle_mech_rad();
        trace.worst_velocity = std::max(trace.worst_velocity, std::fabs(speed));
        trace.worst_accel = std::max(trace.worst_accel, std::fabs(speed - previous_speed) / kDt);
        peak = std::max(peak, position);
        previous_speed = speed;
    }

    trace.overshoot_rad = peak - kTargetRad;
    trace.final_err_rad = std::fabs(hal.position.angle_mech_rad() - kTargetRad);

    EXPECT_EQ(foc::ErrorCode::Ok, trace.status);

    // The velocity limit binds but is not exceeded (the inner loop regulates it, so allow a little).
    EXPECT_LE(trace.worst_velocity, 1.05F * kVelocityLimit);
    EXPECT_GE(trace.worst_velocity, 0.90F * kVelocityLimit); // ...and it really did reach the limit

    // The acceleration limit holds on the plant, not only inside the profile.
    EXPECT_LE(trace.worst_accel, 1.5F * kAccelLimit);

    EXPECT_LE(trace.overshoot_rad, 0.02F);
    EXPECT_LE(trace.final_err_rad, 0.01F);
}



/**
 * @brief [en] The same move without the profile: the bare P loop turns the step into a velocity step, which is exactly
 *             the acceleration the profile exists to remove. This is the negative control that proves the test above is
 *             measuring something real.
 * @brief [zh] 同一次运动去掉规划器：纯 P 环把位置阶跃变成速度阶跃，而这正是规划器要消掉的加速度。 这条反向对照证明上面
 *             那条用例量的是真东西。
 */
/**
 * @brief [en] What the profile's own structure bounds: the cruise ends at the deceleration threshold, and the shaping
 *             gain does not set it.
 *
 * @brief [zh] 这个结构自身给出的界：巡航在减速阈值处结束，而 shaping gain 不决定它。
 *
 *   The hypothesis this case was written to test was `e = v_limit / shaping_gain`, the textbook following error of a
 *   first-order tracker on a ramp. **The measurement refuted it**: the cruise gap came out at 0.496 rad against 0.100
 *   predicted, and -- the part that settles it -- doubling the shaping gain left it at 0.496. A number that does not
 *   move when its supposed parameter doubles is not that parameter's.
 *
 *   The reason is structural, and it is worth having in the claim rather than in a comment: `MotionProfile` does not
 *   generate a position reference to follow. It shapes a *velocity* command from the position error, so there is no
 *   ramp and therefore no ramp-following error. What the profile does bound is where the cruise ends: it decelerates
 *   when the remaining distance reaches `v^2 / (2a)`, so the cruise gap is bounded below by that threshold and the
 *   dwell is a consequence of the two limits, not of the gain.
 *
 *   Criterion, written before this run: the smallest cruise gap agrees with `v_limit^2 / (2 * accel_limit)` within 25
 *   %, and the cruise window is non-empty (otherwise the assertion is vacuous). 本用例原本要检验的假设是 `e = v_limit /
 *   shaping_gain`，即一阶跟踪器跟斜坡的教科书跟随误差。 **实测把它否掉了** ：巡航间隙 0.496 rad，而预测 0.100；而决定性
 *   的一点是——把 shaping gain 加倍，它仍然是 0.496。一个在其所谓参数加倍时不动的数，就不是那个参数的。
 *
 *   原因是结构性的，值得写进主张而不是留在注释里：`MotionProfile` 不产生被跟随的位置参考，它用位置误差塑造 **速度** 指
 *   令，所以这里没有斜坡、也就没有斜坡跟随误差。它真正给出界的是巡航在哪里结束：剩余距离降到 `v^2 / (2a)` 时开始减速，
 *   于是巡航间隙以下限的形式被那个阈值夹住，停留时间是两个限值的后果，不是增益的。
 *
 *   判据（写在本次测量之前）：最小巡航间隙与 `v_limit^2 / (2 * accel_limit)` 相差 ≤25 %，且巡航窗口非空（否则断言是空
 *   的）。
 */
TEST(SilMotionProfile, TheCruiseEndsAtTheDecelerationThresholdNotAtTheShapingGain) {
    constexpr float kShapingGain = 20.0F;
    const auto cruise = [](float shaping_gain) -> std::pair<float, float> {
        using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                         foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                         foc::core::MotionProfile>;
        const foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
        foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
        foc::sil::PlantHal hal(plant, kVBus);
        const typename MotorT::Policies policies{
            foc::core::BackEmfObserver({mc.rs, mc.ld}),
            foc::core::Svpwm{},
            foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
            foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
            foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
            foc::core::NullReferenceGenerator{},
            foc::core::NullFeedforward{},
            foc::core::MotionProfile{foc::core::MotionProfile::Config{shaping_gain, kVelocityLimit, kAccelLimit}}};
        MotorT motor(mc, policies, hal);
        static_cast<void>(motor.init());
        motor.set_position(kTargetRad);
        static_cast<void>(motor.start());

        float min_gap = 1.0e9F;
        float mean_gap = 0.0F;
        int gap_count = 0;
        const int steps = static_cast<int>(1.0F / kDt);
        for (int i = 0; i < steps; ++i) {
            static_cast<void>(motor.tick(kDt));
            hal.advance(kDt, kSubsteps);
            const float speed = std::fabs(motor.telemetry().speed_mech_rad_s);
            const float gap = std::fabs(kTargetRad - hal.position.angle_mech_rad());
            if (speed >= 0.95F * kVelocityLimit) { // the cruise: the velocity cap is active
                min_gap = std::min(min_gap, gap);
                mean_gap += gap;
                ++gap_count;
            }
        }
        if (gap_count == 0) {
            return {-1.0F, -1.0F};
        }
        return {min_gap, mean_gap / static_cast<float>(gap_count)};
    };

    const auto base = cruise(kShapingGain);
    const auto doubled = cruise(2.0F * kShapingGain);
    const float threshold = (kVelocityLimit * kVelocityLimit) / (2.0F * kAccelLimit);
    std::printf("\n[cruise] v %0.1f rad/s, a %0.0f rad/s^2, deceleration threshold %0.4f rad\n",
                static_cast<double>(kVelocityLimit), static_cast<double>(kAccelLimit), static_cast<double>(threshold));
    std::printf("%-22s %12s %12s\n", "shaping gain", "min gap", "mean gap");
    std::printf("%-22s %12.4f %12.4f\n", "20 rad/s", static_cast<double>(base.first),
                static_cast<double>(base.second));
    std::printf("%-22s %12.4f %12.4f\n", "40 rad/s", static_cast<double>(doubled.first),
                static_cast<double>(doubled.second));

    ASSERT_GE(base.first, 0.0F) << "the move has to reach the velocity limit at all, or the window is empty";
    EXPECT_GE(base.first, 0.8F * threshold) << "the cruise cannot end before the deceleration threshold";
    EXPECT_LE(base.first, 1.25F * threshold) << "and it does not linger past it either";
    // The refutation, kept as an assertion so it cannot quietly come back: the shaping gain does not set this number.
    // 把证伪留住：shaping gain 不决定这个数。
    EXPECT_NEAR(doubled.first, base.first, 0.15F * base.first) << "doubling the shaping gain must not move the cruise "
                                                                  "gap; when it does not, the gap is not the gain's";
}

TEST(SilMotionProfile, WithoutTheProfileTheSameMoveIsAVelocityStep) {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, foc::core::NullFeedforward, foc::core::PositionP>;
    const foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
    foc::sil::PlantHal hal(plant, kVBus);

    const typename MotorT::Policies policies{
        foc::core::BackEmfObserver({mc.rs, mc.ld}),
        foc::core::Svpwm{},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
        foc::core::NullReferenceGenerator{},
        foc::core::NullFeedforward{},
        // No acceleration limit: a bare P loop, which is the point of this control.
        // 不给加速度限制：纯 P 环，这才是这条对照的意义。
        foc::core::PositionP{foc::position_p_config(mc, 20.0F, kVelocityLimit, 0.0F)}};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_position(kTargetRad);
    static_cast<void>(motor.start());

    float worst_accel = 0.0F;
    float previous_speed = 0.0F;
    const int steps = static_cast<int>(1.0F / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        const float speed = motor.telemetry().speed_mech_rad_s;
        worst_accel = std::max(worst_accel, std::fabs(speed - previous_speed) / kDt);
        previous_speed = speed;
    }

    EXPECT_GT(worst_accel, 3.0F * kAccelLimit);
}



namespace {

struct MoveOutcome final {
    double time_to_band_s;   ///< [en] first time the gap is inside the band. [zh] 间隙首次进入带内的时间。
    /**
     * @brief [en] smallest gap while the measured speed is at the limit.
     * @brief [zh] 实测速度处于上限时的最小间隙。
     */
    double min_cruise_gap;
    double overshoot_rad;
    double final_gap_rad;
};

auto run_move(float gain, float velocity_limit, float accel_limit, float target, double seconds,
              float band_rad) -> MoveOutcome {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi,
                                     foc::sil::PlantHal, foc::core::NullReferenceGenerator,
                                     foc::core::NullFeedforward, foc::core::MotionProfile>;
    const foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs,
                                     3.0F, 20000.0F, 0.0F};
    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b,
                                                    kPolePairs});
    foc::sil::PlantHal hal(plant, kVBus);
    const typename MotorT::Policies policies{
        foc::core::BackEmfObserver({mc.rs, mc.ld}),
        foc::core::Svpwm{},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
        foc::core::NullReferenceGenerator{},
        foc::core::NullFeedforward{},
        foc::core::MotionProfile{foc::core::MotionProfile::Config{gain, velocity_limit,
                                                                  accel_limit}}};
    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_position(target);
    static_cast<void>(motor.start());

    MoveOutcome outcome{-1.0, 1.0e9, -1.0e9, 0.0};
    double time = 0.0;
    const int steps = static_cast<int>(seconds / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        time += static_cast<double>(kDt);
        const float speed = motor.telemetry().speed_mech_rad_s;
        const float position = hal.position.angle_mech_rad();
        const double gap = std::fabs(static_cast<double>(target) -
                                     static_cast<double>(position));
        outcome.overshoot_rad = std::max(outcome.overshoot_rad,
                                         static_cast<double>(position) -
                                             static_cast<double>(target));
        if (std::fabs(speed) >= 0.95F * velocity_limit) {
            outcome.min_cruise_gap = std::min(outcome.min_cruise_gap, gap);
        }
        if (outcome.time_to_band_s < 0.0 && gap < static_cast<double>(band_rad)) {
            outcome.time_to_band_s = time;
        }
    }
    outcome.final_gap_rad = std::fabs(static_cast<double>(hal.position.angle_mech_rad()) -
                                      static_cast<double>(target));
    return outcome;
}

} // namespace



/**
 * @brief [en] The move takes exactly the time the trapezoid's own kinematics give it, so the header's "not
 *             time-optimal" caveat is narrower than it reads.
 *
 * @brief [zh] 这次运动花的时间恰好等于梯形规划自己运动学给出的那个数，所以头文件那句"不是时间最优"的说明比字面上要窄。
 *
 *   The header says the profile "is not time-optimal against a moving target". That caveat is about retargeting. For a
 *   rest-to-rest move whose limits are all reachable, the phase sequence is ramp / cruise / constant-deceleration, and
 *   the *move's* kinematics -- nothing to do with this class -- put the total at `D/v + v/a`. The prediction used here
 *   is the same expression minus the tail still to be covered at the moment the gap crosses the band, `sqrt(2*eps/a)`,
 *   because the crossing is what is observable, not the landing.
 *
 *   Two independent things are being checked. First, the number: the profile never computes a total time, so if any
 *   coefficient inside it were wrong the sum would move. Second, the *structure*: the cruise runs at exactly `v`, so
 *   moving the target 3 rad further must cost exactly `3/v` seconds. That difference is measured and is a much sharper
 *   test than either absolute time -- it cancels the ramp and the brake entirely.
 *
 *   头文件说规划器"对移动目标不是时间最优"。那句说明是关于中途改目标的。对于限制都能达到的点到点运动，相位序列是加速/
 *   匀速/恒定减速，而 **这次运动** 的运动学（与这个类无关）给出总时间 `D/v + v/a`。这里用的预测是同一个式子减去间隙穿过
 *   带子那一刻尚未走完的尾巴 `sqrt(2*eps/a)`，因为可观测的是穿越而不是落地。
 *
 *   检查的是两件独立的事。第一是数：规划器从不计算总时间，所以其中任何系数错了这个和都会动。第二是 **结构** ：巡航严格
 *   以 `v` 走，所以把目标再挪 3 rad 必须恰好多花 `3/v` 秒。这个差值是量出来的，而且比任何一个绝对时间都锋利——它把加速段
 *   和制动段整个约掉了。
 */
TEST(SilMotionProfile, TheMoveTakesTheTimeTheTrapezoidsKinematicsGiveIt) {
    constexpr float kBand = 0.010F;
    const float nominal_gain = 20.0F;
    const float velocity_limit = kVelocityLimit;
    const float accel_limit = kAccelLimit;

    const struct {
        float distance;
        float velocity_limit;
        float accel;
    } moves[] = {{1.0F, velocity_limit, accel_limit},
                 {4.0F, velocity_limit, accel_limit},
                 {1.0F, 1.0F, accel_limit}};

    std::printf("\n%-9s %-8s %-8s %12s %12s %8s\n", "D", "v", "a", "predicted", "measured",
                "ratio");
    double predicted_by_move[3] = {0.0, 0.0, 0.0};
    double measured_by_move[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; ++i) {
        const auto& move = moves[i];
        const double ramp_and_cruise =
            static_cast<double>(move.distance) / move.velocity_limit +
            static_cast<double>(move.velocity_limit) / move.accel;
        const double tail = std::sqrt(2.0 * static_cast<double>(kBand) / move.accel);
        const double predicted = ramp_and_cruise - tail;
        const MoveOutcome outcome =
            run_move(nominal_gain, move.velocity_limit, move.accel, move.distance,
                     ramp_and_cruise + 1.0, kBand);
        ASSERT_GE(outcome.time_to_band_s, 0.0) << "the move has to reach the band";
        predicted_by_move[i] = predicted;
        measured_by_move[i] = outcome.time_to_band_s;
        std::printf("%-9.1f %-8.1f %-8.1f %12.4f %12.4f %8.4f\n",
                    static_cast<double>(move.distance),
                    static_cast<double>(move.velocity_limit),
                    static_cast<double>(move.accel), predicted, outcome.time_to_band_s,
                    outcome.time_to_band_s / predicted);

        EXPECT_NEAR(predicted, outcome.time_to_band_s, 0.02 * predicted);
        // Teeth: the reading that forgets the braking phase is `D/v`, and it is outside the band
        // accepted above for every move here.
        // 牙齿：忘掉制动段的读法是 `D/v`，对这里每一次运动它都落在上面接受的范围之外。
        const double no_brake = static_cast<double>(move.distance) / move.velocity_limit;
        EXPECT_GT(std::fabs(no_brake - outcome.time_to_band_s) / outcome.time_to_band_s, 0.02)
            << "the braking phase has to show up in the total time";
    }

    // The structural check: the cruise really is at `v`, so 3 more radians cost exactly 3/v.
    const double extra = measured_by_move[1] - measured_by_move[0];
    const double extra_predicted = 3.0 / static_cast<double>(velocity_limit);
    std::printf("[cruise] 3 rad further costs %.4f s, and 3/v is %.4f s\n", extra,
                extra_predicted);
    EXPECT_NEAR(extra_predicted, extra, 2.0e-3);
}



/**
 * @brief [en] The axis travels a catch-up distance the profile does not budget for, and one recovered lag time predicts
 *             it on two configurations it was not recovered from.
 *
 * @brief [zh] 轴会多走一段规划器没有预算的追赶距离，而一个从某组配置反解出的滞后时间能预测另外两组。
 *
 *   The profile's promise is a stopping distance of `v^2/(2a)`, and it keeps that promise about its *command*. The axis
 *   does not follow the command exactly, so the gap at which the measured speed has fallen by 5 % sits below that
 *   threshold. The deficit has two parts, and only one of them is new: the command itself takes `0.05*v/a` to fall 5 %
 *   (kinematics, already known), and the measurement chain adds a lag `tau`, so the axis covers `v*(0.05*v/a + tau)`
 *   beyond the threshold.
 *
 *   `tau` is recovered from one configuration and then used to predict the other two, which is the only honest way to
 *   fit a number: if you recover it from all of them, you have not measured anything. It comes out at a few
 *   milliseconds and is *not* derived from anything here -- its candidates are the windowed speed estimate and the two
 *   inner loops, and this case does not separate them. What it does establish is the shape: the deficit vanishes with
 *   `v^2/a` and the lag term scales with `v` alone, which is why halving the acceleration almost doubles it.
 *
 *   规划器承诺的制动距离是 `v^2/(2a)`，而这个承诺是关于它的 **指令** 的。轴并不精确跟随指令，所以实测速度掉了 5 % 时的
 *   间隙落在那个阈值之下。缺口有两部分，只有一部分是新的：指令自己掉 5 % 需要 `0.05*v/a`（运动学，已知），量测链再加一
 *   个滞后 `tau`，于是轴比阈值多走 `v*(0.05*v/a + tau)`。
 *
 *   `tau` 从一组配置反解出来，然后用来预测另外两组——这是拟合一个数唯一诚实的用法：如果从所有配置里反解，那就什么都没量
 *   到。它是几毫秒，且 **不是** 从这里的任何东西推导出来的：它的候选是加窗转速估计与两个内层环，本用例不区分它们。它确
 *   立的是形状：缺口随 `v^2/a` 消失，滞后项只随 `v` 线性增长，这就是为什么加速度减半几乎让它翻倍。
 */
TEST(SilMotionProfile, TheAxisAddsACatchUpDistanceTheProfileDoesNotBudgetFor) {
    constexpr float kGain = 20.0F;
    constexpr float kBand = 0.010F;

    const struct {
        float velocity_limit;
        float accel;
    } configs[] = {{2.0F, 10.0F}, {2.0F, 20.0F}, {1.0F, 10.0F}};

    double deficit[3] = {0.0, 0.0, 0.0};
    double threshold[3] = {0.0, 0.0, 0.0};
    for (int i = 0; i < 3; ++i) {
        const auto& config = configs[i];
        threshold[i] = static_cast<double>(config.velocity_limit * config.velocity_limit) /
                       (2.0 * static_cast<double>(config.accel));
        const MoveOutcome outcome = run_move(kGain, config.velocity_limit, config.accel,
                                             1.0F, 3.0, kBand);
        ASSERT_GT(outcome.min_cruise_gap, 0.0) << "the cruise window has to be non-empty";
        deficit[i] = threshold[i] - outcome.min_cruise_gap;
    }

    // Kinematic part, known before anything is measured: how far the command travels while its own
    // speed falls by the 5 % that the cruise criterion asks for.
    // 运动学部分，在量任何东西之前就知道：巡航判据要求速度掉 5 %，指令在这段时间里走过多远。
    const auto kinematic = [](const double v, const double a) { return 0.05 * v * v / a; };

    const double tau = (deficit[0] - kinematic(configs[0].velocity_limit, configs[0].accel)) /
                       static_cast<double>(configs[0].velocity_limit);

    std::printf("\n%-8s %-8s %12s %12s %12s %12s %10s\n", "v", "a", "threshold", "deficit",
                "kinematic", "predicted", "tau_ms");
    for (int i = 0; i < 3; ++i) {
        const double v = static_cast<double>(configs[i].velocity_limit);
        const double a = static_cast<double>(configs[i].accel);
        const double predicted = kinematic(v, a) + v * tau;
        std::printf("%-8.1f %-8.1f %12.5f %12.5f %12.5f %12.5f %10.3f\n", v, a, threshold[i],
                    deficit[i], kinematic(v, a), predicted, 1.0e3 * tau);
        // The first row is the recovery, the other two are held-out predictions.
        // 第一行是反解，另外两行是留出来的预测。
        EXPECT_NEAR(predicted, deficit[i], 0.05 * deficit[i])
            << "row " << i << ": the deficit is a kinematic part plus a lag that scales with v";
    }

    // And the boundary this case exists to draw: the deficit is not small, so the threshold does not
    // describe the axis.
    // 以及这条用例要划的那条边界：缺口不小，所以那个阈值并不描述轴。
    std::printf("[boundary] the axis passes the deceleration threshold at %.1f %% of it: the "
                "promise is about the command\n",
                100.0 * (threshold[0] - deficit[0]) / threshold[0]);
    EXPECT_GT(deficit[0], 0.10 * threshold[0])
        << "if the axis really stopped at v^2/(2a) this row would not need a boundary clause";
}
