// Unit tests for the shaft-lock alignment routine: the properties that decide whether the captured
// offset can be trusted.
// 锁轴对齐流程的单元测试：决定捕获到的偏置是否可信的那几条性质。

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <foc/core/alignment.hpp>
#include <foc/core/traits.hpp>



namespace {

constexpr float kDt = 1e-4F;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 6.28318530717958647692F;

auto make_config() -> foc::core::AlignConfig {
    // current, ramp, hold, tolerance, verify, forced angle
    return foc::core::AlignConfig{1.0F, 0.01F, 0.05F, 0.01F, 0.02F, 0.0F};
}

/// Drive a routine to completion against a scripted sensor, returning the last output.
struct ScriptedRun final {
    foc::core::AlignOutput last{};
    foc::core::AlignState state{foc::core::AlignState::Idle};
    float elapsed_s{0.0F};
    bool finished{false};
    bool failed{false};
};

/// `angle_at(t)` lets a test decide what the rotor does; the routine never sees time itself.
template <typename AngleAt>
auto run_scripted(foc::core::AlignRoutine& routine, float limit_s, AngleAt angle_at) -> ScriptedRun {
    ScriptedRun run{};
    float t = 0.0F;
    while (t < limit_s) {
        run.last = routine.step(kDt, angle_at(t));
        t += kDt;
        if (run.last.finished || run.last.failed) {
            break;
        }
    }
    run.state = routine.state();
    run.elapsed_s = t;
    run.finished = run.last.finished;
    run.failed = run.last.failed;
    return run;
}

/// Wrap into [0, 2*pi), for comparing angles that live on a circle.
auto wrap_two_pi(float angle_rad) -> float {
    float wrapped = std::fmod(angle_rad, kTwoPi);
    if (wrapped < 0.0F) {
        wrapped += kTwoPi;
    }
    return wrapped;
}

/// Shortest signed distance between two angles, rad.
auto angular_difference(float a, float b) -> float {
    const float d = wrap_two_pi(a - b);
    return (d > kPi) ? (d - kTwoPi) : d;
}



/**
 * @brief [en] Rotor that follows the field with a first-order lag, driven by the routine's phase.
 *        [zh] 以流程阶段驱动、以一阶滞后跟随磁场的转子。
 *
 * @note [en] Phase-driven rather than time-driven on purpose: a test must not break when a phase
 *       boundary moves by one control period. `follow_sign` of -1 models an inverted encoder, which
 *       is the failure worth detecting.
 *       [zh] 故意用阶段而非时间驱动：阶段边界移动一个控制周期不应让测试失效。`follow_sign` 取 -1
 *       即模拟编码器符号接反，那正是值得检出的失败。
 */
class ScriptedRotor final {
  public:
    ScriptedRotor(const foc::core::AlignRoutine& routine, float start_rad, float pole_pairs, float rate_per_s,
                  float follow_sign)
        : routine_(&routine), start_rad_(start_rad), rate_per_s_(rate_per_s), follow_sign_(follow_sign),
          commanded_rad_((kPi / 2.0F) / pole_pairs), angle_rad_(start_rad) {}

    auto operator()(float /*t_s*/) -> float {
        const bool verifying = (routine_->state() == foc::core::AlignState::Verify);
        const float target = verifying ? (start_rad_ + (follow_sign_ * commanded_rad_)) : start_rad_;
        const float alpha = (rate_per_s_ * kDt < 1.0F) ? (rate_per_s_ * kDt) : 1.0F;
        angle_rad_ += ((target - angle_rad_) * alpha);
        return angle_rad_;
    }

  private:
    const foc::core::AlignRoutine* routine_;
    float start_rad_;
    float rate_per_s_;
    float follow_sign_;
    float commanded_rad_;
    float angle_rad_;
};

} // namespace



TEST(Alignment, AnUnconfiguredAlignerIsInertRatherThanBroken) {
    // A compiled-in aligner that nobody configured must not stop a motor from starting: `NotImpl`
    // means "alignment was not requested", which keeps the feature inert instead of failing the
    // machine. A *partially* configured recipe is still an error -- see the next test.
    // 编进来但没人配置的对齐器不得阻止电机启动：`NotImpl` 表示“没有请求对齐”，让特性保持静默而不是
    // 让机器失败。**半配置**的配方仍然是错误——见下一个用例。
    foc::core::AlignRoutine routine;
    EXPECT_EQ(foc::ErrorCode::NotImpl, routine.validate());
    EXPECT_FALSE(routine.begin(7U));
    EXPECT_EQ(foc::core::AlignState::Idle, routine.state()) << "nothing was requested, so nothing failed";
    EXPECT_EQ(foc::core::AlignFailure::None, routine.failure());
}



TEST(Alignment, APartiallyConfiguredRecipeIsRejected) {
    foc::core::AlignConfig partial{};
    partial.current_a = 1.0F; // everything else left at zero
    foc::core::AlignRoutine routine{partial};
    EXPECT_EQ(foc::ErrorCode::Param, routine.validate());
    EXPECT_FALSE(routine.begin(7U));
    EXPECT_EQ(foc::core::AlignFailure::NotConfigured, routine.failure());
}



TEST(Alignment, EveryIllegalFieldIsNamedByTheSameRejection) {
    struct Case final {
        foc::core::AlignConfig config;
        const char* what;
    } const cases[] = {
        {{0.0F, 0.0F, 0.05F, 0.01F, 0.0F, 0.0F}, "zero current"},
        {{1.0F, -0.1F, 0.05F, 0.01F, 0.0F, 0.0F}, "negative ramp"},
        {{1.0F, 0.0F, 0.0F, 0.01F, 0.0F, 0.0F}, "zero hold"},
        {{1.0F, 0.0F, 0.05F, 0.0F, 0.0F, 0.0F}, "zero tolerance"},
        {{1.0F, 0.0F, 0.05F, 0.01F, -1.0F, 0.0F}, "negative verify"},
        {{1.0F, 0.0F, 0.05F, 0.01F, 0.0F, std::numeric_limits<float>::infinity()}, "non-finite forced angle"},
    };
    for (const Case& c : cases) {
        const foc::core::AlignRoutine routine{c.config};
        EXPECT_EQ(foc::ErrorCode::Param, routine.validate()) << c.what;
    }
    // The control case: a complete recipe validates.
    // 对照组：完整配方通过校验。
    const foc::core::AlignRoutine ok{make_config()};
    EXPECT_EQ(foc::ErrorCode::Ok, ok.validate());
}



TEST(Alignment, ZeroRampIsLegalAndStartsTheHoldImmediately) {
    foc::core::AlignConfig config = make_config();
    config.ramp_s = 0.0F;
    foc::core::AlignRoutine routine{config};
    ASSERT_TRUE(routine.begin(1U));
    EXPECT_EQ(foc::core::AlignState::Settle, routine.state());

    const foc::core::AlignOutput first = routine.step(kDt, 0.0F);
    EXPECT_FLOAT_EQ(config.current_a, first.id_a);
}



TEST(Alignment, TheRampReachesTheConfiguredCurrentAndNotAHairMore) {
    foc::core::AlignConfig config = make_config();
    config.ramp_s = 0.01F;
    foc::core::AlignRoutine routine{config};
    ASSERT_TRUE(routine.begin(1U));

    float previous = -1.0F;
    for (int i = 0; i < 200; ++i) {
        const foc::core::AlignOutput out = routine.step(kDt, 0.0F);
        EXPECT_LE(out.id_a, config.current_a);
        EXPECT_GE(out.id_a, previous); // monotonic: no overshoot in the ramp
        previous = out.id_a;
    }
    EXPECT_FLOAT_EQ(config.current_a, previous);
}



TEST(Alignment, AStationaryRotorYieldsItsOwnAngleAsTheOffset) {
    // Pole pairs 1 and a forced electrical angle of 0: the captured offset must equal the angle the
    // sensor reported while the rotor was held.
    // 极对数 1、强制电角度 0：捕获到的偏置必须等于转子被拉住期间传感器报出的角度。
    foc::core::AlignConfig config = make_config();
    config.verify_s = 0.0F; // verify off so the rotor can stay put
    foc::core::AlignRoutine routine{config};
    ASSERT_TRUE(routine.begin(1U));

    const ScriptedRun run = run_scripted(routine, 1.0F, [](float) { return 0.30F; });
    ASSERT_TRUE(run.finished);
    EXPECT_EQ(foc::core::AlignState::Done, routine.state());
    EXPECT_NEAR(0.30F, routine.offset_mech_rad(), 1e-5F);
    EXPECT_LT(routine.spread_rad(), 1e-3F);
}



TEST(Alignment, TheOffsetScalesWithPolePairs) {
    // A forced electrical angle of pi/2 on a 4-pole-pair machine is pi/8 mechanical; the offset is
    // the reading minus that, not the reading itself.
    // 4 极对电机上 pi/2 的电角度等于 pi/8 的机械角；偏置是读数减去它，而不是读数本身。
    foc::core::AlignConfig config = make_config();
    config.verify_s = 0.0F;
    config.align_angle_e_rad = kPi / 2.0F;
    foc::core::AlignRoutine routine{config};
    ASSERT_TRUE(routine.begin(4U));

    const ScriptedRun run = run_scripted(routine, 1.0F, [](float) { return 1.0F; });
    ASSERT_TRUE(run.finished);
    EXPECT_NEAR(wrap_two_pi(1.0F - (kPi / 8.0F)), routine.offset_mech_rad(), 1e-5F);
}



TEST(Alignment, TheCaptureAveragesAcrossTheTwoPiWrap) {
    // Readings either side of 2*pi are 0.02 rad apart, not half a turn apart. An arithmetic mean
    // would answer pi here; the circular mean must not.
    // 2π 两侧的读数相距 0.02 rad，而不是半圈。算术平均在这里会给出 pi；圆统计平均不得如此。
    foc::core::AlignConfig config = make_config();
    config.verify_s = 0.0F;
    foc::core::AlignRoutine routine{config};
    ASSERT_TRUE(routine.begin(1U));

    const ScriptedRun run = run_scripted(routine, 1.0F, [](float t) {
        // Alternate between just below and just above the wrap point.
        // 在环绕点两侧交替。
        return ((static_cast<int>(t * 10'000.0F) % 2) == 0) ? (kTwoPi - 0.005F) : 0.005F;
    });
    ASSERT_TRUE(run.finished);
    EXPECT_LT(std::fabs(angular_difference(0.0F, routine.offset_mech_rad())), 0.02F);
}



TEST(Alignment, ARotorThatNeverStopsIsRejected) {
    // A mean over a swing is a number, not an angle the rotor is at: the capture must be refused.
    // 在摆动上求平均得到的是一个数字，而不是转子所在的角度：必须拒绝这次捕获。
    foc::core::AlignRoutine routine{make_config()};
    ASSERT_TRUE(routine.begin(1U));

    const ScriptedRun run = run_scripted(routine, 1.0F, [](float t) {
        return 0.5F * std::sin(40.0F * t); // keeps swinging through the whole hold
    });
    EXPECT_TRUE(run.failed);
    EXPECT_EQ(foc::core::AlignState::Failed, routine.state());
    EXPECT_EQ(foc::core::AlignFailure::NotSettled, routine.failure());
}



TEST(Alignment, TheVerifyPhaseAdvancesTheForcedAngleAQuarterTurn) {
    foc::core::AlignRoutine routine{make_config()};
    ASSERT_TRUE(routine.begin(1U));

    // Rotor stays put, so the verify phase is reached and its forced angle is observable.
    // 转子不动，因此会走到校验阶段，其强制角可被观察。
    float verify_angle = 0.0F;
    bool saw_verify = false;
    float t = 0.0F;
    while (t < 0.2F) {
        static_cast<void>(routine.step(kDt, 0.7F));
        t += kDt;
        if (routine.state() != foc::core::AlignState::Verify) {
            continue;
        }
        // The tick that *entered* the phase still commanded the alignment angle; the next one is
        // the first that actually forces the quarter turn.
        // 进入该阶段那一 tick 仍给的是对齐角；下一个 tick 才第一次真正强制四分之一圈。
        const foc::core::AlignOutput out = routine.step(kDt, 0.7F);
        t += kDt;
        verify_angle = out.theta_e_rad;
        saw_verify = true;
        break;
    }
    ASSERT_TRUE(saw_verify);
    EXPECT_NEAR(kPi / 2.0F, verify_angle, 1e-6F);
}



TEST(Alignment, ARotorThatFollowsTheWrongWayIsRejected) {
    // This is the failure an offset cannot repair: an inverted sensor sign. The field is advanced a
    // quarter turn, and the reading moves the other way.
    // 这是偏置修不好的那种失败：传感器符号接反。磁场前进四分之一圈，而读数朝相反方向走。
    foc::core::AlignRoutine routine{make_config()};
    ASSERT_TRUE(routine.begin(1U));
    ScriptedRotor rotor(routine, 0.7F, 1.0F, 400.0F, -1.0F);

    const ScriptedRun run = run_scripted(routine, 1.0F, [&rotor](float t) { return rotor(t); });
    EXPECT_TRUE(run.failed);
    EXPECT_EQ(foc::core::AlignFailure::DirectionWrong, routine.failure());
}



TEST(Alignment, ARotorThatFollowsTheRightWayIsAccepted) {
    foc::core::AlignRoutine routine{make_config()};
    ASSERT_TRUE(routine.begin(1U));
    ScriptedRotor rotor(routine, 0.7F, 1.0F, 400.0F, 1.0F);

    const ScriptedRun run = run_scripted(routine, 1.0F, [&rotor](float t) { return rotor(t); });
    ASSERT_TRUE(run.finished);
    EXPECT_EQ(foc::core::AlignState::Done, routine.state());
    EXPECT_NEAR(0.7F, routine.offset_mech_rad(), 1e-4F);
}



TEST(Alignment, ARotorThatDoesNotFollowIsRejected) {
    // A jammed or over-loaded shaft: it stays put through the verify phase. Nothing here says the
    // offset is wrong, only that it was never demonstrated -- `NotSettled`, not `DirectionWrong`.
    // 卡死或过载的轴：整个校验阶段都不动。这不能说明偏置错了，只能说明它从未被验证过——是
    // `NotSettled` 而不是 `DirectionWrong`。
    foc::core::AlignRoutine routine{make_config()};
    ASSERT_TRUE(routine.begin(1U));
    ScriptedRotor rotor(routine, 0.7F, 1.0F, 400.0F, 0.0F);

    const ScriptedRun run = run_scripted(routine, 1.0F, [&rotor](float t) { return rotor(t); });
    EXPECT_TRUE(run.failed);
    EXPECT_EQ(foc::core::AlignFailure::NotSettled, routine.failure());
}



TEST(Alignment, TheFinishingTickCommandsZeroCurrent) {
    // The hand-over tick must already be harmless: the caller may enable its closed loop on the
    // very next tick.
    // 交接那一 tick 必须已经无害：调用方可能在下一个 tick 就切进闭环。
    foc::core::AlignConfig config = make_config();
    config.verify_s = 0.0F;
    foc::core::AlignRoutine routine{config};
    ASSERT_TRUE(routine.begin(1U));

    const ScriptedRun run = run_scripted(routine, 1.0F, [](float) { return 0.0F; });
    ASSERT_TRUE(run.finished);
    EXPECT_FLOAT_EQ(0.0F, run.last.id_a);
}



TEST(Alignment, ANonPositiveStepDoesNotAdvanceTheSequence) {
    foc::core::AlignRoutine routine{make_config()};
    ASSERT_TRUE(routine.begin(1U));
    const foc::core::AlignState before = routine.state();

    static_cast<void>(routine.step(0.0F, 0.0F));
    static_cast<void>(routine.step(-1e-4F, 0.0F));
    EXPECT_EQ(before, routine.state());

    // And the sequence still completes after the bad steps.
    // 且在这些坏步长之后流程仍能完成。
    ScriptedRotor rotor(routine, 0.2F, 1.0F, 400.0F, 1.0F);
    const ScriptedRun run = run_scripted(routine, 1.0F, [&rotor](float t) { return rotor(t); });
    EXPECT_TRUE(run.finished);
}



TEST(Alignment, ResetForgetsTheResultAndTheFailure) {
    foc::core::AlignRoutine routine{make_config()};
    ASSERT_TRUE(routine.begin(1U));
    const ScriptedRun run = run_scripted(routine, 1.0F, [](float t) {
        return 0.5F * std::sin(40.0F * t);
    });
    ASSERT_TRUE(run.failed);

    routine.reset();
    EXPECT_EQ(foc::core::AlignState::Idle, routine.state());
    EXPECT_EQ(foc::core::AlignFailure::None, routine.failure());
    EXPECT_FLOAT_EQ(0.0F, routine.offset_mech_rad());
    // A recipe survives a reset: it is configuration, not progress.
    // 配方在复位后仍然保留：它是配置，不是进度。
    EXPECT_EQ(foc::ErrorCode::Ok, routine.validate());
}



TEST(Alignment, TheNullAlignerHasNothingToDo) {
    foc::core::NullAlignment none;
    EXPECT_EQ(foc::ErrorCode::Ok, none.validate());
    EXPECT_FALSE(none.begin(7U));

    const foc::core::AlignOutput out = none.step(kDt, 3.0F);
    EXPECT_FLOAT_EQ(0.0F, out.id_a);
    EXPECT_FALSE(out.finished);
    EXPECT_FALSE(out.failed);
}



TEST(Alignment, BothAlignersSatisfyTheSlotContract) {
    static_assert(foc::core::concepts::is_aligner_v<foc::core::AlignRoutine>, "AlignRoutine must fit the slot");
    static_assert(foc::core::concepts::is_aligner_v<foc::core::NullAlignment>, "NullAlignment must fit the slot");
    static_assert(foc::core::concepts::has_align_offset_v<foc::core::AlignRoutine>,
                  "AlignRoutine must report its capture");
    static_assert(!foc::core::concepts::has_align_offset_v<foc::core::NullAlignment>,
                  "NullAlignment has no capture to report");
    SUCCEED();
}



TEST(Alignment, TheRecipeCanBeReplacedBeforeTheSequenceStarts) {
    // An unconfigured instance is legal to hold; the recipe arrives through set_config() and is
    // readable back, so a caller can log what it actually ran with.
    // 持有未配置的实例是合法的；配方通过 set_config() 送入，并可读回，于是调用方能记录它真正用的什么。
    foc::core::AlignRoutine routine;
    EXPECT_EQ(foc::ErrorCode::NotImpl, routine.validate());

    routine.set_config(make_config());
    EXPECT_EQ(foc::ErrorCode::Ok, routine.validate());
    EXPECT_FLOAT_EQ(make_config().current_a, routine.config().current_a);
    EXPECT_FLOAT_EQ(make_config().hold_s, routine.config().hold_s);
}



TEST(Alignment, SteppingAfterTheSequenceIsOverDoesNothing) {
    // Idempotent end state: a caller that keeps ticking (a lazy hand-over) must not see a second
    // "finished" edge, because two edges would mean two offset writes.
    // 结束态幂等：落后一拍交接的调用方不得看到第二次 finished，因为两次边沿等于写了两次偏置。
    foc::core::AlignConfig config = make_config();
    config.verify_s = 0.0F;
    foc::core::AlignRoutine routine{config};
    ASSERT_TRUE(routine.begin(1U));

    const ScriptedRun run = run_scripted(routine, 1.0F, [](float) { return 0.25F; });
    ASSERT_TRUE(run.finished);
    const float captured = routine.offset_mech_rad();

    for (int i = 0; i < 100; ++i) {
        const foc::core::AlignOutput out = routine.step(kDt, 3.0F);
        EXPECT_FALSE(out.finished);
        EXPECT_FALSE(out.failed);
        EXPECT_FLOAT_EQ(0.0F, out.id_a);
    }
    EXPECT_EQ(foc::core::AlignState::Done, routine.state());
    EXPECT_FLOAT_EQ(captured, routine.offset_mech_rad());
}
