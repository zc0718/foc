// Unit tests for the accel-limited position profile: the invariants a planner must not break.
// 加速度受限位置规划器的单元测试：规划器不得破坏的那几条不变式。

#include <algorithm>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <foc/core/motion_profile.hpp>



namespace {

constexpr float kDt = 1e-4F;

auto make_profile(float gain, float velocity_limit, float accel_limit) -> foc::core::MotionProfile {
    return foc::core::MotionProfile{foc::core::MotionProfile::Config{gain, velocity_limit, accel_limit}};
}

} // namespace



TEST(MotionProfile, VelocityNeverExceedsItsLimit) {
    foc::core::MotionProfile profile = make_profile(100.0F, 2.0F, 10.0F);

    float worst = 0.0F;
    for (int i = 0; i < 20'000; ++i) {
        const float velocity = profile.step(100.0F /* far away */, 0.0F, kDt);
        worst = std::max(worst, std::fabs(velocity));
    }

    EXPECT_FLOAT_EQ(2.0F, worst); // the limit binds within a few ms and then holds
}



TEST(MotionProfile, TheCommandChangesNoFasterThanTheAccelerationLimit) {
    foc::core::MotionProfile profile = make_profile(100.0F, 2.0F, 10.0F);
    const float max_change = 10.0F * kDt;

    float previous = 0.0F;
    for (int i = 0; i < 5'000; ++i) {
        // Alternate the target so the command is pushed in both directions as hard as possible.
        const float target = ((i / 500) % 2 == 0) ? 100.0F : -100.0F;
        const float velocity = profile.step(target, 0.0F, kDt);
        EXPECT_LE(std::fabs(velocity - previous), max_change + 1e-6F);
        previous = velocity;
    }
}



TEST(MotionProfile, OnTargetTheCommandDecaysToZero) {
    foc::core::MotionProfile profile = make_profile(20.0F, 5.0F, 50.0F);

    float velocity = 1.0F;
    for (int i = 0; i < 5'000; ++i) {
        velocity = profile.step(1.0F, 1.0F, kDt); // error is zero: the demand is zero
    }

    EXPECT_NEAR(0.0F, velocity, 1e-6F);
}



TEST(MotionProfile, ResetPutsTheCommandBackToRest) {
    foc::core::MotionProfile profile = make_profile(20.0F, 5.0F, 50.0F);
    static_cast<void>(profile.step(10.0F, 0.0F, kDt));
    ASSERT_GT(profile.velocity_command_rad_s(), 0.0F);

    profile.reset();
    EXPECT_FLOAT_EQ(0.0F, profile.velocity_command_rad_s());
}



TEST(MotionProfile, JunkInDoesNotBecomeGarbageOut) {
    foc::core::MotionProfile profile = make_profile(20.0F, 5.0F, 50.0F);
    static_cast<void>(profile.step(0.1F, 0.0F, kDt));
    const float held = profile.velocity_command_rad_s();

    // A non-finite setpoint, a non-finite feedback and a non-positive dt all hold the last command.
    EXPECT_FLOAT_EQ(held, profile.step(std::numeric_limits<float>::quiet_NaN(), 0.0F, kDt));
    EXPECT_FLOAT_EQ(held, profile.step(0.0F, std::numeric_limits<float>::infinity(), kDt));
    EXPECT_FLOAT_EQ(held, profile.step(0.0F, 0.0F, 0.0F));
}



TEST(MotionProfile, AZeroedConfigurationIsInert) {
    foc::core::MotionProfile profile; // Config{}: no gain, no limits

    for (int i = 0; i < 100; ++i) {
        EXPECT_FLOAT_EQ(0.0F, profile.step(10.0F, 0.0F, kDt));
    }
}



TEST(MotionProfile, APerMoveLimitReplacesTheConfiguredOne) {
    // The plan is configured for a fast joint; this one move is a slow approach. A non-zero per-move
    // limit replaces the configured value for that call only, so the next move gets its own.
    // 规划器按快关节配置；而这一次运动是慢速接近。非零的本次限制只在那一次调用内替换配置值，
    // 于是下一次运动可以有自己的限制。
    foc::core::MotionProfile profile = make_profile(100.0F, 2.0F, 10.0F);
    const foc::core::PositionInput slow{100.0F /* far away */, 0.1F /* rad/s */, 0.0F, 0.0F};

    float worst = 0.0F;
    for (int i = 0; i < 2'000; ++i) {
        const float velocity = profile.step(slow, 0.0F, kDt);
        worst = std::max(worst, std::fabs(velocity));
    }
    EXPECT_NEAR(0.1F, worst, 1e-6F);

    // The configured limit is untouched: the next move is fast again.
    // 配置里的限制没有被改动：下一次运动立刻恢复快速。
    const foc::core::PositionInput fast{100.0F, 0.0F, 0.0F, 0.0F};
    float next = 0.0F;
    for (int i = 0; i < 2'000; ++i) {
        next = profile.step(fast, 0.0F, kDt);
    }
    EXPECT_NEAR(2.0F, next, 1e-6F);
}



TEST(MotionProfile, AZeroPerMoveLimitMeansUseTheConfiguredOne) {
    // 0 is only ever a default where it is not a legal limit; this is one of those places.
    // 0 只在它本身不构成合法限值的地方才表示默认值；这里正是这种地方。
    foc::core::MotionProfile profile = make_profile(100.0F, 2.0F, 10.0F);
    const foc::core::PositionInput unset{100.0F, 0.0F, 0.0F, 0.0F};

    float worst = 0.0F;
    for (int i = 0; i < 2'000; ++i) {
        worst = std::max(worst, std::fabs(profile.step(unset, 0.0F, kDt)));
    }
    EXPECT_NEAR(2.0F, worst, 1e-6F);
}



TEST(MotionProfile, ANegativePerMoveLimitIsIgnoredRatherThanReversed) {
    // A negative limit is the caller's bug. Reading it as "go the other way" would be worse than
    // ignoring it, and reading it as an absolute value would hide the bug.
    // 负的限制是调用方的 bug。当成"反向运动"比忽略更糟，而当成绝对值会掩盖这个 bug。
    foc::core::MotionProfile profile = make_profile(100.0F, 2.0F, 10.0F);
    const foc::core::PositionInput negative{100.0F, -1.0F, -1.0F, 0.0F};

    float worst = 0.0F;
    for (int i = 0; i < 2'000; ++i) {
        worst = std::max(worst, std::fabs(profile.step(negative, 0.0F, kDt)));
    }
    EXPECT_NEAR(2.0F, worst, 1e-6F); // the configured limits applied instead
}



TEST(MotionProfile, ATighterPerMoveAccelLimitMakesTheCommandRampSlower) {
    foc::core::MotionProfile profile = make_profile(100.0F, 5.0F, 1000.0F);
    const foc::core::PositionInput gentle{100.0F, 0.0F, 10.0F, 0.0F};

    // After 5 ms at 10 rad/s^2 the command can be at most 0.05 rad/s.
    // 在 10 rad/s^2 下跑 5 ms，指令最多到 0.05 rad/s。
    float velocity = 0.0F;
    for (int i = 0; i < 50; ++i) {
        velocity = profile.step(gentle, 0.0F, kDt);
    }
    EXPECT_NEAR(0.05F, velocity, 1e-5F);
}



TEST(MotionProfile, TheStiffnessScaleSlowsTheApproachWithoutChangingTheLimits) {
    // Scaling the shaping gain is how a joint is made compliant for one move: the same limits, a
    // gentler approach.
    // 缩放塑造增益就是让关节为一次运动变柔顺的办法：限制不变，接近更温和。
    // A huge acceleration limit so the rate limiter and the braking curve stay out of the way: this
    // case is about the gain, and mixing in the shaping would measure something else.
    // 加速度上限取得极大，让限速率与制动曲线都不介入：本条关心的是增益，掺进塑造就成测别的东西了。
    foc::core::MotionProfile stiff = make_profile(100.0F, 5.0F, 1.0e6F);
    foc::core::MotionProfile soft = make_profile(100.0F, 5.0F, 1.0e6F);

    const foc::core::PositionInput stiff_request{0.05F, 0.0F, 0.0F, 1.0F};
    const foc::core::PositionInput soft_request{0.05F, 0.0F, 0.0F, 0.1F};

    EXPECT_NEAR(5.0F, stiff.step(stiff_request, 0.0F, kDt), 1e-5F); // 100 * 0.05, capped at the limit
    EXPECT_NEAR(0.5F, soft.step(soft_request, 0.0F, kDt), 1e-5F);   // a tenth of the gain
}
