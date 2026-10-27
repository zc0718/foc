// Unit tests for the simple proportional position controller: it has to honour the same per-move
// limits the planner does, and it has to be honest about what it is.
// 简单比例位置控制器的单元测试：它必须和规划器一样遵守按次给出的限制，并且对自己的性质诚实。

#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/motion_profile.hpp>
#include <foc/core/position.hpp>
#include <foc/core/traits.hpp>



namespace {

constexpr float kDt = 1e-4F;

auto make_controller(float gain, float velocity_limit, float accel_limit) -> foc::core::PositionP {
    return foc::core::PositionP{foc::core::PositionP::Config{gain, velocity_limit, accel_limit}};
}

} // namespace



TEST(PositionP, OnTargetItCommandsNothing) {
    foc::core::PositionP controller = make_controller(10.0F, 5.0F, 0.0F);
    EXPECT_FLOAT_EQ(0.0F, controller.step(foc::core::PositionInput{0.3F, 0.0F, 0.0F}, 0.3F, kDt));
}



TEST(PositionP, TheVelocityCommandIsClampedToItsLimit) {
    foc::core::PositionP controller = make_controller(10.0F, 5.0F, 0.0F);
    // 100 rad of error at gain 10 would ask for 1000 rad/s.
    EXPECT_FLOAT_EQ(5.0F, controller.step(foc::core::PositionInput{100.0F, 0.0F, 0.0F}, 0.0F, kDt));
    EXPECT_FLOAT_EQ(-5.0F, controller.step(foc::core::PositionInput{-100.0F, 0.0F, 0.0F}, 0.0F, kDt));
}



TEST(PositionP, APerMoveVelocityLimitReplacesTheConfiguredOne) {
    foc::core::PositionP controller = make_controller(10.0F, 5.0F, 0.0F);
    EXPECT_FLOAT_EQ(0.2F, controller.step(foc::core::PositionInput{100.0F, 0.2F, 0.0F}, 0.0F, kDt));
    // and the configuration is untouched for the next move
    // 且配置对下一次运动没有变化
    EXPECT_FLOAT_EQ(5.0F, controller.step(foc::core::PositionInput{100.0F, 0.0F, 0.0F}, 0.0F, kDt));
}



TEST(PositionP, APerMoveAccelLimitRateLimitsTheCommand) {
    foc::core::PositionP controller = make_controller(1000.0F, 50.0F, 0.0F);
    const foc::core::PositionInput request{100.0F, 0.0F, 100.0F};

    // One step at 100 rad/s^2 can only reach 0.01 rad/s, far below the 50 rad/s the P term wants.
    // 在 100 rad/s^2 下走一步只能到 0.01 rad/s，远低于比例项要的 50 rad/s。
    EXPECT_NEAR(0.01F, controller.step(request, 0.0F, kDt), 1e-6F);
    EXPECT_NEAR(0.02F, controller.step(request, 0.0F, kDt), 1e-6F);
}



TEST(PositionP, WithoutAnAccelLimitTheCommandFollowsTheProportionalTerm) {
    // `accel_limit_rad_s2 == 0` in the *configuration* is a legal choice ("no rate limit"), not a
    // fallback: a bare P loop is exactly what some position loops want.
    // 配置里的 `accel_limit_rad_s2 == 0` 是合法选择（"不限速率"），不是回退：有些位置环要的就是纯 P。
    foc::core::PositionP controller = make_controller(10.0F, 5.0F, 0.0F);
    EXPECT_FLOAT_EQ(0.5F, controller.step(foc::core::PositionInput{0.05F, 0.0F, 0.0F}, 0.0F, kDt));
}



TEST(PositionP, AnUnusableStepHoldsThePreviousCommand) {
    foc::core::PositionP controller = make_controller(10.0F, 5.0F, 0.0F);
    static_cast<void>(controller.step(foc::core::PositionInput{0.05F, 0.0F, 0.0F}, 0.0F, kDt));

    // A non-positive step must not be read as "infinitely fast": hold, do not jump.
    // 非正步长不能被理解成"无限快"：保持，不要跳变。
    EXPECT_FLOAT_EQ(0.5F, controller.step(foc::core::PositionInput{0.5F, 0.0F, 0.0F}, 0.0F, 0.0F));
    EXPECT_FLOAT_EQ(0.5F, controller.step(foc::core::PositionInput{0.5F, 0.0F, 0.0F}, 0.0F, -kDt));
}



TEST(PositionP, ResetRestartsTheCommandFromRestNotFromWhereItWas) {
    // The rate limiter has memory; `reset()` is what clears it, and the evidence is that the next
    // step starts from zero again instead of continuing from the previous command.
    // 限速率器有记忆；`reset()` 就是清掉它的动作，证据是下一步又从零开始，而不是接着上一条指令走。
    foc::core::PositionP controller = make_controller(1000.0F, 50.0F, 100.0F);
    const foc::core::PositionInput request{100.0F, 0.0F, 100.0F};

    for (int i = 0; i < 50; ++i) {
        static_cast<void>(controller.step(request, 0.0F, kDt)); // ~0.5 rad/s by now
    }
    EXPECT_GT(controller.step(request, 0.0F, kDt), 0.4F);

    controller.reset();
    EXPECT_NEAR(0.01F, controller.step(request, 0.0F, kDt), 1e-6F); // one acceleration step
    // The configuration is untouched by a reset: it is configuration, not progress.
    // 复位不改变配置：配置是配置，进度是进度。
    EXPECT_FLOAT_EQ(1000.0F, controller.config().gain_1_s);
}



TEST(PositionP, EveryShippedPositionControllerSatisfiesTheWidenedSlot) {
    // The slot takes `step(const PositionInput&, feedback, dt)`. The three-argument controller shape
    // is *not* enough, and that is deliberate: a move carries limits, not only an endpoint.
    // 槽位接收 `step(const PositionInput&, feedback, dt)`。三参数的控制器形状**不够**，这是故意的：
    // 一次运动带着限制，不只有终点。
    static_assert(foc::core::concepts::is_position_control_v<foc::core::PositionP>,
                  "PositionP must satisfy the position slot");
    static_assert(foc::core::concepts::is_position_control_v<foc::core::NullPositionControl>,
                  "NullPositionControl must satisfy the position slot");
    static_assert(foc::core::concepts::is_position_control_v<foc::core::MotionProfile>,
                  "MotionProfile must satisfy the position slot");
    static_assert(!foc::core::concepts::is_controller_v<foc::core::PositionP>,
                  "PositionP must not claim the plain controller shape");
    SUCCEED();
}



TEST(PositionP, TheStiffnessScaleMultipliesTheGainNotTheLimits) {
    // Compliance is a gain, so the joint still cannot exceed its limits: a compliant joint is soft,
    // not unconstrained.
    // 柔顺是增益，因此关节仍不能越限：柔顺的关节是软的，不是无约束的。
    foc::core::PositionP controller = make_controller(10.0F, 5.0F, 0.0F);

    EXPECT_FLOAT_EQ(1.0F, controller.step(foc::core::PositionInput{0.1F, 0.0F, 0.0F, 1.0F}, 0.0F, kDt));
    EXPECT_FLOAT_EQ(0.2F, controller.step(foc::core::PositionInput{0.1F, 0.0F, 0.0F, 0.2F}, 0.0F, kDt));
    // 0 = configured stiffness
    // 0 = 用配置的刚度
    EXPECT_FLOAT_EQ(1.0F, controller.step(foc::core::PositionInput{0.1F, 0.0F, 0.0F, 0.0F}, 0.0F, kDt));
    // and the velocity limit still binds however soft the gain is
    // 而且无论增益多软，速度上限照样生效
    EXPECT_FLOAT_EQ(5.0F, controller.step(foc::core::PositionInput{100.0F, 0.0F, 0.0F, 0.01F}, 0.0F, kDt));
}
