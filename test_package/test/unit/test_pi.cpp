#include <gtest/gtest.h>

#include <foc/core/pi.hpp>
#include <foc/core/types.hpp>



namespace {

auto make_config(float kp, float ki) -> foc::core::Pi::Config {
    return foc::core::Pi::Config{kp, ki, -100.0F, 100.0F, -50.0F, 50.0F};
}

} // namespace



TEST(Pi, ProportionalOnlyPassesGainThrough) {
    foc::core::Pi pi(make_config(2.0F, 0.0F));

    EXPECT_FLOAT_EQ(2.0F, pi.step(1.0F, 0.0F, 0.001F));
    EXPECT_EQ(foc::ErrorCode::Ok, pi.status());
}



TEST(Pi, IntegralAccumulatesKiTimesErrorTimesDt) {
    foc::core::Pi pi(make_config(0.0F, 1.0F));

    EXPECT_FLOAT_EQ(0.1F, pi.step(1.0F, 0.0F, 0.1F));
    EXPECT_FLOAT_EQ(0.2F, pi.step(1.0F, 0.0F, 0.1F));
    EXPECT_FLOAT_EQ(0.2F, pi.integral());
}



TEST(Pi, OutputIsClamped) {
    foc::core::Pi pi(make_config(1000.0F, 0.0F));

    EXPECT_FLOAT_EQ(100.0F, pi.step(1.0F, 0.0F, 0.001F));
    EXPECT_FLOAT_EQ(100.0F, pi.output());
}



TEST(Pi, IntegralIsClampedToPreventWindup) {
    foc::core::Pi pi(make_config(0.0F, 1000.0F));

    // ki * error * dt = 1.0 per step, so 50 steps reach the integral clamp and stay there.
    for (int i = 0; i < 100; ++i) {
        static_cast<void>(pi.step(1.0F, 0.0F, 0.001F));
    }

    EXPECT_FLOAT_EQ(50.0F, pi.integral());
    EXPECT_FLOAT_EQ(50.0F, pi.output());
}



TEST(Pi, NonPositiveDtIsRejectedAndHoldsOutput) {
    foc::core::Pi pi(make_config(1.0F, 0.0F));

    EXPECT_FLOAT_EQ(0.0F, pi.step(1.0F, 0.0F, 0.0F));
    EXPECT_EQ(foc::ErrorCode::Param, pi.status());

    // stays latched until reset()
    EXPECT_FLOAT_EQ(0.0F, pi.step(1.0F, 0.0F, 0.001F));

    pi.reset();
    EXPECT_EQ(foc::ErrorCode::Ok, pi.status());
    EXPECT_FLOAT_EQ(1.0F, pi.step(1.0F, 0.0F, 0.001F));
}



TEST(Pi, NegativeGainIsRejectedAtConstruction) {
    const foc::core::Pi pi(make_config(-1.0F, 0.0F));

    EXPECT_EQ(foc::ErrorCode::Param, pi.status());
}



TEST(Pi, ResetClearsIntegral) {
    foc::core::Pi pi(make_config(0.0F, 1.0F));
    static_cast<void>(pi.step(1.0F, 0.0F, 0.1F));

    pi.reset();

    EXPECT_FLOAT_EQ(0.0F, pi.integral());
    EXPECT_FLOAT_EQ(0.0F, pi.output());
}



TEST(Pi, APerCallGainScaleMultipliesTheProportionalTermWithoutTouchingTheConfig) {
    // Compliance is a per-call request, not a reconfiguration: the stiff call after the compliant one
    // must get the configured gains back with no restore step.
    // 柔顺是按次的请求，不是重配置：柔顺调用之后的刚性调用必须直接拿回配置的增益，不需要恢复步骤。
    foc::core::Pi controller{foc::core::Pi::Config{2.0F, 0.0F, -100.0F, 100.0F, -100.0F, 100.0F}};

    const foc::core::SpeedInput compliant{1.0F, 0.2F, 0.0F}; // target, kp_scale, ilimit_scale
    EXPECT_NEAR(2.0F * 0.2F * 1.0F, controller.step(compliant, 0.0F, 1e-3F), 1e-5F);

    // 0 means "no override": the configured gain is used unchanged.
    // 0 表示"不覆盖"：按配置的增益原样使用。
    const foc::core::SpeedInput unset{1.0F, 0.0F, 0.0F};
    EXPECT_NEAR(2.0F, controller.step(unset, 0.0F, 1e-3F), 1e-4F);
}



TEST(Pi, AZeroIntegralScaleReleasesStoredTorqueAtOnce) {
    // This is what makes `ilimit_scale` useful: the integral is re-clamped into the scaled window on
    // the step that asks for it, so a joint that is about to be pushed stops holding, instead of
    // leaking its stored torque away over many steps.
    // 这正是 `ilimit_scale` 有用的原因：积分项在提出请求的那一步就被重新夹进缩放后的窗口，于是即将
    // 被推的关节立刻停止保持，而不是花很多步把已存转矩漏掉。
    foc::core::Pi controller{foc::core::Pi::Config{0.0F, 10.0F, -100.0F, 100.0F, -100.0F, 100.0F}};

    // Wind the integral up against a constant error.
    // 用恒定误差把积分积起来。
    for (int i = 0; i < 500; ++i) {
        static_cast<void>(controller.step(1.0F, 0.0F, 1e-3F));
    }
    ASSERT_GT(controller.integral(), 1.0F);

    // A tiny integral scale squeezes the window, and the stored torque goes with it immediately.
    // 极小的积分缩放把窗口挤窄，已存的转矩随之立即消失。
    const foc::core::SpeedInput limp{1.0F, 0.0F, 0.001F};
    static_cast<void>(controller.step(limp, 0.0F, 1e-3F));
    EXPECT_LT(controller.integral(), 0.11F);
}



TEST(Pi, AScaledStepKeepsTheAntiWindupProperty) {
    // Scaling must not break the invariant the integral clamp exists for: the integral never leaves
    // the (scaled) window.
    // 缩放不得破坏积分限幅存在的理由：积分项永不离开（缩放后的）窗口。
    foc::core::Pi controller{foc::core::Pi::Config{1.0F, 100.0F, -100.0F, 100.0F, -1.0F, 1.0F}};
    const foc::core::SpeedInput half{5.0F, 0.5F, 0.5F}; // integral window becomes +-0.5

    for (int i = 0; i < 1'000; ++i) {
        static_cast<void>(controller.step(half, 0.0F, 1e-3F));
        EXPECT_LE(controller.integral(), 0.5F + 1e-5F);
        EXPECT_GE(controller.integral(), -0.5F - 1e-5F);
    }
}
