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



TEST(Pi, ErrorIsTheSignedDifferenceOfSetpointAndFeedback) {
    // Every other case in this file feeds a zero feedback and therefore an error of exactly one, where
    // `setpoint - feedback` and `setpoint + feedback` produce the same number, and so do `ki * error` and
    // `ki / error`. The mutation run found both surviving, which says the suite could not see a sign error
    // in the error term at all. This case uses errors that are neither zero nor one, so the arithmetic is
    // observable.
    // 本文件其余用例的反馈都是 0、误差都是 1，此时 `setpoint - feedback` 与 `+`、`ki * error` 与 `/`
    // 给出同一个数——变异测试正是发现这两个都存活。本用例取非 0 非 1 的误差，让算式可观测。
    foc::core::Pi controller{foc::core::Pi::Config{2.0F, 4.0F, -100.0F, 100.0F, -100.0F, 100.0F}};

    // error = +0.5: proportional 1.0, integral 0 + 4 * 0.5 * 0.1 = 0.2, output 1.2.
    EXPECT_NEAR(1.2F, controller.step(1.0F, 0.5F, 0.1F), 1e-6F);
    EXPECT_NEAR(0.2F, controller.integral(), 1e-6F);

    // error = -0.25: the proportional term changes sign, the integral unwinds to 0.2 + 4 * -0.25 * 0.1 = 0.1,
    // output -0.5 + 0.1 = -0.4.
    EXPECT_NEAR(-0.4F, controller.step(0.75F, 1.0F, 0.1F), 1e-6F);
    EXPECT_NEAR(0.1F, controller.integral(), 1e-6F);
}



TEST(Pi, AZeroIntegralScaleMeansNoOverrideRatherThanAnEmptyWindow) {
    // `AZeroIntegralScaleReleasesStoredTorque` uses a small *non-zero* scale, so "0 means no override" and
    // "0 means an empty window" were both consistent with it. The two only differ at exactly zero, which is
    // the boundary of `ilimit_scale > 0` -- and that comparison was found surviving.
    // `AZeroIntegralScaleReleasesStoredTorque` 用的是非零的小缩放，因此"0 表示不覆盖"与"0 表示空窗口"
    // 两种读法都符合它。二者只在恰好为 0 时不同，而那正是 `ilimit_scale > 0` 的边界——该比较被检出存活。
    foc::core::Pi controller{foc::core::Pi::Config{0.0F, 10.0F, -100.0F, 100.0F, -100.0F, 100.0F}};

    for (int i = 0; i < 10; ++i) {
        static_cast<void>(controller.step(1.0F, 0.0F, 1e-3F));
    }
    ASSERT_NEAR(0.1F, controller.integral(), 1e-6F);

    const foc::core::SpeedInput unset{1.0F, 0.0F, 0.0F};
    static_cast<void>(controller.step(unset, 0.0F, 1e-3F));

    // No override: the window stays +-100 and the integral keeps one more step, 0.1 + 10 * 1 * 0.001.
    EXPECT_NEAR(0.11F, controller.integral(), 1e-6F);
    EXPECT_NEAR(0.11F, controller.output(), 1e-6F);
}



TEST(Pi, GainsAndLimitsExactlyAtTheirBoundaryAreAccepted) {
    // The validity check is written with >= and <=, so a config *at* the boundary is legal: an integral-only
    // controller (kp == 0) and a proportional-only one (ki == 0) are both real controllers, and a window
    // whose two ends coincide is a valid (if useless) window. Mutating any of those to a strict comparison
    // must fail this case.
    // 有效性检查用的是 >= 与 <=，因此恰好落在边界上的配置是合法的：纯积分（kp == 0）与纯比例（ki == 0）
    // 都是真实控制器，两端重合的窗口也合法（虽然无用）。把任一比较改成严格比较都必须让本用例失败。
    const foc::core::Pi zero_gains{foc::core::Pi::Config{0.0F, 0.0F, -100.0F, 100.0F, -50.0F, 50.0F}};
    EXPECT_EQ(foc::ErrorCode::Ok, zero_gains.status());

    const foc::core::Pi coincident{foc::core::Pi::Config{1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F}};
    EXPECT_EQ(foc::ErrorCode::Ok, coincident.status());

    // A real inversion is still rejected: the boundary being legal must not turn the check into a no-op.
    const foc::core::Pi inverted{foc::core::Pi::Config{1.0F, 1.0F, 1.0F, 0.0F, 1.0F, 0.0F}};
    EXPECT_EQ(foc::ErrorCode::Param, inverted.status());
}



TEST(Pi, ScalingIsExactOnBothWindowEndsAndZeroLeavesThemUntouched) {
    // Two survivors that no case could see, both on the *lower* end of the integral window: every existing
    // case drives the integral positive, so `integral_min * scale` could be a division and the "no override"
    // fallback could be 1.001 instead of 1.0 without changing a single asserted number. The window here is
    // asymmetric and the integral is wound negative, so each end and each scale factor is observable.
    // 有两个存活体没有任何用例能看见，都在积分窗口的**下端**：现有用例只把积分往正向推，因此把
    // `integral_min * scale` 改成除法、或把"不覆盖"的回退值从 1.0 改成 1.001，都不会改变任何一个被断言的数。
    // 这里窗口不对称、积分往负向积，于是两端与两档缩放都变得可观测。
    foc::core::Pi controller{foc::core::Pi::Config{0.0F, 1000.0F, -100.0F, 100.0F, -0.2F, 0.5F}};

    // error = -1, so each step adds 1000 * -1 * 0.001 = -1: the lower end clamps at exactly the config's -0.2.
    for (int i = 0; i < 10; ++i) {
        static_cast<void>(controller.step(0.0F, 1.0F, 1e-3F));
    }
    EXPECT_NEAR(-0.2F, controller.integral(), 1e-6F);

    // scale 0.5 halves both ends: [-0.1, 0.25]. Halved, not doubled.
    const foc::core::SpeedInput limp{0.0F, 0.0F, 0.5F};
    static_cast<void>(controller.step(limp, 1.0F, 1e-3F));
    EXPECT_NEAR(-0.1F, controller.integral(), 1e-6F);

    // scale 0 uses the configured window unchanged -- exactly, not scaled by something close to one.
    const foc::core::SpeedInput unset{0.0F, 0.0F, 0.0F};
    static_cast<void>(controller.step(unset, 1.0F, 1e-3F));
    EXPECT_NEAR(-0.2F, controller.integral(), 1e-6F);
}
