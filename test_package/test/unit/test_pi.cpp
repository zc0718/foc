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
