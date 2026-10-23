#include <gtest/gtest.h>

#include <foc/core/limits.hpp>



TEST(Limits, ClampKeepsInteriorValues) { EXPECT_FLOAT_EQ(0.5F, foc::core::clamp(0.5F, -1.0F, 1.0F)); }



TEST(Limits, ClampSaturatesBothSides) {
    EXPECT_FLOAT_EQ(-1.0F, foc::core::clamp(-5.0F, -1.0F, 1.0F));
    EXPECT_FLOAT_EQ(1.0F, foc::core::clamp(5.0F, -1.0F, 1.0F));
}



TEST(Limits, ClampIsConstexpr) {
    static_assert(foc::core::clamp(42, 0, 10) == 10, "clamp must be usable in constant expressions");
    static_assert(foc::core::clamp(-3, 0, 10) == 0, "clamp must be usable in constant expressions");
}



TEST(SlewLimiter, RampsTowardsTargetAtConfiguredRate) {
    foc::core::SlewLimiter limiter(10.0F); // 10 units per second

    EXPECT_FLOAT_EQ(1.0F, limiter.step(100.0F, 0.1F));
    EXPECT_FLOAT_EQ(2.0F, limiter.step(100.0F, 0.1F));
}



TEST(SlewLimiter, ReachesTargetWhenInsideOneStep) {
    foc::core::SlewLimiter limiter(10.0F);

    EXPECT_FLOAT_EQ(0.5F, limiter.step(0.5F, 0.1F));
    EXPECT_FLOAT_EQ(0.5F, limiter.value());
}



TEST(SlewLimiter, LimitsNegativeDirection) {
    foc::core::SlewLimiter limiter(10.0F);
    limiter.reset(1.0F);

    EXPECT_FLOAT_EQ(0.0F, limiter.step(-100.0F, 0.1F));
}



TEST(SlewLimiter, NonPositiveDtHoldsValue) {
    foc::core::SlewLimiter limiter(10.0F);

    EXPECT_FLOAT_EQ(0.0F, limiter.step(100.0F, 0.0F));
    EXPECT_FLOAT_EQ(0.0F, limiter.step(100.0F, -1.0F));
}
