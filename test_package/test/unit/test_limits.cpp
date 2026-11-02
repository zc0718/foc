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



/**
 * @brief [en] The step is the *difference* `target - value`, and its sign decides which way the output moves.
 *
 * @brief [zh] 步进是**差** `target - value`，它的符号决定输出往哪边走。
 *
 *     Mutation testing found this: replacing `target - value_` with `target + value_` in the slew limiter survived the
 *     whole suite. Every existing case starts from zero or has a target large enough to dominate the current value, so
 *     the two expressions agree on the branch they take and no assertion can tell them apart. Here the value is 5 and
 *     the target is 3: the difference is -2, twice the one-step budget of 1, so the output must land on 4 -- while
 *     `target + value_` reads +8 and would walk the output *up* to 6, in the opposite direction from its own target.
 *
 *     变异测试发现的：把斜率限制器里的 `target - value_` 换成 `target + value_` 后整个套件都活着。现有用例要么从零开
 *     始，要么目标大到压过当前值，于是两个表达式落在同一个分支上，任何断言都分不开它们。这里当前值是 5、目标是 3：差是
 *     -2，是单步预算 1 的两倍，所以输出必须落到 4——而 `target + value_` 读到 +8，会把输出**往上**推到 6，朝着与它自己的
 *     目标相反的方向。
 */
TEST(SlewLimiter, TheErrorTermIsADifferenceSoItsSignIsObservable) {
    foc::core::SlewLimiter limiter(10.0F);
    limiter.reset(5.0F);

    EXPECT_FLOAT_EQ(4.0F, limiter.step(3.0F, 0.1F)); // down one step, not up
    EXPECT_FLOAT_EQ(3.0F, limiter.step(3.0F, 0.1F)); // and then inside one step, so it lands exactly

    limiter.reset(-5.0F);
    EXPECT_FLOAT_EQ(-4.0F, limiter.step(-3.0F, 0.1F)); // the mirror image
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
