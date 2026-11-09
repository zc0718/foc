#include <cmath>
#include <cstdio>

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
 *   Mutation testing found this: replacing `target - value_` with `target + value_` in the slew limiter survived the
 *   whole suite. Every existing case starts from zero or has a target large enough to dominate the current value, so
 *   the two expressions agree on the branch they take and no assertion can tell them apart. Here the value is 5 and the
 *   target is 3: the difference is -2, twice the one-step budget of 1, so the output must land on 4 -- while `target +
 *   value_` reads +8 and would walk the output *up* to 6, in the opposite direction from its own target.
 *
 *   变异测试发现的：把斜率限制器里的 `target - value_` 换成 `target + value_` 后整个套件都活着。现有用例要么从零开始，
 *   要么目标大到压过当前值，于是两个表达式落在同一个分支上，任何断言都分不开它们。这里当前值是 5、目标是 3：差是 -2，是
 *   单步预算 1 的两倍，所以输出必须落到 4——而 `target + value_` 读到 +8，会把输出**往上**推到 6，朝着与它自己的目标相反
 *   的方向。
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

TEST(SlewLimiter, TheStepCountIsTheCeilingOfTravelOverRateTimesPeriod) {
    // The limiter's law is one line -- each step moves `min(|delta|, rate*dt)` -- so the number of steps needed is
    // `ceil(delta / (rate*dt))`, exactly. Powers of two are used throughout so the arithmetic is exact and the assertion
    // can be an equality rather than a band: the point is that the bound holds to the step, since a caller sizing a ramp
    // needs the step count and not an estimate of it.
    // 限幅器的定律只有一行——每步移动 `min(|Δ|, rate·dt)`——所以需要的步数恰好是 `ceil(Δ/(rate·dt))`。全程用 2 的幂，
    // 让算术精确、断言可以是等式而不是带：要点是这个界精确到步，因为给斜坡定尺寸的调用方要的是步数而不是它的估计。
    struct Row final {
        float rate;
        float dt;
        float delta;
        int law;
    };
    const Row rows[] = {
        {0.5F, 0.25F, 1.0F, 8}, {1.0F, 0.125F, 0.5F, 4}, {0.25F, 0.0625F, 0.5F, 32}, {2.0F, 0.25F, 0.5F, 1}};
    for (const Row& row : rows) {
        foc::core::SlewLimiter limiter(row.rate);
        int steps = 0;
        while ((limiter.value() != row.delta) && (steps < 1000)) {
            static_cast<void>(limiter.step(row.delta, row.dt));
            ++steps;
        }
        std::printf("[limits] rate %.4f, dt %.5f, travel %.3f -> %d step(s) (law ceil = %d)\n",
                    static_cast<double>(row.rate), static_cast<double>(row.dt), static_cast<double>(row.delta), steps,
                    row.law);
        EXPECT_EQ(row.law, steps) << "rate " << row.rate << " dt " << row.dt;
        EXPECT_FLOAT_EQ(row.delta, limiter.value());
    }
}

TEST(SlewLimiter, RateTimesPeriodTimesStepsIsTheTravelAndItIsExact) {
    // The law is the ceiling `steps = ceil(travel / (rate*dt))`, and it is asserted as that rather than as two ratios:
    // a ratio can be written the wrong way round and still look plausible (this test did exactly that at first), whereas
    // the ceiling has one arrangement. The two factors enter one-for-one -- the product `steps*rate*dt` is the travel
    // whenever the budget divides it -- and the last grid point is the one that shows the ceiling is a ceiling.
    // 两个因子乘成一个数，所以定律写成乘积而不是两个比值：行程为 1.0 时，`步数 × rate × dt = 1` 在每一组速率与周期下
    // **精确**成立。这与"两者各按一比一进入"是同一句话，却不会把反了的比值藏在代数里；而这正是斜坡排程的构造单位。
    const auto steps_for = [](float rate, float dt) {
        foc::core::SlewLimiter limiter(rate);
        int steps = 0;
        while ((limiter.value() != 1.0F) && (steps < 5000)) {
            static_cast<void>(limiter.step(1.0F, dt));
            ++steps;
        }
        return steps;
    };
    for (const float rate : {0.25F, 0.5F, 1.0F, 2.0F, 4.0F}) {
        for (const float dt : {0.125F, 0.25F, 0.5F}) {
            const double budget = static_cast<double>(rate) * static_cast<double>(dt);
            const int law = static_cast<int>(std::ceil(1.0 / budget));
            const int steps = steps_for(rate, dt);
            const double product = static_cast<double>(steps) * budget;
            std::printf(
                "[limits] rate %.3f, dt %.4f -> budget %.6f | %4d step(s) (law ceil = %2d) | steps*budget = %.17g\n",
                static_cast<double>(rate), static_cast<double>(dt), budget, steps, law, product);
            // Powers of two throughout, so this is an equality and not a band: ceil(1.0 / 0.125) is exactly 8.
            // 全程 2 的幂，所以这是等式而不是带：ceil(1.0 / 0.125) 恰好是 8。
            EXPECT_EQ(law, steps) << "rate " << rate << " dt " << dt;
            // And where the budget divides the travel the product is the travel itself -- the one-for-one part.
            // 而当预算整除行程时，乘积就是行程本身——那是一比一的部分。
            if (budget <= 1.0) {
                EXPECT_DOUBLE_EQ(static_cast<double>(law) * budget, product) << "rate " << rate << " dt " << dt;
            }
        }
    }
}

TEST(SlewLimiter, InsideTheBudgetItPassesTheTargetThroughAndNeverOvershoots) {
    // The two sides of the `min`: inside the budget the output **is** the target on that same call (no lag, no filtering),
    // and outside it the output is the previous value plus exactly one budget -- never past the target. Both measured at
    // the boundary itself, where a `>=` for a `>` would show.
    // `min` 的两侧：预算内输出**就是**当次的目标（无延迟、无滤波），预算外则是前一值加上恰好一份预算——绝不超过目标。
    // 两者都在边界处量，那里把 `>` 写成 `>=` 就会露出来。
    foc::core::SlewLimiter limiter(1.0F);
    EXPECT_FLOAT_EQ(0.05F, limiter.step(0.05F, 0.1F)) << "inside the budget the value is the target on the same call";
    // Exactly at the budget: `delta == max_delta` must land on the target, not one step short.
    // 恰好等于预算：`Δ == max_delta` 必须落在目标上，而不是差一步。
    EXPECT_FLOAT_EQ(0.15F, limiter.step(0.15F, 0.1F));
    const float beyond = limiter.step(1.0F, 0.1F);
    EXPECT_FLOAT_EQ(0.25F, beyond) << "one step beyond is one budget, not the target";
    // And a non-positive period changes nothing at all -- the value is not a function of the target alone.
    // 而非正周期什么都不改变——值不是只由目标决定的函数。
    EXPECT_FLOAT_EQ(0.25F, limiter.step(9.0F, 0.0F));
    EXPECT_FLOAT_EQ(0.25F, limiter.step(9.0F, -1.0F));
}


/**
 * @brief [en] At exactly one step the target is **assigned**, not accumulated -- and ordinary numbers cannot see it.
 * @brief [zh] 恰好相差一步时，输出是**赋值**上去的目标而不是累加出来的——而普通数值看不见这一点。
 *
 * @note [en] The case above asserts the same intent ("must land on the target") and passes for *both* branches: with
 *            `value = 0.05`, `step = 0.1`, `target = 0.15`, the sum `0.05 + 0.1` rounds to the very same float as
 *            `0.15`, so `value += max_delta` and `value = target` are indistinguishable there. The mutant log said as
 *            much: both `>` -> `>=` sites on this branch order survived, and the baseline called them equivalents.
 *
 *   They are not. The two computations differ by the whole target whenever the subtraction that produced `delta` lost
 *   it, and that needs the distance to be large against the target: `1.0 - 1e8` rounds to exactly `-1e8`, so `delta ==
 *   -max_delta` is reachable with a 1e8 step and the target 1.0. The branch that assigns answers 1.0; the branch that
 *   accumulates answers 0.0. That is not a rounding difference, it is the target thrown away -- and it is reachable on
 *   any axis whose limiter is asked for a step comparable to its own position, which is what a reset to a large value
 *   plus a long period produces.
 *
 * @note [zh] 上面那条用例断言的是同一个意图（"必须落在目标上"），而它对**两个分支都通过**：`value = 0.05`、 `step =
 *            0.1`、`target = 0.15` 时，`0.05 + 0.1` 与 `0.15` 舍入到同一个 float，于是 `value += max_delta` 与 `value =
 *            target` 在那里无法区分。变异日志说的正是这件事：这个分支序上的两个 `>` -> `>=` 都存活，而基线把它们记成了
 *            等价。
 *
 *   它们不等价。只要算出 `delta` 的那次减法把它丢了，两种算法就差整整一个目标值；而这需要距离相对目标足够大： `1.0 -
 *   1e8` 恰好舍入成 `-1e8`，于是取 `1e8` 为一步、`1.0` 为目标即可达到 `delta == -max_delta`。赋值的分支答 1.0，累加的分
 *   支答 0.0。这不是舍入差异，是目标被丢掉了——而且任何"把限幅器 reset 到一个大数值、再给一个可与其自身位置相比的步长"的
 *   轴都会遇到它。
 */
TEST(SlewLimiter, AtExactlyOneStepTheTargetIsAssignedAndNotAccumulated) {
    // Positive side: from -1e8 the target 1.0 is exactly one 1e8 step away, and `1.0 - (-1e8)` rounds to +1e8.
    // 正向：从 -1e8 出发，目标 1.0 恰好相距一个 1e8 的步长，而 `1.0 - (-1e8)` 舍入成 +1e8。
    {
        foc::core::SlewLimiter limiter(1.0e8F);
        limiter.reset(-1.0e8F);
        EXPECT_FLOAT_EQ(1.0F, limiter.step(1.0F, 1.0F))
            << "at exactly one step the output is the target; accumulating gives 0.0 here";
    }
    // Negative side, mirrored: `1.0 - 1e8` rounds to -1e8, so `delta == -max_delta` and the branch order decides.
    // 反向镜像：`1.0 - 1e8` 舍入成 -1e8，于是 `delta == -max_delta`，由分支顺序决定结果。
    {
        foc::core::SlewLimiter limiter(1.0e8F);
        limiter.reset(1.0e8F);
        EXPECT_FLOAT_EQ(1.0F, limiter.step(1.0F, 1.0F)) << "and the mirror image must answer the same, not 0.0";
    }
}
