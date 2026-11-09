// The brake chopper: hysteresis that actually holds, a duty that cannot exceed what the resistor was rated for, and a
// thermal budget that latches instead of politely resuming.
//
// The criteria, written before the run:
//
//   1. **the gap is the hysteresis.** Ramped slowly through the band, the chopper changes state **once**, and at the
//      first sample past the threshold; a bang-bang without the gap changes state at the sampling rate, which is a
//      pulse per control period forever;
//   2. **inside the band the state is kept**, so the boundary is `>` and not `>=`: a voltage exactly on `v_off` does not
//      switch it off, and exactly on `v_on` does not switch it on;
//   3. duty and current stay inside `[0, duty_max]` and the current is `duty*v/r` to the bit;
//   4. **the budget latches.** Driven past its A^2*s, the chopper stops -- and it does **not** resume when the bus falls
//      back to a quiet voltage, because the resistor is still hot. A chopper that resumed would destroy the resistor
//      politely, one period at a time;
//   5. **the budget cools**, so the latch is not a once-in-a-life event: over a long quiet stretch the accumulated heat
//      falls and the chopper can work again after `reset()` -- the difference between "cooled" and "cleared";
//   6. `min_on_s` is a floor on the pulse, not decoration: a dip shorter than it does not switch the resistor off;
//   7. a chopper that cannot act is **refused**, and the refusal leaves the state exactly as it was.
//
// 判据写在跑前（见上）：间隔就是滞环（缓慢扫过区间只切一次，且发生在越过阈值的第一拍）；区间内保持状态（所以边界是 `>`
// 而不是 `>=`）；占空比与电流有界且 `i = duty·v/r` 逐位成立；预算 **锁存** 并且母线回落到平静电压时 **不** 自行恢复（电阻还是
// 热的）；预算会 **冷却** （所以锁存不是一辈子一次），但"冷却过了"与"清除了"是两件事；`min_on_s` 是脉冲下限而不是装饰；动不了
// 的斩波器被拒绝，而拒绝不改任何状态。

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/brake_chopper.hpp>

namespace {

using foc::ErrorCode;
using foc::core::BrakeChopper;
using foc::core::BrakeChopperConfig;
using foc::core::BrakeChopperOutput;

constexpr float kDt = 50e-6F;

auto config(float budget_a2s = 0.0F, float cool_tau_s = 0.0F, float min_on_s = 0.0F) -> BrakeChopperConfig {
    BrakeChopperConfig c{};
    c.v_on = 28.0F;
    c.v_off = 26.0F;
    c.r_ohm = 10.0F;
    c.duty_max = 0.9F;
    c.min_on_s = min_on_s;
    c.min_off_s = 0.0F;
    c.i2t_limit_a2s = budget_a2s;
    c.i2t_cool_tau_s = cool_tau_s;
    return c;
}

// 1 and 2. The band, walked slowly, and the boundary that decides whether it holds.
TEST(BrakeChopper, TheGapIsTheHysteresisAndTheBoundaryIsStrict) {
    BrakeChopper chopper(config());
    // Rise from 24 V to 30 V in 10 mV steps, one control period each.
    int transitions = 0;
    bool previous = false;
    float first_on_v = 0.0F;
    for (int k = 0; k <= 600; ++k) {
        const float v = 24.0F + (0.01F * static_cast<float>(k));
        const BrakeChopperOutput out = chopper.step(v, kDt);
        ASSERT_EQ(out.status, ErrorCode::Ok);
        if (out.on != previous) {
            ++transitions;
            if (out.on) {
                first_on_v = v;
            }
            previous = out.on;
        }
    }
    EXPECT_EQ(transitions, 1) << "a chopper without a gap changes state at the sampling rate";
    EXPECT_GT(first_on_v, 28.0F) << "the boundary is `> v_on`, so exactly 28.00 V must not switch it on";
    EXPECT_LT(first_on_v, 28.02F);

    // Falling back into the band keeps it on: 26.50 V is between the two thresholds and the state must be kept.
    BrakeChopper staying(config());
    static_cast<void>(staying.step(29.0F, kDt));
    ASSERT_TRUE(staying.on());
    static_cast<void>(staying.step(26.5F, kDt));
    EXPECT_TRUE(staying.on()) << "inside the band the chopper does not change its mind";
    // Exactly on `v_off` still keeps it on (`< v_off` is the rule), one step below switches it off.
    static_cast<void>(staying.step(26.0F, kDt));
    EXPECT_TRUE(staying.on());
    static_cast<void>(staying.step(25.99F, kDt));
    EXPECT_FALSE(staying.on());
}

// 3. The bounds and the current law -- swept over the whole bus range, with the wrong law computed alongside so the
//    check cannot be a comment with a runtime cost. A chopper that ignored `duty_max` leaves the duty bounds, and one
//    that reported `v/r` instead of `duty*v/r` is wrong by exactly the duty factor on every driven period.
// 3. 界与电流律——扫遍整个母线范围，并把"错误的律"一并算出来，于是这条检查不是一条带运行时开销的注释。忽略
//    `duty_max` 的斩波器会越出占空比界，而把电流报成 `v/r` 而不是 `duty*v/r` 的那个，在每一个被驱动的周期上都差一个
//    占空比因子。
TEST(BrakeChopper, TheDutyAndTheCurrentAreTheLawAndNothingElse) {
    BrakeChopper chopper(config());
    const BrakeChopperOutput on = chopper.step(30.0F, kDt);
    ASSERT_EQ(on.status, ErrorCode::Ok);
    EXPECT_TRUE(on.on);
    EXPECT_FLOAT_EQ(on.duty, 0.9F);
    EXPECT_FLOAT_EQ(on.current_a, (0.9F * 30.0F) / 10.0F); // 2.7 A
    EXPECT_LE(on.duty, chopper.config().duty_max);
    EXPECT_GE(on.current_a, 0.0F);

    const BrakeChopperOutput off = chopper.step(24.0F, kDt);
    EXPECT_FALSE(off.on);
    EXPECT_FLOAT_EQ(off.duty, 0.0F);
    EXPECT_FLOAT_EQ(off.current_a, 0.0F);

    // The same law, swept: a sawtooth across the whole plausible bus range, 2000 periods, so every voltage is visited
    // in both directions and the counts below are not a single lucky point.
    BrakeChopper swept(config());
    float worst_duty_excess = 0.0F;
    float worst_current_error = 0.0F;
    float worst_wrong_law_error = 1e30F;
    int driven = 0;
    for (int k = 0; k < 4000; ++k) {
        const int phase = k % 200;
        const float v = (phase < 100) ? (20.0F + (0.2F * static_cast<float>(phase)))
                                      : (40.0F - (0.2F * static_cast<float>(phase - 100)));
        const BrakeChopperOutput out = swept.step(v, kDt);
        ASSERT_EQ(out.status, ErrorCode::Ok);
        worst_duty_excess = std::fmax(worst_duty_excess, out.duty - swept.config().duty_max);
        EXPECT_GE(out.duty, 0.0F);
        EXPECT_GE(out.current_a, 0.0F);
        worst_current_error =
            std::fmax(worst_current_error, std::fabs(out.current_a - ((out.duty * v) / swept.config().r_ohm)));
        if (out.duty > 0.0F) {
            ++driven;
            // What a chopper reporting the resistor current as if the duty were 1 would say.
            worst_wrong_law_error =
                std::fmin(worst_wrong_law_error, std::fabs(out.current_a - (v / swept.config().r_ohm)));
        }
    }
    std::printf("[chopper] sweep: %d driven period(s), worst duty excess %.3e, worst |i - duty*v/r| %.3e, "
                "the wrong law is off by at least %.3f A\n",
                driven, static_cast<double>(worst_duty_excess), static_cast<double>(worst_current_error),
                static_cast<double>(worst_wrong_law_error));
    EXPECT_LE(worst_duty_excess, 0.0F);
    EXPECT_EQ(worst_current_error, 0.0F) << "current == duty*v/r is not an approximation";
    EXPECT_GT(worst_wrong_law_error, 0.05F) << "the wrong law has to be visibly wrong, or the check above is empty";
    EXPECT_GT(driven, 1000) << "the sweep must actually drive the chopper, or this case tests nothing";
}

// 3b. The heat is the **mean square**, not the square of the period average. Two computations of one quantity, and
//     the difference between them is exactly `1/duty_max` -- which is the whole reason this case exists: the first
//     version of this header integrated `current_a^2`, so a chopper driven at 35 % duty spent 2.9x the budget it was
//     given while every number it reported looked conservative. Found in SIL, where the bus meter books the same
//     energy from the other end (`r * i2t` must equal the energy the resistor actually took).
// 3b. 热量是 **均方** ，不是周期平均值的平方。同一个量的两次计算，差别恰好是 `1/duty_max`——整个用例存在的理由：这个头
//     文件的第一版积的是 `current_a²`，于是一个按 35 % 占空比驱动的斩波器会花掉预算的 2.9 倍，而它报出的每个数字都显
//     得是保守的。在 SIL 里发现：母线 meter 从另一端记着同一份能量（`r * i²t` 必须等于电阻真正取走的能量）。
TEST(BrakeChopper, TheHeatIsTheMeanSquareAndNotTheSquareOfTheMean) {
    const BrakeChopperConfig cfg = config();
    BrakeChopper chopper(cfg);
    const double on_current = static_cast<double>(30.0F) / static_cast<double>(cfg.r_ohm); // 3 A while closed
    const double mean_square = on_current * on_current * static_cast<double>(cfg.duty_max);
    const double square_of_mean =
        (on_current * static_cast<double>(cfg.duty_max)) * (on_current * static_cast<double>(cfg.duty_max));
    ASSERT_GT(mean_square, square_of_mean) << "the fixture needs a duty below 1 for the two to differ at all";

    const BrakeChopperOutput out = chopper.step(30.0F, kDt);
    ASSERT_TRUE(out.on);
    std::printf("\n[chopper] heat over one period: mean square %.6e vs squared mean %.6e (ratio %.4f = 1/duty)\n",
                mean_square * static_cast<double>(kDt), square_of_mean * static_cast<double>(kDt),
                mean_square / square_of_mean);
    EXPECT_DOUBLE_EQ(chopper.i2t_a2s(), mean_square * static_cast<double>(kDt));
    EXPECT_NEAR(mean_square / square_of_mean, 1.0 / static_cast<double>(cfg.duty_max), 1e-12)
        << "the two laws differ by exactly the reciprocal of the duty";
    // And the average current that the bus carries is still the duty-averaged one: the two numbers describe the same
    // switch from two sides, so neither may be "fixed" into the other.
    EXPECT_FLOAT_EQ(out.current_a, static_cast<float>(on_current) * cfg.duty_max);
}

// 4. The latch, and the thing it must not do.
TEST(BrakeChopper, TheBudgetLatchesAndDoesNotResumeWhenTheBusGoesQuiet) {
    BrakeChopper chopper(config(/*budget*/ 0.5F));
    const float on_current = 30.0F / config().r_ohm; // 3 A, not the 2.7 A period average
    const float per_period = (on_current * on_current) * 0.9F * kDt;
    const int expected_periods = static_cast<int>(0.5F / per_period); // ~1234

    int driven = 0;
    for (int k = 0; k < 5000; ++k) {
        const BrakeChopperOutput out = chopper.step(30.0F, kDt);
        ASSERT_EQ(out.status, ErrorCode::Ok);
        if (out.duty > 0.0F) {
            ++driven;
        }
        if (out.budget_exceeded) {
            break;
        }
    }
    std::printf("\n[chopper] budget 0.5 A^2*s at %.2f A closed (%.2f A average): driven %d period(s), expected "
                "about %d\n",
                static_cast<double>(on_current), static_cast<double>(on_current * 0.9F), driven, expected_periods);
    EXPECT_TRUE(chopper.budget_exceeded());
    EXPECT_NEAR(static_cast<float>(driven), static_cast<float>(expected_periods), 4.0F);
    EXPECT_FALSE(chopper.on());

    // The load-bearing half: the bus falls to a completely quiet voltage and the chopper still does nothing.
    for (int k = 0; k < 100; ++k) {
        const BrakeChopperOutput out = chopper.step(10.0F, kDt);
        EXPECT_EQ(out.duty, 0.0F) << "a latched chopper that resumed would cook the resistor one period at a time";
        EXPECT_TRUE(out.budget_exceeded);
    }
}

// 5. Cooling is real, and it is not the same thing as clearing -- and a cooling budget can be **unreachable**.
//
// The first version of this case asked for a 0.5 A^2*s budget with a 50 ms cooling constant, and the latch never
// fired. That was not a bug: with cooling the heat settles at `I^2 * tau` (the balance of the heat going in and `q/tau`
// going out), so a budget above that value is a budget the resistor can never spend. 8.1 A^2 * 0.05 s = 0.405, which
// is below 0.5 -- the fixture was asking for something physically impossible, and the test was right to refuse it.
//
// So the criterion is derived instead of picked: trip happens when `q(t) = (I^2)*tau*(1 - exp(-t/tau))` reaches the
// budget, i.e. at `t = -tau*ln(1 - limit/(I^2*tau))`. `I^2` there is the **mean square** -- `duty*(v/r)^2`, not the
// square of the period average; see case 3b for why those are not the same number.
// 第一版这个用例要的是 0.5 A²s 的预算配 50 ms 的冷却常数，而锁存从来没跳过。那不是缺陷：带冷却时热量稳定在 `I²·τ`
// （热量进来、`q/τ` 出去之间的平衡），所以高于这个值的预算是电阻 **永远花不掉** 的预算。8.1 × 0.05 = 0.405，低于
// 0.5——是夹具在要求一件物理上不可能的事，用例拒绝它是对的。所以判据改成推导：`q(t) = I²τ(1 − e^{-t/τ})` 达到预算时跳，
// 即 `t = −τ·ln(1 − limit/(I²τ))`。这里的 `I²` 是 **均方** ——`duty·(v/r)²`，不是周期平均值的平方；两者为何不同见用例 3b。
TEST(BrakeChopper, TheBudgetTripsAtTheClosedFormTimeAndCoolsAfterwards) {
    // The mean square current, not the square of the period average: `duty*(v/r)^2` = 0.9*9 = 8.1 A^2.
    const float closed_current_a = 30.0F / 10.0F; // 3 A while the switch is closed
    const float i_squared = 0.9F * closed_current_a * closed_current_a;
    const float budget = 0.5F;
    const float tau = 0.2F;
    ASSERT_GT(i_squared * tau, budget) << "the fixture must ask for a budget the resistor can actually spend";

    const float predicted_s = -tau * std::log(1.0F - (budget / (i_squared * tau)));
    const int predicted_periods = static_cast<int>(predicted_s / kDt);
    std::printf("\n[chopper] closed form: i^2*tau = %.4f A^2*s, budget %.2f, trip at %.6f s = %d periods at %.0f us\n",
                static_cast<double>(i_squared * tau), static_cast<double>(budget), static_cast<double>(predicted_s),
                predicted_periods, static_cast<double>(kDt * 1e6F));

    BrakeChopper chopper(config(budget, tau));
    int periods = 0;
    while ((periods < 20000) && !chopper.budget_exceeded()) {
        static_cast<void>(chopper.step(30.0F, kDt));
        ++periods;
    }
    std::printf("[chopper] measured trip at %d period(s), heat %.6f A^2*s\n", periods,
                static_cast<double>(chopper.i2t_a2s()));
    EXPECT_TRUE(chopper.budget_exceeded());
    // The exponential is exact, so the only disagreement allowed is the sub-step resolution plus the discrete-time
    // version of the same equation.
    EXPECT_NEAR(static_cast<float>(periods), static_cast<float>(predicted_periods),
                0.02F * static_cast<float>(predicted_periods));

    const double at_trip = chopper.i2t_a2s();
    for (int k = 0; k < 6000; ++k) { // 300 ms = 1.5 time constants
        static_cast<void>(chopper.step(10.0F, kDt));
    }
    const double cooled = chopper.i2t_a2s();
    std::printf("[chopper] heat at trip %.6f -> cooled to %.6f after 300 ms (tau 200 ms)\n",
                static_cast<double>(at_trip), static_cast<double>(cooled));
    EXPECT_LT(cooled, 0.25F * at_trip) << "1.5 time constants leave about a fifth";
    EXPECT_TRUE(chopper.budget_exceeded()) << "cooled is not cleared";

    chopper.reset();
    EXPECT_FALSE(chopper.budget_exceeded());
    EXPECT_DOUBLE_EQ(chopper.i2t_a2s(), 0.0);
    EXPECT_TRUE(chopper.step(30.0F, kDt).on) << "after a reset the chopper is allowed to work again";
}

// 5b. The other half of that derivation: a budget above `i^2 * tau` is unreachable, and the chopper correctly never
//     trips. Recorded because the first version of the case above assumed otherwise.
TEST(BrakeChopper, ABudgetAboveTheCoolingEquilibriumIsNeverSpent) {
    const float closed_current_a = 30.0F / 10.0F;
    const float tau = 0.05F;
    const float equilibrium = 0.9F * closed_current_a * closed_current_a * tau; // 0.405 A^2*s
    BrakeChopper chopper(config(/*budget*/ 0.5F, tau));                         // above the equilibrium on purpose

    double heat = 0.0;
    for (int k = 0; k < 20000; ++k) { // 1 s = 20 time constants
        heat = chopper.step(30.0F, kDt).i2t_a2s;
    }
    std::printf("[chopper] budget 0.5 above the %.4f equilibrium: heat settles at %.4f, latched=%d\n",
                static_cast<double>(equilibrium), static_cast<double>(heat),
                static_cast<int>(chopper.budget_exceeded()));
    EXPECT_FALSE(chopper.budget_exceeded());
    EXPECT_NEAR(heat, equilibrium, 0.01F * equilibrium) << "the heat settles at i^2 * tau, and that is the whole story";
}

// 6. `min_on_s` is a floor on the pulse.
TEST(BrakeChopper, TheMinimumOnTimeIsAFloorOnThePulse) {
    // 1 ms floor; the bus dips below `v_off` for only 0.5 ms, then comes back.
    BrakeChopper chopper(config(0.0F, 0.0F, /*min_on*/ 1e-3F));
    static_cast<void>(chopper.step(30.0F, kDt));
    ASSERT_TRUE(chopper.on());
    for (int k = 0; k < 10; ++k) { // 0.5 ms below v_off
        static_cast<void>(chopper.step(20.0F, kDt));
        EXPECT_TRUE(chopper.on()) << "a pulse shorter than the floor is not a pulse";
    }
    for (int k = 0; k < 30; ++k) { // keep going: the floor has passed, so it may now switch off
        static_cast<void>(chopper.step(20.0F, kDt));
    }
    EXPECT_FALSE(chopper.on());
}

// 7. A chopper that cannot act is refused, and the refusal is not a state change.
TEST(BrakeChopper, AChopperThatCannotActIsRefusedRatherThanDefaulted) {
    struct Case final {
        const char* what;
        BrakeChopperConfig config;
    };
    BrakeChopperConfig zero_resistor = config();
    zero_resistor.r_ohm = 0.0F;
    BrakeChopperConfig inverted = config();
    inverted.v_on = 20.0F;
    inverted.v_off = 26.0F;
    BrakeChopperConfig zero_duty = config();
    zero_duty.duty_max = 0.0F;

    const Case cases[] = {
        {"a zero resistor", zero_resistor}, {"an inverted band", inverted}, {"a zero duty", zero_duty}};
    for (const Case& c : cases) {
        BrakeChopper chopper(c.config);
        const BrakeChopperOutput out =
            chopper.step(40.0F, kDt); // far above the threshold, and it must still do nothing
        EXPECT_EQ(out.status, ErrorCode::Param) << c.what;
        EXPECT_FLOAT_EQ(out.duty, 0.0F) << c.what;
        EXPECT_FLOAT_EQ(out.current_a, 0.0F) << c.what;
        EXPECT_FALSE(out.on) << c.what;
    }

    // A period of zero cannot be integrated, and it must not advance anything either.
    BrakeChopper chopper(config());
    ASSERT_TRUE(chopper.step(30.0F, kDt).on);
    const double heat_before = chopper.i2t_a2s();
    EXPECT_EQ(chopper.step(30.0F, 0.0F).status, ErrorCode::Param);
    EXPECT_DOUBLE_EQ(chopper.i2t_a2s(), heat_before);
    EXPECT_TRUE(chopper.on());
}

} // namespace
