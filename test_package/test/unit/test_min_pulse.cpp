// F05 - the gate driver's minimum pulse. A pulse narrower than `t_min` is not transmitted; what that costs is a number
// the caller can write down before it happens, and a residue can be carried so the average stops paying for it.
// F05 - 栅极驱动器的最小脉宽。窄于 `t_min` 的脉冲传不出去；它的代价是调用方**在它发生之前**就能写下的一个数，而且可以
// 把余量带过去，让平均值不必为它付账。

#include <algorithm>
#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/min_pulse.hpp>

namespace {

using foc::core::MinPulse;
using foc::core::MinPulseConfig;
using foc::core::MinPulseOutput;
using foc::core::Uvw;

constexpr float kPeriodS = 1.0e-4F; // 10 kHz: T is only ever used as a scale here
constexpr float kBusV = 24.0F;

auto make(float min_pulse_s) -> MinPulse { return MinPulse{MinPulseConfig{min_pulse_s, kPeriodS}}; }

auto request(float u, float v, float w) -> Uvw<float> { return Uvw<float>{u, v, w}; }

// A brace-initialised `MinPulse` cannot sit inside an `EXPECT_*` macro: the commas inside the braces split the macro's
// arguments. One helper, so the refusal cases stay one line each.
// 花括号初始化的 `MinPulse` 放不进 `EXPECT_*` 宏：括号里的逗号会把宏的参数切开。写成一个小助手，拒绝用例因此各自一行。
auto validate_of(float min_pulse_s, float period_s) -> foc::ErrorCode {
    return MinPulse{MinPulseConfig{min_pulse_s, period_s}}.validate();
}

} // namespace

// The law first, and on the whole range rather than at two points: the driver has exactly two thresholds and a
// pass-through, so a sweep can check all three branches and the boundary of each.
// 先看律，而且是在整段范围上而不是两个点上：驱动器只有两个阈值加一个直通，于是一次扫描就能检查三个分支和各自的边界。
TEST(MinPulse, TheDriverHasTwoThresholdsAndAPassThrough) {
    constexpr float kMin = 0.10F; // m = t_min/T
    MinPulse stage = make(kMin * kPeriodS);

    int below = 0;
    int above = 0;
    int passed = 0;
    for (int k = 0; k <= 200; ++k) {
        const float d = static_cast<float>(k) / 200.0F;
        const MinPulseOutput out = stage.apply(request(d, d, d), kBusV);
        ASSERT_EQ(foc::ErrorCode::Ok, out.status);
        const float u = out.duty.u;
        if (d < kMin) {
            // The on-pulse is too short to exist: nothing is transmitted, and the whole pulse is the loss.
            EXPECT_FLOAT_EQ(0.0F, u) << "d " << d;
            EXPECT_FLOAT_EQ(d * kPeriodS * kBusV, out.lost_vs.u) << "d " << d;
            ++below;
        } else if ((1.0F - d) < kMin) {
            // The *off*-pulse is too short, so the high side cannot turn off: the whole period conducts.
            EXPECT_FLOAT_EQ(1.0F, u) << "d " << d;
            EXPECT_FLOAT_EQ((d - 1.0F) * kPeriodS * kBusV, out.lost_vs.u) << "d " << d;
            ++above;
        } else {
            EXPECT_FLOAT_EQ(d, u) << "d " << d;
            EXPECT_FLOAT_EQ(0.0F, out.lost_vs.u) << "d " << d;
            ++passed;
        }
    }
    // All three branches were actually taken, otherwise the sweep is not testing what it says.
    // 三个分支都真的走到了，否则这次扫描测的不是它声称的东西。
    EXPECT_GT(below, 0);
    EXPECT_GT(above, 0);
    EXPECT_GT(passed, 0);
    EXPECT_EQ(201, below + above + passed); // 201 points, 0.5 % apart
}

// The accounting closes: what is not delivered is exactly what the loss report says, measured as the difference between
// the request and the run, so the two cannot be two unrelated numbers that happen to look plausible.
// 账要闭合：没交付的正好等于损失报告说的，用「请求与实际之差」量，于是两者不可能是两个碰巧看起来合理的无关数字。
TEST(MinPulse, TheLossReportIsTheSameNumberAsTheDifferenceItClaims) {
    constexpr float kMin = 0.10F;
    MinPulse stage = make(kMin * kPeriodS);
    double worst = 0.0;
    for (int k = 0; k <= 100; ++k) {
        for (int j = 0; j <= 30; ++j) {
            // Inside `[m, 1-m]` on purpose: outside it the driver narrows the pulse and `narrowed` *should* be true, so a grid
            // that straddles the band would be asserting the opposite of the law it is checking.
            // 特意取在 `[m, 1−m]` 之内：在带外驱动器会截窄脉冲、`narrowed` **应该**为真，跨带的网格等于在断言它所检查的律的反面。
            // Strictly inside, not on the edge: `min_duty()` comes out of a division, so the float that a nominal 10 % turns
            // into can be a hair above or below `0.1F`, and a grid that touches the boundary would be testing rounding.
            // 严格落在带内而不是边缘：`min_duty()` 来自一次除法，名义 10 % 变成的那个 float 可能比 `0.1F` 高也可能低，
            // 贴着边界取点的网格测的是舍入。
            const float d = 0.15F + (0.70F * static_cast<float>(k)) / 100.0F;
            const float e = 0.15F + (0.70F * static_cast<float>(j)) / 30.0F;
            const MinPulseOutput out = stage.apply(request(d, e, (d + e) * 0.5F), kBusV);
            const double stated = static_cast<double>(out.lost_vs.u);
            const double implied = static_cast<double>(d - out.duty.u) * kPeriodS * kBusV;
            worst = std::max(worst, std::fabs(stated - implied));
            EXPECT_FALSE(out.narrowed) << "d " << d << " e " << e; // nothing was narrow at these points
        }
    }
    EXPECT_LT(worst, 1.0e-9); // the two are the same expression, so this is a rounding bound and not a tolerance

    const MinPulseOutput narrow = stage.apply(request(0.02F, 0.5F, 0.98F), kBusV);
    EXPECT_TRUE(narrow.narrowed);
    EXPECT_FLOAT_EQ(0.0F, narrow.duty.u);
    EXPECT_FLOAT_EQ(1.0F, narrow.duty.w);
    EXPECT_FLOAT_EQ(0.02F * kPeriodS * kBusV, narrow.lost_vs.u);
    EXPECT_NEAR(static_cast<double>((0.98F - 1.0F) * kPeriodS * kBusV), static_cast<double>(narrow.lost_vs.w),
                1.0e-6 * 4.8e-5); // the stretched phase delivers *more*, and the 4-ulp `EXPECT_FLOAT_EQ`
                                  // is not the right tool for a product of three floats
}

// The falsifier the batch asked for: raise the threshold ten times and the volt-seconds go out of the bound that was
// written down first. It also has to saturate where it should -- a lost pulse cannot cost more than the whole pulse --
// because a number that keeps growing is not this law.
// 本批次要求的证伪：阈值抬十倍，伏秒就超掉**事先**写下的界。它还必须在该饱和的地方饱和——丢一个脉冲不可能比整个脉冲更
// 贵——因为一个会一直长大的数不是这条律。
TEST(MinPulse, TenTimesTheThresholdMovesTheVoltSecondsOutOfTheWrittenBound) {
    constexpr float kDuty = 0.12F;
    const float bound = 0.10F * kPeriodS * kBusV; // written down before the measurement

    const MinPulseOutput slack = make(0.05F * kPeriodS).apply(request(kDuty, kDuty, kDuty), kBusV);
    const MinPulseOutput tight = make(0.50F * kPeriodS).apply(request(kDuty, kDuty, kDuty), kBusV);

    std::printf("[minpulse] d = %.3f | m = 5 %%  -> lost %.3e V*s | m = 50 %% -> lost %.3e V*s | bound %.3e V*s\n",
                static_cast<double>(kDuty), static_cast<double>(slack.lost_vs.u), static_cast<double>(tight.lost_vs.u),
                static_cast<double>(bound));
    EXPECT_FLOAT_EQ(0.0F, slack.lost_vs.u);                     // 12 % survives a 5 % limit
    EXPECT_FLOAT_EQ(kDuty * kPeriodS * kBusV, tight.lost_vs.u); // and dies at a 50 % one
    EXPECT_GT(tight.lost_vs.u, bound);

    // Saturation: asked for 2 %, the loss is the pulse and not the threshold, whatever the threshold is.
    const MinPulseOutput tiny_loose = make(0.05F * kPeriodS).apply(request(0.02F, 0.02F, 0.02F), kBusV);
    const MinPulseOutput tiny_tight = make(0.40F * kPeriodS).apply(request(0.02F, 0.02F, 0.02F), kBusV);
    EXPECT_FLOAT_EQ(tiny_loose.lost_vs.u, tiny_tight.lost_vs.u) << "a lost pulse costs the pulse, not the limit";
    EXPECT_FLOAT_EQ(0.02F * kPeriodS * kBusV, tiny_tight.lost_vs.u);
}

// The residue: `sum(applied) = sum(requested) - carry` over any window, the carry never exceeds one minimum pulse, and
// therefore the *average* error over N periods is at most m/N -- the number that makes the carry worth having.
// 余量：任意窗口上 `Σapplied = Σrequested − carry`，余量不超过一个最小脉冲，于是 N 个周期的**平均**误差至多 m/N——
// 这个数才是带余量值得做的理由。
TEST(MinPulse, TheResidueMakesTheAverageExactAndNeverExceedsOnePulse) {
    constexpr float kMin = 0.10F;
    constexpr int kPeriods = 100;
    const float d = 0.05F; // half of the smallest transmissible pulse: dropped every period without a carry
    MinPulse stage = make(kMin * kPeriodS);

    double sum_requested = 0.0;
    double sum_applied = 0.0;
    int transmitted = 0;
    for (int k = 0; k < kPeriods; ++k) {
        const MinPulseOutput out = stage.shape(request(d, d, d), kBusV);
        ASSERT_EQ(foc::ErrorCode::Ok, out.status);
        sum_requested += static_cast<double>(d);
        // The bridge runs the target minus the residue that is still owed; `out.duty` is the *target*, and conflating the two
        // is the mistake this line exists to prevent (the first version of this case counted `out.duty` and reported that
        // nothing was ever transmitted).
        // 桥臂跑的是「目标减去仍欠的余量」；`out.duty` 是**目标**，把两者混为一谈正是这一行要防的错（这条用例的第一版就是数的
        // `out.duty`，于是报出「什么都没送出去」）。
        const double applied_u = static_cast<double>(out.duty.u) - static_cast<double>(stage.carry().u);
        sum_applied += applied_u;
        if (applied_u > 0.0) {
            ++transmitted;
        }
        EXPECT_LE(std::fabs(static_cast<double>(stage.carry().u)), static_cast<double>(kMin) + 1.0e-6);
    }

    // The counterfactual, measured with the same request and the *same* class: no carry, so every period is dropped and the
    // mean voltage is short by the whole request -- 0.05 duty = 1.2 V at 24 V, for as long as the loop runs.
    // 反事实，用同一个请求、**同一个类**量：不带余量，于是每个周期都被丢掉，平均电压整整少掉请求——0.05 占空比 = 24 V 下
    // 1.2 V，只要环路还在跑就一直少。
    MinPulse plain = make(kMin * kPeriodS);
    double plain_sum = 0.0;
    for (int k = 0; k < kPeriods; ++k) {
        plain_sum += static_cast<double>(plain.apply(request(d, d, d), kBusV).duty.u);
    }
    const double plain_mean_error = (kPeriods * static_cast<double>(d)) - plain_sum;
    std::printf("[minpulse] without the carry: mean short by %.4f duty = %.3f V over %d periods\n",
                plain_mean_error / kPeriods, plain_mean_error / kPeriods * kBusV, kPeriods);
    EXPECT_GT(plain_mean_error / kPeriods, 0.049) << "every period dropped, as the rule says";

    const double identity_error = std::fabs((sum_applied + static_cast<double>(stage.carry().u)) - sum_requested);
    std::printf("[minpulse] %d periods at d = %.3f, m = %.2f: transmitted %d pulses, carry %.4f, "
                "mean error %.2e duty (= %.3e V)\n",
                kPeriods, static_cast<double>(d), static_cast<double>(kMin), transmitted,
                static_cast<double>(stage.carry().u), std::fabs(sum_requested - sum_applied) / kPeriods,
                std::fabs(sum_requested - sum_applied) / kPeriods * kBusV);
    EXPECT_LT(identity_error, 1.0e-6) << "sum(applied) + carry == sum(requested)";
    // Without the carry, 100 % of these periods would be dropped: 0.05 duty = 1.2 V of missing mean voltage.
    EXPECT_GT(transmitted, 0) << "a carry that never gets delivered is a carry that only accumulates";
    const double mean_error = std::fabs(sum_requested - sum_applied) / kPeriods;
    EXPECT_LT(mean_error, static_cast<double>(kMin) / kPeriods + 1.0e-6) << "m/N is the bound the class promises";
}

// The configurations that are refused are the ones that cannot *be* a limit, and the default is "no limit at all" --
// which has to stay the identity, because that is how every measurement with the feature off is taken.
// 被拒绝的配置是那些**不能算作**极限的；而默认值是「没有极限」——它必须保持为恒等映射，因为所有「关掉这个功能」的实测
// 都是这么做的。
TEST(MinPulse, TheRefusedConfigurationsCannotBeALimitAndOffIsTheIdentity) {
    const MinPulse off = make(0.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, off.validate());
    EXPECT_FLOAT_EQ(0.0F, off.min_duty());
    const MinPulseOutput identity = off.apply(request(0.0F, 0.37F, 1.0F), kBusV);
    EXPECT_FLOAT_EQ(0.0F, identity.duty.u);
    EXPECT_FLOAT_EQ(0.37F, identity.duty.v);
    EXPECT_FLOAT_EQ(1.0F, identity.duty.w);
    EXPECT_FALSE(identity.narrowed);

    EXPECT_EQ(foc::ErrorCode::Param, make(-1.0F).validate()) << "a negative limit is a typo, not a limit";
    EXPECT_EQ(foc::ErrorCode::Param, validate_of(0.1F * kPeriodS, 0.0F)) << "no period";
    EXPECT_EQ(foc::ErrorCode::Param, validate_of(0.1F * kPeriodS, -1.0F)) << "negative period";
    // `2*t_min > T`: the pulse and the gap after it are both too short, so no duty whatsoever is transmissible.
    EXPECT_EQ(foc::ErrorCode::Param, validate_of(0.6F * kPeriodS, kPeriodS));
    EXPECT_EQ(foc::ErrorCode::Ok, make(0.5F * kPeriodS).validate()) << "exactly half is still a limit";

    // A refusal has to leave the state alone, not only return a code: the residue is what the next period will act on, so a
    // rejected call that half-updated it would corrupt a period that never ran. Checked on a class that *has* a residue.
    // 拒绝除了返回错误码，还必须**不动状态**：余量是下一周期要照着做的量，一次半途更新的被拒调用会污染一个从未运行过的
    // 周期。在一个**确实**带有余量的实例上检查这一点。
    MinPulse carried = make(0.10F * kPeriodS);
    (void)carried.shape(request(0.05F, 0.05F, 0.05F), kBusV);
    const float residue_before = carried.carry().u;
    ASSERT_GT(residue_before, 0.0F); // the fixture only means something if there was something to lose
    const MinPulseOutput refused = carried.shape(request(0.05F, 0.05F, 0.05F), 0.0F); // a bus that is not a voltage
    EXPECT_EQ(foc::ErrorCode::Param, refused.status);
    EXPECT_FLOAT_EQ(residue_before, carried.carry().u) << "a refused period must not touch the residue";
    EXPECT_FLOAT_EQ(residue_before, carried.carry().v);
    EXPECT_FLOAT_EQ(residue_before, carried.carry().w);

    // A bus voltage that is not a voltage cannot produce volt-seconds, and saying Ok would turn a bug into a number.
    EXPECT_EQ(foc::ErrorCode::Param, make(0.1F * kPeriodS).apply(request(0.5F, 0.5F, 0.5F), 0.0F).status);
    EXPECT_EQ(foc::ErrorCode::Param, make(0.1F * kPeriodS).apply(request(0.5F, 0.5F, 0.5F), NAN).status);
    // Out-of-range duties are clamped rather than propagated, and the clamp is visible as a loss.
    const MinPulseOutput clamped = make(0.1F * kPeriodS).apply(request(-0.2F, 0.5F, 1.3F), kBusV);
    EXPECT_FLOAT_EQ(0.0F, clamped.duty.u);
    EXPECT_FLOAT_EQ(1.0F, clamped.duty.w);
}
