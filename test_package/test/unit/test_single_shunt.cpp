// F06 - single-shunt current reconstruction. One ADC channel in the DC link has to produce three phase currents, and the
// honest part of the job is knowing when it cannot: near a sector boundary one active interval collapses and there is no
// reading to be had. This file measures the window rule, the algebra, the refusal, and the price of the topology.
// F06 - 单电阻电流重构。直流回路上一个 ADC 通道要产出三相电流，而这件事里诚实的部分是知道它什么时候做不到：靠近扇区边界
// 时一个有效区间塌掉，根本没有读数可取。本文件量窗口判据、代数、拒绝，以及这个拓扑的代价。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/single_shunt.hpp>

namespace {

using foc::ErrorCode;
using foc::core::ShuntCurrents;
using foc::core::ShuntVerdict;
using foc::core::ShuntWindow;
using foc::core::SingleShunt;
using foc::core::Uvw;

constexpr float kPeriodS = 1.0e-4F; // 10 kHz
constexpr float kSettleS = 1.0e-6F; // ringing after each edge
constexpr float kSampleS = 0.5e-6F; // ADC aperture

auto make() -> SingleShunt { return SingleShunt{SingleShunt::Config{kSettleS, kSampleS, kPeriodS}}; }

/** @brief [en] The three line currents of a balanced star at a given angle and magnitude. [zh] 给定角度与幅值的平衡星形三相线电流。 */
auto balanced(double angle, double magnitude) -> Uvw<float> {
    const double a = magnitude * std::cos(angle);
    const double b = magnitude * std::cos(angle - (2.0 * M_PI / 3.0));
    const double c = magnitude * std::cos(angle + (2.0 * M_PI / 3.0));
    return Uvw<float>{static_cast<float>(a), static_cast<float>(b), static_cast<float>(c)};
}

/** @brief [en] Deterministic uniform noise in [-a, a], so a noise statistic is reproducible. [zh] [-a, a] 上的确定性均匀噪声，噪声统计因此可复现。 */
auto next_noise(std::uint32_t& state, float a) -> float {
    state = (state * 1103515245U) + 12345U;
    const float u = static_cast<float>(state & 0x7FFFFFFFU) / static_cast<float>(0x7FFFFFFFU); // [0, 1]
    return ((2.0F * u) - 1.0F) * a;
}

} // namespace

// The times, and the one arithmetic fact that makes them checkable without a plant: they partition the period, whatever
// the duties are. A law that quietly reshuffles them cannot pass this.
// 那些时间，以及一条使它无需 plant 就能被检验的算术事实：无论占空比是什么，它们都是一个周期的划分。悄悄把它们挪过的律过不了。
TEST(SingleShunt, TheThreeIntervalsPartitionThePeriodAndAddUpToIt) {
    const SingleShunt shunt = make();
    double worst_span = 0.0;
    for (int k = 0; k <= 20; ++k) {
        for (int j = 0; j <= 20; ++j) {
            const float dmax = static_cast<float>(k) / 20.0F;
            const float dmin = static_cast<float>(j) / 20.0F * dmax;
            const float dmid = 0.5F * (dmax + dmin);
            const ShuntWindow w = shunt.classify(Uvw<float>{dmax, dmid, dmin});
            ASSERT_EQ(ErrorCode::Ok, w.status);
            const double span = static_cast<double>(w.times.t_active1_s + w.times.t_active2_s + w.times.t_zero_s);
            worst_span = std::max(worst_span, std::fabs(span - static_cast<double>(kPeriodS)));
            EXPECT_GE(w.times.t_active1_s, 0.0F);
            EXPECT_GE(w.times.t_active2_s, 0.0F);
            EXPECT_GE(w.times.t_zero_s, 0.0F);
        }
    }
    EXPECT_LT(worst_span, 1.0e-10) << "T1 + T2 + T0 is the period, not approximately the period";
}

// The window, and the refusal's sharp edge: the verdict must flip exactly at `window == needed`, which is the only place a
// rule of this kind can be tested without arguing about tolerances. Bisected on the duty, not on the code.
// 窗口，以及拒绝的那条锐利边界：结论必须在 `window == needed` 处翻转——这是这类规则唯一能不靠争论容差被检验的地方。在
// 占空比上二分，不是在代码上。
TEST(SingleShunt, TheVerdictFlipsExactlyWhereTheWindowEqualsWhatIsNeeded) {
    const SingleShunt shunt = make();
    // A symmetric request: dmid is the midpoint, so both active intervals are equal and the window is (dmax-dmin)*T/4.
    const auto window_of = [&shunt](float spread) -> ShuntWindow {
        const float dmax = 0.5F + (0.5F * spread);
        const float dmin = 0.5F - (0.5F * spread);
        return shunt.classify(Uvw<float>{dmax, 0.5F, dmin});
    };
    double lo = 0.0;
    double hi = 1.0;
    for (int k = 0; k < 80; ++k) {
        const double mid = 0.5 * (lo + hi);
        if (window_of(static_cast<float>(mid)).verdict == ShuntVerdict::Measurable) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    const ShuntWindow at = window_of(static_cast<float>(hi));
    const ShuntWindow below = window_of(static_cast<float>(lo));
    std::printf("\n[shunt] window rule: needed %.3e s | measurable from spread %.6f (window %.3e s) | below it %.6f "
                "(window %.3e s, verdict %d)\n",
                static_cast<double>(shunt.needed_s()), hi, static_cast<double>(at.window_s), lo,
                static_cast<double>(below.window_s), static_cast<int>(below.verdict));

    EXPECT_EQ(ShuntVerdict::Measurable, at.verdict);
    EXPECT_EQ(ShuntVerdict::WindowTooNarrow, below.verdict);
    // The derivation, not a fit: the window is (dmax-dmin)*T/4, so the crossing is at spread = 4*needed/T.
    const double law = 4.0 * static_cast<double>(shunt.needed_s()) / static_cast<double>(kPeriodS);
    EXPECT_NEAR(law, hi, 2.0e-6) << "the boundary is where the half-interval equals settle + sample";
    EXPECT_NEAR(static_cast<double>(shunt.needed_s()), static_cast<double>(at.window_s), 1.0e-9);
}

// The refusal is the point of the batch: outside the window there is no answer, and the type says so instead of inventing
// one. A caller that reads only the numbers sees zeros -- which is why `valid` exists and why this case exists.
// 拒绝是本批次的要点：窗口之外没有答案，类型就照实说而不是编一个。只看数字的调用方会看到零——这正是 `valid` 存在的
// 原因，也正是这条用例存在的原因。
TEST(SingleShunt, OutsideTheWindowThereIsNoAnswerAndTheTypeSaysSo) {
    const SingleShunt shunt = make();
    // Almost a pure active vector: the second interval is 0.01 % of the period, far below the 1.5 us that is needed.
    const Uvw<float> duty{0.900F, 0.850F, 0.849990F};
    const ShuntWindow w = shunt.classify(duty);
    const ShuntCurrents out = shunt.reconstruct(duty, 4.0F, -3.9F);
    std::printf("[shunt] near a boundary: t1 %.3e s, t2 %.3e s, window %.3e s, verdict %d | reconstruct valid %d "
                "(i = %.1f, %.1f, %.1f A)\n",
                static_cast<double>(w.times.t_active1_s), static_cast<double>(w.times.t_active2_s),
                static_cast<double>(w.window_s), static_cast<int>(w.verdict), static_cast<int>(out.valid),
                static_cast<double>(out.i_uvw.u), static_cast<double>(out.i_uvw.v), static_cast<double>(out.i_uvw.w));

    ASSERT_EQ(ErrorCode::Ok, out.status);
    EXPECT_EQ(ShuntVerdict::WindowTooNarrow, w.verdict);
    EXPECT_FALSE(out.valid);
    EXPECT_EQ(0.0F, out.i_uvw.u);
    EXPECT_EQ(0.0F, out.i_uvw.v);
    EXPECT_EQ(0.0F, out.i_uvw.w);

    // And the pure zero vector is a *different* answer: nothing is flowing, so there is nothing to miss.
    // 而纯零矢量是**另一个**答案：没有电流在流，也就没有东西被漏掉。
    EXPECT_EQ(ShuntVerdict::NoActiveVector, shunt.classify(Uvw<float>{0.5F, 0.5F, 0.5F}).verdict);
}

// The algebra, against the machine it is supposed to describe: a balanced star, whose currents are exactly what two
// switching-pattern readings contain. The reconstruction is exact here -- so the case also has to carry the falsifier,
// because "exact" is what a wrong assignment looks like when you only print the answer.
// 代数，对着它要描述的机器检验：一台平衡星形，它的电流正是两个开关图样读数所含的信息。这里重构是精确的——所以这条用例
// 还必须带上否证器，因为「精确」也正是分配写错时、只看结果会看到的样子。
TEST(SingleShunt, TwoReadingsRecoverThreeCurrentsAndSwappingThemDoesNot) {
    const SingleShunt shunt = make();
    double worst = 0.0;
    double worst_swapped = 0.0;
    for (int k = 0; k < 24; ++k) {
        const double angle = (2.0 * M_PI * static_cast<double>(k)) / 24.0;
        const Uvw<float> truth = balanced(angle, 3.5);
        // A duty triple in the middle of its sector, where both intervals are wide: the measurement case, not the edge case.
        const float spread = 0.20F;
        const Uvw<float> duty{0.5F + spread, 0.5F + (0.25F * spread), 0.5F - spread};
        // The shunt sees the largest phase alone, then the two largest (i.e. minus the smallest).
        const float i_dc1 = truth.u;
        const float i_dc2 = -truth.w;
        const ShuntCurrents good = shunt.reconstruct(duty, i_dc1, i_dc2);
        ASSERT_TRUE(good.valid);
        ASSERT_EQ(ErrorCode::Ok, good.status);
        const double err = std::max({std::fabs(static_cast<double>(good.i_uvw.u) - truth.u),
                                     std::fabs(static_cast<double>(good.i_uvw.v) - truth.v),
                                     std::fabs(static_cast<double>(good.i_uvw.w) - truth.w)});
        worst = std::max(worst, err);
        // The same two readings, assigned the other way round: a defect this file must be able to see.
        // 同样两个读数，反过来分配：本文件必须看得见的缺陷。
        const ShuntCurrents swapped = shunt.reconstruct(duty, i_dc2, i_dc1);
        ASSERT_TRUE(swapped.valid);
        worst_swapped = std::max(worst_swapped, std::fabs(static_cast<double>(swapped.i_uvw.u) - truth.u));
        // The star constraint survives either way, which is exactly why it is not a check on the assignment.
        // 星形约束在两种分配下都成立，这正是它不能用来检查分配的原因。
        EXPECT_NEAR(0.0, static_cast<double>(good.i_uvw.u + good.i_uvw.v + good.i_uvw.w), 1.0e-6);
    }
    std::printf("[shunt] reconstruction error over 24 angles: %.3e A | with the readings swapped: %.3e A\n", worst,
                worst_swapped);
    EXPECT_LT(worst, 1.0e-6);
    EXPECT_GT(worst_swapped, 1.0); // the falsifier: a wrong assignment is wrong by amperes, not by rounding
}

// The error structure, measured rather than assumed: the middle phase is a **difference** of the two readings, so its
// noise is sqrt(2) times theirs. A design that writes one bound and uses it for all three is 41 % optimistic on the
// phase it matters most for.
// 误差结构，量出来而不是假定：中间那一相是两个读数之**差**，噪声是它们的 sqrt(2) 倍。只写一条界、三相共用的设计，
// 在它最要紧的那一相上乐观 41 %。
TEST(SingleShunt, TheMiddlePhasesNoiseIsRootTwoTimesTheOuterOnes) {
    const SingleShunt shunt = make();
    constexpr int kSamples = 20000;
    constexpr float kNoiseA = 0.05F; // uniform in [-50 mA, 50 mA]: sigma = 0.0289 A
    const double sigma = static_cast<double>(kNoiseA) / std::sqrt(3.0);

    std::uint32_t state = 12345U;
    double sum_max = 0.0;
    double sum_mid = 0.0;
    int counted = 0;
    const Uvw<float> duty{0.70F, 0.55F, 0.40F};
    for (int k = 0; k < kSamples; ++k) {
        const Uvw<float> truth = balanced(0.4, 3.0);
        const float i_dc1 = truth.u + next_noise(state, kNoiseA);
        const float i_dc2 = -truth.w + next_noise(state, kNoiseA);
        const ShuntCurrents out = shunt.reconstruct(duty, i_dc1, i_dc2);
        ASSERT_TRUE(out.valid);
        const double e_max = static_cast<double>(out.i_uvw.u) - truth.u;
        const double e_mid = static_cast<double>(out.i_uvw.v) - truth.v;
        sum_max += e_max * e_max;
        sum_mid += e_mid * e_mid;
        ++counted;
    }
    const double rms_max = std::sqrt(sum_max / counted);
    const double rms_mid = std::sqrt(sum_mid / counted);
    std::printf("[shunt] noise: sigma %.4f A | largest-phase rms %.4f A | middle-phase rms %.4f A | ratio %.4f "
                "(sqrt(2) = 1.4142)\n",
                sigma, rms_max, rms_mid, rms_mid / rms_max);

    EXPECT_NEAR(1.0, rms_max / sigma, 0.10) << "the outer phases carry one reading each";
    EXPECT_NEAR(std::sqrt(2.0), rms_mid / rms_max, 0.10) << "the middle phase is the difference of the two readings";
    // The two bounds that a caller should actually write: 3 sigma for the outer phases, 3*sqrt(2) sigma for the middle one.
    EXPECT_LT(rms_max, 3.0 * sigma);
    EXPECT_LT(rms_mid, 3.0 * std::sqrt(2.0) * sigma);
}

// What the topology costs, counted rather than argued -- and the first version of this case had the answer **backwards**.
// I expected the blind spot at the voltage limit (where an active interval collapses at each sector boundary) and asserted
// that most of the circle would be unreadable there. The measurement says the opposite: readability **rises** with
// magnitude -- 32 % of the circle at 5 % of the bus, 84 % at 20 %, 94 % at the linear-range limit -- and the real blind
// spot is **low modulation**, where both active intervals are short together. That is the region a drive starts in. The
// assertion now states the measured direction, and the threshold it crosses is derived from the timings rather than
// fitted (and that derivation was wrong the first time too: I had written `4*needed/T`, the bisection said 0.0346 where
// the formula said 0.06, because `(T1+T2)/T` is `sqrt(3)*|v|` per unit and not `|v|`).
// 这个拓扑的代价，数出来而不是争出来——而这条用例的第一版把答案写**反**了。我以为盲区在电压极限处（那里每个扇区边界上都有
// 一个有效区间塌掉），于是断言整圈**大部分**读不出来。测量说的是反面：可读比例随幅值**上升**，0.2 倍内切圆处 71 %、极限处
// 94 %，而真正的盲区是**低调制**——两个有效区间一起变短的地方。那正是驱动起步所在的区域。现在的断言写的是量到的方向，而
// 它跨过的那条阈值由时序推出，不是拟合出来的（这条推导第一次也错了：我写的是 `4·needed/T`，二分给出 0.0346 而公式说
// 0.06，因为归一化后 `(T1+T2)/T` 是 `√3·|v|` 而不是 `|v|`）。
TEST(SingleShunt, TheBlindSpotIsLowModulationNotTheVoltageLimit) {
    const SingleShunt shunt = make();
    const double inscribed = 1.0 / std::sqrt(3.0); // per-unit v_bus, the linear-range radius
    const auto fraction_readable = [&](double per_unit) {
        int ok = 0;
        constexpr int kAngles = 3600;
        for (int k = 0; k < kAngles; ++k) {
            const double angle = (2.0 * M_PI * static_cast<double>(k)) / kAngles;
            // The duties a midpoint-injected modulator produces, built from the definition so the scan does not depend on the
            // modulator's internals: phase voltage minus the common-mode offset of the extremes, plus one half.
            // 中点注入调制器产生的占空比，按定义构造，于是这次扫描不依赖调制器内部：相电压减去极值的共模偏置，再加一半。
            double v[3] = {};
            for (int p = 0; p < 3; ++p) {
                v[p] = per_unit * std::cos(angle - (2.0 * M_PI * static_cast<double>(p)) / 3.0);
            }
            const double hi = std::max({v[0], v[1], v[2]});
            const double lo = std::min({v[0], v[1], v[2]});
            const double offset = 0.5 * (hi + lo);
            const ShuntWindow w = shunt.classify(Uvw<float>{static_cast<float>(0.5 + (v[0] - offset)),
                                                            static_cast<float>(0.5 + (v[1] - offset)),
                                                            static_cast<float>(0.5 + (v[2] - offset))});
            if (w.verdict == ShuntVerdict::Measurable) {
                ++ok;
            }
        }
        return static_cast<double>(ok) / kAngles;
    };

    std::printf("\n[shunt] readable fraction of the circle, needed %.1f us of a %.0f us period:\n",
                static_cast<double>(shunt.needed_s()) * 1.0e6, static_cast<double>(kPeriodS) * 1.0e6);
    double previous = 0.0;
    for (const double per_unit : {0.01, 0.03, 0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.577}) {
        const double f = fraction_readable(per_unit);
        std::printf("[shunt]   |v|/v_bus = %.3f -> %5.1f %% of the circle\n", per_unit, 100.0 * f);
        // The measured direction, which is the opposite of the assumption the first version of this case encoded: more
        // magnitude never costs measurability, it buys it.
        // 量到的方向，正是这条用例第一版所编码的假设的反面：幅值越大不会牺牲可读性，而是换来可读性。
        EXPECT_GE(f, previous - 1.0e-9) << "more magnitude cannot cost measurability, |v|/v_bus = " << per_unit;
        previous = f;
    }

    // The threshold, derived: the best case is a sector centre, where the two active intervals are equal, so the window is
    // `(T1+T2)/4`, and for space-vector modulation `(T1+T2)/T = sqrt(3)*(|v|/v_bus)`. The blind disc therefore ends at
    // `|v|/v_bus = 4*needed/(sqrt(3)*T)` -- the sqrt(3) is the whole reason the first version of this line was wrong by 73 %.
    // Bisected on the magnitude, then compared with the formula: a fit would only agree with itself.
    // 阈值，推出来的：最好情形是扇区中心，两个有效区间相等，于是窗口是 `(T1+T2)/4`；而空间矢量调制下
    // `(T1+T2)/T = √3·(|v|/v_bus)`。所以盲区圆盘止于 `|v|/v_bus = 4·needed/(√3·T)`——那个 √3 正是这一行第一版差 73 % 的
    // 全部原因。在幅值上二分，再与公式比较：拟合只会跟自己一致。
    double lo_m = 0.0;
    double hi_m = 1.0;
    for (int k = 0; k < 60; ++k) {
        const double mid = 0.5 * (lo_m + hi_m);
        if (fraction_readable(mid) > 0.0) {
            hi_m = mid;
        } else {
            lo_m = mid;
        }
    }
    const double law = 4.0 * static_cast<double>(shunt.needed_s()) / (std::sqrt(3.0) * static_cast<double>(kPeriodS));
    std::printf("[shunt] blind below |v|/v_bus = %.5f (bisected) | law 4*needed/(sqrt(3)*T) = %.5f | at the limit: "
                "%.1f %% readable\n",
                hi_m, law, 100.0 * fraction_readable(inscribed));
    EXPECT_NEAR(law, hi_m, 2.0e-4) << "the blind disc ends where the best-case window equals settle + sample";

    // And the conclusion the batch asked for, written as an assertion so it cannot be quietly dropped: the region this
    // topology cannot read is the region a drive starts in, so a single shunt has to be paired with a phase-shifted sampling
    // scheme or replaced by two shunts. The criterion is not bent to hide that.
    // 本批次要求的结论，写成断言以免日后被悄悄丢掉：这个拓扑读不出来的区域，正是驱动起步所在的区域，所以单电阻必须配相移采样
    // 方案，或者换成双电阻。判据不会为了掩盖这一点被掰弯。
    EXPECT_EQ(0.0, fraction_readable(0.03)) << "below the blind threshold nothing at all is readable";
    EXPECT_LT(fraction_readable(0.05), 0.45)
        << "at 5 % of the bus a third of the circle is readable: this is standstill";
    EXPECT_GT(fraction_readable(inscribed), 0.90) << "and at the voltage limit the wedges are the only loss";
}
