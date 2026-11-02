// The frequency response of the speed loop, measured rather than estimated -- and this file exists because the
// step-response criterion cannot see a 20 % gain error while this one can. That claim was measured, not assumed: see
// A-11 in the foundation log.
// 速度环的频率响应，是测出来的而不是估计出来的——本文件存在的理由，正是阶跃判据看不见 20% 的增益错误，而这个看得见。
// 这条断言是量出来的，不是假设的：见地基台账 A-11。

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/sinusoid_fit.hpp>

#include "sil_bench.hpp"


namespace {

constexpr float kPi = 3.14159265358979F;
constexpr float kAmplitudeRadS = 10.0F; ///< [en] small enough that the current limit never engages. [zh] 小到不触发限流。
constexpr float kMinus3Db = 0.70710678F;
/**
 * @brief [en] a tolerance nothing passes: used once to print the real numbers.
 * @brief [zh] 没人能过的容差：用来把真实数字打印一次。
 */
constexpr float kFailing = 1.0e-5F;


/**
 * @brief One measured point of the frequency response.
 */
struct Point final {
    float gain;
    float lag_rad;
    bool ok;
};


/**
 * @brief [en] Drive the reference with `A*cos(w t)` and fit both the command and the response over the same ticks.
 *
 * @brief [zh] 用 `A*cos(w t)` 驱动给定，并在同一批节拍上同时拟合给定与响应。
 *
 *     The command is fitted out of a recorded array rather than assumed analytically: both windows then share one time
 *     origin, so the fitted phases subtract directly and the answer does not depend on bookkeeping about where the
 *     recording started. 给定从录下来的数组里拟合，而不是解析地假设：两侧因此共享同一个时间原点，拟合相位可以直接相减，
 *     答案也就不再依赖 "录制从哪儿开始"的记账。
 */
auto measure(const foc::sil::BenchConfig &base, float omega_rad_s) -> Point {
    foc::sil::Bench bench(base, foc::sil::nominal_plant_params());
    bench.motor().set_speed(0.0F);
    if (bench.init_and_start() != foc::ErrorCode::Ok) {
        return Point{0.0F, 0.0F, false};
    }

    const float dt = base.dt_ctrl_s;
    const float period_s = 2.0F * kPi / omega_rad_s;
    float t = 0.0F;

    // Six periods of settling: this measures the steady-state response, not the transient.
    // 先走六个周期：测的是稳态响应，不是暂态。
    const int settle_ticks = static_cast<int>((6.0F * period_s) / dt);
    for (int i = 0; i < settle_ticks; ++i) {
        bench.motor().set_speed(kAmplitudeRadS * std::cos(omega_rad_s * t));
        bench.tick_once();
        t += dt;
    }

    std::vector<float> command;
    std::vector<float> response;
    const int record_ticks = static_cast<int>((4.0F * period_s) / dt);
    command.reserve(static_cast<std::size_t>(record_ticks));
    response.reserve(static_cast<std::size_t>(record_ticks));
    for (int i = 0; i < record_ticks; ++i) {
        const float c = kAmplitudeRadS * std::cos(omega_rad_s * t);
        bench.motor().set_speed(c);
        bench.tick_once();
        command.push_back(c);
        response.push_back(bench.sample().speed);
        t += dt;
    }

    const foc::core::SinusoidFit fed = foc::core::sinusoid_fit(command.data(), command.size(), dt, omega_rad_s);
    const foc::core::SinusoidFit out = foc::core::sinusoid_fit(response.data(), response.size(), dt, omega_rad_s);
    if ((fed.status != foc::ErrorCode::Ok) || (out.status != foc::ErrorCode::Ok)) {
        return Point{0.0F, 0.0F, false};
    }
    return Point{out.amplitude / fed.amplitude, fed.phase_rad - out.phase_rad, true};
}


/**
 * @brief The -3 dB crossing, found by bisection so that the answer does not assume the shape it is measuring. The first draft interpolated in log-log space between 0.5 wb and 2 wb and reported 180 rad/s for a loop whose per-point first-order equivalents were 236.4, 235.8, 236.4 and 238.3 rad/s -- a 28 % error made entirely of interpolation. A first-order magnitude is only straight in log-log space well above its corner, and a bisection needs no such assumption. 用二分找 -3 dB 交点，使答案不假设它正在测量的形状。第一版在 0.5 wb 与 2 wb 之间做对数插值，对一个逐点一阶等效带宽分别为 236.4、235.8、236.4、238.3 rad/s 的环路报出 180 rad/s——28% 的误差完全来自插值。一阶幅值只在其拐点以上很远才是对数直线，而二分不需要这个假设。
 */
auto minus3db(const foc::sil::BenchConfig &cfg, float omega_low, float omega_high) -> float {
    float lo = omega_low;
    float hi = omega_high;
    for (int i = 0; i < 8; ++i) {
        const float mid = std::sqrt(lo * hi);
        const Point p = measure(cfg, mid);
        if (!p.ok) {
            return 0.0F;
        }
        if (p.gain > kMinus3Db) {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    return std::sqrt(lo * hi);
}

} // namespace



TEST(BodeSweep, TheLoopPassesLowFrequenciesAndRollsOffMonotonically) {
    // What is true of *any* stable low-pass, asserted on the loop itself: unity gain well below the corner, and a
    // magnitude that never rises as the frequency goes up. The shape in between is not asserted, because this bench's
    // loop is not first-order -- its per-point first-order equivalents fall from 269.5 to 188.8 rad/s across the sweep
    // -- and a test that demanded first-order shape here would be testing the plant, not the loop.
    // 任何稳定低通都成立的性质，在环路自身上断言：拐点以下远带的单位增益，以及随频率单调不升的幅值。中间的形状不做
    // 断言，因为本台架的环路并不是一阶——其逐点一阶等效带宽在扫频范围内从 269.5 落到 188.8 rad/s——在这里要求一阶形状
    // 测的就是 plant，不是环路。
    const foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
    const float wb = cfg.omega_speed_rad_s;

    float previous = 2.0F;
    for (const float multiple : {0.1F, 0.5F, 1.0F, 2.0F}) {
        const Point p = measure(cfg, multiple * wb);
        ASSERT_TRUE(p.ok) << "multiple " << multiple;
        std::printf("SWEEP %.2f wb (%.1f rad/s): gain %.6f lag %+.4f rad\n", multiple, multiple * wb, p.gain, p.lag_rad);
        EXPECT_LT(p.gain, previous + 1.0e-4F) << "the magnitude rose between " << previous << " and " << p.gain;
        previous = p.gain;
        // Lag grows with frequency and is positive: the response trails the command.
        // 滞后随频率增长且为正：响应落在给定之后。
        EXPECT_GT(p.lag_rad, 0.0F) << "multiple " << multiple;
    }

    const Point passband = measure(cfg, 0.1F * wb);
    ASSERT_TRUE(passband.ok);
    EXPECT_NEAR(1.0F, passband.gain, 0.02F) << "the loop does not pass a tenth of its bandwidth flat";
}


TEST(BodeSweep, TheThreeDecibelPointIsTheBandwidthTheGainLawWasAskedFor) {
    const foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
    const float wb = cfg.omega_speed_rad_s;

    const Point half = measure(cfg, 0.5F * wb);
    const Point twice = measure(cfg, 2.0F * wb);
    ASSERT_TRUE(half.ok);
    ASSERT_TRUE(twice.ok);
    ASSERT_GT(half.gain, kMinus3Db) << "the sweep does not bracket -3 dB below: " << half.gain;
    ASSERT_LT(twice.gain, kMinus3Db) << "the sweep does not bracket -3 dB above: " << twice.gain;

    const float crossing = minus3db(cfg, 0.5F * wb, 2.0F * wb);
    std::printf("SWEEP -3 dB at %.2f rad/s against %.2f requested (%+.2f%%)\n", crossing, wb,
                100.0F * ((crossing / wb) - 1.0F));
    // Measured at 234.91 rad/s, -6.04 %. The band is the plan's 10 %, and it is a *sharp* criterion in a way the
    // step-response one is not: a 20 % error in the speed loop's proportional gain moves this number by about 20 %
    // (verified by perturbing it), while it moved the step fit by 9.9 %, inside its own band.
    // 实测 234.91 rad/s，即 -6.04%。带是计划的 10%，而且它比阶跃判据**锐利**：速度环比例增益 20% 的错误会让这个数
    // 移动约 20%（经摄动验证），而它只让阶跃拟合移动 9.9%，仍在后者自己的带内。
    EXPECT_NEAR(1.0F, crossing / wb, 0.10F) << "the -3 dB point is not where the gain law was asked to put it";
}


namespace {

constexpr float kFitTwoPi = 6.28318530717959F;

/**
 * @brief [en] A tone of known amplitude, phase and offset, sampled the way the sweep samples it.
 * @brief [zh] 一个幅值、相位、偏置都已知的音，按扫频的采样方式取样。
 */
auto tone(std::vector<float> &y, std::size_t n, float dt, float omega, float amp, float phase, float dc) -> void {
    y.resize(n);
    for (std::size_t i = 0U; i < n; ++i) {
        y[i] = static_cast<float>(dc + (amp * std::cos((omega * dt * static_cast<float>(i)) + phase)));
    }
}

/**
 * @brief [en] The estimator the header claims to be better than: one rectangular-window DFT bin, `2/N*|bin|`.
 *
 * @brief [zh] 头文件声称比它更好的那个估计器：矩形窗只取一个 DFT bin，`2/N·|bin|`。
 *
 *     This is not a strawman and it is not a reimplementation of the thing under test: it is the *other* way to do this
 *     job, it is exact on a whole number of cycles, and the header's claim is precisely that it is biased when the
 *     window is not one. Writing it down is what turns that sentence into a measurement. 这不是稻草人，也不是被测对象的
 *     另一种写法：它是做这件事的**另一种**方式，在整周期窗口上是精确的，而头文件的主张恰恰是"窗口不是整周期时它有偏"。
 *     把它写下来，那句话才变成一次测量。
 */
auto bin_amplitude(const std::vector<float> &y, float dt, float omega) -> float {
    double c = 0.0;
    double s = 0.0;
    for (std::size_t i = 0U; i < y.size(); ++i) {
        const double t = static_cast<double>(dt) * static_cast<double>(i);
        c += static_cast<double>(y[i]) * std::cos(static_cast<double>(omega) * t);
        s += static_cast<double>(y[i]) * std::sin(static_cast<double>(omega) * t);
    }
    return static_cast<float>(2.0 * std::sqrt((c * c) + (s * s)) / static_cast<double>(y.size()));
}

/**
 * @brief [en] The same projection with the `c0` column dropped, i.e. no DC term.
 *
 * @brief [zh] 同一个投影，但去掉 `c0` 列，也就是没有直流项。
 *
 *     Exists only so the "a DC offset is harmless because the fit carries a constant column" claim has something that
 *     would fail if the column were not there. Two normal equations instead of three. 只为让"直流偏置无害，因为拟合带常
 *     数项"这条主张有一个"若没有那一列就会失败"的对照。两个正规方程而不是三个。
 */
auto amplitude_without_dc(const std::vector<float> &y, float dt, float omega) -> float {
    double cc = 0.0;
    double ss = 0.0;
    double yc = 0.0;
    double ys = 0.0;
    for (std::size_t i = 0U; i < y.size(); ++i) {
        const double t = static_cast<double>(dt) * static_cast<double>(i);
        const double c = std::cos(static_cast<double>(omega) * t);
        const double s = std::sin(static_cast<double>(omega) * t);
        const double v = static_cast<double>(y[i]);
        cc += c * c;
        ss += s * s;
        yc += v * c;
        ys += v * s;
    }
    const double c1 = (cc > 0.0) ? (yc / cc) : 0.0;
    const double c2 = (ss > 0.0) ? (ys / ss) : 0.0;
    return static_cast<float>(std::sqrt((c1 * c1) + (c2 * c2)));
}

} // namespace


TEST(SinusoidFit, AWholeCycleWindowIsRecoveredByBothEstimators) {
    // Baseline for the comparison below. On a whole number of cycles the plain bin is *exact*, so a large error
    // there would mean the comparison estimator is broken rather than the fit being good.
    // 下面那个比较的基线。整周期上单 bin 是**精确**的，所以若那里出现大误差，说明比较用的估计器坏了，而不是拟合好。
    const float dt = 1.0e-4F;
    const float omega = kFitTwoPi * 50.0F;
    const float amp = 3.0F;
    std::vector<float> y;
    tone(y, 600U, dt, omega, amp, 0.7F, 0.0F); // 600 samples * 1e-4 s = 0.06 s = exactly 3 cycles

    const foc::core::SinusoidFit fit = foc::core::sinusoid_fit(y.data(), y.size(), dt, omega);
    ASSERT_EQ(foc::ErrorCode::Ok, fit.status);
    const float bin = bin_amplitude(y, dt, omega);
    std::printf("\n[L3] whole cycles: fit %.9f, bin %.9f (asked %.1f)\n", static_cast<double>(fit.amplitude),
                static_cast<double>(bin), static_cast<double>(amp));
    EXPECT_NEAR(amp, fit.amplitude, 1.0e-5F);
    EXPECT_NEAR(amp, bin, 1.0e-5F) << "the comparison estimator has to be exact here or it is not a fair comparison";
}


TEST(SinusoidFit, TheFitIsUnbiasedOnAWindowThatIsNotAWholeNumberOfCycles) {
    // The header's own claim, made measurable: "exact for a window that contains whole cycles and unbiased for one
    // that does not, which a plain DFT bin would not be".
    //
    // Criterion, written before the run: the fit's error must stay under 1 % on a 3.4-cycle window, the bin's must
    // exceed the fit's by at least 3x, and the same comparison at a whole number of cycles must collapse to a ratio
    // near 1. If the measured ratio is under 3 the sentence in the header is wrong and gets corrected instead of the
    // threshold getting widened.
    // 头文件自己的主张，此处把它变成可测的："整周期精确、非整周期无偏，而单 bin 做不到"。
    //
    // 判据写在跑之前：3.4 周期窗口上拟合误差必须低于 1%，单 bin 的误差必须至少是拟合的 3 倍，且同一比较在整周期上
    // 必须塌回接近 1 的比值。若实测比值低于 3，说明头文件那句话是错的，那就改那句话而不是把阈值放宽。
    const float dt = 1.0e-4F;
    const float omega = kFitTwoPi * 50.0F;
    const float amp = 3.0F;

    const auto error_ratio = [&](std::size_t n) {
        std::vector<float> y;
        tone(y, n, dt, omega, amp, 0.7F, 0.0F);
        const foc::core::SinusoidFit fit = foc::core::sinusoid_fit(y.data(), y.size(), dt, omega);
        EXPECT_EQ(foc::ErrorCode::Ok, fit.status) << "n " << n;
        const float fit_err = std::fabs((fit.amplitude / amp) - 1.0F);
        const float bin_err = std::fabs((bin_amplitude(y, dt, omega) / amp) - 1.0F);
        return std::pair<float, float>{fit_err, bin_err};
    };

    std::printf("\n[L3] window sensitivity of the fit against a single DFT bin\n");
    std::printf("        %6s %8s | %10s %10s %8s\n", "n", "cycles", "fit error", "bin error", "ratio");
    for (const std::size_t n : {400U, 500U, 600U, 640U, 680U, 700U}) {
        const auto [fit_err, bin_err] = error_ratio(n);
        const float cycles = static_cast<float>(n) * dt * omega / kFitTwoPi;
        // The ratio is only meaningful while the fit's error is above its own rounding floor; below that the
        // quotient is a division by noise and prints millions.
        // 比值只在拟合误差高于它自己的舍入底噪时有意义；低于那个底噪时商就是除噪，会印出百万级。
        if (fit_err > 1.0e-5F) {
            std::printf("        %6zu %8.2f | %9.4f%% %9.4f%% %8.1f\n", n, static_cast<double>(cycles),
                        100.0 * static_cast<double>(fit_err), 100.0 * static_cast<double>(bin_err),
                        static_cast<double>(bin_err / fit_err));
        } else {
            std::printf("        %6zu %8.2f | %9.4f%% %9.4f%% %8s\n", n, static_cast<double>(cycles),
                        100.0 * static_cast<double>(fit_err), 100.0 * static_cast<double>(bin_err), "-");
        }
        EXPECT_LT(fit_err, 0.01F) << "the fit is biased on a " << cycles << "-cycle window";
    }

    const auto [whole_fit, whole_bin] = error_ratio(600U);
    const auto [part_fit, part_bin] = error_ratio(680U);
    ASSERT_GT(part_fit, 0.0F);
    std::printf("        whole: fit %.3e, bin %.3e; partial: fit %.3e, bin %.3e\n", static_cast<double>(whole_fit),
                static_cast<double>(whole_bin), static_cast<double>(part_fit), static_cast<double>(part_bin));
    EXPECT_LT(whole_bin, 5.0F * whole_fit + 1.0e-7F)
        << "on whole cycles the two estimators must agree, or the difference below is not about the window";
    EXPECT_GT(part_bin, 3.0F * part_fit) << "the header claims the bin is biased here and the fit is not -- it is not";
}


TEST(SinusoidFit, TheSensitivityToBeingAskedAboutTheWrongFrequencyIsMeasured) {
    // The realistic way this function is wrong in service: the tone is not exactly at the frequency you ask about,
    // because the drive, the clock or the machine moved.
    //
    // The first draft of this case asserted that the error grows *monotonically* with the offset. The measurement
    // refused: +0.5 % gives 0.095 % while -0.5 % gives 0.218 %, and +1 % gives less than either, because the error
    // depends on how much of a cycle the mismatch accumulated before the window ended, not just on its size. So the
    // criterion is the analytic envelope instead, which is a *prediction* rather than a shape: the projection of a
    // tone at `w+dw` onto the basis at `w` is attenuated by `|sinc(dw*T/2)|`, and the measured error has to land
    // within one percentage point of it at every offset.
    // 这个函数在实机上出错的最现实方式：音并不正好在你所问的频率上——驱动器、时钟或机器动了。
    //
    // 本用例第一版断言误差随偏移**单调**增长。测量拒绝了这个说法：+0.5% 给 0.095%，而 -0.5% 给 0.218%，+1% 比两者都小——
    // 因为误差取决于窗口结束前失配累积了多少个周期，而不只是失配多大。所以判据换成解析包络，那是**预测**而不是形状：
    // `w+dw` 上的音向 `w` 的基函数投影时被 `|sinc(dw·T/2)|` 衰减，实测误差必须在每个偏移上都落在它 1 个百分点之内。
    const float dt = 1.0e-4F;
    const float omega = kFitTwoPi * 50.0F;
    const float amp = 3.0F;
    const std::size_t n = 680U; // 3.4 cycles at the nominal frequency
    const double window_s = static_cast<double>(n) * static_cast<double>(dt);

    std::printf("\n[L3] asking at the nominal frequency while the tone is elsewhere\n");
    std::printf("        %8s %12s %10s %12s %8s\n", "offset", "fit amplitude", "error", "sinc envelope", "gap");
    float at_five = 0.0F;
    for (const float offset : {0.0F, 0.005F, -0.005F, 0.01F, -0.01F, 0.02F, -0.02F, 0.05F}) {
        std::vector<float> y;
        tone(y, n, dt, omega * (1.0F + offset), amp, 0.7F, 0.0F);
        const foc::core::SinusoidFit fit = foc::core::sinusoid_fit(y.data(), y.size(), dt, omega);
        ASSERT_EQ(foc::ErrorCode::Ok, fit.status) << "offset " << offset;
        const float err = std::fabs((fit.amplitude / amp) - 1.0F);
        const double half = (static_cast<double>(offset) * static_cast<double>(omega) * window_s) / 2.0;
        const double envelope = (std::fabs(half) > 1.0e-12) ? std::fabs(std::sin(half) / half) : 1.0;
        const float predicted = static_cast<float>(std::fabs(envelope - 1.0));
        std::printf("        %+7.2f%% %12.6f %9.3f%% %11.3f%% %+7.3f%%\n", 100.0 * static_cast<double>(offset),
                    static_cast<double>(fit.amplitude), 100.0 * static_cast<double>(err),
                    100.0 * static_cast<double>(predicted), 100.0 * static_cast<double>(err - predicted));
        EXPECT_NEAR(err, predicted, 0.01F) << "offset " << offset << ": the sinc envelope is the model here";
        if (offset == 0.05F) {
            at_five = err;
        }
    }
    EXPECT_GT(at_five, 0.02F) << "a 5 % frequency error has to be visible, or the sweep cannot report its own";
}


TEST(SinusoidFit, TheConstantColumnIsWhatMakesADcOffsetHarmless) {
    // Both halves are needed: the claim, and a version of the estimator without the column that the claim would be
    // false about. A criterion that cannot tell them apart is decoration.
    // 两半都需要：主张本身，以及一个"若没有那一列、主张就不成立"的对照。分不开二者的判据只是装饰。
    const float dt = 1.0e-4F;
    const float omega = kFitTwoPi * 50.0F;
    const float amp = 3.0F;

    std::vector<float> clean;
    tone(clean, 680U, dt, omega, amp, 0.7F, 0.0F);
    std::vector<float> offset;
    tone(offset, 680U, dt, omega, amp, 0.7F, 5.0F); // +5 V on a 3 V tone

    const foc::core::SinusoidFit fit_clean = foc::core::sinusoid_fit(clean.data(), clean.size(), dt, omega);
    const foc::core::SinusoidFit fit_offset = foc::core::sinusoid_fit(offset.data(), offset.size(), dt, omega);
    ASSERT_EQ(foc::ErrorCode::Ok, fit_clean.status);
    ASSERT_EQ(foc::ErrorCode::Ok, fit_offset.status);

    const float shift = std::fabs((fit_offset.amplitude / fit_clean.amplitude) - 1.0F);
    const float naive_shift = std::fabs((amplitude_without_dc(offset, dt, omega) / amp) - 1.0F);
    std::printf("\n[L3] a +5 V offset on a 3 V tone: fit %.6f (%.4f%%), without the c0 column %.6f (%.1f%%)\n",
                static_cast<double>(fit_offset.amplitude), 100.0 * static_cast<double>(shift),
                static_cast<double>(amplitude_without_dc(offset, dt, omega)),
                100.0 * static_cast<double>(naive_shift));
    EXPECT_LT(shift, 0.005F) << "the c0 column is supposed to absorb the offset entirely";
    // The first draft asserted the naive estimator would be off by more than 50 %. It is off by 7.6 %: without a
    // constant column the offset only reaches the amplitude through the basis's own non-orthogonality over a
    // non-integer window, which is a few percent, not a catastrophe. The criterion is corrected to the size the
    // mechanism actually has, and the ratio to the fit is kept as the discriminating part.
    // 第一版断言朴素估计器会偏 50% 以上。它只偏 7.6%：没有常数项时，偏置只能经由基函数在非整周期窗口上的非正交性
    // 进入幅值，量级是几个百分点，不是灾难。判据改成该机理真实的量级，并把与拟合的比值保留为判别部分。
    EXPECT_GT(naive_shift, 0.05F) << "and without it the same offset has to be visibly wrong";
    EXPECT_GT(naive_shift, 20.0F * shift) << "...by a margin that leaves no doubt which one is doing the work";
}

