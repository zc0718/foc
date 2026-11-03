// The analysis layer's arithmetic, pinned against signals whose answer is known by hand -- and then against the speed
// loop's own gain law, which is where the measurement stops being self-referential.
// 分析层的算术，用"答案在手算里"的信号钉住——再钉到速度环自己的增益律上，这一步测量才不再是自说自话。

#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/step_metrics.hpp>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"


namespace {

constexpr float kDt = 1e-4F;



/**
 * @brief A first-order approach to `target` with time constant `tau`, sampled at `kDt`.
 */
auto first_order(float tau_s, float target, std::size_t n) -> std::vector<float> {
    std::vector<float> y(n, 0.0F);
    for (std::size_t i = 0U; i < n; ++i) {
        y[i] = target * (1.0F - std::exp(-static_cast<float>(i) * kDt / tau_s));
    }
    return y;
}

} // namespace



TEST(StepMetrics, AFittedDecayReportsTheTimeConstantThatWasUsed) {
    // The arithmetic must recover a number it was never told. Three time constants spanning a decade, because a
    // formula that is right at one sample rate can be wrong at another.
    // 算术必须复原一个它从未被告知的数。三个跨越一个数量级的时间常数，因为在一个采样率上正确的公式在另一个上可能错。
    for (const float tau : {0.0050F, 0.0100F, 0.0400F}) {
        const std::vector<float> y = first_order(tau, 100.0F, 2000U);
        const foc::core::StepMetrics m = foc::core::step_metrics(y.data(), y.size(), kDt, 100.0F, 2.0F);

        ASSERT_EQ(foc::ErrorCode::Ok, m.status) << "tau=" << tau;
        // 1 % is the discretisation budget, not a fudge: sampling a continuous exponential and fitting it by least
        // squares costs about half a percent at these rates, measured at 0.51 % for tau = 10 ms.
        // 1% 是离散化预算，不是含糊：把连续指数采样后做最小二乘，在这些采样率下大约损失半个百分点（τ=10 ms 实测 0.51%）。
        EXPECT_NEAR(1.0F, m.tau_s / tau, 0.01F) << "tau=" << tau;
        EXPECT_NEAR(1.0F / tau, m.bandwidth_rad_s, 0.01F / tau) << "tau=" << tau;
        // A first-order approach to within 2 % takes -ln(0.02) = 3.912 time constants.
        // 一阶响应进入 2% 带所需时间是 -ln(0.02) = 3.912 个时间常数。
        EXPECT_NEAR(3.912F * tau, m.settling_s, static_cast<float>(kDt)) << "tau=" << tau;
        EXPECT_FLOAT_EQ(0.0F, m.overshoot_fraction) << "tau=" << tau;
    }
}


TEST(StepMetrics, AnOvershootIsMeasuredAnalyticallyAndKeptOutOfTheFit) {
    // A second-order response with a known peak. Two claims at once: the overshoot number matches the closed form, and
    // the samples above the target are excluded from the fit -- the second is the one that matters, because including
    // them asks for `ln` of a negative number and the first draft of the fit was quietly wrong in exactly that way.
    // 已知峰值的二阶响应。同时两条断言：超调与闭式解一致，且越过目标的样本被排除在拟合之外——后一条才是要害，因为把
    // 它们纳入等于对负数取 ln，而拟合的第一版正是这样安静地错的。
    const float zeta = 0.4F;
    const float omega_n = 100.0F;
    const float omega_d = omega_n * std::sqrt(1.0F - (zeta * zeta));
    const std::size_t n = 2000U;
    std::vector<float> y(n, 0.0F);
    std::size_t first_crossing = n;
    for (std::size_t i = 0U; i < n; ++i) {
        const float t = static_cast<float>(i) * kDt;
        const float decay = std::exp(-zeta * omega_n * t);
        y[i] = 1.0F - (decay * ((std::cos(omega_d * t)) + ((zeta / std::sqrt(1.0F - (zeta * zeta))) * std::sin(omega_d * t))));
        if ((first_crossing == n) && (y[i] >= 1.0F)) {
            first_crossing = i;
        }
    }
    ASSERT_LT(first_crossing, n) << "the signal must overshoot for this case to mean anything";

    const foc::core::StepMetrics m = foc::core::step_metrics(y.data(), y.size(), kDt, 1.0F, 0.02F);
    ASSERT_EQ(foc::ErrorCode::Ok, m.status);

    // Peak of a second-order step: exp(-pi*zeta/sqrt(1-zeta^2)).
    // 二阶阶跃的峰值：exp(-πζ/√(1-ζ²))。
    const float expected_overshoot = std::exp(-3.14159265358979F * zeta / std::sqrt(1.0F - (zeta * zeta)));
    EXPECT_NEAR(expected_overshoot, m.overshoot_fraction, 0.01F);

    // The fit must equal the fit on the monotone prefix, bit for bit: proof that nothing above the target was used.
    // 拟合必须与"截到首次越过目标处"的前缀拟合逐位相同：这就是"目标之上的样本一个都没被采用"的证明。
    const foc::core::StepMetrics prefix =
        foc::core::step_metrics(y.data(), first_crossing, kDt, 1.0F, 0.02F);
    ASSERT_EQ(foc::ErrorCode::Ok, prefix.status);
    EXPECT_FLOAT_EQ(prefix.bandwidth_rad_s, m.bandwidth_rad_s);
    EXPECT_FLOAT_EQ(prefix.tau_s, m.tau_s);
}


TEST(StepMetrics, AResponseThatNeverSettlesSaysSoInsteadOfReturningZero) {
    // -1 and 0 are different answers: "never arrived" and "never left". A library that returned 0 for both would let a
    // runaway axis look like a perfectly settled one.
    // -1 与 0 是两个不同的答案："从未到达"与"从未离开"。把两者都返回 0 的库，会让一个跑飞的轴看起来完美稳定。
    std::vector<float> ramp(500U, 0.0F);
    for (std::size_t i = 0U; i < ramp.size(); ++i) {
        ramp[i] = static_cast<float>(i) * kDt;
    }
    const foc::core::StepMetrics m = foc::core::step_metrics(ramp.data(), ramp.size(), kDt, 100.0F, 0.02F);
    EXPECT_FLOAT_EQ(-1.0F, m.settling_s);

    const std::vector<float> flat(500U, 0.0F);
    const foc::core::StepMetrics inside = foc::core::step_metrics(flat.data(), flat.size(), kDt, 0.0F, 0.02F);
    EXPECT_FLOAT_EQ(0.0F, inside.settling_s) << "a window that equals its target starts settled";
}


TEST(StepMetrics, AWindowWithoutADecayReportsStateRatherThanAConfidentNumber) {
    // Three samples that jump straight to the target carry no decay. The bandwidth must be 0 *and* the status must say
    // why, while the settling numbers stay valid -- a response can settle without being fittable, and refusing to report
    // that would throw away the half that was measured.
    // 三个样本直接跳到目标，里面没有衰减。带宽必须为 0、状态必须说明原因，而稳定时间等数仍然有效——一段响应可以稳定却不可
    // 拟合，拒绝报告它就是丢掉了已经测到的那一半。
    const std::vector<float> jump{0.0F, 100.0F, 100.0F, 100.0F, 100.0F};
    const foc::core::StepMetrics m = foc::core::step_metrics(jump.data(), jump.size(), kDt, 100.0F, 2.0F);
    EXPECT_EQ(foc::ErrorCode::State, m.status);
    EXPECT_FLOAT_EQ(0.0F, m.bandwidth_rad_s);
    EXPECT_FLOAT_EQ(0.0F, m.tau_s);
    EXPECT_FLOAT_EQ(kDt, m.settling_s) << "the settling half is still measured: it settled one sample in";
}


TEST(StepMetrics, ArgumentsThatCannotDescribeAStepAreParam) {
    const std::vector<float> y = first_order(0.01F, 100.0F, 100U);

    EXPECT_EQ(foc::ErrorCode::Param, foc::core::step_metrics(nullptr, y.size(), kDt, 100.0F, 2.0F).status);
    EXPECT_EQ(foc::ErrorCode::Param, foc::core::step_metrics(y.data(), 2U, kDt, 100.0F, 2.0F).status);
    EXPECT_EQ(foc::ErrorCode::Param, foc::core::step_metrics(y.data(), y.size(), 0.0F, 100.0F, 2.0F).status);
    EXPECT_EQ(foc::ErrorCode::Param, foc::core::step_metrics(y.data(), y.size(), -kDt, 100.0F, 2.0F).status);
    EXPECT_EQ(foc::ErrorCode::Param, foc::core::step_metrics(y.data(), y.size(), kDt, 100.0F, -1.0F).status);

    // A window that never moves has no step in it, and `y[0] == target` is the only way to say so without a flag.
    // 完全不动的窗口里没有阶跃，而 `y[0] == target` 是唯一不需要额外标志就能说出这件事的方式。
    const std::vector<float> still(100U, 7.0F);
    EXPECT_EQ(foc::ErrorCode::Param, foc::core::step_metrics(still.data(), still.size(), kDt, 7.0F, 0.1F).status);
}


TEST(StepMetrics, TheSpeedLoopsMeasuredBandwidthIsTheOneTheGainLawAskedFor) {
    // The criterion this capability exists for. `speed_pi_config()` cancels the mechanical pole and leaves `wb / s`, a
    // first-order loop whose -3 dB point is the `omega_speed_rad_s` it was asked for. Fitting the loop's own trace and
    // comparing with the request therefore measures the gain law, not the fit.
    //
    // The step is deliberately small (10 rad/s): a large step saturates the current limit, and a saturated response is
    // not the linear loop the claim is about. The standard bench's speed bandwidth is 250 rad/s, so tau = 4 ms and the
    // 50 us control period gives 80 samples per time constant.
    //
    // 这个能力存在的理由。`speed_pi_config()` 抵消机械极点后留下 `wb / s`，一个 -3 dB 点等于所请求 `omega_speed_rad_s`
    // 的一阶环。因此拟合环路自己的轨迹并与请求比较，测的是增益律而不是拟合。
    //
    // 阶跃故意取小（10 rad/s）：大幅阶跃会打满电流限幅，而饱和的响应不是这条断言所指的线性环。标准台架速度带宽
    // 250 rad/s，即 τ = 4 ms，50 µs 的控制周期给出每个时间常数 80 个样本。
    const foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
    foc::sil::Bench bench(cfg, foc::sil::nominal_plant_params());
    bench.motor().set_speed(0.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());
    bench.run(2e-3F);

    const float target = 10.0F;
    bench.motor().set_speed(target);
    std::vector<float> speed;
    // The window starts *at the command*: `y[0]` must be the pre-step value, and the samples the bench yields are all
    // after it. Prepending the held zero is what makes `speed[i]` sit at exactly `i * dt` from the step, which the
    // metric's fit assumes -- without it the fitted origin is one period late and the bias shows up as bandwidth.
    // 窗口从**发令时刻**开始：`y[0]` 必须是阶跃前的值，而台架给出的样本全在其后。补上那个保持为零的样本，才使
    // `speed[i]` 正好落在距阶跃 `i * dt` 处，这正是指标的拟合所假设的——否则拟合原点晚一个周期，偏差会以带宽的形式出现。
    speed.push_back(0.0F);
    bench.run(0.08F, [&speed](const foc::sil::Bench &b) { speed.push_back(b.sample().speed); });
    ASSERT_GT(speed.size(), 100U);

    const foc::core::StepMetrics m = foc::core::step_metrics(speed.data(), speed.size(), cfg.dt_ctrl_s, target, 0.02F * target);
    ASSERT_EQ(foc::ErrorCode::Ok, m.status);

    // Why the band is 10 % and not 2 %: the cascade is not the `wb/s` the gain law derives, because the inner current
    // loop is finite. Measured attribution on this bench -- raising the inner bandwidth 2000 -> 4000 rad/s moves the
    // number from +9.4 % to +5.8 %, halving the control period moves it by 0.07 points, and giving the window its
    // pre-step sample moves it by 0.30 (9.45 % -> 9.15 %). The inner loop is the story; the rest is a first-order
    // model applied to a cascade.
    // 为什么判据是 10% 而不是 2%：级联并不是增益律推导出的那个 `wb/s`，因为内层电流环是有限的。本台架上的实测归因
    // ——内层带宽 2000 → 4000 rad/s 把数字从 +9.4% 挪到 +5.8%，控制周期减半挪 0.07 个点，给窗口补上阶跃前样本挪
    // 0.30（9.45% → 9.15%）。
    // 内层环才是主因，其余来自把一阶模型套在级联上。
    const float requested = cfg.omega_speed_rad_s;
    EXPECT_NEAR(1.0F, m.bandwidth_rad_s / requested, 0.10F)
        << "measured " << m.bandwidth_rad_s << " rad/s against a requested " << requested
        << " rad/s: the gain law and its documentation have parted ways";
}


TEST(StepMetrics, TheMeasuredBandwidthFollowsTheRequestRatherThanIgnoringIt) {
    // The absolute band above is weak, and this case exists because that was measured rather than assumed: scaling the
    // speed loop's proportional gain by 0.8 -- a 20 % error in the gain law -- moved the reading only to 225.31 rad/s,
    // i.e. -9.88 %, *inside* the 10 % band. The estimator carries a +9.15 % bias of its own (a first-order fit on a
    // cascade), and a bias of the same size as the defect hides it.
    //
    // What that bias cannot hide is a gain law that ignores the request: doubling the requested bandwidth and
    // halving it must move the measurement by the same factor, because the estimator is the same on both sides and its
    // bias largely cancels in the ratio. Measured: 134.66 rad/s at a 125 request and 272.87 at 250, a ratio of 2.026
    // (1.3 % from the ideal 2), against a 5 % band. This does not catch a scale error -- the sweep that would is the
    // next increment -- but it catches the failure the absolute band is blindest to.
    //
    // 上面那个绝对带是弱的，本用例存在正是因为这一点是被测出来的、不是被假设的：把速度环的比例增益乘 0.8——增益律
    // 20% 的错误——只把读数挪到 225.31 rad/s，即 -9.88%，**在** 10% 带内。估计量自带 +9.15% 的偏差（把一阶拟合
    // 套在级联上），与被测缺陷同量级的偏差会把它藏起来。
    //
    // 这种偏差藏不住的是"增益律根本不看请求"：把请求带宽加倍、减半，测量必须同比例变化，因为两侧用的是同一个估计量，
    // 其偏差在比值里大体抵消。这不覆盖比例错误——那要靠扫频，属于下一个增量——但它覆盖了绝对带最盲的那一类失败。
    const auto measured_at = [](float requested_rad_s) {
        foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
        cfg.omega_speed_rad_s = requested_rad_s;
        foc::sil::Bench bench(cfg, foc::sil::nominal_plant_params());
        bench.motor().set_speed(0.0F);
        static_cast<void>(bench.init_and_start());
        bench.run(2e-3F);
        const float target = 10.0F;
        bench.motor().set_speed(target);
        std::vector<float> speed{0.0F};
        bench.run(0.08F, [&speed](const foc::sil::Bench &b) { speed.push_back(b.sample().speed); });
        return foc::core::step_metrics(speed.data(), speed.size(), cfg.dt_ctrl_s, target, 0.02F * target).bandwidth_rad_s;
    };

    const float slow = measured_at(125.0F);
    const float fast = measured_at(250.0F);
    ASSERT_GT(slow, 0.0F);
    ASSERT_GT(fast, 0.0F);
    EXPECT_NEAR(2.0F, fast / slow, 0.05F)
        << "asked for 125 and 250 rad/s, measured " << slow << " and " << fast
        << ": the loop is not being tuned by the bandwidth it is given";
}


TEST(StepMetrics, TheLibraryAndTheSilHelperAgreeOnSettlingTime) {
    // The definition exists twice on purpose -- once in the library for callers, once in the SIL harness that predates
    // it -- so the two are compared rather than assumed to match. A test helper and a shipped function that disagree
    // about "settled" would make every SIL number and every customer number incomparable.
    // 这个定义是刻意存在两份的——库给调用方的一份、早于它的 SIL 台架一份——因此比对而不是假设它们一致。测试助手与
    // 出厂函数对"已稳定"有分歧，会让每一个 SIL 数字与每一个客户数字无法互相比较。
    const std::vector<float> y = first_order(0.01F, 100.0F, 2000U);
    std::vector<foc::sil::Sample> trace;
    trace.reserve(y.size());
    for (std::size_t i = 0U; i < y.size(); ++i) {
        foc::sil::Sample s{};
        s.t_s = static_cast<float>(i) * kDt;
        s.speed = y[i];
        trace.push_back(s);
    }

    const float helper = foc::sil::settling_time_s(trace, &foc::sil::Sample::speed, 100.0F, 2.0F);
    const foc::core::StepMetrics m = foc::core::step_metrics(y.data(), y.size(), kDt, 100.0F, 2.0F);
    EXPECT_FLOAT_EQ(helper, m.settling_s) << "helper " << helper << " vs library " << m.settling_s;
}


namespace {

constexpr double kProbeTauS = 0.004;  ///< 4 ms, i.e. 250 rad/s
constexpr double kProbeStepA = 10.0;  ///< the step amplitude the probe signals approach

/**
 * @brief [en] A first-order approach with an independent gain, a DC offset and optional Gaussian noise, in float.
 *
 * @brief [zh] 带独立增益、直流偏置与可选高斯噪声的一阶逼近，用 float 表示。
 *
 *     `gain` is the ratio between the value the response actually converges to and the `target` the analyser is told.
 *     At 1.0 they agree and the fit is exact; otherwise the caller has passed a target the loop does not reach, which
 *     is what a P-only loop with steady-state droop does to whoever measures it.
 *
 *     `gain` 是响应真正收敛到的值与告知分析器的 `target` 之间的比。为 1.0 时两者一致、拟合精确；否则调用方给的是一个环
 *     路到不了的目标——那正是带静差的比例环对"拿它做测量的人"所做的事。
 */
auto probe_signal(double dt_s, std::size_t n, double gain, double offset, double noise_sigma, unsigned seed)
    -> std::vector<float> {
    std::vector<float> y(n, 0.0F);
    unsigned state = seed;
    for (std::size_t i = 0U; i < n; ++i) {
        const double t = static_cast<double>(i) * dt_s;
        double value = offset + (gain * kProbeStepA * (1.0 - std::exp(-t / kProbeTauS)));
        if (noise_sigma > 0.0) {
            state = (state * 1103515245U) + 12345U;
            const double u1 = static_cast<double>((state >> 16U) & 0x7fffU) / 32768.0;
            state = (state * 1103515245U) + 12345U;
            const double u2 = static_cast<double>((state >> 16U) & 0x7fffU) / 32768.0;
            const double gauss = std::sqrt(-2.0 * std::log(u1 + 1.0e-12)) * std::cos(6.283185307179586 * u2);
            value += noise_sigma * kProbeStepA * gauss;
        }
        y[i] = static_cast<float>(value);
    }
    return y;
}

/** @brief [en] Fit the probe and return `tau / kProbeTauS`, or a negative number when the fit refused.
 *  @brief [zh] 拟合探针信号并返回 `tau / kProbeTauS`，拟合被拒时返回负数。 */
auto fitted_ratio(double dt_s, std::size_t n, double gain, double offset, double noise_sigma = 0.0, unsigned seed = 1U)
    -> double {
    const std::vector<float> y = probe_signal(dt_s, n, gain, offset, noise_sigma, seed);
    const auto metrics = foc::core::step_metrics(y.data(), y.size(), static_cast<float>(dt_s),
                                                 static_cast<float>(kProbeStepA + offset),
                                                 static_cast<float>(0.02 * kProbeStepA));
    if (metrics.status != foc::ErrorCode::Ok) {
        return -1.0;
    }
    return static_cast<double>(metrics.tau_s) / kProbeTauS;
}

} // namespace

/**
 * @brief [en] The fit is exact in its own sample rate, in its window, and in a DC offset -- three coefficients of
 *             exactly zero, and they are the ones that say what kind of estimator this is.
 *
 * @brief [zh] 拟合在自身的采样率、窗口与直流偏置上都是精确的——三个**恰好为零**的系数，而它们说明的正是这是什么估计器。
 *
 *     A sensitivity table with only non-zero rows cannot distinguish a good estimator from a lucky one; the interesting
 *     rows here are the zeros, because each of them is a structural fact that a plausible alternative implementation
 *     would break:
 *
 *     - **sample rate, 0.2 ms to 2 ms (10x): ratio 1.000000 -> 0.9999995.** The regressor is `t` and the ordinate is
 *     `ln(1 - progress)`; for a sampled exponential that relation is *exactly* linear at any spacing, so the fit is not
 *     approximating a continuous curve. An implementation that fitted a continuous-time approximation, or that assumed
 *     a particular number of samples per time constant, would move here. - **window, 10 ms to 40 ms (2.5x more
 *     samples): ratio 1.000000 -> 0.999999.** Same reason -- every sample is on the line, so adding samples adds
 *     information and not bias. - **DC offset, +-1 and +5 on a 10 A step: ratio 1.000000.** The offset appears in
 *     `y[0]` and in `y[i]` and cancels in `progress`. That cancellation is the reason a biased current sensor does not
 *     corrupt a bandwidth measurement -- and a version that forgot to subtract `y[0]` would show it immediately.
 *
 *     The boundary is stated rather than hidden: at an 80 ms window (**20 time constants**) the ratio is 1.002148, and
 *     that is not the fit's fault -- the samples are `float32`, and `1 - exp(-20)` cannot be represented finely enough
 *     to keep `ln(1 - progress)` straight. So the estimator's accuracy at long windows is set by the *sample format*,
 *     not by the algebra, which is worth knowing before recording a 20-time-constant window and expecting 1e-6.
 *
 *     只有非零行的敏感度表分不清"好估计器"与"运气好的估计器"；这里有意思的行是那三个零，因为它们每一个都是一条会被貌似
 *     合理的替代实现破坏的结构性事实：采样率（0.2→2 ms）**1.000000 → 0.9999995**、窗口（10→40 ms）**1.000000 →
 *     0.999999**（理由相同：每个样本都在那条直线上，加样本只加信息不加偏置）、直流偏置 ±1 与 +5（对 10 A 阶跃）
 *     **1.000000**（偏置在 `y[0]` 与 `y[i]` 里同时出现、在 `progress` 里约掉——这正是偏置的电流传感器不会污染带宽测量的
 *     原因，而一个忘了减 `y[0]` 的版本会立刻显形）。
 *
 *     边界写出来而不是藏起来：**20 个时间常数**（80 ms）处比值是 1.002148，那不是拟合的问题——样本是 `float32`， `1 −
 *     exp(−20)` 无法表示得足够细以让 `ln(1 − progress)` 保持平直。所以长窗口下的精度由**样本格式**而不是代数决定，这一
 *     点在录一段 20τ 的窗口并期待 1e-6 之前值得知道。
 */
TEST(StepMetrics, TheFitIsExactInItsOwnSampleRateWindowAndDcOffset) {
    // Sample rate: same physical window, tenfold change in spacing.
    // 采样率：同一个物理窗口，间距变化十倍。
    for (const double dt : {0.0002, 0.0005, 0.001, 0.002}) {
        const auto n = static_cast<std::size_t>(0.020 / dt) + 1U;
        const double ratio = fitted_ratio(dt, n, 1.0, 0.0);
        ASSERT_GT(ratio, 0.0) << "the fit refused a clean exponential at dt " << dt;
        EXPECT_NEAR(1.0, ratio, 1.0e-5) << "dt " << dt;
        std::printf("\n[step] dt %.4f s (n=%zu): tau ratio %.9f\n", dt, n, ratio);
    }
    // Window: same spacing, 2.5x more samples.
    // 窗口：同一间距，多 2.5 倍的样本。
    for (const double window : {0.010, 0.020, 0.040}) {
        const auto n = static_cast<std::size_t>(window / 0.0002) + 1U;
        const double ratio = fitted_ratio(0.0002, n, 1.0, 0.0);
        ASSERT_GT(ratio, 0.0);
        EXPECT_NEAR(1.0, ratio, 1.0e-5) << "window " << window;
    }
    // DC offset: a biased sensor must not move the bandwidth.
    // 直流偏置：偏置的传感器不得移动带宽。
    for (const double offset : {-1.0, 0.0, 1.0, 5.0}) {
        const double ratio = fitted_ratio(0.0002, 101U, 1.0, offset);
        ASSERT_GT(ratio, 0.0);
        EXPECT_NEAR(1.0, ratio, 1.0e-6) << "offset " << offset;
    }
    // And the stated boundary: 20 time constants is the sample format, not the algebra.
    // 以及写明的边界：20 个时间常数处是样本格式在说话，不是代数。
    const double long_window = fitted_ratio(0.0002, 401U, 1.0, 0.0);
    EXPECT_GT(long_window, 1.0015) << "the float32 resolution limit moved, so revisit the stated boundary";
    EXPECT_LT(long_window, 1.0030) << "the float32 resolution limit moved, so revisit the stated boundary";
}

/**
 * @brief [en] The one error that really costs: a final value the caller did not pass as the target. A 5 % droop
 *             inflates the fitted time constant by 44 %, and noise moves it 1.3 % per 1 %.
 *
 * @brief [zh] 真正花钱的那一个误差：调用方没有把响应真正收敛到的值当作 `target`。5 % 的静差把拟合出的时间常数抬高 44
 *             %，而噪声每 1 % 只动 1.3 %。
 *
 *     The fit is exact in everything the model can absorb (see the case above), so the error sources that survive are
 *     the ones that put the samples *off* the model's line at the far end:
 *
 *     | what is wrong | ratio to the true tau | |---------------|-----------------------| | final value 5 % **below**
 *     the target (P-only droop) | **1.438404** | | final value 5 % **above** the target | **0.735124** | | final value
 *     10 % below | **1.767368** | | final value 10 % above | **0.660542** | | noise, sigma = 1 % of the step (200
 *     seeds) | **0.987336** | | noise, sigma = 2 % | **0.982475** |
 *
 *     Two things follow, and both are operational. **Asymmetric and large**: passing the *command* as the target while
 *     the loop has a steady-state error is the mistake that matters -- 5 % costs 44 % -- and it is the mistake a P-only
 *     speed loop invites, so the target has to be the measured settled value, not the request. **Noise is cheap**: 1 %
 *     of the step buys 1.3 % of tau, which is why this estimator can be run on a bench with ordinary current noise and
 *     still resolve a 10 % tuning change.
 *
 *     拟合在模型能吸收的一切上都是精确的（见上一个用例），所以存活下来的误差源是那些把样本在**远端**推离模型直线的：终
 *     值比 target 低 5 %（比例环静差）**1.438404**、高 5 % **0.735124**、低 10 % **1.767368**、高 10 % **0.660542**；噪
 *     声 σ = 阶跃的 1 %（200 个种子）**0.987336**、2 % 时 **0.982475**。
 *
 *     两个后果都可操作。**不对称且很大**：环路有静差时把**指令**当 target 传进去是真正要命的错——5 % 就花掉 44 %—— 而这
 *     正是比例速度环招来的错，所以 target 必须是实测的稳态值，不是请求值。**噪声很便宜**：1 % 的阶跃只买 1.3 % 的时间常
 *     数，这正是这个估计器能在有普通电流噪声的台架上跑、还能分辨 10 % 整定变化的原因。
 */
TEST(StepMetrics, ADroopInTheFinalValueCostsFarMoreThanNoiseDoes) {
    struct GainRow final {
        double gain;
        double expected;
    };
    for (const GainRow& row : {GainRow{1.10, 0.660542}, GainRow{1.05, 0.735124}, GainRow{0.95, 1.438404},
                               GainRow{0.90, 1.767368}}) {
        const double ratio = fitted_ratio(0.0002, 101U, row.gain, 0.0);
        ASSERT_GT(ratio, 0.0) << "gain " << row.gain;
        EXPECT_NEAR(row.expected, ratio, 0.002) << "gain " << row.gain;
    }
    // A droop and a surplus must not look the same: the map is not symmetric, and the asymmetry is what makes the
    // sign of the error identifiable from the measurement.
    // 低落与超出不得看起来一样：这个映射不对称，而正是不对称让误差的符号可从测量中辨认。
    const double droop = fitted_ratio(0.0002, 101U, 0.95, 0.0);
    const double surplus = fitted_ratio(0.0002, 101U, 1.05, 0.0);
    EXPECT_GT(droop, 1.4) << "a 5 % droop must lengthen the measured tau";
    EXPECT_LT(surplus, 0.8) << "and a 5 % surplus must shorten it";
    // Asymmetric, but only mildly: the measured distances from 1.0 are 0.438404 (droop) against 0.265124 (surplus),
    // i.e. 1.65x. Asserting a larger asymmetry was my own first guess and it was wrong, so the band is the measured
    // ratio and not a round number chosen to look decisive.
    // 不对称，但只是轻微地：实测离 1.0 的距离是 0.438404（低落）对 0.265124（超出），即 1.65 倍。断言更大的不对称
    // 是我自己的第一版猜测，而它是错的，所以带子取实测比值，而不是一个挑来显得果断的整数。
    EXPECT_GT(std::fabs(droop - 1.0), 1.5 * std::fabs(surplus - 1.0)) << "the map was expected to be asymmetric";
    EXPECT_LT(std::fabs(droop - 1.0), 1.8 * std::fabs(surplus - 1.0)) << "and asymmetric by about 1.65x, not more";

    // Noise: averaged over 200 deterministic seeds, so the number is reproducible rather than "about".
    // 噪声：对 200 个确定性种子取平均，所以这个数可复现，而不是"大概"。
    double previous_ratio = 1.0;
    for (const double sigma : {0.01, 0.02}) {
        double sum = 0.0;
        int fitted = 0;
        for (unsigned seed = 1U; seed <= 200U; ++seed) {
            const double ratio = fitted_ratio(0.0002, 101U, 1.0, 0.0, sigma, seed * 7919U);
            if (ratio > 0.0) {
                sum += ratio;
                ++fitted;
            }
        }
        ASSERT_EQ(200, fitted) << "the fit refused a noisy but healthy response at sigma " << sigma;
        const double mean_ratio = sum / static_cast<double>(fitted);
        std::printf("[step] noise %.0f %% of the step: mean tau ratio %.6f over %d seeds\n", sigma * 100.0, mean_ratio,
                    fitted);
        // Noise biases tau *low*, between one and two times sigma (measured 1.27 % at sigma = 1 %, 1.75 % at 2 %).
        // The first version of this bound claimed "less than 1:1" -- measured 1.27, so the claim was wrong and the
        // band is the measurement rather than the guess.
        // 噪声把 tau 往**低**推，约在一到两倍 σ 之间（实测 σ=1 % 时 1.27 %，2 % 时 1.75 %）。这条界的第一个版本写的是
        // "小于 1:1"——实测 1.27，所以那个断言是错的，带子改成实测值而不是那个猜的数。
        EXPECT_LT(mean_ratio, 1.0) << "noise was expected to shorten the fitted tau, at sigma " << sigma;
        EXPECT_GT(mean_ratio, 1.0 - (2.0 * sigma)) << "noise cost more than 2:1 at sigma " << sigma;
        EXPECT_LT(mean_ratio, previous_ratio) << "more noise must shorten the fit further, at sigma " << sigma;
        previous_ratio = mean_ratio;
    }
}

/**
 * @brief [en] The two ends of the settling rule -- `0` for a response that never leaves the band and `-1` for one that
 *             never reaches it -- and the arrival time in between against its closed form.
 *
 * @brief [zh] 稳定规则的两端——从未离开带内的响应为 `0`、永不到达的为 `-1`——以及两者之间那个到达时刻对着闭式。
 *
 *     `settling_s` promises three different things and two of them had never been produced: `0` when the response
 *     starts inside the band, `-1` when it never gets there, and the arrival time otherwise. The `0` looks like a
 *     degenerate corner until it is stated as a contract: a caller that reports "settled at 0 s" for a loop that never
 *     moved is wrong in the direction that hurts, because 0 s reads as *perfect*. The arrival in the middle has a
 *     closed form -- the response is inside once `step*exp(-t/tau) <= band` -- so it is compared to arithmetic rather
 *     than to a remembered number: measured **18.5000 ms** against `tau*ln(step/band) = 18.4207 ms`, one sample grid
 *     apart.
 *
 *     `settling_s` 承诺三件不同的事，而其中两件从未被产出过：起始就在带内为 `0`、永不到达为 `-1`、其余是到达时刻。那个
 *     `0` 看着像退化的角落，直到它被写成契约：对一个根本没动过的环路报"0 s 就稳定了"是朝**有害**方向的错，因为 0 s 读起
 *     来是**完美**。中间那个到达时刻有闭式——一旦 `step·exp(−t/τ) ≤ band` 就进带——所以它对着算术比而不是对着一个记下来的
 *     数：实测 **18.5000 ms** 对 `τ·ln(step/band) = 18.4207 ms`，差一个采样栅格。
 *
 *     Stated rather than tested: the fit's `!(slope < 0)` guard is **unreachable**. Every regression sample has `0 <
 *     progress < 1`, hence `0 < w < 1`, hence `ln(w) < 0`, and `t > 0`, so the through-origin slope is a sum of
 *     strictly negative terms -- it cannot come out non-negative. A receding response therefore reports `State` from
 *     the *earlier* guard (fewer than three usable samples), which the existing case already covers. The dead guard is
 *     harmless and worth keeping as a tripwire, but no test can reach it, and claiming otherwise would be claiming a
 *     branch that does not exist.
 *
 *     写明而不是测出：拟合的 `!(slope < 0)` 守卫**不可达**。每个回归样本都有 `0 < progress < 1`，于是 `0 < w < 1`，于是
 *     `ln(w) < 0`，而 `t > 0`，所以过原点斜率是一串严格负数之和——它不可能非负。因此"退回"的响应是由**更早**那条守卫（可
 *     用样本少于三个）报 `State` 的，而那条已有用例覆盖。这条死守卫无害、值得留着当绊线，但没有任何测试能到达它，声称能
 *     到达就是在声称一个不存在的分支。
 */
TEST(StepMetrics, TheTwoEndsOfTheSettlingRuleAreZeroAndMinusOne) {
    // Already inside: a half-width band as wide as the whole gap, so no sample is ever outside it.
    // 已在带内：半宽等于整个间距的带宽，于是没有哪个样本落在带外。
    const std::vector<float> almost_there = first_order(0.004F, 10.0F, 50U);
    const auto inside = foc::core::step_metrics(almost_there.data(), almost_there.size(), kDt, 10.0F, 10.0F);
    EXPECT_EQ(foc::ErrorCode::Ok, inside.status);
    EXPECT_FLOAT_EQ(0.0F, inside.settling_s) << "a response that never leaves the band has settled at 0 s, not -1";

    // Then not inside at all: the same window with a band far narrower than its tail reports "never", which is -1 --
    // and the difference between -1 and 0 is the whole point of having both.
    // 然后再看完全不在带内：同一窗口配一条远窄于其尾巴的带报"永不"，即 -1——而 -1 与 0 之间的差别正是两者都存在的原因。
    const auto never = foc::core::step_metrics(almost_there.data(), almost_there.size(), kDt, 10.0F, 1e-4F);
    EXPECT_FLOAT_EQ(-1.0F, never.settling_s) << "a response that never arrives reported something else";

    // And the positive arrival time, which has a closed form: the response is inside the band once
    // `step*exp(-t/tau) <= band`, i.e. `t >= tau*ln(step/band)`. For a 10 rad/s step, a 0.1 rad/s band and a 4 ms
    // time constant that is `4 ms * ln(100) = 18.42 ms`; the sample grid can only resolve it to a period, so the band
    // below is two periods. (My first version of this line used `ln(1000)` -- it assumed a 0.1 % band from a 1 % one
    // and predicted 27.6 ms, which the measurement refused. The formula, not the number, was wrong.)
    // 以及正的到达时刻，它有闭式：一旦 `step·exp(−t/τ) ≤ band` 即 `t ≥ τ·ln(step/band)` 就进了带。对 10 rad/s 的阶跃、
    // 0.1 rad/s 的带、4 ms 的时间常数，那是 `4 ms · ln(100) = 18.42 ms`；采样栅格只能分辨到一个周期，所以下面的带是
    // 两个周期。（这一行第一版用了 `ln(1000)`——它把 1 % 的带当成了 0.1 %，于是预测 27.6 ms，被测量挡下了。错的是公式，
    // 不是那个数。）
    const std::vector<float> long_window = first_order(0.004F, 10.0F, 401U);
    const auto arriving = foc::core::step_metrics(long_window.data(), long_window.size(), kDt, 10.0F, 0.1F);
    const float predicted_s = 0.004F * std::log(10.0F / 0.1F);
    EXPECT_NEAR(predicted_s, arriving.settling_s, 2.0F * kDt) << "the arrival time is `tau*ln(step/band)`";
    std::printf("[step] settling measured %.4f ms against tau*ln(step/band) = %.4f ms\n",
                static_cast<double>(arriving.settling_s) * 1e3, static_cast<double>(predicted_s) * 1e3);
}
