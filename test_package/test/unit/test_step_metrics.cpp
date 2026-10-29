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
