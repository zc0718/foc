#include <cmath>
#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/filters.hpp>
#include <foc/core/sinusoid_fit.hpp>
#include <foc/core/types.hpp>



TEST(FirstOrderLowPass, RejectsCutoffAboveNyquist) {
    foc::core::FirstOrderLowPass filter;

    // dt = 1e-4 s -> Nyquist is 5 kHz; 5 kHz itself is not below Nyquist.
    EXPECT_EQ(foc::ErrorCode::Param, filter.configure(5000.0F, 1e-4F));
    EXPECT_FALSE(filter.configured());
}



TEST(FirstOrderLowPass, RejectsNonPositiveArguments) {
    foc::core::FirstOrderLowPass filter;

    EXPECT_EQ(foc::ErrorCode::Param, filter.configure(100.0F, 0.0F));
    EXPECT_EQ(foc::ErrorCode::Param, filter.configure(0.0F, 1e-4F));
}



TEST(FirstOrderLowPass, PassesInputThroughWhenUnconfigured) {
    foc::core::FirstOrderLowPass filter;

    EXPECT_FLOAT_EQ(3.5F, filter.step(3.5F));
}



TEST(FirstOrderLowPass, FirstSampleSeedsTheState) {
    foc::core::FirstOrderLowPass filter;
    ASSERT_EQ(foc::ErrorCode::Ok, filter.configure(100.0F, 1e-4F));

    EXPECT_FLOAT_EQ(2.0F, filter.step(2.0F));
}



TEST(FirstOrderLowPass, ConvergesTowardsConstantInput) {
    foc::core::FirstOrderLowPass filter;
    ASSERT_EQ(foc::ErrorCode::Ok, filter.configure(200.0F, 1e-4F));

    float out = 0.0F;
    for (int i = 0; i < 2000; ++i) {
        out = filter.step(1.0F);
    }

    EXPECT_NEAR(1.0F, out, 1e-3F);
}



TEST(FirstOrderLowPass, IsMonotonicForStepInput) {
    foc::core::FirstOrderLowPass filter;
    ASSERT_EQ(foc::ErrorCode::Ok, filter.configure(100.0F, 1e-4F));

    float previous = filter.step(0.0F);
    for (int i = 0; i < 100; ++i) {
        const float out = filter.step(1.0F);
        EXPECT_GE(out, previous);
        EXPECT_LE(out, 1.0F);
        previous = out;
    }
}



TEST(FirstOrderLowPass, ResetReseedsFromNextSample) {
    foc::core::FirstOrderLowPass filter;
    ASSERT_EQ(foc::ErrorCode::Ok, filter.configure(100.0F, 1e-4F));
    static_cast<void>(filter.step(10.0F));

    filter.reset();

    EXPECT_FLOAT_EQ(0.5F, filter.step(0.5F));
}



TEST(MovingAverage, AveragesTheFirstSamples) {
    foc::core::MovingAverage<float, 4> avg;

    EXPECT_FLOAT_EQ(1.0F, avg.step(1.0F));
    EXPECT_FLOAT_EQ(1.5F, avg.step(2.0F));
    EXPECT_FLOAT_EQ(2.0F, avg.step(3.0F));
    EXPECT_FLOAT_EQ(2.5F, avg.step(4.0F));
    EXPECT_EQ(4U, avg.size());
}



TEST(MovingAverage, DropsTheOldestSampleOnceFull) {
    foc::core::MovingAverage<float, 4> avg;
    for (int i = 1; i <= 4; ++i) {
        static_cast<void>(avg.step(static_cast<float>(i)));
    }

    // window becomes 2,3,4,5
    EXPECT_FLOAT_EQ(3.5F, avg.step(5.0F));
    EXPECT_EQ(4U, avg.size());
}



TEST(MovingAverage, ResetClearsTheWindow) {
    foc::core::MovingAverage<float, 2> avg;
    static_cast<void>(avg.step(10.0F));
    static_cast<void>(avg.step(20.0F));

    avg.reset();

    EXPECT_EQ(0U, avg.size());
    EXPECT_FLOAT_EQ(5.0F, avg.step(5.0F));
}



TEST(MovingAverage, WorksWithDouble) {
    foc::core::MovingAverage<double, 3> avg;
    static_cast<void>(avg.step(1.0));
    static_cast<void>(avg.step(2.0));
    static_cast<void>(avg.step(4.0));

    // window [1,2,4] -> push 4 -> [2,4,4]
    EXPECT_NEAR(10.0 / 3.0, avg.step(4.0), 1e-12);
}

namespace {

/** @brief [en] `|H|` of `y[n] = (1-alpha)*y[n-1] + alpha*x[n]` at a sampling angle `w` (rad/sample). */
/** @brief [zh] `y[n] = (1−α)·y[n−1] + α·x[n]` 在采样角 `w`（rad/采样）处的 `|H|`。 */
auto lowpass_gain(double alpha, double w) -> double {
  const double p = 1.0 - alpha;
  return alpha / std::sqrt(1.0 - (2.0 * p * std::cos(w)) + (p * p));
}

/** @brief [en] `arg H` of the same recurrence, rad. */
/** @brief [zh] 同一条递推式的 `arg H`，rad。 */
auto lowpass_phase(double alpha, double w) -> double {
  const double p = 1.0 - alpha;
  return -std::atan2(p * std::sin(w), 1.0 - (p * std::cos(w)));
}

/** @brief [en] The frequency at which `|H| = 1/sqrt(2)`, found on the analytic model. */
/** @brief [zh] `|H| = 1/√2` 处的频率，在解析模型上求解。 */
auto lowpass_corner_hz(double alpha, double dt) -> double {
  double lo = 1.0e-9;
  double hi = 0.5 / dt; // Nyquist
  for (int k = 0; k < 200; ++k) {
    const double mid = 0.5 * (lo + hi);
    if (lowpass_gain(alpha, 2.0 * M_PI * mid * dt) > (1.0 / std::sqrt(2.0))) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return 0.5 * (lo + hi);
}

/** @brief [en] The alpha `configure()` computes, so the model and the code cannot drift. */
/** @brief [zh] `configure()` 算出的那个 α，好让模型与代码不会走散。 */
auto alpha_of(double fc_hz, double dt) -> double {
  const double w = 2.0 * M_PI * fc_hz * dt;
  return w / (1.0 + w);
}

struct Swept final {
  double gain;
  double phase_rad;
};

/**
 * @brief [en] Drive a sine through a filter and measure it with the least-squares estimator, input phase subtracted.
 * @brief [zh] 把一条正弦打进滤波器，用最小二乘估计器量它，并减掉输入的相位。
 */
template <typename Filter> auto swept(Filter& filter, double f, double dt, std::size_t skip, std::size_t n) -> Swept {
  std::vector<float> command(n);
  std::vector<float> response(n);
  const double omega = 2.0 * M_PI * f;
  for (std::size_t k = 0; k < n; ++k) {
    command[k] = static_cast<float>(std::cos(omega * static_cast<double>(k) * dt));
    response[k] = filter.step(command[k]);
  }
  const foc::core::SinusoidFit in = foc::core::sinusoid_fit(command.data() + skip, n - skip, static_cast<float>(dt),
                                                           static_cast<float>(omega));
  const foc::core::SinusoidFit out = foc::core::sinusoid_fit(response.data() + skip, n - skip, static_cast<float>(dt),
                                                            static_cast<float>(omega));
  return Swept{static_cast<double>(out.amplitude) / static_cast<double>(in.amplitude),
               static_cast<double>(out.phase_rad) - static_cast<double>(in.phase_rad)};
}

} // namespace

TEST(FirstOrderLowPass, TheCornerIsBelowTheRequestAndTheShortfallGrowsWithTheSampling) {
  // `configure(fc, dt)` pre-warps with `alpha = w/(1+w)`, `w = 2*pi*fc*dt`, which is not the exact exponential. The
  // consequence is worth publishing because a user reads `cutoff_hz` as "the -3 dB point": it is not. Measured on the
  // analytic z-domain response -- and *the direction is the opposite of the obvious guess*: this recurrence is faster
  // than the analogue prototype, so the real corner sits **below** the request, by 1.5 % at `fc*dt = 0.005` and by a
  // third at `fc*dt = 0.25`.
  // `configure(fc, dt)` 用 `alpha = w/(1+w)`（`w = 2π·fc·dt`）做预畸变，它不是精确指数。后果值得公布，因为使用者读
  // `cutoff_hz` 时读的是"−3 dB 点"：并不是。在解析 z 域响应上量出——而且**方向与最直觉的猜测相反**：这个递推比模拟原型
  // **更快**，所以真实转折点在请求值**以下**，`fc·dt = 0.005` 时低 1.5 %，`0.25` 时低三分之一。
  const double dt = 50.0e-6; // 20 kHz
  double previous = 1.0;
  for (const double fc : {100.0, 500.0, 1000.0, 2000.0, 5000.0}) {
    const double corner = lowpass_corner_hz(alpha_of(fc, dt), dt);
    const double ratio = corner / fc;
    std::printf("[filters] fc %6.1f Hz at 20 kHz (fc*dt %.4f) -> real -3 dB %8.2f Hz = %.4f x the request\n", fc,
                fc * dt, corner, ratio);
    // Always *below* the request, and the shortfall grows with `fc*dt` -- the direction is what a user needs.
    // 永远**低于**请求值，且缺口随 `fc·dt` 增大——方向正是使用者需要知道的。
    EXPECT_LT(ratio, 1.0) << "fc " << fc;
    EXPECT_LT(ratio, previous) << "the shortfall stopped growing with fc*dt at fc " << fc;
    previous = ratio;
  }
  // At 100 Hz on 20 kHz the request is honoured to better than 2 %; at 5 kHz it is short by more than a third.
  // 20 kHz 下的 100 Hz 请求被兑现到 2 % 以内；5 kHz 的请求短了三分之一以上。
  EXPECT_GT(lowpass_corner_hz(alpha_of(100.0, dt), dt) / 100.0, 0.98);
  EXPECT_LT(lowpass_corner_hz(alpha_of(5000.0, dt), dt) / 5000.0, 0.70);
}

TEST(FirstOrderLowPass, TheSweptMagnitudeAndPhaseAreTheZDomainModel) {
  // The measured leg, with an instrument that shares no code with the model: drive a sine, fit the response with the
  // least-squares estimator in `sinusoid_fit.hpp`, fit the command too and subtract, so what is left is the filter.
  // 实测那一腿，用的仪器与模型不共享代码：驱动一条正弦，用 `sinusoid_fit.hpp` 的最小二乘估计器拟合响应，输入也拟合
  // 再相减，剩下的就是滤波器。
  const float dt = 50.0e-6F;
  const float fc = 1000.0F;
  const double alpha = alpha_of(static_cast<double>(fc), static_cast<double>(dt));
  const auto sweep = [&](float f) {
    foc::core::FirstOrderLowPass filter;
    const bool ok = filter.configure(fc, dt) == foc::ErrorCode::Ok;
    return ok ? swept(filter, static_cast<double>(f), static_cast<double>(dt), 1024, 4096) : Swept{0.0, 0.0};
  };
  for (const float f : {100.0F, 250.0F, 500.0F, 750.0F, 1000.0F, 1250.0F, 2000.0F}) {
    const Swept measured = sweep(f);
    const double w = 2.0 * M_PI * static_cast<double>(f) * static_cast<double>(dt);
    const double model = lowpass_gain(alpha, w);
    std::printf("[filters] f %6.1f Hz | measured |H| %.5f | z-domain %.5f | error %+.3f %% | phase %+.4f (model "
                "%+.4f)\n",
                static_cast<double>(f), measured.gain, model, 100.0 * ((measured.gain / model) - 1.0),
                measured.phase_rad, lowpass_phase(alpha, w));
    // 1 % on the magnitude: the instrument is a projection, not an estimate, so this is a real agreement rather than a
    // tolerance chosen to fit the numbers.
    // 幅值 1 %：仪器是投影而不是估计，所以这是真实的一致，不是照着结果挑的容差。
    EXPECT_NEAR(model, measured.gain, 0.01 * model) << "f " << f;
    EXPECT_NEAR(lowpass_phase(alpha, w), measured.phase_rad, 0.02) << "f " << f;
    EXPECT_LT(measured.phase_rad, 0.0) << "a low-pass cannot lead";
  }
  // The phase at the corner, pinned: `-0.7051 rad`, not the analogue prototype's `-pi/4`.
  // 转折点处的相位，钉住：`−0.7051 rad`，不是连续原型的 `−π/4`。
  const double corner_phase = lowpass_phase(alpha, 2.0 * M_PI * 1000.0 * static_cast<double>(dt));
  std::printf("[filters] the model's own phase at 1 kHz is %+.4f rad (the analogue prototype says %+.4f)\n",
              corner_phase, -0.25 * M_PI);
  EXPECT_NEAR(-0.7051, corner_phase, 0.01);
}

TEST(FirstOrderLowPass, ATenthOfAPeriodWrongMovesTheCornerByRoughlyThatMuch) {
  // The error source a caller actually has: the control period the filter was configured with is not the one the loop
  // runs at. `alpha` is built from `fc*dt`, so a period `k` times too large moves the corner by about `k` -- measured
  // on the analytic response, and the coefficient is close to one because the pre-warp is monotone in `fc*dt`.
  // 调用方真有的误差源：滤波器配置时用的控制周期不是环实际跑的周期。`alpha` 由 `fc·dt` 构成，于是周期大 `k` 倍
  // 会让转折点大致移动 `k` 倍——在解析响应上量出，系数接近 1，因为预畸变对 `fc·dt` 是单调的。
  const double fc = 1000.0;
  const double dt = 50.0e-6;
  const double nominal = lowpass_corner_hz(alpha_of(fc, dt), dt);
  for (const double k : {0.9, 0.95, 1.05, 1.1}) {
    // The *configured* period is `k*dt`; the loop still samples at `dt`, so the response is evaluated at `dt`.
    // **配置**的周期是 `k·dt`；环仍然按 `dt` 采样，所以响应在 `dt` 上取值。
    const double wrong = lowpass_corner_hz(alpha_of(fc, k * dt), dt);
    std::printf("[filters] period x%.2f | corner %.2f Hz | ratio %.4f vs k %.4f\n", k, wrong, wrong / nominal, k);
    EXPECT_NEAR(k, wrong / nominal, 0.06) << "period x" << k;
  }
}

TEST(MovingAverage, TheNullTheDelayAndTheDcGainAreExact) {
  // A moving average of `N` has three closed forms and all three are exact: DC gain 1, nulls at `k*fs/N` (the Dirichlet
  // kernel), and a group delay of exactly `(N-1)/2` samples. Measured by sweeping the same sine through it and fitting.
  // 长度 `N` 的滑动平均有三条闭式，三条都精确：直流增益 1、在 `k·fs/N` 处为零（Dirichlet 核）、群延迟恰好
  // `(N−1)/2` 个采样。用同一条正弦扫过去拟合量出。
  constexpr std::size_t kN = 8;
  const float dt = 50.0e-6F;
  const double fs = 1.0 / static_cast<double>(dt);
  const auto sweep = [&](double f) {
    foc::core::MovingAverage<float, kN> average;
    return swept(average, f, static_cast<double>(dt), 1024, 4096);
  };
  // DC: the mean of a constant is that constant, to the bit.
  // 直流：常数的均值就是那个常数，逐位。
  {
    foc::core::MovingAverage<float, kN> average;
    float last = 0.0F;
    for (int k = 0; k < 100; ++k) {
      last = average.step(3.25F);
    }
    EXPECT_FLOAT_EQ(3.25F, last);
  }
  // The first null, at `fs/N`, where the Dirichlet kernel is exactly zero.
  // 第一个零点，在 `fs/N` 处，那里 Dirichlet 核精确为零。
  const Swept null_measured = sweep(fs / static_cast<double>(kN));
  std::printf("[filters] moving average N=%zu | first null at %.1f Hz: |H| = %.3e\n", kN, fs / static_cast<double>(kN),
              null_measured.gain);
  EXPECT_LT(null_measured.gain, 1.0e-5) << "the null at fs/N is not a null";
  // Group delay: `(N-1)/2` samples, measured as the phase slope well below the first null.
  // 群延迟：`(N−1)/2` 个采样，在远低于第一个零点的频率上以相位斜率量出。
  const double low_f = fs / (10.0 * static_cast<double>(kN));
  const Swept low_measured = sweep(low_f);
  const double delay_samples = -low_measured.phase_rad / (2.0 * M_PI * low_f * static_cast<double>(dt));
  std::printf("[filters] group delay at %.1f Hz: %.4f samples (law (N-1)/2 = %.1f)\n", low_f, delay_samples,
              static_cast<double>(kN - 1) / 2.0);
  EXPECT_NEAR(static_cast<double>(kN - 1) / 2.0, delay_samples, 0.1);
  // And the magnitude between the nulls is the Dirichlet kernel, exactly.
  // 零点之间的幅值正是 Dirichlet 核，精确。
  const double quarter = fs / (4.0 * static_cast<double>(kN));
  const double dirichlet = std::sin(kN * M_PI * quarter / fs) / (kN * std::sin(M_PI * quarter / fs));
  const Swept quarter_measured = sweep(quarter);
  std::printf("[filters] at %.1f Hz: measured %.5f, Dirichlet %.5f\n", quarter, quarter_measured.gain, dirichlet);
  EXPECT_NEAR(dirichlet, quarter_measured.gain, 0.005 * std::fabs(dirichlet));
}
