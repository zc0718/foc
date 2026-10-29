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
