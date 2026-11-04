// F33, the experiment that decides the compensator's shape: a cogging torque injected into the plant the closed loop runs
// on, and the question of where it shows up. If the loop merely *absorbs* it, a compensator is unnecessary; if it shows up
// in the current demand, a compensator has something to do and a known size; and if that size shrinks with speed, the
// compensator has a frequency boundary. All three are measured here.
// F33 里决定补偿器形状的实验：把齿槽转矩注入闭环真正跑的那台 plant，问它出现在哪里。若环路只是**吸收**了它，补偿器就不
// 必要；若它出现在电流需求里，补偿器就有活干、而且有已知的量级；若这个量级随转速缩小，补偿器就有频率边界。三条都在这里量。

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "sil_bench.hpp"

namespace {

using foc::sil::Bench;
using foc::sil::BenchConfig;
using foc::sil::CoggingHarmonic;
using foc::sil::standard_bench_config;

constexpr int kOrder = 6;

/** @brief Fourier amplitude of `series` at `order`, with the samples taken at the mechanical angles in `theta`. */
auto harmonic_amplitude(const std::vector<float>& theta, const std::vector<float>& series, int order) -> float {
    float re = 0.0F;
    float im = 0.0F;
    for (std::size_t n = 0; n < theta.size(); ++n) {
        re = static_cast<float>(re + (series[n] * std::cos(static_cast<float>(order) * theta[n])));
        im = static_cast<float>(im + (series[n] * std::sin(static_cast<float>(order) * theta[n])));
    }
    const auto count = static_cast<float>(theta.size());
    return (2.0F / count) * std::hypot(re, im);
}

/**
 * @brief [en] Hold a slow rotation with the speed loop and report the order-6 content of the q-axis current.
 * @brief [zh] 用速度环守住慢转，报出 q 轴电流里 6 次的内容。
 */
struct LoopResult final {
    float iq_order_a;
    float speed_order_rad_s;
    float iq_mean_a;
};

auto run_loop(float omega_mech, float amplitude_nm) -> LoopResult {
    Bench bench{standard_bench_config()};
    CoggingHarmonic table{static_cast<std::uint8_t>(kOrder), amplitude_nm, 0.0F};
    if (amplitude_nm > 0.0F) {
        bench.plant().set_cogging(&table, 1);
    }
    bench.motor().set_speed(omega_mech);
    EXPECT_EQ(foc::ErrorCode::Ok, bench.init_and_start());

    std::vector<float> theta;
    std::vector<float> iq;
    std::vector<float> speed;
    bench.run(0.5F); // let the loops settle before recording
    bench.run(2.0F, [&](Bench& b) {
        const foc::sil::Sample sample = b.sample();
        theta.push_back(b.plant().theta_mech_rad());
        iq.push_back(sample.iq_a);
        speed.push_back(sample.speed);
    });
    float mean = 0.0F;
    for (const float value : iq) {
        mean = static_cast<float>(mean + (value / static_cast<float>(iq.size())));
    }
    return LoopResult{harmonic_amplitude(theta, iq, kOrder), harmonic_amplitude(theta, speed, kOrder), mean};
}

} // namespace

// The compensator's rationale, as a number: at a slow rotation the cogging torque reappears in the q-axis current almost
// exactly as `T_cog / Kt`. That is the quantity a compensator has to supply *without waiting for a speed error* -- the loop
// produces it too, but only after the rotor has already been pushed, which is what "cogging" feels like.
// 补偿器的理由，一个数：在慢转时齿槽转矩几乎正好以 `T_cog / Kt` 重现于 q 轴电流。这正是补偿器要**不等速度误差**就供给的量
// ——环路也会产出它，但只在转子已经被推动之后，而那正是「齿槽」的手感。
TEST(SilCoggingLoop, AtLowSpeedTheCoggingReappearsInTheCurrentAsTorqueOverKt) {
    constexpr float kAmplitude = 4.0e-3F;
    const float kt = foc::sil::nominal_spec().kt;
    const float expected = kAmplitude / kt; // 4 mN*m / 0.05 N*m/A = 0.08 A

    const LoopResult off = run_loop(10.0F, 0.0F);
    const LoopResult on = run_loop(10.0F, kAmplitude);

    // With no injection there is no 6th harmonic worth the name.
    // 没有注入时就没有值得一提的 6 次成分。
    EXPECT_LT(off.iq_order_a, 2.0e-3F);
    // With it, the amplitude is the analytic one, to 5 % -- the loop does not merely notice the torque, it reproduces it in
    // the current at essentially unit gain, and that is the number a feedforward has to match.
    // 有注入时幅值就是解析值，误差 5 % 以内——环路不只是察觉到转矩，它几乎以单位增益把它复现在电流里，而那就是前馈必须对上的数。
    EXPECT_NEAR(expected, on.iq_order_a, expected * 0.05F) << "measured " << on.iq_order_a << " expected " << expected;
    EXPECT_GT(on.iq_order_a, off.iq_order_a * 20.0F);
}

// The boundary: the loop's ability to deliver that current falls off with the ripple frequency, because the ripple it is
// answering is a *speed* ripple and the speed loop is what answers it. Measured monotonically decreasing, so a compensator
// is not redundant at any of these speeds -- it is the only thing left when the loop can no longer follow.
// 边界：环路交付那份电流的能力随纹波频率下降，因为它回答的那个纹波是**速度**纹波、而回答它的是速度环。实测单调下降，所以补偿器
// 在这些转速上都不是多余的——当环路跟不上时，它是唯一剩下的东西。
TEST(SilCoggingLoop, TheLoopsShareOfTheCoggingFallsAsTheRippleFrequencyRises) {
    constexpr float kAmplitude = 4.0e-3F;
    const LoopResult slow = run_loop(10.0F, kAmplitude);
    const LoopResult medium = run_loop(40.0F, kAmplitude);
    const LoopResult fast = run_loop(80.0F, kAmplitude);

    EXPECT_GT(slow.iq_order_a, medium.iq_order_a);
    EXPECT_GT(medium.iq_order_a, fast.iq_order_a);
    // At 80 rad/s mechanical the 6th harmonic is 76 Hz, i.e. well above the 250 rad/s (40 Hz) speed loop, and the share has
    // roughly halved. Asserting "roughly halved" rather than "fell a little" is what makes this a boundary instead of an
    // observation: if it were still near unity, the compensator's whole argument would be weaker.
    // 80 rad/s 机械处 6 次是 76 Hz，远高于 250 rad/s（40 Hz）的速度环，份额大约减半。断言「大约减半」而不是「略有下降」，才让它
    // 成为一条边界而不是一句观察：若它仍接近单位，补偿器的整个理由都会变弱。
    const float ratio = fast.iq_order_a / slow.iq_order_a;
    EXPECT_GT(ratio, 0.30F);
    EXPECT_LT(ratio, 0.70F);
}

// A second, independent reading of the same event: the average q-axis current does not move when cogging is injected,
// because cogging averages to zero over a revolution. If it did move, the injection would be a load in disguise and every
// number above would be measuring a steady-state offset rather than a ripple.
// 同一件事的第二个独立读数：注入齿槽时 q 轴电流的**平均值**不动，因为齿槽在一转上平均为零。若它动了，注入就是伪装的负载，上面
// 每个数量的都会是稳态偏置而不是纹波。
TEST(SilCoggingLoop, TheInjectionMovesTheRippleAndNotTheAverage) {
    constexpr float kAmplitude = 4.0e-3F;
    const LoopResult off = run_loop(20.0F, 0.0F);
    const LoopResult on = run_loop(20.0F, kAmplitude);
    EXPECT_NEAR(off.iq_mean_a, on.iq_mean_a, std::fabs(off.iq_mean_a) * 0.1F + 1.0e-3F);
}
