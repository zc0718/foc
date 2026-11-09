// F05, the plant half: the switching inverter now refuses a pulse narrower than `t_min`, and `core::MinPulse` predicts
// what that costs. This file is where the prediction is charged against the model -- the two are written independently
// (the plant does not call the policy, the policy does not know the plant) so agreement is a measurement.
// F05 的 plant 半边：开关逆变器现在会拒送窄于 `t_min` 的脉冲，而 `core::MinPulse` 预测它的代价。本文件就是拿模型的数去
// 收这笔账——两边是独立写的（plant 不调用策略、策略不知道 plant），所以「一致」是一次测量。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/min_pulse.hpp>

#include "sil_discrete_inverter.hpp"

namespace {

using foc::core::MinPulse;
using foc::core::MinPulseConfig;
using foc::core::MinPulseOutput;
using foc::core::Uvw;

constexpr float kBusV = 24.0F;
constexpr float kSwHz = 10.0e3F;
constexpr float kPeriodS = 1.0F / kSwHz;
constexpr float kDeadS = 3.0e-6F; // 3 % of the period

auto request(float u, float v, float w) -> Uvw<float> { return Uvw<float>{u, v, w}; }

/**
 * @brief [en] Mean pole voltages over **one** switching period, V, with the current held at the sub-step value.
 * @brief [zh] **一个** 开关周期内的平均极电压，V，电流在每个子步保持不变。
 * @note [en] The inverter is stateless across periods by construction (section "carrier" in its own header), so one
 *            period is a complete measurement and no settling is needed.
 * @note [zh] 按构造这个逆变器不跨周期存状态（见它自己头文件里的「载波」一节），所以一个周期就是一次完整测量，不需要任何
 *            建立时间。
 */
auto mean_poles(Uvw<float> duty, float t_min_s, Uvw<float> i_uvw, float t_dead_s = 0.0F, int sub = 200) -> Uvw<float> {
    foc::sil::InverterConfig config{};
    config.f_sw_hz = kSwHz;
    config.t_dead_s = t_dead_s;
    config.v_fwd_v = 0.0F; // ideal devices: the only mechanisms left are the two being compared
    config.sub_per_switch = sub;
    config.t_min_s = t_min_s;
    foc::sil::DiscreteInverter inverter(config);

    const auto steps = inverter.begin_period(duty, kBusV, kPeriodS);
    EXPECT_GT(steps, 0);
    Uvw<float> sum{};
    for (std::int32_t k = 0; k < steps; ++k) {
        (void)inverter.step(i_uvw);
        const Uvw<float> poles = inverter.pole_voltages();
        sum.u += poles.u;
        sum.v += poles.v;
        sum.w += poles.w;
    }
    const float scale = (steps > 0) ? (1.0F / static_cast<float>(steps)) : 0.0F;
    return Uvw<float>{sum.u * scale, sum.v * scale, sum.w * scale};
}

} // namespace

// The law, in the plant rather than in the policy: one phase is asked for a pulse too short to exist, one is asked for a
// gap too short to exist, one is in the middle. The mean pole voltages are the *transmitted* duties times the bus, and
// the error against the request is the policy's `lost_vs / T`, to the last digit.
// 律，这次在 plant 里而不是在策略里：一相被要求送一个短到不存在的脉冲，一相被要求留一个短到不存在的间隔，一相落在中间。
// 平均极电压就是 **送出** 的占空比乘母线，而它与请求之差正好等于策略的 `lost_vs / T`，到最后一位。
TEST(SilMinPulse, ThePlantTransmitsWhatTheDriverAllowsAndThePolicyKnowsThePrice) {
    const float min_pulse_s = 0.10F * kPeriodS;
    const Uvw<float> duty = request(0.02F, 0.50F, 0.98F);
    const Uvw<float> currents = request(1.0F, -1.0F, 1.0F); // no dead time here, so only the driver's rule acts
    const Uvw<float> measured = mean_poles(duty, min_pulse_s, currents);

    const MinPulse policy = MinPulse{MinPulseConfig{min_pulse_s, kPeriodS}};
    const MinPulseOutput predicted = policy.apply(duty, kBusV);

    std::printf("\n[minpulse] plant mean poles: u %.4f V, v %.4f V, w %.4f V | requested: u %.4f, v %.4f, w %.4f\n",
                static_cast<double>(measured.u), static_cast<double>(measured.v), static_cast<double>(measured.w),
                static_cast<double>(duty.u * kBusV), static_cast<double>(duty.v * kBusV),
                static_cast<double>(duty.w * kBusV));

    // What the bridge ran, read straight off the plant, is the policy's answer -- including the phase that was stretched
    // to a full period because its *gap* was too short.
    // 桥臂实际跑的，直接从 plant 读出来，就是策略的答案——包括那一相因为 **间隔** 太短而被拉成整周期的。
    EXPECT_NEAR(static_cast<double>(predicted.duty.u), static_cast<double>(measured.u / kBusV), 1.0e-6);
    EXPECT_NEAR(static_cast<double>(predicted.duty.v), static_cast<double>(measured.v / kBusV), 1.0e-6);
    EXPECT_NEAR(static_cast<double>(predicted.duty.w), static_cast<double>(measured.w / kBusV), 1.0e-6);

    // And the price, in the same units: the volt-seconds the policy reported over one period are the mean volts missing.
    // 以及代价，用同样的单位：策略报的一个周期的伏秒，就是平均少掉的伏数。
    EXPECT_NEAR(static_cast<double>(predicted.lost_vs.u) / static_cast<double>(kPeriodS),
                static_cast<double>(duty.u * kBusV) - static_cast<double>(measured.u), 1.0e-5);
    EXPECT_NEAR(static_cast<double>(predicted.lost_vs.w) / static_cast<double>(kPeriodS),
                static_cast<double>(duty.w * kBusV) - static_cast<double>(measured.w), 1.0e-5);
    // The two that are actually wrong are the two that straddle the rail: 0.48 V short and 0.48 V long, which cancel in the
    // *vector* for this particular request -- stated because a reader will otherwise assume a dropped pulse is visible in
    // the Clarke transform, and here it is not.
    // 真正错掉的两相正好一正一反：少 0.48 V、多 0.48 V，对 **这个** 请求它们在矢量上互相抵消——写下来是因为否则读的人会以为
    // 丢脉冲在 Clarke 变换里看得见，而这里恰好看不见。
    EXPECT_NEAR(0.48, static_cast<double>(duty.u * kBusV) - static_cast<double>(measured.u), 0.01);
    EXPECT_NEAR(-0.48, static_cast<double>(duty.w * kBusV) - static_cast<double>(measured.w), 0.01);
}

// The batch's falsifier, now on the plant: ten times the limit and the delivered voltage leaves the bound that was
// written down first. The pulse is not "a bit worse" -- it is gone.
// 本批次的证伪，这回在 plant 上：极限抬十倍，交付电压就离开 **事先** 写下的界。脉冲不是「差一点」，而是没了。
TEST(SilMinPulse, TenTimesTheLimitTakesThePulseOffThePlant) {
    const Uvw<float> duty = request(0.12F, 0.12F, 0.12F);
    const Uvw<float> currents = request(1.0F, 1.0F, -2.0F);
    const float bound_v = 1.0F; // written down before the measurement

    const Uvw<float> loose = mean_poles(duty, 0.05F * kPeriodS, currents);
    const Uvw<float> tight = mean_poles(duty, 0.50F * kPeriodS, currents);

    std::printf("[minpulse] plant at d = 0.12: t_min 5 %% -> u %.4f V | t_min 50 %% -> u %.4f V | bound %.3f V\n",
                static_cast<double>(loose.u), static_cast<double>(tight.u), static_cast<double>(bound_v));

    EXPECT_NEAR(0.12 * static_cast<double>(kBusV), static_cast<double>(loose.u), 1.0e-4) << "5 % limit transmits";
    EXPECT_NEAR(0.0, static_cast<double>(tight.u), 1.0e-6) << "50 % limit does not";
    EXPECT_GT(std::fabs(static_cast<double>(loose.u - tight.u)), static_cast<double>(bound_v));
}

// The residue, closed on the plant: 100 periods of a request that is half of the smallest transmissible pulse. Without a
// carry every period is dropped and the plant's mean is zero; with one, the plant's mean over the window is the request.
// 余量，在 plant 上闭合：100 个周期，请求是最小可传脉冲的一半。不带余量时每周期都被丢掉、plant 的平均是零；带上余量，
// 窗口上的平均就是请求本身。
TEST(SilMinPulse, TheCarryHoldsThePlantAverageWhereTheRequestAskedForIt) {
    constexpr int kPeriods = 100;
    const float min_pulse_s = 0.10F * kPeriodS;
    const float d = 0.05F;
    const Uvw<float> request_uvw = request(d, d, d);
    const Uvw<float> currents = request(1.0F, 0.0F, -1.0F);

    MinPulse shaped = MinPulse{MinPulseConfig{min_pulse_s, kPeriodS}};
    double shaped_sum = 0.0;
    double plain_sum = 0.0;
    for (int k = 0; k < kPeriods; ++k) {
        const MinPulseOutput out = shaped.shape(request_uvw, kBusV);
        shaped_sum += static_cast<double>(mean_poles(out.duty, min_pulse_s, currents).u);
        plain_sum += static_cast<double>(mean_poles(request_uvw, min_pulse_s, currents).u);
    }
    const double shaped_mean = shaped_sum / kPeriods;
    const double plain_mean = plain_sum / kPeriods;
    std::printf("[minpulse] plant over %d periods at d = %.3f: with the carry %.5f V, without it %.5f V, "
                "requested %.5f V\n",
                kPeriods, static_cast<double>(d), shaped_mean, plain_mean, static_cast<double>(d * kBusV));

    EXPECT_NEAR(static_cast<double>(d * kBusV), shaped_mean, 1.0e-4)
        << "the average is the request, measured on the plant";
    EXPECT_NEAR(0.0, plain_mean, 1.0e-6) << "and without the carry the pulse never appears at all";
}

// The two mechanisms do **not** separate by subtraction, and this case measures how far apart they are rather than
// claiming they are independent. With dead time on, a dropped pulse also removes the two edges its windows sat on, so the
// textbook `-(t_dead/T)*v_bus*sign(i)` no longer describes the period. The remainder is bounded by one dead window, and
// that bound is what is asserted.
// 两个机制 **不能** 靠相减分离，这条用例量的是它们差多少，而不是宣称它们独立。死区开着时，一个被丢掉的脉冲同时去掉了死区
// 窗口所在的两个边沿，于是教科书的 `−(t_dead/T)·v_bus·sign(i)` 不再描述这个周期。残差被一个死区窗口界住，断言的是那条界。
TEST(SilMinPulse, ADroppedPulseAndTheDeadTimeDoNotSeparateBySubtraction) {
    const Uvw<float> duty = request(0.01F, 0.50F, 0.70F);
    const Uvw<float> currents = request(2.0F, -1.0F, -1.0F); // signs decide which rail the dead windows clamp to
    const double dead_window_v = static_cast<double>((kDeadS / kPeriodS) * kBusV);

    const Uvw<float> with_dead = mean_poles(duty, 0.10F * kPeriodS, currents, kDeadS);
    const Uvw<float> without = mean_poles(duty, 0.0F, currents, kDeadS);
    const Uvw<float> no_dead_reference = mean_poles(duty, 0.0F, currents, 0.0F);

    const double pure_minpulse_v = static_cast<double>(duty.u * kBusV) - static_cast<double>(without.u);
    const double dead_alone_v = static_cast<double>(no_dead_reference.u - with_dead.u);
    const double combined_v = static_cast<double>(without.u) - static_cast<double>(with_dead.u);
    std::printf("[minpulse] dropped pulse: min-pulse alone %.4f V, dead time alone %.4f V, both %.4f V, "
                "naive sum %.4f V, |sum - both| %.4f V (one dead window = %.4f V)\n",
                pure_minpulse_v, dead_alone_v, combined_v, pure_minpulse_v + dead_alone_v,
                std::fabs(pure_minpulse_v + dead_alone_v - combined_v), dead_window_v);

    EXPECT_GT(pure_minpulse_v, 0.2) << "the dropped pulse really is the dominant term here";
    // The two do not add up: a pulse the driver never sent has no edges, so the dead time has nothing to sit on and its
    // own share disappears with it. That is the whole reason this case exists -- it is the batch's stated correction point,
    // measured rather than argued.
    // 两者不相加：驱动器没送出的脉冲没有边沿，死区也就没有落脚处，它那一份随脉冲一起消失。这正是这条用例存在的理由——它
    // 是本批次写下的纠错点，用测量而不是用论证给出。
    EXPECT_GT(std::fabs(pure_minpulse_v + dead_alone_v - combined_v), 0.2)
        << "the effects double-count, they do not add";
    EXPECT_LT(std::fabs(pure_minpulse_v - combined_v), dead_window_v)
        << "and what is left over is bounded by the dead window the pulse took with it";
}
