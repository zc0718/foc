// S7 - multi-rate. The speed loop is decimated by elapsed time, not by a tick count, so a 10x or
// 20x slower outer loop must still meet the speed-step acceptance thresholds. This is the closed-loop
// counterpart of Motor.SpeedLoopDecimationKeepsTheIntegralOverTheSameElapsedTime: that one proves the
// integral is preserved, this one proves the loop still behaves.
// S7 - 多速率。速度环按累计时间降采样，而不是按 tick 计数，因此外环慢 10 倍或 20 倍时仍须满足 S2
// 的验收阈值。这是 Motor.SpeedLoopDecimationKeepsTheIntegralOverTheSameElapsedTime 的闭环对照：
// 那条证明积分守恒，这条证明环路行为不变。

#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"



namespace {

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 4;
constexpr int kPolePairs = 7;
constexpr float kVBus = 24.0F;
constexpr float kOmegaCurrent = 2000.0F;
constexpr float kOmegaSpeed = 250.0F;
constexpr float kTargetSpeed = 100.0F;
constexpr float kRunSeconds = 0.40F;
constexpr float kSettleBand = 0.02F;
constexpr float kSettleFloorS = 0.05F;

struct Outcome final {
    float overshoot_pct;
    float settle_s;
    float steady_err_pct;
    foc::ErrorCode status;
};

/**
 * @brief [en] One speed step with the outer loop running every `speed_loop_period_s` seconds (0 = every tick).
 * @brief [zh] 以 `speed_loop_period_s` 秒为周期（0 = 每 tick）跑一次速度阶跃。
 */
auto step_speed(float speed_loop_period_s) -> Outcome {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal>;
    foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
    mc.speed_loop_period_s = speed_loop_period_s;

    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
    foc::sil::PlantHal hal(plant, kVBus);
    const typename MotorT::Policies policies{foc::core::BackEmfObserver({mc.rs, mc.ld}),
                                             foc::core::Svpwm{},
                                             foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                             foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                             foc::core::Pi{foc::speed_pi_config(mc, kOmegaSpeed)}};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_speed(kTargetSpeed);
    static_cast<void>(motor.start());

    float peak = 0.0F;
    float settle = 0.0F;
    const int steps = static_cast<int>(kRunSeconds / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        const float t = static_cast<float>(i + 1) * kDt;
        const float w = motor.telemetry().speed_mech_rad_s;
        if (w > peak) {
            peak = w;
        }
        if ((t > kSettleFloorS) && (std::fabs(w - kTargetSpeed) > (kSettleBand * kTargetSpeed))) {
            settle = t; // last moment it was outside the band
        }
    }

    const float final_speed = motor.telemetry().speed_mech_rad_s;
    return Outcome{100.0F * (peak - kTargetSpeed) / kTargetSpeed, settle,
                   100.0F * std::fabs(final_speed - kTargetSpeed) / kTargetSpeed, motor.status()};
}

} // namespace



/**
 * @brief [en] A decimated outer loop must keep the speed-step acceptance thresholds, and must not change the
 *             closed-loop response by more than a fixed factor. The speed-loop bandwidth here is 250 rad/s (a 4 ms time
 *             constant), so 500 us and even 1 ms remain oversampled -- that is exactly why the decimation is allowed to
 *             be this coarse, and the test pins that reasoning.
 * @brief [zh] 降采样的外环必须保持 S2 的验收阈值，且不能把闭环响应改变超过一个固定倍数。这里的速度环 带宽是 250 rad/s（
 *             时间常数 4 ms），因此 500 us 甚至 1 ms 仍然是过采样——这正是降采样可以 这么粗的理由，用例把这条推理钉住。
 */
TEST(SilMultirate, DecimatedSpeedLoopKeepsTheS2Thresholds) {
    const Outcome every = step_speed(0.0F);
    const Outcome dec10 = step_speed(10.0F * kDt);
    const Outcome dec20 = step_speed(20.0F * kDt);

    for (const Outcome& outcome : {every, dec10, dec20}) {
        EXPECT_EQ(foc::ErrorCode::Ok, outcome.status);
        EXPECT_LE(outcome.overshoot_pct, 10.0F);  // S2: overshoot <= 10%
        EXPECT_LE(outcome.steady_err_pct, 1.0F);  // S2: steady-state error <= 1%
        EXPECT_GT(outcome.settle_s, 0.0F);        // it did leave the band and come back
    }

    // The response may shift slightly, but not by an order of magnitude.
    EXPECT_NEAR(every.settle_s, dec10.settle_s, 0.5F * every.settle_s);
    EXPECT_NEAR(every.settle_s, dec20.settle_s, 0.5F * every.settle_s);
    EXPECT_NEAR(every.overshoot_pct, dec20.overshoot_pct, 1.0F);
}



namespace {

constexpr std::uint32_t kTraceCapacity = 12000U;

struct Decimated final {
    /**
     * @brief [en] measured spacing between outer-loop steps, in control ticks.
     * @brief [zh] 两次外环步之间实测的控制拍数。
     */
    float period_ticks;
    float overshoot_pct;
    float settle_s;
    float steady_err_pct;
};

/**
 * @brief [en] A speed step with the outer loop's *instants* observed through the trace channel.
 *
 * @brief [zh] 一次速度阶跃，外环的**步进时刻**通过轨迹通道观测。
 *
 *   The spacing is read from where `i_ref_q` moves, because the current reference is recomputed only on an outer tick:
 *   between them the motor holds it. That makes the decimation a measured quantity rather than a restatement of the
 *   configuration. 间距从 `i_ref_q` 发生变化的位置读出，因为电流给定只在外环步进时重算，两步之间电机保持它。于是降采样
 *   成为一个被量出来的量，而不是把配置复述一遍。
 */
auto decimated_step(float speed_loop_period_s, foc::core::TraceSample* buffer) -> Decimated {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi,
                                     foc::sil::PlantHal, foc::core::NullReferenceGenerator,
                                     foc::core::NullFeedforward, foc::core::NullPositionControl,
                                     foc::core::NullAlignment, foc::core::SpanTrace>;
    foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F,
                               20000.0F, 0.0F};
    mc.speed_loop_period_s = speed_loop_period_s;
    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
    foc::sil::PlantHal hal(plant, kVBus);
    typename MotorT::Policies policies{foc::core::BackEmfObserver({mc.rs, mc.ld}),
                                       foc::core::Svpwm{},
                                       foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                       foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                       foc::core::Pi{foc::speed_pi_config(mc, kOmegaSpeed)}};
    policies.trace.attach(buffer, kTraceCapacity);
    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_speed(kTargetSpeed);
    static_cast<void>(motor.start());

    float peak = 0.0F;
    float settle = 0.0F;
    const int steps = static_cast<int>(kRunSeconds / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        const float t = static_cast<float>(i + 1) * kDt;
        const float w = motor.telemetry().speed_mech_rad_s;
        peak = std::max(peak, w);
        if ((t > kSettleFloorS) && (std::fabs(w - kTargetSpeed) > (kSettleBand * kTargetSpeed))) {
            settle = t;
        }
    }
    const float final_speed = motor.telemetry().speed_mech_rad_s;

    // The outer loop's instants: where the current reference moves. The smallest gap is the period,
    // because a saturated or steady run can repeat the same value twice in a row.
    // 外环的步进时刻：电流给定发生变化的地方。最小间距就是周期，因为饱和或稳态时同一个值可能连着
    // 出现两次。
    const std::uint32_t count = motor.trace().size();
    std::uint32_t last = 0U;
    std::uint32_t smallest = 0xFFFFFFFFU;
    for (std::uint32_t k = 1U; k < count; ++k) {
        if (motor.trace().at(k - 1U).i_ref_q != motor.trace().at(k).i_ref_q) {
            if (k > last) {
                smallest = std::min(smallest, k - last);
            }
            last = k;
        }
    }
    return Decimated{static_cast<float>(smallest), 100.0F * (peak - kTargetSpeed) / kTargetSpeed, settle,
                     100.0F * std::fabs(final_speed - kTargetSpeed) / kTargetSpeed};
}

} // namespace



/**
 * @brief [en] The requested period is a *lower bound*: the loop runs on whole control ticks, so the actual period is
 *             `dt*ceil(T/dt)`, and the float accumulation of `dt` can add one more.
 *
 * @brief [zh] 请求的周期是一个**下界**：环路按整控制拍运行，所以实际周期是 `dt*ceil(T/dt)`，而 `dt` 的浮点累加还可能再
 *             加一拍。
 *
 *   This is the row's sensitivity coefficient for the loop period, and it is a trap rather than a nicety: asking for
 *   1.2 ticks gets 2. The accumulation is the second half. `speed_elapsed_s_` is a float32 that accumulates `dt` and
 *   compares against the period, so when the rounded sum falls just short of the request the loop waits an extra tick
 *   -- measured at 20x, 50x, 100x and 160x, where 8.000 ms becomes 8.050 ms, while 5x and 10x land exactly. A float32
 *   sum of twenty 50 us steps is not 1 ms to the last bit, and the difference shows up as one control tick.
 *
 *   这就是本行关于环路周期的敏感度系数，而且它是陷阱而不是细节：要 1.2 拍会得到 2 拍。浮点累加是另一半。
 *   `speed_elapsed_s_` 是一个累加 `dt` 并与周期比较的 float32，于是当舍入后的和比请求略小时，环路会多等一拍——在 20×、
 *   50×、100×、160× 上量到，8.000 ms 变成 8.050 ms，而 5× 与 10× 恰好落在点上。二十个 50 µs 的 float32 之和不是精确的 1
 *   ms，差一拍就是这么来的。
 */
TEST(SilMultirate, TheRequestedPeriodIsALowerBoundAndTheTickIsTheUnit) {
    static foc::core::TraceSample buffer[kTraceCapacity];
    const struct {
        float requested_ticks;
        const char* note;
    } requests[] = {{1.0F, "every tick"},  {1.2F, "1.2 ticks becomes 2"},
                    {1.5F, "1.5 becomes 2"}, {2.0F, "exact"},
                    {5.0F, "exact"},         {10.0F, "exact"},
                    {20.0F, "float sum falls short"}, {50.0F, "float sum falls short"}};

    std::printf("\n%-12s %12s %12s %10s\n", "requested_us", "actual_us", "ceil_us", "extra ticks");
    for (const auto& request : requests) {
        const float period = request.requested_ticks * kDt;
        const Decimated run = decimated_step(period, buffer);
        const float ceil_ticks = std::ceil(request.requested_ticks - 1.0e-4F);
        const float actual_ticks = run.period_ticks;
        std::printf("%-12.1f %12.1f %12.1f %10.2f  (%s)\n", static_cast<double>(request.requested_ticks),
                    static_cast<double>(actual_ticks), static_cast<double>(ceil_ticks),
                    static_cast<double>(actual_ticks - ceil_ticks), request.note);

        // The unit is the control tick: the loop cannot run more often than every tick, and it never
        // runs on a fractional one.
        // 单位是控制拍：环路不可能比每拍更频繁，也不会跑在小数拍上。
        EXPECT_GE(actual_ticks, ceil_ticks) << "the requested period is a lower bound";
        EXPECT_LE(actual_ticks, ceil_ticks + 1.0F)
            << "and at most one tick of float accumulation may be added";
        EXPECT_FLOAT_EQ(actual_ticks, std::floor(actual_ticks)) << "the period is a whole number of ticks";
        // And it really is decimation: the outer loop steps less often than the control loop.
        // 而且它确实是降采样：外环比控制环步进得少。
        EXPECT_LE(actual_ticks, static_cast<float>(kRunSeconds / kDt)) << "the trace must hold a run";
    }

    // The trap, named: 1.2 ticks is not 1.2 ticks.
    const Decimated one_point_two = decimated_step(1.2F * kDt, buffer);
    const Decimated two = decimated_step(2.0F * kDt, buffer);
    std::printf("[trap] 1.2 ticks and 2.0 ticks both run every %.0f tick(s)\n",
                static_cast<double>(one_point_two.period_ticks));
    EXPECT_FLOAT_EQ(two.period_ticks, one_point_two.period_ticks)
        << "there is no period between one and two ticks, so the finer request buys nothing";
}



/**
 * @brief [en] Sampling the outer loop changes nothing until `omega_b*T` reaches 2, and the response breaks exactly
 *             there.
 *
 * @brief [zh] 外环采样在 `omega_b*T` 到达 2 之前什么也不改变，而响应恰在那里坏掉。
 *
 *   The z-domain model is the row's second source, and it is independent of the code: with the plant reduced to its
 *   inertia (`Kt/(J*s)`, the speed loop's own design cancels the `J/b` pole) and a zero-order hold, the open loop is
 *   `L(z) = [omega_b*T*(z-1) + omega_b*T^2/tau_b]/(z-1)^2`, so
 *
 *   - the **slow** root sits at `z = 1 - omega_b*T`, i.e. a real-time time constant of `T/(1-z) = 1/omega_b` --
 *   **independent of T**. That is the whole reason decimation is allowed to be coarse: the integral is rescaled by the
 *   elapsed time, so the loop's speed is a property of the gains and not of the sampling. - the **fast** root runs off
 *   to `z = -1` at `omega_b*T = 2`, which is marginal stability. With `omega_b = 250 rad/s` and `dt = 50 us` that is `T
 *   = 8 ms`, i.e. **160x** decimation.
 *
 *   Measured: the settle time is flat from 1x to 140x (0.0685 s against 0.0690 s, 0.7 %) and then breaks -- 0.1058 s at
 *   160x and 0.5000 s at 180x, with the overshoot going 1.73 % -> 3.62 % -> 7.28 %. The boundary is not a tuning
 *   constant anywhere in the code; it falls out of the model.
 *
 *   z 域模型是本行的第二来源，且与代码无关：把 plant 归约为惯量（`Kt/(J*s)`，速度环自身的设计把 `J/b` 极点对消掉了）并
 *   加零阶保持，开环为 `L(z) = [omega_b*T*(z-1) + omega_b*T^2/tau_b]/(z-1)^2`，于是
 *
 *   - **慢**根位于 `z = 1 - omega_b*T`，即实时时间常数 `T/(1-z) = 1/omega_b`——**与 T 无关**。这正是降采样可以这么粗的全
 *   部理由：积分按经过时间重新标度，所以环路的快慢是增益的属性、不是采样的属性。 - **快**根在 `omega_b*T = 2` 处跑到 `z
 *   = -1`，即临界稳定。`omega_b = 250 rad/s`、`dt = 50 µs` 对应 `T = 8 ms`，也就是 **160×** 降采样。
 *
 *   实测：稳定时间从 1× 到 140× 都是平的（0.0685 对 0.0690，差 0.7 %），之后断掉——160× 时 0.1058 s、180× 时 0.5000 s，
 *   超调 1.73 % → 3.62 % → 7.28 %。这条边界不是代码里的任何整定常数，它是从模型里掉出来的。
 */
TEST(SilMultirate, TheResponseIsFlatUntilOmegaBandTReachesTwo) {
    static foc::core::TraceSample buffer[kTraceCapacity];
    const float limit_ticks = 2.0F / (kOmegaSpeed * kDt);
    std::printf("\n[boundary] omega_b %0.0f rad/s, dt %0.0f us -> the model puts marginal stability at "
                "%0.0f ticks (%.1f ms)\n",
                static_cast<double>(kOmegaSpeed), static_cast<double>(kDt * 1.0e6),
                static_cast<double>(limit_ticks), static_cast<double>(2.0 / kOmegaSpeed * 1.0e3));

    float flat_settle = 0.0F;
    for (const float factor : {1.0F, 10.0F, 20.0F, 100.0F, 140.0F}) {
        const Decimated run = decimated_step(factor * kDt, buffer);
        std::printf("%6.0fx  period %6.2f ticks  overshoot %5.2f %%  settle %.4f s  steady %5.3f %%\n",
                    static_cast<double>(factor), static_cast<double>(run.period_ticks),
                    static_cast<double>(run.overshoot_pct), static_cast<double>(run.settle_s),
                    static_cast<double>(run.steady_err_pct));
        // Well inside the model's boundary the response must not move with the sampling.
        // 在模型边界以内，响应不能随采样变化。
        EXPECT_LE(run.overshoot_pct, 5.0F) << "inside the boundary the step must stay well behaved";
        EXPECT_LE(run.settle_s, 0.09F) << "and the settling time is 1/omega_b, independent of T";
    }
    flat_settle = decimated_step(kDt, buffer).settle_s;

    // The two decimations the original case already covers are still inside the boundary, which is why
    // they pass: 10x and 20x are 0.5 ms and 1 ms against an 8 ms limit.
    // 原用例覆盖的那两档仍在边界以内，这正是它们能过的原因：10× 与 20× 是 0.5 ms 与 1 ms，限值是 8 ms。
    EXPECT_LT(20.0F, limit_ticks);

    // And past it the response breaks, on the side the model says.
    // 越过它之后响应就坏掉，而且坏在模型说的那一侧。
    const Decimated near = decimated_step(140.0F * kDt, buffer);
    const Decimated at = decimated_step(160.0F * kDt, buffer);
    const Decimated past = decimated_step(180.0F * kDt, buffer);
    std::printf("%6.0fx  period %6.2f ticks  overshoot %5.2f %%  settle %.4f s\n", 160.0,
                static_cast<double>(at.period_ticks), static_cast<double>(at.overshoot_pct),
                static_cast<double>(at.settle_s));
    std::printf("%6.0fx  period %6.2f ticks  overshoot %5.2f %%  settle %.4f s\n", 180.0,
                static_cast<double>(past.period_ticks), static_cast<double>(past.overshoot_pct),
                static_cast<double>(past.settle_s));

    EXPECT_LT(near.settle_s, 0.09F) << "140x is still inside the boundary";
    EXPECT_GT(at.settle_s, 1.5F * flat_settle) << "at 160x the model's marginal point, the response moves";
    EXPECT_GT(past.settle_s, 3.0F * flat_settle) << "past it the response really degrades";
    EXPECT_GT(past.overshoot_pct, 2.0F * near.overshoot_pct) << "and the overshoot goes with it";
}
