// Conan::ImportStart
#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/brake_chopper.hpp>
#include <foc/core/bus_current_limit.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_bus.hpp"
#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"
// Conan::ImportEnd

namespace {

// The bus current ceiling, on the bench where it can be measured and where the command can be scaled.
//
// The law is feedback, so the interesting questions are the ones feedback makes interesting: does it actually hold the
// ceiling, does the same bench exceed it with the limiter out of the path, and does the authority come back at the rate
// it was configured to. Each of those is measured against a number this file computed itself -- a peak it first measured
// with no limit at all, and a step count it derives from the configured rate.
//
// 母线电流上限，在能测到它、也能缩放指令的台架上。
//
// 律是反馈，所以有意义的问题都是反馈带来的：它真的压住了没有、同一个台架在限制器不在路径里时会不会超限、以及权限是不是
// 按配置的速率回来的。每一条都对账到本文件自己算出来的一个数——它先在不加限制时量到的峰值，以及由配置速率推出来的步数。

constexpr float kDt = 50e-6F;
constexpr int kSubsteps = 4;
constexpr int kPolePairs = 7;
constexpr float kVBus = 24.0F;

using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal>;
using foc::core::BusCurrentLimit;
using foc::core::BusCurrentLimitConfig;
using foc::core::BusCurrentLimitOutput;

auto motor_config() -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
}

struct Bench final {
    explicit Bench(float v_bus)
        : plant(foc::sil::PlantParams{motor_config().rs, motor_config().ld, motor_config().lq, motor_config().kt,
                                      motor_config().j, motor_config().b, kPolePairs}),
          hal(plant, v_bus),
          motor(motor_config(),
                MotorT::Policies{foc::core::BackEmfObserver({motor_config().rs, motor_config().ld}), foc::core::Svpwm{},
                                 foc::core::Pi{foc::current_pi_config(motor_config(), 2000.0F, v_bus)},
                                 foc::core::Pi{foc::current_pi_config(motor_config(), 2000.0F, v_bus)},
                                 foc::core::Pi{foc::speed_pi_config(motor_config(), 250.0F)}},
                hal) {}

    [[nodiscard]] auto bus_current_a() -> float {
        float amps = 0.0F;
        const bool ok = hal.dc_link.read_bus_current_a(amps);
        return ok ? amps : 0.0F;
    }

    foc::sil::PmsmPlant plant;
    foc::sil::PlantHal hal;
    MotorT motor;
};

auto limit_config(float i_max_a) -> BusCurrentLimitConfig {
    BusCurrentLimitConfig config{};
    config.i_max_a = i_max_a;
    config.headroom_a = 0.05F * i_max_a;
    config.gain_per_a_s = 400.0F;  // authority per ampere-second: a tenth of full authority per 0.25 A of error
    config.fall_per_s = 200.0F;
    config.rise_per_s = 2.0F;
    return config;
}

/**
 * @brief Run the bench for `ticks` with a torque command, optionally through the limiter, and report the peak bus current.
 */
struct BenchRun final {
    double peak_a;
    double settled_a;  // mean over the last tenth of the run
    float last_scale;
    int ticks_limited;
    foc::sil::BusReport report;
};

auto drive(double torque_a, float i_max_a, bool limit, int ticks) -> BenchRun {
    foc::sil::BusMeter meter(foc::sil::BusParams{0.0F, kVBus, 0.0F, 0.0F, 0.0F});
    Bench bench(kVBus);
    bench.hal.set_bus(&meter);

    BusCurrentLimit limiter;
    if (limit) {
        EXPECT_EQ(foc::ErrorCode::Ok, limiter.configure(limit_config(i_max_a)));
    }
    EXPECT_EQ(foc::ErrorCode::Ok, bench.motor.init());
    EXPECT_EQ(foc::ErrorCode::Ok, bench.motor.start());

    BenchRun run{0.0, 0.0, 1.0F, 0, {}};
    double tail_sum = 0.0;
    int tail_count = 0;
    const int tail_from = (ticks * 9) / 10;
    for (int k = 0; k < ticks; ++k) {
        float scale = 1.0F;
        if (limit) {
            const auto& out = limiter.step(bench.bus_current_a(), kDt);
            scale = out.scale;
            run.last_scale = out.scale;
            if (out.limiting) {
                ++run.ticks_limited;
            }
        }
        bench.motor.set_torque(torque_a * scale);
        EXPECT_EQ(foc::ErrorCode::Ok, bench.motor.tick(kDt));
        bench.hal.advance(kDt, kSubsteps);
        const double amps = static_cast<double>(bench.bus_current_a());
        run.peak_a = (amps > run.peak_a) ? amps : run.peak_a;
        if (k >= tail_from) {
            tail_sum += amps;
            ++tail_count;
        }
    }
    run.settled_a = (tail_count > 0) ? (tail_sum / tail_count) : 0.0;
    run.report = meter.report();
    return run;
}

// 1. Three numbers that cannot describe a band are refused, and an unconfigured limiter passes everything through.
TEST(BusCurrentLimit, AConfigurationThatCannotBeABandIsRefused) {
    {
        BusCurrentLimit limiter;
        BusCurrentLimitConfig config = limit_config(1.0F);
        config.i_max_a = 0.0F;
        EXPECT_EQ(foc::ErrorCode::Param, limiter.configure(config));
    }
    {
        BusCurrentLimit limiter;
        BusCurrentLimitConfig config = limit_config(1.0F);
        config.headroom_a = config.i_max_a;
        EXPECT_EQ(foc::ErrorCode::Param, limiter.configure(config)) << "aiming at zero leaves no authority at all";
    }
    {
        BusCurrentLimit limiter;
        BusCurrentLimitConfig config = limit_config(1.0F);
        config.rise_per_s = 0.0F;
        EXPECT_EQ(foc::ErrorCode::Param, limiter.configure(config));
    }
    {
        BusCurrentLimit limiter;
        const BusCurrentLimitOutput& out = limiter.step(10.0F, kDt);
        EXPECT_FLOAT_EQ(1.0F, out.scale) << "unconfigured is not 'clamp everything'";
        EXPECT_FALSE(out.limiting);
    }
}

// 2. No measurement is not a pass: the authority relaxes and the output says it was never told.
TEST(BusCurrentLimit, AMissingMeasurementRelaxesTheAuthorityAndSaysItWasNeverTold) {
    BusCurrentLimit limiter;
    ASSERT_EQ(foc::ErrorCode::Ok, limiter.configure(limit_config(1.0F)));
    // Pull the authority down first, so the relaxation has somewhere to go.
    // 先把权限拉下来，好让松开有地方可去。
    for (int k = 0; k < 20; ++k) {
        static_cast<void>(limiter.step(2.0F, kDt));
    }
    const float squeezed = limiter.output().scale;
    ASSERT_LT(squeezed, 0.9F);

    for (int k = 0; k < 200; ++k) {
        const auto& out = limiter.step(std::nanf(""), kDt);
        EXPECT_FALSE(out.measured) << "silence has to be visible, not implied";
    }
    EXPECT_GT(limiter.output().scale, squeezed) << "a dead shunt must not keep the drive strangled";
    EXPECT_LE(limiter.output().scale, 1.0F);
}

// 3. Below the aim the command is untouched, and above it the authority is never boosted.
TEST(BusCurrentLimit, BelowTheAimNothingHappens) {
    BusCurrentLimit limiter;
    ASSERT_EQ(foc::ErrorCode::Ok, limiter.configure(limit_config(1.0F)));
    for (int k = 0; k < 100; ++k) {
        const auto& out = limiter.step(0.5F, kDt);  // half the ceiling
        ASSERT_TRUE(out.measured);
        ASSERT_FLOAT_EQ(1.0F, out.scale);
        ASSERT_FALSE(out.limiting);
    }
}

// 4. The ceiling holds on the bench, and the same bench exceeds it with the limiter out of the path.
TEST(BusCurrentLimit, TheCeilingHoldsOnTheBenchAndTheSameBenchExceedsItWithout) {
    // Long enough for the integrator to arrive: the ceiling is not a step, and the authority is earned down by an
    // error that takes time to build. 60 ms of 20 kHz is 1200 ticks and the loop is still moving at that point.
    // 足够长，让积分器走到位：上限不是一步到位的，权限是被需要时间积累的误差压下来的。20 kHz 下 60 ms 是 1200 拍，
    // 而那时环路仍在移动。
    constexpr int kTicks = 4000;  // 200 ms
    constexpr float kTorque = 2.0F;

    // The ceiling comes from a baseline this file measured, not from a number picked to look good: measure first, then
    // ask the limiter to hold 60 % of what the same command drew without it.
    // 上限来自本文件自己量到的基线，不是挑来好看的一个数：先量，再要求限制器压住同一条指令在不加限制时抽到的峰值的 60 %。
    const BenchRun unlimited = drive(kTorque, 0.0F, /*limit=*/false, kTicks);
    ASSERT_GT(unlimited.peak_a, 0.0) << "the baseline has to draw something";
    const float ceiling_a = static_cast<float>(0.6 * unlimited.peak_a);
    std::printf("\n[bus limit] unlimited peak %.4f A, ceiling set to %.4f A\n", unlimited.peak_a,
                static_cast<double>(ceiling_a));

    const BenchRun limited = drive(kTorque, ceiling_a, /*limit=*/true, kTicks);
    std::printf("[bus limit] limited peak %.4f A, settled %.4f A, scale %.3f, limited ticks %d\n", limited.peak_a,
                limited.settled_a, static_cast<double>(limited.last_scale), limited.ticks_limited);
    EXPECT_GT(limited.ticks_limited, 0) << "the limiter has to have been doing something";
    EXPECT_LT(limited.peak_a, unlimited.peak_a) << "it has to be lower than the run with no limiter at all";
    // What a limit can promise, and what it cannot: it holds a **continuous** draw at the ceiling, and it cannot see a
    // step before the step happens -- the authority has to be earned down by an error that a step produces. So the claim
    // is on the settled value, with the peak reported next to it rather than asserted into silence.
    // 限值能给什么、不能给什么：它把 **持续** 的抽取压在上限上，而它看不见尚未发生的阶跃——权限只能被阶跃产生的误差压下来。
    // 所以声称的是稳定值，峰值报在旁边而不是被断言掉。
    EXPECT_LE(limited.settled_a, static_cast<double>(ceiling_a) * 1.02)
        << "a continuous draw must settle at the ceiling it was given";
    EXPECT_GT(limited.settled_a, static_cast<double>(ceiling_a) * 0.5) << "and it must not have throttled to nothing";
}

// 5. Authority comes back at the configured rate, not in one step -- that is what keeps the boundary calm.
TEST(BusCurrentLimit, TheAuthorityComesBackAtTheConfiguredRate) {
    BusCurrentLimit limiter;
    BusCurrentLimitConfig config = limit_config(1.0F);
    config.headroom_a = 0.0F;
    config.gain_per_a_s = 400.0F;
    config.fall_per_s = 1000.0F;
    config.rise_per_s = 1.0F;  // one full unit of authority per second
    ASSERT_EQ(foc::ErrorCode::Ok, limiter.configure(config));

    // Two times the ceiling for a while, which drives the authority down hard.
    // 两倍上限持续一会儿，把权限压下去。
    for (int k = 0; k < 100; ++k) {
        static_cast<void>(limiter.step(2.0F, kDt));
    }
    const float squeezed = limiter.output().scale;
    ASSERT_LT(squeezed, 0.6F) << "the fall has to have happened fast, as configured";

    // Now the bus current disappears below the aim; the authority may only rise at 1.0 per second.
    // 现在母线电流掉到瞄准值以下；权限每秒只能升 1.0。
    int steps_to_recover = 0;
    while ((limiter.output().scale < 1.0F) && (steps_to_recover < 100000)) {
        static_cast<void>(limiter.step(0.1F, kDt));
        ++steps_to_recover;
    }
    // 1.0 unit of authority at 1.0 per second from `squeezed` is `(1 - squeezed)` seconds, i.e. that many dt steps.
    // 每秒 1.0 的速率下，从 `squeezed` 回到满需要 `(1 - squeezed)` 秒，也就是那么多 dt 步。
    const int expected = static_cast<int>(std::lround((1.0F - squeezed) / (config.rise_per_s * kDt)));
    std::printf("[bus limit] recovery from %.3f took %d steps, the configured rate says %d\n",
                static_cast<double>(squeezed), steps_to_recover, expected);
    EXPECT_LE(steps_to_recover, expected + 2) << "the rise is rate-limited by the number it was configured with";
    EXPECT_GE(steps_to_recover, expected - 2) << "and it is not slower than that either";
}

// 6. With the chopper in the same bench the energy books still close: the limiter changes the command, not the physics.
TEST(BusCurrentLimit, TheEnergyBooksStillCloseWithTheLimiterInThePath) {
    // A capacitor holds the bus, a source and a load nearly cancel, and the chopper has a resistor to work with.
    // 一个电容撑着母线，源与负载几乎抵消，而斩波器有一个电阻可用。
    foc::sil::BusMeter meter(foc::sil::BusParams{1e-2F, kVBus, 0.35F, 0.35F, /*resistor*/ 6.0F});
    Bench bench(kVBus);
    bench.hal.set_bus(&meter);
    foc::core::BrakeChopper chopper;
    {
        foc::core::BrakeChopperConfig chopper_config{};
        chopper_config.v_on = kVBus + 1.0F;
        chopper_config.v_off = kVBus + 0.5F;
        chopper_config.r_ohm = 6.0F;
        chopper_config.duty_max = 0.9F;
        chopper_config.min_on_s = 0.0F;
        chopper_config.min_off_s = 0.0F;
        chopper_config.i2t_limit_a2s = 100.0F;
        chopper_config.i2t_cool_tau_s = 1.0F;
        chopper.set_config(chopper_config);
        ASSERT_EQ(foc::ErrorCode::Ok, chopper.validate());
    }

    BusCurrentLimit limiter;
    ASSERT_EQ(foc::ErrorCode::Ok, limiter.configure(limit_config(1.0F)));
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.start());

    constexpr int kTicks = 800;
    for (int k = 0; k < kTicks; ++k) {
        const auto& out = limiter.step(bench.bus_current_a(), kDt);
        bench.motor.set_torque(2.0F * out.scale);
        ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.tick(kDt));
        bench.hal.advance(kDt, kSubsteps);
        static_cast<void>(chopper.step(bench.hal.dc_link.voltage_v(), kDt));
    }

    const foc::sil::BusReport report = meter.report();
    std::printf("[bus limit] with chopper: drawn %.6f J, residual %.3e J of %.6f J drawn\n", report.drawn_j,
                report.residual_j, report.drawn_j);
    ASSERT_GT(report.drawn_j, 0.0);
    const double relative = std::fabs(report.residual_j) / report.drawn_j;
    EXPECT_LT(relative, 1e-3)
        << "scaling a command cannot create or destroy energy: the meter's books have to close as they did without it";
}

// 7. Two configurations the guard accepts or refuses *at* the edge, where the harness found nothing looking: `headroom_a
// == 0` (aim == ceiling) is legal and must stay legal, and a zero gain is a limiter that never moves, which is refused.
// 守卫在**边上**接受或拒绝的两种配置，而门禁发现没人看那里：`headroom_a == 0`（瞄准值等于上限）是合法的，必须继续
// 合法；而零增益是一个永远不会动的限制器，要被拒绝。
TEST(BusCurrentLimit, TheEdgeOfTheConfigurationGuardIsEachNumbersOwnPromise) {
    {
        BusCurrentLimit limiter;
        BusCurrentLimitConfig config = limit_config(1.0F);
        config.headroom_a = 0.0F; // aim == i_max: hold the ceiling, not a point below it
        EXPECT_EQ(foc::ErrorCode::Ok, limiter.configure(config))
            << "a band with no headroom is still a band; refusing it would take away the strictest legal setting";
        // And it does something: at the ceiling the authority is pulled down, not left at 1.
        // 而且它确实起作用：在**上限上**权限就被往下拉，而不是停在 1。
        for (int k = 0; k < 20; ++k) {
            static_cast<void>(limiter.step(1.5F, kDt));
        }
        EXPECT_LT(limiter.output().scale, 1.0F);
    }
    {
        // Each of the three rates on its own: a limiter that cannot move (zero gain), cannot come back, or cannot give
        // way is not a limiter, and the guard says so per number rather than per group -- the harness found `rise` was
        // the only one of the three a case ever zeroed.
        // 三个速率各自来一遍：一个不能动的（零增益）、回不来的、或让不了的，都不是限制器；守卫是逐个数表态的，而不是
        // 整组——门禁发现三个里只有一个（`rise`）曾被某个用例置零。
        for (const int which : {0, 1, 2}) {
            BusCurrentLimit limiter;
            BusCurrentLimitConfig config = limit_config(1.0F);
            if (which == 0) {
                config.fall_per_s = 0.0F;
            } else if (which == 1) {
                config.rise_per_s = 0.0F;
            } else {
                config.gain_per_a_s = 0.0F;
            }
            EXPECT_EQ(foc::ErrorCode::Param, limiter.configure(config)) << "rate #" << which << " of three";
        }
    }
}

// 8. A period of zero changes nothing -- including the flag that says whether anything was measured. The harness found
// the guard's `||` flippable to `&&`, which let a zero-length step run the body: the scale came out the same (the
// arithmetic of a zero step is a no-op) but `measured` was overwritten, and that flag is the whole difference between
// "the bus is fine" and "nobody told me".
// 周期为零的步什么都不改——**包括**那个说明是否测到了数的标志。门禁发现守卫里的 `||` 可以翻成 `&&`，从而让长度为
// 零的一步跑进函数体：`scale` 出来是一样的（零步的算术是空操作），但 `measured` 被覆盖了，而那个标志正是"母线很好"
// 与"没人告诉我"之间的全部区别。
TEST(BusCurrentLimit, AZeroPeriodDoesNotEvenRewriteTheMeasuredFlag) {
    BusCurrentLimit limiter;
    ASSERT_EQ(foc::ErrorCode::Ok, limiter.configure(limit_config(1.0F)));
    const auto& squeezed = limiter.step(2.0F, kDt);
    ASSERT_TRUE(squeezed.measured);
    const float scale = squeezed.scale;
    const bool limiting = squeezed.limiting;
    ASSERT_LT(scale, 1.0F);

    for (const float dt : {0.0F, -kDt}) {
        const auto& held = limiter.step(std::nanf(""), dt);
        EXPECT_TRUE(held.measured) << "a refused step reports last step's reading, and last step was told";
        EXPECT_EQ(scale, held.scale);
        EXPECT_EQ(limiting, held.limiting);
    }
}

// 9. One step follows the law exactly: the number the authority moves by is `gain * (error) * dt`, computed here from
// the same three configured values. The harness could turn either `*` into a `/` and stay green -- every other case in
// this file measures a *property* of the loop (it holds the ceiling, it comes back at a rate) and a property survives a
// law that is off by orders of magnitude in the middle.
// 一步严格照那条律走：权限移动的量是 `gain *（误差）* dt`，这里是拿那三个配置值自己算出来的。门禁把两个 `*` 中的任何
// 一个翻成 `/` 都能全绿——本文件其他用例量的都是环的**性质**（压得住、按速率回来），而性质是能在律差了好几个数量级时
// 依然成立的。
TEST(BusCurrentLimit, OneStepMovesByTheLawTheThreeNumbersDescribe) {
    BusCurrentLimit limiter;
    const BusCurrentLimitConfig config = limit_config(1.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, limiter.configure(config));
    ASSERT_FLOAT_EQ(1.0F, limiter.output().scale);

    // Inside the rate limit on both sides: an error of `i_max/20` moves the authority by `1 - gain*error*dt`, which is
    // `1 - 1e-3` here -- well inside what `fall_per_s * dt = 0.01` allows, so the law is what is under test and not the
    // floor.
    // 两侧都在限斜率之内：`i_max/20` 的误差让权限移动 `1 - gain*误差*dt`，这里是 `1 - 1e-3`——远在
    // `fall_per_s * dt = 0.01` 允许的范围内，所以被测量的是那条律，而不是底。
    const float aim_a = config.i_max_a - config.headroom_a;
    const float measured_a = aim_a + (config.i_max_a / 20.0F);
    const auto& out = limiter.step(measured_a, kDt);
    const float expected = 1.0F - (config.gain_per_a_s * (measured_a - aim_a) * kDt);
    EXPECT_NEAR(expected, out.scale, 1.0e-6F)
        << "one step from full authority has to move by exactly gain * error * dt, not merely 'downwards'";
    EXPECT_TRUE(out.limiting) << "and it is below full authority, so it has to say so";
}

// 10. The authority never rises above one, however far below the aim the current falls: the saturation is the promise,
// and the only thing that would break it is `wanted` running away upwards. The harness showed the two numbers in the
// saturation could be moved without anything noticing, because every case reaches the ceiling from below and stops.
// 无论电流掉到瞄准值以下多少，权限都不超过一：饱和就是这个承诺，而唯一能破坏它的是 `wanted` 往上跑。门禁显示那两处
// 常数可以被改动而无人发现，因为本文件每个用例都是从下面够到上限然后停住。
TEST(BusCurrentLimit, TheAuthorityNeverRisesAboveOneHoweverFarBelowTheAim) {
    BusCurrentLimit limiter;
    BusCurrentLimitConfig config = limit_config(1.0F);
    config.rise_per_s = 1.0e6F; // the rate limit must not be what stops it, so that the saturation is
    ASSERT_EQ(foc::ErrorCode::Ok, limiter.configure(config));

    // Zero current against an aim of `i_max - headroom`: the integrator wants `1 + gain*aim*dt = 1.019` here.
    // 零电流对上 `i_max - headroom` 的瞄准值：积分器想要 `1 + gain*瞄准值*dt = 1.019`。
    const auto& out = limiter.step(0.0F, kDt);
    EXPECT_EQ(1.0F, out.scale) << "full authority is the ceiling of the *state*, not just of the ramp";
    EXPECT_FALSE(out.limiting);
    EXPECT_FLOAT_EQ(1.0F, limiter.output().scale);
}

// 11. The fall has a rate too, and it is the configured one: one step from full authority against an absurd current falls
// by exactly `fall_per_s * dt`, not all the way to the answer. The harness could turn that `*` into a `/` and stay green
// because case 5 only asserts that the fall *happened* fast -- "fast" is satisfied by "instant".
// 下降也有速率，而且就是配置的那个：从满权限对上一个荒谬的电流走一步，只下降 `fall_per_s * dt`，不是一路掉到答案上。
// 门禁把那个 `*` 翻成 `/` 也能全绿，因为用例 5 只断言下降**很快**——而"很快"是"瞬间"也能满足的。
TEST(BusCurrentLimit, TheFallIsRateLimitedByItsOwnNumber) {
    BusCurrentLimit limiter;
    const BusCurrentLimitConfig config = limit_config(1.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, limiter.configure(config));
    ASSERT_FLOAT_EQ(1.0F, limiter.output().scale);

    // 100 A against an aim of 0.95: the integrator asks for `1 - 1.98`, so only the floor can decide the answer.
    // 100 A 对上 0.95 的瞄准值：积分器要的是 `1 - 1.98`，于是只有底能决定答案。
    const auto& out = limiter.step(100.0F, kDt);
    const float expected = 1.0F - (config.fall_per_s * kDt);
    EXPECT_NEAR(expected, out.scale, 1.0e-6F)
        << "one step may only give up fall_per_s * dt of authority, however bad the current is";
}

// 12. Full authority means exactly one, on a fresh limiter and after `reset()` -- not 1.001, and not "whatever the last
// state was plus a bit". The harness found three constants in that neighbourhood movable without a red case: the
// member initializer, the value `reset()` writes, and the scale it publishes.
// 满权限就是恰好一，无论是新对象还是 `reset()` 之后——不是 1.001，也不是"上一个状态再加一点"。门禁在这一带找到三个可以
// 随便动的常数：成员初始化值、`reset()` 写入的值、以及它公布出去的那个 scale。
TEST(BusCurrentLimit, FullAuthorityIsExactlyOneFreshAndAfterReset) {
    const BusCurrentLimitConfig config = limit_config(1.0F);
    const float aim_a = config.i_max_a - config.headroom_a;
    const float measured_a = aim_a + (config.i_max_a / 20.0F);
    const float expected = 1.0F - (config.gain_per_a_s * (measured_a - aim_a) * kDt);

    BusCurrentLimit limiter;
    ASSERT_EQ(foc::ErrorCode::Ok, limiter.configure(config));
    EXPECT_NEAR(expected, limiter.step(measured_a, kDt).scale, 1.0e-6F)
        << "a fresh limiter starts the law from exactly 1.0";

    // Squeeze it, then reset: both the state and the published scale go back to exactly one.
    // 先压下去，再 reset：状态与公布出去的 scale 都回到恰好一。
    for (int k = 0; k < 20; ++k) {
        static_cast<void>(limiter.step(2.0F, kDt));
    }
    ASSERT_LT(limiter.output().scale, 0.99F);
    limiter.reset();
    EXPECT_EQ(1.0F, limiter.output().scale) << "reset publishes exactly full authority";
    EXPECT_NEAR(expected, limiter.step(measured_a, kDt).scale, 1.0e-6F)
        << "and the law after a reset starts from that same one";
}

} // namespace
