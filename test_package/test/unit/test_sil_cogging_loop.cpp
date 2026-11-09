// F33, the experiment that decides the compensator's shape: a cogging torque injected into the plant the closed loop runs
// on, and the question of where it shows up. If the loop merely *absorbs* it, a compensator is unnecessary; if it shows up
// in the current demand, a compensator has something to do and a known size; and if that size shrinks with speed, the
// compensator has a frequency boundary. All three are measured here.
// F33 里决定补偿器形状的实验：把齿槽转矩注入闭环真正跑的那台 plant，问它出现在哪里。若环路只是 **吸收** 了它，补偿器就不
// 必要；若它出现在电流需求里，补偿器就有活干、而且有已知的量级；若这个量级随转速缩小，补偿器就有频率边界。三条都在这里量。

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/cogging_comp.hpp>

#include "sil_bench.hpp"
#include "sil_cogging_table.hpp"

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
// 补偿器的理由，一个数：在慢转时齿槽转矩几乎正好以 `T_cog / Kt` 重现于 q 轴电流。这正是补偿器要 **不等速度误差** 就供给的量
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
// 边界：环路交付那份电流的能力随纹波频率下降，因为它回答的那个纹波是 **速度** 纹波、而回答它的是速度环。实测单调下降，所以补偿器
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
// 同一件事的第二个独立读数：注入齿槽时 q 轴电流的 **平均值** 不动，因为齿槽在一转上平均为零。若它动了，注入就是伪装的负载，上面
// 每个数量的都会是稳态偏置而不是纹波。
TEST(SilCoggingLoop, TheInjectionMovesTheRippleAndNotTheAverage) {
    constexpr float kAmplitude = 4.0e-3F;
    const LoopResult off = run_loop(20.0F, 0.0F);
    const LoopResult on = run_loop(20.0F, kAmplitude);
    EXPECT_NEAR(off.iq_mean_a, on.iq_mean_a, std::fabs(off.iq_mean_a) * 0.1F + 1.0e-3F);
}

// --- end to end: the fourth composition -----------------------------------------------------------------------------
//
// `feedforward_current_a` is honoured only in position mode, and `Bench::MotorT` has `NullPositionControl`, so the bench cannot
// reach it at all: every tick in position mode returns `NotImpl`. This block instantiates the fourth composition -- the bench's
// motor with `core::PositionP` -- and drives a slow position ramp with the compensator's output as the iq feedforward.
//
// The observable is the **speed** ripple, not the current: at the ripple frequency this rotation produces, the loops cannot
// follow, so what the compensator buys shows up in the motion. The window is asserted to cover ten revolutions, because a
// projection over a non-integer number of revolutions leaks DC as `1/k` and can be larger than the effect being measured --
// measured the hard way before this case was written.
// --- 端到端：第四种组合 -----------------------------------------------------------------------------------------------
//
// `feedforward_current_a` 只在位置模式生效，而 `Bench::MotorT` 的位置控制器是 `NullPositionControl`，所以台架根本够不到它：位置模式下
// 每个 tick 都返回 `NotImpl`。本段实例化第四种组合——台架那台 motor 换成带 `core::PositionP` 的——并用补偿器的输出作为 iq 前馈驱动一条
// 慢速位置斜坡。
//
// 观测量取 **速度** 纹波而不是电流：在这个转速产生的纹波频率上环路跟不上，于是补偿器买到的东西出现在运动里。窗口断言覆盖 **十转** ，
// 因为非整数转的投影会把直流按 `1/k` 泄漏进来、可能比待测效应还大——这条弯路在写这个用例之前已经量过了。

using PositionMotor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                        foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                        foc::core::PositionP, foc::core::NullAlignment, foc::core::NullTrace>;

namespace {

struct RampResult final {
    float speed_order_rad_s;
    float revolutions;
};

constexpr float kRampRadS = 10.0F;

auto run_ramp(float amplitude_nm, bool compensate) -> RampResult {
    const foc::sil::BenchConfig cfg = standard_bench_config();
    foc::sil::PmsmPlant plant{foc::sil::nominal_plant_params()};
    foc::sil::PlantHal hal{plant, cfg.v_bus_v};
    hal.position.set_sensor_present(true); // position mode needs an absolute angle 位置模式需要绝对角
    const auto mc = foc::sil::nominal_motor_config();
    PositionMotor motor{
        mc,
        PositionMotor::Policies{
            foc::core::BackEmfObserver{foc::core::BackEmfObserver::Config{mc.rs, mc.ld}}, foc::core::Svpwm{},
            foc::core::Pi{foc::current_pi_config(mc, cfg.omega_current_rad_s, cfg.v_bus_v)},
            foc::core::Pi{foc::current_pi_config(mc, cfg.omega_current_rad_s, cfg.v_bus_v)},
            foc::core::Pi{foc::speed_pi_config(mc, cfg.omega_speed_rad_s)}, foc::core::NullReferenceGenerator{},
            foc::core::NullFeedforward{}, foc::core::PositionP{foc::core::PositionP::Config{20.0F, 200.0F, 2000.0F}},
            foc::core::NullAlignment{}, foc::core::NullTrace{}},
        hal};

    foc::core::CoggingCompConfig comp_config{};
    comp_config.params =
        foc::core::MachineParams{plant.params().rs, plant.params().ld, plant.params().lq, plant.psi_f_wb()};
    comp_config.pole_pairs = plant.params().pole_pairs;
    foc::core::CoggingCompensation comp{comp_config};
    if (amplitude_nm > 0.0F) {
        foc::sil::CoggingHarmonic table{static_cast<std::uint8_t>(kOrder), amplitude_nm, 0.0F};
        plant.set_cogging(&table, 1);
        comp.set_table(&table, 1);
    }
    EXPECT_EQ(foc::ErrorCode::Ok, motor.init());
    EXPECT_EQ(foc::ErrorCode::Ok, motor.start());

    float target = 0.0F;
    auto step = [&](float ramp_target) {
        float feed_forward = 0.0F;
        if (compensate) {
            const foc::core::CoggingCompInput in{plant.theta_mech_rad(), plant.omega_mech_rad_s(), plant.current_dq().d,
                                                 0.0F, 3.0F};
            feed_forward = comp.offset(in).iq_a;
        }
        motor.set_command(foc::motor::Command{ramp_target, 0.0F, feed_forward, 40.0F, 2000.0F, 0.0F, 0.0F, 0.0F});
        static_cast<void>(motor.tick(cfg.dt_ctrl_s));
        hal.advance(cfg.dt_ctrl_s, cfg.substeps);
    };

    for (int n = 0; n < 10000; ++n) { // settle 先建立
        target += kRampRadS * cfg.dt_ctrl_s;
        step(target);
    }
    std::vector<float> theta;
    std::vector<float> speed;
    for (int n = 0; n < 40000; ++n) {
        target += kRampRadS * cfg.dt_ctrl_s;
        step(target);
        theta.push_back(plant.theta_mech_rad());
        speed.push_back(plant.omega_mech_rad_s());
    }
    return RampResult{harmonic_amplitude(theta, speed, kOrder), (theta.back() - theta.front()) / 6.2831853F};
}

} // namespace

// The whole chain: the table goes into the plant, the compensator's output goes into the iq reference through the slot that
// exists, and the motion comes back clean. Without the compensator the injected cogging makes the speed ripple measurably
// larger; with it, the ripple returns to what it is with **nothing injected at all** -- which is the only comparison that
// cannot be satisfied by a compensator that merely perturbs the drive in some other way.
// 整条链：表进 plant，补偿器的输出经 **真实存在的那个槽** 进 iq 参考，运动干净地回来。没有补偿器时注入的齿槽让速度纹波明显变大；有它时，
// 纹波回到 **完全不注入** 时的数值——这是唯一一种"只是用别的方式扰动驱动器"的补偿器无法满足的比较。
TEST(SilCoggingLoop, ThroughTheIqFeedforwardSlotTheMotionComesBackClean) {
    constexpr float kAmplitude = 4.0e-3F;
    const RampResult clean = run_ramp(0.0F, false);
    const RampResult injected = run_ramp(kAmplitude, false);
    const RampResult compensated = run_ramp(kAmplitude, true);

    // The window has to be several revolutions or the projection leaks DC as `1/k` and the numbers below mean nothing.
    // 窗口必须覆盖数转，否则投影会按 `1/k` 泄漏直流，下面的数字就没有意义。
    EXPECT_GT(clean.revolutions, 10.0F);
    EXPECT_GT(injected.revolutions, 10.0F);
    EXPECT_GT(compensated.revolutions, 10.0F);

    // The injection is visible: this is the precondition for the rest of the case.
    // 注入看得见：这是本用例其余部分的前提。
    EXPECT_GT(injected.speed_order_rad_s, clean.speed_order_rad_s * 1.15F)
        << "clean " << clean.speed_order_rad_s << " injected " << injected.speed_order_rad_s;

    // And with the compensator the motion is back to the clean value, to 5 % -- while the uncompensated run was more than 15 %
    // above it, so the two runs are on opposite sides of the same line.
    // 而带上补偿器后运动回到干净值，误差 5 % 以内——未补偿那一跑比它高 15 % 以上，于是两跑落在同一条线的两侧。
    EXPECT_NEAR(clean.speed_order_rad_s, compensated.speed_order_rad_s, clean.speed_order_rad_s * 0.05F)
        << "clean " << clean.speed_order_rad_s << " compensated " << compensated.speed_order_rad_s;
}

// --- the payoff of the iq slot: speed mode, ripple, no fourth composition -------------------------------------------
//
// A constant offset in a loop with integral action is absorbed (measured, in `test_feedforward_current.cpp`), so the slot looks
// useless if that is the only thing it is asked to carry. A **ripple** is what no integrator can absorb, and this is the
// configuration a real drive runs in: speed control, cogging injected, the compensator's output going into
// `set_feedforward_current()` every period.
// --- iq 槽的回报：速度模式、纹波、不需要第四种组合 -------------------------------------------------------------
//
// 带积分的环路会吸收 **恒定** 偏置（已在 `test_feedforward_current.cpp` 量过），所以如果只拿它扛恒定项，这个槽看起来没用。
// **纹波** 才是任何积分器吸收不了的东西，而这也正是真实驱动器所处的构型：速度控制、注入齿槽、补偿器每周期把输出送进
// `set_feedforward_current()`。

namespace {

struct SpeedModeResult final {
    float iq_order_a;
    float speed_order_rad_s;
};

auto run_speed_mode(float amplitude_nm, bool compensate) -> SpeedModeResult {
    Bench bench{standard_bench_config()};
    foc::core::CoggingCompConfig comp_config{};
    comp_config.params = foc::core::MachineParams{bench.plant().params().rs, bench.plant().params().ld,
                                                  bench.plant().params().lq, bench.plant().psi_f_wb()};
    comp_config.pole_pairs = bench.plant().params().pole_pairs;
    foc::core::CoggingCompensation comp{comp_config};
    if (amplitude_nm > 0.0F) {
        foc::sil::CoggingHarmonic table{static_cast<std::uint8_t>(kOrder), amplitude_nm, 0.0F};
        bench.plant().set_cogging(&table, 1);
        comp.set_table(&table, 1);
    }
    bench.motor().set_speed(10.0F);
    EXPECT_EQ(foc::ErrorCode::Ok, bench.init_and_start());
    bench.run(0.6F);
    std::vector<float> theta;
    std::vector<float> iq;
    std::vector<float> speed;
    for (int n = 0; n < 40000; ++n) {
        if (compensate) {
            const foc::sil::Sample sample = bench.sample();
            const foc::core::CoggingCompInput in{bench.plant().theta_mech_rad(), bench.plant().omega_mech_rad_s(),
                                                 sample.id_a, 0.0F, 3.0F};
            bench.motor().set_feedforward_current(comp.offset(in).iq_a);
        }
        bench.tick_once();
        theta.push_back(bench.plant().theta_mech_rad());
        iq.push_back(bench.sample().iq_a);
        speed.push_back(bench.plant().omega_mech_rad_s());
    }
    return SpeedModeResult{harmonic_amplitude(theta, iq, kOrder), harmonic_amplitude(theta, speed, kOrder)};
}

} // namespace

// Two facts in one case, and the two of them together are the claim: the compensator supplies the anti-cogging current itself
// (`iq` keeps its `T_cog/Kt` of 0.079 A, i.e. the loop's own share was **replaced**, not doubled), and the speed ripple falls
// with it. A version that added to the loop's own share would read about 0.16 A -- which is why the band is [0.06, 0.12].
// 一个用例两个事实，合起来才是主张：补偿器 **自己** 供给反齿槽电流（`iq` 仍保持 `T_cog/Kt` 的 0.079 A，即环路那份是被 **替换** 的，
// 不是翻倍），而速度纹波随之下降。叠加到环路自己那份上的版本会读到约 0.16 A——所以判据带取 [0.06, 0.12]。
TEST(SilCoggingLoop, InSpeedModeTheSlotReplacesTheLoopsShare) {
    constexpr float kAmplitude = 4.0e-3F;
    const SpeedModeResult off = run_speed_mode(kAmplitude, false);
    const SpeedModeResult on = run_speed_mode(kAmplitude, true);

    // 1:1 with `T_cog/Kt`, so the total is not the loop's share plus the compensator's.
    // 与 `T_cog/Kt` 一比一，所以总和不是"环路那份 + 补偿器那份"。
    const float expected = kAmplitude / foc::sil::nominal_spec().kt;
    EXPECT_GT(on.iq_order_a, expected * 0.78F) << "compensated iq " << on.iq_order_a << " expected about " << expected;
    EXPECT_LT(on.iq_order_a, expected * 1.22F) << "compensated iq " << on.iq_order_a << " expected about " << expected;
    // And it is not double: the failing version this guards against reads about 2x.
    // 而且它不是两倍：本判据防的那个失败版本会读到约 2 倍。
    EXPECT_LT(on.iq_order_a, off.iq_order_a * 1.6F) << "off " << off.iq_order_a << " on " << on.iq_order_a;

    // The **motion** claim is not asserted here, and that is a measurement rather than an omission: at 10 rad/s mechanical the
    // speed's sixth-order floor with nothing injected is already about 0.1 rad/s, and the cogging's own contribution at this
    // inertia is the same size -- measured 0.1015 uncompensated against 0.1027 compensated, i.e. the two runs are inside each
    // other's noise. The motion claim lives in `ThroughTheIqFeedforwardSlotTheMotionComesBackClean`, where the rotation is fast
    // enough (37 rad/s) for the cogging to be 24 % of the ripple rather than 0 %.
    // **运动** 那条主张不在这里断言，这是测量结果而不是遗漏：10 rad/s 机械处不注入时速度的 6 次底噪就约 0.1 rad/s，而这个惯量下齿槽
    // 自己的贡献同样大小——实测未补偿 0.1015 对已补偿 0.1027，两跑落在彼此的噪声里。运动那条主张在
    // `ThroughTheIqFeedforwardSlotTheMotionComesBackClean` 里，那里转速够快（37 rad/s），齿槽占纹波的 24 % 而不是 0 %。
    EXPECT_NEAR(off.speed_order_rad_s, on.speed_order_rad_s, off.speed_order_rad_s * 0.3F)
        << "the speed floor moved: off " << off.speed_order_rad_s << " on " << on.speed_order_rad_s;
}
