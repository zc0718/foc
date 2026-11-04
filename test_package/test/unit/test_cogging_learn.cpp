// F47 - learning a cogging table from the current a speed loop already supplies. The criteria all compare the learned table to
// the one that was **injected**, because "it learned something" is satisfied by learning the wrong thing, and the whole point of a
// self-learning piece is that nobody knows the answer in advance.
// F47——从速度环本来就在供给的电流里学出一张齿槽表。所有判据都把学到的表与**注入的**表相比，因为"它学到了东西"这个条件，学到错的
// 东西也满足，而自学习这件的全部意义就是事先没人知道答案。

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/cogging_learn.hpp>

#include "sil_bench.hpp"
#include "sil_cogging_table.hpp"

namespace {

using foc::ErrorCode;
using foc::core::CoggingCompHarmonic;
using foc::core::CoggingLearnConfig;
using foc::core::CoggingLearner;
using foc::core::CoggingLearnReport;
using foc::sil::Bench;

constexpr float kTwoPi = 6.28318530718F;
constexpr float kKt = 0.05F; // nominal machine: `kt` 标称机的 `kt`


/** @brief Fourier amplitude of `series` at `order`, sampled at the mechanical angles in `theta`. */
auto harmonic_amplitude(const std::vector<float>& theta, const std::vector<float>& series, int order) -> float {
    float re = 0.0F;
    float im = 0.0F;
    for (std::size_t n = 0; n < theta.size(); ++n) {
        re = static_cast<float>(re + (series[n] * std::cos(static_cast<float>(order) * theta[n])));
        im = static_cast<float>(im + (series[n] * std::sin(static_cast<float>(order) * theta[n])));
    }
    return (2.0F / static_cast<float>(theta.size())) * std::hypot(re, im);
}

auto learn_config(std::uint8_t order) -> CoggingLearnConfig {
    CoggingLearnConfig config{};
    config.params = foc::core::MachineParams{0.5F, 4.0e-4F, 4e-4F, kKt / (1.5F * 7.0F)};
    config.pole_pairs = 7U;
    config.orders[0] = order;
    config.count = 1;
    config.revolutions = 2;
    config.min_speed_rad_s = 0.5F;
    config.max_speed_rad_s = 10.0F;
    config.speed_tolerance = 0.10F;
    return config;
}

/** @brief Run the bench with a cogging table injected and nothing compensating, and learn from the current the loop supplies. */
auto learn_from_bench(float amplitude_nm, float phase_rad, std::uint8_t order, float speed_rad_s)
    -> std::pair<CoggingLearnReport, CoggingCompHarmonic> {
    Bench bench{foc::sil::standard_bench_config()};
    foc::sil::CoggingHarmonic table{order, amplitude_nm, phase_rad};
    bench.plant().set_cogging(&table, 1);
    bench.motor().set_speed(speed_rad_s);
    EXPECT_EQ(ErrorCode::Ok, bench.init_and_start());
    CoggingLearner learner{learn_config(order)};
    bench.run(0.8F); // let the loops settle before learning starts 先让环路稳定再开始学
    // The record itself: every control period, the current the loop supplied and the angle it supplied it at.
    // 记录本身：每个控制周期，环路供给的电流与它供给时的角度。
    CoggingCompHarmonic learned{};
    CoggingLearnReport report{};
    bool finished = false;
    bench.run(6.0F, [&](Bench& b) {
        if (finished) {
            return;
        }
        const foc::sil::Sample sample = b.sample();
        learner.accumulate(b.plant().theta_mech_rad(), sample.iq_a, sample.id_a, b.plant().omega_mech_rad_s(), false);
        if (learner.ready()) {
            report = learner.finish(&learned, 1);
            finished = (report.status == ErrorCode::Ok);
        }
    });
    return {report, learned};
}

} // namespace

// The headline, and the only form of it that means anything: the amplitude of the learned table is the amplitude that was
// injected, and its phase is the phase that was injected. The phase is where a wrong sign convention shows up, so it is asserted
// at three injected phases rather than one -- a sign error is invisible at `phi = 0` and at `phi = pi/2` it looks like an
// amplitude swap.
// 要点，且是它唯一有意义的形式：学到的表幅值等于注入的幅值，相位等于注入的相位。符号约定错就错在相位上，所以在**三个**注入相位上断言
// 而不是一个——符号错误在 `φ = 0` 处看不见，而在 `φ = π/2` 处看起来像幅值被调换了。
TEST(CoggingLearn, TheLearnedTableIsTheInjectedOne) {
    constexpr float kAmplitude = 4.0e-3F;
    for (const float injected_phase : {0.0F, 0.9F, 2.4F}) {
        const auto [report, learned] = learn_from_bench(kAmplitude, injected_phase, 6U, 5.0F);
        ASSERT_EQ(ErrorCode::Ok, report.status) << "phase " << injected_phase;
        ASSERT_EQ(1, report.count);
        EXPECT_GT(report.revolutions, 2.0F);
        EXPECT_EQ(6, static_cast<int>(learned.order));
        // 5 rad/s is where the loop's own lag is −7.8° (measured), so the phase bound is that, not zero.
        // 5 rad/s 处环路自身滞后是 −7.8°（实测），所以相位界是那个，不是零。
        EXPECT_NEAR(kAmplitude, learned.amplitude_nm, kAmplitude * 0.10F) << "phase " << injected_phase;
        float phase_error = learned.phase_rad - injected_phase;
        while (phase_error > 3.14159265F) {
            phase_error = static_cast<float>(phase_error - kTwoPi);
        }
        while (phase_error < -3.14159265F) {
            phase_error = static_cast<float>(phase_error + kTwoPi);
        }
        EXPECT_LT(std::fabs(phase_error), 0.20F) << "phase " << injected_phase << " error " << phase_error;
    }
}

// The counterfactual, without which the case above is satisfiable by a learner that always answers with the same table: inject a
// different order and the learned table must follow it, both in which order it names and in how big it is.
// 反事实，没有它上面那条可以被"永远回答同一张表"的学习器满足：换一个注入次数，学到的表必须跟着变——无论在它报的是哪一次，还是在多大。
TEST(CoggingLearn, ADifferentInjectionGivesADifferentTable) {
    const auto [report_two, learned_two] = learn_from_bench(3.0e-3F, 0.4F, 2U, 5.0F);
    const auto [report_six, learned_six] = learn_from_bench(1.0e-3F, -1.1F, 6U, 5.0F);
    ASSERT_EQ(ErrorCode::Ok, report_two.status);
    ASSERT_EQ(ErrorCode::Ok, report_six.status);
    EXPECT_EQ(2, static_cast<int>(learned_two.order));
    EXPECT_EQ(6, static_cast<int>(learned_six.order));
    // And the amplitudes differ by the ratio they were injected at, so the learner is not reporting a machine constant.
    // 而且幅值之比等于注入之比，所以学习器报的不是某个机器常数。
    EXPECT_NEAR(3.0F, learned_two.amplitude_nm / learned_six.amplitude_nm, 0.35F);
}

// Nothing is invented: an order that was asked for but never injected comes back at zero, and an order that was injected but not
// asked for does not appear at all. A compensator acting on an extrapolated harmonic would inject a ripple of its own.
// 什么都不编造：被要求但没被注入的次数回来是零，被注入但没被要求的次数根本不出现。作用在外插出来的谐波上的补偿器会注入自己的纹波。
TEST(CoggingLearn, OrdersThatWereNotInjectedComeBackEmptyAndNothingIsInvented) {
    Bench bench{foc::sil::standard_bench_config()};
    foc::sil::CoggingHarmonic table{6U, 4.0e-3F, 0.0F};
    bench.plant().set_cogging(&table, 1);
    bench.motor().set_speed(5.0F);
    ASSERT_EQ(ErrorCode::Ok, bench.init_and_start());

    CoggingLearnConfig config = learn_config(2U);
    config.orders[0] = 2U;
    config.orders[1] = 6U; // asked for both, injected only one 两个都要，只注入了一个
    config.count = 2;
    CoggingLearner learner{config};
    bench.run(0.8F);
    CoggingCompHarmonic learned[2]{};
    CoggingLearnReport report{};
    // A zero-initialised report already reads `Ok`, so the "have we finished" state has to be its own flag -- guarding on the
    // status silently skipped every sample and the case then compares an empty report against the injected table.
    // 零初始化的 report 读出来就是 `Ok`，所以"是否已完成"必须是独立的标志——用状态做闸会静默跳过每一个样本，于是用例拿一张空报告
    // 去和注入的表比。
    bool finished = false;
    bench.run(6.0F, [&](Bench& b) {
        if (finished) {
            return;
        }
        const foc::sil::Sample sample = b.sample();
        learner.accumulate(b.plant().theta_mech_rad(), sample.iq_a, sample.id_a, b.plant().omega_mech_rad_s(), false);
        if (learner.ready()) {
            report = learner.finish(learned, 2);
            finished = (report.status == ErrorCode::Ok);
        }
    });
    ASSERT_EQ(ErrorCode::Ok, report.status);
    ASSERT_EQ(2, report.count);
    // The two entries are the two asked for, in the order asked, and the one that was never injected is empty.
    // 两条就是要求的两条、按要求的顺序，而没被注入的那条是空的。
    EXPECT_EQ(2, static_cast<int>(learned[0].order));
    EXPECT_EQ(6, static_cast<int>(learned[1].order));
    EXPECT_LT(learned[0].amplitude_nm, 4.0e-3F * 0.10F) << "order 2 came back at " << learned[0].amplitude_nm;
    EXPECT_NEAR(4.0e-3F, learned[1].amplitude_nm, 4.0e-3F * 0.12F);

    // And nothing outside the asked-for set is written: `finish` reports exactly `count`, so a caller indexing the array by
    // "everything it learned" cannot read past it.
    // 而要求之外什么都没写：`finish` 报出的正好是 `count`，于是按"它学到的一切"索引数组的调用方读不到界外。
    CoggingLearnReport dry = learner.finish(nullptr, 0);
    EXPECT_EQ(ErrorCode::State, dry.status);
}

// The four refusals, each with its own reason, all reachable: that is what makes them refusals rather than wishes.
// 四条拒绝，各有理由、都够得到——这才让它们成为拒绝而不是愿望。
TEST(CoggingLearn, RefusesRunsItCannotLearnFrom) {
    const CoggingLearnConfig config = learn_config(6U);
    CoggingCompHarmonic out[1]{};

    // 1. Nothing accumulated.
    // 1. 什么都没攒。
    CoggingLearner empty{config};
    EXPECT_EQ(ErrorCode::State, empty.finish(out, 1).status);
    EXPECT_FALSE(empty.ready());

    // 2. Too few revolutions: a quarter turn cannot separate a harmonic from the DC it rides on.
    // 2. 转数不够：四分之一转分不开一个谐波与它骑着的直流。
    CoggingLearner partial{config};
    for (int n = 0; n < 1000; ++n) {
        partial.accumulate(static_cast<float>(n) * 0.0015F, 0.05F, 0.0F, 5.0F, false);
    }
    const CoggingLearnReport few = partial.finish(out, 1);
    EXPECT_EQ(ErrorCode::State, few.status);
    EXPECT_LT(few.revolutions, 2.0F);

    // 3. Speed outside the band -- below it the loop's share is not measurable, above it the lag is baked in.
    // 3. 转速出带——低于下界那份不可测，高于上界滞后会被烘进去。
    CoggingLearner slow{config};
    for (int n = 0; n < 4000; ++n) {
        slow.accumulate(static_cast<float>(n) * 0.004F, 0.05F, 0.0F, 0.2F, false);
    }
    EXPECT_EQ(ErrorCode::State, slow.finish(out, 1).status);
    CoggingLearner fast{config};
    for (int n = 0; n < 4000; ++n) {
        fast.accumulate(static_cast<float>(n) * 0.02F, 0.05F, 0.0F, 40.0F, false);
    }
    EXPECT_EQ(ErrorCode::State, fast.finish(out, 1).status);

    // 4. Speed that moved: a projection assumes the angle advances uniformly.
    // 4. 转速在变：投影假设角度均匀推进。
    CoggingLearner unsteady{config};
    for (int n = 0; n < 4000; ++n) {
        const float speed = (n < 2000) ? 3.0F : 8.0F;
        unsteady.accumulate(static_cast<float>(n) * 0.004F, 0.05F, 0.0F, speed, false);
    }
    EXPECT_EQ(ErrorCode::State, unsteady.finish(out, 1).status);

    // 5. A saturated sample anywhere in the window: the loop was railing, not answering.
    // 5. 窗口内任何一次饱和：环路在撞限，不是在回答。
    CoggingLearner saturated{config};
    for (int n = 0; n < 4000; ++n) {
        saturated.accumulate(static_cast<float>(n) * 0.004F, 0.05F, 0.0F, 5.0F, n == 1234);
    }
    EXPECT_EQ(ErrorCode::State, saturated.finish(out, 1).status);

    // And a configuration that cannot work is `Param`, not `State`: it will never become ready no matter how long it runs.
    // 而一个不可能成立的配置是 `Param` 而不是 `State`：它跑多久都不会变好。
    CoggingLearnConfig broken = config;
    broken.revolutions = 0;
    EXPECT_EQ(ErrorCode::Param, CoggingLearner{broken}.finish(out, 1).status);
    broken = config;
    broken.orders[0] = 0U; // a constant torque is a load, not cogging 恒定转矩是负载，不是齿槽
    EXPECT_EQ(ErrorCode::Param, CoggingLearner{broken}.validate());
    broken = config;
    broken.max_speed_rad_s = 0.1F; // an inverted band 倒置的带
    EXPECT_EQ(ErrorCode::Param, CoggingLearner{broken}.validate());
}

// --- the chain: a table learned at one speed, applied through the existing slot at another -------------------------------
//
// The speed loop is the wrong place to *see* the payoff: at the ripple frequency a slow rotation produces, the loop answers the
// cogging itself, so `iq` stays at `T_cog/Kt` whether the compensator helps or not -- it **replaces** the loop's share rather than
// adding to it (measured in `test_sil_cogging_loop.cpp`). The motion is where it shows, and the composition that can measure it is
// a position ramp, which this file instantiates for itself the same way that one does.
// --- 链条：在一个转速学出的表，经**已有的那个槽**用在另一个转速 -------------------------
//
// 速度环是**看**回报的错误位置：慢转速产生的纹波频率上环路自己就回答了齿槽，所以无论补偿器帮没帮上忙，`iq` 都停在 `T_cog/Kt`——它是
// **替换**环路那份而不是加上去（`test_sil_cogging_loop.cpp` 量过）。纹波要在运动里看，而能量到它的组合是位置斜坡，本文件照那一份的做法
// 自己实例化一份。

using PositionMotor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                        foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                        foc::core::PositionP, foc::core::NullAlignment, foc::core::NullTrace>;

namespace {

struct RampRun final {
    float speed_order_rad_s;
    float revolutions;
};

/**
 * @brief Drive a position ramp at `rate_rad_s` with the compensator's output going into the iq feedforward slot.
 * @param inject_nm Amplitude of the cogging put into the plant, 0 for none.
 * @param apply Table the compensator runs from, nullptr for no compensation.
 */
// @brief 用补偿器的输出进 iq 前馈槽驱动一条 `rate_rad_s` 的位置斜坡。
// @param inject_nm 注入 plant 的齿槽幅值，0 表示不注入。
// @param apply 补偿器使用的表，nullptr 表示不补偿。
auto run_ramp(float inject_nm, float rate_rad_s, const CoggingCompHarmonic* apply) -> RampRun {
    const foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
    foc::sil::PmsmPlant plant{foc::sil::nominal_plant_params()};
    foc::sil::PlantHal hal{plant, cfg.v_bus_v};
    hal.position.set_sensor_present(true);
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
    foc::sil::CoggingHarmonic injected{6U, inject_nm, 0.3F};
    if (inject_nm > 0.0F) {
        plant.set_cogging(&injected, 1);
    }
    if (apply != nullptr) {
        comp.set_table(apply, 1); // learned, not injected 是用学到的，不是注入的
    }
    EXPECT_EQ(ErrorCode::Ok, motor.init());
    EXPECT_EQ(ErrorCode::Ok, motor.start());

    float target = 0.0F;
    auto step = [&](float ramp_target) {
        float feed_forward = 0.0F;
        if (apply != nullptr) {
            const foc::core::CoggingCompInput in{plant.theta_mech_rad(), plant.omega_mech_rad_s(), plant.current_dq().d,
                                                 0.0F, 3.0F};
            feed_forward = comp.offset(in).iq_a;
        }
        motor.set_command(foc::motor::Command{ramp_target, 0.0F, feed_forward, 40.0F, 2000.0F, 0.0F, 0.0F, 0.0F});
        static_cast<void>(motor.tick(cfg.dt_ctrl_s));
        hal.advance(cfg.dt_ctrl_s, cfg.substeps);
    };

    for (int n = 0; n < 10000; ++n) { // settle 先建立
        target += rate_rad_s * cfg.dt_ctrl_s;
        step(target);
    }
    std::vector<float> theta;
    std::vector<float> speed;
    for (int n = 0; n < 40000; ++n) {
        target += rate_rad_s * cfg.dt_ctrl_s;
        step(target);
        theta.push_back(plant.theta_mech_rad());
        speed.push_back(plant.omega_mech_rad_s());
    }
    return RampRun{harmonic_amplitude(theta, speed, 6), (theta.back() - theta.front()) / kTwoPi};
}

} // namespace

// The chain closes, with the **learned** table rather than the injected one, and at a speed it was never learned at: the table is
// learned from the current a 5 rad/s ramp makes the loop supply, then applied to a 10 rad/s ramp. The residual is not zero by
// construction -- the learned phase carries the loop's own lag from the learning speed, which leaves `2*sin(d/2)` of the cogging --
// so the criterion is that the motion lands inside the clean run's band rather than exactly on it.
// 链条闭合，用的是**学到的**表而不是注入的，而且用在它从未学习过的转速上：表从 5 rad/s 斜坡下环路供给的电流里学出来，再用于 10 rad/s
// 的斜坡。残差在构造上就不是零——学到的相位带着学习转速下环路自己的滞后，于是剩下 `2·sin(d/2)` 的齿槽——所以判据是运动落进干净那一跑的
// 带内，而不是精确落在它上面。
TEST(CoggingLearn, TheLearnedTableCancelsTheInjectionItWasLearnedFrom) {
    constexpr float kAmplitude = 4.0e-3F;
    constexpr float kApplyRate = 10.0F;

    // 1. Learn, at 5 rad/s, from the current a **speed** loop makes itself supply -- the configuration the learner is for.
    // 1. 学：在 5 rad/s 处，从**速度**环自己供给的电流里学——学习器就是为这个构型写的。
    const auto [report, learned] = learn_from_bench(kAmplitude, 0.3F, 6U, 5.0F);
    ASSERT_EQ(ErrorCode::Ok, report.status);
    ASSERT_GT(learned.amplitude_nm, 0.0F);
    EXPECT_EQ(6, static_cast<int>(learned.order));

    // 2. Three runs on the composition that can *see* the payoff -- a position ramp, which asks the machine for ~37 rad/s, i.e. a
    // speed the table was never learned at. The learning run could not have used this composition: its speed is far outside the
    // band the learner accepts, and demanding the band is the refusal that keeps a table from being learned where the loop is not
    // faithful.
    // 2. 在**能看见**回报的组合上跑三次——位置斜坡，向机器要约 37 rad/s，也就是表从未学习过的转速。学习那一跑不可能用这个组合：它的转速
    // 远在学习器接受的带外，而要求这个带正是那条拒绝，它挡住"在环路不可信的地方学表"。
    const RampRun clean = run_ramp(0.0F, kApplyRate, nullptr);
    const RampRun injected = run_ramp(kAmplitude, kApplyRate, nullptr);
    const RampRun compensated = run_ramp(kAmplitude, kApplyRate, &learned);

    // Ten revolutions, or the projection leaks DC as `1/k` and none of the numbers below mean anything.
    // 十转，否则投影按 `1/k` 泄漏直流，下面所有数字都没有意义。
    EXPECT_GT(clean.revolutions, 10.0F);
    EXPECT_GT(injected.revolutions, 10.0F);
    EXPECT_GT(compensated.revolutions, 10.0F);

    // Recorded so the numbers behind the bounds below are in the CI artifact rather than in a comment.
    // 记录下来，好让下面那些界背后的数字落在 CI 产物里而不是藏在一行注释里。
    RecordProperty("clean_speed_order_rad_s", clean.speed_order_rad_s);
    RecordProperty("injected_speed_order_rad_s", injected.speed_order_rad_s);
    RecordProperty("compensated_speed_order_rad_s", compensated.speed_order_rad_s);
    RecordProperty("learned_amplitude_nm", learned.amplitude_nm);
    RecordProperty("learned_phase_rad", learned.phase_rad);

    // The injection is visible in the motion at all -- the precondition for the rest.
    // 注入在运动里确实看得见——这是其余部分的前提。
    EXPECT_GT(injected.speed_order_rad_s, clean.speed_order_rad_s * 1.15F)
        << "clean " << clean.speed_order_rad_s << " injected " << injected.speed_order_rad_s;

    // And the learned table takes it back inside the clean run's band, while the uncompensated run was more than 15 % above it. The
    // residual is not zero by construction: the learned phase carries the loop's own lag from the learning speed, which leaves
    // `2*sin(d/2)` of the cogging -- 13.6 % for the 7.8 deg measured at 5 rad/s.
    // 而学到的表把它拿回到干净那一跑的带内，而未补偿那一跑比它高 15 % 以上。残差在构造上就不是零：学到的相位带着学习转速下环路自己的
    // 滞后，于是剩下 `2·sin(d/2)` 的齿槽——5 rad/s 处实测 7.8°，即 13.6 %。
    EXPECT_NEAR(clean.speed_order_rad_s, compensated.speed_order_rad_s, clean.speed_order_rad_s * 0.06F)
        << "clean " << clean.speed_order_rad_s << " compensated " << compensated.speed_order_rad_s
        << " (learned amplitude " << learned.amplitude_nm << " Nm, phase " << learned.phase_rad << " rad)";
}
