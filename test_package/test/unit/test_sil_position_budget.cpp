// The positioning error budget: every known contributor gets a bound, and the sum has to bracket the measurement.
// 定位误差预算：每个已知误差源都有一个界，而它们的合计必须把实测值夹住。
//
// Why a budget and not one more assertion. A single "final error < 5 mrad" case says the loop is good today; it says
// nothing about which physical effect that 5 mrad is made of, so it cannot tell anyone whether a *different* encoder
// or a stiffer coupling would still make it. The budget is the difference between "measured OK" and "knows why".
//
// Criterion, written before the sweep:
//   * each contributor's single-source effect is measured, and where a provable bound exists the measurement must sit
//     under it -- the encoder's contribution can never exceed half a count; that is arithmetic, not a tuning choice;
//   * the **sum** of the single-source effects must bracket the combined measurement:
//         measured_combined <= budget <= 1.5 * measured_combined
//     The lower half is the part that matters. A missing term makes the measurement exceed the budget, and the upper
//     half stops the budget from being padded until it is unfalsifiable;
//   * the falsifier: dropping the dominant term has to break the lower half. If it does not, that term was not
//     load-bearing and the table is describing something else.
//
// Not in the table, and said out loud rather than omitted: **torsional elasticity**. It needs a two-mass model, the
// plant is single-inertia, and an effect with no injection point cannot be bounded here -- recording it as a number
// would be inventing one. Same for gear-ratio error, which is a frame transformation this rig does not model.
// 表里没有的两项，明说而不是略去： **扭转弹性** 需要两质量模型，而 plant 是单惯量，没有注入口的效应在这里给不出界——
// 编一个数就是造假。齿轮比误差同理，它是本台架不建模的坐标变换。
//
// 为什么是预算而不是再加一条断言。单独一条"末端误差 < 5 mrad"只能说明环路今天不错，说不出那 5 mrad 由哪些物理效应
// 构成，于是也无法回答"换一个编码器、或换一根更硬的联轴器，还能不能用"。预算是"量到了"与"知道为什么"之间的差别。

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/pi.hpp>
#include <foc/core/position.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/motor/motor.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"
#include "sil_plant_hal.hpp"

namespace {

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 4;
constexpr float kVBus = 24.0F;
constexpr float kTargetRad = 1.0F;
constexpr float kRunSeconds = 1.2F;
constexpr float kPositionBandwidth = 20.0F;
constexpr float kVelocityLimit = 50.0F;

/**
 * @brief [en] The contributors this rig can inject, one at a time.
 * @brief [zh] 本台架能逐个注入的误差源。
 */
struct Effects final {
    float resolution_rad{0.0F}; ///< encoder LSB, rad; 0 = ideal
    float t_static_nm{0.0F};    ///< breakaway torque, N*m; 0 = none
    float v_dead{0.0F};         ///< per-phase dead-time voltage, V; 0 = none
};

/**
 * @brief [en] One step, measured where it matters: at the rotor, not at the reading.
 * @brief [zh] 一次阶跃，量在要紧的地方：转子上，而不是读数上。
 */
struct Outcome final {
    float true_error_rad{0.0F};   ///< |plant angle - target|: where the load actually is
    float reading_error_rad{0.0F}; ///< |sensor angle - target|: what the loop believes
};

auto run_step(const Effects& fx, float target_rad) -> Outcome {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                     foc::core::PositionP>;
    const foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, 7U, 3.0F, 20000.0F, 0.0F};
    // `t_static_nm` is part of the plant's construction, not a setter: the friction model is what the rig is made of,
    // and a settable breakaway would let a test change the machine halfway through a run.
    // `t_static_nm` 属于 plant 的构造参数而不是 setter：摩擦模型是台架的组成部分，可随时改的起动转矩会让测试在运行中途
    // 换掉机器。
    foc::sil::PmsmPlant plant(
        foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, 7U, fx.t_static_nm, fx.t_static_nm, 0.05F});
    foc::sil::PlantHal hal(plant, kVBus);
    hal.position.set_resolution_rad(fx.resolution_rad);
    hal.set_deadtime_voltage(fx.v_dead);

    const typename MotorT::Policies policies{
        foc::core::BackEmfObserver({mc.rs, mc.ld}),
        foc::core::Svpwm{},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
        foc::core::NullReferenceGenerator{},
        foc::core::NullFeedforward{},
        foc::core::PositionP{foc::position_p_config(mc, kPositionBandwidth, kVelocityLimit, 0.0F)}};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_position(target_rad);
    static_cast<void>(motor.start());

    const int steps = static_cast<int>(kRunSeconds / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
    }
    return Outcome{std::fabs(plant.theta_mech_rad() - target_rad),
                   std::fabs(hal.position.angle_mech_rad() - target_rad)};
}

/**
 * @brief [en] The worst case over a sweep of targets inside one count.
 * @brief [zh] 在一个计数内扫过若干目标位后的最坏值。
 * @note [en] This is not thoroughness for its own sake. A quantisation error is *signed* and its sign depends on where
 *            the target falls between two counts: the first run of this table measured -0.26 mrad for a 2 mrad encoder,
 *            i.e. it made the result *better* by cancelling the loop's own residual. A budget cannot be built from one
 *            lucky target, so the target is swept across a count and the worst case is taken.
 * @note [zh] 不是为了周全而周全。量化误差是 **带符号** 的，符号取决于目标落在两个计数之间的哪里：本表第一次运行时， 2
 *            mrad 的编码器量出 -0.26 mrad——也就是把结果变得 **更好** ，抵掉了环路自身残差。预算不能建立在一个碰巧的目标
 *            上，所以在一个计数内扫过目标并取最坏。
 */
auto worst_over_targets(const Effects& fx, float lsb, int points) -> Outcome {
    Outcome worst{};
    for (int k = 0; k < points; ++k) {
        const float offset = (lsb > 0.0F) ? (lsb * static_cast<float>(k) / static_cast<float>(points)) : 0.0F;
        const Outcome one = run_step(fx, kTargetRad + offset);
        worst.true_error_rad = std::fmax(worst.true_error_rad, one.true_error_rad);
        worst.reading_error_rad = std::fmax(worst.reading_error_rad, one.reading_error_rad);
    }
    return worst;
}

} // namespace

TEST(SilPositionBudget, EveryContributorHasABoundAndTheirSumBracketsTheMeasurement) {
    constexpr float kLsb = 2.0e-3F;      // 2 mrad per count: a coarse but realistic incremental encoder on this axis
    constexpr float kBreakawayNm = 2.0e-4F; // a small but not negligible dry friction
    constexpr float kDeadTimeV = 0.48F;     // 1 us at 20 kHz on a 24 V bus

    constexpr int kPoints = 8;
    const Outcome clean = worst_over_targets(Effects{}, kLsb, kPoints);
    Effects res_fx{};
    res_fx.resolution_rad = kLsb;
    const Outcome resolution = worst_over_targets(res_fx, kLsb, kPoints);
    Effects fric_fx{};
    fric_fx.t_static_nm = kBreakawayNm;
    const Outcome friction = worst_over_targets(fric_fx, kLsb, kPoints);
    Effects dead_fx{};
    dead_fx.v_dead = kDeadTimeV;
    const Outcome deadtime = worst_over_targets(dead_fx, kLsb, kPoints);
    Effects all_fx{};
    all_fx.resolution_rad = kLsb;
    all_fx.t_static_nm = kBreakawayNm;
    all_fx.v_dead = kDeadTimeV;
    const Outcome combined = worst_over_targets(all_fx, kLsb, kPoints);

    // Each row is the worst *increase* over the clean run, floored at zero: a source cannot make the budget smaller, or
    // it would be a source that improves accuracy.
    // 每一行是相对干净基线的 **最坏增量** ，并以零为下限：误差源不能把预算变小，否则它就是一个能提高精度的误差源。
    const float e_res = std::fmax(0.0F, resolution.true_error_rad - clean.true_error_rad);
    const float e_fric = std::fmax(0.0F, friction.true_error_rad - clean.true_error_rad);
    const float e_dead = std::fmax(0.0F, deadtime.true_error_rad - clean.true_error_rad);
    // The budget keeps the baseline residual as its own row: it is a real error of the loop, and leaving it out is
    // exactly the "missing term" this table exists to catch.
    // 预算把基线残差保留为独立一行：它是环路的真实误差，漏掉它正是这张表要抓的"漏项"。
    const float budget = clean.true_error_rad + e_res + e_fric + e_dead;
    const float measured = combined.true_error_rad;

    std::printf("\n[budget] positioning error, target %0.2f rad swept over one %0.0f mrad count, kp %0.0f rad/s\n",
                static_cast<double>(kTargetRad), static_cast<double>(kLsb * 1000.0F),
                static_cast<double>(kPositionBandwidth));
    std::printf("%-26s %12s %12s\n", "source", "worst err", "delta");
    std::printf("%-26s %12.6f %12s\n", "loop residual (baseline)", static_cast<double>(clean.true_error_rad), "-");
    std::printf("%-26s %12.6f %12.6f\n", "encoder 2 mrad LSB", static_cast<double>(resolution.true_error_rad),
                static_cast<double>(e_res));
    std::printf("%-26s %12.6f %12.6f\n", "breakaway 0.2 mN*m", static_cast<double>(friction.true_error_rad),
                static_cast<double>(e_fric));
    std::printf("%-26s %12.6f %12.6f\n", "dead time 0.48 V", static_cast<double>(deadtime.true_error_rad),
                static_cast<double>(e_dead));
    std::printf("%-26s %12.6f   budget %0.6f\n", "all three", static_cast<double>(measured),
                static_cast<double>(budget));

    // 1. The provable bound: no encoder can contribute more than half a count.
    // 可实现上界：任何编码器的贡献都不会超过半格。
    EXPECT_LE(e_res, (kLsb * 0.5F) + 1.0e-5F) << "half an LSB is arithmetic, not a tuning choice";
    // 2. Steady state is where the integrator has had its say: a constant voltage error cannot survive it.
    // 稳态上积分器已经生效：恒定的电压误差不可能留下来。
    EXPECT_LT(e_dead, e_res) << "a constant voltage offset is integrated away; a resolution limit is not";
    // 3. The bracket. The lower half catches a missing term; the upper half keeps the budget falsifiable.
    // 夹逼。下半抓漏项，上半防止预算被注水到无法证伪。
    EXPECT_LE(measured, budget) << "a contributor is missing: the measurement exceeds the budget";
    EXPECT_LE(budget, 1.5F * measured) << "the budget is padded: it over-predicts by more than 50 %";
    // 4. The falsifier: the dominant row has to be load-bearing.
    // 反证：主导项必须是承重的。
    const float without_dominant =
        budget - std::fmax(e_res, std::fmax(e_fric, e_dead));
    EXPECT_GT(measured, without_dominant) << "dropping the dominant term must break the lower half of the bracket";
}
