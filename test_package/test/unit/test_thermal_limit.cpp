// F36's second half: turning a thermal description into a current a drive can promise, and into a time budget. Every
// criterion here is measured against the **plant**, not against the same formula written twice -- a module that inverts its
// own algebra perfectly and disagrees with the model it describes would pass a self-consistent test and fail in the drive.
// F36 的后半：把一份热描述换成一个驱动器可以承诺的电流，以及一份时间预算。这里每条判据都对着 **plant** 量，而不是把一个公式写两遍
// ——一个完美地反解了自己那点代数、却和它所描述的那个模型不一致的模块，能通过自我一致的用例，然后在驱动器里失败。

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <foc/core/thermal_limit.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

using foc::ErrorCode;
using foc::core::ThermalBudget;
using foc::core::ThermalLimit;
using foc::core::ThermalLimitConfig;
using foc::core::ThermalLimitReport;

constexpr float kCopperAlpha = 3.93e-3F;
constexpr float kInfinity = std::numeric_limits<float>::infinity();

auto limit_config(float rt_k_per_w, float ct_j_per_k, float t_allowed_c) -> ThermalLimitConfig {
    const auto nameplate = foc::sil::nominal_plant_params();
    ThermalLimitConfig config{};
    // The real machine, so the numbers below are the ones every other SIL case uses; only `rs` is read by this module.
    // 用真机参数，好让下面的数字与其它 SIL 用例是同一套；本模块只读 `rs`。
    config.params = foc::core::MachineParams{nameplate.rs, nameplate.ld, nameplate.lq,
                                             nameplate.kt / (1.5F * nameplate.pole_pairs)};
    config.alpha_per_k = kCopperAlpha;
    config.rt_k_per_w = rt_k_per_w;
    config.ct_j_per_k = ct_j_per_k;
    config.t_ambient_c = 20.0F;
    config.t_allowed_c = t_allowed_c;
    return config;
}

/** @brief What the naive law gives: the feedback factor dropped. The number the module has to be *different* from. */
// 天真的律给出的数：丢掉反馈因子。模块必须与它 **不同** 的那个数。
auto naive_continuous_a(float rs20, float rt, float t_ambient_c, float t_allowed_c) -> float {
    return std::sqrt((t_allowed_c - t_ambient_c) / (1.5F * rt * rs20));
}

/**
 * @brief Hold a current on the plant with the harness playing the current loop, and report where it settles.
 * @param current_a the current to hold, A.
 * @param plant_rt the plant's own `Rt`, K/W.
 * @param plant_ct the plant's own `Ct`, J/K.
 * @param seconds how long to run.
 */
// 用夹具扮演电流环在 plant 上保持一个电流，并报出它收敛到哪里。
auto settle_on_plant(float current_a, float plant_rt, float plant_ct, float seconds) -> float {
    foc::sil::PmsmPlant plant{foc::sil::nominal_plant_params()};
    foc::sil::PmsmPlant::ThermalConfig thermal{};
    thermal.rt_k_per_w = plant_rt;
    thermal.ct_j_per_k = plant_ct;
    thermal.t_ambient_c = 20.0F;
    plant.set_thermal(thermal);
    plant.set_winding_temperature(20.0F, kCopperAlpha);
    const float dt = 5.0e-5F;
    const int steps = static_cast<int>(seconds / dt);
    for (int n = 0; n < steps; ++n) {
        plant.set_voltage_dq(foc::core::Dq<float>{current_a * plant.effective_rs(), 0.0F});
        plant.advance(dt);
    }
    return plant.winding_temperature_c();
}

} // namespace

// The definition, measured from both sides: the current this module calls continuous settles the plant **exactly** at the
// temperature the reserve was sized against -- and one 5 % above it does not. A module that was 15 % optimistic would still
// pass "it settles near 105" and fail the second half.
// 定义，从两侧量：本模块称为"可持续"的电流让 plant **恰好** 收敛到定余量时用的那个温度上——而高 5 % 的那个不会。一个乐观 15 % 的模块
// 仍能通过"大体收敛在 105 附近"，但通不过后半条。
TEST(ThermalLimit, TheContinuousCurrentSettlesThePlantAtTheAllowedTemperatureAndFivePercentMoreDoesNot) {
    constexpr float kRt = 4.0F;
    const ThermalLimit limit{limit_config(kRt, 0.5F, 105.0F)};
    const ThermalLimitReport report = limit.continuous();
    ASSERT_EQ(ErrorCode::Ok, report.status);
    ASSERT_GT(report.continuous_a, 0.0F);
    EXPECT_FLOAT_EQ(105.0F, report.settle_at_cont_c);
    EXPECT_FLOAT_EQ(kRt * 0.5F, report.thermal_time_const_s);

    // The settle of that current, straight from the plant, after 40 thermal time constants.
    // 那个电流的稳态，直接取自 plant，跑过 40 个热时间常数。
    const float settled = settle_on_plant(report.continuous_a, kRt, 0.5F, 80.0F);
    EXPECT_NEAR(105.0F, settled, 2.0F) << "continuous " << report.continuous_a << " A settled at " << settled << " C";

    // And 5 % more is *not* sustainable: it settles above the reserve, which is what "continuous" has to mean.
    // 而多 5 % **不** 可持续：它收敛在余量之上，这才是"可持续"必须意味着的东西。
    const float hotter = settle_on_plant(report.continuous_a * 1.05F, kRt, 0.5F, 80.0F);
    EXPECT_GT(hotter, 105.0F + 2.0F) << "1.05x settled at " << hotter << " C";
}

// The feedback factor is the whole difference between this module and arithmetic anyone would write in a minute: the naive
// `sqrt((T-Ta)/(1.5*Rt*rs))` reads **15.5 %** high at 105 C on copper, and the plant proves which one is right by settling
// above the reserve when the naive current is held. The case asserts the ratio *and* the consequence.
// 反馈因子就是本模块与"任何人一分钟就能写出来的算术"之间的全部差别：天真的 `sqrt((T−Ta)/(1.5·Rt·rs))` 在铜的 105 ℃ 处读到
// **高 15.5 %** ，而 plant 用"保持天真电流时收敛在余量之上"证明了谁对。本用例既断言比值，也断言后果。
TEST(ThermalLimit, TheNaiveLawReadsFifteenPercentHighAndThePlantSaysSo) {
    constexpr float kRt = 4.0F;
    const auto params = foc::sil::nominal_plant_params();
    const ThermalLimit limit{limit_config(kRt, 0.5F, 105.0F)};
    const ThermalLimitReport report = limit.continuous();
    ASSERT_EQ(ErrorCode::Ok, report.status);
    const float naive = naive_continuous_a(params.rs, kRt, 20.0F, 105.0F);
    RecordProperty("continuous_a", report.continuous_a);
    RecordProperty("naive_a", naive);
    // `sqrt(1 + alpha*(105-20))` = 1.155, so the naive answer is that much too high. Stated as a ratio so it cannot drift.
    // `sqrt(1 + α·(105−20))` = 1.155，于是天真答案高出这么多。写成比值，免得漂移。
    EXPECT_NEAR(std::sqrt(1.0F + (kCopperAlpha * 85.0F)), naive / report.continuous_a, 0.01F)
        << "naive " << naive << " correct " << report.continuous_a;

    const float held = settle_on_plant(naive, kRt, 0.5F, 80.0F);
    EXPECT_GT(held, 105.0F + 3.0F) << "the naive current settled the plant at " << held << " C";
}

// The time budget, measured as a crossing: give the plant a current whose settle is above the reserve, run until it reaches
// the reserve, and compare with the module's closed form. Two currents and two starting temperatures, so the answer cannot
// be a constant that happens to be right once.
// 时间预算，以穿越来量：给 plant 一个稳态高于余量的电流，跑到它到达余量，再与模块的闭式解比。两个电流、两个起始温度，于是答案不可能是
// 一个碰巧对上一次的常数。
TEST(ThermalLimit, TheBudgetPredictsWhenThePlantActuallyCrosses) {
    constexpr float kRt = 4.0F;
    constexpr float kCt = 0.5F;
    const ThermalLimit limit{limit_config(kRt, kCt, 105.0F)};
    const ThermalLimitReport report = limit.continuous();
    ASSERT_EQ(ErrorCode::Ok, report.status);

    // A crossing, computed and measured, for two currents above the continuous one.
    // 两个高于可持续值的电流，各算一次也各量一次。
    for (const float factor : {1.2F, 1.5F}) {
        const float current = report.continuous_a * factor;
        const ThermalBudget budget = limit.budget(current, 20.0F);
        ASSERT_EQ(ErrorCode::Ok, budget.status);
        ASSERT_TRUE(budget.crosses) << "factor " << factor;
        ASSERT_GT(budget.seconds, 0.0F);
        EXPECT_GT(budget.settle_c, 105.0F) << "factor " << factor << " settle " << budget.settle_c;

        // Run the plant to exactly that instant and check where it got to.
        // 把 plant 正好跑到那一刻，看它到了哪里。
        const float at_prediction = settle_on_plant(current, kRt, kCt, budget.seconds);
        EXPECT_NEAR(105.0F, at_prediction, 3.0F) << "factor " << factor << " current " << current << " A, "
                                                 << budget.seconds << " s, plant at " << at_prediction << " C";
    }

    // The budget is not a constant: a hotter start crosses sooner, and the ratio is the exponential's, not a guess.
    // 预算不是一个常数：起点越热穿越越早，而比值是指数的，不是猜的。
    const float current = report.continuous_a * 1.2F;
    const float from_cold = limit.budget(current, 20.0F).seconds;
    const float from_warm = limit.budget(current, 60.0F).seconds;
    EXPECT_LT(from_warm, from_cold * 0.75F) << "cold " << from_cold << " warm " << from_warm;
}

// The other half of the answer, and the one that keeps a scheduler from throttling a machine that is fine: a current whose
// settle is at or below the reserve is reported as **never** crossing, with an infinite budget -- not as a large number.
// 答案的另一半，也是让调度不去压一台本来没问题的机器的那个：稳态在余量处或以下的电流被报成 **永不** 穿越、预算为无穷——而不是一个
// 大数字。
TEST(ThermalLimit, ASustainableCurrentIsReportedAsNeverCrossingRatherThanAsALongTime) {
    constexpr float kRt = 4.0F;
    const ThermalLimit limit{limit_config(kRt, 0.5F, 105.0F)};
    const float continuous = limit.continuous().continuous_a;

    const ThermalBudget at_limit = limit.budget(continuous, 20.0F);
    EXPECT_FALSE(at_limit.crosses) << "the continuous current is by definition sustainable";
    EXPECT_EQ(kInfinity, at_limit.seconds);

    const ThermalBudget below = limit.budget(continuous * 0.5F, 20.0F);
    EXPECT_FALSE(below.crosses);
    EXPECT_EQ(kInfinity, below.seconds);
    EXPECT_LT(below.settle_c, 105.0F);

    // And the degenerate boundaries report a decision rather than a negative or a NaN: already at the reserve is zero time,
    // and a current with no steady state at all diverges rather than settling.
    // 而两个退化的边界报的是决定，而不是负数或 NaN：已经在余量处就是零时间；完全没有稳态的电流是发散而不是收敛。
    EXPECT_FLOAT_EQ(0.0F, limit.budget(continuous * 1.5F, 105.0F).seconds);
    EXPECT_FLOAT_EQ(0.0F, limit.budget(continuous * 1.5F, 120.0F).seconds);
    // The run-away current: `Rt*k*alpha > 1` has no steady state, so the settle is infinite and the crossing is finite.
    // 失控电流：`Rt·k·α > 1` 没有稳态，于是稳态是无穷、穿越是有限的。
    const float runaway_a = std::sqrt(2.0F / (kRt * 1.5F * foc::sil::nominal_plant_params().rs * kCopperAlpha));
    EXPECT_EQ(kInfinity, limit.settle_temperature_c(runaway_a * 1.1F));
    const ThermalBudget diverging = limit.budget(runaway_a * 1.1F, 20.0F);
    ASSERT_EQ(ErrorCode::Ok, diverging.status);
    EXPECT_TRUE(diverging.crosses);
    EXPECT_GT(diverging.seconds, 0.0F) << "a diverging node still reaches the reserve in finite time";
    EXPECT_TRUE(std::isfinite(diverging.seconds));
    EXPECT_EQ(kInfinity, diverging.settle_c) << "no steady state means no settle, not a negative asymptote";
}

// Refusals: a configuration that cannot answer says so, and says it as `Param` rather than as a zero current a host might
// believe.
// 拒绝：无法回答的配置如实说，而且说的是 `Param`，不是一个宿主可能会相信的零电流。
TEST(ThermalLimit, RefusesConfigurationsThatCannotAnswer) {
    ThermalLimitConfig config = limit_config(4.0F, 0.5F, 105.0F);
    EXPECT_EQ(ErrorCode::Ok, ThermalLimit{config}.validate());

    config = limit_config(4.0F, 0.5F, 105.0F);
    config.rt_k_per_w = 0.0F; // no cooling path at all 完全没有散热路径
    EXPECT_EQ(ErrorCode::Param, ThermalLimit{config}.validate());
    EXPECT_EQ(ErrorCode::Param, ThermalLimit{config}.continuous().status);

    config = limit_config(4.0F, 0.5F, 105.0F);
    config.params.rs = 0.0F; // no copper to heat 没有可发热的铜
    EXPECT_EQ(ErrorCode::Param, ThermalLimit{config}.validate());

    config = limit_config(4.0F, 0.5F, 105.0F);
    config.alpha_per_k = -1.0F; // a winding that cools as it heats is not a model 越热越凉的绕组不是模型
    EXPECT_EQ(ErrorCode::Param, ThermalLimit{config}.validate());

    config = limit_config(4.0F, 0.5F, 105.0F);
    config.t_allowed_c = 20.0F; // equal to ambient: no current can reach it 等于环境温度：没有电流能到达
    EXPECT_EQ(ErrorCode::Param, ThermalLimit{config}.validate());
    config.t_allowed_c = 10.0F; // below ambient 低于环境温度
    EXPECT_EQ(ErrorCode::Param, ThermalLimit{config}.validate());

    // A missing `Ct` is *not* a configuration error: the continuous current does not need it, and only the time budget has
    // to say that it cannot name a time.
    // 缺 `Ct` **不是** 配置错误：持续电流不需要它，只有时间预算必须说明它报不出时间。
    config = limit_config(4.0F, 0.0F, 105.0F);
    EXPECT_EQ(ErrorCode::Ok, ThermalLimit{config}.validate());
    const ThermalLimit no_time{config};
    EXPECT_EQ(0.0F, no_time.continuous().thermal_time_const_s);
    const ThermalBudget crosses = no_time.budget(no_time.continuous().continuous_a * 2.0F, 20.0F);
    EXPECT_EQ(ErrorCode::Ok, crosses.status);
    EXPECT_TRUE(crosses.crosses);
    EXPECT_EQ(kInfinity, crosses.seconds) << "it crosses, but no `Ct` was given so no time can be named";
}

// The chain the two halves were built for: the temperature the plant will actually reach at the *continuous* current leaves
// the protector at full authority, while an overshoot current walks into the derating band. That is what "scheduling" buys
// over "derating": the drive stays out of the band instead of being cut down inside it.
// 这两半为之而建的链条：在 **可持续** 电流下 plant 实际会达到的温度让保护器保持全权限，而一个过冲电流会走进降额带。这就是"调度"
// 相对"降额"买到的东西：驱动器留在带外，而不是在带内被切。
TEST(ThermalLimit, TheContinuousCurrentKeepsTheProtectorAtFullAuthorityWhileAnOvershootWalksIntoTheBand) {
    using foc::core::ProtectionConfig;
    using foc::core::Protector;
    constexpr float kRt = 4.0F;
    const ThermalLimit limit{limit_config(kRt, 0.5F, 105.0F)};
    const float continuous = limit.continuous().continuous_a;

    ProtectionConfig limits{};
    limits.i_hard_a = 40.0F; // wide, so only the temperature decides 放宽，只让温度说话
    limits.i_max_a = 10.0F;
    limits.i2t_a2s = 4.5F;
    limits.i2t_cooling_s = 0.5F;
    limits.v_max_v = 48.0F;
    limits.v_min_v = 12.0F;
    limits.t_trip_c = 130.0F;
    limits.t_derate_start_c = 105.0F; // the reserve *is* the derating knee 余量就是降额拐点
    limits.t_derate_end_c = 130.0F;
    Protector protector{limits};

    const float settled_c = settle_on_plant(continuous, kRt, 0.5F, 80.0F);
    EXPECT_NEAR(105.0F, settled_c, 2.0F);
    EXPECT_FLOAT_EQ(limits.i_max_a, protector.derated_limit(settled_c))
        << "at its own reserve the winding is still at full authority, settled at " << settled_c;

    const float overshoot_c = settle_on_plant(continuous * 1.2F, kRt, 0.5F, 80.0F);
    EXPECT_LT(protector.derated_limit(overshoot_c), limits.i_max_a)
        << "1.2x settled at " << overshoot_c << " C and should be derated";
}

// The guards that keep a scheduler from acting on a number it did not get: a configuration that cannot answer and a sample
// that is not a number both come back as refusals or as NaN, never as a plausible current or a plausible time. A drive that
// read "0 s" from a NaN input would throttle to nothing and look like it was working.
// 挡在"调度拿着一个它没拿到的数去做事"之前的护栏：无法回答的配置与非数值样本，要么作为拒绝返回，要么作为 NaN 返回，绝不作为一个
// 看起来合理的电流或时间。一个从 NaN 输入读出"0 s"的驱动器会把功率压到零，而表面上一切正常。
TEST(ThermalLimit, AConfigurationThatCannotAnswerRefusesRatherThanGuessing) {
    const ThermalLimit broken{limit_config(0.0F, 0.5F, 105.0F)}; // no cooling path 没有散热路径
    EXPECT_EQ(ErrorCode::Param, broken.continuous().status);
    EXPECT_EQ(ErrorCode::Param, broken.budget(1.0F, 20.0F).status);
    EXPECT_TRUE(std::isnan(broken.settle_temperature_c(1.0F))) << "a bad configuration must not produce a temperature";

    ThermalLimitConfig config = limit_config(4.0F, 0.5F, 105.0F);
    config.t_allowed_c = NAN;
    EXPECT_EQ(ErrorCode::Param, ThermalLimit{config}.validate());
    config = limit_config(4.0F, 0.5F, 105.0F);
    config.t_ambient_c = NAN;
    EXPECT_EQ(ErrorCode::Param, ThermalLimit{config}.validate());
    config = limit_config(4.0F, 0.5F, 105.0F);
    config.params.rs = NAN;
    EXPECT_EQ(ErrorCode::Param, ThermalLimit{config}.validate());

    // A sample that is not a number is refused, and a current with no finite square is "crosses immediately" rather than a
    // silent NaN propagating into a duty cycle.
    // 非数值样本被拒绝；而平方非有限的电流是"立刻穿越"，而不是一个静默传播进占空比的 NaN。
    const ThermalLimit limit{limit_config(4.0F, 0.5F, 105.0F)};
    EXPECT_EQ(ErrorCode::Param, limit.budget(NAN, 20.0F).status);
    EXPECT_EQ(ErrorCode::Param, limit.budget(1.0F, NAN).status);
    const ThermalBudget huge = limit.budget(std::numeric_limits<float>::max(), 20.0F);
    EXPECT_EQ(ErrorCode::Ok, huge.status);
    EXPECT_TRUE(huge.crosses);
    EXPECT_LT(huge.seconds, 1.0e-3F) << "a diverging node reaches the reserve almost immediately: " << huge.seconds;
    EXPECT_EQ(kInfinity, huge.settle_c) << "and it has no settle to report, only an asymptote it runs away from";

    // And the accessors report back exactly what was configured, so a caller can log the assumption it acted on.
    // 而访问器原样报出配置进去的数，于是调用方可以把它的假设记进日志。
    EXPECT_FLOAT_EQ(4.0F, limit.thermal_resistance_k_per_w());
    EXPECT_FLOAT_EQ(2.0F, limit.continuous().thermal_time_const_s); // 4 K/W * 0.5 J/K 4 K/W × 0.5 J/K
}

// The one current at which the node stops being a first-order lag and becomes a pure integrator: `b = k*alpha - 1/Rt = 0`, the
// exact boundary between "settles" and "runs away". The answer stops being an exponential and becomes `t = dT*Ct/a`, which is
// the case that proves the module is solving the equation rather than fitting the usual shape -- a version that only knew the
// exponential would divide by zero here.
// 使节点不再是惯性环节、而变成纯积分器的那个电流：`b = k·α − 1/Rt = 0`，正是"收敛"与"失控"的边界。答案不再是指数，而是
// `t = ΔT·Ct/a`——这个用例证明模块是在解方程，而不是在套那个常见形状：只会指数的那一版会在这里除以零。
TEST(ThermalLimit, AtTheStabilityBoundaryTheNodeIsAPureIntegratorAndTheTimeIsLinear) {
    ThermalLimitConfig config{};
    const auto nameplate = foc::sil::nominal_plant_params();
    config.params = foc::core::MachineParams{nameplate.rs, nameplate.ld, nameplate.lq,
                                             nameplate.kt / (1.5F * nameplate.pole_pairs)};
    config.alpha_per_k = 1.0F; // chosen so `Rt*k*alpha` can be hit exactly 选得让 `Rt·k·α` 能被精确命中
    config.rt_k_per_w = 2.0F;
    config.ct_j_per_k = 0.5F;
    config.t_ambient_c = 20.0F;
    config.t_allowed_c = 105.0F;
    const ThermalLimit limit{config};

    // `k = 1.5*rs*i^2`, so `b = 0` at `i^2 = 1/(Rt*1.5*rs*alpha)`; with these numbers `k = 0.5` and the slope is
    // `a/Ct = (k*(1-20*alpha) + Ta/Rt)/Ct = (-9.5 + 10)/0.5 = 1 K/s`.
    // `k = 1.5·rs·i²`，所以 `b = 0` 在 `i² = 1/(Rt·1.5·rs·α)` 处；用这些数 `k = 0.5`，斜率是
    // `a/Ct = (k·(1−20α) + Ta/Rt)/Ct = (−9.5 + 10)/0.5 = 1 K/s`。
    const float boundary_a = std::sqrt(1.0F / (config.rt_k_per_w * 1.5F * nameplate.rs * config.alpha_per_k));
    EXPECT_EQ(kInfinity, limit.settle_temperature_c(boundary_a))
        << "a pure integrator even at the boundary: no steady state";

    const ThermalBudget at_boundary = limit.budget(boundary_a, 20.0F);
    ASSERT_EQ(ErrorCode::Ok, at_boundary.status);
    EXPECT_TRUE(at_boundary.crosses);
    EXPECT_EQ(kInfinity, at_boundary.settle_c);
    // 85 K of rise at 1 K/s.
    // 1 K/s 下升 85 K。
    EXPECT_NEAR(85.0F, at_boundary.seconds, 1.0F) << "measured " << at_boundary.seconds;
}
