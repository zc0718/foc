// The breakaway sequence: its own three criteria, then one against the machine that actually sticks.
// 起步序列：它自己的三条判据，再加一条对着真正会卡住的机器。

#include <cmath>
#include <cstddef>

#include <gtest/gtest.h>

#include <foc/core/stall_start.hpp>

#include "sil_bench.hpp" // nominal_plant_params()
#include "sil_pmsm_plant.hpp"


namespace {

constexpr float kDt = 1e-4F;

auto recipe(float breakaway_nm, float max_nm) -> foc::core::StallStartConfig {
    foc::core::StallStartConfig config{};
    config.breakaway_nm = breakaway_nm;
    config.max_torque_nm = max_nm;
    config.ramp_nm_per_s = 0.5F;   // 0.03 N*m of breakaway is reached in 60 ms
    config.release_speed_rad_s = 0.5F;
    config.give_up_s = 0.2F;
    return config;
}

} // namespace



TEST(StallStart, TheRampReleasesTheAxisTheMomentTheTorqueIsEnough) {
    // The torque at release is the handover value, so the speed loop starts from a known current. It must be the torque
    // that was *just enough*, not the limit: a sequence that reported its ceiling would hand over a wound-up command,
    // which is the failure this whole mechanism exists to avoid.
    // 松脱时的力矩就是交接值，因此速度环从已知电流起步。它必须是"刚刚够"的那个力矩、不是限幅：报出上限的序列交接的是
    // 一条饱和指令，而那正是本机制要避免的失败。
    foc::core::StallStart sequence(recipe(0.030F, 0.200F));
    ASSERT_EQ(foc::ErrorCode::Ok, sequence.validate());
    ASSERT_TRUE(sequence.begin());

    float speed = 0.0F;
    foc::core::StallStartOutput out{};
    int ticks = 0;
    while ((sequence.phase() == foc::core::StallStartPhase::Ramping) && (ticks < 2000)) {
        out = sequence.step(kDt, speed);
        // The plant is a stiction block: nothing moves until the torque beats the breakaway point, then it moves.
        // 被控对象是一个静摩擦块：力矩战胜起动点之前什么也不动，之后才动。
        speed = (out.torque_nm > 0.030F) ? 0.8F : 0.0F;
        ++ticks;
    }
    std::printf("\n[3.2-s] released at %.1f ms with %.4f N*m (breakaway 0.030)\n", static_cast<double>(ticks) * kDt * 1e3,
                static_cast<double>(out.torque_nm));

    EXPECT_EQ(foc::core::StallStartPhase::Released, out.phase) << "60 ms of ramp at 0.5 N*m/s reaches 0.030 N*m";
    ASSERT_EQ(foc::core::StallStartPhase::Released, sequence.phase());
    EXPECT_GE(out.torque_nm, 0.030F);
    EXPECT_LT(out.torque_nm, 0.031F) << "released just above the breakaway point, not at the limit";
    EXPECT_FALSE(out.limited);
    // And it holds: a released sequence keeps reporting the handover torque while the loop takes over.
    // 而且它保持：已松脱的序列在环路接手期间持续报告交接力矩。
    const foc::core::StallStartOutput after = sequence.step(kDt, 40.0F);
    EXPECT_EQ(foc::core::StallStartPhase::Released, after.phase);
    EXPECT_FLOAT_EQ(out.torque_nm, after.torque_nm);
}


TEST(StallStart, ALimitUnderTheBreakawayPointIsRefusedRatherThanRampedIntoAWall) {
    // A recipe whose ceiling is below the breakaway torque can only ever produce `Failed`. Accepting it would be
    // accepting a sequence that is known in advance not to work, and a caller would find out 200 ms later.
    // 上限低于起动力矩的配方只可能产出 `Failed`。接受它就等于接受一条 **事先已知不工作** 的序列，而调用方要 200 ms 后
    // 才知道。
    foc::core::StallStart impossible(recipe(0.030F, 0.020F));
    EXPECT_EQ(foc::ErrorCode::Param, impossible.validate());
    EXPECT_FALSE(impossible.begin());
    EXPECT_EQ(foc::core::StallStartPhase::Idle, impossible.phase());
    const auto out = impossible.step(kDt, 0.0F);
    EXPECT_EQ(foc::ErrorCode::Param, out.status) << "the refusal is reported on every step, not just at begin()";

    // A degenerate recipe is refused too, and each number has its own way of being wrong.
    // 退化配方同样被拒，而每个数都有自己的错法。
    auto zero_ramp = recipe(0.030F, 0.200F);
    zero_ramp.ramp_nm_per_s = 0.0F;
    EXPECT_EQ(foc::ErrorCode::Param, foc::core::StallStart(zero_ramp).validate());
    auto zero_release = recipe(0.030F, 0.200F);
    zero_release.release_speed_rad_s = 0.0F;
    EXPECT_EQ(foc::ErrorCode::Param, foc::core::StallStart(zero_release).validate());
    auto no_grace = recipe(0.030F, 0.200F);
    no_grace.give_up_s = 0.0F;
    EXPECT_EQ(foc::ErrorCode::Param, foc::core::StallStart(no_grace).validate());
}


TEST(StallStart, AStuckAxisSaysFailedInsteadOfRampingForever) {
    // The load is within the limit but the rotor never follows -- a jam. The sequence must stop saying "starting" and
    // say "this cannot be started", because a silent axis is indistinguishable from a working one from the outside.
    // 负载在限幅之内但转子始终不跟随——卡死。序列必须停止说"正在起动"而改说"起不来"，因为从外面看，一根沉默的轴与一根
    // 正常的轴无法区分。
    foc::core::StallStart sequence(recipe(0.030F, 0.200F));
    ASSERT_TRUE(sequence.begin());
    foc::core::StallStartOutput out{};
    int ticks = 0;
    while ((sequence.phase() == foc::core::StallStartPhase::Ramping) && (ticks < 20000)) {
        out = sequence.step(kDt, 0.0F); // the rotor never moves
        ++ticks;
    }
    std::printf("[3.2-s] jammed axis: %s after %.0f ms at %.3f N*m\n",
                (out.phase == foc::core::StallStartPhase::Failed) ? "Failed" : "still ramping",
                static_cast<double>(ticks) * kDt * 1e3, static_cast<double>(out.torque_nm));
    EXPECT_EQ(foc::core::StallStartPhase::Failed, out.phase);
    EXPECT_TRUE(out.limited);
    EXPECT_FLOAT_EQ(0.0F, out.torque_nm) << "a failed sequence commands nothing";
    // Ramp to 0.200 N*m at 0.5 N*m/s is 400 ms, then the 200 ms grace: it must not have concluded before that.
    // 以 0.5 N·m/s 爬到 0.200 N·m 要 400 ms，再加 200 ms 宽限：结论不得早于此。
    EXPECT_GT(static_cast<double>(ticks) * kDt, 0.55) << "it gave up before it had even reached its limit";
}


TEST(StallStart, TheSequenceBreaksARealStuckMachineAndSaysWhen) {
    // The criterion that ties the sequence to the physics: a plant with stiction and a load, driven by the sequence's
    // own torque. Two independent statements must agree -- the sequence says "released" on the period the rotor
    // crosses the release speed, and the plant says the rotor was at rest until the torque beat 0.020 + 0.010.
    // 把序列与物理绑在一起的判据：一台带静摩擦与负载的 plant，由序列自己的力矩驱动。两个独立陈述必须一致——序列在转子
    // 越过松脱转速的那一拍说"已松脱"，而 plant 说转子在力矩战胜 0.020 + 0.010 之前一直静止。
    auto params = foc::sil::nominal_plant_params();
    params.b = 1.0e-3F;
    params.t_static_nm = 0.020F;
    params.t_coulomb_nm = 0.010F;
    // The stiction band has to be far below the release threshold, or the two describe the same speed and the case
    // cannot tell "it started" from "it is still being held". At 0.5 rad/s against a 0.5 rad/s release the first run of
    // this case reported the same torque for "last at rest" and "first moving", which is exactly that collision.
    // 静摩擦带必须远低于松脱阈值，否则两者描述同一个速度，用例分不清"起步了"与"还被按着"。以 0.5 rad/s 的带对 0.5 rad/s
    // 的松脱阈值时，本用例第一版把"最后静止"与"首次运动"报成了同一个力矩，正是这个碰撞。
    params.omega_eps_rad_s = 0.01F;
    constexpr float kLoadNm = 0.010F;
    const float psi_f = params.kt / (1.5F * static_cast<float>(params.pole_pairs));

    foc::sil::PmsmPlant plant(params);
    plant.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
    plant.set_load_torque(kLoadNm);

    auto plant_recipe = recipe(params.t_static_nm + kLoadNm, 0.200F);
    // A low release threshold for this case, and the reason is arithmetic: the handover value is `breakaway` plus
    // whatever the ramp added while the rotor accelerated to the threshold. At 0.5 rad/s that is 5.4 ms of ramp and
    // 2.7 mN*m -- measured, and larger than the thing being checked. At 0.05 rad/s the rotor needs one period.
    // 本用例用很低的松脱阈值，理由是算术：交接值 = 起动点 + 转子加速到阈值期间斜坡又加上去的量。0.5 rad/s 时那是 5.4 ms
    // 的斜坡与 2.7 mN·m——实测值，比被检查的量还大。0.05 rad/s 时转子只需一个周期。
    plant_recipe.release_speed_rad_s = 0.05F;
    foc::core::StallStart sequence(plant_recipe);
    ASSERT_TRUE(sequence.begin());

    float last_at_rest_nm = 0.0F;      // the highest torque the plant still reported as "not moving"
    float first_moving_nm = 0.0F;      // the first torque at which it reported movement
    float torque_at_release_nm = 0.0F; // the sequence's own handover value
    int ticks = 0;
    int release_tick = -1;
    while ((sequence.phase() == foc::core::StallStartPhase::Ramping) && (ticks < 20000)) {
        const auto now = plant.state();
        const auto out = sequence.step(kDt, now.omega_mech);
        if ((out.phase == foc::core::StallStartPhase::Released) && (release_tick < 0)) {
            torque_at_release_nm = out.torque_nm;
            release_tick = ticks;
        }
        if (std::fabs(now.omega_mech) <= params.omega_eps_rad_s) {
            last_at_rest_nm = out.torque_nm;
        } else if (first_moving_nm == 0.0F) {
            first_moving_nm = out.torque_nm;
        }
        // A current source fed from the setpoint: the sequence asks for a torque, the rig delivers that torque.
        // 由给定值驱动的电流源：序列要一个力矩，台架就交付那个力矩。
        const float iq_a = out.torque_nm / params.kt;
        const float omega_e = now.omega_mech * static_cast<float>(params.pole_pairs);
        plant.set_voltage_dq(foc::core::Dq<float>{-(omega_e * params.lq * iq_a), (params.rs * iq_a) + (omega_e * psi_f)});
        plant.advance(kDt);
        ++ticks;
    }
    const float breakaway_nm = params.t_static_nm + kLoadNm;
    const float step_nm = 0.5F * kDt;
    std::printf("[3.2-s] plant: last at rest %.4f, first moving %.4f N*m; sequence released at %.4f N*m, %.1f ms\n",
                static_cast<double>(last_at_rest_nm), static_cast<double>(first_moving_nm),
                static_cast<double>(torque_at_release_nm), static_cast<double>(release_tick) * kDt * 1e3);

    EXPECT_EQ(foc::core::StallStartPhase::Released, sequence.phase()) << "the sequence must have released";
    // The breakaway point is *bracketed* by the two torques the plant reported, and the gap is one ramp step: the
    // rotor did not move at `last_at_rest` and did move at `first_moving`. Demanding that they straddle the configured
    // value is the physics claim; demanding that one of them equal it to the last digit would be demanding a threshold
    // crossing land exactly on a sample.
    // 起动点被 plant 报出的两个力矩 **夹住** ，且间隔一个斜坡步：在 `last_at_rest` 时没动，在 `first_moving` 时动了。要求
    // 两者夹住配置值是物理断言；要求其中一个逐位等于它，等于要求一次阈值穿越正好落在采样点上。
    // 1 mN*m of slack is the *observation* lag, measured at 0.5 mN*m: the plant's threshold is exact inside the model,
    // but `state()` is read before the step that crosses it, so the crossing is seen about ten periods late.
    // 1 mN·m 的余量是 **观测** 滞后，实测 0.5 mN·m：plant 内部阈值是精确的，但 `state()` 在读它的那一步之前被读取，于是
    // 穿越点晚约十个周期才被看见。
    EXPECT_LE(last_at_rest_nm, breakaway_nm + 0.001F) << "it was still at rest at the breakaway point";
    EXPECT_GT(first_moving_nm, breakaway_nm - 0.001F) << "and it moved just above it";

    // The two sources agree on *when* to within the time the rotor needs to reach the detection speed: the sequence
    // releases on the period the speed first crosses `release_speed_rad_s`, the plant's first observable motion is the
    // period before that. Measured 60.1 ms against 65.5 ms -- a 5.4 ms lag, which is the acceleration of a 0.023 N*m
    // surplus torque on this inertia, not a disagreement.
    // 两个来源对"何时"的一致度，等于转子达到检测转速所需的时间：序列在转速首次越过 `release_speed_rad_s` 的那一拍松脱，
    // plant 的首次可观测运动是它前一拍。实测 60.1 ms 对 65.5 ms——5.4 ms 的滞后，那是 0.023 N·m 富余力矩在这个惯量上
    // 的加速度，不是分歧。
    EXPECT_NEAR(breakaway_nm, torque_at_release_nm, 0.001F) << "the sequence hands over just above breakaway";
    EXPECT_GT(release_tick, 0);
    EXPECT_NEAR(0.060, static_cast<double>(release_tick) * kDt, 0.005) << "0.030 N*m at 0.5 N*m/s is 60 ms";
}

namespace {

/**
 * @brief [en] What one breakaway run measured, and which of the two stiction numbers moved to produce it.
 * @brief [zh] 一次挣脱运行量到了什么，以及是哪两个静摩擦数中的哪一个被挪动才得到的。
 */
struct BreakawayRun final {
    float release_nm{0.0F};       ///< the sequence's handover value, N*m
    float release_ms{0.0F};       ///< when it said Released
    float last_at_rest_nm{0.0F};  ///< the highest torque the plant still reported as "not moving"
    float first_moving_nm{0.0F};  ///< the first torque at which it reported movement
    bool released{false};
    bool failed{false};
};

/**
 * @brief [en] Drive a stiction plant with the sequence's own torque and report both clocks.
 *
 * @brief [zh] 用序列自己的力矩驱动一台带静摩擦的 plant，并报出两个时钟。
 *
 *   `true_breakaway_nm` is the plant's stiction plus the constant load; `recipe_breakaway_nm` is what the caller *told*
 *   the sequence. They are separate arguments because every sensitivity row below is about the difference between them.
 *
 *   `true_breakaway_nm` 是 plant 的静摩擦加恒定负载； `recipe_breakaway_nm` 是调用方 **告诉** 序列的那个数。两者是分开
 *   的参数，因为下面每一行敏感度关心的都是它们之差。
 */
auto run_breakaway(float recipe_breakaway_nm, float true_breakaway_nm, float ramp_nm_per_s,
                   int max_ticks = 40000) -> BreakawayRun {
    constexpr float kLoadNm = 0.010F;
    auto params = foc::sil::nominal_plant_params();
    params.b = 1.0e-3F;
    params.t_static_nm = true_breakaway_nm - kLoadNm;
    params.t_coulomb_nm = kLoadNm;
    params.omega_eps_rad_s = 0.01F;
    const float psi_f = params.kt / (1.5F * static_cast<float>(params.pole_pairs));

    foc::sil::PmsmPlant plant(params);
    plant.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
    plant.set_load_torque(kLoadNm);

    foc::core::StallStartConfig config{};
    config.breakaway_nm = recipe_breakaway_nm;
    config.max_torque_nm = 0.200F;
    config.ramp_nm_per_s = ramp_nm_per_s;
    // The same low detection speed the plant case above uses, and for the same arithmetic reason: at 0.5 rad/s the
    // handover value would carry 2.7 mN*m of ramp that the rotor spent accelerating to the threshold.
    // 与上面那个 plant 用例相同的低检测转速，理由是同一个算术：0.5 rad/s 时交接值会带上转子加速到阈值期间斜坡加的
    // 2.7 mN·m。
    config.release_speed_rad_s = 0.05F;
    config.give_up_s = 0.2F;
    foc::core::StallStart sequence(config);
    static_cast<void>(sequence.begin());

    BreakawayRun run;
    int ticks = 0;
    while ((sequence.phase() == foc::core::StallStartPhase::Ramping) && (ticks < max_ticks)) {
        const auto now = plant.state();
        const auto out = sequence.step(kDt, now.omega_mech);
        if ((out.phase == foc::core::StallStartPhase::Released) && !run.released) {
            run.released = true;
            run.release_nm = out.torque_nm;
            run.release_ms = static_cast<float>(ticks) * kDt * 1e3F;
        }
        if (out.phase == foc::core::StallStartPhase::Failed) {
            run.failed = true;
        }
        if (std::fabs(now.omega_mech) <= params.omega_eps_rad_s) {
            run.last_at_rest_nm = out.torque_nm;
        } else if (run.first_moving_nm == 0.0F) {
            run.first_moving_nm = out.torque_nm;
        }
        const float iq_a = out.torque_nm / params.kt;
        const float omega_e = now.omega_mech * static_cast<float>(params.pole_pairs);
        plant.set_voltage_dq(
            foc::core::Dq<float>{-(omega_e * params.lq * iq_a), (params.rs * iq_a) + (omega_e * psi_f)});
        plant.advance(kDt);
        ++ticks;
    }
    return run;
}

} // namespace

/**
 * @brief [en] A step before `begin()` says `State` and commands nothing -- it does not start the sequence by accident.
 *
 * @brief [zh] `begin()` 之前的一步答 `State` 且什么都不下发——它不会顺手把序列启动起来。
 *
 *   `step()` is called from a control loop, and a loop calls it every period whether or not the sequence was started,
 *   because the sequence's recipe is validated at configuration time and the loop has no reason to ask again. So the
 *   Idle branch is the one that decides whether an un-started sequence is harmless: a version that ramped from the
 *   first `step()` would push a torque into the axis the moment the object exists, with no `begin()` anywhere -- and a
 *   caller that only ever reads `torque_nm` would never see why.
 *
 *   The falsifier is the same call after `begin()`: it must ramp. Without that half, a `step()` that always answered
 *   `State` would pass the first assertion and the module would be dead.
 *
 *   `step()` 由控制环每周期调用，不管序列有没有启动，因为配方是在配置时校验的、环路没有理由再问一次。所以 Idle 那一条分
 *   支决定了"未启动的序列是否无害"：一个从第一次 `step()` 就开始爬升的版本，会在对象一存在时就把力矩推进轴里，而整个过
 *   程中没有任何 `begin()` ——只读 `torque_nm` 的调用方永远看不出原因。
 *
 *   伪证是 `begin()` 之后的同一次调用：它必须爬升。没有这一半，一个永远答 `State` 的 `step()` 也能过第一条断言，而那样
 *   整个模块就是死的。
 */
TEST(StallStart, AStepBeforeBeginReportsStateAndCommandsNothing) {
    foc::core::StallStart sequence(recipe(0.030F, 0.200F));
    ASSERT_EQ(foc::ErrorCode::Ok, sequence.validate());
    for (int i = 0; i < 50; ++i) {
        const auto out = sequence.step(kDt, 0.0F);
        EXPECT_EQ(foc::core::StallStartPhase::Idle, out.phase) << "tick " << i;
        EXPECT_EQ(foc::ErrorCode::State, out.status) << "tick " << i;
        EXPECT_FLOAT_EQ(0.0F, out.torque_nm) << "an un-started sequence commanded torque, tick " << i;
        EXPECT_FALSE(out.limited);
    }
    EXPECT_FLOAT_EQ(0.0F, sequence.torque_nm()) << "the state moved even though nothing was asserted";
    EXPECT_EQ(foc::core::StallStartPhase::Idle, sequence.phase());

    // And the same call after `begin()` ramps -- otherwise "answer State" would be a module that does nothing.
    // 而 `begin()` 之后同一次调用会爬升——否则"答 State"就是一个什么都不做的模块。
    ASSERT_TRUE(sequence.begin());
    const auto ramping = sequence.step(kDt, 0.0F);
    EXPECT_EQ(foc::core::StallStartPhase::Ramping, ramping.phase);
    EXPECT_EQ(foc::ErrorCode::Ok, ramping.status);
    EXPECT_GT(ramping.torque_nm, 0.0F) << "begin() did not start the ramp";
    EXPECT_NEAR(0.5F * kDt, ramping.torque_nm, 1e-7F) << "one period at 0.5 N*m/s";
}

/**
 * @brief [en] The ramp rate sets *when* the axis goes, not *how much* torque it takes -- the plateau is flat while the
 *             release instant moves as `1/rate`.
 *
 * @brief [zh] 斜坡速率决定轴 **何时** 动，不决定 **要多少** 力矩——平台是平的，而松脱时刻按 `1/速率` 移动。
 *
 *   This is the one property that makes the handover value usable: if the reported torque moved with the ramp rate,
 *   then "0.0307 N*m at 61 ms" would be a fact about a tuning constant rather than about the machine, and a
 *   commissioning run at a different rate would read a different breakaway point for the same motor.
 *
 *   Measured on the stiction plant (true breakaway 0.030 N*m):
 *
 *   | ramp (N*m/s) | released at | handover torque | |--------------|-------------|-----------------| | 0.25 | 121.3 ms
 *   | 0.0303 N*m | | 0.50 | 61.3 ms | 0.0307 N*m | | 1.00 | 31.3 ms | 0.0313 N*m |
 *
 *   So the time halves with the doubled rate (121.3/61.3 = 1.98, 61.3/31.3 = 1.96) while the torque moves by 1 mN*m
 *   over a 4x rate change. That residual is *observation* lag, not physics: the loop reads `state()` before the step
 *   that crosses the threshold, so at 1 N*m/s ten periods of blindness are ten times the torque. The bound below is
 *   therefore `one observation lag`, written as the number it was measured at rather than as a percentage.
 *
 *   这是让交接值可用的唯一属性：如果报告的力矩跟着斜坡速率走，那"61 ms 时 0.0307 N·m"就是关于一个整定常数的事实、而不是
 *   关于这台机器的事实，换一个速率做调试就会对同一台电机读出不同的起动点。
 *
 *   在静摩擦 plant 上实测（真实起动点 0.030 N·m）见上表。所以时间随速率翻倍而减半（121.3/61.3 = 1.98， 61.3/31.3 =
 *   1.96），而力矩在 4 倍速率变化下只动 1 mN·m。那点残留是 **观测** 滞后而不是物理：环路在跨阈值的那一拍之前读
 *   `state()`，所以 1 N·m/s 时十个周期的"看不见"就是十倍的力矩。因此下面的界写成实测的那个数，而不是一个百分比。
 */
TEST(StallStart, TheRampRateSetsWhenTheAxisGoesAndNotHowMuchTorqueItTakes) {
    constexpr float kTrueBreakawayNm = 0.030F;
    const BreakawayRun slow = run_breakaway(kTrueBreakawayNm, kTrueBreakawayNm, 0.25F);
    const BreakawayRun mid = run_breakaway(kTrueBreakawayNm, kTrueBreakawayNm, 0.50F);
    const BreakawayRun fast = run_breakaway(kTrueBreakawayNm, kTrueBreakawayNm, 1.00F);
    ASSERT_TRUE(slow.released);
    ASSERT_TRUE(mid.released);
    ASSERT_TRUE(fast.released);
    std::printf("\n[stall-start] ramp .25/.50/1.00 -> released at %.1f/%.1f/%.1f ms with %.4f/%.4f/%.4f N*m\n",
                static_cast<double>(slow.release_ms), static_cast<double>(mid.release_ms),
                static_cast<double>(fast.release_ms), static_cast<double>(slow.release_nm),
                static_cast<double>(mid.release_nm), static_cast<double>(fast.release_nm));

    // When: `1/rate`, to within one ramp step.
    // 何时：`1/速率`，误差在一个斜坡步之内。
    EXPECT_NEAR(2.0F, slow.release_ms / mid.release_ms, 0.06F) << "halving the rate must double the release time";
    EXPECT_NEAR(2.0F, mid.release_ms / fast.release_ms, 0.06F) << "doubling the rate must halve the release time";
    // How much: flat, to within one observation lag (measured 1 mN*m over a 4x rate change).
    // 多少：是平的，误差在一个观测滞后之内（4 倍速率变化实测 1 mN·m）。
    const float spread = std::max({slow.release_nm, mid.release_nm, fast.release_nm}) -
                         std::min({slow.release_nm, mid.release_nm, fast.release_nm});
    EXPECT_LT(spread, 0.0015F) << "the handover torque moved with the ramp rate: spread " << spread << " N*m";
    // And every one of them is the machine's breakaway point, not the ceiling.
    // 而它们每一个都是这台机器的起动点，不是限幅。
    for (const BreakawayRun &run : {slow, mid, fast}) {
        EXPECT_NEAR(kTrueBreakawayNm, run.release_nm, 0.0015F);
        EXPECT_LT(run.release_nm, 0.200F);
    }
}

/**
 * @brief [en] The recipe's breakaway number is checked and then never steers: a 50 % error in it changes the handover
 *             torque by **0.0000 N*m**, while the same sweep on a different plant moves it by 10 mN*m.
 *
 * @brief [zh] 配方里的起动点被检查、然后从不参与操纵：它有 50 % 的错，交接力矩改变 **0.0000 N·m** ；而同样的扫描换一台
 *             plant，它就动了 10 mN·m。
 *
 *   Found by asking which field of `StallStartConfig` a sensitivity table could be built for, and then reading the
 *   code: `breakaway_nm` appears in `validate()` and nowhere else. The ramp does not use it, the release test does not
 *   use it, `Failed` does not use it. So the sequence is *insensitive to its own brightest-sounding input* -- which is
 *   either a design property worth stating or a bug, and the second half of this case is what decides between them.
 *
 *   It is a property, and the reason is the mechanism the header claims: the handover value is the torque the rotor
 *   actually moved on, measured, so no recipe number can bias it. Measured, true breakaway 0.030 N*m, recipes at 0.015
 *   / 0.030 / 0.045: **0.0307 N*m at 61.3 ms, all three**. Then the same three recipes against a plant whose breakaway
 *   is 0.040 N*m: **0.0405 N*m at 81.0 ms, all three** -- so the number tracks the machine and not the recipe, and a
 *   version that reported `config_.breakaway_nm` would fail the second sweep while passing the first.
 *
 *   What the recipe *does* do is decide whether the sequence is allowed to run at all (`max_torque_nm > breakaway_nm`),
 *   so a wild over-estimate is not free: at 0.199 N*m against a 0.2 N*m limit it validates, and at 0.201 it is refused.
 *   Both halves are asserted, because "the number is harmless" and "the number is ignored" are different statements.
 *
 *   这是靠"`StallStartConfig` 里哪个字段能建敏感度表"这个问题找到的，然后去读代码： `breakaway_nm` 只出现在
 *   `validate()` 里，别处没有。斜坡不用它、松脱判定不用它、 `Failed` 不用它。所以序列对 **自己听起来最重要的那个输入**
 *   不敏感——这要么是一条值得写下来的设计属性，要么是一个缺陷，而本用例的后半段就是用来分开这两者的。
 *
 *   它是属性，理由正是头文件声称的机理：交接值就是转子真正动起来的那个力矩，是 **测出来的** ，所以任何配方数字都偏不了
 *   它。实测（真实起动点 0.030 N·m，配方 0.015/0.030/0.045）：三个都是 **61.3 ms 时 0.0307 N·m** 。再把同样三份配方打到
 *   起动点为 0.040 N·m 的 plant 上：三个都是 **81.0 ms 时 0.0405 N·m** ——所以这个数跟着机器走、不跟着配方走，而一个报
 *   `config_.breakaway_nm` 的版本会过第一轮、败在第二轮。
 *
 *   配方真正决定的是序列 **允不允许跑** （ `max_torque_nm > breakaway_nm`），所以离谱的高估并非免费：0.199 N·m 对 0.2
 *   N·m 的限幅能过校验，0.201 就被拒。两半都断言，因为"这个数无害"与"这个数被忽略"是两句不同的话。
 */
TEST(StallStart, TheRecipeBreakawayIsCheckedButNeverSteersTheHandoverValue) {
    constexpr float kTrueNm = 0.030F;
    const BreakawayRun low = run_breakaway(0.015F, kTrueNm, 0.5F);
    const BreakawayRun honest = run_breakaway(0.030F, kTrueNm, 0.5F);
    const BreakawayRun high = run_breakaway(0.045F, kTrueNm, 0.5F);
    ASSERT_TRUE(honest.released);
    std::printf("[stall-start] recipe .015/.030/.045 on a 0.030 machine -> %.4f/%.4f/%.4f N*m at %.1f/%.1f/%.1f ms\n",
                static_cast<double>(low.release_nm), static_cast<double>(honest.release_nm),
                static_cast<double>(high.release_nm), static_cast<double>(low.release_ms),
                static_cast<double>(honest.release_ms), static_cast<double>(high.release_ms));

    // A 50 % error on either side moves the handover value by nothing at all.
    // 正负 50 % 的错都不移动交接值。
    EXPECT_FLOAT_EQ(honest.release_nm, low.release_nm) << "a low recipe biased the handover value";
    EXPECT_FLOAT_EQ(honest.release_nm, high.release_nm) << "a high recipe biased the handover value";
    EXPECT_FLOAT_EQ(honest.release_ms, low.release_ms);
    EXPECT_FLOAT_EQ(honest.release_ms, high.release_ms);

    // The falsifier: change the *machine*, not the recipe, and the number follows. A configuration-driven value cannot
    // do this.
    // 伪证：改 **机器** 而不是配方，这个数就跟着动。由配置决定的数做不到这一点。
    const BreakawayRun heavier = run_breakaway(0.030F, 0.040F, 0.5F);
    ASSERT_TRUE(heavier.released);
    EXPECT_NEAR(0.040F, heavier.release_nm, 0.001F) << "the handover value did not follow the machine";
    EXPECT_GT(heavier.release_nm, honest.release_nm + 0.008F) << "10 mN*m of extra stiction must show up in the value";
    EXPECT_GT(heavier.release_ms, honest.release_ms + 10.0F) << "and 10 mN*m more ramp at 0.5 N*m/s is 20 ms";

    // What the recipe *is* for: the refusal, and it is a real gate rather than decoration.
    // 配方真正的作用：拒绝，而且它是一道真闸门、不是装饰。
    auto near_limit = recipe(0.199F, 0.200F);
    EXPECT_EQ(foc::ErrorCode::Ok, foc::core::StallStart(near_limit).validate());
    auto past_limit = recipe(0.201F, 0.200F);
    EXPECT_EQ(foc::ErrorCode::Param, foc::core::StallStart(past_limit).validate());
}

/**
 * @brief [en] A jammed axis is declared failed at `limit/rate + give_up`, exactly, and that instant moves with
 *             `give_up` one for one.
 *
 * @brief [zh] 卡住的轴在 `限幅/速率 + 宽限` 被判定失败，精确到拍，而那个时刻随 `give_up` 一比一移动。
 *
 *   The existing jam case asserts that it fails and that it did not fail too early. Both are weak in the same
 *   direction: a `Failed` that arrived twice as late as it should would pass. The instant is closed-form -- the ramp
 *   clips at `max/rate` and the grace starts there -- so it can be compared to arithmetic instead of to "later than
 *   0.55 s".
 *
 *   Measured with 0.2 N*m at 0.5 N*m/s and a 1e-4 s period: clipping at 4000 ticks (400.0 ms) and `Failed` at
 *   **exactly 6000**ticks for a 0.2 s grace. With a 0.4 s grace it is**8001**, not 8000: `hold_s_` is a float32
 *   accumulator, and 4000 additions of 1e-4 s do not land on 0.4 exactly. That one tick is recorded rather than tuned
 *   away -- the accumulator cannot be made exact for every period, and a criterion that demanded 8000 would be a
 *   criterion about float32 rather than about the sequence.
 *
 *   已有的卡死用例断言了它会失败、且没有太早失败。两者在同一种意义上是弱的：一个迟到一倍的 `Failed` 也会通过。这个时刻
 *   是闭式的——斜坡在 `限幅/速率` 处削平、宽限从那里开始——所以它可以对着算术比，而不是对着"晚于 0.55 s"。
 *
 *   实测（0.2 N·m、0.5 N·m/s、1e-4 s 周期）：4000 拍（400.0 ms）削平，0.2 s 宽限下 `Failed` 正好落在第 **6000** 拍；
 *   0.4 s 宽限下是 **8001** 而不是 8000—— `hold_s_` 是 float32 累加器，1e-4 s 加 4000 次不会正好落在 0.4 上。这一拍是
 *   **记录** 而不是调掉的：累加器不可能对每个周期都精确，而一条要求 8000 的判据就成了关于 float32 的判据，而不是关于这
 *   个序列的判据。
 */
TEST(StallStart, AFailedAxisIsDeclaredAtTheClippingTimePlusTheGrace) {
    auto jammed_ticks = [](float give_up_s) {
        foc::core::StallStart sequence(recipe(0.030F, 0.200F));
        foc::core::StallStartConfig config = sequence.config();
        config.give_up_s = give_up_s;
        sequence.set_config(config);
        static_cast<void>(sequence.begin());
        int ticks = 0;
        while ((sequence.phase() == foc::core::StallStartPhase::Ramping) && (ticks < 100000)) {
            static_cast<void>(sequence.step(kDt, 0.0F));
            ++ticks;
        }
        return ticks;
    };
    // 0.200 N*m at 0.5 N*m/s clips at 4000 ticks.
    // 0.200 N·m 以 0.5 N·m/s 在 4000 拍处削平。
    const int short_grace = jammed_ticks(0.2F);
    const int long_grace = jammed_ticks(0.4F);
    std::printf("\n[stall-start] jammed: grace 0.2 s -> %d ticks (%.1f ms), grace 0.4 s -> %d ticks (%.1f ms)\n",
                short_grace, static_cast<double>(short_grace) * kDt * 1e3, long_grace,
                static_cast<double>(long_grace) * kDt * 1e3);

    EXPECT_EQ(6000, short_grace) << "4000 ticks of ramp plus 2000 of grace, exactly";
    // The grace enters one for one: doubling it adds 2000 ticks, and the one extra tick is the float32 accumulator.
    // 宽限一比一进入：翻倍就是多 2000 拍，而多出来的那一拍是 float32 累加器。
    EXPECT_GE(long_grace - short_grace, 2000);
    EXPECT_LE(long_grace - short_grace, 2001) << "the grace is not the only thing that moved";
    EXPECT_GE(long_grace, 8000);
}

