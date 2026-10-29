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
    // 上限低于起动力矩的配方只可能产出 `Failed`。接受它就等于接受一条**事先已知不工作**的序列，而调用方要 200 ms 后
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
    // 起动点被 plant 报出的两个力矩**夹住**，且间隔一个斜坡步：在 `last_at_rest` 时没动，在 `first_moving` 时动了。要求
    // 两者夹住配置值是物理断言；要求其中一个逐位等于它，等于要求一次阈值穿越正好落在采样点上。
    // 1 mN*m of slack is the *observation* lag, measured at 0.5 mN*m: the plant's threshold is exact inside the model,
    // but `state()` is read before the step that crosses it, so the crossing is seen about ten periods late.
    // 1 mN·m 的余量是**观测**滞后，实测 0.5 mN·m：plant 内部阈值是精确的，但 `state()` 在读它的那一步之前被读取，于是
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
