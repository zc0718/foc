// The torque map, checked two ways: against the expression it claims to be (arithmetic), and against the mechanics the
// SIL bench actually obeys (`J*dw/dt + B*w`), which shares no line with it.
// 力矩地图的两条验法：对着它声称是的那条表达式（算术），以及对着 SIL 台架真正服从的力学（`J·dω/dt + B·ω`）——后者
// 与它不共享任何一行。

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

#include <foc/motor/torque_map.hpp>

#include "sil_bench.hpp"


namespace {

auto config_with(float kt, float ld, float lq, float i_max) -> foc::motor::MotorConfig {
    foc::motor::MotorConfig config{};
    config.rs = 0.5F;
    config.ld = ld;
    config.lq = lq;
    config.kt = kt;
    config.j = 1e-5F;
    config.b = 1e-6F;
    config.pole_pairs = 7;
    config.i_max_a = i_max;
    config.pwm_freq_hz = 20000.0F;
    config.deadtime_ns = 500.0F;
    return config;
}

} // namespace



TEST(TorqueMap, TheEstimateIsTheExpressionThePlantIntegrates) {
    // With saliency the torque depends on `id` too, and the sign of that dependence is the sign of `ld - lq`. A map that
    // dropped the term would still be monotone in `iq` and would still look plausible on a surface machine.
    // 有凸极时力矩还依赖 `id`，这个依赖的符号就是 `ld − lq` 的符号。漏掉该项的地图在 `iq` 上仍然单调，在表贴机上仍然
    // 看起来合理。
    const auto salient = foc::torque_map(config_with(0.05F, 4e-4F, 8e-4F, 5.0F), -1.0F, 1.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, salient.status);

    const float id_a = 0.3F;
    const float iq_a = 2.0F;
    const float expected = (0.05F * iq_a) + (1.5F * 7.0F * (4e-4F - 8e-4F) * id_a * iq_a);
    EXPECT_NEAR(expected, foc::torque_estimate(salient, id_a, iq_a), 1.0e-7F);

    // The same machine with the saliency the other way round: the reluctance term changes sign, and the effect of `id`
    // with it.
    // 同一台机器、凸极反过来：磁阻项变号，`id` 的作用随之变号。
    const auto inverted = foc::torque_map(config_with(0.05F, 8e-4F, 4e-4F, 5.0F), -1.0F, 1.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, inverted.status);
    // `ld < lq` (the usual interior machine) makes positive `id` *reduce* the torque; `ld > lq` makes it add. The first
    // draft of this case asserted the opposite, which is the sign convention being wrong in the test rather than in the
    // map -- found by the case failing against arithmetic that was already correct.
    // `ld < lq`（常见的内嵌式）下正的 `id` *减少*力矩；`ld > lq` 下则增加。本用例第一版断言了相反的方向，那是测试里的
    // 符号约定错了、不是地图错了——是它对着本来就正确的算术失败而被发现的。
    EXPECT_LT(foc::torque_estimate(salient, id_a, iq_a), 0.05F * iq_a);
    EXPECT_GT(foc::torque_estimate(inverted, id_a, iq_a), 0.05F * iq_a);
}


TEST(TorqueMap, TheCommandPathRoundTripsThroughTheEstimate) {
    // A commanded torque that the estimate would not confirm is a command that does not deliver what it says.
    // 一条估计不认可的指令，就是一条不交付它声称之物的指令。
    const auto map = foc::torque_map(config_with(0.05F, 4e-4F, 8e-4F, 5.0F), -1.0F, 1.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, map.status);

    for (const float id_a : {0.0F, 0.5F, -0.5F}) {
        for (const float wanted : {0.02F, 0.1F, -0.15F}) {
            const foc::TorqueCommand command = foc::torque_command(map, wanted, id_a);
            ASSERT_EQ(foc::ErrorCode::Ok, command.status) << "id " << id_a << " wanted " << wanted;
            EXPECT_FALSE(command.limited);
            EXPECT_NEAR(wanted, foc::torque_estimate(map, id_a, command.iq_a), 1.0e-6F)
                << "id " << id_a << " wanted " << wanted;
        }
    }
}


TEST(TorqueMap, AOneWayLimitRefusesToPullBackwards) {
    // The tensioner case: 0 N*m on the bottom, and a request below it must be *reported* as clipped rather than
    // silently obeyed or silently ignored.
    // 张力控制的情形：下限 0 N·m，而低于它的请求必须被**报告**为裁剪，而不是静默执行或静默忽略。
    const auto map = foc::torque_map(config_with(0.05F, 4e-4F, 8e-4F, 5.0F), 0.0F, 1.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, map.status);
    EXPECT_FLOAT_EQ(0.0F, map.min_nm);
    EXPECT_FLOAT_EQ(0.25F, map.max_nm) << "the upper limit was narrowed to kt * i_max";

    const foc::TorqueCommand forward = foc::torque_command(map, 0.2F, 0.0F);
    EXPECT_FALSE(forward.limited);
    EXPECT_NEAR(0.2F, forward.torque_nm, 1.0e-6F);

    const foc::TorqueCommand backward = foc::torque_command(map, -0.2F, 0.0F);
    EXPECT_TRUE(backward.limited);
    EXPECT_FLOAT_EQ(0.0F, backward.torque_nm);
    EXPECT_FLOAT_EQ(0.0F, backward.iq_a);
}


TEST(TorqueMap, TheLimitsAreNarrowedToWhatTheInverterCanDeliver) {
    const auto map = foc::torque_map(config_with(0.05F, 4e-4F, 8e-4F, 2.0F), -100.0F, 100.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, map.status);
    EXPECT_FLOAT_EQ(-0.1F, map.min_nm) << "kt * i_max bounds both ends";
    EXPECT_FLOAT_EQ(0.1F, map.max_nm);

    const foc::TorqueCommand over = foc::torque_command(map, 0.5F, 0.0F);
    EXPECT_TRUE(over.limited);
    EXPECT_NEAR(0.1F, over.torque_nm, 1.0e-6F);
    EXPECT_NEAR(2.0F, over.iq_a, 1.0e-6F);
}


TEST(TorqueMap, ANameplateThatCannotProduceTorqueIsRefused) {
    const float nan = std::nanf("");
    EXPECT_EQ(foc::ErrorCode::Param, foc::torque_map(config_with(0.0F, 4e-4F, 8e-4F, 5.0F), -1.0F, 1.0F).status)
        << "kt <= 0 says the machine produces no torque per amp";
    EXPECT_EQ(foc::ErrorCode::Param, foc::torque_map(config_with(-0.05F, 4e-4F, 8e-4F, 5.0F), -1.0F, 1.0F).status);
    EXPECT_EQ(foc::ErrorCode::Param, foc::torque_map(config_with(0.05F, 4e-4F, 8e-4F, 0.0F), -1.0F, 1.0F).status);
    EXPECT_EQ(foc::ErrorCode::Param, foc::torque_map(config_with(0.05F, 4e-4F, 8e-4F, 5.0F), 1.0F, -1.0F).status)
        << "min above max is not an interval";
    EXPECT_EQ(foc::ErrorCode::Param, foc::torque_map(config_with(0.05F, 4e-4F, 8e-4F, 5.0F), nan, 1.0F).status);

    auto no_poles = config_with(0.05F, 4e-4F, 8e-4F, 5.0F);
    no_poles.pole_pairs = 0U;
    EXPECT_EQ(foc::ErrorCode::Param, foc::torque_map(no_poles, -1.0F, 1.0F).status);

    // An unusable map still writes every field, so a caller who ignores the status gets zeros rather than an
    // uninitialised limit that might be permissive.
    // 不可用的地图仍然写满字段，因此忽略状态的调用方拿到的是零，而不是一条可能很宽的未初始化限幅。
    const auto refused = foc::torque_map(config_with(0.0F, 4e-4F, 8e-4F, 5.0F), -1.0F, 1.0F);
    EXPECT_FLOAT_EQ(0.0F, refused.min_nm);
    EXPECT_FLOAT_EQ(0.0F, refused.max_nm);
    EXPECT_FLOAT_EQ(0.0F, foc::torque_estimate(refused, 1.0F, 1.0F));
}


TEST(TorqueMap, TheEstimateMatchesTheMechanicsOfTheSilBench) {
    // The criterion, and it does not share a line with the estimate: the estimate comes from the measured dq currents
    // through the nameplate, the mechanical torque from `J*dw/dt + B*w` through the recorded speed. Two independent
    // measurements of one quantity, in the same window.
    //
    // The comparison is restricted to the ramp: at steady state `dw/dt` is zero and the mechanical torque collapses to
    // `B*w`, so a steady-state comparison would test the friction model, not the torque map.
    // 判据，且它与估计不共享一行：估计由实测 dq 电流经铭牌得到，机械力矩由录下的转速经 `J·dω/dt + B·ω` 得到。同一个量的
    // 两个独立测量，在同一个窗口里。比较限定在爬升段：稳态下 `dω/dt` 为零、机械力矩塌缩成 `B·ω`，在稳态比较测的是摩擦
    // 模型而不是力矩地图。
    const foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
    const foc::sil::PlantParams plant = foc::sil::nominal_plant_params();
    const foc::motor::MotorConfig mc = foc::sil::nominal_motor_config();
    const auto map = foc::torque_map(mc, -1.0F, 1.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, map.status);

    // 2 A stays inside the 5 A limit; 20 A makes the loop saturate, and a torque map that only worked in the linear
    // region would be a map for a bench, not for a machine.
    // 2 A 在 5 A 限幅之内；20 A 让环路饱和，而只在性线区工作的力矩地图是台架的、不是机器的。
    for (const float iq_command_a : {2.0F, 20.0F}) {
        foc::sil::Bench bench(cfg, plant);
        bench.motor().set_torque(0.0F);
        ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());
        bench.run(2e-3F);
        bench.motor().set_torque(iq_command_a);
        std::vector<foc::sil::Sample> trace;
        bench.run(0.02F, [&trace](const foc::sil::Bench &b) { trace.push_back(b.sample()); });
        ASSERT_GT(trace.size(), 20U);

        double sum_estimate = 0.0;
        double sum_mechanical = 0.0;
        std::size_t used = 0U;
        for (std::size_t i = 1U; i + 1U < trace.size(); ++i) {
            const float estimate = foc::torque_estimate(map, trace[i].id_a, trace[i].iq_a);
            const float acceleration = (trace[i + 1U].speed - trace[i - 1U].speed) / (2.0F * cfg.dt_ctrl_s);
            const float mechanical = (plant.j * acceleration) + (plant.b * trace[i].speed);
            // Only the samples where something is actually being produced: dividing by a near-zero torque makes the
            // relative error meaningless.
            // 只取真正在产生力矩的样本：除以接近零的力矩会让相对误差失去意义。
            if (std::fabs(estimate) < 0.02F) {
                continue;
            }
            sum_estimate += static_cast<double>(estimate);
            sum_mechanical += static_cast<double>(mechanical);
            ++used;
        }
        ASSERT_GT(used, 10U) << "iq " << iq_command_a;
        const double relative = (sum_estimate - sum_mechanical) / sum_mechanical;
        std::printf("TORQUE iq %5.1f A: estimate %.6f N*m, mechanics %.6f N*m, %+.3f%% over %zu samples\n",
                    iq_command_a, sum_estimate / static_cast<double>(used), sum_mechanical / static_cast<double>(used),
                    100.0 * relative, used);
        // Measured: +0.006 % at 2 A and +0.007 % at 20 A, against a 5 % band. The band is the plan's, and the measured
        // agreement is three orders of magnitude inside it -- the two routes are the same physics read twice, and the
        // residual is the forward-difference error in `dw/dt`.
        // 实测：2 A 下 +0.006%、20 A 下 +0.007%，对 5% 的带。带是计划定的，而实测一致度在带内深三个数量级——两条路是
        // 同一份物理被读了两次，残差是 `dω/dt` 的前向差分误差。
        EXPECT_NEAR(1.0, sum_estimate / sum_mechanical, 0.05) << "iq " << iq_command_a;
    }
}
