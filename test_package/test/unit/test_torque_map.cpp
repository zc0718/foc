// The torque map, checked two ways: against the expression it claims to be (arithmetic), and against the mechanics the
// SIL bench actually obeys (`J*dw/dt + B*w`), which shares no line with it.
// 力矩地图的两条验法：对着它声称是的那条表达式（算术），以及对着 SIL 台架真正服从的力学（`J·dω/dt + B·ω`）——后者
// 与它不共享任何一行。

#include <cmath>
#include <cstddef>
#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/transforms.hpp>
#include <foc/motor/torque_map.hpp>

#include "sil_abc_plant.hpp"
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


namespace {

namespace core = foc::core;
namespace sil = foc::sil;

// The measured error bars this table is built from. They are deliberately not round numbers picked to make a
// table look reasonable: each is a number this project measured somewhere else, and naming the source is what
// separates a sensitivity table from the author's imagination.
// 本表依据的实测误差条。它们刻意不是为了让表格好看而挑的整数：每一个都是本项目在别处测出来的数字，写明出处
// 正是"敏感度表"与"作者的想象力"之间的分别。
constexpr float kPsiFErrorOrnl = 0.0893F;   // A-93: FEMM psi_f is +8.93 % against the ORNL bench measurement.
constexpr float kLdSaturationA92 = -0.134F; // A-92: a 50-100 A nameplate reads 13.4 % high on Ld at 200 A.
constexpr float kLqSaturationA92 = -0.081F; // A-92: and 8.1 % high on Lq.

auto scaled(float value, float relative) noexcept -> float { return value * (1.0F + relative); }

struct AbcReading final {
    float id_a;
    float iq_a;
    float torque_nm;
};

// The six-point recipe from `test_abc_plant.cpp`, reused rather than reinvented: the plant is driven with the
// inverse-Park of the dq voltage the dq model predicts, integrated at a frozen frame, and read back at the
// angle it finished on. No d-q quantity is formed anywhere inside the plant.
// 复用 `test_abc_plant.cpp` 的六点配方而不是另造一个：用 dq 模型预测的 dq 电压的 inverse-Park 驱动 plant，在冻结
// 坐标下积分，并在它结束时的角度上读回。plant 内部任何地方都不构造 d-q 量。
auto drive_abc(float omega_e, float id_command, float iq_command) noexcept -> AbcReading {
    const sil::MachineSpec s = sil::nominal_spec();
    const float psi = s.kt / (1.5F * static_cast<float>(s.pole_pairs));
    const core::Dq<float> v_dq{(s.rs * id_command) - (omega_e * s.lq * iq_command),
                               (s.rs * iq_command) + (omega_e * s.ld * id_command) + (omega_e * psi)};
    const sil::AbcPlant::Params params{s.rs, s.ld, psi, s.pole_pairs};
    sil::AbcPlant plant(params);
    plant.set_state(0.0F, omega_e);
    const float dt = 5.0e-6F;
    const int steps = static_cast<int>((0.02F / dt) + 0.5F);
    for (int k = 0; k < steps; ++k) {
        plant.set_voltage(core::inv_clarke(core::inv_park(v_dq, plant.omega_e() * static_cast<float>(k) * dt)));
        plant.advance(dt);
    }
    const float theta_end = plant.omega_e() * static_cast<float>(steps) * dt;
    const core::Dq<float> i_dq = core::park(core::clarke(plant.current()), theta_end);
    return AbcReading{i_dq.d, i_dq.q, plant.torque()};
}

} // namespace


TEST(TorqueMap, TheNameplateErrorBarPropagatesAndIsReported) {
    // The first half of what L3 asks for: a **measured** sensitivity -- how far the number moves when the
    // nameplate is wrong in the ways it is known to be wrong.
    //
    // Criterion, written before the run. A relative error `e` on `kt` moves the estimate by
    // `e*kt*iq / estimate`, and at the `id` used here the reluctance term *subtracts*, so that denominator is
    // smaller than `kt*iq`: the error can only be amplified. With `e = 8.93 %` the result is therefore above
    // 8.93 % and so above the map's own 5 % band. That is the finding this case exists to produce, and it is
    // not a defect in the map: the map converts a nameplate, so its absolute accuracy is the nameplate's.
    // L3 要求的前一半：**实测**敏感度——当铭牌以已知的方式错时，这个数会移动多远。
    const float ld = 4e-4F;
    const float lq = 8e-4F;
    const float kt = 0.05F;
    const float id_a = 1.0F;
    const float iq_a = 2.0F;

    const auto base = foc::torque_map(config_with(kt, ld, lq, 5.0F), -1.0F, 1.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, base.status);
    const float reference = foc::torque_estimate(base, id_a, iq_a);
    ASSERT_GT(reference, 0.0F);

    struct Case final {
        const char *name;
        float dkt;
        float dld;
        float dlq;
    };
    const Case cases[] = {
        {"psi_f +8.93 % (ORNL measured)", kPsiFErrorOrnl, 0.0F, 0.0F},
        {"Ld -13.4 % (a 200 A nameplate)", 0.0F, kLdSaturationA92, 0.0F},
        {"Lq -8.1 % (a 200 A nameplate)", 0.0F, 0.0F, kLqSaturationA92},
        {"all three at once", kPsiFErrorOrnl, kLdSaturationA92, kLqSaturationA92},
    };

    std::printf("\n[L3] torque map against the nameplate it was handed\n");
    std::printf("        %-32s %12s %10s\n", "nameplate error", "torque N*m", "error");
    std::printf("        %-32s %12.6f %9s\n", "none", static_cast<double>(reference), "-");

    float psi_effect = 0.0F;
    for (const Case &c : cases) {
        const auto map =
            foc::torque_map(config_with(scaled(kt, c.dkt), scaled(ld, c.dld), scaled(lq, c.dlq), 5.0F), -1.0F, 1.0F);
        ASSERT_EQ(foc::ErrorCode::Ok, map.status) << c.name;
        const float estimate = foc::torque_estimate(map, id_a, iq_a);
        const float relative = (estimate - reference) / reference;
        if ((c.dkt != 0.0F) && (c.dld == 0.0F)) {
            psi_effect = relative;
        }
        std::printf("        %-32s %12.6f %+9.3f%%\n", c.name, static_cast<double>(estimate),
                    100.0 * static_cast<double>(relative));
        // Arithmetic, restated in closed form: the map is linear in each knob, so this must reproduce the code.
        // 算术，用闭式复述一遍：地图对每个旋钮都是线性的，所以这条必须复现代码。
        const float expected = (scaled(kt, c.dkt) * iq_a) +
                               (1.5F * 7.0F * (scaled(ld, c.dld) - scaled(lq, c.dlq)) * id_a * iq_a);
        EXPECT_NEAR(expected, estimate, 1.0e-7F) << c.name;
    }

    EXPECT_GT(psi_effect, kPsiFErrorOrnl) << "a subtracting reluctance term can only amplify a kt error, never damp it";
    EXPECT_GT(std::fabs(psi_effect), 0.05F)
        << "and that puts the externally measured psi_f bar outside the map's own 5 % band -- a fact about the "
           "nameplate, not about the code";
}


TEST(TorqueMap, ThePhaseVariableModelWitnessesTheMagnetTorqueAndNotTheSaliency) {
    // The second half of what L3 asks for: a cross-check against a model that never forms a d-q quantity and
    // never uses `kt`. `AbcPlant` integrates three phase fluxes and reads torque off the co-energy, which is
    // as independent as this repository can currently get -- and the point of the two tables below is to state
    // exactly how far that independence reaches.
    // L3 要求的后一半：对着一个从不构造 d-q 量、也从不使用 `kt` 的模型对拍。
    const auto surface = foc::torque_map(sil::nominal_motor_config(), -1.0F, 1.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, surface.status);
    EXPECT_FLOAT_EQ(0.0F, surface.reluctance) << "the nominal machine is surface-mounted, so the term is zero";
    const auto salient = foc::torque_map(config_with(0.05F, 4e-4F, 8e-4F, 5.0F), -1.0F, 1.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, salient.status);

    std::printf("\n[L3] phase-variable co-energy torque vs the map, id = 0\n");
    std::printf("        %7s %6s | %11s %11s %9s\n", "w_e", "i_q", "abc N*m", "map N*m", "delta");
    for (const float omega_e : {0.0F, 200.0F, 800.0F}) {
        for (const float iq_command : {1.0F, 2.0F}) {
            const AbcReading r = drive_abc(omega_e, 0.0F, iq_command);
            const float mapped = foc::torque_estimate(surface, r.id_a, r.iq_a);
            ASSERT_GT(std::fabs(r.torque_nm), 1.0e-3F) << "omega " << omega_e << " iq " << iq_command;
            std::printf("        %7.0f %6.1f | %11.6f %11.6f %+8.3f%%\n", static_cast<double>(omega_e),
                        static_cast<double>(iq_command), static_cast<double>(r.torque_nm),
                        static_cast<double>(mapped), 100.0 * static_cast<double>((mapped - r.torque_nm) / r.torque_nm));
            EXPECT_NEAR(1.0, mapped / r.torque_nm, 0.02)
                << "omega " << omega_e << " iq " << iq_command << ": the co-energy model and kt*iq must agree";
        }
    }

    std::printf("\n[L3] what the phase-variable leg cannot see\n");
    std::printf("        %6s %6s | %11s %11s %9s %9s\n", "i_d", "i_q", "abc N*m", "map N*m", "saliency", "share");
    for (const float id_command : {-1.0F, 1.0F}) {
        const AbcReading r = drive_abc(200.0F, id_command, 1.0F);
        const float mapped = foc::torque_estimate(salient, r.id_a, r.iq_a);
        const float reluctance = salient.reluctance * r.id_a * r.iq_a;
        const float share = reluctance / r.torque_nm;
        std::printf("        %6.2f %6.2f | %11.6f %11.6f %+9.6f %+8.2f%%\n", static_cast<double>(r.id_a),
                    static_cast<double>(r.iq_a), static_cast<double>(r.torque_nm), static_cast<double>(mapped),
                    static_cast<double>(reluctance), 100.0 * static_cast<double>(share));
        // The whole of the disagreement, and nothing else, is the saliency term.
        // 不一致的全部、且仅有磁阻项。
        EXPECT_NEAR(reluctance, mapped - r.torque_nm, 1.0e-5F + (1.0e-3F * std::fabs(mapped)));
        EXPECT_GT(std::fabs(share), 0.05F)
            << "the blind spot is not small, which is exactly why it is measured and named rather than glossed over";
    }
}


TEST(TorqueMap, ThePhaseVariableCrossCheckRejectsAWrongGain) {
    // A cross-check that cannot go red proves nothing, so the same comparison is run against a map whose gain
    // is 10 % low. It has to fail: if the co-energy witness absorbed a gain error, the leg above would be
    // decoration.
    // 不能变红的对拍证明不了任何事，所以把同一个比较对一个增益低 10 % 的地图再跑一遍。
    const sil::MachineSpec s = sil::nominal_spec();
    const auto wrong = foc::torque_map(config_with(0.9F * s.kt, s.ld, s.lq, 5.0F), -1.0F, 1.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, wrong.status);
    const AbcReading r = drive_abc(200.0F, 0.0F, 1.0F);
    const float ratio = foc::torque_estimate(wrong, r.id_a, r.iq_a) / r.torque_nm;
    std::printf("\n[L3] falsifier: a 10 %% low gain reads %.6f against the co-energy witness\n",
                static_cast<double>(ratio));
    EXPECT_LT(ratio, 0.95F) << "the cross-check has to reject a gain error of the size it claims to detect";
}
