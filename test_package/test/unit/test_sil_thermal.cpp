// F36, the port half: a winding that heats itself. Every criterion here compares the plant against a **closed form** of
// the one-node equation it claims to integrate, because "the temperature goes up when current flows" is satisfied by any
// monotone mistake, and the interesting part -- what the steady state actually is, and when there is none -- is exactly
// what a proportionality would get wrong.
// F36 的端口半：会自己发热的绕组。这里每条判据都把 plant 与它所声称积分的那条单节点方程的**闭式解**相比，因为"有电流温度
// 就上升"这个条件，任何单调的错误都满足，而真正有意思的部分——稳态到底是多少、以及什么时候根本没有稳态——恰恰是"正比
// 关系"这种实现会算错的东西。

#include <cmath>

#include <gtest/gtest.h>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

using foc::sil::PmsmPlant;

constexpr float kAlphaCopper = 3.93e-3F; // 1/K, copper 铜的温度系数

/** @brief The closed-form settle of the one-node equation when the loss grows with `Rs(T)`. */
auto settle_current_fed(float rs20, float current_a, float rt_k_per_w, float alpha_per_k, float t_ambient_c) -> float {
    // P(T) = 1.5*i^2*rs20*(1 + alpha*(T-20)), so T = Ta + Rt*P(T) is linear in T and one root is all there is:
    // T = (Ta + Rt*P20*(1-20*alpha)) / (1 - Rt*P20*alpha), and the denominator is where runaway begins.
    // P(T) = 1.5·i²·rs20·(1 + α·(T−20))，于是 T = Ta + Rt·P(T) 对 T 是线性的、只有一个根，分母归零处就是失控的边界。
    const float p20 = 1.5F * current_a * current_a * rs20;
    return (t_ambient_c + (rt_k_per_w * p20 * (1.0F - (20.0F * alpha_per_k)))) /
           (1.0F - (rt_k_per_w * p20 * alpha_per_k));
}

/** @brief The settle of a voltage-fed winding: `(T-Ta)*rs20*(1+alpha*(T-20)) = 1.5*Rt*v^2`, the positive root. */
auto settle_voltage_fed(float rs20, float v_d, float rt_k_per_w, float alpha_per_k, float t_ambient_c) -> float {
    const float c = 1.5F * rt_k_per_w * v_d * v_d;
    const float qa = alpha_per_k;
    const float qb = 1.0F - (20.0F * alpha_per_k) - (alpha_per_k * t_ambient_c);
    const float qc = (-t_ambient_c * (1.0F - (20.0F * alpha_per_k))) - (c / rs20);
    const float disc = (qb * qb) - (4.0F * qa * qc);
    if (!(disc > 0.0F)) {
        return NAN; // no steady state exists 不存在稳态
    }
    return (-qb + std::sqrt(disc)) / (2.0F * qa);
}

} // namespace

// The headline: with the temperature coefficient off the model must settle exactly where `Ta + Rt*P` says, and reach 63.2 %
// of that rise after exactly one `Rt*Ct`. Two numbers, and each of them can be wrong on its own -- a wrong `1.5` in the
// loss moves the first and not the second, a wrong `Ct` moves the second and not the first.
// 要点：温度系数关掉时，模型必须精确收敛在 `Ta + Rt·P` 指的地方，并且在恰好一个 `Rt·Ct` 之后到达那个温升的 63.2 %。两个数，
// 各自都能单独错——损耗里那个 `1.5` 错了只动第一个，`Ct` 错了只动第二个。
TEST(SilThermal, SettlesAtTaPlusRtTimesLossAndReachesSixtyThreePercentAtOneTau) {
    const auto nameplate = foc::sil::nominal_plant_params();
    PmsmPlant plant{nameplate};
    const float v_d =
        0.5F; // a d-axis voltage with the rotor at rest: torque is zero, so the rotor stays put 静止时加 d 轴电压：转矩为零，转子不动
    foc::sil::PmsmPlant::ThermalConfig thermal{};
    thermal.rt_k_per_w = 8.0F;
    thermal.ct_j_per_k = 0.5F; // tau = 4 s
    thermal.t_ambient_c = 20.0F;
    plant.set_thermal(thermal);
    plant.set_winding_temperature(20.0F, 0.0F); // no drift: the loss is a constant 不漂：损耗是常数
    plant.set_voltage_dq(foc::core::Dq<float>{v_d, 0.0F});

    const float expected_loss = 1.5F * v_d * v_d / nameplate.rs; // i_d = v_d/rs at steady state 稳态 i_d = v_d/rs
    const float expected_rise = thermal.rt_k_per_w * expected_loss;
    ASSERT_FLOAT_EQ(4.0F, plant.thermal_time_constant_s());

    const float dt = 5.0e-5F;
    float at_tau = 0.0F;
    float elapsed = 0.0F;
    // Eight time constants, so the 0.5 % bound below is about the model and not about `1 - exp(-t/tau)` still being 2 %.
    // 八个时间常数，好让下面那个 0.5 % 的界关于模型本身，而不是关于 `1 − exp(−t/τ)` 还差 2 %。
    for (int n = 0; n < 640000; ++n) { // 32 s = 8 tau
        plant.advance(dt);
        elapsed += dt;
        if (at_tau == 0.0F && elapsed >= 4.0F) {
            at_tau = plant.winding_temperature_c();
        }
    }

    // The loss the plant actually had, so the criterion does not depend on my arithmetic being right about the current.
    // plant 实际承受的损耗，好让判据不依赖我对电流的算术是对的。
    const float measured_loss = plant.copper_loss_w();
    EXPECT_NEAR(expected_loss, measured_loss, expected_loss * 0.01F) << "measured loss " << measured_loss;
    EXPECT_NEAR(20.0F + expected_rise, plant.winding_temperature_c(), expected_rise * 0.005F)
        << "settle " << plant.winding_temperature_c() << " expected " << (20.0F + expected_rise);

    // `1 - 1/e` of the way there after exactly one `Rt*Ct`, which is what "the step is the exact solution" buys.
    // 恰好一个 `Rt·Ct` 之后走完 `1 − 1/e`，这就是"步进用的是精确解"买到的东西。
    const float expected_at_tau = 20.0F + (expected_rise * (1.0F - std::exp(-1.0F)));
    EXPECT_NEAR(expected_at_tau, at_tau, expected_rise * 0.02F)
        << "at tau " << at_tau << " expected " << expected_at_tau;
}

// The feedback, in the direction the physics puts it: a voltage-fed winding is **self-limiting** -- it heats, `Rs` rises,
// the same volts draw less current, and the loss falls. So the settle is *below* the no-drift prediction, and it is the
// root of a quadratic rather than a product. A model that treats the loss as a constant would pass the case above and fail
// here by the whole gap between the two numbers.
// 反馈，方向由物理决定：电压供电的绕组是**自限的**——它变热、`Rs` 升高、同样的电压抽出更小的电流、损耗反而下降。所以稳态**低于**
// 不漂时的预测，而且是二次方程的根而不是一个乘积。把损耗当常数处理的模型能通过上面那条，在这里会整整差出两个数之间的那个间隙。
TEST(SilThermal, AVoltageFedWindingIsSelfLimitingAndSettlesOnTheQuadraticRoot) {
    const auto nameplate = foc::sil::nominal_plant_params();
    const float v_d = 3.0F;
    const float rt = 8.0F;
    const float expected = settle_voltage_fed(nameplate.rs, v_d, rt, kAlphaCopper, 20.0F);
    const float no_drift = 20.0F + (rt * 1.5F * v_d * v_d / nameplate.rs);
    ASSERT_TRUE(std::isfinite(expected));
    ASSERT_GT(no_drift, expected + 5.0F) << "the two predictions have to be far apart or this case proves nothing";

    PmsmPlant plant{nameplate};
    foc::sil::PmsmPlant::ThermalConfig thermal{};
    thermal.rt_k_per_w = rt;
    thermal.ct_j_per_k = 0.5F;
    thermal.t_ambient_c = 20.0F;
    plant.set_thermal(thermal);
    plant.set_winding_temperature(20.0F, kAlphaCopper);
    plant.set_voltage_dq(foc::core::Dq<float>{v_d, 0.0F});
    for (int n = 0; n < 600000; ++n) { // 30 s, past seven time constants 30 s，超过七个时间常数
        plant.advance(5.0e-5F);
    }
    EXPECT_NEAR(expected, plant.winding_temperature_c(), expected * 0.02F)
        << "settle " << plant.winding_temperature_c() << " quadratic root " << expected << " no-drift " << no_drift;
    // And `Rs` really moved with it -- otherwise the case would be measuring a temperature nobody uses.
    // 而且 `Rs` 确实跟着动了——否则这条量的是一个没人用的温度。
    EXPECT_GT(plant.effective_rs(), nameplate.rs * 1.05F) << "rs " << plant.effective_rs();
}

// The other direction, and the one that has engineering consequences: hold the **current** and the loss grows with
// temperature, so the steady state is a division by `1 - Rt*P*alpha`. Below the boundary it settles; above it the
// denominator changes sign and there is no steady state at all. Both sides are measured with the same loss and only the
// cooling changed, so the boundary is found rather than asserted -- and the same machine is stable or not depending on how
// it is cooled.
// 另一个方向，也是后果更严重的那个：把**电流**握住，损耗就随温度增长，于是稳态是一个除以 `1 − Rt·P·α` 的结果。边界以下它收敛，
// 边界以上分母变号、**根本不存在稳态**。两侧用同一个损耗、只改散热，所以边界是被量出来的而不是被断言的——同一台机器是否稳
// 定，取决于它怎么被冷却。
TEST(SilThermal, ACurrentFedWindingSettlesBelowRtTimesPLossTimesAlphaAndRunsAwayAboveIt) {
    const auto nameplate = foc::sil::nominal_plant_params();
    const float current_a = 3.0F;
    const float p20 = 1.5F * current_a * current_a * nameplate.rs;
    const float boundary_rt = 1.0F / (p20 * kAlphaCopper); // `Rt*P20*alpha = 1` 失控边界
    const float cooled_rt = boundary_rt * 0.2F;
    const float starved_rt = boundary_rt * 1.2F;

    // The harness plays the current loop: `v_d = i*Rs(T)` every step, so the current -- and therefore the physics that
    // makes this non-linear -- is held while the temperature moves. One line, and it is exactly what a current-regulated
    // drive does; leaving it out would test a voltage-fed machine instead.
    // 夹具扮演电流环：每步 `v_d = i·Rs(T)`，于是电流——以及让它非线性的那份物理——被握住，而温度在动。就一行，而它正是电流
    // 调节驱动器所做的事；不写它就变成在测电压供电的机器。
    auto run = [&](float rt) -> std::pair<float, float> {
        PmsmPlant plant{nameplate};
        foc::sil::PmsmPlant::ThermalConfig thermal{};
        thermal.rt_k_per_w = rt;
        thermal.ct_j_per_k = 0.2F;
        thermal.t_ambient_c = 20.0F;
        plant.set_thermal(thermal);
        plant.set_winding_temperature(20.0F, kAlphaCopper);
        float early = 0.0F;
        float late = 0.0F;
        for (int n = 0; n < 400000; ++n) { // 20 s
            plant.set_voltage_dq(foc::core::Dq<float>{current_a * plant.effective_rs(), 0.0F});
            plant.advance(5.0e-5F);
            if (n == 300000) { // 15 s, ten thermal time constants 15 s，十个热时间常数
                early = plant.winding_temperature_c();
            }
            late = plant.winding_temperature_c();
        }
        return {early, late};
    };

    const auto [early_cooled, late_cooled] = run(cooled_rt);
    const float expected = settle_current_fed(nameplate.rs, current_a, cooled_rt, kAlphaCopper, 20.0F);
    ASSERT_TRUE(std::isfinite(expected));
    EXPECT_NEAR(expected, late_cooled, expected * 0.02F) << "settle " << late_cooled << " closed form " << expected;
    // A settle, not a drift: the last 5 s moved it by less than a tenth of the settle, and both points are past 8 tau.
    // 是收敛不是漂移：最后 5 s 移动的量不到稳态值的十分之一，而且两个点都过了 8 个 τ。
    EXPECT_LT(std::fabs(late_cooled - early_cooled), expected * 0.10F)
        << "early " << early_cooled << " late " << late_cooled;

    // Above the boundary the same machine has no steady state. The closed form says so in its own way -- the denominator
    // `1 - Rt*P20*alpha` has changed sign, so the only root it can offer is a negative temperature -- and the simulation
    // has to agree by not settling: still climbing after 20 s, and climbing *faster* than the cooled machine was.
    // 边界以上同一台机器没有稳态。闭式解用它自己的方式说明了这件事——分母 `1 − Rt·P20·α` 变了号，于是它能给出的唯一根是一个负温
    // 度——而仿真必须以"不收敛"来同意：20 s 之后还在爬，而且比那台冷得好的机器爬得**更快**。
    const auto [early_starved, late_starved] = run(starved_rt);
    const float starved_root = settle_current_fed(nameplate.rs, current_a, starved_rt, kAlphaCopper, 20.0F);
    EXPECT_GT(starved_rt * p20 * kAlphaCopper, 1.0F) << "the starved case must be past the boundary";
    EXPECT_LT(starved_root, 0.0F) << "the closed form must report no physical root, got " << starved_root;
    // Still climbing after 20 s -- more than five times what the cooled machine gained over the same window -- while being
    // far above the temperature the cooled one settled at. Both halves are needed: "high" alone is satisfied by a wrong
    // gain, and "climbing" alone by anything with a long time constant.
    // 20 s 之后还在爬——同一窗口内涨的是那台冷得好的机器的五倍以上——同时远高于对方收敛到的温度。两半都要：「高」单独可以被一个
    // 错的增益满足，「在爬」单独可以被任何时间常数很长的东西满足。
    EXPECT_GT(late_starved - early_starved, std::fmax(late_cooled - early_cooled, 1.0F) * 5.0F)
        << "starved " << early_starved << " -> " << late_starved << " cooled " << early_cooled << " -> " << late_cooled;
    EXPECT_GT(late_starved, late_cooled * 3.0F) << "starved " << late_starved << " cooled settle " << late_cooled;
}

// Off by default, and off means *exactly* off: the same trajectory bit for bit. This is the criterion that keeps the port
// from moving every number in the repository that was measured through this plant before it existed.
// 默认关闭，而且关闭意味着**精确**关闭：同一条轨迹逐位相同。正是这条判据让本端口不影响它出现之前经这台 plant 量出的每一个数。
TEST(SilThermal, OffIsBitIdenticalToThePlantWithoutThePort) {
    const auto nameplate = foc::sil::nominal_plant_params();
    auto run = [&](bool with_port) -> PmsmPlant {
        PmsmPlant plant{nameplate};
        if (with_port) {
            foc::sil::PmsmPlant::ThermalConfig thermal{};
            thermal.rt_k_per_w =
                0.0F; // rt <= 0 means off, even though a config was handed over 给了配置但 rt<=0 就是关
            thermal.ct_j_per_k = 0.5F;
            plant.set_thermal(thermal);
        }
        plant.set_winding_temperature(40.0F, kAlphaCopper);
        plant.set_voltage_dq(foc::core::Dq<float>{1.0F, 0.5F});
        for (int n = 0; n < 4000; ++n) {
            plant.advance(5.0e-5F);
        }
        return plant;
    };
    const PmsmPlant plain = run(false);
    const PmsmPlant ported = run(true);
    EXPECT_EQ(plain.state().id_a, ported.state().id_a);
    EXPECT_EQ(plain.state().iq_a, ported.state().iq_a);
    EXPECT_EQ(plain.state().omega_mech, ported.state().omega_mech);
    EXPECT_EQ(plain.state().theta_mech, ported.state().theta_mech);
    EXPECT_EQ(plain.effective_rs(), ported.effective_rs());
    EXPECT_EQ(plain.winding_temperature_c(), ported.winding_temperature_c());
    EXPECT_EQ(0.0F, ported.thermal_time_constant_s());
}

// --- the chain: the temperature this port produces is the input `Protector` derates against ---------------------------
//
// F36's criterion in the plan is "the derating curve matches 3.00 A @ 45 C / 1.80 A @ 80 C / 0 A @ 105 C point by point".
// **I first read that as a contradiction and it is not one** -- my first probe used thresholds of my own invention (a 45->105
// ramp) and got 1.25 A at 80 C, which "proved" the three points were not collinear. Measured against the configuration this
// repository actually ships (`test_protection.cpp`: `i_max` 3 A, ramp 60 -> 110 C, trip 105 C) all three hold, and the
// reason the third one holds is the interesting part: at 45 C the curve is still flat (the knee is 60 C, not 45), at 80 C it
// reads exactly 1.80 A, and the **0 A at 105 C is the trip, not the curve** -- `step()` reports `OverTemperature`,
// `output_allowed = false` and `i_limit_a = 0`, while `derated_limit(105)` on its own still reads 0.30 A. Two quantities,
// and a derating scheduler has to know which one it is looking at.
// --- 链条：本端口产出的温度，正是 `Protector` 降额时读的那个数 -----------------------------------------
//
// 计划里 F36 的判据是"降额曲线与 3.00 A@45 ℃ / 1.80 A@80 ℃ / 0 A@105 ℃ 逐点一致"。**我先把它读成了矛盾，而它不是。** 我第一版
// 探针用了自己发明的阈值（45→105 的斜坡），在 80 ℃ 处读到 1.25 A，"证明"了三点不共线。对着本仓库**实际出货**的配置
// （`test_protection.cpp`：`i_max` 3 A、斜坡 60→110 ℃、105 ℃ 跳闸）量，三点全部成立，而第三条成立的理由才是有意思的地方：
// 45 ℃ 处曲线还是平的（拐点是 60 ℃ 而不是 45），80 ℃ 处正好读到 1.80 A，而 **105 ℃ 的那个 0 A 是跳闸、不是曲线**——
// `step()` 报 `OverTemperature`、`output_allowed = false`、`i_limit_a = 0`，而单看 `derated_limit(105)` 仍是 0.30 A。
// 两个不同的量，而做降额调度的人必须知道自己看的是哪一个。
TEST(SilThermal, TheTrajectoryReachesTheDeratingCurveTheProtectorActuallyHas) {
    using foc::core::ProtectionConfig;
    using foc::core::ProtectionInput;
    using foc::core::ProtectionOutput;
    using foc::core::Protector;

    // The shipped configuration, copied from `test_protection.cpp` rather than invented here.
    // 出货配置，抄自 `test_protection.cpp`，不是在这里发明的。
    ProtectionConfig limits{};
    limits.i_hard_a = 4.0F;
    limits.i_max_a = 3.0F;
    limits.i2t_a2s = 4.5F;
    limits.i2t_cooling_s = 0.5F;
    limits.v_max_v = 48.0F;
    limits.v_min_v = 12.0F;
    limits.t_trip_c = 105.0F;
    limits.t_derate_start_c = 60.0F;
    limits.t_derate_end_c = 110.0F;
    Protector protector{limits};

    // The three points the README row claims, each measured, plus the distinction that makes the third one true.
    // README 行声称的三点，逐点量，外加让第三点成立的那个区分。
    EXPECT_FLOAT_EQ(3.00F, protector.derated_limit(45.0F)); // below the knee, not at it 在拐点以下，不是拐点
    EXPECT_FLOAT_EQ(1.80F, protector.derated_limit(80.0F));
    EXPECT_NEAR(0.30F, protector.derated_limit(105.0F), 1.0e-6F); // the curve, still non-zero 曲线，仍非零
    // The third number needs one more distinction than the row makes. `derated_limit(105)` is 0.30 A and a **fresh**
    // protector still allows output at exactly 105.000 C, because the trip test is a strict `> t_trip_c`; the row's
    // "0 A and no output at 105 C" is true just above it, and true at 105 C once the trip has latched. Both are asserted
    // here rather than argued, and the boundary is the interesting part: one comparison decides whether a winding at its
    // own trip temperature is allowed to keep pushing.
    // 第三个数字需要比那一行多一个区分。`derated_limit(105)` 是 0.30 A，而**新建的**保护器在恰好 105.000 ℃ 处仍然允许输出，
    // 因为跳闸判定是严格的 `> t_trip_c`；行里那句"105 ℃ 时 0 A 且无输出"在略高于它时成立，在**已跳闸**之后 105 ℃ 处也成立。
    // 两者都在这里被断言而不是被论证，而边界本身才是有意思的部分：一个比较符决定了处在自身跳闸温度上的绕组还能不能继续使劲。
    ProtectionInput hot{};
    hot.i_magnitude_a = 0.5F;
    hot.v_bus_v = 24.0F;
    hot.omega_e_rad_s = 100.0F;
    hot.flux_error_ratio = 0.0F;
    hot.flux_valid = true;
    hot.temperature_c = 105.0F;
    Protector fresh{limits};
    const ProtectionOutput at_trip = fresh.step(hot, 5.0e-5F);
    EXPECT_TRUE(at_trip.output_allowed) << "the trip is strictly above t_trip_c, so 105.000 C still outputs";
    EXPECT_NEAR(0.30F, at_trip.i_limit_a, 1.0e-6F);
    hot.temperature_c = 106.0F;
    const ProtectionOutput above = fresh.step(hot, 5.0e-5F);
    EXPECT_FALSE(above.output_allowed);
    EXPECT_FLOAT_EQ(0.0F, above.i_limit_a);
    EXPECT_EQ(foc::core::FaultCode::OverTemperature, above.fault);
    hot.temperature_c = 105.0F; // back to the trip temperature: the latch holds 回到跳闸温度：锁存不松
    const ProtectionOutput latched = fresh.step(hot, 5.0e-5F);
    EXPECT_FALSE(latched.output_allowed) << "the row's '0 A and no output at 105 C' is the latched state";
    EXPECT_FLOAT_EQ(0.0F, latched.i_limit_a);

    // Monotone and flat below the knee, so "hotter is strictly less authority" holds where it should.
    // 单调且在拐点以下平直，于是"更热一定更没权限"在它该成立的地方成立。
    EXPECT_FLOAT_EQ(3.00F, protector.derated_limit(20.0F));
    float previous = protector.derated_limit(45.0F);
    for (float t = 46.0F; t <= 110.0F; t += 1.0F) {
        const float here = protector.derated_limit(t);
        EXPECT_LE(here, previous) << "T " << t;
        previous = here;
    }

    // Now the chain itself: run the machine to a thermal settle inside the derating band and hand the temperature over.
    // 然后是链条本身：把机器跑到落在降额带内的热稳态，再把温度交过去。
    const auto nameplate = foc::sil::nominal_plant_params();
    const float current_a = 3.0F;
    const float p20 = 1.5F * current_a * current_a * nameplate.rs;
    const float boundary_rt = 1.0F / (p20 * kAlphaCopper);
    PmsmPlant plant{nameplate};
    foc::sil::PmsmPlant::ThermalConfig thermal{};
    thermal.rt_k_per_w = boundary_rt * 0.2F;
    thermal.ct_j_per_k = 0.2F;
    thermal.t_ambient_c = 20.0F;
    plant.set_thermal(thermal);
    plant.set_winding_temperature(20.0F, kAlphaCopper);
    for (int n = 0; n < 400000; ++n) { // 20 s
        plant.set_voltage_dq(foc::core::Dq<float>{current_a * plant.effective_rs(), 0.0F});
        plant.advance(5.0e-5F);
    }
    const float settled_c = plant.winding_temperature_c();
    const float allowed_a = protector.derated_limit(settled_c);
    RecordProperty("settled_c", settled_c);
    RecordProperty("allowed_at_settled_a", allowed_a);
    ASSERT_GT(settled_c, limits.t_derate_start_c) << "the settle has to land inside the band to test the chain";
    ASSERT_LT(settled_c, limits.t_trip_c) << "and below the trip, or the curve is not what limits the drive";

    // The chain bites: at the temperature the winding actually reaches, the drive is allowed **less** than the current that
    // heated it there. That is the whole reason a scheduler exists, and it is a number rather than a worry.
    // 链条确实咬住了：在绕组实际达到的温度上，驱动器被允许的电流**小于**把它加热到那里的那个电流。这正是调度存在的全部理由，
    // 而且它是一个数，不是一个担忧。
    EXPECT_LT(allowed_a, current_a) << "allowed " << allowed_a << " A at " << settled_c << " C against " << current_a
                                    << " A";
    EXPECT_NEAR(3.0F * (1.0F - ((settled_c - 60.0F) / 50.0F)), allowed_a, 1.0e-4F) << "settled_c " << settled_c;
}
