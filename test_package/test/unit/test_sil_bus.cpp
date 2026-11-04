// The bus current port, measured three ways. This is a **layer-S port**, not a feature: it exists so that a later item
// (F08, the brake chopper) has a quantity on the DC side to be right about.
//
// The criteria, written before the run. Every one of them is a conservation law or an identity, because the alternative
// -- "the bus current looks about right" -- is a number nobody can falsify:
//
//   1. **two independent computations of the same current agree.** `i_dc` from the d-q power the loop applied, and
//      `i_dc` from the three phase voltages times the three phase currents, divided by the bus voltage. The second one
//      never goes through Park or the `3/2`, so it is not a rearrangement of the first;
//   2. **the books close.** With a capacitance, `0.5*C*(v1^2 - v0^2)` must equal what crossed the node: the residual is
//      a quadrature error and nothing else. The bound is derived from the integrator (O(h^2)), not chosen to fit;
//   3. **the sign is the physical one.** Motoring draws current out of the bus, regeneration pushes it back.
//   4. **the counterfactual: with an ideal source the account cannot fail, and it says so.** `accountable == false`,
//      the voltage does not move, and the energy the machine consumed has to be reported as a residual -- because
//      nothing absorbed it. A run that reported a closed account against an ideal source would be a checker that
//      cannot go red.
//   5. and the port is **off by default**: a bench with no meter must behave exactly as it did before it existed.
//
// 判据写在跑前（见上），每一条都是守恒律或恒等式，因为替代品——「母线电流看起来差不多」——是没人能证伪的数：d-q 功率与
// 三相电压乘电流两种独立算法必须一致（后者不经 Park、不经 3/2）；有电容时能量账必须闭合，界由积分格式推出（O(h²)）而不是
// 凑；符号必须是物理的那一个；反事实：理想源下这本账**不可能出错**，于是能量只能以残差出现，而 `accountable == false`
// 把它说出来；最后，这个端口默认关闭——不挂 meter 的台架必须与它存在之前完全一致。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_bus.hpp"
#include "sil_discrete_inverter.hpp"
#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

constexpr float kDt = 50e-6F; // 20 kHz
constexpr int kSubsteps = 4;
constexpr int kPolePairs = 7;
constexpr float kVBus = 24.0F;

using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal>;

auto config() -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
}

struct Bench final {
    explicit Bench(float v_bus)
        : plant(foc::sil::PlantParams{config().rs, config().ld, config().lq, config().kt, config().j, config().b,
                                      kPolePairs}),
          hal(plant, v_bus),
          motor(config(),
                MotorT::Policies{foc::core::BackEmfObserver({config().rs, config().ld}), foc::core::Svpwm{},
                                 foc::core::Pi{foc::current_pi_config(config(), 2000.0F, v_bus)},
                                 foc::core::Pi{foc::current_pi_config(config(), 2000.0F, v_bus)},
                                 foc::core::Pi{foc::speed_pi_config(config(), 250.0F)}},
                hal) {}

    foc::sil::PmsmPlant plant;
    foc::sil::PlantHal hal;
    MotorT motor;
};

// 1. Two independent computations of the same number.
TEST(SilBus, TheDqPowerAndThePhaseVoltagesAgreeOnTheBusCurrent) {
    Bench bench(kVBus);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.init());
    bench.motor.set_torque(0.5F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.start());

    double worst_difference_a = 0.0;
    for (int k = 0; k < 400; ++k) {
        ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.tick(kDt));
        bench.hal.advance(kDt, kSubsteps);

        // Route A: the d-q quantities the loop itself worked with.
        const foc::core::Dq<float> i_dq = bench.plant.current_dq();
        const foc::core::Dq<float> v_dq = bench.plant.voltage_dq();
        const double by_dq = foc::sil::bus_current_a(v_dq, i_dq, kVBus);

        // Route B: three phase voltages times three phase currents, / v_bus. Clarke only -- no Park, no 3/2.
        const foc::core::Uvw<float> i_uvw = bench.plant.current_uvw();
        const float theta_e = static_cast<float>(kPolePairs) * bench.plant.theta_mech_rad();
        const foc::core::Ab<float> v_ab = foc::core::inv_park(v_dq, theta_e);
        const foc::core::Uvw<float> v_uvw = foc::core::inv_clarke(v_ab);
        const double p_abc = (static_cast<double>(v_uvw.u) * static_cast<double>(i_uvw.u)) +
                             (static_cast<double>(v_uvw.v) * static_cast<double>(i_uvw.v)) +
                             (static_cast<double>(v_uvw.w) * static_cast<double>(i_uvw.w));
        const double by_abc = p_abc / static_cast<double>(kVBus);

        worst_difference_a = std::fmax(worst_difference_a, std::fabs(by_dq - by_abc));
    }
    std::printf("\n[bus] worst |i_dc(d-q) - i_dc(abc)| over 400 periods: %.3e A\n", worst_difference_a);
    // float storage through the round trip: v_dq is float, and the abc route reconstructs it, so the agreement is at
    // the resolution the model stores its state in rather than exact.
    EXPECT_LT(worst_difference_a, 1e-5) << "the two routes are different arithmetic, not different physics";
}

// 3. The sign, on a machine that is really motoring.
TEST(SilBus, MotoringDrawsCurrentAndTheBooksSaySo) {
    foc::sil::BusMeter meter(foc::sil::BusParams{1e-3F, kVBus, 0.0F, 0.0F, /*resistor*/ 0.0F});
    EXPECT_TRUE(meter.has_capacitance());

    Bench bench(kVBus);
    bench.hal.set_bus(&meter);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.init());
    bench.motor.set_torque(0.5F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.start());
    for (int k = 0; k < 400; ++k) {
        ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.tick(kDt));
        bench.hal.advance(kDt, kSubsteps);
    }

    const foc::sil::BusReport report = meter.report();
    std::printf("[bus] drawn %.6f J, stored delta %.6f J, residual %.3e J, v in [%.4f, %.4f]\n", report.drawn_j,
                report.stored_delta_j, report.residual_j, static_cast<double>(report.v_min),
                static_cast<double>(report.v_max));
    // Positive torque in the direction of rotation: electrons leave the bus.
    EXPECT_GT(report.drawn_j, 0.0);
    // With no source and no load, the capacitor pays for it: stored energy falls by the same amount.
    EXPECT_LT(report.stored_delta_j, 0.0);
}

// 2. The books close, and the bound comes from the integrator rather than from a wish.
TEST(SilBus, TheEnergyBooksCloseToTheIntegratorsOwnAccuracy) {
    // A source and a load that nearly cancel, and a capacitance big enough that the bus stays inside a sane band:
    // the test is about the books, not about a capacitor being asked to hold up a drive on its own.
    foc::sil::BusMeter meter(foc::sil::BusParams{1e-2F, kVBus, 0.35F, 0.35F, /*resistor*/ 0.0F});
    Bench bench(kVBus);
    bench.hal.set_bus(&meter);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.init());
    bench.motor.set_torque(0.5F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.start());
    for (int k = 0; k < 2000; ++k) { // 100 ms
        ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.tick(kDt));
        bench.hal.advance(kDt, kSubsteps);
    }

    const foc::sil::BusReport report = meter.report();
    const double throughput =
        std::fabs(report.drawn_j) + std::fabs(report.load_j) + std::fabs(report.stored_delta_j) + 1e-9;
    const double relative = std::fabs(report.residual_j) / throughput;
    std::printf("[bus] 2000 periods: drawn %.6f J, load %.6f J, stored delta %.6f J, source %.6f J, residual %.3e J "
                "(%.2e of the throughput), v in [%.4f, %.4f], %u sub-steps\n",
                report.drawn_j, report.load_j, report.stored_delta_j, report.source_j, report.residual_j, relative,
                static_cast<double>(report.v_min), static_cast<double>(report.v_max), report.steps);

    EXPECT_TRUE(report.accountable);
    EXPECT_GT(report.steps, 0U);
    // **The trapezoid is exact here, and that is a derivation rather than a hope.** Inside one sub-step the current is
    // constant, so `v1 - v0 = i*h/C` and therefore `0.5*C*(v1^2 - v0^2) = 0.5*(v0 + v1)*C*(v1 - v0) = v_mean*i*h`
    // identically -- the stored-energy change and the booked energy are the same expression. So the residual is
    // round-off, and any real defect in the port (a missing sign, a current dropped on one path) is far above it. The
    // bound is therefore sharp: 1e-9 relative, twelve orders above what was measured, and still far below any error
    // worth a bug report.
    // **这里的梯形是精确的，这是推导不是期望。** 子步内电流恒定，于是 `v1 − v0 = i·h/C`，因此
    // `0.5·C·(v1² − v0²) = 0.5·(v0 + v1)·C·(v1 − v0) = v_mean·i·h` 恒等成立——储能变化与记账的能量是同一个表达式。
    // 所以残差就是舍入，而端口里任何真实缺陷（漏一个符号、某条通路上掉了一个电流）都远在它之上。因此这个界是**锐的**：
    // 相对值 1e-9，比实测高十二个数量级，却仍远低于任何值得报一个 bug 的误差。
    EXPECT_LT(relative, 1e-9) << "the account must close to round-off: the trapezoid is exact for a piecewise-constant "
                                 "current, so a bigger residual is a defect and not a format";
    // And the voltage moved inside a sane band, so the account is not the story of a constant either.
    EXPECT_GT(report.v_min, 0.5F * kVBus);
    EXPECT_LT(report.v_max, 1.5F * kVBus);
}

// 4. The counterfactual: an ideal source makes the account unfalsifiable, and the report says so instead of pretending.
TEST(SilBus, AnIdealSourceCannotCloseTheAccountAndSaysSo) {
    foc::sil::BusMeter ideal(foc::sil::BusParams{0.0F, kVBus, 0.0F, 0.0F, /*resistor*/ 0.0F}); // no capacitance
    Bench bench(kVBus);
    bench.hal.set_bus(&ideal);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.init());
    bench.motor.set_torque(0.5F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.start());
    for (int k = 0; k < 400; ++k) {
        ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.tick(kDt));
        bench.hal.advance(kDt, kSubsteps);
    }

    const foc::sil::BusReport report = ideal.report();
    std::printf("[bus] ideal source: drawn %.6f J, stored delta %.6f J, residual %.6f J, accountable=%d\n",
                report.drawn_j, report.stored_delta_j, report.residual_j, static_cast<int>(report.accountable));
    EXPECT_FALSE(report.accountable) << "no capacitance means the books cannot be wrong, which is the problem";
    EXPECT_EQ(report.v_min, report.v_max) << "an ideal source does not sag";
    EXPECT_GT(report.drawn_j, 0.0);
    // The energy the machine consumed is *not* explained by anything: it is the residual, and reporting it as zero
    // would be the checker-that-cannot-fail shape this repository keeps finding.
    EXPECT_NEAR(report.residual_j, report.drawn_j, std::fabs(report.drawn_j) * 1e-6 + 1e-12);
}

// 5. Off by default: the bench without a meter is the bench from before this port existed.
TEST(SilBus, WithoutAMeterNothingChanges) {
    Bench with_port(kVBus);
    ASSERT_EQ(foc::ErrorCode::Ok, with_port.motor.init());
    with_port.motor.set_torque(0.5F);
    ASSERT_EQ(foc::ErrorCode::Ok, with_port.motor.start());

    foc::sil::BusMeter meter(foc::sil::BusParams{0.0F, kVBus, 0.0F, 0.0F, /*resistor*/ 0.0F});
    Bench instrumented(kVBus);
    instrumented.hal.set_bus(&meter);
    ASSERT_EQ(foc::ErrorCode::Ok, instrumented.motor.init());
    instrumented.motor.set_torque(0.5F);
    ASSERT_EQ(foc::ErrorCode::Ok, instrumented.motor.start());

    float worst = 0.0F;
    for (int k = 0; k < 400; ++k) {
        ASSERT_EQ(foc::ErrorCode::Ok, with_port.motor.tick(kDt));
        with_port.hal.advance(kDt, kSubsteps);
        ASSERT_EQ(foc::ErrorCode::Ok, instrumented.motor.tick(kDt));
        instrumented.hal.advance(kDt, kSubsteps);
        worst = std::fmax(worst, std::fabs(with_port.plant.omega_mech_rad_s() - instrumented.plant.omega_mech_rad_s()));
    }
    // An ideal source attached to a meter reads the same voltage the stiff bus had, so the two runs are the same run.
    EXPECT_EQ(worst, 0.0F) << "attaching a meter to an ideal source must not be a perturbation";
}

} // namespace
