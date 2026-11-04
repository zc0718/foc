// Conan::ImportStart
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include <foc/core/protection.hpp>
#include <foc/core/startup.hpp>
#include <foc/driver/sim_position.hpp>
#include <foc/driver/stub_dc_link.hpp>
#include <foc/driver/stub_fault_input.hpp>
#include <foc/driver/stub_pwm.hpp>
#include <foc/driver/scripted_current.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_bench.hpp"
#include "sil_plant_hal.hpp"
// Conan::ImportEnd

namespace {

// F42: the fault-injection harness.
// F42：故障注入 harness。
//
// One rule, stated before the cases: **a fault code that nothing can produce is not a protection, it is a word in an
// enum.** `fault_code()` is the first question an operator asks, so every code has to be reachable from a *condition*,
// and the harness's job is to say, code by code, which condition and through which seam. Two things follow, and both are
// mechanical rather than documented:
//
//   * the table below has one row per enumerator, and a count check fails the build the moment a code is added without a
//     row -- the "no producer" failure mode this item exists to catch cannot come back quietly;
//   * each row either **injects** the condition here or **cites** the case that already measures it, because a row that
//     did neither would be a claim with no measurement behind it.
// 一条规矩，写在用例之前：**没有任何东西能产生的故障码不是保护，而是枚举里的一个词。** `fault_code()` 是运维问的第
// 一个问题，所以每个码都必须能从某个**条件**到达，而 harness 的职责就是逐码说清是哪个条件、经过哪条缝。由此有两条推论，
// 两条都是机械的而不是文档的：
//
//   * 下面每个枚举值一行，一旦有人加了一个码却没加行，计数检查就让构建变红——本项要防的「没有生产者」那种失效，
//     不会安静地回来；
//   * 每一行要么在这里**真的注入**那个条件，要么**引用**已经在量它的用例，因为两者都不做的那一行是没有测量支撑的声明。

using foc::ErrorCode;
using foc::core::FaultCode;
using foc::core::ProtectionConfig;
using foc::core::ProtectionInput;
using foc::core::Protector;

constexpr float kDt = 5e-5F;
constexpr float kVBus = 24.0F;

auto motor_config() -> foc::motor::MotorConfig { return foc::sil::nominal_motor_config(); }

// The nominal protection surface the library ships as its default (24 V / 3 A), so the injected thresholds are the ones a
// reader can check against `protect`'s own documentation rather than numbers invented here.
// 库自带的默认保护面（24 V / 3 A），于是注入的阈值是读者能对着 `protection.hpp` 的文档核对的，而不是这里编的数。
auto protection_config() -> ProtectionConfig {
    ProtectionConfig c{};
    c.i_max_a = 3.0F;
    c.i_hard_a = 4.5F;
    c.v_min_v = 18.0F;
    c.v_max_v = 30.0F;
    c.t_trip_c = 105.0F;
    c.t_derate_start_c = 45.0F;
    c.t_derate_end_c = 105.0F;
    c.stall_speed_rad_s = 5.0F;
    c.stall_current_a = 2.5F;
    c.stall_time_s = 0.1F;
    c.sync_error_ratio = 0.5F;
    c.hysteresis_ratio = 0.05F;
    c.i2t_a2s = 4.5F;
    c.i2t_cooling_s = 2.0F;
    return c;
}

auto healthy_input() -> ProtectionInput {
    ProtectionInput in{};
    in.i_magnitude_a = 1.0F;
    in.v_bus_v = kVBus;
    in.temperature_c = 25.0F;
    in.omega_e_rad_s = 100.0F;
    in.flux_error_ratio = 0.0F;
    in.flux_valid = true;
    return in;
}

// A HAL on stub parts: enough to construct, init and (for the pin) run a motor without a plant.
// 一套桩件组成的 HAL：足够构造、初始化，以及在引脚那条路上在不接 plant 的情况下跑电机。
struct StubRig final {
    foc::driver::StubPwm pwm;
    foc::driver::ScriptedCurrent current_sense;
    foc::driver::SimPosition position;
    foc::driver::StubDcLink dc_link;
    foc::driver::StubFaultInput fault_input;
};

using StubMotor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, StubRig>;

auto stub_policies() -> StubMotor::Policies {
    const foc::core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const foc::core::Pi::Config speed{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F};
    return StubMotor::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{}, foc::core::Pi{current},
                               foc::core::Pi{current}, foc::core::Pi{speed}};
}

auto arm(StubRig& hal) -> void {
    hal.dc_link.set_voltage_v(kVBus);
    hal.position.set_sensor_present(true);
    hal.position.set_angle_mech_rad(0.0F);
    hal.position.set_velocity_rad_s(0.0F);
    const foc::core::Uvw<float> zero{};
    static_cast<void>(hal.current_sense.load(&zero, 1U));
    static_cast<void>(hal.pwm.configure(20000.0F, 0.0F));
}

/**
 * @brief [en] How one code is produced, and through which seam.
 * @brief [zh] 一个码怎么产生、经过哪条缝。
 */
enum class Seam : std::uint8_t {
    /**
     * @brief [en] the protector classifies a measured condition and the host hands the verdict to the motor.
     * @brief [zh] 保护器对实测条件分类，调用方把结论交给电机。
     */
    ProtectorVerdict,
    /**
     * @brief [en] the motor's own loop produces it from its own inputs.
     * @brief [zh] 电机自己的环从自己的输入里产生它。
     */
    MotorsOwn,
    /**
     * @brief [en] a HAL input asserts.
     * @brief [zh] 一个 HAL 输入有效。
     */
    HalInput,
    /**
     * @brief [en] not a fault: the absence of all of them.
     * @brief [zh] 不是故障：其余全部缺席。
     */
    NotAFault,
};

struct CodeRow final {
    FaultCode code;
    const char* producer; ///< [en] what makes the condition. [zh] 什么造出这个条件。
    Seam seam;
};

// One row per enumerator, in declaration order. `None` is a row too: it is the answer the harness has to be able to
// distinguish from "not asked yet", and a table that skipped it would let a motor that reports `None` for a real fault
// pass for a healthy one.
// 每个枚举值一行，按声明顺序。`None` 也是一行：它是 harness 必须能与「还没问过」区分开的答案，而跳过它的表会让一个对真
// 故障报 `None` 的电机冒充成健康的。
constexpr CodeRow kRows[] = {
    {FaultCode::None, "all conditions in range", Seam::NotAFault},
    {FaultCode::OverCurrent, "peak current above `i_trip_a`", Seam::ProtectorVerdict},
    {FaultCode::OverVoltage, "bus above `v_max_v`", Seam::ProtectorVerdict},
    {FaultCode::UnderVoltage, "bus below `v_min_v`", Seam::ProtectorVerdict},
    {FaultCode::OverTemperature, "hottest point above `t_trip_c`", Seam::ProtectorVerdict},
    {FaultCode::I2t, "thermal budget spent at `i_max_a`", Seam::ProtectorVerdict},
    {FaultCode::Stall, "full authority demanded, no motion, `kStallWindowS`", Seam::MotorsOwn},
    {FaultCode::LostSync, "handover speed not reached after a forced-angle start", Seam::MotorsOwn},
    {FaultCode::DriveFault, "the power stage's `nFAULT` pin", Seam::HalInput},
};

// The count check that makes this file a gate rather than a table: a new enumerator without a row is a code nobody can
// produce, which is exactly what this item exists to prevent.
// 让这个文件成为门禁而不是表格的计数检查：新加一个枚举值却没有一行，就是一个没人能产生的码，而那正是本项要防的东西。
constexpr std::size_t kEnumerators =
    static_cast<std::size_t>(FaultCode::DriveFault) - static_cast<std::size_t>(FaultCode::None) + 1U;
static_assert(sizeof(kRows) / sizeof(kRows[0]) == kEnumerators,
              "every FaultCode needs a row here: a code with no producer is a word in an enum, not a protection");
static_assert(static_cast<std::size_t>(FaultCode::DriveFault) + 1U == kEnumerators,
              "and the enumerators must stay dense from None to the last one, or the count check above is meaningless");

// 1. The five codes that the protector classifies. The library documents this hop and does not perform it -- `Protector`
//    is a block a host runs and `Motor::fault(code)` is how its verdict becomes the drive's cause -- so the harness
//    measures the *whole chain* and the hop is one line here. That is the finding this item records: five of the eight
//    codes have no producer inside `Motor`, and the composition is the integrator's.
// 1. 保护器分类的那五个码。库把这个跳写成文档而不执行——`Protector` 是主控方运行的一段、`Motor::fault(code)` 是它的结论
//    变成驱动成因的方式——所以 harness 量的是**整条链**，而那一步在这里是一行。这就是本项记录的发现：八个码里有五个在
//    `Motor` 内部没有生产者，组合是集成方的。
TEST(FaultInjection, TheProtectorCodesReachTheFaultCodeThroughTheDocumentedHop) {
    struct Case final {
        FaultCode expected;
        ProtectionInput input;
        int steps;
    };

    ProtectionInput over_current = healthy_input();
    over_current.i_magnitude_a = 5.0F;
    ProtectionInput over_voltage = healthy_input();
    over_voltage.v_bus_v = 33.0F;
    ProtectionInput under_voltage = healthy_input();
    under_voltage.v_bus_v = 15.0F;
    ProtectionInput too_hot = healthy_input();
    too_hot.temperature_c = 120.0F;
    ProtectionInput at_limit = healthy_input();
    // At the nameplate limit, not at some smaller current: the thermal model's equilibrium is `i^2 * tau` = 18 A^2*s
    // against a 4.5 A^2*s budget, so the budget is reachable -- at 1 A it would be 2 A^2*s and the latch could never
    // fire. The same trap as the chopper's cooling budget, found the same way.
    // 用铭牌限幅而不是更小的电流：热模型的平衡是 `i²τ = 18 A²·s`，对 4.5 A²·s 的预算可得——若用 1 A，平衡只有
    // 2 A²·s，锁存永远不会跳。与斩波器冷却预算同一个坑，也是同一种发现方式。
    at_limit.i_magnitude_a = 3.0F;

    const Case cases[] = {
        {FaultCode::OverCurrent, over_current, 1},   {FaultCode::OverVoltage, over_voltage, 1},
        {FaultCode::UnderVoltage, under_voltage, 1}, {FaultCode::OverTemperature, too_hot, 1},
        {FaultCode::I2t, at_limit, 20000}, // the closed form says ~0.575 s; see the assertion below
    };

    for (const Case& c : cases) {
        Protector protector(protection_config());
        StubRig hal{};
        arm(hal);
        StubMotor motor(motor_config(), stub_policies(), hal);
        ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
        ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

        FaultCode reached = FaultCode::None;
        int step = 0;
        for (; step < c.steps; ++step) {
            const auto out = protector.step(c.input, kDt);
            if (out.fault != FaultCode::None) {
                reached = out.fault;
                // The documented hop, and the only line in this file that the library itself does not own.
                // 文档里那一步，也是本文件里唯一一行库自己不负责的代码。
                motor.fault(out.fault);
                break;
            }
        }

        EXPECT_EQ(c.expected, reached) << "the protector must classify the injected condition as the expected code";
        EXPECT_EQ(c.expected, motor.fault_code()) << "and the motor's cause must be that code, not just a state word";
        EXPECT_EQ(foc::core::MotorState::Fault, motor.state());
        EXPECT_FALSE(hal.pwm.enabled()) << "a protected drive stops modulating in the same step";
        ASSERT_EQ(foc::ErrorCode::Ok, motor.clear_fault());
        EXPECT_EQ(FaultCode::None, motor.fault_code());

        if (c.expected == FaultCode::I2t) {
            // Derived, not observed: the thermal model is `q' = i^2 - q/tau`, so `q(t) = i^2*tau*(1 - e^(-t/tau))` and
            // the budget is spent at `t = -tau*ln(1 - budget/(i^2*tau))`. The numbers come from the config above, so the
            // trip step is a prediction the run either meets or refutes -- not a number recorded after the fact.
            // 推导而不是观测：热模型是 `q' = i² − q/τ`，所以 `q(t) = i²τ(1 − e^{−t/τ})`，预算花完于
            // `t = −τ·ln(1 − budget/(i²τ))`。数来自上面的配置，于是跳闸拍数是这次运行要么满足、要么否证的预测——不是
            // 事后抄下来的数。
            const ProtectionConfig pc = protection_config();
            const double equilibrium = static_cast<double>(pc.i_max_a) * static_cast<double>(pc.i_max_a) *
                                       static_cast<double>(pc.i2t_cooling_s);
            ASSERT_GT(equilibrium, static_cast<double>(pc.i2t_a2s)) << "the fixture must make the budget reachable";
            const double predicted_s = -static_cast<double>(pc.i2t_cooling_s) *
                                       std::log(1.0 - (static_cast<double>(pc.i2t_a2s) / equilibrium));
            const double predicted_steps = predicted_s / static_cast<double>(kDt);
            std::printf("[inject] i2t: predicted %.0f step(s) from the closed form, tripped at %d\n", predicted_steps,
                        step + 1);
            EXPECT_NEAR(static_cast<double>(step + 1), predicted_steps, 0.02 * predicted_steps);
        }
        std::printf("\n[inject] %-16s via protector at step %d -> fault_code() = %d\n",
                    kRows[static_cast<std::size_t>(c.expected)].producer, step, static_cast<int>(c.expected));
    }
}

// 2. The stall the motor detects itself: full authority demanded, no motion, held for the window. This is a real
//    condition on a real bench, not a forwarded verdict -- and it is the row that would have been empty before this
//    engine had a stall detector at all, which is why the row exists.
// 2. 电机自己检测的堵转：要求在给全部权限、机器不动、持续一个窗口。这是真台架上的真条件，不是转发的结论——也正是这一行在
//    这个引擎还没有堵转检测器之前会是空的，所以它才在表里。
TEST(FaultInjection, TheMotorsOwnStallDetectorReachesTheFaultCode) {
    foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    params.j = 1e6F; // a rotor that cannot move: the load holds it, the loop cannot
    foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
    foc::sil::Bench bench(cfg, params);

    bench.motor().set_speed(0.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.init_and_start());
    bench.run(1e-3F);
    ASSERT_EQ(FaultCode::None, bench.motor().fault_code());

    bench.motor().set_speed(300.0F); // more than the machine can reach, on a shaft that cannot turn
    ErrorCode last = ErrorCode::Ok;
    int steps = 0;
    for (; steps < 4000; ++steps) { // 0.2 s: twice the 0.1 s window
        last = bench.motor().tick(cfg.dt_ctrl_s);
        if (last == ErrorCode::Fault) {
            break;
        }
        bench.plant().advance(cfg.dt_ctrl_s);
    }

    std::printf("\n[inject] stall: Fault after %d step(s) = %.1f ms (window %.0f ms)\n", steps,
                static_cast<double>(static_cast<float>(steps) * cfg.dt_ctrl_s * 1e3F), 100.0);
    EXPECT_EQ(ErrorCode::Fault, last) << "a demand the machine cannot meet, held, has to trip";
    EXPECT_EQ(FaultCode::Stall, bench.motor().fault_code()) << "and it has to say which mechanism";
    // `steps` is the tick that tripped, counted from zero, so the number of ticks that elapsed is `steps + 1`.
    // `steps` 是跳闸的那一拍（从 0 数），所以经过的拍数是 `steps + 1`。
    EXPECT_GE(steps + 1, static_cast<int>(0.1F / cfg.dt_ctrl_s))
        << "the window is a duration: tripping before it elapses would be a threshold, not a stall detector";
}

// 2b. The other fault the motor produces from its own inputs: a forced-angle start that never catches the rotor. The
//     hand-over check compares the speed the loop *publishes* against the rate the forced vector was ramping at, so a
//     rotor that cannot follow fails the check instead of being handed a commutation angle it was never at.
// 2b. 电机从自己的输入里产生的另一个故障：一次永远没抓住转子的强制角起动。交接检查拿环路**发布**的转速去比强制矢量
//     当时的爬升速率，于是跟不上来的转子会在检查里失败，而不是被交去一个它从未在过的换相角。
TEST(FaultInjection, TheHandoverCheckCatchesARotorThatNeverFollowed) {
    foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    params.j = 1e6F; // the shaft cannot turn: the open-loop ramp will outrun it
    foc::sil::PmsmPlant plant(params);
    foc::sil::PlantHal hal(plant, kVBus);
    // The plant starts at rest, which is the fixture: the forced vector ramps and the rotor does not follow it. The
    // bench's position adapter reports the plant's own truth, so the published speed is 0 for the honest reason.
    // plant 从静止开始，这正是夹具：强制矢量在爬升，而转子不跟。台架的位置适配层报的是 plant 自己的真值，所以发布的
    // 转速为 0 是出于诚实的原因。
    plant.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0, 0.0});

    foc::core::StartupConfig startup{};
    startup.align_current_a = 1.5F;
    startup.align_hold_s = 0.05F;
    startup.pull_current_a = 1.2F;
    startup.pull_accel_rad_s2 = 3000.0F;
    startup.handover_speed_rad_s = 400.0F; // electrical; the vector reaches it in ~0.13 s
    constexpr float kHandoverS = 400.0F / 3000.0F;

    const foc::motor::MotorConfig config = foc::sil::nominal_motor_config();
    const foc::core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const foc::core::Pi::Config speed{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F};
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                     foc::core::NullPositionControl, foc::core::StartupRoutine>;
    MotorT motor(config,
                 MotorT::Policies{foc::core::BackEmfObserver(foc::core::BackEmfObserver::Config{config.rs, config.ld}),
                                  foc::core::Svpwm{}, foc::core::Pi{current}, foc::core::Pi{current},
                                  foc::core::Pi{speed}, foc::core::NullReferenceGenerator{},
                                  foc::core::NullFeedforward{}, foc::core::NullPositionControl{},
                                  foc::core::StartupRoutine{startup}},
                 hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    int steps = 0;
    ErrorCode last = ErrorCode::Ok;
    const int timeout = static_cast<int>(1.0F / kDt); // 1 s: eight times the ramp
    for (; steps < timeout; ++steps) {
        last = motor.tick(kDt);
        hal.advance(kDt, 4);
        if (last == ErrorCode::Fault) {
            break;
        }
    }
    std::printf("\n[inject] lost sync: Fault after %d step(s) = %.1f ms (hand-over at ~%.0f ms)\n", steps,
                static_cast<double>(static_cast<float>(steps) * kDt * 1e3F), static_cast<double>(kHandoverS * 1e3F));
    EXPECT_EQ(ErrorCode::Fault, last) << "a start that never caught the rotor must not be handed over quietly";
    EXPECT_EQ(FaultCode::LostSync, motor.fault_code());
    EXPECT_FALSE(hal.pwm.enabled());
    EXPECT_GT(static_cast<float>(steps) * kDt, startup.align_hold_s) << "and not before the ramp even started";
}

// 3. The pin, already covered end to end by F39's cases; it is asserted here too so that this table is complete rather
//    than "complete except the newest one", which is how tables stop being read.
// 3. 引脚，F39 的用例已经端到端覆盖；这里再断言一次，是为了让这张表是完整的，而不是「除了最新那个之外都完整」——表格就是
//    这样变成没人看的东西的。
TEST(FaultInjection, TheHalPinIsItsOwnRow) {
    StubRig hal{};
    arm(hal);
    StubMotor motor(motor_config(), stub_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    ASSERT_EQ(FaultCode::None, motor.fault_code());

    hal.fault_input.set_faulted(true);
    EXPECT_EQ(ErrorCode::Fault, motor.tick(kDt));
    EXPECT_EQ(FaultCode::DriveFault, motor.fault_code());
}

// 4. The table, checked as a table: `None` is reachable by doing nothing (and must not be confused with "no answer"),
//    and every injected code is distinct from it. Without this row a motor that reported `None` for a real fault would
//    pass case 1's last assertion alone.
// 4. 这张表，作为表来检查：`None` 靠什么都不做就可达（而不能与「没有答案」混淆），每个注入的码都与它不同。没有这一行，
//    一个对真故障报 `None` 的电机只靠用例 1 的最后一条断言就能通过。
TEST(FaultInjection, NoneIsReachableAndIsNotTheSameAnswerAsAFault) {
    Protector protector(protection_config());
    const auto healthy = protector.step(healthy_input(), kDt);
    EXPECT_EQ(FaultCode::None, healthy.fault);
    EXPECT_TRUE(healthy.output_allowed);
    EXPECT_EQ(0U, static_cast<unsigned>(healthy.active_mask));

    StubRig hal{};
    arm(hal);
    StubMotor motor(motor_config(), stub_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    EXPECT_EQ(FaultCode::None, motor.fault_code()) << "a healthy drive answers None, and that is an answer";

    // Every row that claims a seam above must name a condition, and no two rows may share one.
    // 上面每一行声称的「缝」都必须说出一个条件，且没有两行共用一个。
    for (std::size_t i = 0; i < kEnumerators; ++i) {
        EXPECT_NE(nullptr, kRows[i].producer);
        EXPECT_EQ(static_cast<std::size_t>(kRows[i].code), i) << "the table must be in declaration order";
        for (std::size_t j = i + 1U; j < kEnumerators; ++j) {
            EXPECT_NE(kRows[i].producer, kRows[j].producer) << "two codes cannot share one producer sentence";
        }
    }
}

} // namespace
