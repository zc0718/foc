// Conan::ImportStart
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/protection.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/motor/drive_supervisor.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>
// Conan::ImportEnd

namespace {

// The drive's tick order, checked where order is the only thing that can be wrong.
//
// Each of these pieces is correct on its own and useless in the wrong sequence, which is why the cases here are about
// *when* rather than *whether*: the chopper keeps dissipating after the output is off, the bus voltage needs no second
// tick to be acted on, and an over-current stops the output on the first tick it is visible rather than on the tick
// after the machine has already been switching into it.
//
// 驱动器的 tick 顺序，在最容易错的"顺序"本身上检查。
//
// 这几件单独看都对、放错先后就都没用，所以这里的用例讲的是"何时"而不是"是否"：斩波器在输出关掉之后继续耗能、
// 母线电压不需要第二拍才被处置、过流在它第一次可见的那一拍就停下输出，而不是在机器已经朝它开关了一拍之后。
//
// The one thing that is deliberately *not* claimed: this type does not enforce the derated limit. A hot machine is not a
// faulty one, so the limit is published for the caller's `Command::max_current_a` and the case below shows the curve
// falling while `fault` stays `None` -- a number that reaches the controller, not a command that gets refused.
// 也有一条刻意 **不** 声称的：本类型不执行降额后的限值。热的机器不是故障的机器，所以限值发布给调用方的
// `Command::max_current_a`，下面的用例展示曲线下降而 `fault` 保持 `None`——一个送到控制器的数，而不是被拒绝的指令。

using foc::ErrorCode;
using foc::core::FaultCode;
using foc::core::ProtectionConfig;

constexpr float kDt = 50e-6F;
constexpr float kVBus = 24.0F;

using Hal = foc::driver::StubHal;

auto motor_config() -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U, 3.0F, 20000.0F, 0.0F};
}

auto current_pi_config() -> foc::core::Pi::Config {
    return foc::core::Pi::Config{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
}

using MotorType = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, Hal>;
using Supervisor = foc::motor::DriveSupervisor<MotorType, Hal>;

auto policies() -> MotorType::Policies {
    return MotorType::Policies{
        foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{}, foc::core::Pi{current_pi_config()},
        foc::core::Pi{current_pi_config()},
        foc::core::Pi{foc::core::Pi::Config{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F}}};
}

// A protection band wide enough that only the thing under test can trip it.
auto wide_protection() -> ProtectionConfig {
    ProtectionConfig p{};
    p.i_hard_a = 100.0F;
    p.i_max_a = 10.0F;
    p.i2t_a2s = 1000.0F;
    p.i2t_cooling_s = 10.0F;
    p.v_max_v = 60.0F;
    p.v_min_v = 10.0F;
    p.t_trip_c = 150.0F;
    p.t_derate_start_c = 60.0F;
    p.t_derate_end_c = 120.0F;
    p.stall_speed_rad_s = 1.0F;
    p.stall_current_a = 50.0F;
    p.stall_time_s = 10.0F;
    p.sync_error_ratio = 1.0F;
    p.hysteresis_ratio = 0.05F;
    return p;
}

auto chopper_config() -> foc::core::BrakeChopperConfig {
    foc::core::BrakeChopperConfig c{};
    c.v_on = 30.0F;
    c.v_off = 28.0F;
    c.r_ohm = 10.0F;
    c.duty_max = 0.9F;
    // Zero dwell on purpose: the chopper's minimum off-time is a real property, and leaving it at a millisecond would
    // make these cases measure the dwell instead of the order they are about. The dwell has its own cases.
    // 驻留时间故意取零：斩波器的最短关断时间是真实性质，把它留在一毫秒会让这些用例去量驻留而不是它们要讲的顺序。驻留时间
    // 有它自己的用例。
    c.min_on_s = 0.0F;
    c.min_off_s = 0.0F;
    c.i2t_limit_a2s = 5.0F;
    c.i2t_cool_tau_s = 1.0F;
    return c;
}

auto config(const ProtectionConfig& protection = wide_protection()) -> Supervisor::Config {
    Supervisor::Config c{};
    c.protection = protection;
    c.chopper = chopper_config();
    c.pole_pairs = 7.0F;
    return c;
}

// One drive on the stub hardware: the HAL, the motor, and the supervisor bound to both.
struct Drive final {
    Hal hal{};
    MotorType motor{motor_config(), policies(), hal};
    Supervisor supervisor{motor, hal};

    Drive() {
        static_cast<void>(hal.pwm.configure(20000.0F, 0.0F));
        hal.position.set_sensor_present(true);
        hal.position.set_angle_mech_rad(0.0F);
        hal.position.set_velocity_rad_s(0.0F);
        hal.dc_link.set_voltage_v(kVBus);
        load_current(0.0F);
    }

    void load_current(float amps) {
        const foc::core::Uvw<float> sample{amps, -0.5F * amps, -0.5F * amps};
        static_cast<void>(hal.current_sense.load(&sample, 1U));
    }

    auto bring_up(const Supervisor::Config& c = config()) -> void {
        ASSERT_EQ(ErrorCode::Ok, motor.init());
        ASSERT_EQ(ErrorCode::Ok, supervisor.configure(c));
        ASSERT_EQ(ErrorCode::Ok, motor.start());
        ASSERT_TRUE(hal.pwm.enabled());
    }
};

// 1. A band that cannot be a band is refused before a single tick runs, and a refusal leaves nothing half-stored.
TEST(DriveSupervisor, AConfigurationThatCannotBeABandIsRefusedBeforeAnyTick) {
    struct Case final {
        const char* what;
        void (*bend)(Supervisor::Config&);
    };
    const Case cases[] = {
        {"a trip point below its own derating end",
         [](Supervisor::Config& c) { c.protection.t_trip_c = c.protection.t_derate_end_c - 1.0F; }},
        {"a derating end below its own start",
         [](Supervisor::Config& c) { c.protection.t_derate_end_c = c.protection.t_derate_start_c - 1.0F; }},
        {"a bus window the wrong way round", [](Supervisor::Config& c) { c.protection.v_max_v = c.protection.v_min_v; }},
        {"no hard current limit", [](Supervisor::Config& c) { c.protection.i_hard_a = 0.0F; }},
        {"no thermal budget", [](Supervisor::Config& c) { c.protection.i2t_a2s = 0.0F; }},
        {"a chopper that would switch on below where it switches off",
         [](Supervisor::Config& c) { c.chopper.v_on = c.chopper.v_off; }},
        {"no pole pairs", [](Supervisor::Config& c) { c.pole_pairs = 0.0F; }},
    };
    for (const Case& one : cases) {
        Drive drive;
        Supervisor::Config c = config();
        one.bend(c);
        EXPECT_EQ(ErrorCode::Param, drive.supervisor.configure(c)) << one.what;
        // Nothing was stored, so the step still refuses for the original reason rather than running on a half-applied
        // band.
        // 什么都没存下来，所以步进仍然因为最初那个原因拒绝，而不是在半套区间上跑起来。
        EXPECT_EQ(ErrorCode::State, drive.supervisor.step(kDt)) << one.what;
        EXPECT_EQ(0U, drive.hal.pwm.write_count()) << one.what;
    }
}

// 2. Unconfigured is a refusal, not a default band. This is the failure the whole type exists to prevent.
TEST(DriveSupervisor, AnUnconfiguredSupervisorRefusesToStepRatherThanRunUnprotected) {
    Drive drive;
    ASSERT_EQ(ErrorCode::Ok, drive.motor.init());
    ASSERT_EQ(ErrorCode::Ok, drive.motor.start());
    EXPECT_EQ(ErrorCode::State, drive.supervisor.step(kDt));
    EXPECT_EQ(0U, drive.hal.pwm.write_count()) << "nothing may be modulated by an unprotected drive";
    EXPECT_EQ(0U, drive.supervisor.status().tick) << "a refused step is not a step";
}

// 3. An over-current stops the output on the first tick it is visible -- and without the supervisor nothing stops it at
//    all. The one-tick latency is a property of using the loop's own conversion, stated on the type; what is asserted
//    here is that the stop costs no extra tick once the current is in hand.
TEST(DriveSupervisor, AnOverCurrentStopsTheOutputOnTheFirstTickItIsVisible) {
    Drive drive;
    ProtectionConfig p = wide_protection();
    p.i_hard_a = 5.0F;
    drive.bring_up(config(p));
    drive.load_current(6.0F);

    // Tick 1: the protector's input is the loop's *previous* conversion, which is still zero. The loop runs.
    EXPECT_EQ(ErrorCode::Ok, drive.supervisor.step(kDt));
    EXPECT_EQ(1U, drive.hal.pwm.write_count()) << "the first tick has nothing to act on yet";
    EXPECT_TRUE(drive.hal.pwm.enabled());

    // Tick 2: the current is now in hand, so this tick does not switch.
    const std::uint32_t writes_before = drive.hal.pwm.write_count();
    EXPECT_EQ(ErrorCode::Fault, drive.supervisor.step(kDt));
    EXPECT_EQ(writes_before, drive.hal.pwm.write_count()) << "a tick that trips must not also modulate";
    EXPECT_FALSE(drive.hal.pwm.enabled()) << "PWM off, and before the state says anything";
    EXPECT_EQ(foc::core::MotorState::Fault, drive.motor.state());
    EXPECT_EQ(FaultCode::OverCurrent, drive.motor.fault_code()) << "the operator gets a cause, not just a trip";
    EXPECT_EQ(FaultCode::OverCurrent, drive.supervisor.status().fault);
    EXPECT_FALSE(drive.supervisor.status().output_allowed);
    EXPECT_EQ(0.0F, drive.supervisor.status().i_limit_a) << "a tripped step has no authority at all";

    // The counterfactual, same fixture and same current, with the supervisor left out of the path: the loop keeps
    // switching into 6 A with a 5 A hard limit, for as long as a case is willing to run it.
    // 反事实：同一个夹具、同一股电流，只是把监督器从路径里拿掉——控制环继续朝 6 A 开关，想跑多久跑多久。
    Drive bare;
    ASSERT_EQ(ErrorCode::Ok, bare.motor.init());
    ASSERT_EQ(ErrorCode::Ok, bare.motor.start());
    bare.load_current(6.0F);
    for (int tick = 0; tick < 5; ++tick) {
        ASSERT_EQ(ErrorCode::Ok, bare.motor.tick(kDt));
    }
    EXPECT_EQ(5U, bare.hal.pwm.write_count()) << "five ticks unprotected is five ticks of switching";
    EXPECT_EQ(foc::core::MotorState::Run, bare.motor.state());
    EXPECT_EQ(FaultCode::None, bare.motor.fault_code()) << "and nobody would ever have been told";
}

// 4. The bus is read on this tick, so an over-voltage costs no latency at all.
TEST(DriveSupervisor, AnOverVoltageIsActedOnInTheTickItAppears) {
    Drive drive;
    ProtectionConfig p = wide_protection();
    p.v_max_v = 26.0F;
    p.v_min_v = 10.0F;
    drive.bring_up(config(p));
    drive.hal.dc_link.set_voltage_v(30.0F);
    drive.load_current(0.1F);

    EXPECT_EQ(ErrorCode::Fault, drive.supervisor.step(kDt)) << "no second tick for a slow signal";
    EXPECT_EQ(0U, drive.hal.pwm.write_count()) << "the first tick already refused to switch";
    EXPECT_EQ(FaultCode::OverVoltage, drive.motor.fault_code());
    EXPECT_EQ(30.0F, drive.supervisor.status().v_bus_v);
}

// 5. The three remaining causes nobody else in the library can see, each reaching `fault_code()` by name.
TEST(DriveSupervisor, UnderVoltageOverTemperatureAndTheThermalBudgetEachNameTheirCause) {
    {
        Drive drive;
        ProtectionConfig p = wide_protection();
        p.v_min_v = 20.0F;
        drive.bring_up(config(p));
        drive.hal.dc_link.set_voltage_v(18.0F);
        EXPECT_EQ(ErrorCode::Fault, drive.supervisor.step(kDt));
        EXPECT_EQ(FaultCode::UnderVoltage, drive.motor.fault_code());
    }
    {
        Drive drive;
        drive.bring_up();
        drive.hal.temperature.set_celsius(160.0F);
        EXPECT_EQ(ErrorCode::Fault, drive.supervisor.step(kDt));
        EXPECT_EQ(FaultCode::OverTemperature, drive.motor.fault_code());
        EXPECT_EQ(160.0F, drive.supervisor.status().temperature_c);
    }
    {
        // The budget is spent over time rather than crossed at once: that is the mechanism, so the case has to spend it.
        // 预算是随时间花掉的，不是一步越过的：那正是这个机理，所以用例必须把它花掉。
        Drive drive;
        ProtectionConfig p = wide_protection();
        p.i_max_a = 10.0F;
        p.i2t_a2s = 0.002F;
        p.i2t_cooling_s = 1000.0F;
        drive.bring_up(config(p));
        drive.load_current(6.0F);
        ErrorCode rc = ErrorCode::Ok;
        int steps = 0;
        while ((rc == ErrorCode::Ok) && (steps < 10)) {
            rc = drive.supervisor.step(kDt);
            ++steps;
        }
        EXPECT_EQ(ErrorCode::Fault, rc) << "the budget has to run out of its own accord";
        EXPECT_EQ(FaultCode::I2t, drive.motor.fault_code());
        EXPECT_GT(drive.supervisor.protector().thermal_budget(), p.i2t_a2s)
            << "the budget that tripped is a number, not a flag";
    }
}

// 6. The chopper acts on the bus, not on the machine: it must keep dissipating while the motor is in Fault.
TEST(DriveSupervisor, TheChopperKeepsDissipatingAfterTheOutputIsOff) {
    Drive drive;
    ProtectionConfig p = wide_protection();
    p.v_max_v = 60.0F;
    p.v_min_v = 10.0F;
    p.i_hard_a = 5.0F;
    drive.bring_up(config(p));
    drive.load_current(6.0F);
    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.step(kDt));
    ASSERT_EQ(ErrorCode::Fault, drive.supervisor.step(kDt));
    ASSERT_EQ(foc::core::MotorState::Fault, drive.motor.state());

    // A coasting load pushes the bus over the chopper's on-threshold while the inverter is off.
    // 滑行负载把母线推过斩波器的开通阈值，而逆变器是关着的。
    drive.hal.dc_link.set_voltage_v(34.0F);
    const std::uint32_t writes = drive.hal.pwm.write_count();
    EXPECT_EQ(ErrorCode::Fault, drive.supervisor.step(kDt));
    EXPECT_GT(drive.supervisor.status().chopper_duty, 0.0F) << "a stopped drive still has a bus";
    EXPECT_FALSE(drive.hal.pwm.enabled());
    EXPECT_EQ(writes, drive.hal.pwm.write_count()) << "and it still is not switching the machine";
}

// 7. No temperature channel: no derating, and the status says so instead of implying the machine is cool.
TEST(DriveSupervisor, AMissingTemperatureChannelLeavesTheLimitAloneAndSaysSoInTheStatus) {
    Drive drive;
    drive.bring_up();
    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.step(kDt));
    EXPECT_TRUE(std::isnan(drive.supervisor.status().temperature_c))
        << "0 C and 'nobody measured' are different statements, and the status is where the difference survives";
    EXPECT_FLOAT_EQ(10.0F, drive.supervisor.status().i_limit_a) << "no measurement means no derating";

    // With a channel that answers, the same field carries the number that was used.
    // 有一个会回答的通道时，同一个字段带的是被用到的那个数。
    drive.hal.temperature.set_celsius(90.0F);
    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.step(kDt));
    EXPECT_FLOAT_EQ(90.0F, drive.supervisor.status().temperature_c);
    EXPECT_LT(drive.supervisor.status().i_limit_a, 10.0F);
}

// 8. Derating falls and does not trip: a hot machine keeps a smaller authority, it does not lose the drive.
TEST(DriveSupervisor, TheDeratingCurveFallsMonotonicallyAndNeverTrips) {
    Drive drive;
    drive.bring_up();
    float previous = 10.0F;
    for (const float celsius : {20.0F, 60.0F, 80.0F, 100.0F, 120.0F}) {
        drive.hal.temperature.set_celsius(celsius);
        ASSERT_EQ(ErrorCode::Ok, drive.supervisor.step(kDt)) << celsius;
        EXPECT_EQ(FaultCode::None, drive.supervisor.status().fault) << celsius;
        EXPECT_LE(drive.supervisor.status().i_limit_a, previous) << celsius;
        previous = drive.supervisor.status().i_limit_a;
    }
    EXPECT_EQ(0.0F, previous) << "the band has to end at zero authority, not at a small one";
    EXPECT_TRUE(drive.hal.pwm.enabled()) << "a hot drive that is still switching is the whole point of derating";
}

// 9. Clearing leaves the thermal budget where it was: the winding does not cool because a button was pressed.
TEST(DriveSupervisor, ClearingTheFaultKeepsTheThermalBudget) {
    Drive drive;
    ProtectionConfig p = wide_protection();
    p.i2t_a2s = 0.002F;
    p.i2t_cooling_s = 1000.0F;
    drive.bring_up(config(p));
    drive.load_current(6.0F);
    ErrorCode rc = ErrorCode::Ok;
    int steps = 0;
    while ((rc == ErrorCode::Ok) && (steps < 10)) {
        rc = drive.supervisor.step(kDt);
        ++steps;
    }
    ASSERT_EQ(ErrorCode::Fault, rc);
    const float spent = drive.supervisor.protector().thermal_budget();
    ASSERT_GT(spent, 0.0F);

    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.clear());
    EXPECT_EQ(FaultCode::None, drive.supervisor.status().fault);
    EXPECT_FLOAT_EQ(spent, drive.supervisor.protector().thermal_budget())
        << "the budget is heat, and heat is not cleared by an operator action";
    EXPECT_EQ(foc::core::MotorState::Idle, drive.motor.state());
}

} // namespace
