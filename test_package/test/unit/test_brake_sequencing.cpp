// Conan::ImportStart
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/protection.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/driver/scripted_current.hpp>
#include <foc/driver/sim_position.hpp>
#include <foc/driver/stub_dc_link.hpp>
#include <foc/driver/stub_fault_input.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/driver/stub_mechanical_brake.hpp>
#include <foc/hal/mechanical_brake.hpp>
#include <foc/hal/pwm.hpp>
#include <foc/motor/drive_supervisor.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>
// Conan::ImportEnd

namespace {

// The two orders that keep a vertical load from falling, and the one call each of them must not make.
//
// The two devices live in different objects, so the order between them cannot be observed from either one alone. This
// file therefore writes a HalPolicy whose pulse stage and whose brake both append to **one** log, and every case reads
// that log. A case that only checked "the brake ended up engaged" would pass for a drive that dropped its load on the
// way there.
//
// 让垂直负载不掉下去的两个顺序，以及每一个都不能做的那一次调用。
//
// 两个设备在两个不同的对象里，所以它们之间的先后无法从任何单独一个对象看出来。因此本文件写一个 HalPolicy，让它的功率级与
// 它的抱闸都往 **同一份** 日志里追加，每个用例都读那份日志。只检查"抱闸最后是抱住的"用例，对一个在路上把负载放掉了的
// 驱动器也会通过。

using foc::ErrorCode;
using foc::core::FaultCode;
using foc::core::ProtectionConfig;

constexpr float kDt = 50e-6F;
constexpr float kVBus = 24.0F;

/**
 * @brief One entry in the shared log; the *pair* of fields is what makes the order observable.
 */
struct Event final {
    const char* device;
    const char* action;
};

using Log = std::vector<Event>;

/**
 * @brief A pulse stage that writes to the shared log before doing what `StubPwm` does.
 */
class LoggingPwm final : public foc::hal::Pwm {
  public:
    explicit LoggingPwm(Log& log) noexcept : log_(&log) {}

    auto configure(float freq_hz, float deadtime_ns) noexcept -> ErrorCode override {
        static_cast<void>(freq_hz);
        static_cast<void>(deadtime_ns);
        return ErrorCode::Ok;
    }

    void write_duty(foc::core::Uvw<float> duty) noexcept override {
        last_duty_ = duty;
        ++writes_;
    }

    auto enable() noexcept -> ErrorCode override {
        log_->push_back({"pwm", "enable"});
        enabled_ = true;
        return ErrorCode::Ok;
    }

    void disable() noexcept override {
        log_->push_back({"pwm", "disable"});
        enabled_ = false;
    }

    [[nodiscard]] auto fault() const noexcept -> bool override { return false; }

    [[nodiscard]] auto enabled() const noexcept -> bool { return enabled_; }
    [[nodiscard]] auto writes() const noexcept -> std::uint32_t { return writes_; }

  private:
    Log* log_;
    foc::core::Uvw<float> last_duty_{};
    bool enabled_{false};
    std::uint32_t writes_{0U};
};

/**
 * @brief The same brake as the shipped stub, with the log in front of it.
 */
class LoggingBrake final : public foc::hal::MechanicalBrake {
  public:
    explicit LoggingBrake(Log& log) noexcept : log_(&log) {}

    auto engage() noexcept -> ErrorCode override {
        log_->push_back({"brake", "engage"});
        if (engage_fault_) {
            return ErrorCode::State;
        }
        engaged_ = true;
        return ErrorCode::Ok;
    }

    auto release() noexcept -> ErrorCode override {
        log_->push_back({"brake", "release"});
        if (release_fault_) {
            return ErrorCode::State;
        }
        engaged_ = false;
        return ErrorCode::Ok;
    }

    [[nodiscard]] auto engaged() noexcept -> bool override { return engaged_; }

    void set_engage_fault(bool fault) noexcept { engage_fault_ = fault; }
    void set_release_fault(bool fault) noexcept { release_fault_ = fault; }

  private:
    Log* log_;
    bool engaged_{true};
    bool engage_fault_{false};
    bool release_fault_{false};
};

/**
 * @brief A HalPolicy made of the shipped stubs plus the two logging devices.
 */
struct LoggingHal final {
    using Pwm = LoggingPwm;
    using CurrentSense = foc::driver::ScriptedCurrent;
    using Position = foc::driver::SimPosition;
    using DcLink = foc::driver::StubDcLink;
    using FaultInput = foc::driver::StubFaultInput;

    explicit LoggingHal(Log& log) noexcept : pwm(log), mechanical_brake(log) {}

    Pwm pwm;
    CurrentSense current_sense;
    Position position;
    DcLink dc_link;
    FaultInput fault_input;
    LoggingBrake mechanical_brake;
};

auto motor_config() -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U, 3.0F, 20000.0F, 0.0F};
}

auto current_pi_config() -> foc::core::Pi::Config {
    return foc::core::Pi::Config{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
}

using MotorType = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, LoggingHal>;
using Supervisor = foc::motor::DriveSupervisor<MotorType, LoggingHal>;

template <typename MotorT> auto policies() -> typename MotorT::Policies {
    return typename MotorT::Policies{
        foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{}, foc::core::Pi{current_pi_config()},
        foc::core::Pi{current_pi_config()},
        foc::core::Pi{foc::core::Pi::Config{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F}}};
}

auto protection() -> ProtectionConfig {
    ProtectionConfig p{};
    p.i_hard_a = 5.0F;
    p.i_max_a = 3.0F;
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

template <typename SupervisorT> auto supervisor_config() -> typename SupervisorT::Config {
    typename SupervisorT::Config c{};
    c.protection = protection();
    c.chopper.v_on = 30.0F;
    c.chopper.v_off = 28.0F;
    c.chopper.r_ohm = 10.0F;
    c.chopper.duty_max = 0.9F;
    c.chopper.min_on_s = 0.0F;
    c.chopper.min_off_s = 0.0F;
    c.chopper.i2t_limit_a2s = 5.0F;
    c.chopper.i2t_cool_tau_s = 1.0F;
    c.pole_pairs = 7.0F;
    return c;
}

/**
 * @brief A drive on the logging hardware.
 */
struct Drive final {
    Log log{};
    LoggingHal hal{log};
    MotorType motor{motor_config(), policies<MotorType>(), hal};
    Supervisor supervisor{motor, hal};

    Drive() {
        hal.position.set_sensor_present(true);
        hal.position.set_angle_mech_rad(0.0F);
        hal.position.set_velocity_rad_s(0.0F);
        hal.dc_link.set_voltage_v(kVBus);
        const foc::core::Uvw<float> zero{};
        static_cast<void>(hal.current_sense.load(&zero, 1U));
        static_cast<void>(hal.pwm.configure(20000.0F, 0.0F));
    }

    [[nodiscard]] auto index_of(const char* device, const char* action) const -> int {
        for (std::size_t i = 0U; i < log.size(); ++i) {
            if ((std::string(log[i].device) == device) && (std::string(log[i].action) == action)) {
                return static_cast<int>(i);
            }
        }
        return -1;
    }
};

// 1. Enable: the loop starts first, the brake lets go second -- and the other order is one the type does not take.
TEST(BrakeSequencing, EnablingReleasesTheBrakeOnlyAfterTheLoopIsRunning) {
    Drive drive;
    ASSERT_EQ(ErrorCode::Ok, drive.motor.init());
    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.configure(supervisor_config<Supervisor>()));
    ASSERT_TRUE(drive.hal.mechanical_brake.engaged()) << "a spring-applied brake holds before anything is powered";

    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.enable());

    const int enabled = drive.index_of("pwm", "enable");
    const int released = drive.index_of("brake", "release");
    ASSERT_GE(enabled, 0) << "the loop has to have been powered";
    ASSERT_GE(released, 0) << "and the brake has to have let go";
    EXPECT_LT(enabled, released) << "the opposite order is a vertical axis taking a step down on every enable";
    EXPECT_FALSE(drive.hal.mechanical_brake.engaged());
    EXPECT_TRUE(drive.hal.pwm.enabled());
    EXPECT_FALSE(drive.supervisor.status().brake_engaged) << "and the status agrees with the device";
}

// 2. Disable: the brake holds first, the loop stops second. The load is never held by nothing.
TEST(BrakeSequencing, DisablingHoldsBeforeTheLoopStops) {
    Drive drive;
    ASSERT_EQ(ErrorCode::Ok, drive.motor.init());
    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.configure(supervisor_config<Supervisor>()));
    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.enable());
    drive.log.clear();

    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.disable());

    const int engaged = drive.index_of("brake", "engage");
    const int disabled = drive.index_of("pwm", "disable");
    ASSERT_GE(engaged, 0);
    ASSERT_GE(disabled, 0);
    EXPECT_LT(engaged, disabled) << "cutting the inverter first free-falls the load for as long as the brake takes";
    EXPECT_TRUE(drive.hal.mechanical_brake.engaged());
    EXPECT_FALSE(drive.hal.pwm.enabled());
    EXPECT_TRUE(drive.supervisor.status().brake_engaged);
}

// 3. A brake that refuses to release does not leave the drive running: the loop is stopped again and the caller is told.
TEST(BrakeSequencing, ARefusedReleaseStopsTheLoopAgainRatherThanLeavingItRunning) {
    Drive drive;
    ASSERT_EQ(ErrorCode::Ok, drive.motor.init());
    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.configure(supervisor_config<Supervisor>()));
    drive.hal.mechanical_brake.set_release_fault(true);

    EXPECT_NE(ErrorCode::Ok, drive.supervisor.enable());
    EXPECT_TRUE(drive.hal.mechanical_brake.engaged()) << "the shaft is still held, which is the safe half";
    EXPECT_FALSE(drive.hal.pwm.enabled()) << "and the drive is not left running against a brake that thinks it is on";
    // The log says it: the release was attempted, and a stop followed it.
    const int released = drive.index_of("brake", "release");
    const int disabled = drive.index_of("pwm", "disable");
    ASSERT_GE(released, 0);
    ASSERT_GE(disabled, 0);
    EXPECT_LT(released, disabled) << "the stop is the response to the refusal, not a step that was already planned";
}

// 4. A brake that refuses to hold does not stop the drive: the error comes back with the loop still enabled.
TEST(BrakeSequencing, ARefusedEngageLeavesTheDriveRunningBecauseTheAlternativeIsDroppingTheLoad) {
    Drive drive;
    ASSERT_EQ(ErrorCode::Ok, drive.motor.init());
    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.configure(supervisor_config<Supervisor>()));
    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.enable());
    drive.hal.mechanical_brake.set_engage_fault(true);

    EXPECT_NE(ErrorCode::Ok, drive.supervisor.disable());
    EXPECT_TRUE(drive.hal.pwm.enabled()) << "cutting the output anyway would turn a failed brake into a dropped load";
    EXPECT_FALSE(drive.hal.mechanical_brake.engaged());
    EXPECT_FALSE(drive.supervisor.status().brake_engaged) << "the status reports the device, not the intention";
}

// 5. A trip holds the load on its way down, and a brake that fails to do so never delays the stop.
TEST(BrakeSequencing, ATripAsksTheBrakeToHoldAndStopsRegardlessOfTheAnswer) {
    Drive drive;
    ASSERT_EQ(ErrorCode::Ok, drive.motor.init());
    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.configure(supervisor_config<Supervisor>()));
    ASSERT_EQ(ErrorCode::Ok, drive.supervisor.enable());
    drive.hal.mechanical_brake.set_engage_fault(true);  // the brake is the thing that fails
    drive.log.clear();

    drive.hal.dc_link.set_voltage_v(70.0F);  // over-voltage: the protector trips on this tick
    EXPECT_EQ(ErrorCode::Fault, drive.supervisor.step(kDt));

    const int engaged = drive.index_of("brake", "engage");
    const int disabled = drive.index_of("pwm", "disable");
    ASSERT_GE(engaged, 0) << "the brake is asked, from the same rule disable() follows";
    ASSERT_GE(disabled, 0) << "and the output still goes away";
    EXPECT_LT(engaged, disabled);
    EXPECT_FALSE(drive.hal.pwm.enabled()) << "a protection that waits for a brake is not a protection";
    EXPECT_EQ(FaultCode::OverVoltage, drive.motor.fault_code());
    EXPECT_FALSE(drive.supervisor.status().brake_engaged) << "and the failed grip is not reported as a grip";
}

// 6. A HalPolicy with no brake sequences with one step fewer -- and does not invent a state for a device that is absent.
TEST(BrakeSequencing, ADriveWithoutABrakeStillEnablesAndNeverClaimsOne) {
    // The shipped stub HAL has no `mechanical_brake` member at all.
    static_assert(!foc::core::concepts::has_mechanical_brake_v<foc::driver::StubHal>,
                  "the no-brake fixture has to be a policy that really lacks the member");
    static_assert(foc::core::concepts::has_mechanical_brake_v<LoggingHal>,
                  "and the one under test has to really have it, or the case above proves nothing about the branch");

    foc::driver::StubHal hal{};
    using PlainMotor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi,
                                         foc::driver::StubHal>;
    using PlainSupervisor = foc::motor::DriveSupervisor<PlainMotor, foc::driver::StubHal>;
    static_cast<void>(hal.pwm.configure(20000.0F, 0.0F));
    hal.position.set_sensor_present(true);
    hal.dc_link.set_voltage_v(kVBus);
    const foc::core::Uvw<float> zero{};
    static_cast<void>(hal.current_sense.load(&zero, 1U));

    PlainMotor motor(motor_config(), policies<PlainMotor>(), hal);
    PlainSupervisor supervisor(motor, hal);
    ASSERT_EQ(ErrorCode::Ok, motor.init());
    ASSERT_EQ(ErrorCode::Ok, supervisor.configure(supervisor_config<PlainSupervisor>()));
    ASSERT_EQ(ErrorCode::Ok, supervisor.enable());
    EXPECT_TRUE(hal.pwm.enabled());
    EXPECT_FALSE(supervisor.status().brake_engaged) << "nothing there is not the same as something released";
    ASSERT_EQ(ErrorCode::Ok, supervisor.disable());
    EXPECT_FALSE(hal.pwm.enabled());
}

// 7. The stub's own contract: it starts holding, and a refused call leaves the state where it was.
TEST(BrakeSequencing, TheReferenceBrakeStartsHoldingAndARefusalChangesNothing) {
    foc::driver::StubMechanicalBrake brake{};
    EXPECT_TRUE(brake.engaged()) << "a spring-applied brake holds at power-on";
    EXPECT_EQ(ErrorCode::Ok, brake.engage()) << "and asking it to hold what is already held is not an error";
    EXPECT_TRUE(brake.engaged());

    brake.set_release_fault(true);
    EXPECT_EQ(ErrorCode::State, brake.release());
    EXPECT_TRUE(brake.engaged()) << "a refused release leaves the shaft held";
    brake.set_release_fault(false);
    ASSERT_EQ(ErrorCode::Ok, brake.release());
    EXPECT_FALSE(brake.engaged());

    brake.set_engage_fault(true);
    EXPECT_EQ(ErrorCode::State, brake.engage());
    EXPECT_FALSE(brake.engaged()) << "a refused engage leaves the shaft free";
    brake.set_engage_fault(false);

    const auto counts = brake.call_counts();
    EXPECT_EQ(2U, counts.first) << "refusals are counted: they are what a sequencer has to reason about";
    EXPECT_EQ(2U, counts.second);
}

} // namespace
