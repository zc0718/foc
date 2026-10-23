#include <array>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/sm.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/types.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>



namespace {

using Motor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::driver::StubHal>;

constexpr float kDt = 1e-4F;
constexpr float kVBus = 24.0F;

auto make_config() -> foc::motor::MotorConfig {
    // rs, ld, lq, kt, j, b, pole_pairs, i_max, pwm_freq, deadtime
    return foc::motor::MotorConfig{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-5F, 1e-6F, 7U, 5.0F, 20000.0F, 500.0F};
}

auto make_policies() -> Motor::Policies {
    const foc::core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const foc::core::Pi::Config speed{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F};
    return Motor::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{}, foc::core::Pi{current},
                           foc::core::Pi{current}, foc::core::Pi{speed}};
}

/// Sensored standstill rig: unknown angles never enter the picture, so the expected duties are exact.
auto make_hal(foc::driver::StubHal& hal) -> void {
    hal.dc_link.set_voltage_v(kVBus);
    hal.position.set_sensor_present(true);
    hal.position.set_angle_mech_rad(0.0F);
    hal.position.set_velocity_rad_s(0.0F);

    const foc::core::Uvw<float> zero{};
    ASSERT_EQ(foc::ErrorCode::Ok, hal.current_sense.load(&zero, 1U));
}

} // namespace



TEST(Motor, TenStepReplayIsDeterministic) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    ASSERT_EQ(foc::core::MotorState::Run, motor.state());

    motor.set_torque(0.5F);

    std::array<foc::core::Uvw<float>, 10> duties{};
    for (auto& duty : duties) {
        ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
        duty = hal.pwm.last_duty();
    }

    EXPECT_EQ(10U, hal.pwm.write_count());
    for (const auto& duty : duties) {
        EXPECT_FLOAT_EQ(duties[0].u, duty.u);
        EXPECT_FLOAT_EQ(duties[0].v, duty.v);
        EXPECT_FLOAT_EQ(duties[0].w, duty.w);
    }
}



TEST(Motor, DutyMatchesTheModulatorForTheCommandedVoltage) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    // kp = 1, zero current, iq_ref = 0.5 A, theta = 0 -> v_q = 0.5 V, v_d = 0 V.
    motor.set_torque(0.5F);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, 0.5F}, kVBus);
    const foc::core::Uvw<float> actual = hal.pwm.last_duty();

    EXPECT_NEAR(expected.duty.u, actual.u, 1e-6F);
    EXPECT_NEAR(expected.duty.v, actual.v, 1e-6F);
    EXPECT_NEAR(expected.duty.w, actual.w, 1e-6F);
    EXPECT_FLOAT_EQ(0.0F, motor.telemetry().iq_a);
}



TEST(Motor, TickBeforeStartIsRejected) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());

    EXPECT_EQ(foc::ErrorCode::State, motor.tick(kDt));
    EXPECT_EQ(0U, hal.pwm.write_count());
}



TEST(Motor, NonPositiveDtIsRejected) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    EXPECT_EQ(foc::ErrorCode::Param, motor.tick(0.0F));
    EXPECT_EQ(0U, hal.pwm.write_count());
}



TEST(Motor, SpeedModeDrivesIqFromTheSpeedLoop) {
    foc::driver::StubHal hal;
    make_hal(hal);
    hal.position.set_velocity_rad_s(100.0F); // measured 100 rad/s against a 50 rad/s reference

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    motor.set_speed(50.0F);
    EXPECT_EQ(foc::motor::ReferenceMode::Speed, motor.mode());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    // speed PI: kp * (50 - 100) = -0.5 A -> v_q = -0.5 V at theta = 0.
    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, -0.5F}, kVBus);
    EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);
}



TEST(Motor, StopDisablesTheInverter) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    EXPECT_EQ(foc::ErrorCode::Ok, motor.stop());
    EXPECT_EQ(foc::core::MotorState::Idle, motor.state());
    EXPECT_FALSE(hal.pwm.enabled());
}



TEST(Motor, StartIsIllegalFromFault) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    motor.fault();
    EXPECT_EQ(foc::core::MotorState::Fault, motor.state());

    EXPECT_EQ(foc::ErrorCode::State, motor.start());
    EXPECT_EQ(foc::ErrorCode::Ok, motor.clear_fault());
    EXPECT_EQ(foc::core::MotorState::Idle, motor.state());
}



TEST(Motor, SensorlessRunUsesTheObserverAndSeedsItOnStart) {
    foc::driver::StubHal hal;
    make_hal(hal);
    hal.position.set_sensor_present(false);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    // start() seeded the flux from the nameplate: psi = Kt / (1.5 * pole_pairs).
    const float psi = foc::psi_f_wb(motor.config());
    EXPECT_NEAR(psi, motor.observer().flux().alpha, 1e-6F);

    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    EXPECT_EQ(1U, hal.pwm.write_count());
}
