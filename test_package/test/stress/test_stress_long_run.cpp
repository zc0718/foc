// Long-run stress: the same control loop the unit suite exercises, but driven for far longer than
// any single unit test and across repeated reference-mode churn. It stays deliberately cheap (a
// fraction of a second of wall time) so every non-coverage leg can afford it -- the value here is
// the iteration count, not a clever scenario.
//
// This file also keeps `test_package/test/stress/` alive in git: an empty directory is not tracked,
// and the recipe writes its generated gtest `main.cpp` into this directory before configuring.



#include <cmath>
#include <cstdint>

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
constexpr std::uint32_t kTicks = 200000U; // 20 s of simulated time
constexpr std::uint32_t kModeSwitches = 2000U;

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

/// Sensored standstill rig: the sensor is the only angle source, so nothing depends on the observer
/// converging before the loop can be exercised.
auto make_hal(foc::driver::StubHal& hal) -> void {
    hal.dc_link.set_voltage_v(kVBus);
    hal.position.set_sensor_present(true);
    hal.position.set_angle_mech_rad(0.0F);
    hal.position.set_velocity_rad_s(0.0F);

    const foc::core::Uvw<float> zero{};
    ASSERT_EQ(foc::ErrorCode::Ok, hal.current_sense.load(&zero, 1U));
}

auto finite(float value) -> bool { return std::isfinite(value); }

auto duty_in_range(const foc::core::Uvw<float>& duty) -> bool {
    const auto one_ok = [](float d) { return finite(d) && d >= 0.0F && d <= 1.0F; };
    return one_ok(duty.u) && one_ok(duty.v) && one_ok(duty.w);
}

} // namespace



TEST(Stress, TwentySecondsOfTorqueStepsStayFiniteAndBounded) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    ASSERT_EQ(foc::core::MotorState::Run, motor.state());

    const std::uint32_t half = kTicks / 2U;
    std::uint32_t failed_ticks = 0U;
    std::uint32_t out_of_range_duties = 0U;
    std::uint32_t non_finite_readbacks = 0U;

    for (std::uint32_t i = 0U; i < kTicks; ++i) {
        // A hard sign flip every half run: the worst case for an integrator that is not clamped.
        motor.set_torque(i < half ? 1.0F : -1.0F);
        if (motor.tick(kDt) != foc::ErrorCode::Ok) {
            ++failed_ticks;
        }
        if (!duty_in_range(hal.pwm.last_duty())) {
            ++out_of_range_duties;
        }
        const foc::motor::Telemetry t = motor.telemetry();
        if (!finite(t.speed_mech_rad_s) || !finite(t.id_a) || !finite(t.iq_a)) {
            ++non_finite_readbacks;
        }
    }

    EXPECT_EQ(0U, failed_ticks);
    EXPECT_EQ(0U, out_of_range_duties);
    EXPECT_EQ(0U, non_finite_readbacks);
    EXPECT_EQ(kTicks, hal.pwm.write_count());
    EXPECT_EQ(foc::core::MotorState::Run, motor.state());
    EXPECT_EQ(foc::ErrorCode::Ok, motor.status());
}



TEST(Stress, RepeatedModeChurnDoesNotLatchOrDrift) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    for (std::uint32_t i = 0U; i < kModeSwitches; ++i) {
        if (i % 2U == 0U) {
            motor.set_torque(1.5F);
        } else {
            motor.set_speed(50.0F);
        }
        ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    }

    // Switching the source of the q-axis reference must never move the state machine or latch a
    // fault: the reference is data, not a command to stop.
    EXPECT_EQ(foc::core::MotorState::Run, motor.state());
    EXPECT_EQ(foc::ErrorCode::Ok, motor.status());
    EXPECT_TRUE(duty_in_range(hal.pwm.last_duty()));

    const foc::motor::Telemetry t = motor.telemetry();
    EXPECT_TRUE(finite(t.speed_mech_rad_s));
    EXPECT_TRUE(finite(t.id_a));
    EXPECT_TRUE(finite(t.iq_a));
}
