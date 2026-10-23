#include <array>

#include <gtest/gtest.h>

#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
#include <foc/driver/scripted_current.hpp>
#include <foc/driver/sim_position.hpp>
#include <foc/driver/stub_dc_link.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/driver/stub_pwm.hpp>
#include <foc/motor/foc_loop.hpp>
#include <foc/motor/motor_config.hpp>



TEST(StubPwm, ConfigureValidatesItsArguments) {
    foc::driver::StubPwm pwm;

    EXPECT_EQ(foc::ErrorCode::Param, pwm.configure(0.0F, 500.0F));
    EXPECT_EQ(foc::ErrorCode::Param, pwm.configure(20000.0F, -1.0F));
    EXPECT_EQ(foc::ErrorCode::Ok, pwm.configure(20000.0F, 500.0F));
    EXPECT_FLOAT_EQ(20000.0F, pwm.freq_hz());
}



TEST(StubPwm, EnableRequiresConfiguration) {
    foc::driver::StubPwm pwm;

    EXPECT_EQ(foc::ErrorCode::State, pwm.enable());
    EXPECT_FALSE(pwm.enabled());

    ASSERT_EQ(foc::ErrorCode::Ok, pwm.configure(10000.0F, 0.0F));
    EXPECT_EQ(foc::ErrorCode::Ok, pwm.enable());
    EXPECT_TRUE(pwm.enabled());

    pwm.disable();
    EXPECT_FALSE(pwm.enabled());
}



TEST(StubPwm, RecordsDutyAndFault) {
    foc::driver::StubPwm pwm;
    ASSERT_EQ(foc::ErrorCode::Ok, pwm.configure(10000.0F, 0.0F));

    const foc::core::Uvw<float> duty{0.1F, 0.2F, 0.3F};
    pwm.write_duty(duty);

    EXPECT_FLOAT_EQ(0.2F, pwm.last_duty().v);
    EXPECT_EQ(1U, pwm.write_count());
    EXPECT_FALSE(pwm.fault());

    pwm.set_fault(true);
    EXPECT_TRUE(pwm.fault());

    pwm.reset();
    EXPECT_FALSE(pwm.fault());
    EXPECT_EQ(0U, pwm.write_count());
    EXPECT_FLOAT_EQ(0.0F, pwm.freq_hz());
}



TEST(ScriptedCurrent, LoadValidatesItsArguments) {
    foc::driver::ScriptedCurrent sense;
    const foc::core::Uvw<float> sample{};

    EXPECT_EQ(foc::ErrorCode::Param, sense.load(nullptr, 1U));
    EXPECT_EQ(foc::ErrorCode::Param, sense.load(&sample, 0U));
    EXPECT_EQ(foc::ErrorCode::Param, sense.load(&sample, foc::driver::ScriptedCurrent::kCapacity + 1U));
}



TEST(ScriptedCurrent, ReplaysTheScriptAndHoldsTheLastSample) {
    foc::driver::ScriptedCurrent sense;
    const std::array<foc::core::Uvw<float>, 3> script{foc::core::Uvw<float>{1.0F, 1.0F, 1.0F},
                                                      foc::core::Uvw<float>{2.0F, 2.0F, 2.0F},
                                                      foc::core::Uvw<float>{3.0F, 3.0F, 3.0F}};
    ASSERT_EQ(foc::ErrorCode::Ok, sense.load(script.data(), script.size()));

    EXPECT_FLOAT_EQ(1.0F, sense.read_uvw().u);
    EXPECT_FLOAT_EQ(2.0F, sense.read_uvw().u);
    EXPECT_FLOAT_EQ(3.0F, sense.read_uvw().u);
    EXPECT_TRUE(sense.exhausted());
    EXPECT_FLOAT_EQ(3.0F, sense.read_uvw().u); // last sample repeats
}



TEST(ScriptedCurrent, UnloadedSenseReadsZeroAndCannotCalibrate) {
    foc::driver::ScriptedCurrent sense;

    EXPECT_FLOAT_EQ(0.0F, sense.read_uvw().u);
    EXPECT_FALSE(sense.exhausted());
    EXPECT_EQ(foc::ErrorCode::State, sense.calibrate_offset());
    EXPECT_FALSE(sense.calibrated());
}



TEST(ScriptedCurrent, CalibrationAndRateAreRecorded) {
    foc::driver::ScriptedCurrent sense;
    const foc::core::Uvw<float> sample{};
    ASSERT_EQ(foc::ErrorCode::Ok, sense.load(&sample, 1U));

    EXPECT_EQ(foc::ErrorCode::Ok, sense.calibrate_offset());
    EXPECT_TRUE(sense.calibrated());

    sense.set_sample_rate_hz(10000.0F);
    EXPECT_FLOAT_EQ(10000.0F, sense.sample_rate_hz());

    sense.reset();
    EXPECT_FLOAT_EQ(0.0F, sense.sample_rate_hz());
    EXPECT_FALSE(sense.calibrated());
}



TEST(SimPosition, WrapsTheAngleIntoZeroToTwoPi) {
    foc::driver::SimPosition position;

    position.set_angle_mech_rad(-1.0F);
    EXPECT_NEAR(foc::core::kTwoPi<float> - 1.0F, position.angle_mech_rad(), 1e-5F);

    position.set_angle_mech_rad(foc::core::kTwoPi<float> + 1.0F);
    EXPECT_NEAR(1.0F, position.angle_mech_rad(), 1e-5F);
}



TEST(SimPosition, ReportsVelocityAndSensorPresence) {
    foc::driver::SimPosition position;
    EXPECT_TRUE(position.has_sensor());

    position.set_velocity_rad_s(42.0F);
    EXPECT_FLOAT_EQ(42.0F, position.velocity_rad_s());

    position.set_sensor_present(false);
    EXPECT_FALSE(position.has_sensor());
}



TEST(StubDcLink, ReportsTheSettableVoltage) {
    foc::driver::StubDcLink link;

    EXPECT_FLOAT_EQ(0.0F, link.voltage_v());
    link.set_voltage_v(48.0F);
    EXPECT_FLOAT_EQ(48.0F, link.voltage_v());
}



TEST(StubHal, ResetRestoresEveryMember) {
    foc::driver::StubHal hal;
    ASSERT_EQ(foc::ErrorCode::Ok, hal.pwm.configure(10000.0F, 0.0F));
    hal.dc_link.set_voltage_v(24.0F);
    hal.position.set_velocity_rad_s(10.0F);
    hal.position.set_sensor_present(false);

    hal.reset();

    EXPECT_FLOAT_EQ(0.0F, hal.pwm.freq_hz());
    EXPECT_FLOAT_EQ(0.0F, hal.dc_link.voltage_v());
    EXPECT_FLOAT_EQ(0.0F, hal.position.velocity_rad_s());
    EXPECT_TRUE(hal.position.has_sensor());
}



TEST(MotorConfig, FluxLinkageFollowsTheTorqueConstant) {
    foc::motor::MotorConfig config{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-5F, 1e-6F, 7U, 5.0F, 20000.0F, 500.0F};
    EXPECT_NEAR(0.05F / (1.5F * 7.0F), foc::psi_f_wb(config), 1e-9F);

    config.pole_pairs = 0U;
    EXPECT_FLOAT_EQ(0.0F, foc::psi_f_wb(config));
}



TEST(FocLoop, MeasureDqAtZeroAngleEqualsTheClarkeOutput) {
    const foc::core::Uvw<float> i_uvw{1.0F, -0.5F, -0.5F};

    const foc::core::Dq<float> dq = foc::motor::measure_dq(i_uvw, 0.0F);

    EXPECT_NEAR(1.0F, dq.d, 1e-6F);
    EXPECT_NEAR(0.0F, dq.q, 1e-6F);
}



TEST(FocLoop, ApplyVoltageMatchesTheModulator) {
    const foc::core::Dq<float> v_dq{0.0F, 0.5F};

    const foc::core::SvpwmResult frame = foc::motor::apply_voltage(v_dq, 0.0F, 24.0F);
    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, 0.5F}, 24.0F);

    EXPECT_NEAR(expected.duty.u, frame.duty.u, 1e-6F);
    EXPECT_EQ(foc::ErrorCode::Ok, frame.status);
}
