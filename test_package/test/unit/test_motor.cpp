#include <array>
#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/feedforward.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/reference.hpp>
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



namespace {

/// [en] A reference generator that sets the d axis itself and records what the motor handed it.
/// [zh] 自行给出 d 轴、并记录电机交给它的输入的参考生成器。
class FixedIdReference final {
  public:
    auto generate(const foc::core::ReferenceInput& in) noexcept -> foc::core::Dq<float> {
        last_omega_e = in.omega_e_rad_s;
        last_v_bus = in.v_bus_v;
        return foc::core::Dq<float>{id_a, in.iq_command_a};
    }

    float id_a{0.0F};
    float last_omega_e{0.0F};
    float last_v_bus{0.0F};
};

using MotorWithReference =
    foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::driver::StubHal,
                      FixedIdReference>;

auto make_policies_with_reference() -> MotorWithReference::Policies {
    const foc::core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const foc::core::Pi::Config speed{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F};
    return MotorWithReference::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{},
                                        foc::core::Pi{current}, foc::core::Pi{current}, foc::core::Pi{speed},
                                        FixedIdReference{}};
}

} // namespace



TEST(Motor, HardwareFaultLatchesBeforeAnyCurrentIsComputed) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    ASSERT_TRUE(hal.pwm.enabled());

    hal.pwm.set_fault(true);

    EXPECT_EQ(foc::ErrorCode::Fault, motor.tick(kDt));
    EXPECT_EQ(foc::ErrorCode::Fault, motor.status());
    EXPECT_EQ(foc::core::MotorState::Fault, motor.state());
    EXPECT_FALSE(hal.pwm.enabled());
    EXPECT_EQ(0U, hal.pwm.write_count()); // nothing was written on the faulting step
}



TEST(Motor, ClearingTheLatchIsNotClearingTheInverter) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    hal.pwm.set_fault(true);
    ASSERT_EQ(foc::ErrorCode::Fault, motor.tick(kDt));

    // Clearing the latch while the inverter still reports the fault re-latches immediately.
    ASSERT_EQ(foc::ErrorCode::Ok, motor.clear_fault());
    EXPECT_EQ(foc::core::MotorState::Idle, motor.state());
    EXPECT_EQ(foc::ErrorCode::Fault, motor.tick(kDt));

    // Only after the hardware reports healthy again does the motor come back.
    hal.pwm.set_fault(false);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.clear_fault());
    EXPECT_EQ(foc::core::MotorState::Idle, motor.state());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    EXPECT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    EXPECT_EQ(1U, hal.pwm.write_count());
}



TEST(Motor, ReferenceGeneratorOwnsTheDqSplit) {
    foc::driver::StubHal hal;
    make_hal(hal);

    MotorWithReference motor(make_config(), make_policies_with_reference(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    motor.set_torque(0.5F);
    motor.reference().id_a = -0.2F;
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    // The generator saw this step's bus voltage and electrical speed (standstill here).
    EXPECT_FLOAT_EQ(kVBus, motor.reference().last_v_bus);
    EXPECT_FLOAT_EQ(0.0F, motor.reference().last_omega_e);

    // kp = 1, zero current and theta = 0: v_d = -0.2 V, v_q = 0.5 V.
    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{-0.2F, 0.5F}, kVBus);
    EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);
    EXPECT_NEAR(expected.duty.v, hal.pwm.last_duty().v, 1e-6F);
    EXPECT_NEAR(expected.duty.w, hal.pwm.last_duty().w, 1e-6F);
}



TEST(Motor, AReferenceOutsideTheNameplateIsClampedPerAxis) {
    foc::driver::StubHal hal;
    make_hal(hal);

    MotorWithReference motor(make_config(), make_policies_with_reference(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    motor.set_torque(0.5F);
    motor.reference().id_a = 1000.0F; // nameplate i_max is 5 A
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{5.0F, 0.5F}, kVBus);
    EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);
    EXPECT_NEAR(expected.duty.v, hal.pwm.last_duty().v, 1e-6F);
    EXPECT_NEAR(expected.duty.w, hal.pwm.last_duty().w, 1e-6F);
}



TEST(Motor, ANonFiniteReferenceCollapsesToZeroInsteadOfPoisoningTheLoop) {
    foc::driver::StubHal hal;
    make_hal(hal);

    MotorWithReference motor(make_config(), make_policies_with_reference(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    motor.set_torque(0.5F);
    motor.reference().id_a = std::numeric_limits<float>::quiet_NaN();
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, 0.5F}, kVBus);
    EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);
    EXPECT_TRUE(std::isfinite(motor.telemetry().id_a));
}



TEST(Motor, SensorVelocityIsUsedWhenItIsFinite) {
    foc::driver::StubHal hal;
    make_hal(hal);
    hal.position.set_velocity_rad_s(3.0F);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    EXPECT_FLOAT_EQ(3.0F, motor.telemetry().speed_mech_rad_s);
}



TEST(Motor, SensorWithoutVelocityFallsBackToTheObserverSpeed) {
    foc::driver::StubHal hal;
    make_hal(hal);
    // The HAL contract defines NaN as "this sensor cannot measure velocity".
    hal.position.set_velocity_rad_s(std::numeric_limits<float>::quiet_NaN());

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    motor.set_torque(1.0F);

    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    }

    // The speed must come from the observer, not from the NaN the sensor reported. The exact
    // value is whatever the flux observer estimates on this static rig (zero current, a voltage
    // command and no plant), so only its finiteness and magnitude are asserted here.
    const foc::motor::Telemetry telemetry = motor.telemetry();
    EXPECT_TRUE(std::isfinite(telemetry.speed_mech_rad_s));
    EXPECT_LT(std::fabs(telemetry.speed_mech_rad_s), 1e4F);
    EXPECT_TRUE(std::isfinite(telemetry.iq_a));
}



namespace {

using MotorWithFeedforward =
    foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::driver::StubHal,
                      foc::core::NullReferenceGenerator, foc::core::DecouplingFeedforward>;

/// [en] Zero-gain PI, so whatever reaches the modulator is the feedforward alone.
/// [zh] 零增益 PI，于是送到调制器的只有前馈项。
auto make_zero_gain_policies(const foc::motor::MotorConfig& config,
                             foc::core::DecouplingFeedforward::Terms terms) -> MotorWithFeedforward::Policies {
    const foc::core::Pi::Config zero{0.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    foc::core::DecouplingFeedforward::Config ff = foc::feedforward_config(config);
    ff.terms = terms;
    return MotorWithFeedforward::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{},
                                          foc::core::Pi{zero}, foc::core::Pi{zero}, foc::core::Pi{zero},
                                          foc::core::NullReferenceGenerator{},
                                          foc::core::DecouplingFeedforward{ff}};
}

/// [en] Load a current vector whose dq image at theta = 0 is exactly (id, iq).
/// [zh] 载入一个电流矢量：theta = 0 时它的 dq 像正好是 (id, iq)。
auto load_dq_current(foc::driver::StubHal& hal, float id_a, float iq_a) -> void {
    const foc::core::Uvw<float> i_uvw = foc::core::inv_clarke(foc::core::Ab<float>{id_a, iq_a});
    ASSERT_EQ(foc::ErrorCode::Ok, hal.current_sense.load(&i_uvw, 1U));
}

} // namespace



TEST(Motor, FeedforwardAddsCrossCouplingAndBemfAfterThePi) {
    foc::driver::StubHal hal;
    make_hal(hal);
    hal.position.set_velocity_rad_s(10.0F); // omega_e = 10 * 7 = 70 rad/s
    load_dq_current(hal, 1.0F, 2.0F);

    const foc::motor::MotorConfig config = make_config();
    MotorWithFeedforward motor(config,
                               make_zero_gain_policies(config, foc::core::DecouplingFeedforward::Terms::CrossAndBemf),
                               hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    // Reported current is the dq image of what the sense stub delivered, not the reference.
    EXPECT_FLOAT_EQ(1.0F, motor.telemetry().id_a);
    EXPECT_FLOAT_EQ(2.0F, motor.telemetry().iq_a);
    EXPECT_EQ(foc::core::DecouplingFeedforward::Terms::CrossAndBemf, motor.feed_forward().config().terms);

    const float omega_e = 70.0F;
    const float v_d_ff = -omega_e * config.lq * 2.0F;
    const float v_q_ff = (omega_e * config.ld * 1.0F) + (omega_e * foc::psi_f_wb(config));
    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{v_d_ff, v_q_ff}, kVBus);

    EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);
    EXPECT_NEAR(expected.duty.v, hal.pwm.last_duty().v, 1e-6F);
    EXPECT_NEAR(expected.duty.w, hal.pwm.last_duty().w, 1e-6F);
}



TEST(Motor, EachFeedforwardTermCanBeSelectedOnItsOwn) {
    const foc::motor::MotorConfig config = make_config();
    const float omega_e = 70.0F;

    // Cross only: no back-EMF on q.
    {
        foc::driver::StubHal hal;
        make_hal(hal);
        hal.position.set_velocity_rad_s(10.0F);
        load_dq_current(hal, 0.0F, 2.0F);

        MotorWithFeedforward motor(
            config, make_zero_gain_policies(config, foc::core::DecouplingFeedforward::Terms::Cross), hal);
        ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
        ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
        ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

        const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(
            foc::core::Ab<float>{-omega_e * config.lq * 2.0F, 0.0F}, kVBus);
        EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);
    }

    // Back-EMF only: the d axis stays untouched even with a large q current.
    {
        foc::driver::StubHal hal;
        make_hal(hal);
        hal.position.set_velocity_rad_s(10.0F);
        load_dq_current(hal, 1.0F, 2.0F);

        MotorWithFeedforward motor(
            config, make_zero_gain_policies(config, foc::core::DecouplingFeedforward::Terms::Bemf), hal);
        ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
        ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
        ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

        const foc::core::SvpwmResult expected =
            foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, omega_e * foc::psi_f_wb(config)}, kVBus);
        EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);
    }
}



namespace {

/// [en] Run a constant speed error for `ticks` ticks and return the duty the speed loop produced.
/// [zh] 用恒定速度误差跑 `ticks` 个 tick，返回速度环产生的占空比。
auto duty_after_speed_error(float speed_loop_period_s, int ticks) -> foc::core::Uvw<float> {
    foc::driver::StubHal hal;
    make_hal(hal); // measured speed 0, so a 100 rad/s reference is a constant 100 rad/s error

    foc::motor::MotorConfig config = make_config();
    config.speed_loop_period_s = speed_loop_period_s;

    // Current loop kp = 1, ki = 0 -> v_q equals iq_ref; speed loop kp = 0, ki = 1 -> out = integral.
    const foc::core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const foc::core::Pi::Config speed{0.0F, 1.0F, -5.0F, 5.0F, -5.0F, 5.0F};

    Motor motor(config,
                Motor::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{}, foc::core::Pi{current},
                                foc::core::Pi{current}, foc::core::Pi{speed}},
                hal);
    static_cast<void>(motor.init());
    motor.set_speed(100.0F);
    static_cast<void>(motor.start());

    for (int i = 0; i < ticks; ++i) {
        static_cast<void>(motor.tick(kDt));
    }
    return hal.pwm.last_duty();
}

} // namespace



TEST(Motor, SpeedLoopDecimationKeepsTheIntegralOverTheSameElapsedTime) {
    constexpr int kTicks = 21;
    const float expected_iq = 100.0F * static_cast<float>(kTicks) * kDt; // ki = 1, error = 100 A
    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, expected_iq}, kVBus);

    // Every tick (period 0) and once every ten ticks must integrate the same total time. A decimator
    // that passed `dt_s` instead of the accumulated interval would land ~7x lower here.
    const foc::core::Uvw<float> every_tick = duty_after_speed_error(0.0F, kTicks);
    const foc::core::Uvw<float> decimated = duty_after_speed_error(10.0F * kDt, kTicks);

    EXPECT_NEAR(expected.duty.u, every_tick.u, 1e-6F);
    EXPECT_NEAR(expected.duty.u, decimated.u, 1e-6F);
    EXPECT_NEAR(every_tick.u, decimated.u, 1e-6F);
}



TEST(Motor, SpeedLoopPeriodMustNotBeNegative) {
    foc::driver::StubHal hal;
    make_hal(hal);

    foc::motor::MotorConfig config = make_config();
    config.speed_loop_period_s = -1e-3F;

    Motor motor(config, make_policies(), hal);
    EXPECT_EQ(foc::ErrorCode::Param, motor.init());
    EXPECT_EQ(0U, hal.pwm.write_count());
}



namespace {

/// [en] A HalPolicy that also answers "what time is it", which is what enables the overrun metric.
/// [zh] 额外能回答“现在几点”的 HalPolicy；过载指标就是靠它才启用。
class StepClock final {
  public:
    auto now_us() noexcept -> std::uint64_t { return now_us_; }

    std::uint64_t now_us_{0U};
};

struct HalWithClock final {
    using Pwm = foc::driver::StubPwm;
    using CurrentSense = foc::driver::ScriptedCurrent;
    using Position = foc::driver::SimPosition;
    using DcLink = foc::driver::StubDcLink;

    Pwm pwm;
    CurrentSense current_sense;
    Position position;
    DcLink dc_link;
    StepClock clock;
};

using MotorWithClock =
    foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, HalWithClock>;

} // namespace



TEST(Motor, TickIntervalIsMeasuredOnlyWhenTheHalHasAClock) {
    HalWithClock hal;
    hal.dc_link.set_voltage_v(kVBus);
    hal.position.set_sensor_present(true);
    hal.position.set_angle_mech_rad(0.0F);
    hal.position.set_velocity_rad_s(0.0F);
    const foc::core::Uvw<float> zero{};
    ASSERT_EQ(foc::ErrorCode::Ok, hal.current_sense.load(&zero, 1U));

    const foc::core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const foc::core::Pi::Config speed{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F};
    MotorWithClock motor(make_config(),
                         MotorWithClock::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{},
                                                  foc::core::Pi{current}, foc::core::Pi{current},
                                                  foc::core::Pi{speed}},
                         hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    motor.set_torque(0.5F);

    hal.clock.now_us_ = 1'000U;
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    EXPECT_EQ(0U, motor.telemetry().max_period_us); // the first sample only seeds the time base

    hal.clock.now_us_ = 1'500U;
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    EXPECT_EQ(500U, motor.telemetry().max_period_us);

    hal.clock.now_us_ = 1'600U;
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    EXPECT_EQ(500U, motor.telemetry().max_period_us); // the maximum is sticky, not the last value
}



TEST(Motor, AClocklessHalReportsNoPeriodAtAll) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    motor.set_torque(0.5F);

    for (int i = 0; i < 3; ++i) {
        ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    }

    // No clock, no time base: the library must not invent one.
    EXPECT_EQ(0U, motor.telemetry().max_period_us);
}



namespace {

using MotorWithPosition =
    foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::driver::StubHal,
                      foc::core::NullReferenceGenerator, foc::core::NullFeedforward, foc::core::PositionP>;

/// [en] Plain proportional current/speed loops, so the duties show the position loop's arithmetic.
/// [zh] 纯比例的电流/速度环，于是占空比直接反映位置环的算术。
auto make_position_policies(float position_kp, float velocity_limit_rad_s) -> MotorWithPosition::Policies {
    const foc::core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const foc::core::Pi::Config speed{1.0F, 0.0F, -5.0F, 5.0F, -5.0F, 5.0F};
    // No acceleration limit: these cases are about the position loop's arithmetic, so the simplest
    // honest controller is a bare P.
    // 不给加速度限制：这几条用例关心的是位置环的算术，因此最诚实的控制器就是纯 P。
    const foc::core::PositionP::Config position{position_kp, velocity_limit_rad_s, 0.0F};
    return MotorWithPosition::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}),
                                       foc::core::Svpwm{},
                                       foc::core::Pi{current},
                                       foc::core::Pi{current},
                                       foc::core::Pi{speed},
                                       foc::core::NullReferenceGenerator{},
                                       foc::core::NullFeedforward{},
                                       foc::core::PositionP{position}};
}

} // namespace



TEST(Motor, PositionModeWithoutAPositionLoopIsNotImplAndNotLatched) {
    foc::driver::StubHal hal;
    make_hal(hal);

    Motor motor(make_config(), make_policies(), hal); // default: NullPositionControl
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    motor.set_position(0.1F);
    EXPECT_EQ(foc::motor::ReferenceMode::Position, motor.mode());
    EXPECT_EQ(foc::ErrorCode::NotImpl, motor.tick(kDt));

    // A capability that was not compiled in is not a fault: nothing is latched, so the very next
    // tick reports the same thing instead of turning into a state the caller must clear.
    EXPECT_EQ(foc::ErrorCode::Ok, motor.status());
    EXPECT_EQ(foc::ErrorCode::NotImpl, motor.tick(kDt));
    EXPECT_EQ(0U, hal.pwm.write_count());
}



TEST(Motor, PositionModeWithoutASensorIsNotImpl) {
    foc::driver::StubHal hal;
    make_hal(hal);
    hal.position.set_sensor_present(false);

    MotorWithPosition motor(make_config(), make_position_policies(10.0F, 50.0F), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    motor.set_position(0.1F);
    EXPECT_EQ(foc::ErrorCode::NotImpl, motor.tick(kDt));
    EXPECT_EQ(foc::ErrorCode::Ok, motor.status());
}



TEST(Motor, PositionLoopFeedsTheSpeedLoopTheVelocityItAsks) {
    foc::driver::StubHal hal;
    make_hal(hal); // angle 0, velocity 0

    MotorWithPosition motor(make_config(), make_position_policies(10.0F, 50.0F), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    motor.set_position(0.1F);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    // position: 10 * (0.1 - 0) = 1.0 rad/s; speed: 1 * (1.0 - 0) = 1.0 A; theta = 0 -> v_q = 1.0 V
    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, 1.0F}, kVBus);
    EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);
    EXPECT_NEAR(expected.duty.v, hal.pwm.last_duty().v, 1e-6F);
    EXPECT_NEAR(expected.duty.w, hal.pwm.last_duty().w, 1e-6F);
}



TEST(Motor, TheVelocityCommandIsClampedByTheLoopsOwnOutputRange) {
    foc::driver::StubHal hal;
    make_hal(hal);

    MotorWithPosition motor(make_config(), make_position_policies(10.0F, 50.0F), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    motor.set_position(10.0F); // 10 * 10 = 100 rad/s demanded, the loop may only ask for 50
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    // 50 rad/s -> 50 A demanded by the speed loop -> clamped to the 5 A nameplate.
    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, 5.0F}, kVBus);
    EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);
}



TEST(Motor, SwitchingModesLeavesPositionBehind) {
    foc::driver::StubHal hal;
    make_hal(hal);

    MotorWithPosition motor(make_config(), make_position_policies(10.0F, 50.0F), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    motor.set_position(1.0F);
    EXPECT_EQ(foc::motor::ReferenceMode::Position, motor.mode());

    motor.set_torque(0.5F);
    EXPECT_EQ(foc::motor::ReferenceMode::Torque, motor.mode());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, 0.5F}, kVBus);
    EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);

    motor.set_speed(0.0F);
    EXPECT_EQ(foc::motor::ReferenceMode::Speed, motor.mode());
}



TEST(Motor, TheCommandBoundsTheCurrentForThatMoveOnly) {
    foc::driver::StubHal hal;
    make_hal(hal);

    MotorWithPosition motor(make_config(), make_position_policies(10.0F, 50.0F), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    // The saw demands 100 rad/s -> 100 A, far beyond both the nameplate (5 A) and this move's 1 A.
    motor.set_command(foc::motor::Command{10.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F});
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, 1.0F}, kVBus);
    EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);

    // A plain set_torque is not affected by that ceiling: it is a different mode.
    motor.set_torque(0.5F);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    const foc::core::SvpwmResult torque = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, 0.5F}, kVBus);
    EXPECT_NEAR(torque.duty.u, hal.pwm.last_duty().u, 1e-6F);
}



TEST(Motor, TheCommandFeedforwardIsAddedAfterTheSpeedLoop) {
    foc::driver::StubHal hal;
    make_hal(hal);

    MotorWithPosition motor(make_config(), make_position_policies(10.0F, 50.0F), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    // Already on target, so the loops contribute nothing: the duty is the feedforward alone.
    motor.set_command(foc::motor::Command{0.0F, 0.0F, 0.5F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F});
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    const foc::core::SvpwmResult expected = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, 0.5F}, kVBus);
    EXPECT_NEAR(expected.duty.u, hal.pwm.last_duty().u, 1e-6F);
}



// ---------------------------------------------------------------------------------------------
// Alignment slot: the machine parks in Align until the sequence says it is safe to close the loop.
// 对齐槽位：机器停在 Align，直到流程表示可以闭环。
// ---------------------------------------------------------------------------------------------

namespace {

using MotorWithAligner =
    foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::driver::StubHal,
                      foc::core::NullReferenceGenerator, foc::core::NullFeedforward, foc::core::NullPositionControl,
                      foc::core::AlignRoutine>;

auto make_alignment_recipe(float hold_s, float verify_s) -> foc::core::AlignConfig {
    // current, ramp, hold, tolerance, verify, forced angle
    return foc::core::AlignConfig{1.0F, 0.0F, hold_s, 0.01F, verify_s, 0.0F};
}

auto make_aligner_policies(foc::core::AlignConfig recipe) -> MotorWithAligner::Policies {
    const foc::core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const foc::core::Pi::Config speed{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F};
    return MotorWithAligner::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}),
                                      foc::core::Svpwm{},
                                      foc::core::Pi{current},
                                      foc::core::Pi{current},
                                      foc::core::Pi{speed},
                                      foc::core::NullReferenceGenerator{},
                                      foc::core::NullFeedforward{},
                                      foc::core::NullPositionControl{},
                                      foc::core::AlignRoutine{recipe}};
}

} // namespace



TEST(Motor, StartParksTheMachineInAlignUntilTheSequenceFinishes) {
    foc::driver::StubHal hal;
    make_hal(hal);
    hal.position.set_angle_mech_rad(0.3F); // the rotor is parked away from electrical zero

    MotorWithAligner motor(make_config(), make_aligner_policies(make_alignment_recipe(0.01F, 0.0F)), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    EXPECT_EQ(foc::core::MotorState::Align, motor.state());

    // The sequence holds one current for 10 ms, then hands over.
    // 流程保持一个电流 10 ms，然后交接。
    int ticks_while_aligning = 0;
    while ((motor.state() == foc::core::MotorState::Align) && (ticks_while_aligning < 500)) {
        ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt)) << "alignment ticks must not fail";
        ++ticks_while_aligning;
    }

    EXPECT_EQ(foc::core::MotorState::Run, motor.state());
    EXPECT_NEAR(100.0F, static_cast<float>(ticks_while_aligning), 2.0F);
    EXPECT_EQ(foc::core::AlignState::Done, motor.aligner().state());
    // The captured offset was handed to the sensor through the optional HAL hook.
    // 捕获到的偏置通过可选 HAL 钩子交给了传感器。
    EXPECT_NEAR(0.3F, hal.position.mech_offset_rad(), 1e-4F);
}



TEST(Motor, AFailedAlignmentLatchesAFaultAndDisablesTheOutputs) {
    foc::driver::StubHal hal;
    make_hal(hal);

    MotorWithAligner motor(make_config(), make_aligner_policies(make_alignment_recipe(0.05F, 0.0F)), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    ASSERT_EQ(foc::core::MotorState::Align, motor.state());

    // A rotor that keeps moving is never captured: the hold window sees a swing, not a position.
    // 一直在动的转子永远不会被捕获：保持窗口看到的是摆动，而不是位置。
    while (motor.state() == foc::core::MotorState::Align) {
        hal.position.set_angle_mech_rad(1.0F * static_cast<float>(hal.pwm.write_count()) + 1.0F);
        const foc::ErrorCode rc = motor.tick(kDt);
        if (rc != foc::ErrorCode::Ok) {
            break;
        }
    }

    EXPECT_EQ(foc::core::MotorState::Fault, motor.state());
    EXPECT_EQ(foc::ErrorCode::Fault, motor.status());
    EXPECT_EQ(foc::core::AlignFailure::NotSettled, motor.aligner().failure());
    // Outputs off: an alignment that did not converge leaves the rotor at an unmeasured angle.
    // 输出已关闭：没收敛的对齐会把转子留在未被测量的角度上。
    EXPECT_FALSE(hal.pwm.enabled());
}



TEST(Motor, AnInvalidRecipeIsRejectedBeforeTheInverterIsEnabled) {
    foc::driver::StubHal hal;
    make_hal(hal);

    // Zero hold time: nothing to average over.
    // 保持时间为零：没有东西可平均。
    MotorWithAligner motor(make_config(), make_aligner_policies(make_alignment_recipe(0.0F, 0.0F)), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    EXPECT_EQ(foc::ErrorCode::Param, motor.start());
    EXPECT_EQ(foc::ErrorCode::Param, motor.status());
    EXPECT_FALSE(hal.pwm.enabled());
    EXPECT_EQ(foc::core::MotorState::Idle, motor.state());
}



// ---------------------------------------------------------------------------------------------
// Compliance knobs: the scales have to reach the slots, and an unreachable request must say so.
// 柔顺旋钮：缩放必须抵达槽位；抵达不了的请求必须明说。
// ---------------------------------------------------------------------------------------------

namespace {

/// [en] A pre-scaling controller: three arguments, no notion of gain scaling.
/// [zh] 缩放之前的控制器：三个参数，不认识增益缩放。
class PlainController final {
  public:
    [[nodiscard]] auto step(float setpoint, float feedback, float /*dt_s*/) noexcept -> float {
        return setpoint - feedback;
    }

    void reset() noexcept {}

    [[nodiscard]] auto status() const noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
};

using MotorWithPlainSpeed =
    foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, PlainController, foc::driver::StubHal,
                      foc::core::NullReferenceGenerator, foc::core::NullFeedforward, foc::core::PositionP>;

} // namespace



TEST(Motor, TheStiffnessScaleReachesThePositionSlot) {
    foc::driver::StubHal hal;
    make_hal(hal);

    MotorWithPosition motor(make_config(), make_position_policies(10.0F, 50.0F), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    hal.position.set_angle_mech_rad(0.0F);

    // 0.1 rad of error at gain 10 asks for 1 rad/s; the P speed loop turns that into 1 A.
    // 增益 10 下 0.1 rad 误差要 1 rad/s；比例速度环把它变成 1 A。
    motor.set_command(foc::motor::Command{0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F});
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    const foc::core::SvpwmResult stiff = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, 1.0F}, kVBus);
    EXPECT_NEAR(stiff.duty.u, hal.pwm.last_duty().u, 1e-6F);

    // Half the stiffness: half the current for the same error.
    // 刚度减半：同样误差下电流减半。
    motor.set_command(foc::motor::Command{0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.5F, 0.0F, 0.0F});
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    const foc::core::SvpwmResult soft = foc::core::Svpwm::modulate(foc::core::Ab<float>{0.0F, 0.5F}, kVBus);
    EXPECT_NEAR(soft.duty.u, hal.pwm.last_duty().u, 1e-6F);
}



TEST(Motor, AScalingRequestTheSlotCannotDeliverIsReportedNotIgnored) {
    // A speed controller without the scaling hook still fills the slot -- until somebody asks for
    // compliance. Then the answer is `NotImpl`, not a joint that pretends to be soft.
    // 没有缩放钩子的速度控制器仍然能占这个槽位——直到有人要求柔顺为止。那时答案是 `NotImpl`，
    // 而不是一个假装自己很软的关节。
    foc::driver::StubHal hal;
    make_hal(hal);
    hal.position.set_angle_mech_rad(0.0F);

    MotorWithPlainSpeed motor(make_config(),
                              MotorWithPlainSpeed::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}),
                                                            foc::core::Svpwm{}, PlainController{}, PlainController{},
                                                            PlainController{}, foc::core::NullReferenceGenerator{},
                                                            foc::core::NullFeedforward{},
                                                            foc::core::PositionP{{10.0F, 50.0F, 0.0F}}},
                              hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    // No scaling asked for: the plain slot works.
    // 没有请求缩放：朴素槽位正常工作。
    motor.set_command(foc::motor::Command{0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F});
    EXPECT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));

    // Compliance asked for: reported, and deliberately not latched (a missing capability is not a
    // fault the caller has to clear).
    // 请求柔顺：明说，且故意不锁存（缺能力不是调用方必须清除的故障）。
    motor.set_command(foc::motor::Command{0.1F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.5F});
    EXPECT_EQ(foc::ErrorCode::NotImpl, motor.tick(kDt));
    EXPECT_EQ(foc::core::MotorState::Run, motor.state()) << "not latched";
}
