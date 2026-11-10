// Complete example: one motor, one control loop, no hardware.
//
// The same program runs against the SIL plant in the test package; here the reference stubs keep it
// short enough to read. Swapping StubHal for your board's HAL is the only change needed to run a
// real inverter -- every other line stays identical.
//
// The listing is included verbatim by tutorial.dox, so its comments stay in one language on
// purpose: gettext cannot filter a code block, and a bilingual comment would leak Chinese into the
// English page.

// Build it the way the `demo` gate does -- the example calls into the library, so the
// sources are part of the command, and `-Iinclude` alone fails at the link. A renamed
// source turns that gate red instead of leaving a stale instruction here:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude
//   docs/doxygen/dox/demos/servo-loop.cxx src/core/*.cpp
//   src/core/observer/*.cpp src/motor/*.cpp src/driver/*.cpp src/api/foc_api.c
//   src/api/bridge.cpp src/version.c
//   -o /tmp/servo-loop

#include <cstdio>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

int main() {
    // 1. the machine nameplate, SI units
    const foc::motor::MotorConfig config{
        0.5F,   // stator resistance, ohm
        4e-4F,  // d-axis inductance, H
        4e-4F,  // q-axis inductance, H
        0.05F,  // torque constant, N*m/A
        1e-5F,  // rotor inertia, kg*m^2
        1e-6F,  // viscous friction, N*m*s
        7U,     // pole pairs
        5.0F,   // per-axis current limit, A
        20000.0F, // carrier frequency, Hz
        500.0F    // dead time, ns
    };

    // 2. the policies: what is a template parameter is chosen here, once, at compile time
    using Motor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi,
                                    foc::driver::StubHal>;
    const auto current = foc::current_pi_config(config, 2000.0F, 24.0F);

    foc::driver::StubHal hal;  // on a board: your own HalPolicy
    Motor motor(config,
                Motor::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{},
                                foc::core::Pi{current}, foc::core::Pi{current},
                                foc::core::Pi{foc::speed_pi_config(config, 200.0F)}},
                hal);

    // 3. lifecycle: init() reads the nameplate, start() enables the output stage
    if (motor.init() != foc::ErrorCode::Ok) {
        return 1;
    }
    if (motor.start() != foc::ErrorCode::Ok) {
        return 1;
    }

    // 4. the control loop: call tick() from your PWM interrupt
    motor.set_torque(1.0F);  // q-axis current reference, A
    for (int i = 0; i < 1000; ++i) {
        if (motor.tick(1e-4F) != foc::ErrorCode::Ok) {
            break;
        }
    }

    // 5. read back what happened
    const foc::motor::Telemetry telemetry = motor.telemetry();
    std::printf("speed %.3f rad/s, id %.3f A, iq %.3f A\n",
                static_cast<double>(telemetry.speed_mech_rad_s),
                static_cast<double>(telemetry.id_a),
                static_cast<double>(telemetry.iq_a));

    motor.stop();
    return 0;
}
