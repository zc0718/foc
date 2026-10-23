// Conan::ImportStart
#include <foc/driver/stub_hal.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {

void StubHal::reset() noexcept {
    pwm.reset();
    current_sense.reset();
    position.set_angle_mech_rad(0.0F);
    position.set_velocity_rad_s(0.0F);
    position.set_sensor_present(true);
    dc_link.set_voltage_v(0.0F);
}

} // namespace driver

} // namespace foc
