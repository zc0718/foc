// Conan::ImportStart
#include <foc/driver/stub_hal.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {


/**
 * @brief [en] Implements StubHal::reset(); the contract is documented on its declaration.
 *        [zh] 实现 StubHal::reset()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
void StubHal::reset() noexcept {
    pwm.reset();
    current_sense.reset();
    position.set_angle_mech_rad(0.0F);
    position.set_offset_mech_rad(0.0F);
    position.set_velocity_rad_s(0.0F);
    position.set_sensor_present(true);
    dc_link.set_voltage_v(0.0F);
}

} // namespace driver

} // namespace foc
