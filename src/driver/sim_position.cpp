// Conan::ImportStart
#include <foc/driver/sim_position.hpp>

#include <cmath>
// Conan::ImportEnd

namespace foc {

namespace driver {

void SimPosition::set_angle_mech_rad(float angle_rad) noexcept {
    const float two_pi = 6.28318530717958647692F;
    float wrapped = std::fmod(angle_rad, two_pi);
    if (wrapped < 0.0F) {
        wrapped += two_pi;
    }
    angle_rad_ = wrapped;
}

void SimPosition::set_velocity_rad_s(float velocity_rad_s) noexcept { velocity_rad_s_ = velocity_rad_s; }

auto SimPosition::angle_mech_rad() noexcept -> float { return angle_rad_; }

auto SimPosition::velocity_rad_s() noexcept -> float { return velocity_rad_s_; }

auto SimPosition::has_sensor() const noexcept -> bool { return sensor_present_; }

void SimPosition::set_sensor_present(bool present) noexcept { sensor_present_ = present; }

} // namespace driver

} // namespace foc
