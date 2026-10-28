// Conan::ImportStart
#include <foc/driver/sim_position.hpp>

#include <cmath>
// Conan::ImportEnd

namespace foc {

namespace driver {


/**
 * @brief [en] Implements SimPosition::set_angle_mech_rad(); the contract is documented on its declaration.
 * @brief [zh] 实现 SimPosition::set_angle_mech_rad()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
void SimPosition::set_angle_mech_rad(float angle_rad) noexcept {
    const float two_pi = 6.28318530717958647692F;
    float wrapped = std::fmod(angle_rad, two_pi);
    if (wrapped < 0.0F) {
        wrapped += two_pi;
    }
    angle_rad_ = wrapped;
}

void SimPosition::set_offset_mech_rad(float offset_rad) noexcept {
    const float two_pi = 6.28318530717958647692F;
    float wrapped = std::fmod(offset_rad, two_pi);
    if (wrapped < 0.0F) {
        wrapped += two_pi;
    }
    offset_rad_ = wrapped;
}

auto SimPosition::mech_offset_rad() const noexcept -> float { return offset_rad_; }

void SimPosition::set_velocity_rad_s(float velocity_rad_s) noexcept { velocity_rad_s_ = velocity_rad_s; }

auto SimPosition::angle_mech_rad() noexcept -> float {
    const float two_pi = 6.28318530717958647692F;
    float corrected = std::fmod(angle_rad_ - offset_rad_, two_pi);
    if (corrected < 0.0F) {
        corrected += two_pi;
    }
    return corrected;
}

auto SimPosition::velocity_rad_s() noexcept -> float { return velocity_rad_s_; }

auto SimPosition::has_sensor() const noexcept -> bool { return sensor_present_; }

void SimPosition::set_sensor_present(bool present) noexcept { sensor_present_ = present; }

} // namespace driver

} // namespace foc
