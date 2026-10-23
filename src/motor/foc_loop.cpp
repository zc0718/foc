// Conan::ImportStart
#include <foc/motor/foc_loop.hpp>

#include <foc/core/transforms.hpp>
// Conan::ImportEnd

namespace foc {

namespace motor {

auto measure_dq(Uvw<float> i_uvw, float theta_e_rad) noexcept -> Dq<float> {
    return core::park(core::clarke(i_uvw), theta_e_rad);
}

auto apply_voltage(Dq<float> v_dq, float theta_e_rad, float v_bus) noexcept -> core::SvpwmResult {
    return core::Svpwm::modulate(core::inv_park(v_dq, theta_e_rad), v_bus);
}

} // namespace motor

} // namespace foc
