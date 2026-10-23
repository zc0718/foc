// Conan::ImportStart
#include <foc/driver/stub_dc_link.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {

void StubDcLink::set_voltage_v(float voltage_v) noexcept { voltage_v_ = voltage_v; }

auto StubDcLink::voltage_v() noexcept -> float { return voltage_v_; }

} // namespace driver

} // namespace foc
