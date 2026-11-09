// Conan::ImportStart
#include <foc/driver/stub_dc_link.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_driver
 * @file stub_dc_link.cpp
 *
 * @brief [en] Implements the DC-link stub: a settable bus voltage, so a test can say what the bus is.
 * @brief [zh] 母线桩件的实现：母线电压可设，于是用例能直接指定母线是多少。
 * @since 0.1.0
 */

namespace foc {

namespace driver {

void StubDcLink::set_voltage_v(float voltage_v) noexcept { voltage_v_ = voltage_v; }

auto StubDcLink::voltage_v() noexcept -> float { return voltage_v_; }

} // namespace driver

} // namespace foc
