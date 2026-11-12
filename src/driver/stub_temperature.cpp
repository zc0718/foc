// Conan::ImportStart
#include <foc/driver/stub_temperature.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_driver
 * @file stub_temperature.cpp
 *
 * @brief [en] Implements the temperature stub: a settable reading that starts invalid.
 * @brief [zh] 温度桩件的实现：一个可设的读数，且初始为无效。
 * @since 0.1.0
 */

namespace foc {

namespace driver {

void StubTemperature::set_celsius(float celsius) noexcept {
    reading_.celsius = celsius;
    reading_.valid = true;
}

void StubTemperature::set_invalid() noexcept { reading_.valid = false; }

auto StubTemperature::read() noexcept -> hal::TemperatureReading { return reading_; }

} // namespace driver

} // namespace foc
