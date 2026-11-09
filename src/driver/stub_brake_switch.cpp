// Conan::ImportStart
#include <cmath>

#include <foc/driver/stub_brake_switch.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_driver
 * @file stub_brake_switch.cpp
 *
 * @brief [en] Implements the brake-switch stub: it records the duty instead of driving it.
 * @brief [zh] 制动开关桩件的实现：记录占空比，而不是驱动它。
 * @since 0.1.0
 */

namespace foc {

namespace driver {

auto StubBrakeSwitch::configure(float freq_hz) noexcept -> ErrorCode {
    if (!(freq_hz > 0.0F) || !std::isfinite(freq_hz)) {
        return ErrorCode::Param;
    }
    freq_hz_ = freq_hz;
    return ErrorCode::Ok;
}

void StubBrakeSwitch::write_duty(float duty) noexcept {
    duty_ = duty;
    enabled_ = duty > 0.0F;
    ++writes_;
}

void StubBrakeSwitch::disable() noexcept {
    duty_ = 0.0F;
    enabled_ = false;
}

auto StubBrakeSwitch::duty() const noexcept -> float { return duty_; }

auto StubBrakeSwitch::enabled() const noexcept -> bool { return enabled_; }

auto StubBrakeSwitch::write_count() const noexcept -> std::uint32_t { return writes_; }

auto StubBrakeSwitch::freq_hz() const noexcept -> float { return freq_hz_; }

} // namespace driver

} // namespace foc
