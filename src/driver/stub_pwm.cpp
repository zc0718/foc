// Conan::ImportStart
#include <foc/driver/stub_pwm.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {

auto StubPwm::configure(float freq_hz, float deadtime_ns) noexcept -> ErrorCode {
    if (!(freq_hz > 0.0F) || !(deadtime_ns >= 0.0F)) {
        return ErrorCode::Param;
    }

    freq_hz_ = freq_hz;
    deadtime_ns_ = deadtime_ns;
    configured_ = true;
    return ErrorCode::Ok;
}

void StubPwm::write_duty(core::Uvw<float> duty) noexcept {
    duty_ = duty;
    ++writes_;
}

auto StubPwm::enable() noexcept -> ErrorCode {
    if (!configured_) {
        return ErrorCode::State;
    }

    enabled_ = true;
    return ErrorCode::Ok;
}

void StubPwm::disable() noexcept { enabled_ = false; }

auto StubPwm::fault() const noexcept -> bool { return fault_; }

void StubPwm::reset() noexcept {
    duty_ = core::Uvw<float>{};
    freq_hz_ = 0.0F;
    deadtime_ns_ = 0.0F;
    writes_ = 0;
    configured_ = false;
    enabled_ = false;
    fault_ = false;
}

void StubPwm::set_fault(bool active) noexcept { fault_ = active; }

auto StubPwm::last_duty() const noexcept -> core::Uvw<float> { return duty_; }

auto StubPwm::write_count() const noexcept -> std::uint32_t { return writes_; }

auto StubPwm::enabled() const noexcept -> bool { return enabled_; }

auto StubPwm::freq_hz() const noexcept -> float { return freq_hz_; }

} // namespace driver

} // namespace foc
