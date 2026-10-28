// Conan::ImportStart
#include <foc/driver/stub_pwm.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {


/**
 * @brief [en] Implements StubPwm::configure(); the contract is documented on its declaration.
 * @brief [zh] 实现 StubPwm::configure()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto StubPwm::configure(float freq_hz, float deadtime_ns) noexcept -> ErrorCode {
    if (!(freq_hz > 0.0F) || !(deadtime_ns >= 0.0F)) {
        return ErrorCode::Param;
    }

    freq_hz_ = freq_hz;
    deadtime_ns_ = deadtime_ns;
    configured_ = true;
    return ErrorCode::Ok;
}


/**
 * @brief [en] Implements StubPwm::write_duty(); the contract is documented on its declaration.
 * @brief [zh] 实现 StubPwm::write_duty()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
void StubPwm::write_duty(core::Uvw<float> duty) noexcept {
    duty_ = duty;
    history_[history_next_] = duty;
    history_next_ = (history_next_ + 1U) % kHistoryCapacity;
    if (history_count_ < kHistoryCapacity) {
        ++history_count_;
    }
    ++writes_;
}


/**
 * @brief [en] Implements StubPwm::enable(); the contract is documented on its declaration.
 * @brief [zh] 实现 StubPwm::enable()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto StubPwm::enable() noexcept -> ErrorCode {
    if (!configured_) {
        return ErrorCode::State;
    }

    enabled_ = true;
    return ErrorCode::Ok;
}

void StubPwm::disable() noexcept { enabled_ = false; }

auto StubPwm::fault() const noexcept -> bool { return fault_; }


/**
 * @brief [en] Implements StubPwm::reset(); the contract is documented on its declaration.
 * @brief [zh] 实现 StubPwm::reset()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
void StubPwm::reset() noexcept {
    duty_ = core::Uvw<float>{};
    history_ = {};
    history_count_ = 0U;
    history_next_ = 0U;
    freq_hz_ = 0.0F;
    deadtime_ns_ = 0.0F;
    writes_ = 0;
    configured_ = false;
    enabled_ = false;
    fault_ = false;
}

void StubPwm::set_fault(bool active) noexcept { fault_ = active; }

auto StubPwm::last_duty() const noexcept -> core::Uvw<float> { return duty_; }


/**
 * @brief [en] Implements StubPwm::duty_history(); the contract is documented on its declaration.
 * @brief [zh] 实现 StubPwm::duty_history()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto StubPwm::duty_history(std::size_t back) const noexcept -> core::Uvw<float> {
    if (back >= history_count_) {
        return core::Uvw<float>{};
    }
    // history_next_ points at the slot the next write will use, so the newest one sits behind it.
    const std::size_t index = (history_next_ + kHistoryCapacity - 1U - back) % kHistoryCapacity;
    return history_[index];
}


auto StubPwm::history_count() const noexcept -> std::size_t { return history_count_; }

auto StubPwm::write_count() const noexcept -> std::uint32_t { return writes_; }

auto StubPwm::enabled() const noexcept -> bool { return enabled_; }

auto StubPwm::freq_hz() const noexcept -> float { return freq_hz_; }

} // namespace driver

} // namespace foc
