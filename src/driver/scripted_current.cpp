// Conan::ImportStart
#include <foc/driver/scripted_current.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {

auto ScriptedCurrent::load(const core::Uvw<float>* samples, std::size_t count) noexcept -> ErrorCode {
    if ((samples == nullptr) || (count == 0U) || (count > kCapacity)) {
        return ErrorCode::Param;
    }

    for (std::size_t i = 0; i < count; ++i) {
        samples_[i] = samples[i];
    }
    count_ = count;
    index_ = 0U;
    calibrated_ = false;
    return ErrorCode::Ok;
}

auto ScriptedCurrent::read_uvw() noexcept -> core::Uvw<float> {
    if (count_ == 0U) {
        return core::Uvw<float>{};
    }

    const core::Uvw<float> sample = samples_[index_];
    if ((index_ + 1U) < count_) {
        ++index_;
    }
    return sample;
}

auto ScriptedCurrent::calibrate_offset() noexcept -> ErrorCode {
    if (count_ == 0U) {
        return ErrorCode::State;
    }

    calibrated_ = true;
    return ErrorCode::Ok;
}

auto ScriptedCurrent::sample_rate_hz() const noexcept -> float { return rate_hz_; }

void ScriptedCurrent::set_sample_rate_hz(float rate_hz) noexcept { rate_hz_ = rate_hz; }

void ScriptedCurrent::reset() noexcept {
    for (auto& sample : samples_) {
        sample = core::Uvw<float>{};
    }
    count_ = 0U;
    index_ = 0U;
    rate_hz_ = 0.0F;
    calibrated_ = false;
}

auto ScriptedCurrent::exhausted() const noexcept -> bool { return (count_ != 0U) && ((index_ + 1U) >= count_); }

auto ScriptedCurrent::calibrated() const noexcept -> bool { return calibrated_; }

} // namespace driver

} // namespace foc
