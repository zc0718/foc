// Conan::ImportStart
#include <foc/core/filters.hpp>

#include <foc/core/transforms.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

auto FirstOrderLowPass::configure(float cutoff_hz, float dt_s) noexcept -> ErrorCode {
    if (!(dt_s > 0.0F) || !(cutoff_hz > 0.0F)) {
        configured_ = false;
        return ErrorCode::Param;
    }

    if (!(cutoff_hz < (static_cast<float>(1) / (static_cast<float>(2) * dt_s)))) {
        configured_ = false;
        return ErrorCode::Param;
    }

    const float two_pi_fc_dt = static_cast<float>(2) * kPi<float> * cutoff_hz * dt_s;
    alpha_ = two_pi_fc_dt / (static_cast<float>(1) + two_pi_fc_dt);
    configured_ = true;
    primed_ = false;
    return ErrorCode::Ok;
}

auto FirstOrderLowPass::step(float input) noexcept -> float {
    if (!configured_) {
        return input;
    }

    if (!primed_) {
        state_ = input;
        primed_ = true;
        return state_;
    }

    state_ += static_cast<float>(alpha_ * (input - state_));
    return state_;
}

void FirstOrderLowPass::reset() noexcept {
    state_ = 0.0F;
    primed_ = false;
}

auto FirstOrderLowPass::configured() const noexcept -> bool { return configured_; }

} // namespace core

} // namespace foc
