// Conan::ImportStart
#include <foc/core/limits.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

SlewLimiter::SlewLimiter(float rate_per_s) noexcept : rate_per_s_(rate_per_s), value_(0.0F) {}

auto SlewLimiter::step(float target, float dt_s) noexcept -> float {
    if (dt_s > 0.0F) {
        const float max_delta = rate_per_s_ * dt_s;
        const float delta = target - value_;
        if (delta > max_delta) {
            value_ += max_delta;
        } else if (delta < -max_delta) {
            value_ -= max_delta;
        } else {
            value_ = target;
        }
    }
    return value_;
}

void SlewLimiter::reset(float value) noexcept { value_ = value; }

auto SlewLimiter::value() const noexcept -> float { return value_; }

} // namespace core

} // namespace foc
