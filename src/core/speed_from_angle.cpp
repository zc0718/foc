// Conan::ImportStart
#include <foc/core/speed_from_angle.hpp>
// Conan::ImportEnd

#include <cmath>

namespace foc {

namespace core {

namespace {

constexpr float kPi = 3.14159265358979F;
constexpr float kTwoPi = 6.28318530717959F;

/** @brief [en] Wrap to (-pi, pi]. [zh] 折到 (-pi, pi]。 */
auto wrap_pi(float angle) noexcept -> float {
    while (angle <= -kPi) {
        angle += kTwoPi;
    }
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    return angle;
}

} // namespace

SpeedFromAngle::SpeedFromAngle(Config config) noexcept
    : config_(config), continuous_rad_(0.0F), previous_rad_(0.0F), speed_mech_rad_s_(0.0F), last_dt_s_(0.0F),
      filled_(0U), write_index_(0U), primed_(false), ready_(false), lost_count_(false) {
    if (config_.pole_pairs == 0U) {
        config_.pole_pairs = 1U;
    }
    if (config_.window_ticks < 2U) {
        config_.window_ticks = 2U;
    }
    if (config_.window_ticks > kMaxWindowTicks) {
        config_.window_ticks = kMaxWindowTicks;
    }
}

void SpeedFromAngle::reset() noexcept {
    continuous_rad_ = 0.0F;
    previous_rad_ = 0.0F;
    speed_mech_rad_s_ = 0.0F;
    last_dt_s_ = 0.0F;
    filled_ = 0U;
    write_index_ = 0U;
    primed_ = false;
    ready_ = false;
    lost_count_ = false;
}

void SpeedFromAngle::update(float theta_e_rad, float dt_s) noexcept {
    if (!(dt_s > 0.0F)) {
        return;
    }
    last_dt_s_ = dt_s;

    if (!primed_) {
        continuous_rad_ = theta_e_rad;
        previous_rad_ = theta_e_rad;
        primed_ = true;
    } else {
        const float delta = wrap_pi(theta_e_rad - previous_rad_);
        previous_rad_ = theta_e_rad;
        if (std::fabs(delta) >= (0.9F * kPi)) {
            // Half an electrical turn between samples cannot be unwrapped, so the count is reported as lost rather than
            // guessed -- the same rule as `PositionReference`, for the same reason.
            // 两次采样之间半个电角度无法解开折返，因此报"丢计数"而不是猜——与 `PositionReference` 同一条规矩、
            // 同一个理由。
            lost_count_ = true;
        }
        continuous_rad_ += delta;
    }

    buffer_[write_index_] = continuous_rad_;
    write_index_ = static_cast<std::uint16_t>((write_index_ + 1U) % config_.window_ticks);
    if (filled_ < config_.window_ticks) {
        ++filled_;
    }

    if (filled_ < config_.window_ticks) {
        return; // a difference across a partly-filled window would divide by the wrong interval
    }
    ready_ = true;
    const float older = buffer_[write_index_]; // the slot just overwritten is the oldest sample
    const float span_s = static_cast<float>(config_.window_ticks - 1U) * last_dt_s_;
    const float speed_electrical = (continuous_rad_ - older) / span_s;
    speed_mech_rad_s_ = speed_electrical / static_cast<float>(config_.pole_pairs);
}

} // namespace core

} // namespace foc
