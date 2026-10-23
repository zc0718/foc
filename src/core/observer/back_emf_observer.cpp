// Conan::ImportStart
#include <foc/core/observer/back_emf_observer.hpp>

#include <cmath>

#include <foc/core/transforms.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {

constexpr auto valid_config(const BackEmfObserver::Config& config) noexcept -> bool {
    return (config.rs >= 0.0F) && (config.l > 0.0F);
}

/// Wrap to [0, 2*pi).
auto wrap_two_pi(float angle) noexcept -> float {
    while (angle < 0.0F) {
        angle += kTwoPi<float>;
    }
    while (angle >= kTwoPi<float>) {
        angle -= kTwoPi<float>;
    }
    return angle;
}

/// Wrap to (-pi, pi].
auto wrap_pi(float angle) noexcept -> float {
    while (angle <= -kPi<float>) {
        angle += kTwoPi<float>;
    }
    while (angle > kPi<float>) {
        angle -= kTwoPi<float>;
    }
    return angle;
}

} // namespace

BackEmfObserver::BackEmfObserver(Config config) noexcept
    : config_(config), psi_{}, i_prev_{}, state_{}, status_(valid_config(config) ? ErrorCode::Ok : ErrorCode::Param),
      primed_(false), theta_primed_(false) {}

void BackEmfObserver::seed_flux(Ab<float> psi_ab) noexcept { psi_ = psi_ab; }

auto BackEmfObserver::update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState {
    if (!(dt_s > 0.0F)) {
        status_ = ErrorCode::Param;
        return state_;
    }

    if (status_ == ErrorCode::Param) {
        return state_;
    }

    // d(psi)/dt = v - Rs*i - L*di/dt, integrated with the L*di term taken analytically from the
    // current difference so no derivative approximation is needed.
    if (!primed_) {
        i_prev_ = i_ab;
        primed_ = true;
    }

    const float e_alpha = static_cast<float>(v_ab.alpha - (config_.rs * i_ab.alpha));
    const float e_beta = static_cast<float>(v_ab.beta - (config_.rs * i_ab.beta));

    psi_.alpha = static_cast<float>(psi_.alpha + (e_alpha * dt_s) - (config_.l * (i_ab.alpha - i_prev_.alpha)));
    psi_.beta = static_cast<float>(psi_.beta + (e_beta * dt_s) - (config_.l * (i_ab.beta - i_prev_.beta)));
    i_prev_ = i_ab;

    const float theta = wrap_two_pi(std::atan2(psi_.beta, psi_.alpha));
    const float omega = theta_primed_ ? static_cast<float>(wrap_pi(theta - state_.theta_e) / dt_s) : 0.0F;

    state_.theta_e = theta;
    state_.omega_e = omega;
    theta_primed_ = true;
    return state_;
}

void BackEmfObserver::reset() noexcept {
    psi_ = Ab<float>{};
    i_prev_ = Ab<float>{};
    state_ = ObsState{0.0F, 0.0F};
    primed_ = false;
    theta_primed_ = false;
    status_ = valid_config(config_) ? ErrorCode::Ok : ErrorCode::Param;
}

auto BackEmfObserver::status() const noexcept -> ErrorCode { return status_; }

auto BackEmfObserver::flux() const noexcept -> Ab<float> { return psi_; }

} // namespace core

} // namespace foc
