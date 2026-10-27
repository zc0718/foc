// Conan::ImportStart
#include <foc/core/observer/nonlinear_flux_observer.hpp>

#include <cmath>

#include <foc/core/transforms.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {


/**
 * @brief [en] Implements valid_config(); the contract is documented on its declaration.
 *        [zh] 实现 valid_config()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
constexpr auto valid_config(const NonlinearFluxObserver::Config& config) noexcept -> bool {
    return (config.rs >= 0.0F) && (config.l > 0.0F) && (config.psi_f > 0.0F) && (config.gain_ratio > 0.0F) &&
           (config.min_gain_rad_s > 0.0F) && (config.min_flux_wb > 0.0F);
}


/**
 * @brief [en] Implements wrap_two_pi(); the contract is documented on its declaration.
 *        [zh] 实现 wrap_two_pi()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto wrap_two_pi(float angle) noexcept -> float {
    while (angle < 0.0F) {
        angle += kTwoPi<float>;
    }
    while (angle >= kTwoPi<float>) {
        angle -= kTwoPi<float>;
    }
    return angle;
}


/**
 * @brief [en] Implements wrap_pi(); the contract is documented on its declaration.
 *        [zh] 实现 wrap_pi()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
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

NonlinearFluxObserver::NonlinearFluxObserver(Config config) noexcept
    : config_(config), psi_{}, i_prev_{}, state_{0.0F, 0.0F},
      status_(valid_config(config) ? ErrorCode::Ok : ErrorCode::Param), locked_(false), primed_(false),
      theta_primed_(false) {}

void NonlinearFluxObserver::seed_flux(Ab<float> psi_ab) noexcept { psi_ = psi_ab; }

void NonlinearFluxObserver::apply_params(const MachineParams& params) noexcept {
    config_.rs = params.rs;
    config_.l = params.lq;
    config_.psi_f = params.psi_f;
    status_ = valid_config(config_) ? ErrorCode::Ok : ErrorCode::Param;
}

auto NonlinearFluxObserver::update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState {
    if (!(dt_s > 0.0F)) {
        status_ = ErrorCode::Param;
        return state_;
    }

    if (status_ == ErrorCode::Param) {
        return state_;
    }

    if (!primed_) {
        i_prev_ = i_ab;
        primed_ = true;
    }

    // Step 1 -- the same prediction the open-loop observer makes: `d(psi)/dt = v - Rs*i - L*di/dt`, with the
    // L*di term taken analytically from the current difference.
    // 第一步——与开环观测器相同的预测：`d(psi)/dt = v − Rs·i − L·di/dt`，其中 L·di 项由电流差分解析给出。
    const float e_alpha = static_cast<float>(v_ab.alpha - (config_.rs * i_ab.alpha));
    const float e_beta = static_cast<float>(v_ab.beta - (config_.rs * i_ab.beta));
    psi_.alpha = static_cast<float>(psi_.alpha + (e_alpha * dt_s) - (config_.l * (i_ab.alpha - i_prev_.alpha)));
    psi_.beta = static_cast<float>(psi_.beta + (e_beta * dt_s) - (config_.l * (i_ab.beta - i_prev_.beta)));
    i_prev_ = i_ab;

    // Step 2 -- the term this observer exists for. `psi_` already holds the permanent-magnet flux estimate,
    // so `(psi_f^2 - |psi|^2) * psi` points back towards the circle of radius psi_f: zero on it, outwards
    // when it has shrunk, inwards when it has grown. Written in the plan's terms, eta == psi_ here because
    // the L*i contribution was removed incrementally by the step above.
    // 第二步——本观测器存在的那一项。`psi_` 已经是永磁磁链估计，于是 `(ψf² − |psi|²)·psi` 指回半径 ψf 的圆：
    // 在圆上为零、缩小时朝外、胀大时朝内。用计划的说法，这里的 eta 就是 psi_——L·i 的贡献已由上一步增量地减掉。
    const float magnitude_squared = (psi_.alpha * psi_.alpha) + (psi_.beta * psi_.beta);
    const float psi_f_squared = config_.psi_f * config_.psi_f;
    // The rate follows the estimated speed: it must stay well below the rotation it supports, or the radial
    // push and the rotation fight (see the header for the measured grid). The floor keeps it alive at a
    // standstill, where `omega_e` is zero but a wrong parameter still stretches the circle.
    // 速率跟随估计转速：它必须明显低于它支撑的旋转，否则径向推力会与旋转打架（网格实测见头文件）。
    // 下限保证静止时仍然生效——那时 `omega_e` 为零，但错误的参数照样会把那个圆拉歪。
    const float scheduled_gain = std::fmax(config_.min_gain_rad_s, config_.gain_ratio * std::fabs(state_.omega_e));
    const float scale = ((0.5F * scheduled_gain) / psi_f_squared) * (psi_f_squared - magnitude_squared);
    psi_.alpha = static_cast<float>(psi_.alpha + ((scale * psi_.alpha) * dt_s));
    psi_.beta = static_cast<float>(psi_.beta + ((scale * psi_.beta) * dt_s));

    const float magnitude = std::sqrt((psi_.alpha * psi_.alpha) + (psi_.beta * psi_.beta));
    locked_ = magnitude > config_.min_flux_wb;

    const float theta = wrap_two_pi(std::atan2(psi_.beta, psi_.alpha));
    const float omega = theta_primed_ ? static_cast<float>(wrap_pi(theta - state_.theta_e) / dt_s) : 0.0F;

    state_.theta_e = theta;
    state_.omega_e = omega;
    theta_primed_ = true;
    return state_;
}

void NonlinearFluxObserver::reset() noexcept {
    psi_ = Ab<float>{};
    i_prev_ = Ab<float>{};
    state_ = ObsState{0.0F, 0.0F};
    locked_ = false;
    primed_ = false;
    theta_primed_ = false;
    status_ = valid_config(config_) ? ErrorCode::Ok : ErrorCode::Param;
}

auto NonlinearFluxObserver::status() const noexcept -> ErrorCode { return status_; }

auto NonlinearFluxObserver::locked() const noexcept -> bool { return locked_; }

auto NonlinearFluxObserver::flux() const noexcept -> Ab<float> { return psi_; }

auto NonlinearFluxObserver::state() const noexcept -> ObsState { return state_; }

} // namespace core

} // namespace foc
