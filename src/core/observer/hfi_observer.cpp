// Conan::ImportStart
#include <foc/core/observer/hfi_observer.hpp>

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
constexpr auto valid_config(const HfiObserver::Config& config) noexcept -> bool {
    return (config.inject_v > 0.0F) && (config.inject_hz > 0.0F) && (config.demod_lpf_hz > 0.0F) &&
           (config.demod_lpf_hz < config.inject_hz) && (config.pll_bandwidth_hz > 0.0F) && (config.pll_zeta > 0.0F) &&
           (config.demod_gain_a != 0.0F) && (config.min_hf_current_a >= 0.0F);
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

} // namespace

HfiObserver::HfiObserver(Config config) noexcept
    : config_(config), demod_lpf_a_(), demod_lpf_b_(), amplitude_lpf_(), state_{0.0F, 0.0F},
      status_(valid_config(config) ? ErrorCode::Ok : ErrorCode::Param), phase_applied_(0.0F), theta_applied_(0.0F),
      omega_int_(0.0F), error_(0.0F), hf_amp_(0.0F), dt_configured_(0.0F), primed_(false), locked_(false) {}

auto HfiObserver::predicted_gain_a(float inject_v, float inject_hz, float ld, float lq) noexcept -> float {
    // Derived from the linearised HF model: a voltage V_h along an axis that is `dtheta` away from d
    // lands partly on d and partly on q, and the two halves respond through Ld and Lq. The difference
    // between those responses, seen on the estimated q axis, is the error signal.
    // 由线性化高频模型导出：与 d 轴相差 `dtheta` 的轴上施加 V_h，会一部分落在 d、一部分落在 q，两半
    // 各自经 Ld 与 Lq 响应。二者之差投影到估计 q 轴上，就是误差信号。
    const float wh = kTwoPi<float> * inject_hz;
    const float denominator = 2.0F * wh * ld * lq;
    if (!(denominator > 0.0F) || !std::isfinite(denominator)) {
        return 0.0F;
    }
    return static_cast<float>(inject_v * (ld - lq) / denominator);
}

auto HfiObserver::configure_filters(float dt_s) noexcept -> ErrorCode {
    // Three cascaded one-poles, two on the error signal and one on the amplitude used by the lock
    // detector. Two stages on the error: the demodulated signal carries a ripple at twice the
    // injection frequency, and one pole at fc puts only ~20 dB between that ripple and the loop.
    // 三个一阶低通：误差信号两级、锁定判据的幅值一级。误差用两级的原因：解调信号带有两倍注入频率的
    // 纹波，单极点在 fc 处只提供约 20 dB 抑制。
    const ErrorCode a = demod_lpf_a_.configure(config_.demod_lpf_hz, dt_s);
    const ErrorCode b = demod_lpf_b_.configure(config_.demod_lpf_hz, dt_s);
    const ErrorCode c = amplitude_lpf_.configure(config_.demod_lpf_hz, dt_s);
    if ((a != ErrorCode::Ok) || (b != ErrorCode::Ok) || (c != ErrorCode::Ok)) {
        return ErrorCode::Param;
    }
    dt_configured_ = dt_s;
    return ErrorCode::Ok;
}

auto HfiObserver::update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState {
    // The contract hands the applied voltage over, but a saliency estimate must not depend on the
    // voltage *command*: that would make the angle inherit every current-loop error. The rotor angle
    // is in the current response alone.
    // 契约会把施加电压交过来，但凸极估计不能依赖电压**指令**：那会让角度继承电流环的一切误差。转子
    // 角度只在电流响应里。
    static_cast<void>(v_ab);

    if (!(dt_s > 0.0F)) {
        status_ = ErrorCode::Param;
        return state_;
    }

    if (status_ == ErrorCode::Param) {
        return state_;
    }

    // Reconfigure only when the period actually changed. The comparison is exact on purpose: the
    // period is either the same value or it is not, and a tolerance here would be a number nobody
    // could justify from the physics.
    // 仅在周期真的变了时重配。这里故意用精确比较：周期要么是同一个值、要么不是，加一个容差只会得到
    // 一个没人能从物理上解释的数字。
    if (dt_s != dt_configured_) {
        if (configure_filters(dt_s) != ErrorCode::Ok) {
            status_ = ErrorCode::Param;
            return state_;
        }
    }

    if (!primed_) {
        // Nothing has been applied yet, so there is no current response to demodulate; the first
        // step only establishes the phase and the frame the next sample belongs to.
        // 还没有施加任何东西，因此没有可解调的电流响应；第一步只确立下一个采样所属的相位与坐标系。
        phase_applied_ = 0.0F;
        theta_applied_ = state_.theta_e;
        primed_ = true;
        locked_ = false;
        phase_applied_ = wrap_two_pi(phase_applied_ + (kTwoPi<float> * config_.inject_hz * dt_s));
        return state_;
    }

    // Project the measured current on the frame that the voltage in effect was oriented with, and
    // mix with cos(w_h t): the current lags the injection by 90 degrees, so cos is the phase that
    // produces a DC term. Mixing with sin would average to zero.
    // 把实测电流投影到当时电压所用的坐标系上，再与 cos(w_h t) 相乘：电流滞后注入 90 度，故 cos 才是
    // 产生直流项的那个相位。用 sin 相乘会平均成零。
    const float c = std::cos(theta_applied_);
    const float s = std::sin(theta_applied_);
    const float i_along = (i_ab.alpha * c) + (i_ab.beta * s); // estimated d axis
    const float i_quad = (i_ab.beta * c) - (i_ab.alpha * s);  // estimated q axis
    const float carrier = std::cos(phase_applied_);

    const float along_demod = 2.0F * i_along * carrier;
    const float quad_demod = 2.0F * i_quad * carrier;

    hf_amp_ = std::fabs(amplitude_lpf_.step(along_demod));

    const float filtered = demod_lpf_b_.step(demod_lpf_a_.step(quad_demod));

    // Lock means "there is a HF current to read". Without it the loop would integrate noise into an
    // angle that looks like a measurement; holding the last angle and reporting zero speed is the
    // most honest thing an observer without a validity channel can do. The error signal is zeroed
    // for the same reason: an unlocked demodulated sample is noise, and a number that looks like a
    // measurement invites being used as one.
    // 锁定意味着“有高频电流可读”。没有它，环路会把噪声积成一个看起来像测量值的角度；保持上次角度并
    // 报零速，是一个没有有效性通道的观测器能做的最诚实的事。误差信号同样置零，理由相同：未锁定时的
    // 解调样本就是噪声，而一个看起来像测量值的数字会被当成测量值用。
    locked_ = hf_amp_ > config_.min_hf_current_a;

    if (locked_) {
        error_ = filtered / config_.demod_gain_a;
        const float wn = kTwoPi<float> * config_.pll_bandwidth_hz;
        const float kp = 2.0F * config_.pll_zeta * wn;
        const float ki = wn * wn;
        omega_int_ = static_cast<float>(omega_int_ + (ki * error_ * dt_s));
        const float omega = omega_int_ + (kp * error_);
        state_.omega_e = omega;
        state_.theta_e = wrap_two_pi(state_.theta_e + (omega * dt_s));
    } else {
        error_ = 0.0F;
        omega_int_ = 0.0F;
        state_.omega_e = 0.0F;
    }

    // The phase and the frame for the period that starts now: the same pair update() will demodulate
    // the next sample against, which is what makes the one-step delay consistent instead of an
    // unstated phase error.
    // 即将开始那一周期的相位与坐标系：下一次 update() 解调时用的就是同一对量，于是那一步延迟是自洽的
    // 相位关系，而不是一个没说出口的相位误差。
    phase_applied_ = wrap_two_pi(phase_applied_ + (kTwoPi<float> * config_.inject_hz * dt_s));
    theta_applied_ = state_.theta_e;
    return state_;
}

auto HfiObserver::injection_ab() const noexcept -> Ab<float> {
    if (status_ == ErrorCode::Param) {
        return Ab<float>{0.0F, 0.0F};
    }
    const float v = config_.inject_v * std::sin(phase_applied_);
    return Ab<float>{static_cast<float>(v * std::cos(theta_applied_)),
                     static_cast<float>(v * std::sin(theta_applied_))};
}

void HfiObserver::reset() noexcept {
    demod_lpf_a_.reset();
    demod_lpf_b_.reset();
    amplitude_lpf_.reset();
    state_ = ObsState{0.0F, 0.0F};
    phase_applied_ = 0.0F;
    theta_applied_ = 0.0F;
    omega_int_ = 0.0F;
    error_ = 0.0F;
    hf_amp_ = 0.0F;
    dt_configured_ = 0.0F;
    primed_ = false;
    locked_ = false;
    status_ = valid_config(config_) ? ErrorCode::Ok : ErrorCode::Param;
}

auto HfiObserver::status() const noexcept -> ErrorCode { return status_; }

auto HfiObserver::locked() const noexcept -> bool { return locked_; }

auto HfiObserver::error() const noexcept -> float { return error_; }

auto HfiObserver::state() const noexcept -> ObsState { return state_; }

auto HfiObserver::hf_current_a() const noexcept -> float { return hf_amp_; }

} // namespace core

} // namespace foc
