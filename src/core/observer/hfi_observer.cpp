// Conan::ImportStart
#include <foc/core/observer/hfi_observer.hpp>

#include <cmath>

#include <foc/core/limits.hpp>
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
    : config_(config), current_hp_alpha_(), current_hp_beta_(), demod_lpf_a_(), demod_lpf_b_(), amplitude_lpf_(),
      state_{0.0F, 0.0F}, status_(valid_config(config) ? ErrorCode::Ok : ErrorCode::Param), phase_applied_(0.0F),
      theta_applied_(0.0F), omega_int_(0.0F), error_(0.0F), hf_amp_(0.0F), dt_configured_(0.0F),
      // Ten time constants of the demodulation filter: the residual of a step response is e^-10, about
      // 5e-5. Integrating sooner reads the carrier switching on as signal, which at the designed loop
      // gain throws the estimate across the whole speed range in a few hundred microseconds.
      // 解调滤波器的十个时间常数：阶跃响应的残余是 e^-10，约 5e-5。更早积分就是把"载波刚加上"读成
      // 信号，在设计的环路增益下几百微秒内就能把估计甩过整个速度范围。
      blank_s_(10.0F / (kTwoPi<float> * config.demod_lpf_hz)),
      blank_left_s_(10.0F / (kTwoPi<float> * config.demod_lpf_hz)), primed_(false), no_signal_s_(0.0F), locked_(false) {
}

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
    // Five one-poles: two high-pass the current (one per stationary axis), two on the error signal and
    // one on the amplitude the lock detector uses. Two stages on the error because the demodulated
    // signal carries a ripple at twice the injection frequency, and a single pole at fc leaves only
    // ~20 dB between that ripple and the loop.
    // 五个一阶：两个对电流做高通（静止系每轴一个）、两个在误差信号上、一个在锁定判据的幅值上。误差用
    // 两级，因为解调信号带有两倍注入频率的纹波，单极点在 fc 处只有约 20 dB 抑制。
    const ErrorCode alpha_hp = current_hp_alpha_.configure(config_.current_hp_hz, dt_s);
    const ErrorCode beta_hp = current_hp_beta_.configure(config_.current_hp_hz, dt_s);
    const ErrorCode a = demod_lpf_a_.configure(config_.demod_lpf_hz, dt_s);
    const ErrorCode b = demod_lpf_b_.configure(config_.demod_lpf_hz, dt_s);
    const ErrorCode c = amplitude_lpf_.configure(config_.demod_lpf_hz, dt_s);
    if ((alpha_hp != ErrorCode::Ok) || (beta_hp != ErrorCode::Ok) || (a != ErrorCode::Ok) || (b != ErrorCode::Ok) ||
        (c != ErrorCode::Ok)) {
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
    // Blanking is re-armed every time the carrier *comes back*, not only at construction: a caller that
    // winds the injection down (a blending observer does) brings it up again later, and that edge looks
    // exactly like the first one -- a step of current into filters that have been sitting on nothing.
    // 屏蔽在载波**回来**的每一次都重新武装，而不只在构造时：把注入收掉的调用方（混合观测器就是这样）
    // 之后会再把它加回来，而那个边沿与第一次完全相同——一记电流阶跃打进一直没东西可看的滤波器里。
    // Blanking is re-armed when the carrier comes back *after having been gone long enough for the
    // filters to have lost their state* -- not on every threshold crossing. Both wrong versions were
    // tried and both broke the loop: re-arming on the blanked verdict (false throughout blanking) never
    // started it at all, and re-arming on the raw edge made it re-arm forever once a caller scaled the
    // carrier down until it hovered around the threshold. The physical question is "was it really off",
    // so the condition is how long it has been off.
    // 屏蔽在载波"消失足够久、滤波器状态已丢"之后回来时重新武装——不是每次过阈值都武装。两个错误版本都试过
    // 且都把环路弄坏了：按屏蔽后的结论武装（屏蔽期间恒为假）等于永不启动；按原始边沿武装，则在调用方把载波
    // 缩到阈值附近来回时永远重新武装。物理上的问题是"它真的关过吗"，所以条件就是关掉多久。
    const bool has_signal = hf_amp_ > config_.min_hf_current_a;
    no_signal_s_ = has_signal ? 0.0F : static_cast<float>(no_signal_s_ + dt_s);
    if (has_signal && (no_signal_s_ >= blank_s_)) {
        blank_left_s_ = blank_s_;
        no_signal_s_ = 0.0F;
    }
    blank_left_s_ = (blank_left_s_ > dt_s) ? static_cast<float>(blank_left_s_ - dt_s) : 0.0F;
    locked_ = has_signal && (blank_left_s_ <= 0.0F);

    if (locked_) {
        // Clamped to the range the normalisation itself defines: `demod_gain_a` is the amplitude at
        // 90 degrees of error, so anything beyond +-1 is not a measurement of sin(2*dtheta) -- it is
        // a transient (the carrier switching on, a load step) being read as signal. Without this the
        // start-up transient alone produced errors around 4, which at the designed loop gain kicked
        // the estimate straight over the pi/2 boundary and parked it on the mirror.
        // 钳在归一化本身定义的范围内：`demod_gain_a` 就是误差 90 度时的幅值，因此超出 ±1 的值不是
        // sin(2·dtheta) 的测量值，而是把暂态（载波刚加上、负载阶跃）当信号读。没有这一条，仅起动暂态
        // 就会产生约 4 的误差，在设计的环路增益下把估计直接踢过 π/2 边界、停到镜像上。
        error_ = core::clamp(filtered / config_.demod_gain_a, -1.0F, 1.0F);
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
    current_hp_alpha_.reset();
    current_hp_beta_.reset();
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
    blank_left_s_ = blank_s_;
    primed_ = false;
    no_signal_s_ = 0.0F;
    locked_ = false;
    status_ = valid_config(config_) ? ErrorCode::Ok : ErrorCode::Param;
}

auto HfiObserver::status() const noexcept -> ErrorCode { return status_; }

auto HfiObserver::locked() const noexcept -> bool { return locked_; }

auto HfiObserver::error() const noexcept -> float { return error_; }

auto HfiObserver::state() const noexcept -> ObsState { return state_; }

void HfiObserver::seed_angle(float theta_e, float omega_e) noexcept {
    state_.theta_e = wrap_two_pi(theta_e);
    state_.omega_e = omega_e;
    omega_int_ = omega_e;
    theta_applied_ = state_.theta_e;
}

void HfiObserver::apply_params(const MachineParams& params) noexcept {
    config_.demod_gain_a = predicted_gain_a(config_.inject_v, config_.inject_hz, params.ld, params.lq);
}

void HfiObserver::apply_polarity(bool reversed) noexcept {
    if (!reversed) {
        return;
    }
    // Both the published estimate and the frame the next injection is oriented with, so the very next
    // period already pulses along the corrected axis.
    // 同时改已发布的估计与下一次注入所用的坐标系，于是紧接着的那个周期就沿修正后的轴注入。
    state_.theta_e = wrap_two_pi(state_.theta_e + kPi<float>);
    theta_applied_ = state_.theta_e;
}

auto HfiObserver::hf_current_a() const noexcept -> float { return hf_amp_; }

} // namespace core

} // namespace foc
