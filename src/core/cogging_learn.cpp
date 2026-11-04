// Conan::ImportStart
#include <foc/core/cogging_learn.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {

constexpr float kTwoPi = 6.28318530718F;

} // namespace

auto CoggingLearner::validate() const noexcept -> ErrorCode {
    const MachineParams& m = config_.params;
    if (!std::isfinite(m.psi_f) || !(m.ld > 0.0F) || !(m.lq > 0.0F) || (m.psi_f < 0.0F)) {
        return ErrorCode::Param;
    }
    if (config_.pole_pairs == 0U) {
        return ErrorCode::Param;
    }
    if ((config_.count <= 0) || (config_.count > 6) || (config_.revolutions < 1)) {
        return ErrorCode::Param;
    }
    for (int k = 0; k < config_.count; ++k) {
        // Order 0 is a constant torque, which no compensator built from this table could cancel in phase -- and which is a
        // load, not cogging.
        // 0 次是恒定转矩，用这张表造出的补偿器无法按相位抵消它——而它是负载，不是齿槽。
        if (config_.orders[k] == 0U) {
            return ErrorCode::Param;
        }
    }
    const float low = config_.min_speed_rad_s;
    const float high = config_.max_speed_rad_s;
    if (!std::isfinite(low) || !std::isfinite(high) || (low < 0.0F) || !(high > low)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.speed_tolerance) || !(config_.speed_tolerance > 0.0F)) {
        return ErrorCode::Param;
    }
    return ErrorCode::Ok;
}

auto CoggingLearner::reset() noexcept -> void {
    for (int k = 0; k < 6; ++k) {
        re_[k] = 0.0F;
        im_[k] = 0.0F;
    }
    theta_first_ = 0.0F;
    theta_last_ = 0.0F;
    omega_min_ = 0.0F;
    omega_max_ = 0.0F;
    id_sum_ = 0.0F;
    samples_ = 0;
    saturated_ = false;
    started_ = false;
}

auto CoggingLearner::accumulate(float theta_mech_rad, float iq_a, float id_a, float omega_mech_rad_s,
                                bool saturated) noexcept -> void {
    if (!std::isfinite(theta_mech_rad) || !std::isfinite(iq_a) || !std::isfinite(id_a) ||
        !std::isfinite(omega_mech_rad_s)) {
        return;
    }
    const float speed = std::fabs(omega_mech_rad_s);
    if (!started_) {
        theta_first_ = theta_mech_rad;
        omega_min_ = speed;
        omega_max_ = speed;
        started_ = true;
    }
    theta_last_ = theta_mech_rad;
    omega_min_ = std::fmin(omega_min_, speed);
    omega_max_ = std::fmax(omega_max_, speed);
    saturated_ = saturated_ || saturated;
    id_sum_ = static_cast<float>(id_sum_ + id_a);
    for (int k = 0; k < config_.count; ++k) {
        const float angle = static_cast<float>(config_.orders[k]) * theta_mech_rad;
        re_[k] = static_cast<float>(re_[k] + (iq_a * std::cos(angle)));
        im_[k] = static_cast<float>(im_[k] + (iq_a * std::sin(angle)));
    }
    ++samples_;
}

auto CoggingLearner::revolutions() const noexcept -> float {
    if (samples_ < 2) {
        return 0.0F;
    }
    return std::fabs(theta_last_ - theta_first_) / kTwoPi;
}

auto CoggingLearner::ready() const noexcept -> bool {
    return (samples_ >= 2) && (revolutions() >= static_cast<float>(config_.revolutions));
}

auto CoggingLearner::finish(CoggingCompHarmonic* out, int capacity) const noexcept -> CoggingLearnReport {
    CoggingLearnReport report{ErrorCode::Param, 0, revolutions(), omega_min_, omega_max_};
    if (validate() != ErrorCode::Ok) {
        return report;
    }
    if (samples_ < 2) {
        report.status = ErrorCode::State;
        return report;
    }
    // The three refusals that are about the run rather than the answer, each with its own reason.
    // 三条关于"运行"而不是关于"答案"的拒绝，各有自己的理由。
    if (revolutions() < static_cast<float>(config_.revolutions)) {
        report.status =
            ErrorCode::State; // not enough revolutions to separate a harmonic from the DC it rides on 转数不够
        return report;
    }
    if (saturated_) {
        report.status = ErrorCode::State; // the loop was railing, not answering 环路在撞限，不是在回答
        return report;
    }
    if ((omega_max_ < config_.min_speed_rad_s) || (omega_min_ > config_.max_speed_rad_s)) {
        report.status = ErrorCode::State; // outside the band where the loop is faithful 不在环路可信的带内
        return report;
    }
    const float spread = (omega_max_ > 0.0F) ? ((omega_max_ - omega_min_) / omega_max_) : 0.0F;
    if (spread > config_.speed_tolerance) {
        report.status = ErrorCode::State; // a projection assumes the angle advances uniformly 投影假设角度均匀推进
        return report;
    }
    if ((out == nullptr) || (capacity < config_.count)) {
        report.status = ErrorCode::State; // nowhere to put the answer 没地方放答案
        return report;
    }

    const float mean_id = id_sum_ / static_cast<float>(samples_);
    const float kt_eff = 1.5F * static_cast<float>(config_.pole_pairs) *
                         (config_.params.psi_f + ((config_.params.ld - config_.params.lq) * mean_id));
    const auto normalise = 2.0F / static_cast<float>(samples_);
    for (int k = 0; k < config_.count; ++k) {
        const float re = re_[k] * normalise;
        const float im = im_[k] * normalise;
        const float amplitude_current = std::hypot(re, im);
        // `y = A*sin(k*theta + phi)` projects to `re = A*sin(phi)`, `im = A*cos(phi)`, so the phase is `atan2(re, im)`.
        // `y = A·sin(k·θ + φ)` 投影得到 `re = A·sin(φ)`、`im = A·cos(φ)`，所以相位是 `atan2(re, im)`。
        float phase = static_cast<float>(std::atan2(re, im) + 3.14159265358979323846F);
        // Wrapped into (-pi, pi] so two tables learned from the same machine print the same number.
        // 归一化到 (−π, π]，好让同一台机器学出的两张表打印出同一个数。
        while (phase > 3.14159265358979323846F) {
            phase = static_cast<float>(phase - kTwoPi);
        }
        // The loop supplies the current that makes `-T_cog`, i.e. `iq = (A/kt)*sin(k*theta + phi_inj + pi)`. Multiplying by `kt`
        // and shifting the phase by `pi` returns `A*sin(k*theta + phi_inj)` -- the torque the machine felt. Expressing it that way
        // rather than as a negative amplitude matters: "peak of this harmonic" has to be a positive number, and a negative
        // amplitude is a sign error waiting to be read as a magnitude.
        // 环路供给的是产生 `−T_cog` 的电流，即 `iq = (A/kt)·sin(k·θ + φ_inj + π)`。乘上 `kt` 再把相位平移 `π` 就回到
        // `A·sin(k·θ + φ_inj)`——机器实际感受到的转矩。写成这样而不是用负幅值是要紧的：「该谐波的峰值」必须是个正数，而负幅值
        // 就是一个等着被读成量值的符号错误。
        out[k] = CoggingCompHarmonic{config_.orders[k], kt_eff * amplitude_current, phase};
    }
    report.status = ErrorCode::Ok;
    report.count = config_.count;
    return report;
}

} // namespace core

} // namespace foc
