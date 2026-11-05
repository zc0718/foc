// Conan::ImportStart
#include <foc/core/auto_tune.hpp>
// Conan::ImportEnd

#include <cmath>

namespace foc {

namespace core {

namespace {

constexpr double kSingular = 1.0e-12;

// Solve the symmetric 3x3 normal equations by Cramer's rule. Order is `[iq, omega, 1]`, so the intercept is the third
// parameter and an unknown constant load has somewhere to go that is not the slope on the current.
// 用克拉默法则解对称的 3×3 正规方程。顺序是 `[iq, ω, 1]`，所以截距是第三个参数，而一个未知的恒定负载有地方可去，且那里不是对
// 电流的斜率。
struct Fit final {
    bool ok;
    double a_iq;   // d(alpha)/d(iq) = kt/J
    double a_w;    // d(alpha)/d(omega) = -b/J
    double a_zero; // the intercept: an unknown constant torque divided by J
};

auto solve(const double m[9]) -> double {
    return (m[0] * ((m[4] * m[8]) - (m[5] * m[7]))) - (m[1] * ((m[3] * m[8]) - (m[5] * m[6]))) +
           (m[2] * ((m[3] * m[7]) - (m[4] * m[6])));
}

} // namespace

auto SpeedTuner::validate() const noexcept -> ErrorCode {
    if (!std::isfinite(config_.kt_nm_per_a) || !(config_.kt_nm_per_a > 0.0F)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.j_expected_kg_m2) || !(config_.j_expected_kg_m2 > 0.0F)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.b_expected_nm_s) || (config_.b_expected_nm_s < 0.0F)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.bandwidth_rad_s) || !(config_.bandwidth_rad_s > 0.0F)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.i_max_a) || !(config_.i_max_a > 0.0F)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.tolerance) || !(config_.tolerance > 0.0F)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.min_current_a) || (config_.min_current_a < 0.0F)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.max_residual) || !(config_.max_residual > 0.0F)) {
        return ErrorCode::Param;
    }
    if (config_.min_samples < 3) {
        return ErrorCode::Param; // three parameters need three points at the very least 三个参数至少需要三个点
    }
    return ErrorCode::Ok;
}

auto SpeedTuner::reset() noexcept -> void {
    s_ii_ = 0.0;
    s_iw_ = 0.0;
    s_ww_ = 0.0;
    s_ia_ = 0.0;
    s_wa_ = 0.0;
    s_a_ = 0.0;
    s_aa_ = 0.0;
    s_i_ = 0.0;
    s_w_ = 0.0;
    previous_iq_ = 0.0F;
    previous_omega_ = 0.0F;
    previous_has_ = false;
    iq_min_ = 0.0F;
    iq_max_ = 0.0F;
    saturated_ = false;
    samples_ = 0;
}

auto SpeedTuner::samples() const noexcept -> int { return samples_; }

auto SpeedTuner::accumulate(float dt_s, float iq_a, float omega_mech_rad_s, bool saturated) noexcept -> void {
    if (!(dt_s > 0.0F) || !std::isfinite(dt_s) || !std::isfinite(iq_a) || !std::isfinite(omega_mech_rad_s)) {
        return;
    }
    // The acceleration is formed from the *previous* speed, and it is paired with the previous current and speed, so every
    // term of the fit describes the same interval. Pairing the new acceleration with the new current would attribute one
    // period's cause to the next period's effect.
    // 加速度由**上一个**转速构成，并与上一个电流、上一个转速配对，于是拟合的每一项描述的都是同一个区间。把新加速度与新电流配对，
    // 等于把这一周期的原因归给下一周期的结果。
    if (previous_has_) {
        const double alpha =
            (static_cast<double>(omega_mech_rad_s) - static_cast<double>(previous_omega_)) / static_cast<double>(dt_s);
        const double i = static_cast<double>(previous_iq_);
        const double w = static_cast<double>(previous_omega_);
        s_ii_ += i * i;
        s_iw_ += i * w;
        s_ww_ += w * w;
        s_ia_ += i * alpha;
        s_wa_ += w * alpha;
        s_a_ += alpha;
        s_aa_ += alpha * alpha;
        s_i_ += i;
        s_w_ += w;
        iq_min_ = (samples_ == 0) ? previous_iq_ : std::fmin(iq_min_, previous_iq_);
        iq_max_ = (samples_ == 0) ? previous_iq_ : std::fmax(iq_max_, previous_iq_);
        ++samples_;
    }
    previous_iq_ = iq_a;
    previous_omega_ = omega_mech_rad_s;
    previous_has_ = true;
    saturated_ = saturated_ || saturated;
}

auto SpeedTuner::nameplate_gains() const noexcept -> SpeedTuneResult {
    // The same expression, in the same order, as `speed_pi_config()` -- so a caller that applies an invalid result applies
    // exactly the gains it had, not a re-computation that happens to be close.
    // 与 `speed_pi_config()` 同一个表达式、同一个运算顺序——于是施加了无效结果的调用方施加的正是它原有的增益，而不是一次"碰巧很接近"
    // 的重算。
    const float kp = config_.bandwidth_rad_s * config_.j_expected_kg_m2 / config_.kt_nm_per_a;
    const float ki = config_.bandwidth_rad_s * config_.b_expected_nm_s / config_.kt_nm_per_a;
    return SpeedTuneResult{ErrorCode::Ok, false, 0.0F, 0.0F, 0.0F, kp, ki, 0.0F, "", samples_};
}

auto SpeedTuner::finish() const noexcept -> SpeedTuneResult {
    if (validate() != ErrorCode::Ok) {
        SpeedTuneResult bad = nameplate_gains();
        bad.status = ErrorCode::Param;
        bad.reason = "the configuration cannot be evaluated";
        return bad;
    }
    SpeedTuneResult refused = nameplate_gains();
    if (samples_ < config_.min_samples) {
        refused.reason = "the window is shorter than min_samples";
        return refused;
    }
    if (saturated_) {
        refused.reason = "a period in the window was saturated, so the loop was not delivering the reference";
        return refused;
    }
    const float spread = iq_max_ - iq_min_;
    if (std::fmax(std::fabs(iq_max_), std::fabs(iq_min_)) < config_.min_current_a) {
        refused.reason = "the current never reached min_current_a, so the window carries no information";
        return refused;
    }
    if (spread < 2.0F * config_.min_current_a) {
        refused.reason = "the current did not vary enough for a slope to be measured";
        return refused;
    }

    // Normal equations for `alpha = a*iq + d*w + c`, in `[iq, w, 1]` order.
    // `alpha = a·iq + d·w + c` 的正规方程，按 `[iq, w, 1]` 的顺序。
    const double n = static_cast<double>(samples_);
    const double m[9] = {s_ii_, s_iw_, s_i_, s_iw_, s_ww_, s_w_, s_i_, s_w_, n};
    const double rhs[3] = {s_ia_, s_wa_, s_a_};
    const double det = solve(m);
    // The determinant of a normal-equation matrix scales with the cube of the data, so the test has to be relative:
    // `|det|` against the product of the diagonal terms, which is what it would be if the regressors were orthogonal.
    // 正规方程矩阵的行列式随数据的立方缩放，所以判定必须是相对的：`|det|` 对着对角项之积比，后者正是二者正交时的值。
    const double scale = std::fmax((s_ii_ * s_ww_ * n), std::fmax(s_ii_, std::fmax(s_ww_, 1.0)));
    if (!std::isfinite(det) || (std::fabs(det) < (1.0e-9 * scale))) {
        refused.reason = "the regressors are collinear: the current and the speed did not separate, so no slope exists";
        return refused;
    }

    double cof[9];
    cof[0] = (m[4] * m[8]) - (m[5] * m[7]);
    cof[1] = -((m[3] * m[8]) - (m[5] * m[6]));
    cof[2] = (m[3] * m[7]) - (m[4] * m[6]);
    cof[3] = -((m[1] * m[8]) - (m[2] * m[7]));
    cof[4] = (m[0] * m[8]) - (m[2] * m[6]);
    cof[5] = -((m[0] * m[7]) - (m[1] * m[6]));
    cof[6] = (m[1] * m[5]) - (m[2] * m[4]);
    cof[7] = -((m[0] * m[5]) - (m[2] * m[3]));
    cof[8] = (m[0] * m[4]) - (m[1] * m[3]);
    // `A^-1 = adj(A)/det`, and for a symmetric matrix the cofactor matrix is its own transpose.
    // `A⁻¹ = adj(A)/det`，而对称矩阵的余子式矩阵就是它自己的转置。
    const double a_iq = ((cof[0] * rhs[0]) + (cof[3] * rhs[1]) + (cof[6] * rhs[2])) / det;
    const double a_w = ((cof[1] * rhs[0]) + (cof[4] * rhs[1]) + (cof[7] * rhs[2])) / det;
    const double a_zero = ((cof[2] * rhs[0]) + (cof[5] * rhs[1]) + (cof[8] * rhs[2])) / det;

    SpeedTuneResult out = nameplate_gains();
    out.reason = "";
    if (!std::isfinite(a_iq) || !(a_iq > 0.0)) {
        out.reason = "the acceleration did not follow the current: the identified acceleration per amp is not positive";
        return out;
    }
    // Residual: how much of the window the model does not explain, relative to the acceleration itself.
    // 残差：模型解释不了窗口的多少，相对于加速度自身。
    // `ss_res = sum(alpha^2) - beta'X'alpha` and `ss_tot = sum(alpha^2) - n*mean^2`, both from the moment sums.
    // `ss_res = Σα² − β'X'α`、`ss_tot = Σα² − n·mean²`，都由矩和给出。
    const double mean = s_a_ / n;
    const double explained = (a_iq * s_ia_) + (a_w * s_wa_) + (a_zero * s_a_);
    const double ss_res = s_aa_ - explained;
    const double ss_tot = s_aa_ - (n * mean * mean);
    const double relative = (ss_tot > 0.0) ? std::sqrt(std::fmax(ss_res, 0.0) / ss_tot) : 1.0;
    out.residual = static_cast<float>(relative);
    out.kt_over_j = static_cast<float>(a_iq);
    out.b_over_j = static_cast<float>(-a_w);
    out.j_kg_m2 = static_cast<float>(static_cast<double>(config_.kt_nm_per_a) / a_iq);

    if (!std::isfinite(out.residual) || (out.residual > config_.max_residual)) {
        out.reason =
            "the model does not explain the window (friction, a detent or a load that moves), so the fit is not a "
            "measurement";
        return out;
    }
    if (!(out.b_over_j >= 0.0F)) {
        out.reason = "the identified mechanical pole is negative, which no passive machine has";
        return out;
    }
    const float ratio = out.j_kg_m2 / config_.j_expected_kg_m2;
    if ((ratio > (1.0F + config_.tolerance)) || (ratio < (1.0F - config_.tolerance))) {
        out.reason =
            "the measured inertia is not the one in the configuration, so this is not the machine you described";
        return out;
    }

    // The gains come from the **ratios**, which is why a wrong nameplate `kt` or `J` cannot move them: `kp = bw/(kt/J)` and
    // `ki = bw*(b/J)/(kt/J)`.
    // 增益来自**比值**，这正是错误的铭牌 `kt` 或 `J` 动不了它们的原因：`kp = bw/(kt/J)`、`ki = bw·(b/J)/(kt/J)`。
    out.valid = true;
    out.kp_a_per_rad_s = config_.bandwidth_rad_s / out.kt_over_j;
    out.ki_a_per_rad_s2 = (config_.bandwidth_rad_s * out.b_over_j) / out.kt_over_j;
    return out;
}

} // namespace core

} // namespace foc
