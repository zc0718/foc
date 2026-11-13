// Conan::ImportStart
#include <foc/core/thermal_limit.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file thermal_limit.cpp
 *
 * @brief [en] Implements the thermal limit: the steady-state root of the loss equation and the time budget to it.
 * @brief [zh] 热限值的实现：损耗方程的稳态根，以及到达它的时间预算。
 * @see Ledger `E-THERM-1` (where a held current settles, and the edge past which no settle exists) and
 *      `ThermalLimit.TheContinuousCurrentSettlesThePlantAtTheAllowedTemperatureAndFivePercentMoreDoesNot`.
 *      ／ 台账 `E-THERM-1`（保持电流的稳态温度，以及越过之后不存在稳态的那条边界）与
 *      `ThermalLimit.TheContinuousCurrentSettlesThePlantAtTheAllowedTemperatureAndFivePercentMoreDoesNot`。
 * @since 0.1.0
 */

#include <cmath>
#include <limits>

namespace foc {

namespace core {

namespace {
/** @cond */

constexpr double kLossPerAmpSquared = 1.5; // `P = 1.5*i^2*rs`, the same amplitude-invariant 3/2 the torque carries
constexpr float kInfinity = std::numeric_limits<float>::infinity();

/** @endcond */
} // namespace

auto ThermalLimit::validate() const noexcept -> ErrorCode {
    const double rs = static_cast<double>(config_.params.rs);
    const double rt = static_cast<double>(config_.rt_k_per_w);
    const double alpha = static_cast<double>(config_.alpha_per_k);
    if (!std::isfinite(rs) || !(rs > 0.0) || !std::isfinite(rt) || !(rt > 0.0)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(alpha) || (alpha < 0.0)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.t_ambient_c) || !std::isfinite(config_.t_allowed_c)) {
        return ErrorCode::Param;
    }
    // A reserve that is not above ambient is not a reserve: no current can heat the winding *down* to it, so the question
    // this class answers has no answer rather than the answer zero.
    // 不高于环境温度的余量不是余量：没有电流能把绕组 **加热** 到它，于是本类要回答的问题是"没有答案"，而不是"答案为零"。
    if (!(static_cast<double>(config_.t_allowed_c) > static_cast<double>(config_.t_ambient_c))) {
        return ErrorCode::Param;
    }
    return ErrorCode::Ok;
}

auto ThermalLimit::continuous() const noexcept -> ThermalLimitReport {
    ThermalLimitReport report{ErrorCode::Param, 0.0F, 0.0F, 0.0F};
    if (validate() != ErrorCode::Ok) {
        return report;
    }
    const double rt = static_cast<double>(config_.rt_k_per_w);
    const double alpha = static_cast<double>(config_.alpha_per_k);
    const double t_allowed = static_cast<double>(config_.t_allowed_c);
    // `i_cont^2 = (t_allowed - Ta) / (Rt*1.5*rs20*(1 + alpha*(t_allowed-20)))`. The parenthesised factor is the feedback: it says the winding is *less* able to carry current at the temperature the reserve is sized against, and dropping it is the naive answer that reads 15.5 % high on copper at 105 C. No guards here: `validate()` has already established `rs > 0`, `rt > 0` and `t_allowed > t_ambient`, which is exactly what makes the denominator and the numerator positive. A second check that can never fire is a line nobody can test.
    // `i_cont² = (t_allowed − Ta)/(Rt·1.5·rs20·(1 + α·(t_allowed−20)))`。括号里那一项就是反馈：它说绕组在" 余量所在的那个 温度"上 **更带不动** 电流，而丢掉它得到的是在铜的 105 ℃ 处高 15.5 % 的天真答案。 这里 不设护栏： `validate()` 已经确立 `rs > 0`、 `rt > 0`、 `t_allowed > t_ambient`，而这正是分母与分子为 正的条件。再加一道 永不触发的检查，就是一行谁也测不到的东西。
    const double denominator =
        rt * kLossPerAmpSquared * static_cast<double>(config_.params.rs) * (1.0 + (alpha * (t_allowed - 20.0)));
    const double squared = (t_allowed - static_cast<double>(config_.t_ambient_c)) / denominator;
    report.status = ErrorCode::Ok;
    report.continuous_a = static_cast<float>(std::sqrt(squared));
    report.settle_at_cont_c =
        static_cast<float>(t_allowed); // by construction; reported so the caller sees the assumption
    report.thermal_time_const_s = (config_.ct_j_per_k > 0.0F) ? (config_.rt_k_per_w * config_.ct_j_per_k) : 0.0F;
    return report;
}

auto ThermalLimit::settle_temperature_c(float current_a) const noexcept -> float {
    if (validate() != ErrorCode::Ok) {
        return NAN;
    }
    const double i_sq = static_cast<double>(current_a) * static_cast<double>(current_a);
    if (!std::isfinite(i_sq)) {
        return kInfinity;
    }
    const double k_loss = kLossPerAmpSquared * static_cast<double>(config_.params.rs) * i_sq;
    const double alpha = static_cast<double>(config_.alpha_per_k);
    const double rt = static_cast<double>(config_.rt_k_per_w);
    // `Ct*dT/dt = (k(1-20a) + Ta/Rt) + T*(k*a - 1/Rt)`, i.e. `dT/dt = (a + b*T)/Ct`, so the settle is `-a/b` and it only
    // exists while `b < 0` -- which is exactly `Rt*k*alpha < 1`, the same boundary the plant shows from the other side.
    // `Ct·dT/dt = (k(1−20α) + Ta/Rt) + T·(k·α − 1/Rt)`，即 `dT/dt = (a + b·T)/Ct`，于是稳态是 `−a/b`，且只在 `b < 0`
    // （也就是 `Rt·k·α < 1`）时存在——与 plant 从另一侧看到的正是同一个边界。
    const double b = (k_loss * alpha) - (1.0 / rt);
    if (b >= 0.0) {
        return kInfinity;
    }
    const double a = (k_loss * (1.0 - (20.0 * alpha))) + (static_cast<double>(config_.t_ambient_c) / rt);
    return static_cast<float>(-a / b);
}

auto ThermalLimit::budget(float current_a, float t_now_c) const noexcept -> ThermalBudget {
    const ThermalLimitReport cont = continuous();
    ThermalBudget out{ErrorCode::Param, false, 0.0F, 0.0F};
    if (cont.status != ErrorCode::Ok) {
        return out;
    }
    if (!std::isfinite(current_a) || !std::isfinite(t_now_c)) {
        return out;
    }
    const double i_sq = static_cast<double>(current_a) * static_cast<double>(current_a);
    const double k_loss = kLossPerAmpSquared * static_cast<double>(config_.params.rs) * i_sq;
    const double alpha = static_cast<double>(config_.alpha_per_k);
    const double rt = static_cast<double>(config_.rt_k_per_w);
    const double b = (k_loss * alpha) - (1.0 / rt);
    const double a = (k_loss * (1.0 - (20.0 * alpha))) + (static_cast<double>(config_.t_ambient_c) / rt);
    const double t_star = static_cast<double>(config_.t_allowed_c);
    const double t_now = static_cast<double>(t_now_c);

    out.status = ErrorCode::Ok;
    if (!std::isfinite(i_sq)) {
        out.crosses = true;
        out.seconds = 0.0F;
        out.settle_c = kInfinity;
        return out;
    }
    if (t_now >= t_star) {
        // Already at or past the reserve: the answer is "no time left", not "a negative time".
        // 已经到或超过余量：答案是"没有时间了"，不是"负的时间"。
        out.crosses = true;
        out.seconds = 0.0F;
        out.settle_c = settle_temperature_c(current_a);
        return out;
    }
    if (std::fabs(b) < 1.0e-12) {
        // `b == 0`: the node is a pure integrator; its slope is `a/Ct` with no exponential in it.
        // `b == 0`：节点是纯积分器，斜率是 `a/Ct`，没有指数项。
        out.settle_c = kInfinity;
        out.crosses = (a > 0.0);
        if (!out.crosses || (!(config_.ct_j_per_k > 0.0F))) {
            out.seconds = kInfinity; // sustainable, or no `Ct` supplied so no time can be named
            return out;
        }
        out.seconds = static_cast<float>((t_star - t_now) * static_cast<double>(config_.ct_j_per_k) / a);
        return out;
    }
    const double t_eq = -a / b;
    // `t_eq` is the *asymptote* of `dT/dt = (a + b*T)/Ct`, and when `b > 0` that asymptote is a negative number the node
    // runs **away** from -- reporting it as "where this settles" would hand a scheduler a plausible-looking -234 C. So the
    // reported settle is infinity whenever there is no steady state, while the crossing time below still uses the asymptote,
    // because the exact solution needs it.
    // `t_eq` 是 `dT/dt = (a + b·T)/Ct` 的 **渐近线** ；当 `b > 0` 时那是一个节点正在 **远离** 的负数——把它当成"收敛到哪里"报出去，
    // 等于递给调度一个看着挺合理的 −234 ℃。所以只要没有稳态，报出的稳态就是无穷；而下面的穿越时间仍然用那条渐近线，因为精确解需要它。
    out.settle_c = (b < 0.0) ? static_cast<float>(t_eq) : kInfinity;
    // Sustainability is decided in **current**, not by comparing two temperatures: `t_eq` at `i_cont` sits
    // a rounding error above `t_star`, and a comparison in temperature would then hand back a finite
    // crossing time for the very current the class calls continuous. The tolerance is on the current (
    // `1e-6` relative), so "continuous means forever" holds by construction and any current measurably
    // above it still gets its honest, finite crossing.
    // 可持续性用 **电流** 判定，而不是比较两个温度： `i_cont` 处的 `t_eq` 会高出 `t_star` 一个舍入误差，若
    // 按温度比较，本类称为可持续的 那个电流反而会拿到一个有限的穿越时间。容差落在电流上（相对 `1e-6`），于
    // 是"可持续即永远"按构造成立，而任何可测地高于它的电流 仍会拿到它诚实的、有限的穿越时间。
    const float i_cont = continuous().continuous_a;
    const bool sustainable = (b < 0.0) && (std::fabs(current_a) <= (static_cast<double>(i_cont) * (1.0 + 1.0e-6)));
    out.crosses = !sustainable;
    if (sustainable) {
        out.seconds = kInfinity;
        return out;
    }
    if (!(config_.ct_j_per_k > 0.0F)) {
        out.seconds = kInfinity; // the crossing exists but no `Ct` was supplied, so no time can be named
        return out;
    }
    // `t = (Ct/b)*ln((T* - T_eq)/(T_now - T_eq))`, the exact solution, valid for either sign of `b` --
    // which is why the run-away case needs no separate branch and no integration.
    // `t = (Ct/b)·ln((T* − T_eq)/(T_now − T_eq))`，精确解，对 `b` 的两种符号都成立——正因为如此，失控那一支
    // 不需要单独的 分支，也不需要积分。
    const double ratio = (t_star - t_eq) / (t_now - t_eq);
    if (!(ratio > 0.0)) {
        out.seconds = 0.0F;
        return out;
    }
    const double seconds = (static_cast<double>(config_.ct_j_per_k) / b) * std::log(ratio);
    out.seconds = static_cast<float>(seconds > 0.0 ? seconds : 0.0);
    return out;
}

auto ThermalLimit::thermal_resistance_k_per_w() const noexcept -> float { return config_.rt_k_per_w; }

} // namespace core

} // namespace foc
