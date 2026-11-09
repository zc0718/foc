// Conan::ImportStart
#include <foc/core/observer/dual_inductance_observer.hpp>

#include <cmath>

#include <foc/core/transforms.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file dual_inductance_observer.cpp
 *
 * @brief [en] Implements the dual-inductance observer: the complex-vector form, seeding, and the ratio that decides when the quadratic term stops mattering.
 * @brief [zh] 双电感观测器的实现：复矢量形式、播种，以及决定二次项何时可忽略的那个比值。
 * @since 0.1.0
 */

namespace foc {

namespace core {

namespace {
/** @cond */


/**
 * @brief [en] Implements valid_config(); the contract is documented on its declaration.
 * @brief [zh] 实现 valid_config()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
constexpr auto valid_config(const DualInductanceObserver::Config& config) noexcept -> bool {
    if (!((config.rs >= 0.0F) && (config.ld > 0.0F) && (config.lq > 0.0F) && (config.psi_f > 0.0F))) {
        return false;
    }
    // The gain is only read by the corrected source, so a caller that leaves it at zero on the integrator must not be
    // refused for a value it never uses -- and a caller that selects the correction with a gain outside (0, 1] must be.
    // 增益只被修正来源读取，所以用了积分来源、把增益留在零的调用方不该因为一个它从不使用的值被拒；而选了修正、却给了
    // (0, 1] 之外增益的调用方必须被拒。
    return (config.source == DualInductanceObserver::FluxSource::Integrator) ||
           ((config.correction_gain > 0.0F) && (config.correction_gain <= 1.0F));
}


/**
 * @brief [en] Wrap to [0, 2pi).
 * @brief [zh] 归一到 [0, 2π)。
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
 * @brief [en] Wrap to (-pi, pi].
 * @brief [zh] 归一到 (−π, π]。
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


/**
 * @brief [en] A complex number as two floats, so no `<complex>` is pulled into an ISR translation unit.
 * @brief [zh] 用两个 float 表示的复数，避免把 `<complex>` 拉进 ISR 的编译单元。
 * @since 0.1.0
 */
struct Cx final {
    float re;
    float im;
};

/**
 * @brief [en] `a * b`.
 * @brief [zh] `a * b`。
 * @since 0.1.0
 */
constexpr auto mul(Cx a, Cx b) noexcept -> Cx {
    return Cx{(a.re * b.re) - (a.im * b.im), (a.re * b.im) + (a.im * b.re)};
}

/**
 * @brief [en] `a + b`.
 * @brief [zh] `a + b`。
 * @since 0.1.0
 */
constexpr auto add(Cx a, Cx b) noexcept -> Cx { return Cx{a.re + b.re, a.im + b.im}; }

/**
 * @brief [en] `a - b`.
 * @brief [zh] `a - b`。
 * @since 0.1.0
 */
constexpr auto sub(Cx a, Cx b) noexcept -> Cx { return Cx{a.re - b.re, a.im - b.im}; }

/**
 * @brief [en] `a / b`, left to the hardware to saturate to inf/NaN rather than guarded: the caller has already
 *             established that `|b|` is not negligible.
 * @brief [zh] `a / b`；不做保护、交由硬件产生 inf/NaN——调用方已确保 `|b|` 不可忽略。
 * @since 0.1.0
 */
constexpr auto div(Cx a, Cx b) noexcept -> Cx {
    const float d = (b.re * b.re) + (b.im * b.im);
    return Cx{((a.re * b.re) + (a.im * b.im)) / d, ((a.im * b.re) - (a.re * b.im)) / d};
}

/**
 * @brief [en] `k * a`.
 * @brief [zh] `k * a`。
 * @since 0.1.0
 */
constexpr auto scale(float k, Cx a) noexcept -> Cx { return Cx{k * a.re, k * a.im}; }

/**
 * @brief [en] `|a|`.
 * @brief [zh] `|a|`。
 * @since 0.1.0
 */
auto magnitude(Cx a) noexcept -> float { return std::sqrt((a.re * a.re) + (a.im * a.im)); }

/**
 * @brief [en] `conj(a)`.
 * @brief [zh] `conj(a)`。
 * @since 0.1.0
 */
constexpr auto conj(Cx a) noexcept -> Cx { return Cx{a.re, -a.im}; }

/**
 * @brief [en] Principal square root of a complex number, `sqrt(r) * (cos(phi/2) + j*sin(phi/2))`.
 * @brief [zh] 复数的主平方根， `sqrt(r)·(cos(arg/2) + j·sin(arg/2))`。
 * @since 0.1.0
 */
auto sqrt_cx(Cx a) noexcept -> Cx {
    const float r = magnitude(a);
    const float half = 0.5F * std::atan2(a.im, a.re);
    const float s = std::sqrt(r);
    return Cx{s * std::cos(half), s * std::sin(half)};
}

/**
 * @brief [en] Angle of a complex number in [0, 2pi).
 * @brief [zh] 复数的角度，[0, 2π)。
 * @since 0.1.0
 */
auto arg(Cx a) noexcept -> float { return wrap_two_pi(std::atan2(a.im, a.re)); }


/**
 * @brief [en] Below this ratio of `|L1 * i|` to `psi_f` the quadratic term is dropped and the linear root is used.
 * @details [en] The dropped term is the saliency's own flux, and the linear answer it perturbs is off by about
 *               `|L1*i|/psi_f` in angle -- so at one part in ten thousand the discard is worth 1e-4 rad, three orders
 *               below the 0.01 rad the observer is asked for. The branch is not a convenience: `L1 * conj(i)` is the
 *               quadratic coefficient, so at **zero current** the equation is genuinely linear and dividing by that
 *               coefficient would answer with inf.
 * @details [zh] 被丢掉的项是凸极自身的磁链，被它扰动的线性答案角度偏差约为 `|L1·i|/psi_f` ——因此取万分之一时，这个丢弃
 *               值 1e-4 rad，比要求观测器做到的 0.01 rad 低三个数量级。这个分支不是便利： `L1·conj(i)` 就是二次项系数，
 *               **零电流** 时方程真的是线性的，除以该系数只会得到 inf。
 * @since 0.1.0
 */
constexpr float kLinearRatio = 1.0e-4F;

/** @endcond */
} // namespace

DualInductanceObserver::DualInductanceObserver(Config config) noexcept
    : config_(config), l0_(0.5F * (config.ld + config.lq)), l1_(0.5F * (config.ld - config.lq)), psi_{},
      seed_theta_(0.0F), state_{}, status_(valid_config(config) ? ErrorCode::Ok : ErrorCode::Param),
      theta_primed_(false), root_gap_rad_(0.0F), radius_error_(0.0F) {}

void DualInductanceObserver::seed_flux(Ab<float> psi_ab) noexcept {
    psi_ = psi_ab;
    // The stator flux leads the rotor by at most `atan(|L1*i| / psi_f)` -- under 10 degrees even at 100 A on the Prius
    // pair -- so its angle is a good enough tie-breaker for the first root and no better is needed: the seed only has to
    // pick the branch, the measurement picks the angle.
    // 定子磁链超前转子至多 `atan(|L1·i|/psi_f)`——即使在 Prius 电感对、100 A 时也不到 10°——所以它的角度足够给首个根
    // 定夺，且不需要更准：种子只负责选分支，角度由测量给出。
    seed_theta_ = arg(Cx{psi_ab.alpha, psi_ab.beta});
}

void DualInductanceObserver::seed_at(float theta_e, Ab<float> i_ab) noexcept {
    // `psi_f*e^(j*theta) + L0*i + L1*conj(i)*e^(j*2*theta)`, the stator flux of a salient machine -- the same relation
    // the quadratic in `update()` inverts, written forwards.
    // `psi_f·e^(jθ) + L0·i + L1·conj(i)·e^(j2θ)`，凸极机的定子磁链——正是 `update()` 里那个二次方程所反解的关系，正向写一遍。
    const float c1 = std::cos(theta_e);
    const float s1 = std::sin(theta_e);
    const float c2 = std::cos(2.0F * theta_e);
    const float s2 = std::sin(2.0F * theta_e);
    const float psi_f = config_.psi_f;
    psi_.alpha = (psi_f * c1) + (l0_ * i_ab.alpha) + (l1_ * ((i_ab.alpha * c2) + (i_ab.beta * s2)));
    psi_.beta = (psi_f * s1) + (l0_ * i_ab.beta) + (l1_ * ((i_ab.alpha * s2) - (i_ab.beta * c2)));
    seed_theta_ = arg(Cx{psi_.alpha, psi_.beta});
}

void DualInductanceObserver::apply_params(const MachineParams& params) noexcept {
    config_.rs = params.rs;
    config_.ld = params.ld;
    config_.lq = params.lq;
    config_.psi_f = params.psi_f;
    l0_ = 0.5F * (config_.ld + config_.lq);
    l1_ = 0.5F * (config_.ld - config_.lq);
    status_ = valid_config(config_) ? ErrorCode::Ok : ErrorCode::Param;
}

auto DualInductanceObserver::update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState {
    if (!(dt_s > 0.0F)) {
        status_ = ErrorCode::Param;
        return state_;
    }

    if (status_ == ErrorCode::Param) {
        return state_;
    }

    // d(psi)/dt = v - Rs*i. No inductance term at all: with saliency the current's own flux is not `L*i`, and subtracting
    // a single `L` here is exactly the approximation this observer exists to remove.
    // dψ/dt = v − Rs·i。完全不含电感项：有凸极时电流自身的磁链不是 `L·i`，在这里减掉单一 `L` 恰是本观测器要消除的近似。
    const float e_alpha = v_ab.alpha - (config_.rs * i_ab.alpha);
    const float e_beta = v_ab.beta - (config_.rs * i_ab.beta);
    psi_.alpha = psi_.alpha + (e_alpha * dt_s);
    psi_.beta = psi_.beta + (e_beta * dt_s);

    // Solve `L1*conj(i)*x^2 + psi_f*x + (L0*i - psi) = 0` for `x = e^(j*theta)`.
    // 对 `x = e^(jθ)` 解 `L1·conj(i)·x² + psi_f·x + (L0·i − ψ) = 0`。
    const Cx i_cx{i_ab.alpha, i_ab.beta};
    const Cx psi_cx{psi_.alpha, psi_.beta};
    const Cx a = scale(l1_, conj(i_cx));
    const Cx b{config_.psi_f, 0.0F};
    const Cx c = sub(scale(l0_, i_cx), psi_cx);

    Cx root_a{};
    Cx root_b{};
    if (magnitude(a) < (kLinearRatio * config_.psi_f)) {
        // Genuinely (or numerically) surface-mounted: one root, `x = -c / b`.
        // 真正（或数值上）是表贴式：只有一个根，`x = −c/b`。
        root_a = div(scale(-1.0F, c), b);
        root_b = root_a;
    } else {
        // Stable pair of roots: `q = -0.5*(b + s*D)` with `s` chosen so the sum does not cancel, then `q/a` and `c/q`.
        // 稳定的两根公式：`q = −0.5·(b + s·D)`，`s` 取使求和不相消的方向，再取 `q/a` 与 `c/q`。
        const Cx discriminant = sqrt_cx(sub(mul(b, b), scale(4.0F, mul(a, c))));
        const float sign = ((b.re * discriminant.re) + (b.im * discriminant.im)) < 0.0F ? -1.0F : 1.0F;
        const Cx q = scale(-0.5F, add(b, scale(sign, discriminant)));
        root_a = div(q, a);
        root_b = div(c, q);
    }

    // Of the two rotor positions consistent with this flux and current, take the one continuous with the previous
    // estimate; after a seed, continuous with the angle the seed implied.
    // 与本次磁链、电流一致的两个转子位置中，取与上一次估计连续的那个；播种后的第一步，则与种子隐含的角度连续。
    const float reference = theta_primed_ ? state_.theta_e : seed_theta_;
    const float gap_a = std::fabs(wrap_pi(arg(root_a) - reference));
    const float gap_b = std::fabs(wrap_pi(arg(root_b) - reference));
    const Cx chosen = (gap_b < gap_a) ? root_b : root_a;
    root_gap_rad_ = std::fabs(wrap_pi(arg(root_a) - arg(root_b)));
    // `norm(x) - 1`: zero iff the state is consistent with this current, which is the only case in which the root is a
    // rotor position at all. It is the one signal that survives a wrong seed -- see the header.
    // `norm(x) − 1`：当且仅当状态与该电流自洽时为零，而那也正是这个根是转子位置的唯一情形。它是错误种子下唯一还活着的
    // 信号——见头文件。
    const float radius = magnitude(chosen);
    radius_error_ = radius - 1.0F;
    if ((config_.source == FluxSource::Corrected) && (radius > 1.0e-6F)) {
        // Pull the state back onto the flux relation, by the fraction of the residual the gain asks for. The fixed point
        // is `radius == 1`, i.e. a state a rotor position can explain; what this buys is that a nameplate error in `Rs`
        // stops accumulating in the state. It does **not** repair a wrong seed -- the nearest consistent state to a
        // mis-seeded one is still the wrong one -- and that is why both sources exist instead of one replacing the other.
        // 把状态按增益所要求的那部分残差拉回磁链关系上。不动点是 `radius == 1`，即一个能被转子位置解释的状态；它买到的是
        // `Rs` 的铭牌误差不再在状态里累积。它 **不会** 修复错误的种子——离一个错种子"最近的自洽状态"仍然是错的那个——这正是
        // 两种来源并存、而不是一种取代另一种的原因。
        const Cx unit = scale(1.0F / radius, chosen);
        const Cx target =
            add(scale(config_.psi_f, unit), add(scale(l0_, i_cx), mul(scale(l1_, conj(i_cx)), mul(unit, unit))));
        const float gain = config_.correction_gain;
        psi_.alpha = psi_.alpha + (gain * (target.re - psi_.alpha));
        psi_.beta = psi_.beta + (gain * (target.im - psi_.beta));
    }

    const float theta = arg(chosen);
    const float omega = theta_primed_ ? static_cast<float>(wrap_pi(theta - state_.theta_e) / dt_s) : 0.0F;

    state_.theta_e = theta;
    state_.omega_e = omega;
    theta_primed_ = true;
    return state_;
}

void DualInductanceObserver::reset() noexcept {
    psi_ = Ab<float>{};
    seed_theta_ = 0.0F;
    state_ = ObsState{0.0F, 0.0F};
    theta_primed_ = false;
    root_gap_rad_ = 0.0F;
    radius_error_ = 0.0F;
    status_ = valid_config(config_) ? ErrorCode::Ok : ErrorCode::Param;
}

auto DualInductanceObserver::status() const noexcept -> ErrorCode { return status_; }

auto DualInductanceObserver::flux() const noexcept -> Ab<float> { return psi_; }

} // namespace core

} // namespace foc
