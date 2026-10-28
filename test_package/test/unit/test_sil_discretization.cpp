// S20 - does the SMO's advantage come from the observer, or from the integrator
// it happens to share with the incumbent? Same switching bench, four variants,
// only the integration scheme differs. S20 - SMO
// 的优势来自观测器，还是来自它与在位者恰好共用的那个积分式？同一台开关台架，四个变体，只有
// 积分格式不同。

#include <cmath>
#include <cstdio>
#include <initializer_list>

#include <gtest/gtest.h>

#include <foc/core/observer/nonlinear_flux_observer.hpp>

#include "sil_switching_rig.hpp"

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

constexpr float kBelief = 1.10F;
constexpr float kGainRatio = 0.3F;
constexpr float kMinGain = 3.0F;
constexpr float kSpeeds[] = {20.0F, 100.0F, 200.0F, 400.0F, 800.0F};

auto believed_rs() -> float { return kBelief * sil::nominal_spec().rs; }

auto wrap_local(float from, float to) noexcept -> float {
  float delta = to - from;
  while (delta <= -core::kPi<float>) {
    delta += core::kTwoPi<float>;
  }
  while (delta > core::kPi<float>) {
    delta -= core::kTwoPi<float>;
  }
  return delta;
}

/**
 * @brief [en] A faithful copy of `NonlinearFluxObserver`'s arithmetic, written
 * out so the integration scheme can be swapped without touching the library.
 *        [zh] `NonlinearFluxObserver`
 * 运算的忠实副本，写出来是为了能在不动库的前提下替换积分格式。
 *
 * @note [en] C2-a checks this against the shipped class. If the copy ever
 * drifts, the comparison below is between two different observers and means
 * nothing -- exactly the failure mode this batch exists to avoid. [zh] C2-a
 * 把它对着出厂类检查。副本一旦漂移，下面的对照就是在比较两个不同的观测器，什么也不说明—— 而这正是本批要避免的那类失败。
 * @since 0.1.0
 */
class EulerFlux final {
public:
  EulerFlux(float rs, float l, float psi_f) noexcept
      : rs_(rs), l_(l), psi_f_(psi_f) {}

  void seed(const core::Ab<float> &psi) noexcept { psi_ = psi; }

  auto update(const core::Ab<float> &v, const core::Ab<float> &i,
              float dt) noexcept -> core::ObsState {
    if (!primed_) {
      i_prev_ = i;
      primed_ = true;
    }
    const float e_a = static_cast<float>(v.alpha - (rs_ * i.alpha));
    const float e_b = static_cast<float>(v.beta - (rs_ * i.beta));
    psi_.alpha = static_cast<float>(psi_.alpha + (e_a * dt) -
                                    (l_ * (i.alpha - i_prev_.alpha)));
    psi_.beta = static_cast<float>(psi_.beta + (e_b * dt) -
                                   (l_ * (i.beta - i_prev_.beta)));
    i_prev_ = i;
    correct(dt);
    return state_;
  }

protected:
  void correct(float dt) noexcept {
    const float magnitude_squared =
        (psi_.alpha * psi_.alpha) + (psi_.beta * psi_.beta);
    const float psi_f_squared = psi_f_ * psi_f_;
    const float scheduled =
        std::fmax(kMinGain, kGainRatio * std::fabs(state_.omega_e));
    const float scale = ((0.5F * scheduled) / psi_f_squared) *
                        (psi_f_squared - magnitude_squared);
    psi_.alpha = static_cast<float>(psi_.alpha + ((scale * psi_.alpha) * dt));
    psi_.beta = static_cast<float>(psi_.beta + ((scale * psi_.beta) * dt));

    float theta = std::atan2(psi_.beta, psi_.alpha);
    while (theta < 0.0F) {
      theta += core::kTwoPi<float>;
    }
    while (theta >= core::kTwoPi<float>) {
      theta -= core::kTwoPi<float>;
    }
    const float omega =
        primed_state_
            ? static_cast<float>(wrap_local(state_.theta_e, theta) / dt)
            : 0.0F;
    state_.theta_e = theta;
    state_.omega_e = omega;
    primed_state_ = true;
  }

  core::Ab<float> psi_{};
  core::Ab<float> i_prev_{};
  core::ObsState state_{};
  bool primed_{false};
  bool primed_state_{false};

private:
  float rs_;
  float l_;
  float psi_f_;
};

/// [en] The same observer with `(v - Rs*i)` integrated by the trapezoid instead
/// of the endpoint value.
///      [zh] 同一个观测器，`(v − Rs·i)` 用梯形而不是端点值积分。
class TustinFlux final {
public:
  TustinFlux(float rs, float l, float psi_f) noexcept
      : rs_(rs), l_(l), psi_f_(psi_f) {}

  void seed(const core::Ab<float> &psi) noexcept { psi_ = psi; }

  auto update(const core::Ab<float> &v, const core::Ab<float> &i,
              float dt) noexcept -> core::ObsState {
    const float e_a = static_cast<float>(v.alpha - (rs_ * i.alpha));
    const float e_b = static_cast<float>(v.beta - (rs_ * i.beta));
    if (!primed_) {
      // No previous integrand: the first step falls back to the endpoint,
      // exactly like Euler. 没有上一个被积函数：第一步退回端点式，与欧拉相同。
      psi_.alpha = static_cast<float>(psi_.alpha + (e_a * dt));
      psi_.beta = static_cast<float>(psi_.beta + (e_b * dt));
      primed_ = true;
    } else {
      psi_.alpha =
          static_cast<float>(psi_.alpha + (0.5F * dt * (e_a + e_prev_.alpha)) -
                             (l_ * (i.alpha - i_prev_.alpha)));
      psi_.beta =
          static_cast<float>(psi_.beta + (0.5F * dt * (e_b + e_prev_.beta)) -
                             (l_ * (i.beta - i_prev_.beta)));
    }
    i_prev_ = i;
    e_prev_ = core::Ab<float>{e_a, e_b};
    correct(dt);
    return state_;
  }

private:
  void correct(float dt) noexcept {
    const float magnitude_squared =
        (psi_.alpha * psi_.alpha) + (psi_.beta * psi_.beta);
    const float psi_f_squared = psi_f_ * psi_f_;
    const float scheduled =
        std::fmax(kMinGain, kGainRatio * std::fabs(state_.omega_e));
    const float scale = ((0.5F * scheduled) / psi_f_squared) *
                        (psi_f_squared - magnitude_squared);
    psi_.alpha = static_cast<float>(psi_.alpha + ((scale * psi_.alpha) * dt));
    psi_.beta = static_cast<float>(psi_.beta + ((scale * psi_.beta) * dt));

    float theta = std::atan2(psi_.beta, psi_.alpha);
    while (theta < 0.0F) {
      theta += core::kTwoPi<float>;
    }
    while (theta >= core::kTwoPi<float>) {
      theta -= core::kTwoPi<float>;
    }
    const float omega =
        primed_state_
            ? static_cast<float>(wrap_local(state_.theta_e, theta) / dt)
            : 0.0F;
    state_.theta_e = theta;
    state_.omega_e = omega;
    primed_state_ = true;
  }

  float rs_;
  float l_;
  float psi_f_;
  core::Ab<float> psi_{};
  core::Ab<float> i_prev_{};
  core::Ab<float> e_prev_{};
  core::ObsState state_{};
  bool primed_{false};
  bool primed_state_{false};
};

/**
 * @brief [en] The SMO with Heun (explicit trapezoid) integration of the current
 * equation. [zh] 电流方程用 Heun（显式梯形）积分的 SMO。
 *
 * @note [en] Deliberately identical to `sil::SmoControl` apart from the
 * integrator: same sliding surface, same saturation, same filter, and the
 * filter is fed the **same** `z` (the one at the start of the step), so the
 * only difference measured is the integration. [zh] 除积分器之外刻意与
 * `sil::SmoControl` 完全一致：同滑模面、同饱和、同滤波器，且滤波器拿到的是
 *       **同一个** `z`（步首那个），所以量到的差别只有积分。
 * @since 0.1.0
 */
class HeunSmo final {
public:
  HeunSmo(float rs, float l, float gain, float fc_hz, float layer,
          float dt_s) noexcept
      : rs_(rs), l_(l), gain_(gain), layer_(layer), dt_(dt_s),
        alpha_(std::exp(-core::kTwoPi<float> * fc_hz * dt_s)) {}

  auto update(const core::Ab<float> &v, const core::Ab<float> &i) noexcept
      -> core::ObsState {
    const float z_a = gain_ * sat(static_cast<float>(i_hat_.alpha - i.alpha));
    const float z_b = gain_ * sat(static_cast<float>(i_hat_.beta - i.beta));

    const float f1a =
        static_cast<float>((v.alpha - (rs_ * i_hat_.alpha) - z_a) / l_);
    const float f1b =
        static_cast<float>((v.beta - (rs_ * i_hat_.beta) - z_b) / l_);
    const float pred_a = static_cast<float>(i_hat_.alpha + (dt_ * f1a));
    const float pred_b = static_cast<float>(i_hat_.beta + (dt_ * f1b));
    const float f2a = static_cast<float>(
        (v.alpha - (rs_ * pred_a) -
         (gain_ * sat(static_cast<float>(pred_a - i.alpha)))) /
        l_);
    const float f2b = static_cast<float>(
        (v.beta - (rs_ * pred_b) -
         (gain_ * sat(static_cast<float>(pred_b - i.beta)))) /
        l_);
    i_hat_.alpha =
        static_cast<float>(i_hat_.alpha + (0.5F * dt_ * (f1a + f2a)));
    i_hat_.beta = static_cast<float>(i_hat_.beta + (0.5F * dt_ * (f1b + f2b)));

    e_.alpha =
        static_cast<float>((alpha_ * e_.alpha) + ((1.0F - alpha_) * z_a));
    e_.beta = static_cast<float>((alpha_ * e_.beta) + ((1.0F - alpha_) * z_b));

    float theta = std::atan2(-e_.alpha, e_.beta);
    while (theta < 0.0F) {
      theta += core::kTwoPi<float>;
    }
    state_.theta_e = theta;
    return state_;
  }

private:
  [[nodiscard]] auto sat(float value) const noexcept -> float {
    const float normalized = value / layer_;
    return (normalized > 1.0F) ? 1.0F
                               : ((normalized < -1.0F) ? -1.0F : normalized);
  }

  float rs_;
  float l_;
  float gain_;
  float layer_;
  float dt_;
  float alpha_;
  core::Ab<float> i_hat_{};
  core::Ab<float> e_{};
  core::ObsState state_{};
};

template <typename Observer, typename Make>
auto flux_error(const sil::SwitchingRigConfig &config, Make &&make) -> float {
  Observer observer = make();
  observer.seed(core::Ab<float>{sil::SwitchingRig::psi_f(), 0.0F});
  return sil::run_switched(config, [&observer](sil::SwitchingRig &rig) {
    return observer.update(rig.command(), rig.current(),
                           sil::SwitchingRig::kControlPeriod);
  });
}

auto shipped_flux_error(const sil::SwitchingRigConfig &config) -> float {
  core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
      believed_rs(), sil::nominal_spec().lq, sil::SwitchingRig::psi_f(),
      kGainRatio, kMinGain, 0.5F * sil::SwitchingRig::psi_f()});
  observer.seed_flux(core::Ab<float>{sil::SwitchingRig::psi_f(), 0.0F});
  return sil::run_switched(config, [&observer](sil::SwitchingRig &rig) {
    return observer.update(rig.command(), rig.current(),
                           sil::SwitchingRig::kControlPeriod);
  });
}

auto smo_error(const sil::SwitchingRigConfig &config, bool heun) -> float {
  const sil::SmoTuning tuning =
      sil::scheduled_smo(config.omega_e, sil::SwitchingRig::psi_f());
  if (heun) {
    HeunSmo observer(believed_rs(), sil::nominal_spec().lq, tuning.gain,
                     tuning.fc_hz, tuning.layer,
                     sil::SwitchingRig::kControlPeriod);
    return sil::run_switched(config, [&observer](sil::SwitchingRig &rig) {
      return observer.update(rig.command(), rig.current());
    });
  }
  sil::SmoControl observer(believed_rs(), sil::nominal_spec().lq, tuning.gain,
                           tuning.fc_hz, tuning.layer);
  return sil::run_switched(config, [&observer](sil::SwitchingRig &rig) {
    return observer.update(rig.command(), rig.current());
  });
}

} // namespace

TEST(SilDiscretization, C2a_TheTestCopyReproducesTheShippedObserver) {
  // Without this, every number below compares an observer against a slightly
  // different observer and the conclusion is about the copy, not about the
  // discretisation.
  // 少了这一条，下面所有数字都是在拿一个观测器和"另一个略有不同的观测器"比较，结论就成了关于副本的，
  // 而不是关于离散化的。
  std::printf("\n[S20] C2-a: test-local Euler copy vs shipped "
              "`NonlinearFluxObserver`\n");
  float worst = 0.0F;
  for (const float omega : kSpeeds) {
    sil::SwitchingRigConfig config{};
    config.omega_e = omega;
    const float shipped = shipped_flux_error(config);
    const float copy = flux_error<EulerFlux>(config, [] {
      return EulerFlux(believed_rs(), sil::nominal_spec().lq,
                       sil::SwitchingRig::psi_f());
    });
    std::printf("[S20]   %5.0f rad/s: shipped %.6f, copy %.6f, diff %.2e\n",
                static_cast<double>(omega), static_cast<double>(shipped),
                static_cast<double>(copy), static_cast<double>(shipped - copy));
    worst = std::fmax(worst, std::fabs(shipped - copy));
  }
  std::printf("[S20] C2-a: worst difference %.2e rad\n",
              static_cast<double>(worst));
  EXPECT_LT(worst, 1.0e-4F);
}

TEST(SilDiscretization, C2b_TheRankingDoesNotComeFromTheIntegrator) {
  // The question this batch was asked to settle. If the SMO wins only because
  // the incumbent is on a cruder integrator, then "the SMO is better" is a
  // statement about numerical analysis, not about estimation -- and adopting it
  // on that basis would be a mistake. 本批要结清的那个问题。如果 SMO
  // 赢只是因为在位者用了更粗的积分器，那"SMO 更好"就是一句数值分析的话，
  // 不是一句估计的话——据此采用它是个错误。
  //
  // Note the criterion as *revised*: the original C2-b said "the SMO still wins
  // at every point under the trapezoid". C1 already showed the ranking reverses
  // at 800 rad/s under Euler, so that criterion could never have been met, and
  // keeping it would have been a way of never writing the answer down. What is
  // tested instead is that the **sign** of the ranking at each speed survives
  // the change of scheme. 注意判据是**改写过**的：原 C2-b 说"梯形下 SMO
  // 在每个点仍然赢"。C1 已经显示欧拉下 800 rad/s 处排序就是
  // 反的，所以那条判据不可能被满足，留着它等于永远不写答案。现在检验的是：每个速度点上排序的**符号**在换
  // 积分格式之后是否保持不变。
  std::printf(
      "[S20] C2-b: Euler vs Heun/Tustin, same bench, same tuning schedule\n");
  std::printf("[S20] %8s %10s %10s %10s %10s %9s %9s\n", "omega_e", "NLFO eul",
              "NLFO tus", "SMO eul", "SMO heun", "ratio eul", "ratio tus");

  int sign_matches = 0;
  float shrink[5] = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
  int index = 0;
  for (const float omega : kSpeeds) {
    sil::SwitchingRigConfig config{};
    config.omega_e = omega;

    const float flux_euler = flux_error<EulerFlux>(config, [] {
      return EulerFlux(believed_rs(), sil::nominal_spec().lq,
                       sil::SwitchingRig::psi_f());
    });
    const float flux_tustin = flux_error<TustinFlux>(config, [] {
      return TustinFlux(believed_rs(), sil::nominal_spec().lq,
                        sil::SwitchingRig::psi_f());
    });
    const float smo_euler = smo_error(config, false);
    const float smo_heun = smo_error(config, true);

    const float ratio_euler = flux_euler / smo_euler;
    const float ratio_tustin = flux_tustin / smo_heun;
    std::printf("[S20] %8.0f %10.4f %10.4f %10.4f %10.4f %9.2f %9.2f\n",
                static_cast<double>(omega), static_cast<double>(flux_euler),
                static_cast<double>(flux_tustin),
                static_cast<double>(smo_euler), static_cast<double>(smo_heun),
                static_cast<double>(ratio_euler),
                static_cast<double>(ratio_tustin));

    const bool same_side = ((ratio_euler > 1.0F) == (ratio_tustin > 1.0F));
    if (same_side) {
      ++sign_matches;
    }
    // How much of the advantage is the incumbent's integrator? This fraction
    // grows with speed.
    // 优势里有多少是在位者的积分器？这个比例随转速升高而变大。
    shrink[index] = (ratio_euler - ratio_tustin) / ratio_euler;
    ++index;

    // The scheme has to actually do something, or "the ranking survives" is
    // vacuous. 积分格式必须真的起了作用，否则"排序不变"是空话。
    EXPECT_GT(std::fabs(flux_tustin - flux_euler), 1.0e-5F)
        << "the schemes must differ at " << omega;
  }

  std::printf(
      "[S20] C2-b: ranking sign matches %d/5; share of the advantage the finer "
      "scheme gives back: "
      "%.0f %% / %.0f %% / %.0f %% / %.0f %% / %.0f %% (20..800 rad/s)\n",
      sign_matches, static_cast<double>(100.0F * shrink[0]),
      static_cast<double>(100.0F * shrink[1]),
      static_cast<double>(100.0F * shrink[2]),
      static_cast<double>(100.0F * shrink[3]),
      static_cast<double>(100.0F * shrink[4]));

  // C2-b: the *sign* of the ranking is a property of the observers, not of the
  // integrator. C2-b：排序的**符号**是观测器的性质，不是积分器的。
  EXPECT_EQ(5, sign_matches);

  // C2-d, and this is what the batch was for: the *size* of the advantage is
  // not. A finer scheme on both sides takes back 25 % of it at 400 rad/s and 52
  // % at 800 -- so a large part of "the SMO is more accurate" at those speeds
  // is the incumbent's own Euler lag, i.e. an artefact of the current
  // discretisation. The low band is different: under 15 % at 20 and 100 rad/s,
  // so there the advantage is real.
  // C2-d，这才是本批真正要找的东西：优势的**大小**不是。两边都换细格式之后，400
  // rad/s 处要被拿回 25%，800 处 52%——那些转速下"SMO
  // 更准"有很大一部分是在位者自己的欧拉滞后，也就是当前离散化的伪迹。低端不同：
  // 20 与 100 rad/s 处不到 15%，那里优势是真的。
  EXPECT_LT(shrink[0], 0.15F) << "low speed: the advantage has to be robust";
  EXPECT_LT(shrink[1], 0.15F) << "low speed: the advantage has to be robust";
  EXPECT_GT(shrink[3], 0.20F)
      << "mid band: the advantage has to be visibly scheme-dependent";
  EXPECT_GT(shrink[4], 0.40F)
      << "top of the range: it is mostly the incumbent's integrator";
}
