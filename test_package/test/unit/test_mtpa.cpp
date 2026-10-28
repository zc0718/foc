// P10 - MTPA. The d-axis current that gets the most torque per amp, and the
// question that decides whether it is worth shipping at all: does turning it on
// require a machine the rest of the library cannot observe?
// P10——MTPA。每安培转矩最大的那个 d 轴电流，以及决定它值不值得出厂的那个问题：
// 打开它是否要求一台库里其它部分观测不了的机器？

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/reference.hpp>
#include <foc/core/transforms.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"
#include "sil_switching_rig.hpp"

namespace foc {

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

/**
 * @brief [en] The criterion, written before the code it judges.
 *        [zh] 判据，写在它所判定的代码之前。
 *
 * @note [en] `i_char = psi_f/Ld` is the one dimensionless switch that decides
 * whether MTPA is worth anything: it is the current at which the reluctance
 * term is the same size as the magnet term. At `i_char/i_max = 4` (the nominal
 * surface machine) the best possible gain is 3 %; at `i_char/i_max = 1` it is
 * 30 %. So the criterion is stated at the point where MTPA *should* pay, and
 * the nominal machine is used to state where it should not. [zh] `i_char =
 * ψf/Ld` 是决定 MTPA 值不值钱的那个无量纲开关：它就是磁阻项与磁钢项同量级
 * 的电流。`i_char/i_max = 4`（标称表贴机）时最好也就 3 %；`i_char/i_max = 1`
 * 时是 30 %。 所以判据写在 MTPA
 * **该**值钱的那个点上，并用标称机说明它在哪儿不值钱。
 *
 * 1. [en] `Lq/Ld = 2`, `i_char = i_max` -> torque at full current at least
 *    **+20 %** over `i_d = 0` (the closed-form answer is 30.2 %; 20 % leaves
 * room for the bisection and for the fact that the two are compared at equal
 *    *current*, not equal *voltage*).
 *    [zh] `Lq/Ld = 2`、`i_char = i_max` → 满电流下转矩比 `i_d = 0` 至少 **+20
 * %** （闭式解是 30.2 %；留 20 %
 * 是给二分和"两者是在等**电流**而不是等**电压**下比较的" 这件事留余量）。
 * 2. [en] `|i|` never exceeds `i_max`, at any request, including absurd ones.
 *    [zh] 任何给定下 `|i|` 都不超过 `i_max`，包括荒谬的给定。
 * 3. [en] `Lq = Ld` gives `i_d = 0` **exactly**, not "close to zero".
 *    [zh] `Lq = Ld` 下 `i_d` **恰好**为 0，不是"接近 0"。
 * 4. [en] Turning MTPA on must not push the observer past its ceiling on the
 *    salient machine: angle error <= 0.15 rad with the single-`L` observer that
 *    ships. If it does, MTPA does not ship -- a torque gain bought with a
 * broken angle estimate is not a gain. [zh] 打开 MTPA
 * 不得把观测器推过天花板：在凸极机上，出厂的单 `L` 观测器角度误差 ≤ 0.15
 * rad。若超了，MTPA 不出厂——用坏掉的角度估计换来的转矩增益不是增益。
 */
constexpr float kMinTorqueGain = 1.20F;
constexpr float kAngleCeilingRad = 0.15F;

/// [en] The salient machine the criterion is stated on. [zh]
/// 判据所针对的凸极机。
auto salient_spec() -> sil::MachineSpec {
  sil::MachineSpec s = sil::nominal_spec();
  s.ld = 2.0e-4F; // half of Lq = 2:1 saliency
  s.lq = 4.0e-4F;
  return s;
}

auto psi_f_of(const sil::MachineSpec &s) -> float {
  return s.kt / (1.5F * static_cast<float>(s.pole_pairs));
}

/// [en] Torque from a dq current, in N*m. [zh] 由 dq 电流算转矩，N·m。
auto torque_of(const sil::MachineSpec &s, core::Dq<float> i) -> float {
  return 1.5F * static_cast<float>(s.pole_pairs) *
         ((psi_f_of(s) * i.q) + ((s.ld - s.lq) * i.d * i.q));
}

auto make_reference(const sil::MachineSpec &s, float i_max)
    -> core::MtpaReference {
  return core::MtpaReference(
      core::MtpaReference::Config{s.ld, s.lq, psi_f_of(s), i_max});
}

auto request(core::MtpaReference &ref, float iq) -> core::Dq<float> {
  core::ReferenceInput in{};
  in.iq_command_a = iq;
  return ref.generate(in);
}

} // namespace

TEST(Mtpa, P10_TheGainIsWorthShippingWhereTheReluctanceTermIs) {
  const sil::MachineSpec s = salient_spec();
  const float i_char = psi_f_of(s) / s.ld;
  const float i_max = i_char; // the criterion's operating point
  auto ref = make_reference(s, i_max);
  std::printf("\n[P10] Lq/Ld = %.2f, i_char = %.2f A, i_max = %.2f A\n",
              static_cast<double>(s.lq / s.ld), static_cast<double>(i_char),
              static_cast<double>(i_max));

  // Walk the request across the current range and report the gain at each
  // point, because a single number at i_max hides whether the curve is doing
  // anything at the currents people actually use.
  // 把给定在整个电流范围走一遍、逐点报增益，因为只报 i_max
  // 一个数会掩盖"曲线在人真正用的 电流上到底有没有作用"。
  std::printf("     %8s %10s %10s %10s %8s\n", "i_req", "id mtpa", "T mtpa",
              "T id=0", "gain");
  const float probes[] = {0.25F, 0.5F, 0.75F, 1.0F, 1.5F};
  float gain_at_full = 0.0F;
  for (const float f : probes) {
    const core::Dq<float> i = request(ref, f * i_max);
    // Compared at the same *current magnitude*: `i_d = 0` with the same |i|.
    const float mag = std::sqrt((i.d * i.d) + (i.q * i.q));
    const float t_mtpa = torque_of(s, i);
    const float t_zero = torque_of(s, core::Dq<float>{0.0F, mag});
    const float gain = t_mtpa / t_zero;
    if (f >= 1.0F) {
      gain_at_full = gain;
    }
    std::printf("     %8.3f %10.4f %10.6f %10.6f %8.4f\n",
                static_cast<double>(f * i_max), static_cast<double>(i.d),
                static_cast<double>(t_mtpa), static_cast<double>(t_zero),
                static_cast<double>(gain));

    // Rule 2: the limit holds at every request, including the ones beyond it.
    // 规则 2：无论给定是什么（包括超出限幅的），限幅都成立。
    EXPECT_LE(mag, i_max * (1.0F + 1.0e-4F)) << "request " << f * i_max;
    // The curve must actually be on the negative d side: that is the whole
    // mechanism.
    // 曲线必须真的在负 d 侧：这就是全部机理。
    EXPECT_LT(i.d, 0.0F) << "request " << f * i_max;
  }

  // Rule 1: at full current the gain is at least the criterion.
  // 规则 1：满电流下增益至少达到判据。
  std::printf("     gain at full current %.2f %%, criterion %.0f %%\n",
              static_cast<double>(100.0F * (gain_at_full - 1.0F)),
              static_cast<double>(100.0F * (kMinTorqueGain - 1.0F)));
  EXPECT_GE(gain_at_full, kMinTorqueGain);
}

TEST(Mtpa, P10_AtAbsurdRequestsTheCurveIsWalkedRatherThanScaled) {
  const sil::MachineSpec s = salient_spec();
  const float i_max = 1.0F;
  auto ref = make_reference(s, i_max);

  // Walking the curve keeps "most torque per amp" true at the clamp as well;
  // clamping to `i_d = 0` does not. The difference is measurable at the clamp,
  // which is the only place a clamp policy can be judged.
  // 沿曲线走让"每安培转矩最大"在限幅处也保住；夹到 `i_d = 0`
  // 保不住。差别在限幅处可以量出来， 而限幅处是唯一能评判限幅策略的地方。
  const core::Dq<float> i = request(ref, 100.0F * i_max);
  const float mag = std::sqrt((i.d * i.d) + (i.q * i.q));
  const float t_walked = torque_of(s, i);
  const float t_flat = torque_of(s, core::Dq<float>{0.0F, i_max});
  std::printf("[P10] at the clamp: walked (%0.4f, %0.4f) T=%.6f, id=0 clamp "
              "T=%.6f, walked is %.1f %% better\n",
              static_cast<double>(i.d), static_cast<double>(i.q),
              static_cast<double>(t_walked), static_cast<double>(t_flat),
              static_cast<double>(100.0F * ((t_walked / t_flat) - 1.0F)));

  EXPECT_NEAR(i_max, mag, 1.0e-3F * i_max);
  EXPECT_GT(t_walked, t_flat);

  // And the point must still be on the curve: the closed form at the clamped
  // `i_q` has to reproduce the returned `i_d`. Without this the "walk" could be
  // any path that happens to satisfy the circle.
  // 而且这个点必须仍在曲线上：在限幅后的 `i_q` 上跑闭式，要复现返回的
  // `i_d`。没有这一条， "沿曲线走"可能只是任何一条碰巧满足圆约束的路径。
  const core::Dq<float> on_curve = request(ref, i.q);
  EXPECT_NEAR(i.d, on_curve.d, 1.0e-5F);
}

TEST(Mtpa, P10_ASurfaceMachineGetsExactlyZeroNotAnEpsilon) {
  // `Lq = Ld` is not a small saliency, it is no saliency. An epsilon in the
  // denominator would produce `i_d ~ psi_f/eps`, which is not a smaller version
  // of the right answer -- it is a different number entirely.
  // `Lq = Ld` 不是"凸极很小"，是没有凸极。在分母里放一个极小量会给出 `i_d ~
  // ψf/eps`， 那不是正确答案的小号版本，而是完全另一个数。
  sil::MachineSpec s = sil::nominal_spec();
  s.lq = s.ld;
  auto ref = make_reference(s, 3.0F);
  const core::Dq<float> i = request(ref, 1.0F);
  std::printf("[P10] surface machine (Lq == Ld): i_d = %.9f A\n",
              static_cast<double>(i.d));
  EXPECT_EQ(0.0F, i.d);

  // The nominal machine is the case MTPA should NOT pay on, and it is worth
  // recording the number so nobody later claims a gain there.
  // 标称机是 MTPA
  // **不该**值钱的情形，把这个数记下来，免得以后有人在那儿声称有增益。
  sil::MachineSpec nominal = sil::nominal_spec();
  const float i_char = psi_f_of(nominal) / nominal.ld;
  std::printf("[P10] nominal machine i_char = %.2f A, i_char/i_max = %.2f "
              "-> best possible gain well under 5 %%\n",
              static_cast<double>(i_char),
              static_cast<double>(i_char / nominal.i_max_a));
  EXPECT_GT(i_char / nominal.i_max_a, 3.0F);
}

TEST(Mtpa, P10_TurningMtpaOnDoesNotBreakTheObserverOnTheSalientMachine) {
  // Rule 4, and the one that decides whether this ships. MTPA only pays on a
  // salient machine, and the shipped observer is a single-`L` model. So the
  // question is not "is the torque gain real" but "can the library still see
  // the rotor". 规则 4，也是决定这东西出不出厂的那一条。MTPA
  // 只在凸极机上值钱，而出厂观测器是单 `L`
  // 模型。所以问题不是"转矩增益是不是真的"，而是"库还能不能看见转子"。
  const sil::MachineSpec s = salient_spec();
  sil::SwitchingRigConfig config{};
  config.spec = s;
  core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
      config.belief_rs * s.rs, s.lq, psi_f_of(s), 0.3F, 3.0F,
      0.5F * psi_f_of(s)});
  observer.seed_flux(core::Ab<float>{psi_f_of(s), 0.0F});
  const float error =
      sil::run_switched(config, [&observer](sil::SwitchingRig &rig) {
        return observer.update(rig.command(), rig.current(),
                               sil::SwitchingRig::kControlPeriod);
      });
  std::printf("[P10] Lq/Ld = %.2f, single-L observer angle error %.4f rad "
              "(ceiling %.2f)\n",
              static_cast<double>(s.lq / s.ld), static_cast<double>(error),
              static_cast<double>(kAngleCeilingRad));
  EXPECT_LT(error, kAngleCeilingRad) << "MTPA does not ship if turning on the "
                                        "machine it needs breaks the angle";
}

} // namespace foc
