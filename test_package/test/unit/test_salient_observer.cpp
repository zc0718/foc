// S21 - the saliency question, answered: the boundary is not where it was
// recorded, and the observer built to move it did not move it. This file exists
// to hold those two measurements so nobody re-derives them from memory.
// S21——凸极那个问题，答案在此：边界并不在记录的位置，而为挪开它造的那个观测器没挪开。本文件存在
// 的意义是把这两次测量留在原处，免得以后有人凭记忆重建。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/nonlinear_flux_observer.hpp>

#include "sil_bench.hpp"
#include "sil_switching_rig.hpp"

namespace foc {

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

/**
 * @brief [en] The question, the criterion written for it, and the verdict.
 *
 *     The recorded claim, from the gap-closing plan, was: "the single-`L` observer survives to `Lq/Ld = 2` (0.1433) and
 *     fails at 2.5 (0.1678)". A two-inductance observer was then built to move that boundary, against the criterion:
 *     strictly better at `Lq/Ld = 2`, inside 0.15 rad at 2.5, and no worse on a surface machine.
 *
 *     What the rig says. **With `i_d = 0` the error does not depend on the saliency ratio at all** (0.1044 / 0.1048 /
 *     0.1035 for ratios 1.0 / 2.0 / 2.5). That is arithmetic, not coincidence: a single-`L` observer's model error is
 *     `(Ld - Lq)*i_d`, so with `i_d = 0` a salient machine *is* a surface machine to it. **The boundary therefore does
 *     not exist where it was claimed to**, and the plan's entry is corrected rather than trusted.
 *
 *     The real limitation is `i_d != 0`, and it is worse than any saliency ratio: at 200 rad/s with `i_d = -1 A` the
 *     single-`L` observer reads **0.5079 rad even on a surface machine**, three times the ceiling. Same mechanism as
 *     P8: the flux target is a constant `psi_f`, so any d-axis current changes the flux it is supposed to track, and at
 *     low speed the back-EMF is too small to outvote the error. At 3200 rad/s the same d-axis current is harmless
 *     (0.0705 rad in P8), so this is a **low-speed** constraint on using `i_d` rather than a general one.
 *
 *     And the observer that was supposed to fix it: built, run, **worse than nothing**, withdrawn. Half the error of
 *     the incumbent (0.2517 against 0.4942 at `Lq/Ld = 2`) but nowhere near the 0.15 ceiling, and its residual error
 *     was flat in saliency too -- so it had not been observing the saliency, it had accidentally improved the `i_d`
 *     handling. Raising its correction rate made it worse (1.02 rad), the signature of a wrong *structure* rather than
 *     a wrong *gain*, so the header was deleted instead of tuned. The next attempt has to start from the magnitude
 *     target, not from a second inductance.
 *
 * @brief [zh] 问题、为它写下的判据、以及裁决。
 *
 *     记录在缺口计划里的主张是："单 `L` 观测器撑到 `Lq/Ld = 2`（0.1433）， 2.5 时不行（0.1678）"。随后为挪开这条边界造
 *     了一个双电感观测器，判据是：`Lq/Ld = 2` 时严格更好、2.5 时进到 0.15 rad 以内、表贴机不变差。
 *
 *     台架给出的。**`i_d = 0` 时误差与凸极比完全无关**（比值 1.0 / 2.0 / 2.5 分别是 0.1044 / 0.1048 / 0.1035）。这是算
 *     术而不是巧合：单 `L` 观测器的模型误差就是 `(Ld − Lq)·id`，所以 `i_d = 0` 时凸极机**对它而言就是**表贴机。**因此那
 *     条边界并不在 被声称的位置**，计划里的那一条被更正，而不是被沿用。
 *
 *     真正的限制是 `i_d ≠ 0`，而且它比任何凸极比都严重：200 rad/s、 `i_d = −1 A` 下，单 `L` 观测器**即使在表贴机上也读
 *     0.5079 rad**，是天花板的三倍。机理 与 P8 相同：磁链目标是常数 `ψf`，所以任何 d 轴电流都会改变它本该跟踪的那个磁
 *     链，而低速下 反电势太小，压不住这个误差。3200 rad/s 下同样的 d 轴电流无害（P8 量到 0.0705 rad），所以 这是对"能不
 *     能用 `i_d`"的**低速**限制，不是普遍限制。
 *
 *     以及那个本该修好它的观测器：造了、跑了、**比没有更糟**、撤回。误差是位者的一半 （`Lq/Ld = 2` 时 0.2517 对
 *     0.4942），但离 0.15 天花板很远，而且它的残余误差同样与凸极比 无关——所以它并没有在观测凸极，它只是碰巧改善了 `i_d`
 *     的处理。把修正速率调高反而更糟 （1.02 rad），这是**结构**错了而不是**增益**错了的特征，所以那个头文件被删掉而不是
 *     被调参。 下一次尝试必须从幅值目标出发，而不是从第二个电感出发。
 */
constexpr float kCeilingRad = 0.15F;
constexpr float kSpeedRad = 200.0F;
constexpr float kIqA = 0.5F;
constexpr float kIdA = -1.0F;

auto psi_f_of(const sil::MachineSpec &s) -> float {
  return s.kt / (1.5F * static_cast<float>(s.pole_pairs));
}

auto spec_with_ratio(float ratio) -> sil::MachineSpec {
  sil::MachineSpec s = sil::nominal_spec();
  s.lq = 4.0e-4F;
  s.ld = s.lq / ratio;
  return s;
}

auto rig_config(const sil::MachineSpec &s, float id_a)
    -> sil::SwitchingRigConfig {
  sil::SwitchingRigConfig config{};
  config.spec = s;
  config.omega_e = kSpeedRad;
  config.iq_a = kIqA;
  config.id_a = id_a;
  return config;
}

auto incumbent_error(const sil::MachineSpec &s, float id_a) -> float {
  core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
      1.05F * s.rs, s.lq, psi_f_of(s), 0.3F, 3.0F, 0.5F * psi_f_of(s)});
  observer.seed_flux(core::Ab<float>{psi_f_of(s), 0.0F});
  return sil::run_switched(
      rig_config(s, id_a), [&observer](sil::SwitchingRig &rig) {
        return observer.update(rig.command(), rig.current(),
                               sil::SwitchingRig::kControlPeriod);
      });
}

} // namespace

TEST(SalientObserver, WithZeroDaxisCurrentTheSaliencyRatioDoesNotMatter) {
  // The claim under test: "the single-L observer survives to Lq/Ld = 2 and
  // fails at 2.5". If that were true the three numbers below would climb with
  // the ratio. They do not move at all, and the reason is arithmetic rather
  // than empirical: the model error is `(Ld - Lq)*i_d`. 被测的主张："单 L
  // 观测器撑到 Lq/Ld = 2，2.5 时不行"。若那为真，下面三个数应随比值上升。
  // 它们根本不动，而原因是算术而不是经验：模型误差就是 `(Ld − Lq)·id`。
  std::printf("\n[S21] single-L at %.0f rad/s, iq = %.1f A, i_d = 0\n",
              static_cast<double>(kSpeedRad), static_cast<double>(kIqA));
  std::printf("     %-10s %12s\n", "Lq/Ld", "error rad");
  float values[3] = {0.0F, 0.0F, 0.0F};
  int index = 0;
  for (const float ratio : {1.0F, 2.0F, 2.5F}) {
    values[index] = incumbent_error(spec_with_ratio(ratio), 0.0F);
    std::printf("     %-10.2f %12.4f\n", static_cast<double>(ratio),
                static_cast<double>(values[index]));
    ++index;
  }
  // Flat to within a few percent across a 2.5x change in saliency. The
  // tolerance is loose because ripple and noise differ between the three runs;
  // the point is that there is no trend. 在凸极比变化 2.5
  // 倍的范围内平到几个百分点以内。容差宽是因为三次运行的纹波与噪声不同；要点是
  // 没有趋势。
  EXPECT_LT(std::fabs(values[2] - values[0]), 0.05F * values[0])
      << "the error must be flat in the saliency ratio when i_d = 0";
}

TEST(SalientObserver,
     NonZeroDaxisCurrentIsTheRealLimitAndItIsWorseThanSaliency) {
  // The limit that does exist, and it is not about having two inductances: it
  // is about the observer's flux target being a constant. `i_d = -1 A` costs
  // more angle than any saliency ratio does, on a *surface* machine, at low
  // speed. That is the constraint which decides whether MTPA and this observer
  // can be used together below base speed.
  // 确实存在的那个限制，而且它与"有两个电感"无关：它与观测器的磁链目标是常数有关。
  // `i_d = −1 A`
  // 在**表贴**机上、低速下付出的角度代价比任何凸极比都大。就是这条约束在决定基速
  // 以下能不能把 MTPA 和这个观测器一起用。
  const sil::MachineSpec surface = spec_with_ratio(1.0F);
  const sil::MachineSpec two = spec_with_ratio(2.0F);
  const float id_zero_surface = incumbent_error(surface, 0.0F);
  const float id_neg_surface = incumbent_error(surface, kIdA);
  const float id_neg_two = incumbent_error(two, kIdA);
  std::printf("\n[S21] the limit is i_d, not saliency (%.0f rad/s)\n",
              static_cast<double>(kSpeedRad));
  std::printf("     %-34s %12s\n", "condition", "error rad");
  std::printf("     %-34s %12.4f\n", "surface, i_d = 0",
              static_cast<double>(id_zero_surface));
  std::printf("     %-34s %12.4f\n", "surface, i_d = -1 A",
              static_cast<double>(id_neg_surface));
  std::printf("     %-34s %12.4f\n", "Lq/Ld = 2, i_d = -1 A",
              static_cast<double>(id_neg_two));

  // 1. `i_d != 0` is the dominant term: it multiplies the error by several, on
  // a machine with no
  //    saliency at all.
  // 1. `i_d ≠ 0` 是主项：在一台完全没有凸极的机器上，它就把误差放大了数倍。
  EXPECT_GT(id_neg_surface, 2.0F * id_zero_surface);
  // 2. And it takes the observer outside the ceiling on its own, which is why
  // the operating range
  //    has to be stated in terms of `i_d` and speed together.
  // 2. 而且它自己就把观测器推出天花板，所以工作范围必须把 `i_d`
  // 与转速一起说清楚。
  EXPECT_GT(id_neg_surface, kCeilingRad);
  // 3. Saliency adds a little on top, but it is not the lever -- 2.5x the
  // saliency ratio moves the
  //    error by a small fraction of what 1 A of d-axis current does.
  // 3. 凸极在此之上再加一点，但它不是那根杠杆——2.5 倍的凸极比带来的变化，只是 1
  // A d 轴电流
  //    的零头。
  EXPECT_LT(std::fabs(id_neg_two - id_neg_surface), 0.10F * id_neg_surface);
}

} // namespace foc
