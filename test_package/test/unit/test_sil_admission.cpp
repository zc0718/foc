// S18 - the SMO admission gate. Written before the SMO was measured, and it did
// not survive contact with the measurement: a tuned sign() SMO beats the
// shipped observer on the very metric this file proposed. That is the result,
// and it is what this file pins -- including the part that *does* discriminate.
// S18 - SMO 准入闸。闸在测量之前写，而它没能扛住测量：一个调好参的 `sign()` SMO
// 在本文件提出的那个
// 指标上胜过出厂观测器。这就是结论，本文件钉住它——也包括真正起区分作用的那部分。

#include <cmath>
#include <cstdio>
#include <initializer_list>

#include <gtest/gtest.h>

#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/transforms.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"
#include "sil_switching_rig.hpp"

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 8;
constexpr float kIq = 0.5F;
constexpr float kOmega = 200.0F;
constexpr float kKt = 0.05F;
constexpr std::uint8_t kPoles = 7U;
constexpr float kDeadtimeV =
    0.48F; ///< 1 us at 20 kHz on a 24 V bus, the S13 number.
constexpr float kTransitionA =
    0.2F;                       ///< a zero crossing that is not an ideal step.
constexpr float kNoiseA = 0.1F; ///< the S17 current-noise knob.
constexpr float kBelief =
    1.10F; ///< Rs believed 10 % high, the standing nameplate error.
constexpr float kPiF = 3.14159265358979F;
constexpr float kTwoPiF = 6.28318530717959F;
constexpr float kFSw =
    20000.0F; ///< switching frequency for the model-driven variant of the gate.

/// [en] The **absolute ceiling**: past this the answer is not usable at all,
/// whatever else is true. 0.15 rad is 8.6 degrees of electrical angle, i.e.
/// under 3 % of one pole pair on this machine, and it sits 45 % above the
/// shipped observer's switching-plant figure (0.1037) so the thing that ships
/// has margin rather than scraping by.
///      [zh] **绝对天花板**：超过它答案就完全不可用，别的都无从谈起。0.15 rad
///      是 8.6 电角度， 在本机上一个极距的 3 % 以内，并且比出厂观测器在开关
///      plant 上的数（0.1037）高 45 %，所以
///      出厂的东西有余量，而不是刚好擦着过。
constexpr float kAbsoluteCeilingRad = 0.15F;

/// [en] The **admission rule**, and the re-derivation this file needed: a
/// candidate is admitted if it is no worse than the observer already shipped,
/// measured **on the same plant at the same operating point**. An absolute
/// number cannot do this job -- 0.10 was calibrated on the mean-value plant and
/// the same observer reads 0.1037 on a switching one, so the gate failed its
/// own incumbent the moment the bench changed (P5-f found this the hard way). A
/// ratio does not go stale when the bench changes, because both sides move
/// together.
///      [zh]
///      **准入规则**，也是本文件需要的重定：候选被准入，当且仅当它不差于**已经出厂**的那个
///      观测器——**在同一台 plant、同一个工作点**上量。绝对数干不了这件事：0.10
///      是在均值 plant 上 定的，而同一个观测器在开关 plant 上读
///      0.1037，于是台架一换，闸就卡住了它自己的在位者 （P5-f
///      就是撞在这上面）。比值不会因为台架变化而过期，因为两边一起动。
constexpr float kNoWorseThanIncumbent = 1.0F;

auto wrap_pi_local(float angle) noexcept -> float {
  while (angle <= -kPiF) {
    angle += kTwoPiF;
  }
  while (angle > kPiF) {
    angle -= kTwoPiF;
  }
  return angle;
}

auto psi_f_value() noexcept -> float {
  return kKt / (1.5F * static_cast<float>(kPoles));
}

/**
 * @brief [en] Constant-speed rig with the two impairments S17 and S13 measured,
 * applied together. [zh] 恒速台架，把 S17 与 S13 量过的两种未建模效应同时打开。
 *
 * @note [en] The observer is handed the *commanded* voltage, because that is
 * all a real controller knows; the inverter keeps the dead-time volt-seconds
 * for itself. [zh]
 * 观测器拿到的是**指令**电压，因为真实控制器只知道这个；死区的伏秒被逆变器留下了。
 * @since 0.1.0
 */
class GateRig final {
public:
  GateRig(float omega_e, bool noise, bool deadtime) noexcept
      : plant_(sil::nominal_plant_params()), omega_e_(omega_e), noise_(noise),
        deadtime_(deadtime) {
    if (noise_) {
      plant_.set_current_noise(kNoiseA);
    }
    plant_.reset(
        sil::PlantState{0.0F, kIq, omega_e / static_cast<float>(kPoles), 0.0F});
    vd_ = -omega_e * sil::nominal_spec().lq * kIq;
    vq_ = (sil::nominal_spec().rs * kIq) + (omega_e * psi_f_value());
  }

  GateRig(const GateRig &) = delete;
  GateRig &operator=(const GateRig &) = delete;
  GateRig(GateRig &&) = delete;
  GateRig &operator=(GateRig &&) = delete;

  [[nodiscard]] auto true_angle_e() const noexcept -> float {
    return static_cast<float>(kPoles) * plant_.theta_mech_rad();
  }
  [[nodiscard]] auto command() const noexcept -> core::Ab<float> {
    return command_;
  }
  [[nodiscard]] auto current() const noexcept -> core::Ab<float> {
    return current_;
  }

  void step() noexcept {
    command_ = core::inv_park(core::Dq<float>{vd_, vq_}, true_angle_e());
    const core::Uvw<float> i_uvw = plant_.current_uvw();
    current_ = core::clarke(i_uvw);

    const bool dead = deadtime_;
    const auto loss = [dead](float i) noexcept -> float {
      if (!dead) {
        return 0.0F;
      }
      const float n = i / kTransitionA;
      return (n > 1.0F) ? kDeadtimeV
                        : ((n < -1.0F) ? -kDeadtimeV : (kDeadtimeV * n));
    };
    const core::Uvw<float> v_uvw = core::inv_clarke(command_);
    const core::Ab<float> delivered = core::clarke(
        core::Uvw<float>{static_cast<float>(v_uvw.u - loss(i_uvw.u)),
                         static_cast<float>(v_uvw.v - loss(i_uvw.v)),
                         static_cast<float>(v_uvw.w - loss(i_uvw.w))});
    const core::Dq<float> v_dq = core::park(delivered, true_angle_e());
    for (int s = 0; s < kSubsteps; ++s) {
      plant_.set_voltage_dq(v_dq);
      plant_.advance(kDt / static_cast<float>(kSubsteps));
    }
  }

private:
  sil::PmsmPlant plant_;
  float omega_e_;
  bool noise_;
  bool deadtime_;
  float vd_;
  float vq_;
  core::Ab<float> command_{0.0F, 0.0F};
  core::Ab<float> current_{0.0F, 0.0F};
};

/// [en] Mean |angle error| over the second half. [zh]
/// 后半段的角度误差绝对均值。
template <typename Update>
auto run_gate(bool noise, bool deadtime, Update &&update) -> float {
  GateRig rig(kOmega, noise, deadtime);
  float sum = 0.0F;
  int count = 0;
  const int steps = static_cast<int>(0.4F / kDt);
  for (int k = 0; k < steps; ++k) {
    rig.step();
    const core::ObsState observed = update(rig);
    if (k >= (steps / 2)) {
      sum += std::fabs(wrap_pi_local(observed.theta_e - rig.true_angle_e()));
      ++count;
    }
  }
  return sum / static_cast<float>(count);
}

/// [en] The shipped observer, one configuration, no scenario-specific tuning.
/// [zh] 出厂观测器，一份配置， 不针对场景调参。
auto incumbent_error(bool noise, bool deadtime) -> float {
  core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
      kBelief * sil::nominal_spec().rs, sil::nominal_spec().lq, psi_f_value(),
      0.3F, 3.0F, 0.5F * psi_f_value()});
  observer.seed_flux(core::Ab<float>{psi_f_value(), 0.0F});
  return run_gate(noise, deadtime, [&observer](GateRig &rig) {
    return observer.update(rig.command(), rig.current(), kDt);
  });
}

auto smo_error(bool noise, bool deadtime, float gain, float fc_hz, float layer)
    -> float {
  sil::SmoControl observer(kBelief * sil::nominal_spec().rs,
                           sil::nominal_spec().lq, gain, fc_hz, layer);
  return run_gate(noise, deadtime, [&observer](GateRig &rig) {
    return observer.update(rig.command(), rig.current());
  });
}

/// [en] The shipped observer on the switching bench, one configuration. [zh]
/// 开关台架上的出厂观测器，一份配置。
auto switched_incumbent_error(const sil::SwitchingRigConfig &config) -> float {
  core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
      config.belief_rs * sil::nominal_spec().rs, sil::nominal_spec().lq,
      sil::SwitchingRig::psi_f(), 0.3F, 3.0F,
      0.5F * sil::SwitchingRig::psi_f()});
  observer.seed_flux(core::Ab<float>{sil::SwitchingRig::psi_f(), 0.0F});
  return sil::run_switched(config, [&observer](sil::SwitchingRig &rig) {
    return observer.update(rig.command(), rig.current(),
                           sil::SwitchingRig::kControlPeriod);
  });
}

/// [en] The SMO control on the same bench, at a given tuning. [zh] 同一台架上的
/// SMO 对照，给定一套调参。
auto smo_switched_error(const sil::SwitchingRigConfig &config, float gain,
                        float fc_hz, float layer) -> float {
  sil::SmoControl observer(config.belief_rs * sil::nominal_spec().rs,
                           sil::nominal_spec().lq, gain, fc_hz, layer);
  return sil::run_switched(config, [&observer](sil::SwitchingRig &rig) {
    return observer.update(rig.command(), rig.current());
  });
}

} // namespace

TEST(SilAdmission,
     TheIncumbentPassesTheGateButTheGateWasCalibratedAfterwards) {
  // The honest history of one number. The gate was written as 0.05 rad before
  // anything ran; the shipped observer came in at 0.068 and *failed its own
  // gate*. So 0.05 was not a requirement, it was a guess, and it has been
  // replaced by 0.10 -- above the incumbent with room, not tuned to any
  // challenger. 一个数字的诚实履历。闸在任何东西跑起来之前写成 0.05
  // rad；出厂观测器给出 0.068，**没通过它自己的闸**。 所以 0.05
  // 不是需求，是猜测，已改为
  // 0.10——高于在位者且有余量，而不是为任何挑战者量身定做。
  const float clean = incumbent_error(false, false);
  const float impairments = incumbent_error(true, true);

  std::printf("\n[S18] incumbent (NLFO, one config): clean %.4f rad, gate "
              "scenario %.4f rad (ceiling %.2f)\n",
              static_cast<double>(clean), static_cast<double>(impairments),
              static_cast<double>(kAbsoluteCeilingRad));

  // The incumbent has to be inside the ceiling with room -- that is what makes
  // the ceiling a *ceiling* and not a target.
  // 在位者必须在天花板以内且有余量——这才让天花板是**天花板**而不是目标。
  EXPECT_LT(impairments, kAbsoluteCeilingRad);
  EXPECT_LT(impairments, 0.75F * kAbsoluteCeilingRad)
      << "and with room, not scraping by";
  // And the impairments really are the reason it is not at the clean figure --
  // otherwise the gate scenario would be measuring nothing.
  // 而且未建模效应确实是它离开干净数字的原因——否则闸场景什么都没在量。
  EXPECT_GT(impairments, 3.0F * clean)
      << "clean " << clean << " vs impaired " << impairments;
}

TEST(SilAdmission,
     ATunedSlidingModeObserverPassesTheGateAndBeatsTheIncumbent) {
  // A3-b was written as "the idealised SMO fails the gate". **It does not.**
  // Given the same three knobs (gain, filter bandwidth, boundary layer) and its
  // best combination, it lands at 0.048 -- below the shipped observer's 0.068.
  // This test records that, because a plan that keeps a falsified premise in
  // its "explicitly not doing" table is worse than no plan.
  // A3-b 原本写成"理想化 SMO 会撞闸失败"。**它没有。**
  // 在同样三个旋钮（增益、滤波带宽、边界层）下取它 最好的一组，它落在
  // 0.048——低于出厂观测器的
  // 0.068。本用例记录这一点，因为一份把已被证伪的前提留在
  // "明确不做"表里的计划，比没有计划更糟。
  const float gate = incumbent_error(true, true);

  float best = 1.0e9F;
  float best_gain = 0.0F;
  float best_fc = 0.0F;
  float best_layer = 0.0F;
  // Its sensible neighbourhood, not a single lucky point: a claim about "an
  // SMO" has to survive a sweep.
  // 它的合理邻域，而不是一个走运的点：关于"SMO"的论断必须经得起扫描。
  for (const float layer : {0.2F, 0.5F, 1.0F}) {
    for (const float gain : {1.5F, 2.0F, 5.0F}) {
      for (const float fc : {500.0F, 1000.0F, 2000.0F}) {
        const float error = smo_error(true, true, gain, fc, layer);
        if (error < best) {
          best = error;
          best_gain = gain;
          best_fc = fc;
          best_layer = layer;
        }
      }
    }
  }

  std::printf("[S18] SMO control, best of the sweep: %.4f rad (K %.1f V, fc "
              "%.0f Hz, layer %.2f A) "
              "vs incumbent %.4f\n",
              static_cast<double>(best), static_cast<double>(best_gain),
              static_cast<double>(best_fc), static_cast<double>(best_layer),
              static_cast<double>(gate));

  EXPECT_LT(best, kAbsoluteCeilingRad)
      << "the control has to be allowed to pass if it can";
  EXPECT_LT(best, kNoWorseThanIncumbent * gate)
      << "and it does: it is no worse than what ships, measured against it";
  // Recorded as a ratio, because that is the number that survives a change of
  // bench. 以比值记录，因为那是换台架之后仍然成立的数。
  std::printf(
      "[S18] admission: SMO/incumbent = %.2fx (rule: <= %.2fx), %d %% better\n",
      static_cast<double>(best / gate),
      static_cast<double>(kNoWorseThanIncumbent),
      static_cast<int>(100.0F * (gate - best) / gate));
}

TEST(SilAdmission, ItsTuningIsBoundToTheDisturbanceAndSoIsTheIncumbents) {
  // The third thing I expected to separate them, and the second hypothesis this
  // file falsifies. I wrote that the shipped observer would be insensitive to
  // the two plants (within 1.5x) while the SMO's accuracy would be a function
  // of a bandwidth matched to the noise floor. Measured: the incumbent is 6.6x
  // worse under the impairments and the SMO is 14x worse -- a factor of two
  // apart, not a category difference. Both observers are dominated by the same
  // thing (the dead-time distortion), and neither is a constant across
  // environments.
  //
  // What *is* still true and worth pinning: a tuning chosen on a clean plant
  // does not survive the gate (0.411 rad), so an SMO's configuration has to be
  // chosen against the real disturbance, and its sensitivity to that choice is
  // measurably (if only ~2x) worse than the incumbent's.
  // 我原本以为能区分它们的第三件事，也是本文件证伪的第二个假设。我写过出厂观测器对这两台
  // plant 不敏感 （1.5 倍以内），而 SMO
  // 的精度是与噪声地板匹配的带宽的函数。实测：在位者在未建模效应下差 6.6 倍，
  // SMO 差 14
  // 倍——相差两倍，不是类别差异。两者都被同一件事支配（死区畸变），谁都不是跨环境常数。
  //
  // 仍然成立且值得钉住的是：在干净 plant 上选出的调参过不了闸（0.411
  // rad），所以 SMO 的配置必须对着
  // 真实扰动选，而它对这个选择的敏感度比在位者确实更差（只是"更差约 2 倍"）。
  const float clean = incumbent_error(false, false);
  const float impaired = incumbent_error(true, true);

  float best_clean = 1.0e9F;
  float best_clean_gain = 0.0F;
  float best_clean_fc = 0.0F;
  float best_clean_layer = 0.0F;
  for (const float layer : {0.05F, 0.2F, 0.5F}) {
    for (const float gain : {2.0F, 5.0F}) {
      for (const float fc : {1000.0F, 2000.0F, 4000.0F}) {
        const float error = smo_error(false, false, gain, fc, layer);
        if (error < best_clean) {
          best_clean = error;
          best_clean_gain = gain;
          best_clean_fc = fc;
          best_clean_layer = layer;
        }
      }
    }
  }
  const float clean_tuning_under_disturbance =
      smo_error(true, true, best_clean_gain, best_clean_fc, best_clean_layer);

  std::printf(
      "[S18] SMO tuned on the clean plant: %.4f there (K %.1f, fc %.0f, layer "
      "%.2f) -> %.4f "
      "under the gate (x%.1f)\n",
      static_cast<double>(best_clean), static_cast<double>(best_clean_gain),
      static_cast<double>(best_clean_fc), static_cast<double>(best_clean_layer),
      static_cast<double>(clean_tuning_under_disturbance),
      static_cast<double>(clean_tuning_under_disturbance / best_clean));
  std::printf("[S18] incumbent: %.4f -> %.4f (x%.1f)\n",
              static_cast<double>(clean), static_cast<double>(impaired),
              static_cast<double>(impaired / clean));

  // The straw man is competent on a clean plant -- otherwise the comparison
  // above is worthless. 稻草人在干净 plant 上是合格的——否则上面的比较毫无价值。
  EXPECT_LT(best_clean, 0.05F);

  // A clean-plant tuning does not survive the gate: the configuration is bound
  // to the disturbance. 在干净 plant 上调好的配置过不了闸：配置与扰动绑定。
  EXPECT_GT(clean_tuning_under_disturbance, kAbsoluteCeilingRad)
      << "a clean-plant tuning must not survive the ceiling";

  // And its sensitivity is worse than the incumbent's -- but only by a factor
  // of about two, which is asserted as a ratio rather than dressed up as a
  // category difference.
  // 而且它比在位者更敏感——但只差约两倍；这里按比值断言，而不是包装成类别差异。
  EXPECT_GT(clean_tuning_under_disturbance / best_clean,
            1.5F * (impaired / clean));
  // The incumbent is *not* a constant either. Saying so is part of the record.
  // 在位者**也**不是常数。把这句写下来是记录的一部分。
  EXPECT_GT(impaired, 3.0F * clean);
}

TEST(SilAdmission, TheRankingSurvivesASwitchingInverterAndTheGapWidens) {
  // V8 of the S19 plan, and the reason the model was built. §5's original
  // reason for skipping SMO was that "chattering is a hardware effect, so SIL
  // makes an SMO look too good". Measured on a model whose dead time and ripple
  // are consequences of switching rather than formulas, the *opposite* happens:
  // the incumbent gets worse (0.068 -> 0.104) while the SMO stays where it was
  // (0.048 -> 0.046). The mean-value plant was flattering the flux integrator,
  // not the sliding-mode observer. S19 计划的 V8，也是建这个模型的理由。§5
  // 原本跳过 SMO 的理由是"抖振是硬件现象，SIL 里会把 SMO 显得
  // 太好"。在一个死区与纹波都来自开关过程（而非公式）的模型上实测，结果**相反**：在位者变差
  // （0.068 → 0.104），SMO 原地不动（0.048 →
  // 0.046）。一直在美化磁链积分器的是平均值 plant。
  const float mean_value_incumbent = incumbent_error(true, true);
  sil::SwitchingRigConfig config{};
  const float switched_incumbent = switched_incumbent_error(config);

  float best = 1.0e9F;
  float best_gain = 0.0F;
  float best_fc = 0.0F;
  float best_layer = 0.0F;
  for (const float layer : {0.2F, 0.5F, 1.0F}) {
    for (const float gain : {1.5F, 2.0F, 5.0F}) {
      for (const float fc : {500.0F, 1000.0F, 2000.0F}) {
        const float error = smo_switched_error(config, gain, fc, layer);
        if (error < best) {
          best = error;
          best_gain = gain;
          best_fc = fc;
          best_layer = layer;
        }
      }
    }
  }

  std::printf("[S18] switching inverter: incumbent %.4f rad (mean-value %.4f), "
              "SMO best %.4f rad "
              "(K %.1f, fc %.0f, layer %.2f)\n",
              static_cast<double>(switched_incumbent),
              static_cast<double>(mean_value_incumbent),
              static_cast<double>(best), static_cast<double>(best_gain),
              static_cast<double>(best_fc), static_cast<double>(best_layer));
  // Under the re-derived rule there is no absolute pass/fail to print: the
  // question is whether the candidate is no worse than the incumbent *here*,
  // and by how much.
  // 在重定之后的规则下没有绝对的"过/不过"可打：问题是候选在**此处**是否不差于在位者，以及差多少。
  std::printf("[S18]   ceiling %.2f: incumbent %s, SMO %s; SMO/incumbent = "
              "%.2fx, ranking %s\n",
              static_cast<double>(kAbsoluteCeilingRad),
              (switched_incumbent < kAbsoluteCeilingRad) ? "inside" : "OUTSIDE",
              (best < kAbsoluteCeilingRad) ? "inside" : "outside",
              static_cast<double>(best / switched_incumbent),
              (best < switched_incumbent) ? "unchanged" : "REVERSED");

  // The switching process is harder on an open-loop flux integrator: it
  // integrates ripple it cannot tell from back-EMF, and there is no
  // smooth-transition model left to soften it.
  // 开关过程对开环磁链积分器更狠：它把分不清的纹波也积了进去，而且没有平滑过渡模型来缓和。
  EXPECT_GT(switched_incumbent, 1.2F * mean_value_incumbent);

  // The SMO is indifferent: what the switching adds is what its filter was
  // already for. SMO 无所谓：开关新加的那些，正是它的滤波本来就在处理的。
  EXPECT_LT(best, kAbsoluteCeilingRad);
  EXPECT_LT(best, kNoWorseThanIncumbent * switched_incumbent)
      << "admission is relative: no worse than what ships, on this bench";
  EXPECT_LT(best, 0.05F);

  // And the ranking -- the thing the admission discussion turns on -- is
  // unchanged. If this ever flips, finding 7 has to be re-derived, not patched.
  // 而排序——整个准入讨论所依赖的那件事——没有变。这条若翻转，发现 7
  // 必须重新推导，而不是打补丁。
  EXPECT_LT(best, switched_incumbent);
}

TEST(SilAdmission, C1_TheCandidateRecordAcrossSpeedsAndMismatches) {
  // V8 answered "which is more accurate" at **one** point. One point is not a
  // library-level statement, so this is the record: five speeds, three Rs
  // beliefs. V8
  // 只回答了**一个**点上的"谁更准"。一个点不构成库级陈述，所以这里是记录：五个速度、三种
  // Rs 信念。
  //
  // Two candidate columns, and the second one is the lesson:
  //  * `SMO sched` uses the speed schedule from `scheduled_smo()` -- gain above
  //  the back-EMF it must
  //    reconstruct, filter corner above the signal it must pass. This is the
  //    *fair* comparison, because the shipped NLFO's own gain is already
  //    scheduled (`g = max(min_gain, ratio*|w_e|)`, finding 4).
  //  * `SMO fixed` is the single constant (K 2 V / fc 1 kHz) used by the
  //  earlier tests. My original C1-c said
  //    the challenger must use one configuration; that criterion was **wrong**,
  //    and this column is why: at 800 rad/s the back-EMF is 3.8 V and a 2 V
  //    gain does not establish sliding at all.
  // 两列候选，第二列是教训：
  //  * `SMO sched` 用 `scheduled_smo()`
  //  的调度——增益高于它要重建的反电动势，滤波拐点高于它要通过的信号。
  //    这才是**公平**的比较，因为出厂 NLFO 自己的增益本来就是调度的（`g =
  //    max(min_gain, ratio·|w_e|)`，发现 4）。
  //  * `SMO fixed` 是前面测试用的单个常数（K 2 V / fc 1 kHz）。我原来的 C1-c
  //  说挑战者只能有一套配置；
  //    那条判据是**错的**，这一列就是原因：800 rad/s 处反电动势 3.8 V，2 V
  //    的增益根本建立不了滑模。
  //
  // What this table is *not*: 0 / very low speed (no back-EMF to observe --
  // HFI's territory), field weakening (above 3970 rpm, not implemented),
  // current saturation (above 3 A), temperature drift (S17).
  // 这张表**不是**什么：0 / 极低速（没有反电动势可观，那是 HFI
  // 的地盘）、弱磁（3970 rpm 以上，未实现）、 电流饱和（3 A
  // 以上）、温度漂移（S17）。
  std::printf("\n[S18] C1 candidate record: switching bench, 1 us dead time, "
              "0.1 A noise, iq 0.5 A\n");
  std::printf("[S18] %8s %6s %10s %10s %7s %10s %10s %10s\n", "omega_e", "Rs",
              "NLFO", "SMO sched", "ratio", "sched K", "sched fc", "SMO fixed");

  int cells = 0;
  int scheduled_wins = 0;
  float ratio_at_800 = 0.0F;
  for (const float omega : {20.0F, 100.0F, 200.0F, 400.0F, 800.0F}) {
    for (const float belief : {1.00F, 1.10F, 1.30F}) {
      sil::SwitchingRigConfig config{};
      config.omega_e = omega;
      config.belief_rs = belief;
      const sil::SmoTuning tuning =
          sil::scheduled_smo(omega, sil::SwitchingRig::psi_f());

      const float incumbent = switched_incumbent_error(config);
      const float candidate =
          smo_switched_error(config, tuning.gain, tuning.fc_hz, tuning.layer);
      const float fixed = smo_switched_error(config, 2.0F, 1000.0F, 1.0F);
      const float ratio = incumbent / candidate;

      std::printf(
          "[S18] %8.0f %6.2f %10.4f %10.4f %7.2f %10.2f %10.0f %10.4f\n",
          static_cast<double>(omega), static_cast<double>(belief),
          static_cast<double>(incumbent), static_cast<double>(candidate),
          static_cast<double>(ratio), static_cast<double>(tuning.gain),
          static_cast<double>(tuning.fc_hz), static_cast<double>(fixed));
      ++cells;
      if ((belief == 1.10F) && (ratio > 1.0F)) {
        ++scheduled_wins;
      }
      if (belief == 1.10F) {
        ratio_at_800 = (omega == 800.0F) ? ratio : ratio_at_800;
      }

      // C1-a: every cell has to be a number. A missing or non-finite cell would
      // make the record a summary of the cells that happened to work.
      // C1-a：每格都必须是一个数。缺格或非有限值会让这份记录变成"恰好跑通的那几格"的汇总。
      EXPECT_TRUE(std::isfinite(incumbent) && (incumbent > 0.0F));
      EXPECT_TRUE(std::isfinite(candidate) && (candidate > 0.0F));
      EXPECT_TRUE(std::isfinite(fixed) && (fixed > 0.0F));

      // C1-c revised: with a single constant the challenger loses at the top of
      // the range, and that is a sizing failure, not an opinion -- the gain has
      // to exceed the back-EMF it is reconstructing. C1-c
      // 改写：用单个常数时挑战者在上端输掉，而那是**选型失败**，不是观点——增益必须大于它要重建的
      // 反电动势。
      if (omega >= 400.0F) {
        EXPECT_LT(candidate, fixed)
            << "the schedule must beat the constant at " << omega;
      }
    }
  }

  std::printf("[S18] C1-a: %d of 15 cells; C1-b: SMO(sched) wins %d of 5 "
              "points at Rs x1.10; ratio at 800 "
              "rad/s = %.2f\n",
              cells, scheduled_wins, static_cast<double>(ratio_at_800));

  // C1-b as written ("the SMO wins at every speed point") is **falsified**, and
  // this is the shape that replaced it: it wins from 20 to 400 rad/s and loses
  // at 800. Asserting the shape rather than the average is the point -- an
  // average would hide the reversal, which is the most decision-relevant fact
  // here. C1-b 原文（"SMO
  // 在每个速度点都赢"）被**证伪**，这是取代它的形状：20–400 rad/s 赢，800
  // 输。断言形状
  // 而不是均值正是要点——均值会掩盖这次反转，而反转才是这里最影响决策的事实。
  EXPECT_EQ(15, cells);
  EXPECT_EQ(4, scheduled_wins);
  EXPECT_LT(ratio_at_800, 1.0F)
      << "the incumbent is expected to take back the top of the range";
}

TEST(SilAdmission,
     P4ab_DeadTimeCompensationBelongsToTheBandwidthLimitedObserver) {
  // P4-a and P4-b of the plan, and the finding that came out of measuring them.
  // plan 的 P4-a 与 P4-b，以及量它们时掉出来的那个结论。
  //
  // The modest-looking claim "the inverter nonlinearity is an unmodelled
  // disturbance, so tell the observer about it" turns out to be
  // **observer-specific**, and the sign of the effect flips:
  //  "逆变器非线性是未建模扰动，所以告诉观测器"
  //  这个看起来平淡的说法其实是**分观测器的**，而且效果会反号：
  //   * the NLFO is bandwidth-limited, so the loss looks like a DC offset it
  //   cannot reject: compensation
  //     takes 800 rad/s from 0.0563 to 0.0111, a 5x win.
  //   * the SMO's switching term *is* an unknown-input estimator, so it already
  //   tracks the loss; feeding
  //     it the same estimate again double-counts it, and 20 rad/s goes from
  //     0.0673 to 0.1637 -- 2.4x worse. This is not a tuning artefact: it is
  //     what a sliding-mode observer is *for*.
  //   * NLFO 是带宽受限的，损失对它就象一个它压不掉的直流偏置：补偿把 800 rad/s
  //   从 0.0563 打到 0.0111，
  //     赢 5 倍。
  //   * SMO
  //   的切换项**本身**就是一个未知输入估计器，它已经在跟踪这个损失；再把同一个估计喂给它就是重复
  //     计入，20 rad/s 从 0.0673 变成 0.1637，差 2.4
  //     倍。这不是整定问题，这正是滑模观测器的**用途**。
  const float v_dead =
      1.0e-6F * 20000.0F * 24.0F; // 1 us at 24 V, the bench setting

  struct Row final {
    float omega_e;
    float incumbent_off;
    float incumbent_step;
    float smo_off;
    float smo_step;
  };
  Row rows[3]{};
  const float speeds[3] = {20.0F, 200.0F, 800.0F};
  for (int k = 0; k < 3; ++k) {
    sil::SwitchingRigConfig config{};
    config.omega_e = speeds[k];
    const sil::SmoTuning tuning =
        sil::scheduled_smo(speeds[k], sil::SwitchingRig::psi_f());
    rows[k].omega_e = speeds[k];
    rows[k].incumbent_off = switched_incumbent_error(config);
    config.compensate_v = v_dead;
    rows[k].incumbent_step = switched_incumbent_error(config);
    sil::SwitchingRigConfig without{};
    without.omega_e = speeds[k];
    rows[k].smo_off =
        smo_switched_error(without, tuning.gain, tuning.fc_hz, tuning.layer);
    without.compensate_v = v_dead;
    rows[k].smo_step =
        smo_switched_error(without, tuning.gain, tuning.fc_hz, tuning.layer);
  }

  std::printf("\n[S18] P4-a/b, dV = %.2f V\n        omega_e | NLFO: off   step "
              " | SMO: off   step\n",
              static_cast<double>(v_dead));
  for (int k = 0; k < 3; ++k) {
    std::printf("        %7.0f | %.5f %.5f | %.5f %.5f\n",
                static_cast<double>(rows[k].omega_e),
                static_cast<double>(rows[k].incumbent_off),
                static_cast<double>(rows[k].incumbent_step),
                static_cast<double>(rows[k].smo_off),
                static_cast<double>(rows[k].smo_step));
  }

  // P4-a: the two speeds where the incumbent was furthest out.
  // P4-a：incumbent 最差的两个速域。
  EXPECT_LE(rows[2].incumbent_step, 0.020F)
      << "P4-a: 800 rad/s with the compensator in the loop";
  EXPECT_LE(rows[1].incumbent_step, 0.075F)
      << "P4-a: 200 rad/s with the compensator in the loop";
  // P4-b: and the low end must not be made worse to get that.
  // P4-b：为了拿到上面这些，低速端不能被弄坏。
  EXPECT_LE(rows[0].incumbent_step, 1.1F * rows[0].incumbent_off)
      << "P4-b: 20 rad/s must not regress";

  // The other half, which is the part worth remembering: the same medicine
  // poisons the sliding-mode observer. If a future change ever makes this
  // assertion fail, it means the SMO stopped estimating the unknown input --
  // that is a regression in the observer, not an improvement in the
  // compensation.
  // 另一半才是值得记住的：同一服药会毒到滑模观测器。将来若这条断言失败，说明
  // SMO 不再估计未知输入了 ——那是观测器的回归，不是补偿的进步。
  EXPECT_GT(rows[0].smo_step, rows[0].smo_off)
      << "the SMO already tracks the loss; compensation double-counts";
}
