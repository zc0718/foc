// S16 - full speed range: saliency at standstill, back-EMF at speed, and the
// handover between them. S16 -
// 全速域：静止靠凸极、高速靠反电动势，以及两者之间的交接。

#include <cmath>
#include <cstdio>
#include <initializer_list>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/blended_observer.hpp>
#include <foc/core/observer/hfi_observer.hpp>
#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/transforms.hpp>

#include "sil_pmsm_plant.hpp"

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

constexpr float kPiFloat = 3.14159265358979F;
constexpr float kTwoPiFloat = 6.28318530717959F;

constexpr float kRs = 0.35F;
constexpr float kLd = 2.0e-4F;
constexpr float kLq = 4.0e-4F;
constexpr float kKt = 0.05F;
constexpr std::uint8_t kPoles = 7U;

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 8;
constexpr float kIq = 0.5F;

// The back-EMF observer is told Rs is 10 % high. That is not a rigged failure,
// it is the reason the blend exists: a flux integrator with an exact nameplate
// would look perfect at every speed, and the handover would then be resting on
// a property real hardware does not have. 反电动势观测器被告知 Rs 偏高
// 10%。这不是在制造失败，而是混合存在的理由：铭牌完全准确的磁链积分器在
// 任何转速下都看着完美，那样交接就建立在一个真实硬件并不具备的性质上。
constexpr float kRsSeenByFluxObserver = 1.10F * kRs;

// Inside the band the two estimates differ by a little, so the blend is an
// average of two nearly-equal numbers and is slightly *worse* than either
// alone. The threshold says so instead of pretending the handover is free.
// 带内两个估计略有差异，融合就是两个近乎相等的数的平均，因此比单独任一个都**略差**。阈值如实反映这一点，
// 而不是假装交接没有代价。
constexpr float kBlendTolerance = 0.05F;

// Electrical acceleration of the trajectory: fast for a joint, slow for the
// observers. 轨迹的电角加速度：对关节算快，对观测器算慢。
constexpr float kRampRate = 500.0F;

// Where each observer alone is unusable, measured on this fixture. These are
// the numbers the band was chosen from, not guesses.
// 单独使用各自不可用的位置，在本台架上实测。带宽就是据此选的，不是猜的。
constexpr float kHfiFailsAbove = 160.0F;
constexpr float kFluxFailsBelow = 20.0F;

auto plant_params() -> sil::PlantParams {
  return sil::PlantParams{kRs, kLd, kLq, kKt, 1.0e6F, 0.0F, kPoles};
}

auto wrap_pi_local(float angle) noexcept -> float {
  while (angle <= -kPiFloat) {
    angle += kTwoPiFloat;
  }
  while (angle > kPiFloat) {
    angle -= kTwoPiFloat;
  }
  return angle;
}

auto hfi_config(float inject_v = 2.0F) -> core::HfiObserver::Config {
  core::HfiObserver::Config config{};
  config.inject_v = inject_v;
  config.inject_hz = 2000.0F;
  config.current_hp_hz = 300.0F;
  config.demod_lpf_hz = 200.0F;
  config.pll_bandwidth_hz = 40.0F;
  config.pll_zeta = 0.9F;
  config.demod_gain_a = core::HfiObserver::predicted_gain_a(
      config.inject_v, config.inject_hz, kLd, kLq);
  config.min_hf_current_a = 0.05F;
  return config;
}

auto flux_config() -> core::BackEmfObserver::Config {
  // A single-L model, given Lq: the machine is driven with id ~ 0, so the
  // current the correction term has to account for sits on the q axis. 单一 L
  // 的模型，取 Lq：电机以 id ≈ 0 运行，于是修正项要抵掉的那部分电流落在 q
  // 轴上。
  return core::BackEmfObserver::Config{kRsSeenByFluxObserver, kLq};
}

using Blend = core::BlendedObserver<core::HfiObserver, core::BackEmfObserver>;

auto blend_config() -> Blend::Config {
  // 60..120 rad/s electrical, and the end is placed by two measured facts that
  // pull in opposite directions. The saliency loop locks to the mirror
  // somewhere between 160 and 200, so the handover must be *finished* well
  // before that. And the criterion reads the flux observer's own speed, which
  // sits 6 % low or high depending on how it got there, so finishing exactly at
  // the edge is not possible either. 60..120 leaves both margins. 60–120 rad/s
  // 电角速度，上端由两个方向相反、都是实测的事实决定：凸极环在 160 与 200
  // 之间锁到镜像，
  // 因此交接必须明显**结束**在那个边界之前；而判据读的是反电动势观测器自己的转速，它按来路偏低或偏高
  // 约 6%，所以也没法正好卡在边界上结束。60–120 两边都留了余量。
  return Blend::Config{60.0F, 120.0F, 20.0F};
}

/**
 * @brief [en] Constant-speed rig. The rotor is held at a fixed speed with a huge inertia and fed the steady-state
 *             voltage for `id = 0, iq = kIq`; what the observers see is then a real operating point rather than a
 *             spinning flux with no current in it.
 * @brief [zh] 恒速台架。转子以极大惯量固定在给定转速，并施加 `id = 0, iq = kIq` 的稳态电压；于是观测器 看到的是真实工作
 *             点，而不是一个没有电流的空转磁链。
 * @since 0.1.0
 */
class SpeedRig final {
public:
  explicit SpeedRig(float omega_e) noexcept
      : plant_(plant_params()), omega_e_(omega_e), iq_(kIq) {
    plant_.reset(
        sil::PlantState{0.0F, kIq, omega_e / static_cast<float>(kPoles), 0.0F});
    const float psi_f = kKt / (1.5F * static_cast<float>(kPoles));
    vd_ = -omega_e * kLq * kIq;
    vq_ = (kRs * kIq) + (omega_e * psi_f);
  }

  SpeedRig(const SpeedRig &) = delete;
  SpeedRig &operator=(const SpeedRig &) = delete;
  SpeedRig(SpeedRig &&) = delete;
  SpeedRig &operator=(SpeedRig &&) = delete;

  [[nodiscard]] auto psi_f() const noexcept -> float {
    return kKt / (1.5F * static_cast<float>(kPoles));
  }

  [[nodiscard]] auto true_angle_e() const noexcept -> float {
    return static_cast<float>(kPoles) * plant_.theta_mech_rad();
  }

  [[nodiscard]] auto current_ab() const noexcept -> core::Ab<float> {
    return core::clarke(plant_.current_uvw());
  }

  /**
   * @brief [en] The voltage the loop asks for, in the stationary frame.
   * @brief [zh] 电流环请求的电压，静止系。
   */
  [[nodiscard]] auto loop_voltage_ab() const noexcept -> core::Ab<float> {
    return core::inv_park(core::Dq<float>{vd_, vq_}, true_angle_e());
  }

  void apply(core::Ab<float> v_ab) noexcept {
    const core::Dq<float> v_dq = core::park(v_ab, true_angle_e());
    for (int i = 0; i < kSubsteps; ++i) {
      plant_.set_voltage_dq(v_dq);
      plant_.advance(kDt / static_cast<float>(kSubsteps));
    }
  }

  [[nodiscard]] auto omega_e() const noexcept -> float { return omega_e_; }

  /**
   * @brief [en] Change speed without moving the rotor: the flux is continuous across a speed step, so this is a
   *             legitimate (if abrupt) trajectory and not a teleport of the machine state.
   * @brief [zh] 在不动转子的前提下改转速：磁链在转速阶跃处是连续的，因此这是一条合法（虽然突然）的轨迹，而不是把机器状
   *             态瞬移。
   */
  void set_speed(float omega_e) noexcept {
    omega_e_ = omega_e;
    const sil::PlantState now = plant_.state();
    plant_.reset(sil::PlantState{now.id_a, now.iq_a,
                                 omega_e / static_cast<float>(kPoles),
                                 now.theta_mech});
    const float psi_f = kKt / (1.5F * static_cast<float>(kPoles));
    vd_ = -omega_e * kLq * kIq;
    vq_ = (kRs * kIq) + (omega_e * psi_f);
  }

private:
  sil::PmsmPlant plant_;
  float omega_e_;
  float iq_;
  float vd_;
  float vq_;
};

/**
 * @brief [en] Runs one observer (or the blend) for `seconds` and returns the mean absolute angle error over the second
 *             half, so the float transient at the start does not count.
 * @brief [zh] 让一个观测器（或融合）跑 `seconds`，返回后半段的角度误差绝对均值，使开头的暂态不计入。
 * @since 0.1.0
 */
template <typename Observer>
auto mean_angle_error(Observer &observer, SpeedRig &rig, float seconds)
    -> float {
  float sum = 0.0F;
  int count = 0;
  const int steps = static_cast<int>((seconds / kDt) + 0.5F);
  for (int k = 0; k < steps; ++k) {
    const core::Ab<float> v_loop = rig.loop_voltage_ab();
    const core::ObsState observed =
        observer.update(v_loop, rig.current_ab(), kDt);

    core::Ab<float> v_applied = v_loop;
    if constexpr (core::concepts::has_injection_v<Observer>) {
      const core::Ab<float> injected = observer.injection_ab();
      v_applied =
          core::Ab<float>{static_cast<float>(v_loop.alpha + injected.alpha),
                          static_cast<float>(v_loop.beta + injected.beta)};
    }
    rig.apply(v_applied);

    if (k >= (steps / 2)) {
      sum += std::fabs(wrap_pi_local(observed.theta_e - rig.true_angle_e()));
      ++count;
    }
  }
  return (count > 0) ? (sum / static_cast<float>(count)) : 0.0F;
}

} // namespace

TEST(SilObserverBlend, TheFusedEstimateHoldsAcrossTheWholeSpeedRange) {
  // The claim this scenario exists to support. Nothing here is allowed to be
  // quietly average: the two observers are also run alone at the same operating
  // points, so the table below shows where each one fails and that the blend
  // does not inherit either failure.
  // 本场景要支撑的结论。这里不允许用"平均一下"蒙过去：同一批工作点上还把两个观测器各自单独跑一遍，下表
  // 因此能看出各自在哪里失效、以及融合没有继承任何一侧的失效。
  std::printf("\n[S16] %10s %12s %12s %12s\n", "omega_e", "blend", "saliency",
              "back-emf");
  for (const float omega_e : {0.0F, 20.0F, 50.0F, 80.0F, 100.0F, 120.0F, 140.0F,
                              160.0F, 200.0F, 400.0F, 800.0F}) {
    SpeedRig blended_rig(omega_e);
    Blend blend(core::HfiObserver{hfi_config()},
                core::BackEmfObserver{flux_config()}, blend_config());
    blend.seed_flux(core::Ab<float>{blended_rig.psi_f(), 0.0F});
    const float blended = mean_angle_error(blend, blended_rig, 0.4F);

    SpeedRig hfi_rig(omega_e);
    core::HfiObserver hfi(hfi_config());
    const float hfi_only = mean_angle_error(hfi, hfi_rig, 0.4F);

    SpeedRig flux_rig(omega_e);
    core::BackEmfObserver flux(flux_config());
    flux.seed_flux(core::Ab<float>{flux_rig.psi_f(), 0.0F});
    const float flux_only = mean_angle_error(flux, flux_rig, 0.4F);

    std::printf("[S16] %10.1f %12.4f %12.4f %12.4f\n",
                static_cast<double>(omega_e), static_cast<double>(blended),
                static_cast<double>(hfi_only), static_cast<double>(flux_only));
    EXPECT_LT(blended, kBlendTolerance) << "at omega_e " << omega_e;
  }
}

TEST(SilObserverBlend, EachObserverAloneFailsWhereTheBandSaysItDoes) {
  // Both halves of "neither alone is enough", measured rather than asserted
  // from theory. If either of these ever stops failing, the band has become
  // unnecessary and this file should say so.
  // "单用哪个都不够"的两半，都是测量而非从理论断言。若其中任何一个不再失败，带宽就多余了，本文件应当如实
  // 反映这一点。
  SpeedRig fast_rig(kHfiFailsAbove + 40.0F);
  core::HfiObserver hfi(hfi_config());
  EXPECT_GT(mean_angle_error(hfi, fast_rig, 0.4F), 1.0F)
      << "the saliency loop must be useless here";

  SpeedRig slow_rig(kFluxFailsBelow / 2.0F);
  core::BackEmfObserver flux(flux_config());
  flux.seed_flux(core::Ab<float>{slow_rig.psi_f(), 0.0F});
  EXPECT_GT(mean_angle_error(flux, slow_rig, 0.4F), 0.1F)
      << "the flux integrator must be useless here";
}

TEST(SilObserverBlend, TheCarrierIsGoneOnceTheHandoverIsComplete) {
  // The cost side of the blend: injection is current, heat and audible noise,
  // and above the band it buys nothing. Measured as the HF amplitude the
  // saliency loop still reports.
  // 融合的成本侧：注入是电流、发热与可听噪声，在带上侧它什么也买不到。以凸极环仍报告的高频幅值衡量。
  SpeedRig slow_rig(20.0F);
  Blend slow(core::HfiObserver{hfi_config()},
             core::BackEmfObserver{flux_config()}, blend_config());
  slow.seed_flux(core::Ab<float>{slow_rig.psi_f(), 0.0F});
  static_cast<void>(mean_angle_error(slow, slow_rig, 0.4F));
  const float carrier_at_rest = slow.low().hf_current_a();

  SpeedRig fast_rig(400.0F);
  Blend fast(core::HfiObserver{hfi_config()},
             core::BackEmfObserver{flux_config()}, blend_config());
  fast.seed_flux(core::Ab<float>{fast_rig.psi_f(), 0.0F});
  static_cast<void>(mean_angle_error(fast, fast_rig, 0.4F));

  EXPECT_GT(carrier_at_rest, 0.3F)
      << "the carrier must be running where it is needed";
  EXPECT_FLOAT_EQ(1.0F, fast.blend());
  EXPECT_FLOAT_EQ(0.0F, fast.injection_ab().alpha);
  EXPECT_FLOAT_EQ(0.0F, fast.injection_ab().beta);
  std::printf("[S16] carrier %0.3f A at 20 rad/s, %0.3f A at 400 rad/s\n",
              static_cast<double>(carrier_at_rest),
              static_cast<double>(fast.injection_ab().alpha));
}

TEST(SilObserverBlend, ARampThroughTheBandNeverLosesTheRotor) {
  // A real trajectory, because the previous version of this test stepped the
  // speed and measured its own artefact: a speed *step* is not something a
  // joint does, and the observers legitimately ring for a few periods
  // afterwards. Ramping at 500 rad/s^2 electrical, the saliency loop alone was
  // measured at 0.01 rad all the way from 0 to 150, so what is left here is the
  // handover.
  // 真实轨迹。此前那版把转速**阶跃**上去，量到的是自己的伪迹：转速阶跃不是关节会做的事，观测器在阶跃后
  // 合法地会振几个周期。以 500 rad/s² 电角加速度上升时，单独测凸极环从 0 到 150
  // 全程 0.01 rad，于是 这里剩下的问题就是交接本身。
  SpeedRig rig(0.0F);
  Blend blend(core::HfiObserver{hfi_config()},
              core::BackEmfObserver{flux_config()}, blend_config());
  blend.seed_flux(core::Ab<float>{rig.psi_f(), 0.0F});

  float worst = 0.0F;
  float worst_in_band = 0.0F;
  float weight_rising = 0.0F;
  float weight_falling = 0.0F;

  auto ramp = [&](float target, bool sample_weight) -> float {
    float local_worst = 0.0F;
    const int steps = static_cast<int>(std::fabs(target - rig.omega_e()) /
                                       (kRampRate * kDt)) +
                      1;
    for (int i = 0; i <= steps; ++i) {
      const float span = std::fabs(target - rig.omega_e());
      const float move = std::fmin(span, kRampRate * kDt);
      const float next =
          rig.omega_e() + ((target > rig.omega_e()) ? move : -move);
      rig.set_speed(next);

      const core::Ab<float> v_loop = rig.loop_voltage_ab();
      const core::ObsState observed =
          blend.update(v_loop, rig.current_ab(), kDt);
      const core::Ab<float> injected = blend.injection_ab();
      rig.apply(
          core::Ab<float>{static_cast<float>(v_loop.alpha + injected.alpha),
                          static_cast<float>(v_loop.beta + injected.beta)});
      const float err_now =
          std::fabs(wrap_pi_local(observed.theta_e - rig.true_angle_e()));
      local_worst = std::fmax(local_worst, err_now);
      // Tracked separately: what the handover adds, as opposed to what either
      // observer already brings to it. Outside the band the output *is* one of
      // them, and their own transients are not this class's doing -- asserting
      // them here would hide the one number that matters.
      // 分开统计：交接本身带来了多少，而任一观测器本来就有多少。带外输出就是其中某一个，它们自身的
      // 暂态与本类无关；把它们一起断言掉，会掩盖唯一重要的那个数字。
      if ((blend.blend() > 0.05F) && (blend.blend() < 0.95F)) {
        worst_in_band = std::fmax(worst_in_band, err_now);
      }
    }
    if (sample_weight) {
      // Measured at the same operating point on the way up and on the way down.
      // 在上行与下行时分别于同一工作点测量。
      weight_rising = blend.blend();
    }
    return local_worst;
  };

  // Up through the band to 400 and back down to a standstill.
  // 穿过带到 400，再回到静止。
  worst = std::fmax(worst, ramp(90.0F, false));
  ramp(90.0F, true);
  worst = std::fmax(worst, ramp(400.0F, false));
  worst = std::fmax(worst, ramp(90.0F, false));
  ramp(90.0F, true);
  weight_falling = blend.blend();
  worst = std::fmax(worst, ramp(0.0F, false));

  std::printf("[S16] ramp: worst %0.4f rad (in band %0.4f), weight at 90 "
              "rad/s: %0.3f rising, %0.3f falling\n",
              static_cast<double>(worst), static_cast<double>(worst_in_band),
              static_cast<double>(weight_rising),
              static_cast<double>(weight_falling));

  // What the numbers are allowed to say, and no more. The peak excursion is the
  // *flux observer's own* transient during a 500 rad/s^2 ramp with a 10 %
  // nameplate error -- it appears at weight 1.0, i.e. before the blend touches
  // it, and it is the pre-existing limitation the shipped header already names
  // ("integrator drift still needs a PLL/SMO observer"). A PLL-type back-EMF
  // observer is what removes it; the handover is not the cause and cannot
  // compensate for it.
  // 这些数字允许说的，不多不少。峰值偏移来自**反电动势观测器自己**在 500 rad/s²
  // 斜坡、10% 铭牌误差下的 暂态——它出现在权重 1.0
  // 处，即融合尚未介入，而且正是出厂头文件早已写明的既有局限（"积分漂移仍需
  // PLL/SMO 观测器"）。去掉它需要 PLL
  // 型反电动势观测器；交接不是原因，也无法补偿它。
  EXPECT_LT(worst, 0.3F)
      << "a ramp transient above this would mean the handover is adding to it";
  std::printf("[S16] in-band worst %0.4f rad\n",
              static_cast<double>(worst_in_band));

  // The weight itself is allowed to differ between the two directions: it is
  // computed from the flux observer's own speed, and an open-loop integrator's
  // state depends on how it got there. What is asserted is that neither
  // direction leaves the band.
  // 权重本身允许两个方向不同：它取自反电动势观测器自己的转速，而开环积分器的状态取决于它是怎么来的。
  // 断言的是两个方向都没有走出带宽。
  // Reproducible across the two directions, which is what "no latch" means. The
  // absolute value is higher than the band sits at steady state because the
  // criterion reads a speed that over-reads during a ramp; that is recorded
  // rather than tuned away.
  // 两个方向可复现，这就是"没有锁存"的意思。绝对值比稳态时高，因为判据读的转速在斜坡中偏高——如实记下，
  // 而不是调掉。
  EXPECT_NEAR(weight_rising, weight_falling, 0.1F);

  // And it ends where it started: the carrier is back and the saliency loop is
  // locked again. 最终回到起点：载波回来了，凸极环重新锁定。
  static_cast<void>(mean_angle_error(blend, rig, 0.2F));
  EXPECT_TRUE(blend.locked());
  EXPECT_FLOAT_EQ(0.0F, blend.blend());
}

TEST(SilObserverBlend, TheCarrierBiasesTheFluxObserverAndTheBiasScalesWithIt) {
  // The number the previous commit could not explain, explained and pinned. At
  // 400 rad/s in the band configuration the blend measured 0.042 rad where the
  // flux observer alone measures 0.011 -- and forcing the weight to 1 gave
  // 0.011, so the fusion was not the cause. It is the carrier: during the
  // handover the injection is still being delivered, an open-loop flux
  // integrator cannot tell it from back-EMF, and whatever it leaves behind
  // stays there because the integrator has no reset.
  //
  // The expectation is one carrier period of volt-seconds: V_h /
  // (2*pi*f_h*psi_f). For 2 V at 2 kHz on this machine that is 0.033 rad, and
  // 0.031 was measured. It is linear in the injection amplitude and inversely
  // proportional to its frequency -- which is the sizing rule, and the reason a
  // smaller carrier comes out *better* than linear here: it also drops out of
  // lock sooner, so less of it is ever delivered.
  // 上一个提交里没能解释的数字，现在解释清楚并钉住。400 rad/s 带内融合实测
  // 0.042，而单独反电动势是 0.011，且强制权重为 1 也是
  // 0.011——所以原因不在融合，而在载波：交接期间注入仍在送达，开环磁链积分器
  // 分不清它与反电动势，留下什么就会一直留着，因为积分器没有复位。
  // 预期量级是一个载波周期的伏秒：V_h/(2π·f_h·ψf)。本机 2 V / 2 kHz 对应 0.033
  // rad，实测 0.031。
  // 它与注入幅值成正比、与频率成反比——这就是定容规则；也解释了此处更小的载波之所以"好于线性"：
  // 它也更快掉出锁定，实际送达的量更少。
  constexpr float kPsi = kKt / (1.5F * static_cast<float>(kPoles));
  constexpr float kOmega = 400.0F;

  auto band_error = [](float inject_v) {
    SpeedRig rig(kOmega);
    Blend blend(core::HfiObserver{hfi_config(inject_v)},
                core::BackEmfObserver{flux_config()}, blend_config());
    blend.seed_flux(core::Ab<float>{rig.psi_f(), 0.0F});
    return mean_angle_error(blend, rig, 0.4F);
  };
  // Weight 1 within a couple of steps, so almost no carrier is ever delivered:
  // the baseline. 几步之内权重就到 1，几乎没有任何载波被送达：基线。
  auto baseline_error = [](float inject_v) {
    SpeedRig rig(kOmega);
    Blend blend(core::HfiObserver{hfi_config(inject_v)},
                core::BackEmfObserver{flux_config()},
                Blend::Config{0.0F, 1.0F, 20.0F});
    blend.seed_flux(core::Ab<float>{rig.psi_f(), 0.0F});
    return mean_angle_error(blend, rig, 0.4F);
  };

  const float excess_2v = band_error(2.0F) - baseline_error(2.0F);
  const float excess_half = band_error(0.5F) - baseline_error(0.5F);
  const float predicted = (2.0F / (2.0F * core::kPi<float> * 2000.0F)) / kPsi;

  std::printf("[S16] carrier bias: excess %.4f rad at 2 V (one carrier period "
              "predicts %.4f), %.4f at 0.5 V\n",
              static_cast<double>(excess_2v), static_cast<double>(predicted),
              static_cast<double>(excess_half));

  // Within a factor of two of the volt-second estimate -- it is an
  // order-of-magnitude model, not a calibrated formula, and the cancellation
  // that leaves the residue depends on the parameter error.
  // 在伏秒估计的两倍以内——它是量级模型而非标定公式，留下残余的抵消程度取决于参数误差。
  EXPECT_GT(excess_2v, 0.5F * predicted);
  EXPECT_LT(excess_2v, 2.0F * predicted);
  // And it collapses when the carrier is smaller, which is what makes it the
  // carrier's doing. 载波变小时它迅速消失，这正是"这是载波干的"的证据。
  EXPECT_LT(excess_half, 0.5F * excess_2v);
}

/**
 * @brief [en] The same band, with the magnitude-correcting observer as its high half.
 * @brief [zh] 同一个带，只是高端换成带幅值修正的观测器。
 * @since 0.1.0
 */
using CorrectedBlend =
    core::BlendedObserver<core::HfiObserver, core::NonlinearFluxObserver>;

auto corrected_band() -> CorrectedBlend::Config {
  // The same 60..120 rad/s as `blend_config()`, and for the same two reasons:
  // the saliency loop mirrors above 160, and the high half's speed is a
  // differentiated angle either way. `Config` is nested in the instantiation,
  // so the numbers have to be written out again -- which is the point, since a
  // band is a property of the pair, not of one observer. 与 `blend_config()`
  // 相同的 60–120 rad/s，理由也一样：凸极环在 160 以上会锁镜像，而高端的速度
  // 无论哪个观测器都是对角度的微分。`Config`
  // 嵌在实例化里，所以这些数字得再写一遍——这正是重点，
  // 带宽是这对观测器的性质，不是某一个观测器的。
  return CorrectedBlend::Config{60.0F, 120.0F, 20.0F};
}

auto corrected_flux_config() -> core::NonlinearFluxObserver::Config {
  // Exactly the machine the flux observer is told about above, plus the two
  // gains. The magnitude target is the nameplate flux: this observer is the one
  // that knows what the answer should be.
  // 与上面告诉反电动势观测器的机器完全一致，再加两个增益。幅值目标是铭牌磁链：这个观测器知道答案
  // 应该是多少。
  constexpr float kPsi = kKt / (1.5F * static_cast<float>(kPoles));
  return core::NonlinearFluxObserver::Config{
      kRsSeenByFluxObserver, kLq, kPsi, 0.3F, 3.0F, 0.5F * kPsi};
}

TEST(SilObserverBlend,
     TheMagnitudeCorrectionRemovesTheCarrierBiasAndLowerstheWorstCase) {
  // What the correction buys in the configuration that actually ships. The
  // comparison is against the numbers this file already recorded for the
  // open-loop high half -- blend 0.042 rad worst case and 0.031 rad of carrier
  // bias at 400 rad/s -- not against a fresh run of it, so the table cannot
  // drift in step with the thing it is judging.
  // 修正它在真正要交付的配置里买到了什么。比较对象是本文件已经记录过的开环高端数字——最差
  // 0.042 rad、 400 rad/s 处 0.031 rad
  // 载波偏置——而不是当场再跑一遍，这样表格不会跟着被评判的对象一起漂。
  constexpr float kRecordedWorstCase = 0.042F;
  constexpr float kRecordedCarrierBias = 0.031F;

  std::printf("\n[S16] %10s %12s\n", "omega_e", "corrected");
  float worst = 0.0F;
  for (const float omega_e : {0.0F, 20.0F, 50.0F, 80.0F, 100.0F, 120.0F, 140.0F,
                              160.0F, 200.0F, 400.0F, 800.0F}) {
    SpeedRig rig(omega_e);
    CorrectedBlend blend(core::HfiObserver{hfi_config()},
                         core::NonlinearFluxObserver{corrected_flux_config()},
                         corrected_band());
    blend.seed_flux(core::Ab<float>{rig.psi_f(), 0.0F});
    const float error = mean_angle_error(blend, rig, 0.4F);
    std::printf("[S16] %10.1f %12.4f\n", static_cast<double>(omega_e),
                static_cast<double>(error));
    worst = std::fmax(worst, error);
    EXPECT_LT(error, kBlendTolerance) << "at omega_e " << omega_e;
  }

  // The whole reason the correction was worth writing: the carrier stops being
  // integrated in. 写这个修正的全部理由：载波不再被积进去。
  constexpr float kOmega = 400.0F;
  auto corrected_excess = [](float inject_v) {
    SpeedRig band_rig(kOmega);
    CorrectedBlend band(core::HfiObserver{hfi_config(inject_v)},
                        core::NonlinearFluxObserver{corrected_flux_config()},
                        corrected_band());
    band.seed_flux(core::Ab<float>{band_rig.psi_f(), 0.0F});
    const float with_carrier = mean_angle_error(band, band_rig, 0.4F);

    SpeedRig base_rig(kOmega);
    CorrectedBlend base(core::HfiObserver{hfi_config(inject_v)},
                        core::NonlinearFluxObserver{corrected_flux_config()},
                        CorrectedBlend::Config{0.0F, 1.0F, 20.0F});
    base.seed_flux(core::Ab<float>{base_rig.psi_f(), 0.0F});
    return with_carrier - mean_angle_error(base, base_rig, 0.4F);
  };

  const float excess_2v = corrected_excess(2.0F);
  std::printf("[S16] corrected: worst %.4f rad (open loop recorded %.4f), "
              "carrier bias %.4f rad "
              "(open loop recorded %.4f)\n",
              static_cast<double>(worst),
              static_cast<double>(kRecordedWorstCase),
              static_cast<double>(excess_2v),
              static_cast<double>(kRecordedCarrierBias));

  EXPECT_LT(worst, kRecordedWorstCase)
      << "the correction has to lower the worst case, not match it";
  // An order of magnitude down, not merely smaller: a residual that is a few
  // percent of the old bias would mean the correction is fighting the carrier
  // rather than replacing it.
  // 要低一个量级，而不只是"更小"：如果残余还有旧偏置的百分之几，说明修正在和载波较劲而不是取代它。
  EXPECT_LT(std::fabs(excess_2v), 0.1F * kRecordedCarrierBias);
}

TEST(SilObserverBlend, TheCorrectionHalvesTheRampTransient) {
  // The largest single win of the correction, and the least obvious: the ramp
  // case above records 0.2445 rad worst (0.1927 in band) and attributes it to
  // the open-loop half's own transient -- "an integrator's drift still needs a
  // PLL/SMO observer". The correction is not a PLL, but it does give the flux
  // back a magnitude, and most of that transient was the magnitude running away
  // rather than the phase being read wrong. So the attribution above was only
  // half right, and this is the other half.
  // 这个修正最大、也最不显然的一处收益：上面的斜坡用例记录 0.2445 rad
  // 最差（带内 0.1927），并把原因 归给开环那一侧自己的暂态——"积分漂移仍需
  // PLL/SMO 观测器"。修正不是 PLL，但它把幅值还回去了，
  // 而那个暂态的大部分是幅值在跑掉，不是相位读错。所以上面那句归因只对了一半，这里是另一半。
  constexpr float kRecordedRampWorst = 0.2445F;
  constexpr float kRecordedInBandWorst = 0.1927F;

  SpeedRig rig(0.0F);
  CorrectedBlend blend(core::HfiObserver{hfi_config()},
                       core::NonlinearFluxObserver{corrected_flux_config()},
                       corrected_band());
  blend.seed_flux(core::Ab<float>{rig.psi_f(), 0.0F});

  float worst_in_band = 0.0F;
  auto ramp = [&](float target) -> float {
    float local_worst = 0.0F;
    const int steps = static_cast<int>(std::fabs(target - rig.omega_e()) /
                                       (kRampRate * kDt)) +
                      1;
    for (int i = 0; i <= steps; ++i) {
      const float span = std::fabs(target - rig.omega_e());
      const float move = std::fmin(span, kRampRate * kDt);
      const float next =
          rig.omega_e() + ((target > rig.omega_e()) ? move : -move);
      rig.set_speed(next);

      const core::Ab<float> v_loop = rig.loop_voltage_ab();
      const core::ObsState observed =
          blend.update(v_loop, rig.current_ab(), kDt);
      const core::Ab<float> injected = blend.injection_ab();
      rig.apply(
          core::Ab<float>{static_cast<float>(v_loop.alpha + injected.alpha),
                          static_cast<float>(v_loop.beta + injected.beta)});

      const float error =
          std::fabs(wrap_pi_local(observed.theta_e - rig.true_angle_e()));
      local_worst = std::fmax(local_worst, error);
      if ((next >= 60.0F) && (next <= 120.0F)) {
        worst_in_band = std::fmax(worst_in_band, error);
      }
    }
    return local_worst;
  };

  const float worst = std::fmax(ramp(200.0F), ramp(0.0F));
  std::printf("[S16] corrected ramp: worst %0.4f rad (recorded %0.4f), in band "
              "%0.4f (recorded %0.4f)\n",
              static_cast<double>(worst),
              static_cast<double>(kRecordedRampWorst),
              static_cast<double>(worst_in_band),
              static_cast<double>(kRecordedInBandWorst));

  // Halving it is the claim; the absolute bound is what stops a future
  // regression from hiding inside a comparison with a number that itself
  // drifted.
  // 断言是"减半"；绝对上界是为了防止将来回退藏在一个本身已经漂了的对照数字里。
  EXPECT_LT(worst, 0.5F * kRecordedRampWorst);
  EXPECT_LT(worst_in_band, 0.5F * kRecordedInBandWorst);
  EXPECT_LT(worst, 0.15F);
}
