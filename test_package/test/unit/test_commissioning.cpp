// Conan::ImportStart
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>
#include <utility>

#include <foc/core/commissioning.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>

#include "sil_bench.hpp"
#include "sil_discrete_inverter.hpp"
#include "sil_plant_hal.hpp"
#include "sil_switching_rig.hpp"
// Conan::ImportEnd

namespace foc {

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

constexpr float kPwmHz = 20000.0F;
constexpr float kPeriod = 1.0F / kPwmHz;
constexpr float kSwitchingTDead = 1.0e-6F;
constexpr float kVBus = 24.0F;
constexpr int kSubSteps = 40;

/// [en] The dead-time voltage at 24 V with a 1 us dead time: 0.48 V. [zh]
/// 24 V、1 µs 死区对应的死区电压：0.48 V。
constexpr float kVDead = kSwitchingTDead * kPwmHz * kVBus;
/// [en] What the same loss looks like **on the d axis**. Projecting the three
/// per-phase steps
/// `(+dV, -dV, -dV)` onto `d` gives `(2/3)*(dV + dV/2 + dV/2) = (4/3)*dV`: an
/// axis sees **more** than a phase does, because the two opposing phases push
/// the same way once projected. Every number measured on an axis is in this
/// convention; the per-phase number the compensator uses is not, and the
/// difference is exactly this factor.
///      [zh] 同一个损失在 **d 轴上**是什么样。把三相的阶跃 `(+dV, -dV, -dV)`
///      投影到 `d` 得
///      `(2/3)·(dV + dV/2 + dV/2) =
///      (4/3)·dV`：轴上的数**比**每相**大**，因为两相反向的阶跃投影
///      之后同向相加。所有在轴上量到的数都是这个约定；补偿器用的每相数不是，两者差的正是这个系数。
constexpr float kVAxisDead = (4.0F / 3.0F) * kVDead;

/// [en] A standstill bench: the plant plus the switching inverter, driven with
/// a fixed d-q command. `period()` runs exactly one switching period and
/// returns that period's **mean** current, which is the only quantity a
/// commissioning sweep should ever look at.
///      [zh] 堵转台架：plant 加开关逆变器，用固定的 d-q 指令驱动。`period()`
///      正好跑一个开关周期并返回该周期的**均值**电流——整定扫描唯一该看的量。
class StandstillRig final {
public:
  explicit StandstillRig(const sil::PlantParams &params) noexcept
      : plant_(params), inverter_(sil::InverterConfig{kPwmHz, kSwitchingTDead,
                                                      0.0F, kSubSteps}),
        poles_(static_cast<float>(params.pole_pairs)) {
    plant_.reset(sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
  }

  auto plant() noexcept -> sil::PmsmPlant & { return plant_; }

  /// [en] The last d-q voltage the inverter actually delivered. [zh]
  /// 逆变器最后一次真正交付的 d-q 电压。
  auto applied() const noexcept -> core::Dq<float> { return applied_; }

  auto period(float v_d, float v_q) noexcept -> core::Dq<float> {
    const core::Ab<float> command = core::inv_park(
        core::Dq<float>{v_d, v_q}, poles_ * plant_.theta_mech_rad());
    const core::SvpwmResult modulated = core::Svpwm::modulate(
        core::ModulatorInput{command, kVBus, plant_.current_uvw()});
    const int steps = inverter_.begin_period(modulated.duty, kVBus, kPeriod);
    const float h = inverter_.step_length_s();
    float sum_d = 0.0F;
    float sum_q = 0.0F;
    for (int k = 0; k < steps; ++k) {
      const core::Ab<float> applied = inverter_.step(plant_.current_uvw());
      applied_ = core::park(applied, poles_ * plant_.theta_mech_rad());
      plant_.set_voltage_dq(applied_);
      plant_.advance(h);
      const core::Dq<float> i_dq = plant_.current_dq();
      sum_d += i_dq.d;
      sum_q += i_dq.q;
    }
    const float inverse = 1.0F / static_cast<float>(steps);
    return core::Dq<float>{sum_d * inverse, sum_q * inverse};
  }

  /// [en] Hold a command for `periods` periods; return the last period's mean.
  ///      [zh] 保持一个指令 `periods` 个周期；返回最后一个周期的均值。
  auto hold(float v_d, float v_q, int periods) noexcept -> core::Dq<float> {
    core::Dq<float> mean{0.0F, 0.0F};
    for (int k = 0; k < periods; ++k) {
      mean = period(v_d, v_q);
    }
    return mean;
  }

private:
  sil::PmsmPlant plant_;
  sil::DiscreteInverter inverter_;
  float poles_;
  core::Dq<float> applied_{0.0F, 0.0F};
};

/// [en] The nominal machine as the bench sees it. [zh] 台架眼中的标称机。
auto nominal_params() -> sil::PlantParams {
  return sil::plant_params_from(sil::nominal_spec());
}

/// [en] A salient machine: `Lq = 2*Ld`. The nominal one has `Ld == Lq`, which
/// makes a d/q mix-up invisible -- exactly the kind of bug this test is for.
///      [zh] 一台凸极机：`Lq = 2·Ld`。标称机 `Ld == Lq`，会把 d/q 弄反这种错误
///      藏得干干净净——而这正是本测试要找的东西。
auto salient_params() -> sil::PlantParams {
  sil::PlantParams params = nominal_params();
  params.ld = 4.0e-4F;
  params.lq = 8.0e-4F;
  return params;
}

} // namespace

// ---------------------------------------------------------------------------
// P5-a: stator resistance and the dead-time voltage out of one DC sweep.
// P5-a：一次直流扫描同时得到定子电阻与死区电压。
// ---------------------------------------------------------------------------

TEST(Commissioning, P5a_ResistanceAndDeadTimeComeOutOfOneDcSweep) {
  // The sweep a commissioning routine would actually run: a list of voltages,
  // each held until the current settles, and the settled pair (v, i) fed to the
  // fit. No bisection, no targeting of currents -- targeting would need the
  // model being identified.
  // 整定程序真正会跑的扫描：一串电压，每个都保持到电流稳定，把稳定后的 (v, i)
  // 送进拟合。 不做二分、不瞄准某个电流——瞄准要用到正在被辨识的那个模型。
  StandstillRig rig(nominal_params());
  core::RsEstimator estimator(core::RsEstimator::Config{0.5F, 3.0F});
  const float commands[] = {0.900F, 1.050F, 1.200F, 1.400F, 1.650F, 1.950F};
  std::printf("\n[P5-a] DC sweep, V -> settled I\n");
  for (const float v : commands) {
    const core::Dq<float> mean = rig.hold(v, 0.0F, 120);
    const bool used = estimator.add(v, mean.d);
    std::printf("        %.3f V -> %.5f A (%s)\n", static_cast<double>(v),
                static_cast<double>(mean.d), used ? "used" : "rejected");
    EXPECT_TRUE(used) << "every point of this sweep is above the fade band";
  }
  const core::RsEstimator::Result fit = estimator.solve();
  const float truth = nominal_params().rs;
  std::printf("        rs %.5f ohm (truth %.5f, %+.2f %%), deadtime_v %.5f V "
              "(truth %.5f), residual %.5f V\n",
              static_cast<double>(fit.rs), static_cast<double>(truth),
              static_cast<double>(100.0F * (fit.rs - truth) / truth),
              static_cast<double>(fit.deadtime_v),
              static_cast<double>(kVAxisDead),
              static_cast<double>(fit.residual_v));

  EXPECT_TRUE(fit.valid);
  // P5-a of the plan: within 5 % of the winding's own value.
  // plan 的 P5-a：与绕组自身的值差 5 % 以内。
  EXPECT_NEAR(truth, fit.rs, 0.05F * truth);
  // And the intercept is the inverter's dead-time voltage, within 20 %: it is a
  // *by-product* of separating the two terms, not something the sweep was
  // designed for, so the looser bound is the honest one. It still has to be the
  // right order of magnitude, or the separation did not happen (a fit that put
  // everything into `rs` would leave this near zero).
  // 截距是逆变器的死区电压，20 %
  // 以内：它是分离两项的**副产品**，不是扫描专门设计的对象，
  // 所以宽一点的界更诚实。但它仍必须是对的数量级，否则分离根本没发生（把一切都塞进
  // `rs` 的拟合会把它留在零附近）。
  EXPECT_NEAR(kVAxisDead, fit.deadtime_v, 0.2F * kVAxisDead);
}

TEST(Commissioning, P5a_TheWindingTemperatureIsMeasuredRatherThanAssumed) {
  // Copper moves about 0.4 %/K, so a winding at +100 K reads 40 % high. The
  // nameplate in `nominal_params()` is the cold number; the point of measuring
  // is that the *hot* machine is what the observers will actually run against.
  // 铜的温度系数约 0.4 %/K，+100 K 的绕组读数高 40 %。`nominal_params()`
  // 里的铭牌是冷态数； "测"的意义就在于观测器实际面对的是一台**热**机。
  sil::PlantParams hot = nominal_params();
  hot.rs = 1.4F * nominal_params().rs; // +100 K

  StandstillRig rig(hot);
  core::RsEstimator estimator(core::RsEstimator::Config{0.5F, 3.0F});
  for (const float v : {1.100F, 1.300F, 1.500F, 1.750F, 2.050F, 2.400F}) {
    const core::Dq<float> mean = rig.hold(v, 0.0F, 120);
    static_cast<void>(estimator.add(v, mean.d));
  }
  const core::RsEstimator::Result fit = estimator.solve();
  std::printf("[P5-a] hot winding (rs %.3f ohm): fit %.5f ohm, cold nameplate "
              "%.3f ohm\n",
              static_cast<double>(hot.rs), static_cast<double>(fit.rs),
              static_cast<double>(nominal_params().rs));

  EXPECT_TRUE(fit.valid);
  EXPECT_NEAR(hot.rs, fit.rs, 0.05F * hot.rs);
  // And the nameplate *would* have been wrong -- otherwise this test proves
  // nothing. 而铭牌**确实**是错的——否则这个测试什么也证明不了。
  EXPECT_GT(std::fabs(nominal_params().rs - hot.rs), 0.2F * hot.rs);
}

TEST(Commissioning,
     P5a_SamplesInsideTheFadeBandAreRefusedRatherThanAveragedIn) {
  // Below a few tens of milliamps the loss is not a step yet (SIL S13), so the
  // model being fitted does not hold there. The estimator must say so instead
  // of tilting `rs` downwards: a sample at 5 mA would put almost the whole 0.48
  // V into the intercept and drag the slope down.
  // 几十毫安以下损失还不是阶跃（SIL
  // S13），拟合的模型在那里不成立。估计器必须说出来，而不是 把 `rs` 往下带：5
  // mA 的样本会把几乎整个 0.48 V 塞进截距，把斜率拽下来。
  StandstillRig rig(nominal_params());
  core::RsEstimator estimator(core::RsEstimator::Config{0.5F, 3.0F});
  int used = 0;
  for (const float v : {0.500F, 0.510F, 0.520F, 0.530F, 0.540F, 0.550F}) {
    const core::Dq<float> mean = rig.hold(v, 0.0F, 120);
    used += estimator.add(v, mean.d) ? 1 : 0;
  }
  const core::RsEstimator::Result fit = estimator.solve();
  std::printf("[P5-a] 0.50-0.55 V sweep (all inside the fade band): %d used, "
              "valid %d\n",
              used, fit.valid ? 1 : 0);

  EXPECT_EQ(0, used) << "every one of these points is below i_min_a";
  EXPECT_FALSE(fit.valid);
  EXPECT_EQ(0.0F, fit.rs)
      << "no number is invented when the model does not hold";
}

TEST(Commissioning, P5a_OneCurrentCannotSeparateTheResistanceFromTheDeadTime) {
  // Six samples, all settled at the same current. Two unknowns cannot come out
  // of one point, and the estimator must refuse rather than return the slope of
  // noise.
  // 六个样本，全稳定在同一个电流上。两个未知数不能从一个点里出来，估计器必须拒绝，而不是返回
  // 噪声的斜率。
  StandstillRig rig(nominal_params());
  core::RsEstimator estimator(core::RsEstimator::Config{0.5F, 3.0F});
  for (int k = 0; k < 6; ++k) {
    const core::Dq<float> mean = rig.hold(1.200F, 0.0F, 120);
    static_cast<void>(estimator.add(1.200F, mean.d));
  }
  const core::RsEstimator::Result fit = estimator.solve();
  std::printf("[P5-a] six samples at one current: valid %d\n",
              fit.valid ? 1 : 0);
  EXPECT_EQ(6, estimator.samples());
  EXPECT_FALSE(fit.valid)
      << "no slope in i means no separation of rs from deadtime_v";
}

// ---------------------------------------------------------------------------
// P5-b: rotor flux linkage from a zero-current spin.
// P5-b：零电流旋转下的转子磁链。
// ---------------------------------------------------------------------------

namespace {

/// [en] Spin the plant at a frozen electrical speed with a pure integrator
/// closing the loop on `i_q = 0`, and return the settled voltage magnitude.
/// Freezing the speed (`j = 1e6`) is the same trick the locked-rotor tests use
/// in the other direction: the mechanical equation is taken out of the
/// experiment so that the *electrical* one is all that is left.
///      [zh] 把 plant 冻在一个电角速度上旋转，用一个纯积分器把 `i_q = 0`
///      闭上，返回稳定后的电压 幅值。冻结转速（`j =
///      1e6`）和堵转测试用的是同一个手法的反方向：把机械方程从实验里拿掉，
///      只剩电气方程。
auto zero_current_voltage(float omega_e, const sil::PlantParams &params)
    -> float {
  sil::PlantParams frozen = params;
  frozen.j = 1.0e6F;
  frozen.b = 0.0F;
  StandstillRig rig(frozen);
  rig.plant().reset(sil::PlantState{
      0.0F, 0.0F, omega_e / static_cast<float>(params.pole_pairs), 0.0F});

  float v_q = omega_e * 4.0e-3F;   // a nameplate-grade starting guess
  const float ki = 400.0F;         // V/(A*s)
  for (int k = 0; k < 1200; ++k) { // 60 ms
    const core::Dq<float> mean = rig.period(0.0F, v_q);
    v_q += ki * (0.0F - mean.q) * kPeriod;
  }
  return std::fabs(v_q);
}

} // namespace

TEST(Commissioning, P5b_FluxComesOutOfTwoZeroCurrentSpeeds) {
  const sil::PlantParams params = nominal_params();
  const float truth = sil::SwitchingRig::psi_f();
  core::FluxEstimator estimator(core::FluxEstimator::Config{0.02F, 100.0F});

  std::printf("\n[P5-b] zero-current spin, omega_e -> settled |v|\n");
  for (const float omega_e : {300.0F, 400.0F, 600.0F, 800.0F}) {
    const float v = zero_current_voltage(omega_e, params);
    const bool used = estimator.add(omega_e, v, 0.0F);
    std::printf("        %6.0f rad/s -> %.5f V (%s)\n",
                static_cast<double>(omega_e), static_cast<double>(v),
                used ? "used" : "rejected");
    EXPECT_TRUE(used);
  }
  const core::FluxEstimator::Result fit = estimator.solve();
  std::printf("        psi_f %.6f Wb (truth %.6f, %+.2f %%), offset %.5f V, "
              "residual %.5f V\n",
              static_cast<double>(fit.psi_f), static_cast<double>(truth),
              static_cast<double>(100.0F * (fit.psi_f - truth) / truth),
              static_cast<double>(fit.offset_v),
              static_cast<double>(fit.residual_v));

  EXPECT_TRUE(fit.valid);
  EXPECT_NEAR(truth, fit.psi_f, 0.05F * truth);
  // A zero-current spin needs no `rs*i` and no dead-time term, so the intercept
  // should be small. 零电流旋转不需要 `rs·i`
  // 项也不需要死区项，所以截距应该很小。
  EXPECT_LT(std::fabs(fit.offset_v), 0.05F);
}

TEST(Commissioning, P5b_AConstantOffsetShowsUpInTheInterceptAndNotInTheFlux) {
  // Two speeds are enough *because* the offset is a free parameter: a constant
  // volt added to every measurement moves the intercept and leaves the slope
  // alone. This is the property that makes the method usable on real hardware,
  // where some offset always survives.
  // 之所以两个转速就够，**正因为**偏置是自由参数：给每个测量加一个常数电压只移动截距、不动斜率。
  // 正是这条性质让方法在真实硬件上可用——那里总归会剩一点偏置。
  core::FluxEstimator estimator(core::FluxEstimator::Config{0.02F, 100.0F});
  const float truth = sil::SwitchingRig::psi_f();
  const float bias = 0.03F;
  for (const float omega_e : {300.0F, 400.0F, 600.0F, 800.0F}) {
    static_cast<void>(estimator.add(omega_e, (omega_e * truth) + bias, 0.0F));
  }
  const core::FluxEstimator::Result fit = estimator.solve();
  std::printf("[P5-b] with a %.3f V bias: psi_f %.6f Wb, offset %.5f V\n",
              static_cast<double>(bias), static_cast<double>(fit.psi_f),
              static_cast<double>(fit.offset_v));

  EXPECT_NEAR(truth, fit.psi_f, 0.02F * truth);
  EXPECT_NEAR(bias, fit.offset_v, 0.005F);
}

TEST(Commissioning, P5b_OneSpeedAndCurrentCarryingSamplesAreBothRefused) {
  core::FluxEstimator estimator(core::FluxEstimator::Config{0.02F, 100.0F});
  // One speed is not a slope.
  // 一个转速不是斜率。
  for (int k = 0; k < 4; ++k) {
    static_cast<void>(estimator.add(400.0F, 1.9F, 0.0F));
  }
  EXPECT_FALSE(estimator.solve().valid);

  core::FluxEstimator guarded(core::FluxEstimator::Config{0.02F, 100.0F});
  int used = 0;
  for (const float omega_e : {300.0F, 400.0F, 600.0F, 800.0F}) {
    used += guarded.add(omega_e, omega_e * 4.7e-3F, 0.15F) ? 1 : 0;
  }
  std::printf("[P5-b] samples taken with 150 mA flowing: %d used\n", used);
  EXPECT_EQ(0, used) << "a spinning current is not a zero-current spin";
  EXPECT_FALSE(guarded.solve().valid);
}

// ---------------------------------------------------------------------------
// ---------------------------------------------------------------------------
// P5-d: current-sensor offset and gain.
// P5-d：电流传感器的零漂与增益。
// ---------------------------------------------------------------------------

TEST(Commissioning,
     P5d_TheOffsetNeedsNoReferenceAndTheGainCannotExistWithoutOne) {
  // The asymmetry is the finding, not a limitation of the implementation.
  // 不对称本身就是结论，不是实现的局限。
  core::CurrentSenseCalibrator offset_only(
      core::CurrentSenseCalibrator::Config{0.5F, 0.01F});
  // A channel with a +50 mA offset, read twenty times with the bridge really
  // off. 一条带 +50 mA 零漂的通道，在桥真的关断时读二十次。
  for (int k = 0; k < 20; ++k) {
    const float reading =
        0.05F + (((k % 3) - 1) * 2.0e-4F); // +-0.2 mA of noise
    EXPECT_TRUE(offset_only.add_zero(reading));
  }
  const core::CurrentSenseCalibrator::Result offset = offset_only.solve();
  std::printf("\n[P5-d] a +50 mA channel read at zero current: offset %.5f A, "
              "spread %.5f A, offset_valid %d, gain_valid %d\n",
              static_cast<double>(offset.zero_offset_a),
              static_cast<double>(offset.zero_spread_a),
              offset.offset_valid ? 1 : 0, offset.gain_valid ? 1 : 0);

  EXPECT_TRUE(offset.offset_valid);
  EXPECT_NEAR(0.05F, offset.zero_offset_a, 0.001F);
  // And no gain: twenty readings at one (zero) current determine no slope. The
  // estimator refuses instead of returning 1.0 and calling it calibrated.
  // 而增益没有：二十个读数都在同一个（零）电流上，定不出斜率。估计器拒绝，而不是返回
  // 1.0 然后宣称已校正。
  EXPECT_FALSE(offset.gain_valid)
      << "a gain is underdetermined at any single current";
}

TEST(Commissioning, P5d_GainAndOffsetComeOutTogetherAgainstAReference) {
  // The pair a real calibration collects: sweep the current with a clamp meter
  // on the wire, and regress the channel against it. Gain 0.9 (-10 %), offset
  // 30 mA.
  // 真实标定会采的那一对：一边扫电流一边用钳形表读导线，再把通道对它回归。增益
  // 0.9 （−10 %），零漂 30 mA。
  constexpr float kTrueGain = 0.9F;
  constexpr float kTrueOffset = 0.03F;
  core::CurrentSenseCalibrator calibrator(
      core::CurrentSenseCalibrator::Config{0.5F, 0.01F});
  for (const float truth : {-2.0F, -1.0F, -0.5F, 0.5F, 1.0F, 2.0F}) {
    EXPECT_TRUE(calibrator.add((kTrueGain * truth) + kTrueOffset, truth));
  }
  // Plus the zero-current points, which is how the two paths meet.
  // 再加上零电流点——两条路就是在这里汇合的。
  for (int k = 0; k < 8; ++k) {
    static_cast<void>(calibrator.add_zero(kTrueOffset));
  }
  const core::CurrentSenseCalibrator::Result fit = calibrator.solve();
  std::printf(
      "[P5-d] gain %.6f (truth %.6f, %+.4f %%), offset %.5f A (truth %.5f A), "
      "zero-current offset %.5f A, residual %.6f A\n",
      static_cast<double>(fit.gain), static_cast<double>(kTrueGain),
      static_cast<double>(100.0F * (fit.gain - kTrueGain) / kTrueGain),
      static_cast<double>(fit.offset_a), static_cast<double>(kTrueOffset),
      static_cast<double>(fit.zero_offset_a),
      static_cast<double>(fit.residual_a));

  EXPECT_TRUE(fit.gain_valid);
  EXPECT_TRUE(fit.offset_valid);
  EXPECT_NEAR(kTrueGain, fit.gain, 0.01F);
  // P5-d of the plan lives here: the offset to within 10 mA, from either path.
  // plan 的 P5-d 就在这里：零漂在 10 mA 以内，两条路走哪条都行。
  EXPECT_NEAR(kTrueOffset, fit.offset_a, 0.005F);
  EXPECT_NEAR(kTrueOffset, fit.zero_offset_a, 0.005F);
  // Two independent estimates of the same number must agree, or one path is
  // wrong. 同一个数的两个独立估计必须一致，否则其中一条路是错的。
  EXPECT_NEAR(fit.offset_a, fit.zero_offset_a, 0.005F);
}

TEST(Commissioning,
     P5d_TheThreeWireSumFindsThatSomethingIsWrongButNotWhichPhase) {
  // True phase currents sum to zero, so the *readings* sum to the sum of the
  // offsets: measurable with no reference at all. But it is one equation for
  // three unknowns, so it is a diagnostic and not a calibration. Both halves
  // are asserted.
  // 真实相电流之和为零，所以**读数**之和就是零漂之和：完全不需要基准就能测。但它是三个未知数的
  // 一个方程，所以它是诊断而不是标定。两半都在这里断言。
  const float offsets[3] = {0.05F, -0.02F, -0.01F};
  float sum = 0.0F;
  for (const float offset : offsets) {
    sum += offset;
  }
  std::printf(
      "[P5-d] three-wire sum with offsets (%.3f, %.3f, %.3f) A: %.4f A -- "
      "detected, not resolvable\n",
      static_cast<double>(offsets[0]), static_cast<double>(offsets[1]),
      static_cast<double>(offsets[2]), static_cast<double>(sum));

  EXPECT_GT(std::fabs(sum), 0.01F) << "the sum says a channel is off";
  // The same sum comes from infinitely many splits -- here is another one.
  // 同一个和有无穷多种拆法——这里是另一种。
  const float alt[3] = {0.03F, 0.03F, -0.04F};
  float alt_sum = 0.0F;
  for (const float value : alt) {
    alt_sum += value;
  }
  EXPECT_NEAR(sum, alt_sum, 1.0e-6F) << "the constraint cannot tell these two "
                                        "apart, so it cannot name a phase";
}

// ---------------------------------------------------------------------------
// P5-e: the order of the commissioning sequence, made quantitative.
// P5-e：把整定顺序这件事定量化。
// ---------------------------------------------------------------------------

TEST(Commissioning, P5e_ASensorOffsetCorruptsTheAlignedAngleSoTheOrderMatters) {
  // S10 already checks that the aligner captures the mount offset to 0.02 rad,
  // so P5-e's criterion is met -- what is *not* covered is that the aligner's
  // accuracy rests on the current sensors, and that is what decides the order
  // of the sequence. S10 已经验证对齐器把安装偏置抓到 0.02 rad 以内，所以 P5-e
  // 的判据是满足的——没被覆盖的是：
  // 对齐器的精度建立在电流传感器之上，而这决定了整定顺序。
  //
  // The aligner forces the current vector to a known electrical angle and reads
  // the encoder. A per-phase offset biases the vector it actually produced by
  // `offset/pull` radians (worst case, offset on the phase that is orthogonal
  // to the pull). At 100 mA on a 1 A pull that is 0.1 rad, five times the
  // aligner's own gate.
  // 对齐器把电流矢量强制到一个已知电角度再读编码器。逐相零漂让它真正产生的矢量偏
  // `offset/pull` 弧度（最坏情况：零漂落在与拉力正交的那一相上）。1 A 拉力上的
  // 100 mA 就是 0.1 rad，是对齐器自己那个闸的五倍。
  constexpr float kPullA = 1.0F;
  constexpr float kOffsetA = 0.10F;
  const float angle_error = std::atan2(kOffsetA, kPullA);
  std::printf(
      "\n[P5-e] a %.0f mA offset on a %.1f A alignment pull is %.4f rad "
      "(%.2f deg); the aligner's own gate is 0.02 rad (%.2f deg)\n",
      static_cast<double>(1000.0F * kOffsetA), static_cast<double>(kPullA),
      static_cast<double>(angle_error),
      static_cast<double>(57.2958F * angle_error),
      static_cast<double>(57.2958F * 0.02F));

  // Measured against the gate the aligner actually promises, so the ordering
  // claim is arithmetic rather than rhetorical.
  // 沿用它真正承诺的那个闸来比，所以"顺序"这个说法是算术，不是修辞。
  // 0.0997 rad measured: five times the gate, so the ordering claim is
  // arithmetic. 实测 0.0997 rad：五倍于闸，所以"顺序"是算术结论。
  EXPECT_GT(angle_error, 0.08F)
      << "calibrating the current sensors first is a precondition, not hygiene";
  // And it scales the way `atan` scales, which is *not* linearly: the error is
  // `atan(offset/pull)`, so halving the offset leaves slightly **more** than
  // half the error. The first version of this assertion said "neutral to four
  // digits" and the measurement said no (0.049958 against 0.049834) -- a
  // concave function, exactly as the geometry requires. 而且它按 `atan`
  // 的方式缩放，**不是**线性的：误差是 `atan(offset/pull)`，所以零漂减半留下
  // 的误差略**多于**一半。这个断言的第一版说"四位有效数字内相等"，实测否掉了（0.049958
  // 对 0.049834）——凹函数，正是几何要求的。
  const float half = std::atan2(0.5F * kOffsetA, kPullA);
  EXPECT_GT(half, 0.5F * angle_error)
      << "atan is concave, so the error is sublinear";
  EXPECT_LT(half, 0.505F * angle_error) << "but only just, at this angle";
}

// ---------------------------------------------------------------------------
// P5-f: the library-level claim -- three unrelated machines, one tuning.
// P5-f：库级断言——三台互不相干的机器，一套整定。
// ---------------------------------------------------------------------------

namespace {

/// [en] Deterministic pseudo-random factor in `[0.7, 1.3]`, so "three random
/// machines" means the same three every run. A flaky library-level gate would
/// be worse than no gate.
///      [zh] 确定性伪随机因子
///      `[0.7, 1.3]`，于是"三台随机机器"每次都是同样三台。会飘的库级闸
///      比没有闸更糟。
auto spread(std::uint32_t &state) noexcept -> float {
  state = (state * 1664525U) + 1013904223U;
  return 0.7F + (0.6F * (static_cast<float>(state >> 8) / 16777216.0F));
}

/// [en] The admission measurement S18 uses, on an arbitrary machine and at the
/// **nominal** tuning: the flux observer is told that machine's nameplate
/// numbers (`psi_f = Kt/(1.5*p)`) while `Ks` and `Ki` stay at 0.3 / 3.0. That
/// is what "no retuning" means.
///      [zh] S18
///      用的准入测量，跑在任意机器上、用**标称**整定：磁链观测器被告知该机器的铭牌数
///      （`psi_f = Kt/(1.5·p)`），而 `Ks`、`Ki` 仍是 0.3
///      / 3.0。这就是"不重调"。
auto gate_error(const sil::MachineSpec &spec, float gain_scale = 1.0F)
    -> float {
  sil::SwitchingRigConfig config{};
  config.spec = spec;
  const float psi = spec.kt / (1.5F * static_cast<float>(spec.pole_pairs));
  core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
      config.belief_rs * spec.rs, spec.lq, psi, 0.3F * gain_scale,
      3.0F * gain_scale, 0.5F * psi});
  observer.seed_flux(core::Ab<float>{psi, 0.0F});
  return sil::run_switched(config, [&observer](sil::SwitchingRig &rig) {
    return observer.update(rig.command(), rig.current(),
                           sil::SwitchingRig::kControlPeriod);
  });
}

} // namespace

TEST(Commissioning, P5f_ThreeUnrelatedMachinesAtOneTuning) {
  // The only library-level criterion in P5: swap the motor, keep the tuning,
  // and see whether the observer still lands inside the gate. Nothing is
  // re-tuned; the only change is three nameplate numbers. P5
  // 唯一的库级判据：换电机、保持整定，看观测器还能不能落进闸里。什么都不重调，变的只有三个
  // 铭牌数字。
  constexpr float kGateRad = 0.10F;
  std::uint32_t state = 12345U;
  sil::MachineSpec machines[3];
  for (int k = 0; k < 3; ++k) {
    machines[k] = sil::nominal_spec();
    machines[k].rs = sil::nominal_spec().rs * spread(state);
    machines[k].ld = sil::nominal_spec().ld * spread(state);
    machines[k].lq =
        machines[k].ld; // non-salient: the observer takes one L anyway
    machines[k].kt = sil::nominal_spec().kt * spread(state);
  }
  machines[0] =
      sil::nominal_spec(); // the incumbent's own machine, as a reference

  std::printf("\n[P5-f] gate %.2f rad, one tuning, three machines\n",
              static_cast<double>(kGateRad));
  float errors[3]{};
  int inside = 0;
  for (int k = 0; k < 3; ++k) {
    errors[k] = gate_error(machines[k]);
    inside += (errors[k] <= kGateRad) ? 1 : 0;
    std::printf(
        "        #%d: rs %.4f, L %.2e, kt %.4f (psi_f %.2e) -> %.5f rad %s\n",
        k, static_cast<double>(machines[k].rs),
        static_cast<double>(machines[k].lq),
        static_cast<double>(machines[k].kt),
        static_cast<double>(
            machines[k].kt /
            (1.5F * static_cast<float>(machines[k].pole_pairs))),
        static_cast<double>(errors[k]), (errors[k] <= kGateRad) ? "in" : "OUT");
  }
  std::printf("        %d of 3 inside the gate\n", inside);

  // **P5-f fails, and it fails at the reference machine.** 0.10373 rad against
  // a 0.10 gate is the same number S18 recorded for the incumbent on a
  // switching inverter, so the plan's promise
  // ("three machines, no retuning, all inside 0.10") was never reachable -- it
  // was written before that number existed. Recorded rather than retuned: a
  // threshold moved to fit the measurement is not a threshold.
  // **P5-f 失败，而且在参考机器上就失败。** 0.10373 rad 对 0.10 的闸，正是 S18
  // 在开关逆变器上 记下的那个数，所以计划里那句"三台机器、不重调、全在 0.10
  // 内"从来就没达到过——它是在那个数
  // 出现之前写的。记下来，而不是调掉：为迁就测量而挪动的阈值不是阈值。
  EXPECT_GT(errors[0], kGateRad)
      << "as written, the reference machine is 3.7 % outside its own gate";

  // The three machines rank by how far their rotor flux is from nominal, not by
  // `rs` and not by `L`: #1 has kt x0.77 and is 77 % outside, #2 has kt x1.06
  // and is inside, #0 is exact and is 3.7 % outside. That is the diagnostic the
  // rollback rule asks for. 三台机器按"转子磁链离标称有多远"排序，而不是按 `rs`
  // 或 `L`：#1 的 kt×0.77 在闸外 77 %， #2 的 kt×1.06 在闸内，#0
  // 精确相等却在闸外 3.7 %。这正是回退规则要的那个诊断。
  const float flux_ratio[3] = {machines[0].kt / sil::nominal_spec().kt,
                               machines[1].kt / sil::nominal_spec().kt,
                               machines[2].kt / sil::nominal_spec().kt};
  const float deviation[3] = {std::fabs(flux_ratio[0] - 1.0F),
                              std::fabs(flux_ratio[1] - 1.0F),
                              std::fabs(flux_ratio[2] - 1.0F)};
  // Measured 4.0x (0.2325 against 0.0577): three is the claim the data
  // supports.
  EXPECT_GT(deviation[1], 3.0F * deviation[2])
      << "the flux ratios must actually differ";
  EXPECT_GT(errors[1], errors[2]) << "the machine furthest from nominal flux "
                                     "is the one furthest outside the gate";
  EXPECT_LE(errors[2], 0.15F) << "the near-nominal machine is the control case";

  // Ablation on machine #1 (`kt` alone, then `rs` alone, then `L` alone):
  // 0.16075 / 0.10619 / 0.11217 against 0.10373 for nominal. So `kt` moves the
  // error 55 %, `L` 8 %, `rs` 2 %. Scaling the observer's `Ks`/`Ki` by the flux
  // ratio only gets machine #1 to 0.13190 -- a third of the way -- so "make the
  // gains dimensionless" is necessary and not sufficient. 对 1
  // 号机做单参数消融（只改 `kt`、只改 `rs`、只改 `L`）：0.16075 / 0.10619 /
  // 0.11217，标称是 0.10373。所以 `kt` 让误差走 55 %，`L` 8 %，`rs` 2
  // %。把观测器的 `Ks`/`Ki` 按磁链比缩放只把 1 号机拉到
  // 0.13190——走了三分之一——所以"把增益做成无量纲"是**必要而不充分**的。
  static_cast<void>(inside);
}

// P5-c: incremental inductance from the flux integral.
// P5-c：从磁链积分求增量电感。
// ---------------------------------------------------------------------------

namespace {

/// [en] Step one axis of a standstill rig from zero current and integrate
/// exactly `periods` switching periods. `hold_shaft` freezes the mechanical
/// equation -- a real commissioning locks the shaft, and P5-c's second test
/// shows what happens without it.
///      [zh] 在堵转台架上给一个轴加阶跃，从零电流出发，精确积分 `periods`
///      个开关周期。`hold_shaft` 冻结机械方程——真实整定会把轴锁死；P5-c
///      的第二个测试展示不锁会怎样。
auto step_axis(const sil::PlantParams &params, bool on_d_axis, int periods,
               bool hold_shaft)
    -> core::IncrementalInductanceEstimator::Result {
  sil::PlantParams machine = params;
  if (hold_shaft) {
    machine.j = 1.0e6F;
    machine.b = 0.0F;
  }
  StandstillRig rig(machine);
  // The loss an axis sees depends on the angle the current sits at, so the d
  // and q steps need different numbers -- see `axis_deadtime_v`.
  // 一个轴看到的损失取决于电流所在的角度，所以 d、q 阶跃要不同的数——见
  // `axis_deadtime_v`。
  const float axis_loss =
      core::axis_deadtime_v(kVDead, on_d_axis ? 0.0F : 1.5707964F);
  core::IncrementalInductanceEstimator estimator(
      core::IncrementalInductanceEstimator::Config{params.rs, axis_loss, 0.5F});
  const float v_step = 1.6F;

  // Settle at zero volts, then begin from the measured mean current.
  // 先在零电压上稳定，再从实测均值电流开始。
  const core::Dq<float> before = rig.hold(0.0F, 0.0F, 40);
  estimator.begin(on_d_axis ? before.d : before.q);

  for (int k = 0; k < periods; ++k) {
    const core::Dq<float> mean =
        rig.period(on_d_axis ? v_step : 0.0F, on_d_axis ? 0.0F : v_step);
    static_cast<void>(
        estimator.add(v_step, on_d_axis ? mean.d : mean.q, kPeriod));
  }
  return estimator.solve();
}

} // namespace

TEST(Commissioning, P5c_InductanceComesOutOfTheFluxIntegralOnEitherAxis) {
  // A salient machine, so a d/q mix-up cannot hide. The nominal machine has `Ld
  // == Lq`, and a test on it would pass with the axes swapped -- which is the
  // failure mode worth catching, because `Ld` and `Lq` differ by a factor of
  // two or more on a real IPM. 用凸极机，d/q 弄反藏不住。标称机 `Ld ==
  // Lq`，在它上面做测试即使轴弄反了也会过——而真实的 IPM 上 `Ld` 与 `Lq`
  // 差两倍以上，正是这个失效模式最值得抓。
  const sil::PlantParams params = salient_params();
  const core::IncrementalInductanceEstimator::Result on_d =
      step_axis(params, true, 48, true);
  const core::IncrementalInductanceEstimator::Result on_q =
      step_axis(params, false, 48, true);

  std::printf(
      "\n[P5-c] salient machine (Ld %.2e, Lq %.2e H):\n"
      "        d-axis: L %.5e H (%+.2f %%), flux %.5e Wb, di %.4f A\n"
      "        q-axis: L %.5e H (%+.2f %%), flux %.5e Wb, di %.4f A\n",
      static_cast<double>(params.ld), static_cast<double>(params.lq),
      static_cast<double>(on_d.l_h),
      static_cast<double>(100.0F * (on_d.l_h - params.ld) / params.ld),
      static_cast<double>(on_d.flux_wb), static_cast<double>(on_d.delta_i_a),
      static_cast<double>(on_q.l_h),
      static_cast<double>(100.0F * (on_q.l_h - params.lq) / params.lq),
      static_cast<double>(on_q.flux_wb), static_cast<double>(on_q.delta_i_a));

  EXPECT_TRUE(on_d.valid);
  EXPECT_TRUE(on_q.valid);
  // P5-c of the plan: within 10 %.
  // plan 的 P5-c：10 % 以内。
  EXPECT_NEAR(params.ld, on_d.l_h, 0.10F * params.ld);
  EXPECT_NEAR(params.lq, on_q.l_h, 0.10F * params.lq);
  // The two must be distinguishable, or "either axis" is not a claim worth
  // making. 两者必须可区分，否则"两个轴都行"这句话不值得说。
  EXPECT_GT(1.5F * on_d.l_h, 0.0F);
  EXPECT_GT(on_q.l_h, 1.5F * on_d.l_h)
      << "Lq is twice Ld here; a d/q swap would show up as this failing";
}

TEST(Commissioning,
     P5c_TheAxisProjectionIsWhatTheQStepNeedsAndTheDNumberIsNotIt) {
  // Two hypotheses died here, and both died by measurement rather than by
  // argument. 两个假设在这里死掉，都是被测量杀掉的，不是被辩论杀掉的。
  //
  //   1. A *crossing* bias (`deadtime_v * t_cross`) was the first guess. Wrong:
  //   the two axes erred in
  //      **opposite** directions (d +2.9 %, q -7.0 %), and a single
  //      over-subtraction cannot do that.
  //      第一个猜想是**过零**偏差。错：两个轴误差**方向相反**（d +2.9 %，q −7.0
  //      %），单一"减多了" 造不出这个。
  //   2. A *mechanical* one (`the q step makes torque, the rotor accelerates,
  //   the back-EMF is missing`)
  //      was the second. Also wrong, and the test that killed it was locking
  //      the shaft: the locked reading was *worse*, -15.7 % against -13.2 %.
  //      第二个是**机械**猜想（q
  //      轴阶跃产生转矩、转子加速、缺反电动势）。也错，杀掉它的实验是锁轴：
  //      锁住之后反而更差，−15.7 % 对 −13.2 %。
  //
  // What it actually is: the dead-time step projects onto an axis with a factor
  // that depends on where the current is. On d the phases carry `(+i, -i/2,
  // -i/2)` and the factor is `(2/3)*(1+1/2+1/2) = 4/3`; on q they carry `(0,
  // +i*sqrt(3)/2, -i*sqrt(3)/2)`, one phase carries nothing, and the factor is
  // `(2/3)*(sqrt(3)/2+sqrt(3)/2) = 2/sqrt(3)`. At 24 V that is 0.640 V against
  // 0.554 V -- a 0.085 V shortfall, and 0.085 V over 2.4 ms is 2.0e-4 Wb, which
  // is 16 % of `Lq*delta_i`. The measurement and the model agree to the third
  // digit, which is why this is a finding and not a patch: using the d-axis
  // number on the q axis is a *predictable* 16 % error.
  // 真正的原因是：死区阶跃投影到一个轴上，系数取决于电流在哪。d 轴相电流是
  // `(+i, −i/2, −i/2)`， 系数 `(2/3)·(1+1/2+1/2) = 4/3`；q 轴是 `(0, +i·√3/2,
  // −i·√3/2)`，有一相不带电流，系数
  // `(2/3)·(√3/2+√3/2) = 2/√3`。24 V 下是 0.640 V 对 0.554 V——差 0.085 V，而
  // 0.085 V 在 2.4 ms 上是 2.0e-4 Wb，正是 `Lq·Δi` 的 16
  // %。测量与模型三位有效数字都对得上，所以这是一条**发现**而 不是补丁：在 q
  // 轴上用 d 轴的数，是一个**可预测的** 16 % 误差。
  const sil::PlantParams params = salient_params();
  const core::IncrementalInductanceEstimator::Result right =
      step_axis(params, false, 48, true);

  // The same step, but told the d-axis projection.
  // 同一个阶跃，但给它 d 轴的投影。
  StandstillRig rig(params);
  sil::PlantParams locked = params;
  locked.j = 1.0e6F;
  locked.b = 0.0F;
  StandstillRig locked_rig(locked);
  core::IncrementalInductanceEstimator wrong(
      core::IncrementalInductanceEstimator::Config{params.rs, kVAxisDead,
                                                   0.5F});
  const core::Dq<float> before = locked_rig.hold(0.0F, 0.0F, 40);
  wrong.begin(before.q);
  for (int k = 0; k < 48; ++k) {
    const core::Dq<float> mean = locked_rig.period(0.0F, 1.6F);
    static_cast<void>(wrong.add(1.6F, mean.q, kPeriod));
  }
  const core::IncrementalInductanceEstimator::Result misinformed =
      wrong.solve();
  static_cast<void>(rig);

  std::printf(
      "[P5-c] q-axis step, Lq truth %.3e H: right projection %.3e (%+.2f %%), "
      "d-axis projection %.3e (%+.2f %%)\n",
      static_cast<double>(params.lq), static_cast<double>(right.l_h),
      static_cast<double>(100.0F * (right.l_h - params.lq) / params.lq),
      static_cast<double>(misinformed.l_h),
      static_cast<double>(100.0F * (misinformed.l_h - params.lq) / params.lq));

  // The right projection lands inside the 10 % gate; the wrong one does not,
  // and it lands *low* because too much loss was subtracted from the flux.
  // 对的投影落进 10 %
  // 闸内；错的落不进去，而且偏低，因为从磁链里减掉了太多损失。
  EXPECT_NEAR(params.lq, right.l_h, 0.10F * params.lq);
  EXPECT_LT(misinformed.l_h, right.l_h)
      << "the d-axis number over-subtracts on the q axis";
  EXPECT_LT(misinformed.l_h, 0.90F * params.lq)
      << "and by more than the gate, or this is not worth a test";
}

TEST(Commissioning, P5c_AStepTooShortToSpanIsNotAnIdentification) {
  // Three periods move the current a few hundred milliamps; dividing by that is
  // dividing by noise, and the estimator has to refuse rather than report a
  // confident wrong number.
  // 三个周期只推动几百毫安；拿它做除数是拿噪声做除数，估计器必须拒绝，而不是自信地报一个错数。
  const sil::PlantParams params = salient_params();
  const core::IncrementalInductanceEstimator::Result short_step =
      step_axis(params, true, 1, true);

  core::IncrementalInductanceEstimator never_begun(
      core::IncrementalInductanceEstimator::Config{
          params.rs, core::axis_deadtime_v(kVDead, 0.0F), 0.5F});
  static_cast<void>(never_begun.add(1.6F, 0.4F, kPeriod));
  static_cast<void>(never_begun.add(1.6F, 0.8F, kPeriod));
  static_cast<void>(never_begun.add(1.6F, 1.2F, kPeriod));

  std::printf(
      "[P5-c] one-period step: valid %d, di %.4f A; never begun: valid %d\n",
      short_step.valid ? 1 : 0, static_cast<double>(short_step.delta_i_a),
      never_begun.solve().valid ? 1 : 0);

  EXPECT_FALSE(short_step.valid) << "one period is not enough to divide by";
  EXPECT_FALSE(never_begun.solve().valid)
      << "begin() is what makes it a measurement";
}

} // namespace foc
