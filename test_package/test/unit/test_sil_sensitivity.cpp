// T1 - the sensitivity table. The SIL plant is a model, and a model's
// unmodelled effects are exactly where a benchmark lies. This file turns "we
// assumed it was negligible" into a number with a threshold, for each effect
// separately. T1——敏感度表。SIL plant
// 是模型，而模型里未建模的效应正是基准撒谎的地方。本文件把
// "我们假设它可以忽略"变成一个有阈值的数，逐效应给出。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/transforms.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"

namespace foc {

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

constexpr float kPi = 3.14159265358979F;
constexpr float kTwoPi = 6.28318530717959F;
constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 8;

/**
 * @brief [en] The criterion, written before the sweep was run.
 *        [zh] 判据，写在跑扫描之前。
 *
 * @note [en] Three rules, and the third is the one that matters.
 * [zh] 三条规则，第三条才是要紧的。
 *
 * 1. [en] Any single effect at a *plausible worst case* must leave the angle
 *    error at or below `kCeilingRad`. An effect that breaks this must not be
 *    described as "negligible" anywhere in the docs.
 *    [zh] 任何单一效应在**合理最坏**取值下，角度误差必须 ≤ `kCeilingRad`。
 *    破坏这一条的效应，文档里任何地方都不得写成"可忽略"。
 * 2. [en] An effect that raises the error by more than `kMaterialRatio` over the clean baseline goes on the "the user
 * must measure this" list -- it is not a modelling nicety, it is the difference between a valid and an invalid
 * benchmark on their hardware.
 *    [zh] 相对干净基线抬高超过 `kMaterialRatio` 的效应进入"必须由用户实测"清单——它不是建模
 *    上的讲究，而是"在用户的硬件上基准有效还是无效"的区别。
 * 3. [en] An effect may only be called "negligible" if it moves the error by
 * less than `kNegligibleRatio` **and** that was measured here. "Obviously
 * small" is not an entry point. [zh] 只有在本文件里量到误差变化小于
 * `kNegligibleRatio` 的效应，才允许被称作"可忽略"。 "显然很小"不是入场券。
 */
constexpr float kCeilingRad = 0.15F;
constexpr float kMaterialRatio = 1.15F;
constexpr float kNegligibleRatio = 1.02F;

/// [en] Light-load high speed: the case where an uncompensated iron-loss branch
/// stops being a perturbation and becomes the dominant current. [zh]
/// 轻载高速：未补偿的铁损支路从"扰动"变成"主导电流"的那个情形。
constexpr float kHighSpeedRad = 2800.0F;
constexpr float kHighSpeedIq = 0.1F;

auto wrap_pi(float angle) noexcept -> float {
  while (angle <= -kPi) {
    angle += kTwoPi;
  }
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  return angle;
}

auto psi_f_value() noexcept -> float {
  const sil::MachineSpec s = sil::nominal_spec();
  return s.kt / (1.5F * static_cast<float>(s.pole_pairs));
}

/**
 * @brief [en] One row of the sweep: which effect, how much of it.
 *        [zh] 扫描的一行：哪个效应，多大幅度。
 *
 * @since 0.1.0
 */
struct Effects final {
  float dsat_depth{0.0F};
  float qsat_depth{0.0F};
  float rm_ohm{0.0F};
  float gain_u{1.0F};
  float gain_v{1.0F};
  float gain_w{1.0F};
  float offset_a{0.0F};
  float noise_a{0.0F};
  float deadtime_v{0.0F};
};

/**
 * @brief [en] Constant-speed rig, one effect set at a time, sensor errors
 * applied on the *reading*. [zh]
 * 恒速台架，一次只开一组效应，传感误差作用在**读数**上。
 *
 * @note [en] The profile switches, then the voltages are set, so the sample the
 * observer sees is taken after the switching period -- the same order as the
 * switching rig, and the same order a real controller runs in.
 * [zh]
 * 先切开关、再给电压，所以观测器看到的采样点取在开关周期之后——与开关台架同序，
 * 也与真实控制器的运行同序。
 * @since 0.1.0
 */
class SweepRig final {
public:
  SweepRig(float omega_e, float iq_target, const Effects &fx) noexcept
      : plant_(pinned_plant_params()), omega_e_(omega_e) {
    plant_.set_d_axis_saturation(fx.dsat_depth, 1.0F);
    plant_.set_q_axis_saturation(fx.qsat_depth, 1.0F);
    plant_.set_iron_loss(fx.rm_ohm);
    plant_.set_current_noise(fx.noise_a);
    fx_ = fx;
    const float poles = static_cast<float>(sil::nominal_spec().pole_pairs);
    plant_.reset(sil::PlantState{0.0F, iq_target, omega_e / poles, 0.0F});
    iq_target_ = iq_target;
    const sil::MachineSpec s = sil::nominal_spec();
    vd_ = -omega_e * s.lq * iq_target;
    vq_ = (s.rs * iq_target) + (omega_e * psi_f_value());
  }

  SweepRig(const SweepRig &) = delete;
  SweepRig &operator=(const SweepRig &) = delete;
  SweepRig(SweepRig &&) = delete;
  SweepRig &operator=(SweepRig &&) = delete;

  /// [en] Pin the rotor: with `j` nominal the plant accelerates at
  /// `Te/j = 250 rad/s^2` and the run would measure a different speed than the
  /// one in its label. Every row in this file therefore shares one speed gate.
  /// [zh] 把转子钉住：用标称 `j` 时 plant 以 `Te/j = 250 rad/s²`
  /// 加速，这一趟量的 就不是标签上那个转速。因此本文件每一行共用同一道转速闸。
  [[nodiscard]] static auto pinned_plant_params() noexcept -> sil::PlantParams {
    sil::PlantParams p = sil::nominal_plant_params();
    p.j = 1.0e6F;
    return p;
  }

  [[nodiscard]] auto true_angle_e() const noexcept -> float {
    return static_cast<float>(sil::nominal_spec().pole_pairs) *
           plant_.theta_mech_rad();
  }
  [[nodiscard]] auto command() const noexcept -> core::Ab<float> {
    return command_;
  }
  [[nodiscard]] auto current() const noexcept -> core::Ab<float> {
    return sensed_;
  }
  /// [en] The current the plant actually settles at, for the report. [zh]
  /// plant 实际稳定到的电流，用于报告。
  [[nodiscard]] auto magnetizing_iq() const noexcept -> float {
    return plant_.magnetizing_current_dq().q;
  }
  /// [en] The current the sensor reports, A. [zh] 传感器报告的电流，A。
  [[nodiscard]] auto sensed_iq() const noexcept -> float {
    return plant_.current_dq().q;
  }

  void step() noexcept {
    const float theta = true_angle_e();
    // The command that puts the *equilibrium* at the target state, with the
    // shunt's demand fed forward. Solved on the target state, not on the
    // current one: reading `ic` back from the plant closes a loop with gain
    // `omega*L/Rs` (2.24 here), which walks the operating point away instead of
    // pinning it. That was v1 of this rig, and the test caught it.
    // 这个指令把**平衡点**放在目标状态上，并把并联支路的需求前馈进去。要在目标状态上求解，不能
    // 从 plant 读回 `ic`：那会闭成一个增益
    // `ω·L/Rs`（这里是 2.24）的环，把工作点越推越远而不是
    // 钉住。这是本台架的第一版，测试抓到了。
    const sil::MachineSpec spec = sil::nominal_spec();
    const float rm = (fx_.rm_ohm > 0.0F) ? fx_.rm_ohm : 1.0e9F;
    const float icq = (omega_e_ * psi_f_value()) / rm;
    vd_ = -omega_e_ * spec.lq * iq_target_;
    vq_ = static_cast<float>((spec.rs * (iq_target_ + icq)) +
                             (omega_e_ * psi_f_value()));
    command_ = core::inv_park(core::Dq<float>{vd_, vq_}, theta);
    // Sample the sensor, with per-phase gain and offset applied to the reading.
    // 采传感器，增益与零偏逐相加在读数上。
    const core::Uvw<float> raw = plant_.current_uvw();
    const core::Uvw<float> seen{
        static_cast<float>((fx_.gain_u * raw.u) + fx_.offset_a),
        static_cast<float>((fx_.gain_v * raw.v) + fx_.offset_a),
        static_cast<float>((fx_.gain_w * raw.w) + fx_.offset_a)};
    sensed_ = core::clarke(seen);

    // The inverter keeps the dead-time volt-seconds; the observer only ever
    // sees the command. 逆变器留下死区伏秒；观测器永远只看到指令。
    const auto loss = [this](float i) noexcept -> float {
      if (!(fx_.deadtime_v > 0.0F)) {
        return 0.0F;
      }
      const float n = i / 0.2F;
      return (n > 1.0F)
                 ? fx_.deadtime_v
                 : ((n < -1.0F) ? -fx_.deadtime_v : (fx_.deadtime_v * n));
    };
    const core::Uvw<float> v_uvw = core::inv_clarke(command_);
    const core::Ab<float> delivered = core::clarke(
        core::Uvw<float>{static_cast<float>(v_uvw.u - loss(raw.u)),
                         static_cast<float>(v_uvw.v - loss(raw.v)),
                         static_cast<float>(v_uvw.w - loss(raw.w))});
    const core::Dq<float> v_dq = core::park(delivered, theta);
    for (int s = 0; s < kSubsteps; ++s) {
      plant_.set_voltage_dq(v_dq);
      plant_.advance(kDt / static_cast<float>(kSubsteps));
    }
  }

private:
  sil::PmsmPlant plant_;
  float omega_e_;
  float iq_target_{0.0F};
  float vd_{0.0F};
  float vq_{0.0F};
  Effects fx_{};
  core::Ab<float> command_{0.0F, 0.0F};
  core::Ab<float> sensed_{0.0F, 0.0F};
};

/// [en] Mean |angle error| over the second half of the run, plus the current
/// the plant settled at. One observer, one configuration, no per-scenario
/// tuning. [zh] 后半段角度误差绝对均值，加上 plant
/// 稳定到的电流。一份观测器配置，不针对场景调参。
struct Outcome final {
  float angle_rad;
  float magnetizing_iq_a;
  float sensed_iq_a;
};

auto measure(float omega_e, float iq_target, const Effects &fx) -> Outcome {
  SweepRig rig(omega_e, iq_target, fx);
  const sil::MachineSpec s = sil::nominal_spec();
  core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
      1.10F * s.rs, s.lq, psi_f_value(), 0.3F, 3.0F, 0.5F * psi_f_value()});
  observer.seed_flux(core::Ab<float>{psi_f_value(), 0.0F});
  float sum = 0.0F;
  int count = 0;
  const int steps = static_cast<int>(0.4F / kDt);
  for (int k = 0; k < steps; ++k) {
    rig.step();
    const core::ObsState observed =
        observer.update(rig.command(), rig.current(), kDt);
    if (k >= (steps / 2)) {
      sum += std::fabs(wrap_pi(observed.theta_e - rig.true_angle_e()));
      ++count;
    }
  }
  return Outcome{sum / static_cast<float>(count), rig.magnetizing_iq(),
                 rig.sensed_iq()};
}

/**
 * @brief [en] One row of the printed table. [zh] 打印表格的一行。
 * @since 0.1.0
 */
struct Row final {
  const char *name;
  Outcome at_slow; ///< [en] 20 rad/s. [zh]
  Outcome at_mid;  ///< [en] 200 rad/s. [zh]
  Outcome at_fast; ///< [en] 800 rad/s. [zh]
};

auto measure_all(const Effects &fx) -> Row {
  return Row{"", measure(20.0F, 0.5F, fx), measure(200.0F, 0.5F, fx),
             measure(800.0F, 0.5F, fx)};
}

} // namespace

TEST(SilSensitivity, EachUnmodelledEffectHasAMeasuredBound) {
  const Row base = measure_all(Effects{});

  std::printf(
      "\n[T1] angle error (rad), NLFO observer, iq = 0.5 A, rotor pinned\n");
  std::printf("     %-26s %10s %10s %10s\n", "effect", "20 rad/s", "200 rad/s",
              "800 rad/s");
  const auto print_row = [](const Row &r) {
    std::printf("     %-26s %10.4f %10.4f %10.4f\n", r.name,
                static_cast<double>(r.at_slow.angle_rad),
                static_cast<double>(r.at_mid.angle_rad),
                static_cast<double>(r.at_fast.angle_rad));
  };
  print_row(Row{"ideal plant", base.at_slow, base.at_mid, base.at_fast});

  // Each structure owns its measured row so the assertions below can name it.
  // 每个结构体各自持有量到的那一行，下面断言就能指名道姓。
  const Row dsat = [&] {
    Effects fx{};
    fx.dsat_depth = 0.3F;
    Row r = measure_all(fx);
    r.name = "d-saturation 30 % @ 1 A";
    return r;
  }();
  const Row qsat = [&] {
    Effects fx{};
    fx.qsat_depth = 0.3F;
    Row r = measure_all(fx);
    r.name = "q-saturation 30 % @ 1 A";
    return r;
  }();
  const Row rm2000 = [&] {
    Effects fx{};
    fx.rm_ohm = 2000.0F;
    Row r = measure_all(fx);
    r.name = "iron loss Rm 2000 ohm";
    return r;
  }();
  const Row rm200 = [&] {
    Effects fx{};
    fx.rm_ohm = 200.0F;
    Row r = measure_all(fx);
    r.name = "iron loss Rm 200 ohm";
    return r;
  }();
  const Row gain10 = [&] {
    Effects fx{};
    fx.gain_u = 1.10F;
    fx.gain_v = 1.10F;
    fx.gain_w = 1.10F;
    Row r = measure_all(fx);
    r.name = "sensor gain +10 % uniform";
    return r;
  }();
  const Row gain_mismatch = [&] {
    Effects fx{};
    fx.gain_u = 1.10F;
    fx.gain_v = 0.90F;
    fx.gain_w = 1.00F;
    Row r = measure_all(fx);
    r.name = "sensor gain u+10 v-10 %";
    return r;
  }();
  const Row offset = [&] {
    Effects fx{};
    fx.offset_a = 0.05F;
    Row r = measure_all(fx);
    r.name = "sensor offset 50 mA common";
    return r;
  }();
  const Row combined = [&] {
    Effects fx{};
    fx.dsat_depth = 0.3F;
    fx.qsat_depth = 0.3F;
    fx.rm_ohm = 200.0F;
    fx.gain_u = 1.10F;
    fx.gain_v = 0.90F;
    fx.offset_a = 0.05F;
    fx.noise_a = 0.1F;
    fx.deadtime_v = 0.48F;
    Row r = measure_all(fx);
    r.name = "all of the above together";
    return r;
  }();

  for (const Row *r : {&dsat, &qsat, &rm2000, &rm200, &gain10, &gain_mismatch,
                       &offset, &combined}) {
    print_row(*r);
  }

  // Rule 0, the one this file had to add after the first run: the table is only
  // read where the *baseline* passes. At 20 rad/s the ideal plant already reads
  // above the ceiling, so that column says nothing about any effect -- it is
  // measuring the observer's own low-speed floor, which is a separate and
  // already-known limit (handover below ~50 rad/s).
  // 规则 0，第一次跑完才加的那条：表只在**基线**通过的转速上读。20 rad/s 时理想
  // plant
  // 就已经高于天花板，那一列对任何效应都不说明问题——它量的是观测器自己的低速地板，
  // 那是另一个已知道限制（~50 rad/s 以下的切换）。
  std::printf(
      "     rule 0: the 20 rad/s column is the observer's own floor, not an "
      "effect reading\n");
  EXPECT_GT(base.at_slow.angle_rad, kCeilingRad)
      << "if this ever passes, the low-speed floor moved and the docs must say "
         "so";

  // Rule 1: nothing at a plausible worst case breaks the ceiling, *where the
  // baseline meets it*.
  // 规则 1：在基线达标的地方，任何效应在合理最坏取值下都不得击穿天花板。
  for (const Row *r : {&dsat, &qsat, &rm2000, &rm200, &gain10, &gain_mismatch,
                       &offset, &combined}) {
    EXPECT_LT(r->at_mid.angle_rad, kCeilingRad) << r->name;
    EXPECT_LT(r->at_fast.angle_rad, kCeilingRad) << r->name;
  }
  EXPECT_LT(base.at_mid.angle_rad, kCeilingRad);
  EXPECT_LT(base.at_fast.angle_rad, kCeilingRad);

  // Rule 2: the classification, generated from the measurement rather than
  // asserted by hand, and printed so the "must measure this" list is derived
  // from data instead of taste.
  // 规则
  // 2：分类由测量生成而不是手写，并打印出来，好让"必须实测"清单出自数据而非口味。
  std::printf(
      "     rule 2, material (> %.2fx baseline where the baseline passes):",
      static_cast<double>(kMaterialRatio));
  int material = 0;
  for (const Row *r : {&dsat, &qsat, &rm2000, &rm200, &gain10, &gain_mismatch,
                       &offset, &combined}) {
    const float worst =
        std::fmax(r->at_mid.angle_rad / base.at_mid.angle_rad,
                  r->at_fast.angle_rad / base.at_fast.angle_rad);
    if (worst > kMaterialRatio) {
      std::printf(" %s (%.2fx)", r->name, static_cast<double>(worst));
      ++material;
    }
  }
  std::printf("\n");
  EXPECT_GE(material, 1)
      << "the sweep must find at least one material effect, or it is not "
         "reaching the observer";

  // Rule 3: the three effects that measured inside `kNegligibleRatio` of the
  // baseline, each pinned so a future model change cannot quietly make them
  // matter without this file saying so.
  // 规则 3：量到落在 `kNegligibleRatio`
  // 以内的三个效应，逐个钉住，好让将来改模型不能悄悄
  // 让它们变重要而不被本文件说出来。
  //
  // Iron loss at 0.5 A really is negligible *at this load*: the shunt draws
  // `omega*psi_f/Rm` regardless of load, so "negligible" is a statement about
  // the operating point, not about the effect. The next test makes that the
  // point. 0.5 A 下铁损确实可忽略——但并联支路抽走的 `ω·ψf/Rm`
  // 与负载无关，所以"可忽略"是关于
  // 工作点的陈述，不是关于效应的。下一个测试正是要把这一点做成结论。
  // Iron loss at 0.5 A moves the error 4 % -- inside "material", outside
  // "negligible". It stays at 4 % because the shunt draws `omega*psi_f/Rm`
  // regardless of load, and at 0.5 A that is 2.4 % of the current. The next
  // test gives the case where load falls to 0.1 A. 0.5 A 下铁损把误差推高 4
  // %——在"重要"以内、"可忽略"以外。它停在 4 %，是因为并联 支路抽走的 `ω·ψf/Rm`
  // 与负载无关，而在 0.5 A 时它是电流的 2.4 %。下一个测试给出负载 降到 0.1 A
  // 的情形。
  EXPECT_GT(rm200.at_mid.angle_rad, base.at_mid.angle_rad);
  EXPECT_LT(rm200.at_mid.angle_rad, kMaterialRatio * base.at_mid.angle_rad);
  // d-axis saturation is *identically* inert here because `i_d = 0`: this row
  // is a null test of the rig, not evidence that d-saturation never matters (at
  // 50 rad/s with `i_d < 0` the same knob moved the error 16-fold).
  // 这里 `i_d = 0`，d 轴饱和**恒等**不动：这一行是台架的空测，不是 d
  // 轴饱和永远无关的证据 （在 50 rad/s、`i_d < 0` 时同一个旋钮把误差推高了 16
  // 倍）。
  EXPECT_NEAR(dsat.at_mid.angle_rad, base.at_mid.angle_rad, 1.0e-4F);
  EXPECT_NEAR(dsat.at_fast.angle_rad, base.at_fast.angle_rad, 1.0e-4F);
  // A common offset on all three phases cancels *identically* in Clarke -- the
  // measurement lands inside 0.1 % because it is an algebraic identity, not an
  // approximation. That is exactly why the board self-test measures a per-phase
  // offset (P7) rather than a common one: the common case is invisible.
  // 三相共模零偏在 Clarke 里**恒等**抵消——量到的差别在 0.1 %
  // 以内，因为它是代数恒等式
  // 而不是近似。这正是板级自检要量**逐相**零偏（P7）的原因：共模那种根本看不见。
  EXPECT_NEAR(offset.at_fast.angle_rad, base.at_fast.angle_rad,
              1.0e-3F * base.at_fast.angle_rad);
  // Differential gain mismatch turned out far smaller than predicted -- it
  // lands as a ~5 % magnitude change plus a ~1.6 deg rotation, and the observer
  // absorbs most of the rotation. The prediction was wrong and the number
  // stays. 差分增益失配比预想的小得多——它落成 ~5 % 幅值变化加 ~1.6°
  // 旋转，观测器把旋转吃掉了大半。 预想错了，数字保留。
  EXPECT_LT(gain_mismatch.at_mid.angle_rad,
            kMaterialRatio * base.at_mid.angle_rad);
}

TEST(SilSensitivity,
     IronLossIsLoadIndependentWhichIsWhyItCannotBeATorqueLoss) {
  // The plant-level property that makes iron loss a different animal from a
  // shaft torque loss: the shunt draws `omega*psi_f/Rm`, which does not contain
  // the load. A torque-loss term would scale with load and could never
  // reproduce "the sensor is reading 67 % more current than makes torque".
  // 让铁损与轴转矩损失本质不同的 plant 层性质：并联支路抽走
  // `ω·ψf/Rm`，里面没有负载。
  // 转矩损失项会随负载缩放，永远造不出"传感器读数比产生转矩的电流多 67
  // %"这种事。
  const float speeds[2] = {kHighSpeedRad, 200.0F};
  for (const float omega : speeds) {
    Effects iron{};
    iron.rm_ohm = 200.0F;
    const Outcome light = measure(omega, 0.1F, iron);
    const Outcome heavy = measure(omega, 0.5F, iron);
    // What the shunt diverts is `sensed - magnetizing`: the state, not the
    // command, is what makes torque.
    // 并联支路分走的就是 `sensed − magnetizing`：产生转矩的是状态，不是指令。
    const float drawn_light = light.sensed_iq_a - light.magnetizing_iq_a;
    const float drawn_heavy = heavy.sensed_iq_a - heavy.magnetizing_iq_a;
    const float predicted = psi_f_value() * omega / 200.0F;
    std::printf(
        "\n[T1] %6.0f rad/s, Rm 200 ohm: shunt draws %.4f A at 0.1 A load, "
        "%.4f A at 0.5 A load "
        "(omega*psi_f/Rm = %.4f A)\n",
        static_cast<double>(omega), static_cast<double>(drawn_light),
        static_cast<double>(drawn_heavy), static_cast<double>(predicted));

    // 1. The rig really does hold the torque current -- otherwise the row
    //    measures the operating point drifting, not the effect.
    // 1. 台架确实握住了转矩电流——否则这一行量的是工作点在漂，不是效应。
    // 0.5 % is the hold tolerance: the command is a feedforward solved on the
    // target state, so a small residual is expected and is not the effect.
    // 0.5 %
    // 是握住容差：指令是在目标状态上解出的前馈，留一点残差是预期的，它不是被测效应。
    EXPECT_NEAR(0.1F, light.magnetizing_iq_a, 5.0e-3F);
    EXPECT_NEAR(0.5F, heavy.magnetizing_iq_a, 5.0e-3F);
    // 2. The diverted current matches `omega*psi_f/Rm`: the shunt is the model
    //    it claims to be, not a fudge factor.
    // 2. 分走的电流与 `ω·ψf/Rm`
    // 相符：并联支路就是它自称的那个模型，不是凑出来的系数。
    EXPECT_NEAR(predicted, drawn_light, 0.05F * predicted + 1.0e-4F);
    // 3. Load independent: five times the load, the same diverted current.
    // 3. 与负载无关：五倍负载，分走的电流不变。
    EXPECT_NEAR(drawn_light, drawn_heavy, 0.05F * predicted + 1.0e-4F);
  }

  // At the top of the speed range with a light load the shunt takes most of the
  // current, so at that operating point an iron-loss-free model is not
  // imprecise, it is wrong about which current is flowing.
  // 在转速上限的轻载点，并联支路拿走了大部分电流，那个工作点上没有铁损的模型不是不够准，
  // 而是对"到底在流哪个电流"说错了。
  Effects iron{};
  iron.rm_ohm = 200.0F;
  const Outcome light = measure(kHighSpeedRad, 0.1F, iron);
  const float ratio =
      (light.sensed_iq_a - light.magnetizing_iq_a) / light.magnetizing_iq_a;
  std::printf(
      "[T1] at %.0f rad/s and 0.1 A load the shunt is %.0f %% of the torque "
      "current, and it does not scale with load\n",
      static_cast<double>(kHighSpeedRad), static_cast<double>(100.0F * ratio));
  EXPECT_GT(ratio, 0.5F);
}

} // namespace foc
