// S19 - the discrete inverter model, and the checks that decide whether it is
// worth its cost. S19 - 离散逆变器模型，以及决定它值不值这个代价的那些检验。
//
// The model is only useful if it can be *falsified*, so every property it is
// supposed to have is checked against something computed independently: an
// algebraic identity (V1), a formula derived from the physics (V2), the bench
// it is meant to replace (V5), and the analytic fade it is meant to explain
// (V6).
// 模型只有能被证伪才有用，所以它该有的每条性质都对着一个独立算出来的东西检查：一条代数恒等式（V1）、
// 一条从物理推出来的公式（V2）、它要替代的那台台架（V5），以及它要解释的那条解析淡出（V6）。

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <initializer_list>

#include <gtest/gtest.h>

#include <foc/core/svpwm.hpp>

#include "sil_bench.hpp"
#include "sil_discrete_inverter.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

constexpr float kVBus = 24.0F;
constexpr float kDtCtrl = 5.0e-5F; // 20 kHz control and switching
constexpr float kFSw = 20000.0F;
constexpr float kTDead = 1.0e-6F; // 2 % of the switching period

auto sgn(float x) -> float {
  return static_cast<float>((x > 0.0F) - (x < 0.0F));
}

/// [en] Mean alpha-beta voltage over one control period, with a frozen current.
/// [zh] 一个控制周期的平均 αβ 电压，电流冻结。
auto mean_voltage(sil::DiscreteInverter &inverter,
                  const core::ModulatorInput &request, float dt)
    -> core::Ab<float> {
  const core::SvpwmResult modulated = core::Svpwm::modulate(request);
  const int steps = inverter.begin_period(modulated.duty, request.v_bus, dt);
  float alpha = 0.0F;
  float beta = 0.0F;
  for (int k = 0; k < steps; ++k) {
    const core::Ab<float> v = inverter.step(request.i_uvw);
    alpha += static_cast<float>(v.alpha);
    beta += static_cast<float>(v.beta);
  }
  const float n = static_cast<float>(steps);
  return core::Ab<float>{alpha / n, beta / n};
}

struct RippleResult final {
  float mean_u;
  float peak_to_peak;
  float at_boundary;
  float at_mid;
  float mid_minus_mean;
};

/// [en] Locked rotor, fixed dq voltage, settled; then one period traced
/// sub-step by sub-step.
///      [zh] 锁转子、固定 dq 电压、先跑稳；再逐子步描一个周期。
auto ripple(float v_d, float t_dead, int sub_per_switch) -> RippleResult {
  sil::PmsmPlant plant(sil::nominal_plant_params());
  plant.reset(sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
  const core::ModulatorInput request{core::Ab<float>{v_d, 0.0F}, kVBus,
                                     core::Uvw<float>{}};
  const core::SvpwmResult modulated = core::Svpwm::modulate(request);
  sil::DiscreteInverter inverter(
      sil::InverterConfig{kFSw, t_dead, 0.0F, sub_per_switch});

  const int steps = inverter.begin_period(modulated.duty, kVBus, kDtCtrl);
  const float h = inverter.step_length_s();

  // L/R here is 0.8 ms, so 20 ms is long enough for the DC value to be the DC
  // value. Getting this wrong is how a ripple measurement turns into a
  // transient measurement. 这里的 L/R 是 0.8 ms，所以 20 ms
  // 足够让直流分量成为直流分量。这一步做错，纹波测量就变成了暂态测量。
  for (int period = 0; period < 400; ++period) {
    static_cast<void>(inverter.begin_period(modulated.duty, kVBus, kDtCtrl));
    for (int k = 0; k < steps; ++k) {
      plant.set_voltage_dq(
          core::park(inverter.step(plant.current_uvw()), 0.0F));
      plant.advance(h);
    }
  }

  RippleResult result{0.0F, 0.0F, plant.current_uvw().u, 0.0F, 0.0F};
  static_cast<void>(inverter.begin_period(modulated.duty, kVBus, kDtCtrl));
  float sum = 0.0F;
  float low = 1.0e9F;
  float high = -1.0e9F;
  for (int k = 0; k < steps; ++k) {
    plant.set_voltage_dq(core::park(inverter.step(plant.current_uvw()), 0.0F));
    plant.advance(h);
    const float i_u = plant.current_uvw().u;
    sum += i_u;
    low = std::fmin(low, i_u);
    high = std::fmax(high, i_u);
    if (k == (steps / 2)) {
      result.at_mid = i_u;
    }
  }
  result.mean_u = sum / static_cast<float>(steps);
  result.peak_to_peak = high - low;
  result.mid_minus_mean = result.at_mid - result.mean_u;
  return result;
}

struct LowCurrentPoint final {
  float mean_u;       ///< settled phase-u current
  float mean_voltage; ///< mean alpha voltage over the same period
  float ripple;
};

/// [en] Locked rotor at a small dq voltage: settle, then measure the current
/// and the mean voltage together.
///      [zh] 小 dq 电压下的锁转子：跑稳，然后在同一周期内一起量电流与平均电压。
auto low_current_point(float v_d) -> LowCurrentPoint {
  sil::PmsmPlant plant(sil::nominal_plant_params());
  plant.reset(sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
  const core::ModulatorInput request{core::Ab<float>{v_d, 0.0F}, kVBus,
                                     core::Uvw<float>{}};
  const core::SvpwmResult modulated = core::Svpwm::modulate(request);
  sil::DiscreteInverter inverter(sil::InverterConfig{kFSw, kTDead, 0.0F, 400});
  const int steps = inverter.begin_period(modulated.duty, kVBus, kDtCtrl);
  const float h = inverter.step_length_s();

  for (int period = 0; period < 600; ++period) {
    static_cast<void>(inverter.begin_period(modulated.duty, kVBus, kDtCtrl));
    for (int k = 0; k < steps; ++k) {
      plant.set_voltage_dq(
          core::park(inverter.step(plant.current_uvw()), 0.0F));
      plant.advance(h);
    }
  }

  static_cast<void>(inverter.begin_period(modulated.duty, kVBus, kDtCtrl));
  float sum_i = 0.0F;
  float sum_v = 0.0F;
  float low = 1.0e9F;
  float high = -1.0e9F;
  for (int k = 0; k < steps; ++k) {
    const core::Ab<float> v = inverter.step(plant.current_uvw());
    sum_v += static_cast<float>(v.alpha);
    plant.set_voltage_dq(core::park(v, 0.0F));
    plant.advance(h);
    const float i_u = plant.current_uvw().u;
    sum_i += i_u;
    low = std::fmin(low, i_u);
    high = std::fmax(high, i_u);
  }
  const float n = static_cast<float>(steps);
  return LowCurrentPoint{sum_i / n, sum_v / n, high - low};
}

} // namespace

TEST(SilInverter, V1_TheDutyReachesTheMachineUnchangedWithoutDeadTime) {
  // The algebraic identity the whole model rests on: `clarke(pole voltages)` is
  // the commanded vector, because the neutral point floats and SVPWM's midpoint
  // injection is exactly the common mode that cancels. If this ever fails,
  // every other number in this file is meaningless.
  // 整个模型依赖的代数恒等式：`clarke(极电压)` 就是指令矢量——中性点浮地，而
  // SVPWM 的中点注入正是那个
  // 被消掉的共模。这条一旦不成立，本文件其余所有数字都没有意义。
  sil::DiscreteInverter inverter(sil::InverterConfig{kFSw, 0.0F, 0.0F, 40});
  float worst = 0.0F;
  for (const float v_alpha : {0.5F, 3.0F, 7.0F, 12.0F}) {
    for (const float v_beta : {-4.0F, 0.0F, 0.9F, 4.0F}) {
      const core::ModulatorInput request{core::Ab<float>{v_alpha, v_beta},
                                         kVBus,
                                         core::Uvw<float>{0.3F, -0.7F, 0.2F}};
      // Stay out of the over-modulation corner, where the modulator itself
      // clips. 避开过调制区，那里是调制器自己削顶。
      if ((v_alpha * v_alpha) + (v_beta * v_beta) >
          (0.9F * kVBus * 0.577F) * (0.9F * kVBus * 0.577F)) {
        continue;
      }
      const core::Ab<float> mean = mean_voltage(inverter, request, kDtCtrl);
      worst = std::fmax(worst, std::fabs(static_cast<float>(
                                   mean.alpha - request.v_ab.alpha)));
      worst = std::fmax(
          worst, std::fabs(static_cast<float>(mean.beta - request.v_ab.beta)));
    }
  }
  std::printf("\n[S19] V1 duty->pole->clarke worst error %.2e V\n",
              static_cast<double>(worst));
  EXPECT_LT(worst, 1.0e-4F);
}

TEST(SilInverter, V2_TheDeadTimeVoltSecondsFallOutOfTheSwitching) {
  // The claim that makes this model worth building: the dead-time error is a
  // *consequence* of "turn-off is immediate, turn-on is delayed, the conducting
  // device is chosen by the current", not an input. So the number the model
  // produces has to match `-(t_dead/T)*v_bus*sign(i)`, which the model never
  // sees.
  // 让这个模型值得被写出来的那句话：死区误差是"关断立即、开通延迟、导通器件由电流决定"的**推论**，
  // 不是输入。所以模型给出的数必须等于
  // `−(t_dead/T)·v_bus·sign(i)`——而模型从来没见过这个式子。
  float loss_at_one_us = 0.0F;
  std::printf("[S19] V2 dead-time law\n%10s %8s %12s %12s %10s\n", "t_dead_us",
              "i_sign", "measured", "predicted", "err/vd %");
  for (const float t_dead_us : {0.5F, 1.0F, 2.0F}) {
    for (const float i_sign : {1.0F, -1.0F}) {
      sil::DiscreteInverter inverter(
          sil::InverterConfig{kFSw, t_dead_us * 1.0e-6F, 0.0F, 200});
      const core::Uvw<float> i_uvw{0.7F * i_sign, -0.7F * i_sign,
                                   0.7F * i_sign};
      const core::ModulatorInput request{core::Ab<float>{3.0F, 0.9F}, kVBus,
                                         i_uvw};
      const core::Ab<float> mean = mean_voltage(inverter, request, kDtCtrl);

      const float v_dead = inverter.deadtime_voltage(kVBus);
      const core::Ab<float> predicted = core::clarke(
          core::Uvw<float>{-v_dead * sgn(i_uvw.u), -v_dead * sgn(i_uvw.v),
                           -v_dead * sgn(i_uvw.w)});
      const float measured =
          static_cast<float>(mean.alpha - request.v_ab.alpha);
      const float error_pct =
          std::fabs(measured - static_cast<float>(predicted.alpha)) / v_dead *
          100.0F;
      std::printf(
          "%10.1f %8.0f %12.5f %12.5f %10.3f\n", static_cast<double>(t_dead_us),
          static_cast<double>(i_sign), static_cast<double>(measured),
          static_cast<double>(predicted.alpha), static_cast<double>(error_pct));

      // Within 1 % of the volt-seconds the dead time removes: the *mean* is
      // exact (windows are integrated as fractions), so anything larger would
      // be a bookkeeping bug, not resolution. 与死区拿走的伏秒相差 1%
      // 以内：**均值**是精确的（窗口按比例积分），所以更大的误差是记账错误，
      // 不是分辨率不足。
      EXPECT_LT(error_pct, 1.0F);
      // Same quantity, same sign: the model loses exactly the volt-seconds the
      // prediction says, and the direction of the error vector follows the
      // current.
      // 同一个量、同一个符号：模型丢掉的正是预测说的那些伏秒，误差矢量的方向跟随电流。
      EXPECT_NEAR(measured, static_cast<float>(predicted.alpha),
                  0.002F * v_dead);
      EXPECT_LT(measured * i_sign, 0.0F)
          << "a positive current must lose voltage, a negative one gain it";
      if (t_dead_us == 1.0F) {
        loss_at_one_us = -measured * i_sign;
      }
    }
  }

  // Linear in the dead time: 2 us has to cost exactly twice what 1 us costs.
  // 与死时间成正比：2 us 的代价必须正好是 1 us 的两倍。
  sil::DiscreteInverter doubled(sil::InverterConfig{kFSw, 2.0e-6F, 0.0F, 200});
  const core::Uvw<float> i_uvw{0.7F, -0.7F, 0.7F};
  const core::ModulatorInput request{core::Ab<float>{3.0F, 0.9F}, kVBus, i_uvw};
  const core::Ab<float> mean_doubled = mean_voltage(doubled, request, kDtCtrl);
  const float loss_doubled =
      static_cast<float>(request.v_ab.alpha - mean_doubled.alpha);
  // `loss_at_one_us` is "voltage lost by a positive current" too, so the ratio
  // is a plain number. `loss_at_one_us`
  // 同样是"正电流丢掉的电压"，所以比值就是一个普通数。
  std::printf("[S19]    linearity: 1 us %.5f V, 2 us %.5f V (ratio %.4f)\n",
              static_cast<double>(loss_at_one_us),
              static_cast<double>(loss_doubled),
              static_cast<double>(loss_doubled / loss_at_one_us));
  EXPECT_NEAR(2.0F, loss_doubled / loss_at_one_us, 0.01F);
}

TEST(SilInverter, V3_TheRippleIsThereAndTheSampleInstantSeesIt) {
  // Two things at once: the current really ripples at the switching frequency
  // (the mean-value plant has no ripple at all), and the mean sits at the
  // period boundary and at the middle -- which is *why* "sample at the carrier
  // peak" works, and is not something the model was told.
  // 两件事一起：电流真的在开关频率上纹波（平均值 plant
  // 完全没有纹波），而均值同时落在周期边界和中点
  // ——这正是"在载波峰取样"能成立的原因，而且没有人告诉过模型这件事。
  const RippleResult no_dead = ripple(3.0F, 0.0F, 400);
  const RippleResult with_dead = ripple(3.0F, kTDead, 400);

  std::printf("[S19] V3 locked rotor: ripple %.5f A, boundary %.5f, middle "
              "%.5f, mean %.5f (no dead time)\n",
              static_cast<double>(no_dead.peak_to_peak),
              static_cast<double>(no_dead.at_boundary),
              static_cast<double>(no_dead.at_mid),
              static_cast<double>(no_dead.mean_u));
  std::printf(
      "[S19]    with 1 us dead time: mean %.5f (v_d/Rs would be %.5f)\n",
      static_cast<double>(with_dead.mean_u),
      static_cast<double>(3.0F / sil::nominal_spec().rs));

  // The current really does move within a period.
  EXPECT_GT(no_dead.peak_to_peak, 0.05F);
  // And the DC value is what Ohm's law says it is, not something else.
  EXPECT_NEAR(3.0F / sil::nominal_spec().rs, no_dead.mean_u, 0.01F);

  // The sample instants: both land within 2 % of the ripple of the true mean,
  // so a controller that samples there is not reading a different machine.
  // 两个取样时刻都落在真实均值的 2%
  // 纹波以内，所以在那里取样的控制器读的不是另一台机器。
  EXPECT_LT(std::fabs(no_dead.at_boundary - no_dead.mean_u),
            0.02F * no_dead.peak_to_peak);
  EXPECT_LT(std::fabs(no_dead.mid_minus_mean), 0.02F * no_dead.peak_to_peak);

  // Dead time lowers the current: 3 V commanded, the phases lose
  // clarke(-vd*sign(i)) which is 0.64 V on the d axis, so the settled current
  // is (3 - 0.64)/Rs. 死区把电流压低：指令 3 V，三相丢掉 clarke(−vd·sign(i)) 在
  // d 轴上的 0.64 V，于是稳态电流是 (3 − 0.64)/Rs。
  const float expected = (3.0F - 0.64F) / sil::nominal_spec().rs;
  EXPECT_NEAR(expected, with_dead.mean_u, 0.03F);
}

TEST(SilInverter, V4_SubStepsChangeTheRippleNotTheMean) {
  // The property that lets CI afford this model: the sub-step count is a
  // resolution knob for the *ripple* only, because the mean over a sub-step is
  // integrated exactly. If the mean moved with the sub-step count, every number
  // in the repository would depend on a fidelity setting. 这条性质才让 CI
  // 付得起这个模型：子步数只是**纹波**的分辨率旋钮，因为子步内的均值是精确积分的。
  // 若均值随子步数变，仓库里每个数字都会依赖一个保真度设置。
  const RippleResult coarse = ripple(3.0F, kTDead, 40);
  const RippleResult fine = ripple(3.0F, kTDead, 400);
  std::printf(
      "[S19] V4 sub 40: mean %.6f ripple %.5f | sub 400: mean %.6f ripple %.5f "
      "| mean drift %.3f %%, "
      "ripple drift %.1f %%\n",
      static_cast<double>(coarse.mean_u),
      static_cast<double>(coarse.peak_to_peak),
      static_cast<double>(fine.mean_u), static_cast<double>(fine.peak_to_peak),
      static_cast<double>(std::fabs(coarse.mean_u - fine.mean_u) / fine.mean_u *
                          100.0F),
      static_cast<double>(std::fabs(coarse.peak_to_peak - fine.peak_to_peak) /
                          fine.peak_to_peak * 100.0F));

  EXPECT_LT(std::fabs(coarse.mean_u - fine.mean_u),
            0.001F * std::fabs(fine.mean_u));
  EXPECT_LT(std::fabs(coarse.peak_to_peak - fine.peak_to_peak),
            0.05F * fine.peak_to_peak);
}

TEST(SilInverter, V5_WithoutDeadTimeItReproducesTheMeanValueBench) {
  // Degenerate equivalence: with the dead time and the conduction drop switched
  // off, attaching the model must land on the bench it replaces. Without this,
  // a difference in the closed-loop numbers could not be attributed to
  // switching at all.
  // 退化等价：死区与导通压降都关掉时，挂上模型必须落回它要替代的那台台架。少了这一条，闭环数字上的
  // 差异就无法归因于开关过程。
  auto run = [](bool switching) -> float {
    foc::sil::Bench bench(foc::sil::standard_bench_config());
    sil::DiscreteInverter inverter(sil::InverterConfig{kFSw, 0.0F, 0.0F, 50});
    if (switching) {
      bench.hal().set_inverter(&inverter);
    }
    static_cast<void>(bench.init_and_start());
    bench.run(2.0e-3F);
    for (int i = 0; i < 1000; ++i) {
      bench.motor().set_speed(100.0F);
      bench.tick_once();
    }
    return bench.motor().telemetry().speed_mech_rad_s;
  };
  const float mean_value = run(false);
  const float switched = run(true);
  const float relative =
      std::fabs(switched - mean_value) / std::fabs(mean_value);

  std::printf("[S19] V5 speed after 50 ms: mean-value %.5f rad/s, switched "
              "%.5f rad/s (rel diff %.2e)\n",
              static_cast<double>(mean_value), static_cast<double>(switched),
              static_cast<double>(relative));
  EXPECT_LT(relative, 0.02F);
}

TEST(SilInverter,
     V6_TheModelGrowsTheLowCurrentFadeAndItIsSteeperThanTheFormula) {
  // A2 derived `loss = dV*clamp(i/I_ripple)` by averaging the rail occupancy
  // over a whole period. The switching model says only the *dead windows*
  // matter, which is 4 % of the period, so the real fade is steeper. Measuring
  // it is the point: the formula is a design aid, the model is the referee. A2
  // 通过在整个周期上平均母线占用，推出 `loss =
  // dV·clamp(i/I_ripple)`。开关模型说只有**死区窗口**算数， 而那是周期的
  // 4%，所以真实的淡出更陡。量出这件事就是本检验的意义：公式是设计辅助，模型是裁判。
  std::printf("[S19] V6 dead-time loss vs current (ripple ~0.006-0.02 A)\n%10s "
              "%12s %12s %12s %10s\n",
              "v_cmd", "mean_i", "loss", "sign_loss", "ratio");
  struct Item final {
    float v_d;
    float min_ratio; // allowed ratio window at that command
    float max_ratio;
  };
  // Large current: the sign() law must hold. Small current: it must have faded
  // well below it. 大电流：`sign()` 律必须成立。小电流：必须已经远低于它。
  for (const Item item : {Item{3.0F, 0.99F, 1.01F}, Item{1.0F, 0.99F, 1.01F},
                          Item{0.2F, 0.0F, 0.35F}}) {
    // The loss and the current have to come from the *same* period: the loss
    // depends on the current, so measuring them separately would be measuring
    // two different operating points.
    // 损失与电流必须取自**同一个**周期：损失依赖电流，分开量等于量了两个不同的工作点。
    const LowCurrentPoint settled = low_current_point(item.v_d);
    sil::DiscreteInverter inverter(
        sil::InverterConfig{kFSw, kTDead, 0.0F, 400});
    const core::ModulatorInput request{core::Ab<float>{item.v_d, 0.0F}, kVBus,
                                       core::Uvw<float>{}};
    const float v_dead = inverter.deadtime_voltage(kVBus);
    const float i_mean = settled.mean_u;
    const float mean_alpha = settled.mean_voltage;
    const core::Uvw<float> i_pattern{i_mean, -0.5F * i_mean, -0.5F * i_mean};
    const core::Ab<float> sign_loss = core::clarke(
        core::Uvw<float>{-v_dead * sgn(i_pattern.u), -v_dead * sgn(i_pattern.v),
                         -v_dead * sgn(i_pattern.w)});
    const float measured =
        std::fabs(mean_alpha - static_cast<float>(request.v_ab.alpha));
    const float predicted = std::fabs(static_cast<float>(sign_loss.alpha));
    const float ratio = measured / predicted;
    std::printf("%10.3f %12.5f %12.5f %12.5f %10.3f\n",
                static_cast<double>(item.v_d), static_cast<double>(i_mean),
                static_cast<double>(measured), static_cast<double>(predicted),
                static_cast<double>(ratio));
    EXPECT_GT(ratio, item.min_ratio);
    EXPECT_LT(ratio, item.max_ratio);
  }
}

TEST(SilInverter, V7_DeterministicClampedAndRefusesAMisTimedRequest) {
  // A simulator that is not reproducible turns every failure into a story. And
  // a mis-timed request has to be refused: silently rounding a control period
  // to a different switching period would put a frequency error into every
  // number downstream.
  // 不可复现的仿真会把每次失败变成一段故事。而错配时序的请求必须被拒绝：把控制周期悄悄凑成另一个开关
  // 周期，等于给下游每个数字都灌进一个频率误差。
  const RippleResult first = ripple(3.0F, kTDead, 50);
  const RippleResult second = ripple(3.0F, kTDead, 50);
  EXPECT_FLOAT_EQ(first.mean_u, second.mean_u);
  EXPECT_FLOAT_EQ(first.peak_to_peak, second.peak_to_peak);

  sil::DiscreteInverter inverter(sil::InverterConfig{kFSw, kTDead, 0.0F, 40});
  // 20 kHz switching, 30 kHz "control": not a whole number of switching
  // periods. 20 kHz 开关、30 kHz"控制"：不是开关周期的整数倍。
  EXPECT_EQ(0, inverter.begin_period(core::Uvw<float>{0.5F, 0.5F, 0.5F}, kVBus,
                                     1.0F / 30000.0F));
  // Five switching periods is fine.
  // 五个开关周期可以。
  EXPECT_EQ(200, inverter.begin_period(core::Uvw<float>{0.5F, 0.5F, 0.5F},
                                       kVBus, 5.0F / kFSw));

  // Out-of-range duties are clamped and reported, rather than producing a
  // negative on-time. 越界占空比被钳位并可见，而不是产生负导通时间。
  static_cast<void>(inverter.begin_period(core::Uvw<float>{-0.2F, 0.5F, 1.3F},
                                          kVBus, kDtCtrl));
  EXPECT_TRUE(inverter.clamped());
  EXPECT_FLOAT_EQ(0.0F, inverter.duty().u);
  EXPECT_FLOAT_EQ(1.0F, inverter.duty().w);
}
