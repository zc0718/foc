// S13 - inverter dead time. This scenario exists for one reason: the PRD's U1
// said "dead-time distortion cannot be verified in SIL", and that was a
// statement about the *plant*. With a dead-time model the distortion becomes a
// number CI can print, which is what turns the known gap into a measured one.
// S13 - 逆变器死区。本场景只为一件事存在：PRD 的 U1 说"死区畸变在 SIL
// 里无法验证"，而那是一句关于
// **被控对象**的话。加上死区模型之后，畸变变成 CI
// 能打印出来的数字——这才把"已知缺口"变成"已测缺口"。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/deadtime.hpp>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"

namespace {

constexpr float kTwoPi = 6.28318530718F;
constexpr float kSineHz = 1.0F;
constexpr float kAmplitudeRadS =
    2.0F; // the amplitude where the distortion is largest, see below
constexpr float kRunSeconds = 2.0F;
constexpr float kSettleSeconds = 0.25F;
constexpr float kPwmHz = 20000.0F;
constexpr float kDeadtimeS = 1.0e-6F;

/// The voltage a dead time of `kDeadtimeS` injects per phase at the given bus
/// voltage:
/// `(t_dead / T_pwm) * v_bus`, i.e. the missing volt-seconds spread over the
/// switching period. 给定母线电压下，`kDeadtimeS` 死区每相注入的电压：`(t_dead
/// / T_pwm) * v_bus`。
auto deadtime_voltage(float v_bus) -> float {
  return (kDeadtimeS * kPwmHz) * v_bus;
}

struct SineResult final {
  float worst_error_rad_s;
  float measured_amplitude_rad_s;
};

/// S3's 1 Hz tracking scenario at a low amplitude, with the dead time switched
/// on or off. S3 的 1 Hz 跟踪场景（低幅值），可开关死区模型。
auto run_low_amplitude_sine(float v_dead) -> SineResult {
  foc::sil::Bench bench(foc::sil::standard_bench_config());
  bench.hal().set_deadtime_voltage(v_dead);
  bench.motor().set_speed(0.0F);
  static_cast<void>(bench.init_and_start());
  bench.run(2e-3F);

  const int steps =
      static_cast<int>((kRunSeconds / bench.config().dt_ctrl_s) + 0.5F);
  const int skip =
      static_cast<int>((kSettleSeconds / bench.config().dt_ctrl_s) + 0.5F);

  std::vector<foc::sil::Sample> trace;
  trace.reserve(static_cast<std::size_t>(steps - skip));

  SineResult result{0.0F, 0.0F};
  for (int i = 0; i < steps; ++i) {
    const float t = bench.elapsed_s();
    const float ref = kAmplitudeRadS * std::sin(kTwoPi * kSineHz * t);
    bench.motor().set_speed(ref);
    bench.tick_once();
    if (i >= skip) {
      trace.push_back(bench.sample());
      const float error = std::fabs(trace.back().speed - ref);
      result.worst_error_rad_s =
          (error > result.worst_error_rad_s) ? error : result.worst_error_rad_s;
    }
  }

  const std::vector<float> speed =
      foc::sil::series_of(trace, &foc::sil::Sample::speed);
  result.measured_amplitude_rad_s =
      0.5F * (foc::sil::max_of(speed) - foc::sil::min_of(speed));
  return result;
}

using Compensated = foc::core::DeadTimeCompensator<foc::core::Svpwm>;
using MotorWithCompensation =
    foc::motor::Motor<foc::core::BackEmfObserver, Compensated, foc::core::Pi,
                      foc::sil::PlantHal>;

/// The same low-amplitude sine, with a dead-time compensator in front of the
/// modulator. 同一个低幅值正弦，但调制器前面有死区补偿。
auto run_compensated_sine(float v_dead, float transition_a = 0.0F,
                          float ripple_a = 0.0F) -> SineResult {
  const foc::motor::MotorConfig mc = foc::sil::nominal_motor_config();
  foc::sil::PmsmPlant plant(
      foc::sil::plant_params_from(foc::sil::nominal_spec()));
  foc::sil::PlantHal hal(plant, 24.0F);
  hal.set_deadtime_voltage(v_dead);
  hal.set_deadtime_transition(transition_a);

  const foc::sil::BenchConfig cfg = foc::sil::standard_bench_config();
  const typename MotorWithCompensation::Policies policies{
      foc::core::BackEmfObserver({mc.rs, mc.ld}),
      Compensated{foc::core::DeadTimeCompensator<foc::core::Svpwm>::Config{
          v_dead, 0.02F, ripple_a}},
      foc::core::Pi{
          foc::current_pi_config(mc, cfg.omega_current_rad_s, cfg.v_bus_v)},
      foc::core::Pi{
          foc::current_pi_config(mc, cfg.omega_current_rad_s, cfg.v_bus_v)},
      foc::core::Pi{foc::sil::speed_pi_config(mc, cfg.omega_speed_rad_s)}};

  MotorWithCompensation motor(mc, policies, hal);
  static_cast<void>(motor.init());
  static_cast<void>(motor.start());

  const int steps = static_cast<int>((kRunSeconds / cfg.dt_ctrl_s) + 0.5F);
  const int skip = static_cast<int>((kSettleSeconds / cfg.dt_ctrl_s) + 0.5F);

  SineResult result{0.0F, 0.0F};
  std::vector<float> speeds;
  speeds.reserve(static_cast<std::size_t>(steps - skip));
  for (int i = 0; i < steps; ++i) {
    const float t = static_cast<float>(i) * cfg.dt_ctrl_s;
    const float ref = kAmplitudeRadS * std::sin(kTwoPi * kSineHz * t);
    motor.set_speed(ref);
    static_cast<void>(motor.tick(cfg.dt_ctrl_s));
    hal.advance(cfg.dt_ctrl_s, cfg.substeps);
    if (i >= skip) {
      const float speed = motor.telemetry().speed_mech_rad_s;
      speeds.push_back(speed);
      const float error = std::fabs(speed - ref);
      result.worst_error_rad_s =
          (error > result.worst_error_rad_s) ? error : result.worst_error_rad_s;
    }
  }
  const float high = *std::max_element(speeds.begin(), speeds.end());
  const float low = *std::min_element(speeds.begin(), speeds.end());
  result.measured_amplitude_rad_s = 0.5F * (high - low);
  return result;
}

} // namespace

TEST(SilS13Deadtime, TheModelInjectsTheVoltSecondsTheDeadTimeTakes) {
  // 1 us at 20 kHz is 2 % of the switching period; at a 24 V bus that is 0.48 V
  // per phase, which is the quantity every dead-time compensation scheme has to
  // reconstruct. 20 kHz 下 1 us 占开关周期的 2%；24 V 母线上就是每相 0.48
  // V——这正是所有死区补偿方案必须重建的量。
  EXPECT_NEAR(0.48F, deadtime_voltage(24.0F), 1e-3F);

  foc::sil::PmsmPlant plant(foc::sil::nominal_plant_params());
  foc::sil::PlantHal hal(plant, 24.0F);
  EXPECT_FLOAT_EQ(0.0F, hal.deadtime_voltage());
  hal.set_deadtime_voltage(deadtime_voltage(24.0F));
  EXPECT_NEAR(0.48F, hal.deadtime_voltage(), 1e-5F);
  // A negative setting is not "reverse dead time": it clamps to off.
  // 负的设定不是"反向死区"：它会被钳到关闭。
  hal.set_deadtime_voltage(-1.0F);
  EXPECT_FLOAT_EQ(0.0F, hal.deadtime_voltage());
}

TEST(SilS13Deadtime, LowCurrentTrackingDegradesOnceDeadTimeIsModelled) {
  // The distortion is *not* monotonic in amplitude, and the sweep below is why
  // this test uses 2 rad/s rather than "as slow as possible" -- measured
  // worst-error ratio with 1 us of dead time:
  //   1 rad/s -> x1.00, 2 rad/s -> x1.48, 5 rad/s -> x1.13, 10 rad/s -> x1.03
  // At large amplitudes the resistive and back-EMF voltages dwarf the 0.48 V
  // the dead time removes; at the very lowest amplitudes this model's `sign(0)
  // = 0` makes the zero-crossing neighbourhood *better* than hardware --
  // exactly the idealisation U1 warns about.
  // 畸变对幅值**不是单调的**，所以本用例取 2 rad/s 而不是"越慢越好"——1 us
  // 死区下实测最差 误差比值：1 rad/s → x1.00，2 rad/s → x1.48，5 rad/s →
  // x1.13，10 rad/s → x1.03。 幅值大时电阻压降与反电动势远大于死区拿走的 0.48
  // V；幅值极低时本模型的 `sign(0) = 0` 让零电流附近比真实硬件**更好**——这正是
  // U1 所警告的那层理想化。
  const SineResult ideal = run_low_amplitude_sine(0.0F);
  const SineResult with_deadtime =
      run_low_amplitude_sine(deadtime_voltage(24.0F));

  // The ideal case tracks tightly -- this is the control the library already
  // promises. 理想情况跟踪得很紧——这是库本来就承诺的控制质量。
  EXPECT_LE(ideal.worst_error_rad_s, 0.05F * kAmplitudeRadS)
      << "ideal worst error " << ideal.worst_error_rad_s;

  // The distortion is real and measurable: the worst error grows, while the
  // loop still tracks. 畸变是真实的、可测的：最差误差变大，而环路仍然在跟踪。
  EXPECT_GT(with_deadtime.worst_error_rad_s, 1.3F * ideal.worst_error_rad_s)
      << "ideal " << ideal.worst_error_rad_s << " vs dead time "
      << with_deadtime.worst_error_rad_s;
  EXPECT_GE(with_deadtime.measured_amplitude_rad_s, 0.9F * kAmplitudeRadS)
      << "the loop must not collapse";

  // Recorded, not asserted: the ratio is the size of the gap U1 describes, and
  // it is what a dead-time compensation stage would have to remove.
  // 只记录不断言：这个比值就是 U1
  // 所描述的缺口大小，也是死区补偿环节需要消掉的东西。
  std::printf("[S13] worst error: ideal %.5f rad/s, with 1us dead time %.5f "
              "rad/s (x%.2f)\n",
              static_cast<double>(ideal.worst_error_rad_s),
              static_cast<double>(with_deadtime.worst_error_rad_s),
              static_cast<double>(with_deadtime.worst_error_rad_s /
                                  ideal.worst_error_rad_s));
}

TEST(SilS13Deadtime, CompensatingTheDeadTimeClosesTheGapItOpened) {
  // The point of the previous test: it measured a gap. This one shows the gap
  // is *closable* by a library stage rather than by hand-tuning, and by how
  // much. Same scenario, same plant, same dead time -- only the modulator
  // differs.
  // 上一条用例的意义在于它量出了一个缺口。这一条证明缺口可以**被库里的一个环节**关掉而不是靠手调，
  // 并量化关掉了多少。同样的场景、同样的 plant、同样的死区——只有调制器不同。
  const float v_dead = deadtime_voltage(24.0F);
  const SineResult ideal = run_low_amplitude_sine(0.0F);
  const SineResult uncompensated = run_low_amplitude_sine(v_dead);
  const SineResult compensated = run_compensated_sine(v_dead);

  // The uncompensated build really is worse (same assertion as above, kept so
  // this test fails for the same reason if the plant ever drifts).
  // 未补偿的构建确实更差（同上一条用例的断言，保留它是为了让本用例在 plant
  // 漂移时因同样理由失败）。
  EXPECT_GT(uncompensated.worst_error_rad_s, 1.3F * ideal.worst_error_rad_s);

  // And compensation brings it back within 15 % of the ideal -- i.e. the x1.48
  // gap is gone. 补偿把它带回理想值的 15% 以内——也就是那个 ×1.48 的缺口消失了。
  EXPECT_LT(compensated.worst_error_rad_s, 1.15F * ideal.worst_error_rad_s)
      << "ideal " << ideal.worst_error_rad_s << ", compensated "
      << compensated.worst_error_rad_s << ", uncompensated "
      << uncompensated.worst_error_rad_s;
  EXPECT_GE(compensated.measured_amplitude_rad_s, 0.95F * kAmplitudeRadS);

  std::printf("[S13] worst error: ideal %.5f, uncompensated %.5f (x%.2f), "
              "compensated %.5f (x%.2f)\n",
              static_cast<double>(ideal.worst_error_rad_s),
              static_cast<double>(uncompensated.worst_error_rad_s),
              static_cast<double>(uncompensated.worst_error_rad_s /
                                  ideal.worst_error_rad_s),
              static_cast<double>(compensated.worst_error_rad_s),
              static_cast<double>(compensated.worst_error_rad_s /
                                  ideal.worst_error_rad_s));
}

TEST(SilS13Deadtime, ASmoothZeroCrossingIsNotWhatSignCompensationAssumes) {
  // The compensator models the inverter as `v_dead * sign(i)`. A real inverter
  // clamps to a rail and approaches it smoothly, so near a zero crossing the
  // model over-corrects -- which is exactly the part of the remaining 14 % that
  // this library cannot remove. Stated as a measurement: turn the smoothness on
  // and the *compensated* result must get worse, because the compensation is
  // not modelling what is happening. 补偿器把逆变器建模成
  // `v_dead·sign(i)`。真实逆变器钳位到母线并平滑趋近，因此在过零附近模型会**过修
  // 正**——那正是本库消不掉的那 14%
  // 里的成因。用测量说明：打开平滑性，**补偿后**的结果必须变差，因为
  // 补偿没有在建模真实发生的事情。
  const float v_dead = deadtime_voltage(24.0F);
  std::printf("[S13] sweep of the dead-zone transition width (compensated):\n");
  float previous = 0.0F;
  for (const float width : {0.0F, 0.05F, 0.1F, 0.2F, 0.5F}) {
    const SineResult result = run_compensated_sine(v_dead, width);
    std::printf("[S13]   %4.2f A -> %.5f rad/s%s\n", static_cast<double>(width),
                static_cast<double>(result.worst_error_rad_s),
                (width == 0.0F) ? "  (the ideal step the model assumes)" : "");
    if (width > 0.0F) {
      EXPECT_GT(result.worst_error_rad_s, previous)
          << "smoother must be worse here, width " << width;
    }
    previous = result.worst_error_rad_s;
  }
}

TEST(SilS13Deadtime, TheRampFadesTheCorrectionWhereTheSignIsNotYetInformation) {
  // Fixes finding 1 from the other side. The sweep above showed that a `sign()`
  // correction is *worse than nothing* once the zero crossing is not an ideal
  // step -- because it commands a full +-0.48 V square wave at currents where
  // the inverter's own loss has already faded to a fraction of that. The ramp
  // `clamp(i / I_ripple)` is what the averaged dead-time error actually is (see
  // the derivation on `DeadTimeCompensator`), so it fades out exactly where the
  // step over-corrects.
  //
  // Two things are asserted, and they have to be asserted *together*: the ramp
  // never loses to doing nothing (which "do nothing" would also pass), and the
  // legacy column still reproduces the recorded numbers (otherwise the
  // comparison is against a moving target). 从另一侧修掉发现
  // 1。上面的扫描显示：只要过零不是理想阶跃，`sign()`
  // 修正就**比不补更差**——它在
  // 逆变器自身损耗已经淡到一小部分的电流处，仍命令了一个完整的 ±0.48 V
  // 方波。斜坡 `clamp(i/I_ripple)` 才是死区误差平均值本身（推导见
  // `DeadTimeCompensator`），所以它恰好在阶跃过修的地方淡出。
  // 两件事必须**一起**断言：斜坡从不输给不补偿（"什么都不做"也能通过这一条），以及旧的那一列仍复现
  // 记录值（否则就是拿一个会动的靶子做比较）。
  const float v_dead = deadtime_voltage(24.0F);
  const float ripple =
      foc::core::DeadTimeCompensator<foc::core::Svpwm>::predicted_ripple_a(
          24.0F, 1.0F / kPwmHz, foc::sil::nominal_spec().ld);
  const SineResult uncompensated = run_low_amplitude_sine(v_dead);

  std::printf("[S13] ripple %.3f A (v_bus*T/(8L)), uncompensated %.5f rad/s\n",
              static_cast<double>(ripple),
              static_cast<double>(uncompensated.worst_error_rad_s));
  std::printf("[S13] %6s %12s %12s\n", "width", "sign()", "ramp");

  // The recorded legacy column, so a change in the step path shows up as a
  // mismatch here. 记录的旧行为一列，阶跃路径若变化会在这里显示为不匹配。
  const float legacy[] = {0.05791F, 0.13498F, 0.14364F, 0.16482F, 0.17555F};
  int index = 0;
  for (const float width : {0.0F, 0.05F, 0.1F, 0.2F, 0.5F}) {
    const SineResult stepped = run_compensated_sine(v_dead, width);
    const SineResult ramped = run_compensated_sine(v_dead, width, ripple);
    std::printf("[S13] %6.2f %12.5f %12.5f\n", static_cast<double>(width),
                static_cast<double>(stepped.worst_error_rad_s),
                static_cast<double>(ramped.worst_error_rad_s));

    EXPECT_NEAR(legacy[index], stepped.worst_error_rad_s, 2.0e-4F)
        << "ripple 0 has to be bit-for-bit the old behaviour, width " << width;

    if (width > 0.0F) {
      // Where the step loses -- i.e. every inverter that really exists -- the
      // ramp must beat it *and* must not lose to doing nothing.
      // 在阶跃会输的地方——也就是每个真实存在的逆变器——斜坡必须胜过它，且不能输给不补偿。
      EXPECT_LT(ramped.worst_error_rad_s, 0.5F * stepped.worst_error_rad_s)
          << "width " << width;
      EXPECT_LE(ramped.worst_error_rad_s, uncompensated.worst_error_rad_s)
          << "width " << width;
    } else {
      // And the price, asserted instead of hidden. At width 0 the plant *is*
      // `sign()`, so the ramp under-corrects by construction and the smooth
      // shape cannot win there. Measured 0.08062 vs 0.07506 uncompensated and
      // 0.05791 for the step: a 7 % penalty on a model that no inverter
      // implements, paid for a 2.7x gain on the ones that do. A criterion
      // written before this was measured said "never worse than nothing, at
      // every width" -- that criterion was wrong, because it asks one shape to
      // be optimal against two different plants.
      // 以及代价，断言它而不是藏起来。width 0 时 plant **就是**
      // `sign()`，所以斜坡按构造欠补， 平滑形状在那里不可能赢。实测
      // 0.08062，不补偿 0.07506，阶跃 0.05791：在一个没有逆变器 实现的模型上付
      // 7%，换取在真实模型上 2.7 倍的收益。量之前写下的判据说"任何宽度都不比
      // 不补偿差"——那条判据是错的，它在要求同一个形状对两台不同的 plant
      // 都最优。
      EXPECT_GT(ramped.worst_error_rad_s, stepped.worst_error_rad_s);
      EXPECT_LT(ramped.worst_error_rad_s,
                1.1F * uncompensated.worst_error_rad_s);
    }
    ++index;
  }
}
