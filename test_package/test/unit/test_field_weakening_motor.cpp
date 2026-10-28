// P11 - field weakening as a product, not a solver. Two questions: does a drive
// built with the policy hold a speed past the base speed it is supposed to
// unlock, and is the observer's magnitude-correction law the right *shape* once
// the flux it is correcting is genuinely changed by the operating point?
// P11——弱磁作为产品而不是求解器。两个问题：装上该策略的驱动器是否守得住它承诺解锁的
// 那个基速以上的转速；以及一旦工作点真的改变了它要修正的那个磁链，观测器幅值修正律的
// **形状**还对不对。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/field_weakening.hpp>
#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/transforms.hpp>
#include <foc/motor/motor.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"

namespace foc {

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

/**
 * @brief [en] `v_max/v_bus` for SVPWM's linear range.
 * @brief [zh] SVPWM 线性区的 `v_max/v_bus`。
 */
constexpr float kOneOverSqrt3 = 0.5773502691896257F;

/**
 * @brief [en] The target: 10 % past the 2910 rad/s base speed at a 24 V bus.
 * @brief [zh] 目标：24 V 母线、2910 rad/s 基速之外 10 %。
 */
constexpr float kTargetRad = 3200.0F;
constexpr float kNormalisedTargetRad = 0.055F;
constexpr float kAngleCeilingRad = 0.15F;

/**
 * @brief [en] The criterion, written before it was measured.
 *
 *     Parked **at** the target speed and told to stay there, a drive with `FieldWeakeningReference` holds it (>= 0.97
 *     of target) and spends `i_d < -2 A` doing so; the same drive with a `d = 0` reference falls back below 0.95 of the
 *     target. The contrast is the result: "it held 3200" alone could mean the target was not past the base speed. The
 *     normalisation hypothesis: if the magnitude correction is normalised (`k*(1 - |psi_hat|/psi_target)` instead of
 *     `k*(psi_target - |psi_hat|)`), the field-weakened angle error drops to <= 0.055 rad. Otherwise the status quo
 *     stands and the number that refuted it is recorded.
 *
 * @brief [zh] 判据，写在测量之前。
 *
 *     1. **停在**目标转速上并被告知保持，装 `FieldWeakeningReference` 的驱动器守得住 （≥ 目标的 0.97）并为此花掉 `i_d <
 *     −2 A`；同样的驱动器换成 `d = 0` 参考会掉到目标的 0.95 以下。对比才是结论：只说"它守住了 3200"也可能意味着目标没超
 *     过基速。 2. 归一化假设：若把幅值修正归一化，弱磁区角误差降到 ≤ 0.055 rad；否则维持现状， 并记录下反驳它的那个数。
 *
 * @note [en] What hypothesis 2 actually is. Normalising by `psi_target` is the *same* law with the gain divided by
 *            `psi_target` -- a factor of 210 on this machine. It is not a new law; it is a 210x gain increase wearing a
 *            dimensionless hat, and it is testable with the knob that already exists.
 * @note [zh] 假设 2 到底是什么。按 `psi_target` 归一化就是**同一个律**把增益除以 `psi_target` ——在本机上是 210 倍。它不
 *            是新律，是一个戴了无量纲帽子的 210 倍增益，用现成的旋钮就能测。
 */
auto psi_f_of(const sil::MachineSpec &s) -> float {
  return s.kt / (1.5F * static_cast<float>(s.pole_pairs));
}

/**
 * @brief [en] The analytical base speed at 24 V: `omega*psi_f = v_max`.
 * @brief [zh] 24 V 下解析基速：`ω·ψf = v_max`。
 */
auto base_speed_rad(const sil::MachineSpec &s) -> float {
  return (kOneOverSqrt3 * 24.0F) / psi_f_of(s);
}

auto fw_config(const sil::MachineSpec &s)
    -> core::FieldWeakeningReference::Config {
  return core::FieldWeakeningReference::Config{s.rs, s.lq, psi_f_of(s),
                                               s.i_max_a, kOneOverSqrt3};
}

/**
 * @brief [en] Rig at one field-weakened operating point, held there by an exact feedforward, so the only thing that
 *             varies across a sweep is the observer.
 * @brief [zh] 某一个弱磁工作点上的台架，用精确前馈握在那里，所以扫描中唯一在变的是观测器。
 * @since 0.1.0
 */
class FwRig final {
public:
  FwRig(float omega_e, core::Dq<float> current) noexcept
      : plant_(params()), omega_e_(omega_e) {
    const sil::MachineSpec s = sil::nominal_spec();
    plant_.reset(sil::PlantState{
        0.0F, current.q, omega_e / static_cast<float>(s.pole_pairs), 0.0F});
    // Exact steady-state command for the requested current.
    // 为所请求电流给出的精确稳态指令。
    vd_ = static_cast<float>((s.rs * current.d) - (omega_e * s.lq * current.q));
    vq_ = static_cast<float>((s.rs * current.q) + (omega_e * s.ld * current.d) +
                             (omega_e * psi_f_of(s)));
  }

  FwRig(const FwRig &) = delete;
  FwRig &operator=(const FwRig &) = delete;
  FwRig(FwRig &&) = delete;
  FwRig &operator=(FwRig &&) = delete;

  /**
   * @brief [en] The rotor is pinned: this is a speed test, not a mechanics test.
   * @brief [zh] 转子被钉住：这是转速测试，不是力学测试。
   */
  [[nodiscard]] static auto params() noexcept -> sil::PlantParams {
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
    return current_;
  }

  void step() noexcept {
    const float theta = true_angle_e();
    command_ = core::inv_park(core::Dq<float>{vd_, vq_}, theta);
    current_ = core::clarke(plant_.current_uvw());
    plant_.set_voltage_dq(core::Dq<float>{vd_, vq_});
    for (int s = 0; s < 8; ++s) {
      plant_.advance(5.0e-5F / 8.0F);
    }
  }

private:
  sil::PmsmPlant plant_;
  float omega_e_;
  float vd_{0.0F};
  float vq_{0.0F};
  core::Ab<float> command_{0.0F, 0.0F};
  core::Ab<float> current_{0.0F, 0.0F};
};

auto wrap_pi(float angle) noexcept -> float {
  constexpr float kPi = 3.14159265358979F;
  constexpr float kTwoPi = 6.28318530717959F;
  while (angle <= -kPi) {
    angle += kTwoPi;
  }
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  return angle;
}

/**
 * @brief [en] Mean |angle error| for the observer at one correction gain. A non-finite answer is returned as-is,
 *             because "the estimate blew up" is a result and not something to be scrubbed out of the table.
 * @brief [zh] 一个修正增益下观测器的角误差绝对均值。非有限值原样返回，因为"估计炸了"是一个结果，不该被从表里抹掉。
 */
auto fw_angle_error(core::Dq<float> current, float gain_ratio) -> float {
  const sil::MachineSpec s = sil::nominal_spec();
  core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
      1.10F * s.rs, s.lq, psi_f_of(s), gain_ratio, 3.0F, 0.5F * psi_f_of(s)});
  observer.seed_flux(core::Ab<float>{psi_f_of(s), 0.0F});
  FwRig rig(kTargetRad, current);
  float sum = 0.0F;
  int count = 0;
  const float dt = 5.0e-5F;
  const int steps = static_cast<int>((0.4F / dt) + 0.5F);
  for (int k = 0; k < steps; ++k) {
    rig.step();
    const core::ObsState observed =
        observer.update(rig.command(), rig.current(), dt);
    if (k >= (steps / 2)) {
      const float error =
          std::fabs(wrap_pi(observed.theta_e - rig.true_angle_e()));
      sum = std::isfinite(error) ? (sum + error) : (sum + 1.0e6F);
      ++count;
    }
  }
  const float mean = sum / static_cast<float>(count);
  return (mean > 1.0e5F) ? std::nanf("") : mean;
}

} // namespace

/**
 * @brief [en] Closed-loop speed test through the real voltage limit.
 * @brief [zh] 穿过真实电压上限的闭环转速测试。
 * @note [en] The current loop is *perfect* on purpose: it is given the exact steady-state voltage for the reference it
 *            is asked for, and then the **vector is clamped to what SVPWM can deliver**. So the only thing that can
 *            stop the drive is the voltage budget, which is exactly the quantity field weakening exists to manage.
 *            Modelling a PI here would measure the PI's tuning instead.
 * @note [zh] 电流环故意做成**完美**的：它按给定算出精确稳态电压，然后把**矢量夹到 SVPWM 能 交付的上限**。于是能拦住驱动
 *            器的只剩电压预算，而那正是弱磁存在的理由。在这里建一个 PI， 量到的会是 PI 的整定。
 * @since 0.1.0
 */
struct SpeedRun final {
  float speed_rad_s;
  float id_a;
  float iq_a;
};

auto run_speed_loop(bool with_policy, float iq_command, float start_rad,
                    float seconds) -> SpeedRun {
  const sil::MachineSpec s = sil::nominal_spec();
  const float psi = psi_f_of(s);
  const float v_max = kOneOverSqrt3 * 24.0F;
  core::FieldWeakeningReference policy(fw_config(s));
  sil::PmsmPlant plant(sil::nominal_plant_params());
  plant.reset(sil::PlantState{
      0.0F, 0.0F, start_rad / static_cast<float>(s.pole_pairs), 0.0F});
  const float dt = 5.0e-5F;
  const int steps = static_cast<int>((seconds / dt) + 0.5F);
  for (int k = 0; k < steps; ++k) {
    const float omega_e =
        static_cast<float>(s.pole_pairs) * plant.omega_mech_rad_s();
    core::ReferenceInput in{};
    in.iq_command_a = iq_command;
    in.omega_e_rad_s = omega_e;
    in.v_bus_v = 24.0F;
    in.i_meas = plant.magnetizing_current_dq();
    const core::Dq<float> ref =
        with_policy ? policy.generate(in) : core::Dq<float>{0.0F, iq_command};
    // The exact steady-state voltage for that reference, then the clamp the
    // inverter actually has.
    // 该参考的精确稳态电压，再加逆变器真有的那个夹紧。
    float vd = static_cast<float>((s.rs * ref.d) - (omega_e * s.lq * ref.q));
    float vq = static_cast<float>((s.rs * ref.q) + (omega_e * s.ld * ref.d) +
                                  (omega_e * psi));
    const float mag = std::sqrt((vd * vd) + (vq * vq));
    if (mag > v_max) {
      const float scale = v_max / mag;
      vd = static_cast<float>(vd * scale);
      vq = static_cast<float>(vq * scale);
    }
    plant.set_voltage_dq(core::Dq<float>{vd, vq});
    for (int sub = 0; sub < 8; ++sub) {
      plant.advance(dt / 8.0F);
    }
  }
  const core::Dq<float> end = plant.magnetizing_current_dq();
  return SpeedRun{static_cast<float>(s.pole_pairs) * plant.omega_mech_rad_s(),
                  end.d, end.q};
}

TEST(FieldWeakeningMotor, ThePolicyEngagesEndToEndThroughMotor) {
  // P11-d exists because P11-a's first version was wrong and said the opposite.
  // It ran 1.5 s and read the *final* state, by which time the bench's P-only
  // speed loop had let the machine decay to a speed where no weakening is
  // needed, so `i_d` read ~0 and the test concluded "the policy never engages
  // inside Motor". It does: 2 ms after being parked at 3200 rad/s with the
  // setpoint above it, `i_d` is already -2 A. P11-d pins the moment instead of
  // the aftermath, which is the only window in which the claim is even
  // meaningful. P11-d 存在，是因为 P11-a
  // 的第一版是错的、而且结论反过来。它跑了 1.5 s 去读**终态**， 那时台架的纯 P
  // 速度环已经让电机衰减到不需要弱磁的转速，`i_d` 读到 ~0，测试于是断言 "策略在
  // `Motor` 里从未生效"。它生效了：停在 3200 rad/s、给定在上方，2 ms 后 `i_d`
  // 已经是 −2 A。P11-d
  // 钉住**那一刻**而不是事后——只有那个窗口里这个主张才有意义。
  using MotorT =
      foc::motor::Motor<core::NonlinearFluxObserver, core::Svpwm, core::Pi,
                        sil::PlantHal, core::FieldWeakeningReference,
                        core::NullFeedforward, core::PositionP,
                        core::NullAlignment>;
  const sil::BenchConfig cfg = sil::standard_bench_config();
  const sil::MachineSpec spec = sil::nominal_spec();
  const foc::motor::MotorConfig mc = sil::motor_config_from(spec);
  const float poles = static_cast<float>(spec.pole_pairs);

  sil::PmsmPlant plant(sil::plant_params_from(spec));
  sil::PlantHal hal(plant, cfg.v_bus_v);
  hal.reset(sil::PlantState{0.0F, 0.0F, kTargetRad / poles, 0.0F});
  core::FieldWeakeningReference policy(fw_config(spec));
  core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
      1.05F * spec.rs, spec.lq, psi_f_of(spec), 0.3F, 3.0F,
      0.5F * psi_f_of(spec)});
  MotorT motor(
      mc,
      typename MotorT::Policies{
          observer, core::Svpwm{},
          core::Pi{
              sil::current_pi_config(mc, cfg.omega_current_rad_s, cfg.v_bus_v)},
          core::Pi{
              sil::current_pi_config(mc, cfg.omega_current_rad_s, cfg.v_bus_v)},
          core::Pi{sil::speed_pi_config(mc, cfg.omega_speed_rad_s)}, policy,
          core::NullFeedforward{},
          core::PositionP{foc::position_p_config(mc, 10.0F, 20.0F, 0.0F)},
          core::NullAlignment{}},
      hal);
  ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
  ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
  motor.set_speed(1.15F * kTargetRad / poles);

  const int steps = static_cast<int>((2.0e-3F / cfg.dt_ctrl_s) + 0.5F);
  for (int k = 0; k < steps; ++k) {
    static_cast<void>(motor.tick(cfg.dt_ctrl_s));
    hal.advance(cfg.dt_ctrl_s, cfg.substeps);
  }
  const core::Dq<float> i = plant.current_dq();
  const float speed = poles * plant.omega_mech_rad_s();
  std::printf(
      "\n[P11-d] through Motor, 2 ms after parking at %.0f rad/s: speed %.0f, "
      "i_d %.4f, i_q %.4f\n",
      static_cast<double>(kTargetRad), static_cast<double>(speed),
      static_cast<double>(i.d), static_cast<double>(i.q));

  // The whole point: the d-axis current exists, it is large, it is negative,
  // and the drive is still above base speed while it flows. 要点全在这里：d
  // 轴电流存在、量不小、是负的，而且它流动时驱动器仍在基速以上。
  EXPECT_LT(i.d, -1.5F) << "the policy must engage inside Motor";
  EXPECT_GT(speed, base_speed_rad(spec))
      << "and it must be engaging above base speed";
}

TEST(FieldWeakeningMotor, TheVoltageBudgetIsWhatUnlocksTheSpeed) {
  // The product claim, measured through the only thing that can stop it. Park
  // two identical motors at 3000 rad/s with a 1 A torque command and let them
  // run: the one with the policy weakens its field, keeps its current inside
  // the voltage budget and accelerates; the one with `d = 0` runs out of volts,
  // loses its current and falls back. That contrast is the whole mechanism in
  // one run. 产品主张，通过唯一能拦住它的东西来量。把两台相同的电机停在 3000
  // rad/s、给 1 A 转矩
  // 指令让它们跑：装策略的那台削弱磁场、把电流留在电压预算内、于是加速；`d = 0`
  // 的那台 电压用尽、电流丢掉、于是掉回去。这个对比就是整个机理，一趟跑完。
  const sil::MachineSpec s = sil::nominal_spec();
  const float start = 3000.0F;
  const SpeedRun plain = run_speed_loop(false, 1.0F, start, 1.0F);
  const SpeedRun weak = run_speed_loop(true, 1.0F, start, 1.0F);
  std::printf("\n[P11-a] from %.0f rad/s with a 1 A torque command, 1 s\n",
              static_cast<double>(start));
  std::printf("     %-26s %12s %10s %10s\n", "reference", "speed rad/s", "id A",
              "iq A");
  std::printf("     %-26s %12.1f %10.4f %10.4f\n", "d = 0",
              static_cast<double>(plain.speed_rad_s),
              static_cast<double>(plain.id_a), static_cast<double>(plain.iq_a));
  std::printf("     %-26s %12.1f %10.4f %10.4f\n", "FieldWeakeningReference",
              static_cast<double>(weak.speed_rad_s),
              static_cast<double>(weak.id_a), static_cast<double>(weak.iq_a));

  // 1. With the policy the drive keeps accelerating past the target, because
  // the
  //    current it asks for still fits in the voltage budget.
  // 1. 有策略时驱动器越过目标继续加速，因为它要的电流仍装得进电压预算。
  EXPECT_GT(weak.speed_rad_s, kTargetRad);
  EXPECT_LT(weak.id_a, -0.5F) << "and it pays for that with d-axis current";
  EXPECT_GT(std::fabs(weak.iq_a), 0.9F) << "while keeping the torque current";
  // 2. Without it the same command cannot be delivered above base speed, so the
  //    motor falls back below where it started.
  // 2. 没有它，同一条指令在基速以上交付不了，电机掉回起点以下。
  EXPECT_LT(plain.speed_rad_s, start);
  EXPECT_LT(std::fabs(plain.iq_a), 0.9F)
      << "because the current is what gives way";
  // The plain case does not sit at `i_d = 0` either: once the voltage vector is
  // clamped, the current vector rotates on its own and lands a little negative.
  // That is physics, not the policy -- the difference in magnitude is the
  // point, so it is stated as a comparison rather than against zero.
  // 不用策略的那台也不停在 `i_d =
  // 0`：电压矢量被夹住之后，电流矢量自己会转，落得略微为负。
  // 那是物理，不是策略——要点在量级之差，所以写成比较而不是与零比。
  EXPECT_GT(plain.id_a, 0.25F * weak.id_a);
}
TEST(FieldWeakeningMotor,
     TheNormalisationHypothesisIsAGainChangeAndItWasMeasured) {
  // The field-weakened operating point: here the magnet flux the correction
  // pulls towards is genuinely not the flux present.
  // 弱磁工作点：这里"修正所拉向的磁钢磁链"确实不是"实际存在的磁链"。
  const core::Dq<float> current{-2.98F, 1.0F};
  std::printf("\n[P11-b] at %.0f rad/s, i = (%.2f, %.2f) A\n",
              static_cast<double>(kTargetRad), static_cast<double>(current.d),
              static_cast<double>(current.q));
  std::printf("     %12s %14s %14s\n", "gain_ratio", "error rad", "normalised");
  // The normalised law with the incumbent's 0.3 is 0.3/psi_f = 63. Sweeping the
  // existing knob lands exactly on that number, which is why this is a sweep
  // and not a code change. 现有 0.3 的归一化律就是 0.3/ψf =
  // 63。扫现有旋钮正好落在这个数上，这也是它是一次扫描 而不是一次改代码的原因。
  const float gains[] = {0.03F, 0.3F, 3.0F, 30.0F, 63.0F};
  float best = 1.0e9F;
  float at_incumbent = 0.0F;
  bool any_blew_up = false;
  for (const float g : gains) {
    const float error = fw_angle_error(current, g);
    const bool finite = std::isfinite(error);
    any_blew_up = any_blew_up || !finite;
    if (finite) {
      best = (error < best) ? error : best;
    }
    if (std::fabs(g - 0.3F) < 1.0e-6F) {
      at_incumbent = error;
    }
    if (finite) {
      std::printf("     %12.1f %14.5f %14s\n", static_cast<double>(g),
                  static_cast<double>(error), (g > 10.0F) ? "yes" : "");
    } else {
      std::printf("     %12.1f %14s %14s\n", static_cast<double>(g), "diverged",
                  (g > 10.0F) ? "yes" : "");
    }
  }
  std::printf("     incumbent k=0.3: %.5f rad; best finite point in the sweep: "
              "%.4f rad; "
              "target for normalisation: %.3f rad\n",
              static_cast<double>(at_incumbent), static_cast<double>(best),
              static_cast<double>(kNormalisedTargetRad));

  // Two things must hold, and together they are the honest verdict.
  // 两件事必须成立，合起来才是诚实的结论。
  // 1. The incumbent gain is inside the ceiling. This is what ships.
  // 1. 出厂增益在天花板以内。这是出厂的那个。
  EXPECT_LT(at_incumbent, kAngleCeilingRad);
  // 2. The normalised gains (30x and 210x) do not deliver the target. Whatever
  //    they do -- including diverging -- the hypothesis is refuted, and the
  //    numbers stay on the record instead of being quietly dropped.
  // 2. 归一化增益（30 倍与 210
  // 倍）没有给出目标。无论它们做了什么——包括发散——假设都被
  //    反驳，而且这些数留在记录里，不被悄悄丢掉。
  const bool normalisation_meets_target =
      std::isfinite(best) && (best <= kNormalisedTargetRad) && !any_blew_up;
  std::printf("     hypothesis %s\n",
              normalisation_meets_target ? "holds" : "refuted");
  EXPECT_FALSE(normalisation_meets_target)
      << "if this ever starts holding, the correction law should be normalised "
         "and the "
         "finding rewritten";
  // 3. And the sweep has to be doing something: the error must move with the
  // gain,
  //    or it is not reaching the correction at all.
  // 3. 而且扫描必须真的在做事情：误差得随增益变化，否则它根本没碰到修正项。
  EXPECT_GT(std::fabs(fw_angle_error(current, 3.0F) - at_incumbent), 1.0e-3F);
}

TEST(FieldWeakeningMotor, TheWorthItTestIsAvailableBeforeYouTuneAnything) {
  // Productisation means the library answers "is this worth doing on my motor"
  // before the user spends a day tuning it. The answer for the nominal machine
  // is *no*, and saying so is the feature.
  // 产品化意味着库能在用户花一天调参**之前**回答"我这台电机值不值得做"。标称机的答案是
  // **不值得**，把这个说出来才是那个功能。
  const sil::MachineSpec s = sil::nominal_spec();
  const core::MachineParams machine{s.rs, s.ld, s.lq, psi_f_of(s)};
  const auto verdict = core::is_field_weakening_worthwhile(machine, s.i_max_a);
  std::printf(
      "[P11-c] nominal machine: i_char %.2f A, i_char/i_max %.2f, speed gain "
      "%.3f, worthwhile %s\n",
      static_cast<double>(verdict.i_char_a), static_cast<double>(verdict.ratio),
      static_cast<double>(verdict.speed_gain),
      verdict.worthwhile ? "yes" : "no");
  EXPECT_FALSE(verdict.worthwhile) << "x1.34 speed for 4x the current "
                                      "characteristic is not worth the tuning";
  EXPECT_GT(verdict.ratio, 3.0F);
}

} // namespace foc
