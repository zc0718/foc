// Conan::ImportStart
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

#include <foc/core/alignment.hpp>
#include <foc/core/feedforward.hpp>
#include <foc/core/field_weakening.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/position.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>
#include <foc/motor/motor.hpp>

#include "sil_bench.hpp"
#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"
// Conan::ImportEnd

namespace foc {

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

constexpr float kVBus = 24.0F;
constexpr float kVMaxRatio = 1.0F / 1.7320508F; // SVPWM linear range

auto nominal_psi() -> float {
  return sil::nominal_spec().kt /
         (1.5F * static_cast<float>(sil::nominal_spec().pole_pairs));
}

auto nominal_params() -> core::MachineParams {
  return core::MachineParams{sil::nominal_spec().rs, sil::nominal_spec().ld,
                             sil::nominal_spec().lq, nominal_psi()};
}

auto policy_config(const core::MachineParams &params, float i_max_a)
    -> core::FieldWeakeningReference::Config {
  return core::FieldWeakeningReference::Config{
      params.rs, params.ld, params.psi_f, i_max_a, kVMaxRatio};
}

/// [en] The exact steady-state voltage magnitude at `(i_d, i_q, omega)`, `Rs`
/// included. This is what the `Rs`-free decision has to be checked against.
///      [zh] `(i_d, i_q, ω)` 处精确的稳态电压幅值，含 `Rs`。`Rs`-free
///      的判定必须对着它验。
auto exact_voltage(const core::MachineParams &p, float i_d, float i_q,
                   float omega) -> float {
  const float v_d = (p.rs * i_d) - (omega * p.lq * i_q);
  const float v_q = (p.rs * i_q) + (omega * p.ld * i_d) + (omega * p.psi_f);
  return std::sqrt((v_d * v_d) + (v_q * v_q));
}

} // namespace

TEST(FieldWeakening, TheAdviceSaysDoNotBuildItForThisMachine) {
  // P8-d. The whole point of this test is that it says **no** with numbers: an
  // advisory that always says yes is not an advisory.
  // P8-d。这个测试的全部意义是它**用数说不**：永远说"该做"的建议不是建议。
  const core::MachineParams nominal = nominal_params();
  const core::FieldWeakeningAdvice advice =
      core::is_field_weakening_worthwhile(nominal, sil::nominal_spec().i_max_a);

  core::MachineParams high_l = nominal;
  high_l.ld = 4.0e-3F;
  high_l.lq = 4.0e-3F;
  const core::FieldWeakeningAdvice high =
      core::is_field_weakening_worthwhile(high_l, 3.0F);

  std::printf("\n[P8-d] nominal : i_char %.3f A = %.2fx i_max, speed ceiling "
              "x%.3f -> %s\n",
              static_cast<double>(advice.i_char_a),
              static_cast<double>(advice.ratio),
              static_cast<double>(advice.speed_gain),
              advice.worthwhile ? "build it" : "DO NOT");
  std::printf("       10x L   : i_char %.3f A = %.2fx i_max, speed ceiling "
              "x%.3f -> %s\n",
              static_cast<double>(high.i_char_a),
              static_cast<double>(high.ratio),
              static_cast<double>(high.speed_gain),
              high.worthwhile ? "build it" : "DO NOT");

  // The nominal machine is 3.97x on the wrong side of its own characteristic
  // current. 标称机在它自己的特征电流的错的一侧，差 3.97 倍。
  EXPECT_NEAR(3.97F, advice.ratio, 0.05F);
  EXPECT_FALSE(advice.worthwhile);
  // And the ceiling it can buy is the +34 % the envelope measurement showed.
  // 它能买到的上限就是包络实测的那个 +34 %。
  EXPECT_NEAR(1.34F, advice.speed_gain, 0.02F);
  // Ten times the inductance flips the verdict, with no other change.
  // 电感放大十倍就翻转了结论，其它什么都没动。
  EXPECT_TRUE(high.worthwhile);
  EXPECT_NEAR(0.40F, high.ratio, 0.02F);
}

TEST(FieldWeakening, ThePolicyFindsTheEnvelopeAndTheExactVoltageAgrees) {
  // P8-a. The policy solves the ellipse **without `Rs`**; this test is what
  // keeps that approximation honest, by evaluating the full voltage equation at
  // the point the policy chose. P8-a。策略**不含 `Rs`**
  // 地解椭圆；本测试就是让这个近似保持诚实的东西——在策略选出的点上算完整
  // 电压方程。
  const core::MachineParams params = nominal_params();
  core::FieldWeakeningReference policy(
      policy_config(params, sil::nominal_spec().i_max_a));

  std::printf(
      "\n[P8-a] policy output vs the exact envelope (24 V, i_max %.1f A)\n",
      static_cast<double>(sil::nominal_spec().i_max_a));
  std::printf("        %8s | %8s %8s | %8s %8s\n", "w_e", "i_d", "i_q",
              "|v| exact", "v_max");
  const float v_max = kVMaxRatio * kVBus;
  struct Row final {
    float omega;
    float i_d;
    float i_q;
    float volts;
  };
  Row rows[6]{};
  const float omegas[6] = {2000.0F, 2910.0F, 3200.0F,
                           3500.0F, 3800.0F, 5000.0F};
  for (int k = 0; k < 6; ++k) {
    const core::ReferenceInput in{3.0F, omegas[k], kVBus,
                                  core::Dq<float>{0.0F, 0.0F}};
    const core::Dq<float> ref = policy.generate(in);
    rows[k] = Row{omegas[k], ref.d, ref.q,
                  exact_voltage(params, ref.d, ref.q, omegas[k])};
    std::printf("        %8.0f | %8.3f %8.3f | %8.4f %8.4f\n",
                static_cast<double>(omegas[k]), static_cast<double>(ref.d),
                static_cast<double>(ref.q), static_cast<double>(rows[k].volts),
                static_cast<double>(v_max));
    // Inside the current limit, always.
    // 永远在电流限幅内。
    EXPECT_LE((ref.d * ref.d) + (ref.q * ref.q),
              sil::nominal_spec().i_max_a * sil::nominal_spec().i_max_a *
                  1.001F);
    // And inside the voltage ceiling to within the `Rs` the decision ignored (3
    // % of `v_max` is 0.42 V, which is what 3 A through 0.5 ohm is worth).
    // 并且在电压上限内，误差不超过判定忽略掉的那个 `Rs`（`v_max` 的 3 % 是 0.42
    // V，正是 3 A 流过 0.5 Ω 的值）。
    if (ref.q > 0.0F) {
      EXPECT_LE(rows[k].volts, v_max * 1.03F) << "omega " << omegas[k];
    }
  }

  // No field weakening below the base speed, and real `i_d` above it.
  // 基速以下不弱磁，以上有真实的 `i_d`。
  EXPECT_FLOAT_EQ(0.0F, rows[0].i_d) << "2000 rad/s still fits with i_d = 0";
  EXPECT_LT(rows[2].i_d, -2.2F) << "3200 rad/s needs real d-axis current";
  // The plan's P8-a criterion, on the numbers the envelope measured.
  // 计划里 P8-a 的判据，对着包络实测的那些数。
  EXPECT_GE(rows[2].i_q, 1.9F) << "and it keeps torque there";
  EXPECT_LT(rows[2].i_q, 3.0F)
      << "but not all of it: the ellipse is what limits";
  // At the top of the range the torque is gone -- which is the physical fact
  // the advice is built on.
  // 在区间顶端转矩已经没了——这正是那个建议所依据的物理事实。
  EXPECT_LT(rows[4].i_q, 0.4F) << "3800 rad/s is the end of the envelope";
}

TEST(FieldWeakening, TheDaxisReferenceIsContinuousAcrossTheBaseSpeed) {
  // P8-c, at the level where it can actually be checked without a full loop:
  // `i_d(omega)` must be continuous. A discontinuous reference is what winds a
  // current integrator up and costs an overshoot on the way back down.
  // P8-c，在不需要整个环路就能检查的那一层：`i_d(ω)`
  // 必须连续。不连续的参考正是把电流积分器卷上去、 再在回程付出过冲的东西。
  const core::MachineParams params = nominal_params();
  core::FieldWeakeningReference policy(
      policy_config(params, sil::nominal_spec().i_max_a));

  float biggest_step = 0.0F;
  float previous = 0.0F;
  float worst_omega = 0.0F;
  for (int k = 0; k <= 400; ++k) {
    const float omega =
        1500.0F + (30.0F * static_cast<float>(k)); // 1500..13500
    const core::ReferenceInput in{3.0F, omega, kVBus,
                                  core::Dq<float>{0.0F, 0.0F}};
    const float i_d = policy.generate(in).d;
    if (k > 0) {
      const float step = std::fabs(i_d - previous);
      if (step > biggest_step) {
        biggest_step = step;
        worst_omega = omega;
      }
    }
    previous = i_d;
  }
  std::printf(
      "[P8-c] largest i_d step over a 30 rad/s sweep: %.5f A at %.0f rad/s\n",
      static_cast<double>(biggest_step), static_cast<double>(worst_omega));

  // A 30 rad/s step in speed may not move `i_d` by more than a small fraction
  // of the current limit. If it did, the reference would be a cliff and the
  // integrator would be shocked. 转速每走 30 rad/s，`i_d`
  // 的移动不能超过电流限幅的一小部分。超了就说明参考是一道悬崖，积分器会被震。
  EXPECT_LT(biggest_step, 0.10F * sil::nominal_spec().i_max_a)
      << "a continuous reference cannot jump a tenth of the current limit in "
         "30 rad/s";
}

TEST(FieldWeakening, TheObserverStaysInsideTheGateInTheWeakFluxRegion) {
  // P8-b. The weak-flux region changes the *stator* flux to `psi_f + Ld*i_d`
  // while the observer pulls its magnitude towards `psi_f`; the question is
  // whether that is fatal. Measured: it is not, but it costs -- and the
  // corrected reference buys most of it back (P8-f).
  // P8-b。弱磁区把**定子**磁链改成 `ψf + Ld·i_d`，而观测器把幅值拉向
  // `ψf`；问题是这致不致命。实测：
  // 不致命，但有代价——而修正后的参考能把大部分买回来（P8-f）。
  const core::MachineParams params = nominal_params();
  const float psi = params.psi_f;

  // The operating point the policy picks at 3800 rad/s, driven open-loop from
  // the model so the observer is the only thing under test. 策略在 3800 rad/s
  // 选出的工作点，用模型开环驱动，让观测器成为唯一被测的东西。
  core::FieldWeakeningReference policy(
      policy_config(params, sil::nominal_spec().i_max_a));
  const core::Dq<float> ref = policy.generate(
      core::ReferenceInput{3.0F, 3800.0F, kVBus, core::Dq<float>{0.0F, 0.0F}});

  auto run = [&](bool corrected_reference) -> float {
    sil::PlantParams plant_params = sil::plant_params_from(sil::nominal_spec());
    plant_params.j = 1.0e6F;
    plant_params.b = 0.0F;
    sil::PmsmPlant plant(plant_params);
    const float poles = static_cast<float>(sil::nominal_spec().pole_pairs);
    plant.reset(sil::PlantState{ref.d, ref.q, 3800.0F / poles, 0.0F});
    const core::Dq<float> v{params.rs * ref.d - 3800.0F * params.lq * ref.q,
                            params.rs * ref.q + 3800.0F * params.ld * ref.d +
                                3800.0F * psi};
    plant.set_voltage_dq(v);

    core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
        params.rs, params.lq, psi, 0.3F, 3.0F, 0.5F * psi});
    observer.seed_flux(core::Ab<float>{psi, 0.0F});

    const float dt = 5.0e-5F;
    float sum = 0.0F;
    int count = 0;
    for (int k = 0; k < 4000; ++k) {
      plant.advance(dt);
      const float theta_e = poles * plant.theta_mech_rad();
      const core::Ab<float> i_ab = core::inv_park(plant.current_dq(), theta_e);
      const core::Ab<float> v_ab = core::inv_park(v, theta_e);
      if (corrected_reference) {
        // P8-f: tell the correction what the stator flux actually is.
        // P8-f：告诉修正环定子磁链实际是多少。
        observer.apply_params(core::MachineParams{
            params.rs, params.ld, params.lq,
            core::flux_reference_wb(psi, params.ld, plant.current_dq().d)});
      }
      const core::ObsState state = observer.update(v_ab, i_ab, dt);
      if (k >= 2000) {
        float error = state.theta_e - theta_e;
        while (error <= -core::kPi<float>) {
          error += core::kTwoPi<float>;
        }
        while (error > core::kPi<float>) {
          error -= core::kTwoPi<float>;
        }
        sum += std::fabs(error);
        ++count;
      }
    }
    return sum / static_cast<float>(count);
  };

  const float plain = run(false);
  const float corrected = run(true);
  std::printf(
      "[P8-b] 3800 rad/s, i_d %.3f A, i_q %.3f A: angle error %.5f rad plain, "
      "%.5f rad with the flux reference corrected (%.1f %% better)\n",
      static_cast<double>(ref.d), static_cast<double>(ref.q),
      static_cast<double>(plain), static_cast<double>(corrected),
      static_cast<double>(100.0F * (plain - corrected) / plain));

  // P8-b's criterion: inside the gate.
  // P8-b 的判据：在闸内。
  EXPECT_LE(plain, 0.10F) << "the observer has to survive the weak-flux region";
  // P8-f, and the answer is **no**. Aiming the magnitude correction at `psi_f +
  // Ld*i_d` -- the physically correct stator flux -- makes the angle error
  // **worse**, by more than a factor of two. The mechanism is not the magnitude
  // itself: the correction `k*(target - |psi_hat|)` is also the authority that
  // keeps the *angle* honest, so lowering the target weakens the thing that was
  // holding the estimate together. Recorded as a measured negative and not
  // pursued -- the plain reference is already inside the gate.
  // P8-f，答案是**不**。把幅值修正瞄向 `ψf +
  // Ld·i_d`（物理上正确的定子磁链）会让角度误差**变差**，
  // 差一倍以上。机理不在幅值本身：修正项 `k·(目标 − |ψ̂|)`
  // 同时也是让**角度**保持诚实的那个权限，
  // 所以压低目标等于削弱了原本撑住估计的东西。作为**实测的否定结果**记录并不再追——朴素参考本就在闸内。
  EXPECT_GT(corrected, plain) << "the physically correct flux reference "
                                 "measured worse: keep the plain one";
}

TEST(FieldWeakening, ItIsADropInPolicyAndNothingElseChanges) {
  // P8-e. The whole architectural claim: `Motor` already hands the reference
  // generator
  // `{iq, omega_e, v_bus, i_meas}`, so field weakening is **one template
  // argument**. No new slot, no new state, no migration of the existing
  // instantiations. This test is the proof -- it builds a working motor with
  // the policy in place and makes it produce torque.
  // P8-e。整个架构主张：`Motor` 本来就把 `{iq, ω_e, v_bus, i_meas}`
  // 交给参考发生器，所以弱磁只是
  // **一个模板实参**。没有新槽位、没有新状态、既有实例化不需要迁移。本测试就是证明——它带着这个
  // 策略建一台能跑的电机并让它出转矩。
  using MotorT = foc::motor::Motor<core::BackEmfObserver, core::Svpwm, core::Pi,
                                   sil::PlantHal, core::FieldWeakeningReference,
                                   core::NullFeedforward, core::PositionP,
                                   core::NullAlignment>;
  const sil::BenchConfig cfg = sil::standard_bench_config();
  const sil::MachineSpec spec = sil::nominal_spec();
  const foc::motor::MotorConfig mc = sil::motor_config_from(spec);
  sil::PmsmPlant plant(sil::plant_params_from(spec));
  sil::PlantHal hal(plant, cfg.v_bus_v);
  hal.reset(sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
  core::FieldWeakeningReference policy(
      policy_config(nominal_params(), spec.i_max_a));

  MotorT motor(
      mc,
      typename MotorT::Policies{
          core::BackEmfObserver{
              core::BackEmfObserver::Config{spec.rs, spec.ld}},
          core::Svpwm{},
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
  motor.set_torque(2.0F);
  const int steps = static_cast<int>((20.0e-3F / cfg.dt_ctrl_s) + 0.5F);
  for (int k = 0; k < steps; ++k) {
    static_cast<void>(motor.tick(cfg.dt_ctrl_s));
    hal.advance(cfg.dt_ctrl_s, cfg.substeps);
  }
  const float i_q = std::fabs(plant.current_dq().q);
  std::printf("[P8-e] motor with FieldWeakeningReference as a template "
              "argument: state %d, "
              "iq %.4f A at standstill\n",
              static_cast<int>(motor.state()), static_cast<double>(i_q));

  // Running, and producing the commanded torque -- with `i_d` still zero
  // because the speed is zero, which is exactly the behaviour the policy
  // promises. 在跑，并输出给定的转矩——而且因为转速为零，`i_d`
  // 仍为零，这正是策略承诺的行为。
  EXPECT_EQ(core::MotorState::Run, motor.state());
  EXPECT_GT(i_q, 1.5F) << "below base speed the policy is transparent";
  EXPECT_LT(std::fabs(plant.current_dq().d), 0.2F)
      << "and it does not spend d-axis current for nothing";
}

} // namespace foc
