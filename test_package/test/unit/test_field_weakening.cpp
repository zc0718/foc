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

/**
 * @brief [en] The exact steady-state voltage magnitude at `(i_d, i_q, omega)`, `Rs` included. This is what the
 *             `Rs`-free decision has to be checked against.
 * @brief [zh] `(i_d, i_q, ω)` 处精确的稳态电压幅值，含 `Rs`。`Rs`-free 的判定必须对着它验。
 */
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


TEST(FieldWeakening, OneNumberDecidesItAndEveryRealMachineHereFailsIt) {
  // C7 step (1): the go/no-go as a **selection function**, run over the machines this project actually has
  // parameters for. The interesting part is not that it says no -- the single-machine case already did that -- but
  // that it says no to all of them, including a traction motor designed by people who knew what they were doing, and
  // that the two-condition rule in the header is really one condition.
  // C7 第（1）步：把做/不做做成**选型函数**，跑在本项目真正有参数的机器上。有价值的不是它说"不"——单机用例
  // 已经说过——而是它对**全部**这些机器都说"不"，包括一台由懂行的人设计的牵引电机；以及头文件里那条"两条件"
  // 规则其实只是一条。
  struct Row final {
    const char* name;
    float ld;
    float lq;
    float psi_f;
    float i_max;
    const char* source;
  };
  // Every number below is transcribed from a document, not fitted: the paper tables are in `workspace/materials/`,
  // `IPMSM_A` is the pyleecan geometry of the 2004 Prius traction motor with `psi_f` = 0.188 Wb and Ld/Lq measured by
  // the FEMM differential route, and the SIL machine is this repository's own.
  // 下面每个数都是抄自文档、不是拟合的：论文表格在 `workspace/materials/`，`IPMSM_A` 是 2004 Prius 牵引电机的
  // pyleecan 几何，ψf = 0.188 Wb、Ld/Lq 由 FEMM 差分路线量出，SIL 机是本仓库自己的。
  const Row rows[] = {
      {"SIL nominal", sil::nominal_spec().ld, sil::nominal_spec().lq, nominal_psi(), sil::nominal_spec().i_max_a,
       "this repository, machine_params.hpp"},
      {"startup-2026 T2", 5.25e-3F, 12.0e-3F, 0.1827F, 4.0F, "startup paper, table 2"},
      {"mtpa-2021 T1 raw", 0.282e-3F, 0.0828e-3F, 0.0182F, 51.6F, "mtpa paper, table 1 as printed"},
      {"mtpa-2021 T1 swapped", 0.0828e-3F, 0.282e-3F, 0.0182F, 51.6F, "same table, Ld/Lq exchanged (our correction)"},
      {"IPMSM_A = Prius 2004", 0.1899e-3F, 0.6406e-3F, 0.188F, 250.0F, "pyleecan geometry + FEMM, DOE locked rotor"},
  };
  int accepted = 0;
  int worthwhile = 0;
  float closest = 0.0F;
  for (const Row& row : rows) {
    // The health check comes first, and it is not a formality: a parameter table with `Lq < Ld` cannot describe an
    // interior-PM machine at all, so every number derived from it afterwards is fiction. This is why the mtpa table's
    // raw row is excluded rather than averaged in.
    // 体检先行，而且不是形式：`Lq < Ld` 的参数表根本描述不了内置式永磁电机，之后从它推出的每个数都是虚构的。
    // 这就是 mtpa 表原样那一行被排除而不是被平均进来的原因。
    // A table with `Lq < Ld` cannot describe an interior-PM machine; `Lq == Ld` is a surface machine, which is a real
    // machine but not one this criterion was written for -- both are still read through the same function.
    // `Lq < Ld` 的表描述不了内置式永磁电机；`Lq == Ld` 是表贴机，是真实机器但不是这条判据面向的对象——
    // 两者都照样过同一个函数。
    const bool is_a_machine = row.lq >= row.ld;
    const core::MachineParams params{0.0F, row.ld, row.lq, row.psi_f};
    const core::FieldWeakeningAdvice advice = core::is_field_weakening_worthwhile(params, row.i_max);
    // The one number that decides it, from the row's own nameplate: `lambda = Ld*i_max/psi_f`. The header's rule has
    // two halves; below it is proved that the second can never bind, so this is the criterion.
    // 决定它的一个数，来自该行自己的铭牌：`lambda = Ld·i_max/ψf`。头文件的规则有两半；下面证明第二半永不生效，
    // 所以这就是判据。
    const float lambda = (row.ld * row.i_max) / row.psi_f;
    std::printf("[C7] %-22s ld %.5f lq %.5f psi %.5f i_max %6.1f | i_char %8.3f A ratio %6.3f lambda %6.3f "
                "gain x%7.3f -> %s%s\n",
                row.name, static_cast<double>(row.ld), static_cast<double>(row.lq), static_cast<double>(row.psi_f),
                static_cast<double>(row.i_max), static_cast<double>(advice.i_char_a),
                static_cast<double>(advice.ratio), static_cast<double>(lambda), static_cast<double>(advice.speed_gain),
                advice.worthwhile ? "BUILD IT" : "do not", is_a_machine ? "" : "   [not a machine: Lq < Ld]");
    if (!is_a_machine) {
      continue;
    }
    ++accepted;
    // The verdict is checked against the rule recomputed from the row, not trusted.
    // 结论对着从该行重算出来的规则验，不是信它。
    EXPECT_EQ((lambda >= 1.0F), advice.worthwhile) << row.name << ": lambda " << lambda;
    if (advice.worthwhile) {
      ++worthwhile;
    } else if (row.ld * row.i_max / row.psi_f > closest) {
      closest = lambda;
      (void)closest;
    }
  }
  EXPECT_EQ(4, accepted) << "the health check should have rejected exactly one row";
  // **This is the finding**: not one of the four real machines has enough inductance per unit of flux to earn field
  // weakening, and the closest is the Prius traction motor -- a machine built for exactly this -- at lambda = 0.51,
  // i.e. 3.3x short of the 5/3 the criterion asks for. So "field weakening" is a niche feature for low-flux,
  // high-inductance designs, and claiming it in general would be claiming it for machines that do not exist here.
  // **这就是发现**：四台真实机器里没有一台的"每单位磁链的电感"足以撑起弱磁，最接近的是 Prius 牵引电机——一台
  // 正为此而造的机器——λ = 0.51，比判据要的 5/3 差 3.3 倍。所以弱磁是一个面向"低磁链、高电感"设计的窄众特性，
  // 泛泛地声称它，等于为这里并不存在的机器声称。
  EXPECT_EQ(0, worthwhile) << "a real machine passed; the table's conclusion needs rewriting, not the assertion";
}

TEST(FieldWeakening, TheVerdictIsOneComparisonAndTheOldThresholdsCouldNeverDecideIt) {
  // The header used to state the rule as `ratio < 0.6 && speed_gain >= 1.5` and give a rationale for each half. Both
  // halves are **implied** by the branch that actually decides -- `i_char <= i_max`, i.e. `lambda = Ld*i_max/psi_f >= 1`
  // -- so the two-condition expression was unreachable-as-true and the thresholds were decoration (F-107). This case
  // pins the boundary the function really has, and pins that the old pair could never have moved it.
  // 头文件原先把规则写成 `ratio < 0.6 && speed_gain >= 1.5`，并为每一半给理由。两半都被真正决定它的那个分支
  // ——`i_char <= i_max`，即 `λ = Ld·i_max/ψf >= 1`——所**蕴含**，于是那条两条件表达式永远不可能为真、两个阈值只是
  // 装饰（F-107）。本用例钉住函数真正的边界，并钉住那对旧阈值从来不可能移动它。
  const float psi = 4.762e-3F;
  const float i_max = 3.0F;
  const auto machine_at = [&](float lambda) {
    const float ld = (lambda * psi) / i_max;
    return core::MachineParams{0.0F, ld, 2.0F * ld, psi};
  };
  int implied_ratio = 0;
  int implied_gain = 0;
  int samples = 0;
  for (int k = 0; k <= 400; ++k) {
    // `lambda` from 0.02 to 6: below the line, at it, and far above it.
    // `λ` 从 0.02 到 6：线下一段、线上、远在其上。
    const float lambda = 0.02F * std::pow(300.0F, static_cast<float>(k) / 400.0F);
    const core::FieldWeakeningAdvice advice = core::is_field_weakening_worthwhile(machine_at(lambda), i_max);
    ++samples;
    // The implemented rule, at every sample.
    // 实现的规则，在每个采样点上。
    EXPECT_EQ(lambda >= 1.0F, advice.worthwhile) << "lambda " << lambda;
    // And the implication: whenever the *old* ratio threshold passes, so does the verdict, and whenever it fails the
    // verdict can still pass -- which is exactly why it was not the deciding test.
    // 以及这个蕴含：旧比率阈值通过时结论必然通过，而它不通过时结论仍可能通过——这正是它不是决定性检验的原因。
    if (advice.ratio < 0.6F) {
      ++implied_ratio;
      EXPECT_TRUE(advice.worthwhile) << "the ratio half was the strict one at lambda " << lambda;
    }
    if (advice.speed_gain >= 1.5F) {
      ++implied_gain;
    }
  }
  // The sweep has to actually reach both sides of both old thresholds, or the implications below prove nothing.
  // 扫描必须真正跨过两个旧阈值的两侧，否则下面的蕴含什么也证明不了。
  EXPECT_EQ(401, samples);
  EXPECT_GT(implied_ratio, 0) << "the sweep never reached the region where the old ratio threshold passes";
  EXPECT_LT(implied_ratio, samples) << "the old ratio threshold passed everywhere, so it was not a threshold";
  EXPECT_GT(implied_gain, implied_ratio) << "the gain half never fired more widely than the ratio half, so the two "
                                           "thresholds would have agreed and there would be nothing to remove";

  // The flip point, bisected on the function itself: `lambda = 1`, where the current limit can just null the flux.
  // 翻转点，直接在函数上二分：`λ = 1`，即电流限幅刚好能把磁链削到零。
  float lo = 0.5F;
  float hi = 2.0F;
  ASSERT_FALSE(core::is_field_weakening_worthwhile(machine_at(lo), i_max).worthwhile);
  ASSERT_TRUE(core::is_field_weakening_worthwhile(machine_at(hi), i_max).worthwhile);
  for (int k = 0; k < 40; ++k) {
    const float mid = 0.5F * (lo + hi);
    if (core::is_field_weakening_worthwhile(machine_at(mid), i_max).worthwhile) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  std::printf("[C7] the verdict flips between lambda %.6f and %.6f (at 1.0 the flux is exactly nullable)\n",
              static_cast<double>(lo), static_cast<double>(hi));
  EXPECT_NEAR(1.0F, hi, 1.0e-5F);
  EXPECT_NEAR(1.0F, lo, 1.0e-5F);
  // Stated where it bites: the ceiling is *infinite* on one side of that line and finite on the other, so the model's
  // verdict is a step. A machine 1 % inside is not 1 % better than one 1 % outside; it is the point where this
  // closed form stops having a finite answer.
  // 在它真正咬人的地方说清楚：这条线一侧的上限是**无穷**、另一侧是有限，所以模型给出的结论是一个台阶。
  // 差 1 % 进线的机器并不比差 1 % 出线的机器好 1 %；那是这个闭式解不再有有限答案的那个点。
  // Both ends are read *away* from the knife edge on purpose: `lambda = 1` and `lambda = 0.99` are computed as
  // `psi - Ld*i_max`, a subtraction of two nearly equal floats, so whether the residual lands just above or exactly
  // on zero depends on the platform's rounding -- macOS answered differently from Linux here, which is a property of
  // the arithmetic and not of the rule. `lambda = 2` and `lambda = 0.9` carry the same statement with room around them.
  // 两端都刻意**离开**刀刃读：`λ = 1` 与 `λ = 0.99` 都算作 `ψ − Ld·i_max`，两个几乎相等的 float 相减，余量是
  // 略大于零还是恰好为零取决于平台的舍入——macOS 与 Linux 在这里给出了不同答案，那是算术的性质而不是规则的性质。
  // `λ = 2` 与 `λ = 0.9` 把同一句话带上了余量。
  EXPECT_EQ(INFINITY, core::is_field_weakening_worthwhile(machine_at(2.0F), i_max).speed_gain);
  EXPECT_NEAR(10.0F, core::is_field_weakening_worthwhile(machine_at(0.9F), i_max).speed_gain, 0.05F);
}


TEST(FieldWeakening, TheFluxSensitivityIsAsymmetricAndTheCurrentCircleIsWhy) {
  // C7 step (2): the operating point under `Rs +/- 20 %` and `psi_f +/- 10 %`, which is what S4 asks for. Two things
  // come out of it that a single symmetric coefficient would have hidden.
  //
  // **First**, the naive law is an upper bound that is never reached. `Rs = 0`, `i_q = 0` gives
  // `i_d = (v_max/omega - psi_f)/Ld`, hence `d i_d / d psi_f = -1/Ld`; measured, the flux swing moves `i_d` by only
  // **0.67 / 0.50 / 0.24 / 0.01** of that at 2910 / 3200 / 3500 / 3800 rad/s. The reason is that the policy is at the
  // current limit at **every** one of those speeds (`|i| = i_max` to 4 decimal places), so the d axis cannot take all
  // of the flux change it is being handed.
  //
  // **Second, and this is the actionable half**: the two directions are not symmetric. Over-estimating `psi_f` by
  // 10 % moves `i_d` by -0.795 A at the base speed but by **-0.016 A** at 3800 rad/s, because there the point is
  // already at `i_d = -2.98` of a 3 A budget and `i_q` has been eaten down to 0.31 -- there is nothing left to give up.
  // Under-estimating it still moves `i_d` by +0.49 A at the same speed. Asymmetry **1.31x at the base speed, 30.4x at
  // the top of the envelope**. So a `psi_f` that reads high is the dangerous sign, and it is dangerous exactly where
  // field weakening is doing the work.
  //
  // C7 第（2）步：`Rs ± 20 %` 与 `ψf ± 10 %` 下的工作点，也就是 S4 要的那张表。这里出来两件单个对称系数会掩盖的事。
  // **第一**，那条朴素定律是永远达不到的上界：实测磁链摆幅只把它推动 **0.67 / 0.50 / 0.24 / 0.01**（2910/3200/
  // 3500/3800 rad/s），因为策略在**每一个**这些转速上都顶在电流限幅上（`|i| = i_max` 到四位小数），d 轴吃不下
  // 全部被交过来的磁链变化。**第二，也是可行动的那一半**：两个方向并不对称。把 `ψf` 高估 10 % 在基速只推动
  // −0.795 A，到 3800 rad/s 只推动 **−0.016 A**——那里工作点已经在 3 A 预算的 −2.98 上、`i_q` 被吃到 0.31，
  // 没有东西可让了；而低估它同转速下仍推动 +0.49 A。**不对称度在基速 1.31 倍，到包络顶端 30.4 倍。** 所以读高的
  // `ψf` 才是危险的方向，而它恰好在弱磁真正干活的地方最危险。
  const core::MachineParams params = nominal_params();
  const float i_max = sil::nominal_spec().i_max_a;
  const auto point = [&](float rs, float psi, float omega) {
    core::FieldWeakeningReference policy(core::FieldWeakeningReference::Config{rs, params.ld, psi, i_max, kVMaxRatio});
    return policy.generate(core::ReferenceInput{3.0F, omega, kVBus, core::Dq<float>{0.0F, 0.0F}});
  };
  struct Row final {
    float omega;
    float i_d;
    float i_q;
    float up;
    float down;
    float up_share;
    float down_share;
    float asymmetry;
    float rs_share;
  };
  Row rows[4]{};
  const float omegas[4] = {2910.0F, 3200.0F, 3500.0F, 3800.0F};
  const float law = 0.1F * params.psi_f / params.ld; // `-d i_d` per +10 % of flux, `Rs = 0`, `i_q = 0`
  for (int k = 0; k < 4; ++k) {
    const core::Dq<float> base = point(params.rs, params.psi_f, omegas[k]);
    const float up = point(params.rs, params.psi_f * 1.1F, omegas[k]).d - base.d;
    const float down = point(params.rs, params.psi_f * 0.9F, omegas[k]).d - base.d;
    const float rs_swing = point(params.rs * 1.2F, params.psi_f, omegas[k]).d - point(params.rs * 0.8F, params.psi_f, omegas[k]).d;
    rows[k] = Row{omegas[k], base.d, base.q, up, down, up / -law, down / law, std::fabs(down / up), rs_swing / (up - down)};
    std::printf("[C7] w %6.0f | i_d %8.4f i_q %6.4f |i| %6.4f | +10%% psi -> %+8.4f (%.2f law) | -10%% psi -> "
                "%+8.4f (%.2f law) | asym %5.2fx | 20%% Rs is %4.1f %% of the swing\n",
                static_cast<double>(rows[k].omega), static_cast<double>(rows[k].i_d), static_cast<double>(rows[k].i_q),
                static_cast<double>(std::sqrt((rows[k].i_d * rows[k].i_d) + (rows[k].i_q * rows[k].i_q))),
                static_cast<double>(rows[k].up), static_cast<double>(rows[k].up_share), static_cast<double>(rows[k].down),
                static_cast<double>(rows[k].down_share), static_cast<double>(rows[k].asymmetry),
                static_cast<double>(100.0F * std::fabs(rows[k].rs_share)));
    // The point is on the current circle at every speed in the weakened region -- which is *why* the law is only an
    // upper bound, and it is measured rather than argued.
    // 工作点在弱磁区的每个转速上都坐在电流圆上——这正是那条定律只是上界的原因，而且是量出来的。
    const float magnitude = std::sqrt((rows[k].i_d * rows[k].i_d) + (rows[k].i_q * rows[k].i_q));
    EXPECT_NEAR(i_max, magnitude, 0.001F * i_max) << "the current limit no longer binds at omega " << omegas[k];
    // Sign, in both directions: more flux needs more negative `i_d`.
    // 两个方向的符号都对：磁链更大要更负的 `i_d`。
    EXPECT_LT(rows[k].up, 0.0F) << "omega " << omegas[k];
    EXPECT_GT(rows[k].down, 0.0F) << "omega " << omegas[k];
    // And the law is an upper bound that is never reached, in either direction.
    // 而那条定律是两向上都达不到的上界。
    EXPECT_LT(rows[k].up_share, 1.0F) << "omega " << omegas[k];
    EXPECT_LT(rows[k].down_share, 1.0F) << "omega " << omegas[k];
    // `Rs` never dominates the flux, at any speed -- which is what makes the policy's `Rs`-free ellipse defensible
    // rather than a compromise.
    // `Rs` 在任何转速下都不敌磁链——这正是策略那条不含 `Rs` 的椭圆可辩护、而不是妥协的原因。
    EXPECT_LT(std::fabs(rows[k].rs_share), 0.25F) << "omega " << omegas[k];
  }
  // The asymmetry is the finding: it grows by more than an order of magnitude across the envelope.
  // 不对称就是发现本身：它在包络内涨了一个数量级以上。
  EXPECT_NEAR(1.31F, rows[0].asymmetry, 0.05F);
  for (int k = 1; k < 4; ++k) {
    EXPECT_GT(rows[k].asymmetry, rows[k - 1].asymmetry) << "the asymmetry stopped growing at omega " << omegas[k];
  }
  EXPECT_GT(rows[3].asymmetry, 20.0F) << "an over-read flux is no longer much more damaging than an under-read one";
  // Both directions also shrink with speed: there is less `i_q` left to spend on the discrepancy.
  // 两个方向都随转速变小：留给出这笔差额的 `i_q` 越来越少。
  for (int k = 1; k < 4; ++k) {
    EXPECT_LT(std::fabs(rows[k].up), std::fabs(rows[k - 1].up));
    EXPECT_LT(rows[k].down, rows[k - 1].down);
  }
}

} // namespace foc
