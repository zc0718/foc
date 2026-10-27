// Conan::ImportStart
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

#include <foc/core/alignment.hpp>
#include <foc/core/feedforward.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/position.hpp>
#include <foc/core/protection.hpp>
#include <foc/core/reference.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>

#include "sil_bench.hpp"
#include "sil_plant_hal.hpp"
// Conan::ImportEnd

namespace foc {

namespace {

namespace core = foc::core;

constexpr float kDt = 5.0e-5F; // 20 kHz

/// [en] A machine that is healthy in every respect, so each test can move
/// exactly one thing.
///      [zh] 各方面都健康的机器，于是每个测试只动一样东西。
auto healthy() -> core::ProtectionInput {
  return core::ProtectionInput{1.5F, 24.0F, 45.0F, 200.0F, 0.02F, true};
}

auto config() -> core::ProtectionConfig {
  core::ProtectionConfig config{};
  config.i_hard_a = 4.0F;
  config.i_max_a = 3.0F;
  // The budget is `i_cont^2 * tau`, not "one second at rated". With a 3 A
  // continuous rating and a 0.5 s thermal constant the equilibrium budget
  // is 4.5 A^2*s, so 3 A sits *at* the limit forever and anything above it
  // spends the margin at a finite rate. A budget that is not `i^2*tau` either
  // trips a machine running at its own nameplate or never trips at all.
  // 预算是 `i_cont²·τ`，不是"额定下跑一秒"。3 A 连续额定、0.5 s
  // 热时间常数下平衡预算就是 4.5 A²·s，于是 3 A
  // 永远坐在限值上，超过它的部分按有限速率花掉余量。不是 `i²τ` 的预算，
  // 要么会跳掉一台照铭牌运行的机器，要么永远不跳。
  config.i2t_a2s = 4.5F; // 3 A squared times 0.5 s
  config.i2t_cooling_s =
      0.5F; // the winding's thermal constant, not a tuning number
  config.v_max_v = 30.0F;
  config.v_min_v = 18.0F;
  config.t_trip_c = 100.0F;
  config.t_derate_start_c = 60.0F;
  config.t_derate_end_c = 110.0F;
  config.stall_speed_rad_s = 5.0F;
  config.stall_current_a = 1.0F;
  config.stall_time_s = 0.5F;
  config.sync_error_ratio = 0.25F;
  config.hysteresis_ratio = 0.05F;
  return config;
}

/// [en] How far the *input* has to move for a fault of this class to appear.
///      [zh] 让某类故障出现，**输入**需要偏到哪里。
auto perturbed(core::FaultCode code) -> core::ProtectionInput {
  core::ProtectionInput input = healthy();
  switch (code) {
  case core::FaultCode::OverCurrent:
    input.i_magnitude_a = 4.5F;
    break;
  case core::FaultCode::OverVoltage:
    input.v_bus_v = 31.0F;
    break;
  case core::FaultCode::UnderVoltage:
    input.v_bus_v = 17.0F;
    break;
  case core::FaultCode::OverTemperature:
    input.temperature_c = 101.0F;
    break;
  case core::FaultCode::I2t:
    // Above rated but below the hard limit: this is the only window where `I2t`
    // is the *first* mechanism to speak. 高于额定、低于硬限：这是 `I2t`
    // 能第一个开口的唯一窗口。
    input.i_magnitude_a = 3.5F;
    break;
  case core::FaultCode::Stall:
    input.i_magnitude_a = 2.0F;
    input.omega_e_rad_s = 0.0F;
    break;
  case core::FaultCode::LostSync:
    input.flux_error_ratio = 0.4F;
    break;
  case core::FaultCode::None:
    break;
  }
  return input;
}

/// [en] Run one perturbation to steady state and report what the protector
/// said. The thermal budget and the stall timer are durations, so "steady
/// state" is `steps` steps of holding the input.
///      [zh]
///      把一次扰动跑到稳态，报告保护器说了什么。热预算与堵转计时是**时长**，所以"稳态"是
///      保持该输入 `steps` 步。
auto classify(core::FaultCode target, int steps = 1) -> core::FaultCode {
  core::Protector protector(config());
  const core::ProtectionInput input = perturbed(target);
  core::ProtectionOutput out{};
  for (int k = 0; k < steps; ++k) {
    out = protector.step(input, kDt);
  }
  return out.fault;
}

/// [en] The code as text, so the printed matrix is readable. [zh]
/// 码的文本形式，让打印出来的矩阵可读。
auto fault_name(core::FaultCode code) -> const char * {
  switch (code) {
  case core::FaultCode::None:
    return "None";
  case core::FaultCode::OverCurrent:
    return "OverCurrent";
  case core::FaultCode::OverVoltage:
    return "OverVoltage";
  case core::FaultCode::UnderVoltage:
    return "UnderVoltage";
  case core::FaultCode::OverTemperature:
    return "OverTemperature";
  case core::FaultCode::I2t:
    return "I2t";
  case core::FaultCode::Stall:
    return "Stall";
  case core::FaultCode::LostSync:
    return "LostSync";
  }
  return "?";
}

constexpr float kStallSteps = 0.5F / kDt; // stall_time_s

/// [en] How long `I2t` actually takes to trip at a steady current, with
/// cooling. The naive answer `budget/i^2` is **too short**: the budget leaks
/// while it fills, so the trip arrives later by
/// `tau*ln(1/(1-limit/(i^2*tau)))`. At 3.5 A against a 4.5 A^2*s budget and a
/// 0.5 s constant that is 0.663 s against the naive 0.367 s -- nearly twice --
/// and a test written with the naive number would conclude the protection was
/// broken.
///      [zh] `I2t` 在恒定电流下实际要多久才跳，含冷却。朴素答案 `预算/i²`
///      **偏短**：预算一边充 一边漏，所以跳闸来得更晚，晚
///      `τ·ln(1/(1−limit/(i²τ)))`。3.5 A、4.5 A²·s 预算、0.5 s 时间常数下是
///      0.663 s 对朴素的 0.367
///      s——差了近一倍——用朴素数写的测试会得出"保护坏了"。
auto i2t_trip_seconds(float current_a) -> float {
  const float equilibrium = current_a * current_a * 0.5F;
  return 0.5F * std::log(1.0F / (1.0F - (4.5F / equilibrium)));
}

} // namespace

TEST(Protection, P6a_EveryFaultIsClassifiedAsItselfAndNothingElse) {
  // The injection matrix the plan asks for: one row per mechanism, each row
  // perturbing exactly one input, and the reported code has to be that
  // mechanism. 100 % or the classification is useless
  // -- an operator reads one code and does one thing.
  // 计划要的注入矩阵：每个机理一行，每行只扰动一个输入，报出的码必须是那个机理。100
  // % 正确， 否则分类就没用——运维只读一个码、只做一件事。
  struct Row final {
    core::FaultCode code;
    int steps;
  };
  const Row rows[] = {
      {core::FaultCode::None, 1},
      {core::FaultCode::OverCurrent, 1},
      {core::FaultCode::OverVoltage, 1},
      {core::FaultCode::UnderVoltage, 1},
      {core::FaultCode::OverTemperature, 1},
      {core::FaultCode::I2t,
       static_cast<int>(i2t_trip_seconds(3.5F) / kDt) + 4},
      {core::FaultCode::Stall, static_cast<int>(kStallSteps) + 2}};
  std::printf("\n[P6-a] injection matrix\n");
  int correct = 0;
  for (const Row &row : rows) {
    const core::FaultCode got = classify(row.code, row.steps);
    const bool ok = (got == row.code);
    correct += ok ? 1 : 0;
    std::printf("        injected %-16s -> %-16s %s\n", fault_name(row.code),
                fault_name(got), ok ? "ok" : "WRONG");
    EXPECT_EQ(row.code, got) << "injected " << fault_name(row.code);
  }
  // Lost sync is evaluated without any timing, so it is its own row.
  const core::FaultCode lost = classify(core::FaultCode::LostSync);
  std::printf("        injected %-16s -> %-16s %s\n",
              fault_name(core::FaultCode::LostSync), fault_name(lost),
              (lost == core::FaultCode::LostSync) ? "ok" : "WRONG");
  correct += (lost == core::FaultCode::LostSync) ? 1 : 0;
  std::printf("        %d of %d rows correct\n", correct,
              static_cast<int>(sizeof(rows) / sizeof(rows[0])) + 1);
  EXPECT_EQ(8, correct) << "the matrix has to be exact, not mostly right";
}

TEST(Protection, P6b_ATripRemovesOutputAuthorityInTheSameStepItIsDetected) {
  // "Within one control period" means *this* step, not the next one. The
  // measurement and the decision share a step, so the PWM update that follows
  // already has no authority.
  // "一个控制周期之内"指的是**这一步**，不是下一步。测量与判定共用一步，所以紧随其后的
  // PWM 更新 已经没有权限。
  core::Protector protector(config());
  const core::ProtectionInput before = healthy();
  const core::ProtectionOutput quiet = protector.step(before, kDt);
  EXPECT_TRUE(quiet.output_allowed);
  EXPECT_FLOAT_EQ(config().i_max_a, quiet.i_limit_a);

  const core::ProtectionInput over = perturbed(core::FaultCode::OverCurrent);
  const core::ProtectionOutput tripped = protector.step(over, kDt);

  std::printf("[P6-b] over-current step: fault %s, allowed %d, i_limit %.2f A "
              "(was %.2f A)\n",
              fault_name(tripped.fault), tripped.output_allowed ? 1 : 0,
              static_cast<double>(tripped.i_limit_a),
              static_cast<double>(quiet.i_limit_a));

  EXPECT_EQ(core::FaultCode::OverCurrent, tripped.fault);
  EXPECT_FALSE(tripped.output_allowed)
      << "the same step that saw 4.5 A must already refuse to switch";
  EXPECT_FLOAT_EQ(0.0F, tripped.i_limit_a)
      << "a caller that only reads the limit must not keep pushing";
  // And it stays refused while the cause is present.
  // 成因还在时就一直拒绝。
  const core::ProtectionOutput again = protector.step(over, kDt);
  EXPECT_FALSE(again.output_allowed);
}

TEST(Protection, P6c_WhatSelfClearsDoesAndWhatDoesNotStaysUntilReset) {
  // P6-c, both halves. The split is physical (see `fault_is_latched`): a
  // sagging bus lifts, a shorted winding does not fix itself. P6-c
  // 的两半。这个分法是物理的（见
  // `fault_is_latched`）：塌下去的母线会自己回来，短路的绕组 不会自己修好。
  core::Protector protector(config());

  // (1) Auto: over-voltage while the machine regenerates, then the bus comes
  // back. （1）自恢复：机器回馈时母线抬高，然后母线回落。
  core::ProtectionInput high_bus = healthy();
  high_bus.v_bus_v = 31.0F;
  const core::ProtectionOutput over = protector.step(high_bus, kDt);
  const core::ProtectionOutput back = protector.step(healthy(), kDt);
  std::printf("[P6-c] over-voltage %s -> after the bus recovers %s\n",
              fault_name(over.fault), fault_name(back.fault));
  EXPECT_EQ(core::FaultCode::OverVoltage, over.fault);
  EXPECT_EQ(core::FaultCode::None, back.fault)
      << "over-voltage has to clear itself";
  EXPECT_TRUE(back.output_allowed);

  // (2) Latched: the same over-current, then the current goes away.
  // （2）锁存：同样给一次过流，然后把电流撤掉。
  core::Protector latched(config());
  const core::ProtectionOutput trip =
      latched.step(perturbed(core::FaultCode::OverCurrent), kDt);
  EXPECT_EQ(core::FaultCode::OverCurrent, trip.fault);
  for (int k = 0; k < 200; ++k) { // 10 ms of nothing wrong
    static_cast<void>(latched.step(healthy(), kDt));
  }
  const core::ProtectionOutput still = latched.step(healthy(), kDt);
  std::printf("[P6-c] over-current %s -> 10 ms later, everything healthy: %s\n",
              fault_name(trip.fault), fault_name(still.fault));
  EXPECT_EQ(core::FaultCode::OverCurrent, still.fault)
      << "an over-current is not something that stops being true when the "
         "current stops";
  EXPECT_FALSE(still.output_allowed);

  // (3) An operator resets it, and only then does it run again.
  // （3）运维复位，只有那时才重新允许运行。
  latched.reset();
  const core::ProtectionOutput after_reset = latched.step(healthy(), kDt);
  std::printf("[P6-c] after reset(): %s, allowed %d\n",
              fault_name(after_reset.fault),
              after_reset.output_allowed ? 1 : 0);
  EXPECT_EQ(core::FaultCode::None, after_reset.fault);
  EXPECT_TRUE(after_reset.output_allowed);
}

TEST(Protection, P6d_DeratingIsMonotoneAndItsEndsAreTheConfiguredCurve) {
  // P6-d. A derating curve that is not monotone is a curve that hands authority
  // back to a hotter machine, which is the one thing it must never do.
  // P6-d。不单调的降额曲线会把权限还给更热的机器，而那正是它绝不该做的事。
  core::Protector protector(config());
  const float t_start = config().t_derate_start_c;
  const float t_end = config().t_derate_end_c;
  const float i_nom = config().i_max_a;

  float previous = 1.0e9F;
  bool monotone = true;
  std::printf("\n[P6-d] derating curve, i_max %.2f A, %.0f..%.0f deg C\n",
              static_cast<double>(i_nom), static_cast<double>(t_start),
              static_cast<double>(t_end));
  for (int k = 0; k <= 20; ++k) {
    const float t = static_cast<float>(k) * 5.0F; // 0..100 deg C
    const float limit = protector.derated_limit(t);
    monotone = monotone && (limit <= previous);
    previous = limit;
    if ((k % 4) == 0) {
      std::printf("        %5.0f C -> %.4f A\n", static_cast<double>(t),
                  static_cast<double>(limit));
    }
  }
  EXPECT_TRUE(monotone) << "the limit must never increase with temperature";

  // The ends, against the configured curve rather than against a remembered
  // number. 端点，对配置的曲线，而不是对一个记下来的数。
  EXPECT_FLOAT_EQ(i_nom, protector.derated_limit(t_start));
  EXPECT_FLOAT_EQ(i_nom, protector.derated_limit(0.0F))
      << "flat below the derating start";
  EXPECT_NEAR(0.5F * i_nom, protector.derated_limit(0.5F * (t_start + t_end)),
              1.0e-5F)
      << "the middle of the segment is half the nominal";
  EXPECT_FLOAT_EQ(0.0F, protector.derated_limit(t_end));
  EXPECT_FLOAT_EQ(0.0F, protector.derated_limit(t_end + 20.0F));
}

TEST(Protection, P6d_TheThermalBudgetCoolsAndTheStallNeedsItsDuration) {
  // The two faults that are *durations* rather than thresholds, and both would
  // be wrong as plain counters. (1) `I2t` has to cool, or an idle machine
  // carries a stale memory. (2) A stall is current-without-motion for a *time*;
  // 0.4 s of it is normal acceleration, 0.6 s is a jammed axis.
  // 两个**时长**型而非阈值型的故障，两者写成朴素计数器都错。（1）`I2t`
  // 必须会冷却，否则闲置的
  // 机器会背着过期的记忆。（2）堵转是"有力气没运动"持续**一段时长**；0.4 s
  // 是正常加速，0.6 s 是 轴卡了。
  core::Protector protector(config());

  // (2) first, because (1) tampers with the budget.
  // 先测（2），因为（1）会动热预算。
  core::ProtectionInput jam = healthy();
  jam.i_magnitude_a = 2.0F;
  jam.omega_e_rad_s = 0.0F;
  const int under = static_cast<int>(0.4F / kDt);
  for (int k = 0; k < under; ++k) {
    const core::ProtectionOutput out = protector.step(jam, kDt);
    ASSERT_EQ(core::FaultCode::None, out.fault)
        << "0.4 s is acceleration, not a stall";
  }
  EXPECT_NEAR(0.4F, protector.stall_duration(), 2.0F * kDt);
  const int over = static_cast<int>(0.2F / kDt);
  core::ProtectionOutput stall{};
  for (int k = 0; k < over; ++k) {
    stall = protector.step(jam, kDt);
  }
  std::printf(
      "[P6-x] stall: 0.4 s -> none, then %.2f s -> %s (duration %.3f s)\n",
      static_cast<double>(0.6F), fault_name(stall.fault),
      static_cast<double>(protector.stall_duration()));
  EXPECT_EQ(core::FaultCode::Stall, stall.fault);
  // (1) the budget fills above rated and cools below the continuous-safe
  // current. （1）预算在额定之上充、在连续安全电流之下冷。
  core::Protector thermal(config());
  core::ProtectionInput hot = healthy();
  hot.i_magnitude_a = 3.5F; // above the 3 A rating, below the 4 A hard limit
  const float cool_start = thermal.thermal_budget();
  for (int k = 0; k < static_cast<int>(0.3F / kDt); ++k) {
    static_cast<void>(thermal.step(hot, kDt));
  }
  const float warmed = thermal.thermal_budget();
  // 1.5 A is *below* the continuous-safe current `sqrt(4.5/0.5) = 3 A`, which
  // is the only regime in which the budget cools at all: anywhere above
  // `sqrt(q/tau)` it keeps filling, however small the current looks. The first
  // version of this test held 1.5 A and expected cooling, and the measurement
  // said the budget had gone **up** -- which is the model being right. 1.5 A
  // **低于**连续安全电流 `√(4.5/0.5) = 3 A`，这是预算唯一会冷却的区间：在
  // `√(q/τ)` 之上， 无论电流看着多小它都在继续充。本测试第一版保持 1.5 A
  // 并期望冷却，实测说预算**涨了**——那正是 模型在正确地做事。
  core::ProtectionInput cool = healthy();
  cool.i_magnitude_a = 1.5F;
  for (int k = 0; k < static_cast<int>(0.5F / kDt); ++k) {
    static_cast<void>(thermal.step(cool, kDt));
  }
  const float cooled = thermal.thermal_budget();
  std::printf("[P6-x] I2t budget: %.4f -> %.4f (100 ms at 3.5 A) -> %.4f (50 "
              "ms at 1.5 A);\n"
              "        trip at 3.5 A takes %.3f s, not budget/i^2 = %.3f s\n",
              static_cast<double>(cool_start), static_cast<double>(warmed),
              static_cast<double>(cooled),
              static_cast<double>(i2t_trip_seconds(3.5F)),
              static_cast<double>(4.5F / (3.5F * 3.5F)));
  EXPECT_GT(warmed, cool_start);
  EXPECT_LT(cooled, warmed) << "a budget that cannot cool makes an idle "
                               "machine remember a stale overload";
  EXPECT_GT(cooled, 0.0F);
  // And the trip time itself is the finding: the cooling term delays it by
  // nearly 2x.
  EXPECT_GT(i2t_trip_seconds(3.5F), 1.7F * (4.5F / (3.5F * 3.5F)))
      << "the budget leaks while it fills, so `budget/i^2` is not the trip "
         "time";
}

// ---------------------------------------------------------------------------
// P6-b, on a plant rather than on a policy object.
// P6-b，在 plant 上而不是在策略对象上。
// ---------------------------------------------------------------------------

namespace {

using TripMotor =
    foc::motor::Motor<core::BackEmfObserver, core::Svpwm, core::Pi,
                      foc::sil::PlantHal, core::NullReferenceGenerator,
                      core::NullFeedforward, core::PositionP,
                      core::NullAlignment>;

/// [en] A bench that can be tripped: the whole motor, a plant, and a handle on
/// the PWM enable.
///      [zh] 可以被跳闸的台架：完整电机、plant，以及一个 PWM 使能的把手。
/// [en] The nominal machine with its shaft held, so a current decay can be
/// attributed to `L/R` alone. Letting it spin adds the back-EMF to the braking
/// and makes the decay faster than `L/R` -- which is true, and is the second
/// half of what this test is about.
///      [zh] 标称机 + 锁住的轴，于是电流衰减可以只归因于
///      `L/R`。让它转起来会把反电动势加进制动， 衰减就比 `L/R`
///      快——那是真的，也正是本测试后半段要说的事。
auto held_spec() -> foc::sil::MachineSpec {
  foc::sil::MachineSpec spec = foc::sil::nominal_spec();
  spec.j = 1.0e6F;
  spec.b = 0.0F;
  return spec;
}

class TripRig final {
public:
  TripRig()
      : cfg_(foc::sil::standard_bench_config()), spec_(held_spec()),
        mc_(foc::sil::motor_config_from(spec_)),
        plant_(foc::sil::plant_params_from(spec_)), hal_(plant_, cfg_.v_bus_v),
        motor_(mc_,
               typename TripMotor::Policies{
                   core::BackEmfObserver{
                       core::BackEmfObserver::Config{spec_.rs, spec_.ld}},
                   core::Svpwm{},
                   core::Pi{foc::sil::current_pi_config(
                       mc_, cfg_.omega_current_rad_s, cfg_.v_bus_v)},
                   core::Pi{foc::sil::current_pi_config(
                       mc_, cfg_.omega_current_rad_s, cfg_.v_bus_v)},
                   core::Pi{
                       foc::sil::speed_pi_config(mc_, cfg_.omega_speed_rad_s)},
                   core::NullReferenceGenerator{}, core::NullFeedforward{},
                   core::PositionP{
                       foc::position_p_config(mc_, 10.0F, 20.0F, 0.0F)},
                   core::NullAlignment{}},
               hal_) {
    hal_.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
  }

  auto start() -> bool {
    if (motor_.init() != foc::ErrorCode::Ok) {
      return false;
    }
    return motor_.start() == foc::ErrorCode::Ok;
  }

  void step_for(float seconds) {
    const int steps = static_cast<int>((seconds / cfg_.dt_ctrl_s) + 0.5F);
    for (int i = 0; i < steps; ++i) {
      static_cast<void>(motor_.tick(cfg_.dt_ctrl_s));
      hal_.advance(cfg_.dt_ctrl_s, cfg_.substeps);
    }
  }

  auto motor() noexcept -> TripMotor & { return motor_; }
  auto hal() noexcept -> foc::sil::PlantHal & { return hal_; }
  auto current_a() noexcept -> float {
    return std::fabs(plant_.current_dq().q);
  }
  auto dt() const noexcept -> float { return cfg_.dt_ctrl_s; }

private:
  foc::sil::BenchConfig cfg_;
  foc::sil::MachineSpec spec_;
  foc::motor::MotorConfig mc_;
  foc::sil::PmsmPlant plant_;
  foc::sil::PlantHal hal_;
  TripMotor motor_;
};

} // namespace

TEST(Protection,
     P6b_OnAPlantTheCommandDiesInOnePeriodAndTheCurrentTakesLOverR) {
  // The policy-level test above says the *decision* is taken in the same step.
  // This one asks what that means physically, and the answer is not what the
  // plan assumed: **the current cannot be clamped to zero in one control
  // period**, because it flows through an inductance. What can be clamped is
  // the *command*: the PWM is disabled in the same step, and the current then
  // decays with the plant's own `L/R`. At 0.4 mH and 0.5 ohm that is 0.8 ms,
  // sixteen control periods.
  // 上面那个策略层测试说的是**判定**发生在同一步。这一个问它在物理上意味着什么，而答案不是计划
  // 假设的那样：**电流不可能在一个控制周期内被钳到零**，因为它流过一个电感。能被钳住的是**指令**：
  // PWM 在同一步被关掉，然后电流按 plant 自己的 `L/R` 衰减。0.4 mH、0.5 Ω 下是
  // 0.8 ms，十六个 控制周期。
  TripRig rig;
  ASSERT_TRUE(rig.start());
  rig.motor().set_torque(2.0F);
  rig.step_for(20.0e-3F); // 400 periods: the current loop settles

  const float before = rig.current_a();
  const bool enabled_before = rig.hal().pwm.enabled();
  std::printf("\n[P6-b/plant] settled at %.4f A, pwm enabled %d\n",
              static_cast<double>(before), enabled_before ? 1 : 0);
  ASSERT_GT(before, 1.5F)
      << "the operating point has to be real for a decay to mean anything";
  ASSERT_TRUE(enabled_before);

  // The protector sees the over-current and the caller obeys it in the same
  // tick. 保护器看到过流，调用方在同一个 tick 里执行。
  core::Protector protector(config());
  core::ProtectionInput input = healthy();
  input.i_magnitude_a = 4.5F;
  const core::ProtectionOutput decision = protector.step(input, rig.dt());
  ASSERT_FALSE(decision.output_allowed);
  rig.motor().fault();

  // The command is gone immediately: no delay, no next-period grace.
  // 指令立即消失：不延迟，没有下一个周期的宽限。
  EXPECT_FALSE(rig.hal().pwm.enabled())
      << "the bridge is off in the same step the decision was made";
  EXPECT_EQ(core::MotorState::Fault, rig.motor().state());

  // And one control period later the current is *not* zero -- it cannot be.
  // 一个控制周期之后电流**不是**零——不可能是。
  rig.step_for(rig.dt());
  const float after_one = rig.current_a();
  // The decay: measure the time constant and compare it with `L/R` rather than
  // with a remembered number. Two points give it in closed form.
  // 衰减：量时间常数并与 `L/R` 比较，而不是与一个记下来的数比较。两点闭式即可。
  rig.step_for(0.4e-3F);
  const float after_half = rig.current_a();
  rig.step_for(0.4e-3F);
  const float after_full = rig.current_a();
  const float tau_measured = 0.4e-3F / std::log(after_half / after_full);
  const float tau_expected =
      foc::sil::nominal_spec().lq / foc::sil::nominal_spec().rs;

  std::printf("        after 1 period %.4f A (%.1f %% of the operating point), "
              "after 0.4 ms %.4f A, after 0.8 ms %.4f A\n",
              static_cast<double>(after_one),
              static_cast<double>(100.0F * after_one / before),
              static_cast<double>(after_half), static_cast<double>(after_full));
  std::printf("        decay constant %.4f ms, L/R %.4f ms\n",
              static_cast<double>(1.0e3F * tau_measured),
              static_cast<double>(1.0e3F * tau_expected));

  EXPECT_GT(after_one, 0.5F * before)
      << "one period cannot remove an inductive current: the plan's criterion "
         "is physically impossible";
  EXPECT_LT(after_one, before) << "but it must already be falling";
  EXPECT_NEAR(tau_expected, tau_measured, 0.25F * tau_expected)
      << "the decay is the plant's L/R, not something the protection controls";
  EXPECT_LT(after_full, 0.4F * before)
      << "and it is gone in a couple of time constants";
}
} // namespace foc
