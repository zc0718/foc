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

/**
 * @brief [en] A machine that is healthy in every respect, so each test can move exactly one thing.
 * @brief [zh] 各方面都健康的机器，于是每个测试只动一样东西。
 */
constexpr auto both_bits(std::uint16_t a, std::uint16_t b) -> std::uint16_t { return static_cast<std::uint16_t>(a | b); }

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

/**
 * @brief [en] How far the *input* has to move for a fault of this class to appear.
 * @brief [zh] 让某类故障出现，**输入**需要偏到哪里。
 */
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

/**
 * @brief [en] Run one perturbation to steady state and report what the protector said. The thermal budget and the stall
 *             timer are durations, so "steady state" is `steps` steps of holding the input.
 * @brief [zh] 把一次扰动跑到稳态，报告保护器说了什么。热预算与堵转计时是**时长**，所以"稳态"是保持该输入 `steps` 步。
 */
auto classify(core::FaultCode target, int steps = 1) -> core::FaultCode {
  core::Protector protector(config());
  const core::ProtectionInput input = perturbed(target);
  core::ProtectionOutput out{};
  for (int k = 0; k < steps; ++k) {
    out = protector.step(input, kDt);
  }
  return out.fault;
}

/**
 * @brief [en] The code as text, so the printed matrix is readable.
 * @brief [zh] 码的文本形式，让打印出来的矩阵可读。
 */
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

/**
 * @brief [en] How long `I2t` actually takes to trip at a steady current, with cooling. The naive answer `budget/i^2` is
 *             **too short**: the budget leaks while it fills, so the trip arrives later by
 *             `tau*ln(1/(1-limit/(i^2*tau)))`. At 3.5 A against a 4.5 A^2*s budget and a 0.5 s constant that is 0.663 s
 *             against the naive 0.367 s -- nearly twice -- and a test written with the naive number would conclude the
 *             protection was broken.
 * @brief [zh] `I2t` 在恒定电流下实际要多久才跳，含冷却。朴素答案 `预算/i²` **偏短**：预算一边充 一边漏，所以跳闸来得更
 *             晚，晚 `τ·ln(1/(1−limit/(i²τ)))`。3.5 A、4.5 A²·s 预算、0.5 s 时间常数下是 0.663 s 对朴素的 0.367 s——差了
 *             近一倍——用朴素数写的测试会得出"保护坏了"。
 */
auto i2t_trip_seconds(float current_a) -> float {
  const float equilibrium = current_a * current_a * 0.5F;
  return 0.5F * std::log(1.0F / (1.0F - (4.5F / equilibrium)));
}

} // namespace

TEST(Protection, EveryFaultIsClassifiedAsItselfAndNothingElse) {
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

TEST(Protection, ATripRemovesOutputAuthorityInTheSameStepItIsDetected) {
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

TEST(Protection, WhatSelfClearsDoesAndWhatDoesNotStaysUntilReset) {
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

TEST(Protection, DeratingIsMonotoneAndItsEndsAreTheConfiguredCurve) {
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

TEST(Protection, TheTemperatureLadderIsGradedRatherThanACliff) {
  // The plan's criterion for graded fault actions, and the case that was missing: the derating test above asserts the
  // *curve* `derated_limit(T)`, which is a pure function, and nothing ever stepped the protector with a temperature
  // inside the band. An implementation that tripped at `t_derate_start_c` -- no derating at all -- would pass every
  // case this file had, because the curve it never consults is still correct.
  //
  // The discriminating assertion is the middle rung: inside the band the output is **still allowed** and the limit is
  // strictly between zero and nominal. That is precisely what "skip the derating and trip" cannot produce.
  //
  // 计划给分级动作定的判据，也正是缺失的那个用例：上面的降额测试断言的是**曲线** `derated_limit(T)`，那是个纯函数，
  // 而从来没有让保护器在带内的温度上走过一步。一个在 `t_derate_start_c` 就跳闸的实现——完全不做降额——会通过本文件
  // 原有的每一个用例，因为它从不查询的那条曲线本身是对的。
  //
  // 判别性的断言是中间那一级：带内输出**仍然被允许**、限幅严格落在 0 与额定之间。这正是"跳过降额直接跳闸"给不出的。
  core::Protector protector(config());
  const float t_start = config().t_derate_start_c;
  const float t_trip = config().t_trip_c;
  const float i_nom = config().i_max_a;
  const auto heat_to = [](float temperature_c) {
    core::ProtectionInput input = healthy();
    input.temperature_c = temperature_c;
    input.i_magnitude_a = 1.0F; // no current fault, so only the heat speaks
    return input;
  };

  // Rung 0: cold. Full authority, no over-temperature bit.
  // 第 0 级：常温。全权限，无过温位。
  const core::ProtectionOutput cold = protector.step(heat_to(45.0F), 1e-4F);
  std::printf("\n[P6-e] ladder: %.0f C -> limit %.3f A, output %d, fault %s\n", 45.0, static_cast<double>(cold.i_limit_a),
              cold.output_allowed ? 1 : 0, fault_name(cold.fault));
  EXPECT_TRUE(cold.output_allowed);
  EXPECT_FLOAT_EQ(i_nom, cold.i_limit_a);

  // Rung 1: inside the band, below the trip. The limit falls, the output stays.
  // 第 1 级：带内、跳闸温度以下。限幅下降，输出保留。
  const float middle_c = 0.5F * (t_start + t_trip);
  const core::ProtectionOutput warm = protector.step(heat_to(middle_c), 1e-4F);
  std::printf("[P6-e] ladder: %.0f C -> limit %.3f A, output %d, fault %s\n", static_cast<double>(middle_c),
              static_cast<double>(warm.i_limit_a), warm.output_allowed ? 1 : 0, fault_name(warm.fault));
  EXPECT_TRUE(warm.output_allowed)
      << "inside the derating band the axis must still run, at reduced authority -- tripping here is the failure this "
         "case exists to catch";
  EXPECT_LT(warm.i_limit_a, i_nom) << "the limit must have come down";
  EXPECT_GT(warm.i_limit_a, 0.0F) << "and it must not have reached zero before the trip";
  // No fault code here, and that is the design rather than an omission: `output_allowed` is defined as
  // `active_mask == 0`, so a "fault" in this library *means* "authority removed". Derating is the other decision --
  // the axis still runs, at reduced authority -- and it is announced by `i_limit_a` falling below the `i_max_a` the
  // caller configured. A `derating` flag was considered and rejected: it would be a seventh element on a tuple that
  // five consumers already unpack, for a boolean the number beside it already states.
  // 这里没有故障码，而这是设计而不是遗漏：`output_allowed` 的定义是 `active_mask == 0`，因此本库的"故障"**就是**"权限被
  // 收回"。降额是另一个决定——轴仍在跑，只是权限降低——它由 `i_limit_a` 跌到调用方自己配置的 `i_max_a` 之下来宣告。曾
  // 考虑加一个 `derating` 布尔并否决了：那是给五个消费者已经在解包的元组加第七个元素，只为重复旁边那个数已经说明的事。
  EXPECT_EQ(core::FaultCode::None, warm.fault) << "a fault here means 'authority removed', and it has not been";
  EXPECT_EQ(0U, warm.active_mask) << "derating is not a fault: the axis is still running";

  // Rung 2: above the trip temperature. Now authority goes to zero and switching stops.
  // 第 2 级：超过跳闸温度。权限归零，开关停止。
  const core::ProtectionOutput hot = protector.step(heat_to(t_trip + 5.0F), 1e-4F);
  std::printf("[P6-e] ladder: %.0f C -> limit %.3f A, output %d, fault %s\n", static_cast<double>(t_trip + 5.0F),
              static_cast<double>(hot.i_limit_a), hot.output_allowed ? 1 : 0, fault_name(hot.fault));
  EXPECT_FALSE(hot.output_allowed);
  EXPECT_FLOAT_EQ(0.0F, hot.i_limit_a);
  EXPECT_EQ(core::FaultCode::OverTemperature, hot.fault);
}


TEST(Protection, TwoFaultsAtOnceReportTheLowerRankedCodeAndBothBits) {
  // "Ranked by the enum order, so current before heat before motion" is what the header claims and no case checked: every
  // case so far moved one input, so `active_mask` and the ranking were unexercised. A machine does not fail politely,
  // and the code a log shows has to be the one a person would act on first.
  // 头文件声称"按枚举顺序排名，电流先于热、热先于运动"，而没有任何用例检查过：至今每个用例只动一个输入，于是
  // `active_mask` 与排名从未被激励。机器不会礼貌地只坏一处，而日志里出现的那个码必须是人会先处理的那个。
  core::Protector protector(config());

  core::ProtectionInput both = healthy();
  both.i_magnitude_a = 4.5F;  // OverCurrent, index 1
  both.v_bus_v = 31.0F;       // OverVoltage, index 2
  const core::ProtectionOutput over = protector.step(both, 1e-4F);
  const auto bit = [](core::FaultCode code) { return static_cast<std::uint16_t>(1U << static_cast<std::uint8_t>(code)); };
  std::printf("[P6-f] current+voltage -> fault %s, mask 0x%03x\n", fault_name(over.fault),
              static_cast<unsigned>(over.active_mask));
  EXPECT_EQ(core::FaultCode::OverCurrent, over.fault) << "the lower index wins";
  EXPECT_EQ(both_bits(bit(core::FaultCode::OverCurrent), bit(core::FaultCode::OverVoltage)), over.active_mask)
      << "both faults are still visible, even though only one is reported";

  // The same test with the pair the other way round in the enum: under-voltage (2) against over-temperature (3).
  // 同样的检验换一对枚举位置：欠压（2）对过温（3）。
  core::Protector second(config());
  core::ProtectionInput pair = healthy();
  pair.v_bus_v = 17.0F;
  pair.temperature_c = 105.0F;
  const core::ProtectionOutput second_out = second.step(pair, 1e-4F);
  std::printf("[P6-f] voltage+heat   -> fault %s, mask 0x%03x\n", fault_name(second_out.fault),
              static_cast<unsigned>(second_out.active_mask));
  EXPECT_EQ(core::FaultCode::UnderVoltage, second_out.fault);
  EXPECT_EQ(both_bits(bit(core::FaultCode::UnderVoltage), bit(core::FaultCode::OverTemperature)),
            second_out.active_mask);
}


TEST(Protection, TheThermalBudgetCoolsAndTheStallNeedsItsDuration) {
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

/**
 * @brief [en] A bench that can be tripped: the whole motor, a plant, and a handle on the PWM enable. The nominal
 *             machine with its shaft held, so a current decay can be attributed to `L/R` alone. Letting it spin adds
 *             the back-EMF to the braking and makes the decay faster than `L/R` -- which is true, and is the second
 *             half of what this test is about.
 * @brief [zh] 可以被跳闸的台架：完整电机、plant，以及一个 PWM 使能的把手。标称机 + 锁住的轴，于是电流衰减可以只归因于
 *             `L/R`。让它转起来会把反电动势加进制动， 衰减就比 `L/R` 快——那是真的，也正是本测试后半段要说的事。
 */
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
     OnAPlantTheCommandDiesInOnePeriodAndTheCurrentTakesLOverR) {
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

/**
 * @brief [en] An invalid flux trips `LostSync` on its own, with the error ratio comfortably inside its limit -- the
 *             boundary pair MC/DC asked for, and a real hole in the behaviour.
 *
 * @brief [zh] 磁链无效本身就触发 `LostSync`，而误差比舒服地落在限值之内——MC/DC 要的那条边界对，也是一个真实的行为空洞。
 *
 *     The decision is `(!input.flux_valid) || (abs(flux_error_ratio) > sync_error_ratio)`. Every existing case feeds
 *     `flux_valid = true`, so the first term has never been true and its independent effect was never shown: the
 *     short-circuit means a machine that says "my flux estimate is not valid" reaches `LostSync` **without any error
 *     ratio at all**, and nothing tested that. The pair is therefore the boundary one -- the same input twice,
 *     differing only in `flux_valid`, with the ratio held small in both -- so the first term is the only thing that can
 *     change the answer.
 *
 *     那个判定是 `(!input.flux_valid) || (abs(flux_error_ratio) > sync_error_ratio)`。既有用例全部喂 `flux_valid =
 *     true`，于是第一项从未为真、它的独立作用从未被演示：短路意味着一台说"我的磁链估计无效" 的机器**根本不需要误差比**
 *     就会落到 `LostSync`，而这件事没有任何用例测过。所以这一对是边界对——同一个输入跑两次，只差 `flux_valid`，两次的误
 *     差比都很小——于是第一项是唯一能改变答案的东西。
 */
TEST(Protection, AnInvalidFluxTripsLostSyncWithoutAnyErrorRatio) {
  foc::core::Protector protection(config());
  foc::core::ProtectionInput input = healthy();
  // The ratio is already tiny (0.02 against a limit well above it), so the second term cannot fire.
  // 误差比本就已经很小（0.02，远低于限值），所以第二项不可能触发。
  ASSERT_TRUE(input.flux_valid);
  ASSERT_LT(std::fabs(input.flux_error_ratio), config().sync_error_ratio);

  EXPECT_EQ(foc::core::FaultCode::None, protection.step(input, 1.0e-3F).fault)
      << "the healthy input must not trip anything, or the pair is meaningless";

  input.flux_valid = false;
  const foc::core::ProtectionOutput verdict = protection.step(input, 1.0e-3F);
  EXPECT_EQ(foc::core::FaultCode::LostSync, verdict.fault)
      << "an invalid flux has to reach LostSync on the first term alone";
  EXPECT_LT(std::fabs(input.flux_error_ratio), config().sync_error_ratio)
      << "and the second term must still be false, or this proves nothing about the first";
}

/**
 * @brief [en] A configuration whose clearing thresholds are exactly representable, so "just below the band" is a place
 *             the test can actually stand.
 *
 * @brief [zh] 清除阈值可精确表示的配置，于是"刚好在带内/带外"是测试真能站住的位置。
 *
 *     Mutation found this: eleven mutants in the four clearing expressions survived, all of them arithmetic on `x*(1
 *     -/+ h)`. The reason was not a weak assertion but an absent one -- no case had ever made a tripped fault *stop*
 *     being tripped, so the whole clearing half of `keep()` was unmeasured. The header claims hysteresis as one of the
 *     three things this block does ("a bus hovering on the threshold would otherwise chatter"), and the claim had no
 *     test. The thresholds here are dyadic (`128*0.5 = 64`) so the boundary is a number a test can name exactly; with
 *     the shipped `t_trip = 100, h = 0.05` the product is 94.999999... and no assertion can stand on it.
 *
 *     变异测试发现的：四条清除表达式里的十一个变异体全部存活，全是 `x*(1 ∓ h)` 上的算术。原因不是断言写弱了，是根本没写
 *     ——没有任何用例让已跳闸的故障**停止**跳闸，于是 `keep()` 的清除那一半整体未被测量。头文件把迟滞列为本块的三个特性
 *     之一（"母线悬在阈值上会反复抖"），而这个主张没有用例。这里的阈值取二进制可精确表示的值（`128*0.5 = 64`），边界才
 *     是测试能精确指定的数；若用出厂值 `t_trip = 100, h = 0.05`，乘积是 94.999999…，没有断言能站在上面。
 */
auto band_config() -> core::ProtectionConfig {
  core::ProtectionConfig band = config();
  band.t_trip_c = 128.0F;   // 128*(1-0.5) = 64 exactly
  band.v_max_v = 32.0F;     //  32*(1-0.5) = 16 exactly
  band.v_min_v = 8.0F;      //   8*(1+0.5) = 12 exactly
  band.hysteresis_ratio = 0.5F;
  return band;
}

/**
 * @brief [en] The clear is strictly below the clearing threshold, and each self-clearing mechanism has its own.
 * @brief [zh] 清除严格发生在清除阈值之下，且每个自恢复机理各有自己的那一个。
 */
TEST(Protection, TheClearingBandIsWhereTheFaultStopsBeingReported) {
  // Over-temperature: trip at 129, sit exactly on 128*(1-0.5) = 64, then step below it.
  {
    core::Protector protector(band_config());
    core::ProtectionInput input = healthy();
    input.temperature_c = 129.0F;
    EXPECT_EQ(core::FaultCode::OverTemperature, protector.step(input, kDt).fault);
    input.temperature_c = 64.0F;
    EXPECT_EQ(core::FaultCode::OverTemperature, protector.step(input, kDt).fault)
        << "exactly on the clearing threshold is still inside the band: the clear is strict, so the fault stays";
    input.temperature_c = 63.0F;
    EXPECT_EQ(core::FaultCode::None, protector.step(input, kDt).fault) << "one step below it clears";
  }
  // Over-voltage: 32*(1-0.5) = 16, and 15 must not fall foul of the under-voltage threshold (8).
  {
    core::Protector protector(band_config());
    core::ProtectionInput input = healthy();
    input.v_bus_v = 33.0F;
    EXPECT_EQ(core::FaultCode::OverVoltage, protector.step(input, kDt).fault);
    input.v_bus_v = 16.0F;
    EXPECT_EQ(core::FaultCode::OverVoltage, protector.step(input, kDt).fault);
    input.v_bus_v = 15.0F;
    EXPECT_EQ(core::FaultCode::None, protector.step(input, kDt).fault);
  }
  // Under-voltage clears *upwards*: 8*(1+0.5) = 12, and 13 is still below 32 so nothing else trips.
  {
    core::Protector protector(band_config());
    core::ProtectionInput input = healthy();
    input.v_bus_v = 7.0F;
    EXPECT_EQ(core::FaultCode::UnderVoltage, protector.step(input, kDt).fault);
    input.v_bus_v = 12.0F;
    EXPECT_EQ(core::FaultCode::UnderVoltage, protector.step(input, kDt).fault);
    input.v_bus_v = 13.0F;
    EXPECT_EQ(core::FaultCode::None, protector.step(input, kDt).fault);
  }
}

/**
 * @brief [en] Every trip threshold is strict: the limit itself does not trip, and neither does the stall timer at
 *             exactly its own duration.
 *
 * @brief [zh] 每个跳闸阈值都是严格的：限值本身不跳，堵转计时恰好等于时长也不跳。
 *
 *     Six `>`/`<` mutants survived by being `>=`/`<=` instead, because no case had ever put an input *exactly* on a
 *     threshold. That is a specification question, not a tolerance: "does the protection fire at 4.0 A when the hard
 *     limit is 4.0 A" has one right answer, and the code answers "no". The stall timer is the interesting one -- it is
 *     a float32 accumulation, so the test uses `1/64 s` steps, which are exact, and can therefore stand on 0.5 exactly
 *     after 32 of them.
 *
 *     六个 `>`/`<` 变异体改成 `>=`/`<=` 后存活，因为没有用例把输入放在阈值**正上**。这是规格问题不是容差： "硬限 4.0 A
 *     时 4.0 A 跳不跳"只有一个正确答案，代码答"不跳"。堵转计时更有意思——它是 float32 累加，所以用例用 `1/64 s` 的步长（
 *     可精确表示），于是能精确站在 32 步之后的 0.5 上。
 */
TEST(Protection, EachTripThresholdIsStrictAndTheLimitItselfDoesNotTrip) {
  constexpr float kExactDt = 1.0F / 64.0F;
  auto fault_of = [](auto prepare, int steps, float dt) {
    core::Protector protector(config());
    core::ProtectionInput input = healthy();
    prepare(input);
    core::ProtectionOutput out{};
    for (int k = 0; k < steps; ++k) {
      out = protector.step(input, dt);
    }
    return out.fault;
  };

  EXPECT_EQ(core::FaultCode::None,
            fault_of([](core::ProtectionInput &i) { i.temperature_c = 100.0F; }, 1, kDt))
      << "100 C against a 100 C trip is not above it";
  EXPECT_EQ(core::FaultCode::None, fault_of([](core::ProtectionInput &i) { i.v_bus_v = 30.0F; }, 1, kDt))
      << "30 V against a 30 V ceiling is not above it";
  EXPECT_EQ(core::FaultCode::None, fault_of([](core::ProtectionInput &i) { i.v_bus_v = 18.0F; }, 1, kDt))
      << "18 V against an 18 V floor is not below it";
  EXPECT_EQ(core::FaultCode::None, fault_of([](core::ProtectionInput &i) { i.i_magnitude_a = 4.0F; }, 1, kDt))
      << "4.0 A against a 4.0 A hard limit is not above it";
  EXPECT_EQ(core::FaultCode::None,
            fault_of([](core::ProtectionInput &i) { i.flux_error_ratio = 0.25F; }, 1, kDt))
      << "a ratio equal to its limit is not above it";
  // Stall: `|i|` exactly at the stall current, and `|omega|` exactly at the stall speed, are both inside the
  // condition "above the current AND below the speed" -- neither may start the timer.
  EXPECT_EQ(core::FaultCode::None,
            fault_of([](core::ProtectionInput &i) { i.i_magnitude_a = 1.0F; i.omega_e_rad_s = 0.0F; }, 40, kExactDt))
      << "an exact-1.0 A current is not above the 1.0 A stall current";
  EXPECT_EQ(core::FaultCode::None,
            fault_of([](core::ProtectionInput &i) { i.i_magnitude_a = 2.0F; i.omega_e_rad_s = 5.0F; }, 40, kExactDt))
      << "an exact-5.0 rad/s speed is not below the 5.0 rad/s stall speed";
  // And the timer itself: 32 steps of exactly 1/64 s make exactly 0.5 s, which is not *above* 0.5 s.
  EXPECT_EQ(core::FaultCode::None,
            fault_of([](core::ProtectionInput &i) { i.i_magnitude_a = 2.0F; i.omega_e_rad_s = 0.0F; }, 32, kExactDt))
      << "exactly the stall duration is not above it";
  EXPECT_EQ(core::FaultCode::Stall,
            fault_of([](core::ProtectionInput &i) { i.i_magnitude_a = 2.0F; i.omega_e_rad_s = 0.0F; }, 33, kExactDt))
      << "one step past it is";
}

/**
 * @brief [en] `latched_mask` is a different promise from `active_mask`, and nothing was checking it.
 *
 * @brief [zh] `latched_mask` 与 `active_mask` 是两个不同的承诺，而此前没人检查过它。
 *
 *     Two mutants survived here and they are the whole of the second mask: `latched_mask = 0` turned into `= 1` on a
 *     healthy step, and the bit-OR `1U << code` doubled into `2U << code`. Both survived because every case reads
 *     `fault` or `active_mask`, so the field a caller uses to decide "this one will not clear on its own" was written
 *     and never read.
 *
 *     这里有两个变异体存活，而它们覆盖了第二个掩码的全部：健康步上 `latched_mask = 0` 被改成 `= 1`，以及位或 `1U <<
 *     code` 被翻倍成 `2U << code`。两者都存活，因为所有用例只读 `fault` 或 `active_mask`——调用方用来判断"这一条不会自己
 *     好"的字段，写了但从来没人读。
 */
TEST(Protection, TheLatchedMaskSaysWhichFaultsWillNotClearOnTheirOwn) {
  core::Protector healthy_step(config());
  const core::ProtectionOutput quiet = healthy_step.step(healthy(), kDt);
  EXPECT_EQ(core::FaultCode::None, quiet.fault);
  EXPECT_EQ(static_cast<std::uint16_t>(0U), quiet.active_mask) << "a healthy step has nothing active";
  EXPECT_EQ(static_cast<std::uint16_t>(0U), quiet.latched_mask) << "and nothing latched";
  EXPECT_TRUE(quiet.output_allowed);

  core::Protector over_current(config());
  const core::ProtectionOutput latched = over_current.step(perturbed(core::FaultCode::OverCurrent), kDt);
  EXPECT_EQ(core::FaultCode::OverCurrent, latched.fault);
  EXPECT_EQ(static_cast<std::uint16_t>(1U << 1), latched.active_mask);
  EXPECT_EQ(static_cast<std::uint16_t>(1U << 1), latched.latched_mask)
      << "over-current is one of the faults that will not clear on its own";

  core::Protector over_temperature(config());
  const core::ProtectionOutput self_clearing =
      over_temperature.step(perturbed(core::FaultCode::OverTemperature), kDt);
  EXPECT_EQ(core::FaultCode::OverTemperature, self_clearing.fault);
  EXPECT_EQ(static_cast<std::uint16_t>(1U << 4), self_clearing.active_mask);
  EXPECT_EQ(static_cast<std::uint16_t>(0U), self_clearing.latched_mask)
      << "over-temperature is in the hysteresis set, so it is active and not latched";
}

/**
 * @brief [en] A zero cooling constant means "no cooling", not "cool infinitely": the guard that says so had no test.
 *
 * @brief [zh] 冷却常数为零意味着"不冷却"，不是"无限冷却"：那个说这件事的守卫没有用例。
 *
 *     The guard is `(i2t_cooling_s > 0) ? q*(dt/i2t_cooling_s) : 0`, and the mutant makes it `>=`, which turns `dt / 0`
 *     into `inf` and empties the budget every step. Nothing caught it because every configuration in the suite supplies
 *     a positive thermal constant, so the branch that exists *for* the zero case was never taken. With no cooling the
 *     budget is exactly `i^2 * t`: 3 A for 32 steps of 1/64 s is 4.5 A^2*s -- the budget itself -- and the thirty-third
 *     step is the first that spends past it.
 *
 *     守卫是 `(i2t_cooling_s > 0) ? q*(dt/i2t_cooling_s) : 0`，变异体把它改成 `>=`，于是 `dt / 0` 变成 `inf`，每步把预
 *     算清空。没人抓到，因为套件里所有配置都给了正的散热常数，那个**为**零值而存在的分支从未走到。不冷却时预算正好是
 *     `i²·t`：3 A 走 32 个 1/64 s 步恰为 4.5 A²·s（预算本身），第 33 步才是第一步花过头。
 */
TEST(Protection, AZeroCoolingConstantMeansTheBudgetOnlyAccumulates) {
  core::ProtectionConfig no_cooling = config();
  no_cooling.i2t_cooling_s = 0.0F;
  core::Protector protector(no_cooling);
  core::ProtectionInput input = healthy();
  input.i_magnitude_a = 3.0F;
  constexpr float kExactDt = 1.0F / 64.0F;

  core::ProtectionOutput out{};
  for (int k = 0; k < 32; ++k) {
    out = protector.step(input, kExactDt);
  }
  EXPECT_EQ(core::FaultCode::None, out.fault) << "32 steps of 3 A is exactly the 4.5 A^2*s budget, not past it";
  out = protector.step(input, kExactDt);
  EXPECT_EQ(core::FaultCode::I2t, out.fault) << "the thirty-third step spends past it, which only works if the budget "
                                               "kept accumulating instead of being emptied by a division by zero";
}

/**
 * @brief [en] The `I2t` clear is hysteresis too, and its band is in A^2*s: a spent budget that is decaying must stay
 *             spent until it is *below* `budget*(1-h)`.
 *
 * @brief [zh] `I2t` 的清除也是迟滞，只是它的带在 A²·s 里：正在衰减的已花预算必须一直算"已花"，直到它**低于**
 *             `budget*(1-h)`。
 *
 *     Two more mutants survived here -- the `*` turned into `/` and the `-` into `+` in `budget*(1-h)` -- because the
 *     only `I2t` case in the suite drives the budget *up* to a trip and stops. The decay half was never run. The
 *     charging current has to be **above** `budget/tau` (3 A only approaches the 4.5 A^2*s budget asymptotically and
 *     never crosses it -- the first version of this case made that mistake and failed on its own trip assertion), so
 *     the charge is 4 A, which reaches 4.9 in 30 steps. With `budget = 4.5` and `h = 0.5` the band is 2.25..4.5, while
 *     the mutant thresholds are 9 and 6.75 -- both of which clear while the budget is still well inside the band, so
 *     one step in the middle tells them apart.
 *
 *     这里又有两个变异体存活——`budget*(1-h)` 里的 `*` 变 `/`、`-` 变 `+`——因为套件里唯一的 `I2t` 用例只把预算 **推上去
 *     **跳到闸就停了，衰减那一半从未跑过。充电电流必须**高于** `budget/tau`（3 A 只会渐近逼近 4.5 A²·s 而永远不越过——这
 *     条用例的第一版就犯了这个错，被它自己的跳闸断言挡住了），所以用 4 A，30 步到 4.9。 `budget = 4.5`、`h = 0.5` 时带
 *     是 2.25..4.5，而变异的阈值是 9 与 6.75——两者都会在预算还稳稳落在带内时就清除，所以带中间走一步就能分开它们。
 */
TEST(Protection, TheSpentBudgetStaysSpentUntilItIsBelowTheBand) {
  constexpr float kExactDt = 1.0F / 64.0F;
  core::ProtectionConfig wide_band = config();
  wide_band.hysteresis_ratio = 0.5F; // the shipped 0.05 gives a band 4.275..4.5, too narrow to stand in
  core::Protector protector(wide_band);
  core::ProtectionInput charging = healthy();
  charging.i_magnitude_a = 4.0F; // above the 3 A equilibrium current (i^2 > budget/tau), and not above the 4.0 A hard limit
  for (int k = 0; k < 30; ++k) {
    protector.step(charging, kExactDt); // q crosses 4.5 at step 27 and reaches about 4.9
  }

  core::ProtectionInput idle = healthy();
  idle.i_magnitude_a = 0.0F;
  core::ProtectionOutput out{};
  for (int k = 0; k < 10; ++k) {
    out = protector.step(idle, kExactDt); // q decays to about 3.6, comfortably inside the 2.25..4.5 band
  }
  EXPECT_EQ(core::FaultCode::I2t, out.fault)
      << "inside the clearing band the budget is still spent: I2t is active and not latching, so this is the hysteresis";

  for (int k = 0; k < 17; ++k) {
    out = protector.step(idle, kExactDt); // q decays to about 2.1, below 4.5*0.5 = 2.25
  }
  EXPECT_EQ(core::FaultCode::None, out.fault) << "below the band it clears";
}

/**
 * @brief [en] Past the end of the derating ramp the limit is exactly zero -- not slightly negative.
 *
 * @brief [zh] 越过降额斜坡的终点后，限幅**恰好**为零，而不是一个很小的负数。
 *
 *     `used >= 1` returns 0 and the next line would return `i_max*(1-used)`, which is negative for any `used` just
 *     above 1. The mutant moves the comparison to `> 1.001`, so a temperature between the two hands the caller a small
 *     negative current limit -- a limit that a caller clamping with it turns into a *reversed* command. The path is
 *     unreachable through `step()` (the trip fires first at `t_trip < t_derate_end` in every sane config), which is
 *     exactly why it needs a case that calls `derated_limit()` directly.
 *
 *     `used >= 1` 返回 0，而下一行会返回 `i_max*(1-used)`——对任何略大于 1 的 `used` 都是负数。变异体把比较挪到 `>
 *     1.001`，于是落在两者之间的温度会给调用方一个很小的**负**限幅；调用方拿它做钳位，就把指令钳成了反向。这条路径从
 *     `step()` 走不到（任何合理配置下 `t_trip < t_derate_end`，先跳闸），所以它需要一条直接调用 `derated_limit()` 的用
 *     例。
 */
TEST(Protection, PastTheEndOfTheDeratingRampTheLimitIsExactlyZero) {
  const core::Protector protector(config()); // derating runs 60 C -> 110 C
  EXPECT_FLOAT_EQ(config().i_max_a, protector.derated_limit(60.0F)) << "at the start of the ramp the full limit";
  EXPECT_FLOAT_EQ(0.0F, protector.derated_limit(110.0F)) << "at the end of the ramp nothing";
  // Swept rather than sampled at one magic temperature: the claim is that the limit is zero *everywhere* past the
  // end, so the case walks the first tenth of a degree in 5 mK steps. A 0.1 % shift of the `used >= 1` comparison
  // turns into a small negative limit at 110.02 C, which is inside this sweep and outside a test that only probes
  // 110.5.
  // 这里是扫而不是只取一个魔法温度：主张是越过终点后**处处**为零，所以用例把最初 0.1 度按 5 mK 走一遍。
  // 把 `used >= 1` 挪动 0.1 % 后，110.02 C 处会出现一个很小的负限幅——它落在这个扫的范围里，却跑得出只探 110.5 的用例。
  for (int k = 0; k <= 20; ++k) {
    const float temperature_c = 110.0F + (0.005F * static_cast<float>(k));
    EXPECT_FLOAT_EQ(0.0F, protector.derated_limit(temperature_c))
        << "past the end of the ramp at " << temperature_c << " C the limit is zero, never negative";
  }
  EXPECT_FLOAT_EQ(0.0F, protector.derated_limit(400.0F)) << "nor far past it";
}

/**
 * @brief [en] `reset()` clears what latched and keeps the thermal memory; `reset_cold()` is the one that also forgets
 *             the heat.
 *
 * @brief [zh] `reset()` 清掉锁存的、保留热记忆；`reset_cold()` 才是连热度一起忘掉的那个。
 *
 *     Under AddressSanitizer the two loops in `reset()`/`reset_cold()` were the only survivors left that no arithmetic
 *     could explain, and the reason was not subtlety: **nothing in this module's tests ever called them**. They are
 *     reached from the C ABI (`bridge.cpp` calls `reset_cold()`), so line coverage was satisfied and the claim was
 *     never checked -- an operator action whose entire contract ("clear the latched faults", "`I2t` is *not* cleared
 *     here, the winding is still hot") had no direct witness.
 *
 *     在 AddressSanitizer 下，`reset()`/`reset_cold()` 里那两个循环是最后一批算术解释不了的存活体，而原因并不微妙：**本
 *     模块的用例从未调用过它们**。它们从 C ABI 走得到（`bridge.cpp` 调 `reset_cold()`），所以行覆盖率是满足的，而主张从
 *     未被检查——一个运维动作的完整契约（"清掉锁存的故障"、"这里**不**清 `I2t`，绕组还热着"）没有直接证词。
 */
TEST(Protection, ResetClearsTheLatchedOnesButKeepsTheHeatUntilResetCold) {
  core::Protector protector(config());
  core::ProtectionInput over_current = healthy();
  over_current.i_magnitude_a = 4.5F;
  core::ProtectionOutput out{};
  for (int k = 0; k < 10; ++k) {
    out = protector.step(over_current, kDt);
  }
  ASSERT_EQ(core::FaultCode::OverCurrent, out.fault);
  ASSERT_NE(static_cast<std::uint16_t>(0U), out.latched_mask) << "over-current is a latched fault";
  const float spent = protector.thermal_budget();
  ASSERT_GT(spent, 0.0F) << "the overload spent thermal budget, which is the point of the next assertion";

  protector.reset();
  EXPECT_FLOAT_EQ(spent, protector.thermal_budget())
      << "reset() must not clear the thermal budget: the winding is still hot";

  const core::ProtectionOutput after = protector.step(healthy(), kDt);
  EXPECT_EQ(core::FaultCode::None, after.fault) << "the latched fault is gone";
  EXPECT_EQ(static_cast<std::uint16_t>(0U), after.latched_mask);

  protector.reset_cold();
  EXPECT_FLOAT_EQ(0.0F, protector.thermal_budget()) << "reset_cold() is the one that forgets the heat";
}

} // namespace foc
