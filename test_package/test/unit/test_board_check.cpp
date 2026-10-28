// Conan::ImportStart
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

#include <foc/core/board_check.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>

#include "sil_bench.hpp"
#include "sil_discrete_inverter.hpp"
#include "sil_plant_hal.hpp"
// Conan::ImportEnd

namespace foc {

namespace {

namespace core = foc::core;

/// [en] A board that is wired correctly. Every check has to be green on this
/// one, or the suite cries wolf and the user learns to ignore it.
///      [zh]
///      接对的板子。所有检查在它上面必须全绿，否则整套检查会喊狼来了，用户就学会无视它了。
auto good_board() -> core::BoardMeasurements {
  core::BoardMeasurements m{};
  m.sum_current_a = 0.002F;
  m.offset_a[0] = 0.003F;
  m.offset_a[1] = -0.002F;
  m.offset_a[2] = 0.001F;
  m.response_sign[0] = 1.0F;  // +i on u
  m.response_sign[1] = -1.0F; // -i/2 on v
  m.response_sign[2] = -1.0F; // -i/2 on w
  m.ripple_measured_a = 0.30F;
  m.ripple_expected_a = 0.375F; // v_bus/(8*L*f_sw) at 24 V, 0.4 mH, 20 kHz
  m.deadtime_v = 0.641F;
  m.deadtime_cmd_v = 0.640F;
  m.encoder_offset_rad = 0.008F;
  m.rs_ohm = 0.5015F;
  m.rs_nameplate_ohm = 0.5F;
  m.psi_f_wb = 4.79e-3F;
  m.psi_f_nameplate_wb = 4.762e-3F;
  m.observer_error_rad = 0.05F;
  return m;
}

auto item_name(core::BoardItem item) -> const char * {
  switch (item) {
  case core::BoardItem::CurrentSum:
    return "sum(i_u,i_v,i_w)";
  case core::BoardItem::CurrentOffset:
    return "zero offset";
  case core::BoardItem::CurrentPolarity:
    return "polarity";
  case core::BoardItem::SamplingInstant:
    return "ripple ratio";
  case core::BoardItem::DeadTime:
    return "dead time [V]";
  case core::BoardItem::EncoderOffset:
    return "encoder [rad]";
  case core::BoardItem::Parameters:
    return "parameter";
  case core::BoardItem::ObserverHealth:
    return "observer [rad]";
  }
  return "?";
}

/// [en] Print one report: every row with its number, which is criterion P7-c.
///      [zh] 打印一份报告：每一行带它的数，这就是 P7-c 的判据。
void print_report(const char *label, const core::BoardReport &report) {
  std::printf("\n[P7] %s: %d row(s), %d failure(s), cause = %s", label,
              report.count, report.failures,
              core::board_cause_name(report.cause));
  if (report.cause_phase < 3) {
    std::printf(" (phase %c)", "uvw"[report.cause_phase]);
  }
  std::printf("\n");
  for (int k = 0; k < report.count; ++k) {
    const core::BoardReading &row = report.items[k];
    std::printf("        %-16s %12.5f vs %12.5f  %s\n", item_name(row.item),
                static_cast<double>(row.value), static_cast<double>(row.limit),
                row.pass ? "ok" : "FAIL");
  }
}

} // namespace

TEST(BoardCheck, ACorrectBoardProducesNoComplaintAndStillCarriesNumbers) {
  const core::BoardReport report =
      core::check_board(good_board(), core::BoardLimits{});
  print_report("correct board", report);

  // P7-b: zero false positives. A consistency suite that complains about a good
  // board is worse than no suite, because it teaches the user to ignore it.
  // P7-b：零误报。对一块好板子也会抱怨的一致性套件比没有更糟，因为它教用户去无视它。
  EXPECT_TRUE(report.all_green());
  EXPECT_EQ(0, report.failures);
  EXPECT_EQ(core::BoardCause::None, report.cause);
  // P7-c: and every row carries a number, not just a verdict.
  // P7-c：每一行都带一个数，不只是一个判定。
  EXPECT_GE(report.count, 10);
  for (int k = 0; k < report.count; ++k) {
    EXPECT_TRUE(std::isfinite(report.items[k].value))
        << item_name(report.items[k].item);
    EXPECT_NE(0.0F, report.items[k].limit) << item_name(report.items[k].item);
  }
}

TEST(BoardCheck,
     AReversedChannelIsNamedAsAPolarityFaultAndNotAsThreeSeparateOnes) {
  // A channel whose sign is inverted. The physics of the signature: a positive
  // d-axis drive puts
  // `(+i, -i/2, -i/2)` on the phases, and negating one of them turns the sum
  // from zero into `-2i`
  // -- the sum grows instead of cancelling. It also makes the resistance fit's
  // slope negative, so the dead-time identification has nothing to report.
  // 一条符号被取反的通道。这个特征的物理：正 d 轴驱动在三相上给 `(+i, −i/2,
  // −i/2)`，把其中一相 取反会让"和"从零变成
  // `−2i`——和是变大而不是抵消。它同时让电阻拟合的斜率变负，于是死区辨识
  // 什么都报不出来。
  core::BoardMeasurements m = good_board();
  m.response_sign[1] = +1.0F; // inverted
  m.offset_a[1] = +0.002F;    // an inversion mirrors the offset too
  m.sum_current_a = -2.0F;    // -(2 A) on v, so the sum no longer cancels
  m.deadtime_v = 0.0F;        // the fit refused: slope was negative

  const core::BoardReport report = core::check_board(m, core::BoardLimits{});
  print_report("phase v inverted", report);

  // P7-a: named, and named as *one* thing.
  // P7-a：被指名，而且是指名为**一件**事。
  EXPECT_EQ(core::BoardCause::PolarityReversed, report.cause);
  EXPECT_EQ(1, report.cause_phase) << "the report has to say which phase";
  // Three items failed but only one cause is reported -- that is the point of
  // the whole file. 三项失败了，但只报一个成因——这正是本文件的全部意义。
  EXPECT_GE(report.failures, 3);
  EXPECT_FALSE(report.all_green());
}

TEST(BoardCheck,
     ASamplingInstantFaultShowsUpAsRippleAndIsNotBlamedOnTheDeadTime) {
  // The ADC sampling somewhere other than the centre of the zero vector: it
  // then reads the ripple instead of the average. The number is the ratio of
  // measured to predicted peak-to-peak, which is the only threshold that
  // survives a change of bus voltage or inductance. ADC
  // 采样点不在零矢量中心：它读到的是纹波而不是均值。那个数是实测峰峰值对预测峰峰值的比值，
  // 是在母线电压或电感变化下唯一还能用的阈值。
  core::BoardMeasurements m = good_board();
  m.ripple_measured_a = 0.90F; // 2.4x what the switching period implies
  m.sum_current_a = 0.08F;     // off-centre sampling also biases the mean
  m.deadtime_v = 0.90F;        // ...and makes the dead-time fit read high

  const core::BoardReport report = core::check_board(m, core::BoardLimits{});
  print_report("sampling off centre", report);

  EXPECT_EQ(core::BoardCause::SamplingInstant, report.cause);
  // Two other items are failing (the sum, and the dead time) -- and neither is
  // the cause. 另外两项也在失败（和、死区）——而它们都不是成因。
  EXPECT_GE(report.failures, 3);
  EXPECT_EQ(3, report.cause_phase) << "this cause is not per-phase";
}

TEST(BoardCheck, AnOffsetIsAlsoWhatBreaksTheAlignmentAndTheReportSaysSo) {
  // An offset on every channel. Two items fail for the *same* reason, and the
  // second one is not obvious: the aligner forces the current vector onto a
  // known angle, and an offset tilts it by `atan(offset/pull)` = 0.12 rad on a
  // 1 A pull -- six times the alignment gate. A report that listed "encoder
  // offset out of tolerance" on its own would send the user to the encoder,
  // which is the one thing that is not wrong.
  // 三条通道都有零漂。两项因为**同一个**原因失败，而第二项并不显然：对齐器把电流矢量强制到已知
  // 角度上，零漂把它按 `atan(offset/pull)` 偏 0.12 rad（1 A
  // 拉力下）——是对齐闸的六倍。单独报
  // "编码器偏置超差"会把用户支到编码器那里，而那恰恰是唯一没坏的东西。
  core::BoardMeasurements m = good_board();
  m.offset_a[0] = 0.120F;
  m.offset_a[1] = 0.090F;
  m.offset_a[2] = -0.060F;
  m.sum_current_a = 0.150F;                       // KCL: the offsets add up
  m.encoder_offset_rad = std::atan2(0.12F, 1.0F); // the aligner's tilt

  const core::BoardReport report = core::check_board(m, core::BoardLimits{});
  print_report("offsets on all three channels", report);

  EXPECT_EQ(core::BoardCause::OffsetBias, report.cause);
  EXPECT_EQ(0, report.cause_phase) << "the worst channel is u";
  // Both the sum and the encoder row failed, and the cause explains both.
  // "和"和编码器两行都失败了，而这个成因同时解释了两者。
  EXPECT_GE(report.failures, 2);
}

TEST(BoardCheck, FourWrongBoardsFourDifferentNames) {
  // The summary P7-a asks for: four boards, four distinct names, no collisions.
  // One cause per board is only useful if different faults do not produce the
  // same answer. P7-a
  // 要的那个汇总：四块板、四个不同的名字、没有撞车。一块一个成因，只有在不同故障不会给出同一个
  // 答案时才有用。
  core::BoardMeasurements correct = good_board();
  core::BoardMeasurements inverted = good_board();
  inverted.response_sign[0] = -1.0F;
  inverted.sum_current_a = 2.0F;
  inverted.deadtime_v = 0.0F;
  core::BoardMeasurements sampling = good_board();
  sampling.ripple_measured_a = 0.90F;
  core::BoardMeasurements offsets = good_board();
  offsets.offset_a[0] = 0.12F;
  offsets.sum_current_a = 0.12F;
  core::BoardMeasurements wrong_machine = good_board();
  wrong_machine.rs_ohm =
      0.80F; // 60 % off the nameplate: the parameters are for another motor
  wrong_machine.psi_f_wb = 6.5e-3F;

  const core::BoardMeasurements boards[5] = {correct, inverted, sampling,
                                             offsets, wrong_machine};
  const core::BoardCause expected[5] = {
      core::BoardCause::None, core::BoardCause::PolarityReversed,
      core::BoardCause::SamplingInstant, core::BoardCause::OffsetBias,
      core::BoardCause::ParametersOff};
  const char *labels[5] = {"correct", "u inverted", "sampling", "offsets",
                           "wrong machine"};

  std::printf("\n[P7-a] five boards, five verdicts\n");
  int correct_names = 0;
  for (int k = 0; k < 5; ++k) {
    const core::BoardReport report =
        core::check_board(boards[k], core::BoardLimits{});
    const bool ok = (report.cause == expected[k]);
    correct_names += ok ? 1 : 0;
    std::printf("        %-14s -> %-18s %s\n", labels[k],
                core::board_cause_name(report.cause), ok ? "ok" : "WRONG");
    EXPECT_EQ(expected[k], report.cause) << labels[k];
  }
  EXPECT_EQ(5, correct_names)
      << "the names have to be exact, or an operator acts on the wrong part";
}

// ---------------------------------------------------------------------------
// The measurement half: a runnable sequence on a board that can be wired wrong.
// 测量半边：一个能跑的序列，对象是一块可以接错的板子。
// ---------------------------------------------------------------------------

namespace {

namespace sil = foc::sil;

/// [en] A standstill board that can be mis-wired the ways P7-a names. The
/// faults live in the
/// **sensor path** -- sign, offset -- because that is where they live on real
/// hardware: the power stage is fine, the reading of it is not. `read_uvw()`
/// returns the sample the ISR would see, taken at the trigger point, not "now".
///      [zh] 一块可以按 P7-a
///      点名的方式接错的堵转板子。故障在**传感器路径**——符号、零漂——因为真实
///      硬件上它们就在那里：功率级是好的，读数不是。`read_uvw()` 返回 ISR
///      会看到的那个采样，取自触发 点，而不是"现在"。
class FaultyBoard final {
public:
  struct Fault final {
    float sign[3] = {1.0F, 1.0F, 1.0F};
    float offset[3] = {0.0F, 0.0F, 0.0F};
  };

  FaultyBoard(const sil::PlantParams &params, Fault fault)
      : plant_(params),
        inverter_(sil::InverterConfig{20000.0F, 1.0e-6F, 0.0F, 40}),
        fault_(fault) {
    plant_.reset(sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
  }

  void set_dq(core::Dq<float> v) { command_ = v; }
  auto switching_period_s() const noexcept -> float { return kPeriod; }
  auto v_bus_v() const noexcept -> float { return kVBus; }

  void advance(float seconds) {
    const int periods = static_cast<int>((seconds / kPeriod) + 0.5F);
    for (int k = 0; k < periods; ++k) {
      run_period();
    }
  }

  /// [en] Less than a period; unused by the standstill sequence but part of the
  /// concept. [zh] 不足一个周期；堵转序列不用它，但概念里有。
  void step_advance(float seconds) {
    const float ratio = kPeriod / ((seconds > 0.0F) ? seconds : kPeriod);
    const int steps = (ratio >= 1.0F) ? static_cast<int>(ratio + 0.5F) : 1;
    run_period_steps(steps);
  }

  /// [en] Move the ADC trigger inside the switching period, 0..1. [zh]
  /// 把 ADC 触发点在开关周期内移动，0..1。
  void set_sample_delay(float fraction) {
    sample_fraction_ = (fraction < 0.0F) ? 0.0F : ((fraction > 1.0F) ? 1.0F : fraction);
  }
  [[nodiscard]] auto sample_delay() const noexcept -> float {
    return sample_fraction_;
  }

  auto read_uvw() const noexcept -> core::Uvw<float> {
    return core::Uvw<float>{
        static_cast<float>((fault_.sign[0] * held_.u) + fault_.offset[0]),
        static_cast<float>((fault_.sign[1] * held_.v) + fault_.offset[1]),
        static_cast<float>((fault_.sign[2] * held_.w) + fault_.offset[2])};
  }

private:
  static constexpr float kPeriod = 5.0e-5F;
  static constexpr float kVBus = 24.0F;

  void run_period() { run_period_steps(40); }

  void run_period_steps(int steps_wanted) {
    const float poles = static_cast<float>(plant_.params().pole_pairs);
    const core::Ab<float> command =
        core::inv_park(command_, poles * plant_.theta_mech_rad());
    const core::SvpwmResult modulated = core::Svpwm::modulate(
        core::ModulatorInput{command, kVBus, plant_.current_uvw()});
    const int steps = inverter_.begin_period(modulated.duty, kVBus, kPeriod);
    if (steps <= 0) {
      return;
    }
    const float h = inverter_.step_length_s();
    // The trigger: the ISR latches the current at `sample_fraction_` through the
    // period. That is what makes an off-centre trigger *measurable* -- moving it
    // moves the reading, and the reading is what the loop consumes.
    // 触发：ISR 在周期内 `sample_fraction_` 处锁存电流。这正是"偏心的触发点可以被量到"的原因
    // ——移动它读数就动，而读数正是环消费的东西。
    const int sample_index =
        static_cast<int>((sample_fraction_ * static_cast<float>(steps)) + 0.5F);
    for (int k = 0; k < steps; ++k) {
      if (k == sample_index) {
        held_ = plant_.current_uvw();
      }
      const core::Ab<float> applied = inverter_.step(plant_.current_uvw());
      plant_.set_voltage_dq(
          core::park(applied, poles * plant_.theta_mech_rad()));
      plant_.advance(h);
    }
    if (sample_index >= steps) {
      held_ = plant_.current_uvw();
    }
    static_cast<void>(steps_wanted);
  }

  sil::PmsmPlant plant_;
  sil::DiscreteInverter inverter_;
  Fault fault_{};
  float sample_fraction_{0.5F};
  core::Dq<float> command_{0.0F, 0.0F};
  core::Uvw<float> held_{0.0F, 0.0F, 0.0F};
};

auto self_test_config() -> core::BoardSelfTest::Config {
  core::BoardSelfTest::Config config{};
  config.probe_v[0] = 0.90F;
  config.probe_v[1] = 1.05F;
  config.probe_v[2] = 1.20F;
  config.probe_v[3] = 1.40F;
  config.probe_v[4] = 1.65F;
  config.probe_v[5] = 1.95F;
  config.settle_steps = 120;
  config.average_steps = 20;
  config.zero_steps = 20;
  config.ripple_steps = 1;
  config.delay_steps = 16;
  config.l_h = sil::nominal_spec().lq;
  config.i_min_a = 0.4F;
  return config;
}

/// [en] The numbers that need the rotor to turn, supplied by the caller: the
/// spin test and the aligner produce them, and this test is about the
/// standstill half.
///      [zh]
///      需要转子转起来才能得到的数，由调用方提供：旋转测试与对齐器产出它们，而本测试的对象是堵转
///      半边。
auto spinning_truth() -> core::BoardMeasurements {
  core::BoardMeasurements m{};
  const float psi = sil::nominal_spec().kt /
                    (1.5F * static_cast<float>(sil::nominal_spec().pole_pairs));
  m.psi_f_wb = psi;
  m.psi_f_nameplate_wb = psi;
  m.rs_nameplate_ohm = sil::nominal_spec().rs;
  // In the **axis** convention, because that is what the identification
  // measures: a per-phase step of `dV` projects onto the d axis as `(4/3)*dV`.
  // Comparing a per-phase number with an axis number is a 33 % error that looks
  // exactly like a bad board -- finding 17 of the plan, met again here.
  // 用**轴上**的约定，因为辨识量到的就是轴上的量：每相的 `dV` 投影到 d 轴是
  // `(4/3)·dV`。拿每相数 和轴上数比，是一个 33 %
  // 的误差，看起来和一块坏板子一模一样——plan 的发现 17 在这里又遇到一次。
  m.deadtime_cmd_v = core::axis_deadtime_v(1.0e-6F * 20000.0F * 24.0F, 0.0F);
  m.encoder_offset_rad = 0.005F;
  m.observer_error_rad = 0.04F;
  // The trigger sweep is a bench facility the four-call concept does not
  // abstract, so the self-test does not measure it; the caller supplies the
  // number a good board would produce.
  // 触发点扫描是四调用概念没有抽象的台架能力，所以自检不测它；这个数由调用方给出，取好板子会产出的值。
  m.ripple_measured_a = 0.30F;
  m.ripple_expected_a = 0.375F;
  return m;
}

} // namespace

TEST(BoardCheck, TheSelfTestMeasuresAndTheChecklistNamesTheCause) {
  // P7-a end to end: the sequence **measures**, `check_board` **judges**, and
  // the answer names the thing the operator has to touch. Three boards, one
  // `Fault` setting each. P7-a 端到端：序列**测量**，`check_board`
  // **判定**，答案点名运维要动手的那个东西。三块板子，各一个 `Fault` 设置。
  struct Case final {
    const char *label;
    FaultyBoard::Fault fault;
    core::BoardCause expected;
  };
  const Case cases[] = {
      {"good board", FaultyBoard::Fault{}, core::BoardCause::None},
      {"phase u inverted",
       FaultyBoard::Fault{{-1.0F, 1.0F, 1.0F}, {0.0F, 0.0F, 0.0F}},
       core::BoardCause::PolarityReversed},
      {"offset on u",
       FaultyBoard::Fault{{1.0F, 1.0F, 1.0F}, {0.15F, 0.0F, 0.0F}},
       core::BoardCause::OffsetBias},
  };

  std::printf("\n[P7-a/self-test] three boards, measured then judged\n");
  int named = 0;
  for (const Case &c : cases) {
    FaultyBoard board(sil::plant_params_from(sil::nominal_spec()), c.fault);
    core::BoardSelfTest self_test(self_test_config());
    const core::BoardSelfTest::Result run =
        self_test.run(board, spinning_truth());
    const core::BoardReport report =
        core::check_board(run.measurements, core::BoardLimits{});
    const bool ok = (report.cause == c.expected) && run.complete;
    named += ok ? 1 : 0;
    std::printf(
        "        %-17s sum %7.3f off %6.3f %6.3f %6.3f  rs %6.3f (n%d)  "
        "dt %6.3f  -> %-18s %s\n",
        c.label, static_cast<double>(run.measurements.sum_current_a),
        static_cast<double>(run.measurements.offset_a[0]),
        static_cast<double>(run.measurements.offset_a[1]),
        static_cast<double>(run.measurements.offset_a[2]),
        static_cast<double>(run.rs_ohm), static_cast<int>(run.points_used),
        static_cast<double>(run.measurements.deadtime_v),
        core::board_cause_name(report.cause), ok ? "ok" : "WRONG");
    EXPECT_EQ(c.expected, report.cause) << c.label;
    EXPECT_TRUE(run.complete) << c.label;
    EXPECT_GE(report.count, 10) << c.label;
  }
  EXPECT_EQ(3, named) << "measurement plus judgement has to name every case";
}


TEST(BoardCheck, TheSelfTestFindsWhereTheAdcTriggerSits) {
  // T4: the ripple check only has signal if the bench can move its trigger. With
  // a movable trigger the standstill sequence reports **where** the trigger is,
  // and what that position is worth in amps.
  // T4：只有当台架能移动触发点时，纹波检查才有信号。触发点可动之后，堵转序列就能报出触发点
  // **在哪**、以及那个位置值多少安培的误差。
  auto config = self_test_config();
  core::BoardSelfTest self_test(config);
  const core::BoardMeasurements spinning = spinning_truth();

  struct Case final {
    const char *label;
    float trigger;
  };
  const Case cases[] = {{"centre of period", 0.5F}, {"near the ripple peak", 0.75F}};
  std::printf("\n[T4] trigger sweep, %d points per switching period\n", config.delay_steps);
  std::printf("     %-22s %8s %8s %10s %10s\n", "trigger at", "reported", "best",
              "ripple A", "error A");
  float ripple[2] = {0.0F, 0.0F};
  float best[2] = {0.0F, 0.0F};
  float error[2] = {0.0F, 0.0F};
  int k = 0;
  for (const Case &c : cases) {
    FaultyBoard board(sil::nominal_plant_params(), FaultyBoard::Fault{});
    board.set_sample_delay(c.trigger);
    const auto result = self_test.run(board, spinning);
    ripple[k] = result.measurements.ripple_measured_a;
    best[k] = result.sample_delay_best_fraction;
    error[k] = result.sample_error_a;
    std::printf("     %-22s %8.3f %8.3f %10.4f %10.4f\n", c.label,
                static_cast<double>(result.sample_delay_fraction),
                static_cast<double>(result.sample_delay_best_fraction),
                static_cast<double>(ripple[k]), static_cast<double>(error[k]));

    // 1. The sweep ran, and it put the trigger back where it found it.
    // 1. 扫描跑了，并且把触发点放回了它原来在的地方。
    ASSERT_TRUE(result.sample_delay_measured) << c.label;
    EXPECT_NEAR(c.trigger, result.sample_delay_fraction, 1.0e-6F) << c.label;
    EXPECT_NEAR(c.trigger, board.sample_delay(), 1.0e-6F) << c.label;
    // 2. The ripple is a property of the drive, not of where the trigger happens
    //    to be -- a sweep that reported the setting back would be measuring
    //    nothing.
    // 2. 纹波是驱动的性质，不是触发点碰巧在哪的性质——把设置原样报回来的扫描等于什么都没量。
    EXPECT_GT(ripple[k], 0.0F) << c.label;
    EXPECT_LT(ripple[k], result.measurements.ripple_expected_a) << c.label;
    ++k;
  }

  // 3. Same ripple, different sampling error: that is the discrimination the
  //    item exists for. The optimum is the same for both boards, because it is a
  //    property of where the current crosses its period mean inside this drive --
  //    which is *not* necessarily the centre of the period, and is exactly why
  //    the self-test measures it instead of assuming 0.5.
  // 3. 同一个纹波、不同的采样误差：这就是该项存在的判别力。最优点对两块板相同，因为它是"在这
  //    个驱动下电流在哪里穿过周期均值"的性质——而那**不一定**是周期中心，这也正是自检要量它
  //    而不是假设 0.5 的原因。
  EXPECT_NEAR(ripple[0], ripple[1], 0.01F * ripple[0]);
  EXPECT_NEAR(best[0], best[1], 1.0e-6F);
  EXPECT_GT(error[1], 3.0F * error[0])
      << "the trigger near the ripple peak must read worse than the one at the "
         "period centre";
}

TEST(BoardCheck, TheSamplingErrorIsZeroAtTheCentreAndMaximalAtTheEdge) {
  // The number the position is worth, on the same board with nothing else
  // changed: what the ADC reads minus what the current really is. A flat plant
  // with a fine trigger grid is the only place this can be checked without
  // arguing about PWM alignment.
  // 位置值多少：在同一块板子上、别的都不动，就是"ADC 读到的减去电流真值"。只有在台架平直、
  // 触发网格足够细的条件下这件事才能在不争论 PWM 对齐的前提下被检查。
  auto config = self_test_config();
  core::BoardSelfTest self_test(config);
  const core::BoardMeasurements spinning = spinning_truth();

  FaultyBoard centred(sil::nominal_plant_params(), FaultyBoard::Fault{});
  centred.set_sample_delay(0.5F);
  const auto at_centre = self_test.run(centred, spinning);

  FaultyBoard peak(sil::nominal_plant_params(), FaultyBoard::Fault{});
  peak.set_sample_delay(0.75F);
  const auto at_peak = self_test.run(peak, spinning);

  std::printf("[T4] sampling error: 0.5 -> %.4f A, 0.75 -> %.4f A (ripple p-p "
              "%.4f A)\n",
              static_cast<double>(at_centre.sample_error_a),
              static_cast<double>(at_peak.sample_error_a),
              static_cast<double>(at_centre.measurements.ripple_measured_a));
  // Both errors are bounded by the ripple itself -- the bookkeeping has to stay
  // inside physics even when the fault is at its worst.
  // 两个误差都被纹波本身界住——即使故障最严重，账也不能超出物理。
  EXPECT_LT(at_peak.sample_error_a,
            0.5F * at_centre.measurements.ripple_measured_a);
  EXPECT_GT(at_peak.sample_error_a, 4.0F * at_centre.sample_error_a);
}

TEST(BoardCheck, EveryItemAndCauseHasADistinctName) {
  // The 5-minute procedure dumps the report with these two functions, so a
  // duplicated name would make two different faults look like one line of output.
  // 5 分钟流程就是用这两个函数导出报告的，名字重复会让两个不同的故障看起来像同一行输出。
  const core::BoardItem items[] = {
      core::BoardItem::CurrentSum,       core::BoardItem::CurrentOffset,
      core::BoardItem::CurrentPolarity,  core::BoardItem::SamplingInstant,
      core::BoardItem::DeadTime,         core::BoardItem::EncoderOffset,
      core::BoardItem::Parameters,       core::BoardItem::ObserverHealth};
  const core::BoardCause causes[] = {
      core::BoardCause::None,            core::BoardCause::OffsetBias,
      core::BoardCause::PolarityReversed, core::BoardCause::SamplingInstant,
      core::BoardCause::DeadTimeMismatch, core::BoardCause::EncoderOffset,
      core::BoardCause::ParametersOff,   core::BoardCause::ObserverUnhealthy};
  for (const auto item : items) {
    ASSERT_NE(nullptr, core::board_item_name(item));
    for (const auto other : items) {
      if (item != other) {
        EXPECT_STRNE(core::board_item_name(item), core::board_item_name(other));
      }
    }
  }
  for (const auto cause : causes) {
    ASSERT_NE(nullptr, core::board_cause_name(cause));
    for (const auto other : causes) {
      if (cause != other) {
        EXPECT_STRNE(core::board_cause_name(cause), core::board_cause_name(other));
      }
    }
  }
}

} // namespace foc
