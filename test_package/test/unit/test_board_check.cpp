// Conan::ImportStart
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

#include <foc/core/board_check.hpp>
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

TEST(BoardCheck, P7b_ACorrectBoardProducesNoComplaintAndStillCarriesNumbers) {
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
     P7a_AReversedChannelIsNamedAsAPolarityFaultAndNotAsThreeSeparateOnes) {
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
     P7a_ASamplingInstantFaultShowsUpAsRippleAndIsNotBlamedOnTheDeadTime) {
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

TEST(BoardCheck, P7a_AnOffsetIsAlsoWhatBreaksTheAlignmentAndTheReportSaysSo) {
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

TEST(BoardCheck, P7a_FourWrongBoardsFourDifferentNames) {
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

} // namespace foc
