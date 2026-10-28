// A single `Ld` is a claim about one operating point. This file pins down what a map promises instead: interpolation
// inside the measured range, clamping outside it, and no invented rows.
// 单个 `Ld` 是关于一个工作点的主张。本文件把表承诺的东西钉住：实测范围内插值、范围外钳位、不凭空造行。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/machine_map.hpp>

namespace foc {

namespace {

auto three_rows() -> core::MachineMap {
    core::MachineMap map{};
    // A knee: the inductances fall away as the machine saturates, the flux rises slightly.
    // 一个拐点：随着机器饱和，电感下落，磁链略微上升。
    map.rows[0] = core::MachineMapPoint{0.0F, 4.0e-4F, 6.0e-4F, 0.050F};
    map.rows[1] = core::MachineMapPoint{2.0F, 3.0e-4F, 4.5e-4F, 0.052F};
    map.rows[2] = core::MachineMapPoint{4.0F, 2.0e-4F, 3.0e-4F, 0.054F};
    map.count = 3U;
    return map;
}

} // namespace

TEST(MachineMap, AtAMeasuredRowTheMapReturnsExactlyThatRow) {
  // The criterion: interpolation must not move the measurements. If the knots are not reproduced, everything else
  // about the map is arithmetic on top of a wrong table.
  // 判据：插值不得移动实测值。若节点都复现不出来，表的其它一切都是建立在错表上的算术。
  const core::MachineMap map = three_rows();
  ASSERT_TRUE(map.valid());

  for (std::size_t i = 0U; i < map.count; ++i) {
    const core::MachinePoint point = map.at(map.rows[i].i_a);
    EXPECT_FLOAT_EQ(point.ld_h, map.rows[i].ld_h);
    EXPECT_FLOAT_EQ(point.lq_h, map.rows[i].lq_h);
    EXPECT_FLOAT_EQ(point.psi_f_wb, map.rows[i].psi_f_wb);
  }
}

TEST(MachineMap, BetweenRowsItInterpolatesLinearly) {
  const core::MachineMap map = three_rows();
  const core::MachinePoint mid = map.at(1.0F); // halfway between rows 0 and 1
  std::printf("\n[map] Ld(1.0 A) = %.6e H (rows %.6e / %.6e)\n", static_cast<double>(mid.ld_h),
              static_cast<double>(map.rows[0].ld_h), static_cast<double>(map.rows[1].ld_h));
  EXPECT_NEAR(mid.ld_h, 3.5e-4F, 1.0e-9F);
  EXPECT_NEAR(mid.lq_h, 5.25e-4F, 1.0e-9F);
  EXPECT_NEAR(mid.psi_f_wb, 0.051F, 1.0e-6F);
}

TEST(MachineMap, OutsideTheMeasuredRangeTheAnswerIsClampedNotExtrapolated) {
  // Extrapolating a saturation curve past the last measurement invents behaviour nobody measured, and it does so
  // quietly. The end row is the only defensible answer, and it is the same answer for a negative current because the
  // map is even.
  // 把饱和曲线外推到最后一个测点之外，等于安静地发明没人测过的行为。端行是唯一站得住的答案，而由于表对电流是
  // 偶函数，负电流得到同一个答案。
  const core::MachineMap map = three_rows();
  EXPECT_FLOAT_EQ(map.at(9.0F).ld_h, map.rows[2].ld_h);
  EXPECT_FLOAT_EQ(map.at(-9.0F).ld_h, map.rows[2].ld_h);
  EXPECT_FLOAT_EQ(map.at(0.0F).ld_h, map.rows[0].ld_h);
  // -3 A has magnitude 3 A, which is *inside* the measured range: the sign does not matter, the magnitude does.
  // −3 A 的幅值是 3 A，**在**实测范围之内：符号无关，幅值有关。
  EXPECT_NEAR(map.at(-3.0F).ld_h, 2.5e-4F, 1.0e-9F);
}

TEST(MachineMap, ASingleRowIsAConstantAndSaysSo) {
  // A nameplate parameter set is not a lie, it is a one-row map. It must answer everywhere rather than pretending to
  // have a shape.
  // 铭牌参数集不是谎话，它是一行表。它必须在处处作答，而不是假装自己有形状。
  const core::MachineParams params{0.5F, 4.0e-4F, 6.0e-4F, 0.05F};
  const core::MachineMap map = core::MachineMap::from_params(params, 0.0F);
  ASSERT_TRUE(map.valid());
  EXPECT_FLOAT_EQ(map.at(0.0F).ld_h, params.ld);
  EXPECT_FLOAT_EQ(map.at(5.0F).ld_h, params.ld);
  EXPECT_FLOAT_EQ(map.at(100.0F).lq_h, params.lq);
  EXPECT_FLOAT_EQ(map.at(2.0F).with_rs(params.rs).psi_f, params.psi_f);
}

TEST(MachineMap, AnUnsortedOrDegenerateMapIsRejected) {
  core::MachineMap map = three_rows();
  ASSERT_TRUE(map.valid());

  map.rows[1].i_a = 0.0F; // not strictly increasing
  EXPECT_FALSE(map.valid()) << "rows must be sorted by current, or interpolation brackets the wrong pair";

  map = three_rows();
  map.rows[2].ld_h = 0.0F; // a zero inductance makes every downstream gain infinite
  EXPECT_FALSE(map.valid());

  map = three_rows();
  map.count = 0U;
  EXPECT_FALSE(map.valid());

  map = three_rows();
  map.count = core::MachineMap::kCapacity + 1U;
  EXPECT_FALSE(map.valid()) << "the capacity is static; a count past it must not be trusted";
}

} // namespace foc
