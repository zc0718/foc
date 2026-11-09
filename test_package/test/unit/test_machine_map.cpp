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
  // −3 A 的幅值是 3 A， **在** 实测范围之内：符号无关，幅值有关。
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

namespace {

/** @brief [en] An analytic saturation curve of the shape a real map is measured from: `L0/(1 + (i/i_k)^2)`. */
/** @brief [zh] 真实表所测的那种饱和形状的解析曲线：`L0/(1 + (i/i_k)²)`。 */
auto saturation(float i, double l0, double ik) -> double {
  const double u = static_cast<double>(i) / ik;
  return l0 / (1.0 + (u * u));
}

/** @brief [en] Its second derivative, in closed form -- what the chord error is measured against. */
/** @brief [zh] 它的二阶导数闭式——弦误差就是对着它量的。 */
auto saturation_second_derivative(float i, double l0, double ik) -> double {
  const double u = static_cast<double>(i) / ik;
  const double den = 1.0 + (u * u);
  return (l0 / (ik * ik)) * (((6.0 * u * u) - 2.0) / (den * den * den));
}

/** @brief [en] A map whose rows are `saturation()` sampled every `h` amperes, up to `rows` points. */
/** @brief [zh] 每 `h` 安培取一个 `saturation()` 采样、共 `rows` 点的表。 */
auto sampled_map(float h, float offset, std::size_t rows, double l0, double ik) -> foc::core::MachineMap {
  foc::core::MachineMap map{};
  map.count = rows;
  for (std::size_t k = 0; k < rows; ++k) {
    const float i = static_cast<float>(static_cast<double>(offset) + (static_cast<double>(k) * static_cast<double>(h)));
    const float l = static_cast<float>(saturation(i, l0, ik));
    map.rows[k] = foc::core::MachineMapPoint{i, l, 2.0F * l, l * 0.1F};
  }
  return map;
}

} // namespace

TEST(MachineMap, TheInterpolationErrorIsTheChordErrorAndItGoesAsTheSquareOfTheSpacing) {
  // Linear interpolation is an approximation with a **known** error: on a smooth curve the chord sits `f''*h^2/8` from
  // it at the midpoint of each interval. That is the number a user needs -- with rows every `h` amperes, how wrong is
  // the inductance between them -- and it is checked against an analytic saturation curve, not against a second
  // interpolation. The h^2 is what makes the law falsifiable: halving the spacing must quarter the error.
  // 线性插值是有 **已知** 误差的近似：在光滑曲线上，弦在每个区间中点离曲线 `f″·h²/8`。这正是使用者要的那个数——每 `h`
  // 安培一行时，行与行之间电感错多少——而它是对着解析饱和曲线验的，不是对着第二份插值。`h²` 正是让定律可被否证的东西：
  // 间距减半，误差必须降到四分之一。
  const double l0 = 4.0e-4;
  const double ik = 120.0;
  struct Row final {
    float h;
    float i;
    double measured;
    double law;
  };
  Row rows[6]{};
  int k = 0;
  for (const float h : {40.0F, 20.0F, 10.0F}) {
    // The midpoint of the second interval, which is inside the curved region (`i/ik` near 1) and away from both ends.
    // 第二个区间的中点：落在弯曲区域（`i/ik` 接近 1）、且离两端都远。
    const float midpoint = 60.0F;
    // Rows offset so that 60 A is the centre of a chord at every spacing: the three errors then share one `f''`.
    // 行偏移到让 60 A 在每种间距下都是弦的中点：三个误差于是共享同一个 `f''`。
    const foc::core::MachineMap map =
        sampled_map(h, static_cast<float>(60.0 - (0.5 * static_cast<double>(h))), foc::core::MachineMap::kCapacity,
                    l0, ik);
    const double interpolated = static_cast<double>(map.at(midpoint).ld_h);
    const double truth = saturation(midpoint, l0, ik);
    // The chord sits below a concave-down curve, so `truth - chord` is positive and the magnitude is `|f''|*h^2/8`.
    // 凹曲线之下弦在下方，所以 `真值 − 弦` 为正，量值是 `|f''|·h²/8`。
    const double law = -saturation_second_derivative(midpoint, l0, ik) * static_cast<double>(h) *
                       static_cast<double>(h) / 8.0;
    rows[k] = Row{h, midpoint, truth - interpolated, law};
    std::printf("[map] rows every %5.1f A | at %6.1f A: interpolation off by %+.3e H (chord law %+.3e H, ratio %6.3f)\n",
                static_cast<double>(h), static_cast<double>(midpoint), rows[k].measured, rows[k].law,
                rows[k].measured / rows[k].law);
    // The law is leading order in `h^2`, so at coarse spacing it is 8 % off and at fine spacing 0.5 %: what is
    // asserted is a 10 % band **plus** the convergence, which is the part that cannot be a coincidence.
    // 这条定律是 `h²` 的领头阶，所以粗间距下差 8 %、细间距下差 0.5 %：断言的是 10 % 的带 **加上** 收敛性——
    // 后者才是无法巧合的部分。
    EXPECT_NEAR(rows[k].law, rows[k].measured, 0.10 * std::fabs(rows[k].law)) << "h " << h;
    ++k;
  }
  // h^2: every halving of the row spacing quarters the error. Three points, two ratios.
  // `h²`：行距每减半一次，误差降到四分之一。三个点，两个比值。
  for (int r = 2; r < k; ++r) {
    const double ratio = rows[r - 1].measured / rows[r].measured;
    std::printf("[map] spacing %5.1f -> %5.1f A | error ratio %.3f (law 4.0)\n", static_cast<double>(rows[r - 1].h),
                static_cast<double>(rows[r].h), ratio);
    EXPECT_NEAR(4.0, ratio, 0.2) << "the error did not quarter when the spacing halved";
  }
  // The ratio to the chord law tends to one as the spacing falls: that is the asymptotics showing itself.
  // 与弦定律的比值随间距减小趋于一：渐近性自己现身。
  EXPECT_LT(std::fabs(rows[1].measured / rows[1].law - 1.0), std::fabs(rows[0].measured / rows[0].law - 1.0));
  EXPECT_LT(std::fabs(rows[2].measured / rows[2].law - 1.0), std::fabs(rows[1].measured / rows[1].law - 1.0));
  // At a measured row the answer is the row, to the bit: interpolation has no error where there is nothing to interpolate.
  // 在实测行上答案就是那一行，逐位：没有东西可插值的地方，插值没有误差。
  const foc::core::MachineMap map = sampled_map(20.0F, 0.0F, foc::core::MachineMap::kCapacity, l0, ik);
  for (const float i : {0.0F, 20.0F, 100.0F, 160.0F}) {
    const double truth = saturation(i, l0, ik);
    EXPECT_NEAR(truth, static_cast<double>(map.at(i).ld_h), 1.0e-9) << "row at " << i;
  }
  // And outside the measured range the answer is the end row, not an extrapolation -- a straight extension of a
  // saturating curve is the one error this component must not make, because it invents inductance that was never measured.
  // 而在实测范围之外，答案是端行而不是外推——把饱和曲线直线延长正是这个部件绝不能犯的错，那等于凭空造出从未测过的电感。
  // The last row of a full map: `(kCapacity - 1) * spacing`.
  // 满表最后一行：`(kCapacity − 1) · 间距`。
  const double end_row = saturation(static_cast<float>(foc::core::MachineMap::kCapacity - 1U) * 20.0F, l0, ik);
  // `float` precision, because the row itself is stored as one: 1e-9 relative would be asking the storage for
  // digits it does not have.
  // `float` 精度，因为那一行本身就存成 float：1e-9 相对误差是在向存储索取它没有的位数。
  EXPECT_NEAR(end_row, static_cast<double>(map.at(1.0e4F).ld_h), 1.0e-6 * end_row);
}
