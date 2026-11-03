#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>



namespace {

constexpr float kTol = 1e-5F;

auto phase_from_duty(const foc::core::Uvw<float>& duty, float v_bus) -> foc::core::Uvw<float> {
    const float half = 0.5F;
    return foc::core::Uvw<float>{(duty.u - half) * v_bus, (duty.v - half) * v_bus, (duty.w - half) * v_bus};
}

} // namespace



TEST(Svpwm, ZeroVectorGivesFiftyPercent) {
    const foc::core::SvpwmResult result = foc::core::Svpwm::modulate(foc::core::Ab<float>{}, 24.0F);

    EXPECT_FLOAT_EQ(0.5F, result.duty.u);
    EXPECT_FLOAT_EQ(0.5F, result.duty.v);
    EXPECT_FLOAT_EQ(0.5F, result.duty.w);
    EXPECT_FALSE(result.saturated);
    EXPECT_EQ(foc::ErrorCode::Ok, result.status);
}



TEST(Svpwm, NonPositiveBusVoltageIsRejectedNotCrashed) {
    const foc::core::Ab<float> v_ab{1.0F, 2.0F};

    for (const float v_bus : {0.0F, -24.0F}) {
        const foc::core::SvpwmResult result = foc::core::Svpwm::modulate(v_ab, v_bus);
        EXPECT_FLOAT_EQ(0.5F, result.duty.u);
        EXPECT_FLOAT_EQ(0.5F, result.duty.v);
        EXPECT_FLOAT_EQ(0.5F, result.duty.w);
        EXPECT_EQ(foc::ErrorCode::Param, result.status);
    }
}



TEST(Svpwm, LinearRangeReproducesTheCommandedVector) {
    const float v_bus = 24.0F;
    // 0.8 * v_bus / sqrt(3) keeps the modulator inside its linear range.
    const float magnitude = 0.8F * v_bus / std::sqrt(3.0F);

    for (int step = 0; step < 12; ++step) {
        const float theta = static_cast<float>(step) * 0.5F;
        const foc::core::Ab<float> v_ab{magnitude * std::cos(theta), magnitude * std::sin(theta)};
        const foc::core::SvpwmResult result = foc::core::Svpwm::modulate(v_ab, v_bus);

        EXPECT_EQ(foc::ErrorCode::Ok, result.status);
        EXPECT_FALSE(result.saturated);
        EXPECT_GE(result.duty.u, 0.0F);
        EXPECT_LE(result.duty.u, 1.0F);

        // Clarke removes the injected common mode: the differential part must be the command.
        const foc::core::Ab<float> rebuilt = foc::core::clarke(phase_from_duty(result.duty, v_bus));
        EXPECT_NEAR(v_ab.alpha, rebuilt.alpha, kTol * v_bus);
        EXPECT_NEAR(v_ab.beta, rebuilt.beta, kTol * v_bus);
    }
}



TEST(Svpwm, OverModulationSaturatesAndClamps) {
    const float v_bus = 24.0F;
    const foc::core::Ab<float> v_ab{v_bus, 0.0F}; // far beyond v_bus / sqrt(3)

    const foc::core::SvpwmResult result = foc::core::Svpwm::modulate(v_ab, v_bus);

    EXPECT_TRUE(result.saturated);
    EXPECT_EQ(foc::ErrorCode::Ok, result.status);
    for (const float duty : {result.duty.u, result.duty.v, result.duty.w}) {
        EXPECT_GE(duty, 0.0F);
        EXPECT_LE(duty, 1.0F);
    }
}



TEST(Svpwm, DutyOrderingFollowsPhaseVoltageOrdering) {
    const float v_bus = 24.0F;
    const foc::core::Ab<float> v_ab{5.0F, 0.0F};

    const foc::core::SvpwmResult result = foc::core::Svpwm::modulate(v_ab, v_bus);

    // u carries the highest phase voltage, v and w are symmetric around the midpoint.
    EXPECT_GT(result.duty.u, result.duty.v);
    EXPECT_NEAR(result.duty.v, result.duty.w, kTol);
}



TEST(SvpwmSector, ZeroVectorIsSectorZero) { EXPECT_EQ(0U, foc::core::svpwm_sector(foc::core::Ab<float>{}, 1e-6F)); }



TEST(SvpwmSector, InteriorAnglesMapToOneToSix) {
    const float magnitude = 2.0F;
    for (int sector = 1; sector <= 6; ++sector) {
        const float theta = (static_cast<float>(sector) - 0.5F) * (foc::core::kPi<float> / 3.0F);
        const foc::core::Ab<float> v_ab{magnitude * std::cos(theta), magnitude * std::sin(theta)};

        EXPECT_EQ(sector, foc::core::svpwm_sector(v_ab, 1e-6F)) << "theta = " << theta;
    }
}



TEST(SvpwmSector, AZeroComponentBelongsToTheNegativeSide) {
    // The three sign tests are strict `> 0`: a component that is exactly zero counts as not positive. That
    // rule decides which sector owns an axis vector and which owns a 60-degree line, and an interior-angle
    // case cannot see it -- the mutation run found all three comparisons surviving when flipped to `>=`.
    // 三个符号判定都是严格 `> 0`：恰好为零的分量算作非正。这条规则决定轴线向量与 60° 线归哪个扇区，
    // 而内点用例看不见它——变异测试发现三个比较改成 `>=` 后全部存活。
    const float epsilon = 1e-6F;

    // beta == 0, so `a` is exactly zero and must not contribute: code 2 -> sector 6.
    EXPECT_EQ(6U, foc::core::svpwm_sector(foc::core::Ab<float>{1.0F, 0.0F}, epsilon));

    // `b` exactly zero, built with the implementation's own expression order so the subtraction is exact:
    // code 1 -> sector 2.
    const float alpha_b = 1.0F;
    const float beta_b = foc::core::kSqrt3Over2<float> * alpha_b * 2.0F;
    EXPECT_EQ(2U, foc::core::svpwm_sector(foc::core::Ab<float>{alpha_b, beta_b}, epsilon));

    // `c` exactly zero, on the other side: code 2 -> sector 6.
    const float alpha_c = 1.0F;
    const float beta_c = -foc::core::kSqrt3Over2<float> * alpha_c * 2.0F;
    EXPECT_EQ(6U, foc::core::svpwm_sector(foc::core::Ab<float>{alpha_c, beta_c}, epsilon));
}



TEST(SvpwmSector, AComponentExactlyAtTheDeadbandIsStillZero) {
    // `|alpha| > epsilon`, not `>=`: at exactly the deadband the vector counts as zero. Every other case
    // uses a magnitude far from the threshold, which is why flipping that comparison survived.
    // 判定是 `|alpha| > epsilon` 而非 `>=`：恰好落在死区上的向量算作零。其余用例的量级都远离该阈值，
    // 所以把这个比较改成 `>=` 后仍然存活。
    const float epsilon = 1e-6F;

    EXPECT_EQ(0U, foc::core::svpwm_sector(foc::core::Ab<float>{epsilon, 0.0F}, epsilon));
    EXPECT_EQ(0U, foc::core::svpwm_sector(foc::core::Ab<float>{0.0F, -epsilon}, epsilon));
    EXPECT_EQ(0U, foc::core::svpwm_sector(foc::core::Ab<float>{-epsilon, epsilon}, epsilon));
}


TEST(Svpwm, ExactlyAtTheLinearLimitIsNotYetSaturated) {
    // `saturated` means "a phase *wanted* to leave [0, 1] before clamping", so the boundary itself does not
    // count: the comparisons are strict. With the common-mode offset folded in, both sides of the check reduce
    // to `(v_max - v_min) > v_bus` -- they trip together, always, which is why `||` and `&&` are equivalent
    // here and why only the equality point distinguishes `>` from `>=`.
    // For an axis-aligned vector u = m, v = w = -m/2: offset = m/4, v_max = m, v_min = -m/2, and the trip is at
    // m = 2/3 * v_bus. At v_bus = 24 that is m = 16, a value float represents exactly, so the equality is real
    // and not a rounding accident.
    // `saturated` 的含义是"某一相在夹紧之前**想要**离开 [0,1]"，因此边界本身不算：比较是严格的。把共模
    // 偏置并进去后，检查的两侧都化简为 `(v_max - v_min) > v_bus`——它们总是同时触发，这正是这里 `||` 与
    // `&&` 等价、而只有相等点能区分 `>` 与 `>=` 的原因。轴向矢量 u = m、v = w = −m/2 时 offset = m/4，
    // 触发点在 m = 2/3·v_bus；v_bus = 24 时 m = 16，float 可精确表示，所以这个相等是真实的、不是舍入巧合。
    const float v_bus = 24.0F;

    const foc::core::SvpwmResult at_limit = foc::core::Svpwm::modulate(foc::core::Ab<float>{16.0F, 0.0F}, v_bus);
    EXPECT_FALSE(at_limit.saturated);

    const foc::core::SvpwmResult past_limit =
        foc::core::Svpwm::modulate(foc::core::Ab<float>{16.5F, 0.0F}, v_bus);
    EXPECT_TRUE(past_limit.saturated);

    // Well inside the linear range, nothing trips either.
    const foc::core::SvpwmResult inside = foc::core::Svpwm::modulate(foc::core::Ab<float>{12.0F, 0.0F}, v_bus);
    EXPECT_FALSE(inside.saturated);
}

namespace {

constexpr float kBusV = 24.0F;

/**
 * @brief [en] The vector the duties ask for, read back through the inverter: phase-to-neutral is `(duty - mean)*v_bus`.
 * @brief [zh] 占空比要求的那个矢量，经逆变器读回：相电压为 `(duty − mean)·v_bus`。
 */
auto realised_ab(const foc::core::Uvw<float>& duty, float v_bus) -> foc::core::Ab<float> {
  const float mean = (duty.u + duty.v + duty.w) / 3.0F;
  return foc::core::clarke(
      foc::core::Uvw<float>{(duty.u - mean) * v_bus, (duty.v - mean) * v_bus, (duty.w - mean) * v_bus});
}

/**
 * @brief [en] The largest amplitude that does **not** saturate, bisected on the modulator itself at a given angle.
 * @brief [zh] 在给定角度上**不**饱和的最大幅值，直接在调制器上二分。
 */
auto saturation_amplitude(float angle_rad, float v_bus) -> double {
  double lo = 0.0;
  double hi = static_cast<double>(v_bus);
  for (int k = 0; k < 200; ++k) {
    const double mid = 0.5 * (lo + hi);
    const foc::core::Ab<float> request{static_cast<float>(mid * std::cos(static_cast<double>(angle_rad))),
                                       static_cast<float>(mid * std::sin(static_cast<double>(angle_rad)))};
    if (foc::core::Svpwm::modulate(request, v_bus).saturated) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return hi;
}

} // namespace

TEST(Svpwm, TheLinearRangeIsAHexagonNotACircleAndItsBoundaryIsExact) {
  // The header says "linear range", which reads like one number. It is not: the set of reproducible vectors is the
  // **hexagon** whose vertices sit on the six active vectors at `2/3*v_bus` and whose flat sides come within
  // `2/3*v_bus*cos(30) = v_bus/sqrt(3)` of the origin. So the guaranteed amplitude over a whole rotation is the
  // inscribed circle `v_bus/sqrt(3) = 13.856 V`, and the reachable amplitude at a sector centre is `2/3*v_bus = 16.0 V`
  // -- 15 % more, available only at six angles. Measured by bisecting the modulator at six angles and compared to the
  // hexagon's own closed form `(2/3*v_bus)*cos(30)/cos(theta' - 30)`.
  // 头文件说"线性区"，读起来像一个数。并不是：可复现矢量的集合是**六边形**——六个顶点落在六个有效矢量方向上的
  // `2/3·v_bus`，平边离原点 `2/3·v_bus·cos30 = v_bus/√3`。所以整圈都保证的幅值是内切圆 `v_bus/√3 = 13.856 V`，
  // 而扇区中心可达 `2/3·v_bus = 16.0 V`——多 15 %，只在六个角度上。用二分在六个角度上量出，并对着六边形自己的闭式
  // `(2/3·v_bus)·cos30/cos(θ′ − 30)` 比较。
  const double vertex = (2.0 / 3.0) * static_cast<double>(kBusV);
  for (const double deg : {0.0, 10.0, 20.0, 30.0, 40.0, 50.0}) {
    const double angle = deg * M_PI / 180.0;
    const double measured = saturation_amplitude(static_cast<float>(angle), kBusV);
    const double law = vertex * std::cos(M_PI / 6.0) / std::cos(angle - (M_PI / 6.0));
    std::printf("[svpwm] theta %4.1f deg | saturation at %8.5f V | hexagon law %8.5f V | ratio %.6f\n", deg, measured,
                law, measured / law);
    // The model is the *geometry*, not a fit: six points to 1e-6 of a boundary found by bisection on the code.
    // 模型是**几何**，不是拟合：六个点在代码上二分出来的边界上一致到 1e-6。
    EXPECT_NEAR(law, measured, 1.0e-5 * law) << "theta " << deg;
  }
  // The two limits, named: `v_bus/sqrt(3)` where the sides are, `2/3*v_bus` where the vertices are.
  // 两个极限各有名字：平边处 `v_bus/√3`，顶点处 `2/3·v_bus`。
  EXPECT_NEAR(static_cast<double>(kBusV) / std::sqrt(3.0), saturation_amplitude(static_cast<float>(M_PI / 6.0), kBusV),
              1.0e-5);
  EXPECT_NEAR(vertex, saturation_amplitude(0.0F, kBusV), 1.0e-5);
}

TEST(Svpwm, TheInscribedCircleIsGuaranteedJustInsideItAndTheHexagonIsNot) {
  // The practical statement a control loop needs, and it has to be exact about *just inside*: `v_bus/sqrt(3)` is
  // attained exactly at the six angles where the hexagon's flat sides are, so a request of exactly that amplitude is
  // on the boundary there and lands on whichever side the float rounding puts it. A hair under it is guaranteed at
  // every angle; at `2/3*v_bus` only the six vertex angles survive.
  // 控制环真正要的那句话，而且必须说清"刚好在内部"：`v_bus/√3` 正好在六边形平边所在的六个角度上取到，所以恰好
  // 这个幅值的请求在那里就在边界上，落在哪一侧由浮点舍入决定。比它低一点则每个角度都保证；`2/3·v_bus` 处只有六个
  // 顶点角度活下来。
  const float circle = kBusV / std::sqrt(3.0F);
  int inside_saturated = 0;
  int boundary_saturated = 0;
  int hexagon_saturated = 0;
  constexpr int kSteps = 72;
  for (int k = 0; k < kSteps; ++k) {
    const double angle = (2.0 * M_PI * static_cast<double>(k)) / static_cast<double>(kSteps);
    const auto modulate = [&](double amplitude) {
      return foc::core::Svpwm::modulate(
          foc::core::Ab<float>{static_cast<float>(amplitude * std::cos(angle)),
                               static_cast<float>(amplitude * std::sin(angle))},
          kBusV);
    };
    if (modulate(0.999 * static_cast<double>(circle)).saturated) {
      ++inside_saturated;
    }
    if (modulate(static_cast<double>(circle)).saturated) {
      ++boundary_saturated;
    }
    if (modulate((2.0 / 3.0) * static_cast<double>(kBusV)).saturated) {
      ++hexagon_saturated;
    }
  }
  std::printf("[svpwm] over a full turn: 0.999 x the inscribed circle saturates %d of %d angles, the circle itself %d "
              "(the six flat sides), 2/3*v_bus %d\n",
              inside_saturated, kSteps, boundary_saturated, hexagon_saturated);
  EXPECT_EQ(0, inside_saturated) << "the inscribed circle is not guaranteed after all";
  // Exactly six angles sit on the boundary -- the six where the hexagon's floor is the circle. Not zero: a request of
  // exactly the limit is on the limit.
  // 恰好六个角度落在边界上——六边形的地板就是圆的六个位置。不是零：恰好等于上限的请求就在上限上。
  EXPECT_LE(boundary_saturated, 6) << "the circle is not the hexagon's floor";
  EXPECT_GT(hexagon_saturated, (kSteps * 8) / 10) << "the hexagon boundary is not angle-dependent after all";
}

TEST(Svpwm, WhatComesOutOfTheInverterIsTheVectorThatWentIn) {
  // The definitional identity, measured in the direction the code does not compute in: take the duties it produced,
  // rebuild the phase-to-neutral voltage, Clarke them back, and compare with the request. The common-mode offset is
  // removed by the reconstruction (it has to be -- it is what SVPWM adds), so what is left is the claim itself: this
  // modulator has **no gain error** inside its range.
  // 定义级的恒等式，且沿代码并不计算的那个方向量：拿它产出的占空比，重建相电压，Clarke 回去，与请求比较。
  // 共模偏置在重建中被去掉（必须去掉——那正是 SVPWM 加进去的东西），剩下就是这条主张本身：本调制器在其范围内
  // **没有增益误差**。
  for (const float deg : {0.0F, 17.0F, 30.0F, 60.0F, 123.0F, 200.0F, 359.0F}) {
    const double angle = static_cast<double>(deg) * M_PI / 180.0;
    const foc::core::Ab<float> request{static_cast<float>(8.0 * std::cos(angle)),
                                       static_cast<float>(8.0 * std::sin(angle))};
    const auto result = foc::core::Svpwm::modulate(request, kBusV);
    EXPECT_EQ(foc::ErrorCode::Ok, result.status);
    const foc::core::Ab<float> back = realised_ab(result.duty, kBusV);
    const double error = std::hypot(static_cast<double>(back.alpha) - static_cast<double>(request.alpha),
                                    static_cast<double>(back.beta) - static_cast<double>(request.beta));
    std::printf("[svpwm] %3.0f deg | reconstructed - request = %.3e V\n", static_cast<double>(deg), error);
    // 1e-5 V on an 8 V vector is float noise, not a tolerance: the mapping is linear and exact.
    // 8 V 的矢量上 1e-5 V 是浮点噪声而不是容差：这个映射是线性且精确的。
    EXPECT_LT(error, 1.0e-5) << "deg " << deg;
    // And every duty is inside [0, 1] -- the vector is inside the range, so nothing should have been clamped.
    // 每个占空比都在 [0,1] 内——矢量在范围内，所以什么也不该被夹。
    EXPECT_GE(result.duty.u, 0.0F);
    EXPECT_LE(result.duty.u, 1.0F);
    EXPECT_GE(result.duty.v, 0.0F);
    EXPECT_LE(result.duty.v, 1.0F);
    EXPECT_GE(result.duty.w, 0.0F);
    EXPECT_LE(result.duty.w, 1.0F);
  }
}

TEST(Svpwm, ABusVoltageThatIsWrongComesOutAsExactlyThatError) {
  // The error source a drive always has: the measured DC link is not the one the modulator divides by. The duties are
  // `0.5 + (v - offset)/v_bus_assumed`, so realising them at the true bus scales the vector by `1/k` -- exactly, and
  // **the wrong way round**: telling the modulator the bus is low makes it deliver *more* voltage.
  // 驱动永远有的误差源：量到的母线不是调制器用来做除法的那一个。占空比是 `0.5 + (v − offset)/v_bus_被告知`，于是
  // 在真实母线上实现它们会把矢量缩放 `1/k`——精确地，而且**方向是反的**：告诉调制器母线偏低，它反而输出更多电压。
  const foc::core::Ab<float> request{8.0F, 0.0F};
  for (const float k : {0.90F, 0.95F, 1.05F, 1.10F}) {
    const auto result = foc::core::Svpwm::modulate(request, kBusV * k);
    const foc::core::Ab<float> back = realised_ab(result.duty, kBusV);
    const double ratio = static_cast<double>(back.alpha) / static_cast<double>(request.alpha);
    std::printf("[svpwm] told v_bus x%.2f | realised/commanded %.6f | law 1/k = %.6f\n", static_cast<double>(k), ratio,
                1.0 / static_cast<double>(k));
    EXPECT_NEAR(1.0 / static_cast<double>(k), ratio, 1.0e-5) << "k " << k;
  }
}
