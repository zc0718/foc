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

    const foc::core::SvpwmResult past_limit = foc::core::Svpwm::modulate(foc::core::Ab<float>{16.5F, 0.0F}, v_bus);
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
        std::printf("[svpwm] theta %4.1f deg | saturation at %8.5f V | hexagon law %8.5f V | ratio %.6f\n", deg,
                    measured, law, measured / law);
        // The model is the *geometry*, not a fit: six points to 1e-6 of a boundary found by bisection on the code.
        // 模型是**几何**，不是拟合：六个点在代码上二分出来的边界上一致到 1e-6。
        EXPECT_NEAR(law, measured, 1.0e-5 * law) << "theta " << deg;
    }
    // The two limits, named: `v_bus/sqrt(3)` where the sides are, `2/3*v_bus` where the vertices are.
    // 两个极限各有名字：平边处 `v_bus/√3`，顶点处 `2/3·v_bus`。
    EXPECT_NEAR(static_cast<double>(kBusV) / std::sqrt(3.0),
                saturation_amplitude(static_cast<float>(M_PI / 6.0), kBusV), 1.0e-5);
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
            return foc::core::Svpwm::modulate(foc::core::Ab<float>{static_cast<float>(amplitude * std::cos(angle)),
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
    std::printf(
        "[svpwm] over a full turn: 0.999 x the inscribed circle saturates %d of %d angles, the circle itself %d "
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
        std::printf("[svpwm] told v_bus x%.2f | realised/commanded %.6f | law 1/k = %.6f\n", static_cast<double>(k),
                    ratio, 1.0 / static_cast<double>(k));
        EXPECT_NEAR(1.0 / static_cast<double>(k), ratio, 1.0e-5) << "k " << k;
    }
}

namespace {

/**
 * @brief [en] The law this file used to implement: clamp each phase to [0, 1] and let the vector fall where it may.
 * @brief [zh] 本文件原先实现的律：把每一相夹到 [0, 1]，矢量落到哪儿算哪儿。
 * @note [en] Kept here, in the *test*, for two reasons and neither is nostalgia. It is the falsifier -- the two
 *            overmodulation laws are compared on the same grid, so "the new one is better" is a measurement rather than
 *            a preference. And it is the record: the numbers in the claim row that were measured before this change
 *            were measured under this law, so the file that replaces it has to be able to reproduce them.
 * @note [zh] 留在**测试**里，两个理由，都不是怀旧。它是证伪器——两种过调制律在同一张网格上比较，于是"新的更好"是一次测量
 *            而不是一种偏好。它也是记录：这条 claim 行里在这次改动之前量到的数字是在这条律下量的，替换它的文件必须能把
 *            它们复现出来。
 */
auto clamp_law_ab(const foc::core::Ab<float>& request, float v_bus) -> foc::core::Ab<float> {
    const foc::core::Uvw<float> phase = foc::core::inv_clarke(request);
    const float v_max = std::max(phase.u, std::max(phase.v, phase.w));
    const float v_min = std::min(phase.u, std::min(phase.v, phase.w));
    const float offset = 0.5F * (v_max + v_min);
    const auto duty_of = [&](float p) {
        const float d = 0.5F + ((p - offset) / v_bus);
        return std::min(1.0F, std::max(0.0F, d));
    };
    const foc::core::Uvw<float> duty{duty_of(phase.u), duty_of(phase.v), duty_of(phase.w)};
    return realised_ab(duty, v_bus);
}

/**
 * @brief [en] The hexagon's own radius at an angle: `(2/3)*v_bus*cos(30)/cos(theta' - 30)` inside a sector.
 * @brief [zh] 六边形在该角度上的半径：扇区内 `(2/3)·v_bus·cos30/cos(θ′ − 30)`。
 */
auto hex_radius(double angle_rad, double v_bus) -> double {
    // The hexagon of *this* modulator has a **vertex on the alpha axis** -- the six active vectors are where the
    // saturation bisection finds `2/3*v_bus` -- so a sector runs from 0 to 60 degrees and is centred at 30. Writing the
    // sector as centred at 0 instead is a 30-degree rotation of the boundary and it makes the law look violated at every
    // angle; that mistake was in the first version of this helper and the `EXPECT_NEAR` below is what caught it.
    // 本调制器的六边形**在 α 轴上有一个顶点**——六个有效矢量正是二分找到 `2/3·v_bus` 的地方——所以扇区从 0 到 60 度、
    // 中心在 30。若把扇区写成以 0 为中心，边界就被转了 30 度，于是这条律在每个角度上看起来都被违反；这个错在助手函数的
    // 第一版里，是下面那条 `EXPECT_NEAR` 抓住的。
    const double sector = std::fmod(angle_rad, M_PI / 3.0);
    return (2.0 / 3.0) * v_bus * std::cos(M_PI / 6.0) / std::cos(sector - (M_PI / 6.0));
}

struct Fundamental final {
    double amplitude_v;
    double worst_angle_rad;
    double worst_magnitude_v; ///< [en] |v| minus the radial law, over the rotation.
};

/**
 * @brief [en] First harmonic of what the inverter actually applies, over one electrical rotation.
 * @brief [zh] 逆变器实际施加的量的基波，在一个电周期上积分。
 * @param radial [en] true = the shipping law; false = the clamp law it replaced.
 * @param radial [zh] true = 出厂律；false = 它替换掉的夹紧律。
 * @note [en] The harmonics come from the **duties**, read back through the inverter, not from the request: a law that
 *            distorts the vector would otherwise be scored on its intentions.
 * @note [zh] 谐波从**占空比**经逆变器读回得到，而不是从请求得到：否则一条扭曲矢量的律会被按它的**意图**打分。
 */
auto probe(const float request_v, float v_bus, bool radial, int samples = 3600) -> Fundamental {
    Fundamental out{0.0, 0.0, 0.0};
    const double step = (2.0 * M_PI) / static_cast<double>(samples);
    double c1 = 0.0;
    for (int k = 0; k < samples; ++k) {
        const double theta = step * static_cast<double>(k);
        const foc::core::Ab<float> request{static_cast<float>(static_cast<double>(request_v) * std::cos(theta)),
                                           static_cast<float>(static_cast<double>(request_v) * std::sin(theta))};
        const foc::core::Ab<float> applied =
            radial ? realised_ab(foc::core::Svpwm::modulate(request, v_bus).duty, v_bus) : clamp_law_ab(request, v_bus);
        const double mag = std::sqrt((static_cast<double>(applied.alpha) * applied.alpha) +
                                     (static_cast<double>(applied.beta) * applied.beta));
        const double angle = std::atan2(static_cast<double>(applied.beta), static_cast<double>(applied.alpha));
        double delta = std::fabs(angle - theta);
        if (delta > M_PI) {
            delta = (2.0 * M_PI) - delta;
        }
        out.worst_angle_rad = std::max(out.worst_angle_rad, delta);
        const double law = std::min(static_cast<double>(request_v), hex_radius(theta, static_cast<double>(v_bus)));
        out.worst_magnitude_v = std::max(out.worst_magnitude_v, std::fabs(mag - law));
        c1 += (static_cast<double>(applied.alpha) * std::cos(theta)) * step;
    }
    out.amplitude_v = c1 / M_PI;
    return out;
}

constexpr double kSixStepPerBus = 2.0 / M_PI; // 0.6366: the six-step fundamental, in units of v_bus

} // namespace

// F03, first half: above the hexagon the request is **scaled**, not clamped per phase. The difference is measurable and
// it is the angle: a clamp corrects only the phase that wanted out, so its correction is not common-mode and Clarke does
// not remove it, while a scale is a common factor that Clarke sees as a magnitude change only.
// F03 前半：越过六边形后，请求是被**缩放**的，而不是逐相夹紧。差别是可量的，而且就在角度上：夹紧只修正想出去的那一相，
// 于是它的修正不是共模量、Clarke 消不掉；而缩放是三相共同的比例因子，Clarke 只看到幅值变化。
TEST(Svpwm, OvermodulationKeepsTheCommandedAngleAndLandsOnTheHexagon) {
    const double v_bus = static_cast<double>(kBusV);
    double worst_angle_new = 0.0;
    double worst_angle_old = 0.0;
    double worst_law_error = 0.0;

    for (const double ratio : {0.60, 0.70, 0.90, 1.50, 3.00}) {
        for (int deg = 0; deg < 60; deg += 5) {
            const double angle = static_cast<double>(deg) * M_PI / 180.0;
            const double request_v = ratio * v_bus;
            const foc::core::Ab<float> request{static_cast<float>(request_v * std::cos(angle)),
                                               static_cast<float>(request_v * std::sin(angle))};
            const float scale = (request_v > hex_radius(angle, v_bus))
                                    ? static_cast<float>(hex_radius(angle, v_bus) / request_v)
                                    : 1.0F;
            const foc::core::Ab<float> applied = realised_ab(foc::core::Svpwm::modulate(request, kBusV).duty, kBusV);
            const double applied_angle =
                std::atan2(static_cast<double>(applied.beta), static_cast<double>(applied.alpha));
            const double applied_mag =
                std::hypot(static_cast<double>(applied.alpha), static_cast<double>(applied.beta));
            const double law = request_v * static_cast<double>(scale); // = min(|v|, r_hex(theta))

            double angle_error = std::fabs(applied_angle - angle);
            if (angle_error > M_PI) {
                angle_error = (2.0 * M_PI) - angle_error;
            }
            worst_angle_new = std::max(worst_angle_new, angle_error);
            worst_law_error = std::max(worst_law_error, std::fabs(applied_mag - law));
            EXPECT_NEAR(law, applied_mag, 1.0e-5 * law) << "ratio " << ratio << " deg " << deg;
        }
    }

    // The falsifier needs its own, finer grid, and that is a lesson rather than a detail. The rotation the clamp law
    // inflicts is a **max over a peaked function**, so a five-degree grid reports whatever the peak happens to fall
    // next to -- on the grid above the clamp scores 0.349 rad (20.0 deg), and on a 0.1-degree sweep of the same request
    // it scores 0.412 rad (23.6 deg). The first version of this case printed the coarse number and it went into the
    // claim row, where it was wrong by 15 %. The angle *law* may be checked on a coarse grid because it holds
    // everywhere; a worst case may not.
    // 证伪器需要自己、更细的网格，这是一条教训而不是细节。夹紧律造成的旋转是**一个尖峰函数的最大值**，于是五度的
    // 网格只会报出尖峰恰好落在哪一格旁边——在上面那张网格上夹紧律得 0.349 rad（20.0 度），而同一个请求用 0.1 度
    // 扫一遍得 0.412 rad（23.6 度）。这条用例的第一版印的是那个粗值，它还进了 claim 行，错了 15 %。角度**律**可以在
    // 粗网格上查，因为它处处成立；**最坏值**不可以。
    for (const double ratio : {1.50, 3.00}) {
        for (int step = 0; step < 3600; ++step) {
            const double angle = (2.0 * M_PI * static_cast<double>(step)) / 3600.0;
            const double request_v = ratio * v_bus;
            const foc::core::Ab<float> request{static_cast<float>(request_v * std::cos(angle)),
                                               static_cast<float>(request_v * std::sin(angle))};
            const foc::core::Ab<float> clamped = clamp_law_ab(request, kBusV);
            const double clamped_angle =
                std::atan2(static_cast<double>(clamped.beta), static_cast<double>(clamped.alpha));
            double clamp_error = std::fabs(clamped_angle - angle);
            if (clamp_error > M_PI) {
                clamp_error = (2.0 * M_PI) - clamp_error;
            }
            worst_angle_old = std::max(worst_angle_old, clamp_error);

            // The shipping law is measured on this same fine grid, so the two numbers that get compared -- here, in the
            // printout and in the claim row -- come from one sweep. It does not need the resolution (it sits at the floor)
            // but a fine number next to a coarse one is not a comparison, and the claim row quotes both.
            // 出厂律在同一张细网格上量，于是被比较的两个数——这里、印出来的、claim 行里的——来自同一次扫描。它并不需要这个
            // 分辨率（它就在地板上），但细数跟在粗数后面不是一次比较，而 claim 行两个都引。
            const foc::core::Ab<float> scaled = realised_ab(foc::core::Svpwm::modulate(request, kBusV).duty, kBusV);
            double scaled_error =
                std::fabs(std::atan2(static_cast<double>(scaled.beta), static_cast<double>(scaled.alpha)) - angle);
            if (scaled_error > M_PI) {
                scaled_error = (2.0 * M_PI) - scaled_error;
            }
            worst_angle_new = std::max(worst_angle_new, scaled_error);
        }
    }

    std::printf("\n[svpwm] on a 0.1-degree sweep: scaled %.3e rad, clamped %.3e rad (%.2f deg); "
                "|applied| - min(|v|, r_hex) worst %.3e V\n",
                worst_angle_new, worst_angle_old, worst_angle_old * 180.0 / M_PI, worst_law_error);
    EXPECT_LT(worst_angle_new, 1.0e-5) << "a scaled vector stays on the commanded ray";
    EXPECT_GT(worst_angle_old, 0.40) << "and the clamp does not: this is what the case is for, and 0.412 rad (23.6 "
                                        "deg) at three times the bus is what it actually costs";
    EXPECT_LT(worst_law_error, 1.0e-4 * v_bus) << "the delivered magnitude is the stated law, not a leftover";
}

// F03, second half: the utilisation curve. Monotone from the linear range into overmodulation, exact below the inscribed
// circle (`delivered == requested`), continuous at both boundaries, and it **stops** at the mode-I ceiling.
// F03 后半：利用率曲线。从线性区进入过调制**单调**，内切圆以内**精确等于请求**，两个边界都**连续**，并且它**停在**
// mode-I 上限上。
TEST(Svpwm, TheFundamentalRisesMonotonicallyAndStopsAtTheModeOneCeiling) {
    const double v_bus = static_cast<double>(kBusV);
    const double inscribed = v_bus / std::sqrt(3.0);

    // 1. Exact below the inscribed circle, in every direction: a scale of 1 is not applied at all.
    for (const double ratio : {0.10, 0.30, 0.55, 0.60, 0.74, 0.99}) {
        const double request_v = ratio * inscribed;
        const Fundamental f = probe(static_cast<float>(request_v), kBusV, true);
        EXPECT_NEAR(request_v, f.amplitude_v, 1.0e-5 * request_v)
            << "inside the linear range the modulator is the identity";
        EXPECT_LT(f.worst_angle_rad, 1.0e-6);
    }

    // 2. Monotone across the boundary and out into overmodulation.
    std::vector<double> requests;
    for (const double ratio : {0.90, 0.98, 0.999, 1.001, 1.02, 1.06, 1.10, 1.20, 1.40, 2.00, 4.00}) {
        requests.push_back(ratio * inscribed);
    }
    std::printf("[svpwm] utilisation ladder (inscribed circle %.4f V):\n", inscribed);
    double previous = 0.0;
    for (const double request_v : requests) {
        const Fundamental f = probe(static_cast<float>(request_v), kBusV, true);
        std::printf("[svpwm]   request %8.4f V -> fundamental %8.4f V (%.5f v_bus), worst angle %.2e rad\n", request_v,
                    f.amplitude_v, f.amplitude_v / v_bus, f.worst_angle_rad);
        // The tolerance is the quadrature's, not the law's: once the request is past the ceiling the delivered vector is
        // the *same* vector every time, so the only thing left to disagree is the 3600-sample sum of a smooth periodic
        // function (measured jitter 9e-8 V = 6e-9 relative). A real non-monotonicity would be orders of magnitude larger.
        // 容差是**求积**的，不是律的：请求越过上限后交付的是**同一个**矢量，剩下能不一致的只有那条 3600 点求和（实测抖动
        // 9e-8 V = 6e-9 相对）。真正的不单调要大好几个量级。
        EXPECT_GE(f.amplitude_v, previous - (1.0e-6 * previous)) << "utilisation must never fall as the request grows";
        previous = f.amplitude_v;
    }

    // 3. Continuity at the boundary, stated as the quantity that has to be continuous: the **deficit**
    //    `request - delivered`. It is zero below the boundary (the modulator is the identity there) and it must leave zero
    //    without a step, or the loop would see its own limit appear as a discontinuity. Comparing two *different* requests
    //    instead -- which is what the first version of this case did -- measures the request difference, not a jump.
    // 3. 边界的连续性，写成那个必须连续的量：**欠额** `request − delivered`。边界以下它为 0（调制器在那里是恒等映射），
    //    而它离开 0 时不许有台阶，否则环路会看到自己的极限像一次跳变。第一版比较的是**两个不同请求**的交付值——那量到的是
    //    请求之差，不是跳变。
    const auto deficit = [&](double ratio) {
        const double request_v = ratio * inscribed;
        const Fundamental f = probe(static_cast<float>(request_v), kBusV, true);
        return request_v - f.amplitude_v;
    };
    const double d_below = deficit(0.999);
    const double d_at = deficit(1.000);
    const double d_just = deficit(1.001);
    const double d_2pct = deficit(1.020);
    std::printf("[svpwm] deficit at the boundary: 0.999x %.3e V, 1.000x %.3e V, 1.001x %.3e V, 1.02x %.3e V\n", d_below,
                d_at, d_just, d_2pct);
    EXPECT_LT(std::fabs(d_below), 1.0e-4 * inscribed);
    EXPECT_LT(std::fabs(d_at), 1.0e-4 * inscribed);
    EXPECT_LT(std::fabs(d_just), 1.0e-3 * inscribed) << "no step at the boundary";
    EXPECT_LT(d_2pct, 1.0e-2 * inscribed) << "and it grows smoothly after it";

    // 4. And the ceiling, named: this law stops at 95.1 % of the six-step fundamental and does not go further, however
    //    hard it is asked. That is not a defect of the implementation -- it is what "keep the commanded angle" costs.
    const Fundamental hard = probe(static_cast<float>(4.0 * v_bus), kBusV, true);
    const double ceiling_per_bus = hard.amplitude_v / v_bus;
    std::printf("[svpwm] ceiling %.5f v_bus = %.4f x the six-step fundamental %.5f v_bus\n", ceiling_per_bus,
                ceiling_per_bus / kSixStepPerBus, kSixStepPerBus);
    EXPECT_LT(ceiling_per_bus, kSixStepPerBus) << "the mode-I law does not reach six-step, and saying so is the claim";
    EXPECT_NEAR(0.9514, ceiling_per_bus / kSixStepPerBus, 5.0e-3);
}

// F03, the trade-off itself, measured: the clamp law reaches the six-step fundamental and pays for it by rotating the
// applied vector. Both numbers are on one grid, so a later reader can re-open the decision with evidence instead of taste.
// F03 的取舍本身，量出来：夹紧律能到达六步基波，代价是把施加矢量转掉。两个数在同一张网格上，于是后来的人可以拿证据重新
// 打开这个决定，而不是凭口味。
TEST(Svpwm, TheClampLawBuysTheSixStepFundamentalByRotatingTheVector) {
    const double v_bus = static_cast<double>(kBusV);
    const Fundamental radial = probe(static_cast<float>(3.0 * v_bus), kBusV, true);
    const Fundamental clamped = probe(static_cast<float>(3.0 * v_bus), kBusV, false);
    std::printf("\n[svpwm] at 3x v_bus: scaled fundamental %.5f (%.5f v_bus, angle err %.2e rad); "
                "clamped %.5f (%.5f v_bus, angle err %.2e rad = %.1f deg)\n",
                radial.amplitude_v, radial.amplitude_v / v_bus, radial.worst_angle_rad, clamped.amplitude_v,
                clamped.amplitude_v / v_bus, clamped.worst_angle_rad, clamped.worst_angle_rad * 180.0 / M_PI);

    EXPECT_NEAR(radial.amplitude_v / v_bus, 0.6057, 2.0e-3) << "mode-I ceiling, the number the claim row names";
    EXPECT_NEAR(clamped.amplitude_v / v_bus, static_cast<double>(kSixStepPerBus), 2.0e-3)
        << "and the clamp really does reach six-step: if this stops being true the trade-off has changed";
    EXPECT_GT(clamped.worst_angle_rad, 0.05) << "the price, in radians of flux-axis error";
    EXPECT_LT(radial.worst_angle_rad, 1.0e-5) << "and the thing the price buys";
}

// F03: what the limit *looks like* when it is asked for far more than it has. A two-level waveform -- every phase at a
// rail, never in between -- whose fundamental is the mode-I ceiling. This is the assertion that would notice a "fix"
// that merely reduced the amplitude, and it is a shape rather than a value.
// F03：当请求远大于能力时，极限**长什么样**。两电平波形——每一相都在轨道上、从不居中——其基波是 mode-I 上限。这条断言
// 会发现一个"只降低了幅值"的假修法，而它是一条**形状**断言而不是数值断言。
TEST(Svpwm, AskedFarEnoughTheWaveformIsTwoLevel) {
    // The shape of the limit is not "every phase on a rail" -- with midpoint injection exactly one phase is at 1 and one at
    // 0 at every instant, and the third sits in between, which is what makes the *line-to-line* voltage the full bus. The
    // claim is therefore about the **span**: the duties must use the whole [0, 1] range at every angle, and the middle
    // phase may only touch a rail at the six vertices. Asking for a thousand times the bus must look exactly like asking
    // for three times it -- overmodulation saturates in shape, not in magnitude.
    // 极限的形状不是"每一相都在轨道上"——中点注入下每个瞬间恰好一相在 1、一相在 0、第三相在中间，正是这一点让**线电压**
    // 用满母线。所以论断是关于**跨度**的：占空比在每个角度上都必须用满 [0,1]，而中间那一相只允许在六个顶点上碰到轨道。
    // 请求一千倍母线必须与请求三倍看起来完全一样——过调制饱和在**形状**上，而不是幅值上。
    const int samples = 720;
    int middle_at_rail = 0;
    bool span_full = true;
    for (int k = 0; k < samples; ++k) {
        const double theta = (2.0 * M_PI) * static_cast<double>(k) / static_cast<double>(samples);
        const foc::core::Ab<float> request{static_cast<float>(1.0e3 * std::cos(theta)),
                                           static_cast<float>(1.0e3 * std::sin(theta))};
        const auto result = foc::core::Svpwm::modulate(request, kBusV);
        const foc::core::Uvw<float> d = result.duty;
        const float hi = std::max(d.u, std::max(d.v, d.w));
        const float lo = std::min(d.u, std::min(d.v, d.w));
        if ((std::fabs(static_cast<double>(hi) - 1.0) > 1.0e-6) || (std::fabs(static_cast<double>(lo)) > 1.0e-6)) {
            span_full = false;
        }
        for (const float duty : {d.u, d.v, d.w}) {
            const bool at_rail = (static_cast<double>(duty) < 1.0e-6) || (static_cast<double>(duty) > 1.0 - 1.0e-6);
            if (at_rail) {
                ++middle_at_rail;
            }
        }
    }
    std::printf("[svpwm] at a 1000x request: %d of %d phase samples sit on a rail (2 per angle expected, +6 at the "
                "vertices)\n",
                middle_at_rail, 3 * samples);
    EXPECT_TRUE(span_full) << "the duties must span the whole bus at every angle";
    EXPECT_GE(middle_at_rail, 2 * samples) << "at least two phases on the rails: that is what uses the bus";
    EXPECT_LE(middle_at_rail, (2 * samples) + 12) << "and the third only at the vertices, or the span claim is empty";
}
