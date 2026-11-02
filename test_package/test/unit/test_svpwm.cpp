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
