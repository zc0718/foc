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
