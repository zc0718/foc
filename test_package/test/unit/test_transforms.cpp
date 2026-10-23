#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>



namespace {

constexpr float kTol = 1e-5F;

} // namespace



TEST(Transforms, ClarkeIsAmplitudeInvariant) {
    // A balanced set of amplitude 1 maps to a vector of amplitude 1 on the alpha axis.
    const foc::core::Uvw<float> uvw{1.0F, -0.5F, -0.5F};
    const foc::core::Ab<float> ab = foc::core::clarke(uvw);

    EXPECT_NEAR(1.0F, ab.alpha, kTol);
    EXPECT_NEAR(0.0F, ab.beta, kTol);
}



TEST(Transforms, ClarkeNinetyDegrees) {
    const float root3_over_2 = std::sqrt(3.0F) / 2.0F;
    const foc::core::Uvw<float> uvw{0.0F, root3_over_2, -root3_over_2};
    const foc::core::Ab<float> ab = foc::core::clarke(uvw);

    EXPECT_NEAR(0.0F, ab.alpha, kTol);
    EXPECT_NEAR(1.0F, ab.beta, kTol);
}



TEST(Transforms, ClarkeOfZeroIsZero) {
    const foc::core::Ab<float> ab = foc::core::clarke(foc::core::Uvw<float>{});

    EXPECT_NEAR(0.0F, ab.alpha, kTol);
    EXPECT_NEAR(0.0F, ab.beta, kTol);
}



TEST(Transforms, InverseClarkeRoundTrips) {
    const foc::core::Ab<float> ab{0.3F, -0.7F};
    const foc::core::Uvw<float> uvw = foc::core::inv_clarke(ab);
    const foc::core::Ab<float> back = foc::core::clarke(uvw);

    EXPECT_NEAR(ab.alpha, back.alpha, kTol);
    EXPECT_NEAR(ab.beta, back.beta, kTol);
}



TEST(Transforms, ParkAtZeroAngleIsIdentity) {
    const foc::core::Ab<float> ab{0.4F, -0.2F};
    const foc::core::Dq<float> dq = foc::core::park(ab, 0.0F);

    EXPECT_NEAR(ab.alpha, dq.d, kTol);
    EXPECT_NEAR(ab.beta, dq.q, kTol);
}



TEST(Transforms, ParkAtNinetyDegreesSwapsAxes) {
    const foc::core::Ab<float> ab{0.4F, -0.2F};
    const foc::core::Dq<float> dq = foc::core::park(ab, foc::core::kPi<float> / 2.0F);

    EXPECT_NEAR(ab.beta, dq.d, kTol);
    EXPECT_NEAR(-ab.alpha, dq.q, kTol);
}



TEST(Transforms, InverseParkRoundTrips) {
    const foc::core::Dq<float> dq{0.25F, -0.75F};
    const float theta = 0.9F;
    const foc::core::Ab<float> ab = foc::core::inv_park(dq, theta);
    const foc::core::Dq<float> back = foc::core::park(ab, theta);

    EXPECT_NEAR(dq.d, back.d, kTol);
    EXPECT_NEAR(dq.q, back.q, kTol);
}



TEST(Transforms, DoublePrecisionKeepsAccuracy) {
    const foc::core::Uvw<double> uvw{1.0, -0.5, -0.5};
    const foc::core::Ab<double> ab = foc::core::clarke(uvw);

    EXPECT_NEAR(1.0, ab.alpha, 1e-12);
    EXPECT_NEAR(0.0, ab.beta, 1e-12);
}
