#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>



static_assert(foc::core::concepts::is_observer_v<foc::core::BackEmfObserver>,
              "BackEmfObserver must satisfy the observer policy contract");



namespace {

// Synthetic surface-mounted PMSM drive, exact signals so the observer can be judged on its own.
constexpr float kPsiM = 0.01F;    // Wb, rotor flux linkage
constexpr float kOmegaE = 100.0F; // rad/s electrical
constexpr float kRs = 0.5F;       // ohm
constexpr float kL = 4e-4F;       // H
constexpr float kI = 1.0F;        // A, current amplitude
constexpr float kPhi = 0.3F;      // rad, current phase offset
constexpr float kDt = 1e-5F;      // s, 100 kHz for a tight integration

auto wrap_pi(float angle) -> float {
    while (angle <= -foc::core::kPi<float>) {
        angle += foc::core::kTwoPi<float>;
    }
    while (angle > foc::core::kPi<float>) {
        angle -= foc::core::kTwoPi<float>;
    }
    return angle;
}

/**
 * @brief Advances the synthetic drive one step and returns the true electrical angle.
 */
auto drive_step(float& t, foc::core::Ab<float>& v_ab, foc::core::Ab<float>& i_ab) -> float {
    const float theta = kOmegaE * t;
    const float cos_t = std::cos(theta + kPhi);
    const float sin_t = std::sin(theta + kPhi);

    i_ab = foc::core::Ab<float>{kI * cos_t, kI * sin_t};

    // di/dt = I * omega * [-sin(theta+phi), cos(theta+phi)]
    const float di_alpha = -kI * kOmegaE * sin_t;
    const float di_beta = kI * kOmegaE * cos_t;

    // back-EMF of the rotor flux: e = psi_m * omega * [-sin(theta), cos(theta)]
    const float e_alpha = -kPsiM * kOmegaE * std::sin(theta);
    const float e_beta = kPsiM * kOmegaE * std::cos(theta);

    v_ab = foc::core::Ab<float>{kRs * i_ab.alpha + kL * di_alpha + e_alpha, kRs * i_ab.beta + kL * di_beta + e_beta};
    t += kDt;
    return theta;
}

} // namespace



TEST(BackEmfObserver, TracksAngleWhenSeededWithInitialFlux) {
    foc::core::BackEmfObserver observer({kRs, kL});
    ASSERT_EQ(foc::ErrorCode::Ok, observer.status());

    // At t = 0 the rotor flux points along alpha: psi(0) = psi_m * [1, 0].
    observer.seed_flux(foc::core::Ab<float>{kPsiM, 0.0F});

    float t = 0.0F;
    float theta_true = 0.0F;
    foc::core::ObsState state{};
    const int steps = static_cast<int>(foc::core::kTwoPi<float> / (kOmegaE * kDt));

    for (int i = 0; i < 2 * steps; ++i) {
        foc::core::Ab<float> v_ab{};
        foc::core::Ab<float> i_ab{};
        theta_true = drive_step(t, v_ab, i_ab);
        state = observer.update(v_ab, i_ab, kDt);

        if (i > steps) { // one electrical cycle of settling
            EXPECT_LT(std::fabs(wrap_pi(state.theta_e - theta_true)), 1e-2F) << "step " << i;
        }
    }

    EXPECT_NEAR(kOmegaE, state.omega_e, 0.05F * kOmegaE);
}



TEST(BackEmfObserver, NonPositiveDtIsRejectedAndHolds) {
    foc::core::BackEmfObserver observer({kRs, kL});
    ASSERT_EQ(foc::ErrorCode::Ok, observer.status());

    const foc::core::ObsState state = observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, 0.0F);

    EXPECT_EQ(foc::ErrorCode::Param, observer.status());
    EXPECT_FLOAT_EQ(0.0F, state.theta_e);

    // the latched error keeps holding even when a valid dt arrives...
    const foc::core::ObsState after = observer.update(foc::core::Ab<float>{1.0F, 1.0F}, foc::core::Ab<float>{}, kDt);
    EXPECT_FLOAT_EQ(state.theta_e, after.theta_e);
    EXPECT_EQ(foc::ErrorCode::Param, observer.status());

    // ...until reset() clears it.
    observer.reset();
    EXPECT_EQ(foc::ErrorCode::Ok, observer.status());
}



TEST(BackEmfObserver, InvalidParametersAreRejected) {
    const foc::core::BackEmfObserver zero_inductance({kRs, 0.0F});
    EXPECT_EQ(foc::ErrorCode::Param, zero_inductance.status());

    const foc::core::BackEmfObserver negative_resistance({-1.0F, kL});
    EXPECT_EQ(foc::ErrorCode::Param, negative_resistance.status());
}



TEST(BackEmfObserver, SeedAndResetControlTheFluxState) {
    foc::core::BackEmfObserver observer({kRs, kL});

    observer.seed_flux(foc::core::Ab<float>{kPsiM, 0.0F});
    EXPECT_FLOAT_EQ(kPsiM, observer.flux().alpha);

    observer.reset();
    EXPECT_FLOAT_EQ(0.0F, observer.flux().alpha);
    EXPECT_FLOAT_EQ(0.0F, observer.flux().beta);
}



TEST(BackEmfObserver, SpeedEstimateWrapsAcrossTheAngleBoundary) {
    foc::core::BackEmfObserver observer({0.0F, kL}); // Rs = 0 keeps the arithmetic readable
    observer.seed_flux(foc::core::Ab<float>{1.0F, 0.0F});

    // First step only primes the angle reference.
    static_cast<void>(observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, kDt));

    // Drive the flux from the +alpha axis into the third quadrant within a single step, so the
    // angle difference between samples exceeds pi and the wrap has to kick in.
    const float dt = 1e-3F;
    const foc::core::ObsState state =
        observer.update(foc::core::Ab<float>{-2000.0F, -1000.0F}, foc::core::Ab<float>{}, dt);

    // atan2(-1, -1) = -3*pi/4, wrapped to +3*pi/4 by the angle normaliser, so the delta is -3*pi/4.
    EXPECT_NEAR(-2.35619F, state.omega_e * dt, 1e-3F);
}
