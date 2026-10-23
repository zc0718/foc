#include <gtest/gtest.h>

#include <foc/core/observer/ideal_observer.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/types.hpp>



static_assert(foc::core::concepts::is_observer_v<foc::core::IdealObserver>,
              "IdealObserver must satisfy the observer policy contract");



TEST(IdealObserver, ReportsTheTruthExactly) {
    foc::core::IdealObserver observer;
    observer.set_truth(1.234F, 78.5F);

    const foc::core::ObsState state =
        observer.update(foc::core::Ab<float>{99.0F, -99.0F}, foc::core::Ab<float>{1.0F, 1.0F}, 1e-4F);

    // "R.E." = reported exactly: the cheat channel must not filter, scale or delay anything.
    EXPECT_FLOAT_EQ(1.234F, state.theta_e);
    EXPECT_FLOAT_EQ(78.5F, state.omega_e);
}



TEST(IdealObserver, ResetClearsTheTruth) {
    foc::core::IdealObserver observer;
    observer.set_truth(1.0F, 2.0F);

    observer.reset();

    const foc::core::ObsState state = observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, 1e-4F);
    EXPECT_FLOAT_EQ(0.0F, state.theta_e);
    EXPECT_FLOAT_EQ(0.0F, state.omega_e);
}



TEST(IdealObserver, IgnoresElectricalInputs) {
    foc::core::IdealObserver observer;
    observer.set_truth(0.5F, 10.0F);

    const foc::core::ObsState a =
        observer.update(foc::core::Ab<float>{1.0F, 2.0F}, foc::core::Ab<float>{3.0F, 4.0F}, 1e-4F);
    const foc::core::ObsState b =
        observer.update(foc::core::Ab<float>{-1.0F, -2.0F}, foc::core::Ab<float>{-3.0F, -4.0F}, 5e-5F);

    EXPECT_FLOAT_EQ(a.theta_e, b.theta_e);
    EXPECT_FLOAT_EQ(a.omega_e, b.omega_e);
}
