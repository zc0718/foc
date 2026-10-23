#include <cmath>

#include <gtest/gtest.h>

#include "sil_pmsm_plant.hpp"

#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>



namespace {

constexpr float kDt = 2e-5F; // plant step, 50 kHz

auto plant_params() -> foc::sil::PlantParams {
    // rs, ld, lq, kt, j, b, pole_pairs
    return foc::sil::PlantParams{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-5F, 1e-3F, 7U};
}

auto idle_state() -> foc::sil::PlantState { return foc::sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F}; }

void run(foc::sil::PmsmPlant& plant, float seconds) {
    const int steps = static_cast<int>(seconds / kDt);
    for (int i = 0; i < steps; ++i) {
        plant.advance(kDt);
    }
}

} // namespace



TEST(PmsmPlant, ElectricalStepResponseMatchesTheAnalyticRL) {
    const foc::sil::PlantParams params = plant_params();
    foc::sil::PmsmPlant plant(params);
    plant.reset(idle_state());
    plant.set_voltage_dq(foc::core::Dq<float>{1.0F, 0.0F}); // vd = 1 V, rotor cannot move (iq = 0)

    const float tau = params.ld / params.rs; // 0.8 ms
    const float target = 1.0F / params.rs;   // 2 A

    float elapsed = 0.0F;
    for (const float multiples : {1.0F, 2.0F, 4.0F}) {
        const float until = multiples * tau;
        run(plant, until - elapsed);
        elapsed = until;

        const float expected = target * (1.0F - std::exp(-elapsed / tau));
        EXPECT_NEAR(expected, plant.current_dq().d, 0.01F * target) << "t = " << elapsed;
    }

    EXPECT_NEAR(0.0F, plant.current_dq().q, 1e-6F);
    EXPECT_NEAR(0.0F, plant.omega_mech_rad_s(), 1e-6F);
}



TEST(PmsmPlant, MechanicalSteadyStateSatisfiesTheTorqueBalance) {
    const foc::sil::PlantParams params = plant_params();
    foc::sil::PmsmPlant plant(params);
    plant.reset(idle_state());
    plant.set_voltage_dq(foc::core::Dq<float>{0.0F, 5.0F});

    run(plant, 0.2F); // mechanical time constant is J/B = 10 ms

    const float omega = plant.omega_mech_rad_s();
    const float torque = plant.torque_nm();

    EXPECT_GT(omega, 0.0F);
    // At steady state: Te - B*wm - T_load = 0
    EXPECT_NEAR(0.0F, torque - (params.b * omega), 1e-4F);
}



TEST(PmsmPlant, LoadTorqueReducesTheSteadySpeed) {
    const foc::sil::PlantParams params = plant_params();

    foc::sil::PmsmPlant free_running(params);
    free_running.reset(idle_state());
    free_running.set_voltage_dq(foc::core::Dq<float>{0.0F, 5.0F});
    run(free_running, 0.2F);

    foc::sil::PmsmPlant loaded(params);
    loaded.reset(idle_state());
    loaded.set_voltage_dq(foc::core::Dq<float>{0.0F, 5.0F});
    loaded.set_load_torque(0.05F);
    run(loaded, 0.2F);

    EXPECT_GT(free_running.omega_mech_rad_s(), loaded.omega_mech_rad_s());
}



TEST(PmsmPlant, PositiveTorqueWithPositiveSpeedDeliversMechanicalPower) {
    const foc::sil::PlantParams params = plant_params();
    foc::sil::PmsmPlant plant(params);
    plant.reset(idle_state());
    plant.set_voltage_dq(foc::core::Dq<float>{0.0F, 5.0F});

    run(plant, 0.2F);

    const float mechanical_power = plant.torque_nm() * plant.omega_mech_rad_s();
    EXPECT_GT(plant.torque_nm(), 0.0F);
    EXPECT_GT(plant.omega_mech_rad_s(), 0.0F);
    EXPECT_GT(mechanical_power, 0.0F);
}



TEST(PmsmPlant, PhaseCurrentsRoundTripBackToTheDqFrame) {
    const foc::sil::PlantParams params = plant_params();
    foc::sil::PmsmPlant plant(params);
    plant.reset(idle_state());
    plant.set_voltage_dq(foc::core::Dq<float>{1.0F, 2.0F});

    run(plant, 0.05F);

    const float theta_e = static_cast<float>(params.pole_pairs) * plant.theta_mech_rad();
    const foc::core::Dq<float> rebuilt = foc::core::park(foc::core::clarke(plant.current_uvw()), theta_e);
    const foc::core::Dq<float> actual = plant.current_dq();

    EXPECT_NEAR(actual.d, rebuilt.d, 1e-4F);
    EXPECT_NEAR(actual.q, rebuilt.q, 1e-4F);
}



TEST(PmsmPlant, RotorFluxFollowsTheTorqueConstant) {
    const foc::sil::PlantParams params = plant_params();
    const foc::sil::PmsmPlant plant(params);

    const float expected = params.kt / (1.5F * static_cast<float>(params.pole_pairs));
    EXPECT_NEAR(expected, plant.psi_f_wb(), 1e-9F);
}



TEST(PmsmPlant, NonPositiveStepIsIgnored) {
    foc::sil::PmsmPlant plant(plant_params());
    plant.reset(idle_state());

    plant.advance(0.0F);
    plant.advance(-1.0F);

    EXPECT_FLOAT_EQ(0.0F, plant.state().id_a);
}
