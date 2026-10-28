// S12 - compliance: the joint pushed by an external load. Two knobs decide what happens, and the
// scenario is written to show which one actually does the work.
// S12 - 柔顺性：被外部负载推动的关节。两个旋钮决定结果，本场景的写法就是为了指出到底是哪一个在起作用。

#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/motion_profile.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_bench.hpp"
#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"



namespace {

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 4;
constexpr float kVBus = 24.0F;
constexpr float kTargetRad = 0.20F;
constexpr float kLoadTorqueNm = 8e-3F;

using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                 foc::core::NullReferenceGenerator, foc::core::NullFeedforward, foc::core::PositionP>;

/**
 * @brief Settle on the target, then apply a load torque, and report how far the rotor gave way.
 */
auto sag_under_load(float kp_scale, float ilimit_scale) -> float {
    const foc::motor::MotorConfig mc = foc::sil::nominal_motor_config();
    foc::sil::PmsmPlant plant(foc::sil::plant_params_from(foc::sil::nominal_spec()));
    foc::sil::PlantHal hal(plant, kVBus);

    const typename MotorT::Policies policies{
        foc::core::BackEmfObserver({mc.rs, mc.ld}),
        foc::core::Svpwm{},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::current_pi_config(mc, 2'000.0F, kVBus)},
        foc::core::Pi{foc::speed_pi_config(mc, 250.0F)},
        foc::core::NullReferenceGenerator{},
        foc::core::NullFeedforward{},
        foc::core::PositionP{foc::position_p_config(mc, 20.0F, 50.0F, 0.0F)}};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    static_cast<void>(motor.start());

    // target, iq ceiling, iq feedforward, v limit, a limit, kp_scale, kd_scale, ilimit_scale
    const foc::motor::Command command{kTargetRad, 0.0F, 0.0F, 0.0F, 0.0F, kp_scale, 0.0F, ilimit_scale};
    motor.set_command(command);

    // Settle on target first.
    // 先稳定在目标上。
    const int settle_steps = static_cast<int>(0.5F / kDt);
    for (int i = 0; i < settle_steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
    }
    const float settled = hal.position.true_angle_mech_rad();

    // Now lean on it.
    // 现在压上去。
    plant.set_load_torque(kLoadTorqueNm);
    const int load_steps = static_cast<int>(0.3F / kDt);
    for (int i = 0; i < load_steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
    }
    return settled - hal.position.true_angle_mech_rad();
}

} // namespace



/**
 * @brief [en] Measured facts first: a constant load always produces some sag, and the two knobs move it in the same
 *             direction, by different amounts.
 * @brief [zh] 先摆事实：恒定负载总会产生一定的下垂，而两个旋钮把它往同一个方向推，只是幅度不同。
 * @note [en] The stiff case is not zero, and writing down *why* matters more than the number: the position loop is
 *            proportional, so it can only answer a load by commanding a velocity, and the speed integrator absorbs that
 *            load only while there is a velocity error to integrate. The sag is therefore the integral's wind-up
 *            transient, and it settles slowly.
 * @note [zh] 刚性工况也不是 0，而且写下**为什么**比写下数字更重要：位置环是比例环节，它只能靠"发出一个 速度"来回应负
 *            载，而速度积分器只在存在速度误差时才吸收负载。所以这段下垂就是积分器的建立过程， 收敛得很慢。
 */
TEST(SilCompliance, AConstantLoadAlwaysProducesSomeSag) {
    const float stiff = sag_under_load(0.0F, 0.0F);
    EXPECT_GT(stiff, 5e-3F) << "a proportional position loop cannot hold a load at exactly zero error";
    EXPECT_LT(stiff, 2e-2F) << "and the integrator still does most of the work: " << stiff << " rad";
}



TEST(SilCompliance, SofteningTheStiffnessIncreasesTheSag) {
    // Monotonic in the knob, which is what makes it usable as a compliance setting. The values are
    // the measured ones for this rig; the assertion is the ordering, not the numbers.
    // 随旋钮单调变化，这正是它能当柔顺参数使用的原因。数值是本台架的实测值；断言的是大小顺序，不是数值。
    const float full = std::fabs(sag_under_load(0.0F, 0.0F));      // configured stiffness
    const float fifth = std::fabs(sag_under_load(0.2F, 0.0F));     // 20 % stiffness
    const float twentieth = std::fabs(sag_under_load(0.05F, 0.0F)); // 5 % stiffness

    EXPECT_LT(full, fifth) << full << " vs " << fifth;
    EXPECT_LT(fifth, twentieth) << fifth << " vs " << twentieth;
}



TEST(SilCompliance, ReleasingTheIntegralAlsoIncreasesTheSag) {
    // The second knob works too, and it works in the same direction -- but in this rig it is the
    // weaker of the two, which is worth stating out loud instead of asserting a theory that the
    // measurement does not support.
    // 第二个旋钮同样有效，方向相同——但在本台架上它是较弱的那一个，这值得直接说出来，而不是去断言一个
    // 实测并不支持的理论。
    const float stiff = std::fabs(sag_under_load(0.0F, 0.0F));
    const float released = std::fabs(sag_under_load(0.0F, 0.001F));
    EXPECT_GT(released, stiff) << released << " vs " << stiff;

    // And it is a smaller effect than the stiffness knob here: documented, measured, not assumed.
    // 而它的作用在本台架上小于刚度旋钮：这是记录下来的实测，不是假设。
    EXPECT_LT(released, std::fabs(sag_under_load(0.2F, 0.0F)));
}
