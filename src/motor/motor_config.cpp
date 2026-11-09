// Conan::ImportStart
#include <foc/motor/motor_config.hpp>

#include <foc/core/limits.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_motor
 * @file motor_config.cpp
 *
 * @brief [en] Implements the nameplate admission and the gain derivations the motor is constructed from.
 * @brief [zh] 实现铭牌准入，以及电机构造所依据的增益推导。
 * @since 0.1.0
 */

namespace foc {


/**
 * @brief [en] Implements psi_f_wb(); the contract is documented on its declaration.
 * @brief [zh] 实现 psi_f_wb()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto psi_f_wb(const motor::MotorConfig& config) noexcept -> float {
    if (config.pole_pairs == 0U) {
        return 0.0F;
    }

    const float one_and_half = 1.5F;
    return config.kt / (one_and_half * static_cast<float>(config.pole_pairs));
}


/**
 * @brief [en] Implements current_pi_config(); the contract is documented on its declaration.
 * @brief [zh] 实现 current_pi_config()；契约文档在对应的声明处。
 * @attacher
 * @see `the case named below` case `ApiPiGains.TheGainsFollowTheDesignRuleAndTheLoopDeliversThem`, ledger `E-CURPI-1`
 * @since 0.1.0
 */
auto current_pi_config(const motor::MotorConfig& config, float bandwidth_rad_s, float v_bus) noexcept
    -> core::Pi::Config {
    const float two = 2.0F;
    const float half = 0.5F;
    const float kp = core::clamp((two * bandwidth_rad_s * config.ld) - config.rs, 0.0F, 1.0e6F);
    const float ki = bandwidth_rad_s * bandwidth_rad_s * config.ld;
    const float v_lim = half * v_bus;
    return core::Pi::Config{kp, ki, -v_lim, v_lim, -v_lim, v_lim};
}


/**
 * @brief [en] Implements speed_pi_config(); the contract is documented on its declaration.
 * @brief [zh] 实现 speed_pi_config()；契约文档在对应的声明处。
 * @attacher
 * @see `the case named below` case `AutoTune.IdentifiesTheMachineItWasToldAboutAndLandsOnTheNameplateGains`, ledger `E-SPDPI-1`
 * @since 0.1.0
 */
auto speed_pi_config(const motor::MotorConfig& config, float bandwidth_rad_s) noexcept -> core::Pi::Config {
    const float quarter = 0.25F;
    const float kp = bandwidth_rad_s * config.j / config.kt;
    const float ki = bandwidth_rad_s * config.b / config.kt;
    const float lim = config.i_max_a;
    const float integral_lim = quarter * config.i_max_a;
    return core::Pi::Config{kp, ki, -lim, lim, -integral_lim, integral_lim};
}


/**
 * @brief [en] Implements feedforward_config(); the contract is documented on its declaration.
 * @brief [zh] 实现 feedforward_config()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto feedforward_config(const motor::MotorConfig& config) noexcept -> core::DecouplingFeedforward::Config {
    return core::DecouplingFeedforward::Config{config.ld, config.lq, psi_f_wb(config),
                                               core::DecouplingFeedforward::Terms::CrossAndBemf};
}


/**
 * @brief [en] Implements position_p_config(); the contract is documented on its declaration.
 * @brief [zh] 实现 position_p_config()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto position_p_config(const motor::MotorConfig& config, float bandwidth_rad_s, float velocity_limit_rad_s,
                       float accel_limit_rad_s2) noexcept -> core::PositionP::Config {
    static_cast<void>(config); // the gains follow from the bandwidth alone; no nameplate term here
    return core::PositionP::Config{bandwidth_rad_s, velocity_limit_rad_s, accel_limit_rad_s2};
}

} // namespace foc
