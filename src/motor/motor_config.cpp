// Conan::ImportStart
#include <foc/motor/motor_config.hpp>

#include <foc/core/limits.hpp>
// Conan::ImportEnd

namespace foc {


/**
 * @brief [en] Implements psi_f_wb(); the contract is documented on its declaration.
 *        [zh] 实现 psi_f_wb()；契约文档在对应的声明处。
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
 *        [zh] 实现 current_pi_config()；契约文档在对应的声明处。
 * @attacher
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
 *        [zh] 实现 speed_pi_config()；契约文档在对应的声明处。
 * @attacher
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

} // namespace foc
