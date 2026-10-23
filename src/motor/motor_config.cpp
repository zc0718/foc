// Conan::ImportStart
#include <foc/motor/motor_config.hpp>
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

} // namespace foc
