// Conan::ImportStart
#include <foc/motor/foc_loop.hpp>

#include <foc/core/transforms.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_motor
 * @file foc_loop.cpp
 *
 * @brief [en] Implements one frame of the control loop in the order the hardware needs it: sample, observe, regulate, modulate.
 * @brief [zh] 按硬件需要的次序实现控制环的一帧：采样、观测、调节、调制。
 * @since 0.1.0
 */

namespace foc {

namespace motor {


/**
 * @brief [en] Implements measure_dq(); the contract is documented on its declaration.
 * @brief [zh] 实现 measure_dq()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto measure_dq(Uvw<float> i_uvw, float theta_e_rad) noexcept -> Dq<float> {
    return core::park(core::clarke(i_uvw), theta_e_rad);
}


/**
 * @brief [en] Implements apply_voltage(); the contract is documented on its declaration.
 * @brief [zh] 实现 apply_voltage()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto apply_voltage(Dq<float> v_dq, float theta_e_rad, float v_bus) noexcept -> core::SvpwmResult {
    return core::Svpwm::modulate(core::inv_park(v_dq, theta_e_rad), v_bus);
}

} // namespace motor

} // namespace foc
