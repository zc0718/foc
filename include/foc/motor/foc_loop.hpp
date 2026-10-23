// Conan::ImportStart
#pragma once
#include <foc/core/svpwm.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace motor {

using core::Dq;
using core::Uvw;


/**
 * @brief [en] One frame of the current loop, in the order the hardware needs it.
 *        [zh] 电流环的一帧，按硬件需要的顺序给出。
 *
 * @since 0.1.0
 * @exporter
 */
struct LoopFrame final {
    Dq<float> i_dq;  ///< [en] measured current in the rotating frame. [zh] 旋转系实测电流。
    Uvw<float> duty; ///< [en] duty written to the inverter. [zh] 写给逆变器的占空比。
    bool saturated;  ///< [en] true when the modulator clamped. [zh] 调制器钳位时为 true。
};


/**
 * @brief [en] Clarke + Park: measured three-phase currents to the rotating frame.
 *        [zh] Clarke + Park：三相实测电流到旋转系。
 *
 * @param i_uvw [en] phase currents in A. [zh] 三相电流，A。
 * @param theta_e_rad [en] electrical angle in rad. [zh] 电角度，rad。
 * @return [en] d-q currents in A. [zh] d-q 电流，A。
 * @context [en] Current-loop ISR: noexcept, no allocation. [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] auto measure_dq(Uvw<float> i_uvw, float theta_e_rad) noexcept -> Dq<float>;


/**
 * @brief [en] Inverse Park + SVPWM: a voltage command in the rotating frame to three duties.
 *        [zh] 逆 Park + SVPWM：旋转系电压指令到三相占空比。
 *
 * @param v_dq [en] voltage command in V. [zh] 电压指令，V。
 * @param theta_e_rad [en] electrical angle in rad. [zh] 电角度，rad。
 * @param v_bus [en] DC link voltage in V. [zh] 母线电压，V。
 * @return [en] duties plus saturation and status flags. [zh] 占空比以及饱和/状态标志。
 * @context [en] Current-loop ISR: noexcept, no allocation. [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] auto apply_voltage(Dq<float> v_dq, float theta_e_rad, float v_bus) noexcept -> core::SvpwmResult;

} // namespace motor

} // namespace foc
