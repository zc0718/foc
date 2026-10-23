// Conan::ImportStart
#pragma once

#include <cstdint>
// Conan::ImportEnd

namespace foc {

namespace motor {

/**
 * @brief [en] Nameplate parameters of one motor, SI units. Populated once at construction.
 *        [zh] 单台电机的铭牌参数，SI 单位。构造时一次填好。
 *
 * @since 0.1.0
 */
struct MotorConfig final {
    float rs;                ///< [en] stator resistance in ohm. [zh] 定子电阻，Ω。
    float ld;                ///< [en] d-axis inductance in H. [zh] d 轴电感，H。
    float lq;                ///< [en] q-axis inductance in H. [zh] q 轴电感，H。
    float kt;                ///< [en] torque constant in N*m/A. [zh] 转矩常数，N·m/A。
    float j;                 ///< [en] rotor inertia in kg*m^2. [zh] 转子惯量，kg·m²。
    float b;                 ///< [en] viscous friction in N*m*s. [zh] 粘性摩擦，N·m·s。
    std::uint8_t pole_pairs; ///< [en] pole pairs, >= 1. [zh] 极对数，>= 1。
    float i_max_a;           ///< [en] current limit per axis in A. [zh] 每轴电流限幅，A。
    float pwm_freq_hz;       ///< [en] carrier frequency in Hz. [zh] 载波频率，Hz。
    float deadtime_ns;       ///< [en] inverter dead time in ns. [zh] 逆变器死区时间，ns。
};

} // namespace motor

/**
 * @brief [en] Rotor flux linkage implied by the torque constant: `psi_f = Kt / (1.5 * pole_pairs)`.
 *        Needed to seed a flux-linkage observer during the align phase.
 *        [zh] 由转矩常数反推的转子磁链：`psi_f = Kt / (1.5 * pole_pairs)`。用于在对齐阶段为
 *        磁链观测器播种。
 *
 * @return [en] flux linkage in Wb, or 0 when pole_pairs is 0. [zh] 磁链，Wb；极对数为 0 时返回 0。
 * @since 0.1.0
 */
auto psi_f_wb(const motor::MotorConfig& config) noexcept -> float;

} // namespace foc
