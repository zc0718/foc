// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/core/pi.hpp>
// Conan::ImportEnd

namespace foc {

namespace motor {


/**
 * @brief [en] Nameplate parameters of one motor, SI units. Populated once at construction.
 *        [zh] 单台电机的铭牌参数，SI 单位。构造时一次填好。
 *
 * @since 0.1.0
 * @exporter
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
 * @exporter
 */
auto psi_f_wb(const motor::MotorConfig& config) noexcept -> float;



/**
 * @brief [en] Current-loop PI for a critically damped bandwidth: `ki = wb^2 * L`, `kp = 2*wb*L - R`.
 *        Substituting the plant pole again gives `s^2 + 2*wb*s + wb^2`, so the closed loop has
 *        no overshoot at all; every gain is derived from the nameplate, never hand-tuned.
 *        [zh] 按临界阻尼带宽给出的电流环 PI：`ki = wb^2 * L`、`kp = 2*wb*L - R`。代回被控对象
 *        极点后正好得到 `s^2 + 2*wb*s + wb^2`，因此闭环无超调；全部增益由铭牌参数推导，不手调。
 *
 * @param config [en] nameplate parameters. [zh] 铭牌参数。
 * @param bandwidth_rad_s [en] target closed-loop bandwidth (> 0). [zh] 目标闭环带宽（> 0）。
 * @param v_bus [en] DC link voltage, used for the output clamp (+-v_bus/2). [zh] 母线电压，用于输出限幅（±v_bus/2）。
 * @return [en] gains and limits; `kp` is floored at 0 for a very resistive machine.
 *         [zh] 增益与限幅；电阻很大的电机 `kp` 会取 0 作为下界。
 * @since 0.1.0
 * @exporter
 */
auto current_pi_config(const motor::MotorConfig& config, float bandwidth_rad_s, float v_bus) noexcept
    -> core::Pi::Config;



/**
 * @brief [en] Speed-loop PI that cancels the mechanical pole `B/J`: `kp = wb*J/Kt`, `ki = wb*B/Kt`.
 *        The loop then collapses to `wb/s`, i.e. a first-order closed loop with time constant
 *        `1/wb`.
 *        [zh] 抵消机械极点 `B/J` 的速度环 PI：`kp = wb*J/Kt`、`ki = wb*B/Kt`。环路随后退化为
 *        `wb/s`，即时间常数为 `1/wb` 的一阶闭环。
 *
 * @param config [en] nameplate parameters. [zh] 铭牌参数。
 * @param bandwidth_rad_s [en] target closed-loop bandwidth (> 0). [zh] 目标闭环带宽（> 0）。
 * @return [en] gains and limits. The integral clamp is a quarter of `i_max_a`, not the full
 *         output range: the PI has no back-calculation, so a wide integral window is what makes a
 *         saturated speed step wind up (see the SIL S2/S4 scenarios).
 *         [zh] 增益与限幅。积分限幅取 `i_max_a` 的四分之一而非满量程：本 PI 没有
 *         back-calculation，积分窗口给满量程正是速度阶跃打满输出后积分饱和的原因（见 SIL 的
 *         S2/S4 场景）。
 * @since 0.1.0
 * @exporter
 */
auto speed_pi_config(const motor::MotorConfig& config, float bandwidth_rad_s) noexcept -> core::Pi::Config;

} // namespace foc
