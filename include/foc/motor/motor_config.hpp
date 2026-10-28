// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/core/feedforward.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/position.hpp>
// Conan::ImportEnd

namespace foc {

namespace motor {


/**
 * @brief [en] Nameplate parameters of one motor, SI units. Populated once at construction.
 *        [zh] 单台电机的铭牌参数，SI 单位。构造时一次填好。
 *
 *        [en] The struct carries both the machine itself and the setup decisions that go with it
 *        (limits, carrier, dead time, loop period). A field of zero there means "use the natural
 *        behaviour": `speed_loop_period_s = 0` runs the speed loop on every tick, which is what
 *        the library did before the field existed.
 *        [zh] 本结构同时携带电机本体与随之而来的设置决策（限幅、载波、死区、环路周期）。其中取值为 0
 *        的字段表示“使用自然行为”：`speed_loop_period_s = 0` 表示每个 tick 都跑速度环，也就是这个
 *        字段出现之前库的行为。
 *
 * @since 0.1.0
 * @exporter
 */
struct MotorConfig final {
    float rs;                        ///< [en] stator resistance in ohm. [zh] 定子电阻，Ω。
    float ld;                        ///< [en] d-axis inductance in H. [zh] d 轴电感，H。
    float lq;                        ///< [en] q-axis inductance in H. [zh] q 轴电感，H。
    float kt;                        ///< [en] torque constant in N*m/A. [zh] 转矩常数，N·m/A。
    float j;                         ///< [en] rotor inertia in kg*m^2. [zh] 转子惯量，kg·m²。
    float b;                         ///< [en] viscous friction in N*m*s. [zh] 粘性摩擦，N·m·s。
    std::uint8_t pole_pairs;         ///< [en] pole pairs, >= 1. [zh] 极对数，>= 1。
    float i_max_a;                   ///< [en] current limit per axis in A. [zh] 每轴电流限幅，A。
    float pwm_freq_hz;               ///< [en] carrier frequency in Hz. [zh] 载波频率，Hz。
    float deadtime_ns;               ///< [en] inverter dead time in ns. [zh] 逆变器死区时间，ns。
    float speed_loop_period_s{0.0F}; ///< [en] speed-loop period in s, 0 = every tick. [zh] 速度环周期，s，0 = 每 tick。
    float position_loop_period_s{
        0.0F}; ///< [en] position-loop period in s, 0 = every tick. [zh] 位置环周期，s，0 = 每 tick。
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
 *         saturated speed step wind up (the current-step and saturation-limit scenarios show it).
 *         [zh] 增益与限幅。积分限幅取 `i_max_a` 的四分之一而非满量程：本 PI 没有
 *         back-calculation，积分窗口给满量程正是速度阶跃打满输出后积分饱和的原因（见 SIL 的
 *         电流阶跃与饱和限幅两个场景）。
 * @since 0.1.0
 * @exporter
 */
auto speed_pi_config(const motor::MotorConfig& config, float bandwidth_rad_s) noexcept -> core::Pi::Config;


/**
 * @brief [en] Feedforward terms derived from the nameplate: Ld, Lq and `psi_f` in one place.
 *        [zh] 由铭牌推导的前馈项：Ld、Lq 与 `psi_f` 在同一处给出。
 *
 *        [en] Exists so a caller never retypes machine constants: the same `MotorConfig` that
 *        tunes the loops also configures the feedforward, and `psi_f` keeps the single definition
 *        in psi_f_wb().
 *        [zh] 存在的意义是调用方永不需要重抄电机常数：整定环路用的那份 `MotorConfig` 同时配置
 *        前馈，而 `psi_f` 只在 psi_f_wb() 里定义一次。
 *
 * @param config [en] nameplate parameters. [zh] 铭牌参数。
 * @return [en] a config carrying both terms (cross-coupling and back-EMF).
 *         [zh] 同时包含交叉耦合与反电动势两项的配置。
 * @since 0.1.0
 * @exporter
 */
auto feedforward_config(const motor::MotorConfig& config) noexcept -> core::DecouplingFeedforward::Config;


/**
 * @brief [en] Position-loop configuration: `gain = bandwidth`, plus both limits.
 *        [zh] 位置环配置：`增益 = 带宽`，外加两道限制。
 *
 *        [en] With the speed loop much faster than the position loop, `gain = w` closes the position
 *        loop as a first-order system of bandwidth `w` (the velocity loop contributes the `1/s`).
 *        There is no integral term: a constant load torque is a disturbance *inside* the speed loop,
 *        whose integrator already removes it, so a second integrator would only add windup. That is
 *        why this controller is a P and the helper is called `position_p_config` -- the old name
 *        claimed a PI that never had an I.
 *        [zh] 速度环远快于位置环时，`gain = w` 让位置环闭环成一阶系统、带宽 `w`（`1/s` 由速度环
 *        提供）。这里没有积分项：恒定负载转矩是速度环**内部**的扰动，速度环积分器已经消掉它，再加
 *        一个积分器只会带来积分饱和。所以这个控制器是 P，辅助函数叫 `position_p_config`——旧名字
 *        声称的 PI 从来没有 I。
 *
 * @param config [en] nameplate parameters. [zh] 铭牌参数。
 * @param bandwidth_rad_s [en] position-loop bandwidth in rad/s (> 0, and well below the speed loop).
 *                        [zh] 位置环带宽，rad/s（> 0，且应明显低于速度环）。
 * @param velocity_limit_rad_s [en] ceiling for the velocity command in rad/s (> 0).
 *                              [zh] 速度指令的上限，rad/s（> 0）。
 * @param accel_limit_rad_s2 [en] rate limit for the velocity command, rad/s^2; 0 = unlimited.
 *                           [zh] 速度指令的加速度上限，rad/s^2；0 = 不限。
 * @return [en] a `PositionP::Config` suitable for the position slot.
 *         [zh] 可直接用于位置槽位的 `PositionP::Config`。
 * @since 0.1.0
 * @exporter
 */
auto position_p_config(const motor::MotorConfig& config, float bandwidth_rad_s, float velocity_limit_rad_s,
                       float accel_limit_rad_s2) noexcept -> core::PositionP::Config;

} // namespace foc
