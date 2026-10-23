// Conan::ImportStart
#pragma once
// Conan::ImportEnd

namespace foc {

namespace hal {

/**
 * @brief [en] Rotor position / velocity source.
 *        [zh] 转子位置/速度来源。
 *
 * @since 0.1.0
 */
class PositionSensor {
  public:
    virtual ~PositionSensor() = default;

    /**
     * @brief [en] Mechanical rotor angle, already freed from mounting offset.
     *        [zh] 机械转子角度，已消除安装偏置。
     *
     * @return [en] Angle in rad, normalised to [0, 2*pi). [zh] 角度，单位 rad，归一化到 [0, 2*pi)。
     * @context [en] Current-loop ISR. [zh] 电流环 ISR。
     * @since 0.1.0
     */
    virtual auto angle_mech_rad() noexcept -> float = 0;

    /**
     * @brief [en] Mechanical rotor velocity.
     *        [zh] 机械转子速度。
     *
     * @return [en] Velocity in rad/s; NaN means "not supported", the observer policy takes over.
     *         [zh] 速度，单位 rad/s；返回 NaN 表示不支持，由观测器策略接管。
     * @context [en] Current-loop ISR. [zh] 电流环 ISR。
     * @since 0.1.0
     */
    virtual auto velocity_rad_s() noexcept -> float = 0;

    /**
     * @brief [en] Whether a physical sensor is present.
     *        [zh] 是否存在物理传感器。
     *
     * @return [en] true when a sensor feeds the angles above. [zh] true 表示角度来自物理传感器。
     * @context [en] Any context. [zh] 任意上下文。
     * @since 0.1.0
     */
    virtual auto has_sensor() const noexcept -> bool = 0;
};

} // namespace hal

} // namespace foc
