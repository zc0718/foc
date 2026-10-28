// Conan::ImportStart
#pragma once
// Conan::ImportEnd

namespace foc {

namespace hal {


/**
 * @brief [en] Rotor position / velocity source.
 * @brief [zh] 转子位置/速度来源。
 * @since 0.1.0
 * @exporter
 */
class PositionSensor {
  public:
    virtual ~PositionSensor() = default;

    /**
     * @brief [en] Mechanical rotor angle, already freed from mounting offset.
     * @brief [zh] 机械转子角度，已消除安装偏置。
     * @return [en] Angle in rad. A single-turn sensor may normalise to [0, 2*pi); a sensor that can count turns must
     *              return a continuous angle instead, because position control cannot recover turns it was never told
     *              about.
     * @return [zh] 角度，单位 rad。单圈传感器可以归一化到 [0, 2*pi)；能计圈数的传感器必须返回**连续** 角度，因为位置控
     *              制无法还原它没被告知的圈数。
     * @context [en] Current-loop ISR.
     * @context [zh] 电流环 ISR。
     * @note [en] An implementation may offer `set_offset_mech_rad(float)` and `mech_offset_rad() const noexcept ->
     *            float` to own that correction itself; when both exist, `Motor` writes the offset captured by the
     *            alignment routine into them. The pair is optional on purpose -- a sensor whose offset is written
     *            elsewhere at boot simply omits it, and the motor then leaves it untouched (detected by
     *            `core::concepts::has_mech_offset_v`, never assumed).
     * @note [zh] 实现可以提供 `set_offset_mech_rad(float)` 与 `mech_offset_rad() const noexcept -> float` 来自己拥有这
     *            个校正；两者都存在时， `Motor` 会把对齐流程捕获的偏置写进去。这一对故意做成可选——偏置在别处于启动时写
     *            入的 传感器直接不提供即可，电机也不会去动它（由 `core::concepts::has_mech_offset_v` 探测， 从不假设
     *            ）。
     * @since 0.1.0
     */
    virtual auto angle_mech_rad() noexcept -> float = 0;

    /**
     * @brief [en] Mechanical rotor velocity.
     * @brief [zh] 机械转子速度。
     * @return [en] Velocity in rad/s; NaN means "not supported", the observer policy takes over.
     * @return [zh] 速度，单位 rad/s；返回 NaN 表示不支持，由观测器策略接管。
     * @context [en] Current-loop ISR.
     * @context [zh] 电流环 ISR。
     * @since 0.1.0
     */
    virtual auto velocity_rad_s() noexcept -> float = 0;

    /**
     * @brief [en] Whether a physical sensor is present.
     * @brief [zh] 是否存在物理传感器。
     * @return [en] true when a sensor feeds the angles above.
     * @return [zh] true 表示角度来自物理传感器。
     * @context [en] Any context.
     * @context [zh] 任意上下文。
     * @since 0.1.0
     */
    virtual auto has_sensor() const noexcept -> bool = 0;
};

} // namespace hal

} // namespace foc
