// Conan::ImportStart
#pragma once
#include <limits>
#include <foc/hal/position.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {


/**
 * @brief [en] Reference `foc::hal::PositionSensor` implementation: the SIL plant (or a test) writes
 *        the angle and velocity, the control loop reads them.
 *        [zh] `foc::hal::PositionSensor` 的参考实现：SIL 被控对象（或测试）写角度与速度，
 *        控制环读它们。
 *
 * @context [en] Same context as the contract it implements. [zh] 与其实现的契约同上下文。
 * @since 0.1.0
 * @exporter
 */
class SimPosition final : public hal::PositionSensor {
  public:
    SimPosition() noexcept = default;

    /**
     * @brief [en] Set the mechanical angle (normalised to [0, 2*pi) on write).
     *        [zh] 设置机械角度（写入时归一化到 [0, 2π)）。
     * @since 0.1.0
     */
    void set_angle_mech_rad(float angle_rad) noexcept;

    /**
     * @brief [en] Set the mechanical velocity.
     *        [zh] 设置机械速度。
     *
     * @param velocity_rad_s [en] velocity in rad/s; NaN keeps "not simulated".
     *                       [zh] 速度，rad/s；NaN 表示"仍未模拟"。
     * @since 0.1.0
     */
    void set_velocity_rad_s(float velocity_rad_s) noexcept;

    /**
     * @brief [en] Mechanical angle in rad, normalised to [0, 2*pi).
     *        [zh] 机械角度，rad，归一化到 [0, 2π)。
     * @since 0.1.0
     */
    auto angle_mech_rad() noexcept -> float override;

    /**
     * @brief [en] Mechanical velocity in rad/s (NaN when not simulated: use set_velocity_rad_s()).
     *        [zh] 机械速度，rad/s（未设置时为 NaN，请用 set_velocity_rad_s()）。
     * @since 0.1.0
     */
    auto velocity_rad_s() noexcept -> float override;

    /**
     * @brief [en] Always true: this stub stands for a physical sensor.
     *        [zh] 恒为 true：本桩代表一个物理传感器。
     * @since 0.1.0
     */
    [[nodiscard]] auto has_sensor() const noexcept -> bool override;

    /**
     * @brief [en] Model a sensor-less setup: has_sensor() starts returning false.
     *        [zh] 模拟无感场景：has_sensor() 开始返回 false。
     * @since 0.1.0
     */
    void set_sensor_present(bool present) noexcept;

  private:
    float angle_rad_{0.0F};
    float velocity_rad_s_{std::numeric_limits<float>::quiet_NaN()};
    bool sensor_present_{true};
};

} // namespace driver

} // namespace foc
