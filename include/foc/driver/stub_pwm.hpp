// Conan::ImportStart
#pragma once

#include <cstdint>

#include <foc/core/types.hpp>
#include <foc/hal/pwm.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {


/**
 * @brief [en] Reference `foc::hal::Pwm` implementation: records what was written instead of driving
 *        hardware. Used by SIL and by the unit tests as the default HalPolicy member.
 *        [zh] `foc::hal::Pwm` 的参考实现：不驱动硬件，只记录写入内容。SIL 与单元测试的默认 HalPolicy 成员。
 *
 * @context [en] Same context as the contract it implements. [zh] 与其实现的契约同上下文。
 * @since 0.1.0
 * @exporter
 */
class StubPwm final : public hal::Pwm {
  public:
    StubPwm() noexcept = default;

    /**
     * @brief [en] Record the carrier settings.
     *        [zh] 记录载波配置。
     * @return [en] Always Ok. [zh] 恒为 Ok。
     * @since 0.1.0
     */
    auto configure(float freq_hz, float deadtime_ns) noexcept -> ErrorCode override;

    /**
     * @brief [en] Record the duty of the next period.
     *        [zh] 记录下一周期的占空比。
     * @since 0.1.0
     */
    void write_duty(core::Uvw<float> duty) noexcept override;

    /**
     * @brief [en] Enable the stage.
     *        [zh] 使能输出级。
     * @since 0.1.0
     */
    auto enable() noexcept -> ErrorCode override;

    /**
     * @brief [en] Disable the stage.
     *        [zh] 关闭输出级。
     * @since 0.1.0
     */
    void disable() noexcept override;

    /**
     * @brief [en] Current fault flag (injectable through set_fault()).
     *        [zh] 当前故障标志（可通过 set_fault() 注入）。
     * @since 0.1.0
     */
    [[nodiscard]] auto fault() const noexcept -> bool override;

    /**
     * @brief [en] Clear everything back to the power-on state.
     *        [zh] 复位到上电状态。
     * @since 0.1.0
     */
    void reset() noexcept;

    /**
     * @brief [en] Inject or clear a hardware fault.
     *        [zh] 注入或清除硬件故障。
     * @since 0.1.0
     */
    void set_fault(bool active) noexcept;

    /**
     * @brief [en] Last written duty.
     *        [zh] 最近写入的占空比。
     * @since 0.1.0
     */
    [[nodiscard]] auto last_duty() const noexcept -> core::Uvw<float>;

    /**
     * @brief [en] How many times write_duty() was called.
     *        [zh] write_duty() 被调用的次数。
     * @since 0.1.0
     */
    [[nodiscard]] auto write_count() const noexcept -> std::uint32_t;

    /**
     * @brief [en] Whether the stage is currently enabled.
     *        [zh] 输出级当前是否使能。
     * @since 0.1.0
     */
    [[nodiscard]] auto enabled() const noexcept -> bool;

    /**
     * @brief [en] Configured carrier frequency, 0 before configure().
     *        [zh] 已配置的载波频率，configure() 之前为 0。
     * @since 0.1.0
     */
    [[nodiscard]] auto freq_hz() const noexcept -> float;

  private:
    core::Uvw<float> duty_;
    float freq_hz_{0.0F};
    float deadtime_ns_{0.0F};
    std::uint32_t writes_{0};
    bool configured_{false};
    bool enabled_{false};
    bool fault_{false};
};

} // namespace driver

} // namespace foc
