// Conan::ImportStart
#pragma once
#include <array>
#include <cstddef>
#include <cstdint>
#include <foc/core/types.hpp>
#include <foc/hal/pwm.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {


/**
 * @brief [en] Reference `foc::hal::Pwm` implementation: records what was written instead of driving hardware. Used by
 *             SIL and by the unit tests as the default HalPolicy member.
 * @brief [zh] `foc::hal::Pwm` 的参考实现：不驱动硬件，只记录写入内容。SIL 与单元测试的默认 HalPolicy 成员。
 * @context [en] Same context as the contract it implements.
 * @context [zh] 与其实现的契约同上下文。
 * @since 0.1.0
 * @exporter
 */
class StubPwm final : public hal::Pwm {
  public:
    /// [en] Depth of the duty history ring. [zh] 占空比历史环形缓冲的深度。
    static constexpr std::size_t kHistoryCapacity = 32U;

    StubPwm() noexcept = default;

    /**
     * @brief [en] Record the carrier settings.
     * @brief [zh] 记录载波配置。
     * @return [en] Always Ok.
     * @return [zh] 恒为 Ok。
     * @since 0.1.0
     */
    auto configure(float freq_hz, float deadtime_ns) noexcept -> ErrorCode override;

    /**
     * @brief [en] Record the duty of the next period.
     * @brief [zh] 记录下一周期的占空比。
     * @since 0.1.0
     */
    void write_duty(core::Uvw<float> duty) noexcept override;

    /**
     * @brief [en] Enable the stage.
     * @brief [zh] 使能输出级。
     * @since 0.1.0
     */
    auto enable() noexcept -> ErrorCode override;

    /**
     * @brief [en] Disable the stage.
     * @brief [zh] 关闭输出级。
     * @since 0.1.0
     */
    void disable() noexcept override;

    /**
     * @brief [en] Current fault flag (injectable through set_fault()).
     * @brief [zh] 当前故障标志（可通过 set_fault() 注入）。
     * @since 0.1.0
     */
    [[nodiscard]] auto fault() const noexcept -> bool override;

    /**
     * @brief [en] Clear everything back to the power-on state.
     * @brief [zh] 复位到上电状态。
     * @since 0.1.0
     */
    void reset() noexcept;

    /**
     * @brief [en] Inject or clear a hardware fault.
     * @brief [zh] 注入或清除硬件故障。
     * @since 0.1.0
     */
    void set_fault(bool active) noexcept;

    /**
     * @brief [en] Last written duty.
     * @brief [zh] 最近写入的占空比。
     * @since 0.1.0
     */
    [[nodiscard]] auto last_duty() const noexcept -> core::Uvw<float>;

    /**
     * @brief [en] Duty written `back` calls ago (0 = newest), keeping the last `kHistoryCapacity` writes in a fixed
     *             ring buffer.
     * @brief [zh] `back` 次调用之前写入的占空比（0 = 最新），只保留最近 `kHistoryCapacity` 次写入（固定容量环形缓冲）。
     * @param back [en] 0 is the newest write.
     * @param back [zh] 0 为最新一次写入。
     * @return [en] the recorded duty, or an all-zero vector once `back` is outside the recorded range (nothing to
     *              show).
     * @return [zh] 记录到的占空比；`back` 超出已记录范围时返回全零矢量（无内容可看）。
     * @post [en] Once the ring is full the oldest entry is dropped, not an error: a control loop must never be asked to
     *            handle a bookkeeping overflow.
     * @post [zh] 环形缓冲满后丢弃最旧的一条，**不报错**：控制环不该为“记账溢出”付出代价。
     * @since 0.1.0
     */
    [[nodiscard]] auto duty_history(std::size_t back) const noexcept -> core::Uvw<float>;

    /**
     * @brief [en] How many duties are currently recorded, at most kHistoryCapacity.
     * @brief [zh] 当前已记录的占空比数量，最大 kHistoryCapacity。
     * @since 0.1.0
     */
    [[nodiscard]] auto history_count() const noexcept -> std::size_t;

    /**
     * @brief [en] How many times write_duty() was called.
     * @brief [zh] write_duty() 被调用的次数。
     * @since 0.1.0
     */
    [[nodiscard]] auto write_count() const noexcept -> std::uint32_t;

    /**
     * @brief [en] Whether the stage is currently enabled.
     * @brief [zh] 输出级当前是否使能。
     * @since 0.1.0
     */
    [[nodiscard]] auto enabled() const noexcept -> bool;

    /**
     * @brief [en] Configured carrier frequency, 0 before configure().
     * @brief [zh] 已配置的载波频率，configure() 之前为 0。
     * @since 0.1.0
     */
    [[nodiscard]] auto freq_hz() const noexcept -> float;

  private:
    core::Uvw<float> duty_;
    std::array<core::Uvw<float>, kHistoryCapacity> history_{};
    std::size_t history_count_{0};
    std::size_t history_next_{0};
    float freq_hz_{0.0F};
    float deadtime_ns_{0.0F};
    std::uint32_t writes_{0};
    bool configured_{false};
    bool enabled_{false};
    bool fault_{false};
};

} // namespace driver

} // namespace foc
