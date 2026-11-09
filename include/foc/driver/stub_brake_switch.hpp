// Conan::ImportStart
#pragma once
#include <cstdint>

#include <foc/hal/brake_switch.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {


/**
 * @brief [en] Reference `foc::hal::BrakeSwitch`: records the duty instead of driving hardware.
 * @brief [zh] `foc::hal::BrakeSwitch` 的参考实现：不驱动硬件，只记录占空比。
 * @note [en] Two callers need the same thing for different reasons. A unit test asks "what did the policy write"; the
 *            SIL bench asks the same question and then turns the answer into current through a resistor, which is the
 *            only way a brake chopper can be observed at all -- a chopper whose output nobody converts to current is a
 *            state machine with a nice printout.
 * @note [zh] 两个调用方出于不同的理由需要同一件事。单元测试问"策略写了什么"；SIL 台架问同一个问题，然后把答案变成流过一
 *            个电阻的电流——这是斩波器唯一可被观测的方式：一个输出没人换算成电流的斩波器，只是一个打印得很好看的状态机。
 * @since 0.1.0
 * @exporter
 */
class StubBrakeSwitch final : public hal::BrakeSwitch {
  public:
    StubBrakeSwitch() noexcept = default;

    /**
     * @brief [en] Record the chopping frequency; `<= 0` is refused as `Param`.
     * @brief [zh] 记录斩波频率； `<= 0` 被拒绝为 `Param`。
     */
    auto configure(float freq_hz) noexcept -> ErrorCode override;

    /**
     * @brief [en] Record the duty **verbatim**, without clamping.
     * @brief [zh] **原样** 记录占空比，不限幅。
     * @note [en] Verbatim on purpose: the contract says the caller clamps, so a stub that silently clamped would hide
     *            exactly the defect it exists to make visible.
     * @note [zh] 故意原样记录：契约说由调用方限幅，所以一个悄悄限幅的桩会把它本该暴露的缺陷藏起来。
     */
    void write_duty(float duty) noexcept override;

    /** @brief [en] Open the switch. [zh] 断开开关。 */
    void disable() noexcept override;

    /** @brief [en] The last duty written; 0 after `disable()` or before any write. [zh] 最近写入的占空比；`disable()` 之后或从未写入时为 0。 */
    [[nodiscard]] auto duty() const noexcept -> float;

    /** @brief [en] Whether a non-zero duty is being held. [zh] 当前是否保持着非零占空比。 */
    [[nodiscard]] auto enabled() const noexcept -> bool;

    /** @brief [en] How many times `write_duty()` was called. [zh] `write_duty()` 被调用的次数。 */
    [[nodiscard]] auto write_count() const noexcept -> std::uint32_t;

    /** @brief [en] The configured chopping frequency, 0 before `configure()`. [zh] 已配置的斩波频率，`configure()` 之前为 0。 */
    [[nodiscard]] auto freq_hz() const noexcept -> float;

  private:
    float freq_hz_{0.0F};
    float duty_{0.0F};
    std::uint32_t writes_{0U};
    bool enabled_{false};
};

} // namespace driver

} // namespace foc
