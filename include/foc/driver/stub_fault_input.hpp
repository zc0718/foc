// Conan::ImportStart
#pragma once
#include <cstdint>

#include <foc/hal/fault_input.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {


/**
 * @brief [en] Reference `foc::hal::FaultInput`: reports what it was told, and nothing else.
 * @brief [zh] `foc::hal::FaultInput` 的参考实现：告诉它什么就报什么。
 * @note [en] Two roles, and the second is the reason it is in `driver/` rather than in a test. A **simulator** installs
 *            it and never raises it, which is how a host with no power stage writes down "there is no fault path here"
 *            instead of leaving the question unanswered -- `Motor::start()` refuses silence. A
 *            **test or a fault-injection harness** raises it and checks that the drive stops, that `fault_code()` names
 *            the cause, and that the latch survives the pin going quiet again.
 * @note [zh] 两个角色，而第二个才是它在 `driver/` 而不在测试里的理由。 **仿真器** 装上它、从不拉高——这就是没有功率级的
 *            主控方把"这里没有故障通路"写下来的方式，而不是把问题悬着不管：`Motor::start()` 拒绝沉默。**测试或故障注入
 *             harness** 拉高它，然后检查驱动停了、`fault_code()` 说出了成因、以及引脚恢复安静之后锁存仍在。
 * @since 0.1.9
 * @exporter
 */
class StubFaultInput final : public hal::FaultInput {
  public:
    StubFaultInput() noexcept = default;

    /**
     * @brief [en] Assert or clear the fault, the way a pin would.
     * @brief [zh] 像引脚那样置位或清除故障。
     */
    void set_faulted(bool faulted) noexcept {
        faulted_ = faulted;
        ++changes_;
    }

    /** @brief [en] The state the next read will report. [zh] 下一次读取会报的状态。 */
    [[nodiscard]] auto faulted() noexcept -> bool override { return faulted_; }

    /** @brief [en] Whether the last read saw a fault, without advancing anything. [zh] 最近一次读取是否看到故障，不推进任何东西。 */
    [[nodiscard]] auto is_faulted() const noexcept -> bool { return faulted_; }

    /**
     * @brief [en] How many times the injected state changed.
     * @brief [zh] 注入的状态被改过多少次。
     * @note [en] For the harness that has to prove it actually injected: a case that asserted "the pin was raised"
     *            without counting the writes could pass on a harness that never wrote anything.
     * @note [zh] 给那个必须证明"确实注入了"的 harness：只断言"引脚被拉高了"而不数写入次数的用例，会在一个什么都没写的
     *            harness 上通过。
     */
    [[nodiscard]] auto change_count() const noexcept -> std::uint32_t { return changes_; }

    /** @brief [en] Back to the power-on state: not faulted, counter cleared. [zh] 回到上电态：无故障、计数清零。 */
    void reset() noexcept {
        faulted_ = false;
        changes_ = 0U;
    }

  private:
    bool faulted_{false};
    std::uint32_t changes_{0U};
};

} // namespace driver

} // namespace foc
