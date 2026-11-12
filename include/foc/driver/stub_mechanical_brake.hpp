// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/hal/mechanical_brake.hpp>
#include <utility>
// Conan::ImportEnd

/*!
 * @ingroup group_driver
 * @file stub_mechanical_brake.hpp
 *
 * @brief [en] A holding brake a test can drive, including the states where a call fails.
 * @brief [zh] 用例可以驱动的保持抱闸，包括调用失败的那些状态。
 * @since 0.1.0
 */

namespace foc {

namespace driver {

/**
 * @brief [en] Reference `foc::hal::MechanicalBrake` implementation with recording of what it was asked to do.
 * @brief [zh] `foc::hal::MechanicalBrake` 的参考实现，并记录它被要求做过什么。
 * @note [en] It starts **engaged**, which is what a spring-applied brake does at power-on and is the state every
 *            sequencer has to handle first. A stub that starts released would let an enable sequence pass without ever
 *            asking the one question that matters on a vertical axis. `engage()` and `release()` can each be made to
 *            fail, and the counts are kept, because the interesting cases are not "it worked" but "it refused, and what
 *            did the caller do then".
 * @note [zh] 它从 **抱住** 开始，这是弹簧施力抱闸上电时的样子，也是每个编排方首先要处理的状态。一个从"松开"开始的桩件，
 *            会让使能流程在没有问过垂直轴上最要紧的那个问题的情况下通过。 `engage()` 与 `release()` 都能被设为失败，并
 *            保留调用次数，因为有意义的用例不是"它成功了"，而是"它拒绝了，然后调用方做了什么"。
 * @since 0.1.0
 * @exporter
 */
class StubMechanicalBrake final : public hal::MechanicalBrake {
  public:
    StubMechanicalBrake() noexcept = default;

    /**
     * @brief [en] Make the shaft hold.
     * @brief [zh] 让轴抱住。
     * @return [en] `Ok`, or `State` when `set_engage_fault(true)` was called.
     * @return [zh] `Ok`；调用过 `set_engage_fault(true)` 时返回 `State`。
     * @since 0.1.0
     */
    auto engage() noexcept -> ErrorCode override;

    /**
     * @brief [en] Let the shaft turn freely.
     * @brief [zh] 让轴自由转动。
     * @return [en] `Ok`, or `State` when `set_release_fault(true)` was called.
     * @return [zh] `Ok`；调用过 `set_release_fault(true)` 时返回 `State`。
     * @since 0.1.0
     */
    auto release() noexcept -> ErrorCode override;

    /**
     * @brief [en] Whether the brake is holding right now.
     * @brief [zh] 抱闸此刻是否在抱住。
     * @return [en] true while the shaft is held; a failed call leaves it where it was.
     * @return [zh] 轴被抱住时为 true；调用失败时不改变原状态。
     * @since 0.1.0
     */
    [[nodiscard]] auto engaged() noexcept -> bool override;

    /**
     * @brief [en] Make `engage()` refuse from now on.
     * @brief [zh] 从现在起让 `engage()` 拒绝。
     * @param fault [en] true to refuse.
     * @param fault [zh] true 表示拒绝。
     * @since 0.1.0
     */
    void set_engage_fault(bool fault) noexcept;

    /**
     * @brief [en] Make `release()` refuse from now on.
     * @brief [zh] 从现在起让 `release()` 拒绝。
     * @param fault [en] true to refuse.
     * @param fault [zh] true 表示拒绝。
     * @since 0.1.0
     */
    void set_release_fault(bool fault) noexcept;

    /**
     * @brief [en] How many times each call was made, refusals included.
     * @brief [zh] 每种调用发生了多少次，含被拒绝的那些。
     * @return [en] `{engage, release}` counts.
     * @return [zh] `{engage, release}` 两次数。
     * @since 0.1.0
     */
    [[nodiscard]] auto call_counts() const noexcept -> std::pair<std::uint32_t, std::uint32_t>;

    /**
     * @brief [en] Back to the power-on state: engaged, no faults, counters cleared.
     * @brief [zh] 回到上电状态：抱住、无故障、计数清零。
     * @since 0.1.0
     */
    void reset() noexcept;

  private:
    bool engaged_{true};
    bool engage_fault_{false};
    bool release_fault_{false};
    std::uint32_t engages_{0U};
    std::uint32_t releases_{0U};
};

} // namespace driver

} // namespace foc
