// Conan::ImportStart
#pragma once
#include <cstdint>

#include <foc/core/position_reference.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] What the homing search is doing right now.
 * @brief [zh] 回零搜索当前在做什么。
 * @since 0.1.6
 * @exporter
 */
enum class HomingState : std::uint8_t {
    Idle = 0,   ///< [en] not started or reset. [zh] 未开始或已复位。
    Creep = 1,  ///< [en] turning slowly, waiting for the index mark. [zh] 慢速旋转，等待索引标记。
    Done = 2,   ///< [en] the origin is established. [zh] 原点已建立。
    Failed = 3, ///< [en] the search gave up. [zh] 搜索放弃。
};


/**
 * @brief [en] Why a homing search stopped.
 * @brief [zh] 一次回零搜索为什么停下。
 * @since 0.1.6
 * @exporter
 */
enum class HomingFailure : std::uint8_t {
    None = 0,          ///< [en] no failure. [zh] 无失败。
    NotConfigured = 1, ///< [en] the recipe was rejected. [zh] 配方被拒。
    TimedOut = 2,      ///< [en] the mark was never seen. [zh] 从未见到标记。
};


/**
 * @brief [en] Recipe for the search that turns slowly until the index mark arrives.
 * @brief [zh] "慢速转直到索引到来"这一搜索的配方。
 * @note [en] Every number is the caller's, for the same reason `StartupConfig`'s are: what a safe creep speed is
 *            depends on the mechanism, and a plausible-looking default would be indistinguishable from a considered
 *            one.
 * @note [zh] 每个数字都由调用方给出，理由与 `StartupConfig` 相同：安全的爬行转速取决于机构，一个看起来合理的默认值与一
 *            个经过考虑的值无法区分。
 * @since 0.1.6
 * @exporter
 */
struct HomingConfig final {
    float creep_speed_rad_s; ///< [en] |speed| while searching, rad/s (> 0). [zh] 搜索时的转速绝对值，rad/s（> 0）。
    float timeout_s;         ///< [en] give up after this long, s (> 0). [zh] 超过这个时间就放弃，s（> 0）。
    int8_t direction;        ///< [en] +1 or -1; which way to turn. [zh] +1 或 -1；朝哪边转。
};


/**
 * @brief [en] Decides how fast to turn while looking for the index mark; it does not actuate anything.
 *
 * @brief [zh] 决定找索引标记期间以多快转动；它不驱动任何东西。
 *
 *     Why it is separate from `Motor`: `Motor` composes its policies statically and has no homing slot, and adding one
 *     would make every motor instantiation carry a search it may never run. The decision itself is small and worth
 *     testing on its own -- the caller reads `speed_ref_rad_s` out of `step()` and applies it, which is also what lets
 *     the same routine drive a real drive, a SIL rig or a bench without any of them being special. `Motor` 静态组合策略
 *     且没有回零槽位，加一个会让每个电机实例都携带一次可能永远不会用的搜索。这个决策本身很小，值得单独测试——调用方从
 *     `step()` 读出 `speed_ref_rad_s` 并施加，这也让同一个流程能驱动真实驱动、SIL 台架或试验台，而不需要其中任何一个"特
 *     殊"。
 *
 * @note [en] The only failure it reports is the timeout. Losing the turn count while searching is **not** a failure
 *            here: the search exists precisely because the origin is unknown, and the mark is what restores the count
 *            (see `PositionReference::lost_count()`). A routine that gave up on a lost count would refuse to search
 *            whenever the axis was bumped before homing, which is the normal case.
 * @note [zh] 它唯一报告的是超时。搜索期间丢失圈计数在这里**不是**失败：搜索之所以存在正是因为原点未知，而标记正是恢复计
 *            数的东西（见 `PositionReference::lost_count()`）。一个因为丢计数就放弃的流程，会在轴回零前被动过时拒绝搜索
 *            ——而那正是常态。
 * @since 0.1.6
 * @exporter
 */
class HomingRoutine final {
  public:
    /**
     * @brief [en] What the caller should apply this period.
     * @brief [zh] 调用方本周期应当施加的量。
     * @since 0.1.6
     * @exporter
     */
    struct Output final {
        float speed_ref_rad_s{0.0F}; ///< [en] speed command, 0 unless searching. [zh] 转速指令，非搜索期间为 0。
        HomingState state{HomingState::Idle};
        bool finished{false}; ///< [en] `Done` or `Failed`; either way the caller stops here. [zh] `Done` 或 `Failed`。
    };

    explicit HomingRoutine(HomingConfig config) noexcept;

    /**
     * @brief [en] Whether the recipe can be carried out at all.
     * @brief [zh] 这个配方能否被执行。
     * @return [en] `FOC_ERR_NOT_IMPL` when every field is zero -- the documented way of saying "no homing was asked
     *              for", as `StartupRoutine::validate()` does; `FOC_ERR_PARAM` for any non-positive or non-finite
     *              value, or a direction that is not +1 or -1; otherwise `Ok`.
     * @return [zh] 所有字段为零时返回 `FOC_ERR_NOT_IMPL`——与 `StartupRoutine::validate()` 一样，这是"没有要求回零" 的书
     *              面写法；出现非正、非有限的值或方向不是 +1/-1 时返回 `FOC_ERR_PARAM`；否则 `Ok`。
     * @since 0.1.6
     */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode;

    /**
     * @brief [en] Start searching, or finish immediately if the origin already exists.
     * @brief [zh] 开始搜索；若原点已存在则立即结束。
     * @param reference [en] the reference the search is looking for an origin in.
     * @param reference [zh] 搜索要在其中找原点的参考。
     * @return [en] `true` when the search is over before it starts (`Done`), `false` when it has begun creeping.
     * @return [zh] 搜索尚未开始就已结束（`Done`）时返回 `true`；已开始爬行时返回 `false`。
     * @note [en] Already homed means already done. Turning a homed axis to look for a mark it is already sitting on
     *            would be a move nobody asked for.
     * @note [zh] 已回零即已完成。让一个已回零的轴转起来去找它已经坐着的标记，是一次没人要求过的运动。
     * @since 0.1.6
     */
    auto begin(const PositionReference& reference) noexcept -> bool;

    /**
     * @brief [en] Advance one control period.
     * @brief [zh] 推进一个控制周期。
     * @param reference [en] the reference being homed.
     * @param reference [zh] 正在回零的参考。
     * @param dt_s [en] control period in s; a non-positive value leaves the state untouched.
     * @param dt_s [zh] 控制周期，s；非正值不改变状态。
     * @return [en] The speed to apply and the state afterwards.
     * @return [zh] 应当施加的转速，以及推进之后的状态。
     * @context [en] Current-loop ISR: noexcept, no allocation.
     * @context [zh] 电流环 ISR：noexcept、不分配。
     * @since 0.1.6
     */
    auto step(const PositionReference& reference, float dt_s) noexcept -> Output;

    /** @brief [en] Current state. [zh] 当前状态。 */
    [[nodiscard]] auto state() const noexcept -> HomingState { return state_; }

    /** @brief [en] Why it stopped. [zh] 为什么停下。 */
    [[nodiscard]] auto failure() const noexcept -> HomingFailure { return failure_; }

    /** @brief [en] Time spent searching so far, s. [zh] 目前已搜索的时间，s。 */
    [[nodiscard]] auto elapsed_s() const noexcept -> float { return elapsed_s_; }

    /** @brief [en] Back to `Idle` with the timer cleared, keeping the recipe. [zh] 回到 `Idle` 并清空计时，保留配方。 */
    void reset() noexcept;

  private:
    HomingConfig config_;
    HomingState state_;
    HomingFailure failure_;
    float elapsed_s_;
};


} // namespace core

} // namespace foc
