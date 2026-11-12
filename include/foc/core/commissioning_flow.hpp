// Conan::ImportStart
#pragma once
#include <array>
#include <cstdint>
#include <foc/core/types.hpp>
#include <optional>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file commissioning_flow.hpp
 *
 * @brief [en] The session around the four estimators: which stage runs now, and what survives a failure.
 * @brief [zh] 四个估计器外面的一次会话：现在该跑哪一段，以及失败之后还剩下什么。
 * @since 0.1.0
 */

namespace foc {

namespace core {


/**
 * @brief [en] The four things a commissioning session can identify.
 * @brief [zh] 一次标定会话可以辨识的四件事。
 * @note [en] The enumerators are the estimators that exist, not a taxonomy: a fifth estimator gets a fifth enumerator,
 *            which is why this is an enum and not a string.
 * @note [zh] 枚举项就是已有的那几个估计器，不是一套分类法：多一个估计器就多一个枚举项，这也正是它是枚举而不是字符串的原
 *            因。
 * @since 0.1.0
 * @exporter
 */
enum class CommissioningStage : std::uint8_t {
    /**
     * @brief [en] stator resistance (and the inverter's dead time), `RsEstimator`.
     * @brief [zh] 定子电阻（以及逆变器死区）， `RsEstimator`。
     */
    Resistance = 0,
    /**
     * @brief [en] rotor flux, `FluxEstimator`.
     * @brief [zh] 转子磁链， `FluxEstimator`。
     */
    Flux = 1,
    /**
     * @brief [en] the current-sense gain and offset, `CurrentSenseCalibrator`.
     * @brief [zh] 电流采样的增益与偏置， `CurrentSenseCalibrator`。
     */
    CurrentSense = 2,
    /**
     * @brief [en] incremental inductance, `IncrementalInductanceEstimator`.
     * @brief [zh] 增量电感， `IncrementalInductanceEstimator`。
     */
    Inductance = 3,
};

/**
 * @brief [en] How many stages the enumeration has.
 * @brief [zh] 这个枚举有多少段。
 * @since 0.1.0
 */
inline constexpr std::size_t kCommissioningStageCount = 4U;

/**
 * @brief [en] The stages one machine needs, in the order this caller wants them run.
 * @brief [zh] 这台机器需要哪几段，按这个调用方希望的顺序。
 * @note [en] The order is the caller's, and that is a deliberate limit: the dependencies between the estimators are
 *            physical facts about a machine (a flux estimate that divides by a resistance measured at another
 *            temperature is a different number), so the library does not pretend to know them. What it does know is
 *            whether the order was followed -- see `CommissioningFlow`.
 * @note [zh] 顺序由调用方给，这是刻意的界限：估计器之间的依赖是关于某台机器的物理事实（在另一个温度下测得的电阻去除出来
 *            的磁链是另一个数），所以库不假装知道它们。它知道的是这个顺序有没有被遵守——见 `CommissioningFlow`。
 * @since 0.1.0
 * @exporter
 */
struct CommissioningPlan final {
    /**
     * @brief [en] the stages to run, in order; entries past `count` are ignored.
     * @brief [zh] 要跑的段，按顺序； `count` 之后的项被忽略。
     */
    std::array<CommissioningStage, kCommissioningStageCount> order{};
    /**
     * @brief [en] how many of `order` are used, 1..4.
     * @brief [zh] `order` 里用了几个，1..4。
     */
    std::uint8_t count{0U};
};

/**
 * @brief [en] Where a session is.
 * @brief [zh] 一次会话走到哪儿了。
 * @note [en] `Running` is a state of its own on purpose: a session that was interrupted has one stage that started and
 *            never reported, and a host that reloads after a reset has to be able to tell "not started" from "started
 *            and lost".
 * @note [zh] `Running` 是有意单独存在的状态：被打断的会话有一段是"开始了但没汇报"的，而复位后重新载入的宿主必须分得清 "
 *            没开始"与"开始了但丢了"。
 * @since 0.1.0
 * @exporter
 */
enum class CommissioningState : std::uint8_t {
    /**
     * @brief [en] waiting for the next stage to be started.
     * @brief [zh] 等下一段开始。
     */
    Idle,
    /**
     * @brief [en] a stage has been started and has not reported.
     * @brief [zh] 某一段已经开始，还没汇报。
     */
    Running,
    /**
     * @brief [en] every stage in the plan completed.
     * @brief [zh] 计划里的每一段都完成了。
     */
    Done,
    /**
     * @brief [en] a stage reported a failure; `completed_mask()` says how far it got.
     * @brief [zh] 某一段报了失败； `completed_mask()` 说明走到哪一步。
     */
    Failed,
};

/**
 * @brief [en] One commissioning session: the order enforced, the partial result kept, the stopping point named.
 *
 *   Three things a session has to guarantee and a caller cannot see it doing, which is why it is a type rather than a
 *   paragraph in an integration guide:
 *
 *   1. **Each stage runs once, in the declared order.** Starting a stage that is not next is refused, and so is
 *   starting the same stage twice: a re-run silently overwrites a good number with a worse one, and the caller who
 *   re-runs it is usually looking at the symptom, not the cause.
 *
 *   2. **A failure stops the session but not the facts.** `completed_mask()` keeps what was obtained before the
 *   failure, because a partially commissioned machine is still information -- and because re-obtaining it is not free:
 *   the injected identification this library ships carries an irreversible-demagnetisation risk, so the results already
 *   in hand are worth more than the tidiness of forgetting them.
 *
 *   3. **The stopping point is a stage, not a boolean.** `failed_stage()` names which stage gave up, so a log line says
 *   "flux failed" rather than "commissioning failed", and the next run knows where to start looking.
 *
 *   What it does **not** do: it does not run the estimators, it does not own them, and it does not know the physics. It
 *   is a session record -- which is exactly the piece that integration projects were each writing for themselves.
 *
 * @brief [zh] 一次标定会话：守住顺序、留下部分结果、点名停在哪一段。
 *
 *   一个会话必须保证而调用方看不见的三件事，这正是它是个类型而不是集成手册里一段话的原因：
 *
 *   1. **每段只跑一次，按声明的顺序。** 开始一段"还不是下一段"的会被拒绝，同一段开始两次也会：重跑会把一个好数字静默地
 *   换成更差的，而重跑的人通常在看症状，不是在看原因。
 *
 *   2. **失败停住会话，但不停住事实。** `completed_mask()` 保留失败之前已经拿到的东西，因为一台标定了一半的机器仍然是信
 *   息——也因为重新拿并不免费：本库自带的注入式辨识带着不可逆退磁的风险，所以手里已有的结果比"忘掉它更整齐" 值钱。
 *
 *   3. **停住的地方是一段，不是一个布尔。** `failed_stage()` 说清是哪一段放弃的，于是一条日志写的是"磁链失败"而不是 "标
 *   定失败"，下一次也就知道从哪儿查起。
 *
 *   它 **不** 做的事：不跑估计器、不拥有它们、也不懂物理。它是一份会话记录——而这恰好是每个集成项目都在自己写的那一块。
 *
 * @context [en] Commissioning only: not the control loop, no allocation, no HAL.
 * @context [zh] 仅标定阶段：不在控制环里、不分配、不碰 HAL。
 * @since 0.1.0
 * @exporter
 */
class CommissioningFlow final {
  public:
    /**
     * @brief [en] Take the plan. A plan that cannot be a plan is refused by `begin()`, which is where a caller is
     *             looking anyway.
     * @brief [zh] 接住计划。描述不出计划的计划由 `begin()` 拒绝——反正调用方看的是那里。
     * @param plan [en] the stages, in order.
     * @param plan [zh] 要跑的段，按顺序。
     * @since 0.1.0
     */
    explicit CommissioningFlow(const CommissioningPlan& plan) noexcept : plan_(plan) {}

    /**
     * @brief [en] Start a stage.
     * @brief [zh] 开始一段。
     * @param stage [en] the stage the caller is about to run.
     * @param stage [zh] 调用方即将跑的那一段。
     * @return [en] `Ok`; `Param` for a plan that is not a plan (empty, too long, or carrying a stage twice); `State`
     *              when the session is finished, failed, already running a stage, or when `stage` is not the next one.
     * @return [zh] `Ok`；计划不成计划（空的、太长的、或同一段出现两次）返回 `Param`；会话已结束、已失败、已经有一段在
     *              跑、或 `stage` 不是下一段时返回 `State`。
     * @post [en] `state()` becomes `Running` and `next()` answers nothing until the stage reports.
     * @post [zh] `state()` 变成 `Running`，在该段汇报之前 `next()` 什么也不答。
     * @since 0.1.0
     */
    auto begin(CommissioningStage stage) noexcept -> ErrorCode {
        if (!plan_ok()) {
            return ErrorCode::Param;
        }
        if ((state_ != CommissioningState::Idle) || (next_index_ >= plan_.count) ||
            (plan_.order[next_index_] != stage)) {
            return ErrorCode::State;
        }
        state_ = CommissioningState::Running;
        in_progress_ = stage;
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] Report that the stage in progress succeeded.
     * @brief [zh] 汇报正在跑的那一段成功了。
     * @param stage [en] the stage that finished.
     * @param stage [zh] 跑完的那一段。
     * @return [en] `Ok`; `State` when no stage is in progress or when `stage` is not the one that is.
     * @return [zh] `Ok`；没有段在跑、或 `stage` 不是正在跑的那一段时返回 `State`。
     * @post [en] the stage joins `completed_mask()`; the session returns to `Idle`, or becomes `Done` when it was the
     *            last one in the plan.
     * @post [zh] 这一段并入 `completed_mask()`；会话回到 `Idle`，若它是计划里的最后一段则变成 `Done`。
     * @since 0.1.0
     */
    auto complete(CommissioningStage stage) noexcept -> ErrorCode {
        if ((state_ != CommissioningState::Running) || (in_progress_ != stage)) {
            return ErrorCode::State;
        }
        completed_mask_ = static_cast<std::uint8_t>(completed_mask_ | bit(stage));
        in_progress_.reset();
        ++next_index_;
        state_ = (next_index_ >= plan_.count) ? CommissioningState::Done : CommissioningState::Idle;
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] Report that the stage in progress failed.
     * @brief [zh] 汇报正在跑的那一段失败了。
     * @param stage [en] the stage that gave up.
     * @param stage [zh] 放弃的那一段。
     * @return [en] `Ok`; `State` when no stage is in progress or when `stage` is not the one that is.
     * @return [zh] `Ok`；没有段在跑、或 `stage` 不是正在跑的那一段时返回 `State`。
     * @post [en] `completed_mask()` is **unchanged** -- what was already obtained stays obtained -- and the session is
     *            `Failed` until `reset()`. Starting anything else after a failure is refused rather than resumed: the
     *            numbers downstream of a stage that failed were measured against a machine in an unknown state.
     * @post [zh] `completed_mask()` **不变** ——已经拿到的仍然算拿到——会话在 `reset()` 之前都是 `Failed`。失败之后不允许
     *            开始别的段，只能重置：失败那一段之后的数字，是在一台状态未知的机器上测出来的。
     * @since 0.1.0
     */
    auto fail(CommissioningStage stage) noexcept -> ErrorCode {
        if ((state_ != CommissioningState::Running) || (in_progress_ != stage)) {
            return ErrorCode::State;
        }
        failed_ = stage;
        in_progress_.reset();
        state_ = CommissioningState::Failed;
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] The stage to run now, or nothing when there is none.
     * @brief [zh] 现在该跑的那一段；没有时什么都不答。
     * @return [en] the next stage while `Idle`; `nullopt` while `Running` (a stage is already under way), and after
     *              `Done` or `Failed`, or when the plan is not a plan.
     * @return [zh] `Idle` 时是下一段； `Running` 时（已经有一段在跑）、 `Done`/`Failed` 之后、以及计划不成计划时为
     *              `nullopt`。
     * @since 0.1.0
     */
    [[nodiscard]] auto next() const noexcept -> std::optional<CommissioningStage> {
        if (!plan_ok() || (state_ != CommissioningState::Idle) || (next_index_ >= plan_.count)) {
            return std::nullopt;
        }
        return plan_.order[next_index_];
    }

    /**
     * @brief [en] Which stages have completed, one bit per stage, and it survives a failure.
     * @brief [zh] 哪几段完成了，每段一位，且它在失败之后依然成立。
     * @return [en] bits `1 << static_cast<std::uint8_t>(stage)`.
     * @return [zh] 位 `1 << static_cast<std::uint8_t>(stage)`。
     * @note [en] This is the field that keeps a partial session from impersonating a complete one: `state() == Done` is
     *            the claim "everything the plan asked for was obtained", and a caller that stores only the numbers
     *            without this mask cannot tell the two apart afterwards.
     * @note [zh] 这个字段是"部分完成的会话不得冒充完整会话"的地方： `state() == Done` 声称"计划里要的都拿到了"，而一个
     *            只存数字、不存这个掩码的调用方，事后就分不清这两者。
     * @since 0.1.0
     */
    [[nodiscard]] auto completed_mask() const noexcept -> std::uint8_t { return completed_mask_; }

    /**
     * @brief [en] The stage that gave up, if one did.
     * @brief [zh] 放弃的那一段，如果有。
     * @since 0.1.0
     */
    [[nodiscard]] auto failed_stage() const noexcept -> std::optional<CommissioningStage> { return failed_; }

    /**
     * @brief [en] Where the session is.
     * @brief [zh] 会话走到哪儿了。
     * @since 0.1.0
     */
    [[nodiscard]] auto state() const noexcept -> CommissioningState { return state_; }

    /**
     * @brief [en] The plan in force.
     * @brief [zh] 当前生效的计划。
     * @since 0.1.0
     */
    [[nodiscard]] auto plan() const noexcept -> const CommissioningPlan& { return plan_; }

    /**
     * @brief [en] Whether the plan describes a session at all: 1..4 stages, none of them twice, none of them invented.
     * @brief [zh] 这份计划到底描述的是一次会话吗：1..4 段，没有哪一段出现两次，也没有哪一段是编出来的。
     * @return [en] true when `begin()` will be able to make progress.
     * @return [zh] `begin()` 能往前走时为 true。
     * @note [en] A repeated stage in the plan is refused rather than de-duplicated: collapsing `Rs, Rs` into one stage
     *            would be the library quietly editing a caller's instruction, and the caller would never learn that the
     *            estimate it asked for twice is only in the plan once.
     * @note [zh] 计划里重复的段被拒绝而不是去重：把 `Rs, Rs` 合并成一段，等于库悄悄改写了调用方的指令，而调用方永远不会
     *            知道它要求做两次的那个估计只排在计划里一次。
     * @since 0.1.0
     */
    [[nodiscard]] auto plan_ok() const noexcept -> bool {
        if ((plan_.count == 0U) || (plan_.count > kCommissioningStageCount)) {
            return false;
        }
        std::uint8_t seen = 0U;
        for (std::uint8_t i = 0U; i < plan_.count; ++i) {
            if (static_cast<unsigned>(plan_.order[i]) >= kCommissioningStageCount) {
                return false;
            }
            const std::uint8_t flag = bit(plan_.order[i]);
            if ((seen & flag) != 0U) {
                return false;
            }
            seen = static_cast<std::uint8_t>(seen | flag);
        }
        return true;
    }

    /**
     * @brief [en] Forget the session, keeping the plan.
     * @brief [zh] 丢掉这次会话，计划保留。
     * @post [en] `Idle`, nothing completed, nothing failed, `next()` is the first stage again.
     * @post [zh] `Idle`，没有完成项、没有失败项， `next()` 又是第一段。
     * @since 0.1.0
     */
    void reset() noexcept {
        state_ = CommissioningState::Idle;
        next_index_ = 0U;
        completed_mask_ = 0U;
        in_progress_.reset();
        failed_.reset();
    }

  private:
    [[nodiscard]] static auto bit(CommissioningStage stage) noexcept -> std::uint8_t {
        return static_cast<std::uint8_t>(1U << static_cast<unsigned>(stage));
    }

    CommissioningPlan plan_{};
    CommissioningState state_{CommissioningState::Idle};
    std::uint8_t next_index_{0U};
    std::uint8_t completed_mask_{0U};
    std::optional<CommissioningStage> in_progress_{};
    std::optional<CommissioningStage> failed_{};
};

} // namespace core

} // namespace foc
