// The session around the four estimators: what it guarantees, and what it refuses to guess.
// 四个估计器外面的那次会话：它保证什么，以及它拒绝猜什么。
//
// The estimators themselves are covered by their own cases (`test_commissioning.cpp`), so nothing here runs a bench:
// the claims are about the *record*, and a case that dragged in a plant would be measuring the plant.
// 估计器本身由它们自己的用例覆盖（`test_commissioning.cpp`），所以这里不跑台架：这里的断言是关于那份 **记录** 的，
// 而一条把 plant 拖进来的用例量的是 plant。

#include <cstdint>

#include <gtest/gtest.h>

#include <foc/core/commissioning_flow.hpp>

namespace {

using foc::core::CommissioningFlow;
using foc::core::CommissioningPlan;
using foc::core::CommissioningStage;
using foc::core::CommissioningState;
using foc::core::kCommissioningStageCount;

auto plan_of(std::initializer_list<CommissioningStage> stages) -> CommissioningPlan {
    CommissioningPlan plan{};
    for (const CommissioningStage stage : stages) {
        plan.order[plan.count] = stage;
        ++plan.count;
    }
    return plan;
}

constexpr auto bit(CommissioningStage stage) -> std::uint8_t {
    return static_cast<std::uint8_t>(1U << static_cast<unsigned>(stage));
}

// The count has to equal the number of enumerators, because the plan's array is sized from it: a fifth stage gets a
// fifth enumerator *and* a bigger count, and a count that drifted from the enumeration would make the array and the
// range check disagree about what a legal stage is.
// 这个计数必须等于枚举项的个数，因为计划的数组就是按它开的：多一段就多一个枚举项 **并且** 计数加一；一个与枚举脱
// 节的计数，会让数组与范围检查对"哪一段合法"各说各话。
static_assert(foc::core::kCommissioningStageCount == 4U, "the enumeration has four stages");

} // namespace


/**
 * @brief [en] The order is enforced in both directions: a stage that is not next cannot start, and a stage that is
 *             already done cannot start again.
 *
 * @brief [zh] 顺序两个方向都被守住：不是下一段的不能开始，已经做完的也不能再来一次。
 *
 *   Both halves are the same bug seen from two sides. Starting out of order makes a number depend on a measurement that
 *   has not happened yet; re-running a stage overwrites a good number with a worse one while the log says
 *   "commissioning succeeded". Neither is caught by any assertion about the numbers themselves, which is why they are
 *   asserted here instead.
 *
 * @note [en] Criterion, written before the run: every wrong `begin()` answers `State` and leaves `next()` and the mask
 *            untouched, and the session only reaches `Done` after the last stage in the plan completes.
 * @note [zh] 判据（写在本次测量之前）：每一次错误的 `begin()` 都答 `State` 且不动 `next()` 与掩码，而会话只有在计划里最
 *            后一段完成之后才到 `Done`。
 */
TEST(CommissioningFlow, AStageThatIsNotNextIsRefusedAndADoneStageCannotStartAgain) {
    CommissioningFlow flow(plan_of({CommissioningStage::Resistance, CommissioningStage::Flux}));

    EXPECT_TRUE(flow.plan_ok());
    ASSERT_EQ(foc::ErrorCode::Ok, flow.begin(CommissioningStage::Resistance));
    EXPECT_EQ(CommissioningState::Running, flow.state());
    // While a stage is under way there is no "next": the caller is inside it.
    EXPECT_FALSE(flow.next().has_value());

    // The stage in progress is the only one that can report.
    EXPECT_EQ(foc::ErrorCode::State, flow.complete(CommissioningStage::Flux));
    EXPECT_EQ(foc::ErrorCode::State, flow.fail(CommissioningStage::Flux));
    ASSERT_EQ(foc::ErrorCode::Ok, flow.complete(CommissioningStage::Resistance));

    EXPECT_EQ(CommissioningState::Idle, flow.state());
    EXPECT_EQ(bit(CommissioningStage::Resistance), flow.completed_mask());
    ASSERT_TRUE(flow.next().has_value());
    EXPECT_EQ(CommissioningStage::Flux, flow.next().value());

    // A stage that already reported cannot be started again, and one that is not next cannot jump the queue.
    EXPECT_EQ(foc::ErrorCode::State, flow.begin(CommissioningStage::Resistance));
    EXPECT_EQ(foc::ErrorCode::State, flow.begin(CommissioningStage::Inductance));
    EXPECT_EQ(bit(CommissioningStage::Resistance), flow.completed_mask());
    ASSERT_TRUE(flow.next().has_value());
    EXPECT_EQ(CommissioningStage::Flux, flow.next().value());

    ASSERT_EQ(foc::ErrorCode::Ok, flow.begin(CommissioningStage::Flux));
    ASSERT_EQ(foc::ErrorCode::Ok, flow.complete(CommissioningStage::Flux));
    EXPECT_EQ(CommissioningState::Done, flow.state());
    EXPECT_FALSE(flow.next().has_value());
    EXPECT_EQ(static_cast<std::uint8_t>(bit(CommissioningStage::Resistance) | bit(CommissioningStage::Flux)),
              flow.completed_mask());
    // Nothing can start once the plan is finished.
    EXPECT_EQ(foc::ErrorCode::State, flow.begin(CommissioningStage::Inductance));
}


/**
 * @brief [en] A failure stops the session, names the stage, and **keeps what was already obtained**.
 *
 * @brief [zh] 失败停住会话、点名是哪一段，并且 **保留已经拿到的东西** 。
 *
 *   This is the case the type exists for. The alternative -- forget everything and start over -- is what an integration
 *   project writes when it has no record type, and it is the expensive one: the injected identification this library
 *   ships carries an irreversible-demagnetisation risk, so re-obtaining an `Rs` that was already measured is risk
 *   bought for tidiness. The mask is what keeps the partial result honest afterwards; without it a stored set cannot
 *   tell "all four stages" from "one stage and a crash".
 *
 * @note [en] Criterion, written before the run: after the failure the mask equals exactly the stages that completed,
 *            `failed_stage()` names the one that gave up, `next()` answers nothing, and no other stage can start until
 *            `reset()` -- which then clears the mask but keeps the plan.
 * @note [zh] 判据（写在本次测量之前）：失败之后掩码恰好等于完成过的那几段， `failed_stage()` 点名放弃的那一段，
 *            `next()` 什么都不答，且在 `reset()` 之前别的段都开不了——而 `reset()` 清掉掩码但保留计划。
 */
TEST(CommissioningFlow, AFailureKeepsWhatWasAlreadyObtainedAndNamesTheStageThatGaveUp) {
    CommissioningFlow flow(plan_of({CommissioningStage::Resistance, CommissioningStage::Flux,
                                    CommissioningStage::Inductance}));
    ASSERT_EQ(foc::ErrorCode::Ok, flow.begin(CommissioningStage::Resistance));
    ASSERT_EQ(foc::ErrorCode::Ok, flow.complete(CommissioningStage::Resistance));
    ASSERT_EQ(foc::ErrorCode::Ok, flow.begin(CommissioningStage::Flux));
    ASSERT_EQ(foc::ErrorCode::Ok, flow.fail(CommissioningStage::Flux));

    EXPECT_EQ(CommissioningState::Failed, flow.state());
    EXPECT_EQ(bit(CommissioningStage::Resistance), flow.completed_mask());
    ASSERT_TRUE(flow.failed_stage().has_value());
    EXPECT_EQ(CommissioningStage::Flux, flow.failed_stage().value());
    EXPECT_FALSE(flow.next().has_value());

    // Nothing resumes past a failure: the stages downstream were measured against an unknown machine.
    EXPECT_EQ(foc::ErrorCode::State, flow.begin(CommissioningStage::Inductance));
    EXPECT_EQ(foc::ErrorCode::State, flow.begin(CommissioningStage::Resistance));
    EXPECT_EQ(foc::ErrorCode::State, flow.complete(CommissioningStage::Inductance));

    flow.reset();
    EXPECT_EQ(CommissioningState::Idle, flow.state());
    EXPECT_EQ(0U, flow.completed_mask());
    EXPECT_FALSE(flow.failed_stage().has_value());
    ASSERT_TRUE(flow.next().has_value());
    EXPECT_EQ(CommissioningStage::Resistance, flow.next().value());
    // The plan survives the reset, which is the whole difference between "forget the session" and "use another plan".
    EXPECT_TRUE(flow.plan_ok());
}


/**
 * @brief [en] A plan that cannot be a plan is refused instead of being repaired.
 *
 * @brief [zh] 描述不出计划的计划被拒绝，而不是被修好。
 *
 *   Four shapes: empty, longer than the enumeration, a stage twice, and a stage that is not in the enumeration at all.
 *   De-duplicating the third one would be the library editing the caller's instruction -- and the caller would never
 *   learn that the estimate it asked for twice is only in the plan once.
 *
 * @note [en] Criterion, written before the run: `plan_ok()` is false for all four, `begin()` answers `Param` rather
 *            than `State` (the plan is wrong, not the timing), and `next()` answers nothing so a caller that ignores
 *            the return of `begin()` still cannot run a stage by accident.
 * @note [zh] 判据（写在本次测量之前）：四种形状的 `plan_ok()` 都为假， `begin()` 答 `Param` 而不是 `State`（错的是计
 *            划，不是时机），且 `next()` 什么都不答，于是忽略 `begin()` 返回值的调用方依然开不了任何一段。
 */
TEST(CommissioningFlow, APlanThatCannotBeAPlanIsRefusedRatherThanRepaired) {
    const CommissioningPlan empty{};
    EXPECT_FALSE(CommissioningFlow(empty).plan_ok());

    CommissioningPlan too_long = plan_of({CommissioningStage::Resistance, CommissioningStage::Flux,
                                          CommissioningStage::CurrentSense, CommissioningStage::Inductance});
    too_long.count = static_cast<std::uint8_t>(kCommissioningStageCount + 1U);
    EXPECT_FALSE(CommissioningFlow(too_long).plan_ok());

    const CommissioningPlan twice =
        plan_of({CommissioningStage::Resistance, CommissioningStage::Resistance});
    EXPECT_FALSE(CommissioningFlow(twice).plan_ok());

    CommissioningPlan invented = plan_of({CommissioningStage::Resistance});
    invented.order[0] = static_cast<CommissioningStage>(kCommissioningStageCount);
    EXPECT_FALSE(CommissioningFlow(invented).plan_ok());

    for (const CommissioningPlan& bad : {empty, too_long, twice, invented}) {
        CommissioningFlow flow(bad);
        EXPECT_EQ(foc::ErrorCode::Param, flow.begin(CommissioningStage::Resistance));
        EXPECT_FALSE(flow.next().has_value());
        EXPECT_EQ(CommissioningState::Idle, flow.state());
        EXPECT_EQ(0U, flow.completed_mask());
    }
}


/**
 * @brief [en] A second stage cannot start while one is already running.
 *
 * @brief [zh] 已经有一段在跑的时候，第二段开不了。
 *
 *   Without this the session would have two stages in flight and one `in_progress_` slot, so the second report would be
 *   refused while the first stage's operator is still measuring -- and the log would say the session is running with no
 *   way to say which stage it means.
 *
 * @note [en] This case exists because the mutation gate found it missing: rewriting the guard's `||` as `&&` left the
 *            whole suite green, which is the definition of an assertion that was not being made.
 * @note [zh] 这条用例是被变异门禁找出来的：把那个守卫里的 `||` 改写成 `&&` 之后整套用例仍是绿的，而这正是"那条断言根本
 *            没被做"的定义。
 */
TEST(CommissioningFlow, ASecondStageCannotStartWhileOneIsAlreadyRunning) {
    CommissioningFlow flow(plan_of({CommissioningStage::Resistance, CommissioningStage::Flux}));
    ASSERT_EQ(foc::ErrorCode::Ok, flow.begin(CommissioningStage::Resistance));

    // Not the next stage, and not a second start of the one in flight: both are `State`, and neither moves anything.
    EXPECT_EQ(foc::ErrorCode::State, flow.begin(CommissioningStage::Flux));
    EXPECT_EQ(foc::ErrorCode::State, flow.begin(CommissioningStage::Resistance));
    EXPECT_EQ(CommissioningState::Running, flow.state());
    EXPECT_FALSE(flow.next().has_value());
    EXPECT_EQ(0U, flow.completed_mask());

    // And the stage in flight still owns the session.
    ASSERT_EQ(foc::ErrorCode::Ok, flow.complete(CommissioningStage::Resistance));
    EXPECT_EQ(CommissioningState::Idle, flow.state());
    ASSERT_TRUE(flow.next().has_value());
    EXPECT_EQ(CommissioningStage::Flux, flow.next().value());
}


/**
 * @brief [en] A plan that uses all four stages is legal, and the four stages are four *different* stages.
 *
 * @brief [zh] 用上全部四段的计划是合法的，而且这四段确实是四个 **不同** 的段。
 *
 *   Two separate ways of being wrong, both of them silent until now:
 *
 *   1. A plan of the full length is the shape that sits on the boundary of the range check, so an off-by-one there
 *   turns the only complete plan a caller can write into a refusal. No other case reaches four stages.
 *
 *   2. If two enumerators ever shared a value their mask bits would collide, and `plan_ok()` would refuse a plan naming
 *   both -- which from the outside looks like "the plan is wrong" rather than "the enumeration is". Running all four
 *   and reading the mask back tells the two apart: the mask has to reach `0b1111`, not stop at three bits.
 *
 * @note [en] Criterion, written before the run: `plan_ok()` is true, every stage starts and completes in order, the
 *            session ends `Done`, and the mask is exactly those four bits.
 * @note [zh] 判据（写在本次测量之前）： `plan_ok()` 为真、每段按顺序开始并完成、会话以 `Done` 结束、掩码恰好是那四位。
 */
TEST(CommissioningFlow, AFullPlanOfAllFourStagesIsLegalAndTheStagesAreDistinct) {
    CommissioningFlow flow(plan_of({CommissioningStage::Resistance, CommissioningStage::Flux,
                                    CommissioningStage::CurrentSense, CommissioningStage::Inductance}));
    ASSERT_TRUE(flow.plan_ok());

    const CommissioningStage all[] = {CommissioningStage::Resistance, CommissioningStage::Flux,
                                      CommissioningStage::CurrentSense, CommissioningStage::Inductance};
    std::uint8_t expected = 0U;
    for (const CommissioningStage stage : all) {
        ASSERT_TRUE(flow.next().has_value());
        EXPECT_EQ(stage, flow.next().value());
        ASSERT_EQ(foc::ErrorCode::Ok, flow.begin(stage));
        ASSERT_EQ(foc::ErrorCode::Ok, flow.complete(stage));
        expected = static_cast<std::uint8_t>(expected | bit(stage));
        // Four stages, four bits: a shared enumerator value shows up here as a mask that never reaches 0x0F.
        EXPECT_EQ(expected, static_cast<std::uint8_t>(expected & 0x0FU));
    }

    EXPECT_EQ(CommissioningState::Done, flow.state());
    EXPECT_EQ(static_cast<std::uint8_t>(0x0FU), flow.completed_mask());
}
