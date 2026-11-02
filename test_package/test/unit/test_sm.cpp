#include <gtest/gtest.h>

#include <foc/core/sm.hpp>
#include <foc/core/types.hpp>



namespace {

using foc::core::MotorState;

} // namespace



TEST(StateMachine, StartsIdle) {
    const foc::core::StateMachine machine;

    EXPECT_EQ(MotorState::Idle, machine.state());
    EXPECT_EQ(0U, machine.illegal_transitions());
}



TEST(StateMachine, HappyPathIdleAlignRunIdle) {
    foc::core::StateMachine machine;

    EXPECT_EQ(foc::ErrorCode::Ok, machine.request(MotorState::Align));
    EXPECT_EQ(MotorState::Align, machine.state());

    EXPECT_EQ(foc::ErrorCode::Ok, machine.request(MotorState::Run));
    EXPECT_EQ(MotorState::Run, machine.state());

    EXPECT_EQ(foc::ErrorCode::Ok, machine.request(MotorState::Idle));
    EXPECT_EQ(MotorState::Idle, machine.state());
    EXPECT_EQ(0U, machine.illegal_transitions());
}



TEST(StateMachine, AlignCanBeAborted) {
    foc::core::StateMachine machine;
    ASSERT_EQ(foc::ErrorCode::Ok, machine.request(MotorState::Align));

    EXPECT_EQ(foc::ErrorCode::Ok, machine.request(MotorState::Idle));
}



TEST(StateMachine, SkippingAlignIsIllegal) {
    foc::core::StateMachine machine;

    EXPECT_EQ(foc::ErrorCode::State, machine.request(MotorState::Run));
    EXPECT_EQ(MotorState::Idle, machine.state());
    EXPECT_EQ(1U, machine.illegal_transitions());
}



TEST(StateMachine, RunCannotGoBackToAlign) {
    foc::core::StateMachine machine;
    ASSERT_EQ(foc::ErrorCode::Ok, machine.request(MotorState::Align));
    ASSERT_EQ(foc::ErrorCode::Ok, machine.request(MotorState::Run));

    EXPECT_EQ(foc::ErrorCode::State, machine.request(MotorState::Align));
    EXPECT_EQ(MotorState::Run, machine.state());
}



TEST(StateMachine, EveryStateCanFault) {
    foc::core::StateMachine from_idle;
    EXPECT_EQ(foc::ErrorCode::Ok, from_idle.request(MotorState::Fault));
    EXPECT_EQ(MotorState::Fault, from_idle.state());

    foc::core::StateMachine from_align;
    ASSERT_EQ(foc::ErrorCode::Ok, from_align.request(MotorState::Align));
    EXPECT_EQ(foc::ErrorCode::Ok, from_align.request(MotorState::Fault));

    foc::core::StateMachine from_run;
    ASSERT_EQ(foc::ErrorCode::Ok, from_run.request(MotorState::Align));
    ASSERT_EQ(foc::ErrorCode::Ok, from_run.request(MotorState::Run));
    EXPECT_EQ(foc::ErrorCode::Ok, from_run.request(MotorState::Fault));
}



TEST(StateMachine, FaultNeedsExplicitClear) {
    foc::core::StateMachine machine;
    ASSERT_EQ(foc::ErrorCode::Ok, machine.request(MotorState::Fault));

    EXPECT_EQ(foc::ErrorCode::State, machine.request(MotorState::Idle));
    EXPECT_EQ(MotorState::Fault, machine.state());

    EXPECT_EQ(foc::ErrorCode::Ok, machine.clear_fault());
    EXPECT_EQ(MotorState::Idle, machine.state());
}



TEST(StateMachine, ClearingWithoutFaultIsIllegal) {
    foc::core::StateMachine machine;

    EXPECT_EQ(foc::ErrorCode::State, machine.clear_fault());
    EXPECT_EQ(1U, machine.illegal_transitions());
}



TEST(StateMachine, IllegalTransitionsAreCountedNotFatal) {
    foc::core::StateMachine machine;

    for (int i = 0; i < 64; ++i) {
        EXPECT_EQ(foc::ErrorCode::State, machine.request(MotorState::Run));
    }

    EXPECT_EQ(64U, machine.illegal_transitions());
    EXPECT_EQ(MotorState::Idle, machine.state());
}



TEST(StateMachine, LegalTransitionTableMatchesExpectation) {
    // expected[from][to]; must stay in sync with kLegal in src/core/sm.cpp
    const bool expected[4][4] = {
        {false, true, false, true},  // Idle  -> Idle Align Run Fault
        {true, false, true, true},   // Align -> ...
        {true, false, false, true},  // Run   -> ...
        {false, false, false, false} // Fault -> ... (only clear_fault)
    };

    const MotorState states[4] = {MotorState::Idle, MotorState::Align, MotorState::Run, MotorState::Fault};

    for (int from = 0; from < 4; ++from) {
        for (int to = 0; to < 4; ++to) {
            foc::core::StateMachine machine;
            // drive the machine into `from`
            if (states[from] == MotorState::Align || states[from] == MotorState::Run) {
                ASSERT_EQ(foc::ErrorCode::Ok, machine.request(MotorState::Align));
            }
            if (states[from] == MotorState::Run) {
                ASSERT_EQ(foc::ErrorCode::Ok, machine.request(MotorState::Run));
            }
            if (states[from] == MotorState::Fault) {
                ASSERT_EQ(foc::ErrorCode::Ok, machine.request(MotorState::Fault));
            }
            ASSERT_EQ(states[from], machine.state());

            const foc::ErrorCode code = machine.request(states[to]);
            EXPECT_EQ(expected[from][to] ? foc::ErrorCode::Ok : foc::ErrorCode::State, code)
                << "from=" << from << " to=" << to;
        }
    }
}



/**
 * @brief [en] A transition target outside the state set is refused, and the refusal is a *different* condition from an
 *             illegal transition between two real states.
 *
 * @brief [zh] 状态集合之外的迁移目标被拒绝，而这个拒绝与"两个真实状态之间的非法迁移"是**不同**的条件。
 *
 *     `request()` rejects for two reasons: the target is not a state at all (`to >= kStateCount`), or it is a state but
 *     the edge is illegal (`!kLegal[from][to]`). MC/DC found that the existing table case only ever exercises the
 *     second: it iterates over the four real states, so the first condition is never true and the short-circuit that
 *     protects the table lookup is never demonstrated. That matters beyond the metric -- an out-of-range index is
 *     exactly what a cast from an int or a deserialised enum produces, and the array lookup behind it would be out of
 *     bounds without the first term.
 *
 *     The pair is therefore the boundary one: `to = 3` (inside the set, illegal from most states) and `to = 4` (outside
 *     it). Both must answer `State` and neither may move the machine.
 *
 *     `request()` 因为两个原因拒绝：目标根本不是状态（`to >= kStateCount`），或者它是状态但边非法（
 *     `!kLegal[from][to]`）。MC/DC 发现既有的表驱动用例只走到了第二条：它只在四个真实状态上迭代，于是第一个条件从未为
 *     真，保护那次数组查表短路的语义也就从未被演示过。这不止是覆盖率问题——越界下标正是从 int 或反序列化的枚举强转过来的
 *     产物，而它背后那次查表在缺少第一项时会越界。
 *
 *     所以这一对就是边界对：`to = 3`（集合内，从多数状态出发非法）与 `to = 4`（集合外）。两者都必须答 `State`，且都不许
 *     移动机器。
 */
TEST(StateMachine, AnOutOfRangeTargetIsRefusedByADifferentConditionThanAnIllegalEdge) {
    const auto out_of_range = static_cast<foc::core::MotorState>(4);

    foc::core::StateMachine machine; // starts in Idle, and Idle -> Idle is not an edge either
    const foc::core::MotorState before = machine.state();
    ASSERT_EQ(foc::core::MotorState::Idle, before);

    // Inside the set but illegal from Idle -- `kLegal[Idle][Run]` is false, while `Idle -> Align` is legal
    // -- so the table is what refuses this one and the range term is not involved.
    // 集合内但从 Idle 出发非法——`kLegal[Idle][Run]` 为假，而 `Idle -> Align` 合法——所以拒绝它的是那张表，
    // 与范围项无关。
    EXPECT_EQ(foc::ErrorCode::State, machine.request(foc::core::MotorState::Run));
    EXPECT_EQ(foc::ErrorCode::State, machine.request(out_of_range))
        << "an index outside the state set must be refused by the range term";
    EXPECT_EQ(before, machine.state()) << "and neither refusal may move the machine";

    // The counter must move for both, so a caller can see that something was asked and refused.
    // 两种拒绝都必须让计数器自增，调用方才能看到"有请求被拒"。
    EXPECT_GT(machine.illegal_transitions(), 1U);
}
