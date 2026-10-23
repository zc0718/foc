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
