#include <cmath>
#include <cstdio>

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

namespace {

/** @brief [en] Drive a fresh machine into `target` through legal requests only; false if that is impossible. */
/** @brief [zh] 只用合法请求把一台新机器开进 `target`；做不到则返回 false。 */
auto enter(foc::core::StateMachine& machine, foc::core::MotorState target) -> bool {
  switch (target) {
  case foc::core::MotorState::Idle:
    return true;
  case foc::core::MotorState::Align:
    return machine.request(foc::core::MotorState::Align) == foc::ErrorCode::Ok;
  case foc::core::MotorState::Run:
    return (machine.request(foc::core::MotorState::Align) == foc::ErrorCode::Ok) &&
           (machine.request(foc::core::MotorState::Run) == foc::ErrorCode::Ok);
  case foc::core::MotorState::Fault:
    return machine.request(foc::core::MotorState::Fault) == foc::ErrorCode::Ok;
  }
  return false;
}

} // namespace

TEST(StateMachine, TheTransitionGraphHasSevenEdgesAndFaultIsTheOnlySink) {
  // The table test above checks the cells; this checks the **shape** the cells make, which is what a reader of the
  // class documentation actually relies on. The relation is measured on the code (four start states x four targets),
  // then the graph properties are computed from it: every state reachable from Idle, `Fault` the only state with no
  // outgoing edge, and exactly the seven edges the documented rule accounts for -- `Idle -> Align`, `Idle -> Fault`,
  // `Align -> Run`, `Align -> Idle`, `Align -> Fault`, `Run -> Idle`, `Run -> Fault`.
  // 上面的表用例检查格子；这一个检查格子构成的**形状**，而读类文档的人真正依赖的就是形状。关系在代码上量出（四个起始
  // 状态 × 四个目标），再从它计算图性质：每个状态都可从 Idle 到达、`Fault` 是唯一没有出边的状态、且恰好是文档规则解释
  // 得通的七条边。
  const foc::core::MotorState states[4] = {foc::core::MotorState::Idle, foc::core::MotorState::Align,
                                           foc::core::MotorState::Run, foc::core::MotorState::Fault};
  bool legal[4][4] = {};
  for (int from = 0; from < 4; ++from) {
    for (int to = 0; to < 4; ++to) {
      foc::core::StateMachine machine;
      ASSERT_TRUE(enter(machine, states[from])) << "cannot reach state " << from;
      ASSERT_EQ(states[from], machine.state());
      legal[from][to] = machine.request(states[to]) == foc::ErrorCode::Ok;
    }
  }
  int edges = 0;
  int out_degree[4] = {};
  int in_degree[4] = {};
  for (int from = 0; from < 4; ++from) {
    for (int to = 0; to < 4; ++to) {
      if (legal[from][to]) {
        ++edges;
        ++out_degree[from];
        ++in_degree[to];
      }
    }
  }
  std::printf("[sm] measured relation: %d legal edge(s); out-degree", edges);
  for (int k = 0; k < 4; ++k) {
    std::printf(" %d", out_degree[k]);
  }
  std::printf("; in-degree");
  for (int k = 0; k < 4; ++k) {
    std::printf(" %d", in_degree[k]);
  }
  std::printf("\n");
  // Seven edges, and the count is what makes the documentation checkable: `Idle->Align->Run` is the forward path, the
  // two edges back to `Idle` are the aborts, and three edges lead to `Fault`.
  // 七条边，而这个数正是让文档可核查的东西：`Idle→Align→Run` 是前进路径，回到 `Idle` 的两条边是中止，三条边通向 `Fault`。
  EXPECT_EQ(7, edges);
  // `Fault` is the only sink: nothing leaves it through `request()` -- `clear_fault()` owns that edge, which is why it
  // is not in the table.
  // `Fault` 是唯一的汇：`request()` 出不去——那条边归 `clear_fault()`，所以它不在表里。
  EXPECT_EQ(0, out_degree[3]) << "Fault has an outgoing request() edge after all";
  for (int k = 0; k < 3; ++k) {
    EXPECT_GT(out_degree[k], 0) << "state " << k << " became a dead end";
  }
  // Every live state can fault, and `Fault` itself has no self-loop.
  // 每个活状态都能故障，而 `Fault` 自己也没有自环。
  EXPECT_EQ(3, in_degree[3]) << "not every live state can reach Fault";
  // Breadth-first from Idle over the measured relation: all four states are reachable, so no state is unreachable code.
  // 从 Idle 在量出的关系上做广度优先：四个状态全部可达，所以没有不可达状态的死代码。
  bool seen[4] = {true, false, false, false};
  int frontier[4] = {0, 0, 0, 0};
  frontier[0] = 1;
  for (int round = 0; round < 4; ++round) {
    int next[4] = {0, 0, 0, 0};
    for (int from = 0; from < 4; ++from) {
      if (frontier[from] == 0) {
        continue;
      }
      for (int to = 0; to < 4; ++to) {
        if (legal[from][to] && !seen[to]) {
          seen[to] = true;
          next[to] = 1;
        }
      }
    }
    for (int k = 0; k < 4; ++k) {
      frontier[k] = next[k];
    }
  }
  for (int k = 0; k < 4; ++k) {
    EXPECT_TRUE(seen[k]) << "state " << k << " is not reachable from Idle";
  }
  // And no state has a self-loop through `request()`: `request(current)` is a refusal, not a no-op. That is the sharp
  // edge this measurement found and the header now states -- a supervisory loop that re-asserts its target every cycle
  // accumulates `illegal_transitions` for doing so.
  // 而且没有状态有 `request()` 自环：`request(当前)` 是拒绝而不是空操作。这就是这次测量发现、并由头文件现在写明的那
  // 个尖角——每个周期重申一次目标的管理环，会因此累积 `illegal_transitions`。
  for (int k = 0; k < 4; ++k) {
    EXPECT_FALSE(legal[k][k]) << "state " << k << " became its own legal target";
  }
}

TEST(StateMachine, ARefusalChangesNothingButTheCounter) {
  // The sensitivity that matters for a safety-subset state machine is a **zero**: an illegal request must leave the
  // state exactly where it was and move the counter by exactly one, for every illegal cell and for targets outside the
  // enum. Measured over all sixteen cells plus two out-of-range targets.
  // 对安全子集里的状态机，要紧的敏感度是一个**零**：非法请求必须把状态原地不动、把计数器恰好加一，对每个非法格子、以及
  // 枚举之外的目标都成立。对全部十六个格子加两个越界目标量出。
  const foc::core::MotorState states[4] = {foc::core::MotorState::Idle, foc::core::MotorState::Align,
                                           foc::core::MotorState::Run, foc::core::MotorState::Fault};
  const foc::core::MotorState out_of_range[2] = {static_cast<foc::core::MotorState>(4),
                                                 static_cast<foc::core::MotorState>(200)};
  int refusals = 0;
  for (int from = 0; from < 4; ++from) {
    for (int to = 0; to < 4; ++to) {
      foc::core::StateMachine machine;
      ASSERT_TRUE(enter(machine, states[from]));
      const std::uint32_t before = machine.illegal_transitions();
      const bool refused = machine.request(states[to]) == foc::ErrorCode::State;
      if (!refused) {
        continue;
      }
      ++refusals;
      EXPECT_EQ(states[from], machine.state()) << "a refusal moved the machine from " << from << " to " << to;
      EXPECT_EQ(before + 1U, machine.illegal_transitions()) << "a refusal moved the counter by something else";
    }
    for (const foc::core::MotorState bogus : out_of_range) {
      foc::core::StateMachine machine;
      ASSERT_TRUE(enter(machine, states[from]));
      const std::uint32_t before = machine.illegal_transitions();
      EXPECT_EQ(foc::ErrorCode::State, machine.request(bogus)) << "an out-of-range target was accepted";
      EXPECT_EQ(states[from], machine.state()) << "an out-of-range target moved the machine";
      EXPECT_EQ(before + 1U, machine.illegal_transitions());
    }
  }
  std::printf("[sm] %d of 16 cells refused, plus 8 out-of-range targets, all inert\n", refusals);
  // Nine of sixteen cells are refusals: the seven legal edges are the other seven.
  // 十六格里九格是拒绝：另外七格是那七条合法边。
  EXPECT_EQ(9, refusals);
}

TEST(StateMachine, ReassertingTheCurrentStateCostsAnIllegalTransitionEachTime) {
  // The sharp edge, measured rather than mentioned: `request(current)` is not a no-op, it is an illegal transition. A
  // supervisor that re-asserts "Run" every cycle therefore fills the diagnostic counter with refusals that mean
  // nothing, which is exactly what `illegal_transitions()` is supposed to be used to detect.
  // 那个尖角，量出来而不是提一句：`request(当前)` 不是空操作，是一次非法迁移。于是每个周期重申一次"Run"的管理层会把
  // 诊断计数器填满毫无意义的拒绝——而 `illegal_transitions()` 恰恰是用来发现"有请求被拒"的。
  const foc::core::MotorState states[4] = {foc::core::MotorState::Idle, foc::core::MotorState::Align,
                                           foc::core::MotorState::Run, foc::core::MotorState::Fault};
  for (const foc::core::MotorState state : states) {
    foc::core::StateMachine machine;
    ASSERT_TRUE(enter(machine, state));
    const std::uint32_t before = machine.illegal_transitions();
    for (int k = 0; k < 3; ++k) {
      EXPECT_EQ(foc::ErrorCode::State, machine.request(state)) << "re-asserting state " << static_cast<int>(state);
    }
    EXPECT_EQ(before + 3U, machine.illegal_transitions());
    EXPECT_EQ(state, machine.state());
  }
}
