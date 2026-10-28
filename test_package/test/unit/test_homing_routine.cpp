// W7-a, increment 5: the search for the index mark, as a policy rather than as a few lines in whichever caller needs it.
//
// The routine decides; it does not actuate. That split is what lets the same code drive a real drive, a SIL rig or a
// bench, and it is why these cases can be decisive without a plant: the whole claim is about what the routine commands
// and when it stops.
//
// W7-a 增量 5：把"找索引标记"做成策略，而不是散在某个需要它的调用方里。
// 这个流程只做**决定**，不做驱动。正是这个切分让同一段代码能驱动真实驱动、SIL 台架或试验台，也是这些用例不必带 plant
// 就能有分辨力的原因：全部主张都在于它下达什么指令、以及在什么时候停下。
//
// The criterion, fixed before the numbers below were measured:
//   C1  an axis that is already homed is done before it starts: `begin()` reports finished, the state is `Done`, and
//       nothing is commanded. Turning a homed axis to look for a mark it is sitting on is a move nobody asked for.
//   C2  searching commands `direction * creep_speed` every period until the mark arrives, then commands exactly zero on
//       the tick it does, and reports finished. No overshoot in the command: the caller applies what it is given.
//   C3  a mark that never arrives ends in `TimedOut` with a zero command, and the timeout is crossed on the period it
//       says it is -- a search that keeps turning after announcing failure is worse than one that never started.
//   C4  an all-zero recipe means "no homing was asked for" (`NotImpl`), and `begin()` then reports finished rather than
//       creeping; a zero is not a malformed recipe, it is the absence of a request.
//   C5  a recipe that is present but impossible is `Param`: zero or negative creep, zero timeout, direction outside
//       +/-1. The two answers must stay distinct for the same reason the observer selector's three do.
//
// 判据在量之前定下：
//   C1 已经回零的轴在开始之前就结束：`begin()` 报已完成、状态 `Done`、不下达任何指令。让一个已回零的轴转起来去找它
//      已经坐着的标记，是一次没人要求过的运动。
//   C2 搜索期间每个周期都下达 `direction * creep_speed`，在标记到来的那一个周期**恰好**下达 0 并报完成。指令不超调：
//      调用方施加什么，就是这里给出的什么。
//   C3 永远不来的标记以 `TimedOut` 结束、指令为 0，且超时正是在它声称的那个周期被跨过——一个宣布失败后还在继续转的
//      搜索，比一个从未开始的更糟。
//   C4 全零配方表示"没有要求回零"（`NotImpl`），此时 `begin()` 报已完成而不是爬行；零不是写错的配方，是没有请求。
//   C5 存在但不可能执行的配方是 `Param`：零或负的爬行速度、零超时、方向不在 ±1。两种回答必须分开，理由与观测器
//      选择器那三种回答相同。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/homing.hpp>
#include <foc/core/position_reference.hpp>

namespace {

namespace core = foc::core;

constexpr float kCreep = 1.5F;
constexpr float kTimeoutS = 0.25F;
constexpr float kDt = 5.0e-5F;
constexpr std::uint8_t kPoles = 7U;

auto recipe(float creep, float timeout, int8_t direction) -> core::HomingConfig {
    return core::HomingConfig{creep, timeout, direction};
}

auto usable_recipe() -> core::HomingConfig { return recipe(kCreep, kTimeoutS, 1); }

auto make_reference() -> core::PositionReference {
    return core::PositionReference(core::PositionReference::Config{kPoles, 0.0F});
}

/** @brief [en] Drive the reference one tick, optionally with the index pulse. */
/** @brief [zh] 推进参考一个周期，可选携带索引脉冲。 */
void advance(core::PositionReference& reference, int step, bool pulse) {
    static_cast<void>(reference.update(0.01F * static_cast<float>(step), pulse));
}

} // namespace

TEST(HomingRoutine, AnAlreadyHomedAxisIsDoneBeforeItStarts) {
    // C1
    core::PositionReference reference = make_reference();
    advance(reference, 1, true); // the mark is here; the origin exists
    ASSERT_TRUE(reference.homed());

    core::HomingRoutine routine(usable_recipe());
    EXPECT_TRUE(routine.begin(reference)) << "a homed axis must not start a search";
    EXPECT_EQ(core::HomingState::Done, routine.state());
    const core::HomingRoutine::Output out = routine.step(reference, kDt);
    EXPECT_FLOAT_EQ(0.0F, out.speed_ref_rad_s) << "nothing may be commanded on an axis that is already referenced";
    EXPECT_TRUE(out.finished);
}

TEST(HomingRoutine, SearchingCommandsTheCreepSpeedAndStopsOnTheExactTickTheMarkArrives) {
    // C2
    for (const int8_t direction : {static_cast<int8_t>(1), static_cast<int8_t>(-1)}) {
        core::PositionReference reference = make_reference();
        core::HomingRoutine routine(recipe(kCreep, kTimeoutS, direction));
        ASSERT_FALSE(routine.begin(reference));
        ASSERT_EQ(core::HomingState::Creep, routine.state());

        // Creep for a while, then deliver the mark.
        // 先爬行一段，再送来标记。
        float commanded = 0.0F;
        for (int i = 1; i <= 100; ++i) {
            advance(reference, i, false);
            commanded = routine.step(reference, kDt).speed_ref_rad_s;
            ASSERT_FLOAT_EQ(static_cast<float>(direction) * kCreep, commanded) << "step " << i;
        }
        ASSERT_FALSE(reference.homed());

        advance(reference, 101, true);
        ASSERT_TRUE(reference.homed());
        const core::HomingRoutine::Output out = routine.step(reference, kDt);
        std::printf("\n[homing] direction %+d: after the mark, commanded %.1f rad/s, state %d, finished %d\n",
                    static_cast<int>(direction), static_cast<double>(out.speed_ref_rad_s),
                    static_cast<int>(out.state), static_cast<int>(out.finished));
        EXPECT_FLOAT_EQ(0.0F, out.speed_ref_rad_s) << "the command must be zero on the tick the origin exists";
        EXPECT_EQ(core::HomingState::Done, out.state);
        EXPECT_TRUE(out.finished);
        EXPECT_EQ(core::HomingFailure::None, routine.failure());
    }
}

TEST(HomingRoutine, AMarkThatNeverArrivesTimesOutAndStopsCommanding) {
    // C3
    core::PositionReference reference = make_reference();
    core::HomingRoutine routine(usable_recipe());
    ASSERT_FALSE(routine.begin(reference));

    const int ticks = static_cast<int>(kTimeoutS / kDt);
    int first_failed_tick = -1;
    for (int i = 1; i <= (ticks + 10); ++i) {
        advance(reference, i, false); // no mark, ever
        const core::HomingRoutine::Output out = routine.step(reference, kDt);
        if (out.finished && (first_failed_tick < 0)) {
            first_failed_tick = i;
            EXPECT_EQ(core::HomingState::Failed, out.state);
            EXPECT_EQ(core::HomingFailure::TimedOut, routine.failure());
            EXPECT_FLOAT_EQ(0.0F, out.speed_ref_rad_s) << "failure must not leave a command behind";
        }
        if (first_failed_tick > 0) {
            EXPECT_EQ(core::HomingState::Failed, out.state) << "the state must stay failed";
            EXPECT_FLOAT_EQ(0.0F, out.speed_ref_rad_s) << "a failed search must not keep turning";
        }
    }
    std::printf("[homing] the timeout was crossed on tick %d, expected %d (+/- 1)\n", first_failed_tick, ticks);
    ASSERT_GT(first_failed_tick, 0);
    EXPECT_LE(std::abs(first_failed_tick - ticks), 1) << "the search ran for longer or shorter than its own recipe";
}

TEST(HomingRoutine, AZeroRecipeMeansNoHomingWasAskedFor) {
    // C4
    core::PositionReference reference = make_reference();
    core::HomingRoutine routine(recipe(0.0F, 0.0F, 0));
    EXPECT_EQ(foc::ErrorCode::NotImpl, routine.validate());
    EXPECT_TRUE(routine.begin(reference)) << "a recipe that asks for nothing must finish rather than creep";
    EXPECT_FLOAT_EQ(0.0F, routine.step(reference, kDt).speed_ref_rad_s);
}

TEST(HomingRoutine, ARecipeThatIsPresentButImpossibleIsAParameterError) {
    // C5
    for (const core::HomingConfig& bad :
         {recipe(0.0F, kTimeoutS, 1), recipe(-1.0F, kTimeoutS, 1), recipe(kCreep, 0.0F, 1), recipe(kCreep, kTimeoutS, 0),
          recipe(kCreep, kTimeoutS, 2)}) {
        core::HomingRoutine routine(bad);
        EXPECT_EQ(foc::ErrorCode::Param, routine.validate())
            << "creep " << bad.creep_speed_rad_s << " timeout " << bad.timeout_s << " dir " << static_cast<int>(bad.direction);
        EXPECT_EQ(core::HomingState::Failed, routine.state());
        EXPECT_EQ(core::HomingFailure::NotConfigured, routine.failure());
    }
}
