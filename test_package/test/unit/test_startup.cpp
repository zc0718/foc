// Open-loop start-up: the four answers a recipe can give, and what each state outputs.
//
// `StartupRoutine` had no unit test at all -- only a SIL run -- so the states after Align, the refusal paths and
// `reset()` were never executed by anything. These cases assert what the header promises in each of them.
//
// 开环起动：配方能给出的四种答案，以及每个状态输出什么。
// `StartupRoutine` 之前**没有**单元测试，只有 SIL 运行，因此 Align 之后的状态、拒绝路径与 `reset()` 从未被执行过。
//
// The criteria, fixed before the numbers below were measured:
//   C1  an all-zero recipe is `NotImpl`, not `Param` -- "this build does not start this way" is an answer the
//       caller acts on (fall back to Align -> Run), and a blank struct is not a user error.
//   C2  one zero field in an otherwise usable recipe is `Param`: "not requested" and "requested wrongly" have to
//       be distinguishable, or a typo silently disables the feature.
//   C3  `begin()` refuses a zero pole count and a bad recipe, and both leave `Failed` behind, so a caller that
//       ignores the return value cannot step an executable state.
//   C4  a tick with dt <= 0 or NaN asks for nothing: the output stays zeroed and no state moves.
//   C5  Align holds a still vector at `align_current_a` and captures the offset from the mechanical angle handed
//       in on the **last** hold tick; Pull ramps at `pull_accel_rad_s2`; Done is reached on the first tick whose
//       vector speed passes `handover_speed_rad_s`, and that tick reports `finished`.
//   C6  Done keeps answering the last vector (a caller ticking once more gets a defined current, not zeros), and
//       Failed reports `failed` instead of going quiet.
//   C7  `reset()` returns to Idle and forgets the captured offset, so a restart does not inherit the last run's
//       rotor position.
//
// 判据写在量之前：C1 零配方是 NotImpl；C2 半张配方是 Param；C3 begin 拒绝并落 Failed；C4 无效 dt 什么都不要；
// C5 Align 静止、在保持的最后一拍捕获偏置、Pull 按声明的加速度爬升、第一拍越过交接转速即 Done 并报 finished；
// C6 Done 继续回答上一个矢量、Failed 明说 failed；C7 reset 回到 Idle 并丢掉偏置。

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include <foc/core/startup.hpp>

namespace {

namespace core = foc::core;

constexpr float kDt = 0.001F;

/**
 * @brief Holds 10 ms, then accelerates the vector to 20 rad/s at 2000 rad/s^2: reachable in ten ticks.
 */
auto make_config() -> core::StartupConfig {
  return core::StartupConfig{1.0F, 0.01F, 2.0F, 2000.0F, 20.0F};
}

} // namespace

TEST(Startup, C1_ARecipeNobodyAskedForIsNotImpl) {
  const core::StartupRoutine routine{core::StartupConfig{0.0F, 0.0F, 0.0F, 0.0F, 0.0F}};
  EXPECT_EQ(routine.validate(), foc::ErrorCode::NotImpl);
}

TEST(Startup, C2_HalfARecipeIsRefusedAsAParameter) {
  core::StartupConfig config = make_config();
  config.pull_accel_rad_s2 = 0.0F;
  const core::StartupRoutine routine{config};
  EXPECT_EQ(routine.validate(), foc::ErrorCode::Param);
}

TEST(Startup, C3_BeginRefusesAndLeavesFailedBehind) {
  core::StartupRoutine routine{make_config()};
  EXPECT_FALSE(routine.begin(0U));
  EXPECT_EQ(routine.state(), core::StartupState::Failed);

  core::StartupRoutine unusable{core::StartupConfig{1.0F, 0.0F, 2.0F, 2000.0F, 20.0F}};
  EXPECT_FALSE(unusable.begin(7U));
  EXPECT_EQ(unusable.state(), core::StartupState::Failed);

  // and a usable recipe still starts, so the two refusals above are the only reason they were refused
  EXPECT_TRUE(routine.begin(7U));
  EXPECT_EQ(routine.state(), core::StartupState::Align);
}

TEST(Startup, C4_AnUnusableTickAsksForNothing) {
  core::StartupRoutine routine{make_config()};
  ASSERT_TRUE(routine.begin(7U));
  for (const float dt : {0.0F, -0.001F, std::numeric_limits<float>::quiet_NaN()}) {
    const core::AlignOutput out = routine.step(dt, 0.3F);
    EXPECT_EQ(out.id_a, 0.0F);
    EXPECT_EQ(out.theta_e_rad, 0.0F);
    EXPECT_FALSE(out.finished);
    EXPECT_FALSE(out.failed);
    EXPECT_EQ(routine.state(), core::StartupState::Align);
  }
}

TEST(Startup, C5_TheSequenceCapturesTheOffsetThenHandsOverAtTheDeclaredSpeed) {
  const core::StartupConfig config = make_config();
  core::StartupRoutine routine{config};
  ASSERT_TRUE(routine.begin(7U));

  // Align: the vector does not move, and the offset is the mechanical angle handed in at the end of the hold.
  int align_ticks = 0;
  while (routine.state() == core::StartupState::Align && align_ticks < 100) {
    const core::AlignOutput out = routine.step(kDt, 0.25F);
    EXPECT_FLOAT_EQ(out.id_a, config.align_current_a);
    EXPECT_FLOAT_EQ(out.theta_e_rad, 0.0F);
    ++align_ticks;
  }
  EXPECT_GE(align_ticks, 10);
  EXPECT_EQ(routine.state(), core::StartupState::Pull);
  EXPECT_FLOAT_EQ(routine.offset_mech_rad(), 0.25F);

  // Pull: declared current, declared acceleration; hand-over on the first tick at or past the speed.
  core::AlignOutput out{};
  int pull_ticks = 0;
  while (routine.state() == core::StartupState::Pull && pull_ticks < 10000) {
    out = routine.step(kDt, 0.25F);
    EXPECT_FLOAT_EQ(out.id_a, config.pull_current_a);
    ++pull_ticks;
  }
  EXPECT_EQ(routine.state(), core::StartupState::Done);
  EXPECT_TRUE(out.finished);
  // v = a*t with one tick of granularity, so the crossing is within one tick of the declared speed.
  EXPECT_GE(routine.forced_speed_rad_s(), config.handover_speed_rad_s);
  EXPECT_LE(routine.forced_speed_rad_s(),
            config.handover_speed_rad_s + kDt * config.pull_accel_rad_s2 + 1.0e-3F);
}

TEST(Startup, C6_DoneKeepsAnsweringAndFailedSaysSo) {
  core::StartupRoutine routine{make_config()};
  ASSERT_TRUE(routine.begin(7U));
  for (int i = 0; i < 10; ++i) {
    static_cast<void>(routine.step(kDt, 0.1F));
  }
  while (routine.state() == core::StartupState::Pull) {
    static_cast<void>(routine.step(kDt, 0.1F));
  }
  ASSERT_EQ(routine.state(), core::StartupState::Done);
  const core::AlignOutput after = routine.step(kDt, 0.1F);
  EXPECT_FLOAT_EQ(after.id_a, make_config().pull_current_a);
  EXPECT_FALSE(after.finished);
  EXPECT_FALSE(after.failed);

  core::StartupRoutine failed{make_config()};
  ASSERT_FALSE(failed.begin(0U));
  const core::AlignOutput out = failed.step(kDt, 0.0F);
  EXPECT_TRUE(out.failed);
  EXPECT_EQ(out.id_a, 0.0F);
}

TEST(Startup, C7_ResetForgetsTheRunItJustDid) {
  core::StartupRoutine routine{make_config()};
  ASSERT_TRUE(routine.begin(7U));
  for (int i = 0; i < 10; ++i) {
    static_cast<void>(routine.step(kDt, 0.4F));
  }
  ASSERT_FLOAT_EQ(routine.offset_mech_rad(), 0.4F);

  routine.reset();
  EXPECT_EQ(routine.state(), core::StartupState::Idle);
  EXPECT_FLOAT_EQ(routine.offset_mech_rad(), 0.0F);
  EXPECT_FLOAT_EQ(routine.forced_speed_rad_s(), 0.0F);
  EXPECT_FLOAT_EQ(routine.forced_angle_rad(), 0.0F);

  // Idle answers nothing at all, and it is not a failure either.
  const core::AlignOutput idle = routine.step(kDt, 0.0F);
  EXPECT_EQ(idle.id_a, 0.0F);
  EXPECT_FALSE(idle.finished);
  EXPECT_FALSE(idle.failed);
}
