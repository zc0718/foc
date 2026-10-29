// W7-a: turning an electrical estimate into a position a position loop can use.
//
// The observer gives an angle that wraps every 2*pi/pole_pairs and starts wherever it happens to start. Unwrapping makes
// it continuous; an index mark fixes where zero is. These cases test the two halves separately and then together,
// because each one alone is useless in a different way.
//
// W7-a：把一个电角度估计变成位置环可用的位置。
// 观测器给出的角度每 `2*pi/极对数` 折一次，而且从哪里开始就从哪里开始。解开折返使它连续；索引标记把零点钉住。
// 下面分别测这两半、再测它们合起来——因为任何一半单独都是**以不同方式**无用的。
//
// The criterion, fixed before the numbers below were measured:
//   C1  over five mechanical turns the unwrapped angle tracks the truth within 1e-3 rad at every sample, with no report
//       of a lost count. A reconstruction that drifts is not a reconstruction.
//   C2  before any index there is no origin: `homed()` is false and `referenced_mech_rad()` reads 0, which is the
//       documented stand-in -- and the reason a caller has to ask `homed()` rather than trust the number.
//   C3  at the index the referenced angle reads exactly `index_mech_rad`; one turn later it reads that plus 2*pi, i.e.
//       the mark starts the count rather than pinning the angle to a single-turn value forever.
//   C4  a later pass of the mark removes drift and does **not** rewind the count. Drift is injected as a deliberate bias
//       in the electrical angle; requiring the referenced value to stay put is what separates "snap" from "redefine".
//   C5  a step too large to unwrap is reported (`lost_count()`), and the next index clears it -- the mark is exactly the
//       information that was lost.
//
// 判据在量之前定下：
//   C1 五个机械圈内，解开折返的角度在每次采样都与真值相差不超过 1e-3 rad，且不报告丢计数。会漂的重建不是重建。
//   C2 索引之前没有原点：`homed()` 为假、`referenced_mech_rad()` 读 0，这是写明的占位值——也是调用方必须查
//      `homed()` 而不是直接信这个数的原因。
//   C3 在索引处基准角**恰好**读 `index_mech_rad`；一圈之后读它加 2π，也就是说标记是**开始计数**，而不是把角度永久
//      钉成一个单圈值。
//   C4 标记重复经过时消除漂移、且**不**让计数倒退。漂移以电角度上刻意加偏置的方式注入；要求基准值原地不动，正是把
//      "吸附"与"重新定义"区分开的那一条。
//   C5 大到无法解开折返的跳变会被报告（`lost_count()`），并由下一次索引清除——标记正是丢失掉的那个信息。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/position_reference.hpp>

namespace {

namespace core = foc::core;

constexpr float kPi = 3.14159265358979F;
constexpr float kTwoPi = 6.28318530717959F;
constexpr std::uint8_t kPoles = 7U;
constexpr float kMechStepRad = 0.02F; // per sample; 0.14 rad electrical, well inside the half-turn limit
constexpr float kDriftElectricalRad = 1.0F; // 0.143 rad mechanical, i.e. seven ticks of travel: distinguishable

auto make_reference() -> core::PositionReference {
    return core::PositionReference(core::PositionReference::Config{kPoles, 0.0F});
}

/** @brief [en] Electrical angle for a true mechanical angle, wrapping as a real encoder/observer would. */
/** @brief [zh] 由真实机械角给出电角度，按真实传感器/观测器的方式折返。 */
auto electrical_of(float mech_rad) -> float {
    return std::fmod(mech_rad * static_cast<float>(kPoles), kTwoPi);
}

} // namespace

TEST(PositionReference, TheUnwrappedAngleTracksTheTruthAcrossFiveTurns) {
    // C1
    core::PositionReference reference = make_reference();
    constexpr int kSamples = static_cast<int>((5.0F * kTwoPi) / kMechStepRad);
    float worst = 0.0F;
    for (int i = 0; i < kSamples; ++i) {
        const float truth = kMechStepRad * static_cast<float>(i);
        static_cast<void>(reference.update(electrical_of(truth), false));
        worst = std::max(worst, std::fabs(reference.unwrapped_mech_rad() - truth));
    }
    std::printf("\n[reference] five turns: worst |unwrapped - truth| = %.6f rad\n", static_cast<double>(worst));
    EXPECT_LT(worst, 1e-3F) << "the reconstruction drifts, so it is not usable for position";
    EXPECT_FALSE(reference.lost_count()) << "a smooth sweep reported a lost count";
}

TEST(PositionReference, WithoutAnIndexThereIsNoOrigin) {
    // C2
    core::PositionReference reference = make_reference();
    for (int i = 0; i < 200; ++i) {
        static_cast<void>(reference.update(electrical_of(kMechStepRad * static_cast<float>(i)), false));
    }
    EXPECT_FALSE(reference.homed());
    EXPECT_FLOAT_EQ(0.0F, reference.referenced_mech_rad()) << "an un-homed reference must not invent an origin";
    EXPECT_NE(0.0F, reference.unwrapped_mech_rad()) << "the relative angle is available even without an origin";
}

TEST(PositionReference, TheIndexStartsTheCountInsteadOfPinningIt) {
    // C3
    core::PositionReference reference = make_reference();
    const float index_truth = kTwoPi; // the mark sits at one mechanical turn
    bool seen = false;
    float at_index = 0.0F;

    // Walk to the mark and one further turn, pulsing the index on the sample that crosses it.
    // 走到标记处并再走一圈，在跨过标记的那一次采样上给出索引脉冲。
    float previous = 0.0F;
    for (int i = 0; i <= static_cast<int>((2.0F * index_truth) / kMechStepRad); ++i) {
        const float truth = kMechStepRad * static_cast<float>(i);
        const bool crossed = (std::floor(truth / kTwoPi) != std::floor(previous / kTwoPi));
        previous = truth;
        if (reference.update(electrical_of(truth), crossed)) {
            if (!seen) {
                at_index = reference.referenced_mech_rad();
                seen = true;
                EXPECT_FLOAT_EQ(0.0F, at_index) << "the mark must define exactly index_mech_rad";
            }
        }
    }
    ASSERT_TRUE(seen) << "the index was never seen";
    ASSERT_TRUE(reference.homed());
    std::printf("[reference] after a second pass of the mark the count reads %.4f rad\n",
                static_cast<double>(reference.referenced_mech_rad()));
    // Two marks passed -> the count advanced by exactly one mechanical turn past the origin.
    // 经过两次标记 → 计数从原点起正好前进了一个机械圈。
    EXPECT_NEAR(kTwoPi, reference.referenced_mech_rad(), 0.05F)
        << "the mark pinned the angle instead of starting a count";
}

TEST(PositionReference, ARepeatPassOfTheMarkRemovesDriftWithoutRewinding) {
    // C4: the electrical angle carries a bias after homing, i.e. the reconstruction is drifting. The bias is chosen to
    // be **larger than one tick of travel** (1.0 rad electrical = 0.143 rad mechanical against a 0.02 rad step), because
    // a smaller drift cannot be separated from the ordinary advance between the sample before the mark and the mark
    // itself -- the first version of this case injected 0.01 rad and measured exactly that ambiguity.
    // C4：索引之后电角度带上偏置，即重建正在漂移。偏置刻意取得**大于一个 tick 的行程**
    // （电 1.0 rad = 机械 0.143 rad，而每 tick 走 0.02 rad），因为更小的漂移无法与"标记前那一 tick 的正常前进"
    // 区分开——这个用例的第一版注入 0.01 rad，量到的正是那个歧义。
    core::PositionReference reference = make_reference();
    float previous = 0.0F;
    float last_referenced = 0.0F;
    float before_second_mark = 0.0F;
    float after_second_mark = 0.0F;
    int marks = 0;

    for (int i = 0; i <= static_cast<int>((2.05F * kTwoPi) / kMechStepRad); ++i) {
        const float truth = kMechStepRad * static_cast<float>(i);
        const bool crossed = (std::floor(truth / kTwoPi) != std::floor(previous / kTwoPi));
        previous = truth;
        // Drift only after the origin exists, so C3's exactness is not disturbed by this case.
        // 只在原点建立之后注入漂移，以免本用例干扰 C3 的精确性。
        const float bias = (marks > 0) ? kDriftElectricalRad : 0.0F;
        if (crossed && (marks == 1)) {
            before_second_mark = last_referenced; // the value on the tick before the snap
        }
        if (reference.update(electrical_of(truth) + bias, crossed)) {
            ++marks;
            if (marks == 2) {
                after_second_mark = reference.referenced_mech_rad();
            }
        }
        last_referenced = reference.referenced_mech_rad();
    }
    ASSERT_EQ(2, marks) << "expected the mark twice";
    std::printf("[reference] with drift, just before the second mark %.4f rad -> at it %.4f rad\n",
                static_cast<double>(before_second_mark), static_cast<double>(after_second_mark));
    EXPECT_NEAR(kTwoPi, after_second_mark, 0.02F) << "the repeat pass did not land on the count";
    EXPECT_NEAR(kDriftElectricalRad / static_cast<float>(kPoles), before_second_mark - after_second_mark, 0.03F)
        << "the pass must remove the injected drift, and by exactly that much";
}

TEST(PositionReference, AJumpTooLargeToUnwrapIsReportedAndTheMarkClearsIt) {
    // C5
    core::PositionReference reference = make_reference();
    static_cast<void>(reference.update(0.0F, false));
    // Half an electrical turn between samples: indistinguishable from a wrap, so it must be reported.
    // 两次采样之间半个电角度：与一次折返无法区分，因此必须被报告。
    static_cast<void>(reference.update(kPi * 0.95F, false));
    EXPECT_TRUE(reference.lost_count()) << "an unwrappable step was accepted silently";

    static_cast<void>(reference.update(kPi * 0.95F, true));
    EXPECT_FALSE(reference.lost_count()) << "the mark is exactly the information that was lost and must restore it";
    EXPECT_TRUE(reference.homed());
}

// C6  a zero pole count is treated as one pole instead of dividing by zero. Unclamped, the very first sample
//     would read `inf` or `nan`; the mechanical angle would then be useless for every later sample too.
// C6  极对数取 0 时按一拍处理，而不是除以零。不夹取的话，第一次采样就会读出 `inf`/`nan`，此后每一拍的机械角
//     都会一起失去意义。
TEST(PositionReference, C6_AZeroPoleCountIsOnePoleNotADivision) {
  core::PositionReference reference{core::PositionReference::Config{0U, 0.0F}};
  static_cast<void>(reference.update(1.0F, false));
  ASSERT_TRUE(std::isfinite(reference.unwrapped_mech_rad())) << "a zero pole count must not divide by zero";
  EXPECT_FLOAT_EQ(reference.unwrapped_mech_rad(), 1.0F);
  static_cast<void>(reference.update(1.5F, false));
  EXPECT_NEAR(reference.unwrapped_mech_rad(), 1.5F, 1.0e-5F);
}

// C7  `reset()` forgets the origin *and* the unwrap. After it, the first sample re-primes the estimator, so an
//     angle far from the pre-reset one is a new starting point rather than a jump of several turns.
// C7  `reset()` 同时忘掉原点与折返累积。重置后第一次采样重新预置，因此与重置前相差很大的角度是**新的起点**，
//     而不是几圈的跳变。
TEST(PositionReference, C7_ResetForgetsTheOriginAndTheUnwrap) {
  core::PositionReference reference = make_reference();
  for (int i = 0; i < 40; ++i) {
    static_cast<void>(reference.update(electrical_of(static_cast<float>(i) * kMechStepRad), i == 10));
  }
  ASSERT_TRUE(reference.homed());
  ASSERT_NE(reference.referenced_mech_rad(), 0.0F);

  reference.reset();
  EXPECT_FALSE(reference.homed());
  EXPECT_FLOAT_EQ(reference.referenced_mech_rad(), 0.0F);
  EXPECT_FLOAT_EQ(reference.unwrapped_mech_rad(), 0.0F);
  EXPECT_FALSE(reference.lost_count());

  // Far from where it stopped. Re-priming means the mechanical angle is this sample's **principal value**: an
  // electrical angle only pins the mechanical one modulo 2*pi/poles, so 3.0 rad reads 3.0 mod 0.8976 = 0.3072.
  // The point of the case is that it is that value and not a continuation of the old count -- and that a 3 rad
  // jump is not reported as a lost count, because after a reset there is nothing to have lost.
  constexpr float kFarMechRad = 3.0F;
  constexpr float kPolesPerTurnRad = kTwoPi / static_cast<float>(kPoles);
  static_cast<void>(reference.update(electrical_of(kFarMechRad), false));
  const float principal = std::fmod(kFarMechRad, kPolesPerTurnRad);
  EXPECT_NEAR(reference.unwrapped_mech_rad(), principal, 1e-4F) << "reset must not leave a stale previous angle";
  EXPECT_FALSE(reference.lost_count());
}
