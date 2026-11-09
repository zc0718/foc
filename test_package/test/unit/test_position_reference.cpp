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

/** @brief [en] Wrap an angle into (-pi, pi], the way the reconstruction's snap does. */
/** @brief [zh] 把角度折到 (-pi, pi]，与重建的吸附一致。 */
auto wrap_to_pi(float angle) -> float {
    while (angle <= -kPi) {
        angle += kTwoPi;
    }
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    return angle;
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


namespace {

/**
 * @brief [en] A sample step well below every jitter and delay under test, so the pulse lands where the mark test puts
 *             it rather than on an arbitrary grid point.
 * @brief [zh] 远小于所有待测抖动与延迟的采样步长，使脉冲落在用例指定的位置而不是某个栅格点上。
 */
constexpr float kFineStep = 2.0e-4F;

struct Ramp final {
    double error_rad;       ///< [en] referenced angle minus truth at the end. [zh] 结束时基准角减真值。
    int marks;              ///< [en] marks that were accepted, the defining one included. [zh] 被接受的标记数，含定义原点的那个。
    double turn_length;     ///< [en] referenced angle one true turn past the origin. [zh] 原点之后一个真实圈处的基准角。
    double last_snap;       ///< [en] how far the last accepted mark moved the origin. [zh] 最后一个被接受的标记把原点移动了多少。
    /**
     * @brief [en] the wrapped residual the last accepted mark was asked to remove.
     * @brief [zh] 最后一个被接受的标记被要求消除的、折返后的残差。
     */
    float last_residual;
};

/**
 * @brief [en] A mechanical ramp through the electrical angle, one mark per revolution.
 *
 * @brief [zh] 通过电角度驱动的机械斜坡，每转一圈一个标记。
 *
 *   The origin is defined by a mark at mechanical zero, so every later number is measured from a known place; every
 *   subsequent pulse arrives `mark_delay_rad` after the geometric crossing, which is how a mounting tolerance or a
 *   threshold jitter shows up on a real machine. 原点由机械零位处的一个标记定义，于是后面每个数都从一个已知位置量起；此
 *   后每个脉冲都在几何穿越之后 `mark_delay_rad` 到达，这正是真实机器上安装公差或阈值抖动出现的方式。
 */
auto run_ramp(core::PositionReference& reference, double turns, float mech_step, float mark_delay_rad,
              int skip_turn) -> Ramp {
    reference.reset();
    Ramp out{0.0, 0, 0.0, 0.0, 0.0F};
    static_cast<void>(reference.update(electrical_of(0.0F), true));
    ++out.marks;
    const int steps = static_cast<int>((turns * kTwoPi) / mech_step);
    double next_mark = kTwoPi + static_cast<double>(mark_delay_rad);
    int pass = 0;
    float truth = 0.0F;
    for (int i = 1; i <= steps; ++i) {
        truth = mech_step * static_cast<float>(i);
        if (out.turn_length == 0.0 && static_cast<double>(truth) >= kTwoPi - 0.5 * mech_step) {
            out.turn_length = static_cast<double>(reference.referenced_mech_rad());
        }
        if (static_cast<double>(truth) >= next_mark) {
            ++pass;
            next_mark += kTwoPi;
            if (pass == skip_turn) {
                static_cast<void>(reference.update(electrical_of(truth), false));
                continue;
            }
            const float before = reference.referenced_mech_rad();
            out.last_residual = wrap_to_pi(before - 0.0F);
            if (reference.update(electrical_of(truth), true)) {
                ++out.marks;
                out.last_snap = static_cast<double>(reference.referenced_mech_rad() - before);
            }
            continue;
        }
        static_cast<void>(reference.update(electrical_of(truth), false));
    }
    out.error_rad = static_cast<double>(reference.referenced_mech_rad()) - truth;
    return out;
}

} // namespace



/**
 * @brief [en] Four error sources on the counting channel, each with a closed form written down before it was measured.
 *
 * @brief [zh] 计数通道上的四个误差源，每个都有一个先写后量的闭式。
 *
 *   The register asked for a sensitivity table, and a counting channel gives one the continuous-angle channels cannot:
 *   every coefficient is exact, because the channel is a sum of integers and one division. So this case checks four
 *   *identities* rather than four magnitudes.
 *
 *   1. **Wrap rate.** The electrical angle wraps every `2*pi/p` mechanically, so `p*s/(2*pi)` electrical turns pass in
 *   `s` mechanical radians. This is the input signal, not the reconstruction, and stating it is what makes the other
 *   three readable.
 *
 *   2. **Pole-pair error.** Each electrical step is divided by the *configured* pole count, so the reconstruction's
 *   **turn length is `2*pi*p/p'`**, not `2*pi`. That is a scale error, and the index is the only thing that can see it,
 *   because a mark measures exactly one turn. Its residue at the next mark is `2*pi*|1 - p/p'|`, which for any integer
 *   error is at least `2.22*|p'-p|` times the resolution the channel declares -- so with the guard below the mark is
 *   **refused and reported** instead of silently sawing the count back once per turn. A wrong pole count is therefore
 *   detectable from the second mark onward: a capability this row did not have before.
 *
 *   3. **Mark jitter.** The snap moves the origin onto the *pulse*, so a mark that fires `j` late moves the origin by
 *   exactly `j`, on every pass. Measured as the snap itself, not as an end-of-run error, because that error also
 *   carries the unwrap's own drift.
 *
 *   4. **A missed mark** costs **no counts**. The count is carried by the unwrap; the mark only removes drift. A missed
 *   mark therefore costs one turn of drift, of order 1e-3 rad -- and that is exactly what separates it from a lost
 *   count, which costs a whole turn.
 *
 *   登记册要的是一张敏感度表，而计数通道给出的表是连续角度通道给不出的：每个系数都精确，因为这个通道就是整数求和加一次
 *   除法。所以本用例核对的是四条**恒等式**而不是四个量级。
 *
 *   1. **折返率**。电角度每 `2*pi/p` 机械弧度折一次，所以 `s` 机械弧度里经过 `p*s/(2*pi)` 个电周期。这是输入信号而不是
 *   重建；把它写清楚，后面三条才读得懂。
 *
 *   2. **极对数误差**。每个电步长除以**配置的**极对数，所以重建出来的**一圈长度是 `2*pi*p/p'`** 而不是 `2*pi`。这是比例
 *   误差，而索引是唯一能看见它的东西，因为标记量的正是一圈。它在下一次标记处的残差是 `2*pi*|1 − p/p'|`，对任何整数误差
 *   都不小于通道声明分辨力的 `2.22*|p'−p|` 倍——所以有了下面那条护栏，标记会被**拒绝并报告**，而不是每圈把计数静默锯回去
 *   一次。于是极对数配错从第二个标记起就是**可检出**的：这是本行此前没有的能力。
 *
 *   3. **标记抖动**。吸附把原点移到**脉冲**上，所以晚 `j` 的标记每次经过都把原点移动恰好 `j`。量的是吸附本身，而不是终
 *   点误差——后者还带着解折返自己的漂移。
 *
 *   4. **漏掉一个标记**不损失任何计数。计数由解折返携带，标记只消除漂移。所以漏一个标记的代价是一圈的漂移，量级 1e-3
 *   rad——这正是它与"丢计数"（损失整整一圈）的区别。
 */
TEST(PositionReference, TheCountingChannelHasFourErrorSourcesAndEachHasAClosedForm) {
    // (1) The input signal's wrap rate.
    std::printf("\n%-10s %-8s %16s %12s\n", "step", "turns", "electrical turns", "p*s/(2pi)");
    for (const auto& item : {std::pair<float, double>{0.02F, 1.5}, {0.005F, 3.25}, {0.05F, 2.0}}) {
        const double distance = item.second * kTwoPi;
        const double measured = static_cast<double>(kPoles) * distance / kTwoPi;
        std::printf("%-10.4f %-8.2f %16.3f %12.3f\n", static_cast<double>(item.first), item.second,
                    measured, item.second * static_cast<double>(kPoles));
        EXPECT_NEAR(item.second * kPoles, measured, 1e-9);
    }

    // (2) A wrong pole count changes the reconstructed turn length, and the mark reports it.
    std::printf("\n%-10s %16s %16s %10s %10s\n", "configured", "turn length", "2*pi*p/p'", "residue",
                "reported");
    for (const std::uint8_t wrong : {6U, 8U}) {
        core::PositionReference reference(core::PositionReference::Config{wrong, 0.0F});
        const Ramp run = run_ramp(reference, 3.0, kMechStepRad, 0.0F, 0);
        const double predicted_turn = kTwoPi * static_cast<double>(kPoles) / static_cast<double>(wrong);
        const double residue = kTwoPi * std::fabs(1.0 - static_cast<double>(kPoles) / wrong);
        const double resolution = 0.9 * kPi / static_cast<double>(wrong);
        std::printf("%-10u %16.4f %16.4f %10.4f %10d\n", static_cast<unsigned>(wrong),
                    run.turn_length, predicted_turn, residue, static_cast<int>(reference.lost_count()));
        EXPECT_NEAR(predicted_turn, run.turn_length, 0.02 * predicted_turn)
            << "the reconstruction's turn is 2*pi*p/p', which is what a mark can measure";
        EXPECT_TRUE(reference.lost_count()) << "a wrong pole count must be reported, not absorbed";
        EXPECT_GT(residue / resolution, 2.0) << "an integer pole error is always visible to the mark";
    }

    // (3) Mark jitter moves the origin by exactly the jitter.
    // A sample step well below the jitter, or the measurement is of the sample grid rather than of the
    // mark: at a 0.02 rad step the pulse can arrive anywhere in a 0.02 rad window, which is larger than
    // the smallest jitter under test.
    // 采样步长要远小于抖动，否则量的是采样栅格而不是标记：在 0.02 rad 步长下脉冲可以落在 0.02 rad
    // 的窗口里，比最小的待测抖动还大。
    // The jitter is absorbed into the origin at the *first* repeat pass and never appears again -- later
    // passes only remove drift -- so "the last snap" measures drift, not the jitter. Differencing two
    // runs that share every other detail isolates it: both runs do the same arithmetic on the same
    // number of samples, so the drift cancels and what is left is the mark's own displacement.
    // 抖动在**第一次**重复经过时就被吸收进原点，此后不再出现——之后的经过只消除漂移——所以"最后一次
    // 吸附"量到的是漂移而不是抖动。把两次除这一项外完全相同的运行相减即可隔离它：两次运行对同样多的
    // 采样做同样的算术，于是漂移抵消，剩下的就是标记自身的位移。
    std::printf("\n%-10s %16s %16s\n", "jitter", "run - clean", "ratio");
    for (const float jitter : {0.001F, 0.01F, 0.05F}) {
        core::PositionReference jittered = make_reference();
        core::PositionReference clean = make_reference();
        const Ramp with_jitter = run_ramp(jittered, 4.0, kFineStep, jitter, 0);
        const Ramp without = run_ramp(clean, 4.0, kFineStep, 0.0F, 0);
        const double difference = with_jitter.error_rad - without.error_rad;
        std::printf("%-10.4f %16.6f %16.6f\n", static_cast<double>(jitter), difference,
                    difference / static_cast<double>(jitter));
        EXPECT_NEAR(static_cast<double>(jitter), std::fabs(difference), 2.0e-4)
            << "a mark that fires j late displaces the origin by exactly j";
        EXPECT_GE(with_jitter.marks, 4) << "the marks have to be believed at this size, or the case is empty";
        EXPECT_EQ(with_jitter.marks, without.marks);
    }

    // (4) A missed mark costs drift, not counts.
    core::PositionReference missed = make_reference();
    const Ramp missed_run = run_ramp(missed, 5.0, kMechStepRad, 0.0F, 3);
    core::PositionReference clean = make_reference();
    const Ramp clean_run = run_ramp(clean, 5.0, kMechStepRad, 0.0F, 0);
    std::printf("\n[missed mark] error %.2e vs %.2e rad, marks %d vs %d\n", missed_run.error_rad,
                clean_run.error_rad, missed_run.marks, clean_run.marks);
    EXPECT_EQ(clean_run.marks - 1, missed_run.marks) << "exactly one mark must have been skipped";
    EXPECT_LT(std::fabs(missed_run.error_rad), 0.02) << "a missed mark costs a turn of drift, not a turn";
    EXPECT_TRUE(missed.homed());
}



/**
 * @brief [en] A mark that cannot be the mark is reported instead of moving the origin by up to half a turn.
 *
 * @brief [zh] 不可能是标记的那个标记会被报告，而不是把原点移走最多半圈。
 *
 *   The electrical path already refuses to guess: a step at or beyond `0.9*pi` electrical is reported rather than
 *   resolved, because picking a direction gives an angle that looks fine and is a whole turn out. The index path had no
 *   such rule, and it was the more dangerous of the two: a spurious pulse produced a snap of `wrap_pi(phase)` -- up to
 *   3.14 rad of **mechanical** angle, seven times the electrical step it was not guarding against -- and nothing
 *   reported it. The next real mark then moved it back, so the defect presented as a transient nobody could reproduce.
 *
 *   The fix is the same rule in the same units: past `0.9*pi/p` of mechanical residual the two sources are not
 *   describing the same event, so the pulse is refused and flagged. Below that the pulse is *believed*, and that
 *   boundary is stated rather than hidden: at `p = 7` the threshold is 0.404 rad, which is the resolution floor the
 *   channel already declares for the electrical step and 400 times the drift measured over five turns.
 *
 *   电角度通路本来就不猜：达到或超过电 `0.9*pi` 的步长被**报告**而不是被解掉，因为选一个方向会给出一个看起来没问题、却
 *   差了整整一圈的角度。索引通路没有这条规则，而它才是更危险的那条：一个虚假脉冲产生的吸附是 `wrap_pi(相位)`——最多 3.14
 *   **机械**弧度，是它本该防住的电步长的七倍——而且没有任何东西报告它。下一次真实标记又把它移回来，于是这个缺陷表现为一
 *   个没人能复现的瞬变。
 *
 *   修法就是同一条规则、同一个单位：机械残差超过 `0.9*pi/p` 时两个源描述的不是同一个事件，于是脉冲被拒绝并置旗。低于它
 *   时脉冲是**被相信的**，而这条边界写在这里而不是藏着：`p = 7` 时阈值是 0.404 rad，正是该通道为电步长已经声明过的分辨
 *   力下限，也是五圈实测漂移的 400 倍。
 */
TEST(PositionReference, AMarkThatCannotBeTheMarkIsReportedInsteadOfMovingTheOrigin) {
    const float resolution = 0.9F * kPi / static_cast<float>(kPoles);
    std::printf("\n%-12s %14s %14s %10s %10s\n", "phase (turn)", "would-be snap", "actual move",
                "accepted", "reported");
    for (const float phase : {0.25F, 0.5F, 0.75F}) {
        core::PositionReference reference = make_reference();
        reference.reset();
        float truth = 0.0F;
        static_cast<void>(reference.update(electrical_of(truth), true)); // the defining mark
        float before = 0.0F;
        float after = 0.0F;
        float would_be = 0.0F;
        bool accepted = true;
        bool reported = false;
        float previous = 0.0F;
        const int steps = static_cast<int>((3.0 * kTwoPi) / kMechStepRad);
        for (int i = 1; i <= steps; ++i) {
            truth = kMechStepRad * static_cast<float>(i);
            const bool crossed = (std::floor(truth / kTwoPi) != std::floor(previous / kTwoPi));
            previous = truth;
            const bool spurious = std::fabs(truth - phase * kTwoPi) < 0.5F * kMechStepRad;
            if (spurious) {
                before = reference.referenced_mech_rad();
                would_be = -wrap_to_pi(before - 0.0F);
                accepted = reference.update(electrical_of(truth), true);
                after = reference.referenced_mech_rad();
                reported = reference.lost_count();
                continue;
            }
            static_cast<void>(reference.update(electrical_of(truth), crossed));
        }
        std::printf("%-12.2f %14.4f %14.4f %10d %10d\n", static_cast<double>(phase),
                    static_cast<double>(would_be), static_cast<double>(after - before),
                    static_cast<int>(accepted), static_cast<int>(reported));

        EXPECT_FALSE(accepted) << "a mark that cannot be the mark must be refused";
        EXPECT_TRUE(reported) << "and it must be reported, or the caller never learns";
        EXPECT_GT(std::fabs(would_be), 20.0F * std::fabs(after - before))
            << "the refused snap has to be far larger than the ordinary advance";
        EXPECT_GT(std::fabs(would_be), 0.2F) << "the probe phases have to be well away from the mark";
    }

    // The boundary, stated: inside the channel's resolution the pulse is believed, so its displacement
    // reaches the angle and nothing is reported. The same differencing as above measures it, which is
    // what brackets the threshold: 0.1 rad is believed, 1.57 rad is refused, and the rule between them
    // is 0.9*pi/p.
    // 边界写在这里：在通道分辨力以内脉冲是被相信的，于是它的位移进入角度、没有任何报告。用上面同样的
    // 差分量它，这就把阈值夹住了：0.1 rad 被相信，1.57 rad 被拒绝，而它们之间的规则是 `0.9*pi/p`。
    core::PositionReference believed = make_reference();
    core::PositionReference clean_twin = make_reference();
    const Ramp late = run_ramp(believed, 4.0, kFineStep, 0.1F, 0);
    const Ramp clean_run = run_ramp(clean_twin, 4.0, kFineStep, 0.0F, 0);
    const double displacement = late.error_rad - clean_run.error_rad;
    std::printf("[boundary] a mark 0.100 rad late (threshold %.3f) moves the angle by %.5f, "
                "reported %d; the 1.57 rad cases above were refused\n",
                static_cast<double>(resolution), displacement,
                static_cast<int>(believed.lost_count()));
    EXPECT_NEAR(0.1, std::fabs(displacement), 2.0e-4)
        << "below the resolution the mark is believed and its jitter reaches the angle";
    EXPECT_FALSE(believed.lost_count()) << "below the resolution there is nothing to report";
    EXPECT_EQ(late.marks, clean_run.marks) << "nothing was refused, or this is not the boundary case";
}

