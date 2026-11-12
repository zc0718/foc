// Conan::ImportStart
#include <cmath>
#include <cstddef>

#include <gtest/gtest.h>

#include <foc/core/angle_source.hpp>
// Conan::ImportEnd

namespace {

// What "the angle is wrong" is allowed to mean, and what the drive says about it.
//
// The type being tested decides between offers. Every case here therefore has two halves: what it answered, and whether
// that answer is defensible against a number the test computed itself -- the disagreement it was fed, or the angle the
// sources were telling apart on purpose. A case that only checked "the status changed" would pass for a type that
// returns `Degraded` on a timer.
//
// 测的是"角度错了"这件事允许怎么表达，以及驱动器对此说了什么。
//
// 被测类型在各份报价之间做选择。因此这里每个用例都有两半：它答了什么，以及这个答案能否对着用例自己算出来的一个数
// 站住脚——它被喂进去的分歧，或各来源故意说不到一起的那个角度。只检查"状态变了"的用例，对一个按定时器返回
// `Degraded` 的类型也会通过。

using foc::ErrorCode;
using foc::core::AngleOffer;
using foc::core::AngleQuality;
using foc::core::AngleSource;
using foc::core::AngleSourceConfig;
using foc::core::AngleSourceKind;

constexpr float kDt = 50e-6F;
constexpr float kTwoPi = 6.28318530717958647692F;

auto make_config() -> AngleSourceConfig {
    AngleSourceConfig config{};
    config.handover_window_s = 2e-3F;
    config.disagree_degraded_rad = 0.05F;
    config.disagree_blind_rad = 0.20F;
    return config;
}

// A band whose bars and probe values are all exact in float32: 1/16 and 1/4 are powers of two, and `1.0F - 1/16` is
// exact by Sterbenz' lemma, so "exactly on the bar" tests the comparison instead of testing rounding.
// 一个两条边界与探点值在 float32 里都精确的区间：1/16 与 1/4 是 2 的幂，而 `1.0F - 1/16` 按 Sterbenz 引理是精确的，
// 于是"恰好在边界上"测的是比较本身，而不是测舍入。
auto power_of_two_config() -> AngleSourceConfig {
    AngleSourceConfig config{};
    config.handover_window_s = 2e-3F;
    config.disagree_degraded_rad = 0.0625F;
    config.disagree_blind_rad = 0.25F;
    return config;
}

auto make_offer(AngleSourceKind kind, float angle_rad, float speed_rad_s = 1.0F) -> AngleOffer {
    return AngleOffer{kind, true, angle_rad, speed_rad_s};
}

auto make_missing(AngleSourceKind kind) -> AngleOffer { return AngleOffer{kind, false, 0.0F, 0.0F}; }

// 1. Three numbers that cannot describe a band are refused before a single step runs.
TEST(AngleSource, AConfigurationThatCannotBeABandIsRefused) {
    for (const float window : {0.0F, -1e-3F}) {
        AngleSource source;
        AngleSourceConfig config = make_config();
        config.handover_window_s = window;
        EXPECT_EQ(ErrorCode::Param, source.configure(config)) << window;
    }
    {
        AngleSource source;
        AngleSourceConfig config = make_config();
        config.disagree_degraded_rad = 0.0F;
        EXPECT_EQ(ErrorCode::Param, source.configure(config));
    }
    {
        // Blind must be *above* degraded, otherwise there is an interval of disagreement with no name.
        // 盲必须 **高于** 降级，否则存在一段没有名字的分歧区间。
        AngleSource source;
        AngleSourceConfig config = make_config();
        config.disagree_blind_rad = config.disagree_degraded_rad;
        EXPECT_EQ(ErrorCode::Param, source.configure(config));
    }
    {
        AngleSource source;
        const AngleOffer offers[1] = {make_offer(AngleSourceKind::Sensor, 0.3F)};
        EXPECT_EQ(AngleQuality::Blind, source.step(offers, 1U, kDt).quality) << "unconfigured is not 'use whatever'";
    }
}

// 2. Nothing answered: hold the angle, report zero speed, and say so.
TEST(AngleSource, NothingAnsweringIsBlindWithAHeldAngleAndNoSpeed) {
    AngleSource source;
    ASSERT_EQ(ErrorCode::Ok, source.configure(make_config()));
    const AngleOffer offers[2] = {make_missing(AngleSourceKind::Sensor), make_missing(AngleSourceKind::Observer)};
    const auto& reading = source.step(offers, 2U, kDt);
    EXPECT_EQ(AngleQuality::Blind, reading.quality);
    EXPECT_EQ(AngleSourceKind::None, reading.source);
    EXPECT_FLOAT_EQ(0.0F, reading.angle_mech_rad) << "a held angle starts at zero and stays there";
    EXPECT_FLOAT_EQ(0.0F, reading.speed_mech_rad_s) << "a held angle has no speed to report";

    const auto& again = source.step(offers, 2U, kDt);
    EXPECT_EQ(AngleQuality::Blind, again.quality);
    EXPECT_FLOAT_EQ(0.0F, again.speed_mech_rad_s) << "and it does not drift while nobody is speaking";
}

// 3. One live source is `Degraded`, never `Ok`: with nothing to compare against, agreement has stopped being checkable.
TEST(AngleSource, ASingleLiveSourceIsDegradedBecauseAgreementCannotBeChecked) {
    AngleSource source;
    ASSERT_EQ(ErrorCode::Ok, source.configure(make_config()));
    const AngleOffer offers[2] = {make_offer(AngleSourceKind::Sensor, 0.4F, 2.0F),
                                  make_missing(AngleSourceKind::Observer)};
    const auto& reading = source.step(offers, 2U, kDt);
    EXPECT_EQ(AngleQuality::Degraded, reading.quality);
    EXPECT_EQ(AngleSourceKind::Sensor, reading.source);
    EXPECT_TRUE(std::isnan(reading.disagreement_rad)) << "and the reading says there was nothing to compare with";
    EXPECT_FLOAT_EQ(0.4F, reading.angle_mech_rad) << "the answer itself is still the live source's";
    EXPECT_FLOAT_EQ(2.0F, reading.speed_mech_rad_s);
}

// 4 & 5. The status follows the disagreement the test itself put between the two sources -- three shapes, one rule.
TEST(AngleSource, TheStatusFollowsTheMeasuredDisagreement) {
    struct Case final {
        float disagreement_rad;
        AngleQuality want;
    };
    const Case cases[] = {
        {0.010F, AngleQuality::Ok},        // below the degraded bar
        {0.0625F, AngleQuality::Ok},       // exactly on it: "at or below" is still agreement
        {0.070F, AngleQuality::Degraded},  // just over
        {0.250F, AngleQuality::Degraded},  // exactly on the blind bar
        {0.300F, AngleQuality::Blind},     // over it
        {0.900F, AngleQuality::Blind},     // a source has gone somewhere else entirely
    };
    for (const Case& one : cases) {
        AngleSource source;
        ASSERT_EQ(ErrorCode::Ok, source.configure(power_of_two_config()));
        const AngleOffer offers[2] = {make_offer(AngleSourceKind::Sensor, 1.0F, 1.0F),
                                      make_offer(AngleSourceKind::Observer, 1.0F - one.disagreement_rad, 1.0F)};
        const auto& reading = source.step(offers, 2U, kDt);
        EXPECT_EQ(one.want, reading.quality) << one.disagreement_rad;
        // The reported disagreement is the number the decision was made on, not a second computation of it.
        // 报出来的分歧就是做决定用的那个数，而不是它的第二次计算。
        EXPECT_NEAR(one.disagreement_rad, reading.disagreement_rad, 1e-6F) << one.disagreement_rad;
        if (one.want == AngleQuality::Blind) {
            EXPECT_FLOAT_EQ(0.0F, reading.speed_mech_rad_s) << "and a blind reading hands back no speed";
        }
    }
}

// 5b. The same rule has to survive the wrap: 0.05 rad apart across +-pi is still 0.05 rad apart.
TEST(AngleSource, DisagreementIsMeasuredAcrossTheWrapNotThroughIt) {
    AngleSource source;
    ASSERT_EQ(ErrorCode::Ok, source.configure(make_config()));
    const float near_pi = 3.12F;
    const float past_pi = -3.12F;
    const AngleOffer offers[2] = {make_offer(AngleSourceKind::Sensor, near_pi, 1.0F),
                                  make_offer(AngleSourceKind::Observer, past_pi, 1.0F)};
    const auto& reading = source.step(offers, 2U, kDt);
    // 3.12 and -3.12 are 0.043 rad apart going forwards through pi, and 6.24 rad apart going the long way round.
    // 3.12 与 -3.12 穿过 pi 往前走相差 0.043 rad，绕远路则相差 6.24 rad。
    EXPECT_NEAR(std::fabs(kTwoPi - 6.24F), reading.disagreement_rad, 3e-3F) << reading.disagreement_rad;
    EXPECT_NE(AngleQuality::Blind, reading.quality) << "a wrapped pair is not a broken pair";
}

// 6. The window is a number: a source that stops answering is held for exactly that long, then replaced.
TEST(AngleSource, ALostSourceIsHeldForTheDeclaredWindowAndThenReplaced) {
    AngleSource source;
    ASSERT_EQ(ErrorCode::Ok, source.configure(make_config()));
    const AngleOffer alive[2] = {make_offer(AngleSourceKind::Sensor, 0.8F, 3.0F),
                                 make_offer(AngleSourceKind::Observer, 0.8F, 3.0F)};
    const auto& first = source.step(alive, 2U, kDt);
    ASSERT_EQ(AngleQuality::Ok, first.quality) << "the fixture has to start healthy or the count means nothing";
    ASSERT_EQ(AngleSourceKind::Sensor, first.source);

    // The sensor stops answering; the observer keeps reporting the same angle the machine is actually at.
    // 传感器不再回答；观测器继续报机器真正所在的那个角度。
    const AngleOffer sensor_gone[2] = {make_missing(AngleSourceKind::Sensor),
                                       make_offer(AngleSourceKind::Observer, 0.8F, 3.0F)};
    int steps = 0;
    int switched_at = -1;
    for (; steps < 200; ++steps) {
        const auto& reading = source.step(sensor_gone, 2U, kDt);
        if (reading.source == AngleSourceKind::Observer) {
            switched_at = steps + 1;
            break;
        }
        EXPECT_EQ(AngleSourceKind::Sensor, reading.source) << "inside the window the old source still answers";
        EXPECT_EQ(AngleQuality::Degraded, reading.quality) << "and it is already saying so";
    }
    // 2 ms at 50 us is 40 steps; the switch lands on the step where the elapsed time reaches the window.
    // 2 ms 在 50 us 步长上是 40 步；换源落在累计时间达到窗口的那一步上。
    EXPECT_EQ(40, switched_at) << "the window has to be the number it says it is";
}

// 7. A hand-over carries the angle across the switch; it does not jump by the offset between the two sources' zeros.
TEST(AngleSource, AHandOverDoesNotJumpTheAngle) {
    AngleSource source;
    ASSERT_EQ(ErrorCode::Ok, source.configure(make_config()));
    // The two sources disagree about where zero is by 0.02 rad -- a real mounting offset, and both are "right".
    // 两个来源对零点在哪相差 0.02 rad——真实的安装偏移，而两者都"没错"。
    const AngleOffer healthy[2] = {make_offer(AngleSourceKind::Sensor, 1.00F, 2.0F),
                                   make_offer(AngleSourceKind::Observer, 0.98F, 2.0F)};
    ASSERT_EQ(AngleQuality::Ok, source.step(healthy, 2U, kDt).quality);
    ASSERT_FLOAT_EQ(1.00F, source.reading().angle_mech_rad);

    const AngleOffer sensor_gone[2] = {make_missing(AngleSourceKind::Sensor),
                                       make_offer(AngleSourceKind::Observer, 0.98F, 2.0F)};
    for (int i = 0; i < 41; ++i) {
        static_cast<void>(source.step(sensor_gone, 2U, kDt));
    }
    const auto& after = source.reading();
    ASSERT_EQ(AngleSourceKind::Observer, after.source) << "the hand-over has to have happened for this to mean much";
    EXPECT_NEAR(1.00F, after.angle_mech_rad, 1e-5F)
        << "the reported angle continues from where it was, not from the incoming source's own zero";
    // The arithmetic that produced it, restated by the test rather than by the type: the incoming reading plus the
    // offset measured at the hand-over (1.00 - 0.98) is the angle that was already being reported.
    // 产生它的算术，由用例重述而不是由类型重述：进来的读数加上换源那一刻量到的偏移（1.00 − 0.98），就是当时一直在
    // 报的那个角度。
    EXPECT_NEAR(after.angle_mech_rad, 0.98F + (1.00F - 0.98F), 1e-5F);
}

// 8. The counterfactual: what the call site used to do, and why the type is not decoration.
TEST(AngleSource, WithoutArbitrationAFailedSensorIsAConfidentZero) {
    // The old shape in one line: take the sensor, and when it says nothing take zero.
    // 旧形状就一行：取传感器，它不说话时取零。
    const auto naive = [](bool sensor_valid, float sensor_angle) { return sensor_valid ? sensor_angle : 0.0F; };

    const float truth = 1.10F;
    EXPECT_FLOAT_EQ(truth, naive(true, truth));

    // The sensor dies at t = 0. The machine keeps turning, so the angle keeps moving; the naive path reports a
    // perfectly ordinary zero the whole time and there is no word anywhere that says otherwise.
    // 传感器在 t=0 死掉。机器继续转，于是角度继续走；朴素路径一直报一个完全普通的零，而任何地方都没有一个词说不是这样。
    const float naive_angle = naive(false, truth);
    const float naive_error = std::fabs(std::remainder(naive_angle - truth, kTwoPi));
    EXPECT_NEAR(truth, naive_error, 1e-5F) << "the error is the whole angle, and it is reported as a value";

    // The arbiter, on the same event: it keeps answering with the observable it still has and says `Degraded`.
    // 仲裁器在同一个事件上：用它仍然拥有的那个观测量继续回答，并说 `Degraded`。
    AngleSource source;
    ASSERT_EQ(ErrorCode::Ok, source.configure(make_config()));
    const AngleOffer alive[2] = {make_offer(AngleSourceKind::Sensor, truth, 1.0F),
                                 make_offer(AngleSourceKind::Observer, truth, 1.0F)};
    ASSERT_EQ(AngleQuality::Ok, source.step(alive, 2U, kDt).quality);
    const AngleOffer gone[2] = {make_missing(AngleSourceKind::Sensor),
                                make_offer(AngleSourceKind::Observer, truth, 1.0F)};
    const auto& during_window = source.step(gone, 2U, kDt);
    EXPECT_EQ(AngleQuality::Degraded, during_window.quality) << "the difference is this word";
    EXPECT_NEAR(truth, during_window.angle_mech_rad, 1e-5F) << "and the answer stayed the machine's angle";
    EXPECT_LT(std::fabs(std::remainder(during_window.angle_mech_rad - truth, kTwoPi)), 0.05F);
}

// 9. A reset makes the next step a first step, held angle and offsets included.
TEST(AngleSource, AResetForgetsTheHeldAngleAndTheHandOverOffset) {
    AngleSource source;
    ASSERT_EQ(ErrorCode::Ok, source.configure(make_config()));
    const AngleOffer alive[2] = {make_offer(AngleSourceKind::Sensor, 1.00F, 1.0F),
                                 make_offer(AngleSourceKind::Observer, 0.98F, 1.0F)};
    ASSERT_EQ(AngleQuality::Ok, source.step(alive, 2U, kDt).quality);

    source.reset();
    EXPECT_EQ(AngleQuality::Blind, source.reading().quality);
    EXPECT_FLOAT_EQ(0.0F, source.reading().angle_mech_rad) << "a reset leaves nothing behind to be inherited";

    // After a reset the arbiter is unconfigured again, so it refuses rather than using a zeroed-out band.
    // 复位之后仲裁器又回到未配置，于是它拒绝，而不是拿一个被清零的区间去用。
    const AngleOffer observer_only[2] = {make_missing(AngleSourceKind::Sensor),
                                         make_offer(AngleSourceKind::Observer, 0.98F, 1.0F)};
    EXPECT_EQ(AngleQuality::Blind, source.step(observer_only, 2U, kDt).quality);

    ASSERT_EQ(ErrorCode::Ok, source.configure(make_config()));
    const auto& fresh = source.step(observer_only, 2U, kDt);
    EXPECT_EQ(AngleSourceKind::Observer, fresh.source);
    EXPECT_FLOAT_EQ(0.98F, fresh.angle_mech_rad) << "and the offset from before the reset is gone with it";
}

} // namespace
