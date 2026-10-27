#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/polarity.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>



namespace {

constexpr float kDt = 5e-5F; // s, 20 kHz
constexpr float kPulseV = 1.0F;

auto valid_config() -> foc::core::MagnetPolarityTest::Config {
    return foc::core::MagnetPolarityTest::Config{kPulseV, 2e-4F, 1e-4F, 0.05F};
}

/**
 * @brief [en] A machine reduced to the one phenomenon the test is supposed to read: the d-axis
 *        inductance depends on the direction of the current, so the same volt-second pulse rises
 *        further one way than the other.
 *        [zh] 把电机简化成测试唯一要读的那个现象：d 轴电感与电流方向有关，于是相同伏秒的脉冲在一个方向上
 *        升得更多。
 */
class FakeDx final {
  public:
    FakeDx(float ld_positive, float ld_negative) noexcept : ld_positive_(ld_positive), ld_negative_(ld_negative) {}

    void apply(foc::core::Ab<float> v_ab, float dt_s) noexcept { i_ += v_ab.alpha * dt_s / (v_ab.alpha >= 0.0F ? ld_positive_ : ld_negative_); }

    [[nodiscard]] auto current() const noexcept -> foc::core::Ab<float> { return foc::core::Ab<float>{i_, 0.0F}; }

  private:
    float ld_positive_;
    float ld_negative_;
    float i_{0.0F};
};

/// [en] Runs the whole sequence and returns the verdict. [zh] 跑完整个序列并返回结论。
auto run_sequence(const foc::core::MagnetPolarityTest::Config& config, float ld_positive, float ld_negative) -> foc::core::PolarityVerdict {
    foc::core::MagnetPolarityTest test(config);
    test.begin(0.0F); // pulses along alpha

    FakeDx machine(ld_positive, ld_negative);
    for (int i = 0; i < 20000; ++i) {
        const foc::core::MagnetPolarityTest::Output out = test.step(kDt, machine.current());
        machine.apply(out.v_ab, kDt);
        if (!out.running) {
            return out.verdict;
        }
    }
    return foc::core::PolarityVerdict::Inconclusive;
}

} // namespace

TEST(MagnetPolarityTest, ARiseThatIsLargerTowardsTheMagnetMeansAligned) {
    // Reinforcing the rotor flux saturates the iron, so `+d` has the smaller incremental inductance
    // and the larger rise. This is the machine where the saliency loop's answer was already right.
    // 增强转子磁链使铁心饱和，所以 `+d` 的增量电感更小、上升更大。这就是凸极环答案本来就对的那台机器。
    EXPECT_EQ(foc::core::PolarityVerdict::Aligned, run_sequence(valid_config(), 1.8e-4F, 2.2e-4F));
}

TEST(MagnetPolarityTest, ARiseThatIsLargerAgainstTheEstimateMeansReversed) {
    // The same asymmetry, seen through an estimate that is half a turn out: `+d` of the estimate is the
    // magnet's `-d`, so the larger rise lands on the negative pulse.
    // 同样的不对称，但估计差半圈来看：估计的 `+d` 是磁钢的 `−d`，于是更大的上升落在负脉冲上。
    EXPECT_EQ(foc::core::PolarityVerdict::Reversed, run_sequence(valid_config(), 2.2e-4F, 1.8e-4F));
}

TEST(MagnetPolarityTest, AMachineWithoutSaturationAnswersNothingRatherThanGuessing) {
    // No asymmetry, no information. Reporting a direction anyway would be a coin flip presented as a
    // measurement, and the caller would have no way to tell it apart from a real one.
    // 没有不对称就没有信息。硬报一个方向等于把抛硬币当成测量值，而调用方无法把它与真测量区分开。
    EXPECT_EQ(foc::core::PolarityVerdict::Inconclusive, run_sequence(valid_config(), 2.0e-4F, 2.0e-4F));

    // And an asymmetry below the threshold is not promoted either, even though it is non-zero.
    // 低于阈值的差异同样不升级为结论，即使它不为零。
    foc::core::MagnetPolarityTest::Config strict = valid_config();
    strict.min_asymmetry_a = 100.0F;
    EXPECT_EQ(foc::core::PolarityVerdict::Inconclusive, run_sequence(strict, 1.8e-4F, 2.2e-4F));
}

TEST(MagnetPolarityTest, ThePulsesPointAlongTheGivenEstimate) {
    // The direction is the caller's estimate, not something the test invents: pulsing along a frame it
    // chose itself would measure a different axis than the one whose polarity is in question.
    // 方向来自调用方的估计，而不是测试自己编的：沿自选坐标系脉冲，测到的就不是待判极性的那条轴。
    foc::core::MagnetPolarityTest test(valid_config());
    const float theta = 0.7F;
    test.begin(theta);

    bool saw_positive = false;
    bool saw_negative = false;
    FakeDx machine(1.8e-4F, 2.2e-4F);
    for (int i = 0; i < 20000; ++i) {
        const foc::core::MagnetPolarityTest::Output out = test.step(kDt, machine.current());
        machine.apply(out.v_ab, kDt);

        const float magnitude = std::sqrt((out.v_ab.alpha * out.v_ab.alpha) + (out.v_ab.beta * out.v_ab.beta));
        if (magnitude > 0.0F) {
            // Either exactly +pulse or exactly -pulse along that axis -- never anything in between.
            // 要么正好是沿该轴的 +脉冲，要么正好是 −脉冲，绝无中间值。
            EXPECT_NEAR(kPulseV, magnitude, 1e-6F);
            const float expected = (out.v_ab.alpha * std::cos(theta)) + (out.v_ab.beta * std::sin(theta));
            EXPECT_NEAR(expected > 0.0F ? kPulseV : -kPulseV, expected, 1e-6F);
            EXPECT_NEAR(0.0F, (out.v_ab.beta * std::cos(theta)) - (out.v_ab.alpha * std::sin(theta)), 1e-6F);
            saw_positive = saw_positive || (expected > 0.0F);
            saw_negative = saw_negative || (expected < 0.0F);
        }
        if (!out.running) {
            break;
        }
    }
    EXPECT_TRUE(saw_positive) << "no +d pulse was ever applied";
    EXPECT_TRUE(saw_negative) << "no -d pulse was ever applied";
}

TEST(MagnetPolarityTest, BeginningAgainClearsAPreviousVerdict) {
    // Retrying after Inconclusive is the reason begin() exists, and it only works if the old answer is
    // cleared: a stale verdict turns "ask again" into "you already asked".
    // Inconclusive 之后补试正是 begin() 存在的理由，而它只有在旧结论被清掉时才有效：留着旧结论会把
    // “再问一次”变成“你问过了”。
    foc::core::MagnetPolarityTest test(valid_config());
    test.begin(0.0F);
    FakeDx salient(1.8e-4F, 2.2e-4F);
    for (int i = 0; i < 20000; ++i) {
        const foc::core::MagnetPolarityTest::Output out = test.step(kDt, salient.current());
        salient.apply(out.v_ab, kDt);
        if (!out.running) {
            break;
        }
    }
    ASSERT_EQ(foc::core::PolarityVerdict::Aligned, test.verdict());

    test.begin(0.0F);
    EXPECT_EQ(foc::core::PolarityVerdict::Inconclusive, test.verdict());
    EXPECT_EQ(foc::ErrorCode::Ok, test.status());
}

TEST(MagnetPolarityTest, RejectsEachUnusableConfiguration) {
    // Zero amplitude and zero duration would produce two identical, empty pulses and therefore a
    // confident "Inconclusive" -- which reads like a property of the machine instead of a
    // configuration nobody filled in.
    // 零幅值与零时长会产生两个完全相同、什么也没测到的脉冲，从而给出一个自信的 Inconclusive——那读起来
    // 像机器的属性，而不像一份没人填的配置。
    for (int field = 0; field < 4; ++field) {
        foc::core::MagnetPolarityTest::Config config = valid_config();
        switch (field) {
        case 0: config.pulse_v = 0.0F; break;
        case 1: config.pulse_s = 0.0F; break;
        case 2: config.settle_s = -1e-3F; break;
        default: config.min_asymmetry_a = -1.0F; break;
        }

        foc::core::MagnetPolarityTest test(config);
        EXPECT_EQ(foc::ErrorCode::Param, test.status()) << "field " << field;
        test.begin(0.0F);
        EXPECT_EQ(foc::ErrorCode::Param, test.status()) << "field " << field;

        // A rejected test applies nothing: a caller that ignored status() still cannot put a pulse of
        // a size nobody chose on the phases.
        // 被拒绝的测试不施加任何东西：即使调用方忽略了 status()，也无法把一份没人选定大小的脉冲加到各相上。
        const foc::core::MagnetPolarityTest::Output out = test.step(kDt, foc::core::Ab<float>{1.0F, 0.0F});
        EXPECT_FLOAT_EQ(0.0F, out.v_ab.alpha);
        EXPECT_FLOAT_EQ(0.0F, out.v_ab.beta);
    }
}

TEST(MagnetPolarityTest, RejectsNonPositivePeriod) {
    foc::core::MagnetPolarityTest test(valid_config());
    test.begin(0.0F);
    ASSERT_EQ(foc::ErrorCode::Ok, test.status());

    static_cast<void>(test.step(0.0F, foc::core::Ab<float>{0.0F, 0.0F}));
    EXPECT_EQ(foc::ErrorCode::Param, test.status());
}
