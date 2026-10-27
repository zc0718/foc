#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/observer/blended_observer.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>



namespace {

/**
 * @brief [en] A stand-in observer whose answer is dictated by the test.
 *        [zh] 答案完全由测试指定的替身观测器。
 *
 * @note [en] The blending maths is what is under test here, so it needs inputs it can predict; the
 *       real observers are exercised by S16.
 *       [zh] 这里考的是混合算法本身，因此需要可预测的输入；真实观测器由 S16 覆盖。
 * @since 0.1.0
 */
struct FakeObserver final {
    foc::core::ObsState answer{0.0F, 0.0F};
    foc::core::Ab<float> carrier{0.0F, 0.0F};
    bool lock{true};
    int injections{0};

    auto update(foc::core::Ab<float>, foc::core::Ab<float>, float) noexcept -> foc::core::ObsState { return answer; }

    [[nodiscard]] auto injection_ab() const noexcept -> foc::core::Ab<float> {
        return carrier;
    }

    [[nodiscard]] auto locked() const noexcept -> bool { return lock; }

    void reset() noexcept { answer = foc::core::ObsState{0.0F, 0.0F}; }
};

using Blend = foc::core::BlendedObserver<FakeObserver, FakeObserver>;

/// [en] Wraps to (-pi, pi]. [zh] 归一化到 (-π, π]。
auto wrap_to_pi(float angle) noexcept -> float {
    while (angle <= -foc::core::kPi<float>) {
        angle += foc::core::kTwoPi<float>;
    }
    while (angle > foc::core::kPi<float>) {
        angle -= foc::core::kTwoPi<float>;
    }
    return angle;
}

auto make_blend(float start, float end, float lpf = 20.0F) -> Blend {
    return Blend(FakeObserver{}, FakeObserver{}, Blend::Config{start, end, lpf});
}

/// [en] Drives the blend for a while at a constant high-speed reading, leaving the angles the caller
///      set alone. [zh] 以恒定高速读数驱动混合一段时间，不动调用方设定的角度。
void settle(Blend& blend, float high_speed, int steps = 4000) {
    blend.high().answer.omega_e = high_speed;
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(blend.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, 5e-5F));
    }
}

} // namespace

static_assert(foc::core::concepts::is_observer_v<Blend>, "the blend must satisfy the observer contract");
static_assert(foc::core::concepts::has_injection_v<Blend>, "the blend must forward the carrier hook");
static_assert(foc::core::concepts::has_locked_v<Blend>, "the blend must forward the lock hook");

TEST(BlendedObserver, TheWeightRampsAcrossTheBandAndSaturatesOutsideIt) {
    Blend blend = make_blend(100.0F, 200.0F);

    settle(blend, 50.0F);
    EXPECT_FLOAT_EQ(0.0F, blend.blend());
    settle(blend, 150.0F);
    EXPECT_NEAR(0.5F, blend.blend(), 0.01F);
    settle(blend, 250.0F);
    EXPECT_FLOAT_EQ(1.0F, blend.blend());
    // Coming back down must traverse the same ramp: the weight is a function of speed, not a latch.
    // 回落时必须走同一条斜坡：权重是转速的函数，不是锁存器。
    settle(blend, 150.0F);
    EXPECT_NEAR(0.5F, blend.blend(), 0.01F);
    settle(blend, 50.0F);
    EXPECT_FLOAT_EQ(0.0F, blend.blend());
}

TEST(BlendedObserver, TheAngleBlendNeverJumpsAWholeTurnAtTheWrap) {
    // The two estimates sit on opposite sides of the wrap whenever one of them crosses zero. A plain
    // weighted sum of 0.05 and 6.23 gives ~3.1 -- a half-turn error introduced by the mixer itself.
    // 只要其中一个跨过零点，两个估计就分处环绕的两侧。把 0.05 与 6.23 直接加权平均得到约 3.1——
    // 半圈的误差由混合器自己引入。
    Blend blend = make_blend(100.0F, 200.0F);
    blend.low().answer = foc::core::ObsState{foc::core::kTwoPi<float> - 0.05F, 0.0F};
    blend.high().answer = foc::core::ObsState{0.05F, 300.0F};
    settle(blend, 300.0F);

    // Fully handed over, so the answer is the high-speed one exactly -- on the same side of the wrap.
    // 完全交接，于是答案就是高速的那一个，且落在环绕的同一侧。
    EXPECT_NEAR(0.0F, wrap_to_pi(blend.state().theta_e - 0.05F), 1e-3F);

    // Half way: still inside a hundredth of a radian of the two, not half a turn away.
    // 一半权重时：仍在两者百分之一弧度之内，而不是差半圈。
    Blend halfway = make_blend(100.0F, 200.0F);
    halfway.low().answer = foc::core::ObsState{foc::core::kTwoPi<float> - 0.05F, 0.0F};
    halfway.high().answer = foc::core::ObsState{0.05F, 150.0F};
    settle(halfway, 150.0F);
    EXPECT_NEAR(0.0F, wrap_to_pi(halfway.state().theta_e), 0.06F);
}

TEST(BlendedObserver, TheCarrierIsWoundDownByTheHandoverAndIsZeroAtTheEnd) {
    // Paying for a carrier that is no longer needed is a real cost -- current, heat, audible noise --
    // so it must go to exactly zero, not merely to something small.
    // 为一个不再需要的载波付出代价是真实成本——电流、发热、可听噪声——因此它必须精确到零，而不是"很小"。
    Blend blend = make_blend(100.0F, 200.0F);
    blend.low().carrier = foc::core::Ab<float>{4.0F, 0.0F};

    settle(blend, 50.0F);
    EXPECT_FLOAT_EQ(4.0F, blend.injection_ab().alpha);

    settle(blend, 150.0F);
    EXPECT_NEAR(2.0F, blend.injection_ab().alpha, 0.05F);

    settle(blend, 300.0F);
    EXPECT_FLOAT_EQ(0.0F, blend.injection_ab().alpha);
    EXPECT_FLOAT_EQ(0.0F, blend.injection_ab().beta);
}

TEST(BlendedObserver, StandstillNoiseInTheHighSpeedObserverDoesNotOpenTheHandover) {
    // A flux observer's speed is differentiated from an angle, so at standstill it is noise around zero.
    // Rectifying that noise before filtering gives it a positive mean and hands control to a source
    // with nothing to say -- measured as 0.30 rad of error at standstill before the order was fixed.
    // 磁链观测器的转速是对角度微分得来的，静止时就是绕零的噪声。先取模再滤波会得到正均值，把控制权交给
    // 一个无话可说的源——顺序修正之前，静止处实测到 0.30 rad 的误差。
    Blend blend = make_blend(100.0F, 200.0F);
    blend.low().answer = foc::core::ObsState{0.1F, 0.0F};

    float seed = 1.0F;
    for (int i = 0; i < 20000; ++i) {
        // Deterministic alternating noise of +-150 rad/s: if it were rectified before filtering, the
        // weight would sit around 0.7 and the answer would be a mix of a good estimate and a garbage one.
        // 确定性的 ±150 rad/s 交替噪声：若在滤波之前取模，权重会停在 0.7 左右，答案就成了好估计与垃圾的混合。
        seed = -seed;
        blend.high().answer = foc::core::ObsState{3.0F, 150.0F * seed};
        static_cast<void>(blend.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, 5e-5F));
    }
    EXPECT_LT(blend.blend(), 0.02F);
    EXPECT_NEAR(0.1F, blend.state().theta_e, 0.01F);
}

TEST(BlendedObserver, ABlindObserverContributesNothingAboveTheBand) {
    // The low-speed observer's own verdict has to be able to take it out of the mix. Two reasons, both
    // measured: at the far end of the handover the carrier has been wound down until the loop honestly
    // reports itself blind, and a blind saliency loop can be a whole turn out -- mixing even a few
    // percent of an estimate that is pi away injects a large error, because the shortest arc to a value
    // pi away is pi.
    // 低速观测器自己的结论必须能把它移出混合。两条理由都是实测：交接末端载波被收掉，直到环路诚实报告
    // 自己看不见；而看不见的凸极环可能差了整整一圈——把差 π 的估计混进哪怕百分之几，也会注入很大的
    // 误差，因为到差 π 的值的最短弧就是 π。
    Blend blend = make_blend(100.0F, 200.0F);
    blend.low().lock = false;
    blend.high().answer = foc::core::ObsState{0.5F, 150.0F};

    settle(blend, 150.0F);
    EXPECT_TRUE(blend.locked()) << "inside the band the high-speed side is in its regime";
    EXPECT_FLOAT_EQ(1.0F, blend.blend()) << "a blind low-speed observer gets no share";
    EXPECT_NEAR(0.5F, blend.state().theta_e, 1e-3F) << "the answer is the high-speed estimate alone";

    // Below the band there is nothing to hand over to, so the weight stays where the speed puts it and
    // the verdict follows the observer that actually has to answer.
    // 带下侧无处可交，权重就留在转速决定的位置，结论也随那个真正要负责回答的观测器。
    Blend slow = make_blend(100.0F, 200.0F);
    slow.low().lock = false;
    slow.high().answer = foc::core::ObsState{0.5F, 50.0F};
    settle(slow, 50.0F);
    EXPECT_FLOAT_EQ(0.0F, slow.blend());
    EXPECT_FALSE(slow.locked()) << "below the band, a blind low-speed observer means no answer at all";

    // And with the low-speed side healthy, the band is what decides -- not the weight.
    // 低速侧健康时，决定结论的是带宽而不是权重。
    Blend healthy = make_blend(100.0F, 200.0F);
    healthy.high().answer = foc::core::ObsState{0.5F, 150.0F};
    settle(healthy, 150.0F);
    EXPECT_TRUE(healthy.locked());
    EXPECT_LT(healthy.blend(), 1.0F) << "still blending, yet the answer already counts";
}

TEST(BlendedObserver, RejectsAnEmptyBandAndANonPositiveFilter) {
    // An empty band is a hard switch driven by an estimated speed, which chatters; it is refused rather
    // than supported. A zero filter leaves the weight at the mercy of a differentiated angle.
    // 空带宽就是由估计转速驱动的硬开关，会抖；因此拒绝而不支持。零滤波则让权重受制于微分角度。
    EXPECT_EQ(foc::ErrorCode::Param, make_blend(100.0F, 100.0F).status());
    EXPECT_EQ(foc::ErrorCode::Param, make_blend(200.0F, 100.0F).status());
    EXPECT_EQ(foc::ErrorCode::Param, make_blend(100.0F, 200.0F, 0.0F).status());
    EXPECT_EQ(foc::ErrorCode::Ok, make_blend(100.0F, 200.0F).status());

    Blend blend = make_blend(100.0F, 100.0F);
    EXPECT_FLOAT_EQ(0.0F, blend.update(foc::core::Ab<float>{1.0F, 0.0F}, foc::core::Ab<float>{1.0F, 0.0F}, 5e-5F).theta_e);
    EXPECT_FLOAT_EQ(0.0F, blend.injection_ab().alpha);
}

TEST(BlendedObserver, RejectsNonPositivePeriodAndResetClearsTheHandover) {
    Blend blend = make_blend(100.0F, 200.0F);
    settle(blend, 300.0F);
    ASSERT_FLOAT_EQ(1.0F, blend.blend());

    blend.reset();
    EXPECT_FLOAT_EQ(0.0F, blend.blend());
    EXPECT_FLOAT_EQ(0.0F, blend.state().theta_e);
    EXPECT_EQ(foc::ErrorCode::Ok, blend.status());

    static_cast<void>(blend.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, 0.0F));
    EXPECT_EQ(foc::ErrorCode::Param, blend.status());
}
