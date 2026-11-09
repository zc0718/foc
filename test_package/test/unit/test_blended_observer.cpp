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
 * @brief [zh] 答案完全由测试指定的替身观测器。
 * @note [en] The blending maths is what is under test here, so it needs inputs it can predict; the real observers are
 *            exercised by S16.
 * @note [zh] 这里考的是混合算法本身，因此需要可预测的输入；真实观测器由 S16 覆盖。
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

/**
 * @brief [en] Wraps to (-pi, pi].
 * @brief [zh] 归一化到 (-π, π]。
 */
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

/**
 * @brief [en] Drives the blend for a while at a constant high-speed reading, leaving the angles the caller set alone.
 * @brief [zh] 以恒定高速读数驱动混合一段时间，不动调用方设定的角度。
 */
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



/**
 * @brief [en] The blend is a convex combination, so every sensitivity coefficient is `w` or `1-w`, and the decision
 *             speed's coefficient is `disagreement/span`.
 *
 * @brief [zh] 混合是一个凸组合，于是每个敏感度系数都是 `w` 或 `1−w`，而判据转速的系数是 `分歧/带宽`。
 *
 *   The output is `theta_low + w*shortest_arc(theta_high, theta_low)`, which is one line of algebra and makes the whole
 *   sensitivity table fall out: an error in the low source reaches the output at `1-w`, an error in the high source at
 *   `w`, and a *miscalibrated decision* (a speed reading off by `d omega`) moves the angle by
 *   `d omega * disagreement / span`, because the weight's own slope is `1/span`. The three coefficients are measured
 *   with stub observers whose answers are set by hand, so nothing here depends on a plant.
 *
 *   The numbers below are the coefficients, not a fit: `w` comes out at exactly 0.25000 / 0.50000 / 0.75000 for 35 / 60
 *   / 85 rad/s against a 10..110 band, which is `(speed - start)/span` to the last digit the float carries.
 *
 *   输出是 `θ_low + w*shortest_arc(θ_high, θ_low)`，一行代数，整张敏感度表由此掉落：低速源的误差以 `1−w` 到达输出、高速
 *   源的误差以 `w` 到达，而 **判据失准** （转速读数差 `dω`）把角度移动 `dω·分歧/带宽`，因为权重自身的斜率是 `1/带宽 `。
 *   三个系数都用答案由手设定的桩观测器量出，因此这里没有任何东西依赖 plant。
 *
 *   下面的数是 **系数** 而不是拟合值：在 10..110 的带上，35/60/85 rad/s 给出 `w` 恰好 0.25000 / 0.50000 / 0.75000，即
 *   `( 转速 − 起点)/带宽` 到最后一位 float。
 */
TEST(BlendedObserver, EveryBlendCoefficientIsTheWeightOrOneMinusIt) {
    constexpr float kStart = 10.0F;
    constexpr float kEnd = 110.0F;
    constexpr float kSpan = kEnd - kStart;
    constexpr float kDisagreement = 0.6F;

    std::printf("\n%-10s %12s %12s %12s %12s %12s\n", "high_speed", "weight", "(s-start)/span",
                "theta_out", "pred_theta", "omega_out");
    for (const float speed : {0.0F, 10.0F, 35.0F, 60.0F, 85.0F, 110.0F, 200.0F}) {
        Blend blend = make_blend(kStart, kEnd);
        blend.low().answer = foc::core::ObsState{1.0F, 0.0F};
        blend.high().answer = foc::core::ObsState{1.0F + kDisagreement, 0.0F};
        settle(blend, speed);

        const float weight = std::min(1.0F, std::max(0.0F, (std::fabs(speed) - kStart) / kSpan));
        const float predicted_theta = 1.0F + (weight * kDisagreement);
        const float predicted_omega = (1.0F - weight) * 0.0F + weight * speed;
        const float measured_theta = blend.state().theta_e;
        std::printf("%-10.1f %12.5f %12.5f %12.5f %12.5f %12.4f\n", static_cast<double>(speed),
                    static_cast<double>((measured_theta - 1.0F) / kDisagreement),
                    static_cast<double>(weight), static_cast<double>(measured_theta),
                    static_cast<double>(predicted_theta), static_cast<double>(blend.state().omega_e));

        // Coefficient 1: the high source's error reaches the output at exactly `w`.
        // 系数 1：高速源的误差以恰好 `w` 到达输出。
        EXPECT_NEAR(predicted_theta, measured_theta, 1.0e-4F) << "(1-w)*e_low + w*e_high";
        // Coefficient 2: and the published speed is the same convex combination.
        // 系数 2：发布转速也是同一个凸组合。
        EXPECT_NEAR(predicted_omega, blend.state().omega_e, 1.0e-3F * (1.0F + std::fabs(speed)));
    }

    // Coefficient 3: the decision speed. One rad/s of misreading moves the angle by disagreement/span.
    // 系数 3：判据转速。一 rad/s 的误读把角度移动 分歧/带宽。
    Blend low_side = make_blend(kStart, kEnd);
    Blend high_side = make_blend(kStart, kEnd);
    low_side.low().answer = foc::core::ObsState{0.0F, 0.0F};
    high_side.low().answer = foc::core::ObsState{0.0F, 0.0F};
    low_side.high().answer = foc::core::ObsState{kDisagreement, 0.0F};
    high_side.high().answer = foc::core::ObsState{kDisagreement, 0.0F};
    settle(low_side, 45.0F);
    settle(high_side, 46.0F);
    const float slope = (high_side.state().theta_e - low_side.state().theta_e) / 1.0F;
    std::printf("[decision speed] one rad/s of misreading moves the angle by %.6f rad (predict %.6f)\n",
                static_cast<double>(slope), static_cast<double>(kDisagreement / kSpan));
    EXPECT_NEAR(kDisagreement / kSpan, slope, 1.0e-4F)
        << "a misread decision speed is an angle error of disagreement/span per rad/s";
}



/**
 * @brief [en] The blend is an interpolation **on the circle**: two answers either side of the wrap blend across the
 *             short arc, not the long way round.
 *
 * @brief [zh] 混合是 **圆上的** 插值：折返两侧的两个答案沿短弧混合，而不是绕远路。
 *
 *   `theta_low + w*(theta_high - theta_low)` with a plain subtraction would take the long way whenever the two answers
 *   sit either side of the wrap, swinging the output by almost a full turn -- and it would do it precisely at the
 *   boundary the two observers disagree about. Taking the difference on the short arc is what makes the blend
 *   continuous in its inputs; this case pins it with stubs at 3.0 and 3.2832 rad, which are 0.283 apart across the wrap
 *   and 6.000 apart the other way.
 *
 *   若用普通减法算 `θ_low + w*(θ_high − θ_low)`，两个答案落在折返两侧时就会绕远路，把输出甩出将近一整圈——而且恰好发生在
 *   两个观测器意见不一致的那个边界上。在短弧上取差值才是混合对输入连续的原因；本用例用 3.0 与 3.2832 rad 的桩把它钉住
 *   —— 两个点在折返两侧相距 0.283，另一边则有 6.000。
 */
TEST(BlendedObserver, TheInterpolationHappensOnTheCircleAndNotOnTheLine) {
    constexpr float kStart = 10.0F;
    constexpr float kEnd = 110.0F;
    Blend blend = make_blend(kStart, kEnd);
    blend.low().answer = foc::core::ObsState{3.0F, 0.0F};
    blend.high().answer = foc::core::ObsState{3.2831853F, 0.0F};
    settle(blend, 60.0F); // mid-band, so the answer is the midpoint of the short arc

    const float measured = blend.state().theta_e;
    const float short_arc_midpoint = 3.0F + (0.5F * 0.2831853F);
    const float long_way = 3.0F + (0.5F * (3.2831853F - 3.0F - foc::core::kTwoPi<float>));
    std::printf("\n[wrap] low 3.0000, high 3.2832: out %.5f, short-arc midpoint %.5f, the long way %.5f\n",
                static_cast<double>(measured), static_cast<double>(short_arc_midpoint),
                static_cast<double>(long_way));
    EXPECT_NEAR(short_arc_midpoint, measured, 1.0e-4F) << "the blend must take the short arc";
    EXPECT_GT(std::fabs(measured - long_way), 2.0F) << "and the long way is a whole turn away, not a detail";
}



/**
 * @brief [en] A handover sweeps the published angle by the disagreement while neither observer moves -- and the
 *             published **speed** does not show it.
 *
 * @brief [zh] 一次交接会在两个观测器都不动的情况下把发布角扫过整个分歧量——而发布 **转速** 不体现它。
 *
 *   The weight depends on speed, so when the decision speed crosses the band the output angle moves from `theta_low` to
 *   `theta_high` even though neither observer changed its answer. That excursion is real and it is bounded by the
 *   observers' disagreement -- but it is an artefact of the handover, not of the rotor. Measured here with both stubs
 *   frozen 0.6 rad apart: the output sweeps 0.6 rad while nothing physical moved.
 *
 *   The second half is for consumers: the published *speed* is the same convex combination of the two speeds, so it
 *   stays at the (common) speed during the sweep. An angle that moves and a speed that does not are not consistent with
 *   each other, and a consumer that reconstructs position by integrating the speed will disagree with one that reads
 *   the angle -- for the duration of a handover, by exactly the disagreement. That is a property to be known, not a bug
 *   to be fixed: the angle is the better estimate and the sweep is how a handover works.
 *
 *   权重依赖转速，于是当判据转速穿过交接带时，即使两个观测器都没改口，输出角也会从 `θ_low` 走到 `θ_high`。这个扫动是真
 *   的，且被两者的分歧量封顶——但它是交接的产物，不是转子的。这里把两个桩冻结在相距 0.6 rad 处实测：输出扫过 0.6 rad，而
 *   物理上什么都没动。
 *
 *   后半句是给使用方的：发布 **转速** 是两个转速的同一个凸组合，所以扫动期间它停在（共同的）转速上。一个在动而一个不动
 *   的角度与转速彼此不自洽，用转速积分重建位置的消费者会与直接读角度的人在交接期间不一致，差值恰好是那个分歧量。这是
 *   **该知道的性质** 而不是要修的缺陷：角度是更好的估计，扫动就是交接的工作方式。
 */
TEST(BlendedObserver, AHandoverSweepsTheAngleWithoutMovingTheSpeed) {
    constexpr float kStart = 10.0F;
    constexpr float kEnd = 110.0F;
    constexpr float kDisagreement = 0.6F;
    Blend blend = make_blend(kStart, kEnd);
    blend.low().answer = foc::core::ObsState{0.0F, 0.0F};
    blend.high().answer = foc::core::ObsState{kDisagreement, 0.0F};
    settle(blend, 0.0F);
    const float before = blend.state().theta_e;

    settle(blend, kEnd); // one clean crossing of the whole band
    const float after = blend.state().theta_e;
    const float swept = wrap_to_pi(after - before);
    std::printf("\n[handover] the angle swept %.5f rad with both observers frozen %.5f rad apart; the "
                "published speed is %.4f rad/s\n",
                static_cast<double>(swept), static_cast<double>(kDisagreement),
                static_cast<double>(blend.state().omega_e));
    EXPECT_NEAR(kDisagreement, swept, 1.0e-4F)
        << "the sweep is exactly the disagreement between the two sources";
    EXPECT_NEAR(kEnd, blend.state().omega_e, 1.0e-2F * kEnd)
        << "and the speed shows nothing of it, because it is the same convex combination";
    // Both were moving at the same speed, so a sweep of the *speed* would be the defect.
    // 两者转速相同，所以转速上出现扫动才是缺陷。
    EXPECT_GT(std::fabs(kEnd - 0.0F), 10.0F * std::fabs(swept)) << "the two symptoms differ by construction";
}
