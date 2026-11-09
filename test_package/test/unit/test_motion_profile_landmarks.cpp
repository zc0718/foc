// The three ceilings of `MotionProfile` switch exactly where the algebra says, and the shaping gain
// appears in only one of the two switch points.
// `MotionProfile` 的三个上限精确地在其代数解处切换，而 shaping gain 只出现在两个切换点中的一个。

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/motion_profile.hpp>



namespace {

constexpr float kSettleDt = 5e-4F;

/**
 * @brief [en] The command a fresh profile settles on when the error is held constant.
 *
 * @brief [zh] 误差保持恒定时，一个全新的规划器最终稳定到的指令。
 *
 *   With nothing moving the demand is a constant, so the rate limiter walks the command onto it and stops there. That
 *   makes `min(g*|e|, sqrt(2*a*|e|), v_limit)` readable one ceiling at a time, with no plant and without
 *   re-implementing the law: the number measured is the law's own output. 反馈不动时需求是常数，于是限速率把指令走到它
 *   上面并停住。这让 `min(g*|e|, sqrt(2*a*|e|), v_limit)` 可以逐个上限读出来——不需要 plant，也不需要重写那条律：量到的
 *   就是律自己的输出。
 */
auto settled_command(float gain, float velocity_limit, float accel_limit, float error) -> float {
    foc::core::MotionProfile profile{
        foc::core::MotionProfile::Config{gain, velocity_limit, accel_limit}};
    float velocity = 0.0F;
    for (int i = 0; i < 4'000; ++i) {
        velocity = profile.step(error, 0.0F, kSettleDt);
    }
    return velocity;
}

/**
 * @brief [en] The largest error at which the velocity ceiling still binds, found by bisection.
 * @brief [zh] 速度上限仍然生效的最大误差，用二分找出。
 */
auto cruise_exit(float gain, float velocity_limit, float accel_limit) -> float {
    float below = 1.0e-6F;
    float above = 10.0F;
    for (int i = 0; i < 48; ++i) {
        const float middle = 0.5F * (below + above);
        const float command = settled_command(gain, velocity_limit, accel_limit, middle);
        if (command < velocity_limit * (1.0F - 1.0e-5F)) {
            below = middle;
        } else {
            above = middle;
        }
    }
    return above;
}

} // namespace



/**
 * @brief [en] The proportional term and the braking curve cross at `2a/g^2`, and the measurement there can tell which
 *             of the two formulas the code is using.
 *
 * @brief [zh] 比例项与制动曲线在 `2a/g^2` 处交叉，且那里的量测能分辨代码用的是哪个公式。
 *
 *   The crossover solves `g*e = sqrt(2*a*e)`, i.e. `e* = 2a/g^2`. Below it the proportional term is the smaller of the
 *   two, above it the braking curve is -- and the two differ by more than 40 % at the outer probe points, so agreeing
 *   with one is not agreeing with the other. The competing reading `sqrt(a*e)` (the constant 2 dropped) puts its
 *   crossover at `a/g^2`, half of this one, and is therefore excluded by the same measurement. 交叉点是 `g*e =
 *   sqrt(2*a*e)` 的解，即 `e* = 2a/g^2`。低于它在两者中更小的是比例项，高于它更小的是制动曲线；而外侧探点上两者相差
 *   40 % 以上，所以"与其中一个相符"不等于"与另一个也相符"。竞争读法 `sqrt(a*e)`（丢掉常数 2）的交叉点在 `a/g^2`，是这里
 *   的一半，因此被同一次量测排除。
 */
TEST(MotionProfile, TheProportionalTermAndTheBrakingCurveCrossAtTwoASquareOverGainSquared) {
    const struct {
        float gain;
        float accel;
    } pairs[] = {{20.0F, 10.0F}, {40.0F, 10.0F}, {20.0F, 20.0F}};

    std::printf("\n%-10s %-10s %-10s %12s %12s %12s\n", "gain", "accel", "e*", "e", "measured",
                "min(g*e, sqrt(2ae))");
    for (const auto& pair : pairs) {
        const float crossover = 2.0F * pair.accel / (pair.gain * pair.gain);
        for (const float k : {0.5F, 1.0F, 2.0F}) {
            const float error = k * crossover;
            const float proportional = pair.gain * error;
            const float braking = std::sqrt(2.0F * pair.accel * error);
            const float expected = std::min(proportional, braking);
            const float wrong = std::min(proportional, std::sqrt(pair.accel * error));
            const float measured = settled_command(pair.gain, 100.0F, pair.accel, error);
            std::printf("%-10.1f %-10.1f %-10.5f %12.6f %12.6f %12.6f\n",
                        static_cast<double>(pair.gain), static_cast<double>(pair.accel),
                        static_cast<double>(crossover), static_cast<double>(error),
                        static_cast<double>(measured), static_cast<double>(expected));

            // The law's own number, to a tenth of a percent.
            EXPECT_NEAR(expected, measured, 1.0e-3F * expected);
            // Above the proportional crossover the two formulas are far apart, so this measurement
            // has teeth. At `k = 0.5` they agree -- that point is the *wrong* formula's own
            // crossover (`a/g^2`) and the proportional term binds in both readings, so a probe
            // there would prove nothing. That is why the teeth check starts at `k = 1`.
            // 在比例交叉点以上两个公式相差很远，所以这条量测有牙齿。在 `k = 0.5` 处它们是一致的
            // ——那个点正是**错**公式自己的交叉点（`a/g^2`），两种读法都由比例项生效，在那里探
            // 什么也证明不了。这就是牙齿检查从 `k = 1` 开始的原因。
            if (k >= 1.0F) {
                EXPECT_GT(std::fabs(expected - wrong) / expected, 0.25F)
                    << "the probe has to be able to tell sqrt(2ae) from sqrt(ae)";
            }
            // At the crossover itself both ceilings are equal, which is the definition being used.
            if (k == 1.0F) {
                EXPECT_NEAR(proportional, braking, 1.0e-3F * expected);
            }
        }
    }
}



/**
 * @brief [en] The cruise ends at `v^2/(2a)`, which contains neither the shaping gain nor the distance.
 *
 * @brief [zh] 巡航在 `v^2/(2a)` 处结束，这个式子里既没有 shaping gain 也没有行程。
 *
 *   Found by bisection, not by evaluating a formula, so the exponents below are measured rather than restated: doubling
 *   the acceleration halves the exit, halving the velocity limit quarters it (the square), and multiplying the move by
 *   four leaves it untouched. The competing reading `v^2/a` (the constant 2 dropped again) is a factor of two away and
 *   is excluded. 用二分找出而不是代公式，所以下面的指数是量出来的而不是复述的：加速度加倍使退出点减半，速度上限减半使它
 *   变为四分之一（平方），行程乘四则完全不动。竞争读法 `v^2/a`（又丢掉常数 2）相差一倍，被排除。
 */
TEST(MotionProfile, TheCruiseEndsAtVelocitySquaredOverTwiceTheAcceleration) {
    const struct {
        float velocity_limit;
        float accel;
    } pairs[] = {{2.0F, 10.0F}, {2.0F, 20.0F}, {1.0F, 10.0F}};

    const float nominal = (2.0F * 2.0F) / (2.0F * 10.0F);
    std::printf("\n%-14s %-8s %12s %12s %10s\n", "v_limit", "accel", "v^2/(2a)", "measured",
                "ratio");
    for (const auto& pair : pairs) {
        const float expected =
            (pair.velocity_limit * pair.velocity_limit) / (2.0F * pair.accel);
        const float measured = cruise_exit(20.0F, pair.velocity_limit, pair.accel);
        const float ratio = measured / expected;
        std::printf("%-14.2f %-8.1f %12.5f %12.5f %10.4f\n",
                    static_cast<double>(pair.velocity_limit), static_cast<double>(pair.accel),
                    static_cast<double>(expected), static_cast<double>(measured),
                    static_cast<double>(ratio));

        EXPECT_NEAR(1.0F, ratio, 0.02F) << "the cruise exit is v^2/(2a)";
        // The halved-constant reading puts the exit at `v^2/a`, a factor of two away, and this
        // measurement excludes it rather than merely being consistent with the right one.
        // 丢掉常数 2 的读法把退出点放在 `v^2/a`，相差一倍；这条量测排除它，而不只是与对的那个
        // 相符。
        const float halved_constant =
            (pair.velocity_limit * pair.velocity_limit) / pair.accel;
        EXPECT_GT(std::fabs(measured - halved_constant) / halved_constant, 0.3F);
    }

    // The two exponents, stated as the ratios they predict.
    const float halved_accel = cruise_exit(20.0F, 2.0F, 20.0F) / cruise_exit(20.0F, 2.0F, 10.0F);
    const float halved_velocity =
        cruise_exit(20.0F, 1.0F, 10.0F) / cruise_exit(20.0F, 2.0F, 10.0F);
    std::printf("[exponents] accel %.4f (predict 0.5), velocity %.4f (predict 0.25),"
                " distance independent by construction\n",
                static_cast<double>(halved_accel), static_cast<double>(halved_velocity));
    EXPECT_NEAR(0.5F, halved_accel, 0.02F);
    EXPECT_NEAR(0.25F, halved_velocity, 0.015F);
    EXPECT_GT(nominal, 0.0F);
}



/**
 * @brief [en] The shaping gain reaches the structure only through the one inequality `g >= 2a/v`, and this is why
 *             doubling it moved nothing measurable (A-40).
 *
 * @brief [zh] shaping gain 只通过 `g >= 2a/v` 这一个不等式进入结构，这就是加倍它什么都没动的原因（A-40）。
 *
 *   A-40 recorded a negative result: the cruise gap stayed at 0.496 rad when the shaping gain doubled, and that refuted
 *   `e = v/g`. The refutation is right but it was an absence of explanation. The condition is this: at the cruise exit
 *   `e_c = v^2/(2a)` the proportional ceiling is `g*e_c`, and it binds only if `g*e_c < v`, i.e. `g < 2a/v`. At `g =
 *   20, a = 10, v = 2` the boundary is exactly 10, so the gain sits on the safe side and *no* change to it can move the
 *   exit -- which is what "a number that does not move when its parameter doubles is not that parameter's" looks like
 *   once the reason is written down. Below the boundary the ceiling that binds is the proportional one and the exit
 *   moves to `v/g`, where the gain is the whole story. A-40 记的是一个负结果：shaping gain 加倍时巡航间隙停在 0.496
 *   rad，从而否掉了 `e = v/g`。否掉是对的，但那只是"没有解释"。条件是：在巡航退出点 `e_c = v^2/(2a)` 处比例上限是
 *   `g*e_c`，它只有在 `g*e_c < v`（即 `g < 2a/v`）时才生效。在 `g = 20, a = 10, v = 2` 时边界正好是 10，所以增益在安全
 *   一侧，**任何**改动都动不了退出点——这就是"在其所谓参数加倍时不动的数就不是那个参数的"在写下原因之后的形状。边界以下
 *   生效的是比例上限，退出点移到 `v/g`，那里增益就是全部故事。
 */
TEST(MotionProfile, TheShapingGainIsAbsentFromTheCruiseExitAboveTwiceTheAccelerationOverVelocity) {
    constexpr float kGain = 20.0F;
    constexpr float kVelocityLimit = 2.0F;
    constexpr float kAccel = 10.0F;
    constexpr float kBoundary = (2.0F * kAccel) / kVelocityLimit; // 10 rad/s per rad
    constexpr float kExit = (kVelocityLimit * kVelocityLimit) / (2.0F * kAccel); // 0.2 rad

    // At the boundary the proportional ceiling is exactly the velocity limit, so both readings agree.
    EXPECT_NEAR(kVelocityLimit, settled_command(kBoundary, kVelocityLimit, kAccel, kExit), 1.0e-4F);

    // Above it, the gain is not in the answer: the command at the exit is the velocity limit for any
    // gain on the safe side.
    for (const float gain : {kBoundary, kGain, 2.0F * kGain, 100.0F * kGain}) {
        EXPECT_NEAR(kVelocityLimit, settled_command(gain, kVelocityLimit, kAccel, kExit), 1.0e-4F)
            << "gain " << gain << " is above 2a/v and must not reach the exit";
    }

    // Below it, the proportional ceiling is the binding one and the command at the exit is `g*e`,
    // which is *less* than the velocity limit: the cruise ends earlier, at `v/g`.
    const float weak_gain = 0.5F * kBoundary;
    const float weak_command = settled_command(weak_gain, kVelocityLimit, kAccel, kExit);
    std::printf("[regime] 2a/v %.2f; gain %.1f above -> exit command %.4f; gain %.1f below -> %.4f"
                " (g*e = %.4f, v*g/g = v would need gain %.1f)\n",
                static_cast<double>(kBoundary), static_cast<double>(kGain),
                static_cast<double>(settled_command(kGain, kVelocityLimit, kAccel, kExit)),
                static_cast<double>(weak_gain), static_cast<double>(weak_command),
                static_cast<double>(weak_gain * kExit), static_cast<double>(kBoundary));
    EXPECT_NEAR(weak_gain * kExit, weak_command, 1.0e-4F);
    EXPECT_LT(weak_command, 0.75F * kVelocityLimit)
        << "below 2a/v the gain really does take the cruise away";
}
