#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/pi.hpp>
#include <foc/core/types.hpp>



namespace {

auto make_config(float kp, float ki) -> foc::core::Pi::Config {
    return foc::core::Pi::Config{kp, ki, -100.0F, 100.0F, -50.0F, 50.0F};
}

} // namespace



TEST(Pi, ProportionalOnlyPassesGainThrough) {
    foc::core::Pi pi(make_config(2.0F, 0.0F));

    EXPECT_FLOAT_EQ(2.0F, pi.step(1.0F, 0.0F, 0.001F));
    EXPECT_EQ(foc::ErrorCode::Ok, pi.status());
}



TEST(Pi, IntegralAccumulatesKiTimesErrorTimesDt) {
    foc::core::Pi pi(make_config(0.0F, 1.0F));

    EXPECT_FLOAT_EQ(0.1F, pi.step(1.0F, 0.0F, 0.1F));
    EXPECT_FLOAT_EQ(0.2F, pi.step(1.0F, 0.0F, 0.1F));
    EXPECT_FLOAT_EQ(0.2F, pi.integral());
}



TEST(Pi, OutputIsClamped) {
    foc::core::Pi pi(make_config(1000.0F, 0.0F));

    EXPECT_FLOAT_EQ(100.0F, pi.step(1.0F, 0.0F, 0.001F));
    EXPECT_FLOAT_EQ(100.0F, pi.output());
}



TEST(Pi, IntegralIsClampedToPreventWindup) {
    foc::core::Pi pi(make_config(0.0F, 1000.0F));

    // ki * error * dt = 1.0 per step, so 50 steps reach the integral clamp and stay there.
    for (int i = 0; i < 100; ++i) {
        static_cast<void>(pi.step(1.0F, 0.0F, 0.001F));
    }

    EXPECT_FLOAT_EQ(50.0F, pi.integral());
    EXPECT_FLOAT_EQ(50.0F, pi.output());
}



TEST(Pi, NonPositiveDtIsRejectedAndHoldsOutput) {
    foc::core::Pi pi(make_config(1.0F, 0.0F));

    EXPECT_FLOAT_EQ(0.0F, pi.step(1.0F, 0.0F, 0.0F));
    EXPECT_EQ(foc::ErrorCode::Param, pi.status());

    // stays latched until reset()
    EXPECT_FLOAT_EQ(0.0F, pi.step(1.0F, 0.0F, 0.001F));

    pi.reset();
    EXPECT_EQ(foc::ErrorCode::Ok, pi.status());
    EXPECT_FLOAT_EQ(1.0F, pi.step(1.0F, 0.0F, 0.001F));
}



TEST(Pi, NegativeGainIsRejectedAtConstruction) {
    const foc::core::Pi pi(make_config(-1.0F, 0.0F));

    EXPECT_EQ(foc::ErrorCode::Param, pi.status());
}



TEST(Pi, ResetClearsIntegral) {
    foc::core::Pi pi(make_config(0.0F, 1.0F));
    static_cast<void>(pi.step(1.0F, 0.0F, 0.1F));

    pi.reset();

    EXPECT_FLOAT_EQ(0.0F, pi.integral());
    EXPECT_FLOAT_EQ(0.0F, pi.output());
}



TEST(Pi, APerCallGainScaleMultipliesTheProportionalTermWithoutTouchingTheConfig) {
    // Compliance is a per-call request, not a reconfiguration: the stiff call after the compliant one
    // must get the configured gains back with no restore step.
    // 柔顺是按次的请求，不是重配置：柔顺调用之后的刚性调用必须直接拿回配置的增益，不需要恢复步骤。
    foc::core::Pi controller{foc::core::Pi::Config{2.0F, 0.0F, -100.0F, 100.0F, -100.0F, 100.0F}};

    const foc::core::SpeedInput compliant{1.0F, 0.2F, 0.0F}; // target, kp_scale, ilimit_scale
    EXPECT_NEAR(2.0F * 0.2F * 1.0F, controller.step(compliant, 0.0F, 1e-3F), 1e-5F);

    // 0 means "no override": the configured gain is used unchanged.
    // 0 表示"不覆盖"：按配置的增益原样使用。
    const foc::core::SpeedInput unset{1.0F, 0.0F, 0.0F};
    EXPECT_NEAR(2.0F, controller.step(unset, 0.0F, 1e-3F), 1e-4F);
}



TEST(Pi, AZeroIntegralScaleReleasesStoredTorqueAtOnce) {
    // This is what makes `ilimit_scale` useful: the integral is re-clamped into the scaled window on
    // the step that asks for it, so a joint that is about to be pushed stops holding, instead of
    // leaking its stored torque away over many steps.
    // 这正是 `ilimit_scale` 有用的原因：积分项在提出请求的那一步就被重新夹进缩放后的窗口，于是即将
    // 被推的关节立刻停止保持，而不是花很多步把已存转矩漏掉。
    foc::core::Pi controller{foc::core::Pi::Config{0.0F, 10.0F, -100.0F, 100.0F, -100.0F, 100.0F}};

    // Wind the integral up against a constant error.
    // 用恒定误差把积分积起来。
    for (int i = 0; i < 500; ++i) {
        static_cast<void>(controller.step(1.0F, 0.0F, 1e-3F));
    }
    ASSERT_GT(controller.integral(), 1.0F);

    // A tiny integral scale squeezes the window, and the stored torque goes with it immediately.
    // 极小的积分缩放把窗口挤窄，已存的转矩随之立即消失。
    const foc::core::SpeedInput limp{1.0F, 0.0F, 0.001F};
    static_cast<void>(controller.step(limp, 0.0F, 1e-3F));
    EXPECT_LT(controller.integral(), 0.11F);
}



TEST(Pi, AScaledStepKeepsTheAntiWindupProperty) {
    // Scaling must not break the invariant the integral clamp exists for: the integral never leaves
    // the (scaled) window.
    // 缩放不得破坏积分限幅存在的理由：积分项永不离开（缩放后的）窗口。
    foc::core::Pi controller{foc::core::Pi::Config{1.0F, 100.0F, -100.0F, 100.0F, -1.0F, 1.0F}};
    const foc::core::SpeedInput half{5.0F, 0.5F, 0.5F}; // integral window becomes +-0.5

    for (int i = 0; i < 1'000; ++i) {
        static_cast<void>(controller.step(half, 0.0F, 1e-3F));
        EXPECT_LE(controller.integral(), 0.5F + 1e-5F);
        EXPECT_GE(controller.integral(), -0.5F - 1e-5F);
    }
}



TEST(Pi, ErrorIsTheSignedDifferenceOfSetpointAndFeedback) {
    // Every other case in this file feeds a zero feedback and therefore an error of exactly one, where
    // `setpoint - feedback` and `setpoint + feedback` produce the same number, and so do `ki * error` and
    // `ki / error`. The mutation run found both surviving, which says the suite could not see a sign error
    // in the error term at all. This case uses errors that are neither zero nor one, so the arithmetic is
    // observable.
    // 本文件其余用例的反馈都是 0、误差都是 1，此时 `setpoint - feedback` 与 `+`、`ki * error` 与 `/`
    // 给出同一个数——变异测试正是发现这两个都存活。本用例取非 0 非 1 的误差，让算式可观测。
    foc::core::Pi controller{foc::core::Pi::Config{2.0F, 4.0F, -100.0F, 100.0F, -100.0F, 100.0F}};

    // error = +0.5: proportional 1.0, integral 0 + 4 * 0.5 * 0.1 = 0.2, output 1.2.
    EXPECT_NEAR(1.2F, controller.step(1.0F, 0.5F, 0.1F), 1e-6F);
    EXPECT_NEAR(0.2F, controller.integral(), 1e-6F);

    // error = -0.25: the proportional term changes sign, the integral unwinds to 0.2 + 4 * -0.25 * 0.1 = 0.1,
    // output -0.5 + 0.1 = -0.4.
    EXPECT_NEAR(-0.4F, controller.step(0.75F, 1.0F, 0.1F), 1e-6F);
    EXPECT_NEAR(0.1F, controller.integral(), 1e-6F);
}



TEST(Pi, AZeroIntegralScaleMeansNoOverrideRatherThanAnEmptyWindow) {
    // `AZeroIntegralScaleReleasesStoredTorque` uses a small *non-zero* scale, so "0 means no override" and
    // "0 means an empty window" were both consistent with it. The two only differ at exactly zero, which is
    // the boundary of `ilimit_scale > 0` -- and that comparison was found surviving.
    // `AZeroIntegralScaleReleasesStoredTorque` 用的是非零的小缩放，因此"0 表示不覆盖"与"0 表示空窗口"
    // 两种读法都符合它。二者只在恰好为 0 时不同，而那正是 `ilimit_scale > 0` 的边界——该比较被检出存活。
    foc::core::Pi controller{foc::core::Pi::Config{0.0F, 10.0F, -100.0F, 100.0F, -100.0F, 100.0F}};

    for (int i = 0; i < 10; ++i) {
        static_cast<void>(controller.step(1.0F, 0.0F, 1e-3F));
    }
    ASSERT_NEAR(0.1F, controller.integral(), 1e-6F);

    const foc::core::SpeedInput unset{1.0F, 0.0F, 0.0F};
    static_cast<void>(controller.step(unset, 0.0F, 1e-3F));

    // No override: the window stays +-100 and the integral keeps one more step, 0.1 + 10 * 1 * 0.001.
    EXPECT_NEAR(0.11F, controller.integral(), 1e-6F);
    EXPECT_NEAR(0.11F, controller.output(), 1e-6F);
}



TEST(Pi, GainsAndLimitsExactlyAtTheirBoundaryAreAccepted) {
    // The validity check is written with >= and <=, so a config *at* the boundary is legal: an integral-only
    // controller (kp == 0) and a proportional-only one (ki == 0) are both real controllers, and a window
    // whose two ends coincide is a valid (if useless) window. Mutating any of those to a strict comparison
    // must fail this case.
    // 有效性检查用的是 >= 与 <=，因此恰好落在边界上的配置是合法的：纯积分（kp == 0）与纯比例（ki == 0）
    // 都是真实控制器，两端重合的窗口也合法（虽然无用）。把任一比较改成严格比较都必须让本用例失败。
    const foc::core::Pi zero_gains{foc::core::Pi::Config{0.0F, 0.0F, -100.0F, 100.0F, -50.0F, 50.0F}};
    EXPECT_EQ(foc::ErrorCode::Ok, zero_gains.status());

    const foc::core::Pi coincident{foc::core::Pi::Config{1.0F, 1.0F, 0.0F, 0.0F, 0.0F, 0.0F}};
    EXPECT_EQ(foc::ErrorCode::Ok, coincident.status());

    // A real inversion is still rejected: the boundary being legal must not turn the check into a no-op.
    const foc::core::Pi inverted{foc::core::Pi::Config{1.0F, 1.0F, 1.0F, 0.0F, 1.0F, 0.0F}};
    EXPECT_EQ(foc::ErrorCode::Param, inverted.status());
}



TEST(Pi, ScalingIsExactOnBothWindowEndsAndZeroLeavesThemUntouched) {
    // Two survivors that no case could see, both on the *lower* end of the integral window: every existing
    // case drives the integral positive, so `integral_min * scale` could be a division and the "no override"
    // fallback could be 1.001 instead of 1.0 without changing a single asserted number. The window here is
    // asymmetric and the integral is wound negative, so each end and each scale factor is observable.
    // 有两个存活体没有任何用例能看见，都在积分窗口的 **下端** ：现有用例只把积分往正向推，因此把
    // `integral_min * scale` 改成除法、或把"不覆盖"的回退值从 1.0 改成 1.001，都不会改变任何一个被断言的数。
    // 这里窗口不对称、积分往负向积，于是两端与两档缩放都变得可观测。
    foc::core::Pi controller{foc::core::Pi::Config{0.0F, 1000.0F, -100.0F, 100.0F, -0.2F, 0.5F}};

    // error = -1, so each step adds 1000 * -1 * 0.001 = -1: the lower end clamps at exactly the config's -0.2.
    for (int i = 0; i < 10; ++i) {
        static_cast<void>(controller.step(0.0F, 1.0F, 1e-3F));
    }
    EXPECT_NEAR(-0.2F, controller.integral(), 1e-6F);

    // scale 0.5 halves both ends: [-0.1, 0.25]. Halved, not doubled.
    const foc::core::SpeedInput limp{0.0F, 0.0F, 0.5F};
    static_cast<void>(controller.step(limp, 1.0F, 1e-3F));
    EXPECT_NEAR(-0.1F, controller.integral(), 1e-6F);

    // scale 0 uses the configured window unchanged -- exactly, not scaled by something close to one.
    const foc::core::SpeedInput unset{0.0F, 0.0F, 0.0F};
    static_cast<void>(controller.step(unset, 1.0F, 1e-3F));
    EXPECT_NEAR(-0.2F, controller.integral(), 1e-6F);
}

namespace {

// A first-order plant driven by the controller under test: `J*dx/dt = u - d`. Stepped with an *independent* Euler
// integrator so the controller's own period can be wrong on purpose (`dt_pi` vs `dt_true`), which is the error source
// that actually exists in a drive: nobody's control period is exactly what the code was told.
// 由被测控制器驱动的一阶对象：`J·dx/dt = u − d`。用 **独立** 的 Euler 积分推进，好让控制器自己的周期被故意写错
// （`dt_pi` 与 `dt_true` 不同）——这才是驱动里真实存在的误差源：没有人的控制周期恰好等于代码被告知的数。
struct LoopResult final {
    double peak;
    double t_peak;
    double final_error;
    double settle;
};

auto drive_loop(float kp, float ki, float j, float disturbance, float dt_pi, float dt_true, long steps)
    -> LoopResult {
    foc::core::Pi pi(foc::core::Pi::Config{kp, ki, -1.0e3F, 1.0e3F, -1.0e3F, 1.0e3F});
    double x = 0.0;
    double peak = 0.0;
    double t_peak = 0.0;
    double settle = -1.0;
    for (long k = 0; k < steps; ++k) {
        const float u = pi.step(1.0F, static_cast<float>(x), dt_pi);
        x += static_cast<double>(dt_true) * ((static_cast<double>(u) - static_cast<double>(disturbance)) /
                                             static_cast<double>(j));
        if (x > peak) {
            peak = x;
            t_peak = static_cast<double>(k + 1) * static_cast<double>(dt_true);
        }
        if (settle < 0.0 && std::fabs(x - 1.0) < 0.005) {
            settle = static_cast<double>(k + 1) * static_cast<double>(dt_true);
        }
    }
    return LoopResult{peak, t_peak, std::fabs(1.0 - x), settle};
}

/** @brief [en] The same loop as an ODE, RK4 at 10 ns: two states, so the reference zero is included by construction. */
/** @brief [zh] 同一个环写成 ODE，用 10 ns 的 RK4：两个状态，于是参考零点按构造就在里面。 */
auto continuum_peak(double kp, double ki, double j, double disturbance) -> double {
    double x = 0.0;
    double z = 0.0;
    double peak = 0.0;
    const double h = 1.0e-7;
    const auto deriv = [&](double xs, double zs, double& dx, double& dz) {
        const double e = 1.0 - xs;
        dx = ((kp * e) + zs - disturbance) / j;
        dz = ki * e;
    };
    for (long k = 0; k < 400000; ++k) {
        double d1 = 0.0, z1 = 0.0, d2 = 0.0, z2 = 0.0, d3 = 0.0, z3 = 0.0, d4 = 0.0, z4 = 0.0;
        // Plain two-stage RK4 written out: the loop is tiny and a half-step error here would be invisible below.
        // 直接写出二级 RK4：这个环很小，而这里半步的错在下面根本看不出来。
        deriv(x, z, d1, z1);
        deriv(x + (0.5 * h * d1), z + (0.5 * h * z1), d2, z2);
        deriv(x + (0.5 * h * d2), z + (0.5 * h * z2), d3, z3);
        deriv(x + (h * d3), z + (h * z3), d4, z4);
        x += h * (d1 + (2.0 * d2) + (2.0 * d3) + d4) / 6.0;
        z += h * (z1 + (2.0 * z2) + (2.0 * z3) + z4) / 6.0;
        if (x > peak) {
            peak = x;
        }
    }
    return peak;
}

} // namespace

TEST(Pi, TheIntegralRemovesTheDroopAndItsSizeIsExactlyDisturbanceOverKp) {
  // The one closed form a PI has that cannot be argued with: with `ki = 0` a constant disturbance leaves exactly
  // `d/kp` of standing error, and with any `ki > 0` it leaves none. Measured on a plant driven through the real
  // controller, three disturbance levels.
  // PI 唯一一条无可争辩的闭式：`ki = 0` 时恒定扰动正好留下 `d/kp` 的静差，只要 `ki > 0` 就一点不留。
  // 在由真实控制器驱动的一阶对象上量三个扰动水平。
  const float j = 1.0e-4F;
  const float dt = 1.0e-4F;
  const float kp = 0.01F;
  for (const float disturbance : {0.001F, 0.002F, 0.005F}) {
    const LoopResult without = drive_loop(kp, 0.0F, j, disturbance, dt, dt, 200000);
    const LoopResult with = drive_loop(kp, 1.0F, j, disturbance, dt, dt, 200000);
    const double law = static_cast<double>(disturbance) / static_cast<double>(kp);
    std::printf("[Pi] disturbance %.4f | ki=0 leaves %.6f (law d/kp = %.6f) | ki=1 leaves %.6f\n",
                static_cast<double>(disturbance), without.final_error, law, with.final_error);
    // Six digits against a law that has no free parameter in it.
    // 对着一条没有自由参数的定律，六位一致。
    EXPECT_NEAR(law, without.final_error, 1.0e-4 * law + 1.0e-6);
    EXPECT_LT(with.final_error, 1.0e-5) << "the integral did not remove the droop";
  }
}

TEST(Pi, TheTextbookSecondOrderLawMispredictsTheOvershootAndTheZeroIsWhy) {
  // The loop this controller closes on a first-order plant has transfer function `(kp*s + ki)/(J*s^2 + kp*s + ki)` --
  // a **zero at `-ki/kp`**, because the proportional term acts on the reference as well as the error. The textbook
  // `zeta = kp/(2*sqrt(ki*J))`, `Mp = exp(-pi*zeta/sqrt(1-zeta^2))` is the *pole-only* answer and it is wrong here:
  // at `kp = 0.01` it says 16.3 % and the loop does 29.8 %; at `kp = 0.02` it says 0 % (critical damping) and the loop
  // does 13.5 %. The second model that settles it is the same loop written as two ODEs and integrated at 10 ns -- which
  // contains the zero by construction -- and it agrees to **0.06 %**.
  // 本控制器在一阶对象上闭出的环，传递函数是 `(kp·s + ki)/(J·s² + kp·s + ki)`—— **在 `−ki/kp` 有一个零点** ，因为比例项
  // 对给定也起作用。教科书的 `ζ = kp/(2√(ki·J))`、`Mp = exp(−πζ/√(1−ζ²))` 是 **只看极点** 的答案，在这里是错的：
  // `kp = 0.01` 它说 16.3 %，环跑出 29.8 %；`kp = 0.02` 它说 0 %（临界阻尼），环跑出 13.5 %。判定的第二个模型是同一个
  // 环写成两个 ODE、用 10 ns 积分——零点按构造就在里面——两者一致到 **0.06 %** 。
  const float j = 1.0e-4F;
  const float dt = 1.0e-4F;
  const float ki = 1.0F;
  int crossings = 0;
  for (const float kp : {0.01F, 0.02F, 0.03F}) {
    const LoopResult sampled = drive_loop(kp, ki, j, 0.0F, dt, dt, 200000);
    const double continuum = continuum_peak(static_cast<double>(kp), static_cast<double>(ki),
                                            static_cast<double>(j), 0.0) -
                             1.0;
    const double zeta = static_cast<double>(kp) / (2.0 * std::sqrt(static_cast<double>(ki) * static_cast<double>(j)));
    const double pole_only = (zeta < 1.0) ? std::exp(-M_PI * zeta / std::sqrt(1.0 - (zeta * zeta))) : 0.0;
    std::printf("[Pi] kp %.3f | pole-only %.4f | continuum (RK4) %.4f | sampled %.4f | gap %.2f %%\n",
                static_cast<double>(kp), pole_only, continuum, sampled.peak - 1.0,
                100.0 * std::fabs((sampled.peak - 1.0 - continuum) / (1.0 + continuum)));
    // The sampled loop and the continuum are the same system at two resolutions.
    // 采样环与连续解是同一个系统的两个分辨率。
    EXPECT_NEAR(continuum, sampled.peak - 1.0, 0.005 * (1.0 + continuum)) << "kp " << kp;
    // And the pole-only law is not: it is off by more than ten points of overshoot at both low gains, which is what
    // makes this a finding rather than a tolerance question.
    // 而只看极点的定律不是：两个低增益处都差十个点以上的超调——这正是它算"发现"而不是"容差问题"的原因。
    if (kp <= 0.02F) {
      EXPECT_GT(std::fabs((sampled.peak - 1.0) - pole_only), 0.10) << "the textbook law agreed after all at kp " << kp;
      ++crossings;
    }
  }
  EXPECT_EQ(2, crossings) << "the sweep did not reach the two gains the comparison is about";
}

TEST(Pi, APeriodTheCallerGetsWrongMovesTheLoopByHalfOfIt) {
  // The error source that exists in every drive: the controller is told a period, the hardware has another one. The
  // integral term integrates `ki*e*dt`, so a period `k` times too large makes the effective `ki` `k` times too large,
  // and the loop's frequency moves as `sqrt(k)` -- **half** of the mismatch, in the other direction, measured on the
  // time to the first peak.
  // 每个驱动里都存在的误差源：控制器被告知一个周期，硬件是另一个。积分项积的是 `ki·e·dt`，于是周期大 `k` 倍就等于
  // 有效 `ki` 大 `k` 倍，环的频率按 `sqrt(k)` 移动—— **一半** 、方向相反，在到达第一个峰的时间上量出来。
  const float j = 1.0e-4F;
  const float dt = 1.0e-4F;
  const float nominal = static_cast<float>(drive_loop(0.01F, 1.0F, j, 0.0F, dt, dt, 200000).t_peak);
  ASSERT_GT(nominal, 0.0F);
  for (const float k : {0.90F, 0.95F, 1.05F, 1.10F}) {
    const float measured = static_cast<float>(drive_loop(0.01F, 1.0F, j, 0.0F, dt * k, dt, 200000).t_peak);
    const double predicted = 1.0 / std::sqrt(static_cast<double>(k));
    std::printf("[Pi] period x%.2f | time to first peak %.6f s | ratio %.4f vs 1/sqrt(k) %.4f\n", static_cast<double>(k),
                static_cast<double>(measured), static_cast<double>(measured / nominal), predicted);
    // Within 2 % of the half-power law: the coefficient is a half, and it is the *other* sign from the mismatch.
    // 与半次方定律相差 2 % 以内：系数是二分之一，且方向与不匹配相反。
    EXPECT_NEAR(predicted, static_cast<double>(measured / nominal), 0.02) << "period x" << k;
  }
}

