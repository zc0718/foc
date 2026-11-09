#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/traits.hpp>



namespace {

constexpr float kPsi = 4.76e-3F;
constexpr float kDt = 5e-5F;

auto config(float rs, float gain_ratio = 0.3F) -> foc::core::NonlinearFluxObserver::Config {
    return foc::core::NonlinearFluxObserver::Config{rs, 4e-4F, kPsi, gain_ratio, 3.0F, 0.5F * kPsi};
}

auto drive(foc::core::NonlinearFluxObserver& observer, int steps, float v_alpha, float v_beta,
           float i_alpha = 0.0F) -> float {
    const foc::core::Ab<float> v{v_alpha, v_beta};
    // A purely resistive current keeps the test about the flux, not about the machine.
    // 纯阻性的电流让这个测试只谈磁链，不谈机器。
    const foc::core::Ab<float> i{i_alpha, 0.0F};
    for (int k = 0; k < steps; ++k) {
        static_cast<void>(observer.update(v, i, kDt));
    }
    const foc::core::Ab<float> flux = observer.flux();
    return std::sqrt((flux.alpha * flux.alpha) + (flux.beta * flux.beta));
}

} // namespace

static_assert(foc::core::concepts::is_observer_v<foc::core::NonlinearFluxObserver>,
              "the nonlinear observer must satisfy the observer contract");
static_assert(foc::core::concepts::has_seed_flux_v<foc::core::NonlinearFluxObserver>,
              "the motor seeds it during start-up, like the open-loop one");
static_assert(foc::core::concepts::has_apply_params_v<foc::core::NonlinearFluxObserver>,
              "identification has to be able to reach it");

TEST(NonlinearFluxObserver, TheCorrectionPullsTheFluxMagnitudeBackToTheNameplate) {
    // The whole reason for the extra term. Start the estimate 40 % too large and let it run with a voltage
    // that would keep it at that size; the open-loop observer would sit there forever, this one walks back.
    // 那一项存在的全部理由。把估计初值放到大 40%，再用一个"维持这个大小"的电压跑；开环观测器会永远停在那里，
    // 这个会走回来。
    foc::core::NonlinearFluxObserver observer(config(0.35F));
    observer.seed_flux(foc::core::Ab<float>{1.4F * kPsi, 0.0F});

    // Two seconds, because the correction is quadratic near the target: it is fastest when the error is
    // largest and vanishes as the circle is reached, so the last of it is slow by construction.
    // 两秒，因为修正在目标附近是二次的：误差最大时最快、到达圆周时归零，所以最后那一点按构造就很慢。
    const float magnitude = drive(observer, 40000, 0.0F, 0.0F);
    EXPECT_NEAR(1.0F, magnitude / kPsi, 0.10F) << "the magnitude has to come home";

    // And from below: seeded too small, it grows back.
    // 从下方也一样：初值太小，它会长回来。
    foc::core::NonlinearFluxObserver small(config(0.35F));
    small.seed_flux(foc::core::Ab<float>{0.6F * kPsi, 0.0F});
    EXPECT_NEAR(1.0F, drive(small, 40000, 0.0F, 0.0F) / kPsi, 0.10F);
}

TEST(NonlinearFluxObserver, AnUnseededObserverStaysAtZero) {
    // The correction is proportional to the flux vector, so at zero it has no direction to push in. Same
    // start-up rule as the open-loop observer, and the reason the motor seeds both from the nameplate.
    // 修正正比于磁链矢量，因此为零时它没有可推的方向。与开环观测器同一条起动规则，也正是电机按铭牌给两者
    // 播种的原因。
    foc::core::NonlinearFluxObserver observer(config(0.35F));
    // The voltage is exactly `Rs * i`, so the integrand is zero and the only thing acting is the correction --
    // which, at zero flux, has no direction to push in.
    // 电压正好等于 `Rs·i`，被积函数为零，唯一在起作用的是修正——而在零磁链处它没有可推的方向。
    const float i_alpha = 0.5F;
    EXPECT_FLOAT_EQ(0.0F, drive(observer, 2000, 0.35F * i_alpha, 0.0F, i_alpha));
    EXPECT_FALSE(observer.locked());
}

TEST(NonlinearFluxObserver, RejectsEachUnusableConfiguration) {
    // The gain is a *fraction* of the estimated speed with a floor, so both halves have to be positive: a
    // negative ratio would push the flux away from the nameplate instead of towards it, which looks like a
    // diverging observer rather than a typo.
    // 增益是估计转速的一个 **比例** 加一个下限，因此两者都必须为正：负比例会把磁链推离铭牌而不是推向它，
    // 看起来像观测器发散，而不像一个笔误。
    for (int field = 0; field < 6; ++field) {
        foc::core::NonlinearFluxObserver::Config bad = config(0.35F);
        switch (field) {
        case 0: bad.rs = -1.0F; break;
        case 1: bad.l = 0.0F; break;
        case 2: bad.psi_f = 0.0F; break;
        case 3: bad.gain_ratio = 0.0F; break;
        case 4: bad.min_gain_rad_s = 0.0F; break;
        default: bad.min_flux_wb = 0.0F; break;
        }
        foc::core::NonlinearFluxObserver observer(bad);
        EXPECT_EQ(foc::ErrorCode::Param, observer.status()) << "field " << field;
        EXPECT_FLOAT_EQ(0.0F, observer.update(foc::core::Ab<float>{1.0F, 0.0F}, foc::core::Ab<float>{1.0F, 0.0F}, kDt)
                                  .theta_e);
    }

    foc::core::NonlinearFluxObserver observer(config(0.35F));
    ASSERT_EQ(foc::ErrorCode::Ok, observer.status());
    static_cast<void>(observer.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, 0.0F));
    EXPECT_EQ(foc::ErrorCode::Param, observer.status());

    observer.reset();
    EXPECT_EQ(foc::ErrorCode::Ok, observer.status());
    EXPECT_FLOAT_EQ(0.0F, observer.state().theta_e);
}

TEST(NonlinearFluxObserver, TakingNewParametersDoesNotResetTheEstimate) {
    // Unlike the open-loop observer, this one does not need a re-seed after a parameter change: the
    // correction keeps working whatever the integrator already accumulated. What it must not do is throw
    // the accumulated flux away.
    // 与开环观测器不同，这一个在参数变更后不需要重新播种：无论积分器已经积出什么，修正都继续工作。它不该做的
    // 是把已积出的磁链丢掉。
    foc::core::NonlinearFluxObserver observer(config(0.35F));
    observer.seed_flux(foc::core::Ab<float>{kPsi, 0.0F});
    static_cast<void>(drive(observer, 100, 0.1F, 0.0F));
    const foc::core::Ab<float> before = observer.flux();

    observer.apply_params(foc::core::MachineParams{0.50F, 2e-4F, 4e-4F, kPsi});
    EXPECT_FLOAT_EQ(before.alpha, observer.flux().alpha);
    EXPECT_FLOAT_EQ(before.beta, observer.flux().beta);
    EXPECT_EQ(foc::ErrorCode::Ok, observer.status());

    // An unusable update is rejected the same way as a bad construction.
    // 不可用的更新与错误的构造一样被拒绝。
    observer.apply_params(foc::core::MachineParams{0.50F, 2e-4F, 4e-4F, 0.0F});
    EXPECT_EQ(foc::ErrorCode::Param, observer.status());
    observer.apply_params(foc::core::MachineParams{0.50F, 2e-4F, 4e-4F, kPsi});
    EXPECT_EQ(foc::ErrorCode::Ok, observer.status());
}


TEST(NonlinearFluxObserver, TheInductiveDropIsSubtractedWhenTheCurrentChanges) {
    // `d(psi)/dt = v - Rs*i - L*di/dt`, and the last term is taken from the current *difference*. Every
    // other case in this file feeds a constant current, so `i - i_prev` is identically zero and the term
    // never acts: the mutation run changed its subtraction into an addition and no case noticed.
    // 这一项是 `d(psi)/dt = v - Rs*i - L*di/dt` 的末项，取自电流 **差分** 。本文件其余用例喂的都是恒定电流，
    // 于是 `i - i_prev` 恒为零、该项从不生效：变异测试把它的减法改成加法，没有一个用例察觉。
    //
    // Two properties of this observer decide whether the term is observable at all, both measured while
    // writing this case: (1) the flux must be seeded *along the same direction* as the current steps, or the
    // angle jumps on the first real sample and the radial gain follows that fake speed; (2) the steps must be
    // proportional, so the direction never changes and the radial correction stays negligible.
    // 观测器的两个性质决定该项是否可观测，都是写这条用例时实测到的：(1) 磁链初值必须与电流步进 **同向** ，
    // 否则第一个真实采样就会造成角度跳变、径向增益跟着那个假转速走；(2) 步子必须等比例，方向不变，
    // 径向修正才可忽略。
    const float step_alpha = 0.05F;
    const float step_beta = 0.025F;
    const float l = 4e-4F; // config()'s inductance

    foc::core::NonlinearFluxObserver observer(config(0.0F)); // Rs = 0: only the inductive drop acts
    const foc::core::Ab<float> seed{-1.0e-4F, -5.0e-5F}; // same direction as -(di): no angle step
    observer.seed_flux(seed);

    // The first update primes the previous sample instead of differentiating against a zero it never saw.
    // 第一次 update 只喂入上一拍采样，不与"从未发生过"的零值做差分。
    static_cast<void>(observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, kDt));
    // The radial correction acts on every update, so "unchanged" is up to that correction, not bit-exact.
    // 径向修正每次 update 都会生效，所以"未变"是指在该修正的量级内，而不是逐位相等。
    EXPECT_NEAR(seed.alpha, observer.flux().alpha, 1e-7F);
    EXPECT_FLOAT_EQ(0.0F, observer.state().omega_e) << "a seeded angle with no rotation is not a speed";

    static_cast<void>(observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{step_alpha, step_beta}, kDt));
    EXPECT_NEAR(seed.alpha - (l * step_alpha), observer.flux().alpha, 1e-7F);
    EXPECT_NEAR(seed.beta - (l * step_beta), observer.flux().beta, 1e-7F);

    // A second step *from a non-zero sample*. With `i_prev` at zero the two readings of the difference --
    // `i - i_prev` and `i + i_prev` -- are the same number, so only this one can tell them apart.
    // 从非零采样再走一步。`i_prev` 为零时 `i - i_prev` 与 `i + i_prev` 是同一个数，只有这一步能区分二者。
    static_cast<void>(
        observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{2.0F * step_alpha, 2.0F * step_beta}, kDt));
    EXPECT_NEAR(seed.alpha - (2.0F * l * step_alpha), observer.flux().alpha, 1e-7F);
    EXPECT_NEAR(seed.beta - (2.0F * l * step_beta), observer.flux().beta, 1e-7F);
}


TEST(NonlinearFluxObserver, TheResistiveDropIsSubtractedToo) {
    // The other half of `d(psi)/dt = v - Rs*i - L*di/dt`, and it was invisible for a different reason:
    // the case that does carry a current sets `v = Rs * i`, so the resistive drop cancels exactly and `v - Rs*i`
    // and `v + Rs*i` give the same zero. Meanwhile the inductive case above uses Rs = 0 to isolate its own term.
    // Between them, no case ever had Rs and current at once -- and the subtraction at that site survived.
    // 同一个式子的另一半，而它不可见的原因不同：唯一带电流的用例让 `v = Rs * i`，电阻压降正好抵消，
    // `v - Rs*i` 与 `v + Rs*i` 都给出零；而上面那个感应项用例用 Rs = 0 来隔离自己。两者之间，没有任何用例
    // 同时具备 Rs 与电流——该处的减法因此存活。
    foc::core::NonlinearFluxObserver observer(config(0.35F));
    observer.seed_flux(foc::core::Ab<float>{});
    static_cast<void>(observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, kDt)); // prime

    static_cast<void>(observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{1.0F, 0.5F}, kDt));

    // alpha: (0 - 0.35 * 1.0) * 5e-5 - 4e-4 * 1.0 = -1.75e-5 - 4.0e-4
    // beta : (0 - 0.35 * 0.5) * 5e-5 - 4e-4 * 0.5 = -8.75e-6 - 2.0e-4
    const foc::core::Ab<float> flux = observer.flux();
    EXPECT_NEAR(-4.175e-4F, flux.alpha, 1e-7F);
    EXPECT_NEAR(-2.0875e-4F, flux.beta, 1e-7F);
}


TEST(NonlinearFluxObserver, TheAngleIsWrappedIntoZeroToTwoPi) {
    // `wrap_two_pi` promises [0, 2pi), so an angle of exactly zero must stay zero: the loop that raises a
    // negative angle is `< 0`, not `<= 0`. Flipping it to `<=` sends a zero angle to 2pi, which is outside
    // the promised range -- and no case in this file ever had an estimate pointing along +alpha, which is the
    // only way atan2 returns exactly zero.
    // `wrap_two_pi` 承诺 [0, 2pi)，因此恰好为零的角度必须保持为零：抬升负角的循环是 `< 0` 而非 `<= 0`。
    // 改成 `<=` 会把零角送成 2pi，越过承诺的范围；而本文件从未有过的估计恰好指向 +alpha 的情形，
    // 正是 atan2 唯一可能返回精确零的路径。
    foc::core::NonlinearFluxObserver observer(config(0.35F));
    observer.seed_flux(foc::core::Ab<float>{kPsi, 0.0F});
    static_cast<void>(observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, kDt));

    EXPECT_FLOAT_EQ(0.0F, observer.state().theta_e);
    EXPECT_LT(observer.state().theta_e, 6.2832F); // inside [0, 2pi), not on its upper end
}



TEST(NonlinearFluxObserver, LockedNeedsTheFluxToBeAboveTheFloor) {
    // `locked()` is what a supervisor reads before believing the angle, and it is the only consumer of the
    // estimated magnitude. No case asserted it, so `alpha*alpha + beta*beta` could become a subtraction, or
    // both products could become divisions (magnitude sqrt(2) regardless of the flux), and everything passed.
    // `locked()` 是监管者在采信角度之前读的量，也是估计幅值的唯一消费者。此前没有任何用例断言它，
    // 于是 `alpha*alpha + beta*beta` 可以变成减法、两个乘法可以变成除法（幅值恒为 sqrt(2)）而全部通过。
    // Below the floor: the flux is genuinely too small to mean anything.
    {
        foc::core::NonlinearFluxObserver tiny(config(0.0F));
        tiny.seed_flux(foc::core::Ab<float>{});
        static_cast<void>(tiny.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, kDt));
        static_cast<void>(tiny.update(foc::core::Ab<float>{}, foc::core::Ab<float>{1.0F, 0.5F}, kDt));
        EXPECT_LT(std::sqrt((4.0e-4F * 4.0e-4F) + (2.0e-4F * 2.0e-4F)), 0.5F * kPsi);
        EXPECT_FALSE(tiny.locked());
    }

    // On the circle at 45 degrees: alpha^2 - beta^2 == 0 there, so a sign error in the magnitude would call
    // this unlocked, and a wrong radial term would never bring it back to the nameplate magnitude.
    // 在 45° 的圆周上：那里 alpha^2 - beta^2 == 0，幅值里的符号错误会把它判成未锁定，而径向项若有错
    // 就永远回不到铭牌幅值。
    {
        const float diagonal = kPsi / std::sqrt(2.0F);
        foc::core::NonlinearFluxObserver observer(config(0.35F));
        observer.seed_flux(foc::core::Ab<float>{diagonal, diagonal});
        const float magnitude = drive(observer, 20000, 0.0F, 0.0F);
        EXPECT_NEAR(1.0F, magnitude / kPsi, 0.10F) << "a 45-degree seed has to come home too";
        EXPECT_TRUE(observer.locked());
    }

    // Seeded along +beta, so the *beta* radial term is the only one that can do the work. Every other case
    // is seeded along +alpha, where that term is multiplied by zero and cannot be wrong.
    // 沿 +beta 播种，于是 **beta** 的径向项是唯一起作用的项。其余用例都沿 +alpha 播种，那里 beta 项乘以零，
    // 错也看不出来。
    {
        foc::core::NonlinearFluxObserver observer(config(0.35F));
        observer.seed_flux(foc::core::Ab<float>{0.0F, 1.4F * kPsi});
        const foc::core::Ab<float> flux_after = observer.flux();
        EXPECT_GT(flux_after.beta, 0.0F);
        const float magnitude = drive(observer, 20000, 0.0F, 0.0F);
        EXPECT_NEAR(1.0F, magnitude / kPsi, 0.10F) << "a beta-axis seed has to come home too";
    }
}



TEST(NonlinearFluxObserver, TheEstimatedSpeedComesFromTheAngleStep) {
    // `omega_e = wrap_pi(theta - theta_prev) / dt`. Nothing asserted the estimate, so the subtraction could
    // become an addition (which is invisible while theta_prev is zero) and the division by dt could become a
    // multiplication (a factor of dt^2). Here the estimate starts at a non-zero angle and advances by a known
    // increment, so both are observable.
    // `omega_e = wrap_pi(theta - theta_prev) / dt`。此前没有任何用例断言这个估计，于是减法可以变成加法
    // （在 theta_prev 为零时不可见）、除以 dt 可以变成乘以 dt（差 dt² 倍）。这里从非零角起步、按已知增量
    // 前进，两者都变得可观测。
    const float omega = 100.0F;
    const float theta0 = 0.5F;
    foc::core::NonlinearFluxObserver observer(config(0.0F));
    observer.seed_flux(foc::core::Ab<float>{kPsi * std::cos(theta0), kPsi * std::sin(theta0)});

    static_cast<void>(observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, kDt)); // prime
    EXPECT_FLOAT_EQ(theta0, observer.state().theta_e);

    // Perpendicular voltage of magnitude omega*psi_f rotates the estimate by omega*dt in one sample.
    const foc::core::Ab<float> v{-omega * kPsi * std::sin(theta0), omega * kPsi * std::cos(theta0)};
    static_cast<void>(observer.update(v, foc::core::Ab<float>{}, kDt));

    EXPECT_NEAR(omega, observer.state().omega_e, 1.0F);
}

