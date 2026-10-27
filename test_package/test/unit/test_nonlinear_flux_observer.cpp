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
    // 增益是估计转速的一个**比例**加一个下限，因此两者都必须为正：负比例会把磁链推离铭牌而不是推向它，
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
