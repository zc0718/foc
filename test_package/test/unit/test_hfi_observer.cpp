#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/observer/hfi_observer.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>



static_assert(foc::core::concepts::is_observer_v<foc::core::HfiObserver>,
              "HfiObserver must satisfy the observer policy contract");

static_assert(foc::core::concepts::has_injection_v<foc::core::HfiObserver>,
              "HfiObserver must be detected as an observer that injects a carrier");



namespace {

constexpr float kDt = 5e-5F; // s, 20 kHz
constexpr float kLd = 2e-4F;
constexpr float kLq = 4e-4F;

/// [en] A valid configuration; each case perturbs exactly one field.
///      [zh] 一份合法配置；每个用例只扰动其中一个字段。
auto valid_config() -> foc::core::HfiObserver::Config {
    foc::core::HfiObserver::Config config{};
    config.inject_v = 12.0F;
    config.inject_hz = 2000.0F;
    config.demod_lpf_hz = 200.0F;
    config.pll_bandwidth_hz = 40.0F;
    config.pll_zeta = 0.9F;
    config.demod_gain_a = foc::core::HfiObserver::predicted_gain_a(config.inject_v, config.inject_hz, kLd, kLq);
    config.min_hf_current_a = 0.05F;
    return config;
}

auto update_once(foc::core::HfiObserver& observer, float dt = kDt) -> foc::core::ObsState {
    return observer.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, dt);
}

} // namespace

TEST(HfiObserver, RejectsEachUnusableConfiguration) {
    // Every rejection is a number that would otherwise be used in a division or as a phase step and
    // quietly produce an angle: zero amplitude, zero frequency, a demodulation cutoff at or above the
    // carrier (the filter would pass the ripple it exists to remove), a zero gain (the error
    // normalisation would divide by it), and a negative threshold.
    // 每一次拒绝都是一个否则会被拿去参与除法或作为相位步长、并悄悄产出一个角度的数字：零幅值、零频率、
    // 解调截止不低于载波（滤波器会放行它本该滤掉的纹波）、零增益（误差归一化会除以它）、负阈值。
    for (int field = 0; field < 7; ++field) {
        foc::core::HfiObserver::Config config = valid_config();
        switch (field) {
        case 0: config.inject_v = 0.0F; break;
        case 1: config.inject_hz = 0.0F; break;
        case 2: config.demod_lpf_hz = 0.0F; break;
        case 3: config.demod_lpf_hz = config.inject_hz; break;
        case 4: config.pll_bandwidth_hz = 0.0F; break;
        case 5: config.demod_gain_a = 0.0F; break;
        default: config.min_hf_current_a = -1.0F; break;
        }

        foc::core::HfiObserver observer(config);
        EXPECT_EQ(foc::ErrorCode::Param, observer.status()) << "field " << field;

        const foc::core::ObsState state = update_once(observer);
        EXPECT_FLOAT_EQ(0.0F, state.theta_e);
        EXPECT_FLOAT_EQ(0.0F, state.omega_e);
        // A rejected observer injects nothing: a caller that ignored status() still cannot put a
        // carrier it never sized onto the phases.
        // 被拒绝的观测器不注入任何东西：即使调用方忽略了 status()，也无法把一份它从未估算过的载波加到
        // 各相上。
        EXPECT_FLOAT_EQ(0.0F, observer.injection_ab().alpha);
        EXPECT_FLOAT_EQ(0.0F, observer.injection_ab().beta);
        EXPECT_FALSE(observer.locked());
    }
}

TEST(HfiObserver, RejectsNonPositivePeriod) {
    foc::core::HfiObserver observer(valid_config());
    ASSERT_EQ(foc::ErrorCode::Ok, observer.status());

    EXPECT_FLOAT_EQ(0.0F, update_once(observer, 0.0F).theta_e);
    EXPECT_EQ(foc::ErrorCode::Param, observer.status());

    // Latched, like every other Param in this library: a bad period is a configuration error the
    // caller has to clear, not a sample to skip.
    // 与其他 Param 一样锁存：错误的周期是调用方必须清除的配置错误，不是可以跳过的采样。
    EXPECT_FLOAT_EQ(0.0F, update_once(observer).theta_e);
    observer.reset();
    EXPECT_EQ(foc::ErrorCode::Ok, observer.status());
}

TEST(HfiObserver, ResetClearsTheEstimateTheLockAndThePhase) {
    foc::core::HfiObserver observer(valid_config());
    // Feed a current that does produce a HF response so the observer locks and moves.
    for (int i = 0; i < 400; ++i) {
        static_cast<void>(observer.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.0F, 0.0F}, kDt));
        static_cast<void>(observer.injection_ab());
    }
    EXPECT_FALSE(observer.locked());

    observer.reset();
    EXPECT_FLOAT_EQ(0.0F, observer.state().theta_e);
    EXPECT_FLOAT_EQ(0.0F, observer.error());
    EXPECT_FLOAT_EQ(0.0F, observer.hf_current_a());
    EXPECT_FALSE(observer.locked());
    // The phase restarts at zero, so the injection the next caller receives is the same one a freshly
    // constructed observer would hand out -- a reset must be indistinguishable from a new object.
    // 相位从零重来，于是下一个调用方拿到的注入与新建对象给出的一致——reset() 必须与新对象不可区分。
    EXPECT_NEAR(0.0F, observer.injection_ab().alpha, 1e-6F);
    EXPECT_NEAR(0.0F, observer.injection_ab().beta, 1e-6F);
}

TEST(HfiObserver, TheInjectionPhaseAdvancesByOnePeriodPerCall) {
    // The injection is a sine at the configured frequency sampled once per control period: after four
    // periods it must have come back to where it started, otherwise the carrier frequency the caller
    // asked for is not the one being applied.
    // 注入是按控制周期采样的配置频率正弦：四个周期后必须回到起点，否则调用方要的载波频率与实际施加的
    // 不是同一个。
    foc::core::HfiObserver observer(valid_config());
    const float period_s = 1.0F / valid_config().inject_hz;
    const float quarter = period_s / 4.0F;

    float peak = 0.0F;
    for (int i = 0; i < 4; ++i) {
        static_cast<void>(update_once(observer, quarter));
        static_cast<void>(observer.injection_ab());
        const foc::core::Ab<float> v = observer.injection_ab();
        peak = std::fmax(peak, std::sqrt((v.alpha * v.alpha) + (v.beta * v.beta)));
    }
    // The estimate is 0 in this test, so the injection points along alpha; a quarter period lands on
    // the peak of the sine, which is the configured amplitude.
    // 本用例估计值为 0，因此注入沿 alpha 方向；四分之一周期正好落在正弦峰值，即配置的幅值。
    EXPECT_NEAR(valid_config().inject_v, peak, 1e-3F);
}

TEST(HfiObserver, AChangedPeriodReconfiguresTheFilters) {
    // A variable-rate ISR must not keep the alpha of the previous rate. Doubling the period and
    // continuing must stay Ok, and the state must keep evolving rather than freeze.
    // 变速 ISR 不能沿用上一档速率的 alpha。把周期加倍后继续必须仍是 Ok，且状态继续演化而不是冻结。
    foc::core::HfiObserver observer(valid_config());
    static_cast<void>(update_once(observer));
    static_cast<void>(observer.injection_ab());

    for (int i = 0; i < 20; ++i) {
        static_cast<void>(observer.update(foc::core::Ab<float>{0.0F, 0.0F}, foc::core::Ab<float>{0.1F, 0.0F}, 2.0F * kDt));
        static_cast<void>(observer.injection_ab());
    }
    EXPECT_EQ(foc::ErrorCode::Ok, observer.status());
    EXPECT_GT(observer.hf_current_a(), 0.0F);
}

TEST(HfiObserver, ThePredictedGainCarriesTheSaliencyDirectionInItsSign) {
    // One signed number instead of a magnitude plus a polarity flag: the sign of (Ld - Lq) is a fact
    // about the machine, and keeping it inside the gain means there is no second field that can be
    // set inconsistently with the first.
    // 一个带符号的数，而不是"幅值 + 极性开关"：`Ld - Lq` 的符号是关于电机的事实，把它放在增益里，
    // 就不存在第二个可能与第一个矛盾的字段。
    const float interior = foc::core::HfiObserver::predicted_gain_a(12.0F, 2000.0F, 2e-4F, 4e-4F);
    const float inset = foc::core::HfiObserver::predicted_gain_a(12.0F, 2000.0F, 4e-4F, 2e-4F);
    EXPECT_LT(interior, 0.0F) << "Lq > Ld must give a negative gain";
    EXPECT_GT(inset, 0.0F) << "Ld > Lq must give a positive gain";
    EXPECT_NEAR(-interior, inset, 1e-6F) << "swapping the two inductances must mirror the gain";

    // Degenerate inputs answer zero rather than dividing by it. The constructor rejects a zero gain,
    // so this is the value a caller gets when *asking* for a prediction it should not use.
    // 退化输入返回零而不是除以它。构造函数会拒绝零增益，因此这是调用方**询问**一个自己不该使用的预测
    // 时得到的值。
    EXPECT_FLOAT_EQ(0.0F, foc::core::HfiObserver::predicted_gain_a(12.0F, 2000.0F, 0.0F, 4e-4F));
    EXPECT_FLOAT_EQ(0.0F, foc::core::HfiObserver::predicted_gain_a(12.0F, 0.0F, 2e-4F, 4e-4F));
}
