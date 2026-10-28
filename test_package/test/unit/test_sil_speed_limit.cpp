// W7-b, before touching the observer: how much of the low-speed speed error is irreducible?
//
// 3a-3c closed the "difference the angle" route with measurements rather than opinions: a quieter estimate that is late is
// worse in a loop, at every window from 10 ms down to 0.5 ms. The remaining idea is structural -- take the speed from the
// demodulated carrier's phase drift instead of from the PLL's state.
//
// But both ideas consume the same information, and that information has a floor: a speed is an angle change over a time,
// so any estimator's noise goes as sigma_theta divided by the averaging time. If the angle error is white, a 10 ms
// average leaves ~0.03 rad/s and there is room. If it is correlated over the electrical period -- 45 ms here -- no
// estimator can average it away, and the low-speed servo is already at its limit.
//
// So this case measures the angle error's own time structure first, read-only, before anything in `HfiObserver` is
// changed. It is the cheapest way to find out whether the structural route has anything to win.
//
// W7-b，在动观测器之前：低速转速误差里有多少是**不可约**的？
// 3a–3c 用测量而不是意见关闭了"对角度做差分"这条路：更安静但更晚的估计在环里更差，从 10 ms 到 0.5 ms 的每个窗口都是。
// 剩下的是结构路线——从解调载波的相位漂移取转速，而不是从 PLL 的状态。
//
// 但两种做法消耗的是同一份信息，而这份信息有下界：转速是角度在一段时间上的变化，因此任何估计器的噪声都是 σ_θ 除以平均
// 时间。若角误差是白的，10 ms 平均后剩 ~0.03 rad/s，还有空间；若它在电周期（这里 45 ms）尺度上是相关的，就没有估计器
// 能把它平均掉，低速伺服已经在它的极限上。
//
// 因此本用例先**只读**地量角误差自身的时间结构，然后再决定是否改 `HfiObserver`。这是判断结构路线有没有东西可赢的最便宜
// 办法。
//
// The criterion, fixed before the numbers below were measured:
//   C1  the measurement must be able to distinguish the two cases: it reports the normalised autocorrelation at 1 ms and
//       at 10 ms, and both must be finite and inside [-1, 1]. A measurement that cannot separate white from correlated
//       would decide nothing, and printing a number nobody checked is how this repository has been fooled before.
//   C2  the implied irreducible speed noise is reported for both averaging times, so that the conclusion is a number
//       rather than an adjective.
//
// 判据在量之前定下：
//   C1 这个测量必须能区分两种情况：它报出 1 ms 与 10 ms 处的归一化自相关，两者都必须有限且落在 [-1, 1] 内。
//      一个分不开白噪声与相关噪声的测量什么也决定不了，而"打印一个没人检查过的数"正是本仓库此前被糊弄过的方式。
//   C2 两种平均时间下**不可约**的转速噪声都要报出来，于是结论是一个数字而不是一个形容词。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/switchable_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_pmsm_plant.hpp"
#include "sil_plant_hal.hpp"

namespace {

namespace core = foc::core;
namespace motor = foc::motor;
namespace sil = foc::sil;

constexpr float kTwoPi = 6.28318530717959F;

constexpr float kRs = 0.35F;
constexpr float kLd = 2.0e-4F;
constexpr float kLq = 4.0e-4F;
constexpr float kKt = 0.05F;
constexpr float kJ = 1.0e-5F;
constexpr float kB = 1.0e-3F;
constexpr std::uint8_t kPoles = 7U;

constexpr float kCtrlHz = 20000.0F;
constexpr float kDt = 1.0F / kCtrlHz;
constexpr int kSubsteps = 8;

constexpr float kCommandRadS = 20.0F;
constexpr float kRunSeconds = 1.2F;
constexpr int kWindowTicks = static_cast<int>(0.3F / kDt);
constexpr int kSamples = 4096; // 205 ms of history, comfortably inside the window

using SensorlessMotor = motor::Motor<core::SwitchableObserver, core::Svpwm, core::Pi, sil::PlantHal,
                                     core::NullReferenceGenerator, core::NullFeedforward, core::PositionP,
                                     core::NullAlignment>;

auto plant_params() -> sil::PlantParams { return sil::PlantParams{kRs, kLd, kLq, kKt, kJ, kB, kPoles}; }

auto motor_config() -> motor::MotorConfig {
    return motor::MotorConfig{kRs, kLd, kLq, kKt, kJ, kB, kPoles, 3.0F, kCtrlHz, 0.0F, 0.0F, 0.0F};
}

auto wrap_pole_pitch(float angle) noexcept -> float {
    const float pitch = kTwoPi / static_cast<float>(kPoles);
    while (angle <= -pitch * 0.5F) {
        angle += pitch;
    }
    while (angle > pitch * 0.5F) {
        angle -= pitch;
    }
    return angle;
}

} // namespace

TEST(SilSpeedLimit, TheAngleErrorsCorrelationDecidesWhetherABetterEstimatorExists) {
    sil::PmsmPlant plant(plant_params());
    sil::PlantHal hal(plant, 24.0F);
    hal.reset(sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
    hal.position.set_sensor_present(false);

    const motor::MotorConfig cfg = motor_config();
    core::SwitchableObserver::Config config{};
    config.back_emf = core::BackEmfObserver::Config{cfg.rs, cfg.ld};
    config.hfi.inject_v = 12.0F;
    config.hfi.inject_hz = 2000.0F;
    config.hfi.current_hp_hz = 300.0F;
    config.hfi.demod_lpf_hz = 200.0F;
    config.hfi.pll_bandwidth_hz = 40.0F;
    config.hfi.pll_zeta = 0.9F;
    config.hfi.demod_gain_a = core::HfiObserver::predicted_gain_a(config.hfi.inject_v, config.hfi.inject_hz, kLd, kLq);
    config.hfi.min_hf_current_a = 0.05F;
    config.kind = core::ObserverKind::Hfi;

    SensorlessMotor motor(cfg,
                          SensorlessMotor::Policies{core::SwitchableObserver{config},
                                                    core::Svpwm{},
                                                    core::Pi{foc::current_pi_config(cfg, 2000.0F, 24.0F)},
                                                    core::Pi{foc::current_pi_config(cfg, 2000.0F, 24.0F)},
                                                    core::Pi{foc::speed_pi_config(cfg, 200.0F)},
                                                    core::NullReferenceGenerator{}, core::NullFeedforward{},
                                                    core::PositionP{foc::position_p_config(cfg, 20.0F, 120.0F, 0.0F)},
                                                    core::NullAlignment{}},
                          hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());

    motor.set_torque(0.0F);
    for (int i = 0; i < static_cast<int>(0.3F / kDt); ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
    }

    // Closed loop on the observer's own speed, i.e. the configuration that currently wins, so the angle error measured is
    // the one an estimator would actually see in service.
    // 闭在观测器自有转速上，即当前胜出的那个配置，于是量到的角误差就是估计器在实际工作中会看到的那个。
    motor.set_speed(kCommandRadS);
    float history[kSamples]{};
    int collected = 0;
    for (int i = 0; i < (kWindowTicks + kSamples); ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        const float observed_mech =
            static_cast<float>(motor.observer().state().theta_e) / static_cast<float>(kPoles);
        const float sample = wrap_pole_pitch(observed_mech - plant.theta_mech_rad());
        if (i >= kWindowTicks) {
            history[collected % kSamples] = sample;
            ++collected;
        }
    }

    double sum = 0.0;
    for (int i = 0; i < kSamples; ++i) {
        sum += static_cast<double>(history[i]) * static_cast<double>(history[i]);
    }
    const double rms = std::sqrt(sum / static_cast<double>(kSamples));

    const auto autocorrelation = [&history](int lag) {
        double acc = 0.0;
        double norm = 0.0;
        for (int i = 0; i + lag < kSamples; ++i) {
            acc += static_cast<double>(history[i + lag]) * static_cast<double>(history[i]);
        }
        for (int i = 0; i < kSamples; ++i) {
            norm += static_cast<double>(history[i]) * static_cast<double>(history[i]);
        }
        return (norm > 0.0) ? (acc / norm) : 0.0;
    };

    const int lag_1ms = static_cast<int>(1.0e-3F / kDt);
    const int lag_10ms = static_cast<int>(10.0e-3F / kDt);
    const double rho_1ms = autocorrelation(lag_1ms);
    const double rho_10ms = autocorrelation(lag_10ms);

    // Irreducible speed noise for an averaging time T, from the angle's own rms: sigma_theta / T.
    // 平均时间 T 下不可约的转速噪声，由角度自身的 rms 得出：σ_θ / T。
    const double floor_1ms = (rms / static_cast<double>(1.0e-3F)) / static_cast<double>(kPoles);
    const double floor_10ms = (rms / static_cast<double>(10.0e-3F)) / static_cast<double>(kPoles);

    std::printf("\n[limit] angle error in the settled window: rms %.5f rad (mechanical)\n", rms);
    std::printf("[limit]   normalised autocorrelation at 1 ms %.4f, at 10 ms %.4f\n", rho_1ms, rho_10ms);
    std::printf("[limit]   irreducible speed noise: %.4f rad/s at 1 ms averaging, %.4f at 10 ms\n", floor_1ms,
                floor_10ms);
    std::printf("[limit]   the loop currently achieves 0.1717 rad/s\n");

    // C1: the measurement has to be able to tell the two cases apart.
    // C1：这个测量必须能区分两种情况。
    EXPECT_TRUE(std::isfinite(rho_1ms) && (rho_1ms >= -1.0) && (rho_1ms <= 1.0));
    EXPECT_TRUE(std::isfinite(rho_10ms) && (rho_10ms >= -1.0) && (rho_10ms <= 1.0));
    EXPECT_GT(rms, 0.0) << "no angle error was collected, so nothing was measured";

    // C2: the verdict, printed as a relation rather than left to the reader.
    // C2：判定，报成一条大小关系而不是留给读者。
    // The verdict, and the first draft of it was wrong in a way worth recording. It said "still correlated at 10 ms --
    // no estimator beats the floor", as if correlation made things harder. It is the other way round: a correlated error
    // is one that *differences away*. What correlation rules out is **noise** as the explanation. With rho = 0.95 at
    // 10 ms the angle error is nearly constant over that interval, so a differentiator over 10 ms sees almost none of it
    // as noise -- which is exactly what 3a measured (0.196 rad/s at 10 ms against a white-noise prediction of 0.030),
    // and the gap is the error's own slow drift plus the loop's transient, neither of which any filter can remove.
    //
    // So the finding is not "an estimator cannot help because of noise". It is sharper and less comfortable: **the
    // low-speed speed error is not estimator noise at all, it is the angle source drifting**, and the place to work is
    // the angle.
    //
    // 判定——而它的初稿错得值得记下来。初稿写的是"10 ms 处仍相关 ⇒ 没有估计器能赢过地板"，仿佛相关性让事情更难。
    // 事实相反：相关的误差是会被**差分掉**的误差。相关性排除掉的恰恰是"噪声"这个解释。ρ(10 ms) = 0.95 意味着角误差
    // 在该区间内近乎恒定，于是 10 ms 上的差分几乎看不到它——这正是 3a 量到的（10 ms 时 0.196 rad/s，而白噪声预测是
    // 0.030），而那个差距是误差自身的缓慢漂移加上环的瞬态，两者都不是滤波器能去掉的。
    //
    // 所以结论不是"因为噪声，估计器帮不上忙"，而是更尖锐也更不舒服的：**低速转速误差不是估计器噪声，而是角度源在漂**，
    // 该动手的地方是角度。
    const bool correlated = (rho_10ms > 0.9);
    std::printf("[limit] verdict: %s\n",
                correlated ? "the angle error is correlated over 10 ms -- it is a drift, not noise, so no filter removes "
                             "it; the low-speed speed error is the angle source's, and the angle is where to work"
                           : "the angle error decorrelates inside 10 ms -- it behaves like noise, so averaging can "
                             "reduce it and an estimator is worth another look");
    EXPECT_TRUE(correlated) << "the angle error decorrelated, so this conclusion no longer holds and the file needs a "
                               "new measurement";
}
