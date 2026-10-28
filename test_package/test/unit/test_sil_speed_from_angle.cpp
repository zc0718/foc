// W7-b, third increment: ask the same angle a different question.
//
// The sweep settled that the injection observer's reported speed is not a tuning problem: the best of fifteen (bandwidth,
// damping) points beat the default by 2.75x against a 4x decision threshold. The angle, meanwhile, is good to 0.0015 rad
// at this speed. So the next question is whether differencing that angle gives a better speed than the loop state does --
// and if it does, by how much against what the noise model predicts.
//
// W7-b 第三个增量：对同一个角度问另一个问题。
// 扫描已经确定注入观测器报出的转速不是调参问题：十五个 (带宽, 阻尼) 点里最优点相对默认只改善 2.75 倍，而判定阈值是 4 倍。
// 与此同时，这个转速下的角度好到 0.0015 rad。于是下一个问题是：对这个角度做差分，是否比环路状态给出更好的转速——以及
// 若更好，好多少，与噪声模型的预测是否相符。
//
// The criterion, fixed before the numbers below were measured:
//   C1  the differenced speed is at least **4x** better than the observer's own reported speed (<= 0.20 rad/s against
//       0.7995) -- the same factor the sweep used, so the two results are comparable rather than each flattering itself.
//   C2  it lands within a factor of **10** of the predicted `sigma_theta * sqrt(2) / (window_ticks * dt)` = 0.030 rad/s
//       mechanical, i.e. between 0.003 and 0.30. Inside that, the noise model is validated and the next step is known;
//       outside it, the angle's error is not white noise and the model is what needs fixing.
//   C3  the estimator had filled its window before the measurement window opened. Without this, a speed of 0 for the
//       first 200 ticks would satisfy C1 trivially -- a criterion that a silent output can pass is not a criterion.
//
// 判据在量之前定下：
//   C1 差分得到的转速比观测器自己报的至少好 **4 倍**（<= 0.20 rad/s 对 0.7995）——与扫描用同一个倍数，于是两个结果可比，
//      而不是各自往自己脸上贴金。
//   C2 落在预测值 `σ_θ·√2/(window_ticks·dt)` = 机械 0.030 rad/s 的 **10 倍**范围之内，即 0.003 到 0.30 之间。
//      在范围内，噪声模型成立、下一步已知；在范围外，说明角度的误差不是白噪声，需要修的是模型。
//   C3 估计器在测量窗口打开之前已经填满窗口。缺这一条，前 200 tick 输出 0 就能轻易满足 C1——一条"沉默的输出"也能通过的
//      判据不是判据。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/switchable_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/speed_from_angle.hpp>
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
constexpr float kWindowSeconds = 0.3F;

constexpr float kPllErrorBaselineRadS = 0.7995F; // measured in increment 1 at this operating point
constexpr float kRequiredFactor = 4.0F;
constexpr float kPredictedErrorRadS = 0.030F;
constexpr float kModelFactor = 10.0F;

auto plant_params() -> sil::PlantParams { return sil::PlantParams{kRs, kLd, kLq, kKt, kJ, kB, kPoles}; }

auto motor_config() -> motor::MotorConfig {
    return motor::MotorConfig{kRs, kLd, kLq, kKt, kJ, kB, kPoles, 3.0F, kCtrlHz, 0.0F, 0.0F, 0.0F};
}

using SensorlessMotor = motor::Motor<core::SwitchableObserver, core::Svpwm, core::Pi, sil::PlantHal,
                                     core::NullReferenceGenerator, core::NullFeedforward, core::PositionP,
                                     core::NullAlignment>;

} // namespace

TEST(SilSpeedFromAngle, DifferencingTheAngleBeatsTheObserversOwnSpeed) {
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

    // The window is the one the prediction assumes: 200 ticks of 50 us = 10 ms.
    // 窗口就是预测所假定的那个：50 us 的 200 个 tick = 10 ms。
    core::SpeedFromAngle differenced(core::SpeedFromAngle::Config{200U, kPoles});

    motor.set_torque(0.0F);
    for (int i = 0; i < static_cast<int>(0.3F / kDt); ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        differenced.update(static_cast<float>(motor.observer().state().theta_e), kDt);
    }

    motor.set_speed(kCommandRadS);
    const int total = static_cast<int>(kRunSeconds / kDt);
    const int window_start = total - static_cast<int>(kWindowSeconds / kDt);

    float worst_differenced = 0.0F;
    float worst_pll = 0.0F;
    bool ready_at_window_open = false;
    for (int i = 0; i < total; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        differenced.update(static_cast<float>(motor.observer().state().theta_e), kDt);
        if (i == window_start) {
            ready_at_window_open = differenced.ready();
        }
        if (i < window_start) {
            continue;
        }
        worst_differenced = std::max(worst_differenced, std::fabs(differenced.mech_rad_s() - plant.omega_mech_rad_s()));
        worst_pll = std::max(worst_pll,
                             std::fabs((static_cast<float>(motor.observer().state().omega_e) / static_cast<float>(kPoles)) -
                                       plant.omega_mech_rad_s()));
    }

    const float factor = worst_pll / std::max(worst_differenced, 1.0e-6F);
    std::printf("\n[from-angle] command %.1f rad/s mech; worst over the last %.1f s\n", static_cast<double>(kCommandRadS),
                static_cast<double>(kWindowSeconds));
    std::printf("[from-angle]   observer's own omega_e : %.4f rad/s\n", static_cast<double>(worst_pll));
    std::printf("[from-angle]   differenced angle       : %.4f rad/s  (ready at window open: %d)\n",
                static_cast<double>(worst_differenced), static_cast<int>(ready_at_window_open));
    std::printf("[from-angle]   improvement %.1fx; predicted %.4f, model band %.4f..%.4f\n", static_cast<double>(factor),
                static_cast<double>(kPredictedErrorRadS), static_cast<double>(kPredictedErrorRadS / kModelFactor),
                static_cast<double>(kPredictedErrorRadS * kModelFactor));

    // C3 first: a silent estimator would satisfy C1 for free.
    // 先查 C3：一个沉默的估计器会白白满足 C1。
    ASSERT_TRUE(ready_at_window_open) << "the estimator had not filled its window, so its output was not a measurement";
    EXPECT_TRUE(differenced.ready());
    EXPECT_FALSE(differenced.lost_count()) << "the differencing lost the angle count during the run";

    // C1
    EXPECT_LE(worst_differenced, kPllErrorBaselineRadS / kRequiredFactor)
        << "differencing the angle did not beat the observer's own speed by the factor the sweep's threshold used";
    // And the baseline it is being compared against must still be there, or the comparison is against a memory.
    // 而被比较的那个基线必须仍然成立，否则就是在和记忆比较。
    EXPECT_NEAR(kPllErrorBaselineRadS, worst_pll, 0.15F) << "the observer's own speed error moved, so the baseline is stale";

    // C2
    EXPECT_GT(worst_differenced, kPredictedErrorRadS / kModelFactor) << "far better than the noise model predicts";
    EXPECT_LT(worst_differenced, kPredictedErrorRadS * kModelFactor) << "far worse than the noise model predicts";
}
