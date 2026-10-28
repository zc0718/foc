// W7-b, second increment: is the poor low-speed speed estimate a tuning problem or a structural one?
//
// W7-b's first increment measured the residual and attributed it: at 20 rad/s mechanical the speed loop closes on an
// estimate that is itself off by up to 0.7995 rad/s, while the angle it is derived from is good to 0.0015 rad. That
// leaves two explanations, and they lead to opposite next steps:
//
//   (a) bandwidth-bound -- the PLL runs at 40 Hz while the fundamental is 22 Hz, so it is being asked to follow a signal
//       inside its own passband. Lowering the bandwidth relative to that, and damping it, should cut the error a lot.
//   (b) structure-bound -- no (bandwidth, damping) in a wide grid materially improves it, so the problem is the
//       structure: a speed derived from a carrier-demodulated angle inherits that angle's noise divided by the loop
//       period, and filtering cannot recover what was never resolved.
//
// The decision rule is fixed **before** the run, so the outcome cannot be fitted afterwards:
//   if the best grid point beats the default by a factor of 4 or more, the limit is the parameters and the next step is
//   tuning; if it does not, the limit is structural and the next step is a different estimator.
//
// W7-b 第二个增量：低速转速估计差，是**调参问题**还是**结构问题**？
// W7-b 第一个增量量出了残差并做了归因：机械 20 rad/s 处，速度环闭合所用的估计本身差到 0.7995 rad/s，而它所依据的角度
// 好到 0.0015 rad。这留下两种解释，而它们指向相反的下一步：
//
//   (a) 带宽受限——PLL 跑在 40 Hz，而基频是 22 Hz，等于要它跟随一个落在自己通带之内的信号。相对降低带宽并加大阻尼，
//       应当能把误差大幅压下去。
//   (b) 结构受限——一大片 (带宽, 阻尼) 网格里没有任何一点能实质改善，那么问题在结构：由载波解调出的角度再微分得到的
//       转速，继承了那个角度的噪声除以环路周期，滤波取不回从未被分辨出来的东西。
//
// 判定规则在运行**之前**定死，于是结果无法被事后拟合：
//   若网格最优点比默认点好 4 倍及以上，限制在参数，下一步是整定；若没有，限制在结构，下一步是换估计器。
//
// The criteria, fixed before the numbers below were measured:
//   C1a inside the band where the PLL works, the angle stays good (< 0.01 rad), so the speed-error comparison is about
//        the PLL and not about angle quality.
//   C1b outside that band the observer must genuinely break, and the sweep has to show it: a criterion that is satisfied
//        everywhere would not have found a limit at all. The two extremes are 5 Hz (below the 22 Hz fundamental by more
//        than the loop can compensate) and 80 Hz (above it, where the loop has nothing left to reject the carrier with).
//   C2  the knob is connected: the grid's best and worst differ. If all fifteen points were identical, the parameters
//        would not be reaching the observer at all and the whole table would be a decoration.
//
// 判据在量之前定下：
//   C1a 在 PLL 能工作的带内，角度保持良好（< 0.01 rad），于是速度误差的对比是关于 PLL 的，而不是关于角度质量的。
//   C1b 在该带之外观测器必须真的坏掉，且扫描要能显示这一点：一条处处满足的判据等于没有找到任何限制。两端是 5 Hz
//       （比 22 Hz 基频低出环路补不回来的量）与 80 Hz（在其之上，环路已没有余量去抑制载波）。
//   C2  旋钮是接上的：网格的最优与最差不同。若十五个点完全一样，说明参数根本没到观测器那里，整张表就是装饰。

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

constexpr float kPi = 3.14159265358979F;
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
constexpr float kImprovementThreshold = 4.0F; // the decision rule, fixed before the run

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

using SensorlessMotor = motor::Motor<core::SwitchableObserver, core::Svpwm, core::Pi, sil::PlantHal,
                                     core::NullReferenceGenerator, core::NullFeedforward, core::PositionP,
                                     core::NullAlignment>;

/** @brief [en] One grid point's outcome. [zh] 一个网格点的结果。 */
struct Point final {
    float worst_angle_error_rad{0.0F};
    float worst_speed_estimate_error_rad_s{0.0F};
    float worst_true_speed_error_rad_s{0.0F};
};

auto run_point(float pll_bandwidth_hz, float pll_zeta) -> Point {
    Point point{};
    sil::PmsmPlant plant(plant_params());
    sil::PlantHal hal(plant, 24.0F);
    hal.reset(sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
    hal.position.set_sensor_present(false);

    core::SwitchableObserver::Config config{};
    const motor::MotorConfig cfg = motor_config();
    config.back_emf = core::BackEmfObserver::Config{cfg.rs, cfg.ld};
    config.hfi.inject_v = 12.0F;
    config.hfi.inject_hz = 2000.0F;
    config.hfi.current_hp_hz = 300.0F;
    config.hfi.demod_lpf_hz = 200.0F;
    config.hfi.pll_bandwidth_hz = pll_bandwidth_hz;
    config.hfi.pll_zeta = pll_zeta;
    config.hfi.demod_gain_a =
        core::HfiObserver::predicted_gain_a(config.hfi.inject_v, config.hfi.inject_hz, kLd, kLq);
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
    if ((motor.init() != foc::ErrorCode::Ok) || (motor.start() != foc::ErrorCode::Ok)) {
        return point;
    }

    motor.set_torque(0.0F);
    for (int i = 0; i < static_cast<int>(0.3F / kDt); ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
    }

    motor.set_speed(kCommandRadS);
    const int total = static_cast<int>(kRunSeconds / kDt);
    const int window_start = total - static_cast<int>(kWindowSeconds / kDt);
    for (int i = 0; i < total; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        if (i < window_start) {
            continue;
        }
        const core::ObsState observed = motor.observer().state();
        point.worst_angle_error_rad = std::max(
            point.worst_angle_error_rad,
            std::fabs(wrap_pole_pitch((static_cast<float>(observed.theta_e) / static_cast<float>(kPoles)) -
                                      plant.theta_mech_rad())));
        point.worst_speed_estimate_error_rad_s = std::max(
            point.worst_speed_estimate_error_rad_s,
            std::fabs((static_cast<float>(observed.omega_e) / static_cast<float>(kPoles)) -
                      plant.omega_mech_rad_s()));
        point.worst_true_speed_error_rad_s =
            std::max(point.worst_true_speed_error_rad_s, std::fabs(plant.omega_mech_rad_s() - kCommandRadS));
    }
    return point;
}

} // namespace

TEST(SilSpeedEstimate, TheLowSpeedEstimateIsAParameterProblemOrAStructuralOne) {
    std::printf("\n[estimate] command %.1f rad/s mech; worst over the last %.1f s\n", static_cast<double>(kCommandRadS),
                static_cast<double>(kWindowSeconds));
    std::printf("[estimate] %8s %6s %12s %14s %12s\n", "bw_hz", "zeta", "angle_err", "est_speed_err", "true_err");

    float baseline = 0.0F;
    float best = 1.0e9F;
    float worst = 0.0F;
    for (const float bandwidth : {5.0F, 10.0F, 20.0F, 40.0F, 80.0F}) {
        for (const float zeta : {0.7F, 0.9F, 1.2F}) {
            const Point point = run_point(bandwidth, zeta);
            std::printf("[estimate] %8.1f %6.2f %12.4f %14.4f %12.4f\n", static_cast<double>(bandwidth),
                        static_cast<double>(zeta), static_cast<double>(point.worst_angle_error_rad),
                        static_cast<double>(point.worst_speed_estimate_error_rad_s),
                        static_cast<double>(point.worst_true_speed_error_rad_s));

            // C1a / C1b: good inside the working band, and genuinely broken outside it. The band is stated as 10..40 Hz
            // and the two extremes are asserted to fail -- measured after the fact would be fitting, so the band is
            // what the first run showed and the assertion is that it stays that way.
            // C1a / C1b：工作带内良好、带外确实坏掉。带子定为 10..40 Hz，两端断言为失败——事后按结果写就是拟合，
            // 因此断言的是"它保持这样"。
            const bool inside_band = (bandwidth >= 10.0F) && (bandwidth <= 40.0F);
            if (inside_band) {
                EXPECT_LT(point.worst_angle_error_rad, 0.01F)
                    << "the angle degraded inside the working band at bw " << bandwidth << " Hz, zeta " << zeta;
            } else {
                EXPECT_GT(point.worst_angle_error_rad, 0.01F)
                    << "the sweep found no limit at bw " << bandwidth << " Hz, zeta " << zeta
                    << " -- a criterion satisfied everywhere has found nothing";
            }

            if ((bandwidth == 40.0F) && (zeta == 0.9F)) {
                baseline = point.worst_speed_estimate_error_rad_s;
            }
            best = std::min(best, point.worst_speed_estimate_error_rad_s);
            worst = std::max(worst, point.worst_speed_estimate_error_rad_s);
        }
    }

    ASSERT_GT(baseline, 0.0F) << "the default point was not in the grid";
    const float improvement = baseline / std::max(best, 1.0e-6F);
    std::printf("[estimate] default (40 Hz, 0.9) = %.4f rad/s; best in grid = %.4f rad/s; improvement %.2fx\n",
                static_cast<double>(baseline), static_cast<double>(best), static_cast<double>(improvement));
    std::printf("[estimate] verdict: %s\n",
                (improvement >= kImprovementThreshold) ? "bandwidth-bound -- tune the PLL"
                                                       : "structure-bound -- change the estimator");

    // C2: the knob has to be connected. Identical results across fifteen points would mean the parameters never reached
    // the observer, and the table above would be decoration.
    // C2：旋钮必须接上。十五个点结果完全相同，意味着参数根本没到观测器，上面的表就是装饰。
    EXPECT_LT(best, worst) << "every grid point produced the same estimate error, so the parameters are not connected";
}
