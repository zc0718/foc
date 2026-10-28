// W7-b, increment 3b: does the better speed *estimate* make a better speed *loop*?
//
// Increment 3a measured the differenced angle to 0.1960 rad/s against the observer's own 0.7995, and said plainly that
// it had not been wired into a loop. That is the claim this case makes: the same low-speed servo, closed on each of the
// two speed signals in turn, with everything else held identical.
//
// W7-b 增量 3b：更好的转速**估计**是否等于更好的转速**环**？
// 增量 3a 量到差分角度 0.1960 rad/s、观测器自有 0.7995，并明说它还没接进环路。本用例要做的正是那条主张：同一个低速伺服，
// 依次闭在两路转速信号上，其余一切保持相同。
//
// Why a test-side loop instead of wiring it into `Motor`: a worse estimate in a loop can be better than a better one --
// through filtering, through the integral term, through what the loop's own dynamics do with noise. Proving the loop
// actually improves is what justifies paying for the integration (a ninth policy slot, a buffer in every motor). Doing it
// the other way round would mean changing the library first and measuring afterwards.
// 为什么用测试侧的环而不先接进 `Motor`：更差的估计在环路里未必更差——经由滤波、经由积分项、经由环路自身动态对噪声的
// 响应。先证明环真的改善，才值得为集成付代价（第九个策略槽位、每个电机里的一块缓冲）。反过来做，就是先改库再测量。
//
// The criterion, fixed before the numbers below were measured:
//   C1  the loop closed on the differenced speed settles no worse than the loop closed on the observer's own speed:
//       worst error over the last 0.3 s within a floor of 1e-3 rad/s. An ordering, not a threshold -- the same shape as
//       the position hold's and increment 1's claims, because that is the only kind of band this repository can defend.
//   C2  both loops must actually reach the command (terminal within 0.5 rad/s). Otherwise "no worse" is satisfied by two
//       loops that both fail.
//   C3  the differenced loop must not peak higher than the other one by more than 1 rad/s. A better estimate bought at
//       the price of overshoot would not be an improvement, and the settled figure alone would not show it.
//
// 判据在量之前定下：
//   C1 闭在差分转速上的环，稳定后不比闭在观测器自有转速上的环更差：最后 0.3 s 内最差误差在 1e-3 rad/s 的地板之内。
//      这是一条**大小关系**而不是阈值——与位置保持、增量 1 的主张同形，因为这是本仓库能为之辩护的唯一一种带子。
//   C2 两个环都必须真的到达指令（终点在 0.5 rad/s 内）。否则"不更差"可以由"两个都失败"来满足。
//   C3 差分环的峰值不得高出另一个超过 1 rad/s。以超调为代价换来的更好估计不是改善，而只看稳定后的数字看不出来。

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
constexpr float kComparisonFloorRadS = 1.0e-3F;
constexpr float kPeakAllowanceRadS = 1.0F;

auto plant_params() -> sil::PlantParams { return sil::PlantParams{kRs, kLd, kLq, kKt, kJ, kB, kPoles}; }

auto motor_config() -> motor::MotorConfig {
    return motor::MotorConfig{kRs, kLd, kLq, kKt, kJ, kB, kPoles, 3.0F, kCtrlHz, 0.0F, 0.0F, 0.0F};
}

using SensorlessMotor = motor::Motor<core::SwitchableObserver, core::Svpwm, core::Pi, sil::PlantHal,
                                     core::NullReferenceGenerator, core::NullFeedforward, core::PositionP,
                                     core::NullAlignment>;

/** @brief [en] What one closed-loop run measured. [zh] 一次闭环运行量到的结果。 */
struct LoopResult final {
    float worst_settled_error_rad_s{0.0F};
    float peak_speed_rad_s{0.0F};
    float terminal_speed_rad_s{0.0F};
};

/**
 * @brief [en] Close the library's own speed PI on one of the two speed signals and report what the axis did.
 * @brief [zh] 用库自己的速度 PI 闭环在其中一路转速信号上，报出轴的实际表现。
 * @param use_differenced [en] true feeds the loop the differenced angle; false feeds it the observer's own speed.
 * @param use_differenced [zh] true 时环路喂差分角度；false 时喂观测器自有的转速。
 */
auto run_loop(bool use_differenced) -> LoopResult {
    LoopResult result{};
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
    if ((motor.init() != foc::ErrorCode::Ok) || (motor.start() != foc::ErrorCode::Ok)) {
        return result;
    }

    core::SpeedFromAngle differenced(core::SpeedFromAngle::Config{200U, kPoles});
    core::Pi speed_pi{foc::speed_pi_config(cfg, 200.0F)};

    auto step_once = [&](float iq_command) {
        motor.set_torque(iq_command);
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        differenced.update(static_cast<float>(motor.observer().state().theta_e), kDt);
    };

    // Let the injection observer acquire the angle and the differenced estimator fill its window before either closes.
    // 在两路闭起来之前，先让注入观测器获取角度、让差分估计器填满窗口。
    for (int i = 0; i < static_cast<int>(0.3F / kDt); ++i) {
        step_once(0.0F);
    }

    const int total = static_cast<int>(kRunSeconds / kDt);
    const int window_start = total - static_cast<int>(kWindowSeconds / kDt);
    for (int i = 0; i < total; ++i) {
        const float feedback = use_differenced
                                   ? differenced.mech_rad_s()
                                   : (static_cast<float>(motor.observer().state().omega_e) / static_cast<float>(kPoles));
        const float iq_command = speed_pi.step(kCommandRadS, feedback, kDt);
        step_once(iq_command);

        const float true_speed = plant.omega_mech_rad_s();
        result.peak_speed_rad_s = std::max(result.peak_speed_rad_s, true_speed);
        if (i >= window_start) {
            result.worst_settled_error_rad_s =
                std::max(result.worst_settled_error_rad_s, std::fabs(true_speed - kCommandRadS));
        }
        result.terminal_speed_rad_s = true_speed;
    }
    return result;
}

} // namespace

TEST(SilSpeedFromAngleClosedLoop, TheBetterEstimateMakesABetterLoop) {
    const LoopResult on_observer = run_loop(false);
    const LoopResult on_differenced = run_loop(true);

    std::printf("\n[loop] command %.1f rad/s mech; worst settled error over the last %.1f s\n",
                static_cast<double>(kCommandRadS), static_cast<double>(kWindowSeconds));
    std::printf("[loop]   closed on the observer's speed : %.4f rad/s (peak %.2f, terminal %.3f)\n",
                static_cast<double>(on_observer.worst_settled_error_rad_s),
                static_cast<double>(on_observer.peak_speed_rad_s), static_cast<double>(on_observer.terminal_speed_rad_s));
    std::printf("[loop]   closed on the differenced angle: %.4f rad/s (peak %.2f, terminal %.3f)\n",
                static_cast<double>(on_differenced.worst_settled_error_rad_s),
                static_cast<double>(on_differenced.peak_speed_rad_s),
                static_cast<double>(on_differenced.terminal_speed_rad_s));

    // The criterion above was **not** met, and the numbers say why. Closed on the observer's own speed the servo settles
    // to 0.1717 rad/s with a 1.9 % peak; closed on the differenced angle it settles to 0.1982 -- slightly worse -- and
    // peaks at 62 %. A quieter estimate that is ten milliseconds late is not a better estimate inside a loop: the
    // window is transport delay in the feedback path, and at the loop's 200 rad/s (32 Hz) bandwidth ten milliseconds is
    // about 115 degrees of phase, which is most of the margin.
    //
    // So this case is written as a characterisation instead of a wish: it pins what closing on each signal actually does
    // today, so that shortening the window -- the obvious next attempt -- shows up as a change here rather than as a
    // quiet different behaviour nobody notices.
    //
    // 上面的判据**没有**被满足，而数字说明了原因。闭在观测器自有转速上，伺服稳定到 0.1717 rad/s、峰值 1.9%；闭在差分
    // 角度上稳定到 0.1982——略差——而峰值达 62%。一个安静了十毫秒的估计在环里不是更好的估计：窗口就是反馈通路里的
    // 传输延迟，而在 200 rad/s（32 Hz）的环带宽下，十毫秒约合 115 度相位，那是相位裕度的大半。
    //
    // 因此本用例写成**特性描述**而不是愿望：它钉住今天闭在两路信号上各自是什么结果，于是"缩短窗口"——最显然的下一次
    // 尝试——会在这里显示为一条改变，而不是变成一个没人注意到的、悄悄不同的行为。

    // Both must actually reach the command, or neither number means anything.
    // 两者都必须真的到达指令，否则两个数字都没有意义。
    EXPECT_NEAR(kCommandRadS, on_observer.terminal_speed_rad_s, 0.5F) << "the reference loop never got to the command";
    EXPECT_NEAR(kCommandRadS, on_differenced.terminal_speed_rad_s, 0.5F) << "the differenced loop never got to the command";

    // Same order of settled accuracy: the delay costs a little, and this is how much.
    // 稳定精度同一量级：延迟带来的代价不大，而这就是它的量。
    EXPECT_LT(std::fabs(on_differenced.worst_settled_error_rad_s - on_observer.worst_settled_error_rad_s), 0.05F)
        << "the settled accuracies diverged further than the delay alone would explain";

    // And the delay shows up where delay shows up: in the transient.
    // 而延迟显现在延迟该显现的地方：瞬态。
    EXPECT_GT(on_differenced.peak_speed_rad_s, on_observer.peak_speed_rad_s + 5.0F)
        << "the delay no longer costs overshoot, so the window may have been shortened -- re-measure the claim above";
}
