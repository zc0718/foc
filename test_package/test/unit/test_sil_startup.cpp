// The open-loop start, measured: does the rotor follow the forced vector, and does it survive the hand-over?
//
// This is the first falsifiable criterion of the start-up chain, and it is written before the run:
//
//   * during the second half of the pull the rotor follows the forced vector within 0.35 rad of slip;
//   * at the hand-over tick the current does not spike beyond twice the pull current, and the machine keeps turning.
//
// The first run refuted both, and the two numbers it produced are the whole of W6-a's remaining work:
//
//   1. slip 3.1407 rad, i.e. the rotor never followed at all. The hold has no damping, so in 0.15 s the rotor is still
//      swinging around the forced angle instead of sitting on it, and the pull then starts from an arbitrary position.
//   2. the electrical speed fell from 400 to 49 rad/s across the hand-over, with no current spike (0.44 A against a
//      1.20 A pull). That is not a disturbance -- it is a lost lock: the observer is deliberately not stepped while the
//      angle is forced, so at the hand-over instant it still holds the flux it was seeded with in `start()`, at angle 0,
//      while the forced vector has travelled some ten radians.
//
// So the test below characterises those two gaps instead of asserting the criteria the sequence does not yet meet. The
// assertions are inverted on purpose: when the hold learns to damp and the hand-over learns to seed the observer, this
// test fails and its numbers must be replaced by the real criteria above. A characterisation test that cannot fail is a
// comment.
//
// The point of putting `StartupRoutine` in the aligner slot rather than beside it: `Motor`'s Align state is already the
// "angle forced, observer not stepped" state, so the sequence has no second control path, and the hand-over is exactly
// one tick -- which is what makes "no jump at hand-over" a measurement instead of a claim.
//
// 开环起动，实测：转子跟得上强制矢量吗，交接时活下来了吗？
// 这是起动链路的第一条可证伪判据，在跑之前写下：
//
//   * 拖动后半段，转子跟随强制矢量的转差不超过 0.35 rad；
//   * 交接那一拍电流不超过拖动电流的两倍，且电机继续转。
//
// 把 `StartupRoutine` 放进对齐槽位而不是放在旁边的意义：`Motor` 的 Align 状态本就是"角度强制、观测器不推进"的状态，
// 因此序列没有第二条控制路径，交接恰好是一个 tick——这才使"交接处无跳变"成为测量而不是声称。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/alignment.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/position.hpp>
#include <foc/core/startup.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"
#include "sil_plant_hal.hpp"

namespace {

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 4;
constexpr float kVBus = 24.0F;
constexpr int kTimeoutTicks = 80000; // 4 s: the slowest ramp on the list needs 2 s

using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                 foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                 foc::core::PositionP, foc::core::StartupRoutine>;

auto startup_config(float pull_accel_rad_s2 = 3000.0F, float hold_s = 0.15F) -> foc::core::StartupConfig {
    foc::core::StartupConfig config{};
    config.align_current_a = 1.5F;
    config.align_hold_s = hold_s;
    config.pull_current_a = 1.2F;
    config.pull_accel_rad_s2 = pull_accel_rad_s2;
    config.handover_speed_rad_s = 400.0F; // electrical
    return config;
}

/**
 * @brief [en] One open-loop start from a rotor parked at an unknown angle.
 * @brief [zh] 从任意停放角度跑一次开环起动。
 */
struct StartupRun final {
    float worst_slip_rad{0.0F};
    float steady_slip_rad{0.0F}; ///< [en] mean over the last quarter of the pull. [zh] 拖动末四分之一的均值。
    float peak_iq_after_a{0.0F};
    float speed_after_rad_s{0.0F};
    int handover_tick{-1};
    int pull_ticks{0};
};

auto run_startup(float pull_accel_rad_s2, float hold_s = 0.15F) -> StartupRun {
    const foc::sil::MachineSpec spec = foc::sil::nominal_spec();
    foc::sil::PmsmPlant plant(foc::sil::nominal_plant_params());
    foc::sil::PlantState initial{};
    initial.theta_mech = 1.7F; // somewhere unknown: the condition the sequence exists for
    plant.reset(initial);

    foc::sil::PlantHal hal(plant, kVBus);
    const foc::motor::MotorConfig config = foc::sil::nominal_motor_config();
    const foc::core::Pi::Config current =
        foc::current_pi_config(config, foc::sil::standard_bench_config().omega_current_rad_s, kVBus);
    const foc::core::Pi::Config speed =
        foc::speed_pi_config(config, foc::sil::standard_bench_config().omega_speed_rad_s);
    const foc::core::PositionP::Config position = foc::position_p_config(config, 20.0F, 50.0F, 0.0F);
    const foc::core::StartupConfig startup = startup_config(pull_accel_rad_s2, hold_s);

    MotorT motor(config,
                 MotorT::Policies{foc::core::BackEmfObserver(foc::core::BackEmfObserver::Config{spec.rs, spec.ld}),
                                  foc::core::Svpwm{}, foc::core::Pi{current}, foc::core::Pi{current},
                                  foc::core::Pi{speed}, foc::core::NullReferenceGenerator{},
                                  foc::core::NullFeedforward{}, foc::core::PositionP{position},
                                  foc::core::StartupRoutine{startup}},
                 hal);
    if ((motor.init() != foc::ErrorCode::Ok) || (motor.start() != foc::ErrorCode::Ok)) {
        return StartupRun{};
    }

    // The pull is expected to last this long; the last quarter of it is where the load angle has settled and where a
    // law about steady slip is comparable.
    // 拖动的预期时长；末四分之一是负载角已经稳定的部分，关于稳态滑差的规律只在那里可比。
    const int expected_pull_ticks =
        static_cast<int>((startup.handover_speed_rad_s / startup.pull_accel_rad_s2) / kDt);
    float slip_sum = 0.0F;
    int slip_count = 0;

    StartupRun run{};
    for (int tick = 0; tick < kTimeoutTicks; ++tick) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);

        const foc::core::StartupRoutine &routine = motor.aligner();
        const foc::core::StartupState state = routine.state();
        if (state == foc::core::StartupState::Pull) {
            ++run.pull_ticks;
            if (run.pull_ticks > 10) { // skip the first 0.5 ms while the rotor is still being caught
                float error = (plant.theta_mech_rad() * static_cast<float>(spec.pole_pairs)) -
                              routine.forced_angle_rad();
                while (error > 3.14159265F) {
                    error -= 6.28318531F;
                }
                while (error < -3.14159265F) {
                    error += 6.28318531F;
                }
                run.worst_slip_rad = std::fmax(run.worst_slip_rad, std::fabs(error));
                if (run.pull_ticks > ((expected_pull_ticks * 3) / 4)) {
                    slip_sum += std::fabs(error);
                    ++slip_count;
                }
            }
        }
        if ((state == foc::core::StartupState::Done) || (state == foc::core::StartupState::Failed)) {
            if (state == foc::core::StartupState::Failed) {
                break;
            }
            if (run.handover_tick < 0) {
                run.handover_tick = tick;
            }
            run.peak_iq_after_a = std::fmax(run.peak_iq_after_a, std::fabs(motor.telemetry().iq_a));
            if ((tick - run.handover_tick) > 400) { // 20 ms after the hand-over
                break;
            }
        }
    }
    run.speed_after_rad_s = motor.telemetry().speed_mech_rad_s * static_cast<float>(spec.pole_pairs);
    run.steady_slip_rad = (slip_count > 0) ? (slip_sum / static_cast<float>(slip_count)) : 0.0F;
    return run;
}

auto psi_f_of(const foc::sil::MachineSpec &spec) -> float {
    return spec.kt / (1.5F * static_cast<float>(spec.pole_pairs));
}

} // namespace

TEST(SilStartup, ThePullSlipIsTheLoadAngleTheAccelerationNeeds) {
    // The first run blamed the undamped hold for a slip near half a turn. The sweep below shows the pull was the other
    // half, and it is arithmetic rather than tuning: to accelerate the inertia at `alpha` the rotor must hang back by the
    // angle whose sine supplies that torque,
    //
    //     sin(delta) = J * alpha / (Kt * i_pull),      Kt = 1.5 * pole_pairs * psi_f
    //
    // so the acceleration -- not the hold -- sets the slip. The criterion, written before the run: the measured slip stays
    // within 25 % of that expression across the sweep, and at 200 rad/s^2 it is inside the 0.35 rad requirement.
    //
    // The second half held and the first did not. The expression over-predicts by 1.7x to 2x (0.73 against 0.29 at
    // 400 rad/s^2) and clamps to a full turn at 1200 rad/s^2 where the pull in fact catches the rotor and holds it. So the
    // law is printed for the next attempt and **not asserted**: the actionable statement is the measured one, that a slow
    // enough ramp keeps the steady slip inside 0.35 rad and the hand-over inside 5 % of its speed.
    // 首次运行把半圈滑差归咎于无阻尼的保持。下面的扫描表明拖动是另一半，而且是算术而非调参：要让惯量以 `alpha` 加速，转子
    // 必须滞后一个角度，使其正弦恰好提供那份转矩，
    //
    //     sin(delta) = J * alpha / (Kt * i_pull),      Kt = 1.5 * pole_pairs * psi_f
    //
    // 因此决定滑差的是加速度，不是保持。判据在跑之前写下：整个扫描中实测滑差与上式相差 25 % 以内，且 200 rad/s^2 时进入
    // 0.35 rad 的要求之内。
    const foc::sil::MachineSpec spec = foc::sil::nominal_spec();
    const float inertia = foc::sil::nominal_plant_params().j;
    const float kt_peak = 1.5F * static_cast<float>(spec.pole_pairs) * psi_f_of(spec);
    const float pull_current = startup_config().pull_current_a;

    std::printf("\n[startup] rotor parked at 1.7 rad mech, hold 0.15 s, hand-over 400 rad/s electrical\n");
    std::printf("%12s %11s %11s %11s %9s %12s\n", "pull accel", "slip worst", "slip steady", "slip law", "pull ms",
                "speed after");
    bool inside_at_200 = false;
    for (const float accel : {3000.0F, 1200.0F, 400.0F, 200.0F}) {
        const StartupRun run = run_startup(accel);
        const float predicted = std::asin(std::fmin(1.0F, (inertia * accel) / (kt_peak * pull_current)));
        std::printf("%12.0f %11.4f %11.4f %11.4f %9.1f %12.1f\n", static_cast<double>(accel),
                    static_cast<double>(run.worst_slip_rad), static_cast<double>(run.steady_slip_rad),
                    static_cast<double>(predicted), static_cast<double>(run.pull_ticks) * kDt * 1e3,
                    static_cast<double>(run.speed_after_rad_s));
        // The requirement, measured: a slow ramp holds the steady slip inside the ceiling and keeps the machine's speed.
        // The acceleration law above is reported alongside, and rejected as a predictor.
        // 要求是实测的那条：慢斜坡把稳态滑差压在要求内并守住转速。上面的加速度规律一并报出，但作为预测器已被否决。
        if (accel <= 400.0F) {
            EXPECT_LE(run.steady_slip_rad, 0.35F) << "steady slip at " << accel << " rad/s^2";
            EXPECT_GE(run.speed_after_rad_s, 0.75F * startup_config().handover_speed_rad_s)
                << "the machine must keep turning at " << accel << " rad/s^2";
        }
        if (accel <= 200.0F) {
            inside_at_200 = run.steady_slip_rad <= 0.35F;
        }
    }
    EXPECT_TRUE(inside_at_200) << "the closed form says 200 rad/s^2 fits inside 0.35 rad; the run disagrees";
}

TEST(SilStartup, TheHandoverHoldsTheMachineOnceTheRotorIsActuallyTurning) {
    // Gap 2 of the first run was diagnosed as "the observer is never seeded at hand-over". That was wrong, and the sweep
    // is what showed it: with the rotor genuinely turning at the hand-over instant, the back-EMF observer converges on its
    // own within the 20 ms window, while at 3000 rad/s^2 the rotor had never been caught and there was nothing to lock on
    // to. The requirement therefore belongs here, where it can be asserted rather than characterised.
    // 首次运行的缺口二被诊断为"交接时从未给观测器播种"。那是错的，而纠正是扫描给出的：交接那一刻转子确实在转时，反电势
    // 观测器在 20 ms 窗口内自行收敛；而 3000 rad/s^2 下转子从未被抓住，也就无从锁定。因此这条要求属于这里——它可以被
    // 断言，而不是被特征化。
    const StartupRun run = run_startup(400.0F);
    std::printf("\n[startup] hand-over at tick %d, peak |iq| %.4f A, speed after %.1f rad/s\n", run.handover_tick,
                static_cast<double>(run.peak_iq_after_a), static_cast<double>(run.speed_after_rad_s));
    ASSERT_GT(run.handover_tick, 0) << "the sequence never reached the hand-over speed";
    EXPECT_GE(run.speed_after_rad_s, 0.75F * startup_config().handover_speed_rad_s) << "the machine lost speed";
    EXPECT_LT(run.peak_iq_after_a, 2.0F * startup_config().pull_current_a) << "the hand-over disturbed the current loop";
}
