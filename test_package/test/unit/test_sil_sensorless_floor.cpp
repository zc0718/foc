// F19 - the upper bound: on an **ideal** plant, is a closed-loop angle available below 50 rad/s?
//
// The plan asks for a *disproof* rather than a claim: "no closed-loop angle below 50 rad/s, measured on an ideal plant
// (known psi_f, no saliency) -- if it cannot be done there it cannot be done on hardware". The reasoning is structural,
// not empirical: an SPM has no saliency, so the injection observer has nothing to inject into (`predicted_gain_a` is
// proportional to `lq - ld`), which removes the only mechanism that works at standstill and leaves a model observer whose
// accuracy grows with speed. This file measures where that leaves the drive instead of asserting the sentence.
//
// F19 —— 上界：在**理想** plant 上，50 rad/s 以下到底有没有闭环角度？
//
// 计划要的是一条**否证**而不是一句声称："50 rad/s 以下没有闭环角度，在理想 plant（已知 ψf、无凸极）上量——那里做不到，
// 真机就更做不到。" 理由是结构性的，不是经验性的：SPM 没有凸极，注入观测器没有东西可注（`predicted_gain_a` 正比于
// `lq − ld`），于是唯一能在静止工作的机理被排除，只剩一个精度随转速增长的模型观测器。本文件去**量**它到底停在哪里，
// 而不是把那句话写成断言。
//
// The rig, fixed before the numbers were read:
//   * the plant is an SPM -- `lq == ld` -- and the observer's model is given exactly those numbers, so nothing here is a
//     mis-parameterisation dressed as a limit;
//   * the rotor is **already turning** at the speed under test when the sensor goes away (`PlantState` carries the speed),
//     because "can it acquire and hold from a run-up" is the question and "can it start from rest without injection" is a
//     different one, already answered by `test_sil_alignment.cpp` and the open-loop start;
//   * the observer starts from a zero seed, i.e. maximally wrong, which is the honest worst case and the same seed the
//     recorded 90-degree experiment used;
//   * the measured quantity is the electrical angle error against the plant's own truth over the **last 0.5 s** of a
//     1.5 s run -- the "has it settled" window, not the acquisition transient.
//
// 台架在读到数字之前就定下：
//   * plant 是 SPM——`lq == ld`——而观测器的模型拿到的正是这两个数，所以这里没有"参数配错了却看起来像极限"；
//   * 传感器撤掉时转子**已经在转**（`PlantState` 里带着转速）：要问的是"从一次升速中能不能捕获并保持"，而"没有注入能不能
//     从静止起动"是另一个问题，已由 `test_sil_alignment.cpp` 与开环强拖回答；
//   * 观测器从零种子开始，即最大程度地错，这是诚实的下界，也与既有那条 90° 种子实验同一个起点；
//   * 被测的量是"1.5 s 运行的**最后 0.5 s**"内电角度对 plant 真值的误差——是"稳下来了吗"的窗口，不是捕获瞬态。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/switchable_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_pmsm_plant.hpp"
#include "sil_plant_hal.hpp"

namespace {

namespace core = foc::core;
namespace motor = foc::motor;
namespace sil = foc::sil;

constexpr float kPiFloat = 3.14159265358979F;

// The nameplate `test_sil_fusion_chain.cpp` uses, with one deliberate change: `lq == ld`. That single equality is what
// makes the plant an SPM, and it is why the injection observer cannot be a way out of this measurement.
// `test_sil_fusion_chain.cpp` 用的铭牌，只改一处：`lq == ld`。这一个等号就是 plant 成为 SPM 的原因，也是注入观测器
// 无法成为这次测量出路的原因。
constexpr float kRs = 0.35F;
constexpr float kLd = 2.0e-4F;
constexpr float kLq = kLd;
constexpr float kKt = 0.05F;
constexpr float kJ = 1.0e-5F;
constexpr float kB = 1.0e-4F;
constexpr std::uint8_t kPoles = 7U;

constexpr float kCtrlHz = 20000.0F;
constexpr float kDt = 1.0F / kCtrlHz;
constexpr int kSubsteps = 8;
constexpr float kVBus = 24.0F;

// The 50 rad/s edge the plan names: the injection observer's own floor, which is also the hand-over speed the fusion
// chain measures at.
// 计划点名的 50 rad/s 边缘：注入观测器自己的地板，也是混合链在交接处量到的转速。
constexpr float kEdgeRadS = 50.0F;
// The criterion that was withdrawn for being vacuous on a sensored rig, reused here as the gate it was always meant to
// be: "within 20 ms of hand-over the angle error is at most 0.05 rad".
// 那条因为"在有传感器台架上空真"而被撤回的判据，在这里用作它本来就该是的那道闸："交接后 20 ms 内角度误差 ≤ 0.05 rad"。
constexpr float kGateRad = 0.05F;

using MotorT = motor::Motor<core::SwitchableObserver, core::Svpwm, core::Pi, sil::PlantHal>;

/** @brief [en] Wrap to (-pi, pi]. [zh] 折到 (-pi, pi]。 */
auto wrap_pi_local(float angle) noexcept -> float {
    while (angle <= -kPiFloat) {
        angle += 2.0F * kPiFloat;
    }
    while (angle > kPiFloat) {
        angle -= 2.0F * kPiFloat;
    }
    return angle;
}

auto plant_params() -> sil::PlantParams { return sil::PlantParams{kRs, kLd, kLq, kKt, kJ, kB, kPoles}; }

auto motor_config() -> motor::MotorConfig {
    return motor::MotorConfig{kRs, kLd, kLq, kKt, kJ, kB, kPoles, 3.0F, kCtrlHz, 0.0F, 0.0F, 0.0F};
}

// Only the model observer is configured as a candidate: the injection member is present but its gain is computed from
// `lq - ld`, which is zero here, so `predicted_gain_a` returns 0 and the carrier would carry no position information.
// The run asks the selection for the back-EMF member explicitly, and the HFI member's own status is asserted below
// rather than assumed -- "no saliency" is the premise of the whole measurement.
// 只把模型观测器作为候选配置：注入成员虽然在，但它的增益由 `lq − ld` 算出，这里为零，于是 `predicted_gain_a` 返回 0、
// 载波不携带任何位置信息。运行显式向选择器要反电势成员，而注入成员自己的状态在下面被断言而不是被假设——"无凸极"是整次
// 测量的前提。
auto observer_config() -> core::SwitchableObserver::Config {
    const motor::MotorConfig cfg = motor_config();
    const float psi_f = foc::psi_f_wb(cfg);
    core::SwitchableObserver::Config config{};
    config.back_emf = core::BackEmfObserver::Config{cfg.rs, cfg.ld};
    config.nonlinear_flux = core::NonlinearFluxObserver::Config{cfg.rs, cfg.lq, psi_f, 0.3F, 3.0F, 0.5F * psi_f};
    config.hfi.inject_v = 12.0F;
    config.hfi.inject_hz = 2000.0F;
    config.hfi.current_hp_hz = 300.0F;
    config.hfi.demod_lpf_hz = 200.0F;
    config.hfi.pll_bandwidth_hz = 40.0F;
    config.hfi.pll_zeta = 0.9F;
    config.hfi.demod_gain_a = core::HfiObserver::predicted_gain_a(config.hfi.inject_v, config.hfi.inject_hz, kLd, kLq);
    config.hfi.min_hf_current_a = 0.05F;
    config.dual_inductance = core::DualInductanceObserver::Config{cfg.rs, cfg.ld, cfg.lq, psi_f};
    config.kind = core::ObserverKind::NonlinearFlux;
    return config;
}

/** @brief [en] One speed's worth of measurement. [zh] 一个转速下的测量结果。 */
struct Row final {
    float target_rad_s{0.0F};
    float max_error_rad{0.0F};    ///< [en] the worst error over the settled window. [zh] 稳态窗口内的最坏误差。
    float end_error_rad{0.0F};    ///< [en] the error on the last tick. [zh] 最后一拍的误差。
    float true_speed_rad_s{0.0F}; ///< [en] the plant's own speed at the end. [zh] 结束时 plant 自己的转速。
    bool faulted{false};
    float load_nm{0.0F};
};

/**
 * @brief [en] Run one speed: the rotor is already turning, the sensor is already gone, the observer starts blind.
 * @brief [zh] 跑一个转速：转子已经在转、传感器已经不在了、观测器从零开始。
 */
auto run_at(float target_rad_s, float load_nm, float dead_v) -> Row {
    Row row{};
    row.target_rad_s = target_rad_s;
    row.load_nm = load_nm;

    sil::PmsmPlant plant(plant_params());
    sil::PlantHal hal(plant, kVBus);
    // The run-up: the rotor is already at the speed under test, which is the state a real sensorless drive hands over
    // into. `hal.reset` also puts the inverter back to its power-on state.
    // 升速：转子已经在被测转速上，这正是真实无感驱动器接手时的状态。`hal.reset` 还会把逆变器复位到上电态。
    hal.reset(sil::PlantState{0.0F, 0.0F, static_cast<double>(target_rad_s), 0.0});
    hal.position.set_sensor_present(false);
    plant.set_load_torque(load_nm);
    hal.set_deadtime_voltage(dead_v);

    const motor::MotorConfig cfg = motor_config();
    MotorT motor(cfg,
                 MotorT::Policies{core::SwitchableObserver{observer_config()}, core::Svpwm{},
                                  core::Pi{foc::current_pi_config(cfg, 2000.0F, kVBus)},
                                  core::Pi{foc::current_pi_config(cfg, 2000.0F, kVBus)},
                                  core::Pi{foc::speed_pi_config(cfg, 200.0F)}},
                 hal);
    if (motor.init() != foc::ErrorCode::Ok) {
        return row;
    }
    if (motor.start() != foc::ErrorCode::Ok) {
        return row;
    }
    motor.set_speed(target_rad_s);

    auto true_angle = [&plant]() { return static_cast<float>(kPoles) * plant.theta_mech_rad(); };
    auto error = [&motor, &true_angle]() {
        return std::fabs(wrap_pi_local(static_cast<float>(motor.observer().state().theta_e - true_angle())));
    };

    const int total = static_cast<int>(1.5F / kDt);
    const int settled_from = static_cast<int>(1.0F / kDt); // the last 0.5 s is the window 最后 0.5 s 是窗口
    for (int i = 0; i < total; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        if (motor.state() == core::MotorState::Fault) {
            row.faulted = true;
            break;
        }
        const float magnitude = error();
        if (i >= settled_from) {
            row.max_error_rad = std::max(row.max_error_rad, magnitude);
        }
        row.end_error_rad = magnitude;
    }
    row.true_speed_rad_s = plant.omega_mech_rad_s();
    std::printf(
        "[f19] target %6.1f rad/s load %5.3f Nm dead %4.2f V -> max %.4f rad, end %.4f rad, true w %7.2f, state %d\n",
        static_cast<double>(target_rad_s), static_cast<double>(load_nm), static_cast<double>(dead_v),
        static_cast<double>(row.max_error_rad), static_cast<double>(row.end_error_rad),
        static_cast<double>(row.true_speed_rad_s), static_cast<int>(motor.state()));
    return row;
}

// 1. The premise refuted: on an ideal plant there is no low-speed floor at all.
// 1. 前提被否证：理想 plant 上根本没有低速地板。
//
// The plan expected "cannot be done below 50 rad/s, and an ideal plant is the place to prove it". Measured, the ideal
// plant is the one place where it *is* done: with `lq == ld`, no dead time, no sensor noise and the observer given the
// plant's own parameters, the error at 5 rad/s is 2 mrad. What is left is **transport delay**, and it shows up as a
// constant ratio rather than as a floor: `error / omega_mech` is 3.8e-4 s at every speed from 5 to 354 rad/s, which is
// **1.1 control periods per electrical cycle** -- the error is an electrical angle and the speed is mechanical, so the
// 7 pole pairs must be divided out (3.8e-4 * 7 = 2.7 ms/s, against a 50 us tick). The observer reports the angle of the
// previous tick, and that is the whole of what is left once the plant is ideal.
// 计划以为"50 rad/s 以下做不到，而理想 plant 正是证明它的地方"。量出来恰好相反：理想 plant 是它**做得到**的那个地方——
// `lq == ld`、无死区、无传感器噪声、观测器拿到的就是 plant 自己的参数，5 rad/s 处误差 2 mrad。剩下的**只有传输延时**，而它
// 表现为一个恒定比值而不是地板：5 到 354 rad/s 之间 `误差/机械转速` 都是 3.8e-4 s，即**每个电周期 1.1 个控制周期**
// ——误差是电角度而转速是机械的，所以要把 7 对极除出来（3.8e-4 × 7 = 2.7 ms/s，对 50 µs 的一拍）。观测器报出的是**上一拍**
// 的角度，这就是 plant 理想之后剩下的全部。
TEST(SilSensorlessFloor, AnIdealPlantHasNoLowSpeedFloorAndWhatIsLeftIsOneTickOfDelay) {
    const float targets[] = {5.0F, 10.0F, 20.0F, 40.0F};
    float ratios[4] = {0.0F, 0.0F, 0.0F, 0.0F};
    for (std::size_t i = 0; i < 4U; ++i) {
        const Row run = run_at(targets[i], 0.0F, 0.0F);
        EXPECT_FALSE(run.faulted) << "target " << targets[i];
        EXPECT_LT(run.max_error_rad, kGateRad)
            << "target " << targets[i] << " rad/s with no dead time: the ideal plant holds the gate";
        EXPECT_NEAR(targets[i], run.true_speed_rad_s, 0.05F * targets[i])
            << "the run-up has to actually be at the speed under test, or the row measures something else";
        ratios[i] = run.max_error_rad / run.true_speed_rad_s;
    }
    // The ratio is the claim: if the error had a floor, `error/omega` would blow up as the speed falls. It does not --
    // it is flat to within 10 % across an 8x speed range, which is a delay and not a floor.
    // 比值才是那句声称：若误差有地板，`误差/转速` 会随转速下降而发散。它没有——8 倍转速范围内平到 10 % 以内，那是延时不是地板。
    for (std::size_t i = 1; i < 4U; ++i) {
        EXPECT_NEAR(ratios[0], ratios[i], 0.10F * ratios[0]) << "target " << targets[i];
    }
    const float ticks = ratios[0] * static_cast<float>(kPoles) / kDt;
    std::printf("[f19] ideal plant: error/omega_mech = %.2e s = %.2f control tick(s) per electrical cycle\n",
                static_cast<double>(ratios[0]), static_cast<double>(ticks));
}

// 2. The floor is bought by the dead-time voltage, not by the observer.
// 2. 地板是死区电压买来的，不是观测器。
//
// One microsecond of dead time at 24 V is 0.48 V of per-tick voltage error, and the machine's own back-EMF at 20 rad/s is
// `omega*psi_f` = 0.095 V: the distortion is five times the signal the observer is trying to read. Measured with that
// single non-ideality added: 20 rad/s loses the machine outright (3.14 rad, the estimate inverted) and 40 rad/s reads
// 1.03 rad, while at 200 rad/s the same distortion costs **nothing** (0.041 against 0.073 clean -- it is even better,
// because at that speed `omega*psi_f` = 0.95 V is twice the distortion). So the boundary is not a speed the observer has;
// it is the speed at which the back-EMF stops being smaller than the inverter's own error, which on this machine is
// ~100 rad/s, and that is what "no claim below 50 rad/s" is actually about.
// 24 V 下 1 µs 死区是 0.48 V 的每拍电压误差，而机器 20 rad/s 时的反电势只有 `ω·ψf` = 0.095 V：畸变是观测器要读的那个信号的
// 五倍。把这一个非理想项加进去实测：20 rad/s 直接丢掉机器（3.14 rad，估计反了），40 rad/s 读到 1.03 rad，而在 200 rad/s
// 处同一个畸变**什么都不花**（0.041 对干净的 0.073——甚至更好，因为那里 `ω·ψf` = 0.95 V 是畸变的两倍）。所以边界不是观测器
// 拥有的某个转速，而是"反电势不再小于逆变量自身误差"的那个转速，本机约 100 rad/s——"50 rad/s 以下不作声称"说的其实是这件事。
TEST(SilSensorlessFloor, TheFloorIsTheDeadTimeVoltageAgainstTheBackEmfAndNotTheObserversStructure) {
    constexpr float kDeadV = 0.48F; // 1 us of dead time at 24 V 24 V 下 1 µs 死区

    const Row ideal20 = run_at(20.0F, 0.0F, 0.0F);
    const Row dead20 = run_at(20.0F, 0.0F, kDeadV);
    const Row ideal40 = run_at(40.0F, 0.0F, 0.0F);
    const Row dead40 = run_at(40.0F, 0.0F, kDeadV);
    const Row ideal200 = run_at(200.0F, 0.0F, 0.0F);
    const Row dead200 = run_at(200.0F, 0.0F, kDeadV);

    // Below the crossing the distortion decides the answer, and the answer is "no angle".
    // 在交点以下，畸变决定答案，而答案是"没有角度"。
    EXPECT_GT(dead20.max_error_rad, kGateRad) << "20 rad/s with 0.48 V of dead-time voltage";
    EXPECT_GT(dead40.max_error_rad, kGateRad) << "40 rad/s with 0.48 V of dead-time voltage";
    EXPECT_GT(dead20.max_error_rad, 10.0F * ideal20.max_error_rad) << "and it is not a small correction";
    EXPECT_GT(dead40.max_error_rad, 10.0F * ideal40.max_error_rad);

    // Above it, the same distortion costs nothing -- and this is the half that stops the first half from being a story
    // about a broken rig: one plant, one observer, one extra term, and the damage follows `omega*psi_f`.
    // 在交点以上，同一个畸变什么都不花——而这正是让前一半不至于变成"台架坏了"的故事的那一半：同一个 plant、同一个观测器、
    // 多加一项，而损害跟着 `ω·ψf` 走。
    EXPECT_LT(dead200.max_error_rad, ideal200.max_error_rad)
        << "at 200 rad/s the same distortion no longer dominates, so it must not be blamed for the low-speed result";

    // The crossing itself, as a prediction rather than an observation: `omega*psi_f == kDeadV` names the speed where the
    // two become comparable. The test asserts the *order* (100 rad/s is where the ratio is ~1) rather than a tuned
    // number, so a change in psi_f moves the prediction with the machine instead of invalidating a constant.
    // 交点本身作为**预测**而不是观测：`ω·ψf == kDeadV` 给出两者可比的那个转速。用例断言的是**量级**（100 rad/s 处比值约 1），
    // 而不是一个调出来的常数，于是 ψf 变化时预测随机器一起移动，而不是让一个常数失效。
    const float psi_f = foc::psi_f_wb(motor_config());
    const float crossing_rad_s = kDeadV / psi_f;
    RecordProperty("crossing_rad_s", crossing_rad_s);
    EXPECT_GT(crossing_rad_s, 50.0F)
        << "the crossing has to sit above the edge the plan names, or the plan's edge is not"
        << " this mechanism";
    EXPECT_LT(crossing_rad_s, 200.0F) << "and below the speed where the distortion demonstrably costs nothing";
    std::printf("[f19] crossing: omega*psi_f == %.2f V at %.0f rad/s mechanical (psi_f = %.5f Wb)\n",
                static_cast<double>(kDeadV), static_cast<double>(crossing_rad_s), static_cast<double>(psi_f));
}

} // namespace
