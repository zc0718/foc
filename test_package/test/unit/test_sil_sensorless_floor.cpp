// F19 - the upper bound: on an **ideal** plant, is a closed-loop angle available below 50 rad/s?
//
// The plan asks for a *disproof* rather than a claim: "no closed-loop angle below 50 rad/s, measured on an ideal plant
// (known psi_f, no saliency) -- if it cannot be done there it cannot be done on hardware". The reasoning is structural,
// not empirical: an SPM has no saliency, so the injection observer has nothing to inject into (`predicted_gain_a` is
// proportional to `lq - ld`), which removes the only mechanism that works at standstill and leaves a model observer whose
// accuracy grows with speed. This file measures where that leaves the drive instead of asserting the sentence.
//
// F19 —— 上界：在 **理想** plant 上，50 rad/s 以下到底有没有闭环角度？
//
// 计划要的是一条 **否证** 而不是一句声称："50 rad/s 以下没有闭环角度，在理想 plant（已知 ψf、无凸极）上量——那里做不到，
// 真机就更做不到。" 理由是结构性的，不是经验性的：SPM 没有凸极，注入观测器没有东西可注（`predicted_gain_a` 正比于
// `lq − ld`），于是唯一能在静止工作的机理被排除，只剩一个精度随转速增长的模型观测器。本文件去 **量** 它到底停在哪里，
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
//   * 传感器撤掉时转子 **已经在转** （`PlantState` 里带着转速）：要问的是"从一次升速中能不能捕获并保持"，而"没有注入能不能
//     从静止起动"是另一个问题，已由 `test_sil_alignment.cpp` 与开环强拖回答；
//   * 观测器从零种子开始，即最大程度地错，这是诚实的下界，也与既有那条 90° 种子实验同一个起点；
//   * 被测的量是"1.5 s 运行的 **最后 0.5 s**"内电角度对 plant 真值的误差——是"稳下来了吗"的窗口，不是捕获瞬态。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include "sil_sensorless_bench.hpp"

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

// The rig moved to `sil_sensorless_bench.hpp` so that the SOP replay can drive **the same** measurement on a second
// machine: one rig, two questions. The defaults there reproduce this file's numbers exactly -- the twelve `[f19]` lines
// this test prints were diffed before and after the move and are identical.
// 台架搬到 `sil_sensorless_bench.hpp`，好让 SOP 重放能在第二台机器上驱动 **同一个** 测量：一份台架，两个问题。那边的默认值
// 逐字复现本文件的数字——本用例打印的十二行 `[f19]` 在搬动前后逐行 diff 过，完全相同。
constexpr float kEdgeRadS = 50.0F; // the edge the plan names 计划点名的边缘
constexpr float kGateRad = sil::kSopGateRad;
const foc::sil::SopMachine kMachine{}; // machine A: the nameplate as it is 机器 A：原样铭牌

/** @brief [en] One speed, on machine A. [zh] 一个转速，机器 A。 */
auto run_at(float target_rad_s, float load_nm, float dead_v) -> sil::SensorlessRow {
    const sil::SensorlessRow row = sil::run_sensorless(target_rad_s, load_nm, dead_v, kMachine, /*quiet=*/true);
    std::printf("[f19] target %6.1f rad/s load %5.3f Nm dead %4.2f V -> max %.4f rad, end %.4f rad, true w %7.2f, "
                "state %d\n",
                static_cast<double>(row.target_rad_s), static_cast<double>(row.load_nm), static_cast<double>(dead_v),
                static_cast<double>(row.max_error_rad), static_cast<double>(row.end_error_rad),
                static_cast<double>(row.true_speed_rad_s), row.faulted ? 4 : 2);
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
// 计划以为"50 rad/s 以下做不到，而理想 plant 正是证明它的地方"。量出来恰好相反：理想 plant 是它 **做得到** 的那个地方——
// `lq == ld`、无死区、无传感器噪声、观测器拿到的就是 plant 自己的参数，5 rad/s 处误差 2 mrad。剩下的 **只有传输延时** ，而它
// 表现为一个恒定比值而不是地板：5 到 354 rad/s 之间 `误差/机械转速` 都是 3.8e-4 s，即 **每个电周期 1.1 个控制周期**
// ——误差是电角度而转速是机械的，所以要把 7 对极除出来（3.8e-4 × 7 = 2.7 ms/s，对 50 µs 的一拍）。观测器报出的是 **上一拍**
// 的角度，这就是 plant 理想之后剩下的全部。
TEST(SilSensorlessFloor, AnIdealPlantHasNoLowSpeedFloorAndWhatIsLeftIsOneTickOfDelay) {
    const float targets[] = {5.0F, 10.0F, 20.0F, 40.0F};
    float ratios[4] = {0.0F, 0.0F, 0.0F, 0.0F};
    for (std::size_t i = 0; i < 4U; ++i) {
        const sil::SensorlessRow run = run_at(targets[i], 0.0F, 0.0F);
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
    const float ticks = ratios[0] * static_cast<float>(sil::kSopPoles) / sil::kSopDt;
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
// 处同一个畸变 **什么都不花** （0.041 对干净的 0.073——甚至更好，因为那里 `ω·ψf` = 0.95 V 是畸变的两倍）。所以边界不是观测器
// 拥有的某个转速，而是"反电势不再小于逆变量自身误差"的那个转速，本机约 100 rad/s——"50 rad/s 以下不作声称"说的其实是这件事。
TEST(SilSensorlessFloor, TheFloorIsTheDeadTimeVoltageAgainstTheBackEmfAndNotTheObserversStructure) {
    constexpr float kDeadV = 0.48F; // 1 us of dead time at 24 V 24 V 下 1 µs 死区

    const sil::SensorlessRow ideal20 = run_at(20.0F, 0.0F, 0.0F);
    const sil::SensorlessRow dead20 = run_at(20.0F, 0.0F, kDeadV);
    const sil::SensorlessRow ideal40 = run_at(40.0F, 0.0F, 0.0F);
    const sil::SensorlessRow dead40 = run_at(40.0F, 0.0F, kDeadV);
    const sil::SensorlessRow ideal200 = run_at(200.0F, 0.0F, 0.0F);
    const sil::SensorlessRow dead200 = run_at(200.0F, 0.0F, kDeadV);

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
    // 交点本身作为 **预测** 而不是观测：`ω·ψf == kDeadV` 给出两者可比的那个转速。用例断言的是 **量级** （100 rad/s 处比值约 1），
    // 而不是一个调出来的常数，于是 ψf 变化时预测随机器一起移动，而不是让一个常数失效。
    const float psi_f = foc::psi_f_wb(sil::sop_motor_config(kMachine));
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
