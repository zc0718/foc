// SOP, second system (F19): replay the procedure on the sensorless floor. The question the plan asks is whether the
// no-angle edge below 50 rad/s can be delivered as a *procedure* -- identify, propose, measure, refuse -- the way the
// outer loop's could. The answer this file measures is **no**, and it says exactly which part fails.
// SOP 的第二个系统（F19）：把流程在无感地板上重放一遍。计划问的是"50 rad/s 以下没有角度"那条边缘能不能像外环那样作为
// **流程** 交付——辨识、提议、测量、拒绝。本文件量出来的答案是 **不能** ，并指出是哪一步失败。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include "sil_sensorless_bench.hpp"

namespace {

namespace sil = foc::sil;

constexpr float kDeadV = 0.48F; ///< [en] 1 us of dead time at 24 V. [zh] 24 V 下 1 µs 死区。

/** @brief [en] Machine A: the nameplate as it is. [zh] 机器 A：原样铭牌。 */
constexpr sil::SopMachine kMachineA{1.0F};
/** @brief [en] Machine B: the same machine with a 25 % stronger magnet. [zh] 机器 B：同一台机器，磁钢强 25 %。 */
constexpr sil::SopMachine kMachineB{1.25F};

/**
 * @brief [en] The speed at which the dead-time voltage equals the back-EMF, `V_dead / psi_f`.
 * @brief [zh] 死区电压与反电势相等的那个转速， `V_dead / psi_f`。
 * @param machine [en] which machine.
 * @param machine [zh] 哪台机器。
 * @param dead_v [en] per-tick dead-time voltage, V.
 * @param dead_v [zh] 每拍死区电压，V。
 * @return [en] the crossing, rad/s mechanical.
 * @return [zh] 交点，机械 rad/s。
 * @note [en] This is the **prediction** the F19 row carries, and the point of this file is that it is a lower bound
 *            rather than the answer: it says where the distortion becomes comparable, not where an angle becomes
 *            usable.
 * @note [zh] 这就是 F19 那一行携带的 **预测** ，而本文件的要点是：它是下界而不是答案——它说的是畸变什么时候变得可比，而
 *            不是角度什么时候变得可用。
 */
auto predicted_crossing(const sil::SopMachine& machine, float dead_v) -> float {
    return dead_v / foc::psi_f_wb(sil::sop_motor_config(machine));
}

} // namespace

/**
 * @brief [en] Stage 1 identifies a **constant**, and the boundary the row predicts is not the boundary the measurement
 *             finds: the settled-window error is **not monotone in speed**.
 *
 * @brief [zh] 第 1 步辨识出一个 **常数** ，而那一行预测的边界不是测量找到的边界：稳态窗口内的误差 **对转速不单调** 。
 *
 *   Two facts, both measured on machine A, and the second one is the reason this procedure cannot be delivered.
 *
 *   **The identification is clean.** With an ideal inverter the angle error is exactly one tick of transport delay:
 *   `error/omega_mech` = 3.77e-4 s at 10 rad/s and 3.80e-4 s at 40, i.e. a constant to 1 %, and it is a delay (multiply
 *   by the 7 pole pairs and divide by the 50 us tick to get 54 control periods of electrical angle per second of lag).
 *   That is a real, identifiable quantity and stage 1 succeeds.
 *
 *   **The measurement refuses to be a criterion.** With 1 us of dead time the worst error over the settled window is
 *   0.1591 / 0.0679 / 0.0835 / 0.0757 / 0.0410 / 0.0898 rad at 60 / 80 / 90 / 120 / 200 / 300 rad/s: it
 *   **goes down and then up again**. What the window measures is the amplitude of a lightly damped oscillation, and an
 *   amplitude has no monotone relation to speed, so "how far is this from failing" is not a function of speed here and
 *   the "smallest speed at which an angle may be claimed" does not exist as an interval. The 101 rad/s the row predicts
 *   is a **lower bound**: the first speed that passes the 0.05 rad gate at all is 200 rad/s, roughly twice it.
 *
 *   两条事实，都在机器 A 上测，而第二条正是这条流程交付不了的原因。
 *
 *   **辨识是干净的。** 理想逆变下角度误差正好是一拍传输延时：10 rad/s 处 `误差/机械转速` = 3.77e-4 s、40 rad/s 处
 *   3.80e-4 s，即 1 % 以内的一个常数，而且它是延时（乘 7 对极、除 50 µs 一拍，得到每秒钟滞后折合 54 个控制周期的电角
 *   度）。那是一个真实、可辨识的量，第 1 步成功。
 *
 *   **测量拒绝成为判据。** 1 µs 死区下，稳态窗口内的最坏误差在 60 / 80 / 90 / 120 / 200 / 300 rad/s 处分别是 0.1591 /
 *   0.0679 / 0.0835 / 0.0757 / 0.0410 / 0.0898 rad： **先降后升** 。窗口量到的是一个轻阻尼振荡的幅值，而幅值与转速没有
 *   单调关系，于是"离失败多远"在这里不是转速的函数，"可以声称角度的最低转速"也不作为一个区间存在。那一行预测的 101
 *   rad/s 是一个 **下界** ：真正通过 0.05 rad 那道闸的第一个转速是 200 rad/s，大约它的两倍。
 */
TEST(SopSensorlessFloor, TheDelayIsIdentifiedButTheBoundaryIsNotAMonotoneCriterion) {
    // Stage 1: the transport delay, on an ideal inverter.
    // 第 1 步：传输延时，理想逆变。
    const sil::SensorlessRow ideal10 = sil::run_sensorless(10.0F, 0.0F, 0.0F, kMachineA);
    const sil::SensorlessRow ideal40 = sil::run_sensorless(40.0F, 0.0F, 0.0F, kMachineA);
    const float ratio10 = ideal10.max_error_rad / ideal10.true_speed_rad_s;
    const float ratio40 = ideal40.max_error_rad / ideal40.true_speed_rad_s;
    RecordProperty("delay_ratio_10", ratio10);
    RecordProperty("delay_ratio_40", ratio40);
    ASSERT_FALSE(ideal10.faulted);
    ASSERT_FALSE(ideal40.faulted);
    EXPECT_NEAR(ratio10, ratio40, 0.10F * ratio10) << "a delay is a constant ratio, not a floor";
    EXPECT_NEAR(3.8e-4F, ratio10, 0.10F * 3.8e-4F) << "and it is the 3.8e-4 s the F19 row already recorded";

    // Stage 3: the scan. What it produces is not the shape a criterion needs.
    // 第 3 步：扫描。它产出的是判据需要的那种形状以外的东西。
    const float speeds[] = {60.0F, 80.0F, 90.0F, 120.0F, 200.0F, 300.0F};
    float errors[6] = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F};
    float first_passing = 0.0F;
    for (std::size_t i = 0; i < 6U; ++i) {
        const sil::SensorlessRow row = sil::run_sensorless(speeds[i], 0.0F, kDeadV, kMachineA);
        errors[i] = row.max_error_rad;
        if ((first_passing == 0.0F) && !row.faulted && (row.max_error_rad < sil::kSopGateRad)) {
            first_passing = speeds[i];
        }
    }
    RecordProperty("first_passing_rad_s", first_passing);
    EXPECT_GT(first_passing, 0.0F) << "at least one speed has to pass, or the scan measured nothing";

    // The two teeth. Neither is a tolerance: each is a violation of the shape a criterion would need.
    // 两颗牙齿。都不是容差，而是"判据所需要的那种形状"被破坏的证据。
    EXPECT_LT(errors[1], errors[0]) << "80 rad/s is quieter than 60 (0.0679 against 0.1591)";
    EXPECT_GT(errors[2], errors[1]) << "and 90 is louder than 80 again (0.0835 against 0.0679) -- not monotone";
    EXPECT_GT(errors[4], 0.0F);
    EXPECT_LT(errors[4], errors[3]) << "200 is the quiet point of the scan, and it is surrounded by louder ones";

    // The prediction is a lower bound, and the case says how much of one.
    // 预测是一个下界，而用例说出了它下到什么程度。
    const float crossing = predicted_crossing(kMachineA, kDeadV);
    RecordProperty("predicted_crossing_rad_s", crossing);
    std::printf("[sop-f19] predicted crossing %.1f rad/s, first passing speed %.0f rad/s (ratio %.2f)\n",
                static_cast<double>(crossing), static_cast<double>(first_passing),
                static_cast<double>(first_passing / crossing));
    EXPECT_LT(crossing, first_passing) << "the closed form names where the distortion becomes comparable, not where the"
                                       << " angle becomes usable";
    EXPECT_GT(first_passing / crossing, 1.5F) << "and on this rig it is off by more than half";
}

/**
 * @brief [en] Stage 2 on a **second machine**: a stronger magnet must move the predicted boundary **down** -- and the
 *             measurement moves the other way. The prediction is refuted, and the case keeps the numbers.
 *
 * @brief [zh] 第 2 步换一台 **机器** ：更强的磁钢应当把预测边界往 **下** 推——而测量往反方向走。预测被否证，用例把数字留
 *             下。
 *
 *   Machine B is the same machine with a 25 % stronger magnet, described correctly to the drive (plant and model
 *   alike), so this is a comparison of two machines and not of two descriptions. `psi_f` rises 25 %, so `V_dead/psi_f`
 *   falls from **100.8** to **80.6 rad/s**: at 90 rad/s machine B should be on the safe side of its own boundary while
 *   machine A is not.
 *
 *   Measured, machine B is **worse at every speed tried**: 0.1738 against 0.0835 rad at 90, 0.1140 against 0.0757 at
 *   120, 0.0618 against 0.0410 at 200. So `V_dead/psi_f` is not the law: `psi_f` also moves the observer's model, the
 *   back-EMF the loop has to read, and the gains the speed loop holds the machine with, and those terms win. That is a
 *   **refuted hypothesis**, recorded rather than quietly dropped -- and it is the second reason the F19 procedure
 *   cannot be delivered as a criterion: its one closed form does not survive a change of machine.
 *
 *   机器 B 是同一台机器换 25 % 更强的磁钢，并且被正确地描述给驱动器（plant 与模型同时缩放），所以这是两台机器之间的比
 *   较，不是两份描述之间的比较。 `psi_f` 升 25 %，于是 `V_dead/psi_f` 从 **100.8** 落到 **80.6 rad/s** ：在 90 rad/s 处
 *   机器 B 本该站在自己边界的安全侧，而机器 A 不是。
 *
 *   实测机器 B **在每个试过的转速上都更差** ：90 rad/s 处 0.1738 对 0.0835、120 处 0.1140 对 0.0757、200 处 0.0618 对
 *   0.0410。所以 `V_dead/psi_f` 不是那条律： `psi_f` 同时移动观测器的模型、环路要读的反电势、以及速度环拿住机器的增益，
 *   而那些项赢了。这是一个 **被否证的假设** ，记录下来而不是悄悄丢掉——它也是 F19 这条流程不能作为判据交付的第二个原因：
 *   它唯一的闭式解经不起换一台机器。
 */
TEST(SopSensorlessFloor, AStrongerMagnetMovesThePredictionDownAndTheMeasurementUp) {
    const float crossing_a = predicted_crossing(kMachineA, kDeadV);
    const float crossing_b = predicted_crossing(kMachineB, kDeadV);
    RecordProperty("crossing_a", crossing_a);
    RecordProperty("crossing_b", crossing_b);
    std::printf("[sop-f19] crossing: psi x1.00 -> %.1f rad/s, psi x1.25 -> %.1f rad/s\n",
                static_cast<double>(crossing_a), static_cast<double>(crossing_b));

    // The prediction moves down by exactly the magnet's factor -- this half is arithmetic and it holds.
    // 预测正好按磁钢的倍数下移——这一半是算术，成立。
    EXPECT_NEAR(crossing_a / crossing_b, 1.25F, 0.01F) << "V_dead/psi_f scales as 1/psi_f";

    // The measurement moves up. Three speeds, each a separate chance for the prediction to be right.
    // 测量往上走。三个转速，每一个都是预测本可以成立的机会。
    const float speeds[] = {90.0F, 120.0F, 200.0F};
    for (const float speed : speeds) {
        const sil::SensorlessRow a = sil::run_sensorless(speed, 0.0F, kDeadV, kMachineA);
        const sil::SensorlessRow b = sil::run_sensorless(speed, 0.0F, kDeadV, kMachineB);
        std::printf("[sop-f19] %5.0f rad/s: psi x1.00 %.4f rad, psi x1.25 %.4f rad\n", static_cast<double>(speed),
                    static_cast<double>(a.max_error_rad), static_cast<double>(b.max_error_rad));
        EXPECT_GT(b.max_error_rad, a.max_error_rad)
            << "at " << speed << " rad/s a stronger magnet made the estimate worse, so the crossing is not the law";
    }
    // And 90 rad/s is the speed where the two predictions disagree about the answer: A says refuse (90 < 100.8), B says
    // claim (90 > 80.6). The measurement says B is the worse of the two, which is the opposite of what the closed form
    // promised -- so no minimum speed can be derived from it.
    // 而 90 rad/s 正是两个预测给出不同答案的转速：A 说拒绝（90 < 100.8），B 说可声称（90 > 80.6）。测量说 B 是两者中更差的那个，
    // 与那个闭式的承诺正好相反——于是推不出任何最低转速。
    const sil::SensorlessRow a90 = sil::run_sensorless(90.0F, 0.0F, kDeadV, kMachineA);
    const sil::SensorlessRow b90 = sil::run_sensorless(90.0F, 0.0F, kDeadV, kMachineB);
    EXPECT_LT(crossing_b, 90.0F) << "the prediction puts B on the safe side at 90 rad/s";
    EXPECT_GT(crossing_a, 90.0F) << "and A on the unsafe side";
    EXPECT_GE(b90.max_error_rad, a90.max_error_rad) << "the machine predicted to be safer is the worse one";
}
