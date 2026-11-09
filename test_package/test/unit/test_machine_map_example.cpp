// `machine_map`, assembled: a text file becomes a motor, and the current the table did *not* measure is visible as such.
//
// The first test is the example the batch asked for -- twenty-odd lines from a document to a running `Motor`. The rest
// are the criteria, written before the run:
//
//   1. the example is real: the document's `rs` and its rows reach the motor, and the drive ticks;
//   2. a current past the last row answers with the last row **exactly**, and `covers()` says so -- the clamp is
//      askable, because a clamped answer is arithmetically indistinguishable from a measurement;
//   3. the extrapolation a naive implementation would produce is computed here and differs, so this test cannot pass by
//      clamping and extrapolating giving the same number;
//   4. the map is even in current: a negative current is the same point as its magnitude.
//
// 第一个用例就是批次要的那个示例——从文档到一台跑起来的 `Motor` 二十来行。其余是判据，写在跑前：示例是真的（文档的
// `rs` 与行确实到了电机上，且驱动器真的在 tick）；超出最后一行的电流 **恰好** 用最后一行作答且 `covers()` 说出来；
// 朴素实现会给出的外推值在这里被算出来并要求与钳位值不同，于是这条不可能靠"钳位与外推恰好同值"而通过；表对电流是偶
// 函数。

#include <cstring>

#include <gtest/gtest.h>

#include <foc/core/machine_map.hpp>
#include <foc/core/machine_map_text.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/motor/motor.hpp>

namespace {

namespace core = foc::core;
namespace motor = foc::motor;
namespace driver = foc::driver;

using foc::ErrorCode;

// A three-row document: the same machine at 0 A, 4 A and 8 A. `ld` collapses with current, which is the whole reason
// the table exists.
constexpr char kDocument[] =
    "format=1\n"
    "rs=0.5\n"
    "row=0,4e-4,4e-4,0.05\n"
    "row=4,2.4e-4,4e-4,0.05\n"
    "row=8,1.6e-4,4e-4,0.05\n";

constexpr float kRs = 0.5F;
constexpr float kPolePairs = 7.0F;
constexpr float kDt = 1e-4F;

// --- the example: text -> document -> map -> Motor -------------------------------------------------------------
TEST(MachineMapExample, ATextFileBecomesAMotor) {
    core::MachineDocument document{};
    core::TextError error{};
    ASSERT_TRUE(core::read_document(kDocument, std::strlen(kDocument), &document, &error))
        << "line " << error.line << ", status " << static_cast<int>(error.status);
    ASSERT_EQ(document.map.count, 3U);

    // The nameplate the document carries, as a motor the library can turn.
    const core::MachinePoint at_zero = document.map.at(0.0F);
    const motor::MotorConfig config{document.params.rs, at_zero.ld_h, at_zero.lq_h,
                                    1.5F * kPolePairs * at_zero.psi_f_wb, 1e-5F, 1e-6F,
                                    static_cast<std::uint8_t>(kPolePairs), 5.0F, 20000.0F, 500.0F};
    const core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const core::Pi::Config speed{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F};
    using Motor = motor::Motor<core::BackEmfObserver, core::Svpwm, core::Pi, driver::StubHal>;
    const Motor::Policies policies{core::BackEmfObserver({kRs, 4e-4F}), core::Svpwm{}, core::Pi{current},
                                   core::Pi{current}, core::Pi{speed}};

    driver::StubHal hal;
    hal.dc_link.set_voltage_v(24.0F);
    hal.position.set_sensor_present(true);
    const core::Uvw<float> zero{};
    ASSERT_EQ(ErrorCode::Ok, hal.current_sense.load(&zero, 1U));

    Motor drive(config, policies, hal);
    ASSERT_EQ(ErrorCode::Ok, drive.init());
    ASSERT_EQ(ErrorCode::Ok, drive.start());

    // At each operating point the map is what gets handed over -- that is the assembly.
    for (std::size_t row = 0U; row < document.map.count; ++row) {
        drive.apply_params(document.map.at(document.map.rows[row].i_a).with_rs(kRs));
        ASSERT_EQ(ErrorCode::Ok, drive.tick(kDt)) << "row " << row;
    }
    EXPECT_EQ(3U, hal.pwm.write_count()) << "the drive ran three periods, one per row";
}

// --- criterion 1, the part that has teeth: the numbers really arrive ---------------------------------------------
TEST(MachineMapExample, TheDocumentsNumbersReachTheObserver) {
    // Two drives built identically; one is told what the document says, the other is told a machine with ten times the
    // q-axis inductance. Then each observer is fed the *same* voltages and currents. If `apply_params` forwarded
    // nothing -- which is what this test found when it was first written: the declaration had no definition anywhere in
    // the tree, and nothing called it, so it linked only because nobody asked -- the two estimates would be equal.
    // 两台完全一样搭起来的驱动器；一台被告知文档描述的那台机器，另一台被告知的机器 q 轴电感是它的十倍。然后给两个观测器
    // 喂 **相同** 的电压与电流。如果 `apply_params` 什么都没转发——这正是这条用例第一次写出来时发现的：那个声明在整棵树里
    // 没有定义，也没有人调用，于是它只因为没人问起才"链接得过"——两次估计就会相等。
    core::MachineDocument document{};
    ASSERT_TRUE(core::read_document(kDocument, std::strlen(kDocument), &document, nullptr));

    const core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const core::Pi::Config speed{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F};
    using Motor = motor::Motor<core::BackEmfObserver, core::Svpwm, core::Pi, driver::StubHal>;
    const motor::MotorConfig config{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-5F, 1e-6F, 7U, 5.0F, 20000.0F, 500.0F};
    auto policies = [&](float lq_h) {
        return Motor::Policies{core::BackEmfObserver({0.5F, lq_h}), core::Svpwm{}, core::Pi{current},
                               core::Pi{current}, core::Pi{speed}};
    };
    driver::StubHal hal_a;
    driver::StubHal hal_b;
    Motor informed(config, policies(4e-4F), hal_a);
    Motor misled(config, policies(4e-3F), hal_b);

    informed.apply_params(document.map.at(8.0F).with_rs(kRs));
    // The same current, but a machine ten times more inductive in q: the estimate it produces is a different machine's.
    const core::MachinePoint wrong = document.map.at(8.0F);
    misled.apply_params(core::MachineParams{kRs, wrong.ld_h, 10.0F * wrong.lq_h, wrong.psi_f_wb});

    // The observer subtracts `L * di`, so the current has to move for the inductance to show up: the first update only
    // primes its previous-current register. Both get the identical inputs.
    // 观测器减掉的是 `L·di`，所以电流必须发生变化电感才显现：第一次 update 只是把"上一次电流"填上。两台喂的输入完全相同。
    const core::Ab<float> v{1.0F, 0.5F};
    const core::Ab<float> zero{};
    const core::Ab<float> i{0.7F, -0.3F};
    static_cast<void>(informed.observer().update(v, zero, kDt));
    static_cast<void>(misled.observer().update(v, zero, kDt));
    const core::ObsState a = informed.observer().update(v, i, kDt);
    const core::ObsState b = misled.observer().update(v, i, kDt);

    EXPECT_NE(a.theta_e, b.theta_e) << "the updated parameters must change what the observer computes";
    // And by exactly the amount the two inductances differ, applied to the current step: the difference is not "some
    // change", it is `(L_misled - L_informed) * di` on the alpha axis.
    // 而且恰好是两个电感之差作用在电流变化上：这个差不是"有变化"，它是 alpha 轴上的 `(L_误导 − L_告知)·di`。
    const float expected = ((10.0F * wrong.lq_h) - wrong.lq_h) * i.alpha;
    EXPECT_NEAR(informed.observer().flux().alpha - misled.observer().flux().alpha, expected, 1e-9F);
}

// --- criterion 2 and 3: the clamp is exact, and it is not an extrapolation --------------------------------------
TEST(MachineMapExample, PastTheLastRowTheAnswerIsTheLastRowAndSaysSo) {
    core::MachineDocument document{};
    ASSERT_TRUE(core::read_document(kDocument, std::strlen(kDocument), &document, nullptr));

    const core::MachinePoint last = document.map.at(8.0F);
    const core::MachinePoint beyond = document.map.at(40.0F);
    EXPECT_FLOAT_EQ(last.ld_h, beyond.ld_h);
    EXPECT_FLOAT_EQ(last.lq_h, beyond.lq_h);
    EXPECT_FLOAT_EQ(last.psi_f_wb, beyond.psi_f_wb);

    // What a table that extrapolated the last segment would answer at 40 A. It has to be a different number, or the
    // clamp cannot be told apart from extrapolation and this test proves nothing.
    const float slope = (document.map.rows[2].ld_h - document.map.rows[1].ld_h) /
                        (document.map.rows[2].i_a - document.map.rows[1].i_a);
    const float extrapolated = document.map.rows[2].ld_h + (slope * (40.0F - document.map.rows[2].i_a));
    EXPECT_NE(extrapolated, beyond.ld_h) << "40 A must discriminate clamping from extrapolation";
    EXPECT_LT(extrapolated, 0.0F) << "the extrapolation even leaves the physical range, which is the point";

    // And the clamp is askable, which is what makes it observable rather than silent.
    EXPECT_TRUE(document.map.covers(0.0F));
    EXPECT_TRUE(document.map.covers(8.0F));
    EXPECT_FALSE(document.map.covers(8.001F));
    EXPECT_FALSE(document.map.covers(40.0F));
}

// --- criterion 4: the map is even in current -------------------------------------------------------------------
TEST(MachineMapExample, TheMapIsEvenInCurrent) {
    core::MachineDocument document{};
    ASSERT_TRUE(core::read_document(kDocument, std::strlen(kDocument), &document, nullptr));
    const core::MachinePoint positive = document.map.at(2.0F);
    const core::MachinePoint negative = document.map.at(-2.0F);
    EXPECT_FLOAT_EQ(positive.ld_h, negative.ld_h);
    EXPECT_FLOAT_EQ(positive.lq_h, negative.lq_h);
    EXPECT_FLOAT_EQ(positive.psi_f_wb, negative.psi_f_wb);
}

// --- an invalid table answers nothing, and says so through the same question -------------------------------------------------
TEST(MachineMapExample, AnInvalidTableCoversNothing) {
    core::MachineMap map{};
    EXPECT_FALSE(map.covers(0.0F)) << "an empty map covers no current at all";
    EXPECT_FALSE(map.valid());
}

} // namespace
