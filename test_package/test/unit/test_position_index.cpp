// W7-a, increment 2: the index mark as an optional HAL capability.
//
// The mark is what turns a continuous angle into an absolute one, so what matters about it is not that it exists but
// that it behaves like an edge and like a *physical* feature: one pulse per revolution, none when the rotor is still, and
// independent of whatever correction offset the drive happens to be applying.
//
// W7-a 增量 2：索引标记作为 HAL 的可选能力。
// 标记是把连续角度变成绝对角度的东西，因此要紧的不是"它存在"，而是它表现得像**边沿**、像**物理特征**：
// 每圈恰好一个脉冲、转子静止时没有脉冲、并且与驱动器当时应用的校正偏置无关。
//
// The criterion, fixed before the numbers below were measured:
//   C1  detection is by capability, not assumption: a sensor that offers the pair reports it, one that does not is
//       reported as not offering it. A library that required the mark would make every sensor carry one; a library that
//       assumed it would read garbage from sensors that never had one.
//   C2  exactly one pulse per mechanical revolution over several revolutions -- not one per tick, and not one per turn
//       only sometimes.
//   C3  a rotor that is not moving produces no pulses, which is what separates an edge from a level.
//   C4  the mark does not move when the correction offset does: it is a property of the rotor, so a pulse arrives at the
//       same truth angle before and after the offset is changed.
//
// 判据在量之前定下：
//   C1 靠能力探测、不靠假设：提供这一对的传感器报"有"，没提供的报"没有"。要求它的库会迫使每个传感器都带一个标记；
//      假设它的库会从根本没有标记的传感器上读出垃圾。
//   C2 数个机械圈内每圈恰好一个脉冲——不是每 tick 一个，也不是"只有时候"每圈一个。
//   C3 不动的转子不产生脉冲，这是边沿与电平的分界。
//   C4 校正偏置改变时标记不移动：它是转子的属性，因此改变偏置前后脉冲都出现在同一个真值角度上。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/traits.hpp>
#include <foc/driver/sim_position.hpp>
#include <foc/motor/motor.hpp>

#include "sil_pmsm_plant.hpp"
#include "sil_plant_hal.hpp"

namespace {

namespace sil = foc::sil;

constexpr float kTwoPi = 6.28318530717959F;

auto nominal_spec_params() -> sil::PlantParams {
    return sil::PlantParams{0.35F, 2.0e-4F, 4.0e-4F, 0.05F, 1.0e-5F, 1.0e-3F, 7U};
}

/** @brief [en] Put the rotor at a mechanical angle without any electrical activity. */
/** @brief [zh] 不产生任何电气活动地把转子放到某个机械角度。 */
void place_rotor(sil::PmsmPlant& plant, float theta_mech_rad) {
    plant.reset(sil::PlantState{0.0F, 0.0F, 0.0F, theta_mech_rad});
}

} // namespace

TEST(PositionIndex, TheMarkIsAnOptionalCapabilityRatherThanARequirement) {
    // C1
    static_assert(foc::core::concepts::has_index_v<sil::PlantPosition>,
                  "the SIL sensor offers the pair, so the concept must find it");
    static_assert(!foc::core::concepts::has_index_v<foc::driver::SimPosition>,
                  "SimPosition does not offer the pair, so the concept must say so rather than assume one");
    SUCCEED();
}

TEST(PositionIndex, OnePulsePerRevolutionAndNoneWhenTheRotorIsStill) {
    // C2 and C3
    sil::PmsmPlant plant(nominal_spec_params());
    sil::PlantPosition position(plant);
    position.set_index_position_mech_rad(1.0F); // an arbitrary mark position, deliberately not zero

    // Three revolutions in small steps, plus one revolution held still.
    // 三圈小步前进，再静止一整段。
    constexpr int kStepsPerTurn = 720;
    const float step = kTwoPi / static_cast<float>(kStepsPerTurn);
    int pulses = 0;
    for (int i = 0; i < (3 * kStepsPerTurn); ++i) {
        place_rotor(plant, step * static_cast<float>(i));
        if (position.index_pulse()) {
            ++pulses;
        }
    }
    const int after_rotation = pulses;
    for (int i = 0; i < kStepsPerTurn; ++i) {
        if (position.index_pulse()) {
            ++pulses; // the rotor has not moved: any pulse here is a level masquerading as an edge
        }
    }
    std::printf("\n[index] three revolutions produced %d pulse(s); a still rotor added %d\n", after_rotation,
                pulses - after_rotation);
    EXPECT_EQ(3, after_rotation) << "the pulse is not one-per-revolution";
    EXPECT_EQ(after_rotation, pulses) << "a rotor that is not moving produced a pulse";
}

TEST(PositionIndex, TheMarkStaysOnTheRotorWhenTheOffsetChanges) {
    // C4: the same truth angle must produce the pulse before and after a correction offset is written.
    // C4：同一个真值角度，在写入校正偏置前后都必须产生脉冲。
    constexpr float kMark = 0.7F;
    const auto pulses_at_mark = [](bool with_offset) {
        sil::PmsmPlant plant(nominal_spec_params());
        sil::PlantPosition position(plant);
        if (with_offset) {
            position.set_offset_mech_rad(0.31F);
        }
        position.set_index_position_mech_rad(kMark);
        // Approach the mark from below in small steps and report how many pulses were seen.
        // 从小步从下方逼近标记，报出看到了几个脉冲。
        int seen = 0;
        for (int i = 0; i <= 200; ++i) {
            const float theta = kMark - 0.5F + (0.005F * static_cast<float>(i));
            place_rotor(plant, theta);
            if (position.index_pulse()) {
                ++seen;
            }
        }
        return seen;
    };

    const int without_offset = pulses_at_mark(false);
    const int with_offset = pulses_at_mark(true);
    std::printf("[index] pulses crossing the mark: %d without offset, %d with offset\n", without_offset, with_offset);
    EXPECT_EQ(1, without_offset) << "crossing the mark once must pulse once";
    EXPECT_EQ(without_offset, with_offset) << "the mark moved when the correction offset changed";
}

namespace {

using PositionMotor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, sil::PlantHal,
                                        foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                        foc::core::PositionP, foc::core::NullAlignment>;

} // namespace

TEST(PositionIndex, APositionCommandNeedsAReferenceRatherThanASensor) {
    // Increment 3's claim, and only that claim: the *admission* of a position command changes from "a sensor is present"
    // to "an absolute angle exists". Whether the loop then tracks is a different question, measured on the fusion-chain
    // rig where a real sensor-less rotation exists; asserting it here would need a rotation this rig does not produce,
    // and the rotor is placed by hand precisely so that the reference and the mark can be driven independently.
    // 增量 3 的主张，且仅是这一条：位置指令的**准入**从"有传感器"变为"有绝对角度"。闭环随后是否跟踪是另一个问题，
    // 在有真实无感旋转的融合链路台架上测量；在这里断言它需要本台架产生不了的旋转，而转子刻意是手工摆放的，正是为了让
    // 参考与标记能被独立驱动。
    sil::PmsmPlant plant(nominal_spec_params());
    sil::PlantHal hal(plant, 24.0F);
    place_rotor(plant, 0.0F);
    hal.position.set_sensor_present(false);

    const foc::motor::MotorConfig cfg{0.35F, 2.0e-4F, 4.0e-4F, 0.05F, 1.0e-5F, 1.0e-3F, 7U, 3.0F, 20000.0F, 0.0F, 0.0F,
                                      0.0F};
    PositionMotor motor(cfg,
                        PositionMotor::Policies{
                            foc::core::BackEmfObserver{foc::core::BackEmfObserver::Config{cfg.rs, cfg.ld}},
                            foc::core::Svpwm{},
                            foc::core::Pi{foc::current_pi_config(cfg, 2000.0F, 24.0F)},
                            foc::core::Pi{foc::current_pi_config(cfg, 2000.0F, 24.0F)},
                            foc::core::Pi{foc::speed_pi_config(cfg, 200.0F)},
                            foc::core::NullReferenceGenerator{}, foc::core::NullFeedforward{},
                            foc::core::PositionP{foc::position_p_config(cfg, 20.0F, 50.0F, 0.0F)},
                            foc::core::NullAlignment{}},
                        hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    ASSERT_EQ(foc::core::MotorState::Run, motor.state());

    motor.set_position(0.0F);
    EXPECT_EQ(foc::ErrorCode::NotImpl, motor.tick(5.0e-5F))
        << "without a mark there is no absolute angle, so the command must be refused rather than closed on zero";
    EXPECT_FALSE(motor.position_reference().homed());

    // Install the mark and walk the rotor across it. The observer is not producing a rotation here, so what this
    // establishes is the origin, not a tractable angle -- see the note above.
    // 装上一个标记并把转子移过它。此处观测器没有产生旋转，因此这一步建立的是**原点**，不是可用的角度——见上方说明。
    hal.position.set_index_position_mech_rad(0.5F);
    for (int i = 0; i <= 200; ++i) {
        place_rotor(plant, 0.0F + (0.005F * static_cast<float>(i)));
        static_cast<void>(motor.tick(5.0e-5F));
    }
    std::printf("\n[position] after crossing the mark: homed=%d lost_count=%d referenced=%.4f rad\n",
                static_cast<int>(motor.position_reference().homed()),
                static_cast<int>(motor.position_reference().lost_count()),
                static_cast<double>(motor.position_reference().referenced_mech_rad()));
    ASSERT_TRUE(motor.position_reference().homed()) << "the mark was crossed but the origin was never established";

    motor.set_position(0.0F);
    EXPECT_NE(foc::ErrorCode::NotImpl, motor.tick(5.0e-5F))
        << "with an origin the command must be attempted, not refused as a missing capability";
}
