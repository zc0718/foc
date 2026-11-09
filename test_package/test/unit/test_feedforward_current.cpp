// The iq feedforward slot: what it means, and the four things that have to be true of it before a compensator can rely on
// it. Before this, the header said "iq added after the speed loop" and the code applied it in position mode only, so a
// speed-mode host had no way to use the field at all -- `set_command()` was the only writer and it changes the mode.
// iq 前馈槽：它是什么意思，以及在补偿器能依赖它之前必须成立的四个事实。在这之前，头文件写的是「速度环后叠加的 iq」，而代码只在位置
// 模式应用它，于是速度模式的宿主根本没法用这个字段——唯一的写入者 `set_command()` 会改变模式。

#include <cmath>
#include <optional>

#include <gtest/gtest.h>

#include <foc/core/position.hpp>

#include "sil_bench.hpp"

namespace {

using foc::sil::Bench;
using foc::sil::standard_bench_config;

constexpr float kIq = 0.6F;

/** @brief The bench, but with a position controller so that position mode is admissible at all. */
using BenchMotor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                     foc::core::PositionP, foc::core::NullAlignment, foc::core::NullTrace>;

struct Rig final {
    explicit Rig(float omega_mech) : hal{plant, cfg_.v_bus_v} {
        hal.position.set_sensor_present(true);
        const auto mc = foc::sil::nominal_motor_config();
        motor.emplace(mc,
                      BenchMotor::Policies{foc::core::BackEmfObserver{foc::core::BackEmfObserver::Config{mc.rs, mc.ld}},
                                           foc::core::Svpwm{},
                                           foc::core::Pi{foc::current_pi_config(mc, cfg_.omega_current_rad_s, 24.0F)},
                                           foc::core::Pi{foc::current_pi_config(mc, cfg_.omega_current_rad_s, 24.0F)},
                                           foc::core::Pi{foc::speed_pi_config(mc, cfg_.omega_speed_rad_s)},
                                           foc::core::NullReferenceGenerator{}, foc::core::NullFeedforward{},
                                           foc::core::PositionP{foc::core::PositionP::Config{20.0F, 200.0F, 2000.0F}},
                                           foc::core::NullAlignment{}, foc::core::NullTrace{}},
                      hal);
        motor->set_speed(omega_mech);
        EXPECT_EQ(foc::ErrorCode::Ok, motor->init());
        EXPECT_EQ(foc::ErrorCode::Ok, motor->start());
    }

    auto settle_and_mean_iq(float seconds) -> float {
        motor->set_speed(40.0F);
        const int settle = static_cast<int>(seconds / cfg_.dt_ctrl_s);
        for (int n = 0; n < settle; ++n) {
            static_cast<void>(motor->tick(cfg_.dt_ctrl_s));
            hal.advance(cfg_.dt_ctrl_s, cfg_.substeps);
        }
        float sum = 0.0F;
        int count = 0;
        for (int n = 0; n < 2000; ++n) {
            static_cast<void>(motor->tick(cfg_.dt_ctrl_s));
            hal.advance(cfg_.dt_ctrl_s, cfg_.substeps);
            if (n > 500) {
                sum = static_cast<float>(sum + plant.current_dq().q);
                ++count;
            }
        }
        return sum / static_cast<float>(count);
    }

    foc::sil::BenchConfig cfg_{standard_bench_config()};
    foc::sil::PmsmPlant plant{foc::sil::nominal_plant_params()};
    foc::sil::PlantHal hal;
    std::optional<BenchMotor> motor{};
};

} // namespace

// The headline, measured in the only place a constant offset is visible: the **transient**. A step of 0.2 A lifts the measured
// current to +0.174 A within about 0.85 ms and then the loop unwinds it -- so the field does reach the reference path, and the
// steady state below is a separate fact rather than a contradiction.
// 要点，量在恒定偏置唯一可见的地方： **瞬态** 。0.2 A 的阶跃在约 0.85 ms 内把实测电流抬到 +0.174 A，随后环路把它收回——所以这个字段
// 确实到了参考路径，而下面的稳态是另一个事实，不是矛盾。
TEST(FeedforwardCurrent, SpeedModeCarriesTheOffsetIntoTheCurrent) {
    Rig rig{40.0F};
    const float baseline = rig.settle_and_mean_iq(0.4F);
    rig.motor->set_feedforward_current(0.2F);
    float peak = 0.0F;
    for (int n = 0; n < 64; ++n) {
        static_cast<void>(rig.motor->tick(rig.cfg_.dt_ctrl_s));
        rig.hal.advance(rig.cfg_.dt_ctrl_s, rig.cfg_.substeps);
        peak = std::fmax(peak, rig.plant.current_dq().q - baseline);
    }
    // 0.174 A measured against the 0.2 A asked for: the loop has already started to unwind, which is why the bound is 0.15 and
    // not 0.19 -- and a field that were ignored would leave this at zero.
    // 实测 0.174 A 对应要求的 0.2 A：环路已经开始收回，所以界是 0.15 而不是 0.19——而一个被忽略的字段会让这个数是零。
    EXPECT_GT(peak, 0.15F) << "peak deviation " << peak;
    EXPECT_LT(peak, 0.21F) << "peak deviation " << peak;
}

// And the finding that the first version of this case got wrong: in a loop with integral action a **constant** offset is
// absorbed -- after a second the measured current is back on the baseline to six decimals. That is not a defect of the slot; it
// is why a constant gravity or friction term is not what this slot is for, and why the cogging case (a ripple, which no
// integrator can absorb) is the one that shows it working.
// 而本用例第一版弄错的那个事实：在带积分作用的环路里， **恒定** 偏置会被吸收——一秒之后实测电流回到基线，到六位小数。这不是槽的缺陷；
// 这正是"恒定重力/摩擦项不是这个槽的用途"的原因，也是齿槽那件（纹波，任何积分器都吸收不了）才看得出它工作的原因。
TEST(FeedforwardCurrent, AConstantOffsetInSpeedModeIsAbsorbedByTheIntegrator) {
    Rig rig{40.0F};
    const float baseline = rig.settle_and_mean_iq(0.4F);
    rig.motor->set_feedforward_current(0.2F);
    for (int n = 0; n < 40000; ++n) { // 2 s
        static_cast<void>(rig.motor->tick(rig.cfg_.dt_ctrl_s));
        rig.hal.advance(rig.cfg_.dt_ctrl_s, rig.cfg_.substeps);
    }
    EXPECT_NEAR(baseline, rig.plant.current_dq().q, 1.0e-5F);
}

// Position mode carries it too -- the mode where it used to be the only place it worked. Asserted the same way, on the
// transient, because the steady state answers the same question the same way.
// 位置模式也带得动它——那曾是它唯一生效的模式。同样断言在瞬态上，因为稳态对同一个问题的回答方式相同。
TEST(FeedforwardCurrent, PositionModeCarriesItIntoTheCurrent) {
    Rig rig{0.0F};
    rig.motor->set_command(foc::motor::Command{0.0F, 0.0F, 0.0F, 40.0F, 2000.0F, 0.0F, 0.0F, 0.0F});
    for (int n = 0; n < 8000; ++n) {
        static_cast<void>(rig.motor->tick(rig.cfg_.dt_ctrl_s));
        rig.hal.advance(rig.cfg_.dt_ctrl_s, rig.cfg_.substeps);
    }
    const float baseline = rig.plant.current_dq().q;
    rig.motor->set_command(foc::motor::Command{0.0F, 0.0F, 0.2F, 40.0F, 2000.0F, 0.0F, 0.0F, 0.0F});
    float peak = 0.0F;
    for (int n = 0; n < 64; ++n) {
        static_cast<void>(rig.motor->tick(rig.cfg_.dt_ctrl_s));
        rig.hal.advance(rig.cfg_.dt_ctrl_s, rig.cfg_.substeps);
        peak = std::fmax(peak, rig.plant.current_dq().q - baseline);
    }
    EXPECT_GT(peak, 0.15F) << "peak deviation " << peak;
}

// The trap that made the gap invisible: `set_command()` switches the axis to position mode. A setter that changed what was
// being controlled would be unusable for a per-period compensator, so `set_feedforward_current()` must not touch the mode.
// 让那个缺口看不见的陷阱：`set_command()` 会把轴切到位置模式。一个会改变被控对象的设置器对逐周期补偿器来说不可用，所以
// `set_feedforward_current()` 不得触碰模式。
TEST(FeedforwardCurrent, SettingItDoesNotChangeTheMode) {
    Rig rig{40.0F};
    for (int n = 0; n < 2000; ++n) {
        static_cast<void>(rig.motor->tick(rig.cfg_.dt_ctrl_s));
        rig.hal.advance(rig.cfg_.dt_ctrl_s, rig.cfg_.substeps);
    }
    const float speed_before = rig.plant.omega_mech_rad_s();
    rig.motor->set_feedforward_current(0.05F);
    for (int n = 0; n < 2000; ++n) {
        static_cast<void>(rig.motor->tick(rig.cfg_.dt_ctrl_s));
        rig.hal.advance(rig.cfg_.dt_ctrl_s, rig.cfg_.substeps);
    }
    // Still speed-controlled: the axis is at the reference it was given, not parked at a position.
    // 依然是速度控制：轴在它被给的参考上，而不是停在某个位置。
    EXPECT_GT(std::fabs(rig.plant.omega_mech_rad_s()), 30.0F);
    EXPECT_GT(std::fabs(rig.plant.omega_mech_rad_s() - speed_before), 0.0F);
}

// Zero is the value every existing host has, and zero has to be exactly "not there". The comparison is on the raw floats of
// the current and the speed, not on a tolerance: a term that is present but numerically tiny would move a claim's last digit
// and nobody would notice for months.
// 零是所有既有宿主都有的值，而零必须精确地等于「不存在」。比较的是电流与转速的原始浮点，不是容差：一个存在但数值极小的项会改变某条
// 声称的最后一位，而几个月内没人会注意到。
TEST(FeedforwardCurrent, ZeroIsBitIdenticalToNeverSettingIt) {
    Rig without{40.0F};
    for (int n = 0; n < 6000; ++n) {
        static_cast<void>(without.motor->tick(without.cfg_.dt_ctrl_s));
        without.hal.advance(without.cfg_.dt_ctrl_s, without.cfg_.substeps);
    }
    Rig with_zero{40.0F};
    with_zero.motor->set_feedforward_current(0.0F);
    for (int n = 0; n < 6000; ++n) {
        static_cast<void>(with_zero.motor->tick(with_zero.cfg_.dt_ctrl_s));
        with_zero.hal.advance(with_zero.cfg_.dt_ctrl_s, with_zero.cfg_.substeps);
    }
    EXPECT_EQ(without.plant.current_dq().q, with_zero.plant.current_dq().q);
    EXPECT_EQ(without.plant.current_dq().d, with_zero.plant.current_dq().d);
    EXPECT_EQ(without.plant.omega_mech_rad_s(), with_zero.plant.omega_mech_rad_s());
}
