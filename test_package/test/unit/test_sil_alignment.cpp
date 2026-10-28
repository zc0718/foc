// S10: shaft-lock alignment end to end -- an unknown encoder mounting offset, found by the routine,
// written into the sensor, and then used by the closed loops.
// S10：锁轴对齐端到端——一个未知的编码器安装偏置，由流程找出、写进传感器，再被闭环使用。

#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include <foc/core/alignment.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"
#include "sil_plant_hal.hpp"



namespace {

constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kPositionBandwidth = 20.0F; ///< [en] position-loop bandwidth, rad/s. [zh] 位置环带宽，rad/s。
constexpr float kVelocityLimit = 50.0F;     ///< [en] speed setpoint limit, rad/s. [zh] 速度给定上限，rad/s。

auto poles() -> float { return static_cast<float>(foc::sil::nominal_spec().pole_pairs); }

/// One mechanical pole pitch: the periodic length of the electrical angle, and therefore the
/// resolution limit of any alignment that starts from an unknown rotor angle.
/// 一个机械极距：电角度的周期长度，也因此是从未知转子角出发的对齐所能达到的分辨极限。
auto pitch_rad() -> float { return kTwoPi / poles(); }

/// Wrap into (-pitch/2, pitch/2].
auto wrap_pitch(float angle_rad) -> float {
    const float pitch = pitch_rad();
    float wrapped = std::fmod(angle_rad + (pitch / 2.0F), pitch);
    if (wrapped < 0.0F) {
        wrapped += pitch;
    }
    return wrapped - (pitch / 2.0F);
}

/// Wrap into (-pi, pi].
auto wrap_pi(float angle_rad) -> float {
    float wrapped = std::fmod(angle_rad + 3.14159265358979323846F, kTwoPi);
    if (wrapped < 0.0F) {
        wrapped += kTwoPi;
    }
    return wrapped - 3.14159265358979323846F;
}

/// The nominal machine, but with the mechanical damping a geared joint actually has. Alignment needs
/// the rotor to come to rest inside the hold window, and the nominal `b` is so small that the free
/// oscillation would still be ringing after two seconds.
/// 标称电机，但带上减速关节实际的机械阻尼。对齐需要转子在保持窗口内停下来，而标称的 `b` 太小，
/// 自由振荡两秒后仍在响。
auto joint_spec() -> foc::sil::MachineSpec {
    foc::sil::MachineSpec spec = foc::sil::nominal_spec();
    spec.j = 1e-5F;
    spec.b = 5e-3F;
    return spec;
}

auto align_recipe() -> foc::core::AlignConfig {
    // current, ramp, hold, tolerance, verify, forced angle
    return foc::core::AlignConfig{0.5F, 0.05F, 0.20F, 0.02F, 0.10F, 0.0F};
}



/**
 * @brief [en] Bench rig: plant + HAL + a motor whose alignment slot is the type under test.
 *        [zh] 台架：plant + HAL + 对齐槽位为被测类型的电机。
 */
template <typename AlignerT> class Rig final {
  public:
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                     foc::core::PositionP, AlignerT>;

    Rig(float theta0_rad, float mount_offset_rad, AlignerT aligner)
        : cfg(foc::sil::standard_bench_config()), spec(joint_spec()), mc(foc::sil::motor_config_from(spec)),
          plant(foc::sil::plant_params_from(spec)), hal(plant, cfg.v_bus_v),
          motor(mc,
                typename MotorT::Policies{
                    foc::core::BackEmfObserver{foc::core::BackEmfObserver::Config{spec.rs, spec.ld}},
                    foc::core::Svpwm{},
                    foc::core::Pi{foc::sil::current_pi_config(mc, cfg.omega_current_rad_s, cfg.v_bus_v)},
                    foc::core::Pi{foc::sil::current_pi_config(mc, cfg.omega_current_rad_s, cfg.v_bus_v)},
                    foc::core::Pi{foc::sil::speed_pi_config(mc, cfg.omega_speed_rad_s)},
                    foc::core::NullReferenceGenerator{}, foc::core::NullFeedforward{},
                    foc::core::PositionP{foc::position_p_config(mc, kPositionBandwidth, kVelocityLimit, 0.0F)},
                    aligner},
                hal) {
        hal.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, theta0_rad});
        hal.position.set_mount_error_rad(mount_offset_rad);
    }

    Rig(const Rig&) = delete;
    Rig& operator=(const Rig&) = delete;
    Rig(Rig&&) = delete;
    Rig& operator=(Rig&&) = delete;

    void step(float seconds) {
        const int steps = static_cast<int>((seconds / cfg.dt_ctrl_s) + 0.5F);
        for (int i = 0; i < steps; ++i) {
            static_cast<void>(motor.tick(cfg.dt_ctrl_s));
            hal.advance(cfg.dt_ctrl_s, cfg.substeps);
            elapsed_s += cfg.dt_ctrl_s;
        }
    }

    /// [en] init + start, then tick until the machine leaves Align. Returns the elapsed time. [zh] init + start，然后 tick 到离开 Align，返回耗时。
    auto run_through_alignment(float limit_s) -> float {
        if (motor.init() != foc::ErrorCode::Ok) {
            return -1.0F;
        }
        if (motor.start() != foc::ErrorCode::Ok) {
            return -1.0F;
        }
        const float began = elapsed_s;
        while ((motor.state() == foc::core::MotorState::Align) && ((elapsed_s - began) < limit_s)) {
            step(cfg.dt_ctrl_s);
        }
        return elapsed_s - began;
    }

    [[nodiscard]] auto reported_angle() -> float { return hal.position.angle_mech_rad(); }
    [[nodiscard]] auto true_angle() const -> float { return plant.theta_mech_rad(); }
    [[nodiscard]] auto true_speed() const -> float { return plant.omega_mech_rad_s(); }

    foc::sil::BenchConfig cfg;
    foc::sil::MachineSpec spec;
    foc::motor::MotorConfig mc;
    foc::sil::PmsmPlant plant;
    foc::sil::PlantHal hal;
    MotorT motor;
    float elapsed_s{0.0F};
};

} // namespace



TEST(SilAlignment, TheRotorSettlesOnTheForcedFieldAndTheOffsetIsCaptured) {
    // A joint whose encoder is mounted 2.3 rad off, starting from an arbitrary rotor angle.
    // 一个编码器装偏 2.3 rad 的关节，起始转子角任意。
    constexpr float kMountOffset = 2.3F;
    Rig<foc::core::AlignRoutine> rig(1.0F, kMountOffset, foc::core::AlignRoutine{align_recipe()});

    const float alignment_s = rig.run_through_alignment(2.0F);
    ASSERT_GT(alignment_s, 0.0F) << "init()/start() failed";
    // ramp + hold + verify, with one control period of slack
    EXPECT_NEAR(0.35F, alignment_s, 2.0F * rig.cfg.dt_ctrl_s);
    ASSERT_EQ(foc::core::MotorState::Run, rig.motor.state());
    ASSERT_EQ(foc::core::AlignState::Done, rig.motor.aligner().state());

    const float captured = rig.motor.aligner().offset_mech_rad();
    // Only the electrical part is observable: the rotor parks at the *nearest* pole, so the captured
    // offset is right modulo one pole pitch. Asserting equality in rad would be asserting that the
    // rotor happened to start in the right pole sector.
    // 只有电角度部分可观测：转子停在**最近的**极上，因此捕获的偏置在一个极距的模下正确。断言 rad 相等
    // 等于断言转子恰好起始在正确的极扇区里。
    EXPECT_NEAR(0.0F, wrap_pitch(captured - kMountOffset), 0.02F)
        << "captured " << captured << " expected " << kMountOffset << " mod " << pitch_rad();
    EXPECT_LT(rig.motor.aligner().spread_rad(), align_recipe().settle_tolerance_rad);

    // And the sensor now reports the true angle, again modulo one pole pitch.
    // 传感器现在汇报真实角度，同样在一个极距的模下。
    EXPECT_NEAR(0.0F, wrap_pitch(rig.reported_angle() - rig.true_angle()), 0.02F);
}



TEST(SilAlignment, WithoutAlignmentTheSameCommandProducesTorqueInTheWrongDirection) {
    // Negative control. Same machine, same mounting error, same torque command -- but no alignment.
    // The commutation offset is then 2.3*7 rad off, i.e. close to half a turn, so the q-axis current
    // the controller thinks it is applying lands almost entirely on -q.
    // 负向对照。同一台电机、同一个安装误差、同一个转矩指令——但不做对齐。此时换相偏置差了 2.3*7 rad，
    // 接近半圈，控制器以为施加的 q 轴电流几乎整个落在 -q 上。
    Rig<foc::core::NullAlignment> rig(1.0F, 2.3F, foc::core::NullAlignment{});
    ASSERT_EQ(foc::ErrorCode::Ok, rig.motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, rig.motor.start());
    ASSERT_EQ(foc::core::MotorState::Run, rig.motor.state());

    rig.motor.set_torque(0.5F);
    rig.step(0.05F);

    EXPECT_LT(rig.true_speed(), -1.0F) << "a nearly inverted commutation angle must not produce positive torque";
}



TEST(SilAlignment, AfterAlignmentTheSameTorqueCommandProducesTorqueInTheRightDirection) {
    Rig<foc::core::AlignRoutine> rig(1.0F, 2.3F, foc::core::AlignRoutine{align_recipe()});
    ASSERT_GT(rig.run_through_alignment(2.0F), 0.0F);
    ASSERT_EQ(foc::core::MotorState::Run, rig.motor.state());

    rig.motor.set_torque(0.5F);
    rig.step(0.05F);

    EXPECT_GT(rig.true_speed(), 1.0F) << "an aligned machine must produce positive torque for positive iq";
    // The current loop kept its promise as well: the d-q currents it reports are the planted ones.
    // 电流环也守住了承诺：它报出的 d-q 电流就是 plant 里的那一对。
    const foc::motor::Telemetry tm = rig.motor.telemetry();
    EXPECT_NEAR(0.5F, tm.iq_a, 0.02F);
    EXPECT_NEAR(0.0F, tm.id_a, 0.02F);
}



TEST(SilAlignment, PositionControlWorksOnTheAlignedFrame) {
    // The point of the whole exercise: position mode was unusable on real hardware exactly because
    // the frame it closes on was unknown. After alignment, a 0.2 rad move lands where it was asked.
    // 整套流程的意义：位置模式在真机上不可用，原因正是它所闭环的坐标系未知。对齐之后，0.2 rad 的移动
    // 落在被要求的位置上。
    Rig<foc::core::AlignRoutine> rig(1.0F, 2.3F, foc::core::AlignRoutine{align_recipe()});
    ASSERT_GT(rig.run_through_alignment(2.0F), 0.0F);
    ASSERT_EQ(foc::core::MotorState::Run, rig.motor.state());

    const float start_reported = rig.reported_angle();
    const float start_true = rig.true_angle();
    const float target = start_reported + 0.2F;
    ASSERT_LT(target, kTwoPi) << "the test target must not cross the sensor's wrap";

    rig.motor.set_position(target);
    rig.step(0.5F);

    EXPECT_NEAR(0.0F, rig.reported_angle() - target, 0.01F);
    // The rotor really moved by the same amount: this is not an artefact of the offset bookkeeping.
    // 转子真的移动了同样的量：这不是偏置记账造成的假象。
    EXPECT_NEAR(0.2F, wrap_pi(rig.true_angle() - start_true), 0.015F);
}



TEST(SilAlignment, PositionControlFailsWithoutAlignment) {
    // Negative control for the same claim: with the mounting offset unknown, the frame the position
    // loop closes on is not the frame the rotor lives in. Commanding the same +0.2 rad from the same
    // starting point ends somewhere else entirely.
    // 同一条结论的负向对照：安装偏置未知时，位置环所闭环的坐标系并非转子所在的坐标系。从同一起点发出
    // 同样的 +0.2 rad，最终停的地方完全不同。
    Rig<foc::core::NullAlignment> rig(1.0F, 2.3F, foc::core::NullAlignment{});
    ASSERT_EQ(foc::ErrorCode::Ok, rig.motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, rig.motor.start());
    ASSERT_EQ(foc::core::MotorState::Run, rig.motor.state());

    const float start_reported = rig.reported_angle();
    const float start_true = rig.true_angle();
    rig.motor.set_position(start_reported + 0.2F);
    rig.step(0.5F);

    EXPECT_GT(std::fabs(wrap_pi(rig.true_angle() - start_true - 0.2F)), 0.1F)
        << "without the offset the rotor must not end up 0.2 rad away in the true frame";
}
