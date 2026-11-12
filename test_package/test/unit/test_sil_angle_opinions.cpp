// The observer's second opinion, measured rather than assumed reachable.
// 观测器的那份第二意见——量出来的，而不是假定它可达。
//
// `AngleSource` arbitrates between offers; the assembled drive makes its own choice in one line and has never formed
// two offers at once on a *working* sensored axis, because the observer is only stepped when the loop needs its angle
// or when it is injecting a carrier. So the arbitration type was tested against offers the assembly could not produce.
// This file closes that: the two telemetry angles make the disagreement observable on a real bench, and the case then
// hands them to `AngleSource` to show what a quality verdict would have been.
//
// `AngleSource` 在各份报价之间仲裁；而装配层自己在一行里做出选择，并且在**工作正常的**带传感器轴上从未同时构造过两份
// 报价——因为观测器只在环需要它的角度、或它在注入载波时才被推进。于是那个仲裁类型的用例，对着的是装配层造不出来的候选。
// 本文件补上这一段：两个遥测角度让分歧在真实台架上可见，用例再把它俩交给 `AngleSource`，看质量判决本来会是什么。

#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/angle_source.hpp>
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

auto wrap_pi(float angle_rad) -> float {
    float wrapped = std::fmod(angle_rad + 3.14159265358979323846F, kTwoPi);
    if (wrapped < 0.0F) {
        wrapped += kTwoPi;
    }
    return wrapped - 3.14159265358979323846F;
}

/**
 * @brief [en] Plant + HAL + a motor, on a quietly spinning rotor.
 *
 *   The rotor is brought up to speed by the plant itself (`PlantState` with a mechanical speed), because a second
 *   opinion about an angle is only interesting while something is turning: at standstill every estimator agrees
 *   trivially.
 *
 * @brief [zh] plant + HAL + 一台电机，转子在平稳地转。
 *
 *   转速由 plant 自己给出（ `PlantState` 带机械速度），因为关于角度的第二意见只有在 **转起来** 的时候才有意义：静止时每
 *   个估计器都平凡地一致。
 */
class Rig final {
  public:
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal>;

    explicit Rig(float mount_offset_rad) : spec(foc::sil::nominal_spec()), mc(foc::sil::motor_config_from(spec)),
                                           plant(foc::sil::plant_params_from(spec)), hal(plant, 24.0F),
                                           motor(mc, policies(), hal) {
        hal.position.set_mount_error_rad(mount_offset_rad);
    }

    Rig(const Rig&) = delete;
    Rig& operator=(const Rig&) = delete;
    Rig(Rig&&) = delete;
    Rig& operator=(Rig&&) = delete;

    /**
     * @brief [en] Bring the axis up under its own speed loop and let the observer follow it.
     *
     *   The plant is **not** teleported onto a spinning rotor after the observer has been seeded: that leaves the
     *   integrator a permanent constant offset (measured: 0.50 rad on a 7-pole-pair machine), which would then be the
     *   thing this case measures instead of the mounting error. A real speed command accelerates the rotor the observer
     *   is already integrating, so the two opinions start from the same history.
     *
     * @brief [zh] 让轴在自己的速度环下转起来，观测器跟着它走。
     *
     *   观测器播种之后 **不** 把 plant 瞬移到转动的转子上：那会给积分器留一个永久的常数偏置（实测 7 极对机器上是 0.50
     *   rad），于是被测量的就成了它而不是安装误差。用一条真实的速度指令去加速转子，观测器积分的正是同一段运动，两个意见
     *   才有同一个历史。
     */
    void settle(float seconds, float omega_mech) {
        const int steps = static_cast<int>((seconds / kDt) + 0.5F);
        for (int i = 0; i < steps; ++i) {
            last_sensor_angle_mech = hal.position.angle_mech_rad(); // exactly what the next tick will read
            static_cast<void>(motor.tick(kDt));
            hal.advance(kDt, 4);
        }
        static_cast<void>(omega_mech);
    }

    /**
     * @brief [en] init, start, run through alignment, then hold a speed reference and let the loop settle.
     * @brief [zh] init、start、走完对齐，然后守住一条速度给定让环稳定下来。
     */
    auto run_at_speed(float omega_mech) -> foc::motor::Telemetry {
        static_cast<void>(motor.init());
        if (motor.start() != foc::ErrorCode::Ok) {
            return foc::motor::Telemetry{};
        }
        while (motor.state() == foc::core::MotorState::Align) {
            static_cast<void>(motor.tick(kDt));
            hal.advance(kDt, 4);
        }
        motor.set_speed(omega_mech);
        settle(3.0F, omega_mech);
        return motor.telemetry();
    }

    [[nodiscard]] auto poles() const -> float { return static_cast<float>(spec.pole_pairs); }

    /**
     * @brief [en] The mechanical angle the sensor reported on the last tick the loop ran -- what `angle_mech_rad()`
     *             answered at the top of that tick, kept because the plant advances afterwards and reading it later is
     *             two samples late (measured: 0.105 rad electrical at 150 rad/s).
     * @brief [zh] 环跑最后一拍时传感器报的机械角——即那一拍读到 tick 顶部的 `angle_mech_rad()` 值。留着它是因为 plant 在
     *             那之后还会前进，事后再读会晚两个采样（实测 150 rad/s 下电角度差 0.105 rad）。
     */
    float last_sensor_angle_mech{0.0F};

    foc::sil::MachineSpec spec;
    foc::motor::MotorConfig mc;
    foc::sil::PmsmPlant plant;
    foc::sil::PlantHal hal;
    MotorT motor;

  private:
    static constexpr float kDt = 5e-5F;

    static auto policies() -> typename MotorT::Policies {
        const foc::sil::MachineSpec spec = foc::sil::nominal_spec();
        const foc::motor::MotorConfig mc = foc::sil::motor_config_from(spec);
        return typename MotorT::Policies{
            foc::core::BackEmfObserver{foc::core::BackEmfObserver::Config{spec.rs, spec.ld}}, foc::core::Svpwm{},
            foc::core::Pi{foc::sil::current_pi_config(mc, 2000.0F, 24.0F)},
            foc::core::Pi{foc::sil::current_pi_config(mc, 2000.0F, 24.0F)},
            foc::core::Pi{foc::sil::speed_pi_config(mc, 250.0F)}};
    }
};

auto angle_source_config() -> foc::core::AngleSourceConfig {
    foc::core::AngleSourceConfig config{};
    config.handover_window_s = 2e-3F;
    config.disagree_degraded_rad = 0.10F;
    config.disagree_blind_rad = 0.40F;
    return config;
}

// 1. A sensor that also reports a speed leaves the observer unstepped, so there is no second opinion -- and the
// telemetry says `NaN` rather than a zero. "No second opinion" has to be distinguishable from "a second opinion that
// agrees", or an arbitration would call silence consent.
// 1. 传感器同时报速度时，观测器不被推进，于是没有第二意见——遥测答 `NaN` 而不是零。"没有第二意见"必须与"第二意见也
//    同意"可分，否则仲裁会把沉默当成同意。
TEST(AngleOpinions, ASensorWithASpeedLeavesNoSecondOpinionAtAll) {
    Rig rig(0.0F);
    const foc::motor::Telemetry telemetry = rig.run_at_speed(150.0F);
    EXPECT_FALSE(std::isnan(telemetry.theta_e_rad)) << "the loop always commutated on something";
    EXPECT_TRUE(std::isnan(telemetry.observer_theta_e_rad))
        << "the observer was never stepped, so there is nothing to compare the sensor against";
}

// 2. An encoder that reports an angle and no speed is the one configuration where both opinions exist in the same
// tick: the sensor owns the angle, the observer owns the speed, and the loop commutates on the sensor while carrying an
// observer estimate it never looks at. That is the gap this whole line is about, and it is provable without attributing
// the gap to anything: the published angle has to be the sensor's own reading, and the observer's has to be a *different*
// number that the assembly could have used.
// 2. 只报角度、不报速度的编码器，是两个意见能在同一拍存在的唯一配置：传感器管角度，观测器管转速，而环用传感器换相、
//    同时带着一个它从不看的观测器估计。这就是本线一直在说的那个缺口，而它可以**不归因**地证明：公布的角度必须是传感器
//    自己的读数，而观测器那个必须是一个**不同**的数、而且还是装配层本可以采用的数。
//
//   What this case deliberately does not claim: that the difference between the two equals the mounting error. It also
//   carries the observer's own error at this operating point and the two wrap conventions (a mechanical turn against an
//   electrical one), and separating those is a different measurement with a different rig -- writing the equality here
//   would have been a number that agrees for the wrong reason.
//   本用例刻意**不**主张：两者之差等于安装误差。它还带着观测器在此工作点上的自身误差，以及两套归一化约定（机械圈对电
//   周期），把它们分开是另一台架上的另一次测量——在这里写那个等式，等于写一个因为错的原因才对上的数。
TEST(AngleOpinions, BothOpinionsExistAndTheAssemblyUsesOnlyOneOfThem) {
    Rig rig(0.10F);                                 // a sensor mounted 0.1 rad off
    rig.hal.position.set_velocity_supported(false); // angle from the sensor, speed from the observer
    const foc::motor::Telemetry telemetry = rig.run_at_speed(150.0F);

    const float poles = rig.poles();
    const float expected_used = std::remainder(rig.last_sensor_angle_mech * poles, 2.0F * 3.14159265358979323846F);
    std::printf("\n[opinions] used %.4f rad, observer %.4f rad, sensor says %.4f rad\n",
                static_cast<double>(telemetry.theta_e_rad), static_cast<double>(telemetry.observer_theta_e_rad),
                static_cast<double>(expected_used));

    EXPECT_NEAR(expected_used, telemetry.theta_e_rad, 1.0e-4F)
        << "the loop commutated on the sensor, exactly -- the observer's answer was not consulted";
    ASSERT_FALSE(std::isnan(telemetry.observer_theta_e_rad)) << "yet the observer ran, so a second opinion existed";
    EXPECT_GT(std::fabs(std::remainder(telemetry.theta_e_rad - telemetry.observer_theta_e_rad, 6.283185307179586F)), 0.05F)
        << "and it is a different answer: the two opinions are not the same number twice";

    // What the arbitration would have said, fed exactly those two numbers -- the type's verdict on real data instead of
    // on offers the assembly cannot form.
    // 仲裁对**这两个数**本来会说什么——该类型的判决现在对着真实数据，而不是对着装配层造不出来的候选。
    foc::core::AngleSource arbiter;
    ASSERT_EQ(foc::ErrorCode::Ok, arbiter.configure(angle_source_config()));
    const foc::core::AngleOffer offers[2] = {
        foc::core::AngleOffer{foc::core::AngleSourceKind::Sensor, true, telemetry.theta_e_rad, 0.0F},
        foc::core::AngleOffer{foc::core::AngleSourceKind::Observer, true, telemetry.observer_theta_e_rad, 0.0F}};
    const foc::core::AngleReading& reading = arbiter.step(offers, 2U, 5e-5F);
    EXPECT_NE(foc::core::AngleQuality::Ok, reading.quality)
        << "two estimates this far apart are not a confident measurement, and the arbiter says so";
    EXPECT_FLOAT_EQ(std::fabs(std::remainder(telemetry.theta_e_rad - telemetry.observer_theta_e_rad, 6.283185307179586F)),
                    reading.disagreement_rad)
        << "and the number the verdict was made on is the gap between the two published angles";
}

} // namespace
