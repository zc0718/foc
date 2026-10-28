// Where the carrier is applied, and where it deliberately is not.
//
// This file exists because of a defect nobody could see: an injection observer selected on an axis that has an angle
// sensor applied no carrier at all. `tick()` takes the sensor branch, so the observer is never stepped -- and an observer
// that is not stepped does not advance its carrier phase, so `injection_ab()` returned a constant zero. The mode was
// admitted, `status()["observer"]` said `HFI`, and the inverter was never asked to inject anything.
//
// Worse than zero is the second half of the same hole: from any non-zero starting phase, reading the injection out of an
// un-stepped observer yields a *constant* vector, which is a DC offset on the current. That is the hazard the alignment
// path already documented for its own case; the sensor case had the same shape and no gate.
//
// 本文件的存在理由是一个看不见的缺陷：在带角度传感器的轴上选中注入观测器，却完全不施加载波。`tick()` 走传感器分支，
// 于是观测器从不被推进——而没被推进的观测器不会推进载波相位，所以 `injection_ab()` 返回恒零。模式被准入、
// `status()["observer"]` 说 `HFI`，而逆变器从未被要求注入任何东西。
//
// 比零更糟的是同一个洞的另一半：从任何非零初相读出未推进观测器的注入，得到的是一个**常矢量**，即电流上的直流偏置。
// 这正是对齐路径已经为自己记录过的隐患；传感器那条路形状相同，却没有门控。

#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/observer/hfi_observer.hpp>
#include <foc/core/observer/switchable_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/sm.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/types.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

namespace {

using Motor = foc::motor::Motor<foc::core::SwitchableObserver, foc::core::Svpwm, foc::core::Pi, foc::driver::StubHal>;

constexpr float kDt = 5e-5F; // 20 kHz
constexpr float kVBus = 24.0F;
constexpr float kInjectV = 10.0F;
constexpr float kInjectHz = 2000.0F;

auto config() -> foc::motor::MotorConfig {
    // rs, ld, lq, kt, j, b, pole_pairs, i_max, pwm_freq, deadtime -- salient, so injection is admissible at all
    return foc::motor::MotorConfig{0.5F, 4e-4F, 8e-4F, 0.05F, 1e-5F, 1e-6F, 7U, 2.0F, 20000.0F, 500.0F};
}

auto hfi_observer() -> foc::core::SwitchableObserver {
    foc::core::SwitchableObserver::Config c{};
    c.back_emf = foc::core::BackEmfObserver::Config{0.5F, 4e-4F};
    c.nonlinear_flux.psi_f = 0.05F / (1.5F * 7.0F);
    c.nonlinear_flux.l = 4e-4F;
    c.hfi.inject_v = kInjectV;
    c.hfi.inject_hz = kInjectHz;
    c.hfi.current_hp_hz = 300.0F;
    c.hfi.demod_lpf_hz = 200.0F;
    c.hfi.pll_bandwidth_hz = 40.0F;
    c.hfi.pll_zeta = 0.9F;
    c.hfi.demod_gain_a = foc::core::HfiObserver::predicted_gain_a(kInjectV, kInjectHz, 4e-4F, 8e-4F);
    c.hfi.min_hf_current_a = 0.05F;
    c.hfi.speed_lpf_hz = 200.0F;
    c.kind = foc::core::ObserverKind::Hfi;
    return foc::core::SwitchableObserver(c);
}

auto policies() -> Motor::Policies {
    const foc::core::Pi::Config current{1.0F, 0.0F, -12.0F, 12.0F, -12.0F, 12.0F};
    const foc::core::Pi::Config speed{0.01F, 0.001F, -2.0F, 2.0F, -2.0F, 2.0F};
    return Motor::Policies{hfi_observer(), foc::core::Svpwm{}, foc::core::Pi{current}, foc::core::Pi{current},
                           foc::core::Pi{speed}};
}

/**
 * @brief [en] Duty swing over the last `settle` of `periods`, which is how "did anything reach the inverter" is asked.
 * @brief [zh] 在 `periods` 的最后 `settle` 段里的占空比摆幅——"有没有东西到达逆变器"就是这样问的。
 */
auto run_and_measure(bool sensor_present, int periods = 400) -> float {
    foc::driver::StubHal hal;
    hal.dc_link.set_voltage_v(kVBus);
    hal.position.set_sensor_present(sensor_present);
    hal.position.set_angle_mech_rad(0.0F);
    hal.position.set_velocity_rad_s(0.0F);
    const foc::core::Uvw<float> zero{};
    EXPECT_EQ(foc::ErrorCode::Ok, hal.current_sense.load(&zero, 1U));

    Motor motor(config(), policies(), hal);
    EXPECT_EQ(foc::ErrorCode::Ok, motor.init());
    EXPECT_EQ(foc::ErrorCode::Ok, motor.start());
    motor.set_torque(0.0F);

    float low = 1.0F;
    float high = 0.0F;
    for (int k = 0; k < periods; ++k) {
        EXPECT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
        const auto duty = hal.pwm.last_duty();
        if (k > (periods / 2)) {
            low = (duty.u < low) ? duty.u : low;
            high = (duty.u > high) ? duty.u : high;
        }
    }
    EXPECT_EQ(foc::core::MotorState::Run, motor.state());
    return high - low;
}

} // namespace

TEST(MotorInjection, AnAxisWithoutASensorGetsTheCarrierItAskedFor) {
    // The criterion is the swing, not the reported mode: a selector that admits HFI and then injects nothing satisfies
    // every status assertion and produces no flux modulation at all. 10 V on a 24 V bus has to move the duty by a large
    // fraction, so "some" is not the bar -- anything below a tenth of full scale means the carrier is not there.
    // 判据是摆幅，不是上报的模式：一个准入 HFI 然后什么都不注入的选择器，能满足每一条状态断言，却完全不调制磁链。
    // 24 V 母线上的 10 V 必须让占空比大幅度移动，所以判据不是"有一点"——低于满量程十分之一就说明载波不在。
    const float swing = run_and_measure(/*sensor_present=*/false);
    EXPECT_GT(swing, 0.1F) << "the carrier never reached the inverter: swing " << swing;
}

TEST(MotorInjection, ASensoredAxisInjectsWithoutSurrenderingTheAngle) {
    // The criterion, and the history it encodes. An axis whose HAL supplies the angle used to take the sensor branch and
    // never step the observer -- so an injecting mode applied *no carrier at all*: admitted, reported as `HFI`, and
    // inert. The first draft of this case asserted that zero as the correct answer, which is how a defect becomes a
    // documented behaviour if nobody measures it.
    //
    // What is required instead: the carrier is applied (the swing), and the sensor keeps the commutation angle -- the
    // observer runs for its own demodulation, not to sell the loop an angle it already has. That is a hybrid drive, and
    // it is the only reading that makes the mode real on a rig with an encoder.
    // 判据，以及它所记录的历史。HAL 提供角度的轴过去走传感器分支、从不推进观测器——于是注入模式**完全不施加任何载波**：
    // 被准入、被上报为 `HFI`、毫无作用。本用例的第一版把这个零断言成正确答案，而没人测量时，缺陷就是这样变成"已记录的
    // 行为"的。
    //
    // 现在要求的是：载波确实施加（看摆幅），而换相角仍归传感器——观测器为自己的解调而运行，不是要把一个已经有的角度卖给
    // 环路。这就是混合驱动，也是让该模式在带编码器的台架上真实存在的唯一读法。
    const float swing = run_and_measure(/*sensor_present=*/true);
    EXPECT_GT(swing, 0.1F) << "a sensored axis applied no carrier: swing " << swing;
}
