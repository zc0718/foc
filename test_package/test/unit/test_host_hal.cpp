// The other half of the HAL contract: the same control loop with its measurements coming from a host and its duties
// going back to that host. Everything the Python stack and the FMU adapter will build on starts here -- if this file's
// claims hold, "the same loop, verified in three places" is a statement about the loop rather than about the stub.
// HAL 契约的另一半：同一个控制环，测量值来自主控方，占空比送回主控方。Python 栈与 FMU 适配器要建的东西都从这里
// 开始——本文件的断言成立，"同一个环，在三处验证过"这句话谈的才是环，而不是桩。

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/driver/callback_hal.hpp>
#include <foc/motor/motor.hpp>

#include "sil_bench.hpp"

namespace foc {

namespace {

constexpr float kDtS = 1.0F / 20000.0F;

/**
 * @brief [en] What the host knows: the plant's state, plus how often the loop asked it anything.
 * @brief [zh] 主控方知道的东西：被控对象的状态，以及控制环问过它多少次。
 */
struct Host final {
    float current_a[3]{0.0F, 0.0F, 0.0F};
    float angle_rad{0.0F};
    float v_bus_v{24.0F};
    int duty_calls{0};
    int current_calls{0};
    float last_duty[3]{0.0F, 0.0F, 0.0F};
};

auto write_duty(void* user, const float uvw[3]) -> void {
    auto* host = static_cast<Host*>(user);
    host->duty_calls += 1;
    for (int i = 0; i < 3; ++i) {
        host->last_duty[i] = uvw[i];
    }
}

auto read_current(void* user, float uvw[3]) -> void {
    auto* host = static_cast<Host*>(user);
    host->current_calls += 1;
    for (int i = 0; i < 3; ++i) {
        uvw[i] = host->current_a[i];
    }
}

auto read_angle(void* user) -> float { return static_cast<Host*>(user)->angle_rad; }

auto read_voltage(void* user) -> float { return static_cast<Host*>(user)->v_bus_v; }

auto full_callbacks(Host& host) -> driver::HalCallbacks {
    driver::HalCallbacks callbacks{};
    callbacks.user = &host;
    callbacks.pwm_write_duty = &write_duty;
    callbacks.read_current = &read_current;
    callbacks.read_angle_mech_rad = &read_angle;
    callbacks.read_voltage_v = &read_voltage;
    return callbacks;
}

using HostMotor = motor::Motor<core::BackEmfObserver, core::Svpwm, core::Pi, driver::CallbackHal>;

auto policies(const motor::MotorConfig& mc) -> HostMotor::Policies {
    return HostMotor::Policies{core::BackEmfObserver{core::BackEmfObserver::Config{mc.rs, mc.ld}}, core::Svpwm{},
                               core::Pi{sil::current_pi_config(mc, 2000.0F, 24.0F)},
                               core::Pi{sil::current_pi_config(mc, 2000.0F, 24.0F)},
                               core::Pi{sil::speed_pi_config(mc, 50.0F)}};
}

} // namespace

TEST(HostHal, TheLoopAsksTheHostForMeasurementsAndGivesItDuties) {
  // The criterion, written before the measurement: a host that supplies currents, an angle and a bus voltage must see
  // its inverter driven. If `duty_calls` stays 0 the loop never reached the output stage, and if `current_calls` stays 0
  // it never read the phase currents -- either way the HAL is not being used, whatever the status says.
  // 先写下的判据：提供电流、角度与母线电压的主控方，必须看到自己的逆变器被驱动。若 `duty_calls` 恒为 0，说明环
  // 没走到输出级；若 `current_calls` 恒为 0，说明它没读相电流——两种情况都意味着 HAL 没被用上，无论状态字怎么说。
  Host host{};
  const driver::HalCallbacks callbacks = full_callbacks(host);
  EXPECT_TRUE(callbacks.valid());

  driver::CallbackHal hal(callbacks);
  const motor::MotorConfig mc = sil::nominal_motor_config();
  HostMotor motor(mc, policies(mc), hal);

  ASSERT_EQ(motor.start(), ErrorCode::Ok);
  for (int i = 0; i < 200; ++i) {
    host.angle_rad = 0.1F * static_cast<float>(i);
    (void)motor.tick(kDtS);
  }

  std::printf("\n[host-hal] %d duty write(s), %d current read(s) over 200 ticks\n", host.duty_calls,
              host.current_calls);
  EXPECT_GT(host.current_calls, 0) << "the loop never read the host's phase currents";
  EXPECT_GT(host.duty_calls, 0) << "the loop never wrote a duty cycle to the host";
  for (int i = 0; i < 3; ++i) {
    EXPECT_GE(host.last_duty[i], -1.0e-6F);
    EXPECT_LE(host.last_duty[i], 1.0F + 1.0e-6F);
  }
}

TEST(HostHal, AHostWithoutTheRequiredCallbacksIsRejected) {
  // A partially-filled callback set is a programming error, not a degraded mode: with no way to write a duty the loop
  // would compute forever and never switch, and with no angle it would run on a constant. `valid()` is what lets the C
  // ABI answer FOC_ERR_PARAM instead of pretending.
  // 填了一半的回调集是编程错误，不是降级模式：没有占空比出口，环会永远计算却从不开关；没有角度，它就对着一个常数
  // 跑。`valid()` 就是让 C ABI 能答 FOC_ERR_PARAM 而不是假装没事的东西。
  Host host{};
  driver::HalCallbacks callbacks = full_callbacks(host);
  EXPECT_TRUE(callbacks.valid());

  callbacks.pwm_write_duty = nullptr;
  EXPECT_FALSE(callbacks.valid());

  callbacks = full_callbacks(host);
  callbacks.read_current = nullptr;
  EXPECT_FALSE(callbacks.valid());

  callbacks = full_callbacks(host);
  callbacks.read_angle_mech_rad = nullptr;
  EXPECT_FALSE(callbacks.valid());

  callbacks = full_callbacks(host);
  callbacks.read_voltage_v = nullptr;
  EXPECT_FALSE(callbacks.valid());

  // Optional callbacks may be absent: no fault path, no speed measurement, no clock.
  // 可选项可以缺席：没有故障通道、没有转速测量、没有时钟。
  callbacks = full_callbacks(host);
  EXPECT_TRUE(callbacks.valid());
}

TEST(HostHal, TheClockIsAbsentUnlessTheHostSuppliesOne) {
  // Without `now_us` the clock reports 0 rather than inventing a timebase: a host with no clock must not be given the
  // impression that the library measured an interval.
  // 没有 `now_us` 时时钟报 0 而不是编一个时基：没有时钟的主控方不该产生"库量到了间隔"的印象。
  Host host{};
  driver::HalCallbacks callbacks = full_callbacks(host);
  driver::CallbackClock clock(callbacks);
  EXPECT_EQ(clock.now_us(), 0U);

  callbacks.now_us = [](void*) -> std::uint64_t { return 12345U; };
  clock.bind(callbacks);
  EXPECT_EQ(clock.now_us(), 12345U);
}

} // namespace foc
