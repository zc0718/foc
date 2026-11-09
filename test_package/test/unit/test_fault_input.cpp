// Conan::ImportStart
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include <foc/core/protection.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/driver/scripted_current.hpp>
#include <foc/driver/sim_position.hpp>
#include <foc/driver/stub_dc_link.hpp>
#include <foc/driver/stub_fault_input.hpp>
#include <foc/driver/stub_pwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>
// Conan::ImportEnd

namespace {

// F39: the power stage's fault input.
// F39：功率级的故障输入。
//
// Two claims, and the order they are written in is the order they matter in.
//
//   1. **A HAL that cannot answer is refused.** Everywhere else in this HAL an absent capability degrades honestly --
//      no clock means no timebase. A missing *fault* input degrades toward "the power stage is fine", which is the one
//      answer nobody may give on the drive's behalf. So `start()` answers `NotImpl` and the axis stays in Idle with the
//      output stage disabled. This case is written first because it is the one that can be got wrong silently: a motor
//      that never asked is indistinguishable, from the outside, from one that asked and was told "all clear".
//   2. **The asserted input reaches `fault_code()`**, not just the state word. That distinction is the whole of F42's
//      criterion as well: a fault that only moves the state machine leaves the operator with "the drive tripped" and no
//      first question to ask.
// 两条论断，写的顺序就是它们重要的顺序。
//
//   1. **答不上来的 HAL 被拒绝。** 本 HAL 的其他地方，能力缺席会诚实降级——没有时钟就没有时基。而 **故障** 输入的
//      缺席是朝"功率级没事"降级的，那是没有人有资格替驱动器给出的答案。所以 `start()` 答 `NotImpl`，轴留在 Idle、输出级
//      保持关闭。这条放在最前面写，因为它是唯一可能被悄悄做错的一条：一台从不提问的电机，从外面看与一台问过并被告知
//      "一切正常"的电机一样。
//   2. **拉高的输入要落到 `fault_code()` 上** ，而不只是状态字。这个区别就是 F42 判据的全部：只让状态机动的故障，会给
//      运维留下"驱动器跳了"和没有一个可以问的第一个问题。

using foc::ErrorCode;
using foc::core::FaultCode;

constexpr float kDt = 50e-6F;
constexpr float kVBus = 24.0F;

auto make_config() -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U, 3.0F, 20000.0F, 0.0F};
}

auto make_policies() -> foc::core::Pi::Config {
    return foc::core::Pi::Config{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
}

// A HAL with everything a motor needs **except** the fault input: the shape this whole item is about.
// 一个除故障输入之外什么都不缺的 HAL：整个这一项说的就是这个形状。
struct HalWithoutFaultInput final {
    foc::driver::StubPwm pwm;
    foc::driver::ScriptedCurrent current_sense;
    foc::driver::SimPosition position;
    foc::driver::StubDcLink dc_link;
};

// The same HAL with it, so that a refusal can be attributed to the missing member rather than to the HAL being broken.
// 同一个 HAL 加上它，于是一次拒绝可以归因到缺的那个成员，而不是归因到"这个 HAL 本身坏了"。
struct HalWithFaultInput final {
    foc::driver::StubPwm pwm;
    foc::driver::ScriptedCurrent current_sense;
    foc::driver::SimPosition position;
    foc::driver::StubDcLink dc_link;
    foc::driver::StubFaultInput fault_input;
};

template <typename Hal>
using MotorOn = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, Hal>;

template <typename Hal> auto arm(Hal& hal) -> void {
    hal.dc_link.set_voltage_v(kVBus);
    hal.position.set_sensor_present(true);
    hal.position.set_angle_mech_rad(0.0F);
    hal.position.set_velocity_rad_s(0.0F);
    const foc::core::Uvw<float> zero{};
    static_cast<void>(hal.current_sense.load(&zero, 1U));
    static_cast<void>(hal.pwm.configure(20000.0F, 0.0F));
}

template <typename Hal> auto policies() -> typename MotorOn<Hal>::Policies {
    return typename MotorOn<Hal>::Policies{
        foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{}, foc::core::Pi{make_policies()},
        foc::core::Pi{make_policies()}, foc::core::Pi{foc::core::Pi::Config{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F}}};
}

// 1. The refusal, by name and by consequence: no start, no output stage, and the cause is the missing member.
// 1. 拒绝，按名字也按后果：不启动、输出级不动，而成因就是缺的那个成员。
TEST(FaultInput, AHostThatCannotReportItsPowerStageIsRefusedRatherThanAssumedHealthy) {
    // The trait is the machinery the refusal rests on, asserted first so a framework change cannot quietly make it
    // always-true: the same HAL reports false without the member and true with it.
    // 这个 trait 是拒绝所依赖的机制，先断言它，于是框架的变化不能悄悄让它恒为真：同一个 HAL 没有成员时报 false、有成员
    // 时报 true。
    static_assert(!foc::core::concepts::has_fault_input_v<HalWithoutFaultInput>,
                  "a HAL without a fault_input member must be detected as unable to answer");
    static_assert(foc::core::concepts::has_fault_input_v<HalWithFaultInput>,
                  "a HAL with a conforming fault_input member must be detected as able to answer");

    HalWithoutFaultInput hal{};
    arm(hal);
    MotorOn<HalWithoutFaultInput> motor(make_config(), policies<HalWithoutFaultInput>(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());

    EXPECT_EQ(ErrorCode::NotImpl, motor.start()) << "silence is not a clean bill of health";
    EXPECT_EQ(foc::core::MotorState::Idle, motor.state()) << "and the axis must not be left half-started";
    EXPECT_FALSE(hal.pwm.enabled()) << "nothing may be modulated by a drive that cannot be told to stop";
    EXPECT_EQ(FaultCode::None, motor.fault_code())
        << "a refused assembly is a usage error, not a fault the drive found";

    // The counterfactual, from the same fixture with one member added: the refusal is about the missing input, not about
    // the HAL being unusable.
    // 反事实，同一个夹具加一个成员：拒绝是关于缺的那个输入的，不是关于这个 HAL 不能用的。
    HalWithFaultInput ok_hal{};
    arm(ok_hal);
    MotorOn<HalWithFaultInput> ok_motor(make_config(), policies<HalWithFaultInput>(), ok_hal);
    ASSERT_EQ(foc::ErrorCode::Ok, ok_motor.init());
    EXPECT_EQ(foc::ErrorCode::Ok, ok_motor.start()) << "one member is the whole difference";
    EXPECT_TRUE(ok_hal.pwm.enabled());
    EXPECT_EQ(FaultCode::None, ok_motor.fault_code());
}

// 2. The asserted input becomes a cause, and the cause outlives the pin.
// 2. 拉高的输入变成一个成因，而且这个成因比引脚活得久。
TEST(FaultInput, TheAssertedPinReachesTheFaultCodeAndTheLatchOutlivesThePin) {
    HalWithFaultInput hal{};
    arm(hal);
    MotorOn<HalWithFaultInput> motor(make_config(), policies<HalWithFaultInput>(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    motor.set_torque(0.5F);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    ASSERT_EQ(FaultCode::None, motor.fault_code());

    // Inject, the way the pin would report it.
    // 注入，就像引脚会报的那样。
    hal.fault_input.set_faulted(true);
    EXPECT_EQ(ErrorCode::Fault, motor.tick(kDt));
    EXPECT_EQ(foc::core::MotorState::Fault, motor.state());
    EXPECT_EQ(FaultCode::DriveFault, motor.fault_code()) << "the state word alone is not a cause";
    EXPECT_FALSE(hal.pwm.enabled()) << "a tripped power stage stops being modulated on the same tick";

    // The pin going quiet does *not* resume the drive: the fault is latched, so a driver that reported a desaturation and
    // then reset itself still needs somebody to clear the fault. This is the half that a state-only implementation gets
    // wrong, because it re-reads the pin and finds nothing.
    // 引脚恢复安静 **不会** 让驱动复活：故障是锁存的，所以一个报了去饱和、随后自己复位的驱动器仍然需要有人来清除故障。
    // 这正是"只看状态"的实现做错的那一半——它会重读引脚，什么也没看到。
    hal.fault_input.set_faulted(false);
    EXPECT_EQ(ErrorCode::Fault, motor.tick(kDt)) << "the latch is the answer, not the pin";
    EXPECT_EQ(FaultCode::DriveFault, motor.fault_code());

    // And clearing is a decision with a name, not a timeout.
    // 而清除是一个有名字的决定，不是一次超时。
    ASSERT_EQ(foc::ErrorCode::Ok, motor.clear_fault());
    EXPECT_EQ(FaultCode::None, motor.fault_code());
    // And after a clear the machine is in Idle, not running: `tick()` answering `State` here is the difference between
    // "the fault is gone" and "somebody asked me to drive again", and the two must not be the same answer.
    // 而清除之后机器在 Idle、不在运行：`tick()` 在这里答 `State`，正是"故障没了"与"有人让我再驱动"之间的差别，两者不应
    // 是同一个答案。
    EXPECT_EQ(ErrorCode::State, motor.tick(kDt));
    EXPECT_EQ(FaultCode::None, motor.fault_code()) << "the pin is quiet, so the tick raises nothing new";
}

// 3. It outranks the running loop: a fault arriving mid-run stops the output in that tick, whatever the speed command
//    was doing. The plan's inversion of the usual order -- fault first, everything else after -- is the property under
//    test, so the case runs the machine and then asserts the very next tick.
// 3. 它压过正在跑的环：运行中途到来的故障在那一拍就停掉输出，不管速度给定在做什么。计划里"故障在前、其余在后"的次序就是
//    被测的性质，所以这个用例先把机器跑起来，然后断言紧接着的那一拍。
TEST(FaultInput, ThePinOutranksTheLoopItInterrupts) {
    HalWithFaultInput hal{};
    arm(hal);
    MotorOn<HalWithFaultInput> motor(make_config(), policies<HalWithFaultInput>(), hal);
    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    motor.set_torque(1.0F);
    for (int k = 0; k < 40; ++k) {
        ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
    }
    ASSERT_TRUE(hal.pwm.enabled());
    const auto duty_before = hal.pwm.last_duty();

    hal.fault_input.set_faulted(true);
    EXPECT_EQ(ErrorCode::Fault, motor.tick(kDt));
    EXPECT_EQ(FaultCode::DriveFault, motor.fault_code());
    EXPECT_FALSE(hal.pwm.enabled());
    // The duty register is left where it was: `disable()` is what tells the stage to stop, and a library that also wrote a
    // zero vector would be pretending it can choose what the hardware does while the hardware is reporting a fault.
    // 占空比寄存器留在原处：让输出级停下的是 `disable()`，而一个顺手写个零矢量的库，是在假装它能替一个正在报故障的硬件
    // 决定硬件该做什么。
    EXPECT_FLOAT_EQ(duty_before.u, hal.pwm.last_duty().u);
}

// 4. The stub answers what it was told and counts the tells, because a harness that asserted "the pin was raised" without
//    a write count can pass on a harness that wrote nothing.
// 4. 桩答它被告知的、并数次数——因为只断言"引脚被拉高了"而不数写入次数的 harness，会在一个什么都没写的 harness 上通过。
TEST(FaultInput, TheStubRecordsInjectionSoAHarnessCannotProveNothingHappened) {
    foc::driver::StubFaultInput input;
    EXPECT_FALSE(input.faulted());
    EXPECT_EQ(0U, input.change_count());

    input.set_faulted(true);
    EXPECT_TRUE(input.faulted());
    EXPECT_TRUE(input.is_faulted());
    EXPECT_EQ(1U, input.change_count());

    input.set_faulted(false);
    EXPECT_FALSE(input.faulted());
    EXPECT_EQ(2U, input.change_count());

    input.set_faulted(true);
    input.reset();
    EXPECT_FALSE(input.faulted());
    EXPECT_EQ(0U, input.change_count()) << "a reset is a power-on, not another injection";
}

// 5. The cause has a name on the C surface and it is the *first* question an operator asks. The range check in
//    `foc_bridge_fault_name` used to stop at `LostSync`; a new code that is appended without widening it would answer "?"
//    for a fault the library itself can raise, which is the shape this project keeps finding.
// 5. 这个成因在 C 表面上有名字，而那是运维问的 **第一个** 问题。`foc_bridge_fault_name` 的范围检查原来停在 `LostSync`；
//    追加一个新码而不放宽它，就会为一个库自己就会报的故障答 "?"，而这正是本项目反复发现的形状。
TEST(FaultInput, TheNewCauseIsLastAndStillLatched) {
    using foc::core::fault_is_latched;
    EXPECT_TRUE(fault_is_latched(FaultCode::DriveFault)) << "a driver that tripped is not a transient";
    EXPECT_LT(static_cast<int>(FaultCode::LostSync), static_cast<int>(FaultCode::DriveFault))
        << "appended, so every existing code keeps its number";
    EXPECT_EQ(static_cast<int>(FaultCode::DriveFault), static_cast<int>(FaultCode::None) + 8);
}

} // namespace
