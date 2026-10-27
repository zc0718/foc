#include <cstdint>

#include <gtest/gtest.h>

#include <foc/api/foc_api.h>
#include <foc/api/foc_types.h>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/ideal_observer.hpp>
#include <foc/core/alignment.hpp>
#include <foc/core/motion_profile.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/position.hpp>
#include <foc/core/reference.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/types.hpp>
#include <foc/driver/scripted_current.hpp>
#include <foc/driver/sim_position.hpp>
#include <foc/driver/stub_dc_link.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/driver/stub_pwm.hpp>
#include <foc/hal/clock.hpp>
#include <foc/hal/current_sense.hpp>
#include <foc/hal/dc_link.hpp>
#include <foc/hal/position.hpp>
#include <foc/hal/pwm.hpp>



namespace {

/**
 * @brief [en] Minimal HAL stubs: they exist only to prove the traits accept a conforming type.
 *        [zh] 最小 HAL 桩：只为证明 trait 接受符合契约的类型。
 */
class FakePwm final : public foc::hal::Pwm {
  public:
    auto configure(float, float) noexcept -> foc::ErrorCode override { return foc::ErrorCode::Ok; }

    void write_duty(foc::core::Uvw<float>) noexcept override { ++writes; }

    auto enable() noexcept -> foc::ErrorCode override { return foc::ErrorCode::Ok; }

    void disable() noexcept override {}

    auto fault() const noexcept -> bool override { return false; }

    std::int32_t writes{0};
};

class FakeCurrentSense final : public foc::hal::CurrentSense {
  public:
    auto read_uvw() noexcept -> foc::core::Uvw<float> override { return foc::core::Uvw<float>{}; }

    auto calibrate_offset() noexcept -> foc::ErrorCode override { return foc::ErrorCode::Ok; }

    auto sample_rate_hz() const noexcept -> float override { return 10'000.0F; }
};

class FakePosition final : public foc::hal::PositionSensor {
  public:
    auto angle_mech_rad() noexcept -> float override { return 0.0F; }

    auto velocity_rad_s() noexcept -> float override { return 0.0F; }

    auto has_sensor() const noexcept -> bool override { return true; }
};

class FakeDcLink final : public foc::hal::DcLink {
  public:
    auto voltage_v() noexcept -> float override { return 24.0F; }
};

class FakeClock final : public foc::hal::Clock {
  public:
    auto now_us() noexcept -> std::uint64_t override { return ticks; }

    std::uint64_t ticks{0};
};

/// [en] HalPolicy shapes for the optional-clock detection.
/// [zh] 用于可选时钟检测的 HalPolicy 形状。
struct ClockCarrier final {
    FakeClock clock;
};

struct NoClockCarrier final {
    int other{0};
};

struct WrongClockCarrier final {
    int clock{0};
};

/// Deliberately incomplete: missing write_duty/enable/disable/fault.
class BrokenPwm {
  public:
    auto configure(float, float) noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
};

/// Conforming observer, and one that returns the wrong type.
class FakeObserver {
  public:
    auto update(foc::core::Ab<float>, foc::core::Ab<float>, float) noexcept -> foc::core::ObsState {
        return foc::core::ObsState{};
    }
};

class WrongReturnObserver {
  public:
    auto update(foc::core::Ab<float>, foc::core::Ab<float>, float) noexcept -> float { return 0.0F; }
};

/// Conforming reference generator, and one that returns the wrong type.
class FakeReference {
  public:
    auto generate(const foc::core::ReferenceInput& in) noexcept -> foc::core::Dq<float> {
        return foc::core::Dq<float>{0.0F, in.iq_command_a};
    }
};

class WrongReference {
  public:
    auto generate(const foc::core::ReferenceInput&) noexcept -> float { return 0.0F; }
};

/// Conforming feedforward, and one that returns the wrong type.
class FakeFeedforward {
  public:
    auto generate(const foc::core::FeedforwardInput&) noexcept -> foc::core::Dq<float> {
        return foc::core::Dq<float>{};
    }
};

class WrongFeedforward {
  public:
    auto generate(const foc::core::FeedforwardInput&) noexcept -> float { return 0.0F; }
};

} // namespace



static_assert(foc::hal::concepts::is_pwm_v<FakePwm>, "FakePwm must satisfy foc::hal::Pwm");
static_assert(foc::hal::concepts::is_current_sense_v<FakeCurrentSense>,
              "FakeCurrentSense must satisfy foc::hal::CurrentSense");
static_assert(foc::hal::concepts::is_position_sensor_v<FakePosition>,
              "FakePosition must satisfy foc::hal::PositionSensor");
static_assert(foc::hal::concepts::is_dc_link_v<FakeDcLink>, "FakeDcLink must satisfy foc::hal::DcLink");
static_assert(foc::hal::concepts::is_clock_v<FakeClock>, "FakeClock must satisfy foc::hal::Clock");

static_assert(!foc::hal::concepts::is_pwm_v<BrokenPwm>, "an incomplete stub must fail the Pwm contract");
static_assert(!foc::hal::concepts::is_clock_v<FakeDcLink>, "traits must not cross-match contracts");

// Optional clock member on a HalPolicy: absent is legal, and a wrong-shaped `clock` counts as absent.
static_assert(foc::core::concepts::has_clock_v<ClockCarrier>, "a conforming clock member must be detected");
static_assert(!foc::core::concepts::has_clock_v<NoClockCarrier>, "no clock member means no time base");
static_assert(!foc::core::concepts::has_clock_v<WrongClockCarrier>,
              "an unrelated member named clock must be treated as absent, not as an error");
static_assert(!foc::core::concepts::has_clock_v<foc::driver::StubHal>,
              "the shipped stub HAL has no clock, so the overrun metric stays off");

// Observer policy contract (motor side) and its optional seed hook.
static_assert(foc::core::concepts::is_observer_v<FakeObserver>, "FakeObserver must satisfy the observer policy");
static_assert(foc::core::concepts::is_observer_v<foc::core::BackEmfObserver>,
              "BackEmfObserver must satisfy the observer policy");
static_assert(foc::core::concepts::is_observer_v<foc::core::IdealObserver>,
              "IdealObserver must satisfy the observer policy");
static_assert(!foc::core::concepts::is_observer_v<WrongReturnObserver>,
              "an observer returning the wrong type must be rejected");
static_assert(!foc::core::concepts::is_observer_v<FakeDcLink>,
              "a HAL stub is not an observer");
static_assert(foc::core::concepts::has_seed_flux_v<foc::core::BackEmfObserver>,
              "BackEmfObserver advertises seed_flux");
static_assert(!foc::core::concepts::has_seed_flux_v<foc::core::IdealObserver>,
              "IdealObserver takes the truth directly, so it must not advertise seed_flux");

// Reference-generator policy contract: the dq split is a policy, never a literal.
static_assert(foc::core::concepts::is_reference_generator_v<FakeReference>,
              "FakeReference must satisfy the reference-generator policy");
static_assert(foc::core::concepts::is_reference_generator_v<foc::core::NullReferenceGenerator>,
              "NullReferenceGenerator must satisfy the reference-generator policy");
static_assert(!foc::core::concepts::is_reference_generator_v<WrongReference>,
              "a generator returning the wrong type must be rejected");
static_assert(!foc::core::concepts::is_reference_generator_v<FakeObserver>,
              "an observer is not a reference generator");

// Feedforward policy contract: decoupling is a stage after the PI, not a wider PI signature.
static_assert(foc::core::concepts::is_feedforward_v<FakeFeedforward>,
              "FakeFeedforward must satisfy the feedforward policy");
static_assert(foc::core::concepts::is_feedforward_v<foc::core::NullFeedforward>,
              "NullFeedforward must satisfy the feedforward policy");
static_assert(foc::core::concepts::is_feedforward_v<foc::core::DecouplingFeedforward>,
              "DecouplingFeedforward must satisfy the feedforward policy");
static_assert(!foc::core::concepts::is_feedforward_v<WrongFeedforward>,
              "a feedforward returning the wrong type must be rejected");
static_assert(!foc::core::concepts::is_feedforward_v<FakeReference>,
              "a reference generator is not a feedforward stage");

// Controller-slot contract: one shape fills the two current axes and the speed loop. The position
// slot is deliberately *not* that shape any more -- it takes a move (see `PositionInput`), so the
// assertions for it live next to `motion_profile`/`position` where the widened contract is declared.
// 控制器槽位契约：一个形状填两个电流轴与速度环。位置槽位**故意**不再使用该形状——它接收的是一次
// 运动（见 `PositionInput`），因此针对它的断言放在声明加宽契约的地方。
static_assert(foc::core::concepts::is_controller_v<foc::core::Pi>, "Pi must satisfy the controller slot");
// `MotionProfile` keeps the three-argument shape as a convenience overload, so it satisfies this
// trait too; what matters is that it *also* satisfies the position slot, which is asserted where
// that contract is declared (test_position_p.cpp).
// `MotionProfile` 保留了带便利性质的三参数重载，因此它也满足本 trait；真正重要的是它**同时**满足
// 位置槽位，那条断言放在声明该契约的地方（test_position_p.cpp）。
static_assert(!foc::core::concepts::is_controller_v<FakeObserver>, "an observer is not a controller");
static_assert(!foc::core::concepts::is_controller_v<FakeDcLink>, "a HAL stub is not a controller");

// The shipped driver stubs are the reference implementations: they must satisfy the contracts.
static_assert(foc::hal::concepts::is_pwm_v<foc::driver::StubPwm>, "StubPwm must satisfy foc::hal::Pwm");
static_assert(foc::hal::concepts::is_current_sense_v<foc::driver::ScriptedCurrent>,
              "ScriptedCurrent must satisfy foc::hal::CurrentSense");
static_assert(foc::hal::concepts::is_position_sensor_v<foc::driver::SimPosition>,
              "SimPosition must satisfy foc::hal::PositionSensor");
static_assert(foc::hal::concepts::is_dc_link_v<foc::driver::StubDcLink>,
              "StubDcLink must satisfy foc::hal::DcLink");

static_assert(static_cast<std::int32_t>(foc::ErrorCode::Ok) == static_cast<std::int32_t>(FOC_OK),
              "foc::ErrorCode must mirror foc_err_t (Ok)");
static_assert(static_cast<std::int32_t>(foc::ErrorCode::Param) == static_cast<std::int32_t>(FOC_ERR_PARAM),
              "foc::ErrorCode must mirror foc_err_t (Param)");
static_assert(static_cast<std::int32_t>(foc::ErrorCode::NoResource) == static_cast<std::int32_t>(FOC_ERR_NO_RESOURCE),
              "foc::ErrorCode must mirror foc_err_t (NoResource)");
static_assert(static_cast<std::int32_t>(foc::ErrorCode::State) == static_cast<std::int32_t>(FOC_ERR_STATE),
              "foc::ErrorCode must mirror foc_err_t (State)");
static_assert(static_cast<std::int32_t>(foc::ErrorCode::NotImpl) == static_cast<std::int32_t>(FOC_ERR_NOT_IMPL),
              "foc::ErrorCode must mirror foc_err_t (NotImpl)");
static_assert(static_cast<std::int32_t>(foc::ErrorCode::Fault) == static_cast<std::int32_t>(FOC_ERR_FAULT),
              "foc::ErrorCode must mirror foc_err_t (Fault)");



TEST(TraitsContract, ConformingStubsAreAccepted) {
    FakePwm pwm;
    FakeCurrentSense sense;
    FakePosition position;
    FakeDcLink link;
    FakeClock clock;

    EXPECT_EQ(foc::ErrorCode::Ok, pwm.configure(20'000.0F, 500.0F));
    pwm.write_duty(foc::core::Uvw<float>{});
    EXPECT_EQ(1, pwm.writes);
    EXPECT_EQ(foc::ErrorCode::Ok, pwm.enable());
    pwm.disable();
    EXPECT_FALSE(pwm.fault());

    EXPECT_FLOAT_EQ(0.0F, sense.read_uvw().u);
    EXPECT_EQ(foc::ErrorCode::Ok, sense.calibrate_offset());
    EXPECT_FLOAT_EQ(10'000.0F, sense.sample_rate_hz());

    EXPECT_FLOAT_EQ(0.0F, position.angle_mech_rad());
    EXPECT_FLOAT_EQ(0.0F, position.velocity_rad_s());
    EXPECT_TRUE(position.has_sensor());

    EXPECT_FLOAT_EQ(24.0F, link.voltage_v());

    EXPECT_EQ(0U, clock.now_us());
    clock.ticks = 1'000U;
    EXPECT_EQ(1'000U, clock.now_us());
}



TEST(TraitsContract, ContractsWorkThroughBaseReferences) {
    FakePwm pwm;
    foc::hal::Pwm& base = pwm;

    base.write_duty(foc::core::Uvw<float>{0.5F, 0.5F, 0.5F});

    EXPECT_EQ(1, pwm.writes);
    EXPECT_EQ(foc::ErrorCode::Ok, base.configure(10'000.0F, 250.0F));
}



TEST(TraitsContract, TheAlignmentSlotIsCheckedWithTheSameStrictness) {
    // Both shipped aligners fit the slot, and the null one reports no result: "I did not align" is
    // not the same statement as "I aligned and here is the offset".
    // 两个内置对齐器都满足槽位；空实现不报结果：\"我没对齐\"与\"我对齐了，偏置在这里\"不是同一句话。
    static_assert(foc::core::concepts::is_aligner_v<foc::core::AlignRoutine>,
                  "AlignRoutine must satisfy the aligner contract");
    static_assert(foc::core::concepts::is_aligner_v<foc::core::NullAlignment>,
                  "NullAlignment must satisfy the aligner contract");
    static_assert(foc::core::concepts::has_align_offset_v<foc::core::AlignRoutine>,
                  "AlignRoutine must report the captured offset");
    static_assert(!foc::core::concepts::has_align_offset_v<foc::core::NullAlignment>,
                  "NullAlignment has no capture to report");
    SUCCEED();
}



TEST(TraitsContract, TheMechanicalOffsetHookIsOptionalAndDetected) {
    // A sensor may own the mounting-offset correction or may not; both are legal HAL
    // implementations, and the motor writes only to the ones that can take it.
    // 传感器可以自己拥有安装偏置校正，也可以不拥有；两种都是合法的 HAL 实现，电机只写给它接得住的那些。
    static_assert(foc::core::concepts::has_mech_offset_v<foc::driver::SimPosition>,
                  "SimPosition declares the optional offset hook");
    static_assert(!foc::core::concepts::has_mech_offset_v<FakePosition>,
                  "a sensor without the hook must be detected as absent, not as an error");

    foc::driver::SimPosition position;
    position.set_angle_mech_rad(1.0F);
    position.set_offset_mech_rad(0.25F);
    EXPECT_FLOAT_EQ(0.75F, position.angle_mech_rad());
    EXPECT_FLOAT_EQ(0.25F, position.mech_offset_rad());
}
