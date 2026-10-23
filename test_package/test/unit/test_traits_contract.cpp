#include <cstdint>

#include <gtest/gtest.h>

#include <foc/api/foc_api.h>
#include <foc/api/foc_types.h>
#include <foc/core/traits.hpp>
#include <foc/core/types.hpp>
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

/// Deliberately incomplete: missing write_duty/enable/disable/fault.
class BrokenPwm {
  public:
    auto configure(float, float) noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
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
