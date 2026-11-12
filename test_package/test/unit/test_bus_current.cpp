// The bus-current port: an optional capability on the DC link, plus the identity that makes one number checkable.
//
// 母线电流端口：直流母线上的可选能力，以及让这个数可核对的那条恒等式。

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/traits.hpp>
#include <foc/core/types.hpp>
#include <foc/driver/stub_dc_link.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/hal/dc_link.hpp>

#include "sil_bus.hpp"

namespace {

/** @brief [en] A link that measures voltage only: the normal case for a bus fed from a supply. [zh] 只测电压的母线。 */
class VoltageOnlyLink final : public foc::hal::DcLink {
  public:
    auto voltage_v() noexcept -> float override { return 24.0F; }
};

/** @brief [en] A link whose bus-current member answers with the wrong type. [zh] 母线电流成员返回错误类型的母线。 */
class WrongShapeLink final : public foc::hal::DcLink {
  public:
    auto voltage_v() noexcept -> float override { return 24.0F; }
    auto read_bus_current_a(float& /*out*/) noexcept -> int { return 0; }
};

struct VoltageOnlyHal final {
    foc::driver::StubPwm pwm{};
    VoltageOnlyLink dc_link{};
};

struct WrongShapeHal final {
    foc::driver::StubPwm pwm{};
    WrongShapeLink dc_link{};
};

/**
 * @brief [en] The inverse Park and Clarke written out, peak-valued, with no library call.
 * @brief [zh] 手写出来的逆 Park 与逆 Clarke，幅值不变型，不调库。
 * @return [en] the three phase voltages or currents, V or A.
 * @return [zh] 三相电压或电流，V 或 A。
 * @note [en] Written here on purpose: the library's own transforms would agree with the library's own power factor
 *            whatever it happened to be. This expression is the definition of the amplitude-invariant convention, so
 *            the sum it produces is an independent statement about the `3/2`.
 * @note [zh] 故意写在这里：用库自己的变换去对账，无论那个功率系数是多少都会一致。这个表达式是幅值不变约定的定义，于是它
 *            给出的和是对 `3/2` 的一个独立陈述。
 */
struct Abc final {
    float u{0.0F};
    float v{0.0F};
    float w{0.0F};
};

auto to_abc(float d, float q, float theta_rad) -> Abc {
    const float c = std::cos(theta_rad);
    const float s = std::sin(theta_rad);
    const float alpha = (d * c) - (q * s);
    const float beta = (d * s) + (q * c);
    constexpr float kSqrt3Over2 = 0.86602540378F;
    Abc out{};
    out.u = alpha;
    out.v = (-0.5F * alpha) + (kSqrt3Over2 * beta);
    out.w = (-0.5F * alpha) - (kSqrt3Over2 * beta);
    return out;
}

} // namespace

TEST(BusCurrent, TheStubStartsWithoutAReadingAndResetTakesItBack) {
    foc::driver::StubHal hal{};
    float amps = 12345.0F;
    EXPECT_FALSE(hal.dc_link.read_bus_current_a(amps)) << "a bus current nobody set is not a measurement";
    EXPECT_FLOAT_EQ(amps, 12345.0F) << "a refused read wrote into the receiver";

    hal.dc_link.set_bus_current_a(2.5F);
    ASSERT_TRUE(hal.dc_link.read_bus_current_a(amps));
    EXPECT_FLOAT_EQ(amps, 2.5F);

    hal.reset();
    EXPECT_FALSE(hal.dc_link.read_bus_current_a(amps)) << "a reset let the previous case's current leak forward";
}

TEST(BusCurrent, TheCapabilityIsDetectedOnTheLinkAndNotOnAMemberName) {
    EXPECT_TRUE(foc::core::concepts::has_bus_current_v<foc::driver::StubHal>);
    EXPECT_FALSE(foc::core::concepts::has_bus_current_v<VoltageOnlyHal>)
        << "a bus fed from a supply measures no current, and that has to be answerable rather than assumed";
    EXPECT_FALSE(foc::core::concepts::has_bus_current_v<WrongShapeHal>)
        << "a member answering int is absent, not a compile error somewhere else";
}

TEST(BusCurrent, TheDqPowerFormAndThePhaseFormAgreeAtEveryAngle) {
    // The law: for the amplitude-invariant convention this library uses, `sum(v_k * i_k)` is exactly
    // `1.5*(v_d*i_d + v_q*i_q)` -- the same number for every rotor angle, because the rotation preserves power.
    // 定律：对本库使用的幅值不变约定，`sum(v_k * i_k)` 精确等于 `1.5*(v_d*i_d + v_q*i_q)`——因为旋转保功率，所以对
    // 每一个转子角都是同一个数。
    constexpr float kVd = 3.0F;
    constexpr float kVq = -2.0F;
    constexpr float kId = 1.2F;
    constexpr float kIq = 0.7F;
    constexpr float kVBus = 24.0F;

    float worst_relative = 0.0F;
    for (const float theta : {0.0F, 0.37F, 1.0F, 2.5F, -1.2F}) {
        const Abc v_abc = to_abc(kVd, kVq, theta);
        const Abc i_abc = to_abc(kId, kIq, theta);
        const float phase_power = (v_abc.u * i_abc.u) + (v_abc.v * i_abc.v) + (v_abc.w * i_abc.w);

        const double current_a =
            foc::sil::bus_current_a(foc::core::Dq<float>{kVd, kVq}, foc::core::Dq<float>{kId, kIq}, kVBus);
        const auto bus_power = static_cast<float>(current_a * static_cast<double>(kVBus));
        const float relative = std::fabs(bus_power - phase_power) / std::fmax(std::fabs(phase_power), 1.0F);
        worst_relative = std::fmax(worst_relative, relative);
        std::printf("[bus-current] theta %+.2f: phase sum %.6f W, d-q power %.6f W (relative %.3e)\n",
                    static_cast<double>(theta), static_cast<double>(phase_power), static_cast<double>(bus_power),
                    static_cast<double>(relative));
        EXPECT_NEAR(bus_power, phase_power, 0.01F) << "at theta " << theta;
    }
    std::printf("[bus-current] worst relative disagreement across angles: %.3e\n", static_cast<double>(worst_relative));
}

TEST(BusCurrent, ABusOfZeroVoltsAnswersZeroRatherThanDividing) {
    const double current_a = foc::sil::bus_current_a(foc::core::Dq<float>{3.0F, -2.0F}, foc::core::Dq<float>{1.2F, 0.7F},
                                                    0.0F);
    EXPECT_DOUBLE_EQ(current_a, 0.0);
}
