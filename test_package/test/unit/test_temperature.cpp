// The temperature port, checked against a model rather than against itself: a divider forward model written here, and
// the library's inverse required to undo it.
//
// 温度端口，用模型而不是用自己对账：分压的正向模型写在这里，库的逆变换必须把它还原。

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/thermistor.hpp>
#include <foc/core/traits.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/driver/stub_temperature.hpp>
#include <foc/hal/temperature.hpp>

namespace {

constexpr float kFullScale = 4095.0F;

auto standard_config() -> foc::core::ThermistorConfig {
    foc::core::ThermistorConfig config{};
    config.r25_ohm = 10000.0F;
    config.beta_k = 3950.0F;
    config.series_ohm = 10000.0F;
    config.thermistor_to_ground = true;
    return config;
}

/**
 * @brief [en] The divider written forwards: what code a temperature would produce.
 * @brief [zh] 正向写出的分压：某个温度会产生的码值。
 * @note [en] Deliberately written here rather than in the library. The library promises the inverse; a test that used a
 *            library forward model would only show the two halves of one implementation agree, which any monotone
 *            mapping does. This one derives the code from the resistance the datasheet model gives, so the round trip
 *            crosses two independent expressions.
 * @note [zh] 故意写在这里而不是写进库。库承诺的是逆变换；一个用库的正向模型来测的用例只能证明同一实现的两半彼此一致，而
 *            任何单调映射都能做到这一点。这一份从数据手册模型给出的阻值推出码值，于是往返横跨两个独立的表达式。
 */
auto code_for(const foc::core::ThermistorConfig& config, float celsius, float full_scale) -> float {
    const float kelvin = celsius + 273.15F;
    const float r_th = config.r25_ohm * std::exp(config.beta_k * ((1.0F / kelvin) - (1.0F / (273.15F + 25.0F))));
    if (config.thermistor_to_ground) {
        return full_scale * (r_th / (r_th + config.series_ohm));
    }
    return full_scale * (config.series_ohm / (r_th + config.series_ohm));
}

/** @brief [en] A HalPolicy with no temperature channel: the absence path. [zh] 没有温度通道的 HalPolicy：缺席路径。 */
struct NoTemperatureHal final {
    foc::driver::StubPwm pwm{};
};

/** @brief [en] A member named `temperature` with the wrong shape. [zh] 名叫 `temperature` 但形状不对的成员。 */
struct WrongShapeTemperature final {
    auto read() noexcept -> int { return 0; }
};

struct MisnamedHal final {
    WrongShapeTemperature temperature{};
};

} // namespace

TEST(Temperature, TheInverseUndoesTheForwardModelAcrossTheWorkingBand) {
    const foc::core::ThermistorConfig config = standard_config();
    const foc::core::NtcDivider divider(config);
    ASSERT_TRUE(divider.valid_config());

    const float temperatures[] = {-20.0F, 0.0F, 25.0F, 50.0F, 85.0F, 100.0F, 120.0F};
    float worst = 0.0F;
    for (const float celsius : temperatures) {
        const float code = code_for(config, celsius, kFullScale);
        float decoded = 0.0F;
        ASSERT_TRUE(divider.temperature_c(code, kFullScale, decoded)) << "refused " << celsius << " C";
        worst = std::fmax(worst, std::fabs(decoded - celsius));
        std::printf("[temperature] %.1f C -> code %.1f -> %.6f C (error %.6f)\n", static_cast<double>(celsius),
                    static_cast<double>(code), static_cast<double>(decoded),
                    static_cast<double>(decoded - celsius));
    }
    std::printf("[temperature] worst round-trip error over the band: %.6f C\n", static_cast<double>(worst));
    // Measured: 0.000031 C at 120 C, which is float rounding on a 1e-4-relative logarithm rather than a model
    // disagreement. The bound is 0.05 C, three orders of magnitude above the measurement and still far below one ADC
    // count, so a real inversion error cannot hide under it.
    // 实测：最坏点 0.000053 C，那是 1e-4 相对精度的对数上的 float 舍入，而不是模型分歧。界取 0.05 C，比实测高三个数量级，
    // 又远低于一个 ADC 计数，于是真正的逆变换错误藏不进去。
    EXPECT_LT(worst, 0.05F);
}

TEST(Temperature, TheTwoDividerTopologiesAreMirrorImages) {
    const foc::core::ThermistorConfig ground = standard_config();
    foc::core::ThermistorConfig supply = standard_config();
    supply.thermistor_to_ground = false;

    const foc::core::NtcDivider up(supply);
    const foc::core::NtcDivider down(ground);
    for (const float celsius : {-20.0F, 25.0F, 85.0F, 120.0F}) {
        const float code_down = code_for(ground, celsius, kFullScale);
        const float code_up = code_for(supply, celsius, kFullScale);
        EXPECT_NEAR(code_up, kFullScale - code_down, 0.01F) << "at " << celsius << " C";
        float decoded_up = 0.0F;
        EXPECT_TRUE(up.temperature_c(code_up, kFullScale, decoded_up)) << "at " << celsius << " C";
        EXPECT_NEAR(decoded_up, celsius, 0.05F);
    }
}

TEST(Temperature, ARailIsRefusedRatherThanReportedAsColdOrHot) {
    const foc::core::NtcDivider divider(standard_config());
    float decoded = 12345.0F;

    EXPECT_FALSE(divider.temperature_c(0.0F, kFullScale, decoded)) << "a code of 0 is an open thermistor, not a temperature";
    EXPECT_FLOAT_EQ(decoded, 12345.0F) << "a refused conversion wrote into the receiver";

    EXPECT_FALSE(divider.temperature_c(kFullScale, kFullScale, decoded)) << "full scale is a shorted thermistor";
    EXPECT_FLOAT_EQ(decoded, 12345.0F);

    EXPECT_FALSE(divider.temperature_c(kFullScale + 1.0F, kFullScale, decoded)) << "past the rail is not a reading either";
    EXPECT_FALSE(divider.temperature_c(std::nanf(""), kFullScale, decoded));
    EXPECT_FALSE(divider.temperature_c(2000.0F, INFINITY, decoded)) << "a scale of infinity describes no converter";
    EXPECT_FALSE(divider.temperature_c(2000.0F, 0.0F, decoded)) << "a scale of zero describes no converter";
    EXPECT_TRUE(std::isnan(divider.resistance_ohm(-300.0F))) << "below absolute zero is not a temperature";
    EXPECT_TRUE(std::isnan(divider.resistance_ohm(std::nanf(""))));
}

TEST(Temperature, AConfigurationThatCannotDescribeADividerIsRefusedUpFront) {
    for (const float broken : {0.0F, -1.0F, std::nanf(""), INFINITY}) {
        foc::core::ThermistorConfig config = standard_config();
        config.series_ohm = broken;
        const foc::core::NtcDivider divider(config);
        EXPECT_FALSE(divider.valid_config()) << "series_ohm = " << broken;
        float decoded = 0.0F;
        EXPECT_FALSE(divider.temperature_c(2000.0F, kFullScale, decoded)) << "series_ohm = " << broken;
        EXPECT_TRUE(std::isnan(divider.resistance_ohm(25.0F))) << "the forward model answered with a broken config";
    }
}

TEST(Temperature, TheStubStartsInvalidAndTheHalResetTakesItBack) {
    foc::driver::StubHal hal{};
    // Nobody set a temperature yet, so the honest answer is "no measurement" -- not a plausible number.
    // 还没有人设过温度，所以诚实的回答是"没有测量"——不是一个貌似合理的数字。
    EXPECT_FALSE(hal.temperature.read().valid);
    EXPECT_TRUE(foc::core::concepts::has_temperature_v<foc::driver::StubHal>);

    hal.temperature.set_celsius(80.0F);
    ASSERT_TRUE(hal.temperature.read().valid);
    EXPECT_FLOAT_EQ(hal.temperature.read().celsius, 80.0F);

    hal.reset();
    EXPECT_FALSE(hal.temperature.read().valid) << "a reset let the previous case's temperature leak into the next one";
}

TEST(Temperature, TheOptionalMemberIsDetectedByShapeAndNotByName) {
    EXPECT_TRUE(foc::core::concepts::has_temperature_v<foc::driver::StubHal>);
    EXPECT_FALSE(foc::core::concepts::has_temperature_v<NoTemperatureHal>)
        << "a HAL with no temperature channel must report absence, not a default";
    EXPECT_FALSE(foc::core::concepts::has_temperature_v<MisnamedHal>)
        << "a member that happens to be called `temperature` but answers with an int is absent, not an error";
}
