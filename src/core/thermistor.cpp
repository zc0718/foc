// Conan::ImportStart
#include <foc/core/thermistor.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file thermistor.cpp
 *
 * @brief [en] Implements the divider inverse and the beta model's forward form, both pure arithmetic.
 * @brief [zh] 实现分压逆变换与 beta 模型的正向形式，两者都是纯算术。
 * @since 0.1.0
 */

namespace foc {

namespace core {

namespace {

/**
 * @brief [en] 25 degrees C in kelvin, the reference the beta model is written against.
 * @brief [zh] 25 摄氏度对应的开尔文值，beta 模型的参考点。
 * @note [en] Not a tuned constant: `273.15 + 25` is the definition of the model's reference temperature, and writing it
 *            as an expression keeps the two halves of it in one place.
 * @note [zh] 不是调出来的常数： `273.15 + 25` 就是该模型参考温度的定义，写成表达式让它的两半待在同一处。
 * @since 0.1.0
 */
constexpr float kKelvinAt25C = 273.15F + 25.0F;

/**
 * @brief [en] Absolute zero as an offset, for the degrees-C <-> kelvin conversion.
 * @brief [zh] 绝对零点的偏移量，用于摄氏度与开尔文互换。
 * @since 0.1.0
 */
constexpr float kKelvinOffset = 273.15F;

} // namespace

NtcDivider::NtcDivider(const ThermistorConfig& config) noexcept : config_(config) {}

auto NtcDivider::valid_config() const noexcept -> bool {
    return std::isfinite(config_.r25_ohm) && (config_.r25_ohm > 0.0F) && std::isfinite(config_.beta_k) &&
           (config_.beta_k > 0.0F) && std::isfinite(config_.series_ohm) && (config_.series_ohm > 0.0F);
}

auto NtcDivider::resistance_ohm(float celsius) const noexcept -> float {
    if (!valid_config() || !std::isfinite(celsius)) {
        return std::nanf("");
    }
    const float kelvin = celsius + kKelvinOffset;
    if (!(kelvin > 0.0F)) {
        return std::nanf("");
    }
    return config_.r25_ohm * std::exp(config_.beta_k * ((1.0F / kelvin) - (1.0F / kKelvinAt25C)));
}

auto NtcDivider::temperature_c(float code, float full_scale, float& out_c) const noexcept -> bool {
    if (!valid_config() || !std::isfinite(code) || !std::isfinite(full_scale) || !(full_scale > 0.0F)) {
        return false;
    }
    // A rail is not a temperature: it is what an open or a shorted thermistor produces, and on the curve both are
    // ordinary numbers. Refusing them is the difference between "cold" and "not there" surviving this function.
    // 轨不是温度：它是热敏电阻开路或短路所产生的东西，而两者在曲线上都是普通数字。拒绝它们，正是"很冷"与"不在"的
    // 区别能在这个函数里活下来的原因。
    if (!(code > 0.0F) || !(code < full_scale)) {
        return false;
    }
    const float ratio = code / full_scale;
    const float r_th = config_.thermistor_to_ground ? (config_.series_ohm * (ratio / (1.0F - ratio)))
                                                    : (config_.series_ohm * ((1.0F - ratio) / ratio));
    if (!std::isfinite(r_th) || !(r_th > 0.0F)) {
        return false;
    }
    const float kelvin = 1.0F / ((1.0F / kKelvinAt25C) + (std::log(r_th / config_.r25_ohm) / config_.beta_k));
    if (!std::isfinite(kelvin) || !(kelvin > 0.0F)) {
        return false;
    }
    out_c = kelvin - kKelvinOffset;
    return std::isfinite(out_c);
}

} // namespace core

} // namespace foc
