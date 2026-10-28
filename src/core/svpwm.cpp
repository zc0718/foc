// Conan::ImportStart
#include <foc/core/svpwm.hpp>

#include <cmath>

#include <foc/core/limits.hpp>
#include <foc/core/transforms.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {

constexpr auto half() noexcept -> float { return static_cast<float>(1) / static_cast<float>(2); }

/// Sector per sign code N = a + 2b + 4c, index 0 unused (zero vector).
constexpr std::uint8_t kSectorFromCode[8] = {0U, 2U, 6U, 1U, 4U, 3U, 5U, 4U};


/**
 * @brief [en] Implements next_duty(); the contract is documented on its declaration.
 * @brief [zh] 实现 next_duty()；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto next_duty(float phase_v, float offset_v, float v_bus) noexcept -> float {
    return clamp(static_cast<float>(half() + ((phase_v - offset_v) / v_bus)), 0.0F, 1.0F);
}

} // namespace

auto Svpwm::modulate(Ab<float> v_ab, float v_bus) noexcept -> SvpwmResult {
    if (!(v_bus > 0.0F)) {
        return SvpwmResult{Uvw<float>{half(), half(), half()}, false, ErrorCode::Param};
    }

    const Uvw<float> phase = inv_clarke(v_ab);
    const float v_max =
        (phase.u > phase.v) ? ((phase.u > phase.w) ? phase.u : phase.w) : ((phase.v > phase.w) ? phase.v : phase.w);
    const float v_min =
        (phase.u < phase.v) ? ((phase.u < phase.w) ? phase.u : phase.w) : ((phase.v < phase.w) ? phase.v : phase.w);
    const float offset = static_cast<float>(half() * (v_max + v_min));

    const Uvw<float> duty{next_duty(phase.u, offset, v_bus), next_duty(phase.v, offset, v_bus),
                          next_duty(phase.w, offset, v_bus)};

    // Saturation means at least one phase wanted to leave [0, 1] before clamping.
    const float limit_high = static_cast<float>(half() * v_bus) + offset;
    const float limit_low = static_cast<float>(-half() * v_bus) + offset;
    const bool saturated = (v_max > limit_high) || (v_min < limit_low);

    return SvpwmResult{duty, saturated, ErrorCode::Ok};
}

/**
 * @brief [en] Implements modulate(const ModulatorInput&); the contract is documented on its declaration.
 * @brief [zh] 实现 modulate(const ModulatorInput&)；契约文档在对应的声明处。
 * @attacher
 * @since 0.1.0
 */
auto Svpwm::modulate(const ModulatorInput& request) noexcept -> SvpwmResult {
    // SVPWM itself has no use for the currents: it is a pure map from a stator voltage to duties.
    // The request carries them so that a stage in front of the modulator can compensate for what
    // the inverter really applies, and so that the two cannot drift apart by being asked separately.
    // SVPWM 本身用不到电流：它是"定子电压 → 占空比"的纯映射。请求带着电流，是为了让调制器前面的
    // 一级能补偿逆变器真正施加的电压，也为了让这两者不会因为分别询问而走偏。
    return modulate(request.v_ab, request.v_bus);
}

auto svpwm_sector(Ab<float> v_ab, float epsilon) noexcept -> std::uint8_t {
    if (!(std::fabs(v_ab.alpha) > epsilon) && !(std::fabs(v_ab.beta) > epsilon)) {
        return 0U;
    }

    const float a = v_ab.beta;
    const float b = static_cast<float>(half() * (kSqrt3Over2<float> * v_ab.alpha * 2.0F - v_ab.beta));
    const float c = static_cast<float>(half() * (-kSqrt3Over2<float> * v_ab.alpha * 2.0F - v_ab.beta));

    const std::uint8_t code =
        static_cast<std::uint8_t>(((a > 0.0F) ? 1U : 0U) + ((b > 0.0F) ? 2U : 0U) + ((c > 0.0F) ? 4U : 0U));

    return kSectorFromCode[code & 0x7U];
}

} // namespace core

} // namespace foc
