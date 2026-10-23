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
