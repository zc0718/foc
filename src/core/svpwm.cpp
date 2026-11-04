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

/**
 * @brief Sector per sign code N = a + 2b + 4c, index 0 unused (zero vector).
 */
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

    const Uvw<float> requested = inv_clarke(v_ab);
    const float v_max = (requested.u > requested.v) ? ((requested.u > requested.w) ? requested.u : requested.w)
                                                    : ((requested.v > requested.w) ? requested.v : requested.w);
    const float v_min = (requested.u < requested.v) ? ((requested.u < requested.w) ? requested.u : requested.w)
                                                    : ((requested.v < requested.w) ? requested.v : requested.w);
    const float offset = static_cast<float>(half() * (v_max + v_min));
    // The whole linear-range condition, in one number: with the common-mode offset folded in, "a phase wants
    // to leave [0, 1]" is exactly `(v_max - v_min) > v_bus`. Strict, so the boundary itself is legal.
    // 整个线性区条件就是一个数：把共模偏置换进去后，"某一相想离开 [0,1]" 恰好等于 `(v_max - v_min) > v_bus`。
    // 严格比较，所以边界本身是合法的。
    const float spread = v_max - v_min;
    const bool saturated = spread > v_bus;

    // Overmodulation: scale the **vector** onto the hexagon instead of clamping each phase to a rail.
    // 过调制：把**矢量**缩放到六边形上，而不是把每一相各自夹到轨道上。
    //
    // The difference is not cosmetic and it is the whole reason this line exists. A per-phase clamp is one-sided:
    // only the phase that wanted out gets corrected, so the correction is not a common-mode term, and Clarke does
    // not remove it -- the applied vector both shrinks and **rotates** away from the command, and the rotation
    // grows with the request instead of saturating: measured on the old law, 0.09 deg at 1.02x the inscribed
    // circle, 18.90 deg at 3x, 23.30 deg at 5x, 26.63 deg at 10x and 29.60 deg at 100x -- it converges on 30 deg,
    // the half-sector that separates a commanded ray from the vertex it is being dragged to. Scaling is a common
    // factor on all three phases, so Clarke sees a pure magnitude change: the applied vector stays on the commanded
    // ray and its tip lands on the hexagon, which makes the delivered magnitude the stated law
    // `min(|v|, r_hex(theta))` rather than whatever the clamp happened to leave. It also keeps the asymptotic
    // fundamental: as the request grows the duties become a two-level waveform and the fundamental reaches
    // `2/pi*v_bus`, i.e. six-step, without a second algorithm.
    // 差别不是美观问题，而是这一行的全部理由。逐相夹紧是**单边**的：只有想出去的那一相被修正，于是修正项不是共模量，
    // Clarke 也消不掉它——实际施加的矢量既缩短又**旋转**，而且旋转量随请求**增大**、不会饱和：在老律上量到内切圆的
    // 1.02 倍处 0.09 度、3 倍处 18.90 度、5 倍处 23.30 度、10 倍处 26.63 度、100 倍处 29.60 度——它收敛到 30 度，即指令方向
    // 与被拖向的顶点之间的半个扇区。缩放是三相共同的比例因子，Clarke 看到的只是幅值变化：施加矢量仍**留在指令方向上**、
    // 末端落在六边形上，于是交付幅值成为一条写下来的律 `min(|v|, r_hex(θ))`，而不是夹紧碰巧剩下的东西。它还保住了
    // 渐近的基波：请求再大，占空比退化成两电平波形，基波到达 `2/π·v_bus`——即六步——**不需要第二套算法**。
    const float scale = saturated ? (v_bus / spread) : 1.0F;
    const Uvw<float> phase{requested.u * scale, requested.v * scale, requested.w * scale};
    const float scaled_offset = offset * scale;

    const Uvw<float> duty{next_duty(phase.u, scaled_offset, v_bus), next_duty(phase.v, scaled_offset, v_bus),
                          next_duty(phase.w, scaled_offset, v_bus)};

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
