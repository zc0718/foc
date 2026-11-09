// Conan::ImportStart
#include <foc/core/single_shunt.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {

/**
 * @brief [en] The three duties in descending order, plus which phase each one belongs to.
 *
 *   One pass, no sorting of an array, because "which phase is the largest" is the thing the reconstruction needs and
 *   sorting indices is the place a sector table would have crept back in.
 *
 * @brief [zh] 三个占空比按从大到小，以及每一个属于哪一相。一次遍历、不排数组，因为重构需要的正是「哪一相最大」，而排序
 *             索引正是扇区表会趁机溜回来的地方。
 * @attacher
 * @since 0.1.0
 */
struct Ranked final {
    float max_v;
    float mid_v;
    float min_v;
    /**
     * @brief [en] Which phase each duty belongs to: 0 = u, 1 = v, 2 = w.
     * @brief [zh] 每个占空比属于哪一相：0 = u，1 = v，2 = w。
     */
    std::uint8_t max_k;
    std::uint8_t mid_k;
    std::uint8_t min_k;
};

auto rank(const Uvw<float>& duty) noexcept -> Ranked {
    const float d[3] = {duty.u, duty.v, duty.w};
    std::uint8_t order[3] = {0U, 1U, 2U};
    const auto worse = [&](std::uint8_t a, std::uint8_t b) noexcept -> bool { return d[a] < d[b]; };
    // Three comparisons, three swaps: a sorting network for n = 3, written out so the indices travel with the values.
    // 三次比较、三次交换：n = 3 的排序网络，写开是为了让索引跟着数值一起走。
    if (worse(order[0], order[1])) {
        const std::uint8_t t = order[0];
        order[0] = order[1];
        order[1] = t;
    }
    if (worse(order[1], order[2])) {
        const std::uint8_t t = order[1];
        order[1] = order[2];
        order[2] = t;
    }
    if (worse(order[0], order[1])) {
        const std::uint8_t t = order[0];
        order[0] = order[1];
        order[1] = t;
    }
    return Ranked{d[order[0]], d[order[1]], d[order[2]], order[0], order[1], order[2]};
}

auto sanitised(const Uvw<float>& duty) noexcept -> Uvw<float> {
    return Uvw<float>{clamp(duty.u, 0.0F, 1.0F), clamp(duty.v, 0.0F, 1.0F), clamp(duty.w, 0.0F, 1.0F)};
}

auto out_of_range(const Uvw<float>& duty) noexcept -> bool {
    return !(duty.u >= 0.0F && duty.u <= 1.0F && duty.v >= 0.0F && duty.v <= 1.0F && duty.w >= 0.0F && duty.w <= 1.0F);
}

} // namespace

auto SingleShunt::classify(Uvw<float> duty) const noexcept -> ShuntWindow {
    const ShuntWindow empty{ErrorCode::Param, ShuntVerdict::WindowTooNarrow, ShuntTimes{}, 0.0F, needed_s()};
    if (validate() != ErrorCode::Ok || out_of_range(duty)) {
        return empty;
    }
    const Ranked r = rank(sanitised(duty));
    const float period = config_.switching_period_s;
    const ShuntTimes times{(r.max_v - r.mid_v) * period, (r.mid_v - r.min_v) * period,
                           (1.0F - r.max_v + r.min_v) * period};
    // Half of the shorter active interval: a 7-segment pattern applies each active vector twice per period.
    // 较短那个有效区间的一半：七段式图样每周期把每个有效矢量施加两次。
    const float window = 0.5F * ((times.t_active1_s < times.t_active2_s) ? times.t_active1_s : times.t_active2_s);
    ShuntVerdict verdict = ShuntVerdict::Measurable;
    if (!(times.t_active1_s > 0.0F) && !(times.t_active2_s > 0.0F)) {
        verdict = ShuntVerdict::NoActiveVector;
    } else if (!(window >= needed_s())) {
        verdict = ShuntVerdict::WindowTooNarrow;
    }
    return ShuntWindow{ErrorCode::Ok, verdict, times, window, needed_s()};
}

auto SingleShunt::reconstruct(Uvw<float> duty, float i_dc_active1_a, float i_dc_active2_a) const noexcept
    -> ShuntCurrents {
    const ShuntWindow window = classify(duty);
    if (window.status != ErrorCode::Ok) {
        return ShuntCurrents{ErrorCode::Param, false, Uvw<float>{}};
    }
    if (window.verdict != ShuntVerdict::Measurable) {
        // The window is too narrow (or there is no active vector at all). Returning a number here would be returning a
        // reading taken during a transition, which is not a noisy measurement but a wrong one -- so the currents stay at
        // zero and `valid` says why. 窗口太窄（或者根本没有有效矢量）。在这里返回一个数，等于返回一个在换向过程中读到的
        // 读数，那不是「有噪声的测量」而是「错的测量」——所以电流留零，由 `valid` 说明原因。
        return ShuntCurrents{ErrorCode::Ok, false, Uvw<float>{}};
    }
    const Ranked r = rank(sanitised(duty));
    float i[3] = {0.0F, 0.0F, 0.0F};
    i[r.max_k] = i_dc_active1_a;
    i[r.min_k] = -i_dc_active2_a;
    i[r.mid_k] = -(i[r.max_k] + i[r.min_k]);
    return ShuntCurrents{ErrorCode::Ok, true, Uvw<float>{i[0], i[1], i[2]}};
}

} // namespace core

} // namespace foc
