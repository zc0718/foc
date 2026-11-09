// Conan::ImportStart
#include <foc/core/min_pulse.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {

/**
 * @brief [en] The driver rule for one phase, on a duty already clamped into [0, 1].
 *
 *   Shared by `apply()` and `shape()` so the two can never disagree about what the bridge does with a narrow pulse.
 *
 * @brief [zh] 单相的驱动规则，作用在已钳进 [0, 1] 的占空比上。 `apply()` 与 `shape()` 共用它，于是两者对「桥臂拿窄脉冲
 *             怎么办」永远不会分歧。
 * @attacher
 * @since 0.1.0
 */
auto driver_rule(float duty, float min_duty) noexcept -> float {
    if (duty < min_duty) {
        return 0.0F; // the on-pulse is too short to be transmitted 导通脉冲短到传不出去
    }
    if ((1.0F - duty) < min_duty) {
        return 1.0F; // the off-pulse is too short, so the high side cannot turn off 关断脉冲太短，高侧关不掉
    }
    return duty;
}

/**
 * @brief [en] What one period's difference between requested and applied duty is worth in volt-seconds.
 * @brief [zh] 一个周期里「请求」与「实际」之差值多少伏秒。
 * @note [en] `(duty - applied) * T` is the width of the pulse that did not happen, and multiplying by the bus gives the
 *            volt-seconds the bridge will not deliver. The sign is kept: a stretched pulse is negative.
 * @note [zh] `(duty − applied)·T` 是那个没发生的脉冲的宽度，乘上母线就是桥臂不会交付的伏秒。符号保留：被拉宽的脉冲是负
 *            的。
 * @attacher
 * @since 0.1.0
 */
auto lost_volt_seconds(float requested, float applied, float period_s, float v_bus) noexcept -> float {
    return (requested - applied) * period_s * v_bus;
}

} // namespace

auto MinPulse::apply(Uvw<float> duty, float v_bus) const noexcept -> MinPulseOutput {
    if (validate() != ErrorCode::Ok || !std::isfinite(v_bus) || !(v_bus > 0.0F)) {
        return MinPulseOutput{ErrorCode::Param, duty, Uvw<float>{}, false};
    }
    const float m = min_duty();
    const float t = config_.switching_period_s;
    const float ru = clamp(duty.u, 0.0F, 1.0F);
    const float rv = clamp(duty.v, 0.0F, 1.0F);
    const float rw = clamp(duty.w, 0.0F, 1.0F);
    const float au = driver_rule(ru, m);
    const float av = driver_rule(rv, m);
    const float aw = driver_rule(rw, m);
    return MinPulseOutput{ErrorCode::Ok, Uvw<float>{au, av, aw},
                          Uvw<float>{lost_volt_seconds(ru, au, t, v_bus), lost_volt_seconds(rv, av, t, v_bus),
                                     lost_volt_seconds(rw, aw, t, v_bus)},
                          (au != ru) || (av != rv) || (aw != rw)};
}

auto MinPulse::shape(Uvw<float> duty, float v_bus) noexcept -> MinPulseOutput {
    if (validate() != ErrorCode::Ok || !std::isfinite(v_bus) || !(v_bus > 0.0F)) {
        return MinPulseOutput{ErrorCode::Param, duty, Uvw<float>{}, false};
    }
    const float m = min_duty();
    const float t = config_.switching_period_s;
    /**
     * @brief [en] The target is the request plus what the average still owes; the driver then does what it does, and
     *             what remains
     * @brief [zh] owed becomes the next period's carry. 目标是「请求 + 平均意义上还欠的」，随后驱动器做它该做的事，剩下
     *             的欠账成为 下一周期的余量。
     */
    const float tu = clamp(duty.u + carry_.u, 0.0F, 1.0F);
    const float tv = clamp(duty.v + carry_.v, 0.0F, 1.0F);
    const float tw = clamp(duty.w + carry_.w, 0.0F, 1.0F);
    const float au = driver_rule(tu, m);
    const float av = driver_rule(tv, m);
    const float aw = driver_rule(tw, m);
    carry_ = Uvw<float>{tu - au, tv - av, tw - aw};
    return MinPulseOutput{ErrorCode::Ok, Uvw<float>{tu, tv, tw},
                          Uvw<float>{lost_volt_seconds(tu, au, t, v_bus), lost_volt_seconds(tv, av, t, v_bus),
                                     lost_volt_seconds(tw, aw, t, v_bus)},
                          (au != clamp(duty.u, 0.0F, 1.0F)) || (av != clamp(duty.v, 0.0F, 1.0F)) ||
                              (aw != clamp(duty.w, 0.0F, 1.0F))};
}

} // namespace core

} // namespace foc
