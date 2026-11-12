// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file angle_source.hpp
 *
 * @brief [en] Which angle to trust this tick, and what to say when the answer is not trustworthy.
 * @brief [zh] 本拍该信哪个角度，以及当这个答案不可信时该怎么说。
 * @since 0.1.0
 */

namespace foc {

namespace core {

/**
 * @brief [en] Where an angle came from.
 * @brief [zh] 一个角度来自哪里。
 * @since 0.1.0
 * @exporter
 */
enum class AngleSourceKind : std::uint8_t {
    /**
     * @brief [en] nothing was offered: the arbiter has no opinion.
     * @brief [zh] 什么都没提供：仲裁器没有意见。
     */
    None = 0,
    /**
     * @brief [en] a position sensor.
     * @brief [zh] 位置传感器。
     */
    Sensor,
    /**
     * @brief [en] the flux observer.
     * @brief [zh] 磁链观测器。
     */
    Observer,
    /**
     * @brief [en] the hall decoder.
     * @brief [zh] 霍尔解码器。
     */
    Hall,
};

/**
 * @brief [en] How much the answer can be trusted.
 * @brief [zh] 这个答案可以信到什么程度。
 * @note [en] Three states rather than a boolean, because "wrong" and "unknown" call for opposite actions: a sensor that
 *            disagrees with the observer is a reason to keep the drive running at reduced authority and tell somebody,
 *            while an axis with nothing left is a reason to stop. A `bool` collapses those two into one word and the
 *            caller has to guess which was meant.
 * @note [zh] 三态而不是布尔，因为"错了"和"不知道"要求相反的动作：传感器与观测器互相不符，是降权继续跑并告诉人的理由；而
 *            一个什么都不剩的轴，是停下来的理由。 `bool` 会把这两件事压成同一个词，调用方只能猜是哪一个。
 * @since 0.1.0
 * @exporter
 */
enum class AngleQuality : std::uint8_t {
    /**
     * @brief [en] the sources agree; use it.
     * @brief [zh] 各来源彼此相符；可以用。
     */
    Ok = 0,
    /**
     * @brief [en] usable, but something about it is not right; the reason is in the reading.
     * @brief [zh] 能用，但其中有不对的地方；原因在读数里。
     */
    Degraded,
    /**
     * @brief [en] not usable: hold what you had and do not close a loop on this.
     * @brief [zh] 不可用：保持原状，不要用它闭合任何环。
     */
    Blind,
};

/**
 * @brief [en] One source's offer for this tick.
 * @brief [zh] 某个来源在本拍的报价。
 * @note [en] `valid == false` is an offer, not an absence: a sensor that is still wired but has stopped answering is
 *            the case this whole type exists for, and it can only be reported by naming it.
 * @note [zh] `valid == false` 是一份报价而不是缺席：传感器还接着、但已经不再回答，正是本类型存在的理由，而它只能靠点名
 *            来报告。
 * @since 0.1.0
 * @exporter
 */
struct AngleOffer final {
    /**
     * @brief [en] which source is speaking.
     * @brief [zh] 是哪个来源在说话。
     */
    AngleSourceKind kind;
    /**
     * @brief [en] whether this offer carries a usable measurement this tick.
     * @brief [zh] 这份报价本拍是否带着一个可用的测量。
     */
    bool valid;
    /**
     * @brief [en] mechanical angle, rad.
     * @brief [zh] 机械角，rad。
     */
    float angle_mech_rad;
    /**
     * @brief [en] mechanical speed, rad/s.
     * @brief [zh] 机械角速度，rad/s。
     */
    float speed_mech_rad_s;
};

/**
 * @brief [en] The numbers the arbitration rests on.
 * @brief [zh] 仲裁所依据的几个数。
 * @since 0.1.0
 * @exporter
 */
struct AngleSourceConfig final {
    /**
     * @brief [en] how long the preferred source may be missing before the arbiter moves on, s.
     * @brief [zh] 首选来源可以缺席多久，超过之后仲裁器才换源，s。
     */
    float handover_window_s;
    /**
     * @brief [en] disagreement between two live sources that already counts as degraded, rad.
     * @brief [zh] 两个都在的来源之间，达到多少分歧就算降级，rad。
     */
    float disagree_degraded_rad;
    /**
     * @brief [en] disagreement past which neither source can be believed, rad.
     * @brief [zh] 分歧超过多少时两个来源都不可信，rad。
     */
    float disagree_blind_rad;
};

/**
 * @brief [en] What the arbiter concluded this tick.
 * @brief [zh] 仲裁器本拍的结论。
 * @since 0.1.0
 * @exporter
 */
struct AngleReading final {
    /**
     * @brief [en] the angle to use, held at its last value while nothing is available.
     * @brief [zh] 要用的角度；什么都不available时保持上一次的值。
     */
    float angle_mech_rad;
    /**
     * @brief [en] the speed that goes with it; 0 while blind, because a held angle has no speed.
     * @brief [zh] 与之配套的转速；盲态时为 0，因为保持的角度没有速度。
     */
    float speed_mech_rad_s;
    /**
     * @brief [en] how far the answer can be trusted.
     * @brief [zh] 这个答案可以信到什么程度。
     */
    AngleQuality quality;
    /**
     * @brief [en] which source produced `angle_mech_rad`.
     * @brief [zh] `angle_mech_rad` 是哪个来源给出的。
     */
    AngleSourceKind source;
    /**
     * @brief [en] how far the two best live sources disagreed, rad; `NaN` when only one is live.
     * @brief [zh] 两个最好的活着的来源分歧多少，rad；只有一个活着时为 `NaN`。
     */
    float disagreement_rad;
    /**
     * @brief [en] how long the preferred source has been missing, s.
     * @brief [zh] 首选来源已经缺席多久，s。
     */
    float lost_s;
};

/**
 * @brief [en] Picks one angle, holds briefly when the best source stops answering, rates the answer.
 *
 *   Three rules, and each one is a decision that the naive alternative gets wrong:
 *
 *   1. **The caller states its trust order; this type does not invent one.** The offers arrive in priority order and
 *   the first valid one wins. Which sensor a machine trusts more than its observer is a fact about the machine.
 *
 *   2. **A source that stops answering is held for a window, then replaced.** Switching the instant a sample goes
 *   missing turns one dropped SPI frame into a source change; holding forever turns a dead encoder into a drive that
 *   believes a stale angle. The window is a number the caller sets, and the reading reports how long the preferred
 *   source has been gone so the decision can be audited after the fact.
 *
 *   3. **Two live sources that disagree are the only honest measure of being wrong.** There is no other way to know:
 *   nothing inside a sensor says "I am 0.3 rad off". So when two are alive, their disagreement is what decides `Ok`,
 *   `Degraded` or `Blind` -- and when only one is left, the arbiter says `Degraded` rather than `Ok`, because agreement
 *   has stopped being checkable at all.
 *
 *   What it is **not**: it is not an observer and it does not estimate anything. It has no model of the machine, and
 *   with a single source it can only report that no second opinion exists -- which is why the single-source state is
 *   `Degraded` and not `Ok`.
 *
 * @brief [zh] 从各份报价里挑一个角度，最好的那个不答时先坚持一下，并说明答案可信到什么程度。
 *
 *   三条规则，每一条都是"朴素的替代做法会做错"的决定：
 *
 *   1. **信任顺序由调用方给出，本类型不自己发明。** 报价按优先级到达，第一个有效的胜出。一台机器更信哪个传感器而不是它
 *   的观测器，是关于那台机器的事实。
 *
 *   2. **不再回答的来源先保持一个窗口，然后才被替换。** 一丢样本就换源，会把一次丢掉的 SPI 帧变成一次换源；永远保持，会
 *   把一个死掉的编码器变成一台相信陈旧角度的驱动器。窗口是调用方设定的一个数，而读数会报出首选来源已经缺席多久，于是这
 *   个决定事后可以审计。
 *
 *   3. **两个都活着的来源彼此不符，是唯一诚实的"我错了"的量度。** 没有别的办法知道：传感器内部没有任何东西会说 "我偏了
 *   0.3 rad"。所以两个都活着时，是它们的分歧决定 `Ok`/`Degraded`/`Blind`；而只剩一个时，仲裁器说 `Degraded` 而不是
 *   `Ok`，因为"相符"这件事已经无法核对了。
 *
 *   它 **不是** 什么：它不是观测器，也不估计任何东西。它没有机器的模型，而在只有一个来源时，它只能报告"没有第二意见" ——
 *   这正是单来源状态是 `Degraded` 而不是 `Ok` 的原因。
 *
 * @context [en] Current-loop ISR, every step, before the control frame uses an angle.
 * @context [zh] 电流环 ISR，每步，在控制帧用角度之前。
 * @since 0.1.0
 * @exporter
 */
class AngleSource final {
  public:
    /**
     * @brief [en] The most offers one call can carry.
     * @brief [zh] 一次调用最多能带几份报价。
     */
    static constexpr std::size_t kMaxOffers = 3U;

    /**
     * @brief [en] Accept the thresholds, or refuse them before a single tick runs.
     * @brief [zh] 接受这些阈值，或者在还没跑一拍之前就拒绝它们。
     * @param config [en] the window and the two disagreement thresholds.
     * @param config [zh] 窗口与两个分歧阈值。
     * @return [en] `Ok`, or `Param` for a window that is not positive, thresholds that are not positive, or a blind
     *              threshold that is not above the degraded one -- three numbers that cannot describe a band.
     * @return [zh] `Ok`；窗口不为正、阈值不为正、或盲阈值不高于降级阈值时返回 `Param`——三个无法构成区间的数。
     * @since 0.1.0
     */
    auto configure(const AngleSourceConfig& config) noexcept -> ErrorCode {
        const bool window_ok = (config.handover_window_s > 0.0F) && std::isfinite(config.handover_window_s);
        const bool bars_ok =
            (config.disagree_degraded_rad > 0.0F) && (config.disagree_blind_rad > config.disagree_degraded_rad);
        if (!window_ok || !bars_ok) {
            return ErrorCode::Param;
        }
        config_ = config;
        configured_ = true;
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] Decide this tick.
     * @brief [zh] 决定这一拍。
     * @param offers [en] `count` offers, **in the caller's trust order**; the first valid one is the candidate.
     * @param offers [zh] `count` 份报价， **按调用方的信任顺序** ；第一个有效的就是候选。
     * @param count [en] how many offers there are, at most `kMaxOffers`.
     * @param count [zh] 有几份报价，最多 `kMaxOffers`。
     * @param dt_s [en] step length in s; `<= 0` is refused as `Param` and changes nothing.
     * @param dt_s [zh] 步长，s； `<= 0` 被拒绝为 `Param` 且什么都不改。
     * @return [en] the reading; before the first step it reports `Blind` with a zero angle, which is the honest answer
     *              for "nobody has offered anything yet".
     * @return [zh] 读数；首拍之前它报告 `Blind` 与零角度，这对"还没有人提供过任何东西"是诚实的答案。
     * @post [en] A `Blind` reading holds the previous angle and reports zero speed, so a caller that ignores `quality`
     *            still cannot steer the machine with a held constant.
     * @post [zh] `Blind` 读数保持上一次的角度并报告零速度，于是忽略 `quality` 的调用方也无法用一个被保持的常数去操纵机
     *            器。
     * @since 0.1.0
     */
    auto step(const AngleOffer* offers, std::size_t count, float dt_s) noexcept -> const AngleReading& {
        if (!configured_ || !(dt_s > 0.0F) || (offers == nullptr) || (count == 0U)) {
            reading_.quality = AngleQuality::Blind;
            reading_.source = AngleSourceKind::None;
            reading_.speed_mech_rad_s = 0.0F;
            return reading_;
        }
        if (count > kMaxOffers) {
            count = kMaxOffers;
        }

        // The first valid offer in the caller's order, and the next valid one after it: the gap between those two is
        // the only measurement of correctness this type has.
        // 调用方顺序里第一个有效的报价，以及它之后的再一个：这两者之间的差距，是本类型唯一拥有的正确性量度。
        std::size_t best = count;
        std::size_t second = count;
        for (std::size_t i = 0U; i < count; ++i) {
            if (!offers[i].valid) {
                continue;
            }
            if (best == count) {
                best = i;
            } else if (second == count) {
                second = i;
            }
        }

        const bool preferred_missing = (best != 0U);
        reading_.lost_s = preferred_missing ? (reading_.lost_s + dt_s) : 0.0F;

        if (best == count) {
            // Nothing at all answered. Hold the angle, report zero speed, and say "blind" rather than "still fine".
            // 一个都没有回答。保持角度、报零速度，并说"盲"，而不是"仍然正常"。
            reading_.quality = AngleQuality::Blind;
            reading_.source = AngleSourceKind::None;
            reading_.speed_mech_rad_s = 0.0F;
            reading_.disagreement_rad = std::nanf("");
            return reading_;
        }

        const AngleOffer& chosen = offers[best];
        // Hold the previous source until the window has elapsed: a single missing sample is not a source change.
        // 在窗口走完之前保持上一个来源：丢一个样本不等于换一次源。
        if (preferred_missing && (source_ != AngleSourceKind::None) && (reading_.lost_s < config_.handover_window_s)) {
            reading_.quality = AngleQuality::Degraded;
            reading_.speed_mech_rad_s = 0.0F;
            reading_.disagreement_rad = std::nanf("");
            return reading_;
        }

        float disagreement = std::nanf("");
        if (second != count) {
            disagreement = std::fabs(std::remainder(chosen.angle_mech_rad - offers[second].angle_mech_rad, kTwoPiRad));
        }
        reading_.disagreement_rad = disagreement;

        // A hand-over carries the angle across the switch instead of jumping: the incoming source's own zero is not
        // the outgoing source's zero, and a step in the angle is a step in the current command.
        // 接管时把角度带过去而不是跳过去：进来的来源自己的零点不是出去的那个来源的零点，而角度上的一步就是电流给定上的
        // 一步。
        float angle = chosen.angle_mech_rad;
        if ((source_ != AngleSourceKind::None) && (chosen.kind != source_)) {
            const float before = reading_.angle_mech_rad;
            offset_rad_ = wrap(before - chosen.angle_mech_rad);
        }
        angle = wrap(angle + offset_rad_);

        reading_.angle_mech_rad = angle;
        reading_.speed_mech_rad_s = chosen.speed_mech_rad_s;
        reading_.source = chosen.kind;
        source_ = chosen.kind;

        if (preferred_missing) {
            // The window ran out and the answer now comes from a source the caller ranked lower: usable, not good.
            // 窗口用完了，答案现在来自调用方排名更低的来源：能用，但不算好。
            reading_.quality = AngleQuality::Degraded;
            return reading_;
        }
        if (std::isnan(disagreement)) {
            // Only one source is left, so agreement has stopped being checkable. That is not `Ok`; it is the absence
            // of the evidence `Ok` would need.
            // 只剩下一个来源，于是"相符"已经无从核对。这不是 `Ok`，而是 `Ok` 本当需要的那个证据不存在。
            reading_.quality = AngleQuality::Degraded;
            return reading_;
        }
        if (disagreement > config_.disagree_blind_rad) {
            reading_.quality = AngleQuality::Blind;
            reading_.speed_mech_rad_s = 0.0F;
            return reading_;
        }
        reading_.quality = (disagreement > config_.disagree_degraded_rad) ? AngleQuality::Degraded : AngleQuality::Ok;
        return reading_;
    }

    /**
     * @brief [en] The reading of the last step.
     * @brief [zh] 最近一步的读数。
     * @return [en] the reading; `Blind` with a zero angle before the first step.
     * @return [zh] 读数；首步之前为 `Blind` 与零角度。
     * @since 0.1.0
     */
    [[nodiscard]] auto reading() const noexcept -> const AngleReading& { return reading_; }

    /**
     * @brief [en] Forget the held angle, the offset and the lost timer; the next step is a first step.
     * @brief [zh] 忘掉保持的角度、偏移与计时；下一拍算第一拍。
     * @since 0.1.0
     */
    void reset() noexcept {
        reading_ = AngleReading{0.0F, 0.0F, AngleQuality::Blind, AngleSourceKind::None, std::nanf(""), 0.0F};
        offset_rad_ = 0.0F;
        source_ = AngleSourceKind::None;
        configured_ = false;
    }

  private:
    /**
     * @brief [en] Fold an angle difference into `[-pi, pi)`.
     * @brief [zh] 把角度差折进 `[-pi, pi)`。
     * @param delta_rad [en] an unbounded difference, rad.
     * @param delta_rad [zh] 未加约束的差，rad。
     * @return [en] the same rotation, in the half-open interval.
     * @return [zh] 同一个旋转，落在左闭右开区间里。
     * @note [en] `std::remainder` rather than a hand-rolled loop: it is the IEEE operation that already means "the
     *            symmetric remainder", and a second implementation of it would be a second answer to one question.
     * @note [zh] 用 `std::remainder` 而不是自己写循环：它本来就是 IEEE 里表示"对称余数"的那个运算，而第二份实现就是同一
     *            个问题上的第二个答案。
     * @since 0.1.0
     */
    [[nodiscard]] static auto wrap(float delta_rad) noexcept -> float { return std::remainder(delta_rad, kTwoPiRad); }

    static constexpr float kTwoPiRad = 6.28318530717958647692F;

    AngleSourceConfig config_{};
    AngleReading reading_{0.0F, 0.0F, AngleQuality::Blind, AngleSourceKind::None, std::nanf(""), 0.0F};
    float offset_rad_{0.0F};
    AngleSourceKind source_{AngleSourceKind::None};
    bool configured_{false};
};

} // namespace core

} // namespace foc
