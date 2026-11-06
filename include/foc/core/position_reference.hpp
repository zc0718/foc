// Conan::ImportStart
#pragma once
#include <cstdint>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] A continuous mechanical angle from an electrical estimate, with an index mark to fix its origin.
 *
 *     Why this exists: position control needs two things an observer cannot give on its own -- a *continuous* angle
 *     (turn counting, not a single-turn value) and an *absolute* one (a known zero). The observer supplies an
 *     electrical angle that wraps every `2*pi/pole_pairs` mechanically and starts wherever it happens to start.
 *     Unwrapping the wraps turns it into a continuous relative angle; an index pulse then fixes where zero is. Neither
 *     half alone is enough: an unwrapped angle cannot say which turn it is on, and an index without unwrapping says
 *     nothing about travel between two passes of the mark.
 *
 * @brief [zh] 由电角度估计构造的连续机械角度，并用索引标记把它的原点钉住。
 *
 *     存在的理由：位置控制需要两样观测器自己给不了的东西——**连续** 角度（能数圈，而不是单圈值）和**绝对**角度（已知的零
 *     点）。观测器给出的是每 `2*pi/极对数` 就折一次的机械角度，而且从哪里开始就从哪里开始。把折返解开，就得到连续的相对
 *     角度；再由索引脉冲把零点钉住。两半缺一不可：解开折返的角度说不出自己在第几圈，而没有解折返的索引对两次经过标记之
 *     间的行程一无所知。
 *
 * @note [en] The reconstruction assumes the electrical angle advances by less than `pi` between samples. That is a
 *            constraint on the caller (control period x electrical speed), not a tuning knob, and `update()` reports
 *            having seen an implausible step rather than silently choosing a direction. At the nominal 20 kHz and 7
 *            pole pairs the limit is about 9000 rad/s mechanical, far above any machine this library targets.
 * @note [zh] 重建假定两次采样之间电角度前进不足 `pi`。这是对调用方的约束（控制周期 × 电角速度），不是可调参数；
 *            `update()` 遇到不可能的跳变会**报告**出来，而不是静默地选一个方向。在标称 20 kHz、7 极对下，该上限约为机械
 *            9000 rad/s，远高于本库面向的任何机器。
 * @since 0.1.6
 * @exporter
 */
class PositionReference final {
  public:
    /**
     * @brief [en] Pole pairs, and where the index mark physically sits.
     * @brief [zh] 极对数，以及索引标记在机械角上的实际位置。
     * @note [en] `index_mech_rad` is the mechanical angle the *index* stands for -- usually 0, but a fixture may define
     *            its zero elsewhere and this is how that is expressed without a second offset field.
     * @note [zh] `index_mech_rad` 是索引所代表的机械角度——通常是 0，但工装可以把零点定义在别处，这就是表达它的方式，不
     *            需要再开一个偏置字段。
     * @since 0.1.6
     */
    struct Config final {
        std::uint8_t pole_pairs{1U};
        float index_mech_rad{0.0F};
    };

    explicit PositionReference(Config config) noexcept;

    /**
     * @brief [en] Advance by one sample of the electrical angle, optionally with the index pulse for this tick.
     * @brief [zh] 用一次电角度采样推进，可携带本 tick 的索引脉冲。
     * @param theta_e_rad [en] the electrical angle in rad, any range.
     * @param theta_e_rad [zh] 电角度，rad，任意范围。
     * @param index_pulse [en] true on the tick the mark was crossed.
     * @param index_pulse [zh] 本 tick 越过标记时为 true。
     * @return [en] `true` if this tick established (or re-established) the origin. `false` otherwise, including when a
     *              step was rejected.
     * @return [zh] 本 tick 建立（或重建）了原点时为 `true`。其他情况为 `false`，包括跳变被拒绝时。
     * @context [en] Current-loop ISR: noexcept, no allocation.
     * @context [zh] 电流环 ISR：noexcept、不分配。
     * @since 0.1.6
     */
    auto update(float theta_e_rad, bool index_pulse) noexcept -> bool;

    /**
     * @brief [en] The continuous angle since the first sample, in mechanical rad, without an origin.
     * @brief [zh] 自第一次采样以来的连续机械角度，无原点。
     * @note [en] This is the right quantity for relative motion and the wrong one for a position command: it starts at
     *            whatever the first sample was. Use `referenced_mech_rad()` once `homed()`.
     * @note [zh] 相对运动用它是对的，位置指令用它就是错的：它的起点就是第一次采样的值。`homed()` 之后请用
     *            `referenced_mech_rad()`。
     * @since 0.1.6
     */
    [[nodiscard]] auto unwrapped_mech_rad() const noexcept -> float { return unwrapped_mech_rad_; }

    /**
     * @brief [en] The mechanical angle referred to the index mark.
     * @brief [zh] 以索引标记为基准的机械角度。
     * @return [en] A continuous angle that reads `Config::index_mech_rad` at the mark and keeps counting turns away
     *              from it; `0` before the first index, which is why `homed()` must be consulted rather than trusted to
     *              be true.
     * @return [zh] 在标记处读数为 `Config::index_mech_rad`、并继续累计圈数的连续角度；第一次索引之前为 `0`——这正是必须
     *              查 `homed()` 而不是假定它成立的原因。
     * @since 0.1.6
     */
    [[nodiscard]] auto referenced_mech_rad() const noexcept -> float { return referenced_mech_rad_; }

    /** @brief [en] Whether an index has been seen since construction or reset. [zh] 自构造或复位以来是否见过索引。 */
    [[nodiscard]] auto homed() const noexcept -> bool { return homed_; }

    /**
     * @brief [en] Whether the reconstruction is no longer trustworthy: a step too large to unwrap, or a mark that
     *             disagrees with the count by more than the channel can resolve.
     * @brief [zh] 重建是否已不可信：无法解开折返的跳变，或与计数相差超过通道分辨力的标记。
     * @note [en] Sticky until `reset()` or the next accepted mark. A reconstruction that quietly guessed its way
     *            through a jump would produce a plausible angle that is off by a whole turn, which is worse than saying
     *            it lost count. The index path is guarded because a spurious pulse would otherwise move the origin by
     *            up to half a mechanical turn and nothing could tell: the same rule the electrical step follows, in the
     *            same units.
     * @note [zh] 一直保持到 `reset()` 或下一个被接受的标记。悄悄猜过去的重建会给出一个"看起来合理"却差了整整一圈的角
     *            度，那比承认丢了计数更糟。索引通路也受同一个护栏约束，否则一个虚假脉冲会把原点移走最多半个机械圈而无从
     *            得知——与电角度跳变遵守同一条规则、同一个单位。
     * @since 0.1.6
     */
    [[nodiscard]] auto lost_count() const noexcept -> bool { return lost_count_; }

    /**
     * @brief [en] Forget the origin, the turn count and any jump, keeping the configuration.
     * @brief [zh] 忘掉原点、圈数与跳变，保留配置。
     * @since 0.1.6
     */
    void reset() noexcept;

  private:
    Config config_;
    /**
     * @brief [en] The shift that makes the unwrapped angle read `index_mech_rad` at the mark.
     *
     *     It is only ever corrected, never redefined.
     *
     * @brief [zh] 把解开折返的角度平移多少才在标记处读到 `index_mech_rad`。
     *
     *     它只会被修正，不会被重新定义。
     */
    float index_offset_rad_;
    float previous_theta_e_rad_;
    float unwrapped_mech_rad_;
    float referenced_mech_rad_;
    bool primed_;
    bool homed_;
    bool lost_count_;
};


} // namespace core

} // namespace foc
