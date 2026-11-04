// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>

#include <foc/core/limits.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] What one control period's switching pattern leaves for a single shunt to measure.
 * @brief [zh] 单个采样电阻在一个控制周期的开关图样里，能用来测量的是什么。
 * @note [en] The two active intervals are the ones a 7-segment pattern applies **twice** per period, so the usable
 *            window is half of each: `min(t_active1, t_active2)/2`. That half is not a detail -- it is where an
 *            otherwise-correct design fails by a factor of two at the modulation edge.
 * @note [zh] 两个有效区间在七段式图样里每个周期被**分两次**施加，所以可用窗口是它们各自的一半： `min(t_active1,
 *            t_active2)/2`。这个「一半」不是细节——正是在这里，一个别的方面都正确的设计会在调制边界上差一个两倍。
 * @since 0.1.9
 * @exporter
 */
struct ShuntTimes final {
    float
        t_active1_s; ///< [en] only the largest-duty phase conducts: `(dmax - dmid)*T`, s. [zh] 只有占空比最大的那一相导通：`(dmax − dmid)·T`，s。
    float
        t_active2_s; ///< [en] the two largest conduct: `(dmid - dmin)*T`, s. [zh] 最大的两相导通：`(dmid − dmin)·T`，s。
    float t_zero_s;  ///< [en] all three low: `(1 - dmax + dmin)*T`, s. [zh] 三相全低：`(1 − dmax + dmin)·T`，s。
};

/**
 * @brief [en] Whether the currents can be measured at all this period -- three answers, because two are different
 *             failures.
 * @brief [zh] 本周期到底能不能测——三种答案，因为其中两种是不同的失败。
 * @note [en] `WindowTooNarrow` and `NoActiveVector` are not the same thing and must not be collapsed into "not
 *            measurable": the first says *this* period cannot be measured and the next one probably can (the reference
 *            is somewhere near a sector boundary), the second says no current is flowing through the shunt at all, so
 *            there is nothing to measure and nothing to miss.
 * @note [zh] `WindowTooNarrow` 与 `NoActiveVector` 不是一回事，不许合并成「测不了」：前者说的是**这个**周期测不了、下一
 *            个大概就行（给定落在扇区边界附近），后者说的是电流根本不流经采样电阻，没什么可测、也没什么可漏。
 * @since 0.1.9
 * @exporter
 */
enum class ShuntVerdict : std::uint8_t {
    Measurable = 0, ///< [en] both windows are long enough. [zh] 两个窗口都够长。
    WindowTooNarrow, ///< [en] at least one active interval is shorter than settle + sample. [zh] 至少一个有效区间短于建立加采样。
    NoActiveVector, ///< [en] both vanish: the period is one zero vector and the shunt carries nothing. [zh] 两者都消失：整周期是一个零矢量，采样电阻上没有电流。
};

/**
 * @brief [en] The verdict, the times it was reached from, and the one number that decided it.
 * @brief [zh] 结论、得出结论所用到的那些时间，以及决定它的那一个数。
 * @since 0.1.9
 * @exporter
 */
struct ShuntWindow final {
    ErrorCode
        status; ///< [en] Ok, or Param for a configuration that cannot be judged. [zh] Ok；配置无法判定时为 Param。
    ShuntVerdict verdict; ///< [en] three answers, not a boolean. [zh] 三种答案，不是一个布尔。
    ShuntTimes times;     ///< [en] the pattern's three intervals. [zh] 图样的三个区间。
    float
        window_s; ///< [en] `min(t_active1, t_active2)/2`: what is actually left to sample. [zh] `min(t1, t2)/2`：真正剩下可采样的时间。
    float
        needed_s; ///< [en] `t_settle + t_sample`, the number the window is compared with. [zh] `t_settle + t_sample`，窗口与之比较的那个数。
};

/**
 * @brief [en] What was reconstructed, and whether it may be used.
 * @brief [zh] 重构出来的东西，以及它能不能用。
 * @note [en] A refusal leaves `i_uvw` at zero **and** `valid` false, and no caller can tell those two apart by looking
 *            at the numbers -- which is exactly why the flag exists. A reconstructed zero that is really a refusal is
 *            the defect this type is shaped to prevent: it looks like a motor drawing no current, and a loop will act
 *            on it.
 * @note [zh] 拒绝时 `i_uvw` 留零**并且** `valid` 为假，而调用方单看数字是分不出这两者的——这正是那个标志存在的原因。「其
 *            实是拒绝」的重构零值，正是这个类型要防的缺陷：它看起来像一台不取电流的电机，而环路会照它动作。
 * @since 0.1.9
 * @exporter
 */
struct ShuntCurrents final {
    ErrorCode status; ///< [en] Ok, or Param for a duty that is not a duty. [zh] Ok；占空比不像占空比时为 Param。
    bool
        valid; ///< [en] false = do not use `i_uvw`, it is not a measurement. [zh] 假 = 别用 `i_uvw`，它不是一个测量值。
    Uvw<float>
        i_uvw; ///< [en] the three phase currents, A, summing to zero by construction. [zh] 三相电流，A，按构造和为零。
};

/**
 * @brief [en] Single-shunt current reconstruction: the window rule, and the three lines of algebra that use it.
 * @brief [zh] 单电阻电流重构：窗口判据，以及用到它的那三行代数。
 *
 * @details [en] One shunt sits in the DC link return, so it measures whatever current the *active vector* happens to
 *               route through it: during `V_k` that is the largest-duty phase alone, during `V_{k+1}` it is the two
 *               largest (equivalently the negative of the smallest). Two readings are therefore enough for three
 *               currents, because the machine is a floating star and they sum to zero:
 *
 *     `i_max = i_dc(V_k)`, `i_min = -i_dc(V_{k+1})`, `i_mid = -(i_max + i_min)`.
 *
 *     That is the whole reconstruction, and the sector table that usually accompanies it is deliberately **not** here:
 *     the answer depends only on which phase has which duty, so a table indexed by sector number would add a second
 *     place for the sector convention to be wrong without adding a single measurement.
 *
 *     What decides whether it can be done at all is the window. A 7-segment pattern applies each active vector twice
 *     per period, so the time available after the ringing settles and before the next edge is `min(t_active1,
 *     t_active2)/2`, and it must be at least `t_settle + t_sample`. Near a sector boundary one of those intervals goes
 *     to zero, and **that** is the operating region where a single shunt has no answer -- not an approximation, an
 *     absence. The policy's answer is to say so: `classify()` returns `WindowTooNarrow`, and `reconstruct()` returns
 *     `valid == false` rather than a plausible-looking number built from a reading taken during a transition.
 *
 *     The error structure is worth stating because it is not uniform: the largest and smallest phases carry the ADC
 *     error of one reading each, while the middle phase is a *difference*, so its noise is `sqrt(2)` times larger. A
 *     bound written for the first two and used for the third is 41 % too optimistic, which is why the test asserts two
 *     different bounds rather than one.
 *
 * @details [zh] 一个采样电阻放在直流回路里，于是它量到的是**有效矢量**恰好让它经过的那个电流：`V_k` 期间是占空比最大的
 *               那一相单独，`V_{k+1}` 期间是最大的两相（等价地，最小的那一相的相反数）。所以三次电流只需要两个读数，因
 *               为电机是浮地星形、三者和为零：
 *
 *     `i_max = i_dc(V_k)`、`i_min = −i_dc(V_{k+1})`、`i_mid = −(i_max + i_min)`。
 *
 *     这就是全部重构；而通常随它一起出现的那张扇区表被**刻意**省掉了：答案只取决于哪一相的占空比排在哪个位置，所以一张
 *     按扇区号索引的表只会给「扇区约定写错」多一个地方，却不多一条测量。
 *
 *     决定能不能做的是窗口。七段式图样每个周期把每个有效矢量施加两次，所以振铃建立之后、下一个边沿之前可用的时间是
 *     `min(t_active1, t_active2)/2`，它必须不小于 `t_settle + t_sample`。靠近扇区边界时其中一个区间趋于零，**那里**才是
 *     单电阻没有答案的工作区——不是近似不好，是没有。策略的回答就是把它说出来：`classify()` 返回 `WindowTooNarrow`，
 *     `reconstruct()` 返回 `valid == false`，而不是拿一个在换向过程中读到的数拼出一个看起来合理的电流。
 *
 *     误差结构值得写下来，因为它并不均匀：最大和最小两相各承担一个读数的 ADC 误差，而中间那一相是一个**差**，噪声大
 *     `sqrt(2)` 倍。按前两相写的界用在第三相上会乐观 41 %，这也是测试断言**两条**不同的界而不是一条的原因。
 *
 * @context [en] Current-loop ISR: noexcept, no allocation, no state.
 * @context [zh] 电流环 ISR：noexcept、不分配、无状态。
 * @since 0.1.9
 * @exporter
 */
class SingleShunt final {
  public:
    /**
     * @brief [en] The shunt's own timing, and the period the duty is relative to.
     * @brief [zh] 采样电阻自己的时间，以及占空比所相对的周期。
     * @since 0.1.9
     * @exporter
     */
    struct Config final {
        float t_settle_s; ///< [en] ringing + propagation after each edge, s. [zh] 每个边沿之后的振铃加传输延迟，s。
        float t_sample_s; ///< [en] the ADC's own aperture/hold, s. [zh] ADC 自己的采样保持时间，s。
        float switching_period_s; ///< [en] period the duty is relative to, s. [zh] 占空比所相对的周期，s。
    };

    SingleShunt() noexcept = default;
    explicit SingleShunt(const Config& config) noexcept : config_(config) {}

    [[nodiscard]] auto config() const noexcept -> const Config& { return config_; }

    void set_config(const Config& config) noexcept { config_ = config; }

    /**
     * @brief [en] Ok when the configuration describes a shunt that could measure something.
     * @brief [zh] 配置描述的采样电阻至少有可能量到东西时为 Ok。
     * @note [en] A zero period is refused, and so is a `needed_s` that leaves no window anywhere: if `2*(t_settle +
     *            t_sample) >= T` then even a perfectly symmetric pattern cannot be read, and saying Ok would turn a
     *            hardware that cannot work into a policy that occasionally reports a current.
     * @note [zh] 零周期被拒绝；把窗口彻底挤没的 `needed_s` 也被拒绝：若 `2·(t_settle + t_sample) ≥ T`，那么连完全对称的
     *            图样都读不出来，说它 Ok 等于把一块干不了活的硬件变成一个偶尔报出电流的策略。
     */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode {
        if (!std::isfinite(config_.t_settle_s) || !std::isfinite(config_.t_sample_s) ||
            !std::isfinite(config_.switching_period_s)) {
            return ErrorCode::Param;
        }
        if (!(config_.switching_period_s > 0.0F) || (config_.t_settle_s < 0.0F) || (config_.t_sample_s < 0.0F)) {
            return ErrorCode::Param;
        }
        if (2.0F * needed_s() >= config_.switching_period_s) {
            return ErrorCode::Param;
        }
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] `t_settle + t_sample`, s: the shortest window that can be read.
     * @brief [zh] `t_settle + t_sample`，s：能读到东西的最短窗口。
     */
    [[nodiscard]] auto needed_s() const noexcept -> float { return config_.t_settle_s + config_.t_sample_s; }

    /**
     * @brief [en] The pattern's three intervals and whether the two active ones can be sampled.
     * @brief [zh] 图样的三个区间，以及其中两个有效的能不能被采到。
     * @param duty [en] per-phase duty in [0, 1]; outside is clamped and reported as Param.
     * @param duty [zh] 每相占空比，[0, 1]；越界会被钳住并报 Param。
     * @since 0.1.9
     * @exporter
     */
    [[nodiscard]] auto classify(Uvw<float> duty) const noexcept -> ShuntWindow;

    /**
     * @brief [en] Three currents from two shunt readings, or a refusal.
     * @brief [zh] 两个采样读数换三相电流，或者一个拒绝。
     * @param duty [en] the same duties the readings were taken under; they decide which phase is which.
     * @param duty [zh] 读数取自的那组占空比；它们决定哪一相是哪一相。
     * @param i_dc_active1_a [en] the shunt current during `t_active1` (largest phase alone), A.
     * @param i_dc_active1_a [zh] `t_active1` 期间（最大相单独）采样电阻上的电流，A。
     * @param i_dc_active2_a [en] the shunt current during `t_active2` (two largest phases), A.
     * @param i_dc_active2_a [zh] `t_active2` 期间（最大的两相）采样电阻上的电流，A。
     * @since 0.1.9
     * @exporter
     */
    [[nodiscard]] auto reconstruct(Uvw<float> duty, float i_dc_active1_a, float i_dc_active2_a) const noexcept
        -> ShuntCurrents;

  private:
    Config config_{};
};

} // namespace core

} // namespace foc
