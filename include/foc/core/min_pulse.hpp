// Conan::ImportStart
#pragma once
#include <cmath>

#include <foc/core/limits.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] The gate driver's own limit: a pulse narrower than this is not transmitted at all.
 * @brief [zh] 栅极驱动器自己的极限：比这更窄的脉冲根本不会被送出。
 * @note [en] `min_pulse_s` is a property of the **driver**, not of the modulator. A bootstrap high-side driver needs a
 *            minimum on-time to recharge its capacitor, an isolated driver has a propagation delay that swallows short
 *            pulses, and the datasheet gives one number covering both edges. `switching_period_s` is here only because
 *            the modulator works in duty: `t_min / T` is the smallest duty that survives, and it is the whole of the
 *            configuration once that division is done.
 * @note [zh] `min_pulse_s` 是 **驱动器** 的属性，不是调制器的。自举高侧驱动器需要最小导通时间给电容充电，隔离驱动器的传
 *            输延迟会吞掉短脉冲，而数据手册只给一个覆盖两个边沿的数。`switching_period_s` 在这里只是因为调制器用占空比
 *            工作：`t_min / T` 是能活下来的最小占空比，做完这个除法之后整个配置就只剩这一个数。
 * @since 0.1.9
 * @exporter
 */
struct MinPulseConfig final {
    float
        min_pulse_s; ///< [en] shortest pulse the driver transmits, s; `<= 0` = off. [zh] 驱动器能送出的最短脉冲，s；`<= 0` 表示关闭。
    float switching_period_s; ///< [en] the period the duty is relative to, s. [zh] 占空比所相对的周期，s。
};

/**
 * @brief [en] One period of the driver's decision, and what it costs in volt-seconds.
 * @brief [zh] 驱动器一个周期的决定，以及它在伏秒上的代价。
 * @note [en] `lost_vs` is signed and **per phase**, positive meaning the bridge will deliver *less* than was asked. It
 *            is the quantity a caller needs in order to stop lying about its own output: a control loop that assumes
 *            its commands are realised will integrate a voltage error that nothing in its feedback loop can see,
 *            because the bridge is inside the loop and the voltage is not measured.
 * @note [zh] `lost_vs` 是 **每相** 带符号的，正数表示桥臂 **少** 交付。调用方需要它，才能停止对自己的输出撒谎：一个假定
 *            指令被如实执行的环路，会积分一项反馈里看不见的电压误差，因为桥臂在环内、电压却没被量。
 * @since 0.1.9
 * @exporter
 */
struct MinPulseOutput final {
    ErrorCode
        status; ///< [en] Ok, or Param for a configuration/voltage that cannot be shaped. [zh] Ok；配置或母线电压无法整形时为 Param。
    Uvw<float> duty; ///< [en] the duty the bridge will actually run. [zh] 桥臂实际会跑的占空比。
    Uvw<float>
        lost_vs; ///< [en] volt-seconds not delivered this period, V*s, positive = short. [zh] 本周期未交付的伏秒，V·s，正数表示少给。
    bool
        narrowed; ///< [en] at least one phase's pulse was not transmitted as asked. [zh] 至少有一相的脉冲没有按请求送出。
};

/**
 * @brief [en] What a real gate driver does to a narrow pulse, and how to keep the average exact anyway.
 * @brief [zh] 真实栅极驱动器对窄脉冲做了什么，以及怎么把平均值照样守住。
 *
 * @details [en] The bridge has a floor that no controller can argue with: below `t_min` the driver does not transmit
 *               the pulse. Two things follow, and they are usually conflated.
 *
 *   **The law.** For one phase, with `m = t_min/T` and the request `d` in [0, 1]: `applied = 0` when `d < m` (the
 *   on-pulse is too short to exist), `applied = 1` when `1 - d < m` (the *off*-pulse is too short, so the high side
 *   cannot turn off), and `applied = d` otherwise. The period-mean phase voltage is therefore short by `(d - applied) *
 *   v_bus`, which is a number the caller can write down *before* it happens. That is the whole point of `apply()`: not
 *   to change anything, but to say what the bridge will do.
 *
 *   **The residue.** `shape()` adds the lost duty to the next period's request and remembers what is still owed, so
 *   `sum(applied) = sum(requested) - carry` over any window: the *average* is exact and only the residue is left. The
 *   residue is bounded by one minimum pulse (`abs(carry) <= m` in duty, i.e. one `t_min` window), so over `N` periods
 *   the mean error is at most `m/N` -- 0.024 % of full duty at `m = 2 %` and `N = 100`, against the 2 % that dropping
 *   the pulse every period would cost. A delta-sigma modulator, played on a pulse width.
 *
 *   What this class **cannot** do, stated because the alternative is a caller that believes otherwise: it cannot make
 *   the driver transmit a pulse it refuses, it does not model the propagation-delay *jitter* around the two edges, and
 *   the residue assumes the same `t_min` next period -- a driver whose limit changes (temperature, bus voltage) breaks
 *   the exactness, not the bound.
 *
 * @details [zh] 桥臂有一个任何控制器都争不过的下限：窄于 `t_min` 的脉冲，驱动器不送。由此有两件事，而它们经常被混为一
 *               谈。
 *
 *   **律。** 单相，`m = t_min/T`，请求 `d` ∈ [0, 1]：`d < m` 时 `applied = 0`（导通脉冲短到不存在），`1 − d < m` 时
 *   `applied = 1`（ **关断** 脉冲太短，高侧关不掉），其余 `applied = d`。于是周期平均相电压少了 `(d − applied)·v_bus`——
 *   一个调用方 **在它发生之前** 就能写下来的数。`apply()` 的全部意义就在这里：不改变任何东西，只说桥臂会怎么做。
 *
 *   **余量。** `shape()` 把丢掉的那份占空比加到下一周期的请求上，并记住还欠多少，于是在任何窗口上 `Σapplied =
 *   Σrequested − carry`：**平均**是精确的，剩下的只是余量。余量被一个最小脉冲界住（占空比上 `abs(carry) ≤ m`，即一个
 *   `t_min` 窗口），所以 `N` 个周期的平均误差至多 `m/N`——`m = 2 %`、`N = 100` 时是全占空比的 0.024 %，而每个周期都把脉
 *   冲丢掉的做法要付 2 %。一个调在脉宽上的 delta-sigma 调制器。
 *
 *   这个类 **做不到** 什么，写下来是因为不写就会有调用方以为它做得到：它无法让驱动器送出一个它拒绝送的脉冲；它不建模两
 *   个边沿的传输延迟 **抖动** ；余量假定下一周期的 `t_min` 相同——一个极限会变的驱动器（温度、母线电压）破坏的是 **精确
 *   性**，不是那条界。
 *
 * @context [en] Current-loop ISR: noexcept, no allocation, one branch per phase when disabled.
 * @context [zh] 电流环 ISR：noexcept、不分配、未启用时每相一个分支。
 * @since 0.1.9
 * @exporter
 */
class MinPulse final {
  public:
    MinPulse() noexcept = default;
    explicit MinPulse(const MinPulseConfig& config) noexcept : config_(config) {}

    [[nodiscard]] auto config() const noexcept -> const MinPulseConfig& { return config_; }

    /**
     * @brief [en] Take a new configuration; clears the residue, because the old one is denominated in the old `t_min`.
     * @brief [zh] 换一份配置；清掉余量，因为旧余量是按旧的 `t_min` 计的。
     */
    void set_config(const MinPulseConfig& config) noexcept {
        config_ = config;
        reset();
    }

    /**
     * @brief [en] Ok when the configuration describes a driver limit that a duty can actually satisfy.
     * @brief [zh] 配置描述的驱动极限是一个占空比真的能满足的极限时为 Ok。
     * @note [en] `min_pulse_s <= 0` is **not** refused: that is a driver with no limit, which is the default and is how
     *            every measurement in this repository is taken with the feature off. What is refused is a limit that
     *            cannot be a limit: a negative one (a typo), or `2*t_min > T`, where a pulse and the gap after it are
     *            both too short and no duty whatsoever is transmissible.
     * @note [zh] **不** 拒绝 `min_pulse_s <= 0`：那是没有极限的驱动器，也就是默认值，本仓库里所有「关掉这个功能」的实测
     *            都是这么做的。被拒绝的是 **不能算作极限** 的极限：负值（打错了），或者 `2·t_min > T`——那时脉冲和它后面
     *            的间隔都太短，任何占空比都传不出去。
     */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode {
        if (!std::isfinite(config_.min_pulse_s) || !std::isfinite(config_.switching_period_s)) {
            return ErrorCode::Param;
        }
        if (!(config_.switching_period_s > 0.0F) || (config_.min_pulse_s < 0.0F)) {
            return ErrorCode::Param;
        }
        if (config_.min_pulse_s > 0.0F && (2.0F * config_.min_pulse_s > config_.switching_period_s)) {
            return ErrorCode::Param;
        }
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] The smallest duty that survives, `t_min/T`; 0 when the limit is off.
     * @brief [zh] 能存活的最小占空比 `t_min/T`；极限关闭时为 0。
     */
    [[nodiscard]] auto min_duty() const noexcept -> float {
        return (config_.min_pulse_s > 0.0F) ? (config_.min_pulse_s / config_.switching_period_s) : 0.0F;
    }

    /**
     * @brief [en] What the bridge will do with this request, with **no memory**: the pure driver rule.
     * @brief [zh] 桥臂会拿这个请求做什么， **不带记忆** ：纯粹的驱动规则。
     * @param duty [en] per-phase request in [0, 1]; outside is clamped and reported as Param.
     * @param duty [zh] 每相请求，[0, 1]；越界会被钳住并报 Param。
     * @param v_bus [en] DC link in V, used only to convert duty into volt-seconds.
     * @param v_bus [zh] 母线电压，V，只用来把占空比换成伏秒。
     * @return [en] the applied duty, the volt-seconds the bridge will not deliver, and whether anything was narrowed.
     * @return [zh] 实际施加的占空比、桥臂不会交付的伏秒，以及是否有任何一相被截窄。
     * @since 0.1.9
     * @exporter
     */
    [[nodiscard]] auto apply(Uvw<float> duty, float v_bus) const noexcept -> MinPulseOutput;

    /**
     * @brief [en] The same, plus the residue: what is lost this period is added to the next one's request.
     * @brief [zh] 同样的东西再加余量：本周期丢掉的，加到下一周期的请求上。
     * @param duty [en] per-phase request in [0, 1]; outside is clamped and reported as Param.
     * @param duty [zh] 每相请求，[0, 1]；越界会被钳住并报 Param。
     * @param v_bus [en] DC link in V.
     * @param v_bus [zh] 母线电压，V。
     * @note [en] `duty` in the result is **what to command**, which is the request plus the carried residue; `lost_vs`
     *            is what is *still* lost after the carry, and over a window it sums to `carry[window end]` -- that
     *            identity is the thing a test can hold the class to, and it is the reason the residue is exposed at
     *            all.
     * @note [zh] 结果里的 `duty` 是 **该命令什么** ，即请求加上带过来的余量；`lost_vs` 是带上余量之后 **仍然** 丢掉的部
     *            分，它在任意窗口上求和等于 `carry[窗口末]`——这条恒等式是测试能拿来卡住这个类的东西，也正是余量被暴露出
     *            来的原因。
     * @since 0.1.9
     * @exporter
     */
    [[nodiscard]] auto shape(Uvw<float> duty, float v_bus) noexcept -> MinPulseOutput;

    /**
     * @brief [en] The duty still owed to the average, per phase; bounded by `min_duty()`.
     * @brief [zh] 平均意义上仍欠着的占空比，每相；由 `min_duty()` 界住。
     */
    [[nodiscard]] auto carry() const noexcept -> Uvw<float> { return carry_; }

    /**
     * @brief [en] Forget the residue (a mode change, a re-arm, a different machine).
     * @brief [zh] 忘掉余量（换模式、重新使能、换一台电机）。
     */
    void reset() noexcept { carry_ = Uvw<float>{}; }

  private:
    MinPulseConfig config_{};
    Uvw<float> carry_{};
};

} // namespace core

} // namespace foc
