// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <foc/core/cogging_comp.hpp>
#include <foc/core/machine_params.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] What the learner is asked for: which orders, over how many revolutions, and at what speed.
 *
 *   **The speed band is the whole point, and it is measured rather than assumed.** The learner reads the q-axis current
 *   the speed loop supplies, so what it learns is the loop's *answer* to the cogging, and that answer has the loop's
 *   own frequency response in it. Measured on the SIL bench, sixth order, injected phase 0.3 rad (negative lag = the
 *   loop answers late): 2 rad/s -> 1.9 Hz, gain 1.02, lag -4.8 deg; 5 -> 4.8 Hz, 0.99, -7.8 deg; 10 -> 9.6 Hz, 0.99,
 *   -14.6 deg; 20 -> 19.1 Hz, 0.92, -26.3 deg; 40 -> 38.2 Hz, 0.76, -46.1 deg.
 *
 *   A table learned where the lag is `d` cancels only to `2*sin(d/2)` of the original ripple: 13.6 % at 5 rad/s, 25 %
 *   at 10, 46 % at 20, 78 % at 40. So learning above `max_speed_rad_s` is not "a bit worse" -- it is baking a phase
 *   error into the table that no amount of averaging removes. The residual is not zero even inside the band, and that
 *   is a property of the method rather than a defect: the table carries the lag of where it was learned.
 *
 * @brief [zh] 向学习器要什么：学哪几次、跨多少转、在什么转速下。
 *
 *   **转速带是全部要点，而且是量出来的、不是假设的。** 学习器读的是速度环供给的 q 轴电流，所以它学到的是环路对齿槽的
 *   **回答** ，而那个回答里带着环路自己的频率响应。SIL 台架实测（6 次，注入相位 0.3 rad；负号表示环路答得晚）：2 rad/s
 *   → 1.9 Hz、增益 1.02、滞后 −4.8°；5 → 4.8 Hz、0.99、−7.8°；10 → 9.6 Hz、0.99、−14.6°；20 → 19.1 Hz、0.92、 −26.3°；
 *   40 → 38.2 Hz、0.76、−46.1°。
 *
 *   在滞后为 `d` 处学到的表只能抵消到原纹波的 `2·sin(d/2)`：5 rad/s 处 13.6 %、10 rad/s 25 %、20 rad/s 46 %、40 rad/s
 *   78 %。所以高于 `max_speed_rad_s` 去学不是"稍差一点"，而是把一个相位误差烘进表里，再怎么平均也去不掉。带内残差也不是
 *   零，那是方法本身的性质而不是缺陷：表带着学习处的滞后。
 * @since 0.1.0
 * @exporter
 */
struct CoggingLearnConfig final {
    MachineParams
        params; ///< [en] `psi_f`/`ld`/`lq` used to turn the learned current into a torque. [zh] 用 `psi_f`/`ld`/`lq` 把学到的电流换成转矩。
    std::uint8_t pole_pairs;  ///< [en] ditto. [zh] 同上。
    std::uint8_t orders[6]{}; ///< [en] which mechanical orders to learn. [zh] 要学哪几个机械次数。
    int count{0};             ///< [en] how many entries are valid. [zh] 有效条数。
    int revolutions{2}; ///< [en] how many mechanical revolutions the window must hold. [zh] 窗口至少要跨多少机械转。
    float min_speed_rad_s{
        0.5F}; ///< [en] below this the loop's share is not measurable; refuse. [zh] 低于此值环路那份不可测；拒绝。
    float max_speed_rad_s{10.0F}; ///< [en] above this the lag is baked in; refuse. [zh] 高于此值会把滞后烘进去；拒绝。
    float speed_tolerance{
        0.10F}; ///< [en] allowed spread of `|omega|` over the window, relative. [zh] 窗口内 `|ω|` 允许的相对散布。
};

/**
 * @brief [en] What a finished learning pass says about itself, whether or not it produced a table.
 * @brief [zh] 一次学习结束时对自己的交代，无论有没有产出表。
 * @note [en] `revolutions` and the speed band are reported on every exit, including the refused ones: a refusal that
 *            says nothing leaves the caller guessing whether it needs to spin longer, slower, or steadier, and those
 *            are three different actions.
 * @note [zh] 每一次退出（包括被拒绝的）都报出 `revolutions` 与转速带：一句什么都不说的拒绝，会让调用方去猜它该转更久、
 *            转更慢、还是转得更稳——而那是三种不同的动作。
 * @since 0.1.0
 * @exporter
 */
struct CoggingLearnReport final {
    ErrorCode status;  ///< [en] `Ok`, `Param`, or `State` (not ready). [zh] `Ok`、`Param`，或 `State`（没准备好）。
    int count;         ///< [en] how many harmonics were written. [zh] 写出了几条谐波。
    float revolutions; ///< [en] how many mechanical revolutions the window covered. [zh] 窗口跨了多少机械转。
    float speed_min;   ///< [en] smallest `|omega|` seen, rad/s. [zh] 见到的最小 `|ω|`，rad/s。
    float speed_max;   ///< [en] largest `|omega|` seen, rad/s. [zh] 见到的最大 `|ω|`，rad/s。
};

/**
 * @brief [en] Learn a cogging table from the current a speed loop already supplies.
 *
 *   So a compensator can be built for a machine nobody has data for.
 *
 *   **The law, and why the sign is self-consistent.** With the compensator off and at steady speed, the loop's q-axis
 *   current *is* the anti-cogging current. Projecting it on the mechanical angle at order `k` gives `re = A*sin(phi)`,
 *   `im = A*cos(phi)` for `y = A*sin(k*theta + phi)`; converting that current back through `dT/diq` gives the torque
 *   with the **opposite** sign, which is exactly what `CoggingCompensation` wants to cancel. So a learned table can be
 *   handed straight to the compensator without a sign convention for the caller to remember.
 *
 *   **What it refuses, and why each refusal exists.** Fewer revolutions than asked (`State`): one revolution cannot
 *   separate a harmonic from the DC it rides on. Speed outside `[min, max]` (`State`): below it the loop's share is not
 *   measurable, above it the table carries the loop's lag. Speed that moved more than `speed_tolerance` over the window
 *   (`State`): a projection assumes the angle advances uniformly, and a run that accelerated is answering a different
 *   question. A saturated sample anywhere in the window (`State`): the loop was not answering the cogging, it was
 *   railing.
 *
 *   **What it cannot do.** It learns a *sum of orders*, never more: an order that was not asked for is absent from the
 *   table rather than extrapolated, because a compensator acting on an invented harmonic would inject a ripple of its
 *   own. It learns the loop's response, so it is faithful only inside the band above. And it does not know whether the
 *   ripple it found is cogging at all -- a dead-time or sensor ripple at the same order would be learned just as
 *   happily.
 *
 * @brief [zh] 从速度环本来就在供给的电流里学出一张齿槽表，于是没有数据的机器也能造出补偿器。
 *
 *   **律，以及符号为何自洽。** 补偿器关闭、稳态转速下，环路的 q 轴电流 **就是** 反齿槽电流。把它按机械角投影到 `k` 次
 *   上，对 `y = A·sin(k·θ + φ_inj)` 得到 `re = A·sin(φ_inj)`、 `im = A·cos(φ_inj)`；把这个电流经 `dT/diq` 换回转矩会得
 *   到 **相反** 的符号，而那正是 `CoggingCompensation` 要抵消的东西。所以学到的表可以直接交给补偿器，不需要调用方记住任
 *   何符号约定。
 *
 *   **它拒绝什么，以及每条拒绝为什么存在。** 转数不够（ `State`）：一转分不开一个谐波与它骑着的直流。转速出带（
 *   `State`）：低于下界那份不可测，高于上界表里带着环路滞后。窗口内转速变化超过 `speed_tolerance`（ `State`）：投影假设
 *   角度均匀推进，一个加速中的运行回答的是另一个问题。窗口内任何一次饱和（ `State`）：环路不是在回答齿槽，它在撞限。
 *
 *   **它做不到什么。** 它学的是 **若干次的和** ，仅此而已：没被要求的次数在表里是 **缺席** 而不是外插，因为作用在一个编
 *   造出来的谐波上的补偿器会注入自己的纹波。它学的是环路的响应，所以只在上面的带内可信。而且它不判断找到的纹波是不是齿
 *   槽—— 同一次上的死区或传感器纹波也会被同样高兴地学进去。
 * @since 0.1.0
 * @exporter
 */
class CoggingLearner final {
  public:
    CoggingLearner() noexcept = default;

    explicit CoggingLearner(const CoggingLearnConfig& config) noexcept : config_(config) {}

    /** @brief [en] Throw away everything accumulated so far. [zh] 丢弃已累积的一切。 */
    void reset() noexcept;

    /**
     * @brief [en] Feed one control period.
     * @brief [zh] 喂一个控制周期。
     * @param theta_mech_rad [en] mechanical angle, unwrapped.
     * @param theta_mech_rad [zh] 机械角，不做归一化。
     * @param iq_a [en] the q-axis current the loop supplied, A.
     * @param iq_a [zh] 环路供给的 q 轴电流，A。
     * @param id_a [en] the d-axis current at the same instant, A (used for `dT/diq`).
     * @param id_a [zh] 同一时刻的 d 轴电流，A（用于 `dT/diq`）。
     * @param omega_mech_rad_s [en] mechanical speed, rad/s.
     * @param omega_mech_rad_s [zh] 机械转速，rad/s。
     * @param saturated [en] whether the current loop was railing this period.
     * @param saturated [zh] 本周期电流环是否撞限。
     * @since 0.1.0
     */
    void accumulate(float theta_mech_rad, float iq_a, float id_a, float omega_mech_rad_s, bool saturated) noexcept;

    /** @brief [en] Whether enough has been accumulated to try. [zh] 是否已攒够可以一试。 */
    [[nodiscard]] auto ready() const noexcept -> bool;

    /** @brief [en] Revolutions covered so far. [zh] 目前覆盖的转数。 */
    [[nodiscard]] auto revolutions() const noexcept -> float;

    /**
     * @brief [en] Turn what was accumulated into a table the compensator can cancel.
     * @brief [zh] 把累积的东西变成补偿器能抵消的表。
     * @param out [en] destination, `capacity` entries; may be null when `capacity` is 0 (a dry run that only reports).
     * @param out [zh] 目标， `capacity` 条； `capacity` 为 0 时可传空指针（只报数的试跑）。
     * @param capacity [en] how many entries `out` holds.
     * @param capacity [zh] `out` 能装几条。
     * @return [en] `Ok` and the table, or `State` with the reasons above, or `Param` for a configuration that cannot
     *              work.
     * @return [zh] `Ok` 与表；或上述理由下的 `State`；或配置本身不成立时的 `Param`。
     * @note [en] The result is a **torque** table in N*m, i.e. the same type and units the compensator and the plant
     *            speak.
     * @note [zh] 结果是 N·m 的 **转矩** 表，即补偿器与被控对象共用的类型与单位。
     * @since 0.1.0
     */
    [[nodiscard]] auto finish(CoggingCompHarmonic* out, int capacity) const noexcept -> CoggingLearnReport;

    /** @brief [en] Whether the configuration could ever work. [zh] 配置是否有可能成立。 */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode;

  private:
    CoggingLearnConfig config_{};
    float re_[6]{};
    float im_[6]{};
    float theta_first_{0.0F};
    float theta_last_{0.0F};
    float omega_min_{0.0F};
    float omega_max_{0.0F};
    float id_sum_{0.0F};
    int samples_{0};
    bool saturated_{false};
    bool started_{false};
};

} // namespace core

} // namespace foc
