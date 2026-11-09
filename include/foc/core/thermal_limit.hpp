// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/core/machine_params.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] The thermal facts a current limit is derived from: the winding's resistance at 20 C, its copper
 *             coefficient, how well it is cooled, and the temperature the caller refuses to exceed.
 *
 *   `Protector` already derates against a temperature -- and it says in its own header that it does not know what that
 *   temperature is, because `t_derate_*` are numbers the caller supplies. This type answers the question that comes
 *   *before* the protector: with this cooling, what current can be held **forever** without the winding arriving at
 *   `t_allowed_c`? That is a property of the machine and its mounting, it does not change tick to tick, and getting it
 *   wrong is not a tuning error -- it is a promise the drive cannot keep.
 *
 * @brief [zh] 电流限幅所依据的热事实：绕组 20 °C 时的电阻、铜的温度系数、散热好坏，以及调用方拒绝超过的温度。
 *
 *   `Protector` 已经会按温度降额——而它自己的头文件里就说了它不知道那个温度是多少，因为 `t_derate_*` 是调用方给的数。本
 *   类型回答保护器 **之前** 的那个问题：以这样的散热， **一直** 跑下去而不让绕组到达 `t_allowed_c`，电流能有多大？那是
 *   机器与它的安装方式的属性，不随 tick 变化，而算错了不是整定误差——是驱动器兑现不了的承诺。
 * @since 0.1.0
 * @exporter
 */
struct ThermalLimitConfig final {
    MachineParams
        params; ///< [en] `rs` is the resistance at **20 C**; `ld`/`lq`/`psi_f` are unused. [zh] `rs` 是 **20 °C** 下的电阻；`ld`/`lq`/`psi_f` 不用。
    float alpha_per_k{3.93e-3F}; ///< [en] copper's temperature coefficient, 1/K. [zh] 铜的温度系数，1/K。
    float rt_k_per_w{
        0.0F}; ///< [en] thermal resistance to ambient, K/W; <=0 is a configuration error. [zh] 到环境的热阻，K/W；<=0 是配置错误。
    float ct_j_per_k{
        0.0F}; ///< [en] thermal capacitance, J/K; only the time budget needs it, and <=0 means "no budget". [zh] 热容，J/K；只有时间预算需要它，<=0 表示"不给预算"。
    float t_ambient_c{20.0F}; ///< [en] ambient the heat leaks to, C. [zh] 散热到的环境温度，°C。
    float t_allowed_c{
        105.0F}; ///< [en] the temperature the reserve is sized against, C. [zh] 用来定这份余量的温度，°C。
};

/**
 * @brief [en] How much current the winding can hold indefinitely, and what that does to the temperature model.
 * @brief [zh] 绕组能一直扛住多大电流，以及那对温度模型意味着什么。
 * @since 0.1.0
 * @exporter
 */
struct ThermalLimitReport final {
    ErrorCode status; ///< [en] `Ok` or `Param`. [zh] `Ok` 或 `Param`。
    float
        continuous_a; ///< [en] the current whose settle is exactly `t_allowed_c`, A. [zh] 稳态恰好是 `t_allowed_c` 的那个电流，A。
    float
        settle_at_cont_c; ///< [en] where that current settles, C -- equal to `t_allowed_c` by construction. [zh] 该电流的稳态温度，°C——按构造等于 `t_allowed_c`。
    float thermal_time_const_s; ///< [en] `Rt*Ct` when `ct > 0`, else 0. [zh] `ct > 0` 时为 `Rt·Ct`，否则 0。
};

/**
 * @brief [en] What a given current does to the winding.
 *
 *   Where it settles, and how long it may be run before the limit is reached. `crosses == false` means the answer is
 *   **not** "a long time" but "never": the settle is at or below `t_allowed_c`, so the current is sustainable and the
 *   seconds field is `infinity`. Collapsing those two into one large number is how a scheduler ends up throttling a
 *   machine that was never in trouble.
 *
 * @brief [zh] 某个电流对绕组做了什么。
 *
 *   它收敛到哪里，以及在触及限值之前还能跑多久。 `crosses == false` 的意思 **不是**"很久"，而是"永远不会"：稳态等于或低
 *   于 `t_allowed_c`，于是这个电流是可持续的、秒数字段是 `infinity`。把这两种情况折成一个大数字，正是调度去压一台本来毫
 *   无危险的机器的方式。
 * @since 0.1.0
 * @exporter
 */
struct ThermalBudget final {
    ErrorCode
        status;   ///< [en] `Ok`, or `Param` when the configuration cannot answer. [zh] `Ok`；配置无法回答时是 `Param`。
    bool crosses; ///< [en] whether this current ever reaches `t_allowed_c`. [zh] 这个电流是否会到达 `t_allowed_c`。
    float
        seconds; ///< [en] time to reach it, s; `infinity` when `crosses` is false. [zh] 到达它所需时间，s；`crosses` 为假时是 `infinity`。
    float settle_c; ///< [en] where this current settles if it is held, C. [zh] 一直保持该电流时的稳态温度，°C。
};

/**
 * @brief [en] Turn a thermal description into a current the drive may promise.
 *
 *   And into the time budget at any other current.
 *
 *   **The law.** A current-fed winding's loss grows with its own temperature, `P = 1.5*i^2*rs20*(1 + alpha*(T-20))`, so
 *   the settle is the root of `T = Ta + Rt*P(T)` rather than a product: `T(i) = (Ta + A*i^2) / (1 - B*i^2)`, with
 *   `A = Rt*1.5*rs20*(1-20*alpha)` and `B = Rt*1.5*rs20*alpha`, and inverting it at `T = t_allowed` gives the
 *   continuous current
 *
 *   \f[ i_{cont} = \sqrt{\frac{t_{allowed} - T_a}{R_t\,1.5\,r_{s20}\,(1 + \alpha\,(t_{allowed} - 20))}} \f]
 *
 *   The factor `(1 + alpha*(t_allowed-20))` is the whole content of the feedback: dropping it is the naive
 *   `sqrt((T-Ta)/(1.5*Rt*rs20))`, which at 105 C on copper is **15.5 % too high**, and a drive built on it settles
 *   *above* the limit it thinks it is respecting. The test asserts both numbers, not just the right one.
 *
 *   **The time budget.** From a winding already at `t_now`, the exact solution of the same equation gives
 *
 *   \f[ t = -\tau\,\ln\frac{T_{eq} - t_{allowed}}{T_{eq} - t_{now}} \f]
 *
 *   so no integration is needed and no step size can change the answer. `tau = Rt*Ct` is the caller's, which is why
 *   `Ct` enters through the config even though the continuous current does not need it.
 *
 *   **What this is not.** It is not a measurement of `Rt`/`Ct` -- those are the caller's numbers, and this repository
 *   has never measured a winding's cooling. It is also not a controller: it answers "may I hold this, and for how
 *   long", and the host decides what to do when the answer is "not for long enough".
 *
 * @brief [zh] 把一份热描述换成一个驱动器可以承诺的电流，以及任意其它电流下的时间预算。
 *
 *   **律。** 电流供电的绕组，其损耗随自身温度增长， `P = 1.5·i²·rs20·(1 + α·(T−20))`，所以稳态是 `T = Ta + Rt·P(T)` 的
 *   根而不是一个乘积： `T(i) = (Ta + A·i²)/(1 − B·i²)`，其中 `A = Rt·1.5·rs20·(1−20α)`、 `B = Rt·1.5·rs20·α`；在
 *   `T = t_allowed` 处反解即得持续电流 `i_cont = sqrt((t_allowed − Ta)/(Rt·1.5·rs20·(1 + α·(t_allowed − 20))))`。那个
 *   `(1 + α·(t_allowed−20))` 就是反馈的全部内容：丢掉它得到的是天真的 `sqrt((T−Ta)/(1.5·Rt·rs20))`，在铜的 105 °C 处
 *   **高 15.5 %** ，而建立在它之上的驱动器会收敛到它自以为在遵守的限值 **之上** 。用例把两个数都断言，不只断言对的那
 *   个。
 *
 *   **时间预算。** 从已经处于 `t_now` 的绕组出发，同一方程的精确解给出
 *   `t = −τ·ln((T_eq − t_allowed)/(T_eq − t_now)) `，于是不需要积分，也没有步长能改变答案。 `τ = Rt·Ct` 是调用方的数，
 *   所以 `Ct` 经由配置进来，尽管持续电流并不需要它。
 *
 *   **它不是什么。** 它不是对 `Rt`/`Ct` 的测量——那是调用方的数，而本仓库从未量过任何绕组的散热。它也不是控制器：它回答"
 *   这个电流能不能保持、能保持多久"，至于"不够久"时怎么办，是宿主决定的。
 * @since 0.1.0
 * @exporter
 */
class ThermalLimit final {
  public:
    ThermalLimit() noexcept = default;

    explicit ThermalLimit(const ThermalLimitConfig& config) noexcept : config_(config) {}

    /** @brief [en] Whether the configuration could ever answer. [zh] 该配置是否有可能回答。 */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode;

    /** @brief [en] The current that can be held indefinitely, with its settle. [zh] 可以一直保持的电流，以及它的稳态温度。 */
    [[nodiscard]] auto continuous() const noexcept -> ThermalLimitReport;

    /** @brief [en] Where a current settles if it is held, C; `infinity` when the model has no steady state. [zh] 一直保持某电流时的稳态温度，°C；模型无稳态时是 `infinity`。 */
    [[nodiscard]] auto settle_temperature_c(float current_a) const noexcept -> float;

    /**
     * @brief [en] How long `current_a` may be held from `t_now_c` before the winding reaches `t_allowed_c`.
     * @brief [zh] 从 `t_now_c` 出发、以 `current_a` 保持，绕组到达 `t_allowed_c` 之前还能坚持多久。
     * @param current_a [en] the current under consideration, A.
     * @param current_a [zh] 待考察的电流，A。
     * @param t_now_c [en] the winding temperature now, C.
     * @param t_now_c [zh] 当前绕组温度，°C。
     * @since 0.1.0
     */
    [[nodiscard]] auto budget(float current_a, float t_now_c) const noexcept -> ThermalBudget;

    /** @brief [en] `Rt`, ohm-shaped but K/W: what was configured. [zh] 配置进来的 `Rt`（K/W）。 */
    [[nodiscard]] auto thermal_resistance_k_per_w() const noexcept -> float;

  private:
    ThermalLimitConfig config_{};
};

} // namespace core

} // namespace foc
