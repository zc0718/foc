// Conan::ImportStart
#pragma once
#include <cmath>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file thermistor.hpp
 *
 * @brief [en] An NTC in a divider, turned into a temperature -- and a refusal when the code says the sensor is gone.
 * @brief [zh] 分压里的 NTC 换算成温度——以及当码值说明传感器不在时的拒绝。
 * @warning [en] Both ways this sensor fails look like a number. An open thermistor reads as the rail the pull-up
 *              pushes it to and a short reads as the other rail, so a channel that only converts codes reports a
 *              *plausible* temperature next to a broken sensor -- and a derating schedule built on it derates a
 *              machine that is not there. That is why the conversion refuses a rail instead of returning it, and why
 *              the refusal has to reach whoever makes the decision rather than being flattened into a default.
 * @warning [zh] 这种传感器失效的两种样子都像一个数字。热敏电阻开路读到上拉把它顶到的那侧轨，短路读到另一侧轨，于是
 *              只做换算的通道会在一个坏传感器旁边报出一个 **貌似合理的温度** ，而建在它上的降额计划会给一台不在的电机
 *              降额。这就是换算要拒绝两端轨、而不是把它们返回的理由；也是那个拒绝必须传到做决定的那一方、不能被压成
 *              一个默认值的理由。
 * @since 0.1.0
 */

namespace foc {

namespace core {


/**
 * @brief [en] The three numbers an NTC datasheet gives, plus which leg of the divider it is.
 * @brief [zh] NTC 数据手册给出的三个数，外加它在分压里是哪条腿。
 * @note [en] `beta_k` is the two-point model's exponent rather than a full Steinhart-Hart polynomial. That is a choice
 *            with a stated cost: over the 20-100 degree C band this library cares about, the beta model is within a
 *            fraction of a degree of the datasheet, and it needs no coefficients that a caller would have to fit. A
 *            wider band is where the polynomial earns its extra constants, and that broadening belongs to whoever needs
 *            it.
 * @note [zh] `beta_k` 是两点模型的指数，而不是完整的 Steinhart-Hart 多项式。这是一个 **带代价的选择** ：在本库关心的
 *            20–100 摄氏度带内，beta 模型与数据手册相差不到零点几度，而且不需要调用方去拟合的系数。更宽的带才是多项式用
 *            额外常数换来的地方，那次扩展属于真正需要它的人。
 * @since 0.1.0
 * @exporter
 */
struct ThermistorConfig final {
    /**
     * @brief [en] resistance at 25 degrees C, ohm.
     * @brief [zh] 25 摄氏度时的阻值，欧姆。
     */
    float r25_ohm{10000.0F};
    /**
     * @brief [en] the beta exponent, kelvin.
     * @brief [zh] beta 指数，开尔文。
     */
    float beta_k{3950.0F};
    /**
     * @brief [en] the fixed resistor in the other leg, ohm.
     * @brief [zh] 另一条腿上的固定电阻，欧姆。
     */
    float series_ohm{10000.0F};
    /**
     * @brief [en] true when the thermistor is the leg to ground; false when it is the leg to the supply.
     * @brief [zh] true 表示热敏电阻是接地那条腿；false 表示它是接电源那条腿。
     */
    bool thermistor_to_ground{true};
};


/**
 * @brief [en] Convert a divider's ADC code into a temperature, or say that it is not one.
 * @brief [zh] 把分压的 ADC 码值换算成温度，或者说它不是温度。
 * @details [en] Two steps, both invertible and both tested as such: the divider gives `R_th` from the code ratio, and
 *               the beta model gives the temperature from `R_th`. Writing the forward model out (`resistance_ohm`) is
 *               what makes the pair checkable -- a converter tested only against itself would pass for any monotone
 *               curve.
 * @details [zh] 两步，都可逆、都按可逆来测：分压由码值比给出 `R_th`，beta 模型由 `R_th` 给出温度。把正向模型写出来（
 *               `resistance_ohm`）正是让这一对可核对的东西——一个只跟自己对账的换算器，对任何单调曲线都会通过。
 * @note [en] Both rails are refused rather than reported. A code of 0 or full scale is what an open or a shorted
 *            thermistor produces, and both are *plausible temperatures* on the curve; answering `false` is the only way
 *            the difference between "cold" and "not there" survives. A configured divider whose numbers cannot be right
 *            is refused the same way, so a typo in `series_ohm` cannot silently produce a temperature.
 * @note [zh] 两端轨都拒绝而不是上报。码值 0 或满量程正是热敏电阻开路或短路所产生的样子，而它们在曲线上都是
 *            **貌似合理的温度** ；回答 `false` 是"很冷"与"不在"之间的区别得以留存的唯一办法。配置本身就不可能成立的换算
 *            器也照此拒绝，于是一个写错的 `series_ohm` 不会静默产出一个温度。
 * @see Ledger `E-NTC-1` (the two-point model and its inverse, with a worked evaluation) and `Temperature.TheInverseUndoesTheForwardModelAcrossTheWorkingBand`. ／ 台账 `E-NTC-1`（两点模型及其逆，附一次实算）与 `Temperature.TheInverseUndoesTheForwardModelAcrossTheWorkingBand`。
 * @since 0.1.0
 * @exporter
 */
class NtcDivider final {
  public:
    /**
     * @brief [en] Build a converter from the datasheet numbers.
     * @brief [zh] 由数据手册的数字构造一个换算器。
     * @param config [en] the three numbers and the topology.
     * @param config [zh] 三个数与拓扑。
     * @since 0.1.0
     */
    explicit NtcDivider(const ThermistorConfig& config = ThermistorConfig{}) noexcept;

    /**
     * @brief [en] Whether the configuration can describe a divider at all.
     * @brief [zh] 这份配置到底能不能描述一个分压。
     * @return [en] true for three positive, finite numbers.
     * @return [zh] 三个数都为正且有限时为 true。
     * @note [en] Asked at construction time by whoever builds the converter, because the alternative is discovering it
     *            at the first reading -- in a fault path, where the answer matters least and costs most.
     * @note [zh] 由构造换算器的一方在构造时问，因为另一种可能是到第一次读数才发现——那是在故障路径上，那时答案最不重要、
     *            代价却最大。
     * @since 0.1.0
     */
    [[nodiscard]] auto valid_config() const noexcept -> bool;

    /**
     * @brief [en] The resistance the thermistor would have at a temperature: the forward model.
     * @brief [zh] 热敏电阻在某个温度下的阻值：正向模型。
     * @param celsius [en] temperature, degrees C.
     * @param celsius [zh] 温度，摄氏度。
     * @return [en] resistance in ohm; NaN when the configuration is unusable.
     * @return [zh] 阻值，欧姆；配置不可用时为 NaN。
     * @note [en] Exists so the inverse can be tested against a model instead of against itself, and so a host can
     *            sanity-check a table it was given.
     * @note [zh] 它存在，是为了让逆变换能对着一个模型被检验，而不是对着自己；也让宿主能对拿到的那张表做一次合理性检查。
     * @since 0.1.0
     */
    [[nodiscard]] auto resistance_ohm(float celsius) const noexcept -> float;

    /**
     * @brief [en] Convert one ADC code into a temperature.
     * @brief [zh] 把一个 ADC 码值换算成温度。
     * @param code [en] the raw conversion, 0..`full_scale`.
     * @param code [zh] 原始转换值，0..`full_scale`。
     * @param full_scale [en] the code a rail would produce (`2^n - 1` for an n-bit converter, or the reference count).
     * @param full_scale [zh] 满轨会产生的码值（n 位转换器的 `2^n - 1`，或参考计数）。
     * @param out_c [en] receiver, written only when the return value is true.
     * @param out_c [zh] 接收者，只在返回 true 时写入。
     * @return [en] true when the code is a temperature this converter can defend.
     * @return [zh] 当这个码值是一个本换算器承担得起的温度时为 true。
     * @since 0.1.0
     */
    [[nodiscard]] auto temperature_c(float code, float full_scale, float& out_c) const noexcept -> bool;

  private:
    ThermistorConfig config_;
};

} // namespace core

} // namespace foc
