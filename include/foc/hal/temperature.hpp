// Conan::ImportStart
#pragma once
// Conan::ImportEnd

/*!
 * @ingroup group_hal
 * @file temperature.hpp
 *
 * @brief [en] The winding-temperature source a derating limit is derived from.
 * @brief [zh] 降额限值所依据的绕组温度来源。
 * @since 0.1.0
 */

namespace foc {

namespace hal {


/**
 * @brief [en] One reading: a temperature and whether it is a measurement at all.
 * @brief [zh] 一次读取：一个温度，以及它到底是不是一次测量。
 * @details [en] The `valid` flag is not defensive decoration. An NTC channel fails in exactly two ways that look like
 *               numbers -- an open thermistor reads as the rail the pull-up pushes it to, and a short reads as the
 *               other rail -- and both are *plausible temperatures* to any code that only looks at the value. A source
 *               that reports a number without saying whether it measured anything has moved that question into every
 *               consumer, where it will be answered differently in each of them.
 * @details [zh] `valid` 不是防御性的装饰。NTC 通道失效的方式恰好是两种"看起来像数字"的样子——热敏电阻开路读到上拉把它顶
 *               到的那一侧轨，短路读到另一侧轨——而两者对任何只读数值的代码都是 **貌似合理的温度** 。一个只报数字、不说
 *               自己是否真的测到了什么的来源，等于把这个问题推给每一个消费者，并在每一个消费者那里得到不同的答案。
 * @since 0.1.0
 * @exporter
 */
struct TemperatureReading final {
    /**
     * @brief [en] winding temperature, degrees C; meaningless when `valid` is false.
     * @brief [zh] 绕组温度，摄氏度； `valid` 为 false 时无意义。
     */
    float celsius{0.0F};
    /**
     * @brief [en] whether this reading came from a working sensor.
     * @brief [zh] 这次读数是否来自一个正常工作的传感器。
     */
    bool valid{false};
};


/**
 * @brief [en] Winding temperature source.
 * @brief [zh] 绕组温度来源。
 * @note [en] Optional in a `HalPolicy`: a drive with no temperature channel is legal, and the absence has to be
 *            answerable rather than silent -- a derating schedule that quietly assumes 25 degrees C is claiming a
 *            measurement nobody made. `core::concepts::has_temperature_v` is how a consumer asks whether it is there.
 * @note [zh] 在 `HalPolicy` 里是 **可选** 的：没有温度通道的驱动是合法的，而缺席必须是可回答的而不是静默的——一个悄悄假
 *            定 25 摄氏度的降额计划，是在声称一次没人做过的测量。消费者用 `core::concepts::has_temperature_v` 问它在不
 *            在。
 * @since 0.1.0
 * @exporter
 */
class Temperature {
  public:
    virtual ~Temperature() = default;

    /**
     * @brief [en] Read the winding temperature.
     * @brief [zh] 读取绕组温度。
     * @return [en] the reading, and whether it is a measurement.
     * @return [zh] 读数，以及它是否是一次测量。
     * @context [en] Any context, must not block.
     * @context [zh] 任意上下文，不得阻塞。
     * @since 0.1.0
     */
    virtual auto read() noexcept -> TemperatureReading = 0;
};

} // namespace hal

} // namespace foc
