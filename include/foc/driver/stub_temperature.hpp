// Conan::ImportStart
#pragma once
#include <foc/hal/temperature.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_driver
 * @file stub_temperature.hpp
 *
 * @brief [en] A temperature source a test can set, including the state where the sensor is gone.
 * @brief [zh] 用例可以设定的温度来源，包括传感器不在的状态。
 * @since 0.1.0
 */

namespace foc {

namespace driver {


/**
 * @brief [en] Reference `foc::hal::Temperature` implementation with a settable reading.
 * @brief [zh] `foc::hal::Temperature` 的参考实现，读数可设。
 * @note [en] The default is `valid == false`, not 25 degrees C. A stub that reports a plausible temperature before
 *            anybody set one is answering a question it was never asked, and code that reads it would pass a test that
 *            never supplied a sensor. Starting invalid means the first thing every consumer has to handle is the
 *            absence, which is the order that survives contact with hardware.
 * @note [zh] 默认是 `valid == false`，不是 25 摄氏度。一个在没人设定之前就报出一个貌似合理温度的桩件，是在回答一个从来
 *            没人问过它的问题，而读它的代码会通过一个从未提供传感器的用例。从"无效"开始，意味着每个消费者必须先处理缺席
 *            ——那是与硬件接触之后仍然活着的顺序。
 * @since 0.1.0
 * @exporter
 */
class StubTemperature final : public hal::Temperature {
  public:
    StubTemperature() noexcept = default;

    /**
     * @brief [en] Report this temperature from now on.
     * @brief [zh] 从现在起上报这个温度。
     * @param celsius [en] winding temperature, degrees C.
     * @param celsius [zh] 绕组温度，摄氏度。
     * @since 0.1.0
     */
    void set_celsius(float celsius) noexcept;

    /**
     * @brief [en] Report "no measurement" from now on, as a dead channel would.
     * @brief [zh] 从现在起上报"没有测量"，如同一个失效的通道。
     * @note [en] A separate call rather than `set_celsius(NaN)`: NaN is a *value* here, and the whole point of the flag
     *            is that the question "is this a measurement" is not answered by looking at the number.
     * @note [zh] 单独的调用而不是 `set_celsius(NaN)`：在这里 NaN 是一个 **值** ，而那个标志的全部意义就是"这是不是一次
     *            测量"不能靠看数字来回答。
     * @since 0.1.0
     */
    void set_invalid() noexcept;

    /**
     * @brief [en] The configured reading.
     * @brief [zh] 当前配置的读数。
     * @return [en] the reading, valid only after `set_celsius`.
     * @return [zh] 读数，只在 `set_celsius` 之后有效。
     * @since 0.1.0
     */
    auto read() noexcept -> hal::TemperatureReading override;

  private:
    hal::TemperatureReading reading_{0.0F, false};
};

} // namespace driver

} // namespace foc
