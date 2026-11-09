// Conan::ImportStart
#pragma once
#include <cstdint>
// Conan::ImportEnd

namespace foc {

namespace hal {


/**
 * @brief [en] The three hall sensors, as one three-bit field.
 * @brief [zh] 三个霍尔传感器，作为一个三位字段。
 * @note [en] A port rather than a pin-reading helper so that a host with an inverter's own commutation input, a shift
 *            register, or a timer's Hall interface can all feed the same decoder. The decoder is where the sector
 *            mapping and the edge timing live; reading the pins is the host's business, and this is the only thing it
 *            has to implement.
 * @note [zh] 做成一个端口而不是读引脚的工具函数，是为了让带换相输入的逆变器、移位寄存器、或定时器的霍尔接口都能喂给同一
 *            个解码器。扇区映射与边沿计时在解码器里；读引脚是宿主的事，而它只需要实现这一件事。
 * @since 0.1.0
 * @exporter
 */
class Hall {
  public:
    virtual ~Hall() = default;

    /**
     * @brief [en] The sensor bits, bit `i` = sensor `i`, in `0..7`.
     * @brief [zh] 传感器位，第 `i` 位 = 第 `i` 个传感器，取值 `0..7`。
     * @return [en] the code; `0` and `7` are the two combinations a working set never produces.
     * @return [zh] 编码； `0` 与 `7` 是正常传感器组永远不会产生的两种组合。
     * @context [en] Control context.
     * @context [zh] 控制上下文。
     * @since 0.1.0
     */
    virtual auto states() noexcept -> std::uint8_t = 0;
};

} // namespace hal

} // namespace foc
