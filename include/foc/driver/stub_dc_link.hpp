// Conan::ImportStart
#pragma once
#include <foc/hal/dc_link.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_driver
 * @file stub_dc_link.hpp
 *
 * @brief [en] A DC link whose bus voltage is settable from the outside.
 * @brief [zh] 母线电压可由外部设定的直流母线。
 * @since 0.1.0
 */

namespace foc {

namespace driver {


/**
 * @brief [en] Reference `foc::hal::DcLink` implementation with a settable bus voltage.
 * @brief [zh] `foc::hal::DcLink` 的参考实现，母线电压可设。
 * @since 0.1.0
 * @exporter
 */
class StubDcLink final : public hal::DcLink {
  public:
    StubDcLink() noexcept = default;

    /**
     * @brief [en] Set the reported bus voltage.
     * @brief [zh] 设置上报的母线电压。
     * @since 0.1.0
     */
    void set_voltage_v(float voltage_v) noexcept;

    /**
     * @brief [en] Reported bus voltage in V.
     * @brief [zh] 上报的母线电压，V。
     * @since 0.1.0
     */
    auto voltage_v() noexcept -> float override;

    /**
     * @brief [en] Report this bus current from now on.
     * @brief [zh] 从现在起上报这个母线电流。
     * @param bus_current_a [en] current into the inverter, A.
     * @param bus_current_a [zh] 流入逆变器的电流，A。
     * @since 0.1.0
     */
    void set_bus_current_a(float bus_current_a) noexcept;

    /**
     * @brief [en] Report "not measured" from now on, as an unpopulated shunt would.
     * @brief [zh] 从现在起上报"没有测到"，如同没贴的分流电阻。
     * @since 0.1.0
     */
    void set_bus_current_unavailable() noexcept;

    /**
     * @brief [en] The bus current, if this link measures one.
     * @brief [zh] 母线电流，如果这个母线测了的话。
     * @param out_a [en] receiver, written only when the return value is true.
     * @param out_a [zh] 接收者，只在返回 true 时写入。
     * @return [en] false until `set_bus_current_a` has been called.
     * @return [zh] 在 `set_bus_current_a` 被调用之前一律 false。
     * @note [en] **Not** an override: the `hal::DcLink` contract does not require a bus current, because a bus fed from
     *            a supply may not measure one and making it mandatory would refuse those drives. This member is what
     *            `core::concepts::has_bus_current_v` looks for, which is how a consumer asks instead of assuming.
     * @note [zh] **不是** override： `hal::DcLink` 契约不要求母线电流，因为由电源供电的母线可能根本不测它，而把它变成必
     *            需会拒绝掉那些驱动。这个成员正是 `core::concepts::has_bus_current_v` 寻找的东西，消费者靠它去问而不是
     *            去假定。
     * @since 0.1.0
     */
    auto read_bus_current_a(float& out_a) noexcept -> bool;

  private:
    float voltage_v_{0.0F};
    float bus_current_a_{0.0F};
    bool bus_current_valid_{false};
};

} // namespace driver

} // namespace foc
