// Conan::ImportStart
#pragma once
// Conan::ImportEnd

/*!
 * @ingroup group_hal
 * @file dc_link.hpp
 *
 * @brief [en] The DC bus voltage source the modulator scales its duties by.
 * @brief [zh] 调制器据以缩放占空比的直流母线电压来源。
 * @since 0.1.0
 */

namespace foc {

namespace hal {


/**
 * @brief [en] DC link (bus) voltage source.
 * @brief [zh] 直流母线电压来源。
 * @since 0.1.0
 * @exporter
 */
class DcLink {
  public:
    virtual ~DcLink() = default;

    /**
     * @brief [en] Measured bus voltage.
     * @brief [zh] 实测母线电压。
     * @return [en] Voltage in V; may be averaged over the last control period.
     * @return [zh] 电压，单位 V；可在最近一个控制周期内平均。
     * @context [en] Current-loop ISR.
     * @context [zh] 电流环 ISR。
     * @since 0.1.0
     */
    virtual auto voltage_v() noexcept -> float = 0;
};

} // namespace hal

} // namespace foc
