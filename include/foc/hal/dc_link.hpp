// Conan::ImportStart
#pragma once
// Conan::ImportEnd

namespace foc {

namespace hal {


/**
 * @brief [en] DC link (bus) voltage source.
 *        [zh] 直流母线电压来源。
 *
 * @since 0.1.0
 * @exporter
 */
class DcLink {
  public:
    virtual ~DcLink() = default;

    /**
     * @brief [en] Measured bus voltage.
     *        [zh] 实测母线电压。
     *
     * @return [en] Voltage in V; may be averaged over the last control period.
     *         [zh] 电压，单位 V；可在最近一个控制周期内平均。
     * @context [en] Current-loop ISR. [zh] 电流环 ISR。
     * @since 0.1.0
     */
    virtual auto voltage_v() noexcept -> float = 0;
};

} // namespace hal

} // namespace foc
