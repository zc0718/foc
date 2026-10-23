// Conan::ImportStart
#pragma once
#include <foc/hal/dc_link.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {


/**
 * @brief [en] Reference `foc::hal::DcLink` implementation with a settable bus voltage.
 *        [zh] `foc::hal::DcLink` 的参考实现，母线电压可设。
 *
 * @since 0.1.0
 * @exporter
 */
class StubDcLink final : public hal::DcLink {
  public:
    StubDcLink() noexcept = default;

    /**
     * @brief [en] Set the reported bus voltage.
     *        [zh] 设置上报的母线电压。
     * @since 0.1.0
     */
    void set_voltage_v(float voltage_v) noexcept;

    /**
     * @brief [en] Reported bus voltage in V.
     *        [zh] 上报的母线电压，V。
     * @since 0.1.0
     */
    auto voltage_v() noexcept -> float override;

  private:
    float voltage_v_{0.0F};
};

} // namespace driver

} // namespace foc
