// Conan::ImportStart
#pragma once
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace hal {

using core::Uvw;


/**
 * @brief [en] Three-phase current sensing.
 * @brief [zh] 三相电流采样。
 * @since 0.1.0
 * @exporter
 */
class CurrentSense {
  public:
    virtual ~CurrentSense() = default;

    /**
     * @brief [en] Read the three phase currents of the current control period.
     * @brief [zh] 读取本控制周期的三相电流。
     * @return [en] Phase currents in A.
     * @return [zh] 三相电流，单位 A。
     * @pre [en] The platform guarantees the ADC conversion of this period is complete and aligned with the PWM carrier
     *           (see the platform implementation notes).
     * @pre [zh] 平台需保证本周期 ADC 转换完成且与 PWM 载波对齐（见平台实现说明）。
     * @context [en] Current-loop ISR, hard real time, no allocation.
     * @context [zh] 电流环 ISR，硬实时，不分配。
     * @since 0.1.0
     */
    virtual auto read_uvw() noexcept -> Uvw<float> = 0;

    /**
     * @brief [en] Sample the zero-current offsets (power-up calibration).
     * @brief [zh] 采集零电流偏置（上电校准）。
     * @return [en] Ok on success, a timeout/error code from the implementation otherwise.
     * @return [zh] 成功返回 Ok，否则由实现返回超时/错误码。
     * @pre [en] The inverter output is disabled.
     * @pre [zh] 逆变输出已关闭。
     * @context [en] Initialisation only, may block.
     * @context [zh] 仅初始化阶段，允许阻塞。
     * @since 0.1.0
     */
    virtual auto calibrate_offset() noexcept -> ErrorCode = 0;

    /**
     * @brief [en] Effective sampling rate of read_uvw().
     * @brief [zh] read_uvw() 的实际采样率。
     * @return [en] Sample rate in Hz.
     * @return [zh] 采样率，单位 Hz。
     * @context [en] Any context.
     * @context [zh] 任意上下文。
     * @since 0.1.0
     */
    virtual auto sample_rate_hz() const noexcept -> float = 0;
};

} // namespace hal

} // namespace foc
