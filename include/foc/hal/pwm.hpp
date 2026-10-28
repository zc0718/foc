// Conan::ImportStart
#pragma once
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {


/**
 * @brief [en] Hardware abstraction contracts. Every implementation obligation is documented on the interface; the
 *             contract itself carries no implementation.
 * @brief [zh] 硬件抽象契约层。实现义务写在接口注释上；契约本身不含任何实现。
 * @since 0.1.0
 */
namespace hal {

using core::Uvw;


/**
 * @brief [en] Three-phase inverter output stage (PWM).
 * @brief [zh] 三相逆变输出级（PWM）。
 * @context [en] write_duty runs inside the current-loop ISR: no allocation, no blocking.
 * @context [zh] write_duty 在电流环 ISR 内运行：不分配、不阻塞。
 * @since 0.1.0
 * @exporter
 */
class Pwm {
  public:
    virtual ~Pwm() = default;

    /**
     * @brief [en] Configure carrier frequency and dead time.
     * @brief [zh] 配置载波频率与死区时间。
     * @param freq_hz [en] carrier frequency in Hz.
     * @param freq_hz [zh] 载波频率，单位 Hz。
     * @param deadtime_ns [en] dead time in ns.
     * @param deadtime_ns [zh] 死区时间，单位 ns。
     * @return [en] Ok on success; the code is implementation defined otherwise.
     * @return [zh] 成功返回 Ok；其余错误码由实现定义。
     * @pre [en] Not yet enabled.
     * @pre [zh] 尚未 enable。
     * @post [en] Output still disabled.
     * @post [zh] 输出仍为关闭状态。
     * @context [en] Initialisation only.
     * @context [zh] 仅初始化阶段。
     * @since 0.1.0
     */
    virtual auto configure(float freq_hz, float deadtime_ns) noexcept -> ErrorCode = 0;

    /**
     * @brief [en] Write the three phase duties for the next PWM period.
     * @brief [zh] 写入下一 PWM 周期的三相占空比。
     * @param duty [en] duty ratios already clamped to [0, 1].
     * @param duty [zh] 已限幅到 [0, 1] 的占空比。
     * @pre [en] configure() and enable() succeeded.
     * @pre [zh] configure() 与 enable() 已成功。
     * @post [en] Takes effect from the next carrier period.
     * @post [zh] 从下一个载波周期生效。
     * @context [en] Current-loop ISR, hard real time.
     * @context [zh] 电流环 ISR，硬实时。
     * @since 0.1.0
     */
    virtual void write_duty(Uvw<float> duty) noexcept = 0;

    /**
     * @brief [en] Enable the output stage.
     * @brief [zh] 使能输出级。
     * @return [en] Ok on success.
     * @return [zh] 成功返回 Ok。
     * @post [en] Output enabled, duty is 50% or the zero vector.
     * @post [zh] 输出使能，占空比为 50% 或零矢量。
     * @context [en] Initialisation only.
     * @context [zh] 仅初始化阶段。
     * @since 0.1.0
     */
    virtual auto enable() noexcept -> ErrorCode = 0;

    /**
     * @brief [en] Disable the output stage into its documented safe state.
     * @brief [zh] 关闭输出级并进入实现文档化的安全状态。
     * @post [en] Outputs off; the electrical state is implementation documented.
     * @post [zh] 输出关闭；电气状态由实现说明。
     * @context [en] Init, fault handling, or shutdown.
     * @context [zh] 初始化、故障处理或停机。
     * @since 0.1.0
     */
    virtual void disable() noexcept = 0;

    /**
     * @brief [en] Hardware fault flag (over-current, ...); false when the implementation has none.
     * @brief [zh] 硬件故障标志（过流等）；实现无此能力时恒为 false。
     * @context [en] Any context, must not block.
     * @context [zh] 任意上下文，不得阻塞。
     * @since 0.1.0
     */
    virtual auto fault() const noexcept -> bool = 0;
};

} // namespace hal

} // namespace foc
