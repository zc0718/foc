// Conan::ImportStart
#pragma once
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace hal {


/**
 * @brief [en] The brake resistor's switch: one duty output, and the safe state is "open".
 * @brief [zh] 制动电阻的开关：一个占空比输出，而安全状态是"断开"。
 *
 * @details [en] This is the output stage of the brake chopper, not the chopper. `core::BrakeChopper` decides *whether*
 *               the resistor should be driven and at what duty; this contract is the one thing left that the board
 *               owns: turn a switch on for a fraction of a period.
 *
 *   Three deliberate differences from `hal::Pwm`, each of which a reader has to be able to check rather than assume:
 *
 *   1. **One duty, not three.** A chopper is a single switch across the bus. Handing it a `Uvw` would invite a caller
 *   to write a chopper waveform through the inverter's contract, where the common-mode convention that makes SVPWM work
 *   would silently rescale it.
 *
 *   2. **No `enable()`.** `Pwm` needs an explicit enable because three half-bridges can be enabled into a short; a
 *   brake switch's dangerous state is *conducting*, so requiring two calls to make heat is not a safety feature, it is
 *   a way to have the safe state depend on the order of two calls. `write_duty(0)` is the safe state and `disable()` is
 *   the same thing said out loud.
 *
 *   3. **No `fault()`.** A chopper's over-temperature is the resistor's business, and the library already models it as
 *   an I2t budget with cooling (`core::BrakeChopperConfig::i2t_limit_a2s`). A second source for one effect would make
 *   the numbers unreadable -- the same rule this repository applies to `PlantHal::set_inverter` and the dead-time
 *   knobs.
 *
 * @details [zh] 这是制动斩波器的输出级，不是斩波器本身。 `core::BrakeChopper` 决定电阻 **该不该** 被驱动、占空比多少；
 *               这个契约只剩板子拥有的一件事：让一个开关在一个周期里导通一段比例。
 *
 *   与 `hal::Pwm` 的三个差异都是有意的，而且每一个读者都应当能自己核对而不是接受：
 *
 *   1. **一个占空比，不是三个。** 斩波器是跨在母线上的一个开关。给它一个 `Uvw` 会诱使调用方把斩波波形从逆变器的契约里写
 *   出去，而那里让 SVPWM 成立的共模约定会悄悄把它重新缩放。
 *
 *   2. **没有 `enable()`。** `Pwm` 需要显式使能，因为三个半桥可能被使能到短路；而制动开关的危险状态是 **导通** ，所以要
 *   求两次调用才能发热不是安全特性，而是让安全状态取决于两次调用的顺序。 `write_duty(0)` 就是安全状态， `disable()` 是
 *   把同一句话说出来。
 *
 *   3. **没有 `fault()`。** 斩波器的过温是电阻的事，而库已经把它建模成带冷却的 I²t 预算（
 *   `core::BrakeChopperConfig::i2t_limit_a2s`）。一个效应两个来源会让数字无法解读——与 `PlantHal::set_inverter` 和死区旋
 *   钮遵循的是同一条规矩。
 *
 * @note [en] Not a member of `Motor`'s HalPolicy, on purpose: the chopper does not act on the machine, it acts on the
 *            bus, and it keeps running when the drive is disabled -- which is exactly when a regenerating machine needs
 *            it most. `driver::StubHal` therefore does not contain one; a host composes it beside the motor, from the
 *            same control tick.
 * @note [zh] 有意 **不** 放进 `Motor` 的 HalPolicy：斩波器不作用于电机，它作用于母线，而且在驱动被关闭之后还在运行——那
 *            恰好是一台回馈中的电机最需要它的时候。所以 `driver::StubHal` 里没有它；调用方在与电机同一个控制拍里把它组
 *            合在电机旁边。
 * @context [en] `write_duty` runs inside the control ISR: no allocation, no blocking.
 * @context [zh] `write_duty` 在控制 ISR 内运行：不分配、不阻塞。
 * @since 0.1.0
 * @exporter
 */
class BrakeSwitch {
  public:
    virtual ~BrakeSwitch() = default;

    /**
     * @brief [en] Configure the chopping frequency.
     * @brief [zh] 配置斩波频率。
     * @param freq_hz [en] switching frequency in Hz; `<= 0` must be refused.
     * @param freq_hz [zh] 开关频率，Hz； `<= 0` 必须被拒绝。
     * @return [en] Ok on success; implementations refuse what they cannot do rather than substituting a default.
     * @return [zh] 成功返回 Ok；实现应当拒绝自己做不到的，而不是替换成一个默认值。
     * @pre [en] Not yet writing a duty.
     * @pre [zh] 尚未写入占空比。
     * @context [en] Initialisation only.
     * @context [zh] 仅初始化阶段。
     * @since 0.1.0
     */
    virtual auto configure(float freq_hz) noexcept -> ErrorCode = 0;

    /**
     * @brief [en] Drive the switch for the next period: `duty` of the time closed, `1 - duty` open.
     * @brief [zh] 驱动下一个周期：闭合 `duty` 的时间，断开 `1 - duty`。
     * @param duty [en] duty ratio already clamped to [0, 1]; 0 is the safe state.
     * @param duty [zh] 已限幅到 [0, 1] 的占空比；0 即安全状态。
     * @post [en] Takes effect from the next switching period.
     * @post [zh] 从下一个开关周期生效。
     * @context [en] Control ISR, hard real time.
     * @context [zh] 控制 ISR，硬实时。
     * @since 0.1.0
     */
    virtual void write_duty(float duty) noexcept = 0;

    /**
     * @brief [en] Open the switch and keep it open.
     * @brief [zh] 断开开关并保持断开。
     * @post [en] The resistor is not driven from the next switching period until the next `write_duty`.
     * @post [zh] 从下一个开关周期起电阻不再被驱动，直到下一次 `write_duty`。
     * @since 0.1.0
     */
    virtual void disable() noexcept = 0;
};

} // namespace hal

} // namespace foc
