// Conan::ImportStart
#pragma once
#include <foc/core/types.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_hal
 * @file mechanical_brake.hpp
 *
 * @brief [en] The holding brake on the shaft, as opposed to the chopper on the bus.
 * @brief [zh] 轴上的保持抱闸，与母线上的斩波器相对。
 * @since 0.1.0
 */

namespace foc {

namespace hal {

/**
 * @brief [en] A spring-applied holding brake: it holds the shaft when it is not energised.
 *
 *   This is **not** `hal::BrakeSwitch`, and the two are one word apart in English and nothing alike in what they do.
 *   `BrakeSwitch` drives a resistor across the DC link, acts on the bus, and keeps working after the inverter is
 *   disabled. This one acts on the **shaft**: it is what stops a vertical load from falling while the drive is off.
 *
 *   The contract is written for the fail-safe arrangement (spring applies, coil releases) because that is the one where
 *   losing a wire is a stop rather than a fall. A drive with a non-fail-safe brake can still implement this: `engage()`
 *   is "do whatever makes the shaft hold", and which way the coil is wired is the driver's business.
 *
 *   What the contract does **not** carry is time. A real brake takes tens of milliseconds to grip or let go, and how
 *   long is a property of the brake and its load, not of the interface -- so the waiting belongs to whoever sequences
 *   the enable, and it is a number the integrator supplies.
 *
 * @brief [zh] 弹簧施力的保持抱闸：不通电时它抱住轴。
 *
 *   这 **不是** `hal::BrakeSwitch`：两者在英文里只差一个词，做的事情毫无共同之处。 `BrakeSwitch` 驱动母线上的一个电阻，
 *   作用于母线，且在逆变器关掉之后继续工作。这一个作用于 **轴** ：它是驱动器断电时不让垂直负载掉下去的东西。
 *
 *   契约按失效安全（弹簧施力、线圈释放）写，因为那是"掉一根线是一次停机而不是一次坠落"的那一种。非失效安全的抱闸也可以
 *   实现它： `engage()` 的语义是"做任何能让轴抱住的事"，线圈怎么接线是驱动方的事。
 *
 *   契约 **不** 携带时间。真实的抱闸夹紧或松开要几十毫秒，而多少毫秒是抱闸与其负载的属性、不是接口的属性——所以等待属于
 *   编排使能的那一方，并且是一个由集成方给出的数。
 *
 * @note [en] Optional in a `HalPolicy`: a horizontal axis with no brake is legal.
 *            `core::concepts::has_mechanical_brake_v` is how a sequencer asks whether it is there, and an absent brake
 *            means "there is nothing to sequence" rather than "assume it is released".
 * @note [zh] 在 `HalPolicy` 里是 **可选** 的：没有抱闸的水平轴是合法的。编排方用
 *            `core::concepts::has_mechanical_brake_v` 问它在不在，而缺席的含义是"没有可编排的东西"，不是"假定它松着"。
 * @since 0.1.0
 * @exporter
 */
class MechanicalBrake {
  public:
    virtual ~MechanicalBrake() = default;

    /**
     * @brief [en] Make the shaft hold.
     * @brief [zh] 让轴抱住。
     * @return [en] `Ok`, or whatever the driver reports.
     * @return [zh] `Ok`，或驱动方报告的错误。
     * @note [en] A refusal here is the dangerous one to swallow: a caller that ignores it and cuts the inverter anyway
     *            has dropped a load and called it a stop.
     * @note [zh] 这里的拒绝是那种吞掉就危险的：忽略它、照样关掉逆变器的调用方，把一次负载坠落叫做了一次停机。
     * @context [en] Any context, must not block.
     * @context [zh] 任意上下文，不得阻塞。
     * @since 0.1.0
     */
    virtual auto engage() noexcept -> ErrorCode = 0;

    /**
     * @brief [en] Let the shaft turn freely.
     * @brief [zh] 让轴自由转动。
     * @return [en] `Ok`, or whatever the driver reports.
     * @return [zh] `Ok`，或驱动方报告的错误。
     * @note [en] Only ever called when the motor is already producing torque; releasing a brake before that is how a
     *            vertical axis takes a step down.
     * @note [zh] 只在电机已经在出扭矩之后才被调用；在那之前松开抱闸，是垂直轴往下掉一格的方式。
     * @context [en] Any context, must not block.
     * @context [zh] 任意上下文，不得阻塞。
     * @since 0.1.0
     */
    virtual auto release() noexcept -> ErrorCode = 0;

    /**
     * @brief [en] Whether the brake is holding right now.
     * @brief [zh] 抱闸此刻是否在抱住。
     * @return [en] true while the shaft is held.
     * @return [zh] 轴被抱住时为 true。
     * @note [en] Asked rather than inferred: "I called `engage()`" and "the shaft is held" are different claims, and a
     *            sequencer that only knows the first will cut the inverter on the strength of a call that may have
     *            failed.
     * @note [zh] 去问而不是去推断："我调了 `engage()`"与"轴抱住了"是两句不同的话，而只知道前一句的编排方，会凭一次可能
     *            已经失败的调用去关逆变器。
     * @context [en] Any context, must not block.
     * @context [zh] 任意上下文，不得阻塞。
     * @since 0.1.0
     */
    [[nodiscard]] virtual auto engaged() noexcept -> bool = 0;
};

} // namespace hal

} // namespace foc
