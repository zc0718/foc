// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file bus_current_limit.hpp
 *
 * @brief [en] The ceiling on what the bus may draw, expressed as an authority over the torque command.
 * @brief [zh] 母线允许抽取多少的上限，表达成对转矩指令的权限。
 * @since 0.1.0
 */

namespace foc {

namespace core {

/**
 * @brief [en] What the limiter aims at and how fast it may move.
 * @brief [zh] 限制器瞄准什么，以及它能多快地移动。
 * @since 0.1.0
 * @exporter
 */
struct BusCurrentLimitConfig final {
    /**
     * @brief [en] the ceiling the bus current may not exceed, A, positive when the drive is drawing.
     * @brief [zh] 母线电流不得超过的上限，A，驱动器抽取时为正值。
     */
    float i_max_a;
    /**
     * @brief [en] how far below the ceiling to aim, A.
     * @brief [zh] 瞄准的上限之下留多少，A。
     * @note [en] Not politeness. A limiter that aims *at* its own ceiling spends its life above and below it, and the
     *            number an oscilloscope reports would be the overshoot rather than the limit.
     * @note [zh] 不是客气。瞄着自家上限的限制器，一辈子都在它上下摆动，而示波器报出来的数会是超调而不是限值。
     */
    float headroom_a;
    /**
     * @brief [en] integral gain on the current error, in authority per ampere-second.
     * @brief [zh] 电流误差上的积分增益，单位是权限每（安培·秒）。
     * @note [en] Integral rather than a ratio, because a ratio has a **biased** fixed point: at low speed the bus
     *            current grows with the square of the torque, so `scale = aim / measured` settles where that equation
     *            crosses the plant rather than where the current reaches the aim -- this class did exactly that in its
     *            first version and sat 17 % above its own ceiling. An integrator drives the *error* to zero whatever
     *            the exponent is, which is the one property a limit needs and the one a ratio cannot promise.
     * @note [zh] 积分而不是比值，因为比值的不动点 **有偏** ：低速时母线电流随转矩的平方增长，于是
     *            `scale = aim / measured` 停在那条方程与 plant 的交点上，而不是停在电流到达瞄准值的地方——这个类的第一版
     *            正是如此，稳稳停在自己上限之上 17 %。积分器把 **误差** 驱到零，与指数无关，而这正是限值必须有、比值给
     *            不出的那条性质。
     */
    float gain_per_a_s;
    /**
     * @brief [en] how fast the allowance may fall, per second, in authority (1 = full torque).
     * @brief [zh] 权限每秒最多下降多少（1 = 全转矩）。
     */
    float fall_per_s;
    /**
     * @brief [en] how fast it may come back, per second. Slower than `fall_per_s` on purpose.
     * @brief [zh] 权限每秒最多回升多少。故意比 `fall_per_s` 慢。
     * @note [en] The asymmetry is what keeps a drive at the boundary out of a control-rate oscillation: giving
     *            authority back at the same speed it is taken away is a two-state flip-flop, and the machine feels it.
     * @note [zh] 这个不对称是让边界上的驱动器不陷入控制频率抖动的原因：把权限以被拿走的速度还回去，是一个两态翻转，机器
     *            能感觉到它。
     */
    float rise_per_s;
};

/**
 * @brief [en] What the limiter decided this step.
 * @brief [zh] 限制器本步的决定。
 * @since 0.1.0
 * @exporter
 */
struct BusCurrentLimitOutput final {
    /**
     * @brief [en] the multiplier to apply to the torque command, 1 = untouched.
     * @brief [zh] 要乘到转矩指令上的系数，1 = 未动。
     */
    float scale;
    /**
     * @brief [en] whether a bus current arrived this step.
     * @brief [zh] 本步有没有收到母线电流。
     */
    bool measured;
    /**
     * @brief [en] whether the authority is below 1, i.e. the limiter is doing something.
     * @brief [zh] 权限是否低于 1，即限制器正在起作用。
     */
    bool limiting;
};

/**
 * @brief [en] Holds the bus current under a ceiling by taking authority from the torque command.
 *
 *   Three things worth knowing before reading the code, because each of them is a decision that the obvious alternative
 *   gets wrong:
 *
 *   1. **It is feedback, and it says so.** The law is `scale = aim / measured`, slewed -- not a model of the machine. A
 *   model-based limit would need `vd`, `vq`, `id` and `iq` at every step and would be wrong by exactly as much as the
 *   model is: the bus current is the one quantity in this loop that *is* measured, so it is the one that should be
 *   closed on.
 *
 *   2. **A missing measurement is not a pass.** With `NaN` in, the authority relaxes at `rise_per_s` -- the drive is
 *   not punished for a dead shunt -- and the output says `measured == false`, so a caller that needs the limit enforced
 *   can refuse to run rather than believe it is being protected.
 *
 *   3. **It never gives authority back faster than it takes it away.** See `rise_per_s`.
 *
 *   What it is **not**: it is not a protection and it does not stop anything. A bus current that exceeds the ceiling
 *   because the machine is regenerating, or because a command is far outside what any authority can hide, is the
 *   protector's business -- this type only ever scales a number.
 *
 * @brief [zh] 通过从转矩指令上拿走权限把母线电流压在限值之下，然后再慢慢还回去。
 *
 *   读代码之前值得先知道三件事，每一件都是"朴素替代做法会做错"的决定：
 *
 *   1. **它是反馈，而且它自己说了。** 律是 `scale = aim / measured` 再限斜率，不是机器的模型。基于模型的限值每步都要
 *   `vd`、 `vq`、 `id`、 `iq`，而且会恰好错成模型错的那样多：母线电流是这个环里 **唯一真的被量到的** 量，所以它也是唯一
 *   应该被闭环的量。
 *
 *   2. **没有测量不是放行。** 输入 `NaN` 时权限按 `rise_per_s` 松开——驱动器不为通路上的一个死分流电阻受过——而输出报
 *   `measured == false`，于是需要这条限值真的生效的调用方可以拒绝运行，而不是相信自己正被保护着。
 *
 *   3. **它还回去的速度永远不快于拿走的速度。** 见 `rise_per_s`。
 *
 *   它 **不是** 什么：它不是保护，也不停任何东西。母线电流越过上限——因为机器在回馈，或者因为指令远在任何权限都遮不住的
 *   地方——是保护器的事；本类型只缩放一个数。
 *
 * @context [en] Current-loop ISR, every step, before the reference is handed to the loop.
 * @context [zh] 电流环 ISR，每步，在给定送进控制环之前。
 * @since 0.1.0
 * @exporter
 */
class BusCurrentLimit final {
  public:
    /**
     * @brief [en] Accept the ceiling and the two rates, or refuse them before a single step runs.
     * @brief [zh] 接受上限与两个速率，或者在还没跑一步之前就拒绝它们。
     * @param config [en] the ceiling, the headroom and the two rates.
     * @param config [zh] 上限、余量与两个速率。
     * @return [en] `Ok`, or `Param` for a ceiling that is not positive, a headroom that is not below it, or rates that
     *              are not positive.
     * @return [zh] `Ok`；上限不为正、余量不小于它、或速率不为正时返回 `Param`。
     * @since 0.1.0
     */
    auto configure(const BusCurrentLimitConfig& config) noexcept -> ErrorCode {
        const bool ceiling_ok = (config.i_max_a > 0.0F) && std::isfinite(config.i_max_a);
        const bool headroom_ok = (config.headroom_a >= 0.0F) && (config.headroom_a < config.i_max_a);
        const bool rates_ok = (config.fall_per_s > 0.0F) && (config.rise_per_s > 0.0F) && (config.gain_per_a_s > 0.0F);
        if (!ceiling_ok || !headroom_ok || !rates_ok) {
            return ErrorCode::Param;
        }
        config_ = config;
        configured_ = true;
        output_ = BusCurrentLimitOutput{1.0F, false, false};
        authority_ = 1.0F;
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] Decide this step's authority.
     * @brief [zh] 决定本步的权限。
     * @param bus_current_a [en] measured current into the inverter, A; `NaN` when nothing measured it.
     * @param bus_current_a [zh] 实测流入逆变器的电流，A；没人测到时为 `NaN`。
     * @param dt_s [en] step length in s; `<= 0` is refused as `Param` and changes nothing.
     * @param dt_s [zh] 步长，s； `<= 0` 被拒绝为 `Param` 且什么都不改。
     * @return [en] the decision; before `configure()` the authority is 1 and `measured` is false.
     * @return [zh] 决定； `configure()` 之前权限为 1 且 `measured` 为 false。
     * @post [en] `scale` never rises faster than `rise_per_s` per second and never falls faster than `fall_per_s`.
     * @post [zh] `scale` 每秒上升不快于 `rise_per_s`，下降不快于 `fall_per_s`。
     * @since 0.1.0
     */
    auto step(float bus_current_a, float dt_s) noexcept -> const BusCurrentLimitOutput& {
        if (!configured_ || !(dt_s > 0.0F)) {
            return output_;
        }
        output_.measured = std::isfinite(bus_current_a);
        float wanted = 1.0F;
        if (output_.measured) {
            const float aim_a = config_.i_max_a - config_.headroom_a;
            // Integral on the error, with the authority clamped to a restriction: it may hand back what it took, never
            // more than 1. A current below the aim pushes the authority *up*, which is exactly how the fixed point ends
            // up where the current equals the aim instead of where a ratio happens to balance.
            // 误差上的积分，且权限被夹在"限制"这一侧：它可以把拿走的还回来，但不超过 1。低于瞄准值的电流会把权限往上推，
            // 这正是让不动点落在"电流等于瞄准值"而不是"比值恰好平衡"的方式。
            wanted = authority_ - (config_.gain_per_a_s * (bus_current_a - aim_a) * dt_s);
        }
        // Both directions are rate-limited against the **one** state, `authority_`, and neither has a state of its own.
        // That is not a simplification: a second integrator tracking the same quantity is a second thing that can be
        // out of step with it, and the first version of this class had exactly that bug -- the rise limiter started at
        // zero, so an unlimited drive was strangled to 1e-4 authority on its first step.
        // 两个方向都对 **同一个** 状态 `authority_` 限斜率，谁也不带自己的状态。这不是简化：第二个跟踪同一个量的积分器，就是
        // 第二个可能与它不同步的东西，而这个类的第一版恰好有这个缺陷——上升限幅器从零开始，于是一台本该不受限的驱动器在第一步
        // 就被掐到 1e-4 的权限。
        const float floor_a = authority_ - (config_.fall_per_s * dt_s);
        const float ceiling_a = authority_ + (config_.rise_per_s * dt_s);
        if (wanted < authority_) {
            authority_ = (wanted > floor_a) ? wanted : floor_a;
        } else {
            authority_ = (wanted < ceiling_a) ? wanted : ceiling_a;
        }
        // Saturation, after the rate limits rather than inside them. A floor expressed as "how far it may fall" is a
        // *step size*, not a bound: the integrator can step past zero by less than one step and stay there, and this
        // class did exactly that -- a negative authority is a torque command in the opposite direction, which is worse
        // than any limit it was trying to enforce.
        // 饱和放在限斜率 **之后** 而不是里面。用"最多能降多少"表达的底是一个 **步长** ，不是边界：积分器可以一步跨过零并停在
        // 那里，而这个类正是如此——负权限是一条反方向的转矩指令，比它想强制的任何限值都糟。
        if (authority_ < 0.0F) {
            authority_ = 0.0F;
        }
        if (authority_ > 1.0F) {
            authority_ = 1.0F;
        }
        output_.scale = authority_;
        output_.limiting = authority_ < (1.0F - 1e-6F);
        return output_;
    }

    /**
     * @brief [en] The decision of the last step.
     * @brief [zh] 最近一步的决定。
     * @return [en] the output; `{1, false, false}` before the first step.
     * @return [zh] 输出；首步之前为 `{1, false, false}`。
     * @since 0.1.0
     */
    [[nodiscard]] auto output() const noexcept -> const BusCurrentLimitOutput& { return output_; }

    /**
     * @brief [en] Back to full authority, as at power-on.
     * @brief [zh] 回到全权限，如同上电时。
     * @since 0.1.0
     */
    void reset() noexcept {
        authority_ = 1.0F;
        output_ = BusCurrentLimitOutput{1.0F, false, false};
    }

  private:
    BusCurrentLimitConfig config_{};
    float authority_{1.0F};
    BusCurrentLimitOutput output_{1.0F, false, false};
    bool configured_{false};
};

} // namespace core

} // namespace foc
