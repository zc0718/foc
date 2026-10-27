// Conan::ImportStart
#pragma once
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Everything a reference generator may look at, filled once per current-loop step.
 *        [zh] 参考生成器可以看的全部输入，每个电流环周期填一次。
 *
 *        [en] `iq_command_a` carries the torque setpoint in torque mode and the speed loop's
 *        output in speed mode: the generator never has to know which mode is active, because by
 *        the time it runs the outer loop has already collapsed the command onto the q axis.
 *        [zh] `iq_command_a` 在转矩模式下是转矩给定，在速度模式下是速度环的输出：参考生成器
 *        不需要知道当前是哪种模式，因为它运行时外环已经把给定归约到 q 轴上了。
 *
 * @context [en] Current-loop ISR, read-only for the policy. [zh] 电流环 ISR；对策略只读。
 * @since 0.1.0
 * @exporter
 */
struct ReferenceInput final {
    float iq_command_a;  ///< [en] q-axis command in A. [zh] q 轴给定，A。
    float omega_e_rad_s; ///< [en] electrical angular speed in rad/s. [zh] 电角速度，rad/s。
    float v_bus_v;       ///< [en] DC link voltage in V, this step. [zh] 本周期母线电压，V。
    Dq<float> i_meas;    ///< [en] measured dq current in A. [zh] 实测 dq 电流，A。
};


/**
 * @brief [en] The default reference generator: d axis held at zero, q axis passed through.
 *        [zh] 默认参考生成器：d 轴恒为 0，q 轴直通。
 *
 *        [en] "d = 0" is optimal for a surface-mounted machine and nothing more: an interior-PM
 *        machine earns extra torque from a negative d current (MTPA), and any machine needs d
 *        current to run past its base speed (field weakening). That is why the split is a policy
 *        with an input struct, rather than a `0.0F` written into the current loop.
 *        [zh] “d = 0”只对表贴式电机最优，仅此而已：内嵌式电机靠负 d 轴电流拿到额外转矩（MTPA），
 *        任何电机超过基速后都要靠 d 轴电流做弱磁。所以这个“拆分”是一个带输入结构的策略，而不是
 *        写死在电流环里的 `0.0F`。
 *
 * @note [en] A reference generator must be default constructible: the five-policy aggregate form
 *       leaves this member defaulted, which is what keeps existing assembly points source-compatible.
 *       [zh] 参考生成器必须可默认构造：五元聚合初始化形式会把该成员留作默认值，现有装配点因此保持
 *       源码兼容。
 * @context [en] Current-loop ISR: noexcept, no allocation. [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class NullReferenceGenerator final {
  public:
    /**
     * @brief [en] Split the q-axis command into the dq reference.
     *        [zh] 把 q 轴给定拆分成 dq 参考。
     *
     * @param in [en] inputs of this control step. [zh] 本控制步的输入。
     * @return [en] the dq reference in A; `Motor` clamps it to the nameplate and drops non-finite
     *         values before it reaches the current loop.
     *         [zh] dq 参考，A；`Motor` 在送入电流环之前按铭牌限幅并丢弃非有限值。
     * @since 0.1.0
     */
    [[nodiscard]] auto generate(const ReferenceInput& in) noexcept -> Dq<float> {
        return Dq<float>{0.0F, in.iq_command_a};
    }
};

} // namespace core

} // namespace foc
