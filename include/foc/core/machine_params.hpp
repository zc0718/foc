// Conan::ImportStart
#pragma once
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file machine_params.hpp
 *
 * @brief [en] The electrical facts about a machine that its algorithms need, in SI units.
 * @brief [zh] 电机的算法所需的电气事实，采用 SI 单位。
 * @since 0.1.0
 */

namespace foc {

namespace core {


/**
 * @brief [en] The electrical facts about a machine that its algorithms need.
 *
 *   Resistance, the two inductances, and the rotor flux linkage.
 *
 *   Why this type exists instead of every observer having its own numbers: parameter identification produces exactly
 *   these four, and without a shared value type the result has nowhere to go -- each policy would keep its own copy of
 *   a number that is supposed to describe the same machine. Observers implement `apply_params(const MachineParams&)`
 *   when they can take an update, and `has_apply_params_v` is how the motor layer finds out.
 *
 * @brief [zh] 算法所需的电机电气事实：电阻、两个电感、转子磁链。
 *
 *   为什么要有这个类型，而不是让每个观测器自带一套数字：参数辨识产出的正是这四个，没有共享的值类型， 结果就无处可去——
 *   每个策略会各存一份本该描述 **同一台** 机器的数字。观测器若能接受更新就实现 `apply_params(const MachineParams&)`，电
 *   机层用 `has_apply_params_v` 来发现。
 *
 * @note [en] Units are SI and, more importantly, they are *the same units everywhere*: `rs` is at whatever temperature
 *            the caller means (the plant does not tell you what it is), and `psi_f` is the flux linkage, not the torque
 *            constant -- `psi_f = Kt / (1.5 * pole_pairs)`, which `motor_config.hpp` can compute from a nameplate.
 *            Mixing Kt into a voltage equation is dimensionally wrong and is PRD erratum D24 in this repository.
 * @note [zh] SI 单位，更重要的是 **到处是同一套单位** ： `rs` 是调用方所指温度下的值（plant 不会告诉你 温度）， `psi_f`
 *            是磁链而不是转矩常数—— `psi_f = Kt / (1.5 * pole_pairs)`， `motor_config.hpp` 可由铭牌算出。把 Kt 塞进电压
 *            方程量纲不对，在本仓库里是 PRD 勘误 D24。
 * @since 0.1.0
 * @exporter
 */
struct MachineParams final {
    /**
     * @brief [en] stator resistance in ohm, >= 0.
     * @brief [zh] 定子电阻，Ω，>= 0。
     */
    float rs;
    /**
     * @brief [en] d-axis inductance in H, > 0.
     * @brief [zh] d 轴电感，H，> 0。
     */
    float ld;
    /**
     * @brief [en] q-axis inductance in H, > 0.
     * @brief [zh] q 轴电感，H，> 0。
     */
    float lq;
    /**
     * @brief [en] rotor (permanent-magnet) flux linkage in Wb.
     * @brief [zh] 转子（永磁）磁链，Wb。
     */
    float psi_f;
};


} // namespace core

} // namespace foc
