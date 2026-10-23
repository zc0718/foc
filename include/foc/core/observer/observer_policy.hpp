// Conan::ImportStart
#pragma once
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Observer output: electrical angle and angular speed.
 *        [zh] 观测器输出：电角度与电角速度。
 *
 * @since 0.1.0
 * @exporter
 */
struct ObsState final {
    float theta_e; ///< [en] electrical angle in rad, normalised to [0, 2*pi). [zh] 电角度，rad，归一化到 [0, 2π)。
    float omega_e; ///< [en] electrical angular speed in rad/s. [zh] 电角速度，rad/s。
};


/**
 * @brief [en] Observer policy contract. Every observer exposes
 *        `auto update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState`
 *        and is checked by `hal::concepts::is_observer_v` (or by a static_assert at the assembly point).
 *        [zh] 观测器策略契约。每个观测器都提供
 *        `auto update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState`，
 *        并由 `is_observer_v`（或组装点的 static_assert）检查。
 *
 *        Why alpha-beta and not d-q: the whole point of an observer is to produce the angle that the
 *        d-q transform needs, so its inputs cannot already be in the rotating frame.
 *        为什么用 αβ 而不是 dq：观测器的职责正是给出 dq 变换所需的角，输入不可能已经在旋转系里。
 *
 * @context [en] Current-loop ISR: noexcept, no allocation. [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 */
namespace observer {


/**
 * @brief [en] Compile-time tag of the policy contract above (documentation anchor only).
 *        [zh] 上述策略契约的编译期锚点（仅作文档锚点，无运行期实体）。
 * @since 0.1.0
 * @exporter
 */
struct Policy final {};

} // namespace observer

} // namespace core

} // namespace foc
