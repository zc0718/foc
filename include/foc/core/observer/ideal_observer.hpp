// Conan::ImportStart
#pragma once
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Ideal observer for SIL only: it reports the plant's true angle and speed.
 * @brief [zh] 仅用于 SIL 的理想观测器：直接给出被控对象的真实角度与转速。
 * @warning [en] **Cheat channel.** This class deliberately bypasses the electrical model and is never allowed on a real
 *               motor: its purpose is to isolate controller behaviour from observer error in the simulation. Must not
 *               be used in a `HalPolicy` that ships.
 * @warning [zh] **作弊通道**。本类刻意绕过电气模型，绝不允许用于真实电机：它存在的意义是把 控制器行为与观测器误差在仿真
 *               中解耦。不得用于任何要出厂的 `HalPolicy`。
 * @context [en] SIL / tests only.
 * @context [zh] 仅 SIL / 测试。
 * @since 0.1.0
 * @exporter
 */
class IdealObserver final {
  public:
    /**
     * @brief [en] Feed the plant truth (called by the SIL harness, not by the controller).
     * @brief [zh] 输入仿真真值（由 SIL 夹具调用，不由控制器调用）。
     * @param theta_e_rad [en] true electrical angle in rad.
     * @param theta_e_rad [zh] 真实电角度，rad。
     * @param omega_e_rad_s [en] true electrical angular speed in rad/s.
     * @param omega_e_rad_s [zh] 真实电角速度，rad/s。
     * @since 0.1.0
     */
    void set_truth(float theta_e_rad, float omega_e_rad_s) noexcept;

    /**
     * @brief [en] Report the truth; the electrical inputs are ignored by design.
     * @brief [zh] 返回真值；按设计忽略电气输入。
     * @param v_ab [en] unused.
     * @param v_ab [zh] 未使用。
     * @param i_ab [en] unused.
     * @param i_ab [zh] 未使用。
     * @param dt_s [en] unused.
     * @param dt_s [zh] 未使用。
     * @return [en] the last truth set via set_truth().
     * @return [zh] 最近一次 set_truth() 设置的真值。
     * @since 0.1.0
     */
    auto update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState;

    /**
     * @brief [en] Clear the stored truth.
     * @brief [zh] 清除已存真值。
     * @since 0.1.0
     */
    void reset() noexcept;

  private:
    ObsState truth_;
};

} // namespace core

} // namespace foc
