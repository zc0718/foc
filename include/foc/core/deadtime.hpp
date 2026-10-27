// Conan::ImportStart
#pragma once
#include <cmath>
#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Adds back what the inverter's dead time takes away, then hands the request to a real
 *        modulator.
 *        [zh] 把逆变器死区吃掉的电压加回去，再把请求交给真正的调制器。
 *
 *        [en] Dead time clamps each phase to the rail its current is coming from for `t_dead`, so the
 *        inverter delivers `v_cmd - v_dead * sign(i_phase)` per phase. Commanding
 *        `v_desired + v_dead * sign(i_phase)` therefore lands on `v_desired`, provided the stage can
 *        see the phase currents -- which is exactly why `ModulatorInput` carries them.
 *        [zh] 死区在每个 `t_dead` 内把该相钳位到电流来自的那条母线，因此逆变器每相交付
 *        `v_cmd - v_dead * sign(i_phase)`。于是命令 `v_desired + v_dead * sign(i_phase)` 就能落到
 *        `v_desired`——前提是这一级看得到相电流，这正是 `ModulatorInput` 带着它们的原因。
 *
 *        [en] It is a *decorator*, not a flag on `Svpwm`: space-vector modulation and inverter
 *        linearisation are two different jobs, and any modulator can be compensated this way.
 *        [zh] 它是**装饰器**而不是 `Svpwm` 上的一个开关：空间矢量调制与逆变器线性化是两件事，而且
 *        任何调制器都能用这种方式被补偿。
 *
 * @note [en] The correction is applied in the **phase** domain (inverse Clarke, per-phase sign, Clarke
 *       back), because that is where the physical mechanism lives: the sign belongs to each phase, not
 *       to the vector. Applying it to the vector magnitude would compensate a different machine.
 *       [zh] 修正在**相域**进行（逆 Clarke、按相取符号、再 Clarke 回去），因为物理机制就在那里：
 *       符号属于每一相，而不属于矢量。按矢量幅值补偿等于在补偿另一台电机。
 *
 * @note [en] What it cannot do: near zero current the real inverter's transition is a finite slope
 *       with ringing, not a sign function, so a `sign()`-based correction is a *first-order* fix. The
 *       measured effect of exactly this stage is in SIL S13 (worst tracking error ×1.48 → ×1.0x).
 *       [zh] 它做不到的：零电流附近真实逆变器的过渡是一段有限斜率并伴随振荡，而不是符号函数，因此
 *       基于 `sign()` 的修正是**一阶**修正。这一级究竟消掉了多少，实测见 SIL S13（最差跟踪误差
 *       ×1.48 → ×1.0x）。
 *
 * @context [en] Current-loop ISR: noexcept, no allocation, one branch when disabled.
 *           [zh] 电流环 ISR：noexcept、不分配、未启用时只有一个分支。
 * @since 0.1.0
 * @exporter
 */
template <typename Modulator> class DeadTimeCompensator final {
  public:
    /**
     * @brief [en] The two numbers this stage needs.
     *        [zh] 这一级需要的两个数。
     *
     * @note [en] `deadtime_v == 0` disables the correction entirely (the request passes through
     *       untouched), which is how the same build can be measured with and without it.
     *       [zh] `deadtime_v == 0` 完全关闭修正（请求原样透传），同一个构建因此可以用它开/关来测量。
     * @since 0.1.0
     * @exporter
     */
    struct Config final {
        float deadtime_v;    ///< [en] (t_dead/T_pwm)*v_bus, V; 0 = off. [zh] (t_dead/T_pwm)*v_bus，V；0 = 关闭。
        float min_current_a; ///< [en] below this |i| the sign is not trusted, A. [zh] |i| 低于此值时不信任符号，A。
    };

    DeadTimeCompensator() noexcept = default;

    explicit DeadTimeCompensator(const Config& config) noexcept : config_(config) {}

    /**
     * @brief [en] Compensated duties for this step.
     *        [zh] 本步补偿后的占空比。
     *
     * @param request [en] desired stator voltage, bus and measured currents. [zh] 期望定子电压、母线与实测电流。
     * @return [en] duties plus saturation and status, from the wrapped modulator. [zh] 占空比与饱和/状态，来自被包装的调制器。
     * @since 0.1.0
     */
    [[nodiscard]] auto modulate(const ModulatorInput& request) noexcept -> SvpwmResult {
        if (!(config_.deadtime_v > 0.0F)) {
            return Modulator::modulate(request);
        }

        // Per phase: the sign is the phase's own, and below `min_current_a` it is noise rather than
        // information -- commanding a correction there would inject a square wave into a signal that
        // is mostly offset error.
        // 按相：符号属于该相自己；|i| 低于 `min_current_a` 时它是噪声而不是信息——在那里施加修正等于
        // 往一个基本只有偏置误差的信号里注入方波。
        const Uvw<float> v_uvw = inv_clarke(request.v_ab);
        const float gain = config_.deadtime_v;
        const Uvw<float> corrected{v_uvw.u + (gain * sign_of(request.i_uvw.u, config_.min_current_a)),
                                   v_uvw.v + (gain * sign_of(request.i_uvw.v, config_.min_current_a)),
                                   v_uvw.w + (gain * sign_of(request.i_uvw.w, config_.min_current_a))};

        ModulatorInput forwarded = request;
        forwarded.v_ab = clarke(corrected);
        return Modulator::modulate(forwarded);
    }

    /// [en] The configuration in force. [zh] 当前生效的配置。
    [[nodiscard]] auto config() const noexcept -> const Config& { return config_; }

    /// [en] Change the configuration (dead time belongs to the board, so it is a boot-time value). [zh] 修改配置（死区属于板子，因此是启动期取值）。
    void configure(const Config& config) noexcept { config_ = config; }

  private:
    /// [en] +1 / -1 by sign, 0 inside the dead band. [zh] 按符号取 ±1，死区内取 0。
    [[nodiscard]] static auto sign_of(float value, float deadband) noexcept -> float {
        if (!(value > deadband)) {
            if (!(value < -deadband)) {
                return 0.0F;
            }
            return -1.0F;
        }
        return 1.0F;
    }

    Config config_{};
};

} // namespace core

} // namespace foc
