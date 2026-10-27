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
 * @brief [en] Adds back what the inverter's dead time takes away, then hands
 * the request to a real modulator. [zh]
 * 把逆变器死区吃掉的电压加回去，再把请求交给真正的调制器。
 *
 *        [en] Dead time clamps each phase to the rail its current is coming
 * from for `t_dead`, so the inverter delivers `v_cmd - v_dead * sign(i_phase)`
 * per phase. Commanding `v_desired + v_dead * sign(i_phase)` therefore lands on
 * `v_desired`, provided the stage can see the phase currents -- which is
 * exactly why `ModulatorInput` carries them. [zh] 死区在每个 `t_dead`
 * 内把该相钳位到电流来自的那条母线，因此逆变器每相交付 `v_cmd - v_dead *
 * sign(i_phase)`。于是命令 `v_desired + v_dead * sign(i_phase)` 就能落到
 *        `v_desired`——前提是这一级看得到相电流，这正是 `ModulatorInput`
 * 带着它们的原因。
 *
 *        [en] It is a *decorator*, not a flag on `Svpwm`: space-vector
 * modulation and inverter linearisation are two different jobs, and any
 * modulator can be compensated this way. [zh] 它是**装饰器**而不是 `Svpwm`
 * 上的一个开关：空间矢量调制与逆变器线性化是两件事，而且
 *        任何调制器都能用这种方式被补偿。
 *
 * @note [en] The correction is applied in the **phase** domain (inverse Clarke,
 * per-phase sign, Clarke back), because that is where the physical mechanism
 * lives: the sign belongs to each phase, not to the vector. Applying it to the
 * vector magnitude would compensate a different machine. [zh]
 * 修正在**相域**进行（逆 Clarke、按相取符号、再 Clarke
 * 回去），因为物理机制就在那里：
 *       符号属于每一相，而不属于矢量。按矢量幅值补偿等于在补偿另一台电机。
 *
 * @note [en] What it cannot do: near zero current the real inverter's
 * transition is a finite slope with ringing, not a sign function, so a
 * `sign()`-based correction is a *first-order* fix. The measured effect of
 * exactly this stage is in SIL S13 (worst tracking error ×1.48 → ×1.0x). [zh]
 * 它做不到的：零电流附近真实逆变器的过渡是一段有限斜率并伴随振荡，而不是符号函数，因此
 *       基于 `sign()` 的修正是**一阶**修正。这一级究竟消掉了多少，实测见 SIL
 * S13（最差跟踪误差 ×1.48 → ×1.0x）。
 *
 * @context [en] Current-loop ISR: noexcept, no allocation, one branch when
 * disabled. [zh] 电流环 ISR：noexcept、不分配、未启用时只有一个分支。
 * @since 0.1.0
 * @exporter
 */
template <typename Modulator> class DeadTimeCompensator final {
  public:
    /**
   * @brief [en] The two numbers this stage needs.
   *        [zh] 这一级需要的两个数。
   *
   * @note [en] `deadtime_v == 0` disables the correction entirely (the request
   * passes through untouched), which is how the same build can be measured with
   * and without it. [zh] `deadtime_v == 0`
   * 完全关闭修正（请求原样透传），同一个构建因此可以用它开/关来测量。
   * @since 0.1.0
   * @exporter
   */
    struct Config final {
        float deadtime_v;    ///< [en] (t_dead/T_pwm)*v_bus, V; 0 = off. [zh]
                             ///< (t_dead/T_pwm)*v_bus，V；0 = 关闭。
        float min_current_a; ///< [en] below this |i| the sign is not trusted, A.
                             ///< [zh] |i| 低于此值时不信任符号，A。
        /// [en] Peak switching ripple in A; > 0 turns the hard `sign()` step into
        /// the ramp that the
        ///      averaged dead-time error actually follows. 0 keeps the legacy step.
        ///      See `predicted_ripple_a()`. [zh] 开关纹波峰值，A；> 0 时把 `sign()`
        ///      的硬阶跃换成死区误差平均值真正服从的斜坡。 0 保持旧的阶跃。见
        ///      `predicted_ripple_a()`。
        float i_ripple_a;
    };

    /**
   * @brief [en] The ripple amplitude the ramp needs, from numbers the board
   * already knows. [zh] 斜坡需要的纹波幅值，由板子本来就知道的数算出来。
   *
   * @param v_bus [en] DC link voltage in V. [zh] 母线电压，V。
   * @param dt_s [en] switching (not control) period in s. [zh]
   * 开关周期（不是控制周期），s。
   * @param l_h [en] inductance the ripple flows through in H. [zh]
   * 纹波流过的电感，H。
   * @return [en] peak ripple in A, i.e. half of `v_bus*dt/(4L)` (the
   * peak-to-peak value at 50 % duty). [zh] 纹波峰值，A，即 `v_bus·dt/(4L)`（50%
   * 占空比下的峰峰值）的一半。
   *
   * @note [en] It is an estimate with one job: give the ramp a physical scale
   * instead of a hand-tuned number. Being wrong by a factor of two changes how
   * quickly the correction fades in, not whether it does. [zh]
   * 这个估计只干一件事：给斜坡一个物理尺度，而不是一个手调的数字。差两倍只改变修正淡入的
   *       快慢，不改变它是否淡入。
   * @since 0.1.0
   */
    [[nodiscard]] static constexpr auto predicted_ripple_a(float v_bus, float dt_s, float l_h) noexcept -> float {
        return ((v_bus > 0.0F) && (dt_s > 0.0F) && (l_h > 0.0F)) ? ((v_bus * dt_s) / (8.0F * l_h)) : 0.0F;
    }

    DeadTimeCompensator() noexcept = default;

    explicit DeadTimeCompensator(const Config& config) noexcept : config_(config) {}

    /**
   * @brief [en] Compensated duties for this step.
   *        [zh] 本步补偿后的占空比。
   *
   * @param request [en] desired stator voltage, bus and measured currents. [zh]
   * 期望定子电压、母线与实测电流。
   * @return [en] duties plus saturation and status, from the wrapped modulator.
   * [zh] 占空比与饱和/状态，来自被包装的调制器。
   * @since 0.1.0
   */
    [[nodiscard]] auto modulate(const ModulatorInput& request) noexcept -> SvpwmResult {
        if (!(config_.deadtime_v > 0.0F)) {
            return Modulator::modulate(request);
        }

        // Per phase: the sign is the phase's own, and below `min_current_a` it is
        // noise rather than information -- commanding a correction there would
        // inject a square wave into a signal that is mostly offset error.
        // 按相：符号属于该相自己；|i| 低于 `min_current_a`
        // 时它是噪声而不是信息——在那里施加修正等于
        // 往一个基本只有偏置误差的信号里注入方波。
        const Uvw<float> v_uvw = inv_clarke(request.v_ab);
        const float gain = config_.deadtime_v;

        // With a ripple scale the correction is the ramp the *averaged* error
        // follows: over one switching period the phase current sweeps +-I_ripple,
        // spends `(1 - i/I_ripple)/2` of it on each rail, and the volt-seconds that
        // leaves behind are `deadtime_v * i/I_ripple` -- linear, saturating at |i|
        // = I_ripple. That is why a `sign()` step is right only when the current is
        // much larger than its own ripple, and why the step is *worse* than nothing
        // when it is not.
        // 有了纹波尺度，修正就是**平均**误差所服从的那条斜坡：一个开关周期内相电流扫过
        // ±I_ripple，在每条 母线上各待 `(1 − i/I_ripple)/2`，留下的伏秒就是
        // `deadtime_v·i/I_ripple`——线性，在 |i| = I_ripple 处饱和。这就是为什么
        // `sign()` 只在电流远大于自身纹波时才对，也是为什么否则它**比不补 更差**。
        const float ripple = config_.i_ripple_a;
        const float scale = (ripple > config_.min_current_a) ? ripple : config_.min_current_a;
        const Uvw<float> corrected{static_cast<float>(v_uvw.u + (gain * shape_of(request.i_uvw.u, scale, ripple))),
                                   static_cast<float>(v_uvw.v + (gain * shape_of(request.i_uvw.v, scale, ripple))),
                                   static_cast<float>(v_uvw.w + (gain * shape_of(request.i_uvw.w, scale, ripple)))};

        ModulatorInput forwarded = request;
        forwarded.v_ab = clarke(corrected);
        return Modulator::modulate(forwarded);
    }

    /// [en] The configuration in force. [zh] 当前生效的配置。
    [[nodiscard]] auto config() const noexcept -> const Config& { return config_; }

    /// [en] Change the configuration (dead time belongs to the board, so it is a
    /// boot-time value). [zh] 修改配置（死区属于板子，因此是启动期取值）。
    void configure(const Config& config) noexcept { config_ = config; }

  private:
    /// [en] +1 / -1 by sign, 0 inside the dead band. [zh] 按符号取 ±1，死区内取
    /// 0。
    [[nodiscard]] static auto sign_of(float value, float deadband) noexcept -> float {
        if (!(value > deadband)) {
            if (!(value < -deadband)) {
                return 0.0F;
            }
            return -1.0F;
        }
        return 1.0F;
    }

    /// [en] The ramp when a ripple scale is known, the legacy step when it is
    /// not. [zh] 知道纹波尺度时用斜坡， 不知道时用旧的阶跃。
    [[nodiscard]] static auto shape_of(float value, float scale, float ripple) noexcept -> float {
        if (!(ripple > 0.0F)) {
            return sign_of(value, scale);
        }
        const float normalized = value / ((scale > 0.0F) ? scale : ripple);
        if (!(normalized > -1.0F)) {
            return -1.0F;
        }
        if (!(normalized < 1.0F)) {
            return 1.0F;
        }
        return normalized;
    }

    Config config_{};
};

} // namespace core

} // namespace foc
