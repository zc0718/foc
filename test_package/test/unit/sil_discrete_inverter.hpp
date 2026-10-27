// Conan::ImportStart
#pragma once

#include <cmath>
#include <cstdint>

#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace sil {

using core::Ab;
using core::Uvw;

/**
 * @brief [en] What one inverter leg needs to be simulated.
 *        [zh] 一条逆变器桥臂要仿真所需要的全部参数。
 *
 * @since 0.1.0
 */
struct InverterConfig final {
  /// [en] switching frequency in Hz. Must divide the control rate exactly (see
  /// `set_control_period()`).
  ///      [zh] 开关频率，Hz。必须整除控制频率（见 `set_control_period()`）。
  float f_sw_hz;
  /// [en] dead time per transition in s, applied to turn-on only. [zh]
  /// 每次换向的死区，s，只延迟开通。
  float t_dead_s;
  /// [en] conduction drop per device in V; 0 = ideal switches. [zh]
  /// 每个器件的导通压降，V；0 = 理想开关。
  float v_fwd_v;
  /// [en] integration sub-steps per switching period; only limits how finely
  /// the ripple is drawn.
  ///      [zh] 每个开关周期的积分子步数；只决定纹波画得多细。
  std::int32_t sub_per_switch;
};

/**
 * @brief [en] A switching inverter, as the plant sees it: per-leg gate signals,
 * dead time, freewheeling diodes, and the three rail voltages that result. [zh]
 * 从被控对象看过去的开关逆变器：每条桥臂的门极信号、死区、续流二极管，以及由此得到的三相
 *        极电压。
 *
 *        [en] Why this exists: the average-value model in `PlantHal` holds one
 * voltage vector for a whole control period, so the current it produces is
 * smooth and the dead-time error is a formula that was handed to it. Any
 * observer that reacts to ripple -- a sliding-mode observer above all -- is
 * then being graded against the formula rather than against hardware. Here the
 * dead-time volt-seconds are not an input: they *fall out* of "turn-off is
 * immediate, turn-on is delayed, and whichever device still conducts is decided
 * by the current". The identity `v_err = -(t_dead/T)*v_bus*sign(i)` is
 * therefore a property that can be *checked* (S19 V2), not an assumption that
 * is applied (A2's ramp is checked the same way, S19 V6). [zh]
 * 存在的理由：`PlantHal`
 * 里的平均值模型在整个控制周期内保持一个电压矢量，于是它产生的
 *        电流是平滑的、死区误差是一条被喂给它的公式。任何对纹波有反应的观测器——首当其冲是滑模
 *        观测器——因此是在对着公式打分，而不是对着硬件打分。这里死区伏秒不是输入：它从"关断立即、
 *        开通延迟、仍导通的那只器件由电流决定"里**落出来**。所以 `v_err =
 * −(t_dead/T)·v_bus·sign(i)` 是一条可以被**检验**的性质（S19
 * V2），而不是被施加的假设（A2 的斜坡同样如此，S19 V6）。
 *
 *        [en] Carrier: centre-aligned, so the high window of leg x is
 * `[(1-d)/2, (1+d)/2]*T`. Every switching period is then self-contained -- at
 * the period boundary all three legs are commanded low -- so the model keeps
 * **no state across periods** and any period is reproducible from its own
 *        inputs. That is what makes single-period verification (V1/V2/V3)
 * meaningful. [zh] 载波：中心对齐，于是 x 相的高侧窗口是 `[(1-d)/2,
 * (1+d)/2]·T`。每个开关周期因此是自包含的
 *        ——周期边界处三相都被命令为低侧——所以模型**不跨周期存状态**，任何周期都能由它自己的输入复现。
 *        这正是单周期验证（V1/V2/V3）有意义的原因。
 *
 *        [en] Resolution: within a sub-step the high-side and dead-time windows
 * are integrated *as fractions*, so the **mean** voltage over a sub-step is
 * exact and dead time is never quantised; the sub-step length only limits how
 * finely the ripple is resolved. A coarse model therefore still gets V2 right,
 * which is the property that lets CI use 20 sub-steps instead of 2000. [zh]
 * 分辨率：子步内高侧窗口与死区窗口按**比例**积分，所以子步的**平均**电压是精确的、死区从不被
 *        量化；子步长度只限制纹波的分辨率。因此粗模型仍然能算对 V2，这才让 CI
 * 能用 20 个子步而不是 2000 个。
 *
 * @note [en] The neutral point is not modelled: the machine is a floating star,
 * so only line-to-line voltages matter and the common mode cancels in the
 * Clarke transform. That is also why `clarke(pole voltages)` must reproduce the
 * commanded `v_ab` exactly when dead time is off (V1) -- the same identity that
 * makes `Svpwm`'s midpoint injection equivalent to the classic sector
 *       formulation.
 *       [zh] 中性点不建模：电机是浮地星形，只有线电压有意义，共模在 Clarke
 * 变换里自动消掉。这也是为什么 死区关闭时 `clarke(极电压)` 必须精确复现指令
 * `v_ab`（V1）——正是同一条恒等式让 `Svpwm` 的中点 注入与经典扇区公式等价。
 * @note [en] Known idealisations, stated so the numbers are read correctly: the
 * current sign inside a dead-time window is taken at the sub-step boundary (so
 * a zero crossing inside one sub-step is resolved to sub-step accuracy);
 * reverse recovery, ringing and device capacitance are absent. [zh]
 * 已知的理想化，写下来是为了让数字被正确解读：死区窗口内的电流方向取子步边界处的值（所以子步
 *       内部的过零只有子步精度）；反向恢复、振荡、器件电容均未建模。
 * @context [en] Host-side SIL only: never part of the shipped library.
 *           [zh] 仅 host 侧 SIL：绝不进入发布库。
 * @since 0.1.0
 */
class DiscreteInverter final {
public:
  explicit DiscreteInverter(const InverterConfig &config) noexcept
      : config_(config) {
    t_sw_ = (config_.f_sw_hz > 0.0F) ? (1.0F / config_.f_sw_hz) : 0.0F;
    sub_per_switch_ =
        (config_.sub_per_switch >= 2) ? config_.sub_per_switch : 2;
  }

  DiscreteInverter(const DiscreteInverter &) = delete;
  DiscreteInverter &operator=(const DiscreteInverter &) = delete;
  DiscreteInverter(DiscreteInverter &&) = delete;
  DiscreteInverter &operator=(DiscreteInverter &&) = delete;

  /// [en] True when the configuration can produce a switching period at all.
  /// [zh] 配置至少能产生一个开关周期。
  [[nodiscard]] auto valid() const noexcept -> bool { return t_sw_ > 0.0F; }

  [[nodiscard]] auto switching_period_s() const noexcept -> float {
    return t_sw_;
  }

  [[nodiscard]] auto deadtime_s() const noexcept -> float {
    return config_.t_dead_s;
  }

  /// [en] Rail-to-rail voltage loss the dead time injects per phase,
  /// `(t_dead/T)*v_bus`, V. This is the
  ///      number the *verification* predicts; the model never uses it.
  ///      [zh]
  ///      死区每相注入的电压损失，`(t_dead/T)·v_bus`，V。这是**验证**要预测的数；模型自己不用它。
  [[nodiscard]] auto deadtime_voltage(float v_bus) const noexcept -> float {
    return (t_sw_ > 0.0F) ? ((config_.t_dead_s / t_sw_) * v_bus) : 0.0F;
  }

  /**
   * @brief [en] Latch one control period: the duties, the bus, and how many
   * sub-steps it is worth. [zh]
   * 锁存一个控制周期：占空比、母线电压，以及它值多少个子步。
   *
   * @param duty [en] per-phase duty from the modulator. Clamped to [0, 1];
   * out-of-range is reported by `clamped()`. [zh] 调制器给出的每相占空比。钳到
   * [0, 1]；越界由 `clamped()` 报告。
   * @param v_bus [en] DC link voltage in V. [zh] 母线电压，V。
   * @param dt_ctrl_s [en] control period in s; must be a whole number of
   * switching periods, otherwise the request is refused (0 sub-steps) rather
   * than silently mis-timed. [zh]
   * 控制周期，s；必须是开关周期的整数倍，否则请求被拒绝（0 子步）而不是被悄悄
   *                  错配时间。
   * @return [en] number of sub-steps to run, 0 if the request was refused. [zh]
   * 要跑的子步数；请求被 拒绝时为 0。
   * @since 0.1.0
   */
  [[nodiscard]] auto begin_period(Uvw<float> duty, float v_bus,
                                  float dt_ctrl_s) noexcept -> std::int32_t {
    clamped_ = false;
    if (!valid() || !(v_bus > 0.0F) || !(dt_ctrl_s > 0.0F)) {
      steps_ = 0;
      return 0;
    }

    const float switches = dt_ctrl_s / t_sw_;
    const float rounded =
        static_cast<float>(static_cast<std::int32_t>(switches + 0.5F));
    if (!(rounded >= 1.0F) || !(std::fabs(switches - rounded) < 1.0e-3F)) {
      steps_ = 0;
      return 0;
    }

    duty_ = clamp_duty(duty);
    v_bus_ = v_bus;
    steps_ = static_cast<std::int32_t>(rounded) * sub_per_switch_;
    step_length_ = dt_ctrl_s / static_cast<float>(steps_);
    time_in_period_ = 0.0F;
    return steps_;
  }

  /// [en] Number of sub-steps the latched period asked for. [zh]
  /// 已锁存周期要求的子步数。
  [[nodiscard]] auto steps() const noexcept -> std::int32_t { return steps_; }

  /// [en] Sub-step length in s. [zh] 子步长度，s。
  [[nodiscard]] auto step_length_s() const noexcept -> float {
    return step_length_;
  }

  /// [en] True if the request had to be clamped into [0, 1]. [zh]
  /// 请求是否被钳进 [0, 1]。
  [[nodiscard]] auto clamped() const noexcept -> bool { return clamped_; }

  /// [en] The duties actually used (after clamping). [zh]
  /// 实际使用的占空比（钳位之后）。
  [[nodiscard]] auto duty() const noexcept -> Uvw<float> { return duty_; }

  /**
   * @brief [en] The stationary-frame voltage for the next sub-step, given the
   * currents at its start. [zh] 下一个子步的静止系电压，电流取其起点处的值。
   *
   * @param i_uvw [en] phase currents at the start of the sub-step, A; only
   * their **sign** is used, and only during dead-time windows. [zh]
   * 子步起点处的相电流，A；只用到它们的**符号**， 且只在死区窗口内。
   * @return [en] `clarke` of the three pole voltages, V. [zh] 三相极电压的
   * `clarke`，V。
   * @since 0.1.0
   */
  [[nodiscard]] auto step(const Uvw<float> &i_uvw) noexcept -> Ab<float> {
    if (steps_ <= 0) {
      return Ab<float>{};
    }

    const float window = step_length_;
    const float from = time_in_period_;
    const float to = from + window;

    const float pole_u = pole_voltage(duty_.u, i_uvw.u, from, to);
    const float pole_v = pole_voltage(duty_.v, i_uvw.v, from, to);
    const float pole_w = pole_voltage(duty_.w, i_uvw.w, from, to);

    time_in_period_ += window;
    while (time_in_period_ >= t_sw_) {
      time_in_period_ -= t_sw_;
    }
    return core::clarke(Uvw<float>{pole_u, pole_v, pole_w});
  }

  /// [en] Back to the power-on state. [zh] 回到上电状态。
  void reset() noexcept {
    duty_ = Uvw<float>{};
    time_in_period_ = 0.0F;
    steps_ = 0;
    clamped_ = false;
  }

private:
  [[nodiscard]] auto clamp_duty(Uvw<float> duty) noexcept -> Uvw<float> {
    const auto one = [this](float d) noexcept -> float {
      if (!(d >= 0.0F)) {
        clamped_ = true;
        return 0.0F;
      }
      if (d > 1.0F) {
        clamped_ = true;
        return 1.0F;
      }
      return d;
    };
    return Uvw<float>{one(duty.u), one(duty.v), one(duty.w)};
  }

  /// [en] Length of the intersection of `[from, to)` with `[a, b)`, both taken
  /// modulo T.
  ///      [zh] `[from, to)` 与 `[a, b)` 的交集长度，两者都按 T 取模。
  [[nodiscard]] auto overlap(float a, float b, float from,
                             float to) const noexcept -> float {
    const float span = to - from;
    if (!(span > 0.0F)) {
      return 0.0F;
    }
    const auto mod = [this](float t) noexcept -> float {
      while (t < 0.0F) {
        t += t_sw_;
      }
      while (t >= t_sw_) {
        t -= t_sw_;
      }
      return t;
    };
    // Walk the sub-step in at most two pieces so a window straddling T still
    // counts. 把子步切成至多两段，跨 T 的窗口因此仍被计入。
    float total = 0.0F;
    float cursor = from;
    float remaining = span;
    while (remaining > 0.0F) {
      const float piece =
          (mod(cursor) + remaining > t_sw_) ? (t_sw_ - mod(cursor)) : remaining;
      const float lo = mod(cursor);
      const float hi = lo + piece;
      const float start = mod(a);
      const float end = start + (b - a);
      const float left = (lo > start) ? lo : start;
      const float right = (hi < end) ? hi : end;
      if (right > left) {
        total += (right - left);
      }
      cursor += piece;
      remaining -= piece;
    }
    return total;
  }

  /// [en] Average pole voltage over one sub-step for one leg. [zh]
  /// 一条桥臂在一个子步内的平均极电压。
  [[nodiscard]] auto pole_voltage(float duty, float current, float from,
                                  float to) const noexcept -> float {
    const float period = t_sw_;
    const float dead = config_.t_dead_s;
    const float half_low = 0.5F * (1.0F - duty) * period;  // commanded turn-on
    const float half_high = 0.5F * (1.0F + duty) * period; // commanded turn-off
    const float span = to - from;

    // Turn-off is immediate at `half_high`; turn-on waits `dead` after
    // `half_low`. What is left between those two windows is the high-side
    // conducting time. 关断在 `half_high` 处立即生效；开通要等 `half_low` 之后
    // `dead`。两个死区窗口之间的部分就是高侧 导通时间。
    const float high_from = half_low + dead;
    const float high_to = half_high;
    const float rise_from = half_low;
    const float rise_to = half_low + dead;
    const float fall_from = half_high;
    const float fall_to = half_high + dead;

    const float f_high = (high_to > high_from)
                             ? (overlap(high_from, high_to, from, to) / span)
                             : 0.0F;
    const float f_dead = (dead > 0.0F)
                             ? ((overlap(rise_from, rise_to, from, to) +
                                 overlap(fall_from, fall_to, from, to)) /
                                span)
                             : 0.0F;
    const float f_low = 1.0F - f_high - f_dead;

    // While both devices are off the phase is clamped to whichever rail its
    // current is coming from: positive current freewheels through the lower
    // diode, negative through the upper one. This single line is the entire
    // dead-time effect.
    // 两只器件都关断时，该相被钳位在电流来自的那条母线上：正电流经下管二极管续流，负电流经上管。
    // 死区的全部效应就是这一行。
    const float v_high = v_bus_ - config_.v_fwd_v;
    const float v_low = config_.v_fwd_v;
    const float v_dead_window = (current > 0.0F) ? v_low : v_high;

    return ((f_high * v_high) + (f_low * v_low)) + (f_dead * v_dead_window);
  }

  InverterConfig config_{};
  float t_sw_{0.0F};
  std::int32_t sub_per_switch_{2};
  Uvw<float> duty_{};
  float v_bus_{0.0F};
  std::int32_t steps_{0};
  float step_length_{0.0F};
  float time_in_period_{0.0F};
  bool clamped_{false};
};

} // namespace sil

} // namespace foc
