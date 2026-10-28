// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] What went wrong, one code per physical mechanism.
 *
 *     Why an enum and not `ErrorCode::Fault`. A single latched fault code is enough to stop a bench test and useless in
 *     a robot: "the drive tripped" tells an operator to send the unit back, while "the DC link went under 18 V" tells
 *     them the supply sagged. The codes are listed in the order the protector ranks them, so a step that trips two
 *     things reports the outer one -- `OverCurrent` before `I2t` before `Stall`, because the current is what breaks
 *     hardware first.
 *
 * @brief [zh] 出了什么事，每种物理机理一个码。
 *
 *     为什么要枚举而不是只用 `ErrorCode::Fault`。一个锁存故障码够停下一次台架测试，在机器人里 没用："驱动器跳了"只能让
 *     运维把机器寄回厂；"母线掉到 18 V 以下"告诉他供电塌了。 这些码按保护器的排序列出，所以一步里同时触发两件事时报最外
 *     层那个——`OverCurrent` 先于 `I2t` 先于 `Stall`，因为先坏硬件的总是电流。
 * @since 0.1.0
 * @exporter
 */
enum class FaultCode : std::uint8_t {
    None = 0,
    OverCurrent,     ///< [en] |i| above the hard limit. [zh] |i| 超过硬限。
    OverVoltage,     ///< [en] DC link above the limit. [zh] 母线超过上限。
    UnderVoltage,    ///< [en] DC link below the limit. [zh] 母线低于下限。
    OverTemperature, ///< [en] winding or heatsink above the limit. [zh]
                     ///< 绕组或散热器超过上限。
    I2t,             ///< [en] the thermal budget is spent. [zh] 热预算用完了。
    Stall,           ///< [en] current without motion for too long. [zh]
                     ///< 有电流却没有运动，且持续太久。
    LostSync,        ///< [en] the observer has no usable angle. [zh]
                     ///< 观测器给不出可用的角度。
};

/**
 * @brief [en] Whether a fault clears by itself once its cause goes away.
 *
 *     The split is physical, not a matter of taste. A bus that sagged lifts again, a winding that overheated cools, and
 *     an `I2t` budget refills when the current stops -- those clear themselves, and latching them would turn a
 *     transient into a service call. A hard over-current, a stall and a lost angle mean something is **wrong** (a
 *     shorted winding, a jammed axis, a mis-set offset): retrying those automatically is how a drive destroys itself in
 *     a loop. They stay latched until somebody calls `reset()`.
 *
 * @brief [zh] 故障在成因消失后是否自行消失。
 *
 *     这个分法是物理的，不是口味问题。塌下去的母线会自己回来、过热的绕组会凉、`I2t` 预算在电流 停下后自己回填——它们自己
 *     会清，锁存它们等于把一次暂态变成一次报修。硬过流、堵转、失步意味着 有东西**坏了**（绕组短路、轴卡死、偏置设错）：
 *     自动重试正是驱动器把自己反复烧掉的方式。 它们保持锁存，直到有人 `reset()`。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] constexpr auto fault_is_latched(FaultCode code) noexcept -> bool {
    switch (code) {
    case FaultCode::OverCurrent:
    case FaultCode::Stall:
    case FaultCode::LostSync:
        return true;
    default:
        return false;
    }
}

/**
 * @brief [en] Everything the protector looks at, in one place.
 * @brief [zh] 保护器看的全部东西，放在一处。
 * @since 0.1.0
 * @exporter
 */
struct ProtectionInput final {
    float i_magnitude_a;    ///< [en] peak phase current, A. [zh] 相电流峰值，A。
    float v_bus_v;          ///< [en] DC link, V. [zh] 母线电压，V。
    float temperature_c;    ///< [en] hottest measured point, deg C. [zh]
                            ///< 测到的最热点，℃。
    float omega_e_rad_s;    ///< [en] electrical speed, rad/s. [zh]
                            ///< 电角速度，rad/s。
    float flux_error_ratio; ///< [en] |psi_est - psi_f| / psi_f, 0 = healthy.
                            ///< [zh] |ψ估计 − ψf|/ψf，0 = 正常。
    bool flux_valid;        ///< [en] the observer has an answer at all. [zh]
                            ///< 观测器到底给不给得出答案。
};

/**
 * @brief [en] Thresholds and the one time constant that is a physical property of the machine.
 * @brief [zh] 阈值，以及那个属于机器物理属性时间常数。
 * @since 0.1.0
 * @exporter
 */
struct ProtectionConfig final {
    float i_hard_a;          ///< [en] instantaneous trip, A. [zh] 瞬时跳闸，A。
    float i_max_a;           ///< [en] nominal continuous limit before derating, A.
                             ///< [zh] 降额前的额定连续限幅，A。
    float i2t_a2s;           ///< [en] thermal budget, A^2*s. [zh] 热预算，A²·s。
    float i2t_cooling_s;     ///< [en] the budget's decay time constant, s; this is
                             ///< the winding's thermal time constant in the squared
                             ///< current, not a tuning number. [zh] 预算的衰减时间常数，s；
                             ///< 它是绕组在电流平方上的热时间常数，不是整定值。
    float v_max_v;           ///< [en] DC link upper limit, V. [zh] 母线上限，V。
    float v_min_v;           ///< [en] DC link lower limit, V. [zh] 母线下限，V。
    float t_trip_c;          ///< [en] temperature that trips, deg C. [zh]
                             ///< 跳闸温度，℃。
    float t_derate_start_c;  ///< [en] where derating begins, deg C. [zh]
                             ///< 开始降额的温度，℃。
    float t_derate_end_c;    ///< [en] where the limit reaches zero, deg C. [zh]
                             ///< 限幅降到零的温度，℃。
    float stall_speed_rad_s; ///< [en] below this the axis is not moving, rad/s.
                             ///< [zh] 低于此值算没有运动，rad/s。
    float stall_current_a;   ///< [en] above this the axis is pushing, A. [zh]
                             ///< 高于此值算在使劲，A。
    float stall_time_s;      ///< [en] and for this long before it is a stall, s.
                             ///< [zh] 持续这么久才算堵转，s。
    float sync_error_ratio;  ///< [en] flux error that counts as lost, 1 = 100 %.
                             ///< [zh] 算失步的磁链误差，1 = 100 %。
    float hysteresis_ratio;  ///< [en] an auto fault clears only inside
                             ///< `(1-h)*threshold`, e.g. 0.05. [zh]
                             ///< 自恢复故障只在 `(1−h)·阈值` 以内才清，例如 0.05。
};

/**
 * @brief [en] What the protector says to do this step.
 * @brief [zh] 本步保护器给出的处置。
 * @since 0.1.0
 * @exporter
 */
struct ProtectionOutput final {
    FaultCode fault;            ///< [en] the highest-ranked active fault. [zh]
                                ///< 排名最高的活动故障。
    std::uint16_t active_mask;  ///< [en] bit `1 << code` for each active fault.
                                ///< [zh] 每个活动故障一位 `1 << code`。
    std::uint16_t latched_mask; ///< [en] the same, for faults that will not
                                ///< clear on their own. [zh]
                                ///< 同上，但只含不会自行清除的故障。
    float i_limit_a;            ///< [en] the derated current limit, 0 when tripped.
                                ///< [zh] 降额后的电流限幅，跳闸时为 0。
    bool output_allowed;        ///< [en] false = stop switching this step. [zh]
                                ///< false = 本步停止开关。
};

/**
 * @brief [en] Classifies six faults, latches what should stay latched, and derates before it trips.
 *
 *     Three things it does that a plain threshold check does not. (1) **Derating before tripping**: the current limit
 *     falls monotonically from `t_derate_start_c` to `t_derate_end_c`, so a hot machine loses authority gradually
 *     instead of failing at a cliff edge. (2) **`I2t` as a thermal model, not a counter**: `q' = i^2 - q/tau`, so the
 *     budget *cools* while the current is off. A counter that only increments would make a machine that has been idle
 *     for an hour still carry the memory of a two-second overload. (3) **Hysteresis on exactly the faults that
 *     self-clear**: a bus hovering on the threshold would otherwise chatter the output enable at the control rate, and
 *     chatter is how a protection becomes a failure source.
 *
 *     What it is **not**, said here because "thermal model" invites the wrong reading. `q` is in A^2*s, not degrees:
 *     the block has no idea what the winding temperature is, and its `t_derate_*` thresholds are a *number the caller
 *     supplies*, not a measurement this code can make. It models one lumped copper loss with one time constant. It does
 *     not model the magnet, the bearings, the ambient, or a hot spot away from whatever sensor the caller has. A real
 *     drive needs a thermistor or an estimator alongside this, and no amount of tuning here replaces that.
 *
 * @brief [zh] 分类六类故障、锁存该锁存的、并在跳闸之前先降额。
 *
 *     它做三件单纯阈值检查不会做的事。（1）**跳闸之前先降额**：电流限幅从 `t_derate_start_c` 单调降到
 *     `t_derate_end_c`，热的机器逐渐失去权限，而不是在悬崖边 上突然失效。（2）**`I2t` 是热模型不是计数器**：`q' = i² −
 *     q/τ`，所以电流停下时预算会 **冷却**。只增不减的计数器会让闲置一小时的机器仍然背着两秒过载的记忆。（3）**只有自恢
 *     复 的那些才加迟滞**：母线悬在阈值上时，否则输出使能会按控制频率反复抖，而抖动正是把一个 保护变成故障源的方式。
 *
 *     它**不是**什么，写在这里因为"热模型"容易招来误读。`q` 的单位是 A²·s，不是度： 这段代码不知道绕组温度是多少，它的
 *     `t_derate_*` 阈值是**调用方给的数**，不是本代码能 测出来的量。它用一个时间常数建模一个集总的铜损。它不建模磁钢、
 *     不建模轴承、不建模环境温度、 也不建模远离调用方那个传感器的热点。真实的驱动器需要热敏电阻或温度估计器与它并存，这
 *     里 怎么调都替代不了那件事。
 *
 * @context [en] Current-loop ISR, every step.
 * @context [zh] 电流环 ISR，每步。
 * @since 0.1.0
 * @exporter
 */
class Protector final {
  public:
    explicit Protector(const ProtectionConfig& config) noexcept : config_(config) {}

    /**
     * @brief [en] Advance one control period and say what to do.
     * @brief [zh] 推进一个控制周期并给出处置。
     * @param input [en] the measurements of this step.
     * @param input [zh] 本步的测量值。
     * @param dt_s [en] the period, s.
     * @param dt_s [zh] 周期，s。
     * @return [en] the ranked fault, the masks, the derated limit and whether output is allowed.
     * @return [zh] 排名后的故障、掩码、降额限幅，以及是否允许输出。
     * @note [en] The decision is taken **in the same step as the measurement**, so `output_allowed == false` is
     *            available before the next PWM update. A protection that answers one step late is a protection that
     *            lets one more period of over-current through, and at 20 kHz that is the difference between a trip and
     *            a destroyed bridge.
     * @note [zh] 判定与测量在**同一步**完成，所以 `output_allowed == false` 在下一次 PWM 更新之前就可用。 晚一步作答的
     *            保护，等于再放过一个周期的过流；20 kHz 下那就是"跳闸"与"炸桥"的差别。
     * @since 0.1.0
     */
    auto step(const ProtectionInput& input, float dt_s) noexcept -> ProtectionOutput {
        // The thermal model runs whether or not anything tripped: heat does not
        // pause because a threshold was crossed.
        // 热模型一直在跑，不管有没有跳闸：热量不会因为越了阈值就停下。
        if (dt_s > 0.0F) {
            const float cooled = (config_.i2t_cooling_s > 0.0F) ? (q_ * (dt_s / config_.i2t_cooling_s)) : 0.0F;
            q_ = static_cast<float>(q_ + ((input.i_magnitude_a * input.i_magnitude_a * dt_s) - cooled));
            q_ = (q_ > 0.0F) ? q_ : 0.0F;
        }

        // Stall is a *duration*, so it needs a timer rather than a threshold.
        // 堵转是一个**持续时长**，所以它需要计时器而不是阈值。
        const bool pushing = (std::fabs(input.i_magnitude_a) > config_.stall_current_a) &&
                             (std::fabs(input.omega_e_rad_s) < config_.stall_speed_rad_s);
        if (pushing) {
            stall_s_ = static_cast<float>(stall_s_ + (dt_s > 0.0F ? dt_s : 0.0F));
        } else {
            stall_s_ = 0.0F;
        }

        // Evaluate every mechanism, independent of the others, so the matrix in
        // the test is a matrix and not a chain of `else if`s.
        // 每个机理独立判定，这样测试里的矩阵是矩阵，而不是一串 `else if`。
        const float h = (config_.hysteresis_ratio > 0.0F) ? config_.hysteresis_ratio : 0.0F;
        const bool hot = keep(FaultCode::OverTemperature, tripped_, std::fabs(input.temperature_c) > config_.t_trip_c,
                              std::fabs(input.temperature_c) < (config_.t_trip_c * (1.0F - h)));
        const bool over_v = keep(FaultCode::OverVoltage, tripped_, std::fabs(input.v_bus_v) > config_.v_max_v,
                                 std::fabs(input.v_bus_v) < (config_.v_max_v * (1.0F - h)));
        const bool under_v = keep(FaultCode::UnderVoltage, tripped_, std::fabs(input.v_bus_v) < config_.v_min_v,
                                  std::fabs(input.v_bus_v) > (config_.v_min_v * (1.0F + h)));
        const bool spent = keep(FaultCode::I2t, tripped_, q_ > config_.i2t_a2s, q_ < (config_.i2t_a2s * (1.0F - h)));
        // The latched ones have no clearing condition: `keep()` with `clear`
        // false keeps them set until `reset()`.
        // 锁存的那些没有清除条件：`keep()` 的 `clear` 传 false，它们一直保持到 `reset()`。
        const bool over_i =
            keep(FaultCode::OverCurrent, tripped_, std::fabs(input.i_magnitude_a) > config_.i_hard_a, false);
        const bool stalled = keep(FaultCode::Stall, tripped_, stall_s_ > config_.stall_time_s, false);
        const bool lost =
            keep(FaultCode::LostSync, tripped_,
                 (!input.flux_valid) || (std::fabs(input.flux_error_ratio) > config_.sync_error_ratio), false);

        const bool flags[8] = {false, over_i, over_v, under_v, hot, spent, stalled, lost};
        ProtectionOutput out{};
        out.active_mask = 0;
        out.latched_mask = 0;
        out.fault = FaultCode::None;
        // Ranked by the enum order, so "current before heat before motion".
        // 按枚举顺序排名，"电流先于热度先于运动"。
        for (std::uint8_t code = 1; code < 8; ++code) {
            if (!flags[code]) {
                continue;
            }
            const auto as_code = static_cast<FaultCode>(code);
            out.active_mask = static_cast<std::uint16_t>(out.active_mask | (1U << code));
            if (fault_is_latched(as_code)) {
                out.latched_mask = static_cast<std::uint16_t>(out.latched_mask | (1U << code));
            }
            if (out.fault == FaultCode::None) {
                out.fault = as_code;
            }
        }

        // A tripped step has no authority at all, so the limit and the enable
        // are the same decision stated twice -- which is deliberate: a caller
        // that only reads `i_limit_a` must not be able to keep pushing.
        // 跳闸的这一步没有任何权限，所以限幅和使能是同一个决定的两种说法——这是故意的：
        // 只读 `i_limit_a` 的调用方也不能继续使劲。
        out.output_allowed = (out.active_mask == 0U);
        out.i_limit_a = out.output_allowed ? derated_limit(input.temperature_c) : 0.0F;
        return out;
    }

    /**
     * @brief [en] The current limit as a function of temperature.
     * @brief [zh] 电流限幅随温度的函数。
     * @note [en] Monotone by construction: flat, then one straight segment to zero, then zero. Not a table, because a
     *            table invites interpolation choices and the damping between two table points is exactly where a
     *            machine oscillates.
     * @note [zh] 构造上就单调：先平，再一段直线到零，然后为零。不用表，因为表会引出插值选择，而两个表点之间 正是机器开
     *            始振荡的地方。
     * @since 0.1.0
     */
    [[nodiscard]] auto derated_limit(float temperature_c) const noexcept -> float {
        if (!(config_.i_max_a > 0.0F)) {
            return 0.0F;
        }
        if (temperature_c <= config_.t_derate_start_c) {
            return config_.i_max_a;
        }
        const float span = config_.t_derate_end_c - config_.t_derate_start_c;
        if (!(span > 0.0F)) {
            return 0.0F;
        }
        const float used = (temperature_c - config_.t_derate_start_c) / span;
        if (used >= 1.0F) {
            return 0.0F;
        }
        return config_.i_max_a * (1.0F - used);
    }

    /// [en] The thermal budget, A^2*s. [zh] 热预算，A²·s。
    [[nodiscard]] auto thermal_budget() const noexcept -> float { return q_; }

    /// [en] How long the axis has been pushing without moving, s. [zh]
    /// 轴有劲没动的时间，s。
    [[nodiscard]] auto stall_duration() const noexcept -> float { return stall_s_; }

    /**
     * @brief [en] Clear the latched faults. An operator action, never automatic.
     * @brief [zh] 清除锁存故障。这是运维动作，永不自作主张。
     * @note [en] `I2t` is *not* cleared here: the winding is still hot, and pretending otherwise is how the same
     *            overload happens twice.
     * @note [zh] 这里**不**清 `I2t`：绕组还热着，假装不热正是同一次过载发生两遍的方式。
     * @since 0.1.0
     */
    void reset() noexcept {
        for (std::uint8_t code = 1; code < 8; ++code) {
            if (fault_is_latched(static_cast<FaultCode>(code))) {
                tripped_[code] = false;
            }
        }
        stall_s_ = 0.0F;
    }

    /// [en] Clear everything including the thermal memory. [zh]
    /// 清掉一切，包括热记忆。
    void reset_cold() noexcept {
        reset();
        for (int k = 0; k < 8; ++k) {
            tripped_[k] = false;
        }
        q_ = 0.0F;
        stall_s_ = 0.0F;
    }

  private:
    /// [en] Set-and-hold with hysteresis, in one place so every fault behaves
    /// the same way. [zh] 带迟滞的置位/保持，集中在一处，让每个故障行为一致。
    auto keep(FaultCode code, bool (&state)[8], bool condition, bool clear_condition) noexcept -> bool {
        auto index = static_cast<std::uint8_t>(code);
        if (condition) {
            state[index] = true;
        } else if (clear_condition) {
            state[index] = false;
        }
        return state[index];
    }

    ProtectionConfig config_{};
    bool tripped_[8]{};
    float q_{0.0F};
    float stall_s_{0.0F};
};

} // namespace core

} // namespace foc
