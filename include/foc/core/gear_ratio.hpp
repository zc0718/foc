// Conan::ImportStart
#pragma once

#include <cmath>
#include <cstdint>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


namespace {

/**
 * @brief [en] Fold an angle into `[-pi, pi)`.
 * @brief [zh] 把角度折进 `[-pi, pi)`。
 */
inline auto wrap_pi(float angle_rad) noexcept -> float {
    constexpr float kPiRad = 3.14159265358979323846F;
    constexpr float kTwoPiRad = 6.28318530717958647692F;
    float wrapped = std::fmod(angle_rad + kPiRad, kTwoPiRad);
    if (wrapped < 0.0F) {
        wrapped += kTwoPiRad;
    }
    return wrapped - kPiRad;
}

} // namespace


/**
 * @brief [en] How a slave axis follows a master: `slave = ratio * master + offset_rad`.
 * @brief [zh] 从轴如何跟随主轴：`从轴 = 传动比 × 主轴 + offset_rad`。
 * @note [en] `ratio` is negative for a reversing gear and must not be zero -- a zero ratio is a coupling that silently
 *            holds still while the master moves, which is indistinguishable from a slave that is stuck.
 * @note [zh] 换向齿轮的 `ratio` 为负，且不得为零——零传动比是一个主轴运动时悄悄不动的耦合，与一根卡住的从轴无法区分。
 * @since 0.1.10
 */
struct GearConfig final {
    float ratio;      ///< [en] slave radians per master radian, non-zero. [zh] 每主轴弧度的从轴弧度，非零。
    float offset_rad; ///< [en] slave position when the master reads zero, rad. [zh] 主轴读数为零时的从轴位置，rad。
};


/**
 * @brief [en] What the gear asked for this period.
 * @brief [zh] 本周期齿轮要求的东西。
 * @since 0.1.10
 * @exporter
 */
struct GearOutput final {
    /**
     * @brief [en] `Ok` normally, `Param` when the ratio is unusable or the sample is not finite. The first sample is
     *             `Ok`: it defines the origin, so the answer is well defined rather than missing.
     * @brief [zh] 通常为 `Ok`；传动比不可用或采样非有限时为 `Param`。第一个采样也是 `Ok`：它定义原点，所以答案是有定义
     *             的而不是缺失的。
     */
    ErrorCode status;
    float slave_rad;            ///< [en] the slave target, rad. [zh] 从轴目标，rad。
    float master_unwrapped_rad; ///< [en] the master angle with the wrap removed, rad. [zh] 去掉卷绕后的主轴角度，rad。
    /**
     * @brief [en] the slave's velocity reference, rad/s -- the master's own speed times the ratio.
     * @brief [zh] 从轴的速度参考，rad/s——主轴自身转速乘传动比。
     * @note [en] Not a convenience. A position loop told only where to be lags a moving target by `v * kd / kp`, which
     *            on the SIL pair below is **58 mrad of synchronisation error** while the axes are perfectly geared;
     *            feeding this number forward as `+ kd * v_ref` cancels that term exactly for a constant speed. A gear
     *            without a velocity output is a lag generator with a ratio.
     * @note [zh] 不是便利项。只被告知"该在哪"的位置环跟随一个移动目标时滞后 `v · kd / kp`，在下面那对 SIL 轴上就是 **58
     *            mrad 的同步误差**，而两轴完全按齿比同步；把这个数作为 `+ kd · v_ref` 前馈进去，对恒速情形正好抵消该
     *            项。一个没有速度输出的齿轮，就是一台带传动比的滞后发生器。
     */
    float slave_velocity_rad_s;
};


/**
 * @brief [en] Electronic gearing with the master's wrapping removed, so a slave does not lose a revolution per turn.
 *
 * @brief [zh] 电子齿轮，并去掉主轴的卷绕，使从轴不会每次旋转丢掉一圈。
 *
 *     The whole difficulty is in one word: *unwrapped*. An encoder hands over an angle in `[0, 2*pi)`, so a master that
 *     turns three times reads 0, then 2*pi, then 0 again -- and a gear that multiplies what it is given hands the slave
 *     a reference that walks backwards twice per three revolutions. The fix is to accumulate the shortest arc between
 *     consecutive samples, which is exact for any sampling rate that resolves the master's motion (the same condition
 *     every observer in this library states), and which costs one state variable.
 *
 *     Why it is a separate stage rather than a line in the caller: the unwrapped master is also the *synchronisation
 *     error's* reference. Measuring the slave against a wrapped master is how a perfectly synchronised pair reports a
 *     6.28 rad error once a revolution, and the measurement is the point of the capability.
 *
 *     全部难点在一个词里：**去卷绕**。编码器给出的角度在 `[0, 2π)` 内，所以转三圈的主轴读出 0、2π、0——而一个直接乘下去
 *     的齿轮会给从轴一个每三圈倒退两次的参考。修法是对相邻采样之间的最短弧做累加，只要采样率能分辨主轴的运动就精确（这
 *     正是本库每个观测器都声明的同一个条件），代价是一个状态变量。
 *
 *     为什么它是独立一级而不是调用方的一行：去卷绕后的主轴同时也是**同步误差**的参照。拿卷绕的主轴去量从轴，正是一对完
 *     全同步的轴每转一圈报出 6.28 rad 误差的方式，而那个测量正是这条能力的要点。
 * @since 0.1.10
 * @exporter
 */
class ElectronicGear final {
  public:
    ElectronicGear() noexcept = default;
    explicit ElectronicGear(const GearConfig& config) noexcept : config_(config) {}

    /**
     * @brief [en] Replace the gear. The master reference is kept: changing a ratio does not move the master.
     * @brief [zh] 换齿轮。主轴的参照保留：改传动比并不会移动主轴。
     */
    void set_config(const GearConfig& config) noexcept { config_ = config; }

    [[nodiscard]] auto config() const noexcept -> const GearConfig& { return config_; }

    /**
     * @brief [en] `Ok` when the ratio is finite and non-zero, and the offset is finite.
     * @brief [zh] 传动比有限且非零、偏置有限时为 `Ok`。
     */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode {
        if (!std::isfinite(config_.ratio) || !std::isfinite(config_.offset_rad)) {
            return ErrorCode::Param;
        }
        return (config_.ratio != 0.0F) ? ErrorCode::Ok : ErrorCode::Param;
    }

    /**
     * @brief [en] Forget the unwrap state. Call it after anything that makes the master's history meaningless -- a
     *             re-home, a power cycle, a bus reset -- because the next sample is then taken as the new origin.
     * @brief [zh] 忘掉去卷绕状态。在任何让主轴历史失去意义的事情之后调用——重新回零、断电重启、总线复位——因为下一个采样
     *             会被当作新的原点。
     */
    void reset() noexcept {
        started_ = false;
        unwrapped_rad_ = 0.0F;
        last_wrapped_rad_ = 0.0F;
    }

    /**
     * @brief [en] One master sample in, one slave target out.
     * @brief [zh] 一个主轴采样进，一个从轴目标出。
     * @param master_wrapped_rad [en] the master angle as the encoder reports it, rad. @param master_wrapped_rad
     * @param master_wrapped_rad [zh] 编码器报告的主轴角度，rad。
     * @param dt_s [en] the sample interval, s; `<= 0` leaves the velocity reference at zero and does not affect the
     *                  position. @param dt_s
     * @param dt_s [zh] 采样间隔，s；`<= 0` 时速度参考为零，位置不受影响。
     * @note [en] The first sample defines the origin, so `master_unwrapped_rad` starts at zero and the slave starts at
     *            `offset_rad`. That is a choice, and the alternative -- believing the first reading is an absolute
     *            position -- is the same mistake as believing a first interval is a sector period.
     * @note [zh] 第一个采样定义原点，因此 `master_unwrapped_rad` 从零开始、从轴从 `offset_rad` 开始。这是一个选择，而另
     *            一种做法——相信第一次读数就是绝对位置——与相信"第一段间隔是一个扇区周期"是同一种错误。
     * @since 0.1.10
     */
    [[nodiscard]] auto apply(float master_wrapped_rad, float dt_s) noexcept -> GearOutput {
        GearOutput out{validate(), 0.0F, 0.0F, 0.0F};
        if (out.status != ErrorCode::Ok) {
            return out;
        }
        if (!std::isfinite(master_wrapped_rad)) {
            out.status = ErrorCode::Param;
            return out;
        }
        if (!started_) {
            started_ = true;
            last_wrapped_rad_ = master_wrapped_rad;
            unwrapped_rad_ = 0.0F;
        } else {
            const float delta = wrap_pi(master_wrapped_rad - last_wrapped_rad_);
            unwrapped_rad_ += delta;
            last_wrapped_rad_ = master_wrapped_rad;
            if (dt_s > 0.0F) {
                const float master_velocity = delta / dt_s;
                out.slave_velocity_rad_s = config_.ratio * master_velocity;
            }
        }
        out.master_unwrapped_rad = unwrapped_rad_;
        out.slave_rad = (config_.ratio * unwrapped_rad_) + config_.offset_rad;
        out.status = ErrorCode::Ok;
        return out;
    }

    /**
     * @brief [en] The unwrapped master angle as of the last sample, rad.
     * @brief [zh] 最近一次采样时的去卷绕主轴角度，rad。
     * @since 0.1.10
     */
    [[nodiscard]] auto master_unwrapped_rad() const noexcept -> float { return unwrapped_rad_; }

    /**
     * @brief [en] The largest jump between two consecutive master samples that counts as motion rather than a jump,
     *             rad.
     * @brief [zh] 相邻两个主轴采样之间被视为运动而不是跳变的跳变上限，rad。
     * @note [en] The shortest arc assumes the master moves less than half a turn per sample. Every observer in this
     *            library states the same assumption; here it is written down because the gear has no other way to tell
     *            a fast reversal from a wrap.
     * @note [zh] 最短弧假设主轴每个采样移动不到半圈。本库每个观测器都声明同一个假设；这里写下来是因为齿轮没有别的办法区
     *            分一次快速反向和一次卷绕。
     */
    static constexpr float kMaxStepRad = 3.14159265358979323846F;

  private:
    GearConfig config_{};
    float unwrapped_rad_{0.0F};
    float last_wrapped_rad_{0.0F};
    bool started_{false};
};


} // namespace core

} // namespace foc
