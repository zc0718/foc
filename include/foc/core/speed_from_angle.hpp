// Conan::ImportStart
#pragma once
#include <cstdint>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Mechanical speed from an angle, by differencing it across a fixed delay.
 *
 * @brief [zh] 由角度求机械转速：跨固定延迟做差分。
 *
 *     Why it exists: an observer's own `omega_e` is a loop state -- a PLL locked to a demodulated carrier -- and at low
 *     speed that state is poor even when the *angle* is excellent. Measured on the injection observer at 20 rad/s
 *     mechanical: the angle is off by 0.0015 rad while its reported speed is off by 0.7995 rad/s, and a fifteen-point
 *     sweep of the loop's bandwidth and damping could only improve that by 2.75x. Differencing the angle instead asks a
 *     different question of the same signal, and the noise it carries is predictable from the angle's own error:
 *     `sigma_v = sigma_theta * sqrt(2) / (window_ticks * dt)`. 存在的理由：观测器自己的 `omega_e` 是一个**环路状态**——
 *     锁在解调载波上的 PLL——而低速时即便**角度**极好，这个状态也很差。在注入观测器上实测机械 20 rad/s：角度只差 0.0015
 *     rad，而它报出的转速差到 0.7995 rad/s，且对该环路的带宽与阻尼做十五点扫描也只能改善 2.75 倍。改为对角度做差分，是
 *     对同一个信号问另一个问题，而它带的噪声可以由角度自身的误差预测：`σ_v = σ_θ·√2/(window_ticks·dt)`。
 *
 * @note [en] Deliberately the cheapest form that can be measured: a delayed difference rather than a least-squares
 *            slope. The slope over the same window would be `sqrt(12/N)` times quieter, which is a factor of 4.1 at N =
 *            200 -- worth having, and not worth building before the cheap form has shown that differencing the angle is
 *            the right direction at all. The output is 0 until the window has filled. A speed that starts by reporting
 *            the difference across a partly-filled window would report a number divided by the wrong interval, which is
 *            worse than reporting nothing.
 * @note [zh] 刻意做成**能被测量**的最便宜形式：延迟差分，而不是最小二乘斜率。同一窗口上的斜率会比它安静 `√(12/N)` 倍，
 *            在 N = 200 时是 4.1 倍——值得有，但在便宜形式证明"对角度做差分"方向正确之前不值得先建。窗口填满之前输出为
 *            0。一开始就用半满窗口报差分，等于把一个数除以错误的区间——那比什么都不报更糟。
 * @since 0.1.6
 * @exporter
 */
class SpeedFromAngle final {
  public:
    /**
     * @brief [en] Window length and the machine's pole pairs.
     * @brief [zh] 窗口长度与极对数。
     * @since 0.1.6
     */
    struct Config final {
        std::uint16_t window_ticks{
            200U}; ///< [en] samples between the two differences, >= 2. [zh] 两次差分之间相隔的采样数，>= 2。
        std::uint8_t pole_pairs{1U}; ///< [en] pole pairs, >= 1. [zh] 极对数，>= 1。
    };

    /**
     * @brief The largest window; the buffer is fixed so that nothing here can allocate. 最大窗口；缓冲区是固定的，于是这里任何东西都不会分配。
     */
    static constexpr std::uint16_t kMaxWindowTicks = 512U;

    explicit SpeedFromAngle(Config config) noexcept;

    /**
     * @brief [en] Advance with one electrical angle sample.
     * @brief [zh] 用一个电角度采样推进。
     * @param theta_e_rad [en] electrical angle, any range.
     * @param theta_e_rad [zh] 电角度，任意范围。
     * @param dt_s [en] control period in s; a non-positive value is ignored.
     * @param dt_s [zh] 控制周期，s；非正值被忽略。
     * @context [en] Current-loop ISR: noexcept, no allocation, O(1).
     * @context [zh] 电流环 ISR：noexcept、不分配、O(1)。
     * @since 0.1.6
     */
    void update(float theta_e_rad, float dt_s) noexcept;

    /**
     * @brief [en] Mechanical speed in rad/s, 0 until `ready()`.
     * @brief [zh] 机械转速，rad/s；`ready()` 之前为 0。
     * @since 0.1.6
     */
    [[nodiscard]] auto mech_rad_s() const noexcept -> float { return ready_ ? speed_mech_rad_s_ : 0.0F; }

    /** @brief [en] Whether the window has filled at least once. [zh] 窗口是否已经填满过。 */
    [[nodiscard]] auto ready() const noexcept -> bool { return ready_; }

    /**
     * @brief [en] Whether the last update saw a step too large to unwrap.
     * @brief [zh] 上一次推进是否遇到无法解开折返的跳变。
     * @note [en] Same rule as `PositionReference`: an angle that jumped by half a turn between samples cannot be
     *            unwrapped, and guessing would produce a speed that looks plausible and is a whole turn out.
     * @note [zh] 与 `PositionReference` 同一条规矩：两次采样之间跳过半个电角度的角度无法解开折返，猜一个会给出一个看起
     *            来合理、却差了整整一圈的转速。
     * @since 0.1.6
     */
    [[nodiscard]] auto lost_count() const noexcept -> bool { return lost_count_; }

    /** @brief [en] Back to empty, keeping the configuration. [zh] 回到空状态，保留配置。 */
    void reset() noexcept;

  private:
    Config config_;
    float buffer_[kMaxWindowTicks]{};
    float continuous_rad_; ///< [en] unwrapped electrical angle. [zh] 解开折返后的电角度。
    float previous_rad_;   ///< [en] the previous raw sample. [zh] 上一个原始采样。
    float speed_mech_rad_s_;
    float last_dt_s_;
    std::uint16_t filled_;
    std::uint16_t write_index_;
    bool primed_;
    bool ready_;
    bool lost_count_;
};


} // namespace core

} // namespace foc
