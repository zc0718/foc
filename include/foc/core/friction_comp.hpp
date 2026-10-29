// Conan::ImportStart
#pragma once

#include <cmath>
#include <cstdint>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] The friction a drive is told to cancel, and the speeds that decide when it is at rest.
 * @brief [zh] 驱动被告知要抵消的摩擦，以及判定"静止"的速度。
 * @note [en] These are *estimates of a machine*, not tuning knobs: a value twice the truth drives the axis instead of
 *            cancelling it, and `validate()` refuses the one case it can see coming -- a Coulomb term larger than the
 *            breakaway point, which is a machine that cannot exist and would make the compensation a motor.
 * @note [zh] 这些是对**一台机器的估计**，不是整定旋钮：取真值的两倍就不再是抵消而是驱动，而 `validate()` 会拒绝它唯一能
 *            预见的那种情形——库仑项大于起动点，那是一台不可能存在的机器，也会让补偿变成一个电机。
 * @since 0.1.9
 */
struct FrictionConfig final {
    float
        coulomb_nm; ///< [en] kinetic friction to cancel while moving, N*m (>= 0). [zh] 运动时要抵消的库仑摩擦，N·m（>= 0）。
    float static_nm; ///< [en] breakaway estimate, N*m (>= `coulomb_nm`). [zh] 起动点估计，N·m（>= `coulomb_nm`）。
    float viscous_nm_per_rad_s; ///< [en] viscous coefficient, N*m*s/rad (>= 0). [zh] 粘性系数，N·m·s/rad（>= 0）。
    float
        rest_band_rad_s; ///< [en] |measured speed| at or below this counts as at rest, rad/s (>= 0). [zh] 实测转速绝对值不高于此即视为静止，rad/s（>= 0）。
};


/**
 * @brief [en] What the compensation adds this period, and what it noticed.
 * @brief [zh] 本周期补偿要加多少，以及它注意到了什么。
 * @since 0.1.9
 * @exporter
 */
struct FrictionOutput final {
    /**
     * @brief [en] `Ok` normally; `Param` when the configuration is unusable, in which case nothing is added.
     * @brief [zh] 正常为 `Ok`；配置不可用时为 `Param`，此时不加任何东西。
     */
    ErrorCode status;
    float torque_nm; ///< [en] torque to add to the loop's command, N*m. [zh] 要加到环路指令上的力矩，N·m。
    bool at_rest;    ///< [en] both speeds are inside the rest band. [zh] 两个速度都在静止带内。
    /**
     * @brief [en] At rest while being asked for more than the breakaway estimate: the axis is stalled, or the estimate
     *             is low. Reported rather than acted on -- which of the two it is cannot be told from here.
     * @brief [zh] 处于静止而被要求的力矩超过起动点估计：轴堵住了，或者估计偏低。只报告不动作——是哪一种这里看不出来。
     */
    bool stalled;
};


/**
 * @brief [en] Cancel the friction the loop has to fight, without becoming a motor.
 *
 * @brief [zh] 抵消环路不得不对抗的摩擦，同时不让自己变成电机。
 *
 *     Why this is a separate stage and not part of the current PI. Friction is a torque, and the PI regulates a
 *     current; adding a Coulomb term inside it would make the integrator's own state depend on which way the axis last
 *     moved. It does not fit the `FeedForward` slot either: that slot emits a *voltage* from `(omega_e, i_meas)` and
 *     never sees the reference direction, which is the only clean signal a Coulomb term has.
 *
 *     The law is two terms and a latch: `coulomb_nm * sign + viscous_nm_per_rad_s * omega_meas`. There is no `dt`,
 *     because nothing integrates -- and that is the point. A Coulomb term multiplying `sign(omega_meas)` chatters at
 *     rest, where the measured speed is zero-mean noise, and the chatter is a torque oscillation of amplitude `2 *
 *     coulomb_nm` that shakes the very thing the stage exists to steady. The latch is what removes it: the sign is
 *     updated from the reference when there is one, from the loop's demand when the demand is larger than the Coulomb
 *     term itself, and otherwise **kept**. The price is a stale direction when neither is informative, and it is stated
 *     rather than hidden.
 *
 *     Why it helps at all: at rest a proportional loop stops when `|kp * error| <= static`, so it leaves a standing
 *     error of `static / kp` and no amount of gain removes it (raising the gain only scales the same dead band down).
 *     With a latched Coulomb term the loop stops when `|kp * error + coulomb| <= static`, i.e. at `(static - coulomb) /
 *     kp` on the arrival side. Those two numbers are arithmetic, not tuning, and the case that ships with this header
 *     measures both of them on the SIL plant. 为什么这是独立一级而不是电流 PI 的一部分。摩擦是力矩，而 PI 调节的是电
 *     流；把库仑项塞进去会让积分器的状态取决于轴上一次往哪边动。它也不适配 `FeedForward` 槽：那个槽从 `(omega_e,
 *     i_meas)` 发出**电压**，永远看不到给定方向，而那正是库仑项唯一干净的信号。
 *
 *     规律是两项加一个锁存：`coulomb_nm * sign + viscous_nm_per_rad_s * omega_meas`。没有 `dt`，因为没有东西在积分 ——这
 *     正是要点。用 `sign(omega_meas)` 的库仑项在静止处会抖，那里的实测速度是零均值噪声，而抖动是幅度 `2 * coulomb_nm`
 *     的力矩振荡，恰好摇动这一级存在的目的。锁存去掉它：符号在有速度给定时取给定，在环路力矩指令大于库仑项本身时取指
 *     令，其余情况**保持**。代价是两者都无信息时方向可能是旧的，这个代价写出来而不是藏起来。
 *
 *     为什么它有用：静止时比例环在 `|kp * 误差| <= 静摩擦` 处停下，于是留下 `静摩擦 / kp` 的稳态误差，加增益也去不掉（
 *     只是把同一条死区按比例缩小）。有了锁存的库仑项，环路在 `|kp * 误差 + 库仑| <= 静摩擦` 处停下，即到达侧的 `(静摩擦
 *     − 库仑) / kp`。这两个数是算术、不是整定，而随这个头文件一起发布的用例在 SIL plant 上把两个都量了。
 * @since 0.1.9
 * @exporter
 */
class FrictionComp final {
  public:
    FrictionComp() noexcept = default;
    explicit FrictionComp(const FrictionConfig& config) noexcept : config_(config) {}

    /**
     * @brief [en] Replace the estimate. The latched direction survives: changing a friction estimate says nothing about
     *             which way the axis last moved, and restarting the latch would make the first period after a re-tune
     *             chatter.
     * @brief [zh] 换估计值。锁存的方向保留：改一个摩擦估计并不说明轴上次往哪边动，而重启锁存会让重新整定后的第一个周期
     *             抖动。
     */
    void set_config(const FrictionConfig& config) noexcept { config_ = config; }

    [[nodiscard]] auto config() const noexcept -> const FrictionConfig& { return config_; }

    /**
     * @brief [en] `Ok` when the estimate describes a real machine; `Param` otherwise.
     * @brief [zh] 估计值描述一台真实机器时为 `Ok`；否则 `Param`。
     * @note [en] The refusal that matters is `static_nm < coulomb_nm`: a machine whose kinetic friction exceeds its
     *            breakaway cannot be built, and a compensation built on such a pair would push the axis at rest rather
     *            than cancel friction -- it would be a motor with a negative damping coefficient.
     * @note [zh] 要紧的拒绝是 `static_nm < coulomb_nm`：动摩擦大于起动的机器造不出来，而建在这样一对数上的补偿会在静止
     *            时推轴而不是抵消摩擦——那就是一台阻尼系数为负的电机。
     */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode {
        if (!std::isfinite(config_.coulomb_nm) || !std::isfinite(config_.static_nm) ||
            !std::isfinite(config_.viscous_nm_per_rad_s) || !std::isfinite(config_.rest_band_rad_s)) {
            return ErrorCode::Param;
        }
        if (config_.coulomb_nm < 0.0F || config_.viscous_nm_per_rad_s < 0.0F || config_.rest_band_rad_s < 0.0F) {
            return ErrorCode::Param;
        }
        if (config_.static_nm < config_.coulomb_nm) {
            return ErrorCode::Param;
        }
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] Forget the latched direction (not the estimate): the next period takes its sign from the reference or
     *             the demand, or starts pushing the positive way.
     * @brief [zh] 忘掉锁存的方向（不忘记估计值）：下一个周期的符号取自给定或指令，否则从正向开始。
     */
    void reset() noexcept { sign_ = 1.0F; }

    /**
     * @brief [en] One control period: what torque to add.
     * @brief [zh] 一个控制周期：要加多少力矩。
     * @param speed_ref_rad_s [en] the reference speed if the loop has one, else 0. @param speed_ref_rad_s
     * @param speed_ref_rad_s [zh] 环路若有速度给定就传它，否则传 0。
     * @param speed_meas_rad_s [en] the measured mechanical speed. @param speed_meas_rad_s
     * @param speed_meas_rad_s [zh] 实测机械转速。
     * @param demand_nm [en] what the loop is asking for, before compensation. @param demand_nm
     * @param demand_nm [zh] 环路在补偿之前要求的力矩。
     * @since 0.1.9
     */
    [[nodiscard]] auto step(float speed_ref_rad_s, float speed_meas_rad_s, float demand_nm) noexcept -> FrictionOutput {
        FrictionOutput out{validate(), 0.0F, false, false};
        if (out.status != ErrorCode::Ok) {
            return out;
        }
        const float measured = std::fabs(speed_meas_rad_s);
        const float reference = std::fabs(speed_ref_rad_s);
        // The reference decides when there is one, the demand decides when it is larger than the term being applied,
        // and otherwise the latch holds. The middle rule exists because at rest the demand is the only quantity that
        // knows which side of the target the error is on; the last rule exists because near zero the demand's sign is
        // noise and a flipping sign is a torque square wave.
        // 有给定时由给定决定；指令大于正在施加的项时由指令决定；其余情况由锁存保持。中间那条存在，是因为静止时只有指令
        // 知道误差在目标的哪一侧；最后那条存在，是因为零附近的指令符号是噪声，而翻转的符号就是一条力矩方波。
        if (reference > 0.0F) {
            sign_ = (speed_ref_rad_s > 0.0F) ? 1.0F : -1.0F;
        } else if (std::fabs(demand_nm) > config_.coulomb_nm) {
            sign_ = (demand_nm > 0.0F) ? 1.0F : -1.0F;
        }

        out.at_rest = (reference <= 0.0F) && (measured <= config_.rest_band_rad_s);
        out.torque_nm = (config_.coulomb_nm * sign_) + (config_.viscous_nm_per_rad_s * speed_meas_rad_s);
        out.stalled = out.at_rest && (std::fabs(demand_nm) >= config_.static_nm);
        return out;
    }

    /**
     * @brief [en] The direction currently latched, -1 or +1.
     * @brief [zh] 当前锁存的方向，−1 或 +1。
     * @since 0.1.9
     */
    [[nodiscard]] auto sign() const noexcept -> float { return sign_; }

  private:
    FrictionConfig config_{};
    float sign_{1.0F};
};


} // namespace core

} // namespace foc
