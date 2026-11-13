// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file sensor_monitor.hpp
 *
 * @brief [en] Whether the position sensor's own readings hold together, judged without a second opinion.
 * @brief [zh] 位置传感器自己的读数是否自洽——不用第二个意见来判。
 * @since 0.1.0
 */

namespace foc {

namespace core {


/**
 * @brief [en] What the monitor decided about this tick's reading.
 * @brief [zh] 监视器对这一拍读数的判决。
 * @note [en] `Unobserved` is not `Ok`. A drive whose sensor has never disagreed because nobody compared anything would
 *            otherwise read as healthy, which is the same failure `fault_input` refuses at the port: silence is not
 *            consent.
 * @note [zh] `Unobserved` 不是 `Ok`。否则一台"从来没人比过、因此从不分歧"的驱动就会读作健康，而这正是 `fault_input` 在
 *            端口上拒绝的那种失败：沉默不是同意。
 * @since 0.1.0
 * @exporter
 */
enum class SensorVerdict : std::uint8_t {
    /**
     * @brief [en] the reading is consistent with the last one and with the configured motion.
     * @brief [zh] 读数与上一次一致，也与配置的运动范围一致。
     */
    Ok = 0,
    /**
     * @brief [en] it is not: something moved further than the mechanism can, or faster than it can.
     * @brief [zh] 不一致：某处移动得比机构能有的更远，或更快。
     */
    Implausible = 1,
    /**
     * @brief [en] this tick has nothing to judge: a first sample, a refused period, a reading that is not a number.
     * @brief [zh] 这一拍没有可判的东西：首个采样、被拒绝的周期、不是数的读数。
     */
    Unobserved = 2,
};

/**
 * @brief [en] Which check failed, kept separate because the two point at different causes.
 * @brief [zh] 是哪一条检查没过——分开保留，因为两者指向不同的成因。
 * @note [en] A jump between two samples is wiring, a lost frame or a slipping coupling; a speed the mechanism cannot
 *            produce is a pulse train losing counts or a scale that no longer matches the encoder. Collapsing them into
 *            "bad sensor" would send a fitter to check both ends of the same cable twice.
 * @note [zh] 两个采样之间的跳变是接线、丢帧或联轴器打滑；机构产生不出来的转速是脉冲串在丢计数，或分度已与编码器不符。把
 *            它们并成"传感器坏了"，会让现场的人把同一根线的两头各查一遍。
 * @since 0.1.0
 * @exporter
 */
enum class SensorFault : std::uint8_t {
    /**
     * @brief [en] nothing failed; also what an `Unobserved` tick reports.
     * @brief [zh] 没有失败； `Unobserved` 的那一拍报的也是它。
     */
    None = 0,
    /**
     * @brief [en] the step between two samples exceeded what `max_speed * dt` allows.
     * @brief [zh] 两个采样之间的步长超过了 `max_speed * dt` 允许的量。
     */
    AngleJump = 1,
    /**
     * @brief [en] the reported speed is one the mechanism cannot produce.
     * @brief [zh] 报出的转速是机构产生不出来的。
     */
    ImplausibleSpeed = 2,
    /**
     * @brief [en] the reading is not a finite number.
     * @brief [zh] 读数不是有限的数。
     */
    NotANumber = 3,
};

/**
 * @brief [en] What this drive is allowed to do, so that "implausible" is derived rather than picked.
 * @brief [zh] 这台驱动被允许做什么，好让"不合常理"是推出来的而不是挑出来的。
 * @since 0.1.0
 * @exporter
 */
struct SensorMonitorConfig final {
    /**
     * @brief [en] the fastest the mechanism itself can move, rad/s; the step bound is this times the period.
     * @brief [zh] 机构本身能达到的最快速度，rad/s；步长界就是这个数乘以周期。
     */
    float max_speed_rad_s;
    /**
     * @brief [en] what `max_speed * dt` may be exceeded by before it is called a jump, rad.
     * @brief [zh] `max_speed * dt` 允许被超过多少才算跳变，rad。
     * @note [en] Not a fudge factor for rounding: it is where the caller states how much a real encoder may move in one
     *            sample beyond the mechanism's ceiling -- quantisation, a filter's overshoot, a coupling's compliance.
     *            Zero is legal and means "the ceiling is exact".
     * @note [zh] 不是给舍入留的余量：调用方在这里说明真实编码器在一个采样里能越过机构上限多少——量化、滤波器过冲、联轴器
     *            柔性。取零是合法的，意思是"上限是精确的"。
     */
    float jump_tolerance_rad;
};

/**
 * @brief [en] The judgement, with the numbers it was made on.
 * @brief [zh] 判决，连同据以判决的那些数。
 * @since 0.1.0
 * @exporter
 */
struct SensorMonitorOutput final {
    /**
     * @brief [en] what this tick's reading is worth.
     * @brief [zh] 这一拍的读数算什么。
     */
    SensorVerdict verdict;
    /**
     * @brief [en] which check failed, `None` when nothing did.
     * @brief [zh] 是哪一条检查没过，都没过时是 `None`。
     */
    SensorFault fault;
    /**
     * @brief [en] this tick's angle change, wrapped into `(-pi, pi]`, rad.
     * @brief [zh] 这一拍的角度变化，归一到 `(-pi, pi]`，rad。
     */
    float step_rad;
    /**
     * @brief [en] what the configuration allowed this tick, rad: `max_speed * dt + tolerance`.
     * @brief [zh] 配置允许这一拍走多远，rad： `max_speed * dt + tolerance`。
     */
    float bound_rad;
    /**
     * @brief [en] implausible readings since the last `reset()`.
     * @brief [zh] 自上次 `reset()` 以来不合常理的读数个数。
     * @note [en] A count rather than a latch: one glitch in a million ticks and one every hundred are different
     *            machines, and which of them is worth stopping for is the application's decision. The host stages the
     *            response; this class only refuses to lose the evidence.
     * @note [zh] 计数而不是锁存：百万拍里一次的毛刺与每一百拍一次是两台不同的机器，而对哪一种该停机是应用的判断。宿主负
     *            责分级响应；本类只负责不把证据弄丢。
     */
    std::uint32_t implausible_count;
};

/**
 * @brief [en] Does this encoder's own history hold together? Judged from the readings alone.
 * @brief [zh] 这个编码器自己的历史自洽吗？只用读数本身来判。
 * @note [en] The observer-disagreement monitor answers a different and narrower question: it needs a second estimator,
 *            and that estimator is only trustworthy above a speed this library has measured (`omega*psi_f ~ V_dead`,
 *            101 rad/s on the nominal machine). A servo spends its life below that, so the two are not alternatives --
 *            this one works at standstill and at speed, and it costs nothing beyond the subtraction. What it can see: a
 *            wire that comes loose, a frame that is dropped, a coupling that slips, a pulse train losing counts, a
 *            reading that stops being a number. What it cannot see: an encoder that lies *smoothly* -- a mounting
 *            offset that drifts, a calibration that was never right, a scale that is wrong by half a percent. Those
 *            move every sample by a plausible step, and only a second opinion can catch them. That limit is the reason
 *            the other monitor exists, not a reason to widen this one. The step is compared **wrapped**, so the
 *            ordinary crossing of the `2*pi` boundary is not a jump: it is the one place where a naive difference
 *            reports a full turn in one sample, and it is why `wrap` is applied before the comparison rather than after
 *            it.
 *
 * @note [zh] 观测器分歧监视回答的是另一个更窄的问题：它需要第二个估计器，而那个估计器只在这台库量过的转速以上可信（
 *            `omega*psi_f ~ V_dead`，标称机器上是 101 rad/s）。伺服的一生大多在那以下，所以两者不是备选关系——本类在静止
 *            和高速都可用，代价只有一次减法。
 *
 *   它看得见：松掉的线、丢掉的帧、打滑的联轴器、丢计数的脉冲串、不再是数的读数。它看不见： **平滑地** 撒谎的编码器——缓
 *   慢漂移的安装偏置、从来没对过的标定、差了半个百分点的分度。那些每一拍都只走一个看起来合理的步长，只有第二意见能抓
 *   住。这条限制是另一个监视器存在的理由，不是把本类放宽的理由。
 *
 *   步长按 **归一化后** 比较，所以普通地跨过 `2*pi` 边界不算跳变：那是朴素差分会在一拍里报出整整一圈的地方，也是 `wrap`
 *   用在比较 **之前** 而不是之后的原因。
 * @since 0.1.0
 * @exporter
 */
class SensorMonitor final {
  public:
    /**
     * @brief [en] Accept the two numbers, or refuse them before a single tick runs.
     * @brief [zh] 接受这两个数，或者在还没跑一拍之前就拒绝它们。
     * @return [en] `Ok`; `Param` for a ceiling that is not positive or not finite, or a tolerance that is negative or
     *              not finite. A tolerance of zero is accepted -- "the ceiling is exact" is a position, not a typo.
     * @return [zh] `Ok`；上限不为正或非有限、余量为负或非有限时返回 `Param`。余量为零是接受的——"上限是精确的"是一种立
     *              场，不是笔误。
     */
    auto configure(const SensorMonitorConfig& config) noexcept -> ErrorCode {
        const bool ceiling_ok = (config.max_speed_rad_s > 0.0F) && std::isfinite(config.max_speed_rad_s);
        const bool tolerance_ok = (config.jump_tolerance_rad >= 0.0F) && std::isfinite(config.jump_tolerance_rad);
        if (!ceiling_ok || !tolerance_ok) {
            return ErrorCode::Param;
        }
        config_ = config;
        configured_ = true;
        reset();
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] Judge one reading.
     * @brief [zh] 判一个读数。
     * @param angle_mech_rad [en] the mechanical angle this tick, rad, wrapped into `[0, 2*pi)` by the sensor.
     * @param angle_mech_rad [zh] 本拍机械角度，rad，由传感器归一到 `[0, 2*pi)`。
     * @param speed_mech_rad_s [en] the speed the same sensor reported, rad/s; `NaN` when the sensor has no speed
     *                              channel, which then simply skips that check.
     * @param speed_mech_rad_s [zh] 同一传感器报的转速，rad/s；传感器没有转速通道时为 `NaN`，此时那条检查直接跳过。
     * @param dt_s [en] the period, s; `<= 0` or non-finite measures nothing: the sample is not adopted, the count is
     *                  untouched, the step and the bound report zero, and the answer is `Unobserved`.
     * @param dt_s [zh] 周期，s； `<= 0` 或非有限时什么都没量到：采样不被采纳、计数不动、步长与界报零，答 `Unobserved`。
     * @return [en] the judgement, with the step it measured and the bound it measured it against.
     * @return [zh] 判决，连同它量到的步长与据以比较的界。
     * @post [en] The first sample after `configure()`/`reset()` answers `Unobserved`: with nothing to compare against,
     *            "consistent" would be a claim about zero data.
     * @post [zh] `configure()`/`reset()` 之后的第一拍答 `Unobserved`：没有可比的东西时，"一致"是关于零条数据的断言。
     */
    auto step(float angle_mech_rad, float speed_mech_rad_s, float dt_s) noexcept -> const SensorMonitorOutput& {
        output_.fault = SensorFault::None;
        output_.step_rad = 0.0F;
        output_.bound_rad = 0.0F;
        if (!configured_ || !(dt_s > 0.0F) || !std::isfinite(dt_s)) {
            output_.verdict = SensorVerdict::Unobserved;
            return output_;
        }
        if (!std::isfinite(angle_mech_rad)) {
            // A reading that is not a number *is* information: the sensor said something impossible, which is not the
            // same as saying nothing. The previous sample stands, so the next good reading is compared against the last
            // good one rather than against a value that was never an angle.
            // 不是数的读数**是**信息：传感器说了不可能的东西，这与什么都没说不是一回事。上一个采样保留，于是下一个好读数
            // 是与上一个好读数比较，而不是与一个从来不是角度的值比较。
            output_.verdict = SensorVerdict::Implausible;
            output_.fault = SensorFault::NotANumber;
            ++output_.implausible_count;
            return output_;
        }
        output_.bound_rad = (config_.max_speed_rad_s * dt_s) + config_.jump_tolerance_rad;
        output_.step_rad = have_previous_ ? wrap(angle_mech_rad - previous_rad_) : 0.0F;
        const bool first = !have_previous_;
        previous_rad_ = angle_mech_rad;
        have_previous_ = true;
        if (first) {
            // Nothing to compare against yet. Reporting `Ok` here would be a verdict about zero samples.
            // 还没有可比的东西。此时报 `Ok` 等于对零个采样下判决。
            output_.verdict = SensorVerdict::Unobserved;
            return output_;
        }
        if (std::fabs(output_.step_rad) > output_.bound_rad) {
            output_.verdict = SensorVerdict::Implausible;
            output_.fault = SensorFault::AngleJump;
            ++output_.implausible_count;
            return output_;
        }
        if (std::isfinite(speed_mech_rad_s) && (std::fabs(speed_mech_rad_s) > config_.max_speed_rad_s)) {
            output_.verdict = SensorVerdict::Implausible;
            output_.fault = SensorFault::ImplausibleSpeed;
            ++output_.implausible_count;
            return output_;
        }
        output_.verdict = SensorVerdict::Ok;
        return output_;
    }

    /**
     * @brief [en] The last judgement.
     * @brief [zh] 最近一次的判决。
     */
    [[nodiscard]] auto output() const noexcept -> const SensorMonitorOutput& { return output_; }

    /**
     * @brief [en] Forget the previous sample and the count: the next tick is a first tick again.
     * @brief [zh] 忘掉上一个采样与计数：下一拍又是第一拍。
     * @note [en] Use it when the axis was moved by something this sensor did not see -- a hand, a second controller, a
     *            mechanical re-clamp -- because the step across that move is legitimately larger than the ceiling and
     *            the alternative is a monitor that reports a fault for a correct reading.
     * @note [zh] 轴被本传感器没看见的东西动过时用它——一只手、另一个控制器、一次机械重新装夹：跨越那次移动的步长合理地大
     *            于上限，否则监视器就会为一条正确的读数报故障。
     */
    void reset() noexcept {
        have_previous_ = false;
        previous_rad_ = 0.0F;
        output_ = SensorMonitorOutput{SensorVerdict::Unobserved, SensorFault::None, 0.0F, 0.0F, 0U};
    }

    [[nodiscard]] auto configured() const noexcept -> bool { return configured_; }

  private:
    [[nodiscard]] static auto wrap(float delta_rad) noexcept -> float { return std::remainder(delta_rad, kTwoPiRad); }

    static constexpr float kTwoPiRad = 6.28318530717958647692F;

    SensorMonitorConfig config_{0.0F, 0.0F};
    SensorMonitorOutput output_{SensorVerdict::Unobserved, SensorFault::None, 0.0F, 0.0F, 0U};
    float previous_rad_{0.0F};
    bool have_previous_{false};
    bool configured_{false};
};

} // namespace core

} // namespace foc
