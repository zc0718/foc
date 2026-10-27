// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] One thing on the board that can be wrong, independent of the
 * others. [zh] 板子上可能出错的一件事，与其它项独立。
 * @since 0.1.0
 * @exporter
 */
enum class BoardItem : std::uint8_t {
    CurrentSum,      ///< [en] `i_u + i_v + i_w` under a steady drive; KCL says
                     ///< zero. [zh] 稳态驱动下 `i_u + i_v + i_w`；基尔霍夫说它
                     ///< 是零。
    CurrentOffset,   ///< [en] what a channel reads with the bridge off. [zh]
                     ///< 桥关断时通道读数。
    CurrentPolarity, ///< [en] the sign of the response to a known drive. [zh]
                     ///< 对已知驱动的响应符号。
    SamplingInstant, ///< [en] measured switching ripple against the predicted
                     ///< one. [zh] 实测开关纹波对预测纹波。
    DeadTime,        ///< [en] identified dead-time volts against the board's
                     ///< setting. [zh] 辨识出的死区伏秒对板子的设定。
    EncoderOffset,   ///< [en] the mount offset left after alignment. [zh]
                     ///< 对齐之后残留的安装偏置。
    Parameters,      ///< [en] `Rs`, `psi_f`, `L` against the nameplate. [zh]
                     ///< `Rs`、`ψf`、`L` 对铭牌。
    ObserverHealth,  ///< [en] observer error against a known trajectory. [zh]
                     ///< 对已知轨迹的观测器误差。
};

/**
 * @brief [en] The one thing that most likely needs fixing. [zh]
 * 最可能需要修的那一件事。
 *
 *        [en] The list is a *diagnosis*, not a list of failures: "two items
 * failed" is not actionable, "the sampling instant is wrong" is. The mapping
 * from failure pattern to cause is the whole value of this file -- see
 * `check_board()`. [zh]
 * 这份清单是**诊断**，不是失败列表："两项失败了"没法处理，"采样时刻错了"可以。从失败模式到
 * 成因的映射就是本文件的全部价值——见 `check_board()`。
 * @since 0.1.0
 * @exporter
 */
enum class BoardCause : std::uint8_t {
    None = 0,
    OffsetBias,        ///< [en] a channel's zero is off. [zh] 某通道的零点偏了。
    PolarityReversed,  ///< [en] a channel's sign is inverted. [zh]
                       ///< 某通道的符号反了。
    SamplingInstant,   ///< [en] the ADC samples somewhere other than the centre
                       ///< of the zero vector. [zh] ADC 采样点不在零矢量中心。
    DeadTimeMismatch,  ///< [en] the board's dead time is not what it was told.
                       ///< [zh] 板子的死区不是它被告知的值。
    EncoderOffset,     ///< [en] the encoder mount offset was not applied. [zh]
                       ///< 编码器安装偏置没有被用上。
    ParametersOff,     ///< [en] `Rs`/`psi_f`/`L` disagree with the machine. [zh]
                       ///< `Rs`/`ψf`/`L` 与机器不符。
    ObserverUnhealthy, ///< [en] the observer's angle is wrong on a trajectory it
                       ///< should track. [zh] 在本该跟上的轨迹上角度不对。
};

/**
 * @brief [en] Everything the checks measure, in one place. Every field is a
 * **number**, which is the point: the report has to say "0.041 A" and not
 * "fail". [zh]
 * 检查量到的全部东西，放在一处。每个字段都是一个**数**，这正是要点：报告要说"0.041 A"，
 * 不是"失败"。
 * @since 0.1.0
 * @exporter
 */
struct BoardMeasurements final {
    float sum_current_a;      ///< [en] mean `i_u+i_v+i_w` under a steady drive, A.
                              ///< [zh] 稳态驱动下 `i_u+i_v+i_w` 的均值，A。
    float offset_a[3];        ///< [en] zero-current reading per phase, A. [zh]
                              ///< 每相的零电流读数，A。
    float response_sign[3];   ///< [en] sign of each phase's response to a positive
                              ///< d-axis drive, +1/-1/+2 = unknown. [zh] 正 d 轴
                              ///< 驱动下各相响应的符号，+1/−1/+2 = 未知。
    float ripple_measured_a;  ///< [en] peak-to-peak ripple the ADC actually
                              ///< sees, A. [zh] ADC 实际看到的纹波峰峰值，A。
    float ripple_expected_a;  ///< [en] what the switching period implies, A.
                              ///< [zh] 开关周期应有的值，A。
    float deadtime_v;         ///< [en] identified dead-time volts (axis
                              ///< projected), V. [zh] 辨识出的死区伏秒（轴上投
                              ///< 影），V。
    float deadtime_cmd_v;     ///< [en] what the board was configured with, V.
                              ///< [zh] 板子配置的值，V。
    float encoder_offset_rad; ///< [en] mount offset left after alignment, rad.
                              ///< [zh] 对齐后残留的安装偏置，rad。
    float rs_ohm;             ///< [en] identified resistance, ohm. [zh]
                              ///< 辨识出的电阻，Ω。
    float rs_nameplate_ohm;   ///< [en] nameplate resistance, ohm. [zh]
                              ///< 铭牌电阻，Ω。
    float psi_f_wb;           ///< [en] identified flux linkage, Wb. [zh]
                              ///< 辨识出的磁链，Wb。
    float psi_f_nameplate_wb; ///< [en] nameplate flux linkage, Wb. [zh]
                              ///< 铭牌磁链，Wb。
    float observer_error_rad; ///< [en] observer angle error on a known
                              ///< trajectory, rad. [zh] 已知轨迹上的观测器角度
                              ///< 误差，rad。
};

/**
 * @brief [en] The tolerances, separate from the measurements so a report can be
 * re-judged without re-measuring. [zh]
 * 容差与测量值分开，于是同一份测量可以在不重测的情况下重新判定。
 * @since 0.1.0
 * @exporter
 */
struct BoardLimits final {
    float sum_current_a{0.05F};      ///< [en] |sum| allowed, A. [zh]
                                     ///< 允许的 |和|，A。
    float offset_a{0.02F};           ///< [en] |zero reading| allowed, A. [zh]
                                     ///< 允许的 |零读数|，A。
    float ripple_ratio{1.5F};        ///< [en] measured/expected ripple ceiling.
                                     ///< [zh] 实测/预期纹波的上限。
    float deadtime_ratio{0.25F};     ///< [en] dead-time relative error allowed.
                                     ///< [zh] 允许的死区相对误差。
    float encoder_offset_rad{0.02F}; ///< [en] residual mount offset allowed,
                                     ///< rad. [zh] 允许的残余安装偏置，rad。
    float parameter_ratio{0.10F};    ///< [en] `Rs`/`psi_f` relative error allowed.
                                     ///< [zh] `Rs`/`ψf` 允许的相对误差。
    float observer_error_rad{0.10F}; ///< [en] observer error allowed, rad. [zh]
                                     ///< 允许的观测器误差，rad。
};

/**
 * @brief [en] One check's outcome: what was measured, against what, and the
 * verdict. [zh]
 * 一项检查的结果：量到了什么、对着什么比、以及判定。
 * @since 0.1.0
 * @exporter
 */
struct BoardReading final {
    BoardItem item;     ///< [en] which check. [zh] 哪一项。
    float value;        ///< [en] the measured number. [zh] 量到的数。
    float limit;        ///< [en] the number it was compared with. [zh] 比较用的数。
    bool pass;          ///< [en] the verdict. [zh] 判定。
    std::uint8_t phase; ///< [en] 0..2 when the item is per-phase, 3 otherwise.
                        ///< [zh] 逐相项为 0..2，否则 3。
};

/**
 * @brief [en] What the user gets: every number, and one cause to act on. [zh]
 * 用户拿到的东西：每个数，以及一个可以动手的成因。
 * @since 0.1.0
 * @exporter
 */
struct BoardReport final {
    static constexpr int kMaxItems = 12;
    BoardReading items[kMaxItems]{};    ///< [en] one row per check. [zh]
                                        ///< 每项一行。
    int count{0};                       ///< [en] rows filled. [zh] 已填行数。
    int failures{0};                    ///< [en] rows that failed. [zh]
                                        ///< 失败行数。
    BoardCause cause{BoardCause::None}; ///< [en] the one to fix first. [zh]
                                        ///< 最先该修的那个。
    std::uint8_t cause_phase{3};        ///< [en] which phase the cause names, or 3.
                                        ///< [zh] 成因指的是哪一相，或 3。

    /// [en] Everything passed. [zh] 全部通过。
    [[nodiscard]] auto all_green() const noexcept -> bool { return failures == 0; }
};

namespace detail {
/// [en] Relative error, safe at zero. [zh] 相对误差，零处安全。
[[nodiscard]] inline auto rel_error(float measured, float nominal) noexcept -> float {
    const float scale = std::fabs(nominal);
    if (!(scale > 1.0e-9F)) {
        return std::fabs(measured);
    }
    return std::fabs(measured - nominal) / scale;
}
} // namespace detail

/**
 * @brief [en] Runs the checks and names the most likely single cause. [zh]
 * 跑完检查，并指出最可能的那个单一成因。
 *
 *        [en] Why a cause and not a list. A board that is mis-wired usually
 * fails **several** items at once, and the items are not independent: an
 * inverted current channel makes the three-wire sum non-zero *and* the
 * dead-time identification come out negative *and* the flux identification
 * reverse. Reporting three failures sends the user to three places; the
 * intended behaviour is to report the one thing that explains the most of them.
 * The scoring below is exactly that: each candidate cause counts how many of the
 * failed items it accounts for, and the winner is reported. [zh]
 * 为什么报一个成因而不是一张表。接错的板子通常**同时**弄坏好几项，而这些项并不独立：一条反
 * 相的电流通道会让三线之和不为零、让死区辨识出负值、让磁链辨识反号。报三条失败会把用户支到
 * 三个地方；该做的是报那个"解释得最多"的一件。下面的打分就是这个：每个候选成因数一数它解释
 * 了几条失败，胜者上报。
 *
 * @param m [en] what was measured. [zh] 量到了什么。
 * @param limits [en] the tolerances. [zh] 容差。
 * @return [en] every number plus the ranked cause. [zh] 每个数，加上排名后的成因。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto check_board(const BoardMeasurements& m, const BoardLimits& limits) noexcept -> BoardReport {
    BoardReport report{};
    auto add = [&report](BoardItem item, float value, float limit, bool pass, std::uint8_t phase) {
        if (report.count >= BoardReport::kMaxItems) {
            return;
        }
        report.items[report.count] = BoardReading{item, value, limit, pass, phase};
        ++report.count;
        report.failures += pass ? 0 : 1;
    };

    // 1. Kirchhoff. The true phase currents sum to zero, so a steady non-zero
    // sum is either an offset or a channel that has been negated -- a negation
    // turns `+i, -i/2, -i/2` into `-i, -i/2, -i/2`, whose sum is `-2i`, so the
    // sum *grows* rather than cancelling.
    // 1. 基尔霍夫。真实相电流之和为零，所以稳态下和不为零要么是零漂、要么是一条通道被取了反——
    // 取反把 `+i, −i/2, −i/2` 变成 `−i, −i/2, −i/2`，其和是 `−2i`，于是和是**变大**而不是抵消。
    const bool sum_ok = std::fabs(m.sum_current_a) <= limits.sum_current_a;
    add(BoardItem::CurrentSum, m.sum_current_a, limits.sum_current_a, sum_ok, 3);

    // 2. Per-phase zero reading and response sign, which are what make the sum
    // fail in the first place.
    // 2. 逐相零读数与响应符号，它们才是让"和"失败的根源。
    bool any_offset = false;
    bool any_flip = false;
    std::uint8_t first_offset = 3;
    std::uint8_t first_flip = 3;
    for (std::uint8_t phase = 0; phase < 3; ++phase) {
        const float offset = m.offset_a[phase];
        const bool ok = std::fabs(offset) <= limits.offset_a;
        add(BoardItem::CurrentOffset, offset, limits.offset_a, ok, phase);
        if (!ok) {
            any_offset = true;
            first_offset = (first_offset == 3) ? phase : first_offset;
        }
        // A positive d-axis drive puts `+i` on u and `-i/2` on v and w.
        // 正 d 轴驱动在 u 上给 `+i`，在 v、w 上给 `−i/2`。
        const float expected = (phase == 0) ? 1.0F : -1.0F;
        const float measured = m.response_sign[phase];
        const bool sign_ok = std::fabs(measured) < 1.5F && ((measured * expected) > 0.0F);
        add(BoardItem::CurrentPolarity, measured, expected, sign_ok, phase);
        if (!sign_ok) {
            any_flip = true;
            first_flip = (first_flip == 3) ? phase : first_flip;
        }
    }

    // 3. Sampling instant. At the centre of the zero vector the ADC sees a
    // current that is *at* its average; off-centre it sees the ripple, and the
    // measured peak-to-peak therefore exceeds what the switching period
    // predicts. The ratio is the number; there is no absolute threshold that
    // would work across bus voltages and inductances.
    // 3. 采样时刻。零矢量中心处 ADC 看到的是电流的均值；偏离中心就落在纹波上，于是实测峰峰值
    // 超过开关周期预测的值。比值才是那个数；任何绝对阈值都跨不过母线电压与电感的变化。
    const float ratio = (m.ripple_expected_a > 1.0e-9F) ? (m.ripple_measured_a / m.ripple_expected_a) : 0.0F;
    const bool ripple_ok = ratio <= limits.ripple_ratio;
    add(BoardItem::SamplingInstant, ratio, limits.ripple_ratio, ripple_ok, 3);

    // 4. Dead time: the board knows what it was configured with, and the
    // identification knows what it actually is.
    // 4. 死区：板子知道自己被配成了多少，辨识知道它实际是多少。
    const bool deadtime_ok = detail::rel_error(m.deadtime_v, m.deadtime_cmd_v) <= limits.deadtime_ratio;
    add(BoardItem::DeadTime, m.deadtime_v, m.deadtime_cmd_v, deadtime_ok, 3);

    // 5. Encoder: after alignment the residual mount offset should be nothing.
    // 5. 编码器：对齐之后残余安装偏置应该为零。
    const bool encoder_ok = std::fabs(m.encoder_offset_rad) <= limits.encoder_offset_rad;
    add(BoardItem::EncoderOffset, m.encoder_offset_rad, limits.encoder_offset_rad, encoder_ok, 3);

    // 6. Parameters against the nameplate.
    // 6. 参数对铭牌。
    const bool rs_ok = detail::rel_error(m.rs_ohm, m.rs_nameplate_ohm) <= limits.parameter_ratio;
    add(BoardItem::Parameters, m.rs_ohm, m.rs_nameplate_ohm, rs_ok, 3);
    const bool flux_ok = detail::rel_error(m.psi_f_wb, m.psi_f_nameplate_wb) <= limits.parameter_ratio;
    add(BoardItem::Parameters, m.psi_f_wb, m.psi_f_nameplate_wb, flux_ok, 3);

    // 7. The observer on a trajectory it is supposed to track.
    // 7. 观测器在本该跟上的轨迹上。
    const bool observer_ok = m.observer_error_rad <= limits.observer_error_rad;
    add(BoardItem::ObserverHealth, m.observer_error_rad, limits.observer_error_rad, observer_ok, 3);

    // --- diagnosis: which single cause explains the most failures? ---
    // --- 诊断：哪一个单一成因解释得最多？---
    struct Candidate final {
        BoardCause cause;
        int score;
    };
    Candidate best{BoardCause::None, 0};
    auto consider = [&best](BoardCause cause, int score) {
        if (score > best.score) {
            best = Candidate{cause, score};
        }
    };
    // A flipped channel explains the sum *and* its own row, and (see the header
    // of this file) it is also what makes a dead-time identification run
    // negative -- so it is the strongest single explanation available.
    // 一条反相通道解释了"和"和它自己那一行，并且（见本文件开头）它也是让死区辨识出负值的
    // 原因——所以它是可用的最强单一解释。
    consider(BoardCause::PolarityReversed, (any_flip ? 2 : 0) + (sum_ok ? 0 : 1) + (deadtime_ok ? 0 : 1));
    consider(BoardCause::OffsetBias, (any_offset ? 2 : 0) + (sum_ok ? 0 : 1));
    consider(BoardCause::SamplingInstant, (ripple_ok ? 0 : 2) + (deadtime_ok ? 0 : 1));
    consider(BoardCause::DeadTimeMismatch, deadtime_ok ? 0 : 2);
    consider(BoardCause::EncoderOffset, encoder_ok ? 0 : 2);
    consider(BoardCause::ParametersOff, (rs_ok ? 0 : 1) + (flux_ok ? 0 : 1));
    consider(BoardCause::ObserverUnhealthy, observer_ok ? 0 : 2);

    report.cause = best.cause;
    switch (best.cause) {
    case BoardCause::PolarityReversed:
        report.cause_phase = first_flip;
        break;
    case BoardCause::OffsetBias:
        report.cause_phase = first_offset;
        break;
    default:
        report.cause_phase = 3;
        break;
    }
    return report;
}

/**
 * @brief [en] The cause as text, for a log line or a screen. [zh]
 * 成因的文本形式，给日志或界面用。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] constexpr auto board_cause_name(BoardCause cause) noexcept -> const char* {
    switch (cause) {
    case BoardCause::None:
        return "none";
    case BoardCause::OffsetBias:
        return "current offset";
    case BoardCause::PolarityReversed:
        return "current polarity";
    case BoardCause::SamplingInstant:
        return "sampling instant";
    case BoardCause::DeadTimeMismatch:
        return "dead time";
    case BoardCause::EncoderOffset:
        return "encoder offset";
    case BoardCause::ParametersOff:
        return "machine parameters";
    case BoardCause::ObserverUnhealthy:
        return "observer";
    }
    return "?";
}

} // namespace core

} // namespace foc
