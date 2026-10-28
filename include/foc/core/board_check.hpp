// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <type_traits>
#include <utility>

#include <foc/core/commissioning.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] One thing on the board that can be wrong, independent of the others.
 * @brief [zh] 板子上可能出错的一件事，与其它项独立。
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
 * @brief [en] The one thing that most likely needs fixing.
 *
 *     The list is a *diagnosis*, not a list of failures: "two items failed" is not actionable, "the sampling instant is
 *     wrong" is. The mapping from failure pattern to cause is the whole value of this file -- see `check_board()`.
 *
 * @brief [zh] 最可能需要修的那一件事。
 *
 *     这份清单是**诊断**，不是失败列表："两项失败了"没法处理，"采样时刻错了"可以。从失败模式到 成因的映射就是本文件的全
 *     部价值——见 `check_board()`。
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
 * @brief [en] Everything the checks measure, in one place. Every field is a **number**, which is the point: the report
 *             has to say "0.041 A" and not "fail".
 * @brief [zh] 检查量到的全部东西，放在一处。每个字段都是一个**数**，这正是要点：报告要说"0.041 A"， 不是"失败"。
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
 * @brief [en] The tolerances, separate from the measurements so a report can be re-judged without re-measuring.
 * @brief [zh] 容差与测量值分开，于是同一份测量可以在不重测的情况下重新判定。
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
 * @brief [en] One check's outcome: what was measured, against what, and the verdict.
 * @brief [zh] 一项检查的结果：量到了什么、对着什么比、以及判定。
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
 * @brief [en] What the user gets: every number, and one cause to act on.
 * @brief [zh] 用户拿到的东西：每个数，以及一个可以动手的成因。
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

/// [en] Does the bench let the ADC trigger be moved? [zh] 台架能否移动 ADC 触发点？
template <typename Bench, typename = void> struct movable_trigger : std::false_type {};

template <typename Bench>
struct movable_trigger<Bench, std::void_t<decltype(std::declval<Bench&>().set_sample_delay(0.0F)),
                                          decltype(std::declval<const Bench&>().sample_delay())>> : std::true_type {};
} // namespace detail

/**
 * @brief [en] Runs the checks and names the most likely single cause.
 *
 *     Why a cause and not a list. A board that is mis-wired usually fails **several** items at once, and the items are
 *     not independent: an inverted current channel makes the three-wire sum non-zero *and* the dead-time identification
 *     come out negative *and* the flux identification reverse. Reporting three failures sends the user to three places;
 *     the intended behaviour is to report the one thing that explains the most of them. The scoring below is exactly
 *     that: each candidate cause counts how many of the failed items it accounts for, and the winner is reported.
 *
 * @brief [zh] 跑完检查，并指出最可能的那个单一成因。
 *
 *     为什么报一个成因而不是一张表。接错的板子通常**同时**弄坏好几项，而这些项并不独立：一条反 相的电流通道会让三线之和
 *     不为零、让死区辨识出负值、让磁链辨识反号。报三条失败会把用户支到 三个地方；该做的是报那个"解释得最多"的一件。下面
 *     的打分就是这个：每个候选成因数一数它解释 了几条失败，胜者上报。
 *
 * @param m [en] what was measured.
 * @param m [zh] 量到了什么。
 * @param limits [en] the tolerances.
 * @param limits [zh] 容差。
 * @return [en] every number plus the ranked cause.
 * @return [zh] 每个数，加上排名后的成因。
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
 * @brief [en] The standstill half of the board self-test: it **measures** the
 * numbers `check_board()` judges. [zh]
 * 板级自检的堵转半边：它**量出** `check_board()` 要判定的那些数。
 *
 *        [en] Why a template and not a class with virtuals. The bench is whatever the user's board is, and this project
 *        has no exceptions, no RTTI and no allocation in the control path; a template over a four-call concept costs
 *        nothing and keeps the sequence in one readable place. The concept is: ``` auto set_dq(Dq<float>) -> void; //
 *        hold a voltage in the rotor frame auto advance(float) -> void; // let time pass (>= 1 switching period) auto
 *        step_advance(float) -> void; // advance by less than one period auto read_uvw() -> Uvw<float>; // the ADC
 *        result the ISR reads (a sample // taken at the trigger point, not "now") auto switching_period_s() -> float;
 *        // optional, detected by SFINAE -- see `sample_delay_measured`: auto set_sample_delay(float fraction) -> void;
 *        // move the ADC trigger auto sample_delay() const -> float; // where it is now, 0..1 ```
 *        [zh] 为什么是模板而不是带虚函数的类。台架就是用户那块板子，而本项目在控制路径上 没有异常、没有
 *        RTTI、不分配内存；模板对着一个四调用的概念，零开销，并把整个序列留在一处 可读的地方。概念如上。
 *
 *        [en] What it can and cannot measure at standstill, stated up front because a self-test that silently skips
 *        half the checks is worse than none:
 *        - **can**: per-phase offsets (zero current is its own reference),
 *          polarity (drive a known vector and read the signs), `Rs` and the
 *          dead-time voltage (two points, see `RsEstimator`), and the switching
 *          ripple the ADC actually sees.
 *        - **cannot**: `psi_f`, the encoder mount offset and the observer's
 *          health, all of which need the rotor to turn. Those come from the
 *          spin test and the aligner, and are passed in.
 *        - **can measure the ADC trigger, if the bench lets it move the
 *          trigger**: the ripple check compares what the ADC sees with what the
 *          switching period implies, and the only way to know what the ADC sees
 *          is to move the trigger across the period and watch the reading move.
 *          That is why `set_sample_delay` / `sample_delay` are in the concept at
 *          all. Because a bench that cannot move its trigger is a perfectly
 *          reasonable thing for a user to have, they are **optional**: detected
 *          by SFINAE, and when absent `ripple_measured_a` is whatever the caller
 *          supplied.
 *        [zh] 堵转能测什么、不能测什么，先说清楚——静默跳过一半检查的自检比没有更糟：
 *        - **能**：逐相零漂（零电流自己就是基准）、极性（给一个已知矢量读符号）、`Rs` 与
 *          死区伏秒（两点，见 `RsEstimator`）、以及 ADC 实际看到的开关纹波。
 *        - **不能**：`ψf`、编码器安装偏置、观测器健康度，它们都需要转子转起来。那些来自
 *          旋转测试与对齐器，由调用方传入。
 *        - **能测 ADC 触发点，前提是台架允许移动触发**：纹波检查拿 ADC 看到的与开关周期应有的比，
 *          而要知道 ADC 到底看到什么，唯一办法是把触发点在整个周期里移动并看读数怎么动。这就是
 *          `set_sample_delay` / `sample_delay` 在前面的概念里的原因。因为"台架不能移触发点"对用户
 *          是完全合理的处境，它们是**可选**的：由 SFINAE 探测，缺省时 `ripple_measured_a` 用调用方
 *          给的值。
 *
 * @context [en] Commissioning, standstill, once per board. [zh]
 * 整定阶段，堵转，每块板一次。
 * @since 0.1.0
 * @exporter
 */
class BoardSelfTest final {
  public:
    /**
     * @brief [en] How hard to probe and how long to wait.
     * @brief [zh] 探多深、等多久。
     * @since 0.1.0
     */
    struct Config final {
        float probe_v[6];  ///< [en] the d-axis voltages to sweep, V; six or more,
                           ///< because the `Rs` fit needs them. [zh]
                           ///< 要扫描的 d 轴电压，V；六个以上，`Rs` 拟合需要。
        int settle_steps;  ///< [en] switching periods to wait per point. [zh]
                           ///< 每个点等多少个开关周期。
        int average_steps; ///< [en] periods to average per point. [zh]
                           ///< 每个点平均多少个周期。
        int zero_steps;    ///< [en] zero-current samples to average. [zh]
                           ///< 要平均的零电流样本数。
        int ripple_steps;  ///< [en] sub-steps per period for the ripple read. [zh]
                           ///< 读纹波时每周期取多少子步。
        int delay_steps;   ///< [en] points of the switching period to sweep when locating the
                           ///< ADC trigger; 0 = do not sweep. [zh]
                           ///< 定位 ADC 触发点时在开关周期内扫多少点；0 = 不扫。
        float l_h;         ///< [en] inductance, H -- for the ripple bound only. [zh]
                           ///< 电感，H——只用于纹波界。
        float i_min_a;     ///< [en] below this the loss is not a step yet, A. [zh]
                           ///< 低于此值时损失还不是阶跃，A。
    };

    /**
     * @brief [en] What the standstill half produced.
     * @brief [zh] 堵转半边量到了什么。
     * @since 0.1.0
     */
    struct Result final {
        BoardMeasurements measurements;   ///< [en] the numbers to judge. [zh]
                                          ///< 待判定的数。
        float rs_ohm;                     ///< [en] identified resistance, ohm. [zh]
                                          ///< 辨识出的电阻，Ω。
        bool complete;                    ///< [en] every step ran. [zh] 每一步都跑完了。
        int points_used;                  ///< [en] voltage points the `Rs` fit accepted. [zh]
                                          ///< `Rs` 拟合采用的电压点数。
        bool sample_delay_measured;       ///< [en] the bench could move its trigger, and the
                                          ///< sweep ran. [zh] 台架能移触发点且扫过了。
        float sample_delay_fraction;      ///< [en] where the trigger actually sits, in switching
                                          ///< periods. [zh] 触发点实际所在位置，以开关周期为单位。
        float sample_delay_best_fraction; ///< [en] where it should sit -- the centre, in
                                          ///< periods. [zh] 它该在的位置——中心，以周期为单位。
        float sample_error_a;             ///< [en] how far the reading at the actual trigger sits
                                          ///< from the period average, A. [zh]
                                          ///< 实际触发点上的读数偏离周期均值多远，A。
    };

    explicit BoardSelfTest(const Config& config) noexcept : config_(config) {}

    /**
     * @brief [en] Run the sequence.
     * @brief [zh] 跑完整个序列。
     * @param bench [en] the board, through the four-call concept in the class note.
     * @param bench [zh] 板子，通过类注释里的四调用概念。
     * @param spinning [en] the numbers that need the rotor to turn, supplied by the caller.
     * @param spinning [zh] 需要转子转起来才能得到的数，由调用方提供。
     * @return [en] every measured number, ready for `check_board()`.
     * @return [zh] 量到的每个数，可直接交给 `check_board()`。
     * @since 0.1.0
     */
    template <typename Bench> auto run(Bench& bench, const BoardMeasurements& spinning) noexcept -> Result {
        Result result{};
        result.measurements = spinning;
        if (!(config_.settle_steps > 0) || !(config_.average_steps > 0) || !(config_.zero_steps > 0)) {
            return result;
        }
        const float period = bench.switching_period_s();

        // --- 1. zero current: the readings *are* the offsets ---
        // --- 1. 零电流：读数**就是**零漂 ---
        bench.set_dq(Dq<float>{0.0F, 0.0F});
        for (int k = 0; k < config_.settle_steps; ++k) {
            bench.advance(period);
        }
        float sum[3]{0.0F, 0.0F, 0.0F};
        for (int k = 0; k < config_.zero_steps; ++k) {
            const Uvw<float> i = bench.read_uvw();
            sum[0] = static_cast<float>(sum[0] + i.u);
            sum[1] = static_cast<float>(sum[1] + i.v);
            sum[2] = static_cast<float>(sum[2] + i.w);
            bench.advance(period);
        }
        const float inverse = 1.0F / static_cast<float>(config_.zero_steps);
        for (int phase = 0; phase < 3; ++phase) {
            result.measurements.offset_a[phase] = sum[phase] * inverse;
        }

        // --- 2. sweep the d axis: polarity, `Rs`, dead time, and the sum ---
        // --- 2. 扫 d 轴：极性、`Rs`、死区、以及三线和 ---
        RsEstimator rs_estimator(RsEstimator::Config{config_.i_min_a, 0.0F});
        float v_used = 0.0F;
        float sum_current = 0.0F;
        int sum_samples = 0;
        for (int p = 0; p < 6; ++p) {
            const float v = config_.probe_v[p];
            if (!(v > 0.0F)) {
                continue;
            }
            bench.set_dq(Dq<float>{v, 0.0F});
            for (int k = 0; k < config_.settle_steps; ++k) {
                bench.advance(period);
            }
            Uvw<float> mean{0.0F, 0.0F, 0.0F};
            for (int k = 0; k < config_.average_steps; ++k) {
                const Uvw<float> i = bench.read_uvw();
                mean = Uvw<float>{static_cast<float>(mean.u + i.u), static_cast<float>(mean.v + i.v),
                                  static_cast<float>(mean.w + i.w)};
                bench.advance(period);
            }
            const float s = 1.0F / static_cast<float>(config_.average_steps);
            mean = Uvw<float>{static_cast<float>(mean.u * s), static_cast<float>(mean.v * s),
                              static_cast<float>(mean.w * s)};
            // The offsets have to come out before the signs mean anything.
            // 符号要有意义，先得把零漂去掉。
            const float u = static_cast<float>(mean.u - result.measurements.offset_a[0]);
            const float vv = static_cast<float>(mean.v - result.measurements.offset_a[1]);
            const float w = static_cast<float>(mean.w - result.measurements.offset_a[2]);
            sum_current = static_cast<float>(sum_current + ((u + vv) + w));
            ++sum_samples;
            // The current magnitude on the axis, from the three phases: the
            // vector magnitude is `sqrt(2/3*(u^2+v^2+w^2))`.
            // 轴上的电流幅值，由三相得到：矢量幅值是 `sqrt(2/3·(u²+v²+w²))`。
            const float magnitude = std::sqrt(0.6666667F * (((u * u) + (vv * vv)) + (w * w)));
            static_cast<void>(rs_estimator.add(v, magnitude));
            if (p == 5) {
                // The signs and the ripple come from the strongest point, where
                // the signal-to-noise is best.
                // 符号与纹波取自最强的那个点，那里信噪比最好。
                result.measurements.response_sign[0] = (u > 0.0F) ? 1.0F : -1.0F;
                result.measurements.response_sign[1] = (vv > 0.0F) ? 1.0F : -1.0F;
                result.measurements.response_sign[2] = (w > 0.0F) ? 1.0F : -1.0F;
                v_used = v;
                // The sampling-instant check is not a by-product of the `Rs` sweep: finding the
                // trigger means moving it across the period and watching the reading move, which
                // section 2b does *if* the bench can move its trigger. Measuring the ripple by
                // looking harder at one trigger point would only measure aliasing.
                // 采样时刻这项不是 `Rs` 扫描的副产品：要找到触发点，得把它在整个周期里移动并看读数怎么动，
                // 那由第 2b 节在**台架能移触发**时做。只在同一个触发点上看得更仔细，量到的只是混叠。
            }
        }

        // --- 3. the fit ---
        // --- 3. 拟合 ---
        const RsEstimator::Result fit = rs_estimator.solve();
        result.points_used = rs_estimator.samples();
        result.rs_ohm = fit.rs;
        result.measurements.rs_ohm = fit.rs;
        result.measurements.deadtime_v = fit.deadtime_v;
        if (sum_samples > 0) {
            result.measurements.sum_current_a = sum_current / static_cast<float>(sum_samples);
        }
        // The ripple bound the library already uses elsewhere, so the ratio the
        // user sees is comparable with the numbers in the dead-time tests.
        // 库在别处已经在用的那个纹波界，于是用户看到的比值与死区测试里的数可比。
        const float bounce = bench.v_bus_v();
        result.measurements.ripple_expected_a =
            (config_.l_h > 0.0F) ? ((bounce * period) / (8.0F * config_.l_h)) : 0.0F;
        // --- 2b. the ADC trigger, when the bench can move it ---
        // --- 2b. ADC 触发点，当台架能移它时 ---
        sweep_sample_delay(bench, v_used, period, result);
        result.complete = (result.points_used > 0) && (sum_samples > 0);
        return result;
    }

    /// [en] The config in force. [zh] 当前配置。
    [[nodiscard]] auto config() const noexcept -> const Config& { return config_; }

  private:
    static constexpr int kMaxDelayPoints = 32;

    /// [en] Current magnitude from the three phases, offsets removed. [zh]
    /// 由三相得到电流幅值，已去零漂。
    [[nodiscard]] auto magnitude(const Uvw<float>& raw, const Result& result) const noexcept -> float {
        const float u = static_cast<float>(raw.u - result.measurements.offset_a[0]);
        const float v = static_cast<float>(raw.v - result.measurements.offset_a[1]);
        const float w = static_cast<float>(raw.w - result.measurements.offset_a[2]);
        return std::sqrt(0.6666667F * (((u * u) + (v * v)) + (w * w)));
    }

    /**
     * @brief [en] Move the ADC trigger across the switching period and read what moves.
     *
     *     Three numbers come out of the same sweep. The reading's **peak-to-peak across the period** is the ripple the
     *     ADC actually sees, which is what the `SamplingInstant` check compares against the switching period. The
     *     **deviation of the reading at the board's own trigger from the period average** (`sample_error_a`) is what
     *     that position costs the loop in amps. The **position of the minimum deviation** is where the trigger ought to
     *     be -- and it is reported rather than assumed, because it is *not* always the centre of the period: a
     *     centre-aligned PWM puts the mean at the centre, an edge-aligned one puts it wherever the current crosses its
     *     average, which depends on the duty. Assuming 0.5 would be a guess wearing a constant.
     *
     * @brief [zh] 把 ADC 触发点在开关周期里挪一遍，看什么在动。
     *
     *     同一次扫描给出三个数。读数在**整个周期里的峰峰值**就是 ADC 实际看到的纹波，即 `SamplingInstant` 项与开关周期
     *     对比的那个量。**板子自己那个触发点上的读数相对周期均值的 偏离**（`sample_error_a`）就是那个位置在安培上让环付
     *     出多少。**偏离最小处的位置**才是触发点 该在的地方——而它是被**量出来并上报**的，不是被假设的，因为它**并不总是
     *     **周期中心：中心对齐的 PWM 把均值放在中心，边沿对齐的把均值放在电流穿过自身平均值的时刻，那取决于占空比。假设
     *     0.5 是一个披着常数外衣的猜测。
     *
     * @note [en] Skipped entirely when the bench has no trigger to move; that is detected at compile time, not at run
     *            time, so a bench without it costs nothing and does not need a stub.
     * @note [zh] 台架没有可动的触发点时整段跳过；这在编译期探测，所以没有该能力的台架零开销、也不必写桩。
     * @since 0.1.0
     */
    template <typename Bench>
    auto sweep_sample_delay(Bench& bench, float v_drive, float period, Result& result) noexcept -> void {
        if constexpr (detail::movable_trigger<Bench>::value) {
            const int points = (config_.delay_steps < kMaxDelayPoints) ? config_.delay_steps : kMaxDelayPoints;
            if ((points < 3) || !(v_drive > 0.0F) || !(config_.average_steps > 0)) {
                return;
            }
            const float saved = bench.sample_delay();
            bench.set_dq(Dq<float>{v_drive, 0.0F});
            float readings[kMaxDelayPoints]{};
            float sum = 0.0F;
            for (int k = 0; k < points; ++k) {
                bench.set_sample_delay(static_cast<float>(k) / static_cast<float>(points));
                for (int s = 0; s < config_.settle_steps; ++s) {
                    bench.advance(period);
                }
                float acc = 0.0F;
                for (int s = 0; s < config_.average_steps; ++s) {
                    acc = static_cast<float>(acc + magnitude(bench.read_uvw(), result));
                    bench.advance(period);
                }
                readings[k] = acc / static_cast<float>(config_.average_steps);
                sum = static_cast<float>(sum + readings[k]);
            }
            const float period_mean = sum / static_cast<float>(points);
            float lo = readings[0];
            float hi = readings[0];
            float best_dev = std::fabs(readings[0] - period_mean);
            int best = 0;
            for (int k = 1; k < points; ++k) {
                lo = (readings[k] < lo) ? readings[k] : lo;
                hi = (readings[k] > hi) ? readings[k] : hi;
                const float dev = std::fabs(readings[k] - period_mean);
                if (dev < best_dev) {
                    best_dev = dev;
                    best = k;
                }
            }
            result.measurements.ripple_measured_a = static_cast<float>(hi - lo);
            result.sample_delay_best_fraction = static_cast<float>(best) / static_cast<float>(points);
            // The error is read at the setting the board *actually* uses, not at
            // the best one: the sweep is how we learn what the right setting is,
            // the board's own setting is what it has to live with.
            // 误差按板子**实际**用的那个设置读，不是按最好的那个：扫描用来弄清正确设置是什么，
            // 而板子得过日子的是它自己的设置。
            const int at = static_cast<int>((saved * static_cast<float>(points)) + 0.5F);
            const int clamped = (at < 0) ? 0 : ((at >= points) ? (points - 1) : at);
            result.sample_error_a = std::fabs(readings[clamped] - period_mean);
            bench.set_sample_delay(saved);
            result.sample_delay_fraction = saved;
            result.sample_delay_measured = true;
        } else {
            static_cast<void>(bench);
            static_cast<void>(v_drive);
            static_cast<void>(period);
            static_cast<void>(result);
        }
    }

    Config config_{};
};

/**
 * @brief [en] The item as text, for a log line or a screen.
 * @brief [zh] 检查项的文本形式，给日志或界面用。
 * @param item [en] which check.
 * @param item [zh] 哪一项。
 * @return [en] a stable, lowercase name; never null.
 * @return [zh] 稳定的小写名字；不会是空指针。
 * @note [en] Paired with `board_cause_name` so a dump loop is two calls and no switch statement in the user's code --
 *            the procedure document in the repo is meant to be copy-pasteable.
 * @note [zh] 与 `board_cause_name` 配对，于是导出一段日志就是两次调用，用户代码里不需要 switch—— 仓库里的流程文档是要能
 *            直接照抄的。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] constexpr auto board_item_name(BoardItem item) noexcept -> const char* {
    switch (item) {
    case BoardItem::CurrentSum:
        return "current sum";
    case BoardItem::CurrentOffset:
        return "current offset";
    case BoardItem::CurrentPolarity:
        return "current polarity";
    case BoardItem::SamplingInstant:
        return "sampling instant";
    case BoardItem::DeadTime:
        return "dead time";
    case BoardItem::EncoderOffset:
        return "encoder offset";
    case BoardItem::Parameters:
        return "machine parameters";
    case BoardItem::ObserverHealth:
        return "observer";
    }
    return "?";
}

/**
 * @brief [en] The cause as text, for a log line or a screen.
 * @brief [zh] 成因的文本形式，给日志或界面用。
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
