// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <foc/core/machine_params.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] One harmonic of the cogging torque a compensator will cancel: `amplitude * sin(order*theta_mech +
 *             phase)`.
 *
 *   `order` is **per mechanical revolution**. The compensator and the plant that injects it must speak the same
 *   indexing or the compensation silently cancels nothing -- and a factor of `pole_pairs` is the natural way to get
 *   that wrong, so the type is shared with the SIL injection rather than re-declared here. `order`
 *
 * @brief [zh] 补偿器要抵消的齿槽转矩的一个谐波：`amplitude · sin(order·θ_mech + phase)`。
 *
 *   是 **每机械转** 的次数。补偿器与注入它的被控对象必须用同一套索引，否则补偿会悄无声息地什么都不抵消——而 "错一个极对
 *   数" 是弄错它的最自然方式，所以这个类型与 SIL 注入共用，而不是在这里重新声明一遍。
 * @since 0.1.9
 * @exporter
 */
struct CoggingCompHarmonic final {
    std::uint8_t order; ///< [en] per mechanical revolution. [zh] 每机械转的次数。
    float amplitude_nm; ///< [en] peak of this harmonic, N*m. [zh] 该谐波的峰值，N·m。
    float phase_rad;    ///< [en] phase at `theta_mech = 0`, rad. [zh] `θ_mech = 0` 处的相位，rad。
};

/**
 * @brief [en] The compensator's machine and its two limits.
 *
 *   How much current it may add, and where it stops asking.
 *
 * @brief [zh] 补偿器的机器与两个限制。
 *
 *   能加多少电流，以及从哪里开始不再要求。
 *
 * @note [en] `fade_from_hz` / `fade_to_hz` are the frequency boundary the SIL measurement puts a number on: with the
 *            speed loop holding 10 rad/s mechanical, an injected 4 mN*m reappears in the q-axis current at `T_cog/Kt`
 *            (0.079 A measured against 0.080 A predicted), but at 80 rad/s -- a 76 Hz sixth harmonic, well above the
 *            250 rad/s speed loop -- only 49 % of that share survives. Above the current loop's bandwidth a demand for
 *            ripple current buys heat and phase-shifted current rather than smooth motion, so the fade is a decision
 *            about *where to stop*, and it is configured rather than assumed. `fade_from_hz == 0` disables it.
 * @note [zh] `fade_from_hz` / `fade_to_hz` 是 SIL 测量给出数字的那条频率边界：速度环守住 10 rad/s 机械时，注入的 4 mN·m
 *            以 `T_cog/Kt` 重现于 q 轴电流（实测 0.079 A，预测 0.080 A），但 80 rad/s（6 次 76 Hz，远高于 250 rad/s 的
 *            速度环）处只剩那份份额的 49 %。超过电流环带宽之后，要求纹波电流买到的是热和相移的电流而不是平稳的运动，所
 *            以淡出是一个 **在哪里停** 的决定，而且它是配置出来的，不是假设出来的。`fade_from_hz == 0` 表示不淡出。
 * @since 0.1.9
 * @exporter
 */
struct CoggingCompConfig final {
    MachineParams params;    ///< [en] `psi_f`, `ld`, `lq`; `rs` unused. [zh] 用 `psi_f`、`ld`、`lq`；不用 `rs`。
    std::uint8_t pole_pairs; ///< [en] the electrical-to-mechanical bridge. [zh] 电角与机械角之间的桥。
    float fade_from_hz;      ///< [en] compensation starts fading here; 0 = never. [zh] 从这里开始淡出；0 = 不淡出。
    float fade_to_hz;        ///< [en] fully faded here; must exceed `fade_from_hz`. [zh] 到这里完全淡出；须大于前者。
};

/**
 * @brief [en] One control period's worth of compensation, and which of the limits bound it.
 *
 *   `gain` is reported separately from `iq_a` because the two failure modes are different and a trace has to tell them
 *   apart: `gain` is "the frequency fade decided not to ask for this", `clipped` is "the current ceiling refused it". A
 *   compensator that hides either one looks like a compensator that works. `gain`
 *
 * @brief [zh] 一个控制周期的补偿量，以及是哪个上限限制住了它。
 *
 *   与 `iq_a` 分开报告，是因为两种失效不同、轨迹必须能分开：`gain` 是「频率淡出决定不要这个」，`clipped` 是「电流上限不
 *   让」。把任一藏起来的补偿器，看起来都像一个能用的补偿器。
 * @since 0.1.9
 * @exporter
 */
struct CoggingCompOutput final {
    ErrorCode status; ///< [en] `Ok`, or why not. [zh] `Ok`，或为什么不是。
    float iq_a;       ///< [en] current to **add** to the q-axis reference, A. [zh] 要 **加** 到 q 轴参考上的电流，A。
    float torque_nm;  ///< [en] the cancelled torque before any limit, N*m. [zh] 未被任何限制影响前的被抵消转矩，N·m。
    float gain;       ///< [en] the frequency fade in [0, 1]. [zh] 频率淡出系数，[0, 1]。
    bool clipped;     ///< [en] the current ceiling trimmed the answer. [zh] 电流上限削掉了答案的一部分。
};

/**
 * @brief [en] One control period's inputs: where the rotor is, what current it is carrying, and how fast it turns.
 * @brief [zh] 一个控制周期的输入：转子在哪、带着什么电流、转得多快。
 * @since 0.1.9
 * @exporter
 */
struct CoggingCompInput final {
    float theta_mech_rad;   ///< [en] mechanical angle, unwrapped or not. [zh] 机械角，归一化与否都可。
    float omega_mech_rad_s; ///< [en] mechanical speed, rad/s. [zh] 机械转速，rad/s。
    float id_a;             ///< [en] d-axis current now, A. [zh] 当前 d 轴电流，A。
    float iq_command_a;     ///< [en] the reference this offset will be added to, A. [zh] 本偏置要加到的那个参考，A。
    float i_max_a;          ///< [en] the current ceiling, A. [zh] 电流上限，A。
};

/**
 * @brief [en] Torque-ripple feedforward: cancel a cogging table by asking for the current that would produce its
 *             negative, from the angle, **without waiting for a speed error**.
 *
 *   **Why feedforward and not a correction.** Measured on the closed loop (25 % of a revolution's worth of samples,
 *   projected on the mechanical angle): a 4 mN*m sixth harmonic injected at 10 rad/s reappears in the measured q-axis
 *   current at 0.079 A against the 0.080 A that `T_cog/Kt` predicts. The loops therefore *do* answer cogging -- but
 *   only after the rotor has already been pushed, and the price is the speed ripple the torque caused on the way. A
 *   feedforward pays none of that: it computes the same current from the angle a period earlier and never lets the
 *   error happen.
 *
 *   **The gain is the salient machine's, not the nameplate's.** `T = 1.5*p*(psi_f + (ld-lq)*id)*iq`, so the current
 *   that produces a wanted torque depends on `id`. Using `Kt = 1.5*p*psi_f` alone scales the answer by the ratio
 *   `(psi_f + (ld-lq)*id)/psi_f` -- on the F32 profile at `id = -1.8 A` that ratio is **2.64**, so the naive gain asks
 *   for 2.64x the current the machine actually needs and the ripple comes back
 *   **inverted at 1.5x its original amplitude**. Worse than doing nothing, and it lands exactly where compensation is
 *   most wanted (high speed, d-axis current flowing).
 *
 *   **Where it stops.** Above the current loop's bandwidth the demand buys heat and phase-shifted current instead of
 *   smooth motion: the same measurement shows the loop's own share of the cogging falling to 49 % at a 76 Hz ripple.
 *   `fade_from_hz`/`fade_to_hz` make that a configured decision, and the fade is reported so a trace can tell "we chose
 *   not to ask" from "we asked and the ceiling refused".
 *
 *   **What it cannot do.** It cancels a torque ripple. It does not find the table (that is the self-learning piece), it
 *   does not correct for a table whose phase is wrong (a wrong phase cancels *nothing* -- the falsifier in the test),
 *   and at the flux-nulling current `psi_f + (ld-lq)*id = 0` the machine cannot produce torque at all, so the offset is
 *   refused rather than divided by.
 *
 * @brief [zh] 转矩纹波前馈：由角度算出能产生齿槽 **负值** 的电流，从参考侧抵消一张齿槽表， **不等速度误差** 。
 *
 *   **为什么是前馈而不是修正。** 闭环实测（一整转的采样按机械角投影）：10 rad/s 处注入 4 mN·m 的 6 次分量，实测 q 轴电
 *   流里重现 **0.079 A** ，而 `T_cog/Kt` 预测 **0.080 A** 。所以环路 **确实** 会回答齿槽——但只在转子已经被推动之后，代
 *   价是那一路上它引起的速度纹波。前馈一点也不用付：它早一个周期就从角度算出同一份电流，从不给误差发生的机会。
 *
 *   **增益是凸极机的，不是铭牌的。** `T = 1.5·p·(ψf + (Ld−Lq)·id)·iq`，所以产生某个转矩所需的电流取决于 `id`。只用 `Kt
 *   = 1.5·p·ψf` 会把答案按 `(ψf + (Ld−Lq)·id)/ψf` 的比例缩放——在 F32 那台档案机上 `id = −1.8 A` 时该比例是 **2.64** ，
 *   于是天真的增益要了机器实际需要的 **2.64 倍** 电流，纹波 **被翻到反向、幅值是原来的 1.5 倍** 。比什么都不做更糟，而
 *   且它恰好落在最需要补偿的地方（高速、d 轴有电流）。
 *
 *   **在哪里停。** 超过电流环带宽之后，要求到的纹波电流买到的是热与相移电流而不是平稳运动：同一次测量显示环路自己那份齿
 *   槽在 76 Hz 纹波处已降到 49 %。`fade_from_hz`/`fade_to_hz` 把这个决定做成配置项，并把淡出量报出来，好让轨迹区分「我
 *   们选择不要」与「要了但被上限拒了」。
 *
 *   **它做不到什么。** 它抵消的是转矩纹波；它不负责找表（那是自学习那件），也不纠正相位错的表（相位错则**什么都不抵消**
 *   ——用例里的反证），而在削磁电流处 `ψf + (Ld−Lq)·id = 0`，机器根本出不了转矩，于是偏置被拒绝而不是被除。
 * @since 0.1.9
 * @exporter
 */
class CoggingCompensation final {
  public:
    /** @brief [en] How many harmonics a table may hold. [zh] 一张表最多能装几条谐波。 */
    static constexpr int kMaxHarmonics = 6;

    CoggingCompensation() noexcept = default;

    explicit CoggingCompensation(const CoggingCompConfig& config) noexcept : config_(config) {}

    /**
     * @brief [en] Install or clear the table the compensator cancels.
     * @brief [zh] 装入或清空补偿器要抵消的表。
     * @param harmonics [en] the table; null or `count <= 0` clears it. Order 0 is dropped (a constant torque is a load,
     *                       not cogging).
     * @param harmonics [zh] 表；空指针或 `count <= 0` 表示清空。0 次被丢弃（恒定转矩是负载，不是齿槽）。
     * @param count [en] how many entries are valid. Clamped to `kMaxHarmonics`.
     * @param count [zh] 有效条数。截断到 `kMaxHarmonics`。
     * @since 0.1.9
     */
    void set_table(const CoggingCompHarmonic* harmonics, int count) noexcept;

    /** @brief [en] How many harmonics are installed. [zh] 当前装了几条谐波。 */
    [[nodiscard]] auto harmonics() const noexcept -> int { return count_; }

    /**
     * @brief [en] Whether the configuration can produce an offset at all.
     * @brief [zh] 配置能不能算出一个偏置。
     * @return [en] `Ok`, or `Param` for a non-positive inductance, a negative flux linkage, zero pole pairs, a negative
     *              or infinite fade band, or a `fade_to_hz` that does not exceed `fade_from_hz`.
     * @return [zh] `Ok`；电感非正、磁链为负、极对数为零、淡出带为负或无穷、或 `fade_to_hz` 不大于 `fade_from_hz` 时为
     *              `Param`。
     * @since 0.1.9
     */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode;

    /**
     * @brief [en] The q-axis current to add this period.
     * @brief [zh] 本周期要加的 q 轴电流。
     * @param in [en] angle, speed, `id`, the reference and the ceiling.
     * @param in [zh] 角度、转速、`id`、参考与上限。
     * @return [en] the offset with its fade and clipping reported; `Param` for a non-finite input or a machine that
     *              cannot produce torque at this `id`.
     * @return [zh] 偏置，连同淡出与削顶一起报出；输入非有限、或机器在该 `id` 处出不了转矩时为 `Param`。
     * @note [en] The offset is **added** to `iq_command_a`, and the ceiling is applied to the *sum*, because that is
     *            what the current loop will see. Clipping the offset alone would let the total exceed the limit while
     *            reporting a happy offset.
     * @note [zh] 偏置是 **加** 到 `iq_command_a` 上的，上限作用在 **和** 上，因为电流环看到的就是那个和。只对偏置削顶会
     *            让总电流越限，却报出一个愉快的偏置。
     * @since 0.1.9
     */
    [[nodiscard]] auto offset(const CoggingCompInput& in) const noexcept -> CoggingCompOutput;

  private:
    CoggingCompConfig config_{};
    CoggingCompHarmonic table_[kMaxHarmonics]{};
    int count_{0};
};

} // namespace core

} // namespace foc
