// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Phase of the rotor-alignment sequence.
 * @brief [zh] 转子对齐流程的阶段。
 * @since 0.1.0
 * @exporter
 */
enum class AlignState : std::uint8_t {
    Idle = 0,   ///< [en] not started or reset. [zh] 未开始或已复位。
    Ramp = 1,   ///< [en] current rising. [zh] 电流爬升中。
    Settle = 2, ///< [en] holding, averaging. [zh] 保持并平均角度。
    Verify = 3, ///< [en] quarter-turn check. [zh] 四分之一圈校验。
    Done = 4,   ///< [en] offset is valid. [zh] 偏置有效。
    Failed = 5, ///< [en] see failure(). [zh] 见 failure()。
};


/**
 * @brief [en] Why an alignment sequence was rejected.
 * @brief [zh] 对齐流程被拒的原因。
 * @note [en] A reason is a diagnosis, not a control decision: the motor layer turns any failure into a latched fault,
 *            because an alignment that did not converge leaves the rotor at an unknown angle, and continuing to
 *            modulate would be guessing.
 * @note [zh] 原因只是诊断，不是控制决策：电机层把任何失败都变成锁存故障——没收敛的对齐会把转子留在 未知角度上，继续调制
 *            就是靠猜。
 * @since 0.1.0
 * @exporter
 */
enum class AlignFailure : std::uint8_t {
    None = 0,           ///< [en] no failure. [zh] 无失败。
    NotConfigured = 1,  ///< [en] recipe rejected. [zh] 配方被拒。
    NotSettled = 2,     ///< [en] rotor kept moving. [zh] 转子仍在动。
    DirectionWrong = 3, ///< [en] rotor moved backwards. [zh] 转子反向移动。
};


/**
 * @brief [en] Alignment recipe. Every number is the caller's: the library ships no default.
 * @brief [zh] 对齐配方。每个数字都由调用方给出，库不自带默认值。
 * @note [en] `current_a`, `hold_s` and `settle_tolerance_rad` must be strictly positive; a zero there is rejected by
 *            `validate()` instead of being replaced by a plausible-looking constant. The three that may legitimately be
 *            zero say so below. A **default-constructed** recipe (every field zero) means something else: alignment was
 *            never requested, and `validate()` answers `NotImpl` rather than `Param`. That is what lets an aligner be
 *            compiled in and stay inert until somebody configures it. Any other partial recipe is a `Param` error -- a
 *            half-configured alignment does not exist.
 *
 * @note [zh] `current_a`、`hold_s`、`settle_tolerance_rad` 必须严格为正；那里的 0 会被 `validate()` 拒绝，而不是被换成
 *            一个看起来合理的常数。可以合法取 0 的三项在下面各自说明。
 *
 *     **默认构造**的配方（全零）是另一回事：调用方没有请求过对齐，`validate()` 答 `NotImpl` 而不是 `Param`。这就是"对齐
 *     器编进来了，但在被配置之前保持静默"的实现方式。其余任何半配置 的配方都是 `Param` 错误——"配置了一半的对齐"不存在。
 * @since 0.1.0
 * @exporter
 */
struct AlignConfig final {
    float current_a;            ///< [en] pull strength, A (> 0). [zh] 拉力，A（> 0）。
    float ramp_s;               ///< [en] rise time, s (0 = step). [zh] 爬升时间，s（0 = 直接给足）。
    float hold_s;               ///< [en] window, s (> 0). [zh] 窗口，s（> 0）。
    float settle_tolerance_rad; ///< [en] allowed spread, rad (> 0). [zh] 允许离散度，rad（> 0）。
    float verify_s;             ///< [en] check hold, s (0 = skip). [zh] 校验保持，s（0 = 跳过）。
    float align_angle_e_rad;    ///< [en] forced angle, rad (usually 0). [zh] 强制角，rad（通常为 0）。
};


/**
 * @brief [en] What the aligner asks the current loop to do on one tick.
 * @brief [zh] 对齐策略在一个 tick 上要求电流环做什么。
 * @since 0.1.0
 * @exporter
 */
struct AlignOutput final {
    float id_a;        ///< [en] d-axis current, A. [zh] d 轴电流，A。
    float theta_e_rad; ///< [en] forced electrical angle, rad. [zh] 强制电角度，rad。
    bool finished;     ///< [en] accepted on this tick. [zh] 本次 tick 捕获通过。
    bool failed;       ///< [en] rejected on this tick. [zh] 本次 tick 捕获被拒。
};


/**
 * @brief [en] The "this build does not align" placeholder for the alignment slot.
 *
 *     Satisfies the aligner shape, so it can serve as the default template argument; `begin()` answers `false`, which
 *     tells `Motor` to go straight from Align to Run exactly as it did before the slot existed. A build with an
 *     absolute encoder and a stored offset genuinely has nothing to do here.
 *
 * @brief [zh] 对齐槽位的「本次构建不做对齐」占位实现。
 *
 *     满足对齐器形状，因此可作为默认模板实参；`begin()` 返回 `false`，告诉 `Motor` 从 Align 直接进 Run——与槽位出现之前
 *     的行为完全一致。装了绝对值编码器且偏置已存的构建在这里 确实无事可做。
 * @since 0.1.0
 * @exporter
 */
class NullAlignment final {
  public:
    /// [en] Nothing to configure. [zh] 无需配置。
    [[nodiscard]] auto validate() const noexcept -> ErrorCode { return ErrorCode::Ok; }

    /// [en] No sequence: the caller goes straight to closed loop. [zh] 无流程：直接进闭环。
    auto begin(std::uint32_t pole_pairs) noexcept -> bool {
        static_cast<void>(pole_pairs);
        return false;
    }

    /// [en] Zero current at electrical zero; never finished, never failed. [zh] 零电流、电角度 0；永不完成也永不失败。
    [[nodiscard]] auto step(float dt_s, float angle_mech_rad) noexcept -> AlignOutput {
        static_cast<void>(dt_s);
        static_cast<void>(angle_mech_rad);
        return AlignOutput{0.0F, 0.0F, false, false};
    }

    /// [en] Nothing to reset. [zh] 无状态可复位。
    void reset() noexcept {}
};


/**
 * @brief [en] Shaft-lock alignment: find the sensor's electrical offset by pulling the rotor to a known field
 *             direction.
 *
 *     The method, in three sentences. A stator current vector held at electrical angle `align_angle_e_rad` in the
 *     **stationary** frame drags the rotor until its d axis points the same way, so at rest the true electrical angle
 *     equals the forced one. The sensor then reads `align_angle_e_rad/pole_pairs + offset` in mechanical terms, so
 *     `offset = reading - align_angle_e_rad/pole_pairs`. Because the rotor settles at the *nearest* equilibrium, only
 *     the electrical part is observable: the offset is determined modulo `2*pi/pole_pairs`, which is why absolute
 *     multi-turn position still needs an index.
 *
 * @brief [zh] 锁轴对齐：把转子拉到已知的磁场方向，从而求出传感器的电角度偏置。
 *
 *     方法三句话：把定子电流矢量**固定在静止坐标系**的电角度 `align_angle_e_rad` 上，转子会被 拖到其 d 轴与之一致，因此
 *     静止时真实电角度就等于强制角。此时传感器读到的机械角是 `align_angle_e_rad/pole_pairs + offset`，于是 `offset = 读
 *     数 - align_angle_e_rad/pole_pairs`。 因为转子停靠在**最近的**平衡点，只有电角度部分可观测：偏置只能确定到
 *     `2*pi/pole_pairs` 的模，所以绝对多圈位置仍然需要一个索引信号。
 *
 * @note [en] The angle must be forced, not read back. Regulating `id` against the *uncorrected* sensor angle produces a
 *            constant torque error of `sin(offset)`: the field chases the rotor and the machine turns forever instead
 *            of aligning. That is why this routine returns the angle it wants rather than only a current. Verification
 *            is not optional decoration. An inverted sensor sign produces an offset that is wrong by 180 electrical
 *            degrees and *looks* perfectly converged; advancing the forced angle by a quarter turn and checking that
 *            the rotor followed in the same direction is what separates "aligned" from "aligned backwards". Set
 *            `verify_s` to 0 only when the wiring is already known-good and the extra hold is unacceptable.
 *
 * @note [zh] 角度必须被**强制**，不能回读。拿**未校正**的传感器角去调 `id`，等效转矩误差恒为 `sin(offset)`：磁场追着转
 *            子跑，电机只会一直转，永远对不齐。这就是本流程除了电流之外还要 返回角度的原因。
 *
 *     校验不是装饰。传感器符号接反会得到一个差 180 电角度、却**看起来**收敛得很好的偏置； 把强制角前进四分之一圈并检查
 *     转子是否同向跟随，才是"对齐"与"反向对齐"的分界。只有在接线 已知正确、且不接受额外保持时间时，才把 `verify_s` 设为
 *     0。
 *
 * @context [en] step() is called from the current-loop ISR: noexcept, no allocation, no I/O.
 * @context [zh] step() 由电流环 ISR 调用：noexcept、不分配、无 I/O。
 * @since 0.1.0
 * @exporter
 */
class AlignRoutine final {
  public:
    /// [en] Idle, unconfigured: validate() answers Param until set_config() gives real numbers. [zh] 空闲未配置：在 set_config() 给出真实数字之前 validate() 返回 Param。
    AlignRoutine() noexcept = default;

    /// [en] Idle with a recipe. [zh] 带配方的空闲实例。
    explicit AlignRoutine(AlignConfig config) noexcept;

    /// [en] Replace the recipe (only meaningful before begin()). [zh] 替换配方（只在 begin() 之前有意义）。
    void set_config(AlignConfig config) noexcept;

    /// [en] The recipe in force. [zh] 当前生效的配方。
    [[nodiscard]] auto config() const noexcept -> const AlignConfig&;

    /// [en] Ok when the recipe is usable, NotImpl when alignment was never requested, Param otherwise.
    /// [zh] 配方可用返回 Ok，从未请求对齐返回 NotImpl，其余情况返回 Param。
    [[nodiscard]] auto validate() const noexcept -> ErrorCode;

    /**
     * @brief [en] Start a sequence.
     * @brief [zh] 开始一次流程。
     * @param pole_pairs [en] machine pole pairs, used to convert the forced electrical angle to mechanical.
     * @param pole_pairs [zh] 电机极对数，用于把强制电角度换算成机械角。
     * @return [en] true when a sequence started, false when there is nothing to do.
     * @return [zh] 流程已启动返回true，无事可做返 回false。
     * @note [en] Call `validate()` first: an invalid recipe is a configuration error the caller must see, not a silent
     *            skip. `false` covers two different situations, and `state()` distinguishes them: an unset recipe
     *            leaves the routine in `Idle` (nothing was requested), a broken one leaves it in `Failed` with
     *            `failure() == NotConfigured`.
     * @note [zh] 先调 `validate()`：配方非法是调用方必须看到的配置错误，不能被静默跳过。 `false` 覆盖两种不同情况，由
     *            `state()` 区分：未配置的配方让流程停在 `Idle`（没人 请求过），坏掉的配方让它停在 `Failed` 且
     *            `failure() == NotConfigured`。
     * @since 0.1.0
     */
    auto begin(std::uint32_t pole_pairs) noexcept -> bool;

    /**
     * @brief [en] Advance the sequence by one control period.
     * @brief [zh] 推进一个控制周期。
     * @param dt_s [en] step length in s; a non-positive value leaves the sequence untouched.
     * @param dt_s [zh] 步长，s；非正值不推进流程。
     * @param angle_mech_rad [en] sensor reading (already offset-corrected), rad.
     * @param angle_mech_rad [zh] 传感器读数（已减偏置），rad。
     * @since 0.1.0
     */
    [[nodiscard]] auto step(float dt_s, float angle_mech_rad) noexcept -> AlignOutput;

    /// [en] Back to Idle, result cleared. [zh] 回到 Idle 并清空结果。
    void reset() noexcept;

    /// [en] Current phase. [zh] 当前阶段。
    [[nodiscard]] auto state() const noexcept -> AlignState;

    /// [en] Last failure reason. [zh] 最近一次失败原因。
    [[nodiscard]] auto failure() const noexcept -> AlignFailure;

    /// [en] Captured mechanical offset, rad in [0, 2*pi); valid once Done. [zh] 捕获到的机械偏置，rad，[0, 2π)；Done 后有效。
    [[nodiscard]] auto offset_mech_rad() const noexcept -> float;

    /// [en] Spread of the accepted capture, rad. [zh] 被接受捕获的离散度，rad。
    [[nodiscard]] auto spread_rad() const noexcept -> float;

  private:
    auto fail(AlignFailure reason) noexcept -> AlignOutput;
    void finish() noexcept;
    void enter(AlignState next) noexcept;
    void accumulate(float angle_mech_rad) noexcept;
    [[nodiscard]] auto capture_mean() noexcept -> float;

    AlignConfig config_{};
    AlignState state_{AlignState::Idle};
    AlignFailure failure_{AlignFailure::None};
    std::uint32_t pole_pairs_{0U};
    float elapsed_s_{0.0F};
    float id_a_{0.0F};
    float offset_mech_rad_{0.0F};
    float spread_rad_{0.0F};
    float last_spread_rad_{0.0F};
    float first_mean_rad_{0.0F};
    // Accumulated in double on purpose: hundreds of float samples summed in float put a noise floor
    // of roughly 1e-3 rad on the very number that decides accept/reject, which is the same order as
    // a sensible tolerance. The trig arguments stay float -- that is all the sensor gives -- but the
    // summation must not lose what little the samples carry.
    // 故意用 double 累加：数百个 float 采样用 float 求和，会给这个决定"接受/拒绝"的数字压上约 1e-3 rad
    // 的噪声地板，正好与合理容差同一量级。三角函数的自变量仍是 float（传感器只给得出这么多），但求和
    // 不能把采样里那点信息再丢掉。
    double sum_sin_{0.0};
    double sum_cos_{0.0};
    std::uint32_t samples_{0U};
};

} // namespace core

} // namespace foc
