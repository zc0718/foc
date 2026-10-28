// Conan::ImportStart
#pragma once
#include <cstdint>

#include <foc/core/alignment.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Phase of the start-up sequence.
 * @brief [zh] 起动流程的阶段。
 * @note [en] `Align` and `Pull` both run inside `Motor`'s Align state: that state is the one where the angle is forced
 *            and the observer is not stepped, which is exactly what an open-loop start needs. The sequence therefore
 *            needs no new motor state -- it is a different recipe for the same state.
 * @note [zh] `Align` 与 `Pull` 都运行在 `Motor` 的 Align 状态里：那个状态正是角度被强制、观测器不推进的状态，而开环起动
 *            需要的正是它。因此这条序列不需要新的电机状态——它是同一个状态的另一份配方。
 * @since 0.1.5
 * @exporter
 */
enum class StartupState : std::uint8_t {
    Idle = 0,   ///< [en] not started or reset. [zh] 未开始或已复位。
    Align = 1,  ///< [en] rotor pulled to a known angle and held. [zh] 把转子拉到已知角度并保持。
    Pull = 2,   ///< [en] forced vector accelerating open-loop. [zh] 强制矢量开环加速。
    Done = 3,   ///< [en] hand-over speed reached; control belongs to the observer. [zh] 已到交接转速；控制权归观测器。
    Failed = 4, ///< [en] the recipe could not be carried out. [zh] 配方无法执行。
};


/**
 * @brief [en] What a start-up sequence needs.
 * @brief [zh] 一条起动序列需要的东西。
 * @note [en] Two currents, not one: the pull strength that holds the rotor still is not the strength that drags it
 *            around a full turn, and using the same number for both either fails to move the rotor or overheats it.
 * @note [zh] 两个电流而不是一个：把转子按住的拉力和把它拖过整圈的拉力不是同一个值，两者共用一个数要么拖不动，要么发热。
 * @since 0.1.5
 * @exporter
 */
struct StartupConfig final {
    float
        align_current_a; ///< [en] current that holds the rotor at the first angle, A (> 0). [zh] 把转子按在首个角上的电流，A（> 0）。
    float align_hold_s; ///< [en] how long to hold before pulling, s (> 0). [zh] 拖动前保持多久，s（> 0）。
    float
        pull_current_a; ///< [en] current while the vector accelerates, A (> 0). [zh] 矢量加速期间施加的电流，A（> 0）。
    float
        pull_accel_rad_s2; ///< [en] electrical acceleration of the forced vector, rad/s^2 (> 0). [zh] 强制矢量的电角加速度，rad/s^2（> 0）。
    float
        handover_speed_rad_s; ///< [en] electrical speed at which control is handed over, rad/s (> 0). [zh] 交出控制权的电角速度，rad/s（> 0）。
};


/**
 * @brief [en] Open-loop start: hold, then drag the rotor up to a speed, then hand over.
 *
 * @brief [zh] 开环起动：先按住，再把转子拖到某个转速，然后交接。
 *
 *     Satisfies the aligner interface `Motor` uses, so it is a drop-in for the alignment slot rather than a second
 *     control path. `finished` is reported once the forced vector reaches `handover_speed_rad_s`, and that is the tick
 *     `Motor` leaves its Align state -- so the hand-over instant is a single well-defined tick, which is what makes "no
 *     jump at hand-over" a measurable claim rather than a wish. 满足 `Motor` 使用的对齐器接口，因此它是**对齐槽位**的一
 *     个实现，而不是第二条控制路径。强制矢量到达 `handover_speed_rad_s` 时报 `finished`，那一刻正是 `Motor` 离开 Align
 *     状态的 tick——于是交接瞬间是一个明确定义的 tick，"交接处无跳变"才成为可测量的断言而不是愿望。
 *
 * @context [en] Start-up only. Once the sequence is Done the routine stays inert until reset().
 * @context [zh] 仅起动阶段。序列进入 Done 后本类保持静默，直到 reset()。
 * @since 0.1.5
 * @exporter
 */
class StartupRoutine final {
  public:
    /**
     * @brief [en] Idle, unconfigured: validate() answers NotImpl.
     * @brief [zh] 空闲未配置：validate() 返回 NotImpl。
     */
    StartupRoutine() noexcept = default;

    /**
     * @brief [en] Idle with a recipe.
     * @brief [zh] 带配方的空闲实例。
     */
    explicit StartupRoutine(StartupConfig config) noexcept;

    /**
     * @brief [en] Replace the recipe (only meaningful before begin()).
     * @brief [zh] 替换配方（只在 begin() 之前有意义）。
     */
    void set_config(StartupConfig config) noexcept;

    /**
     * @brief [en] The recipe in force.
     * @brief [zh] 当前生效的配方。
     */
    [[nodiscard]] auto config() const noexcept -> const StartupConfig& { return config_; }

    /**
     * @brief [en] Ok when the recipe is usable, NotImpl when no start-up was requested, Param otherwise.
     * @brief [zh] 配方可用返回 Ok，未请求起动返回 NotImpl，其余返回 Param。
     */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode;

    /**
     * @brief [en] Begin the sequence.
     * @brief [zh] 开始序列。
     * @param pole_pairs [en] pole pairs, >= 1.
     * @param pole_pairs [zh] 极对数，>= 1。
     * @return [en] true if the sequence started.
     * @return [zh] 序列是否已开始。
     */
    auto begin(std::uint32_t pole_pairs) noexcept -> bool;

    /**
     * @brief [en] Advance the sequence by one control period.
     * @brief [zh] 把序列推进一个控制周期。
     * @param dt_s [en] control period in s.
     * @param dt_s [zh] 控制周期，s。
     * @param angle_mech_rad [en] the sensor's reading, used only to record the offset at the end of the hold.
     * @param angle_mech_rad [zh] 传感器读数，仅用于在保持结束时记录偏置。
     * @return [en] the forced current and angle, as the aligner interface expects.
     * @return [zh] 强制电流与角度，按对齐器接口的约定给出。
     */
    [[nodiscard]] auto step(float dt_s, float angle_mech_rad) noexcept -> AlignOutput;

    /**
     * @brief [en] Back to Idle.
     * @brief [zh] 回到 Idle。
     */
    void reset() noexcept;

    /**
     * @brief [en] Current phase.
     * @brief [zh] 当前阶段。
     */
    [[nodiscard]] auto state() const noexcept -> StartupState { return state_; }

    /**
     * @brief [en] Mechanical offset captured at the end of the hold, rad.
     * @brief [zh] 保持结束时捕获的机械偏置，rad。
     */
    [[nodiscard]] auto offset_mech_rad() const noexcept -> float { return offset_mech_; }

    /**
     * @brief [en] Speed the forced vector is currently at, electrical rad/s.
     * @brief [zh] 强制矢量当前的转速，电角速度 rad/s。
     * @note [en] Exposed so a caller can watch the sequence without opening the routine up: `Motor::aligner()` returns
     *            a const reference, and this is the only number in it that changes during the pull.
     * @note [zh] 暴露它的目的是让调用方能观察序列而无需打开本类：`Motor::aligner()` 给出 const 引用，而拖动期间其中唯一
     *            在变的数就是它。
     */
    [[nodiscard]] auto forced_speed_rad_s() const noexcept -> float { return omega_e_; }

    /**
     * @brief [en] The angle the forced vector is at, electrical rad in [0, 2*pi).
     * @brief [zh] 强制矢量当前的角度，电角度 rad，位于 [0, 2π)。
     */
    [[nodiscard]] auto forced_angle_rad() const noexcept -> float { return theta_e_; }

  private:
    StartupConfig config_{};
    StartupState state_{StartupState::Idle};
    float elapsed_s_{0.0F};
    float theta_e_{0.0F};
    float omega_e_{0.0F};
    float offset_mech_{0.0F};
    std::uint32_t pole_pairs_{1U};
};


} // namespace core

} // namespace foc
