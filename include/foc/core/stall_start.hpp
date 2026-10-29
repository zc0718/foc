// Conan::ImportStart
#pragma once
#include <cmath>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Where a breakaway sequence is.
 * @brief [zh] 起步序列处在哪一步。
 * @since 0.1.8
 * @exporter
 */
enum class StallStartPhase : std::uint8_t {
    Idle = 0,     ///< [en] not started, or reset. [zh] 未开始或已复位。
    Ramping = 1,  ///< [en] torque rising toward the breakaway point. [zh] 力矩朝起动点上升。
    Released = 2, ///< [en] the rotor moved; the speed loop owns the axis. [zh] 转子已动，控制权归速度环。
    Failed = 3,   ///< [en] the torque limit was reached and the rotor never moved. [zh] 力矩到顶而转子始终未动。
};


/**
 * @brief [en] What a breakaway sequence needs to know.
 * @brief [zh] 一条起步序列需要知道的东西。
 * @note [en] Every number here is physical, and the one that matters is `max_torque_nm`: a sequence whose limit is
 *            below `breakaway_nm` is not a sequence, it is a machine that cannot start, and `validate()` refuses it
 *            rather than ramping into a wall.
 * @note [zh] 这里每个数都是物理的，其中要紧的是 `max_torque_nm`：限幅低于 `breakaway_nm` 的序列不是序列，而是一台根本起
 *            不来的机器；`validate()` 会拒绝它，而不是朝墙上顶。
 * @since 0.1.8
 * @exporter
 */
struct StallStartConfig final {
    /**
     * @brief [en] stiction plus load the rotor must beat to move, N*m (> 0).
     * @brief [zh] 转子起步必须战胜的静摩擦加负载，N·m（> 0）。
     */
    float breakaway_nm;
    /**
     * @brief [en] the most the drive may command, N*m; must exceed `breakaway_nm`.
     * @brief [zh] 驱动允许下发的上限，N·m；必须大于 `breakaway_nm`。
     */
    float max_torque_nm;
    /**
     * @brief [en] how fast the torque is raised, N*m/s (> 0). Slow enough not to slam the load when it lets go, fast
     *             enough not to feel like a stall.
     * @brief [zh] 力矩上升速率， N·m/s（> 0）。慢到负载松脱时不至于猛冲，快到不像是堵转。
     */
    float ramp_nm_per_s;
    /**
     * @brief [en] mechanical speed that counts as "it moved", rad/s (> 0).
     * @brief [zh] 判定"动了"的机械转速，rad/s（> 0）。
     */
    float release_speed_rad_s;
    /**
     * @brief [en] how long the torque may sit at its limit without motion before the sequence declares failure, s (>
     *             0).
     * @brief [zh] 力矩停在限幅而转子不动多久后判定失败，s（> 0）。
     */
    float give_up_s;
};


/**
 * @brief [en] What the sequence wants this period.
 * @brief [zh] 序列本周期要什么。
 * @since 0.1.8
 * @exporter
 */
struct StallStartOutput final {
    StallStartPhase phase; ///< [en] the phase after this step. [zh] 本步之后的阶段。
    /**
     * @brief [en] Ok while running; Param when the recipe is unusable; State when not started.
     * @brief [zh] 运行中为 Ok；配方不可用时为 Param；未开始时为 State。
     */
    ErrorCode status;
    float torque_nm; ///< [en] torque to command, N*m (0 unless Ramping or Released). [zh] 要下发的力矩，N·m。
    bool limited;    ///< [en] true when the ramp has reached `max_torque_nm`. [zh] 斜坡已到 `max_torque_nm`。
};


/**
 * @brief [en] Break a stuck rotor loose on purpose, then hand it to the speed loop with the integral still clean.
 *
 * @brief [zh] 主动把卡住的转子挣脱，然后在积分仍然干净的情况下交给速度环。
 *
 *     Why this is not just "let the speed loop do it". A speed PI *does* eventually break a stuck axis loose -- its
 *     integral ramps the current until the torque passes the breakaway point -- and that is the problem: by the time it
 *     happens the integral has stored the whole windup, so the moment the rotor lets go, the stored torque drives it
 *     past the setpoint and the loop spends as long again recovering. This sequence does the same thing deliberately:
 *     it raises the torque at a known rate, reports the instant the rotor moves, and the value it reports is the
 *     handover torque -- so the speed loop starts from a known current instead of from a wound-up integrator.
 *
 *     `Failed` is a first-class answer, not an error path. A load the drive cannot move at its limit is a fact about
 *     the machine, and a sequence that ramps forever instead of saying so leaves the operator watching a silent axis.
 *
 *     为什么这不是"让速度环自己干"。速度 PI 确实**最终**能把卡住的轴弄动——它的积分把电流顶到力矩超过起动点 ——而这正是问
 *     题所在：等它发生时，积分已经存下了整个饱和量，于是转子一松脱，存下的力矩就把它推过给定，环路要用同样长的时间收回
 *     来。本序列把同一件事**有意地**做：以已知速率抬升力矩，报告转子动起来的那一刻，而它报告的数值就是交接力矩——于是速
 *     度环从一个已知电流起步，而不是从一个饱和的积分起步。
 *
 *     `Failed` 是一等答案，不是错误路径。驱动在限幅内搬不动的负载是关于这台机器的事实，而一条"永远爬升、什么也不说" 的
 *     序列会让操作者盯着一根沉默的轴。
 * @since 0.1.8
 * @exporter
 */
class StallStart final {
  public:
    StallStart() noexcept = default;
    explicit StallStart(const StallStartConfig& config) noexcept : config_(config) {}

    /**
     * @brief [en] Replace the recipe. Only meaningful before `begin()`: changing the breakaway estimate mid-pull would
     *             make `Failed` mean two different things in one run.
     * @brief [zh] 换配方。只在 `begin()` 之前有意义：爬升途中改起动点估计，会让 `Failed` 在一次运行里有两个含义。
     */
    void set_config(const StallStartConfig& config) noexcept {
        config_ = config;
        phase_ = StallStartPhase::Idle;
        torque_nm_ = 0.0F;
        hold_s_ = 0.0F;
    }

    [[nodiscard]] auto config() const noexcept -> const StallStartConfig& { return config_; }

    /**
     * @brief [en] Ok when the recipe describes a machine that can start; Param when it does not.
     * @brief [zh] 配方描述的机器起得来时为 Ok；否则 Param。
     * @note [en] The refusal that matters is `max_torque_nm <= breakaway_nm`: a sequence whose limit is under the
     *            breakaway point can only ever produce `Failed`, so accepting it would be accepting a recipe that is
     *            known in advance not to work.
     * @note [zh] 要紧的拒绝是 `max_torque_nm <= breakaway_nm`：限幅在起动点之下的序列只可能产出 `Failed`，接受它就等于
     *            接受一份**事先就知道不工作**的配方。
     */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode {
        if (!(config_.breakaway_nm > 0.0F) || !std::isfinite(config_.breakaway_nm) ||
            !(config_.max_torque_nm > config_.breakaway_nm) || !(config_.ramp_nm_per_s > 0.0F) ||
            !(config_.release_speed_rad_s > 0.0F) || !(config_.give_up_s > 0.0F)) {
            return ErrorCode::Param;
        }
        return ErrorCode::Ok;
    }

    /**
     * @brief Start a sequence. Answers false when the recipe is unusable, and leaves the phase `Idle`.
     */
    auto begin() noexcept -> bool {
        if (validate() != ErrorCode::Ok) {
            phase_ = StallStartPhase::Idle;
            return false;
        }
        phase_ = StallStartPhase::Ramping;
        torque_nm_ = 0.0F;
        hold_s_ = 0.0F;
        return true;
    }

    /**
     * @brief [en] Advance one control period and answer what torque to command.
     * @brief [zh] 推进一个控制周期，并回答要下发多少力矩。
     * @param dt_s [en] the period, s. @param dt_s
     * @param dt_s [zh] 周期，s。
     * @param speed_mech_rad_s [en] the measured mechanical speed, rad/s. @param speed_mech_rad_s
     * @param speed_mech_rad_s [zh] 实测机械转速。
     */
    [[nodiscard]] auto step(float dt_s, float speed_mech_rad_s) noexcept -> StallStartOutput {
        StallStartOutput out{StallStartPhase::Idle, ErrorCode::State, 0.0F, false};
        const ErrorCode usable = validate();
        if (usable != ErrorCode::Ok) {
            out.status = usable;
            return out;
        }
        if (phase_ == StallStartPhase::Idle) {
            return out;
        }

        const float motion = std::fabs(speed_mech_rad_s);
        if (phase_ == StallStartPhase::Ramping) {
            // Released is decided *before* the ramp moves: the instant the rotor is seen moving, the torque it moved
            // on is the handover value, and raising it further would be pushing a rotor that no longer needs pushing.
            // 先判"已松脱"再抬升：看见转子动的那一刻，让它动起来的力矩就是交接值，继续抬升等于推一个已经不需要推的转子。
            if (motion > config_.release_speed_rad_s) {
                phase_ = StallStartPhase::Released;
            } else if (dt_s > 0.0F) {
                torque_nm_ += config_.ramp_nm_per_s * dt_s;
                if (torque_nm_ >= config_.max_torque_nm) {
                    torque_nm_ = config_.max_torque_nm;
                    hold_s_ += dt_s;
                    if (hold_s_ >= config_.give_up_s) {
                        phase_ = StallStartPhase::Failed;
                    }
                } else {
                    hold_s_ = 0.0F;
                }
            }
        }

        out.phase = phase_;
        out.status = ErrorCode::Ok;
        out.torque_nm = (phase_ == StallStartPhase::Failed) ? 0.0F : torque_nm_;
        out.limited = (torque_nm_ >= config_.max_torque_nm);
        return out;
    }

    [[nodiscard]] auto phase() const noexcept -> StallStartPhase { return phase_; }
    [[nodiscard]] auto torque_nm() const noexcept -> float { return torque_nm_; }

    void reset() noexcept {
        phase_ = StallStartPhase::Idle;
        torque_nm_ = 0.0F;
        hold_s_ = 0.0F;
    }

  private:
    StallStartConfig config_{};
    StallStartPhase phase_{StallStartPhase::Idle};
    float torque_nm_{0.0F};
    float hold_s_{0.0F};
};


} // namespace core

} // namespace foc
