// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] Two pi in double: a mathematical constant, not a tuned number.
 * @brief [zh] 双精度的两倍圆周率：数学常数，不是调出来的数字。
 * @note [en] Local rather than shared because the only other copy (`gear_ratio.hpp`) is a float inside a function. A
 *            double is needed here: the whole-turn part of the count is multiplied by it.
 * @note [zh] 放在本地而不是共享：另一份（ `gear_ratio.hpp`）是函数内的 float。这里需要双精度，因为计数的整圈部分要与它
 *            相乘。
 * @since 0.1.0
 */
constexpr double kTwoPiRad = 6.283185307179586476925286766559;

/**
 * @brief [en] What a step/direction interface needs to know about the axis.
 * @brief [zh] 脉冲/方向接口需要知道的轴参数。
 * @note [en] `pulses_per_rev` is the whole reason this is not a plain multiplication: with an integer count per
 *            revolution the mapping to radians is **exact**, and an encoder that reports something else (or a gearbox
 *            that does not divide) has to be refused rather than approximated.
 * @note [zh] `pulses_per_rev` 正是这里不能只是乘一下的理由：每圈整数个脉冲时，到弧度的映射是 **精确** 的；报别的数（或
 *            齿轮箱除不尽）的接口应当被拒绝，而不是被近似。
 * @since 0.1.0
 * @exporter
 */
struct StepCommandConfig final {
    /**
     * @brief [en] whole pulses per revolution, >= 1.
     * @brief [zh] 每圈整脉冲数，>= 1。
     */
    std::uint32_t pulses_per_rev;
    /**
     * @brief [en] flip the sign the direction line means.
     * @brief [zh] 反转方向线的含义。
     */
    bool invert_direction;
    /**
   * @brief [en] The largest change one control period may carry, rad; 0 disables the check.
   * @brief [zh] 一个控制周期允许的最大变化量，rad；为 0 关闭该检查。
   * @note [en] This is not a limit and it does not clamp: it is the question "is this more than one period of motion?",
   *            and the answer is reported beside an exactly-counted position rather than replacing it. Clamping here
   *            would hide a lost count behind a plausible number.
   * @note [zh] 它不是限幅、也不做钳位：它问的是“这是不是超过了一个周期的运动量？”，答案与 **精确计数** 的位置并列报出，
   *            而不是取代它。在这里钳位，会把丢掉的计数藏到一个看起来很合理的数字后面。
   */
    float max_step_rad;
};

/**
 * @brief [en] What one control period of the step interface produced.
 * @brief [zh] 脉冲接口在一个控制周期内产出的东西。
 * @since 0.1.0
 * @exporter
 */
struct StepCommandOutput final {
    /**
     * @brief [en] Ok, or Param when the configuration is unusable.
     * @brief [zh] Ok；配置不可用时 Param。
     */
    ErrorCode status;
    /**
     * @brief [en] position reference, rad.
     * @brief [zh] 位置参考，rad。
     */
    float position_rad;
    /**
     * @brief [en] signed pulse count, exact.
     * @brief [zh] 有符号脉冲计数，精确。
     */
    std::int64_t pulses;
    /**
     * @brief [en] change since the previous call.
     * @brief [zh] 相对上一次调用的变化。
     */
    float delta_rad;
    /**
     * @brief [en] `delta_rad` exceeded `max_step_rad`.
     * @brief [zh] `delta_rad` 超过 `max_step_rad`。
     */
    bool jump;
};

/**
 * @brief [en] Turn a pulse train plus a direction line into a position reference, exactly.
 *
 *   The integer count is the truth and the radians are derived from it, not the other way round. That ordering is what
 *   makes the mapping exact at every whole turn: `pulses / pulses_per_rev` is counted in revolutions and
 *   `pulses % pulses_per_rev` is the remainder inside the turn, so a full revolution adds exactly `2*pi` and never
 *   accumulates a rounding residue. Converting to float first and integrating would drift, and at a large enough count
 *   it would stop advancing at all -- the same shape as F-40 and F-105, which is why the counter here is an integer and
 *   the floating point only appears at the end.
 *
 * @brief [zh] 把脉冲串加方向线变成一个位置参考，而且是精确的。
 *
 *   整数计数是真值，弧度由它导出，而不是反过来。这个次序正是整圈处映射精确的原因： `pulses / pulses_per_rev` 数的是圈
 *   数， `pulses % pulses_per_rev` 是圈内余量，于是整圈恰好加 `2*pi`，永远不会积累舍入残差。先转成 float 再积分会漂移，
 *   计数一大还会彻底停止推进——与 F-40、F-105 同形，所以这里计数器是整数，浮点只在最后出现。
 *
 * @note [en] `jump` is a report, not a limit: an exactly counted position that moved further than one period of motion
 *            allows is information the caller needs, and clamping it would erase it.
 * @note [zh] `jump` 是报告而不是限幅：一个精确计数、但移动量超过单周期允许值的位置，是调用方需要知道的信息，钳掉它就等
 *            于抹掉它。
 * @since 0.1.0
 * @exporter
 */
class StepCommand final {
  public:
    StepCommand() noexcept = default;
    explicit StepCommand(const StepCommandConfig& config) noexcept : config_(config) {}

    [[nodiscard]] auto config() const noexcept -> const StepCommandConfig& { return config_; }

    void set_config(const StepCommandConfig& config) noexcept {
        config_ = config;
        reset();
    }

    /** @brief [en] Ok when the interface can be mapped exactly; Param otherwise.
   * [zh] 能精确映射时 Ok，否则 Param。 */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode {
        if ((config_.pulses_per_rev == 0U) || !(config_.max_step_rad >= 0.0F) || !std::isfinite(config_.max_step_rad)) {
            return ErrorCode::Param;
        }
        return ErrorCode::Ok;
    }

    void reset() noexcept {
        pulses_ = 0;
        last_rad_ = 0.0F;
    }

    /**
   * @brief [en] Advance one control period.
   * @brief [zh] 推进一个控制周期。
   * @param pulses_this_period [en] pulses counted since the previous call, signed by the interface's own direction; a
   *                                caller that only sees one edge passes `+1` or `-1`. A count rather than an edge
   *                                because a single edge per period **silently under-counts** as soon as the axis moves
   *                                faster than the control rate.
   * @param pulses_this_period [zh] 自上次调用以来数到的脉冲数，符号取接口自己的方向；只能看见一个边沿的调用方传 `+1` 或
   *                                `-1`。用计数而不是边沿，是因为每周期只取一个边沿，在轴比控制频率更快时会
   *                                **静默 少计** 。
   */
    [[nodiscard]] auto step(std::int64_t pulses_this_period) noexcept -> StepCommandOutput {
        StepCommandOutput out{ErrorCode::Ok, 0.0F, pulses_, 0.0F, false};
        if (validate() != ErrorCode::Ok) {
            out.status = ErrorCode::Param;
            return out;
        }
        const std::int64_t delta = config_.invert_direction ? -pulses_this_period : pulses_this_period;
        pulses_ += delta;
        const float now = position_rad();
        out.status = ErrorCode::Ok;
        out.position_rad = now;
        out.pulses = pulses_;
        out.delta_rad = now - last_rad_;
        out.jump = (config_.max_step_rad > 0.0F) && (std::fabs(out.delta_rad) > config_.max_step_rad);
        last_rad_ = now;
        return out;
    }

    [[nodiscard]] auto pulses() const noexcept -> std::int64_t { return pulses_; }

    /**
   * @brief [en] The position reference in rad, computed from the integer count.
   * @brief [zh] 由整数计数算出的位置参考，rad。
   * @note [en] Whole turns are counted as integers and only the remainder inside the turn is scaled, so a full
   *            revolution contributes exactly `2*pi` rather than `2*pi` plus whatever the division lost.
   * @note [zh] 整圈用整数计，只有圈内余量参与缩放，因此整圈贡献的恰好是 `2*pi`，而不是 `2*pi` 加上除法丢掉的那一点。
   */
    [[nodiscard]] auto position_rad() const noexcept -> float {
        if (config_.pulses_per_rev == 0U) {
            return 0.0F;
        }
        const auto ppr = static_cast<std::int64_t>(config_.pulses_per_rev);
        const std::int64_t turns = pulses_ / ppr;  // truncates toward zero, which is what the remainder needs
        const std::int64_t within = pulses_ % ppr; // signed remainder: -1 pulse is 0 turns and -1/ppr
        return static_cast<float>(static_cast<double>(turns) * kTwoPiRad +
                                  static_cast<double>(within) * kTwoPiRad / static_cast<double>(ppr));
    }

  private:
    StepCommandConfig config_{};
    std::int64_t pulses_{0};
    float last_rad_{0.0F};
};

/**
 * @brief [en] What an analogue input needs to know about the axis.
 * @brief [zh] 模拟量输入需要知道的轴参数。
 * @since 0.1.0
 * @exporter
 */
struct AnalogCommandConfig final {
    /**
     * @brief [en] the voltage that means full scale, > 0.
     * @brief [zh] 对应满量程的电压，> 0。
     */
    float full_scale_v;
    /**
     * @brief [en] what full scale commands, in the caller's unit.
     * @brief [zh] 满量程对应的指令值。
     */
    float full_scale_value;
    /**
   * @brief [en] The band around zero that commands nothing, V (>= 0).
   * @brief [zh] 零点附近不产生指令的死区带，V（>= 0）。
   * @note [en] It is *subtracted*, not merely gated: a band that only zeroed its inside would leave a step at its own
   *            edge, so a slowly rising input would jump from nothing to a finite command.
   * @note [zh] 它是被 **减掉** 的，不只是把内部清零：只把带内清零的写法会在带边留下一个台阶，于是缓慢上升的输入会从零直
   *            接跳到一个有限指令。
   */
    float deadband_v;
};

/**
 * @brief [en] What one control period of the analogue input produced.
 * @brief [zh] 模拟量输入在一个控制周期内产出的东西。
 * @since 0.1.0
 * @exporter
 */
struct AnalogCommandOutput final {
    ErrorCode status;
    /**
     * @brief [en] the command, in the caller's unit.
     * @brief [zh] 指令值，单位由调用方定义。
     */
    float value;
    /**
     * @brief [en] the input is inside the band, so `value` is exactly zero.
     * @brief [zh] 输入在带内， `value` 恰为零。
     */
    bool in_deadband;
};

/**
 * @brief [en] Map an analogue voltage onto a command.
 *
 *   The dead band is subtracted rather than written as a threshold that switches.
 *
 * @brief [zh] 把一个模拟电压映射成指令。
 *
 *   死区是被减掉的，而不是一个开关阈值。
 * @since 0.1.0
 * @exporter
 */
class AnalogCommand final {
  public:
    AnalogCommand() noexcept = default;
    explicit AnalogCommand(const AnalogCommandConfig& config) noexcept : config_(config) {}

    [[nodiscard]] auto config() const noexcept -> const AnalogCommandConfig& { return config_; }
    void set_config(const AnalogCommandConfig& config) noexcept { config_ = config; }

    [[nodiscard]] auto validate() const noexcept -> ErrorCode {
        if (!(config_.full_scale_v > 0.0F) || !(config_.full_scale_value > 0.0F) || !(config_.deadband_v >= 0.0F) ||
            !(config_.deadband_v < config_.full_scale_v) || !std::isfinite(config_.deadband_v)) {
            return ErrorCode::Param;
        }
        return ErrorCode::Ok;
    }

    [[nodiscard]] auto step(float volts) noexcept -> AnalogCommandOutput {
        AnalogCommandOutput out{ErrorCode::Ok, 0.0F, false};
        if (validate() != ErrorCode::Ok) {
            out.status = ErrorCode::Param;
            return out;
        }
        if (!std::isfinite(volts)) {
            out.status = ErrorCode::Param;
            return out;
        }
        const float magnitude = std::fabs(volts);
        if (magnitude <= config_.deadband_v) {
            out.in_deadband = true; // value stays exactly zero: no residue, no creep
            return out;
        }
        const float sign = (volts < 0.0F) ? -1.0F : 1.0F;
        const float span = config_.full_scale_v - config_.deadband_v;
        float scaled = (magnitude - config_.deadband_v) / span;
        if (scaled > 1.0F) {
            scaled = 1.0F; // beyond full scale the command saturates rather than
                           // exceeding the caller's unit
        }
        out.value = sign * scaled * config_.full_scale_value;
        return out;
    }

  private:
    AnalogCommandConfig config_{};
};

/**
 * @brief [en] What a pulse-width input needs to know about the axis.
 * @brief [zh] 脉宽输入需要知道的轴参数。
 * @since 0.1.0
 * @exporter
 */
struct PpmCommandConfig final {
    /**
     * @brief [en] the shortest valid frame, > 0.
     * @brief [zh] 最短有效帧，> 0。
     */
    float min_us;
    /**
     * @brief [en] the longest valid frame, > `min_us`.
     * @brief [zh] 最长有效帧，> `min_us`。
     */
    float max_us;
    /**
     * @brief [en] the frame that commands zero, inside `[min_us, max_us]`.
     * @brief [zh] 指令为零的帧宽，落在区间内。
     */
    float center_us;
    /**
     * @brief [en] what the frame at either end commands.
     * @brief [zh] 两端帧宽对应的指令值。
     */
    float full_scale_value;
    /**
     * @brief [en] silence after which the input is not valid, > 0.
     * @brief [zh] 多久没有新帧后输入失效，> 0。
     */
    float timeout_s;
};

/**
 * @brief [en] What one control period of the pulse-width input produced.
 * @brief [zh] 脉宽输入在一个控制周期内产出的东西。
 * @since 0.1.0
 * @exporter
 */
struct PpmCommandOutput final {
    ErrorCode status;
    /**
     * @brief [en] the command; exactly zero whenever `valid` is false.
     * @brief [zh] 指令值； `valid` 为假时恰为零。
     */
    float value;
    /**
     * @brief [en] a frame arrived recently and was inside the frame window.
     * @brief [zh] 最近收到过帧且落在有效窗口内。
     */
    bool valid;
};

/**
 * @brief [en] Map a pulse width onto a command, refusing both ends and timing silence in integer nanoseconds.
 * @brief [zh] 把脉宽映射成指令：两端之外的帧一律拒绝，静默超时用整数纳秒计。
 * @note [en] An out-of-window frame is not a small command, it is a broken link: `min_us`/`max_us` exist to detect
 *            that, so a frame outside them commands exactly zero and says `valid = false` rather than being clamped
 *            into a plausible value.
 * @note [zh] 窗口外的帧不是一个小指令，而是一条断掉的链路： `min_us`/`max_us` 存在的目的就是发现它，因此落在窗口外的帧
 *            给出恰为零的指令并说 `valid = false`，而不是被钳成一个看起来合理的值。
 * @since 0.1.0
 * @exporter
 */
class PpmCommand final {
  public:
    PpmCommand() noexcept = default;
    explicit PpmCommand(const PpmCommandConfig& config) noexcept : config_(config) {}

    [[nodiscard]] auto config() const noexcept -> const PpmCommandConfig& { return config_; }
    void set_config(const PpmCommandConfig& config) noexcept {
        config_ = config;
        silence_.reset();
    }

    [[nodiscard]] auto validate() const noexcept -> ErrorCode {
        if (!(config_.min_us > 0.0F) || !(config_.max_us > config_.min_us) || !(config_.center_us >= config_.min_us) ||
            !(config_.center_us <= config_.max_us) || !(config_.full_scale_value > 0.0F) ||
            !(config_.timeout_s > 0.0F)) {
            return ErrorCode::Param;
        }
        return ErrorCode::Ok;
    }

    void reset() noexcept { silence_.reset(); }

    /**
   * @brief [en] Advance one control period with the frame width seen, or 0 for "no frame this period".
   * @brief [zh] 推进一个控制周期，传入看到的帧宽；本周期没有帧时传 0。
   */
    [[nodiscard]] auto step(float width_us, float dt_s) noexcept -> PpmCommandOutput {
        PpmCommandOutput out{ErrorCode::Ok, 0.0F, false};
        if (validate() != ErrorCode::Ok) {
            out.status = ErrorCode::Param;
            return out;
        }
        if (width_us <= 0.0F) {
            // No frame: the command decays to zero only once the silence proves the
            // link is gone. Before that the last command stands, because a frame
            // every other period is a normal 200 Hz link.
            // 没有帧：只有当静默长到能证明链路断了，指令才归零。在此之前保持上一条，因为隔周期来一帧是
            // 正常的 200 Hz 链路。
            if (dt_s > 0.0F) {
                silence_.add_sample(dt_s);
            }
            out.valid = !(silence_.seconds() >= config_.timeout_s);
            out.value = out.valid ? last_value_ : 0.0F;
            return out;
        }
        silence_.reset();
        if ((width_us < config_.min_us) || (width_us > config_.max_us)) {
            // Outside the frame window: a broken link, not a small command.
            // 窗口之外：断掉的链路，而不是一个小指令。
            last_value_ = 0.0F;
            return out;
        }
        const float span = (width_us >= config_.center_us) ? (config_.max_us - config_.center_us)
                                                           : (config_.center_us - config_.min_us);
        const float scaled = (width_us - config_.center_us) / span;
        last_value_ = scaled * config_.full_scale_value;
        out.value = last_value_;
        out.valid = true;
        return out;
    }

  private:
    PpmCommandConfig config_{};
    /**
     * @brief [en] how long since the last frame.
     * @brief [zh] 距上一帧多久。
     */
    ElapsedClock silence_;
    float last_value_{0.0F};
};

} // namespace core

} // namespace foc
