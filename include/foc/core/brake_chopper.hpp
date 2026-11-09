// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] What a brake chopper is allowed to do.
 * @brief [zh] 制动斩波器被允许做什么。
 * @note [en] `v_on > v_off` is not tidiness: the gap **is** the hysteresis, and without it the chopper switches at the
 *            converter's own sampling rate -- a pulse per control period, forever, with the resistor seeing a duty it
 *            never asked for. The minimum on and off times are the second guard, because hysteresis alone still
 *            chatters when the bus sits exactly on a threshold with ripple on top of it.
 * @note [zh] `v_on > v_off` 不是整洁问题：那道间隔**就是**滞环，没有它斩波器会以变换器自己的采样率开关——每个控制周期一
 *            个脉冲、永远如此，而电阻承受着一个没人要求的占空比。最小导通/关断时间是第二道保险，因为母线正好停在一个阈
 *            值上、上面还叠着纹波时，光有滞环仍然会抖。
 * @since 0.1.9
 * @exporter
 */
struct BrakeChopperConfig final {
    float v_on;      ///< [en] turn on above this, V. [zh] 高于此值导通，V。
    float v_off;     ///< [en] turn off below this, V. [zh] 低于此值关断，V。
    float r_ohm;     ///< [en] brake resistor, ohm, > 0. [zh] 制动电阻，Ω，> 0。
    float duty_max;  ///< [en] the largest duty the resistor is driven at, (0, 1]. [zh] 驱动电阻的最大占空比，(0, 1]。
    float min_on_s;  ///< [en] shortest on-time, s. [zh] 最短导通时间，s。
    float min_off_s; ///< [en] shortest off-time, s. [zh] 最短关断时间，s。
    float
        i2t_limit_a2s; ///< [en] the resistor's thermal budget, A^2*s; `<= 0` = no budget. [zh] 电阻的热预算，A²·s；`<= 0` 表示没有预算。
    float
        i2t_cool_tau_s; ///< [en] cooling time constant, s; `<= 0` = it never cools. [zh] 冷却时间常数，s；`<= 0` 表示不冷却。
};


/**
 * @brief [en] One period of brake-chopper decision.
 * @brief [zh] 制动斩波器一个周期的决定。
 * @note [en] `budget_exceeded` is a **latch**, and that is the whole safety argument: once the resistor has spent its
 *            thermal budget the chopper does not resume on the next quiet period, because the resistor is still hot and
 *            a controller that forgot that would destroy it politely, one period at a time. The latch is cleared only
 *            by `reset()`, and a caller that never resets it has a bus that will keep rising -- which is the correct,
 *            visible failure rather than a hidden one.
 * @note [zh] `budget_exceeded` 是一个**锁存**，整个安全论证就在这里：电阻把热预算花完之后，斩波器不会在下一个平静的周期
 *            自行恢复，因为电阻还是热的，而一个忘了这一点的控制器会很有礼貌地、每个周期一点地把它烧掉。锁存只能由
 *            `reset()` 清除，而一个从不清除它的调用方会得到一条持续上升的母线——那是正确的、看得见的失败，而不是看不见
 *            的。
 * @since 0.1.9
 * @exporter
 */
struct BrakeChopperOutput final {
    ErrorCode status; ///< [en] Ok, or Param for a configuration that cannot chop. [zh] Ok；配置无法斩波时为 Param。
    float duty;       ///< [en] duty applied to the resistor, 0..duty_max. [zh] 施加到电阻上的占空比，0..duty_max。
    float
        current_a; ///< [en] period-average current into the resistor, A (= duty*v/r). [zh] 流入电阻的周期平均电流，A。
    float
        i2t_a2s; ///< [en] the resistor's I2t: integral of the **mean square** current, A^2*s. [zh] 电阻的 I²t：**均方**电流的积分，A²·s。
    bool on;     ///< [en] the chopper is on this period. [zh] 本周期斩波器导通。
    bool
        budget_exceeded; ///< [en] latched: the budget is spent, the duty is forced to 0. [zh] 锁存：预算已耗尽，占空比强制为 0。
};


/**
 * @brief [en] A braking chopper: hysteresis on the bus voltage, a bounded duty, and the resistor's thermal budget.
 * @brief [zh] 制动斩波器：母线电压上的滞环、有界的占空比，以及电阻的热预算。
 *
 * @details [en] The chopper exists for the one case the drive cannot avoid: a machine that is regenerating faster than
 *               the supply can absorb. There is nowhere for the energy to go, so it goes into a resistor -- on purpose,
 *               before the bus finds its own way to get rid of it.
 *
 *   Two things about the design are worth stating because both were available and would have been wrong. The duty is
 *   **bang-bang at `duty_max`**, not proportional to the overvoltage: a proportional law makes the resistor's power a
 *   function of how far the bus went over, which means the first thing it does about a fast rise is too little, and the
 *   thing it does at the threshold is nothing at all. And the budget is integrated with a **cooling** term, because a
 *   budget without cooling is a latch that fires once in the machine's life and then never again.
 *
 * @details [zh] 斩波器只为一件事存在，而那是驱动躲不开的：机器回馈得比电源能吸收的更快。能量无处可去，于是**有意地**送
 *               进一个电阻，赶在母线自己找到办法处理它之前。
 *
 *   设计上有两件事值得写下来，因为两个替代方案都唾手可得而且都是错的。占空比是 **`duty_max` 上的 bang-bang**，而不是正
 *   比于超压量：按比例律，电阻的功率取决于母线超了多少，于是它对一次快速上升的第一反应**太小**，而在阈值处的反应干脆是
 *   **零**。预算是带**冷却**项积分的，因为没有冷却的预算是一辈子只跳一次、之后再也不跳的锁存器。
 * @since 0.1.9
 * @exporter
 */
class BrakeChopper final {
  public:
    BrakeChopper() noexcept = default;
    explicit BrakeChopper(const BrakeChopperConfig& config) noexcept : config_(config) {}

    [[nodiscard]] auto config() const noexcept -> const BrakeChopperConfig& { return config_; }

    void set_config(const BrakeChopperConfig& config) noexcept {
        config_ = config;
        reset();
    }

    /**
     * @brief [en] Ok when the configuration describes a chopper that can act at all.
     * @brief [zh] 配置描述的斩波器至少能动时为 Ok。
     * @note [en] `i2t_limit_a2s <= 0` is **not** refused here: a caller may legitimately run a chopper with no thermal
     *            model (a lab rig with a huge resistor), and refusing it would push that caller into inventing a
     *            number. What is refused is a chopper that cannot act: a zero resistor, an open or inverted band, a
     *            zero duty.
     * @note [zh] 这里**不**拒绝 `i2t_limit_a2s <= 0`：调用方可以合法地跑一个没有热模型的斩波器（试验台上挂了个巨大的电
     *            阻），拒绝它只会把那个调用方逼着去编一个数。被拒绝的是**动不了**的斩波器：零电阻、敞开或反过来的区间、
     *            零占空比。
     */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode {
        const bool band_ok = (config_.v_on > config_.v_off) && (config_.v_off > 0.0F);
        const bool action_ok = (config_.r_ohm > 0.0F) && (config_.duty_max > 0.0F) && (config_.duty_max <= 1.0F);
        const bool times_ok = (config_.min_on_s >= 0.0F) && (config_.min_off_s >= 0.0F);
        if (!band_ok || !action_ok || !times_ok || !std::isfinite(config_.v_on)) {
            return ErrorCode::Param;
        }
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] Decide one period from the bus voltage.
     * @brief [zh] 由母线电压决定一个周期。
     * @param v_bus [en] measured bus voltage, V.
     * @param v_bus [zh] 实测母线电压，V。
     * @param dt_s [en] period, s; `<= 0` is refused as `Param` and leaves everything as it was.
     * @param dt_s [zh] 周期，s；`<= 0` 被拒绝为 `Param` 且什么都不改。
     * @return [en] the decision, including the resistor's heat.
     * @return [zh] 决定，包含电阻的热量。
     */
    auto step(float v_bus, float dt_s) noexcept -> BrakeChopperOutput {
        BrakeChopperOutput out{};
        out.status = validate();
        if ((out.status != ErrorCode::Ok) || !(dt_s > 0.0F) || !std::isfinite(v_bus)) {
            out.status = ErrorCode::Param;
            out.i2t_a2s = static_cast<float>(i2t_a2s_);
            out.on = on_;
            return out;
        }
        if (budget_exceeded_) {
            // The resistor is still hot: cool it, report it, and drive nothing. See the note on the latch.
            // 电阻还是热的：冷却它、报告它、不驱动任何东西。见锁存上那条注释。
            cool(dt_s);
            out.budget_exceeded = true;
            out.i2t_a2s = static_cast<float>(i2t_a2s_);
            return out;
        }

        if (on_) {
            if ((v_bus < config_.v_off) && on_clock_.reached(config_.min_on_s)) {
                on_ = false;
                off_clock_.reset();
            }
        } else {
            if ((v_bus > config_.v_on) && off_clock_.reached(config_.min_off_s)) {
                on_ = true;
                on_clock_.reset();
            }
        }
        // Integer nanosecond clocks for the dwell times: a minimum on-time is a **duration**, and a float that is
        // accumulated with a period loses the small increments once it is large (F-105) -- which for a chopper that
        // stays on for minutes means the floor stops holding exactly when it matters.
        // 驻留时间用整数纳秒时钟：最短导通时间是**时长**，而一个按周期累加的 float 在变大之后会把小增量丢掉（F-105）——
        // 对一个连续导通数分钟的斩波器，那意味着下限恰好在最要紧的时候失效。
        if (on_) {
            on_clock_.add_sample(dt_s);
        } else {
            off_clock_.add_sample(dt_s);
        }

        if (on_) {
            out.duty = config_.duty_max;
            // `current_a` is the **period average** -- what the bus actually carries out through the switch.
            // 电流是**周期平均**：母线经开关实际送出的那个量。
            out.current_a = (out.duty * v_bus) / config_.r_ohm;
            // The heat is *not* the square of that average. The switch conducts at `v/r` for a fraction `duty` of the
            // period, so the mean of the square is `duty*(v/r)^2`, which exceeds `(duty*v/r)^2` by exactly `1/duty_max`.
            // Integrating the average would let a resistor driven at 35% duty spend 2.9x the budget it was given, and
            // the number would look *conservative* while doing it -- the failure mode is invisible in the direction
            // that matters. Found by cross-checking this integral against the bus meter's own energy book in SIL, where
            // `r * i2t` and the energy the meter attributes to the resistor are two computations of one quantity.
            // 热量**不是**那个平均值的平方。开关在一个周期里只有 `duty` 的比例以 `v/r` 导通，所以平方的均值是
            // `duty*(v/r)^2`，比 `(duty*v/r)^2` 大整整 `1/duty_max`。若积的是平均值，一个按 35% 占空比驱动的电阻会花掉
            // 它预算的 2.9 倍，而那个数字看起来还是**保守**的——失效方向恰好是看不见的那一侧。这一处是在 SIL 里把该积分与母线
            // meter 自己的能量账对账时发现的：`r * i2t` 与 meter 记在电阻名下的能量是同一个量的两次计算。
            const double on_current = static_cast<double>(v_bus) / static_cast<double>(config_.r_ohm);
            i2t_a2s_ += on_current * on_current * static_cast<double>(config_.duty_max) * static_cast<double>(dt_s);
            if ((config_.i2t_limit_a2s > 0.0F) && (i2t_a2s_ > static_cast<double>(config_.i2t_limit_a2s))) {
                budget_exceeded_ = true;
                out.duty = 0.0F;
                out.current_a = 0.0F;
                on_ = false;
            }
        }
        cool(dt_s);
        out.on = on_;
        out.budget_exceeded = budget_exceeded_;
        out.i2t_a2s = static_cast<float>(i2t_a2s_);
        return out;
    }

    [[nodiscard]] auto on() const noexcept -> bool { return on_; }
    [[nodiscard]] auto i2t_a2s() const noexcept -> double { return i2t_a2s_; }
    [[nodiscard]] auto budget_exceeded() const noexcept -> bool { return budget_exceeded_; }

    /** @brief [en] Forget the heat and the latch; the next period is a first period. [zh] 清掉热量与锁存；下一周期算第一周期。 */
    void reset() noexcept {
        on_ = false;
        i2t_a2s_ = 0.0;
        budget_exceeded_ = false;
        on_clock_.reset();
        off_clock_.reset();
    }

  private:
    /**
     * @brief [en] Exponential cooling toward zero, applied whether or not the resistor is being driven.
     * @brief [zh] 指数冷却至零，无论电阻是否在被驱动都施加。
     */
    void cool(float dt_s) noexcept {
        if (config_.i2t_cool_tau_s > 0.0F) {
            i2t_a2s_ *= std::exp(-static_cast<double>(dt_s) / static_cast<double>(config_.i2t_cool_tau_s));
        }
    }

    BrakeChopperConfig config_{};
    bool on_{false};
    bool budget_exceeded_{false};
    // A `double`, not a `float`, and not an `ElapsedClock`: this is the **integral** `int i^2 dt`, not a duration, so it
    // has no business being counted in nanoseconds -- but it does need more than 24 bits of mantissa, because a slow
    // ramp (a few mA over minutes) is exactly the case a float accumulator quantises away.
    // 用 `double` 而不是 `float`，也不是 `ElapsedClock`：这是**积分** `∫i²dt` 而不是时长，所以没有理由按纳秒计数——但它确实
    // 需要超过 24 位尾数，因为一个缓慢的累积（几毫安、几分钟）正是 float 累加器会量化掉的那种情况。
    double i2t_a2s_{0.0};
    ElapsedClock on_clock_{};
    ElapsedClock off_clock_{};
};


} // namespace core

} // namespace foc
