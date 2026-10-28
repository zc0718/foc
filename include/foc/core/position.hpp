// Conan::ImportStart
#pragma once
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] What the position slot is asked for on one step: where to go, and how hard.
 *
 *     Two limit fields instead of one target, because a *move* has a shape, not just an endpoint: a slow approach and a
 *     fast traverse of the same distance are different motions. The limits are per move, so the same controller serves
 *     both without being rebuilt.
 *
 * @brief [zh] 位置槽位在一步里被要求做什么：去哪里，以及用多大力。
 *
 *     两个限制字段而不是一个目标，因为一次**运动**有形状而不只有终点：同样距离的慢速接近 与快速穿越是两种运动。限制按次
 *     给出，所以同一个控制器服务两者而不必重建。
 *
 * @note [en] `0` means "no override, use the controller's own configured value". That is the same rule the rest of the
 *            project uses: 0 is only ever a default where 0 is not a legal value. A negative limit is the caller's bug
 *            and is treated as 0 rather than as "go backwards". `kp_scale` is the compliance knob: 1 is the configured
 *            stiffness, 0.2 is a joint that yields when pushed, and 0 (the default) leaves the controller exactly as
 *            configured. It scales the *gain*, never a limit, so a compliant joint still cannot exceed the limits.
 *
 * @note [zh] `0` 表示"不覆盖，用控制器自己配置的值"。这与项目其余部分同一条规则：0 只在 0 本身 不合法的地方才表示默认
 *            值。负数限制是调用方的 bug，按 0 处理，而不是"反向运动"。
 *
 *     `kp_scale` 是柔顺旋钮：1 是配置的刚度，0.2 是被推时会退让的关节，0（默认）则完全保持 配置。它缩放的是**增益**而非
 *     限值，因此柔顺的关节仍然不能越限。
 * @since 0.1.0
 * @exporter
 */
struct PositionInput final {
    float target_rad;           ///< [en] target mechanical angle, rad. [zh] 目标机械角，rad。
    float velocity_limit_rad_s; ///< [en] per-move ceiling, 0 = own. [zh] 本次速度上限，0 = 用自身配置。
    float accel_limit_rad_s2;   ///< [en] per-move rate limit, 0 = own. [zh] 本次加速度上限，0 = 用自身配置。
    float kp_scale;             ///< [en] stiffness scale, 0 = own. [zh] 刚度缩放，0 = 用自身配置。
};


/**
 * @brief [en] The "position mode is not compiled in" placeholder for the position-loop slot.
 *
 *     It satisfies the controller shape (`step(setpoint, feedback, dt)` + `reset()`), so it can serve as the default
 *     template argument; `Motor` recognises it and answers `ErrorCode::NotImpl` instead of silently holding zero
 *     velocity. A mode that was not compiled in has to say so.
 *
 * @brief [zh] 位置环槽位的「位置模式未编入」占位实现。
 *
 *     它满足控制器形状（`step(setpoint, feedback, dt)` + `reset()`），因此可以作为默认模板 实参；`Motor` 认得它并以
 *     `ErrorCode::NotImpl` 作答，而不是静默保持零速度。没编进来的模式 必须自己说出来。
 *
 * @note [en] The slot takes `step(const PositionInput&, feedback, dt)`, not the plain controller shape: a move has a
 *            velocity and an acceleration, not only an endpoint (see `PositionInput`). Two implementations ship:
 *            `core::PositionP` (proportional, no look-ahead, overshoots when a limit binds) and `core::MotionProfile`
 *            (adds the braking curve, so a limited move stops where it was told). Damping comes from the cascade either
 *            way: the inner speed PI holds the velocity, so a P position loop closes as a first-order system and a
 *            constant load torque is rejected by the speed loop's integrator.
 * @note [zh] 槽位接收的是 `step(const PositionInput&, feedback, dt)` 而非朴素控制器形状：一次运动有 速度和加速度，不只
 *            有一个终点（见 `PositionInput`）。内置两种实现：`core::PositionP`（比例、 不向前看，限制真正生效时会冲过
 *            头）与 `core::MotionProfile`（加入制动曲线，受限的运动停在 被告知的位置）。两种情况下阻尼都来自级联：内层
 *            速度 PI 守住速度，于是 P 位置环的闭环是一阶 系统，恒定负载转矩由速度环积分器消掉。
 * @since 0.1.0
 * @exporter
 */
class NullPositionControl final {
  public:
    /**
     * @brief [en] No position control: always commands zero velocity.
     * @brief [zh] 无位置控制：恒给零速度。
     * @since 0.1.0
     */
    [[nodiscard]] auto step(const PositionInput& /*request*/, float /*feedback*/, float /*dt_s*/) noexcept -> float {
        return 0.0F;
    }

    /**
     * @brief [en] Nothing to reset.
     * @brief [zh] 没有需要复位的状态。
     * @since 0.1.0
     */
    void reset() noexcept {}
};

/**
 * @brief [en] Proportional position control: velocity demand plus both limits, without the braking curve.
 *
 *     This is the simple half of the position slot: `velocity = gain * error`, clamped to the velocity limit and
 *     rate-limited by the acceleration limit. It honours per-move limits exactly like `MotionProfile` does, so a move
 *     never silently loses its shape.
 *
 * @brief [zh] 比例位置控制：速度需求加两道限制，但**没有**制动曲线。
 *
 *     这是位置槽位的简单一半：`速度 = 增益 × 误差`，按速度上限钳位、按加速度上限限速率。 它和 `MotionProfile` 一样遵守
 *     按次给出的限制，因此一次运动不会悄悄丢掉形状。
 *
 * @note [en] What it does *not* do is look ahead. With a velocity or acceleration limit binding it overshoots by
 *            roughly the braking distance, because the proportional term only starts easing off when the error is
 *            already small -- measured at 0.108 rad on a 1 rad step in the position-step scenario. `MotionProfile`
 *            exists for exactly that reason; pick it whenever a limit will actually bind.
 * @note [zh] 它**不做**的是向前看。速度或加速度限制真正起作用时，它会冲过目标大约一个制动距离， 因为比例项要等误差已经
 *            很小才开始回落——在 1 rad 位置阶跃上实测为 0.108 rad。 `MotionProfile` 正是为此存在的；只要限制会真正生效，
 *            就用它。
 * @since 0.1.0
 * @exporter
 */
class PositionP final {
  public:
    /**
     * @brief [en] The three numbers a proportional position loop declares.
     * @brief [zh] 比例位置环需要声明的三个数。
     * @note [en] `accel_limit_rad_s2 == 0` means "rate-limit nothing", which is a legal choice for this controller (a P
     *            loop has no acceleration concept of its own). Unlike a `0` in a per-move override, it is not a
     *            fallback: it is the configuration saying so.
     * @note [zh] `accel_limit_rad_s2 == 0` 表示"不做速率限制"，对这个控制器而言是合法选择（P 环 本身没有加速度概念）。
     *            与按次覆盖里的 0 不同，这不是回退，而是配置本身的表达。
     * @since 0.1.0
     * @exporter
     */
    struct Config final {
        float gain_1_s;             ///< [en] velocity per rad of error, 1/s. [zh] 每弧度误差的速度，1/s。
        float velocity_limit_rad_s; ///< [en] own ceiling, rad/s. [zh] 自身速度上限，rad/s。
        float accel_limit_rad_s2; ///< [en] own rate limit, rad/s^2; 0 = none. [zh] 自身加速度上限，rad/s^2；0 = 不限。
    };

    PositionP() noexcept = default;

    explicit PositionP(const Config& config) noexcept;

    /**
     * @brief [en] Velocity command for this step.
     * @brief [zh] 本步的速度指令。
     * @param request [en] target plus optional per-move ceilings (0 = use the configuration).
     * @param request [zh] 目标以及可选的本次上限（0 = 用配置）。
     * @param feedback [en] measured mechanical angle in rad.
     * @param feedback [zh] 实测机械角，rad。
     * @param dt_s [en] step length in s; <= 0 holds the previous command.
     * @param dt_s [zh] 步长，s；<= 0 保持上一条指令。
     * @since 0.1.0
     */
    [[nodiscard]] auto step(const PositionInput& request, float feedback, float dt_s) noexcept -> float;

    /// [en] Forget the command. [zh] 丢弃当前指令。
    void reset() noexcept;

    /// [en] The configuration in force. [zh] 当前生效的配置。
    [[nodiscard]] auto config() const noexcept -> const Config&;

  private:
    Config config_{};
    float velocity_rad_s_{0.0F};
};

} // namespace core

} // namespace foc
