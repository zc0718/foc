// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/core/types.hpp>
#include <foc/hal/clock.hpp>
#include <foc/hal/current_sense.hpp>
#include <foc/hal/dc_link.hpp>
#include <foc/hal/position.hpp>
#include <foc/hal/pwm.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {

/**
 * @brief [en] The five HAL operations a host (test bench, Python, FMU) supplies as plain C calls.
 *
 *     A drive whose sensors are stubs can only be simulated against the stubs. This struct is the other half of the
 *     design: the same control loop, with its measurements coming from a caller and its duties going back to that
 *     caller. The callbacks are plain function pointers rather than virtuals because the callers that need this are a C
 *     ABI, a Python extension and an FMU -- none of which can hand a C++ object across.
 *
 *     Four callbacks are required and are checked by `valid()`: duties out, phase currents in, mechanical angle in, DC
 *     link voltage in. The rest are optional and are documented where they are declared. A NULL optional callback means
 *     "this host has no such facility", never "assume a convenient value": `pwm_fault` returning absent means no fault
 *     is reported, `velocity_rad_s` absent means the host cannot measure speed, and `now_us` absent means the caller
 *     trusts the `dt` it passes to each tick.
 *
 * @brief [zh] 主控方（测试台、Python、FMU）以纯 C 调用提供的五个 HAL 操作。
 *
 *     传感器是桩的驱动，只能对着桩仿真。这个结构是设计的另一半：同一个控制环，测量值来自调用方，占空比送回调用方。用普
 *     通函数指针而不是虚函数，是因为需要它的调用方是 C ABI、Python 扩展与 FMU——它们都无法跨边界传 C++ 对象。
 *
 *     四个回调是必需的，由 `valid()` 检查：占空比送出、相电流读入、机械角读入、母线电压读入。其余是可选的，在声明处逐个
 *     说明。可选项为 NULL 的含义是"这个主控方没有这个能力"，**不是**"假定一个方便的值"： `pwm_fault` 缺省即不报故障，
 *     `velocity_rad_s` 缺省即主控方测不到转速，`now_us` 缺省即调用方信任自己每拍传入的 `dt`。
 *
 * @note [en] `configure()` on the HAL is a no-op here: the host owns its own carrier frequency and dead time. The
 *            controller's request is a wish, not a command, once the switching hardware belongs to someone else.
 * @note [zh] 这里的 HAL `configure()` 是空操作：载波频率与死区归主控方的硬件所有。当开关硬件属于别人时，控制器的
 *            configure 只是一个愿望，不是命令。
 * @since 0.1.2
 * @exporter
 */
struct HalCallbacks final {
    /**
     * @brief [en] Opaque pointer handed back to every callback.
     * @brief [zh] 原样传回每个回调的不透明指针。
     */
    void* user;

    /**
     * @brief [en] Receive the three phase duties, each in [0, 1]. **Required.**
     * @brief [zh] 接收三相占空比，各在 [0, 1]。**必需**。
     */
    void (*pwm_write_duty)(void* user, const float duty_uvw[3]);

    /**
     * @brief [en] Phase currents in A, U/V/W. **Required.**
     * @brief [zh] 三相电流，A，U/V/W。**必需**。
     */
    void (*read_current)(void* user, float uvw_a[3]);

    /**
     * @brief [en] Mechanical angle in rad. **Required.**
     * @brief [zh] 机械角，rad。**必需**。
     */
    float (*read_angle_mech_rad)(void* user);

    /**
     * @brief [en] DC link voltage in V. **Required.**
     * @brief [zh] 母线电压，V。**必需**。
     */
    float (*read_voltage_v)(void* user);

    /**
     * @brief [en] Mechanical angular speed in rad/s; NULL = not measurable by this host.
     * @brief [zh] 机械角速度，rad/s； NULL = 本主控方测不到。
     */
    float (*read_velocity_rad_s)(void* user);

    /**
     * @brief [en] Non-zero when the inverter reports a fault; NULL = no fault path.
     * @brief [zh] 逆变器报故障时返回非零； NULL = 没有故障通道。
     */
    int32_t (*pwm_fault)(void* user);

    /**
     * @brief [en] Enable the output stage; 0 = ok. NULL = nothing to enable.
     * @brief [zh] 使能输出级；0 = 成功。NULL = 无需使能。
     */
    int32_t (*pwm_enable)(void* user);

    /**
     * @brief [en] Disable the output stage. NULL = nothing to disable.
     * @brief [zh] 关闭输出级。NULL = 无需关闭。
     */
    void (*pwm_disable)(void* user);

    /**
     * @brief [en] Monotonic microseconds; NULL = trust the `dt` given to each tick.
     * @brief [zh] 单调递增微秒；NULL = 信任每拍的 `dt`。
     */
    std::uint64_t (*now_us)(void* user);

    /**
     * @brief [en] Whether every required callback is present.
     * @brief [zh] 必需的回调是否都在。
     * @return [en] true when duties, currents, angle and bus voltage all have a callback.
     * @return [zh] 占空比、电流、角度、母线电压四个回调齐全时为 true。
     */
    [[nodiscard]] auto valid() const noexcept -> bool {
        return (pwm_write_duty != nullptr) && (read_current != nullptr) && (read_angle_mech_rad != nullptr) &&
               (read_voltage_v != nullptr);
    }
};


/**
 * @brief [en] Convert a callback's 0/-1 style code into the core error enum.
 * @brief [zh] 把回调的 0/-1 风格返回码转为核心错误枚举。
 */
[[nodiscard]] constexpr auto to_error(std::int32_t code) noexcept -> foc::ErrorCode {
    switch (code) {
    case 0:
        return foc::ErrorCode::Ok;
    case -1:
        return foc::ErrorCode::Param;
    case -2:
        return foc::ErrorCode::NoResource;
    case -3:
        return foc::ErrorCode::State;
    case -4:
        return foc::ErrorCode::NotImpl;
    default:
        return foc::ErrorCode::Fault;
    }
}


/**
 * @brief [en] PWM driven by host callbacks.
 * @brief [zh] 由主控方回调驱动的 PWM。
 * @since 0.1.2
 */
class CallbackPwm final : public hal::Pwm {
  public:
    CallbackPwm() noexcept = default;

    explicit CallbackPwm(const HalCallbacks& callbacks) noexcept : callbacks_(callbacks) {}

    void bind(const HalCallbacks& callbacks) noexcept { callbacks_ = callbacks; }

    auto configure(float /*freq_hz*/, float /*deadtime_ns*/) noexcept -> ErrorCode override {
        // The switching hardware belongs to the host; see the note on `HalCallbacks`.
        // 开关硬件归主控方所有；见 `HalCallbacks` 上的说明。
        return ErrorCode::Ok;
    }

    void write_duty(core::Uvw<float> duty) noexcept override {
        if (callbacks_.pwm_write_duty == nullptr) {
            return;
        }
        const float uvw[3] = {duty.u, duty.v, duty.w};
        callbacks_.pwm_write_duty(callbacks_.user, uvw);
    }

    auto enable() noexcept -> ErrorCode override {
        if (callbacks_.pwm_enable == nullptr) {
            return ErrorCode::Ok;
        }
        return to_error(callbacks_.pwm_enable(callbacks_.user));
    }

    void disable() noexcept override {
        if (callbacks_.pwm_disable != nullptr) {
            callbacks_.pwm_disable(callbacks_.user);
        }
    }

    [[nodiscard]] auto fault() const noexcept -> bool override {
        if (callbacks_.pwm_fault == nullptr) {
            return false;
        }
        return callbacks_.pwm_fault(callbacks_.user) != 0;
    }

  private:
    HalCallbacks callbacks_{};
};


/**
 * @brief [en] Current sense fed by a host callback.
 * @brief [zh] 由主控方回调提供读数电流采样。
 * @since 0.1.2
 */
class CallbackCurrent final : public hal::CurrentSense {
  public:
    CallbackCurrent() noexcept = default;

    explicit CallbackCurrent(const HalCallbacks& callbacks) noexcept : callbacks_(callbacks) {}

    void bind(const HalCallbacks& callbacks) noexcept { callbacks_ = callbacks; }

    auto read_uvw() noexcept -> core::Uvw<float> override {
        float uvw[3] = {0.0F, 0.0F, 0.0F};
        if (callbacks_.read_current != nullptr) {
            callbacks_.read_current(callbacks_.user, uvw);
        }
        return core::Uvw<float>{uvw[0], uvw[1], uvw[2]};
    }

    auto calibrate_offset() noexcept -> ErrorCode override {
        // Offset calibration belongs to the host's analogue front end.
        // 零漂标定属于主控方的模拟前端。
        return ErrorCode::NotImpl;
    }

    [[nodiscard]] auto sample_rate_hz() const noexcept -> float override { return sample_rate_hz_; }

    void set_sample_rate_hz(float rate_hz) noexcept { sample_rate_hz_ = rate_hz; }

  private:
    HalCallbacks callbacks_{};
    float sample_rate_hz_{0.0F};
};


/**
 * @brief [en] Position sensor fed by host callbacks.
 * @brief [zh] 由主控方回调提供读数的位置传感器。
 * @note [en] `has_sensor()` is always true: a host that supplies an angle is, by definition, supplying a measurement. A
 *            sensorless drive still hands the observer's source here -- in co-simulation the plant owns the truth.
 * @note [zh] `has_sensor()` 恒为 true：主控方给出角度，按定义就是在给出测量。无感驱动同样把观测器的源放在这里 ——联合仿
 *            真里，真值属于被控对象那一侧。
 * @since 0.1.2
 */
class CallbackPosition final : public hal::PositionSensor {
  public:
    CallbackPosition() noexcept = default;

    explicit CallbackPosition(const HalCallbacks& callbacks) noexcept : callbacks_(callbacks) {}

    void bind(const HalCallbacks& callbacks) noexcept { callbacks_ = callbacks; }

    auto angle_mech_rad() noexcept -> float override {
        if (callbacks_.read_angle_mech_rad == nullptr) {
            return 0.0F;
        }
        return callbacks_.read_angle_mech_rad(callbacks_.user);
    }

    auto velocity_rad_s() noexcept -> float override {
        if (callbacks_.read_velocity_rad_s == nullptr) {
            return 0.0F;
        }
        return callbacks_.read_velocity_rad_s(callbacks_.user);
    }

    [[nodiscard]] auto has_sensor() const noexcept -> bool override { return true; }

  private:
    HalCallbacks callbacks_{};
};


/**
 * @brief [en] DC link fed by a host callback.
 * @brief [zh] 由主控方回调提供读数的母线。
 * @since 0.1.2
 */
class CallbackDcLink final : public hal::DcLink {
  public:
    CallbackDcLink() noexcept = default;

    explicit CallbackDcLink(const HalCallbacks& callbacks) noexcept : callbacks_(callbacks) {}

    void bind(const HalCallbacks& callbacks) noexcept { callbacks_ = callbacks; }

    auto voltage_v() noexcept -> float override {
        if (callbacks_.read_voltage_v == nullptr) {
            return 0.0F;
        }
        return callbacks_.read_voltage_v(callbacks_.user);
    }

  private:
    HalCallbacks callbacks_{};
};


/**
 * @brief [en] Clock fed by a host callback.
 * @brief [zh] 由主控方回调提供时间的时钟。
 * @note [en] Without `now_us` the clock reports 0 and the control loop measures nothing: the `dt` the caller passes to
 *            each tick is then the only statement about time, which is the honest answer for a host that has no clock.
 * @note [zh] 没有 `now_us` 时时钟恒报 0，控制环也就测不到任何时间：此时调用方每拍传入的 `dt` 是唯一关于时间的陈述——对一
 *            个没有时钟的主控方来说，这就是诚实的答案。
 * @since 0.1.2
 */
class CallbackClock final : public hal::Clock {
  public:
    CallbackClock() noexcept = default;

    explicit CallbackClock(const HalCallbacks& callbacks) noexcept : callbacks_(callbacks) {}

    void bind(const HalCallbacks& callbacks) noexcept { callbacks_ = callbacks; }

    auto now_us() noexcept -> std::uint64_t override {
        if (callbacks_.now_us == nullptr) {
            return 0U;
        }
        return callbacks_.now_us(callbacks_.user);
    }

  private:
    HalCallbacks callbacks_{};
};


/**
 * @brief [en] A HAL policy whose five interfaces are supplied by a host.
 *
 *     Use it where the plant lives outside the library: a test bench, a Python model, an FMU, or a bench script driving
 *     a real inverter. The control path is identical to `StubHal` -- same `Motor` template, same ticks -- only the
 *     measurements' origin changes. That is what makes "the same loop, verified in three places" a statement about the
 *     loop rather than about the stub.
 *
 * @brief [zh] 五个接口都由主控方提供的 HAL 策略。
 *
 *     用在被控对象位于库外的场合：测试台、Python 模型、FMU，或驱动真实逆变器的台架脚本。控制路径与 `StubHal` 完全一致——
 *     同一个 `Motor` 模板、同样的 tick——只有测量值的来源不同。这正是"同一个环，在三处验证过"这句话谈的是环而不是桩的原
 *     因。
 *
 * @note [en] `bind()` may be called between ticks to point the same motor at different callbacks; it does not reset the
 *            controller state, because swapping a data source is not a reason to forget the integrator.
 * @note [zh] `bind()` 可以在 tick 之间调用，把同一个电机指向不同的回调；它不会重置控制器状态——换数据源不是忘记积分器的
 *            理由。
 * @since 0.1.2
 * @exporter
 */
struct CallbackHal final {
    using Pwm = CallbackPwm;
    using CurrentSense = CallbackCurrent;
    using Position = CallbackPosition;
    using DcLink = CallbackDcLink;
    using Clock = CallbackClock;

    CallbackPwm pwm{};
    CallbackCurrent current_sense{};
    CallbackPosition position{};
    CallbackDcLink dc_link{};
    CallbackClock clock{};

    CallbackHal() noexcept = default;

    explicit CallbackHal(const HalCallbacks& callbacks) noexcept
        : pwm(callbacks), current_sense(callbacks), position(callbacks), dc_link(callbacks), clock(callbacks) {}

    /**
     * @brief [en] Point every interface at the same callback set.
     * @brief [zh] 把所有接口指向同一组回调。
     * @param callbacks [en] must satisfy `valid()`; an invalid set is recorded as-is and rejected by the caller.
     * @param callbacks [zh] 必须满足 `valid()`；无效的集合会被原样记下，由调用方拒绝。
     */
    void bind(const HalCallbacks& callbacks) noexcept {
        pwm.bind(callbacks);
        current_sense.bind(callbacks);
        position.bind(callbacks);
        dc_link.bind(callbacks);
        clock.bind(callbacks);
    }

    /**
     * @brief [en] Nothing to reset: the state lives on the host's side.
     * @brief [zh] 没有要重置的：状态在主控方那一侧。
     */
    void reset() noexcept {}
};

} // namespace driver

} // namespace foc
