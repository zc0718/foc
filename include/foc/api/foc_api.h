// Conan::ImportStart
#pragma once
#include <foc/api/foc_types.h>
// Conan::ImportEnd


/**
 * @brief [en] Create a motor instance from the C path.
 *        [zh] 从 C 路径创建一个电机实例。
 *
 * @param params [en] non-NULL motor parameters. [zh] 非 NULL 的电机参数。
 * @param out [en] non-NULL receiver of the new handle. [zh] 非 NULL 的新句柄接收者。
 * @return [en] FOC_OK, FOC_ERR_PARAM on a NULL argument, FOC_ERR_NO_RESOURCE when the object
 *         pool is exhausted.
 *         [zh] FOC_OK；参数为 NULL 返回 FOC_ERR_PARAM；对象池耗尽返回 FOC_ERR_NO_RESOURCE。
 * @post [en] `*out` holds a handle owned by the caller until foc_motor_destroy().
 *       [zh] `*out` 持有句柄，所有权归调用方，直到 foc_motor_destroy()。
 * @context [en] Initialisation only. [zh] 仅初始化阶段。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_create(const foc_motor_params_t *params, foc_motor_t *out);


/**
 * @brief [en] Create a motor with an explicit setup instead of the library defaults.
 *        [zh] 用显式设置而非库默认值创建电机。
 *
 * @param params [en] non-NULL machine parameters. [zh] 非 NULL 的电机参数。
 * @param setup [en] optional settings; NULL means "every library default". Fields the caller's
 *              `struct_size` does not cover keep their default.
 *              [zh] 可选设置；NULL 表示“全部使用库默认值”。调用方 `struct_size` 未覆盖的字段保持默认值。
 * @param out [en] non-NULL receiver of the new handle. [zh] 非 NULL 的新句柄接收者。
 * @return [en] FOC_OK, FOC_ERR_PARAM (NULL argument, an invalid parameter or an invalid setup
 *         field), or FOC_ERR_NO_RESOURCE when the object pool is exhausted.
 *         [zh] FOC_OK；参数为 NULL、参数或设置字段非法时 FOC_ERR_PARAM；对象池耗尽返回
 *         FOC_ERR_NO_RESOURCE。
 * @post [en] `foc_motor_get_setup()` reports exactly what was applied.
 *       [zh] `foc_motor_get_setup()` 会准确报告实际生效的设置。
 * @context [en] Initialisation only. [zh] 仅初始化阶段。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_create_ex(const foc_motor_params_t *params, const foc_motor_setup_t *setup, foc_motor_t *out);


/**
 * @brief [en] Read back the setup a motor actually runs with, defaults included.
 *        [zh] 读回电机实际生效的设置（含默认值）。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @param out [en] non-NULL receiver; the library writes the whole struct and sets `struct_size`.
 *              [zh] 非 NULL 的接收者；库写入整个结构并回填 `struct_size`。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL / stale handle or a NULL receiver.
 *         [zh] FOC_OK；句柄为 NULL/已失效或接收者为 NULL 时返回 FOC_ERR_PARAM。
 * @note [en] Exists so a caller never has to guess which default it got: the answer is observable.
 *       [zh] 存在的意义是调用方永远不必猜自己拿到了哪个默认值：答案是可观测的。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_get_setup(foc_motor_t motor, foc_motor_setup_t *out);


/**
 * @brief [en] Destroy a motor instance and invalidate its handle.
 *        [zh] 销毁电机实例并使句柄失效。
 *
 * @param motor [en] handle from foc_motor_create(); NULL is accepted and returns FOC_ERR_PARAM.
 *              [zh] 由 foc_motor_create() 得到的句柄；NULL 会被接受并返回 FOC_ERR_PARAM。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for NULL / stale handles.
 *         [zh] FOC_OK；NULL 或已失效句柄返回 FOC_ERR_PARAM。
 * @context [en] Non-real-time. [zh] 非实时上下文。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_destroy(foc_motor_t motor);


/**
 * @brief [en] Enable the output stage of a motor.
 *        [zh] 使能电机的输出级。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @return [en] FOC_OK, FOC_ERR_PARAM, or FOC_ERR_STATE for an illegal state transition.
 *         [zh] FOC_OK；FOC_ERR_PARAM；非法状态迁移返回 FOC_ERR_STATE。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_enable(foc_motor_t motor);


/**
 * @brief [en] Disable the output stage of a motor.
 *        [zh] 关闭电机的输出级。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @return [en] FOC_OK or FOC_ERR_PARAM. [zh] FOC_OK 或 FOC_ERR_PARAM。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_disable(foc_motor_t motor);


/**
 * @brief [en] Clear a latched fault and return the motor to Idle.
 *        [zh] 清除锁存故障并把电机回到 Idle。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @return [en] FOC_OK when the motor was in Fault; FOC_ERR_STATE when it was not; FOC_ERR_PARAM
 *         for a NULL or stale handle.
 *         [zh] 处于 Fault 时返回 FOC_OK；不在 Fault 时返回 FOC_ERR_STATE；NULL 或失效句柄返回
 *         FOC_ERR_PARAM。
 * @note [en] A hardware fault that is still asserted re-latches on the next foc_tick(): clearing
 *       the latch never clears the inverter.
 *       [zh] 硬件故障若仍存在，下一次 foc_tick() 会立刻重新锁存：清零标志从不等于清除硬件故障。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_clear_fault(foc_motor_t motor);


/**
 * @brief [en] Set the speed setpoint.
 *        [zh] 设置速度给定。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @param rad_per_s [en] mechanical speed in rad/s. [zh] 机械速度，单位 rad/s。
 * @return [en] FOC_OK or FOC_ERR_PARAM. [zh] FOC_OK 或 FOC_ERR_PARAM。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_speed(foc_motor_t motor, float rad_per_s);


/**
 * @brief [en] Double-precision twin of foc_set_speed(), used by the `_Generic` macro
 *        `foc_set_speed_g()` when the argument is not a `float` literal.
 *        [zh] foc_set_speed() 的双精度孪生版本，由 `_Generic` 宏 `foc_set_speed_g()` 在实参不是
 *        `float` 字面量时选用。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @param rad_per_s [en] mechanical speed in rad/s; narrowed to float inside. [zh] 机械速度，rad/s；内部收窄为 float。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL handle or a non-finite value.
 *         [zh] FOC_OK；句柄为 NULL 或数值非有限返回 FOC_ERR_PARAM。
 * @note [en] The library computes in single precision: the double path avoids a lossy conversion
 *       at the *call site*, it does not raise the internal precision.
 *       [zh] 库内以单精度计算：double 通道避免的是**调用点**的隐式收窄，并不提高内部精度。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_speed_f64(foc_motor_t motor, double rad_per_s);


/**
 * @brief [en] Set the torque setpoint (q-axis current).
 *        [zh] 设置转矩给定（q 轴电流）。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @param iq_a [en] q-axis current in A. [zh] q 轴电流，单位 A。
 * @return [en] FOC_OK or FOC_ERR_PARAM. [zh] FOC_OK 或 FOC_ERR_PARAM。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_torque(foc_motor_t motor, float iq_a);


/**
 * @brief [en] Double-precision twin of foc_set_torque(), see foc_set_speed_f64().
 *        [zh] foc_set_torque() 的双精度孪生版本，说明见 foc_set_speed_f64()。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @param iq_a [en] q-axis current in A; narrowed to float inside. [zh] q 轴电流，A；内部收窄为 float。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL handle or a non-finite value.
 *         [zh] FOC_OK；句柄为 NULL 或数值非有限返回 FOC_ERR_PARAM。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_torque_f64(foc_motor_t motor, double iq_a);


/**
 * @brief [en] Set the position setpoint: mechanical angle, closed by the position loop.
 *        [zh] 设置位置给定：机械角，由位置环闭环。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @param position_rad [en] target in the frame the position sensor reports. [zh] 目标值，与位置传感器
 *                     汇报的坐标系一致。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL handle or a non-finite value. The
 *         missing-capability case (no position sensor) surfaces from foc_tick() as
 *         FOC_ERR_NOT_IMPL and is deliberately not latched.
 *         [zh] FOC_OK；句柄为 NULL 或数值非有限返回 FOC_ERR_PARAM。缺能力（没有位置传感器）的情况
 *         由 foc_tick() 以 FOC_ERR_NOT_IMPL 报出，且故意不锁存。
 * @note [en] The target is expressed in the frame the sensor reports, which is the one *after* the
 *       offset captured by alignment. Absolute multi-turn position still needs an index signal:
 *       alignment resolves the electrical offset only, and only modulo one pole pitch.
 *       [zh] 目标用传感器汇报的坐标系表达，也就是**扣除**对齐捕获的偏置之后的那一个。绝对多圈位置
 *       仍需要索引信号：对齐只解决电角度偏置，而且只能定到一个极距的模。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_position(foc_motor_t motor, float position_rad);


/**
 * @brief [en] Double-precision twin of foc_set_position(), see foc_set_speed_f64().
 *        [zh] foc_set_position() 的双精度孪生版本，说明见 foc_set_speed_f64()。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @param position_rad [en] target mechanical angle in rad; narrowed to float inside. [zh] 目标机械角，
 *                     rad；内部收窄为 float。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL handle or a non-finite value. [zh] FOC_OK；句柄为
 *         NULL 或数值非有限返回 FOC_ERR_PARAM。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_position_f64(foc_motor_t motor, double position_rad);


/**
 * @brief [en] Set a full position move: target, per-move current ceiling and a feedforward torque.
 *        [zh] 设置一次完整的位置运动：目标、本次运动的电流上限、以及前馈转矩。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @param command [en] non-NULL move. A negative `max_current_a` is rejected rather than used as a
 *                ceiling: it would invert the clamp instead of bounding it.
 *                [zh] 非 NULL 的运动。负的 `max_current_a` 会被拒绝而不是当作上限使用：那会把钳位
 *                反号，而不是限住。
 * @return [en] FOC_OK, or FOC_ERR_PARAM. [zh] FOC_OK 或 FOC_ERR_PARAM。
 * @note [en] `feedforward_current_a` is where gravity or friction belongs: it is added after the
 *       speed loop, so it does not fight the integrator.
 *       [zh] 重力或摩擦项应放在 `feedforward_current_a`：它加在速度环之后，不会和积分器打架。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_command(foc_motor_t motor, const foc_motor_command_t *command);


/**
 * @brief [en] Read the motor's own account of its state: mode, sticky error, alignment result.
 *        [zh] 读回电机对自身状态的陈述：模式、粘滞错误、对齐结果。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @param out [en] non-NULL receiver; `struct_size` is filled in by the library. [zh] 非 NULL 的接收者；
 *             `struct_size` 由库回填。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL / stale handle or a NULL receiver.
 *         [zh] FOC_OK；句柄为 NULL/已失效或接收者为 NULL 时返回 FOC_ERR_PARAM。
 * @note [en] One call instead of six getters, and `struct_size` lets it grow without breaking a
 *       caller compiled against an older header.
 *       [zh] 一次调用替代六个 getter，而 `struct_size` 让它可以在不破坏旧头编译的调用方的前提下增长。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_get_status(foc_motor_t motor, foc_motor_status_t *out);


/**
 * @brief [en] Advance the control loop by one step (host / SIL driven).
 *        [zh] 推进一个控制步（host / SIL 驱动）。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @param dt_s [en] step length in s; non-positive values are rejected. [zh] 步长，单位 s；非正值被拒绝。
 * @return [en] FOC_OK, FOC_ERR_PARAM, or FOC_ERR_STATE. [zh] FOC_OK、FOC_ERR_PARAM 或 FOC_ERR_STATE。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_tick(foc_motor_t motor, float dt_s);


/**
 * @brief [en] Read back the telemetry of the last control step.
 *        [zh] 读回最近一个控制步的遥测。
 *
 * @param motor [en] valid handle. [zh] 有效句柄。
 * @param speed_rad_s [en] optional out, mechanical speed in rad/s. [zh] 可选输出，机械速度 rad/s。
 * @param id [en] optional out, d-axis current in A. [zh] 可选输出，d 轴电流 A。
 * @param iq [en] optional out, q-axis current in A. [zh] 可选输出，q 轴电流 A。
 * @return [en] FOC_OK or FOC_ERR_PARAM. [zh] FOC_OK 或 FOC_ERR_PARAM。
 * @pre [en] At least one output pointer is non-NULL. [zh] 至少一个输出指针非 NULL。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_get_state(foc_motor_t motor, float *speed_rad_s, float *id, float *iq);


/**
 * @brief [en] `_Generic` dispatch for the setpoints, so `foc_set_speed_g(m, 50)` and
 *        `foc_set_speed_g(m, 50.0f)` both work without an explicit cast. The `default:` branch
 *        routes every other arithmetic type to the double entry point, which means passing a
 *        pointer or a struct is a compile error instead of a silent conversion.
 *        [zh] 给定值的 `_Generic` 分发：`foc_set_speed_g(m, 50)` 与 `foc_set_speed_g(m, 50.0f)`
 *        都无需显式转换。`default:` 分支把其余算术类型送到 double 入口，因此传指针或结构体会
 *        直接编译失败，而不是被静默转换。
 *
 * @note [en] Only defined for C11 and later; C++ callers pick the float or the _f64 function
 *       themselves (there is no `_Generic` in C++).
 *       [zh] 只在 C11 及以后定义；C++ 调用方自己选 float 或 _f64 版本（C++ 没有 `_Generic`）。
 * @since 0.1.0
 * @exporter
 */
#if !defined(__cplusplus) && defined(__STDC_VERSION__) && (__STDC_VERSION__ >= 201112L)
#define foc_set_speed_g(motor, value)                                                                                  \
    _Generic((value), float: foc_set_speed, default: foc_set_speed_f64)((motor), (value))
#define foc_set_torque_g(motor, value)                                                                                 \
    _Generic((value), float: foc_set_torque, default: foc_set_torque_f64)((motor), (value))
#define foc_set_position_g(motor, value)                                                                               \
    _Generic((value), float: foc_set_position, default: foc_set_position_f64)((motor), (value))
#endif
