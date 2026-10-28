// Conan::ImportStart
#pragma once
#include <foc/api/foc_types.h>
// Conan::ImportEnd


/**
 * @brief [en] Create a motor instance from the C path.
 * @brief [zh] 从 C 路径创建一个电机实例。
 * @param params [en] non-NULL motor parameters.
 * @param params [zh] 非 NULL 的电机参数。
 * @param out [en] non-NULL receiver of the new handle.
 * @param out [zh] 非 NULL 的新句柄接收者。
 * @return [en] FOC_OK, FOC_ERR_PARAM on a NULL argument, FOC_ERR_NO_RESOURCE when the object pool is exhausted.
 * @return [zh] FOC_OK；参数为 NULL 返回 FOC_ERR_PARAM；对象池耗尽返回 FOC_ERR_NO_RESOURCE。
 * @post [en] `*out` holds a handle owned by the caller until foc_motor_destroy().
 * @post [zh] `*out` 持有句柄，所有权归调用方，直到 foc_motor_destroy()。
 * @context [en] Initialisation only.
 * @context [zh] 仅初始化阶段。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_create(const foc_motor_params_t *params, foc_motor_t *out);


/**
 * @brief [en] Create a motor with an explicit setup instead of the library defaults.
 * @brief [zh] 用显式设置而非库默认值创建电机。
 * @param params [en] non-NULL machine parameters.
 * @param params [zh] 非 NULL 的电机参数。
 * @param setup [en] optional settings; NULL means "every library default". Fields the caller's `struct_size` does not
 *                   cover keep their default.
 * @param setup [zh] 可选设置；NULL 表示“全部使用库默认值”。调用方 `struct_size` 未覆盖的字段保持默认值。
 * @param out [en] non-NULL receiver of the new handle.
 * @param out [zh] 非 NULL 的新句柄接收者。
 * @return [en] FOC_OK, FOC_ERR_PARAM (NULL argument, an invalid parameter or an invalid setup field), or
 *              FOC_ERR_NO_RESOURCE when the object pool is exhausted.
 * @return [zh] FOC_OK；参数为 NULL、参数或设置字段非法时 FOC_ERR_PARAM；对象池耗尽返回 FOC_ERR_NO_RESOURCE。
 * @post [en] `foc_motor_get_setup()` reports exactly what was applied.
 * @post [zh] `foc_motor_get_setup()` 会准确报告实际生效的设置。
 * @context [en] Initialisation only.
 * @context [zh] 仅初始化阶段。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_create_ex(const foc_motor_params_t *params, const foc_motor_setup_t *setup, foc_motor_t *out);

/**
 * @brief [en] Create a motor whose measurements come from the caller and whose duties go back to it.
 * @brief [zh] 创建一个电机：测量值来自调用方，占空比送回调用方。
 * @param params [en] non-NULL machine parameters.
 * @param params [zh] 非 NULL 的电机参数。
 * @param setup [en] optional; NULL means the documented defaults.
 * @param setup [zh] 可选；NULL 表示使用文档化的默认值。
 * @param callbacks [en] non-NULL, `struct_size` filled, the four required callbacks present.
 * @param callbacks [zh] 非 NULL，`struct_size` 已填，四个必需回调齐全。
 * @param out [en] non-NULL receiver of the handle.
 * @param out [zh] 非 NULL 的句柄接收者。
 * @return [en] FOC_OK, FOC_ERR_PARAM for a bad argument or an incomplete callback set, or FOC_ERR_NO_RESOURCE when the
 *              host pool is exhausted.
 * @return [zh] FOC_OK；参数不合法或回调集不完整返回 FOC_ERR_PARAM；主控池用尽返回 FOC_ERR_NO_RESOURCE。
 * @note [en] The handle from this call is accepted by every other `foc_*` function exactly like the one from
 *            `foc_motor_create()`: one control path, two ways of feeding it measurements.
 * @note [zh] 本函数签发的句柄被所有其它 `foc_*` 函数原样接受，与 `foc_motor_create()` 的句柄无差别：一条控制路径，两种
 *            喂测量值的方式。
 * @since 0.1.2
 * @exporter
 */
foc_err_t foc_motor_create_host(const foc_motor_params_t *params, const foc_motor_setup_t *setup,
                                const foc_hal_callbacks_t *callbacks, foc_motor_t *out);


/**
 * @brief [en] Read back the setup a motor actually runs with, defaults included.
 * @brief [zh] 读回电机实际生效的设置（含默认值）。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @param out [en] non-NULL receiver; the library writes the whole struct and sets `struct_size`.
 * @param out [zh] 非 NULL 的接收者；库写入整个结构并回填 `struct_size`。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL / stale handle or a NULL receiver.
 * @return [zh] FOC_OK；句柄为 NULL/已失效或接收者为 NULL 时返回 FOC_ERR_PARAM。
 * @note [en] Exists so a caller never has to guess which default it got: the answer is observable.
 * @note [zh] 存在的意义是调用方永远不必猜自己拿到了哪个默认值：答案是可观测的。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_get_setup(foc_motor_t motor, foc_motor_setup_t *out);


/**
 * @brief [en] Hand an enabled motor over to a different observer while it is running.
 * @brief [zh] 在运行中把已使能的电机交接给另一个观测器。
 * @param motor [en] handle from foc_motor_create(); a motor in the Run state.
 * @param motor [zh] 由 foc_motor_create() 得到的句柄；须处于 Run 状态。
 * @param observer [en] the mode to hand over to, a `foc_observer_t` value.
 * @param observer [zh] 要交接过去的模式，取 `foc_observer_t` 的值。
 * @return [en] `FOC_OK` once the hand-over is done (including when the mode was already running), `FOC_ERR_STATE` when
 *              the motor is not in Run, `FOC_ERR_PARAM` when the mode is outside the enumeration or this machine cannot
 *              run it (see the admission notes on `foc_motor_setup_t`).
 * @return [zh] 交接完成（含该模式本来就在运行）返回 `FOC_OK`；电机不在 Run 状态返回 `FOC_ERR_STATE`；模式在枚举之外或这
 *              台机器跑不了该模式返回 `FOC_ERR_PARAM`（准入条件见 `foc_motor_setup_t` 的说明）。
 * @note [en] The incoming observer is seeded from the outgoing one's current estimate before the selection changes, and
 *            that is the whole reason this exists as a function. Every observer in the library except the injection one
 *            is an integrator: an observer that has not been stepped since the motor started holds whatever it was
 *            seeded with at start-up, which is not where the rotor is. Switching without that seed is how a chain steps
 *            by a quarter or half turn and looks like a convention bug. The injection observer cannot be seeded and
 *            acquires the angle itself, so handing over *to* it starts from no information -- legitimate at standstill,
 *            and reported by `foc_motor_get_status()` like any other mode.
 * @note [zh] 在改变选择之前，接手的观测器会用交出者当前的估计播种，这正是它必须是一个函数而不是一个字段的全部理由。库中
 *            除注入观测器以外都是积分器：自电机起动以来没被推进过的观测器，保留的是起动时播下的东西，而那不是转子所在的
 *            位置。少了这份种子就切换，正是一条链会跳四分之一圈或半圈、看起来像约定错误的原因。注入观测器无法被播种、自
 *            己获取角度，因此交接**给**它会从零信息开始——在静止时可以接受，并且与其他模式一样由
 *            `foc_motor_get_status()` 报出。
 * @context [en] Control context or a task that is not ticking at the same time; not ISR-safe against `foc_tick()`.
 * @context [zh] 控制上下文，或一个不与 `foc_tick()` 同时运行的任务；不可与 `foc_tick()` 并发。
 * @since 0.1.6
 * @exporter
 */
foc_err_t foc_motor_set_observer(foc_motor_t motor, foc_observer_t observer);


/**
 * @brief [en] Destroy a motor instance and invalidate its handle.
 * @brief [zh] 销毁电机实例并使句柄失效。
 * @param motor [en] handle from foc_motor_create(); NULL is accepted and returns FOC_ERR_PARAM.
 * @param motor [zh] 由 foc_motor_create() 得到的句柄；NULL 会被接受并返回 FOC_ERR_PARAM。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for NULL / stale handles.
 * @return [zh] FOC_OK；NULL 或已失效句柄返回 FOC_ERR_PARAM。
 * @context [en] Non-real-time.
 * @context [zh] 非实时上下文。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_destroy(foc_motor_t motor);


/**
 * @brief [en] Enable the output stage of a motor.
 * @brief [zh] 使能电机的输出级。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @return [en] FOC_OK, FOC_ERR_PARAM, or FOC_ERR_STATE for an illegal state transition.
 * @return [zh] FOC_OK；FOC_ERR_PARAM；非法状态迁移返回 FOC_ERR_STATE。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_enable(foc_motor_t motor);


/**
 * @brief [en] Disable the output stage of a motor.
 * @brief [zh] 关闭电机的输出级。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @return [en] FOC_OK or FOC_ERR_PARAM.
 * @return [zh] FOC_OK 或 FOC_ERR_PARAM。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_disable(foc_motor_t motor);


/**
 * @brief [en] Clear a latched fault and return the motor to Idle.
 * @brief [zh] 清除锁存故障并把电机回到 Idle。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @return [en] FOC_OK when the motor was in Fault; FOC_ERR_STATE when it was not; FOC_ERR_PARAM for a NULL or stale
 *              handle.
 * @return [zh] 处于 Fault 时返回 FOC_OK；不在 Fault 时返回 FOC_ERR_STATE；NULL 或失效句柄返回 FOC_ERR_PARAM。
 * @note [en] A hardware fault that is still asserted re-latches on the next foc_tick(): clearing the latch never clears
 *            the inverter.
 * @note [zh] 硬件故障若仍存在，下一次 foc_tick() 会立刻重新锁存：清零标志从不等于清除硬件故障。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_clear_fault(foc_motor_t motor);


/**
 * @brief [en] Set the speed setpoint.
 * @brief [zh] 设置速度给定。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @param rad_per_s [en] mechanical speed in rad/s.
 * @param rad_per_s [zh] 机械速度，单位 rad/s。
 * @return [en] FOC_OK or FOC_ERR_PARAM.
 * @return [zh] FOC_OK 或 FOC_ERR_PARAM。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_speed(foc_motor_t motor, float rad_per_s);


/**
 * @brief [en] Double-precision twin of foc_set_speed(), used by the `_Generic` macro `foc_set_speed_g()` when the
 *             argument is not a `float` literal.
 * @brief [zh] foc_set_speed() 的双精度孪生版本，由 `_Generic` 宏 `foc_set_speed_g()` 在实参不是 `float` 字面量时选用。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @param rad_per_s [en] mechanical speed in rad/s; narrowed to float inside.
 * @param rad_per_s [zh] 机械速度，rad/s；内部收窄为 float。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL handle or a non-finite value.
 * @return [zh] FOC_OK；句柄为 NULL 或数值非有限返回 FOC_ERR_PARAM。
 * @note [en] The library computes in single precision: the double path avoids a lossy conversion at the *call site*, it
 *            does not raise the internal precision.
 * @note [zh] 库内以单精度计算：double 通道避免的是**调用点**的隐式收窄，并不提高内部精度。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_speed_f64(foc_motor_t motor, double rad_per_s);


/**
 * @brief [en] Set the torque setpoint (q-axis current).
 * @brief [zh] 设置转矩给定（q 轴电流）。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @param iq_a [en] q-axis current in A.
 * @param iq_a [zh] q 轴电流，单位 A。
 * @return [en] FOC_OK or FOC_ERR_PARAM.
 * @return [zh] FOC_OK 或 FOC_ERR_PARAM。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_torque(foc_motor_t motor, float iq_a);


/**
 * @brief [en] Double-precision twin of foc_set_torque(), see foc_set_speed_f64().
 * @brief [zh] foc_set_torque() 的双精度孪生版本，说明见 foc_set_speed_f64()。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @param iq_a [en] q-axis current in A; narrowed to float inside.
 * @param iq_a [zh] q 轴电流，A；内部收窄为 float。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL handle or a non-finite value.
 * @return [zh] FOC_OK；句柄为 NULL 或数值非有限返回 FOC_ERR_PARAM。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_torque_f64(foc_motor_t motor, double iq_a);


/**
 * @brief [en] Set the position setpoint: mechanical angle, closed by the position loop.
 * @brief [zh] 设置位置给定：机械角，由位置环闭环。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @param position_rad [en] target in the frame the position sensor reports.
 * @param position_rad [zh] 目标值，与位置传感器 汇报的坐标系一致。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL handle or a non-finite value. The missing-capability case (no
 *              position sensor) surfaces from foc_tick() as FOC_ERR_NOT_IMPL and is deliberately not latched.
 * @return [zh] FOC_OK；句柄为 NULL 或数值非有限返回 FOC_ERR_PARAM。缺能力（没有位置传感器）的情况 由 foc_tick() 以
 *              FOC_ERR_NOT_IMPL 报出，且故意不锁存。
 * @note [en] The target is expressed in the frame the sensor reports, which is the one *after* the offset captured by
 *            alignment. Absolute multi-turn position still needs an index signal: alignment resolves the electrical
 *            offset only, and only modulo one pole pitch.
 * @note [zh] 目标用传感器汇报的坐标系表达，也就是**扣除**对齐捕获的偏置之后的那一个。绝对多圈位置 仍需要索引信号：对齐
 *            只解决电角度偏置，而且只能定到一个极距的模。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_position(foc_motor_t motor, float position_rad);


/**
 * @brief [en] Double-precision twin of foc_set_position(), see foc_set_speed_f64().
 * @brief [zh] foc_set_position() 的双精度孪生版本，说明见 foc_set_speed_f64()。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @param position_rad [en] target mechanical angle in rad; narrowed to float inside.
 * @param position_rad [zh] 目标机械角， rad；内部收窄为 float。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL handle or a non-finite value.
 * @return [zh] FOC_OK；句柄为 NULL 或数值非有限返回 FOC_ERR_PARAM。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_position_f64(foc_motor_t motor, double position_rad);


/**
 * @brief [en] Set a full position move: target, per-move current ceiling and a feedforward torque.
 * @brief [zh] 设置一次完整的位置运动：目标、本次运动的电流上限、以及前馈转矩。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @param command [en] non-NULL move. A negative `max_current_a` is rejected rather than used as a ceiling: it would
 *                     invert the clamp instead of bounding it.
 * @param command [zh] 非 NULL 的运动。负的 `max_current_a` 会被拒绝而不是当作上限使用：那会把钳位 反号，而不是限住。
 * @return [en] FOC_OK, or FOC_ERR_PARAM.
 * @return [zh] FOC_OK 或 FOC_ERR_PARAM。
 * @note [en] `feedforward_current_a` is where gravity or friction belongs: it is added after the speed loop, so it does
 *            not fight the integrator.
 * @note [zh] 重力或摩擦项应放在 `feedforward_current_a`：它加在速度环之后，不会和积分器打架。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_set_command(foc_motor_t motor, const foc_motor_command_t *command);


/**
 * @brief [en] Read the motor's own account of its state: mode, sticky error, alignment result.
 * @brief [zh] 读回电机对自身状态的陈述：模式、粘滞错误、对齐结果。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @param out [en] non-NULL receiver; `struct_size` is filled in by the library.
 * @param out [zh] 非 NULL 的接收者； `struct_size` 由库回填。
 * @return [en] FOC_OK, or FOC_ERR_PARAM for a NULL / stale handle or a NULL receiver.
 * @return [zh] FOC_OK；句柄为 NULL/已失效或接收者为 NULL 时返回 FOC_ERR_PARAM。
 * @note [en] One call instead of six getters, and `struct_size` lets it grow without breaking a caller compiled against
 *            an older header.
 * @note [zh] 一次调用替代六个 getter，而 `struct_size` 让它可以在不破坏旧头编译的调用方的前提下增长。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_motor_get_status(foc_motor_t motor, foc_motor_status_t *out);


/**
 * @brief [en] Advance the control loop by one step (host / SIL driven).
 * @brief [zh] 推进一个控制步（host / SIL 驱动）。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @param dt_s [en] step length in s; non-positive values are rejected.
 * @param dt_s [zh] 步长，单位 s；非正值被拒绝。
 * @return [en] FOC_OK, FOC_ERR_PARAM, or FOC_ERR_STATE.
 * @return [zh] FOC_OK、FOC_ERR_PARAM 或 FOC_ERR_STATE。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_tick(foc_motor_t motor, float dt_s);


/**
 * @brief [en] Read back the telemetry of the last control step.
 * @brief [zh] 读回最近一个控制步的遥测。
 * @param motor [en] valid handle.
 * @param motor [zh] 有效句柄。
 * @param speed_rad_s [en] optional out, mechanical speed in rad/s.
 * @param speed_rad_s [zh] 可选输出，机械速度 rad/s。
 * @param id [en] optional out, d-axis current in A.
 * @param id [zh] 可选输出，d 轴电流 A。
 * @param iq [en] optional out, q-axis current in A.
 * @param iq [zh] 可选输出，q 轴电流 A。
 * @return [en] FOC_OK or FOC_ERR_PARAM.
 * @return [zh] FOC_OK 或 FOC_ERR_PARAM。
 * @pre [en] At least one output pointer is non-NULL.
 * @pre [zh] 至少一个输出指针非 NULL。
 * @since 0.1.0
 * @exporter
 */
foc_err_t foc_get_state(foc_motor_t motor, float *speed_rad_s, float *id, float *iq);


/**
 * @brief [en] `_Generic` dispatch for the setpoints, so `foc_set_speed_g(m, 50)` and `foc_set_speed_g(m, 50.0f)` both
 *             work without an explicit cast. The `default:` branch routes every other arithmetic type to the double
 *             entry point, which means passing a pointer or a struct is a compile error instead of a silent conversion.
 * @brief [zh] 给定值的 `_Generic` 分发：`foc_set_speed_g(m, 50)` 与 `foc_set_speed_g(m, 50.0f)` 都无需显式转换。
 *             `default:` 分支把其余算术类型送到 double 入口，因此传指针或结构体会 直接编译失败，而不是被静默转换。
 * @note [en] Only defined for C11 and later; C++ callers pick the float or the _f64 function themselves (there is no
 *            `_Generic` in C++).
 * @note [zh] 只在 C11 及以后定义；C++ 调用方自己选 float 或 _f64 版本（C++ 没有 `_Generic`）。
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


/* ------------------------------------------------------------------------------------------------
 * Commissioning, protection and the reference policies (ABI 0.1.1).
 * 整定、保护与参考策略（ABI 0.1.1）。
 * ---------------------------------------------------------------------------------------------- */

/**
 * @brief [en] The dead-time voltage projected onto an axis, in volts.
 * @brief [zh] 死区伏秒投影到某一轴上的电压，V。
 * @param deadtime_s [en] the inverter's dead time, s.
 * @param deadtime_s [zh] 逆变器死区，s。
 * @param switching_hz [en] carrier frequency, Hz.
 * @param switching_hz [zh] 载波频率，Hz。
 * @param v_bus_v [en] DC link, V.
 * @param v_bus_v [zh] 母线电压，V。
 * @param angle_rad [en] the current vector's angle; 0 = d axis, pi/2 = q axis.
 * @param angle_rad [zh] 电流矢量角；0 = d 轴，π/2 = q 轴。
 * @return [en] the axis-projected voltage, V; 0 on a non-finite argument.
 * @return [zh] 轴上投影后的电压，V；参数非有限时返回 0。
 * @note [en] Exists because the per-phase number and the axis number differ by 4/3 on the d axis and 2/sqrt(3) on q,
 *            and using one for the other is a 33 % error that reads exactly like a dead-time fault on a good board.
 * @note [zh] 存在的原因是每相数与轴上数在 d 轴差 4/3、在 q 轴差 2/√3，混用是一个 33 % 的误差， 在好板子上读起来和死区故
 *            障一模一样。
 * @since 0.1.1
 * @exporter
 */
float foc_axis_deadtime_v(float deadtime_s, float switching_hz, float v_bus_v, float angle_rad);

/**
 * @brief [en] Is field weakening worth doing on this machine, from nameplate numbers alone?
 * @brief [zh] 只用铭牌数字判断：这台机器值不值得做弱磁？
 * @param params [en] non-NULL machine parameters.
 * @param params [zh] 非 NULL 的电机参数。
 * @param i_max_a [en] per-axis current limit, A.
 * @param i_max_a [zh] 每轴电流限幅，A。
 * @param out [en] non-NULL receiver; `struct_size` must be set by the caller.
 * @param out [zh] 非 NULL 接收者；`struct_size` 须由调用方填好。
 * @return [en] FOC_OK, or FOC_ERR_PARAM.
 * @return [zh] FOC_OK，或 FOC_ERR_PARAM。
 * @since 0.1.1
 * @exporter
 */
foc_err_t foc_field_weakening_worthwhile(const foc_motor_params_t *params, float i_max_a, foc_fw_verdict_t *out);

/**
 * @brief [en] The MTPA dq current reference for a q-axis command.
 * @brief [zh] 给定 q 轴指令时的 MTPA dq 电流参考。
 * @param params [en] non-NULL machine parameters.
 * @param params [zh] 非 NULL 的电机参数。
 * @param i_max_a [en] per-axis current limit, A.
 * @param i_max_a [zh] 每轴电流限幅，A。
 * @param iq_command_a [en] the q-axis command, A.
 * @param iq_command_a [zh] q 轴给定，A。
 * @param id_a [en] non-NULL receiver of the d-axis reference, A.
 * @param id_a [zh] 非 NULL 的 d 轴参考接收者，A。
 * @param iq_a [en] non-NULL receiver of the q-axis reference, A.
 * @param iq_a [zh] 非 NULL 的 q 轴参考接收者，A。
 * @return [en] FOC_OK, or FOC_ERR_PARAM.
 * @return [zh] FOC_OK，或 FOC_ERR_PARAM。
 * @note [en] On a surface machine (`Ld == Lq`) `*id_a` is exactly 0, not an epsilon: an epsilon divides by nothing
 *            meaningful.
 * @note [zh] 表贴机（`Ld == Lq`）上 `*id_a` **恰好**为 0，不是极小量：极小量除的是一个没有 意义的数。
 * @warning [en] This answers the MTPA *curve*. It takes no speed, so it cannot know what the sensorless observer can
 *               see: with `i_d = -1 A` that observer reads 0.5079 rad at 200 rad/s against its 0.15 rad ceiling, while
 *               the same current is harmless at 3200 rad/s. A control loop that will actually apply the result must go
 *               through `MtpaReference`, whose `i_d_per_speed_a_s_per_rad` refuses d-axis current below the speed where
 *               the back-EMF carries the observer.
 * @warning [zh] 这个函数回答的是 MTPA **曲线**。它没有转速入参，因此无从知道无感观测器看得见什么：`i_d = −1 A` 时 该观
 *               测器在 200 rad/s 读到 0.5079 rad，而它公开的天花板是 0.15 rad；同一个电流在 3200 rad/s 却无害。 要真正
 *               施加这个结果的闭环必须走 `MtpaReference`，它的 `i_d_per_speed_a_s_per_rad` 会在反电势撑起观测器 的转速
 *               以下拒绝 d 轴电流。
 * @since 0.1.1
 * @exporter
 */
foc_err_t foc_mtpa_reference(const foc_motor_params_t *params, float i_max_a, float iq_command_a, float *id_a,
                             float *iq_a);

/**
 * @brief [en] The current-loop PI the library would run for this setup, clamps included.
 * @brief [zh] 这个设置下库会使用的电流环 PI，含限幅。
 * @param params [en] non-NULL machine parameters.
 * @param params [zh] 非 NULL 的电机参数。
 * @param setup [en] NULL for the library defaults, otherwise `struct_size` filled.
 * @param setup [zh] NULL 表示用库默认值，否则 `struct_size` 已填。
 * @param out [en] non-NULL receiver; `struct_size` must be set by the caller.
 * @param out [zh] 非 NULL 接收者；`struct_size` 须由调用方填好。
 * @return [en] FOC_OK, or FOC_ERR_PARAM on a NULL, a wrong `struct_size`, or a non-positive parameter.
 * @return [zh] FOC_OK；NULL、`struct_size` 不对或参数非正时返回 FOC_ERR_PARAM。
 * @note [en] Same numbers the motor constructor uses, because it is the same helper. A tuning tool outside the library
 *            therefore reproduces the shipped loop instead of re-deriving it -- and a caller that finds these gains
 *            unsuitable can change the bandwidth in the setup rather than post-processing the result.
 * @note [zh] 与电机构造函数用的是同一个辅助函数，因此数字相同。库外的整定工具于是复现出厂环路，而不是另推一遍；调用方若
 *            觉得这些增益不合适，应当改设置里的带宽，而不是对结果做二次加工。
 * @warning [en] The requested bandwidth is only meaningful above `Rs / (2*Ld)`, where the proportional gain `2*w*Ld -
 *               Rs` goes to zero and is floored there. Below that floor the loop is integral-only and delivers far less
 *               than it was asked for: measured on the nominal machine (`Rs = 0.5`, `Ld = 4e-4`), 200 rad/s takes 31 ms
 *               to reach 63 % where `1/w` is 5 ms. A tuner that trusts the request blindly ships a loop six times
 *               slower than its own documentation says.
 * @warning [zh] 只有在 `Rs / (2·Ld)` 以上，所请求的带宽才有意义：那里比例增益 `2·w·Ld − Rs` 归零并被钳在下限。门槛以下
 *               环路退化为纯积分，交付的远少于所请求的：在标称电机（`Rs = 0.5`、`Ld = 4e-4`）上实测，200 rad/s 到达
 *               63 % 要 31 ms，而 `1/w` 是 5 ms。盲信请求的整定工具会交付一个比自己文档所说的慢六倍的环。
 * @since 0.1.3
 * @exporter
 */
foc_err_t foc_current_pi_gains(const foc_motor_params_t *params, const foc_motor_setup_t *setup, foc_pi_gains_t *out);

/**
 * @brief [en] The speed-loop PI the library would run for this setup, clamps included.
 * @brief [zh] 这个设置下库会使用的速度环 PI，含限幅。
 * @param params [en] non-NULL machine parameters.
 * @param params [zh] 非 NULL 的电机参数。
 * @param setup [en] NULL for the library defaults, otherwise `struct_size` filled.
 * @param setup [zh] NULL 表示用库默认值，否则 `struct_size` 已填。
 * @param out [en] non-NULL receiver; `struct_size` must be set by the caller.
 * @param out [zh] 非 NULL 接收者；`struct_size` 须由调用方填好。
 * @return [en] FOC_OK, or FOC_ERR_PARAM on a NULL, a wrong `struct_size`, or a non-positive parameter.
 * @return [zh] FOC_OK；NULL、`struct_size` 不对或参数非正时返回 FOC_ERR_PARAM。
 * @note [en] The speed loop cancels the mechanical pole `B/J`, so its gains depend on the inertia and on `kt` as well
 *            as on the requested bandwidth. Reading them here keeps a tuning tool from assuming a different convention.
 * @note [zh] 速度环抵消机械极点 `B/J`，因此其增益除了带宽还取决于惯量与 `kt`。从这里读数，整定工具就不会自作主张换一套
 *            约定。
 * @since 0.1.3
 * @exporter
 */
foc_err_t foc_speed_pi_gains(const foc_motor_params_t *params, const foc_motor_setup_t *setup, foc_pi_gains_t *out);

/**
 * @brief [en] Stator resistance and the constant voltage loss, fitted to a standstill sweep.
 * @brief [zh] 用堵转扫描拟合出的定子电阻与恒定电压损失。
 * @param params [en] non-NULL machine parameters; its current limit bounds the sweep.
 * @param params [zh] 非 NULL 的电机参数；其电流限幅约束扫描范围。
 * @param v_command [en] non-NULL array of settled voltage magnitudes, V.
 * @param v_command [zh] 非 NULL 的稳定电压幅值数组，V。
 * @param i_current [en] non-NULL array of settled current magnitudes, A, one per voltage.
 * @param i_current [zh] 非 NULL 的稳定电流幅值数组，A，与电压一一对应。
 * @param count [en] number of samples in both arrays.
 * @param count [zh] 两个数组的样本数。
 * @param out [en] non-NULL receiver; `struct_size` must be set by the caller.
 * @param out [zh] 非 NULL 接收者；`struct_size` 须由调用方填好。
 * @return [en] FOC_OK, or FOC_ERR_PARAM on a NULL, a wrong `struct_size`, or a non-positive parameter.
 * @return [zh] FOC_OK；NULL、`struct_size` 不对或参数非正时返回 FOC_ERR_PARAM。
 * @note [en] FOC_OK says the call ran, not that the fit means anything: read `valid` in the result. A sweep whose
 *            currents are all alike has no slope to find, and answering an unaudited number would be worse than
 *            answering none.
 * @note [zh] FOC_OK 只表示调用跑完了，不表示拟合有意义：请看结果里的 `valid`。电流全都相近的扫描里没有斜率可找，此时给
 *            出一个未经审查的数字比不给更糟。
 * @context [en] Commissioning stage, rotor at standstill.
 * @context [zh] 整定阶段，转子静止。
 * @since 0.1.3
 * @exporter
 */
foc_err_t foc_rs_estimate(const foc_motor_params_t *params, const float *v_command, const float *i_current,
                          uint32_t count, foc_identify_result_t *out);

/**
 * @brief [en] Rotor flux linkage and the constant measurement offset, fitted to a zero-current sweep.
 * @brief [zh] 用零电流扫描拟合出的转子磁链与恒定测量偏置。
 * @param params [en] non-NULL machine parameters.
 * @param params [zh] 非 NULL 的电机参数。
 * @param omega_e [en] non-NULL array of electrical speeds, rad/s.
 * @param omega_e [zh] 非 NULL 的电角速度数组，rad/s。
 * @param v_magnitude [en] non-NULL array of commanded voltage magnitudes, V.
 * @param v_magnitude [zh] 非 NULL 的命令电压幅值数组，V。
 * @param i_magnitude [en] non-NULL array of current magnitudes, A; samples near zero current are the informative ones.
 * @param i_magnitude [zh] 非 NULL 的电流幅值数组，A；接近零电流的样本才是有效样本。
 * @param count [en] number of samples in all three arrays.
 * @param count [zh] 三个数组的样本数。
 * @param out [en] non-NULL receiver; `struct_size` must be set by the caller.
 * @param out [zh] 非 NULL 接收者；`struct_size` 须由调用方填好。
 * @return [en] FOC_OK, or FOC_ERR_PARAM on a NULL, a wrong `struct_size`, or a non-positive parameter.
 * @return [zh] FOC_OK；NULL、`struct_size` 不对或参数非正时返回 FOC_ERR_PARAM。
 * @note [en] The fit is a straight line through voltage against speed: the slope is the flux linkage and the intercept
 *            is everything the model does not describe (inverter loss, sensor offset). Read `valid` before believing
 *            it.
 * @note [zh] 拟合是电压对转速的一条直线：斜率即磁链，截距是模型不描述的一切（逆变器损耗、传感器偏置）。相信它之前先看
 *            `valid`。
 * @context [en] Commissioning stage, machine spinning, zero current command.
 * @context [zh] 整定阶段，电机旋转，电流给定为零。
 * @since 0.1.3
 * @exporter
 */
foc_err_t foc_flux_estimate(const foc_motor_params_t *params, const float *omega_e, const float *v_magnitude,
                            const float *i_magnitude, uint32_t count, foc_identify_result_t *out);

/**
 * @brief [en] Judge a board from what was measured.
 * @brief [zh] 用测到的数判定一块板子。
 * @param measurements [en] non-NULL, `struct_size` filled.
 * @param measurements [zh] 非 NULL，`struct_size` 已填。
 * @param limits [en] non-NULL, `struct_size` filled.
 * @param limits [zh] 非 NULL，`struct_size` 已填。
 * @param out [en] non-NULL report; `struct_size` must be set by the caller.
 * @param out [zh] 非 NULL 报告；`struct_size` 须由调用方填好。
 * @return [en] FOC_OK, or FOC_ERR_PARAM on a NULL or a wrong `struct_size`.
 * @return [zh] FOC_OK；NULL 或 `struct_size` 不对时返回 FOC_ERR_PARAM。
 * @context [en] Commissioning, once per board.
 * @context [zh] 整定阶段，每块板一次。
 * @since 0.1.1
 * @exporter
 */
foc_err_t foc_board_check(const foc_board_measurements_t *measurements, const foc_board_limits_t *limits,
                          foc_board_report_t *out);

/**
 * @brief [en] A stable lowercase name for a `FOC_BOARD_ITEM_*` value.
 * @brief [zh] `FOC_BOARD_ITEM_*` 值的稳定小写名字。
 * @param item [en] the value.
 * @param item [zh] 该值。
 * @return [en] a static string, never NULL; `"?"` for an out-of-range value.
 * @return [zh] 静态字符串，不会是 NULL；越界返回 `"?"`。
 * @since 0.1.1
 * @exporter
 */
const char *foc_board_item_name(int32_t item);

/**
 * @brief [en] A stable lowercase name for a `FOC_BOARD_CAUSE_*` value.
 * @brief [zh] `FOC_BOARD_CAUSE_*` 值的稳定小写名字。
 * @param cause [en] the value.
 * @param cause [zh] 该值。
 * @return [en] a static string, never NULL; `"?"` for an out-of-range value.
 * @return [zh] 静态字符串，不会是 NULL；越界返回 `"?"`。
 * @since 0.1.1
 * @exporter
 */
const char *foc_board_cause_name(int32_t cause);

/**
 * @brief [en] A stable lowercase name for a `FOC_FAULT_*` value.
 * @brief [zh] `FOC_FAULT_*` 值的稳定小写名字。
 * @param fault [en] the value.
 * @param fault [zh] 该值。
 * @return [en] a static string, never NULL; `"?"` for an out-of-range value.
 * @return [zh] 静态字符串，不会是 NULL；越界返回 `"?"`。
 * @note [en] Paired with `foc_board_*_name()` so a log line is three calls and no switch statement in the caller's
 *            code.
 * @note [zh] 与 `foc_board_*_name()` 配对，于是一条日志就是三次调用，调用方代码里不需要 switch。
 * @since 0.1.1
 * @exporter
 */
const char *foc_fault_name(int32_t fault);

/**
 * @brief [en] Create a protector.
 * @brief [zh] 创建一个保护器。
 * @param config [en] optional; NULL means every library default.
 * @param config [zh] 可选；NULL 表示全用库默认值。
 * @param out [en] non-NULL receiver of the new handle.
 * @param out [zh] 非 NULL 的新句柄接收者。
 * @return [en] FOC_OK, FOC_ERR_PARAM, or FOC_ERR_NO_RESOURCE when the pool is exhausted.
 * @return [zh] FOC_OK、FOC_ERR_PARAM，或池耗尽时的 FOC_ERR_NO_RESOURCE。
 * @post [en] `*out` is owned by the caller until `foc_protector_destroy()`.
 * @post [zh] `*out` 归调用方所有，直到 `foc_protector_destroy()`。
 * @since 0.1.1
 * @exporter
 */
foc_err_t foc_protector_create(const foc_protection_config_t *config, foc_protector_t *out);

/**
 * @brief [en] Advance one control period.
 * @brief [zh] 推进一个控制周期。
 * @param protector [en] a live handle.
 * @param protector [zh] 有效句柄。
 * @param in [en] non-NULL measurements.
 * @param in [zh] 非 NULL 测量值。
 * @param dt_s [en] the period, s; must be finite and positive.
 * @param dt_s [zh] 周期，s；须有限且为正。
 * @param out [en] non-NULL receiver; `struct_size` must be set by the caller.
 * @param out [zh] 非 NULL 接收者；`struct_size` 须由调用方填好。
 * @return [en] FOC_OK, FOC_ERR_PARAM, or FOC_ERR_STATE on a stale handle.
 * @return [zh] FOC_OK、FOC_ERR_PARAM，或过期句柄时的 FOC_ERR_STATE。
 * @context [en] Current-loop ISR.
 * @context [zh] 电流环 ISR。
 * @since 0.1.1
 * @exporter
 */
foc_err_t foc_protector_step(foc_protector_t protector, const foc_protection_input_t *in, float dt_s,
                             foc_protection_output_t *out);

/**
 * @brief [en] Clear the faults.
 * @brief [zh] 清除故障。
 * @param protector [en] a live handle.
 * @param protector [zh] 有效句柄。
 * @param cold [en] non-zero also clears the I^2t thermal memory; zero keeps it, because a machine that just tripped is
 *                  still hot.
 * @param cold [zh] 非零同时清除 I²t 热记忆；零则保留，因为刚跳闸的机器还热着。
 * @return [en] FOC_OK or FOC_ERR_STATE.
 * @return [zh] FOC_OK 或 FOC_ERR_STATE。
 * @since 0.1.1
 * @exporter
 */
foc_err_t foc_protector_reset(foc_protector_t protector, int32_t cold);

/**
 * @brief [en] Release a protector.
 * @brief [zh] 释放一个保护器。
 * @param protector [en] the handle; NULL is a no-op.
 * @param protector [zh] 句柄；NULL 视为无事。
 * @return [en] FOC_OK or FOC_ERR_STATE on a stale handle.
 * @return [zh] FOC_OK，或过期句柄时的 FOC_ERR_STATE。
 * @since 0.1.1
 * @exporter
 */
foc_err_t foc_protector_destroy(foc_protector_t protector);

/**
 * @brief [en] The highest HFI injection carrier a given control rate can resolve, in Hz.
 * @brief [zh] 给定控制率能分辨的最高 HFI 注入载波，Hz。
 * @param pwm_freq_hz [en] the control-loop rate in Hz; 0 or negative is answered with 0.
 * @param pwm_freq_hz [zh] 控制环频率，Hz；0 或负数答 0。
 * @return [en] `pwm_freq_hz / 2.5`, the measured ceiling; 0 for a non-positive rate, which admits nothing.
 * @return [zh] `pwm_freq_hz / 2.5`，即实测天花板；非正频率返回 0，此时什么都不准入。
 * @note [en] Ask this **before** choosing `foc_motor_setup_t::inject_hz`. Past the ceiling the injection observer stops
 *            working silently -- the demodulated amplitude collapses, the lock detector drops out, the angle freezes
 *            and the speed reads zero, which a loop cannot tell from a stalled rotor. `foc_motor_create_ex()` refuses
 *            such a request outright, and `foc_motor_get_status()` reports `observer_locked` for the ones it cannot
 *            catch. The 2.5 is measured: at 20 kHz, 8000 Hz (2.50 control periods per carrier period) held in every
 *            cell of the envelope sweep, 8400 Hz (2.38) already failed at the widest demodulation cutoff, and 9200 Hz
 *            (2.17) failed at all of them.
 * @note [zh] 在选 `foc_motor_setup_t::inject_hz` **之前**问它。越过天花板后注入观测器会静默失效——解调幅值塌掉、锁定判据
 *            脱落、角度冻结、转速读零，环无法把它与堵转区分开。`foc_motor_create_ex()` 会直接拒绝这样的请求，而它没能拦
 *            下的情况由 `foc_motor_get_status()` 的 `observer_locked` 报出。 2.5 是实测的：20 kHz 下 8000 Hz（每载波周
 *            期 2.50 个控制周期）在包线扫描的每一格都站住，8400 Hz（2.38）在最宽解调截止频率上已经失败，9200 Hz（2.17）
 *            在三个上全部失败。
 * @since 0.1.7
 * @exporter
 */
float foc_hfi_max_inject_hz(float pwm_freq_hz);

/**
 * @brief [en] Size an injection configuration from the machine and the control rate.
 * @brief [zh] 由机器与控制率定容一份注入配置。
 * @param params [en] the machine; it must be salient, because HFI has nothing to measure otherwise.
 * @param params [zh] 机器；必须有凸极，否则 HFI 没有可测的东西。
 * @param pwm_freq_hz [en] the control-loop rate in Hz, > 0.
 * @param pwm_freq_hz [zh] 控制环频率，Hz，> 0。
 * @param hf_current_a [en] the HF current you are willing to inject, peak A, > 0. This is the knob that decides how
 *                          large the torque ripple and the extra losses are, so it is an argument rather than a
 *                          default.
 * @param hf_current_a [zh] 你愿意注入的高频电流，峰值 A，> 0。它是决定转矩纹波与附加损耗大小的旋钮，因此作为参数给出，
 *                          而不是藏一个默认值。
 * @param out [en] filled on success.
 * @param out [zh] 成功时填入。
 * @return [en] FOC_OK; FOC_ERR_PARAM for a NULL pointer, a non-positive rate or budget, or a surface machine.
 * @return [zh] FOC_OK；空指针、非正的频率或预算、或表贴机时返回 FOC_ERR_PARAM。
 * @note [en] The result is **admissible**: feeding it into `foc_motor_setup_t` yields a setup `foc_motor_create_ex()`
 *            accepts. That is the property the C case asserts, because a sizing tool that suggests a configuration the
 *            library then refuses would be worse than no tool. The numbers it returns are the measured ones, each
 *            documented where it is derived: the carrier is `foc_hfi_max_inject_hz(pwm_freq_hz)`, the amplitude is
 *            `hf_current_a * 2*pi*inject_hz * ld`, and the two cutoffs are 200 Hz each -- the values the SIL envelope
 *            sweep measured as good across standstill to 50 rad/s mechanical.
 * @note [zh] 结果是**可准入的**：填进 `foc_motor_setup_t` 会得到一个 `foc_motor_create_ex()` 接受的配置。这正是 C 侧用
 *            例断言的性质，因为一个建议出库随后会拒绝的配置的定容工具，比没有工具更糟。它返回的数字都是实测的，每一个都
 *            在其推导处有文档：载波是 `foc_hfi_max_inject_hz(pwm_freq_hz)`，幅值是 `hf_current_a · 2π·inject_hz · ld`，
 *            两个截止频率各 200 Hz——即 SIL 包线扫描在静止到 50 rad/s 机械范围内实测良好的那些值。
 * @since 0.1.7
 * @exporter
 */
foc_err_t foc_hfi_size(const foc_motor_params_t *params, float pwm_freq_hz, float hf_current_a, foc_hfi_sizing_t *out);
