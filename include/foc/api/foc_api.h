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
#endif
