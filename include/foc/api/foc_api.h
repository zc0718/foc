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
