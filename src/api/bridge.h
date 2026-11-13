// Conan::ImportStart
#pragma once

#include <stdint.h>

#include <foc/api/foc_types.h>
// Conan::ImportEnd

/**
 * @brief [en] Internal bridge between the C API (foc_api.c, compiled as C) and the C++ core (bridge.cpp). Never
 *             installed, never included by the C++ core: the dependency is one-way.
 *
 *   `foc_api.c` validates raw pointers and then hands over to these functions; everything that touches the object pool,
 *   the generation counters or the `Motor<>` instances lives on the C++ side, where the template is visible.
 *   `foc_api.c`
 *
 * @brief [zh] C API（foc_api.c，按 C 编译）与 C++ 核心（bridge.cpp）之间的内部桥。不安装、 不被 C++ 核心包含：依赖严格
 *
 *   单向。
 *
 *   先校验原始指针，再交给这些函数；对象池、代际计数与 `Motor<>` 实例全部在 C++ 侧处理（那里才看得到模板）。
 * @since 0.1.0
 */

#ifdef __cplusplus
extern "C" {
#endif

foc_err_t foc_bridge_create(const foc_motor_params_t *params, foc_motor_t *out);
foc_err_t foc_bridge_create_ex(const foc_motor_params_t *params, const foc_motor_setup_t *setup, foc_motor_t *out);
foc_err_t foc_bridge_create_host(const foc_motor_params_t *params, const foc_motor_setup_t *setup,
                                 const foc_hal_callbacks_t *callbacks, foc_motor_t *out);
foc_err_t foc_bridge_get_setup(foc_motor_t motor, foc_motor_setup_t *out);
foc_err_t foc_bridge_get_params(foc_motor_t motor, foc_motor_params_t *out);
foc_err_t foc_bridge_apply_params(foc_motor_t motor, const foc_motor_params_t *params, const foc_motor_setup_t *setup);
foc_err_t foc_bridge_set_observer(foc_motor_t motor, foc_observer_t observer);
foc_err_t foc_bridge_set_trace(foc_motor_t motor, foc_trace_sample_t *samples, uint32_t capacity);
uint32_t foc_bridge_trace_size(foc_motor_t motor);
uint32_t foc_bridge_trace_dropped(foc_motor_t motor);
foc_err_t foc_bridge_trace_read(foc_motor_t motor, uint32_t index, foc_trace_sample_t *out);
foc_err_t foc_bridge_trace_reset(foc_motor_t motor);
foc_err_t foc_bridge_destroy(foc_motor_t motor);
foc_err_t foc_bridge_enable(foc_motor_t motor);
foc_err_t foc_bridge_disable(foc_motor_t motor);
foc_err_t foc_bridge_clear_fault(foc_motor_t motor);
foc_err_t foc_bridge_begin_stall_start(foc_motor_t motor, const foc_stall_start_config_t *recipe);
foc_err_t foc_bridge_get_stall_start(foc_motor_t motor, foc_stall_start_status_t *out);
foc_err_t foc_bridge_cancel_stall_start(foc_motor_t motor);
foc_err_t foc_bridge_set_speed(foc_motor_t motor, float rad_per_s);
foc_err_t foc_bridge_set_torque(foc_motor_t motor, float iq_a);
foc_err_t foc_bridge_set_position(foc_motor_t motor, float position_rad);
foc_err_t foc_bridge_set_command(foc_motor_t motor, const foc_motor_command_t *command);
foc_err_t foc_bridge_get_status(foc_motor_t motor, foc_motor_status_t *out);
foc_err_t foc_bridge_get_position_absolute(foc_motor_t motor, float *position_mech_rad);
foc_err_t foc_bridge_get_position_absolute(foc_motor_t motor, float *position_mech_rad);
foc_err_t foc_bridge_tick(foc_motor_t motor, float dt_s);
foc_err_t foc_bridge_get_state(foc_motor_t motor, float *speed_rad_s, float *id, float *iq);

/* --- ABI 0.1.0: commissioning, protection and the reference policies --- */
/* --- ABI 0.1.0：整定、保护与参考策略 --- */

float foc_bridge_axis_deadtime_v(float deadtime_s, float switching_hz, float v_bus_v, float angle_rad);
foc_err_t foc_bridge_fw_worthwhile(const foc_motor_params_t *params, float i_max_a, foc_fw_verdict_t *out);
foc_err_t foc_bridge_current_pi_gains(const foc_motor_params_t *params, const foc_motor_setup_t *setup,
                                      foc_pi_gains_t *out);

foc_err_t foc_bridge_speed_pi_gains(const foc_motor_params_t *params, const foc_motor_setup_t *setup,
                                    foc_pi_gains_t *out);

foc_err_t foc_bridge_rs_estimate(const foc_motor_params_t *params, const float *v_command, const float *i_current,
                                 uint32_t count, foc_identify_result_t *out);

foc_err_t foc_bridge_flux_estimate(const foc_motor_params_t *params, const float *omega_e, const float *v_magnitude,
                                   const float *i_magnitude, uint32_t count, foc_identify_result_t *out);

foc_err_t foc_bridge_mtpa_reference(const foc_motor_params_t *params, float i_max_a, float iq_command_a, float *id_a,
                                    float *iq_a);
foc_err_t foc_bridge_board_check(const foc_board_measurements_t *measurements, const foc_board_limits_t *limits,
                                 foc_board_report_t *out);
const char *foc_bridge_board_item_name(int32_t item);
const char *foc_bridge_board_cause_name(int32_t cause);
const char *foc_bridge_fault_name(int32_t fault);
foc_err_t foc_bridge_protector_create(const foc_protection_config_t *config, foc_protector_t *out);
foc_err_t foc_bridge_protector_step(foc_protector_t protector, const foc_protection_input_t *in, float dt_s,
                                    foc_protection_output_t *out);
foc_err_t foc_bridge_protector_reset(foc_protector_t protector, int32_t cold);
foc_err_t foc_bridge_protector_destroy(foc_protector_t protector);

float foc_bridge_hfi_max_inject_hz(float pwm_freq_hz);
foc_err_t foc_bridge_hfi_size(const foc_motor_params_t *params, float pwm_freq_hz, float hf_current_a,
                              foc_hfi_sizing_t *out);
foc_err_t foc_bridge_step_metrics(const float *y, uint32_t n, float dt_s, float target, float band,
                                  foc_step_metrics_t *out);
foc_err_t foc_bridge_sinusoid_fit(const float *y, uint32_t n, float dt_s, float omega_rad_s, foc_sinusoid_fit_t *out);
foc_err_t foc_bridge_torque_map_build(const foc_motor_params_t *params, float i_max_a, float min_nm, float max_nm,
                                      foc_torque_map_t *out);
float foc_bridge_torque_estimate(const foc_torque_map_t *map, float id_a, float iq_a);
foc_err_t foc_bridge_torque_command(const foc_torque_map_t *map, float requested_nm, float id_a,
                                    foc_torque_command_t *out);
foc_err_t foc_bridge_autotune_create(const foc_autotune_config_t *config, foc_autotune_t *out);
foc_err_t foc_bridge_autotune_accumulate(foc_autotune_t handle, float dt_s, float iq_a, float omega_mech_rad_s,
                                         int32_t saturated);
foc_err_t foc_bridge_autotune_finish(foc_autotune_t handle, foc_autotune_result_t *out);
foc_err_t foc_bridge_autotune_reset(foc_autotune_t handle);
foc_err_t foc_bridge_autotune_destroy(foc_autotune_t handle);
foc_err_t foc_bridge_cogging_learn_create(const foc_cogging_learn_config_t *config, foc_cogging_learn_t *out);
foc_err_t foc_bridge_cogging_learn_accumulate(foc_cogging_learn_t handle, float theta_mech_rad, float iq_a, float id_a,
                                              float omega_mech_rad_s, int32_t saturated);
foc_err_t foc_bridge_cogging_learn_ready(foc_cogging_learn_t handle, int32_t *out_ready);
foc_err_t foc_bridge_cogging_learn_finish(foc_cogging_learn_t handle, foc_cogging_harmonic_t *out_table,
                                          int32_t capacity, foc_cogging_learn_result_t *out_result);
foc_err_t foc_bridge_cogging_learn_reset(foc_cogging_learn_t handle);
foc_err_t foc_bridge_cogging_learn_destroy(foc_cogging_learn_t handle);
uint32_t foc_bridge_reg_count(void);
int32_t foc_bridge_reg_read(foc_motor_t motor, uint32_t index, uint32_t *raw);
uint32_t foc_bridge_frame_encode(uint8_t seq, uint8_t type, const uint8_t *payload, uint8_t length, uint8_t *out,
                                 uint32_t capacity);
int32_t foc_bridge_frame_decode(const uint8_t *in, uint32_t length, foc_frame_t *out);
foc_err_t foc_bridge_supervisor_create(foc_motor_t motor, const foc_supervisor_config_t *config, foc_supervisor_t *out);
foc_err_t foc_bridge_supervisor_step(foc_supervisor_t supervisor, float dt_s);
foc_err_t foc_bridge_supervisor_status(foc_supervisor_t supervisor, foc_supervisor_status_t *out);
foc_err_t foc_bridge_supervisor_enable(foc_supervisor_t supervisor);
foc_err_t foc_bridge_supervisor_disable(foc_supervisor_t supervisor);
foc_err_t foc_bridge_supervisor_clear(foc_supervisor_t supervisor);
foc_err_t foc_bridge_supervisor_destroy(foc_supervisor_t supervisor);

foc_err_t foc_bridge_sensor_monitor_create(float max_speed_rad_s, float jump_tolerance_rad, foc_sensor_monitor_t *out);
foc_err_t foc_bridge_sensor_monitor_step(foc_sensor_monitor_t monitor, float angle_mech_rad, float speed_mech_rad_s,
                                         float dt_s, foc_sensor_verdict_t *verdict, foc_sensor_fault_t *fault);
foc_err_t foc_bridge_sensor_monitor_status(foc_sensor_monitor_t monitor, float *step_rad, float *bound_rad,
                                           uint32_t *implausible_count);
foc_err_t foc_bridge_sensor_monitor_reset(foc_sensor_monitor_t monitor);
foc_err_t foc_bridge_sensor_monitor_destroy(foc_sensor_monitor_t monitor);
foc_err_t foc_bridge_commissioned_set_encode(const foc_commissioned_set_t *set, uint8_t *buffer, uint32_t capacity,
                                             uint32_t *written);
foc_err_t foc_bridge_commissioned_set_decode(const uint8_t *buffer, uint32_t length, foc_commissioned_set_t *out,
                                             foc_set_status_t *status, uint32_t *bad_offset);
#ifdef __cplusplus
}
#endif
