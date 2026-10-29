// Conan::ImportStart
#include <foc/api/foc_api.h>

#include <math.h>
#include <stddef.h>

#include "bridge.h"
// Conan::ImportEnd

/**
 * @brief [en] The C path's own layer: raw-pointer validation and float/double entry points, all compiled as C. Anything
 *             that needs the C++ core is one call away, in bridge.h.
 * @brief [zh] C 路径自己的那一层：原始指针校验与 float/double 入口，全部按 C 编译。任何需要 C++ 核心的事情都只是
 *             bridge.h 里的一次调用。
 * @since 0.1.0
 */

foc_err_t foc_motor_create(const foc_motor_params_t *params, foc_motor_t *out) {
    if ((params == NULL) || (out == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_create(params, out);
}

foc_err_t foc_motor_create_ex(const foc_motor_params_t *params, const foc_motor_setup_t *setup, foc_motor_t *out) {
    if ((params == NULL) || (out == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_create_ex(params, setup, out);
}

foc_err_t foc_motor_create_host(const foc_motor_params_t *params, const foc_motor_setup_t *setup,
                                const foc_hal_callbacks_t *callbacks, foc_motor_t *out) {
    if ((params == NULL) || (out == NULL) || (callbacks == NULL)) {
        return FOC_ERR_PARAM;
    }
    if (callbacks->struct_size != (uint32_t)sizeof(foc_hal_callbacks_t)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_create_host(params, setup, callbacks, out);
}

foc_err_t foc_motor_get_setup(foc_motor_t motor, foc_motor_setup_t *out) {
    if ((motor == NULL) || (out == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_get_setup(motor, out);
}

foc_err_t foc_motor_set_observer(foc_motor_t motor, foc_observer_t observer) {
    if ((motor == NULL) || (observer < FOC_OBSERVER_BACK_EMF) || (observer > FOC_OBSERVER_BLENDED)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_set_observer(motor, observer);
}

foc_err_t foc_motor_set_trace(foc_motor_t motor, foc_trace_sample_t *samples, uint32_t capacity) {
    if ((motor == NULL) || ((samples == NULL) && (capacity != 0U)) || ((samples != NULL) && (capacity == 0U))) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_set_trace(motor, samples, capacity);
}

uint32_t foc_motor_trace_size(foc_motor_t motor) {
    if (motor == NULL) {
        return 0U;
    }
    return foc_bridge_trace_size(motor);
}

uint32_t foc_motor_trace_dropped(foc_motor_t motor) {
    if (motor == NULL) {
        return 0U;
    }
    return foc_bridge_trace_dropped(motor);
}

foc_err_t foc_motor_trace_read(foc_motor_t motor, uint32_t index, foc_trace_sample_t *out) {
    if ((motor == NULL) || (out == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_trace_read(motor, index, out);
}

foc_err_t foc_motor_trace_reset(foc_motor_t motor) {
    if (motor == NULL) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_trace_reset(motor);
}

foc_err_t foc_motor_destroy(foc_motor_t motor) {
    if (motor == NULL) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_destroy(motor);
}

foc_err_t foc_motor_enable(foc_motor_t motor) {
    if (motor == NULL) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_enable(motor);
}

foc_err_t foc_motor_disable(foc_motor_t motor) {
    if (motor == NULL) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_disable(motor);
}

foc_err_t foc_motor_clear_fault(foc_motor_t motor) {
    if (motor == NULL) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_clear_fault(motor);
}

foc_err_t foc_set_speed(foc_motor_t motor, float rad_per_s) {
    if ((motor == NULL) || !isfinite(rad_per_s)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_set_speed(motor, rad_per_s);
}

foc_err_t foc_set_speed_f64(foc_motor_t motor, double rad_per_s) {
    if ((motor == NULL) || !isfinite(rad_per_s)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_set_speed(motor, (float)rad_per_s);
}

foc_err_t foc_set_torque(foc_motor_t motor, float iq_a) {
    if ((motor == NULL) || !isfinite(iq_a)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_set_torque(motor, iq_a);
}

foc_err_t foc_set_torque_f64(foc_motor_t motor, double iq_a) {
    if ((motor == NULL) || !isfinite(iq_a)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_set_torque(motor, (float)iq_a);
}

foc_err_t foc_set_position(foc_motor_t motor, float position_rad) {
    if ((motor == NULL) || !isfinite(position_rad)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_set_position(motor, position_rad);
}

foc_err_t foc_set_position_f64(foc_motor_t motor, double position_rad) {
    if ((motor == NULL) || !isfinite(position_rad)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_set_position(motor, (float)position_rad);
}

foc_err_t foc_set_command(foc_motor_t motor, const foc_motor_command_t *command) {
    if ((motor == NULL) || (command == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_set_command(motor, command);
}

foc_err_t foc_motor_get_status(foc_motor_t motor, foc_motor_status_t *out) {
    if ((motor == NULL) || (out == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_get_status(motor, out);
}

foc_err_t foc_tick(foc_motor_t motor, float dt_s) {
    if ((motor == NULL) || !isfinite(dt_s) || !(dt_s > 0.0F)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_tick(motor, dt_s);
}

foc_err_t foc_get_state(foc_motor_t motor, float *speed_rad_s, float *id, float *iq) {
    if (motor == NULL) {
        return FOC_ERR_PARAM;
    }
    if ((speed_rad_s == NULL) && (id == NULL) && (iq == NULL)) {
        return FOC_ERR_PARAM; /* the precondition: at least one output */
    }
    return foc_bridge_get_state(motor, speed_rad_s, id, iq);
}

/* ------------------------------------------------------------------------------------------------
 * ABI 0.1.1: commissioning, protection and the reference policies.
 * ABI 0.1.1：整定、保护与参考策略。
 *
 * Same division of labour as the motor path: this file owns the raw-pointer and finiteness checks
 * that a C caller can trip over, and the bridge owns everything that needs the C++ core.
 * 与电机路径同样的分工：本文件负责 C 调用方会踩到的裸指针与有限性检查，桥负责一切需要 C++ 核心的东西。
 * ---------------------------------------------------------------------------------------------- */

float foc_axis_deadtime_v(float deadtime_s, float switching_hz, float v_bus_v, float angle_rad) {
    return foc_bridge_axis_deadtime_v(deadtime_s, switching_hz, v_bus_v, angle_rad);
}

foc_err_t foc_field_weakening_worthwhile(const foc_motor_params_t *params, float i_max_a, foc_fw_verdict_t *out) {
    if ((params == NULL) || (out == NULL) || !isfinite(i_max_a)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_fw_worthwhile(params, i_max_a, out);
}

foc_err_t foc_mtpa_reference(const foc_motor_params_t *params, float i_max_a, float iq_command_a, float *id_a,
                             float *iq_a) {
    if ((params == NULL) || (id_a == NULL) || (iq_a == NULL) || !isfinite(iq_command_a)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_mtpa_reference(params, i_max_a, iq_command_a, id_a, iq_a);
}

foc_err_t foc_current_pi_gains(const foc_motor_params_t *params, const foc_motor_setup_t *setup, foc_pi_gains_t *out) {
    if ((params == NULL) || (out == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_current_pi_gains(params, setup, out);
}

foc_err_t foc_speed_pi_gains(const foc_motor_params_t *params, const foc_motor_setup_t *setup, foc_pi_gains_t *out) {
    if ((params == NULL) || (out == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_speed_pi_gains(params, setup, out);
}

foc_err_t foc_rs_estimate(const foc_motor_params_t *params, const float *v_command, const float *i_current,
                          uint32_t count, foc_identify_result_t *out) {
    if ((params == NULL) || (v_command == NULL) || (i_current == NULL) || (out == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_rs_estimate(params, v_command, i_current, count, out);
}

foc_err_t foc_flux_estimate(const foc_motor_params_t *params, const float *omega_e, const float *v_magnitude,
                            const float *i_magnitude, uint32_t count, foc_identify_result_t *out) {
    if ((params == NULL) || (omega_e == NULL) || (v_magnitude == NULL) || (i_magnitude == NULL) || (out == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_flux_estimate(params, omega_e, v_magnitude, i_magnitude, count, out);
}

foc_err_t foc_board_check(const foc_board_measurements_t *measurements, const foc_board_limits_t *limits,
                          foc_board_report_t *out) {
    if ((measurements == NULL) || (limits == NULL) || (out == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_board_check(measurements, limits, out);
}

const char *foc_board_item_name(int32_t item) { return foc_bridge_board_item_name(item); }

const char *foc_board_cause_name(int32_t cause) { return foc_bridge_board_cause_name(cause); }

const char *foc_fault_name(int32_t fault) { return foc_bridge_fault_name(fault); }

foc_err_t foc_protector_create(const foc_protection_config_t *config, foc_protector_t *out) {
    if (out == NULL) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_protector_create(config, out);
}

foc_err_t foc_protector_step(foc_protector_t protector, const foc_protection_input_t *in, float dt_s,
                             foc_protection_output_t *out) {
    if ((protector == NULL) || (in == NULL) || (out == NULL) || !isfinite(dt_s) || !(dt_s > 0.0F)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_protector_step(protector, in, dt_s, out);
}

foc_err_t foc_protector_reset(foc_protector_t protector, int32_t cold) {
    if (protector == NULL) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_protector_reset(protector, cold);
}

foc_err_t foc_protector_destroy(foc_protector_t protector) { return foc_bridge_protector_destroy(protector); }

float foc_hfi_max_inject_hz(float pwm_freq_hz) { return foc_bridge_hfi_max_inject_hz(pwm_freq_hz); }

foc_err_t foc_hfi_size(const foc_motor_params_t *params, float pwm_freq_hz, float hf_current_a, foc_hfi_sizing_t *out) {
    if ((params == NULL) || (out == NULL) || !isfinite(pwm_freq_hz) || !(pwm_freq_hz > 0.0F) ||
        !isfinite(hf_current_a) || !(hf_current_a > 0.0F)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_hfi_size(params, pwm_freq_hz, hf_current_a, out);
}

foc_err_t foc_step_metrics(const float *y, uint32_t n, float dt_s, float target, float band, foc_step_metrics_t *out) {
    if ((y == NULL) || (out == NULL) || (n < 3U) || !isfinite(dt_s) || !(dt_s > 0.0F) || !isfinite(target) ||
        !isfinite(band) || !(band >= 0.0F)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_step_metrics(y, n, dt_s, target, band, out);
}

foc_err_t foc_sinusoid_fit(const float *y, uint32_t n, float dt_s, float omega_rad_s, foc_sinusoid_fit_t *out) {
    if ((y == NULL) || (out == NULL) || (n < 6U) || !isfinite(dt_s) || !(dt_s > 0.0F) || !isfinite(omega_rad_s) ||
        !(omega_rad_s > 0.0F) || !((omega_rad_s * dt_s) < 3.14159265F)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_sinusoid_fit(y, n, dt_s, omega_rad_s, out);
}
