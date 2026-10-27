// Conan::ImportStart
#include <foc/api/foc_api.h>

#include <math.h>
#include <stddef.h>

#include "bridge.h"
// Conan::ImportEnd

/**
 * @brief [en] The C path's own layer: raw-pointer validation and float/double entry points, all
 *        compiled as C. Anything that needs the C++ core is one call away, in bridge.h.
 *        [zh] C 路径自己的那一层：原始指针校验与 float/double 入口，全部按 C 编译。任何需要
 *        C++ 核心的事情都只是 bridge.h 里的一次调用。
 *
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

foc_err_t foc_motor_get_setup(foc_motor_t motor, foc_motor_setup_t *out) {
    if ((motor == NULL) || (out == NULL)) {
        return FOC_ERR_PARAM;
    }
    return foc_bridge_get_setup(motor, out);
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
