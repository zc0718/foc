// Conan::ImportStart
#pragma once

#include <stdint.h>

#include <foc/api/foc_types.h>
// Conan::ImportEnd

/**
 * @brief [en] Internal bridge between the C API (foc_api.c, compiled as C) and the C++ core (bridge.cpp). Never
 *             installed, never included by the C++ core: the dependency is one-way.
 *
 * @brief [zh] C API（foc_api.c，按 C 编译）与 C++ 核心（bridge.cpp）之间的内部桥。不安装、 不被 C++ 核心包含：依赖严格
 *             单向。
 *
 *     `foc_api.c` validates raw pointers and then hands over to these functions; everything that touches the object
 *     pool, the generation counters or the `Motor<>` instances lives on the C++ side, where the template is visible.
 *     `foc_api.c` 先校验原始指针，再交给这些函数；对象池、代际计数与 `Motor<>` 实例全部在 C++ 侧处理（那里才看得到模
 *     板）。
 * @since 0.1.0
 */

#ifdef __cplusplus
extern "C" {
#endif

foc_err_t foc_bridge_create(const foc_motor_params_t *params, foc_motor_t *out);
foc_err_t foc_bridge_create_ex(const foc_motor_params_t *params, const foc_motor_setup_t *setup, foc_motor_t *out);
foc_err_t foc_bridge_get_setup(foc_motor_t motor, foc_motor_setup_t *out);
foc_err_t foc_bridge_destroy(foc_motor_t motor);
foc_err_t foc_bridge_enable(foc_motor_t motor);
foc_err_t foc_bridge_disable(foc_motor_t motor);
foc_err_t foc_bridge_clear_fault(foc_motor_t motor);
foc_err_t foc_bridge_set_speed(foc_motor_t motor, float rad_per_s);
foc_err_t foc_bridge_set_torque(foc_motor_t motor, float iq_a);
foc_err_t foc_bridge_set_position(foc_motor_t motor, float position_rad);
foc_err_t foc_bridge_set_command(foc_motor_t motor, const foc_motor_command_t *command);
foc_err_t foc_bridge_get_status(foc_motor_t motor, foc_motor_status_t *out);
foc_err_t foc_bridge_tick(foc_motor_t motor, float dt_s);
foc_err_t foc_bridge_get_state(foc_motor_t motor, float *speed_rad_s, float *id, float *iq);

/* --- ABI 0.1.1: commissioning, protection and the reference policies --- */
/* --- ABI 0.1.1：整定、保护与参考策略 --- */

float foc_bridge_axis_deadtime_v(float deadtime_s, float switching_hz, float v_bus_v, float angle_rad);
foc_err_t foc_bridge_fw_worthwhile(const foc_motor_params_t *params, float i_max_a, foc_fw_verdict_t *out);
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

#ifdef __cplusplus
}
#endif
