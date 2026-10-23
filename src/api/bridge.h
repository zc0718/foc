// Conan::ImportStart
#pragma once

#include <stdint.h>

#include <foc/api/foc_types.h>
// Conan::ImportEnd

/**
 * @brief [en] Internal bridge between the C API (foc_api.c, compiled as C) and the C++ core
 *        (bridge.cpp). Never installed, never included by the C++ core: the dependency is one-way.
 *        [zh] C API（foc_api.c，按 C 编译）与 C++ 核心（bridge.cpp）之间的内部桥。不安装、
 *        不被 C++ 核心包含：依赖严格单向。
 *
 *        `foc_api.c` validates raw pointers and then hands over to these functions; everything that
 *        touches the object pool, the generation counters or the `Motor<>` instances lives on the
 *        C++ side, where the template is visible.
 *        `foc_api.c` 先校验原始指针，再交给这些函数；对象池、代际计数与 `Motor<>` 实例全部在
 *        C++ 侧处理（那里才看得到模板）。
 *
 * @since 0.1.0
 */

#ifdef __cplusplus
extern "C" {
#endif

foc_err_t foc_bridge_create(const foc_motor_params_t *params, foc_motor_t *out);
foc_err_t foc_bridge_destroy(foc_motor_t motor);
foc_err_t foc_bridge_enable(foc_motor_t motor);
foc_err_t foc_bridge_disable(foc_motor_t motor);
foc_err_t foc_bridge_set_speed(foc_motor_t motor, float rad_per_s);
foc_err_t foc_bridge_set_torque(foc_motor_t motor, float iq_a);
foc_err_t foc_bridge_tick(foc_motor_t motor, float dt_s);
foc_err_t foc_bridge_get_state(foc_motor_t motor, float *speed_rad_s, float *id, float *iq);

#ifdef __cplusplus
}
#endif
