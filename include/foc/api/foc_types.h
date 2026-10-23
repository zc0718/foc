// Conan::ImportStart
#pragma once
#include <stdint.h>
// Conan::ImportEnd


/**
 * @brief [en] Opaque motor handle of the C path. Created by foc_motor_create(), invalidated by
 *        foc_motor_destroy(); a stale handle is detected through an internal generation counter.
 *        [zh] C 路径的不透明电机句柄。由 foc_motor_create() 创建、foc_motor_destroy() 失效；
 *        内部用代际计数识别失效句柄。
 *
 * @since 0.1.0
 * @exporter
 */
typedef struct foc_motor_s *foc_motor_t;


/**
 * @brief [en] Error codes of the C path. The numeric values are frozen: they mirror
 *        foc::ErrorCode one to one (see include/foc/core/types.hpp).
 *        [zh] C 路径的错误码。数值已冻结：与 foc::ErrorCode 一一对应
 *        （见 include/foc/core/types.hpp）。
 *
 * @since 0.1.0
 * @exporter
 */
// The base type stays `int`: C99 has no fixed underlying type for an enum, and the size of
// foc_err_t is part of the frozen C ABI (foc::ErrorCode mirrors these values one to one).
// 基类型保持 `int`：C99 的 enum 没有固定底层类型，且 foc_err_t 的大小属于已冻结的 C ABI
// （foc::ErrorCode 与这些数值一一对应）。
// NOLINTNEXTLINE(performance-enum-size)
typedef enum foc_err {
    FOC_OK = 0,
    FOC_ERR_PARAM = -1,
    FOC_ERR_NO_RESOURCE = -2,
    FOC_ERR_STATE = -3,
    FOC_ERR_NOT_IMPL = -4
} foc_err_t;


/**
 * @brief [en] Motor parameters, SI units, as required by foc_motor_create().
 *        [zh] 电机参数，SI 单位，供 foc_motor_create() 使用。
 *
 * @since 0.1.0
 * @exporter
 */
typedef struct foc_motor_params {
    float rs;
    float ld;
    float lq;
    float kt;
    float j;
    float b;
    uint8_t pole_pairs;
} foc_motor_params_t;
