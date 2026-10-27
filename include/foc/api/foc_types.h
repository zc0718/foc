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
    FOC_ERR_NOT_IMPL = -4,
    FOC_ERR_FAULT = -5 /* the inverter reported a hardware fault; clear with foc_motor_clear_fault() */
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


/**
 * @brief [en] Optional setup for the C path: every value the library would otherwise have to guess.
 *        [zh] C 路径的可选设置：不提供时库只能自己猜的那些值。
 *
 * @note [en] `struct_size` must be `sizeof(foc_motor_setup_t)`. A caller built against an older
 *       header passes a smaller value and the library only reads the fields that value covers, so
 *       growing this struct never reads past the caller's memory.
 *       [zh] `struct_size` 必须是 `sizeof(foc_motor_setup_t)`。用旧头编译的调用方会传更小的值，
 *       库只读取该值覆盖到的字段，因此本结构以后增长也绝不会读到调用方内存之外。
 *
 * @note [en] Where 0 is not a legal value (current limit, carrier, bus voltage, bandwidths), 0
 *       means "library default"; any other value is authoritative and validated, so a negative
 *       limit is a parameter error rather than a silent fallback. `deadtime_ns` and
 *       `speed_loop_period_s` take 0 literally, because 0 is a legal value for both.
 *       [zh] 在 0 不是合法取值的字段上（限流、载波、母线电压、带宽），0 表示“使用库默认值”；
 *       其余取值一律采信并校验，因此负数限流是参数错误而不是静默回落。
 *       `deadtime_ns` 与 `speed_loop_period_s` 的 0 按字面生效，因为 0 对它们都是合法取值。
 *
 * @note [en] The `align_*` fields are the exception that proves the rule: they are taken literally,
 *       and `align_current_a == 0` means "do not align" -- a legal, common choice for a machine
 *       whose offset is already known. But a *half*-configured alignment is rejected: with
 *       `align_current_a == 0` every other `align_*` field must be 0 as well, and with it non-zero
 *       `align_hold_s` and `align_tolerance_rad` become mandatory.
 *       [zh] `align_*` 字段是那条规则的例外：它们按字面生效，`align_current_a == 0` 意为“不做对齐”
 *       ——对偏置已知的电机这是一个合法且常见的选择。但**半配置**的对齐会被拒绝：`align_current_a`
 *       为 0 时其余 `align_*` 字段也必须为 0；它非 0 时 `align_hold_s` 与 `align_tolerance_rad`
 *       变成必填。
 *
 * @since 0.1.0
 * @exporter
 */
typedef struct foc_motor_setup {
    uint32_t struct_size;              /* sizeof(foc_motor_setup_t); smaller = an older header */
    float i_max_a;                     /* per-axis current limit, A; 0 = library default */
    float pwm_freq_hz;                 /* carrier frequency, Hz; 0 = library default */
    float deadtime_ns;                 /* inverter dead time, ns; taken literally */
    float v_bus_v;                     /* nominal DC link voltage, V; 0 = library default */
    float current_bandwidth_rad_s;     /* current-loop bandwidth, rad/s; 0 = library default */
    float speed_bandwidth_rad_s;       /* speed-loop bandwidth, rad/s; 0 = library default */
    float speed_loop_period_s;         /* speed-loop period, s; 0 = every tick */
    float position_bandwidth_rad_s;    /* position-loop bandwidth, rad/s; 0 = library default */
    float velocity_limit_rad_s;        /* speed setpoint ceiling for moves, rad/s; 0 = library default */
    float position_loop_period_s;      /* position-loop period, s; 0 = every tick */
    float position_accel_limit_rad_s2; /* position velocity-command rate limit, rad/s^2; 0 = none */
    float align_current_a;             /* alignment current, A; 0 = do not align */
    float align_ramp_s;                /* alignment ramp time, s; 0 = apply at once */
    float align_hold_s;                /* alignment hold window, s; required when aligning */
    float align_tolerance_rad;         /* accepted spread during the hold, rad; required when aligning */
    float align_verify_s;              /* direction-check hold, s; 0 = skip the check */
    float align_angle_e_rad;           /* forced electrical angle, rad; 0 = the usual zero */
} foc_motor_setup_t;


/**
 * @brief [en] Operating mode state, mirroring foc::core::MotorState. Values are frozen.
 *        [zh] 运行模式状态，与 foc::core::MotorState 对应。数值已冻结。
 *
 * @since 0.1.0
 * @exporter
 */
// NOLINTNEXTLINE(performance-enum-size)
typedef enum foc_motor_state {
    FOC_MOTOR_IDLE = 0, /* outputs off */
    FOC_MOTOR_ALIGN = 1,
    FOC_MOTOR_RUN = 2,
    FOC_MOTOR_FAULT = 3 /* latched; clear with foc_motor_clear_fault() */
} foc_motor_state_t;


/**
 * @brief [en] Alignment phase, mirroring foc::core::AlignState. Values are frozen.
 *        [zh] 对齐阶段，与 foc::core::AlignState 对应。数值已冻结。
 *
 * @since 0.1.0
 * @exporter
 */
// NOLINTNEXTLINE(performance-enum-size)
typedef enum foc_align_state {
    FOC_ALIGN_IDLE = 0,
    FOC_ALIGN_RAMP = 1,
    FOC_ALIGN_SETTLE = 2,
    FOC_ALIGN_VERIFY = 3,
    FOC_ALIGN_DONE = 4,
    FOC_ALIGN_FAILED = 5
} foc_align_state_t;


/**
 * @brief [en] Why an alignment sequence was rejected, mirroring foc::core::AlignFailure.
 *        [zh] 对齐流程被拒的原因，与 foc::core::AlignFailure 对应。
 *
 * @since 0.1.0
 * @exporter
 */
// NOLINTNEXTLINE(performance-enum-size)
typedef enum foc_align_failure {
    FOC_ALIGN_NO_FAILURE = 0,
    FOC_ALIGN_NOT_CONFIGURED = 1,
    FOC_ALIGN_NOT_SETTLED = 2,
    FOC_ALIGN_DIRECTION_WRONG = 3
} foc_align_failure_t;


/**
 * @brief [en] One position move: where to go, how hard to push, how fast, and what to add on top.
 *        [zh] 一次位置运动：去哪里、允许多大推力、多快、以及额外叠加多少。
 *        [zh] 一次位置运动：去哪里、允许用多大力、额外叠加多少。
 *
 * @since 0.1.0
 * @exporter
 */
typedef struct foc_motor_command {
    float position_rad;          /* target mechanical angle, rad */
    float max_current_a;         /* q-axis ceiling for this move, A; 0 = nameplate limit */
    float feedforward_current_a; /* q-axis current added after the speed loop, A */
    float max_velocity_rad_s;    /* speed ceiling for this move, rad/s; 0 = the controller's own */
    float max_accel_rad_s2;      /* velocity rate limit for this move, rad/s^2; 0 = the controller's */
    float kp_scale;              /* stiffness scale, 1 = configured, 0 = leave it configured */
    float kd_scale;              /* speed-loop gain scale, 1 = configured, 0 = leave it configured */
    float ilimit_scale;          /* speed-loop integral-limit scale, 1 = configured, 0 = leave it */
} foc_motor_command_t;


/**
 * @brief [en] Everything the motor can be asked about its own state, in one read.
 *        [zh] 一次读回电机的全部自述状态。
 *
 * @note [en] `struct_size` must be `sizeof(foc_motor_status_t)`; the library writes the fields that
 *       value covers, so growing this struct never writes past the caller's object.
 *       [zh] `struct_size` 必须是 `sizeof(foc_motor_status_t)`；库只写入该值覆盖到的字段，因此本结构
 *       以后增长也绝不会写到调用方对象之外。
 *
 * @note [en] The two state fields are `int32_t` rather than the enum types on purpose: a C enum's
 *       underlying type is implementation-defined, and these values are part of the frozen ABI.
 *       The `FOC_MOTOR_*`, `FOC_ALIGN_*` constants are the contract; the field is just its storage.
 *       [zh] 两个状态字段故意用 `int32_t` 而不是枚举类型：C 枚举的底层类型由实现决定，而这些数值属于
 *       冻结的 ABI。契约是 `FOC_MOTOR_*`、`FOC_ALIGN_*` 这些常量，字段只是它的存储。
 *
 * @since 0.1.0
 * @exporter
 */
typedef struct foc_motor_status {
    uint32_t struct_size;   /* sizeof(foc_motor_status_t) */
    int32_t mode_state;     /* foc_motor_state_t: Idle / Align / Run / Fault */
    int32_t error;          /* last sticky error, a foc_err_t value */
    int32_t align_state;    /* foc_align_state_t */
    int32_t align_failure;  /* foc_align_failure_t */
    float align_offset_rad; /* captured mechanical offset, rad in [0, 2*pi); valid once DONE */
    float align_spread_rad; /* spread of the accepted capture, rad */
    uint32_t max_period_us; /* worst control-tick interval seen, us; 0 without a clock */
} foc_motor_status_t;
