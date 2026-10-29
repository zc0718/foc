// Conan::ImportStart
#pragma once
#include <stdint.h>
// Conan::ImportEnd


/**
 * @brief [en] Opaque motor handle of the C path. Created by foc_motor_create(), invalidated by foc_motor_destroy(); a
 *             stale handle is detected through an internal generation counter.
 * @brief [zh] C 路径的不透明电机句柄。由 foc_motor_create() 创建、foc_motor_destroy() 失效； 内部用代际计数识别失效句
 *             柄。
 * @since 0.1.0
 * @exporter
 */
typedef struct foc_motor_s *foc_motor_t;


/**
 * @brief [en] Error codes of the C path. The numeric values are frozen: they mirror foc::ErrorCode one to one (see
 *             include/foc/core/types.hpp).
 * @brief [zh] C 路径的错误码。数值已冻结：与 foc::ErrorCode 一一对应 （见 include/foc/core/types.hpp）。
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
 * @brief [zh] 电机参数，SI 单位，供 foc_motor_create() 使用。
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
 * @brief [en] Which observer the motor runs.
 * @brief [zh] 电机运行哪一个观测器。
 * @note [en] all four modes are wired, and each one is admitted on facts rather than on a switch: `NONLINEAR_FLUX`
 *            needs only the nameplate, `HFI` needs a stated carrier and a salient machine, `BLENDED` needs those plus
 *            an ordered handover band. A request that this machine cannot honour answers `FOC_ERR_PARAM`. A mode that
 *            is named but not wired answers the error rather than silently falling back to the default, because a
 *            silent fallback is indistinguishable from success. `foc_motor_status_t::observer` reports the mode that is
 *            actually running, so a caller never has to infer it from what it asked for.
 * @note [zh] `BACK_EMF`（出厂默认）与 `NONLINEAR_FLUX` 已接入；`HFI` 与 `BLENDED` 仍回答 `FOC_ERR_NOT_IMPL`。增加一种模
 *            式不应改变本头的含义，只改变哪些取值回答该错误。被命名却未接入的模式回答该错误，而不是静默退回默认值 ——因
 *            为静默退回与成功无法区分。`foc_motor_status_t::observer` 报出实际在跑的模式，调用方无需从自己请求了什么去
 *            推断。
 * @since 0.1.4
 * @exporter
 */
// NOLINTNEXTLINE(performance-enum-size) -- a C ABI enum cannot carry a C++ base type.
typedef enum foc_observer {
    FOC_OBSERVER_BACK_EMF = 0,       /* back-EMF observer; the shipped default */
    FOC_OBSERVER_NONLINEAR_FLUX = 1, /* non-linear flux observer */
    FOC_OBSERVER_HFI = 2,            /* high-frequency injection, zero-speed */
    FOC_OBSERVER_BLENDED = 3         /* the blend of the two across speed */
} foc_observer_t;

/**
 * @brief [en] Optional setup for the C path: every value the library would otherwise have to guess.
 * @brief [zh] C 路径的可选设置：不提供时库只能自己猜的那些值。
 * @note [en] `struct_size` must be `sizeof(foc_motor_setup_t)`. A caller built against an older header passes a smaller
 *            value and the library only reads the fields that value covers, so growing this struct never reads past the
 *            caller's memory. Where 0 is not a legal value (current limit, carrier, bus voltage, bandwidths), 0 means
 *            "library default"; any other value is authoritative and validated, so a negative limit is a parameter
 *            error rather than a silent fallback. `deadtime_ns` and `speed_loop_period_s` take 0 literally, because 0
 *            is a legal value for both. The `align_*` fields are the exception that proves the rule: they are taken
 *            literally, and `align_current_a == 0` means "do not align" -- a legal, common choice for a machine whose
 *            offset is already known. But a *half*-configured alignment is rejected: with `align_current_a == 0` every
 *            other `align_*` field must be 0 as well, and with it non-zero `align_hold_s` and `align_tolerance_rad`
 *            become mandatory.
 *
 * @note [zh] `struct_size` 必须是 `sizeof(foc_motor_setup_t)`。用旧头编译的调用方会传更小的值， 库只读取该值覆盖到的字
 *            段，因此本结构以后增长也绝不会读到调用方内存之外。
 *
 *     在 0 不是合法取值的字段上（限流、载波、母线电压、带宽），0 表示“使用库默认值”； 其余取值一律采信并校验，因此负数
 *     限流是参数错误而不是静默回落。 `deadtime_ns` 与 `speed_loop_period_s` 的 0 按字面生效，因为 0 对它们都是合法取
 *     值。
 *
 *     `align_*` 字段是那条规则的例外：它们按字面生效，`align_current_a == 0` 意为“不做对齐” ——对偏置已知的电机这是一个
 *     合法且常见的选择。但**半配置**的对齐会被拒绝：`align_current_a` 为 0 时其余 `align_*` 字段也必须为 0；它非 0 时
 *     `align_hold_s` 与 `align_tolerance_rad` 变成必填。
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
    int32_t observer;                  /* foc_observer_t; 0 = FOC_OBSERVER_BACK_EMF, the shipped default */
    float inject_v;                    /* HFI carrier amplitude, peak V; required for FOC_OBSERVER_HFI */
    float inject_hz;                   /* HFI carrier frequency, Hz; required for FOC_OBSERVER_HFI */
    float blend_start_rad_s;           /* below this speed only the injection observer runs; required for BLENDED */
    float blend_end_rad_s;             /* above this speed only the back-EMF observer runs; required for BLENDED */
    float handover_lpf_hz;             /* smoothing of the speed the handover is decided on, Hz; required for BLENDED */
} foc_motor_setup_t;


/**
 * @brief [en] Operating mode state, mirroring foc::core::MotorState. Values are frozen.
 * @brief [zh] 运行模式状态，与 foc::core::MotorState 对应。数值已冻结。
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
 * @brief [zh] 对齐阶段，与 foc::core::AlignState 对应。数值已冻结。
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
 * @brief [zh] 对齐流程被拒的原因，与 foc::core::AlignFailure 对应。
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
 * @brief [zh] 一次位置运动：去哪里、允许多大推力、多快、以及额外叠加多少。 一次位置运动：去哪里、允许用多大力、额外叠加
 *             多少。
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
 * @brief [zh] 一次读回电机的全部自述状态。
 * @note [en] `struct_size` must be `sizeof(foc_motor_status_t)`; the library writes the fields that value covers, so
 *            growing this struct never writes past the caller's object. The two state fields are `int32_t` rather than
 *            the enum types on purpose: a C enum's underlying type is implementation-defined, and these values are part
 *            of the frozen ABI. The `FOC_MOTOR_*`, `FOC_ALIGN_*` constants are the contract; the field is just its
 *            storage.
 *
 * @note [zh] `struct_size` 必须是 `sizeof(foc_motor_status_t)`；库只写入该值覆盖到的字段，因此本结构 以后增长也绝不会写
 *            到调用方对象之外。
 *
 *     两个状态字段故意用 `int32_t` 而不是枚举类型：C 枚举的底层类型由实现决定，而这些数值属于 冻结的 ABI。契约是
 *     `FOC_MOTOR_*`、`FOC_ALIGN_*` 这些常量，字段只是它的存储。
 * @since 0.1.0
 * @exporter
 */
/**
 * @brief [en] An injection configuration sized from the machine and the control rate, rather than guessed.
 * @brief [zh] 由机器与控制率定容出来的注入配置，而不是猜出来的。
 * @note [en] Every field forwards a measured rule, not a preference: the carrier is the highest the rate can resolve
 *            (which is the best angle bias measured, 33x better than 2000 Hz at 8000), the amplitude is the volts that
 *            draw the HF current *you* budget (`I_hf * w_h * Ld`, so the amplitude becomes a decision about injected
 *            current instead of a magic voltage), and the two cutoffs are the fixed values the envelope sweep measured
 *            -- 200 Hz for demodulation, 200 Hz for the published speed.
 * @note [zh] 每个字段都转达一条实测规则而不是偏好：载波取该控制率能分辨的最高值（也就是实测最好的角度偏差——8000 Hz 比
 *            2000 Hz 好 33 倍），幅值取能拉出**你**预算的高频电流所需的那份伏特（`I_hf·ω_h·Ld`，于是幅值变成一个关于注
 *            入电流的决定，而不是一个魔法电压），两个截止频率是包线扫描量到的固定值——解调 200 Hz、已发布转速 200 Hz。
 * @since 0.1.7
 */
typedef struct foc_hfi_sizing {
    uint32_t struct_size; /* sizeof(foc_hfi_sizing_t) */
    float inject_hz;      /* the carrier this control rate can resolve, Hz */
    float inject_v;       /* peak volts that draw `hf_current_a` on this machine's d axis, V */
    float demod_lpf_hz;   /* demodulation cutoff, Hz */
    float speed_lpf_hz;   /* cutoff on the published speed, Hz */
    float hf_current_a;   /* the peak HF current `inject_v` draws, A */
} foc_hfi_sizing_t;

typedef struct foc_motor_status {
    uint32_t struct_size;   /* sizeof(foc_motor_status_t) */
    int32_t mode_state;     /* foc_motor_state_t: Idle / Align / Run / Fault */
    int32_t error;          /* last sticky error, a foc_err_t value */
    int32_t align_state;    /* foc_align_state_t */
    int32_t align_failure;  /* foc_align_failure_t */
    float align_offset_rad; /* captured mechanical offset, rad in [0, 2*pi); valid once DONE */
    float align_spread_rad; /* spread of the accepted capture, rad */
    uint32_t max_period_us; /* worst control-tick interval seen, us; 0 without a clock */
    int32_t observer;       /* the observer actually running (a foc_observer_t); appended, gated by struct_size */
    /* 1 when the selected observer says it is producing a measurement, 0 when it is holding the last angle. Appended;
       gated by struct_size. The distinction matters because a holding observer is indistinguishable from a stopped
       machine by its angle alone -- the injection observer past what the control rate can resolve freezes exactly this
       way. Observers without a lock detector (back-EMF) answer 1: they have no channel that could withhold a verdict. */
    /* 选中的观测器表示自己正在产出测量时为 1，正在保持上一个角度时为 0。追加字段，由 struct_size 门控。这个区分重要，
       因为一个正在保持的观测器仅凭它的角度与一台停住的机器无法区分——超出控制率分辨能力的注入观测器正是这样冻结的。
       没有锁定判据的观测器（反电势）答 1：它们不存在能扣住结论的通道。 */
    int32_t observer_locked;
    /* the selected observer's normalised error signal, about -sin(2*angle_error) for the injection observer and 0.0
       where that observer publishes none. Appended; gated by struct_size. Zero is ambiguous on purpose: a locked
       injection observer also sits near zero, so read `observer_locked` alongside it. */
    /* 选中观测器归一化的误差信号，注入观测器约为 -sin(2·角度误差)，不发布该信号的观测器为 0.0。追加字段，由
       struct_size 门控。零是有意保留歧义的：锁定的注入观测器同样停在零附近，因此要与 `observer_locked` 一起读。 */
    float observer_error;
} foc_motor_status_t;


/* ------------------------------------------------------------------------------------------------
 * Commissioning, protection and the reference policies.
 * 整定、保护与参考策略。
 *
 * [en] These exist because the C++ side grew five modules that a C consumer could
 * not reach at all: `protection.hpp`, `commissioning.hpp`, `board_check.hpp`,
 * `field_weakening.hpp` and `MtpaReference`. A "stable C11 ABI" that only covers
 * the motor object is a promise about half the library.
 * [zh] 这些之所以存在，是因为 C++ 侧长出了五个 C 消费方**根本够不到**的模块：
 * `protection.hpp`、`commissioning.hpp`、`board_check.hpp`、`field_weakening.hpp` 与
 * `MtpaReference`。只覆盖电机对象的"稳定 C11 ABI"是关于半个库的承诺。
 *
 * [en] Enum-like fields are `int32_t` storage with `FOC_*` constants as the
 * contract, for the same reason the motor status does it: a C enum's underlying
 * type is implementation-defined and these values are frozen.
 * [zh] 类枚举字段用 `int32_t` 存储、以 `FOC_*` 常量为契约，理由与电机状态相同：C 枚举
 * 的底层类型由实现决定，而这些数值是冻结的。
 * ---------------------------------------------------------------------------------------------- */

/**
 * @brief [en] The checks `foc_board_check()` reports on.
 * @brief [zh] `foc_board_check()` 报告的检查项。
 * @note [en] A C enum, like every other constant in this header: `#define` would be a constant declared by macro, which
 *            this project's tidy gate rejects, and the values are frozen either way.
 * @note [zh] C 枚举，与本头文件里其它常量一致：`#define` 属于"用宏声明常量"，被本项目的 tidy 门拒 绝；两种写法下这些数
 *            值同样冻结。
 * @since 0.1.1
 * @exporter
 */
// NOLINTNEXTLINE(performance-enum-size) -- a C ABI enum cannot carry a C++ base type.
typedef enum foc_board_item {
    FOC_BOARD_ITEM_CURRENT_SUM = 0,
    FOC_BOARD_ITEM_CURRENT_OFFSET = 1,
    FOC_BOARD_ITEM_CURRENT_POLARITY = 2,
    FOC_BOARD_ITEM_SAMPLING_INSTANT = 3,
    FOC_BOARD_ITEM_DEAD_TIME = 4,
    FOC_BOARD_ITEM_ENCODER_OFFSET = 5,
    FOC_BOARD_ITEM_PARAMETERS = 6,
    FOC_BOARD_ITEM_OBSERVER_HEALTH = 7
} foc_board_item_t;

/**
 * @brief [en] The single cause `foc_board_check()` names.
 * @brief [zh] `foc_board_check()` 点名的单一成因。
 * @since 0.1.1
 * @exporter
 */
// NOLINTNEXTLINE(performance-enum-size) -- a C ABI enum cannot carry a C++ base type.
typedef enum foc_board_cause {
    FOC_BOARD_CAUSE_NONE = 0,
    FOC_BOARD_CAUSE_OFFSET_BIAS = 1,
    FOC_BOARD_CAUSE_POLARITY_REVERSED = 2,
    FOC_BOARD_CAUSE_SAMPLING_INSTANT = 3,
    FOC_BOARD_CAUSE_DEAD_TIME_MISMATCH = 4,
    FOC_BOARD_CAUSE_ENCODER_OFFSET = 5,
    FOC_BOARD_CAUSE_PARAMETERS_OFF = 6,
    FOC_BOARD_CAUSE_OBSERVER_UNHEALTHY = 7
} foc_board_cause_t;

/**
 * @brief [en] `foc_protection_output_t.fault` domain; the same ranking as `FaultCode`.
 * @brief [zh] `foc_protection_output_t.fault` 的取值域；排名与 `FaultCode` 一致。
 * @since 0.1.1
 * @exporter
 */
// NOLINTNEXTLINE(performance-enum-size) -- a C ABI enum cannot carry a C++ base type.
typedef enum foc_fault {
    FOC_FAULT_NONE = 0,
    FOC_FAULT_OVER_CURRENT = 1,
    FOC_FAULT_OVER_VOLTAGE = 2,
    FOC_FAULT_UNDER_VOLTAGE = 3,
    FOC_FAULT_OVER_TEMPERATURE = 4,
    FOC_FAULT_I2T = 5,
    FOC_FAULT_STALL = 6,
    FOC_FAULT_LOST_SYNC = 7
} foc_fault_t;

/** [en] Rows in `foc_board_report_t`. Fixed so the report has no growth path to
 * negotiate. [zh] `foc_board_report_t` 的行数。固定，于是这份报告不需要为增长再谈一次。 */
// NOLINTNEXTLINE(performance-enum-size) -- same: C cannot specify the underlying type.
typedef enum foc_board_report_capacity { FOC_BOARD_MAX_ITEMS = 12 } foc_board_report_capacity_t;

/**
 * @brief [en] Everything a board check measures, in one place.
 * @brief [zh] 板级检查量到的全部东西，放在一处。
 * @note [en] `struct_size` first, like every other struct here: the library only writes the fields that value covers.
 * @note [zh] 与这里其它结构一样 `struct_size` 在最前：库只写入该值覆盖到的字段。
 * @since 0.1.1
 * @exporter
 */
typedef struct foc_board_measurements {
    uint32_t struct_size;
    float sum_current_a;      /* mean i_u+i_v+i_w under a steady drive, A */
    float offset_a[3];        /* zero-current reading per phase, A */
    float response_sign[3];   /* +1 / -1 / +2 = unknown, per phase */
    float ripple_measured_a;  /* ripple the ADC sees, A */
    float ripple_expected_a;  /* what the switching period implies, A */
    float deadtime_v;         /* identified dead-time volts (axis), V */
    float deadtime_cmd_v;     /* what the board was configured with, V */
    float encoder_offset_rad; /* residual mount offset, rad */
    float rs_ohm;             /* identified resistance, ohm */
    float rs_nameplate_ohm;   /* nameplate resistance, ohm */
    float psi_f_wb;           /* identified flux linkage, Wb */
    float psi_f_nameplate_wb; /* nameplate flux linkage, Wb */
    float observer_error_rad; /* observer angle error on a known trajectory, rad */
} foc_board_measurements_t;

/**
 * @brief [en] The tolerances, separate from the measurements.
 * @brief [zh] 容差，与测量值分开。
 * @since 0.1.1
 * @exporter
 */
typedef struct foc_board_limits {
    uint32_t struct_size;
    float sum_current_a;      /* |sum| allowed, A */
    float offset_a;           /* |zero reading| allowed, A */
    float ripple_ratio;       /* measured/expected ripple ceiling */
    float deadtime_ratio;     /* dead-time relative error allowed */
    float encoder_offset_rad; /* residual mount offset allowed, rad */
    float parameter_ratio;    /* Rs / psi_f relative error allowed */
    float observer_error_rad; /* observer error allowed, rad */
} foc_board_limits_t;

/** @brief [en] One row of the report. [zh] 报告的一行。 @since 0.1.1 @exporter */
typedef struct foc_board_reading {
    int32_t item;  /* FOC_BOARD_ITEM_* */
    float value;   /* the measured number */
    float limit;   /* the number it was compared with */
    int32_t pass;  /* non-zero = pass */
    int32_t phase; /* 0..2 for per-phase items, 3 otherwise */
} foc_board_reading_t;

/**
 * @brief [en] Every number plus the one cause to fix first.
 * @brief [zh] 每个数，加上那个最先该修的成因。
 * @since 0.1.1
 * @exporter
 */
typedef struct foc_board_report {
    uint32_t struct_size;
    foc_board_reading_t items[FOC_BOARD_MAX_ITEMS];
    int32_t count;       /* rows filled */
    int32_t failures;    /* rows that failed */
    int32_t cause;       /* FOC_BOARD_CAUSE_* */
    int32_t cause_phase; /* 0..2, or 3 */
} foc_board_report_t;

/**
 * @brief [en] The go/no-go answer from nameplate numbers alone.
 * @brief [zh] 只用铭牌数字得到的做/不做判定。
 * @since 0.1.1
 * @exporter
 */
typedef struct foc_fw_verdict {
    uint32_t struct_size;
    float i_char_a;   /* psi_f / Ld, A: where reluctance and magnet terms match */
    float ratio;      /* i_char / i_max; below 0.6 the region is worth having */
    float speed_gain; /* speed multiple the flux can buy */
    int32_t worthwhile;
} foc_fw_verdict_t;

/**
 * @brief [en] A PI loop's gains and clamps, as the library itself would set them.
 * @brief [zh] 一个 PI 环的增益与限幅，按库自己会设的值给出。
 * @note [en] All six numbers, not just `kp`/`ki`: a caller reproducing the loop outside the library needs the clamps
 *            too, and the clamps are where a reproduction usually diverges. The values come from the same helper the
 *            motor constructor calls, so a tuning tool cannot drift from the shipped loop.
 * @note [zh] 六个值全给，不只 `kp`/`ki`：在库外复现这个环的调用方还需要限幅，而差别通常就出在限幅上。取值来自电机构造函
 *            数调用的同一个辅助函数，因此整定工具无法与出厂环路漂移。
 * @since 0.1.3
 * @exporter
 */
typedef struct foc_pi_gains {
    uint32_t struct_size;
    float kp;           /* proportional gain */
    float ki;           /* integral gain */
    float out_min;      /* clamp on the controller output */
    float out_max;      /* clamp on the controller output */
    float integral_min; /* clamp on the integrator state */
    float integral_max; /* clamp on the integrator state */
} foc_pi_gains_t;

/**
 * @brief [en] What one identification produced, and how much it is worth.
 * @brief [zh] 一次辨识得到了什么，以及它值多少。
 * @note [en] `residual` is the part the model could not explain, in the same unit as the samples (V). `valid` is the
 *            honest gate: it is false when there are too few samples or the sweep is too narrow for a slope, and then
 *            `value` means nothing. A caller that ignores `valid` gets a number that looks like a measurement.
 * @note [zh] `residual` 是模型解释不掉的部分，单位与样本一致（V）。`valid` 是那道诚实的门：样本太少或扫描跨度不足以定出
 *            斜率时为 false，此时 `value` 没有意义。忽略 `valid` 的调用方会得到一个看起来像测量值的数字。
 * @since 0.1.3
 * @exporter
 */
typedef struct foc_identify_result {
    uint32_t struct_size;
    float value;    /* Rs in ohm, or psi_f in Wb, depending on the call */
    float offset;   /* the constant term the fit found, V */
    float residual; /* rms of what the model could not explain, V */
    int32_t valid;  /* non-zero only when the samples could support a slope */
    int32_t samples;
} foc_identify_result_t;

/**
 * @brief [en] Protection thresholds. Every field is used exactly as given -- a zero is a zero and it will trip
 *             immediately. Pass NULL to `foc_protector_create()` to get the library defaults for the nominal 24 V / 3 A
 *             machine instead of guessing them.
 * @brief [zh] 保护阈值。每个字段**按原样**使用——零就是零，它会立刻跳闸。要标称 24 V / 3 A 机器上的库默认值，给
 *             `foc_protector_create()` 传 NULL，而不是自己猜。
 * @since 0.1.1
 * @exporter
 */
typedef struct foc_protection_config {
    uint32_t struct_size;
    float i_hard_a;
    float i_max_a;
    float i2t_a2s;
    float i2t_cooling_s;
    float v_max_v;
    float v_min_v;
    float t_trip_c;
    float t_derate_start_c;
    float t_derate_end_c;
    float stall_speed_rad_s;
    float stall_current_a;
    float stall_time_s;
    float sync_error_ratio;
    float hysteresis_ratio;
} foc_protection_config_t;

/** @brief [en] One step's measurements. [zh] 一步的测量值。 @since 0.1.1 @exporter */
typedef struct foc_protection_input {
    uint32_t struct_size;
    float i_magnitude_a;    /* peak phase current, A */
    float v_bus_v;          /* DC link, V */
    float temperature_c;    /* hottest measured point, deg C */
    float omega_e_rad_s;    /* electrical speed, rad/s */
    float flux_error_ratio; /* |psi_est - psi_f| / psi_f, 0 = healthy */
    int32_t flux_valid;     /* non-zero = the observer has an answer */
} foc_protection_input_t;

/**
 * @brief [en] What to do this step.
 * @brief [zh] 本步的处置。
 * @since 0.1.1
 * @exporter
 */
typedef struct foc_protection_output {
    uint32_t struct_size;
    int32_t fault;          /* FOC_FAULT_* */
    uint32_t active_mask;   /* bit 1<<code per active fault */
    uint32_t latched_mask;  /* the same, faults that will not self-clear */
    float i_limit_a;        /* derated current limit, 0 when tripped */
    int32_t output_allowed; /* 0 = stop switching this step */
    float i2t_budget;       /* the thermal budget in A^2*s */
} foc_protection_output_t;

/**
 * @brief [en] The HAL operations a host supplies, as plain C function pointers.
 * @brief [zh] 主控方提供的 HAL 操作，以普通 C 函数指针给出。
 * @details [en] Four are required and are checked by the library: duties out, phase currents in, mechanical angle in,
 *               DC link voltage in. The rest are optional, and NULL means "this host has no such facility" rather than
 *               "assume a convenient value" -- no clock reports 0 instead of inventing a timebase.
 * @details [zh] 四个是必需的，由库检查：占空比送出、相电流读入、机械角读入、母线电压读入。其余是可选的， NULL 的含义是"
 *               这个主控方没有这个能力"，而不是"假定一个方便的值"——没有时钟就报 0，不编一个时基。
 * @since 0.1.2
 * @exporter
 */
typedef struct foc_hal_callbacks {
    uint32_t struct_size; /* sizeof(foc_hal_callbacks_t) */
    void *user;           /* handed back to every callback unchanged */

    /* Required: receive the three phase duties, each in [0, 1]. */
    void (*pwm_write_duty)(void *user, const float duty_uvw[3]);
    /* Required: phase currents in A, U/V/W. */
    void (*read_current)(void *user, float uvw_a[3]);
    /* Required: mechanical angle in rad. */
    float (*read_angle_mech_rad)(void *user);
    /* Required: DC link voltage in V. */
    float (*read_voltage_v)(void *user);

    /* Optional: mechanical speed in rad/s; NULL = this host cannot measure it. */
    float (*read_velocity_rad_s)(void *user);
    /* Optional: non-zero when the inverter reports a fault; NULL = no fault path. */
    int32_t (*pwm_fault)(void *user);
    /* Optional: enable the output stage, 0 = ok; NULL = nothing to enable. */
    int32_t (*pwm_enable)(void *user);
    /* Optional: disable the output stage; NULL = nothing to disable. */
    void (*pwm_disable)(void *user);
    /* Optional: monotonic microseconds; NULL = trust the dt given to each tick. */
    uint64_t (*now_us)(void *user);
} foc_hal_callbacks_t;

/** @brief [en] Handle to a protector instance. [zh] 保护器实例的句柄。 @since 0.1.1 */
typedef struct foc_protector_s *foc_protector_t;


/**
 * @brief [en] One period of the control loop, copied out of the C++ trace channel.
 * @brief [zh] 控制环的一个周期，从 C++ 轨迹通道拷出。
 * @note [en] The layout is the C++ `core::TraceSample` field for field, and `bridge.cpp` asserts that at compile time
 *            rather than trusting this comment. Seven numbers, two states, and the platform timestamp truncated to 32
 *            bits; `t_us == 0` on every sample means the HAL has no clock.
 * @note [zh] 布局与 C++ 的 `core::TraceSample` 逐字段一致，`bridge.cpp` 在编译期断言这一点而不靠这条注释。七个数、两个
 *            状态，以及截断到 32 位的平台时间戳；每条 `t_us == 0` 表示 HAL 没有时钟。
 * @since 0.1.8
 * @exporter
 */
typedef struct foc_trace_sample {
    uint32_t t_us;    /* platform clock, low 32 bits; 0 = this HAL has no clock */
    float i_d;        /* measured d-axis current, A */
    float i_q;        /* measured q-axis current, A */
    float i_ref_d;    /* demanded d-axis current, A */
    float i_ref_q;    /* demanded q-axis current, A */
    float v_d;        /* applied d-axis voltage, V */
    float v_q;        /* applied q-axis voltage, V */
    float speed_mech; /* mechanical speed, rad/s */
    uint8_t mode;     /* foc_motor_state_t at the end of the period */
    uint8_t status;   /* foc_err_t at the end of the period, as its unsigned byte */
    uint16_t pad;     /* keeps the record a fixed size; written as 0 */
} foc_trace_sample_t;


/**
 * @brief [en] What a recorded step response did, measured rather than eyeballed.
 * @brief [zh] 一段记录下来的阶跃响应到底做了什么，是测出来的而不是看出来的。
 * @note [en] `status` is the diagnostic channel and not decoration: `FOC_OK` means a first-order decay was fitted,
 *            `FOC_ERR_STATE` means the window carried no usable decay -- `bandwidth_rad_s` is 0 **and the settling
 *            numbers are still valid**, because a response can settle without being fittable. `FOC_ERR_PARAM` means the
 *            arguments cannot describe a step at all. A bandwidth of 0 therefore never means "not measured by
 *            accident".
 * @note [zh] `status` 是诊断通道，不是装饰：`FOC_OK` 表示拟合到一阶衰减，`FOC_ERR_STATE` 表示窗口里没有可用的衰减—— 此
 *            时 `bandwidth_rad_s` 为 0，**而稳定时间等数仍然有效**，因为一段响应可以稳定却不可拟合。`FOC_ERR_PARAM` 表
 *            示参数根本描述不了一个阶跃。因此带宽为 0 永远不会是"不小心没测"。
 * @since 0.1.8
 * @exporter
 */
typedef struct foc_step_metrics {
    uint32_t struct_size;      /* sizeof(foc_step_metrics_t) */
    int32_t status;            /* foc_err_t of the measurement itself */
    float bandwidth_rad_s;     /* -3 dB of the fitted first-order loop, rad/s; 0 unless status is FOC_OK */
    float tau_s;               /* fitted time constant, s; 1 / bandwidth_rad_s */
    float settling_s;          /* s from the first sample until it stays inside the band; -1 = never */
    float overshoot_fraction;  /* peak beyond the target, as a fraction of the step */
    float tail_error_fraction; /* mean of the last tenth minus the target, as a fraction of the step */
} foc_step_metrics_t;
