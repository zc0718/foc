// Conan::ImportStart
#include "bridge.h"

#include <cmath>
#include <limits>
#include <cstddef>
#include <algorithm>
#include <cstring>
#include <cstdint>
#include <new>

#include <foc/core/alignment.hpp>
#include <foc/core/board_check.hpp>
#include <foc/core/commissioning.hpp>
#include <foc/core/field_weakening.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/switchable_observer.hpp>
#include <foc/core/protection.hpp>
#include <foc/core/reference.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/driver/callback_hal.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>
// Conan::ImportEnd

namespace foc {

namespace api {

namespace {

// The C path is the *whole* motor: position loop and aligner included. A C caller cannot add
// template parameters, so the choice has to be made here; the defaults (`NullAlignment` etc.) would
// silently cap the C API at a subset of the C++ one, which is the "library that only serves one
// scenario" trap this project keeps refusing to fall into.
// C 路径给的是**完整**电机：位置环与对齐器都在。C 调用方无法追加模板参数，因此这个选择必须在这里
// 做定；用默认的空策略（`NullAlignment` 等）会把 C API 静默地封成 C++ API 的子集，而那正是本项目
// 一直在拒绝的"只服务一种场景的库"陷阱。
using MotorT = motor::Motor<core::SwitchableObserver, core::Svpwm, core::Pi, driver::StubHal,
                            core::NullReferenceGenerator, core::NullFeedforward, core::PositionP, core::AlignRoutine>;

/**
 * @brief [en] The same motor with a host-fed HAL: used only by `foc_motor_create_host()`.
 * @brief [zh] 同一个电机，HAL 由主控方喂数据：只被 `foc_motor_create_host()` 使用。
 * @note [en] Identical policy list on purpose. A second policy composition would be a second control path, and the
 *            point of the two pools is that the loop under test is one object.
 * @note [zh] 策略列表刻意完全一致。换一套策略组合就是第二条控制路径，而两个池的意义恰恰在于被测的环是同一个。
 * @since 0.1.2
 */
using MotorT_Host =
    motor::Motor<core::SwitchableObserver, core::Svpwm, core::Pi, driver::CallbackHal, core::NullReferenceGenerator,
                 core::NullFeedforward, core::PositionP, core::AlignRoutine>;

/**
 * @brief [en] Object pool capacity: static storage, no heap, no fragmentation.
 * @brief [zh] 对象池容量：静态存储、无堆、无碎片。
 */
constexpr std::size_t kMaxMotors = 4U;

/**
 * @brief [en] The values the C path runs with. Defaults live here; the caller overrides them.
 * @brief [zh] C 路径实际使用的取值。默认值放在这里，由调用方覆盖。
 * @since 0.1.0
 */
struct Setup final {
    float i_max_a;                  ///< [en] per-axis current limit in A. [zh] 每轴电流限幅，A。
    float pwm_freq_hz;              ///< [en] carrier frequency in Hz. [zh] 载波频率，Hz。
    float deadtime_ns;              ///< [en] inverter dead time in ns. [zh] 逆变器死区时间，ns。
    float v_bus_v;                  ///< [en] nominal DC link voltage in V. [zh] 母线电压，V。
    float current_bandwidth_rad_s;  ///< [en] current-loop bandwidth, rad/s. [zh] 电流环带宽，rad/s。
    float speed_bandwidth_rad_s;    ///< [en] speed-loop bandwidth, rad/s. [zh] 速度环带宽，rad/s。
    float speed_loop_period_s;      ///< [en] speed-loop period, s (0 = every tick). [zh] 速度环周期，s（0 = 每 tick）。
    float position_bandwidth_rad_s; ///< [en] position-loop bandwidth, rad/s. [zh] 位置环带宽，rad/s。
    float velocity_limit_rad_s;     ///< [en] speed ceiling for position moves, rad/s. [zh] 位置运动的速度上限，rad/s。
    /**
     * @brief [en] position-loop period, s (0 = every tick).
     * @brief [zh] 位置环周期，s（0 = 每 tick）。
     */
    float position_loop_period_s;
    /**
     * @brief [en] position velocity rate limit (0 = none).
     * @brief [zh] 位置速度指令的加速度上限（0 = 不限）。
     */
    float position_accel_limit_rad_s2;
    float align_current_a;     ///< [en] alignment current, A (0 = do not align). [zh] 对齐电流，A（0 = 不对齐）。
    float align_ramp_s;        ///< [en] alignment ramp, s. [zh] 对齐爬升，s。
    float align_hold_s;        ///< [en] alignment hold window, s. [zh] 对齐保持窗口，s。
    float align_tolerance_rad; ///< [en] accepted spread, rad. [zh] 允许离散度，rad。
    float align_verify_s;      ///< [en] direction check hold, s. [zh] 方向校验保持，s。
    float align_angle_e_rad;   ///< [en] forced electrical angle, rad. [zh] 强制电角度，rad。
    /**
     * @brief [en] which observer to run, as a `foc_observer_t`; 0 is the shipped default.
     * @brief [zh] 运行哪个观测器，取 `foc_observer_t`；0 是出厂默认。
     */
    int32_t observer;
    float inject_v;          ///< [en] HFI carrier amplitude, peak V. [zh] HFI 载波幅值，峰值 V。
    float inject_hz;         ///< [en] HFI carrier frequency, Hz. [zh] HFI 载波频率，Hz。
    float blend_start_rad_s; ///< [en] below this only the injection observer runs. [zh] 低于此值只用注入观测器。
    float blend_end_rad_s;   ///< [en] above this only the back-EMF observer runs. [zh] 高于此值只用反电势观测器。
    float handover_lpf_hz;   ///< [en] smoothing of the handover speed, Hz. [zh] 交接判据转速的平滑，Hz。
};


/**
 * @brief [en] What `foc_motor_create()` has always used.
 * @brief [zh] `foc_motor_create()` 一直使用的取值。
 */
constexpr Setup kDefaultSetup{5.0F, 20000.0F, 500.0F, 24.0F, 2000.0F, 200.0F,
                              0.0F, 20.0F,    50.0F,  0.0F,  0.0F,    0.0F,
                              0.0F, 0.0F,     0.0F,   0.0F,  0.0F,    static_cast<std::int32_t>(FOC_OBSERVER_BACK_EMF),
                              0.0F, 0.0F,     0.0F,   0.0F,  0.0F};

/**
 * @brief [en] The demodulation filter's cutoff in Hz: the value `test_sil_hfi.cpp` fixes for every case, so that a
 *             failure can never be blamed on the filter instead of on the physics.
 * @brief [zh] 解调滤波截止频率，Hz：`test_sil_hfi.cpp` 对所有用例都固定此值，于是失败永远不会被归咎于滤波器而非物理。
 */
constexpr float kHfiDemodLpfHz = 200.0F;

/**
 * @brief [en] The carrier must clear that cutoff by at least this factor, or the filter removes the very component
 *             being demodulated.
 * @brief [zh] 载波至少要高出该截止频率这么多倍，否则滤波器会把要解调的那个分量本身滤掉。
 */
constexpr float kHfiMinRatio = 2.0F;



/**
 * @brief [en] Whether the caller's struct is large enough to contain the field ending at `end_offset`.
 * @brief [zh] 调用方的结构是否足够大，能包含以 `end_offset` 结尾的那个字段。
 * @since 0.1.0
 */
auto covers(const foc_motor_setup_t& setup, std::size_t end_offset) noexcept -> bool {
    return static_cast<std::size_t>(setup.struct_size) >= end_offset;
}


/**
 * @brief [en] The nameplate and the loop configurations every C-path motor is built from.
 * @brief [zh] 每个 C 路径电机都由它构造出的铭牌与环路配置。
 * @note [en] One resolver, three callers: both constructors and the gain queries. The gain query therefore cannot
 *            answer something the shipped loop does not use -- which is the only way a tuning tool outside the library
 *            can be trusted to reproduce it.
 * @note [zh] 一个解析函数，三个调用方：两个构造函数与增益查询。于是增益查询不可能答出出厂环路没有使用的值——这也是库外整
 *            定工具能够被信任为复现它的唯一理由。
 * @since 0.1.3
 */
struct Resolved final {
    foc::motor::MotorConfig config;
    foc::core::Pi::Config current;
    foc::core::Pi::Config speed;
    foc::core::PositionP::Config position;
};


auto resolve_config(const foc_motor_params_t& params, const Setup& resolved) noexcept -> Resolved {
    const foc::motor::MotorConfig config{params.rs,
                                         params.ld,
                                         params.lq,
                                         params.kt,
                                         params.j,
                                         params.b,
                                         params.pole_pairs,
                                         resolved.i_max_a,
                                         resolved.pwm_freq_hz,
                                         resolved.deadtime_ns,
                                         resolved.speed_loop_period_s,
                                         resolved.position_loop_period_s};
    return Resolved{config, foc::current_pi_config(config, resolved.current_bandwidth_rad_s, resolved.v_bus_v),
                    foc::speed_pi_config(config, resolved.speed_bandwidth_rad_s),
                    foc::position_p_config(config, resolved.position_bandwidth_rad_s, resolved.velocity_limit_rad_s,
                                           resolved.position_accel_limit_rad_s2)};
}


/**
 * @brief [en] Merge the caller's setup over the defaults, reading only what their `struct_size` covers, so an older and
 *             shorter struct is safe by construction rather than by convention.
 * @brief [zh] 把调用方的设置叠在默认值上，只读取其 `struct_size` 覆盖到的字段；于是更短的旧结构是 “结构上安全”而不是“约
 *             定安全”。
 * @since 0.1.0
 */
auto resolve_setup(const foc_motor_setup_t* requested) noexcept -> Setup {
    Setup out = kDefaultSetup;
    if (requested == nullptr) {
        return out;
    }
    if (!covers(*requested, offsetof(foc_motor_setup_t, struct_size) + sizeof(requested->struct_size))) {
        return out;
    }

    // 0 means "unset" exactly where 0 is not a legal value; every other value is authoritative and
    // therefore validated (a negative limit is a parameter error, not a silent default).
    // 0 仅在 0 本身不合法时才表示“未设置”；其余取值一律采信并校验（负数限流是参数错误，不是静默
    // 回落默认值）。
    if (covers(*requested, offsetof(foc_motor_setup_t, i_max_a) + sizeof(requested->i_max_a)) &&
        (requested->i_max_a != 0.0F)) {
        out.i_max_a = requested->i_max_a;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, pwm_freq_hz) + sizeof(requested->pwm_freq_hz)) &&
        (requested->pwm_freq_hz != 0.0F)) {
        out.pwm_freq_hz = requested->pwm_freq_hz;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, deadtime_ns) + sizeof(requested->deadtime_ns))) {
        out.deadtime_ns = requested->deadtime_ns;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, v_bus_v) + sizeof(requested->v_bus_v)) &&
        (requested->v_bus_v != 0.0F)) {
        out.v_bus_v = requested->v_bus_v;
    }
    if (covers(*requested,
               offsetof(foc_motor_setup_t, current_bandwidth_rad_s) + sizeof(requested->current_bandwidth_rad_s)) &&
        (requested->current_bandwidth_rad_s != 0.0F)) {
        out.current_bandwidth_rad_s = requested->current_bandwidth_rad_s;
    }
    if (covers(*requested,
               offsetof(foc_motor_setup_t, speed_bandwidth_rad_s) + sizeof(requested->speed_bandwidth_rad_s)) &&
        (requested->speed_bandwidth_rad_s != 0.0F)) {
        out.speed_bandwidth_rad_s = requested->speed_bandwidth_rad_s;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, speed_loop_period_s) + sizeof(requested->speed_loop_period_s))) {
        out.speed_loop_period_s = requested->speed_loop_period_s;
    }
    if (covers(*requested,
               offsetof(foc_motor_setup_t, position_bandwidth_rad_s) + sizeof(requested->position_bandwidth_rad_s)) &&
        (requested->position_bandwidth_rad_s != 0.0F)) {
        out.position_bandwidth_rad_s = requested->position_bandwidth_rad_s;
    }
    if (covers(*requested,
               offsetof(foc_motor_setup_t, velocity_limit_rad_s) + sizeof(requested->velocity_limit_rad_s)) &&
        (requested->velocity_limit_rad_s != 0.0F)) {
        out.velocity_limit_rad_s = requested->velocity_limit_rad_s;
    }
    if (covers(*requested,
               offsetof(foc_motor_setup_t, position_loop_period_s) + sizeof(requested->position_loop_period_s))) {
        out.position_loop_period_s = requested->position_loop_period_s;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, position_accel_limit_rad_s2) +
                               sizeof(requested->position_accel_limit_rad_s2))) {
        out.position_accel_limit_rad_s2 = requested->position_accel_limit_rad_s2;
    }
    // The observer is read literally, like dead time: 0 is a legal value (and the default), so "unset" and "the default"
    // mean the same thing here and there is nothing to distinguish.
    // 观测器按字面读取，与死区同理：0 是合法值（也是默认值），因此"未设置"与"默认"在此含义相同，无需区分。
    if (covers(*requested, offsetof(foc_motor_setup_t, observer) + sizeof(requested->observer))) {
        out.observer = requested->observer;
    }
    // The carrier's amplitude and frequency are the caller's decision -- the bus voltage and the current limit bound
    // them -- and have no defensible default, so they are read literally like `observer`: zero means "not supplied",
    // and a mode that needs them refuses rather than inventing a value.
    // 载波幅值与频率由调用方决定（受母线电压与电流限幅约束），没有站得住的默认值，因此与 `observer` 一样按字面读取：
    // 零表示"未提供"，需要它们的模式会拒绝，而不是编一个值。
    if (covers(*requested, offsetof(foc_motor_setup_t, inject_v) + sizeof(requested->inject_v))) {
        out.inject_v = requested->inject_v;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, inject_hz) + sizeof(requested->inject_hz))) {
        out.inject_hz = requested->inject_hz;
    }
    // Same rule again: the band belongs to the machine's speed range, so there is nothing sensible to default it to,
    // and a request for a blend without one is refused rather than handed a guess.
    // 规则相同：交接带属于机器的转速范围，没有合理的默认值可填，因此没有交接带的混合请求会被拒绝，而不是被猜一个。
    if (covers(*requested, offsetof(foc_motor_setup_t, blend_start_rad_s) + sizeof(requested->blend_start_rad_s))) {
        out.blend_start_rad_s = requested->blend_start_rad_s;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, blend_end_rad_s) + sizeof(requested->blend_end_rad_s))) {
        out.blend_end_rad_s = requested->blend_end_rad_s;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, handover_lpf_hz) + sizeof(requested->handover_lpf_hz))) {
        out.handover_lpf_hz = requested->handover_lpf_hz;
    }
    // Alignment deliberately breaks the "0 = library default" rule: 0 is a meaningful value here
    // ("do not align"), so every field is taken literally and any non-zero value is authoritative.
    // 对齐故意打破“0 = 库默认值”的规则：0 在这里是有意义的值（“不对齐”），所以每个字段按字面生效，
    // 任何非 0 值都采信。
    if (covers(*requested, offsetof(foc_motor_setup_t, align_current_a) + sizeof(requested->align_current_a))) {
        out.align_current_a = requested->align_current_a;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, align_ramp_s) + sizeof(requested->align_ramp_s))) {
        out.align_ramp_s = requested->align_ramp_s;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, align_hold_s) + sizeof(requested->align_hold_s))) {
        out.align_hold_s = requested->align_hold_s;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, align_tolerance_rad) + sizeof(requested->align_tolerance_rad))) {
        out.align_tolerance_rad = requested->align_tolerance_rad;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, align_verify_s) + sizeof(requested->align_verify_s))) {
        out.align_verify_s = requested->align_verify_s;
    }
    if (covers(*requested, offsetof(foc_motor_setup_t, align_angle_e_rad) + sizeof(requested->align_angle_e_rad))) {
        out.align_angle_e_rad = requested->align_angle_e_rad;
    }
    return out;
}


/**
 * @brief [en] Whether a resolved setup describes a machine the core would accept.
 * @brief [zh] 合并后的设置是否描述了一台核心能接受的电机。
 * @since 0.1.0
 */
/**
 * @brief [en] Whether the requested observer can actually be run.
 * @brief [zh] 所请求的观测器是否真的可以运行。
 * @return [en] `FOC_OK` for a mode this machine can run, `FOC_ERR_PARAM` for a request it cannot honour -- a value
 *              outside the enumeration, or a mode whose parameters are missing or contradicted by the machine.
 * @return [zh] 这台机器能跑的模式返回 `FOC_OK`；满足不了的请求返回 `FOC_ERR_PARAM`——枚举之外的取值，或参数缺失、或与机
 *              器相矛盾的请求。
 * @note [en] The answers are kept apart on purpose. Answering `FOC_OK` and running the default instead would be a
 *            silent substitution. Every mode in the enumeration is wired today, so nothing here answers
 *            `FOC_ERR_NOT_IMPL`; that answer stays in the contract for a mode added later, and a branch that could
 *            never be reached was removed rather than left in place looking like coverage.
 * @note [zh] 几种回答刻意分开。答 `FOC_OK` 却跑默认值，是一种静默替换。今天枚举里的每个模式都已接入，因此这里不再回答
 *            `FOC_ERR_NOT_IMPL`；该回答留在契约里供以后新增的模式使用，而一个永远走不到的分支被删掉，而不是留下来充作覆
 *            盖。
 * @since 0.1.4
 */
auto observer_status(const Setup& setup, const foc_motor_params_t& params) noexcept -> foc_err_t {
    switch (setup.observer) {
    case FOC_OBSERVER_BACK_EMF:
    case FOC_OBSERVER_NONLINEAR_FLUX:
        return FOC_OK;
    case FOC_OBSERVER_HFI:
        // Three conditions, each a fact about the request or the machine rather than a preference:
        //   - the carrier has to be stated, because there is no defensible default for it;
        //   - the machine has to be salient. `predicted_gain_a()` is proportional to `(lq - ld)`, so on a surface
        //     machine the demodulated signal is identically zero and no follower can lock. The observer's own
        //     constructor rejects `demod_gain_a == 0`, so admitting the mode would only move the refusal later, into
        //     a construction whose status nobody reads;
        //   - the carrier has to clear the demodulation cutoff by `kHfiMinRatio`, which is what keeps the filter from
        //     removing the component it is supposed to demodulate.
        // 三个条件，每一个都是关于请求或机器的事实而非偏好：
        //   - 载波必须给出，因为它没有站得住的默认值；
        //   - 机器必须有凸极。`predicted_gain_a()` 正比于 `(lq - ld)`，表贴机上解调信号恒为零、任何跟随器都锁不住。
        //     观测器自己的构造函数就拒绝 `demod_gain_a == 0`，放行只会把拒绝推后到一次没人读状态返回值的构造里；
        //   - 载波必须比解调截止频率高出 `kHfiMinRatio` 倍，这正是让滤波器不会滤掉它本该解调的那个分量的条件。
        if (!(std::isfinite(setup.inject_v) && (setup.inject_v > 0.0F) && std::isfinite(setup.inject_hz) &&
              (setup.inject_hz > (kHfiMinRatio * kHfiDemodLpfHz)))) {
            return FOC_ERR_PARAM;
        }
        if (!(std::isfinite(params.ld) && std::isfinite(params.lq) && (params.ld > 0.0F) && (params.lq > 0.0F) &&
              (params.ld != params.lq))) {
            return FOC_ERR_PARAM;
        }
        return FOC_OK;
    case FOC_OBSERVER_BLENDED:
        // A blend is the injection observer below the band and the back-EMF observer above it, so it inherits both
        // requirements and adds one of its own: an ordered band with a settled handover filter. Refusing a backwards or
        // empty band is the same kind of answer as refusing a surface machine for HFI -- the request is well formed and
        // this machine cannot honour it.
        // 混合就是"带下用注入观测器、带上用反电势观测器"，因此继承两者的要求，并多一条自己的：一个有序的带与一个已定
        // 的交接滤波器。拒绝一个反向或空心的带，与为 HFI 拒绝表贴机是同一类回答——请求写法正确，而这台机器满足不了。
        if (!(std::isfinite(setup.inject_v) && (setup.inject_v > 0.0F) && std::isfinite(setup.inject_hz) &&
              (setup.inject_hz > (kHfiMinRatio * kHfiDemodLpfHz)))) {
            return FOC_ERR_PARAM;
        }
        if (!(std::isfinite(params.ld) && std::isfinite(params.lq) && (params.ld > 0.0F) && (params.lq > 0.0F) &&
              (params.ld != params.lq))) {
            return FOC_ERR_PARAM;
        }
        if (!(std::isfinite(setup.blend_start_rad_s) && (setup.blend_start_rad_s >= 0.0F) &&
              std::isfinite(setup.blend_end_rad_s) && (setup.blend_end_rad_s > setup.blend_start_rad_s) &&
              std::isfinite(setup.handover_lpf_hz) && (setup.handover_lpf_hz > 0.0F))) {
            return FOC_ERR_PARAM;
        }
        return FOC_OK;
    default:
        return FOC_ERR_PARAM;
    }
}


auto setup_ok(const Setup& setup) noexcept -> bool {
    if (!(std::isfinite(setup.i_max_a) && (setup.i_max_a > 0.0F) && std::isfinite(setup.pwm_freq_hz) &&
          (setup.pwm_freq_hz > 0.0F) && std::isfinite(setup.deadtime_ns) && (setup.deadtime_ns >= 0.0F) &&
          std::isfinite(setup.v_bus_v) && (setup.v_bus_v > 0.0F) && std::isfinite(setup.current_bandwidth_rad_s) &&
          (setup.current_bandwidth_rad_s > 0.0F) && std::isfinite(setup.speed_bandwidth_rad_s) &&
          (setup.speed_bandwidth_rad_s > 0.0F) && std::isfinite(setup.speed_loop_period_s) &&
          (setup.speed_loop_period_s >= 0.0F) && std::isfinite(setup.position_bandwidth_rad_s) &&
          (setup.position_bandwidth_rad_s > 0.0F) && std::isfinite(setup.velocity_limit_rad_s) &&
          (setup.velocity_limit_rad_s > 0.0F) && std::isfinite(setup.position_loop_period_s) &&
          (setup.position_loop_period_s >= 0.0F) && std::isfinite(setup.position_accel_limit_rad_s2) &&
          (setup.position_accel_limit_rad_s2 >= 0.0F))) {
        return false;
    }

    const bool align_fields_finite = std::isfinite(setup.align_current_a) && std::isfinite(setup.align_ramp_s) &&
                                     std::isfinite(setup.align_hold_s) && std::isfinite(setup.align_tolerance_rad) &&
                                     std::isfinite(setup.align_verify_s) && std::isfinite(setup.align_angle_e_rad);
    if (!align_fields_finite) {
        return false;
    }

    if (setup.align_current_a == 0.0F) {
        // "Do not align" has to be the whole recipe, not half of one: a caller who set the hold time
        // but no current almost certainly meant to set the current too, and silently aligning
        // nothing would hide that right up to the first move on a real shaft.
        // “不对齐”必须是整套配方，而不是半套：设了保持时间却没给电流的调用方几乎肯定也想给电流，
        // 静默地什么都不对齐只会把这个笔误藏到真机第一次运动的时候。
        return (setup.align_ramp_s == 0.0F) && (setup.align_hold_s == 0.0F) && (setup.align_tolerance_rad == 0.0F) &&
               (setup.align_verify_s == 0.0F) && (setup.align_angle_e_rad == 0.0F);
    }
    return (setup.align_current_a > 0.0F) && (setup.align_ramp_s >= 0.0F) && (setup.align_hold_s > 0.0F) &&
           (setup.align_tolerance_rad > 0.0F) && (setup.align_verify_s >= 0.0F);
}


/**
 * @brief [en] One pool slot: inline storage for the motor plus the control block the handle points at. `storage` is the
 *             only place a `Motor<>` ever lives on the C path.
 * @brief [zh] 一个池槽：电机内联存储 + 句柄指向的控制块。`storage` 是 C 路径上 `Motor<>` 唯一 存在的地方。
 * @since 0.1.0
 */
template <typename MotorType, typename HalType> struct SlotT final {
    alignas(MotorType) unsigned char storage[sizeof(MotorType)]{};
    MotorType* instance{nullptr};
    HalType hal{};
    Setup setup{};
    std::uint32_t generation{0U};
    bool alive{false};
};

using Slot = SlotT<MotorT, driver::StubHal>;
using HostSlot = SlotT<MotorT_Host, driver::CallbackHal>;

/**
 * @brief [en] High word of every handle token: "FOC\0". Anything else is not ours.
 * @brief [zh] 每个句柄令牌的高位字："FOC\0"。别的值都不是本池签发的。
 */
constexpr std::uintptr_t kHandleMagic = 0x464F4300U;

/**
 * @brief [en] High word of a host-motor handle: "FOCH", distinct from "FOC".
 * @brief [zh] 主控电机句柄的高位字："FOCH"，与 "FOC" 区分。
 */
constexpr std::uintptr_t kHostHandleMagic = 0x464F4348U;

/**
 * @brief [en] Host motors are per-process experiments, not a fleet.
 * @brief [zh] 主控电机是进程内实验，不是车队。
 */
constexpr std::size_t kMaxHostMotors = 2U;

/**
 * @brief [en] Bits reserved for the slot index in a handle token.
 * @brief [zh] 句柄令牌中槽位索引占用的位。
 */
constexpr std::uint32_t kSlotBits = 8U;


/**
 * @brief [en] Mint a handle token: magic | generation | slot, as a pointer that is *never* dereferenced.
 * @brief [zh] 签发句柄令牌：magic | 代际 | 槽位，形式上是**永不解引用**的指针。
 * @note [en] An earlier version pointed the handle at a per-slot control block; because the block is reused by the next
 *            occupation, a stale handle then read the *new* generation and passed validation. The generation has to
 *            live in the handle value itself.
 * @note [zh] 早先的版本让句柄指向每个槽位的控制块；由于控制块会被下一次占用复用，旧句柄会读到 **新**代际从而通过校验。
 *            代际必须放在句柄值本身里。
 * @since 0.1.0
 */
auto mint_handle(std::uint32_t slot, std::uint32_t generation, std::uintptr_t magic = kHandleMagic) noexcept
    -> foc_motor_t {
    // The token is never dereferenced: it is a handle value, not an address.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, performance-no-int-to-ptr)
    return reinterpret_cast<foc_motor_t>((magic << 32U) | (static_cast<std::uintptr_t>(generation) << kSlotBits) |
                                         static_cast<std::uintptr_t>(slot));
}

/**
 * @brief [en] Static pool: constant-initialised, so there is no dynamic initialisation on start-up.
 * @brief [zh] 静态池：常量初始化，启动期没有动态初始化。
 */
Slot g_pool[kMaxMotors];
HostSlot g_host_pool[kMaxHostMotors];


/**
 * @brief [en] Map a core error code onto the frozen C enum (the values are identical, by contract).
 * @brief [zh] 把核心错误码映射为冻结的 C 枚举（按契约两者数值相同）。
 */
auto to_c(foc::ErrorCode code) noexcept -> foc_err_t {
    static_assert(static_cast<std::int32_t>(foc::ErrorCode::Ok) == static_cast<std::int32_t>(FOC_OK),
                  "foc::ErrorCode must mirror foc_err_t");
    return static_cast<foc_err_t>(static_cast<std::int32_t>(code));
}


/**
 * @brief [en] Whether the parameters describe a machine the core would accept.
 * @brief [zh] 这些参数是否描述的是一台核心能接受的电机。
 */
auto params_ok(const foc_motor_params_t& p) noexcept -> bool {
    if (!std::isfinite(p.rs) || !std::isfinite(p.ld) || !std::isfinite(p.lq) || !std::isfinite(p.kt) ||
        !std::isfinite(p.j) || !std::isfinite(p.b)) {
        return false;
    }
    return (p.rs >= 0.0F) && (p.ld > 0.0F) && (p.lq > 0.0F) && (p.kt > 0.0F) && (p.j > 0.0F) && (p.b >= 0.0F) &&
           (p.pole_pairs >= 1U);
}


/**
 * @brief [en] Resolve a handle to its slot, rejecting anything that is not a live handle.
 * @brief [zh] 把句柄解析为槽位，拒绝任何不是"活句柄"的东西。
 * @return [en] the slot, or nullptr when the handle is foreign, destroyed or stale.
 * @return [zh] 槽位；句柄不属于本池、已销毁或已失效时返回 nullptr。
 * @post [en] No pointer is dereferenced before the membership check, so a garbage handle cannot fault: this is what
 *            turns undefined behaviour into `FOC_ERR_PARAM`.
 * @post [zh] 成员判定之前不解引用任何指针，因此野句柄不会触发访问异常——这就是把未定义行为变成 `FOC_ERR_PARAM` 的地方。
 * @since 0.1.0
 */
template <typename SlotType, std::size_t N>
auto resolve_in(foc_motor_t handle, SlotType (&pool)[N], std::uintptr_t magic) noexcept -> SlotType* {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, performance-no-int-to-ptr)
    const auto raw = reinterpret_cast<std::uintptr_t>(handle);
    if ((raw >> 32U) != magic) {
        return nullptr; // not minted by this pool (or a garbage pointer): no dereference, no fault
    }

    const auto slot_index = static_cast<std::uint32_t>(raw & 0xFFU);
    const auto generation = static_cast<std::uint32_t>((raw >> kSlotBits) & 0xFFFFFFU);
    if (slot_index >= N) {
        return nullptr;
    }

    SlotType& slot = pool[slot_index];
    if (!slot.alive || (slot.generation != generation)) {
        return nullptr; // destroyed, or the slot has been occupied again since (ABA)
    }
    return &slot;
}

auto resolve(foc_motor_t handle) noexcept -> Slot* { return resolve_in(handle, g_pool, kHandleMagic); }

auto resolve_host(foc_motor_t handle) noexcept -> HostSlot* {
    return resolve_in(handle, g_host_pool, kHostHandleMagic);
}


/**
 * @brief [en] The motor living in a slot.
 * @brief [zh] 槽位中存放的电机。
 */
template <typename SlotType> auto motor_of(SlotType& slot) noexcept -> auto& { return *slot.instance; }

/**
 * @brief [en] Run one behaviour against whichever pool the handle belongs to.
 * @brief [zh] 对着句柄所属的那个池执行同一个行为。
 * @note [en] This is the single dispatch point: the entry points keep one body each, and the compiler makes the two
 *            instantiations. Writing a second copy of `tick` for the host pool is exactly the confusion this avoids.
 * @note [zh] 唯一的派发点：各入口各自只有一个实现，两次实例化由编译器完成。给主控池再抄一份 `tick`，正是这里要避免的混
 *            淆。
 * @since 0.1.2
 */
template <typename Fn> auto on_slot(foc_motor_t handle, Fn&& fn) -> foc_err_t {
    if (Slot* slot = resolve(handle)) {
        return to_c(fn(*slot));
    }
    if (HostSlot* slot = resolve_host(handle)) {
        return to_c(fn(*slot));
    }
    return FOC_ERR_PARAM;
}


/**
 * @brief [en] Run the destructor and drop the pointer.
 * @brief [zh] 调用析构并清掉指针。
 * @note [en] The destructor name has to be written where the alias is visible, which is why this lives here and not in
 *            the C block below.
 * @note [zh] 析构名必须写在该别名可见的作用域里，所以这个函数在这里而不在下面的 C 块里。
 * @since 0.1.0
 */
template <typename SlotType> void destroy_motor(SlotType& slot) noexcept {
    if (slot.instance != nullptr) {
        using MotorType = std::remove_pointer_t<decltype(slot.instance)>;
        slot.instance->~MotorType();
        slot.instance = nullptr;
    }
}

} // namespace

} // namespace api

} // namespace foc



// The C API itself. `extern "C"` comes from bridge.h: these definitions must have C linkage or the
// C side cannot link them. Nothing here is visible to C++ users; they call foc::motor::Motor.
// C API 本体。`extern "C"` 来自 bridge.h：这些定义必须具备 C 链接，否则 C 侧链不上。这里的东西
// 对 C++ 用户不可见，他们直接用 foc::motor::Motor。

namespace foc {

namespace api {

// ---------------------------------------------------------------------------------------------
// ABI 0.1.1: commissioning, protection and the reference policies.
// ABI 0.1.1：整定、保护与参考策略。
//
// Why a second pool and a second magic. A `foc_protector_t` and a `foc_motor_t` are both opaque
// pointers; if they shared a magic, passing one where the other belongs would resolve to a slot and
// read a `Protector` out of a `Motor`'s bytes. Two magics make that a clean `FOC_ERR_STATE`, which
// is the whole reason handles carry their kind.
// 为什么要第二个池子和第二个 magic。`foc_protector_t` 与 `foc_motor_t` 都是不透明指针；若共用
// 一个 magic，把一个当成另一个传进去会解析到一个槽位，并从 `Motor` 的字节里读出一个 `Protector`。
// 两个 magic 把这件事变成一个干净的 `FOC_ERR_STATE`——这正是句柄携带"种类"的全部理由。
// ---------------------------------------------------------------------------------------------

/**
 * @brief [en] High word of every protector handle: "FOCP".
 * @brief [zh] 每个保护器句柄的高位字："FOCP"。
 */
constexpr std::uintptr_t kProtectorMagic = 0x464F4350U;

/**
 * @brief [en] A protector pool smaller than the motor pool: a board has one, maybe two.
 * @brief [zh] 保护器池比电机池小：一块板子有一个，最多两个。
 */
constexpr std::size_t kMaxProtectors = 4U;

/**
 * @brief [en] One protector slot.
 * @brief [zh] 一个保护器槽位。
 * @note [en] `Protector` has no default constructor and holds only floats and enums, so placement new into inline bytes
 *            is enough and no destructor call is needed. That is stated rather than assumed: if the type ever grows a
 *            member that needs releasing, this comment is the thing that makes the omission a bug instead of an
 *            oversight.
 * @note [zh] `Protector` 没有默认构造函数，且只含浮点与枚举，所以内联字节上 placement new 就够， 不需要调析构。这一点是
 *            **写明的**而不是假定的：若该类型将来长了需要释放的成员，这段注释会让 那次遗漏成为 bug 而不是疏忽。
 * @since 0.1.1
 */
struct ProtectorSlot final {
    alignas(core::Protector) unsigned char storage[sizeof(core::Protector)]{};
    core::Protector* instance{nullptr};
    std::uint32_t generation{0U};
    bool alive{false};
};

ProtectorSlot g_protectors[kMaxProtectors];

/**
 * @brief [en] Mint a protector token; same encoding as the motor, different magic.
 * @brief [zh] 签发保护器令牌；编码与电机相同，magic 不同。
 */
auto mint_protector(std::uint32_t slot, std::uint32_t generation) noexcept -> foc_protector_t {
    // The token is never dereferenced: it is a handle value, not an address.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, performance-no-int-to-ptr)
    return reinterpret_cast<foc_protector_t>((kProtectorMagic << 32U) |
                                             (static_cast<std::uintptr_t>(generation) << kSlotBits) |
                                             static_cast<std::uintptr_t>(slot));
}

/**
 * @brief [en] Resolve a protector handle; nullptr for anything not minted here.
 * @brief [zh] 解析保护器句柄；不是本池签发的返回 nullptr。
 */
auto resolve_protector(foc_protector_t handle) noexcept -> ProtectorSlot* {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, performance-no-int-to-ptr)
    const auto raw = reinterpret_cast<std::uintptr_t>(handle);
    if ((raw >> 32U) != kProtectorMagic) {
        return nullptr;
    }
    const auto slot_index = static_cast<std::uint32_t>(raw & 0xFFU);
    const auto generation = static_cast<std::uint32_t>((raw >> kSlotBits) & 0xFFFFFFU);
    if (slot_index >= kMaxProtectors) {
        return nullptr;
    }
    ProtectorSlot& slot = g_protectors[slot_index];
    if (!slot.alive || (slot.generation != generation)) {
        return nullptr;
    }
    return &slot;
}

/**
 * @brief [en] `psi_f` is derived, never taken from the caller: `kt` already encodes it and `psi_f = kt/(1.5*p)` is the
 *             single definition.
 * @brief [zh] `psi_f` 由 `kt` 推导，绝不向调用方索取：`kt` 已经包含它，而 `psi_f = kt/(1.5·p)` 是唯一定义。
 */
auto machine_params(const foc_motor_params_t& p) noexcept -> core::MachineParams {
    const float poles = static_cast<float>(p.pole_pairs);
    return core::MachineParams{p.rs, p.ld, p.lq, p.kt / (1.5F * poles)};
}


/**
 * @brief [en] The observer the C path builds: the nameplate, one explicit kind, and nothing the caller has to repeat.
 * @brief [zh] C 路径构造的观测器：铭牌、一个显式的 kind，不让调用方重复任何数字。
 * @param p [en] the caller's nameplate.
 * @param p [zh] 调用方的铭牌。
 * @param kind [en] which of the two the slot starts on.
 * @param kind [zh] 该槽位起始选哪个。
 * @note [en] `psi_f` is **derived**, not requested: `machine_params()` already states the single definition `psi_f = kt
 *            / (1.5 * pole_pairs)` and this function reuses it rather than opening a second one. An earlier version of
 *            this comment claimed the boundary had no way to know the flux linkage and passed zero; that was wrong
 *            twice over -- `pole_pairs` is in `foc_motor_params_t` (it is how the machine's `psi_f` is reached at all),
 *            and the derivation is exact rather than an approximation. The NLFO's three tuning numbers are the ones
 *            this repository already uses everywhere (`gain_ratio` 0.3 is a fraction of `omega_e` and therefore
 *            dimensionless; the floor `min_gain_rad_s` 3.0 only binds below `3.0/0.3 = 10 rad/s`; `min_flux_wb` is half
 *            the nameplate flux so that "no answer below this flux" sits at 50 % of nominal). They are stated here as
 *            the library's defaults, to be made settable when the caller needs to differ -- not silently re-guessed per
 *            call.
 * @note [zh] `psi_f` 是**推导**来的，不向调用方索取：`machine_params()` 已经给出唯一定义 `psi_f = kt /
 *            (1.5·pole_pairs)`，这里复用它而不是另开一个。这段注释的上一版声称该边界无从得知磁链、于是传了 0；那个说法
 *            错了两处——`pole_pairs` 就在 `foc_motor_params_t` 里（电机的 `psi_f` 正是这样得到的），而这个推导是精确的，
 *            不是近似。 NLFO 的三个整定量就是本仓库各处一直在用的那三个（`gain_ratio` 0.3 是 `ω_e` 的比例、因而无量纲；
 *            地板 `min_gain_rad_s` 3.0 只在 `3.0/0.3 = 10 rad/s` 以下起作用；`min_flux_wb` 取铭牌磁链的一半，于是"低于
 *            此磁链不报答案"落在额定的 50 %）。此处作为库默认值写明，等调用方确实需要不同值时再开放，而不是每次调用静默
 *            重新猜一遍。
 * @since 0.1.5
 */
constexpr auto observer_mode(core::ObserverKind kind) noexcept -> std::int32_t {
    switch (kind) {
    case core::ObserverKind::NonlinearFlux:
        return FOC_OBSERVER_NONLINEAR_FLUX;
    case core::ObserverKind::Hfi:
        return FOC_OBSERVER_HFI;
    case core::ObserverKind::Blended:
        return FOC_OBSERVER_BLENDED;
    case core::ObserverKind::BackEmf:
    default:
        return FOC_OBSERVER_BACK_EMF;
    }
}

constexpr auto observer_kind(std::int32_t observer) noexcept -> core::ObserverKind {
    switch (observer) {
    case FOC_OBSERVER_NONLINEAR_FLUX:
        return core::ObserverKind::NonlinearFlux;
    case FOC_OBSERVER_HFI:
        return core::ObserverKind::Hfi;
    case FOC_OBSERVER_BLENDED:
        return core::ObserverKind::Blended;
    case FOC_OBSERVER_BACK_EMF:
    default:
        return core::ObserverKind::BackEmf;
    }
}

inline auto make_observer(const foc_motor_params_t& p, const Setup& setup) noexcept -> core::SwitchableObserver {
    const core::MachineParams machine = machine_params(p);
    const float psi_f = machine.psi_f;

    core::SwitchableObserver::Config config{};
    config.back_emf = core::BackEmfObserver::Config{machine.rs, machine.ld};
    config.nonlinear_flux =
        core::NonlinearFluxObserver::Config{machine.rs, machine.lq, psi_f, 0.3F, 3.0F, 0.5F * psi_f};
    // The injection half is configured exactly the way `test_sil_hfi.cpp` does, because those are the numbers that were
    // measured: the high-pass at 300 Hz (above any fundamental this machine can make, far below the 2 kHz carrier), the
    // demodulation filter at 200 Hz, damping 0.9, and the sensitivity is the observer's own `predicted_gain_a()` at
    // unit scale rather than a hand-tuned figure. The high-frequency current floor is 0.05 A. The amplitude and the
    // frequency are the caller's to choose and are left at zero until the mode is admitted, so this half is
    // constructed but unreachable -- the slot has to be able to hold it before it can be selected.
    // 注入那一半的配置与 `test_sil_hfi.cpp` 完全一致，因为那些是实测过的数字：高通 300 Hz（高于本机能产生的任何基频、
    // 远低于 2 kHz 载波），解调滤波 200 Hz，阻尼 0.9，灵敏度取观测器自己的 `predicted_gain_a()`、比例 1，而不是手调
    // 出来的数。高频电流下限 0.05 A。幅值与频率由调用方决定，在该模式被放行之前保持为零，因此这一半是"构造了但不可达"
    // ——槽位必须先能装得下它，然后才谈得上选中它。
    config.hfi.inject_v = setup.inject_v;
    config.hfi.inject_hz = setup.inject_hz;
    config.hfi.current_hp_hz = 300.0F;
    config.hfi.demod_lpf_hz = kHfiDemodLpfHz;
    config.hfi.pll_bandwidth_hz = 40.0F;
    config.hfi.pll_zeta = 0.9F;
    config.hfi.demod_gain_a =
        core::HfiObserver::predicted_gain_a(setup.inject_v, setup.inject_hz, machine.ld, machine.lq);
    config.hfi.min_hf_current_a = 0.05F;
    config.blended.blend_start_rad_s = setup.blend_start_rad_s;
    config.blended.blend_end_rad_s = setup.blend_end_rad_s;
    config.blended.handover_lpf_hz = setup.handover_lpf_hz;
    config.kind = observer_kind(setup.observer);
    return core::SwitchableObserver(config);
}

/**
 * @brief [en] Map the caller's enumeration onto the slot's kind; only the admitted values arrive here.
 * @brief [zh] 把调用方的枚举映射到槽位的 kind；只有被放行的取值会到达这里。
 * @since 0.1.5
 */



/**
 * @brief [en] The SVPWM linear range, `1/sqrt(3)`; the modulator's property, not the machine's.
 * @brief [zh] SVPWM 线性区 `1/√3`；这是调制器的性质，不是电机的。
 */
constexpr float kSvpwmLinearRatio = 0.5773502691896257F;

/**
 * @brief [en] The library's default protection config: the nominal 24 V / 3 A surface machine.
 * @brief [zh] 库默认保护配置：标称 24 V / 3 A 表贴机。
 */
auto default_protection_config() noexcept -> core::ProtectionConfig {
    core::ProtectionConfig c{};
    c.i_hard_a = 4.0F;
    c.i_max_a = 3.0F;
    c.i2t_a2s = 4.5F; // i_cont^2 * tau = 9 A^2 * 0.5 s
    c.i2t_cooling_s = 0.5F;
    c.v_max_v = 30.0F;
    c.v_min_v = 18.0F;
    c.t_trip_c = 100.0F;
    c.t_derate_start_c = 60.0F;
    c.t_derate_end_c = 110.0F;
    c.stall_speed_rad_s = 5.0F;
    c.stall_current_a = 1.0F;
    c.stall_time_s = 0.5F;
    c.sync_error_ratio = 0.25F;
    c.hysteresis_ratio = 0.05F;
    return c;
}

/**
 * @brief [en] Copy a C config into the core one, field for field, no interpretation.
 * @brief [zh] 把 C 配置逐字段拷进核心配置，不做任何解释。
 */
auto protection_config(const foc_protection_config_t& in) noexcept -> core::ProtectionConfig {
    core::ProtectionConfig c{};
    c.i_hard_a = in.i_hard_a;
    c.i_max_a = in.i_max_a;
    c.i2t_a2s = in.i2t_a2s;
    c.i2t_cooling_s = in.i2t_cooling_s;
    c.v_max_v = in.v_max_v;
    c.v_min_v = in.v_min_v;
    c.t_trip_c = in.t_trip_c;
    c.t_derate_start_c = in.t_derate_start_c;
    c.t_derate_end_c = in.t_derate_end_c;
    c.stall_speed_rad_s = in.stall_speed_rad_s;
    c.stall_current_a = in.stall_current_a;
    c.stall_time_s = in.stall_time_s;
    c.sync_error_ratio = in.sync_error_ratio;
    c.hysteresis_ratio = in.hysteresis_ratio;
    return c;
}

/**
 * @brief [en] Does the caller's `struct_size` cover the struct we are about to write?
 * @brief [zh] 调用方的 `struct_size` 覆盖得住我们要写的结构吗？
 */
template <typename T> auto size_ok(const T& value) noexcept -> bool {
    return value.struct_size == static_cast<std::uint32_t>(sizeof(T));
}

/**
 * @brief [en] Copy the C callback set into the policy struct the driver uses.
 * @brief [zh] 把 C 回调集拷进驱动使用的策略结构。
 * @return [en] the policy struct; `valid()` on it reports whether the required callbacks were present.
 * @return [zh] 策略结构；其 `valid()` 报告必需回调是否齐全。
 * @note [en] Field by field rather than a byte copy: the two layouts differ by the `struct_size` header, and an
 *            accidental match today would be a silent misread tomorrow.
 * @note [zh] 逐字段而不是整块拷贝：两个布局差一个 `struct_size` 头，今天的偶然对齐就是明天的静默误读。
 * @since 0.1.2
 */
auto to_hal_callbacks(const foc_hal_callbacks_t& c) noexcept -> driver::HalCallbacks {
    driver::HalCallbacks callbacks{};
    callbacks.user = c.user;
    callbacks.pwm_write_duty = c.pwm_write_duty;
    callbacks.read_current = c.read_current;
    callbacks.read_angle_mech_rad = c.read_angle_mech_rad;
    callbacks.read_voltage_v = c.read_voltage_v;
    callbacks.read_velocity_rad_s = c.read_velocity_rad_s;
    callbacks.pwm_fault = c.pwm_fault;
    callbacks.pwm_enable = c.pwm_enable;
    callbacks.pwm_disable = c.pwm_disable;
    callbacks.now_us = c.now_us;
    return callbacks;
}


} // namespace api

} // namespace foc

extern "C" {


foc_err_t foc_bridge_create(const foc_motor_params_t* params, foc_motor_t* out) {
    return foc_bridge_create_ex(params, nullptr, out);
}


foc_err_t foc_bridge_create_ex(const foc_motor_params_t* params, const foc_motor_setup_t* setup, foc_motor_t* out) {
    if ((params == nullptr) || (out == nullptr)) {
        return FOC_ERR_PARAM;
    }
    if (!foc::api::params_ok(*params)) {
        return FOC_ERR_PARAM;
    }

    const foc::api::Setup resolved = foc::api::resolve_setup(setup);
    if (!foc::api::setup_ok(resolved)) {
        return FOC_ERR_PARAM;
    }
    const foc_err_t observer = foc::api::observer_status(resolved, *params);
    if (observer != FOC_OK) {
        return observer;
    }

    for (std::size_t i = 0U; i < foc::api::kMaxMotors; ++i) {
        foc::api::Slot& slot = foc::api::g_pool[i];
        if (slot.alive) {
            continue;
        }

        // One resolver builds the nameplate and every loop configuration, so the gain query and the motor
        // constructor cannot disagree about what the shipped loop runs with.
        // 一个解析函数构造铭牌与全部环路配置，于是增益查询与电机构造函数不会对出厂环路用的是什么产生分歧。
        const foc::api::Resolved r = foc::api::resolve_config(*params, resolved);
        const foc::motor::MotorConfig& config = r.config;
        const foc::core::Pi::Config& current = r.current;
        const foc::core::Pi::Config& speed = r.speed;
        const foc::core::PositionP::Config& position = r.position;
        // An all-zero recipe is the documented "no alignment" state: the aligner then answers
        // `NotImpl` from validate() and start() goes straight to Run.
        // 全零配方就是文档化的“不对齐”状态：此时对齐器在 validate() 答 `NotImpl`，start() 直接进 Run。
        const foc::core::AlignConfig alignment{resolved.align_current_a, resolved.align_ramp_s,
                                               resolved.align_hold_s,    resolved.align_tolerance_rad,
                                               resolved.align_verify_s,  resolved.align_angle_e_rad};

        slot.hal.reset();
        slot.hal.dc_link.set_voltage_v(resolved.v_bus_v);

        const foc::api::MotorT::Policies policies{foc::api::make_observer(*params, resolved),
                                                  foc::core::Svpwm{},
                                                  foc::core::Pi{current},
                                                  foc::core::Pi{current},
                                                  foc::core::Pi{speed},
                                                  foc::core::NullReferenceGenerator{},
                                                  foc::core::NullFeedforward{},
                                                  foc::core::PositionP{position},
                                                  foc::core::AlignRoutine{alignment}};

        slot.instance = new (static_cast<void*>(slot.storage)) foc::api::MotorT(config, policies, slot.hal);

        const foc::ErrorCode rc = slot.instance->init();
        if (rc != foc::ErrorCode::Ok) {
            foc::api::destroy_motor(slot);
            return foc::api::to_c(rc);
        }

        slot.setup = resolved;
        ++slot.generation;
        slot.alive = true;
        *out = foc::api::mint_handle(static_cast<std::uint32_t>(i), slot.generation);
        return FOC_OK;
    }

    return FOC_ERR_NO_RESOURCE;
}


foc_err_t foc_bridge_create_host(const foc_motor_params_t* params, const foc_motor_setup_t* setup,
                                 const foc_hal_callbacks_t* callbacks, foc_motor_t* out) {
    if ((params == nullptr) || (out == nullptr) || (callbacks == nullptr)) {
        return FOC_ERR_PARAM;
    }
    if (!foc::api::params_ok(*params)) {
        return FOC_ERR_PARAM;
    }

    const foc::api::Setup resolved = foc::api::resolve_setup(setup);
    if (!foc::api::setup_ok(resolved)) {
        return FOC_ERR_PARAM;
    }
    const foc_err_t observer = foc::api::observer_status(resolved, *params);
    if (observer != FOC_OK) {
        return observer;
    }

    const foc::driver::HalCallbacks hal_callbacks = foc::api::to_hal_callbacks(*callbacks);
    if (!hal_callbacks.valid()) {
        return FOC_ERR_PARAM;
    }

    for (std::size_t i = 0U; i < foc::api::kMaxHostMotors; ++i) {
        foc::api::HostSlot& slot = foc::api::g_host_pool[i];
        if (slot.alive) {
            continue;
        }

        // One resolver builds the nameplate and every loop configuration, so the gain query and the motor
        // constructor cannot disagree about what the shipped loop runs with.
        // 一个解析函数构造铭牌与全部环路配置，于是增益查询与电机构造函数不会对出厂环路用的是什么产生分歧。
        const foc::api::Resolved r = foc::api::resolve_config(*params, resolved);
        const foc::motor::MotorConfig& config = r.config;
        const foc::core::Pi::Config& current = r.current;
        const foc::core::Pi::Config& speed = r.speed;
        const foc::core::PositionP::Config& position = r.position;
        const foc::core::AlignConfig alignment{resolved.align_current_a, resolved.align_ramp_s,
                                               resolved.align_hold_s,    resolved.align_tolerance_rad,
                                               resolved.align_verify_s,  resolved.align_angle_e_rad};

        slot.hal = foc::driver::CallbackHal(hal_callbacks);
        slot.setup = resolved;

        const foc::api::MotorT_Host::Policies policies{foc::api::make_observer(*params, resolved),
                                                       foc::core::Svpwm{},
                                                       foc::core::Pi{current},
                                                       foc::core::Pi{current},
                                                       foc::core::Pi{speed},
                                                       foc::core::NullReferenceGenerator{},
                                                       foc::core::NullFeedforward{},
                                                       foc::core::PositionP{position},
                                                       foc::core::AlignRoutine{alignment}};

        slot.instance = new (static_cast<void*>(slot.storage)) foc::api::MotorT_Host(config, policies, slot.hal);

        const foc::ErrorCode rc = slot.instance->init();
        if (rc != foc::ErrorCode::Ok) {
            foc::api::destroy_motor(slot);
            return foc::api::to_c(rc);
        }

        ++slot.generation;
        slot.alive = true;
        *out = foc::api::mint_handle(static_cast<std::uint32_t>(i), slot.generation, foc::api::kHostHandleMagic);
        return FOC_OK;
    }

    return FOC_ERR_NO_RESOURCE;
}


foc_err_t foc_bridge_get_setup(foc_motor_t motor, foc_motor_setup_t* out) {
    if (out == nullptr) {
        return FOC_ERR_PARAM;
    }
    return foc::api::on_slot(motor, [out](auto& slot) {
        const foc::api::Setup& applied = slot.setup;
        foc_motor_setup_t result{};
        result.struct_size = sizeof(foc_motor_setup_t);
        result.i_max_a = applied.i_max_a;
        result.pwm_freq_hz = applied.pwm_freq_hz;
        result.deadtime_ns = applied.deadtime_ns;
        result.v_bus_v = applied.v_bus_v;
        result.current_bandwidth_rad_s = applied.current_bandwidth_rad_s;
        result.speed_bandwidth_rad_s = applied.speed_bandwidth_rad_s;
        result.speed_loop_period_s = applied.speed_loop_period_s;
        result.position_bandwidth_rad_s = applied.position_bandwidth_rad_s;
        result.velocity_limit_rad_s = applied.velocity_limit_rad_s;
        result.position_loop_period_s = applied.position_loop_period_s;
        result.position_accel_limit_rad_s2 = applied.position_accel_limit_rad_s2;
        result.align_current_a = applied.align_current_a;
        result.align_ramp_s = applied.align_ramp_s;
        result.align_hold_s = applied.align_hold_s;
        result.align_tolerance_rad = applied.align_tolerance_rad;
        result.align_verify_s = applied.align_verify_s;
        result.align_angle_e_rad = applied.align_angle_e_rad;
        *out = result;
        return foc::ErrorCode::Ok;
    });
}


foc_err_t foc_bridge_destroy(foc_motor_t motor) {
    return foc::api::on_slot(motor, [](auto& slot) {
        foc::api::destroy_motor(slot);
        slot.alive = false;
        ++slot.generation; // invalidates every handle taken from this occupation of the slot
        return foc::ErrorCode::Ok;
    });
}


foc_err_t foc_bridge_enable(foc_motor_t motor) {
    return foc::api::on_slot(motor, [](auto& slot) { return slot.instance->start(); });
}


foc_err_t foc_bridge_disable(foc_motor_t motor) {
    return foc::api::on_slot(motor, [](auto& slot) { return slot.instance->stop(); });
}


foc_err_t foc_bridge_clear_fault(foc_motor_t motor) {
    return foc::api::on_slot(motor, [](auto& slot) { return slot.instance->clear_fault(); });
}


foc_err_t foc_bridge_set_speed(foc_motor_t motor, float rad_per_s) {
    if (!std::isfinite(rad_per_s)) {
        return FOC_ERR_PARAM;
    }
    return foc::api::on_slot(motor, [rad_per_s](auto& slot) {
        slot.instance->set_speed(rad_per_s);
        return foc::ErrorCode::Ok;
    });
}


foc_err_t foc_bridge_set_torque(foc_motor_t motor, float iq_a) {
    if (!std::isfinite(iq_a)) {
        return FOC_ERR_PARAM;
    }
    return foc::api::on_slot(motor, [iq_a](auto& slot) {
        slot.instance->set_torque(iq_a);
        return foc::ErrorCode::Ok;
    });
}


foc_err_t foc_bridge_set_position(foc_motor_t motor, float position_rad) {
    if (!std::isfinite(position_rad)) {
        return FOC_ERR_PARAM;
    }
    return foc::api::on_slot(motor, [position_rad](auto& slot) {
        slot.instance->set_position(position_rad);
        return foc::ErrorCode::Ok;
    });
}


foc_err_t foc_bridge_set_command(foc_motor_t motor, const foc_motor_command_t* command) {
    if (command == nullptr) {
        return FOC_ERR_PARAM;
    }
    // Every field is validated, including `max_current_a`: a negative ceiling would clamp *away*
    // the current, i.e. invert the meaning of the limit instead of bounding it.
    // 每个字段都校验，包括 `max_current_a`：负的限幅会把电流钳成反号，即把限流的含义反过来而不是限住它。
    if (!std::isfinite(command->position_rad) || !std::isfinite(command->max_current_a) ||
        !std::isfinite(command->feedforward_current_a) || (command->max_current_a < 0.0F) ||
        !std::isfinite(command->max_velocity_rad_s) || (command->max_velocity_rad_s < 0.0F) ||
        !std::isfinite(command->max_accel_rad_s2) || (command->max_accel_rad_s2 < 0.0F) ||
        !std::isfinite(command->kp_scale) || (command->kp_scale < 0.0F) || !std::isfinite(command->kd_scale) ||
        (command->kd_scale < 0.0F) || !std::isfinite(command->ilimit_scale) || (command->ilimit_scale < 0.0F)) {
        return FOC_ERR_PARAM;
    }
    return foc::api::on_slot(motor, [command](auto& slot) {
        slot.instance->set_command(foc::motor::Command{
            command->position_rad, command->max_current_a, command->feedforward_current_a, command->max_velocity_rad_s,
            command->max_accel_rad_s2, command->kp_scale, command->kd_scale, command->ilimit_scale});
        return foc::ErrorCode::Ok;
    });
}


foc_err_t foc_bridge_get_status(foc_motor_t motor, foc_motor_status_t* out) {
    if (out == nullptr) {
        return FOC_ERR_PARAM;
    }
    return foc::api::on_slot(motor, [out](auto& slot) {
        auto& instance = *slot.instance;
        const foc::motor::Telemetry telemetry = instance.telemetry();
        // The aligner is always an `AlignRoutine` on this path, so the offset is always readable; the
        // C view simply reports it rather than hiding the field behind a capability check.
        // 本路径上的对齐器固定是 `AlignRoutine`，因此偏置总可读；C 视图直接报出来，而不是把这个字段
        // 藏进能力检查后面。
        const foc::core::AlignRoutine& aligner = instance.aligner();

        static_assert(static_cast<std::int32_t>(foc::core::MotorState::Idle) ==
                          static_cast<std::int32_t>(FOC_MOTOR_IDLE),
                      "foc_motor_state_t must mirror foc::core::MotorState");
        static_assert(static_cast<std::int32_t>(foc::core::MotorState::Align) ==
                          static_cast<std::int32_t>(FOC_MOTOR_ALIGN),
                      "foc_motor_state_t must mirror foc::core::MotorState");
        static_assert(static_cast<std::int32_t>(foc::core::MotorState::Run) == static_cast<std::int32_t>(FOC_MOTOR_RUN),
                      "foc_motor_state_t must mirror foc::core::MotorState");
        static_assert(static_cast<std::int32_t>(foc::core::MotorState::Fault) ==
                          static_cast<std::int32_t>(FOC_MOTOR_FAULT),
                      "foc_motor_state_t must mirror foc::core::MotorState");
        static_assert(static_cast<std::int32_t>(foc::core::AlignState::Done) ==
                          static_cast<std::int32_t>(FOC_ALIGN_DONE),
                      "foc_align_state_t must mirror foc::core::AlignState");
        static_assert(static_cast<std::int32_t>(foc::core::AlignState::Failed) ==
                          static_cast<std::int32_t>(FOC_ALIGN_FAILED),
                      "foc_align_state_t must mirror foc::core::AlignState");
        static_assert(static_cast<std::int32_t>(foc::core::AlignFailure::NotSettled) ==
                          static_cast<std::int32_t>(FOC_ALIGN_NOT_SETTLED),
                      "foc_align_failure_t must mirror foc::core::AlignFailure");
        static_assert(static_cast<std::int32_t>(foc::core::AlignFailure::DirectionWrong) ==
                          static_cast<std::int32_t>(FOC_ALIGN_DIRECTION_WRONG),
                      "foc_align_failure_t must mirror foc::core::AlignFailure");

        // The caller declares how much of their struct they own, exactly as they do for the setup. On an *output*
        // struct an undeclared size cannot mean "I want no fields" -- the call would be pointless -- so 0 keeps the
        // historical meaning of "the version I have", while any declared size is respected as a ceiling. Without this
        // the field below would be written past the end of a caller built against the previous header.
        // 调用方声明自己拥有多大，与 setup 一致。在**输出**结构上，"未声明"不可能表示"我不要任何字段"——那样这次调用毫无
        // 意义——因此 0 沿用"我手头的那个版本"的历史含义，而任何已声明的大小都被当作上限。没有这一步，下面新增的字段会被
        // 写到按旧头编译的调用方结构体之外。
        const std::uint32_t declared = out->struct_size;
        const std::size_t limit =
            (declared == 0U) ? sizeof(foc_motor_status_t) : std::min<std::size_t>(declared, sizeof(foc_motor_status_t));

        foc_motor_status_t result{};
        result.struct_size = sizeof(foc_motor_status_t);
        result.mode_state = static_cast<std::int32_t>(instance.state());
        result.error = static_cast<std::int32_t>(foc::api::to_c(instance.status()));
        result.align_state = static_cast<std::int32_t>(aligner.state());
        result.align_failure = static_cast<std::int32_t>(aligner.failure());
        result.align_offset_rad = aligner.offset_mech_rad();
        result.align_spread_rad = aligner.spread_rad();
        result.max_period_us = telemetry.max_period_us;
        result.observer = foc::api::observer_mode(instance.observer().kind());
        std::memcpy(static_cast<void*>(out), &result, limit);
        return foc::ErrorCode::Ok;
    });
}


foc_err_t foc_bridge_tick(foc_motor_t motor, float dt_s) {
    return foc::api::on_slot(motor, [dt_s](auto& slot) { return slot.instance->tick(dt_s); });
}


foc_err_t foc_bridge_get_state(foc_motor_t motor, float* speed_rad_s, float* id, float* iq) {
    return foc::api::on_slot(motor, [speed_rad_s, id, iq](auto& slot) {
        const foc::motor::Telemetry telemetry = slot.instance->telemetry();
        if (speed_rad_s != nullptr) {
            *speed_rad_s = telemetry.speed_mech_rad_s;
        }
        if (id != nullptr) {
            *id = telemetry.id_a;
        }
        if (iq != nullptr) {
            *iq = telemetry.iq_a;
        }
        return foc::ErrorCode::Ok;
    });
}


// ---------------------------------------------------------------------------------------------
// ABI 0.1.1 entry points.
// ABI 0.1.1 入口。
// ---------------------------------------------------------------------------------------------

float foc_bridge_axis_deadtime_v(float deadtime_s, float switching_hz, float v_bus_v, float angle_rad) {
    if (!std::isfinite(deadtime_s) || !std::isfinite(switching_hz) || !std::isfinite(v_bus_v) ||
        !std::isfinite(angle_rad)) {
        return 0.0F;
    }
    // The volt-seconds the dead time costs, then its projection onto the axis. Two steps because
    // they are two different conversions and conflating them is how a 33 % error gets made.
    // 死区造成的伏秒，再投影到轴上。分两步是因为它们是两种不同的换算，混为一谈正是 33 % 误差的
    // 来源。
    const float deadtime_v = deadtime_s * switching_hz * v_bus_v;
    return foc::core::axis_deadtime_v(deadtime_v, angle_rad);
}

foc_err_t foc_bridge_fw_worthwhile(const foc_motor_params_t* params, float i_max_a, foc_fw_verdict_t* out) {
    if ((params == nullptr) || (out == nullptr) || !foc::api::size_ok(*out) || !std::isfinite(i_max_a)) {
        return FOC_ERR_PARAM;
    }
    const foc::core::MachineParams m = foc::api::machine_params(*params);
    const auto verdict = foc::core::is_field_weakening_worthwhile(m, i_max_a);
    out->i_char_a = verdict.i_char_a;
    out->ratio = verdict.ratio;
    out->speed_gain = verdict.speed_gain;
    out->worthwhile = verdict.worthwhile ? 1 : 0;
    return FOC_OK;
}

foc_err_t foc_bridge_mtpa_reference(const foc_motor_params_t* params, float i_max_a, float iq_command_a, float* id_a,
                                    float* iq_a) {
    if ((params == nullptr) || (id_a == nullptr) || (iq_a == nullptr) || !std::isfinite(iq_command_a)) {
        return FOC_ERR_PARAM;
    }
    const foc::core::MachineParams m = foc::api::machine_params(*params);
    // This entry point has no speed argument, so it cannot apply the d-axis allowance that
    // `MtpaReference` enforces with one -- it answers the MTPA *curve*, which is what its name and its
    // stable-ABI documentation promise. An infinite rate is how the curve is asked for without a limit;
    // a caller that wants the guard asks the C++ policy, which knows the speed.
    // 这个入口没有转速参数，所以施加不了 `MtpaReference` 用转速施加的那道 d 轴许可——它回答的是 MTPA
    // **曲线**，这正是它的名字与稳定 ABI 文档所承诺的。无限比率就是“不带上限地问曲线”的写法；想要那道
    // 门的调用方去问知道转速的 C++ 策略。
    foc::core::MtpaReference reference(
        foc::core::MtpaReference::Config{m.ld, m.lq, m.psi_f, i_max_a, std::numeric_limits<float>::infinity()});
    foc::core::ReferenceInput in{};
    in.iq_command_a = iq_command_a;
    const foc::core::Dq<float> i = reference.generate(in);
    *id_a = i.d;
    *iq_a = i.q;
    return FOC_OK;
}


foc_err_t foc_bridge_current_pi_gains(const foc_motor_params_t* params, const foc_motor_setup_t* setup,
                                      foc_pi_gains_t* out) {
    if ((params == nullptr) || (out == nullptr) ||
        (static_cast<std::size_t>(out->struct_size) < sizeof(foc_pi_gains_t))) {
        return FOC_ERR_PARAM;
    }
    const foc::api::Setup resolved = foc::api::resolve_setup(setup);
    if (!foc::api::setup_ok(resolved)) {
        return FOC_ERR_PARAM;
    }
    const foc::api::Resolved r = foc::api::resolve_config(*params, resolved);
    out->kp = r.current.kp;
    out->ki = r.current.ki;
    out->out_min = r.current.out_min;
    out->out_max = r.current.out_max;
    out->integral_min = r.current.integral_min;
    out->integral_max = r.current.integral_max;
    return FOC_OK;
}


foc_err_t foc_bridge_speed_pi_gains(const foc_motor_params_t* params, const foc_motor_setup_t* setup,
                                    foc_pi_gains_t* out) {
    if ((params == nullptr) || (out == nullptr) ||
        (static_cast<std::size_t>(out->struct_size) < sizeof(foc_pi_gains_t))) {
        return FOC_ERR_PARAM;
    }
    const foc::api::Setup resolved = foc::api::resolve_setup(setup);
    if (!foc::api::setup_ok(resolved)) {
        return FOC_ERR_PARAM;
    }
    const foc::api::Resolved r = foc::api::resolve_config(*params, resolved);
    out->kp = r.speed.kp;
    out->ki = r.speed.ki;
    out->out_min = r.speed.out_min;
    out->out_max = r.speed.out_max;
    out->integral_min = r.speed.integral_min;
    out->integral_max = r.speed.integral_max;
    return FOC_OK;
}


foc_err_t foc_bridge_rs_estimate(const foc_motor_params_t* params, const float* v_command, const float* i_current,
                                 std::uint32_t count, foc_identify_result_t* out) {
    if ((params == nullptr) || (v_command == nullptr) || (i_current == nullptr) || (count == 0U) || (out == nullptr) ||
        (static_cast<std::size_t>(out->struct_size) < sizeof(foc_identify_result_t))) {
        return FOC_ERR_PARAM;
    }
    const foc::api::Setup resolved = foc::api::resolve_setup(nullptr);
    // No invented floor: which currents are informative depends on the board's dead zone, which this entry point has no
    // way to know. The caller decides what to feed, and `valid` is where the estimator answers whether it was enough.
    // 不凭空设下限：哪些电流有信息量取决于板子的死区，这个入口无从知道。由调用方决定喂什么，而估计器是否够用，
    // 由 `valid` 回答。
    foc::core::RsEstimator estimator(foc::core::RsEstimator::Config{0.0F, resolved.i_max_a});
    std::uint32_t used = 0U;
    for (std::uint32_t i = 0U; i < count; ++i) {
        used += estimator.add(v_command[i], i_current[i]) ? 1U : 0U;
    }
    const foc::core::RsEstimator::Result r = estimator.solve();
    out->value = r.rs;
    out->offset = r.deadtime_v;
    out->residual = r.residual_v;
    out->valid = r.valid ? 1 : 0;
    out->samples = static_cast<std::int32_t>(used);
    return FOC_OK;
}


foc_err_t foc_bridge_flux_estimate(const foc_motor_params_t* params, const float* omega_e, const float* v_magnitude,
                                   const float* i_magnitude, std::uint32_t count, foc_identify_result_t* out) {
    if ((params == nullptr) || (omega_e == nullptr) || (v_magnitude == nullptr) || (i_magnitude == nullptr) ||
        (count == 0U) || (out == nullptr) ||
        (static_cast<std::size_t>(out->struct_size) < sizeof(foc_identify_result_t))) {
        return FOC_ERR_PARAM;
    }
    const foc::api::Setup resolved = foc::api::resolve_setup(nullptr);
    // The zero-current ceiling and the speed floor are both left open for the same reason as above: the intercept of
    // the fit absorbs whatever constant the board adds, and a floor would be a policy the caller did not ask for.
    // 零电流上限与转速下限都留空，理由同上：拟合的截距会吸收板子加上的常量，而设一个下限等于替调用方做决定。
    foc::core::FluxEstimator estimator(foc::core::FluxEstimator::Config{0.1F * resolved.i_max_a, 0.0F});
    std::uint32_t used = 0U;
    for (std::uint32_t i = 0U; i < count; ++i) {
        used += estimator.add(omega_e[i], v_magnitude[i], i_magnitude[i]) ? 1U : 0U;
    }
    const foc::core::FluxEstimator::Result r = estimator.solve();
    out->value = r.psi_f;
    out->offset = r.offset_v;
    out->residual = r.residual_v;
    out->valid = r.valid ? 1 : 0;
    out->samples = static_cast<std::int32_t>(used);
    return FOC_OK;
}

foc_err_t foc_bridge_board_check(const foc_board_measurements_t* measurements, const foc_board_limits_t* limits,
                                 foc_board_report_t* out) {
    if ((measurements == nullptr) || (limits == nullptr) || (out == nullptr) || !foc::api::size_ok(*measurements) ||
        !foc::api::size_ok(*limits) || !foc::api::size_ok(*out)) {
        return FOC_ERR_PARAM;
    }
    foc::core::BoardMeasurements m{};
    m.sum_current_a = measurements->sum_current_a;
    for (int k = 0; k < 3; ++k) {
        m.offset_a[k] = measurements->offset_a[k];
        m.response_sign[k] = measurements->response_sign[k];
    }
    m.ripple_measured_a = measurements->ripple_measured_a;
    m.ripple_expected_a = measurements->ripple_expected_a;
    m.deadtime_v = measurements->deadtime_v;
    m.deadtime_cmd_v = measurements->deadtime_cmd_v;
    m.encoder_offset_rad = measurements->encoder_offset_rad;
    m.rs_ohm = measurements->rs_ohm;
    m.rs_nameplate_ohm = measurements->rs_nameplate_ohm;
    m.psi_f_wb = measurements->psi_f_wb;
    m.psi_f_nameplate_wb = measurements->psi_f_nameplate_wb;
    m.observer_error_rad = measurements->observer_error_rad;

    foc::core::BoardLimits l{};
    l.sum_current_a = limits->sum_current_a;
    l.offset_a = limits->offset_a;
    l.ripple_ratio = limits->ripple_ratio;
    l.deadtime_ratio = limits->deadtime_ratio;
    l.encoder_offset_rad = limits->encoder_offset_rad;
    l.parameter_ratio = limits->parameter_ratio;
    l.observer_error_rad = limits->observer_error_rad;

    const foc::core::BoardReport report = foc::core::check_board(m, l);
    for (int k = 0; k < FOC_BOARD_MAX_ITEMS; ++k) {
        out->items[k].item = 0;
        out->items[k].value = 0.0F;
        out->items[k].limit = 0.0F;
        out->items[k].pass = 0;
        out->items[k].phase = 3;
    }
    const int rows = (report.count < FOC_BOARD_MAX_ITEMS) ? report.count : FOC_BOARD_MAX_ITEMS;
    for (int k = 0; k < rows; ++k) {
        out->items[k].item = static_cast<std::int32_t>(report.items[k].item);
        out->items[k].value = report.items[k].value;
        out->items[k].limit = report.items[k].limit;
        out->items[k].pass = report.items[k].pass ? 1 : 0;
        out->items[k].phase = static_cast<std::int32_t>(report.items[k].phase);
    }
    out->count = rows;
    out->failures = report.failures;
    out->cause = static_cast<std::int32_t>(report.cause);
    out->cause_phase = static_cast<std::int32_t>(report.cause_phase);
    return FOC_OK;
}

const char* foc_bridge_board_item_name(std::int32_t item) {
    // Range-checked before the cast: an out-of-range int32 is not a valid enumerator, and the cast
    // would be undefined behaviour rather than a lookup miss.
    // 先查范围再转型：越界的 int32 不是合法枚举值，转型会是未定义行为而不是"查不到"。
    if ((item < static_cast<std::int32_t>(foc::core::BoardItem::CurrentSum)) ||
        (item > static_cast<std::int32_t>(foc::core::BoardItem::ObserverHealth))) {
        return "?";
    }
    return foc::core::board_item_name(static_cast<foc::core::BoardItem>(item));
}

const char* foc_bridge_board_cause_name(std::int32_t cause) {
    if ((cause < static_cast<std::int32_t>(foc::core::BoardCause::None)) ||
        (cause > static_cast<std::int32_t>(foc::core::BoardCause::ObserverUnhealthy))) {
        return "?";
    }
    return foc::core::board_cause_name(static_cast<foc::core::BoardCause>(cause));
}

const char* foc_bridge_fault_name(std::int32_t fault) {
    // Range-checked before the cast: an out-of-range int32 is not a valid enumerator.
    // 先查范围再转型：越界的 int32 不是合法枚举值。
    if ((fault < static_cast<std::int32_t>(foc::core::FaultCode::None)) ||
        (fault > static_cast<std::int32_t>(foc::core::FaultCode::LostSync))) {
        return "?";
    }
    switch (static_cast<foc::core::FaultCode>(fault)) {
    case foc::core::FaultCode::None:
        return "none";
    case foc::core::FaultCode::OverCurrent:
        return "over current";
    case foc::core::FaultCode::OverVoltage:
        return "over voltage";
    case foc::core::FaultCode::UnderVoltage:
        return "under voltage";
    case foc::core::FaultCode::OverTemperature:
        return "over temperature";
    case foc::core::FaultCode::I2t:
        return "i2t";
    case foc::core::FaultCode::Stall:
        return "stall";
    case foc::core::FaultCode::LostSync:
        return "lost sync";
    }
    return "?";
}

foc_err_t foc_bridge_protector_create(const foc_protection_config_t* config, foc_protector_t* out) {
    if (out == nullptr) {
        return FOC_ERR_PARAM;
    }
    if (config != nullptr && !foc::api::size_ok(*config)) {
        return FOC_ERR_PARAM;
    }
    std::size_t index = foc::api::kMaxProtectors;
    for (std::size_t k = 0; k < foc::api::kMaxProtectors; ++k) {
        if (!foc::api::g_protectors[k].alive) {
            index = k;
            break;
        }
    }
    if (index >= foc::api::kMaxProtectors) {
        return FOC_ERR_NO_RESOURCE;
    }
    foc::api::ProtectorSlot& slot = foc::api::g_protectors[index];
    const foc::core::ProtectionConfig cfg =
        (config != nullptr) ? foc::api::protection_config(*config) : foc::api::default_protection_config();
    slot.instance = new (slot.storage) foc::core::Protector(cfg);
    slot.alive = true;
    slot.generation = (slot.generation + 1U) & 0xFFFFFFU;
    *out = foc::api::mint_protector(static_cast<std::uint32_t>(index), slot.generation);
    return FOC_OK;
}

foc_err_t foc_bridge_protector_step(foc_protector_t protector, const foc_protection_input_t* in, float dt_s,
                                    foc_protection_output_t* out) {
    if ((in == nullptr) || (out == nullptr) || !foc::api::size_ok(*in) || !foc::api::size_ok(*out) ||
        !std::isfinite(dt_s)) {
        return FOC_ERR_PARAM;
    }
    foc::api::ProtectorSlot* slot = foc::api::resolve_protector(protector);
    if (slot == nullptr) {
        return FOC_ERR_STATE;
    }
    foc::core::ProtectionInput input{};
    input.i_magnitude_a = in->i_magnitude_a;
    input.v_bus_v = in->v_bus_v;
    input.temperature_c = in->temperature_c;
    input.omega_e_rad_s = in->omega_e_rad_s;
    input.flux_error_ratio = in->flux_error_ratio;
    input.flux_valid = (in->flux_valid != 0);

    const foc::core::ProtectionOutput result = slot->instance->step(input, dt_s);
    out->fault = static_cast<std::int32_t>(result.fault);
    out->active_mask = static_cast<std::uint32_t>(result.active_mask);
    out->latched_mask = static_cast<std::uint32_t>(result.latched_mask);
    out->i_limit_a = result.i_limit_a;
    out->output_allowed = result.output_allowed ? 1 : 0;
    out->i2t_budget = slot->instance->thermal_budget();
    return FOC_OK;
}

foc_err_t foc_bridge_protector_reset(foc_protector_t protector, std::int32_t cold) {
    foc::api::ProtectorSlot* slot = foc::api::resolve_protector(protector);
    if (slot == nullptr) {
        return FOC_ERR_STATE;
    }
    if (cold != 0) {
        slot->instance->reset_cold();
    } else {
        slot->instance->reset();
    }
    return FOC_OK;
}

foc_err_t foc_bridge_protector_destroy(foc_protector_t protector) {
    if (protector == nullptr) {
        return FOC_OK;
    }
    foc::api::ProtectorSlot* slot = foc::api::resolve_protector(protector);
    if (slot == nullptr) {
        return FOC_ERR_STATE;
    }
    // No destructor call: `Protector` holds only floats and enums. See foc::api::ProtectorSlot's note.
    // 不调析构：`Protector` 只含浮点与枚举。见 foc::api::ProtectorSlot 的注释。
    slot->instance = nullptr;
    slot->alive = false;
    slot->generation = (slot->generation + 1U) & 0xFFFFFFU; // invalidate outstanding handles
    return FOC_OK;
}

} // extern "C"
