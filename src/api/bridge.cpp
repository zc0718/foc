// Conan::ImportStart
#include "bridge.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <new>

#include <foc/core/alignment.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
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
using MotorT = motor::Motor<core::BackEmfObserver, core::Svpwm, core::Pi, driver::StubHal, core::NullReferenceGenerator,
                            core::NullFeedforward, core::PositionP, core::AlignRoutine>;

/// [en] Object pool capacity: static storage, no heap, no fragmentation.
/// [zh] 对象池容量：静态存储、无堆、无碎片。
constexpr std::size_t kMaxMotors = 4U;

/**
 * @brief [en] The values the C path runs with. Defaults live here; the caller overrides them.
 *        [zh] C 路径实际使用的取值。默认值放在这里，由调用方覆盖。
 *
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
    float position_loop_period_s;   ///< [en] position-loop period, s (0 = every tick).
                                    ///< [zh] 位置环周期，s（0 = 每 tick）。
    float position_accel_limit_rad_s2; ///< [en] position velocity rate limit (0 = none).
                                       ///< [zh] 位置速度指令的加速度上限（0 = 不限）。
    float align_current_a;     ///< [en] alignment current, A (0 = do not align). [zh] 对齐电流，A（0 = 不对齐）。
    float align_ramp_s;        ///< [en] alignment ramp, s. [zh] 对齐爬升，s。
    float align_hold_s;        ///< [en] alignment hold window, s. [zh] 对齐保持窗口，s。
    float align_tolerance_rad; ///< [en] accepted spread, rad. [zh] 允许离散度，rad。
    float align_verify_s;      ///< [en] direction check hold, s. [zh] 方向校验保持，s。
    float align_angle_e_rad;   ///< [en] forced electrical angle, rad. [zh] 强制电角度，rad。
};


/// [en] What `foc_motor_create()` has always used. [zh] `foc_motor_create()` 一直使用的取值。
constexpr Setup kDefaultSetup{5.0F, 20000.0F, 500.0F, 24.0F, 2000.0F, 200.0F, 0.0F, 20.0F, 50.0F,
                              0.0F, 0.0F,     0.0F,   0.0F,  0.0F,    0.0F,   0.0F, 0.0F};


/**
 * @brief [en] Whether the caller's struct is large enough to contain the field ending at
 *        `end_offset`.
 *        [zh] 调用方的结构是否足够大，能包含以 `end_offset` 结尾的那个字段。
 * @since 0.1.0
 */
auto covers(const foc_motor_setup_t& setup, std::size_t end_offset) noexcept -> bool {
    return static_cast<std::size_t>(setup.struct_size) >= end_offset;
}


/**
 * @brief [en] Merge the caller's setup over the defaults, reading only what their `struct_size`
 *        covers, so an older and shorter struct is safe by construction rather than by convention.
 *        [zh] 把调用方的设置叠在默认值上，只读取其 `struct_size` 覆盖到的字段；于是更短的旧结构是
 *        “结构上安全”而不是“约定安全”。
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
 *        [zh] 合并后的设置是否描述了一台核心能接受的电机。
 * @since 0.1.0
 */
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
 * @brief [en] One pool slot: inline storage for the motor plus the control block the handle points
 *        at. `storage` is the only place a `Motor<>` ever lives on the C path.
 *        [zh] 一个池槽：电机内联存储 + 句柄指向的控制块。`storage` 是 C 路径上 `Motor<>` 唯一
 *        存在的地方。
 *
 * @since 0.1.0
 */
struct Slot final {
    alignas(MotorT) unsigned char storage[sizeof(MotorT)]{};
    MotorT* instance{nullptr};
    driver::StubHal hal{};
    Setup setup{};
    std::uint32_t generation{0U};
    bool alive{false};
};

/// [en] High word of every handle token: "FOC\0". Anything else is not ours.
/// [zh] 每个句柄令牌的高位字："FOC\0"。别的值都不是本池签发的。
constexpr std::uintptr_t kHandleMagic = 0x464F4300U;

/// [en] Bits reserved for the slot index in a handle token. [zh] 句柄令牌中槽位索引占用的位。
constexpr std::uint32_t kSlotBits = 8U;


/**
 * @brief [en] Mint a handle token: magic | generation | slot, as a pointer that is *never*
 *        dereferenced.
 *        [zh] 签发句柄令牌：magic | 代际 | 槽位，形式上是**永不解引用**的指针。
 *
 * @note [en] An earlier version pointed the handle at a per-slot control block; because the block
 *       is reused by the next occupation, a stale handle then read the *new* generation and passed
 *       validation. The generation has to live in the handle value itself.
 *       [zh] 早先的版本让句柄指向每个槽位的控制块；由于控制块会被下一次占用复用，旧句柄会读到
 *       **新**代际从而通过校验。代际必须放在句柄值本身里。
 * @since 0.1.0
 */
auto mint_handle(std::uint32_t slot, std::uint32_t generation) noexcept -> foc_motor_t {
    // The token is never dereferenced: it is a handle value, not an address.
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, performance-no-int-to-ptr)
    return reinterpret_cast<foc_motor_t>((kHandleMagic << 32U) |
                                         (static_cast<std::uintptr_t>(generation) << kSlotBits) |
                                         static_cast<std::uintptr_t>(slot));
}

/// [en] Static pool: constant-initialised, so there is no dynamic initialisation on start-up.
/// [zh] 静态池：常量初始化，启动期没有动态初始化。
Slot g_pool[kMaxMotors];


/// [en] Map a core error code onto the frozen C enum (the values are identical, by contract).
/// [zh] 把核心错误码映射为冻结的 C 枚举（按契约两者数值相同）。
auto to_c(foc::ErrorCode code) noexcept -> foc_err_t {
    static_assert(static_cast<std::int32_t>(foc::ErrorCode::Ok) == static_cast<std::int32_t>(FOC_OK),
                  "foc::ErrorCode must mirror foc_err_t");
    return static_cast<foc_err_t>(static_cast<std::int32_t>(code));
}


/// [en] Whether the parameters describe a machine the core would accept.
/// [zh] 这些参数是否描述的是一台核心能接受的电机。
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
 *        [zh] 把句柄解析为槽位，拒绝任何不是"活句柄"的东西。
 *
 * @return [en] the slot, or nullptr when the handle is foreign, destroyed or stale.
 *         [zh] 槽位；句柄不属于本池、已销毁或已失效时返回 nullptr。
 * @post [en] No pointer is dereferenced before the membership check, so a garbage handle cannot
 *       fault: this is what turns undefined behaviour into `FOC_ERR_PARAM`.
 *       [zh] 成员判定之前不解引用任何指针，因此野句柄不会触发访问异常——这就是把未定义行为变成
 *       `FOC_ERR_PARAM` 的地方。
 * @since 0.1.0
 */
auto resolve(foc_motor_t handle) noexcept -> Slot* {
    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast, performance-no-int-to-ptr)
    // Decoding the opaque token: the value is never dereferenced.
    const auto raw = reinterpret_cast<std::uintptr_t>(handle);
    if ((raw >> 32U) != kHandleMagic) {
        return nullptr; // not minted here (or a garbage pointer): no dereference, no fault
    }

    const auto slot_index = static_cast<std::uint32_t>(raw & 0xFFU);
    const auto generation = static_cast<std::uint32_t>((raw >> kSlotBits) & 0xFFFFFFU);
    if (slot_index >= kMaxMotors) {
        return nullptr;
    }

    Slot& slot = g_pool[slot_index];
    if (!slot.alive || (slot.generation != generation)) {
        return nullptr; // destroyed, or the slot has been occupied again since (ABA)
    }
    return &slot;
}


/// [en] The motor living in a slot. [zh] 槽位中存放的电机。
auto motor_of(Slot& slot) noexcept -> MotorT& { return *slot.instance; }


/**
 * @brief [en] Run the destructor and drop the pointer.
 *        [zh] 调用析构并清掉指针。
 *
 * @note [en] The destructor name has to be written where the alias is visible, which is why this
 *       lives here and not in the C block below.
 *       [zh] 析构名必须写在该别名可见的作用域里，所以这个函数在这里而不在下面的 C 块里。
 * @since 0.1.0
 */
void destroy_motor(Slot& slot) noexcept {
    if (slot.instance != nullptr) {
        slot.instance->~MotorT();
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

    for (std::size_t i = 0U; i < foc::api::kMaxMotors; ++i) {
        foc::api::Slot& slot = foc::api::g_pool[i];
        if (slot.alive) {
            continue;
        }

        const foc::motor::MotorConfig config{params->rs,
                                             params->ld,
                                             params->lq,
                                             params->kt,
                                             params->j,
                                             params->b,
                                             params->pole_pairs,
                                             resolved.i_max_a,
                                             resolved.pwm_freq_hz,
                                             resolved.deadtime_ns,
                                             resolved.speed_loop_period_s,
                                             resolved.position_loop_period_s};
        const foc::core::Pi::Config current =
            foc::current_pi_config(config, resolved.current_bandwidth_rad_s, resolved.v_bus_v);
        const foc::core::Pi::Config speed = foc::speed_pi_config(config, resolved.speed_bandwidth_rad_s);
        const foc::core::PositionP::Config position =
            foc::position_p_config(config, resolved.position_bandwidth_rad_s, resolved.velocity_limit_rad_s,
                                   resolved.position_accel_limit_rad_s2);
        // An all-zero recipe is the documented "no alignment" state: the aligner then answers
        // `NotImpl` from validate() and start() goes straight to Run.
        // 全零配方就是文档化的“不对齐”状态：此时对齐器在 validate() 答 `NotImpl`，start() 直接进 Run。
        const foc::core::AlignConfig alignment{resolved.align_current_a, resolved.align_ramp_s,
                                               resolved.align_hold_s,    resolved.align_tolerance_rad,
                                               resolved.align_verify_s,  resolved.align_angle_e_rad};

        slot.hal.reset();
        slot.hal.dc_link.set_voltage_v(resolved.v_bus_v);

        const foc::api::MotorT::Policies policies{foc::core::BackEmfObserver({params->rs, params->ld}),
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


foc_err_t foc_bridge_get_setup(foc_motor_t motor, foc_motor_setup_t* out) {
    if (out == nullptr) {
        return FOC_ERR_PARAM;
    }

    foc::api::Slot* slot = foc::api::resolve(motor);
    if (slot == nullptr) {
        return FOC_ERR_PARAM;
    }

    const foc::api::Setup& applied = slot->setup;
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
    return FOC_OK;
}


foc_err_t foc_bridge_destroy(foc_motor_t motor) {
    foc::api::Slot* slot = foc::api::resolve(motor);
    if (slot == nullptr) {
        return FOC_ERR_PARAM;
    }

    foc::api::destroy_motor(*slot);
    slot->alive = false;
    ++slot->generation; // invalidates every handle taken from this occupation of the slot
    return FOC_OK;
}


foc_err_t foc_bridge_enable(foc_motor_t motor) {
    foc::api::Slot* slot = foc::api::resolve(motor);
    if (slot == nullptr) {
        return FOC_ERR_PARAM;
    }
    return foc::api::to_c(foc::api::motor_of(*slot).start());
}


foc_err_t foc_bridge_disable(foc_motor_t motor) {
    foc::api::Slot* slot = foc::api::resolve(motor);
    if (slot == nullptr) {
        return FOC_ERR_PARAM;
    }
    return foc::api::to_c(foc::api::motor_of(*slot).stop());
}


foc_err_t foc_bridge_clear_fault(foc_motor_t motor) {
    foc::api::Slot* slot = foc::api::resolve(motor);
    if (slot == nullptr) {
        return FOC_ERR_PARAM;
    }
    return foc::api::to_c(foc::api::motor_of(*slot).clear_fault());
}


foc_err_t foc_bridge_set_speed(foc_motor_t motor, float rad_per_s) {
    foc::api::Slot* slot = foc::api::resolve(motor);
    if ((slot == nullptr) || !std::isfinite(rad_per_s)) {
        return FOC_ERR_PARAM;
    }
    foc::api::motor_of(*slot).set_speed(rad_per_s);
    return FOC_OK;
}


foc_err_t foc_bridge_set_torque(foc_motor_t motor, float iq_a) {
    foc::api::Slot* slot = foc::api::resolve(motor);
    if ((slot == nullptr) || !std::isfinite(iq_a)) {
        return FOC_ERR_PARAM;
    }
    foc::api::motor_of(*slot).set_torque(iq_a);
    return FOC_OK;
}


foc_err_t foc_bridge_set_position(foc_motor_t motor, float position_rad) {
    foc::api::Slot* slot = foc::api::resolve(motor);
    if ((slot == nullptr) || !std::isfinite(position_rad)) {
        return FOC_ERR_PARAM;
    }
    foc::api::motor_of(*slot).set_position(position_rad);
    return FOC_OK;
}


foc_err_t foc_bridge_set_command(foc_motor_t motor, const foc_motor_command_t* command) {
    foc::api::Slot* slot = foc::api::resolve(motor);
    if ((slot == nullptr) || (command == nullptr)) {
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
    foc::api::motor_of(*slot).set_command(foc::motor::Command{
        command->position_rad, command->max_current_a, command->feedforward_current_a, command->max_velocity_rad_s,
        command->max_accel_rad_s2, command->kp_scale, command->kd_scale, command->ilimit_scale});
    return FOC_OK;
}


foc_err_t foc_bridge_get_status(foc_motor_t motor, foc_motor_status_t* out) {
    if (out == nullptr) {
        return FOC_ERR_PARAM;
    }
    foc::api::Slot* slot = foc::api::resolve(motor);
    if (slot == nullptr) {
        return FOC_ERR_PARAM;
    }

    foc::api::MotorT& instance = foc::api::motor_of(*slot);
    const foc::motor::Telemetry telemetry = instance.telemetry();
    // The aligner is always an `AlignRoutine` on this path, so the offset is always readable; the
    // C view simply reports it rather than hiding the field behind a capability check.
    // 本路径上的对齐器固定是 `AlignRoutine`，因此偏置总可读；C 视图直接报出来，而不是把这个字段
    // 藏进能力检查后面。
    const foc::core::AlignRoutine& aligner = instance.aligner();

    static_assert(static_cast<std::int32_t>(foc::core::MotorState::Idle) == static_cast<std::int32_t>(FOC_MOTOR_IDLE),
                  "foc_motor_state_t must mirror foc::core::MotorState");
    static_assert(static_cast<std::int32_t>(foc::core::MotorState::Align) == static_cast<std::int32_t>(FOC_MOTOR_ALIGN),
                  "foc_motor_state_t must mirror foc::core::MotorState");
    static_assert(static_cast<std::int32_t>(foc::core::MotorState::Run) == static_cast<std::int32_t>(FOC_MOTOR_RUN),
                  "foc_motor_state_t must mirror foc::core::MotorState");
    static_assert(static_cast<std::int32_t>(foc::core::MotorState::Fault) == static_cast<std::int32_t>(FOC_MOTOR_FAULT),
                  "foc_motor_state_t must mirror foc::core::MotorState");
    static_assert(static_cast<std::int32_t>(foc::core::AlignState::Done) == static_cast<std::int32_t>(FOC_ALIGN_DONE),
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

    foc_motor_status_t result{};
    result.struct_size = sizeof(foc_motor_status_t);
    result.mode_state = static_cast<std::int32_t>(instance.state());
    result.error = static_cast<std::int32_t>(foc::api::to_c(instance.status()));
    result.align_state = static_cast<std::int32_t>(aligner.state());
    result.align_failure = static_cast<std::int32_t>(aligner.failure());
    result.align_offset_rad = aligner.offset_mech_rad();
    result.align_spread_rad = aligner.spread_rad();
    result.max_period_us = telemetry.max_period_us;
    *out = result;
    return FOC_OK;
}


foc_err_t foc_bridge_tick(foc_motor_t motor, float dt_s) {
    foc::api::Slot* slot = foc::api::resolve(motor);
    if (slot == nullptr) {
        return FOC_ERR_PARAM;
    }
    return foc::api::to_c(foc::api::motor_of(*slot).tick(dt_s));
}


foc_err_t foc_bridge_get_state(foc_motor_t motor, float* speed_rad_s, float* id, float* iq) {
    foc::api::Slot* slot = foc::api::resolve(motor);
    if (slot == nullptr) {
        return FOC_ERR_PARAM;
    }

    const foc::motor::Telemetry telemetry = foc::api::motor_of(*slot).telemetry();
    if (speed_rad_s != nullptr) {
        *speed_rad_s = telemetry.speed_mech_rad_s;
    }
    if (id != nullptr) {
        *id = telemetry.id_a;
    }
    if (iq != nullptr) {
        *iq = telemetry.iq_a;
    }
    return FOC_OK;
}

} // extern "C"
