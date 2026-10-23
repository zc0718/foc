// Conan::ImportStart
#include "bridge.h"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <new>

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

using MotorT = motor::Motor<core::BackEmfObserver, core::Svpwm, core::Pi, driver::StubHal>;

/// [en] Object pool capacity: static storage, no heap, no fragmentation.
/// [zh] 对象池容量：静态存储、无堆、无碎片。
constexpr std::size_t kMaxMotors = 4U;

// Host/SIL defaults for the C path. The C API cannot express a custom HAL or hand-picked gains,
// so it runs the stub HAL with bandwidth-derived tuning; hardware users go through the C++ path.
// C 路径的主机/SIL 默认值。C API 无法表达自定义 HAL 或手调增益，因此它跑桩 HAL + 带宽推导的
// 整定；上板用户走 C++ 路径。
constexpr float kPwmFreqHz = 20000.0F;
constexpr float kDeadtimeNs = 500.0F;
constexpr float kIMaxA = 5.0F;
constexpr float kBusV = 24.0F;
constexpr float kCurrentBandwidthRadS = 2000.0F;
constexpr float kSpeedBandwidthRadS = 200.0F;


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
    if ((params == nullptr) || (out == nullptr)) {
        return FOC_ERR_PARAM;
    }
    if (!foc::api::params_ok(*params)) {
        return FOC_ERR_PARAM;
    }

    for (std::size_t i = 0U; i < foc::api::kMaxMotors; ++i) {
        foc::api::Slot& slot = foc::api::g_pool[i];
        if (slot.alive) {
            continue;
        }

        const foc::motor::MotorConfig config{
            params->rs, params->ld,         params->lq,       params->kt,           params->j,
            params->b,  params->pole_pairs, foc::api::kIMaxA, foc::api::kPwmFreqHz, foc::api::kDeadtimeNs};
        const foc::core::Pi::Config current =
            foc::current_pi_config(config, foc::api::kCurrentBandwidthRadS, foc::api::kBusV);
        const foc::core::Pi::Config speed = foc::speed_pi_config(config, foc::api::kSpeedBandwidthRadS);

        slot.hal.reset();
        slot.hal.dc_link.set_voltage_v(foc::api::kBusV);

        const foc::api::MotorT::Policies policies{foc::core::BackEmfObserver({params->rs, params->ld}),
                                                  foc::core::Svpwm{}, foc::core::Pi{current}, foc::core::Pi{current},
                                                  foc::core::Pi{speed}};

        slot.instance = new (static_cast<void*>(slot.storage)) foc::api::MotorT(config, policies, slot.hal);

        const foc::ErrorCode rc = slot.instance->init();
        if (rc != foc::ErrorCode::Ok) {
            foc::api::destroy_motor(slot);
            return foc::api::to_c(rc);
        }

        ++slot.generation;
        slot.alive = true;
        *out = foc::api::mint_handle(static_cast<std::uint32_t>(i), slot.generation);
        return FOC_OK;
    }

    return FOC_ERR_NO_RESOURCE;
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
