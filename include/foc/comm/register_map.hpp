// Conan::ImportStart
#pragma once
#include <cstddef>
#include <cstdint>
#include <cstring>

// The only place under `include/` that includes the C surface, and it does so
// deliberately: the protocol layer is a *consumer* of the C ABI, not part of
// it. Nothing in `core/`, `hal/`, `driver/` or `motor/` may include this
// header.
//
// The include is wrapped in `extern "C"` because this header *calls* the ABI,
// and a C++ translation unit that calls it without the wrapper gets mangled
// references: measured as 237 undefined symbols against the shipped
// `libfoc_c.a`, whose `foc_*` definitions are unmangled. The wrapper is the
// source-tree half of what `conanfile.py::_make_c_compatible()` does to every
// `.h` it installs -- the installed copy of `foc_api.h` already carries the
// guard, so a C++ consumer of the *package* has always seen C linkage while a
// C++ consumer of the *tree* has not. Against the installed header the two
// guards nest, which is legal, so the same source builds both ways.
//
// What is still owed is the guard *inside* `foc/api/foc_api.h`, which would fix
// every C++ caller at once. That changes the linkage of every symbol in the
// public C surface and is its own change with its own measurement; this wrapper
// only aligns the private code that reaches across the boundary.
// 本头文件是 `include/` 下唯一包含 C 表面的地方，而且它是有意这么做的：协议层是
// C ABI 的 **消费者** ，不属于 ABI 本身。 `core/`、`hal/`、`driver/`、`motor/`
// 里任何一个都不许包含它。
//
// 这一条 include 外面套了 `extern "C"`，因为本头文件 **调用** 了这个 ABI：不套的
// C++ 编译单元拿到的是修饰名——对着出厂的 `libfoc_c.a`（其中的 `foc_*` 定义是未
// 修饰的）实测 237 个未定义符号。这个包裹就是 `conanfile.py::_make_c_compatible()`
// 对每个装机 `.h` 所做那一步的源码树版本：装机的那份 `foc_api.h` 早就带着保护，
// 所以 **包** 的 C++ 消费方一直看到 C 链接性，而 **树** 的 C++ 消费方没有。对着
// 装机头两个保护是嵌套的，合法，于是同一份源码两种编法一致。
//
// 仍然欠着的是把保护写进 `foc/api/foc_api.h` 本身——那样一次修好所有 C++ 调用方。
// 那会改变公开 C 表面每个符号的链接性，是另一件事、需要它自己的测量；这里的包裹
// 只负责把跨边界的私有代码对齐。
extern "C" {
#include <foc/api/foc_api.h>
}
// Conan::ImportEnd

/*!
 * @ingroup group_comm
 * @file register_map.hpp
 *
 * @brief [en] The register table a host talks to, dense from 1 so enumeration has no holes.
 * @brief [zh] 主机访问的寄存器表，从 1 起连续编号，枚举时没有空洞。
 * @since 0.1.0
 */

namespace foc {

namespace comm {

/**
 * @brief [en] The address a host reads something at. Dense from 1, so a host can enumerate without a hole table.
 * @brief [zh] 上位机读取某物的地址。从 1 起连续，于是上位机可以枚举而不需要一张空洞表。
 * @note [en] The numbers are the protocol's, not the C ABI's, and they are the only place in the tree where a register
 *            number is written down. Everything that wants to read one of these through a link names it by this enum,
 *            so "the protocol grew a second vocabulary" shows up as a compile error rather than as two tables that
 *            disagree. `std::uint16_t` is the **address width of the protocol**, not the number of registers that exist
 *            today: a later revision can address more than ten, and narrowing this to one byte would have to be undone
 *            at the point where the wire format is already frozen. The layout suggestion and the wire format point in
 *            opposite directions, and the wire wins.
 * @note [zh] 这些编号是 **协议** 的，不是 C ABI 的，而且它们是整棵树里唯一写下寄存器编号的地方。任何想通过链路读这些东
 *            西的代码都用这个枚举来指代，于是"协议长出了第二套词汇"表现为一个编译错误，而不是两张互相矛盾的表格。
 *            `std::uint16_t` 是 **协议的地址宽度** ，不是今天已有的寄存器个数：后续版本可以寻址超过十个，而把它收窄成一
 *            个字节，到线上格式已经冻结的那一刻又得改回来。布局建议与线上格式指向相反的方向，线上格式赢。
 * @since 0.1.0
 * @exporter
 */
// NOLINTNEXTLINE(performance-enum-size)
enum class RegisterId : std::uint16_t {
    /**
     * @brief [en] `foc_motor_status_t::mode_state`.
     * @brief [zh] `foc_motor_status_t::mode_state`。
     */
    State = 1,
    /**
     * @brief [en] `foc_motor_status_t::error`, the sticky error.
     * @brief [zh] `foc_motor_status_t::error`，粘滞错误。
     */
    Error = 2,
    /**
     * @brief [en] worst control-tick interval seen, us.
     * @brief [zh] 见过的最差控制节拍间隔，µs。
     */
    MaxPeriodUs = 3,
    /**
     * @brief [en] the observer actually running.
     * @brief [zh] 实际在跑的观测器。
     */
    Observer = 4,
    /**
     * @brief [en] 1 while the observer says it is measuring.
     * @brief [zh] 观测器自称在测量时为 1。
     */
    ObserverLocked = 5,
    /**
     * @brief [en] retained trace samples.
     * @brief [zh] 保留的轨迹样本数。
     */
    TraceSize = 6,
    /**
     * @brief [en] trace samples overwritten.
     * @brief [zh] 被覆盖的轨迹样本数。
     */
    TraceDropped = 7,
    /**
     * @brief [en] mechanical speed, as float bits.
     * @brief [zh] 机械转速，float 位型。
     */
    SpeedRadS = 8,
    /**
     * @brief [en] d-axis current, as float bits.
     * @brief [zh] d 轴电流，float 位型。
     */
    CurrentD = 9,
    /**
     * @brief [en] q-axis current, as float bits.
     * @brief [zh] q 轴电流，float 位型。
     */
    CurrentQ = 10,
};

/**
 * @brief [en] How to read a register's 32 bits.
 * @brief [zh] 怎么读一个寄存器的那 32 位。
 * @note [en] `FloatBits` exists because a float cannot cross a word-oriented register interface: the reader has to know
 *            it is looking at IEEE-754 bits and not at an integer, and a host that prints 1065353216 as a speed is
 *            reporting a number that is correct to the bit and wrong to the reader.
 * @note [zh] `FloatBits` 的存在是因为 float 过不了面向字的寄存器接口：读方必须知道自己在看的是 IEEE-754 位型而不是整
 *            数，而把 1065353216 当速度打印出来的上位机报的是一个逐位正确、对读者错误的数。
 * @since 0.1.0
 * @exporter
 */
enum class RegisterKind : std::uint8_t {
    /**
     * @brief [en] an unsigned count or code.
     * @brief [zh] 无符号计数或编码。
     */
    Unsigned = 0,
    /**
     * @brief [en] a signed code that can be negative.
     * @brief [zh] 可能为负的有符号编码。
     */
    Signed = 1,
    /**
     * @brief [en] IEEE-754 binary32, bit for bit.
     * @brief [zh] IEEE-754 binary32，逐位。
     */
    FloatBits = 2,
};

/**
 * @brief [en] One register: its number, how to read it, and the C function that answers it.
 * @brief [zh] 一个寄存器：编号、怎么读、以及回答它的那个 C 函数。
 * @note [en] `read` is a **function pointer taken at compile time**, not a name in a table to be looked up later. That
 *            is the whole design: renaming a C function without updating this table is a compile error, in every build
 *            leg, before anything else has a chance to notice. A string-keyed table would compile and fail later, which
 *            means it fails in the field. `abi` is carried for the diagnostics and for the completeness check, and it
 *            is the *name of the function whose address is in `read`* -- the two are written on the same line so they
 *            cannot drift.
 * @note [zh] `read` 是 **编译期取得的函数指针** ，不是一张稍后要查的名字表。这就是整个设计：改了 C 函数名而没更新这张
 *            表，在每一条构建腿里都是编译错误，早于任何其他环节发现问题。以字符串为键的表能编过、在更晚的时候才失败——
 *            而那意味着它在现场失败。 `abi` 是为了诊断与完整性检查而携带的，它
 *            **就是 `read` 里那个函数的地址所属的函数名** ——两者写在同一行，所以漂不开。
 * @since 0.1.0
 * @exporter
 */
struct RegisterEntry final {
    /**
     * @brief [en] the register's wire address.
     * @brief [zh] 寄存器的线上地址。
     */
    RegisterId id;
    /**
     * @brief [en] how to interpret the word.
     * @brief [zh] 怎么解释这个字。
     */
    RegisterKind kind;
    /**
     * @brief [en] the C function behind it.
     * @brief [zh] 背后的 C 函数。
     */
    const char* abi;
    /**
     * @brief [en] the reader.
     * @brief [zh] 读取函数。
     */
    std::int32_t (*read)(foc_motor_t motor, std::uint32_t* raw);
};

/**
 * @brief [en] The table: `kRegisterCount` entries, `RegisterId` order, no holes.
 * @brief [zh] 这张表： `kRegisterCount` 项，按 `RegisterId` 顺序，没有空洞。
 * @note [en] Read-only and static: nothing here allocates, and a host that walks the table costs one function pointer
 *            call per register. It is not in the control path and is not meant to be.
 * @note [zh] 只读且静态：这里什么都不分配，走一遍表的代价是每个寄存器一次函数指针调用。它不在控制路径上，也不打算在。
 * @since 0.1.0
 * @exporter
 */
constexpr std::size_t kRegisterCount = 10U;

namespace detail {

/**
 * @brief [en] The sticky status, read whole.
 *
 *   Five registers live in this one struct, and reading it five times would let a caller see three different instants
 *   in one frame.
 *
 * @brief [zh] 一次性读回整个状态结构。
 *
 *   五个寄存器住在这一个结构里，读五遍会让调用方在同一帧里看到三个不同的时刻。
 */
inline auto status_read(foc_motor_t motor, std::uint32_t* raw, int which) noexcept -> std::int32_t {
    foc_motor_status_t status{};
    status.struct_size = sizeof(status);
    const foc_err_t rc = foc_motor_get_status(motor, &status);
    if (rc != FOC_OK) {
        return rc;
    }
    switch (which) {
    case 0:
        *raw = static_cast<std::uint32_t>(status.mode_state);
        break;
    case 1:
        *raw = static_cast<std::uint32_t>(status.error);
        break;
    case 2:
        *raw = status.max_period_us;
        break;
    case 3:
        *raw = static_cast<std::uint32_t>(status.observer);
        break;
    case 4:
        *raw = static_cast<std::uint32_t>(status.observer_locked);
        break;
    default:
        return FOC_ERR_PARAM;
    }
    return FOC_OK;
}

/**
 * @brief [en] The three state words, read together for the same reason as the status struct.
 * @brief [zh] 三个状态字一起读，理由与状态结构相同。
 */
inline auto state_read(foc_motor_t motor, std::uint32_t* raw, int which) noexcept -> std::int32_t {
    float speed_rad_s = 0.0F;
    float id_a = 0.0F;
    float iq_a = 0.0F;
    const foc_err_t rc = foc_get_state(motor, &speed_rad_s, &id_a, &iq_a);
    if (rc != FOC_OK) {
        return rc;
    }
    const float value = (which == 0) ? speed_rad_s : ((which == 1) ? id_a : iq_a);
    // Bit-exact, not numeric: the register carries the word, and converting
    // through an integer would be a different number that happens to round to the
    // same decimal.
    // 逐位搬运而不是数值转换：寄存器承载的是那个字，而经整数中转会得到一个只是凑巧修约到同一个小数的另一个数。
    //
    // `memcpy` here is the well-defined way to do it and there is no `memcpy_s` to prefer: Annex K is optional, glibc
    // does not implement it, and the bare-metal targets this library ships to do not have it at all. The sizes are
    // equal by the `static_assert` above and the destination is the caller's own word, so the hazard the rule warns
    // about is already excluded by the type. Suppressed with that reason rather than silently ignored.
    // 这里的 `memcpy` 是定义良好的做法，而且没有 `memcpy_s` 可优先：Annex K 是可选的，glibc 不实现它，本库要上的裸机目标
    // 根本没有。尺寸由上面的 `static_assert` 保证相等，目的地是调用方自己的那个字，所以这条规则警告的危险已经被类型排除。
    // 带着这个理由抑制，而不是悄悄无视。
    static_assert(sizeof(float) == sizeof(std::uint32_t), "a register word is 32 bits and so is a float");
    std::memcpy(raw, &value, sizeof(value)); // devskim: ignore DS121708
    return FOC_OK;
}

inline auto read_state(foc_motor_t motor, std::uint32_t* raw) noexcept -> std::int32_t {
    return status_read(motor, raw, 0);
}
inline auto read_error(foc_motor_t motor, std::uint32_t* raw) noexcept -> std::int32_t {
    return status_read(motor, raw, 1);
}
inline auto read_max_period(foc_motor_t motor, std::uint32_t* raw) noexcept -> std::int32_t {
    return status_read(motor, raw, 2);
}
inline auto read_observer(foc_motor_t motor, std::uint32_t* raw) noexcept -> std::int32_t {
    return status_read(motor, raw, 3);
}
inline auto read_locked(foc_motor_t motor, std::uint32_t* raw) noexcept -> std::int32_t {
    return status_read(motor, raw, 4);
}
inline auto read_speed(foc_motor_t motor, std::uint32_t* raw) noexcept -> std::int32_t {
    return state_read(motor, raw, 0);
}
inline auto read_id(foc_motor_t motor, std::uint32_t* raw) noexcept -> std::int32_t {
    return state_read(motor, raw, 1);
}
inline auto read_iq(foc_motor_t motor, std::uint32_t* raw) noexcept -> std::int32_t {
    return state_read(motor, raw, 2);
}

/**
 * @brief [en] `foc_motor_trace_size` answers a count and cannot fail.
 *
 *   A NULL handle reads 0, which is the documented answer.
 *
 * @brief [zh] `foc_motor_trace_size` 直接给出一个计数，不会失败。
 *
 *   NULL 句柄读作 0，那正是它文档里的答案。
 */
inline auto read_trace_size(foc_motor_t motor, std::uint32_t* raw) noexcept -> std::int32_t {
    *raw = foc_motor_trace_size(motor);
    return FOC_OK;
}

inline auto read_trace_dropped(foc_motor_t motor, std::uint32_t* raw) noexcept -> std::int32_t {
    *raw = foc_motor_trace_dropped(motor);
    return FOC_OK;
}

} // namespace detail

/**
 * @brief [en] The register table itself.
 * @brief [zh] 寄存器表本身。
 * @return [en] a static array of `kRegisterCount` entries.
 * @return [zh] 一个含 `kRegisterCount` 项的静态数组。
 * @since 0.1.0
 * @exporter
 */
inline auto register_table() noexcept -> const RegisterEntry* {
    static const RegisterEntry table[kRegisterCount] = {
        {RegisterId::State, RegisterKind::Signed, "foc_motor_get_status", &detail::read_state},
        {RegisterId::Error, RegisterKind::Signed, "foc_motor_get_status", &detail::read_error},
        {RegisterId::MaxPeriodUs, RegisterKind::Unsigned, "foc_motor_get_status", &detail::read_max_period},
        {RegisterId::Observer, RegisterKind::Signed, "foc_motor_get_status", &detail::read_observer},
        {RegisterId::ObserverLocked, RegisterKind::Unsigned, "foc_motor_get_status", &detail::read_locked},
        {RegisterId::TraceSize, RegisterKind::Unsigned, "foc_motor_trace_size", &detail::read_trace_size},
        {RegisterId::TraceDropped, RegisterKind::Unsigned, "foc_motor_trace_dropped", &detail::read_trace_dropped},
        {RegisterId::SpeedRadS, RegisterKind::FloatBits, "foc_get_state", &detail::read_speed},
        {RegisterId::CurrentD, RegisterKind::FloatBits, "foc_get_state", &detail::read_id},
        {RegisterId::CurrentQ, RegisterKind::FloatBits, "foc_get_state", &detail::read_iq},
    };
    return table;
}

/**
 * @brief [en] Read one register into a word; false when the handle or the address is unusable.
 * @brief [zh] 把一个寄存器读进一个字；句柄或地址不可用时返回 false。
 * @param motor [en] the motor handle.
 * @param motor [zh] 电机句柄。
 * @param id [en] which register; an id with no entry is refused, not guessed at.
 * @param id [zh] 哪个寄存器；表里没有的编号被拒绝，不猜。
 * @param raw [en] non-NULL receiver of the 32-bit word.
 * @param raw [zh] 32 位字的非 NULL 接收者。
 * @return [en] true when the word was written.
 * @return [zh] 字被写入时为 true。
 * @note [en] A register the C surface refused leaves `raw` untouched, so a host cannot pair a stale value with a fresh
 *            error -- the same rule as `decode_frame`: fill nothing until every check has passed.
 * @note [zh] C 接口拒绝了的寄存器不会触碰 `raw`，于是上位机不可能把一个陈旧的值与一个新到的错误配成一对——与
 *            `decode_frame` 同一条规则：在所有检查通过之前什么都不填。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto read_register(foc_motor_t motor, RegisterId id, std::uint32_t* raw) noexcept -> bool {
    if (raw == nullptr) {
        return false;
    }
    const RegisterEntry* table = register_table();
    for (std::size_t i = 0U; i < kRegisterCount; ++i) {
        if (table[i].id == id) {
            return table[i].read(motor, raw) == FOC_OK;
        }
    }
    return false;
}

} // namespace comm

} // namespace foc
