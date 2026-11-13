// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include <foc/comm/frame.hpp>
#include <foc/api/foc_types.h>
// Conan::ImportEnd

/*!
 * @ingroup group_comm
 * @file commissioned_set.hpp
 *
 * @brief [en] The numbers a commissioning run measures, in a form that survives a power cycle.
 * @brief [zh] 一次调试流程测出的数字，以能活过一次断电的形态保存。
 * @since 0.1.0
 */

namespace foc {

namespace comm {

/**
 * @brief [en] Bytes one set occupies: marker, schema, machine id, seven numbers, provenance and a check value.
 * @brief [zh] 一组占用的字节数：标识、格式版本、机器编号、七个数字、来源信息与校验值。
 * @note [en] A fixed size rather than a length field: schema 1 has exactly one layout, and a reader that has to parse a
 *            variable-length envelope before it can say "this is not my schema" has already lost the argument it needs
 *            to make. Schema 2 may add bytes; it may not reinterpret these.
 * @note [zh] 定长而不带长度字段：格式 1 只有一个布局，而一个必须先解析变长外壳、才能说出"这不是我的格式"的读取方，早已
 *            输掉它需要打的那场辩论。格式 2 可以追加字节，但不可以重新解释这些字节。
 * @since 0.1.0
 * @exporter
 */
inline constexpr std::size_t kCommissionedSetBytes = 46U;

/**
 * @brief [en] First byte of the marker (`'F'`).
 * @brief [zh] 标识的第一个字节（ `'F'`）。
 * @since 0.1.0
 */
inline constexpr std::uint8_t kCommissionedSetMagic0 = 0x46U;

/**
 * @brief [en] Second byte of the marker (`'S'`, for the set).
 * @brief [zh] 标识的第二个字节（ `'S'`，表示 set）。
 * @since 0.1.0
 */
inline constexpr std::uint8_t kCommissionedSetMagic1 = 0x53U;

/**
 * @brief [en] The only schema this build writes and reads.
 * @brief [zh] 本构建唯一会写入与读取的格式版本。
 * @note [en] A stored blob outlives the firmware that wrote it, so "unknown version" has to be an answer rather than a
 *            best guess: reading a newer set with an older reader is exactly the case where silent misinterpretation
 *            costs a motor.
 * @note [zh] 存下来的字节会比写它的固件活得久，所以"未知版本"必须是一个回答而不是一个猜测：用更旧的读者去读更新的一组数
 *            字，正是"静默误读"会以电机为代价的那种情形。
 * @since 0.1.0
 */
inline constexpr std::uint8_t kCommissionedSetSchema = 1U;

/**
 * @brief [en] Why a byte sequence is not a set.
 * @brief [zh] 一段字节为什么不是一组标定结果。
 * @since 0.1.0
 * @exporter
 */
enum class SetStatus : std::uint8_t {
    /**
     * @brief [en] the bytes are one intact set.
     * @brief [zh] 字节是一组完整的标定结果。
     */
    Ok,
    /**
     * @brief [en] fewer bytes than a set occupies.
     * @brief [zh] 字节数不够一组。
     */
    Short,
    /**
     * @brief [en] the marker is not ours.
     * @brief [zh] 标识不是本格式的。
     */
    BadMagic,
    /**
     * @brief [en] a set of a version this build does not read.
     * @brief [zh] 本构建不读的版本。
     */
    BadSchema,
    /**
     * @brief [en] a reserved byte is not zero, so the writer meant something this reader cannot honour.
     * @brief [zh] 保留字节不是零，说明写入方表达了本读取方无法兑现的语义。
     */
    BadReserved,
    /**
     * @brief [en] the check value does not match the bytes.
     * @brief [zh] 校验值与字节不匹配。
     */
    BadCheckValue,
    /**
     * @brief [en] intact bytes carrying numbers no machine can have.
     * @brief [zh] 字节完好，但里面的数字不是任何电机能有的。
     */
    BadField,
};

/**
 * @brief [en] Where reading stopped, and why.
 * @brief [zh] 读取停在哪里、为什么。
 * @note [en] The offset is part of the answer, not a debugging aid: a host writing into flash has to be told which
 *            field it corrupted, and "invalid set" on its own sends somebody to a hex dump.
 * @note [zh] 偏移量是答案的一部分，不是调试辅助：往 flash 里写的一方需要知道它弄坏的是哪个字段，而一句"无效的组"只会把
 *            人送去翻十六进制。
 * @since 0.1.0
 * @exporter
 */
struct SetError final {
    /**
     * @brief [en] what the reader concluded.
     * @brief [zh] 读取方得出的结论。
     */
    SetStatus status{SetStatus::Ok};
    /**
     * @brief [en] the byte the reader stopped at; 0 when `status` is `Ok`.
     * @brief [zh] 读取方停下的那个字节； `status` 为 `Ok` 时为 0。
     */
    std::size_t offset{0U};
};

/**
 * @brief [en] One commissioning result: the numbers, which machine they belong to, and how they were obtained.
 * @brief [zh] 一次调试的结果：数字、它属于哪台机器，以及它是怎么来的。
 * @details [en] The seven numbers are the same ones `foc_motor_params_t` carries and `foc_motor_apply_params()`
 *               consumes -- deliberately, because a set that needed translating before it could be applied would be a
 *               second vocabulary for the same facts. `machine_id` is the caller's own identifier (a serial number, a
 *               hash of the nameplate); the library does not assign it and never compares it against anything it knows,
 *               because it knows nothing about which machine it is talking to. `measured_at_unix_s` and `temperature_c`
 *               are provenance: they do not enter any control law, and they exist because a support call asks "when was
 *               this measured, and how hot was it".
 * @details [zh] 这七个数字与 `foc_motor_params_t` 携带的、 `foc_motor_apply_params()` 消费的是同一组——这是故意的：一组
 *               需要先翻译才能应用的数字，等于给同一批事实造了第二套词汇。 `machine_id` 是调用方自己的标识（序列号、铭
 *               牌的哈希）；库不分配它，也从不拿它跟任何自己知道的东西比对，因为它根本不知道自己面对的是哪台机器。
 *               `measured_at_unix_s` 与 `temperature_c` 是来源信息：它们不进任何控制律，存在的理由是现场排查会问"什么时
 *               候测的、当时多热"。
 * @note [en] `temperature_c` is NaN when it was not recorded, and that is the documented marker rather than a sentinel
 *            such as `0`: zero degrees is a real temperature, and a set that cannot tell "cold" from "unknown" is the
 *            kind of ambiguity this repository refuses everywhere else. `measured_at_unix_s == 0` is likewise "not
 *            recorded", because 1970 is not a measurement.
 * @note [zh] 温度没有记录时 `temperature_c` 是 NaN，这是文档化的标记而不是 `0` 这样的哨兵：零度是一个真实温度，而一组分
 *            不清"很冷"与"不知道"的数字，正是本仓库在别处一律拒绝的那种歧义。 `measured_at_unix_s == 0` 同样表示"没有记
 *            录"，因为 1970 年不是一次测量。
 * @since 0.1.0
 * @exporter
 */
struct CommissionedSet final {
    /**
     * @brief [en] the caller's identifier for the machine; not interpreted by this library.
     * @brief [zh] 调用方给这台机器的标识；本库不解释它。
     */
    std::uint32_t machine_id{0U};
    /**
     * @brief [en] stator resistance, ohm.
     * @brief [zh] 定子电阻，欧姆。
     */
    float rs{0.0F};
    /**
     * @brief [en] d-axis inductance, henry.
     * @brief [zh] d 轴电感，亨利。
     */
    float ld{0.0F};
    /**
     * @brief [en] q-axis inductance, henry.
     * @brief [zh] q 轴电感，亨利。
     */
    float lq{0.0F};
    /**
     * @brief [en] torque constant, N*m/A.
     * @brief [zh] 转矩常数，N·m/A。
     */
    float kt{0.0F};
    /**
     * @brief [en] rotor inertia, kg*m^2.
     * @brief [zh] 转子惯量，kg·m²。
     */
    float j{0.0F};
    /**
     * @brief [en] viscous friction, N*m*s/rad.
     * @brief [zh] 粘性摩擦，N·m·s/rad。
     */
    float b{0.0F};
    /**
     * @brief [en] pole pairs.
     * @brief [zh] 极对数。
     */
    std::uint8_t pole_pairs{0U};
    /**
     * @brief [en] when it was measured, unix seconds; 0 = not recorded.
     * @brief [zh] 什么时候测的，unix 秒；0 = 没有记录。
     */
    std::uint32_t measured_at_unix_s{0U};
    /**
     * @brief [en] the machine temperature at measurement, deg C; NaN = not recorded.
     * @brief [zh] 测量时机器温度，摄氏度；NaN = 没有记录。
     */
    float temperature_c{std::nanf("")};
};

namespace detail {

/**
 * @brief [en] Little-endian writers and readers, one byte at a time.
 * @brief [zh] 小端写入与读取，一次一个字节。
 * @note [en] Byte-at-a-time rather than `memcpy` of a struct: a stored set crosses from the MCU that wrote it to a host
 *            tool that reads it, and padding, alignment and host endianness are exactly the three things that make a
 *            struct copy non-portable while looking portable.
 * @note [zh] 逐字节写出而不是把结构体 `memcpy` 出去：存下来的这组数字会从写它的 MCU 走到读它的上位机，而填充、对齐与主
 *            机字节序正是三件让"结构体拷贝"看起来可移植、实际不可移植的事。
 * @since 0.1.0
 */
inline void put_u16(std::uint8_t* out, std::uint16_t value) noexcept {
    // Widened before the shift: a `uint16_t` promoted to `int` and then and-ed with `0xFFU` is a signed/unsigned mix,
    // which the analyser refuses and which is one `-fwrapv` away from mattering.
    // 先加宽再移位：`uint16_t` 提升为 `int` 之后与 `0xFFU` 相与，是有符号与无符号混用——分析器拒绝它，而它与一个
    // `-fwrapv` 之差就可能真的出事。
    const auto wide = static_cast<std::uint32_t>(value);
    out[0] = static_cast<std::uint8_t>(wide & 0xFFU);
    out[1] = static_cast<std::uint8_t>((wide >> 8U) & 0xFFU);
}

/**
 * @brief [en] Read a little-endian 16-bit value.
 * @brief [zh] 读一个小端 16 位值。
 * @param in [en] source bytes.
 * @param in [zh] 源字节。
 * @return [en] the value.
 * @return [zh] 读到的值。
 * @since 0.1.0
 */
[[nodiscard]] inline auto get_u16(const std::uint8_t* in) noexcept -> std::uint16_t {
    return static_cast<std::uint16_t>(static_cast<unsigned>(in[0]) | (static_cast<unsigned>(in[1]) << 8U));
}

/**
 * @brief [en] Write a little-endian 32-bit value.
 * @brief [zh] 写出一个小端 32 位值。
 * @param out [en] destination, four bytes.
 * @param out [zh] 目标，四个字节。
 * @param value [en] the value.
 * @param value [zh] 要写出的值。
 * @since 0.1.0
 */
inline void put_u32(std::uint8_t* out, std::uint32_t value) noexcept {
    out[0] = static_cast<std::uint8_t>(value & 0xFFU);
    out[1] = static_cast<std::uint8_t>((value >> 8U) & 0xFFU);
    out[2] = static_cast<std::uint8_t>((value >> 16U) & 0xFFU);
    out[3] = static_cast<std::uint8_t>((value >> 24U) & 0xFFU);
}

/**
 * @brief [en] Read a little-endian 32-bit value.
 * @brief [zh] 读一个小端 32 位值。
 * @param in [en] source bytes.
 * @param in [zh] 源字节。
 * @return [en] the value.
 * @return [zh] 读到的值。
 * @since 0.1.0
 */
[[nodiscard]] inline auto get_u32(const std::uint8_t* in) noexcept -> std::uint32_t {
    const auto b0 = static_cast<std::uint32_t>(in[0]);
    const auto b1 = static_cast<std::uint32_t>(in[1]) << 8U;
    const auto b2 = static_cast<std::uint32_t>(in[2]) << 16U;
    const auto b3 = static_cast<std::uint32_t>(in[3]) << 24U;
    return b0 | b1 | b2 | b3;
}

/**
 * @brief [en] Write a float as its IEEE-754 bit pattern, little-endian.
 * @brief [zh] 以 IEEE-754 位型小端写出一个 float。
 * @param out [en] destination, four bytes.
 * @param out [zh] 目标，四个字节。
 * @param value [en] the value; NaN is written as the NaN it is.
 * @param value [zh] 要写出的值；NaN 就写成它本身。
 * @note [en] The bit pattern is stored, not a decimal rendering: a round trip has to return the same float, and a
 *            printf-style format would make the reader's precision part of the contract.
 * @note [zh] 存的是位型而不是十进制表示：往返必须返回同一个 float，而 printf 风格的格式会把读取方的精度写进契约里。
 * @since 0.1.0
 */
inline void put_f32(std::uint8_t* out, float value) noexcept {
    // Both sides are 32 bits, so the hazard the rule warns about is already excluded by the type, and there is
    // no `memcpy_s` to prefer: Annex K is optional, glibc does not implement it, and the bare-metal targets
    // this library ships to do not have it at all. Suppressed with that reason, as `register_map` does.
    // 两边都是 32 位，这条规则警告的危险已被类型排除；也没有 `memcpy_s` 可优先——Annex K 是可选的，
    // glibc 不实现它，本库要上的裸机目标根本没有。带着理由抑制，与 `register_map` 一致。
    static_assert(sizeof(float) == sizeof(std::uint32_t), "a float bit pattern is a 32-bit word");
    std::uint32_t raw = 0U;
    std::memcpy(&raw, &value, sizeof(raw)); // devskim: ignore DS121708
    put_u32(out, raw);
}

/**
 * @brief [en] Read a float from its IEEE-754 bit pattern.
 * @brief [zh] 从 IEEE-754 位型读出一个 float。
 * @param in [en] source bytes.
 * @param in [zh] 源字节。
 * @return [en] the value.
 * @return [zh] 读到的值。
 * @since 0.1.0
 */
[[nodiscard]] inline auto get_f32(const std::uint8_t* in) noexcept -> float {
    const std::uint32_t raw = get_u32(in);
    float value = 0.0F;
    std::memcpy(&value, &raw, sizeof(value)); // devskim: ignore DS121708
    return value;
}

} // namespace detail

/**
 * @brief [en] Whether these numbers describe a machine at all.
 * @brief [zh] 这些数字描述的是不是一台机器。
 * @param set [en] the set to check.
 * @param set [zh] 要检查的一组数字。
 * @return [en] true when every control-facing field is finite and inside the physical range.
 * @return [zh] 每个面向控制的字段都有限且在物理范围内时为 true。
 * @note [en] The same rules the C entry point applies before it writes anything, and the reason `encode` refuses an
 *            unusable set: a blob is what the *next* boot reads, and a writer that emits one no reader may accept is
 *            manufacturing a boot-time failure. `temperature_c` is deliberately not checked -- it is provenance, and an
 *            unknown temperature is a legitimate state.
 * @note [zh] 与 C 入口在写入之前施加的规则相同，也是 `encode` 拒绝不可用数字的理由：这团字节是 **下一次上电** 要读的东
 *            西，而一个写出"任何读取方都不能接受"的字节的写入方，是在制造一次上电失败。 `temperature_c` 刻意不检查：它
 *            是来源信息，未知温度是合法状态。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto set_is_usable(const CommissionedSet& set) noexcept -> bool {
    if (!std::isfinite(set.rs) || !std::isfinite(set.ld) || !std::isfinite(set.lq) || !std::isfinite(set.kt) ||
        !std::isfinite(set.j) || !std::isfinite(set.b)) {
        return false;
    }
    return (set.rs >= 0.0F) && (set.ld > 0.0F) && (set.lq > 0.0F) && (set.kt > 0.0F) && (set.j > 0.0F) &&
           (set.b >= 0.0F) && (set.pole_pairs >= 1U);
}

/**
 * @brief [en] Write one set; returns the bytes written, or 0 when it does not fit or the numbers are unusable.
 * @brief [zh] 写出一组；返回写入的字节数，放不下或数字不可用时返回 0。
 * @param set [en] the values to store.
 * @param set [zh] 要存的值。
 * @param out [en] destination buffer.
 * @param out [zh] 目标缓冲区。
 * @param capacity [en] bytes available in `out`.
 * @param capacity [zh] `out` 可用的字节数。
 * @return [en] `kCommissionedSetBytes`, or 0 for a short buffer or an unusable set.
 * @return [zh] `kCommissionedSetBytes`；缓冲区不足或数字不可用时返回 0。
 * @note [en] Nothing is partially written: both refusals are decided before the first byte, because a half-written set
 *            in flash is indistinguishable from a corrupt one and turns the writer's bug into the next boot's problem.
 *            `set_is_usable()` is what distinguishes the two refusals, since both answer 0.
 * @note [zh] 不会写一半：两种拒绝都在第一个字节之前就定下来，因为 flash 里写了一半的一组与坏掉的一组无法区分，而那会把
 *            写入方的 bug 变成下一次上电的问题。两种拒绝都答 0，靠 `set_is_usable()` 区分。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto encode_commissioned_set(const CommissionedSet& set, std::uint8_t* out,
                                                  std::size_t capacity) noexcept -> std::size_t {
    if ((out == nullptr) || (capacity < kCommissionedSetBytes) || !set_is_usable(set)) {
        return 0U;
    }
    out[0] = kCommissionedSetMagic0;
    out[1] = kCommissionedSetMagic1;
    out[2] = kCommissionedSetSchema;
    out[3] = 0U;
    detail::put_u32(&out[4], set.machine_id);
    detail::put_f32(&out[8], set.rs);
    detail::put_f32(&out[12], set.ld);
    detail::put_f32(&out[16], set.lq);
    detail::put_f32(&out[20], set.kt);
    detail::put_f32(&out[24], set.j);
    detail::put_f32(&out[28], set.b);
    out[32] = set.pole_pairs;
    out[33] = 0U;
    out[34] = 0U;
    out[35] = 0U;
    detail::put_u32(&out[36], set.measured_at_unix_s);
    detail::put_f32(&out[40], set.temperature_c);
    detail::put_u16(&out[44], crc16_ccitt(out, kCommissionedSetBytes - 2U));
    return kCommissionedSetBytes;
}

/**
 * @brief [en] Read one set, saying exactly where it stopped when it is not one.
 * @brief [zh] 读出一组；不是一组时，准确说出停在哪里。
 * @param in [en] source bytes.
 * @param in [zh] 源字节。
 * @param length [en] how many bytes are available.
 * @param length [zh] 可用字节数。
 * @param out [en] receiver; written only when the status is `Ok`.
 * @param out [zh] 接收者；只在状态为 `Ok` 时写入。
 * @return [en] the status and, when it is not `Ok`, the offset it stopped at.
 * @return [zh] 状态；不是 `Ok` 时，以及停下的偏移。
 * @note [en] The order of the checks is the order a reader can afford: length before any read, marker before the
 *            schema, schema before the payload, check value before the numbers, numbers before the caller sees them.
 *            Every one of them refuses rather than repairs, and the receiver is untouched on every refusal -- a decoder
 *            that fills in a "mostly right" set is how a wrong `ld` reaches a current controller.
 * @note [zh] 检查顺序就是读取方能承受的顺序：先长度再读任何字节，先标识再格式版本，先版本再负载，先校验值再数字，先数字
 *            再让调用方看到。每一道都是拒绝而不是修补，且每次拒绝都不碰接收者——一个填入"大体正确"的一组数字的解码器，正
 *            是错误的 `ld` 走到电流环里的途径。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto decode_commissioned_set(const std::uint8_t* in, std::size_t length,
                                                  CommissionedSet& out) noexcept -> SetError {
    if (in == nullptr) {
        return SetError{SetStatus::Short, 0U};
    }
    if (length < kCommissionedSetBytes) {
        return SetError{SetStatus::Short, length};
    }
    if ((in[0] != kCommissionedSetMagic0) || (in[1] != kCommissionedSetMagic1)) {
        return SetError{SetStatus::BadMagic, (in[0] != kCommissionedSetMagic0) ? 0U : 1U};
    }
    if (in[2] != kCommissionedSetSchema) {
        return SetError{SetStatus::BadSchema, 2U};
    }
    if (in[3] != 0U) {
        return SetError{SetStatus::BadReserved, 3U};
    }
    if (in[33] != 0U) {
        return SetError{SetStatus::BadReserved, 33U};
    }
    if (in[34] != 0U) {
        return SetError{SetStatus::BadReserved, 34U};
    }
    if (in[35] != 0U) {
        return SetError{SetStatus::BadReserved, 35U};
    }
    const std::uint16_t stored = detail::get_u16(&in[kCommissionedSetBytes - 2U]);
    if (crc16_ccitt(in, kCommissionedSetBytes - 2U) != stored) {
        return SetError{SetStatus::BadCheckValue, kCommissionedSetBytes - 2U};
    }

    CommissionedSet decoded{};
    decoded.machine_id = detail::get_u32(&in[4]);
    decoded.rs = detail::get_f32(&in[8]);
    decoded.ld = detail::get_f32(&in[12]);
    decoded.lq = detail::get_f32(&in[16]);
    decoded.kt = detail::get_f32(&in[20]);
    decoded.j = detail::get_f32(&in[24]);
    decoded.b = detail::get_f32(&in[28]);
    decoded.pole_pairs = in[32];
    decoded.measured_at_unix_s = detail::get_u32(&in[36]);
    decoded.temperature_c = detail::get_f32(&in[40]);

    if (!set_is_usable(decoded)) {
        // The offset points at the first control-facing field that cannot be right, so a reader knows which number to
        // re-measure instead of which byte to stare at.
        // 偏移指向第一个不可能正确的控制字段，于是读取方知道该重测哪个数，而不是该盯着哪个字节。
        if (!std::isfinite(decoded.rs) || (decoded.rs < 0.0F)) {
            return SetError{SetStatus::BadField, 8U};
        }
        if (!std::isfinite(decoded.ld) || (decoded.ld <= 0.0F)) {
            return SetError{SetStatus::BadField, 12U};
        }
        if (!std::isfinite(decoded.lq) || (decoded.lq <= 0.0F)) {
            return SetError{SetStatus::BadField, 16U};
        }
        if (!std::isfinite(decoded.kt) || (decoded.kt <= 0.0F)) {
            return SetError{SetStatus::BadField, 20U};
        }
        if (!std::isfinite(decoded.j) || (decoded.j <= 0.0F)) {
            return SetError{SetStatus::BadField, 24U};
        }
        if (!std::isfinite(decoded.b) || (decoded.b < 0.0F)) {
            return SetError{SetStatus::BadField, 28U};
        }
        return SetError{SetStatus::BadField, 32U};
    }
    out = decoded;
    return SetError{SetStatus::Ok, 0U};
}

/**
 * @brief [en] Guard: the stored record mirrors `foc_motor_params_t`, so a field added there must reach the codec.
 * @brief [zh] 护栏：存下来的记录镜像 `foc_motor_params_t`，于是那边新增的字段必须走到编解码里。
 * @note [en] Six floats and one byte, padded to 28. This is not a style check: `from_params()` assigns field by field,
 *            so a *new* field on the ABI struct would be silently left out of every saved record, and the next boot
 *            would restore a partly-defaulted machine. The size is the only thing a compiler can watch for without a
 *            reflection layer, so it is watched -- whoever grows the struct gets this error and has to decide what the
 *            record does with the new number, in schema 2.
 * @note [zh] 六个 float 加一个字节，填充到 28。这不是风格检查： `from_params()` 逐字段赋值，于是 ABI 结构上 **新增** 的
 *            字段会被静默地从每一份存盘里漏掉，下一次上电就会恢复出一台部分默认的机器。没有反射层时，编译器只能盯住尺
 *            寸，那就盯住尺寸——把结构体变大的人会碰到这个错误，并且必须在格式 2 里决定这条记录拿那个新数字怎么办。
 * @since 0.1.0
 */
static_assert(sizeof(foc_motor_params_t) == 28U,
              "foc_motor_params_t changed size: extend the commissioned-set layout and bump its schema");

/**
 * @brief [en] The seven numbers as the frozen C ABI carries them.
 * @brief [zh] 以冻结 C ABI 的方式携带的那七个数字。
 * @param set [en] the stored record.
 * @param set [zh] 存下来的记录。
 * @return [en] the same numbers in `foc_motor_params_t`, ready for `foc_motor_apply_params()`.
 * @return [zh] 同一批数字的 `foc_motor_params_t` 形态，可直接交给 `foc_motor_apply_params()`。
 * @note [en] Exists so an integrator does not write the seven assignments by hand: a transposed pair here is a wrong
 *            machine that still passes every type check, and it would be found by a motor rather than by a compiler.
 * @note [zh] 它存在的意义是让集成方不必手写那七行赋值：这里把两对写反，就是一台仍然通过所有类型检查的错误机器，而发现它
 *            的会是电机而不是编译器。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto to_params(const CommissionedSet& set) noexcept -> foc_motor_params_t {
    foc_motor_params_t params{};
    params.rs = set.rs;
    params.ld = set.ld;
    params.lq = set.lq;
    params.kt = set.kt;
    params.j = set.j;
    params.b = set.b;
    params.pole_pairs = set.pole_pairs;
    return params;
}

/**
 * @brief [en] A record built from the numbers a motor was given, with the provenance left for the caller.
 * @brief [zh] 由一台电机被交给的数字构成的记录，来源信息留给调用方。
 * @param params [en] machine parameters, as `foc_motor_get_params()` reports them.
 * @param params [zh] 机器参数，即 `foc_motor_get_params()` 报出的样子。
 * @return [en] a set with those numbers and `machine_id` / provenance still at their defaults.
 * @return [zh] 带有这些数字、而 `machine_id` 与来源信息仍为默认值的一组。
 * @note [en] The provenance is not copied from anywhere because there is nowhere to copy it from: those are facts about
 *            the measurement, and only the party that took it knows them.
 * @note [zh] 来源信息不从任何地方拷贝，因为没有地方可拷：那些是关于这次测量的事实，只有做测量的那一方知道。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto from_params(const foc_motor_params_t& params) noexcept -> CommissionedSet {
    CommissionedSet set{};
    set.rs = params.rs;
    set.ld = params.ld;
    set.lq = params.lq;
    set.kt = params.kt;
    set.j = params.j;
    set.b = params.b;
    set.pole_pairs = params.pole_pairs;
    return set;
}

} // namespace comm

} // namespace foc