// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>

#include <foc/comm/commissioned_set.hpp>
#include <foc/core/board_check.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_comm
 * @file board_record.hpp
 *
 * @brief [en] One production test's outcome in bytes: which unit, when, and every row it judged.
 * @brief [zh] 一次产线测试的结果写成字节：哪台机器、什么时候、以及它判过的每一行。
 * @since 0.1.0
 */

namespace foc {

namespace comm {


/**
 * @brief [en] Fixed size of one record, as any row of the table covered.
 * @brief [zh] 一条记录的固定字节数。
 * @since 0.1.0
 */
inline constexpr std::size_t kBoardRecordBytes = 164U;

/**
 * @brief [en] First marker byte; `'B'`.
 * @brief [zh] 第一个标识字节； `'B'`。
 * @since 0.1.0
 */
inline constexpr std::uint8_t kBoardRecordMagic0 = 0x42U;

/**
 * @brief [en] Second marker byte; `'R'`. Different from the set's, so the two blobs cannot be mistaken.
 * @brief [zh] 第二个标识字节； `'R'`。与标定集合的不同，所以两团字节不会被认错。
 * @since 0.1.0
 */
inline constexpr std::uint8_t kBoardRecordMagic1 = 0x52U;

/**
 * @brief [en] Layout version of this record.
 * @brief [zh] 本记录的布局版本。
 * @since 0.1.0
 */
inline constexpr std::uint8_t kBoardRecordSchema = 1U;

/**
 * @brief [en] How many rows fit in one record.
 * @brief [zh] 一条记录能装下多少行。
 * @since 0.1.0
 */
inline constexpr std::uint8_t kBoardRecordRows = 12U;

/**
 * @brief [en] Bytes per row: item, phase, verdict, one reserved byte, then value and limit.
 * @brief [zh] 每行的字节数：项、相、判定、一个保留字节，然后 value 与 limit。
 * @since 0.1.0
 */
inline constexpr std::size_t kBoardRecordRowBytes = 12U;

static_assert(18U + (static_cast<std::size_t>(kBoardRecordRows) * kBoardRecordRowBytes) + 2U == kBoardRecordBytes,
              "kBoardRecordBytes is the header, the rows and the check value, and nothing else");

/**
 * @brief [en] How many checks `core::BoardItem` has, asserted against the enumeration.
 * @brief [zh] `core::BoardItem` 有几项，并对枚举断言。
 * @note [en] A ninth check fails the build here until this constant is raised. Without that, a record written by the
 *            newer build reads back in the older one with an item no `switch` handles, and the decoder would be doing
 *            the range check nobody asked it to skip.
 * @note [zh] 第九项检查会让这里编译不过，直到这个常数被加上去。没有它，新构建写出的记录在旧构建里读回来会带着一个任何
 *            `switch` 都不处理的项，而解码器就在做一次没人让它跳过的范围检查。
 * @since 0.1.0
 */
inline constexpr std::uint8_t kBoardRecordItems = 8U;

static_assert(static_cast<std::uint8_t>(core::BoardItem::ObserverHealth) + 1U == kBoardRecordItems,
              "kBoardRecordItems must match core::BoardItem: a new check needs a bigger record, not a silent default");

/**
 * @brief [en] One production test, in the form that survives a power cycle.
 *
 *   The rows are `core::BoardReading`, not a second vocabulary for the same fact: a record that had to be translated
 *   before it could be compared with a live report would be two definitions of "pass" waiting to disagree.
 *
 *   What the encoding guarantees, and why each of them is there:
 *
 *   1. **Fixed size.** 152 bytes whatever the row count, with the unused rows required to be zero. A variable-length
 *   record makes every reader carry a length arithmetic, and that arithmetic is where a corrupt length turns into a
 *   read past the buffer.
 *
 *   2. **The padding is checked, not ignored.** A non-zero byte in a row past `count` is refused as a `BadRow` rather
 *   than skipped: it means the writer had something to say that this reader cannot honour, exactly like a non-zero
 *   reserved byte.
 *
 *   3. **A check value over the payload.** A record that fails it is refused whole; the receiver is not written, so a
 *   caller keeping the previous report keeps it intact.
 *
 * @brief [zh] 一次产线测试，写成能跨断电的形式。
 *
 *   行就是 `core::BoardReading`，不是同一批事实的第二套词汇：一条要先翻译才能与实时报告相比的记录，等于给"通过" 写了两
 *   个定义，等着它们打架。
 *
 *   编码保证三件事，每一件都有理由：
 *
 *   1. **定长。** 不管几行都是 152 字节，没用到的行必须为零。变长记录会让每个读取方都带上长度算术，而那段算术正是一个坏
 *   长度变成越界读的地方。
 *
 *   2. **填充是被检查的，不是被忽略的。** `count` 之后的行里出现非零字节，以 `BadRow` 拒绝而不是跳过：它意味着写入方有
 *   话说，而这个读取方无法兑现——与保留字节非零是同一件事。
 *
 *   3. **负载上有校验值。** 校验不过就整条拒绝，且不写接收者，于是保留上一份报告的调用方能完整保留它。
 *
 * @context [en] Store and load time only: no ISR, no allocation, byte-at-a-time like the set beside it.
 * @context [zh] 只在存取时用：不在 ISR 里、不分配、与旁边那组一样逐字节处理。
 * @since 0.1.0
 * @exporter
 */
struct BoardRecord final {
    /**
     * @brief [en] the caller's identifier for the unit; not interpreted by this library.
     * @brief [zh] 调用方给这台机器的标识；本库不解释它。
     */
    std::uint32_t machine_id{0U};
    /**
     * @brief [en] when it was tested, unix seconds; 0 = not recorded.
     * @brief [zh] 什么时候测的，unix 秒；0 = 没有记录。
     */
    std::uint32_t checked_at_unix_s{0U};
    /**
     * @brief [en] the board temperature at the test, deg C; NaN = not recorded.
     * @brief [zh] 测试时的板温，摄氏度；NaN = 没有记录。
     */
    float temperature_c{std::nanf("")};
    /**
     * @brief [en] the one cause to act on, the same word the live report carries.
     * @brief [zh] 可以动手的那一个成因，与实时报告同一个词。
     */
    core::BoardCause cause{core::BoardCause::None};
    /**
     * @brief [en] rows filled, 0..12.
     * @brief [zh] 已填行数，0..12。
     */
    std::uint8_t count{0U};
    /**
     * @brief [en] one row per check; entries at or past `count` must be zero.
     * @brief [zh] 每项检查一行； `count` 及之后的项必须为零。
     */
    core::BoardReading rows[kBoardRecordRows]{};
};

/**
 * @brief [en] Whether this record is something a reader may accept, judged row by row.
 * @brief [zh] 这条记录是不是读取方能接受的东西，逐行判定。
 * @param record [en] the record to judge.
 * @param record [zh] 要判定的记录。
 * @return [en] true when the count fits, every used row has finite value and limit, `pass` is 0 or 1, `phase` is 0..3,
 *              `item` is a check this build knows, and every unused row is zero.
 * @return [zh] 计数装得下、每个用到的行 value 与 limit 有限、 `pass` 为 0 或 1、 `phase` 为 0..3、 `item` 是本构建知道
 *              的检查、且每个未用的行为零时为 true。
 * @note [en] The same rules the encoder applies before it writes anything, and the reason `encode_board_record()`
 *            refuses an unusable record: a blob is what the *next* boot or the EOL database reads, and a writer that
 *            emits one no reader may accept is manufacturing a failure somewhere else.
 * @note [zh] 与编码器在写第一个字节之前施加的规则相同，也是 `encode_board_record()` 拒绝不可用记录的理由：这团字节是
 *            **下一次上电** 或产线数据库要读的东西，而一个写出"任何读取方都不能接受"的字节的写入方，是在别处制造一次失
 *            败。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto record_is_usable(const BoardRecord& record) noexcept -> bool {
    if (record.count > kBoardRecordRows) {
        return false;
    }
    for (std::uint8_t i = 0U; i < kBoardRecordRows; ++i) {
        const core::BoardReading& row = record.rows[i];
        const bool used = i < record.count;
        if (!used) {
            // An unused row must be zero: a writer with something to say in it is saying something this reader will
            // not act on. `signbit` is part of the test because `-0.0F` compares equal to `0.0F` and encodes as
            // `0x80000000`: a padding row holding a negative zero would pass a value check here and be refused on the
            // way back in, which makes the encoder manufacture a blob its own decoder rejects.
            // 未用到的行必须为零：在里面说话，等于说一件本读取方不会处置的事。`signbit` 是这条测试的一部分，因为
            // `-0.0F` 比较起来等于 `0.0F`、却编码成 `0x80000000`：一个装着负零的填充行会在这里过值检查，却在读回来的
            // 路上被拒——那就成了编码器自己造出一团自家解码器不认的字节。
            if ((row.item != core::BoardItem::CurrentSum) || (row.phase != 0U) || row.pass || (row.value != 0.0F) ||
                (row.limit != 0.0F) || std::signbit(row.value) || std::signbit(row.limit)) {
                return false;
            }
            continue;
        }
        if ((!std::isfinite(row.value)) || (!std::isfinite(row.limit))) {
            return false;
        }
        if ((row.phase > 3U) || (static_cast<std::uint8_t>(row.item) >= kBoardRecordItems)) {
            return false;
        }
    }
    return true;
}

/**
 * @brief [en] Write one record; returns the bytes written, or 0 when it does not fit or is unusable.
 * @brief [zh] 写出一条记录；返回写入的字节数，放不下或记录不可用时返回 0。
 * @param record [en] the values to store.
 * @param record [zh] 要存的值。
 * @param out [en] destination buffer.
 * @param out [zh] 目标缓冲区。
 * @param capacity [en] bytes available in `out`.
 * @param capacity [zh] `out` 可用的字节数。
 * @return [en] `kBoardRecordBytes`, or 0.
 * @return [zh] `kBoardRecordBytes`，或 0。
 * @note [en] Nothing is partially written: both refusals are decided before the first byte, because a half-written
 *            record in flash is indistinguishable from a corrupt one.
 * @note [zh] 不会写一半：两种拒绝都在第一个字节之前定下来，因为 flash 里写了一半的记录与坏掉的一条无法区分。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto encode_board_record(const BoardRecord& record, std::uint8_t* out,
                                              std::size_t capacity) noexcept -> std::size_t {
    if ((out == nullptr) || (capacity < kBoardRecordBytes) || !record_is_usable(record)) {
        return 0U;
    }
    out[0] = kBoardRecordMagic0;
    out[1] = kBoardRecordMagic1;
    out[2] = kBoardRecordSchema;
    out[3] = 0U;
    detail::put_u32(&out[4], record.machine_id);
    detail::put_u32(&out[8], record.checked_at_unix_s);
    detail::put_f32(&out[12], record.temperature_c);
    out[16] = static_cast<std::uint8_t>(record.cause);
    out[17] = record.count;
    for (std::uint8_t i = 0U; i < kBoardRecordRows; ++i) {
        std::uint8_t* row = &out[18U + (static_cast<std::size_t>(i) * kBoardRecordRowBytes)];
        row[0] = static_cast<std::uint8_t>(record.rows[i].item);
        row[1] = record.rows[i].phase;
        row[2] = record.rows[i].pass ? 1U : 0U;
        row[3] = 0U;
        detail::put_f32(&row[4], record.rows[i].value);
        detail::put_f32(&row[8], record.rows[i].limit);
    }
    detail::put_u16(&out[kBoardRecordBytes - 2U], crc16_ccitt(out, kBoardRecordBytes - 2U));
    return kBoardRecordBytes;
}

/**
 * @brief [en] Read one record, saying exactly where it stopped when it is not one.
 * @brief [zh] 读出一条记录；不是一条时，准确说出停在哪里。
 * @param in [en] source bytes.
 * @param in [zh] 源字节。
 * @param length [en] how many bytes are available.
 * @param length [zh] 可用字节数。
 * @param out [en] receiver; written only when the status is `Ok`.
 * @param out [zh] 接收者；只在状态为 `Ok` 时写入。
 * @return [en] the same status vocabulary the commissioned set uses, plus the offset it stopped at.
 * @return [zh] 与标定集合同一套状态词，外加它停下的偏移。
 * @note [en] The order is the order a reader can afford: length before any read, marker before schema, schema before
 *            payload, check value before the numbers, numbers before the padding -- because the padding check needs
 *            `count`, and `count` is only trustworthy once the check value has passed.
 * @note [zh] 顺序就是读取方能承受的顺序：先长度再读字节、先标识再格式版本、先版本再负载、先校验值再数字，最后才是填充
 *            ——因为填充检查需要 `count`，而 `count` 只有在校验值通过之后才可信。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto decode_board_record(const std::uint8_t* in, std::size_t length, BoardRecord& out) noexcept
    -> SetError {
    if (in == nullptr) {
        return SetError{SetStatus::Short, 0U};
    }
    if (length < kBoardRecordBytes) {
        return SetError{SetStatus::Short, length};
    }
    if ((in[0] != kBoardRecordMagic0) || (in[1] != kBoardRecordMagic1)) {
        return SetError{SetStatus::BadMagic, (in[0] != kBoardRecordMagic0) ? 0U : 1U};
    }
    if (in[2] != kBoardRecordSchema) {
        return SetError{SetStatus::BadSchema, 2U};
    }
    if (in[3] != 0U) {
        return SetError{SetStatus::BadReserved, 3U};
    }
    for (std::uint8_t i = 0U; i < kBoardRecordRows; ++i) {
        if (in[18U + (static_cast<std::size_t>(i) * kBoardRecordRowBytes) + 3U] != 0U) {
            return SetError{SetStatus::BadReserved, 18U + (static_cast<std::size_t>(i) * kBoardRecordRowBytes) + 3U};
        }
    }
    const std::uint16_t stored = detail::get_u16(&in[kBoardRecordBytes - 2U]);
    if (crc16_ccitt(in, kBoardRecordBytes - 2U) != stored) {
        return SetError{SetStatus::BadCheckValue, kBoardRecordBytes - 2U};
    }

    BoardRecord decoded{};
    decoded.machine_id = detail::get_u32(&in[4]);
    decoded.checked_at_unix_s = detail::get_u32(&in[8]);
    decoded.temperature_c = detail::get_f32(&in[12]);
    decoded.cause = static_cast<core::BoardCause>(in[16]);
    decoded.count = in[17];
    for (std::uint8_t i = 0U; i < kBoardRecordRows; ++i) {
        const std::uint8_t* row = &in[18U + (static_cast<std::size_t>(i) * kBoardRecordRowBytes)];
        decoded.rows[i].item = static_cast<core::BoardItem>(row[0]);
        decoded.rows[i].phase = row[1];
        decoded.rows[i].pass = (row[2] != 0U);
        decoded.rows[i].value = detail::get_f32(&row[4]);
        decoded.rows[i].limit = detail::get_f32(&row[8]);
    }

    if (decoded.count > kBoardRecordRows) {
        return SetError{SetStatus::BadField, 17U};
    }
    for (std::uint8_t i = 0U; i < kBoardRecordRows; ++i) {
        const core::BoardReading& row = decoded.rows[i];
        const std::size_t at = 18U + (static_cast<std::size_t>(i) * kBoardRecordRowBytes);
        if (i >= decoded.count) {
            // Judged on the bytes rather than on the decoded values: the padding is a reserved space, and a reserved
            // byte that is not zero is the same refusal here as it is in the header.
            // 按字节判而不是按解出来的值判：填充是一段保留空间，而一个不为零的保留字节，在这里与在头部是同一种拒绝。
            for (std::size_t k = 0U; k < kBoardRecordRowBytes; ++k) {
                if (in[at + k] != 0U) {
                    return SetError{SetStatus::BadReserved, at + k};
                }
            }
            continue;
        }
        if (!std::isfinite(row.value) || !std::isfinite(row.limit)) {
            return SetError{SetStatus::BadField, at + 4U};
        }
        if (static_cast<std::uint8_t>(row.item) >= kBoardRecordItems) {
            return SetError{SetStatus::BadField, at};
        }
        if (row.phase > 3U) {
            return SetError{SetStatus::BadField, at + 1U};
        }
        if (in[at + 2U] > 1U) {
            return SetError{SetStatus::BadField, at + 2U};
        }
    }
    out = decoded;
    return SetError{SetStatus::Ok, 0U};
}

} // namespace comm

} // namespace foc
