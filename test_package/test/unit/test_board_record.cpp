// One production test's outcome as bytes: it round-trips exactly, and every way it can be wrong is named.
// 一次产线测试的结果写成字节：它能逐位往返，而每一种坏法都有名字。
//
// The encoder is judged against a decoder written to the same layout, so most of these cases would pass on a
// self-consistent bug. The two that would not are the ones that matter: every single-byte corruption has to be
// refused, and the *offsets* of a hand-crafted payload have to point at the field that broke the rules, not at the
// check value that noticed it.
// 编码器是拿同布局的解码器来判的，所以多数用例在"自洽的 bug"下也会过。真正有分辨力的是两条：每一个单字节变异都必须被拒，
// 以及手工拼出的负载里的 **偏移** 必须指向违规的那个字段，而不是发现它的那个校验值。

#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <gtest/gtest.h>

#include <foc/comm/board_record.hpp>
#include <foc/comm/frame.hpp>

namespace {

using foc::comm::BoardRecord;
using foc::comm::decode_board_record;
using foc::comm::encode_board_record;
using foc::comm::kBoardRecordBytes;
using foc::comm::kBoardRecordMagic0;
using foc::comm::kBoardRecordMagic1;
using foc::comm::kBoardRecordRowBytes;
using foc::comm::kBoardRecordRows;
using foc::comm::kBoardRecordSchema;
using foc::comm::record_is_usable;
using foc::comm::SetError;
using foc::comm::SetStatus;

constexpr std::size_t kRow0 = 18U;

auto row_offset(std::uint8_t index) -> std::size_t {
    return kRow0 + (static_cast<std::size_t>(index) * kBoardRecordRowBytes);
}

// Recompute the check value after a deliberate edit, so the case exercises the *field* checks rather than stopping at
// the check value every time.
// 手工改完之后重算校验值，这样用例检验的是 **字段** 规则，而不是每次都停在校验值上。
void fix_crc(std::uint8_t* bytes) {
    const std::uint16_t crc = foc::comm::crc16_ccitt(bytes, kBoardRecordBytes - 2U);
    bytes[kBoardRecordBytes - 2U] = static_cast<std::uint8_t>(crc & 0xFFU);
    bytes[kBoardRecordBytes - 1U] = static_cast<std::uint8_t>((crc >> 8U) & 0xFFU);
}

void put_f32(std::uint8_t* at, float value) {
    std::uint32_t raw = 0U;
    std::memcpy(&raw, &value, sizeof(raw));
    at[0] = static_cast<std::uint8_t>(raw & 0xFFU);
    at[1] = static_cast<std::uint8_t>((raw >> 8U) & 0xFFU);
    at[2] = static_cast<std::uint8_t>((raw >> 16U) & 0xFFU);
    at[3] = static_cast<std::uint8_t>((raw >> 24U) & 0xFFU);
}

auto sample() -> BoardRecord {
    BoardRecord record{};
    record.machine_id = 0x00C0FFEEU;
    record.checked_at_unix_s = 1'760'000'000U;
    record.temperature_c = std::nanf("");
    record.cause = foc::core::BoardCause::OffsetBias;
    record.count = 5U;

    const foc::core::BoardItem items[5] = {
        foc::core::BoardItem::CurrentSum,      foc::core::BoardItem::CurrentOffset,
        foc::core::BoardItem::CurrentPolarity, foc::core::BoardItem::SamplingInstant,
        foc::core::BoardItem::DeadTime,
    };
    for (std::uint8_t i = 0U; i < record.count; ++i) {
        record.rows[i].item = items[i];
        record.rows[i].phase = (i < 3U) ? i : 3U;
        record.rows[i].pass = (i != 3U); // one failing row, so both verdicts are carried
        record.rows[i].value = 0.125F * static_cast<float>(i + 1U);
        record.rows[i].limit = -0.0625F * static_cast<float>(i + 1U);
    }
    return record;
}

auto as_bytes(const BoardRecord& record, std::uint8_t* buffer) -> std::size_t {
    return encode_board_record(record, buffer, kBoardRecordBytes);
}

auto bytes_of_sample() -> std::array<std::uint8_t, kBoardRecordBytes> {
    std::array<std::uint8_t, kBoardRecordBytes> out{};
    const std::size_t written = encode_board_record(sample(), out.data(), out.size());
    assert(written == kBoardRecordBytes);
    return out;
}

} // namespace


/**
 * @brief [en] A record that goes out comes back with every field the same, including the one that was never recorded.
 *
 * @brief [zh] 写出去的记录读回来每个字段都一样，包括那个从没被记录过的。
 *
 *   `temperature_c` is NaN here on purpose: the point of a provenance field is that "not recorded" survives the trip,
 *   and a NaN that comes back as `0.0F` is a measurement nobody made. The comparison is bitwise on the floats for the
 *   same reason a `==` test would hide a quiet NaN turning into a different quiet NaN.
 *
 * @note [en] Criterion, written before the run: the byte count is `kBoardRecordBytes`, the status is `Ok` with offset
 *            0, every scalar compares equal, and re-encoding the decoded record reproduces the buffer byte for byte.
 * @note [zh] 判据（写在本次测量之前）：字节数等于 `kBoardRecordBytes`、状态为 `Ok` 且偏移 0、每个标量比较相等，且对解出
 *            的记录再编码一次能逐字节复现缓冲区。
 */
TEST(BoardRecord, ARecordRoundTripsWithItsUnrecordedTemperatureStillUnrecorded) {
    const BoardRecord original = sample();
    ASSERT_TRUE(record_is_usable(original));

    std::uint8_t bytes[kBoardRecordBytes] = {};
    ASSERT_EQ(kBoardRecordBytes, as_bytes(original, bytes));

    BoardRecord decoded{};
    const SetError error = decode_board_record(bytes, kBoardRecordBytes, decoded);
    ASSERT_EQ(SetStatus::Ok, error.status);
    EXPECT_EQ(0U, error.offset);

    EXPECT_EQ(original.machine_id, decoded.machine_id);
    EXPECT_EQ(original.checked_at_unix_s, decoded.checked_at_unix_s);
    EXPECT_TRUE(std::isnan(decoded.temperature_c));
    EXPECT_EQ(original.cause, decoded.cause);
    EXPECT_EQ(original.count, decoded.count);
    for (std::uint8_t i = 0U; i < kBoardRecordRows; ++i) {
        EXPECT_EQ(original.rows[i].item, decoded.rows[i].item) << "row " << int{i};
        EXPECT_EQ(original.rows[i].phase, decoded.rows[i].phase) << "row " << int{i};
        EXPECT_EQ(original.rows[i].pass, decoded.rows[i].pass) << "row " << int{i};
        EXPECT_EQ(0, std::memcmp(&original.rows[i].value, &decoded.rows[i].value, sizeof(float))) << "row " << int{i};
        EXPECT_EQ(0, std::memcmp(&original.rows[i].limit, &decoded.rows[i].limit, sizeof(float))) << "row " << int{i};
    }

    std::uint8_t again[kBoardRecordBytes] = {};
    ASSERT_EQ(kBoardRecordBytes, as_bytes(decoded, again));
    EXPECT_EQ(0, std::memcmp(bytes, again, kBoardRecordBytes));
}


/**
 * @brief [en] Every single-byte corruption of the blob is refused.
 *
 * @brief [zh] 这团字节里 **每一个** 单字节变异都被拒绝。
 *
 *   One byte at a time, XOR-ed with 0xFF, over all 152 positions. This is the case that makes the check value and the
 *   padding rule into one statement: a flip in the header, in a payload byte, in a padding byte or in the check value
 *   itself must never come back `Ok`. It also catches the specific mistake of leaving the padding unvalidated, because
 *   a flip there keeps the record otherwise intact.
 *
 * @note [en] The receiver is poisoned before each attempt and must still hold the poison on the refusals that do not
 *            reach it; the case counts the accepted corruptions and prints it, so a future layout that accepts one has
 *            to say so out loud.
 * @note [zh] 每次尝试之前把接收者"下毒"，拒绝的那几次它必须仍然带着毒；用例把被接受的变异个数数出来打印，这样将来某个布
 *            局放过了其中一个，就必须明说。
 */
TEST(BoardRecord, NoSingleByteCorruptionIsAccepted) {
    std::uint8_t bytes[kBoardRecordBytes] = {};
    ASSERT_EQ(kBoardRecordBytes, as_bytes(sample(), bytes));

    int accepted = 0;
    for (std::size_t i = 0U; i < kBoardRecordBytes; ++i) {
        std::uint8_t corrupted[kBoardRecordBytes] = {};
        std::memcpy(corrupted, bytes, kBoardRecordBytes);
        corrupted[i] = static_cast<std::uint8_t>(corrupted[i] ^ 0xFFU);

        BoardRecord receiver{};
        receiver.machine_id = 0xDEADBEEFU; // poison: a refusal must not write through
        const SetError error = decode_board_record(corrupted, kBoardRecordBytes, receiver);
        if (error.status == SetStatus::Ok) {
            ++accepted;
            ADD_FAILURE() << "byte " << i << " was corrupted and still read as a record";
        }
        EXPECT_EQ(0xDEADBEEFU, receiver.machine_id) << "byte " << i;
    }
    std::printf("\n[board-record] %d of %zu single-byte corruptions accepted\n", accepted, kBoardRecordBytes);
    EXPECT_EQ(0, accepted);
}


/**
 * @brief [en] A buffer that is one byte short gets nothing written into it.
 *
 * @brief [zh] 少一个字节的缓冲区，一个字节都不写。
 *
 *   The writer is the one place where "mostly written" is not recoverable: a record truncated in flash is
 *   indistinguishable from a corrupt one, and the caller's next boot reads it.
 *
 * @note [en] The buffer is poisoned first, so "wrote nothing" is checked rather than assumed.
 * @note [zh] 先把缓冲区下毒，于是"什么都没写"是被检查过的，而不是被假定的。
 */
TEST(BoardRecord, AShortBufferIsRefusedAndNothingIsWritten) {
    std::uint8_t buffer[kBoardRecordBytes + 4U] = {};
    std::memset(buffer, 0xA5, sizeof(buffer));
    EXPECT_EQ(0U, encode_board_record(sample(), buffer, kBoardRecordBytes - 1U));
    for (const std::uint8_t byte : buffer) {
        EXPECT_EQ(0xA5U, byte);
    }
    EXPECT_EQ(0U, encode_board_record(sample(), nullptr, kBoardRecordBytes));
}


/**
 * @brief [en] The three structural refusals name themselves and point at the byte that failed.
 *
 * @brief [zh] 三种结构性拒绝各说各的名字，并指向出问题的那个字节。
 *
 *   A decoder that answers "invalid record" sends the reader to a hex dump; the offset is the part that says which of
 *   the 152 bytes the writer got wrong. The check order is part of the contract too: a wrong marker is reported as a
 *   marker, not as a check value, because the two send a person to different places.
 *
 * @note [en] Criterion, written before the run: a wrong marker answers `BadMagic` at the marker byte, a wrong schema
 *            `BadSchema` at 2, a non-zero reserved byte `BadReserved` at 3, and a corrupted payload byte
 *            `BadCheckValue` at the check value's offset -- in that order of precedence.
 * @note [zh] 判据（写在本次测量之前）：标识错答 `BadMagic` 且指向标识字节、格式版本错答 `BadSchema` 指向 2、保留字节非
 *            零答 `BadReserved` 指向 3、负载字节坏了答 `BadCheckValue` 指向校验值的偏移——按这个优先级。
 */
TEST(BoardRecord, TheStructuralRefusalsNameThemselvesAndPointAtTheByte) {
    std::uint8_t bytes[kBoardRecordBytes] = {};
    ASSERT_EQ(kBoardRecordBytes, as_bytes(sample(), bytes));

    {
        std::uint8_t copy[kBoardRecordBytes] = {};
        std::memcpy(copy, bytes, kBoardRecordBytes);
        copy[0] = static_cast<std::uint8_t>(kBoardRecordMagic0 ^ 0x01U);
        BoardRecord receiver{};
        const SetError error = decode_board_record(copy, kBoardRecordBytes, receiver);
        EXPECT_EQ(SetStatus::BadMagic, error.status);
        EXPECT_EQ(0U, error.offset);
    }
    {
        std::uint8_t copy[kBoardRecordBytes] = {};
        std::memcpy(copy, bytes, kBoardRecordBytes);
        copy[1] = static_cast<std::uint8_t>(kBoardRecordMagic1 ^ 0x01U);
        BoardRecord receiver{};
        const SetError error = decode_board_record(copy, kBoardRecordBytes, receiver);
        EXPECT_EQ(SetStatus::BadMagic, error.status);
        EXPECT_EQ(1U, error.offset);
    }
    {
        std::uint8_t copy[kBoardRecordBytes] = {};
        std::memcpy(copy, bytes, kBoardRecordBytes);
        copy[2] = static_cast<std::uint8_t>(kBoardRecordSchema + 1U);
        BoardRecord receiver{};
        const SetError error = decode_board_record(copy, kBoardRecordBytes, receiver);
        EXPECT_EQ(SetStatus::BadSchema, error.status);
        EXPECT_EQ(2U, error.offset);
    }
    {
        std::uint8_t copy[kBoardRecordBytes] = {};
        std::memcpy(copy, bytes, kBoardRecordBytes);
        copy[3] = 0x5AU;
        BoardRecord receiver{};
        const SetError error = decode_board_record(copy, kBoardRecordBytes, receiver);
        EXPECT_EQ(SetStatus::BadReserved, error.status);
        EXPECT_EQ(3U, error.offset);
    }
    {
        // A payload byte, with the check value left alone: the check value is what notices.
        std::uint8_t copy[kBoardRecordBytes] = {};
        std::memcpy(copy, bytes, kBoardRecordBytes);
        copy[row_offset(0U) + 4U] = static_cast<std::uint8_t>(copy[row_offset(0U) + 4U] ^ 0x01U);
        BoardRecord receiver{};
        const SetError error = decode_board_record(copy, kBoardRecordBytes, receiver);
        EXPECT_EQ(SetStatus::BadCheckValue, error.status);
        EXPECT_EQ(kBoardRecordBytes - 2U, error.offset);
    }
    {
        BoardRecord receiver{};
        const SetError error = decode_board_record(bytes, kBoardRecordBytes - 1U, receiver);
        EXPECT_EQ(SetStatus::Short, error.status);
        EXPECT_EQ(kBoardRecordBytes - 1U, error.offset);
    }
}


/**
 * @brief [en] A used row that cannot be one is refused **at its own offset**, even with a valid check value.
 *
 * @brief [zh] 用到的行若不可能是真的，即使校验值有效，也在 **它自己的偏移** 上被拒。
 *
 *   These are the four ways a row can lie: a non-finite measurement, a check this build does not have, a phase outside
 *   0..2, and a verdict that is neither. Recomputing the check value is what makes the case measure the field rules
 *   instead of the checksum -- otherwise every edit below would be answered `BadCheckValue` and the offsets would never
 *   be exercised.
 *
 * @note [en] Criterion, written before the run: each edit answers `BadField` at the byte it broke, not at the row start
 *            and not at the check value.
 * @note [zh] 判据（写在本次测量之前）：每一处改动都答 `BadField`，并指向它弄坏的那个字节——不是行首，也不是校验值。
 */
TEST(BoardRecord, AUsedRowThatCannotBeOneIsRefusedAtItsOwnOffset) {
    const auto decode_edit = [](auto&& edit, std::size_t expect_offset) {
        std::uint8_t copy[kBoardRecordBytes] = {};
        std::memcpy(copy, bytes_of_sample().data(), kBoardRecordBytes);
        edit(copy);
        fix_crc(copy);
        BoardRecord receiver{};
        const SetError error = decode_board_record(copy, kBoardRecordBytes, receiver);
        EXPECT_EQ(SetStatus::BadField, error.status) << "offset " << error.offset;
        EXPECT_EQ(expect_offset, error.offset);
    };

    // A measurement that is not a number.
    decode_edit([](std::uint8_t* b) { put_f32(&b[row_offset(0U) + 4U], std::nanf("")); }, row_offset(0U) + 4U);
    // A check this build does not have.
    decode_edit([](std::uint8_t* b) { b[row_offset(0U)] = foc::comm::kBoardRecordItems; }, row_offset(0U));
    // A phase outside 0..2.
    decode_edit([](std::uint8_t* b) { b[row_offset(0U) + 1U] = 4U; }, row_offset(0U) + 1U);
    // A verdict that is neither pass nor fail.
    decode_edit([](std::uint8_t* b) { b[row_offset(0U) + 2U] = 2U; }, row_offset(0U) + 2U);
}


/**
 * @brief [en] A padding row that speaks is refused, not skipped.
 *
 * @brief [zh] 一个"说话"的填充行被拒绝，而不是被跳过。
 *
 *   The padding is reserved space, so a non-zero byte in it means the writer had something to say that this reader
 *   cannot honour -- the same statement a non-zero reserved byte in the header makes. Skipping it would let a newer
 *   writer put a sixth row into an older record and have it silently ignored, which is the failure mode this rule
 *   exists to prevent.
 *
 * @note [en] The case also covers the count: a record claiming more rows than the layout has is `BadField` at the count
 *            byte, because the count is a number that cannot be one.
 * @note [zh] 这条用例也覆盖计数：声称行数多于布局允许的记录，在计数字节上答 `BadField`，因为那个计数不可能为真。
 */
TEST(BoardRecord, APaddingRowThatSpeaksOrACountThatCannotBeIsRefused) {
    {
        std::uint8_t copy[kBoardRecordBytes] = {};
        std::memcpy(copy, bytes_of_sample().data(), kBoardRecordBytes);
        copy[row_offset(7U) + 6U] = 0x01U; // a byte inside row 7's value, well past `count`
        fix_crc(copy);
        BoardRecord receiver{};
        const SetError error = decode_board_record(copy, kBoardRecordBytes, receiver);
        EXPECT_EQ(SetStatus::BadReserved, error.status);
        EXPECT_EQ(row_offset(7U) + 6U, error.offset);
    }
    {
        std::uint8_t copy[kBoardRecordBytes] = {};
        std::memcpy(copy, bytes_of_sample().data(), kBoardRecordBytes);
        copy[17] = static_cast<std::uint8_t>(kBoardRecordRows + 1U);
        fix_crc(copy);
        BoardRecord receiver{};
        const SetError error = decode_board_record(copy, kBoardRecordBytes, receiver);
        EXPECT_EQ(SetStatus::BadField, error.status);
        EXPECT_EQ(17U, error.offset);
    }
}


/**
 * @brief [en] A negative zero is not a padding zero, and the encoder refuses the record that would need it to be.
 *
 * @brief [zh] 负零不是填充零，而编码器会拒绝那种"需要它是零"的记录。
 *
 *   `-0.0F == 0.0F` is true and their encodings differ, so a padding row holding a negative zero would pass a value
 *   check in the encoder and be refused by the decoder -- the encoder manufacturing a blob its own reader rejects. The
 *   rule is enforced where the record is built, which is the only place it can be fixed.
 *
 * @note [en] Both the padding row and a used row are checked: the first is a refused record, the second is legitimate
 *            (a `-0.0F` measurement is a number) and must still encode.
 * @note [zh] 填充行与用到的行都检查：前者是被拒的记录，后者是合法的（ `-0.0F` 是一个数）且必须仍然能编码。
 */
TEST(BoardRecord, ANegativeZeroInPaddingIsNotAPaddingZero) {
    BoardRecord record = sample();
    record.rows[9].value = -0.0F;
    EXPECT_FALSE(record_is_usable(record));
    std::uint8_t buffer[kBoardRecordBytes] = {};
    EXPECT_EQ(0U, encode_board_record(record, buffer, kBoardRecordBytes));

    BoardRecord used = sample();
    used.rows[0].value = -0.0F; // a real measurement of negative zero is a measurement
    EXPECT_TRUE(record_is_usable(used));
    std::uint8_t bytes[kBoardRecordBytes] = {};
    ASSERT_EQ(kBoardRecordBytes, encode_board_record(used, bytes, kBoardRecordBytes));
    BoardRecord decoded{};
    ASSERT_EQ(SetStatus::Ok, decode_board_record(bytes, kBoardRecordBytes, decoded).status);
    EXPECT_EQ(0, std::memcmp(&used.rows[0].value, &decoded.rows[0].value, sizeof(float)));
}
