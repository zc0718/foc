// What a saved commissioning result has to survive: a round trip with no lost bits, a single changed byte anywhere, a
// field no machine can have, and a version this build does not read.
//
// 存下来的一次调试结果必须扛住什么：不丢一位的往返、任意位置改一个字节、一个不是任何电机能有的字段，以及本构建不读的版本。

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <gtest/gtest.h>

#include <foc/comm/commissioned_set.hpp>

namespace {

using foc::comm::CommissionedSet;
using foc::comm::SetStatus;

auto bits_of(float value) -> std::uint32_t {
    std::uint32_t raw = 0U;
    std::memcpy(&raw, &value, sizeof(raw));
    return raw;
}

auto representative_set() -> CommissionedSet {
    CommissionedSet set{};
    set.machine_id = 0x0BADF00DU;
    set.rs = 0.503674F;
    set.ld = 4.0e-4F;
    set.lq = 8.0e-4F;
    set.kt = 0.0421F;
    set.j = 1.0e-4F;
    set.b = 3.0e-5F;
    set.pole_pairs = 7U;
    set.measured_at_unix_s = 1762900000U;
    set.temperature_c = 41.5F;
    return set;
}

auto encoded(const CommissionedSet& set, std::uint8_t* out) -> std::size_t {
    return foc::comm::encode_commissioned_set(set, out, foc::comm::kCommissionedSetBytes);
}

auto refresh_check_value(std::uint8_t* bytes) -> void {
    foc::comm::detail::put_u16(&bytes[foc::comm::kCommissionedSetBytes - 2U],
                               foc::comm::crc16_ccitt(bytes, foc::comm::kCommissionedSetBytes - 2U));
}

auto sentinel_set() -> CommissionedSet {
    CommissionedSet set{};
    set.machine_id = 0xA5A5A5A5U;
    set.rs = -1.0F;
    set.ld = -1.0F;
    set.lq = -1.0F;
    set.kt = -1.0F;
    set.j = -1.0F;
    set.b = -1.0F;
    set.pole_pairs = 0U;
    set.measured_at_unix_s = 0xA5A5A5A5U;
    set.temperature_c = -1.0F;
    return set;
}

} // namespace

TEST(CommissionedSet, TheRoundTripReturnsTheSameBits) {
    const CommissionedSet set = representative_set();
    std::uint8_t bytes[foc::comm::kCommissionedSetBytes] = {};
    ASSERT_EQ(encoded(set, bytes), foc::comm::kCommissionedSetBytes);

    CommissionedSet back{};
    const foc::comm::SetError error = foc::comm::decode_commissioned_set(bytes, sizeof(bytes), back);
    EXPECT_EQ(error.status, SetStatus::Ok) << "stopped at byte " << error.offset;
    EXPECT_EQ(back.machine_id, set.machine_id);
    EXPECT_EQ(bits_of(back.rs), bits_of(set.rs));
    EXPECT_EQ(bits_of(back.ld), bits_of(set.ld));
    EXPECT_EQ(bits_of(back.lq), bits_of(set.lq));
    EXPECT_EQ(bits_of(back.kt), bits_of(set.kt));
    EXPECT_EQ(bits_of(back.j), bits_of(set.j));
    EXPECT_EQ(bits_of(back.b), bits_of(set.b));
    EXPECT_EQ(back.pole_pairs, set.pole_pairs);
    EXPECT_EQ(back.measured_at_unix_s, set.measured_at_unix_s);
    EXPECT_EQ(bits_of(back.temperature_c), bits_of(set.temperature_c));
}

TEST(CommissionedSet, AnUnrecordedTemperatureComesBackAsTheSameNan) {
    // "Not recorded" is a state the format has to carry, and NaN is the documented marker -- so the round trip has to
    // preserve it rather than normalise it to a number.
    // "没有记录"是格式必须承载的一个状态，而 NaN 是文档化的标记——所以往返必须保留它，而不是把它规整成一个数字。
    CommissionedSet set = representative_set();
    set.temperature_c = std::nanf("");
    std::uint8_t bytes[foc::comm::kCommissionedSetBytes] = {};
    ASSERT_EQ(encoded(set, bytes), foc::comm::kCommissionedSetBytes);

    CommissionedSet back{};
    ASSERT_EQ(foc::comm::decode_commissioned_set(bytes, sizeof(bytes), back).status, SetStatus::Ok);
    EXPECT_TRUE(std::isnan(back.temperature_c));
    EXPECT_EQ(bits_of(back.temperature_c), bits_of(set.temperature_c)) << "the payload NaN changed identity";
}

TEST(CommissionedSet, EverySingleByteChangeIsRefused) {
    // The claim is not "it has a CRC", it is "no single changed byte anywhere is accepted". Every position is swept
    // through all 255 alternatives, which is the cheapest way to make the statement exhaustive rather than sampled.
    // 主张不是"它有个 CRC"，而是"任意一个字节被改过都不被接受"。每个位置都扫过全部 255 种取值，这是把话说成穷举而不是
    // 抽样的最便宜办法。
    const CommissionedSet set = representative_set();
    std::uint8_t good[foc::comm::kCommissionedSetBytes] = {};
    ASSERT_EQ(encoded(set, good), foc::comm::kCommissionedSetBytes);

    int accepted = 0;
    int first_accepted_offset = -1;
    for (std::size_t offset = 0U; offset < foc::comm::kCommissionedSetBytes; ++offset) {
        for (unsigned value = 1U; value < 256U; ++value) {
            std::uint8_t mutated[foc::comm::kCommissionedSetBytes] = {};
            std::memcpy(mutated, good, sizeof(mutated));
            mutated[offset] = static_cast<std::uint8_t>(static_cast<unsigned>(mutated[offset]) ^ value);
            CommissionedSet ignored = sentinel_set();
            if (foc::comm::decode_commissioned_set(mutated, sizeof(mutated), ignored).status == SetStatus::Ok) {
                ++accepted;
                if (first_accepted_offset < 0) {
                    first_accepted_offset = static_cast<int>(offset);
                }
            }
        }
    }
    std::printf("[commissioned-set] %d candidate(s) accepted out of %d\n", accepted,
                static_cast<int>(foc::comm::kCommissionedSetBytes * 255U));
    EXPECT_EQ(accepted, 0) << "first accepted mutation at byte " << first_accepted_offset;

    // And the sweep did not merely break the buffer: the untouched bytes still decode.
    // 而这次扫描也没有把缓冲区弄坏：未被触碰的字节仍然能解出来。
    CommissionedSet back{};
    EXPECT_EQ(foc::comm::decode_commissioned_set(good, sizeof(good), back).status, SetStatus::Ok);
}

TEST(CommissionedSet, AFieldThatCannotBeRightIsNamedInsteadOfReturned) {
    // A stale-but-intact record is the dangerous case: the bytes are pure, the check value matches, and one number is
    // impossible. The reader has to say *which* number, because that is what tells a technician what to re-measure.
    // 字节完好但内容过期的记录才危险：字节是干净的、校验值对得上，而其中一个数字不可能。读取方必须说出是**哪一个**，
    // 因为那才是告诉技师该重测什么的东西。
    const CommissionedSet set = representative_set();
    std::uint8_t bytes[foc::comm::kCommissionedSetBytes] = {};
    ASSERT_EQ(encoded(set, bytes), foc::comm::kCommissionedSetBytes);

    foc::comm::detail::put_f32(&bytes[12], 0.0F); // ld = 0 with a valid check value
    refresh_check_value(bytes);

    CommissionedSet receiver = sentinel_set();
    const foc::comm::SetError error = foc::comm::decode_commissioned_set(bytes, sizeof(bytes), receiver);
    EXPECT_EQ(error.status, SetStatus::BadField);
    EXPECT_EQ(error.offset, 12U) << "the offset must name the field, not the frame";
    EXPECT_EQ(receiver.machine_id, 0xA5A5A5A5U) << "a refused decode wrote into the receiver";
    EXPECT_EQ(bits_of(receiver.ld), bits_of(-1.0F)) << "a refused decode wrote into the receiver";
}

TEST(CommissionedSet, AShortBufferIsRefusedBeforeItIsRead) {
    const CommissionedSet set = representative_set();
    std::uint8_t bytes[foc::comm::kCommissionedSetBytes] = {};
    ASSERT_EQ(encoded(set, bytes), foc::comm::kCommissionedSetBytes);

    CommissionedSet receiver = sentinel_set();
    const foc::comm::SetError empty = foc::comm::decode_commissioned_set(bytes, 0U, receiver);
    EXPECT_EQ(empty.status, SetStatus::Short);
    EXPECT_EQ(empty.offset, 0U);

    const foc::comm::SetError one_short =
        foc::comm::decode_commissioned_set(bytes, foc::comm::kCommissionedSetBytes - 1U, receiver);
    EXPECT_EQ(one_short.status, SetStatus::Short);
    EXPECT_EQ(one_short.offset, foc::comm::kCommissionedSetBytes - 1U);
    EXPECT_EQ(receiver.machine_id, 0xA5A5A5A5U);
}

TEST(CommissionedSet, AVersionThisBuildDoesNotReadIsRefusedRatherThanGuessed) {
    const CommissionedSet set = representative_set();
    std::uint8_t bytes[foc::comm::kCommissionedSetBytes] = {};
    ASSERT_EQ(encoded(set, bytes), foc::comm::kCommissionedSetBytes);

    bytes[2] = static_cast<std::uint8_t>(foc::comm::kCommissionedSetSchema + 1U);
    refresh_check_value(bytes);

    CommissionedSet receiver = sentinel_set();
    const foc::comm::SetError error = foc::comm::decode_commissioned_set(bytes, sizeof(bytes), receiver);
    EXPECT_EQ(error.status, SetStatus::BadSchema);
    EXPECT_EQ(error.offset, 2U);
    EXPECT_EQ(receiver.machine_id, 0xA5A5A5A5U);
}

TEST(CommissionedSet, AReservedByteThatMeansSomethingIsRefused) {
    // A zero reserved byte is the writer saying "I meant nothing by this". A non-zero one is a writer that knew about a
    // meaning this reader does not have, so honouring the rest of the record would be guessing.
    // 保留字节为零，是写入方说"我在这里没有表达任何意思"。非零则是一个知道某种含义、而本读取方没有那种含义的写入方，
    // 于是继续相信记录里其余的部分就是在猜。
    const CommissionedSet set = representative_set();
    std::uint8_t bytes[foc::comm::kCommissionedSetBytes] = {};
    ASSERT_EQ(encoded(set, bytes), foc::comm::kCommissionedSetBytes);

    bytes[33] = 1U;
    refresh_check_value(bytes);

    CommissionedSet receiver = sentinel_set();
    const foc::comm::SetError error = foc::comm::decode_commissioned_set(bytes, sizeof(bytes), receiver);
    EXPECT_EQ(error.status, SetStatus::BadReserved);
    EXPECT_EQ(error.offset, 33U);
}

TEST(CommissionedSet, EncodingRefusesRatherThanWritingSomethingNoReaderMayAccept) {
    // Two refusals answer 0 and both have to leave the buffer alone: a half-written record in flash is
    // indistinguishable from a corrupt one, and the next boot is the one that pays for it.
    // 两种拒绝都答 0，而且都必须不动缓冲区：flash 里写了一半的记录与坏掉的无法区分，而付代价的是下一次上电。
    CommissionedSet unusable = representative_set();
    unusable.ld = 0.0F;
    std::uint8_t bytes[foc::comm::kCommissionedSetBytes] = {};
    std::memset(bytes, 0x5A, sizeof(bytes));
    EXPECT_EQ(encoded(unusable, bytes), 0U);
    for (std::size_t i = 0U; i < sizeof(bytes); ++i) {
        ASSERT_EQ(bytes[i], 0x5AU) << "byte " << i << " was written by a refused encode";
    }

    const CommissionedSet set = representative_set();
    std::memset(bytes, 0x5A, sizeof(bytes));
    EXPECT_EQ(foc::comm::encode_commissioned_set(set, bytes, foc::comm::kCommissionedSetBytes - 1U), 0U);
    for (std::size_t i = 0U; i < sizeof(bytes); ++i) {
        ASSERT_EQ(bytes[i], 0x5AU) << "byte " << i << " was written into a short buffer";
    }
    EXPECT_FALSE(foc::comm::set_is_usable(unusable)) << "the two refusals are told apart by this predicate";
}

TEST(CommissionedSet, ANullBufferIsRefusedRatherThanDereferenced) {
    // The two pointers come from a caller's storage layer, which is exactly where a NULL arrives by accident. Both
    // entry points answer a status rather than faulting, and the encoder still writes nothing.
    // 这两个指针来自调用方的存储层，而那正是 NULL 会意外出现的地方。两个入口都回答一个状态而不是触发异常，编码端仍然不写。
    CommissionedSet receiver = sentinel_set();
    EXPECT_EQ(foc::comm::decode_commissioned_set(nullptr, 0U, receiver).status, SetStatus::Short);
    EXPECT_EQ(receiver.machine_id, 0xA5A5A5A5U);

    const CommissionedSet set = representative_set();
    std::uint8_t bytes[foc::comm::kCommissionedSetBytes] = {};
    EXPECT_EQ(foc::comm::encode_commissioned_set(set, nullptr, foc::comm::kCommissionedSetBytes), 0U);
}

TEST(CommissionedSet, ARecordThatNamesNoPolePairsIsRefusedLikeAnyOtherImpossibleField) {
    // The last field in the record is also a field a machine cannot do without, so the refusal has to name it rather
    // than fall off the end of the chain and answer `Ok`.
    // 记录里的最后一个字段同样是机器不可或缺的字段，所以拒绝必须点到它，而不是从检查链末尾掉出去答 `Ok`。
    const CommissionedSet set = representative_set();
    std::uint8_t bytes[foc::comm::kCommissionedSetBytes] = {};
    ASSERT_EQ(encoded(set, bytes), foc::comm::kCommissionedSetBytes);

    bytes[32] = 0U; // pole pairs
    refresh_check_value(bytes);

    CommissionedSet receiver = sentinel_set();
    const foc::comm::SetError error = foc::comm::decode_commissioned_set(bytes, sizeof(bytes), receiver);
    EXPECT_EQ(error.status, SetStatus::BadField);
    EXPECT_EQ(error.offset, 32U);
    EXPECT_EQ(receiver.machine_id, 0xA5A5A5A5U);
}
