// The frame codec, measured: is the CRC the standard's, does every single-byte corruption get refused, and does the
// sequence counter report a frame that never arrived?
//
// The criteria, written before the run:
//
//   1. the CRC is CRC-16/CCITT-FALSE and not merely *a* CRC: the standard's own check value, 0x29B1, for the ASCII
//      string "123456789";
//   2. a frame survives a round trip byte for byte, payload included;
//   3. **no single-byte corruption is accepted** -- every byte of a real frame replaced by every other value, 3570
//      candidate frames, all refused. Exhaustive, not sampled: a CRC is either a code with a minimum distance or it is
//      decoration, and the only way to know which is to try them all;
//   4. a truncated or over-long buffer is refused *before* any read past its end, and the receiver is left untouched --
//      a decoder that fills the receiver and then reports a bad CRC hands back a plausible frame with one bit wrong;
//   5. the sequence counter reports the frames that never arrived (a skip of one reports 1, not 0), flags a duplicate
//      without moving its expectation, treats 255 -> 0 as in order, and calls a backward jump a regression.
//
// 判据写在跑前（见上）：CRC 必须是 CRC-16/CCITT-FALSE 而不只是"某个 CRC"（标准检查值 0x29B1）；往返逐字节一致；**没有
// 任何一个单字节改动被接受**（14 个字节各换成其余 255 个值，3570 个候选帧全部拒绝——穷举而不是抽样）；截断与超长的缓冲区
// 在越界读之前就被拒绝且接收者不被触碰；序号计数报告从未到达的帧（跳一帧报 1 而不是 0）、把重复标记出来且不推进期望值、
// 把 255→0 当作顺序、把倒退报成回退。

#include <array>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <gtest/gtest.h>

#include <foc/comm/frame.hpp>

namespace {

namespace comm = foc::comm;

using comm::decode_frame;
using comm::encode_frame;
using comm::Frame;
using comm::FrameLayout;
using comm::FrameStatus;
using comm::SequenceChecker;

constexpr std::uint8_t kType = 0x21U;

// 1. The check value is what pins the variant. Without it "it is a CRC-16 CCITT" is not transferable to any other
//    implementation, because six things carry that name and they disagree.
TEST(Frame, TheCrcIsTheStandardsVariantNotJustSomeCrc) {
    const char* check = "123456789";
    const auto crc = comm::crc16_ccitt(reinterpret_cast<const std::uint8_t*>(check), std::strlen(check));
    EXPECT_EQ(crc, 0x29B1U) << "CRC-16/CCITT-FALSE of \"123456789\" is 0x29B1; any other value is another variant";
}

// 2. A round trip keeps every field and leaves the payload where the sender put it.
TEST(Frame, ARoundTripKeepsEveryByte) {
    const std::array<std::uint8_t, 8> payload{0x00U, 0xFFU, 0x01U, 0x80U, 0x7FU, 0xA5U, 0x5AU, 0x42U};
    std::array<std::uint8_t, 32> wire{};
    const std::size_t written = encode_frame(7U, kType, payload.data(), static_cast<std::uint8_t>(payload.size()),
                                             wire.data(), wire.size());
    ASSERT_EQ(written, FrameLayout::kOverhead + payload.size());

    Frame frame{};
    ASSERT_EQ(decode_frame(wire.data(), written, frame), FrameStatus::Ok);
    EXPECT_EQ(frame.seq, 7U);
    EXPECT_EQ(frame.type, kType);
    EXPECT_EQ(frame.length, payload.size());
    EXPECT_EQ(0, std::memcmp(frame.payload, payload.data(), payload.size()));
}

// 3. Exhaustive: every byte, every other value. This is the assertion that says the CRC is doing work.
TEST(Frame, NoSingleByteCorruptionIsAccepted) {
    const std::array<std::uint8_t, 8> payload{1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
    std::array<std::uint8_t, 32> wire{};
    const std::size_t written =
        encode_frame(0x33U, kType, payload.data(), static_cast<std::uint8_t>(payload.size()), wire.data(), wire.size());
    ASSERT_EQ(written, FrameLayout::kOverhead + payload.size());

    std::uint32_t trials = 0U;
    std::uint32_t accepted = 0U;
    for (std::size_t position = 0U; position < written; ++position) {
        const std::uint8_t original = wire[position];
        for (std::uint32_t value = 0U; value < 256U; ++value) {
            if (value == original) {
                continue;
            }
            wire[position] = static_cast<std::uint8_t>(value);
            Frame frame{};
            ++trials;
            if (decode_frame(wire.data(), written, frame) == FrameStatus::Ok) {
                ++accepted;
                std::printf("undetected corruption at byte %zu -> 0x%02X\n", position, value);
            }
        }
        wire[position] = original;
    }
    EXPECT_EQ(trials, written * 255U) << "the sweep must cover every byte and every other value";
    EXPECT_EQ(accepted, 0U) << accepted << " of " << trials << " single-byte corruptions decoded as valid";
    // And the intact frame still decodes, so the loop above did not simply leave the buffer broken.
    Frame frame{};
    EXPECT_EQ(decode_frame(wire.data(), written, frame), FrameStatus::Ok);
}

// 4. The three refusals that protect the receiver's memory, and the fact that `out` is untouched on each of them.
TEST(Frame, TheRefusalsComeBeforeAnyReadPastTheBuffer) {
    const std::array<std::uint8_t, 4> payload{9U, 8U, 7U, 6U};
    std::array<std::uint8_t, 32> wire{};
    const std::size_t written =
        encode_frame(1U, kType, payload.data(), static_cast<std::uint8_t>(payload.size()), wire.data(), wire.size());
    ASSERT_EQ(written, FrameLayout::kOverhead + payload.size());

    Frame untouched{};
    untouched.seq = 0xEEU;
    untouched.length = 0xEEU;

    // A buffer shorter than the header alone.
    Frame a = untouched;
    EXPECT_EQ(decode_frame(wire.data(), FrameLayout::kOverhead - 1U, a), FrameStatus::TooShort);

    // The right length, but starting one byte late -- the resynchronisation failure.
    Frame b = untouched;
    EXPECT_EQ(decode_frame(wire.data() + 1, written - 1U, b), FrameStatus::BadStart);

    // A length field that runs past the buffer: the frame claims 200 payload bytes out of a 10-byte read.
    std::array<std::uint8_t, 32> lying = wire;
    lying[3] = 200U;
    Frame c = untouched;
    EXPECT_EQ(decode_frame(lying.data(), written, c), FrameStatus::BadLength);

    // A single flipped bit in the payload, with a correct length: the checksum is the one that has to catch it.
    std::array<std::uint8_t, 32> flipped = wire;
    flipped[5] ^= 0x01U;
    Frame d = untouched;
    EXPECT_EQ(decode_frame(flipped.data(), written, d), FrameStatus::BadChecksum);

    for (const Frame& frame : {a, b, c, d}) {
        EXPECT_EQ(frame.seq, 0xEEU) << "a refused frame must not write the receiver";
        EXPECT_EQ(frame.length, 0xEEU);
        EXPECT_EQ(frame.payload, nullptr);
    }
}

// 4b. Encoding into a buffer that is too small writes nothing at all.
TEST(Frame, AFrameThatDoesNotFitWritesNothing) {
    const std::array<std::uint8_t, 8> payload{1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U};
    std::array<std::uint8_t, 32> out{};
    out.fill(0x5AU);
    const std::size_t written = encode_frame(0U, kType, payload.data(), 8U, out.data(), FrameLayout::kOverhead + 7U);
    EXPECT_EQ(written, 0U);
    for (const std::uint8_t byte : out) {
        EXPECT_EQ(byte, 0x5AU) << "a half frame on a link is indistinguishable from a corrupt one";
    }
}

// 5. The sequence counter: what a CRC cannot report, reported.
TEST(Frame, TheSequenceCounterReportsWhatTheCrcCannot) {
    SequenceChecker checker;
    EXPECT_EQ(checker.observe(10U).missing, 0U) << "the first frame establishes the expectation";

    // In order.
    auto r = checker.observe(11U);
    EXPECT_EQ(r.missing, 0U);
    EXPECT_FALSE(r.duplicate);
    EXPECT_FALSE(r.out_of_order);

    // One frame never arrived: 12 then 14. That is one missing frame, and reporting 0 here would be the whole failure.
    EXPECT_EQ(checker.observe(12U).missing, 0U);
    r = checker.observe(14U);
    EXPECT_EQ(r.missing, 1U);
    EXPECT_FALSE(r.duplicate);

    // The counter is now expecting 15, so a frame that fills the hole late is out of order, not a loss.
    r = checker.observe(13U);
    EXPECT_TRUE(r.out_of_order);
    EXPECT_EQ(r.missing, 0U);

    // A duplicate is flagged and does not move the expectation: 15 arrives twice, then 16 is still in order.
    EXPECT_EQ(checker.observe(15U).missing, 0U);
    EXPECT_TRUE(checker.observe(15U).duplicate);
    r = checker.observe(16U);
    EXPECT_FALSE(r.duplicate);
    EXPECT_EQ(r.missing, 0U);

    // 255 -> 0 is one step, not a regression.
    SequenceChecker wrap;
    static_cast<void>(wrap.observe(254U));
    r = wrap.observe(255U);
    EXPECT_EQ(r.missing, 0U);
    r = wrap.observe(0U);
    EXPECT_EQ(r.missing, 0U);
    EXPECT_FALSE(r.out_of_order);

    // A three-frame loss is reported as three, and the expectation resynchronises on the frame that did arrive.
    r = checker.observe(20U);
    EXPECT_EQ(r.missing, 3U);
    EXPECT_EQ(checker.observe(21U).missing, 0U);
}

} // namespace
