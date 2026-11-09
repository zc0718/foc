// F20: absolute-encoder frames on a bit-oriented link. The dangerous failure here is not "it crashes" -- it is "it answers a
// plausible angle", because a position controller will act on that. So every criterion below is about a *refusal* producing
// no value at all, and the CRC's teeth are proved by flipping every single bit rather than by spot-checking one.
// F20：面向位链路上的绝对值编码器帧。这里危险的失败不是"崩了"——而是"给出了一个看起来合理的角度"，因为位置控制器会据此动作。
// 所以下面每条判据都是关于"拒绝且不产生任何值"的，而 CRC 的牙齿靠 **逐位翻转** 证明，不靠抽查一位。

#include <cmath>
#include <cstring>
#include <vector>

#include <etl/bit_stream.h>
#include <gtest/gtest.h>

#include <foc/comm/encoder_stream.hpp>

namespace {

using foc::ErrorCode;
using foc::comm::crc6;
using foc::comm::decode_biss;
using foc::comm::EncoderFrame;
using foc::comm::EncoderLayout;
using foc::comm::TurnAssembler;

constexpr int kSt = 12; // 4096 counts per revolution 每转 4096 计数
constexpr int kMt = 4;  // 16 revolutions, so the frame is 4 + 12 + 2 + 6 = 24 bits 16 转，于是帧是 24 位

auto layout() -> EncoderLayout {
    EncoderLayout out{};
    out.single_turn_bits = kSt;
    out.multi_turn_bits = kMt;
    return out;
}

/** @brief Pack a frame the way the encoder would: turns, position, error, warning, CRC. */
// 按编码器的方式拼一帧：转数、位置、错误位、警告位、CRC。
auto pack(std::uint32_t turns, std::uint32_t position, bool error_bit = false, bool warning = false)
    -> std::vector<std::uint8_t> {
    const EncoderLayout shape = layout();
    std::vector<std::uint8_t> bits(static_cast<std::size_t>((shape.frame_bits() + 7) / 8), 0U);
    int index = 0;
    auto put = [&](std::uint32_t value, int count) {
        for (int n = count - 1; n >= 0; --n) {
            const auto bit = static_cast<std::uint8_t>((value >> static_cast<unsigned>(n)) & 0x01U);
            bits[static_cast<std::size_t>(index / 8)] |= static_cast<std::uint8_t>(bit << ((8 - 1) - (index % 8)));
            ++index;
        }
    };
    put(turns, kMt);
    put(position, kSt);
    put(error_bit ? 1U : 0U, 1);
    put(warning ? 1U : 0U, 1);
    const std::uint8_t crc = crc6(bits.data(), shape.data_bits() + 2);
    put(crc, EncoderLayout::kCrcBits);
    return bits;
}

} // namespace

// The headline: the numbers that come back are the numbers that went in, bit for bit, in both fields and for a position on each
// side of the single-turn boundary. "Approximately the angle" is not a criterion here -- the wire is a count, not a
// measurement.
// 要点：回来的数与进去的数逐位相同，两个字段都是，而且位置取在转内边界的每一边。"大致是这个角度"在这里不是判据——线上是计数，
// 不是测量。
TEST(EncoderStream, DecodesTheTransmittedCountsExactly) {
    const EncoderLayout shape = layout();
    EXPECT_EQ(24, shape.frame_bits());
    for (const std::uint32_t turns : {0U, 1U, 7U, 15U}) {
        for (const std::uint32_t position : {0U, 1U, 2047U, 4095U}) {
            const std::vector<std::uint8_t> bits = pack(turns, position);
            const EncoderFrame frame = decode_biss(bits.data(), shape.frame_bits(), shape);
            ASSERT_EQ(ErrorCode::Ok, frame.status) << "turns " << turns << " position " << position;
            EXPECT_TRUE(frame.crc_ok);
            EXPECT_EQ(position, frame.position);
            EXPECT_EQ(turns, frame.turns);
            EXPECT_EQ(((turns << kSt) | position), frame.raw_data);
            EXPECT_FALSE(frame.error_bit);
            EXPECT_FALSE(frame.warning);
        }
    }
}

// The CRC's teeth, proved exhaustively: a six-bit CRC detects **every** single-bit error, so every one of the 24 positions must
// be caught. A spot check of one bit would also pass for a CRC that happened to be an identity on that bit, and the exhaustive
// version costs nothing at this frame size.
// CRC 的牙齿，用穷举证明：六位 CRC 检出 **每一个** 单比特错误，所以 24 个位置一个都不能漏。抽查一位也能被那种"恰好在该位上等价于
// 恒等"的 CRC 通过，而在这个帧长下穷举不要任何代价。
TEST(EncoderStream, EverySingleBitFlipIsCaught) {
    const EncoderLayout shape = layout();
    const std::vector<std::uint8_t> good = pack(3U, 1234U);
    const EncoderFrame clean = decode_biss(good.data(), shape.frame_bits(), shape);
    ASSERT_EQ(ErrorCode::Ok, clean.status) << "the un-flipped frame must decode, or this case proves nothing";

    int caught = 0;
    for (int bit = 0; bit < shape.frame_bits(); ++bit) {
        std::vector<std::uint8_t> flipped = good;
        flipped[static_cast<std::size_t>(bit / 8)] ^= static_cast<std::uint8_t>(1U << ((8 - 1) - (bit % 8)));
        const EncoderFrame frame = decode_biss(flipped.data(), shape.frame_bits(), shape);
        EXPECT_FALSE(frame.crc_ok) << "bit " << bit << " slipped through";
        EXPECT_EQ(ErrorCode::State, frame.status) << "bit " << bit;
        // A frame that failed is reported as arrived-and-not-trusted, never as `Ok` with a shifted value.
        // 失败的帧被报成"到了但不可信"，绝不是 `Ok` 加一个被挪过的值。
        if (!frame.crc_ok) {
            ++caught;
        }
    }
    EXPECT_EQ(shape.frame_bits(), caught);
}

// The failure a value-based check cannot catch is a stream one bit out of alignment, and the honest result of this case is that
// **the CRC does not catch it**. A six-bit CRC detects every single-bit error (the case above proves that exhaustively) and every
// burst up to six bits, but a shift is neither -- it is a different message, and for the frame measured here it passes. So
// alignment has to be the *framing* layer's job: the length check refuses a stream that is not this frame's length, and the
// protocol's own start/stop bits are what a real link uses to know where a frame begins. Recording this as a measured boundary is
// what stops the next person from "strengthening the CRC" to fix a problem the CRC was never able to see.
// 靠"值看起来对不对"抓不住的失败，是 **错开一位** 的码流，而本用例的诚实结果是： **CRC 抓不住它** 。六位 CRC 检出每一个单比特错误
// （上面那条用例穷举证明了这一点）以及长度不超过六位的突发，但错位两者都不是——它是另一条报文，而在实测的这一帧上它 **通过** 。
// 所以对齐只能是 **成帧** 层的活：长度检查拒绝长度不对的码流，而协议自己的起始/停止位才是真实链路用来知道帧从哪里开始的东西。
// 把它记成一条量出来的边界，是为了让下一个人不去"加固 CRC"来修一个 CRC 本来就看不见的问题。
TEST(EncoderStream, TheLengthIsRefusedAndAShiftIsTheFramingLayersProblemNotTheCrc) {
    const EncoderLayout shape = layout();
    const std::vector<std::uint8_t> good = pack(5U, 2000U);
    EXPECT_EQ(ErrorCode::Ok, decode_biss(good.data(), shape.frame_bits(), shape).status);

    // Wrong length: one bit short, one bit long -- refused, and with no value handed back.
    // 长度不对：少一位、多一位——拒绝，而且不交回任何值。
    const EncoderFrame too_short = decode_biss(good.data(), shape.frame_bits() - 1, shape);
    EXPECT_EQ(ErrorCode::Param, too_short.status);
    EXPECT_EQ(0U, too_short.position) << "a refusal must not hand back a value";
    EXPECT_EQ(0U, too_short.turns);
    EXPECT_EQ(ErrorCode::Param, decode_biss(good.data(), shape.frame_bits() + 1, shape).status);

    // Same length, shifted by one bit: prepend a zero and drop the last bit, exactly what a misaligned shift register produces.
    // 同长度、错开一位：前面补一个零、丢掉最后一位，正是错位的移位寄存器所产生的东西。
    std::vector<std::uint8_t> shifted(good.size(), 0U);
    for (int bit = 0; bit < shape.frame_bits(); ++bit) {
        const int from = bit - 1;
        if (from < 0) {
            continue;
        }
        const auto value =
            static_cast<std::uint8_t>((good[static_cast<std::size_t>(from / 8)] >> ((8 - 1) - (from % 8))) & 0x01U);
        shifted[static_cast<std::size_t>(bit / 8)] |= static_cast<std::uint8_t>(value << ((8 - 1) - (bit % 8)));
    }
    const EncoderFrame misaligned = decode_biss(shifted.data(), shape.frame_bits(), shape);
    RecordProperty("shift_passes_crc", misaligned.crc_ok ? 1 : 0);
    EXPECT_TRUE(misaligned.crc_ok)
        << "measured: a one-bit shift of this frame passes the six-bit CRC, which is why the "
           "length check and the framing layer carry alignment";
    // And that is exactly why a *value* check cannot be the defence: the shifted frame decodes to a different, plausible angle.
    // 而这正说明"值检查"不能当防线：错位的那一帧解出的是另一个、看起来合理的角度。
    EXPECT_NE(decode_biss(good.data(), shape.frame_bits(), shape).position, misaligned.position);

    // What the CRC does buy, besides single bits: every burst up to six bits is caught.
    // CRC 除了单比特之外确实买到的东西：长度不超过六位的突发全部被抓。
    for (int start_bit = 0; start_bit < shape.frame_bits() - 6; ++start_bit) {
        std::vector<std::uint8_t> burst = good;
        for (int k = 0; k < 6; ++k) {
            const int bit = start_bit + k;
            burst[static_cast<std::size_t>(bit / 8)] ^= static_cast<std::uint8_t>(1U << ((8 - 1) - (bit % 8)));
        }
        EXPECT_FALSE(decode_biss(burst.data(), shape.frame_bits(), shape).crc_ok)
            << "a six-bit burst starting at " << start_bit << " slipped through";
    }
}

// The assembler's two hard cases, and they are the opposite of each other: a **wrap** is arithmetic (`4090 -> 5` is +11 counts)
// while a **gap** is not (`4090 -> 5` could be +11 or +4107, and the frames in between are gone). Both are asserted, because a
// version that treated every decrease as a wrap would quietly invent a revolution, and a version that treated a wrap as a gap
// would refuse a perfectly healthy rotation.
// 拼接器的两个难例，而它们彼此相反： **回绕** 是算术（`4090 -> 5` 是 +11 个计数），而 **缺口** 不是（`4090 -> 5` 可能是 +11，也可能是
// +4107，中间那些帧已经不在了）。两者都断言，因为"把每一次下降都当回绕"的版本会悄悄编造一圈，而"把回绕当缺口"的版本会拒绝一次
// 完全健康的转动。
TEST(EncoderStream, AWrapIsArithmeticButAGapIsReportedAndNotGuessed) {
    TurnAssembler assembler{
        kSt, 500U}; // the axis cannot move more than 500 of 4096 counts per frame 轴每帧最多走 4096 中的 500
    EXPECT_FALSE(assembler.started());

    const auto first = assembler.push(4090U, true);
    ASSERT_EQ(ErrorCode::Ok, first.status);
    ASSERT_TRUE(assembler.started());
    EXPECT_EQ(0, first.unwrapped) << "the first frame seeds the origin";

    // Forward across the wrap: +11 counts, and no complaint.
    // 正向穿过回绕：+11 个计数，且没有抱怨。
    const auto wrapped = assembler.push(5U, true);
    ASSERT_EQ(ErrorCode::Ok, wrapped.status);
    EXPECT_FALSE(wrapped.jumped);
    EXPECT_EQ(11, wrapped.delta);
    EXPECT_EQ(11, assembler.unwrapped());

    // Backward across the wrap: -11 counts.
    // 反向穿过回绕：−11 个计数。
    const auto back = assembler.push(4090U, true);
    ASSERT_EQ(ErrorCode::Ok, back.status);
    EXPECT_EQ(-11, back.delta);
    EXPECT_EQ(0, assembler.unwrapped());

    // Now a real gap: further than the axis can travel in one frame. The raw difference also has to be interpreted first (`4090 ->
    // 2500` is -1590 raw, i.e. +2506 as a wrap), and *both* readings exceed the declared 500 counts -- which is what makes it a
    // gap rather than a fast move.
    // 现在是一个真的缺口：远于轴在一帧内能走的距离。原始差值还要先解释一次（`4090 -> 2500` 原样是 −1590，按回绕是 +2506），而
    // **两种** 读法都超过声称的 500 个计数——这才让它成为缺口而不是一次快速移动。
    const auto before_gap = assembler.unwrapped();
    const auto gap = assembler.push(2500U, true);
    EXPECT_EQ(ErrorCode::State, gap.status);
    EXPECT_TRUE(gap.jumped);
    EXPECT_EQ(before_gap, gap.unwrapped) << "the count must not move across a gap";
    EXPECT_EQ(before_gap, assembler.unwrapped()) << "and it must not be re-seeded either";

    // Continuity resumes from the last count the assembler actually trusted.
    // 连续性从拼接器真正相信过的最后一个计数恢复。
    const auto resumed = assembler.push(4092U, true);
    ASSERT_EQ(ErrorCode::Ok, resumed.status);
    EXPECT_EQ(2, resumed.delta);
}

// A frame that failed its CRC is not fed at all, and a count that cannot exist at this resolution is refused: neither may move the
// origin, because "the last reading was garbage" and "the axis moved" are different events.
// CRC 不过的帧根本不参与拼接，而这个分辨率下不可能存在的计数同样被拒绝：两者都不能移动原点，因为"上一次读数是垃圾"与"轴动了"
// 是两件不同的事。
TEST(EncoderStream, AFailedFrameNeitherMovesNorReseedsTheOrigin) {
    TurnAssembler assembler{
        kSt, 500U}; // the axis cannot move more than 500 of 4096 counts per frame 轴每帧最多走 4096 中的 500
    ASSERT_EQ(ErrorCode::Ok, assembler.push(100U, true).status);
    const auto origin = assembler.unwrapped();

    const auto bad_crc = assembler.push(110U, false);
    EXPECT_EQ(ErrorCode::State, bad_crc.status);
    EXPECT_EQ(origin, assembler.unwrapped());

    const auto out_of_range = assembler.push(4096U, true);
    EXPECT_EQ(ErrorCode::Param, out_of_range.status);
    EXPECT_EQ(origin, assembler.unwrapped());

    // And the next good frame is measured against the last trusted one, not against the garbage.
    // 而下一帧好数据是对着最后一个可信帧量的，不是对着垃圾。
    const auto good = assembler.push(104U, true);
    ASSERT_EQ(ErrorCode::Ok, good.status);
    EXPECT_EQ(4, good.delta);

    // The first frame ever being bad leaves the assembler un-seeded rather than seeded at garbage.
    // 历史第一帧就是坏的，会让拼接器保持未定原点，而不是把垃圾当原点。
    TurnAssembler fresh{kSt, 500U};
    EXPECT_EQ(ErrorCode::State, fresh.push(7U, false).status);
    EXPECT_FALSE(fresh.started());
    EXPECT_EQ(0, fresh.unwrapped());
}

// The layouts and buffers that cannot describe a frame. Each is `Param` rather than a decoder that reads past its buffer or shifts
// by 32 bits, which is undefined behaviour rather than a wrong answer.
// 那些描述不了一帧的布局与缓冲区。每一种都是 `Param`，而不是一个越界读、或移 32 位的解码器——后者是未定义行为，不是一个错答案。
TEST(EncoderStream, RefusesLayoutsAndBuffersThatCannotDescribeAFrame) {
    const std::vector<std::uint8_t> bits(8, 0xFFU);

    EncoderLayout no_single_turn{};
    no_single_turn.single_turn_bits = 0;
    EXPECT_EQ(ErrorCode::Param, decode_biss(bits.data(), no_single_turn.frame_bits(), no_single_turn).status);

    EncoderLayout too_wide{};
    too_wide.single_turn_bits = 32; // `1U << 32` would be undefined 移 32 位是未定义行为
    EXPECT_EQ(ErrorCode::Param, decode_biss(bits.data(), too_wide.frame_bits(), too_wide).status);

    EncoderLayout too_many{};
    too_many.single_turn_bits = 24;
    too_many.multi_turn_bits = 12; // 36 data bits do not fit a 32-bit count 36 位数据装不进 32 位计数
    EXPECT_EQ(ErrorCode::Param, decode_biss(bits.data(), too_many.frame_bits(), too_many).status);

    EncoderLayout negative_mt{};
    negative_mt.single_turn_bits = 12;
    negative_mt.multi_turn_bits = -1;
    EXPECT_EQ(ErrorCode::Param, decode_biss(bits.data(), negative_mt.frame_bits(), negative_mt).status);

    EXPECT_EQ(ErrorCode::Param, decode_biss(nullptr, 24, layout()).status);

    // The assembler refuses a layout it cannot wrap, and reports it without touching the count.
    // 拼接器拒绝一个它无法处理回绕的布局，并报告出来而不动计数。
    TurnAssembler broken{0, 1U};
    const auto step = broken.push(1U, true);
    EXPECT_EQ(ErrorCode::Param, step.status);
    EXPECT_EQ(0, step.unwrapped);

    // A single-turn layout (no multi-turn bits) decodes with `turns == 0`, so a caller cannot mistake a stale field for data.
    // 单圈布局（没有多圈位）解出的 `turns` 是 0，于是调用方不会把残留字段当成数据。
    EncoderLayout single{};
    single.single_turn_bits = 17;
    const std::vector<std::uint8_t> one_turn =
        pack(0U, 0U); // packed with the 24-bit layout, used only for the length check
    EXPECT_EQ(ErrorCode::Param, decode_biss(one_turn.data(), 24, single).status) << "17 + 2 + 6 = 25 bits, not 24";
}

// The wire, checked against a writer nobody in this repository wrote.
//
// Every case above pins the MSB-first convention **against itself**: `pack()` shifts bits out with its own lambda and
// `decode_biss()` reads them back through `bit_at()`, so a consistent-but-different convention -- LSB-first, say --
// would keep all of them green, because both sides would have moved together. That is not a hypothetical risk for a
// bit-oriented protocol: the convention is the interface, and it is the one thing a caller on the other end of a cable
// cannot renegotiate.
//
// So this case packs the same frame with `etl::bit_stream_writer` -- a fielded implementation by people who never saw
// this file -- and demands the **bytes** are identical, not merely that both round-trip. ETL is a witness here, not a
// dependency: it lives in the test tree only, because measured, `etl/bit_stream.h` costs a consumer TU 348 headers and
// 391 ms (`.github/project/deps.json`). The CRC bits still come from our `crc6`: what this case witnesses is the bit
// *layout*, and pretending otherwise would make it a second test of the CRC instead of a first test of the stream.
//
// 线上格式，用一个本仓库没人写过的写端核对。
//
// 上面所有用例都是 **对着自己** 钉 MSB-first 约定的：`pack()` 用自己的 lambda 移位写出、`decode_biss()` 经 `bit_at()`
// 读回，于是一个"自洽但与世界不同"的约定——比如 LSB-first——会让它们全部保持绿色，因为两侧一起挪了。对面向位的协议
// 来说这不是假想风险：约定就是接口，而它恰恰是电缆另一端的调用方 **无法重新协商** 的那件东西。
//
// 所以这条用例用 `etl::bit_stream_writer`——一个没见过本文件的人写的、实装过的实现——拼同一帧，并要求 **字节** 相同，
// 而不只是双方各自往返一次。ETL 在这里是证人不是依赖：它只活在测试树里，因为实测 `etl/bit_stream.h` 会让一个消费者
// TU 多拉 348 个头、多花 391 ms（`.github/project/deps.json`）。CRC 位仍由我们的 `crc6` 给出：这条用例见证的是位的
// *布局*，假装不是，它就变成了第二个 CRC 测试而不是第一个码流测试。
TEST(EncoderStream, TheWireIsTheMsbFirstStreamAForeignWriterProduces) {
    const EncoderLayout shape = layout();
    const auto bytes = static_cast<std::size_t>((shape.frame_bits() + 7) / 8);
    int compared = 0;
    for (const std::uint32_t turns : {0U, 1U, 7U, 15U}) {
        for (const std::uint32_t position : {0U, 1U, 2047U, 4095U}) {
            for (const bool error_bit : {false, true}) {
                const std::vector<std::uint8_t> ours = pack(turns, position, error_bit, true);

                std::vector<char> foreign(bytes, 0);
                etl::bit_stream_writer writer(etl::span<char>(foreign.data(), foreign.size()), etl::endian::big);
                writer.write_unchecked(turns, kMt);
                writer.write_unchecked(position, kSt);
                writer.write_unchecked(error_bit ? 1U : 0U, 1);
                writer.write_unchecked(1U, 1); // the warning bit
                writer.write_unchecked(crc6(ours.data(), shape.data_bits() + 2), EncoderLayout::kCrcBits);

                ASSERT_EQ(0, std::memcmp(ours.data(), foreign.data(), bytes))
                    << "turns " << turns << " position " << position << " error " << error_bit
                    << ": our packing and a foreign MSB-first writer disagree about the wire";

                const auto* foreign_bits = reinterpret_cast<const std::uint8_t*>(foreign.data());
                const EncoderFrame mine = decode_biss(ours.data(), shape.frame_bits(), shape);
                const EncoderFrame theirs = decode_biss(foreign_bits, shape.frame_bits(), shape);
                EXPECT_EQ(ErrorCode::Ok, theirs.status);
                EXPECT_TRUE(theirs.crc_ok);
                EXPECT_EQ(mine.position, theirs.position);
                EXPECT_EQ(mine.turns, theirs.turns);
                EXPECT_EQ(mine.raw_data, theirs.raw_data);
                EXPECT_EQ(error_bit, theirs.error_bit);
                EXPECT_TRUE(theirs.warning);
                ++compared;
            }
        }
    }
    std::printf("\n[encoder] %d frames packed by ETL's bit_stream_writer: byte-identical to ours\n", compared);
}
