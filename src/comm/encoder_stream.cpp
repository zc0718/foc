// Conan::ImportStart
#include <foc/comm/encoder_stream.hpp>
// Conan::ImportEnd

namespace foc {

namespace comm {

namespace {

// `x^6 + x + 1` is `0b1000011`, but a six-bit register holds only the **remainder**, so the `x^6` term is implicit in the
// shift-out and the constant is `0b000011`. Writing `0x43` folds a bit the register cannot hold: it leaks into bit 6, the wire
// truncates it, and the encoder and decoder then disagree exactly when that bit is set -- measured at position 2047, where the
// recomputed value was `0x7B` against `0x3B` on the wire. The case that caught it was the exhaustive one, and it caught it as a
// *decode failure* rather than as a bad CRC, which is why the sum was worth checking by hand.
// `x^6 + x + 1` 是 `0b1000011`，但六位寄存器只装**余数**，`x^6` 那一项隐含在移出位里，所以常数是 `0b000011`。写成 `0x43`
// 会折入一个寄存器装不下的位：它漏进第 6 位、线上被截断，于是只要该位被置上，编码侧与解码侧就不一致——实测出现在位置 2047，
// 重算值 `0x7B` 对线上 `0x3B`。抓到它的是那条穷举用例，而且它表现为一次**解码失败**而不是一次坏 CRC，所以把这个和手工核对一遍是值得的。
constexpr std::uint8_t kCrcPoly = 0x03U;
constexpr int kBitsPerByte = 8;

/** @brief One bit of an MSB-first stream, or 0 past the end (the caller has already checked the length). */
auto bit_at(const std::uint8_t* bits, int index) noexcept -> std::uint8_t {
    const int byte = index / kBitsPerByte;
    const int shift = (kBitsPerByte - 1) - (index % kBitsPerByte);
    return static_cast<std::uint8_t>((bits[byte] >> shift) & 0x01U);
}

} // namespace

auto crc6(const std::uint8_t* bits, int bit_count, int bit_offset) noexcept -> std::uint8_t {
    if ((bits == nullptr) || (bit_count < 0) || (bit_offset < 0)) {
        return 0U;
    }
    // Long division, bit at a time: shift the next message bit in, and whenever the register's top bit is set, fold the
    // polynomial in. Six bits, no table -- a table would be 64 entries of a polynomial nobody can check by eye, and this
    // runs once per frame, not once per microsecond.
    // 逐位长除法：把下一个报文位移进来，寄存器最高位为 1 时折入多项式。六位、不用表——表是 64 个没人能用眼睛核对的多项式项，
    // 而这段每帧只跑一次，不是每微秒一次。
    std::uint8_t reg = 0U;
    for (int n = 0; n < bit_count; ++n) {
        const std::uint8_t incoming = bit_at(bits, bit_offset + n);
        const std::uint8_t top = static_cast<std::uint8_t>((reg >> 5) & 0x01U);
        reg = static_cast<std::uint8_t>(((reg << 1) | incoming) & 0x3FU);
        if (top != 0U) {
            reg = static_cast<std::uint8_t>(reg ^ kCrcPoly);
        }
    }
    return static_cast<std::uint8_t>(reg & 0x3FU);
}

auto decode_biss(const std::uint8_t* bits, int bit_count, const EncoderLayout& layout) noexcept -> EncoderFrame {
    EncoderFrame out{ErrorCode::Param, 0U, 0U, 0U, false, false, false};
    // `single_turn_bits <= 31` rather than `<= 32`: `1U << 32` is undefined behaviour, and a resolution that wide is not
    // something a 32-bit count can hold anyway. The other side of the same limit is `data_bits() <= 32`.
    // 用 `single_turn_bits <= 31` 而不是 `<= 32`：`1U << 32` 是未定义行为，而那么宽的分辨率也不是 32 位计数装得下的。
    // 同一条限界的另一侧是 `data_bits() <= 32`。
    if ((bits == nullptr) || (layout.single_turn_bits <= 0) || (layout.single_turn_bits > 31) ||
        (layout.multi_turn_bits < 0) || (layout.data_bits() > 32)) {
        return out;
    }
    if (bit_count != layout.frame_bits()) {
        // A stream of the wrong length is a stream that is not this encoder's frame. Refusing is the only answer that
        // cannot produce a plausible angle from a misaligned one.
        // 长度不对的码流就不是这台编码器的帧。拒绝是唯一不会从错位码流里产生一个"看起来合理的角度"的答案。
        return out;
    }

    const int data = layout.data_bits();
    std::uint32_t payload = 0U;
    for (int n = 0; n < data; ++n) {
        payload = (payload << 1) | static_cast<std::uint32_t>(bit_at(bits, n));
    }
    const bool error_bit = bit_at(bits, data) != 0U;
    const bool warning = bit_at(bits, data + 1) != 0U;
    std::uint8_t transmitted = 0U;
    for (int n = 0; n < EncoderLayout::kCrcBits; ++n) {
        transmitted = static_cast<std::uint8_t>((transmitted << 1) | bit_at(bits, data + 2 + n));
    }
    const std::uint8_t computed = crc6(bits, data + 2);

    out.raw_data = payload;
    out.crc_ok = (transmitted == computed);
    const std::uint32_t turn_mask =
        (layout.multi_turn_bits > 0) ? ((1U << static_cast<unsigned>(layout.multi_turn_bits)) - 1U) : 0U;
    out.turns = (payload >> static_cast<unsigned>(layout.single_turn_bits)) & turn_mask;
    out.position = payload & ((1U << static_cast<unsigned>(layout.single_turn_bits)) - 1U);
    out.error_bit = error_bit;
    out.warning = warning;
    // The frame arrived; whether it may be acted on is a separate sentence, and it says so with `State` rather than with a
    // zero that a caller could mistake for "at the origin".
    // 帧到了；能不能据此动作是另一句话，而它用 `State` 说出来，而不是用一个会被调用方误当成"在原点"的零。
    out.status = out.crc_ok ? ErrorCode::Ok : ErrorCode::State;
    return out;
}

auto TurnAssembler::push(std::uint32_t position, bool crc_ok) noexcept -> Step {
    Step step{ErrorCode::Param, false, unwrapped_, 0};
    if ((single_turn_bits_ <= 0) || (single_turn_bits_ > 31)) {
        return step;
    }
    const std::uint32_t span = 1U << static_cast<unsigned>(single_turn_bits_);
    // A declared step above half a turn cannot be used to decide anything: both readings of every difference stay plausible, so
    // there is no answer to give and the layout is refused rather than answered hopefully.
    // 声称的步长超过半圈就无法用来判断任何事：每个差值的两种读法都仍然成立，于是无答案可给，布局被拒绝而不是满怀希望地作答。
    if ((max_step_counts_ == 0U) || (max_step_counts_ > (span / 2U))) {
        return step;
    }
    if ((position >= span) || (!crc_ok)) {
        // A frame that failed its CRC, or a count that cannot exist in this resolution, is not a reading: it neither moves
        // the unwrapped count nor re-seeds it, so a run of bad frames leaves the last good answer standing.
        // CRC 不过的帧、或这个分辨率下不可能存在的计数，都不是读数：它既不动不回绕计数也不重定它，于是一连串坏帧会让最后
        // 一个正确答案留在原处。
        step.status = crc_ok ? ErrorCode::Param : ErrorCode::State;
        return step;
    }
    if (!started_) {
        started_ = true;
        last_ = position;
        step.status = ErrorCode::Ok;
        return step;
    }

    std::int64_t difference = static_cast<std::int64_t>(position) - static_cast<std::int64_t>(last_);
    const std::int64_t half = static_cast<std::int64_t>(span / 2U);
    // A raw difference cannot tell a forward wrap from a backward jump on its own -- `4090 -> 5` is +11 counts if the axis moved
    // forward and -4085 if it moved backward, and both are the same two numbers. The only thing that decides is how far the axis
    // **can** move between two frames, which is the caller's knowledge; that is why `max_step` is a construction parameter and
    // why it is capped at half a turn (above that the two readings are both plausible and the question is undecidable).
    // 原始差值本身分不清"正向回绕"与"反向跳变"——`4090 -> 5` 在正向时是 +11 个计数、反向时是 −4085，而两者是同样的两个数。唯一能
    // 决定的是轴在两帧之间**能**走多远，那是调用方的知识；这就是 `max_step` 是构造参数的原因，也是它被限制在半圈以内的原因
    // （超过半圈，两种读法都成立，问题本身不可判定）。
    if (difference > half) {
        difference -= static_cast<std::int64_t>(span);
    } else if (difference < -half) {
        difference += static_cast<std::int64_t>(span);
    }
    const auto magnitude = (difference < 0) ? -difference : difference;
    if (magnitude > static_cast<std::int64_t>(max_step_counts_)) {
        // More than the application says is possible: a frame was lost, and the travel across it is unknowable. Report it and
        // hold the count; re-seeding here would silently accept a gap the caller must know about.
        // 超过应用声称可能的步长：丢了一帧，而跨过它的行程不可知。如实报告并保持计数；在这里重定原点等于悄悄接受一个调用方
        // 必须知道的缺口。
        step.status = ErrorCode::State;
        step.jumped = true;
        return step;
    }
    unwrapped_ += difference;
    last_ = position;
    step.status = ErrorCode::Ok;
    step.unwrapped = unwrapped_;
    step.delta = difference;
    return step;
}

auto TurnAssembler::reset() noexcept -> void {
    unwrapped_ = 0;
    last_ = 0U;
    started_ = false;
}

} // namespace comm

} // namespace foc
