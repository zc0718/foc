// Conan::ImportStart
#pragma once
#include <cstddef>
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_comm
 * @file encoder_stream.hpp
 *
 * @brief [en] The bit-level contract for reading an absolute encoder off a serial stream: its layout, one frame, and the assembly across turns.
 * @brief [zh] 从串行码流读绝对编码器的位级契约：布局、单帧，以及跨圈拼装。
 * @since 0.1.0
 */

namespace foc {

namespace comm {

/**
 * @brief [en] How many bits an absolute encoder puts where, on a bit-oriented link.
 *
 *   **Why the layout is a parameter and not a constant.** `multi_turn_bits` is zero for a single-turn encoder and
 *   positive for one that reports revolutions, and the two are electrically identical -- which is why a decoder that
 *   assumed one of them would answer a plausible angle for the other's frames. The frame length is derived from these
 *   numbers, so a stream that does not have exactly this many bits is *refused*, not re-aligned: a decoder that shifts
 *   until something checks out will eventually shift a valid frame into a wrong value.
 *
 * @brief [zh] 在面向位的链路上，绝对编码器把各字段放在哪几位。
 *
 *   **为什么布局是参数而不是常数。** 多圈位对单圈编码器是零、对报转数的编码器是正数，而两者在电气上完全一样——这正是 "
 *   假定其中一种"的解码器会对另一种的帧给出一个看起来合理的角度的原因。帧长由这些数推出来，于是位数不是恰好这么多的码流
 *   会被 **拒绝** 而不是重新对齐：一个"挪到某个东西对得上为止"的解码器，最终会把一帧有效数据挪成一个错的值。
 * @since 0.1.0
 * @exporter
 */
struct EncoderLayout final {
    int single_turn_bits{/**
         * @brief [en] resolution within a revolution, bits; > 0 and <= 32 total.
         * @brief [zh] 转内分辨率，位；> 0 且总数 <= 32。
         */
                         0};
    /**
     * @brief [en] revolution count, bits; 0 = single-turn.
     * @brief [zh] 转数位数；0 = 单圈。
     */
    int multi_turn_bits{0};

    /**
     * @brief [en] Data bits before the status bits.
     * @brief [zh] 状态位之前的数据位数。
     */
    [[nodiscard]] auto data_bits() const noexcept -> int { return single_turn_bits + multi_turn_bits; }

    /**
     * @brief [en] A BiSS-C style frame: data, an error bit, a warning bit, a 6-bit CRC.
     * @brief [zh] BiSS-C 风格的一帧：数据、错误位、警告位、6 位 CRC。
     */
    [[nodiscard]] auto frame_bits() const noexcept -> int { return data_bits() + 2 + kCrcBits; }

    static constexpr int kCrcBits =
        /**
         * @brief [en] the CRC is six bits wide, over the data and status bits.
         * @brief [zh] CRC 六位，覆盖数据位与状态位。
         */
        6;
};

/**
 * @brief [en] One decoded frame, including whether it is a reading at all.
 *
 *   `status == State` means the frame *arrived* and is not trustworthy (CRC, encoder error bit, warning). The position
 *   and turns are still filled in, because a caller that logs a bad frame needs to know what it said -- but nothing in
 *   this library will act on them: `TurnAssembler` refuses a frame whose `crc_ok` is false and does not need to look at
 *   the status to do it.
 *
 * @brief [zh] 一帧解码结果，包括它到底算不算一次读数。
 *
 *   `status == State` 的意思是这一帧 **到了** 但不可信（CRC、编码器错误位、警告）。位置与转数仍然填好，因为记录坏帧的调
 *   用方需要知道它说了什么——但本库里没有任何东西会拿它们去动作： `TurnAssembler` 拒绝 `crc_ok` 为假的帧，而且它不需要看
 *   `status` 才这么做。
 * @since 0.1.0
 * @exporter
 */
struct EncoderFrame final {
    ErrorCode
        /**
         * @brief [en] `Ok`, `Param` (the frame cannot be read) or `State` (read, not trustworthy).
         * @brief [zh] `Ok`、 `Param`（读不出）或 `State`（读出了但不可信）。
         */
        status;
    /**
     * @brief [en] raw single-turn counts.
     * @brief [zh] 原始转内计数。
     */
    std::uint32_t position;
    std::uint32_t
        /**
         * @brief [en] raw revolution count; 0 when the layout has no multi-turn bits.
         * @brief [zh] 原始转数；布局没有多圈位时为 0。
         */
        turns;
    /**
     * @brief [en] the data field as transmitted, host order.
     * @brief [zh] 线上传输的数据域（主机序）。
     */
    std::uint32_t raw_data;
    /**
     * @brief [en] the encoder's own error bit.
     * @brief [zh] 编码器自己的错误位。
     */
    bool error_bit;
    /**
     * @brief [en] the encoder's own warning bit.
     * @brief [zh] 编码器自己的警告位。
     */
    bool warning;
    bool
        /**
         * @brief [en] whether the six CRC bits match the data and status bits.
         * @brief [zh] 六位 CRC 与数据位、状态位是否相符。
         */
        crc_ok;
};

/**
 * @brief [en] Six-bit CRC over a bit stream, MSB first: `x^6 + x + 1`, initial value 0.
 * @brief [zh] 位流上的六位 CRC，高位在前： `x^6 + x + 1`，初值 0。
 * @param bits [en] message bits, MSB first inside each byte.
 * @param bits [zh] 报文位，每个字节内高位在前。
 * @param bit_count [en] how many bits of `bits` belong to the message.
 * @param bit_count [zh] `bits` 中属于报文的位数。
 * @param bit_offset [en] first bit to use; lets a caller checksum a slice of a longer buffer.
 * @param bit_offset [zh] 起始位；让调用方对长缓冲区中的一段求和。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] auto crc6(const std::uint8_t* bits, int bit_count, int bit_offset = 0) noexcept -> std::uint8_t;

/**
 * @brief [en] Decode one BiSS-C style frame from a bit-oriented stream.
 *
 *   The stream is MSB first: `turns | position | error | warning | crc[6]`. The caller owns the *framing* (clocking the
 *   bits out of the shift register, including the protocol's start/ACK and stop bits); this function owns the
 *   interpretation and the refusals: a length that is not the layout's, a CRC that does not match, and a layout that
 *   cannot describe any encoder.
 *
 *   **A one-bit shift is not the CRC's job, and that is measured rather than assumed.** The CRC detects every
 *   single-bit error (asserted exhaustively, one flip at a time) and every burst up to six bits -- but a shift is
 *   neither: it is a different message, and for the frame in the test it passes the CRC. So a stream that is out of
 *   alignment can only be caught by *framing*: the length check here, and the protocol's start and stop bits, which the
 *   caller owns. A decoder that "shifts until something checks out" would therefore eventually shift a valid frame into
 *   a wrong value; this one does not shift at all.
 *
 * @brief [zh] 从面向位的码流中解出一帧 BiSS-C 风格的数据。
 *
 *   码流高位在前： `turns | position | error | warning | crc[6]`。调用方拥有 **成帧** （把位从移位寄存器里打出来，包括
 *   协议的起始/ACK 与停止位），本函数拥有解释与拒绝：长度不等于布局给定的、CRC 不符、布局描述不了任何编码器。
 *
 *   **错开一位不是 CRC 的活，而这是量出来的、不是假定的。** CRC 检出每一个单比特错误（逐位翻转穷举断言）以及长度不超过
 *   六位的突发——但错位两者都不是：它是另一条报文，而在用例那一帧上它 **通过了** CRC。所以错位的码流只能由 **成帧** 抓
 *   住：这里的长度检查，以及协议自己的起始位与停止位（属于调用方）。于是"挪到某个东西对得上为止"的解码器，最终会把一帧
 *   有效数据挪成一个错的值；本解码器一格都不挪。
 *
 * @param bits [en] frame bits, MSB first inside each byte; null is `Param`.
 * @param bits [zh] 帧位，每字节内高位在前；空指针为 `Param`。
 * @param bit_count [en] how many bits are available; must equal `layout.frame_bits()`.
 * @param bit_count [zh] 可用位数；必须等于 `layout.frame_bits()`。
 * @param layout [en] the field widths.
 * @param layout [zh] 字段宽度。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] auto decode_biss(const std::uint8_t* bits, int bit_count, const EncoderLayout& layout) noexcept
    -> EncoderFrame;

/**
 * @brief [en] Turn a sequence of single-turn counts into one unwrapped count, and refuse to guess across a gap.
 *
 *   **A bit stream has no sequence number, so "a frame was lost" can only be said one way**: the next reading is
 *   further from the last one than the axis can travel between two frames. That distance is the *caller's* knowledge --
 *   the readout rate and the axis's top speed -- and it is a construction parameter for that reason: a raw count
 *   difference cannot tell a forward wrap from a backward jump on its own (`4090 -> 5` is +11 counts forward and -4085
 *   backward, and both are the same two numbers). It is capped at half a turn, because above that both readings are
 *   plausible and the question is undecidable -- refused as `Param` rather than answered hopefully.
 *
 *   On a reading further than that this class answers `State` and leaves the unwrapped count where it was, because the
 *   travel in between is unknown and inventing it (by advancing one turn, or by the shortest arc) is how a position
 *   controller ends up commanding a move that never happened. A **wrap** is not a gap: `4090 -> 5` on a 4096-count turn
 *   is +11 counts, and that is arithmetic, not inference.
 *
 * @brief [zh] 把一串转内计数拼成一个不回绕的计数，并在缺口处拒绝猜测。
 *
 *   **位流没有序号，所以"丢了一帧"只有一种说法** ：下一次读数与上一次之间，比轴在两帧之间能走的距离更远。那个距离是
 *   **调用方的知识** ——读数率与轴的最高速——它因此是一个构造参数：单凭两个计数分不清正向回绕与反向跳变（ `4090 -> 5` 正
 *   向是 +11 个计数、反向是 −4085，而两者是同样的两个数）。它被限制在半圈以内，因为超过半圈两种读法都成立、问题本身不可
 *   判定 —— 于是报 `Param` 而不是满怀希望地给一个答案。
 *
 *   在超过该距离的读数上，本类答 `State` 并把不回绕计数留在原处，因为中间的行程是未知的，而编造它（前进一圈、或走最短
 *   弧）正是位置控制器最终下达一个从未发生过的运动的方式。 **回绕** 不是缺口：4096 计数的转上 `4090 -> 5` 就是 +11 个计
 *   数，那是算术，不是推断。
 * @since 0.1.0
 * @exporter
 */
class TurnAssembler final {
  public:
    /**
     * @brief [en] What one good frame did to the unwrapped count.
     * @brief [zh] 一帧好数据对不回绕计数做了什么。
     */
    struct Step final {
        ErrorCode
            /**
             * @brief [en] `Ok`, `Param` (bad layout or count), or `State` (a gap was reported).
             * @brief [zh] `Ok`、 `Param`（布局或计数不合法）或 `State`（报了缺口）。
             */
            status;
        bool
            /**
             * @brief [en] whether the reading was further than `max_step_counts` from the last one.
             * @brief [zh] 该读数与上一次相距超过 `max_step_counts`。
             */
            jumped;
        std::int64_t
            /**
             * @brief [en] unwrapped count after this step; unchanged when a gap is reported.
             * @brief [zh] 本步之后的不回绕计数；报缺口时不变。
             */
            unwrapped;
        std::int64_t
            /**
             * @brief [en] how far this reading moved it, signed counts; 0 on a gap.
             * @brief [zh] 本读数让它动了多少，带符号计数；缺口时为 0。
             */
            delta;
    };

    /**
     * @brief [en] An assembler for a resolution and a declared maximum travel per frame.
     * @brief [zh] 给定分辨率与声称的每帧最大行程的拼接器。
     * @param single_turn_bits [en] counts per revolution, as bits; `1..31`.
     * @param single_turn_bits [zh] 每转计数，位数； `1..31`。
     * @param max_step_counts [en] the most the axis can move between two frames, counts; `1..span/2`.
     * @param max_step_counts [zh] 轴在两帧之间最多能走多少计数； `1..span/2`。
     * @since 0.1.0
     */
    TurnAssembler(int single_turn_bits, std::uint32_t max_step_counts) noexcept
        : single_turn_bits_(single_turn_bits), max_step_counts_(max_step_counts) {}

    /**
     * @brief [en] Forget the last reading, so the next one re-seeds the origin.
     * @brief [zh] 忘掉上一次读数，于是下一次重新定原点。
     */
    void reset() noexcept;

    /**
     * @brief [en] Feed one frame.
     * @brief [zh] 喂入一帧。
     * @param position [en] raw single-turn counts from `decode_biss`.
     * @param position [zh] 来自 `decode_biss` 的原始转内计数。
     * @param crc_ok [en] whether that frame's CRC matched; a frame that failed is not fed at all.
     * @param crc_ok [zh] 该帧 CRC 是否通过；不通过的帧根本不参与拼接。
     * @since 0.1.0
     */
    [[nodiscard]] auto push(std::uint32_t position, bool crc_ok) noexcept -> Step;

    /**
     * @brief [en] Unwrapped count so far, or 0 before the first good frame.
     * @brief [zh] 目前的不回绕计数；第一帧好数据之前是 0。
     */
    [[nodiscard]] auto unwrapped() const noexcept -> std::int64_t { return unwrapped_; }

    /**
     * @brief [en] Whether an origin has been seeded.
     * @brief [zh] 是否已经定过原点。
     */
    [[nodiscard]] auto started() const noexcept -> bool { return started_; }

  private:
    int single_turn_bits_{0};
    std::uint32_t max_step_counts_{0};
    std::int64_t unwrapped_{0};
    std::uint32_t last_{0};
    bool started_{false};
};

} // namespace comm

} // namespace foc
