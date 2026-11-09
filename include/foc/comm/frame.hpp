// Conan::ImportStart
#pragma once
#include <cstddef>
#include <cstdint>
// Conan::ImportEnd

namespace foc {

namespace comm {

/**
 * @brief [en] Wire layout of one frame on a byte-oriented link.
 * @brief [zh] 面向字节的链路上，一帧的线上布局。
 *
 * @details [en] `A5 | seq | type | len | payload[len] | crc_hi | crc_lo`
 *
 *   Six bytes of overhead plus the payload, and every one of them earns its place. The start byte is what a receiver
 *   resynchronises on after a corrupt frame -- without it, one bad byte shifts every following frame by one position
 *   and the stream never recovers. `len` is carried rather than derived because the receiver has no other way to know
 *   where the CRC begins; a length that disagrees with the buffer is refused rather than clamped, since clamping is how
 *   a decoder over-reads. `seq` is what lets the *receiver* notice a frame it never saw -- a CRC cannot report a frame
 *   that never arrived. The CRC covers `seq..payload` and deliberately not the start byte: a corrupted start byte fails
 *   the first check anyway, and including it would make the CRC of an otherwise identical frame depend on which
 *   resynchronisation produced it.
 *
 * @details [zh] `A5 | seq | type | len | payload[len] | crc_hi | crc_lo`
 *
 *   六个字节的额外开销加负载，每一个都不是白占的。起始字节是接收方在坏帧之后重新同步的依据——没有它，一个坏字节就会把之
 *   后每一帧都错开一个位置，而这条流永远恢复不过来。 `len` 是携带的而不是推出来的，因为接收方没有别的办法知道 CRC 从哪
 *   里开始；与缓冲区不符的长度是 **拒绝** 而不是钳位，钳位正是解码器越界读的方式。 `seq` 是让 **接收方** 发现一帧它从未
 *   见过的帧的依据——CRC 无法报告一帧根本没有到达的帧。CRC 覆盖 `seq..payload` 而故意不含起始字节：起始字节坏了第一道检
 *   查就不过，而把它算进去会让同一帧的 CRC 取决于它是从哪次重新同步开始的。
 * @since 0.1.0
 * @exporter
 */
struct FrameLayout final {
    /**
     * @brief [en] start-of-frame byte.
     * @brief [zh] 帧起始字节。
     */
    static constexpr std::uint8_t kStart = 0xA5U;
    /**
     * @brief [en] start + seq + type + len + crc(2).
     * @brief [zh] 起始 + seq + type + len + CRC(2)。
     */
    static constexpr std::size_t kOverhead = 6U;
    /**
     * @brief [en] what the one-byte length can say.
     * @brief [zh] 一个字节的长度所能表示的上限。
     */
    static constexpr std::size_t kMaxPayload = 255U;
};

/**
 * @brief [en] Why a byte sequence is not a frame.
 * @brief [zh] 一段字节为什么不是一帧。
 * @note [en] Five reasons and no "generic error": a receiver that cannot say *which* check failed cannot decide whether
 *            to resynchronise, to ask for a retransmission, or to count a link loss -- and those are different actions.
 * @note [zh] 五个原因，没有"一般性错误"：说不清 **哪一道** 检查失败的接收方，无法决定是重新同步、请求重传，还是记一次链
 *            路丢失——而那是三种不同的动作。
 * @since 0.1.0
 * @exporter
 */
enum class FrameStatus : std::uint8_t {
    /**
     * @brief [en] a whole, intact frame.
     * @brief [zh] 完整无损的一帧。
     */
    Ok = 0,
    /**
     * @brief [en] fewer bytes than the layout requires.
     * @brief [zh] 字节数少于布局所需。
     */
    TooShort,
    /**
     * @brief [en] the first byte is not the start-of-frame byte.
     * @brief [zh] 首字节不是帧起始字节。
     */
    BadStart,
    /**
     * @brief [en] the length field runs past the buffer given.
     * @brief [zh] 长度字段越过了给定的缓冲区。
     */
    BadLength,
    /**
     * @brief [en] the CRC does not match.
     * @brief [zh] CRC 不匹配。
     */
    BadChecksum,
};

/**
 * @brief [en] One decoded frame: the header fields, and the payload left where the caller put it.
 * @brief [zh] 一帧解码结果：头字段，以及留在调用方原处的负载。
 * @note [en] `payload` points **into the caller's buffer**. Copying it out would need a size the decoder cannot know,
 *            and a decoder that copies is a decoder that can fail to fit; a pointer cannot fail.
 * @note [zh] `payload` **指向调用方的缓冲区** 。把它拷出来需要一个解码器无从得知的容量，而会拷贝的解码器就是可能装不下
 *            的解码器；指针不会失败。
 * @since 0.1.0
 * @exporter
 */
struct Frame final {
    /**
     * @brief [en] sender's frame counter, modulo 256.
     * @brief [zh] 发送方的帧计数，模 256。
     */
    std::uint8_t seq;
    /**
     * @brief [en] message type; the codec does not interpret it.
     * @brief [zh] 报文类型；编解码器不解释它。
     */
    std::uint8_t type;
    /**
     * @brief [en] payload bytes, 0..255.
     * @brief [zh] 负载字节数，0..255。
     */
    std::uint8_t length;
    /**
     * @brief [en] into the caller's buffer, `length` of them.
     * @brief [zh] 指向调用方缓冲区，共 `length` 个。
     */
    const std::uint8_t* payload;
};

/**
 * @brief [en] CRC-16/CCITT-FALSE: init `0xFFFF`, polynomial `0x1021`, no reflection, no final xor.
 * @brief [zh] CRC-16/CCITT-FALSE：初值 `0xFFFF`、多项式 `0x1021`、不反射、末尾不异或。
 * @param data [en] bytes to cover; `nullptr` with a zero length is allowed and yields the init value.
 * @param data [zh] 要覆盖的字节；长度为零时允许为 `nullptr`，返回初值。
 * @param length [en] how many bytes.
 * @param length [zh] 字节数。
 * @return [en] the checksum.
 * @return [zh] 校验值。
 * @note [en] The specific variant matters and is named rather than described: there are at least six things called
 *            "CRC-16 CCITT", they disagree on init, reflection and final xor, and they produce different check values
 *            for the same bytes. The one number that says which variant this is, is `0x29B1` for the ASCII string
 *            `"123456789"` -- the standard's own check value, asserted in the test. Without it "it is a CRC" is not a
 *            claim that transfers to anyone else's implementation.
 * @note [zh] 具体变体很重要，所以是 **点名** 而不是描述：至少六种东西都叫"CRC-16 CCITT"，它们在初值、反射、末尾异或上各
 *            不相同，对同样字节给出不同的校验值。能说明这是哪一种的那个数，是 ASCII 串 `"123456789"` 的 `0x29B1` ——标准
 *            自己的检查值，由用例断言。没有它，"它是个 CRC"这句话不可移植到别人的实现。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] constexpr auto crc16_ccitt(const std::uint8_t* data, std::size_t length) noexcept -> std::uint16_t {
    std::uint16_t crc = 0xFFFFU;
    if (data == nullptr) {
        return crc;
    }
    for (std::size_t i = 0U; i < length; ++i) {
        crc = static_cast<std::uint16_t>(static_cast<unsigned>(crc) ^ (static_cast<unsigned>(data[i]) << 8U));
        for (std::uint8_t bit = 0U; bit < 8U; ++bit) {
            const bool msb = (static_cast<unsigned>(crc) & 0x8000U) != 0U;
            crc = static_cast<std::uint16_t>(static_cast<unsigned>(crc) << 1U);
            if (msb) {
                crc = static_cast<std::uint16_t>(static_cast<unsigned>(crc) ^ 0x1021U);
            }
        }
    }
    return crc;
}

/**
 * @brief [en] Write one frame; returns the bytes written, or 0 when it does not fit.
 * @brief [zh] 写出一帧；返回写入的字节数，放不下时返回 0。
 * @param seq [en] frame counter, modulo 256.
 * @param seq [zh] 帧计数，模 256。
 * @param type [en] message type, passed through uninterpreted.
 * @param type [zh] 报文类型，原样通过。
 * @param payload [en] payload bytes; `nullptr` is allowed only with a zero length.
 * @param payload [zh] 负载字节；只允许在长度为零时为 `nullptr`。
 * @param length [en] payload bytes, at most 255.
 * @param length [zh] 负载字节数，最多 255。
 * @param out [en] destination buffer.
 * @param out [zh] 目标缓冲区。
 * @param capacity [en] bytes available in `out`.
 * @param capacity [zh] `out` 可用的字节数。
 * @return [en] `length + kOverhead`, or 0 when the frame does not fit or the arguments cannot describe one.
 * @return [zh] `length + kOverhead`；放不下或参数描述不了一帧时返回 0。
 * @note [en] Nothing is partially written: the size is checked before the first byte, because a half frame on a link is
 *            indistinguishable from a corrupt one and turns a caller's bug into the receiver's resynchronisation.
 * @note [zh] 不写出半个帧：长度在写第一个字节之前就检查，因为链路上的半个帧与一个坏帧无法区分，会把调用方的缺陷变成接收
 *            方的重新同步。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto encode_frame(std::uint8_t seq, std::uint8_t type, const std::uint8_t* payload,
                                       std::uint8_t length, std::uint8_t* out, std::size_t capacity) noexcept
    -> std::size_t {
    const std::size_t total = FrameLayout::kOverhead + static_cast<std::size_t>(length);
    if ((out == nullptr) || (total > capacity)) {
        return 0U;
    }
    if ((payload == nullptr) && (length != 0U)) {
        return 0U;
    }
    out[0] = FrameLayout::kStart;
    out[1] = seq;
    out[2] = type;
    out[3] = length;
    for (std::uint8_t i = 0U; i < length; ++i) {
        out[4U + i] = payload[i];
    }
    const std::size_t covered = 3U + static_cast<std::size_t>(length);
    const std::uint16_t crc = crc16_ccitt(&out[1], covered);
    out[4U + length] = static_cast<std::uint8_t>(crc >> 8U);
    out[5U + length] = static_cast<std::uint8_t>(crc & 0xFFU);
    return total;
}

/**
 * @brief [en] Read one frame that starts at `in[0]`.
 * @brief [zh] 读取从 `in[0]` 开始的一帧。
 * @param in [en] buffer beginning with the start-of-frame byte.
 * @param in [zh] 以帧起始字节开头的缓冲区。
 * @param length [en] bytes available.
 * @param length [zh] 可用字节数。
 * @param out [en] receiver, written only when the result is `Ok`.
 * @param out [zh] 接收者，仅在结果为 `Ok` 时写入。
 * @return [en] `Ok` and a filled `out`, or the first check that failed.
 * @return [zh] `Ok` 与填好的 `out`，或第一道未通过的检查。
 * @pre [en] `in` begins at a start-of-frame byte; finding that byte in a stream is the caller's resynchronisation, and
 *           `decode_frame` reporting `BadStart` is how the caller learns it guessed wrong.
 * @pre [zh] `in` 从帧起始字节开始；在一段流里找到那个字节是调用方的重新同步，而 `decode_frame` 报 `BadStart` 正是调用方
 *           得知自己找错了的方式。
 * @note [en] `out` is untouched on failure. A decoder that fills the receiver and *then* reports a bad CRC hands the
 *            caller a plausible frame with one bit wrong, which is worse than no frame at all.
 * @note [zh] 失败时 `out` 不被触碰。先填好接收者 **再** 报 CRC 错的解码器，会把一帧"只错了一位、看起来很合理"的帧交给调
 *            用方，那比没有帧更糟。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto decode_frame(const std::uint8_t* in, std::size_t length, Frame& out) noexcept -> FrameStatus {
    if ((in == nullptr) || (length < FrameLayout::kOverhead)) {
        return FrameStatus::TooShort;
    }
    if (in[0] != FrameLayout::kStart) {
        return FrameStatus::BadStart;
    }
    const std::size_t payload_length = static_cast<std::size_t>(in[3]);
    const std::size_t total = FrameLayout::kOverhead + payload_length;
    if (total > length) {
        return FrameStatus::BadLength;
    }
    const std::size_t covered = 3U + payload_length;
    const std::uint16_t want = crc16_ccitt(&in[1], covered);
    const std::uint16_t got = static_cast<std::uint16_t>((static_cast<unsigned>(in[4U + payload_length]) << 8U) |
                                                         static_cast<unsigned>(in[5U + payload_length]));
    if (want != got) {
        return FrameStatus::BadChecksum;
    }
    out.seq = in[1];
    out.type = in[2];
    out.length = static_cast<std::uint8_t>(payload_length);
    out.payload = &in[4];
    return FrameStatus::Ok;
}

/**
 * @brief [en] What the receiver learned about the sender's frame counter.
 * @brief [zh] 接收方从发送方的帧计数里得知了什么。
 * @note [en] All three can be true at once and none of them is an error: a frame is either delivered or not, and a
 *            counter that reports the second tells the application what the CRC never can.
 * @note [zh] 三者可以同时成立，而且都不是错误：一帧要么到了要么没到，而能报告后者的计数器告诉应用层的东西，是 CRC 永远
 *            说不了的。
 * @since 0.1.0
 * @exporter
 */
struct SequenceResult final {
    /**
     * @brief [en] frames the receiver never saw, 0..127.
     * @brief [zh] 接收方从未见到的帧数，0..127。
     */
    std::uint32_t missing;
    /**
     * @brief [en] the same sequence number again.
     * @brief [zh] 同一个序号又出现了一次。
     */
    bool duplicate;
    /**
     * @brief [en] the counter went backwards by more than one.
     * @brief [zh] 计数倒退了一个以上。
     */
    bool out_of_order;
};

/**
 * @brief [en] Track the sender's frame counter, modulo 256, and say what went missing.
 * @brief [zh] 跟踪发送方的帧计数（模 256），并说出缺了什么。
 * @note [en] The half-range split is the honest part of this class. A difference of 128 or more is read as *backwards*,
 *            because a forward gap of 128 frames or more cannot be told apart from a regression -- and of the two, a
 *            regression is the one that means "the peer restarted or something is replaying frames". A silent choice is
 *            made here and it is written down instead of being left to whichever comparison the code happened to use.
 *            `missing` counts the frames between the one expected and the one that arrived, so a single skipped frame
 *            reports 1 and not 0. That off-by-one is the whole usefulness of the number -- a report of "0 missing" that
 *            arrives with a gap in the counter is worse than no report.
 * @note [zh] 半量程这条分界是这个类里诚实的那一部分。差值达到 128 及以上被读作 **倒退** ，因为向前跳了 128 帧以上与一次
 *            倒退无法区分——而两者之中，倒退才是意味着"对端重启了，或者有什么在重放帧"的那一个。这里做了一个默不作声的选
 *            择，并把它写下来，而不是留给代码恰好用到的那次比较。 `missing` 数的是"期望的那一帧"与"到达的那一帧"之间的
 *            帧，所以跳过一帧报 1 而不是 0。这个差一就是整个数字的价值所在——在计数明明有缺口时却报" 没丢"，比不报更糟。
 * @since 0.1.0
 * @exporter
 */
class SequenceChecker final {
  public:
    /**
   * @brief [en] Feed the next received frame's sequence number.
   * @brief [zh] 喂入下一个收到的帧的序号。
   */
    auto observe(std::uint8_t seq) noexcept -> SequenceResult {
        SequenceResult result{};
        if (!primed_) {
            primed_ = true;
            expected_ = static_cast<std::uint8_t>(seq + 1U);
            return result;
        }
        const std::uint8_t ahead = static_cast<std::uint8_t>(seq - expected_);
        if (ahead == 0U) {
            expected_ = static_cast<std::uint8_t>(seq + 1U);
            return result;
        }
        const std::uint8_t behind = static_cast<std::uint8_t>(expected_ - seq);
        if (behind <= 128U) {
            if (behind == 1U) {
                result.duplicate = true;
            } else {
                result.out_of_order = true;
            }
        } else {
            result.missing = static_cast<std::uint32_t>(ahead);
            expected_ = static_cast<std::uint8_t>(seq + 1U);
        }
        // The expectation is not advanced on a duplicate or a regression: the frame
        // that fills the hole may still arrive, and moving the goalpost would
        // convert one loss into a report of "nothing lost".
        // 重复与倒退都不推进期望值：填补那个空洞的帧仍有可能到达，而挪动标杆会把一次丢失变成"什么都没丢"的报告。
        return result;
    }

    /** @brief [en] Forget the history; the next frame is a first frame. [zh]
   * 清空历史；下一帧算第一帧。 */
    void reset() noexcept {
        primed_ = false;
        expected_ = 0U;
    }

  private:
    std::uint8_t expected_{0U};
    bool primed_{false};
};

} // namespace comm

} // namespace foc
