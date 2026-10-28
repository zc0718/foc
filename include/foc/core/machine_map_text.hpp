// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include <foc/core/machine_map.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] Why a machine document could not be read.
 * @brief [zh] 机器文档读不进来的原因。
 * @since 0.1.2
 */
enum class TextStatus : std::uint8_t {
    Ok = 0,
    Truncated, ///< [en] the output buffer was too small. [zh] 输出缓冲不够。
    BadFormat, ///< [en] a line is not `key=value`, or a value is not a number. [zh] 某行不是 `key=value`，或值不是数字。
    Unsupported,  ///< [en] the `format` version is not one this build knows. [zh] `format` 版本不认识。
    OutOfRange,   ///< [en] a value is not finite, or a length is not positive. [zh] 数值非有限，或长度非正。
    TooManyRows,  ///< [en] more `row=` lines than the map holds. [zh] `row=` 行数超过表的容量。
    NotSorted,    ///< [en] rows are not in strictly increasing current. [zh] 行的电流不是严格递增。
    NoParameters, ///< [en] `rs` was never given. [zh] 没有给 `rs`。
};


/**
 * @brief [en] Where a document went wrong: one-based line, and why.
 * @brief [zh] 文档错在哪里：从 1 开始的行号，以及原因。
 * @since 0.1.2
 */
struct TextError final {
    std::size_t line{0U};
    TextStatus status{TextStatus::Ok};
};


/**
 * @brief [en] A machine description as it travels: the parameters, the current map, and how many keys were ignored.
 * @brief [zh] 会旅行的机器描述：参数、电流表，以及被忽略的键的数量。
 * @details [en] `ignored` is not an error and not a warning: a reader must accept keys it does not know, or the format
 *               can never grow. Counting them is what lets a caller notice that the file says more than this build
 *               understood -- without the library having to know what any of it means.
 * @details [zh] `ignored` 既不是错误也不是警告：读方必须接受自己不认识的键，否则格式永远无法演进。计数的作用是让调用方
 *               察觉"文件里说的比这个版本理解的多"——而不需要库知道那些内容是什么意思。
 * @since 0.1.2
 * @exporter
 */
struct MachineDocument final {
    MachineParams params{};
    MachineMap map{};
    std::uint32_t ignored{0U};
};


namespace detail {

/**
 * @brief [en] Append with bounds checking; false when it would not fit.
 * @brief [zh] 带边界检查的追加；放不下时返回 false。
 */
inline auto append(char* out, std::size_t capacity, std::size_t& used, const char* format, double value) noexcept
    -> bool {
    if ((out == nullptr) || (used >= capacity)) {
        return false;
    }
    const int written = std::snprintf(out + used, capacity - used, format, value);
    if (written < 0) {
        return false;
    }
    if (static_cast<std::size_t>(written) >= (capacity - used)) {
        return false;
    }
    used += static_cast<std::size_t>(written);
    return true;
}

/**
 * @brief [en] Skip spaces and tabs.
 * @brief [zh] 跳过空格与制表符。
 */
inline auto skip_spaces(const char* text, std::size_t length, std::size_t index) noexcept -> std::size_t {
    while ((index < length) && ((text[index] == ' ') || (text[index] == '\t') || (text[index] == '\r'))) {
        ++index;
    }
    return index;
}

/**
 * @brief [en] Read one float and advance; false when the text is not a number.
 * @brief [zh] 读一个浮点并前进；文本不是数字时返回 false。
 */
inline auto read_float(const char* text, std::size_t length, std::size_t& index, float& value) noexcept -> bool {
    char* end = nullptr;
    const char* begin = text + index;
    const float parsed = std::strtof(begin, &end);
    if ((end == begin) || (end > (text + length))) {
        return false;
    }
    index = static_cast<std::size_t>(end - text);
    value = parsed;
    return true;
}

} // namespace detail


/**
 * @brief [en] Write a document as text: one `key=value` per line, rows as `row=i,ld,lq,psi`.
 * @brief [zh] 把文档写成文本：每行一个 `key=value`，行写成 `row=i,ld,lq,psi`。
 * @param document [en] what to write.
 * @param document [zh] 要写的内容。
 * @param out [en] destination buffer; `nullptr` is allowed and then only the length is computed.
 * @param out [zh] 目标缓冲；允许为 `nullptr`，此时只计算长度。
 * @param capacity [en] bytes available in `out`.
 * @param capacity [zh] `out` 可用的字节数。
 * @return [en] characters written excluding the terminator, or 0 when it did not fit.
 * @return [zh] 不含终止符的写入字符数；放不下时返回 0。
 * @note [en] `%.9g` because that is the shortest decimal form that round-trips a `float`: a shorter one loses bits on
 *            the way back and makes a round-trip test a lie.
 * @note [zh] 用 `%.9g`，因为那是能让 `float` 往返的最短十进制形式；更短的形式往回读时会丢位，让往返测试变成谎话。
 * @since 0.1.2
 */
inline auto write_document(const MachineDocument& document, char* out, std::size_t capacity) noexcept -> std::size_t {
    std::size_t used = 0U;
    // No `%d` here: passing a double to `%d` is undefined behaviour, and it wrote `format=0` while
    // looking correct. A literal is both simpler and impossible to get wrong.
    // 这里不用 `%d`：把 double 传给 `%d` 是未定义行为，它写出的 `format=0` 看起来还是对的。字面量更简单，
    // 也不可能写错。
    const bool ok = detail::append(out, capacity, used, "format=1\n", 0.0) &&
                    detail::append(out, capacity, used, "rs=%.9g\n", static_cast<double>(document.params.rs)) && [&]() {
                        for (std::size_t i = 0U; i < document.map.count; ++i) {
                            const MachineMapPoint& row = document.map.rows[i];
                            if (!detail::append(out, capacity, used, "row=%.9g,", static_cast<double>(row.i_a)) ||
                                !detail::append(out, capacity, used, "%.9g,", static_cast<double>(row.ld_h)) ||
                                !detail::append(out, capacity, used, "%.9g,", static_cast<double>(row.lq_h)) ||
                                !detail::append(out, capacity, used, "%.9g\n", static_cast<double>(row.psi_f_wb))) {
                                return false;
                            }
                        }
                        return true;
                    }();
    return ok ? used : 0U;
}


/**
 * @brief [en] Read a document. All or nothing: on failure `document` is not touched.
 * @brief [zh] 读一个文档。全成功或全不动：失败时 `document` 一个字节都不改。
 * @param text [en] the bytes to parse; not required to be NUL-terminated.
 * @param text [zh] 要解析的字节；不要求以 NUL 结尾。
 * @param length [en] how many bytes to read.
 * @param length [zh] 读多少字节。
 * @param document [en] receiver; written only on success.
 * @param document [zh] 接收者；只在成功时写入。
 * @param error [en] optional; line and reason on failure.
 * @param error [zh] 可选；失败时给出行号与原因。
 * @return [en] true when the whole document was understood.
 * @return [zh] 整个文档被理解时为 true。
 * @note [en] Unknown keys are counted and ignored on purpose: a reader that rejects what it does not know can never be
 *            given a newer file, and a format that cannot grow is a format that gets bypassed.
 * @note [zh] 未知键**有意**计数并忽略：拒绝未知内容的读方永远收不到更新的文件，而不能演进的格式最终会被绕开。
 * @since 0.1.2
 */
inline auto read_document(const char* text, std::size_t length, MachineDocument* document, TextError* error) noexcept
    -> bool {
    MachineDocument local{};
    bool have_rs = false;
    std::size_t line_number = 0U;
    std::size_t index = 0U;

    auto fail = [&](TextStatus status, std::size_t line) {
        if (error != nullptr) {
            error->line = line;
            error->status = status;
        }
        return false;
    };

    while (index < length) {
        std::size_t end = index;
        while ((end < length) && (text[end] != '\n')) {
            ++end;
        }
        ++line_number;

        std::size_t cursor = detail::skip_spaces(text, end, index);
        if ((cursor < end) && (text[cursor] != '#')) {
            std::size_t key_end = cursor;
            while ((key_end < end) && (text[key_end] != '=')) {
                ++key_end;
            }
            if (key_end >= end) {
                return fail(TextStatus::BadFormat, line_number);
            }
            const std::size_t key_length = key_end - cursor;
            std::size_t value = detail::skip_spaces(text, end, key_end + 1U);

            const auto key_is = [&](const char* name, std::size_t name_length) {
                return (key_length == name_length) && (std::memcmp(text + cursor, name, name_length) == 0);
            };

            if (key_is("format", 6U)) {
                float version = 0.0F;
                if (!detail::read_float(text, end, value, version)) {
                    return fail(TextStatus::BadFormat, line_number);
                }
                if (version != 1.0F) {
                    return fail(TextStatus::Unsupported, line_number);
                }
            } else if (key_is("rs", 2U)) {
                float rs = 0.0F;
                if (!detail::read_float(text, end, value, rs)) {
                    return fail(TextStatus::BadFormat, line_number);
                }
                if (!std::isfinite(rs) || (rs < 0.0F)) {
                    return fail(TextStatus::OutOfRange, line_number);
                }
                local.params.rs = rs;
                have_rs = true;
            } else if (key_is("row", 3U)) {
                if (local.map.count >= MachineMap::kCapacity) {
                    return fail(TextStatus::TooManyRows, line_number);
                }
                float fields[4] = {0.0F, 0.0F, 0.0F, 0.0F};
                for (std::size_t field = 0U; field < 4U; ++field) {
                    value = detail::skip_spaces(text, end, value);
                    if (!detail::read_float(text, end, value, fields[field])) {
                        return fail(TextStatus::BadFormat, line_number);
                    }
                    value = detail::skip_spaces(text, end, value);
                    if ((field < 3U) && ((value >= end) || (text[value] != ','))) {
                        return fail(TextStatus::BadFormat, line_number);
                    }
                    if ((field < 3U) && (text[value] == ',')) {
                        ++value;
                    }
                }
                value = detail::skip_spaces(text, end, value);
                if (value < end) {
                    // `row=0,4e-4,6e-4,0.05,extra` is a typo, not an annotation. Accepting it would let a
                    // fifth column be silently dropped by every reader.
                    // `row=0,4e-4,6e-4,0.05,extra` 是笔误，不是批注。接受它等于让第五列被每个读方静默丢弃。
                    return fail(TextStatus::BadFormat, line_number);
                }
                if (!std::isfinite(fields[0]) || !std::isfinite(fields[1]) || !std::isfinite(fields[2]) ||
                    !std::isfinite(fields[3])) {
                    return fail(TextStatus::OutOfRange, line_number);
                }
                const MachineMapPoint point{fields[0], fields[1], fields[2], fields[3]};
                if ((local.map.count > 0U) && !(point.i_a > local.map.rows[local.map.count - 1U].i_a)) {
                    return fail(TextStatus::NotSorted, line_number);
                }
                local.map.rows[local.map.count] = point;
                ++local.map.count;
            } else {
                local.ignored += 1U;
            }
        }
        index = end + 1U;
    }

    if (!have_rs) {
        return fail(TextStatus::NoParameters, line_number);
    }
    if (!local.map.valid()) {
        return fail((local.map.count == 0U) ? TextStatus::NoParameters : TextStatus::OutOfRange, line_number);
    }

    if (document != nullptr) {
        *document = local;
    }
    if (error != nullptr) {
        error->line = 0U;
        error->status = TextStatus::Ok;
    }
    return true;
}

} // namespace core

} // namespace foc
