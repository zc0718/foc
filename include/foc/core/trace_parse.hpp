// Conan::ImportStart
#pragma once
#include <cstdint>

#include <foc/core/trace.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file trace_parse.hpp
 *
 * @brief [en] A window of trace samples as the host got it out of the ring, and the samples that window lost.
 * @brief [zh] 主机从环形缓冲取到的一段跟踪样本，以及这段窗口丢掉的样本。
 * @since 0.1.0
 */

namespace foc {

namespace core {

/**
 * @brief [en] A window of trace samples as the host got it out of the ring: the retained samples, and the loss the
 *             recorder declared.
 * @brief [zh] 上位机从环里取出的一段轨迹窗口：保留下来的样本，以及记录器声明的丢失量。
 * @since 0.1.0
 * @exporter
 */
struct TraceSpan final {
    /**
     * @brief [en] oldest first, `size` of them.
     * @brief [zh] 自最旧起，共 `size` 条。
     */
    const TraceSample* samples;
    /**
     * @brief [en] retained samples.
     * @brief [zh] 保留的样本数。
     */
    std::uint32_t size;
    /**
     * @brief [en] from `foc_motor_trace_dropped()`; declared, not inferred.
     * @brief [zh] 来自 `foc_motor_trace_dropped()`；是声明的，不是推出来的。
     */
    std::uint32_t dropped;
};

/**
 * @brief [en] One run of samples that share a motor state and a cadence.
 * @brief [zh] 一段共享同一电机状态与同一节拍的样本。
 * @since 0.1.0
 * @exporter
 */
struct TraceSegment final {
    /**
     * @brief [en] 0-based, from the oldest retained.
     * @brief [zh] 自最旧保留样本起的 0 基下标。
     */
    std::uint32_t first_index;
    /**
     * @brief [en] samples in this segment; 0 = no such segment.
     * @brief [zh] 本段样本数；0 表示没有这一段。
     */
    std::uint32_t count;
    /**
     * @brief [en] `MotorState` shared by all of them.
     * @brief [zh] 它们共有的 `MotorState`。
     */
    std::uint8_t mode;
};

/**
 * @brief [en] What a window says about itself: how much is missing, where, and whether it is the whole run.
 * @brief [zh] 一段窗口对自身的陈述：缺了多少、缺在哪、以及它是否就是整段运行。
 * @note [en] The two losses are separate fields because they are separate facts and call for opposite actions. `lost`
 *            is what the recorder already threw away before the window began -- it is *declared* by the caller and
 *            **cannot** be verified from inside the window, so a parser that "recomputes" it is only repeating the
 *            caller's number back. `missing` is what the window itself shows: places where consecutive timestamps are
 *            more than one period apart, which means the recorder skipped while it was attached. The first says "look
 *            earlier, or attach a bigger ring"; the second says "the recorder itself lost a period", which is a bug or
 *            a clock jump and is nobody's configuration choice. `consistent` is a self-check on this parser and nothing
 *            more: `missing` is counted twice, once by summing the per-pair skips and once from the two endpoint
 *            timestamps, and a disagreement means the scan is wrong (a rounding or wrap-handling defect). It cannot
 *            detect a sample that is genuinely absent -- see above.
 * @note [zh] 两种丢失是两个字段，因为它们是两件不同的事，且需要相反的动作。 `lost` 是记录器在窗口开始 **之前** 就已经扔
 *            掉的东西——由调用方 **声明** ，在窗口内部 **无法** 验证，因此一个"重算"它的解析器只是在把调用方的数字复述一
 *            遍。 `missing` 是窗口自己显示出来的：相邻时间戳相距超过一个周期的那些地方，意味着记录器在挂载期间跳过了样
 *            本。前者说的是"往前看，或者换更大的环"；后者说的是"记录器自己丢了一拍"，那是缺陷或时钟跳变，不是谁选的配
 *            置。 `consistent` 只是这个解析器的自检： `missing` 被数了两遍，一遍累加逐对跳过数，一遍由首尾两个时间戳算
 *            出；两者不一致说明扫描写错了（取整或回绕处理有缺陷）。它检不出"样本真的不在"——见上一条。
 * @since 0.1.0
 * @exporter
 */
struct TraceReport final {
    /**
     * @brief [en] Ok; Param when the argument cannot describe a window at all.
     * @brief [zh] Ok；参数根本无法描述一段窗口时 Param。
     */
    ErrorCode status;
    /**
     * @brief [en] samples considered.
     * @brief [zh] 计入的样本数。
     */
    std::uint32_t samples;
    /**
     * @brief [en] declared loss before the window.
     * @brief [zh] 窗口之前声明的丢失量。
     */
    std::uint32_t lost;
    /**
     * @brief [en] inferred loss inside the window, in periods.
     * @brief [zh] 窗口内部推出的丢失量，以周期计。
     */
    std::uint32_t missing;
    /**
     * @brief [en] places where the cadence broke.
     * @brief [zh] 节拍断掉的位置数。
     */
    std::uint32_t gaps;
    /**
     * @brief [en] places where `t_us` went backwards.
     * @brief [zh] `t_us` 往回跳的位置数。
     */
    std::uint32_t reversals;
    /**
     * @brief [en] state- and cadence-contiguous runs.
     * @brief [zh] 状态与节拍都连续的小段数。
     */
    std::uint32_t segments;
    /**
     * @brief [en] `samples + lost`: how long the run is, in control periods.
     * @brief [zh] `samples + lost`：整段运行有多少个控制周期。
     */
    std::uint32_t periods;
    /**
     * @brief [en] the window carries a clock at all.
     * @brief [zh] 这段窗口是否有时钟。
     */
    bool timed;
    /**
     * @brief [en] the two ways of counting `missing` agree.
     * @brief [zh] `missing` 的两种数法一致。
     */
    bool consistent;
    /**
     * @brief [en] nothing is missing and nothing was declared lost.
     * @brief [zh] 没有缺失，也没有声明丢失。
     */
    bool complete;
};

namespace detail {

/**
 * @brief [en] Modular difference of two 32-bit timestamps, so a wrap is a normal step and not a 71-minute hole.
 * @brief [zh] 两个 32 位时间戳的模差，于是回绕是一个正常步进而不是一个 71 分钟的空洞。
 */
inline auto elapsed_us(std::uint32_t from, std::uint32_t to) noexcept -> std::uint32_t {
    return static_cast<std::uint32_t>(to - from);
}

/**
 * @brief [en] How many periods a gap of `delta_us` covers, rounded to the nearest whole one.
 * @brief [zh] `delta_us` 的间隔覆盖几个周期，取最近整数。
 */
inline auto periods_in(std::uint32_t delta_us, std::uint32_t period_us) noexcept -> std::uint32_t {
    return (delta_us + (period_us / 2U)) / period_us;
}

/**
 * @brief [en] Whether the clock went backwards, which modulo arithmetic cannot distinguish from a very long gap.
 * @brief [zh] 时钟是否往回走了——模运算无法把这种情况与一个极长的间隔区分开。
 * @note [en] Half the 32-bit range is the dividing line. A genuine forward gap of more than 35 minutes inside one
 *            attached window is not a thing this library produces, so treating the upper half as "backwards" is the
 *            only reading that is ever right.
 * @note [zh] 分界线是 32 位范围的一半。一个挂载期内真正向前跳过 35 分钟以上的情况，本库不会产生，所以把上半区读作"往回
 *            走"是唯一始终正确的读法。
 */
inline auto went_backwards(std::uint32_t delta_us) noexcept -> bool { return delta_us >= 0x8000U * 0x10000U; }

} // namespace detail

/**
 * @brief [en] Read a trace window and say what is missing from it, and whether it is the whole run.
 *
 *   The parser exists because a trace out of a ring is not a recording: it is the *last* `capacity` periods, and every
 *   number in it is correct while the window as a whole can quietly be the wrong window. Two things make that visible.
 *   The cadence is checked against the period the caller states -- not guessed from the samples, because a window with
 *   one gap in it would teach the parser the wrong period -- and the timestamp difference is taken modulo 2^32, since
 *   `t_us` is a 32-bit truncation that wraps every 71.6 minutes. A parser that subtracts naively reports a hole of four
 *   billion microseconds at every wrap.
 *
 * @brief [zh] 读取一段轨迹窗口，说出它缺了什么，以及它是否就是整段运行。
 *
 *   解析器的存在是因为从环里取出的轨迹并不是一份录音：它是 **最近** `capacity` 个周期，里面每个数字都对，而窗口整体可能
 *   悄悄是错的那个窗口。有两件事让这一点可见。节拍是拿调用方给定的周期来检查的——不从样本里猜，因为一个带缺口的窗口会教
 *   给解析器错误的周期；时间戳差按模 2^32 取，因为 `t_us` 是每 71.6 分钟回绕一次的 32 位截断。直接相减的解析器会在每次
 *   回绕处报出一个四十多亿微秒的空洞。
 *
 * @param span [en] the window and the loss the recorder declared.
 * @param span [zh] 窗口，以及记录器声明的丢失量。
 * @param period_us [en] the control period in microseconds -- a configuration fact, stated by the caller.
 * @param period_us [zh] 控制周期，微秒——一个由调用方陈述的配置事实。
 * @return [en] the report; `status` is `Param` when `period_us` is 0 or the span is not usable.
 * @return [zh] 报告； `period_us` 为 0 或窗口不可用时 `status` 为 `Param`。
 * @pre [en] `span.samples` is non-null when `span.size` is non-zero, and the samples are the retained window oldest
 *           first, exactly as `foc_motor_trace_read(0..size-1)` returns them.
 * @pre [zh] `span.size` 非零时 `span.samples` 非空，且样本按 `foc_motor_trace_read(0..size-1)` 的顺序自最旧起排列。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto analyze_trace(const TraceSpan& span, std::uint32_t period_us) noexcept -> TraceReport {
    TraceReport report{};
    report.samples = span.size;
    report.lost = span.dropped;
    report.periods = span.size + span.dropped;
    if ((period_us == 0U) || ((span.samples == nullptr) && (span.size != 0U))) {
        report.status = ErrorCode::Param;
        return report;
    }
    if (span.size == 0U) {
        // An empty window is not an error, and it is not complete either: a run of
        // nothing says nothing.
        // 空窗口不是错误，也不是"完整"：什么都没记录，就什么都没说。
        report.status = ErrorCode::Ok;
        report.segments = 0U;
        return report;
    }
    report.status = ErrorCode::Ok;
    const TraceSample* first = span.samples;
    for (std::uint32_t i = 1U; i < span.size; ++i) {
        if (span.samples[i].t_us != first->t_us) {
            report.timed = true;
            break;
        }
    }

    // One walk, two outputs: the segment count, and the anomaly scan. They cannot
    // disagree because they are the same pass -- a second pass over the same rule
    // is how the two silently drift apart after the first edit.
    // 一趟走查，两个产出：段数，以及异常扫描。它们不可能不一致，因为是同一趟——用两趟走同一条规则，正是第一次改动之后
    // 两者悄悄漂开的方式。
    report.segments = 1U;
    std::uint32_t scanned_missing = 0U;
    for (std::uint32_t i = 1U; i < span.size; ++i) {
        bool broke = (span.samples[i].mode != span.samples[i - 1U].mode);
        if (report.timed) {
            const std::uint32_t delta = detail::elapsed_us(span.samples[i - 1U].t_us, span.samples[i].t_us);
            if (delta == 0U) {
                // Two samples claiming the same instant: the clock is coarser than the
                // loop, which is a fact about the platform. It is not a gap and it is
                // not a reversal.
                // 两条样本声称同一时刻：时钟比控制环还粗，这是关于平台的事实。它不是缺口，也不是回退。
            } else if (detail::went_backwards(delta)) {
                ++report.reversals;
                broke = true;
            } else if (detail::periods_in(delta, period_us) != 1U) {
                ++report.gaps;
                scanned_missing += detail::periods_in(delta, period_us) - 1U;
                broke = true;
            }
        }
        if (broke) {
            ++report.segments;
        }
    }

    if (report.timed && (span.size >= 2U)) {
        const std::uint32_t total = detail::elapsed_us(span.samples[0].t_us, span.samples[span.size - 1U].t_us);
        const std::uint32_t by_endpoints = detail::periods_in(total, period_us) + 1U;
        const std::uint32_t missing_by_endpoints = (by_endpoints > span.size) ? (by_endpoints - span.size) : 0U;
        report.missing = scanned_missing;
        report.consistent = (missing_by_endpoints == scanned_missing);
    } else {
        // Without a clock there is nothing to reconcile, so `missing` stays 0 and
        // `consistent` stays false: reporting "consistent" for a window that could
        // not be checked would be the one wrong answer here.
        // 没有时钟就没有东西可对账，于是 `missing` 保持 0，`consistent` 保持
        // false：对一段无法检查的窗口报"一致"， 是这里唯一错误的答案。
        report.consistent = false;
    }

    report.complete = (report.lost == 0U) && (report.missing == 0U) && (report.gaps == 0U) &&
                      (report.reversals == 0U) && report.timed;
    return report;
}

/**
 * @brief [en] The `index`-th segment, without storing them: the same walk, asked for one answer.
 * @brief [zh] 第 `index` 段，不存储它们：同一趟走查，只问一个答案。
 * @param span [en] the window; same contract as `analyze_trace`.
 * @param span [zh] 窗口；契约与 `analyze_trace` 相同。
 * @param period_us [en] the control period in microseconds.
 * @param period_us [zh] 控制周期，微秒。
 * @param index [en] 0-based segment index.
 * @param index [zh] 0 基段下标。
 * @return [en] the segment, or `count == 0` when there is no such segment.
 * @return [zh] 该段；没有这一段时 `count == 0`。
 * @note [en] Enumerating costs one pass per segment, which is the right trade for a host-side tool that walks a few
 *            dozen segments once: the alternative is a second fixed-size table with a capacity nobody can pick
 *            correctly, in a header that is also compiled into firmware.
 * @note [zh] 枚举的代价是每段一趟，这对一个只会走几十段一次的上位机工具是正确的取舍：另一种做法是再加一张固定容量的表，
 *            而那个容量没人能选对，且这个头文件也会被编进固件。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto trace_segment(const TraceSpan& span, std::uint32_t period_us, std::uint32_t index) noexcept
    -> TraceSegment {
    if ((period_us == 0U) || (span.samples == nullptr) || (span.size == 0U)) {
        return TraceSegment{0U, 0U, 0U};
    }
    bool timed = false;
    for (std::uint32_t i = 1U; i < span.size; ++i) {
        if (span.samples[i].t_us != span.samples[0].t_us) {
            timed = true;
            break;
        }
    }
    std::uint32_t segment = 0U;
    std::uint32_t begin = 0U;
    for (std::uint32_t i = 1U; i <= span.size; ++i) {
        bool broke = false;
        if (i < span.size) {
            broke = (span.samples[i].mode != span.samples[i - 1U].mode);
            if (timed) {
                const std::uint32_t delta = detail::elapsed_us(span.samples[i - 1U].t_us, span.samples[i].t_us);
                if (delta != 0U) {
                    broke = broke || detail::went_backwards(delta) || (detail::periods_in(delta, period_us) != 1U);
                }
            }
        }
        if ((i == span.size) || broke) {
            if (segment == index) {
                return TraceSegment{begin, i - begin, span.samples[begin].mode};
            }
            ++segment;
            begin = i;
        }
    }
    return TraceSegment{0U, 0U, 0U};
}

} // namespace core

} // namespace foc
