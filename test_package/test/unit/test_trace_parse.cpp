// The trace parser, measured: does a window say what is missing from it, and does it survive the wrap?
//
// The criteria, written before the run:
//
//   1. a clean window is `complete`, one segment, and reconciles to the period it was recorded at;
//   2. a `t_us` wrap is a **normal step**: the same window that wraps reports zero gaps, while a naive signed
//      subtraction of its two endpoint timestamps says the clock went *backwards* -- the test computes both, so the
//      wrap case cannot pass by the parser and the naive reading agreeing;
//   3. a period missing *inside* the window is found, counted, and breaks the segment -- `missing` is the parser's own
//      inference and `lost` is the recorder's declaration, and they live in different fields;
//   4. a declared loss alone is enough to make the window *not* the whole run, without any interior gap;
//   5. a state change splits segments with no gap at all, and `trace_segment` enumerates exactly the segments the
//      report counted;
//   6. a window with no clock is neither complete nor consistent, because nothing in it can be checked.
//
// 判据在跑前写下（见上）：干净窗口完整且单段；回绕是正常步进（同时算出朴素相减会得出"倒退"，于是这条不可能靠两者
// 碰巧一致而通过）；窗口内部的缺失被找到并切断分段；仅"声明丢失"就足以让窗口不再是整段运行；状态变化在不产生缺口的
// 情况下切段，且 `trace_segment` 枚举出的段数等于报告数出的段数；没有时钟的窗口既不完整也不一致。

#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/sm.hpp>
#include <foc/core/trace.hpp>
#include <foc/core/trace_parse.hpp>

namespace {

using foc::core::analyze_trace;
using foc::core::MotorState;
using foc::core::TraceReport;
using foc::core::TraceSample;
using foc::core::TraceSegment;
using foc::core::TraceSpan;
using foc::core::trace_segment;
using foc::ErrorCode;

constexpr std::uint32_t kPeriodUs = 50U;

// A window of `count` samples at `period_us`, starting at `t0`, all in one state, with the timestamps allowed to wrap.
auto make_window(std::uint32_t count, std::uint32_t period_us, std::uint32_t t0 = 1000U,
                 MotorState state = MotorState::Run) -> std::vector<TraceSample> {
    std::vector<TraceSample> samples(count);
    for (std::uint32_t i = 0U; i < count; ++i) {
        TraceSample sample{};
        sample.t_us = t0 + (i * period_us);
        sample.mode = static_cast<std::uint8_t>(state);
        sample.status = static_cast<std::uint8_t>(foc::ErrorCode::Ok);
        sample.i_q = static_cast<float>(i);
        samples[i] = sample;
    }
    return samples;
}

auto span_of(const std::vector<TraceSample>& samples, std::uint32_t dropped = 0U) -> TraceSpan {
    return TraceSpan{samples.data(), static_cast<std::uint32_t>(samples.size()), dropped};
}

// 1. A clean window is the whole run, in one segment.
TEST(TraceParse, ACleanWindowIsCompleteAndOneSegment) {
    const auto samples = make_window(200U, kPeriodUs);
    const TraceReport report = analyze_trace(span_of(samples), kPeriodUs);
    EXPECT_EQ(report.status, ErrorCode::Ok);
    EXPECT_EQ(report.samples, 200U);
    EXPECT_EQ(report.lost, 0U);
    EXPECT_EQ(report.missing, 0U);
    EXPECT_EQ(report.gaps, 0U);
    EXPECT_EQ(report.reversals, 0U);
    EXPECT_EQ(report.segments, 1U);
    EXPECT_EQ(report.periods, 200U);
    EXPECT_TRUE(report.timed);
    EXPECT_TRUE(report.consistent);
    EXPECT_TRUE(report.complete);
}

// 2. The wrap at 2^32 is a step of one period, not a hole. The naive reading is computed here so that this test cannot
//    pass by both of them being right.
TEST(TraceParse, AWrapIsOnePeriodAndNotAHole) {
    // Start far enough below zero that 64 periods carry the timestamps past the 32-bit boundary.
    const std::uint32_t t0 = 0xFFFFFFFFU - (32U * kPeriodUs);
    const auto samples = make_window(64U, kPeriodUs, t0);
    ASSERT_LT(samples.back().t_us, samples.front().t_us) << "the fixture must actually wrap, and it did not";

    const TraceReport report = analyze_trace(span_of(samples), kPeriodUs);
    EXPECT_EQ(report.missing, 0U);
    EXPECT_EQ(report.gaps, 0U);
    EXPECT_EQ(report.reversals, 0U);
    EXPECT_EQ(report.segments, 1U);
    EXPECT_TRUE(report.complete);

    // What a parser that subtracts the endpoints as signed numbers would conclude. It has to be wrong, loudly.
    const std::int64_t naive = static_cast<std::int64_t>(samples.back().t_us) -
                               static_cast<std::int64_t>(samples.front().t_us);
    EXPECT_LT(naive, 0) << "the naive endpoint difference must be negative across a wrap";
    EXPECT_EQ(report.missing, 0U) << "the parser must not agree with the naive reading";
}

// 3. A period lost while attached is inferred from the timestamps, and splits the run where it happened.
TEST(TraceParse, AnInteriorGapIsFoundAndBreaksTheSegment) {
    auto samples = make_window(200U, kPeriodUs);
    // Three periods of 50 us between index 99 and 100 instead of one: two samples were never recorded.
    for (std::uint32_t i = 100U; i < samples.size(); ++i) {
        samples[i].t_us += 2U * kPeriodUs;
    }
    const TraceReport report = analyze_trace(span_of(samples), kPeriodUs);
    EXPECT_EQ(report.missing, 2U);
    EXPECT_EQ(report.gaps, 1U);
    EXPECT_EQ(report.reversals, 0U);
    EXPECT_EQ(report.segments, 2U);
    EXPECT_EQ(report.lost, 0U) << "an interior gap is not a declared loss";
    EXPECT_TRUE(report.consistent) << "the two ways of counting the missing periods must agree";
    EXPECT_FALSE(report.complete);
}

// 4. A declared loss on its own means the window is not the whole run -- even with no interior gap.
TEST(TraceParse, ADeclaredLossAloneMakesTheWindowPartial) {
    const auto samples = make_window(100U, kPeriodUs);
    const TraceReport report = analyze_trace(span_of(samples, 500U), kPeriodUs);
    EXPECT_EQ(report.lost, 500U);
    EXPECT_EQ(report.missing, 0U) << "nothing inside the window is missing";
    EXPECT_EQ(report.gaps, 0U);
    EXPECT_EQ(report.periods, 600U) << "the run is the retained window plus what was overwritten";
    EXPECT_TRUE(report.consistent);
    EXPECT_FALSE(report.complete);
}

// 5. A state change splits segments with no gap, and the enumerator agrees with the counter.
TEST(TraceParse, StateChangesSplitSegmentsAndTheEnumeratorAgrees) {
    auto samples = make_window(90U, kPeriodUs);
    for (std::uint32_t i = 0U; i < 30U; ++i) {
        samples[i].mode = static_cast<std::uint8_t>(MotorState::Align);
    }
    for (std::uint32_t i = 30U; i < 60U; ++i) {
        samples[i].mode = static_cast<std::uint8_t>(MotorState::Idle);
    }
    const auto span = span_of(samples);
    const TraceReport report = analyze_trace(span, kPeriodUs);
    EXPECT_EQ(report.segments, 3U);
    EXPECT_EQ(report.gaps, 0U);
    EXPECT_EQ(report.missing, 0U);

    const TraceSegment first = trace_segment(span, kPeriodUs, 0U);
    const TraceSegment second = trace_segment(span, kPeriodUs, 1U);
    const TraceSegment third = trace_segment(span, kPeriodUs, 2U);
    EXPECT_EQ(first.first_index, 0U);
    EXPECT_EQ(first.count, 30U);
    EXPECT_EQ(static_cast<MotorState>(first.mode), MotorState::Align);
    EXPECT_EQ(second.first_index, 30U);
    EXPECT_EQ(second.count, 30U);
    EXPECT_EQ(static_cast<MotorState>(second.mode), MotorState::Idle);
    EXPECT_EQ(third.first_index, 60U);
    EXPECT_EQ(third.count, 30U);
    EXPECT_EQ(static_cast<MotorState>(third.mode), MotorState::Run);
    EXPECT_EQ(trace_segment(span, kPeriodUs, 3U).count, 0U) << "there is no fourth segment";

    // The enumerator is a second implementation of the same rule, so it can drift; this is the check that catches it.
    std::uint32_t enumerated = 0U;
    while (trace_segment(span, kPeriodUs, enumerated).count != 0U) {
        ++enumerated;
    }
    EXPECT_EQ(enumerated, report.segments);
}

// 6. No clock: nothing in the window can be checked, so it is neither complete nor consistent.
TEST(TraceParse, AWindowWithoutAClockIsNeitherCompleteNorConsistent) {
    auto samples = make_window(50U, kPeriodUs);
    for (auto& sample : samples) {
        sample.t_us = 0U;
    }
    const TraceReport report = analyze_trace(span_of(samples), kPeriodUs);
    EXPECT_FALSE(report.timed);
    EXPECT_FALSE(report.consistent) << "a window that could not be checked must not report agreement";
    EXPECT_FALSE(report.complete);
    EXPECT_EQ(report.segments, 1U) << "state is still readable without a clock";
}

// 7. A period of zero cannot describe a cadence, and an empty window is not a complete one.
TEST(TraceParse, AnUnusableArgumentIsParamAndAnEmptyWindowIsNotComplete) {
    const auto samples = make_window(10U, kPeriodUs);
    EXPECT_EQ(analyze_trace(span_of(samples), 0U).status, ErrorCode::Param);

    const TraceSpan empty{nullptr, 0U, 0U};
    const TraceReport report = analyze_trace(empty, kPeriodUs);
    EXPECT_EQ(report.status, ErrorCode::Ok);
    EXPECT_EQ(report.segments, 0U);
    EXPECT_FALSE(report.complete);
}

} // namespace
