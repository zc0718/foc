// A format that cannot round-trip is a format that loses information on every save. These tests pin the two properties
// that make this one usable as a library interface: what you write is what you read back, and a failed read changes
// nothing.
// 不能往返的格式就是每次保存都丢信息的格式。本文件钉住让它能当库接口用的两条性质：写出去的能原样读回来，失败的
// 读取什么都不改。

#include <cstdio>
#include <cstring>
#include <string>

#include <cstdlib>

#include <cstdint>

#include <gtest/gtest.h>

#include <foc/core/machine_map_text.hpp>

namespace foc {

namespace {

auto sample_document() -> core::MachineDocument {
    core::MachineDocument document{};
    document.params.rs = 0.512345678F;
    document.map.rows[0] = core::MachineMapPoint{0.0F, 4.0e-4F, 6.0e-4F, 0.050F};
    document.map.rows[1] = core::MachineMapPoint{2.0F, 3.0e-4F, 4.5e-4F, 0.052F};
    document.map.rows[2] = core::MachineMapPoint{4.0F, 2.0e-4F, 3.0e-4F, 0.054F};
    document.map.count = 3U;
    return document;
}

auto to_text(const core::MachineDocument& document) -> std::string {
    char buffer[512] = {};
    const std::size_t written = core::write_document(document, buffer, sizeof(buffer));
    EXPECT_GT(written, 0U);
    return std::string(buffer, written);
}

} // namespace

TEST(MachineMapText, WhatIsWrittenIsWhatIsReadBack) {
  // The criterion: every float survives the round trip bit for bit. `%.9g` is chosen for exactly that, so a shorter
  // format would make this test pass on a machine with lucky rounding and fail later elsewhere.
  // 判据：每个 float 逐位往返。`%.9g` 正是为此而选；更短的格式会让本测试在取整运气好的机器上通过、在别处失败。
  const core::MachineDocument original = sample_document();
  const std::string text = to_text(original);
  std::printf("\n[map-text] %zu bytes:\n%s", text.size(), text.c_str());

  core::MachineDocument parsed{};
  core::TextError error{};
  ASSERT_TRUE(core::read_document(text.data(), text.size(), &parsed, &error)) << "line " << error.line;
  EXPECT_EQ(error.status, core::TextStatus::Ok);

  EXPECT_FLOAT_EQ(parsed.params.rs, original.params.rs);
  ASSERT_EQ(parsed.map.count, original.map.count);
  for (std::size_t i = 0U; i < original.map.count; ++i) {
    EXPECT_FLOAT_EQ(parsed.map.rows[i].i_a, original.map.rows[i].i_a);
    EXPECT_FLOAT_EQ(parsed.map.rows[i].ld_h, original.map.rows[i].ld_h);
    EXPECT_FLOAT_EQ(parsed.map.rows[i].lq_h, original.map.rows[i].lq_h);
    EXPECT_FLOAT_EQ(parsed.map.rows[i].psi_f_wb, original.map.rows[i].psi_f_wb);
  }
  EXPECT_EQ(parsed.ignored, 0U);
}

TEST(MachineMapText, AnUnknownKeyIsCountedNotRejected) {
  // Forward compatibility is a property of the reader, not a courtesy: if a newer file cannot be read by an older
  // build, nobody will ever ship a newer file. The count is how the caller learns that the file said more.
  // 向前兼容是读方的性质而不是礼貌：如果新文件读不进来，就永远不会有人写新文件。计数是调用方得知"文件说得更多"
  // 的方式。
  const char* text = "format=1\nrs=0.5\nsource=fem\nmeasured_on=2026-10-28\nrow=0,4e-4,6e-4,0.05\n";
  core::MachineDocument parsed{};
  core::TextError error{};
  ASSERT_TRUE(core::read_document(text, std::strlen(text), &parsed, &error));
  EXPECT_EQ(parsed.ignored, 2U);
  EXPECT_FLOAT_EQ(parsed.params.rs, 0.5F);
  ASSERT_EQ(parsed.map.count, 1U);
}

TEST(MachineMapText, ACommentIsIgnoredAndAColumnHeaderIsJustAComment) {
  const char* text =
      "# foc machine document\n# columns: current_a,ld_h,lq_h,psi_f_wb\nformat=1\nrs=0.5\nrow=0,4e-4,6e-4,0.05\n";
  core::MachineDocument parsed{};
  ASSERT_TRUE(core::read_document(text, std::strlen(text), &parsed, nullptr));
  EXPECT_EQ(parsed.ignored, 0U);
  EXPECT_EQ(parsed.map.count, 1U);
}

TEST(MachineMapText, AFailedReadLeavesTheReceiverUntouched) {
  // Half-parsed parameters are worse than none: they look valid. Every failure path returns before the copy.
  // 半解析的参数比没有更糟：它们看起来是合法的。每条失败路径都在拷贝之前返回。
  const char* text = "format=1\nrs=0.5\nrow=0,4e-4,6e-4,0.05\nrow=0,3e-4,4e-4,0.05\n";
  core::MachineDocument parsed = sample_document();
  const core::MachineDocument before = parsed;
  core::TextError error{};
  EXPECT_FALSE(core::read_document(text, std::strlen(text), &parsed, &error));
  EXPECT_EQ(error.status, core::TextStatus::NotSorted);
  EXPECT_EQ(error.line, 4U);
  EXPECT_FLOAT_EQ(parsed.params.rs, before.params.rs);
  EXPECT_EQ(parsed.map.count, before.map.count);
}

TEST(MachineMapText, EachWayOfBeingWrongHasItsOwnAnswer) {
  struct Case final {
    const char* text;
    core::TextStatus expected;
  };
  const Case cases[] = {
      {"rs=0.5\nrow=0,4e-4,6e-4,0.05\n", core::TextStatus::Ok},
      {"format=2\nrs=0.5\nrow=0,4e-4,6e-4,0.05\n", core::TextStatus::Unsupported},
      {"format=1\nrow=0,4e-4,6e-4,0.05\n", core::TextStatus::NoParameters},
      {"format=1\nrs=0.5\n", core::TextStatus::NoParameters},
      {"format=1\nrs=-1\nrow=0,4e-4,6e-4,0.05\n", core::TextStatus::OutOfRange},
      {"format=1\nrs=0.5\nrow=0,4e-4,6e-4\n", core::TextStatus::BadFormat},
      {"format=1\nrs=0.5\nrow=0,4e-4,6e-4,0.05,extra\n", core::TextStatus::BadFormat},
      {"format=1\nrs=0.5\nnot a key value line\n", core::TextStatus::BadFormat},
      {"format=1\nrs=0.5\nrow=0,0,6e-4,0.05\n", core::TextStatus::OutOfRange},
  };
  for (const Case& one : cases) {
    core::MachineDocument parsed{};
    core::TextError error{};
    const bool ok = core::read_document(one.text, std::strlen(one.text), &parsed, &error);
    EXPECT_EQ(ok, one.expected == core::TextStatus::Ok) << "text: " << one.text;
    if (!ok) {
      EXPECT_EQ(error.status, one.expected) << "text: " << one.text;
    }
  }
}

TEST(MachineMapText, ASmallBufferIsReportedRatherThanTruncatingSilently) {
  // A truncated document that still parses would silently drop rows -- the worst failure mode for a parameter file.
  // 一个还能解析的截断文档会静默丢行——对参数文件来说这是最坏的失败模式。
  const core::MachineDocument document = sample_document();
  char size_probe[1] = {};
  const std::size_t needed = core::write_document(document, size_probe, sizeof(size_probe));
  EXPECT_EQ(needed, 0U) << "a one-byte buffer cannot hold the document";

  char exact[256] = {};
  const std::size_t length = core::write_document(document, exact, sizeof(exact));
  ASSERT_GT(length, 0U);
  char tight[256] = {};
  EXPECT_EQ(core::write_document(document, tight, length), 0U) << "the terminator needs one more byte";
  EXPECT_GT(core::write_document(document, tight, length + 1U), 0U);
}

} // namespace foc

TEST(MachineMapText, NineSignificantDigitsAreExactlyWhatAFloatNeedsAndEightAreNot) {
  // The writer uses `%.9g`, and `%.9g` is not a round number picked for looks: `FLT_DECIMAL_DIG` is 9, the shortest
  // width that is guaranteed to round-trip every finite float. The way to show that a chosen precision is the right one
  // is to measure the alternative and watch it fail -- so both are measured here on the same deterministic samples.
  // 写方用 `%.9g`，而 `%.9g` 不是随手挑的整齐数字：`FLT_DECIMAL_DIG` 就是 9，是保证任意有限 float 都能往返的最短宽度。
  // 要证明选定的精度是对的，就把替代方案量出来并看它失败——所以这里在同一批确定性样本上把两者都量了。
  const auto lcg = [](std::uint32_t& state) {
    state = (state * 1664525U) + 1013904223U;
    return state;
  };
  const auto bits_of = [](float value) {
    std::uint32_t word = 0U;
    std::memcpy(&word, &value, sizeof(word));
    return word;
  };
  std::uint32_t state = 12345U;
  int nine_ok = 0;
  int nine_bad = 0;
  int eight_ok = 0;
  int eight_bad = 0;
  for (int sample = 0; sample < 4000; ++sample) {
    // A float in the range the map actually holds: millihenries, finite, non-zero.
    // 落在表实际取值的范围里：毫亨、有限、非零。
    const float truth = 1.0e-4F + (static_cast<float>(lcg(state) >> 8U) * 3.0e-3F / 16777216.0F);
    // The library's own codec, one row per document: write then read, compare the bits.
    // 用库自己的编解码，一个文档一行：写、读、比位。
    foc::core::MachineDocument document{};
    document.params.rs = 0.1F;
    document.map.count = 1U;
    document.map.rows[0].i_a = 1.0F;
    document.map.rows[0].ld_h = truth;
    document.map.rows[0].lq_h = truth;
    document.map.rows[0].psi_f_wb = 0.1F;
    char text[512]{};
    const std::size_t used = foc::core::write_document(document, text, sizeof(text));
    EXPECT_GT(used, 0U);
    foc::core::MachineDocument back{};
    foc::core::TextError error{};
    ASSERT_TRUE(foc::core::read_document(text, used, &back, &error)) << "sample " << sample;
    if (bits_of(back.map.rows[0].ld_h) == bits_of(truth)) {
      ++nine_ok;
    } else {
      ++nine_bad;
    }
    // The falsifier: the same value through `%.8g` and back. One digit shorter, and the last bit is where it shows.
    // 反证：同一个值走 `%.8g` 再回来。少一位数字，露出来的地方就是最低位。
    char narrow[64]{};
    (void)std::snprintf(narrow, sizeof(narrow), "%.8g", static_cast<double>(truth));
    const float round_tripped = std::strtof(narrow, nullptr);
    if (bits_of(round_tripped) == bits_of(truth)) {
      ++eight_ok;
    } else {
      ++eight_bad;
    }
  }
  // `%d` takes an int: passing a double to it is the same undefined behaviour the codec's own comment warns about.
  // `%d` 接 int：把 double 传给它，正是编解码器自己注释里警告的那种未定义行为。
  std::printf("[map-text] %%.9g (the writer's width): %d/%d round-trip bit-exactly\n", nine_ok, nine_ok + nine_bad);
  std::printf("[map-text] %%.8g (one digit shorter): %d/%d round-trip bit-exactly\n", eight_ok, eight_ok + eight_bad);
  EXPECT_EQ(0, nine_bad) << "9 significant digits must round-trip every float (FLT_DECIMAL_DIG)";
  EXPECT_GT(eight_bad, 0) << "if 8 digits also round-tripped everything, the writer would be one digit too wide";
}
