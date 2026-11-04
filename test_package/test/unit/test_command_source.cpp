// Command sources, measured: does a pulse train become a position that is exact at every whole turn, does a
// direction line only flip the sign, and does an input that stops making sense say so?
//
// The criteria were written before the run, and each one is a defect this file is meant to be able to catch:
//
//   1. `pulses_per_rev` pulses move the reference by **exactly** one turn, and `k` turns by exactly `k` turns
//      -- no residue that grows. The falsifier is a float-integrated mapping: it drifts, and the drift is
//      invisible until the count is large.
//   2. Reversing either the direction line or the request changes the **sign** and nothing else: the
//      magnitude after `n` pulses one way equals the magnitude after `n` the other way, to the last bit.
//   3. A period that carries more travel than `max_step_rad` is **reported** (`jump`) while the count stays
//      exact. Clamping instead would hide a lost count behind a plausible number, which is the mistake
//      `gear_ratio` already paid for once.
//   4. The analogue dead band is subtracted, so the command is continuous at the band edge rather than
//      stepping from zero to a finite value; full scale lands on full scale exactly.
//   5. A pulse-width frame outside the window is a broken link (`valid = false`, command exactly zero), and
//      silence becomes invalid only after `timeout_s` -- not on the first period without a frame.
//
// 判据在跑之前写下，每一条都对应本文件要能抓到的一个缺陷（见上）。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/command_source.hpp>

namespace {

using foc::core::AnalogCommand;
using foc::core::AnalogCommandConfig;
using foc::ErrorCode;
using foc::core::PpmCommand;
using foc::core::PpmCommandConfig;
using foc::core::StepCommand;
using foc::core::StepCommandConfig;

auto step_config(std::uint32_t pulses_per_rev = 4096U, float max_step_rad = 0.0F,
                 bool invert = false) -> StepCommandConfig {
    StepCommandConfig config{};
    config.pulses_per_rev = pulses_per_rev;
    config.invert_direction = invert;
    config.max_step_rad = max_step_rad;
    return config;
}

/**
 * @brief [en] A whole turn costs exactly a whole turn, however many turns have gone by.
 * @brief [zh] 一整圈恰好是一整圈，无论已经走过多少圈。
 */
TEST(CommandSource, AWholeTurnIsExactlyAWholeTurn) {
    StepCommand command(step_config(4096U));
    const float one_pulse = command.step(1).position_rad;
    for (int i = 1; i < 4096; ++i) {
        command.step(1);
    }
    const float after_one_turn = command.step(0).position_rad;
    EXPECT_EQ(command.pulses(), 4096);

    // Exactly one turn, not one turn plus the sum of 4096 rounding steps. The reference value is computed
    // the same way the class does it (whole turns separately), so this compares the class against the
    // definition rather than against itself.
    const float expected = static_cast<float>(6.283185307179586476925286766559);
    EXPECT_EQ(after_one_turn, expected);
    EXPECT_NEAR(one_pulse, expected / 4096.0F, 1e-9F);

    // And it stays exact far out: 1000 turns is 1000 * 2pi to the last bit the float allows.
    for (int turn = 0; turn < 999; ++turn) {
        command.step(4096);
    }
    EXPECT_EQ(command.pulses(), 4096LL * 1000LL);
    const float many = command.step(0).position_rad;
    EXPECT_EQ(many, static_cast<float>(1000.0 * 6.283185307179586476925286766559));
}

/**
 * @brief [en] Direction is a sign, never a magnitude.
 * @brief [zh] 方向只是符号，从不是幅值。
 */
TEST(CommandSource, DirectionIsASignAndNothingElse) {
    StepCommand forward(step_config());
    StepCommand backward(step_config(4096U, 0.0F, true));
    for (int i = 0; i < 128; ++i) {
        forward.step(1);
        backward.step(1);
    }
    EXPECT_EQ(forward.pulses(), 128);
    EXPECT_EQ(backward.pulses(), -128);
    EXPECT_FLOAT_EQ(backward.step(0).position_rad, -forward.step(0).position_rad);

    // The same holds for the count a hardware timer hands over, including a negative one.
    StepCommand mixed(step_config());
    mixed.step(300);
    mixed.step(-100);
    EXPECT_EQ(mixed.pulses(), 200);
    // Truncation toward zero is what the signed remainder needs: one pulse back is -1/4096 of a turn, and
    // a floor-division version would report it as a whole turn minus one pulse.
    StepCommand edge(step_config(4096U));
    EXPECT_FLOAT_EQ(edge.step(-1).position_rad, -static_cast<float>(6.283185307179586476925286766559 / 4096.0));
}

/**
 * @brief [en] More travel than one period allows is reported, and the count stays exact.
 * @brief [zh] 超过单周期允许的行程被报告，而计数保持精确。
 */
TEST(CommandSource, AJumpIsReportedAndNotClamped) {
    StepCommand command(step_config(4096U, 0.01F));
    const auto quiet = command.step(1);
    EXPECT_FALSE(quiet.jump);
    // 4096 pulses = one turn = 6.28 rad in one period, far past 0.01 rad.
    const auto loud = command.step(4096);
    EXPECT_TRUE(loud.jump);
    EXPECT_GT(loud.delta_rad, 6.0F);
    // Reported, not clamped: the count and the position are the real ones.
    EXPECT_EQ(loud.pulses, 4097);
    EXPECT_GT(loud.position_rad, 6.0F);
    std::printf("[command] a one-turn step reported delta %.4f rad, jump=%d, pulses=%lld\n",
                static_cast<double>(loud.delta_rad), loud.jump ? 1 : 0,
                static_cast<long long>(loud.pulses));
}

/**
 * @brief [en] A recipe that cannot be mapped exactly is refused rather than approximated.
 * @brief [zh] 不能精确映射的配方被拒绝，而不是被近似。
 */
TEST(CommandSource, AnUnmappableInterfaceIsRefused) {
    StepCommand zero(step_config(0U));
    EXPECT_EQ(zero.validate(), ErrorCode::Param);
    EXPECT_EQ(zero.step(10).status, ErrorCode::Param);
    EXPECT_EQ(zero.pulses(), 0);

    StepCommand negative_limit(step_config(4096U, -1.0F));
    EXPECT_EQ(negative_limit.validate(), ErrorCode::Param);
}

/**
 * @brief [en] The analogue dead band is subtracted, so the command has no step at its own edge.
 * @brief [zh] 模拟量的死区是被减掉的，因此指令在带的边缘没有台阶。
 */
TEST(CommandSource, TheDeadBandIsSubtractedSoTheCommandIsContinuous) {
    AnalogCommandConfig config{};
    config.full_scale_v = 10.0F;
    config.full_scale_value = 3000.0F; // rad/s at full scale
    config.deadband_v = 0.5F;
    AnalogCommand command(config);

    EXPECT_EQ(command.step(0.0F).value, 0.0F);
    const auto inside = command.step(0.4F);
    EXPECT_TRUE(inside.in_deadband);
    EXPECT_EQ(inside.value, 0.0F);

    // Just outside the band the command is tiny, not 0.4/10 of full scale: the band was subtracted.
    const auto just_outside = command.step(0.5F + 1e-3F);
    EXPECT_FALSE(just_outside.in_deadband);
    EXPECT_LT(std::fabs(just_outside.value), 1.0F);

    // And full scale lands on full scale exactly.
    EXPECT_FLOAT_EQ(command.step(10.0F).value, 3000.0F);
    // Beyond it, the command saturates rather than exceeding the unit the caller asked for.
    EXPECT_FLOAT_EQ(command.step(12.0F).value, 3000.0F);
    // Sign symmetry.
    EXPECT_FLOAT_EQ(command.step(-10.0F).value, -3000.0F);
}

/**
 * @brief [en] A pulse-width frame outside the window is a broken link, and silence expires.
 * @brief [zh] 窗口外的脉宽帧是断链，静默会到期。
 */
TEST(CommandSource, APulseWidthOutsideTheWindowIsABrokenLink) {
    PpmCommandConfig config{};
    config.min_us = 800.0F;
    config.max_us = 2200.0F;
    config.center_us = 1500.0F;
    config.full_scale_value = 1000.0F;
    config.timeout_s = 0.05F;
    PpmCommand command(config);

    // Centre commands exactly zero, the ends command full scale with opposite signs.
    EXPECT_EQ(command.step(1500.0F, 1e-3F).value, 0.0F);
    EXPECT_FLOAT_EQ(command.step(2200.0F, 1e-3F).value, 1000.0F);
    EXPECT_FLOAT_EQ(command.step(800.0F, 1e-3F).value, -1000.0F);

    // A frame outside the window is not a small command: it is a link that broke.
    const auto broken = command.step(400.0F, 1e-3F);
    EXPECT_FALSE(broken.valid);
    EXPECT_EQ(broken.value, 0.0F);
    const auto too_long = command.step(3000.0F, 1e-3F);
    EXPECT_FALSE(too_long.valid);
    EXPECT_EQ(too_long.value, 0.0F);

    // Silence holds the last command until the timeout proves the link is gone, then zeroes it.
    command.step(2000.0F, 1e-3F);
    const auto held = command.step(0.0F, 0.01F);
    EXPECT_TRUE(held.valid);
    EXPECT_GT(held.value, 0.0F);
    const auto expired = command.step(0.0F, 0.05F);
    EXPECT_FALSE(expired.valid);
    EXPECT_EQ(expired.value, 0.0F);
}
} // namespace
