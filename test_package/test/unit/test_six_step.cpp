// Six-step commutation, measured: does the vector really advance one sector at a time, does an illegal code
// produce no torque, and is a transition the rotor cannot make refused rather than obeyed?
//
// The criteria, written before the run:
//
//   1. walking the legal codes in a forward sequence turns the applied vector by **60 degrees per sector**,
//      measured through Clarke -- i.e. against a different map than the table itself, so an error in the
//      table cannot be confirmed by the table;
//   2. an illegal code (`0` or `7`) answers the **zero vector** and says `holding`: the pins are not naming a
//      position, and a vector invented for them produces torque in a direction nobody chose;
//   3. a jump of two sectors is **refused** (`skipped`) and the previous vector is held -- the table would
//      otherwise answer a vector 120 or 180 degrees away, which is what a missed read looks like;
//   4. reverse is the forward vector turned by **180 degrees**, at the same code;
//   5. the vector magnitude scales with the amplitude, one for one.
//
// 判据在跑前写下（见上）：相邻扇区使矢量转 60°，非法编码给零矢量且说 holding，跨两格被拒绝并保持旧矢量，
// 反向是同编码下转 180°，幅值一一正比。

#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/six_step.hpp>
#include <foc/core/transforms.hpp>
#include <foc/driver/stub_hall.hpp>

namespace {

using foc::core::clarke;
using foc::ErrorCode;
using foc::core::HallDirection;
using foc::core::kHallNoSector;
using foc::core::six_step_duty;
using foc::core::SixStepCommutation;
using foc::core::SixStepConfig;

constexpr float kDeg = 3.14159265358979F / 180.0F;

auto config(float amplitude = 0.8F, HallDirection direction = HallDirection::Forward) -> SixStepConfig {
    SixStepConfig out{};
    out.amplitude = amplitude;
    out.direction = direction;
    out.max_step_sectors = 1U;
    return out;
}

/** @brief [en] The code that names `sector`, found by asking the sector map itself. [zh] 由扇区映射本身找出指向该扇区的编码。 */
auto code_for(std::uint8_t sector) -> std::uint8_t {
    for (std::uint8_t code = 1U; code <= 6U; ++code) {
        if (foc::core::hall_sector(code) == sector) {
            return code;
        }
    }
    return 0U;
}

/** @brief [en] The applied vector's angle, in rad, through Clarke. [zh] 施加矢量的角度，rad，经 Clarke 得到。 */
auto vector_angle_rad(const foc::core::Uvw<float>& duty) -> float {
    const auto ab = clarke(duty);
    return std::atan2(static_cast<float>(ab.beta), static_cast<float>(ab.alpha));
}

auto wrap(float angle) -> float {
    while (angle > 3.14159265F) {
        angle -= 6.28318531F;
    }
    while (angle < -3.14159265F) {
        angle += 6.28318531F;
    }
    return angle;
}

/**
 * @brief [en] Walking the sectors turns the vector by one sector each time.
 * @brief [zh] 逐个走过扇区，矢量每次都转一个扇区。
 */
TEST(SixStep, WalkingTheSectorsTurnsTheVectorSixtyDegreesEach) {
    SixStepCommutation commutation(config());
    for (std::uint8_t sector = 0U; sector < 6U; ++sector) {
        const auto out = commutation.step(code_for(sector));
        ASSERT_EQ(out.status, ErrorCode::Ok);
        EXPECT_FALSE(out.holding);
        EXPECT_FALSE(out.skipped);
        EXPECT_EQ(out.sector, sector);
        if (sector > 0U) {
            const float previous = vector_angle_rad(six_step_duty(code_for(static_cast<std::uint8_t>(sector - 1U)),
                                                                  0.8F, HallDirection::Forward));
            const float now = vector_angle_rad(out.duty);
            EXPECT_NEAR(std::fabs(wrap(now - previous)), 60.0F * kDeg, 1e-4F)
                << "sector " << static_cast<int>(sector) << " did not advance the vector by one sector";
        }
        // Two phases driven, one at mid-rail: that is what six-step is.
        const int mid = (std::fabs(out.duty.u - 0.5F) < 1e-6F ? 1 : 0) + (std::fabs(out.duty.v - 0.5F) < 1e-6F ? 1 : 0) +
                        (std::fabs(out.duty.w - 0.5F) < 1e-6F ? 1 : 0);
        EXPECT_EQ(mid, 1);
    }
}

/**
 * @brief [en] A code that names no position answers the zero vector and says so.
 * @brief [zh] 不指向任何位置的编码给出零矢量并说出来。
 */
TEST(SixStep, AnIllegalCodeIsTheZeroVectorNotAGuess) {
    SixStepCommutation commutation(config());
    static_cast<void>(commutation.step(code_for(2U)));
    for (std::uint8_t illegal : {0U, 7U}) {
        const auto out = commutation.step(illegal);
        EXPECT_TRUE(out.holding);
        EXPECT_FALSE(out.changed);
        EXPECT_FLOAT_EQ(out.duty.u, 0.5F);
        EXPECT_FLOAT_EQ(out.duty.v, 0.5F);
        EXPECT_FLOAT_EQ(out.duty.w, 0.5F);
        // The last sector is remembered: the next legal read is still compared against the real position.
        EXPECT_EQ(commutation.sector(), 2U);
    }
    EXPECT_EQ(commutation.illegal(), 2U);
}

/**
 * @brief [en] A transition the rotor cannot have made is refused, and the old vector is held.
 * @brief [zh] 转子不可能做出的迁移被拒绝，旧矢量被保持。
 */
TEST(SixStep, AJumpOfTwoSectorsIsRefusedAndTheOldVectorHeld) {
    SixStepCommutation commutation(config());
    const auto first = commutation.step(code_for(0U));
    ASSERT_EQ(first.sector, 0U);

    // Sector 3 is three places away -- 180 degrees of rotor motion between two control periods. Obeying the
    // table here would apply a vector pointing the wrong way; holding costs one period of stale torque.
    const auto jumped = commutation.step(code_for(3U));
    EXPECT_TRUE(jumped.skipped);
    EXPECT_FALSE(jumped.changed);
    EXPECT_EQ(jumped.sector, 0U);
    EXPECT_FLOAT_EQ(jumped.duty.u, first.duty.u);
    EXPECT_FLOAT_EQ(jumped.duty.v, first.duty.v);
    EXPECT_FLOAT_EQ(jumped.duty.w, first.duty.w);
    EXPECT_EQ(commutation.rejected(), 1U);

    // A neighbour is accepted, and after that the sequence continues normally.
    const auto neighbour = commutation.step(code_for(1U));
    EXPECT_TRUE(neighbour.changed);
    EXPECT_EQ(neighbour.sector, 1U);
}

/**
 * @brief [en] Reverse is the same table, three sectors later -- 180 degrees.
 * @brief [zh] 反向是同一张表往后挪三格——180°。
 */
TEST(SixStep, ReverseIsTheForwardVectorTurnedByHalfATurn) {
    SixStepCommutation forward(config(0.8F, HallDirection::Forward));
    SixStepCommutation reverse(config(0.8F, HallDirection::Reverse));
    const std::uint8_t code = code_for(1U);
    const float f = vector_angle_rad(forward.step(code).duty);
    const float r = vector_angle_rad(reverse.step(code).duty);
    EXPECT_NEAR(std::fabs(wrap(r - f)), 180.0F * kDeg, 1e-4F);
}

/**
 * @brief [en] The amplitude is a magnitude, one for one.
 * @brief [zh] 幅值就是幅值，一一对应。
 */
TEST(SixStep, TheAmplitudeScalesTheVectorOneForOne) {
    const std::uint8_t code = code_for(4U);
    const auto magnitude = [code](float amplitude) {
        const auto ab = clarke(six_step_duty(code, amplitude, HallDirection::Forward));
        return std::sqrt(static_cast<float>(ab.alpha * ab.alpha + ab.beta * ab.beta));
    };
    const float small = magnitude(0.4F);
    const float large = magnitude(0.8F);
    EXPECT_NEAR(large / small, 2.0F, 1e-5F);

    SixStepConfig bad = config();
    bad.amplitude = 1.5F;
    SixStepCommutation refused(bad);
    EXPECT_EQ(refused.validate(), ErrorCode::Param);
    EXPECT_EQ(refused.step(code).status, ErrorCode::Param);
    EXPECT_EQ(refused.sector(), kHallNoSector);
}
/**
 * @brief [en] The port feeds the policy: the contract is usable by a host that only has the three bits.
 * @brief [zh] 端口喂给策略：只有三个位的宿主也能用这份契约。
 */
TEST(SixStep, TheHallPortFeedsThePolicyDirectly) {
    foc::driver::StubHall pins;
    SixStepCommutation commutation(config());
    pins.set_states(code_for(0U));
    EXPECT_EQ(commutation.step(pins.states()).sector, 0U);
    pins.set_states(code_for(1U));
    EXPECT_EQ(commutation.step(pins.states()).sector, 1U);
    // The stub does not validate, by design: an illegal code reaching the policy is the policy's case, not
    // something a driver hides.
    pins.set_states(0U);
    EXPECT_TRUE(commutation.step(pins.states()).holding);
}
} // namespace
