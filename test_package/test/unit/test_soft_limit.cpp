// 2.3a: the soft limit, as two claims rather than one knob. An out-of-range target is *refused* (a programming fault,
// answered by status and not by a clamp that makes it look plausible), and a fast axis near a wall is *slowed* so that
// `v^2 <= 2*a*d` holds at every sample -- an invariant with no width to tune, checkable in the period that applies it.
// 2.3a：软限位，作为两条主张而不是一个旋钮。越界目标是**被拒绝**的（编程错误，用状态回答，而不是用钳位让它看起来合理），
// 而靠近墙的快轴会被**放慢**，使 `v² <= 2·a·d` 在每个采样上成立——一条没有宽度可调的守恒式，在施加它的那个周期就可检。

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/soft_limit.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

constexpr float kDt = 6.25e-5F;
constexpr float kMaxRad = 0.500F;
constexpr float kMinRad = -0.500F;
constexpr float kPi = 3.14159265358979323846F;

auto window() -> foc::core::SoftLimitConfig { return foc::core::SoftLimitConfig{kMinRad, kMaxRad}; }

} // namespace

/**
 * @brief A target outside the window is refused and the previous one comes back; a target exactly on a wall is accepted and flagged.
 *
 *     Refusal rather than clamping is the whole point: a clamp turns an impossible command into a plausible one and the
 *     caller never learns that its program asked for something it cannot have. The inclusive boundary is the other half
 *     -- "move to the limit" is the one command a soft limit exists to make safe, and an exclusive wall makes the last
 *     usable position depend on the last digit of a float. 拒绝而不是钳位是全部要点：钳位把一条不可能的命令变成一条看起
 *     来合理的命令，调用方永远不知道它的程序要求了得不到的东西。含端点的边界是另一半——"走到限位"正是软限位存在的意义所
 *     在的那条命令，而排他的墙会让最后一个可用位置取决于一个 float 的最后一位。
 */
TEST(SoftLimit, AnOutOfRangeTargetIsRefusedAndThePreviousOneComesBack) {
    foc::core::SoftLimit limit(window());

    auto out = limit.apply(0.100F);
    EXPECT_EQ(out.status, foc::ErrorCode::Ok);
    EXPECT_FLOAT_EQ(out.target_rad, 0.100F);
    EXPECT_FALSE(out.at_wall);

    out = limit.apply(0.800F); // past the high wall
    EXPECT_EQ(out.status, foc::ErrorCode::Param);
    EXPECT_FLOAT_EQ(out.target_rad, 0.100F) << "the refused target must not become the target in force";

    out = limit.apply(-0.800F); // past the low wall
    EXPECT_EQ(out.status, foc::ErrorCode::Param);
    EXPECT_FLOAT_EQ(out.target_rad, 0.100F);

    // Exactly on a wall is inside, and says so.
    // 正好在墙上属于窗口内，并且说出来。
    out = limit.apply(kMaxRad);
    EXPECT_EQ(out.status, foc::ErrorCode::Ok);
    EXPECT_TRUE(out.at_wall);
    EXPECT_FLOAT_EQ(out.target_rad, kMaxRad);

    out = limit.apply(kMinRad);
    EXPECT_EQ(out.status, foc::ErrorCode::Ok);
    EXPECT_TRUE(out.at_wall);
    EXPECT_FLOAT_EQ(out.target_rad, kMinRad);
}

/**
 * @brief A first target outside the window is refused with nothing to fall back on -- inventing a legal position would be worse than saying "no". 第一个目标就在窗口外时被拒绝，且没有可回退的目标——编造一个合法位置比说"不行"更糟。
 */
TEST(SoftLimit, TheFirstTargetIsJudgedLikeEveryOther) {
    foc::core::SoftLimit limit(window());
    const auto out = limit.apply(0.900F);
    EXPECT_EQ(out.status, foc::ErrorCode::Param);
    EXPECT_FLOAT_EQ(out.target_rad, 0.0F) << "no target has ever been accepted, so none can be reported";

    // And the axis is not poisoned by it: the next legal target is admitted.
    // 而且轴不会因此被弄坏：下一个合法目标被接纳。
    EXPECT_EQ(limit.apply(0.050F).status, foc::ErrorCode::Ok);
}

/**
 * @brief A thousand periods of out-of-range commands never move the target in force outside the window. 一千个周期的越界指令从不把生效的目标移出窗口。
 */
TEST(SoftLimit, TheTargetInForceNeverLeavesTheWindowHoweverHardTheCallerPushes) {
    foc::core::SoftLimit limit(window());
    ASSERT_EQ(limit.apply(0.0F).status, foc::ErrorCode::Ok);
    float worst = 0.0F;
    int refusals = 0;
    for (int i = 0; i < 1000; ++i) {
        // A sawtooth of commands, most of them illegal, at both ends.
        // 一串指令，多数非法，两端都有。
        const float ask = ((i % 3) == 0) ? (2.0F + static_cast<float>(i % 7)) : (-0.2F + (0.001F * static_cast<float>(i % 100)));
        const auto out = limit.apply(ask);
        if (out.status == foc::ErrorCode::Param) {
            ++refusals;
        }
        worst = std::max(worst, std::fabs(out.target_rad));
    }
    EXPECT_LE(worst, kMaxRad) << "worst |target| " << worst;
    EXPECT_GT(refusals, 300) << "the run must actually be pushing: " << refusals << " refusals";
}

/**
 * @brief On the plant: the shaper's ceiling is min(sqrt(2*a*(d - v*dt))), and the `v*dt` term is what decides whether the axis stops or crosses. Three runs, one measurement.
 *
 *     The first version of this case asserted the ideal invariant `v^2 <= 2*a*d` on the *plant's* state and failed by
 *     19 mm: the ceiling is respected by the command from the instant it bites, but a loop needs the braking
 *     deceleration to arrive, and while it arrives the axis keeps travelling at `v`. 19 mm at 19.5 rad/s is 1 ms of
 *     travel, which is this drive's delay (one control period plus the 0.8 ms current rise). With that term the axis
 *     stops 1.04 mm short; without it, 19 mm past; and with no shaper at all it crosses the wall and keeps going. 本用
 *     例第一版在**plant 状态**上断言理想守恒式 `v² <= 2·a·d`，以 19 mm 失败：上限从咬合那一刻起就被指令遵守了，但环路需
 *     要制动力矩到位，而它在到位的这段时间里轴仍以 `v` 前进。19.5 rad/s 下的 19 mm 就是 1 ms 的行程，正是这台驱动的延迟
 *     （一个控制周期加 0.8 ms 电流上升）。有这一项时轴停在墙前 1.04 mm，没有时越过 19 mm，而完全不整形时它穿过墙继续
 *     走。
 */
TEST(SoftLimit, TheDelayTermIsTheDifferenceBetweenNineteenMillimetresPastAndOneShort) {
    const foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    const float a_max = params.kt * 3.0F / params.j; // 3 A limit: 1500 rad/s^2 here
    const float psi_f = params.kt / (1.5F * static_cast<float>(params.pole_pairs));
    const float command = 20.0F; // rad/s toward the wall
    const float delay_s = 1.0e-3F;
    const int periods = 4000;

    struct Result final {
        float min_distance;
        int ceiling_violations;
    };

    auto run = [&](bool shaped, float delay, float &min_distance, int &violations) {
        foc::sil::PmsmPlant plant(params);
        plant.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, -0.400F});
        const foc::core::SoftLimit limit(window());
        min_distance = 1.0e9F;
        violations = 0;
        for (int i = 0; i < periods; ++i) {
            const foc::sil::PlantState now = plant.state();
            const float velocity = shaped ? limit.shape(command, now.theta_mech, a_max, delay) : command;
            const float distance = kMaxRad - now.theta_mech;
            min_distance = std::min(min_distance, distance);
            // The ceiling, checked on the command every period it is heading for the wall.
            // 上限，在指令驶向墙的每一个周期上检查。
            if (shaped && distance > 0.0F) {
                const float allowed = 2.0F * a_max * (distance - (velocity * delay));
                if ((velocity * velocity) > (allowed * (1.0F + 1.0e-4F))) {
                    ++violations;
                }
            }
            const float iq = std::max(-3.0F, std::min(3.0F, 5.0F * (velocity - now.omega_mech)));
            const float omega_e = now.omega_mech * static_cast<float>(params.pole_pairs);
            plant.set_voltage_dq(foc::core::Dq<float>{-(omega_e * params.lq * iq), (params.rs * iq) + (omega_e * psi_f)});
            plant.advance(kDt);
        }
    };

    float shaped_distance = 0.0F;
    float ideal_distance = 0.0F;
    float raw_distance = 0.0F;
    int shaped_violations = 0;
    int ideal_violations = 0;
    int raw_violations = 0;
    run(true, delay_s, shaped_distance, shaped_violations);
    run(true, 0.0F, ideal_distance, ideal_violations);
    run(false, 0.0F, raw_distance, raw_violations);

    // The command never breaks its own ceiling, in any of the shaped runs.
    // 指令在任何一趟整形运行里都没有破自己的上限。
    EXPECT_EQ(shaped_violations, 0) << shaped_violations << " periods over the ceiling";
    EXPECT_EQ(ideal_violations, 0) << ideal_violations << " periods over the ideal ceiling";

    // With the delay term the axis stops short; without it, 19 mm past; unshaped, it goes through.
    // 有延迟项时轴停在墙前；没有时越过 19 mm；不整形则直接穿过。
    EXPECT_GT(shaped_distance, 0.0F) << "shaped run reached the wall: closest approach " << shaped_distance;
    EXPECT_NEAR(shaped_distance, 1.04e-3F, 0.6e-3F) << "measured clearance " << shaped_distance;
    EXPECT_LT(ideal_distance, 0.0F) << "the idealised ceiling must overshoot, or the delay term is not load-bearing: "
                                   << ideal_distance;
    EXPECT_NEAR(ideal_distance, -19.0e-3F, 3.0e-3F) << "measured overshoot without the delay term " << ideal_distance;
    // The unshaped run has no ceiling to break -- it crosses the wall, which is the whole point of the comparison.
    // 未整形的那一趟没有上限可破——它穿过墙，那正是这个对比的全部意义。
    EXPECT_LT(raw_distance, -0.5F) << "unshaped, the axis crosses the wall and keeps going: " << raw_distance;
    (void)raw_violations;
}

/**
 * @brief A deceleration of zero is not a default: the shaper passes the command through untouched rather than inventing a stopping distance. 减速度为零不是默认值：整形器把指令原样放过，而不是编造一个停车距离。
 */
TEST(SoftLimit, WithoutAStoppingCapabilityTheShaperDoesNotInventOne) {
    const foc::core::SoftLimit limit(window());
    EXPECT_FLOAT_EQ(limit.shape(5.0F, 0.400F, 0.0F, 0.0F), 5.0F);
    EXPECT_FLOAT_EQ(limit.shape(5.0F, 0.400F, -1.0F, 0.0F), 5.0F);
    // A negative delay is the idealised drive, not a rejected configuration: the term corrects that assumption, so the
    // correction is simply absent.
    // 负延迟是理想化驱动，不是被拒绝的配置：这一项纠正的就是那个假设，于是纠正本身缺席。
    EXPECT_FLOAT_EQ(limit.shape(5.0F, 0.400F, 1500.0F, -1.0e-3F), limit.shape(5.0F, 0.400F, 1500.0F, 0.0F));
    // And an inverted window is refused wholesale.
    // 反了的窗口整体被拒绝。
    foc::core::SoftLimit inverted(foc::core::SoftLimitConfig{1.0F, -1.0F});
    EXPECT_EQ(inverted.validate(), foc::ErrorCode::Param);
    EXPECT_EQ(inverted.apply(0.0F).status, foc::ErrorCode::Param);
}
