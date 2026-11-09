// 2.3a: the soft limit, as two claims rather than one knob. An out-of-range target is *refused* (a programming fault,
// answered by status and not by a clamp that makes it look plausible), and a fast axis near a wall is *slowed* so that
// `v^2 <= 2*a*d` holds at every sample -- an invariant with no width to tune, checkable in the period that applies it.
// 2.3a：软限位，作为两条主张而不是一个旋钮。越界目标是 **被拒绝** 的（编程错误，用状态回答，而不是用钳位让它看起来合理），
// 而靠近墙的快轴会被 **放慢** ，使 `v² <= 2·a·d` 在每个采样上成立——一条没有宽度可调的守恒式，在施加它的那个周期就可检。

#include <algorithm>
#include <cmath>
#include <limits>

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
 *     例第一版在 **plant 状态** 上断言理想守恒式 `v² <= 2·a·d`，以 19 mm 失败：上限从咬合那一刻起就被指令遵守了，但环路
 *     需要制动力矩到位，而它在到位的这段时间里轴仍以 `v` 前进。19.5 rad/s 下的 19 mm 就是 1 ms 的行程，正是这台驱动的延
 *     迟（一个控制周期加 0.8 ms 电流上升）。有这一项时轴停在墙前 1.04 mm，没有时越过 19 mm，而完全不整形时它穿过墙继续
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

    auto run = [&](bool shaped, float delay, float &min_distance, int &violations, float &worst_over) {
        foc::sil::PmsmPlant plant(params);
        plant.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, -0.400F});
        const foc::core::SoftLimit limit(window());
        min_distance = 1.0e9F;
        violations = 0;
        worst_over = 0.0F;
        for (int i = 0; i < periods; ++i) {
            const foc::sil::PlantState now = plant.state();
            // The position is narrowed once, to the value the shaper actually receives. Computing `kMaxRad - theta`
            // in double here would measure the ceiling against a more precise distance than the one the shaper could
            // act on, and near the wall that difference is catastrophic cancellation: `0.5 - theta` is a subtraction
            // of two nearly equal numbers, so a 6e-8 rounding of `theta` is 6e-4 *relative* error on the remaining
            // 1e-4 rad, which shows up as a 0.13 % overshoot. The API is float; the check has to use the same float.
            // 位置只收窄一次，收窄到整形器真正收到的那个值。在这里用 double 算 `kMaxRad − theta`，等于拿一个比整形器
            // 能作用的距离更精确的值去量上限；而贴墙处那是灾难性相消：`0.5 − theta` 是两个相近数相减，`theta` 上 6e-8
            // 的舍入就是剩下 1e-4 rad 上 6e-4 的 **相对** 误差，表现为 0.13 % 的越限。API 是 float，检查就得用同一个
            // float。
            const float position_rad = static_cast<float>(now.theta_mech);
            const float velocity = shaped ? limit.shape(command, position_rad, a_max, delay) : command;
            const float distance = kMaxRad - position_rad;
            min_distance = std::min(min_distance, distance);
            // The ceiling, checked on the command every period it is heading for the wall.
            // 上限，在指令驶向墙的每一个周期上检查。
            if (shaped && distance > 0.0F) {
                const float allowed = 2.0F * a_max * (distance - (velocity * delay));
                if (allowed > 0.0F) {
                    const float over = ((velocity * velocity) / allowed) - 1.0F;
                    worst_over = (over > worst_over) ? over : worst_over;
                    if (over > 1.0e-4F) {
                        ++violations;
                    }
                }
            }
            const float iq = std::max(-3.0F, std::min(3.0F, 5.0F * (velocity - static_cast<float>(now.omega_mech))));
            const float omega_e = static_cast<float>(now.omega_mech) * static_cast<float>(params.pole_pairs);
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
    float shaped_over = 0.0F;
    float ideal_over = 0.0F;
    float raw_over = 0.0F;
    run(true, delay_s, shaped_distance, shaped_violations, shaped_over);
    run(true, 0.0F, ideal_distance, ideal_violations, ideal_over);
    run(false, 0.0F, raw_distance, raw_violations, raw_over);

    // The command never breaks its own ceiling, in any of the shaped runs.
    // 指令在任何一趟整形运行里都没有破自己的上限。
    EXPECT_EQ(shaped_violations, 0) << shaped_violations << " periods over the ceiling";
    // The idealised command is checked by its *size*, not by a count. The shaper's own guard and this checker are two
    // spellings of `v^2 <= 2*a*d`, and in the periods where the ceiling first bites they differ in the last bits, so a
    // count reports a breach of one period in four thousand whose magnitude no plant can see. The size is the physical
    // quantity; the count is a boundary artefact in either direction.
    // 理想指令用越限的 **大小** 来查，而不是用计数。整形器自己的护栏与本检查器是 `v² ≤ 2·a·d` 的两种写法，在上限开始
    // 咬合的那几个周期里它们差在末几位，于是计数会报"四千个周期里有一个越限"，量级小到任何 plant 都看不见。物理量
    // 是越限的大小；计数无论正反都是边界假象。
    std::printf("[soft] worst relative overshoot: shaped %.3e (%d periods), ideal %.3e (%d), unshaped %.3e (%d)\n",
                static_cast<double>(shaped_over), shaped_violations, static_cast<double>(ideal_over),
                ideal_violations, static_cast<double>(raw_over), raw_violations);
    EXPECT_EQ(ideal_violations, 0) << ideal_violations << " periods over the ideal ceiling";
    EXPECT_LT(ideal_over, 1.0e-5F) << "the idealised command really breaks its own ceiling: worst relative overshoot "
                                      << ideal_over;

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



namespace {

/**
 * @brief [en] The largest command the cap lets through untouched at distance `d`.
 *
 * @brief [zh] 距离 `d` 处上限原样放过的最大指令。
 *
 *   The observable is exact, which is why this is a bisection and not a tolerance: below the seam the cap returns the
 *   caller's float unchanged (it is the same value, not a computation of it), above it it returns something strictly
 *   smaller. So "the largest accepted command" is a well-defined number and can be compared to a closed form without
 *   arguing about rounding. 这个观测量是精确的，这正是它用二分而不是容差的原因：接缝以下上限原样返回调用方的那个 float
 *   （是同一个值，不是它的计算结果），接缝以上返回严格更小的东西。所以"被接受的最大指令" 是个良定义的数，可以不带舍入争
 *   论地去和一个闭式比较。
 */
auto largest_accepted(double distance, double decel, double delay) -> double {
    foc::core::SoftLimit limit(
        foc::core::SoftLimitConfig{-1.0e3F, static_cast<float>(distance)});
    double below = 0.0;
    double above = 2.0e2;
    for (int i = 0; i < 80; ++i) {
        const double middle = 0.5 * (below + above);
        const float command = static_cast<float>(middle);
        if (limit.shape(command, 0.0F, static_cast<float>(decel), static_cast<float>(delay)) ==
            command) {
            below = middle;
        } else {
            above = middle;
        }
    }
    return below;
}

} // namespace



/**
 * @brief [en] The cap's seam is `sqrt(a^2*tau^2 + 2*a*d) - a*tau`, homogeneous of degree one in `(d, a)`, and its three
 *             sensitivity coefficients are the ones differentiating that expression gives.
 *
 * @brief [zh] 上限的接缝是 `sqrt(a²τ² + 2·a·d) − a·τ`，在 `(d, a)` 上是一次齐次的，而它的三个灵敏度系数就是对那个式子求
 *             导得到的。
 *
 *   The header states the invariant `v^2 <= 2*a*(d - v*delay)` and stops there. An invariant that is only ever
 *   *checked* leaves the question "how much speed does this distance actually buy me" unanswered, and that is the
 *   question a commissioning engineer asks. The answer is the fixed point of the map: the largest `v` that satisfies
 *   the inequality with equality. Solving the quadratic `v^2 + 2*a*tau*v - 2*a*d = 0` gives
 *   `v* = sqrt(a^2*tau^2 + 2*a*d) - a*tau`.
 *
 *   Three consequences, each measurable on its own. It is **homogeneous of degree one in `(d, a)` jointly**: scale both
 *   the window and the braking capability and every accepted speed scales with them, so a machine twice as strong
 *   working over twice the distance is exactly twice as fast -- not `sqrt(2)` times, which is what the delay-free form
 *   would say. It reduces to `sqrt(2*a*d)` as `tau -> 0`, so the delay term is a correction that never changes the
 *   leading behaviour. And its three partials are `a/(v_seam + a*tau)`, `v_seam^2/(2*a*(v_seam + a*tau))` and
 *   `-a*v_seam/(v_seam + a*tau)`, which is the sensitivity table the registry asked for: one coefficient per error
 *   source (wall position, deceleration, drive delay).
 *
 *   头文件只给出守恒式 `v² ≤ 2·a·(d − v·延迟)` 就停了。一条只被 **检查** 的守恒式没有回答"这段距离到底给我换来多少速度
 *   "，而那正是调试工程师要问的。答案是这个映射的不动点：让不等式取等号的最大 `v`。解二次式 `v² + 2·a·τ·v − 2·a·d = 0`
 *   得 `v* = sqrt(a²τ² + 2·a·d) − a·τ`。
 *
 *   三个后果，每个都可单独量。它在 `(d, a)` 上是 **联合的一次齐次** ：把窗口与制动能力一起放大，每个被接受的速度都随之
 *   放大，所以一台强一倍、行程也长一倍的机器恰好快一倍——而不是 `sqrt(2)` 倍，那是不带延迟的式子会说的。 `τ → 0` 时它退
 *   化为 `sqrt(2·a·d)`，所以延迟项是一个永远不改变主行为的修正。它的三个偏导是 `a/(v_seam+aτ)`、
 *   `v_seam²/(2a(v_seam+aτ))`、 `−a·v_seam/(v_seam+aτ)`，这就是登记册要的那张敏感度表：每个误差源（墙的位置、减速度、驱
 *   动延迟）一个系数。
 */
TEST(SoftLimit, TheCapHasAFixedPointAndThreeSensitivityCoefficients) {
    const struct {
        double distance;
        double decel;
        double delay;
    } cases[] = {{0.1533, 1500.0, 1.0e-3},
                 {0.3333, 1500.0, 1.0e-2},
                 {0.2000, 1500.0, 0.0},
                 {0.0500, 500.0, 2.0e-3},
                 {1.0000, 100.0, 5.0e-3}};

    std::printf("\n%10s %8s %8s %14s %14s %10s\n", "d", "a", "tau", "measured v*", "closed form",
                "ratio");
    for (const auto& item : cases) {
        const double closed_form = std::sqrt(item.decel * item.decel * item.delay * item.delay +
                                             2.0 * item.decel * item.distance) -
                                   item.decel * item.delay;
        const double measured = largest_accepted(item.distance, item.decel, item.delay);
        const double ratio = measured / closed_form;
        std::printf("%10.4f %8.1f %8.4f %14.6f %14.6f %10.6f\n", item.distance, item.decel,
                    item.delay, measured, closed_form, ratio);
        EXPECT_NEAR(1.0, ratio, 1.0e-4) << "the seam is the fixed point of the cap";
    }

    // Homogeneous of degree one in (d, a) jointly, which is the property that makes the answer
    // intuitive rather than merely correct.
    // 在 (d, a) 上联合一次齐次——这条性质让答案不只是正确的，还是直觉上说得通的。
    const double anchor_distance = 0.1533;
    const double anchor_decel = 1500.0;
    const double anchor_delay = 1.0e-3;
    const double anchor = largest_accepted(anchor_distance, anchor_decel, anchor_delay);
    std::printf("[homogeneity] ");
    for (const double k : {0.25, 0.5, 2.0, 4.0}) {
        const double scaled =
            largest_accepted(k * anchor_distance, k * anchor_decel, anchor_delay);
        std::printf("k=%.2f -> %.6f  ", k, scaled / anchor);
        EXPECT_NEAR(k, scaled / anchor, 1.0e-4) << "scaling d and a by " << k << " must scale v* by "
                                                << k;
    }
    std::printf("\n");

    // The three coefficients, against central differences on the seam itself.
    // 三个系数，对着接缝本身的中心差分。
    const double step = 2.0e-3;
    const double dv_dd =
        (largest_accepted(anchor_distance * (1.0 + step), anchor_decel, anchor_delay) -
         largest_accepted(anchor_distance * (1.0 - step), anchor_decel, anchor_delay)) /
        (2.0 * step * anchor_distance);
    const double dv_da =
        (largest_accepted(anchor_distance, anchor_decel * (1.0 + step), anchor_delay) -
         largest_accepted(anchor_distance, anchor_decel * (1.0 - step), anchor_delay)) /
        (2.0 * step * anchor_decel);
    const double dv_delay =
        (largest_accepted(anchor_distance, anchor_decel, anchor_delay * (1.0 + step)) -
         largest_accepted(anchor_distance, anchor_decel, anchor_delay * (1.0 - step))) /
        (2.0 * step * anchor_delay);
    const double denominator = anchor + anchor_decel * anchor_delay;
    const double predicted_d = anchor_decel / denominator;
    const double predicted_a = (anchor * anchor) / (2.0 * anchor_decel * denominator);
    const double predicted_delay = -(anchor_decel * anchor) / denominator;
    std::printf("[sensitivity] wall %.4f (pred %.4f), decel %.4e (pred %.4e), delay %.2f "
                "(pred %.2f)\n",
                dv_dd, predicted_d, dv_da, predicted_a, dv_delay, predicted_delay);
    EXPECT_NEAR(predicted_d, dv_dd, 1.0e-3 * predicted_d);
    EXPECT_NEAR(predicted_a, dv_da, 1.0e-3 * predicted_a);
    EXPECT_NEAR(predicted_delay, dv_delay, 1.0e-3 * std::fabs(predicted_delay));

    // Teeth: the delay-free reading of the same invariant is `sqrt(2*a*d)`, and at the
    // millisecond scale it is only a few percent away -- which is exactly why it survived. At the
    // ten-millisecond case the two are far apart, so that is the point that can tell them apart.
    // 牙齿：同一守恒式的不带延迟读法是 `sqrt(2·a·d)`，在毫秒量级上它只差几个百分点——这正是它
    // 一直活着的原因。在十毫秒那一组两者离得很远，所以那才是能分辨它们的点。
    const double delay_free_anchor = std::sqrt(2.0 * anchor_decel * anchor_distance);
    const double delay_free_slow = std::sqrt(2.0 * 1500.0 * 0.3333);
    std::printf("[teeth] delay-free reading: %.4f vs %.4f at 1 ms (%.1f %%), %.4f vs %.4f at 10 ms "
                "(%.1f %%)\n",
                delay_free_anchor, anchor,
                100.0 * std::fabs(delay_free_anchor - anchor) / anchor, delay_free_slow,
                largest_accepted(0.3333, 1500.0, 1.0e-2),
                100.0 * std::fabs(delay_free_slow - largest_accepted(0.3333, 1500.0, 1.0e-2)) /
                    largest_accepted(0.3333, 1500.0, 1.0e-2));
    EXPECT_GT(std::fabs(delay_free_slow - largest_accepted(0.3333, 1500.0, 1.0e-2)) /
                  largest_accepted(0.3333, 1500.0, 1.0e-2),
              0.4)
        << "the 10 ms case has to be able to exclude the delay-free reading";
}



namespace {

struct Approach final {
    double closest_approach; ///< [en] signed: negative means it crossed the wall. [zh] 带符号：负值表示越过了墙。
    /**
     * @brief [en] distance at which the cap first shrank the command.
     * @brief [zh] 上限第一次缩小指令时的距离。
     */
    double bite_distance;
};

/**
 * @brief [en] Drive at a fixed command straight at the wall, with and without the delay term.
 * @brief [zh] 以固定指令直冲墙面，分别带与不带延迟项。
 */
auto approach(double command, double decel, double delay, bool shaped) -> Approach {
    const foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    const float psi_f = static_cast<float>(params.kt / (1.5F * params.pole_pairs));
    foc::core::SoftLimit limit(window());
    foc::sil::PmsmPlant plant(params);
    plant.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, -0.400F});

    Approach outcome{1.0e9, 0.0};
    bool bit = false;
    for (int i = 0; i < 8000; ++i) {
        const foc::sil::PlantState now = plant.state();
        const float position_rad = static_cast<float>(now.theta_mech);
        const float distance = kMaxRad - position_rad;
        const float velocity =
            shaped ? limit.shape(static_cast<float>(command), position_rad,
                                 static_cast<float>(decel), static_cast<float>(delay))
                   : static_cast<float>(command);
        if (shaped && !bit && velocity < static_cast<float>(command) * (1.0F - 1.0e-6F)) {
            bit = true;
            outcome.bite_distance = static_cast<double>(distance);
        }
        outcome.closest_approach = std::min(outcome.closest_approach, static_cast<double>(distance));
        const float iq = std::max(-3.0F, std::min(3.0F, 5.0F * (velocity - static_cast<float>(now.omega_mech))));
        const float omega_e = static_cast<float>(now.omega_mech) * static_cast<float>(params.pole_pairs);
        plant.set_voltage_dq(foc::core::Dq<float>{-(omega_e * params.lq * iq),
                                                  (params.rs * iq) + (omega_e * psi_f)});
        plant.advance(kDt);
    }
    return outcome;
}

} // namespace



/**
 * @brief [en] On the plant: the delay term shifts the closest approach by exactly the delay travel, and the cap starts
 *             biting exactly at `v^2/(2a) + v*delay`.
 *
 * @brief [zh] 在 plant 上：延迟项把最近接近点推开恰好等于延迟行程的距离，而上限恰好在 `v²/(2a) + v·延迟` 处开始咬合。
 *
 *   The existing case measures one speed, twenty radians per second, and it proves the delay term is load-bearing. It
 *   does not say *how much* it buys, which is the number a user needs, and a single speed cannot say it because every
 *   candidate law fits one point. Two laws are being separated here, and both are statements about the drive, not about
 *   the shaper:
 *
 *   1. the cap begins to bite at `d = v^2/(2a) + v*delay` -- the inverse of the fixed point, so measuring it is
 *   measuring the seam in space rather than in speed;
 *
 *   2. the delay term shifts the closest approach forward by exactly `v*delay` -- the distance the axis covers,
 *   unavoidably, before the braking current arrives. Its gain is therefore **linear in the approach speed**, which is
 *   why it is worth almost nothing at 5 rad/s (4.6 mm) and everything at 20 (20 mm).
 *
 *   Two runs per speed at three speeds. The second law is what makes the first one checkable: it says the difference
 *   between the two closest approaches is the delay travel, and that difference can be measured to four digits because
 *   both runs share the same plant.
 *
 *   已有用例只测了一个速度，二十弧度每秒，它证明延迟项是承重的。它没说 **换来了多少** ，而那才是使用方要的数；而单个速
 *   度说不出来，因为任何候选律都能过一点。这里分辨两条律，它们都是关于驱动的、不是关于整形器的：
 *
 *   1. 上限在 `d = v²/(2a) + v·延迟` 处开始咬合——这是不动点的反函数，量它就是在空间里量接缝，而不是在速度里；
 *
 *   2. 延迟项把最近接近点推开恰好 `v·延迟` ——制动电流到位之前轴不可避免要走的距离。它的收益因此 **与接近速度成正比** ，
 *   这就是它在 5 rad/s 处几乎一文不值（4.6 mm）、在 20 处是全部的原因。
 *
 *   三个速度、每个速度两趟。第二条律才是让第一条可检的东西：它说两个最近接近点之差就是延迟行程，而这个差能测到四位有效
 *   数字，因为两趟共用同一个 plant。
 */
TEST(SoftLimit, TheDelayTermShiftsTheClosestApproachByExactlyTheDelayTravel) {
    const foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    const double decel = static_cast<double>(params.kt * 3.0F / params.j);
    const double delay = 1.0e-3;

    std::printf("\n%8s %14s %14s %14s %14s %10s\n", "v", "bite distance", "v^2/2a+v*tau",
                "shaped close", "ideal close", "shift/limit");
    for (const double command : {20.0, 10.0, 5.0}) {
        const Approach shaped = approach(command, decel, delay, true);
        const Approach ideal = approach(command, decel, 0.0, true);
        const double predicted_bite = command * command / (2.0 * decel) + command * delay;
        const double shift = shaped.closest_approach - ideal.closest_approach;
        const double predicted_shift = command * delay;
        std::printf("%8.1f %14.5f %14.5f %14.5f %14.5f %10.4f\n", command, shaped.bite_distance,
                    predicted_bite, shaped.closest_approach, ideal.closest_approach,
                    shift / predicted_shift);

        // (1) The cap starts biting where the inverse of the fixed point puts it.
        EXPECT_NEAR(predicted_bite, shaped.bite_distance, 0.03 * predicted_bite)
            << "the cap has to engage at v^2/(2a) + v*delay";

        // (2) The shift is the delay travel, and this is the law the row is promoted on.
        EXPECT_NEAR(predicted_shift, shift, 0.02 * predicted_shift)
            << "one delay travel is what the term buys, at every speed";

        // The two runs must disagree about where the wall is, or the delay term is decorative.
        EXPECT_GT(shaped.closest_approach, 0.0) << "the shaped run has to stay inside the window";
        EXPECT_LT(ideal.closest_approach, 0.0) << "the idealised ceiling has to cross it";
    }
}

/**
 * @brief [en] A window that is not an interval is refused in every way it can fail to be one, and the accessors report
 *             the window that is actually in force.
 *
 * @brief [zh] 不是一个区间的窗口，在它失败的每一种方式上都被拒绝；而访问器报出的是真正生效的那个窗口。
 *
 *   The three failures are separate arithmetic accidents -- reversed, and non-finite on either end -- and the
 *   non-finite ones are the ones that matter downstream, because `NaN` compares false against everything: a
 *   `min <= target <= max` test written against a `NaN` wall refuses every target for a reason nothing reports. The
 *   accessor half is the other direction: `set_config()` claims to replace the window, and the only way to see that it
 *   did is to ask for the window back and to watch the new one start refusing.
 *
 *   这三种失败是三起彼此独立的算术事故——反了、以及任一端非有限——而其中非有限的那几个才是下游真正要紧的，因为 `NaN` 与任
 *   何东西比较都为假：用一面 `NaN` 的墙去写 `min <= target <= max `，会以一件没人报出的事情为由拒绝每一个目标。访问器那
 *   一半是另一个方向： `set_config()` 声称换掉了窗口，而要看它真换了，唯一的办法就是把窗口要回来，再看新窗口开始拒绝。
 */
TEST(SoftLimit, AWindowThatIsNotAnIntervalIsRefusedHoweverItIsBroken) {
    const float inf = std::numeric_limits<float>::infinity();
    const float nan_v = std::numeric_limits<float>::quiet_NaN();
    const foc::core::SoftLimitConfig broken[] = {
        foc::core::SoftLimitConfig{1.0F, -1.0F},      // reversed
        foc::core::SoftLimitConfig{nan_v, 0.5F},      // low end not a number
        foc::core::SoftLimitConfig{-0.5F, nan_v},     // high end not a number
        foc::core::SoftLimitConfig{-inf, inf},        // unbounded: not a window at all
    };
    for (const foc::core::SoftLimitConfig& window_in : broken) {
        foc::core::SoftLimit limit(window_in);
        EXPECT_EQ(limit.validate(), foc::ErrorCode::Param);
        // And the refusal reaches the caller as a status, not as a clamped target: the remembered target is
        // still the initial zero because nothing was ever admitted.
        const foc::core::SoftLimitOutput out = limit.apply(0.0F);
        EXPECT_EQ(out.status, foc::ErrorCode::Param);
        EXPECT_FLOAT_EQ(out.target_rad, 0.0F);
        EXPECT_FALSE(out.at_wall);
    }
    EXPECT_EQ(foc::core::SoftLimit(window()).validate(), foc::ErrorCode::Ok);
}

TEST(SoftLimit, SetConfigReplacesTheWindowTheLimitActuallyEnforces) {
    foc::core::SoftLimit limit(window());
    EXPECT_FLOAT_EQ(limit.config().min_rad, kMinRad);
    EXPECT_FLOAT_EQ(limit.config().max_rad, kMaxRad);

    limit.set_config(foc::core::SoftLimitConfig{-0.25F, 0.25F});
    EXPECT_FLOAT_EQ(limit.config().min_rad, -0.25F);
    EXPECT_FLOAT_EQ(limit.config().max_rad, 0.25F);

    // The point of asking: the new window is the one that decides. A target the old window admitted and the
    // new one does not must now be refused, and one inside the new window must be admitted.
    // 追问的意义：新窗口才是做决定的那个。旧窗口接纳、新窗口不接纳的目标此刻必须被拒绝，而新窗口内的目标必须被
    // 接纳。
    EXPECT_EQ(limit.apply(0.4F).status, foc::ErrorCode::Param);
    EXPECT_EQ(limit.apply(0.2F).status, foc::ErrorCode::Ok);
    EXPECT_FLOAT_EQ(limit.target_rad(), 0.2F);
}

TEST(SoftLimit, ResetForgetsTheTargetSoTheFirstTargetRuleAppliesAgain) {
    foc::core::SoftLimit limit(window());
    ASSERT_EQ(limit.apply(0.2F).status, foc::ErrorCode::Ok);
    EXPECT_FLOAT_EQ(limit.target_rad(), 0.2F);

    // The refused target carries the remembered one back to the caller -- that echo is the whole difference
    // between "refused" and "silently kept", so it is the observation that says whether the reset happened.
    // 被拒绝的目标会把记住的那个带回给调用方——这个回声正是"被拒绝"与"被悄悄保留"的全部区别，所以它才是能说明
    // 复位究竟发生了没有的那个观测。
    EXPECT_FLOAT_EQ(limit.apply(0.9F).target_rad, 0.2F);

    limit.reset();
    EXPECT_FLOAT_EQ(limit.target_rad(), 0.0F);
    // Same out-of-window target, and now there is nothing remembered to echo: the reset forgot it and the
    // first-target rule is in force again, so 0.2 is not coming back.
    // 同一个越界目标，而此刻没有可回声的记忆：复位把它忘掉了、首个目标规则重新生效，于是 0.2 不会再回来。
    const foc::core::SoftLimitOutput after_reset = limit.apply(0.9F);
    EXPECT_EQ(after_reset.status, foc::ErrorCode::Param);
    EXPECT_FLOAT_EQ(after_reset.target_rad, 0.0F);
    // A target inside the window is a legal *first* target again, which is what "the limit can be re-armed" means.
    // 窗口内的目标重新成为一个合法的 **首个** 目标，这正是"限位可以重新上膛"的意思。
    EXPECT_EQ(limit.apply(0.2F).status, foc::ErrorCode::Ok);
    EXPECT_FLOAT_EQ(limit.target_rad(), 0.2F);
}

/**
 * @brief [en] The cap is symmetric in the direction of travel, and a zero command is never rewritten.
 *
 * @brief [zh] 上限对行进方向是对称的，而零指令永远不被改写。
 *
 *   The header states the cap in terms of `|v|`, so a reader is entitled to expect the low wall to behave like the high
 *   one mirrored. Until this case the negative direction had never been driven, which means half of the claim "the axis
 *   can always stop before the wall it is heading for" was untested -- the low wall is the wall a negative coordinate
 *   moves toward, and a test that only ever drives `+` cannot see a sign error there.
 *
 *   The distances used are exact: -0.5 + 0.0625 is representable, so `position - min` is exactly 0.0625 and both sides
 *   of the comparison compute `sqrt` of the same float. That is why the equality below is bit-exact and not a tolerance
 *   -- the comparison is of two computations of one number, not of a measurement.
 *
 *   头文件是用 `|v|` 陈述这条上限的，所以读者有权期望低墙表现得像高墙的镜像。在这个用例之前，负方向从未被驱动过，这意味
 *   着"轴总能在它正驶向的那面墙之前停住"这条主张有一半未被测——低墙正是负坐标移动过去的那面墙，而一个永远只驱动 `+` 的测
 *   试看不出那里的符号错误。
 *
 *   所用的距离是精确的：-0.5 + 0.0625 可精确表示，于是 `position - min` 恰好是 0.0625，比较两边算的是同一个 float 的
 *   `sqrt`。这就是下面的等号逐位成立、而不是一个容差的原因——被比较的是一个数的两次计算，不是一次测量。
 */
TEST(SoftLimit, TheCapIsSymmetricInTheDirectionOfTravel) {
    constexpr float kDecel = 1500.0F;
    constexpr float kDistance = 0.0625F;
    const foc::core::SoftLimit limit(window());

    // Far from either wall the command passes through untouched, in both directions.
    // 离两面墙都远时指令原样通过，两个方向都是。
    EXPECT_FLOAT_EQ(limit.shape(5.0F, 0.0F, kDecel, 0.0F), 5.0F);
    EXPECT_FLOAT_EQ(limit.shape(-5.0F, 0.0F, kDecel, 0.0F), -5.0F);

    // At the same distance from each wall the same magnitude comes back, with the sign of the command.
    // The command is well above the ceiling on purpose: below it the cap returns the caller's float unchanged
    // (that is the earlier half of this case), so a smaller command would prove nothing about the sign.
    // 离每面墙同样的距离处，回来的幅值相同，符号跟着指令。指令故意远高于上限：低于上限时上限原样放过调用方的
    // float（那是本用例的上半段），所以一个更小的指令对符号什么也证明不了。
    const float ceiling = std::sqrt(2.0F * kDecel * kDistance);
    EXPECT_FLOAT_EQ(limit.shape(40.0F, kMaxRad - kDistance, kDecel, 0.0F), ceiling);
    EXPECT_FLOAT_EQ(limit.shape(-40.0F, kMinRad + kDistance, kDecel, 0.0F), -ceiling);

    // A command toward a wall with no distance left is removed entirely rather than reversed.
    // 面对一面已经没有距离的墙，指令被整个去掉，而不是反号。
    EXPECT_FLOAT_EQ(limit.shape(-5.0F, kMinRad, kDecel, 0.0F), 0.0F);
    EXPECT_FLOAT_EQ(limit.shape(5.0F, kMaxRad, kDecel, 0.0F), 0.0F);

    // And zero stays zero: a stopped axis is not offered a speed by its own limit.
    // 而零仍是零：停住的轴不会被它自己的限位推出一份速度。
    EXPECT_FLOAT_EQ(limit.shape(0.0F, 0.4F, kDecel, 0.0F), 0.0F);
    EXPECT_FLOAT_EQ(limit.shape(0.0F, -0.4F, kDecel, 0.0F), 0.0F);
}


