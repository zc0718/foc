// The physics 3.1 needs, with its own criteria -- written before any decoder exists, because a Hall decoder tested
// against a live angle (or against a code that never goes illegal) is tested against a sensor nobody sells. What has
// to be true first: six sectors of exactly 60 degrees, one bit per edge, the quantisation that follows from it, and
// the ways a real sensor set degrades -- misplacement, and a dead sensor.
// 3.1 需要的物理，连同它自己的判据——在任何解码器存在之前写，因为用"活的角度"（或永不变非法的编码）去测一个霍尔
// 解码器，等于对着一台没人卖的传感器测。必须先成立的事实：六个恰好 60° 的扇区、每个边沿只翻一位、由此而来的量化、
// 以及真实传感器组的两种退化——装偏，和一路坏掉。

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

#include <gtest/gtest.h>

#include "sil_hall_sensor.hpp"

namespace {

constexpr float kStep = 1.0e-4F;          // rad; 0.0057 deg, so a 60 deg sector is 10471 samples
constexpr float kSectorRad = 1.04719755119659774615F; // 60 deg

/**
 * @brief One stretch of angles over which the code does not change.
 */
struct HallRun final {
    std::uint8_t code;
    float start_rad;
    float width_rad;
};

/**
 * @brief Walk one electrical revolution and cut it where the code changes.
 *
 *     The first and last runs are the same run whenever the sweep crosses an edge at 0, so they are merged; without
 *     that the count depends on where the sweep happened to start, which is a property of the test, not of the sensor.
 *     走一圈电角度，在编码变化处切开。只要扫过的区间在 0 处跨过一个边沿，首尾两段就是同一段，因此合并；不合并的话，段数
 *     取决于扫描从哪里开始，那是测试的性质，不是传感器的性质。
 */
auto scan(const foc::sil::HallSensorModel &model) -> std::vector<HallRun> {
    // One revolution, not two: `kTwoPiRad` is already 2*pi. The first version multiplied it by 2 again and swept 720
    // degrees, which turned six runs into twelve and made every count in this file read double -- caught by the case
    // that asserts "six edges", which is why that assertion is phrased as an absolute number.
    // 一圈，不是两圈：`kTwoPiRad` 本身就是 2π。第一版又乘了一次 2，扫了 720°，把六段变成十二段、让本文件每个计数都
    // 翻倍——抓住它的是那条断言"六个边沿"的用例，这也正是那条断言写成绝对数字的原因。
    const int samples = static_cast<int>(foc::sil::kTwoPiRad / kStep);
    std::vector<HallRun> runs;
    std::uint8_t previous = 0xFFU;
    for (int i = 0; i < samples; ++i) {
        const float theta = static_cast<float>(i) * kStep;
        const std::uint8_t code = model.code(theta);
        if (code != previous) {
            runs.push_back(HallRun{code, theta, kStep});
            previous = code;
        } else {
            runs.back().width_rad += kStep;
        }
    }
    if (runs.size() > 1U && runs.front().code == runs.back().code) {
        const float tail_start = runs.back().start_rad - foc::sil::kTwoPiRad;
        runs.front().width_rad += runs.back().width_rad;
        runs.front().start_rad = tail_start;
        runs.pop_back();
    }
    return runs;
}

/**
 * @brief How many bits change between two codes. A healthy three-Hall set never changes more than one at a time. 两个编码之间翻了几位。健康的三霍尔组任何时刻最多只翻一位。
 */
auto bit_flips(std::uint8_t from, std::uint8_t to) -> int {
    auto x = static_cast<unsigned>(from ^ to);
    int count = 0;
    while (x != 0U) {
        count += static_cast<int>(x & 1U);
        x >>= 1U;
    }
    return count;
}

auto sorted_widths_deg(const std::vector<HallRun> &runs) -> std::vector<float> {
    std::vector<float> widths;
    widths.reserve(runs.size());
    for (const HallRun &run : runs) {
        widths.push_back(run.width_rad * (180.0F / 3.14159265358979323846F));
    }
    std::sort(widths.begin(), widths.end());
    return widths;
}

auto count_illegal(const std::vector<HallRun> &runs) -> int {
    int count = 0;
    for (const HallRun &run : runs) {
        if (!foc::sil::HallSensorModel::legal(run.code)) {
            ++count;
        }
    }
    return count;
}

} // namespace

/**
 * @brief A healthy set of sensors divides one electrical revolution into exactly six arcs of 60 degrees, in the order 1,3,2,6,4,5, and produces no code that a three-bit field has left over.
 *
 *     This is the fact everything downstream spends: 60 degrees is the information a Hall set carries per edge, and the
 *     two surplus codes (0 and 7) are the only fault signal the wiring itself provides. A model with the sensors 90
 *     degrees apart, with two sensors, or with a bit order rotated would fail here and nowhere else. 健康的一组传感器把
 *     一圈电角度切成恰好六段 60° 的弧，顺序是 1,3,2,6,4,5，且不产生三位字段剩下的任何编码。这是下游一切所花的那条事实：
 *     60° 就是霍尔每过一个边沿携带的信息，而两个多余编码（0 与 7）是接线本身提供的唯一故障信号。把传感器做成相隔 90°、
 *     只做两个、或位序转了一格的模型，会在这里、且只在这里失败。
 */
TEST(HallPhysics, AHealthySetDividesTheRevolutionIntoSixSixtyDegreeSectors) {
    const foc::sil::HallSensorModel model{};
    const std::vector<HallRun> runs = scan(model);

    ASSERT_EQ(runs.size(), 6U) << "a three-Hall set has six edges per electrical revolution";
    EXPECT_EQ(count_illegal(runs), 0) << "a healthy set cannot produce 0 or 7";

    std::array<int, 8> seen{};
    for (const HallRun &run : runs) {
        ++seen[run.code];
        EXPECT_NEAR(run.width_rad, kSectorRad, 2.0F * kStep)
            << "sector " << static_cast<int>(run.code) << " is " << (run.width_rad * 57.29578F) << " deg";
    }
    // Each of the six reachable codes appears exactly once, in the physical order of the rotor.
    for (int code = 1; code <= 6; ++code) {
        EXPECT_EQ(seen[static_cast<std::size_t>(code)], 1) << "code " << code;
    }
    EXPECT_EQ(seen[0], 0);
    EXPECT_EQ(seen[7], 0);

    // The order is the rotor's, not an arbitrary permutation: consecutive runs step through the codes in the
    // sequence a real machine produces.
    const std::array<std::uint8_t, 6> expected{1U, 3U, 2U, 6U, 4U, 5U}; // starting at theta = 0
    for (std::size_t i = 0; i < 6U; ++i) {
        EXPECT_EQ(runs[i].code, expected[i]) << "run " << i;
    }
}

/**
 * @brief Every edge is a single bit, and that is what makes "two bits at once" a fault signal.
 *
 *     The six legal codes are the six edges of a cube, and the rotor walks around one face of it: at every transition
 *     exactly one sensor changed. This property is what a decoder can check without knowing the angle, so it is claimed
 *     here as a property of the sensor set, not of the decoder. 每个边沿只翻一位，这正是"一次翻两位"能当故障信号用的原
 *     因。六个合法编码是立方体的六个顶点，转子绕着其中一个面走：每次跳变恰好一个传感器变了。这条性质让解码器不靠角度就
 *     能校验，因此在这里作为传感器组的性质声称，而不是解码器的。
 */
TEST(HallPhysics, EveryEdgeFlipsExactlyOneBitSoTwoFlipsAtOnceIsAFault) {
    const foc::sil::HallSensorModel model{};
    const std::vector<HallRun> runs = scan(model);
    ASSERT_EQ(runs.size(), 6U);

    for (std::size_t i = 0; i < runs.size(); ++i) {
        const std::uint8_t from = runs[i].code;
        const std::uint8_t to = runs[(i + 1U) % runs.size()].code;
        EXPECT_EQ(bit_flips(from, to), 1) << "edge " << static_cast<int>(from) << " -> " << static_cast<int>(to);
    }
}

/**
 * @brief Inside one sector the code says nothing at all: the rotor may be anywhere in 60 degrees, so a decoder that sees only the code is wrong by up to 30 degrees, and no cleverness can improve that.
 *
 *     The claim is a pair of measurements, not a slogan: (a) two angles 59.9 degrees apart produce the *same* code, (b)
 *     across the boundary 0.1 degrees apart they produce *different* codes. Together they say the quantisation is 60
 *     degrees wide and exactly 60 degrees -- not wider (the code does change), not narrower (it does not change for
 *     less). A model that returned a fine angle by accident would fail (a); one that was stuck would fail (b). 一个扇区
 *     内编码什么也不说：转子可以在 60° 内任何位置，所以只看编码的解码器最多错 30°，再聪明也改不了。判据是两条测量，不是
 *     口号：(a) 相隔 59.9° 的两个角度给出**同一个**编码；(b) 跨过边界相隔 0.1° 给出**不同**编码。合起来说明量化宽度是
 *     60° 且正好 60°——不会更宽（编码确实变了），不会更窄（更小的间隔不会变）。一个不小心返回了细角度的模型会挂在 (a)，
 *     一个卡住的模型会挂在 (b)。
 */
TEST(HallPhysics, InsideOneSectorTheCodeCarriesNoAngleInformation) {
    const foc::sil::HallSensorModel model{};
    const std::vector<HallRun> runs = scan(model);
    ASSERT_EQ(runs.size(), 6U);

    // Read the sector out of the sweep instead of assuming where it starts. The sensors sit at 0/120/240 degrees, so
    // the edges land at 30/90/150/... and *no* sector begins at zero; the first version of this case assumed a
    // sector 0..60 and straddled an edge, which is a claim about the arrangement dressed up as a claim about the
    // sensors.
    // 从扫描结果里读出扇区，而不是假设它从哪开始。传感器在 0/120/240°，因此边沿落在 30/90/150/…，**没有**哪个扇区
    // 从零开始；本用例第一版假设了 0..60 的扇区，结果跨过了一条边沿——那是把关于排布的说法打扮成关于传感器的说法。
    const HallRun *widest = &runs.front();
    for (const HallRun &run : runs) {
        if (run.width_rad > widest->width_rad) {
            widest = &run;
        }
    }
    const float margin = 0.02F; // rad inside each edge, so the case is about the interior and not about the boundary
    const float inside_a = widest->start_rad + margin;
    const float inside_b = widest->start_rad + widest->width_rad - margin;
    EXPECT_EQ(model.code(inside_a), model.code(inside_b)) << "59.96 deg apart, same sector, same code";

    // And the boundary is exactly where the sector ends, not somewhere inside it.
    EXPECT_NE(model.code(widest->start_rad - 0.001F), model.code(inside_a)) << "just before the edge";
    EXPECT_NE(model.code(widest->start_rad + widest->width_rad + 0.001F), model.code(inside_b)) << "just after it";

    // Worst case a code-only decoder can do: half the sector, and the sector is measured rather than assumed.
    const float worst_case_deg = (widest->width_rad * 0.5F) * (180.0F / 3.14159265358979323846F);
    EXPECT_NEAR(worst_case_deg, 30.0F, 0.01F) << "half of " << (widest->width_rad * 57.29578F) << " deg";
}

/**
 * @brief Ten degrees of misplacement on one sensor widens two sectors to 70 degrees and narrows two to 50, and -- the part that matters -- keeps every code legal, so no illegal-state check will ever see it.
 *
 *     A misplaced sensor does not lie about *which* bits are on, only about *when* they change, so the code sequence
 *     stays a legal Gray walk with the edges moved. The angle error it causes is therefore bounded and invisible to the
 *     wiring, which is why it has to be found by measuring the sector widths (or by a once-per-revolution index), not
 *     by looking for illegal codes. This case is the reason that sentence is in the documentation. 一个传感器装偏 10°，
 *     会把两个扇区撑到 70°、把两个压到 50°，并且——要紧的是——让每个编码仍然合法，所以任何非法状态检查都看不见它。装偏的
 *     传感器没有谎报**哪些位**是开的，只谎报了**什么时候**变，因此编码序列仍是合法的格雷行走，只是边沿挪了。它造成的角
 *     度误差因此有限、且对连线不可见——这就是为什么必须靠测量扇区宽度（或每转一次的索引）来发现它，而不能靠找非法编码。
 *     用例名的这句话正是这条用例存在的理由。
 */
TEST(HallPhysics, TenDegreesOfMisplacementWidensTwoSectorsAndStaysLegalThroughout) {
    foc::sil::HallSetup setup{};
    setup.mount_error_rad[0] = 10.0F * (3.14159265358979323846F / 180.0F); // 10 deg, direction of rotation
    const foc::sil::HallSensorModel model{setup};
    const std::vector<HallRun> runs = scan(model);

    ASSERT_EQ(runs.size(), 6U) << "the sensor still owns two edges, so there are still six of them";
    EXPECT_EQ(count_illegal(runs), 0) << "misplacement never produces 0 or 7 -- that is what makes it invisible";

    // Measured widths: two sectors of 70, two of 60, two of 50. Ten degrees added to one sensor moves the two
    // boundaries it owns: each of the two adjacent sectors loses 10 on one side and gains 10 on the other.
    // 实测宽度：两个 70、两个 60、两个 50。给一个传感器加 10°，就把它拥有的两条边界都挪了：相邻的两个扇区各在一侧
    // 少 10、另一侧多 10。
    const std::vector<float> widths = sorted_widths_deg(runs);
    ASSERT_EQ(widths.size(), 6U);
    EXPECT_NEAR(widths[0], 50.0F, 0.02F);
    EXPECT_NEAR(widths[1], 50.0F, 0.02F);
    EXPECT_NEAR(widths[2], 60.0F, 0.02F);
    EXPECT_NEAR(widths[3], 60.0F, 0.02F);
    EXPECT_NEAR(widths[4], 70.0F, 0.02F);
    EXPECT_NEAR(widths[5], 70.0F, 0.02F);

    // Consequence for a code-only decoder: its worst-case error grows from 30 to 35 degrees, on the wide sectors.
    EXPECT_NEAR(widths[5] * 0.5F, 35.0F, 0.02F);

    // And the Gray property still holds: one bit per edge, always.
    for (std::size_t i = 0; i < runs.size(); ++i) {
        const std::uint8_t from = runs[i].code;
        const std::uint8_t to = runs[(i + 1U) % runs.size()].code;
        EXPECT_EQ(bit_flips(from, to), 1) << "edge " << i;
    }
}

/**
 * @brief One dead sensor is unmissable for one sixth of a revolution and looks perfectly healthy for the other five sixths -- at 120-degree resolution. And the Gray check does **not** catch it.
 *
 *     This is the number that decides how a Hall fault must be handled. A stuck sensor cannot change its bit, so the
 *     five sectors it should have helped split keep producing legal codes; the angular information silently drops from
 *     60 to 120 degrees, and the edge interval doubles. The wiring shows the fault only where the other two sensors
 *     disagree with the dead one -- exactly one 60-degree window per revolution, i.e. one sixth of the time.
 *
 *     The first version of this case asserted the opposite of what it now measures: it claimed that a dead sensor makes
 *     two bits flip at once (6 -> 4), and that the adjacency check would therefore catch it. It does not. The dead bit
 *     is *frozen*, so every surviving transition still flips exactly one bit -- the walk degenerates onto a smaller
 *     Gray cycle rather than breaking the rule. The measurement is asserted below as a fact, because a decoder that
 *     checked only adjacency would believe a dead sensor to its grave: the channels that do see it are the illegal
 *     window (one sixth of the time) and the doubled edge interval (all of the time). 一路传感器坏掉，在一圈里有六分之
 *     一的时间无处可藏，其余六分之五看起来完全健康——只是分辨率掉到 120°；而格雷校验 **抓不到**它。这个数决定了霍尔故障
 *     必须怎么处理。卡死的传感器无法改变自己那一位，所以它本该参与切分的五个扇区继续产生合法编码；角度信息悄悄从 60° 掉
 *     到 120°，边沿间隔翻倍。接线只在另两路与坏的那路不一致处暴露故障——每圈恰好一个 60° 窗口，也就是六分之一的时间。本
 *     用例的第一版断言与现在测到的相反：它声称坏一路会让两位同时翻转（6 -> 4），因此相邻性校验能抓住它。抓不住。坏掉的
 *     那一位是**冻结**的，所以每次幸存下来的跳变仍然只翻一位——行走退化到一个更小的格雷环上，而不是破坏了规则。下面把这
 *     次测量当作事实断言，因为只查相邻性的解码器会把一路坏掉的传感器信到底：真正看得见它的通道是非法窗口（六分之一的时
 *     间）和翻倍的边沿间隔（全部时间）。
 */
TEST(HallPhysics, ADeadSensorIsSilentForFiveSixthsOfTheRevolutionAndKeepsEveryRemainingCodeLegal) {
    for (const bool stuck_high : {false, true}) {
        foc::sil::HallSetup setup{};
        setup.stuck_sensor = 0U;
        setup.stuck_high = stuck_high;
        const foc::sil::HallSensorModel model{setup};
        const std::vector<HallRun> runs = scan(model);

        SCOPED_TRACE(stuck_high ? "sensor 0 stuck high" : "sensor 0 stuck low");

        // Four edges left, so four runs -- and two of them span two nominal sectors.
        ASSERT_EQ(runs.size(), 4U);
        const std::vector<float> widths = sorted_widths_deg(runs);
        EXPECT_NEAR(widths[0], 60.0F, 0.02F);
        EXPECT_NEAR(widths[1], 60.0F, 0.02F);
        EXPECT_NEAR(widths[2], 120.0F, 0.02F);
        EXPECT_NEAR(widths[3], 120.0F, 0.02F);

        // Exactly one sixth of the revolution is an illegal code. That is the entire visible fault.
        ASSERT_EQ(count_illegal(runs), 1);
        float illegal_deg = 0.0F;
        for (const HallRun &run : runs) {
            if (!foc::sil::HallSensorModel::legal(run.code)) {
                illegal_deg = run.width_rad * (180.0F / 3.14159265358979323846F);
            }
        }
        EXPECT_NEAR(illegal_deg, 60.0F, 0.02F) << "one sector out of six is the whole blind spot complement";

        // Three legal codes are all that is left to name angular position with.
        std::array<int, 8> seen{};
        for (const HallRun &run : runs) {
            ++seen[run.code];
        }
        int distinct_legal = 0;
        for (int code = 1; code <= 6; ++code) {
            if (seen[static_cast<std::size_t>(code)] != 0) {
                ++distinct_legal;
            }
        }
        EXPECT_EQ(distinct_legal, 3) << "six sectors collapse to three distinguishable ones";

        // And the sequence does **not** betray it: a frozen bit keeps every surviving transition Gray-adjacent, so
        // the adjacency rule every decoder carries is satisfied all the way round. This is the assertion that costs
        // a designer their afternoon if they assumed otherwise.
        // 而序列**不会**揭发它：冻结的那一位让每次幸存跳变都保持格雷相邻，于是每个解码器都带着的那条相邻规则，在整圈里
        // 都被满足。这条断言正是"假设相反"会让设计者赔上一下午的地方。
        int flips_seen[4] = {0, 0, 0, 0};
        for (std::size_t i = 0; i < runs.size(); ++i) {
            const int flips = bit_flips(runs[i].code, runs[(i + 1U) % runs.size()].code);
            ++flips_seen[static_cast<std::size_t>(flips)];
        }
        EXPECT_EQ(flips_seen[1], 4) << "every surviving edge still flips exactly one bit";
        EXPECT_EQ(flips_seen[2], 0) << "a dead sensor is invisible to the adjacency check";

        // What *does* see it, all the time: the edges come half as often, so the interval doubles.
        float shortest = 1.0e9F;
        float longest = 0.0F;
        for (const HallRun &run : runs) {
            if (foc::sil::HallSensorModel::legal(run.code)) {
                shortest = std::min(shortest, run.width_rad);
                longest = std::max(longest, run.width_rad);
            }
        }
        EXPECT_NEAR(longest / shortest, 2.0F, 0.01F) << "120 deg against 60 deg: the edge interval doubled";
    }
}

/**
 * @brief With a healthy set, no angle anywhere produces 0 or 7 -- so when the check fires, it means something.
 *
 *     The other half of an admission rule: a fault detector that fires on a healthy rig is a nuisance generator, and
 *     the cheapest way to keep that promise is to sweep the whole revolution at a step far finer than any control
 *     period. This is also where the boundary itself is checked: the model's comparison is `> 0`, and a sensor sitting
 *     exactly on its edge is a real place to be, so the sweep includes the exact edge angles. 准入规则的另一半：在健康
 *     的装置上就报警的故障检测器是个噪声发生器，而兑现这条承诺最便宜的办法是用远细于任何控制周期的步长扫完整圈。边沿本
 *     身也在这里检查：模型的比较是 `> 0`，而传感器恰好停在边沿上是个真实位置，所以扫描包含精确的边沿角。
 */
TEST(HallPhysics, AHealthySetNeverProducesAnIllegalCode) {
    const foc::sil::HallSensorModel model{};
    const int samples = static_cast<int>(foc::sil::kTwoPiRad / 1.0e-5F);
    int illegal = 0;
    for (int i = 0; i < samples; ++i) {
        if (!foc::sil::HallSensorModel::legal(model.code(static_cast<float>(i) * 1.0e-5F))) {
            ++illegal;
        }
    }
    EXPECT_EQ(illegal, 0) << samples << " angles swept";

    // Exactly on the edges, and a hair to either side.
    for (std::uint8_t sensor = 0; sensor < 3U; ++sensor) {
        const float on = model.switch_on_rad(sensor);
        for (const float offset : {0.0F, -1.0e-6F, 1.0e-6F}) {
            const float theta = on + offset;
            EXPECT_TRUE(foc::sil::HallSensorModel::legal(model.code(theta)))
                << "sensor " << static_cast<int>(sensor) << " at edge + " << offset;
        }
    }
}
