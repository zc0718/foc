// SOP (D-9): a capability that is a *procedure* rather than a formula, cut vertically end to end --
// identify -> propose -> measure -> held-out acceptance -> refusal semantics -- and replayed on a second machine.
// This file is the driver the plan asks for, and every number below was measured before the assertion that uses it.
// SOP（D-9）：一种 **流程** 而非公式的能力，竖直切到头——辨识 → 提议 → 测量 → 留出集验收 → 拒绝语义——并在第二台机器上重放。
// 本文件就是计划要的那个驱动，而下面每一个数都是先量出来、再写进断言的。

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/auto_tune.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"
#include "sil_sop_bench.hpp"

namespace {

using foc::ErrorCode;
using foc::core::SpeedTuneConfig;
using foc::core::SpeedTuner;
using foc::core::SpeedTuneResult;

/**
 * @brief [en] The bandwidth this procedure is run at: 250 rad/s mechanical, the F35 row's own design point.
 * @brief [zh] 本流程运行的带宽：250 rad/s 机械，F35 那一行自己的设计点。
 */
constexpr float kBandwidthRadS = 250.0F;

/**
 * @brief [en] The most the gain law itself costs, measured over a decade in `MotorConfig.TheInertiaEstimateIs...`
 *             (1.02516 / 1.02661 / 1.02948 / 1.03462 at 50 / 100 / 200 / 400 rad/s).
 * @brief [zh] 增益律自身的最大代价，在 `MotorConfig.TheInertiaEstimateIs...` 里跨十倍带宽测得（50 / 100 / 200 / 400
 *             rad/s 处 1.02516 / 1.02661 / 1.02948 / 1.03462）。
 */
constexpr double kLawResidualMax = 1.035;

/**
 * @brief [en] The smallest nameplate error this repo already calls actionable: a 20 % inertia error is a 20 % bandwidth
 *             error, measured one-for-one in the same row.
 * @brief [zh] 本仓库已经称为"可操作"的最小铭牌误差：20 % 的惯量误差就是一比一的 20 % 带宽误差，在同一行里测得。
 */
constexpr double kActionableError = 1.20;

/**
 * @brief [en] Upper edge of the acceptance band, **derived** from those two measurements rather than chosen: the
 *             geometric midpoint between the law's own residual and the smallest error worth acting on.
 * @brief [zh] 验收带的上边界， **由那两项测量推出** 而不是挑出来的：增益律自身残差与"值得动手"的最小误差之间的几何中
 *             点。
 * @return [en] `sqrt(1.035 * 1.20)` = 1.1143.
 * @return [zh] `sqrt(1.035·1.20)` = 1.1143。
 * @note [en] A band that is narrower than `kLawResidualMax` would fail the incumbent loop; one that is wider than
 *            `kActionableError` would pass a machine nobody described. Both alternatives are measurable, which is what
 *            makes the midpoint the only defensible choice here.
 * @note [zh] 比 `kLawResidualMax` 更窄的带会把在位环路判不合格；比 `kActionableError` 更宽的带会放过一台没人描述过的机
 *            器。两个反面都是可测的，这才让中点成为这里唯一站得住的选法。
 */
constexpr double kBandUpper = 1.1143;

/**
 * @brief [en] Distance, in log space, from a delivered ratio to the nearest edge of the band. Positive = inside.
 * @brief [zh] 交付比值到最近的带边界的距离，取对数空间。为正 = 在带内。
 * @param ratio [en] measured −3 dB over requested.
 * @param ratio [zh] 实测 −3 dB 与请求值之比。
 * @return [en] `ln(kBandUpper) - |ln(ratio)|`.
 * @return [zh] `ln(kBandUpper) − |ln(ratio)|`。
 * @note [en] Log space because a ratio's distance is symmetric: delivering half the request is as far from the band as
 *            delivering twice it, and a linear distance would call those two different mistakes.
 * @note [zh] 取对数空间是因为比值的距离是对称的：交付一半与交付两倍离带一样远，而线性距离会把这两件事说成不同程度的错。
 */
auto delivered_margin(double ratio) -> double { return std::log(kBandUpper) - std::fabs(std::log(ratio)); }

/** @brief [en] The nameplate the procedure is told about. @brief [zh] 流程被告知的那份铭牌。 */
auto sop_tune_config() -> SpeedTuneConfig {
    const auto spec = foc::sil::nominal_spec();
    SpeedTuneConfig config{};
    config.kt_nm_per_a = spec.kt;
    config.j_expected_kg_m2 = spec.j;
    config.b_expected_nm_s = spec.b;
    config.bandwidth_rad_s = kBandwidthRadS;
    config.i_max_a = spec.i_max_a;
    config.tolerance = 0.25F;
    config.min_current_a = 0.05F;
    config.max_residual = 0.35F;
    config.min_samples = 100;
    return config;
}

/**
 * @brief [en] **Stage 1, identify**: run a current staircase on a machine and feed the tuner what a drive would see.
 * @brief [zh] **第 1 步，辨识** ：在一台机器上跑一条电流阶梯，把驱动器会看到的东西喂给整定器。
 * @param j_scale [en] multiplier on the plant's inertia, so "a machine that is not the one you described" is a machine
 *                     that really exists.
 * @param j_scale [zh] 缩放 plant 的惯量，好让"不是你描述的那台机器"是一台真实存在的机器。
 * @param window_steps [en] how many current levels the window holds -- a **procedure** parameter, not a criterion.
 * @param window_steps [zh] 窗口里有多少级电流——这是 **过程** 参数，不是判据。
 * @param amplitude [en] scale on the current levels, to ask whether it moves the answer at all.
 * @param amplitude [zh] 电流级的缩放，用来问它到底动不动答案。
 * @return [en] the pass's own report.
 * @return [zh] 这次整定自己的交代。
 */
auto identify(float j_scale, int window_steps = 6, float amplitude = 1.0F) -> SpeedTuneResult {
    foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    params.j = params.j * j_scale;
    foc::sil::PmsmPlant plant{params};
    SpeedTuner tuner{sop_tune_config()};
    constexpr float kDt = 5.0e-5F;
    const float rs = params.rs;
    const float ld = params.ld;
    const float lq = params.lq;
    const float psi = plant.psi_f_wb();
    const std::vector<float> base{0.2F, 0.6F, 0.1F, 0.8F, 0.3F, 0.5F, 0.9F, 0.4F};
    for (int s = 0; s < window_steps; ++s) {
        const float reference = amplitude * base[static_cast<std::size_t>(s % static_cast<int>(base.size()))];
        for (int n = 0; n < 400; ++n) { // 20 ms per level 每级 20 ms
            const float omega_e = static_cast<float>(params.pole_pairs) * plant.omega_mech_rad_s();
            const auto state = plant.state();
            // The harness plays the current loop, decoupled, so the identification is about the mechanics alone.
            // 夹具扮演电流环并做解耦，于是辨识只关于机械那一半。
            const float v_d = -omega_e * lq * reference;
            const float v_q = (reference * rs) + (omega_e * ld * state.id_a) + (omega_e * psi);
            plant.set_voltage_dq(foc::core::Dq<float>{v_d, v_q});
            plant.advance(kDt);
            tuner.accumulate(kDt, plant.state().iq_a, plant.omega_mech_rad_s(), false);
        }
    }
    return tuner.finish();
}

/**
 * @brief [en] **Stage 3, measure**: what bandwidth these gains actually deliver on a machine of inertia `j_scale`.
 * @brief [zh] **第 3 步，测量** ：这些增益在惯量为 `j_scale` 的机器上到底交付多少带宽。
 * @note [en] The tuned gains are handed over as the design pair they imply (`j_hat = kt/(kt/J)`, `b_hat = (b/J)*j_hat`
 *            ), which is exactly the formula `speed_pi_config()` uses -- case 1 asserts that substitution is faithful
 *            to the last bit, so this is the same measurement the incumbent loop gets and not a second gain law.
 * @note [zh] 整定出的增益以它们蕴含的设计对交出（ `j_hat = kt/(kt/J)`、 `b_hat = (b/J)·j_hat`），正是
 *            `speed_pi_config()` 用的公式——用例 1 断言这一步替换逐位忠实，所以这就是在位环路拿到的那次测量，而不是第二
 *            条增益律。
 */
auto delivered_ratio(const SpeedTuneResult& gains, float j_scale) -> double {
    const auto spec = foc::sil::nominal_spec();
    const auto plant = foc::sil::nominal_plant_params();
    const float j_hat = spec.kt / gains.kt_over_j;
    const auto b_hat = gains.b_over_j * j_hat;
    return foc::sil::minus3db_over_requested(kBandwidthRadS, plant.j * j_scale, plant.b, j_hat, b_hat, spec.kt);
}

/**
 * @brief [en] **Stage 4, the objective**: how far this measurement sits from the refusal boundary, in the library's own
 *             units.
 * @brief [zh] **第 4 步，目标函数** ：这次测量离拒绝边界还有多远，用库自己的单位。
 * @return [en] `tolerance - |j_identified/j_nameplate - 1|`; it reaches zero exactly where `valid` flips, which is what
 *              makes it a distance to failure rather than a quality score.
 * @return [zh] `tolerance − |j_辨识/j_铭牌 − 1|`；它在 `valid` 翻转处正好归零，这正是"距离失败多远"而不是质量分的原因。
 */
auto identified_margin(const SpeedTuneResult& result) -> float {
    const float ratio = result.j_kg_m2 / foc::sil::nominal_spec().j;
    return sop_tune_config().tolerance - std::fabs(ratio - 1.0F);
}

} // namespace

/**
 * @brief [en] Identify, propose, measure, and record an objective that is a **distance to failure**: zero exactly where
 *             the library refuses.
 *
 * @brief [zh] 辨识、提议、测量，并记录一个 **距离失败多远** 的目标：在库拒绝的那一点上正好归零。
 *
 *   The objective the plan allows is not "how small is the error" but "how far is this from failing". Here the failure
 *   is not invented: `SpeedTuner` refuses when the identified inertia leaves `tolerance`, so
 *   `tolerance - |j_hat/j - 1|` is a number that **crosses zero at the library's own refusal point** and nowhere else.
 *   Measured on the way there (0.25 tolerance): 1.00 -> +0.2498, 1.12 -> +0.1298, 1.20 -> +0.0498, 1.24 -> +0.0098,
 *   1.26 -> -0.0102 and *refused*. The last two rows are the teeth: the objective is not a monotone decoration, it is
 *   the sign that predicts the answer.
 *
 *   计划允许的目标不是"误差多小"，而是"离失败还有多远"。这里的失败不是我编的：辨识出的惯量离开 `tolerance` 时
 *   `SpeedTuner` 就拒绝，于是 `tolerance − |j_hat/j − 1|` 是一个 **在库自己的拒绝点上穿越零** 的数。沿途实测（容差
 *   0.25 ）：1.00 → +0.2498、1.12 → +0.1298、1.20 → +0.0498、1.24 → +0.0098、1.26 → −0.0102 且 **被拒绝** 。最后两行就
 *   是牙齿：这个目标不是单调的装饰，它是预测答案的那个符号。
 */
TEST(Sop, TheObjectiveIsTheDistanceToTheRefusalBoundaryNotTheErrorMagnitude) {
    const SpeedTuneResult nominal = identify(1.0F);
    ASSERT_EQ(ErrorCode::Ok, nominal.status);
    ASSERT_TRUE(nominal.valid) << nominal.reason;

    // The substitution is faithful: the tuned gains are the ones `speed_pi_config()` computes from the identified pair.
    // 替换忠实：整定出的增益就是 `speed_pi_config()` 从辨识对算出的那些。
    auto motor = foc::sil::nominal_motor_config();
    motor.j = foc::sil::nominal_spec().kt / nominal.kt_over_j; // stage 1's answer 第 1 步的答案
    motor.b = nominal.b_over_j * motor.j;
    const auto from_identified = foc::speed_pi_config(motor, kBandwidthRadS);
    RecordProperty("kp_tuned", nominal.kp_a_per_rad_s);
    RecordProperty("kp_from_identified_pair", from_identified.kp);
    EXPECT_FLOAT_EQ(from_identified.kp, nominal.kp_a_per_rad_s);
    EXPECT_FLOAT_EQ(from_identified.ki, nominal.ki_a_per_rad_s2);

    // Stage 3: the loop these gains make delivers the law the baseline was measured with -- inside the band, not at 1.0.
    // 第 3 步：这些增益做出的环交付的是基线被量出来的那条律——在带内，而不是等于 1.0。
    const double ratio = delivered_ratio(nominal, 1.0F);
    RecordProperty("delivered_ratio", ratio);
    RecordProperty("delivered_margin", delivered_margin(ratio));
    EXPECT_GT(ratio, 1.0) << "the gain law overshoots the request; 1.0 was never the target";
    EXPECT_LT(ratio, kLawResidualMax) << "and only by the law's own residual";
    EXPECT_GT(delivered_margin(ratio), 0.0);

    // Stage 4: the objective decreases as the machine moves away and changes sign exactly where the answer does.
    // 第 4 步：机器走得越远，目标越小，并且恰好在答案翻转处变号。
    float previous = identified_margin(nominal);
    RecordProperty("margin_nominal", previous);
    EXPECT_GT(previous, 0.0F);
    for (const float scale : {1.05F, 1.12F, 1.20F, 1.24F, 1.26F}) {
        const SpeedTuneResult moved = identify(scale);
        const float margin = identified_margin(moved);
        std::printf("[sop] j x %.2f -> margin %+.4f valid %d (kp %.5f)\n", static_cast<double>(scale),
                    static_cast<double>(margin), moved.valid ? 1 : 0, static_cast<double>(moved.kp_a_per_rad_s));
        EXPECT_LT(margin, previous) << "the objective has to move with the machine, at " << scale;
        previous = margin;
        // The sign of the objective *is* the answer: a margin at or below zero and a valid pass cannot both happen.
        // 目标的符号 **就是** 答案：非正余量与"通过"不可能同时成立。
        EXPECT_EQ(margin > 0.0F, moved.valid)
            << "margin " << margin << " against valid " << moved.valid << " at " << scale;
    }
}

/**
 * @brief [en] The same procedure replayed on a **second** machine: the proposed gain changes by the machine's own
 *             factor and the criterion still holds -- which is only worth saying because the wrong machine's gain is
 *             measurably caught.
 *
 * @brief [zh] 同一流程在 **第二台** 机器上重放：提议的增益随机器本身的比例改变，而判据仍然成立——这话之所以值得说，是因
 *             为用错机器的增益会被测出来。
 *
 *   Machine A is the nominal plant, machine B is 20 % heavier -- still inside the 25 % tolerance, so B is a machine
 *   somebody could legitimately have described, and the procedure must serve both. Measured:
 *
 *   | | A's gains on A | A's gains on B | B's gains on A | B's gains on B | |---|---|---|---| | delivered ratio |
 *   **1.03136** | **0.85898** | **1.23787** | **1.03114** |
 *
 *   Two readings, and they are different claims. **The criterion moves**: `kp_B/kp_A` = 1.20000, because
 *   `kp = bandwidth/(kt/J)` and B's ratio is 1.2 times A's. **And it still holds**: each machine's own pass lands on
 *   1.031 of the request, i.e. the same law, so the band built from the law accepts both. What makes the second column
 *   a *held-out* measurement rather than a restatement is the third claim -- A's gains on B deliver 0.859,
 *   **outside the band**, and strictly less than B's own gains deliver. The fourth column closes the other direction:
 *   B's gains on A deliver 1.238, **also outside the band** -- which corrected this case's first draft, where I had
 *   assumed a 24 % overshoot was still close enough. It is not: the band rejects *both* cross-applications and accepts
 *   only the machine's own pair. If someone replaces B with a copy of A, the ratios become the same number and the
 *   strict inequalities go red: that is the plan's red proof, and it is a case rather than a paragraph.
 *
 *   A 是标称 plant，B 重 20 %——仍在 25 % 容差内，所以 B 是一台有人可以正当地描述过的机器，流程必须同时服务两者。实测见
 *   上表。两种读法，是两个不同的断言。 **判据会动** ： `kp_B/kp_A` = 1.20000，因为 `kp = bandwidth/(kt/J)` 而 B 的比值
 *   为 A 的 1.2 倍。 **而且仍然成立** ：每台机器自己的整定都落在请求的 1.031，也就是同一条律，于是由律造出的带会接受两
 *   者。让第二列成为 **留出集** 测量而不是复述的，是第三个断言——A 的增益用在 B 上交付 0.859、 **在带外** ，且严格低于 B
 *   自己增益交付的量（0.833 倍）。若把 B 换成 A 的一份拷贝，两个比值变成同一个数，那条严格不等式立刻红：这就是计划要的
 *   能红证明，而且它是一个用例而不是一段话。
 */
TEST(Sop, TheSameProcedureOnASecondMachineMovesTheCriterionAndStillHolds) {
    const SpeedTuneResult a = identify(1.0F);
    const SpeedTuneResult b = identify(1.2F);
    ASSERT_TRUE(a.valid) << a.reason;
    ASSERT_TRUE(b.valid) << b.reason;

    const double r_aa = delivered_ratio(a, 1.0F);
    const double r_ab = delivered_ratio(a, 1.2F);
    const double r_ba = delivered_ratio(b, 1.0F);
    const double r_bb = delivered_ratio(b, 1.2F);
    std::printf("[sop] r_AA %.5f  r_AB %.5f  r_BA %.5f  r_BB %.5f  kpB/kpA %.5f\n", r_aa, r_ab, r_ba, r_bb,
                static_cast<double>(b.kp_a_per_rad_s / a.kp_a_per_rad_s));
    RecordProperty("r_AA", r_aa);
    RecordProperty("r_AB", r_ab);
    RecordProperty("r_BA", r_ba);
    RecordProperty("r_BB", r_bb);

    // The criterion moves with the machine: same procedure, same window, a gain 1.2x larger.
    // 判据跟着机器动：同一流程、同一窗口，增益大 1.2 倍。
    EXPECT_NEAR(1.2, static_cast<double>(b.kp_a_per_rad_s / a.kp_a_per_rad_s), 0.01)
        << "kp " << a.kp_a_per_rad_s << " -> " << b.kp_a_per_rad_s;

    // And still holds: each machine's own pass reproduces the same law, inside the band.
    // 而且仍然成立：每台机器自己的整定都复现同一条律，在带内。
    EXPECT_GT(delivered_margin(r_aa), 0.0) << "r_AA " << r_aa;
    EXPECT_GT(delivered_margin(r_bb), 0.0) << "r_BB " << r_bb;
    EXPECT_NEAR(r_aa, r_bb, 0.002) << "two machines, one law: the residual is the design's, not the machine's";

    // The held-out tooth, strict: a gain fitted on the wrong machine delivers measurably less than the right one.
    // 留出集的牙齿，严格的：用错机器拟合的增益交付的量明显少于对的那些。
    EXPECT_LT(delivered_margin(r_ab), 0.0) << "A's gains on B must fall outside the band; r_AB " << r_ab;
    EXPECT_LT(r_ab, r_bb)
        << "and strictly below what B's own gains deliver -- swap B for a copy of A and this goes red";
    // The other direction is caught too, and this *corrected* the case's first draft: B's gains on A were expected to
    // stay inside the band because they only overshoot by 1.24x. Measured 1.23787 against an edge of 1.1143, they do
    // not. Both cross-applications are rejected, so the band's pass is a statement about *this* machine and not about
    // "a gain close enough".
    // 另一个方向同样被抓到，而这 **更正** 了用例第一稿：本以为 B 的增益用在 A 上只超 1.24 倍、仍在带内。实测 1.23787 对
    // 边界 1.1143，不在带内。两个方向的交叉施加都被拒绝，于是这个带的通过是对 **这台** 机器的判断，而不是"差不多够近"。
    EXPECT_LT(delivered_margin(r_ba), 0.0) << "B's gains on A are outside the band too; r_BA " << r_ba;
    EXPECT_GT(r_ba, r_aa) << "strictly above what A's own gains deliver";
}

/**
 * @brief [en] Data that does not describe this machine answers `valid == false` and hands back **no number** that could
 *             be mistaken for a tuning.
 *
 * @brief [zh] 不描述这台机器的数据答 `valid == false`，并交回 **任何可能被误当成一次整定的数** 。
 *
 *   A machine 50 % heavier than the nameplate is not a subtle case, and that is the point: the refusal has to be
 *   boring. Measured: `valid` false, `reason` non-empty, and the gains **bit-identical** to `speed_pi_config()`'s
 *   nameplate pair -- a caller that applies whatever it gets applies nothing new. The measured ratio is still reported
 *   (1.50026), because "I measured 1.5x of what you told me" is the useful half, and it is a statement
 *   **about the measurement**, not a gain.
 *
 *   一台比铭牌重 50 % 的机器不是微妙的情形，而这正是要点：拒绝必须是无聊的。实测： `valid` 为假、 `reason` 非空，增益与
 *   `speed_pi_config()` 的铭牌对 **逐位相同** ——拿到什么就施加什么的调用方什么都没换。测量到的比值仍报出（1.50026），因
 *   为 "我量到的是你说的 1.5 倍"是有用的那一半，而它是 **关于测量** 的一句话，不是一条增益。
 */
TEST(Sop, DataThatDoesNotDescribeThisMachineIsRefusedWithoutANumber) {
    const SpeedTuneResult refused = identify(1.5F);
    ASSERT_EQ(ErrorCode::Ok, refused.status);
    EXPECT_FALSE(refused.valid);
    EXPECT_FALSE(std::string(refused.reason).empty()) << "a refusal says why";
    std::printf("[sop] refused: %s\n", refused.reason);

    // No number a caller could apply: the gains are the nameplate's, to the last bit.
    // 没有调用方能施加的数：增益就是铭牌的那些，逐位。
    const auto nameplate = foc::speed_pi_config(foc::sil::nominal_motor_config(), kBandwidthRadS);
    EXPECT_FLOAT_EQ(nameplate.kp, refused.kp_a_per_rad_s);
    EXPECT_FLOAT_EQ(nameplate.ki, refused.ki_a_per_rad_s2);

    // The objective is negative here, and it is the same objective case 1 used -- so the sign of one number answers both
    // "may I use these gains?" and "how far was I from being allowed to?".
    // 这里的目标为负，而它就是用例 1 用的那个目标——于是同一个数的符号同时回答"我可以用这组增益吗"和"我离被允许还有多远"。
    const float margin = identified_margin(refused);
    RecordProperty("margin_refused", margin);
    EXPECT_LT(margin, 0.0F) << "margin " << margin;

    // And the measurement is still visible, which is what makes the refusal actionable rather than a dead end.
    // 而测量结果仍然可见，这才让这条拒绝是可操作的而不是死路。
    RecordProperty("measured_inertia_ratio", refused.j_kg_m2 / foc::sil::nominal_spec().j);
    EXPECT_NEAR(1.5F, refused.j_kg_m2 / foc::sil::nominal_spec().j, 0.02F);
}

/**
 * @brief [en] The HPO row: search **procedure** parameters against the held-out margin, and prove that "smaller error"
 *             would have picked a worse procedure.
 *
 * @brief [zh] HPO 那一行：对着 **留出集余量** 搜 **过程** 参数，并证明"误差更小"会挑到一个更差的流程。
 *
 *   Two facts measured over five window lengths, with the injection amplitude swept separately:
 *
 *   1. **The residual and the held-out delivered ratio disagree, and the residual wins the wrong way.** Residual is
 *   smallest at 2 levels (0.00675) and largest at 6 (0.00869); the held-out ratio `r_AB` is *best* at 3 levels
 *   (0.85961) and *worst* at 2 (0.85866). So the setting a "smaller residual" objective would pick is the one that
 *   delivers the least on the machine that took no part in the fit. The gap is small (0.1 %) because this plant is a
 *   clean two-parameter machine -- the point is the **ordering**, not the size.
 *
 *   2. **The injection amplitude is not a parameter at all.** 0.6x, 1.0x and 1.5x the current levels give the *same*
 *   identified ratios, because the fit is linear and scale-invariant. A search over amplitude would be searching a flat
 *   objective -- which is worth recording, since the plan lists injection amplitude as a live knob.
 *
 *   五个窗长上的两项实测，注入幅值另外扫：
 *
 *   1. **残差与留出集的交付比值不一致，而残差赢在错的那一边。** 残差在 2 级最小（0.00675）、6 级最大（0.00869）；留出集
 *   比值 `r_AB` 在 3 级 **最好** （0.85961）、2 级 **最差** （0.85866）。于是"误差更小"这个目标会挑中在没参与拟合的那台
 *   机器上交付最少的那个设置。差距很小（0.1 %），因为这台 plant 是一台干净的两参数机器——要点是 **排序** ，不是幅度。
 *
 *   2. **注入幅值根本不是参数。** 电流级的 0.6×、1.0×、1.5× 给出 **相同** 的辨识比值，因为拟合是线性且尺度不变的。对着
 *   幅值搜索就是在搜一个平坦的目标——这条值得记下来，因为计划把注入幅值列为可动的旋钮。
 */
TEST(Sop, OptimisingTheResidualWouldHavePickedTheWorseProcedure) {
    struct Row final {
        int steps;
        float residual;
        double r_ab;
    };
    const int window_steps[] = {2, 3, 4, 6, 8};
    std::vector<Row> rows;
    for (const int steps : window_steps) {
        const SpeedTuneResult fitted = identify(1.0F, steps);
        ASSERT_TRUE(fitted.valid) << fitted.reason << " at " << steps << " levels";
        rows.push_back(Row{steps, fitted.residual, delivered_ratio(fitted, 1.2F)});
        std::printf("[sop] %d levels: residual %.5f  r_AB %.5f\n", steps, static_cast<double>(fitted.residual),
                    delivered_ratio(fitted, 1.2F));
    }

    // Objective A: the residual ("smaller error"). Objective B: the held-out delivered ratio ("closest to the request
    // on the machine that took no part in the fit").
    // 目标 A：残差（"误差更小"）。目标 B：留出集交付比值（"在没参与拟合的机器上最接近请求"）。
    const Row* by_residual = &rows.front();
    const Row* by_held_out = &rows.front();
    for (const Row& row : rows) {
        if (row.residual < by_residual->residual) {
            by_residual = &row;
        }
        if (std::fabs(row.r_ab - 1.0) < std::fabs(by_held_out->r_ab - 1.0)) {
            by_held_out = &row;
        }
    }
    RecordProperty("best_by_residual_levels", by_residual->steps);
    RecordProperty("best_by_residual_r_ab", by_residual->r_ab);
    RecordProperty("best_by_held_out_levels", by_held_out->steps);
    RecordProperty("best_by_held_out_r_ab", by_held_out->r_ab);
    std::printf("[sop] residual-optimal: %d levels (r_AB %.5f); held-out-optimal: %d levels (r_AB %.5f)\n",
                by_residual->steps, by_residual->r_ab, by_held_out->steps, by_held_out->r_ab);

    // (a) The two objectives do not agree -- if they did, either one would be a proxy for the other and this row would
    // prove nothing.
    // (a) 两个目标不一致——若一致，任何一个都是另一个的代理，这一行就等于什么都没证明。
    EXPECT_NE(by_residual->steps, by_held_out->steps)
        << "residual picked " << by_residual->steps << " levels, held-out picked " << by_held_out->steps;

    // (b) And where they disagree, the residual-optimal setting is strictly worse on the held-out machine.
    // (b) 而在它们分歧的地方，残差最优的那个设置在留出集机器上严格更差。
    EXPECT_LT(by_residual->r_ab, by_held_out->r_ab)
        << "the residual winner delivers " << by_residual->r_ab << " against " << by_held_out->r_ab;

    // (c) The amplitude is not a knob: three amplitudes, one answer.
    // (c) 幅值不是旋钮：三个幅值，一个答案。
    const SpeedTuneResult low = identify(1.0F, 4, 0.6F);
    const SpeedTuneResult mid = identify(1.0F, 4, 1.0F);
    const SpeedTuneResult high = identify(1.0F, 4, 1.5F);
    ASSERT_TRUE(low.valid && mid.valid && high.valid);
    std::printf("[sop] amplitude 0.6/1.0/1.5 -> kt/J %.6f %.6f %.6f\n", static_cast<double>(low.kt_over_j),
                static_cast<double>(mid.kt_over_j), static_cast<double>(high.kt_over_j));
    EXPECT_NEAR(mid.kt_over_j, low.kt_over_j, mid.kt_over_j * 1.0e-4F) << "a linear fit is scale-invariant";
    EXPECT_NEAR(mid.kt_over_j, high.kt_over_j, mid.kt_over_j * 1.0e-4F);
}
