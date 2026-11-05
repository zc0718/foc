// F35: an outer loop that tunes itself from a measurement, and refuses when the measurement is not of the machine the
// caller described. The baseline this sits on was measured first (`-3 dB` against the requested bandwidth: 1.034 at 20 Hz,
// 1.072 at 40 Hz, 1.184 at 80 Hz), so "the gains are right" means "they reproduce the law that baseline was measured with",
// not "the ratio is 1".
// F35：一个从测量里自己整定的外环，并在测量不是对调用方所描述的那台机器时拒绝。它脚下的基线是**先**量的（−3 dB 对请求带宽：
// 20 Hz 处 1.034、40 Hz 处 1.072、80 Hz 处 1.184），所以"增益是对的"意思是"它们复现了那条基线被量出来的律"，而不是"比值等于 1"。

#include <cmath>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/auto_tune.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_bench.hpp"
#include "sil_cogging_table.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

using foc::ErrorCode;
using foc::core::SpeedTuneConfig;
using foc::core::SpeedTuner;
using foc::core::SpeedTuneResult;

/** @brief The nameplate the tuner is told about: the bench's own machine, unscaled. */
// 告诉整定器的铭牌：台架自己那台机器，不缩放。
auto tune_config(float bandwidth_rad_s = 250.0F) -> SpeedTuneConfig {
    const auto spec = foc::sil::nominal_spec();
    SpeedTuneConfig config{};
    config.kt_nm_per_a = spec.kt;
    config.j_expected_kg_m2 = spec.j;
    config.b_expected_nm_s = spec.b;
    config.bandwidth_rad_s = bandwidth_rad_s;
    config.i_max_a = spec.i_max_a;
    config.tolerance = 0.25F;
    config.min_current_a = 0.05F;
    config.max_residual = 0.35F;
    config.min_samples = 100;
    return config;
}

/**
 * @brief Run a current staircase on the plant and feed the tuner what a drive would see.
 * @param j_scale multiplier on the plant's inertia, so "a machine that is not the one you described" is a real machine.
 * @param detent_amplitude_nm cogging injected into the plant, N*m; 0 for none.
 * @param vary_current when false the current is held at one value, so no slope can be measured.
 */
// 在 plant 上跑一条电流阶梯，把驱动器会看到的东西喂给整定器。
// `j_scale` 缩放 plant 的惯量，好让"不是你描述的那台机器"是一台真实存在的机器；`detent_amplitude_nm` 注入齿槽；
// `vary_current` 为假时电流被按住不动，于是量不出斜率。
auto tune_with(float j_scale, float detent_amplitude_nm, bool vary_current) -> SpeedTuneResult {
    foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    params.j = params.j * j_scale;
    foc::sil::PmsmPlant plant{params};
    if (detent_amplitude_nm > 0.0F) {
        foc::sil::CoggingHarmonic detent{2U, detent_amplitude_nm, 0.0F};
        plant.set_cogging(&detent, 1);
    }
    SpeedTuner tuner{tune_config()};
    const float dt = 5.0e-5F;
    const float rs = params.rs;
    const float ld = params.ld;
    const float lq = params.lq;
    const float psi = plant.psi_f_wb();
    const std::vector<float> steps = vary_current ? std::vector<float>{0.2F, 0.6F, 0.1F, 0.8F, 0.3F, 0.5F}
                                                  : std::vector<float>{0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F};
    for (const float reference : steps) {
        for (int n = 0; n < 400; ++n) { // 20 ms per step 每级 20 ms
            const float omega_e = static_cast<float>(params.pole_pairs) * plant.omega_mech_rad_s();
            const auto state = plant.state();
            // The harness plays the current loop, decoupled: `id` held at 0 and `iq` driven to the reference. That is what a
            // drive's current loop does, and it keeps the identification about the *mechanics*, which is what is being tuned.
            // 夹具扮演电流环并做解耦：`id` 保持在 0、`iq` 被驱动到参考。这是驱动器电流环所做的事，而它让辨识只关于**机械**，
            // 也就是真正被整定的那一部分。
            const float v_d = -omega_e * lq * reference;
            const float v_q = (reference * rs) + (omega_e * ld * state.id_a) + (omega_e * psi);
            plant.set_voltage_dq(foc::core::Dq<float>{v_d, v_q});
            plant.advance(dt);
            tuner.accumulate(dt, plant.state().iq_a, plant.omega_mech_rad_s(), false);
        }
    }
    return tuner.finish();
}

} // namespace

// The headline: on the machine it was told about, the pass identifies the two ratios it needs and lands on the **same gains**
// `speed_pi_config()` computes -- one from a measurement of the plant, the other from the nameplate. Two independent routes to
// the same number, which is what makes "the tuner works" a measurement rather than a restatement.
// 要点：在被告知的那台机器上，这次整定辨识出它需要的两个比值，并落在与 `speed_pi_config()` **相同的增益**上——一个来自对 plant 的
// 测量，另一个来自铭牌。两条独立的路给出同一个数，这才让"整定器能用"成为一次测量而不是一句复述。
TEST(AutoTune, IdentifiesTheMachineItWasToldAboutAndLandsOnTheNameplateGains) {
    const SpeedTuneResult tuned = tune_with(1.0F, 0.0F, true);
    ASSERT_EQ(ErrorCode::Ok, tuned.status);
    ASSERT_TRUE(tuned.valid) << "reason: " << tuned.reason;
    ASSERT_GT(tuned.samples, 1000);

    const auto spec = foc::sil::nominal_spec();
    const float expected_kt_over_j = spec.kt / spec.j;
    RecordProperty("kt_over_j", tuned.kt_over_j);
    RecordProperty("expected_kt_over_j", expected_kt_over_j);
    RecordProperty("residual", tuned.residual);
    EXPECT_NEAR(expected_kt_over_j, tuned.kt_over_j, expected_kt_over_j * 0.05F)
        << "measured " << tuned.kt_over_j << " against " << expected_kt_over_j;
    EXPECT_NEAR(spec.j, tuned.j_kg_m2, spec.j * 0.05F) << "inertia back from kt/(kt/J): " << tuned.j_kg_m2;

    // The cross-check that is not a restatement: the motor layer computes the same `Pi::Config` from the nameplate.
    // 不是复述的那一步对照：电机层从铭牌算出同一个 `Pi::Config`。
    const auto nameplate = foc::speed_pi_config(foc::sil::nominal_motor_config(), tune_config().bandwidth_rad_s);
    EXPECT_NEAR(nameplate.kp, tuned.kp_a_per_rad_s, nameplate.kp * 0.05F)
        << "tuned kp " << tuned.kp_a_per_rad_s << " against speed_pi_config " << nameplate.kp;
    EXPECT_NEAR(nameplate.ki, tuned.ki_a_per_rad_s2, std::fmax(nameplate.ki * 0.25F, 0.05F))
        << "tuned ki " << tuned.ki_a_per_rad_s2 << " against speed_pi_config " << nameplate.ki;
    EXPECT_EQ(nameplate.out_max, tune_config().i_max_a); // the ceiling the caller asked for 调用方要求的限幅
}

// It follows the machine, not the nameplate: a machine 20 % heavier -- still inside the tolerance, so it *is* the machine the
// caller described -- gets gains 20 % larger, because `kp = bandwidth/(kt/J)` and the identified ratio moved by that much.
// 它跟着机器走，不跟着铭牌走：一台重 20 % 的机器——仍在容差内，所以它**就是**调用方描述的那台——拿到大 20 % 的增益，因为
// `kp = bandwidth/(kt/J)` 而辨识出的比值正好动了那么多。
TEST(AutoTune, FollowsAMachineThatIsHeavierThanTheNameplateButStillWithinTolerance) {
    const SpeedTuneResult nominal = tune_with(1.0F, 0.0F, true);
    const SpeedTuneResult heavy = tune_with(1.2F, 0.0F, true);
    ASSERT_TRUE(nominal.valid) << nominal.reason;
    ASSERT_TRUE(heavy.valid) << heavy.reason;
    RecordProperty("kp_nominal", nominal.kp_a_per_rad_s);
    RecordProperty("kp_heavy", heavy.kp_a_per_rad_s);
    EXPECT_NEAR(1.2F, heavy.j_kg_m2 / nominal.j_kg_m2, 0.06F) << "measured inertia ratio";
    EXPECT_NEAR(1.2F, heavy.kp_a_per_rad_s / nominal.kp_a_per_rad_s, 0.06F) << "and the gain followed it";
}

// The falsifier the plan named: double the object's inertia without telling the tuner, and it must refuse rather than
// silently re-tune to a machine nobody described. "Refuse" has to mean something concrete, so the gains it hands back are
// asserted **bit-identical** to the nameplate ones: a caller that applies whatever it gets applies nothing new.
// 计划点名的那条能红的证明：把对象的惯量翻倍而不告诉整定器，它必须拒绝，而不是悄悄整定到一台没人描述过的机器上。"拒绝"必须有具体
// 含义，所以它交回的增益被断言与铭牌增益**逐位相同**：拿到什么就施加什么的调用方，什么新东西都没施加。
TEST(AutoTune, ADoubledInertiaIsRefusedAndTheGainsComeBackUntouched) {
    const SpeedTuneResult doubled = tune_with(2.0F, 0.0F, true);
    ASSERT_EQ(ErrorCode::Ok, doubled.status);
    EXPECT_FALSE(doubled.valid) << "a 2x machine is not the one in the configuration";
    EXPECT_NE(std::string(doubled.reason).empty(), true) << "a refusal says why";
    const auto nameplate = foc::speed_pi_config(foc::sil::nominal_motor_config(), tune_config().bandwidth_rad_s);
    EXPECT_FLOAT_EQ(nameplate.kp, doubled.kp_a_per_rad_s) << "no new gains: bit-identical to speed_pi_config";
    EXPECT_FLOAT_EQ(nameplate.ki, doubled.ki_a_per_rad_s2);

    // And the measurement is still reported, because "I measured 2x what you told me" is the useful half of the answer.
    // 而测量结果仍然报出来，因为"我量到的是你说的两倍"才是这个答案里有用的那一半。
    EXPECT_NEAR(2.0F, doubled.j_kg_m2 / foc::sil::nominal_spec().j, 0.12F) << "measured " << doubled.j_kg_m2;
}

// Two facts, and the first one is a correction of what this case was written to expect. A window where the **reference**
// never moves is still a measurement: the step from rest at the start of the window carries the whole slope, and the pass
// identifies the machine from it (measured 0.5158 against the nameplate 0.5000, and it comes back `valid`). What is *not* a
// measurement is a window where nothing happened at all -- constant current, constant speed -- which no plant can produce,
// so that half is fed to the tuner directly. The guard is about the tuner, and this is the shape it has to catch.
// 两个事实，而第一个是对本用例原预期的**更正**：**参考**不动的窗口仍然是一次测量——窗口起点那一步从静止出发，把整条斜率带了进来，
// 整定器据此辨识出了机器（实测 0.5158 对铭牌 0.5000，并返回 `valid`）。真正的"不是测量"是**什么都没发生**的窗口——电流恒定、
// 转速恒定——而那没有任何 plant 能产生，所以这一半直接喂给整定器。护栏是关于整定器的，而这正是它必须挡住的形状。
TEST(AutoTune, ASingleStepIsAlreadyAMeasurementButAWindowWhereNothingMovesIsNot) {
    const SpeedTuneResult one_step = tune_with(1.0F, 0.0F, false);
    EXPECT_TRUE(one_step.valid) << "the initial step is the information: " << one_step.reason;
    EXPECT_NEAR(foc::sil::nominal_spec().kt / foc::sil::nominal_spec().j, one_step.kt_over_j,
                (foc::sil::nominal_spec().kt / foc::sil::nominal_spec().j) * 0.10F);

    SpeedTuner still{tune_config()};
    for (int n = 0; n < 2000; ++n) {
        still.accumulate(5.0e-5F, 0.5F, 0.0F, false); // nothing moves 什么都不动
    }
    const SpeedTuneResult flat = still.finish();
    EXPECT_FALSE(flat.valid);
    EXPECT_NE(std::string(flat.reason).find("vary"), std::string::npos) << "reason was: " << flat.reason;
    const auto nameplate = foc::speed_pi_config(foc::sil::nominal_motor_config(), tune_config().bandwidth_rad_s);
    EXPECT_FLOAT_EQ(nameplate.kp, flat.kp_a_per_rad_s) << "a refusal hands back the nameplate gains";
}

// Something the model cannot explain: a position-dependent detent. `dw/dt = (kt/J)*iq - (b/J)*w` has no term that depends on
// the angle, so a cogging torque lands in the residual instead of in the ratios, and the pass says so rather than tuning to a
// model that does not hold.
// 模型解释不了的东西：随位置变化的定位点。`dω/dt = (kt/J)·iq − (b/J)·ω` 里没有依赖角度的项，于是齿槽转矩落进残差而不是落进比值，
// 而整定如实说出来，而不是整定到一个不成立的模型上。
TEST(AutoTune, APositionDependentDetentShowsUpInTheResidualAndIsRefused) {
    const SpeedTuneResult detent = tune_with(1.0F, 8.0e-3F, true);
    const SpeedTuneResult clean = tune_with(1.0F, 0.0F, true);
    ASSERT_TRUE(clean.valid) << clean.reason;
    RecordProperty("residual_clean", clean.residual);
    RecordProperty("residual_detent", detent.residual);
    EXPECT_GT(detent.residual, clean.residual * 3.0F) << "clean " << clean.residual << " detent " << detent.residual;
    EXPECT_FALSE(detent.valid) << "residual " << detent.residual << " reason " << detent.reason;
}

// A constant unknown load is the case the three-parameter fit is *built* to survive: it lands in the intercept, which is why
// the ratios come out the same and the pass stays valid. Stating it as a criterion keeps someone from "improving" the fit by
// dropping the intercept, which would quietly attribute the load to the current and move `kp`.
// 一个未知的恒定负载，正是这个三参数拟合**生来**要扛住的情形：它落在截距里，于是两个比值不变、整定仍然有效。把它写成判据，是为了
// 防止有人为了"改进"拟合而丢掉截距——那会把负载悄悄归到电流上，并移动 `kp`。
TEST(AutoTune, AConstantUnknownLoadLandsInTheInterceptAndLeavesTheGainsAlone) {
    const SpeedTuneResult clean = tune_with(1.0F, 0.0F, true);
    // A load is applied through the plant's own load port, so the machine really is being pulled on.
    // 负载经 plant 自己的负载端口施加，于是机器确实在被拉。
    foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    foc::sil::PmsmPlant plant{params};
    SpeedTuner tuner{tune_config()};
    const float dt = 5.0e-5F;
    const float psi = plant.psi_f_wb();
    const std::vector<float> steps{0.2F, 0.6F, 0.1F, 0.8F, 0.3F, 0.5F};
    plant.set_load_torque(2.0e-3F); // 2 mN*m, unknown to the tuner 2 mN·m，整定器不知道
    for (const float reference : steps) {
        for (int n = 0; n < 400; ++n) {
            const float omega_e = static_cast<float>(params.pole_pairs) * plant.omega_mech_rad_s();
            const auto state = plant.state();
            const float v_d = -omega_e * params.lq * reference;
            const float v_q = (reference * params.rs) + (omega_e * params.ld * state.id_a) + (omega_e * psi);
            plant.set_voltage_dq(foc::core::Dq<float>{v_d, v_q});
            plant.advance(dt);
            tuner.accumulate(dt, plant.state().iq_a, plant.omega_mech_rad_s(), false);
        }
    }
    const SpeedTuneResult loaded = tuner.finish();
    ASSERT_TRUE(loaded.valid) << loaded.reason;
    EXPECT_NEAR(clean.kt_over_j, loaded.kt_over_j, clean.kt_over_j * 0.05F)
        << "clean " << clean.kt_over_j << " loaded " << loaded.kt_over_j;
    EXPECT_NEAR(clean.kp_a_per_rad_s, loaded.kp_a_per_rad_s, clean.kp_a_per_rad_s * 0.05F)
        << "the proportional gain must not see a constant load";
}

// Refusals with their own reasons, and a configuration that cannot answer at all.
// 各有理由的拒绝，以及一个根本无法回答的配置。
TEST(AutoTune, RefusesShortWindowsSaturatedPeriodsAndImpossibleConfigurations) {
    SpeedTuner short_window{tune_config()};
    for (int n = 0; n < 10; ++n) {
        short_window.accumulate(5.0e-5F, 0.4F, static_cast<float>(n) * 0.01F, false);
    }
    const SpeedTuneResult few = short_window.finish();
    EXPECT_FALSE(few.valid);
    EXPECT_NE(std::string(few.reason).find("min_samples"), std::string::npos) << few.reason;

    SpeedTuner saturated{tune_config()};
    for (int n = 0; n < 500; ++n) {
        saturated.accumulate(5.0e-5F, (n % 2 == 0) ? 0.9F : 0.3F, static_cast<float>(n) * 0.01F, n == 250);
    }
    const SpeedTuneResult railed = saturated.finish();
    EXPECT_FALSE(railed.valid);
    EXPECT_NE(std::string(railed.reason).find("saturated"), std::string::npos) << railed.reason;

    SpeedTuneConfig bad = tune_config();
    bad.kt_nm_per_a = 0.0F;
    EXPECT_EQ(ErrorCode::Param, SpeedTuner{bad}.validate());
    EXPECT_EQ(ErrorCode::Param, SpeedTuner{bad}.finish().status);
    bad = tune_config();
    bad.j_expected_kg_m2 = 0.0F;
    EXPECT_EQ(ErrorCode::Param, SpeedTuner{bad}.validate());
    bad = tune_config();
    bad.bandwidth_rad_s = 0.0F;
    EXPECT_EQ(ErrorCode::Param, SpeedTuner{bad}.validate());
    bad = tune_config();
    bad.min_samples =
        2; // three parameters need three points, and this asks for two 三个参数至少要三个点，而这里只要两个
    EXPECT_EQ(ErrorCode::Param, SpeedTuner{bad}.validate());
    bad = tune_config();
    bad.tolerance = 0.0F; // an exact match is not a tolerance 精确相符不是容差
    EXPECT_EQ(ErrorCode::Param, SpeedTuner{bad}.validate());
}

// The window is a resource, and a commissioning tool runs more than one pass: after a run the sums must be gone, and a sample
// that is not a number must not enter the fit. Both are checked here because both are silent when wrong -- a stale sum
// produces a plausible gain from a mixture of two machines, and a NaN poisons every later fit with no visible symptom.
// 窗口是一种资源，而调试工具会跑不止一次整定：一次运行之后那些和必须清空，而非数值样本不能进入拟合。两者都写在这里，因为两者出错时
// 都是静默的——残留的和会从两台机器的混合里给出一个看起来合理的增益，而一个 NaN 会污染之后每一次拟合并毫无症状。
TEST(AutoTune, ResetClearsTheWindowAndNonFiniteSamplesNeverEnterIt) {
    SpeedTuner tuner{tune_config()};
    for (int n = 0; n < 500; ++n) {
        tuner.accumulate(5.0e-5F, (n % 2 == 0) ? 0.8F : 0.2F, static_cast<float>(n) * 0.01F, false);
    }
    ASSERT_GT(tuner.samples(), 100);
    tuner.reset();
    EXPECT_EQ(0, tuner.samples());
    const SpeedTuneResult after_reset = tuner.finish();
    EXPECT_FALSE(after_reset.valid) << "an empty window is not a measurement";

    SpeedTuner careful{tune_config()};
    careful.accumulate(0.0F, 0.5F, 0.0F, false);   // no time passed 没有时间流过
    careful.accumulate(5.0e-5F, NAN, 0.0F, false); // not a number 不是数值
    careful.accumulate(5.0e-5F, 0.5F, NAN, false); // ditto 同上
    careful.accumulate(NAN, 0.5F, 0.0F, false);    // ditto 同上
    EXPECT_EQ(0, careful.samples()) << "none of those four carried information";

    // Every remaining configuration refusal, one line each: they exist so that a commissioning tool gets `Param` and stops,
    // rather than a gain computed from a half-specified machine.
    // 其余每一条配置拒绝，各一行：它们存在是为了让调试工具拿到 `Param` 就停下，而不是拿到一个由半份说明书算出来的增益。
    const std::vector<SpeedTuneConfig> bad = [] {
        std::vector<SpeedTuneConfig> out;
        SpeedTuneConfig c = tune_config();
        c.b_expected_nm_s = -1.0F;
        out.push_back(c);
        c = tune_config();
        c.i_max_a = 0.0F;
        out.push_back(c);
        c = tune_config();
        c.min_current_a = -1.0F;
        out.push_back(c);
        c = tune_config();
        c.max_residual = 0.0F;
        out.push_back(c);
        c = tune_config();
        c.kt_nm_per_a = NAN;
        out.push_back(c);
        c = tune_config();
        c.j_expected_kg_m2 = NAN;
        out.push_back(c);
        c = tune_config();
        c.bandwidth_rad_s = NAN;
        out.push_back(c);
        c = tune_config();
        c.i_max_a = NAN;
        out.push_back(c);
        c = tune_config();
        c.min_current_a = NAN;
        out.push_back(c);
        c = tune_config();
        c.max_residual = NAN;
        out.push_back(c);
        c = tune_config();
        c.b_expected_nm_s = NAN;
        out.push_back(c);
        c = tune_config();
        c.tolerance = NAN;
        out.push_back(c);
        return out;
    }();
    for (const auto& config : bad) {
        EXPECT_EQ(ErrorCode::Param, SpeedTuner{config}.validate());
        EXPECT_EQ(ErrorCode::Param, SpeedTuner{config}.finish().status);
    }
}
