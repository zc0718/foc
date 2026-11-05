// F33 - the cogging compensator as a torque-ripple feedforward. The criteria are all "the residue after cancellation", because
// a compensator is the one kind of policy whose failure mode is *looking* fine: a wrong phase cancels nothing and a naive gain
// over-compensates by a ratio that is easy to mistake for "close enough".
// F33——齿槽补偿器，做成转矩纹波前馈。所有判据都是「抵消之后的残差」，因为补偿器正是那种失效时会**看起来正常**的策略：相位错
// 则什么都不抵消，而用一个天真的增益则按一个很容易被当成「差不多」的比例过补偿。

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/cogging_comp.hpp>

namespace {

using foc::ErrorCode;
using foc::core::CoggingCompConfig;
using foc::core::CoggingCompensation;
using foc::core::CoggingCompHarmonic;
using foc::core::CoggingCompInput;
using foc::core::CoggingCompOutput;
using foc::core::MachineParams;

constexpr float kTwoPi = 6.28318530718F;

// A surface machine first: `ld == lq`, so `dT/diq` is exactly `1.5*p*psi_f` and the law can be checked against the
// nameplate constant. The salient case is a *separate* test, because that is where the naive gain fails.
// 先来一台表贴机：`ld == lq`，于是 `dT/diq` 正好是 `1.5·p·ψf`，律可以对着铭牌常数核对。凸极是**另一个**用例，因为天真的增益
// 就是在那儿失效的。
auto surface_config(float psi_f) -> CoggingCompConfig {
    CoggingCompConfig config{};
    config.params = MachineParams{0.5F, 4.0e-4F, 4.0e-4F, psi_f};
    config.pole_pairs = 7U;
    config.fade_from_hz = 0.0F;
    config.fade_to_hz = 0.0F;
    return config;
}

auto salient_config(float psi_f) -> CoggingCompConfig {
    CoggingCompConfig config{};
    config.params = MachineParams{0.5F, 4.0e-3F, 8.0e-3F, psi_f};
    config.pole_pairs = 7U;
    config.fade_from_hz = 0.0F;
    config.fade_to_hz = 0.0F;
    return config;
}

auto torque_per_amp(const CoggingCompConfig& config, float id_a) -> float {
    const MachineParams& m = config.params;
    return 1.5F * static_cast<float>(config.pole_pairs) * (m.psi_f + ((m.ld - m.lq) * id_a));
}

auto command(const CoggingCompConfig& config, float theta, float id_a, float omega = 0.0F) -> CoggingCompInput {
    return CoggingCompInput{theta, omega, id_a, 0.0F, 3.0F};
}

} // namespace

// The law, and the residue it leaves: over a revolution the offset is `-Tcog/(1.5*p*psi_f)` and what survives the
// cancellation is zero to float precision. Asserting the residue rather than the offset is deliberate -- the offset could be
// right for the wrong reason (a sign error in both the table and the gain cancels), while the residue is what the machine
// actually feels.
// 律，以及它留下的残差：在一转上偏置就是 `−Tcog/(1.5·p·ψf)`，而抵消后存活的是浮点精度意义上的零。断言残差而不是偏置是有意的
// ——偏置可能因为错的原因而对（表与增益各错一个符号会互相抵消），而残差才是机器真正感受到的东西。
TEST(CoggingComp, TheResidueIsZeroOverAWholeRevolution) {
    const CoggingCompConfig config = surface_config(0.0047619F);
    CoggingCompensation comp{config};
    const CoggingCompHarmonic table{6U, 4.0e-3F, 0.3F};
    comp.set_table(&table, 1);
    ASSERT_EQ(1, comp.harmonics());

    const float kt = torque_per_amp(config, 0.0F);
    constexpr int kPoints = 720;
    for (int n = 0; n < kPoints; ++n) {
        const float theta = (kTwoPi * static_cast<float>(n)) / static_cast<float>(kPoints);
        const CoggingCompOutput out = comp.offset(command(config, theta, 0.0F));
        ASSERT_EQ(ErrorCode::Ok, out.status);
        const float asked = 4.0e-3F * std::sin((6.0F * theta) + 0.3F);
        EXPECT_NEAR(-asked / kt, out.iq_a, 1.0e-6F) << "theta " << theta;
        const float residue = asked + (kt * out.iq_a);
        EXPECT_NEAR(0.0F, residue, 1.0e-9F) << "theta " << theta;
        EXPECT_FLOAT_EQ(1.0F, out.gain);
        EXPECT_FALSE(out.clipped);
    }
}

// The salient machine, which is where a nameplate `Kt` fails. `dT/diq = 1.5*p*(psi_f + (ld-lq)*id)`, so compensating with
// `1.5*p*psi_f` leaves exactly the fraction `(ld-lq)*id/psi_f` of the torque uncancelled. On the F32 profile at `id = -1.8 A`
// that fraction is 62 % of the cogging still there -- and it is worst exactly where compensation is most wanted.
// 凸极机，也就是铭牌 `Kt` 失效的地方。`dT/diq = 1.5·p·(ψf + (Ld−Lq)·id)`，所以用 `1.5·p·ψf` 补偿会恰好留下 `(Ld−Lq)·id/ψf`
// 那一份转矩没抵消。F32 那台档案机在 `id = −1.8 A` 处那份是齿槽的 62 %——而且它在最需要补偿的地方最严重。
TEST(CoggingComp, ASalientMachineNeedsTheActualTorquePerAmpNotTheNameplateKt) {
    const CoggingCompConfig config = salient_config(0.0047619F);
    CoggingCompensation comp{config};
    const CoggingCompHarmonic table{6U, 4.0e-3F, 0.0F};
    comp.set_table(&table, 1);

    const float id = -1.8F;
    const float kt_eff = torque_per_amp(config, id);
    const float kt_nameplate = torque_per_amp(config, 0.0F);
    // With `lq > ld` a negative `id` *adds* reluctance torque, so the true torque per amp is **larger** than the nameplate
    // constant -- and the naive gain therefore asks for more current than the machine needs.
    // `lq > ld` 时负 `id` **增加**磁阻转矩，所以真实的每安培转矩**大于**铭牌常数——于是天真的增益要的电流比机器需要的更多。
    ASSERT_GT(kt_eff, kt_nameplate);

    const float theta = 0.4F;
    const CoggingCompOutput out = comp.offset(command(config, theta, id));
    ASSERT_EQ(ErrorCode::Ok, out.status);
    const float cog = 4.0e-3F * std::sin(6.0F * theta);

    // With the real `dT/diq`, nothing is left.
    // 用真实的 `dT/diq`，什么都不剩。
    EXPECT_NEAR(0.0F, cog + (kt_eff * out.iq_a), 1.0e-9F);

    // With the nameplate constant, the residue is exactly the ratio -- and that number is what a naive implementation would
    // have shipped.
    // 用铭牌常数，残差恰好是那个比例——而那个数就是天真的实现会交付的东西。
    // With the nameplate constant, the residue is the cogging **inverted and amplified** by `kt_eff/kt_nameplate - 1` -- and
    // that number is what a naive implementation would have shipped.
    // 用铭牌常数时，残差是被 `kt_eff/kt_nameplate − 1` **反向并放大**的齿槽——那个数就是天真的实现会交付的东西。
    const float naive_offset = -cog / kt_nameplate;
    const float residual_fraction = (cog + (kt_eff * naive_offset)) / cog;
    // `1 - kt_eff/kt_nameplate`, i.e. negative: the residue is the cogging *inverted* and 1.5x larger.
    // 即 `1 − kt_eff/kt_nameplate`，是负的：残差是**反向**、且大 1.5 倍的齿槽。
    const float predicted = 1.0F - (kt_eff / kt_nameplate);
    EXPECT_NEAR(predicted, residual_fraction, 1.0e-6F);
    EXPECT_LT(residual_fraction, -1.0F); // bigger than the thing it was supposed to remove 比它本该去掉的东西还大
}

// The classic silent failure: a table whose phase is wrong cancels **nothing**. The amplitude of the compensated torque is
// then exactly the amplitude of the injected one -- not smaller, which is what distinguishes "the compensator is
// mis-phased" from "the compensator works a little".
// 经典的静默失效：相位错的表**什么都不**抵消。此时补偿后的转矩幅值正好等于注入的幅值——不是更小，而这正是「补偿器相位错」与
// 「补偿器起了一点作用」的区别。
TEST(CoggingComp, ATableWithTheWrongPhaseCancelsNothingAtAll) {
    const CoggingCompConfig config = surface_config(0.0047619F);
    CoggingCompensation comp{config};
    const CoggingCompHarmonic wrong_phase{6U, 4.0e-3F, 0.3F + (0.5F * 3.14159265F)};
    comp.set_table(&wrong_phase, 1);

    const float kt = torque_per_amp(config, 0.0F);
    float worst = 0.0F;
    constexpr int kPoints = 720;
    for (int n = 0; n < kPoints; ++n) {
        const float theta = (kTwoPi * static_cast<float>(n)) / static_cast<float>(kPoints);
        const CoggingCompOutput out = comp.offset(command(config, theta, 0.0F));
        ASSERT_EQ(ErrorCode::Ok, out.status);
        const float true_cog = 4.0e-3F * std::sin((6.0F * theta) + 0.3F);
        const float after = true_cog + (kt * out.iq_a);
        worst = std::fmax(worst, std::fabs(after));
    }
    // A quadrature error does not leave the ripple where it was: it adds a second sinusoid at right angles, so the residue is
    // `sqrt(2)` times the original amplitude -- **worse than doing nothing**, which is the fact that makes the phase worth
    // measuring rather than assuming.
    // 正交误差不会让纹波停在原处：它加了一条正交的正弦，于是残差是原幅值的 `√2` 倍——**比什么都不做更糟**，而这正是让相位值得
    // 测量、而不是假设的原因。
    EXPECT_NEAR(4.0e-3F * std::sqrt(2.0F), worst, 1.0e-2F * 4.0e-3F);
}

// The frequency boundary, as a decision rather than an opinion: with a fade configured the offset shrinks monotonically in
// speed and is exactly zero at `fade_to_hz`; with no fade configured the offset at high speed is bit-identical to the one at
// standstill. Both halves matter -- the second is what proves the fade is doing something rather than the law drifting.
// 频率边界，做成决定而不是意见：配置了淡出时偏置随转速单调缩小、在 `fade_to_hz` 处正好为零；不配置淡出时高速处的偏置与静止处
// 逐位相同。两半都要紧——后一半才是「淡出确实在做事，而不是律在漂」的证据。
TEST(CoggingComp, TheFadeIsMonotoneAndDisappearsAtItsUpperEdge) {
    CoggingCompConfig faded = surface_config(0.0047619F);
    faded.fade_from_hz = 30.0F;
    faded.fade_to_hz = 90.0F;
    CoggingCompensation with_fade{faded};
    const CoggingCompHarmonic table{6U, 4.0e-3F, 0.0F};
    with_fade.set_table(&table, 1);

    CoggingCompensation without_fade{surface_config(0.0047619F)};
    without_fade.set_table(&table, 1);

    const float theta = 0.1F;
    // Order 6 at `omega` rad/s mechanical is `6*omega/(2*pi)` Hz.
    // 6 次在 `omega` rad/s 机械处是 `6·omega/(2π)` Hz。
    const float at_0 = with_fade.offset(command(faded, theta, 0.0F, 0.0F)).iq_a;
    const float at_31hz = with_fade.offset(command(faded, theta, 0.0F, (31.0F * kTwoPi) / 6.0F)).iq_a;
    const float at_60hz = with_fade.offset(command(faded, theta, 0.0F, (60.0F * kTwoPi) / 6.0F)).iq_a;
    const float at_90hz = with_fade.offset(command(faded, theta, 0.0F, (90.0F * kTwoPi) / 6.0F)).iq_a;

    EXPECT_FLOAT_EQ(std::fabs(at_0), std::fabs(at_0));
    EXPECT_GT(std::fabs(at_0), std::fabs(at_31hz));
    EXPECT_GT(std::fabs(at_31hz), std::fabs(at_60hz));
    EXPECT_FLOAT_EQ(0.0F, at_90hz);

    // Halfway through the band the gain is 1/2, which pins the fade's shape rather than only its direction.
    // 带子中点处增益是 1/2，这钉住的是淡出的**形状**而不只是方向。
    const CoggingCompOutput mid = with_fade.offset(command(faded, theta, 0.0F, (60.0F * kTwoPi) / 6.0F));
    EXPECT_NEAR(0.5F, mid.gain, 1.0e-5F);

    // And with no fade the same high speed gets the same answer as standstill.
    // 而不配置淡出时，同一个高转速得到的答案与静止时相同。
    const float plain_0 = without_fade.offset(command(faded, theta, 0.0F, 0.0F)).iq_a;
    const float plain_fast = without_fade.offset(command(faded, theta, 0.0F, (600.0F * kTwoPi) / 6.0F)).iq_a;
    EXPECT_FLOAT_EQ(plain_0, plain_fast);
}

// The ceiling applies to the sum, not to the offset: what the current loop sees is `iq_command + offset`, so trimming the
// offset alone would report a happy number while the total exceeded the limit.
// 上限作用在**和**上而不是偏置上：电流环看到的是 `iq_command + offset`，所以只削偏置会报出一个愉快的数、而总和越限。
TEST(CoggingComp, TheCurrentCeilingAppliesToTheSumNotToTheOffset) {
    const CoggingCompConfig config = salient_config(0.0047619F);
    CoggingCompensation comp{config};
    // 25 mN*m at `Kt = 0.05` is an offset of 0.5 A: comfortable on its own, and over the ceiling once it is added to a
    // reference that is already at 2.95 A. That is the case worth testing -- an offset that is large on its own would pass
    // for the wrong reason.
    // 25 mN·m 在 `Kt = 0.05` 下是 0.5 A 的偏置：自己一个人很宽裕，加上已经到 2.95 A 的参考就越限。这才是值得测的情形——
    // 一个自己就很大的偏置会因为错的原因通过。
    const CoggingCompHarmonic table{6U, 2.5e-2F, 0.0F};
    comp.set_table(&table, 1);

    // `theta = -pi/12` is the negative peak of the 6th harmonic, so the offset is +0.5 A.
    // `theta = −π/12` 是 6 次的负峰，于是偏置是 +0.5 A。
    const CoggingCompInput in{-0.2617994F, 0.0F, 0.0F, 2.95F,
                              3.0F}; // reference already near the ceiling 参考已接近上限
    const CoggingCompOutput out = comp.offset(in);
    ASSERT_EQ(ErrorCode::Ok, out.status);
    EXPECT_TRUE(out.clipped);
    EXPECT_NEAR(3.0F, in.iq_command_a + out.iq_a, 1.0e-6F);
    // 0.05 A survived out of the 0.5 A asked for: the ceiling trimmed the **sum**, and the offset alone would have fitted.
    // 0.5 A 的要求里只活下来 0.05 A：上限削的是**和**，而偏置自己本来是装得下的。
    const float unclipped = -out.torque_nm / torque_per_amp(config, 0.0F);
    EXPECT_NEAR(0.5F, unclipped, 1.0e-4F);
    EXPECT_NEAR(0.05F, out.iq_a, 1.0e-4F);
}

// Refusals, one line each. The flux-nulling case is the interesting one: at `psi_f + (ld-lq)*id == 0` the machine produces no
// torque for any current, so "what current cancels this torque" has no answer and the honest reply is `Param` rather than a
// number the caller would clamp into looking reasonable.
// 拒绝用例各一行。削磁那条最有意思：`ψf + (Ld−Lq)·id == 0` 处机器对任何电流都不出转矩，于是「什么电流能抵消这个转矩」没有
// 答案，诚实的回答是 `Param`，而不是一个被调用方钳得看起来合理的数。
TEST(CoggingComp, RefusesWhatItCannotAnswer) {
    const CoggingCompConfig good = salient_config(0.0047619F);
    const CoggingCompHarmonic table{6U, 4.0e-3F, 0.0F};

    CoggingCompensation comp{good};
    comp.set_table(&table, 1);
    EXPECT_EQ(ErrorCode::Ok, comp.validate());

    // A table of nothing is not a decision, and it must not look like a fade of zero.
    // 空的表不是一个决定，也不能看起来像增益为零。
    CoggingCompensation empty{good};
    const CoggingCompHarmonic constant{0U, 5.0e-3F, 0.0F};
    empty.set_table(&constant, 1);
    EXPECT_EQ(0, empty.harmonics());
    const CoggingCompOutput none = empty.offset(command(good, 0.1F, 0.0F));
    EXPECT_EQ(ErrorCode::Ok, none.status);
    EXPECT_FLOAT_EQ(0.0F, none.iq_a);

    // Flux nulling: `psi_f + (ld-lq)*id = 0` solves to `id = -psi_f/(ld-lq)`, which is **positive** when `lq > ld`.
    // 削磁：`ψf + (Ld−Lq)·id = 0` 解出 `id = −ψf/(Ld−Lq)`，在 `lq > ld` 时它是**正的**。
    const float null_id = -good.params.psi_f / (good.params.ld - good.params.lq);
    EXPECT_GT(null_id, 0.0F);
    // The residue is a *cancellation*, so its floor is the rounding of the two terms rather than zero: measured 1.3e-9 N*m/A --
    // 2.6e-8 of the machine's own scale -- and it moves by a bit between compilers. It broke a build twice: a bare absolute
    // `1e-9` fell over under the MC/DC-instrumented build, and then the same absolute bound made the *library* answer `Ok`
    // instead of `Param` on macOS, where clang landed the residue at 1.1e-9. Both the bound here and the library's own refusal
    // threshold are now a millionth of `1.5*p*psi_f`, which is above every rounding and below any real torque per amp by
    // orders of magnitude.
    // 残差是**相消**的结果，所以它的地板是两项的舍入而不是零：实测 1.3e-9 N·m/A——机器自身尺度的 2.6e-8——而它在不同编译器之间会
    // 差一位。它两次弄坏过构建：写死的 `1e-9` 先在 MC/DC 插桩构建下翻车，随后同一个绝对界让**库**在 macOS 上答了 `Ok` 而不是
    // `Param`（clang 的残差落在 1.1e-9）。现在这里的界与库自己的拒绝阈值都是 `1.5·p·ψf` 的百万分之一，高于任何舍入、又低于任何
    // 真实的每安培转矩好几个数量级。
    const float torque_scale = 1.5F * static_cast<float>(good.pole_pairs) * good.params.psi_f;
    EXPECT_LT(std::fabs(torque_per_amp(good, null_id)), torque_scale * 1.0e-6F)
        << "residue " << torque_per_amp(good, null_id) << " against scale " << torque_scale;
    EXPECT_EQ(ErrorCode::Param, comp.offset(command(good, 0.1F, null_id)).status);

    // And the configuration refusals.
    // 以及配置层面的拒绝。
    CoggingCompConfig bad = good;
    bad.pole_pairs = 0U;
    EXPECT_EQ(ErrorCode::Param, CoggingCompensation{bad}.validate());
    bad = good;
    bad.params.ld = 0.0F;
    EXPECT_EQ(ErrorCode::Param, CoggingCompensation{bad}.validate());
    bad = good;
    bad.fade_from_hz = 90.0F;
    bad.fade_to_hz = 30.0F; // an inverted band the fade could not interpolate 倒置的带子淡出无法插值
    EXPECT_EQ(ErrorCode::Param, CoggingCompensation{bad}.validate());
    bad = good;
    bad.fade_from_hz = NAN;
    EXPECT_EQ(ErrorCode::Param, CoggingCompensation{bad}.validate());

    EXPECT_EQ(ErrorCode::Param, comp.offset(CoggingCompInput{NAN, 0.0F, 0.0F, 0.0F, 3.0F}).status);
    EXPECT_EQ(ErrorCode::Param, comp.offset(CoggingCompInput{0.0F, INFINITY, 0.0F, 0.0F, 3.0F}).status);
}

// Two orders at once, which is the case the F47 self-learning piece will feed it: the residues of both must vanish in the same
// pass, so the table is a sum and not a choice.
// 两个次数同时在，正是 F47 自学习那件会喂给它的情形：两个残差必须在同一遍里同时归零，所以表是一个和而不是一个选择。
TEST(CoggingComp, TwoOrdersAreCancelledInTheSamePass) {
    const CoggingCompConfig config = salient_config(0.0047619F);
    CoggingCompensation comp{config};
    const CoggingCompHarmonic table[2] = {{2U, 3.0e-3F, 0.4F}, {6U, 1.0e-3F, -1.1F}};
    comp.set_table(table, 2);
    ASSERT_EQ(2, comp.harmonics());

    const float id = -0.5F;
    const float kt = torque_per_amp(config, id);
    constexpr int kPoints = 720;
    float worst = 0.0F;
    for (int n = 0; n < kPoints; ++n) {
        const float theta = (kTwoPi * static_cast<float>(n)) / static_cast<float>(kPoints);
        const CoggingCompOutput out = comp.offset(command(config, theta, id));
        ASSERT_EQ(ErrorCode::Ok, out.status);
        const float cog = (3.0e-3F * std::sin((2.0F * theta) + 0.4F)) + (1.0e-3F * std::sin((6.0F * theta) - 1.1F));
        worst = std::fmax(worst, std::fabs(cog + (kt * out.iq_a)));
    }
    EXPECT_LT(worst, 1.0e-8F);
}
