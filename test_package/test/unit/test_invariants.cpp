// The invariants: statements that must hold no matter which observer, controller or mode is selected.
//
// Why this file exists, separately from the 450-odd scenario cases. A scenario case freezes one configuration and one
// number; it is evidence about that configuration and says nothing about the next one. Breadth multiplies
// configurations, so a foundation defect that lives in a branch nobody selected before becomes reachable later, and no
// existing case notices. An invariant is the other kind of check: it does not name a configuration, so it cannot rot
// when one is added, and it fails for any implementation that breaks the algebra.
//
// Every case here carries its own counter-example. A green invariant proves nothing on its own -- an assertion that can
// never be false is a comment with a runtime cost. So each case also computes what a *wrong* implementation would
// produce and asserts that the wrong one is wrong by a wide margin. That is the red edge: it shows the tolerance is
// doing work rather than swallowing everything.
//
// 不变量：无论选哪个观测器、控制器或模式都必须成立的陈述。
//
// 为什么单独一个文件，与那四百多个场景用例分开。场景用例冻结一个配置和一个数字；它是关于那个配置的证据，对下一个配置
// 什么也没说。广度会让配置数相乘，因此藏在"此前没人选过的分支"里的地基缺陷会在之后变得可达，而已有用例没有一个会察觉。
// 不变量是另一类检查：它不写具体配置，所以新增配置时不会腐化；而任何破坏代数的实现都会让它红。
//
// 本文件每个用例都自带反例。一条绿的不变量本身什么也证明不了——一条永远不可能为假的断言，就是一条带运行时开销的注释。
// 因此每个用例同时算出"错误实现会给出什么"，并断言那个错误的结果错得离谱。这就是红边：它证明容差在干活，而不是把一切
// 都咽下去。

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <foc/api/foc_api.h>
#include <foc/core/limits.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>

#include "sil_pmsm_plant.hpp"

namespace {

using foc::core::Ab;
using foc::core::Dq;
using foc::core::Uvw;
using foc::sil::PmsmPlant;
using foc::sil::PlantParams;
using foc::sil::PlantState;

constexpr float kTwoPi = 6.283185307179586F;

/**
 * @brief [en] A balanced three-phase set of amplitude `a` at electrical angle `phi`.
 * @brief [zh] 幅值为 `a`、电角度为 `phi` 的平衡三相量。
 */
auto balanced(float amplitude, float phi) noexcept -> Uvw<float> {
    const float third = kTwoPi / 3.0F;
    return Uvw<float>{amplitude * std::cos(phi), amplitude * std::cos(phi - third),
                      amplitude * std::cos(phi + third)};
}

/**
 * @brief [en] Distance between two dq pairs, for "how far wrong" comparisons.
 * @brief [zh] 两对 dq 之间的距离，用于"错得多远"的比较。
 */
auto distance(Dq<float> a, Dq<float> b) noexcept -> float { return std::hypot(a.d - b.d, a.q - b.q); }

/**
 * @brief [en] The gains the shipped ABI would run this loop with. Asked, not re-derived: an invariant about the gain
 *             law that recomputed the law would only agree with itself.
 * @brief [zh] 出厂 ABI 会为这个环使用的增益。问出来，而不是重新推导：一条自己重算增益律的不变量，只会与它自己一致。
 */
auto shipped_current_gains(const PlantParams& p, float bandwidth_rad_s, float v_bus_v) noexcept
    -> foc::core::Pi::Config {
    foc_motor_params_t params{};
    params.rs = p.rs;
    params.ld = p.ld;
    params.lq = p.lq;
    params.kt = p.kt;
    params.j = p.j;
    params.b = p.b;
    params.pole_pairs = p.pole_pairs;

    foc_motor_setup_t setup{};
    setup.struct_size = static_cast<std::uint32_t>(sizeof(setup));
    setup.current_bandwidth_rad_s = bandwidth_rad_s;
    setup.v_bus_v = v_bus_v;

    foc_pi_gains_t gains{};
    gains.struct_size = static_cast<std::uint32_t>(sizeof(gains));
    const foc_err_t rc = foc_current_pi_gains(&params, &setup, &gains);
    EXPECT_EQ(rc, FOC_OK);
    return foc::core::Pi::Config{gains.kp, gains.ki, gains.out_min, gains.out_max, gains.integral_min,
                                 gains.integral_max};
}

} // namespace

// ---------------------------------------------------------------------------------------------------------------------
// 1. Amplitude convention, and the zero sequence it deliberately loses.
// 1. 幅值约定，以及它有意丢掉的零序。
// ---------------------------------------------------------------------------------------------------------------------

TEST(Invariants, TheAmplitudeConventionHoldsAndTheZeroSequenceIsLost) {
    // The criterion: a balanced set of amplitude A maps to a vector of amplitude A, for every amplitude and every angle,
    // and the inverse maps it back unchanged. This pins the `2/3` convention: a power-invariant `sqrt(2/3)` variant
    // would put every magnitude a factor of `sqrt(1.5)` = 1.2247 too low, 22 % out -- not a rounding question.
    // 判据：幅值 A 的平衡三相映射为幅值 A 的矢量，对每个幅值与每个角度都成立，且逆变换原样还原。这钉住了 `2/3` 约定：
    // 功率不变式的 `sqrt(2/3)` 变体会让每个幅值低 `sqrt(1.5)` = 1.2247 倍，即差 22%——不是舍入问题。
    const float amplitudes[] = {0.05F, 0.3F, 4.0F, 40.0F};
    for (const float amplitude : amplitudes) {
        for (int step = 0; step < 24; ++step) {
            const float phi = (kTwoPi * static_cast<float>(step)) / 24.0F;
            const Uvw<float> uvw = balanced(amplitude, phi);
            const Ab<float> ab = foc::core::clarke(uvw);
            const float magnitude = std::hypot(ab.alpha, ab.beta);
            EXPECT_NEAR(magnitude, amplitude, 1e-5F * amplitude) << "amplitude " << amplitude << " at phi " << phi;

            // The other convention, computed here so the tolerance above is shown to exclude it.
            // 另一种约定，在这里算出来，以证明上面的容差确实把它排除在外。
            const float power_invariant = magnitude * std::sqrt(1.5F);
            EXPECT_GT(std::fabs(power_invariant - amplitude), 0.2F * amplitude)
                << "the power-invariant convention would not be distinguishable at amplitude " << amplitude;

            const Uvw<float> back = foc::core::inv_clarke(ab);
            EXPECT_NEAR(back.u, uvw.u, 1e-5F * amplitude);
            EXPECT_NEAR(back.v, uvw.v, 1e-5F * amplitude);
            EXPECT_NEAR(back.w, uvw.w, 1e-5F * amplitude);
        }
    }

    // And the part that is *not* invariant, stated rather than discovered: Clarke is blind to the zero sequence, so a
    // common offset added to all three phases does not survive the round trip. The loss equals the offset exactly,
    // which is the assertion -- not "approximately unchanged", which would be a false invariant.
    // 以及那部分*并不*不变的，明确写出来而不是留给别人发现：Clarke 对零序是瞎的，因此三相各加同一个偏置无法通过往返。
    // 损失恰好等于该偏置，这就是断言——不是"大致不变"，那会是一条假不变量。
    const float offset = 2.5F;
    const Uvw<float> clean = balanced(1.0F, 0.4F);
    const Uvw<float> shifted{clean.u + offset, clean.v + offset, clean.w + offset};
    const Uvw<float> recovered = foc::core::inv_clarke(foc::core::clarke(shifted));
    EXPECT_NEAR(recovered.u - shifted.u, -offset, 1e-5F);
    EXPECT_NEAR(recovered.v - shifted.v, -offset, 1e-5F);
    EXPECT_NEAR(recovered.w - shifted.w, -offset, 1e-5F);
}

// ---------------------------------------------------------------------------------------------------------------------
// 2. A rotation of the currents and a rotation of the frame compose.
// 2. 电流的旋转与坐标系的旋转是复合关系。
// ---------------------------------------------------------------------------------------------------------------------

TEST(Invariants, ARotationAndItsFrameComposeExactly) {
    // The criterion: take any dq pair, express it in the abc frame at angle theta, read it back in the frame at
    // theta + delta. The answer must be the pair rotated by -delta, exactly -- `d cos d + q sin d` and `q cos d -
    // d sin d`. Not "the same magnitude": a magnitude-only check would pass for a sign error, which is the error this
    // has to catch.
    // 判据：任取一对 dq，在角度 theta 的三相系里表达它，再在 theta + delta 的系里读回来。答案必须是把这对量旋转
    // -delta，精确地——`d cos δ + q sin δ` 与 `q cos δ − d sin δ`。不是"幅值相同"：只看幅值的检查会让符号错误通过，
    // 而那正是这里必须抓住的错误。
    const Dq<float> pairs[] = {{1.0F, 0.0F}, {0.0F, 1.0F}, {-2.0F, 0.5F}, {3.0F, -4.0F}};
    const float thetas[] = {0.0F, 0.7F, -0.7F, 2.9F, -3.5F, 7.9F, -12.0F};
    const float deltas[] = {0.0F, 0.3F, -1.1F, 2.4F, -3.0F};

    for (const Dq<float> pair : pairs) {
        for (const float theta : thetas) {
            for (const float delta : deltas) {
                const Ab<float> ab = foc::core::inv_park(pair, theta);
                const Dq<float> seen = foc::core::park(ab, theta + delta);
                const float c = std::cos(delta);
                const float s = std::sin(delta);
                const Dq<float> expected{pair.d * c + pair.q * s, pair.q * c - pair.d * s};
                const float tol = 1e-5F * (1.0F + std::fabs(pair.d) + std::fabs(pair.q));
                EXPECT_NEAR(seen.d, expected.d, tol);
                EXPECT_NEAR(seen.q, expected.q, tol);

                // The counter-example: rotating the other way. Its distance from the truth is what the tolerance above
                // has to be far smaller than -- 0.59 for a unit pair at delta = 0.3. At delta = 0 the two rotations
                // are the same rotation, so there is nothing to distinguish and nothing to assert.
                // 反例：反方向旋转。它与真值的距离就是上面容差必须远小于的量——对单位矢量、delta = 0.3 时是 0.59。
                // delta = 0 时两个旋转是同一个旋转，没有可区分的东西，也就无从断言。
                const Dq<float> other_way{pair.d * c - pair.q * s, pair.q * c + pair.d * s};
                if (std::fabs(delta) > 1e-3F) {
                    EXPECT_GT(distance(other_way, expected), 0.1F * (1.0F + std::fabs(pair.q))) << "delta " << delta;
                }
            }
        }
    }
}

// ---------------------------------------------------------------------------------------------------------------------
// 3. Elapsed time decides the integral, not the number of calls.
// 3. 决定积分的是流逝的时间，不是调用次数。
// ---------------------------------------------------------------------------------------------------------------------

TEST(Invariants, ElapsedTimeDecidesTheIntegralNotTheCallCount) {
    // The criterion: driving a PI with the same constant error over the same *duration*, once with `dt` and once with
    // four steps of `dt/4`, produces the same output. For an integrator that accumulates `ki * e * dt` this is exact,
    // not approximate -- so the tolerance can be near float epsilon. Anything that assumes a fixed period (a hard-coded
    // sample time, a call counter, a rate index) breaks this, and nothing else in the suite would.
    // 判据：用同一个恒定误差驱动 PI 同样的*时长*，一次一步 `dt`、一次四步 `dt/4`，输出相同。对累加 `ki*e*dt` 的积分器
    // 这是精确的而非近似的——因此容差可以压到接近 float 机器精度。任何假设周期固定（写死的采样时间、调用计数器、速率
    // 下标）的实现都会破坏这一条，而套件里其他用例不会。
    constexpr float kKi = 2.0F;
    constexpr float kError = 1.0F;
    constexpr float kDuration = 1e-3F;

    const auto make_pi = [] {
        return foc::core::Pi(foc::core::Pi::Config{0.0F, kKi, -1.0e6F, 1.0e6F, -1.0e6F, 1.0e6F});
    };

    auto coarse = make_pi();
    coarse.step(kError, 0.0F, kDuration);
    const float one_step = coarse.integral();

    auto fine = make_pi();
    for (int k = 0; k < 4; ++k) {
        fine.step(kError, 0.0F, kDuration / 4.0F);
    }
    const float four_steps = fine.integral();

    EXPECT_NEAR(four_steps, one_step, 1e-9F);
    // and the value itself is pinned, so a doubled or halved ki does not slip through the equality
    // 而取值本身也被钉住，于是 ki 加倍或减半不能从这条等式里溜过去
    EXPECT_NEAR(one_step, kKi * kError * kDuration, 1e-9F);

    // The counter-example: four steps that each last `dt`, i.e. four times the elapsed time. If the assertion above
    // were insensitive to time it would still hold here.
    // 反例：四步，每步都是 `dt`，即流逝时间是四倍。若上面的断言对时间不敏感，它在这里也会成立。
    auto wrong = make_pi();
    for (int k = 0; k < 4; ++k) {
        wrong.step(kError, 0.0F, kDuration);
    }
    EXPECT_GT(std::fabs(wrong.integral() - one_step), 1e-4F) << "the equality does not distinguish elapsed time";
}

// ---------------------------------------------------------------------------------------------------------------------
// 4. Scaling the impedance and the voltage together changes nothing.
// 4. 阻抗与电压同比例缩放，什么都不变。
// ---------------------------------------------------------------------------------------------------------------------

namespace {

/**
 * @brief [en] Points of the current trajectory, so the comparison looks at the transient rather than only at the end.
 * @brief [zh] 电流轨迹上的取样点，使比较看的是动程而非只有终点。
 */
struct Trace final {
    Dq<float> at_1;
    Dq<float> at_5;
    Dq<float> at_25;
    Dq<float> last;
};

/**
 * @brief [en] One closed current loop: two PIs driving the SIL plant, with a dyno holding the speed.
 * @brief [zh] 一个闭合电流环：两个 PI 驱动 SIL 被控对象，转速由测功机保持。
 */
class CurrentLoop final {
  public:
    CurrentLoop(PlantParams params, foc::core::Pi::Config config, float speed, float dt_s) noexcept
        : plant_(params), d_axis_(config), q_axis_(config), speed_(speed), dt_s_(dt_s) {
        PlantState initial{};
        initial.omega_mech = speed;
        plant_.reset(initial);
        plant_.set_mechanical_speed(speed);
    }

    /**
     * @brief [en] Run `steps` periods at iq = `iq_ref` and id = 0, sampling the transient on the way.
     * @brief [zh] 以 iq = `iq_ref`、id = 0 跑 `steps` 个周期，途中取样动程。
     */
    auto run(int steps, float iq_ref) noexcept -> Trace {
        Trace trace{};
        for (int k = 0; k < steps; ++k) {
            const Dq<float> measured = plant_.current_dq();
            const float vd = d_axis_.step(0.0F, measured.d, dt_s_);
            const float vq = q_axis_.step(iq_ref, measured.q, dt_s_);
            plant_.set_voltage_dq(Dq<float>{vd, vq});
            // The dyno holds the speed: the mechanical state is overwritten every period, as a loading machine does, so
            // the comparison is about the electrical loop and nothing else.
            // 测功机保持转速：每个周期覆写机械状态，与负载机一致，于是比较只涉及电气环。
            plant_.set_mechanical_speed(speed_);
            plant_.advance(dt_s_);

            const Dq<float> now = plant_.current_dq();
            if (k == 0) {
                trace.at_1 = now;
            } else if (k == 4) {
                trace.at_5 = now;
            } else if (k == 24) {
                trace.at_25 = now;
            }
        }
        trace.last = plant_.current_dq();
        return trace;
    }

  private:
    PmsmPlant plant_;
    foc::core::Pi d_axis_;
    foc::core::Pi q_axis_;
    float speed_;
    float dt_s_;
};

} // namespace

TEST(Invariants, ScalingImpedanceAndVoltageTogetherChangesNothing) {
    // The criterion: multiply Rs, Ld, Lq and the flux linkage by k, and the DC link by k. Every electrical equation
    // stays the same, the gains follow (`kp = 2 wb Ld - Rs` and `ki = wb^2 Ld` are homogeneous of degree one once the
    // whole impedance scales, and the output clamp is `v_bus / 2`), so the current trajectory must be *identical*, not
    // merely stable. This is the dimension check: a parameter used in the wrong place, or a constant that forgot to
    // scale, shows up here and almost nowhere else.
    // 判据：把 Rs、Ld、Lq 与磁链同乘 k，母线也乘 k。每个电气方程都不变，增益随之（整个阻抗同比例时
    // `kp = 2·wb·Ld − Rs` 与 `ki = wb²·Ld` 是一次齐次，输出限幅是 `v_bus / 2`），因此电流轨迹必须*逐点相同*，不只是
    // 稳定。这是量纲检查：参数放错位置、或某个常量忘了缩放，会在这里暴露，而几乎在别处不会。
    constexpr int kSteps = 3000; // 150 ms at 20 kHz
    constexpr float kDt = 5e-5F;
    constexpr float kBandwidth = 2.0F * kTwoPi * 400.0F;
    constexpr float kSpeed = 60.0F;
    constexpr float kIq = 1.0F;

    const PlantParams base{/*rs*/ 0.5F, /*ld*/ 4e-4F, /*lq*/ 8e-4F, /*kt*/ 0.0525F,
                           /*j*/ 2e-5F,  /*b*/ 1e-6F,  /*pole_pairs*/ 7};

    // The scaling has to cover every parameter whose dimension is involved, and that includes the mechanical side. The
    // first version of this case scaled Rs, L, kt and v_bus and left J and B alone: torque then scaled by k while the
    // inertia did not, so the speed the RK4 stages carry *within* a period was not proportional, the back-EMF term
    // stopped being proportional with it, and the trajectories differed by up to 3.2e-3 A at k = 25. The symmetry is
    // not "the electrical parameters scale"; it is "everything with a dimension scales", and stating it wrongly is
    // exactly what an invariant is for.
    // 缩放必须覆盖每一个量纲相关的参数，包括机械侧。这个用例的第一版缩放了 Rs、L、kt 与 v_bus，却把 J 与 B 留了下来：
    // 于是转矩乘 k、惯量不变，RK4 各阶段在*周期内*携带的转速不再成比例，反电势项也随之不再成比例，k = 25 时轨迹相差
    // 达 3.2e-3 A。这条对称性不是"电气参数缩放"，而是"凡带量纲的都缩放"——把它陈述错了，正是不变量存在的意义。
    const auto scaled = [](PlantParams p, float k) {
        return PlantParams{p.rs * k, p.ld * k, p.lq * k, p.kt * k, p.j * k, p.b * k, p.pole_pairs};
    };

    const foc::core::Pi::Config base_gains = shipped_current_gains(base, kBandwidth, 24.0F);
    ASSERT_GT(base_gains.kp, 0.0F);
    ASSERT_GT(base_gains.ki, 0.0F);

    CurrentLoop unity(base, base_gains, kSpeed, kDt);
    const Trace reference = unity.run(kSteps, kIq);
    ASSERT_GT(reference.last.q, 0.5F * kIq) << "the loop did not approach its setpoint, so nothing is being compared";
    ASSERT_LT(reference.last.q, 1.5F * kIq) << "the loop overshot into its clamp, so nothing is being compared";

    const float factors[] = {0.25F, 4.0F, 25.0F};
    for (const float k : factors) {
        const foc::core::Pi::Config gains = shipped_current_gains(scaled(base, k), kBandwidth, 24.0F * k);
        CurrentLoop loop(scaled(base, k), gains, kSpeed, kDt);
        const Trace got = loop.run(kSteps, kIq);
        EXPECT_NEAR(got.at_1.d, reference.at_1.d, 1e-4F) << "first period, k = " << k;
        EXPECT_NEAR(got.at_1.q, reference.at_1.q, 1e-4F) << "first period, k = " << k;
        EXPECT_NEAR(got.at_5.q, reference.at_5.q, 1e-4F) << "fifth period, k = " << k;
        EXPECT_NEAR(got.at_25.q, reference.at_25.q, 1e-4F) << "25th period, k = " << k;
        EXPECT_NEAR(got.last.d, reference.last.d, 1e-4F) << "k = " << k;
        EXPECT_NEAR(got.last.q, reference.last.q, 1e-4F) << "k = " << k;
        // the gains themselves are homogeneous, which is the fact the trajectory equality rests on
        // 增益本身是一次齐次的，这是轨迹相等所依赖的事实
        EXPECT_NEAR(gains.kp, k * base_gains.kp, 1e-4F * std::fabs(k * base_gains.kp)) << "kp at k = " << k;
        EXPECT_NEAR(gains.ki, k * base_gains.ki, 1e-4F * std::fabs(k * base_gains.ki)) << "ki at k = " << k;
    }

    // The counter-example: the same scaling, but the gains left at the unscaled values -- one thing forgotten, which is
    // the actual failure mode. It has to be compared in the *transient*, and that is not a detail: the first version of
    // this control compared only the settled value and measured a difference of 4.8e-7, because an integrator absorbs a
    // gain error completely by steady state. A steady-state criterion cannot see a gain error at all; only the first
    // periods can. That failure is why this trace samples at 1, 5 and 25 periods instead of only at the end.
    // 反例：同样的缩放，但增益留在未缩放的值上——忘了一件事，这才是真实的失效方式。它必须在*动程*里比较，这不是细节：
    // 这条反例的第一版只比稳态值，量到的差异是 4.8e-7，因为积分器在稳态会把增益误差完全吸收。稳态判据根本看不见增益
    // 误差，只有最初几个周期看得见。这个失败正是本轨迹在 1、5、25 周期取样的理由。
    CurrentLoop forgotten(scaled(base, 4.0F), base_gains, kSpeed, kDt);
    const Trace wrong = forgotten.run(kSteps, kIq);
    EXPECT_GT(distance(wrong.at_1, reference.at_1), 1e-2F) << "forgetting to scale the gains is not detectable here";
    // and the settled value agrees, which is the measured statement that steady state is the wrong place to look
    // 而稳态值是一致的，这就是"稳态不是查看这件事的地方"的实测陈述
    EXPECT_LT(distance(wrong.last, reference.last), 1e-2F) << "the integrator did not absorb the gain error";
}

// ---------------------------------------------------------------------------------------------------------------------
// 5. A clamp has no step in it.
// 5. 钳位里没有台阶。
// ---------------------------------------------------------------------------------------------------------------------

TEST(Invariants, TheClampIsContinuousAndAJumpWouldShow) {
    // The criterion: sweeping the input across the clamp boundary changes the output by no more than the input changed
    // -- the clamp is 1-Lipschitz. What this catches is the limiter written as a branch that drops the output to zero
    // past the limit, which is a step of the whole limit and reads to a loop as a discontinuity.
    // 判据：让输入扫过钳位边界时，输出的变化不超过输入的变化——钳位是 1-Lipschitz 的。它抓的是那种"越过限幅就把输出
    // 归零"的分支写法，那是整整一个限幅幅度的台阶，在环路看来就是不连续。
    constexpr float kLow = -1.5F;
    constexpr float kHigh = 1.5F;
    constexpr float kStep = 1e-3F;

    // The sweep has to cross the high limit, not stop at it: the first version ran to `kHigh` exactly, so the branch
    // in the counter-example never fired and the control measured a jump of one input step instead of the whole limit.
    // A control that is never exercised looks exactly like a control that passes.
    // 扫描必须越过上限，而不是停在上限：第一版正好跑到 `kHigh`，于是反例里的分支从未触发，量到的是一个输入步长而非
    // 整个限幅。一条从未被行使的控制，看起来和一条通过的控制一模一样。
    float previous_in = kLow;
    float previous_out = foc::core::clamp(previous_in, kLow, kHigh);
    float worst_jump = 0.0F;
    float worst_step_jump = 0.0F;
    const float sweep_end = kHigh + 10.0F * kStep;
    for (float in = kLow + kStep; in <= sweep_end; in += kStep) {
        const float out = foc::core::clamp(in, kLow, kHigh);
        const float delta = std::fabs(out - previous_out);
        EXPECT_LE(delta, std::fabs(in - previous_in) + 1e-6F) << "input " << in;
        worst_jump = std::max(worst_jump, delta);

        // the counter-example, evaluated on the same sweep: past the limit, output zero
        // 反例，在同一次扫描上求值：越过限幅即归零
        const float wrong = (in > kHigh) ? 0.0F : in;
        const float wrong_previous = (previous_in > kHigh) ? 0.0F : previous_in;
        worst_step_jump = std::max(worst_step_jump, std::fabs(wrong - wrong_previous));

        previous_in = in;
        previous_out = out;
    }
    // the good clamp moves at most one input step; the branch moves the whole limit at once
    // 好的钳位最多移动一个输入步长；分支写法一次移动整个限幅
    EXPECT_LE(worst_jump, kStep + 1e-6F);
    EXPECT_GT(worst_step_jump, 0.5F * kHigh);

    // and the same statement for the library's own output stage: a P-only loop swept across its clamp
    // 以及库自己的输出级上的同一陈述：一个只有 P 的环扫过它的钳位
    const foc::core::Pi::Config config{2.0F, 0.0F, -kHigh, kHigh, -kHigh, kHigh};
    float previous = 0.0F;
    bool first = true;
    for (float setpoint = -3.0F; setpoint <= 3.0F; setpoint += kStep) {
        auto pi = foc::core::Pi(config);
        const float out = pi.step(setpoint, 0.0F, 1e-4F);
        if (!first) {
            EXPECT_LE(std::fabs(out - previous), 2.0F * kStep + 1e-5F) << "setpoint " << setpoint;
        }
        previous = out;
        first = false;
    }
}
