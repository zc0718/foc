// F33's prerequisite - the cogging torque injection port. There is no truth for a cogging compensator without an injected
// cogging torque, so the port comes first, and this file pins it three ways: the torque that was asked for is the torque
// the accessor reports, the torque that reaches the *mechanical equation* is the same function with the sign a compensator
// must cancel, and a coasting rotor's speed ripple has an analytic peak-to-peak.
// F33 的前置——齿槽转矩注入端口。没有注入就没有补偿器的真值，所以端口先行；本文件用三种方式钉住它：要的转矩就是访问器
// 报告的那个、进入 **机械方程** 的转矩是同一个函数且符号是补偿器要抵消的那个、以及惰行转子的转速纹波有解析峰峰值。

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include "sil_abc_plant.hpp"

namespace {

using foc::sil::AbcPlant;

constexpr float kTwoPi = 6.28318530718F;

/**
 * @brief The one row this file's table holds: what was injected, and what it did to the speed ripple. 本文件的表只有一行：注入了什么，以及它把转速纹波做成了什么样。
 *
 *     It exists as a named field rather than as a local float because the `budget` gate checks that the registry's
 *     claim ("`set_cogging` is covered by `test_sil_cogging.cpp`, through `ripple_p2p_rad_s`") is a field that file
 *     really has -- a name nothing defines would be a promise with no owner. 它做成具名字段而不是局部浮点数，是因为
 *     `budget` 门会核对登记表里那句话（“`set_cogging` 由 `test_sil_cogging.cpp` 的 `ripple_p2p_rad_s` 覆盖”）在该文件里
 *     真的存在——一个谁也没定义的名字，就是一条没有主语的承诺。
 */
struct CoggingEffects final {
    float injected_nm;      ///< [en] peak of the injected harmonic, N*m. [zh] 注入谐波的峰值，N·m。
    float ripple_p2p_rad_s; ///< [en] measured mechanical speed ripple, rad/s. [zh] 实测机械转速纹波，rad/s。
};

// A passive rotor on purpose: `lambda_m = 0` means no back-EMF, so with zero voltage no current flows and `torque_at()`
// is identically zero. That leaves the injected cogging as the **only** torque in the model, which is what makes the
// coast below a one-line closed form instead of a coupled electromechanical problem.
// 有意选一台无源转子：`lambda_m = 0` 意味着没有反电势，于是零电压下不流电流、`torque_at()` 恒为零。这样注入的齿槽就是
// 模型里 **唯一** 的转矩，下面的惰行才有资格写成一行闭式，而不是一个耦合的机电问题。
auto passive_rotor(float inertia) -> AbcPlant {
    const AbcPlant::Params params{1.0F, 1.0e-3F, 0.0F, 2U};
    AbcPlant plant{params};
    plant.enable_mechanics(AbcPlant::Mechanics{inertia, 0.0F});
    return plant;
}

/** @brief The torque a single-harmonic table asks for, in N*m. */
auto closed_form(float amplitude_nm, int order, float phase_rad, float theta_mech) -> float {
    return amplitude_nm * std::sin((static_cast<float>(order) * theta_mech) + phase_rad);
}

/** @brief Fourier amplitude of `samples` against `order`, with the samples taken at the angles in `theta`. */
auto harmonic_amplitude(const std::vector<float>& theta, const std::vector<float>& samples, int order) -> float {
    float re = 0.0F;
    float im = 0.0F;
    const auto count = static_cast<float>(theta.size());
    for (std::size_t n = 0; n < theta.size(); ++n) {
        const float angle = static_cast<float>(order) * theta[n];
        re = static_cast<float>(re + (samples[n] * std::cos(angle)));
        im = static_cast<float>(im + (samples[n] * std::sin(angle)));
    }
    // Amplitude of the `A*sin(order*theta + phase)` component: this projection is `A/2 * (sin phase, cos phase)`, so the
    // magnitude comes back scaled by `2/N` in both axes and is independent of the phase.
    // `A*sin(order*theta+phase)` 分量的幅值：该投影给出 `A/2·(sin phase, cos phase)`，于是两轴都按 `2/N` 回缩，且与相位无关。
    return (2.0F / count) * std::hypot(re, im);
}

} // namespace

// What was asked for is what the model holds: the accessor, swept over a revolution, is the closed form to float precision,
// its mean is zero, and its peak-to-peak is `2A`. This is the cheap half -- it would pass on a table that was stored and
// never used, which is why it is not the only case here.
// 要的和模型里存的是同一个：访问器在一转上扫描等于闭式解（浮点精度），均值为零，峰峰值为 `2A`。这是便宜的那一半——光
// 存表不用它也能通过，所以它不能是本文件唯一的用例。
TEST(SilCogging, TheInjectedTorqueIsTheOneAskedFor) {
    AbcPlant plant = passive_rotor(1.0e-4F);
    const AbcPlant::CoggingHarmonic table{6U, 2.0e-3F, 0.3F};
    plant.set_cogging(&table, 1);
    ASSERT_EQ(1, plant.cogging_harmonics());

    float minimum = INFINITY;
    float maximum = -INFINITY;
    float mean = 0.0F;
    constexpr int kPoints = 1024;
    for (int n = 0; n < kPoints; ++n) {
        const float theta = (kTwoPi * static_cast<float>(n)) / static_cast<float>(kPoints);
        plant.reset(theta, 0.0F);
        const float measured = plant.cogging_torque_nm();
        EXPECT_NEAR(closed_form(2.0e-3F, 6, 0.3F, theta), measured, 1.0e-8F) << "theta " << theta;
        minimum = std::fmin(minimum, measured);
        maximum = std::fmax(maximum, measured);
        mean = static_cast<float>(mean + (measured / static_cast<float>(kPoints)));
    }
    EXPECT_NEAR(2.0e-3F, (maximum - minimum) / 2.0F, 1.0e-6F);
    EXPECT_NEAR(0.0F, mean, 1.0e-6F);
}

// The order is **per mechanical revolution**, and that is not pedantry: `order` in an electrical frame differs by a factor
// of `pole_pairs`, and a compensator indexed by the wrong one looks like it simply does not work. At `pole_pairs = 2` the
// two periods differ by 2x, so one of them can be confirmed and the other refuted in the same case.
// 次数是 **每机械转** 的，这不是吹毛求疵：电坐标系里的次数差一个极对数，而按错的那个索引的补偿器看起来就是"没生效"。在
// `pole_pairs = 2` 下两个周期差 2 倍，所以同一个用例里可以确认一个、否证另一个。
TEST(SilCogging, TheOrderIsPerMechanicalRevolutionNotPerElectricalOne) {
    AbcPlant plant = passive_rotor(1.0e-4F);
    const AbcPlant::CoggingHarmonic table{6U, 2.0e-3F, 0.0F};
    plant.set_cogging(&table, 1);

    const float theta = 0.017F;
    plant.reset(theta, 0.0F);
    const float here = plant.cogging_torque_nm();

    // One mechanical period of the 6th harmonic: the same torque.
    // 6 次的 **机械** 周期：同一个转矩。
    plant.reset(theta + (kTwoPi / 6.0F), 0.0F);
    EXPECT_NEAR(here, plant.cogging_torque_nm(), 1.0e-9F);

    // The *electrical* period is half of that at two pole pairs, and the torque there is a different number -- so a table
    // interpreted as an electrical order cannot pass both assertions.
    // 两极对下 **电** 周期是它的一半，那里的转矩是另一个值——所以把表当成电次数解释，两条断言不可能同时成立。
    plant.reset(theta + (kTwoPi / (6.0F * 2.0F)), 0.0F);
    EXPECT_GT(std::fabs(plant.cogging_torque_nm() - here), 1.0e-4F);
}

// The part that matters: the torque reaches the *mechanical equation* with the sign a compensator has to cancel. Measured
// by integrating one tiny step from rest and reading `domega = dt*T/J` -- i.e. through the whole mechanical half, not
// through the accessor -- and then compared against the accessor, so the two routes have to agree.
// 要紧的那一半：转矩以补偿器必须抵消的那个符号进入 **机械方程** 。从静止积分一小步、读 `dω = dt·T/J` 来量——走的是整个
// 机械半而不是访问器——再与访问器对照，于是两条路径必须一致。
TEST(SilCogging, TheTorqueReachesTheMechanicalEquationWithTheSignToCancel) {
    constexpr float kJ = 1.0e-4F;
    constexpr float kDt = 1.0e-6F;
    AbcPlant plant = passive_rotor(kJ);
    const AbcPlant::CoggingHarmonic table{6U, 2.0e-3F, 0.3F};
    plant.set_cogging(&table, 1);

    int positive = 0;
    int negative = 0;
    constexpr int kPoints = 256;
    for (int n = 0; n < kPoints; ++n) {
        const float theta = (kTwoPi * static_cast<float>(n)) / static_cast<float>(kPoints);
        plant.reset(theta, 0.0F);
        const float expected = plant.cogging_torque_nm();
        plant.advance(kDt);
        const float measured = (plant.omega_mech_rad_s() * kJ) / kDt;
        EXPECT_NEAR(expected, measured, std::fabs(expected) * 1.0e-3F + 1.0e-9F) << "theta " << theta;
        if (measured > 1.0e-4F) {
            ++positive;
        } else if (measured < -1.0e-4F) {
            ++negative;
        }
    }
    // Both signs occur, so a dropped sign would halve the case rather than pass it.
    // 两个符号都出现过，于是丢掉符号会让用例掉一半而不是通过。
    EXPECT_GT(positive, 100);
    EXPECT_GT(negative, 100);
}

// The analytic anchor, and the one that proves the injection is not merely stored: a coasting rotor with no damping and no
// electromagnetic torque obeys `J*theta'' = A*sin(k*theta)`, and linearising about a constant speed `Omega` gives a speed
// ripple of peak-to-peak `2*A/(J*k*Omega)`. Nothing in the assertion comes from this repository -- it is the same
// perturbation every mechanical engineer writes down.
// 解析锚点，也是"注入不只是存起来了"的证明：无阻尼、无电磁转矩的惰行转子满足 `J·θ'' = A·sin(k·θ)`，在常速 `Ω` 附近线性化
// 得到转速纹波峰峰值 `2·A/(J·k·Ω)`。断言里没有一个数字来自本仓库——那是每个机械工程师都会写下的同一式摄动。
TEST(SilCogging, TheCoastSpeedRippleHasTheAnalyticPeakToPeak) {
    constexpr float kJ = 1.0e-4F;
    constexpr float kOmega = 20.0F;
    constexpr float kAmplitude = 2.0e-3F;
    constexpr int kOrder = 6;
    constexpr float kDt = 1.0e-4F;

    AbcPlant plant = passive_rotor(kJ);
    const AbcPlant::CoggingHarmonic table{static_cast<std::uint8_t>(kOrder), kAmplitude, 0.0F};
    plant.set_cogging(&table, 1);
    plant.reset(0.0F, kOmega);

    float minimum = INFINITY;
    float maximum = -INFINITY;
    float energy_min = INFINITY;
    float energy_max = -INFINITY;
    // Five ripple periods: `k*Omega` is the angular rate of the ripple itself.
    // 五个纹波周期：纹波自身的角速率是 `k·Ω`。
    const int steps = static_cast<int>((5.0F * kTwoPi / (static_cast<float>(kOrder) * kOmega)) / kDt);
    for (int n = 0; n < steps; ++n) {
        plant.advance(kDt);
        const float omega = plant.omega_mech_rad_s();
        const float theta = plant.theta_mech_rad();
        if (n > steps / 10) { // skip nothing physical, but keep the first samples out of the extrema on principle
            minimum = std::fmin(minimum, omega);
            maximum = std::fmax(maximum, omega);
        }
        // `J*w' = A*sin(k*theta)` is the derivative of a conservative system, so `E = J*w^2/2 + (A/k)*cos(k*theta)` is a
        // constant of the motion. Asserting that constant is stronger than asserting the ripple: a leaky integrator can
        // still produce the right peak-to-peak while shedding energy every period.
        // `J·ω' = A·sin(k·θ)` 是一个保守系统的导数式，于是 `E = J·ω²/2 + (A/k)·cos(k·θ)` 是运动常数。断言这个常数比断言纹波更强：
        // 漏能量的积分器仍可能给出正确的峰峰值，同时每个周期都在丢能量。
        const float energy = (0.5F * kJ * omega * omega) +
                             ((kAmplitude / static_cast<float>(kOrder)) * std::cos(static_cast<float>(kOrder) * theta));
        energy_min = std::fmin(energy_min, energy);
        energy_max = std::fmax(energy_max, energy);
    }
    const float expected = (2.0F * kAmplitude) / (kJ * static_cast<float>(kOrder) * kOmega);
    const CoggingEffects effects{kAmplitude, maximum - minimum};
    EXPECT_NEAR(expected, effects.ripple_p2p_rad_s, expected * 0.02F)
        << "measured " << effects.ripple_p2p_rad_s << " expected " << expected;

    // Energy, not "the mean speed does not drift": with `+A*sin(k*theta)` the detents sit at `k*theta = pi`, so the speed
    // *rises* above `Omega` and comes back to exactly `Omega` at `k*theta = 0` -- the midpoint of the extrema is therefore
    // above `Omega` by half the ripple, and asserting the midpoint would have been asserting a mistake of mine rather
    // than the model's behaviour.
    //
    // The bound is **measured, not guessed**: the plant's mechanical half is forward Euler at `h = dt/8`, which is a
    // secular energy error, and over this five-period run it comes to 3.0e-5 of `J*Omega^2/2`. 1e-4 is three times that,
    // so a correct implementation has room and a leaking one does not: a first-order scheme an order of magnitude coarser
    // would land at 1e-3.
    // 是能量，不是"平均转速不漂移"：在 `+A·sin(k·θ)` 下定位点落在 `k·θ = π`，于是转速会 **升到** `Ω` 以上、并在 `k·θ = 0`
    // 处精确回到 `Ω`——因此两个极值的中点在 `Ω` 之上半个纹波，断言那个中点是在断言我自己的错误，而不是模型的行为。
    //
    // 这个界是 **量出来的，不是猜的** ：被控对象的机械半是 `h = dt/8` 的前向欧拉，其能量误差是长期累积型，在这五次纹波周期的
    // 运行里达到 `J·Ω²/2` 的 3.0e-5。1e-4 是它的三倍——正确实现有余量、漏能量的实现没有：再粗一个量级的一阶格式会落在 1e-3。
    EXPECT_LT(energy_max - energy_min, (0.5F * kJ * kOmega * kOmega) * 1.0e-4F);
}

// Off by default, and off is **bit-identical** to a build that never had the port. An error source that moves the frozen
// scenarios cannot be told apart from a regression, which is the rule every other injection port in this repository follows.
// 默认关闭，而且关闭时与"从来没有这个端口"**逐位相同** 。会移动冻结场景的误差源无法与回归区分——这是本仓库其它每个注入端口
// 都遵守的规矩。
TEST(SilCogging, ZeroAmplitudeAndNoTableAreBitIdenticalToOneAnother) {
    constexpr float kDt = 1.0e-4F;
    auto run = [](bool inject_zero) -> std::pair<float, float> {
        AbcPlant plant = passive_rotor(1.0e-4F);
        if (inject_zero) {
            const AbcPlant::CoggingHarmonic zero{6U, 0.0F, 1.234F};
            plant.set_cogging(&zero, 1);
        }
        plant.reset(0.0F, 37.0F);
        for (int n = 0; n < 500; ++n) {
            plant.advance(kDt);
        }
        return {plant.theta_mech_rad(), plant.omega_mech_rad_s()};
    };
    const auto without = run(false);
    const auto with_zero = run(true);
    EXPECT_EQ(without.first, with_zero.first);
    EXPECT_EQ(without.second, with_zero.second);
    EXPECT_EQ(0, passive_rotor(1.0e-4F).cogging_harmonics());

    // An order-0 entry is a constant torque, which belongs to `set_load_torque`; it is dropped rather than silently
    // becoming a second way to set a load.
    // 0 次的条目是恒定转矩，那属于 `set_load_torque`；它被丢弃，而不是悄悄变成设置负载的第二种方式。
    AbcPlant plant = passive_rotor(1.0e-4F);
    const AbcPlant::CoggingHarmonic constant{0U, 5.0e-3F, 0.0F};
    plant.set_cogging(&constant, 1);
    EXPECT_EQ(0, plant.cogging_harmonics());
}

// Two orders at once are separated by the same measurement that will later read a compensator's table: a Fourier
// projection over one revolution. If the projection could not separate them, F47's "the learned table matches the injected
// one" would be unmeasurable, and that would be worth knowing before it is written.
// 两个次数同时在，用的就是日后读补偿器表的那套测量：在一转上做傅里叶投影。若投影分不开它们，F47 那句"学到的表等于注入的表"
// 就无法量——而那件事值得在写之前就知道。
TEST(SilCogging, TwoInjectedOrdersAreSeparableByAProjectionOverOneRevolution) {
    constexpr float kJ = 1.0e-4F;
    constexpr float kDt = 1.0e-6F;
    AbcPlant plant = passive_rotor(kJ);
    const AbcPlant::CoggingHarmonic table[2] = {{2U, 3.0e-3F, 0.0F}, {6U, 1.0e-3F, 0.0F}};
    plant.set_cogging(table, 2);
    ASSERT_EQ(2, plant.cogging_harmonics());

    std::vector<float> theta;
    std::vector<float> torque;
    constexpr int kPoints = 2048;
    for (int n = 0; n < kPoints; ++n) {
        const float angle = (kTwoPi * static_cast<float>(n)) / static_cast<float>(kPoints);
        plant.reset(angle, 0.0F);
        plant.advance(kDt);
        theta.push_back(angle);
        torque.push_back((plant.omega_mech_rad_s() * kJ) / kDt);
    }
    EXPECT_NEAR(3.0e-3F, harmonic_amplitude(theta, torque, 2), 3.0e-5F);
    EXPECT_NEAR(1.0e-3F, harmonic_amplitude(theta, torque, 6), 1.0e-5F);
    // The orders that were NOT injected are empty, which is what makes the two numbers above meaningful.
    // **没有** 注入的次数是空的，这才让上面两个数有意义。
    EXPECT_LT(harmonic_amplitude(theta, torque, 4), 1.0e-5F);
    EXPECT_LT(harmonic_amplitude(theta, torque, 8), 1.0e-5F);
}
