// 2.3b: electronic gearing and two-axis synchronisation. The whole difficulty is one word -- *unwrapped* -- because an
// encoder reads [0, 2*pi) and a gear that multiplies what it is given hands the slave a reference that walks backwards
// twice every three revolutions. And the point of the capability is the *measurement*: comparing against a wrapped master
// is how a perfectly synchronised pair reports 6.28 rad of error once a revolution.
// 2.3b：电子齿轮与两轴同步。全部难点在一个词里——**去卷绕**——因为编码器读的是 [0, 2π)，而一个直接乘下去的齿轮会给从轴
// 一个每三圈倒退两次的参考。这条能力的要点是那个**测量**：拿卷绕的主轴去比，正是一对完全同步的轴每转一圈报出 6.28 rad
// 误差的方式。

#include <algorithm>
#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/gear_ratio.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

constexpr float kDt = 6.25e-5F;
constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kPi = 3.14159265358979323846F;
constexpr float kIMax = 3.0F;

auto wrap_two_pi(float angle_rad) -> float {
    float wrapped = std::fmod(angle_rad, kTwoPi);
    if (wrapped < 0.0F) {
        wrapped += kTwoPi;
    }
    return wrapped;
}

auto ratio_recipe(float ratio, float offset_rad = 0.0F) -> foc::core::GearConfig {
    foc::core::GearConfig config{};
    config.ratio = ratio;
    config.offset_rad = offset_rad;
    return config;
}

/**
 * @brief What a gear built on the wrapped reading would produce -- the law the module exists to replace. 一个建在卷绕读数上的齿轮会给出的东西——正是本模块存在的意义所在要替代的那条规律。
 */
auto naive_slave(float master_wrapped_rad, float ratio, float offset_rad) -> float {
    return (ratio * master_wrapped_rad) + offset_rad;
}

/**
 * @brief One axis: a P-D position loop on the mechanical angle, in front of a current source. 一根轴：机械角上的 P-D 位置环，位于一个电流源之前。
 */
class SimpleAxis final {
  public:
    SimpleAxis(float kp, float kd)
        : params_(foc::sil::nominal_plant_params()), plant_(params_), kp_(kp), kd_(kd),
          psi_f_(params_.kt / (1.5F * static_cast<float>(params_.pole_pairs))) {
        plant_.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
    }

    void step(float target_rad, float velocity_ff_rad_s = 0.0F) {
        const foc::sil::PlantState now = plant_.state();
        // `+ kd * v_ref` is the velocity feedforward that cancels a P-D loop's ramp lag exactly at constant speed. It is
        // the gear's velocity output that makes it possible; without it the loop lags by `v * kd / kp` and no gain
        // removes the term.
        // `+ kd · v_ref` 是恒速下正好抵消 P-D 环斜坡滞后的速度前馈。让它成为可能的是齿轮的速度输出；没有它，环滞后
        // `v · kd / kp`，加增益也去不掉这一项。
        const float demand =
            (kp_ * (target_rad - now.theta_mech)) + (kd_ * velocity_ff_rad_s) - (kd_ * now.omega_mech);
        const float iq = std::max(-kIMax, std::min(kIMax, demand / params_.kt));
        const float omega_e = now.omega_mech * static_cast<float>(params_.pole_pairs);
        plant_.set_voltage_dq(foc::core::Dq<float>{-(omega_e * params_.lq * iq), (params_.rs * iq) + (omega_e * psi_f_)});
        plant_.advance(kDt);
    }

    [[nodiscard]] auto position_rad() const -> float { return plant_.state().theta_mech; }

  private:
    foc::sil::PlantParams params_;
    foc::sil::PmsmPlant plant_;
    float kp_;
    float kd_;
    float psi_f_;
};

} // namespace

/**
 * @brief Three master revolutions, read through a wrapping encoder: the slave advances 18.85 rad, and no consecutive slave target jumps by more than a milliradian. The naive law jumps by 6.28 twice.
 *
 *     This is the capability in one case. A master turning at a constant rate reads 0, 6.28, 0, 6.28 ... and a gear
 *     that multiplies that reading hands the slave a reference that runs to the top and then snaps to the bottom --
 *     twice per three revolutions, because 2*pi does not divide 6.28 exactly in the way the next line of code assumes
 *     it does. 这是收在一个用例里的能力。匀速转动的主轴读出 0、6.28、0、6.28…而一个把该读数乘下去的齿轮给从轴的参考会冲
 *     到顶上再啪地掉回底下——每三圈两次，因为 2π 并不按下一行代码假定的方式整除 6.28。
 */
TEST(ElectronicGear, ThreeWrappedRevolutionsBecomeOneContinuousReference) {
    constexpr float kStepRad = 1.0e-3F;   // per sample: a slow master
    constexpr int kSamples = 18850;       // 18.85 rad, a hair over three revolutions
    constexpr float kRatio = 1.0F;
    constexpr float kOffset = 0.100F;

    foc::core::ElectronicGear gear(ratio_recipe(kRatio, kOffset));
    float previous_slave = 0.0F;
    float previous_naive = 0.0F;
    float worst_jump = 0.0F;
    float worst_naive_jump = 0.0F;
    float master_rad = 0.0F;
    int naive_jumps = 0;
    for (int i = 0; i < kSamples; ++i) {
        master_rad = static_cast<float>(i) * kStepRad;
        const float wrapped = wrap_two_pi(master_rad);
        const foc::core::GearOutput out = gear.apply(wrapped, kDt);
        ASSERT_EQ(out.status, foc::ErrorCode::Ok) << "sample " << i;
        if (i > 0) {
            worst_jump = std::max(worst_jump, std::fabs(out.slave_rad - previous_slave));
            const float naive = naive_slave(wrapped, kRatio, kOffset);
            const float naive_jump = std::fabs(naive - previous_naive);
            worst_naive_jump = std::max(worst_naive_jump, naive_jump);
            if (naive_jump > kPi) {
                ++naive_jumps;
            }
            previous_naive = naive;
        }
        previous_slave = out.slave_rad;
    }

    // The gear advances by the master's travel, and never jumps.
    // 齿轮随主轴的行程前进，且从不跳变。
    EXPECT_NEAR(gear.master_unwrapped_rad(), master_rad, 2.0F * kStepRad);
    EXPECT_NEAR(previous_slave, master_rad + kOffset, 2.0F * kStepRad);
    EXPECT_LT(worst_jump, 2.0F * kStepRad) << "worst consecutive jump " << worst_jump << " rad";

    // The law it replaces jumps by 6.28 twice, and that is the whole argument for the unwrap.
    // 被它替代的那条规律跳了两次 6.28，这正是去卷绕的全部论据。
    EXPECT_EQ(naive_jumps, 2) << "the naive law breaks twice over three revolutions";
    EXPECT_GT(worst_naive_jump, 6.0F) << "worst naive jump " << worst_naive_jump << " rad";
}

/**
 * @brief The ratio and the offset are applied to the unwrapped master, exactly. 传动比与偏置作用在去卷绕后的主轴上，精确地。
 */
TEST(ElectronicGear, TheRatioAndTheOffsetAreExact) {
    for (const float ratio : {2.0F, 0.5F, -1.5F}) {
        SCOPED_TRACE(ratio);
        foc::core::ElectronicGear gear(ratio_recipe(ratio, 0.25F));
        EXPECT_EQ(gear.apply(0.0F, kDt).status, foc::ErrorCode::Ok); // origin
        float master = 0.0F;
        for (int i = 0; i < 4000; ++i) {
            master += 0.0007F;
            const auto out = gear.apply(wrap_two_pi(master), kDt);
            const float expected = (ratio * out.master_unwrapped_rad) + 0.25F;
            EXPECT_NEAR(out.slave_rad, expected, 1.0e-5F) << "sample " << i;
        }
        EXPECT_NEAR(gear.master_unwrapped_rad(), master, 0.002F);
    }
}

/**
 * @brief The synchronisation error of a geared pair is `v * kd / kp` -- measured against that arithmetic, at two rates, at a tenth of the gain, and with the gear's velocity output fed forward.
 *
 *     A pair that is *perfectly* geared still shows an error, and it is not a defect: a position loop following a
 *     moving reference has a standing lag of `v * kd / kp`, which at 10 rad/s, kd 0.02 and kp 8 is **25 mrad**. The
 *     case measures it on an ideal ramp (no master plant at all) where the prediction is exact, then through a real
 *     master axis where the extra 4 mrad is the master's own tracking -- the gear is measuring a machine, not a number.
 *     Halving the master's rate halves the error and a tenth of the slave's gain multiplies it by ten, which is the
 *     formula asserted in both of its arguments. The gear's velocity output then removes most of it, and what is left
 *     is the finite difference of a *measured* position. 一对**完全**同步的轴仍然显示误差，而且那不是缺陷：跟随移动参考
 *     的位置环有一个 `v · kd / kp` 的稳态滞后，在 10 rad/s、 kd 0.02、kp 8 下就是 **25 mrad**。用例先在理想斜坡（完全没
 *     有主轴 plant）上量它，那里预测是精确的；再通过一根真实主轴量，多出的 4 mrad 是主轴自己的跟随误差——齿轮量的是机
 *     器，不是一个数。主轴速率减半则误差减半，从轴增益减到十分之一则误差涨十倍，这正是对公式两个自变量的断言。齿轮的速
 *     度输出随后去掉其中大部分，剩下的是**实测位置**的有限差分。
 */
TEST(ElectronicGear, TheSyncErrorIsTheRampLagInBothOfItsTerms) {
    struct Run final {
        float worst_error;
    };
    // `ideal` drives the gear from the ramp itself; otherwise from a real master axis, which carries its own lag.
    // `ideal` 用斜坡本身驱动齿轮；否则用一根真实主轴驱动，那根主轴带着自己的跟随误差。
    auto run_pair = [](float rate_rad_s, float slave_kp, bool feedforward, bool ideal) {
        foc::core::ElectronicGear gear(ratio_recipe(2.0F, 0.0F));
        SimpleAxis master(8.0F, 0.02F);
        SimpleAxis slave(slave_kp, 0.02F); // damping not scaled with the gain: that is what a weaker axis means
        Run out{0.0F};
        constexpr int kPeriods = 4000; // 0.25 s at 5 rad/s is 1.25 rad of master travel
        for (int i = 0; i < kPeriods; ++i) {
            const float t = static_cast<float>(i) * kDt;
            master.step(rate_rad_s * t);
            const float source = ideal ? (rate_rad_s * t) : master.position_rad();
            const auto geared = gear.apply(wrap_two_pi(source), kDt);
            slave.step(geared.slave_rad, feedforward ? geared.slave_velocity_rad_s : 0.0F);
            if (i > kPeriods / 5) {
                out.worst_error = std::max(out.worst_error, std::fabs(geared.slave_rad - slave.position_rad()));
            }
        }
        return out;
    };
    constexpr float kSlaveVelocity = 10.0F; // twice the master's 5 rad/s
    constexpr float kKd = 0.02F;
    constexpr float kKp = 8.0F;
    const float predicted = (kSlaveVelocity * kKd) / kKp;

    const Run ideal = run_pair(5.0F, kKp, false, true);
    const Run real = run_pair(5.0F, kKp, false, false);
    const Run half_rate = run_pair(2.5F, kKp, false, false);
    const Run weak = run_pair(5.0F, kKp * 0.1F, false, false);
    const Run ideal_ff = run_pair(5.0F, kKp, true, true);
    const Run real_ff = run_pair(5.0F, kKp, true, false);

    // 1. The formula, where nothing else is in the way.
    // 1. 公式，在没有别的东西挡路的地方。
    EXPECT_NEAR(ideal.worst_error, predicted, 0.0015F) << "ideal ramp: " << (ideal.worst_error * 1000.0F) << " mrad";

    // 2. Through a real master: the same plus the master's own tracking, which must be small but is not zero.
    // 2. 通过真实主轴：同样的误差加上主轴自己的跟随，后者必须很小但不是零。
    EXPECT_GT(real.worst_error, 1.05F * ideal.worst_error) << "a measured master must cost something";
    EXPECT_LT(real.worst_error, 1.35F * ideal.worst_error) << "but not much: " << (real.worst_error * 1000.0F);

    // 3. Linear in the velocity, inverse in the gain -- the formula asserted in both of its arguments.
    // 3. 对速度是线性的、对增益是反比的——公式的两个自变量各断言一次。
    EXPECT_NEAR(half_rate.worst_error / real.worst_error, 0.5F, 0.05F) << "half the rate: "
                                                                      << (half_rate.worst_error * 1000.0F) << " mrad";
    EXPECT_NEAR(weak.worst_error / ideal.worst_error, 10.0F, 0.6F) << "a tenth of the gain: "
                                                                  << (weak.worst_error * 1000.0F) << " mrad";

    // 4. The velocity output removes most of it; on an ideal ramp almost all of it, through a real master less, because the
    //    velocity is a finite difference of a position that is itself moving.
    // 4. 速度输出去掉大部分；理想斜坡上几乎全部，通过真实主轴则少一些，因为那速度是一个本身在运动的位置的有限差分。
    EXPECT_LT(ideal_ff.worst_error, 0.1F * ideal.worst_error) << "ideal with feedforward: "
                                                             << (ideal_ff.worst_error * 1000.0F) << " mrad";
    EXPECT_LT(real_ff.worst_error, 0.35F * real.worst_error) << "real with feedforward: "
                                                            << (real_ff.worst_error * 1000.0F) << " mrad";
}

/**
 * @brief A ratio of zero is refused: it is a coupling that silently holds still while the master moves, and a stuck slave looks exactly the same. 零传动比被拒绝：那是一个主轴运动时悄悄不动的耦合，而卡住的从轴看起来一模一样。
 */
TEST(ElectronicGear, AZeroRatioIsRefusedRatherThanSilentlyDecoupling) {
    foc::core::ElectronicGear gear(ratio_recipe(0.0F));
    EXPECT_EQ(gear.validate(), foc::ErrorCode::Param);
    const auto out = gear.apply(1.0F, kDt);
    EXPECT_EQ(out.status, foc::ErrorCode::Param);
    EXPECT_FLOAT_EQ(out.slave_rad, 0.0F) << "a refused gear must not produce a target";

    gear.set_config(ratio_recipe(NAN));
    EXPECT_EQ(gear.validate(), foc::ErrorCode::Param);
    gear.set_config(ratio_recipe(1.0F));
    EXPECT_EQ(gear.validate(), foc::ErrorCode::Ok);

    // A non-finite master sample is refused without moving the unwrap state.
    // 非有限的主轴采样被拒绝，且不移动去卷绕状态。
    const auto before = gear.master_unwrapped_rad();
    EXPECT_EQ(gear.apply(NAN, kDt).status, foc::ErrorCode::Param);
    EXPECT_FLOAT_EQ(gear.master_unwrapped_rad(), before);
}
