// 2.4: friction compensation, measured as a *dead band* rather than as "it feels better". The claim is arithmetic -- a
// proportional loop stops where the torque it can still produce falls under the static friction, so it leaves a band of
// `static / kp`; a Coulomb term latched onto the direction of travel moves that wall by its own size. If the measurement
// does not land on those numbers, the mechanism is not the mechanism.
// 2.4：摩擦补偿，用**死区**来测，而不是用"感觉好多了"。声称是算术——比例环停在"它还能产生的力矩低于静摩擦"的地方，于是
// 留下 `静摩擦 / kp` 的带子；一个锁在运动方向上的库仑项把那堵墙挪动它自己的大小。如果测量不落在那两个数上，那个机理就不是
// 这个机理。

#include <algorithm>
#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include <foc/core/friction_comp.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

constexpr float kDt = 6.25e-5F;
constexpr float kKp = 5.0F;          // N*m/rad
constexpr float kKd = 0.02F;         // N*m*s/rad, the velocity damping a real position loop has
constexpr float kStaticNm = 0.020F;  // the machine's breakaway
constexpr float kCoulombNm = 0.010F; // the machine's kinetic friction
constexpr float kViscous = 1.0e-3F;  // N*m*s/rad
constexpr float kPlantBand = 0.005F; // the plant's numerical at-rest band, a decade below the module's
constexpr float kModuleBand = 0.05F; // the module's at-rest band
constexpr float kIMax = 3.0F;

/**
 * @brief A machine with friction big enough to stop a small command dead, and a rotor light enough to move.
 */
auto friction_plant() -> foc::sil::PlantParams {
    foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    params.b = kViscous;
    params.t_static_nm = kStaticNm;
    params.t_coulomb_nm = kCoulombNm;
    params.omega_eps_rad_s = kPlantBand;
    return params;
}

auto friction_recipe(float coulomb_nm, float static_nm) -> foc::core::FrictionConfig {
    foc::core::FrictionConfig config{};
    config.coulomb_nm = coulomb_nm;
    config.static_nm = static_nm;
    config.viscous_nm_per_rad_s = kViscous;
    config.rest_band_rad_s = kModuleBand;
    return config;
}

/**
 * @brief The voltage that makes both current derivatives zero at the state given -- a current source, so the only thing under test is the mechanical law. Same construction as `test_plant_friction.cpp`, and for the same reason: a `vq` that ignores the d-axis cross-coupling bleeds 5 % of the q current. 让当前状态下两个电流导数都为零的电压——一个电流源，于是被测的只有力学。与 `test_plant_friction.cpp` 同一构造、同一理由：忽略 d 轴交叉耦合的 `vq` 会抽走 5% 的 q 轴电流。
 */
auto current_source(const foc::sil::PlantParams &params, float omega_mech, float iq_a, float psi_f)
    -> foc::core::Dq<float> {
    const float omega_e = omega_mech * static_cast<float>(params.pole_pairs);
    return foc::core::Dq<float>{-(omega_e * params.lq * iq_a), (params.rs * iq_a) + (omega_e * psi_f)};
}

/**
 * @brief A position axis: proportional-derivative loop on the mechanical angle, friction compensation in front of the current source, and three ways of producing the compensation so the latch can be compared with the law it replaces. 一根位置轴：机械角上的比例-微分环、电流源之前的摩擦补偿，以及三种产生补偿的方式，好把锁存与它替代的规律对比。
 */
class Axis final {
  public:
    enum class Law : std::uint8_t {
        Compensated, ///< the module's latched Coulomb term
        Naive,       ///< `coulomb * sign(measured speed)`, no latch -- the law the header argues against
        None,        ///< no compensation
    };

    Axis(Law law, float coulomb_comp_nm, float static_comp_nm, bool noisy)
        : law_(law), params_(friction_plant()), plant_(params_), module_(friction_recipe(coulomb_comp_nm, static_comp_nm)),
          psi_f_(params_.kt / (1.5F * static_cast<float>(params_.pole_pairs))), noisy_(noisy) {
        plant_.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
    }

    /**
     * @brief Run to the end of `seconds` with this position reference, remembering the ripple of the demanded torque over the last 20 % of it.
     */
    void run(float ref_rad, float seconds) {
        const int ticks = static_cast<int>(seconds / kDt);
        const int settle_from = ticks - (ticks / 5);
        float lowest = 1.0e9F;
        float highest = -1.0e9F;
        for (int i = 0; i < ticks; ++i) {
            const foc::sil::PlantState now = plant_.state();
            furthest_rad_ = std::max(furthest_rad_, std::fabs(now.theta_mech));
            const float speed = now.omega_mech + (noisy_ ? noise() : 0.0F);
            // Position plus velocity damping, which is what a real axis has and what keeps a stepped loop out of
            // saturation. The dead band is set by the proportional path at rest, where the damping term is zero, so the
            // arithmetic the cases check is unchanged by it. Without it the loop saturates against `i_max` on the first
            // period and limit-cycles at 125 mrad -- a property of a badly scaled rig, and the first version of this
            // file measured that instead of the friction.
            // 位置加转速阻尼，这是真实轴有的东西，也把阶跃后的环路挡在饱和之外。死区由静止时的比例通路决定，而那里阻尼项为零，
            // 所以用例检查的算术不受它影响。没有它，环路在第一个周期就顶到 `i_max` 并在 125 mrad 上做极限环——那是台架标度
            // 不对的性质，而本文件第一版量的正是它，不是摩擦。
            const float demand = (kKp * (ref_rad - now.theta_mech)) - (kKd * now.omega_mech);
            float compensation = 0.0F;
            if (law_ == Law::Compensated) {
                const foc::core::FrictionOutput out = module_.step(0.0F, speed, demand);
                compensation = out.torque_nm;
                last_output_ = out;
            } else if (law_ == Law::Naive) {
                compensation = (module_.config().coulomb_nm * ((speed > 0.0F) ? 1.0F : -1.0F)) +
                               (module_.config().viscous_nm_per_rad_s * speed);
            }
            const float torque = demand + compensation;
            if (i >= settle_from) {
                lowest = std::min(lowest, torque);
                highest = std::max(highest, torque);
            }
            const float iq = std::max(-kIMax, std::min(kIMax, torque / params_.kt));
            plant_.set_voltage_dq(current_source(params_, now.omega_mech, iq, psi_f_));
            plant_.advance(kDt);
        }
        ripple_nm_ = highest - lowest;
    }

    [[nodiscard]] auto position_rad() const -> float { return plant_.state().theta_mech; }
    [[nodiscard]] auto speed_rad_s() const -> float { return plant_.state().omega_mech; }
    [[nodiscard]] auto ripple_nm() const -> float { return ripple_nm_; }
    [[nodiscard]] auto last_output() const -> const foc::core::FrictionOutput & { return last_output_; }

    /**
     * @brief The furthest the axis has ever been from its start. For a step that overshoots and comes back, this is what "did it move" means -- the final position alone would call a large excursion "no motion". 轴离起点最远到过哪里。对会过冲并回落的阶跃，这才是"它动了没有"的意思——只看最终位置会把一次大行程叫作"没动"。
     */
    [[nodiscard]] auto furthest_rad() const -> float { return furthest_rad_; }

  private:
    /**
     * @brief Zero-mean measurement noise on the speed, so "at rest" means what it means on a real encoder. Deterministic so the case is repeatable, and used only where the sign of a noisy zero is the point. 转速上的零均值测量噪声，让"静止"的含义与真实编码器一致。确定性的，用例可重复；只在"噪声零点的符号"正是要点处使用。
     */
    auto noise() -> float {
        // A 16-bit LFSR: cheap, deterministic, and its distribution does not matter -- only that it crosses zero.
        state_ = static_cast<std::uint16_t>((state_ * 25173U) + 13849U);
        return (static_cast<float>(state_) / 65535.0F - 0.5F) * 0.02F; // +-0.01 rad/s
    }

    Law law_;
    foc::sil::PlantParams params_;
    foc::sil::PmsmPlant plant_;
    foc::core::FrictionComp module_;
    float psi_f_;
    bool noisy_;
    float ripple_nm_{0.0F};
    float furthest_rad_{0.0F};
    foc::core::FrictionOutput last_output_{foc::ErrorCode::State, 0.0F, false, false};
    std::uint16_t state_{1U};
};

/**
 * @brief The smallest reference step that moves the axis at all: the dead band, by bisection.
 *
 *     A proportional loop on a machine with stiction has a *dead band*, not a smooth residual: below a certain error
 *     the torque it can still ask for is under the breakaway and the axis does not move at all. That threshold is the
 *     quantity friction compensation exists to change, so it is what the case measures -- bisection on the reference
 *     step, from rest, 0.15 s per candidate (well past the 28 ms mechanical period). Single-step residuals were tried
 *     first and abandoned: where inside the band the trajectory happens to stop is not reproducible, and one mechanism
 *     read 2.9, 1.1 and 0.24 mrad at three different step sizes. 比例环在有静摩擦的机器上有一个**死区**而不是平滑的残
 *     差：误差低于某值时它还能要到的力矩低于起动点，轴根本不动。那个阈值正是摩擦补偿存在的意义所在，所以用例量它——对参
 *     考阶跃做二分，从静止起步，每个候选 0.15 s（远超过 28 ms 的机械周期）。单次阶跃残差先试过并被放弃：轨迹恰好在带内
 *     哪里停住不可复现，同一个机理在三个不同阶跃下读出 2.9、1.1、0.24 mrad。
 */
auto deadband_rad(Axis::Law law, float coulomb_comp_nm, float static_comp_nm, bool negative = false) -> float {
    float lo = 0.0002F; // 0.2 mrad: below anything physical here
    float hi = 0.012F;  // 12 mrad: three times the uncompensated band
    for (int i = 0; i < 9; ++i) {
        const float mid = 0.5F * (lo + hi);
        Axis axis(law, coulomb_comp_nm, static_comp_nm, false);
        axis.run(negative ? -mid : mid, 0.15F);
        if (axis.furthest_rad() > (0.5F * mid)) {
            hi = mid;
        } else {
            lo = mid;
        }
    }
    return hi;
}

} // namespace

/**
 * @brief Without compensation the dead band is `static / kp`; with a Coulomb term on the arrival side it is `(static - coulomb) / kp`. Two arithmetic numbers, and the difference between them is the whole capability.
 *
 *     `0.020 / 5` = **4.0 mrad** and `0.010 / 5` = **2.0 mrad**, measured as **4.003** and **2.021**. Neither depends
 *     on the loop's bandwidth, the inertia or the bus voltage -- only on the two friction numbers and the gain -- which
 *     is what makes them a criterion rather than a description. `0.020/5` = **4.0 mrad**、`0.010/5` = **2.0 mrad**，实
 *     测 **4.003** 与 **2.021**。两者都不取决于环路带宽、惯量或母线电压 ——只取决于两个摩擦数和增益——这正是它们成为判据
 *     而不是描述的原因。
 */
TEST(FrictionComp, TheDeadBandHalvesBecauseItIsStaticMinusCoulombOverKp) {
    const float off = deadband_rad(Axis::Law::None, 0.0F, 0.0F);
    const float on = deadband_rad(Axis::Law::Compensated, kCoulombNm, kStaticNm);

    EXPECT_NEAR(off, kStaticNm / kKp, 0.0002F) << "measured " << (off * 1000.0F) << " mrad against 4.0 predicted";
    EXPECT_NEAR(on, (kStaticNm - kCoulombNm) / kKp, 0.0002F)
        << "measured " << (on * 1000.0F) << " mrad against 2.0 predicted";
    EXPECT_LT(on / off, 0.55F) << "off " << (off * 1000.0F) << " mrad, on " << (on * 1000.0F) << " mrad";

    // The same two numbers going the other way. A module whose latched direction can only ever have come from a speed
    // reference would pass a positive step and fail this one -- which is exactly why the demand is allowed to set the
    // direction: with no reference it is the only quantity that knows which side of the target the error is on.
    // 同样的两个数反过来走。一个"锁存方向只能来自速度给定"的模块会通过正向阶跃而在这一条上失败——这正是允许指令设定方向
    // 的原因：没有给定时，它是唯一知道误差在目标哪一侧的量。
    const float off_negative = deadband_rad(Axis::Law::None, 0.0F, 0.0F, true);
    const float on_negative = deadband_rad(Axis::Law::Compensated, kCoulombNm, kStaticNm, true);
    EXPECT_NEAR(off_negative, off, 0.0002F) << "negative step: " << (off_negative * 1000.0F) << " mrad";
    EXPECT_NEAR(on_negative, on, 0.0002F) << "negative step: " << (on_negative * 1000.0F) << " mrad";
}

/**
 * @brief An over-estimate under the breakaway narrows the band further -- `(0.020 - 0.015) / 5` = 1.0 mrad, measured 1.007 -- which is why the estimate has to be a measurement: the band follows it wherever it goes. 低于起动点的过估把带子进一步压窄——`(0.020 − 0.015)/5` = 1.0 mrad，实测 1.007——这正是估计值必须来自测量的理由：带子跟着它走到哪算哪。
 */
TEST(FrictionComp, AnOverEstimateUnderTheBreakawayNarrowsTheBandFurther) {
    const float over = deadband_rad(Axis::Law::Compensated, 0.015F, kStaticNm);
    EXPECT_NEAR(over, (kStaticNm - 0.015F) / kKp, 0.0002F) << "measured " << (over * 1000.0F) << " mrad";
}

/**
 * @brief An estimate above the breakaway makes the compensation a motor: with the reference held at zero and no demand at all, the axis walks 5.6 mrad away from where it was told to stay.
 *
 *     This is not a tuning accident, it is the definition -- a term larger than the static friction moves the rotor
 *     with no command behind it. It is why `validate()` refuses the one version that can be seen in the numbers
 *     (kinetic above breakaway) and why the honest defence against the version that cannot be seen is an estimate that
 *     came from a measurement. Measured: **5.625 mrad** with `(0.030, 0.030)` against a machine whose breakaway is
 *     0.020, and **0.000 mrad** with the correct `(0.010, 0.020)`. 高于起动点的估计会让补偿变成一台电机：参考保持为零、
 *     **完全没有任何指令**，轴自己走了 5.6 mrad。这不是整定失误，而是定义——大于静摩擦的项在没有任何指令的情况下推动转
 *     子。这正是 `validate()` 拒绝那个在数字上看得见的形态（动摩擦高于起动）、而对付看不见的那个形态唯一诚实的办法是量
 *     出来的估计值的原因。实测：`(0.030, 0.030)` 对一台起动点 0.020 的机器走了 **5.625 mrad**，正确的 `(0.010, 0.020)`
 *     是 **0.000 mrad**。
 */
TEST(FrictionComp, AnEstimateAboveTheBreakawayWalksTheAxisWithNoCommandAtAll) {
    Axis correct(Axis::Law::Compensated, kCoulombNm, kStaticNm, false);
    correct.run(0.0F, 0.50F);
    EXPECT_LT(std::fabs(correct.position_rad()) * 1000.0F, 0.1F)
        << "the right estimate must not move a commanded-still axis: " << (correct.position_rad() * 1000.0F) << " mrad";

    Axis over(Axis::Law::Compensated, 0.030F, 0.030F, false);
    over.run(0.0F, 0.50F);
    const float walked_mrad = std::fabs(over.position_rad()) * 1000.0F;
    EXPECT_GT(walked_mrad, 3.0F) << "a term above the breakaway is a motor; it walked only " << walked_mrad << " mrad";
    EXPECT_NEAR(walked_mrad, 5.6F, 1.5F) << "measured walk " << walked_mrad << " mrad";
}

/**
 * @brief At rest the compensation is *constant*, and the law it replaces is a square wave: same run, same noise, two laws, one window.
 *
 *     A Coulomb term multiplied by `sign(measured speed)` is the obvious implementation and it is wrong at exactly the
 *     place the stage is needed: at rest the measured speed is zero-mean noise, so the sign flips, and a flipping sign
 *     is a torque oscillation of amplitude twice the term. Measured with +-0.01 rad/s of speed noise: **20.03 mN*m
 *     peak-to-peak for the naive law** -- twice the 10 mN*m term, as it must be -- and **0.036 mN*m for the latch**,
 *     which is only the viscous term acting on the noise it was handed. The latch is 550 times quieter, and both park
 *     the axis within a milliradian of the same place. 静止时补偿是**常数**，而被它替代的那条规律是一条方波：同一段运
 *     行、同一份噪声、两条规律、一个窗口。拿 `sign(实测转速)` 乘库仑项是最直观的实现，而它恰恰在需要这一级的地方是错
 *     的：静止时实测转速是零均值噪声，符号因此翻转，而翻转的符号就是幅度为该项两倍的力矩振荡。在 ±0.01 rad/s 的转速噪声
 *     下实测：**朴素规律峰峰 20.03 mN·m**—— 必然是 10 mN·m 的两倍——而**锁存 0.036 mN·m**，那只是粘性项作用在喂给它的噪
 *     声上。锁存安静 550 倍，而两者把轴停在同一处附近（毫弧度以内）。
 */
TEST(FrictionComp, TheLatchIsWhatKeepsTheCompensationFromBecomingASquareWaveAtRest) {
    Axis naive(Axis::Law::Naive, kCoulombNm, kStaticNm, true);
    naive.run(0.020F, 0.60F);
    Axis latched(Axis::Law::Compensated, kCoulombNm, kStaticNm, true);
    latched.run(0.020F, 0.60F);

    EXPECT_NEAR(naive.ripple_nm(), 2.0F * kCoulombNm, 0.001F) << "the square wave is twice the term, by construction";
    EXPECT_LT(latched.ripple_nm(), 5.0e-5F) << "latched ripple " << (latched.ripple_nm() * 1000.0F) << " mN*m";
    EXPECT_LT(latched.ripple_nm() / naive.ripple_nm(), 0.01F) << "the latch is worth 100x at least";

    // Both settle, and both park within a milliradian of each other: the noise is given no vote on where the axis rests.
    // 两者都稳定下来，停处相差在一毫弧度以内：噪声对"轴停在哪"没有投票权。
    EXPECT_LT(std::fabs(naive.speed_rad_s()), 0.01F);
    EXPECT_LT(std::fabs(latched.speed_rad_s()), 0.01F);
    EXPECT_NEAR(latched.position_rad(), naive.position_rad(), 0.0015F);
}

/**
 * @brief The stall flag: at rest while the loop asks for more than the breakaway estimate. Not while moving, and not while the demand is still under it.
 *
 *     The flag is worth having because it separates "the axis is holding a load" from "the axis is not going anywhere",
 *     and those two look identical in the position signal. What it cannot do is say which of the two causes it: a
 *     stalled axis and an estimate that is too low produce the same reading, and the boundary page says so. 堵转标志：
 *     静止而环路要求的力矩超过起动点估计。运动时不报，指令仍低于它时也不报。这个标志值得有，因为位置信号里 "轴在顶住负
 *     载"与"轴哪儿也不去"完全一样，而它把这两者分开。它做不到的是说出原因：堵住的轴与偏低的估计产生同样的读数，边界页写
 *     明了这一点。
 */
TEST(FrictionComp, TheStallFlagNeedsBothStillnessAndADemandAboveTheBreakaway) {
    // The flag is a decision about two numbers, so it is tested as a decision -- on the module, where the demand is an
    // input rather than something to arrange through a plant.
    // 标志是关于两个数的决定，所以当作决定来测——直接对模块测，指令是输入，而不是要靠 plant 去安排出来的东西。
    foc::core::FrictionComp module(friction_recipe(kCoulombNm, kStaticNm));

    auto out = module.step(0.0F, 0.0F, 0.025F); // at rest, asked for more than the 0.020 N*m breakaway
    EXPECT_TRUE(out.at_rest);
    EXPECT_TRUE(out.stalled) << "a resting axis asked for 0.025 against a 0.020 breakaway is not going anywhere";

    out = module.step(0.0F, 5.0F, 0.025F); // moving: the demand is being obeyed, so nothing is stuck
    EXPECT_FALSE(out.at_rest);
    EXPECT_FALSE(out.stalled) << "a moving axis is not stalled however large the demand";

    out = module.step(0.0F, 0.0F, 0.015F); // at rest with a demand the friction can hold
    EXPECT_TRUE(out.at_rest);
    EXPECT_FALSE(out.stalled) << "holding a load is not the same as being stalled";

    // And on the plant: a settled axis reports at_rest every period, and never stalled.
    // 以及在 plant 上：稳定下来的轴每周期都报 at_rest，且从不报 stalled。
    Axis still(Axis::Law::Compensated, kCoulombNm, kStaticNm, false);
    still.run(0.020F, 0.40F);
    EXPECT_TRUE(still.last_output().at_rest);
    EXPECT_FALSE(still.last_output().stalled);
}

/**
 * @brief The estimate is refused when it describes a machine that cannot exist, and accepted when it does. 当估计值描述一台不可能存在的机器时被拒绝，存在时被接受。
 */
TEST(FrictionComp, AnEstimateThatWouldMakeTheCompensationAMotorIsRefused) {
    foc::core::FrictionComp module(friction_recipe(kCoulombNm, kStaticNm));
    EXPECT_EQ(module.validate(), foc::ErrorCode::Ok);

    // Kinetic friction above breakaway: the compensation would push a resting axis by itself.
    // 动摩擦高于起动：补偿会自己推动一根静止的轴。
    module.set_config(friction_recipe(0.030F, 0.020F));
    EXPECT_EQ(module.validate(), foc::ErrorCode::Param);
    const foc::core::FrictionOutput refused = module.step(0.0F, 0.0F, 0.0F);
    EXPECT_EQ(refused.status, foc::ErrorCode::Param);
    EXPECT_FLOAT_EQ(refused.torque_nm, 0.0F) << "a refused estimate must not add torque to a real axis";

    module.set_config(friction_recipe(-0.001F, 0.020F));
    EXPECT_EQ(module.validate(), foc::ErrorCode::Param);
    module.set_config(foc::core::FrictionConfig{NAN, kStaticNm, 0.0F, kModuleBand});
    EXPECT_EQ(module.validate(), foc::ErrorCode::Param);
    module.set_config(foc::core::FrictionConfig{kCoulombNm, kStaticNm, -1.0F, kModuleBand});
    EXPECT_EQ(module.validate(), foc::ErrorCode::Param);
    module.set_config(foc::core::FrictionConfig{kCoulombNm, kStaticNm, 0.0F, -1.0F});
    EXPECT_EQ(module.validate(), foc::ErrorCode::Param);
}

/**
 * @brief The latched direction follows the reference when there is one, the demand when it is larger than the term, and holds otherwise -- the three rules, one case each. 锁存方向在有给定时跟给定，在指令大于该项时跟指令，其余保持——三条规则，各一条断言。
 */
TEST(FrictionComp, TheSignComesFromTheReferenceThenTheDemandThenStaysWhereItWas) {
    foc::core::FrictionComp module(friction_recipe(kCoulombNm, kStaticNm));
    // A reference wins even with the demand pointing the other way: the caller knows where it is going.
    // 给定优先，即使指令指向另一边：调用方知道它要去哪。
    auto out = module.step(-1.0F, 0.0F, 0.5F);
    EXPECT_LT(out.torque_nm, 0.0F);
    EXPECT_FLOAT_EQ(module.sign(), -1.0F);

    // No reference, a demand above the term: the demand decides, and it turns the latch around.
    // 无给定、指令大于该项：由指令决定，并把锁存转过来。
    out = module.step(0.0F, 0.0F, 0.050F);
    EXPECT_GT(out.torque_nm, 0.0F);
    EXPECT_FLOAT_EQ(module.sign(), 1.0F);

    // No reference, a demand under the term: nothing informative, so the latch holds -- this is the chatter guard.
    // 无给定、指令小于该项：没有信息，锁存保持——这就是防抖的那道护栏。
    out = module.step(0.0F, 0.0F, -0.004F);
    EXPECT_GT(out.torque_nm, 0.0F) << "a demand under the term is not a direction";
    EXPECT_FLOAT_EQ(module.sign(), 1.0F);

    // At rest with a still rotor the viscous term contributes nothing, so the output is the Coulomb term itself.
    // 静止处转子的粘性项不贡献，所以输出就是库仑项本身。
    EXPECT_NEAR(out.torque_nm, kCoulombNm, 1.0e-6F);
    EXPECT_TRUE(out.at_rest);
}
