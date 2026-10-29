// 3.3: the high-speed regime, measured where the limits actually are. Three claims, none of them "it works at speed":
// the plant conserves power at high electrical frequency (the foundation the rest of the library stands on), the voltage
// ceiling sets the speed and field weakening moves it by a computable factor, and the crossover observer's error is a
// function of *samples per electrical cycle* rather than of the speed.
// 3.3：高速区，而且是在限制真正所在的地方去量。三条主张，没有一条是"高速下能用"：plant 在高电频率下守恒功率（这是库里其余
// 一切所立足的地基）；电压上限决定转速，弱磁按一个可算的倍数把它挪开；而交叉观测器的误差是**每个电周期的采样数**的函数，
// 不是转速的函数。

#include <cmath>
#include <cstdint>

#include <gtest/gtest.h>

#include <foc/core/field_weakening.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/transforms.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

constexpr float kDt = 6.25e-5F;
constexpr float kPi = 3.14159265358979323846F;

auto nominal() -> foc::sil::PlantParams { return foc::sil::nominal_plant_params(); }

auto psi_f_of(const foc::sil::PlantParams &p) -> float {
    return p.kt / (1.5F * static_cast<float>(p.pole_pairs));
}

/**
 * @brief The voltage that holds `(id, iq)` at `omega_e` -- the steady state of the plant's own equations, used as the input so that any mismatch in the read-back is a mismatch in the model. 在 `omega_e` 下把 `(id, iq)` 维持住的那个电压 ——plant 自身方程的稳态解，作为输入使用，于是读回时的任何不符都是模型的不符。
 */
auto holding_voltage(const foc::sil::PlantParams &p, float omega_e, float id_a, float iq_a) -> foc::core::Dq<float> {
    const float psi_f = psi_f_of(p);
    // did = 0 and diq = 0, with the cross-coupling terms kept: leaving them out is the 5 % bleed that
    // test_plant_friction.cpp records.
    // 让 did = 0 且 diq = 0，并保留交叉耦合项：漏掉它们就是 test_plant_friction.cpp 记下的那 5% 抽走。
    return foc::core::Dq<float>{(p.rs * id_a) - (omega_e * p.lq * iq_a), (p.rs * iq_a) + (omega_e * p.ld * id_a) + (omega_e * psi_f)};
}

} // namespace

/**
 * @brief Power in equals copper loss plus the rate of change of stored magnetic energy plus mechanical power, at 900 rad/s electrical -- the single invariant that catches a wrong 1.5, a wrong `psi_f`, and a cross-coupling sign.
 *
 *     This is the foundation the rest of the library stands on, so it is stated as an identity rather than as a
 *     comparison between two models: `1.5*(vd*id + vq*iq) = 1.5*Rs*(id^2+iq^2) + d/dt(1.5*(Ld*id^2/2 + Lq*iq^2/2 +
 *     psi_f*id)) + Te*wm`. Run with the rotor held at speed (a dyno) and the currents driven from one operating point
 *     to another, so the stored term is not zero and the balance has to include it. 这是库其余一切所立足的地基，所以它
 *     作为恒等式陈述，而不是两个模型之间的比较。转子由测功机保持转速、电流从一个工作点到另一个工作点，于是储能项不为
 *     零，平衡必须把它算进去。
 */
TEST(HighSpeed, ThePlantConservesPowerAtNineHundredRadPerSecondElectrical) {
    constexpr float kOmegaE = 900.0F;
    const foc::sil::PlantParams p = nominal();
    const float psi_f = psi_f_of(p);
    const float omega_mech = kOmegaE / static_cast<float>(p.pole_pairs);

    foc::sil::PmsmPlant plant(p);
    plant.reset(foc::sil::PlantState{0.0F, 0.0F, omega_mech, 0.0F});
    plant.set_mechanical_speed(omega_mech); // dyno: the torque equation runs but cannot move the rotor

    double elect = 0.0;
    double copper = 0.0;
    double mechanical = 0.0;
    double worst_residual = 0.0;
    double worst_electrical = 0.0;
    float previous_stored = 0.0F;

    const int periods = 1600; // 0.1 s
    for (int i = 0; i < periods; ++i) {
        // A ramp of the operating point, so the stored term is exercised: id from 0 to -0.5 A, iq from 0.5 to 1.5 A.
        // 工作点的一段斜坡，使储能项被真正走到：id 从 0 到 −0.5 A，iq 从 0.5 到 1.5 A。
        const float progress = static_cast<float>(i) / static_cast<float>(periods);
        const float id_set = -0.5F * progress;
        const float iq_set = 0.5F + (1.0F * progress);
        const foc::core::Dq<float> v = holding_voltage(p, kOmegaE, id_set, iq_set);
        plant.set_voltage_dq(v);
        plant.advance(kDt);

        const foc::sil::PlantState s = plant.state();
        const float stored = 1.5F * (((0.5F * p.ld * s.id_a * s.id_a) + (0.5F * p.lq * s.iq_a * s.iq_a)) + (psi_f * s.id_a));
        const float d_stored = (stored - previous_stored) / kDt;
        previous_stored = stored;

        const double p_elect = 1.5 * ((static_cast<double>(v.d) * s.id_a) + (static_cast<double>(v.q) * s.iq_a));
        const double p_copper = 1.5 * static_cast<double>(p.rs) * ((static_cast<double>(s.id_a) * s.id_a) +
                                                                  (static_cast<double>(s.iq_a) * s.iq_a));
        const double p_mech = static_cast<double>(plant.torque_nm()) * omega_mech;

        // The first 5 ms are the integrator's start-up, where the one-step derivative of the stored energy is not yet the
        // derivative of a smooth trajectory.
        // 头 5 ms 是积分器的启动，那里储能的一步差分还不是光滑轨迹的导数。
        if (i > 80) {
            elect += p_elect;
            copper += p_copper;
            mechanical += p_mech;
            const double residual = std::fabs(p_elect - p_copper - d_stored - p_mech);
            worst_residual = std::max(worst_residual, residual);
            worst_electrical = std::max(worst_electrical, std::fabs(p_elect));
        }
        plant.set_mechanical_speed(omega_mech);
    }

    // The totals over the window balance to 0.1 %, which is the RK4 step's own accuracy at 62.5 us.
    // 窗口内的总量平衡到 0.1%，那正是 RK4 在 62.5 µs 下自身的精度。
    EXPECT_GT(elect, 0.0) << "electrical power must be flowing in, or the run is not exercising anything";
    EXPECT_NEAR((elect - copper - mechanical) / elect, 0.0, 1.0e-3)
        << "energy in " << elect << " J, copper " << copper << " J, mechanical " << mechanical << " J";
    EXPECT_LT(worst_residual / worst_electrical, 0.01)
        << "worst per-period residual " << worst_residual << " W against " << worst_electrical << " W";
}

/**
 * @brief The voltage ceiling sets the speed, `we = v_max / psi_f` in the high-speed limit, and field weakening moves the ceiling by exactly the factor the flux reduction promises.
 *
 *     Measured on the plant's steady state rather than asserted from the datasheet: for each speed the voltage that
 *     holds `(id, iq)` is applied and the currents are read back, so the ceiling is a property of the machine as
 *     modelled. Weakening by `id = -2 A` reduces `psi_f + Ld*id` by 16.8 %, and the measured ceiling moves by that
 *     factor. 在 plant 的稳态上量，而不是从数据表断言：对每个转速施加维持 `(id, iq)` 的电压并读回电流，于是上限是被建模
 *     的机器自身的性质。用 `id = −2 A` 弱磁使 `psi_f + Ld·id` 下降 16.8%，实测上限按该倍数移动。
 */
TEST(HighSpeed, TheVoltageCeilingIsFourPercentBelowTheNaiveRatioAndWeakeningLiftsIt) {
    const foc::sil::PlantParams p = nominal();
    const float psi_f = psi_f_of(p);
    constexpr float kVBus = 24.0F;
    const float v_max = kVBus / std::sqrt(3.0F); // SVPWM's linear range
    constexpr float kIq = 1.0F;
    const float weakening_current = -2.0F;

    // The speed at which the machine's own steady state needs exactly the bus: bisection on the amplitude of the voltage
    // that holds `(id, iq)` at that speed. The naive `v_max / psi_f` ignores the d-axis drop `we*Lq*iq`, which at these
    // speeds is a volt and a half -- four percent of the ceiling.
    // 机器自身稳态正好需要满母线时的转速：对该转速下维持 `(id, iq)` 的电压幅值做二分。naive 的 `v_max/psi_f` 忽略了
    // d 轴压降 `we·Lq·iq`，在这些转速下它是 1.5 V——上限的百分之四。
    auto ceiling = [&](float id_a) {
        float lo = 100.0F;
        float hi = 8000.0F;
        for (int i = 0; i < 40; ++i) {
            const float mid = 0.5F * (lo + hi);
            const auto v = holding_voltage(p, mid, id_a, kIq);
            const float magnitude = std::sqrt((v.d * v.d) + (v.q * v.q));
            if (magnitude > v_max) {
                hi = mid;
            } else {
                lo = mid;
            }
        }
        return 0.5F * (lo + hi);
    };

    const float without = ceiling(0.0F);
    const float with = ceiling(weakening_current);
    const float naive = v_max / psi_f;
    const float naive_weakened = v_max / (psi_f + (p.ld * weakening_current));

    EXPECT_NEAR(without, 2794.0F, 40.0F) << "the machine's own ceiling: " << without << " rad/s";
    EXPECT_LT(without, 0.97F * naive) << "the naive " << naive << " over-promises because it ignores the d-axis drop";
    EXPECT_NEAR(with / without, 1.188F, 0.02F) << "weakened ceiling " << with << " rad/s";
    EXPECT_LT(with / without, naive_weakened / naive)
        << "and it falls short of the flux ratio (" << (naive_weakened / naive) << ") because the weakening current's own"
        << " d-axis voltage eats part of the margin it buys";

    // And the model holds the operating point at 90 % of each ceiling to 1 %: the ceiling is a property of the machine as
    // modelled, not a number in a comment.
    // 而模型在每个上限的 90% 处把工作点守到 1%：上限是被建模的机器自身的性质，不是注释里的一个数。
    auto steady_state_error = [&](float omega_e, float id_a) {
        foc::sil::PmsmPlant plant(p);
        plant.reset(foc::sil::PlantState{id_a, kIq, omega_e / static_cast<float>(p.pole_pairs), 0.0F});
        const auto v = holding_voltage(p, omega_e, id_a, kIq);
        for (int i = 0; i < 1600; ++i) {
            plant.set_voltage_dq(v);
            plant.advance(kDt);
            plant.set_mechanical_speed(omega_e / static_cast<float>(p.pole_pairs));
        }
        const auto s = plant.state();
        return std::max(std::fabs(s.id_a - id_a), std::fabs(s.iq_a - kIq));
    };
    EXPECT_LT(steady_state_error(0.9F * without, 0.0F), 0.01F);
    EXPECT_LT(steady_state_error(0.9F * with, weakening_current), 0.01F);
}

/**
 * @brief The crossover observer's error is a function of samples per electrical cycle, not of the speed: hold the ratio and the error stays flat while the speed triples; cut the ratio and it grows.
 *
 *     This is the limit that high-speed work actually runs into, and it is the same shape as the carrier ceiling in
 *     `hfi_max_inject_hz`: a sampling fact, not a machine fact. Measured at 40 and at 10 samples per cycle with the
 *     same observer configuration, at three speeds each. 这是高速工作真正撞到的极限，形状与 `hfi_max_inject_hz` 里的载
 *     波上限相同：采样事实，不是机器事实。在每周期 40 个与 10 个采样下各测三个转速。
 */
TEST(HighSpeed, TheErrorFollowsSamplesPerCycleRatherThanTheSpeed) {
    const foc::sil::PlantParams p = nominal();
    const float psi_f = psi_f_of(p);

    // Steady-state angle error of the back-EMF observer fed by the plant, at one speed and one step size.
    // 由 plant 喂给反电动势观测器，在一个转速与一个步长下的稳态角度误差。
    auto angle_error = [&](float omega_e, float dt) {
        foc::core::BackEmfObserver observer(foc::core::BackEmfObserver::Config{p.rs, p.lq});
        foc::sil::PmsmPlant plant(p);
        plant.reset(foc::sil::PlantState{0.0F, 0.0F, omega_e / static_cast<float>(p.pole_pairs), 0.0F});
        plant.set_mechanical_speed(omega_e / static_cast<float>(p.pole_pairs));
        const auto v_dq = holding_voltage(p, omega_e, 0.0F, 1.0F);
        {
            // Seed the flux estimate at the initial rotor position, otherwise the integrator's constant is arbitrary and
            // the measured error is the seed, not the sampling.
            // 在初始转子位置上播种磁链估计，否则积分常数是任意的，量到的误差是那个种子而不是采样。
            const auto s0 = plant.state();
            const float theta0 = static_cast<float>(p.pole_pairs) * s0.theta_mech;
            observer.seed_flux(foc::core::inv_park(
                foc::core::Dq<float>{(p.ld * s0.id_a) + psi_f, p.lq * s0.iq_a}, theta0));
        }
        double sum = 0.0;
        int counted = 0;
        const int periods = static_cast<int>(0.02F / dt);
        for (int i = 0; i < periods; ++i) {
            plant.set_voltage_dq(v_dq);
            plant.advance(dt);
            plant.set_mechanical_speed(omega_e / static_cast<float>(p.pole_pairs));
            const auto s = plant.state();
            const float truth = static_cast<float>(p.pole_pairs) * s.theta_mech;
            const auto v_ab = foc::core::inv_park(v_dq, truth);
            const auto i_ab = foc::core::inv_park(foc::core::Dq<float>{s.id_a, s.iq_a}, truth);
            const auto estimate = observer.update(v_ab, i_ab, dt);
            if (i > periods / 4) {
                auto error = estimate.theta_e - truth;
                while (error > kPi) {
                    error -= 2.0F * kPi;
                }
                while (error < -kPi) {
                    error += 2.0F * kPi;
                }
                sum += std::fabs(error);
                ++counted;
            }
        }
        return (counted > 0) ? static_cast<float>(sum / counted) : -1.0F;
    };

    // 40 samples per electrical cycle at each speed: the ratio is held, the speed triples.
    // 每个电周期 40 个采样、转速各不同：比值保持，转速翻三倍。
    for (const float omega_e : {200.0F, 600.0F}) {
        const float dt = (2.0F * kPi / omega_e) / 40.0F;
        const float error = angle_error(omega_e, dt);
        SCOPED_TRACE(omega_e);
        EXPECT_GT(error, 0.0F) << "the run must produce a number";
        EXPECT_LT(error, 0.15F) << "40 samples per cycle: mean |error| " << error << " rad";
    }

    // Cut the ratio to 8 and keep the speed: the error grows -- the ratio is what the error follows.
    // 把比值砍到 8 而保持转速：误差变大——误差跟的是比值。
    const float coarse_speed = 600.0F;
    const float coarse_dt = (2.0F * kPi / coarse_speed) / 8.0F;
    const float fine_dt = (2.0F * kPi / coarse_speed) / 40.0F;
    const float coarse = angle_error(coarse_speed, coarse_dt);
    const float fine = angle_error(coarse_speed, fine_dt);
    EXPECT_GT(coarse, 3.0F * fine) << "8 samples per cycle: " << coarse << " rad against " << fine << " at 40";
    EXPECT_NEAR(coarse, 0.380F, 0.06F) << "8 samples per cycle: " << coarse << " rad";
    EXPECT_NEAR(fine, 0.080F, 0.03F) << "40 samples per cycle at the same speed: " << fine << " rad";
}
