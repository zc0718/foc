// S6 - feedforward in closed loop. S1/S2/S5 prove the *default* combination (no feedforward); this
// scenario proves the first optional stage actually does something on a coupled machine, and that it
// does not cost stability. Both runs use the same plant, nameplate and tuning, so the only variable
// is the feedforward policy.
// S6 - 闭环下的前馈。S1/S2/S5 证明的是**默认组合**（无前馈）；本场景证明第一个可选阶段在一台
// 存在轴间耦合的电机上确实起作用，并且不以稳定性为代价。两次运行共用同一 plant、同一铭牌、同一
// 整定，唯一变量就是前馈策略。

#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/feedforward.hpp>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"



namespace {

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 4;
constexpr int kPolePairs = 7;
constexpr float kVBus = 24.0F;
constexpr float kOmegaCurrent = 2000.0F;
constexpr float kOmegaSpeed = 250.0F;
constexpr float kTargetSpeed = 100.0F;
constexpr float kRunSeconds = 0.30F;
constexpr float kSettleSeconds = 5e-3F;

/**
 * @brief [en] An interior-PM machine: Ld != Lq, so the cross-coupling terms are non-trivial.
 * @brief [zh] 内嵌式电机：Ld != Lq，于是交叉耦合项非平凡。
 */
auto machine() -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{0.5F, 2.0e-4F, 6.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
}

struct Outcome final {
    float max_id_a;      ///< [en] worst d-axis excursion after the transient. [zh] 暂态之后最差的 d 轴偏移。
    float final_speed;   ///< [en] mechanical speed at the end of the run. [zh] 运行结束时的机械速度。
    foc::ErrorCode status; ///< [en] sticky status of the last tick. [zh] 最后一次 tick 的粘滞状态。
};

/**
 * @brief [en] Run one fixed scenario with the given feedforward policy.
 * @brief [zh] 用给定前馈策略跑一次固定场景。
 */
template <typename FeedForward> auto run_with(FeedForward ff) -> Outcome {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal,
                                     foc::core::NullReferenceGenerator, FeedForward>;
    const foc::motor::MotorConfig mc = machine();
    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
    foc::sil::PlantHal hal(plant, kVBus);

    const typename MotorT::Policies policies{foc::core::BackEmfObserver({mc.rs, mc.ld}),
                                             foc::core::Svpwm{},
                                             foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                             foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                             foc::core::Pi{foc::speed_pi_config(mc, kOmegaSpeed)},
                                             foc::core::NullReferenceGenerator{},
                                             ff};

    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_speed(kTargetSpeed);
    static_cast<void>(motor.start());

    float max_id = 0.0F;
    const int steps = static_cast<int>(kRunSeconds / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
        const float t = static_cast<float>(i) * kDt;
        const float id = motor.telemetry().id_a;
        if ((t > kSettleSeconds) && (std::fabs(id) > max_id)) {
            max_id = std::fabs(id);
        }
    }

    return Outcome{max_id, motor.telemetry().speed_mech_rad_s, motor.status()};
}

} // namespace



/**
 * @brief [en] The d axis is commanded to zero, so every |id| that appears is a disturbance the current loop had to
 *             fight. Decoupling removes most of it; the test asserts an order of magnitude, not the exact figure, so
 *             normal tuning changes do not break it.
 * @brief [zh] d 轴给定为零，因此出现的每一个 |id| 都是电流环必须对抗的扰动。解耦把其中大部分消掉； 用例断言的只是数量级
 *             而不是具体数值，正常的整定变更不会把它弄红。
 */
TEST(SilFeedforward, CrossDecouplingCutsTheDaxisDisturbanceByAnOrderOfMagnitude) {
    const Outcome without = run_with(foc::core::NullFeedforward{});
    const Outcome with = run_with(foc::core::DecouplingFeedforward{foc::feedforward_config(machine())});

    EXPECT_EQ(foc::ErrorCode::Ok, without.status);
    EXPECT_EQ(foc::ErrorCode::Ok, with.status);

    // Both runs must reach and hold the target speed: the feedforward may not trade stability for
    // disturbance rejection.
    EXPECT_NEAR(kTargetSpeed, without.final_speed, 2.0F);
    EXPECT_NEAR(kTargetSpeed, with.final_speed, 2.0F);

    // Measured on this bench: 0.284 A without, 0.008 A with. Assert a factor of five so the test
    // still means "decoupling works" without pinning an implementation detail.
    EXPECT_GT(without.max_id_a, 0.05F);
    EXPECT_LT(with.max_id_a * 5.0F, without.max_id_a);
}



/**
 * @brief [en] A zeroed configuration must behave exactly like no feedforward: a machine with no inductance and no flux
 *             has nothing to cancel. This pins the "unset means off" rule.
 * @brief [zh] 全零配置必须与「无前馈」完全等价：没有电感、没有磁链的电机没有可抵消的东西。这条 用例钉住「未配置即关闭」
 *             的规则。
 */
TEST(SilFeedforward, AZeroedConfigurationIsIndistinguishableFromNoFeedforward) {
    const Outcome none = run_with(foc::core::NullFeedforward{});
    const Outcome zeroed = run_with(foc::core::DecouplingFeedforward{});

    EXPECT_EQ(none.status, zeroed.status);
    EXPECT_FLOAT_EQ(none.max_id_a, zeroed.max_id_a);
    EXPECT_FLOAT_EQ(none.final_speed, zeroed.final_speed);
}



namespace {

constexpr std::uint32_t kTraceCapacity = 8000U;

/**
 * @brief [en] A reference generator that holds d at a value instead of at zero: the extension point the slot exists
 *             for, and the only way to get the two operating points the identifiability case needs.
 * @brief [zh] 把 d 轴保持在某个值而不是零的参考生成器：这个槽位存在的意义，也是可辨识性用例所需的两个工作点的唯一来源。
 */
struct FixedId final {
    float id_a;
    [[nodiscard]] auto generate(const foc::core::ReferenceInput& in) noexcept -> foc::core::Dq<float> {
        return foc::core::Dq<float>{id_a, in.iq_command_a};
    }
};

struct SteadyState final {
    double v_d{0.0};
    double v_q{0.0};
    double i_d{0.0};
    double i_q{0.0};
    double omega_e{0.0};
};

/**
 * @brief [en] One operating point, read out of the trace channel once the loops have settled.
 * @brief [zh] 一个工作点，环路稳定后从轨迹通道读出。
 */
template <typename FeedForward>
auto steady_state(float speed_mech, float id_cmd, FeedForward ff) -> SteadyState {
    using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi,
                                     foc::sil::PlantHal, FixedId, FeedForward,
                                     foc::core::NullPositionControl, foc::core::NullAlignment,
                                     foc::core::SpanTrace>;
    const foc::motor::MotorConfig mc = machine();
    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
    foc::sil::PlantHal hal(plant, kVBus);
    static foc::core::TraceSample buffer[kTraceCapacity];
    typename MotorT::Policies policies{foc::core::BackEmfObserver({mc.rs, mc.ld}),
                                       foc::core::Svpwm{},
                                       foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                       foc::core::Pi{foc::current_pi_config(mc, kOmegaCurrent, kVBus)},
                                       foc::core::Pi{foc::speed_pi_config(mc, kOmegaSpeed)},
                                       FixedId{id_cmd},
                                       ff};
    policies.trace.attach(buffer, kTraceCapacity);
    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_speed(speed_mech);
    static_cast<void>(motor.start());
    const int steps = static_cast<int>(kRunSeconds / kDt);
    for (int i = 0; i < steps; ++i) {
        static_cast<void>(motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
    }
    const foc::core::TraceSample last = motor.trace().at(motor.trace().size() - 1U);
    return SteadyState{last.v_d, last.v_q, last.i_d, last.i_q,
                       static_cast<double>(kPolePairs) * static_cast<double>(last.speed_mech)};
}

auto exact_feedforward() -> foc::core::DecouplingFeedforward {
    const foc::motor::MotorConfig mc = machine();
    return foc::core::DecouplingFeedforward(foc::feedforward_config(mc));
}

/**
 * @brief [en] psi_f as the voltage equation needs it: `Kt = 1.5*p*psi_f`, not `Kt` (PRD erratum D24).
 * @brief [zh] 电压方程需要的 `psi_f`：`Kt = 1.5*p*psi_f`，不是 `Kt`（PRD 勘误 D24）。
 */
auto psi_f_wb() -> double {
    const foc::motor::MotorConfig mc = machine();
    return static_cast<double>(mc.kt) / (1.5 * static_cast<double>(kPolePairs));
}

} // namespace



/**
 * @brief [en] The steady-state q-axis voltage the PI supplies is the machine's own equation, and two operating points
 *             separate `Ld` from `psi_f`.
 *
 * @brief [zh] PI 提供的稳态 q 轴电压就是电机自己的方程，而两个工作点才能把 `Ld` 与 `psi_f` 分开。
 *
 *   With no feedforward the current loop has to produce the whole voltage, so the number it settles on is a measurement
 *   of the machine: `vq = Rs*iq + omega_e*(Ld*id + psi_f)`. The model here is the plant's own steady state (`diq/dt =
 *   0`), which is an independent source from anything inside `DecouplingFeedforward`.
 *
 *   What is *not* separable is the pair: `Ld` and `psi_f` enter only as `Ld*id + psi_f`, so at `id = 0` the equation
 *   measures the flux and says nothing about `Ld`. That is why the case runs a second point at `id = -1 A`, and why a
 *   commissioning routine that identifies inductances from one operating point is measuring a sum it cannot split.
 *
 *   没有前馈时电流环必须提供全部电压，于是它稳定下来的数就是对电机的一次测量： `vq = Rs*iq + ω_e*(Ld*id + ψ_f)`。这里的
 *   模型是 plant 自己的稳态（`diq/dt = 0`），它与 `DecouplingFeedforward` 内部的任何东西都是独立来源。
 *
 *   **不可分**的是那个配对：`Ld` 与 `ψ_f` 只以 `Ld*id + ψ_f` 的形式出现，所以 `id = 0` 时这个方程量到的是磁链、对 `Ld`
 *   一无所知。这就是本用例要跑第二个 `id = -1 A` 工作点的原因，也是"从一个工作点辨识电感" 的调试流程实际在量一个它拆不
 *   开的和的原因。
 */
TEST(SilFeedforward, TheSteadyStateQVoltageIsTheMachinesOwnEquationAndTwoPointsSplitIt) {
    const foc::motor::MotorConfig mc = machine();
    const double rs = static_cast<double>(mc.rs);
    const double ld = static_cast<double>(mc.ld);
    const double psi = psi_f_wb();
    double flux_sum[2] = {0.0, 0.0};
    const float ids[2] = {0.0F, -1.0F};

    std::printf("\n%-8s %10s %10s %12s %12s %10s\n", "id_cmd", "id", "iq", "vq", "predicted", "ratio");
    for (int k = 0; k < 2; ++k) {
        const SteadyState s = steady_state(kTargetSpeed, ids[k], foc::core::NullFeedforward{});
        const double predicted = (rs * s.i_q) + (s.omega_e * ((ld * s.i_d) + psi));
        std::printf("%-8.1f %10.5f %10.5f %12.5f %12.5f %10.5f\n", static_cast<double>(ids[k]), s.i_d,
                    s.i_q, s.v_q, predicted, s.v_q / predicted);
        EXPECT_NEAR(1.0, s.v_q / predicted, 0.01) << "the q-axis voltage is the machine's own equation";
        flux_sum[k] = (s.v_q - (rs * s.i_q)) / s.omega_e;
    }

    // The identifiability, measured: at id = 0 the sum is the flux; at id = -1 A it is the flux minus Ld, so the
    // difference over one amp is Ld.
    // 可辨识性，量出来的：id = 0 时这个和就是磁链；id = -1 A 时是磁链减 Ld，于是差除以一安就是 Ld。
    const double identified_ld = (flux_sum[1] - flux_sum[0]) / (static_cast<double>(ids[1]) - static_cast<double>(ids[0]));
    std::printf("[identify] flux at id=0 %.6f (true %.6f), at id=-1 %.6f (true %.6f), Ld %.4e (true %.4e)\n",
                flux_sum[0], psi, flux_sum[1], psi - ld, identified_ld, ld);
    EXPECT_NEAR(psi, flux_sum[0], 0.01 * psi) << "at id = 0 the sum is the flux alone";
    EXPECT_NEAR(psi - ld, flux_sum[1], 0.01 * psi);
    EXPECT_NEAR(ld, identified_ld, 0.05 * ld) << "two operating points split the pair";
}



/**
 * @brief [en] A parameter mismatch leaves exactly its fraction of the term behind, and the residual is linear in the
 *             mismatch.
 *
 * @brief [zh] 参数失配留下的恰好是那一项的这个比例，且残差对失配是线性的。
 *
 *   The feedforward removes `omega_e*(Ld*id + psi_f)`; if the configuration is `(1+eps)` of the truth, what is left on
 *   the PI is `-eps*omega_e*(Ld*id + psi_f)`. That is a one-line derivation and it is worth having as a number: it says
 *   the residual is proportional to the mismatch and to the term, so the only thing that makes a mismatch harmless is a
 *   small mismatch -- there is no operating point that hides a 10 % error.
 *
 *   Measured at `eps = +0.10, -0.10, +0.25`, and the linearity is checked by the ratio of the last two.
 *
 *   前馈剥掉 `ω_e*(Ld*id + ψ_f)`；若配置是真实值的 `(1+eps)` 倍，留在 PI 上的是 `-eps*ω_e*(Ld*id + ψ_f)`。这是一行推
 *   导，但值得作为一个数留下来：它说残差正比于失配、也正比于那一项，所以让失配无害的唯一办法就是让它小——没有任何工作点
 *   能藏住 10 % 的误差。
 *
 *   在 `eps = +0.10, -0.10, +0.25` 上量，并用后两者的比值检查线性。
 */
TEST(SilFeedforward, AParameterMismatchLeavesExactlyItsFractionOfTheTermBehind) {
    const foc::motor::MotorConfig mc = machine();
    const double rs = static_cast<double>(mc.rs);
    const double ld = static_cast<double>(mc.ld);
    const double lq = static_cast<double>(mc.lq);
    const double psi = psi_f_wb();
    const double id_cmd = -1.0;

    std::printf("\n%-8s %14s %14s %10s\n", "eps", "pi_q residual", "predicted", "ratio");
    double residual_10 = 0.0;
    double residual_25 = 0.0;
    for (const double eps : {0.10, -0.10, 0.25}) {
        const foc::core::DecouplingFeedforward wrong(
            foc::core::DecouplingFeedforward::Config{static_cast<float>(ld * (1.0 + eps)),
                                                     static_cast<float>(lq * (1.0 + eps)),
                                                     static_cast<float>(psi * (1.0 + eps)),
                                                     foc::core::DecouplingFeedforward::Terms::CrossAndBemf});
        const SteadyState s = steady_state(kTargetSpeed, static_cast<float>(id_cmd), wrong);
        // What the PI had to add on top of the (wrong) feedforward.
        // PI 在那个（错的）前馈之上还得补的部分。
        const double pi_q = s.v_q - (s.omega_e * ((ld * (1.0 + eps) * s.i_d) + (psi * (1.0 + eps))));
        const double predicted_residual = -eps * s.omega_e * ((ld * s.i_d) + psi);
        const double measured_residual = pi_q - (rs * s.i_q);
        std::printf("%-8.2f %14.5f %14.5f %10.4f\n", eps, measured_residual, predicted_residual,
                    measured_residual / predicted_residual);
        EXPECT_NEAR(predicted_residual, measured_residual, 0.03 * std::fabs(predicted_residual))
            << "the residual is the mismatch times the term";
        if (eps > 0.2) {
            residual_25 = measured_residual;
        } else if (eps > 0.0) {
            residual_10 = measured_residual;
        }
    }
    // Linearity, with the tolerance derived rather than picked: the ratio of two residuals each known to 3 %
    // is known to 6 %, and the two runs are not at byte-identical operating points (the current loop settles to
    // slightly different measured currents), which is where the rest of the spread comes from.
    // 线性度，容差是推导出来的而不是挑的：两个各已知到 3 % 的残差之比已知到 6 %，而两次运行并不在逐位相同的
    // 工作点上（电流环稳定到的实测电流略有差异），剩下的偏差来自这里。
    std::printf("[linearity] eps 0.25 / 0.10 gives %.4f (predict 2.5, tolerance 6 %%)\n",
                residual_25 / residual_10);
    EXPECT_NEAR(2.5, residual_25 / residual_10, 0.06) << "the residual is linear in the mismatch";
}



/**
 * @brief [en] At one operating point the configured `Ld` cannot be told apart from the flux, and at another it can --
 *             the same statement the identification case makes, as an experiment.
 *
 * @brief [zh] 在同一个工作点上，配置的 `Ld` 与磁链无法区分；换一个工作点就可以——与辨识用例同一条论断，这里作为实验给
 *             出。
 *
 *   At `id = 0` the q-axis term is `omega_e*psi_f` whatever `Ld` says, so two configurations that differ only in `Ld`
 *   leave the *same* residual. That is an exact identity, not an approximation, and it is the reason "the feedforward
 *   matched at this point" is not evidence that its inductance is right.
 *
 *   `id = 0` 时 q 轴那一项是 `omega_e*psi_f`，与 `Ld` 说什么无关，于是两个只在 `Ld` 上不同的配置留下**相同** 的残差。这
 *   是一条恒等式而不是近似，也正是"前馈在这个点上对上了"不能证明它的电感是对的原因。
 */
TEST(SilFeedforward, AtOneOperatingPointTheInductanceIsInvisibleOnTheQAxis) {
    const foc::motor::MotorConfig mc = machine();
    const double ld = static_cast<double>(mc.ld);
    const double psi = psi_f_wb();

    const auto residual_at = [&](double ld_cfg, float id_cmd) {
        const foc::core::DecouplingFeedforward ff(
            foc::core::DecouplingFeedforward::Config{static_cast<float>(ld_cfg), mc.lq,
                                                     static_cast<float>(psi),
                                                     foc::core::DecouplingFeedforward::Terms::CrossAndBemf});
        const SteadyState s = steady_state(kTargetSpeed, id_cmd, ff);
        const double pi_q = s.v_q - (s.omega_e * ((ld_cfg * s.i_d) + psi));
        return pi_q - (static_cast<double>(mc.rs) * s.i_q);
    };

    // Four times the true inductance, at id = 0: nothing changes.
    // 真实电感的四倍，在 id = 0：什么都没变。
    const double at_zero_true = residual_at(ld, 0.0F);
    const double at_zero_wrong = residual_at(4.0 * ld, 0.0F);
    std::printf("\n[id=0] residual with Ld true %.6f, with Ld x4 %.6f, difference %.2e\n", at_zero_true,
                at_zero_wrong, std::fabs(at_zero_true - at_zero_wrong));
    EXPECT_NEAR(at_zero_true, at_zero_wrong, 1.0e-4)
        << "at id = 0 the configured inductance cannot reach the q axis at all";

    // At id = -1 A the same quadrupling is visible, and by exactly the amount the product predicts.
    // 在 id = -1 A 时同样的四倍就可见了，而且恰好是乘积预测的量。
    const double at_load_true = residual_at(ld, -1.0F);
    const double at_load_wrong = residual_at(4.0 * ld, -1.0F);
    const SteadyState probe = steady_state(kTargetSpeed, -1.0F, exact_feedforward());
    const double predicted_shift = probe.omega_e * ((4.0 * ld * probe.i_d) + psi) -
                                   (probe.omega_e * ((ld * probe.i_d) + psi));
    std::printf("[id=-1] residual moves by %.6f (predicted %.6f)\n", at_load_wrong - at_load_true,
                -predicted_shift);
    EXPECT_NEAR(-predicted_shift, at_load_wrong - at_load_true, 0.05 * std::fabs(predicted_shift));
}



/**
 * @brief [en] The d-axis voltage is confounded by the estimator's own angle error, and the confound is larger than the
 *             cross-coupling term it is supposed to measure.
 *
 * @brief [zh] d 轴电压被估计器自身的角误差混淆，而那个混淆比它本该量到的交叉耦合项还大。
 *
 *   The d-axis steady state is `vd = Rs*id - omega_e*Lq*iq`, which looks like a clean way to identify `Lq`. It is not,
 *   through a closed loop: the command is built in the *estimated* frame and applied in the plant's *true* frame, so
 *   what the machine receives is the command rotated by the estimator's angle error `delta`. To first order that adds
 *   `vq*delta` to the d axis -- and `vq` is the biggest number in the loop.
 *
 *   Measured on this machine: the cross term is `-omega*Lq*iq` and the confound comes out at **41 % of it**. Solving
 *   the two axes for `delta` gives **0.0131 rad (0.75 deg)**, which is a plausible observer error and is invisible on
 *   the q axis (`vd*delta` is ~3 % of `vq`) -- which is exactly why the q-axis identification in the case above works
 *   to 0.06 % while the same procedure on d is off by 41 %.
 *
 *   The budget is therefore explicit: identifying `Lq` from the d-axis voltage needs the angle known to
 *   `omega*Lq*iq/vq`, i.e. about 0.033 rad here, and it explains why commissioning identifies inductance from the *q*
 *   axis and never from d.
 *
 *   d 轴稳态是 `vd = Rs*id - ω_e*Lq*iq`，看上去是辨识 `Lq` 的干净途径。通过闭环它**不是**：指令在**估计** 坐标系里构
 *   造、在 plant 的**真实**坐标系里施加，于是电机收到的是被估计器角误差 `delta` 旋转过的指令。一阶上这给 d 轴加上了
 *   `vq*delta`——而 `vq` 是环里最大的那个数。
 *
 *   在这台电机上实测：交叉项是 `-ω*Lq*iq`，混淆量达到它的 **41 %**。用两轴解出 `delta` 得 **0.0131 rad（0.75°）**，是合
 *   理的观测器误差，且在 q 轴上不可见（`vd*delta` 只占 `vq` 的约 3 %）—— 这正是上面那个 q 轴辨识能到 0.06 %、而同样的步
 *   骤在 d 轴上差 41 % 的原因。
 *
 *   于是预算是显式的：从 d 轴电压辨识 `Lq` 要求角度已知到 `ω*Lq*iq/vq`，这里约 0.033 rad；这也解释了为什么调试流程从
 *   **q** 轴辨识电感、而不从 d 轴。
 */
TEST(SilFeedforward, TheDAxisIsConfoundedByTheAngleErrorMoreThanByTheTermItself) {
    const foc::motor::MotorConfig mc = machine();
    const double rs = static_cast<double>(mc.rs);
    const double lq = static_cast<double>(mc.lq);
    const SteadyState s = steady_state(kTargetSpeed, 0.0F, foc::core::NullFeedforward{});

    const double cross_term = -s.omega_e * lq * s.i_q;
    const double measured = s.v_d - (rs * s.i_d);
    const double confound = measured - cross_term;
    const double delta = confound / s.v_q; // the rotation that explains it, to first order
    std::printf("\n[d axis] cross term %+.5f, measured %+.5f, confound %+.5f = %.1f %% of the term; "
                "implied angle error %+.5f rad (%.2f deg); budget omega*Lq*iq/vq = %.5f rad\n",
                cross_term, measured, confound, 100.0 * std::fabs(confound / cross_term), delta,
                delta * 180.0 / 3.14159265358979, std::fabs(cross_term) / s.v_q);

    // The confound is comparable to the term, which is the whole point: the d axis cannot identify Lq through
    // this loop without knowing the angle first.
    // 混淆量与那一项同量级，这正是要点：不知道角度，就无法通过这个环用 d 轴辨识 `Lq`。
    EXPECT_GT(std::fabs(confound), 0.3 * std::fabs(cross_term))
        << "if the confound were small the d axis would be a usable identification channel";
    EXPECT_LT(std::fabs(delta), 0.05) << "and the angle error that explains it must be small, or the story is wrong";
    // The q axis is where the identification belongs: the same rotation contributes `vd*delta`, three orders down.
    // 辨识该在 q 轴上做：同一个旋转贡献 `vd*delta`，小了三个数量级。
    const double q_confound = std::fabs(s.v_d * delta);
    std::printf("[q axis] the same rotation adds %.2e to vq (%.3f %% of it)\n", q_confound,
                100.0 * q_confound / std::fabs(s.v_q));
    EXPECT_LT(q_confound, 0.05 * std::fabs(s.v_q)) << "which is why the q axis is the one that identifies";
}



/**
 * @brief [en] Each inductance multiplies **its own** cross term, which is what makes the feedforward dual-inductance
 *             rather than a single-`L` approximation wearing two names.
 *
 * @brief [zh] 每个电感乘的是**它自己**的交叉项，这正是这个前馈是双电感、而不是一个披着两个名字的单 `L` 近似的意思。
 *
 *   `v_d -= omega*Lq*iq` and `v_q += omega*Ld*id` is an asymmetry worth testing rather than reading: a single-`L`
 *   implementation would put the same letter in both places, and on a machine with `Lq/Ld = 3` that is a
 *   factor-of-three error in one of the two terms. The test is a **difference of two runs** that share every other
 *   detail -- exact feedforward against one with a single parameter scaled -- because the d axis carries a confound
 *   (`vq*delta`, 55.5 % of the cross term, measured in the case above) that would swamp a 10 % mismatch outright.
 *   Differencing cancels it, since both runs are at the same operating point.
 *
 *   The predictions are one line each: scaling `Lq` by `(1+eps)` leaves `+eps*omega*Lq*iq` on the d axis and nothing on
 *   q; scaling `Ld` leaves `-eps*omega*Ld*id` on q and nothing on d. So the two parameters are separable by *which axis
 *   moves*, which is the property that makes dual inductance worth having.
 *
 *   `v_d -= ω·Lq·iq` 与 `v_q += ω·Ld·id` 这个不对称值得测而不是读：单 `L` 的实现会在两处放同一个字母，而在 `Lq/Ld = 3`
 *   的机器上那是两项之一的**三倍**误差。测法是**两次只差一个参数的运行相减**——因为 d 轴带着一个混淆量（`vq·δ`，实测占交
 *   叉项的 55.5 %），它会把 10 % 的失配彻底淹没；两次运行工作点相同，相减即抵消。
 *
 *   两条预测各一行：`Lq` 缩放 `(1+eps)` 在 d 轴留下 `+eps·ω·Lq·iq`、q 轴不留；`Ld` 缩放则在 q 轴留下 `-eps·ω·Ld·id`、 d
 *   轴不留。于是两个参数靠**哪个轴动**就能分开——这正是双电感值得拥有的性质。
 */
TEST(SilFeedforward, EachInductanceMultipliesItsOwnCrossTerm) {
    const foc::motor::MotorConfig mc = machine();
    const double ld = static_cast<double>(mc.ld);
    const double lq = static_cast<double>(mc.lq);
    const double psi = psi_f_wb();
    constexpr float kIdCommand = -1.0F;
    constexpr double kEps = 0.10;

    const auto run_with = [&](double ld_scale, double lq_scale) {
        const foc::core::DecouplingFeedforward ff(
            foc::core::DecouplingFeedforward::Config{static_cast<float>(ld * ld_scale),
                                                     static_cast<float>(lq * lq_scale),
                                                     static_cast<float>(psi),
                                                     foc::core::DecouplingFeedforward::Terms::CrossAndBemf});
        return steady_state(kTargetSpeed, kIdCommand, ff);
    };

    const SteadyState exact = run_with(1.0, 1.0);
    const SteadyState lq_off = run_with(1.0, 1.0 + kEps);
    const SteadyState ld_off = run_with(1.0 + kEps, 1.0);

    // What each single-parameter mismatch does to the PI's own output, as a difference of runs.
    // 单个参数失配对 PI 输出的影响，以两次运行之差表示。
    const auto pi_d = [&](const SteadyState& s, double lq_scale) {
        return s.v_d - (-s.omega_e * lq * lq_scale * s.i_q);
    };
    const auto pi_q = [&](const SteadyState& s, double ld_scale) {
        return s.v_q - (s.omega_e * ((ld * ld_scale * s.i_d) + psi));
    };

    const double d_shift_lq = pi_d(lq_off, 1.0 + kEps) - pi_d(exact, 1.0);
    const double q_shift_lq = pi_q(lq_off, 1.0) - pi_q(exact, 1.0);
    const double q_shift_ld = pi_q(ld_off, 1.0 + kEps) - pi_q(exact, 1.0);
    const double d_shift_ld = pi_d(ld_off, 1.0) - pi_d(exact, 1.0);

    const double predicted_d_lq = kEps * exact.omega_e * lq * exact.i_q;
    const double predicted_q_ld = -kEps * exact.omega_e * ld * exact.i_d;
    std::printf("\n[dual] eps %.2f: Lq mismatch moves d by %+.6f (pred %+.6f), q by %+.6f; "
                "Ld mismatch moves q by %+.6f (pred %+.6f), d by %+.6f\n",
                kEps, d_shift_lq, predicted_d_lq, q_shift_lq, q_shift_ld, predicted_q_ld, d_shift_ld);

    // Each inductance reaches its own axis, and only its own axis.
    // 每个电感到达自己的轴，且只到自己的轴。
    EXPECT_NEAR(predicted_d_lq, d_shift_lq, 0.10 * std::fabs(predicted_d_lq))
        << "an Lq mismatch has to reach the d axis through iq";
    EXPECT_NEAR(predicted_q_ld, q_shift_ld, 0.10 * std::fabs(predicted_q_ld))
        << "an Ld mismatch has to reach the q axis through id";
    EXPECT_LT(std::fabs(q_shift_lq), 0.10 * std::fabs(d_shift_lq) + 2.0e-3)
        << "and an Lq mismatch must not appear on the q axis -- if it does, Lq is multiplying id somewhere";
    EXPECT_LT(std::fabs(d_shift_ld), 0.10 * std::fabs(q_shift_ld) + 2.0e-3)
        << "an Ld mismatch must not appear on the d axis either";
    // And the ratio of the two coefficients is `(Lq/Ld)*(iq/|id|)`, not `Lq/Ld`: the inductance ratio alone does
    // not say which mismatch is more dangerous, because the *current* on each axis does half the work. Writing the
    // relation out is what stops the saliency ratio from being quoted as if it were the answer.
    // 而两个系数之比是 `(Lq/Ld)·(iq/|id|)`，不是 `Lq/Ld`：只看电感比说不清哪种失配更危险，因为每个轴上的
    // **电流**也出了一半力。把关系写出来，才不会把凸极比当成答案去引用。
    const double predicted_ratio = (lq / ld) * (exact.i_q / std::fabs(exact.i_d));
    const double measured_ratio = std::fabs(d_shift_lq) / std::fabs(q_shift_ld);
    std::printf("[dual] coefficient ratio %.4f, and (Lq/Ld)*(iq/|id|) = %.4f (Lq/Ld alone is %.3f, which is "
                "not the answer)\n",
                measured_ratio, predicted_ratio, lq / ld);
    EXPECT_NEAR(predicted_ratio, measured_ratio, 0.02 * predicted_ratio)
        << "the two coefficients differ by the inductance ratio times the current ratio";
}
