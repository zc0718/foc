// The dual-inductance observer, measured against the single-`L` one it generalises. The saliency term the single-`L`
// model discards is a **closed-form** angle bias; this file holds the algebra, the law, and the floor it is read on.
// 双电感观测器，对着它所推广的单 `L` 观测器量。单 `L` 模型丢掉的凸极项是一个**闭式**角度偏置；
// 本文件留下代数、定律，以及它被读在其上的那个地板。

#include <cmath>
#include <complex>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/dual_inductance_observer.hpp>
#include <foc/core/transforms.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"

namespace foc {

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

/**
 * @brief [en] An interior machine: the nominal nameplate with the two inductances pulled apart.
 *
 * @brief [zh] 一台内嵌式电机：铭牌与标称机相同，只把两个电感分开。
 *
 *     `Lq/Ld = 3` at `psi_f = 4.76 mWb` makes the discarded `L1` term worth `(ld - lq) * i_d / psi_f` in angle -- 0.063
 *     rad at 1.5 A, which is four times the harness floor measured below. `Lq/Ld = 3`、`psi_f = 4.76 mWb` 时，被丢掉的
 *     `L1` 项折算成角度是 `(ld − lq)·i_d/psi_f`——1.5 A 时 0.063 rad，是下面量到的夹具地板的四倍。
 */
auto salient_spec() -> sil::MachineSpec {
    sil::MachineSpec spec = sil::nominal_spec();
    spec.ld = 1.0e-4F;
    spec.lq = 3.0e-4F;
    return spec;
}

auto psi_f_of(const sil::MachineSpec& spec) -> float { return spec.kt / (1.5F * static_cast<float>(spec.pole_pairs)); }

/** @brief [en] The step the harness and the observers share, s. @brief [zh] 夹具与观测器共用的步长，s。 */
constexpr float kDt = 5.0e-5F;

/**
 * @brief [en] Electrical speed the table is measured at, rad/s.
 * @details [en] Chosen to sit **under** the harness floor rather than against it: the flux integrator is a backward
 *               Euler step, so the angle it reports lags the truth by about `w_e*dt/2`, which is 0.0025 rad here and
 *               0.02 rad at 800 rad/s -- the latter comparable to the bias under test. Measurements at 800 rad/s were
 *               made and are not quoted, because at that speed the harness is part of the answer.
 * @details [zh] 取在夹具地板**之下**而不是与它相抵：磁链积分器是后向欧拉步，报出的角度比真值滞后约 `ω_e·dt/2`，这里是
 *               0.0025 rad，800 rad/s 时 0.02 rad——后者与被测偏置同量级。800 rad/s 也量过但没有引用，因为那个速度下夹具
 *               本身已经是答案的一部分。
 */
constexpr float kOmegaE = 100.0F;

/**
 * @brief [en] Wrap to [0, 2pi); the observers' wrappers are file-local, so the harness carries its own.
 * @brief [zh] 归一到 [0, 2π)；观测器的归一函数是文件内部的，夹具自带一份。
 */
auto wrap_two_pi(float angle) -> float {
    while (angle < 0.0F) {
        angle += core::kTwoPi<float>;
    }
    while (angle >= core::kTwoPi<float>) {
        angle -= core::kTwoPi<float>;
    }
    return angle;
}

/**
 * @brief [en] Wrap to (-pi, pi].
 * @brief [zh] 归一到 (−π, π]。
 */
auto wrap_pi(float angle) -> float {
    while (angle <= -core::kPi<float>) {
        angle += core::kTwoPi<float>;
    }
    while (angle > core::kPi<float>) {
        angle -= core::kPi<float>;
    }
    return angle;
}

/**
 * @brief [en] The stator flux a salient machine has at a given rotor angle: the relation the observer inverts.
 * @brief [zh] 凸极机在给定转子角下的定子磁链：观测器要反解的那个关系。
 */
auto stator_flux(const sil::MachineSpec& spec, float theta, float id_a, float iq_a) -> core::Ab<float> {
    const float l0 = 0.5F * (spec.ld + spec.lq);
    const float l1 = 0.5F * (spec.ld - spec.lq);
    const float psi_f = psi_f_of(spec);
    // `psi_f*e^(j*theta) + L0*i + L1*conj(i)*e^(j*2*theta)`, written out in components so no complex type is needed.
    // `psi_f·e^(jθ) + L0·i + L1·conj(i)·e^(j2θ)`，按分量写开，不需要复数类型。
    const float c1 = std::cos(theta);
    const float s1 = std::sin(theta);
    const float c2 = std::cos(2.0F * theta);
    const float s2 = std::sin(2.0F * theta);
    // The current in the alpha-beta frame, built from the rotor-frame pair.
    // αβ 坐标系下的电流，由转子坐标系的那一对构造。
    const float i_alpha = (id_a * c1) - (iq_a * s1);
    const float i_beta = (id_a * s1) + (iq_a * c1);
    return core::Ab<float>{((psi_f * c1) + (l0 * i_alpha) + (l1 * ((i_alpha * c2) + (i_beta * s2)))),
                           ((psi_f * s1) + (l0 * i_beta) + (l1 * ((i_alpha * s2) - (i_beta * c2))))};
}

/**
 * @brief [en] Mean absolute angle error of an observer on a speed-locked plant driven by the exact steady-state d-q
 *             voltage, rad.
 *
 * @brief [zh] 转速锁定、加精确稳态 d-q 电压的 plant 上，某个观测器的平均角度误差，rad。
 *
 *     Why not the switching rig: at `psi_f = 4.76 mWb` the back-EMF at 800 rad/s is 3.8 V, so a few tenths of a volt of
 *     dead-time volt-seconds is already 0.1 rad of flux angle -- the size of the saliency term under test and common to
 *     both observers, which buries exactly the difference this file is about. Measured there, both read 0.13 to 0.15
 *     rad and the dual-`L` looks useless; the impairment was the instrument. The inverter's own contribution is a
 *     different row's claim. 为什么不用开关台架：`psi_f = 4.76 mWb` 时 800 rad/s 的反电势只有 3.8 V，几百毫伏的死区伏秒
 *     就已经是 0.1 rad 磁链角 ——与被测的凸极项同量级且两个观测器共有，正好埋掉本文件要谈的差别。在那里量，两者都读到
 *     0.13~0.15 rad，双电感看起来一无是处；那是仪器坏了。逆变器自身的贡献是另一行主张。
 *
 * @param observer [en] Anything with `seed_flux` and `update`; it is stepped once per plant step.
 * @param omega_e [en] electrical speed, rad/s (the rotor is speed-locked).
 * @param id_a [en] d-axis current to hold, A.
 * @param iq_a [en] q-axis current to hold, A.
 */
template <typename Observer>
auto plant_error(Observer& observer, float omega_e, float id_a, float iq_a) -> float {
    const sil::MachineSpec spec = salient_spec();
    const float psi_f = psi_f_of(spec);
    sil::PmsmPlant plant(sil::plant_params_from(spec));
    plant.set_mechanical_speed(omega_e / static_cast<float>(spec.pole_pairs));
    // The steady-state d-q voltage that holds this point on the machine itself.
    // 在机器本身上维持这个工作点的稳态 d-q 电压。
    const core::Dq<float> v_dq{spec.rs * id_a - (omega_e * spec.lq * iq_a),
                                (spec.rs * iq_a) + (omega_e * ((spec.ld * id_a) + psi_f))};
    plant.set_voltage_dq(v_dq);
    // Settle, so the currents are the commanded ones and not the transient.
    // 稳定下来，让电流就是指令值而不是暂态。
    for (int k = 0; k < 6000; ++k) {
        plant.advance(kDt);
    }
    // Seed with the stator flux the machine actually has, which is what the align phase can compute from the nameplate
    // and the measured current. The seed sets the integration constant only: the model error under test is a *readout*
    // error and is present at every step either way.
    // 用机器实际的定子磁链播种，那是对齐阶段能从铭牌和实测电流算出来的。种子只决定积分常数：被测的模型误差是**读数**
    // 误差，无论如何每一步都在。
    observer.seed_flux(stator_flux(spec, wrap_two_pi(plant.theta_mech_rad() * static_cast<float>(spec.pole_pairs)),
                                  id_a, iq_a));
    float sum = 0.0F;
    int count = 0;
    for (int k = 0; k < 4000; ++k) {
        plant.advance(kDt);
        const float truth = wrap_two_pi(plant.theta_mech_rad() * static_cast<float>(spec.pole_pairs));
        const core::Ab<float> i_ab = core::clarke(plant.current_uvw());
        const core::Ab<float> v_ab = core::inv_park(v_dq, truth);
        sum += std::fabs(wrap_pi(observer.update(v_ab, i_ab, kDt).theta_e - truth));
        ++count;
    }
    return sum / static_cast<float>(count);
}

/** @brief [en] The shipped single-`L` observer (`lq`), on that plant. @brief [zh] 出厂的单 `L` 观测器（用 `lq`），同一 plant。 */
auto single_l_error_at(float omega_e, float id_a) -> float {
    const sil::MachineSpec spec = salient_spec();
    core::BackEmfObserver observer(core::BackEmfObserver::Config{spec.rs, spec.lq});
    return plant_error(observer, omega_e, id_a, 0.5F);
}

/** @brief [en] The dual-inductance observer on that plant. @brief [zh] 双电感观测器，同一 plant。 */
auto dual_l_error_at(float omega_e, float id_a) -> float {
    const sil::MachineSpec spec = salient_spec();
    core::DualInductanceObserver observer(
        core::DualInductanceObserver::Config{spec.rs, spec.ld, spec.lq, psi_f_of(spec)});
    return plant_error(observer, omega_e, id_a, 0.5F);
}

/**
 * @brief [en] The bias the single-`L` model cannot avoid, rad: `atan((ld - lq) * i_d / psi_f)`.
 *
 * @brief [zh] 单 `L` 模型无法避免的偏置，rad：`atan((ld − lq)·i_d/psi_f)`。
 *
 *     Derived in the rotor frame. Subtracting `Lq*i` from the stator flux leaves `psi_f + (ld - lq)*i_d` along the
 *     rotor axis -- the `j*Lq*i_q` part cancels exactly, which is why `i_q` does not appear -- so the reported angle is
 *     off by that ratio's arctangent, **independent of speed**: no integration time makes it go away, so a drive that
 *     pairs `MtpaReference` with a single-`L` observer carries a fixed steering error proportional to the `i_d` the
 *     reference itself asks for. 在转子坐标系里推。从定子磁链里减去 `Lq·i` 后，转子轴上剩下 `psi_f + (ld −
 *     lq)·i_d`——`j·Lq·i_q` 部分精确抵消，这就是 `i_q` 不出现的原因——所以报出的角度偏这个比值的反正切，而且**与转速无关
 *     **：没有任何积分时间能让它消失，所以把 `MtpaReference` 与单 `L` 观测器配在一起的驱动，带着一个正比于参考自己要求
 *     的 `i_d` 的固定指向误差。
 */
auto single_l_law(float id_a) -> float {
    const sil::MachineSpec spec = salient_spec();
    return std::fabs(std::atan((spec.ld - spec.lq) * id_a / psi_f_of(spec)));
}

} // namespace

TEST(DualInductanceObserver, TheQuadraticInvertsTheFluxRelationExactly) {
  // The algebra alone, with no integration and no plant in the way: seed the observer with the stator flux a salient
  // machine has at a known angle, hand it a voltage that leaves that flux unchanged (`v = Rs*i`), and it must answer
  // that angle. Six currents x four angles, at the float floor rather than at a tolerance chosen to be safe.
  // 只有代数，中间没有积分也没有 plant：用凸极机在已知角度下的定子磁链播种，交给它一个不改变该磁链的电压
  // （`v = Rs·i`），它就必须报出那个角度。六个电流 × 四个角度，判据是 float 地板而不是一个挑来求稳的容差。
  const sil::MachineSpec spec = salient_spec();
  core::DualInductanceObserver observer(
      core::DualInductanceObserver::Config{spec.rs, spec.ld, spec.lq, psi_f_of(spec)});
  float worst = 0.0F;
  for (const float theta : {0.0F, 0.7F, 2.5F, 5.1F}) {
    for (const float id : {0.0F, -0.5F, -1.5F, 1.0F, -2.5F, 2.5F}) {
      for (const float iq : {0.0F, 0.5F}) {
        observer.reset();
        observer.seed_flux(stator_flux(spec, theta, id, iq));
        const core::Ab<float> i_ab = core::inv_park(core::Dq<float>{id, iq}, theta);
        const core::Ab<float> v_ab{spec.rs * i_ab.alpha, spec.rs * i_ab.beta};
        const core::ObsState state = observer.update(v_ab, i_ab, kDt);
        const float error = std::fabs(wrap_pi(state.theta_e - theta));
        worst = std::max(worst, error);
        EXPECT_LT(error, 1.0e-3F) << "theta " << theta << ", i_d " << id << ", i_q " << iq;
      }
    }
  }
  std::printf("\n[dual-L] algebra alone: worst angle error %.3e rad over 48 (theta, i_d, i_q) combinations\n",
              static_cast<double>(worst));
}

TEST(DualInductanceObserver, TheSingleLBiasIsTheDiscardedSaliencyTerm) {
  // The law, measured on this repository's own plant. Two claims at once, both falsifiable: the size is
  // `atan((ld - lq)*i_d/psi_f)`, and it grows monotonically with `i_d` -- 0.5 A is where the harness floor (0.0025 rad at
  // this speed) is still a tenth of it, and 2.5 A is where the bias is 37 times the floor.
  // 定律，在本仓库自己的 plant 上量。同时主张两件事，两件都可证伪：大小是 `atan((ld − lq)·i_d/psi_f)`，并且随 `i_d`
  // 单调增长——0.5 A 处夹具地板（该速度下 0.0025 rad）还是它的十分之一，2.5 A 处偏置是地板的 37 倍。
  std::printf("[dual-L] salient machine, iq = 0.5 A, %g rad/s: single-L against the law\n", static_cast<double>(kOmegaE));
  std::printf("     %-10s %12s %12s %8s\n", "i_d A", "measured", "law", "ratio");
  float previous = 0.0F;
  for (const float id : {-0.5F, -1.0F, -1.5F, -2.0F, -2.5F}) {
    const float measured = single_l_error_at(kOmegaE, id);
    const float law = single_l_law(id);
    std::printf("     %-10.2f %12.6f %12.6f %8.3f\n", static_cast<double>(id), static_cast<double>(measured),
                static_cast<double>(law), static_cast<double>(measured / law));
    EXPECT_NEAR(law, measured, 0.35 * law) << "i_d = " << id;
    EXPECT_GT(measured, previous) << "i_d = " << id << ": the bias must grow with the current that causes it";
    previous = measured;
  }
}

TEST(DualInductanceObserver, TheDualInductanceAnswerHasNoDaxisTermLeft) {
  // The same table through the dual-inductance observer, and the claim is not "smaller" but "**flat**": what is left is
  // the harness floor plus a residual that does not respond to `i_d`, so the error at 2.5 A is the error at 0.5 A. That
  // is the difference between a model error and a numerical one, and it is the property `MtpaReference` needs -- a bias
  // that grows with `i_d` is exactly what a reference generator defined by `i_d` cannot tolerate.
  // 同一张表换成双电感观测器，主张不是"更小"而是"**平的**"：剩下的是夹具地板加上一个对 `i_d` 没有响应的残余，所以
  // 2.5 A 处的误差等于 0.5 A 处。这正是模型误差与数值误差的区别，也正是 `MtpaReference` 需要的性质——一个随 `i_d`
  // 增长的偏置，恰恰是由 `i_d` 定义的参考生成器容不下的东西。
  std::printf("[dual-L] the same table through the dual-inductance observer\n");
  std::printf("     %-10s %12s %12s %10s\n", "i_d A", "single-L", "dual-L", "ratio");
  float smallest = 1.0e9F;
  float largest = 0.0F;
  for (const float id : {-0.5F, -1.0F, -1.5F, -2.0F, -2.5F}) {
    const float single = single_l_error_at(kOmegaE, id);
    const float dual = dual_l_error_at(kOmegaE, id);
    std::printf("     %-10.2f %12.6f %12.6f %10.2f\n", static_cast<double>(id), static_cast<double>(single),
                static_cast<double>(dual), static_cast<double>(single / dual));
    smallest = std::min(smallest, dual);
    largest = std::max(largest, dual);
  }
  std::printf("[dual-L] dual-L spread over a 5x current range: %.6f to %.6f rad (ratio %.3f)\n",
              static_cast<double>(smallest), static_cast<double>(largest), static_cast<double>(largest / smallest));
  // Flat: a 5x change of the current that causes the bias moves the dual-L answer by less than 15 %.
  // 平：让偏置产生的那个电流变化 5 倍，双电感的答案动不到 15%。
  EXPECT_LT(largest, 1.15F * smallest) << "flat in i_d";
  // And at the largest current it is more than four times better, which is the reason the type exists.
  // 而在最大电流处好过四倍，这就是这个类型存在的理由。
  EXPECT_LT(dual_l_error_at(kOmegaE, -2.5F) * 4.0F, single_l_error_at(kOmegaE, -2.5F));
}

TEST(DualInductanceObserver, InTheSurfaceMountedLimitItIsTheShippedAlgebra) {
  // On `Ld == Lq` the quadratic degenerates and the answer must be the one the shipped observer already computes --
  // `arg(psi - L*i)` here, `-c/b` there. Compared **without integrating**, because the two accumulate their flux
  // differently (one takes `L*di` inside the integrator, the other subtracts `L0*i` only at readout, which telescopes to
  // the same thing in exact arithmetic but not in float over thousands of steps). No integration, no plant: this is the
  // algebra, which is what a cross-check of new algebra should be.
  // `Ld == Lq` 时二次式退化，答案必须就是出厂观测器已经在算的那个——这里是 `arg(psi − L·i)`，那里是 `−c/b`。
  // 比较时**不积分**，因为两者积累磁链的方式不同（一个把 `L·di` 放进积分器，另一个只在读数时减 `L0·i`；精确算术
  // 里等价，float 下几千步后不等价）。不积分、不接 plant：这才是代数，而交叉校验新代数就该如此。
  const sil::MachineSpec spec = sil::nominal_spec();
  const float psi_f = psi_f_of(spec);
  float worst = 0.0F;
  for (const float theta : {0.0F, 1.1F, 3.3F, 5.5F}) {
    for (const float id : {0.0F, -1.0F, 1.5F}) {
      for (const float iq : {0.0F, 0.5F, -1.0F}) {
        const core::Ab<float> flux = stator_flux(spec, theta, id, iq);
        const core::Ab<float> i_ab = core::inv_park(core::Dq<float>{id, iq}, theta);
        const core::Ab<float> v_ab{spec.rs * i_ab.alpha, spec.rs * i_ab.beta};
        core::BackEmfObserver shipped(core::BackEmfObserver::Config{spec.rs, spec.lq});
        core::DualInductanceObserver dual(core::DualInductanceObserver::Config{spec.rs, spec.ld, spec.lq, psi_f});
        // The shipped observer's state is `psi_be = psi - L*i`, so it is seeded with the difference; the dual-L one keeps
        // the stator flux itself. Seeding each with what it actually holds is what makes the comparison one of algebra.
        // 出厂观测器的状态是 `psi_be = psi − L·i`，所以用差值播种；双电感那个保留定子磁链本身。各自用"它真正持有的
        // 量"播种，比较的才是代数。
        shipped.seed_flux(core::Ab<float>{flux.alpha - (spec.lq * i_ab.alpha), flux.beta - (spec.lq * i_ab.beta)});
        dual.seed_flux(flux);
        const float a = shipped.update(v_ab, i_ab, kDt).theta_e;
        const float b = dual.update(v_ab, i_ab, kDt).theta_e;
        const float gap = std::fabs(wrap_pi(a - b));
        worst = std::max(worst, gap);
        EXPECT_LT(gap, 1.0e-5F) << "theta " << theta << ", i_d " << id << ", i_q " << iq;
        EXPECT_LT(std::fabs(wrap_pi(b - theta)), 1.0e-3F) << "and both must find the angle they were handed";
      }
    }
  }
  std::printf("[dual-L] surface-mounted algebra: shipped against dual-L, worst gap %.3e rad\n", static_cast<double>(worst));
}

TEST(DualInductanceObserver, SeedAtInstallsTheStatorFluxThisObserverHolds) {
  // `seed_at(theta, i)` exists so a hand-over does not have to know that this observer's state is the **stator** flux.
  // Checked the only way that means anything: seed from an angle and a current, then hand it a voltage that leaves the
  // flux where it is (`v = Rs*i`) -- the answer must be that angle, at every current. Seeding with `psi_f*e^(j*theta)`
  // instead leaves the current's own flux out of the integration constant, and a pure integrator never forgets.
  // `seed_at(θ, i)` 存在的意义，是交接方不必知道本观测器的状态是**定子**磁链。只用一个有意义的方式检查：由角度和电流播种，
  // 再交给它一个不改变磁链的电压（`v = Rs·i`）——答案必须是那个角度，在每个电流下都是。若改用 `psi_f·e^(jθ)` 播种，就把
  // 电流自身的磁链漏在积分常数之外，而纯积分器不会遗忘。
  const sil::MachineSpec spec = salient_spec();
  core::DualInductanceObserver observer(
      core::DualInductanceObserver::Config{spec.rs, spec.ld, spec.lq, psi_f_of(spec)});
  float worst = 0.0F;
  for (const float theta : {0.0F, 1.3F, 4.4F}) {
    for (const float id : {0.0F, -2.5F, 2.0F}) {
      for (const float iq : {0.5F, -1.5F}) {
        const core::Ab<float> i_ab = core::inv_park(core::Dq<float>{id, iq}, theta);
        observer.reset();
        observer.seed_at(theta, i_ab);
        const core::Ab<float> v_ab{spec.rs * i_ab.alpha, spec.rs * i_ab.beta};
        const float error = std::fabs(wrap_pi(observer.update(v_ab, i_ab, kDt).theta_e - theta));
        worst = std::max(worst, error);
        EXPECT_LT(error, 1.0e-3F) << "theta " << theta << ", i_d " << id << ", i_q " << iq;
      }
    }
  }
  // And the seed it computes is not the rotor flux: at 2.5 A the difference is large enough to matter, which is why the
  // one-argument hand-over was not good enough for this member.
  // 而它算出的种子不是转子磁链：2.5 A 时差别大到足以算数，这就是单参数交接对这个成员不够用的原因。
  const core::Ab<float> i_ab = core::inv_park(core::Dq<float>{-2.5F, 0.0F}, 0.0F);
  observer.reset();
  observer.seed_at(0.0F, i_ab);
  const float stator = observer.flux().alpha;
  const float rotor_only = psi_f_of(spec);
  const float offset = std::atan2(stator - rotor_only, rotor_only);
  std::printf("[dual-L] seed_at with i_d = -2.5 A: stator %.6e Wb against rotor-only %.6e Wb, offset %.4f rad\n",
              static_cast<double>(stator), static_cast<double>(rotor_only), static_cast<double>(offset));
  std::printf("[dual-L] seed_at: worst angle error %.3e rad over 18 (angle, i_d, i_q) combinations\n",
              static_cast<double>(worst));
  EXPECT_GT(std::fabs(offset), 0.02F) << "the current's own flux is not negligible at this current";
}

TEST(DualInductanceObserver, TheSecondRootIsNotARotorPositionAndAWrongSeedIsPermanent) {
  // Two statements that together close the boundary the capability table listed as an open hole, and the second one is
  // the opposite of what the hole assumed.
  // 两句话一起关掉能力表里那条空洞，而第二句与该空洞原先的假设相反。
  //
  // (1) **The quadratic's second root is not a rotor position.** For every current tried here its modulus is far from
  // one -- 24.8 at `i_d = -2 A` -- so it is a solution of the algebra and not of the machine. That means the
  // disambiguation inside one `(psi, i)` pair is nearly moot: the spurious root is rejected by its *angle*, which sits
  // about pi away, long before continuity has to be delicate. The hole was written as "continuity might pick wrong";
  // measured, there is almost nothing to pick.
  // (1) **二次方程的第二个根不是转子位置。** 这里试过的每个电流下它的模都远离 1——`i_d = −2 A` 时是 24.8——所以它是代数的
  // 解而不是机器的解。这意味着在同一个 `(psi, i)` 对内部，选根几乎是多余的：伪根靠它的**角度**（相差约 π）就被排除了，还轮
  // 不到连续性去做精细判断。空洞原写作"连续性可能选错"；实测下来，几乎没有可供选错的东西。
  //
  // (2) **A wrong seed is nevertheless permanent**, because the seed is not a tie-breaker -- it *is* the integration
  // constant, and a flux integrator has no feedback. Seeded half a turn out, the observer reports half a turn out after
  // 200 steps to three decimals. So the obligation is real and belongs where it already is: `seed_at(theta, i)`, from an
  // angle the caller knows (the align phase), not from a guess.
  // (2) **而错误的种子依然是永久的**，因为种子不是"破平局"用的——它就是积分常数，而磁链积分器没有反馈。偏半圈播种，200 步
  // 之后它仍报出偏半圈，三位小数不变。所以那项义务是真的，而且就在它现在该在的地方：`seed_at(theta, i)`，来自调用方知道的
  // 角度（对齐阶段），而不是猜的。
  const sil::MachineSpec spec = salient_spec();
  const float psi_f = psi_f_of(spec);
  const float l0 = 0.5F * (spec.ld + spec.lq);
  const float l1 = 0.5F * (spec.ld - spec.lq);
  const float theta = 0.4F;
  std::printf("\n[dual-L] the second root, and what a wrong seed costs\n");
  std::printf("     %-10s %12s %12s %10s\n", "i_d A", "second-root modulus", "angle gap", "unit?");
  for (const float id : {-0.5F, -1.0F, -2.0F, -2.5F}) {
    for (const float iq : {0.0F, 0.5F}) {
      const core::Ab<float> i_ab = core::inv_park(core::Dq<float>{id, iq}, theta);
      const core::Ab<float> flux = stator_flux(spec, theta, id, iq);
      // `L1*conj(i)*x^2 + psi_f*x + (L0*i - psi) = 0`, the same quadratic `update()` solves.
      // `L1·conj(i)·x² + psi_f·x + (L0·i − ψ) = 0`，与 `update()` 解的是同一个二次方程。
      const std::complex<float> u{i_ab.alpha, i_ab.beta};
      const std::complex<float> a{l1 * u.real(), -l1 * u.imag()};
      const std::complex<float> b{psi_f, 0.0F};
      const std::complex<float> c{(l0 * i_ab.alpha) - flux.alpha, (l0 * i_ab.beta) - flux.beta};
      const std::complex<float> disc = std::sqrt((b * b) - (4.0F * a * c));
      const std::complex<float> x1 = (-b + disc) / (2.0F * a);
      const std::complex<float> x2 = (-b - disc) / (2.0F * a);
      const std::complex<float>& spurious = (std::fabs(std::abs(x1) - 1.0F) < std::fabs(std::abs(x2) - 1.0F)) ? x2 : x1;
      const float gap = std::fabs(wrap_pi(std::atan2(spurious.imag(), spurious.real()) - theta));
      std::printf("     %-10.2f %12.4f %12.4f %10s\n", static_cast<double>(id), static_cast<double>(std::abs(spurious)),
                  static_cast<double>(gap), (std::fabs(std::abs(spurious) - 1.0F) < 0.01F) ? "yes" : "no");
      EXPECT_GT(std::fabs(std::abs(spurious) - 1.0F), 1.0F) << "i_d " << id << ": the second root is a rotor position after all";
      EXPECT_GT(gap, 1.0F) << "i_d " << id << ": the spurious root is close enough that continuity has to work";
    }
  }

  // (2) **A seed that no angle can explain is corrected at the first step**, which is the opposite of what the hole
  // feared. The flux enters the equation as a *given*, and the only root of unit modulus is the true angle, so an
  // inconsistent seed has nowhere to land. What the seed can still do is choose between two *physical* candidates -- and
  // (1) says there is no second physical candidate for this machine at any current tried. The hole is therefore
  // unreachable, and `root_gap_rad()` is what would show it if a machine ever made it reachable.
  // (2) **没有任何角度能解释的种子，会在第一步被纠正**——这与空洞原先担心的情况相反。磁链在方程里是**已知量**，而唯一模为 1 的
  // 根就是真角度，所以不一致的种子无处落脚。种子仍能做的是在两个**物理**候选之间挑——而 (1) 说在这台机器的任何电流下都没有
  // 第二个物理候选。于是那条空洞不可达，而万一某台机器让它可达，`root_gap_rad()` 就是把这件事显示出来的那个量。
  const float id = -2.0F;
  const float iq = 0.5F;
  const core::Ab<float> i_ab = core::inv_park(core::Dq<float>{id, iq}, theta);
  const core::Ab<float> v_ab{spec.rs * i_ab.alpha, spec.rs * i_ab.beta};
  core::DualInductanceObserver observer(
      core::DualInductanceObserver::Config{spec.rs, spec.ld, spec.lq, psi_f});
  // A flux built for a different current: it cannot be the stator flux of this one.
  // 用另一个电流构造的磁链：它不可能是这个电流的定子磁链。
  observer.seed_flux(stator_flux(spec, theta, -0.5F, -1.5F));
  const float first_error = std::fabs(wrap_pi(observer.update(v_ab, i_ab, kDt).theta_e - theta));
  std::printf("[dual-L] seeded with a flux built for another current: error %.4f rad after one step\n",
              static_cast<double>(first_error));
  // Measured, and it is the uncomfortable answer: 0.1128 rad after one step and the same after 200. The readout is
  // exact *given* the flux, so a wrong integration constant is not corrected by anything -- which is the obligation the
  // header states and the reason `seed_at` takes an angle rather than a guess.
  // 量出来的、也是那个不好受的答案：一步后 0.1128 rad，200 步后仍是 0.1128。给定磁链时读数是精确的，所以错误的积分常数
  // 不会被任何东西纠正——这正是头文件写明的那项义务，也是 `seed_at` 接受角度而不是猜测的原因。
  float settled = first_error;
  for (int k = 0; k < 200; ++k) {
    settled = std::fabs(wrap_pi(observer.update(v_ab, i_ab, kDt).theta_e - theta));
  }
  std::printf("[dual-L] the same seed after 200 steps: %.4f rad\n", static_cast<double>(settled));
  EXPECT_GT(first_error, 0.05F) << "an inconsistent seed was silently corrected; then this case proves nothing";
  EXPECT_NEAR(first_error, settled, 0.01F) << "it must be permanent: a flux integrator has no feedback";
}



TEST(DualInductanceObserver, AnInvalidConfigurationHoldsAndANonPositiveStepIsRejected) {
  // The contract every other observer in this directory keeps, so a caller can treat them uniformly.
  // 本目录里每个观测器都守的契约，调用方才能一视同仁。
  core::DualInductanceObserver bad(core::DualInductanceObserver::Config{0.5F, 0.0F, 3.0e-4F, 4.7e-3F});
  EXPECT_EQ(foc::ErrorCode::Param, bad.status());
  const core::ObsState held = bad.update(core::Ab<float>{1.0F, 0.0F}, core::Ab<float>{1.0F, 0.0F}, kDt);
  EXPECT_FLOAT_EQ(0.0F, held.theta_e);
  core::DualInductanceObserver good(core::DualInductanceObserver::Config{0.5F, 1.0e-4F, 3.0e-4F, 4.7e-3F});
  EXPECT_EQ(foc::ErrorCode::Ok, good.status());
  static_cast<void>(good.update(core::Ab<float>{1.0F, 0.0F}, core::Ab<float>{1.0F, 0.0F}, 0.0F));
  EXPECT_EQ(foc::ErrorCode::Param, good.status());
}

} // namespace foc
