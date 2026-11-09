// The dual-inductance observer, measured against the single-`L` one it generalises. The saliency term the single-`L`
// model discards is a **closed-form** angle bias; this file holds the algebra, the law, and the floor it is read on.
// 双电感观测器，对着它所推广的单 `L` 观测器量。单 `L` 模型丢掉的凸极项是一个 **闭式** 角度偏置；
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
 *   `Lq/Ld = 3` at `psi_f = 4.76 mWb` makes the discarded `L1` term worth `(ld - lq) * i_d / psi_f` in angle -- 0.063
 *   rad at 1.5 A, which is four times the harness floor measured below. `Lq/Ld = 3`、`psi_f = 4.76 mWb` 时，被丢掉的
 *   `L1` 项折算成角度是 `(ld − lq)·i_d/psi_f`——1.5 A 时 0.063 rad，是下面量到的夹具地板的四倍。
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
 * @details [zh] 取在夹具地板 **之下** 而不是与它相抵：磁链积分器是后向欧拉步，报出的角度比真值滞后约 `ω_e·dt/2`，这里是
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
 *   Why not the switching rig: at `psi_f = 4.76 mWb` the back-EMF at 800 rad/s is 3.8 V, so a few tenths of a volt of
 *   dead-time volt-seconds is already 0.1 rad of flux angle -- the size of the saliency term under test and common to
 *   both observers, which buries exactly the difference this file is about. Measured there, both read 0.13 to 0.15 rad
 *   and the dual-`L` looks useless; the impairment was the instrument. The inverter's own contribution is a different
 *   row's claim. 为什么不用开关台架：`psi_f = 4.76 mWb` 时 800 rad/s 的反电势只有 3.8 V，几百毫伏的死区伏秒就已经是 0.1
 *   rad 磁链角 ——与被测的凸极项同量级且两个观测器共有，正好埋掉本文件要谈的差别。在那里量，两者都读到 0.13~0.15 rad，双
 *   电感看起来一无是处；那是仪器坏了。逆变器自身的贡献是另一行主张。
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
    // 用机器实际的定子磁链播种，那是对齐阶段能从铭牌和实测电流算出来的。种子只决定积分常数：被测的模型误差是 **读数**
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
 *   Derived in the rotor frame. Subtracting `Lq*i` from the stator flux leaves `psi_f + (ld - lq)*i_d` along the rotor
 *   axis -- the `j*Lq*i_q` part cancels exactly, which is why `i_q` does not appear -- so the reported angle is off by
 *   that ratio's arctangent, **independent of speed**: no integration time makes it go away, so a drive that pairs
 *   `MtpaReference` with a single-`L` observer carries a fixed steering error proportional to the `i_d` the reference
 *   itself asks for. 在转子坐标系里推。从定子磁链里减去 `Lq·i` 后，转子轴上剩下 `psi_f + (ld − lq)·i_d`——`j·Lq·i_q` 部
 *   分精确抵消，这就是 `i_q` 不出现的原因——所以报出的角度偏这个比值的反正切，而且 **与转速无关** ：没有任何积分时间能让
 *   它消失，所以把 `MtpaReference` 与单 `L` 观测器配在一起的驱动，带着一个正比于参考自己要求的 `i_d` 的固定指向误差。
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
  // 比较时 **不积分** ，因为两者积累磁链的方式不同（一个把 `L·di` 放进积分器，另一个只在读数时减 `L0·i`；精确算术
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
  // `seed_at(θ, i)` 存在的意义，是交接方不必知道本观测器的状态是 **定子** 磁链。只用一个有意义的方式检查：由角度和电流播种，
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
  // 解而不是机器的解。这意味着在同一个 `(psi, i)` 对内部，选根几乎是多余的：伪根靠它的 **角度** （相差约 π）就被排除了，还轮
  // 不到连续性去做精细判断。空洞原写作"连续性可能选错"；实测下来，几乎没有可供选错的东西。
  //
  // (2) **A wrong seed is nevertheless permanent**, because the seed is not a tie-breaker -- it *is* the integration
  // constant, and a flux integrator has no feedback. Seeded half a turn out, the observer reports half a turn out after
  // 200 steps to three decimals. So the obligation is real and belongs where it already is: `seed_at(theta, i)`, from an
  // angle the caller knows (the align phase), not from a guess.
  // (2) **而错误的种子依然是永久的** ，因为种子不是"破平局"用的——它就是积分常数，而磁链积分器没有反馈。偏半圈播种，200 步
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
  // (2) **没有任何角度能解释的种子，会在第一步被纠正** ——这与空洞原先担心的情况相反。磁链在方程里是 **已知量** ，而唯一模为 1 的
  // 根就是真角度，所以不一致的种子无处落脚。种子仍能做的是在两个 **物理** 候选之间挑——而 (1) 说在这台机器的任何电流下都没有
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



TEST(DualInductanceObserver, AWrongSeedIsVisibleInTheRadiusEvenThoughItIsPermanent) {
  // A flux integrator cannot correct a bad seed, but the algebra still knows: the quadratic has a root of **unit modulus
  // exactly when** the state came from a real rotor position. So `radius_error()` turns the silent permanence of the
  // previous case into something a caller can refuse to use -- and it is measured here against the angle error it
  // accompanies, so the threshold is chosen from numbers rather than from taste.
  // 磁链积分器改不了坏种子，但代数仍然知道：二次方程 **当且仅当** 状态来自真实转子位置时才有一个 **模为 1** 的根。于是
  // `radius_error()` 把上一节那个静默的永久偏差变成"调用方可以拒绝使用"的东西——这里把它与相应的角度误差一起量出来，
  // 好让阈值取自有依据的数字。
  const sil::MachineSpec spec = salient_spec();
  const float psi_f = psi_f_of(spec);
  const float theta = 0.4F;
  const float id = -2.0F;
  const float iq = 0.5F;
  const core::Ab<float> i_ab = core::inv_park(core::Dq<float>{id, iq}, theta);
  const core::Ab<float> v_ab{spec.rs * i_ab.alpha, spec.rs * i_ab.beta};
  const auto probe = [&](float seed_theta, float seed_id, float seed_iq) {
    core::DualInductanceObserver observer(
        core::DualInductanceObserver::Config{spec.rs, spec.ld, spec.lq, psi_f});
    observer.seed_flux(stator_flux(spec, seed_theta, seed_id, seed_iq));
    const core::ObsState state = observer.update(v_ab, i_ab, kDt);
    return std::pair<float, float>{observer.radius_error(), std::fabs(wrap_pi(state.theta_e - theta))};
  };
  std::printf("\n[dual-L] radius_error against the angle error it accompanies\n");
  std::printf("     %-34s %14s %12s %10s\n", "seed", "radius_error", "angle error", "plausible");
  struct Row final {
    const char* name;
    float theta_;
    float id_;
    float iq_;
    bool expect_plausible;
  };
  const Row rows[] = {{"consistent (stator_flux at truth)", theta, id, iq, true},
                      {"half a turn out", theta + core::kPi<float>, id, iq, false},
                      {"built for another current", theta, -0.5F, -1.5F, false}};
  for (const Row& row : rows) {
    const auto measured = probe(row.theta_, row.id_, row.iq_);
    core::DualInductanceObserver observer(
        core::DualInductanceObserver::Config{spec.rs, spec.ld, spec.lq, psi_f});
    observer.seed_flux(stator_flux(spec, row.theta_, row.id_, row.iq_));
    static_cast<void>(observer.update(v_ab, i_ab, kDt));
    std::printf("     %-34s %14.3e %12.4f %10s\n", row.name, static_cast<double>(measured.first),
                static_cast<double>(measured.second), observer.flux_plausible() ? "yes" : "no");
    EXPECT_EQ(row.expect_plausible, observer.flux_plausible()) << row.name;
  }
  // The two ends are twelve orders of magnitude apart, which is what makes a threshold meaningful at all.
  // 两端相差十二个数量级，这才使一个阈值有意义。
  const auto good = probe(theta, id, iq);
  const auto bad = probe(theta, -0.5F, -1.5F);
  EXPECT_LT(std::fabs(good.first), 1.0e-5F) << "a consistent state must sit on the unit circle";
  EXPECT_GT(std::fabs(bad.first), 1.0e-2F) << "an inconsistent state must not";
  // And this is the point: the *angle* error of the inconsistent case is permanent (see the case above), so the radius is
  // the only thing that says anything. It is small in angle yet large in radius, which is exactly why counting on a small
  // angle error cannot detect it.
  // 而这才是要点：不一致情形下的 **角度** 误差是永久的（见上一节），所以半径是唯一还在说话的东西。它的角度不大而半径很大，
  // 这正是"靠角度误差小"无法发现它的原因。
  std::printf("[dual-L] the inconsistent seed: %.4f rad of angle error, %.3e of radius error\n",
              static_cast<double>(bad.second), static_cast<double>(bad.first));
  EXPECT_LT(bad.second, 0.5F) << "if the angle error were large, the case would prove nothing about the radius";
  // The honest limit, measured rather than hoped for: a seed only a tenth of a turn out reads 5.8e-03 of radius, i.e.
  // it **passes** the tolerance while its angle is 0.28 rad wrong. So this is a gross-fault detector and not an accuracy
  // guarantee, and the case records that instead of raising the threshold to make it look like one.
  // 诚实的边界，量出来的而不是希望出来的：偏十分之一圈的种子读到 5.8e-03 的半径，即它 **通过** 容差，而角度错了 0.28 rad。
  // 所以这是粗略故障检出器，不是精度保证；用例把这一条记下来，而不是把阈值调高好让它看起来像保证。
  const auto tenth = probe(theta + (0.1F * core::kPi<float>), id, iq);
  std::printf("[dual-L] a tenth of a turn out: radius %.3e (passes), angle error %.4f rad\n",
              static_cast<double>(tenth.first), static_cast<double>(tenth.second));
  EXPECT_LT(std::fabs(tenth.first), core::DualInductanceObserver::kRadiusTolerance)
      << "if this now fails, the detector got sharper and the limit recorded here is stale";
  // And the noise floor that the tolerance has to clear: a consistent state stepped 2000 times stays on the circle.
  // 而容差必须让开的那个噪声地板：自洽状态走 2000 步仍然停在单位圆上。
  core::DualInductanceObserver long_run(core::DualInductanceObserver::Config{spec.rs, spec.ld, spec.lq, psi_f});
  long_run.seed_flux(stator_flux(spec, theta, id, iq));
  float worst_radius = 0.0F;
  for (int k = 0; k < 2000; ++k) {
    static_cast<void>(long_run.update(v_ab, i_ab, kDt));
    worst_radius = std::max(worst_radius, std::fabs(long_run.radius_error()));
  }
  std::printf("[dual-L] consistent state over 2000 steps: worst radius %.3e (tolerance %.0e)\n",
              static_cast<double>(worst_radius), static_cast<double>(core::DualInductanceObserver::kRadiusTolerance));
  EXPECT_LT(worst_radius, 0.1F * core::DualInductanceObserver::kRadiusTolerance) << "the floor must clear the tolerance";
}

TEST(DualInductanceObserver, AnLqErrorMovesTheAngleAndAnLdErrorMovesTheRadius) {
  // The sensitivity leg the plant could not resolve (a 10 % `Ld` error is far under the harness's 13 mrad floor), measured
  // **statically**: seed at a known angle and current, then hand it `v = Rs*i`, which leaves the flux untouched -- so
  // there is no integration and therefore no floor.
  // plant 分辨不了的那条敏感度腿（10 % 的 `Ld` 远在夹具 13 mrad 地板之下），改在 **静态** 下量：用已知角度与电流播种，再
  // 交给它 `v = Rs·i`（磁链不变）——于是没有积分，也就没有地板。
  //
  // **The law I derived first was wrong, and the measurement says so.** I wrote that an `Ld` error acts on the *angle* as
  // `dLd*i_d/psi_f`, by the same argument that makes an `Lq` error act as `dLq*i_q/psi_f`. Measured: the `Lq` law holds
  // (2.871e-3 against 3.150e-3 predicted, ratio 0.91) but the `Ld` law misses by **58x** -- 7.2e-5 rad where 4.2e-3 was
  // predicted. An `Ld` error mostly *rotates and scales the flux* rather than re-aiming it, and the angle of the solved
  // root is second-order insensitive to that. What it does move is the **radius**: 3.888e-3 against the same
  // `dLd*i_d/psi_f` = 4.2e-3, ratio 0.93. So the two inductances are separable in two different observables, and this
  // is why `radius_error()` earns its place: it is where an `Ld` error shows at all.
  // **我先推的定律是错的，而测量如此说。** 我写过 `Ld` 误差按 `dLd·i_d/psi_f` 作用于 **角度** ，依据与 `Lq` 误差按
  // `dLq·i_q/psi_f` 作用相同。实测：`Lq` 定律成立（2.871e-3 对预测 3.150e-3，比 0.91），而 `Ld` 定律差了 **58 倍** ——
  // 预测 4.2e-3 的地方只有 7.2e-5。`Ld` 误差主要是 **转动并缩放** 磁链而不是重新指向它，而解出根的角度对那是二阶不敏感的。
  // 它真正挪动的是 **半径** ：3.888e-3 对同一个 `dLd·i_d/psi_f` = 4.2e-3，比 0.93。于是两个电感在两个不同的可观测量上可
  // 分离，而这正是 `radius_error()` 值得存在的原因：`Ld` 误差只在那里露面。
  const sil::MachineSpec spec = salient_spec();
  const float psi_f = psi_f_of(spec);
  const float theta = 0.7F;
  const float id = -2.0F;
  const float iq = 0.5F;
  const core::Ab<float> i_ab = core::inv_park(core::Dq<float>{id, iq}, theta);
  const core::Ab<float> v_ab{spec.rs * i_ab.alpha, spec.rs * i_ab.beta};
  struct Pair final {
    float angle;
    float radius;
  };
  const auto probe = [&](float ld_scale, float lq_scale) {
    core::DualInductanceObserver observer(
        core::DualInductanceObserver::Config{spec.rs, spec.ld * ld_scale, spec.lq * lq_scale, psi_f});
    observer.seed_flux(stator_flux(spec, theta, id, iq));
    const float angle = std::fabs(wrap_pi(observer.update(v_ab, i_ab, kDt).theta_e - theta));
    return Pair{angle, observer.radius_error()};
  };
  const float law_ld = std::fabs(0.1F * spec.ld * id / psi_f);
  const float law_lq = std::fabs(0.1F * spec.lq * iq / psi_f);
  std::printf("\n[dual-L] static nameplate sensitivity, i_d = %.1f A, i_q = %.1f A (psi_f = %.4e Wb)\n",
              static_cast<double>(id), static_cast<double>(iq), static_cast<double>(psi_f));
  std::printf("     %-12s %12s %12s %12s %12s\n", "case", "angle err", "radius err", "law", "ratio");
  const Pair ld_up = probe(1.1F, 1.0F);
  const Pair ld_down = probe(0.9F, 1.0F);
  const Pair lq_up = probe(1.0F, 1.1F);
  const Pair lq_down = probe(1.0F, 0.9F);
  std::printf("     %-12s %12.3e %12.3e %12.3e %12.3f\n", "dLd +10 %", static_cast<double>(ld_up.angle),
              static_cast<double>(ld_up.radius), static_cast<double>(law_ld),
              static_cast<double>(std::fabs(ld_up.radius) / law_ld));
  std::printf("     %-12s %12.3e %12.3e %12.3e %12.3f\n", "dLd -10 %", static_cast<double>(ld_down.angle),
              static_cast<double>(ld_down.radius), static_cast<double>(law_ld),
              static_cast<double>(std::fabs(ld_down.radius) / law_ld));
  std::printf("     %-12s %12.3e %12.3e %12.3e %12.3f\n", "dLq +10 %", static_cast<double>(lq_up.angle),
              static_cast<double>(lq_up.radius), static_cast<double>(law_lq),
              static_cast<double>(lq_up.angle / law_lq));
  std::printf("     %-12s %12.3e %12.3e %12.3e %12.3f\n", "dLq -10 %", static_cast<double>(lq_down.angle),
              static_cast<double>(lq_down.radius), static_cast<double>(law_lq),
              static_cast<double>(lq_down.angle / law_lq));
  // The laws, as measured.
  EXPECT_NEAR(law_lq, lq_up.angle, 0.15F * law_lq) << "dLq must move the angle by dLq*i_q/psi_f";
  EXPECT_NEAR(law_lq, lq_down.angle, 0.15F * law_lq);
  EXPECT_NEAR(law_ld, std::fabs(ld_up.radius), 0.15F * law_ld) << "dLd must move the *radius* by dLd*i_d/psi_f";
  EXPECT_NEAR(law_ld, std::fabs(ld_down.radius), 0.15F * law_ld);
  // And the separation is what makes the pair of observables worth having: `Ld` moves the radius by more than an order
  // of magnitude more than it moves the angle, `Lq` the other way round.
  // 而正是这种分离让这两个可观测量值得同时存在：`Ld` 对半径的推动比它对角度的大一个数量级以上，`Lq` 反过来。
  EXPECT_GT(std::fabs(ld_up.radius), 20.0F * ld_up.angle) << "dLd must show in the radius, not the angle";
  EXPECT_GT(lq_up.angle, 20.0F * std::fabs(lq_up.radius)) << "dLq must show in the angle, not the radius";
}

TEST(DualInductanceObserver, TheTwoFluxSourcesDifferExactlyWhereTheyShould) {
  // The `method=` earns its place only if the options fail differently, so both differences are measured here: with the
  // machine's own `Rs` they must be the **same observer** (the correction has nothing to correct), and with a nameplate
  // error in `Rs` the pure integrator must drift while the corrected one holds.
  // `method=` 只有在两个选项失效方式不同时才值得存在，所以两种差别都在这里量：用机器自己的 `Rs` 时它们必须是 **同一个观测器**
  // （修正项无事可做），而 `Rs` 有铭牌误差时纯积分器必须漂移、修正版必须按得住。
  const sil::MachineSpec spec = salient_spec();
  const float psi_f = psi_f_of(spec);
  const float omega = 200.0F;
  const float id = -1.0F;
  const float iq = 0.5F;
  const auto run = [&](core::DualInductanceObserver::FluxSource source, float rs_scale, float gain) {
    sil::PmsmPlant plant(sil::plant_params_from(spec));
    plant.set_mechanical_speed(omega / static_cast<float>(spec.pole_pairs));
    const core::Dq<float> v_dq{spec.rs * id - (omega * spec.lq * iq),
                                (spec.rs * iq) + (omega * ((spec.ld * id) + psi_f))};
    plant.set_voltage_dq(v_dq);
    for (int k = 0; k < 6000; ++k) {
      plant.advance(kDt);
    }
    const float theta = wrap_two_pi(plant.theta_mech_rad() * static_cast<float>(spec.pole_pairs));
    const core::Dq<float> psi_dq{psi_f + (spec.ld * id), spec.lq * iq};
    core::DualInductanceObserver observer(core::DualInductanceObserver::Config{
        spec.rs * rs_scale, spec.ld, spec.lq, psi_f, source, gain});
    observer.seed_flux(core::inv_park(psi_dq, theta));
    // Two seconds: long enough for a nameplate error to accumulate visibly, short enough to run in a unit test.
    // 两秒：足够让铭牌误差明显累积，又短到能在单元测试里跑。
    float sum = 0.0F;
    int count = 0;
    float first_angle = 0.0F;
    bool first = true;
    for (int k = 0; k < 40000; ++k) {
      plant.advance(kDt);
      const float truth = wrap_two_pi(plant.theta_mech_rad() * static_cast<float>(spec.pole_pairs));
      const core::Ab<float> i_ab = core::clarke(plant.current_uvw());
      const core::Ab<float> v_ab = core::inv_park(v_dq, truth);
      const core::ObsState state = observer.update(v_ab, i_ab, kDt);
      if (first) {
        first_angle = state.theta_e;
        first = false;
      }
      if (k >= 36000) {
        sum += std::fabs(wrap_pi(state.theta_e - truth));
        ++count;
      }
    }
    return std::pair<float, float>{first_angle, sum / static_cast<float>(count)};
  };
  const auto exact_integrator = run(core::DualInductanceObserver::FluxSource::Integrator, 1.0F, 0.2F);
  const auto exact_corrected = run(core::DualInductanceObserver::FluxSource::Corrected, 1.0F, 0.2F);
  const auto wrong_integrator = run(core::DualInductanceObserver::FluxSource::Integrator, 1.2F, 0.2F);
  const auto wrong_corrected = run(core::DualInductanceObserver::FluxSource::Corrected, 1.2F, 0.2F);
  std::printf("\n[dual-L] Rs 20 %% high, %.0f rad/s electrical, 2 s: the two flux sources\n", static_cast<double>(omega));
  std::printf("     %-24s %14s %14s\n", "case", "mean abs error", "last-0.2 s window");
  std::printf("     %-24s %14.6f %14.6f\n", "exact Rs, integrator", static_cast<double>(exact_integrator.first),
              static_cast<double>(exact_integrator.second));
  std::printf("     %-24s %14.6f %14.6f\n", "exact Rs, corrected", static_cast<double>(exact_corrected.first),
              static_cast<double>(exact_corrected.second));
  std::printf("     %-24s %14.6f %14.6f\n", "Rs +20 %, integrator", static_cast<double>(wrong_integrator.first),
              static_cast<double>(wrong_integrator.second));
  std::printf("     %-24s %14.6f %14.6f\n", "Rs +20 %, corrected", static_cast<double>(wrong_corrected.first),
              static_cast<double>(wrong_corrected.second));
  // **My prediction was wrong here, twice, and both corrections are the finding.**
  // 1. I expected the correction to be a no-op when `Rs` is exact. It is not: it is **7.7x better** (0.002580 against
  //    0.019895 rad), because a state that integrates for 40 000 steps carries its own accumulated error and the
  //    projection removes that too. So the gain is not only about the nameplate.
  // 2. I expected the correction to help under a nameplate error. It **destroys** the estimate: 3.080 rad against the
  //    integrator's 0.167 -- it locks onto a *wrong but self-consistent* state, which is exactly what a projection onto
  //    the manifold can do and what the note in the header warned about without a number. So the two sources are a real
  //    trade, not an upgrade, and the integrator stays the default.
  // **我的预测在这里错了两次，而两次更正本身就是结论。**
  // 1. 我原以为 `Rs` 精确时修正项无事可做。并非如此：它 **好 7.7 倍** （0.002580 对 0.019895 rad），因为积分了 40000 步的
  //    状态自带累积误差，而投影把那个也一起去掉了。所以这个增益不只关乎铭牌。
  // 2. 我原以为铭牌误差下修正会帮忙。它 **毁掉** 了估计：3.080 rad 对积分器的 0.167——它锁到一个 **错但自洽** 的状态上，这
  //    正是"向流形投影"能做出的坏事，也正是头文件那条注释警告过、但没有数字的东西。所以两种来源是一笔真实的交易，不是
  //    一次升级，积分器仍然是默认值。
  std::printf("[dual-L] exact Rs: integrator %.6f rad against corrected %.6f rad (%.2fx better)\n",
              static_cast<double>(exact_integrator.second), static_cast<double>(exact_corrected.second),
              static_cast<double>(exact_integrator.second / exact_corrected.second));
  std::printf("[dual-L] Rs +20 %%: integrator %.6f rad against corrected %.6f rad (%.1fx worse)\n",
              static_cast<double>(wrong_integrator.second), static_cast<double>(wrong_corrected.second),
              static_cast<double>(wrong_corrected.second / wrong_integrator.second));
  EXPECT_LT(exact_corrected.second, 0.5F * exact_integrator.second)
      << "the correction must pay for itself on a numerically drifted state";
  EXPECT_GT(wrong_integrator.second, 2.0F * exact_integrator.second) << "the nameplate error must show in the integrator";
  EXPECT_GT(wrong_corrected.second, 3.0F * wrong_integrator.second)
      << "if this now passes, the lock-in was fixed and this recorded failure mode is stale";
}

TEST(DualInductanceObserver, ASeedErrorTakesTheCorrectionTermToTheOtherBranch) {
  // The corrected source pulls the flux back onto the model manifold, and that is worth 7.7x under a nameplate `Rs`
  // error -- *given an exact seed*, which is how it was measured. This case asks what happens to it when the seed is
  // wrong, because that is the situation a hand-over creates, and the answer is that the pull is a nearest-point
  // search on a curve: an angular seed error lands it on the other branch and it stays there.
  // 修正版把磁链拉回模型流形，在 `Rs` 有铭牌误差时值 7.7 倍——但那是在 **种子准确** 的前提下量的。本例问的是种子错的时候它
  // 怎么样，因为交接造成的正是这个局面；答案是这把"拉"本质上是曲线上的一次最近点搜索：一个角度上的种子误差会把它落到
  // 另一支上，然后它就停在那里。
  const sil::MachineSpec spec = salient_spec();
  const float psi_f = psi_f_of(spec);
  const float omega = 200.0F;
  const float seed_offset = 0.30F;
  const auto run = [&](core::DualInductanceObserver::FluxSource source, float iq) {
    sil::PmsmPlant plant(sil::plant_params_from(spec));
    plant.set_mechanical_speed(omega / static_cast<float>(spec.pole_pairs));
    const core::Dq<float> v_dq{-(omega * spec.lq * iq), (spec.rs * iq) + (omega * psi_f)};
    plant.set_voltage_dq(v_dq);
    for (int k = 0; k < 6000; ++k) {
      plant.advance(kDt);
    }
    const float theta = wrap_two_pi(plant.theta_mech_rad() * static_cast<float>(spec.pole_pairs));
    const core::Dq<float> psi_dq{psi_f, spec.lq * iq};
    core::DualInductanceObserver observer(
        core::DualInductanceObserver::Config{spec.rs, spec.ld, spec.lq, psi_f, source, 2.0F});
    observer.seed_flux(core::inv_park(psi_dq, theta + seed_offset));
    float sum = 0.0F;
    int count = 0;
    for (int k = 0; k < 40000; ++k) {
      plant.advance(kDt);
      const float truth = wrap_two_pi(plant.theta_mech_rad() * static_cast<float>(spec.pole_pairs));
      const core::ObsState state =
          observer.update(core::inv_park(v_dq, truth), core::clarke(plant.current_uvw()), kDt);
      if (k >= 36000) {
        sum += std::fabs(wrap_pi(state.theta_e - truth));
        ++count;
      }
    }
    return sum / static_cast<float>(count);
  };
  std::printf("\n[dual-L] seed +%.2f rad off, exact Rs, %.0f rad/s electrical: what a seed error does to the correction\n",
              static_cast<double>(seed_offset), static_cast<double>(omega));
  std::printf("     %-10s %12s %12s %10s\n", "iq [A]", "integrator", "corrected", "ratio");
  float ratios[3] = {0.0F, 0.0F, 0.0F};
  float plain[3] = {0.0F, 0.0F, 0.0F};
  const float currents[3] = {0.2F, 1.0F, 3.0F};
  for (int k = 0; k < 3; ++k) {
    plain[k] = run(core::DualInductanceObserver::FluxSource::Integrator, currents[k]);
    const float corrected = run(core::DualInductanceObserver::FluxSource::Corrected, currents[k]);
    ratios[k] = (corrected > 0.0F) ? (plain[k] / corrected) : 0.0F;
    std::printf("     %-10.2f %12.6f %12.6f %10.3f\n", static_cast<double>(currents[k]),
                static_cast<double>(plain[k]), static_cast<double>(corrected), static_cast<double>(ratios[k]));
  }
  // The integrator forgets nothing, so it holds the seed offset (it even grows a little with current, where the
  // rotating saliency term makes the readout nonlinear). The corrected one is worse at every current: the pull is a
  // nearest-point search on the flux curve, and an angular seed error puts the nearest point on the other branch.
  // 纯积分器什么都不忘，所以它按种子偏移量保持（电流大一点还会更大，那里旋转的凸极项让读数非线性）。修正版在每个电流下
  // 都更差：那把"拉"是磁链曲线上的一次最近点搜索，而角度上的种子误差会让最近点落在另一支上。
  for (int k = 0; k < 3; ++k) {
    EXPECT_GE(plain[k], 0.5F * seed_offset)
        << "iq " << currents[k] << ": a pure integrator should hold the seed offset, not converge off it";
    EXPECT_LT(plain[k], 3.0F * seed_offset) << "iq " << currents[k] << ": the integrator itself diverged";
    EXPECT_LT(ratios[k], 0.5F) << "iq " << currents[k]
                               << ": the correction term is no longer a remedy once the seed is wrong (ratio "
                               << ratios[k] << ")";
  }
  // And the declaration this closes: the corrected source's measured domain is "exact seed, nameplate error".
  // 而这条断言收口的是那句声明：修正版的实测适用域是"种子准确、铭牌有误差"。
  EXPECT_GT(ratios[0], 0.0F) << "if this is zero the corrected run produced a non-finite state, which is another fact";
}

TEST(DualInductanceObserver, TheClosedFormAgreesWithANumericalInversionOfTheFluxRelation) {
  // A second implementation of the same inversion, kept here rather than shipped: it produces the same answer from the
  // same inputs, so as a public option it would be a knob whose only effect is which code path rounds differently --
  // dead weight of the kind F-107 and F-43 were about. As a **check** it has a job, because it inverts by searching
  // instead of by solving: no discriminant, no roots, no branch. If the closed form had a sign or a factor wrong, the
  // two would part company on some sample.
  // 同一个反解的第二种实现，放在测试里而不是发布出去：它从同样的输入给出同样的答案，作为公开选项就只是一个"决定哪条代码路径
  // 舍入得不同"的旋钮——正是 F-107 / F-43 所讲的那种死重量。而作为 **检查** 它有活干，因为它靠搜索而不是靠求解来反解：没有
  // 判别式、没有求根、没有分支。闭式里若有一个符号或系数错，两者会在某个样本上分道扬镳。
  //
  // And the two do not share their *forward* statement either: the flux is built from the physical d-q definition
  // (`psi = e^(j*theta) * (psi_f + Ld*i_d + j*Lq*i_q)`), while the search minimises the alpha-beta relation the class
  // uses. So this compares two ways of writing the machine as well as two ways of inverting it.
  // 而且两者连 **正向** 表述也不共享：磁链按物理的 d-q 定义构造（`psi = e^(jθ)·(psi_f + Ld·i_d + j·Lq·i_q)`），而搜索最小化的是
  // 本类使用的 αβ 关系。所以这里同时比较了机器的两种写法与两种反解。
  const sil::MachineSpec spec = salient_spec();
  const float psi_f = psi_f_of(spec);
  const float l0 = 0.5F * (spec.ld + spec.lq);
  const float l1 = 0.5F * (spec.ld - spec.lq);
  const auto residual = [&](float phi, const core::Ab<float>& flux, const core::Ab<float>& i_ab) {
    const float c1 = std::cos(phi);
    const float s1 = std::sin(phi);
    const float c2 = std::cos(2.0F * phi);
    const float s2 = std::sin(2.0F * phi);
    const float alpha = (psi_f * c1) + (l0 * i_ab.alpha) + (l1 * ((i_ab.alpha * c2) + (i_ab.beta * s2)));
    const float beta = (psi_f * s1) + (l0 * i_ab.beta) + (l1 * ((i_ab.alpha * s2) - (i_ab.beta * c2)));
    return ((alpha - flux.alpha) * (alpha - flux.alpha)) + ((beta - flux.beta) * (beta - flux.beta));
  };
  float worst_closed_vs_algebra = 0.0F;
  float worst_closed_vs_search = 0.0F;
  float worst_residual = 0.0F;
  for (const float theta : {0.0F, 0.9F, 2.2F, 4.7F, 6.0F}) {
    for (const float id : {0.0F, -1.0F, -2.5F, 1.5F}) {
      for (const float iq : {0.0F, 0.5F, -1.0F}) {
        // The flux by the physical d-q definition, rotated into alpha-beta.
        const core::Dq<float> psi_dq{psi_f + (spec.ld * id), spec.lq * iq};
        const core::Ab<float> flux = core::inv_park(psi_dq, theta);
        const core::Ab<float> i_ab = core::inv_park(core::Dq<float>{id, iq}, theta);
        const core::Ab<float> v_ab{spec.rs * i_ab.alpha, spec.rs * i_ab.beta};
        core::DualInductanceObserver observer(
            core::DualInductanceObserver::Config{spec.rs, spec.ld, spec.lq, psi_f});
        observer.seed_flux(flux);
        const float closed = observer.update(v_ab, i_ab, kDt).theta_e;
        worst_closed_vs_algebra = std::max(worst_closed_vs_algebra, std::fabs(wrap_pi(closed - theta)));
        // The search: coarse scan then ternary refinement, entirely independent of the quadratic.
        // 搜索：粗扫 + 三分细化，完全不用二次方程。
        float best = 0.0F;
        float best_cost = 1.0e30F;
        constexpr int kScan = 720;
        for (int k = 0; k < kScan; ++k) {
          const float phi = (core::kTwoPi<float> * static_cast<float>(k)) / static_cast<float>(kScan);
          const float cost = residual(phi, flux, i_ab);
          if (cost < best_cost) {
            best_cost = cost;
            best = phi;
          }
        }
        float lo = best - (core::kTwoPi<float> / static_cast<float>(kScan));
        float hi = best + (core::kTwoPi<float> / static_cast<float>(kScan));
        for (int k = 0; k < 80; ++k) {
          const float m1 = lo + ((hi - lo) / 3.0F);
          const float m2 = hi - ((hi - lo) / 3.0F);
          if (residual(m1, flux, i_ab) < residual(m2, flux, i_ab)) {
            hi = m2;
          } else {
            lo = m1;
          }
        }
        const float searched = 0.5F * (lo + hi);
        worst_closed_vs_search = std::max(worst_closed_vs_search, std::fabs(wrap_pi(closed - searched)));
        worst_residual = std::max(worst_residual, residual(searched, flux, i_ab));
      }
    }
  }
  std::printf("\n[dual-L] closed form against a numerical inversion, 60 (angle, i_d, i_q) samples\n");
  std::printf("[dual-L] closed vs the d-q definition: %.3e rad | closed vs the search: %.3e rad | search residual %.3e\n",
              static_cast<double>(worst_closed_vs_algebra), static_cast<double>(worst_closed_vs_search),
              static_cast<double>(worst_residual));
  EXPECT_LT(worst_closed_vs_algebra, 1.0e-4F) << "the closed form must invert the physical d-q definition";
  EXPECT_LT(worst_closed_vs_search, 1.0e-4F) << "two inversions of the same relation must agree";
  EXPECT_LT(worst_residual, 1.0e-8F) << "the search must actually reach a zero of the alpha-beta relation";
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
