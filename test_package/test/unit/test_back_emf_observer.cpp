#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>



static_assert(foc::core::concepts::is_observer_v<foc::core::BackEmfObserver>,
              "BackEmfObserver must satisfy the observer policy contract");



namespace {

// Synthetic surface-mounted PMSM drive, exact signals so the observer can be judged on its own.
constexpr float kPsiM = 0.01F;    // Wb, rotor flux linkage
constexpr float kOmegaE = 100.0F; // rad/s electrical
constexpr float kRs = 0.5F;       // ohm
constexpr float kL = 4e-4F;       // H
constexpr float kI = 1.0F;        // A, current amplitude
constexpr float kPhi = 0.3F;      // rad, current phase offset
constexpr float kDt = 1e-5F;      // s, 100 kHz for a tight integration

auto wrap_pi(float angle) -> float {
    while (angle <= -foc::core::kPi<float>) {
        angle += foc::core::kTwoPi<float>;
    }
    while (angle > foc::core::kPi<float>) {
        angle -= foc::core::kTwoPi<float>;
    }
    return angle;
}

/**
 * @brief Advances the synthetic drive one step and returns the true electrical angle.
 */
auto drive_step(float& t, foc::core::Ab<float>& v_ab, foc::core::Ab<float>& i_ab) -> float {
    const float theta = kOmegaE * t;
    const float cos_t = std::cos(theta + kPhi);
    const float sin_t = std::sin(theta + kPhi);

    i_ab = foc::core::Ab<float>{kI * cos_t, kI * sin_t};

    // di/dt = I * omega * [-sin(theta+phi), cos(theta+phi)]
    const float di_alpha = -kI * kOmegaE * sin_t;
    const float di_beta = kI * kOmegaE * cos_t;

    // back-EMF of the rotor flux: e = psi_m * omega * [-sin(theta), cos(theta)]
    const float e_alpha = -kPsiM * kOmegaE * std::sin(theta);
    const float e_beta = kPsiM * kOmegaE * std::cos(theta);

    v_ab = foc::core::Ab<float>{kRs * i_ab.alpha + kL * di_alpha + e_alpha, kRs * i_ab.beta + kL * di_beta + e_beta};
    t += kDt;
    return theta;
}

} // namespace



TEST(BackEmfObserver, TracksAngleWhenSeededWithInitialFlux) {
    foc::core::BackEmfObserver observer({kRs, kL});
    ASSERT_EQ(foc::ErrorCode::Ok, observer.status());

    // At t = 0 the rotor flux points along alpha: psi(0) = psi_m * [1, 0].
    observer.seed_flux(foc::core::Ab<float>{kPsiM, 0.0F});

    float t = 0.0F;
    float theta_true = 0.0F;
    foc::core::ObsState state{};
    const int steps = static_cast<int>(foc::core::kTwoPi<float> / (kOmegaE * kDt));

    for (int i = 0; i < 2 * steps; ++i) {
        foc::core::Ab<float> v_ab{};
        foc::core::Ab<float> i_ab{};
        theta_true = drive_step(t, v_ab, i_ab);
        state = observer.update(v_ab, i_ab, kDt);

        if (i > steps) { // one electrical cycle of settling
            EXPECT_LT(std::fabs(wrap_pi(state.theta_e - theta_true)), 1e-2F) << "step " << i;
        }
    }

    EXPECT_NEAR(kOmegaE, state.omega_e, 0.05F * kOmegaE);
}



TEST(BackEmfObserver, NonPositiveDtIsRejectedAndHolds) {
    foc::core::BackEmfObserver observer({kRs, kL});
    ASSERT_EQ(foc::ErrorCode::Ok, observer.status());

    const foc::core::ObsState state = observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, 0.0F);

    EXPECT_EQ(foc::ErrorCode::Param, observer.status());
    EXPECT_FLOAT_EQ(0.0F, state.theta_e);

    // the latched error keeps holding even when a valid dt arrives...
    const foc::core::ObsState after = observer.update(foc::core::Ab<float>{1.0F, 1.0F}, foc::core::Ab<float>{}, kDt);
    EXPECT_FLOAT_EQ(state.theta_e, after.theta_e);
    EXPECT_EQ(foc::ErrorCode::Param, observer.status());

    // ...until reset() clears it.
    observer.reset();
    EXPECT_EQ(foc::ErrorCode::Ok, observer.status());
}



TEST(BackEmfObserver, InvalidParametersAreRejected) {
    const foc::core::BackEmfObserver zero_inductance({kRs, 0.0F});
    EXPECT_EQ(foc::ErrorCode::Param, zero_inductance.status());

    const foc::core::BackEmfObserver negative_resistance({-1.0F, kL});
    EXPECT_EQ(foc::ErrorCode::Param, negative_resistance.status());
}



TEST(BackEmfObserver, SeedAndResetControlTheFluxState) {
    foc::core::BackEmfObserver observer({kRs, kL});

    observer.seed_flux(foc::core::Ab<float>{kPsiM, 0.0F});
    EXPECT_FLOAT_EQ(kPsiM, observer.flux().alpha);

    observer.reset();
    EXPECT_FLOAT_EQ(0.0F, observer.flux().alpha);
    EXPECT_FLOAT_EQ(0.0F, observer.flux().beta);
}



TEST(BackEmfObserver, SpeedEstimateWrapsAcrossTheAngleBoundary) {
    foc::core::BackEmfObserver observer({0.0F, kL}); // Rs = 0 keeps the arithmetic readable
    observer.seed_flux(foc::core::Ab<float>{1.0F, 0.0F});

    // First step only primes the angle reference.
    static_cast<void>(observer.update(foc::core::Ab<float>{}, foc::core::Ab<float>{}, kDt));

    // Drive the flux from the +alpha axis into the third quadrant within a single step, so the
    // angle difference between samples exceeds pi and the wrap has to kick in.
    const float dt = 1e-3F;
    const foc::core::ObsState state =
        observer.update(foc::core::Ab<float>{-2000.0F, -1000.0F}, foc::core::Ab<float>{}, dt);

    // atan2(-1, -1) = -3*pi/4, wrapped to +3*pi/4 by the angle normaliser, so the delta is -3*pi/4.
    EXPECT_NEAR(-2.35619F, state.omega_e * dt, 1e-3F);
}


namespace {

constexpr float kProbeRs = 0.5F;
constexpr float kProbeL = 8.0e-4F;
constexpr float kProbePsiF = 0.0047619F;
constexpr float kProbeDt = 5e-5F;

auto wrap_pi_probe(float angle_rad) -> float {
    while (angle_rad > 3.14159265358979F) {
        angle_rad -= 6.28318530717959F;
    }
    while (angle_rad < -3.14159265358979F) {
        angle_rad += 6.28318530717959F;
    }
    return angle_rad;
}

/**
 * @brief [en] Drive the observer from the exact steady state of a PMSM, with optional parameter and sensor errors.
 *
 * @brief [zh] 用 PMSM 的精确稳态驱动观测器，可选的参数误差与传感器误差。
 *
 *   The terminal voltage is the **full** one, `Rs*i + dpsi/dt + L*di/dt`. Leaving the inductive drop out was the first
 *   version of this harness and it made the observer look broken -- |psi| swung between 0.60 and 1.63 of the true flux
 *   and the angle error reached 0.94 rad. That is not the observer double-counting `L*di`: it is the harness feeding a
 *   machine whose voltage never had the term the observer subtracts. The distinguishing test was that the damage did
 *   not move when `dt` changed by 10x, while a numerical instability would have.
 *
 *   端电压取 **完整** 的那个：`Rs·i + dψ/dt + L·di/dt`。第一版本的工具漏了电感压降，于是观测器看起来是坏的—— |ψ| 在真值
 *   的 0.60 与 1.63 之间摆动、角度误差到 0.94 rad。那不是观测器把 `L·di` 算了两遍，而是工具喂给了一台电压里从来没有那一
 *   项的机器，而观测器又确实要减它。区分方法很干脆：把 `dt` 改 10 倍，损害一动不动；数值不稳定则会跟着变。
 *
 *   `i_offset_a` is a DC offset on the *measured* current, so the observer's `Rs*i` term carries a constant voltage
 *   error in every step. That is the one input error that does not average out over a revolution.
 *
 *   `i_offset_a` 是 **实测** 电流上的直流偏置，于是观测器的 `Rs·i` 项每一步都带着一个恒定的电压误差。那是唯一一个不会在
 *   一圈里自己平均掉的输入误差。
 */
struct ProbeRun final {
    float err_max_rad{0.0F};
    float err_end_rad{0.0F};
};

auto probe_run(float omega, float iq, float rs_hat, float i_offset_a, int steps) -> ProbeRun {
    foc::core::BackEmfObserver observer({rs_hat, kProbeL});
    observer.seed_flux(foc::core::Ab<float>{kProbePsiF, 0.0F});
    float theta = 0.0F;
    ProbeRun run{};
    for (int k = 0; k < steps; ++k) {
        const float s = std::sin(theta);
        const float c = std::cos(theta);
        const float i_a = -iq * s;
        const float i_b = iq * c;
        const float v_a = (kProbeRs * i_a) - (omega * kProbePsiF * s) - (kProbeL * omega * iq * c);
        const float v_b = (kProbeRs * i_b) + (omega * kProbePsiF * c) - (kProbeL * omega * iq * s);
        const auto state = observer.update(foc::core::Ab<float>{v_a, v_b},
                                           foc::core::Ab<float>{i_a + i_offset_a, i_b + i_offset_a}, kProbeDt);
        theta += omega * kProbeDt;
        const float err = wrap_pi_probe(state.theta_e - std::fmod(theta, 6.28318530717959F));
        run.err_max_rad = std::fmax(run.err_max_rad, std::fabs(err));
        run.err_end_rad = err;
    }
    return run;
}

} // namespace

/**
 * @brief [en] On an exact model the residue is one step of rotation, a resistance error stays **bounded** at about
 *             `eps*Rs*iq/(omega*psi_f)`, and a DC offset is the one error that runs away.
 *
 * @brief [zh] 精确模型下残差是 **一步的旋转** ；电阻误差被 **界住** 在约 `ε·Rs·iq/(ω·ψf)`；而直流偏置是唯一会跑掉的那一
 *             个。
 *
 *   This observer is pure integration -- `psi += (v - Rs*i)*dt - L*di` with **no correction term** -- so the only
 *   question that matters is which input errors accumulate and which do not. Measured here, at `Rs = 0.5`, `L = 8e-4`,
 *   `psi_f = 4.7619e-3`, `dt = 50 us`:
 *
 *   | what is wrong | what the angle error does | |---|---| | nothing (exact model) | **one step of rotation**: 5.2e-4
 *   rad at 20 rad/s, 5.3e-3 at 100, 1.06e-2 at 200 -- proportional to `omega`, i.e. `omega*dt` | | `Rs` +10 % at 200
 *   rad/s | bounded, 0.191 rad; the small-error law `eps*Rs*iq/(omega*psi_f)` says 0.1575 (ratio **1.21**) | | `Rs`
 *   +10 % at 400 rad/s | 0.0912 against 0.0788 (ratio **1.16**) -- the law tightens as the error shrinks | | `Rs` +10 %
 *   at 50 rad/s | **3.14 rad**: the law says 0.63, so it has failed -- the estimate is a full `pi` out and the
 *   small-perturbation form no longer applies | | `L` +-20 % | bounded: 0.191 / 0.212 rad of excursion, no accumulation
 *   | | **DC offset 0.05 A** on the measured current | **0.711 rad in 100 ms, still growing** -- `Rs*off/psi_f = 5.25
 *   rad/s` is the right order |
 *
 *   Two conclusions a user can act on. **A parameter error is a bias, not a runaway**: a 10 % wrong `Rs` costs 0.19 rad
 *   at 200 rad/s and *less* at higher speed, so the honest spec is "bounded, `eps*Rs*iq/(omega*psi_f)` while that is
 *   small", and the case records where that stops being true (the 50 rad/s row, where the error saturates at `pi`).
 *   **A sensor offset is the runaway**: it enters as a constant voltage error in every step, integrates without ever
 *   reversing, and a 50 mA offset puts the estimate 0.71 rad out in 100 ms. That is why a current-sense offset has to
 *   be calibrated out before this observer is trusted, and it is the number the README row now carries.
 *
 *   这个观测器是纯积分——`ψ += (v − Rs·i)·dt − L·di`， **没有修正项** ——所以唯一要紧的问题是哪些输入误差会累积、哪些不
 *   会。实测（`Rs = 0.5`、`L = 8e-4`、`ψf = 4.7619e-3`、`dt = 50 µs`）见上表。两个可操作的结论：
 *   **参数误差是偏置而不是跑飞** ——10 % 的 `Rs` 误差在 200 rad/s 花 0.19 rad、转速更高时更小，所以诚实的规格是"有界，
 *   `ε·Rs·iq/(ω·ψf)`（在它还很小时）"，而本用例把这条律失效的地方也记下来了（50 rad/s 那行，误差饱和到 π）。
 *   **传感器偏置才是跑飞的那一个** ：它每步都以恒定电压误差进入、永不反向地积分，50 mA 的偏置 100 ms 就把估计推出 0.71
 *   rad。所以信任这个观测器之前必须先把电流零偏标定掉，而这个数就是 README 行现在带着的那个。
 */
TEST(BackEmfObserver, TheResidueIsOneStepAndOnlyADcOffsetRunsAway) {
    // Row 1: the exact model leaves exactly one step of rotation, whatever the speed.
    // 第 1 行：精确模型只留下一步的旋转，与转速成比例。
    for (const float omega : {20.0F, 100.0F, 200.0F}) {
        const ProbeRun run = probe_run(omega, 3.0F, kProbeRs, 0.0F, 2000);
        const float one_step = omega * kProbeDt;
        std::printf("\n[back-emf] exact, omega %5.0f: max |err| %.6f rad, one step = %.6f\n", static_cast<double>(omega),
                    static_cast<double>(run.err_max_rad), static_cast<double>(one_step));
        EXPECT_LT(run.err_max_rad, 2.0F * one_step) << "omega " << omega << " is worse than one step of rotation";
        EXPECT_GT(run.err_max_rad, 0.4F * one_step) << "omega " << omega << " is suspiciously better than one step";
    }

    // Row 2: a resistance error is a bounded bias, and the small-error law tightens as the error shrinks.
    // 第 2 行：电阻误差是有界偏置，而小误差下的律随误差变小而收紧。
    for (const float omega : {200.0F, 400.0F}) {
        const ProbeRun run = probe_run(omega, 3.0F, kProbeRs * 1.10F, 0.0F, 8000);
        const float predicted = (0.10F * kProbeRs * 3.0F) / (omega * kProbePsiF);
        const float ratio = run.err_max_rad / predicted;
        std::printf("[back-emf] Rs +10 %%, omega %5.0f: max |err| %.5f rad vs eps*Rs*iq/(omega*psi_f) = %.5f (%.4fx)\n",
                    static_cast<double>(omega), static_cast<double>(run.err_max_rad), static_cast<double>(predicted),
                    static_cast<double>(ratio));
        EXPECT_GT(ratio, 1.0F) << "omega " << omega;
        EXPECT_LT(ratio, 1.35F) << "the small-error law was expected to hold to a third at " << omega << " rad/s";
    }
    // And where it stops holding: at 50 rad/s the estimate is a full pi out, which the law does not describe.
    // 以及它在哪里失效：50 rad/s 时估计整个偏出 π，那是这条律描述不了的。
    const ProbeRun slow = probe_run(50.0F, 3.0F, kProbeRs * 1.10F, 0.0F, 8000);
    const float slow_predicted = (0.10F * kProbeRs * 3.0F) / (50.0F * kProbePsiF);
    std::printf("[back-emf] Rs +10 %%, omega    50: max |err| %.5f rad against the law's %.5f -- saturated\n",
                static_cast<double>(slow.err_max_rad), static_cast<double>(slow_predicted));
    EXPECT_GT(slow.err_max_rad, 1.5F) << "the low-speed row was expected to show the law failing";
    EXPECT_GT(slow.err_max_rad / slow_predicted, 3.0F) << "at 50 rad/s the law must be visibly wrong";

    // Row 3: the DC offset, which is the one that integrates without ever reversing.
    // 第 3 行：直流偏置——唯一一个永不反向地积分的误差。
    const ProbeRun short_run = probe_run(200.0F, 3.0F, kProbeRs, 0.05F, 2000);
    const ProbeRun long_run = probe_run(200.0F, 3.0F, kProbeRs, 0.05F, 4000);
    std::printf("[back-emf] 50 mA of current offset: %.4f rad in 100 ms, %.4f rad in 200 ms\n",
                static_cast<double>(short_run.err_end_rad), static_cast<double>(long_run.err_end_rad));
    EXPECT_GT(std::fabs(short_run.err_end_rad), 0.5F) << "a 50 mA offset was expected to cost half a radian in 100 ms";
    EXPECT_GT(std::fabs(long_run.err_end_rad), std::fabs(short_run.err_end_rad))
        << "the offset case must keep growing, or it is not the runaway";

    // And the same observer with the offset removed stays put, so the growth is the offset and not the harness.
    // 而同一个观测器在去掉偏置之后原地不动，于是那条增长属于偏置而不是工具。
    const ProbeRun clean = probe_run(200.0F, 3.0F, kProbeRs, 0.0F, 2000);
    EXPECT_LT(std::fabs(clean.err_end_rad), 0.02F) << "the clean run drifted, so the comparison above means nothing";
}

/**
 * @brief [en] `apply_params()` takes the **q-axis** inductance, and a parameter set that cannot describe a machine
 *             disables the observer instead of leaving it running on the old one.
 *
 * @brief [zh] `apply_params()` 取的是 **q 轴** 电感，而一组描述不了机器的参数会把观测器关掉，而不是让它带着旧的继续跑。
 *
 *   Nothing had ever called `apply_params()` on this observer, which matters more here than elsewhere: the method sets
 *   `config_.l = params.lq` on a *single-L* model, so on any machine with `Ld != Lq` the observer is deliberately told
 *   to work with the q-axis value. That choice is the reason the `i_d != 0` operating point is handled by a ratio
 *   budget in `combinations.json` rather than by the model, and it had never been pinned by a case -- a version that
 *   took `ld` instead would have passed every existing test.
 *
 *   The check is behavioural rather than a field comparison: an observer re-configured to `lq` must answer exactly like
 *   one constructed with that `lq`, and (the falsifier) *differently* from one constructed with `ld`, on a machine
 *   where the two are far apart. The second half is what proves the case can tell the two apart at all.
 *
 *   此前没有任何东西调用过这个观测器的 `apply_params()`，而这里比别处更要紧：该方法在 **单一 L** 模型上设置 `config_.l
 *   = params.lq`，所以在任何 `Ld != Lq` 的机器上，观测器是被 **有意** 告知用 q 轴值的。这个选择正是 `i_d != 0` 工况由
 *   `combinations.json` 里的比率额度而不是由模型处理的原因，而它从未被一个用例钉住——一个取 `ld` 的版本会通过所有已有测
 *   试。
 *
 *   检查是行为式的而不是比对字段：被重配置到 `lq` 的观测器，必须与用那个 `lq` 构造的观测器 **逐位相同** ，并且（伪证）
 *   与用 `ld` 构造的那个 **不同** ——在两者相差很远的机器上。后一半才证明本用例分得开这两个。
 */
TEST(BackEmfObserver, ApplyParamsUsesTheQAxisInductanceAndRefusesAnImpossibleMachine) {
    constexpr float kLq = 8.0e-4F;
    constexpr float kLd = 2.0e-4F; // four times apart, so "which one" is visible in the answer
    const foc::core::MachineParams machine{kProbeRs, kLd, kLq, kProbePsiF};

    auto drive = [](foc::core::BackEmfObserver& observer) {
        observer.seed_flux(foc::core::Ab<float>{kProbePsiF, 0.0F});
        float theta = 0.0F;
        foc::core::ObsState state{};
        for (int k = 0; k < 200; ++k) {
            const float s = std::sin(theta);
            const float c = std::cos(theta);
            const float omega = 200.0F;
            const float iq = 3.0F;
            const float i_a = -iq * s;
            const float i_b = iq * c;
            const float v_a = (kProbeRs * i_a) - (omega * kProbePsiF * s) - (kLq * omega * iq * c);
            const float v_b = (kProbeRs * i_b) + (omega * kProbePsiF * c) - (kLq * omega * iq * s);
            state = observer.update(foc::core::Ab<float>{v_a, v_b}, foc::core::Ab<float>{i_a, i_b}, kProbeDt);
            theta += omega * kProbeDt;
        }
        return state.theta_e;
    };

    foc::core::BackEmfObserver reconfigured({kProbeRs, kLd}); // built with the *d* axis value on purpose
    reconfigured.apply_params(machine);
    foc::core::BackEmfObserver built_with_q({kProbeRs, kLq});
    foc::core::BackEmfObserver built_with_d({kProbeRs, kLd});

    const float from_params = drive(reconfigured);
    const float from_q = drive(built_with_q);
    const float from_d = drive(built_with_d);
    std::printf("\n[back-emf] apply_params theta %.6f | built with lq %.6f | built with ld %.6f\n",
                static_cast<double>(from_params), static_cast<double>(from_q), static_cast<double>(from_d));

    EXPECT_FLOAT_EQ(from_q, from_params) << "apply_params did not reproduce an observer built with `lq`";
    // The falsifier: on this machine `ld` gives a visibly different answer, so the equality above is a statement.
    // 伪证：在这台机器上 `ld` 给出的答案明显不同，所以上面那个等号是一句有内容的话。
    EXPECT_GT(std::fabs(wrap_pi_probe(from_d - from_q)), 1.0e-3F)
        << "ld and lq gave the same answer here, so this case cannot tell which one apply_params took";

    // A parameter set that cannot describe a machine disables the observer rather than leaving it on the old model.
    // 一组描述不了机器的参数会把观测器关掉，而不是让它留在旧模型上。
    foc::core::BackEmfObserver broken({kProbeRs, kLq});
    broken.apply_params(foc::core::MachineParams{kProbeRs, kLd, 0.0F, kProbePsiF}); // lq == 0
    EXPECT_EQ(foc::ErrorCode::Param, broken.status());
    const auto held = broken.update(foc::core::Ab<float>{1.0F, 1.0F}, foc::core::Ab<float>{0.1F, 0.1F}, kProbeDt);
    EXPECT_FLOAT_EQ(0.0F, held.theta_e) << "a disabled observer must hold, not answer from the old parameters";
}
