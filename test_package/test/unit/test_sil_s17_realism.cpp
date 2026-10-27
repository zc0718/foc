// S17 - the realism knobs, and the reverse assertions that prove they are connected.
// S17 - 真实性旋钮，以及证明它们**真的接上了**的反向断言。

#include <cmath>
#include <cstdio>
#include <initializer_list>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/hfi_observer.hpp>
#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/transforms.hpp>

#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

constexpr float kPiFloat = 3.14159265358979F;
constexpr float kTwoPiFloat = 6.28318530717959F;

constexpr float kRs = 0.35F;
constexpr float kLd = 2.0e-4F;
constexpr float kLq = 4.0e-4F;
constexpr float kKt = 0.05F;
constexpr std::uint8_t kPoles = 7U;

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 8;
constexpr float kIq = 0.5F;

/// [en] Copper's temperature coefficient: 0.393 %/K, the value that makes a 120 K rise a 47 % change
///      in resistance. [zh] 铜的温度系数：0.393 %/K，于是 120 K 升温带来 47% 的电阻变化。
constexpr float kCopperAlpha = 3.93e-3F;

auto plant_params() -> sil::PlantParams {
    return sil::PlantParams{kRs, kLd, kLq, kKt, 1.0e6F, 0.0F, kPoles};
}

auto wrap_pi_local(float angle) noexcept -> float {
    while (angle <= -kPiFloat) {
        angle += kTwoPiFloat;
    }
    while (angle > kPiFloat) {
        angle -= kTwoPiFloat;
    }
    return angle;
}

/**
 * @brief [en] Fixed-speed rig: the rotor is held at a constant electrical speed and fed the steady-state
 *        voltage for `id = 0, iq = kIq`, so an observer sees a real operating point.
 *        [zh] 恒速台架：转子以恒定电角速度运行并施加 `id = 0, iq = kIq` 的稳态电压，于是观测器看到的是
 *        真实工作点。
 * @since 0.1.0
 */
class FluxRig final {
  public:
    explicit FluxRig(float omega_e) noexcept : plant_(plant_params()), omega_e_(omega_e) {
        plant_.reset(sil::PlantState{0.0F, kIq, omega_e / static_cast<float>(kPoles), 0.0F});
        const float psi_f = kKt / (1.5F * static_cast<float>(kPoles));
        vd_ = -omega_e * kLq * kIq;
        vq_ = (kRs * kIq) + (omega_e * psi_f);
    }

    FluxRig(const FluxRig&) = delete;
    FluxRig& operator=(const FluxRig&) = delete;
    FluxRig(FluxRig&&) = delete;
    FluxRig& operator=(FluxRig&&) = delete;

    [[nodiscard]] auto plant() noexcept -> sil::PmsmPlant& { return plant_; }

    /// [en] Heat the winding AND re-solve the steady-state voltage for it. Without the second half, heating
    ///      would quietly move the operating point (iq sags) and the test would be measuring the wrong thing.
    ///      [zh] 加热绕组，并**为它重解**稳态电压。少了后半句，升温会悄悄把工作点挪走（iq 塌下去），
    ///      测试量到的就是另一回事了。
    void set_temperature(float temperature_c, float alpha_per_k) noexcept {
        plant_.set_winding_temperature(temperature_c, alpha_per_k);
        const float rs = plant_.effective_rs();
        vd_ = -omega_e_ * kLq * kIq;
        vq_ = (rs * kIq) + (omega_e_ * psi_f());
    }
    [[nodiscard]] auto omega_e() const noexcept -> float { return omega_e_; }
    [[nodiscard]] auto psi_f() const noexcept -> float { return kKt / (1.5F * static_cast<float>(kPoles)); }

    [[nodiscard]] auto true_angle_e() const noexcept -> float {
        return static_cast<float>(kPoles) * plant_.theta_mech_rad();
    }

    void step() noexcept {
        // The observer's update is a backward-Euler step anchored at the sample instant -- it pairs
        // `dt * (v - Rs*i)` with `L * (i_k - i_{k-1})` -- so it must be handed the voltage belonging to the
        // current sample, not the one from the period that just ended. See the note on `update()`.
        // 观测器的 update 是以采样时刻为锚点的后向欧拉步——它把 `dt*(v - Rs*i)` 与 `L*(i_k - i_{k-1})`
        // 配对——因此必须拿到属于当前采样的电压，而不是刚刚结束那一周期的电压。见 `update()` 上的说明。
        last_current_ = core::clarke(plant_.current_uvw());
        last_voltage_ = core::inv_park(core::Dq<float>{vd_, vq_}, true_angle_e());
        const core::Dq<float> v_dq = core::Dq<float>{vd_, vq_};
        for (int i = 0; i < kSubsteps; ++i) {
            plant_.set_voltage_dq(v_dq);
            plant_.advance(kDt / static_cast<float>(kSubsteps));
        }
    }

    [[nodiscard]] auto last_current() const noexcept -> core::Ab<float> { return last_current_; }

    [[nodiscard]] auto last_voltage() const noexcept -> core::Ab<float> { return last_voltage_; }

  private:
    sil::PmsmPlant plant_;
    float omega_e_;
    float vd_;
    float vq_;
    core::Ab<float> last_current_{0.0F, 0.0F};
    core::Ab<float> last_voltage_{0.0F, 0.0F};
};

/// [en] Mean absolute angle error of the flux observer over the second half of a run.
///      [zh] 磁链观测器在后半段的角度误差绝对均值。
auto flux_angle_error(FluxRig& rig, float belief_scale, float seconds) -> float {
    core::BackEmfObserver observer(core::BackEmfObserver::Config{belief_scale * kRs, kLq});
    observer.seed_flux(core::Ab<float>{rig.psi_f(), 0.0F});

    float sum = 0.0F;
    int count = 0;
    const int steps = static_cast<int>((seconds / kDt) + 0.5F);
    for (int k = 0; k < steps; ++k) {
        rig.step();
        const core::ObsState observed = observer.update(rig.last_voltage(), rig.last_current(), kDt);
        if (k >= (steps / 2)) {
            sum += std::fabs(wrap_pi_local(observed.theta_e - rig.true_angle_e()));
            ++count;
        }
    }
    return (count > 0) ? (sum / static_cast<float>(count)) : 0.0F;
}

} // namespace

TEST(SilS17Realism, TheKnobsAreOffByDefault) {
    // Every scenario written before these knobs existed has to stay bit-for-bit unchanged, otherwise
    // the 16 SIL cases stop being comparable to their own recorded numbers.
    // 这些旋钮之前写的每个场景都必须逐位不变，否则 16 个 SIL 用例就与自己记录的数字不可比了。
    sil::PmsmPlant plant(plant_params());
    EXPECT_FLOAT_EQ(kRs, plant.effective_rs()) << "no temperature model, no drift";
    EXPECT_FLOAT_EQ(kLd, plant.effective_ld(5.0F)) << "no saturation model, no change";

    // Two reads of an identical state must be identical: no noise.
    // 同一状态的两次读数必须完全相同：没有噪声。
    EXPECT_FLOAT_EQ(plant.current_uvw().u, plant.current_uvw().u);
    EXPECT_FLOAT_EQ(plant.current_uvw().v, plant.current_uvw().v);

    // And the dead zone is the ideal step, not a ramp.
    // 而死亡区是理想阶跃，不是斜坡。
    sil::PlantHal hal(plant, 24.0F);
    EXPECT_FLOAT_EQ(0.0F, hal.deadtime_voltage());
}

TEST(SilS17Realism, WindingHeatMakesTheFluxObserverWorse) {
    // The observer keeps believing Rs at 20 C while the winding heats up. This is the assertion that
    // gives a future parameter identifier something to fix -- without it, "identify Rs" would be a
    // solution looking for a problem.
    // 观测器始终相信 20 °C 的 Rs，而绕组在升温。这条断言给了将来的参数辨识一个**待修的东西**——
    // 没有它，"辨识 Rs" 就是在给一个不存在的问题找解法。
    constexpr float kOmega = 200.0F;

    FluxRig cold(kOmega);
    const float at_twenty = flux_angle_error(cold, 1.10F, 0.4F);

    FluxRig hot(kOmega);
    hot.set_temperature(140.0F, kCopperAlpha);
    const float at_hot = flux_angle_error(hot, 1.10F, 0.4F);

    std::printf("[S17] flux error at 200 rad/s: %.4f rad at 20 C, %.4f rad at 140 C (Rs x%.2f)\n",
                static_cast<double>(at_twenty), static_cast<double>(at_hot),
                static_cast<double>(hot.plant().effective_rs() / kRs));
    EXPECT_NEAR(kRs * 1.4716F, hot.plant().effective_rs(), 1e-3F) << "the temperature model itself";
    EXPECT_GT(at_hot, 1.5F * at_twenty) << "heat must actually cost the observer something";
}

TEST(SilS17Realism, SensorNoiseRaisesTheSaliencyLoopFloor) {
    // A saliency loop reads its angle out of a current of a few hundred milliamps of carrier; 0.1 A of
    // sensor noise is a fifth of that. If the estimator is not affected, the noise is not connected.
    // 凸极环从几百毫安载波电流中读角度；0.1 A 的传感器噪声是它的五分之一。若估计器毫无反应，说明噪声没接上。
    const auto run = [](float noise_a) -> float {
        sil::PmsmPlant plant(plant_params());
        plant.reset(sil::PlantState{0.0F, 0.0F, 0.0F, 0.3F / static_cast<float>(kPoles)});
        plant.set_current_noise(noise_a);

        core::HfiObserver::Config config{};
        config.inject_v = 2.0F;
        config.inject_hz = 2000.0F;
        config.current_hp_hz = 300.0F;
        config.demod_lpf_hz = 200.0F;
        config.pll_bandwidth_hz = 40.0F;
        config.pll_zeta = 0.9F;
        config.demod_gain_a = core::HfiObserver::predicted_gain_a(config.inject_v, config.inject_hz, kLd, kLq);
        config.min_hf_current_a = 0.05F;
        core::HfiObserver observer(config);

        float sum = 0.0F;
        int count = 0;
        const int steps = static_cast<int>(1.0F / kDt);
        for (int k = 0; k < steps; ++k) {
            const core::Ab<float> i_ab = core::clarke(plant.current_uvw());
            static_cast<void>(observer.update(core::Ab<float>{0.0F, 0.0F}, i_ab, kDt));
            const core::Ab<float> v_ab = observer.injection_ab();
            for (int i = 0; i < kSubsteps; ++i) {
                const float theta_e = static_cast<float>(kPoles) * plant.theta_mech_rad();
                plant.set_voltage_dq(core::park(v_ab, theta_e));
                plant.advance(kDt / static_cast<float>(kSubsteps));
            }
            if (k >= (steps / 2)) {
                const float truth = static_cast<float>(kPoles) * plant.theta_mech_rad();
                sum += std::fabs(wrap_pi_local(observer.state().theta_e - truth));
                ++count;
            }
        }
        return sum / static_cast<float>(count);
    };

    const float clean = run(0.0F);
    const float noisy = run(0.1F);
    std::printf("[S17] HFI error at standstill: %.4f rad clean, %.4f rad with 0.1 A noise\n",
                static_cast<double>(clean), static_cast<double>(noisy));
    EXPECT_LT(clean, 0.05F);
    EXPECT_GT(noisy, 2.0F * clean) << "noise must raise the floor";
}

TEST(SilS17Realism, RightParametersFromTheStartAreEightTimesBetter) {
    // P1's payoff, stated as a number, and stated the way it actually works. Measured at 200 rad/s with
    // the winding at 140 C: an observer believing Rs 10 % high lands at 0.0398 rad, one believing the
    // true hot value at 0.0050. Parameter accuracy is not a refinement here, it is a factor of eight.
    // P1 的兑现，用数字说，而且按它**实际生效**的方式说。200 rad/s、绕组 140 °C 实测：相信 Rs 偏高 10% 的
    // 观测器落在 0.0398 rad，相信真实热态值的落在 0.0050。参数准确性在这里不是"精细化"，是八倍。
    constexpr float kOmega = 200.0F;

    FluxRig wrong(kOmega);
    wrong.set_temperature(140.0F, kCopperAlpha);
    const float with_wrong_belief = flux_angle_error(wrong, 1.10F, 0.4F);

    FluxRig right(kOmega);
    right.set_temperature(140.0F, kCopperAlpha);
    const float belief = right.plant().effective_rs() / kRs; // what an identifier would have measured
    const float with_right_belief = flux_angle_error(right, belief, 0.4F);

    std::printf("[S17] flux error at 140 C / 200 rad/s: %.4f rad believing Rs x%.2f, %.4f believing x%.2f\n",
                static_cast<double>(with_wrong_belief), static_cast<double>(1.10F),
                static_cast<double>(with_right_belief), static_cast<double>(belief));
    EXPECT_LT(with_right_belief, 0.35F * with_wrong_belief) << "the parameters are the difference";
}

TEST(SilS17Realism, AnUpdateMidRunDoesNotUndoWhatTheIntegratorAlreadyAccumulated) {
    // The uncomfortable half, and the reason `apply_params` alone is not a fix. A flux observer integrates
    // `v - Rs*i`, so an error in Rs leaves a *flux* error behind, and the integrator has no reset. Telling it
    // the right Rs corrects the integrand from that moment on but cannot remove what is already integrated --
    // and measured, it makes the angle error roughly *twice* as bad (0.0402 -> 0.0803), because the leftover
    // vector and the newly-correct rate stop cancelling.
    // 不舒服的那一半，也正是"只做 `apply_params` 不算修好"的原因。磁链观测器积分 `v − Rs·i`，Rs 的误差会留下
    // 一个磁链误差，而积分器没有复位。告诉它正确的 Rs 只修正从此往后的被积函数，清不掉已经积出来的东西——而且
    // 实测角度误差大约**翻倍**（0.0402 → 0.0803），因为残留矢量与新变正确的变化率不再相互抵消。
    constexpr float kOmega = 200.0F;
    constexpr float kSwitchAt = 0.2F;

    FluxRig rig(kOmega);
    rig.set_temperature(140.0F, kCopperAlpha);
    const float hot_rs = rig.plant().effective_rs();

    core::BackEmfObserver observer(core::BackEmfObserver::Config{1.10F * kRs, kLq});
    observer.seed_flux(core::Ab<float>{rig.psi_f(), 0.0F});

    float sum = 0.0F;
    int count = 0;
    bool switched = false;
    const int steps = static_cast<int>(0.6F / kDt);
    for (int k = 0; k < steps; ++k) {
        rig.step();
        const core::ObsState observed = observer.update(rig.last_voltage(), rig.last_current(), kDt);
        const float t = static_cast<float>(k) * kDt;
        if ((t > 0.3F)) {
            sum += std::fabs(wrap_pi_local(observed.theta_e - rig.true_angle_e()));
            ++count;
        }
        if (!switched && (t >= kSwitchAt)) {
            observer.apply_params(core::MachineParams{hot_rs, kLd, kLq, rig.psi_f()});
            switched = true;
        }
    }
    const float after_update = sum / static_cast<float>(count);

    FluxRig reference(kOmega);
    reference.set_temperature(140.0F, kCopperAlpha);
    const float without_update = flux_angle_error(reference, 1.10F, 0.6F);

    std::printf("[S17] mid-run update: %.4f rad after switching to the true Rs, %.4f rad without switching\n",
                static_cast<double>(after_update), static_cast<double>(without_update));
    // Stated as a warning, because "just update the parameters" is the intuitive fix and it is wrong for
    // anything that integrates: re-seed from a known angle instead, or close a loop that pulls the flux
    // magnitude back (which is exactly what the next step in this plan does).
    // 作为警示记下：因为"更新一下参数"是直觉上的修法，而对任何带积分的东西都是错的——应当从已知角度重新播种，
    // 或者加一个把磁链幅值拉回来的环（这正是本计划的下一步）。
    EXPECT_GT(after_update, 1.5F * without_update) << "a mid-run update is not a reset";
}

TEST(SilS17Realism, TheMagnitudeCorrectionBeatsTheOpenLoopObserverOnAverage) {
    // The judgement the plan fixed in advance: NLFO's mean angle error over the speed set must be *strictly*
    // lower than the open-loop observer's. Both run on the same fixture, at the same points, with Rs believed
    // 10 % high. Recorded per point so a future regression shows where it happened, not just that a mean moved.
    // 计划事先定死的判决：NLFO 在该速度集上的平均角度误差必须**严格**低于开环观测器。两者在同一台架、同一批
    // 点、Rs 信念偏高 10% 下运行。逐点记录，这样将来回退时能看到是哪一点坏了，而不是只知道均值动了。
    const float beliefs[] = {1.10F, 1.30F};
    const float speeds[] = {20.0F, 50.0F, 100.0F, 200.0F, 400.0F, 800.0F};
    std::printf("\n[S17] %8s %12s %12s %8s\n", "omega", "open-loop", "nonlinear", "ratio");
    for (const float belief : beliefs) {
        float sum_open = 0.0F;
        float sum_nonlinear = 0.0F;
        for (const float omega : speeds) {
            FluxRig open_rig(omega);
            const float open = flux_angle_error(open_rig, belief, 0.4F);

            FluxRig nonlinear_rig(omega);
            core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
                belief * kRs, kLq, nonlinear_rig.psi_f(), 0.3F, 3.0F, 0.5F * nonlinear_rig.psi_f()});
            observer.seed_flux(core::Ab<float>{nonlinear_rig.psi_f(), 0.0F});
            float sum = 0.0F;
            int count = 0;
            const int steps = static_cast<int>(0.4F / kDt);
            for (int k = 0; k < steps; ++k) {
                nonlinear_rig.step();
                const core::ObsState observed =
                    observer.update(nonlinear_rig.last_voltage(), nonlinear_rig.last_current(), kDt);
                if (k >= (steps / 2)) {
                    sum += std::fabs(wrap_pi_local(observed.theta_e - nonlinear_rig.true_angle_e()));
                    ++count;
                }
            }
            const float nonlinear = sum / static_cast<float>(count);
            std::printf("[S17] %8.0f %12.4f %12.4f %8.2f\n", static_cast<double>(omega),
                        static_cast<double>(open), static_cast<double>(nonlinear),
                        static_cast<double>(open / nonlinear));
            sum_open += open;
            sum_nonlinear += nonlinear;
        }
        std::printf("[S17] belief x%.2f: mean open-loop %.4f, mean nonlinear %.4f\n", static_cast<double>(belief),
                    static_cast<double>(sum_open / 6.0F), static_cast<double>(sum_nonlinear / 6.0F));
        // The plan's P3-a, unchanged since it was written: strictly better on average.
        // 计划的 P3-a，自写下起未改动：均值上严格更好。
        EXPECT_LT(sum_nonlinear, sum_open) << "belief " << belief;
    }
}
