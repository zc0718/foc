// S15 - sensorless start-up: saliency finds the d axis, saturation says which half of it is north.
// S15 - 无感起动：凸极找到 d 轴，饱和判断哪半边是北。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/hfi_observer.hpp>
#include <foc/core/polarity.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_pmsm_plant.hpp"
#include "sil_plant_hal.hpp"

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

constexpr float kPiFloat = 3.14159265358979F;
constexpr float kTwoPiFloat = 6.28318530717959F;

constexpr float kRs = 0.35F;
constexpr float kLd = 2.0e-4F;
constexpr float kLq = 4.0e-4F;
constexpr float kKt = 0.05F;
constexpr float kJ = 1.0e-4F;
constexpr float kB = 1.0e-5F;
constexpr std::uint8_t kPoles = 7U;

constexpr float kCtrlHz = 20000.0F;
constexpr float kDt = 1.0F / kCtrlHz;
constexpr int kSubsteps = 8;

// How strongly the test machine saturates. Real numbers are a fit, not a nameplate figure, so what
// matters is that the documented config is the one the case was verified at.
// 测试机饱和的强弱。真实数值是拟合而非铭牌，因此要紧的是"用例验证时用的就是文档里这份配置"。
constexpr float kSatDepth = 0.35F;
constexpr float kSatCurrentA = 0.6F;

auto plant_params() -> sil::PlantParams { return sil::PlantParams{kRs, kLd, kLq, kKt, kJ, kB, kPoles}; }

/// [en] The same machine with its d-axis inductance collapsing under positive current: the property
///      that makes the magnet polarity observable at all.
///      [zh] 同一台电机，但 d 轴电感在正电流下塌陷——正是这条性质让磁钢极性变得可观测。
auto saturating_plant(float theta_e_true) -> sil::PmsmPlant {
    sil::PmsmPlant plant(plant_params());
    plant.set_d_axis_saturation(kSatDepth, kSatCurrentA);
    plant.reset(sil::PlantState{0.0F, 0.0F, 0.0F, theta_e_true / static_cast<float>(kPoles)});
    return plant;
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

auto hfi_config() -> core::HfiObserver::Config {
    core::HfiObserver::Config config{};
    config.inject_v = 2.0F;
    config.inject_hz = 2000.0F;
    config.current_hp_hz = 300.0F;
    config.demod_lpf_hz = 200.0F;
    config.pll_bandwidth_hz = 40.0F;
    config.pll_zeta = 0.9F;
    config.demod_gain_a = core::HfiObserver::predicted_gain_a(config.inject_v, config.inject_hz, kLd, kLq);
    config.min_hf_current_a = 0.05F;
    return config;
}

/// [en] 1 V for 0.4 ms is 4e-4 V*s, about 2 A of rise into 2e-4 H: big enough to engage saturation,
///      short enough that the rotor does not notice.
///      [zh] 1 V 持续 0.4 ms 是 4e-4 V·s，灌进 2e-4 H 约抬升 2 A：大到足以触发饱和，短到转子察觉不到。
auto polarity_config() -> core::MagnetPolarityTest::Config {
    return core::MagnetPolarityTest::Config{1.0F, 4e-4F, 5e-3F, 0.02F};
}

/**
 * @brief [en] Commissioning sequence exactly as documented: HFI converges, the polarity test pulses
 *        along the estimate, the answer is applied, and only then is the loop handed over.
 *        [zh] 完全按文档的调试顺序：HFI 收敛 → 极性测试沿估计脉冲 → 应用结论 → 之后才交出环路。
 */
struct Commissioning final {
    sil::PmsmPlant plant;
    core::HfiObserver observer;
    core::PolarityVerdict verdict{core::PolarityVerdict::Inconclusive};
    float delta_positive_a{0.0F};
    float delta_negative_a{0.0F};

    explicit Commissioning(float theta_e_true, bool apply_correction, bool saturating = true) noexcept
        : plant(saturating ? saturating_plant(theta_e_true) : silent_plant(theta_e_true)),
          observer(hfi_config()), apply_correction_(apply_correction) {}

    Commissioning(const Commissioning&) = delete;
    Commissioning& operator=(const Commissioning&) = delete;
    Commissioning(Commissioning&&) = delete;
    Commissioning& operator=(Commissioning&&) = delete;

    static auto silent_plant(float theta_e_true) -> sil::PmsmPlant {
        sil::PmsmPlant plant(plant_params());
        plant.reset(sil::PlantState{0.0F, 0.0F, 0.0F, theta_e_true / static_cast<float>(kPoles)});
        return plant;
    }

    [[nodiscard]] auto true_angle() const noexcept -> float {
        return static_cast<float>(kPoles) * plant.theta_mech_rad();
    }

    [[nodiscard]] auto estimate() const noexcept -> float { return observer.state().theta_e; }

    [[nodiscard]] auto error_rad() const noexcept -> float { return wrap_pi_local(estimate() - true_angle()); }

    /// [en] Phase 1: let the saliency loop find the d-axis line. [zh] 第一阶段：让凸极环找到 d 轴线。
    void converge(float seconds) noexcept {
        const int steps = static_cast<int>((seconds / kDt) + 0.5F);
        for (int i = 0; i < steps; ++i) {
            static_cast<void>(observer.update(core::Ab<float>{0.0F, 0.0F}, core::clarke(plant.current_uvw()), kDt));
            apply(observer.injection_ab());
        }
    }

    /// [en] Phase 2: pulse along the estimate and apply the verdict. [zh] 第二阶段：沿估计脉冲并应用结论。
    void resolve_polarity() noexcept {
        core::MagnetPolarityTest test(polarity_config());
        test.begin(observer.state().theta_e);
        for (int i = 0; i < 20000; ++i) {
            const core::MagnetPolarityTest::Output out = test.step(kDt, core::clarke(plant.current_uvw()));
            apply(out.v_ab);
            if (!out.running) {
                verdict = out.verdict;
                delta_positive_a = out.delta_positive_a;
                delta_negative_a = out.delta_negative_a;
                break;
            }
        }
        observer.apply_polarity(apply_correction_ && (verdict == core::PolarityVerdict::Reversed));
    }

  private:
    void apply(core::Ab<float> v_ab) noexcept {
        for (int i = 0; i < kSubsteps; ++i) {
            const float theta_e = static_cast<float>(kPoles) * plant.theta_mech_rad();
            plant.set_voltage_dq(core::park(v_ab, theta_e));
            plant.advance(kDt / static_cast<float>(kSubsteps));
        }
    }

    bool apply_correction_;
};

} // namespace

TEST(SilPolarity, SaturationGivesThePolarityTestSomethingToMeasure) {
    // The plant has to show it before any conclusion drawn from it means anything: the incremental
    // d-axis inductance must fall with positive current. Without this the whole scenario would be
    // measuring nothing and agreeing with itself.
    // plant 必须先自己展示出来，否则由它得出的任何结论都没有意义：增量 d 轴电感必须随正电流下降。
    // 没有这一点，整个场景等于什么也没测、只是自我印证。
    const sil::PmsmPlant plant = saturating_plant(0.0F);
    EXPECT_LT(plant.effective_ld(1.0F), plant.effective_ld(0.0F));
    EXPECT_GT(plant.effective_ld(-1.0F), plant.effective_ld(0.0F));
    // Off by default, so every scenario written before this model existed is untouched.
    // 默认关闭，因此本模型存在之前写的每个场景都不受影响。
    const sil::PmsmPlant ideal(plant_params());
    EXPECT_FLOAT_EQ(kLd, ideal.effective_ld(3.0F));
}

TEST(SilPolarity, ItSaysAlignedWhenTheSaliencyLoopAlreadyLandedOnTheMagnet) {
    // Rotor half a radian from where the estimate starts: inside the +-pi/2 window, so the loop lands
    // on the correct half and the pulse pair should confirm it.
    // 转子距估计起点半弧度：在 ±π/2 窗口内，于是环路落在正确的那半边，脉冲对应当确认它。
    Commissioning run(0.5F, true);
    run.converge(1.0F);
    ASSERT_LT(std::fabs(run.error_rad()), 0.05F);
    run.resolve_polarity();

    EXPECT_EQ(core::PolarityVerdict::Aligned, run.verdict);
    // The reinforcing pulse is the one that rose further -- that is the whole reason the verdict is
    // Aligned rather than a coincidence of the threshold.
    // 上升更多的那个是增强磁链的脉冲——这才是结论为 Aligned 的原因，而不是阈值的巧合。
    EXPECT_GT(std::fabs(run.delta_positive_a), std::fabs(run.delta_negative_a));
    std::printf("[S15] aligned: delta+ %.4f A, delta- %.4f A\n", static_cast<double>(run.delta_positive_a),
                static_cast<double>(run.delta_negative_a));
    // The correction is a no-op when the answer is Aligned: the estimate must not move.
    // 结论为 Aligned 时修正是空操作：估计不得移动。
    EXPECT_LT(std::fabs(run.error_rad()), 0.05F);
}

TEST(SilPolarity, StartingOutsideTheWindowIsCaughtRatherThanTrusted) {
    // Two radians out is past the +-pi/2 window, so the saliency loop locks to the pi mirror: it
    // converges *confidently* and wrongly. That is the failure the polarity test exists for.
    // 偏两弧度超出 ±π/2 窗口，于是凸极环锁到 π 镜像：它**自信地**收敛到了错的地方。这正是极性测试要
    // 抓的失效。
    Commissioning run(2.0F, true);
    run.converge(1.0F);
    ASSERT_NEAR(kPiFloat, std::fabs(run.error_rad()), 0.05F) << "the mirror is the premise of this case";

    run.resolve_polarity();
    EXPECT_EQ(core::PolarityVerdict::Reversed, run.verdict);
    // After the correction the estimate is right, and nothing else was needed to know it.
    // 修正之后估计是对的，而知道这件事并不需要别的信息。
    EXPECT_LT(std::fabs(run.error_rad()), 0.05F);
}

TEST(SilPolarity, WithoutTheCorrectionTheMirrorStaysAndTheDriveIsUpsideDown) {
    // Same run, correction withheld: the estimate stays half a turn out, which is what makes a torque
    // command turn the joint the wrong way. Recorded as a measurement so the cost of skipping the
    // step is not a mystery.
    // 同一次运行但不用修正：估计保持差半圈，这就是"转矩指令把关节转反"的来源。记成测量值，让跳过这一步
    // 的代价不是个谜。
    Commissioning run(2.0F, false);
    run.converge(1.0F);
    run.resolve_polarity();

    EXPECT_EQ(core::PolarityVerdict::Reversed, run.verdict);
    EXPECT_NEAR(kPiFloat, std::fabs(run.error_rad()), 0.05F);
    // Half a turn out means the estimate is the negative of the truth: a command expressed in it comes
    // out inverted at the terminals.
    // 差半圈意味着估计是真值的取反：以它为参考给出的指令到端子上是反的。
    EXPECT_NEAR(-1.0F, std::cos(run.error_rad()), 0.05F) << "the estimate must be anti-parallel, not merely wrong";
}

TEST(SilPolarity, AMachineWithoutSaturationGetsNoAnswerAndNoCorrection) {
    // No saturation, no asymmetry, no information. The third verdict is what keeps the sequence honest:
    // the caller is told "this machine cannot answer" and applies no correction, rather than being
    // handed a coin flip that looks like a measurement.
    // 没有饱和就没有不对称、没有信息。第三种结论让整个流程保持诚实：调用方被告知"这台机器答不了"、不做
    // 任何修正，而不是拿到一个看起来像测量值的抛硬币结果。
    Commissioning run(2.0F, true, /*saturating=*/false);
    run.converge(1.0F);
    run.resolve_polarity();

    EXPECT_EQ(core::PolarityVerdict::Inconclusive, run.verdict);
    EXPECT_NEAR(kPiFloat, std::fabs(run.error_rad()), 0.05F) << "no answer must mean no correction";
    std::printf("[S15] no saturation: delta+ %.4f A, delta- %.4f A\n", static_cast<double>(run.delta_positive_a),
                static_cast<double>(run.delta_negative_a));
}

TEST(SilPolarity, ThePulsesLeaveTheRotorWhereTheyFoundIt) {
    // A polarity test that moved the rotor would be measuring a different machine position on its
    // second pulse, and it would also be a small uncontrolled motion on every power-up.
    // 会推动转子的极性测试，第二个脉冲测的就是另一个转子位置了；而且每次上电都会多一次不受控的小动作。
    Commissioning run(0.5F, true);
    run.converge(1.0F);
    const float theta_before = run.plant.theta_mech_rad();
    run.resolve_polarity();
    const float moved = std::fabs(wrap_pi_local(run.plant.theta_mech_rad() - theta_before));
    EXPECT_LT(moved, 0.05F) << "the pulses must be along d, not q";
}

namespace {

using HfiMotor = foc::motor::Motor<core::HfiObserver, core::Svpwm, core::Pi, sil::PlantHal>;

auto hfi_motor_config() -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{kRs, kLd,   kLq,  kKt,  kJ, kB, kPoles, 3.0F, 20000.0F, 0.0F, 0.0F, 0.0F};
}

auto policies_with(core::HfiObserver observer) -> HfiMotor::Policies {
    const foc::motor::MotorConfig mc = hfi_motor_config();
    return HfiMotor::Policies{observer, core::Svpwm{}, core::Pi{foc::current_pi_config(mc, 2000.0F, 24.0F)},
                              core::Pi{foc::current_pi_config(mc, 2000.0F, 24.0F)},
                              core::Pi{foc::speed_pi_config(mc, 250.0F)}};
}

} // namespace

TEST(SilPolarity, TheCorrectionIsWhatStopsTheJointTurningBackwards) {
    // The point of the whole exercise, measured instead of argued. Same commissioning run, same torque
    // command, same plant -- only the correction differs, and the joint either turns the way it was
    // told or the opposite way. A handed-over observer is copied into the motor, which is exactly the
    // documented hand-over: commission on the bench, then close the loop with what you learned.
    // 整件事的意义，用测量而不是论证给出。同一次调试运行、同一条转矩指令、同一台 plant——只有修不修正
    // 不同，关节要么按被要求的方向转，要么反着转。交接是把观测器拷贝进电机，这正是文档写的交接方式：
    // 先在台架上调试，再用得到的结论闭环。
    float speed_without_correction = 0.0F;
    float speed_with_correction = 0.0F;

    for (int variant = 0; variant < 2; ++variant) {
        const bool correct = (variant == 1);
        Commissioning run(2.0F, correct); // two radians out: the mirror is the premise
        run.converge(1.0F);
        run.resolve_polarity();
        ASSERT_EQ(core::PolarityVerdict::Reversed, run.verdict);

        sil::PlantHal hal(run.plant, 24.0F);
        hal.position.set_sensor_present(false);

        HfiMotor motor(hfi_motor_config(), policies_with(run.observer), hal);
        ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
        ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
        motor.set_torque(0.5F); // a positive torque request: the joint must turn positive

        const int steps = static_cast<int>(0.05F / kDt);
        for (int i = 0; i < steps; ++i) {
            ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
            hal.advance(kDt, kSubsteps);
        }

        const float speed = run.plant.omega_mech_rad_s();
        if (correct) {
            speed_with_correction = speed;
        } else {
            speed_without_correction = speed;
        }
        std::printf("[S15] correction %d: commanded +0.5 A, mechanical speed %+0.3f rad/s\n", correct ? 1 : 0,
                    static_cast<double>(speed));
    }

    EXPECT_GT(speed_with_correction, 1.0F) << "the corrected estimate must produce the commanded direction";
    EXPECT_LT(speed_without_correction, -1.0F) << "the mirror must produce the opposite direction";
}
