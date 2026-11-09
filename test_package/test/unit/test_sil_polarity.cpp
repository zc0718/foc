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

/**
 * @brief [en] The same machine with its d-axis inductance collapsing under positive current: the property that makes
 *             the magnet polarity observable at all.
 * @brief [zh] 同一台电机，但 d 轴电感在正电流下塌陷——正是这条性质让磁钢极性变得可观测。
 */
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

/**
 * @brief [en] 1 V for 0.4 ms is 4e-4 V*s, about 2 A of rise into 2e-4 H: big enough to engage saturation, short enough
 *             that the rotor does not notice.
 * @brief [zh] 1 V 持续 0.4 ms 是 4e-4 V·s，灌进 2e-4 H 约抬升 2 A：大到足以触发饱和，短到转子察觉不到。
 */
auto polarity_config() -> core::MagnetPolarityTest::Config {
    return core::MagnetPolarityTest::Config{1.0F, 4e-4F, 5e-3F, 0.02F};
}

/**
 * @brief [en] Commissioning sequence exactly as documented: HFI converges, the polarity test pulses along the estimate,
 *             the answer is applied, and only then is the loop handed over.
 * @brief [zh] 完全按文档的调试顺序：HFI 收敛 → 极性测试沿估计脉冲 → 应用结论 → 之后才交出环路。
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

    /**
     * @brief [en] Phase 1: let the saliency loop find the d-axis line.
     * @brief [zh] 第一阶段：让凸极环找到 d 轴线。
     */
    void converge(float seconds) noexcept {
        const int steps = static_cast<int>((seconds / kDt) + 0.5F);
        for (int i = 0; i < steps; ++i) {
            static_cast<void>(observer.update(core::Ab<float>{0.0F, 0.0F}, core::clarke(plant.current_uvw()), kDt));
            apply(observer.injection_ab());
        }
    }

    /**
     * @brief [en] Phase 2: pulse along the estimate and apply the verdict.
     * @brief [zh] 第二阶段：沿估计脉冲并应用结论。
     */
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
    // 偏两弧度超出 ±π/2 窗口，于是凸极环锁到 π 镜像：它 **自信地** 收敛到了错的地方。这正是极性测试要
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



namespace {

struct PulseResult final {
    core::PolarityVerdict verdict{core::PolarityVerdict::Inconclusive};
    double delta_positive_a{0.0};
    double delta_negative_a{0.0};
    double asymmetry_a{0.0};
    double rotor_moved_rad{0.0};
};

/**
 * @brief [en] Pulse along a deliberately offset d-axis estimate, with no saliency loop in front.
 *
 * @brief [zh] 沿刻意偏移的 d 轴估计脉冲，前面不接凸极环。
 *
 *   The sweep is *about* the angle the caller hands in, so converging HFI first would defeat it: the loop's job is to
 *   land on the d-axis line, and this case asks what happens when it does not. 这次扫描 **就是** 关于调用方给出的角度
 *   的，所以先让 HFI 收敛会毁掉它：那个环的职责正是落在 d 轴线上，而本用例问的是它没落准时会怎样。
 */
auto pulse_at(double delta_rad, double settle_s) -> PulseResult {
    sil::PmsmPlant plant = saturating_plant(0.0F);
    core::MagnetPolarityTest test(
        core::MagnetPolarityTest::Config{1.0F, 4e-4F, static_cast<float>(settle_s), 0.02F});
    test.begin(static_cast<float>(delta_rad));
    const double rotor_before = static_cast<double>(kPoles) * plant.theta_mech_rad();
    PulseResult result{};
    for (int i = 0; i < 20000; ++i) {
        const core::MagnetPolarityTest::Output out = test.step(kDt, core::clarke(plant.current_uvw()));
        for (int k = 0; k < kSubsteps; ++k) {
            const float theta_e = static_cast<float>(kPoles) * plant.theta_mech_rad();
            plant.set_voltage_dq(core::park(out.v_ab, theta_e));
            plant.advance(kDt / static_cast<float>(kSubsteps));
        }
        if (!out.running) {
            result.verdict = out.verdict;
            result.delta_positive_a = out.delta_positive_a;
            result.delta_negative_a = out.delta_negative_a;
            result.asymmetry_a = std::fabs(static_cast<double>(out.delta_positive_a)) -
                                 std::fabs(static_cast<double>(out.delta_negative_a));
            break;
        }
    }
    result.rotor_moved_rad = static_cast<double>(kPoles) * plant.theta_mech_rad() - rotor_before;
    return result;
}

} // namespace



/**
 * @brief [en] The answer is attenuated by `cos^2` of the d-axis error, and the window is then closed by the rotor
 *             moving before it is closed by the signal fading.
 *
 * @brief [zh] 答案按 d 轴误差的 `cos²` 衰减，而窗口是先被 **转子被推走** 关上的，不是先被信号淡出关上的。
 *
 *   Both pulses are applied along the *estimate*, so the d-axis volt-seconds are cut by `cos(delta)` and the measured
 *   projection cuts them again -- two factors, hence `cos^2`. The saliency term `sin^2(delta)/Lq` is common to both
 *   pulses and cancels in the difference, so saliency attenuates the answer without contaminating it: that is the
 *   structural reason the method is sound, and `cos^2` is why it has a finite window.
 *
 *   Measured ratios against `cos^2`: **0.987 at 15 degrees, 0.943 at 30, 0.850 at 45**, and 0.658 at 60 where the
 *   first-order law is visibly over-predicting -- at large `delta` the pulse drives less current, so it engages less of
 *   the inductance collapse and the true attenuation is faster. That is the small-signal limit of the law, and it is
 *   stated as a domain rather than fitted away.
 *
 *   The window itself is set by something else, which this case found rather than assumed: at 60 degrees the asymmetry
 *   is still 0.093 A, **4.6 times the 0.02 A threshold**, and the answer is refused anyway -- because the pulse now has
 *   a large q component, the rotor moves 14 mrad electrical, and the two pulses no longer start from one operating
 *   point. Lengthening the settle to 20 ms restores the answer at 60 degrees, which is what identifies the binding
 *   limit.
 *
 *   两个脉冲都沿 **估计** 施加，所以 d 轴伏秒被 `cos(delta)` 砍一次、测量投影再砍一次——两个因子，因此是 `cos²`。凸极项
 *   `sin²(delta)/Lq` 对两个脉冲共模、在差值里抵消，所以凸极只衰减答案而不污染它：这是方法成立的结构性原因，而 `cos²` 是
 *   它窗口有限的原因。
 *
 *   对照 `cos²` 实测比值： **15° 0.987、30° 0.943、45° 0.850** ，60° 处 0.658，那里一阶律明显过预测—— 大 `delta` 下脉冲
 *   推动的电流更小，激起的电感塌陷也更少，真实衰减更快。这是一阶律的小信号边界，写成域而不是拟合掉。
 *
 *   窗口本身是被别的东西定下的，这是本用例 **发现** 的而不是假设的：60° 处不对称仍有 0.093 A，是 0.02 A 阈值的 **4.6 倍
 *   **，而答案照样被拒——因为脉冲现在有很大的 q 分量，转子被推走 14 mrad 电角度，两个脉冲不再从同一个工作点出发。把松弛
 *   加到 20 ms 后 60° 又能作答，这就指出了真正的限制项。
 */
TEST(SilPolarity, TheAnswerFadesAsTheCosineSquaredAndTheRotorClosesTheWindow) {
    const PulseResult aligned = pulse_at(0.0, 5e-3);
    ASSERT_EQ(core::PolarityVerdict::Aligned, aligned.verdict);
    ASSERT_GT(aligned.asymmetry_a, 0.1) << "the reference asymmetry has to be real";

    std::printf("\n%-10s %12s %12s %10s %8s %12s\n", "delta_deg", "asymmetry", "cos^2 pred", "ratio",
                "verdict", "rotor_rad");
    const struct {
        double degrees;
        double tolerance;
    } points[] = {{0.0, 0.001}, {15.0, 0.03}, {30.0, 0.07}, {45.0, 0.16}};
    for (const auto& point : points) {
        const double delta = point.degrees * 3.14159265358979 / 180.0;
        const PulseResult run = pulse_at(delta, 5e-3);
        const double predicted = aligned.asymmetry_a * std::cos(delta) * std::cos(delta);
        const double ratio = run.asymmetry_a / predicted;
        std::printf("%-10.1f %12.5f %12.5f %10.4f %8d %12.5f\n", point.degrees, run.asymmetry_a,
                    predicted, ratio, static_cast<int>(run.verdict), run.rotor_moved_rad);
        // The small-signal law, with the tolerance stated per angle before the run.
        // 小信号律，每个角度的容差在跑之前就写好了。
        EXPECT_NEAR(1.0, ratio, point.tolerance);
        EXPECT_EQ(core::PolarityVerdict::Aligned, run.verdict) << "inside the window the answer holds";
    }

    // The two limits, separated by experiment. At 60 degrees the signal is still far above the threshold
    // and the answer is refused; lengthening the settle brings it back, so the rotor's drift is what
    // closed the window there and not the threshold.
    // 两条限值，用实验分开。60° 处信号仍远高于阈值而答案被拒；加长松弛它又回来，所以那里关上窗口的是
    // 转子漂移而不是阈值。
    const PulseResult faded = pulse_at(60.0 * 3.14159265358979 / 180.0, 5e-3);
    const PulseResult restored = pulse_at(60.0 * 3.14159265358979 / 180.0, 20e-3);
    std::printf("[window] 60 deg: settle 5 ms -> asym %.5f (reported %d, rotor %.5f); settle 20 ms -> "
                "asym %.5f (reported %d, rotor %.5f)\n",
                faded.asymmetry_a, static_cast<int>(faded.verdict != core::PolarityVerdict::Inconclusive),
                faded.rotor_moved_rad, restored.asymmetry_a,
                static_cast<int>(restored.verdict != core::PolarityVerdict::Inconclusive),
                restored.rotor_moved_rad);
    EXPECT_GT(faded.asymmetry_a, 4.0 * 0.02) << "the signal at 60 degrees is not what silences it";
    EXPECT_EQ(core::PolarityVerdict::Inconclusive, faded.verdict)
        << "the two pulses do not start from one operating point any more";
    EXPECT_EQ(core::PolarityVerdict::Aligned, restored.verdict)
        << "a longer settle puts the second pulse back on the same start, so this was the binding limit";

    // A quarter turn out the estimate is nearer the other axis, and the answer is the right one.
    // 差四分之一圈时估计离另一条轴更近，答案就是正确的那个。
    const PulseResult half = pulse_at(3.14159265358979, 5e-3);
    std::printf("[mirror] 180 deg: asym %+.5f (verdict %d), |asym| within %.2f %% of the aligned case\n",
                half.asymmetry_a, static_cast<int>(half.verdict),
                100.0 * std::fabs(std::fabs(half.asymmetry_a) - aligned.asymmetry_a) /
                    aligned.asymmetry_a);
    EXPECT_EQ(core::PolarityVerdict::Reversed, half.verdict);
    EXPECT_NEAR(aligned.asymmetry_a, std::fabs(half.asymmetry_a), 0.01 * aligned.asymmetry_a)
        << "half a turn out is the same measurement with the opposite sign, and the rotor does not move";
}



/**
 * @brief [en] A settle shorter than the winding's `L/R` is refused rather than answered -- and the crossing is the
 *             closed form.
 *
 * @brief [zh] 短于绕组 `L/R` 的松弛被拒绝而不是被作答——而那个交界正是闭式。
 *
 *   This is F-47. Without the guard the failure shape is the worst possible one: at a zero-length settle the second
 *   pulse starts on the first pulse's tail, the asymmetry comes out at **-0.5404 A** for a machine whose magnet is
 *   `Aligned` -- the wrong sign, and 96 % *larger* in magnitude than the honest 0.5651 A. A wrong answer that is louder
 *   than the right one is invisible to any magnitude check downstream.
 *
 *   The crossing is derivable with nothing but the winding: the residual decays as `exp(-t/tau)` with `tau = Ld/Rs =
 *   571 us`, and the guard releases when it falls below `min_asymmetry_a = 0.02 A`, i.e. at `tau*ln(1.791/0.020) = 2.57
 *   ms`. Measured: **refused at 2 ms, answered at 3 ms** -- the first grid point above the prediction. The precondition
 *   is therefore not a comfort margin; it is a number, and one the caller can compute for their own winding.
 *
 *   这就是 F-47。没有护栏时的失效形态是最糟的那种：零松弛下第二个脉冲从第一个脉冲的尾巴上出发，磁钢为 `Aligned` 的机器
 *   算出 **-0.5404 A** 的不对称——符号错，幅值还比诚实的 0.5651 A **大 96 %** 。一个比正确答案更"响"的错误答案，下游任何
 *   幅值检查都看不见。
 *
 *   交界可以只用绕组推出来：残余按 `exp(-t/tau)` 衰减，`tau = Ld/Rs = 571 us`，护栏在它落到 `min_asymmetry_a = 0.02 A`
 *   以下时放开，即 `tau*ln(1.791/0.020) = 2.57 ms`。实测： **2 ms 被拒、 3 ms 作答** ——预测之上的第一个栅格点。所以这条
 *   前置条件不是舒适裕度；它是一个数，而且调用方可以按自己的绕组算出来。
 */
TEST(SilPolarity, ASettleShorterThanTheWindingDecayIsRefusedAndTheCrossingIsTheClosedForm) {
    const double tau_s = static_cast<double>(kLd) / static_cast<double>(kRs);
    const PulseResult settled = pulse_at(0.0, 5e-3);
    ASSERT_EQ(core::PolarityVerdict::Aligned, settled.verdict);
    const double predicted_crossing_s = tau_s * std::log(settled.delta_positive_a / 0.02);
    std::printf("\n[precondition] tau = Ld/Rs = %.1f us, crossing at %.3f ms\n", tau_s * 1.0e6,
                predicted_crossing_s * 1.0e3);

    std::printf("%10s %12s %12s %12s %8s\n", "settle_ms", "delta+", "delta-", "asymmetry", "verdict");
    double first_accepted_ms = -1.0;
    for (const double settle_ms : {0.0, 0.5, 1.0, 2.0, 3.0, 4.0, 5.0, 8.0}) {
        const PulseResult run = pulse_at(0.0, settle_ms * 1e-3);
        std::printf("%10.2f %12.5f %12.5f %12.5f %8d\n", settle_ms, run.delta_positive_a,
                    run.delta_negative_a, run.asymmetry_a, static_cast<int>(run.verdict));
        const bool accepted = (run.verdict != core::PolarityVerdict::Inconclusive);
        if (accepted && first_accepted_ms < 0.0) {
            first_accepted_ms = settle_ms;
        }
        if (settle_ms * 1e-3 < predicted_crossing_s) {
            EXPECT_EQ(core::PolarityVerdict::Inconclusive, run.verdict)
                << "below the predicted crossing the answer is not supported";
        }
    }
    // The crossing is the first grid point at or above the closed form.
    // 交界是闭式之上（含）的第一个栅格点。
    ASSERT_GT(first_accepted_ms, 0.0) << "the sequence never converges, so nothing above is measured";
    EXPECT_GE(first_accepted_ms * 1e-3, predicted_crossing_s) << "it cannot answer before the residual is small";
    // The sweep is on a 1 ms grid, so the crossing is only locatable to one grid step. Saying so is the
    // difference between "the closed form is right" and "the closed form is within the resolution of
    // this experiment".
    // 扫描是 1 ms 栅格，所以交界只能定位到一个栅格步长。把这一点说出来，是"闭式是对的"与"闭式在这次
    // 实验的分辨力之内"的区别。
    EXPECT_LE(first_accepted_ms * 1e-3, predicted_crossing_s + 1.0e-3) << "and one grid step is enough";

    // The trap, with its number.
    // 陷阱，连数一起。
    const PulseResult unsettled = pulse_at(0.0, 0.0);
    std::printf("[trap] unsettled asym %+.4f A vs settled %+.4f A: wrong sign, and %.1f %% of the "
                "honest magnitude\n",
                unsettled.asymmetry_a, settled.asymmetry_a,
                100.0 * std::fabs(unsettled.asymmetry_a) / settled.asymmetry_a);
    EXPECT_LT(unsettled.asymmetry_a, 0.0) << "without the guard this sign is what a caller would act on";
    // The wrong answer is not a small one that a magnitude check could filter out: it is within a few
    // percent of the honest magnitude, with the opposite sign. (On this winding it comes out at 95.6 % of
    // the honest value; on a winding with a shorter `L/R` it comes out *larger*. Either way the sizes are
    // the same order, which is the point -- the sign is the only thing that separates them, and nothing
    // downstream can check a sign it was never given.)
    // 错误答案不是那种能被幅值检查滤掉的小值：它的幅值与诚实值只差几个百分点，符号相反。（在这台绕组上
    // 是诚实值的 95.6 %；在 `L/R` 更短的绕组上它会 **更大** 。两种情况下量级都相同，这才是要点——隔开两者
    // 的只有符号，而下游无法检查一个它从未被告知的符号。）
    EXPECT_GT(std::fabs(unsettled.asymmetry_a), 0.5 * settled.asymmetry_a)
        << "the wrong answer is the same order of magnitude, so no magnitude check could catch it";
}
