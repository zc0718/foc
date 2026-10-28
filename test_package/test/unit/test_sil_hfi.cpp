// S14 - high-frequency injection: the angle at standstill, where back-EMF has nothing to say.
// S14 - 高频注入：静止时的角度，正是反电动势无话可说之处。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/hfi_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/traits.hpp>
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

// A salient machine: Lq is twice Ld, which is what gives the injection something to measure. An SPM
// (Ld == Lq) has no signal at all -- S14 could not exist for the nominal machine of S2..S12.
// 凸极电机：Lq 是 Ld 的两倍，这才让注入有东西可测。表贴式（Ld == Lq）根本没有信号——对 S2..S12 的
// 标称电机，S14 不可能存在。
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

auto plant_params() -> sil::PlantParams { return sil::PlantParams{kRs, kLd, kLq, kKt, kJ, kB, kPoles}; }

/// [en] The same machine with the rotor held still. A standstill measurement has to keep the angle
///      it measures fixed: the injected current carries a reluctance-torque ripple with a DC
///      component, so on a free rotor the quantity under test moves while it is being measured --
///      which is exactly the mistake that made an earlier ad-hoc probe of this feature look like a
///      frequency limit that does not exist.
///      [zh] 同一台电机，但转子固定。静止测量必须让被测角度真的不动：注入电流的磁阻转矩纹波含直流
///      分量，自由转子会在测量过程中把被测的量本身改掉——这正是此前一次临时探测把“公式误差”误读成
///      “频率限制”的原因。
auto frozen_plant_params() -> sil::PlantParams { return sil::PlantParams{kRs, kLd, kLq, kKt, 1.0e6F, 0.0F, kPoles}; }

/// [en] Wraps to (-pi, pi], the interval an angle error lives in.
/// [zh] 归一化到 (-π, π]，即角度误差所在的区间。
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
 * @brief [en] Config used by every case here; only the fields a case varies are arguments.
 *        [zh] 此处所有用例共用的配置；只有用例要变的字段才作为参数。
 */
auto make_config(float inject_hz, float inject_v, float pll_bandwidth_hz, float gain_scale) -> core::HfiObserver::Config {
    core::HfiObserver::Config config{};
    config.inject_v = inject_v;
    config.inject_hz = inject_hz;
    // Above any fundamental this case can produce and well below the carrier: the high-pass is what
    // keeps a DC current from leaking through the multiplier as a standing bias.
    // 高于本用例可能产生的任何基频、远低于载波：正是这个高通让直流电流不会经乘法器漏成固定偏置。
    config.current_hp_hz = 300.0F;
    // Fixed at 200 Hz for every case: the demodulation filter is *not* what a case is allowed to
    // vary, otherwise a failure could always be blamed on the filter instead of on the physics.
    // 所有用例都固定 200 Hz：解调滤波器不是用例可以变的东西，否则任何失败都能怪到滤波器而不是物理上。
    config.demod_lpf_hz = 200.0F;
    config.pll_bandwidth_hz = pll_bandwidth_hz;
    config.pll_zeta = 0.9F;
    config.demod_gain_a = gain_scale * core::HfiObserver::predicted_gain_a(inject_v, inject_hz, kLd, kLq);
    config.min_hf_current_a = 0.05F;
    return config;
}

/**
 * @brief [en] Standstill rig: the plant is held at a known angle and fed nothing but the injection.
 *        [zh] 静止台架：plant 停在已知角度，除注入外不加任何电压。
 *
 * @note [en] The call order mirrors `Motor::tick` exactly -- update(), then ask for the injection,
 *       then apply it -- because an off-by-one period in the phase is invisible otherwise.
 *       [zh] 调用顺序与 `Motor::tick` 完全一致——先 update()，再取注入量，再施加——否则相位上差一个
 *       周期的错误是看不出来的。
 */
class StandstillRig final {
  public:
    StandstillRig(float theta_e_true, core::HfiObserver::Config config, bool frozen) noexcept
        : plant_(frozen ? frozen_plant_params() : plant_params()), observer_(config) {
        plant_.reset(sil::PlantState{0.0F, 0.0F, 0.0F, theta_e_true / static_cast<float>(kPoles)});
    }

    StandstillRig(const StandstillRig&) = delete;
    StandstillRig& operator=(const StandstillRig&) = delete;
    StandstillRig(StandstillRig&&) = delete;
    StandstillRig& operator=(StandstillRig&&) = delete;

    void step() noexcept {
        const core::Ab<float> i_ab = core::clarke(plant_.current_uvw());
        last_ = observer_.update(core::Ab<float>{0.0F, 0.0F}, i_ab, kDt);
        const core::Ab<float> v_ab = observer_.injection_ab();
        for (int i = 0; i < kSubsteps; ++i) {
            const float theta_e = static_cast<float>(kPoles) * plant_.theta_mech_rad();
            plant_.set_voltage_dq(core::park(v_ab, theta_e));
            plant_.advance(kDt / static_cast<float>(kSubsteps));
        }
    }

    void run_for(float seconds) noexcept {
        const int steps = static_cast<int>((seconds / kDt) + 0.5F);
        for (int i = 0; i < steps; ++i) {
            step();
        }
    }

    [[nodiscard]] auto true_angle() const noexcept -> float {
        return static_cast<float>(kPoles) * plant_.theta_mech_rad();
    }

    [[nodiscard]] auto estimate() const noexcept -> float { return last_.theta_e; }

    [[nodiscard]] auto error_rad() const noexcept -> float { return wrap_pi_local(last_.theta_e - true_angle()); }

    [[nodiscard]] auto observer() noexcept -> core::HfiObserver& { return observer_; }

    [[nodiscard]] auto plant() noexcept -> sil::PmsmPlant& { return plant_; }

  private:
    sil::PmsmPlant plant_;
    core::HfiObserver observer_;
    core::ObsState last_{0.0F, 0.0F};
};

} // namespace

/// [en] Open-loop error signal at a given angle error: rotor held still and the loop pinned by a
///      0.05 Hz bandwidth, so what is read is the signal and not the loop's response to itself.
///      [zh] 给定角度误差下的开环误差信号：转子固定，环路被 0.05 Hz 带宽钉住，于是读到的是信号本身
///      而不是环路对自身的响应。
auto open_loop_error(float hz, float dtheta) -> float {
    StandstillRig rig(-dtheta, make_config(hz, 12.0F, 0.05F, 1.0F), true);
    rig.run_for(0.10F);
    return rig.observer().error();
}

TEST(SilHfi, ItFindsTheRotorFromAWrongStart) {
    // The estimate starts at 0 while the rotor sits half a radian away in either direction: no
    // information about the rotor exists anywhere except in the current response.
    // 估计值从 0 开始，而转子就在任一方向的半个弧度外：除电流响应外，系统中没有任何关于转子的信息。
    for (const float start : {0.5F, -0.5F}) {
        StandstillRig rig(start, make_config(2000.0F, 12.0F, 40.0F, 1.0F), false);
        rig.run_for(1.0F);
        EXPECT_NEAR(0.0F, rig.error_rad(), 0.02F) << "start " << start;
        EXPECT_TRUE(rig.observer().locked());
    }
}

TEST(SilHfi, TheErrorSignalIsProportionalToSineOfTwiceTheErrorAngle) {
    // The whole method rests on `err ~= -sin(2*dtheta)`. A loop tuned for a linear error and fed a
    // signal that is not one converges to a biased point, so the shape has to be checked, not assumed.
    // 整个方法建立在 `err ≈ -sin(2·dtheta)` 上。为线性误差整定的环路若吃到非线性的信号，会收敛到
    // 有偏的位置，因此这个形状必须被检查，而不是被假设。
    float lowest = 1.0e9F;
    float highest = -1.0e9F;
    float ratio_sum = 0.0F;
    int count = 0;
    for (const float dtheta : {0.02F, 0.05F, 0.10F, 0.20F, 0.30F, 0.50F}) {
        const float ideal = -std::sin(2.0F * dtheta);
        const float ratio = open_loop_error(2000.0F, dtheta) / ideal;
        EXPECT_GT(ratio, 0.0F) << "sign flipped at dtheta " << dtheta;
        lowest = std::fmin(lowest, ratio);
        highest = std::fmax(highest, ratio);
        ratio_sum += ratio;
        ++count;
    }
    const float mean = ratio_sum / static_cast<float>(count);

    // Proportional across a 25x range of error angles: the sensitivity must not depend on how far
    // off the estimate is, which is what "the error signal is sin(2*dtheta)" means operationally.
    // 在 25 倍的角度误差范围内保持正比：灵敏度不得随误差大小变化，这正是“误差信号是 sin(2·dtheta)”
    // 在实际使用上的含义。
    EXPECT_LT((highest - lowest) / mean, 0.20F) << "lowest " << lowest << " highest " << highest;
    // And the sensitivity lands near the first-order prediction (measured mean: 0.81).
    // 灵敏度落在十阶预测附近（实测均值 0.81）。
    EXPECT_NEAR(1.0F, mean, 0.25F);
}

TEST(SilHfi, TheFirstOrderFormulaDegradesAsTheInductiveReactanceFalls) {
    // `predicted_gain_a` ignores Rs. With wh*Ld = 2.5 ohm against Rs = 0.35 the prediction is close;
    // with wh*Ld = 0.63 it is half. The signal stays *proportional* either way -- that is the fact
    // this case exists to pin down: a wrong sensitivity is a loop-gain error, not a broken method.
    // 这里要钉住的事实：`predicted_gain_a` 忽略 Rs。wh·Ld = 2.5 Ω 对 Rs = 0.35 时预测接近；wh·Ld =
    // 0.63 时只有一半。两种情况下信号都**保持正比**——灵敏度偏差是环路增益误差，不是方法失效。
    const float ideal = -std::sin(0.20F);
    const float at_2k = open_loop_error(2000.0F, 0.10F) / ideal;
    const float at_500 = open_loop_error(500.0F, 0.10F) / ideal;

    EXPECT_GT(at_2k, at_500) << "a lower reactance must not look like a *better* prediction";
    EXPECT_NEAR(0.79F, at_2k, 0.10F);
    EXPECT_NEAR(0.51F, at_500, 0.10F);
}

TEST(SilHfi, OutsideTheHalfTurnWindowItLocksToTheMirror) {
    // `sin(2*dtheta)` has a second stable zero at pi. Starting the estimate two radians away (outside
    // the +-pi/2 window) lands there: the d axis is right to within a factor of two and the magnet is
    // backwards. Nothing in the signal can tell the two apart.
    // `sin(2·dtheta)` 在 π 处有第二个稳定零点。估计值起步偏两弧度（超出 ±π/2 窗口）就会落到那里：d 轴
    // 在“差一半”的意义上是对的，磁钢却是反的。信号本身无法区分这两者。
    StandstillRig rig(2.0F, make_config(2000.0F, 12.0F, 40.0F, 1.0F), false);
    rig.run_for(1.0F);
    EXPECT_NEAR(kPiFloat, std::fabs(rig.error_rad()), 0.05F);
}

TEST(SilHfi, AWrongSaliencyDirectionRunsAwayInsteadOfLocking) {
    // A machine with Ld > Lq needs the opposite sign, and passing the wrong one is not a tuning
    // detail: the loop drives the estimate to the unstable equilibrium at +-pi/2 and stays there.
    // Ld > Lq 的电机需要相反的符号，给错不是个整定细节：环路把估计值推到 ±π/2 的不稳定平衡点并停住。
    StandstillRig rig(0.5F, make_config(2000.0F, 12.0F, 40.0F, -1.0F), false);
    rig.run_for(1.0F);
    EXPECT_NEAR(0.5F * kPiFloat, std::fabs(rig.error_rad()), 0.10F);
}

TEST(SilHfi, WithoutHfCurrentItHoldsTheAngleInsteadOfGuessing) {
    // The lock detector is the only channel through which this observer can say "I do not know".
    // Raise the threshold above the current the injection actually produces and the estimate must
    // stop moving rather than integrate noise into a number that looks measured.
    // 锁定判据是这个观测器能表达“我不知道”的唯一通道。把阈值抬到注入实际产生的电流之上，估计值必须
    // 停下，而不是把噪声积成一个看起来像是测量值的数字。
    core::HfiObserver::Config config = make_config(2000.0F, 12.0F, 40.0F, 1.0F);
    config.min_hf_current_a = 100.0F;
    StandstillRig rig(0.5F, config, false);
    rig.run_for(0.5F);
    EXPECT_FALSE(rig.observer().locked());
    EXPECT_NEAR(0.0F, rig.estimate(), 1e-3F);
    EXPECT_FLOAT_EQ(0.0F, rig.observer().error());
}

namespace {

/// [en] Sensorless standstill chain: the shipped `Motor`, the SIL plant pretending there is no
///      encoder, and HFI as the only source of angle.
///      [zh] 无感静止链路：出厂 `Motor` + 假装没有编码器的 SIL plant + 唯一角度来源 HFI。
using HfiMotor = foc::motor::Motor<core::HfiObserver, core::Svpwm, core::Pi, sil::PlantHal>;

auto hfi_motor_config() -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{kRs, kLd,   kLq,  kKt,  kJ, kB, kPoles, 3.0F, 20000.0F, 0.0F, 0.0F, 0.0F};
}

auto hfi_policies(float inject_v) -> HfiMotor::Policies {
    const core::HfiObserver::Config hfi = make_config(2000.0F, inject_v, 40.0F, 1.0F);
    const foc::motor::MotorConfig mc = hfi_motor_config();
    return HfiMotor::Policies{core::HfiObserver{hfi}, core::Svpwm{}, core::Pi{foc::current_pi_config(mc, 2000.0F, 24.0F)},
                              core::Pi{foc::current_pi_config(mc, 2000.0F, 24.0F)},
                              core::Pi{foc::speed_pi_config(mc, 250.0F)}};
}

} // namespace

TEST(SilHfi, TheShippedMotorRunsSensorlessAtStandstill) {
    // End to end: `Motor` cannot tell this observer apart from a back-EMF one, its injection lands on
    // the voltage command, and the angle it produces is good enough to regulate current with.
    // 端到端：`Motor` 无法把这个观测器与反电动势观测器区别对待，它的注入真的落到电压指令上，产出的
    // 角度好到可以用于电流调节。
    sil::PmsmPlant plant(plant_params());
    plant.reset(sil::PlantState{0.0F, 0.0F, 0.0F, 0.5F / static_cast<float>(kPoles)});
    sil::PlantHal hal(plant, 24.0F);
    hal.position.set_sensor_present(false);

    // 2 V of injection: the HF current it draws (V/(w_h*Ld) = 0.8 A) has to fit inside the 3 A limit
    // the current loop is allowed to use, otherwise the carrier is paid for out of the torque budget
    // the application asked for.
    // 2 V 注入：它拉出的高频电流（V/(w_h·Ld) = 0.8 A）必须落在电流环允许使用的 3 A 之内，否则载波就是
    // 从应用要的转矩预算里扣的。
    HfiMotor motor(hfi_motor_config(), hfi_policies(2.0F), hal);

    ASSERT_EQ(foc::ErrorCode::Ok, motor.init());
    ASSERT_EQ(foc::ErrorCode::Ok, motor.start());
    motor.set_torque(0.0F);

    const int steps = static_cast<int>(1.0F / kDt);
    for (int i = 0; i < steps; ++i) {
        ASSERT_EQ(foc::ErrorCode::Ok, motor.tick(kDt));
        hal.advance(kDt, kSubsteps);
    }

    const float theta_true = static_cast<float>(kPoles) * hal.position.true_angle_mech_rad();
    EXPECT_TRUE(motor.observer().locked());

    // The estimate is compared against the sensor it never read.
    // 估计值与它从未读过的传感器比对。
    const float theta_est = motor.observer().state().theta_e;
    std::printf("[S14] chain: true %.4f rad, estimate %.4f rad\n", static_cast<double>(theta_true),
                static_cast<double>(theta_est));
    EXPECT_NEAR(0.0F, wrap_pi_local(theta_est - theta_true), 0.10F);
}
