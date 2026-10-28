// W7-c, increment 2: where does the low-speed speed ripple come from?
//
// Increment 1 ended on a number that re-aimed the wave. The low-speed servo's settled figure at 20 rad/s mech is
// 0.1717 rad/s worst, but splitting it apart gives a steady bias of 0.0039 rad/s (0.02 %) and a *ripple* of 0.0900. So
// nothing about the estimator, the angle or the demodulation window is left on the table -- what is left is a ripple.
// The question this file answers is whose ripple it is, because the answer decides whether W7 continues at all.
//
// There are exactly two candidates and they are separable:
//
//   mechanical  the q-axis current ripples, so the torque ripples, so the speed ripples by `T / (J * omega_ripple)`.
//               With J = 1e-5 this is brutal: a few milliamps of current is half a percent of speed. Nothing in the
//               library can remove it, and an application with a real rotor would not see it in the first place.
//   control     the speed loop, the current loop or the observer oscillates at its own frequency, in which case the
//               ripple would not care how heavy the rotor is and the library is at fault.
//
// The separator is the inertia: the mechanical story predicts the ripple scales as `1/J` and the control story predicts
// it does not move at all. One knob, two outcomes, both informative.
//
// W7-c 增量 2：低速的转速纹波到底是谁的？
//
// 增量 1 收在一个让整波转向的数字上。20 rad/s 机械下低速伺服的稳定值是 0.1717 rad/s，但拆开来看：稳态偏差 0.0039
// rad/s（0.02%），**纹波** 0.0900。于是估计器、角度、解调窗口都没有可做的东西了——剩下的是一个纹波。本文件回答的是
// 这个纹波属于谁，因为答案决定 W7 还要不要继续。
//
// 只有两个候选，而且可以分开：
//
//   mechanical  q 轴电流在纹，于是转矩在纹，于是转速按 `T / (J·omega_ripple)` 在纹。J = 1e-5 时这很残酷：几毫安电流
//               就是半个百分点的转速。库里的任何东西都去不掉它，而有真实转子的应用根本看不到它。
//   control     速度环、电流环或观测器在自己的频率上振荡；那样的话纹波不会在乎转子有多重，而库有问题。
//
// 分界是惯量：机械说纹波按 `1/J` 缩放，控制说它一点也不动。一个旋钮，两种结果，两种都有信息。
//
// The criteria, fixed before the numbers were measured / 判据在量之前定下：
//
//   C1  (is the figure even real) the ripple measured at substep resolution must agree with the ripple measured at the
//       control rate to within 20 %. The control-rate figure samples the speed every 100 us, so anything faster than
//       10 kHz appears folded down to a lower frequency, and the 415 Hz reported last increment would then be an alias
//       of the current-loop or carrier beat rather than a real tone. If they disagree, the frequency and the amplitude
//       both have to be re-measured before either is quoted again.
//   C2  (whose ripple) with the inertia raised tenfold the speed ripple must fall by at least 5x, i.e. it must follow
//       `1/J` to within a factor of two. Falling as `1/J` means mechanical and closes the search; not falling means the
//       loop is oscillating on its own and the next increment is inside the library.
//   C3  (does the chain close) the torque ripple implied by the measured current ripple, propagated through the
//       mechanical equation at the measured dominant frequency, must land within a factor of two of the measured speed
//       ripple. This is the check that the current is the *cause* and not a coincidence with it.
//
//   C1  （这个数到底真不真）子步分辨率下量到的纹波必须与控制率下量到的相差不超过 20%。控制率是每 100 us 采一次转速，
//       于是快于 10 kHz 的东西都会折叠成更低的频率，上一增量报的 415 Hz 就可能是电流环或载波拍频的混叠而不是真实音调。
//       若两者不符，频率与幅值都得重新量，在那之前一个都不能引用。
//   C2  （谁的纹波）惯量提高十倍，转速纹波必须至少降到五分之一，即跟随 `1/J` 到两倍以内。按 `1/J` 下降即机械必然、就此
//       收口；不下降即环路在自己振荡，下一个增量在库里面。
//   C3  （链条闭不闭）由实测电流纹波经机械方程、在实测主导频率上推算出的转矩纹波，必须落在实测转速纹波的两倍之内。
//       这条检查的是电流确实是**原因**，而不是与它巧合同步。
//   C4  （是不是观测器）带传感器那一跑的纹波不超过无感那跑的五分之一，则观测器在通路上。**这条结论后来被撤回了**：
//       那一臂把注入连同观测器一起去掉了，两臂差的是两件事。详见用例正文。
//   C5  （是不是角度）角度纹波的主导频率与电流纹波的一致，且由角度纹波推出的电流落在实测的三倍之内。**否证**：角度在
//       该频率上没有音调，预测偏差 4.97 倍。
//   C6  （是不是注入幅值）伏特降到三分之一，转速纹波至多降到 60%。**否证**：电流纹波确实减半了（0.54），转速纹波几乎不动
//       （0.98），而音调从 418 Hz 挪到 92 Hz。
//   C7  （是不是在追自己的测量）已发布转速的纹波至少是真实转速的 3 倍、且在同一音调上。**一半成立**：倍数 4.48，但音调
//       在 0 Hz——缓慢游走而非 418 Hz 振荡。
//
//   C8  （是不是钳位造成的继电振荡）误差至少十分之一的窗口撞在 ±1 钳位上。**否证**：两种幅值下占用率都是 0.000，环路在这里
//       是线性的；418/92 Hz 的频率移动是过零计数器读多音调信号的产物。
//   C9  （哪个频率拥有它）六个栅格对齐的候选里最大者要比中位数大 1.5 倍。**成立**：2.0000 Hz 就是载波，9.41 倍。
//
//   C10a 已发布转速上的载波音调在选定截止频率上至少降 5 倍。**成立**：200 Hz 下 10.1 倍。
//   C10b 闭环转速纹波随之至少降 2 倍。**否证**：0.0900 → 0.0879，几乎没有变化——那条音调与测量量相关，但它没有一条穿过环路的
//       通路。已发布转速变干净是"应用读到的数"的改善，不是轴纹波的修法。
//   C10c 稳态偏差不得因滤波变大。**成立**：0.0039 → 0.0023，反而更小（判据初稿写成对称区间，已改为单边并注明）。
//
//   C11 载波频率上的转速音调线性于 `inject_v`。**成立**：三分之一的伏特剩 0.36（预测 0.33）。
//   C12 同样伏特下凸极减半，音调随之减半。**否证**：剩 0.97——转矩纹波不是凸极积 `(Ld−Lq)·id·iq`。（第一版的旋钮把伏特
//       随 `Lq` 缩放来"保持高频电流"，但 d 轴高频电流由 `Ld` 决定，那个旋钮同时动了两个量，什么也没测到。）
//   C13 实测转矩音调经 `1/(J·ω)` 推算的转速音调落在实测的 0.7–1.4 倍内。**成立**：1.03 倍。
//   C14 转矩音调除以角度误差，在 20 与 5 rad/s 之间一致。**成立**：0.8035 对 0.8016，1.00 倍。
//
// 于是本文件结束时的事实：纹波 0.09 rad/s，只要注入在跑就在，注入一停就是 0.0000；它不在转子惯量里、不在已发布角度里、
// 不在注入幅值里、不是钳位造成的振荡。**它最大的单条音调是载波本身**，而观测器发布出来的转速带着这条音调的力度是转子的
// 10.4 倍——环路不是在自己生成它，而是在被喂它：`omega_e = omega_int + kp·error`，而 `error` 按构造携带载波频率的成分，
// 在速度环闭上去之前没有任何东西滤过它。

#include <algorithm>
#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/switchable_observer.hpp>
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

constexpr float kRs = 0.35F;
constexpr float kLd = 2.0e-4F;
constexpr float kLq = 4.0e-4F;
constexpr float kKt = 0.05F;
constexpr float kB = 1.0e-3F;
constexpr std::uint8_t kPoles = 7U;

// The inertia the rest of the wave has used. It is 1e-5 kg*m^2, which for this machine is a small rotor, and that is
// exactly why the number under test is small enough to be dominated by ripple.
// 本波其余部分一直用的惯量。1e-5 kg·m² 对这台机器是小转子，这也正是被测数字小到被纹波主导的原因。
constexpr float kBaseJ = 1.0e-5F;

constexpr float kCtrlHz = 20000.0F;
constexpr float kDt = 1.0F / kCtrlHz;
constexpr int kSubsteps = 8;
constexpr float kVBus = 24.0F;
constexpr float kInjectV = 12.0F;
constexpr float kInjectHz = 2000.0F;
constexpr float kDemodHz = 200.0F;

constexpr float kPiLocal = 3.14159265358979F;
constexpr float kTwoPiLocal = 6.28318530717959F;

constexpr float kCommandRadS = 20.0F;
// The measurement window in samples: the Goertzel grid is `control_hz / this`, so the two must be one number.
// 测量窗口的样本数：Goertzel 的栅格是 `control_hz / 这个数`，两者必须是同一个数。
constexpr int kWindowSamples = static_cast<int>(0.3F * kCtrlHz);
constexpr float kSettleSeconds = 0.3F;
constexpr float kRunSeconds = 1.2F;
constexpr float kWindowSeconds = 0.3F;

auto plant_params(float inertia, float lq) -> sil::PlantParams {
    return sil::PlantParams{kRs, kLd, lq, kKt, inertia, kB, kPoles};
}

auto motor_config(float inertia, float lq) -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{kRs, kLd, lq, kKt, inertia, kB, kPoles, 3.0F, kCtrlHz, 0.0F, 0.0F, 0.0F};
}

using SensorlessMotor = foc::motor::Motor<core::SwitchableObserver, core::Svpwm, core::Pi, sil::PlantHal,
                                         core::NullReferenceGenerator, core::NullFeedforward, core::PositionP,
                                         core::NullAlignment>;

/**
 * @brief [en] Six candidate tones, each of which names a mechanism, chosen on the window's own bin grid.
 *
 * @brief [zh] 六个候选音调，每一个都对应一个有名有姓的机理，且都落在窗口自己的频率栅格上。
 *
 *     The grid is `control_hz / window_samples` = 3.333 Hz, and every candidate is an exact multiple of it, so the
 *     Goertzel below reads each one with no leakage from its neighbours. That is not pedantry: a candidate half a bin
 *     off would report the *sum* of the leakage from whatever is near it, and the whole point of this table is to
 *     decide which frequency owns the ripple. 栅格是 `control_hz / 窗口样本数` = 3.333 Hz，每个候选都是它的整数倍，于是
 *     下面的 Goertzel 读每一个都不受邻居泄漏影响。这不是学究气：偏半个栅格的候选会把邻近内容的泄漏**加**进来，而这张表
 *     要决定的恰恰是哪个频率拥有这个纹波。
 */
constexpr int kToneBins[6] = {10, 12, 28, 95, 125, 600};
constexpr float kToneHz[6] = {33.33F, 40.0F, 93.33F, 316.67F, 416.67F, 2000.0F};
constexpr const char* kToneOwner[6] = {"speed loop 200 rad/s", "PLL bandwidth 40 Hz",  "the tone seen at 4 V",
                                       "current loop 2000 rad/s", "the tone seen at 12 V", "the injection carrier"};

/**
 * @brief [en] One Goertzel bin over a fixed-length window: amplitude at one frequency, no transform, no allocation.
 * @brief [zh] 定长窗口上的一个 Goertzel 频点：单一频率的幅值，不做变换、不分配。
 */
class Goertzel final {
  public:
    Goertzel() noexcept = default;
    explicit Goertzel(int bin) noexcept : coeff_(static_cast<float>(2.0 * std::cos((2.0 * 3.14159265358979 * static_cast<double>(bin)) / static_cast<double>(kWindowSamples)))) {}

    void push(float x) noexcept {
        const float s = x + (coeff_ * s_prev_) - s_prev2_;
        s_prev2_ = s_prev_;
        s_prev_ = s;
        ++n_;
    }

    [[nodiscard]] auto amplitude() const noexcept -> float {
        if (n_ == 0) {
            return 0.0F;
        }
        const float power = (s_prev_ * s_prev_) + (s_prev2_ * s_prev2_) - (coeff_ * s_prev_ * s_prev2_);
        return (2.0F * std::sqrt(std::max(power, 0.0F))) / static_cast<float>(n_);
    }

  private:
    float coeff_{0.0F};
    float s_prev_{0.0F};
    float s_prev2_{0.0F};
    int n_{0};
};

/** @brief [en] What one closed-loop run measured, bias and ripple kept apart. [zh] 一次闭环运行量到的结果，偏差与纹波分开。 */
struct RippleResult final {
    float speed_bias_rad_s{0.0F};      ///< [en] mean speed error over the window. [zh] 窗口内转速误差的均值。
    float speed_ripple_ctrl{0.0F};     ///< [en] std sampled at the control rate. [zh] 按控制率采样的标准差。
    float speed_ripple_sub{0.0F};      ///< [en] std sampled at every plant substep (160 kHz). [zh] 每个 plant 子步采样（160 kHz）的标准差。
    float iq_mean_a{0.0F};             ///< [en] mean q-axis current in the rotor frame. [zh] 转子系 q 轴电流均值。
    float iq_ripple_a{0.0F};           ///< [en] its std. [zh] 其标准差。
    float torque_ripple_nm{0.0F};      ///< [en] torque ripple implied by the current ripple. [zh] 由电流纹波推出的转矩纹波。
    float dominant_hz{0.0F};           ///< [en] the ripple's dominant frequency, from substep zero crossings. [zh] 纹波主导频率，由子步过零估计。
    float implied_speed_ripple{0.0F};  ///< [en] what that torque ripple would produce at that frequency. [zh] 该转矩纹波在该频率上会产生的转速纹波。
    float angle_mean_rad{0.0F};        ///< [en] mean observer angle error in the window. [zh] 窗口内观测器角度误差均值。
    float angle_std_rad{0.0F};         ///< [en] its spread. [zh] 其离散。
    float angle_ripple_hz{0.0F};       ///< [en] dominant frequency of the angle *ripple* (crossings about a 5 Hz average). [zh] 角度**纹波**的主导频率（相对 5 Hz 均值过零）。
    float iq_ripple_predicted{0.0F};   ///< [en] what that angle ripple should produce in the q current. [zh] 该角度纹波应在 q 轴电流上产生的量。
    float omega_est_mean{0.0F};        ///< [en] mean published speed from the observer. [zh] 观测器发布的转速均值。
    float omega_est_ripple{0.0F};      ///< [en] its spread -- what the loop actually chases. [zh] 其离散——即环路真正追逐的量。
    float omega_est_hz{0.0F};          ///< [en] its dominant tone. [zh] 其主导音调。
    bool reached_command{false};       ///< [en] whether the loop got to the command at all. [zh] 环路到底有没有到达指令。
    float speed_tone[6]{};             ///< [en] true speed amplitude at each candidate tone. [zh] 真实转速在各候选音调上的幅值。
    float omega_tone[6]{};             ///< [en] published speed amplitude at each candidate. [zh] 已发布转速在各候选上的幅值。
    float error_tone[6]{};             ///< [en] the observer's error signal at each candidate. [zh] 观测器误差信号在各候选上的幅值。
    float clamp_fraction{0.0F};        ///< [en] share of window samples with |error()| at the +-1 clamp. [zh] 窗口内 |error()| 撞上 ±1 钳位的样本比例。
    float torque_tone[6]{};            ///< [en] the integrated torque at each candidate tone. [zh] 积分得到的转矩在各候选音调上的幅值。
};

/**
 * @brief [en] The W7-b low-speed servo, instrumented: speed sampled at every substep and the q-axis current in the
 *             rotor frame recorded alongside it, with the inertia as the only structural variable.
 * @brief [zh] W7-b 的低速伺服，加上仪表：转速在每个子步采样，转子系 q 轴电流与之并记，惯量是唯一的结构性变量。
 * @param inertia [en] rotor inertia in kg*m^2; the speed and current loop gains are derived from it, so the loop keeps
 *                     the same shape and the mechanical transfer is the only thing that changes.
 * @param inertia [zh] 转子惯量，kg·m²；速度环与电流环增益都由它导出，于是环的形状不变，变的只有机械传递。
 */
auto run_ripple(float inertia, bool use_sensor, float inject_v, float speed_lpf_hz = 0.0F, float lq = kLq,
                float command = kCommandRadS) -> RippleResult {
    RippleResult result{};
    sil::PmsmPlant plant(plant_params(inertia, lq));
    sil::PlantHal hal(plant, kVBus);
    hal.reset(sil::PlantState{0.0F, 0.0F, 0.0F, 0.0F});
    // With a sensor present the `Motor` reads the angle and the speed from it and never steps the observer. The
    // observer policy is then back-EMF, not HFI, on purpose: an injection observer that is not stepped does not advance
    // its carrier, and a frozen carrier is a DC voltage on the alignment current -- which would compare the two runs on
    // something other than the angle source.
    // 有传感器时 `Motor` 从它读角度与转速，根本不推进观测器。此时观测器策略用反电势而不是 HFI，是有意的：不被推进的
    // 注入观测器不会推进载波，而冻结的载波就是直流电压——那会让两次运行比的不是角度来源。
    hal.position.set_sensor_present(use_sensor);

    const foc::motor::MotorConfig cfg = motor_config(inertia, lq);
    core::SwitchableObserver::Config config{};
    config.back_emf = core::BackEmfObserver::Config{cfg.rs, cfg.ld};
    config.hfi.inject_v = inject_v;
    config.hfi.inject_hz = kInjectHz;
    config.hfi.current_hp_hz = 300.0F;
    config.hfi.demod_lpf_hz = kDemodHz;
    config.hfi.pll_bandwidth_hz = 40.0F;
    config.hfi.pll_zeta = 0.9F;
    config.hfi.demod_gain_a = core::HfiObserver::predicted_gain_a(inject_v, kInjectHz, kLd, lq);
    config.hfi.min_hf_current_a = 0.05F;
    config.hfi.speed_lpf_hz = speed_lpf_hz;
    config.kind = use_sensor ? core::ObserverKind::BackEmf : core::ObserverKind::Hfi;

    SensorlessMotor motor(cfg,
                          SensorlessMotor::Policies{core::SwitchableObserver{config}, core::Svpwm{},
                                                    core::Pi{foc::current_pi_config(cfg, 2000.0F, kVBus)},
                                                    core::Pi{foc::current_pi_config(cfg, 2000.0F, kVBus)},
                                                    core::Pi{foc::speed_pi_config(cfg, 200.0F)},
                                                    core::NullReferenceGenerator{}, core::NullFeedforward{},
                                                    core::PositionP{foc::position_p_config(cfg, 20.0F, 120.0F, 0.0F)},
                                                    core::NullAlignment{}},
                          hal);
    if ((motor.init() != foc::ErrorCode::Ok) || (motor.start() != foc::ErrorCode::Ok)) {
        return result;
    }

    core::Pi speed_pi{foc::speed_pi_config(cfg, 200.0F)};

    // Everything is accumulated in one pass: no history buffer, because the only thing the substep sampling is for is a
    // variance and a crossing count, and both fit in a few registers.
    // 一次通行全部累加：不留历史缓冲，因为子步采样唯一要的东西是方差与过零计数，两者都装得进几个寄存器。
    Goertzel speed_tones[6];
    Goertzel omega_tones[6];
    Goertzel error_tones[6];
    Goertzel torque_tones[6];
    for (int t = 0; t < 6; ++t) {
        speed_tones[t] = Goertzel(kToneBins[t]);
        omega_tones[t] = Goertzel(kToneBins[t]);
        error_tones[t] = Goertzel(kToneBins[t]);
        torque_tones[t] = Goertzel(kToneBins[t]);
    }
    int clamp_count = 0;
    bool in_window = false;
    double ctrl_sum = 0.0;
    double ctrl_sum_sq = 0.0;
    double iq_sum = 0.0;
    double iq_sum_sq = 0.0;
    double torque_sum = 0.0;
    double torque_sum_sq = 0.0;
    int count = 0;
    double sub_sum = 0.0;
    double sub_sum_sq = 0.0;
    int sub_count = 0;
    int crossings = 0;
    float previous_sub = 0.0F;
    double angle_sum = 0.0;
    double angle_sum_sq = 0.0;
    int angle_crossings = 0;
    // The crossings for the angle are taken about a 5 Hz one-pole of the angle itself, because the angle error has a
    // *mean* (the lag, 0.006 rad at speed) that is twelve times its spread: crossing about zero would count the lag and
    // report zero crossings. The filter is the online stand-in for "mean removed in a second pass".
    // 角度的过零是相对角度自身的 5 Hz 一阶滤波取的，因为角度误差有一个**均值**（恒速下的滞后，0.006 rad），是它离散的
    // 十二倍：相对零过零会去数那个滞后、报出零次。这个滤波器是"第二遍去均值"的在线替身。
    float angle_slow = 0.0F;
    const float angle_alpha = std::min(1.0F, kTwoPiLocal * 5.0F * kDt);
    double omega_est_sum = 0.0;
    double omega_est_sum_sq = 0.0;
    float omega_est_slow = 0.0F;
    int omega_est_crossings = 0;

    auto step_once = [&](float iq_command) {
        motor.set_torque(iq_command);
        static_cast<void>(motor.tick(kDt));
        for (int i = 0; i < kSubsteps; ++i) {
            hal.advance(kDt / static_cast<float>(kSubsteps), 1);
            if (in_window) {
                const float speed = plant.omega_mech_rad_s();
                sub_sum += static_cast<double>(speed);
                sub_sum_sq += static_cast<double>(speed) * static_cast<double>(speed);
                if (sub_count > 0) {
                    // Crossings are counted about the *command*, not about the mean, because the mean is only known
                    // after the pass. The two differ by the bias, 0.004 rad/s against a ripple of 0.09, so the crossing
                    // count is the same either way -- and if that ever stops being true the bias line in the output
                    // says so.
                    // 过零是按**指令**而不是按均值计数的，因为均值要等这一遍走完才知道。两者相差一个偏差，0.004 对 0.09 的
                    // 纹波，故两种数法给出同样的过零数——若哪天不再如此，输出里的偏差那一行会说明。
                    const float centred = speed - command;
                    const float previous_centred = previous_sub - command;
                    if (((previous_centred < 0.0F) && (centred >= 0.0F)) ||
                        ((previous_centred > 0.0F) && (centred <= 0.0F))) {
                        ++crossings;
                    }
                }
                previous_sub = speed;
                ++sub_count;
            }
        }
    };

    for (int i = 0; i < static_cast<int>(kSettleSeconds / kDt); ++i) {
        step_once(0.0F);
    }

    const int total = static_cast<int>(kRunSeconds / kDt);
    const int window_start = total - static_cast<int>(kWindowSeconds / kDt);
    for (int i = 0; i < total; ++i) {
        in_window = (i >= window_start);
        // The loop is closed on whatever the drive actually has: the observer's own speed when the angle comes from the
        // injection, and the encoder's when it does not. Reading the observer in the sensored case would read a speed
        // from an observer that is never stepped -- a stuck zero, and the loop would run away (measured: 57 rad/s).
        // 环路闭在驱动真正拥有的东西上：角度来自注入时用观测器自有的转速，否则用编码器的。在带传感器的情形下读观测器，
        // 读到的是一个从不被推进的观测器给出的转速——一个卡住的零，环路会跑飞（实测 57 rad/s）。
        const float feedback = use_sensor
                                   ? hal.position.velocity_rad_s()
                                   : (static_cast<float>(motor.observer().state().omega_e) / static_cast<float>(kPoles));
        step_once(speed_pi.step(command, feedback, kDt));
        if (in_window) {
            // The current in the rotor's own frame: the plant's instantaneous currents projected with the *true* angle.
            // This is the torque-producing current, which is what the mechanical equation sees.
            // 转子自身坐标系里的电流：用**真实**角度把 plant 的瞬时电流投影过去。这是产生转矩的那份电流，也正是机械方程
            // 所看到的量。
            const core::Dq<float> i_dq =
                core::park(core::clarke(plant.current_uvw()), static_cast<float>(kPoles) * plant.theta_mech_rad());
            // The torque expression the plant integrates: `1.5*p*(psi_f*iq + (Ld-Lq)*id*iq)` with `psi_f = kt/(1.5*p)`,
            // so the first term is exactly `kt*iq`.
            // 与 plant 积分所用的转矩表达式相同：`1.5·p·(psi_f·iq + (Ld−Lq)·id·iq)`，其中 `psi_f = kt/(1.5·p)`，故第一项
            // 正好是 `kt·iq`。
            const float torque = (kKt * i_dq.q) + (1.5F * static_cast<float>(kPoles) * (kLd - lq) * i_dq.d * i_dq.q);
            const float speed = plant.omega_mech_rad_s();
            ctrl_sum += static_cast<double>(speed);
            ctrl_sum_sq += static_cast<double>(speed) * static_cast<double>(speed);
            iq_sum += static_cast<double>(i_dq.q);
            iq_sum_sq += static_cast<double>(i_dq.q) * static_cast<double>(i_dq.q);
            torque_sum += static_cast<double>(torque);
            torque_sum_sq += static_cast<double>(torque) * static_cast<double>(torque);
            if (!use_sensor) {
                // The angle the drive actually commutated with, against the rotor it did not read.
                // 驱动真正用来换相的角度，对上一个它没有读过的转子。
                float angle_error = static_cast<float>(motor.observer().state().theta_e) -
                                    (static_cast<float>(kPoles) * plant.theta_mech_rad());
                while (angle_error <= -kPiLocal) {
                    angle_error += kTwoPiLocal;
                }
                while (angle_error > kPiLocal) {
                    angle_error -= kTwoPiLocal;
                }
                const float previous_slow = angle_slow;
                angle_slow += angle_alpha * (angle_error - angle_slow);
                const float ripple_now = angle_error - angle_slow;
                const float ripple_before = angle_error - previous_slow;
                if (count > 1) {
                    if (((ripple_before < 0.0F) && (ripple_now >= 0.0F)) ||
                        ((ripple_before > 0.0F) && (ripple_now <= 0.0F))) {
                        ++angle_crossings;
                    }
                }
                angle_sum += static_cast<double>(angle_error);
                angle_sum_sq += static_cast<double>(angle_error) * static_cast<double>(angle_error);

                // The published speed: this is the number the speed loop closes on, so its own spread is the question.
                // 已发布的转速：这是速度环闭环用的那个数，因此它自己的离散才是问题所在。
                const float omega_est = static_cast<float>(motor.observer().state().omega_e) / static_cast<float>(kPoles);
                const float previous_omega_slow = omega_est_slow;
                omega_est_slow += angle_alpha * (omega_est - omega_est_slow);
                const float omega_ripple_now = omega_est - omega_est_slow;
                const float omega_ripple_before = omega_est - previous_omega_slow;
                if (count > 1) {
                    if (((omega_ripple_before < 0.0F) && (omega_ripple_now >= 0.0F)) ||
                        ((omega_ripple_before > 0.0F) && (omega_ripple_now <= 0.0F))) {
                        ++omega_est_crossings;
                    }
                }
                omega_est_sum += static_cast<double>(omega_est);
                omega_est_sum_sq += static_cast<double>(omega_est) * static_cast<double>(omega_est);

                // The three series that could carry the tone, all sampled at the control rate on the same grid.
                // 三条可能携带该音调的序列，全部按控制率、在同一栅格上采样。
                const float error_signal = motor.observer().error();
                for (int t = 0; t < 6; ++t) {
                    speed_tones[t].push(speed);
                    omega_tones[t].push(omega_est);
                    error_tones[t].push(error_signal);
                    torque_tones[t].push(torque);
                }
                clamp_count += (std::fabs(error_signal) >= 0.99F) ? 1 : 0;
            }
            ++count;
        }
    }

    const double n = static_cast<double>(count);
    const double speed_mean = (n > 0.0) ? (ctrl_sum / n) : 0.0;
    result.speed_bias_rad_s = static_cast<float>(speed_mean - static_cast<double>(command));
    result.speed_ripple_ctrl =
        static_cast<float>(std::sqrt(std::max(0.0, (ctrl_sum_sq / n) - (speed_mean * speed_mean))));
    const double omega_est_mean = (n > 0.0) ? (omega_est_sum / n) : 0.0;
    result.omega_est_mean = static_cast<float>(omega_est_mean);
    result.omega_est_ripple =
        static_cast<float>(std::sqrt(std::max(0.0, (omega_est_sum_sq / n) - (omega_est_mean * omega_est_mean))));
    result.omega_est_hz =
        static_cast<float>(static_cast<double>(omega_est_crossings) * 0.5 / static_cast<double>(kWindowSeconds));
    const double angle_mean = (n > 0.0) ? (angle_sum / n) : 0.0;
    result.angle_mean_rad = static_cast<float>(angle_mean);
    result.angle_std_rad =
        static_cast<float>(std::sqrt(std::max(0.0, (angle_sum_sq / n) - (angle_mean * angle_mean))));
    result.angle_ripple_hz =
        static_cast<float>(static_cast<double>(angle_crossings) * 0.5 / static_cast<double>(kWindowSeconds));
    // What an angle ripple of `delta` does to the injected voltage: the carrier is oriented along the estimate, so a
    // rotation by `delta` puts `v_h*sin(delta) ~= v_h*delta` on the q axis, and that drives `v_h*delta/|Z|` of current at
    // the ripple's frequency. `|Z| = sqrt(Rs^2 + (wL)^2)` takes `L` between `Ld` and `Lq`; the prediction below uses the
    // mean, and the case prints the spread the saliency gives it.
    // `delta` 的角度纹波对注入电压做了什么：载波沿估计方向，于是转动 `delta` 会在 q 轴上留下
    // `v_h·sin(delta) ≈ v_h·delta`，它在纹波频率上驱动出 `v_h·delta/|Z|` 的电流。`|Z| = sqrt(Rs² + (wL)²)` 中的 `L`
    // 取 `Ld` 与 `Lq` 之间；下面的预测取中值，用例会打印凸极带来的区间。
    {
        static constexpr float kTwoPi = 6.28318530717959F;
        const float omega = kTwoPi * std::max(result.angle_ripple_hz, 1.0F);
        const float impedance = std::sqrt((kRs * kRs) + std::pow(omega * 0.5F * (kLd + kLq), 2.0F));
        result.iq_ripple_predicted = (kInjectV * result.angle_std_rad) / impedance;
    }
    const double iq_mean = (n > 0.0) ? (iq_sum / n) : 0.0;
    result.iq_mean_a = static_cast<float>(iq_mean);
    result.iq_ripple_a = static_cast<float>(std::sqrt(std::max(0.0, (iq_sum_sq / n) - (iq_mean * iq_mean))));
    const double torque_mean = (n > 0.0) ? (torque_sum / n) : 0.0;
    result.torque_ripple_nm =
        static_cast<float>(std::sqrt(std::max(0.0, (torque_sum_sq / n) - (torque_mean * torque_mean))));

    const double sn = static_cast<double>(sub_count);
    const double sub_mean = (sn > 0.0) ? (sub_sum / sn) : 0.0;
    result.speed_ripple_sub =
        (sn > 0.0) ? static_cast<float>(std::sqrt(std::max(0.0, (sub_sum_sq / sn) - (sub_mean * sub_mean)))) : 0.0F;

    // Zero crossings of the substep-resolved speed give the dominant tone: two per cycle, over the window's length.
    // 子步分辨率转速的过零给出主导音调：每周期两次，窗口长度作分母。
    result.dominant_hz = static_cast<float>(static_cast<double>(crossings) * 0.5 / static_cast<double>(kWindowSeconds));

    // C3's prediction: a torque ripple of `sigma_T` at the dominant mechanical frequency moves the speed by
    // `sigma_T / (J*omega_dom)`, the steady-state magnitude response of `J*dw/dt + B*w = T` there (B is negligible:
    // `B/J = 100 rad/s`, far below the tone).
    // C3 的预测：频率为主导机械频率、幅值为 sigma_T 的转矩纹波使转速变化 `sigma_T / (J·omega_dom)`，即
    // `J·dw/dt + B·w = T` 在该频率上的稳态幅频响应（B 可忽略：`B/J = 100 rad/s`，远低于该音调）。
    static constexpr float kTwoPi = 6.28318530717959F;
    const float omega_dom = kTwoPi * std::max(result.dominant_hz, 1.0F);
    result.implied_speed_ripple = result.torque_ripple_nm / (inertia * omega_dom);
    result.reached_command = std::fabs(plant.omega_mech_rad_s() - command) < 0.5F;
    for (int t = 0; t < 6; ++t) {
        result.speed_tone[t] = speed_tones[t].amplitude();
        result.omega_tone[t] = omega_tones[t].amplitude();
        result.error_tone[t] = error_tones[t].amplitude();
        result.torque_tone[t] = torque_tones[t].amplitude();
    }
    result.clamp_fraction = (count > 0) ? (static_cast<float>(clamp_count) / static_cast<float>(count)) : 0.0F;
    return result;
}

} // namespace

TEST(SilSpeedRipple, TheFigureSurvivesSubstepResolution) {
    // C1, and it is asked first because everything else quotes this number. Increment 1 reported a 415 Hz ripple from a
    // signal sampled every 100 us; anything at 9.6 kHz would fold to 400 Hz and look exactly the same.
    // 先问 C1，因为其余一切都要引用这个数。增量 1 从一个每 100 us 采样一次的信号里报出 415 Hz 的纹波；9.6 kHz 上的东西
    // 会折到 400 Hz，看起来一模一样。
    const RippleResult result = run_ripple(kBaseJ, false, kInjectV);

    std::printf("\n[ripple] 20 rad/s mech, J %.0e, carrier %.0f Hz\n", static_cast<double>(kBaseJ),
                static_cast<double>(kInjectHz));
    std::printf("[ripple] speed bias %+.4f rad/s, ripple at %.1f kHz %.4f, at 160 kHz %.4f, dominant %.1f Hz\n",
                static_cast<double>(result.speed_bias_rad_s), static_cast<double>(kCtrlHz / 1000.0F),
                static_cast<double>(result.speed_ripple_ctrl), static_cast<double>(result.speed_ripple_sub),
                static_cast<double>(result.dominant_hz));
    std::printf("[ripple] q current %.3f A mean, %.4f A ripple; torque ripple %.2e N*m\n",
                static_cast<double>(result.iq_mean_a), static_cast<double>(result.iq_ripple_a),
                static_cast<double>(result.torque_ripple_nm));

    // The rig has to have reached the command, or the window is not an operating point.
    // 台架必须到达指令，否则窗口不是一个工作点。
    EXPECT_TRUE(result.reached_command) << "the loop never reached 20 rad/s";

    // C1: the two samplings must agree. A control-rate figure that is much *lower* than the substep one was aliased
    // away, and one that is much higher means something in the sampling itself is the signal.
    // C1：两种采样必须一致。控制率下的数字远**低于**子步的，说明它被混叠掉了；远高于则说明采样本身成了信号。
    const float ratio = (result.speed_ripple_sub > 0.0F) ? (result.speed_ripple_ctrl / result.speed_ripple_sub) : 0.0F;
    std::printf("[ripple] C1 control-rate over substep ripple %.3f (needs 0.80..1.20) -> %s\n",
                static_cast<double>(ratio), ((ratio >= 0.8F) && (ratio <= 1.2F)) ? "met" : "NOT met");
    EXPECT_GT(ratio, 0.8F) << "the control-rate ripple is smaller than the real one: the figure was aliased";
    EXPECT_LT(ratio, 1.2F) << "the control-rate ripple is larger than the real one: the sampling is adding signal";
}

TEST(SilSpeedRipple, TheRippleIsMechanicalOrItIsTheController) {
    // C2 and C3 together: raise the inertia tenfold and ask whether the ripple follows it down.
    // C2 与 C3 合在一起：把惯量提高十倍，看纹波是否跟着下来。
    const RippleResult light = run_ripple(kBaseJ, false, kInjectV);
    const RippleResult heavy = run_ripple(kBaseJ * 10.0F, false, kInjectV);

    std::printf("\n[inertia] %10s %12s %12s %12s %12s\n", "J", "bias", "ripple_ctrl", "iq_ripple", "dominant_hz");
    std::printf("[inertia] %10.0e %12.4f %12.4f %12.4f %12.1f\n", static_cast<double>(kBaseJ),
                static_cast<double>(light.speed_bias_rad_s), static_cast<double>(light.speed_ripple_ctrl),
                static_cast<double>(light.iq_ripple_a), static_cast<double>(light.dominant_hz));
    std::printf("[inertia] %10.0e %12.4f %12.4f %12.4f %12.1f\n", static_cast<double>(kBaseJ * 10.0F),
                static_cast<double>(heavy.speed_bias_rad_s), static_cast<double>(heavy.speed_ripple_ctrl),
                static_cast<double>(heavy.iq_ripple_a), static_cast<double>(heavy.dominant_hz));

    EXPECT_TRUE(light.reached_command && heavy.reached_command) << "a run never reached the command";

    const float fall = (heavy.speed_ripple_ctrl > 0.0F) ? (light.speed_ripple_ctrl / heavy.speed_ripple_ctrl) : 0.0F;
    std::printf("[inertia] C2 ten times the inertia, %.2fx less speed ripple (a 1/J story predicts ~10x, the bar is 5x)"
                " -> %s\n",
                static_cast<double>(fall), (fall >= 5.0F) ? "mechanical" : "NOT mechanical");
    // C3: does the current ripple actually account for the speed ripple it is being blamed for?
    // C3：被归咎的电流纹波到底解不解释得了它被归咎的转速纹波？
    const float explanation = (light.speed_ripple_ctrl > 0.0F) ? (light.implied_speed_ripple / light.speed_ripple_ctrl)
                                                               : 0.0F;
    std::printf("[inertia] C3 %+.4f A of q ripple -> %.2e N*m -> %.4f rad/s at %.0f Hz, against %.4f measured (%.2fx)"
                " -> %s\n",
                static_cast<double>(light.iq_ripple_a), static_cast<double>(light.torque_ripple_nm),
                static_cast<double>(light.implied_speed_ripple), static_cast<double>(light.dominant_hz),
                static_cast<double>(light.speed_ripple_ctrl), static_cast<double>(explanation),
                ((explanation > 0.5F) && (explanation < 2.0F)) ? "the chain closes" : "the chain does not close");

    // C2's criterion is **not** met, and that is the finding rather than a failure of the run. Ten times the inertia
    // bought 0.71x less speed ripple instead of 10x, because the q-axis ripple grew 5.9x at the same time: the speed
    // loop's gains are derived from the inertia, so a heavier rotor is asked for a proportionally larger current, and
    // the `1/J` advantage is cancelled by a proportional increase in `T`. The ripple is therefore *inertia-invariant*,
    // which is a property of a fixed-bandwidth cascade and not of the rotor -- the mechanical story is falsified.
    // C2 的判据**没有**成立，而这正是发现本身，而不是这次运行失败。十倍惯量只换来 0.71 倍的转速纹波，而不是 10 倍，因为
    // q 轴纹波同时涨了 5.9 倍：速度环的增益由惯量导出，于是更重的转子被要求给出成比例更大的电流，`1/J` 的好处被 `T` 的
    // 成比例增加抵消。因此纹波是**惯量无关**的——这是固定带宽串级环的性质，不是转子的性质，机械那个说法被否证了。
    EXPECT_LT(fall, 2.0F) << "the ripple did follow the inertia after all, so the mechanical story holds and this case "
                             "needs rewriting back the other way";
    EXPECT_GT(explanation, 0.5F) << "the current ripple is too small to be the cause: something else drives the ripple";
    EXPECT_LT(explanation, 2.0F) << "the current ripple over-predicts: the mechanical equation is not what limits it";
}

TEST(SilSpeedRipple, TheRippleComesFromTheObserverOrFromTheLoop) {
    // C2 said "not the rotor". That leaves two candidates that a single knob separates: the observer in the current
    // loop's path, or the cascade itself. The knob is the angle source -- the same servo, the same plant, the same
    // tuning, with the angle and speed read from the sensor instead of from the injection observer.
    // C2 说"不是转子"。剩下两个候选，用一个旋钮就能分开：观测器在电流环通路上，还是串级环本身。旋钮就是角度来源——同一个
    // 伺服、同一个被控对象、同样整定，只把角度与转速改为从传感器读。
    //
    //   C4  if the sensored run's ripple is at most a fifth of the sensorless one, the observer is in the path and the
    //       residual is a library question; if the two agree within a factor of two, it is the cascade and there is
    //       nothing in the observer to fix. Either answer ends the search, which is why it is one measurement.
    //   C4  若带传感器那一跑的纹波不超过无感那一跑的五分之一，则观测器在通路上、残余是库的问题；若两者在两倍之内一致，
    //       则是串级环的性质、观测器里没有可修的东西。两个答案都终结这次搜索，所以只量一次。
    const RippleResult sensorless = run_ripple(kBaseJ, false, kInjectV);
    const RippleResult sensored = run_ripple(kBaseJ, true, kInjectV);

    std::printf("\n[angle] %10s %12s %12s %12s %12s\n", "source", "bias", "ripple", "iq_ripple", "dominant_hz");
    std::printf("[angle] %10s %12.4f %12.4f %12.4f %12.1f\n", "injection", static_cast<double>(sensorless.speed_bias_rad_s),
                static_cast<double>(sensorless.speed_ripple_ctrl), static_cast<double>(sensorless.iq_ripple_a),
                static_cast<double>(sensorless.dominant_hz));
    std::printf("[angle] %10s %12.4f %12.4f %12.4f %12.1f\n", "sensor", static_cast<double>(sensored.speed_bias_rad_s),
                static_cast<double>(sensored.speed_ripple_ctrl), static_cast<double>(sensored.iq_ripple_a),
                static_cast<double>(sensored.dominant_hz));

    EXPECT_TRUE(sensorless.reached_command && sensored.reached_command) << "a run never reached the command";

    const float share = (sensorless.speed_ripple_ctrl > 0.0F) ? (sensored.speed_ripple_ctrl / sensorless.speed_ripple_ctrl)
                                                              : 1.0F;
    std::printf("[angle] C4 the sensored run keeps %.3f of the ripple\n", static_cast<double>(share));

    // C4's conclusion is **withdrawn**, and the reason is the most useful thing this case produced. The sensored arm
    // dropped the injection along with the observer -- the policy had to be back-EMF, because an injection observer that
    // is never stepped freezes its carrier into a DC voltage. So the two arms differed in two things, not one: the angle
    // *and* the 12 V carrier. The 0.000 below therefore says only "a drive with no injection has no ripple", which is
    // true and nearly useless; it does not say the observer is in the path. The next case separates the two by moving
    // the injection's amplitude while leaving the angle source alone.
    // C4 的结论**撤回**，撤回的理由是本用例给出的最有用的东西。带传感器那一臂把注入连同观测器一起去掉了——策略必须是反电势，
    // 因为不被推进的注入观测器会把载波冻结成直流电压。于是两臂差的不是一件事而是两件：角度**和** 12 V 载波。因此下面的
    // 0.000 只说明"没有注入的驱动没有纹波"，这句话成立且几乎无用；它并不说明观测器在通路上。下一条用例用"只动注入幅值、
    // 不动角度来源"把两者分开。
    EXPECT_LT(share, 1.0F) << "the sensored arm should be quieter, since it has neither an observer nor a carrier";
}

TEST(SilSpeedRipple, TheAngleCarriesTheSameToneTheCurrentDoes) {
    // C4 said the ripple belongs to the observer. This asks whether it is the angle *in particular*, and it does so by
    // comparing two independent quantities at once: the tone's frequency as seen in the angle and as seen in the q
    // current, and the current the angle ripple should produce against the current that is actually there. If the
    // frequencies disagree, or the prediction misses by more than 3x, the angle is not what the loop is chewing on and
    // the mechanism is somewhere else.
    // C4 说纹波属于观测器。这条问它是否**具体地**属于角度，办法是同时比较两个互相独立的量：音调的频率在角度上看到的
    // 与在 q 轴电流上看到的，以及角度纹波应产生的电流与实际存在的电流。若频率不符、或预测偏差超过 3 倍，则环路嚼的不是
    // 角度，机理在别处。
    const RippleResult result = run_ripple(kBaseJ, false, kInjectV);
    std::printf("\n[tone] angle error mean %+.5f rad, spread %.5f rad, ripple %.1f Hz\n",
                static_cast<double>(result.angle_mean_rad), static_cast<double>(result.angle_std_rad),
                static_cast<double>(result.angle_ripple_hz));
    std::printf("[tone] q current ripple %.4f A at %.1f Hz; an angle ripple of %.5f rad predicts %.4f A\n",
                static_cast<double>(result.iq_ripple_a), static_cast<double>(result.dominant_hz),
                static_cast<double>(result.angle_std_rad), static_cast<double>(result.iq_ripple_predicted));

    EXPECT_TRUE(result.reached_command) << "the loop never reached 20 rad/s";

    const float frequency_ratio =
        (result.dominant_hz > 0.0F) ? (result.angle_ripple_hz / result.dominant_hz) : 0.0F;
    const float prediction_ratio =
        (result.iq_ripple_a > 0.0F) ? (result.iq_ripple_predicted / result.iq_ripple_a) : 0.0F;
    std::printf("[tone] C5 the angle's tone over the current's %.2f (needs 0.80..1.25), the predicted current over the"
                " measured %.2f (needs 0.33..3.00) -> %s\n",
                static_cast<double>(frequency_ratio), static_cast<double>(prediction_ratio),
                ((frequency_ratio > 0.8F) && (frequency_ratio < 1.25F) && (prediction_ratio > 0.33F) &&
                 (prediction_ratio < 3.0F))
                    ? "the angle is what the loop chews on"
                    : "the angle does not explain the current, so the mechanism is upstream of it");

    // The two quantities are measured independently -- one against the rotor, one from the stator current -- so their
    // agreement is a real check and not a restatement.
    // 两个量是独立测的——一个对着转子、一个来自定子电流——所以它们的一致是一次真检查，而不是同一句话的两种说法。
    // The hypothesis is **falsified**: the angle error has no tone at the current's frequency at all (zero crossings
    // about its own 5 Hz average), and its spread over-predicts the current by 5x. So the published angle is not what
    // the loop chews on -- the current ripple is created *inside* the current loop, from the carrier it is asked to
    // track, and not by the angle the carrier is oriented with. That is what the case now asserts, because that is what
    // was measured.
    // 该假设被**否证**：角度误差在电流那个频率上完全没有音调（相对自身 5 Hz 均值零次过零），而它的离散把电流高估了 5 倍。
    // 所以已发布的角度不是环路咀嚼的东西——电流纹波是**在电流环内部**产生的，来自它被要求跟踪的载波，而不是来自载波所对齐的
    // 角度。用例现在断言的就是这个，因为量到的就是这个。
    EXPECT_LT(result.angle_ripple_hz, 0.5F * result.dominant_hz)
        << "the angle does carry the current's tone after all, which would put the mechanism back in the observer";
}

TEST(SilSpeedRipple, TheRippleBelongsToTheInjectionNotToTheAngle) {
    // The arm C4 needed: the angle source stays the injection observer, and only the carrier's amplitude moves. The HF
    // current is proportional to `inject_v`, so an injection-driven ripple scales with it and an angle-driven one does
    // not.
    // C4 缺的那一臂：角度来源保持不变（注入观测器），只动载波的幅值。高频电流正比于 `inject_v`，因此注入驱动的纹波会随之
    // 缩放，角度驱动的不会。
    //
    //   C6  the current and speed ripple must fall with the amplitude: at a third of the volts, at most 60 % of the
    //       ripple. Flat ripple would send the mechanism back to something independent of the carrier, and the previous
    //       case has already excluded the angle.
    //   C6  电流与转速纹波必须随幅值下降：三分之一的伏特，纹波至多 60%。纹波不动则机理回到某个与载波无关的东西上，而上一条
    //       用例已经排除了角度。
    const RippleResult full = run_ripple(kBaseJ, false, 12.0F);
    const RippleResult third = run_ripple(kBaseJ, false, 4.0F);

    std::printf("\n[amplitude] %10s %12s %12s %12s %12s\n", "inject_v", "bias", "speed_ripple", "iq_ripple",
                "dominant_hz");
    std::printf("[amplitude] %10.0f %12.4f %12.4f %12.4f %12.1f\n", 12.0, static_cast<double>(full.speed_bias_rad_s),
                static_cast<double>(full.speed_ripple_ctrl), static_cast<double>(full.iq_ripple_a),
                static_cast<double>(full.dominant_hz));
    std::printf("[amplitude] %10.0f %12.4f %12.4f %12.4f %12.1f\n", 4.0, static_cast<double>(third.speed_bias_rad_s),
                static_cast<double>(third.speed_ripple_ctrl), static_cast<double>(third.iq_ripple_a),
                static_cast<double>(third.dominant_hz));

    EXPECT_TRUE(full.reached_command && third.reached_command) << "a run never reached the command";

    const float speed_share =
        (full.speed_ripple_ctrl > 0.0F) ? (third.speed_ripple_ctrl / full.speed_ripple_ctrl) : 1.0F;
    const float iq_share = (full.iq_ripple_a > 0.0F) ? (third.iq_ripple_a / full.iq_ripple_a) : 1.0F;
    std::printf("[amplitude] C6 a third of the volts keeps %.2f of the speed ripple and %.2f of the current ripple"
                " (needs <= 0.60) -> %s\n",
                static_cast<double>(speed_share), static_cast<double>(iq_share),
                (speed_share <= 0.6F) ? "the injection drives the ripple" : "the ripple does not follow the injection");
    // C6 is **not** met either, and the split it printed is what matters: the current ripple followed the amplitude
    // (0.54) while the speed ripple did not move (0.98), and the tone moved from 418 to 92 Hz. A ripple whose current
    // halves while its speed stays put, at a frequency that depends on the loop gain, is not the carrier's own current:
    // it is the loop re-exciting itself through the number it measures with. That number is the observer's published
    // speed, and the next case looks straight at it.
    // C6 也**没有**成立，而它打印出来的拆分才是要点：电流纹波跟着幅值走了（0.54），转速纹波却不动（0.98），音调从 418 挪到
    // 92 Hz。电流减半而转速不动、且频率随环路增益变化——这不是载波自身电流的纹波，而是环路通过它用来测量的那个数把自己重新
    // 激励起来。那个数是观测器发布的转速，下一条用例直接看它。
    EXPECT_LT(iq_share, 0.6F) << "the current ripple should follow the amplitude, since the HF current does";
}

TEST(SilSpeedRipple, TheLoopChasesARippledEstimate) {
    // Everything excluded so far leaves one number: the speed the loop actually closes on, which is the observer's.
    // The observer publishes `omega_int_ + kp*error`, so whatever ripples in the demodulated error goes straight into
    // the published speed with no filtering but the loop's own, and the speed PI then commands torque proportional to
    // it. If the published speed ripples far more than the rotor does, the loop is chasing its own measurement.
    // 到目前排除掉的一切之后只剩下一个数：环路真正闭在其上的转速，也就是观测器的。观测器发布 `omega_int_ + kp·error`，
    // 于是解调误差里任何纹波都未经滤波直接进入已发布转速，速度 PI 再据此下达成比例的转矩。若已发布转速比转子本身的纹波大
    // 得多，环路就是在追自己的测量。
    //
    //   C7  the published speed must ripple at least 3x more than the true one, and at the same tone, for the loop to be
    //       chasing its own measurement. If they are within a factor of two, the estimate is honest and the ripple is
    //       generated downstream of it -- which would put the current loop back in the frame.
    //   C7  已发布转速的纹波必须至少是真实转速的 3 倍，且在同一个音调上，才说明环路在追自己的测量。若两者在两倍之内，估计
    //       是诚实的、纹波产生在它下游——那样电流环就重新成为嫌疑。
    const RippleResult result = run_ripple(kBaseJ, false, kInjectV);
    std::printf("\n[chase] published speed %.4f rad/s mean, ripple %.4f at %.1f Hz\n",
                static_cast<double>(result.omega_est_mean), static_cast<double>(result.omega_est_ripple),
                static_cast<double>(result.omega_est_hz));
    std::printf("[chase] true speed ripple %.4f at %.1f Hz\n", static_cast<double>(result.speed_ripple_ctrl),
                static_cast<double>(result.dominant_hz));

    EXPECT_TRUE(result.reached_command) << "the loop never reached 20 rad/s";

    const float amplification = (result.speed_ripple_ctrl > 0.0F)
                                    ? (result.omega_est_ripple / result.speed_ripple_ctrl)
                                    : 0.0F;
    const float tone_ratio = (result.dominant_hz > 0.0F) ? (result.omega_est_hz / result.dominant_hz) : 0.0F;
    std::printf("[chase] C7 the estimate ripples %.2fx the rotor (needs >= 3.00), tones %.2f (needs 0.80..1.25) -> %s\n",
                static_cast<double>(amplification), static_cast<double>(tone_ratio),
                ((amplification >= 3.0F) && (tone_ratio > 0.8F) && (tone_ratio < 1.25F))
                    ? "the loop is chasing its own measurement"
                    : "the estimate is honest, so the ripple is made downstream of it");
}

TEST(SilSpeedRipple, TheSpectrumNamesTheToneOrSaysThereIsNone) {
    // Increment 2 ended without a mechanism and with one unreliable number: a crossing counter said 418 Hz at 12 V and
    // 92 Hz at 4 V for the same 0.09 rad/s amplitude. A crossing count cannot tell one tone from three, so this case
    // replaces it with an exact reading at six candidate frequencies, each of which owns a named mechanism, plus the one
    // measurement the relay hypothesis is actually about: how much of the time the demodulated error sits in its +-1
    // clamp.
    // 增量 2 结束在一个没有机理、且有一个不可靠数字的地方：过零计数器在 12 V 说 418 Hz、在 4 V 说 92 Hz，而幅值都是
    // 0.09 rad/s。过零计数分不出一条音调与三条音调，因此本用例用六个候选频率上的精确读数取代它——每个候选都有名有姓
    // 地对应一个机理——并补上继电假设真正关心的那个测量：解调误差有多大比例的时间坐在它的 ±1 钳位里。
    //
    //   C8  the relay story predicts the PLL is driven into its clamp: at least a tenth of the window with `|error|` at
    //       1.0, and much less of it at a third of the volts. If the clamp is never reached, the loop is not saturating
    //       and that story is dead.
    //   C9  one candidate must own the ripple: the largest amplitude among the six must exceed their median by 1.5x. If
    //       none does, the ripple is broadband, and a broadband ripple is noise rather than a limit cycle -- a different
    //       mechanism with a different fix.
    //
    //   C8  继电假设预测 PLL 被驱入钳位：至少十分之一的窗口里 `|error|` 在 1.0，而在三分之一的伏特下要少得多。若钳位从未
    //       到达，环路就没有饱和，那个说法作废。
    //   C9  必须有一个候选拥有这个纹波：六者中最大者要比它们的中位数大 1.5 倍。若没有，纹波是宽带的，而宽带纹波是噪声而不
    //       是极限环——不同的机理、不同的修法。
    const RippleResult full = run_ripple(kBaseJ, false, 12.0F);
    const RippleResult third = run_ripple(kBaseJ, false, 4.0F);

    std::printf("\n[spectrum] amplitudes at six bin-aligned candidates, 12 V injection\n");
    std::printf("[spectrum] %9s %-24s %12s %12s %12s\n", "freq_hz", "who owns that frequency", "true_speed",
                "published", "error_sig");
    for (int t = 0; t < 6; ++t) {
        std::printf("[spectrum] %9.2f %-24s %12.5f %12.5f %12.5f\n", static_cast<double>(kToneHz[t]),
                    kToneOwner[t], static_cast<double>(full.speed_tone[t]), static_cast<double>(full.omega_tone[t]),
                    static_cast<double>(full.error_tone[t]));
    }

    EXPECT_TRUE(full.reached_command && third.reached_command) << "a run never reached the command";

    // C8: is the clamp engaged at all, and does it depend on the volts?
    // C8：钳位到底有没有被用上，以及它是否依赖伏特数。
    std::printf("[spectrum] C8 clamp occupancy: %.3f at 12 V, %.3f at 4 V (needs >= 0.100 and much lower at 4 V)\n",
                static_cast<double>(full.clamp_fraction), static_cast<double>(third.clamp_fraction));
    const bool saturating = full.clamp_fraction >= 0.10F;
    std::printf("[spectrum]     -> %s\n",
                saturating ? "the error spends real time in its clamp, so the PLL is a nonlinearity, not a filter"
                           : "the clamp is never reached, so the loop is linear here and the relay story is dead");

    // C8 is **falsified**: the clamp is never reached at either amplitude, so the loop is linear here and the relay
    // story -- the one the 418/92 Hz frequency shift suggested -- is dead. The shift itself was an artefact of the
    // crossing counter reading a multi-tone signal; the exact amplitudes below show three tones at once. Pinning the
    // absence is worth as much as pinning a presence: if a future change ever drives the error into its clamp, this is
    // the case that says so.
    // C8 被**否证**：两种幅值下钳位都没有到达，因此这里环路是线性的，而 418/92 Hz 的频率移动所暗示的那个继电说法作废。
    // 那个移动本身是过零计数器读一个多音调信号的产物；下面的精确幅值一次显示出三条音调。钉住"不存在"与钉住"存在"一样有
    // 价值：若以后某个改动真的把误差驱入钳位，说出这件事的就是本用例。
    EXPECT_LT(full.clamp_fraction, 0.10F) << "the error now reaches its clamp, so the loop is a nonlinearity and the "
                                              "relay story has to be examined again instead of being closed here";

    // C9: which candidate owns the ripple, if any?
    // C9：若有的话，哪个候选拥有这个纹波？
    int winner = 0;
    float sorted[6]{};
    for (int t = 0; t < 6; ++t) {
        sorted[t] = full.speed_tone[t];
        if (full.speed_tone[t] > full.speed_tone[winner]) {
            winner = t;
        }
    }
    // A six-element median without a library sort: three comparisons would do, six is clearer.
    // 六元素中位数不借库排序：三次比较就够，写六个更清楚。
    for (int i = 0; i < 6; ++i) {
        for (int j = i + 1; j < 6; ++j) {
            if (sorted[j] < sorted[i]) {
                const float swap = sorted[i];
                sorted[i] = sorted[j];
                sorted[j] = swap;
            }
        }
    }
    const float median = 0.5F * (sorted[2] + sorted[3]);
    const float dominance = (median > 0.0F) ? (full.speed_tone[winner] / median) : 0.0F;
    std::printf("[spectrum] C9 the loudest candidate is %.2f Hz (%s) at %.5f rad/s, %.2fx the median (needs 1.50x)"
                " -> %s\n",
                static_cast<double>(kToneHz[winner]), kToneOwner[winner], static_cast<double>(full.speed_tone[winner]),
                static_cast<double>(dominance),
                (dominance >= 1.5F) ? "a tone owns the ripple" : "no tone dominates: the ripple is broadband");

    EXPECT_GT(dominance, 1.5F) << "no candidate dominates the true speed's spectrum, so the ripple is broadband and "
                                  "the limit-cycle story needs replacing rather than confirming";

    // And the number that makes this actionable rather than merely named: the tone is the carrier, and the *published*
    // speed carries it ten times harder than the rotor does. So the rotor is not generating it -- the loop is being fed
    // it. `omega_e` is `omega_int + kp*error` and `error` is the demodulated signal, which carries carrier-frequency
    // content by construction; nothing filters it before a speed loop closes on it.
    // 而让这条从"有名"变成"可动手"的数字是：音调就是载波，而**已发布的**转速带着它的力度是转子的十倍。所以转子不是在生成
    // 它——是环路在被喂它。`omega_e` 等于 `omega_int + kp·error`，而 `error` 就是解调信号，按构造携带载波频率的成分；在
    // 一个速度环闭上去之前，没有任何东西滤过它。
    const float published_over_rotor = (full.speed_tone[5] > 0.0F) ? (full.omega_tone[5] / full.speed_tone[5]) : 0.0F;
    std::printf("[spectrum] the published speed carries the carrier tone %.1fx harder than the rotor does\n",
                static_cast<double>(published_over_rotor));
    EXPECT_GT(published_over_rotor, 5.0F)
        << "the published speed no longer carries the carrier tone, so this conclusion no longer holds";
}

TEST(SilSpeedRipple, ThePublishedSpeedCanBeCleanedAndTheRotorDoesNotCare) {
    // Increment 3 named a defect: `omega_e = omega_int + kp*error`, and `error` carries the carrier, so the published
    // speed holds 0.4180 rad/s at 2000 Hz. This case does two things at once, and the second one is why it is worth
    // keeping the first: it sizes a filter on the published speed, and it tests whether that tone was what the rotor was
    // doing.
    // 增量 3 点名了一个缺陷：`omega_e = omega_int + kp·error`，而 `error` 携带载波，于是已发布转速在 2000 Hz 上带着
    // 0.4180 rad/s。本用例一次做两件事，而第二件正是第一件值得保留的原因：它给已发布转速定容一个滤波器，同时检验那条音调
    // 是否就是转子在做的事。
    //
    // Where the filter goes was measured too. The first version put it on the proportional path *inside* the loop and
    // every cutoff from 50 to 400 Hz ran the axis away (-115 to -170 rad/s): the proportional term is what damps this PLL
    // below its own 40 Hz bandwidth. It now sits at the publication boundary and `theta_e` integrates the raw `omega`.
    // 滤波器放在哪里也是量出来的。第一版把它放在环路**内部**的比例通路上，50 到 400 Hz 每一个截止频率都把轴送飞（−115 到
    // −170 rad/s）：比例项正是在 40 Hz 带宽以下给这个 PLL 提供阻尼的东西。它现在坐在发布边界上，`theta_e` 积分原始 `omega`。
    //
    //   C10a the published carrier tone must fall by at least 5x at the chosen cutoff;
    //   C10b the closed-loop speed ripple must fall by at least 2x -- otherwise the tone was not what the rotor was doing;
    //   C10c the steady bias must stay within 20 % of the unfiltered run: a filter that bought quiet with accuracy is a
    //        different trade and has to be stated as one;
    //   C10d and the loop must still reach the command at every cutoff tried, because a filter that destabilises the loop
    //        would show up here and nowhere else.
    //
    //   C10a 在选定截止频率上，已发布的载波音调必须至少降 5 倍；
    //   C10b 闭环转速纹波必须至少降 2 倍——否则那条音调并不是转子在做的事；
    //   C10c 稳态偏差必须留在未滤波那一跑的 20% 之内：用精度换安静的滤波器是另一笔交易，必须如实说成那样；
    //   C10d 且每个试过的截止频率下环路都必须仍能到达指令，因为一个让环路失稳的滤波器只会在这里显形。
    std::printf("\n[filter] low-pass on the published speed, 20 rad/s mech, 12 V at 2 kHz\n");
    std::printf("[filter] %10s %12s %12s %12s %10s\n", "cutoff_hz", "pub_tone", "speed_ripple", "bias", "reached");

    constexpr float kCutoffs[5] = {0.0F, 50.0F, 100.0F, 200.0F, 400.0F};
    RippleResult rows[5]{};
    for (int i = 0; i < 5; ++i) {
        rows[i] = run_ripple(kBaseJ, false, kInjectV, kCutoffs[i]);
        std::printf("[filter] %10.0f %12.5f %12.5f %12.5f %10s\n", static_cast<double>(kCutoffs[i]),
                    static_cast<double>(rows[i].omega_tone[5]), static_cast<double>(rows[i].speed_ripple_ctrl),
                    static_cast<double>(rows[i].speed_bias_rad_s), rows[i].reached_command ? "yes" : "NO");
    }

    // The chosen cutoff, and the three ratios that justify it.
    // 选定的截止频率，以及为它辩护的三个比值。
    constexpr int kChosen = 3; // 200 Hz
    const float tone_fall = (rows[kChosen].omega_tone[5] > 0.0F) ? (rows[0].omega_tone[5] / rows[kChosen].omega_tone[5])
                                                                : 0.0F;
    const float ripple_fall =
        (rows[kChosen].speed_ripple_ctrl > 0.0F) ? (rows[0].speed_ripple_ctrl / rows[kChosen].speed_ripple_ctrl) : 0.0F;
    const float bias_change = (rows[0].speed_bias_rad_s != 0.0F)
                                  ? (rows[kChosen].speed_bias_rad_s / rows[0].speed_bias_rad_s)
                                  : 1.0F;
    std::printf("[filter] C10a the published carrier tone falls %.1fx (needs 5.0x) -> %s\n",
                static_cast<double>(tone_fall), (tone_fall >= 5.0F) ? "met" : "NOT met");
    std::printf("[filter] C10b the closed-loop speed ripple falls %.2fx (needs 2.0x) -> %s\n",
                static_cast<double>(ripple_fall), (ripple_fall >= 2.0F) ? "met" : "NOT met");
    std::printf("[filter] C10c the steady bias moves to %.2fx of the unfiltered run (must stay under 1.20x) -> %s\n",
                static_cast<double>(bias_change), (bias_change < 1.2F) ? "met" : "NOT met");

    for (int i = 0; i < 5; ++i) {
        EXPECT_TRUE(rows[i].reached_command) << "cutoff " << kCutoffs[i] << " left the loop unable to reach 20 rad/s";
    }
    EXPECT_GT(tone_fall, 5.0F) << "the filter does not remove the carrier from the published speed";
    EXPECT_LT(bias_change, 1.2F) << "the filter cost steady accuracy, which is a different trade than the one claimed";

    // C10b is **falsified**, and it is the more valuable half of this case. Removing 10x of the carrier tone from the
    // published speed moved the closed-loop ripple from 0.0900 to 0.0879 -- nothing. So the tone in the rotor was never
    // the loop chasing its own published speed, however strongly the two correlated at 2000 Hz: a correlation with the
    // measurement is not a path through the loop. What survives is that the published speed is *cleaner*, which is a
    // property of the number the application reads and not of the axis. The case asserts the falsification so that a
    // future change which *does* move the axis shows up here as a change of story rather than as a quiet improvement.
    // C10b 被**否证**，而这是本用例更有价值的那一半。把已发布转速里的载波音调拿走 10 倍，闭环纹波从 0.0900 变到 0.0879
    // ——什么也没发生。所以转子上那条音调从来不是环路在追自己发布的转速，无论两者在 2000 Hz 上相关得多强：与测量量相关不等于
    // 存在一条穿过环路的通路。留下的结论是已发布转速**更干净**了，那是"应用读到的那个数"的性质，不是轴的性质。用例把这次
    // 否证断言下来，于是以后某个**真的**推动轴的改动会在这里表现为故事的改变，而不是一次悄然的改善。
    EXPECT_LT(ripple_fall, 1.5F) << "filtering the published speed now moves the axis, so the tone had a path through "
                                    "the loop after all and this conclusion needs rewriting";
}

TEST(SilSpeedRipple, TheToneIsTheInjectionsOwnCrossCouplingNotAReluctanceTorque) {
    // The last candidate, and the only path by which the carrier reaches the rotor without passing through anything this
    // library publishes: a torque at the carrier frequency produced by the injected current itself. The name for it was
    // expected to be the reluctance product `1.5*p*(Ld-Lq)*id*iq`, and the measurement says it is not that -- see C12.
    // Two independent knobs, both of which predict linearity:
    //   1. the volts. `id` is proportional to `inject_v`, so the torque tone is too.
    //   2. the saliency. At the *same* volts the injected d-axis current is unchanged -- it is set by `Ld`, which this
    //      knob leaves alone -- so halving `(Ld-Lq)` must halve the torque tone. The first version of this knob scaled
    //      `inject_v` with `lq` on the reasoning that `V/(w*L)` would stay constant; that is wrong, because the injected
    //      current flows on the *d* axis and is set by `Ld`, so the knob moved the current and the saliency at once and
    //      tested neither. Recorded because it is the kind of error that looks like a refutation.
    // A third check closes the chain without asking the same question twice: the measured torque tone, pushed through the
    // mechanical equation at 2 kHz, must reproduce the measured speed tone.
    // 最后一个候选，也是载波到达转子**不经过库所发布的任何东西**的唯一通路：注入的 d 轴电流乘上负载电流、经凸极，产生
    // 一个载波频率上的转矩 `1.5·p·(Ld−Lq)·id·iq`。两个独立旋钮检验它，两者都预测线性：
    //   1. 伏特。`id` 正比于 `inject_v`，转矩音调亦然。
    //   2. 凸极。在**同样伏特**下注入的 d 轴电流不变——它由 `Ld` 决定，而这个旋钮不动 `Ld`——于是 `(Ld−Lq)` 减半必然使
    //      转矩音调减半。这个旋钮的第一版把 `inject_v` 随 `lq` 缩放，理由是 `V/(ω·L)` 保持不变；那是错的，因为注入电流
    //      走的是 **d** 轴、由 `Ld` 决定，于是那个旋钮同时动了电流与凸极，两个都没测到。记下来，因为这类错误看起来像否证。
    // 第三项检查闭合链条而不把同一个问题问两遍：实测转矩音调经机械方程在 2 kHz 上推算，必须还原出实测转速音调。
    //
    //   C11 the speed tone must fall with the volts: a third of the volts gives between 0.25x and 0.45x the tone.
    //   C12 at the same volts with the saliency halved, the tone must halve: 0.35x to 0.65x.
    //   C13 the torque tone through `1/(J*w)` must land within 0.7x..1.4x of the measured speed tone.
    //   C11 转速音调必须随伏特下降：三分之一的伏特给出音调的 0.25–0.45 倍。
    //   C12 同样伏特下凸极减半时，音调必须减半：0.35–0.65 倍。
    //   C13 转矩音调经 `1/(J·ω)` 推算必须落在实测转速音调的 0.7–1.4 倍之内。
    constexpr int kCarrierBin = 5; // 2000 Hz
    const RippleResult at_12v = run_ripple(kBaseJ, false, 12.0F);
    const RippleResult at_4v = run_ripple(kBaseJ, false, 4.0F);
    // The same volts, half the saliency: `Ld - lq` goes from 2e-4 to 1e-4 while the injected d current is untouched.
    // 同样伏特、一半凸极：`Ld − lq` 从 2e-4 变到 1e-4，而注入的 d 轴电流原封不动。
    const RippleResult half_saliency = run_ripple(kBaseJ, false, 12.0F, 0.0F, 3.0e-4F);
    const RippleResult full_saliency = run_ripple(kBaseJ, false, 12.0F, 0.0F, 4.0e-4F);

    std::printf("\n[reluctance] the 2000 Hz tone under two independent knobs\n");
    std::printf("[reluctance] %-28s %12s %12s %12s\n", "run", "speed_tone", "torque_tone", "hf_current");
    std::printf("[reluctance] %-28s %12.5f %12.3e %12s\n", "12 V, Lq 4e-4", static_cast<double>(at_12v.speed_tone[kCarrierBin]),
                static_cast<double>(at_12v.torque_tone[kCarrierBin]), "-");
    std::printf("[reluctance] %-28s %12.5f %12.3e %12s\n", "4 V, Lq 4e-4", static_cast<double>(at_4v.speed_tone[kCarrierBin]),
                static_cast<double>(at_4v.torque_tone[kCarrierBin]), "-");
    std::printf("[reluctance] %-28s %12.5f %12.3e %12s\n", "12 V, Lq 3e-4 (half saliency)", static_cast<double>(half_saliency.speed_tone[kCarrierBin]),
                static_cast<double>(half_saliency.torque_tone[kCarrierBin]), "-");

    EXPECT_TRUE(at_12v.reached_command && at_4v.reached_command && half_saliency.reached_command && full_saliency.reached_command)
        << "a run never reached the command";

    const float volts_share = (at_12v.speed_tone[kCarrierBin] > 0.0F)
                                  ? (at_4v.speed_tone[kCarrierBin] / at_12v.speed_tone[kCarrierBin])
                                  : 0.0F;
    const float saliency_share = (full_saliency.speed_tone[kCarrierBin] > 0.0F)
                                     ? (half_saliency.speed_tone[kCarrierBin] / full_saliency.speed_tone[kCarrierBin])
                                     : 0.0F;
    std::printf("[reluctance] C11 a third of the volts keeps %.2f of the tone (a linear reluctance torque predicts 0.33)"
                " -> %s\n",
                static_cast<double>(volts_share),
                ((volts_share > 0.25F) && (volts_share < 0.45F)) ? "linear in the volts" : "NOT linear in the volts");
    std::printf("[reluctance] C12 half the saliency at the same HF current keeps %.2f of the tone (predicts 0.50) -> %s\n",
                static_cast<double>(saliency_share),
                ((saliency_share > 0.35F) && (saliency_share < 0.65F)) ? "linear in the saliency" : "NOT linear in the saliency");

    static constexpr float kTwoPi = 6.28318530717959F;
    const float omega_carrier = kTwoPi * kToneHz[kCarrierBin];
    const float predicted = at_12v.torque_tone[kCarrierBin] / (kBaseJ * omega_carrier);
    const float chain = (at_12v.speed_tone[kCarrierBin] > 0.0F) ? (predicted / at_12v.speed_tone[kCarrierBin]) : 0.0F;
    std::printf("[reluctance] C13 %.3e N*m through 1/(J*w) predicts %.5f rad/s against %.5f measured (%.2fx) -> %s\n",
                static_cast<double>(at_12v.torque_tone[kCarrierBin]), static_cast<double>(predicted),
                static_cast<double>(at_12v.speed_tone[kCarrierBin]), static_cast<double>(chain),
                ((chain > 0.7F) && (chain < 1.4F)) ? "the torque is the source" : "the torque does not explain it");

    EXPECT_GT(volts_share, 0.25F) << "the tone barely moved with the volts, so the injected current is not its source";
    EXPECT_LT(volts_share, 0.45F) << "the tone moved more than linearly with the volts";
    // C12 is **falsified**, with the knob corrected so that only the saliency moves. Half the saliency at the same volts
    // leaves 0.97 of the tone: the torque ripple is not the reluctance product `(Ld-Lq)*id*iq`. What survives is what
    // C11 established -- the tone is linear in the injected current -- and a mechanism with exactly those two signatures
    // is the injected current leaking onto the *torque* axis: the estimate is off by `delta`, so `id_hf*sin(delta)` lands
    // on q and earns `kt` of torque. That is proportional to `inject_v` and indifferent to the saliency, which is what
    // the two knobs measured. The next case tests it directly instead of leaving it as the last explanation standing.
    // C12 被**否证**，而且是在旋钮改对、只动凸极之后。同样伏特下凸极减半，音调还剩 0.97：转矩纹波不是凸极积
    // `(Ld−Lq)·id·iq`。存活下来的是 C11 确立的那条——音调线性于注入电流——而同时具备这两个签名的机理正是注入电流漏到
    // **转矩**轴上：估计偏了 `delta`，于是 `id_hf·sin(delta)` 落到 q 轴、挣到 `kt` 的转矩。它正比于 `inject_v`、与凸极
    // 无关，这正是两个旋钮量到的。下一条用例直接检验它，而不是把它留成"最后一个还站着的解释"。
    // Recorded as the falsification it is: the tone is essentially *indifferent* to the saliency, so the reluctance
    // product is not its source. The assertion pins the measurement, not the wish.
    // 作为它本来就是的否证记录下来：音调对凸极基本**无所谓**，所以凸极积不是它的来源。断言钉住的是测量，不是愿望。
    EXPECT_GT(saliency_share, 0.85F) << "the tone does move with the saliency, so the reluctance product is its source "
                                        "after all and this conclusion has to be rewritten";
    EXPECT_GT(chain, 0.7F) << "the torque tone under-predicts the speed tone";
    EXPECT_LT(chain, 1.4F) << "the torque tone over-predicts the speed tone";
}

TEST(SilSpeedRipple, TheToneIsTheInjectedCurrentLeakingOntoTorque) {
    // The last explanation standing after C12: the torque ripple is `kt * id_hf * sin(delta)`, where `delta` is how far
    // the commutation angle is off. That makes one sharp prediction that needs no new constant, because the rig already
    // reports `delta`: the torque tone divided by the mean angle error must be the *same* at two different speeds, where
    // `delta` differs by a factor of four while `id_hf` does not.
    // C12 之后最后一个还站着的解释：转矩纹波是 `kt·id_hf·sin(delta)`，`delta` 是换相角偏了多少。它给出一个尖锐的预测，
    // 且不需要新的常数，因为台架本来就报出 `delta`：**两个不同转速**下，转矩音调除以角度误差均值必须相同——那里 `delta`
    // 差四倍，而 `id_hf` 不变。
    //
    //   C14 the ratio torque_tone / |angle error| must agree between 20 and 5 rad/s within 35 %. A ratio that scales with
    //       the carrier leakage instead (i.e. is the same absolute torque at both speeds) would rule this out and leave
    //       the tone unexplained again.
    //   C14 20 与 5 rad/s 之间 torque_tone / |角度误差| 必须在 35% 之内一致。若这个比值不成立、而转矩音调在两个转速下
    //       绝对值相同（即与 `delta` 无关），则这条也被排除，音调重新变成没有解释。
    constexpr int kCarrierBin = 5;
    const RippleResult fast = run_ripple(kBaseJ, false, 12.0F, 0.0F, kLq, 20.0F);
    const RippleResult slow = run_ripple(kBaseJ, false, 12.0F, 0.0F, kLq, 5.0F);

    std::printf("\n[leak] the same injection at two speeds, and what the torque tone is proportional to\n");
    std::printf("[leak] %10s %12s %12s %12s %12s\n", "speed", "angle_error", "torque_tone", "ratio", "speed_tone");
    const float fast_ratio = (std::fabs(fast.angle_mean_rad) > 0.0F)
                                 ? (fast.torque_tone[kCarrierBin] / std::fabs(fast.angle_mean_rad))
                                 : 0.0F;
    const float slow_ratio = (std::fabs(slow.angle_mean_rad) > 0.0F)
                                 ? (slow.torque_tone[kCarrierBin] / std::fabs(slow.angle_mean_rad))
                                 : 0.0F;
    std::printf("[leak] %10.1f %12.5f %12.3e %12.4f %12.5f\n", 20.0, static_cast<double>(fast.angle_mean_rad),
                static_cast<double>(fast.torque_tone[kCarrierBin]), static_cast<double>(fast_ratio),
                static_cast<double>(fast.speed_tone[kCarrierBin]));
    std::printf("[leak] %10.1f %12.5f %12.3e %12.4f %12.5f\n", 5.0, static_cast<double>(slow.angle_mean_rad),
                static_cast<double>(slow.torque_tone[kCarrierBin]), static_cast<double>(slow_ratio),
                static_cast<double>(slow.speed_tone[kCarrierBin]));

    EXPECT_TRUE(fast.reached_command && slow.reached_command) << "a run never reached the command";

    const float agreement = (slow_ratio > 0.0F) ? (fast_ratio / slow_ratio) : 0.0F;
    std::printf("[leak] C14 the torque tone per radian of angle error agrees to %.2fx between the two speeds"
                " (needs 0.65..1.35) -> %s\n",
                static_cast<double>(agreement),
                ((agreement > 0.65F) && (agreement < 1.35F)) ? "the tone is id_hf times the angle error"
                                                             : "the tone is not proportional to the angle error");
}
