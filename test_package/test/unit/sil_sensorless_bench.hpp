// Conan::ImportStart
#pragma once
// Conan::ImportEnd

// One sensorless rig, two questions. `test_sil_sensorless_floor.cpp` asks where the angle stops being available (a
// disproof); `test_sop_sensorless_floor.cpp` replays the same procedure on a second machine (a procedure). Two copies of
// the rig would make that replay a comparison of two measurements rather than of two machines, so it lives here with the
// machine's magnet and the inverter's dead time as **parameters** -- the defaults reproduce the F19 file exactly.
// 一个无感台架，两个问题。`test_sil_sensorless_floor.cpp` 问角度从哪里开始不可用（一条否证）；
// `test_sop_sensorless_floor.cpp` 在同一流程上换一台机器重放（一条流程）。两份台架会让那次重放变成两个测量的比较而不是
// 两台机器的比较，所以它在这里，并把机器的磁钢与逆变的死区做成 **参数** ——默认值逐字复现 F19 那个文件。

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>

#include <foc/core/observer/switchable_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_pmsm_plant.hpp"
#include "sil_plant_hal.hpp"

namespace foc {

namespace sil {

constexpr float kSopPi = 3.14159265358979F;

// The nameplate `test_sil_fusion_chain.cpp` uses, with one deliberate change: `lq == ld`. That single equality is what
// makes the plant an SPM, and it is why the injection observer cannot be a way out of this measurement.
// `test_sil_fusion_chain.cpp` 用的铭牌，只改一处：`lq == ld`。这一个等号就是 plant 成为 SPM 的原因，也是注入观测器
// 无法成为这次测量出路的原因。
constexpr float kSopRs = 0.35F;
constexpr float kSopLd = 2.0e-4F;
constexpr float kSopLq = kSopLd;
constexpr float kSopKt = 0.05F;
constexpr float kSopJ = 1.0e-5F;
constexpr float kSopB = 1.0e-4F;
constexpr std::uint8_t kSopPoles = 7U;

constexpr float kSopCtrlHz = 20000.0F;
constexpr float kSopDt = 1.0F / kSopCtrlHz;
constexpr int kSopSubsteps = 8;
constexpr float kSopVBus = 24.0F;

// The gate that was withdrawn for being vacuous on a sensored rig, reused here as the gate it was always meant to be:
// "within 20 ms of hand-over the angle error is at most 0.05 rad".
// 那条因为"在有传感器台架上空真"而被撤回的判据，在这里用作它本来就该是的那道闸："交接后 20 ms 内角度误差 ≤ 0.05 rad"。
constexpr float kSopGateRad = 0.05F;

/** @brief [en] Wrap to (-pi, pi]. [zh] 折到 (-pi, pi]。 */
inline auto sop_wrap_pi(float angle) noexcept -> float {
    while (angle <= -kSopPi) {
        angle += 2.0F * kSopPi;
    }
    while (angle > kSopPi) {
        angle -= 2.0F * kSopPi;
    }
    return angle;
}

/**
 * @brief [en] The machine under test: one nameplate, optionally with a stronger magnet.
 * @brief [zh] 被测机器：一份铭牌，可以选择换一块更强的磁钢。
 * @param psi_scale [en] multiplier on the magnet's strength, **applied to plant and model alike** -- the second machine
 *                       is correctly described, so the replay compares two machines and not two descriptions.
 * @param psi_scale [zh] 磁钢强度的倍数， **plant 与模型同时缩放** ——第二台机器是被正确描述的，于是那次重放比较的是两台
 *                       机器而不是两份描述。
 * @since 0.1.9
 */
struct SopMachine final {
    float psi_scale{1.0F};
};

/** @brief [en] The plant these numbers describe. [zh] 这些数字描述的 plant。 */
inline auto sop_plant_params(const SopMachine& machine) -> PlantParams {
    return PlantParams{kSopRs, kSopLd, kSopLq, kSopKt * machine.psi_scale, kSopJ, kSopB, kSopPoles};
}

/** @brief [en] The same machine as the drive is told about it. [zh] 驱动器被告知的同一台机器。 */
inline auto sop_motor_config(const SopMachine& machine) -> motor::MotorConfig {
    return motor::MotorConfig{kSopRs,     kSopLd, kSopLq,    kSopKt * machine.psi_scale,
                              kSopJ,      kSopB,  kSopPoles, 3.0F,
                              kSopCtrlHz, 0.0F,   0.0F,      0.0F};
}

/**
 * @brief [en] Only the model observer is a candidate: the injection member's gain is computed from `lq - ld`, which is
 *             zero here, so `predicted_gain_a` returns 0 and the carrier would carry no position information.
 * @brief [zh] 只把模型观测器作为候选：注入成员的增益由 `lq − ld` 算出，这里为零，于是 `predicted_gain_a` 返回 0、载波不
 *             携带任何位置信息。
 */
inline auto sop_observer_config(const SopMachine& machine) -> core::SwitchableObserver::Config {
    const motor::MotorConfig cfg = sop_motor_config(machine);
    const float psi_f = foc::psi_f_wb(cfg);
    core::SwitchableObserver::Config config{};
    config.back_emf = core::BackEmfObserver::Config{cfg.rs, cfg.ld};
    config.nonlinear_flux = core::NonlinearFluxObserver::Config{cfg.rs, cfg.lq, psi_f, 0.3F, 3.0F, 0.5F * psi_f};
    config.hfi.inject_v = 12.0F;
    config.hfi.inject_hz = 2000.0F;
    config.hfi.current_hp_hz = 300.0F;
    config.hfi.demod_lpf_hz = 200.0F;
    config.hfi.pll_bandwidth_hz = 40.0F;
    config.hfi.pll_zeta = 0.9F;
    config.hfi.demod_gain_a =
        core::HfiObserver::predicted_gain_a(config.hfi.inject_v, config.hfi.inject_hz, kSopLd, kSopLq);
    config.hfi.min_hf_current_a = 0.05F;
    config.dual_inductance = core::DualInductanceObserver::Config{cfg.rs, cfg.ld, cfg.lq, psi_f};
    config.kind = core::ObserverKind::NonlinearFlux;
    return config;
}

/** @brief [en] One speed's worth of measurement. [zh] 一个转速下的测量结果。 */
struct SensorlessRow final {
    float target_rad_s{0.0F};
    float max_error_rad{0.0F};    ///< [en] the worst error over the settled window. [zh] 稳态窗口内的最坏误差。
    float end_error_rad{0.0F};    ///< [en] the error on the last tick. [zh] 最后一拍的误差。
    float true_speed_rad_s{0.0F}; ///< [en] the plant's own speed at the end. [zh] 结束时 plant 自己的转速。
    bool faulted{false};
    float load_nm{0.0F};
};

/**
 * @brief [en] Run one speed: the rotor is already turning, the sensor is already gone, the observer starts blind.
 * @brief [zh] 跑一个转速：转子已经在转、传感器已经不在了、观测器从零开始。
 * @param target_rad_s [en] the speed under test.
 * @param target_rad_s [zh] 被测转速。
 * @param load_nm [en] constant load torque.
 * @param load_nm [zh] 恒定负载转矩。
 * @param dead_v [en] per-tick dead-time voltage, V; 0 for an ideal inverter.
 * @param dead_v [zh] 每拍死区电压，V；0 表示理想逆变。
 * @param machine [en] which machine.
 * @param machine [zh] 哪台机器。
 * @param quiet [en] false to suppress the per-run line.
 * @param quiet [zh] 为假时不打每行。
 * @return [en] the measured row.
 * @return [zh] 测得的一行。
 * @note [en] 1.5 s at 20 kHz; the **last 0.5 s** is the window, i.e. "has it settled", not the acquisition transient.
 *            The observer starts from a zero seed -- maximally wrong -- which is the honest worst case.
 * @note [zh] 20 kHz 下跑 1.5 s， **最后 0.5 s** 是窗口，即"稳下来了吗"，不是捕获瞬态。观测器从零种子开始——最大程度地错
 *            ——这是诚实的下界。
 * @since 0.1.9
 */
inline auto run_sensorless(float target_rad_s, float load_nm, float dead_v, const SopMachine& machine,
                           bool quiet = false) -> SensorlessRow {
    using MotorT = motor::Motor<core::SwitchableObserver, core::Svpwm, core::Pi, PlantHal>;
    SensorlessRow row{};
    row.target_rad_s = target_rad_s;
    row.load_nm = load_nm;

    PmsmPlant plant(sop_plant_params(machine));
    PlantHal hal(plant, kSopVBus);
    // The run-up: the rotor is already at the speed under test, which is the state a real sensorless drive hands over
    // into. `hal.reset` also puts the inverter back to its power-on state.
    // 升速：转子已经在被测转速上，这正是真实无感驱动器接手时的状态。`hal.reset` 还会把逆变器复位到上电态。
    hal.reset(PlantState{0.0F, 0.0F, static_cast<double>(target_rad_s), 0.0});
    hal.position.set_sensor_present(false);
    plant.set_load_torque(load_nm);
    hal.set_deadtime_voltage(dead_v);

    const motor::MotorConfig cfg = sop_motor_config(machine);
    MotorT motor(cfg,
                 MotorT::Policies{core::SwitchableObserver{sop_observer_config(machine)}, core::Svpwm{},
                                  core::Pi{foc::current_pi_config(cfg, 2000.0F, kSopVBus)},
                                  core::Pi{foc::current_pi_config(cfg, 2000.0F, kSopVBus)},
                                  core::Pi{foc::speed_pi_config(cfg, 200.0F)}},
                 hal);
    if (motor.init() != foc::ErrorCode::Ok) {
        return row;
    }
    if (motor.start() != foc::ErrorCode::Ok) {
        return row;
    }
    motor.set_speed(target_rad_s);

    auto true_angle = [&plant]() -> float { return static_cast<float>(kSopPoles) * plant.theta_mech_rad(); };
    auto error = [&motor, &true_angle]() -> float {
        return std::fabs(sop_wrap_pi(static_cast<float>(motor.observer().state().theta_e - true_angle())));
    };

    const int total = static_cast<int>(1.5F / kSopDt);
    const int settled_from = static_cast<int>(1.0F / kSopDt); // the last 0.5 s is the window 最后 0.5 s 是窗口
    for (int i = 0; i < total; ++i) {
        static_cast<void>(motor.tick(kSopDt));
        hal.advance(kSopDt, kSopSubsteps);
        if (motor.state() == core::MotorState::Fault) {
            row.faulted = true;
            break;
        }
        const float magnitude = error();
        if (i >= settled_from) {
            row.max_error_rad = std::max(row.max_error_rad, magnitude);
        }
        row.end_error_rad = magnitude;
    }
    row.true_speed_rad_s = plant.omega_mech_rad_s();
    if (!quiet) {
        std::printf("[f19] psi x%.2f dead %.2f V: target %6.1f rad/s load %5.3f Nm -> max %.4f rad, end %.4f rad, "
                    "true w %7.2f, faulted %d\n",
                    static_cast<double>(machine.psi_scale), static_cast<double>(dead_v),
                    static_cast<double>(target_rad_s), static_cast<double>(load_nm),
                    static_cast<double>(row.max_error_rad), static_cast<double>(row.end_error_rad),
                    static_cast<double>(row.true_speed_rad_s), row.faulted ? 1 : 0);
    }
    return row;
}

} // namespace sil

} // namespace foc
