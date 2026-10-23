// Conan::ImportStart
#pragma once

#include <cstdint>
#include <utility>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/limits.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_pmsm_plant.hpp"
#include "sil_plant_hal.hpp"
// Conan::ImportEnd

namespace foc {

namespace sil {



/**
 * @brief [en] Machine under test: one table feeds both the plant and the controller nameplate, so
 *        the two can never drift apart. S5 then perturbs the plant only.
 *        [zh] 被控电机：同一张表同时生成 plant 与控制器铭牌，两者不会走偏。S5 只摄动 plant。
 *
 * @since 0.1.0
 */
struct MachineSpec final {
    float rs;                ///< [en] stator resistance in ohm. [zh] 定子电阻，Ω。
    float ld;                ///< [en] d-axis inductance in H. [zh] d 轴电感，H。
    float lq;                ///< [en] q-axis inductance in H. [zh] q 轴电感，H。
    float kt;                ///< [en] torque constant in N*m/A. [zh] 转矩常数，N·m/A。
    float j;                 ///< [en] rotor inertia in kg*m^2. [zh] 转子惯量，kg·m²。
    float b;                 ///< [en] viscous friction in N*m*s. [zh] 粘性摩擦，N·m·s。
    std::uint8_t pole_pairs; ///< [en] pole pairs. [zh] 极对数。
    float i_max_a;           ///< [en] per-axis current limit in A. [zh] 每轴电流限幅，A。
    float pwm_freq_hz;       ///< [en] carrier frequency in Hz. [zh] 载波频率，Hz。
};



/**
 * @brief [en] Bench settings: bus voltage, control period and loop bandwidths.
 *        [zh] 台架设置：母线电压、控制周期与环路带宽。
 *
 * @since 0.1.0
 */
struct BenchConfig final {
    float v_bus_v;             ///< [en] DC link voltage in V. [zh] 母线电压，V。
    float dt_ctrl_s;           ///< [en] control period in s. [zh] 控制周期，s。
    int substeps;              ///< [en] plant sub-steps per control period. [zh] 每控制周期的 plant 子步。
    float omega_current_rad_s; ///< [en] current-loop target bandwidth. [zh] 电流环目标带宽。
    float omega_speed_rad_s;   ///< [en] speed-loop target bandwidth. [zh] 速度环目标带宽。
};



/**
 * @brief [en] One recorded control tick.
 *        [zh] 一个记录下来的控制周期。
 *
 * @since 0.1.0
 */
struct Sample final {
    float t_s;     ///< [en] time since the start of the run. [zh] 运行开始后的时间。
    float speed;   ///< [en] mechanical speed in rad/s. [zh] 机械速度，rad/s。
    float id_a;    ///< [en] measured d-axis current. [zh] 实测 d 轴电流。
    float iq_a;    ///< [en] measured q-axis current. [zh] 实测 q 轴电流。
    float omega_e; ///< [en] electrical speed, diagnostics. [zh] 电角速度，诊断用。
};



/// [en] The nominal machine used by every SIL scenario. [zh] 所有 SIL 场景共用的标称电机。
inline auto nominal_spec() -> MachineSpec {
    // rs     ld      lq      kt     j       b       poles i_max  pwm
    return MachineSpec{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U, 3.0F, 20000.0F};
}



/// [en] Plant parameters matching a spec. [zh] 与铭牌一致的 plant 参数。
inline auto plant_params_from(const MachineSpec& spec) -> PlantParams {
    return PlantParams{spec.rs, spec.ld, spec.lq, spec.kt, spec.j, spec.b, spec.pole_pairs};
}



/// [en] Controller nameplate matching a spec. [zh] 与铭牌一致的控制器视图。
inline auto motor_config_from(const MachineSpec& spec) -> motor::MotorConfig {
    return motor::MotorConfig{spec.rs, spec.ld,         spec.lq,      spec.kt,          spec.j,
                              spec.b,  spec.pole_pairs, spec.i_max_a, spec.pwm_freq_hz, 0.0F};
}



/// [en] Nominal plant parameters. [zh] 标称 plant 参数。
inline auto nominal_plant_params() -> PlantParams { return plant_params_from(nominal_spec()); }



/// [en] Nominal controller nameplate. [zh] 标称控制器铭牌。
inline auto nominal_motor_config() -> motor::MotorConfig { return motor_config_from(nominal_spec()); }



/**
 * @brief [en] Current-loop PI tuned for critical damping: `ki = wn^2*L`, `kp = 2*wn*L - R`,
 *        which cancels the electrical pole and gives `s^2 + 2*wn*s + wn^2`.
 *        [zh] 电流环 PI，按临界阻尼整定：`ki = wn^2*L`、`kp = 2*wn*L - R`，抵消电气极点后得到
 *        `s^2 + 2*wn*s + wn^2`。
 *
 * @since 0.1.0
 */
inline auto current_pi_config(const motor::MotorConfig& mc, float omega_rad_s, float v_bus) -> core::Pi::Config {
    const float kp = core::clamp((2.0F * omega_rad_s * mc.ld) - mc.rs, 0.0F, 1e6F);
    const float ki = omega_rad_s * omega_rad_s * mc.ld;
    const float v_lim = 0.5F * v_bus;
    return core::Pi::Config{kp, ki, -v_lim, v_lim, -v_lim, v_lim};
}



/**
 * @brief [en] Speed-loop PI with pole-zero cancellation of the mechanical pole `B/J`:
 *        `kp = wn*J/Kt`, `ki = wn*B/Kt`.
 *        [zh] 速度环 PI，抵消机械极点 `B/J`：`kp = wn*J/Kt`、`ki = wn*B/Kt`。
 *
 * @since 0.1.0
 */
inline auto speed_pi_config(const motor::MotorConfig& mc, float omega_rad_s) -> core::Pi::Config {
    const float kp = omega_rad_s * mc.j / mc.kt;
    const float ki = omega_rad_s * mc.b / mc.kt;
    const float lim = mc.i_max_a;

    // The PI of this library offers a static integral clamp only, so the integral is bounded to
    // the steady-state current range instead of the full output range: a speed step saturates iq,
    // and an unbounded integral would wind up during the ramp and cost seconds to unwind (S2/S4).
    // The integral still carries the whole steady-state torque, which is a small fraction of i_max.
    const float integral_lim = 0.25F * mc.i_max_a;
    return core::Pi::Config{kp, ki, -lim, lim, -integral_lim, integral_lim};
}



/**
 * @brief [en] Closed-loop bench: plant + HAL adapters + policy-composed motor, ticking at the
 *        control period and pushing every duty into the plant.
 *        [zh] 闭环台架：plant + HAL 适配 + 策略组合电机，按控制周期 tick 并把占空比送入 plant。
 *
 * @since 0.1.0
 */
class Bench final {
  public:
    using MotorT = motor::Motor<core::BackEmfObserver, core::Svpwm, core::Pi, PlantHal>;

    Bench(BenchConfig cfg, PlantParams plant) noexcept
        : cfg_(cfg), plant_(plant), hal_(plant_, cfg.v_bus_v),
          motor_(nominal_motor_config(),
                 MotorT::Policies{
                     core::BackEmfObserver{
                         core::BackEmfObserver::Config{nominal_motor_config().rs, nominal_motor_config().ld}},
                     core::Svpwm{},
                     core::Pi{current_pi_config(nominal_motor_config(), cfg.omega_current_rad_s, cfg.v_bus_v)},
                     core::Pi{current_pi_config(nominal_motor_config(), cfg.omega_current_rad_s, cfg.v_bus_v)},
                     core::Pi{speed_pi_config(nominal_motor_config(), cfg.omega_speed_rad_s)}},
                 hal_),
          elapsed_s_(0.0F) {}

    explicit Bench(BenchConfig cfg) noexcept : Bench(cfg, nominal_plant_params()) {}

    Bench(const Bench&) = delete;
    Bench& operator=(const Bench&) = delete;
    Bench(Bench&&) = delete;
    Bench& operator=(Bench&&) = delete;

    /// [en] init() + start(); returns the first error. [zh] init() + start()，返回首个错误。
    auto init_and_start() noexcept -> ErrorCode {
        const ErrorCode rc = motor_.init();
        if (rc != ErrorCode::Ok) {
            return rc;
        }
        return motor_.start();
    }

    /// [en] One control period: controller tick, then push the duty into the plant. [zh] 一个控制周期。
    void tick_once() noexcept {
        static_cast<void>(motor_.tick(cfg_.dt_ctrl_s));
        hal_.advance(cfg_.dt_ctrl_s, cfg_.substeps);
        elapsed_s_ += cfg_.dt_ctrl_s;
    }

    /// [en] Run for `seconds`, sampling after every tick. [zh] 运行 `seconds` 秒，每 tick 采样一次。
    template <typename OnTick> void run(float seconds, OnTick&& on_tick) {
        const int steps = static_cast<int>((seconds / cfg_.dt_ctrl_s) + 0.5F);
        for (int i = 0; i < steps; ++i) {
            tick_once();
            on_tick(*this);
        }
    }

    void run(float seconds) {
        run(seconds, [](const Bench&) {});
    }

    [[nodiscard]] auto sample() const noexcept -> Sample {
        const motor::Telemetry tm = motor_.telemetry();
        return Sample{elapsed_s_, tm.speed_mech_rad_s, tm.id_a, tm.iq_a,
                      static_cast<float>(nominal_spec().pole_pairs) * tm.speed_mech_rad_s};
    }

    [[nodiscard]] auto config() const noexcept -> const BenchConfig& { return cfg_; }

    [[nodiscard]] auto plant() noexcept -> PmsmPlant& { return plant_; }

    [[nodiscard]] auto hal() noexcept -> PlantHal& { return hal_; }

    [[nodiscard]] auto motor() noexcept -> MotorT& { return motor_; }

    [[nodiscard]] auto elapsed_s() const noexcept -> float { return elapsed_s_; }

  private:
    BenchConfig cfg_;
    PmsmPlant plant_;
    PlantHal hal_;
    MotorT motor_;
    float elapsed_s_;
};



/// [en] Standard bench settings: 24 V bus, 20 kHz, 2 krad/s current loop, 250 rad/s speed loop.
/// [zh] 标准台架设置：24 V 母线、20 kHz、电流环 2 krad/s、速度环 250 rad/s。
inline auto standard_bench_config() -> BenchConfig { return BenchConfig{24.0F, 5e-5F, 4, 2000.0F, 250.0F}; }

} // namespace sil

} // namespace foc
