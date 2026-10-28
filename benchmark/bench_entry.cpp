/**
 * bench_entry.cpp  –  Algorithm Benchmark Entry
 *
 *     ============================================================ HOW TO USE (algorithm engineer)
 *
 *     ============================================================
 *
 *     1. Set MODULE_NAME to identify this build (e.g. "MyAlgo-v2.0").
 *
 *     2. Include your algorithm header(s) below.
 *
 *     3. Add static wrapper functions (one per test item).
 *
 *     4. Add entries to bench_table[].
 *
 *     5. Build with: conan build . -pr:h profiles/<target>.profile
 *
 *     Everything else (linker script, descriptor, RAM init, timing) is handled automatically – do NOT modify below the
 *     separator. ============================================================
 *
 *     What is measured here: the hot paths of this library, one case each. Every case does exactly ONE unit of work per
 *     iteration, so `reported microseconds / repeat` is the cost of one call.
 *
 *     HONEST BOUNDARY: a host (x86_64 Linux) run gives a *relative* cost per operation, not an MCU worst-case execution
 *     time. The target numbers (PRD U6/U7) still require the board, a cycle counter, and the real toolchain — see
 *     benchmark/docs/benchmark-usage.md. 诚实边界：宿主机（x86_64 Linux）跑出来的是**相对**开销，不是 MCU 的 WCET；
 *     U6/U7 仍需上板。
 */
#include <stdint.h>
#include <cmath>
#include "core/het_bench_core.h"

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>


/* ============================================================
 * USER ZONE (algorithm engineer edits only this block)
 * ============================================================ */

constexpr const char *MODULE_NAME = "foc-hot-paths";

/// Control period used by the tick() case; the same 100 µs the SIL bench uses.
constexpr float kDt = 1e-4F;
/// DC link voltage for the SVPWM case.
constexpr float kVBus = 24.0F;

namespace {

using Motor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi,
                                foc::driver::StubHal>;

auto make_config() -> foc::motor::MotorConfig {
    // rs, ld, lq, kt, j, b, pole_pairs, i_max, pwm_freq, deadtime
    return foc::motor::MotorConfig{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-5F, 1e-6F, 7U, 5.0F, 20000.0F, 500.0F};
}

/// Everything the cases share, plus the fixed inputs they chew on.
struct Bench final {
    Bench()
        : hal(),
          motor(make_config(),
                Motor::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{},
                                foc::core::Pi{foc::current_pi_config(make_config(), 2000.0F, kVBus)},
                                foc::core::Pi{foc::current_pi_config(make_config(), 2000.0F, kVBus)},
                                foc::core::Pi{foc::speed_pi_config(make_config(), 200.0F)}},
                hal),
          pi(foc::current_pi_config(make_config(), 2000.0F, kVBus)),
          observer({0.5F, 4e-4F}) {}

    foc::driver::StubHal hal;
    Motor motor;
    foc::core::Pi pi;
    foc::core::BackEmfObserver observer;

    foc::core::Uvw<float> uvw{0.6F, -0.3F, -0.3F};
    foc::core::Ab<float> i_ab{0.5F, 0.2F};
    foc::core::Ab<float> v_ab{3.0F, -1.5F};
    foc::core::Dq<float> dq{0.4F, 0.7F};
    float theta_rad = 0.7F;

    /// Keeps the optimizer from deleting the work, and carries PI feedback between calls.
    volatile float sink = 0.0F;
};

/// Function-local static: no cross-translation-unit state pollution.
auto bench_state() -> Bench & {
    static Bench instance;
    return instance;
}

} // namespace

static void bench_prepare_input(void) {
    Bench &b = bench_state();

    b.hal.reset();
    b.hal.dc_link.set_voltage_v(kVBus);
    b.hal.position.set_sensor_present(true);
    b.hal.position.set_angle_mech_rad(0.0F);
    b.hal.position.set_velocity_rad_s(0.0F);
    const foc::core::Uvw<float> zero{};
    (void)b.hal.current_sense.load(&zero, 1U);

    (void)b.motor.init();
    (void)b.motor.start();
    b.motor.set_torque(1.0F); // a real, non-zero reference: the loop must actually compute
}

/* ---- cases: one unit of work each --------------------------------------------------------- */

/// The ISR path: observer + both current PIs + SVPWM, exactly what runs per PWM period.
static int bench_tick_case(const void * const ctx) // NOSONAR: opaque ctx required by the pFunCase C ABI.
{
    (void)ctx;
    const foc::ErrorCode rc = bench_state().motor.tick(kDt);
    return rc == foc::ErrorCode::Ok;
}

/// Forward transforms: Clarke then Park.
static int bench_clarke_park_case(const void * const ctx) // NOSONAR: opaque ctx required by the C ABI.
{
    (void)ctx;
    Bench &b = bench_state();
    const foc::core::Ab<float> ab = foc::core::clarke(b.uvw);
    const foc::core::Dq<float> dq = foc::core::park(ab, b.theta_rad);
    b.sink = dq.d + dq.q;
    return std::isfinite(b.sink);
}

/// Inverse transforms: inverse Park then inverse Clarke.
static int bench_inv_park_clarke_case(const void * const ctx) // NOSONAR: opaque ctx required by the C ABI.
{
    (void)ctx;
    Bench &b = bench_state();
    const foc::core::Ab<float> ab = foc::core::inv_park(b.dq, b.theta_rad);
    const foc::core::Uvw<float> uvw = foc::core::inv_clarke(ab);
    b.sink = uvw.u + uvw.v + uvw.w;
    return std::isfinite(b.sink);
}

/// Space-vector modulation including the mid-point (min-max) injection.
static int bench_svpwm_case(const void * const ctx) // NOSONAR: opaque ctx required by the C ABI.
{
    (void)ctx;
    Bench &b = bench_state();
    const foc::core::SvpwmResult result = foc::core::Svpwm::modulate(b.v_ab, kVBus);
    b.sink = result.duty.u + result.duty.v + result.duty.w;
    return (result.status == foc::ErrorCode::Ok) && std::isfinite(b.sink);
}

/// One PI step in closed loop (the output feeds the next iteration's feedback), so the case
/// exercises the normal path instead of sitting on a clamp.
static int bench_pi_case(const void * const ctx) // NOSONAR: opaque ctx required by the C ABI.
{
    (void)ctx;
    Bench &b = bench_state();
    b.sink = b.pi.step(1.0F, b.sink, kDt);
    return std::isfinite(b.sink);
}

/// Back-EMF observer update (the slowest block of the ISR path, per the tick() case).
static int bench_observer_case(const void * const ctx) // NOSONAR: opaque ctx required by the C ABI.
{
    (void)ctx;
    Bench &b = bench_state();
    const foc::core::ObsState state = b.observer.update(b.v_ab, b.i_ab, kDt);
    b.sink = state.theta_e + state.omega_e;
    return std::isfinite(b.sink);
}

static const Case bench_table[] = { // NOSONAR: must stay a C array - the macro consumes it as a C aggregate.
    BENCHMARK_CASE_IMPLEMENTATION("tick_full_loop_dt100us", nullptr, bench_tick_case, 100000U),
    BENCHMARK_CASE_IMPLEMENTATION("clarke_park", nullptr, bench_clarke_park_case, 2000000U),
    BENCHMARK_CASE_IMPLEMENTATION("inv_park_clarke", nullptr, bench_inv_park_clarke_case, 2000000U),
    BENCHMARK_CASE_IMPLEMENTATION("svpwm_modulate", nullptr, bench_svpwm_case, 500000U),
    BENCHMARK_CASE_IMPLEMENTATION("pi_step", nullptr, bench_pi_case, 3000000U),
    BENCHMARK_CASE_IMPLEMENTATION("observer_update", nullptr, bench_observer_case, 700000U),
};


BENCHMARK_IMPLEMENTATION(MODULE_NAME, bench_table);
