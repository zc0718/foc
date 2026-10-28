// Third leg of the three-way reconciliation: the same control loop reached two ways.
//
// `sil::Bench` runs the motor compiled into the test, with `PlantHal` as the hardware. `foc_motor_create_host()` runs
// the motor behind the C ABI, with the *same* HAL wrapped in `foc_hal_callbacks_t`. Nothing else differs: same policy
// list, same PI tuning helper, same plant, same command, same schedule. If the two traces agree, the ABI and the
// callback seam are transparent -- and if they ever stop agreeing, this test says so before a board does.
//
// 三方对照的第三腿：同一个控制环经两条路径到达。
// `sil::Bench` 跑编译进测试的电机，硬件是 `PlantHal`；`foc_motor_create_host()` 跑 C ABI 背后的电机，同一个 HAL 被
// 包进 `foc_hal_callbacks_t`。其余全都一样：同一套策略、同一个 PI 整定函数、同一个 plant、同一个给定、同一个时序。
// 两条曲线一致，就说明 ABI 与回调接缝是透明的；哪天不一致了，这条测试会先于板子说出来。

#include <cmath>
#include <cstdio>
#include <vector>

#include <gtest/gtest.h>

#include <foc/api/foc_api.h>
#include <foc/api/foc_types.h>
#include <foc/core/types.hpp>

#include "sil_bench.hpp"
#include "sil_pmsm_plant.hpp"
#include "sil_plant_hal.hpp"

namespace {

constexpr float kDt = 5e-5F;
constexpr int kSubsteps = 4;
constexpr float kVBus = 24.0F;
constexpr int kTicks = 4000;
constexpr float kTorqueA = 0.5F;

/**
 * @brief [en] One trace of the controller's own view, sampled after every tick.
 * @brief [zh] 控制器自身视角的一条曲线，每拍采样。
 */
struct Trace final {
    std::vector<float> id_a;
    std::vector<float> iq_a;
    std::vector<float> speed_rad_s;
};

/**
 * @brief [en] Leg 1: the motor compiled in, hardware through `PlantHal`.
 * @brief [zh] 第一腿：电机编译进测试，硬件走 `PlantHal`。
 */
auto run_compiled_in() -> Trace {
    foc::sil::Bench bench(foc::sil::standard_bench_config(), foc::sil::nominal_plant_params());
    bench.motor().set_torque(kTorqueA);
    EXPECT_EQ(bench.init_and_start(), foc::ErrorCode::Ok);

    Trace trace;
    bench.run(kDt * static_cast<float>(kTicks), [&trace](const foc::sil::Bench& b) {
        const foc::sil::Sample sample = b.sample();
        trace.id_a.push_back(sample.id_a);
        trace.iq_a.push_back(sample.iq_a);
        trace.speed_rad_s.push_back(sample.speed);
    });
    return trace;
}

/**
 * @brief [en] The plant handed to the C ABI as function pointers.
 * @brief [zh] 以函数指针形式交�� C ABI 的 plant。
 */
struct CallbackPlant final {
    explicit CallbackPlant(foc::sil::PmsmPlant& plant) noexcept : hal(plant, kVBus) {}

    foc::sil::PlantHal hal;
};

auto on_write_duty(void* user, const float duty_uvw[3]) -> void {
    static_cast<CallbackPlant*>(user)->hal.pwm.write_duty(
        foc::core::Uvw<float>{duty_uvw[0], duty_uvw[1], duty_uvw[2]});
}

auto on_read_current(void* user, float uvw_a[3]) -> void {
    const foc::core::Uvw<float> current = static_cast<CallbackPlant*>(user)->hal.current_sense.read_uvw();
    uvw_a[0] = current.u;
    uvw_a[1] = current.v;
    uvw_a[2] = current.w;
}

auto on_read_angle(void* user) -> float { return static_cast<CallbackPlant*>(user)->hal.position.angle_mech_rad(); }

auto on_read_voltage(void* user) -> float { return static_cast<CallbackPlant*>(user)->hal.dc_link.voltage_v(); }

auto on_read_velocity(void* user) -> float {
    return static_cast<CallbackPlant*>(user)->hal.position.velocity_rad_s();
}

// `foc_hal_callbacks_t` marks these two optional, and the library is right to: a host may have no gate to enable. But a
// plant that models the inverter's gate enable *does* has one, and omitting the callbacks leaves `PlantPwm` disabled --
// `advance()` then integrates zero voltage and the loop looks perfectly healthy while producing no torque. The first
// version of this test omitted them and measured exactly that: mode Run, no fault, iq pinned at 0.
// `foc_hal_callbacks_t` 把这两个标为可选，库是对的：主控可以没有可控的栅极。但建模了逆变器使能的 plant 是有
// 的，省掉回调会让 `PlantPwm` 停在未使能态——`advance()` 于是按零电压积分，环看起来完全正常却不出转矩。本测试
// 第一版就省了，量到的正是这个：模式 Run、无故障、iq 恒为 0。
// The C surface declares these two as `void` callbacks, so a host whose gate refuses cannot say so -- the library's
// `pwm.enable()` answers an `ErrorCode` and the ABI drops it. Kept as `void` here rather than widening the ABI for a
// test; the gate model in this harness always accepts.
auto on_pwm_enable(void* user) -> int32_t {
    static_cast<CallbackPlant*>(user)->hal.pwm.enable();
    return 0; // the gate model in this harness always accepts
}

auto on_pwm_disable(void* user) -> void { static_cast<CallbackPlant*>(user)->hal.pwm.disable(); }

auto callbacks_for(CallbackPlant& plant) -> foc_hal_callbacks_t {
    foc_hal_callbacks_t callbacks{};
    callbacks.struct_size = sizeof(foc_hal_callbacks_t);
    callbacks.user = &plant;
    callbacks.pwm_write_duty = &on_write_duty;
    callbacks.read_current = &on_read_current;
    callbacks.read_angle_mech_rad = &on_read_angle;
    callbacks.read_voltage_v = &on_read_voltage;
    callbacks.read_velocity_rad_s = &on_read_velocity;
    callbacks.pwm_enable = &on_pwm_enable;
    callbacks.pwm_disable = &on_pwm_disable;
    return callbacks;
}

auto params_from_spec() -> foc_motor_params_t {
    const foc::sil::MachineSpec spec = foc::sil::nominal_spec();
    foc_motor_params_t params{};
    params.rs = spec.rs;
    params.ld = spec.ld;
    params.lq = spec.lq;
    params.kt = spec.kt;
    params.j = spec.j;
    params.b = spec.b;
    params.pole_pairs = spec.pole_pairs;
    return params;
}

auto setup_matching_the_bench() -> foc_motor_setup_t {
    const foc::sil::MachineSpec spec = foc::sil::nominal_spec();
    const foc::sil::BenchConfig bench = foc::sil::standard_bench_config();
    foc_motor_setup_t setup{};
    setup.struct_size = sizeof(foc_motor_setup_t);
    setup.i_max_a = spec.i_max_a;
    setup.pwm_freq_hz = spec.pwm_freq_hz;
    setup.deadtime_ns = 0.0F; // `PlantHal` models no dead time, so neither may the compensation
    setup.v_bus_v = bench.v_bus_v;
    setup.current_bandwidth_rad_s = bench.omega_current_rad_s;
    setup.speed_bandwidth_rad_s = bench.omega_speed_rad_s;
    return setup;
}

/**
 * @brief [en] Leg 2: the same motor behind the C ABI, driven through `foc_hal_callbacks_t`.
 * @brief [zh] 第二腿：同一个电机在 C ABI 背后，经 `foc_hal_callbacks_t` 驱动。
 */
auto run_through_the_abi() -> Trace {
    foc::sil::PmsmPlant plant(foc::sil::nominal_plant_params());
    CallbackPlant callback_plant(plant);
    const foc_hal_callbacks_t callbacks = callbacks_for(callback_plant);
    const foc_motor_params_t params = params_from_spec();
    const foc_motor_setup_t setup = setup_matching_the_bench();

    foc_motor_t motor{};
    EXPECT_EQ(foc_motor_create_host(&params, &setup, &callbacks, &motor), FOC_OK);
    EXPECT_EQ(foc_motor_enable(motor), FOC_OK);
    EXPECT_EQ(foc_set_torque(motor, kTorqueA), FOC_OK);

    Trace trace;
    for (int i = 0; i < kTicks; ++i) {
        EXPECT_EQ(foc_tick(motor, kDt), FOC_OK);
        callback_plant.hal.advance(kDt, kSubsteps);
        float speed = 0.0F;
        float id_a = 0.0F;
        float iq_a = 0.0F;
        EXPECT_EQ(foc_get_state(motor, &speed, &id_a, &iq_a), FOC_OK);
        trace.speed_rad_s.push_back(speed);
        trace.id_a.push_back(id_a);
        trace.iq_a.push_back(iq_a);
    }
    EXPECT_NE(motor, nullptr);
    EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
    return trace;
}

auto worst_gap(const std::vector<float>& a, const std::vector<float>& b) -> float {
    float worst = 0.0F;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const float gap = std::fabs(a[i] - b[i]);
        worst = (gap > worst) ? gap : worst;
    }
    return worst;
}

} // namespace

TEST(SilApiEquivalence, BothPathsProduceTheSameCurrentAndSpeedTrace) {
    // The criterion, written before the run: the two traces agree to within 1e-4 A on both current axes and
    // 1e-3 rad/s on the speed, at every one of the 4000 samples. The comparison is sample by sample rather than at the
    // end, because a seam that shifts the timing leaves the final values looking right.
    // 判据在跑之前写下：两条曲线在 4000 个采样点上逐点一致，电流两轴误差 ≤ 1e-4 A、速度 ≤ 1e-3 rad/s。
    // 逐点比较而不是只比末值：接缝若只错开了时序，末值仍然会好看。
    const Trace compiled_in = run_compiled_in();
    const Trace abi = run_through_the_abi();

    ASSERT_EQ(compiled_in.iq_a.size(), abi.iq_a.size());
    ASSERT_EQ(static_cast<int>(compiled_in.iq_a.size()), kTicks);

    const float gap_iq = worst_gap(compiled_in.iq_a, abi.iq_a);
    const float gap_id = worst_gap(compiled_in.id_a, abi.id_a);
    const float gap_speed = worst_gap(compiled_in.speed_rad_s, abi.speed_rad_s);

    std::printf("\n[abi equivalence] worst gaps over %d samples: iq %.3e A, id %.3e A, speed %.3e rad/s\n", kTicks,
                static_cast<double>(gap_iq), static_cast<double>(gap_id), static_cast<double>(gap_speed));
    std::printf("[abi equivalence] final iq: compiled-in %+.4f A, ABI %+.4f A\n",
                static_cast<double>(compiled_in.iq_a.back()), static_cast<double>(abi.iq_a.back()));

    EXPECT_LE(gap_iq, 1e-4F);
    EXPECT_LE(gap_id, 1e-4F);
    EXPECT_LE(gap_speed, 1e-3F);
}

TEST(SilApiEquivalence, TheLoopActuallyTracksTheCommandItWasGiven) {
    // The equivalence above would also hold if both paths were equally wrong, so one of them is checked against the
    // command too: a 0.5 A torque command on a free-running machine ends up within 1 % of 0.5 A.
    // 上面的等价性在"两条都错得一样"时也成立，所以其中一条还要对着给定校验一次：自由旋转下 0.5 A 的转矩给定，
    // 末值应当落在 0.5 A 的 1% 以内。
    const Trace trace = run_through_the_abi();
    ASSERT_FALSE(trace.iq_a.empty());

    const float final_iq = trace.iq_a.back();
    std::printf("[abi equivalence] the ABI leg tracks %.4f A of the 0.5 A command\n", static_cast<double>(final_iq));

    EXPECT_NEAR(final_iq, kTorqueA, 0.05F * kTorqueA);
    // A positive torque on an unloaded machine has to speed it up.
    EXPECT_GT(trace.speed_rad_s.back(), 0.0F);
}
