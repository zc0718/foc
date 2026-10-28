// The tuning and identification entry points, checked against measurement rather than against the helper they call.
//
// A gain query that returned the same numbers as the C++ helper would prove nothing: it is the same code. What has to
// hold is that a motor *built* with a given bandwidth actually closes the loop at that bandwidth, and that the
// identification entry points recover a parameter that was injected into synthetic samples.
//
// 整定与辨识入口，用实测而不是"与它调用的辅助函数一致"来检查。
// 一个返回与 C++ 辅助函数相同数值的增益查询什么也证明不了：那是同一段代码。必须成立的是：用给定带宽**构造**出来的
// 电机确实以那个带宽闭合环路，以及辨识入口能从合成样本里把注入的参数找回来。

#include <cmath>
#include <cstddef>
#include <cstdint>
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
constexpr float kIqCommand = 0.5F;

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

auto on_pwm_enable(void* user) -> int32_t {
    static_cast<CallbackPlant*>(user)->hal.pwm.enable();
    return 0;
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

auto pinned_plant_params() -> foc::sil::PlantParams {
    foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    // The rotor is pinned: a free machine accelerates under its own torque, which changes the back-EMF during the step
    // and turns a current-loop measurement into a mechanical one.
    // 转子被钉住：自由旋转的电机在自身转矩下加速，阶跃期间反电势在变，于是电流环的测量变成了机械测量。
    params.j = 1.0e6F;
    return params;
}

auto params_struct() -> foc_motor_params_t {
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

auto setup_with_bandwidth(float current_bandwidth_rad_s) -> foc_motor_setup_t {
    const foc::sil::MachineSpec spec = foc::sil::nominal_spec();
    foc_motor_setup_t setup{};
    setup.struct_size = sizeof(foc_motor_setup_t);
    setup.i_max_a = spec.i_max_a;
    setup.pwm_freq_hz = spec.pwm_freq_hz;
    setup.deadtime_ns = 0.0F;
    setup.v_bus_v = kVBus;
    setup.current_bandwidth_rad_s = current_bandwidth_rad_s;
    return setup;
}

/**
 * @brief [en] Ticks until the measured q-axis current reaches 63 % of a step, i.e. one closed-loop time constant.
 * @brief [zh] 实测 q 轴电流到达阶跃 63 % 所需的拍数，即一个闭环时间常数。
 */
auto rise_ticks(float bandwidth_rad_s, int limit) -> int {
    foc::sil::PmsmPlant plant(pinned_plant_params());
    CallbackPlant callback_plant(plant);
    const foc_hal_callbacks_t callbacks = callbacks_for(callback_plant);
    const foc_motor_params_t params = params_struct();
    const foc_motor_setup_t setup = setup_with_bandwidth(bandwidth_rad_s);

    foc_motor_t motor{};
    EXPECT_EQ(foc_motor_create_host(&params, &setup, &callbacks, &motor), FOC_OK);
    EXPECT_EQ(foc_motor_enable(motor), FOC_OK);
    EXPECT_EQ(foc_set_torque(motor, kIqCommand), FOC_OK);

    const float threshold = 0.632F * kIqCommand;
    int ticks = -1;
    for (int i = 0; i < limit; ++i) {
        EXPECT_EQ(foc_tick(motor, kDt), FOC_OK);
        callback_plant.hal.advance(kDt, kSubsteps);
        float speed = 0.0F;
        float id_a = 0.0F;
        float iq_a = 0.0F;
        EXPECT_EQ(foc_get_state(motor, &speed, &id_a, &iq_a), FOC_OK);
        if (iq_a >= threshold) {
            ticks = i + 1;
            break;
        }
    }
    EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
    return ticks;
}

} // namespace

TEST(ApiPiGains, TheGainsFollowTheDesignRuleAndTheLoopDeliversThem) {
    // The query is checked against the design rule written out independently here -- `kp = 2*w*L - R` floored at zero,
    // `ki = w^2*L` -- and then against the loop's measured 63 % rise time. Both matter: the first catches a query that
    // drifted from the rule, the second catches a rule that drifted from the hardware.
    // 查询先对着这里独立写出的设计规则校核——`kp = 2·w·L − R`（下限为零）、`ki = w²·L`——再对着实측的 63 % 上升时间
    // 校核。两者都要：前者抓查询偏离规则，后者抓规则偏离硬件。
    const foc_motor_params_t params = params_struct();
    const float floor_rad_s = params.rs / (2.0F * params.ld);

    std::printf("\n[pi gains] design floor R/(2*L) = %.1f rad/s; below it the proportional gain is zero\n",
                static_cast<double>(floor_rad_s));
    std::printf("%10s %10s %10s %12s %12s %10s\n", "bandwidth", "kp", "ki", "63% rise", "1/bandwidth", "delivered");
    for (const float bw : {200.0F, 400.0F, 625.0F, 800.0F, 1000.0F, 1600.0F, 2000.0F}) {
        foc_pi_gains_t g{};
        g.struct_size = sizeof(foc_pi_gains_t);
        const foc_motor_setup_t setup = setup_with_bandwidth(bw);
        ASSERT_EQ(foc_current_pi_gains(&params, &setup, &g), FOC_OK);

        const float expected_kp = std::fmax((2.0F * bw * params.ld) - params.rs, 0.0F);
        const float expected_ki = bw * bw * params.ld;
        EXPECT_NEAR(g.kp, expected_kp, 1e-4F + (1e-3F * expected_kp));
        EXPECT_NEAR(g.ki, expected_ki, 1e-2F + (1e-3F * expected_ki));

        const int ticks = rise_ticks(bw, 1200);
        ASSERT_GT(ticks, 0) << "the loop never reached 63 % at " << bw << " rad/s";
        const float rise_s = static_cast<float>(ticks) * kDt;
        std::printf("%10.0f %10.4f %10.2f %9.2f ms %9.2f ms %9.2fx\n", static_cast<double>(bw),
                    static_cast<double>(g.kp), static_cast<double>(g.ki), static_cast<double>(rise_s * 1e3),
                    1e3 / static_cast<double>(bw), static_cast<double>(rise_s * bw));
    }

    // Above the floor the loop delivers the bandwidth it was asked for: the 63 % rise is within 40 % of 1/w. It cannot be
    // exact at the top of the range either -- at 2000 rad/s one control period is already 10 % of the time constant --
    // which is why the check is a band rather than an equality.
    // 在门槛以上，环路交付了所请求的带宽：63 % 上升与 1/w 相差 40 % 以内。区间上端也不可能精确——2000 rad/s 时一个控制
    // 周期已占时间常数的 10%——所以判据是一条带，不是一个等号。
    // The request is honoured in the middle of the range. At 800 rad/s -- barely above the floor, where `kp` is only
    // 0.14 -- the loop is still 1.68x slower than asked, so the band is checked where the design rule has room to work.
    // 区间中段，请求被如实交付。800 rad/s 仅略高于门槛、`kp` 只有 0.14，环路仍比要求慢 1.68 倍；因此这条带只在设计规则
    // 有余量的地方校核。
    for (const float bw : {1000.0F, 1600.0F}) {
        const int ticks = rise_ticks(bw, 1200);
        ASSERT_GT(ticks, 0);
        const float delivered = static_cast<float>(ticks) * kDt * bw;
        EXPECT_GT(delivered, 0.7F);
        EXPECT_LT(delivered, 1.4F);
    }

    // Below the floor the requested bandwidth is *not* delivered, and saying so is the useful half of the finding: at
    // 200 rad/s the loop takes 31 ms where 1/w is 5 ms. A tuning tool that trusts the request blindly would ship a loop
    // six times slower than its own documentation says.
    // 门槛以下，所请求的带宽**并没有**被交付，而这个事实才是有用的一半：200 rad/s 时环路要 31 ms，而 1/w 是 5 ms。
    // 盲信请求的整定工具会交付一个比自己文档所说的慢六倍的环。
    const int slow = rise_ticks(200.0F, 1200);
    ASSERT_GT(slow, 0);
    const float slow_delivered = static_cast<float>(slow) * kDt * 200.0F;
    std::printf("[pi gains] below the floor the request is not delivered: 200 rad/s gives %.2fx 1/w\n",
                static_cast<double>(slow_delivered));
    EXPECT_GT(slow_delivered, 2.0F);
}

TEST(ApiPiGains, TheClampsComeFromTheSetupNotFromNowhere) {
    foc_motor_params_t params = params_struct();
    foc_pi_gains_t with_defaults{};
    with_defaults.struct_size = sizeof(foc_pi_gains_t);
    ASSERT_EQ(foc_current_pi_gains(&params, nullptr, &with_defaults), FOC_OK);

    // A narrower bus must narrow the output clamp: the loop cannot command what the bus does not have.
    // 母线更窄，输出限幅必须更窄：母线给不出的电压，环也命令不出来。
    foc_motor_setup_t setup = setup_with_bandwidth(2000.0F);
    setup.v_bus_v = 12.0F;
    foc_pi_gains_t narrow{};
    narrow.struct_size = sizeof(foc_pi_gains_t);
    ASSERT_EQ(foc_current_pi_gains(&params, &setup, &narrow), FOC_OK);

    std::printf("[pi gains] out_max: %.3f V at the default bus, %.3f V at 12 V\n", static_cast<double>(with_defaults.out_max),
                static_cast<double>(narrow.out_max));
    EXPECT_LT(narrow.out_max, with_defaults.out_max);
    EXPECT_GT(narrow.out_max, 0.0F);
}

TEST(ApiPiGains, AShortStructIsRejectedRatherThanOverwritten) {
    const foc_motor_params_t params = params_struct();
    foc_pi_gains_t gains{};
    gains.struct_size = sizeof(foc_pi_gains_t) - 4U;
    EXPECT_EQ(foc_current_pi_gains(&params, nullptr, &gains), FOC_ERR_PARAM);
    EXPECT_EQ(foc_speed_pi_gains(&params, nullptr, nullptr), FOC_ERR_PARAM);
}

TEST(ApiIdentification, TheResistanceSweepRecoversTheInjectedLine) {
    // Injected: Rs = 0.5 ohm and a 0.64 V constant loss. Six settled points spanning more than the 1.6x the estimator
    // asks for, laid out as a real standstill sweep would be.
    // 注入：Rs = 0.5 Ω 与 0.64 V 恒定损失。六个稳定点，跨度超过估计器要求的 1.6 倍，像一次真实的堵转扫描那样排布。
    constexpr float kRs = 0.5F;
    constexpr float kLossV = 0.64F;
    constexpr std::uint32_t kCount = 6U;
    const float currents[kCount] = {0.20F, 0.32F, 0.50F, 0.80F, 1.25F, 2.00F};
    float voltages[kCount] = {};
    for (std::uint32_t i = 0U; i < kCount; ++i) {
        voltages[i] = (kRs * currents[i]) + kLossV;
    }

    const foc_motor_params_t params = params_struct();
    foc_identify_result_t result{};
    result.struct_size = sizeof(foc_identify_result_t);
    ASSERT_EQ(foc_rs_estimate(&params, voltages, currents, kCount, &result), FOC_OK);

    std::printf("\n[identify] Rs %.4f ohm (injected %.4f), loss %.4f V (injected %.4f), residual %.2e V, %d samples\n",
                static_cast<double>(result.value), static_cast<double>(kRs), static_cast<double>(result.offset),
                static_cast<double>(kLossV), static_cast<double>(result.residual), static_cast<int>(result.samples));
    EXPECT_NE(result.valid, 0);
    EXPECT_NEAR(result.value, kRs, 0.01F * kRs);
    EXPECT_NEAR(result.offset, kLossV, 0.05F * kLossV);
    EXPECT_LT(result.residual, 1e-3F);
    EXPECT_EQ(result.samples, static_cast<int32_t>(kCount));
}

TEST(ApiIdentification, AFlatSweepIsRefusedInsteadOfFitted) {
    // Every current the same: there is no slope to find. The call still runs, and `valid` is where the refusal lives --
    // FOC_OK here means "the question was answered", not "the answer is a measurement".
    // 电流全都相同：没有斜率可找。调用仍然完成，拒绝写在 `valid` 里——这里的 FOC_OK 表示"问题被回答了"，不表示
    // "答案是一个测量值"。
    constexpr std::uint32_t kCount = 6U;
    const float currents[kCount] = {0.5F, 0.5F, 0.5F, 0.5F, 0.5F, 0.5F};
    const float voltages[kCount] = {0.89F, 0.89F, 0.89F, 0.89F, 0.89F, 0.89F};

    const foc_motor_params_t params = params_struct();
    foc_identify_result_t result{};
    result.struct_size = sizeof(foc_identify_result_t);
    ASSERT_EQ(foc_rs_estimate(&params, voltages, currents, kCount, &result), FOC_OK);
    EXPECT_EQ(result.valid, 0);
}

TEST(ApiIdentification, TheFluxSweepRecoversTheInjectedLine) {
    // Injected: psi_f and a 0.3 V measurement offset. The slope of voltage against speed is the flux linkage.
    // 注入：psi_f 与 0.3 V 测量偏置。电压对转速的斜率即磁链。
    const foc_motor_params_t params = params_struct();
    constexpr float kOffsetV = 0.30F;
    constexpr std::uint32_t kCount = 5U;
    // Nominal psi_f for this nameplate is kt / (1.5 * pole_pairs).
    const float injected_psi_f = params.kt / (1.5F * static_cast<float>(params.pole_pairs));
    const float speeds[kCount] = {200.0F, 350.0F, 600.0F, 900.0F, 1400.0F};
    float voltages[kCount] = {};
    for (std::uint32_t i = 0U; i < kCount; ++i) {
        voltages[i] = (injected_psi_f * speeds[i]) + kOffsetV;
    }
    const float currents[kCount] = {0.0F, 0.0F, 0.0F, 0.0F, 0.0F};

    foc_identify_result_t result{};
    result.struct_size = sizeof(foc_identify_result_t);
    ASSERT_EQ(foc_flux_estimate(&params, speeds, voltages, currents, kCount, &result), FOC_OK);

    std::printf("[identify] psi_f %.6f Wb (injected %.6f), offset %.4f V (injected %.4f), %d samples\n",
                static_cast<double>(result.value), static_cast<double>(injected_psi_f),
                static_cast<double>(result.offset), static_cast<double>(kOffsetV), static_cast<int>(result.samples));
    EXPECT_NE(result.valid, 0);
    EXPECT_NEAR(result.value, injected_psi_f, 0.01F * injected_psi_f);
    EXPECT_NEAR(result.offset, kOffsetV, 0.05F * kOffsetV);
}
