// The tuning and identification entry points, checked against measurement rather than against the helper they call.
//
// A gain query that returned the same numbers as the C++ helper would prove nothing: it is the same code. What has to
// hold is that a motor *built* with a given bandwidth actually closes the loop at that bandwidth, and that the
// identification entry points recover a parameter that was injected into synthetic samples.
//
// 整定与辨识入口，用实测而不是"与它调用的辅助函数一致"来检查。
// 一个返回与 C++ 辅助函数相同数值的增益查询什么也证明不了：那是同一段代码。必须成立的是：用给定带宽 **构造** 出来的
// 电机确实以那个带宽闭合环路，以及辨识入口能从合成样本里把注入的参数找回来。

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include <gtest/gtest.h>

#include <foc/api/foc_api.h>
#include <foc/api/foc_types.h>
#include <foc/comm/commissioned_set.hpp>
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

// The power stage's fault output: this host models a healthy inverter. Required by the library (see
// `foc_hal_callbacks_t::read_drive_fault`), so it is answered here, once, rather than left NULL.
// 功率级的故障输出：本主控方建模的是一个健康逆变器。库要求它在（见 `foc_hal_callbacks_t::read_drive_fault`），
// 所以在这里回答一次，而不是留成 NULL。
auto on_read_drive_fault(void* /*user*/) -> int32_t { return 0; }


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
    callbacks.read_drive_fault = &on_read_drive_fault;
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
    // 门槛以下，所请求的带宽 **并没有** 被交付，而这个事实才是有用的一半：200 rad/s 时环路要 31 ms，而 1/w 是 5 ms。
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

// `foc_motor_get_setup()` had a readback and the machine parameters had not, so a tuning tool could read every loop
// setting and none of the numbers those loops are built from. The cases below cover both halves of the entry point:
// what was handed in comes back, and a refused call is indistinguishable -- tick for tick -- from a call nobody made.
//
// 设置一直有回读，而机器参数没有，于是整定工具能读到每个环路设置，却读不到那些环路是由哪些数字构造的。下面覆盖这个入口
// 的两半：交进去什么就读回什么；被拒的调用与"根本没人调用过"逐拍不可区分。

namespace {

/**
 * @brief [en] One motor on its own plant, so two of them can be ticked in lockstep.
 * @brief [zh] 一台电机配一个自己的被控对象，于是两台可以同步推进。
 */
struct Twin final {
    explicit Twin(const foc::sil::PlantParams& plant_params) : plant(plant_params), callback_plant(plant) {}
    Twin(const Twin&) = delete;
    Twin& operator=(const Twin&) = delete;
    ~Twin() = default;

    foc::sil::PmsmPlant plant;
    CallbackPlant callback_plant;
    foc_motor_t motor{};
};

auto make_twin(Twin& twin, const foc_motor_params_t& params, std::int32_t use_observer_angle = 0) -> foc_err_t {
    const foc_hal_callbacks_t callbacks = callbacks_for(twin.callback_plant);
    foc_motor_setup_t setup = setup_with_bandwidth(2000.0F);
    setup.use_observer_angle = use_observer_angle;
    return foc_motor_create_host(&params, &setup, &callbacks, &twin.motor);
}

auto start_twin(Twin& twin) -> void {
    const foc_err_t enable = foc_motor_enable(twin.motor);
    EXPECT_EQ(enable, FOC_OK);
    const foc_err_t command = foc_set_torque(twin.motor, kIqCommand);
    EXPECT_EQ(command, FOC_OK);
}

auto step_twin(Twin& twin) -> foc_err_t {
    const foc_err_t rc = foc_tick(twin.motor, kDt);
    twin.callback_plant.hal.advance(kDt, kSubsteps);
    return rc;
}

auto state_as_bits(foc_motor_t motor, std::uint32_t out[3]) -> foc_err_t {
    float speed = 0.0F;
    float id_a = 0.0F;
    float iq_a = 0.0F;
    const foc_err_t rc = foc_get_state(motor, &speed, &id_a, &iq_a);
    std::memcpy(&out[0], &speed, sizeof(float));
    std::memcpy(&out[1], &id_a, sizeof(float));
    std::memcpy(&out[2], &iq_a, sizeof(float));
    return rc;
}

/**
 * @brief [en] Tick both, and require the published state to agree bit for bit at every tick.
 * @brief [zh] 同步推进两者，并要求每一拍的已发布状态逐位相同。
 */
auto tick_in_lockstep(Twin& a, Twin& b, int count) -> void {
    for (int i = 0; i < count; ++i) {
        ASSERT_EQ(step_twin(a), FOC_OK);
        ASSERT_EQ(step_twin(b), FOC_OK);
        std::uint32_t sa[3] = {};
        std::uint32_t sb[3] = {};
        ASSERT_EQ(state_as_bits(a.motor, sa), FOC_OK);
        ASSERT_EQ(state_as_bits(b.motor, sb), FOC_OK);
        ASSERT_EQ(sa[0], sb[0]) << "speed differs at tick " << i;
        ASSERT_EQ(sa[1], sb[1]) << "i_d differs at tick " << i;
        ASSERT_EQ(sa[2], sb[2]) << "i_q differs at tick " << i;
    }
}

} // namespace

TEST(ApiParams, TheReadbackIsWhatWasHandedIn) {
    Twin twin{pinned_plant_params()};
    const foc_motor_params_t params = params_struct();
    ASSERT_EQ(make_twin(twin, params), FOC_OK);

    foc_motor_params_t read{};
    ASSERT_EQ(foc_motor_get_params(twin.motor, &read), FOC_OK);
    EXPECT_FLOAT_EQ(read.rs, params.rs);
    EXPECT_FLOAT_EQ(read.ld, params.ld);
    EXPECT_FLOAT_EQ(read.lq, params.lq);
    EXPECT_FLOAT_EQ(read.kt, params.kt);
    EXPECT_FLOAT_EQ(read.j, params.j);
    EXPECT_FLOAT_EQ(read.b, params.b);
    EXPECT_EQ(read.pole_pairs, params.pole_pairs);

    // The answer is the last call, not the construction: that is the half which had no answer at all before.
    // 答案是最后一次调用，而不是构造时——这正是此前完全没有答案的那一半。
    foc_motor_params_t changed = params;
    changed.ld = params.ld * 1.5F;
    changed.kt = params.kt * 0.5F;
    ASSERT_EQ(foc_motor_apply_params(twin.motor, &changed, nullptr), FOC_OK);

    foc_motor_params_t second{};
    ASSERT_EQ(foc_motor_get_params(twin.motor, &second), FOC_OK);
    EXPECT_FLOAT_EQ(second.ld, changed.ld);
    EXPECT_FLOAT_EQ(second.kt, changed.kt);
    EXPECT_FLOAT_EQ(second.lq, params.lq) << "a field that was not handed in must not drift";

    EXPECT_EQ(foc_motor_destroy(twin.motor), FOC_OK);
}

TEST(ApiParams, ARefusedCallIsIndistinguishableFromNoCall) {
    Twin a{pinned_plant_params()};
    Twin b{pinned_plant_params()};
    const foc_motor_params_t params = params_struct();
    ASSERT_EQ(make_twin(a, params), FOC_OK);
    ASSERT_EQ(make_twin(b, params), FOC_OK);
    start_twin(a);
    start_twin(b);
    tick_in_lockstep(a, b, 40);

    // The comparison above is only worth something if the loop is actually delivering current: two motors parked at
    // zero would agree bit for bit for reasons that have nothing to do with the call under test.
    // 上面的比对只有在环路真的在送电流时才有意义：两台停在零上的电机会逐位一致，而那与本次被测的调用无关。
    {
        float speed = 0.0F;
        float id_a = 0.0F;
        float iq_a = 0.0F;
        ASSERT_EQ(foc_get_state(a.motor, &speed, &id_a, &iq_a), FOC_OK);
        std::printf("[api-params] steady q-axis current %.6f A of %.6f A commanded\n",
                    static_cast<double>(iq_a), static_cast<double>(kIqCommand));
        ASSERT_GT(std::fabs(iq_a), 0.1F) << "the loop is not tracking, so bit-identity proves nothing";
    }

    // Two ways to be refused, each with its own answer: a field the whole-set validation rejects, and a setup the
    // running object cannot absorb at all.
    // 两种被拒的方式各有各的回答：整组校验拒绝的字段，以及运行中的对象根本无法吸收的设置。
    foc_motor_params_t bad = params;
    bad.ld = 0.0F;
    EXPECT_EQ(foc_motor_apply_params(a.motor, &bad, nullptr), FOC_ERR_PARAM);

    const foc_motor_setup_t other = setup_with_bandwidth(500.0F);
    EXPECT_EQ(foc_motor_apply_params(a.motor, &params, &other), FOC_ERR_NOT_IMPL);

    // The twin never saw either call, so from here on A must be bit-identical to it. That is the only form of "nothing
    // was written" observable from outside.
    // 孪生体从未收到这两次调用，所以从此刻起 A 必须与它逐位相同。这是从外部唯一可观测的"什么都没写"。
    tick_in_lockstep(a, b, 40);

    foc_motor_params_t read{};
    ASSERT_EQ(foc_motor_get_params(a.motor, &read), FOC_OK);
    EXPECT_FLOAT_EQ(read.ld, params.ld) << "a refused call must not be remembered as applied";

    EXPECT_EQ(foc_motor_destroy(a.motor), FOC_OK);
    EXPECT_EQ(foc_motor_destroy(b.motor), FOC_OK);
}

TEST(ApiParams, AValidCallDoesNotReachTheSensoredControlFrame) {
    Twin a{pinned_plant_params()};
    Twin b{pinned_plant_params()};
    const foc_motor_params_t params = params_struct();
    ASSERT_EQ(make_twin(a, params), FOC_OK);
    ASSERT_EQ(make_twin(b, params), FOC_OK);
    start_twin(a);
    start_twin(b);
    tick_in_lockstep(a, b, 40);

    {
        float speed = 0.0F;
        float id_a = 0.0F;
        float iq_a = 0.0F;
        ASSERT_EQ(foc_get_state(a.motor, &speed, &id_a, &iq_a), FOC_OK);
        ASSERT_GT(std::fabs(iq_a), 0.1F) << "the loop is not tracking, so bit-identity proves nothing";
    }

    // Two faces on purpose. On this host the angle comes from the sensor and the loop gains were built once at
    // construction, so the observer is the only live consumer of these numbers -- and it does not publish into this
    // loop. A large change must therefore leave the trajectory untouched; if a refactor ever fed the numbers into the
    // control frame, this goes red, which is exactly the change somebody would want to be told about.
    // 故意写两面。本宿主的角度来自传感器，而环路增益在构造时一次性生成，所以这些数字唯一的活消费者是观测器——而它不往
    // 这个环路里发布。于是大的变更必须让轨迹原样不动；哪天有人把这些数字接进控制帧，这条就会红，而那正是应该被告知的变更。
    foc_motor_params_t different = params;
    different.rs = params.rs * 0.5F;
    different.lq = params.lq * 2.0F;
    ASSERT_EQ(foc_motor_apply_params(a.motor, &different, nullptr), FOC_OK);
    tick_in_lockstep(a, b, 40);

    EXPECT_EQ(foc_motor_destroy(a.motor), FOC_OK);
    EXPECT_EQ(foc_motor_destroy(b.motor), FOC_OK);
}

TEST(ApiParams, WhatWasStoredIsWhatComesBackAndWhatGetsApplied) {
    // The whole point of the record: numbers measured once have to survive a power cycle and land on the axis again. The
    // two conversions are the only mapping between the record and the ABI struct, so this is also the case that would
    // catch a transposed pair -- the one defect a compiler cannot see here.
    // 记录的全部意义：测过一次的数字要活过一次断电，并再次落到轴上。两个转换是记录与 ABI 结构之间唯一的映射，所以这也
    // 是能抓到写反一对字段的那条用例——编译器在这里看不见的那个缺陷。
    Twin twin{pinned_plant_params()};
    const foc_motor_params_t params = params_struct();
    ASSERT_EQ(make_twin(twin, params), FOC_OK);

    foc_motor_params_t read{};
    ASSERT_EQ(foc_motor_get_params(twin.motor, &read), FOC_OK);

    foc::comm::CommissionedSet record = foc::comm::from_params(read);
    record.machine_id = 0x1234U;
    record.measured_at_unix_s = 1762900000U;
    record.temperature_c = 41.5F;
    std::uint8_t bytes[foc::comm::kCommissionedSetBytes] = {};
    ASSERT_EQ(foc::comm::encode_commissioned_set(record, bytes, sizeof bytes), foc::comm::kCommissionedSetBytes);

    foc::comm::CommissionedSet decoded{};
    ASSERT_EQ(foc::comm::decode_commissioned_set(bytes, sizeof bytes, decoded).status, foc::comm::SetStatus::Ok);
    const foc_motor_params_t restored = foc::comm::to_params(decoded);

    EXPECT_FLOAT_EQ(restored.rs, read.rs);
    EXPECT_FLOAT_EQ(restored.ld, read.ld);
    EXPECT_FLOAT_EQ(restored.lq, read.lq);
    EXPECT_FLOAT_EQ(restored.kt, read.kt);
    EXPECT_FLOAT_EQ(restored.j, read.j);
    EXPECT_FLOAT_EQ(restored.b, read.b);
    EXPECT_EQ(restored.pole_pairs, read.pole_pairs);
    EXPECT_EQ(decoded.machine_id, 0x1234U);
    EXPECT_EQ(decoded.measured_at_unix_s, 1762900000U);
    EXPECT_FLOAT_EQ(decoded.temperature_c, 41.5F);

    // And it can be handed straight back, unchanged.
    // 而且可以直接交回去，一字不改。
    ASSERT_EQ(foc_motor_apply_params(twin.motor, &restored, nullptr), FOC_OK);
    foc_motor_params_t again{};
    ASSERT_EQ(foc_motor_get_params(twin.motor, &again), FOC_OK);
    EXPECT_FLOAT_EQ(again.rs, restored.rs);
    EXPECT_FLOAT_EQ(again.ld, restored.ld);
    EXPECT_FLOAT_EQ(again.lq, restored.lq);
    EXPECT_FLOAT_EQ(again.kt, restored.kt);
    EXPECT_FLOAT_EQ(again.j, restored.j);
    EXPECT_FLOAT_EQ(again.b, restored.b);
    EXPECT_EQ(again.pole_pairs, restored.pole_pairs);

    EXPECT_EQ(foc_motor_destroy(twin.motor), FOC_OK);
}

TEST(ApiParams, TheNumbersReachTheObserver) {
    // The case above pins where the numbers do *not* go on a sensored host. This one covers the other half: with the
    // angle coming from the observer, the numbers reach the loop that commutates -- and that has to be shown by a
    // difference, because "reaches the observer" is not a quantity the C surface publishes.
    // 上一条钉住带传感器宿主上数字**不去**哪里。这一条覆盖另一半：角度来自观测器时，数字到达换相所依据的环路——而这件事
    // 只能用一个差来证明，因为"到达观测器"不是 C 接口会发布的量。
    Twin a{foc::sil::nominal_plant_params()};
    Twin b{foc::sil::nominal_plant_params()};
    const foc_motor_params_t params = params_struct();
    ASSERT_EQ(make_twin(a, params, 1), FOC_OK) << "the observer-driven host was refused";
    ASSERT_EQ(make_twin(b, params, 1), FOC_OK);
    start_twin(a);
    start_twin(b);

    // Lockstep *without* asserting equality: this case needs the first tick at which they differ, which is the
    // measurement rather than a pass/fail condition.
    // 同步推进但**不断言**相等：本用例要的是它们第一次分开的那一拍，那是测量值，不是布尔条件。
    const auto tick_pair = [](Twin& x, Twin& y) -> bool {
        const foc_err_t rx = step_twin(x);
        const foc_err_t ry = step_twin(y);
        std::uint32_t sx[3] = {};
        std::uint32_t sy[3] = {};
        const foc_err_t gx = state_as_bits(x.motor, sx);
        const foc_err_t gy = state_as_bits(y.motor, sy);
        if (rx != FOC_OK || ry != FOC_OK || gx != FOC_OK || gy != FOC_OK) {
            return false;
        }
        return (sx[0] != sy[0]) || (sx[1] != sy[1]) || (sx[2] != sy[2]);
    };

    int spontaneous = 0;
    for (int i = 0; i < 100; ++i) {
        spontaneous += tick_pair(a, b) ? 1 : 0;
    }
    ASSERT_EQ(spontaneous, 0) << "two identical observer-driven motors diverged before any call was made";

    // Non-vacuity: an observer that publishes zeros would make the comparison above meaningless.
    // 非空真：一个只会发布零的观测器会让上面的比较失去意义。
    float speed = 0.0F;
    float id_a = 0.0F;
    float iq_a = 0.0F;
    ASSERT_EQ(foc_get_state(a.motor, &speed, &id_a, &iq_a), FOC_OK);
    std::printf("[api-params] observer-driven loop: speed %.6f rad/s, iq %.6f A of %.6f A\n", static_cast<double>(speed),
                static_cast<double>(iq_a), static_cast<double>(kIqCommand));
    ASSERT_GT(std::fabs(iq_a), 0.1F) << "the observer-driven loop is not driving anything";

    // Face one: the numbers already in force must be a no-op here too, which is what rules out "any call perturbs it".
    // 第一面：把已经在生效的数字再交一次，在这里也必须是空操作——这排除了"任何一次调用都会扰动它"。
    ASSERT_EQ(foc_motor_apply_params(a.motor, &params, nullptr), FOC_OK);
    int changed_by_noop = 0;
    for (int i = 0; i < 40; ++i) {
        changed_by_noop += tick_pair(a, b) ? 1 : 0;
    }
    EXPECT_EQ(changed_by_noop, 0) << "calling with the numbers already in force changed the trajectory";

    // Face two: a real change has to reach the loop, and the tick at which it does is recorded rather than assumed.
    // 第二面：真的变更必须到达环路，而它到达的那一拍是被记录下来的，不是假设的。
    foc_motor_params_t different = params;
    different.lq = params.lq * 1.5F;
    ASSERT_EQ(foc_motor_apply_params(a.motor, &different, nullptr), FOC_OK);
    int divergence_tick = -1;
    for (int i = 0; i < 40 && divergence_tick < 0; ++i) {
        if (tick_pair(a, b)) {
            divergence_tick = i + 1;
        }
    }
    std::printf("[api-params] lq x1.5 reached the observer-driven loop at tick %d\n", divergence_tick);
    EXPECT_GT(divergence_tick, 0) << "the numbers never reached the loop the observer drives";
    // Measured: 1 tick. The assertion allows two, so a one-tick change in when the observer is stepped does not make
    // this flaky -- while a regression that stops forwarding the numbers cannot pass it at all.
    // 实测：第 1 拍。断言允许两拍，于是观测器推进时机的单拍变化不会让它变瓕——而停止转发数字的回归一次也过不了。
    EXPECT_LE(divergence_tick, 2) << "the numbers reached the loop late enough that something else explains it";
    // What this does not claim: the observer is accurate here. At ~1.1 rad/s it sits below the documented speed floor
    // for back-EMF observation, so the published angle is not a good estimate -- it is only an estimate the numbers
    // demonstrably reach. Accuracy at speed is the observer's own claim row, not this one.
    // 本用例不声称的：观测器在这里是准的。~1.1 rad/s 低于反电势观测器文档化的转速下限，所以发布的角度不是好估计——
    // 它只是“数字确实到达”的那个估计。高速下的精度属于观测器自己的 claim 行，不属于这里。

    EXPECT_EQ(foc_motor_destroy(a.motor), FOC_OK);
    EXPECT_EQ(foc_motor_destroy(b.motor), FOC_OK);
}
