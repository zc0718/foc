// The absolute-position concept at the C ABI, and the refusal that makes it honest.
// C ABI 上的绝对位置概念，以及让这个概念诚实的那次拒绝。
//
// The claim under test has two halves and the second one is the point:
//
//   1. before an index mark has been seen, `foc_motor_get_position_absolute()` answers FOC_ERR_STATE and leaves the
//      receiver untouched -- it does not answer 0, because 0 is a legal mechanical angle and a caller comparing it
//      against a target could not tell it from "homed and parked at the origin";
//   2. once the mark is crossed the same call answers FOC_OK and the angle tracks the reference.
//
// Half 1 alone would be trivial to satisfy and useless: a query that can only refuse is a deleted feature with extra
// steps. So the rig has to be able to reach half 2, and that is where the real finding was: `PositionReference` is only
// advanced while the commutation angle comes from the *observer*, and the callback HAL used to report `has_sensor()`
// unconditionally true. Through this ABI an origin could not exist at all. The setup's `use_observer_angle` is what
// makes it exist, and this file is what keeps it existing.
// 判据有两半，第二半才是重点：
//
//   1. 见过索引标记之前，`foc_motor_get_position_absolute()` 答 FOC_ERR_STATE 且不动接收者——它不答 0，因为 0 是
//      合法机械角，拿它去和目标比的调用方分不出那与"已回零且停在原点"有什么区别；
//   2. 越过标记之后，同一次调用答 FOC_OK，且角度跟着参考走。
//
// 只满足第一半既容易又无用：一个只会拒绝的查询，等于删掉的功能加了几步手续。所以台架必须能走到第二半，而真正的发现
// 就在这里：`PositionReference` 只在换相角来自 **观测器** 时才被推进，而回调 HAL 过去无条件报 `has_sensor()` 为
// true。通过这个 ABI，原点根本不可能存在。setup 里的 `use_observer_angle` 让它成为可能，而本文件负责让它继续可能。

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <gtest/gtest.h>

#include "c_abi.hpp"
#include <foc/api/foc_types.h>

#include "sil_pmsm_plant.hpp"

namespace {

constexpr float kPi = 3.14159265358979F;
constexpr float kTwoPi = 6.28318530717959F;
constexpr float kDt = 5e-5F;
constexpr float kVBus = 24.0F;
/** The load machine's speed. Fast enough that the observer locks from its start seed, which is the condition the whole
 *  scenario depends on. 负载机转速。快到观测器能从初始种子锁定，而这是整个场景所依赖的条件。 */
constexpr float kDynoSpeedRad = 300.0F;
/** Where the mark sits, in the rotor's own frame. 标记在转子自身坐标系里的位置。 */
constexpr float kIndexMarkRad = 0.7F;

/**
 * @brief [en] A host whose measurements come from the SIL plant, so the observer has something real to lock onto.
 * @brief [zh] 测量值来自 SIL plant 的主控方，于是观测器有真东西可以锁。
 */
struct PlantHost final {
    foc::sil::PmsmPlant plant{foc::sil::PlantParams{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, 1U}};
    float duty_u{0.5F};
    float duty_v{0.5F};
    float duty_w{0.5F};
    float v_bus{kVBus};
    bool index_latched{false};
    int index_edges{0};
    /**
     * @brief [en] A one-shot: set when the mark was just crossed, consumed by the next `index_pulse()` call.
     * @brief [zh] 一次性的：刚越过标记时置位，由下一次 `index_pulse()` 调用消费。
     * @note [en] The first version of this rig returned `index_edges > 0`, i.e. a *level*. Every tick after the
     *            crossing then re-established the origin, which pinned the referenced angle at `index_mech_rad` forever
     *            -- the measured 0.0000 -> 0.0000 over 10 ms of rotation. `hal/position.hpp` warns about exactly this
     *            ("a level would count one pass thousands of times"); the warning was right and the rig was wrong.
     * @note [zh] 本台架的第一版返回 `index_edges > 0`，也就是一个 **电平** 。越过之后每一拍都重新建立原点，把
     *            referenced 角度永久钉在 `index_mech_rad` 上——实测 10 ms 的转动里 0.0000 -> 0.0000。 `hal/position.hpp`
     *            早就警告过这一点（"电平会把一次经过数成几千次"）；警告是对的，台架错了。
     */
    bool index_pulse_now{false};
    float previous_angle_rad{0.0F};

    PlantHost() noexcept {
        foc::sil::PlantState initial{};
        initial.theta_mech = 0.0F;
        plant.reset(initial);
    }

    /** One control period of the plant under the duty the library wrote. 库写下的占空比在 plant 上的一拍。 */
    void advance(float dt_s) noexcept {
        const float mean = (duty_u + duty_v + duty_w) * kVBus / 3.0F;
        const foc::core::Uvw<float> v_uvw{duty_u * kVBus - mean, duty_v * kVBus - mean, duty_w * kVBus - mean};
        const float theta_e = static_cast<float>(plant.params().pole_pairs) * plant.theta_mech_rad();
        plant.set_voltage_dq(foc::core::park(foc::core::clarke(v_uvw), theta_e));
        plant.advance(dt_s);
        // A load machine holds the speed; without this the torque would integrate a speed of its own and the
        // scenario would be measuring the plant's inertia instead of the C ABI.
        // 负载机维持转速；否则转矩会积出自己的转速，这个场景就变成了在量 plant 的惯量而不是 C ABI。
        plant.set_mechanical_speed(kDynoSpeedRad);
    }

    void observe_mark() noexcept {
        const float angle = static_cast<float>(std::fmod(plant.theta_mech_rad(), kTwoPi));
        if (!index_latched && previous_angle_rad < kIndexMarkRad && angle >= kIndexMarkRad) {
            index_latched = true;
            ++index_edges;
            index_pulse_now = true;
        }
        previous_angle_rad = angle;
    }
};

auto on_write_duty(void* user, const float duty_uvw[3]) -> void {
    auto* host = static_cast<PlantHost*>(user);
    host->duty_u = duty_uvw[0];
    host->duty_v = duty_uvw[1];
    host->duty_w = duty_uvw[2];
}

auto on_read_current(void* user, float uvw_a[3]) -> void {
    const foc::core::Uvw<float> i = static_cast<PlantHost*>(user)->plant.current_uvw();
    uvw_a[0] = i.u;
    uvw_a[1] = i.v;
    uvw_a[2] = i.w;
}

auto on_read_voltage(void* user) -> float { return static_cast<PlantHost*>(user)->v_bus; }

// The power stage's fault output: this host models a healthy inverter. Required by the library (see
// `foc_hal_callbacks_t::read_drive_fault`), so it is answered here, once, rather than left NULL.
// 功率级的故障输出：本主控方建模的是一个健康逆变器。库要求它在（见 `foc_hal_callbacks_t::read_drive_fault`），
// 所以在这里回答一次，而不是留成 NULL。
auto on_read_drive_fault(void* /*user*/) -> int32_t { return 0; }


auto on_has_index(void* /*user*/) -> int32_t { return 1; }

auto on_index_pulse(void* user) -> int32_t {
    auto* host = static_cast<PlantHost*>(user);
    const bool now = host->index_pulse_now;
    host->index_pulse_now = false; // an edge, consumed once -- never a level
    return now ? 1 : 0;
}

auto sensorless_callbacks(PlantHost& host) -> foc_hal_callbacks_t {
    foc_hal_callbacks_t callbacks{};
    callbacks.struct_size = sizeof(foc_hal_callbacks_t);
    callbacks.user = &host;
    callbacks.pwm_write_duty = &on_write_duty;
    callbacks.read_current = &on_read_current;
    callbacks.read_voltage_v = &on_read_voltage;
    callbacks.read_drive_fault = &on_read_drive_fault;
    // Deliberately no `read_angle_mech_rad` and no `read_velocity_rad_s`: this host has no encoder. The angle has to
    // come from the observer, which is the only path on which an origin can exist.
    // 故意不给 `read_angle_mech_rad` 与 `read_velocity_rad_s`：这台主控方没有编码器。角度必须来自观测器，而那是
    // 原点能够存在的唯一通路。
    callbacks.position_has_index = &on_has_index;
    callbacks.position_index_pulse = &on_index_pulse;
    return callbacks;
}

auto setup_for_sensorless() -> foc_motor_setup_t {
    foc_motor_setup_t setup{};
    setup.struct_size = sizeof(foc_motor_setup_t);
    setup.use_observer_angle = 1;
    setup.i_max_a = 3.0F;
    setup.pwm_freq_hz = 20000.0F;
    setup.v_bus_v = kVBus;
    return setup;
}

auto params_for_rig() -> foc_motor_params_t {
    foc_motor_params_t params{};
    params.rs = 0.5F;
    params.ld = 4.0e-4F;
    params.lq = 4.0e-4F;
    params.kt = 0.05F;
    params.j = 1.0e-4F;
    params.b = 1.0e-4F;
    params.pole_pairs = 1U;
    return params;
}

} // namespace

TEST(ApiCPositionAbsolute, RefusesBeforeAnOriginAndAnswersAfterOne) {
    PlantHost host{};
    foc_hal_callbacks_t callbacks = sensorless_callbacks(host);
    const foc_motor_params_t params = params_for_rig();
    const foc_motor_setup_t setup = setup_for_sensorless();
    foc_motor_t motor{};
    ASSERT_EQ(foc_motor_create_host(&params, &setup, &callbacks, &motor), FOC_OK);

    // A sensor-less host is allowed to have no angle callback, and must be refused when it claims a sensor.
    // 无感主控方被允许没有角度回调；而当它声称有传感器时必须被拒绝。
    {
        PlantHost other{};
        foc_hal_callbacks_t with_sensor = sensorless_callbacks(other);
        foc_motor_setup_t wants_sensor = setup_for_sensorless();
        wants_sensor.use_observer_angle = 0;
        foc_motor_t refused{};
        EXPECT_EQ(foc_motor_create_host(&params, &wants_sensor, &with_sensor, &refused), FOC_ERR_PARAM)
            << "a host with no angle callback cannot claim to have a position sensor";
    }

    // Before the origin: the sentinel must survive, which is the difference between "no answer" and "the answer 0".
    // 原点之前：哨兵值必须原封不动，这就是"没有答案"与"答案是 0"的区别。
    {
        constexpr float kSentinel = -1234.5F;
        float angle = kSentinel;
        EXPECT_EQ(foc_motor_get_position_absolute(motor, &angle), FOC_ERR_STATE);
        EXPECT_FLOAT_EQ(angle, kSentinel) << "a refused query must not write its receiver";

        foc_motor_status_t status{};
        status.struct_size = sizeof(foc_motor_status_t);
        ASSERT_EQ(foc_motor_get_status(motor, &status), FOC_OK);
        EXPECT_EQ(status.homed, 0);
        EXPECT_EQ(status.lost_count, 0);
    }

    ASSERT_EQ(foc_motor_enable(motor), FOC_OK);
    // Spin up and let the observer lock; the origin cannot appear before the mark is crossed, which needs motion.
    // 转起来让观测器锁定；原点不可能在越过标记之前出现，而越过标记需要运动。
    host.index_latched = false;
    host.index_edges = 0;
    host.index_pulse_now = false;
    host.previous_angle_rad = 0.0F;
    int homed_at_tick = -1;
    for (int tick = 0; tick < 60000 && homed_at_tick < 0; ++tick) {
        static_cast<void>(foc_tick(motor, kDt));
        host.advance(kDt);
        host.observe_mark();
        foc_motor_status_t status{};
        status.struct_size = sizeof(foc_motor_status_t);
        if ((foc_motor_get_status(motor, &status) == FOC_OK) && (status.homed != 0)) {
            homed_at_tick = tick;
        }
    }
    ASSERT_GT(homed_at_tick, 0) << "with an index callback the axis has to be able to acquire an origin at all";

    // After the origin: the answer exists and it tracks the reference.
    // 原点之后：答案存在，并且跟着参考走。
    float first_angle = 0.0F;
    ASSERT_EQ(foc_motor_get_position_absolute(motor, &first_angle), FOC_OK);
    // The motor configures `index_mech_rad` as 0, so the origin reads 0 *at the mark* -- an arbitrary but declared
    // zero, which is the whole point of an origin: it is where the machine says zero is.
    // 电机把 `index_mech_rad` 配成 0，所以 **在标记处** 原点读 0——一个任意但被声明的零点，这正是原点的意义：机器说
    // 零在哪，零就在哪。
    EXPECT_NEAR(first_angle, 0.0F, 0.05F) << "the origin reads `index_mech_rad` at the mark";
    for (int tick = 0; tick < 200; ++tick) {
        static_cast<void>(foc_tick(motor, kDt));
        host.advance(kDt);
    }
    float later_angle = 0.0F;
    ASSERT_EQ(foc_motor_get_position_absolute(motor, &later_angle), FOC_OK);
    // The rotor is driven forward by the load machine, so the referenced angle has to grow with it.
    // 转子被负载机带着正转，因此 referenced 角度必须随之增长。
    // 200 ticks at the load machine's speed is 3.0 rad of rotor travel, and the maintained origin has to follow it.
    // 按负载机转速走 200 拍等于转子走过 3.0 rad，而维护中的原点必须跟着走。
    EXPECT_NEAR(later_angle - first_angle, kDynoSpeedRad * 200.0F * kDt, 0.30F)
        << "a maintained origin tracks the rotor; a pinned one would not move";

    std::printf("\n[api-position] origin acquired after %0.1f ms; angle %0.4f -> %0.4f rad over 10 ms\n",
                static_cast<double>(static_cast<float>(homed_at_tick) * kDt * 1000.0F),
                static_cast<double>(first_angle), static_cast<double>(later_angle));
    EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
}
