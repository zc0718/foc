// The host-motor path, exercised through the C ABI rather than through the template: the claim to verify is that
// `foc_motor_create_host()` mints a handle every other entry point accepts unchanged. Two pools, one body of code.
// 主控电机路径，经 C ABI 而不是模板验证：要证的断言是 `foc_motor_create_host()` 签发的句柄被其它所有入口原样接受。
// 两个池，一份代码。

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/api/foc_api.h>
#include <foc/api/foc_types.h>

namespace {

struct Host final {
    float current_a[3]{0.0F, 0.0F, 0.0F};
    float angle_rad{0.0F};
    float v_bus_v{24.0F};
    int duty_calls{0};
    int current_calls{0};
};

auto on_write_duty(void* user, const float /*duty_uvw*/[3]) -> void { static_cast<Host*>(user)->duty_calls += 1; }

auto on_read_current(void* user, float uvw_a[3]) -> void {
    auto* host = static_cast<Host*>(user);
    host->current_calls += 1;
    for (int i = 0; i < 3; ++i) {
        uvw_a[i] = host->current_a[i];
    }
}

auto on_read_angle(void* user) -> float { return static_cast<Host*>(user)->angle_rad; }

auto on_read_voltage(void* user) -> float { return static_cast<Host*>(user)->v_bus_v; }

// The power stage's fault output: this host models a healthy inverter. Required by the library (see
// `foc_hal_callbacks_t::read_drive_fault`), so it is answered here, once, rather than left NULL.
// 功率级的故障输出：本主控方建模的是一个健康逆变器。库要求它在（见 `foc_hal_callbacks_t::read_drive_fault`），
// 所以在这里回答一次，而不是留成 NULL。
auto on_read_drive_fault(void* /*user*/) -> int32_t { return 0; }


auto callbacks_for(Host& host) -> foc_hal_callbacks_t {
    foc_hal_callbacks_t callbacks{};
    callbacks.struct_size = sizeof(foc_hal_callbacks_t);
    callbacks.user = &host;
    callbacks.pwm_write_duty = &on_write_duty;
    callbacks.read_current = &on_read_current;
    callbacks.read_angle_mech_rad = &on_read_angle;
    callbacks.read_voltage_v = &on_read_voltage;
    callbacks.read_drive_fault = &on_read_drive_fault;
    return callbacks;
}

auto nominal_params() -> foc_motor_params_t {
    foc_motor_params_t params{};
    params.rs = 0.5F;
    params.ld = 2.0e-4F;
    params.lq = 4.0e-4F;
    params.kt = 0.02F;
    params.j = 1.0e-5F;
    params.b = 1.0e-6F;
    params.pole_pairs = 1U;
    return params;
}

} // namespace

TEST(ApiCHostMotor, TheWholeDriveRunsWithTheCallersMeasurements) {
    // The criterion: a handle from the host constructor must be usable by every other entry point, and the loop must
    // actually ask the host once per tick. If the dispatch only knew about the stub pool, the first call would already
    // answer FOC_ERR_PARAM.
    // 判据：主控构造签发的句柄必须能被其它每个入口使用，且环每拍都真的问主控一次。如果派发只认桩池，第一次调用
    // 就会答 FOC_ERR_PARAM。
    Host host{};
    const foc_hal_callbacks_t callbacks = callbacks_for(host);
    const foc_motor_params_t params = nominal_params();
    foc_motor_t motor{};

    ASSERT_EQ(foc_motor_create_host(&params, nullptr, &callbacks, &motor), FOC_OK);
    ASSERT_EQ(foc_motor_enable(motor), FOC_OK);
    ASSERT_EQ(foc_set_torque(motor, 0.5F), FOC_OK);
    for (int i = 0; i < 200; ++i) {
        host.angle_rad = 0.1F * static_cast<float>(i);
        ASSERT_EQ(foc_tick(motor, 1.0F / 20000.0F), FOC_OK);
    }

    std::printf("\n[c-api host] %d duty write(s), %d current read(s) over 200 ticks\n", host.duty_calls,
                host.current_calls);
    EXPECT_EQ(host.current_calls, 200);
    EXPECT_EQ(host.duty_calls, 200);

    foc_motor_status_t status{};
    status.struct_size = sizeof(foc_motor_status_t);
    ASSERT_EQ(foc_motor_get_status(motor, &status), FOC_OK);
    EXPECT_EQ(status.mode_state, static_cast<std::int32_t>(FOC_MOTOR_RUN));

    float id = 0.0F;
    float iq = 0.0F;
    float speed = 0.0F;
    ASSERT_EQ(foc_get_state(motor, &speed, &id, &iq), FOC_OK);

    // A destroyed host handle is as dead as a destroyed stub handle: same generation check, same answer.
    // 销毁后的主控句柄与销毁后的桩句柄一样是死的：同一套代际校验，同一个答案。
    ASSERT_EQ(foc_motor_destroy(motor), FOC_OK);
    EXPECT_EQ(foc_tick(motor, 1.0F / 20000.0F), FOC_ERR_PARAM);
}

TEST(ApiCHostMotor, AnIncompleteCallbackSetIsRejectedRatherThanDegraded) {
    // No duty sink means the loop would compute forever and never switch; no angle means it would run against a constant.
    // Both are programming errors and both must be refused at creation.
    // 没有占空比出口，环会永远计算却从不开关；没有角度，它就对着一个常数跑。两者都是编程错误，都必须在创建时拒绝。
    Host host{};
    const foc_motor_params_t params = nominal_params();
    foc_motor_t motor{};
    foc_hal_callbacks_t callbacks = callbacks_for(host);

    callbacks.pwm_write_duty = nullptr;
    EXPECT_EQ(foc_motor_create_host(&params, nullptr, &callbacks, &motor), FOC_ERR_PARAM);

    callbacks = callbacks_for(host);
    callbacks.read_angle_mech_rad = nullptr;
    EXPECT_EQ(foc_motor_create_host(&params, nullptr, &callbacks, &motor), FOC_ERR_PARAM);

    // Two different refusals, and the difference is the point. An **input** struct must be exactly the current size
    // (`foc_api.c` checks it first), so a caller built against an older header is refused outright and never reaches the
    // question of which fields it has. That exact-size rule is what makes the next line possible: within a struct of the
    // right size, a NULL fault output is *known* to be a host saying "I did not wire it", not an old caller, and it is
    // answered with `FOC_ERR_NOT_IMPL` instead of being defaulted to "no fault". Every other required callback keeps
    // `FOC_ERR_PARAM`, because "your argument set is malformed" is exactly what is wrong with it.
    // 两种拒绝，而差别就是重点。**输入**结构必须正好是当前大小（`foc_api.c` 先查这一条），所以按旧头编译的调用方直接被
    // 拒，根本走不到"它有哪些字段"这个问题。正是这条精确大小规则让下一行成为可能：在一个尺寸正确的结构里，NULL 的故障
    // 输出**已知**是主控方在说"我没接"，而不是旧调用方——于是它答 `FOC_ERR_NOT_IMPL`，而不是被默认成"无故障"。其余必需
    // 回调保持 `FOC_ERR_PARAM`，因为"你的参数集合不完整"正是它们的问题。
    callbacks = callbacks_for(host);
    callbacks.struct_size = sizeof(foc_hal_callbacks_t) - 1U;
    EXPECT_EQ(foc_motor_create_host(&params, nullptr, &callbacks, &motor), FOC_ERR_PARAM);

    callbacks = callbacks_for(host);
    callbacks.read_drive_fault = nullptr;
    EXPECT_EQ(foc_motor_create_host(&params, nullptr, &callbacks, &motor), FOC_ERR_NOT_IMPL);

    callbacks = callbacks_for(host);
    callbacks.read_current = nullptr;
    EXPECT_EQ(foc_motor_create_host(&params, nullptr, &callbacks, &motor), FOC_ERR_PARAM)
        << "the other required callbacks keep their answer";
}

TEST(ApiCHostMotor, TheHostPoolIsBoundedAndRecoversAfterDestroy) {
    // The pool is static storage, so exhausting it is a defined answer (`FOC_ERR_NO_RESOURCE`), not a heap failure --
    // and destroying a motor has to give the slot back, or a long-lived Python session would leak capacity.
    // 池是静态存储，用尽是一个**有定义的答案**（`FOC_ERR_NO_RESOURCE`）而不是堆分配失败——而且销毁必须把槽位还回来，
    // 否则长命的 Python 会话会漏掉容量。
    const foc_motor_params_t params = nominal_params();
    Host hosts[4]{};
    foc_motor_t motors[4]{};

    int created = 0;
    for (int i = 0; i < 4; ++i) {
        foc_hal_callbacks_t callbacks = callbacks_for(hosts[i]);
        if (foc_motor_create_host(&params, nullptr, &callbacks, &motors[i]) == FOC_OK) {
            ++created;
        } else {
            break;
        }
    }
    std::printf("[c-api host] created %d motor(s) before the pool said no\n", created);
    EXPECT_GE(created, 1);
    EXPECT_LT(created, 4) << "the host pool is supposed to be bounded";

    ASSERT_EQ(foc_motor_destroy(motors[0]), FOC_OK);
    foc_hal_callbacks_t callbacks = callbacks_for(hosts[0]);
    foc_motor_t again{};
    EXPECT_EQ(foc_motor_create_host(&params, nullptr, &callbacks, &again), FOC_OK)
        << "destroying a host motor must return its slot";

    for (int i = 1; i < created; ++i) {
        (void)foc_motor_destroy(motors[i]);
    }
    (void)foc_motor_destroy(again);
}
