// The observer selector's contract: the default runs, an unimplemented mode says so, a garbage value is a caller error.
//
// The three answers are the point. A selector that accepts `FOC_OBSERVER_HFI` and quietly runs the back-EMF observer
// would be worse than no selector at all, because the caller would believe a capability it does not have.
//
// 观测器选择器的契约：默认值能跑，未实现的模式如实说明，非法取值是调用方的错误。
// 三种回答才是重点。一个接受 `FOC_OBSERVER_HFI` 却悄悄跑反电势观测器的选择器，比没有选择器更糟——调用方会相信自己拥有一个
// 并不存在的能力。

#include <cmath>
#include <cstddef>
#include <cstdint>

#include <gtest/gtest.h>

#include <foc/api/foc_api.h>
#include <foc/api/foc_types.h>

namespace {

auto params_struct() -> foc_motor_params_t {
    foc_motor_params_t params{};
    params.rs = 0.5F;
    params.ld = 4.0e-4F;
    params.lq = 4.0e-4F;
    params.kt = 0.05F;
    params.j = 1.0e-5F;
    params.b = 1.0e-6F;
    params.pole_pairs = 7U;
    return params;
}

auto setup_with_observer(std::int32_t observer) -> foc_motor_setup_t {
    foc_motor_setup_t setup{};
    setup.struct_size = sizeof(foc_motor_setup_t);
    setup.i_max_a = 3.0F;
    setup.pwm_freq_hz = 20000.0F;
    setup.deadtime_ns = 0.0F;
    setup.v_bus_v = 24.0F;
    setup.observer = observer;
    return setup;
}

struct Host final {
    float angle_rad{0.0F};
    float v_bus_v{24.0F};
    int ticks{0};
};

auto on_write_duty(void* user, const float /*duty*/[3]) -> void { static_cast<Host*>(user)->ticks += 1; }
auto on_read_current(void* user, float uvw_a[3]) -> void {
    static_cast<void>(user);
    uvw_a[0] = 0.0F;
    uvw_a[1] = 0.0F;
    uvw_a[2] = 0.0F;
}
auto on_read_angle(void* user) -> float { return static_cast<Host*>(user)->angle_rad; }
auto on_read_voltage(void* user) -> float { return static_cast<Host*>(user)->v_bus_v; }

auto callbacks_for(Host& host) -> foc_hal_callbacks_t {
    foc_hal_callbacks_t callbacks{};
    callbacks.struct_size = sizeof(foc_hal_callbacks_t);
    callbacks.user = &host;
    callbacks.pwm_write_duty = &on_write_duty;
    callbacks.read_current = &on_read_current;
    callbacks.read_angle_mech_rad = &on_read_angle;
    callbacks.read_voltage_v = &on_read_voltage;
    return callbacks;
}

} // namespace

TEST(ApiObserverSelector, TheDefaultIsTheBackEmfObserverAndItRuns) {
    // The criterion: with the field left at zero the motor is created, enabled and ticks, i.e. the new field changes
    // nothing for a caller that never sets it.
    // 判据：字段留零时电机能被创建、使能并 tick，即对从不设置它的调用方，新字段不改变任何行为。
    Host host{};
    const foc_hal_callbacks_t callbacks = callbacks_for(host);
    const foc_motor_params_t params = params_struct();
    const foc_motor_setup_t setup = setup_with_observer(FOC_OBSERVER_BACK_EMF);

    foc_motor_t motor{};
    ASSERT_EQ(foc_motor_create_host(&params, &setup, &callbacks, &motor), FOC_OK);
    ASSERT_EQ(foc_motor_enable(motor), FOC_OK);
    ASSERT_EQ(foc_set_torque(motor, 0.5F), FOC_OK);
    for (int i = 0; i < 10; ++i) {
        host.angle_rad = 0.01F * static_cast<float>(i);
        ASSERT_EQ(foc_tick(motor, 1.0F / 20000.0F), FOC_OK);
    }
    EXPECT_EQ(host.ticks, 10);
    EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
}

TEST(ApiObserverSelector, AModeThatIsNotWiredSaysSoInsteadOfRunningTheDefault) {
    // Each named-but-unwired mode must answer NOT_IMPL from both constructors. Answering OK and running the back-EMF
    // observer would hand the caller a capability the library does not have.
    // 每个已命名但未接入的模式必须在两个构造函数上都答 NOT_IMPL。答 OK 却跑反电势观测器，等于把一个库并不具备的能力交给
    // 调用方。
    const foc_motor_params_t params = params_struct();
    for (const std::int32_t observer :
         {static_cast<std::int32_t>(FOC_OBSERVER_NONLINEAR_FLUX), static_cast<std::int32_t>(FOC_OBSERVER_HFI),
          static_cast<std::int32_t>(FOC_OBSERVER_BLENDED)}) {
        const foc_motor_setup_t setup = setup_with_observer(observer);

        foc_motor_t stub{};
        EXPECT_EQ(foc_motor_create_ex(&params, &setup, &stub), FOC_ERR_NOT_IMPL) << "observer " << observer;
        EXPECT_EQ(stub, nullptr);

        Host host{};
        const foc_hal_callbacks_t callbacks = callbacks_for(host);
        foc_motor_t host_motor{};
        EXPECT_EQ(foc_motor_create_host(&params, &setup, &callbacks, &host_motor), FOC_ERR_NOT_IMPL)
            << "observer " << observer;
        EXPECT_EQ(host_motor, nullptr);
    }
}

TEST(ApiObserverSelector, AValueOutsideTheEnumerationIsACallerError) {
    // Different from "not implemented": the call itself is malformed, and the two answers must not be merged.
    // 与"未实现"不同：这次调用本身是错的，两种回答不能合并。
    const foc_motor_params_t params = params_struct();
    const foc_motor_setup_t setup = setup_with_observer(99);
    foc_motor_t motor{};
    EXPECT_EQ(foc_motor_create_ex(&params, &setup, &motor), FOC_ERR_PARAM);
    EXPECT_EQ(motor, nullptr);

    // A negative value is the same class of error, not a silent default.
    const foc_motor_setup_t negative = setup_with_observer(-1);
    EXPECT_EQ(foc_motor_create_ex(&params, &negative, &motor), FOC_ERR_PARAM);
    EXPECT_EQ(motor, nullptr);
}

TEST(ApiObserverSelector, AnOlderShorterSetupStillMeansTheDefault) {
    // The field was appended, so a caller built against the older header passes a shorter struct and must get the
    // default observer rather than a parameter error.
    // 字段是追加的，因此按旧头编译的调用方传入更短的结构，应当得到默认观测器，而不是参数错误。
    Host host{};
    const foc_hal_callbacks_t callbacks = callbacks_for(host);
    const foc_motor_params_t params = params_struct();
    foc_motor_setup_t setup = setup_with_observer(FOC_OBSERVER_HFI);
    setup.struct_size = static_cast<std::uint32_t>(offsetof(foc_motor_setup_t, observer));

    foc_motor_t motor{};
    EXPECT_EQ(foc_motor_create_host(&params, &setup, &callbacks, &motor), FOC_OK);
    if (motor != nullptr) {
        EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
    }
}
