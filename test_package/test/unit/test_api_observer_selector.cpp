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

/** @brief [en] The same nameplate with real saliency (lq twice ld). [zh] 同一台机器的铭牌，但有真实凸极（lq 为 ld 两倍）。 */
auto salient_params() -> foc_motor_params_t {
    foc_motor_params_t params = params_struct();
    params.lq = 8.0e-4F;
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

auto setup_with_carrier(std::int32_t observer, float inject_v, float inject_hz) -> foc_motor_setup_t {
    foc_motor_setup_t setup = setup_with_observer(observer);
    setup.inject_v = inject_v;
    setup.inject_hz = inject_hz;
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
    // observer would hand the caller a capability the library does not have. `NONLINEAR_FLUX` and `HFI` have since been
    // wired and moved to their own cases; leaving them in this loop would have kept asserting a fact that stopped being
    // true, and the test would have had to be weakened rather than corrected.
    // 每个已命名但未接入的模式必须在两个构造函数上都答 NOT_IMPL。答 OK 却跑反电势观测器，等于把一个库并不具备的能力交给
    // 调用方。`NONLINEAR_FLUX` 后来接上了，已移到下面它自己的用例；把它留在这个循环里就是继续断言一个已经不成立的事实，
    // 那样测试只能被削弱而不能被修正。
    const foc_motor_params_t params = params_struct();
    for (const std::int32_t observer : {static_cast<std::int32_t>(FOC_OBSERVER_BLENDED)}) {
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

TEST(ApiObserverSelector, TheNonlinearFluxModeIsWiredAndItIsWhatActuallyRuns) {
    // "It was created" is not evidence that the selection took: a selector that admitted the mode and then built the
    // default would pass that. The criterion here is the status field, which the bridge fills from the live observer
    // (`instance.observer().kind()`), not from the request. So each mode has to report *itself*, and the two have to
    // report differently -- identical answers would mean the field is echoing the enum back.
    // "创建成功"不能证明选择生效：一个接受该模式却仍构造默认值的实现也能通过。这里的判据是状态字段，bridge 用活着的
    // 观测器（`instance.observer().kind()`）填它，而不是用请求值。于是每种模式必须报出**它自己**，且两者必须不同——
    // 若相同，说明这个字段只是把枚举原样回显。
    const foc_motor_params_t params = params_struct();

    std::int32_t reported[2] = {-1, -1};
    const std::int32_t requested[2] = {static_cast<std::int32_t>(FOC_OBSERVER_BACK_EMF),
                                       static_cast<std::int32_t>(FOC_OBSERVER_NONLINEAR_FLUX)};
    for (int i = 0; i < 2; ++i) {
        const foc_motor_setup_t setup = setup_with_observer(requested[i]);
        foc_motor_t motor{};
        ASSERT_EQ(foc_motor_create_ex(&params, &setup, &motor), FOC_OK) << "observer " << requested[i];
        ASSERT_NE(motor, nullptr);
        ASSERT_EQ(foc_motor_enable(motor), FOC_OK);
        ASSERT_EQ(foc_set_torque(motor, 0.5F), FOC_OK);
        for (int tick = 0; tick < 20; ++tick) {
            ASSERT_EQ(foc_tick(motor, 1.0F / 20000.0F), FOC_OK);
        }

        foc_motor_status_t status{};
        status.struct_size = sizeof(foc_motor_status_t);
        ASSERT_EQ(foc_motor_get_status(motor, &status), FOC_OK);
        reported[i] = status.observer;
        EXPECT_EQ(status.observer, requested[i]) << "the mode that was asked for is not the mode that reports itself";
        EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
    }
    EXPECT_NE(reported[0], reported[1]) << "the status field does not distinguish the two observers";
}

TEST(ApiObserverSelector, AnOlderShorterStatusStructIsNotWrittenPastTheEnd) {
    // The observer field was appended to the status struct, so a caller built against the previous header owns a shorter
    // object. The library must treat the declared size as a ceiling: filling the new field regardless would write past
    // the end of that caller's memory, which is the one failure mode of an output struct that cannot be caught by a
    // value check afterwards. Pre-filling the field with a sentinel and requiring it to survive is the cheapest way to
    // see that the write stayed inside.
    // 观测器字段是追加到状态结构尾部的，因此按旧头编译的调用方拥有更短的对象。库必须把声明的大小当作上限：无论如何都填
    // 新字段就会写到调用方内存之外，而这是输出结构唯一一种事后无法靠取值检查发现的失效方式。先把该字段填成哨兵、再要求它
    // 存活，是"写没有越界"最便宜的观测方式。
    Host host{};
    const foc_hal_callbacks_t callbacks = callbacks_for(host);
    const foc_motor_params_t params = params_struct();
    const foc_motor_setup_t setup = setup_with_observer(FOC_OBSERVER_NONLINEAR_FLUX);

    foc_motor_t motor{};
    ASSERT_EQ(foc_motor_create_host(&params, &setup, &callbacks, &motor), FOC_OK);
    ASSERT_NE(motor, nullptr);

    foc_motor_status_t status{};
    status.struct_size = static_cast<std::uint32_t>(offsetof(foc_motor_status_t, observer));
    status.observer = 12345; // the sentinel: anything the library writes here is a bug
    EXPECT_EQ(foc_motor_get_status(motor, &status), FOC_OK);
    EXPECT_EQ(status.observer, 12345) << "the library wrote past the size the caller declared";
    // The fields the caller does own are still filled.
    EXPECT_EQ(status.mode_state, static_cast<std::int32_t>(FOC_MOTOR_IDLE));
    EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
}

TEST(ApiObserverSelector, TheHfiModeRefusesWithoutACarrierOrWithoutSaliency) {
    // HFI is admitted on facts, not on a switch. Two of them are physical:
    //   - no carrier stated means there is nothing to demodulate, and the library has no defensible default;
    //   - a surface machine has `lq == ld`, and `predicted_gain_a()` is proportional to their difference, so the
    //     demodulated signal is identically zero -- no follower can lock. Refusing here is not caution, it is the same
    //     arithmetic the observer's constructor performs when it rejects `demod_gain_a == 0`.
    // The answers stay distinct: a well-formed request for a mode that is not wired is NOT_IMPL, while a request this
    // machine cannot honour is PARAM. Merging them would let a caller believe the feature exists.
    // HFI 是按事实放行的，不是按开关。其中两条是物理事实：未给出载波就意味着没有东西可解调，而库没有站得住的默认值；
    // 表贴机 `lq == ld`，而 `predicted_gain_a()` 正比于两者之差，于是解调信号恒为零——任何跟随器都锁不住。在这里拒绝不是
    // 保守，而是观测器构造函数拒绝 `demod_gain_a == 0` 时所用的同一套算术。
    // 两种回答保持区分：写法正确但模式未接入是 NOT_IMPL，而"这台机器满足不了"是 PARAM。合并二者会让调用方以为功能存在。
    const std::int32_t hfi = static_cast<std::int32_t>(FOC_OBSERVER_HFI);
    foc_motor_t motor{};

    // Surface machine, carrier supplied: refused because there is no saliency to measure.
    const foc_motor_params_t surface = params_struct();
    const foc_motor_setup_t salient_setup = setup_with_carrier(hfi, 12.0F, 2000.0F);
    EXPECT_EQ(foc_motor_create_ex(&surface, &salient_setup, &motor), FOC_ERR_PARAM) << "surface machine accepted HFI";
    EXPECT_EQ(motor, nullptr);

    // Salient machine, no carrier: refused because nothing would be injected.
    const foc_motor_params_t salient = salient_params();
    const foc_motor_setup_t no_carrier = setup_with_carrier(hfi, 0.0F, 0.0F);
    EXPECT_EQ(foc_motor_create_ex(&salient, &no_carrier, &motor), FOC_ERR_PARAM) << "HFI accepted without a carrier";

    // Salient machine, carrier below the demodulation filter: refused, the filter would remove what it demodulates.
    const foc_motor_setup_t slow_carrier = setup_with_carrier(hfi, 12.0F, 300.0F);
    EXPECT_EQ(foc_motor_create_ex(&salient, &slow_carrier, &motor), FOC_ERR_PARAM) << "carrier at the filter cutoff";

    // The control: everything stated, so it must run and must report itself.
    foc_motor_t hfi_motor{};
    ASSERT_EQ(foc_motor_create_ex(&salient, &salient_setup, &hfi_motor), FOC_OK);
    ASSERT_NE(hfi_motor, nullptr);
    ASSERT_EQ(foc_motor_enable(hfi_motor), FOC_OK);
    ASSERT_EQ(foc_set_torque(hfi_motor, 0.0F), FOC_OK);
    for (int tick = 0; tick < 20; ++tick) {
        ASSERT_EQ(foc_tick(hfi_motor, 1.0F / 20000.0F), FOC_OK);
    }
    foc_motor_status_t status{};
    status.struct_size = sizeof(foc_motor_status_t);
    ASSERT_EQ(foc_motor_get_status(hfi_motor, &status), FOC_OK);
    EXPECT_EQ(status.observer, hfi) << "the mode that was asked for is not the mode that reports itself";
    EXPECT_EQ(foc_motor_destroy(hfi_motor), FOC_OK);
}

TEST(ApiObserverSelector, FourModesHaveFourDistinctAnswersToday) {
    // One row per mode, so that wiring the last one is a change to this table rather than a discovery made by a user.
    // 每个模式一行，于是接入最后一个模式时改动的是这张表，而不是由用户发现。
    const foc_motor_params_t surface = params_struct();
    const foc_motor_params_t salient = salient_params();
    struct Row final {
        std::int32_t mode;
        const foc_motor_params_t* params;
        float inject_v;
        float inject_hz;
        foc_err_t expected;
        const char* why;
    };
    const Row rows[] = {
        {static_cast<std::int32_t>(FOC_OBSERVER_BACK_EMF), &surface, 0.0F, 0.0F, FOC_OK, "wired, the default"},
        {static_cast<std::int32_t>(FOC_OBSERVER_NONLINEAR_FLUX), &surface, 0.0F, 0.0F, FOC_OK, "wired, no carrier"},
        {static_cast<std::int32_t>(FOC_OBSERVER_HFI), &salient, 12.0F, 2000.0F, FOC_OK, "wired, carrier stated"},
        {static_cast<std::int32_t>(FOC_OBSERVER_HFI), &surface, 12.0F, 2000.0F, FOC_ERR_PARAM, "no saliency"},
        {static_cast<std::int32_t>(FOC_OBSERVER_HFI), &salient, 0.0F, 0.0F, FOC_ERR_PARAM, "no carrier"},
        {static_cast<std::int32_t>(FOC_OBSERVER_BLENDED), &salient, 12.0F, 2000.0F, FOC_ERR_NOT_IMPL, "not wired"},
        {99, &surface, 0.0F, 0.0F, FOC_ERR_PARAM, "outside the enumeration"},
    };
    for (const Row& row : rows) {
        const foc_motor_setup_t setup = setup_with_carrier(row.mode, row.inject_v, row.inject_hz);
        foc_motor_t motor{};
        const foc_err_t got = foc_motor_create_ex(row.params, &setup, &motor);
        EXPECT_EQ(got, row.expected) << "mode " << row.mode << ": " << row.why;
        if (got == FOC_OK && motor != nullptr) {
            EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
        }
    }
}
