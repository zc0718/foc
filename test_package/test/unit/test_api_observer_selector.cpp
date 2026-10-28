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
    // The band the repository's own blend rig uses (`test_sil_observer_blend.cpp`): 60 to 120 rad/s with a 20 Hz
    // handover filter. Stated here rather than defaulted in the library, because a band is a property of the machine's
    // speed range and there is nothing sensible to guess.
    // 仓库自己的混合台架（`test_sil_observer_blend.cpp`）用的交接带：60 到 120 rad/s，交接滤波 20 Hz。此处在测试里
    // 写明而不是在库里给默认值，因为交接带属于机器的转速范围，没有什么可以合理猜测的。
    setup.blend_start_rad_s = 60.0F;
    setup.blend_end_rad_s = 120.0F;
    setup.handover_lpf_hz = 20.0F;
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
    // The lock verdict travels with the status, because an observer holding its last angle is indistinguishable from a
    // stopped machine by its angle alone. This rig feeds zero current, so the injection observer has nothing to lock on
    // and must say so; `observer_error` is then exactly zero, which is why the two are reported together.
    // 锁定结论随状态一起返回，因为一个正保持上一个角度的观测器仅凭角度与一台停住的机器无法区分。本台架喂的是零电流，注入
    // 观测器没有可锁的东西、必须说出来；`observer_error` 此时严格为零，这也是两者要一起报的原因。
    EXPECT_EQ(status.observer_locked, 0) << "an injection observer with no current reported a lock";
    EXPECT_FLOAT_EQ(0.0F, status.observer_error);
    EXPECT_EQ(foc_motor_destroy(hfi_motor), FOC_OK);
}

TEST(ApiObserverSelector, ACarrierTheControlRateCannotResolveIsRefusedAtSetup) {
    // The fourth fact HFI is admitted on, and the only one about the *rate* rather than the machine: a carrier needs
    // `kMinSamplesPerCarrier` control periods per cycle to be resolvable. Below that the observer is not noisy, it is
    // dead -- the demodulated amplitude collapses, the lock drops out, the angle freezes and the speed reads zero, which
    // a loop cannot tell from a stalled rotor (measured: -19.5 rad/s of speed bias on the SIL fixture).
    // 第四个放行 HFI 的事实，也是唯一关于**控制率**而非机器的那一个：载波每个周期需要 `kMinSamplesPerCarrier` 个控制
    // 周期才分辨得出来。低于此观测器不是变吵，而是死了——解调幅值塌掉、锁定脱落、角度冻结、转速读零，环无法把它与堵转
    // 区分开（SIL 台架实测转速偏差 -19.5 rad/s）。
    //
    // Refusing at setup rather than at the first tick is the point: `foc_motor_create_ex` is where a caller can still do
    // something else, and an observer that froze mid-run would only be discovered as a runaway axis.
    // 在装配时而不是在第一个 tick 上拒绝才是要点：`foc_motor_create_ex` 是调用方还能改主意的位置，而一个运行中冻结的
    // 观测器只会以一条跑飞的轴的形式被发现。
    const foc_motor_params_t salient = salient_params();
    const std::int32_t hfi = static_cast<std::int32_t>(FOC_OBSERVER_HFI);
    foc_motor_t motor{};

    // 9200 Hz at 20 kHz is 2.17 control periods per carrier period.
    // 20 kHz 上的 9200 Hz 是每个载波周期 2.17 个控制周期。
    const foc_motor_setup_t too_fast = setup_with_carrier(hfi, 12.0F, 9200.0F);
    EXPECT_EQ(foc_motor_create_ex(&salient, &too_fast, &motor), FOC_ERR_PARAM)
        << "a carrier the control rate cannot resolve was accepted";
    EXPECT_EQ(motor, nullptr);

    // Exactly at the ceiling is admitted: 8000 Hz at 20 kHz is 2.50 periods per cycle, the value the SIL envelope
    // certifies. An inequality that refused its own boundary would reject the measured-good configuration.
    // 正好在天花板上是准入的：20 kHz 上的 8000 Hz 是每周期 2.50 个样本，即 SIL 包线认证过的那个值。一条拒绝自身边界的
    // 判据会把实测良好的配置挡在外面。
    const foc_motor_setup_t at_ceiling = setup_with_carrier(hfi, 12.0F, 8000.0F);
    EXPECT_EQ(foc_motor_create_ex(&salient, &at_ceiling, &motor), FOC_OK) << "the ceiling itself was refused";
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);

    // And the same 9200 Hz is admitted once the rate can carry it, which is what makes this a rule about the rate and
    // not a ban on the carrier.
    // 而一旦频率能承载它，同一个 9200 Hz 就被准入，这正是它是一条关于频率的规则、而不是对载波的禁令的原因。
    foc_motor_setup_t faster = setup_with_carrier(hfi, 12.0F, 9200.0F);
    faster.pwm_freq_hz = 40000.0F;
    EXPECT_EQ(foc_motor_create_ex(&salient, &faster, &motor), FOC_OK) << "9200 Hz refused at 40 kHz, where it is fine";
    ASSERT_NE(motor, nullptr);
    EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
}

TEST(ApiObserverSelector, EveryModeInTheEnumerationIsWiredAndEachIsAdmittedOnFacts) {
    // One row per way a request can be accepted or refused, so that a change in what the library promises is a change to
    // this table rather than a discovery made by a user. The refusals are the interesting rows: each names a fact about
    // the machine or the request, and none of them falls back to the default mode.
    // 请求被接受或被拒绝的每一种情形各占一行，于是"库承诺了什么"的变化是这张表的改动，而不是由用户发现。拒绝的那几行才是
    // 重点：每一行都指向一个关于机器或请求的事实，且没有任何一行退回默认模式。
    const foc_motor_params_t surface = params_struct();
    const foc_motor_params_t salient = salient_params();

    struct Row final {
        std::int32_t mode;
        const foc_motor_params_t* params;
        float inject_v;
        float inject_hz;
        float band_end; // 0 means "leave the helper's order; a negative value makes the band backwards"
        foc_err_t expected;
        const char* why;
    };
    const Row rows[] = {
        {static_cast<std::int32_t>(FOC_OBSERVER_BACK_EMF), &surface, 0.0F, 0.0F, 120.0F, FOC_OK, "the default"},
        {static_cast<std::int32_t>(FOC_OBSERVER_NONLINEAR_FLUX), &surface, 0.0F, 0.0F, 120.0F, FOC_OK,
         "wired, nameplate is enough"},
        {static_cast<std::int32_t>(FOC_OBSERVER_HFI), &salient, 12.0F, 2000.0F, 120.0F, FOC_OK, "wired on saliency"},
        {static_cast<std::int32_t>(FOC_OBSERVER_HFI), &surface, 12.0F, 2000.0F, 120.0F, FOC_ERR_PARAM,
         "surface machine has no saliency to measure"},
        {static_cast<std::int32_t>(FOC_OBSERVER_HFI), &salient, 0.0F, 0.0F, 120.0F, FOC_ERR_PARAM, "no carrier"},
        {static_cast<std::int32_t>(FOC_OBSERVER_HFI), &salient, 12.0F, 300.0F, 120.0F, FOC_ERR_PARAM,
         "carrier at the demodulation cutoff"},
        {static_cast<std::int32_t>(FOC_OBSERVER_BLENDED), &salient, 12.0F, 2000.0F, 120.0F, FOC_OK,
         "wired; needs what HFI needs plus a band"},
        {static_cast<std::int32_t>(FOC_OBSERVER_BLENDED), &salient, 12.0F, 2000.0F, 0.0F, FOC_ERR_PARAM,
         "band backwards: end below start"},
        {static_cast<std::int32_t>(FOC_OBSERVER_BLENDED), &surface, 12.0F, 2000.0F, 120.0F, FOC_ERR_PARAM,
         "a blend whose low half cannot work"},
        {static_cast<std::int32_t>(FOC_OBSERVER_BLENDED), &salient, 0.0F, 0.0F, 120.0F, FOC_ERR_PARAM, "no carrier"},
        {99, &surface, 0.0F, 0.0F, 120.0F, FOC_ERR_PARAM, "outside the enumeration"},
    };
    for (const Row& row : rows) {
        foc_motor_setup_t setup = setup_with_carrier(row.mode, row.inject_v, row.inject_hz);
        setup.blend_end_rad_s = row.band_end;
        foc_motor_t motor{};
        const foc_err_t got = foc_motor_create_ex(row.params, &setup, &motor);
        EXPECT_EQ(got, row.expected) << "mode " << row.mode << ": " << row.why;
        if (got == FOC_OK) {
            ASSERT_NE(motor, nullptr);
            foc_motor_status_t status{};
            status.struct_size = sizeof(foc_motor_status_t);
            EXPECT_EQ(foc_motor_get_status(motor, &status), FOC_OK);
            EXPECT_EQ(status.observer, row.mode) << "mode " << row.mode << " does not report itself";
            EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
        }
    }
}

TEST(ApiObserverSelector, AModeCanBeChangedOnAMotorThatIsAlreadyRunning) {
    // The run-time hand-over had to exist as a function rather than a field because the incoming observer has to be
    // seeded from the outgoing estimate first. This case only pins the C contract -- which answers, and what the status
    // then reports. The numerical property (that the estimate does not jump) is `SwitchableObserver.
    // AHandOverCarriesTheEstimateAndASwitchDoesNot` at the C++ level and `SilFusionChain` at the plant level; asserting
    // it here would need an angle the C API does not expose, and inventing one accessor just for this would be worse
    // than naming where the claim is tested.
    // 运行时交接必须是一个函数而不是一个字段，因为接手的观测器要先被交出者的估计播种。本用例只钉 C 契约——哪些回答、
    // 之后状态报出什么。数值性质（估计不跳）由 C++ 层的 `SwitchableObserver.AHandOverCarriesTheEstimateAndASwitchDoesNot`
    // 与 plant 层的 `SilFusionChain` 负责；在这里断言它需要一个 C API 并未暴露的角度，为此外加一个访问器比指明这项
    // 承诺在哪里被测试更糟。
    const foc_motor_params_t params = salient_params();
    const foc_motor_setup_t setup = setup_with_carrier(static_cast<std::int32_t>(FOC_OBSERVER_BACK_EMF), 12.0F, 2000.0F);

    foc_motor_t motor{};
    ASSERT_EQ(foc_motor_create_ex(&params, &setup, &motor), FOC_OK);
    ASSERT_NE(motor, nullptr);

    // Idle: a hand-over would carry an estimate no observer has produced yet.
    EXPECT_EQ(foc_motor_set_observer(motor, FOC_OBSERVER_HFI), FOC_ERR_STATE) << "a motor that never ran has nothing to carry";

    ASSERT_EQ(foc_motor_enable(motor), FOC_OK);
    ASSERT_EQ(foc_set_torque(motor, 0.0F), FOC_OK);
    for (int tick = 0; tick < 20; ++tick) {
        ASSERT_EQ(foc_tick(motor, 1.0F / 20000.0F), FOC_OK);
    }

    EXPECT_EQ(foc_motor_set_observer(motor, FOC_OBSERVER_HFI), FOC_OK);
    foc_motor_status_t status{};
    status.struct_size = sizeof(foc_motor_status_t);
    ASSERT_EQ(foc_motor_get_status(motor, &status), FOC_OK);
    EXPECT_EQ(status.observer, static_cast<std::int32_t>(FOC_OBSERVER_HFI));

    // Handing over to the mode that is already running is not an error, and changes nothing.
    // 交接给正在运行的模式不是错误，也不改变任何东西。
    EXPECT_EQ(foc_motor_set_observer(motor, FOC_OBSERVER_HFI), FOC_OK);
    EXPECT_EQ(foc_motor_get_status(motor, &status), FOC_OK);
    EXPECT_EQ(status.observer, static_cast<std::int32_t>(FOC_OBSERVER_HFI));

    // And back the other way, which is the direction that carries a seed.
    // 再换回来，才是需要携带种子的那个方向。
    EXPECT_EQ(foc_motor_set_observer(motor, FOC_OBSERVER_BACK_EMF), FOC_OK);
    EXPECT_EQ(foc_motor_get_status(motor, &status), FOC_OK);
    EXPECT_EQ(status.observer, static_cast<std::int32_t>(FOC_OBSERVER_BACK_EMF));

    EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
}

TEST(ApiObserverSelector, ARunTimeHandOverIsRefusedWhenThisMachineCannotRunTheMode) {
    // The same admission predicate as construction, applied to the machine that is actually running: switching to HFI on
    // a surface machine must be refused rather than accepted and then quietly doing nothing usable.
    // 与构造时相同的准入判据，作用在实际运行的那台机器上：在表贴机上切到 HFI 必须被拒绝，而不是先接受、之后又悄悄做不到。
    const foc_motor_params_t surface = params_struct();
    const foc_motor_setup_t setup =
        setup_with_carrier(static_cast<std::int32_t>(FOC_OBSERVER_BACK_EMF), 12.0F, 2000.0F);

    foc_motor_t motor{};
    ASSERT_EQ(foc_motor_create_ex(&surface, &setup, &motor), FOC_OK);
    ASSERT_NE(motor, nullptr);
    ASSERT_EQ(foc_motor_enable(motor), FOC_OK);
    ASSERT_EQ(foc_set_torque(motor, 0.0F), FOC_OK);
    for (int tick = 0; tick < 10; ++tick) {
        ASSERT_EQ(foc_tick(motor, 1.0F / 20000.0F), FOC_OK);
    }

    EXPECT_EQ(foc_motor_set_observer(motor, FOC_OBSERVER_HFI), FOC_ERR_PARAM) << "no saliency, no injection observer";
    foc_motor_status_t status{};
    status.struct_size = sizeof(foc_motor_status_t);
    ASSERT_EQ(foc_motor_get_status(motor, &status), FOC_OK);
    EXPECT_EQ(status.observer, static_cast<std::int32_t>(FOC_OBSERVER_BACK_EMF)) << "a refused hand-over changed the mode anyway";

    // A value outside the enumeration is a caller error, checked before anything is touched.
    // 枚举之外的取值是调用方错误，在触碰任何东西之前就被挡下。
    EXPECT_EQ(foc_motor_set_observer(motor, static_cast<foc_observer_t>(99)), FOC_ERR_PARAM);
    EXPECT_EQ(foc_motor_set_observer(nullptr, FOC_OBSERVER_BACK_EMF), FOC_ERR_PARAM);
    EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);
}

TEST(ApiObserverSelector, TheSizingItSuggestsIsAConfigurationItAccepts) {
    // A sizing tool that recommends a configuration the library then refuses would be worse than no tool, so the
    // property under test is not "the numbers look reasonable" but "the library takes them". Everything else here is the
    // arithmetic being pinned: the carrier is the rate's ceiling, and the amplitude is the volts that draw the HF current
    // the caller budgets, `I_hf * w_h * Ld`.
    // 一个建议出库随后会拒绝的配置的定容工具，比没有工具更糟，因此被测的性质不是"数字看起来合理"，而是"库接受它们"。
    // 其余都是钉住算术：载波是该频率的天花板，幅值是拉出调用方预算的高频电流所需的伏特 `I_hf·ω_h·Ld`。
    const foc_motor_params_t salient = salient_params(); // ld 4e-4, lq 8e-4
    foc_hfi_sizing_t sizing{};
    sizing.struct_size = sizeof(foc_hfi_sizing_t);

    ASSERT_EQ(foc_hfi_size(&salient, 20000.0F, 0.5F, &sizing), FOC_OK);
    EXPECT_FLOAT_EQ(8000.0F, sizing.inject_hz) << "the carrier is not the ceiling the rate admits";
    // 0.5 A at 8 kHz through 0.4 mH: 0.5 * 2*pi*8000 * 4e-4 = 10.05 V.
    // 8 kHz 下 0.5 A 穿过 0.4 mH：0.5 × 2π×8000 × 4e-4 = 10.05 V。
    EXPECT_NEAR(0.5F * 2.0F * 3.14159265F * 8000.0F * 4.0e-4F, sizing.inject_v, 0.05F);
    EXPECT_FLOAT_EQ(0.5F, sizing.hf_current_a);
    EXPECT_GT(sizing.demod_lpf_hz, 0.0F);
    EXPECT_LT(sizing.demod_lpf_hz, sizing.inject_hz);
    EXPECT_GT(sizing.speed_lpf_hz, 0.0F);

    // The property that matters: what it describes, the library runs.
    // 要紧的性质：它描述的配置，库会跑。
    foc_motor_setup_t setup = setup_with_observer(static_cast<std::int32_t>(FOC_OBSERVER_HFI));
    setup.inject_v = sizing.inject_v;
    setup.inject_hz = sizing.inject_hz;
    foc_motor_t motor{};
    ASSERT_EQ(foc_motor_create_ex(&salient, &setup, &motor), FOC_OK)
        << "the sizing tool recommended a configuration the library refuses";
    ASSERT_NE(motor, nullptr);
    ASSERT_EQ(foc_motor_enable(motor), FOC_OK);
    for (int tick = 0; tick < 20; ++tick) {
        ASSERT_EQ(foc_tick(motor, 1.0F / 20000.0F), FOC_OK);
    }
    foc_motor_status_t status{};
    status.struct_size = sizeof(foc_motor_status_t);
    ASSERT_EQ(foc_motor_get_status(motor, &status), FOC_OK);
    EXPECT_EQ(status.observer, static_cast<std::int32_t>(FOC_OBSERVER_HFI));
    EXPECT_EQ(foc_motor_destroy(motor), FOC_OK);

    // And the three refusals, each a fact about the request rather than a preference.
    // 以及三种拒绝，每一条都是关于请求的事实而不是偏好。
    const foc_motor_params_t surface = params_struct(); // ld == lq
    EXPECT_EQ(foc_hfi_size(&surface, 20000.0F, 0.5F, &sizing), FOC_ERR_PARAM) << "a surface machine was sized";
    EXPECT_EQ(foc_hfi_size(&salient, 0.0F, 0.5F, &sizing), FOC_ERR_PARAM) << "a zero control rate was sized";
    EXPECT_EQ(foc_hfi_size(&salient, 20000.0F, 0.0F, &sizing), FOC_ERR_PARAM) << "a zero current budget was sized";
}

