// G6 - the C path end to end: lifecycle, pool exhaustion, stale/ABA handles, error mapping and the
// §13.5 dual-path isolation rule. This is a C++ TU on purpose: the *pure C* consumer is covered by
// test_package/c_api_c11.c, which also exercises the _Generic entry points.
// G6 - C 路径端到端：生命周期、池满、失效/ABA 句柄、错误码映射与 §13.5 双路径隔离。这里故意用
// C++ 编译单元；**纯 C** 消费者由 test_package/c_api_c11.c 覆盖，那里同时验证 _Generic 入口。

#include <cmath>
#include <string>

#include <gtest/gtest.h>

#include <foc/api/foc_api.h>
#include <foc/api/foc_types.h>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>



namespace {

/// [en] Owns a C handle so a failing assertion can never leak a pool slot.
/// [zh] 持有 C 句柄，保证断言失败时也不会泄漏池槽。
class CHandle final {
  public:
    CHandle() = default;
    ~CHandle() { reset(); }

    CHandle(const CHandle&) = delete;
    CHandle& operator=(const CHandle&) = delete;

    auto create() -> foc_err_t {
        const foc_motor_params_t params{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U};
        return foc_motor_create(&params, &handle_);
    }

    void reset() {
        if (handle_ != nullptr) {
            static_cast<void>(foc_motor_destroy(handle_));
            handle_ = nullptr;
        }
    }

    [[nodiscard]] auto get() const -> foc_motor_t { return handle_; }

  private:
    foc_motor_t handle_{nullptr};
};

auto params_of() -> foc_motor_params_t { return foc_motor_params_t{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U}; }

} // namespace



TEST(ApiC, LifecycleFromCreateToDestroy) {
    CHandle handle;
    ASSERT_EQ(FOC_OK, handle.create());
    ASSERT_NE(nullptr, handle.get());

    EXPECT_EQ(FOC_OK, foc_motor_enable(handle.get()));
    EXPECT_EQ(FOC_OK, foc_set_torque(handle.get(), 1.0F));

    for (int i = 0; i < 5; ++i) {
        EXPECT_EQ(FOC_OK, foc_tick(handle.get(), 1e-4F));
    }

    float speed = -1.0F;
    float id = -1.0F;
    float iq = -1.0F;
    EXPECT_EQ(FOC_OK, foc_get_state(handle.get(), &speed, &id, &iq));
    EXPECT_TRUE(std::isfinite(speed));
    EXPECT_TRUE(std::isfinite(id));
    EXPECT_TRUE(std::isfinite(iq));

    EXPECT_EQ(FOC_OK, foc_motor_disable(handle.get()));
    EXPECT_EQ(FOC_OK, foc_motor_destroy(handle.get()));
    handle.reset();
}



TEST(ApiC, NullArgumentsAreRejectedNotCrashed) {
    foc_motor_t out = nullptr;
    const foc_motor_params_t params = params_of();

    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_create(nullptr, &out));
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_create(&params, nullptr));
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_destroy(nullptr));
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_enable(nullptr));
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_disable(nullptr));
    EXPECT_EQ(FOC_ERR_PARAM, foc_set_speed(nullptr, 1.0F));
    EXPECT_EQ(FOC_ERR_PARAM, foc_set_torque(nullptr, 1.0F));
    EXPECT_EQ(FOC_ERR_PARAM, foc_tick(nullptr, 1e-4F));

    // Non-finite setpoints never reach the core.
    CHandle handle;
    ASSERT_EQ(FOC_OK, handle.create());
    EXPECT_EQ(FOC_ERR_PARAM, foc_set_speed(handle.get(), std::nanf("")));
    EXPECT_EQ(FOC_ERR_PARAM, foc_set_torque(handle.get(), std::nanf("")));
    EXPECT_EQ(FOC_ERR_PARAM, foc_tick(handle.get(), 0.0F));
    EXPECT_EQ(FOC_ERR_PARAM, foc_tick(handle.get(), -1e-4F));

    // The precondition of foc_get_state: at least one output pointer.
    EXPECT_EQ(FOC_ERR_PARAM, foc_get_state(handle.get(), nullptr, nullptr, nullptr));
}



TEST(ApiC, InvalidParametersAreRejectedAtCreate) {
    foc_motor_t out = nullptr;

    foc_motor_params_t bad = params_of();
    bad.ld = 0.0F;
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_create(&bad, &out));
    EXPECT_EQ(nullptr, out);

    bad = params_of();
    bad.pole_pairs = 0U;
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_create(&bad, &out));

    bad = params_of();
    bad.kt = -1.0F;
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_create(&bad, &out));

    bad = params_of();
    bad.rs = std::nanf("");
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_create(&bad, &out));
}



TEST(ApiC, EveryInvalidParameterShapeIsRejected) {
    foc_motor_t out = nullptr;
    // One case per clause of the parameter contract, so no branch of the validation is dead.
    const foc_motor_params_t bad[] = {
        {0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 0U},   // pole_pairs == 0
        {-1.0F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U},  // rs < 0
        {0.5F, 0.0F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U},    // ld == 0
        {0.5F, 4e-4F, 0.0F, 0.05F, 1e-4F, 1e-4F, 7U},    // lq == 0
        {0.5F, 4e-4F, 4e-4F, 0.0F, 1e-4F, 1e-4F, 7U},    // kt == 0
        {0.5F, 4e-4F, 4e-4F, 0.05F, 0.0F, 1e-4F, 7U},    // j == 0
        {0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, -1.0F, 7U},   // b < 0
        {0.5F, 4e-4F, INFINITY, 0.05F, 1e-4F, 1e-4F, 7U} // non-finite lq
    };
    for (const foc_motor_params_t& params : bad) {
        out = nullptr;
        EXPECT_EQ(FOC_ERR_PARAM, foc_motor_create(&params, &out));
        EXPECT_EQ(nullptr, out);
    }
}



TEST(ApiC, DisableBeforeEnableIsReportedNotLatched) {
    CHandle handle;
    ASSERT_EQ(FOC_OK, handle.create());

    // stop() from Idle is an illegal transition: reported, and the instance stays usable.
    EXPECT_EQ(FOC_ERR_STATE, foc_motor_disable(handle.get()));
    EXPECT_EQ(FOC_OK, foc_motor_enable(handle.get()));
    EXPECT_EQ(FOC_OK, foc_motor_disable(handle.get()));
}



TEST(ApiC, DoublePrecisionEntryPointsReachTheSameCore) {
    CHandle handle;
    ASSERT_EQ(FOC_OK, handle.create());

    EXPECT_EQ(FOC_OK, foc_set_speed_f64(handle.get(), 12.5));
    EXPECT_EQ(FOC_OK, foc_set_torque_f64(handle.get(), -1.5));
    EXPECT_EQ(FOC_ERR_PARAM, foc_set_speed_f64(handle.get(), INFINITY));
    EXPECT_EQ(FOC_ERR_PARAM, foc_set_torque_f64(handle.get(), -INFINITY));
}



TEST(ApiC, PoolExhaustsAndRecovers) {
    constexpr int kPoolSize = 4; // FOC_MAX_MOTORS in bridge.cpp

    CHandle handles[kPoolSize];
    for (CHandle& handle : handles) {
        ASSERT_EQ(FOC_OK, handle.create());
        ASSERT_NE(nullptr, handle.get());
    }

    foc_motor_t extra = nullptr;
    const foc_motor_params_t params = params_of();
    EXPECT_EQ(FOC_ERR_NO_RESOURCE, foc_motor_create(&params, &extra));
    EXPECT_EQ(nullptr, extra);

    handles[1].reset(); // free one slot
    EXPECT_EQ(FOC_OK, foc_motor_create(&params, &extra));
    ASSERT_NE(nullptr, extra);
    EXPECT_EQ(FOC_OK, foc_motor_destroy(extra));
}



TEST(ApiC, StaleHandleIsRejectedAfterDestroy) {
    CHandle handle;
    ASSERT_EQ(FOC_OK, handle.create());
    const foc_motor_t stale = handle.get();

    ASSERT_EQ(FOC_OK, foc_motor_enable(stale));
    ASSERT_EQ(FOC_OK, foc_motor_destroy(stale));

    // Every operation on the dead handle now reports an error instead of touching freed state.
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_enable(stale));
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_disable(stale));
    EXPECT_EQ(FOC_ERR_PARAM, foc_set_speed(stale, 1.0F));
    EXPECT_EQ(FOC_ERR_PARAM, foc_set_torque(stale, 1.0F));
    EXPECT_EQ(FOC_ERR_PARAM, foc_tick(stale, 1e-4F));
    float speed = 0.0F;
    EXPECT_EQ(FOC_ERR_PARAM, foc_get_state(stale, &speed, nullptr, nullptr));
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_destroy(stale)); // double free is an error, not a crash
    handle.reset();
}



TEST(ApiC, GenerationCounterRejectsTheAbausedHandle) {
    // Take the first handle, free it, then occupy the same slot again: the control block address
    // comes back (that is the point of a pool), so only the generation counter can tell the two
    // occupations apart.
    CHandle first;
    ASSERT_EQ(FOC_OK, first.create());
    const foc_motor_t old_handle = first.get();
    ASSERT_EQ(FOC_OK, foc_motor_destroy(old_handle));
    first.reset();

    CHandle second;
    ASSERT_EQ(FOC_OK, second.create());

    EXPECT_EQ(FOC_ERR_PARAM, foc_tick(old_handle, 1e-4F));
    EXPECT_EQ(FOC_OK, foc_motor_enable(second.get()));
}



TEST(ApiC, ForeignPointerIsRejected) {
    // A handle that never came from this pool must be reported, not dereferenced.
    auto* foreign = reinterpret_cast<foc_motor_t>(static_cast<std::uintptr_t>(0x1234U));
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_enable(foreign));
    EXPECT_EQ(FOC_ERR_PARAM, foc_tick(foreign, 1e-4F));
}



TEST(ApiC, ErrorCodesMirrorTheStateMachine) {
    CHandle handle;
    ASSERT_EQ(FOC_OK, handle.create());

    // Not started yet: the core reports State, and the C enum mirrors it one to one.
    EXPECT_EQ(FOC_ERR_STATE, foc_tick(handle.get(), 1e-4F));

    ASSERT_EQ(FOC_OK, foc_motor_enable(handle.get()));
    EXPECT_EQ(FOC_OK, foc_tick(handle.get(), 1e-4F));
}



TEST(ApiC, PoolAndCxxPathDoNotShareState) {
    // §13.5: the two paths have completely separate life cycles. Driving the C pool must not move
    // anything inside an independent C++ instance.
    foc::driver::StubHal hal;
    hal.dc_link.set_voltage_v(24.0F);
    hal.position.set_sensor_present(true);
    hal.position.set_angle_mech_rad(0.0F);
    hal.position.set_velocity_rad_s(0.0F);
    const foc::core::Uvw<float> zero{};
    ASSERT_EQ(foc::ErrorCode::Ok, hal.current_sense.load(&zero, 1U));

    const foc::motor::MotorConfig config{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-6F, 7U, 5.0F, 20000.0F, 500.0F};
    const foc::core::Pi::Config gains{1.0F, 0.0F, -12.0F, 12.0F, -12.0F, 12.0F};
    using CxxMotor =
        foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::driver::StubHal>;
    CxxMotor cxx(config,
                 CxxMotor::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{}, foc::core::Pi{gains},
                                    foc::core::Pi{gains}, foc::core::Pi{gains}},
                 hal);
    ASSERT_EQ(foc::ErrorCode::Ok, cxx.init());
    ASSERT_EQ(foc::ErrorCode::Ok, cxx.start());

    CHandle handle;
    ASSERT_EQ(FOC_OK, handle.create());
    ASSERT_EQ(FOC_OK, foc_motor_enable(handle.get()));
    ASSERT_EQ(FOC_OK, foc_set_speed(handle.get(), 50.0F));
    for (int i = 0; i < 20; ++i) {
        ASSERT_EQ(FOC_OK, foc_tick(handle.get(), 1e-4F));
    }
    float c_speed = 0.0F;
    ASSERT_EQ(FOC_OK, foc_get_state(handle.get(), &c_speed, nullptr, nullptr));

    // The C++ instance has not been ticked: its telemetry is still the initial one, untouched by
    // the twenty C ticks that just ran.
    EXPECT_FLOAT_EQ(0.0F, cxx.telemetry().iq_a);
    EXPECT_EQ(0U, hal.pwm.write_count());
}
