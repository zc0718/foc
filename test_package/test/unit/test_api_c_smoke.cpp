// G6 - the C path end to end: lifecycle, pool exhaustion, stale/ABA handles,
// error mapping and the §13.5 dual-path isolation rule. This is a C++ TU on
// purpose: the *pure C* consumer is covered by test_package/c_api_c11.c, which
// also exercises the _Generic entry points. G6 - C
// 路径端到端：生命周期、池满、失效/ABA 句柄、错误码映射与 §13.5
// 双路径隔离。这里故意用 C++ 编译单元；**纯 C** 消费者由
// test_package/c_api_c11.c 覆盖，那里同时验证 _Generic 入口。

#include <cmath>
#include <cstddef>
#include <cstdint>
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

  CHandle(const CHandle &) = delete;
  CHandle &operator=(const CHandle &) = delete;

  auto create() -> foc_err_t {
    const foc_motor_params_t params{0.5F,  4e-4F, 4e-4F, 0.05F,
                                    1e-4F, 1e-4F, 7U};
    return foc_motor_create(&params, &handle_);
  }

  auto create_ex(const foc_motor_setup_t *setup) -> foc_err_t {
    const foc_motor_params_t params{0.5F,  4e-4F, 4e-4F, 0.05F,
                                    1e-4F, 1e-4F, 7U};
    return foc_motor_create_ex(&params, setup, &handle_);
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

auto params_of() -> foc_motor_params_t {
  return foc_motor_params_t{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-4F, 1e-4F, 7U};
}

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

TEST(ApiC, ClearFaultValidatesTheHandleAndTheState) {
  EXPECT_EQ(FOC_ERR_PARAM, foc_motor_clear_fault(nullptr));

  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create());

  // A healthy motor is not in Fault: clearing is a state error, never a silent
  // success.
  EXPECT_EQ(FOC_ERR_STATE, foc_motor_clear_fault(handle.get()));
  EXPECT_EQ(FOC_OK, foc_motor_enable(handle.get()));
  EXPECT_EQ(FOC_ERR_STATE, foc_motor_clear_fault(handle.get()));
}

TEST(ApiC, CreateExWithNoSetupUsesTheDocumentedDefaults) {
  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create_ex(nullptr));

  foc_motor_setup_t applied{};
  ASSERT_EQ(FOC_OK, foc_motor_get_setup(handle.get(), &applied));
  EXPECT_EQ(sizeof(foc_motor_setup_t), applied.struct_size);
  EXPECT_FLOAT_EQ(5.0F, applied.i_max_a);
  EXPECT_FLOAT_EQ(20'000.0F, applied.pwm_freq_hz);
  EXPECT_FLOAT_EQ(500.0F, applied.deadtime_ns);
  EXPECT_FLOAT_EQ(24.0F, applied.v_bus_v);
  EXPECT_FLOAT_EQ(2'000.0F, applied.current_bandwidth_rad_s);
  EXPECT_FLOAT_EQ(200.0F, applied.speed_bandwidth_rad_s);
  EXPECT_FLOAT_EQ(0.0F, applied.speed_loop_period_s);
}

TEST(ApiC, CreateExAppliesEveryFieldItWasGiven) {
  foc_motor_setup_t requested{};
  requested.struct_size = sizeof(foc_motor_setup_t);
  requested.i_max_a = 2.0F;
  requested.pwm_freq_hz = 10'000.0F;
  requested.deadtime_ns =
      0.0F; // literal zero: an ideal inverter, not "use the default"
  requested.v_bus_v = 48.0F;
  requested.current_bandwidth_rad_s = 1'000.0F;
  requested.speed_bandwidth_rad_s = 50.0F;
  requested.speed_loop_period_s = 5e-4F;

  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create_ex(&requested));

  foc_motor_setup_t applied{};
  ASSERT_EQ(FOC_OK, foc_motor_get_setup(handle.get(), &applied));
  EXPECT_FLOAT_EQ(2.0F, applied.i_max_a);
  EXPECT_FLOAT_EQ(10'000.0F, applied.pwm_freq_hz);
  EXPECT_FLOAT_EQ(0.0F, applied.deadtime_ns);
  EXPECT_FLOAT_EQ(48.0F, applied.v_bus_v);
  EXPECT_FLOAT_EQ(1'000.0F, applied.current_bandwidth_rad_s);
  EXPECT_FLOAT_EQ(50.0F, applied.speed_bandwidth_rad_s);
  EXPECT_FLOAT_EQ(5e-4F, applied.speed_loop_period_s);
}

TEST(ApiC, APartialSetupStructOnlyOverridesWhatItCovers) {
  foc_motor_setup_t requested{};
  // Covers exactly up to (and including) deadtime_ns; v_bus_v and later are
  // outside it.
  requested.struct_size =
      static_cast<std::uint32_t>(offsetof(foc_motor_setup_t, v_bus_v));
  requested.i_max_a = 2.0F;
  requested.pwm_freq_hz = 10'000.0F;
  requested.deadtime_ns = 250.0F;
  requested.v_bus_v = 48.0F; // must be ignored: outside struct_size
  requested.speed_bandwidth_rad_s = 50.0F;

  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create_ex(&requested));

  foc_motor_setup_t applied{};
  ASSERT_EQ(FOC_OK, foc_motor_get_setup(handle.get(), &applied));
  EXPECT_FLOAT_EQ(2.0F, applied.i_max_a);
  EXPECT_FLOAT_EQ(10'000.0F, applied.pwm_freq_hz);
  EXPECT_FLOAT_EQ(250.0F, applied.deadtime_ns);
  EXPECT_FLOAT_EQ(24.0F, applied.v_bus_v); // default kept
  EXPECT_FLOAT_EQ(2'000.0F, applied.current_bandwidth_rad_s);
  EXPECT_FLOAT_EQ(200.0F, applied.speed_bandwidth_rad_s);
}

TEST(ApiC, AZeroSizedSetupStructChangesNothing) {
  foc_motor_setup_t requested{};
  requested.struct_size = 0U;
  requested.i_max_a = 2.0F; // not covered by struct_size, so not read

  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create_ex(&requested));

  foc_motor_setup_t applied{};
  ASSERT_EQ(FOC_OK, foc_motor_get_setup(handle.get(), &applied));
  EXPECT_FLOAT_EQ(5.0F, applied.i_max_a);
}

TEST(ApiC, AnInvalidSetupFieldIsRejectedInsteadOfSilentlyDefaulted) {
  foc_motor_setup_t requested{};
  requested.struct_size = sizeof(foc_motor_setup_t);
  requested.i_max_a = -1.0F;

  CHandle handle;
  EXPECT_EQ(FOC_ERR_PARAM, handle.create_ex(&requested));
  EXPECT_EQ(nullptr, handle.get());

  // Dead time is taken literally, so a negative one is a parameter error, not
  // "use 500 ns".
  requested.i_max_a = 1.0F;
  requested.deadtime_ns = -1.0F;
  EXPECT_EQ(FOC_ERR_PARAM, handle.create_ex(&requested));
}

TEST(ApiC, GetSetupValidatesItsArguments) {
  foc_motor_setup_t applied{};
  EXPECT_EQ(FOC_ERR_PARAM, foc_motor_get_setup(nullptr, &applied));

  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create());
  EXPECT_EQ(FOC_ERR_PARAM, foc_motor_get_setup(handle.get(), nullptr));
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
  EXPECT_EQ(FOC_ERR_PARAM,
            foc_get_state(handle.get(), nullptr, nullptr, nullptr));
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
  // One case per clause of the parameter contract, so no branch of the
  // validation is dead.
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
  for (const foc_motor_params_t &params : bad) {
    out = nullptr;
    EXPECT_EQ(FOC_ERR_PARAM, foc_motor_create(&params, &out));
    EXPECT_EQ(nullptr, out);
  }
}

TEST(ApiC, DisableBeforeEnableIsReportedNotLatched) {
  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create());

  // stop() from Idle is an illegal transition: reported, and the instance stays
  // usable.
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
  for (CHandle &handle : handles) {
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

  // Every operation on the dead handle now reports an error instead of touching
  // freed state.
  EXPECT_EQ(FOC_ERR_PARAM, foc_motor_enable(stale));
  EXPECT_EQ(FOC_ERR_PARAM, foc_motor_disable(stale));
  EXPECT_EQ(FOC_ERR_PARAM, foc_set_speed(stale, 1.0F));
  EXPECT_EQ(FOC_ERR_PARAM, foc_set_torque(stale, 1.0F));
  EXPECT_EQ(FOC_ERR_PARAM, foc_tick(stale, 1e-4F));
  float speed = 0.0F;
  EXPECT_EQ(FOC_ERR_PARAM, foc_get_state(stale, &speed, nullptr, nullptr));
  EXPECT_EQ(FOC_ERR_PARAM,
            foc_motor_destroy(stale)); // double free is an error, not a crash
  handle.reset();
}

TEST(ApiC, GenerationCounterRejectsTheAbausedHandle) {
  // Take the first handle, free it, then occupy the same slot again: the
  // control block address comes back (that is the point of a pool), so only the
  // generation counter can tell the two occupations apart.
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
  auto *foreign =
      reinterpret_cast<foc_motor_t>(static_cast<std::uintptr_t>(0x1234U));
  EXPECT_EQ(FOC_ERR_PARAM, foc_motor_enable(foreign));
  EXPECT_EQ(FOC_ERR_PARAM, foc_tick(foreign, 1e-4F));
}

TEST(ApiC, ErrorCodesMirrorTheStateMachine) {
  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create());

  // Not started yet: the core reports State, and the C enum mirrors it one to
  // one.
  EXPECT_EQ(FOC_ERR_STATE, foc_tick(handle.get(), 1e-4F));

  ASSERT_EQ(FOC_OK, foc_motor_enable(handle.get()));
  EXPECT_EQ(FOC_OK, foc_tick(handle.get(), 1e-4F));
}

TEST(ApiC, PoolAndCxxPathDoNotShareState) {
  // §13.5: the two paths have completely separate life cycles. Driving the C
  // pool must not move anything inside an independent C++ instance.
  foc::driver::StubHal hal;
  hal.dc_link.set_voltage_v(24.0F);
  hal.position.set_sensor_present(true);
  hal.position.set_angle_mech_rad(0.0F);
  hal.position.set_velocity_rad_s(0.0F);
  const foc::core::Uvw<float> zero{};
  ASSERT_EQ(foc::ErrorCode::Ok, hal.current_sense.load(&zero, 1U));

  const foc::motor::MotorConfig config{0.5F,  4e-4F, 4e-4F, 0.05F,    1e-4F,
                                       1e-6F, 7U,    5.0F,  20000.0F, 500.0F};
  const foc::core::Pi::Config gains{1.0F, 0.0F, -12.0F, 12.0F, -12.0F, 12.0F};
  using CxxMotor =
      foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm,
                        foc::core::Pi, foc::driver::StubHal>;
  CxxMotor cxx(config,
               CxxMotor::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}),
                                  foc::core::Svpwm{}, foc::core::Pi{gains},
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

  // The C++ instance has not been ticked: its telemetry is still the initial
  // one, untouched by the twenty C ticks that just ran.
  EXPECT_FLOAT_EQ(0.0F, cxx.telemetry().iq_a);
  EXPECT_EQ(0U, hal.pwm.write_count());
}

TEST(ApiC, ThePositionAndAlignmentFieldsAreResolvedAndReported) {
  // The C path is the whole motor now: position loop and aligner included.
  // Everything it runs with has to be readable back, because a C caller has no
  // other way to find out which default it got. C
  // 路径现在是完整电机：位置环与对齐器都在。它实际运行的每一项都必须能读回，因为
  // C 调用方没有 别的办法知道自己拿到了哪个默认值。
  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create());

  foc_motor_setup_t applied{};
  ASSERT_EQ(FOC_OK, foc_motor_get_setup(handle.get(), &applied));
  EXPECT_EQ(static_cast<uint32_t>(sizeof(foc_motor_setup_t)),
            applied.struct_size);
  EXPECT_GT(applied.position_bandwidth_rad_s, 0.0F);
  EXPECT_GT(applied.velocity_limit_rad_s, 0.0F);
  EXPECT_FLOAT_EQ(0.0F, applied.position_loop_period_s);
  // Alignment is off unless asked for: 0 current is a real value, not "use the
  // default". 除非明确要求，对齐是关闭的：0 电流是真实取值，不是"用默认值"。
  EXPECT_FLOAT_EQ(0.0F, applied.align_current_a);

  foc_motor_setup_t requested{};
  requested.struct_size = sizeof(foc_motor_setup_t);
  requested.position_bandwidth_rad_s = 35.0F;
  requested.velocity_limit_rad_s = 12.0F;
  requested.position_loop_period_s = 1e-3F;
  requested.align_current_a = 0.4F;
  requested.align_hold_s = 0.15F;
  requested.align_tolerance_rad = 0.02F;
  requested.align_verify_s = 0.05F;

  CHandle tuned;
  ASSERT_EQ(FOC_OK, tuned.create_ex(&requested));
  foc_motor_setup_t got{};
  ASSERT_EQ(FOC_OK, foc_motor_get_setup(tuned.get(), &got));
  EXPECT_FLOAT_EQ(35.0F, got.position_bandwidth_rad_s);
  EXPECT_FLOAT_EQ(12.0F, got.velocity_limit_rad_s);
  EXPECT_FLOAT_EQ(1e-3F, got.position_loop_period_s);
  EXPECT_FLOAT_EQ(0.4F, got.align_current_a);
  EXPECT_FLOAT_EQ(0.15F, got.align_hold_s);
  EXPECT_FLOAT_EQ(0.02F, got.align_tolerance_rad);
  EXPECT_FLOAT_EQ(0.05F, got.align_verify_s);
}

TEST(ApiC, AHalfConfiguredAlignmentIsRejected) {
  // "Do not align" has to be the whole recipe. A hold time without a current is
  // almost certainly a typo, and silently aligning nothing would hide it until
  // the first move on a real shaft.
  // "不对齐"必须是整套配方。给了保持时间却没给电流几乎肯定是笔误，静默地什么都不对齐只会把这个
  // 笔误藏到真机第一次运动的时候。
  foc_motor_setup_t requested{};
  requested.struct_size = sizeof(foc_motor_setup_t);
  requested.align_hold_s = 0.2F; // but align_current_a stays 0

  CHandle handle;
  EXPECT_EQ(FOC_ERR_PARAM, handle.create_ex(&requested));

  // And with a current, the hold window and tolerance become mandatory.
  // 反过来，给了电流之后，保持窗口与容差变成必填。
  requested.align_current_a = 0.5F;
  requested.align_hold_s = 0.0F;
  EXPECT_EQ(FOC_ERR_PARAM, handle.create_ex(&requested));

  requested.align_hold_s = 0.2F; // tolerance still 0
  EXPECT_EQ(FOC_ERR_PARAM, handle.create_ex(&requested));
}

TEST(ApiC, WithoutAlignmentTheMotorStartsStraightIntoRun) {
  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create());
  ASSERT_EQ(FOC_OK, foc_motor_enable(handle.get()));

  foc_motor_status_t status{};
  ASSERT_EQ(FOC_OK, foc_motor_get_status(handle.get(), &status));
  EXPECT_EQ(static_cast<int32_t>(FOC_MOTOR_RUN), status.mode_state);
  EXPECT_EQ(static_cast<int32_t>(FOC_ALIGN_IDLE), status.align_state);
  EXPECT_EQ(static_cast<int32_t>(FOC_ALIGN_NO_FAILURE), status.align_failure);
  EXPECT_EQ(FOC_OK, status.error);
  EXPECT_EQ(static_cast<uint32_t>(sizeof(foc_motor_status_t)),
            status.struct_size);
}

TEST(ApiC, WithARecordedAlignmentTheMotorParksInAlignAndThenRuns) {
  foc_motor_setup_t requested{};
  requested.struct_size = sizeof(foc_motor_setup_t);
  requested.align_current_a = 0.5F;
  requested.align_hold_s = 0.005F;
  requested.align_tolerance_rad = 0.05F;

  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create_ex(&requested));
  ASSERT_EQ(FOC_OK, foc_motor_enable(handle.get()));

  foc_motor_status_t status{};
  ASSERT_EQ(FOC_OK, foc_motor_get_status(handle.get(), &status));
  EXPECT_EQ(static_cast<int32_t>(FOC_MOTOR_ALIGN), status.mode_state);

  // The stub sensor reports a fixed angle, so the sequence completes as soon as
  // the hold window elapses and the offset it captures is that angle.
  // 桩传感器报固定角度，因此保持窗口一过流程即完成，捕获到的偏置就是那个角度。
  for (int i = 0;
       i < 200 && status.mode_state == static_cast<int32_t>(FOC_MOTOR_ALIGN);
       ++i) {
    ASSERT_EQ(FOC_OK, foc_tick(handle.get(), 1e-4F));
    ASSERT_EQ(FOC_OK, foc_motor_get_status(handle.get(), &status));
  }
  EXPECT_EQ(static_cast<int32_t>(FOC_MOTOR_RUN), status.mode_state);
  EXPECT_EQ(static_cast<int32_t>(FOC_ALIGN_DONE), status.align_state);
  EXPECT_FLOAT_EQ(0.0F,
                  status.align_offset_rad); // the stub sensor sits at zero
}

TEST(ApiC, PositionAndCommandAreValidatedBeforeTheyReachTheLoop) {
  CHandle handle;
  ASSERT_EQ(FOC_OK, handle.create());

  EXPECT_EQ(FOC_ERR_PARAM, foc_set_position(nullptr, 0.1F));
  EXPECT_EQ(FOC_ERR_PARAM, foc_set_position(handle.get(), std::nanf("")));
  EXPECT_EQ(FOC_OK, foc_set_position(handle.get(), 0.1F));
  EXPECT_EQ(FOC_OK, foc_set_position_f64(handle.get(), 0.2));

  EXPECT_EQ(FOC_ERR_PARAM, foc_set_command(handle.get(), nullptr));
  foc_motor_command_t command{0.5F, -1.0F, 0.0F};
  // A negative ceiling would invert the clamp rather than bound it.
  // 负的限幅会把钳位反号，而不是限住它。
  EXPECT_EQ(FOC_ERR_PARAM, foc_set_command(handle.get(), &command));

  command.max_current_a = 1.0F;
  EXPECT_EQ(FOC_OK, foc_set_command(handle.get(), &command));

  // The per-move speed and acceleration ceilings follow the same rule: 0 means
  // "no override", and a negative value is a parameter error rather than a
  // limit that runs backwards. 按次的速度与加速度上限遵循同一条规则：0
  // 表示"不覆盖"，负值是参数错误而不是反向的限制。
  command.max_velocity_rad_s = -1.0F;
  EXPECT_EQ(FOC_ERR_PARAM, foc_set_command(handle.get(), &command));
  command.max_velocity_rad_s = 2.0F;
  command.max_accel_rad_s2 = -1.0F;
  EXPECT_EQ(FOC_ERR_PARAM, foc_set_command(handle.get(), &command));

  command.max_accel_rad_s2 = 5.0F;
  EXPECT_EQ(FOC_OK, foc_set_command(handle.get(), &command));

  EXPECT_EQ(FOC_ERR_PARAM, foc_motor_get_status(handle.get(), nullptr));
  EXPECT_EQ(FOC_ERR_PARAM, foc_motor_get_status(nullptr, nullptr));
}

TEST(ApiCAbi011, TheNewSurfaceRejectsWhatItCannotHonour) {
  // The C path's job is to turn every misuse into an error code instead of
  // undefined behaviour. The pure-C consumer proves the happy path links and
  // runs; this file proves the unhappy paths return rather than crash, which a
  // C consumer cannot check for itself. C
  // 路径的职责是把每一次误用变成错误码而不是未定义行为。纯 C
  // 消费者证明正常路径能链能跑； 本文件证明异常路径是"返回"而不是"崩"，而那件事
  // C 消费者自己检查不了。
  foc_motor_params_t params{0.5F,    4.0e-4F, 4.0e-4F, 0.05F,
                            1.0e-4F, 1.0e-4F, 7U};

  // --- pure functions: NULL and non-finite ---
  EXPECT_EQ(FOC_ERR_PARAM,
            foc_field_weakening_worthwhile(nullptr, 3.0F, nullptr));
  foc_fw_verdict_t verdict{};
  verdict.struct_size = static_cast<uint32_t>(sizeof(verdict));
  EXPECT_EQ(FOC_ERR_PARAM,
            foc_field_weakening_worthwhile(nullptr, 3.0F, &verdict));
  EXPECT_EQ(FOC_ERR_PARAM,
            foc_field_weakening_worthwhile(&params, NAN, &verdict));
  EXPECT_EQ(FOC_OK, foc_field_weakening_worthwhile(&params, 3.0F, &verdict));

  float id_a = 0.0F;
  float iq_a = 0.0F;
  EXPECT_EQ(FOC_ERR_PARAM,
            foc_mtpa_reference(&params, 3.0F, 1.0F, nullptr, &iq_a));
  EXPECT_EQ(FOC_ERR_PARAM,
            foc_mtpa_reference(&params, 3.0F, 1.0F, &id_a, nullptr));
  EXPECT_EQ(FOC_ERR_PARAM,
            foc_mtpa_reference(nullptr, 3.0F, 1.0F, &id_a, &iq_a));

  // --- board check: struct_size is checked before a single field is written
  // ---
  foc_board_measurements_t m{};
  foc_board_limits_t l{};
  foc_board_report_t report{};
  m.struct_size = static_cast<uint32_t>(sizeof(m));
  l.struct_size = static_cast<uint32_t>(sizeof(l));
  report.struct_size = static_cast<uint32_t>(sizeof(report));
  EXPECT_EQ(FOC_ERR_PARAM, foc_board_check(nullptr, &l, &report));
  EXPECT_EQ(FOC_ERR_PARAM, foc_board_check(&m, nullptr, &report));
  EXPECT_EQ(FOC_ERR_PARAM, foc_board_check(&m, &l, nullptr));
  foc_board_limits_t bad = l;
  bad.struct_size = 4U;
  EXPECT_EQ(FOC_ERR_PARAM, foc_board_check(&m, &bad, &report));

  // --- name lookups are range-checked before the cast ---
  EXPECT_STREQ("?", foc_board_item_name(-1));
  EXPECT_STREQ("?", foc_board_item_name(99));
  EXPECT_STREQ("?", foc_board_cause_name(-1));
  EXPECT_STREQ("?", foc_board_cause_name(99));
  EXPECT_STREQ("current sum", foc_board_item_name(FOC_BOARD_ITEM_CURRENT_SUM));
  EXPECT_STREQ("observer", foc_board_item_name(FOC_BOARD_ITEM_OBSERVER_HEALTH));
}

TEST(ApiCAbi011, TheProtectorPoolRefusesToOverbookAndReusesSlotsSafely) {
  // Two things a C consumer cannot check for itself: that exhausting the pool
  // is an error rather than a wrap-around, and that a handle from a destroyed
  // protector never resolves again even after its slot has been handed to
  // somebody else. The second is the ABA case; it is the reason the generation
  // lives in the handle value. C
  // 消费者自己检查不了的两件事：池子耗尽时给错误而不是绕回；已销毁保护器的句柄在其槽位被
  // 别人接手之后**永远**解析不了。后者就是 ABA
  // 情形，也是代际放在句柄值里的原因。
  foc_protector_t handles[8]{};
  int alive = 0;
  for (int k = 0; k < 8; ++k) {
    const foc_err_t err = foc_protector_create(nullptr, &handles[k]);
    if (err == FOC_OK) {
      ++alive;
    } else {
      EXPECT_EQ(FOC_ERR_NO_RESOURCE, err);
      EXPECT_EQ(nullptr, handles[k]);
    }
  }
  EXPECT_GE(alive, 1);
  EXPECT_LT(alive, 8) << "the pool is static and small by design";

  // Destroy one, mint one: the slot comes back but the old token stays dead.
  const int killed = alive - 1;
  const foc_protector_t stale = handles[killed];
  EXPECT_EQ(FOC_OK, foc_protector_destroy(stale));

  foc_protection_input_t in{};
  foc_protection_output_t out{};
  in.struct_size = static_cast<uint32_t>(sizeof(in));
  out.struct_size = static_cast<uint32_t>(sizeof(out));
  in.v_bus_v = 24.0F;
  in.flux_valid = 1;
  const foc_err_t reused = foc_protector_create(nullptr, &handles[killed]);
  EXPECT_EQ(FOC_OK, reused);
  EXPECT_EQ(FOC_ERR_STATE, foc_protector_step(stale, &in, 5.0e-5F, &out))
      << "the retired token must not resolve to the slot's new tenant";

  for (int k = 0; k < 8; ++k) {
    if (handles[k] != nullptr) {
      static_cast<void>(foc_protector_destroy(handles[k]));
    }
  }
}

TEST(ApiCAbi011, TheWarmResetKeepsTheThermalMemoryAndTheColdOneDropsIt) {
  // The distinction the API exists for: a machine that just tripped is still
  // hot. Clearing a fault is an operator action; clearing the thermal history
  // is a lie unless the machine really cooled. 这个 API
  // 存在的理由：刚跳闸的机器还热着。清故障是运维动作；清热历史是在撒谎，除非机器真凉了。
  foc_protector_t protector = nullptr;
  ASSERT_EQ(FOC_OK, foc_protector_create(nullptr, &protector));

  foc_protection_input_t in{};
  foc_protection_output_t out{};
  in.struct_size = static_cast<uint32_t>(sizeof(in));
  out.struct_size = static_cast<uint32_t>(sizeof(out));
  in.v_bus_v = 24.0F;
  in.temperature_c = 25.0F;
  in.flux_valid = 1;
  in.i_magnitude_a = 3.5F; // above the 3 A continuous rating: spends the budget
  for (int k = 0; k < 400; ++k) {
    ASSERT_EQ(FOC_OK, foc_protector_step(protector, &in, 5.0e-5F, &out));
  }
  const float warm = out.i2t_budget;
  EXPECT_GT(warm, 0.0F);

  ASSERT_EQ(FOC_OK, foc_protector_reset(protector, 0));
  ASSERT_EQ(FOC_OK, foc_protector_step(protector, &in, 5.0e-5F, &out));
  EXPECT_GT(out.i2t_budget, warm)
      << "a warm reset keeps the memory, so the budget carries on";

  ASSERT_EQ(FOC_OK, foc_protector_reset(protector, 1));
  ASSERT_EQ(FOC_OK, foc_protector_step(protector, &in, 5.0e-5F, &out));
  EXPECT_LT(out.i2t_budget, warm) << "a cold reset starts the budget over";

  EXPECT_EQ(FOC_OK, foc_protector_destroy(protector));
  EXPECT_EQ(FOC_ERR_STATE, foc_protector_step(protector, &in, 5.0e-5F, &out));
}
