"""Smoke test for the foc_python module.

Runs as a plain script (CTest checks the exit code), no pytest dependency: the module is built by
the same build that tests it, so the only thing worth proving here is that the bound surface
behaves like the C API it wraps.

`test_package/CMakeLists.txt` runs this only when the package actually ships a `<name>_python`
module (metadata.enable_python_bindings = true); a package without the module skips the test
instead of failing.
"""

import math
import sys

import foc_python as foc


def check_finite(telemetry):
    for key, value in telemetry.items():
        assert math.isfinite(value), f"{key} is not finite: {value}"


def test_metadata():
    assert foc.__version__, "__version__ must come from foc_version_string()"
    assert foc.ErrorCode.OK.value == 0
    assert foc.ErrorCode.PARAM.value == -1


def test_params_roundtrip():
    params = foc.MotorParams(rs=0.4, pole_pairs=4)
    # the struct is float32 on the wire: compare with tolerance, never with ==
    assert math.isclose(params.rs, 0.4, rel_tol=1e-6), params.rs
    assert params.pole_pairs == 4
    assert "MotorParams(" in repr(params)


def test_context_manager_destroys_the_handle():
    with foc.Motor.from_parts() as motor:
        assert motor.alive
        motor.enable()
        motor.set_torque(1.0)
        for _ in range(200):
            motor.tick(1e-4)
        check_finite(motor.telemetry())
    assert not motor.alive, "leaving the with-block must destroy the C handle"
    motor.close()  # idempotent


def test_speed_mode_runs():
    with foc.Motor.from_parts() as motor:
        motor.enable()
        motor.set_speed(50.0)
        for _ in range(10):
            motor.tick(1e-4)
        check_finite(motor.telemetry())


def test_errors_raise_with_the_c_code():
    motor = foc.Motor.from_parts()
    try:
        motor.tick(-1.0)  # non-positive step length
    except foc.FocError as error:
        assert error.code == foc.ErrorCode.PARAM, f"unexpected code {error.code}"
    else:
        raise AssertionError("a negative dt must raise FocError")
    finally:
        motor.close()

    # not enabled yet: the C path answers STATE, and so does the Python face
    idle = foc.Motor.from_parts()
    try:
        idle.tick(1e-4)
    except foc.FocError as error:
        assert error.code == foc.ErrorCode.STATE, f"unexpected code {error.code}"
    else:
        raise AssertionError("ticking before enable() must raise FocError")
    finally:
        idle.close()


def test_pool_exhaustion_is_reported():
    # the C bridge owns a fixed pool (4 slots); the fifth motor must fail, not crash
    held = [foc.Motor.from_parts() for _ in range(4)]
    try:
        foc.Motor.from_parts()
    except foc.FocError as error:
        assert error.code == foc.ErrorCode.NO_RESOURCE, f"unexpected code {error.code}"
    else:
        raise AssertionError("a fifth motor must not be creatable")
    finally:
        for motor in held:
            motor.close()


def test_setup_reaches_the_library():
    setup = foc.MotorSetup(i_max_a=2.0, v_bus_v=48.0)
    with foc.Motor(foc.MotorParams(), setup) as motor:
        applied = motor.setup()
        assert math.isclose(applied.i_max_a, 2.0, rel_tol=1e-6), applied.i_max_a
        assert math.isclose(applied.v_bus_v, 48.0, rel_tol=1e-6), applied.v_bus_v
        # a field the caller left alone keeps the library default
        assert math.isclose(applied.pwm_freq_hz, 20000.0, rel_tol=1e-6), applied.pwm_freq_hz


def test_default_setup_is_observable():
    with foc.Motor(foc.MotorParams()) as motor:
        applied = motor.setup()
        assert math.isclose(applied.i_max_a, 5.0, rel_tol=1e-6), applied.i_max_a
        assert applied.speed_loop_period_s == 0.0, applied.speed_loop_period_s
        assert "MotorSetup(" in repr(applied)


def test_the_injection_can_be_sized_and_watched_from_python():
    # The W7-c diagnostics, on the surface a host-side script actually uses. Two of them, and both are answers the
    # author cannot compute from anything else they have: how high the carrier may go at this control rate, and whether
    # the observer says it is producing a measurement at all.
    # W7-c 的诊断，落在宿主机脚本真正使用的那个表面上。两条，而两者都是作者无法从手头别的信息算出来的答案：这个控制率
    # 下载波最高能到多少，以及观测器是否表示自己在产出测量。
    #
    # The ceiling is the library's own rule, asked rather than re-derived: at 20 kHz it is 8000 Hz, and a rate of zero
    # admits nothing.
    # 天花板是库自己的规则，问出来而不是重新推导：20 kHz 下是 8000 Hz，而频率为零时什么都不准入。
    assert math.isclose(foc.hfi_max_inject_hz(20000.0), 8000.0, rel_tol=1e-6)
    assert math.isclose(foc.hfi_max_inject_hz(40000.0), 16000.0, rel_tol=1e-6)
    assert foc.hfi_max_inject_hz(0.0) == 0.0

    # And the status carries the observer's verdict, with the mode it is really running. A back-EMF observer has no lock
    # detector, so it answers True -- there is no channel that could withhold a verdict -- and publishes no error signal.
    # 而状态带着观测器的结论，连同它真正在跑的模式。反电势观测器没有锁定判据，因此答 True——不存在能扣住结论的通道——
    # 也不发布误差信号。
    with foc.Motor(foc.MotorParams()) as motor:
        status = motor.status()
        assert status["observer"] == foc.Observer.BACK_EMF, status["observer"]
        assert status["observer_locked"] is True, status
        assert status["observer_error"] == 0.0, status["observer_error"]


def test_position_and_alignment_are_reachable_from_python():
    # The Python face must not be a smaller library than the C one: position moves, per-move
    # ceilings and the alignment result are all here.
    # Python 面不得是比 C 面更小的库：位置运动、本次电流上限与对齐结果都在这里。
    setup = foc.MotorSetup(position_bandwidth_rad_s=30.0, velocity_limit_rad_s=10.0)
    with foc.Motor(foc.MotorParams(), setup) as motor:
        applied = motor.setup()
        assert math.isclose(applied.position_bandwidth_rad_s, 30.0, rel_tol=1e-6)
        assert math.isclose(applied.velocity_limit_rad_s, 10.0, rel_tol=1e-6)
        assert applied.align_current_a == 0.0  # alignment off unless asked for

        motor.set_position(0.1)
        motor.set_command(foc.MotorCommand(position_rad=0.2, max_current_a=1.0))

        status = motor.status()
        assert status["mode"] == foc.MotorState.IDLE
        assert status["align_state"] == foc.AlignState.IDLE
        assert status["align_failure"] == foc.AlignFailure.NONE
        assert status["error"] == foc.ErrorCode.OK


def test_an_alignment_recipe_makes_enable_park_in_align():
    setup = foc.MotorSetup(align_current_a=0.5, align_hold_s=0.005, align_tolerance_rad=0.05)
    with foc.Motor(foc.MotorParams(), setup) as motor:
        motor.enable()
        assert motor.status()["mode"] == foc.MotorState.ALIGN
        for _ in range(200):
            if motor.status()["mode"] != foc.MotorState.ALIGN:
                break
            motor.tick(1e-4)
        status = motor.status()
        assert status["mode"] == foc.MotorState.RUN
        assert status["align_state"] == foc.AlignState.DONE


def expect_foc_error(code, action):
    """The module raises FocError; there is no pytest here, so check it by hand."""
    try:
        action()
    except foc.FocError as error:
        assert error.code == code, f"expected {code}, got {error.code}"
        return
    raise AssertionError(f"expected FocError({code}), nothing was raised")


def test_a_half_configured_alignment_is_rejected():
    # "Do not align" has to be the whole recipe: a hold time without a current is a typo, not a
    # request, and the library refuses rather than silently aligning nothing.
    # "不对齐"必须是整套配方：给了保持时间却没给电流是笔误而非请求，库会拒绝而不是静默地什么都不做。
    expect_foc_error(foc.ErrorCode.PARAM, lambda: foc.Motor(foc.MotorParams(), foc.MotorSetup(align_hold_s=0.2)))


def test_a_move_can_carry_its_own_limits():
    # The limits belong to the move, not only to the mechanism: Python can express that.
    # 限制属于运动，而不只属于机构：Python 面能表达这一点。
    with foc.Motor(foc.MotorParams()) as motor:
        motor.set_command(foc.MotorCommand(position_rad=0.3, max_velocity_rad_s=0.5, max_accel_rad_s2=5.0))
        expect_foc_error(
            foc.ErrorCode.PARAM,
            lambda: motor.set_command(foc.MotorCommand(position_rad=0.3, max_accel_rad_s2=-5.0)),
        )


def test_a_negative_current_ceiling_is_rejected():
    with foc.Motor(foc.MotorParams()) as motor:
        expect_foc_error(
            foc.ErrorCode.PARAM,
            lambda: motor.set_command(foc.MotorCommand(position_rad=0.0, max_current_a=-1.0)),
        )


def test_abi_011_pure_functions():
    # The dead-time convention: 4/3 on the d axis, 2/sqrt(3) on q. Same numbers as the C test.
    assert abs(foc.axis_deadtime_v(1e-6, 20000.0, 24.0, 0.0) - 0.64) < 0.01
    assert abs(foc.axis_deadtime_v(1e-6, 20000.0, 24.0, math.pi / 2) - 0.554) < 0.01
    assert foc.axis_deadtime_v(math.nan, 20000.0, 24.0, 0.0) == 0.0

    params = foc.MotorParams(rs=0.5, ld=4e-4, lq=4e-4, kt=0.05, pole_pairs=7)
    i_char, ratio, speed_gain, worthwhile = foc.field_weakening_worthwhile(params, 3.0)
    # psi_f = kt/(1.5*p) = 4.762e-3 Wb, so i_char = psi_f/Ld = 11.9 A: 3.97x i_max.
    assert abs(i_char - 11.9) < 0.05, i_char
    assert ratio > 3.0
    assert not worthwhile, "a surface machine with i_char = 4 i_max is not worth weakening"

    # MTPA on a surface machine is exactly d = 0.
    id_a, iq_a = foc.mtpa_reference(params, 3.0, 1.0)
    assert id_a == 0.0
    assert abs(iq_a - 1.0) < 1e-6


def test_abi_011_board_check():
    measured = foc.BoardMeasurements()
    a = measured  # readability
    a.sum_current_a = 0.0
    # A +d drive puts i_u positive and i_v, i_w negative.
    a.set_response_sign(0, 1.0)
    a.set_response_sign(1, -1.0)
    a.set_response_sign(2, -1.0)
    a.ripple_measured_a = 0.30
    a.ripple_expected_a = 0.375
    a.deadtime_v = 0.640
    a.deadtime_cmd_v = 0.640
    a.rs_ohm = 0.500
    a.rs_nameplate_ohm = 0.500
    a.psi_f_wb = 4.762e-3
    a.psi_f_nameplate_wb = 4.762e-3
    a.observer_error_rad = 0.04
    a.encoder_offset_rad = 0.005

    rows, failures, cause, phase = foc.board_check(measured, foc.BoardLimits())
    assert failures == 0, f"{failures} failed: {rows}"
    assert cause == "none", cause
    assert len(rows) > 0
    assert rows[0][0] and isinstance(rows[0][0], str)

    # Invert one phase: the cause must name polarity, not "several things".
    a.set_response_sign(0, -1.0)
    a.sum_current_a = 0.4
    _, failures, cause, _ = foc.board_check(measured, foc.BoardLimits())
    assert failures > 0
    assert cause == "current polarity", cause

    assert foc.board_item_name(99) == "?"
    assert foc.board_cause_name(-1) == "?"


def test_abi_011_protector():
    with foc.Protector() as protector:
        name, i_limit, allowed, budget, latched, _active = protector.step(
            i_magnitude_a=0.5, v_bus_v=24.0, temperature_c=25.0, omega_e_rad_s=0.0, dt_s=5e-5
        )
        assert name == "none", name
        assert allowed
        assert i_limit > 0.0

        name, i_limit, allowed, _, latched, _ = protector.step(
            i_magnitude_a=5.0, v_bus_v=24.0, temperature_c=25.0, omega_e_rad_s=0.0, dt_s=5e-5
        )
        assert name == "over current", name
        assert not allowed
        assert i_limit == 0.0
        assert latched, "an over-current must latch"

        # Taking the current away does not clear a latched fault.
        name, _, _, _, _, _ = protector.step(
            i_magnitude_a=0.1, v_bus_v=24.0, temperature_c=25.0, omega_e_rad_s=0.0, dt_s=5e-5
        )
        assert name == "over current", name

        protector.reset(cold=True)
        name, _, allowed, _, _, _ = protector.step(
            i_magnitude_a=0.1, v_bus_v=24.0, temperature_c=25.0, omega_e_rad_s=0.0, dt_s=5e-5
        )
        assert name == "none", name
        assert allowed


def test_a_python_plant_closes_the_loop():
    """The plant lives in Python: it supplies the measurements and receives the duties.

    The claim is not that the numbers are good -- this plant is a constant current -- but that the loop runs through
    the Python object once per control step and the motor reaches Run with no sensor compiled in.
    """
    calls = {"duty": 0, "current": 0}

    class Plant:
        def write_duty(self, du, dv, dw):
            calls["duty"] += 1

        def read_current(self):
            calls["current"] += 1
            return (0.0, 0.0, 0.0)

        def angle_mech_rad(self):
            return 0.0

        def voltage_v(self):
            return 24.0

    params = foc.MotorParams()
    params.rs, params.ld, params.lq = 0.5, 4e-4, 4e-4
    params.kt, params.j, params.b, params.pole_pairs = 0.05, 1e-5, 1e-6, 7
    with foc.Motor(params, Plant()) as motor:
        motor.enable()
        motor.set_torque(0.2)
        for _ in range(100):
            motor.tick(1.0 / 20000.0)
        status = motor.status()
    assert status["mode"] == foc.MotorState.RUN, status
    assert calls["duty"] == 100, calls
    assert calls["current"] == 100, calls


def test_a_raising_plant_is_reported_rather_than_swallowed():
    """The control loop is noexcept, so an exception from the plant is latched and re-raised by `tick`.

    A silently ignored plant error would leave the loop running on stale measurements, which is worse than stopping.
    """

    class Angry:
        def write_duty(self, du, dv, dw):
            raise RuntimeError("plant exploded")

        def read_current(self):
            return (0.0, 0.0, 0.0)

        def angle_mech_rad(self):
            return 0.0

        def voltage_v(self):
            return 24.0

    params = foc.MotorParams()
    params.rs, params.ld, params.lq = 0.5, 4e-4, 4e-4
    params.kt, params.j, params.b, params.pole_pairs = 0.05, 1e-5, 1e-6, 7
    with foc.Motor(params, Angry()) as motor:
        motor.enable()
        motor.set_torque(0.2)
        try:
            for _ in range(5):
                motor.tick(1.0 / 20000.0)
        except RuntimeError as error:
            assert "plant raised" in str(error), error
        else:
            raise AssertionError("the plant's exception was swallowed by the loop")


def test_a_plant_missing_a_required_method_is_rejected_at_construction():
    class Half:
        def write_duty(self, du, dv, dw):
            pass

    params = foc.MotorParams()
    try:
        foc.Motor(params, Half())
    except AttributeError as error:
        assert "read_current" in str(error), error
    else:
        raise AssertionError("a plant without the required methods must not be accepted")


def test_the_gains_a_tuning_tool_gets_are_the_ones_the_loop_runs_with():
    """The gain query exists so an outside tool reproduces the shipped loop instead of re-deriving it."""
    params = foc.MotorParams()
    params.rs, params.ld, params.lq = 0.5, 4.0e-4, 4.0e-4
    params.kt, params.j, params.b = 0.05, 1.0e-5, 1.0e-6
    params.pole_pairs = 7

    setup = foc.MotorSetup()
    setup.i_max_a, setup.pwm_freq_hz = 3.0, 20000.0
    setup.deadtime_ns, setup.v_bus_v = 0.0, 24.0
    setup.current_bandwidth_rad_s = 1000.0

    kp, ki, out_min, out_max, int_min, int_max = foc.current_pi_gains(params, setup)
    assert kp > 0.0 and ki > 0.0, (kp, ki)
    # The design rule, written out here rather than taken from the library: kp = 2*w*L - R, ki = w^2*L.
    w = setup.current_bandwidth_rad_s
    assert abs(kp - max(2.0 * w * params.ld - params.rs, 0.0)) < 1e-4, kp
    assert abs(ki - w * w * params.ld) < 1e-2, ki
    # The clamps come from the bus, not from nowhere.
    assert abs(out_max - 0.5 * setup.v_bus_v) < 1e-3, out_max
    assert int_min < int_max

    # None means "the library defaults", and it has to be accepted as such.
    default_kp = foc.current_pi_gains(params)[0]
    assert default_kp > 0.0, default_kp


def test_a_standstill_sweep_returns_the_resistance_it_was_given():
    params = foc.MotorParams()
    params.rs, params.ld, params.lq = 0.5, 4.0e-4, 4.0e-4
    params.kt, params.j, params.b = 0.05, 1.0e-5, 1.0e-6
    params.pole_pairs = 7

    currents = [0.2, 0.32, 0.5, 0.8, 1.25, 2.0]
    voltages = [0.5 * i + 0.64 for i in currents]
    rs, loss_v, residual_v, valid, samples = foc.rs_estimate(params, voltages, currents)
    assert valid, "six settled points spanning 10x must be accepted"
    assert abs(rs - 0.5) < 0.005, rs
    assert abs(loss_v - 0.64) < 0.032, loss_v
    assert residual_v < 1e-3, residual_v
    assert samples == len(currents), samples

    # A flat sweep is refused through `valid` rather than through an exception: the call answered, the answer is not a
    # measurement.
    _, _, _, flat_valid, _ = foc.rs_estimate(params, [0.89] * 6, [0.5] * 6)
    assert not flat_valid, "a sweep with one current has no slope to find"


def test_mismatched_arrays_are_a_python_error_not_a_silent_truncation():
    params = foc.MotorParams()
    try:
        foc.rs_estimate(params, [1.0, 2.0], [0.5])
    except ValueError:
        pass
    else:
        raise AssertionError("mismatched lengths must raise ValueError")


def main():
    failures = 0
    for name, case in sorted(globals().items()):
        if not name.startswith("test_") or not callable(case):
            continue
        try:
            case()
            print(f"  ok   {name}")
        except AssertionError as error:
            failures += 1
            print(f"  FAIL {name}: {error}")
    print(f"{len([n for n in globals() if n.startswith('test_')])} case(s), {failures} failure(s)")
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
