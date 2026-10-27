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
