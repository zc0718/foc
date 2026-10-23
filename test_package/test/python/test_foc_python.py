"""Smoke test for the foc_python module.

Runs as a plain script (CTest checks the exit code), no pytest dependency: the module is built by
the same build that tests it, so the only thing worth proving here is that the bound surface
behaves like the C API it wraps.

`test_package/CMakeLists.txt` runs this only when the package actually ships `foc_python*`
(metadata.enable_python_bindings = true); a package without the module skips the test instead of
failing.
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
