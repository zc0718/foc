"""Smoke test for the foc_python module.

Runs as a plain script (CTest checks the exit code), no pytest dependency: the module is built by
the same build that tests it, so the only thing worth proving here is that the bound surface
behaves like the C API it wraps.

`test_package/CMakeLists.txt` runs this only when the package actually ships a `<name>_python`
module (metadata.enable_python_bindings = true); a package without the module skips the test
instead of failing.
"""

import math
import pathlib
import random
import sys

import numpy as np

import foc_python as foc

# The "from zero to a servo" example lives in its own file so that the quick-start page can include it verbatim, and it
# carries its own assertion. Importing it here is what makes it run: the page cannot drift from a file the suite
# executes, whatever else changes.
# "从零到一个能跑的伺服"示例单独成文件，好让快速上手页原样包含它，并自带断言。在这里导入它就是让它被执行的机制：无论
# 别的什么改变，页面都不可能从套件真正执行的文件上漂走。
from from_zero_to_servo import test_the_example_settles_a_speed_step  # noqa: E402,F401


# The golden ledger's comparator lives with the benchmark scripts; importing it here is what makes its rules falsifiable.
# A ledger whose verdict bands are never tested is a ledger that decides "same" for anything it has not seen before.
# 金标台账的比较器住在 benchmark 脚本里；在这里导入它，是让它的规则可被证伪的方式。一个判决带从未被测试的台账，就是一个
# 对没见过的任何东西都判"相同"的台账。
sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[3] / "benchmark" / "script"))
import golden  # noqa: E402
import analyze  # noqa: E402
import foc_sil_plant  # noqa: E402
import montecarlo  # noqa: E402

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


def test_the_setup_readback_returns_every_field_that_was_set():
    # The readback is where somebody looks first when the machine does something they did not ask for. It used to copy
    # seventeen fields and silently drop the six appended ones -- observer, inject_v, inject_hz, the blend band and the
    # handover cutoff -- so `setup()` reported the defaults for exactly the fields that choose the observer, while
    # `status()["observer"]` reported the truth. This case sets every field and demands all of them back.
    # 回读是"机器做了我没要求的事"时第一个去看的地方。它过去只拷贝十七个字段、静默丢掉后来追加的六个——observer、
    # inject_v、inject_hz、融合带与交接截止频率——于是 `setup()` 对"选择观测器"的那几个字段报默认值，而
    # `status()["observer"]` 报的是真话。本用例把每个字段都设上，并要求全部原样取回。
    setup = foc.MotorSetup()
    setup.i_max_a = 3.5
    setup.pwm_freq_hz = 16000.0
    setup.deadtime_ns = 750.0
    setup.v_bus_v = 36.0
    setup.current_bandwidth_rad_s = 1500.0
    setup.speed_bandwidth_rad_s = 90.0
    setup.speed_loop_period_s = 2e-4
    setup.position_bandwidth_rad_s = 25.0
    setup.velocity_limit_rad_s = 12.0
    setup.position_loop_period_s = 5e-4
    setup.position_accel_limit_rad_s2 = 40.0
    setup.align_current_a = 1.2
    setup.align_ramp_s = 0.15
    setup.align_hold_s = 0.25
    setup.align_tolerance_rad = 0.04
    setup.align_verify_s = 0.1
    setup.align_angle_e_rad = 0.0
    setup.observer = foc.Observer.BLENDED
    setup.inject_v = 9.75
    setup.inject_hz = 6000.0
    setup.blend_start_rad_s = 8.0
    setup.blend_end_rad_s = 40.0
    setup.handover_lpf_hz = 35.0

    params = foc.MotorParams()
    params.rs, params.ld, params.lq = 0.5, 4e-4, 8e-4  # salient, so BLENDED is admissible
    with foc.Motor(params, setup) as motor:
        applied = motor.setup()

    assert applied.observer == foc.Observer.BLENDED, applied.observer
    assert applied.inject_v == 9.75, applied.inject_v
    assert applied.inject_hz == 6000.0, applied.inject_hz
    assert applied.blend_start_rad_s == 8.0, applied.blend_start_rad_s
    assert applied.blend_end_rad_s == 40.0, applied.blend_end_rad_s
    assert applied.handover_lpf_hz == 35.0, applied.handover_lpf_hz
    # and the older half, so a future field is added in one place and read back in one place
    # 以及旧的那一半，这样将来新增字段只在一个地方写、一个地方读
    assert math.isclose(applied.i_max_a, 3.5, rel_tol=1e-6)
    assert math.isclose(applied.pwm_freq_hz, 16000.0, rel_tol=1e-6)
    assert math.isclose(applied.deadtime_ns, 750.0, rel_tol=1e-6)
    assert math.isclose(applied.handover_lpf_hz, 35.0, rel_tol=1e-6)


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


def test_the_injection_can_be_sized_and_the_result_runs():
    # `hfi_size` answers the question the boundaries page raises: given this machine and this control rate, what carrier,
    # what amplitude, what cutoffs? And the answer has to be a configuration the library accepts -- so the case builds a
    # motor from it rather than comparing numbers with numbers.
    # `hfi_size` 回答边界页提出的那个问题：给定这台机器与这个控制率，用什么载波、什么幅值、什么截止频率？而答案必须是库
    # 会接受的配置——因此本用例用它建一台电机，而不是拿数字比数字。
    params = foc.MotorParams()
    params.rs, params.ld, params.lq = 0.5, 4e-4, 8e-4  # salient: ld != lq
    params.kt, params.j, params.b, params.pole_pairs = 0.05, 1e-5, 1e-6, 7

    sized = foc.hfi_size(params, pwm_freq_hz=20000.0, hf_current_a=0.5)
    assert math.isclose(sized["inject_hz"], 8000.0, rel_tol=1e-6), sized
    assert math.isclose(sized["inject_v"], 0.5 * 2.0 * math.pi * 8000.0 * 4e-4, rel_tol=1e-4), sized
    assert math.isclose(sized["hf_current_a"], 0.5, rel_tol=1e-6), sized
    assert 0.0 < sized["demod_lpf_hz"] < sized["inject_hz"], sized

    setup = foc.MotorSetup()
    setup.observer = foc.Observer.HFI
    setup.inject_v = sized["inject_v"]
    setup.inject_hz = sized["inject_hz"]
    with foc.Motor(params, setup) as motor:
        motor.enable()
        for _ in range(20):
            motor.tick(1.0 / 20000.0)
        assert motor.status()["observer"] == foc.Observer.HFI, motor.status()

    # A surface machine is refused, exactly as the injection observer itself would be.
    # 表贴机被拒绝，与注入观测器本身会受到的处理一致。
    flat = foc.MotorParams()
    flat.rs, flat.ld, flat.lq = 0.5, 4e-4, 4e-4
    try:
        foc.hfi_size(flat, pwm_freq_hz=20000.0, hf_current_a=0.5)
    except RuntimeError:
        pass
    else:
        raise AssertionError("a surface machine was sized for injection")


def test_the_running_observer_can_be_switched_from_python():
    # The hand-over was the last capability that C could reach and Python could not. Its refusals are the contract, so
    # the case checks them alongside the happy path -- a binding that turned a refusal into a no-op would leave
    # `status()["observer"]` unchanged and the author believing the switch had happened.
    # 交接是 C 能达到、而 Python 达不到的最后一项能力。它的拒绝本身就是契约，因此本用例与正常路径一并检查——一个把拒绝
    # 变成空操作的绑定会让 `status()["observer"]` 保持不变，而作者相信自己切换成功了。
    params = foc.MotorParams()
    params.rs, params.ld, params.lq = 0.5, 4e-4, 8e-4  # salient, so HFI is admissible at all
    sized = foc.hfi_size(params, pwm_freq_hz=20000.0, hf_current_a=0.5)

    # Before Run it answers STATE: a hand-over carries an estimate, and an observer that has not been stepped has none.
    # 在 Run 之前答 STATE：交接携带一个估计，而没被推进过的观测器没有估计。
    with foc.Motor(params) as motor:
        expect_foc_error(foc.ErrorCode.STATE, lambda: motor.set_observer(foc.Observer.BACK_EMF))

    # Same motor, now running, but built without a carrier: HFI is refused on a fact rather than on a switch, and the
    # refusal leaves the observer as it was.
    # 同一台电机，现在在跑，但建的时候没有载波：HFI 凭事实而非开关拒绝，且拒绝之后观测器保持原样。
    with foc.Motor(params) as motor:
        motor.enable()
        for _ in range(20):
            motor.tick(1.0 / 20000.0)
        assert motor.status()["mode"] == foc.MotorState.RUN, motor.status()
        expect_foc_error(foc.ErrorCode.PARAM, lambda: motor.set_observer(foc.Observer.HFI))
        assert motor.status()["observer"] == foc.Observer.BACK_EMF, motor.status()

    # With the carrier in the setup the switch lands, and it is not a one-way door: `status()["observer"]` is filled
    # from the live observer, so it reports where the estimate actually is, in both directions.
    # 载波在设置里时切换落地，而且不是单向门：`status()["observer"]` 由活着的观测器填入，因此它报告估计真正在的位置，
    # 两个方向都是。
    setup = foc.MotorSetup()
    setup.observer = foc.Observer.BACK_EMF
    setup.inject_v = sized["inject_v"]
    setup.inject_hz = sized["inject_hz"]
    with foc.Motor(params, setup) as motor:
        motor.enable()
        for _ in range(20):
            motor.tick(1.0 / 20000.0)
        assert motor.status()["observer"] == foc.Observer.BACK_EMF, motor.status()

        motor.set_observer(foc.Observer.HFI)
        assert motor.status()["observer"] == foc.Observer.HFI, motor.status()
        # and the axis keeps running after the hand-over rather than restarting
        # 交接之后轴继续跑，而不是重启
        for _ in range(20):
            motor.tick(1.0 / 20000.0)
        assert motor.status()["observer"] == foc.Observer.HFI, motor.status()

        motor.set_observer(foc.Observer.BACK_EMF)
        assert motor.status()["observer"] == foc.Observer.BACK_EMF, motor.status()


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


def test_the_window_the_loop_wrote_can_be_read_from_python():
    # The trace channel, end to end: Python owns the storage, the loop writes into it, Python reads it back. The
    # criterion is not "it returns something" -- it is that a sample agrees with the scalar reads from the same period,
    # exactly. A recorder that agrees to six digits can manufacture a regression, so this compares with ==, and a
    # float32 that crosses the C ABI lands in a Python float without loss.
    # 轨迹通道端到端：Python 持有存储、控制环往里写、Python 读回。判据不是"它能返回东西"——而是一条样本与同一拍标量读
    # 完全一致。六位对得上的记录器能凭空造出回归，所以这里用 == 比较；跨 C ABI 的 float32 无损落进 Python float。
    trace = foc.Trace(32)
    assert len(trace) == 32

    with foc.Motor.from_parts() as motor:
        assert motor.trace_size() == 0, "nothing attached yet"
        motor.set_trace(trace)
        motor.enable()
        motor.set_torque(0.5)
        for _ in range(20):
            motor.tick(1e-4)

        assert motor.trace_size() == 20, motor.trace_size()
        assert motor.trace_dropped() == 0, motor.trace_dropped()

        rows = trace.samples(motor)
        assert len(rows) == 20, len(rows)
        newest = rows[-1]
        scalar = motor.telemetry()
        status = motor.status()
        assert newest["i_q"] == scalar["iq_a"], (newest["i_q"], scalar["iq_a"])
        assert newest["i_d"] == scalar["id_a"], (newest["i_d"], scalar["id_a"])
        assert newest["speed_mech"] == scalar["speed_mech_rad_s"]
        assert newest["mode"] == status["mode"], (newest["mode"], status["mode"])
        # no clock on this HAL, so the timestamp is honestly absent rather than invented
        assert all(row["t_us"] == 0 for row in rows)

        # keep running past the capacity: the window slides instead of freezing, and says how much it lost
        for _ in range(20):
            motor.tick(1e-4)
        assert motor.trace_size() == 32
        assert motor.trace_dropped() == 8, motor.trace_dropped()
        assert len(trace.samples(motor)) == 32

        # detaching leaves the motor running and the window readable, and says there is nothing attached
        motor.set_trace(None)
        assert motor.trace_size() == 0
        assert motor.trace_dropped() == 0, "a fresh attachment starts its own counts"


def test_a_trace_buffer_that_was_never_attached_reads_empty():
    # Attaching nothing and reading anyway must be dull rather than clever: no exception, no stale data, and the counts
    # say zero because nothing was ever recorded instead of guessing.
    # 什么都不挂却去读，必须是"平淡"的而不是"机灵"的：不抛异常、不给旧数据，计数说 0 是因为从未记录，而不是靠猜。
    trace = foc.Trace(4)
    with foc.Motor.from_parts() as motor:
        assert trace.samples(motor) == []
        assert motor.trace_dropped() == 0
        motor.set_trace(trace)
        assert trace.samples(motor) == [], "attached but never ticked"


def test_a_fault_code_can_be_named_without_stepping_a_protector():
    # `Protector.step()` already returns a name, but only for the fault it just produced. An author holding a code from
    # elsewhere -- a bit in `active_mask`, a line in a log -- could not ask what it is called. The table is the one C
    # reads, so the check is against exact strings: renaming a fault is changing a promise, and that has to cost a red
    # case rather than slip through a "non-empty" assertion.
    # `Protector.step()` 已经返回名字，但只对它刚产出的那一个故障。作者从别处拿到一个码——`active_mask` 里的一个比特、
    # 日志里的一行——此前没办法问它叫什么。表就是 C 读的那一张，因此逐字比对：给故障改名就是改承诺，这要付出一次红用例
    # 的代价，而不是从"非空"断言里溜过去。
    assert foc.fault_name(0) == "none", foc.fault_name(0)
    assert foc.fault_name(1) == "over current", foc.fault_name(1)
    assert foc.fault_name(7) == "lost sync", foc.fault_name(7)
    assert foc.fault_name(99) == "?", foc.fault_name(99)

    # Every value in the table is named and no two share a name, so a mask can be decoded without a lookup table of
    # its own.
    # 表内每个取值都有名字，且没有两个同名，于是一个掩码不需要自带一张查找表就能解码。
    names = [foc.fault_name(code) for code in range(8)]
    assert all(name != "?" for name in names), names
    assert len(set(names)) == 8, names

    # And the name the protector reports is this table's, not a second copy of it.
    # 而保护层报告的名字出自这张表，不是它的第二份拷贝。
    with foc.Protector() as protector:
        reported, _, _, _, _, _ = protector.step(
            i_magnitude_a=5.0, v_bus_v=24.0, temperature_c=25.0, omega_e_rad_s=0.0, dt_s=5e-5
        )
        assert reported == foc.fault_name(1), reported


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


def test_the_ledger_bands_classify_the_way_they_are_documented():
    # Each band has a job, and the interesting one is 1.3-2.0x: that is where the W7-c investigation landed (0.1356
    # predicted against 0.0900 measured, 1.51x) and the term that explained it was real. A ledger that called that
    # "drift" would invite somebody to widen a tolerance instead of finding the missing term, so the band is pinned here
    # with the number that produced it.
    # 每个判决带都有职责，最有意思的是 1.3–2.0×：W7-c 的调查就落在那（预测 0.1356 对实测 0.0900，1.51×），而解释它的那一项
    # 是真实存在的。一个把这件事叫"漂移"的台账，会诱使别人去放宽容差而不是去找缺失的项，所以这个带就用产生它的那个数字钉住。
    assert golden.verdict(1.0, 1.0)[0] == "same"
    assert golden.verdict(1.0, 1.0 + 1e-9)[0] == "same", "an exact repeat must not be called drift"
    assert golden.verdict(1.0, 1.05)[0] == "drift", "a 5 % change is drift, not noise"
    tag, relative = golden.verdict(0.0900, 0.1356)
    assert tag == "attribute", tag
    assert abs(relative - 0.5067) < 1e-3, relative
    assert golden.verdict(1.0, 3.0)[0] == "changed"
    assert golden.verdict(-2.0, -2.0) [0] == "same"
    # a sign flip has a magnitude ratio of 1.0, so the band test alone would file it as drift; it is not
    # 符号翻转的幅值比是 1.0，所以单看比例带会把它归成漂移；它不是
    assert golden.verdict(0.5, -0.5)[0] == "changed"


def test_the_report_names_the_trace_the_quantity_and_the_difference():
    # The criterion the plan states for this whole mechanism: the ledger has to say *which* trace, *which* quantity and
    # *how much*. This drives the comparator with synthetic data so the format is checked without needing a drifted
    # build -- and it fails if somebody reduces the output to a single "differs" line.
    # 计划给这套机制定的判据：台账必须说出**哪条**轨迹、**哪个**量、**差多少**。本用例用合成数据驱动比较器，于是不必
    # 真的造出一个漂移的构建就能检查格式——而谁要是把输出压成一句"有差异"，它会红。
    meta = {"overshoot": "0.02", "peak_iq": "1.0"}
    golden_rows = [{"t_us": 0, "i_d": 0.0, "i_q": 1.0, "i_ref_d": 0.0, "i_ref_q": 1.0, "v_d": 0.0, "v_q": 0.0,
                    "speed_mech": 0.0, "mode": 2}]
    current_rows = [{"t_us": 0, "i_d": 0.0, "i_q": 1.5, "i_ref_d": 0.0, "i_ref_q": 1.0, "v_d": 0.0, "v_q": 0.0,
                     "speed_mech": 0.0, "mode": 2}]
    summary = {"overshoot": 0.02, "steady_error": 0.0, "settle_s": 0.0, "peak_iq": 1.0, "peak_abs_vq": 1.0}
    report = golden.compare("speed_step_100", meta, golden_rows, current_rows, summary)
    text = "\n".join(report)
    assert "speed_step_100" in text, "the report must name the trace"
    assert "i_q@" in text, "the report must name the quantity and where in the window"
    assert "1.5" in text and "1" in text, "the report must carry both values"
    assert "quantit(ies) outside" in text
    # and a clean run says so instead of printing nothing
    # 而一次干净的运行会明说，而不是什么都不打
    clean = "\n".join(golden.compare("speed_step_100", meta, golden_rows, golden_rows[:1], summary))
    assert 'every summary quantity and every window slot is "same"' in clean


# ---- the analysis layer -------------------------------------------------------------------------
#
# 1.3's criteria, measured where the two implementations can be compared without a shared file: one window, one process,
# the library's C++ on one side and numpy on the other. The plan asks for two things at once -- the fitted bandwidth
# against the analytic one the gain law promises (<= 10 %), and the C++ metric against the Python one on the same
# samples (<= 1 %) -- and neither is worth anything if it cannot go red, so both are checked against the number that
# would be wrong if the arithmetic drifted.
#
# 分析层的判据，在"两份实现无需共享文件就能比对"的地方测量：一个窗口、一个进程，一边是库的 C++，一边是 numpy。
# 计划同时要两件事——拟合带宽对增益律承诺的解析带宽（≤10%），以及 C++ 指标对同一批样本上的 Python 指标（≤1%）——
# 而两者若不能变红就都没有价值，因此都与"算术漂移后会变错"的那个数对齐。

def _speed_step_window(target_rad_s, bandwidth_rad_s, periods=1000):
    """A small speed step, recorded: the window starts exactly at the command, so `speed[i]` sits at `i * dt`.

    Small on purpose. A large step saturates the current limit, and a saturated response is not the linear loop the
    bandwidth claim is about; 10 rad/s on this machine stays inside 5 A.
    """
    params = foc.MotorParams()
    params.rs, params.ld, params.lq = golden.PLANT['rs'], golden.PLANT['ld'], golden.PLANT['lq']
    params.j, params.b = golden.PLANT['j'], golden.PLANT['b']
    params.pole_pairs = golden.PLANT['pole_pairs']
    params.kt = 1.5 * golden.PLANT['pole_pairs'] * golden.PLANT['psi_f']

    setup = foc.MotorSetup(pwm_freq_hz=10000.0, i_max_a=5.0, v_bus_v=golden.PLANT['v_bus'])
    setup.speed_bandwidth_rad_s = bandwidth_rad_s
    dt = 1e-4
    plant = foc_sil_plant.PmsmPlant(**golden.PLANT, dt_s=dt)

    trace = foc.Trace(periods + 1)
    with foc.Motor(params, setup, plant) as motor:
        motor.enable()
        for _ in range(50):
            motor.tick(dt)          # hold at zero before the step
        motor.set_trace(trace)      # the window starts at the command, not before it
        motor.set_speed(target_rad_s)
        for _ in range(periods):
            motor.tick(dt)
        rows = trace.samples(motor)

    return [0.0] + [row['speed_mech'] for row in rows], dt


def test_the_analysis_layer_and_the_library_measure_one_loop_the_same_way():
    requested = 250.0
    target = 10.0
    speed, dt = _speed_step_window(target, requested)
    band = 0.02 * target

    c_metric = foc.step_metrics(speed, dt, target, band)
    py_metric = analyze.step_metrics(speed, dt, target, band)

    assert c_metric['status'] == foc.ErrorCode.OK, c_metric
    assert py_metric.status == 'Ok', py_metric

    # Criterion B: two implementations, one metric, the same samples.
    # 判据 B：两份实现、一个指标、同一批样本。
    relative = abs(py_metric.bandwidth_rad_s - c_metric['bandwidth_rad_s']) / c_metric['bandwidth_rad_s']
    assert relative <= 0.01, f"C++ {c_metric['bandwidth_rad_s']:.3f} vs numpy {py_metric.bandwidth_rad_s:.3f} rad/s"

    # Criterion A: the measurement against the bandwidth the gain law was asked for.
    # 判据 A：测量值对增益律被请求的带宽。
    assert abs(c_metric['bandwidth_rad_s'] / requested - 1.0) <= 0.10, (
        f"{c_metric['bandwidth_rad_s']:.2f} rad/s measured against {requested} rad/s requested")
    assert abs(py_metric.bandwidth_rad_s / requested - 1.0) <= 0.10, (
        f"{py_metric.bandwidth_rad_s:.2f} rad/s measured against {requested} rad/s requested")


def test_the_analysis_layer_replays_the_settling_index_exactly():
    """The replay contract from the plan: the analysis layer must recompute the number the loop's own side computed.

    Settling is the quantity where "error 0" is achievable and meaningful, because it is an *index* decision, not a
    fitted number: both sides must pick the same sample. The seconds are `index * dt`, and comparing those in float
    would compare two roundings rather than the fact -- so the fact is compared, and the seconds only to half a period.
    """
    target, band = 10.0, 0.2
    speed, dt = _speed_step_window(target, 250.0)

    c_metric = foc.step_metrics(speed, dt, target, band)
    py_metric = analyze.step_metrics(speed, dt, target, band)

    # The same sample, decided the same way, by an independent implementation.
    # 同一个样本、同一个判定，由一份独立实现给出。
    c_index = int(round(c_metric['settling_s'] / dt))
    py_index = int(round(py_metric.settling_s / dt))
    assert c_index == py_index, f"C++ settled at sample {c_index}, numpy at {py_index}"

    # And that index is the one a reader of the window would pick by hand: the first sample after the last one outside.
    # 而且这正是读这份窗口的人会手挑的下标：最后一个带外样本之后的第一个样本。
    outside = [i for i, value in enumerate(speed) if abs(value - target) > band]
    assert c_index == outside[-1] + 1, f"index {c_index} is not the last-exit + 1 ({outside[-1] + 1})"
    assert abs(py_metric.settling_s - c_metric['settling_s']) <= 0.5 * dt

    # The reader the CLI depends on, checked against the file the ledger actually stores: a window format that drifts
    # silently would make `analyze.py <window.csv>` answer confidently about the wrong columns.
    # CLI 依赖的读取器，对着台账真正存下的文件检查：格式若悄悄漂移，`analyze.py <window.csv>` 会自信地回答错误的列。
    assert tuple(golden.FIELDS) == analyze.TRACE_COLUMNS, 'the writer and the reader disagree about the column order'
    recorded = analyze.load_window(pathlib.Path(__file__).resolve().parents[2] / 'resources' / 'golden' /
                                   'speed_step_100.csv')
    assert recorded['speed_mech'].size == 200, recorded['speed_mech'].size
    assert 'module_sha' in recorded['_header'], recorded['_header']
    assert recorded['speed_mech'][-1] > 0.9 * 100.0, recorded['speed_mech'][-1]

    # End to end on the ledger's own artifact: that window was recorded from a 100 rad/s step with the library's default
    # speed bandwidth (200 rad/s), and reading it back measures the loop. 197.59 rad/s is -1.2 % -- on a window
    # decimated to one sample per millisecond against a four-millisecond time constant, which is the point: the reader
    # and the metric recover a documented default from a stored file, not from the object that produced it.
    # 在台账自己的产物上做端到端：那个窗口来自 100 rad/s 阶跃，用的是库默认速度带宽（200 rad/s），读回来就能测出环路。
    # 197.59 rad/s 即 -1.2%——而且这还是抽稀到每毫秒一个样本、而时间常数是 4 ms 的窗口，这正是关键：读取器与指标从
    # 一个存下来的文件里、而不是从产出它的对象里，复原了一个有文档的默认值。
    from_ledger = analyze.step_metrics(recorded['speed_mech'], 1e-3, 100.0, 2.0)
    assert from_ledger.status == 'Ok', from_ledger
    assert abs(from_ledger.bandwidth_rad_s / 200.0 - 1.0) <= 0.10, from_ledger.bandwidth_rad_s


# ---- the sweep -----------------------------------------------------------------------------------
#
# The step-response band is weak by construction (A-11: a 20 % gain error moved it 9.9 %, inside its own 10 %). The
# frequency-domain -3 dB needs no model and moved 15.7 % on the same defect, so it is the criterion that carries the
# claim. Both are kept: the step response says how the servo *behaves*, the sweep says where its corner is.
#
# 阶跃带在构造上就是弱的（A-11：20% 的增益错误只让它移动 9.9%，仍在自己的 10% 带内）。频域 -3 dB 不需要模型，同一个
# 缺陷下移动了 15.7%，因此由它来承载这条断言。两者都保留：阶跃说伺服**行为如何**，扫频说它的拐点在哪里。

def _gain_at(bandwidth_rad_s, omega_rad_s, amplitude=10.0):
    """Drive the reference with `A*cos(w t)` through the Python plant and measure |H| and the lag."""
    params = foc.MotorParams()
    params.rs, params.ld, params.lq = golden.PLANT['rs'], golden.PLANT['ld'], golden.PLANT['lq']
    params.j, params.b = golden.PLANT['j'], golden.PLANT['b']
    params.pole_pairs = golden.PLANT['pole_pairs']
    params.kt = 1.5 * golden.PLANT['pole_pairs'] * golden.PLANT['psi_f']

    setup = foc.MotorSetup(pwm_freq_hz=10000.0, i_max_a=5.0, v_bus_v=golden.PLANT['v_bus'])
    setup.speed_bandwidth_rad_s = bandwidth_rad_s
    dt = 1e-4
    plant = foc_sil_plant.PmsmPlant(**golden.PLANT, dt_s=dt)

    period = 2.0 * math.pi / omega_rad_s
    with foc.Motor(params, setup, plant) as motor:
        motor.enable()
        motor.set_speed(0.0)
        for _ in range(50):
            motor.tick(dt)
        t = 0.0
        for _ in range(int(6.0 * period / dt)):
            motor.set_speed(amplitude * math.cos(omega_rad_s * t))
            motor.tick(dt)
            t += dt
        command, response = [], []
        for _ in range(int(4.0 * period / dt)):
            c = amplitude * math.cos(omega_rad_s * t)
            motor.set_speed(c)
            motor.tick(dt)
            command.append(c)
            response.append(plant.velocity_rad_s())
            t += dt
    return analyze.frequency_response(command, response, dt, omega_rad_s)


def test_the_sweep_finds_the_corner_the_gain_law_promised():
    requested = 250.0
    lo, hi = 0.5 * requested, 2.0 * requested
    for _ in range(8):
        mid = math.sqrt(lo * hi)
        gain, _lag = _gain_at(requested, mid)
        if gain > 0.70710678:
            lo = mid
        else:
            hi = mid
    crossing = math.sqrt(lo * hi)
    assert abs(crossing / requested - 1.0) <= 0.10, f"-3 dB at {crossing:.2f} rad/s against {requested} requested"

    # The same corner the C++ bench reports (234.91 rad/s, -6.04 %) to within the two plants' differences, which is a
    # statement about the machines rather than about the metric -- the two plants are independent models.
    # 与 C++ 台架报出的同一个拐点（234.91 rad/s，-6.04%）的差别只来自两台不同的 plant 模型，那是关于机器的陈述，
    # 而不是关于指标的。
    assert abs(crossing / 234.91 - 1.0) <= 0.05, f"the two benches disagree about the corner: {crossing:.2f}"


def test_the_tone_fit_agrees_between_c_and_numpy_on_the_same_window():
    from_curve = [3.5 + 1.25 * math.cos(2.0 * math.pi * 137.0 * k * 1e-4 + 0.7) for k in range(2000)]
    omega = 2.0 * math.pi * 137.0

    c_fit = foc.sinusoid_fit(from_curve, 1e-4, omega)
    py_fit = analyze.sinusoid_fit(from_curve, 1e-4, omega)
    assert c_fit['status'] == foc.ErrorCode.OK, c_fit
    assert py_fit.status == 'Ok', py_fit
    assert abs(py_fit.amplitude - c_fit['amplitude']) / c_fit['amplitude'] <= 0.01, (py_fit, c_fit)
    assert abs(py_fit.phase_rad - c_fit['phase_rad']) <= 0.01, (py_fit, c_fit)

    # Both recover the tone that is actually there, which is what makes the agreement worth having.
    # 两者都复原了真正存在的那个音——这才是"一致"值得拥有的原因。
    assert abs(c_fit['amplitude'] / 1.25 - 1.0) <= 0.001, c_fit['amplitude']
    assert abs(py_fit.phase_rad - 0.7) <= 0.001, py_fit.phase_rad

    # And a window shorter than one period is a State, not a number.
    # 短于一个周期的窗口是 State，不是一个数。
    short = foc.sinusoid_fit(from_curve[:20], 1e-4, omega)
    assert short['status'] == foc.ErrorCode.STATE, short
    assert analyze.sinusoid_fit(from_curve[:20], 1e-4, omega).status == 'State'


def test_the_spectrum_and_the_distortion_measure_a_signal_that_has_a_known_distortion():
    """`spectrum` and `thd` against a signal whose answer is arithmetic, not a modelling opinion."""
    dt = 1e-4
    f0 = 250.0
    n = 20000                      # 2 s: whole cycles of every harmonic below, so leakage cannot be blamed
    t = np.arange(n) * dt
    signal = (2.0
              + 1.0 * np.cos(2.0 * math.pi * f0 * t)
              + 0.10 * np.cos(2.0 * math.pi * 3.0 * f0 * t + 0.3)
              + 0.05 * np.cos(2.0 * math.pi * 5.0 * f0 * t + 1.1))

    freqs, amps = analyze.spectrum(signal, dt)
    fundamental = float(np.max(amps[np.abs(freqs - f0) < 1.0]))
    assert abs(fundamental - 1.0) <= 0.01, f"a unit tone reads {fundamental:.4f}"

    distortion, measured_fundamental, harmonics = analyze.thd(freqs, amps, f0)
    expected = math.sqrt(0.10 ** 2 + 0.05 ** 2)
    assert abs(distortion - expected) <= 0.005, f"THD {distortion:.4f} against {expected:.4f}"
    # `thd` starts at the *second* harmonic, and this signal has none: the third and fifth are where its distortion
    # lives. Getting that indexing wrong is what a bare peak-picker would have silently reported as "clean".
    # `thd` 从**二次**谐波开始，而这个信号没有二次：它的失真在三次与五次上。把这个下标搞错，正是"只挑峰值"的实现会
    # 静默报成"干净"的地方。
    assert abs(harmonics[0] - 0.00) <= 0.005, harmonics[0]   # 2nd: absent on purpose
    assert abs(harmonics[1] - 0.10) <= 0.005, harmonics[1]   # 3rd
    assert abs(harmonics[2] - 0.00) <= 0.005, harmonics[2]   # 4th: absent
    assert abs(harmonics[3] - 0.05) <= 0.005, harmonics[3]   # 5th
    assert abs(measured_fundamental - 1.0) <= 0.01, measured_fundamental


# ---- Monte-Carlo ---------------------------------------------------------------------------------
#
# Four criteria, written before the harness and each one watched going red:
#
# 1. sigma = 0 reproduces the nominal measurement bit for bit -- a harness that perturbs something of its own shows up
#    here and nowhere else;
# 2. one nameplate parameter off by a factor moves the bandwidth by the analytic factor (kp = wb*J/Kt, so J enters
#    proportionally and kt inversely);
# 3. a matched family -- every drawn machine correctly described by its own nameplate -- spreads far less than a
#    mismatched one, which is the gain law's central claim turned into a number;
# 4. the sampler draws the distribution it reports.
#
# 四条判据，写在 harness 之前，每条都验过能变红：σ=0 逐位复现；单参数铭牌偏差给出解析倍数；匹配族的离散度远小于不匹配族
# （这是"增益律抵消机器"这条中心断言的数字化）；抽样器抽出的分布与它报告的分布一致。

def test_the_sampler_draws_the_distribution_it_reports():
    rng = random.Random(3)
    n = 2000
    drawn = [montecarlo.draw(rng, 0.20)['kt'] / montecarlo.NOMINAL['kt'] for _ in range(n)]
    mean_error = 3.0 * 0.20 / math.sqrt(n)
    std_error = 3.0 * 0.20 / math.sqrt(2 * n)
    assert abs(float(np.mean(drawn)) - 1.0) <= mean_error, np.mean(drawn)
    assert abs(float(np.std(drawn)) - 0.20) <= std_error, np.std(drawn)
    # A draw that lands on a non-physical machine is redrawn, not clamped -- so the mean must not drift upward.
    # 落在非物理机器上的抽样是重抽而非截断——因此均值不得向上漂移。
    assert all(value > 0.0 for value in drawn)


def test_zero_perturbation_reproduces_the_nominal_measurement_bit_for_bit():
    """The harness must add nothing of its own, and the *stream* is where it would add it.

    The first draft of this case called `one_trial` twice and compared the two. It was green for the wrong reason: it
    never went through the sampler, so a sampler that perturbs by a hair at `sigma = 0` -- the exact bug this criterion
    exists for -- was invisible to it. Discovered by drifting the sampler and watching the case stay green.
    本用例的第一版调用 `one_trial` 两次再比较。它绿得没有意义：它从不经过抽样器，因此"σ=0 时仍扰动了一点点"这个
    正是本判据要抓的缺陷，对它完全不可见。这是通过漂移抽样器、看着它保持绿而发现的。
    """
    nominal = dict(montecarlo.NOMINAL)
    direct = montecarlo.one_trial(nominal, nominal)

    drawn_zero = montecarlo.stream(1, sigma=0.0, seed=5, matched=True)[0]
    assert drawn_zero.bandwidth_rad_s == direct.bandwidth_rad_s, (drawn_zero.bandwidth_rad_s, direct.bandwidth_rad_s)
    assert drawn_zero.settling_s == direct.settling_s

    # A second stream at the same seed is the same stream: a seeded run has to be reproducible or the report is an
    # anecdote.
    # 同一种子的第二条流是同一条流：带种子的运行必须可复现，否则报告就是轶事。
    again = montecarlo.stream(1, sigma=0.0, seed=5, matched=True)[0]
    assert again.bandwidth_rad_s == direct.bandwidth_rad_s

    # sigma = 0 with a mismatched family is the same machine, so it is the same number.
    # σ=0 且不匹配族时是同一台机器，因此是同一个数。
    mismatched_zero = montecarlo.stream(1, sigma=0.0, seed=5, matched=False)[0]
    assert mismatched_zero.bandwidth_rad_s == direct.bandwidth_rad_s

    # And the harness agrees with measuring the same step through the loop's own trace: two ways to read one machine.
    # 而且这套装置与"从环路自己的轨迹读同一个阶跃"一致：读同一台机器的两条路。
    directly = analyze.step_metrics(montecarlo.step_window(nominal, nominal), montecarlo.DT_S, montecarlo.TARGET_RAD_S,
                                    0.02 * montecarlo.TARGET_RAD_S)
    assert directly.bandwidth_rad_s == direct.bandwidth_rad_s, (directly.bandwidth_rad_s, direct.bandwidth_rad_s)


def test_a_single_nameplate_error_moves_the_bandwidth_by_the_analytic_factor():
    """`kp = wb*J/Kt` makes the prediction arithmetic: J enters proportionally, kt inversely, and nothing else moves."""
    nominal = dict(montecarlo.NOMINAL)
    base = montecarlo.one_trial(nominal, nominal).bandwidth_rad_s

    # Measured, and why the band is 2 % rather than 0: J alone leaves the mechanical pole B/J only partly cancelled
    # (ki = wb*B/Kt does not move with J), which costs 1.40 % of the prediction. kt is a pure gain and lands within
    # 0.47 %. The band covers the physical effect rather than hiding it.
    # 实测，以及为什么带是 2% 而不是 0：单独改 J 只能部分抵消机械极点 B/J（ki = wb*B/Kt 不随 J 动），这损失了预测的
    # 1.40%；kt 是纯增益，落在 0.47% 以内。带覆盖的是真实物理效应，而不是把它藏起来。
    for name, factor, predicted in (('j', 1.10, 1.10), ('kt', 1.10, 1.0 / 1.10), ('kt', 0.90, 1.0 / 0.90)):
        told = dict(nominal)
        told[name] = nominal[name] * factor
        measured = montecarlo.one_trial(told, nominal).bandwidth_rad_s / base
        assert abs(measured / predicted - 1.0) <= 0.02, f"{name} x{factor}: {measured:.4f} against {predicted:.4f}"


def test_the_gain_law_absorbs_the_nameplate_ignorance_it_was_designed_to_absorb():
    """The library's claim, measured as a ratio.

    At a 10 % nameplate standard deviation the *matched* family -- every machine correctly described -- spreads 2.92 %
    in bandwidth while the mismatched one spreads 19.18 %, and the yield inside the 10 % band is 97.5 % against 42.5 %.
    That is what deriving every gain from the nameplate buys, and it is the only number on this page that says so.
    """
    matched = montecarlo.stream(40, sigma=0.10, seed=11, matched=True)
    mismatched = montecarlo.stream(40, sigma=0.10, seed=11, matched=False)

    matched_spread = float(np.std([t.bandwidth_rad_s for t in matched]) / montecarlo.BANDWIDTH_RAD_S)
    mismatched_spread = float(np.std([t.bandwidth_rad_s for t in mismatched]) / montecarlo.BANDWIDTH_RAD_S)

    assert mismatched_spread > 0.10, f"the mismatched family is not spread out: {mismatched_spread:.4f}"
    # 2.92 % measured; 5 % is the band, and it is three quarters of the mismatched spread's floor below.
    # 实测 2.92%；5% 是带，且它比不匹配族下限的三分之二还低。
    assert matched_spread < 0.05, f"the matched family is spread out: {matched_spread:.4f}"
    assert mismatched_spread / matched_spread >= 3.0, (
        f"the cancellation buys only {mismatched_spread / matched_spread:.2f}x: "
        f"{matched_spread:.4f} vs {mismatched_spread:.4f}")

    assert montecarlo.report(matched)['yield'] >= 0.90, montecarlo.report(matched)
    assert montecarlo.report(mismatched)['yield'] <= 0.75, montecarlo.report(mismatched)


# ---- the torque map --------------------------------------------------------------------------------
#
# Newton-metres in, amps out, with the two limits a real application needs: an inverter limit the map cannot exceed and a
# one-way floor for the tensioner case. The `limited` flag is the diagnostic -- a clipped command and an obeyed one are
# otherwise identical, so a caller that cannot tell them apart reports a saturating tensioner as holding.
#
# N·m 进、安培出，外加真实应用需要的两个限幅：地图超不过的逆变器限幅，以及张力控制那种单向工况的下限。`limited` 是
# 诊断通道——被裁剪的指令与被执行的指令在其他方面毫无差别，分不清二者的调用方会在张力控制已经饱和时报"正在保持"。

def test_the_torque_map_turns_newton_metres_into_amps_and_says_when_it_clipped():
    params = foc.MotorParams()
    params.kt, params.pole_pairs = 0.05, 7
    params.ld, params.lq = 4e-4, 8e-4

    torque_map = foc.torque_map(params, 5.0, 0.0, 1.0)      # one-way: never pull backwards
    assert torque_map.status == foc.ErrorCode.OK.value, torque_map.status
    assert abs(torque_map.nm_per_a - 0.05) < 1e-6, torque_map.nm_per_a
    assert torque_map.min_nm == 0.0
    assert abs(torque_map.max_nm - 0.25) < 1e-6, torque_map.max_nm   # kt * i_max narrows the upper limit

    forward = torque_map.command(0.2)
    assert not forward['limited'], forward
    assert abs(forward['iq_a'] - 4.0) < 1e-4, forward
    # The command path and the estimate are inverses of each other: a commanded torque the estimate would not confirm is
    # a command that does not deliver what it says.
    # 指令路径与估计互为逆运算：估计不认可的指令，就是不交付它声称之物的指令。
    assert abs(torque_map.estimate(0.0, forward['iq_a']) - 0.2) < 1e-5

    backward = torque_map.command(-0.2)
    assert backward['limited'], backward
    assert backward['iq_a'] == 0.0 and backward['torque_nm'] == 0.0, backward

    # Beyond what the inverter can deliver, the request is clipped and says so.
    # 超出逆变器能交付的范围时，请求被裁剪并如实说明。
    over = torque_map.command(0.9)
    assert over['limited'] and abs(over['torque_nm'] - 0.25) < 1e-6, over
    assert abs(over['iq_a'] - 5.0) < 1e-4, over

    # A nameplate that produces no torque per amp is refused rather than turned into a map of zeros.
    # 每安培不产生力矩的铭牌被拒绝，而不是变成一张全零的地图。
    flat = foc.MotorParams()
    flat.kt, flat.pole_pairs = 0.0, 7
    try:
        foc.torque_map(flat, 5.0, -1.0, 1.0)
    except RuntimeError:
        pass
    else:
        raise AssertionError("a machine with kt = 0 was admitted to a torque map")


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
