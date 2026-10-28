#!/usr/bin/env python3
"""Load `foc_controller.fmu` in a standard FMI master and drive the library's loop.

Building an FMU is a zip and a shared object. *Loading* it is the claim that matters: a third-party master, which knows
nothing about this repository, must be able to instantiate the block, feed it measurements and read back duties. FMPy is
that master here, and nothing about it is customised for this library -- if this script needed a special case, the FMU
would be wrong rather than the master.

    python3 api/fmu/smoke.py [--fmu api/fmu/foc_controller.fmu]

The script deliberately asserts on the *observable* contract only: which mode the controller is in, whether it reports an
error, and whether the duties it produced are inside the inverter's range and actually moved. Anything stronger would be
testing this repository's controller through a channel that a third-party tool cannot see.

构建一个 FMU 只是打包一个 zip 加一个共享库。**加载**它才是要证明的主张：一个对本库一无所知的第三方主控必须能实例化这个
块、喂进测量量、读回占空比。这里的第三方主控是 FMPy，而它没有为本库做任何定制——如果这个脚本需要特例，错的会是 FMU 而
不是主控。
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]

# The measurements the master owns, in SI. 1400 mechanical rad/s electrical-equivalent motion over 7 pole pairs is a
# comfortable mid-speed operating point, and 50 us is the rate the C path is tuned for.
# 主控拥有的测量量，SI 单位。7 极对下 1400 机械弧度/秒的等效运动是一个舒适的中速工作点，50 µs 是 C 路径整定时用的周期。
DT = 50e-6
TICKS = 2000
BUS_V = 24.0
IQ_COMMAND_A = 0.5
SPEED_MECH_RAD_S = 1400.0 / 7.0

INPUTS = ("v_bus_v", "iq_command_a", "enable", "angle_mech_rad", "i_u_a", "i_v_a", "i_w_a")
OUTPUTS = ("duty_u", "duty_v", "duty_w", "state", "error")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fmu", default=str(ROOT / "api" / "fmu" / "foc_controller.fmu"))
    args = parser.parse_args()

    try:
        from fmpy import extract, read_model_description
        from fmpy.fmi2 import FMU2Slave
    except ImportError:  # pragma: no cover - the gate reports this as a skip, not a pass
        print("[fmu] fmpy is not importable; this check needs an interpreter that has it", file=sys.stderr)
        return 2

    fmu_path = pathlib.Path(args.fmu)
    if not fmu_path.is_file():
        print(f"[fmu] {fmu_path} does not exist; run api/fmu/build.py first", file=sys.stderr)
        return 2

    description = read_model_description(str(fmu_path))
    print(f"[fmu] model {description.modelName} (FMI {description.fmiVersion}), guid {description.guid}")
    vr = {variable.name: variable.valueReference for variable in description.modelVariables}

    # Before instantiating: the description and the wrapper have to agree about the interface. A variable the master
    # cannot name is a variable the master cannot drive, and finding that out at instantiation gives a worse message than
    # finding it out here.
    # 在实例化之前：描述文件与包装器必须对接口达成一致。主控叫不出名字的变量，主控也驱动不了；在实例化时才发现，得到的
    # 消息比在这里发现更差。
    missing = [name for name in (*INPUTS, *OUTPUTS) if name not in vr]
    if missing:
        print(f"[fmu] modelDescription is missing {missing}", file=sys.stderr)
        return 1

    unzipdir = extract(str(fmu_path), unzipdir=tempfile.mkdtemp(prefix="foc-fmu-"))
    fmu = FMU2Slave(
        guid=description.guid,
        unzipDirectory=unzipdir,
        modelIdentifier=description.coSimulation.modelIdentifier,
        instanceName="foc",
    )
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    fmu.setReal([vr["v_bus_v"]], [BUS_V])
    fmu.setReal([vr["iq_command_a"]], [IQ_COMMAND_A])
    fmu.setBoolean([vr["enable"]], [True])

    time = 0.0
    angle = 0.0
    duty = (0.5, 0.5, 0.5)
    for _ in range(TICKS):
        # The master owns the truth: it moves the rotor and reports the currents it measured. The FMU owns nothing but
        # the loop, which is exactly the division an FMU has to make.
        # 真值归主控：它转动转子、报出它测到的电流。FMU 只拥有环路本身，而这正是 FMU 必须做的划分。
        angle += SPEED_MECH_RAD_S * DT
        fmu.setReal([vr["angle_mech_rad"]], [angle])
        fmu.setReal([vr["i_u_a"], vr["i_v_a"], vr["i_w_a"]], [0.0, 0.0, 0.0])
        fmu.doStep(currentCommunicationPoint=time, communicationStepSize=DT)
        time += DT
        duty = fmu.getReal([vr["duty_u"], vr["duty_v"], vr["duty_w"]])

    state = fmu.getInteger([vr["state"]])[0]
    error = fmu.getInteger([vr["error"]])[0]
    fmu.terminate()
    fmu.freeInstance()

    print(f"[fmu] state {state} (2 = Run), error {error}, duty {duty[0]:.4f} {duty[1]:.4f} {duty[2]:.4f}")

    # Each condition a separate line, so a red run says which part of the contract broke instead of "NOT OK".
    # 每个条件一行，于是失败时说得出契约的哪一部分坏了，而不是笼统的 "NOT OK"。
    failures = []
    if state != 2:
        failures.append(f"the controller is not in Run (state {state})")
    if error != 0:
        failures.append(f"the controller reports an error ({error})")
    if not all(0.0 <= d <= 1.0 for d in duty):
        failures.append(f"a duty left [0, 1]: {duty}")
    if not any(abs(d - 0.5) > 1e-3 for d in duty):
        failures.append(f"every duty stayed at 0.5, so the loop never modulated: {duty}")
    for failure in failures:
        print(f"[fmu] FAIL {failure}", file=sys.stderr)

    if failures:
        return 1
    print("[fmu] OK: a standard FMI master loaded the FMU and drove the library's loop")
    return 0


if __name__ == "__main__":
    sys.exit(main())
