"""The controller as an FMU, loaded by a master that knows nothing about this repository.

`api/fmu/foc_controller.fmu` is the library's loop packaged behind FMI 2.0 for co-simulation. The
claim that matters is not "we built a zip": it is that a third-party master can instantiate the
block, feed it measurements and read back duties. FMPy is that master here, and nothing in this
script is customised for the library -- if it needed a special case, the FMU would be wrong rather
than the master.

    python3 fmu-cosimulation.py [--fmu api/fmu/foc_controller.fmu]

The master owns the truth (it moves the rotor and reports the currents it measured); the FMU owns
nothing but the loop. That division is what makes the block reusable in someone else's plant model.
"""

import argparse
import pathlib
import tempfile

DT = 50e-6
TICKS = 2000
BUS_V = 24.0
IQ_COMMAND_A = 0.5
SPEED_MECH_RAD_S = 200.0


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--fmu", default="api/fmu/foc_controller.fmu")
    args = parser.parse_args()

    from fmpy import extract, read_model_description
    from fmpy.fmi2 import FMU2Slave

    fmu_path = pathlib.Path(args.fmu)
    if not fmu_path.is_file():
        raise SystemExit(f"{fmu_path} does not exist; build it with api/fmu/build.py first")

    # 1. the interface, before anything is instantiated: a variable the master cannot name is a
    # variable the master cannot drive, and finding that out here gives a better message than
    # finding it out at instantiation.
    description = read_model_description(str(fmu_path))
    vr = {variable.name: variable.valueReference for variable in description.modelVariables}
    print(f"model {description.modelName}, FMI {description.fmiVersion}, {len(vr)} variables")

    # 2. instantiate and enter the co-simulation loop
    unzipdir = extract(str(fmu_path), unzipdir=tempfile.mkdtemp(prefix="foc-fmu-"))
    fmu = FMU2Slave(guid=description.guid, unzipDirectory=unzipdir,
                    modelIdentifier=description.coSimulation.modelIdentifier, instanceName="foc")
    fmu.instantiate()
    fmu.setupExperiment(startTime=0.0)
    fmu.enterInitializationMode()
    fmu.exitInitializationMode()

    fmu.setReal([vr["v_bus_v"]], [BUS_V])
    fmu.setReal([vr["iq_command_a"]], [IQ_COMMAND_A])
    fmu.setBoolean([vr["enable"]], [True])

    # 3. drive it: the master advances the rotor and reports what it measured, the FMU answers
    # with the duties it wants applied
    time, angle = 0.0, 0.0
    duty = (0.5, 0.5, 0.5)
    for _ in range(TICKS):
        angle += SPEED_MECH_RAD_S * DT
        fmu.setReal([vr["angle_mech_rad"]], [angle])
        fmu.setReal([vr["i_u_a"], vr["i_v_a"], vr["i_w_a"]], [0.0, 0.0, 0.0])
        fmu.doStep(currentCommunicationPoint=time, communicationStepSize=DT)
        time += DT
        duty = fmu.getReal([vr["duty_u"], vr["duty_v"], vr["duty_w"]])

    state = fmu.getInteger([vr["state"]])[0]
    error = fmu.getInteger([vr["error"]])[0]
    print(f"duty u/v/w = {duty[0]:.4f} / {duty[1]:.4f} / {duty[2]:.4f}   state {state}   error {error}")

    # 4. the observable contract only: inside the inverter's range, and actually asking for
    # something (three duties pinned at 0.5 would be a block that never drives anything)
    if not all(0.0 <= d <= 1.0 for d in duty):
        print("a duty left [0, 1]: the block is not driving an inverter any more")
        return 1
    if max(duty) - min(duty) < 1e-3:
        print("the duties never moved: the loop answered nothing")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
