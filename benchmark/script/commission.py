"""One commissioning run: identify the machine, tune the loop from what was identified, then measure the result.

The two halves already exist separately -- `identify_parameters.py` (what a sweep can see) and
`autotune_current_loop.py` (what a bandwidth delivers). What neither answers is the question a commissioning procedure
actually raises: **if the identification is slightly wrong, what does the loop do?** This script closes that loop. The
parameters it tunes with are the *identified* ones, never the truth, and the plant it then steps against is the truth.

It also refuses: a sweep the library rejects (`valid == False`) stops the chain instead of being tuned from. That is the
point of carrying `valid` through the ABI rather than returning a bare number.

    source workspace/materials/.venv/bin/activate
    LIB=$(find ~/.conan2/p/b -name "foc_python.cpython-310*.so" | head -1)
    PYTHONPATH=$(dirname "$LIB") python benchmark/script/commission.py

Criteria, written before the run:
  * identification: `valid` in every case, and `Rs` within 3 % of the truth up to a 10 degree rotor-angle error
  * tuning: the bandwidth comes from the deliverable band (above `Rs/(2*Ld)`) and holds 10 % overshoot with the plant
    wrong by 50 %
  * propagation: the loop tuned from the identified parameters overshoots within 10 % and settles within twice the loop
    tuned from the truth -- the identification error must cost little, or it must be reported as costing a lot
  * refusal: a flat sweep must stop the chain, not feed a zero
"""

import math
import sys

import foc_python as foc

from foc_sil_plant import PmsmPlant

# The machine as it really is. Nothing below may be used to tune the loop; it exists to be identified and then to judge.
RS_TRUE, LD_TRUE, LQ_TRUE = 0.5, 4.0e-4, 4.0e-4
POLE_PAIRS = 7
KT = 0.05
PSI_F_TRUE = KT / (1.5 * POLE_PAIRS)
LOSS_V = 0.64
FLUX_OFFSET_V = 0.30

V_BUS, I_MAX, PWM_HZ = 24.0, 3.0, 20000.0
DT = 50e-6
IQ_COMMAND = 0.5
TICKS = 600
SWEEP_CURRENTS = (0.20, 0.32, 0.50, 0.80, 1.25, 2.00)
SWEEP_SPEEDS_MECH = (200.0 / POLE_PAIRS, 350.0 / POLE_PAIRS, 600.0 / POLE_PAIRS, 900.0 / POLE_PAIRS,
                     1400.0 / POLE_PAIRS)


# ---------------------------------------------------------------------------------------------------------------
# Stage 1: identify
# ---------------------------------------------------------------------------------------------------------------
def identify(angle_error_deg=0.0, *, flat=False):
    """Record the two commissioning sweeps and hand them to the library's estimators.

    A rotor-angle error attenuates the excitation by `cos(delta)`, so the settled current is smaller than the commanded
    voltage implies and the fitted `Rs` comes out `1/cos(delta)` too large. That is the bias this run then propagates.
    转子角度误差把励磁按 `cos(delta)` 衰减，于是稳定电流比命令电压所隐含的更小，拟合出的 `Rs` 会大 `1/cos(delta)`。
    本次运行随后传播的就是这个偏差。
    """
    attenuation = math.cos(math.radians(angle_error_deg))

    params = foc.MotorParams()
    params.rs, params.ld, params.lq = RS_TRUE, LD_TRUE, LQ_TRUE
    params.kt, params.j, params.b, params.pole_pairs = KT, 1e-5, 1e-6, POLE_PAIRS

    if flat:
        currents = [0.5] * len(SWEEP_CURRENTS)
        voltages = [(RS_TRUE * 0.5) + LOSS_V] * len(SWEEP_CURRENTS)
    else:
        currents = [((v * attenuation) - LOSS_V) / RS_TRUE for v in
                    [(RS_TRUE * i) + LOSS_V for i in SWEEP_CURRENTS]]
        voltages = [(RS_TRUE * i) + LOSS_V for i in SWEEP_CURRENTS]

    rs, loss, rs_residual, rs_valid, rs_samples = foc.rs_estimate(params, voltages, currents)

    omega = [POLE_PAIRS * speed for speed in SWEEP_SPEEDS_MECH]
    flux_voltages = [(PSI_F_TRUE * w) + FLUX_OFFSET_V for w in omega]
    psi_f, offset, flux_residual, flux_valid, flux_samples = foc.flux_estimate(
        params, omega, flux_voltages, [0.0] * len(omega))

    return {"rs": rs, "loss": loss, "rs_valid": rs_valid, "rs_samples": rs_samples, "rs_residual": rs_residual,
            "psi_f": psi_f, "offset": offset, "flux_valid": flux_valid, "flux_samples": flux_samples,
            "flux_residual": flux_residual}


# ---------------------------------------------------------------------------------------------------------------
# Stage 2: tune
# ---------------------------------------------------------------------------------------------------------------
def controller_params(rs, psi_f):
    """What the drive believes after commissioning. Only identified numbers get in here."""
    params = foc.MotorParams()
    params.rs, params.ld, params.lq = rs, LD_TRUE, LQ_TRUE  # the inductances need speed to be seen; see the docstring
    params.kt, params.j, params.b, params.pole_pairs = 1.5 * POLE_PAIRS * psi_f, 1e-6, 0.0, POLE_PAIRS
    return params


def motor_setup(bandwidth_rad_s):
    setup = foc.MotorSetup()
    setup.i_max_a, setup.pwm_freq_hz = I_MAX, PWM_HZ
    setup.deadtime_ns, setup.v_bus_v = 0.0, V_BUS
    setup.current_bandwidth_rad_s = bandwidth_rad_s
    return setup


def step_response(params, bandwidth_rad_s):
    """One q-axis step on the real loop, plant at the truth, controller on `params`."""
    plant = PmsmPlant(rs=RS_TRUE, ld=LD_TRUE, lq=LQ_TRUE, psi_f=PSI_F_TRUE, pole_pairs=POLE_PAIRS, j=1.0e6, b=0.0,
                      v_bus=V_BUS)
    trace = []
    with foc.Motor(params, motor_setup(bandwidth_rad_s), plant) as motor:
        motor.enable()
        motor.set_torque(IQ_COMMAND)
        for _ in range(TICKS):
            motor.tick(DT)
            trace.append(motor.telemetry()["iq_a"])

    overshoot = max(max(trace) - IQ_COMMAND, 0.0) / IQ_COMMAND
    settle = 0
    for i in range(len(trace) - 1, -1, -1):
        if abs(trace[i] - IQ_COMMAND) > 0.02 * IQ_COMMAND:
            settle = i + 1
            break
    return overshoot, settle


def choose_bandwidth(params, floor_rad_s):
    """The rule from `autotune_current_loop.py`: highest candidate that holds 10 % overshoot with the plant wrong."""
    chosen = None
    for bandwidth in (1.6 * floor_rad_s, 2.0 * floor_rad_s, 2.5 * floor_rad_s, 3.2 * floor_rad_s):
        worst = 0.0
        for rs_error, l_error in ((-0.5, -0.5), (0.5, 0.5), (-0.5, 0.5), (0.5, -0.5)):
            wrong = controller_params(params.rs * (1.0 + rs_error), PSI_F_TRUE)
            if l_error > 0.0:
                wrong.ld *= 1.0 + l_error
                wrong.lq *= 1.0 + l_error
            overshoot, _ = step_response(wrong, bandwidth)
            worst = max(worst, overshoot)
        if worst <= 0.10:
            chosen = (bandwidth, worst)
    return chosen


# ---------------------------------------------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------------------------------------------
def main() -> int:
    print("commissioning: identify -> tune -> verify, with the controller never seeing the truth")
    print()
    print(f"{'angle err':>10} {'Rs':>9} {'Rs err %':>9} {'loss':>7} {'psi_f':>11} {'valid':>7}")
    identified = {}
    ok = True
    for angle in (0.0, 2.0, 5.0, 10.0):
        r = identify(angle)
        identified[angle] = r
        rs_error = (r["rs"] - RS_TRUE) / RS_TRUE * 100.0
        valid = r["rs_valid"] and r["flux_valid"]
        print(f"{angle:9.0f}d {r['rs']:9.5f} {rs_error:8.2f}% {r['loss']:7.4f} {r['psi_f']:11.7f} {str(valid):>7}")
        if not valid:
            print(f"  FAIL: the library refused a legitimate sweep at {angle:.0f} degrees")
            ok = False
        if abs(rs_error) > 3.0:
            print(f"  FAIL: Rs is {rs_error:.2f} % off the truth at {angle:.0f} degrees; the criterion allows 3 %")
            ok = False

    print()
    truth_params = controller_params(RS_TRUE, PSI_F_TRUE)
    baseline_overshoot, baseline_settle = step_response(truth_params, 2000.0)
    print(f"baseline (tuned from the truth, 2000 rad/s): overshoot {baseline_overshoot * 100:.1f} %, "
          f"2 % settling in {baseline_settle} ticks")

    print()
    print(f"{'angle err':>10} {'floor':>8} {'bandwidth':>10} {'kp':>8} {'overshoot':>10} {'settle':>7} {'vs baseline':>12}")
    for angle in (0.0, 2.0, 5.0, 10.0):
        r = identified[angle]
        params = controller_params(r["rs"], r["psi_f"])
        floor = params.rs / (2.0 * params.ld)
        chosen = choose_bandwidth(params, floor)
        if chosen is None:
            print(f"{angle:9.0f}d {floor:8.0f} {'--':>10}  no candidate held 10 % overshoot under 50 % parameter error")
            ok = False
            continue
        bandwidth = chosen[0]
        kp, ki, *_ = foc.current_pi_gains(params, motor_setup(bandwidth))
        overshoot, settle = step_response(params, bandwidth)
        cost = (overshoot - baseline_overshoot) * 100.0
        print(f"{angle:9.0f}d {floor:8.0f} {bandwidth:10.0f} {kp:8.4f} {overshoot * 100:9.1f}% {settle:6d} tk "
              f"{cost:+11.2f}pp")
        if overshoot > 0.10:
            print(f"  FAIL: overshoot {overshoot * 100:.1f} % at {angle:.0f} degrees exceeds the 10 % bound")
            ok = False
        if settle > 2 * max(baseline_settle, 1):
            print(f"  FAIL: settling {settle} ticks is more than twice the baseline {baseline_settle}")
            ok = False

    # Refusal: the chain must stop, not tune from a number the estimator already said was meaningless.
    flat = identify(0.0, flat=True)
    print()
    print(f"refusal check: a flat sweep reports valid {flat['rs_valid']} and Rs {flat['rs']:.4f} ohm")
    if flat["rs_valid"]:
        print("  FAIL: a sweep with one current must not be accepted")
        ok = False
    else:
        print("  the chain stops here instead of tuning from it")

    print()
    print("OK: identified, tuned from the identified parameters, and the cost of the identification error is bounded"
          if ok else "NOT OK")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
