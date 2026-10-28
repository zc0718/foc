"""Second reconciliation table: our control loop against motulator's, on one plant.

The first table (`03_model_reconcile.py`) compared two *machine models* with the voltage dictated by the harness. This
one compares two *controllers* and gives both the same machine -- motulator's `SynchronousMachine` -- so the only
difference left is the control loop itself. Ours reaches that machine through the library's host HAL (`foc.Motor`,
i.e. the C ABI, not a Python re-implementation); theirs is `CurrentVectorControl` driven by hand.

Both legs use the same integrator, the same initial state, the same 50 us step, the same 300 V bus, the same command,
the same prescribed speed schedule, and both read the true rotor angle from the machine -- sensor-based on both sides,
so no observer asymmetry is mixed in.

    source workspace/materials/.venv/bin/activate
    LIB=$(find ~/.conan2/p/b -name "foc_python.cpython-310*.so" | head -1)
    PYTHONPATH=$(dirname "$LIB") python benchmark/script/reconcile_controller.py

The speed is *prescribed*, not integrated from torque. An unloaded machine accelerating under its own torque mixes the
mechanical design into the comparison, and the first version of this file did exactly that and settled at 0.66 of the
current command. That 0.66 turned out to be physics rather than a defect: a PI current loop has no steady-state error
against a *step* disturbance (constant back-EMF) but a finite one against a *ramp* (back-EMF changing at psi_f*alpha),

    delta_i = psi_f * alpha / k_i,    k_i = alpha_c * R_s

which for alpha = 2800 rad/s^2 gives 0.212 A. That prediction was written down before the run and is the expected
answer in the ramp phases, not the command. Encountering it as an unexpected number first is the reason the ramp
phases exist at all.

Criterion, written before the run:
  * held phases: q-axis current within 10 % of the 0.5 A command, in both legs
  * ramp phases:  q-axis current within 10 % of the PI ramp prediction, in both legs
  * the two legs within 10 % of each other on the q axis at every phase
  * current-vector misalignment (atan2 of the true d over q current) within 0.15 rad for **our** leg in every phase

Dead time is 0 on our side: motulator's PWM does not model it, and a dead-time model on one side only would measure
the harness rather than the control law.

motulator's own `CurrentVectorControl` is driven through the same plant. Getting it to run at all took two fixes in
this harness, neither of them a claim about their controller: the feedback voltage has to be what their PWM reports as
*realized* (delay compensation and the two-step average included) rather than a reconstruction of the duty, and a cold
start at full speed winds their field-weakening integrator down to the current limit. With those fixed its q axis lands
on the same predictions as ours. Its **d** axis carries an offset that appears while the speed is changing (-0.010 A
when the speed is held, -0.127 A while ramping to 800 rad/s), so the misalignment criterion is asserted for our leg
only. That offset was later confirmed in their own simulation machinery and is theirs, not this adapter's --
`official_cvc_check.py` -- but this adapter does not reproduce its *magnitude* in either direction, so treat the d
column here as indicative only.
"""

import math
import sys
from types import SimpleNamespace

import numpy as np

import foc_python as foc
from motulator.drive.control.sm import CurrentReferenceCfg, CurrentVectorControl
from motulator.drive.model import SynchronousMachine
from motulator.drive.utils import SynchronousMachinePars

DT = 50e-6
N_P, RS, LD, LQ, PSI_F = 1, 0.5, 4.0e-4, 4.0e-4, 0.0476
V_BUS = 300.0
IQ_COMMAND = 0.5
TAU_COMMAND = 1.5 * N_P * PSI_F * IQ_COMMAND
I_MAX = 5.0
ALPHA_C = 2.0 * math.pi * 200.0

# (name, start speed rad/s, prescribed acceleration rad/s^2, ticks). The measurement is taken over each phase's last
# fifth, so the transition into a phase does not pollute its row.
PHASES = (
    ("standstill", 0.0, 0.0, 2000),
    ("ramp to 400", 0.0, 2800.0, 2860),
    ("hold at 400", 400.0, 0.0, 1500),
    ("ramp to 800", 400.0, 2800.0, 2860),
    ("hold at 800", 800.0, 0.0, 1500),
)


def ramp_error_a(accel):
    """The steady-state error a first-order-bandwidth current loop must show against a ramping back-EMF."""
    return PSI_F * accel / (ALPHA_C * RS)


class Machine:
    """motulator's synchronous machine integrated forward Euler -- the one plant used twice."""

    def __init__(self, par):
        self.par = par
        self.mach = SynchronousMachine(par)
        self.w_m = 0.0
        self.mach.inp.w_M = 0.0
        self.mach.set_outputs(0.0)

    # ---- measurements -------------------------------------------------------
    @property
    def i_s(self):
        """Stator current in rotor coordinates (motulator's `i_s` is already in the rotor frame)."""
        return self.mach.i_s

    @property
    def i_ss(self):
        """Stator current in stator coordinates (peak scaling)."""
        return self.mach.out.i_ss

    @property
    def i_uvw(self):
        i_ss = self.i_ss
        iu = i_ss.real
        iv = (i_ss * np.exp(-1j * 2.0 * math.pi / 3.0)).real
        return (iu, iv, -iu - iv)

    @property
    def theta_mech_rad(self):
        return float(np.angle(self.mach.state.exp_j_theta_m)) % (2.0 * math.pi)

    @property
    def theta_elec_rad(self):
        return self.par.n_p * self.theta_mech_rad

    @property
    def misalignment_rad(self):
        """Angle between the true current vector and the q axis of the commanded frame."""
        i = self.i_s
        return math.atan2(i.real, i.imag) if abs(i) > 1e-6 else 0.0

    # ---- one step -----------------------------------------------------------
    def advance(self, u_alpha, u_beta):
        self.mach.inp.u_ss = complex(u_alpha, u_beta)
        self.mach.inp.w_M = self.w_m
        self.mach.set_outputs(0.0)
        d_psi, d_exp = self.mach.rhs()
        self.mach.state.psi_s += DT * d_psi
        exp_j = self.mach.state.exp_j_theta_m + DT * d_exp
        self.mach.state.exp_j_theta_m = exp_j / abs(exp_j)


def duty_to_stator_voltage(du, dv, dw):
    """The same duty -> voltage -> stator-frame mapping on both legs."""
    vu, vv, vw = du * V_BUS, dv * V_BUS, dw * V_BUS
    valpha = (2.0 / 3.0) * (vu - 0.5 * vv - 0.5 * vw)
    vbeta = (2.0 / 3.0) * ((math.sqrt(3.0) / 2.0) * vv - (math.sqrt(3.0) / 2.0) * vw)
    return valpha, vbeta


class OurPlant:
    """The host-HAL contract our Python binding expects, backed by the shared machine."""

    def __init__(self, machine):
        self.machine = machine

    def write_duty(self, du, dv, dw):
        self.machine.advance(*duty_to_stator_voltage(du, dv, dw))

    def read_current(self):
        return self.machine.i_uvw

    def angle_mech_rad(self):
        return self.machine.theta_mech_rad

    def voltage_v(self):
        return V_BUS

    def velocity_rad_s(self):
        return self.machine.w_m


def schedule(machine, k):
    """The prescribed speed schedule: identical for both legs, applied outside the controller."""
    t = k * DT
    for index, (_, w0, accel, ticks) in enumerate(PHASES):
        span = ticks * DT
        if t < span - 1e-12:
            machine.w_m = w0 + accel * t
            return index
        t -= span
    machine.w_m = PHASES[-1][1]
    return len(PHASES) - 1


def summarise(rows):
    """Mean over the last fifth of each phase, and the worst misalignment in that same window."""
    out = []
    for (name, _, accel, ticks), vals in zip(PHASES, rows):
        tail = vals[int(0.8 * ticks):]
        iq = sum(v[0] for v in tail) / len(tail)
        id_ = sum(v[1] for v in tail) / len(tail)
        mis = max(abs(v[2]) for v in tail)
        out.append((name, accel, iq, id_, mis))
    return out


def run_ours(par):
    machine = Machine(par)
    params = foc.MotorParams()
    params.rs, params.ld, params.lq = RS, LD, LQ
    params.kt = 1.5 * N_P * PSI_F
    params.j, params.b, params.pole_pairs = 1.0e-5, 1.0e-6, N_P

    setup = foc.MotorSetup()
    setup.i_max_a = I_MAX
    setup.pwm_freq_hz = 1.0 / DT
    setup.deadtime_ns = 0.0
    setup.v_bus_v = V_BUS
    setup.current_bandwidth_rad_s = ALPHA_C

    rows = [[] for _ in PHASES]
    with foc.Motor(params, setup, OurPlant(machine)) as motor:
        motor.enable()
        motor.set_torque(IQ_COMMAND)
        for k in range(sum(p[3] for p in PHASES)):
            motor.tick(DT)
            phase = schedule(machine, k + 1)
            i = machine.i_s
            rows[phase].append((i.imag, i.real, machine.misalignment_rad))
    return summarise(rows)


def run_theirs(par):
    machine = Machine(par)
    cfg = CurrentReferenceCfg(par, max_i_s=I_MAX, nom_w_m=PHASES[-1][1])
    ctrl = CurrentVectorControl(par, cfg, T_s=DT, J=None, alpha_c=ALPHA_C, sensorless=False)
    ctrl.ref.tau_M = lambda _t: TAU_COMMAND

    rows = [[] for _ in PHASES]
    # Their framework feeds back the voltage the PWM says it realized -- delay compensation and the two-step average
    # included -- not a reconstruction of the duty. Using a reconstruction instead left a steady d-axis error of 0.39 A
    # at 400 rad/s, which was a defect of this harness and looked exactly like one of theirs.
    u_ss_fb = ctrl.pwm.get_realized_voltage()
    for k in range(sum(p[3] for p in PHASES)):
        phase = schedule(machine, k + 1)
        theta_m = machine.theta_elec_rad
        # Exactly the fields `CurrentVectorControl.get_feedback_signals` adds in the sensor-based branch.
        fbk = SimpleNamespace(
            u_dc=V_BUS,
            i_ss=machine.i_ss,
            u_ss=u_ss_fb,
            theta_m=theta_m,
            w_m=machine.w_m,
            i_s=np.exp(-1j * theta_m) * machine.i_ss,
            u_s=np.exp(-1j * theta_m) * u_ss_fb,
            w_s=machine.w_m,
        )
        ref = ctrl.output(fbk)
        u_ss = complex(*duty_to_stator_voltage(*ref.d_abc))
        u_ss_fb = ctrl.pwm.get_realized_voltage()
        machine.advance(u_ss.real, u_ss.imag)
        ctrl.update(fbk, ref)
        i = machine.i_s
        rows[phase].append((i.imag, i.real, machine.misalignment_rad))
    return summarise(rows)


def main() -> int:
    par = SynchronousMachinePars(N_P, RS, LD, LQ, PSI_F)
    ours = run_ours(par)
    theirs = run_theirs(par)

    print(f"iq command {IQ_COMMAND:+.3f} A, current bandwidth {ALPHA_C:.0f} rad/s, ramp-error coefficient "
          f"{PSI_F / (ALPHA_C * RS) * 1e6:.1f} uA per rad/s^2")
    header = (f"{'phase':>12}  {'accel':>7}  {'speed':>6}  {'iq ours':>8} {'iq theirs':>9} {'target':>7} "
              f"{'gap %':>6}  {'id ours':>8} {'id theirs':>9}  {'misalign ours':>13}")
    print(header)
    print("-" * len(header))
    ok = True
    for (name, w0, accel, _), a, b in zip(PHASES, ours, theirs):
        target = IQ_COMMAND - ramp_error_a(accel)
        gap = abs(a[2] - b[2]) / IQ_COMMAND * 100.0
        print(f"{name:>12}  {accel:7.0f}  {w0:6.0f}  {a[2]:8.4f} {b[2]:9.4f} {target:7.4f} {gap:5.2f}%  "
              f"{a[3]:8.4f} {b[3]:9.4f}  {a[4]:13.4f}")
        for tag, row in (("ours", a), ("theirs", b)):
            error = abs(row[2] - target) / IQ_COMMAND * 100.0
            if error > 10.0:
                print(f"  FAIL: {tag} iq {row[2]:+.4f} A is {error:.2f} % of the command away from the predicted "
                      f"{target:+.4f} A in phase '{name}'")
                ok = False
        if gap > 10.0:
            print(f"  FAIL: the legs differ by {gap:.2f} % on iq in phase '{name}'")
            ok = False
        if a[4] > 0.15:
            print(f"  FAIL: our current-vector misalignment {a[4]:.4f} rad in phase '{name}'")
            ok = False

    print()
    print("their misalignment is reported, not asserted: it is smallest where the speed is held and largest during the "
          "ramp, which this harness cannot separate from its own plumbing.")
    for (name, _, _, _), b in zip(PHASES, theirs):
        print(f"  {name:>12}  id theirs {b[3]:+.4f} A  misalignment {b[4]:+.4f} rad")

    print()
    print("OK: both control loops reach the commanded operating point, and follow the PI ramp law, on the same machine "
          "at every prescribed speed" if ok else "NOT OK")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
