"""Reconcile our machine model against a third-party one, openly.

This is the check that the saliency episode said we were missing: two models, the same voltage and the same speed,
compared step by step. Our plant is the dq model the SIL rigs use; the other is `motulator`'s (Aalto Electric Drives),
which is laboratory-validated and nothing to do with this repository.

Criterion, written before the run: with a voltage step applied to a machine spinning at 200 rad/s electrical, the stator
current vector must agree within 1 % in magnitude and 0.01 rad in angle for the whole trajectory. Disagreement is a
finding about one of the models -- it is not to be tuned away.

    source workspace/materials/.venv/bin/activate
    python workspace/materials/lab/03_model_reconcile.py
"""

import cmath
import math
import sys

import numpy as np
from motulator.drive.model import SynchronousMachine
from motulator.drive.utils._helpers import SynchronousMachinePars

DT = 50e-6
STEPS = 400
OMEGA_E = 200.0
# One machine, stated once. Same numbers on both sides; nothing here is fitted to make the comparison agree.
# 一台机器，只陈述一次。两边同一组数字；这里没有任何为了对上而凑的参数。
N_P, RS, LD, LQ, PSI_F = 1, 0.5, 4.0e-4, 4.0e-4, 0.0476


class OurPlant:
    """The dq model our SIL rigs integrate, in its simplest form."""

    def __init__(self):
        self.psi_d = PSI_F
        self.psi_q = 0.0

    def i_s(self):
        return complex((self.psi_d - PSI_F) / LD, self.psi_q / LQ)

    def step(self, u_ss, omega_e):
        u_d = u_ss.real
        u_q = u_ss.imag
        i_s = self.i_s()
        # dψd/dt = ud - Rs·id + ω·ψq ;  dψq/dt = uq - Rs·iq - ω·ψd
        # 这两行的符号就是 dq 磁链方程的全部内容；写反一个就会和任何第三方模型对不上。
        d_psi_d = u_d - RS * i_s.real + omega_e * self.psi_q
        d_psi_q = u_q - RS * i_s.imag - omega_e * self.psi_d
        self.psi_d += DT * d_psi_d
        self.psi_q += DT * d_psi_q


def main() -> int:
    pars = SynchronousMachinePars(n_p=N_P, R_s=RS, L_d=LD, L_q=LQ, psi_f=PSI_F)
    theirs = SynchronousMachine(pars)
    ours = OurPlant()

    theirs.inp.w_M = OMEGA_E / N_P
    theirs.state.exp_j_theta_m = cmath.exp(1j * 0.0)

    worst_magnitude = 0.0
    worst_angle = 0.0
    for k in range(STEPS):
        # A ramp on the stator voltage, so the trajectory visits more than one operating point.
        # 定子电压取一段斜坡，让轨迹经过不止一个工作点。

        u_dq = complex(2.0 + 0.01 * k, 1.0)  # the same physical voltage, stated in rotor coordinates
        ours.step(u_dq, OMEGA_E)
        i_ours = ours.i_s()

        # motulator takes the stator voltage in *stator* coordinates, so the rotor-frame command has to be rotated
        # out to the stator frame by the same angle the rotor has turned through.
        # motulator 的输入是**定子坐标系**的定子电压，所以转子坐标系的给定必须按转子已转过的角度旋回定子坐标系。
        theta_e = OMEGA_E * k * DT
        theirs.inp.u_ss = u_dq * cmath.exp(1j * theta_e)
        theirs.set_outputs(0.0)  # `rhs()` reads `out.i_s`, so the outputs must be current first
        their_state = theirs.state
        d_psi, d_exp = theirs.rhs()
        their_state.psi_s = their_state.psi_s + DT * d_psi
        their_state.exp_j_theta_m = their_state.exp_j_theta_m + DT * d_exp
        their_state.exp_j_theta_m /= abs(their_state.exp_j_theta_m)  # keep it on the unit circle
        i_theirs = theirs.i_s

        magnitude = abs(abs(i_theirs) - abs(i_ours)) / max(abs(i_theirs), 1e-12)
        angle = abs(cmath.phase(i_theirs) - cmath.phase(i_ours))
        worst_magnitude = max(worst_magnitude, magnitude)
        worst_angle = max(worst_angle, angle)
        if k % 100 == 0:
            print(f"step {k:4d}  ours {i_ours:+.6f}  theirs {i_theirs:+.6f}  "
                  f"d|i| {100 * magnitude:.3f}%  dangle {angle:.2e} rad")

    print(f"\nworst magnitude error: {100 * worst_magnitude:.4f} %  (limit 1 %)")
    print(f"worst angle error    : {worst_angle:.3e} rad  (limit 1e-2)")
    ok = (worst_magnitude < 0.01) and (worst_angle < 1e-2)
    print("OK: our model and motulator's agree on this trajectory" if ok
          else "DISAGREE: the two models do not describe the same machine")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
