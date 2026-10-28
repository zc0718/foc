"""motulator's own simulation machinery at our operating point, to close the d-axis question.

`reconcile_controller.py` drives their controller by hand, and its d axis kept a small offset during the ramp phases
that this project could not attribute: their control law, or another artefact of the adapter? Hand-plumbing is exactly
where such an offset can be manufactured, so the question is put to their own code instead --
`model.Drive` + `model.Simulation` + `CurrentVectorControl` -- with a torque command and a free-accelerating machine,
which is the condition where the offset appears. None of this project's adapters is in the path.

    source workspace/materials/.venv/bin/activate
    python benchmark/script/official_cvc_check.py

Two expectations, both closed form and both written before the run:

  * q axis: a constant torque command accelerates the machine at alpha = 1.5*n_p*psi_f*i_q/J, and the current loop's
    finite gain leaves the PI ramp error delta_i = psi_f*alpha/k_i with k_i = alpha_c*R_s. Solving
    i_q = i_cmd - psi_f*(1.5*n_p*psi_f*i_q/J)/(alpha_c*R_s) gives the steady q current -- 0.3245 A here.
  * d axis: if the offset seen through the hand-built adapter were their control law, it must appear here too.

Result: the q axis lands on the closed form to 0.4 % of the command, and the d axis does carry an offset that grows with
speed -- in their machinery, with no code of ours involved. The offset is their current loop's; the adapter, which
showed a tenth of it, was the one under-measuring.

The same numbers come out of motulator 0.19's `model.Simulation` (iq 0.3227 A, id -0.0533 A at the same instant versus
0.3274 A / -0.0537 A here), so this is not a property of the version we happen to pin.
"""

import math
import sys

import numpy as np

from motulator.drive import model
from motulator.drive.control.sm import CurrentReferenceCfg, CurrentVectorControl
from motulator.drive.utils import SynchronousMachinePars

N_P, RS, LD, LQ, PSI_F = 1, 0.5, 4.0e-4, 4.0e-4, 0.0476
J_ROTOR, B_FRICTION = 1.0e-5, 1.0e-6
V_BUS = 300.0
DT = 50e-6
ALPHA_C = 2.0 * math.pi * 200.0
IQ_COMMAND = 0.5
TAU_COMMAND = 1.5 * N_P * PSI_F * IQ_COMMAND
T_STOP = 0.2


def main() -> int:
    par = SynchronousMachinePars(N_P, RS, LD, LQ, PSI_F)
    mdl = model.Drive(
        model.VoltageSourceConverter(u_dc=V_BUS),
        model.SynchronousMachine(par),
        model.StiffMechanicalSystem(J=J_ROTOR, B_L=B_FRICTION),
    )
    cfg = CurrentReferenceCfg(par, max_i_s=5.0, nom_w_m=400.0)
    ctrl = CurrentVectorControl(par, cfg, T_s=DT, J=None, alpha_c=ALPHA_C, sensorless=False)
    ctrl.ref.tau_M = lambda _t: TAU_COMMAND

    model.Simulation(mdl, ctrl).simulate(t_stop=T_STOP)
    i_s = np.asarray(ctrl.data.fbk.i_s)
    w_m = np.asarray(ctrl.data.fbk.w_m)

    # Closed form: constant torque means constant acceleration, and the PI ramp error closes the loop on itself.
    alpha = 1.5 * N_P * PSI_F / J_ROTOR
    predicted_iq = IQ_COMMAND / (1.0 + PSI_F * alpha / (ALPHA_C * RS))

    print(f"motulator's own Drive + Simulation, torque mode, free acceleration over {T_STOP} s ({len(i_s)} samples)")
    print(f"{'speed (rad/s)':>14}  {'id (A)':>9} {'iq (A)':>9}")
    step = max(1, len(i_s) // 6)
    for k in list(range(0, len(i_s), step)) + [len(i_s) - 1]:
        print(f"{w_m[k]:14.2f}  {i_s.real[k]:+9.4f} {i_s.imag[k]:+9.4f}")

    tail = slice(len(i_s) // 2, len(i_s))
    iq_tail = float(np.mean(i_s.imag[tail]))
    id_tail = float(np.mean(i_s.real[tail]))
    id_end = float(i_s.real[-1])

    print()
    print(f"predicted q current : {predicted_iq:+.4f} A")
    print(f"measured  q current : {iq_tail:+.4f} A   ({(iq_tail - predicted_iq) / IQ_COMMAND * 100.0:+.2f} % of "
          f"the command)")
    print(f"measured  d current : {id_tail:+.4f} A mean, {id_end:+.4f} A at the end of the run")
    print(f"misalignment        : {math.atan2(id_tail, iq_tail):+.4f} rad")

    ok = abs(iq_tail - predicted_iq) <= 0.05 * IQ_COMMAND
    print()
    print("OK: their machinery lands on the same closed form, and the d-axis offset turns out to be theirs too"
          if ok else "NOT OK: the q axis disagrees with the closed form in their own machinery")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
