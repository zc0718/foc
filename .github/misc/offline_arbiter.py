#!/usr/bin/env python3
"""L2 offline arbiter: an independent implementation of the SIL closed loop that emits the
reference traces the GTest suite asserts against (PRD 8.5 / G5).

Why it exists
-------------
The C++ core and this script must not share a bug, so the two paths differ on purpose:

  * integrator   C++ hand-written RK4 with a 4-substep zero-order hold;
                 here scipy.integrate.solve_ivp (RK45, tight tolerances);
  * voltage      C++ re-projects the held stationary vector every sub-step;
                 here the stationary vector is a continuous function of the rotor angle handed to
                 the integrator;
  * controller   the *same* discrete PI law, because that is the contract under test - forward
                 Euler integral with clamps, output clamped per axis at v_bus/2.

If the two disagree by more than 2 % (normalised RMSE) the C++ side is wrong until proven
otherwise; the threshold lives in doc/PRD 8.5.

Usage
-----
    python3 .github/misc/offline_arbiter.py            # regenerate every ref_*.csv
    python3 .github/misc/offline_arbiter.py --check    # fail if the committed CSV is stale

The output lands in test_package/resources/ (flat, covered by export_sources = "resources/*").

Requirements: numpy 2.5.4, scipy 1.18.1 (dev-time only, never a Conan dependency).
"""
import argparse
import math
import sys
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

ROOT = Path(__file__).resolve().parents[2]
RESOURCES = ROOT / "test_package" / "resources"

# --- machine, controller and bench constants, mirrored from test_package/test/unit/sil_bench.hpp --
RS = 0.5
LD = 4e-4
LQ = 4e-4
KT = 0.05
J = 1e-4
B = 1e-4
POLE_PAIRS = 7
I_MAX = 3.0
V_BUS = 24.0
DT = 5e-5
OMEGA_CURRENT = 2000.0
OMEGA_SPEED = 250.0

PSI_F = KT / (1.5 * POLE_PAIRS)


def current_gains():
    """Critically damped R-L tracking: ki = wn^2 L, kp = 2 wn L - R."""
    kp = max(2.0 * OMEGA_CURRENT * LD - RS, 0.0)
    ki = OMEGA_CURRENT * OMEGA_CURRENT * LD
    lim = 0.5 * V_BUS
    return kp, ki, lim, 0.5 * V_BUS


def speed_gains():
    """Pole-zero cancellation of B/J plus the anti-windup integral clamp of the bench."""
    kp = OMEGA_SPEED * J / KT
    ki = OMEGA_SPEED * B / KT
    return kp, ki, I_MAX, 0.25 * I_MAX


class Pi:
    """Mirror of foc::core::Pi: forward Euler integral, then both clamps."""

    def __init__(self, kp, ki, out_lim, integral_lim):
        self.kp = kp
        self.ki = ki
        self.out_lim = out_lim
        self.integral_lim = integral_lim
        self.integral = 0.0
        self.output = 0.0

    def step(self, setpoint, feedback, dt):
        error = setpoint - feedback
        self.integral += self.ki * error * dt
        self.integral = min(max(self.integral, -self.integral_lim), self.integral_lim)
        raw = self.kp * error + self.integral
        self.output = min(max(raw, -self.out_lim), self.out_lim)
        return self.output


def svpwm(v_ab, v_bus):
    """Midpoint (min-max) injection followed by the [0, 1] clamp, exactly like foc::core::Svpwm."""
    alpha, beta = v_ab
    v_a = alpha
    v_b = -0.5 * alpha + (math.sqrt(3.0) / 2.0) * beta
    v_c = -0.5 * alpha - (math.sqrt(3.0) / 2.0) * beta
    mid = 0.5 * (max(v_a, v_b, v_c) + min(v_a, v_b, v_c))
    return [min(max(0.5 + (v - mid) / v_bus, 0.0), 1.0) for v in (v_a, v_b, v_c)]


def clarke(v_uvw):
    a, b, c = v_uvw
    return (2.0 / 3.0) * (a - 0.5 * b - 0.5 * c), (b - c) / math.sqrt(3.0)


def inv_clarke(v_ab):
    alpha, beta = v_ab
    return (alpha, -0.5 * alpha + (math.sqrt(3.0) / 2.0) * beta, -0.5 * alpha - (math.sqrt(3.0) / 2.0) * beta)


def inv_park(v_dq, theta):
    d, q = v_dq
    c, s = math.cos(theta), math.sin(theta)
    return d * c - q * s, d * s + q * c


def park(v_ab, theta):
    alpha, beta = v_ab
    c, s = math.cos(theta), math.sin(theta)
    return alpha * c + beta * s, -alpha * s + beta * c


def plant_rhs(t, y, v_ab, theta_e_of):
    """dq currents, mechanical speed and angle with a stationary-frame voltage vector."""
    id_a, iq_a, omega, theta = y
    theta_e = theta_e_of(theta)
    v_d, v_q = park(v_ab, theta_e)
    did = (v_d - RS * id_a + theta_e * LQ * iq_a) / LD
    diq = (v_q - RS * iq_a - theta_e * LD * id_a - theta_e * PSI_F) / LQ
    torque = 1.5 * POLE_PAIRS * (PSI_F * iq_a + (LD - LQ) * id_a * iq_a)
    domega = (torque - B * omega) / J
    return [did, diq, domega, omega]


def simulate_speed_step(target_rad_s, seconds):
    """S2: hold 0 rad/s for 2 ms, then step to the target. Returns rows of (t, speed, id, iq)."""
    current_d = Pi(*current_gains())
    current_q = Pi(*current_gains())
    speed = Pi(*speed_gains())

    theta_e_of = lambda theta: POLE_PAIRS * theta  # noqa: E731 - keep the RHS next to its use
    y = [0.0, 0.0, 0.0, 0.0]

    steps = int(round(seconds / DT))
    warmup = int(round(2e-3 / DT))
    rows = []
    t = 0.0
    for k in range(steps):
        id_a, iq_a, omega, theta = y
        reference = 0.0 if k < warmup else target_rad_s

        iq_ref = speed.step(reference, omega, DT)
        v_d = current_d.step(0.0, id_a, DT)
        v_q = current_q.step(iq_ref, iq_a, DT)

        v_ab_command = inv_park((v_d, v_q), theta_e_of(theta))
        duty = svpwm(v_ab_command, V_BUS)
        v_ab_applied = clarke([d * V_BUS for d in duty])

        solution = solve_ivp(
            plant_rhs,
            (t, t + DT),
            y,
            args=(v_ab_applied, theta_e_of),
            method="RK45",
            rtol=1e-9,
            atol=1e-12,
        )
        y = list(solution.y[:, -1])
        t += DT
        rows.append((t, y[2], id_a, iq_a))
    return rows


# Every fifth control period is stored: the loop time constants are 0.5 ms and 4 ms, so 0.25 ms
# sampling loses nothing while keeping the committed trace small. The GTest compares every fifth
# sample of its own trace, so both sides stay index-aligned.
DECIMATION = 5


def write_csv(path, rows):
    lines = ["t_s,speed_rad_s,id_a,iq_a"]
    for t, omega, id_a, iq_a in rows[DECIMATION - 1 :: DECIMATION]:
        lines.append(f"{t:.9g},{omega:.9g},{id_a:.9g},{iq_a:.9g}")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


SCENARIOS = {
    "ref_s2_speed.csv": lambda: simulate_speed_step(100.0, 0.5),
}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true", help="verify the committed CSV is current")
    args = parser.parse_args()

    RESOURCES.mkdir(parents=True, exist_ok=True)
    status = 0
    for name, build in SCENARIOS.items():
        path = RESOURCES / name
        before = path.read_text(encoding="utf-8") if path.exists() else ""
        rows = build()
        write_csv(path, rows)
        after = path.read_text(encoding="utf-8")
        changed = before != after
        print(f"[arbiter] {name}: {len(rows)} samples, {'updated' if changed else 'unchanged'}")
        if args.check and changed:
            print(f"[arbiter] {name}: stale, rerun without --check", file=sys.stderr)
            status = 1
    return status


if __name__ == "__main__":
    sys.exit(main())
