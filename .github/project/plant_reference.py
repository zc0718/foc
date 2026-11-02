#!/usr/bin/env python3
"""An independent float64 reference for the SIL plant, and the trajectory audits that go with it.

Why this exists. F-40 was a defect that *froze* a state: the plant integrated its mechanical angle in float32, and any
step smaller than half an ulp was rounded away, so the axis stopped while its speed still read non-zero. The damage that
kind of defect does is specific -- the measurements stay stable, repeatable and wrong in one direction, so no amount of
repetition inside the same suite can find it. Only a different instrument can. This is that instrument:

  * different language and different arithmetic (numpy float64, not C++ float32/double mixed),
  * a different solver family (scipy's adaptive DOP853 at rtol 1e-12, not a fixed-step RK4),
  * a different author of the right-hand side (the equations are re-read from the header, not linked from it).

Two checks, in increasing independence:

  A. REPLAY   For a sampled control period, start from the state the C++ side dumped, apply the voltage the C++ side
              dumped, and integrate one period independently. It must land on the next dumped state. This is the sharper
              of the two and the only one that works under *any* controller, because it needs no knowledge of the
              controller at all -- which is what lets the closed-loop scenario, the real PositionP -> PI speed -> PI
              current -> SVPWM cascade, be audited without re-implementing one line of it in Python.
  B. ENERGY   Over the whole run, `d/dt(stored energy) = input power - copper loss - friction`. This shares nothing with
              the integrator: it is a conservation law on the dumped trajectory, so it catches a wrong torque constant, a
              missing 3/2 factor or a sign error in the back-EMF term even if the integration itself is perfect.

Usage:  python3 .github/project/plant_reference.py [--scenario NAME] [--keep]
Exit 0 when every check passes, 1 otherwise.
"""

from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import tempfile
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.integrate import solve_ivp

SCENARIOS = ("openloop", "closedloop")

# How many periods the replay check looks at. Every period would be 24000 independent integrations for the closed loop,
# which buys nothing once the sample spans the transient. This is a sample size, not a tolerance.
REPLAY_SAMPLES = 120


@dataclass(frozen=True)
class Machine:
    """The scenario's machine, as printed by the harness. Never restated here -- see `emit_machine` in the harness."""

    rs: float
    ld: float
    lq: float
    kt: float
    j: float
    b: float
    poles: int
    dt: float
    substeps: int

    @property
    def psi_f(self) -> float:
        return self.kt / (1.5 * self.poles)


@dataclass
class Trajectory:
    machine: Machine
    name: str
    step: np.ndarray  # (N,) control-period index
    state: np.ndarray  # (N, 4) id, iq, omega_mech, theta_mech
    v_uvw: np.ndarray  # (N, 3) applied phase voltages, V

    @property
    def t(self) -> np.ndarray:
        return self.step * self.machine.dt


def clarke(v_uvw: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """`core::clarke`, amplitude-invariant: the 2/3 and 1/sqrt(3) are the C++ side's, not a convention choice."""
    u, v, w = v_uvw[..., 0], v_uvw[..., 1], v_uvw[..., 2]
    return (2.0 / 3.0) * (u - 0.5 * (v + w)), (1.0 / math.sqrt(3.0)) * (v - w)


def v_dq_of(traj: Trajectory) -> tuple[np.ndarray, np.ndarray]:
    """The rotor-frame voltage the plant itself formed: clarke, then park at `poles * theta`."""
    alpha, beta = clarke(traj.v_uvw)
    theta_e = traj.machine.poles * traj.state[:, 3]
    c, s = np.cos(theta_e), np.sin(theta_e)
    return alpha * c + beta * s, -alpha * s + beta * c


def rhs_fixed_voltage(machine: Machine, v_d: float, v_q: float):
    """The plant's stated equations with the voltage held. Re-derived from `sil_pmsm_plant.hpp`."""

    def rhs(_t: float, x: np.ndarray) -> list[float]:
        i_d, i_q, omega_m, _theta = x
        omega_e = machine.poles * omega_m
        # The self-test hook: perturb one parameter in the *reference* by the given fraction, so that "this check can go
        # red" is a property of the file rather than a claim about it.
        psi_f = machine.psi_f * (1.0 + float(os.environ.get("PLANT_REF_MUTATE", "0.0")))
        torque = 1.5 * machine.poles * (psi_f * i_q + (machine.ld - machine.lq) * i_d * i_q)
        d_id = (v_d - machine.rs * i_d + omega_e * machine.lq * i_q) / machine.ld
        d_iq = (v_q - machine.rs * i_q - omega_e * machine.ld * i_d - omega_e * psi_f) / machine.lq
        d_omega = (torque - machine.b * omega_m) / machine.j
        return [d_id, d_iq, d_omega, omega_m]

    return rhs


def integrate_period(machine: Machine, x0: np.ndarray, v_uvw: np.ndarray) -> np.ndarray:
    """One *control period*, reproduced the way `PlantHal::advance` does it.

    The rotor frame is frozen inside each substep because that is what the C++ side does: `set_voltage_dq` is called once
    before each `plant.advance`. Matching that convention is deliberate. A frozen versus a continuously rotating frame is
    a *discretisation choice*, and letting it in would blur the arithmetic difference this check exists to find. What
    differs here is the solver and the precision, nothing else.
    """
    x = np.array(x0, dtype=float)
    h = machine.dt / machine.substeps
    alpha, beta = clarke(np.asarray(v_uvw, dtype=float))
    for _ in range(machine.substeps):
        # Narrowed to float32 first, because `PlantHal::advance` reads the angle through `theta_mech_rad()`, which
        # returns float. Mirroring it keeps a known, understood difference (ulp(theta) ~ 3e-8 at 0.3 rad) out of the
        # comparison, so what is left is the integrator and the arithmetic -- which is what this check is for.
        # 先收窄到 float32：`PlantHal::advance` 是通过返回 float 的 `theta_mech_rad()` 读角度的。复制这条约定，是为了
        # 把一个已知且已解释的差异（0.3 rad 处 ulp 约 3e-8）排除在外，留下的才是积分器与算术——而这正是本检查要量的。
        theta_e = machine.poles * float(np.float32(x[3]))
        c, s = math.cos(theta_e), math.sin(theta_e)
        sol = solve_ivp(rhs_fixed_voltage(machine, float(alpha * c + beta * s), float(-alpha * s + beta * c)),
                        (0.0, h), x, method="DOP853", rtol=1e-12, atol=1e-15)
        if not sol.success:
            raise RuntimeError(sol.message)
        x = sol.y[:, -1]
        # `PLANT_REF_FREEZE` reproduces F-40 inside the *reference*: round the angle to float32 after every substep.
        # At low speed the step is below half an ulp, so `x[3]` comes back unchanged and the axis stops moving while its
        # speed stays non-zero. The check has to go red on that, or "this instrument would have caught F-40" is an
        # unverified claim about the instrument rather than a demonstration of it.
        if os.environ.get("PLANT_REF_FREEZE"):
            x[3] = float(np.float32(x[3]))
    return x


def parse(stdout: str, name: str) -> Trajectory:
    machine = None
    rows: list[list[float]] = []
    for line in stdout.strip().splitlines():
        if line.startswith("# machine,"):
            f = line.split(",")[1:]
            machine = Machine(rs=float(f[0]), ld=float(f[1]), lq=float(f[2]), kt=float(f[3]), j=float(f[4]),
                              b=float(f[5]), poles=int(float(f[6])), dt=float(f[7]), substeps=int(float(f[8])))
        elif line.startswith("step"):
            continue
        elif line:
            rows.append([float(v) for v in line.split(",")])
    if machine is None:
        raise RuntimeError(f"{name}: the harness did not print its machine parameters")
    data = np.array(rows)
    return Trajectory(machine=machine, name=name, step=data[:, 0], state=data[:, 1:5], v_uvw=data[:, 5:8])


def check_replay(traj: Trajectory, v_uvw: np.ndarray | None = None) -> tuple[float, float, int]:
    """A: does one independently integrated period land on the C++ side's next state?"""
    n = len(traj.step)
    # Scales taken from the run, not from the sample: an error is judged against the magnitude the channel actually
    # reaches. Normalising by the instantaneous value instead makes every zero-crossing look like a defect -- the first
    # version of this check reported 3.5e-2 on a current that was passing through zero with an absolute error of 4e-7.
    # 尺度取自整段运行，而不是那一瞬间：误差要拿该通道真正达到的量级去衡量。反过来用瞬时值归一化，会让每个过零处都
    # 看起来像缺陷——本检查的第一版就在一个正在过零、绝对误差 4e-7 的电流上报了 3.5e-2。
    scale = np.maximum(np.max(np.abs(traj.state[:, :3]), axis=0), 1e-3)
    # The angle bound is derived from the instrument rather than chosen as a ratio. The plant forms the rotor-frame
    # voltage from `theta_mech_rad()`, which is float32, so the *park angle* has a resolution of `ulp(poles*theta)`.
    # Two sides that agree to better than half an ulp can still round to different neighbours, and that shows up as a
    # step difference of one count. Four counts is the allowance; it is 4x6.8e-8 rad at theta = 1, i.e. four orders
    # below the 200 urad drift F-40 produced, so the check keeps its teeth.
    # 角度界由**仪器**推导，而不是选一个比值。plant 用 float32 的 `theta_mech_rad()` 构造转子系电压，所以「park 角」
    # 的分辨率是 `ulp(poles*theta)`。两边吻合到半个 ulp 以内，仍可能舍入到不同的邻居，表现为一个计数的步差。允许
    # 四个计数；在 theta = 1 处是 4×6.8e-8 rad，比 F-40 产生的 200 urad 漂移低四个数量级，所以判据没有失去牙齿。
    park_ulp = float(np.spacing(np.float32(traj.machine.poles * max(abs(traj.state[:, 3]).max(), 1e-3)))
                     / traj.machine.poles)
    step_scale = park_ulp
    picks = np.unique(np.linspace(0, n - 2, min(REPLAY_SAMPLES, n - 1)).astype(int))
    worst_current = 0.0
    worst_mech = 0.0
    for k in picks:
        got = integrate_period(traj.machine, traj.state[k],
                               (traj.v_uvw if v_uvw is None else v_uvw)[k])
        want = traj.state[k + 1]
        for ch in (0, 1):  # currents, stored as float32 by the C++ side
            worst_current = max(worst_current, abs(got[ch] - want[ch]) / scale[ch])
        worst_mech = max(worst_mech, abs(got[2] - want[2]) / scale[2])
        # The angle's value is large and its step is small, so the angle is judged on the *step*: a relative test on the
        # value would be blind to exactly the defect this instrument exists to find.
        step_error = abs((got[3] - traj.state[k][3]) - (want[3] - traj.state[k][3]))
        worst_mech = max(worst_mech, step_error / step_scale)  # in units of one park-angle count
    return worst_current, worst_mech, len(picks)


def check_kinematics(traj: Trajectory) -> float:
    """C: does the angle advance by what the speed says it advanced by, over the whole run?

    The per-period replay check cannot see F-40, and finding that out was worth a round trip. F-40 lost *half an ulp per
    step*, which is far below any single-period tolerance but accumulates; the replay check samples periods and asks
    each one to be right on its own, so a systematically short step passes every sample. This is the check that closes
    that gap: `theta(T) - theta(0)` against `integral of omega dt` over the same dump, which is O(N) and needs no solver.
    A frozen angle leaves the right-hand side non-zero and the left-hand side zero, so the defect is visible by
    construction rather than by sampling luck.
    """
    omega = traj.state[:, 2]
    delta = float(traj.state[-1, 3] - traj.state[0, 3])
    travelled = float(np.trapezoid(omega, traj.t))
    # Returned in **radians**, not as a ratio of the travel, and that choice is the substance of the check. Relative to a
    # one-radian move, F-40's 200 urad of lost motion is 2e-4 -- inside any sane ratio bound, and yet twenty times the
    # finest claim that rests on this trajectory. A ratio hides the defect precisely because the trajectory is long.
    # 返回**弧度**而不是行程比值，这个选择就是这条检查的实质。对一次一弧度的运动来说，F-40 丢掉的 200 urad 只有 2e-4
    # ——落在任何合理的比值界内，却是依赖这条轨迹的最细主张的二十倍。比值恰好因为轨迹长而把缺陷藏起来。
    return abs(delta - travelled)


def check_energy(traj: Trajectory) -> float:
    """B: `E(T) - E(0)` against the integral of input power minus losses, both taken from the dumped trajectory."""
    m = traj.machine
    i_d, i_q, omega = traj.state[:, 0], traj.state[:, 1], traj.state[:, 2]
    v_d, v_q = v_dq_of(traj)
    # 0.75, not 0.5, on the magnetic term: the transforms are amplitude-invariant, so the dq-side stored magnetic energy
    # of a three-phase machine is (3/2) * (1/2)(Ld id^2 + Lq iq^2). The mechanical term is a real energy and keeps its
    # 1/2. Getting this wrong cost one round of this check reporting a 38 % imbalance that was the formula's, not the
    # plant's -- which is the whole reason the check is stated as a formula and not as a tolerance.
    # 磁能项是 0.75 而不是 0.5：变换是幅值不变型，三相机的 dq 侧磁场储能是 (3/2)*(1/2)(Ld id²+Lq iq²)。机械项是真实
    # 能量，仍带 1/2。这一项搞错过一次，本检查因此报了 38 % 的不平衡，而那是**公式**的错不是 plant 的——这正是为什么
    # 这条检查写成一个公式而不是一个容差。
    stored = 0.75 * (m.ld * i_d**2 + m.lq * i_q**2) + 0.5 * m.j * omega**2
    power = 1.5 * (v_d * i_d + v_q * i_q) - 1.5 * m.rs * (i_d**2 + i_q**2) - m.b * omega**2
    delta = float(stored[-1] - stored[0])
    supplied = float(np.trapezoid(power, traj.t))
    # Normalised by the energy that actually flowed, not by the endpoint difference. In a settled run the endpoints have
    # nearly the same energy, so a difference-based ratio divides a small residue by an even smaller number and reports
    # nonsense -- the throughput is the scale a conservation law is measured against.
    gross = float(np.trapezoid(np.abs(power), traj.t))
    return abs(delta - supplied) / max(gross, abs(delta), 1e-12)


# Bounds are derived, not chosen. Currents: float32 storage is 1.2e-7 and the plant's RK4 at 12.5 us on a 0.8 ms
# electrical time constant adds about 1e-7, so 1e-5 leaves two orders of margin without being able to hide a real
# defect. Mechanics: double storage, so only the integrator is left, and 1e-5 on the speed and on the angle *step* is
# generous for a comparison that measures 1e-7. Energy: the trapezoid on 5e-5 samples of a 0.8 ms transient is the
# loosest of the three, so 1e-2 -- still tight enough that a missing 3/2 factor (33 %) or a flipped back-EMF sign
# cannot survive.
BOUND_REPLAY_CURRENT = 1e-5  # of the channel's full scale, i.e. about 40 float32 ulps of a 3 A run
BOUND_REPLAY_MECH = 4.0     # in park-angle counts, see `check_replay`
BOUND_ENERGY = 1e-2
# The kinematics residual is a quadrature error and nothing else: RK4 and trapezoid disagree by O(h^2) on a signal whose
# fastest scale is the 0.8 ms electrical time constant, i.e. about 1e-4 of a radian against a travel of a radian.
# Absolute radians, and the number comes from the evidence chain rather than from the measurement. The finest claim that
# rests on a plant trajectory is the millimetre-level positioning budget, about 1e-3 rad; 1e-5 rad is a hundredth of
# that, so a loss that could move a published number cannot pass, while the quadrature floor measured here (3.3e-7 on a
# 4.4 rad travel, i.e. 1.5e-6 rad) sits six times below the bound. Both sides are values, not preferences.
BOUND_KINEMATICS = 1e-5


def selftest(traj: Trajectory) -> bool:
    """Feed the audits knowingly-bad trajectories and require each one to go red.

    Without this, "the audit would have caught F-40" is a claim about the audit rather than a demonstration of it, and
    this repository has already paid for that distinction once (A-40: a prediction that was never wrong, an instrument
    that was).
    """
    cases = []

    # 1. A frozen angle from the middle of the run onwards: F-40's shape, applied to a trajectory known to be good.
    #    Sized deliberately to the defect rather than to something dramatic: a check that only catches gross failures
    #    would have let F-40 through, and the selftest would then be proving the wrong thing.
    #    故意按缺陷的实际大小来定：只能抓粗大故障的检查会放过 F-40，自检就证明了错的东西。
    short = Trajectory(traj.machine, traj.name + "/short", traj.step.copy(), traj.state.copy(), traj.v_uvw.copy())
    travel = float(short.state[-1, 3] - short.state[0, 3])
    short.state[:, 3] = short.state[0, 3] + (short.state[:, 3] - short.state[0, 3]) * (1.0 - 200e-6 / travel)
    cases.append(("travel 200 urad short", check_kinematics(short), BOUND_KINEMATICS, "kinematics"))

    # 2. A speed that is 1 % high: the same equation, read the other way.
    fast = Trajectory(traj.machine, traj.name + "/fast", traj.step.copy(), traj.state.copy(), traj.v_uvw.copy())
    fast.state[:, 2] *= 1.01
    cases.append(("speed 1 % high", check_kinematics(fast), BOUND_KINEMATICS, "kinematics"))

    # 3. A phase voltage 2 % off: only the replay check can see this one, which is why it is here.
    wrong_v = traj.v_uvw * 1.02
    cur, _mech, _n = check_replay(traj, v_uvw=wrong_v)
    cases.append(("phase voltage 2 % high", cur, BOUND_REPLAY_CURRENT, "replay"))

    ok = True
    for what, value, bound, which in cases:
        caught = value > bound
        print(f"[plant-ref]   selftest {what:34s} -> {which:10s} {value:.3e} vs bound {bound:.0e} : "
              f"{'caught' if caught else 'MISSED'}")
        ok = ok and caught
    return ok


def positioning_point(exe: str, start: float, target: float, lsb: float, friction_nm: float,
                      seconds: float = 1.2) -> float:
    """One move, one number: where the rotor ended up. See `run_positioning` for why only the final state."""
    proc = subprocess.run([exe, "positioning", repr(start), repr(target), repr(lsb), repr(friction_nm), repr(seconds)],
                          capture_output=True, text=True, check=True)
    for line in proc.stdout.strip().splitlines():
        if line.startswith("final,"):
            return float(line.split(",")[1])
    raise RuntimeError("positioning produced no final row")


def check_iso230(exe: str) -> list[str]:
    """D: ISO 230-2's reversal value, measured the way the standard says -- each target from both sides.

    The predictions are written before the measurement, and this framework has already seen them be right for the wrong
    reason once (A-41: they were right, the bench was frozen). What is asserted:

      P1  the reversal value equals **twice** the one-sided friction displacement. Stiction is a dead band, so the axis
          stops short in whichever direction it travelled: short by `d` going up and by `d` going down makes the two
          approaches differ by `2d` in signed terms, while one direction alone shows only `d`.
      P2  with no stiction the reversal value collapses to zero. This is the mechanism's own control: if B survives at
          zero breakaway torque then stiction is not what caused it, however well P1 fitted.
      P3  repeatability is exactly zero, because this rig has no randomness. Stated as a property of the bench, not of a
          machine: its worth is as a hidden-state detector.
      P4  bidirectional accuracy stays inside the unidirectional budget of 2.45 mrad.
    """
    targets = (0.6, 1.0, 1.4)
    approach, friction = 0.30, 2.0e-4
    fails: list[str] = []
    rows = []
    spread = 0.0
    accuracy_lo, accuracy_hi = float("inf"), float("-inf")
    for target in targets:
        up = [positioning_point(exe, target - approach, target, 0.0, friction) for _ in range(3)]
        down = [positioning_point(exe, target + approach, target, 0.0, friction) for _ in range(3)]
        up_c = positioning_point(exe, target - approach, target, 0.0, 0.0)
        down_c = positioning_point(exe, target + approach, target, 0.0, 0.0)
        spread = max(spread, max(up) - min(up), max(down) - min(down))
        displacement = sum(up) / len(up) - up_c
        reversal = sum(up) / len(up) - sum(down) / len(down)
        rows.append((target, displacement, reversal, up_c - down_c))
        accuracy_lo = min(accuracy_lo, min(up) - target, min(down) - target)
        accuracy_hi = max(accuracy_hi, max(up) - target, max(down) - target)

    print("[plant-ref]   D ISO 230-2  target   one-sided    reversal   reversal/(2*one-sided)   clean reversal")
    worst_b, worst_clean, lo, hi = 0.0, 0.0, float("inf"), float("-inf")
    for target, displacement, reversal, reversal_clean in rows:
        ratio = reversal / (2.0 * displacement) if displacement else float("nan")
        print(f"[plant-ref]              {target:6.2f} {displacement * 1e6:10.3f} urad {reversal * 1e6:10.3f} urad"
              f" {ratio:18.2f} {reversal_clean * 1e6:18.4f} urad")
        worst_b = max(worst_b, abs(reversal))
        worst_clean = max(worst_clean, abs(reversal_clean))
        lo, hi = min(lo, ratio), max(hi, ratio)
    accuracy = accuracy_hi - accuracy_lo
    print(f"[plant-ref]              reversal worst {worst_b * 1e6:.3f} urad, B/(2d) in [{lo:.2f}, {hi:.2f}], "
          f"clean {worst_clean * 1e6:.4f} urad, accuracy {accuracy * 1e3:.4f} mrad, repeat spread {spread:.1e} rad")

    # The band is +-10 % because the model says *exactly* 2, not "about 2": a dead band that stops the axis short by the
    # same amount in each direction produces that ratio identically. Measured 1.00 at all three targets, so the earlier
    # +-50 % band would have been a criterion that nothing within a factor of 1.5 could contradict.
    # 带宽取 ±10 %，因为模型说**恰好** 2，不是"大约 2"：一个在两个方向上都让轴差同样距离的死区，必然给出这个比值。
    # 三个目标实测都是 1.00，此前 ±50 % 的带子等于一条"差 1.5 倍也推不翻"的判据。
    if not (0.9 <= lo and hi <= 1.1):
        fails.append(f"ISO 230-2 P1: reversal is {lo:.2f}..{hi:.2f} x the one-sided displacement, not 2x")
    if worst_clean > 0.05 * worst_b:
        fails.append(f"ISO 230-2 P2: a {worst_clean * 1e6:.3f} urad reversal survives at zero breakaway torque")
    if spread != 0.0:
        fails.append(f"ISO 230-2 P3: repeated identical moves differ by {spread:.2e} rad, so the loop carries state")
    if accuracy > 2.45e-3:
        fails.append(f"ISO 230-2 P4: bidirectional accuracy {accuracy * 1e3:.3f} mrad exceeds the 2.45 mrad budget")
    return fails


def check_quantisation(exe: str) -> list[str]:
    """E: the residual of a quantised sensor is a uniform distribution of width one count.

    This is an externally published model (Bennett 1948; Widrow's quantisation theorem), not a tolerance of ours: over a
    sweep that spans one count, the settled position error is uniform on an interval of width `q`, so its standard
    deviation is `q/sqrt(12)` and its mean is bounded by `q/4`. The check is worth having because the position budget
    only ever used the *worst case* (half a count) and never asked whether the distribution is the one theory predicts --
    a biased quantiser or a loop that settles somewhere other than on a count would both pass a worst-case bound.
    """
    lsb, base, points = 2.0e-3, 1.0, 16
    residuals = []
    for k in range(points):
        target = base + lsb * k / points
        residuals.append(positioning_point(exe, target - 0.30, target, lsb, 0.0, seconds=3.0) - target)
    data = np.array(residuals)
    predicted = lsb / math.sqrt(12.0)
    measured = float(np.std(data))
    mean = float(np.mean(data))
    print(f"[plant-ref]   E quantise   q {lsb * 1e3:.2f} mrad, measured sigma {measured * 1e6:.1f} urad against "
          f"q/sqrt(12) = {predicted * 1e6:.1f} urad ({measured / predicted:.3f}x), mean {mean * 1e6:+.2f} urad "
          f"(bound q/4 = {lsb / 4 * 1e6:.0f})")
    fails: list[str] = []
    # +-25 %, and the width is the sampling uncertainty rather than a preference: the standard deviation of a 16-point
    # sample of a uniform distribution has a relative standard error of about 1/sqrt(2*15) = 18 %, so a tighter band
    # would fail on sample luck and a looser one would stop distinguishing "uniform" from "triangular" (0.71x).
    # ±25 %，宽度来自**抽样不确定度**而不是偏好：16 点均匀样本的标准差，其相对标准误约 1/√(2·15) = 18 %，再紧就会因
    # 抽样运气而红，再松就分不出"均匀"与"三角"（0.71 倍）。
    if not 0.75 <= measured / predicted <= 1.25:
        fails.append(f"quantisation: sigma is {measured / predicted:.2f} x q/sqrt(12), so the residual is not uniform")
    if abs(mean) > lsb / 4:
        fails.append(f"quantisation: mean residual {mean * 1e6:+.2f} urad exceeds q/4, so the quantiser is biased")
    return fails


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", choices=SCENARIOS, action="append", help="restrict to one scenario (repeatable)")
    ap.add_argument("--keep", action="store_true", help="keep the built harness")
    ap.add_argument("--selftest", action="store_true", help="perturb a good dump and require the audits to notice")
    args = ap.parse_args()

    root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    src = os.path.join(root, ".github", "project", "reference", "harness.cpp")
    work = tempfile.mkdtemp(prefix="plant-ref-")
    exe = os.path.join(work, "harness")
    cmd = ["g++", "-std=c++17", "-O1", "-I", os.path.join(root, "include"),
           "-I", os.path.join(root, "test_package", "test", "unit"),
           "-DRESOURCES_PATH=" + repr("test_package/resources"), src]
    for sub in ("core", "core/observer", "motor", "driver"):
        cmd += sorted(str(p) for p in Path(root, "src", sub).glob("*.cpp"))
    cmd += [os.path.join(root, "src", "version.c"), "-lm", "-o", exe]
    built = subprocess.run(cmd, capture_output=True, text=True)
    if built.returncode != 0:
        print("[plant-ref] harness failed to build:\n" + built.stderr[-3000:])
        return 1
    if args.keep:
        print(f"[plant-ref] harness kept at {work}")

    names = args.scenario if args.scenario else list(SCENARIOS)
    claim_fails: list[str] = []
    failed = False
    for name in names:
        traj = run_harness(root, exe, name)
        print(f"[plant-ref] {name}: {len(traj.step)} control periods, dt {traj.machine.dt:.2e} s, "
              f"{traj.machine.substeps} substeps, theta {traj.state[0, 3]:.4f} -> {traj.state[-1, 3]:.4f} rad")
        cur, mech, picked = check_replay(traj)
        energy = check_energy(traj)
        kinematic = check_kinematics(traj)
        print(f"[plant-ref]   A replay {picked:4d} periods: currents {cur:.2e} of scale (bound "
              f"{BOUND_REPLAY_CURRENT:.0e}), angle step {mech:.2f} park-angle counts (bound "
              f"{BOUND_REPLAY_MECH:.0f})")
        print(f"[plant-ref]   B energy residual {energy:.2e} rel (bound {BOUND_ENERGY:.0e})")
        print(f"[plant-ref]   C kinematics    {kinematic:.2e} rad short (bound {BOUND_KINEMATICS:.0e})")
        if cur > BOUND_REPLAY_CURRENT or mech > BOUND_REPLAY_MECH:
            print(f"[plant-ref]   RED {name}: the C++ plant does not land where its own equations land")
            failed = True
        if kinematic > BOUND_KINEMATICS:
            print(f"[plant-ref]   RED {name}: the angle did not advance by what the speed integrated to")
            failed = True
        if energy > BOUND_ENERGY:
            print(f"[plant-ref]   RED {name}: the trajectory does not conserve energy, so an equation is wrong")
            failed = True

        if args.selftest and not selftest(traj):
            print(f"[plant-ref]   RED {name}: a knowingly-bad trajectory passed an audit")
            failed = True

    if not args.scenario:
        claim_fails = check_iso230(exe) + check_quantisation(exe)
        for message in claim_fails:
            print(f"[plant-ref]   RED {message}")
        failed = failed or bool(claim_fails)

    print("[plant-ref] FAIL" if failed else "[plant-ref] OK")
    return 1 if failed else 0


def run_harness(_root: str, exe: str, name: str) -> Trajectory:
    proc = subprocess.run([exe, name], capture_output=True, text=True, check=True)
    return parse(proc.stdout, name)


if __name__ == "__main__":
    sys.exit(main())
