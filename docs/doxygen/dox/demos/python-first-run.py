"""One motor, one speed loop, no hardware -- the Python door.

`foc_python` is built on the C ABI, and the C ABI is built on the same C++ object the
`c-and-cpp-callers` page measures against. So this script is not a second implementation: it
reaches the same loop, and the only thing that differs is who owns the plant. Here the plant is
this file, written in Python, and the library never sees anything else.

Run it against a built module:

    PYTHONPATH=<dir with foc_python*.so> python3 python-first-run.py

The binding takes an object, not an interface: it looks up `read_current`, `angle_mech_rad`,
`velocity_rad_s`, `voltage_v` and `write_duty` on whatever you pass as `plant`. That is what makes
a Python model usable here without a wrapper -- and it is also why the names in the block below
matter.
"""

import math

# The nameplate, SI units. Salient on purpose: `lq > ld` is the shape MTPA and the observer exist for.
RS = 0.5
LD = 4.0e-4
LQ = 8.0e-4
KT = 0.01
J = 1.0e-4
B = 1.0e-4
POLE_PAIRS = 7
V_BUS = 24.0
DT = 50e-6
SUBSTEPS = 20


class Plant:
    """A dq-frame PMSM, integrated in Python. A stand-in, and the page says so.

    The library owns the loop; this owns the physics: the winding's own L/R, the magnet's flux, the
    inertia. Both halves are needed for a closed loop, and keeping them in separate files is what
    makes "the controller is the same one the C page runs" an observable statement rather than a
    claim.
    """

    def __init__(self) -> None:
        self.id_a = 0.0
        self.iq_a = 0.0
        self.omega_mech = 0.0
        self.theta_mech = 0.0
        self.duty = (0.5, 0.5, 0.5)  # 50 % on all three phases is zero volts

    # --- what the binding reads back ---
    def read_current(self) -> tuple[float, float, float]:
        theta_e = POLE_PAIRS * self.theta_mech
        cos_t, sin_t = math.cos(theta_e), math.sin(theta_e)
        alpha = self.id_a * cos_t - self.iq_a * sin_t
        beta = self.id_a * sin_t + self.iq_a * cos_t
        u = alpha
        v = -0.5 * alpha + (math.sqrt(3.0) / 2.0) * beta
        return u, v, -u - v

    def angle_mech_rad(self) -> float:
        return self.theta_mech

    def velocity_rad_s(self) -> float:
        return self.omega_mech

    def voltage_v(self) -> float:
        return V_BUS

    # --- what the binding writes, and where the physics starts ---
    def write_duty(self, du: float, dv: float, dw: float) -> None:
        self.duty = (du, dv, dw)
        self._advance(DT)

    def _advance(self, interval_s: float) -> None:
        v_u, v_v, v_w = ((d - 0.5) * V_BUS for d in self.duty)
        alpha = (2.0 / 3.0) * (v_u - 0.5 * v_v - 0.5 * v_w)
        beta = (2.0 / 3.0) * (math.sqrt(3.0) / 2.0) * (v_v - v_w)
        theta_e = POLE_PAIRS * self.theta_mech
        cos_t, sin_t = math.cos(theta_e), math.sin(theta_e)
        v_d = alpha * cos_t + beta * sin_t
        v_q = -alpha * sin_t + beta * cos_t
        psi_f = KT / (1.5 * POLE_PAIRS)
        h = interval_s / SUBSTEPS
        for _ in range(SUBSTEPS):
            omega_e = POLE_PAIRS * self.omega_mech
            d_id = (v_d - RS * self.id_a + omega_e * LQ * self.iq_a) / LD
            d_iq = (v_q - RS * self.iq_a - omega_e * LD * self.id_a - omega_e * psi_f) / LQ
            torque = 1.5 * POLE_PAIRS * (psi_f * self.iq_a + (LD - LQ) * self.id_a * self.iq_a)
            d_omega = (torque - B * self.omega_mech) / J
            self.id_a += d_id * h
            self.iq_a += d_iq * h
            self.omega_mech += d_omega * h
            self.theta_mech += self.omega_mech * h


def main() -> int:
    import foc_python as foc

    plant = Plant()
    params = foc.MotorParams(RS, LD, LQ, KT, J, B, POLE_PAIRS)
    motor = foc.Motor(params, plant)  # the plant object is duck-typed, not wrapped

    # The whole life cycle, in the order the wrapper allows it: enable, command, tick, read.
    motor.enable()
    motor.set_speed(50.0)  # mechanical rad/s
    for _ in range(8000):
        motor.tick(DT)

    telemetry = motor.telemetry()  # {speed_mech_rad_s, id_a, iq_a} of the last step
    speed = telemetry["speed_mech_rad_s"]
    print(f"speed {speed:8.3f} rad/s   id {telemetry['id_a']:7.3f} A   iq {telemetry['iq_a']:7.3f} A")
    print(motor.status())  # mode, sticky error, alignment result

    # A script that prints numbers is a demo; one that refuses a wrong answer is evidence. The
    # band is 5 % of the command, wide enough to survive the discrete loop and narrow enough that a
    # sign error or a swapped dq axis cannot pass through it.
    if abs(speed - 50.0) > 0.05 * 50.0:
        print(f"the loop did not reach the command: {speed:.3f} vs 50.0 rad/s")
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
