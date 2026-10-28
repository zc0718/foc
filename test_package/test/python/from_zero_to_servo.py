"""From zero to a running servo, in one file -- and this file is executed by the test suite.

The quick-start page includes this source verbatim, so there is exactly one copy of the example: if it stops working,
a test fails; if the API changes, the page cannot keep showing the old one. Everything below uses only the public
Python module, and every limit it meets on the way is one of the documented boundaries.

    python from_zero_to_servo.py     # runs the step and prints what the axis did

从零到一个能跑的伺服，一个文件——而这个文件由测试套件执行。

快速上手页原样包含这份源码，于是示例只有一份：它若失效，测试会失败；API 若改变，页面也不
可能继续展示旧的。下面全部只用公开的 Python 模块，途中遇到的每一条限制都在文档的边界页里。
"""

from __future__ import annotations

import math

import foc_python as foc

# The machine: a small salient PMSM. Every number is SI, as the library's params struct requires.
# 电机：一台小型凸极 PMSM。全部为 SI 单位，与库的 params 结构所需一致。
RS = 0.35
LD = 2.0e-4
LQ = 4.0e-4
KT = 0.05
J = 1.0e-4
B = 1.0e-3
POLE_PAIRS = 7

V_BUS = 24.0
CONTROL_HZ = 20000.0
DT = 1.0 / CONTROL_HZ
SUBSTEPS = 8


def _clarke(du: float, dv: float, dw: float) -> tuple[float, float]:
    """Phase to stationary two-axis, the same convention the library uses internally."""
    return du, (du + 2.0 * dv) / math.sqrt(3.0)


def _inverse_clarke(alpha: float, beta: float) -> tuple[float, float, float]:
    return alpha, -0.5 * alpha + 0.5 * math.sqrt(3.0) * beta, -0.5 * alpha - 0.5 * math.sqrt(3.0) * beta


class PmsmPlant:
    """The machine behind the HAL callback protocol.

    The library writes a duty cycle and reads currents, an angle and a speed. Nothing here knows about dq control: it is
    a physical model plus the three numbers a drive would measure.

    库通过 HAL 回调协议驱动它：写入占空比，读回电流、角度与转速。这里没有任何 dq 控制：它是物理模型加上一台真实
    驱动器会测到的三个量。
    """

    def __init__(self) -> None:
        self.theta_mech = 0.0
        self.omega_mech = 0.0
        self.id = 0.0
        self.iq = 0.0
        self.duty = (0.5, 0.5, 0.5)  # 50 % is zero volts on all three phases

    # --- the HAL reads: names the binding looks up on this object ---
    def read_current(self) -> tuple[float, float, float]:
        theta_e = POLE_PAIRS * self.theta_mech
        cos_t, sin_t = math.cos(theta_e), math.sin(theta_e)
        alpha = self.id * cos_t - self.iq * sin_t
        beta = self.id * sin_t + self.iq * cos_t
        return _inverse_clarke(alpha, beta)

    def angle_mech_rad(self) -> float:
        return self.theta_mech

    def velocity_rad_s(self) -> float:
        return self.omega_mech

    def voltage_v(self) -> float:
        return V_BUS

    # --- the HAL writes -------------------------------------------------
    def write_duty(self, du: float, dv: float, dw: float) -> None:
        self.duty = (du, dv, dw)
        self._advance(DT)

    # --- the physics ----------------------------------------------------
    def _advance(self, interval_s: float) -> None:
        """Fixed-step integration of the dq model, `SUBSTEPS` times per control period."""
        v_u, v_v, v_w = ((d - 0.5) * V_BUS for d in self.duty)
        v_alpha, v_beta = _clarke(v_u, v_v, v_w)
        theta_e = POLE_PAIRS * self.theta_mech
        cos_t, sin_t = math.cos(theta_e), math.sin(theta_e)
        v_d = v_alpha * cos_t + v_beta * sin_t
        v_q = -v_alpha * sin_t + v_beta * cos_t
        psi_f = KT / (1.5 * POLE_PAIRS)
        h = interval_s / SUBSTEPS
        for _ in range(SUBSTEPS):
            omega_e = POLE_PAIRS * self.omega_mech
            d_id = (v_d - RS * self.id + omega_e * LQ * self.iq) / LD
            d_iq = (v_q - RS * self.iq - omega_e * LD * self.id - omega_e * psi_f) / LQ
            torque = 1.5 * POLE_PAIRS * (psi_f * self.iq + (LD - LQ) * self.id * self.iq)
            d_omega = (torque - B * self.omega_mech) / J
            self.id += d_id * h
            self.iq += d_iq * h
            self.omega_mech += d_omega * h
            self.theta_mech += self.omega_mech * h


def run_speed_step(command_rad_s: float, seconds: float = 0.4) -> dict[str, float]:
    """Close the library's own speed loop on the plant above and step the speed.

    Why the plant reports a velocity as well as an angle: with an angle but no speed the library would take the speed
    from an observer, and the observer would be starting from zero -- a different example (the sensorless one), with its
    own boundaries. This is the sensored path.

    为什么这里既报角度也报转速：只给角度的话，库会从观测器取转速，而观测器是从零起步的——那是另一个例子（无感的那
    一个），有它自己的边界。这里是带传感器的路径。
    """
    params = foc.MotorParams()
    params.rs, params.ld, params.lq = RS, LD, LQ
    params.kt, params.j, params.b = KT, J, B
    params.pole_pairs = POLE_PAIRS

    plant = PmsmPlant()
    peak = 0.0
    worst_settled = 0.0
    ticks = int(seconds / DT)
    settle_from = ticks - int(0.1 / DT)
    with foc.Motor(params, plant) as motor:
        motor.enable()
        motor.set_speed(command_rad_s)
        for tick in range(ticks):
            motor.tick(DT)
            peak = max(peak, plant.omega_mech)
            if tick >= settle_from:
                worst_settled = max(worst_settled, abs(plant.omega_mech - command_rad_s))
        telemetry = motor.telemetry()
        status = motor.status()
    return {
        "command_rad_s": command_rad_s,
        "finished_rad_s": plant.omega_mech,
        "peak_rad_s": peak,
        "worst_settled_rad_s": worst_settled,
        "iq_a": telemetry["iq_a"],
        "observer": status["observer"].name,
        "observer_locked": bool(status["observer_locked"]),
    }


def test_the_example_settles_a_speed_step() -> None:
    # The bands are measured, not wished for. On this machine and this tuning the axis lands 0.007 rad/s short of a
    # 20 rad/s step, peaks 1.3 % high, and never leaves a 0.14 rad/s band over the last 100 ms. The residual is
    # mechanical, not control: with a rotor this light a few milliamps of current ripple is a fraction of a percent of
    # speed, which is what the boundaries page says about the low-speed ripple in the sensorless case as well.
    # A change that breaks these is a change in the library's behaviour and has to be looked at, which is the whole
    # reason the page includes this file instead of a copy of it.
    # 这些带子是量出来的、不是许愿的。在这台机器与这套整定下，轴比 20 rad/s 阶跃少 0.007 rad/s、峰值高 1.3%，最后 100 ms
    # 内不离开 0.14 rad/s 的带子。残余是机械的、不是控制的：转子这么轻时，几毫安的电流纹波就是零点几个百分点的转速——这与
    # 边界页关于无感低速纹波的说法是同一件事。破坏它们的改动都是库行为的改变、必须被查看——这正是页面包含这个文件而不是
    # 副本的全部理由。
    result = run_speed_step(command_rad_s=20.0)
    assert abs(result["finished_rad_s"] - 20.0) < 0.05, result
    assert result["worst_settled_rad_s"] < 0.20, result
    assert result["peak_rad_s"] < 20.6, result
    assert result["observer"] == "BACK_EMF", result
    assert result["observer_locked"] is True, result


if __name__ == "__main__":
    outcome = run_speed_step(command_rad_s=20.0)
    print(f"commanded {outcome['command_rad_s']:.1f} rad/s, finished at {outcome['finished_rad_s']:.3f} rad/s")
    print(f"  peak {outcome['peak_rad_s']:.3f} rad/s, worst settled error {outcome['worst_settled_rad_s']:.4f} rad/s")
    print(f"  q-axis current {outcome['iq_a']:.3f} A, observer {outcome['observer']}")
