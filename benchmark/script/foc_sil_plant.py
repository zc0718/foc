"""The dq plant the commissioning scripts drive: one copy, so two tools cannot disagree about the machine.

This is host-side laboratory code. It exists so the scripts under `benchmark/script/` can measure the library's loops and
estimators against a machine whose parameters are known exactly -- which is the only way any of their answers are
checkable. Nothing here ships.

主机侧的实验室代码。它存在的理由是让 `benchmark/script/` 下的脚本能对着参数完全已知的电机去测库的环与估计器——这是它们
的答案可被检查的唯一前提。这里的东西不随库发布。
"""

import math

DT = 50e-6


class PmsmPlant:
    """dq-axis PMSM, integrated forward Euler once per actuation, behind the host-HAL contract.

    `j` is normally passed enormous: a free machine accelerates under the torque it is being asked to produce, which
    turns a current-loop measurement into a mechanical one.
    dq 轴 PMSM，每次驱动前向欧拉积分一步，挂在主控 HAL 契约之后。

    `j` 通常传极大值：自由旋转的电动机会在自身转矩下加速，从而把电流环的测量变成机械测量。
    """

    def __init__(self, *, rs, ld, lq, psi_f, pole_pairs, j, b, v_bus):
        self.rs, self.ld, self.lq, self.psi_f = rs, ld, lq, psi_f
        self.pole_pairs, self.j, self.b = pole_pairs, j, b
        self.v_bus = v_bus
        self.theta_e, self.omega_e = 0.0, 0.0
        self.id, self.iq = 0.0, 0.0

    # ---- the HAL contract the binding expects ---------------------------------
    def angle_mech_rad(self):
        return (self.theta_e / self.pole_pairs) % (2.0 * math.pi)

    def velocity_rad_s(self):
        return self.omega_e / self.pole_pairs

    def voltage_v(self):
        return self.v_bus

    def read_current(self):
        c, s = math.cos(self.theta_e), math.sin(self.theta_e)
        ialpha = (self.id * c) - (self.iq * s)
        ibeta = (self.id * s) + (self.iq * c)
        iu = ialpha
        iv = (-0.5 * ialpha) + ((math.sqrt(3.0) / 2.0) * ibeta)
        return (iu, iv, -iu - iv)

    def write_duty(self, du, dv, dw):
        vu, vv, vw = du * self.v_bus, dv * self.v_bus, dw * self.v_bus
        valpha = (2.0 / 3.0) * (vu - (0.5 * vv) - (0.5 * vw))
        vbeta = (2.0 / 3.0) * (((math.sqrt(3.0) / 2.0) * vv) - ((math.sqrt(3.0) / 2.0) * vw))
        c, s = math.cos(self.theta_e), math.sin(self.theta_e)
        vd = (valpha * c) + (vbeta * s)
        vq = (-valpha * s) + (vbeta * c)

        did = ((vd - (self.rs * self.id)) + (self.omega_e * self.lq * self.iq)) / self.ld
        diq = ((vq - (self.rs * self.iq)) - (self.omega_e * self.ld * self.id) - (self.omega_e * self.psi_f)) / self.lq
        self.id += did * DT
        self.iq += diq * DT

        torque = 1.5 * self.pole_pairs * ((self.psi_f * self.iq) + ((self.ld - self.lq) * self.id * self.iq))
        self.omega_e += (torque - (self.b * self.omega_e)) / self.j * DT
        self.theta_e += self.omega_e * DT
