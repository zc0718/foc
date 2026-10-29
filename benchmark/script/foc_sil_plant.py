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

    def __init__(self, *, rs, ld, lq, psi_f, pole_pairs, j, b, v_bus, dt_s=DT):
        self.rs, self.ld, self.lq, self.psi_f = rs, ld, lq, psi_f
        self.pole_pairs, self.j, self.b = pole_pairs, j, b
        self.v_bus = v_bus
        # The plant integrates one step per `write_duty` call, so this has to be the period the caller actually ticks
        # at. It used to be the module constant regardless: a caller ticking at 100 us advanced the model by 50 us, and
        # every plant-derived number ran at half speed. Measured as a factor of exactly 2 in the closed-loop bandwidth.
        # 本 plant 每次 `write_duty` 积分一步，因此这里必须是调用方真正的节拍周期。它以前一律用模块常量：以 100 µs
        # 节拍的调用方每次只让模型前进 50 µs，所有来自 plant 的数字都只有一半速度。实测为闭环带宽上恰好 2 倍。
        self.dt_s = dt_s
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
        self.id += did * self.dt_s
        self.iq += diq * self.dt_s

        # `torque` is N*m and `j` is kg*m^2, so `torque / j` is a **mechanical** acceleration -- while `omega_e` is
        # electrical. Adding one to the other made every mechanical transient `pole_pairs` times too slow (measured:
        # 7.002x at seven pole pairs); the friction term had the same mismatch, damping in electrical radians.
        # `torque` 是 N·m、`j` 是 kg·m²，因此 `torque / j` 是**机械**加速度——而 `omega_e` 是电角速度。把两者相加会让
        # 每一个机械暂态慢 `pole_pairs` 倍（实测七对极下 7.002 倍）；摩擦项有同样的错位，按电角度阻尼。
        torque = 1.5 * self.pole_pairs * ((self.psi_f * self.iq) + ((self.ld - self.lq) * self.id * self.iq))
        omega_m = self.omega_e / self.pole_pairs
        omega_m += ((torque - (self.b * omega_m)) / self.j) * self.dt_s
        self.omega_e = omega_m * self.pole_pairs
        self.theta_e += self.omega_e * self.dt_s
