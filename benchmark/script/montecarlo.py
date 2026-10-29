"""Monte-Carlo over machines that are not quite the ones their nameplate describes.

Why this exists. Every gain in this library is derived from the nameplate -- that is the claim the whole tuning story
rests on -- so the honest question for a design engineer is not "is the loop stable" but "how wrong can the nameplate
be before it is not". This module answers that by drawing machines, running the loop on each, and reporting the spread
and the yield.

本模块存在的意义。本库的每一个增益都从铭牌推导——整条整定叙事都建立在这条断言上——所以对设计工程师而言，诚实的
问题不是"环路稳不稳"，而是"铭牌错到什么程度就不成立了"。本模块的办法是抽样机器、逐台跑环、报告离散度与合格率。

Two kinds of family, and they answer different questions.

- **mismatched**: the real machine is nominal, the nameplate is drawn. This measures *nameplate error*, and the
  analytic prediction is exact to first order: `kp = wb*J/Kt`, so a nameplate `J` scaled by (1+d) scales the loop gain
  and therefore the bandwidth by (1+d), and a nameplate `kt` scaled by (1+d) scales it by 1/(1+d).
- **matched**: each drawn machine is *correctly* described by its own nameplate. The gain law cancels the machine, so
  the bandwidth must not move at all -- the spread is the measurement of the claim, and it should be near zero.
  两种族，回答不同的问题。**不匹配**：真机是标称的、铭牌是抽出来的，测的是**铭牌误差**，一阶解析预测是精确的。
  **匹配**：抽出来的每台机器都被它自己的铭牌正确描述，增益律把机器抵消掉，带宽必须完全不动——这个离散度就是那条
  断言本身的测量。

Criteria (written before the code, each one checked in the suite and each one watched going red):

1. `sigma = 0` reproduces the nominal measurement **bit for bit**. A Monte-Carlo harness that perturbs something on its
   own -- a shared plant, a stale seed, a setup applied twice -- shows up here and nowhere else.
2. A single nameplate parameter off by a factor moves the bandwidth by the analytic factor above, within 2 %.
3. A matched family has a bandwidth spread far below the parameter spread, and its yield inside the 10 % band is 100 %.
   抽样器自身的统计（均值、标准差）在蒙特卡罗误差内与请求一致。

判据（先写后量，逐条在套件里检查、也逐条验过能变红）：σ=0 时必须**逐位**复现标称测量；单参数铭牌偏差必须给出解析倍数
（2% 内）；匹配族的带宽离散度必须远小于参数离散度、且 10% 带内合格率 100%。

Not claimed: any absolute failure rate. These are model numbers from a documented plant, at L2 -- the same standing as
every other number on the evidence page. Monte-Carlo turns one measurement into a distribution; it does not turn a
model into hardware.
"""

from __future__ import annotations

import argparse
import json
import math
import random
import sys
from dataclasses import dataclass

import numpy as np

import analyze

# The nominal machine: one table, so the plant and the nameplate cannot drift apart between trials.
# 标称机器：一张表，使 plant 与铭牌在试验之间不会走偏。
NOMINAL = {
    'rs': 0.5,
    'ld': 4e-4,
    'lq': 4e-4,
    'kt': 0.05,
    'j': 1e-5,
    'b': 1e-6,
    'pole_pairs': 7,
    'v_bus': 48.0,
}

DT_S = 1e-4
TARGET_RAD_S = 10.0
BANDWIDTH_RAD_S = 250.0


def _plant(nameplate: dict):
    """The truth behind the nameplate: a `PmsmPlant` whose torque constant is the nameplate's `kt`."""
    from foc_sil_plant import PmsmPlant

    return PmsmPlant(rs=nameplate['rs'], ld=nameplate['ld'], lq=nameplate['lq'],
                     psi_f=nameplate['kt'] / (1.5 * nameplate['pole_pairs']),
                     pole_pairs=nameplate['pole_pairs'], j=nameplate['j'], b=nameplate['b'],
                     v_bus=nameplate['v_bus'], dt_s=DT_S)


def _motor(nameplate: dict, bandwidth_rad_s: float, plant):
    """A motor told the nameplate and driving `plant`."""
    import foc_python as foc

    params = foc.MotorParams()
    params.rs, params.ld, params.lq = nameplate['rs'], nameplate['ld'], nameplate['lq']
    params.kt, params.j, params.b = nameplate['kt'], nameplate['j'], nameplate['b']
    params.pole_pairs = nameplate['pole_pairs']

    setup = foc.MotorSetup(pwm_freq_hz=1.0 / DT_S, i_max_a=5.0, v_bus_v=nameplate['v_bus'])
    setup.speed_bandwidth_rad_s = bandwidth_rad_s
    return foc.Motor(params, setup, plant)


def step_window(nameplate: dict, plant_nameplate: dict, bandwidth_rad_s: float = BANDWIDTH_RAD_S,
                target_rad_s: float = TARGET_RAD_S, periods: int = 1000) -> list:
    """Run one speed step and return the recorded window, oldest first, starting at the command.

    Two nameplates on purpose: the controller is told one, the machine obeys the other. They are the same object for a
    matched family, which is the whole difference between "the nameplate is wrong" and "the machine varies".
    刻意给两张铭牌：控制器被告知一张，机器服从另一张。匹配族里两者是同一个对象——这正是"铭牌错了"与"机器在变"的区别。
    """
    plant = _plant(plant_nameplate)
    window = [0.0]
    with _motor(nameplate, bandwidth_rad_s, plant) as motor:
        motor.enable()
        motor.set_speed(0.0)
        for _ in range(50):
            motor.tick(DT_S)
        motor.set_speed(target_rad_s)
        for _ in range(periods):
            motor.tick(DT_S)
            window.append(plant.velocity_rad_s())
    return window


@dataclass
class Trial:
    """One drawn machine and what its loop did."""

    nameplate: dict
    bandwidth_rad_s: float
    settling_s: float
    status: str

    def as_dict(self) -> dict:
        return {'bandwidth_rad_s': self.bandwidth_rad_s, 'settling_s': self.settling_s, 'status': self.status}


def one_trial(nameplate: dict, plant_nameplate: dict, bandwidth_rad_s: float = BANDWIDTH_RAD_S,
              target_rad_s: float = TARGET_RAD_S) -> Trial:
    window = step_window(nameplate, plant_nameplate, bandwidth_rad_s, target_rad_s)
    measured = analyze.step_metrics(window, DT_S, target_rad_s, 0.02 * target_rad_s)
    return Trial(nameplate, measured.bandwidth_rad_s, measured.settling_s, measured.status)


def draw(rng: random.Random, sigma: float, parameters=('j', 'kt', 'b', 'rs', 'ld', 'lq')) -> dict:
    """One multiplicative draw around nominal: each named parameter is scaled by `1 + N(0, sigma)`.

    Multiplicative because that is the shape of a nameplate error -- a datasheet is not wrong by "0.2 mH", it is wrong
    by "20 %". `pole_pairs` and `v_bus` are not drawn: the first is exact by construction and the second is measured.
    用乘性是因为铭牌误差就是这个形状——数据手册不会错"0.2 mH"，它会错"20%"。`pole_pairs` 与 `v_bus` 不抽：前者构造上
    精确，后者是量出来的。
    """
    drawn = dict(NOMINAL)
    for name in parameters:
        drawn[name] = NOMINAL[name] * (1.0 + (sigma * rng.gauss(0.0, 1.0)))
    if drawn['kt'] <= 0.0 or drawn['j'] <= 0.0:
        # A draw that lands on a non-physical machine is redrawn rather than clamped: clamping would quietly change the
        # distribution the report claims to describe.
        # 落到非物理机器上的抽样重抽而不是截断：截断会悄悄改掉报告声称描述的那个分布。
        return draw(rng, sigma, parameters)
    return drawn


def stream(trials: int, *, sigma: float = 0.0, seed: int = 0, matched: bool = True,
           bandwidth_rad_s: float = BANDWIDTH_RAD_S, target_rad_s: float = TARGET_RAD_S) -> list:
    """`trials` machines. `matched` means each nameplate describes its own machine; otherwise the machine is nominal."""
    rng = random.Random(seed)
    out = []
    for _ in range(trials):
        drawn = draw(rng, sigma)
        truth = drawn if matched else dict(NOMINAL)
        out.append(one_trial(drawn, truth, bandwidth_rad_s, target_rad_s))
    return out


def report(trials: list, bandwidth_rad_s: float = BANDWIDTH_RAD_S, tolerance: float = 0.10) -> dict:
    """Spread and yield of a run, in the terms a specification is written in."""
    values = np.array([t.bandwidth_rad_s for t in trials], dtype=np.float64)
    inside = np.abs(values / bandwidth_rad_s - 1.0) <= tolerance
    return {
        'trials': len(trials),
        'fitted': int(np.count_nonzero(values > 0.0)),
        'median_rad_s': float(np.median(values)),
        'p05_rad_s': float(np.percentile(values, 5)),
        'p95_rad_s': float(np.percentile(values, 95)),
        'yield': float(np.mean(inside)),
        'tolerance': tolerance,
    }


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description='Monte-Carlo over nameplate error.')
    parser.add_argument('--trials', type=int, default=20)
    parser.add_argument('--sigma', type=float, default=0.20, help='relative standard deviation of each parameter')
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--bandwidth', type=float, default=BANDWIDTH_RAD_S)
    parser.add_argument('--matched', action='store_true', help='each nameplate describes its own machine')
    options = parser.parse_args(argv)

    trials = stream(options.trials, sigma=options.sigma, seed=options.seed, matched=options.matched,
                    bandwidth_rad_s=options.bandwidth)
    print(json.dumps(report(trials, options.bandwidth), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
