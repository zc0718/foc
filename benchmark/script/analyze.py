"""The analysis layer: what a recorded window says about the loop that produced it.

Why this module exists. The library can record a window (`Motor.trace()` / `foc_motor_trace_read`) and, since 0.1.8, it
can also measure one (`foc_step_metrics`, reached here as `foc_python.step_metrics`). Two implementations of the same
metric is one too many unless the disagreement is *measured*, so this module is the independent one: numpy, written
against the same definition, and checked against the C++ side to within 1 % on the same samples. When they disagree, one
of them is wrong and the build says so; when only one exists, nobody finds out.

分析层：一段录制窗口对产生它的环路说了什么。

本模块存在的意义。库能录窗口（`Motor.trace()` / `foc_motor_trace_read`），自 0.1.8 起也能测窗口（`foc_step_metrics`，
这里通过 `foc_python.step_metrics` 可达）。同一个指标有两份实现，除非**把分歧测出来**，否则就多了一份：因此本模块
是独立的那一份——numpy 写成、遵循同一份定义、并与 C++ 侧在同一批样本上比到 1% 以内。两者不一致时，其中一个是错的，
构建会说出来；只有一份存在时，没人会知道。

Definitions, once.
-----------------
`step_metrics` follows the definition in `include/foc/core/step_metrics.hpp`, including the parts that look arbitrary:

- the window starts at or before the step; `y[0]` is the pre-step value;
- settling is decided by the *last* sample outside the band, so a response that leaves and re-enters is judged by where
  it ends up;
- the first-order fit uses only samples strictly between the start and the target, because an overshoot would ask for
  `ln` of a non-positive number and the two endpoints carry no information about the time constant;
- `bandwidth_rad_s = 1 / tau`, which is the -3 dB point of `1 / (1 + s*tau)`.

定义只写一次。`step_metrics` 遵循 `include/foc/core/step_metrics.hpp` 里的定义，包括那些看起来随意的部分：窗口从阶跃前
开始、稳定时间由最后一个带外样本决定、一阶拟合只用严格落在起点与目标之间的样本、`bandwidth_rad_s = 1 / tau`。
"""

from __future__ import annotations

import json
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np


@dataclass
class Metrics:
    """The same five numbers the C++ side reports, plus the reason when it cannot fit one of them."""

    status: str
    bandwidth_rad_s: float
    tau_s: float
    settling_s: float
    overshoot_fraction: float
    tail_error_fraction: float

    def as_dict(self) -> dict:
        return {
            'status': self.status,
            'bandwidth_rad_s': self.bandwidth_rad_s,
            'tau_s': self.tau_s,
            'settling_s': self.settling_s,
            'overshoot_fraction': self.overshoot_fraction,
            'tail_error_fraction': self.tail_error_fraction,
        }


def step_metrics(y, dt_s: float, target: float, band: float) -> Metrics:
    """Measure a step response. Raises `ValueError` for arguments that cannot describe a step.

    `status` is ``Ok`` when a decay was fitted and ``State`` when the window carried none -- in the latter case the
    settling numbers are still valid, because a response can settle without being fittable.
    """
    y = np.asarray(y, dtype=np.float64)
    n = y.size
    if n < 3:
        raise ValueError('fewer than three samples cannot describe a step')
    if not dt_s > 0.0:
        raise ValueError('dt_s must be positive')
    if not band >= 0.0:
        raise ValueError('band must not be negative')
    if not np.all(np.isfinite(y)):
        raise ValueError('the window carries a non-finite sample')

    y0 = float(y[0])
    step = target - y0
    if not abs(step) > 0.0:
        raise ValueError('the window does not move: target equals the first sample')

    # Settling: the last sample outside the band decides it.
    # 稳定时间由最后一个带外样本决定。
    outside = np.flatnonzero(np.abs(y - target) > band)
    if outside.size == 0:
        settling_s = 0.0
    elif int(outside[-1]) + 1 >= n:
        settling_s = -1.0
    else:
        settling_s = float(int(outside[-1]) + 1) * float(dt_s)

    tail_count = max(1, n // 10)
    beyond = float(np.max(y)) - target
    overshoot = (beyond / abs(step)) if beyond > 0.0 else 0.0
    tail_error = (float(np.mean(y[-tail_count:])) - target) / abs(step)

    # First-order fit through the origin on the fraction still to go: w = 1 - (y - y0)/step = exp(-t/tau).
    # 对"还差多少"做过原点一阶拟合：w = 1 - (y - y0)/step = exp(-t/tau)。
    t = np.arange(1, n, dtype=np.float64) * float(dt_s)
    w = 1.0 - (y[1:] - y0) / step
    usable = (w > 0.0) & (w < 1.0)
    if int(np.count_nonzero(usable)) < 3:
        return Metrics('State', 0.0, 0.0, settling_s, overshoot, tail_error)

    t_u = t[usable]
    slope = float(np.dot(t_u, np.log(w[usable])) / np.dot(t_u, t_u))
    if not slope < 0.0:
        return Metrics('State', 0.0, 0.0, settling_s, overshoot, tail_error)

    tau = -1.0 / slope
    return Metrics('Ok', 1.0 / tau, tau, settling_s, overshoot, tail_error)


def spectrum(y, dt_s: float):
    """Amplitude spectrum of a window: one-sided, Hann-windowed, corrected so a pure tone reads its own amplitude.

    Returns `(freqs_hz, amplitudes)`. The correction is `2 / sum(window)` per bin, which makes a sine of amplitude A
    read A at its bin instead of A/2 times a window-dependent factor -- the usual way a spectrum silently lies by 2x.
    """
    y = np.asarray(y, dtype=np.float64)
    n = y.size
    if n < 4:
        raise ValueError('a spectrum needs at least four samples')
    window = np.hanning(n)
    scale = 2.0 / float(np.sum(window))
    amps = np.abs(np.fft.rfft((y - float(np.mean(y))) * window)) * scale
    freqs = np.fft.rfftfreq(n, d=float(dt_s))
    return freqs, amps


def thd(freqs, amps, f0_hz: float, harmonics: int = 10) -> tuple:
    """Total harmonic distortion of the tone at `f0_hz`: sqrt(sum of harmonics^2) / fundamental.

    Returns `(thd_fraction, fundamental, harmonic_amplitudes)`. Harmonics above Nyquist are skipped rather than counted
    as zero, because a THD that quietly changes meaning with the sample rate is worse than one that says how many
    harmonics it actually measured.
    """
    def peak_near(f):
        if not (0.0 < f < float(freqs[-1])):
            return 0.0
        k = int(np.argmin(np.abs(freqs - f)))
        lo, hi = max(0, k - 2), min(amps.size, k + 3)
        return float(np.max(amps[lo:hi]))

    fundamental = peak_near(f0_hz)
    if not fundamental > 0.0:
        return float('nan'), 0.0, []
    harmonics_out = [peak_near(f0_hz * (k + 2)) for k in range(harmonics)]
    total = float(np.sqrt(np.sum(np.square(harmonics_out))))
    return total / fundamental, fundamental, harmonics_out


# ---- window files ------------------------------------------------------------------------------
#
# The window format is the one `golden.py` records and `foc_trace_sample_t` defines: `#` comment lines, then a header
# row, then samples oldest first. `load_window` reads it back so the same file can be compared by this module and by a
# C++ case -- one artifact, two readers.
#
# 窗口格式就是 `golden.py` 录制、`foc_trace_sample_t` 定义的那一种：`#` 注释行、表头行、按时间顺序的样本。`load_window`
# 把它读回来，于是一个文件既能被本模块比较、也能被 C++ 用例比较——一个产物，两个读者。

# The column order of a stored window, which is the trace record's field order. `golden.FIELDS` is the same list and
# the Python suite asserts the two are equal: a reader whose labels drift from the writer's order answers confidently
# about the wrong column, and no exception is raised anywhere.
# 存储窗口的列顺序，即轨迹记录的字段顺序。`golden.FIELDS` 是同一张表，Python 套件断言两者相等：读取器的列名若与写入
# 器的顺序漂移，它会自信地就错误的列给出答案，而任何地方都不会抛异常。
TRACE_COLUMNS = ('t_us', 'i_d', 'i_q', 'i_ref_d', 'i_ref_q', 'v_d', 'v_q', 'speed_mech', 'mode')


def load_window(path, columns=TRACE_COLUMNS) -> dict:
    """Load a recorded window into `{column: np.ndarray}` plus a `# k: v` header under the `_header` key.

    The ledger's files carry their field names nowhere but in `columns`: their header is `#` comments only, and the
    first data line is data. A file that *does* have a field row is accepted too -- `columns` is then ignored -- because
    an oscilloscope export and a ledger entry should not need two readers.
    """
    header = {}
    rows = []
    names = ()
    with open(path, 'r', encoding='utf-8') as handle:
        for line in handle:
            line = line.strip()
            if not line:
                continue
            if line.startswith('#'):
                body = line[1:].strip()
                if ':' in body:
                    key, value = body.split(':', 1)
                    header[key.strip()] = value.strip()
                elif ',' in body:
                    names = tuple(name.strip() for name in body.split(','))   # the recorder's field row, commented
                continue
            fields = [field.strip() for field in line.split(',')]
            if not names:
                try:
                    rows.append([float(field) for field in fields])
                    names = tuple(columns)
                except ValueError:
                    names = tuple(fields)   # a real field row: use it
                continue
            rows.append([float(field) for field in fields])
    data = {name: np.array([row[i] for row in rows], dtype=np.float64) for i, name in enumerate(names)}
    data['_header'] = header
    return data


def main(argv=None) -> int:
    """Read one window and print its metrics as JSON. Intentionally dumb: one file in, numbers out.

    The period is an argument because it cannot be inferred from a recorded window: on a HAL with no clock every
    `t_us` is 0, and the recorder's stride means the sample spacing is not the loop period either. A reader that
    guessed would be wrong on exactly the files this tool exists to read.
    """
    argv = list(sys.argv[1:] if argv is None else argv)
    if len(argv) < 4:
        print('usage: analyze.py <window.csv> <dt_s> <target> <band> [column]', file=sys.stderr)
        return 2
    window = load_window(argv[0])
    dt_s, target, band = float(argv[1]), float(argv[2]), float(argv[3])
    column = argv[4] if len(argv) > 4 else 'speed_mech'
    if column not in window:
        print(f'{column!r} is not one of {sorted(k for k in window if not k.startswith("_"))}', file=sys.stderr)
        return 2
    print(json.dumps(step_metrics(window[column], dt_s, target, band).as_dict(), indent=2))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
