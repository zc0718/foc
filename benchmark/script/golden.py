#!/usr/bin/env python3
"""Golden ledger: a few recorded windows, compared on every run, reported and never blocking.

What this is for
----------------
The trace channel answers "what did the loop do". This answers "did it change, and by how much" -- with numbers,
attached to the name of the quantity, instead of an adjective in a commit message. Three scenarios, each a plain
sequence of API calls whose window is recorded through ``foc.Trace``.

What a golden is, and what it is not
------------------------------------
It is a recording from one machine and one build. It is **not** a specification: the same source compiled by a different
compiler, or with different optimisation, produces different floats in the last digits, and a ledger that pretended
otherwise would be the exact failure the project's foundation log is about -- a wrong number being laundered into
reference data. So the header records the provenance (module hash, build type, sample stride), the report says when the
baseline came from a different build, and the comparison is against a stated tolerance rather than bit equality.

Verdict bands, and why they are shaped this way
-----------------------------------------------
    |relative change| <= 1e-5      same            arithmetic noise between builds
    <= 30 %                          drift           worth a look, not necessarily wrong
    1.3x .. 2.0x                     attribute       the signature of a missing term, not of noise
    > 2.0x                           changed         expected after an algorithm change, a surprise otherwise

The 1.3-2.0 band is not arbitrary: it is where the W7-c investigation landed (0.1356 predicted against 0.0900 measured,
1.51x), and the term that explained it was a real one nobody had modelled. A ledger that calls that "drift" invites
somebody to widen a tolerance instead of finding the term.

Why it does not block
---------------------
The plan is explicit: report, do not block. A ledger that gates a build gets a tolerance widened the first time it is
inconvenient, which is how a reference value stops meaning anything. ``--strict`` exists for a release gate, where a
human has already decided that this build and this baseline are comparable.

Usage
-----
    python3 benchmark/script/golden.py --record      # (re)write every golden from the current tree
    python3 benchmark/script/golden.py --check       # compare now against the stored goldens, exit 0
    python3 benchmark/script/golden.py --check --strict   # exit 1 when anything is outside "same"
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
GOLDEN_DIR = ROOT / 'test_package' / 'resources' / 'golden'
sys.path.insert(0, str(ROOT / 'benchmark' / 'script'))

# The machine the scenarios drive. Not optional: through the C ABI a motor is backed by the stub HAL, whose currents are a
# scripted constant, so a scenario written that way records a loop with no physics in it -- which is exactly what the
# first recording of this ledger produced, and why the summaries came out as -1 and 0. A host plant is the only way the
# numbers mean anything.
# 场景驱动的机器，不可省：走 C ABI 时电机背后是桩 HAL，其电流是一个脚本常数，因此那样写的场景记录下来的是一条没有物理的
# 环——这正是本台账第一次录制得到的东西，也是摘要算成 -1 和 0 的原因。只有主控 plant 才能让这些数字有意义。
# The control period every scenario ticks at. One constant, because the plant and the loop have to agree about it
# and they did not: the plant used its own module constant.
# 所有场景的节拍周期。只此一个常量，因为 plant 与环路必须对它有共识——而它们曾经没有：plant 用的是自己的模块常量。
DT_S = 1e-4

PLANT = {
    'rs': 0.5, 'ld': 4e-4, 'lq': 4e-4, 'psi_f': 0.05 / (1.5 * 7), 'pole_pairs': 7,
    'j': 1e-5, 'b': 1e-6, 'v_bus': 48.0,
}

# Columns of a stored sample, in order. The trace record is the source; this list is the serialisation.
FIELDS = ('t_us', 'i_d', 'i_q', 'i_ref_d', 'i_ref_q', 'v_d', 'v_q', 'speed_mech', 'mode')

# Which quantities a report compares. The window is also compared, at a stride, so that a change which moves nothing
# in the summary is still visible where it happened.
SUMMARY = ('overshoot', 'steady_error', 'settle_s', 'peak_iq', 'peak_abs_vq', 'locked')

# The band that means "a term is missing", not "the noise is bigger than I thought".
ATTRIBUTE_BAND = (1.3, 2.0)
NOISE = 1e-5
DRIFT = 0.30


def _module():
    """`foc_python`, from the environment or from the Conan cache. Same optional-material convention as the gates."""
    try:
        return importlib.import_module('foc_python')
    except ImportError:
        pass
    # Newest build first: the cache keeps every package ever produced, and picking one by name is how the first run of
    # this script imported a module that predated the feature it was about to exercise.
    # 最新的构建优先：缓存保留着产出的每一个包，而按名字挑一个，正是本脚本第一次运行时导入了"早于它要测的功能"的模块的方式。
    candidates = sorted(Path.home().glob('.conan2/p/b/foc*/p/lib'),
                        key=lambda directory: directory.stat().st_mtime, reverse=True)
    for directory in candidates:
        if any(directory.glob('foc_python*')):
            sys.path.insert(0, str(directory))
            try:
                return importlib.import_module('foc_python')
            except ImportError:
                sys.path.remove(str(directory))
    raise SystemExit('[golden] no foc_python module: build with -c user.het:enable_python_bindings=True, '
                     'or set PYTHONPATH to the directory holding it')


def _module_sha(module) -> str:
    path = Path(getattr(module, '__file__', '') or '')
    if not path.is_file():
        return 'unknown'
    digest = hashlib.sha256()
    with path.open('rb') as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b''):
            digest.update(chunk)
    return digest.hexdigest()[:12]


# ----------------------------------------------------------------------------------------------------------------------
# The scenarios. Each returns (summary, window) where the window is a list of dicts in FIELDS order.
# 场景。每个返回 (摘要, 窗口)，窗口是按 FIELDS 顺序排列的 dict 列表。
# ----------------------------------------------------------------------------------------------------------------------


def _plant():
    """A fresh machine per scenario: shared state between recordings would be a hidden variable, and the point of a
    ledger is that the numbers came from the same inputs."""
    from foc_sil_plant import PmsmPlant

    # The plant integrates one step per write, so it is told the period the loop ticks at. Before this was wired the
    # plant advanced 50 us per 100 us tick and every window in this ledger described a machine running at half speed.
    # plant 每次写入积分一步，因此要告诉它环路真正的节拍周期。在接上之前，plant 每次 100 µs 节拍只前进 50 µs，
    # 本台账里每个窗口描述的都是一个半速运行的电机。
    return PmsmPlant(**PLANT, dt_s=DT_S)


def speed_step(foc, stride: int):
    """A 100 rad/s step from rest: the response shape, not a setpoint."""
    periods, dt, target = 2000, DT_S, 100.0
    trace = foc.Trace(periods)
    with foc.Motor(foc.MotorParams(), foc.MotorSetup(i_max_a=5.0, v_bus_v=PLANT['v_bus']), _plant()) as motor:
        motor.set_trace(trace)
        motor.enable()
        motor.set_speed(target)
        for _ in range(periods):
            motor.tick(dt)
        rows = trace.samples(motor)
    speeds = [row['speed_mech'] for row in rows]
    overshoot = (max(speeds) - target) / target
    steady = sum(speeds[-100:]) / 100.0
    settle = next((i * dt for i, value in enumerate(speeds) if abs(value - target) <= 0.02 * target), periods * dt)
    summary = {
        'overshoot': overshoot,
        'steady_error': (steady - target) / target,
        'settle_s': settle,
        'peak_iq': max(abs(row['i_q']) for row in rows),
        'peak_abs_vq': max(abs(row['v_q']) for row in rows),
    }
    return summary, rows[::stride]


def standstill_torque(foc, stride: int):
    """A current step at standstill, held: what the injection-free path does with no motion at all."""
    periods, dt, target = 400, DT_S, 0.5
    trace = foc.Trace(periods)
    with foc.Motor(foc.MotorParams(), foc.MotorSetup(i_max_a=5.0, v_bus_v=PLANT['v_bus']), _plant()) as motor:
        motor.set_trace(trace)
        motor.enable()
        motor.set_torque(target)
        for _ in range(periods):
            motor.tick(dt)
        rows = trace.samples(motor)
    tail = rows[-50:]
    steady = sum(row['i_q'] for row in tail) / len(tail)
    summary = {
        'overshoot': (max(row['i_q'] for row in rows) - target) / target,
        'steady_error': (steady - target) / target,
        'settle_s': 0.0,  # a held current at standstill has no shape to time
        'peak_iq': max(abs(row['i_q']) for row in rows),
        'peak_abs_vq': max(abs(row['v_q']) for row in rows),
    }
    return summary, rows[::stride]


def current_limit(foc, stride: int):
    """A command past the current limit, held: the clamp is in the path, and a change to it shows up as a different
    steady current rather than as a different shape.

    This slot was going to be the injection observer. It cannot be, yet: selecting HFI through `Motor` applies no
    carrier at all -- the duty swing over 200 periods is exactly 0.00000 at 2, 4, 6 and 8 kHz on a 20 kHz loop, and every
    ABI-level HFI case so far asserts admission and the reported mode, never the injection. That is recorded in
    foundation-log.rst as an open defect. A ledger must not freeze a baseline around behaviour nobody understands: the
    next person would compare against "locked = False" and explain a real change away.
    """
    periods, dt, limit = 800, DT_S, 2.0
    trace = foc.Trace(periods)
    setup = foc.MotorSetup(i_max_a=limit, v_bus_v=48.0)
    with foc.Motor(foc.MotorParams(), setup, _plant()) as motor:
        motor.set_trace(trace)
        motor.enable()
        motor.set_torque(3.0)  # 50 % past the limit: the clamp is what the window records
        for _ in range(periods):
            motor.tick(dt)
        rows = trace.samples(motor)
    tail = rows[-50:]
    steady = sum(row['i_q'] for row in tail) / len(tail)
    summary = {
        'overshoot': (max(row['i_q'] for row in rows) - limit) / limit,
        'steady_error': (steady - limit) / limit,
        'settle_s': 0.0,
        'peak_iq': max(abs(row['i_q']) for row in rows),
        'peak_abs_vq': max(abs(row['v_q']) for row in rows),
    }
    return summary, rows[::stride]


SCENARIOS = {
    'speed_step_100': ('100 rad/s step from rest, 2000 periods at 100 us', speed_step),
    'standstill_torque': ('0.5 A held at standstill, 400 periods at 100 us', standstill_torque),
    'current_limit': ('3.0 A demanded against a 2.0 A limit, held, 800 periods at 100 us', current_limit),
}

STRIDE = 10


# ----------------------------------------------------------------------------------------------------------------------
# Serialisation and comparison. Plain text on purpose: a golden nobody can read in a diff is a binary blob with a
# commit message attached.
# 序列化与比较。刻意用纯文本：在 diff 里没人能读的基准，就是一个带了提交说明的二进制块。
# ----------------------------------------------------------------------------------------------------------------------


def dump(name: str, summary: dict, window: list, module_sha: str, stride: int) -> str:
    lines = [f'# golden: {name}', f'# what: {SCENARIOS[name][0]}', f'# module_sha: {module_sha}',
             f'# stride: {stride}']
    for key in SUMMARY:
        if key in summary:
            lines.append(f'# {key}: {summary[key]:.9g}')
    lines.append('#')
    lines.append('#' + ','.join(FIELDS))
    for row in window:
        values = [str(row['t_us'])] + [f'{row[key]:.9g}' for key in FIELDS[1:-1]] + [str(int(row['mode']))]
        lines.append(','.join(values))
    return '\n'.join(lines) + '\n'


def load(path: Path) -> tuple[dict, list]:
    meta, rows = {}, []
    for line in path.read_text(encoding='utf-8').splitlines():
        if line.startswith('#') and ',' not in line:
            body = line[1:].strip()
            if ':' in body:
                key, value = body.split(':', 1)
                meta[key.strip()] = value.strip()
            continue
        if line.startswith('#') or not line.strip():
            continue
        parts = line.split(',')
        row = {'t_us': int(parts[0])}
        for index, key in enumerate(FIELDS[1:-1], start=1):
            row[key] = float(parts[index])
        row['mode'] = int(parts[-1])
        rows.append(row)
    return meta, rows


def verdict(golden: float, current: float, scale: float | None = None) -> tuple[str, float]:
    """Classify one quantity. `scale` is what to divide by when the golden itself is near zero."""
    if golden == current:
        return 'same', 0.0
    denominator = abs(golden) if abs(golden) > 1e-12 else (scale if scale else 1.0)
    relative = (current - golden) / denominator
    magnitude = abs(relative)
    if magnitude <= NOISE:
        return 'same', relative
    # A sign flip is not "a factor of one": the ratio of magnitudes is 1.0 while the value moved through zero, so the
    # band test alone would file it as ordinary drift. Two quantities that disagree about direction disagree completely.
    # 符号翻转不是"一倍"：幅值比是 1.0，而值穿过了零，所以单看比例带会把它归成普通漂移。两个在方向上都不同的量，
    # 是完全不同。
    if (golden * current) < 0.0:
        return 'changed', relative
    ratio = abs(current) / abs(golden) if abs(golden) > 1e-12 else math.inf
    if ATTRIBUTE_BAND[0] <= ratio <= ATTRIBUTE_BAND[1]:
        return 'attribute', relative
    if magnitude <= DRIFT:
        return 'drift', relative
    return 'changed', relative


def compare(name: str, meta: dict, golden_rows: list, current_rows: list, current_summary: dict) -> list[str]:
    """Return report lines for one golden. The shape of the output is the criterion: name, quantity, both values, delta."""
    out = []
    mismatched_length = len(golden_rows) != len(current_rows)
    out.append(f'[{name}] {SCENARIOS[name][0]}')
    out.append(f'[{name}] baseline module {meta.get("module_sha", "unknown")}, '
               f'{len(golden_rows)} sample(s) at stride {meta.get("stride", "?")}; now {len(current_rows)}')
    if mismatched_length:
        out.append(f'[{name}] WINDOW LENGTH {len(golden_rows)} -> {len(current_rows)}: the scenario itself changed')
    flagged = []
    for key in SUMMARY:
        recorded = meta.get(key)
        if (recorded is None) or (key not in current_summary):
            continue
        golden_value = float(recorded)
        current_value = current_summary[key]
        tag, relative = verdict(golden_value, current_value)
        out.append(f'[{name}]   {key:14s} {golden_value: .9g} -> {current_value: .9g}  '
                   f'({relative:+.2e}) {tag}')
        if tag != 'same':
            flagged.append((tag, key, golden_value, current_value, relative))
    # the window, quantity by quantity, so a change that cancels in the summary is still visible where it happened
    for position, (old, new) in enumerate(zip(golden_rows, current_rows)):
        for key in FIELDS[1:-1]:
            tag, relative = verdict(old[key], new[key], scale=current_summary['peak_abs_vq'])
            if tag != 'same':
                flagged.append((tag, f'{key}@{position}', old[key], new[key], relative))
    if flagged:
        out.append(f'[{name}] {len(flagged)} quantit(ies) outside "same":')
        for tag, key, old, new, relative in flagged[:40]:
            note = '  <- a missing term looks like this' if tag == 'attribute' else ''
            out.append(f'[{name}]   {tag:9s} {key:18s} {old: .9g} -> {new: .9g} ({relative:+.2%}){note}')
    else:
        out.append(f'[{name}] every summary quantity and every window slot is "same"')
    out.append('')
    return out


def run(names, check: bool) -> tuple[int, str]:
    """Record or compare. Returns (exit code, worst verdict)."""
    foc = _module()
    sha = _module_sha(foc)
    GOLDEN_DIR.mkdir(parents=True, exist_ok=True)
    rank = {'same': 0, 'drift': 1, 'attribute': 2, 'changed': 3}
    worst = 'same'
    compared = 0
    for name in names:
        if name not in SCENARIOS:
            raise SystemExit(f'[golden] unknown scenario {name}; known: {", ".join(SCENARIOS)}')
        summary, window = SCENARIOS[name][1](foc, STRIDE)
        path = GOLDEN_DIR / f'{name}.csv'
        if not check:
            path.write_text(dump(name, summary, window, sha, STRIDE), encoding='utf-8')
            print(f'[golden] recorded {name} -> {path.relative_to(ROOT)} ({len(window)} sample(s), module {sha})')
            continue
        if not path.is_file():
            print(f'[golden] {name}: no baseline at {path.relative_to(ROOT)}; run --record')
            continue
        meta, golden_rows = load(path)
        for line in compare(name, meta, golden_rows, window, summary):
            print(line)
        compared += 1
        for key in SUMMARY:
            if key in meta and key in summary:
                tag, _ = verdict(float(meta[key]), summary[key])
                if rank[tag] > rank[worst]:
                    worst = tag
    if check:
        if compared == 0:
            print('[golden] nothing recorded yet: a ledger with no baseline measures nothing')
            return 0, 'same'
        print(f'[golden] worst verdict over {compared} trace(s): {worst}' +
              ('' if worst == 'same' else ' -- report only; a ledger that blocks gets its tolerance widened'))
    return 0, worst


def main() -> int:
    parser = argparse.ArgumentParser(description='Record and compare golden windows.')
    parser.add_argument('--record', action='store_true', help='(re)write the goldens from the current tree')
    parser.add_argument('--check', action='store_true', help='compare against the stored goldens')
    parser.add_argument('--scenario', action='append', default=None, help='limit to these scenarios (repeatable)')
    parser.add_argument('--strict', action='store_true',
                        help='exit 1 when anything is not "same" (for a release gate, after a human approved the '
                             'comparison)')
    args = parser.parse_args()
    if not (args.record or args.check):
        parser.error('one of --record or --check is required')
    code, worst = run(args.scenario or list(SCENARIOS), check=args.check)
    if args.strict and args.check and worst != 'same':
        return 1
    return code


if __name__ == '__main__':
    sys.exit(main())
