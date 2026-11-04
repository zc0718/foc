#!/usr/bin/env python3
"""Keeps the one clock rule this repository learned the hard way: a duration that only grows is not a float.

`HomingRoutine` decided its timeout on `elapsed_s_ += dt_s` in float. A float sum of periods does not merely lose
precision -- it changes speed and then stops: once `ulp(t) > dt / 2` each addition advances the sum by an ULP instead
of by `dt`, and once `ulp(t) > dt` the addition is a no-op. Measured at a 40 kHz control period: the sum reached
"512 s" after 17 866 579 periods -- 446.664 s of real time, 12.8 % fast -- and then could not advance at all, so a
520 s recipe never timed out and the axis would have crept forever. That is F-105 in `docs/sphinx/source/foundation-log.rst`.

Why a gate and not a paragraph: the same line was written five times in this library, and only one of the five had a
limit large enough to reach the floor. Four latent copies of a defect that fires above a magnitude nobody tests is
exactly what a reviewer cannot see by reading, and "remember to use the clock" is the kind of rule that holds until
somebody writes the next timer. So the rule is mechanical:

  a `float` member that is accumulated with `+= <something> dt` is a defect, unless it is on the allow list below.

The allow list exists because two shapes are legitimately float and are *not* clocks:

  * a phase decimator (`speed_elapsed_s_`, `position_elapsed_s_`) -- cleared every time the loop it gates runs, so its
    magnitude is bounded by the configured period;
  * a window counter bounded by a constant (`stall_elapsed_s_`) -- reaching the constant faults, so it never grows past
    it.

Both are bounded by construction, so `ulp(t)` never leaves the range where a float add of `dt` is exact. An entry here
is a claim about that bound, and the entry carries the reason so the next reader can check it.

**Known limit, measured 2026-11-04 (batch 2).** The classifier is a heuristic on *syntax*, not on meaning: it matches
`+= <expr> dt`. A float accumulator written as `q_ = static_cast<float>(q_ + (... dt ...))` is the same defect and is
not seen -- `core/protection.hpp`'s I2t integral has exactly that shape and passes today. `core/brake_chopper.hpp`'s
heat accumulator was caught only because it was written with `+=`, and it was then made a `double` (an integral, not a
duration: no nanoseconds, but more than 24 bits of mantissa). Widening the pattern would flag `protection.hpp`, whose
row is L3, so that is its own change with its own measurement -- recorded here rather than quietly fixed, because a
gate whose blind spot is undocumented is a gate that will be trusted where it cannot see.

`--selftest` classifies four snippets (growing float, bounded float, integer clock, and the allow-listed names) so a
classifier that has stopped finding anything cannot pass by having nothing to find.
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCAN = ('include', 'src')

# name -> why a float accumulator with this name is bounded. Keyed by member name because the bound is a property of
# what the member counts, and the same name means the same thing everywhere it appears.
ALLOWED = {
    'speed_elapsed_s_': 'a phase decimator: cleared every time the speed loop runs, so ulp(t) <= ulp(period)',
    'position_elapsed_s_': 'a phase decimator: cleared every time the position loop runs, so ulp(t) <= ulp(period)',
    'stall_elapsed_s_': 'bounded by kStallWindowS: reaching it faults, so it never grows past 0.1 s',
}

FLOAT_DECL = re.compile(r'\bfloat\s+([A-Za-z_]\w*)\s*[{;=]')
ACCUMULATE = re.compile(r'\b([A-Za-z_]\w*)\s*\+=\s*([^;]+);')


def scan_text(text: str) -> list[tuple[int, str, str]]:
    """Every `float` member that is accumulated with a `dt` term, as (line number, name, the added expression)."""
    floats: dict[str, int] = {}
    for index, line in enumerate(text.splitlines(), start=1):
        for match in FLOAT_DECL.finditer(line):
            floats.setdefault(match.group(1), index)
    hits: list[tuple[int, str, str]] = []
    for index, line in enumerate(text.splitlines(), start=1):
        for match in ACCUMULATE.finditer(line):
            name, expression = match.group(1), match.group(2)
            if name in floats and re.search(r'\bdt\w*\b', expression):
                hits.append((index, name, expression.strip()))
    return hits


def scan_file(path: Path) -> list[str]:
    try:
        text = path.read_text(encoding='utf-8', errors='replace')
    except OSError as error:  # pragma: no cover - unreadable file is a build problem, not a gate problem
        return [f'{path}: cannot read ({error})']
    problems = []
    rel = path.relative_to(ROOT).as_posix()
    for line, name, expression in scan_text(text):
        if name in ALLOWED:
            continue
        problems.append(f'{rel}:{line}: `{name} += {expression}` -- a duration that only grows must be a '
                        f'`core::ElapsedClock`, not a float (F-105)')
    return problems


def scan_tree() -> list[str]:
    problems = []
    for top in SCAN:
        for path in sorted((ROOT / top).rglob('*.hpp')):
            problems.extend(scan_file(path))
        for path in sorted((ROOT / top).rglob('*.cpp')):
            problems.extend(scan_file(path))
    return problems


def selftest() -> bool:
    growing = 'class A final {\n    float elapsed_s_{0.0F};\n    void step(float dt_s) { elapsed_s_ += dt_s; }\n};'
    bounded = 'class B final {\n    float speed_elapsed_s_{0.0F};\n    void step(float dt_s) { speed_elapsed_s_ += dt_s; }\n};'
    clock = 'class C final {\n    ElapsedClock elapsed_;\n    void step(float dt_s) { elapsed_.add_sample(dt_s); }\n};'
    other = 'class D final {\n    float x_{0.0F};\n    void step(float gain) { x_ += gain; }\n};'
    cases = [('a growing float clock', growing, 1), ('an allow-listed bounded float', bounded, 0),
             ('an integer clock', clock, 0), ('a float that is not a clock', other, 0)]
    ok = True
    for label, text, want in cases:
        # The allow list is applied the same way `scan_file` applies it, so the selftest exercises the real classifier.
        got = len([hit for hit in scan_text(text) if hit[1] not in ALLOWED])
        print(f'[clock-selftest] {label:32s} -> {got} finding(s) (want {want})')
        ok = ok and (got == want)
    return ok


def main(argv: list[str]) -> int:
    if '--selftest' in argv:
        if not selftest():
            print('[clock] FAIL (selftest)', file=sys.stderr)
            return 1
    problems = scan_tree()
    if problems:
        for problem in problems:
            print(f'[clock] {problem}', file=sys.stderr)
        print(f'[clock] FAIL ({len(problems)} problem(s))', file=sys.stderr)
        return 1
    print(f'[clock] OK ({len(ALLOWED)} bounded float accumulators allow-listed, everything else on core::ElapsedClock)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
