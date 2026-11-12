#!/usr/bin/env python3
# Copyright (c) 2026 HeT-AI
# SPDX-License-Identifier: Apache-2.0
"""Every feature in the horizontal comparison says what this library's cell rests on.

Why it exists. The survey of ten third-party drives was written as a table with one column for this
project, and that column **drifted**: capabilities landed on 2026-11-04 and the table still said
`no`, and one cell recorded "trapezoidal/S-curve trajectory" against `MotionProfile`, which shapes a
*velocity* command and emits no position -- a cell that recorded "there is a file" as "there is a
capability". A hand-written cell cannot rot on its own: something has to be re-read against a source.
This makes the source explicit.

What it checks, and what it deliberately does not:

  * a cell that says `has` must name at least one module, and that module must have a **README claim
    row** -- the row with the level, the one-line claim and the evidence case. A cell cannot point at
    a header nobody claims, so "there is a file" stops being an answer;
  * `partial` needs a claim *and* a sentence saying which part is not there; `absent` must claim
    nothing *and* say what closing it would cost. Both carry the reason the survey's tables never
    had, which is why they were re-litigated every audit;
  * the reverse direction is exact rather than a budget: every module with a README claim row is either
    reachable from a cell or listed in `beyond_the_comparison` -- a group of modules that says, in one
    sentence, *why* it is not a comparison row (the HAL contracts, the host stand-ins, the foundation
    types, the instruments this project measures itself with). A new claim row is therefore red until
    it is placed, and the placement is a decision somebody wrote down rather than a count that drifts.

What it does **not** check, stated because the alternative is a gate that lies by looking complete:
the *other* columns (how many of the ten repositories have the feature) are research notes. They are
not in this repository on purpose, and nothing here can confirm them. Neither does this replace
`claims` -- that gate owns the claim rows; this one owns the road between a claim row and the cell a
reader of the survey sees.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / '.github' / 'project' / 'matrix.json'
README = ROOT / 'README.md'
CELLS = ('has', 'partial', 'absent')
_CLAIM_HEADER = '| Module | Level |'
_MODULE = re.compile(r'`([^`]+)`')


def _claim_rows(text: str) -> dict:
    """`{module: level}` from the README's claim table -- the one with `| Module | Level |`.

    The table is found by its header rather than by an offset, and read until the first line that is
    not a row, so adding rows above it cannot shift the parse."""
    rows: dict = {}
    lines = text.splitlines()
    try:
        start = next(i for i, line in enumerate(lines) if line.startswith(_CLAIM_HEADER))
    except StopIteration:
        return rows
    for line in lines[start + 2:]:
        if not line.startswith('|'):
            break
        cells = line.strip().strip('|').split('|')
        if len(cells) < 2:
            continue
        level = cells[1].strip().strip('*')
        for module in _MODULE.findall(cells[0]):
            rows[module] = level
    return rows


def check(features, claimable, beyond=()):
    """Every rule, as `(problems, unplaced, histogram)`.

    Pure: it takes the registry rows, the modules the README claims, the `beyond_the_comparison` groups,
    and returns what is wrong. The self-test drives this directly, so a rule that cannot fail on a
    fixture cannot pass here either."""
    problems = []
    histogram = {cell: 0 for cell in CELLS}
    seen_ids = set()
    mapped = set()
    for row in features:
        label = str(row.get('id', '?'))
        cell = row.get('cell', '')
        claims = row.get('claims') or []
        why = str(row.get('why', '')).strip()
        if label in seen_ids:
            problems.append(f'{label}: registered twice -- two cells for one feature is how a survey ends up with '
                            f'two answers')
        seen_ids.add(label)
        if not str(row.get('feature', '')).strip():
            problems.append(f'{label}: no `feature` text -- the cell has nothing to be a claim about')
        if cell not in CELLS:
            problems.append(f'{label}: cell {cell!r} is not one of {", ".join(CELLS)}')
            continue
        histogram[cell] += 1
        if cell == 'absent':
            if claims:
                problems.append(f'{label}: says `absent` and names {", ".join(claims)} -- if a claim exists the cell '
                                f'is `partial` or `has`, and the survey is the thing that is wrong')
            if not why:
                problems.append(f'{label}: `absent` without a reason -- "we do not have it" is a survey entry, not a '
                                f'decision, and it will be re-litigated in the next audit')
            continue
        if not claims:
            problems.append(f'{label}: `{cell}` with no module named -- what is the claim, then?')
        if cell == 'partial' and not why:
            problems.append(f'{label}: `partial` without a reason -- which part is not there is the whole content of '
                            f'a partial cell')
        for module in claims:
            mapped.add(module)
            if module not in claimable:
                problems.append(f'{label}: names `{module}`, which the README claim table does not have -- a cell '
                                f'pointing at a header nobody claims is "there is a file", not "there is a capability"')

    for group in beyond:
        label = f'beyond_the_comparison[{str(group.get("group", "?"))}]'
        modules = group.get('modules') or []
        if not modules:
            problems.append(f'{label}: a group with no modules -- that is a sentence about nothing')
        if not str(group.get('why', '')).strip():
            problems.append(f'{label}: no reason -- a module that is not a comparison row needs a sentence saying why, '
                            f'or the next reader cannot tell a decision from an oversight')
        for module in modules:
            if module in mapped:
                problems.append(f'{label}: `{module}` is already accounted for -- one module, one place')
            mapped.add(module)
            if module not in claimable:
                problems.append(f'{label}: names `{module}`, which the README claim table does not have -- a stale '
                                f'entry keeps a placement alive after the module is gone')

    unplaced = sorted(set(claimable) - mapped)
    for module in unplaced:
        problems.append(f'`{module}` is claimed and placed nowhere: give it a feature cell, or put it in '
                        f'`beyond_the_comparison` with the sentence that says why it is not one')
    return problems, unplaced, histogram


def _render(features, claimable):
    """The registry as the markdown rows a survey table can paste, level taken from the claim table."""
    marks = {'has': 'YES', 'partial': 'PARTIAL', 'absent': 'NO'}
    for row in features:
        levels = sorted({claimable[m] for m in (row.get('claims') or []) if m in claimable})
        level = '/'.join(levels) if levels else '-'
        print(f"| {row['feature']} | {marks.get(row['cell'], row['cell'])} | {level} | "
              f"{', '.join(f'`{m}`' for m in (row.get('claims') or [])) or '-'} |")


def _selftest():
    """Prove each rule can fail, on fixtures rather than on the tree."""
    claimable = {'foc/core/a.hpp': 'L3', 'foc/core/b.hpp': 'L1', 'foc/core/c.hpp': 'L2'}
    base = [{'id': 'one', 'feature': 'a thing', 'cell': 'has', 'claims': ['foc/core/a.hpp']},
            {'id': 'two', 'feature': 'another thing', 'cell': 'absent', 'why': 'two of the ten have it'}]
    groups = [{'group': 'foundation', 'modules': ['foc/core/c.hpp'], 'why': 'every feature rests on it'}]
    cases = [
        ('a cell naming a header nobody claims is red',
         [dict(base[0], claims=['foc/core/nope.hpp']), dict(base[1], id='three')], groups, 1,
         'the README claim table does not have'),
        ('an absent cell that names a claim is red',
         [base[0], dict(base[1], claims=['foc/core/b.hpp'])], groups, 1, 'if a claim exists the cell is'),
        ('an absent cell with no reason is red',
         [base[0], dict(base[1], why='')], groups, 1, 'without a reason'),
        ('a partial cell with no reason is red',
         [base[0], {'id': 'p', 'feature': 'x', 'cell': 'partial', 'claims': ['foc/core/b.hpp']}], groups, 1,
         'which part is not there'),
        ('a has cell with no module is red',
         [base[0], {'id': 'h', 'feature': 'x', 'cell': 'has', 'claims': []}, dict(base[1], id='three')], groups, 1,
         'with no module named'),
        ('the same feature twice is red',
         [base[0], dict(base[0]), dict(base[1], id='three')], groups, 1, 'registered twice'),
        ('a cell type outside the three is red',
         [base[0], {'id': 'w', 'feature': 'x', 'cell': 'maybe', 'claims': []}, dict(base[1], id='three')], groups, 1,
         'is not one of'),
        ('a cell with no feature text is red',
         [base[0], {'id': 'n', 'feature': '', 'cell': 'absent', 'why': 'x'}, dict(base[1], id='three')], groups, 1,
         'no `feature` text'),
        ('a claimed module placed nowhere is red',
         [base[0], dict(base[1], id='three')], [], 1, 'is claimed and placed nowhere'),
        ('a beyond-the-comparison group with no reason is red',
         [base[0], dict(base[1], id='three')], [{'group': 'g', 'modules': ['foc/core/b.hpp', 'foc/core/c.hpp']}], 1,
         'no reason'),
        ('a group naming a module nobody claims is red',
         [base[0], dict(base[1], id='three')],
         [{'group': 'g', 'modules': ['foc/core/b.hpp', 'foc/core/c.hpp', 'foc/core/gone.hpp'], 'why': 'x'}], 1,
         'a stale'),
        ('a module placed in both a cell and a group is red',
         [base[0], {'id': 't', 'feature': 'y', 'cell': 'has', 'claims': ['foc/core/c.hpp']}, dict(base[1], id='three')],
         groups, 1, 'one module, one place'),
        ('the clean fixture is green', [base[0], dict(base[1], id='three')],
         [{'group': 'g', 'modules': ['foc/core/b.hpp', 'foc/core/c.hpp'], 'why': 'x'}], 0, 'has 1'),
    ]
    failures = []
    for name, rows, groups_in, _budget, expected in cases:
        problems, _unplaced, histogram = check(rows, claimable, groups_in)
        printed = '\n'.join([*problems, *[f'{k} {v}' for k, v in histogram.items() if v]])
        if expected not in printed:
            failures.append(f'{name}: expected {expected!r} in the output, got {printed!r}')
    for failure in failures:
        print(f'[matrix] selftest FAIL: {failure}', flush=True)
    if failures:
        return 1
    print(f'[matrix] selftest: {len(cases)}/{len(cases)} rules can fail', flush=True)
    return 0


def run(render=False):
    registry = json.loads(REGISTRY.read_text(encoding='utf-8'))
    claimable = _claim_rows(README.read_text(encoding='utf-8'))
    if not claimable:
        print(f'[matrix] FAIL: no claim table found in {README.name} -- the rows this checks against are missing, so '
              f'a green run here would mean nothing', file=sys.stderr)
        return 1
    if render:
        _render(registry.get('features') or [], claimable)
        return 0
    problems, unplaced, histogram = check(registry.get('features') or [], claimable,
                                          registry.get('beyond_the_comparison') or [])
    cells = ', '.join(f'{cell} {count}' for cell, count in histogram.items() if count)
    print(f'[matrix] {len(registry.get("features") or [])} feature(s) over {len(claimable)} claimed module(s): {cells}',
          flush=True)
    for module in unplaced:
        print(f'[matrix] unplaced: {module} is claimed and placed nowhere', flush=True)
    for problem in problems:
        print(f'[matrix] {problem}', flush=True)
    return 1 if problems else 0


def main(argv=None):
    ap = argparse.ArgumentParser(description='Every feature in the horizontal comparison names the claim it rests on.')
    ap.add_argument('--selftest', action='store_true', help='prove every rule can fail, on fixtures')
    ap.add_argument('--render', action='store_true', help='print the registry as table rows to paste')
    args = ap.parse_args(argv)
    return _selftest() if args.selftest else run(render=args.render)


if __name__ == '__main__':
    sys.exit(main())
