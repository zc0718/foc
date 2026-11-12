#!/usr/bin/env python3
# Copyright (c) 2026 HeT-AI
# SPDX-License-Identifier: Apache-2.0
"""Every field of a per-tick input struct names where its value comes from.

The defect this exists for is not "a missing feature", it is "a green field nobody
fills". `ProtectionInput.temperature_c`, `ThermalLimitConfig`'s temperature and the
bus current were all, until 2026-11-12, quantities the application had to bring
along and for which this project offered no channel: the HAL had a DC link, a current
sense and a fault input, and no temperature at all. Nothing was red about that. Each
consumer's own tests pass -- they hand the field in themselves -- so the only party
who noticed was whoever had to write the integration, and they noticed by writing the
same thermistor conversion the fifth time.

So the question this asks is narrower than "is it implemented":

    for this field, where does the value come from, and does that place exist?

Five answers are allowed, and the fifth is the one that costs:

    contract  a declared contract names it    `hal/temperature.hpp::Temperature::read`
              (a hardware port under `hal/`, or an injected instrument under `core/` --
              the board-check bench is a contract too, and calling it a `hal` port would
              be a different lie in the other direction)
    host      a C entry hands it in           `foc_set_speed`
    derived   the library computes it from other fields
    constant  it is a number off a datasheet, not a measurement
    unfed     the application measures it and this project offers no channel

`unfed` is not a way to make the gate quiet -- it is the debt list. It prints on every
run with the batch that owns it, and `unfed_budget` ratchets it downward: a new `unfed`
row raises the count above the budget and fails, and paying one off without lowering the
budget also fails, because a budget that never falls is a number nobody re-reads.

Known limit, stated rather than discovered later: this can see that a field has no
channel, and it cannot see that a channel which exists is not wired to anything. That
second gap is real (the temperature contract landed in 8785ea7 with no assembly
point) and it belongs to the drive-supervisor batch, not here. Do not read a green
`ports` as "the numbers arrive".
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / '.github' / 'project' / 'ports.json'
API_HEADER = 'include/foc/api/foc_api.h'
KINDS = ('contract', 'host', 'derived', 'constant', 'unfed')
SUFFIXES = ('Input', 'Measurements')

# `struct ProtectionInput final {` at file scope. The suffix carries the meaning: these
# are the structs a caller fills every tick from what the hardware measured.
_STRUCT = re.compile(r'^struct\s+(\w+)\s+final\s*\{')
# A member of such a struct: `<type> name;` or `<type> name[3];`. `static constexpr` and
# member functions are not members that need a producer, and neither are comment lines.
_MEMBER = re.compile(r'^\s+(?!static\b|constexpr\b|using\b|auto\b|friend\b|template\b)'
                     r'[A-Za-z_][\w:<>,\s]*(?:\s+|\s*\*\s*)(\w+)\s*(?:\[[^\]]*\])?\s*;\s*$')


class Tree:
    """The facts a check may consult: which files exist, and what they say.

    Kept as one object so the self-test can hand over fixtures instead of the real
    repository, and so a rule can never quietly read the tree behind the test's back."""

    def __init__(self, files):
        self.files = files
        self.text = '\n'.join(files.values())

    @classmethod
    def of_repo(cls):
        files = {}
        for base in ('include', 'src'):
            for path in sorted((ROOT / base).rglob('*')):
                if path.suffix in {'.hpp', '.h', '.cpp', '.c', '.inc'}:
                    rel = path.relative_to(ROOT).as_posix()
                    files[rel] = path.read_text(encoding='utf-8', errors='replace')
        return cls(files)

    def has_path(self, rel):
        return rel in self.files

    def has_symbol(self, name):
        return re.search(rf'\b{re.escape(name)}\b', self.text) is not None

    def declares_in(self, rel, name):
        """Is `name` declared in exactly this file -- not merely somewhere in the tree?"""
        return re.search(rf'\b{re.escape(name)}\s*\(', self.files.get(rel, '')) is not None


def discover(corpus):
    """`(struct, field, rel_path, lineno)` for every in-scope member.

    `corpus` is `{relative path: text}`; the self-test passes a fixture and the real run
    passes `include/foc/core/**`. The body ends at the first `};` at column zero, which is
    where the struct's own brace closes -- members are indented, so a nested type would
    have to be indented too and would not terminate the scan early."""
    found = []
    for rel, text in sorted(corpus.items()):
        lines = text.splitlines()
        index = 0
        while index < len(lines):
            match = _STRUCT.match(lines[index])
            if not match or not match.group(1).endswith(SUFFIXES):
                index += 1
                continue
            struct = match.group(1)
            index += 1
            while index < len(lines):
                line = lines[index]
                index += 1
                if line.startswith('};'):
                    break
                member = _MEMBER.match(line)
                if member:
                    found.append((struct, member.group(1), rel, index))
    return found


def _producer_parts(row):
    return [part for part in str(row.get('producer', '')).split('::') if part]


def check(found, rows, budget, tree):
    """Every rule, as `(problems, debts, histogram)`.

    Pure: it takes the discovered fields, the registry rows, the budget and a `Tree`, and
    returns what is wrong. The self-test drives this directly, so a rule that cannot fail
    on a fixture cannot pass here either."""
    problems, debts = [], []
    histogram = {kind: 0 for kind in KINDS}
    by_key = {}
    for row in rows:
        key = (row.get('struct', ''), row.get('field', ''))
        if key in by_key:
            problems.append(f'{key[0]}::{key[1]}: registered twice')
        by_key[key] = row

    discovered = {(struct, field): (rel, lineno) for struct, field, rel, lineno in found}
    for key, (rel, lineno) in sorted(discovered.items()):
        if key not in by_key:
            problems.append(f'{rel}:{lineno}: {key[0]}::{key[1]} has no producer row -- so nobody can say where '
                            f'its value comes from, and no consumer can be asked')
    for key in sorted(by_key):
        if key not in discovered:
            problems.append(f'{key[0]}::{key[1]}: registered but no such field is in scope any more -- a stale '
                            f'row keeps a claim alive after the field is gone')

    for (struct, field), row in sorted(by_key.items()):
        kind = row.get('kind', '')
        label = f'{struct}::{field}'
        if kind not in KINDS:
            problems.append(f'{label}: kind {kind!r} is not one of {", ".join(KINDS)}')
            continue
        histogram[kind] += 1
        if not str(row.get('note', '')).strip():
            problems.append(f'{label}: no note -- a classification without its reason cannot be reviewed, and the '
                            f'next reader cannot tell a decision from a guess')
        parts = _producer_parts(row)
        if kind == 'contract':
            if len(parts) < 2:
                problems.append(f'{label}: a `contract` producer names `<header>::<Class>::<member>`, got '
                                f'{row.get("producer")!r}')
            elif not tree.has_path(f'include/foc/{parts[0]}'):
                problems.append(f'{label}: names {parts[0]}, which is not a header under include/foc/')
            else:
                for symbol in parts[1:]:
                    if not tree.has_symbol(symbol):
                        problems.append(f'{label}: names `{symbol}`, which appears nowhere in the tree -- a '
                                        f'contract that was planned is not a contract that exists')
        elif kind == 'host':
            name = parts[0] if parts else ''
            if not name:
                problems.append(f'{label}: a `host` producer names the C entry, got {row.get("producer")!r}')
            elif not tree.declares_in(API_HEADER, name):
                problems.append(f'{label}: names the C entry `{name}`, which `{API_HEADER}` does not declare')
        elif kind == 'derived':
            if not parts or not tree.has_path(f'include/foc/{parts[0]}'):
                problems.append(f'{label}: a `derived` producer names the header that computes it, got '
                                f'{row.get("producer")!r}')
        elif kind == 'constant':
            if parts:
                problems.append(f'{label}: `constant` takes no producer, got {row.get("producer")!r} -- a number '
                                f'off a datasheet is not produced by anything')
        elif kind == 'unfed':
            owner = str(row.get('owner', '')).strip()
            if not owner:
                problems.append(f'{label}: `unfed` must name the batch that will pay it off, or it is just a way '
                                f'to stop the gate talking')
            debts.append(f'{label} (owner {owner or "?"}): {row.get("note", "").strip()}')

    if len(debts) > budget:
        problems.append(f'{len(debts)} field(s) have no channel and the budget is {budget} -- bring one or raise '
                        f'the budget in the same change, with a reason: {", ".join(d.split(" (")[0] for d in debts)}')
    elif len(debts) < budget:
        problems.append(f'only {len(debts)} field(s) are unfed but the budget still says {budget} -- lower it to '
                        f'{len(debts)}, otherwise the ratchet only ever moves in the direction nobody notices')
    return problems, debts, histogram


def _scope_corpus():
    corpus = {}
    for path in sorted((ROOT / 'include' / 'foc' / 'core').rglob('*.hpp')):
        corpus[path.relative_to(ROOT).as_posix()] = path.read_text(encoding='utf-8')
    return corpus


def _report(problems, debts, histogram, found):
    for debt in debts:
        print(f'[ports] debt: {debt}', flush=True)
    kinds = ', '.join(f'{kind} {count}' for kind, count in histogram.items() if count)
    print(f'[ports] {len(found)} field(s) over {len({s for s, *_ in found})} struct(s): {kinds}', flush=True)
    for problem in problems:
        print(f'[ports] {problem}', flush=True)
    return problems


def run(budget=None):
    registry = json.loads(REGISTRY.read_text(encoding='utf-8'))
    found = discover(_scope_corpus())
    problems, debts, histogram = check(found, registry.get('fields', []),
                                       registry['unfed_budget'] if budget is None else budget, Tree.of_repo())
    return 1 if _report(problems, debts, histogram, found) else 0


def _selftest():
    """Prove each rule can fail, on fixtures rather than on the tree.

    A completeness checker is trivially green on a registry that has nothing in it, so the
    cases here are chosen for the failures that matter: the field that started this line,
    a contract that is named but does not exist, a row left behind by a deleted field, and
    both directions of the ratchet."""
    header = {'include/foc/hal/temperature.hpp': 'class Temperature {\n'
                                                '    virtual auto read() noexcept -> TemperatureReading = 0;\n};\n',
              API_HEADER: 'float foc_get_telemetry(foc_motor_t motor);\n',
              'include/foc/core/protection.hpp': 'struct ProtectionInput final {\n'
                                                 '    float v_bus_v;\n'
                                                 '    float temperature_c;\n};\n'}
    tree = Tree(header)
    found = discover(header)
    base = [{'struct': 'ProtectionInput', 'field': 'v_bus_v', 'kind': 'contract',
             'producer': 'hal/dc_link.hpp::DcLink::voltage_v', 'note': 'the DC link measures it'},
            {'struct': 'ProtectionInput', 'field': 'temperature_c', 'kind': 'contract',
             'producer': 'hal/temperature.hpp::Temperature::read', 'note': 'the winding channel'}]
    cases = [
        ('a missing row is red and names the field', base[:1], 0, 'ProtectionInput::temperature_c has no producer'),
        ('a contract that does not exist is red',
         [base[0], dict(base[1], producer='hal/temperature.hpp::Temperature::readall')], 0,
         'appears nowhere in the tree'),
        ('a row for a field that is gone is red',
         [*base, {'struct': 'ProtectionInput', 'field': 'torque_nm', 'kind': 'derived',
                  'producer': 'core/svpwm.hpp', 'note': 'x'}], 0, 'registered but no such field'),
        ('the same field twice is red', [*base, base[1]], 0, 'registered twice'),
        ('a row with no reason is red', [base[0], dict(base[1], note='')], 0, 'no note'),
        ('a host entry the API header does not declare is red',
         [dict(base[0], kind='host', producer='foc_wind_temperature'), base[1]], 0, 'does not declare'),
        ('an unfed field without an owner is red',
         [base[0], dict(base[1], kind='unfed', producer='')], 1, 'must name the batch'),
        ('an unfed field over budget is red',
         [base[0], dict(base[1], kind='unfed', producer='', owner='N3')], 0, 'and the budget is 0'),
        ('paying one off without lowering the budget is red',
         [base[0], dict(base[1], kind='unfed', producer='', owner='N3')], 2, 'lower it to 1'),
        ('contract flipped to constant is green and visible in the histogram',
         [base[0], dict(base[1], kind='constant', producer='')], 0, 'constant 1'),
        ('an invented header for a derived field is red',
         [base[0], dict(base[1], kind='derived', producer='core/nowhere.hpp')], 0, 'not a header'),
    ]
    failures = []
    for name, rows, budget, expected in cases:
        problems, debts, histogram = check(found, rows, budget, tree)
        printed = '\n'.join(problems) + ('\n' + ', '.join(f'{k} {v}' for k, v in histogram.items() if v)
                                         if expected == 'constant 1' else '')
        if expected not in printed:
            failures.append(f'{name}: expected {expected!r} in the output, got {printed!r}')
    for failure in failures:
        print(f'[ports] selftest FAIL: {failure}', flush=True)
    if failures:
        return 1
    print(f'[ports] selftest: {len(cases)}/{len(cases)} rules can fail', flush=True)
    return 0


def main(argv=None):
    parser = argparse.ArgumentParser(description='Every per-tick input field names a producer that exists.')
    parser.add_argument('--selftest', action='store_true', help='prove each rule can fail, on fixtures')
    args = parser.parse_args(argv)
    return _selftest() if args.selftest else run()


if __name__ == '__main__':
    sys.exit(main())
