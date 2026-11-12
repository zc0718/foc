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
    duck      an injected object the header takes as a template parameter, so there
              is no method to name           `core/board_check.hpp::Bench`
    unfed     the application measures it and this project offers no channel

`unfed` is not a way to make the gate quiet -- it is the debt list. It prints on every
run with the batch that owns it, and `unfed_budget` ratchets it downward: a new `unfed`
row raises the count above the budget and fails, and paying one off without lowering the
budget also fails, because a budget that never falls is a number nobody re-reads.

The same question is asked of the numbers that arrive through a `step()` signature, and
for the same reason. A parameter is a number the caller hands in, so "is it fed" is
settled by the signature; what is not settled is **whether the caller could have obtained
it from this library at all**. `BusCurrentLimit::step(float bus_current_a, float dt_s)`
was written before anything in the tree could measure bus current: the limiter was
correct, its tests passed (they hand it a number), and the quantity had no source. That is
the same green field as an unfilled struct member, invisible for the same reason -- the
axis read struct bodies and a parameter is not in one. Measured when this rule was added:
`bus_current` appeared **zero** times in this registry while being the argument that
matters most to the newest component in `core/`.

Scope, stated as a rule rather than a list: **a scalar `step()` parameter whose name
carries a unit suffix**, because the suffix is this tree's own way of saying "this is a
physical quantity" (`dt_s` excepted -- the tick length is the host's clock, and `clocks`
owns it). A parameter named without a unit (`setpoint`, `feedback`, `target`, `volts`) is
read here as a *role* rather than a quantity, and its provenance is the caller's by
construction. That line has a visible wart: `PpmCommand::step(float width_us, ...)` is in
scope while `AnalogCommand::step(float volts)` is not, on nothing but their names. So the
parameters that are out of scope are printed on every run -- a clean result must not be
read as "every command input has a producer".

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
KINDS = ('contract', 'host', 'derived', 'constant', 'duck', 'unfed')
SUFFIXES = ('Input', 'Measurements')
# A parameter is in scope when its *name* says it is a quantity. The list is the suffixes the tree
# already uses, so the rule reads as "the convention this repository writes to", not as a list of
# the parameters somebody remembered: `_s` is deliberately absent (it would catch `dt_s`), and
# `_rad` is listed before `_rad_s` only for readability -- `str.endswith` tries each in turn.
PARAM_SUFFIXES = ('_a', '_v', '_bus', '_c', '_nm', '_wb', '_ohm', '_rad', '_rad_s', '_rad_s2', '_us', '_hz')
SCALAR_TYPES = {'float', 'double', 'int', 'bool', 'std::int64_t', 'std::uint8_t', 'std::uint32_t',
                'std::int32_t', 'std::size_t'}

# `struct ProtectionInput final {` at file scope. The suffix carries the meaning: these
# are the structs a caller fills every tick from what the hardware measured.
_STRUCT = re.compile(r'^struct\s+(\w+)\s+final\s*\{')
# A member of such a struct: `<type> name;` or `<type> name[3];`. `static constexpr` and
# member functions are not members that need a producer, and neither are comment lines.
_MEMBER = re.compile(r'^\s+(?!static\b|constexpr\b|using\b|auto\b|friend\b|template\b)'
                     r'[A-Za-z_][\w:<>,\s]*(?:\s+|\s*\*\s*)(\w+)\s*(?:\[[^\]]*\])?\s*;\s*$')
# `class BrakeChopper final {` at file scope: the owner of any `step()` that follows, until the
# class's own brace closes at column zero. Same convention as the struct scan, same limitation.
_CLASS = re.compile(r'^class\s+(\w+)\s+final\s*\{')
# `auto step(<params>)`, inline or a declaration. A default argument containing `)` would truncate
# the list, and nothing in the tree has one; the count printed below is what would expose it.
_STEP = re.compile(r'\bauto\s+step\s*\(([^)]*)\)')


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

    def accepts_as_parameter(self, rel, name):
        """Does `this` file take `name` as a template parameter -- a duck-typed contract?"""
        return re.search(rf'template\s*<[^>]*\b{re.escape(name)}\b', self.files.get(rel, '')) is not None


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


def _kind_problems(label, row, tree):
    """What the five kinds each demand of one row, as `(problems, debt, kind)`.

    Shared by both axes on purpose: a `contract` in the parameter table is a contract in exactly the
    same sense as one in the field table, so the rule that makes it falsifiable cannot drift between
    them. The label carries whichever axis is speaking, so the message reads as one sentence on its
    own."""
    problems = []
    kind = row.get('kind', '')
    if kind not in KINDS:
        return [f'{label}: kind {kind!r} is not one of {", ".join(KINDS)}'], None, kind
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
            # Where it is declared, not just that the word exists: the claim a `contract` row makes is
            # "this header offers this channel". Measured on the first version of this rule, which only
            # asked whether the name appeared *anywhere* -- so a method moved to a different header left
            # the row pointing at the old one and stayed green.
            if not tree.declares_in(f'include/foc/{parts[0]}', parts[-1]):
                problems.append(f'{label}: names `{parts[-1]}`, which `{parts[0]}` does not declare -- a row '
                                f'pointing at a header that no longer offers it is a stale pointer, however '
                                f'many other files still say the word')
    elif kind == 'duck':
        if len(parts) < 2:
            problems.append(f'{label}: a `duck` producer names `<header>::<TemplateParameter>`, got '
                            f'{row.get("producer")!r}')
        elif not tree.has_path(f'include/foc/{parts[0]}'):
            problems.append(f'{label}: names {parts[0]}, which is not a header under include/foc/')
        elif not tree.accepts_as_parameter(f'include/foc/{parts[0]}', parts[-1]):
            problems.append(f'{label}: names `{parts[-1]}`, which `{parts[0]}` does not take as a template '
                            f'parameter -- the injected object this row promises no longer fits')
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
        return problems, f'{label} (owner {owner or "?"}): {row.get("note", "").strip()}', kind
    return problems, None, kind


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
        label = f'{struct}::{field}'
        row_problems, debt, kind = _kind_problems(label, row, tree)
        problems.extend(row_problems)
        if kind in KINDS:
            histogram[kind] += 1
        if debt:
            debts.append(debt)

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


def discover_parameters(corpus):
    """`(class, parameter, rel_path, lineno)` for every in-scope `step()` parameter.

    Returns the out-of-scope ones too, as `(class, parameter)`, because the caller prints them: the
    rule that decides scope is a naming convention, and a convention has edges. The self-test passes
    a fixture, the real run passes `include/foc/core/**`.

    A parameter is in scope when the owning class is a `class ... final` at file scope (the same
    convention the struct scan relies on), its declared type is a scalar, and its name ends in one of
    `PARAM_SUFFIXES`. `dt_s` is refused by name: every step has one, it is the host's clock rather
    than a measurement, and a row per class saying "the tick length" would bury the rows that matter.
    """
    found, unsigned = [], []
    for rel, text in sorted(corpus.items()):
        owner = None
        for lineno, line in enumerate(text.splitlines(), start=1):
            name = _CLASS.match(line)
            if name:
                owner = name.group(1)
                continue
            if line.startswith('};'):
                owner = None
                continue
            call = _STEP.search(line)
            if not call or owner is None:
                continue
            for raw in call.group(1).split(','):
                words = re.findall(r'[A-Za-z_]\w*', re.sub(r'/\*.*?\*/', ' ', raw))
                if len(words) < 2:
                    continue  # an unnamed parameter: nothing to ask about
                parameter, declared = words[-1], ' '.join(words[:-1]).replace('const', '').strip()
                if declared not in SCALAR_TYPES:
                    continue
                if parameter == 'dt_s':
                    continue
                if parameter.endswith(PARAM_SUFFIXES):
                    found.append((owner, parameter, rel, lineno))
                elif (owner, parameter) not in unsigned:
                    unsigned.append((owner, parameter))
    return found, unsigned


def check_parameters(found, rows, budget, tree):
    """The parameter axis, as `(problems, debts, histogram)`. Same rules, same shape as `check`."""
    problems, debts = [], []
    histogram = {kind: 0 for kind in KINDS}
    by_key = {}
    for row in rows:
        key = (row.get('class', ''), row.get('parameter', ''))
        if key in by_key:
            problems.append(f'{key[0]}::step({key[1]}): registered twice')
        by_key[key] = row

    discovered = {(owner, parameter): (rel, lineno) for owner, parameter, rel, lineno in found}
    for key, (rel, lineno) in sorted(discovered.items()):
        if key not in by_key:
            problems.append(f'{rel}:{lineno}: {key[0]}::step({key[1]}) has no producer row -- so the quantity '
                            f'arrives through a signature nobody has asked where it comes from')
    for key in sorted(by_key):
        if key not in discovered:
            problems.append(f'{key[0]}::step({key[1]}): registered but no such parameter is in scope any more -- a '
                            f'stale row keeps a claim alive after the parameter is gone')

    for (owner, parameter), row in sorted(by_key.items()):
        label = f'{owner}::step({parameter})'
        row_problems, debt, kind = _kind_problems(label, row, tree)
        problems.extend(row_problems)
        if kind in KINDS:
            histogram[kind] += 1
        if debt:
            debts.append(debt)

    if len(debts) > budget:
        problems.append(f'{len(debts)} parameter(s) name a quantity with no channel and the budget is {budget} -- '
                        f'bring one or raise `unfed_parameter_budget` in the same change, with a reason: '
                        f'{", ".join(d.split(" (")[0] for d in debts)}')
    elif len(debts) < budget:
        problems.append(f'only {len(debts)} parameter(s) are unfed but the budget still says {budget} -- lower it to '
                        f'{len(debts)}, otherwise the ratchet only ever moves in the direction nobody notices')
    return problems, debts, histogram


def _report(problems, debts, histogram, found, params=None):
    for debt in debts:
        print(f'[ports] debt: {debt}', flush=True)
    kinds = ', '.join(f'{kind} {count}' for kind, count in histogram.items() if count)
    print(f'[ports] {len(found)} field(s) over {len({s for s, *_ in found})} struct(s): {kinds}', flush=True)
    if params is not None:
        found_params, unsigned, param_debts, param_histogram = params
        kind_text = ', '.join(f'{kind} {count}' for kind, count in param_histogram.items() if count)
        print(f'[ports] {len(found_params)} step() parameter(s) over '
              f'{len({c for c, *_ in found_params})} class(es): {kind_text or "none"}', flush=True)
        for debt in param_debts:
            print(f'[ports] debt: {debt}', flush=True)
        if unsigned:
            # Not a defect: the scope rule reads a name without a unit as a role. Printed because a
            # rule's edge is where the next reader's question will be.
            print(f'[ports] out of scope (no unit in the name): '
                  f'{", ".join(f"{c}::{p}" for c, p in unsigned)}', flush=True)
    for problem in problems:
        print(f'[ports] {problem}', flush=True)
    return problems


def run(budget=None, param_budget=None):
    registry = json.loads(REGISTRY.read_text(encoding='utf-8'))
    tree = Tree.of_repo()
    corpus = _scope_corpus()
    found = discover(corpus)
    problems, debts, histogram = check(found, registry.get('fields', []),
                                       registry['unfed_budget'] if budget is None else budget, tree)
    found_params, unsigned = discover_parameters(corpus)
    param_problems, param_debts, param_histogram = check_parameters(
        found_params, registry.get('parameters', []),
        registry['unfed_parameter_budget'] if param_budget is None else param_budget, tree)
    problems.extend(param_problems)
    _report(problems, debts, histogram, found, (found_params, unsigned, param_debts, param_histogram))
    return 1 if problems else 0


def _selftest():
    """Prove each rule can fail, on fixtures rather than on the tree.

    A completeness checker is trivially green on a registry that has nothing in it, so the
    cases here are chosen for the failures that matter: the field that started this line,
    a contract that is named but does not exist, a row left behind by a deleted field, and
    both directions of the ratchet."""
    header = {'include/foc/hal/temperature.hpp': 'class Temperature {\n'
                                                '    virtual auto read() noexcept -> TemperatureReading = 0;\n};\n',
              # `voltage_v` is mentioned here but not declared: the fixture for "the word is in the tree,
              # the header no longer offers it", which the first version of the contract rule accepted.
              'include/foc/hal/dc_link.hpp': 'class DcLink {\n'
                                             '    virtual auto voltage_v() noexcept -> float = 0;\n};\n',
              'include/foc/hal/board_power.hpp': '// voltage_v now lives on the board object.\n',
              'include/foc/core/board_check.hpp': 'template <typename Bench> struct Check {};\n',
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
         [base[0], dict(base[1], kind='derived', producer='core/nowhere.hpp')], 0,
         'a `derived` producer names the header that computes it'),
        ('a contract whose header no longer declares it is red, though the word survives elsewhere',
         [dict(base[0], producer='hal/board_power.hpp::voltage_v'), base[1]], 0, 'does not declare'),
        ('a duck-typed contract whose template parameter is gone is red',
         [base[0], dict(base[1], kind='duck', producer='core/protection.hpp::Bench')], 0,
         'does not take as a template parameter'),
        ('a duck-typed contract its header does accept is green and counted',
         [base[0], dict(base[1], kind='duck', producer='core/board_check.hpp::Bench')], 0, 'duck 1'),
    ]
    failures = []
    for name, rows, budget, expected in cases:
        problems, debts, histogram = check(found, rows, budget, tree)
        printed = '\n'.join([*problems, *[f'{k} {v}' for k, v in histogram.items() if v]])
        if expected not in printed:
            failures.append(f'{name}: expected {expected!r} in the output, got {printed!r}')

    # The parameter axis. Same five kinds, so the cases here are the ones its *scope rule* creates on
    # top of the shared rules: a quantity that arrives through a signature, a name without a unit
    # (which a registry may not claim), and `dt_s`, which is out of scope by name rather than by
    # accident -- a row for it must be refused, or the axis would collect one per class.
    param_header = {'include/foc/core/limiter.hpp':
                    'class Limiter final {\n'
                    '    auto step(float bus_current_a, float dt_s) noexcept -> float { return 0.0F; }\n'
                    '    auto step(float setpoint, float dt_s) noexcept -> float { return 0.0F; }\n'
                    '};\n'}
    param_tree = Tree({**param_header,
                       'include/foc/core/traits.hpp': 'bool read_bus_current_a(float& a);\n',
                       'include/foc/hal/position.hpp': 'virtual auto velocity_rad_s() noexcept -> float = 0;\n'})
    param_found, param_unsigned = discover_parameters(param_header)
    param_base = [{'class': 'Limiter', 'parameter': 'bus_current_a', 'kind': 'contract',
                   'producer': 'core/traits.hpp::read_bus_current_a', 'note': 'the optional DC-link channel'}]
    param_cases = [
        ('a parameter with no row is red',
         [], 0, 'Limiter::step(bus_current_a) has no producer'),
        ('a parameter row whose producer symbol is gone is red',
         [dict(param_base[0], producer='core/traits.hpp::read_bus_current')], 0, 'appears nowhere in the tree'),
        ('a row for a parameter outside the scope rule is red',
         [*param_base, {'class': 'Limiter', 'parameter': 'setpoint', 'kind': 'contract',
                        'producer': 'core/traits.hpp::read_bus_current_a', 'note': 'x'}], 0,
         'Limiter::step(setpoint): registered but no such parameter'),
        ('a row for `dt_s` is red even though the signature has one',
         [*param_base, {'class': 'Limiter', 'parameter': 'dt_s', 'kind': 'host',
                        'producer': 'foc_get_telemetry', 'note': 'x'}], 0, 'Limiter::step(dt_s): registered'),
        ('an unfed parameter over budget is red',
         [dict(param_base[0], kind='unfed', producer='', owner='a capture HAL')], 0,
         'and the budget is 0'),
        ('a parameter paid off without lowering the budget is red',
         [dict(param_base[0], kind='unfed', producer='', owner='a capture HAL')], 2, 'lower it to 1'),
    ]
    for name, rows, budget, expected in param_cases:
        problems, _debts, histogram = check_parameters(param_found, rows, budget, param_tree)
        printed = '\n'.join([*problems, *[f'{k} {v}' for k, v in histogram.items() if v]])
        if expected not in printed:
            failures.append(f'{name}: expected {expected!r} in the output, got {printed!r}')
    if param_unsigned != [('Limiter', 'setpoint')]:
        failures.append(f'the scope rule must report the names it left out, got {param_unsigned!r}')
    cases = cases + param_cases
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
