#!/usr/bin/env python3
"""The FMU's two views of the same variable table must agree, and neither may drift on its own.

A value reference is the only thing joining `modelDescription.xml` (what a master reads) to
`fmu_wrapper.c` (what the code switches on). A mismatch is silent in both directions -- the master drives
`angle_mech_rad` and the wrapper answers a duty -- and no compiler can see it, because the two sides are a
number in XML and a number in an anonymous enum. That is the same shape as the defects this repository
keeps finding: wrong, stable, repeatable, and invisible.

So the table lives in `fmu_vars.json` and this check compares both readers to it:

    python3 api/fmu/modelDescription.xml_check.py
    python3 api/fmu/modelDescription.xml_check.py --selftest

The XML is compared by parsing it (names, value references, causality, variability, type) and the wrapper
by reading its enum, because the enum is what `fmi2SetReal`/`fmi2GetReal` index with. When the FMI 3.0
adapter arrives, its XML and its enum are checked against the same file -- that is the whole reason the
registry exists one step before there are two of anything.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
VARS = ROOT / 'api' / 'fmu' / 'fmu_vars.json'
XML = ROOT / 'api' / 'fmu' / 'modelDescription.xml'
WRAPPER = ROOT / 'api' / 'fmu' / 'fmu_wrapper.c'

SCALAR = re.compile(
    r'<ScalarVariable name="(?P<name>[^"]+)" valueReference="(?P<vr>\d+)" '
    r'causality="(?P<causality>[^"]+)" variability="(?P<variability>[^"]+)"[^>]*>\s*'
    r'<(?P<type>Real|Boolean|Integer|String)[^/>]*/?>'
)
ENUM = re.compile(r'\b(?P<key>k[A-Z][A-Za-z0-9]*)\s*=\s*(?P<value>\d+)')


def xml_rows(text: str) -> dict[str, dict]:
    return {match.group('name'): {
        'value_reference': int(match.group('vr')),
        'causality': match.group('causality'),
        'variability': match.group('variability'),
        'type': match.group('type'),
    } for match in SCALAR.finditer(text)}


def wrapper_enum(text: str) -> dict[str, int]:
    return {match.group('key'): int(match.group('value')) for match in ENUM.finditer(text)}


def compare(registry: dict, xml: dict[str, dict], enum: dict[str, int]) -> list[str]:
    """Pure: which of the three views disagree. `enum` maps the wrapper's own enumerator names to numbers."""
    problems: list[str] = []
    variables = registry['variables']
    by_name = {variable['name']: variable for variable in variables}

    for name, row in xml.items():
        if name not in by_name:
            problems.append(f'modelDescription.xml declares {name}, which fmu_vars.json does not have')
            continue
        want = by_name[name]
        for field in ('value_reference', 'causality', 'variability', 'type'):
            if row[field] != want[field]:
                problems.append(f'{name}: modelDescription.xml says {field}={row[field]!r}, '
                                f'fmu_vars.json says {want[field]!r}')
    for name in by_name:
        if name not in xml:
            problems.append(f'{name}: in fmu_vars.json but missing from modelDescription.xml')

    # The wrapper's enum is the third reader. Its *names* are the registry's `cpp_enum`, its numbers are the
    # value references -- so a renumbering on one side and not the other is caught here rather than in the field.
    for variable in variables:
        key = variable['cpp_enum']
        if key not in enum:
            problems.append(f'{variable["name"]}: fmu_wrapper.c has no enumerator {key}')
        elif enum[key] != variable['value_reference']:
            problems.append(f'{variable["name"]}: fmu_wrapper.c has {key} = {enum[key]}, '
                            f'fmu_vars.json says {variable["value_reference"]}')
    known = {variable['cpp_enum'] for variable in variables}
    for key, value in enum.items():
        if key not in known:
            problems.append(f'fmu_wrapper.c enumerator {key} = {value} is not in fmu_vars.json')

    references = [variable['value_reference'] for variable in variables]
    if len(set(references)) != len(references):
        problems.append('fmu_vars.json reuses a value reference')
    if sorted(references) != list(range(1, len(references) + 1)):
        problems.append(f'value references are not 1..{len(references)}: FMI allows any, but a contiguous run is '
                        f'the only version a reader can spot a hole in')
    return problems


def selftest(registry: dict) -> int:
    """Cases that must be caught, so a checker that has stopped working cannot pass by agreeing with itself."""
    xml = xml_rows(XML.read_text(encoding='utf-8'))
    enum = wrapper_enum(WRAPPER.read_text(encoding='utf-8'))
    failures = []

    if compare(registry, xml, enum):
        failures.append('false positive: the files as shipped')

    shifted = {name: dict(row) for name, row in xml.items()}
    shifted['angle_mech_rad']['value_reference'] += 1
    if not compare(registry, shifted, enum):
        failures.append('not caught: an XML value reference moved')

    swapped = dict(enum)
    swapped['kDutyU'], swapped['kDutyV'] = enum['kDutyV'], enum['kDutyU']
    if not compare(registry, xml, swapped):
        failures.append('not caught: two wrapper enumerators swapped, which is the silent case')

    dropped = {name: row for name, row in xml.items() if name != 'enable'}
    if not compare(registry, dropped, enum):
        failures.append('not caught: a variable removed from the XML')

    if failures:
        for line in failures:
            print(f'[fmu-vars] selftest FAIL: {line}', flush=True)
        return 1
    print('[fmu-vars] selftest OK (3 drift modes caught, the shipped files accepted)', flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--selftest', action='store_true', help='prove the checker fails on known-bad input')
    args = parser.parse_args()

    if not VARS.is_file() or not XML.is_file() or not WRAPPER.is_file():
        print(f'[fmu-vars] missing one of {VARS.name}, {XML.name}, {WRAPPER.name}', file=sys.stderr, flush=True)
        return 2

    registry = json.loads(VARS.read_text(encoding='utf-8'))
    if args.selftest:
        return selftest(registry)

    problems = compare(registry,
                       xml_rows(XML.read_text(encoding='utf-8')),
                       wrapper_enum(WRAPPER.read_text(encoding='utf-8')))
    if problems:
        for line in problems:
            print(f'[fmu-vars] {line}', flush=True)
        print(f'[fmu-vars] FAIL ({len(problems)} disagreement(s))', flush=True)
        return 1
    inputs = sum(1 for variable in registry['variables'] if variable['causality'] == 'input')
    outputs = len(registry['variables']) - inputs
    print(f'[fmu-vars] OK ({len(registry["variables"])} variable(s): {inputs} in, {outputs} out; '
          f'XML, wrapper enum and fmu_vars.json agree)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
