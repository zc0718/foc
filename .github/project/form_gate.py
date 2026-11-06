#!/usr/bin/env python3
"""Every shape this library ships in is registered, and its claim is either made or recorded as a gap.

The failure this prevents has already happened here once: the FMU was built, tested and listed in the
plan while no claim row described it, so "we ship an FMU" was true and "what does the FMU promise" had
no answer -- and nothing went red, because the absence of a row is not an error to any other gate. A
second failure is the opposite one: a form is added to the tree and never registered, so it inherits no
obligation at all.

So the unit of checking is the **face**, not the file: the gate derives the faces from the tree (the
umbrella headers, the C ABI, the FMU description, the bindings module), and refuses

  1. a face present in the tree that `forms.json` does not register,
  2. a registered face whose entry point or evidence does not exist,
  3. a registered face with neither `claim_status: claimed` (pointing at README rows) nor `claim_status:
     gap` (with a reason and what would close it).

A `gap` is a passing state on purpose. The point of this gate is that the gap is *recorded and
inspectable*, not that it is red -- a red light for a known, written-down debt is a light people learn
to ignore, and then it cannot warn about the new one.

    python3 .github/project/form_gate.py            # what CI runs
    python3 .github/project/form_gate.py --selftest # prove the checker can still fail
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
FORMS = ROOT / '.github' / 'project' / 'forms.json'
README = ROOT / 'README.md'

sys.path.insert(0, str(Path(__file__).resolve().parent))

from claim_gate import claim_rows  # noqa: E402  (one README parse, one owner of it)

# The faces, derived from the tree rather than listed: each is "if this file exists, that shape ships".
# A new shape means a new entry here *and* an entry in forms.json; the pair is what makes it impossible
# to add a face without stating its obligation.
FACES = {
    'cpp_headers': 'include/foc/motor/motor.hpp',
    'c_abi': 'include/foc/api/foc_api.h',
    'fmu': 'api/fmu/modelDescription.xml',
    'python': 'api/bindings_absent_marker.cpp',  # replaced below when the binding source exists
}
PYTHON_ENTRY = 'api/python_bindings.cpp'


def present_faces() -> dict[str, str]:
    faces = {}
    for name, path in FACES.items():
        real = PYTHON_ENTRY if name == 'python' else path
        if (ROOT / real).is_file():
            faces[name] = real
    return faces


def _row_keys() -> set[str]:
    """The module cells of README's claim table, as `claim_gate` reads them."""
    return {module for modules, _level, _claim, _evidence in claim_rows() for module in modules}


def check(forms: dict, faces: dict[str, str], readme: str) -> list[str]:
    """Pure: given the registry, the faces found and README's text, return the problems."""
    problems: list[str] = []
    registered = {form['id']: form for form in forms['forms']}

    for name in faces:
        if name not in registered:
            problems.append(f'{name}: this face exists in the tree ({faces[name]}) but forms.json does not '
                            f'register it, so it carries no obligation')
    for name, form in registered.items():
        if name not in faces:
            problems.append(f'{name}: registered as a shipped form, but its entry point is missing from the tree')
            continue
        if not (ROOT / form['entry']).is_file():
            problems.append(f'{name}: entry point {form["entry"]} does not exist')
        for item in form.get('evidence', []):
            if not (ROOT / item).is_file():
                problems.append(f'{name}: evidence {item} does not exist')
        status = form.get('claim_status')
        if status == 'claimed':
            claimed_by = form.get('claimed_by')
            if not claimed_by:
                problems.append(f'{name}: says its claim is made but does not say by which README rows')
            else:
                rows = _row_keys()
                if claimed_by == 'all_hpp_rows':
                    # A rule, not a list: "every .hpp row" cannot go stale, and a count of zero means the
                    # rule is satisfied by nothing -- which is the failure, not a pass.
                    if not any(row.endswith('.hpp') for row in rows):
                        problems.append(f'{name}: claims are made by all_hpp_rows, but no .hpp row exists')
                elif isinstance(claimed_by, list):
                    missing = [row for row in claimed_by if row not in rows]
                    if missing:
                        problems.append(f'{name}: claims are made by {missing}, which README has no row for')
                else:
                    problems.append(f'{name}: claimed_by must be "all_hpp_rows" or a list of row keys')
        elif status == 'gap':
            if not form.get('gap'):
                problems.append(f'{name}: recorded as a gap without a reason, which is indistinguishable '
                                f'from forgetting to write one')
        else:
            problems.append(f'{name}: claim_status is {status!r}; every form is either claimed or a recorded gap')
    return problems


def selftest(forms: dict, faces: dict[str, str], readme: str) -> int:
    """Cases that must be caught. A checker that has stopped being able to fail passes by finding nothing."""
    import copy

    def broken(mutate) -> list[str]:
        trial = copy.deepcopy(forms)
        mutate(trial)
        return check(trial, faces, readme)

    failures = []
    if not broken(lambda f: f['forms'].pop(0)):
        failures.append('not caught: an unregistered face (a form removed from forms.json)')
    if not broken(lambda f: f['forms'][0].__setitem__('entry', 'include/foc/does_not_exist.hpp')):
        failures.append('not caught: a registered form whose entry point is gone')
    if not broken(lambda f: f['forms'][0].__setitem__('evidence', ['test_package/test/unit/nope.cpp'])):
        failures.append('not caught: evidence that does not exist')
    if not broken(lambda f: [form.update(claim_status='gap', gap='') for form in f['forms'] if form['id'] == 'fmu']):
        failures.append('not caught: a gap with no reason')
    if not broken(lambda f: [form.update(claim_status='pending') for form in f['forms'] if form['id'] == 'c_abi']):
        failures.append('not caught: a claim_status outside the two that mean something')
    if broken(lambda f: None):
        failures.append('false positive: the registry as shipped')

    if failures:
        for line in failures:
            print(f'[form] selftest FAIL: {line}', flush=True)
        return 1
    print('[form] selftest OK (5 failure modes caught, the shipped registry accepted)', flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--selftest', action='store_true', help='prove the checker still fails on known-bad input')
    args = parser.parse_args()

    forms = json.loads(FORMS.read_text(encoding='utf-8'))
    faces = present_faces()
    readme = README.read_text(encoding='utf-8')
    if args.selftest:
        return selftest(forms, faces, readme)

    problems = check(forms, faces, readme)
    if problems:
        for line in problems:
            print(f'[form] {line}', flush=True)
        print(f'[form] FAIL ({len(problems)} problem(s))', flush=True)
        return 1
    claimed = [form['id'] for form in forms['forms'] if form['claim_status'] == 'claimed']
    gaps = [form['id'] for form in forms['forms'] if form['claim_status'] == 'gap']
    print(f'[form] OK ({len(faces)} face(s) in the tree, {len(forms["forms"])} registered; '
          f'claimed: {", ".join(claimed) or "none"}; recorded gap: {", ".join(gaps) or "none"})', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
