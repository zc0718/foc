#!/usr/bin/env python3
"""Where a third-party package may appear, and whether the ones we declare are actually used.

Two failures, one file, because they are the same failure seen from two sides.

1. **A dependency that reaches a consumer.** A public header is paid for by every consumer TU, on every
   platform, forever. Measured on 2026-11-06, `etl/crc16_ccitt.h` costs a consumer 277 headers and
   242 ms on top of the whole `foc` include face (18.7x). The rule that follows is not a style preference:
   a package appears only on the paths `deps.json` allows for it, and `include/**` allows none of them.

2. **A dependency that reaches nobody.** ETL sat in `conandata.yml` and `metadata.json` with zero
   references for months: every consumer carried a build dependency that produced nothing. That state is
   invisible to every other gate -- nothing is broken, so nothing goes red -- and it is exactly the state
   a compliance reviewer reads as "we did not build this from scratch after all".

So both directions are checked: an include outside its allowed paths fails, and a declared package with
no usage fails. `--selftest` runs first: a classifier that has lost the ability to say "violation" would
otherwise pass by finding nothing to classify.

    python3 .github/project/deps_gate.py             # what CI runs
    python3 .github/project/deps_gate.py --selftest  # prove the checker can still fail
"""
from __future__ import annotations

import argparse
import fnmatch
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEPS = ROOT / '.github' / 'project' / 'deps.json'
CONANDATA = ROOT / 'conandata.yml'
# The two registries this gate owns, and the page a doc-side check renders from them. The page is
# the site's artifact, not this gate's input: `render_check.py` (run by the `docs` gate) is what
# asks whether it is current, so this gate runs with no documentation tree at all.
AUDIT = ROOT / '.github' / 'project' / 'reuse-audit.json'
PAGE = ROOT / 'docs' / 'sphinx' / 'source' / 'third-party.rst'

# Everywhere a dependency could be included from. Build trees are excluded by SKIP_PARTS: they hold
# generated copies whose include lines say nothing about what we ship.
SCAN_DIRS = ['include', 'src', 'api', 'benchmark', 'test_package']
SKIP_PARTS = {'build', 'CMakeFiles', '.git', '__pycache__'}
SUFFIXES = {'.h', '.hpp', '.c', '.cpp', '.cc', '.ixx', '.cppm'}

INCLUDE = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')


def load() -> dict:
    return json.loads(DEPS.read_text(encoding='utf-8'))


def declared_in_conandata() -> dict[str, str]:
    """`name -> version` from conandata.yml, without PyYAML (the gates must not gain a dependency)."""
    versions: dict[str, str] = {}
    for line in CONANDATA.read_text(encoding='utf-8').splitlines():
        match = re.match(r'\s*-\s*"?([A-Za-z0-9_.+-]+)/([^"@\s]+)"?', line)
        if match:
            versions[match.group(1)] = match.group(2)
    return versions


def iter_sources() -> list[Path]:
    files: list[Path] = []
    for directory in SCAN_DIRS:
        base = ROOT / directory
        if not base.is_dir():
            continue
        for path in sorted(base.rglob('*')):
            if path.suffix in SUFFIXES and not (SKIP_PARTS & set(path.parts)):
                files.append(path)
    return files


def _matches(rel: str, pattern: str) -> bool:
    """`src/**` must match `src/comm/a.cpp`; fnmatch's `*` crosses separators, which is what we want."""
    if pattern.endswith('/**'):
        return rel == pattern[:-3] or rel.startswith(pattern[:-2])
    return fnmatch.fnmatch(rel, pattern)


def classify(rel: str, includes: list[str], deps: dict) -> list[str]:
    """Pure: given a repo-relative path and the include spellings it uses, return the violations.

    Kept pure and separate from the filesystem so `--selftest` can feed it cases that must be caught and
    cases that must not, without writing files into the tree.
    """
    problems: list[str] = []
    owners = {prefix: name for name, entry in deps['dependencies'].items()
              for prefix in entry.get('include_prefixes', [])}
    for spelling in includes:
        owner = next((name for prefix, name in owners.items() if spelling.startswith(prefix)), None)
        if owner is None:
            continue
        entry = deps['dependencies'][owner]
        forbidden = [p for p in entry.get('forbidden_paths', []) if _matches(rel, p)]
        if forbidden:
            problems.append(f'{rel}: includes <{spelling}>, but {owner} is forbidden under {forbidden[0]}')
            continue
        if not any(_matches(rel, allowed) for allowed in entry.get('allowed_paths', [])):
            problems.append(f'{rel}: includes <{spelling}>, but {owner} is not allowed here '
                            f'(allowed: {", ".join(entry.get("allowed_paths", [])) or "nowhere"})')
    return problems


def unknown_packages(deps: dict, declared: dict[str, str]) -> list[str]:
    problems = []
    lower = {key.lower(): value for key, value in declared.items()}
    for name, entry in deps['dependencies'].items():
        version = entry.get('version')
        if name.lower() not in lower:
            problems.append(f'{name}: described in deps.json but not declared in conandata.yml')
        elif version != lower[name.lower()]:
            problems.append(f'{name}: deps.json says {version}, conandata.yml says {lower[name.lower()]}')
    for name in declared:
        if name.lower() not in {key.lower() for key in deps['dependencies']}:
            problems.append(f'{name}: declared in conandata.yml but has no deps.json entry '
                            f'(every dependency states where it may appear)')
    return problems


def unused(deps: dict, usage: dict[str, int]) -> list[str]:
    """A declared package nobody includes is a cost with no benefit; that is the debt this gate was asked to keep shut."""
    problems = []
    for name, entry in deps['dependencies'].items():
        if not entry.get('must_be_used', True):
            continue
        if usage.get(name, 0) == 0:
            problems.append(f'{name}: declared and paid for, but no file includes it '
                            f'(expected on: {", ".join(entry.get("allowed_paths", []))})')
    return problems


def selftest(deps: dict) -> int:
    """Cases that must be caught and cases that must be let through. Without this the gate can rot silently."""
    must_fail = [
        ('include/foc/comm/frame.hpp', ['etl/crc16_ccitt.h'], 'public header pulling etl'),
        ('include/foc/core/trace.hpp', ['etl/algorithm.h'], 'public header pulling another etl header'),
        ('src/core/pi.cpp', ['gtest/gtest.h'], 'library source pulling the test framework'),
        ('include/foc/core/strings.hpp', ['pybind11/pybind11.h'], 'public header pulling pybind11'),
    ]
    must_pass = [
        ('src/comm/encoder_stream.cpp', ['etl/bit_stream.h'], 'compiled implementation is allowed'),
        ('test_package/test/unit/test_frame.cpp', ['etl/crc16_ccitt.h', 'gtest/gtest.h'], 'tests are allowed'),
        ('api/python_bindings.cpp', ['pybind11/pybind11.h'], 'the bindings face is allowed'),
        ('include/foc/core/pi.hpp', ['cstdint', 'array'], 'standard headers are not dependencies'),
    ]
    failures: list[str] = []
    for rel, includes, why in must_fail:
        if not classify(rel, includes, deps):
            failures.append(f'not caught: {rel} + <{includes[0]}> ({why})')
    for rel, includes, why in must_pass:
        problems = classify(rel, includes, deps)
        if problems:
            failures.append(f'false positive: {rel} ({why}): {problems[0]}')
    # The other direction: a declared package with no usage must be reported, and one with usage must not.
    if not unused(deps, {}):
        failures.append('not caught: a declared dependency with zero references')
    if unused(deps, {name: 1 for name in deps['dependencies']}):
        failures.append('false positive: a declared dependency that is used')
    if failures:
        for line in failures:
            print(f'[deps] selftest FAIL: {line}', flush=True)
        return 1
    print(f'[deps] selftest OK ({len(must_fail)} violation(s) caught, {len(must_pass)} legal use(s) allowed, '
          f'both directions of the usage check)', flush=True)
    return 0


def _code(text: str) -> str:
    """Backticked identifiers in the registry become `:code:` spans.

    The registry is written for people and puts identifiers in backticks. Letting those reach
    reStructuredText unchanged turns every one of them into a default-role reference, which is legal
    but renders as a citation and reads as emphasis rather than as code.
    """
    return re.sub(r'`([^`]+)`', r':code:`\1`', text)


def _paths(values) -> str:
    """A path list as literal spans. Empty is a fact, not an omission, so it says so."""
    if not values:
        return 'anywhere'
    return ', '.join(f'``{value}``' for value in values)


def render(deps: dict, audit: dict) -> str:
    """The site page for two registries. No hand-written number may appear here."""
    lines = [
        '.. Generated by ``python3 .github/project/deps_gate.py --render``. Do not edit.',
        '.. Sources: ``.github/project/deps.json`` (what we link) and',
        '.. ``.github/project/reuse-audit.json`` (what we may read from). The `docs` gate fails when',
        '.. this page and those registries disagree.',
        '',
        '_`Third-party code: what we link, and what we may read`',
        '=' * (len('Third-party code: what we link, and what we may read') + 4),
        '',
        'Two questions, two registries. `deps.json` records what this library links and where each',
        'package may appear; `reuse-audit.json` records the licence of every upstream project it is',
        'allowed to read from. The library is Apache-2.0 and is meant to be linked into products, so a',
        'single copyleft source file would relicense the whole thing — which is why the second table',
        'exists before anyone opens an upstream repository.',
        '',
        'What this project links',
        '-----------------------',
        '',
        'Every entry below is declared in ``conandata.yml``. Nothing else is linked, vendored or',
        'copied. The same registry drives both checks: an ``#include`` of a package outside the paths',
        'named here fails, and so does a package declared in ``conandata.yml`` that nothing includes.',
        '',
        '.. list-table::',
        '   :header-rows: 1',
        '   :widths: 18 16 10 30 26',
        '',
        '   * - Package',
        '     - Licence',
        '     - Bucket',
        '     - Where it may appear',
        '     - Where it may not',
    ]
    for name, entry in deps['dependencies'].items():
        lines.extend([
            f"   * - {name}/{entry['version']}",
            f"     - {entry['licence']}",
            f"     - {entry['bucket']}",
            # A glob ends in `**`, which reStructuredText reads as the start of a strong-emphasis
            # span; a literal span is both the correct rendering for a path and inert to that rule.
            f"     - {_paths(entry.get('allowed_paths'))}",
            f"     - {_paths(entry.get('forbidden_paths'))}",
        ])
    for name, entry in deps['dependencies'].items():
        title = f"{name} {entry['version']}"
        lines.extend(['', f"_`{title}`", '-' * (len(title) + 4), ''])
        for field, label in (('why_declared', 'Why it is here'),
                             ('what_it_buys', 'What it buys'),
                             ('verdict', 'Verdict')):
            value = entry.get(field)
            if value:
                if lines[-1] != '':
                    lines.append('')
                lines.extend([f'**{label}.** ' + _code(value), ''])
        measured = entry.get('measured') or {}
        if measured:
            lines.extend([
                'Measured cost of putting it on a public header (the reason the paths above are not',
                'negotiable):',
                '',
                '.. list-table::',
                '   :header-rows: 1',
                '   :widths: 34 30 12 24',
                '',
                '   * - Header',
                '     - Public header it would land on',
                '     - Factor',
                '     - Headers pulled',
            ])
            for row in measured.values():
                lines.extend([
                    f"   * - {_code(row['etl_header'])}",
                    f"     - {_code(row['target_header'])}",
                    f"     - {row['factor']}x",
                    f"     - {row['headers_pulled']}",
                ])
            lines.append('')
    lines.extend([
        'What we may read from',
        '---------------------',
        '',
        'Before reading an upstream project closely, vendoring it or copying a function out of it, look',
        'it up here. "Behaviour" means writing our own test for what the other project guarantees;',
        '"code" means copying, and it is only available under a permissive licence.',
        '',

    ])
    verdicts = audit.get('verdicts') or {}
    for project in audit.get('projects') or []:
        key = project.get('vendor')
        verdict = verdicts.get(key) or {}
        label = verdict.get('en') or key or ''
        lines.append(f"``{project['id']}`` — **{project['licence']}**, {label}")
        use = project.get('use') or {}
        lines.append(f"    {_code(use.get('en', ''))}")
        if project.get('evidence'):
            lines.append(f"    **Evidence.** ``{project['evidence']}``")
        lines.append('')
    methods = audit.get('methods') or []
    if methods:
        lines.extend([
            'Working rules',
            '-------------',
            '',
        ])
        for method in methods:
            title = method.get('title') or {}
            body = method.get('body') or {}
            heading = title.get('en', '')
            lines.extend([f"**{heading}**", ''])
            for paragraph in body.get('en') or []:
                lines.extend([_code(paragraph), ''])
    return '\n'.join(lines).rstrip() + '\n'


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--selftest', action='store_true', help='prove the checker still fails on known-bad input')
    parser.add_argument('--render', action='store_true', help="write the Sphinx page from the two registries")
    parser.add_argument('--out', type=Path, default=None,
                        help='where --render writes; the site page by default, a temp path when a doc-side check asks')
    args = parser.parse_args()

    deps = load()
    if args.render:
        target = args.out or PAGE
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(render(deps, json.loads(AUDIT.read_text(encoding='utf-8'))), encoding='utf-8')
        print(f'[deps] wrote {target}', flush=True)
        return 0
    if args.selftest:
        return selftest(deps)

    declared = declared_in_conandata()
    problems = unknown_packages(deps, declared)

    usage = {name: 0 for name in deps['dependencies']}
    owners = {prefix: name for name, entry in deps['dependencies'].items()
              for prefix in entry.get('include_prefixes', [])}
    files = iter_sources()
    for path in files:
        rel = path.relative_to(ROOT).as_posix()
        includes = []
        for number, line in enumerate(path.read_text(encoding='utf-8', errors='replace').splitlines(), start=1):
            match = INCLUDE.match(line)
            if not match:
                continue
            spelling = match.group(1)
            includes.append(spelling)
            owner = next((name for prefix, name in owners.items() if spelling.startswith(prefix)), None)
            if owner is not None:
                usage[owner] += 1
            # No "unknown package" branch on purpose: an include of `<cstdint>` or `<foc/core/pi.hpp>` is not a
            # dependency, and a rule that flagged them would need a hand-maintained list of standard headers --
            # a list that goes stale silently, which is the failure mode this whole line of work is about. The
            # direction that matters (a declared package reaching the public face) is caught by `classify`, and
            # the reverse (a package added to `conandata.yml` without an entry here) by `unknown_packages`.
            # 故意没有“未知包”分支：`<cstdint>` 或 `<foc/core/pi.hpp>` 不是依赖，而要区分它们就得手工维护
            # 一张标准头清单——一张会静默过期的清单，正是这一整条工作要消灭的失效方式。真正要紧的方向
            # （声明过的包渗到公开面）由 `classify` 抓，反方向（加进 `conandata.yml` 却漏了本文件）由
            # `unknown_packages` 抓。
        problems.extend(classify(rel, includes, deps))

    problems.extend(unused(deps, usage))

    if problems:
        for line in problems:
            print(f'[deps] {line}', flush=True)
        print(f'[deps] FAIL ({len(problems)} problem(s) over {len(files)} file(s))', flush=True)
        return 1
    summary = ', '.join(f'{name} {count}' for name, count in sorted(usage.items()))
    print(f'[deps] OK ({len(files)} file(s); references: {summary}; '
          f'{len(declared)} declared in conandata.yml)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
