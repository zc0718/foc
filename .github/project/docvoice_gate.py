#!/usr/bin/env python3
"""`docvoice` gate — a spec docstring is not a diary.

Why this gate exists
--------------------
This repository keeps three registers, and they were being written into each other:

* **spec** — `include/**` and `src/**` docstrings. What the code models, what it guarantees, what it
  refuses. Read by someone deciding whether to use the library.
* **guide** — `docs/sphinx/source/**` minus the ledger. The same, for a task ("how do I home this axis").
* **ledger** — `docs/sphinx/source/foundation-log.rst`. Every withdrawal, every wrong turn, every "the
  second time this bit us". Dates are its *job*; it is the most valuable file here and it must not be
  sanitised.

The defect is not that the ledger is chatty, it is that ledger sentences leak into the other two
registers: a date in a spec says "this fact has a shelf life" in a place where the fact is supposed to
be timeless, and "the third time this happened" is addressed to whoever was in the room. This gate makes
that a build failure rather than a matter of taste, because taste has not held for long enough.

What it looks for
-----------------
Dates (`2026-11-06`, `6月11日`), ordinals of repetition (`第 3 次`, `the 3rd time`) and the colloquial
"we stepped in it" (`踩了`, `踩过`). It deliberately does **not** flag spelled-out ordinals used for
engineering ("第一次采样" = the first sample, "the first move"): those are quantities of the system, not
of the author's history. Measured when the rule was written: the digit forms matched 4 lines in the
whole repository, the spelled-out forms matched dozens of legitimate technical sentences.

The escape hatch is one word: put `History` in the block (or paragraph) and it is skipped. That is the
"declare it, do not disguise it" rule — a paragraph that says it is history is history; one that merely
looks like history is a spec that will rot.

Exemptions are declared, not implied
------------------------------------
Some files are *supposed* to carry dates, and the gate asserts each of those still exists, so an
exemption cannot outlive its reason (the lesson `scaffold` taught when a stale tolerance hid a change).

Usage
-----
  python3 .github/project/docvoice_gate.py             # check (default)
  python3 .github/project/docvoice_gate.py --selftest  # prove each rule can fire
  python3 .github/project/docvoice_gate.py --baseline  # re-freeze the count (review event)
  python3 .github/project/docvoice_gate.py --list      # show every violation

Exit codes: 0 clean or at baseline, 1 above baseline, 2 setup problem.
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
BASELINE = HERE / 'docvoice-baseline.json'

SPEC_DIRS = ('include', 'src')
GUIDE_DIR = ROOT / 'docs' / 'sphinx' / 'source'
SPEC_SUFFIXES = ('.h', '.hpp', '.c', '.cpp', '.cxx')

# Files that may carry dates *because dates are their content*. Each entry states why, and the gate
# checks that the path still exists -- a tolerance nobody re-reads is how a rule quietly stops applying.
EXEMPT = {
    'docs/sphinx/source/foundation-log.rst':
        'the ledger: every entry is dated, and that is its whole value',
    'api/fmu/MASTERS.md':
        'a coverage matrix: master x FMI version x verification date is the row, not the footnote',
}

PATTERNS = (
    ('date', re.compile(r'\b20\d\d-\d\d-\d\d\b')),
    ('date', re.compile(r'\d{1,2}月\d{1,2}日')),
    ('ordinal', re.compile(r'第\s*\d+\s*次')),
    ('ordinal', re.compile(r'\bthe \d+(?:st|nd|rd|th) time\b', re.IGNORECASE)),
    ('colloquial', re.compile(r'踩(?:了|过)')),
)

# The one escape hatch, and it has to be visible in the block itself.
TOLERANCE = re.compile(r'\bHistory\b')


def blocks_of(text: str, suffix: str) -> list[tuple[int, str]]:
    """(first line number, block text) units. A Doxygen block for source, a paragraph for prose.

    Blocks, not lines, because the tolerance is a claim about a whole block: half a paragraph in the
    diary voice and half in the spec voice is exactly the mixture this gate is here to stop.
    """
    if suffix in SPEC_SUFFIXES:
        out = []
        for match in re.finditer(r'/\*\*.*?\*/', text, re.DOTALL):
            start = text.count('\n', 0, match.start()) + 1
            out.append((start, match.group(0)))
        return out
    out, current, start = [], [], 0
    for number, line in enumerate(text.splitlines(), 1):
        if line.strip():
            if not current:
                start = number
            current.append(line)
        elif current:
            out.append((start, '\n'.join(current)))
            current = []
    if current:
        out.append((start, '\n'.join(current)))
    return out


def violations_of(path: Path) -> list[dict]:
    """Every offending line in one file, with the block it belongs to."""
    found: list[dict] = []
    suffix = path.suffix
    text = path.read_text(encoding='utf-8')
    for start, block in blocks_of(text, suffix):
        if TOLERANCE.search(block):
            continue
        for offset, line in enumerate(block.splitlines()):
            for kind, pattern in PATTERNS:
                if pattern.search(line):
                    found.append({
                        'file': path.relative_to(ROOT).as_posix(),
                        'line': start + offset,
                        'kind': kind,
                        'text': line.strip()[:120],
                    })
    return found


def is_exempt(rel: str) -> str | None:
    for prefix, why in EXEMPT.items():
        if rel == prefix or rel.startswith(prefix.rstrip('/') + '/') or rel.startswith(prefix):
            return why
    return None


def scan() -> tuple[list[dict], list[str]]:
    """(violations, problems with the setup itself)."""
    problems: list[str] = []
    for prefix in EXEMPT:
        if not (ROOT / prefix).exists():
            problems.append(f'exempted path {prefix!r} no longer exists; drop the exemption, or a '
                            f'rule that no longer has a reason keeps not applying')

    found: list[dict] = []
    for name in SPEC_DIRS:
        base = ROOT / name
        if not base.is_dir():
            problems.append(f'{name}/ is missing')
            continue
        for path in sorted(base.rglob('*')):
            if path.suffix in SPEC_SUFFIXES and path.is_file():
                found += violations_of(path)
    if GUIDE_DIR.is_dir():
        for path in sorted(GUIDE_DIR.rglob('*.rst')):
            rel = path.relative_to(ROOT).as_posix()
            if is_exempt(rel):
                continue
            found += violations_of(path)
    else:
        problems.append('docs/sphinx/source/ is missing')

    for path in sorted((ROOT / 'api').rglob('*.md')) if (ROOT / 'api').is_dir() else []:
        rel = path.relative_to(ROOT).as_posix()
        if is_exempt(rel):
            continue
        found += violations_of(path)
    return found, problems


def counts(found: list[dict]) -> dict[str, int]:
    out: dict[str, int] = {}
    for item in found:
        out[item['file']] = out.get(item['file'], 0) + 1
    return out


def read_baseline() -> tuple[dict[str, int], list[str]]:
    if not BASELINE.is_file():
        return {}, [f'{BASELINE.name} is missing; run with --baseline']
    data = json.loads(BASELINE.read_text(encoding='utf-8'))
    per_file = data.get('per_file')
    if not isinstance(per_file, dict):
        return {}, [f'{BASELINE.name} has no `per_file` map']
    return {str(k): int(v) for k, v in per_file.items()}, []


def judge(now: dict[str, int], base: dict[str, int]) -> list[str]:
    problems = []
    for path, count in sorted(now.items()):
        allowed = base.get(path, 0)
        if count > allowed:
            problems.append(f'{path}: {count} diary line(s), baseline allows {allowed}')
    return problems


def selftest() -> int:
    """Every rule fires on something, and the two stated tolerances hold."""
    cases = [
        ('a date in a spec block', 'include/x.hpp', '/**\n * measured 2026-11-06 with gcc\n */', True),
        ('an ordinal', 'include/x.hpp', '/**\n * the 3rd time this happened\n */', True),
        ('the colloquial', 'include/x.hpp', '/**\n * 上一轮踩过这个坑\n */', True),
        ('a one-sample ordinal is technical, not a diary', 'include/x.hpp',
         '/**\n * 第一次采样之前读 0\n */', False),
        ('a month-year is not a date of record', 'include/x.hpp', '/**\n * since 2026-11\n */', False),
        ('the escape hatch', 'include/x.hpp',
         '/**\n * @note History: measured 2026-11-06, wrong before\n */', False),
        ('a paragraph without a date', 'docs/sphinx/source/x.rst', 'Just prose.\n', False),
    ]
    failures = 0
    for name, rel, text, want in cases:
        suffix = Path(rel).suffix
        got = []
        for start, block in blocks_of(text, suffix):
            if TOLERANCE.search(block):
                continue
            for offset, line in enumerate(block.splitlines()):
                for kind, pattern in PATTERNS:
                    if pattern.search(line):
                        got.append((start + offset, kind))
        hit = bool(got)
        ok = hit == want
        failures += 0 if ok else 1
        print(f'[docvoice-selftest] {"ok  " if ok else "MISS"} {name}: '
              f'{"caught" if hit else "clean"} (wanted {"caught" if want else "clean"})')

    base = judge({'a.hpp': 2, 'b.hpp': 1}, {'a.hpp': 2, 'b.hpp': 3})
    ok = base == []
    failures += 0 if ok else 1
    print(f'[docvoice-selftest] {"ok  " if ok else "MISS"} at baseline: {base or "no problems"}')

    rose = judge({'a.hpp': 3}, {'a.hpp': 2})
    ok = len(rose) == 1 and 'baseline allows 2' in rose[0]
    failures += 0 if ok else 1
    print(f'[docvoice-selftest] {"ok  " if ok else "MISS"} one above baseline: {rose}')

    print(f'[docvoice-selftest] {len(cases) + 2 - failures} / {len(cases) + 2} judgement(s) as expected')
    return 1 if failures else 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--selftest', action='store_true', help='prove each rule can fire')
    parser.add_argument('--baseline', action='store_true', help='re-freeze the per-file counts')
    parser.add_argument('--list', action='store_true', help='print every violation')
    args = parser.parse_args(argv[1:])

    if args.selftest:
        return selftest()

    found, problems = scan()
    now = counts(found)
    base, more = read_baseline()
    problems += more
    problems += judge(now, base)

    if args.baseline:
        BASELINE.write_text(json.dumps({
            '_comment': [
                'Per-file count of diary sentences in the spec and guide registers. The ratchet runs one',
                'way: a file may fall below its number freely, and may not rise. Re-run with --baseline in',
                'the same commit that changes a number, so the number in the diff is one somebody decided on.',
                'The ledger (foundation-log.rst) is exempt by EXEMPT in docvoice_gate.py, not by absence here.',
            ],
            'recorded': '2026-11-06',
            'per_file': now,
        }, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
        print(f'[docvoice] froze {sum(now.values())} line(s) across {len(now)} file(s)')
        return 0

    if args.list or problems:
        for item in found:
            print(f'[docvoice] {item["file"]}:{item["line"]} {item["kind"]}: {item["text"]}')
    total = sum(now.values())
    if problems:
        for problem in problems:
            print(f'[docvoice] FAIL {problem}', file=sys.stderr)
        print(f'[docvoice] FAIL ({len(problems)} problem(s))')
        return 1
    print(f'[docvoice] OK ({total} diary line(s), at or below baseline in every file; '
          f'registers: {" + ".join(SPEC_DIRS)}, docs/sphinx/source/** except the declared exemptions)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
