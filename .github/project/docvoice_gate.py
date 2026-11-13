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
    # The marker below was selected by measurement, not by taste. Six candidates were run over 1310
    # doc blocks by `.github/project/docvoice/accretion.py`; five were rejected and the numbers are why:
    #   * "而不是 / rather than / instead of"      -> 435 hits: it is the vocabulary of precision.
    #   * "——…但/其实…" (corrects itself)          -> 20 hits, all of them ordinary argument.
    #   * a paragraph opening with a connector      -> 2 hits.
    #   * first person (we / our / 我们)            -> 18 hits, and half are the normative "we" of a
    #     policy statement or a quotation of the caller ("we did not wire the fault pin"), not an author.
    #   * "tried / 试过 / turned out"               -> matched "the widest cutoff tried" and
    #     "不是以后才发现的" (a negation), i.e. technical prose.
    # What survives is the one marker whose meaning cannot be anything else: the text says what **it**
    # used to say. `used to` is restricted to a following state verb, because "used to seed it" is a
    # purpose and matched on the first pass.
    ('history', re.compile(r'早先|原先|上一版|此前|曾经|\ban earlier version\b|\bpreviously\b|'
                           r'\bused to (?:be|have|say|point|read|honour|work|return|do|stop|mean|'
                           r'claim)\b|\boriginally\b|\bthe line above\b', re.IGNORECASE)),
)

# The one escape hatch, and it has to be visible in the block itself.
TOLERANCE = re.compile(r'\bHistory\b')

# Private work codes: a register letter and a number that only the ledger can resolve. No
# legitimate use exists in a shipped surface, so this needs no allowance and no tuning -- measured
# before it was written: `\b[FCA]-\d+\b` matched 25 lines across `include/`, `src/` and `api/**`,
# every one of them a citation of an internal entry.
#
# Two labels that look like candidates and are deliberately **not** in this family, because
# measuring said so:
#   * `L1`..`L4` are the evidence levels, defined in the README legend and used as vocabulary.
#   * `S1`..`S9` and `U1`..`U8` numbered visible lists (the task pages, the honest limits), which a
#     reader resolves by scrolling. They were renamed for tone, not because they were unreadable.
ID_PATTERNS = (
    ('private-id', re.compile(r'\b[FCA]-\d+\b')),
    # The same defect in two more shapes: the review passes were numbered `O-1`.. and the planning
    # rounds `W7-a`, and both were cited in prose exactly as the register codes were. The planning
    # form keeps its letter suffix in the rule (`W7` alone is a wattage as often as a round).
    ('private-id', re.compile(r'\bO-\d+\b')),
    ('private-id', re.compile(r'\bW\d+-[a-z]\b')),
    # The planning document was cited by name too. A bare erratum number (`D24`) is **not** in this
    # family: it cannot be told apart from a diode designator, so that family is covered by editing
    # only -- measured at two sites, both in `include/`, and both rewritten.
    ('private-id', re.compile(r'\bPRD\b')),
)


# A third family, and the only one that is not about *what* a block says but about which half of the
# block says it. A doc block has an English region and a Chinese region, and `docs/build.py` splits them
# by the `[en]`/`[zh]` markers: text before the first `[zh]` is what the English site prints. A Chinese
# sentence on the English side therefore publishes Chinese to a reader who asked for English -- and it
# does so *silently*, because the PDF and the HTML both render.
#
# Measured once at `include/foc/core/motion_profile.hpp`: a paragraph's translation was appended to the
# end of its English original (`... the sign follows the error. 那个制动距离是 ...`). Every other rule
# in this file reads the text's content and had nothing to fire on; the block was in the wrong half.
CJK = re.compile(r'[\u4e00-\u9fff]')


def language_violations(text: str, suffix: str) -> list[tuple[int, str]]:
    """(line number, offending line) for Chinese sitting in a block's English region.

    Only spec blocks: a guide paragraph has no `[en]`/`[zh]` markers to be on the wrong side of --
    the hand-written pages are English and the generated ones carry both registers by construction.
    """
    if suffix not in SPEC_SUFFIXES:
        return []
    found: list[tuple[int, str]] = []
    for start, block in blocks_of(text, suffix):
        side = None
        for offset, line in enumerate(block.splitlines()):
            if '[zh]' in line:
                side = 'zh'
                continue
            if '[en]' in line:
                side = 'en'
                continue
            if CJK.search(line) and side != 'zh':
                found.append((start + offset, line.strip()[:120]))
    return found


def mixed_language(path: Path) -> list[dict]:
    """`language_violations` for one file on disk, in the shape the gate reports."""
    return [{'file': path.relative_to(ROOT).as_posix(), 'line': line, 'kind': 'language', 'text': text}
            for line, text in language_violations(path.read_text(encoding='utf-8'), path.suffix)]



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


def violations_of(path: Path, patterns=PATTERNS, honour_tolerance: bool = True) -> list[dict]:
    """Every offending line in one file, with the block it belongs to."""
    found: list[dict] = []
    suffix = path.suffix
    text = path.read_text(encoding='utf-8')
    for start, block in blocks_of(text, suffix):
        if honour_tolerance and TOLERANCE.search(block):
            continue
        for offset, line in enumerate(block.splitlines()):
            for kind, pattern in patterns:
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


def scan() -> tuple[list[dict], list[dict], list[str]]:
    """(diary violations, private-id violations, problems with the setup itself)."""
    problems: list[str] = []
    for prefix in EXEMPT:
        if not (ROOT / prefix).exists():
            problems.append(f'exempted path {prefix!r} no longer exists; drop the exemption, or a '
                            f'rule that no longer has a reason keeps not applying')

    found: list[dict] = []
    ids: list[dict] = []

    def visit(path: Path, exempt: bool) -> None:
        # The date/ordinal/colloquial rules honour the exemption; the private-code family
        # does not. See the module docstring for why that is not a loophole.
        ids.extend(violations_of(path, ID_PATTERNS, honour_tolerance=False))
        ids.extend(mixed_language(path))
        if not exempt:
            found.extend(violations_of(path))

    for name in SPEC_DIRS:
        base = ROOT / name
        if not base.is_dir():
            problems.append(f'{name}/ is missing')
            continue
        for path in sorted(base.rglob('*')):
            if path.suffix in SPEC_SUFFIXES and path.is_file():
                visit(path, exempt=False)
    if GUIDE_DIR.is_dir():
        for path in sorted(GUIDE_DIR.rglob('*.rst')):
            visit(path, exempt=bool(is_exempt(path.relative_to(ROOT).as_posix())))
    else:
        problems.append('docs/sphinx/source/ is missing')

    for path in sorted((ROOT / 'api').rglob('*.md')) if (ROOT / 'api').is_dir() else []:
        visit(path, exempt=bool(is_exempt(path.relative_to(ROOT).as_posix())))
    return found, ids, problems


def counts(found: list[dict]) -> dict[str, int]:
    out: dict[str, int] = {}
    for item in found:
        out[item['file']] = out.get(item['file'], 0) + 1
    return out


def read_baseline() -> tuple[dict[str, int], dict[str, int], list[str]]:
    """(diary counts, private-id counts, problems). Two maps, one file."""
    if not BASELINE.is_file():
        return {}, {}, [f'{BASELINE.name} is missing; run with --baseline']
    data = json.loads(BASELINE.read_text(encoding='utf-8'))
    problems: list[str] = []
    out: list[dict[str, int]] = []
    for key in ('per_file', 'private_id_per_file'):
        per_file = data.get(key)
        if not isinstance(per_file, dict):
            problems.append(f'{BASELINE.name} has no `{key}` map')
            per_file = {}
        out.append({str(k): int(v) for k, v in per_file.items()})
    return out[0], out[1], problems


def judge(now: dict[str, int], base: dict[str, int], what: str = 'diary line') -> list[str]:
    problems = []
    for path, count in sorted(now.items()):
        allowed = base.get(path, 0)
        if count > allowed:
            problems.append(f'{path}: {count} {what}(s), baseline allows {allowed}')
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
        ('a spec that says what it used to say', 'include/x.hpp',
         '/**\n * the line above used to claim the opposite\n */', True),
        ('"used to" as a purpose is not history', 'include/x.hpp',
         '/**\n * this accessor is used to seed the observer before a run\n */', False),
        ('a paragraph without a date', 'docs/sphinx/source/x.rst', 'Just prose.\n', False),
    ]
    id_cases = [
        ('a private code in a spec block', 'include/x.hpp',
         '/**\n * the same shape as F-40, which is why\n */', True),
        ('a private code in the ledger', 'docs/sphinx/source/foundation-log.rst',
         'Root cause here (A-110).\n', True),
        ('the escape hatch does not cover a code', 'include/x.hpp',
         '/**\n * @note History: measured, see C-7\n */', True),
        ('an evidence level is not a private code', 'include/x.hpp',
         '/**\n * an L2 statement, not an L4 one\n */', False),
        ('a review pass number is a private code', 'docs/sphinx/source/x.rst',
         'It reaches L3 through the pass, O-13, which named a different question.\n', True),
        ('a planning round is a private code', 'docs/sphinx/source/x.rst',
         'The leg has not linked since W7-a.\n', True),
        ('the planning document is not to be named', 'include/x.hpp',
         '/**\n * the target is PRD U6\n */', True),
        ('a wattage is not a private code', 'include/x.hpp',
         '/**\n * a 2 W7 allowance would be a different sentence\n */', False),
        ('a list label is not a private code', 'docs/sphinx/source/x.rst',
         'See the honest limits, U8.\n', False),
    ]
    lang_cases = [
        ('Chinese in the English half of a block', 'include/x.hpp',
         '/**\n * @brief [en] The settle, and its edge.\n *\n * @brief [zh] 稳态与它的边界。\n */', False),
        ('a translation appended to an English paragraph', 'include/x.hpp',
         '/**\n * @brief [en] The settle.\n *\n *   the sign follows the error. 那个制动距离是 **指令** 的属性。\n *\n * @brief [zh] 稳态。\n */', True),
        ('Chinese under its own [zh] tag', 'include/x.hpp',
         '/**\n * @brief [zh] 每弧度误差对应的速度，1/s。\n */', False),
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

    for name, rel, text, want in id_cases:
        suffix = Path(rel).suffix
        got = []
        for start, block in blocks_of(text, suffix):
            for offset, line in enumerate(block.splitlines()):
                for kind, pattern in ID_PATTERNS:
                    if pattern.search(line):
                        got.append((start + offset, kind))
        hit = bool(got)
        ok = hit == want
        failures += 0 if ok else 1
        print(f'[docvoice-selftest] {"ok  " if ok else "MISS"} {name}: '
              f'{"caught" if hit else "clean"} (wanted {"caught" if want else "clean"})')
    total_cases = len(cases) + len(id_cases) + len(lang_cases)

    for name, rel, text, want in lang_cases:
        hit = bool(language_violations(text, Path(rel).suffix))
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

    rose = judge({'a.rst': 1}, {}, what='private code')
    ok = len(rose) == 1 and 'private code(s)' in rose[0]
    failures += 0 if ok else 1
    print(f'[docvoice-selftest] {"ok  " if ok else "MISS"} a private code with no baseline: {rose}')

    print(f'[docvoice-selftest] {total_cases + 3 - failures} / {total_cases + 3} '
          f'judgement(s) as expected')
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

    found, ids, problems = scan()
    now = counts(found)
    id_now = counts(ids)
    base, id_base, more = read_baseline()
    problems += more
    problems += judge(now, base)
    problems += judge(id_now, id_base, what='private code')

    if args.baseline:
        BASELINE.write_text(json.dumps({
            '_comment': [
                'Per-file count of diary sentences in the spec and guide registers. The ratchet runs one',
                'way: a file may fall below its number freely, and may not rise. Re-run with --baseline in',
                'the same commit that changes a number, so the number in the diff is one somebody decided on.',
                'The ledger (foundation-log.rst) is exempt by EXEMPT in docvoice_gate.py, not by absence here.',
                '`private_id_per_file` is the same ratchet for internal work codes (`F-40`, `A-110`). It is',
                'kept in a separate map because EXEMPT does not apply to it: dates are the ledger\'s content,',
                'but the codes are not.',
            ],
            'recorded': '2026-11-09',
            'per_file': now,
            'private_id_per_file': id_now,
        }, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
        print(f'[docvoice] froze {sum(now.values())} diary line(s) across {len(now)} file(s)')
        print(f'[docvoice] froze {sum(id_now.values())} private code(s) across {len(id_now)} file(s)')
        return 0

    if args.list or problems:
        for item in found + ids:
            print(f'[docvoice] {item["file"]}:{item["line"]} {item["kind"]}: {item["text"]}')
    total = sum(now.values())
    if problems:
        for problem in problems:
            print(f'[docvoice] FAIL {problem}', file=sys.stderr)
        print(f'[docvoice] FAIL ({len(problems)} problem(s))')
        return 1
    print(f'[docvoice] OK ({total} diary line(s), {sum(id_now.values())} private code(s), '
          f'at or below baseline in every file; '
          f'registers: {" + ".join(SPEC_DIRS)}, docs/sphinx/source/** except the declared exemptions, '
          f'plus the private-code family over the exemptions too)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
