#!/usr/bin/env python3
"""Probe: does a docstring read like a spec, or like the log of the rounds that produced it?

Why this exists
---------------
`docvoice` (`.github/project/docvoice_gate.py`) refuses the **explicit** forms of a diary entry: a
date, "the 3rd time", "we stepped in it". That is a small, safe rule with no false positives, and it
catches almost nothing — measured on this tree, it flagged 2 lines, both of them written by the same
author an hour earlier.

The defect it aims at is larger and quieter. A docstring that was **accreted** — one patch at a time,
each patch gluing a caveat onto the previous claim — reads as a coherent block on the surface and
carries no number anywhere. Its fingerprint is elsewhere:

  * it says *what changed*, instead of *what is* ("早先这里指向控制块；现在把代际放进句柄值")
  * it corrects itself mid-paragraph ("—— 其实……", "-- except …")
  * it stacks `@note` after `@note`, each one answering a review round
  * it has an author: "we", "我", "别再"
  * its subject alternates between the system and the process that discovered the system

So this tool **measures two independent tracks** and prints them side by side; it does not judge.

  Track A -- the text alone. Fingerprints of accretion that carry no number.
  Track B -- the history. How often the file's comments were rewritten, by what kind of commit, and
             whether the growth was append-only (append-only growth of a *comment* is accretion by
             definition: nothing was ever withdrawn).

The point of printing both is the cross-tab: a file that is high on A **and** high on B is where the
leak actually happened; either track alone produces candidates, and candidates are what a reader
should spend time on. A criterion chosen from one track would land on the wrong target — which is the
mistake this tool exists to avoid making twice.

Not a gate. Nothing here turns red: thresholds picked before the distribution was measured are numbers
chosen to be met (the `mutation-baseline.json` comment says the same thing about a different gate).

Usage
-----
  python3 tools/docvoice/accretion.py                  # summary + top files on both tracks
  python3 tools/docvoice/accretion.py --top 12         # longer lists
  python3 tools/docvoice/accretion.py --blocks         # every block with a non-zero A score
  python3 tools/docvoice/accretion.py --file <path>    # one file in full, with its commit history
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SPEC_DIRS = ('include', 'src')
SPEC_SUFFIXES = ('.h', '.hpp', '.c', '.cpp')
# The ledger is not a spec and is not measured here; it is *supposed* to look like this.
SKIP = ('foundation-log',)

# ---------------------------------------------------------------- track A: the text alone
#
# Each pattern is a fingerprint, not a verdict. `why` is what a reader should check; a hit means
# "read this line", never "this line is wrong".
FINGERPRINTS = (
    ('A1-history', 'says what this text used to say -- a spec has no "used to"',
     re.compile(r'早先|原先|上一版|上上一版|此前|曾经|\ban earlier version\b|\bpreviously\b|'
                r'\bused to\b|\boriginally\b|\bthe line above\b', re.IGNORECASE)),
    # Two markers were tried here and rejected by measurement, and the numbers are why:
    #   * "——…但/其实…" fired 20 times on ordinary argumentative prose ("这是刻意的——…——但也意味着"),
    #     i.e. it measured the author's style, not accretion. Dropped.
    #   * "而不是 / rather than / instead of" fired 435 times in 1310 blocks: it is the vocabulary
    #     of engineering precision ("the sum, not the product"). As a per-line marker it is useless;
    #     as a *density* it is not -- a block with three of them is a list of rejections.
    ('A6-attempt', 'narrates an attempt that was abandoned',
     re.compile(r'踩坑|踩过|试过并|试过一|才发现|结果发现|\btried\b|\bbit us\b|\bturned out\b',
                re.IGNORECASE)),
    # Case-SENSITIVE: the first version used IGNORECASE and matched the imaginary unit `i` in
    # formulas, which produced 130 hits of pure noise. A pronoun is not a variable.
    ('A5-author', 'has an author -- a spec has no first person',
     re.compile(r'我[们]|别再|\bwe\b|\bour\b|\bWe\b|\bOur\b')),
)

DENSITY = (
    ('A7-rejection', 'a list of rejected alternatives, three or more in one block',
     re.compile(r'而不是|\brather than\b|\binstead of\b', re.IGNORECASE), 3),
)

NOTE_STACK = 4

_BLOCK = re.compile(r'/\*\*.*?\*/', re.DOTALL)
_NOTE = re.compile(r'@note\b')


def blocks(path: Path) -> list[tuple[int, str]]:
    text = path.read_text(encoding='utf-8')
    out = []
    for match in _BLOCK.finditer(text):
        start = text.count('\n', 0, match.start()) + 1
        out.append((start, match.group(0)))
    return out


def score_block(text: str) -> dict:
    """(kind -> hit lines, note count, line count) for one doc block."""
    hits: dict[str, list[str]] = {}
    for number, line in enumerate(text.splitlines()):
        for kind, _why, pattern in FINGERPRINTS:
            if pattern.search(line):
                hits.setdefault(kind, []).append(line.strip()[:110])
    return {
        'hits': hits,
        'score': sum(len(v) for v in hits.values()),
        'kinds': len(hits),
        'notes': len(_NOTE.findall(text)),
        'lines': text.count('\n') + 1,
    }


def track_a() -> tuple[dict[str, dict], list[tuple[str, int, dict]]]:
    per_file: dict[str, dict] = {}
    ranked: list[tuple[str, int, dict]] = []
    for name in SPEC_DIRS:
        base = ROOT / name
        if not base.is_dir():
            continue
        for path in sorted(base.rglob('*')):
            if path.suffix not in SPEC_SUFFIXES or not path.is_file():
                continue
            rel = path.relative_to(ROOT).as_posix()
            if any(token in rel for token in SKIP):
                continue
            agg = per_file.setdefault(rel, {'blocks': 0, 'score': 0, 'kinds': {}, 'lines': 0,
                                            'notes': 0})
            for start, text in blocks(path):
                result = score_block(text)
                agg['blocks'] += 1
                agg['score'] += result['score']
                agg['lines'] += result['lines']
                agg['notes'] += result['notes']
                for kind, lines in result['hits'].items():
                    agg['kinds'][kind] = agg['kinds'].get(kind, 0) + len(lines)
                if result['kinds'] >= 2:
                    ranked.append((rel, start, result))
    ranked.sort(key=lambda item: (-item[2]['score'], item[0], item[1]))
    return per_file, ranked


# ---------------------------------------------------------------- track B: the history
_HUNK_FILE = re.compile(r'^\+\+\+ b/(.+)$')
_COMMENT = re.compile(r'^\s*(?:\*|//|/\*)')


def track_b() -> dict[str, dict]:
    """Per-file comment churn from one `git log -p` pass.

    `appends` counts commits whose comment hunks contained **no deleted line at all**: the docstring
    only ever grew there. `fix_share` is the fraction of those commits whose subject is `fix:`/`perf:`
    -- a spec that keeps being corrected by bug fixes is either a spec that lied or a spec that is
    being used as a changelog.
    """
    raw = subprocess.run(
        ['git', 'log', '-p', '--unified=0', '--format=%x01%H%x02%s', '--', *SPEC_DIRS],
        cwd=ROOT, capture_output=True, text=True).stdout
    per_file: dict[str, dict] = {}
    subject, path, added, removed = '', None, 0, 0

    def flush():
        if path is None:
            return
        row = per_file.setdefault(path, {'commits': 0, 'adds': 0, 'dels': 0, 'appends': 0,
                                         'fixes': 0})
        if added or removed:
            row['commits'] += 1
            row['adds'] += added
            row['dels'] += removed
            if removed == 0:
                row['appends'] += 1
            if subject.startswith(('fix', 'perf')):
                row['fixes'] += 1

    for line in raw.splitlines():
        if line.startswith('\x01'):
            flush()
            head = line[1:].split('\x02', 1)
            subject = head[1] if len(head) > 1 else ''
            path, added, removed = None, 0, 0
            continue
        match = _HUNK_FILE.match(line)
        if match:
            # A `+++` line closes the previous file's segment and opens the next one.
            flush()
            path, added, removed = match.group(1), 0, 0
            continue
        if line.startswith(('+++', '---')) or not line.strip():
            continue
        if line.startswith('+') and _COMMENT.match(line[1:]):
            added += 1
        elif line.startswith('-') and _COMMENT.match(line[1:]):
            removed += 1
    flush()

    for row in per_file.values():
        row['append_ratio'] = (row['appends'] / row['commits']) if row['commits'] else 0.0
        row['fix_share'] = (row['fixes'] / row['commits']) if row['commits'] else 0.0
    return per_file


# ---------------------------------------------------------------- track C: per-block provenance
#
# The general signal, and the only one that does not depend on how the sentence is phrased: **who
# built this block, and was anything ever taken out of it.** A block that no commit ever withdrew a
# line from grew purely by addition -- that is what accretion *is*, stated without a metaphor.
#
# Attribution is by text, not by line number: a line that a commit added and that is still in the
# block today belongs to that block, whatever moved around it since. Lines shorter than 15 characters
# and the Doxygen boilerplate are ignored -- ` * @since 0.1.0` is in half the blocks and would attribute
# every commit to all of them.
# A tag *with nothing after it* is boilerplate; a tag carrying a sentence is the prose. The first
# version of this filter dropped every line beginning with `@brief`, which in this repository is the
# prose of nearly every block -- it silently measured 491 of 1310 blocks. Filtering is by how much
# remains after the tag, never by the tag's presence.
_TAG = re.compile(r'^@(?:brief|note|param|return|retval|tparam|throws|post|pre|warning|see|file)\b'
                  r'(?:\s*\[(?:en|zh)\])?\s*')


def _norm(line: str) -> str:
    """One normalisation for both sides.

    Both the comment lead-in and the tag have to go: the patch side writes ` * text`, a `///` block
    writes `/// text`, and the parsed block has neither. Stripping only `*` made the `/// becomes a
    block` sweep look like new content on both sides of the comparison, which is why the three real
    sweeps were not recognised as mechanical.
    """
    return line.strip().lstrip('*/!<').strip()


def _interesting(line: str) -> bool:
    text = _TAG.sub('', _norm(line)).strip()
    if text in ('', '*', '*/', '/**'):
        return False
    return len(text) >= 15 and not text.startswith('---')


# ---------------------------------------------------------------- track D: the structure
#
# The one track that can become a *rule* rather than a suspicion, because it does not try to recognise
# accretion's wording -- it removes accretion's habitat.
#
# Accretion has a physical mechanism: a caveat is discovered, and there is nowhere in the block to put
# it, so it becomes another `@note`. Do that ten times and the block is a pile of footnotes with a
# `@brief` on top. The home already exists in Doxygen's own vocabulary -- `@warning` for a refusal,
# `@see` for a pointer, a body paragraph for the law -- and a limit that lives in `@note` instead of
# `@warning` is the fingerprint of a block that grew by patching. (An earlier version of this comment
# named a six-section skeleton invented in this repo; that template was withdrawn, see plan section 5.3:
# the tag shape is fixed upstream by `_shared/code-conventions.md` + `fcpp/include/doctest.hpp`.)
#
# Measured 2026-11-06: 546 `@note` paragraphs, 197 of them (36 %) carry a refusal or a limit. So the
# class is large, mechanical to detect, and mechanical to fix -- move the sentence into the field a
# reader actually looks at.
REFUSAL = re.compile(r'做不到|不能|不会|不可|不支持|拒绝|\bworse than\b|\bcannot\b|\bdoes not\b|'
                     r'\bmust not\b|\bnever\b|\bis not valid\b|\brejects?\b', re.IGNORECASE)
NOTE_PARA = re.compile(r'@note\b[^\n]*\n((?:[^\n]*\n)*?)(?=\n?\s*\*\s*@|\Z)')


def notes_carrying_limits(path: Path) -> list[tuple[int, str]]:
    """(line, first offending line) for every `@note` paragraph that states a limit."""
    out = []
    text = path.read_text(encoding='utf-8')
    for start, block in blocks(path):
        if not block:
            continue
        for match in NOTE_PARA.finditer(block):
            if not REFUSAL.search(match.group(0)):
                continue
            line = next((ln.strip().lstrip('*').strip() for ln in match.group(0).splitlines()[1:]
                         if REFUSAL.search(ln)), '')
            out.append((start, line[:110]))
    return out


def track_d() -> dict[str, int]:
    per_file: dict[str, int] = {}
    for name in SPEC_DIRS:
        base = ROOT / name
        if not base.is_dir():
            continue
        for path in sorted(base.rglob('*')):
            if path.suffix not in SPEC_SUFFIXES or not path.is_file():
                continue
            hits = notes_carrying_limits(path)
            if hits:
                per_file[path.relative_to(ROOT).as_posix()] = len(hits)
    return per_file


def provenance() -> dict[str, list[tuple[int, str, str, str]]]:
    """file -> [(commit ordinal, subject, added-text, removed-text)] for every comment line.

    The ordinal matters: counting *events* instead of *commits* made one commit that added 61 lines look
    like 40 collaborators, which is the kind of number that would have produced a threshold on the wrong
    quantity. Oldest commit last, because `git log` walks backwards.
    """
    raw = subprocess.run(
        ['git', 'log', '-p', '--unified=0', '--format=%x01%H%x02%s', '--', *SPEC_DIRS],
        cwd=ROOT, capture_output=True, text=True).stdout
    out: dict[str, list[tuple[int, str, str, str]]] = {}
    subject, path, ordinal = '', None, -1
    for line in raw.splitlines():
        if line.startswith('\x01'):
            ordinal += 1
            subject = line[1:].split('\x02', 1)[-1]
            path = None
            continue
        match = _HUNK_FILE.match(line)
        if match:
            path = match.group(1)
            continue
        if path is None or line.startswith(('+++', '---')):
            continue
        if line.startswith('+') and _COMMENT.match(line[1:]):
            out.setdefault(path, []).append((ordinal, subject, _norm(line[1:]), ''))
        elif line.startswith('-') and _COMMENT.match(line[1:]):
            out.setdefault(path, []).append((ordinal, subject, '', _norm(line[1:])))
    return out


def mechanical_commits(prov: dict) -> tuple[set[int], dict[int, str]]:
    """Commits that moved the prose around rather than adding any.

    Two earlier attempts at this are recorded because both failed in the same way -- they measured the
    wrong quantity and looked convincing:

      1. "Did this commit add a line that survives in this block?" With no filter, two sweeps
         (`one tag line per language group`, 2392 added / 2365 removed; `one doc form -- /// becomes a
         block`) added a line to *every* block, so 200 blocks looked accreted.
      2. "Do its added lines share a 20-character head with its removed lines?" Zero hits on a reflow:
         the sweep merged `@brief [en]`/`@brief [zh]` pairs and re-broke every sentence, so no line
         survives verbatim even though nothing new was said.

    What separates them is the **volume ratio**, and the distribution has a gap: measured over the
    commits that touch >= 40 comment lines, the sweeps sit at removed/added 1.32, 1.00, 0.99 and 0.76,
    while every feature and fix commit is at 0.69 or below (the 0.69 one rewrote a whole section
    about observer selection -- a rewrite, and treating it as ordinary work is the conservative
    choice). So: a commit that deleted 70 % as many lines as it added replaced prose; one that deleted
    a tenth of them added to it.

    Deliberately not keyed on the commit subject: a title is a claim, the ratio is a measurement.
    """
    added: dict[int, int] = {}
    removed: dict[int, int] = {}
    subjects: dict[int, str] = {}
    for events in prov.values():
        for ordinal, subject, add, rem in events:
            subjects[ordinal] = subject
            if add and _interesting(add):
                added[ordinal] = added.get(ordinal, 0) + 1
            if rem and _interesting(rem):
                removed[ordinal] = removed.get(ordinal, 0) + 1
    mechanical = set()
    for ordinal, adds in added.items():
        rems = removed.get(ordinal, 0)
        if adds + rems < 40:
            continue
        if rems / adds >= 0.7:
            mechanical.add(ordinal)
    return mechanical, subjects


def track_c(per_file_hist: dict[str, dict], skip: set[int] | None = None) -> list[dict]:
    prov = provenance()
    mechanical, _subjects = mechanical_commits(prov)
    skip = mechanical if skip is None else skip
    rows = []
    for name in SPEC_DIRS:
        base = ROOT / name
        if not base.is_dir():
            continue
        for path in sorted(base.rglob('*')):
            if path.suffix not in SPEC_SUFFIXES or not path.is_file():
                continue
            rel = path.relative_to(ROOT).as_posix()
            events = prov.get(rel, [])
            if not events:
                continue
            for start, text in blocks(path):
                current = {_norm(ln) for ln in text.splitlines() if _interesting(ln)}
                if not current:
                    continue
                built, corrected = set(), set()
                for ordinal, subject, added, removed in events:
                    if ordinal in skip:
                        continue
                    if added and added in current:
                        built.add((ordinal, subject))
                    elif removed:
                        # A withdrawal that shares a 24-character head with a surviving line is a
                        # correction of this block, not of some other one.
                        head = removed[:24]
                        if len(head) >= 20 and any(ln.startswith(head) for ln in current):
                            corrected.add((ordinal, subject))
                rows.append({
                    'file': rel, 'line': start,
                    'built_by': len(built), 'corrected_by': len(corrected),
                    'notes': len(_NOTE.findall(text)), 'lines': text.count('\n') + 1,
                    'subjects': sorted({s for _i, s in built})[:6],
                })
    rows.sort(key=lambda r: (-r['built_by'], r['file'], r['line']))
    return rows


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--top', type=int, default=8)
    parser.add_argument('--blocks', action='store_true', help='list every block with >=2 kinds')
    parser.add_argument('--file', help='one spec file in full')
    args = parser.parse_args(argv[1:])

    a, ranked = track_a()
    b = track_b()

    if args.file:
        rel = args.file
        print(f'=== {rel} ===')
        row = b.get(rel, {})
        print(f"history: {row.get('commits', 0)} commit(s) touched its comments, "
              f"{row.get('appends', 0)} of them append-only "
              f"({row.get('append_ratio', 0):.0%}), {row.get('fixes', 0)} were fix/perf "
              f"({row.get('fix_share', 0):.0%}); +{row.get('adds', 0)}/-{row.get('dels', 0)} lines")
        for rel2, start, result in ranked:
            if rel2 != rel:
                continue
            print(f'\n  block at {rel}:{start}  score {result["score"]} kinds {result["kinds"]} '
                  f'notes {result["notes"]} lines {result["lines"]}')
            for kind, lines in result['hits'].items():
                why = next(w for k, w, _ in FINGERPRINTS if k == kind)
                print(f'    {kind}: {why}')
                for line in lines[:3]:
                    print(f'      | {line}')
        return 0

    total_blocks = sum(row['blocks'] for row in a.values())
    hits = sum(row['score'] for row in a.values())
    print(f'Track A (text): {total_blocks} doc block(s) in {len(a)} file(s), '
          f'{hits} fingerprint line(s), '
          f'{len(ranked)} block(s) with >=2 distinct kinds')
    by_kind: dict[str, int] = {}
    for row in a.values():
        for kind, count in row['kinds'].items():
            by_kind[kind] = by_kind.get(kind, 0) + count
    for kind, _why, _pattern in FINGERPRINTS:
        print(f'  {kind:14} {by_kind.get(kind, 0):4} line(s)')
    print(f'\nTrack B (history): {sum(r["commits"] for r in b.values())} comment-touching '
          f'commit(s) across {len(b)} file(s)')

    if args.blocks:
        print('\n--- blocks with >=2 distinct kinds (read these, do not trust the score) ---')
        for rel, start, result in ranked:
            print(f'\n{rel}:{start}  score {result["score"]} kinds {result["kinds"]} '
                  f'notes {result["notes"]}')
            for kind, lines in result['hits'].items():
                print(f'  {kind}: {lines[0]}')

    # The cross-tab: high on both tracks is where a leak actually accumulated.
    combined = []
    for rel, row in a.items():
        hist = b.get(rel, {})
        if not hist.get('commits'):
            continue
        combined.append((rel, row['score'], hist['commits'], hist.get('append_ratio', 0),
                         hist.get('fix_share', 0)))
    combined.sort(key=lambda item: (-(item[1] * item[2]), item[0]))

    d = track_d()
    print(f'\nTrack D (structure): {sum(d.values())} `@note` paragraph(s) carrying a refusal or a '
          f'limit, in {len(d)} file(s). A limit belongs in `Refusals`; in a note it is a caveat with '
          f'nowhere to live.')
    for rel, count in sorted(d.items(), key=lambda kv: -kv[1])[:args.top]:
        print(f'  {count:3}  {rel}')

    print(f'\n--- cross-tab, top {args.top} by (A score x B commits) ---')
    print(f'{"A":>4} {"B":>4} {"app%":>5} {"fix%":>5}  file')
    for rel, score, commits, append_ratio, fix_share in combined[:args.top]:
        print(f'{score:4} {commits:4} {append_ratio:5.0%} {fix_share:5.0%}  {rel}')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
