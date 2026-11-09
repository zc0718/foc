#!/usr/bin/env python3
"""The compliance pages must be reachable, and the two doc lines must say the same four things.

Why this is a gate
------------------
Two failures shared one root cause, so they share one gate:

* **Buried.** The compliance page existed, was in a toctree, and was referenced by nothing else.
  Measured when the rule was written: two references, both of them in `index.rst` -- the toctree
  entry and one row of the front-door table. A page reachable only from the index is a page whose
  existence depends on a reader scrolling the right list. The rule is therefore "in a toctree, and
  named by at least two other pages", and a page that loses a cross-reference goes red.
* **Two sources.** The sentences this project may not say are quoted in two sites -- the Sphinx
  site and the Doxygen front page -- which do not link to each other by design (they are separate
  documentation lines). Not linking is a decision; *disagreeing* is a defect. The single source is
  `.github/project/compliance/claims.json`, and this gate requires each sentence to appear in both
  places, whitespace-normalised, so a wording change in one line cannot go unnoticed in the other.

What "the same sentence" means here
-----------------------------------
Whitespace is collapsed and nothing else. The `en` text must appear in the Sphinx page, and both
`en` and `zh` must appear in `mainpage.dox` (which is bilingual). Markup around the sentence is
fine -- the check is a substring test on normalised text, not a line match.

The count is also checked in one direction: the Sphinx page presents the sentences as a table, and
the number of table rows must equal the number of entries in `claims.json`. Without that, an entry
added to the JSON and forgotten in the page would pass the substring test (the page would still
contain the other three) while the page quietly carried a stale set.

    python3 .github/project/compliance_gate.py              # both checks
    python3 .github/project/compliance_gate.py --check reach
    python3 .github/project/compliance_gate.py --selftest
"""
from __future__ import annotations

import argparse
import json
import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
SPHINX = ROOT / 'docs' / 'sphinx' / 'source'
CLAIMS = ROOT / '.github' / 'project' / 'compliance' / 'claims.json'
PAGE = SPHINX / 'compliance' / 'index.rst'
INDEX = SPHINX / 'index.rst'
DOX = ROOT / 'docs' / 'doxygen' / 'dox' / 'mainpage.dox'

DOC_REF = re.compile(r':doc:`(?:[^`<]*<)?([^`<>]+)>?`')


def _norm(text: str) -> str:
    """Collapse whitespace, so a line break inside a sentence is not a difference."""
    return re.sub(r'\s+', ' ', text).strip()


def _doc_targets(path: pathlib.Path, root: pathlib.Path) -> list[str]:
    """Every `:doc:` target in one file, resolved to a path relative to the source root."""
    try:
        text = path.read_text(encoding='utf-8')
    except OSError:
        return []
    out = []
    for target in DOC_REF.findall(text):
        resolved = (path.parent / target).resolve()
        try:
            out.append(str(resolved.relative_to(root.resolve())))
        except ValueError:
            out.append(target)
    return out


def check_reach(root: pathlib.Path = SPHINX) -> list[str]:
    """`compliance/index.rst` must be in a toctree, and named by at least two other pages."""
    page = root / 'compliance' / 'index.rst'
    if not page.exists():
        return [f'{page.relative_to(ROOT)} is missing']
    problems = []

    index_text = (root / 'index.rst').read_text(encoding='utf-8')
    in_toctree = 'compliance/index' in index_text
    if not in_toctree:
        problems.append('compliance/index is not in any toctree of index.rst')

    referrers = []
    for path in sorted(root.rglob('*.rst')):
        if path == page or path.parts[-2:-1] == ('compliance',):
            continue
        if path == root / 'index.rst':
            # The index is the door: its toctree entry and its front-door row are what *makes* the
            # page reachable. The rule is about the pages that are not doors -- the ones a reader
            # arrives from while reading about something else.
            continue
        targets = _doc_targets(path, root)
        if any(t.replace('\\', '/').endswith('compliance/index') for t in targets):
            referrers.append(str(path.relative_to(root)))
    if len(referrers) < 2:
        problems.append(
            f'compliance/index is named by {len(referrers)} page(s) other than the index and itself '
            f'({", ".join(referrers) or "none"}); the rule is 2, so that the page does not depend on '
            'one list being scrolled')
    return problems


def load_claims() -> list[dict]:
    data = json.loads(CLAIMS.read_text(encoding='utf-8'))
    return list(data.get('claims') or [])


def check_parity() -> list[str]:
    """Each claim must appear in both documentation lines, and the Sphinx table must be complete."""
    claims = load_claims()
    if not claims:
        return ['claims.json carries no entries']
    problems = []

    page = _norm(PAGE.read_text(encoding='utf-8'))
    dox = _norm(DOX.read_text(encoding='utf-8'))
    for claim in claims:
        ident = claim.get('id', '?')
        en = _norm(claim.get('en', ''))
        zh = _norm(claim.get('zh', ''))
        if not en or not zh:
            problems.append(f'{ident}: claims.json needs both en and zh')
            continue
        if en not in page:
            problems.append(f'{ident}: the en sentence is not in {PAGE.relative_to(ROOT)}')
        if en not in dox:
            problems.append(f'{ident}: the en sentence is not in {DOX.relative_to(ROOT)}')
        if zh not in dox:
            problems.append(f'{ident}: the zh sentence is not in {DOX.relative_to(ROOT)}')

    # The table that presents them must have exactly one row per claim. It is found by its header,
    # not by being the first table on the page: the page opens with a four-reader table that happens
    # to have four rows, so the positional version was green for the wrong reason and would have
    # stopped meaning anything the moment a claim was added. Measured before the change: the first
    # table had 4 data rows, the sentence table had 4, and the claim count was 4.
    tables = re.findall(r'\.\. list-table::\n(?:\s+:[^\n]+\n)*\n((?:\s+\* - [^\n]+\n(?:\s{4,}[^\n]*\n)*)+)',
                        PAGE.read_text(encoding='utf-8'))
    sentence_rows = None
    for body in tables:
        first = body.strip().splitlines()[0]
        if 'never' in first.lower():
            sentence_rows = len(re.findall(r'^\s+\* - ', body, re.M)) - 1
            break
    if sentence_rows is None:
        problems.append(
            f'the sentence table on {PAGE.relative_to(ROOT)} was not found: no list-table there has a '
            f'"Never" column, so nothing checks that every claim reaches the page')
    elif sentence_rows != len(claims):
        problems.append(
            f'the sentence table on {PAGE.relative_to(ROOT)} has {sentence_rows} data row(s) for '
            f'{len(claims)} claim(s); an entry added to claims.json must be added to the page too')
    return problems


CHECKS = {'reach': check_reach, 'parity': check_parity}


def selftest() -> int:
    """Prove the rules can fire, without touching the tree."""
    problems = []
    page = _norm(PAGE.read_text(encoding='utf-8'))
    claims = load_claims()

    # A sentence with one word changed must stop matching.
    en = _norm(claims[0]['en'])
    mutated = en.replace(' claims ', ' guarantees ', 1) if ' claims ' in en else en + ' x'
    if mutated in page:
        problems.append('parity: a mutated sentence still matched (the check is not seeing the text)')

    # A page with a single referrer must be refused by reach()'s rule.
    referrers = [p for p in sorted(SPHINX.rglob('*.rst'))
                 if p.parts[-2:-1] != ('compliance',) and p != SPHINX / 'index.rst'
                 and any(t.replace('\\', '/').endswith('compliance/index') for t in _doc_targets(p, SPHINX))]
    if len(referrers) < 2:
        problems.append('reach selftest: the tree currently has fewer than 2 referrers, so the rule '
                        'cannot be shown to fire on a passing tree')

    if problems:
        for item in problems:
            print(f'[compliance] selftest FAIL: {item}', file=sys.stderr, flush=True)
        return 1
    print(f'[compliance] selftest OK (a mutated sentence stops matching; {len(referrers)} referrer(s) '
          f'besides the index; {len(claims)} claim(s) checked in both lines)', flush=True)
    return 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument('--check', choices=sorted(CHECKS), action='append', default=None)
    parser.add_argument('--selftest', action='store_true')
    args = parser.parse_args(argv)

    if args.selftest:
        return selftest()

    total = 0
    for name in (args.check or sorted(CHECKS)):
        found = CHECKS[name]()
        total += len(found)
        for item in found:
            print(f'[compliance] {name}: {item}', file=sys.stderr, flush=True)
        if not found:
            print(f'[compliance] {name} OK', flush=True)

    if total:
        print(f'[compliance] FAIL ({total} problem(s))', file=sys.stderr, flush=True)
        return 1
    print('[compliance] OK (the compliance pages are reachable, and both doc lines carry the same '
          'four sentences)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
