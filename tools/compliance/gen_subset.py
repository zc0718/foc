#!/usr/bin/env python3
"""Generate and maintain `rules.tsv` — the language-subset rule table.

What this tool is for
---------------------
The rule table is the half of a compliance claim that a reviewer reads first, and it is the half
most likely to go stale: the pinned `.github/misc/.clang-tidy` decides which rules exist, and
nobody remembers to add a row when a glob quietly starts matching one more check. So the tidy-rule
rows are **generated and checked**, and only the human columns (`status`, `why`, `owner`) are
maintained by hand.

Three states are possible for a tidy rule, and they are not interchangeable:
  * enabled by the pinned config and not suppressed  -> `implemented`
  * enabled and suppressed with `// NOLINT...`      -> `deviation` (a record must exist)
  * **not** enabled but suppressed with `// NOLINT...` -> `deviation` as well: the rule applies to
    this code (there is a real site and a signed permit), and "the check is off" is a statement
    about the tool, not about the operand. It used to be filed as `not-applicable`, which is the
    one status a misjudgement could hide in -- the count went down and nothing improved.
  * not enabled and not suppressed                   -> absent from this table
    (a suppression for a rule that is not in force is harmless and worth keeping: it means the day
    somebody enables that family, no code has to change. The record explains why the cast is
    legitimate, so the suppression is not silently load-bearing later.)

Why `--snapshot` exists: `clang-tidy --list-checks` needs a clang-tidy. The gate has to run on
machines where the binary is absent or a different version, so the expansion is pinned into
`tool-checks.txt` and the gate compares the *pinned config* (a file, always readable) against that
snapshot. When the binary is present the snapshot is re-derived and any difference is reported;
when it is absent the row set is still fully checked, just without the "the globs still expand to
the same 101 names" leg.

Usage
-----
  python3 tools/compliance/gen_subset.py --snapshot    # re-derive tool-checks.txt, then sync rules.tsv
  python3 tools/compliance/gen_subset.py               # sync rules.tsv from the snapshot
  python3 tools/compliance/gen_subset.py --check       # report what would change, write nothing
"""
from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
TIDY_CONFIG = ROOT / '.github' / 'misc' / '.clang-tidy'
SNAPSHOT = HERE / 'tool-checks.txt'
MANUAL = HERE / 'rules-manual.json'
RULES = HERE / 'rules.tsv'

COLUMNS = ('rule', 'check', 'status', 'lang', 'delta', 'why', 'owner', 'source')
STATUSES = ('implemented', 'not-applicable', 'deviation', 'open')
DELTAS = ('none', 'gap', 'vacuous', 'drift')

_NOLINT = re.compile(r'NOLINT[A-Z]*\(([^)]*)\)')


def pinned_checks() -> tuple[str, list[str]]:
    """(the `Checks:` value as written, the family globs it names), read from the pinned config.

    Parsed rather than imported on purpose: `.github/misc/.clang-tidy` is byte-pinned by the
    `scaffold` gate, so it is the fact, and this tool must not carry a second copy of it.
    """
    text = TIDY_CONFIG.read_text(encoding='utf-8')
    # Blank lines belong to the block: YAML's `>` folds them into a newline, and the pinned file
    # uses one to separate the two comment groups. A pattern that stops at the first empty line
    # would silently drop every check listed after it -- measured: it dropped both
    # `cppcoreguidelines-*` entries and produced a 99-row table instead of 101.
    block = re.search(r'^Checks:\s*>?\s*\n((?:[ \t]+.*\n|\n)+)', text, re.MULTILINE)
    if not block:
        raise SystemExit(f'gen_subset: no `Checks:` block in {TIDY_CONFIG}')
    items: list[str] = []
    for line in block.group(1).splitlines():
        line = line.split('#', 1)[0].strip()
        for item in line.split(','):
            item = item.strip()
            if item:
                items.append(item)
    return ' '.join(items), items


def tool_version() -> str | None:
    for exe in ('clang-tidy', '/usr/bin/clang-tidy'):
        try:
            done = subprocess.run([exe, '--version'], capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired):
            continue
        if done.returncode == 0:
            first = done.stdout.splitlines()
            return ' '.join(first[:2]) if first else 'unknown'
    return None


def expand(items: list[str]) -> list[str]:
    """The enabled check names, from the installed clang-tidy."""
    done = subprocess.run(['clang-tidy', '--list-checks', '--checks=' + ','.join(items)],
                          capture_output=True, text=True, timeout=120)
    if done.returncode != 0:
        raise SystemExit(f'gen_subset: clang-tidy --list-checks failed:\n{done.stderr[-1500:]}')
    return [line.strip() for line in done.stdout.splitlines() if line.startswith('    ')]


def read_snapshot() -> tuple[list[str], dict[str, str]]:
    """(check names, header fields) from tool-checks.txt."""
    names: list[str] = []
    head: dict[str, str] = {}
    for line in SNAPSHOT.read_text(encoding='utf-8').splitlines():
        if line.startswith('# '):
            key, _, value = line[2:].partition(':')
            head[key.strip()] = value.strip()
        elif line.strip() and not line.startswith('#'):
            names.append(line.strip())
    return names, head


def write_snapshot(names: list[str], checks_value: str) -> None:
    version = tool_version() or 'unavailable'
    lines = [
        '# Snapshot of the checks the pinned .github/misc/.clang-tidy enables.',
        '# Regenerate with: python3 tools/compliance/gen_subset.py --snapshot',
        f'# tool: {version}',
        f'# config: {checks_value}',
        '#',
        '# This file is data, not a wish: the gate compares the pinned config against it, and',
        '# `--snapshot` re-derives it from the installed binary. A difference means the tool',
        '# changed what a glob matches, which is exactly the kind of drift a rule table must',
        '# never absorb silently.',
    ] + names
    SNAPSHOT.write_text('\n'.join(lines) + '\n', encoding='utf-8')


def suppressions() -> dict[str, list[str]]:
    """rule -> the files that suppress it, from `// NOLINT...` comments in the shipped tree."""
    found: dict[str, list[str]] = {}
    for base in ('include', 'src', 'api'):
        for path in sorted((ROOT / base).rglob('*')):
            if path.suffix not in ('.h', '.hpp', '.c', '.cpp') or not path.is_file():
                continue
            for number, line in enumerate(path.read_text(encoding='utf-8').splitlines(), 1):
                for match in _NOLINT.finditer(line):
                    for rule in match.group(1).split(','):
                        rule = rule.strip()
                        if rule:
                            found.setdefault(rule, []).append(
                                f'{path.relative_to(ROOT).as_posix()}:{number}')
    return found


def manual_rows() -> list[dict[str, str]]:
    """The hand-maintained half: everything a tool cannot decide.

    JSON rather than TSV on purpose -- each row carries a long `why` sentence in two languages,
    and a tab-separated line cannot hold one without turning a stray tab into a silent column
    shift. `rules.tsv` is the view for a human who wants to read the table with no tooling.
    """
    data = json.loads(MANUAL.read_text(encoding='utf-8'))
    rows = data['rules'] if isinstance(data, dict) else data
    out = []
    for row in rows:
        missing = [c for c in COLUMNS if c not in row]
        if missing:
            raise SystemExit(f'gen_subset: manual rule {row.get("rule")!r} is missing {missing}')
        out.append({c: str(row[c]) for c in COLUMNS})
    return out


def tidy_rows(names: list[str], suppressed: dict[str, list[str]],
              manual: list[dict[str, str]]) -> list[dict[str, str]]:
    """One row per check the pinned config enables, with the hand-written columns on top."""
    by_rule = {row['rule']: row for row in manual}
    rows = []
    for name in names:
        row = {c: '' for c in COLUMNS}
        row.update(rule=name, check=f'tidy:{name}', lang='any', delta='none', owner='zc',
                   source='clang-tidy (pinned config)')
        if name in suppressed:
            row['status'] = 'deviation'
            row['why'] = f'deviation/{name}.md -- suppressed at ' + ', '.join(
                sorted(suppressed[name]))
        else:
            row['status'] = 'implemented'
            row['why'] = '-'
        # A hand-written row wins on the human columns: a deviation outlives the switch that made
        # it necessary, and a reviewer's reason is not the tool's to overwrite.
        for column in ('status', 'why', 'owner', 'source', 'lang', 'delta'):
            if name in by_rule and by_rule[name][column]:
                row[column] = by_rule[name][column]
        rows.append(row)
    return rows


def render(rows: list[dict[str, str]]) -> str:
    lines = [
        '# Language-subset rule table. Columns: ' + ' | '.join(COLUMNS),
        '# GENERATED by tools/compliance/gen_subset.py from tool-checks.txt (the pinned',
        '# .clang-tidy, expanded) and rules-manual.json (everything a tool cannot decide).',
        '# Edit those two, not this file.',
        '# status: ' + ' / '.join(STATUSES) + '   delta: ' + ' / '.join(DELTAS)
        + ' (see LANG-ALIGNMENT.md section 3)',
        '\t'.join(COLUMNS),
    ]
    for row in rows:
        lines.append('\t'.join(row[c] for c in COLUMNS))
    return '\n'.join(lines) + '\n'


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--snapshot', action='store_true',
                        help='re-derive tool-checks.txt from the installed clang-tidy')
    parser.add_argument('--check', action='store_true', help='report drift, write nothing')
    args = parser.parse_args(argv[1:])

    checks_value, items = pinned_checks()
    if args.snapshot:
        if tool_version() is None:
            print('[gen] clang-tidy is not installed; refusing to overwrite the snapshot',
                  file=sys.stderr)
            return 2
        names = expand(items)
        if args.check:
            print(f'[gen] the snapshot would record {len(names)} checks')
            return 0
        write_snapshot(names, checks_value)
        print(f'[gen] wrote {SNAPSHOT.name} ({len(names)} checks, {tool_version()})')
    elif not SNAPSHOT.is_file():
        print(f'[gen] {SNAPSHOT.name} missing; run with --snapshot', file=sys.stderr)
        return 2

    names, head = read_snapshot()
    if head.get('config') != checks_value:
        print('[gen] the pinned `Checks:` value no longer matches the snapshot:\n'
              f'      snapshot: {head.get("config")}\n      pinned:   {checks_value}\n'
              '      run --snapshot to re-derive it', file=sys.stderr)
        return 1

    manual = manual_rows()
    enabled = set(names)
    # Manual rows whose `rule` is an enabled check name are folded into the generated row, so they
    # are skipped here -- except a deviation, which must survive even if the check later disappears
    # from the config (the suppression is still in the code).
    others = [row for row in manual
              if row['rule'] not in enabled or row['status'] == 'deviation']
    rows: list[dict[str, str]] = []
    seen: set[str] = set()
    for row in tidy_rows(names, suppressions(), manual) + others:
        if row['rule'] in seen:
            continue
        seen.add(row['rule'])
        rows.append(row)
    rows.sort(key=lambda r: (0 if r['check'].startswith('tidy:') else 1, r['rule']))

    text = render(rows)
    current = RULES.read_text(encoding='utf-8') if RULES.is_file() else ''
    if text == current:
        print(f'[gen] {RULES.name} is up to date ({len(rows)} rows)')
        return 0
    if args.check:
        old = set(current.splitlines())
        changed = [line.split('\t')[0] for line in text.splitlines()
                   if line not in old and not line.startswith('#') and '\t' in line]
        print(f'[gen] {RULES.name} is stale; run `python3 tools/compliance/gen_subset.py`')
        if changed:
            print('[gen] rows that would change: ' + ', '.join(changed[:12])
                  + (' ...' if len(changed) > 12 else ''))
        return 1
    RULES.write_text(text, encoding='utf-8')
    tidy = sum(1 for r in rows if r['check'].startswith('tidy:'))
    print(f'[gen] wrote {RULES.name} ({len(rows)} rows: {tidy} tidy + {len(rows) - tidy} other)')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
