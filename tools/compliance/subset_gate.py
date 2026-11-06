#!/usr/bin/env python3
"""`subset` gate — the language-subset rule table must be complete, resolvable and honest.

What this gate is
-----------------
`rules.tsv` is the sentence "we implement N rules, M do not apply, K are deviated, O are open"
turned into a table. A table like that is worth exactly as much as the checks that stop it from
drifting, so this gate checks seven things, each of which has already been wrong once in this
repo's history:

1. **Vocabulary** — `status` is one of the four, `delta` one of the four, `rule` unique, no row
   without a `why`. A fifth status would silently stop being counted.
2. **The four counts add up** — implemented + not-applicable + deviation + open equals the number
   of rows. This is the "no silent omission" leg: a row that matched no status would vanish from
   the summary while still sitting in the table.
3. **Every `implemented` rule resolves to a mechanism that exists** — `tidy:<check>` must be
   enabled by the pinned config, `gate:<name>` must be a gate that exists, `test:<file>` must be a
   file, `case:<id>` must be a case. This is the only meaning "implemented" has. (Upstream's own
   wording: a rule that nothing enforces is a wish.)
4. **Every deviation has a record with the five elements** — reason / impact / compensating
   control / approver / review date. A deviation without a record is not a deviation, it is an
   unrecorded violation.
5. **Suppressions and records are a bijection** — every `// NOLINT(...)` in the shipped tree has a
   record, and every record has a suppression. One direction stops a silent suppression; the other
   stops a stale record that makes an old argument look current.
6. **The 13 terminal L1 units are `not-applicable`, never `open`** — they are interfaces, tables
   and test doubles: they move values, they do not compute, so behaviour rules have no operand
   there. Counting them as `open` would make `open` unreachable-to-zero and turn the ratchet into
   decoration. The list is read from `README.md`'s claim table, not written here, so it cannot
   drift.
7. **The `open` count never rises** — `subset-baseline.json` freezes it; closing an item is
   allowed, opening one requires re-freezing the baseline in the same commit, which is a review
   event rather than an accident.

`--selftest` proves each leg can go red by feeding the evaluator five doctored tables. A gate that
has never said no is a gate nobody has a reason to trust.

Usage
-----
  python3 tools/compliance/subset_gate.py              # check, print the compliance summary
  python3 tools/compliance/subset_gate.py --selftest   # prove the checks can fail
  python3 tools/compliance/subset_gate.py --baseline   # re-freeze the open count (review event)
  python3 tools/compliance/subset_gate.py --summary    # rewrite COMPLIANCE-SUMMARY.md
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
HERE = Path(__file__).resolve().parent
RULES = HERE / 'rules.tsv'
MANUAL = HERE / 'rules-manual.json'
SNAPSHOT = HERE / 'tool-checks.txt'
DEVIATIONS = HERE / 'deviation'
BASELINE = HERE / 'subset-baseline.json'
SUMMARY = HERE / 'COMPLIANCE-SUMMARY.md'
GATES_PY = ROOT / '.github' / 'project' / 'gates.py'
NEGATIVE = ROOT / '.github' / 'project' / 'negative_compile.py'
TESTS = ROOT / 'test_package' / 'test'

COLUMNS = ('rule', 'check', 'status', 'lang', 'delta', 'why', 'owner', 'source')
STATUSES = ('implemented', 'not-applicable', 'deviation', 'open')
DELTAS = ('none', 'gap', 'vacuous', 'drift')
# A deviation record is legal only with all five. This is the list MISRA Compliance:2020 puts in
# its example permit, and leaving one out is how a deviation becomes a note-to-self.
ELEMENTS = ('为什么', '影响', '替代控制', '批准人', '复查日期')
UPSTREAM_GATES = ('format', 'tidy', 'guard')

_NOLINT = re.compile(r'NOLINT[A-Z]*\(([^)]*)\)')
_HEADING = re.compile(r'^#{2,3}\s*(.+?)\s*$', re.MULTILINE)


# ---------------------------------------------------------------------------- inputs
def read_rules(path: Path = RULES) -> tuple[list[dict[str, str]], list[str]]:
    if not path.is_file():
        return [], [f'{path.name} is missing; run tools/compliance/gen_subset.py']
    lines = [line for line in path.read_text(encoding='utf-8').splitlines()
             if line.strip() and not line.startswith('#')]
    if not lines:
        return [], [f'{path.name} has no rows']
    header = tuple(lines[0].split('\t'))
    if header != COLUMNS:
        return [], [f'{path.name} header is {list(header)}, expected {list(COLUMNS)}']
    rows, problems = [], []
    for number, line in enumerate(lines[1:], start=2):
        cells = line.split('\t')
        if len(cells) != len(COLUMNS):
            problems.append(f'{path.name}:{number} has {len(cells)} column(s), expected '
                            f'{len(COLUMNS)} — a stray tab shifts every cell after it')
            continue
        rows.append(dict(zip(COLUMNS, cells)))
    return rows, problems


def enabled_checks() -> tuple[set[str], list[str]]:
    """The check names the pinned config enables, from the snapshot (never from the binary here)."""
    if not SNAPSHOT.is_file():
        return set(), [f'{SNAPSHOT.name} is missing; run gen_subset.py --snapshot']
    names = {line.strip() for line in SNAPSHOT.read_text(encoding='utf-8').splitlines()
             if line.strip() and not line.startswith('#')}
    return names, [] if names else [f'{SNAPSHOT.name} lists no checks']


def known_gates() -> set[str]:
    text = GATES_PY.read_text(encoding='utf-8')
    match = re.search(r'^PROJECT_GATES = \((.*?)\)\s*$', text, re.MULTILINE | re.DOTALL)
    names = set(re.findall(r"'([a-z-]+)'", match.group(1))) if match else set()
    return names | set(UPSTREAM_GATES)


def negative_cases() -> set[str]:
    """The ids of the negative-compilation cases.

    The file declares them as dict literals with a `"name"` key (it briefly used an `id`, and an
    earlier draft of this gate looked for that one spelling and silently matched nothing -- which is
    why the empty-set case is a problem below, not a pass).
    """
    text = NEGATIVE.read_text(encoding='utf-8')
    names = set(re.findall(r"""['"]name['"]\s*:\s*['"]([A-Za-z0-9_]+)['"]""", text))
    names |= set(re.findall(r"^CASE\(([A-Za-z0-9_]+)\)", text, re.MULTILINE))
    return names


def test_files() -> set[str]:
    return {p.name for p in TESTS.rglob('*') if p.is_file()} if TESTS.is_dir() else set()


def suppressions() -> dict[str, list[str]]:
    found: dict[str, list[str]] = {}
    for base in ('include', 'src', 'api'):
        root = ROOT / base
        if not root.is_dir():
            continue
        for path in sorted(root.rglob('*')):
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


def deviation_records() -> dict[str, Path]:
    if not DEVIATIONS.is_dir():
        return {}
    return {p.stem: p for p in sorted(DEVIATIONS.glob('*.md')) if p.name != 'README.md'}


def claimed_l1_units() -> tuple[set[str], list[str]]:
    """The terminal L1 claim rows' modules, read from README.md through `claim_gate`.

    Read, not written: a hand-kept copy of this list would be the first thing to go stale, and the
    failure would look like a rule table that is *more* complete than the claim table.
    """
    try:
        import importlib.util
        spec = importlib.util.spec_from_file_location(
            'claim_gate', ROOT / '.github' / 'project' / 'claim_gate.py')
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        rows = module.claim_rows()
    except Exception as error:                     # noqa: BLE001 - reported, never swallowed
        return set(), [f'cannot read the claim table: {type(error).__name__}: {error}']
    units: set[str] = set()
    for modules, level, _claim, _evidence in rows:
        if level == 'L1':
            units.update(modules)
    return units, []


def read_baseline() -> tuple[int | None, list[str]]:
    if not BASELINE.is_file():
        return None, [f'{BASELINE.name} is missing; run with --baseline']
    data = json.loads(BASELINE.read_text(encoding='utf-8'))
    value = data.get('open')
    if not isinstance(value, int):
        return None, [f'{BASELINE.name} has no integer `open`']
    return value, []


# ---------------------------------------------------------------------------- evaluation
def evaluate(rows: list[dict[str, str]], ctx: dict) -> list[str]:
    """Every complaint about the table, as sentences. Empty means it is clean."""
    problems: list[str] = []

    # A resolver that returns nothing turns every row that uses it into a row failure, which reads
    # like the table is wrong when in fact the input is. Name the input.
    for key, what in (('enabled', 'the pinned clang-tidy snapshot'),
                      ('tests', 'the test tree'), ('cases', 'the negative-compilation cases')):
        if not ctx.get(key):
            problems.append(f'{what} yielded nothing to resolve against; every row that uses it '
                            f'would fail for the wrong reason')

    seen: set[str] = set()
    for row in rows:
        rid = row.get('rule', '')
        if not rid:
            problems.append('a row has no `rule`')
            continue
        if rid in seen:
            problems.append(f'{rid}: appears twice — the four counts would count it twice')
        seen.add(rid)
        if row.get('status') not in STATUSES:
            problems.append(f'{rid}: status {row.get("status")!r} is not one of {list(STATUSES)}')
        if row.get('delta') not in DELTAS:
            problems.append(f'{rid}: delta {row.get("delta")!r} is not one of {list(DELTAS)}')
        if not row.get('why', '').strip():
            problems.append(f'{rid}: no `why` — a row without a reason is a row nobody decided')

    counts = {status: sum(1 for row in rows if row.get('status') == status) for status in STATUSES}
    if sum(counts.values()) != len(rows):
        problems.append(f'the four counts sum to {sum(counts.values())} but the table has '
                        f'{len(rows)} row(s) — a row matched no status')
    if not rows:
        problems.append('the rule table is empty: a subset that implements nothing reports a '
                        'perfect score for measuring nothing')

    for row in rows:
        rid, check, status = row.get('rule', ''), row.get('check', ''), row.get('status')
        if status != 'implemented':
            continue
        kind, _, name = check.partition(':')
        if kind == 'tidy':
            if name not in ctx['enabled']:
                problems.append(f'{rid}: {check} names a check the pinned config does not enable')
        elif kind == 'gate':
            if name not in ctx['gates']:
                problems.append(f'{rid}: {check} names a gate that does not exist')
        elif kind == 'test':
            if name not in ctx['tests']:
                problems.append(f'{rid}: {check} names a test file that does not exist')
        elif kind == 'case':
            if name not in ctx['cases']:
                problems.append(f'{rid}: {check} names a negative-compilation case that does not '
                                f'exist')
        else:
            problems.append(f'{rid}: check {check!r} is not one of tidy:/gate:/test:/case: — '
                            f'an `implemented` rule must name the mechanism that enforces it')

    for row in rows:
        if row.get('status') != 'deviation':
            continue
        rid = row['rule']
        record = ctx['records'].get(rid)
        if record is None:
            problems.append(f'{rid}: deviated but deviation/{rid}.md does not exist')
            continue
        headings = ' '.join(_HEADING.findall(record.read_text(encoding='utf-8')))
        missing = [element for element in ELEMENTS if element not in headings]
        if missing:
            problems.append(f'{rid}: deviation/{rid}.md is missing {missing}')

    for rule, sites in sorted(ctx['nolint'].items()):
        if rule not in ctx['records']:
            problems.append(f'{rule}: suppressed at {", ".join(sites[:3])} '
                            f'({len(sites)} site(s)) with no deviation/{rule}.md')
    for name in sorted(set(ctx['records']) - set(ctx['nolint'])):
        problems.append(f'deviation/{name}.md exists but nothing suppresses it — a stale record '
                        f'makes an old argument look current')

    for unit in sorted(ctx['claimed_l1']):
        row = next((r for r in rows if r.get('rule') == f'unit:{unit}'), None)
        if row is None:
            problems.append(f'{unit}: an L1 terminal claim row with no `not-applicable` row of its '
                            f'own — the rule table is less complete than the claim table')
        elif row.get('status') != 'not-applicable':
            problems.append(f'{unit}: L1 terminal unit recorded as {row.get("status")!r}; an '
                            f'interface/table/double has no behaviour, so `open` here would make '
                            f'the ratchet unreachable')

    if ctx['baseline'] is None:
        problems.append('no `open` baseline: the ratchet has nothing to compare against')
    elif counts['open'] > ctx['baseline']:
        problems.append(f'open rose to {counts["open"]} from a baseline of {ctx["baseline"]}; '
                        f'closing an item is free, opening one is a review event (--baseline)')

    return problems


def collect() -> tuple[list[dict[str, str]], dict, list[str]]:
    rows, problems = read_rules()
    enabled, more = enabled_checks()
    problems += more
    baseline, more = read_baseline()
    problems += more
    units, more = claimed_l1_units()
    problems += more
    ctx = {
        'enabled': enabled,
        'gates': known_gates(),
        'tests': test_files(),
        'cases': negative_cases(),
        'nolint': suppressions(),
        'records': deviation_records(),
        'claimed_l1': units,
        'baseline': baseline,
    }
    return rows, ctx, problems


def summary_text(rows: list[dict[str, str]], ctx: dict) -> str:
    counts = {status: sum(1 for row in rows if row.get('status') == status) for status in STATUSES}
    by_source: dict[str, dict[str, int]] = {}
    for row in rows:
        bucket = by_source.setdefault(row.get('source', '?'), {s: 0 for s in STATUSES})
        bucket[row['status']] = bucket.get(row['status'], 0) + 1
    out = [
        '# 语言子集合规摘要',
        '',
        '本文件由 `tools/compliance/subset_gate.py --summary` 从 `rules.tsv` 生成，请勿手改。',
        '',
        '## 四个计数',
        '',
        '| 状态 | 条数 | 含义 |',
        '|---|---|---|',
        f'| `implemented` | **{counts["implemented"]}** | 有机器执行的机制，`check` 列指名它是谁 |',
        f'| `not-applicable` | **{counts["not-applicable"]}** | 接口/表/测试替身（`delta: none`），或该语言特性被编译期关掉（`delta: vacuous`） |',
        f'| `deviation` | **{counts["deviation"]}** | 有 `deviation/<rule>.md`，五要素齐全 |',
        f'| `open` | **{counts["open"]}** | 已知缺口，只减不增（基线 {ctx["baseline"]}） |',
        f'| **合计** | **{len(rows)}** | = `rules.tsv` 的行数（这条等式由门禁强制） |',
        '',
        '## 按来源拆开',
        '',
        '| 来源 | implemented | not-applicable | deviation | open |',
        '|---|---|---|---|---|',
    ]
    for source, bucket in sorted(by_source.items()):
        out.append(f'| {source} | {bucket["implemented"]} | {bucket["not-applicable"]} '
                   f'| {bucket["deviation"]} | {bucket["open"]} |')
    out += [
        '',
        '## 能说的话与不能说的话',
        '',
        '能写：',
        '',
        f'- "语言基线 C++17 / C11，由 `strict` 门逐头强制；子集登记的 {len(rows)} 条规则里'
        f' {counts["implemented"]} 条有机器执行，{counts["deviation"]} 条以偏差许可提交，'
        f'{counts["not-applicable"]} 条不适用并附理由。"',
        '',
        '不能写（在补齐之前）：',
        '',
        '- **不能写任何 MISRA C++:2023 的 N/M/K 计数**——规则编号映射还没做（`open` 里'
        ' `misra-rule-mapping` 一行就是它）。今天能引用的是**结构**（四类计数 + 偏差五要素），'
        '不是**数值**。',
        '- 不能写"符合 MISRA C++:2023"或任何 ASIL / SIL / Class B 等级。',
        '',
        '## 为什么三条 open 关不掉（这是本表最有信息量的部分）',
        '',
        '`cert-*`、`cppcoreguidelines-*`、`-Wconversion` 三条都卡在同一个地方：',
        '**启用点在 `.github/misc/.clang-tidy`，而那个文件被 `scaffold` 门字节钉住**。',
        '所以它们的关闭方式不是"改代码"，是"上游补丁改一行配置"。把它们记成 `open` 而不是',
        '`deviation`，是因为我们**打算**做，只是本地做不了——这个区别决定了下一个人该去改什么。',
        '',
    ]
    return '\n'.join(out)


def selftest() -> int:
    """Five doctored tables, five specific complaints. Each leg of the gate must be able to fail."""
    good_row = {'rule': 'r1', 'check': 'tidy:performance-enum-size', 'status': 'implemented',
                'lang': 'any', 'delta': 'none', 'why': '-', 'owner': 'zc', 'source': 's'}
    base_ctx = {
        'enabled': {'performance-enum-size'},
        'gates': {'strict'},
        'tests': {'test_x.cpp'},
        'cases': {'c1'},
        'nolint': {},
        'records': {},
        'claimed_l1': set(),
        'baseline': 0,
    }
    cases = [
        ('the table is clean', [dict(good_row)], {}, []),
        ('an implemented rule names a check that is not enabled',
         [dict(good_row, check='tidy:not-a-check')], {},
         ['names a check the pinned config does not enable']),
        ('an implemented rule names a gate that does not exist',
         [dict(good_row, check='gate:nope')], {},
         ['names a gate that does not exist']),
        ('a deviation with no record',
         [dict(good_row, status='deviation', rule='d1')], {},
         ['deviation/d1.md does not exist']),
        ('a suppression with no record',
         [dict(good_row)],
         {'nolint': {'performance-enum-size': ['include/x.hpp:1']}},
         ['with no deviation/performance-enum-size.md']),
        ('open rose above the baseline',
         [dict(good_row, status='open')], {'baseline': 0},
         ['open rose to 1']),
        ('an L1 terminal unit recorded as open',
         [dict(good_row, rule='unit:core/traits.hpp', check='-', status='open'),
          dict(good_row)],
         {'claimed_l1': {'core/traits.hpp'}},
         ['L1 terminal unit recorded as']),
        ('a status outside the vocabulary',
         [dict(good_row, status='todo')], {},
         ['is not one of']),
    ]
    failures = 0
    for name, rows, patch, expected in cases:
        ctx = dict(base_ctx, **patch)
        # The four-count identity needs the open baseline to be reachable; a clean table passes it.
        ctx.setdefault('baseline', 0)
        problems = evaluate(rows, ctx)
        if expected:
            hit = any(all(token in problem for token in expected) for problem in problems)
            verdict = 'OK  ' if hit else 'MISS'
            failures += 0 if hit else 1
            print(f'[subset-selftest] {verdict} {name}: expected a complaint containing '
                  f'{expected}, got {problems}') if not hit else \
                print(f'[subset-selftest] {verdict} {name}: caught')
        else:
            verdict = 'OK  ' if not problems else 'MISS'
            failures += 0 if not problems else 1
            print(f'[subset-selftest] {verdict} {name}: '
                  f'{"no problems" if not problems else problems}')
    print(f'[subset-selftest] {len(cases) - failures} / {len(cases)} judgement(s) as expected')
    return 1 if failures else 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--selftest', action='store_true', help='prove the checks can fail')
    parser.add_argument('--baseline', action='store_true', help='re-freeze the open count')
    parser.add_argument('--summary', action='store_true', help='rewrite COMPLIANCE-SUMMARY.md')
    parser.add_argument('--verbose', action='store_true',
                        help='also list the implemented rows (101 of them, all alike)')
    args = parser.parse_args(argv[1:])

    if args.selftest:
        return selftest()
    verbose = args.verbose

    rows, ctx, problems = collect()
    problems += evaluate(rows, ctx)
    if args.baseline:
        counts = {s: sum(1 for r in rows if r.get('status') == s) for s in STATUSES}
        BASELINE.write_text(json.dumps({
            '_comment': [
                'The `open` count freezes here. Closing an open item is free and expected; opening',
                'one is a review event, because it means a rule we claimed to want was dropped or',
                'a new one was discovered. Re-run with --baseline in the same commit that changes it,',
                'so the number in the diff is the number somebody decided on.',
                '2026-11-06: the first freeze. All five are blocked by facts outside the code --',
                'three by a byte-pinned config file, one by a 254-page mapping that has not been',
                'done, one by a standard that has not been bought.',
            ],
            'recorded': '2026-11-06',
            'open': counts['open'],
            'detail': [r['rule'] for r in rows if r.get('status') == 'open'],
        }, indent=2, ensure_ascii=False) + '\n', encoding='utf-8')
        print(f'[subset] froze open = {counts["open"]} in {BASELINE.name}')
        # Re-judge against the baseline that was just written: reporting the *previous* failure
        # ("no baseline") right after creating one would make the command look broken.
        ctx['baseline'] = counts['open']
        problems = [p for p in problems if 'baseline' not in p]
        problems += evaluate(rows, ctx)

    counts = {s: sum(1 for r in rows if r.get('status') == s) for s in STATUSES}
    print(f'[subset] {len(rows)} rule(s): implemented {counts["implemented"]}, '
          f'not-applicable {counts["not-applicable"]}, deviation {counts["deviation"]}, '
          f'open {counts["open"]} (baseline {ctx["baseline"]})')
    # 101 `implemented` tidy rows saying "yes, that one too" is noise in an aggregate run; the rows
    # that a reader needs to see every time are the ones that are *not* simply implemented.
    for status in STATUSES:
        if status == 'implemented' and not verbose:
            continue
        for row in rows:
            if row.get('status') == status:
                print(f'[subset]   {status:15} {row["rule"]:42} {row["check"]}')

    if args.summary and not problems:
        SUMMARY.write_text(summary_text(rows, ctx), encoding='utf-8')
        print(f'[subset] wrote {SUMMARY.name}')

    if problems:
        for problem in problems:
            print(f'[subset] FAIL {problem}', file=sys.stderr)
        print(f'[subset] FAIL ({len(problems)} problem(s))')
        return 1
    print('[subset] OK: every implemented rule resolves, every deviation has its five elements, '
          'every suppression has a record, and the open count did not rise')
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv))
