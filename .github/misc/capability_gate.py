#!/usr/bin/env python3
"""Capability gate: every capability states its diagnostic channel, its admission, its boundary and its invariant.

Why this exists. The breadth plan's real risk is not that a new algorithm cannot be written; it is that a new algorithm
arrives without the three things that make it debuggable, and nothing notices until field support. The wave that closed
this library's depth axis learned the shape of the omission five times over (see `foundation-log.rst`): an observer that
holds its last angle and reports zero speed, a mode admitted on a switch instead of on a fact, a limit nobody wrote down.
A checklist in a document does not survive; a table whose cells are checked does.

The table is `docs/sphinx/source/capabilities.rst`. Six columns, fixed grammar:

    diagnostic channel | admission | boundary | invariant | ledger

Cell grammar (enforced, because a column of prose is a column nobody parses):

    <Suite.Case>                 a TEST() that exists
    none -- <reason>             no limit is known; requires an L3 claim behind the header
    na -- <reason>               the dimension does not apply; the reason is required
    todo -- <reason>             a known hole, counted and printed on every run

Scope. Every header under a covered root (`core/`, `motor/`) must be either listed in the table or named in
`capability-scope.json` as deliberately unlisted. The unlisted count is printed; a new header has to be one or the other,
so a capability cannot be added with no row at all.

Usage:
    python3 .github/misc/capability_gate.py            # check
    python3 .github/misc/capability_gate.py --verbose  # also print every row
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
PAGE = ROOT / "docs" / "sphinx" / "source" / "capabilities.rst"
SCOPE = ROOT / ".github" / "misc" / "capability-scope.json"
INCLUDE = ROOT / "include" / "foc"
README = ROOT / "README.md"
UNIT = ROOT / "test_package" / "test" / "unit"

COLUMNS = 6
MANAGED = re.compile(r"^(none|na|todo)\s+--\s+(\S.{19,})$")
CASE = re.compile(r"^``([A-Z][A-Za-z0-9_]*)\.([A-Za-z0-9_]+)``$")
HEADER = re.compile(r"^``([^`]+\.(?:hpp|h))``$")


def installed(module: str) -> bool:
    return (INCLUDE / module).is_file()


def covered() -> list[str]:
    scope = json.loads(SCOPE.read_text(encoding="utf-8"))
    out = []
    for root in scope["covered_roots"]:
        for path in (INCLUDE / root).rglob("*"):
            if path.suffix in {".hpp", ".h"}:
                out.append(str(path.relative_to(INCLUDE)).replace("\\", "/"))
    return sorted(out)


def test_index() -> set[str]:
    """Every `TEST(Suite, Case)` in the suite, as `Suite.Case`."""
    found = set()
    for path in list(UNIT.rglob("*.cpp")) + list((ROOT / "test_package" / "test").rglob("*.cpp")):
        text = path.read_text(encoding="utf-8", errors="replace")
        for suite, case in re.findall(r"^TEST\(\s*(\w+)\s*,\s*(\w+)\s*\)", text, re.MULTILINE):
            found.add(f"{suite}.{case}")
    return found


def claim_levels() -> dict[str, str]:
    """module -> level, from the README claim table (the same table the claims gate reads)."""
    text = README.read_text(encoding="utf-8")
    marker = "### What each module claims"
    if marker not in text:
        return {}
    body = re.split(r"\n#{2,3} ", text.split(marker, 1)[1], maxsplit=1)[0]
    levels = {}
    for line in body.splitlines():
        line = line.strip()
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        if len(cells) < 4 or set(cells[0]) <= set("-: ") or cells[0].lower().startswith("module"):
            continue
        level = cells[1].strip("` *")
        for module in (m.strip("` ").removeprefix("foc/") for m in cells[0].split(",") if m.strip("` ")):
            levels[module] = level
    return levels


def ledger_ids() -> set[str]:
    page = ROOT / "docs" / "sphinx" / "source" / "foundation-log.rst"
    if not page.is_file():
        return set()
    return set(re.findall(r"\b([FCA]-\d+)\b", page.read_text(encoding="utf-8")))


def invariant_cases() -> set[str]:
    path = UNIT / "test_invariants.cpp"
    if not path.is_file():
        return set()
    text = path.read_text(encoding="utf-8")
    return set(re.findall(r"^TEST\(Invariants\s*,\s*(\w+)\s*\)", text, re.MULTILINE))


def table_rows() -> list[list[str]]:
    """The capability table, as rows of cells. Blank continuation lines belong to the previous cell's row."""
    text = PAGE.read_text(encoding="utf-8")
    rows: list[list[str]] = []
    current: list[str] | None = None
    for raw in text.splitlines():
        line = raw.rstrip()
        if line.lstrip().startswith(".. list-table"):
            rows.clear()
            current = None
            continue
        stripped = line.strip()
        if stripped.startswith("* - "):
            if current is not None:
                rows.append(current)
            current = [stripped[4:].strip()]
            continue
        if stripped == "* -" or stripped.startswith("- "):
            if current is not None:
                current.append(stripped[2:].strip())
            continue
        if current is not None and stripped:
            # A continuation line belongs to the row only while it is indented; the prose after the
            # table starts at column 0 and must not be swallowed into the last cell.
            # 续行只有在缩进时才属于该行；表后的正文从第 0 列开始，不得被吞进最后一格。
            if not raw.startswith((" ", "\t")):
                rows.append(current)
                current = None
                continue
            current[-1] = f"{current[-1]} {stripped}".strip()
    if current is not None:
        rows.append(current)
    return rows


def normalized(cell: str) -> str:
    return " ".join(cell.split())


def main() -> int:
    verbose = "--verbose" in sys.argv
    if not PAGE.is_file():
        print("[capabilities] FAIL: docs/sphinx/source/capabilities.rst is missing", file=sys.stderr)
        return 1
    if not SCOPE.is_file():
        print("[capabilities] FAIL: .github/misc/capability-scope.json is missing", file=sys.stderr)
        return 1

    problems: list[str] = []
    rows = table_rows()
    header_row = [c for c in rows[0]] if rows else []
    if [c.lower() for c in header_row][:COLUMNS] != [
        "capability",
        "diagnostic channel",
        "admission",
        "boundary",
        "invariant",
        "ledger",
    ]:
        problems.append(f"the table header must be the six fixed columns, got: {header_row}")
    body = rows[1:]

    cases = test_index()
    levels = claim_levels()
    ledger = ledger_ids()
    invariants = invariant_cases()
    listed: set[str] = set()

    for row in body:
        if len(row) < COLUMNS - 1:
            problems.append(f"row with {len(row)} cell(s) (the table has {COLUMNS}): {row[0][:60]}")
            continue
        cells = [normalized(c) for c in row] + [""] * COLUMNS
        header_cell, diag, admit, bound, invariant, ledger_cell = cells[:COLUMNS]

        header = HEADER.match(header_cell)
        if not header:
            problems.append(f"capability cell must be a header path in double backticks: '{header_cell[:60]}'")
            continue
        module = header.group(1)
        listed.add(module)
        if not installed(module):
            problems.append(f"{module}: not an installed header")

        for name, value in (("diagnostic channel", diag), ("admission", admit)):
            if not value:
                problems.append(f"{module}: {name} is empty")
            elif not (MANAGED.match(value) or len(value) >= 20):
                problems.append(f"{module}: {name} says '{value[:40]}', which is neither a channel nor 'na -- reason'")

        if not bound:
            problems.append(f"{module}: boundary is empty")
            continue
        case = CASE.match(bound)
        managed = MANAGED.match(bound)
        if case:
            name = f"{case.group(1)}.{case.group(2)}"
            if name not in cases:
                problems.append(f"{module}: boundary names '{name}', which is not a TEST() in the suite")
        elif managed:
            token = managed.group(1)
            if token == "none" and levels.get(module) != "L3":
                problems.append(
                    f"{module}: boundary says 'none' but the claim is {levels.get(module, 'absent')} -- claiming no "
                    "known limit needs an L3 claim behind it"
                )
        else:
            problems.append(f"{module}: boundary must be ``Suite.Case`` or 'none|na|todo -- reason', got '{bound[:50]}'")

        if not invariant:
            problems.append(f"{module}: invariant participation is empty")
        elif not MANAGED.match(invariant):
            for part in (p.strip() for p in invariant.split(",")):
                if part and part not in invariants:
                    problems.append(f"{module}: invariant '{part}' is not a TEST(Invariants, ...) case")
        if ledger_cell not in {"--", "—"}:
            for token in (t.strip() for t in re.split(r"[,;]", ledger_cell)):
                if token and token not in ledger:
                    problems.append(f"{module}: ledger row '{token}' is not in foundation-log.rst")

        if verbose:
            print(f"  {module:46s} boundary={bound[:28]!r} invoice={invariant[:20]!r}")

    # Scope: every covered header is either listed or deliberately unlisted, and neither list may rot.
    scope = json.loads(SCOPE.read_text(encoding="utf-8"))
    unlisted = set(scope["unlisted"])
    everything = set(covered())
    for module in sorted(unlisted - everything):
        problems.append(f"capability-scope.json lists '{module}' as unlisted, but it is not a header under a covered root")
    for module in sorted(unlisted & listed):
        problems.append(f"'{module}' is both listed and unlisted; drop it from the scope file")
    for module in sorted(everything - unlisted - listed):
        problems.append(
            f"{module} is a capability with no row: list it on capabilities.rst, or add it to capability-scope.json "
            "with a reason"
        )
    for module in sorted(listed - everything):
        problems.append(f"'{module}' is listed but is not under a covered root in capability-scope.json")

    if problems:
        print(f"[capabilities] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1

    todo = sum(1 for row in body if len(row) >= COLUMNS and normalized(row[3]).startswith("todo"))
    na_bound = sum(1 for row in body if len(row) >= COLUMNS and normalized(row[3]).startswith("na"))
    print(
        f"[capabilities] OK ({len(listed)} of {len(everything)} capability header(s) listed, "
        f"{len(unlisted)} unlisted, {todo} boundary hole(s), {na_bound} not-applicable)"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
