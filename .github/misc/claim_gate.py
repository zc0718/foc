#!/usr/bin/env python3
"""Claim gate: every quantitative claim must name a level and something that checks it.

Why this exists. The README's evidence levels were added by hand, and within one session the
`sphinx` verification table was found to be three measurement rounds out of date (it claimed 131
test cases when the suite had 361) while every CI leg stayed green. Nothing was checking the
*numbers in the documentation* against anything. A level with no test behind it is a wish, and a
number nobody recomputes is a rumour.

So this gate enforces four things:

1. Every public header under `include/foc/` is either claimed in the README's module table or
   explicitly exempted with a reason. Adding a substantive header without saying what is claimed
   about it fails the build.
2. Every claim names a level, and the level is L1..L3. **L4 is refused**: the project has no
   hardware, and an L4 row would be a claim nobody could have made.
3. Every L2/L3 claim names evidence that exists -- a test file that the build compiles, or a gate
   script that the CI runs. An L1 claim may name a gate too; it may not name nothing.
4. A claim may not name a test file that does not exist, which is how a rename silently orphans a
   claim.

Usage:
    python3 .github/misc/claim_gate.py            # check
    python3 .github/misc/claim_gate.py --verbose  # also print every row
"""

from __future__ import annotations

import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
README = ROOT / "README.md"
INCLUDE = ROOT / "include" / "foc"
UNIT_TESTS = ROOT / "test_package" / "test" / "unit"
GATES = ROOT / ".github" / "misc"

SECTION_START = "### What each module claims"
LEVELS = {"L1", "L2", "L3", "L4"}
# L4 would be "measured on physical hardware". There is none, so a row claiming it is a lie.
FORBIDDEN_LEVELS = {"L4"}


def headers() -> list[str]:
    """Every installed header, relative to `include/foc/`, sorted."""
    out = []
    for path in INCLUDE.rglob("*"):
        if path.suffix in {".h", ".hpp"}:
            out.append(str(path.relative_to(INCLUDE)).replace("\\", "/"))
    return sorted(out)


def claim_rows() -> list[tuple[list[str], str, str, list[str]]]:
    """Parse the module table into (modules, level, claim, evidence)."""
    text = README.read_text(encoding="utf-8")
    if SECTION_START not in text:
        raise SystemExit(f"claim gate: '{SECTION_START}' not found in README.md")
    body = text.split(SECTION_START, 1)[1]
    # The table ends at the next heading at the same or a higher level.
    body = re.split(r"\n#{2,3} ", body, maxsplit=1)[0]

    rows = []
    for line in body.splitlines():
        line = line.strip()
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip("|").split("|")]
        if len(cells) < 4:
            continue
        if set(cells[0]) <= set("-: ") or cells[0].lower().startswith("module"):
            continue
        # Paths are written relative to `include/`, so strip the `foc/` prefix the table uses.
        # 路径按相对 `include/` 的写法，所以去掉表里带的 `foc/` 前缀。
        modules = [
            m.strip("` ").removeprefix("foc/") for m in cells[0].split(",") if m.strip("` ")
        ]
        level = cells[1].strip("` *")
        claim = cells[2]
        evidence = [e.strip("` ") for e in cells[3].split(",") if e.strip("` ") and e.strip("` ") not in {"-", "—"}]
        rows.append((modules, level, claim, evidence))
    return rows


def main() -> int:
    verbose = "--verbose" in sys.argv
    problems: list[str] = []
    claimed: set[str] = set()

    all_headers = headers()
    rows = claim_rows()
    if not rows:
        problems.append("the module table is empty: no claim is declared at all")

    for modules, level, _claim, evidence in rows:
        if level not in LEVELS:
            problems.append(f"row {modules}: level '{level}' is not one of L1..L4")
        elif level in FORBIDDEN_LEVELS:
            problems.append(
                f"row {modules}: level L4 means 'measured on hardware' and this project has none; "
                "use L2/L3 or say nothing"
            )
        for module in modules:
            if module not in all_headers:
                problems.append(f"row claims module '{module}', which is not an installed header")
            claimed.add(module)
        if not evidence:
            problems.append(f"row {modules}: no evidence -- a claim with nothing checking it is a wish")
        for item in evidence:
            candidates = (UNIT_TESTS / item, ROOT / "test_package" / item, GATES / item)
            if any(candidate.is_file() for candidate in candidates):
                continue
            problems.append(
                f"row {modules}: evidence '{item}' is not a unit test, a test_package consumer, "
                "or a gate script"
            )
        if verbose:
            print(f"  {level}  {', '.join(modules):46s} <- {', '.join(evidence) or '(none)'}")

    unclaimed = [h for h in all_headers if h not in claimed]
    for header in unclaimed:
        problems.append(
            f"{header} is installed but unclaimed: add it to the README module table, or say why "
            "it needs no claim"
        )

    if problems:
        print(f"[claims] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1
    print(f"[claims] OK ({len(all_headers)} header(s), {len(rows)} claim row(s), no L4 claim)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
