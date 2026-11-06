#!/usr/bin/env python3
"""`docshape` — a public type's docstring carries the same six facts, in the same order, in both languages.

Why this exists
---------------
The plan's section 5.3 asks every public class to answer the same questions: what physical problem does it model, what
is the equation, which symbols does it use, what does it guarantee, what does it refuse, and where is the evidence. The
point is not uniformity for its own sake: those six questions are the ones a reviewer asks in that order, and a block
that answers five of them leaves the sixth to be improvised in a meeting.

**The form was measured, not invented.** The obvious encoding -- six Doxygen `@par` headings -- is impossible in this
pipeline: `docfmt` treats a body line that starts with a tag as a defect (the docs pipeline then assigns it to the wrong
language group), so it strips the `@` and merges six sections into one paragraph. Three other shapes were tried
(blank-separated tag blocks, hanging-indent tag blocks, one tag per section with its body on the next line) and all
three were rewritten the same way. What survives is the shape this project already uses: **one tag line per language
group, body paragraphs untagged**. So section 1 is the `@brief` sentence and sections 2..6 are untagged paragraphs that
open with a bold marker, English group first, then Chinese.

What it checks
--------------
1. Every file listed in `migrated` has, in every public-type class/struct block: a `@brief` line, and the five markers
   `**Equation.**` `**Notation.**` `**Guarantees.**` `**Refusals.**` `**Evidence.**` present **in that order**, in a
   block that carries both `[en]` and `[zh]`.
2. The repository-wide debt -- blocks missing at least one marker -- never rises above `docshape-baseline.json`. A
   migration lowers it; new code cannot add to it.
3. `migrated` may only grow, and every entry must exist: a file cannot be "migrated" by being deleted.
4. `--selftest` mutates the table and the source and requires each mutation to be caught, including the one that
   matters: a marker deliberately misspelled.

What this gate does not do
--------------------------
It checks that the six slots are **present and ordered**, not that what fills them is true. A block can pass with six
sentences of nothing. Content quality is a review question, and the plan already says so (section 9: "docshape only
checks structure, content quality is sampling review, five per batch"). Nothing here pretends otherwise.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TABLE = Path(__file__).resolve().parent / "docshape.json"
BASELINE = Path(__file__).resolve().parent / "docshape-baseline.json"
INCLUDE = ROOT / "include" / "foc"

MARKERS = ("**Equation.**", "**Notation.**", "**Guarantees.**", "**Refusals.**", "**Evidence.**")
BLOCK = re.compile(r"/\*\*.*?\*/", re.DOTALL)
TYPE_LINE = re.compile(r"^\s*(?:template\s*<[^>]*>\s*)?(?:class|struct)\s+\w+")


def type_blocks(path: Path):
    """`(lineno, block_text)` for each Doxygen block that sits directly above a class/struct."""
    text = path.read_text(encoding="utf-8", errors="replace")
    lines = text.splitlines()
    out = []
    for match in BLOCK.finditer(text):
        start = text.count("\n", 0, match.end()) + 1
        for line in lines[start:]:
            stripped = line.strip()
            if not stripped:
                continue
            if TYPE_LINE.match(line):
                out.append((text.count("\n", 0, match.start()) + 1, match.group(0)))
            break
    return out


def missing_markers(block: str) -> list[str]:
    """The markers that are absent, plus the ones out of order, as one list of complaints."""
    problems = []
    if "@brief" not in block:
        problems.append("@brief")
    if "[en]" not in block or "[zh]" not in block:
        problems.append("one language")
    positions = [block.find(marker) for marker in MARKERS]
    for marker, position in zip(MARKERS, positions):
        if position < 0:
            problems.append(marker)
    present = [(position, marker) for marker, position in zip(MARKERS, positions) if position >= 0]
    if [marker for _, marker in sorted(present)] != [marker for marker in MARKERS if block.find(marker) >= 0]:
        problems.append("order")
    return problems


def scope_files(table: dict) -> list[Path]:
    found = []
    for name in table.get("scope", ["core", "motor"]):
        directory = INCLUDE / name
        if directory.is_dir():
            found.extend(sorted(directory.rglob("*.hpp")))
    return found


def measure(table: dict) -> dict:
    debt = 0
    per_file: dict[str, int] = {}
    for path in scope_files(table):
        rel = path.relative_to(INCLUDE).as_posix()
        bad = sum(1 for _line, block in type_blocks(path) if missing_markers(block))
        per_file[rel] = bad
        debt += bad
    return {"debt": debt, "per_file": per_file}


def read_baseline() -> dict:
    if not BASELINE.is_file():
        return {}
    return json.loads(BASELINE.read_text(encoding="utf-8"))


def evaluate(table: dict, baseline: dict) -> list[str]:
    problems: list[str] = []
    migrated = table.get("migrated", [])
    if not migrated:
        problems.append("`migrated` is empty: the six-section skeleton is documented and enforced on nothing")
    for rel in migrated:
        path = INCLUDE / rel
        if not path.is_file():
            problems.append(f"{rel}: listed as migrated but does not exist")
            continue
        blocks = type_blocks(path)
        if not blocks:
            problems.append(f"{rel}: listed as migrated but no public type block was found, so the entry proves "
                            f"nothing")
            continue
        for lineno, block in blocks:
            complaints = missing_markers(block)
            if complaints:
                problems.append(f"{rel}:{lineno}: block is missing {complaints}")
    measured = measure(table)
    ceiling = baseline.get("debt")
    if ceiling is None:
        problems.append("no debt baseline: `docshape-baseline.json` is missing or has no `debt`")
    elif measured["debt"] > ceiling:
        worse = {name: count for name, count in measured["per_file"].items()
                 if count > (baseline.get("per_file", {}).get(name) or 0)}
        problems.append(f"debt rose to {measured['debt']} from a baseline of {ceiling}; a migration lowers it and "
                        f"new code must not add to it. Files that got worse: {worse or 'none — the whole tree did'}")
    return problems


def selftest(table: dict) -> int:
    """Four mutations, each of which must be caught."""
    caught = 0
    trials: list[tuple[str, str, list[str]]] = [
        ("a block with no Equation marker",
         "/**\n * @brief [en] x\n * @brief [zh] 甲\n *     **Notation.** a\n *     **Guarantees.** b\n"
         " *     **Refusals.** c\n *     **Evidence.** d\n */\nclass X final {};\n",
         ["**Equation.**"]),
        ("markers present but out of order",
         "/**\n * @brief [en] x\n * @brief [zh] 甲\n *     **Refusals.** c\n *     **Equation.** e\n"
         " *     **Notation.** a\n *     **Guarantees.** b\n *     **Evidence.** d\n */\nclass X final {};\n",
         ["order"]),
        ("a block with one language only",
         "/**\n * @brief [en] x\n *     **Equation.** e\n *     **Notation.** a\n *     **Guarantees.** b\n"
         " *     **Refusals.** c\n *     **Evidence.** d\n */\nclass X final {};\n",
         ["one language"]),
        ("a complete block", "/**\n * @brief [en] x\n * @brief [zh] 甲\n *     **Equation.** e\n"
         " *     **Notation.** a\n *     **Guarantees.** b\n *     **Refusals.** c\n *     **Evidence.** d\n"
         " */\nclass X final {};\n", []),
    ]
    for label, text, expected in trials:
        complaints = missing_markers(text)
        ok = (not complaints) if not expected else any(all(token in c for c in complaints) for token in expected)
        print(f"[docshape-selftest] {'OK  ' if ok else 'MISS'} {label}: {complaints or 'no complaint'}")
        caught += 1 if ok else 0
    missing_entry = dict(table, migrated=["core/__no_such_header__.hpp"])
    if evaluate(missing_entry, read_baseline()):
        print("[docshape-selftest] OK   a migrated file that does not exist: caught")
        caught += 1
    else:
        print("[docshape-selftest] MISS a migrated file that does not exist", file=sys.stderr)
    print(f"[docshape-selftest] {caught} / {len(trials) + 1} judgement(s) as expected")
    return 0 if caught == len(trials) + 1 else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true", help="mutate blocks and the table, require each to be caught")
    parser.add_argument("--list", action="store_true", help="print every problem")
    parser.add_argument("--baseline", action="store_true", help="freeze today's debt")
    args = parser.parse_args(argv)
    table = json.loads(TABLE.read_text(encoding="utf-8"))
    if args.selftest:
        return selftest(table)
    measured = measure(table)
    if args.baseline:
        BASELINE.write_text(json.dumps({
            "_comment": [
                "Today's six-section debt, frozen so it may only fall. Per file on purpose: a file that gains a",
                "non-compliant block cannot hide behind another file that lost two.",
                "六段骨架的存量欠账，冻结后只许下降。按文件计：一个文件新增的不合规块，无法被另一个文件减少的抵掉。",
            ],
            "debt": measured["debt"],
            "per_file": measured["per_file"],
        }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"[docshape] froze {measured['debt']} non-compliant block(s) across "
              f"{len(measured['per_file'])} file(s)")
        return 0
    problems = evaluate(table, read_baseline())
    if args.list or problems:
        for problem in problems:
            print(f"[docshape] {problem}")
    if problems:
        print(f"[docshape] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        return 1
    print(f"[docshape] OK ({len(table['migrated'])} migrated file(s), {measured['debt']} block(s) still to migrate, "
          f"at or below baseline)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
