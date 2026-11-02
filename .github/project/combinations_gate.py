#!/usr/bin/env python3
"""Combination licence matrix for the (reference generator x observer) slot pairings.

Why this exists
---------------
`Motor<Observer, Modulator, Controller, Hal, RefGenerator, ...>` takes both as template parameters, so every
pairing compiles. Nothing in the type system knows that `MtpaReference` asks for a d-axis current the
observer in the slot may not be able to see, and the result of getting it wrong is not a compile error -- it
is a plausible angle that is wrong. `workspace/drafts/foc-maturity-to-L3-plan.md` §C8 asks for the known
boundaries to become a *decidable refusal*; this gate is the decidable half.

What it checks
--------------
1. Every (reference generator x observer) pair discovered in the headers has a cell. `*` is a wildcard that
   covers the rest, so "we forgot" is red and "we decided it is the same for all" is written down.
2. Every cell has a verdict from {allow, conditional, forbid, unmeasured}.
3. `conditional` needs a `condition` and an `enforced_by`; `unmeasured` needs a `why` and a `reason`.
4. `forbid` needs a `falsifier` naming a case that exists -- "do not do this" is a claim like any other.
5. Every file named in `evidence` exists.

`--selftest` mutates a load of the real matrix four ways and requires each to be caught, because a gate that
has never been red is a gate nobody has any reason to trust.
"""

from __future__ import annotations

import argparse
import copy
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MATRIX = Path(__file__).resolve().parent / "combinations.json"
REFERENCE_HEADER = ROOT / "include" / "foc" / "core" / "reference.hpp"
OBSERVER_DIR = ROOT / "include" / "foc" / "core" / "observer"
VERDICTS = {"allow", "conditional", "forbid", "unmeasured"}

# `class X final` at the start of a line, optionally behind a `template<...>`: the blend is a template, and
# the null generator is named `...Generator` rather than `...Reference`. Both are slot policies and both have
# to appear in the pair set, or the matrix would be silent about exactly the rows nobody thought about.
# 行首的 `class X final`，可能位于 `template<...>` 之后：混合观测器是模板，而空生成器叫 `...Generator`
# 而不是 `...Reference`。两者都是槽位策略，都必须出现在配对集合里，否则矩阵恰好会对没人想过的那些行保持
# 沉默。
_CLASS = re.compile(r"^\s*(?:template\s*<[^>]*>\s*)?class\s+(\w*(?:Reference|Generator|Observer))\s+final", re.M)


def _names(path: Path) -> set[str]:
    if not path.exists():
        return set()
    return set(_CLASS.findall(path.read_text(encoding="utf-8")))


def discovered() -> tuple[set[str], set[str]]:
    references = _names(REFERENCE_HEADER)
    observers: set[str] = set()
    if OBSERVER_DIR.is_dir():
        for header in sorted(OBSERVER_DIR.glob("*.hpp")):
            observers |= _names(header)
    return references, observers


def _covered(cells: list[dict], reference: str, observer: str) -> bool:
    for cell in cells:
        if cell.get("reference") in (reference, "*") and cell.get("observer") in (observer, "*"):
            return True
    return False


def _case_exists(name: str) -> bool:
    for path in (ROOT / "test_package" / "test").rglob("*.cpp"):
        if name in path.read_text(encoding="utf-8", errors="replace"):
            return True
    return False


def check(matrix: dict) -> list[str]:
    problems: list[str] = []
    cells = matrix.get("pairs")
    if not isinstance(cells, list) or not cells:
        return ["the matrix has no `pairs` list"]

    references, observers = discovered()
    if not references or not observers:
        problems.append("could not discover the pair set from the headers, so coverage cannot be checked")

    for index, cell in enumerate(cells):
        where = f"cell {index} ({cell.get('reference')} x {cell.get('observer')})"
        verdict = cell.get("verdict")
        if verdict not in VERDICTS:
            problems.append(f"{where}: verdict {verdict!r} is not one of {sorted(VERDICTS)}")
            continue
        if verdict == "conditional":
            for field in ("condition", "enforced_by"):
                if not str(cell.get(field, "")).strip():
                    problems.append(f"{where}: a conditional cell needs `{field}`")
        if verdict == "unmeasured":
            for field in ("why", "reason"):
                if not str(cell.get(field, "")).strip():
                    problems.append(f"{where}: an unmeasured cell needs `{field}`")
        if verdict == "forbid":
            falsifier = str(cell.get("falsifier", "")).strip()
            if not falsifier:
                problems.append(f"{where}: a forbidden pairing needs a `falsifier`")
            elif not _case_exists(falsifier):
                problems.append(f"{where}: falsifier {falsifier!r} is not a case in the test tree")
        evidence = cell.get("evidence", [])
        if not evidence:
            problems.append(f"{where}: no `evidence`, so the verdict is unsupported")
        for relative in evidence:
            if not (ROOT / relative).exists():
                problems.append(f"{where}: evidence file {relative!r} does not exist")

    for reference in sorted(references):
        for observer in sorted(observers):
            if not _covered(cells, reference, observer):
                problems.append(f"pair ({reference}, {observer}) has no verdict and is not covered by a wildcard")

    return problems


def _selftest() -> int:
    matrix = json.loads(MATRIX.read_text(encoding="utf-8"))
    failures = 0

    def expect_red(name: str, mutated: dict) -> None:
        nonlocal failures
        problems = check(mutated)
        if problems:
            print(f"[combinations] selftest OK   {name}: {problems[0]}")
        else:
            print(f"[combinations] selftest FAIL {name}: the mutation was accepted")
            failures += 1

    baseline = check(matrix)
    if baseline:
        print("[combinations] selftest FAIL the unmutated matrix is already red:")
        for problem in baseline:
            print(f"  - {problem}")
        return 1
    print("[combinations] selftest OK   unmutated matrix is green")

    # 1. A verdict that is not one of the four.
    bad = copy.deepcopy(matrix)
    bad["pairs"][0]["verdict"] = "probably fine"
    expect_red("unknown verdict", bad)

    # 2. A forbidden pairing with no falsifier.
    bad = copy.deepcopy(matrix)
    bad["pairs"].append({"reference": "MtpaReference", "observer": "FakeObserver", "verdict": "forbid",
                         "why": "test", "evidence": ["test_package/test/unit/test_mtpa.cpp"]})
    expect_red("forbid without a falsifier", bad)

    # 3. Evidence pointing at a file that does not exist.
    bad = copy.deepcopy(matrix)
    bad["pairs"][0]["evidence"] = ["test_package/test/unit/does_not_exist.cpp"]
    expect_red("missing evidence file", bad)

    # 4. A conditional cell with no condition. This is the one that matters most: an unconditional
    #    "conditional" is a pairing nobody has to think about.
    bad = copy.deepcopy(matrix)
    for cell in bad["pairs"]:
        if cell["verdict"] == "conditional":
            cell.pop("condition", None)
            break
    expect_red("conditional without a condition", bad)

    # 5. Coverage: drop the wildcard that covers the observers nobody wrote a row for.
    # Drop every wildcard, so the observers with no explicit row lose their verdict but keep being discovered.
    # 去掉所有通配格，于是那些没有专门行的观测器失去结论、却仍然会被发现。
    bad = copy.deepcopy(matrix)
    bad["pairs"] = [cell for cell in bad["pairs"] if cell.get("observer") != "*"]
    expect_red("an uncovered pair", bad)

    if failures:
        print(f"[combinations] selftest FAIL {failures} of 5", file=sys.stderr)
        return 1
    print("[combinations] selftest OK (5/5)")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--selftest", action="store_true", help="mutate the matrix and require each to be caught")
    args = parser.parse_args()

    if args.selftest:
        return _selftest()

    if not MATRIX.exists():
        print(f"[combinations] {MATRIX} is missing", file=sys.stderr)
        return 1
    matrix = json.loads(MATRIX.read_text(encoding="utf-8"))
    problems = check(matrix)
    if problems:
        for problem in problems:
            print(f"[combinations] {problem}", file=sys.stderr)
        print(f"[combinations] FAIL {len(problems)} problem(s)", file=sys.stderr)
        return 1

    references, observers = discovered()
    counts: dict[str, int] = {}
    for cell in matrix["pairs"]:
        counts[cell["verdict"]] = counts.get(cell["verdict"], 0) + 1
    summary = ", ".join(f"{counts.get(verdict, 0)} {verdict}" for verdict in ("allow", "conditional", "forbid", "unmeasured"))
    print(f"[combinations] OK ({len(references)} reference(s) x {len(observers)} observer(s), "
          f"{len(matrix['pairs'])} cell(s): {summary})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
