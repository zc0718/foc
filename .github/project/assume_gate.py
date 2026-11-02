#!/usr/bin/env python3
"""Check the assumption register: every cross-cutting assumption names a falsifier that exists.

`claim-basis.json` records what each claim *rests on* (dependency edges) and `claim_gate.py` checks that a
basis has not rotted. That is a statement about the graph, not about the world: a claim can have an intact
basis, green tests, and still be wrong because an assumption underneath it is wrong. This gate is the other
half -- each assumption must name something that fails when the assumption is false, and an assumption we
cannot falsify yet must be listed as a gap with the thing it is waiting for. Gaps may only shrink.

    python3 .github/misc/assume_gate.py            # what CI runs
    python3 .github/misc/assume_gate.py --list     # the register, gap first
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REGISTER = Path(__file__).resolve().with_name("assumptions.json")
GATE_ENTRY_POINTS = (ROOT / ".github" / "misc" / "gates.py", ROOT / ".github" / "project" / "gates.py")
UNIT_TESTS = ROOT / "test_package" / "test" / "unit"
INCLUDE = ROOT / "include" / "foc"
STRENGTHS = {"measured", "argued", "unverified"}
FIELDS = ("id", "statement", "if_wrong", "affects", "falsifier", "strength", "reviewed")


def gate_names() -> set[str]:
    """Every gate name the project accepts, read from the help tables rather than restated here.

    Two entry points, on purpose: the scaffold's `.github/misc/gates.py` (format/tidy/guard) must stay
    byte-identical to upstream, so this project's own gates live in `.github/project/gates.py`. A falsifier may
    name either -- `format` and one of ours are equally good proof that a claim can be caught."""
    names: set[str] = set()
    for path in GATE_ENTRY_POINTS:
        if not path.is_file():
            continue
        block = path.read_text(encoding="utf-8").split("helps = {", 1)[1].split("\n    }", 1)[0]
        names |= set(re.findall(r"^\s*'([a-z][a-z-]*)':", block, re.M))
    return names


def affects_exists(pattern: str) -> bool:
    """An `affects` entry is a header path under include/foc, with an optional trailing `*` for a directory."""
    if pattern.endswith("*.hpp"):
        return any(INCLUDE.rglob(Path(pattern).name))
    return (INCLUDE / pattern).is_file()


def falsifier_problem(falsifier: dict, gates: set[str]) -> str | None:
    kind = falsifier.get("kind")
    ref = falsifier.get("ref") or ""
    if kind == "test":
        return None if (UNIT_TESTS / Path(ref).name).is_file() else f"test '{ref}' does not exist"
    if kind == "gate":
        return None if ref in gates else f"gate '{ref}' is not one of {sorted(gates)}"
    return f"kind '{kind}' is not one of ['gate', 'test']"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="print the register instead of only the verdict")
    args = ap.parse_args()

    if not REGISTER.is_file():
        print(f"[assume] FAIL: no register at {REGISTER}")
        return 1
    data = json.loads(REGISTER.read_text(encoding="utf-8"))
    gates = gate_names()
    problems: list[str] = []
    assumptions = data.get("assumptions") or []
    gaps = data.get("gaps") or []
    baseline = data.get("gaps_baseline")

    seen: set[str] = set()
    for a in assumptions:
        ident = a.get("id", "?")
        missing = [f for f in FIELDS if not a.get(f)]
        if missing:
            problems.append(f"{ident}: missing {missing}")
            continue
        if ident in seen:
            problems.append(f"{ident}: duplicate id")
        seen.add(ident)
        if a["strength"] not in STRENGTHS:
            problems.append(f"{ident}: strength '{a['strength']}' is not one of {sorted(STRENGTHS)}")
        for pattern in a["affects"]:
            if not affects_exists(pattern):
                problems.append(f"{ident}: affects '{pattern}' is not a header under include/foc/")
        why = falsifier_problem(a["falsifier"], gates)
        if why:
            problems.append(f"{ident}: falsifier unusable -- {why}")
        if not re.fullmatch(r"\d{4}-\d{2}-\d{2}", str(a["reviewed"])):
            problems.append(f"{ident}: reviewed '{a['reviewed']}' is not YYYY-MM-DD")

    for g in gaps:
        if g.get("id") in seen:
            problems.append(f"{g.get('id')}: listed both as an assumption and as a gap")
        for field in ("id", "statement", "needs"):
            if not g.get(field):
                problems.append(f"{g.get('id', '?')}: gap is missing '{field}'")

    if baseline is None:
        problems.append("gaps_baseline is missing: the gap count has to be pinned so it can only shrink")
    elif len(gaps) > baseline:
        problems.append(f"gaps grew to {len(gaps)} from a baseline of {baseline}; an assumption without a "
                        f"falsifier is what this file exists to stop, so either write the falsifier or raise "
                        f"the baseline deliberately")

    if args.list:
        print(f"[assume] {len(assumptions)} assumption(s), {len(gaps)} gap(s), baseline {baseline}")
        for g in gaps:
            print(f"  GAP  {g['id']:6} {g['statement'][:96]}")
        for a in assumptions:
            f = a["falsifier"]
            print(f"  ok   {a['id']:6} {a['strength']:8} {f['kind']}:{Path(str(f['ref'])).name[:44]:44} "
                  f"{a['statement'][:70]}")

    for p in problems:
        print(f"[assume] {p}")
    if problems:
        print(f"[assume] FAIL: {len(problems)} problem(s) in the assumption register")
        return 1
    print(f"[assume] OK ({len(assumptions)} assumption(s) falsifiable, {len(gaps)} gap(s), "
          f"baseline {baseline})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
