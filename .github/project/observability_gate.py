#!/usr/bin/env python3
"""Keeps the observability registry honest.

Why a gate and not a paragraph. The rule this enforces -- "when the foundation gains the ability to observe something new,
go back and re-read the evidence chain against it" -- is the kind of rule that is followed exactly as long as somebody
remembers, and then silently stops. Worse, the failure is invisible: nothing goes red when a new instrument arrives and
nobody asks whether it re-opens an old claim. So the registry is data and this script is the judgement, the same shape as
`assumptions.json`/`assume_gate.py` and `error-sources.json`/`budget_gate.py`.

Four things are checked, and the third is the one that matters most:

  1. every upgrade names at least one claim row it re-opened, or says why it re-opened none;
  2. every named claim exists in README's claim table, and its verdict agrees with the level README gives it;
  3. an **implementation-level** leg may not promote anything. Re-implementing the same rule in another language proves
     the code does what the code says. It says nothing about whether the rule is right, and a discipline that let it count
     would turn every port into an evidence upgrade -- which is the cheapest possible way to be wrong.
  4. every L2 claim whose evidence names a `test_sil_*` case is registered somewhere. The proxy and its blind spots are
     written down in the data file rather than left implicit.

`--selftest` mutates a good registry four ways and requires each mutation to be caught.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
REGISTRY = ROOT / ".github" / "project" / "observability.json"
README = ROOT / "README.md"

VERDICTS = {"promoted", "still_short"}
LEGS = {"implementation", "model"}


def claim_rows() -> dict[str, tuple[str, str]]:
    """`{module: (level, evidence)}` from README's claim table. Parsed here rather than imported so the gate has one
    dependency: the file it is judging."""
    rows: dict[str, tuple[str, str]] = {}
    for line in README.read_text(encoding="utf-8").splitlines():
        if not line.strip().startswith("|"):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 4 or cells[1] not in {"L1", "L2", "L3"}:
            continue
        module = cells[0].strip("` ")
        if module.startswith("foc/"):
            rows[module[len("foc/"):]] = (cells[1], cells[3])
    return rows


def check(data: dict, rows: dict[str, tuple[str, str]]) -> list[str]:
    problems: list[str] = []
    seen: set[str] = set()
    registered: set[str] = set()

    for upgrade in data.get("upgrades", []):
        ident = upgrade.get("id", "?")
        if ident in seen:
            problems.append(f"{ident}: duplicate id")
        seen.add(ident)
        leg = upgrade.get("leg")
        if leg not in LEGS:
            problems.append(f"{ident}: leg must be one of {sorted(LEGS)}, not {leg!r}")
        reopened = upgrade.get("reopened") or []
        if not reopened and not upgrade.get("no_effect"):
            problems.append(f"{ident}: re-opened nothing and gave no reason ('no_effect')")

        for item in reopened:
            claim = item.get("claim", "?")
            verdict = item.get("verdict")
            registered.add(claim)
            if claim not in rows:
                problems.append(f"{ident}: '{claim}' is not a claim row in README")
                continue
            level, evidence = rows[claim]
            if verdict not in VERDICTS:
                problems.append(f"{ident}/{claim}: verdict must be one of {sorted(VERDICTS)}")
            if verdict == "promoted":
                if leg == "implementation":
                    problems.append(
                        f"{ident}/{claim}: an implementation-level leg cannot promote -- a port is not a cross-model check")
                if level != "L3":
                    problems.append(f"{ident}/{claim}: verdict says promoted but README says {level}")
            if verdict == "still_short":
                if level == "L3":
                    problems.append(f"{ident}/{claim}: verdict says still shooting short but README already says L3")
                if not item.get("missing"):
                    problems.append(f"{ident}/{claim}: still_short without naming the missing leg")

    # The mechanical trigger: an L2 row measured on a plant trajectory has to be in the registry.
    for module, (level, evidence) in sorted(rows.items()):
        if level == "L2" and "test_sil_" in evidence and module not in registered:
            problems.append(
                f"{module}: L2 measured on a plant trajectory (its evidence names a SIL case) but not in the registry")
    return problems


def selftest(data: dict, rows: dict[str, tuple[str, str]]) -> bool:
    """Mutate a registry that passes and require each mutation to be caught."""
    ok = True

    def expect(label: str, mutated: dict, needle: str) -> None:
        nonlocal ok
        found = any(needle in problem for problem in check(mutated, rows))
        print(f"[observability-selftest] {label:52s} -> {'caught' if found else 'MISSED'}")
        ok = ok and found

    clean = json.loads(json.dumps(data))
    if check(clean, rows):
        print("[observability-selftest] the shipped registry does not pass, so the mutations below prove nothing")
        return False

    mutated = json.loads(json.dumps(data))
    mutated["upgrades"][0]["leg"] = "implementation"
    mutated["upgrades"][0]["reopened"][0]["verdict"] = "promoted"
    expect("implementation leg promoting a row", mutated, "cannot promote")

    mutated = json.loads(json.dumps(data))
    mutated["upgrades"][0]["reopened"][0]["missing"] = ""
    expect("still_short with no missing leg", mutated, "without naming")

    mutated = json.loads(json.dumps(data))
    mutated["upgrades"][0]["reopened"] = []
    mutated["upgrades"][0].pop("no_effect", None)
    expect("upgrade that re-opened nothing", mutated, "gave no reason")

    # Chosen from the data rather than written down: the first version named a row that was later promoted, so the
    # selftest went on passing a case that could no longer happen. A self-test pinned to a moving target stops testing
    # anything the day the target moves -- and it reports that as a failure, which is the only reason it was noticed.
    target = next((module for module, (level, evidence) in sorted(rows.items())
                   if level == "L2" and "test_sil_" in evidence), None)
    if target is None:
        print("[observability-selftest] no L2 SIL row to drop, so the fourth case proves nothing")
        return False
    mutated = json.loads(json.dumps(data))
    for upgrade in mutated["upgrades"]:
        upgrade["reopened"] = [item for item in upgrade.get("reopened") or [] if item["claim"] != target]
    expect(f"an L2 SIL row dropped from the registry ({target})", mutated, "not in the registry")
    return ok


def main(argv: list[str]) -> int:
    data = json.loads(REGISTRY.read_text(encoding="utf-8"))
    rows = claim_rows()
    if "--selftest" in argv:
        if not selftest(data, rows):
            print("[observability] FAIL (selftest)", file=sys.stderr)
            return 1
    problems = check(data, rows)
    for problem in problems:
        print(f"[observability] {problem}", file=sys.stderr)
    if problems:
        print(f"[observability] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        return 1
    count = sum(len(upgrade.get("reopened") or []) for upgrade in data["upgrades"])
    print(f"[observability] OK ({len(data['upgrades'])} upgrade(s), {count} claim row(s) re-read, "
          f"0 promoted on an implementation-level leg)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
