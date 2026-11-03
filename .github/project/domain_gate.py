#!/usr/bin/env python3
"""Applicability domain of the two capabilities the plan called "narrow": C7 (field weakening) and C8 (MTPA).

Why this exists
---------------
The plan's section 0 splits "high" into three things -- the capability exists, the number is credible, and the
**domain** in which it holds is known. The first two have gates; the third was prose. Both capabilities are L3
headers with measured numbers and both are still marked "low" in the strategic map, for exactly one reason: the
region in which they are worth running is narrow, and that region was written down in five different places
instead of one table.

This gate makes the table mechanical. Every row is a **law on machine parameters** plus the machines it was
evaluated on, and the gate recomputes the law and compares it with the number recorded in the table. So the
table cannot drift from the arithmetic, and the arithmetic is tied to the implementation by the C++ case named in
`evidence` -- two links, each of which can go red.

What it checks
--------------
1. Every capability in `required` has at least one row: "we forgot" is red.
2. Every row's `law` is known to this gate (an unknown law is red, not skipped).
3. Each declared machine's recorded value matches the law recomputed from that machine's own parameters.
4. Each declared machine's verdict matches the row's threshold applied to the recomputed value.
5. A row whose verdict is `fails` carries a `note`: a capability that does not hold anywhere is a conclusion, and a
   conclusion has to be written down.
6. Every file named in `evidence` exists.

`--selftest` mutates a load of the real table four ways and requires each to be caught, because a gate that has
never been red is a gate nobody has any reason to trust.
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TABLE = Path(__file__).resolve().parent / "domain.json"


def law_lambda(machine: dict) -> float:
    """`lambda = Ld * i_max / psi_f` -- the field-weakening closure ratio.

    From `core/field_weakening.hpp`: field weakening can only hold the voltage envelope if the characteristic current
    `psi_f / Ld` is inside the current limit, which is the same statement as `Ld * i_max / psi_f >= 1`. The header's
    own rule is the single comparison `i_char <= i_max` (F-107 removed the two thresholds that could never fire).
    从 `core/field_weakening.hpp`：只有当特征电流 `psi_f / Ld` 落在电流限幅之内时，弱磁才可能守住电压包络，这与
    `Ld·i_max/psi_f >= 1` 是同一句话。头文件里的规则就是这个单一比较 `i_char <= i_max`（F-107 删掉了两个永不可能触发的阈值）。
    """
    return machine["ld"] * machine["i_max"] / machine["psi_f"]


def law_i_char(machine: dict) -> float:
    """`i_char = psi_f / (Lq - Ld)` -- the current at which the MTPA d-axis term stops paying.

    The MTPA d-axis current grows with the requested torque and is capped by the saliency's own scale; past
    `psi_f / (Lq - Ld)` the reference is asking for more d-axis current than the saliency can pay for, so the
    capability's domain is "`i_char <= i_max`". A surface machine has no d-axis term at all (`Lq == Ld`), which this
    reports as an infinite characteristic current rather than as a division by zero.
    MTPA 的 d 轴电流随请求转矩增长，却被凸极自身的尺度封顶；越过 `psi_f / (Lq - Ld)` 之后，参考要的 d 轴电流已经超过凸极
    能偿付的量，所以该能力的适用域是"`i_char <= i_max`"。表贴机没有 d 轴项（`Lq == Ld`），这里报成无穷大而不是除零。
    """
    spread = machine["lq"] - machine["ld"]
    if spread <= 0.0:
        return float("inf")
    return machine["psi_f"] / spread


LAWS = {"lambda": (law_lambda, "lambda = Ld*i_max/psi_f"), "i_char": (law_i_char, "i_char = psi_f/(Lq-Ld)")}


def problems_for(table: dict, root: Path) -> list[str]:
    problems: list[str] = []
    rows = table.get("rows", [])
    capabilities = {row.get("capability") for row in rows}
    for required in table.get("required", []):
        if required not in capabilities:
            problems.append(f"capability '{required}' has no domain row: the table is silent about it")
    for row in rows:
        name = f"{row.get('capability')}/{row.get('law')}"
        law_name = row.get("law")
        if law_name not in LAWS:
            problems.append(f"{name}: unknown law '{law_name}' (known: {sorted(LAWS)})")
            continue
        law, _ = LAWS[law_name]
        machines = row.get("machines", [])
        if not machines:
            problems.append(f"{name}: no machine was evaluated, so the row asserts nothing")
        for machine in machines:
            missing = [key for key in ("name", "ld", "lq", "psi_f", "i_max", "recorded") if key not in machine]
            if missing:
                problems.append(f"{name}: machine '{machine.get('name', '?')}' is missing {missing}")
                continue
            computed = law(machine)
            record = machine["recorded"]
            if record is not None and abs(computed - record) > (1.0e-3 * max(abs(computed), 1.0e-12)):
                problems.append(f"{name}/{machine['name']}: recorded {record!r} but the law gives {computed!r}")
            holds = computed >= row["threshold"] if row["comparison"] == ">=" else computed <= row["threshold"]
            expected = row["holds_when"] if holds else row["fails_when"]
            if machine.get("verdict") != expected:
                problems.append(
                    f"{name}/{machine['name']}: verdict '{machine.get('verdict')}' but the law puts it in "
                    f"'{expected}'")
        if "fails" in {machine.get("verdict") for machine in machines} and not row.get("note"):
            problems.append(f"{name}: some machines fail the domain and no note says what that means")
        for evidence in row.get("evidence", []):
            if not (root / evidence).exists():
                problems.append(f"{name}: evidence '{evidence}' does not exist")
        if not row.get("evidence"):
            problems.append(f"{name}: names no evidence, so the law rests on nothing")
    return problems


def selftest(table: dict, root: Path) -> int:
    """Four mutations of the real table, each of which must be caught."""
    caught = 0
    trials: list[tuple[str, dict]] = []

    stale = copy.deepcopy(table)
    for row in stale["rows"]:
        for machine in row.get("machines", []):
            if machine.get("recorded") is not None:
                machine["recorded"] = machine["recorded"] * 1.5
                break
        break
    trials.append(("a recorded value that the law does not give", stale))

    verdict = copy.deepcopy(table)
    for row in verdict["rows"]:
        for machine in row.get("machines", []):
            machine["verdict"] = "fails" if machine.get("verdict") != "fails" else "holds"
            break
        break
    trials.append(("a verdict that disagrees with the law", verdict))

    missing = copy.deepcopy(table)
    if missing["rows"]:
        missing["rows"][0].pop("evidence", None)
    trials.append(("a row with no evidence", missing))

    silent = copy.deepcopy(table)
    if silent.get("required"):
        silent["required"] = list(silent["required"]) + ["__selftest_capability__"]
    trials.append(("a capability with no row", silent))

    for label, mutated in trials:
        if problems_for(mutated, root):
            print(f"[domain-selftest] {label} -> caught")
            caught += 1
        else:
            print(f"[domain-selftest] {label} -> NOT CAUGHT", file=sys.stderr)
    return 0 if caught == len(trials) else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true", help="mutate the table and require each mutation to be caught")
    args = parser.parse_args(argv)
    table = json.loads(TABLE.read_text(encoding="utf-8"))
    if args.selftest:
        return selftest(table, ROOT)
    problems = problems_for(table, ROOT)
    if problems:
        print(f"[domain] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        for problem in problems:
            print(f"  - {problem}", file=sys.stderr)
        return 1
    rows = len(table["rows"])
    machines = sum(len(row.get("machines", [])) for row in table["rows"])
    failing = sum(1 for row in table["rows"] for machine in row.get("machines", [])
                  if machine.get("verdict") == row.get("fails_when"))
    print(f"[domain] OK ({rows} row(s), {machines} machine evaluation(s), {failing} outside the domain)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
