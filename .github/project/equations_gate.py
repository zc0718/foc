#!/usr/bin/env python3
"""The equation ledger: every law the documentation states, tied to the code and recomputed.

Why this exists
---------------
A formula in a docstring is a claim about the code. Until something reads both, it is an opinion that happens to be
typeset. The plan's section 5.2(b) asks for one ledger per law with four things: the code that implements it, a test
that exercises that code, a witness (or the admission that there is none), and a worked evaluation. This gate is what
makes those four things load-bearing rather than decorative.

What it checks
--------------
1. `id` is unique and `law` is one this gate knows: an unknown law is red, never skipped.
2. `impl` is `{file, symbol}`; the file exists and contains the symbol. It is never a line number -- a docstring
   migration moves lines, and this registry has to survive one.
3. `case` is a real `TEST(Suite, Name)` in `test_package/test/unit/`, found by scanning for the macro rather than by
   trusting the string. `via`, when present, names the entry point the case actually calls, and that name must occur in
   the case's own file -- one hop of the chain from test to law, made explicit.
4. `witness` is a pointer (`case:...` or `claim:...`) or the literal `math`, and it carries **no evidence level**: the
   level has exactly one exit (`README.md` + `claim-basis.json`) and a copy here would be stale within one promotion.
5. Every symbol a row names is registered in `symbols.json`. Two registries that name the same quantity must not be
   able to disagree about what it is called.
6. `check.kind == "numeric"`: the law is recomputed here, from the recorded inputs, and compared with `expect`.

What the numeric check proves, and what it does not
---------------------------------------------------
It proves the row is internally consistent and that the number was not typed by hand. It does **not** prove the C++
agrees -- that is the `case`'s job, and this gate can only check that the case exists. Two links, each of which can go
red on its own. Neither one alone is a verification, and the ledger does not pretend otherwise.

`gaps` is for laws that are worth stating and are not yet in the ledger, each with a reason. It is a named debt, not an
omission: the count is printed on every run, so "the equation is not in the table" always has an answer.

Usage
-----
    python3 .github/project/equations_gate.py            # check
    python3 .github/project/equations_gate.py --list     # every problem, one per line
    python3 .github/project/equations_gate.py --selftest # mutate the ledger, require each mutation to be caught
"""

from __future__ import annotations

import argparse
import copy
import json
import math
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
LEDGER = Path(__file__).resolve().parent / "equations.json"
SYMBOLS = Path(__file__).resolve().parent / "symbols.json"
TEST_DIR = ROOT / "test_package" / "test" / "unit"
README = ROOT / "README.md"

WITNESS_POINTERS = ("case:", "claim:")
RELATIVE_TOLERANCE = 1.0e-9


def law_clarke(inputs: dict) -> dict:
    """`alpha = 2/3 (u - (v+w)/2)`, `beta = (v - w)/sqrt(3)` -- amplitude invariant, so a balanced set of amplitude A
    maps to a vector of amplitude A. 幅值不变式：A 的平衡组映射成 A 的矢量。"""
    u, v, w = inputs["u"], inputs["v"], inputs["w"]
    return {"alpha": (2.0 / 3.0) * (u - (v + w) / 2.0), "beta": (v - w) / math.sqrt(3.0)}


def law_inv_clarke(inputs: dict) -> dict:
    """`u = alpha`, `v = -alpha/2 + sqrt(3)/2 beta`, `w = -alpha/2 - sqrt(3)/2 beta`. The zero sequence is gone. 零序已丢。"""
    alpha, beta = inputs["alpha"], inputs["beta"]
    return {"u": alpha, "v": -alpha / 2.0 + math.sqrt(3.0) / 2.0 * beta,
            "w": -alpha / 2.0 - math.sqrt(3.0) / 2.0 * beta}


def law_park(inputs: dict) -> dict:
    """`d = alpha cos + beta sin`, `q = -alpha sin + beta cos` -- the q axis lags the d axis. q 轴滞后 d 轴。"""
    c, s = math.cos(inputs["theta_rad"]), math.sin(inputs["theta_rad"])
    return {"d": inputs["alpha"] * c + inputs["beta"] * s, "q": -inputs["alpha"] * s + inputs["beta"] * c}


def law_inv_park(inputs: dict) -> dict:
    """The transpose of the Park rotation, which is its inverse. Park 旋转的转置，也就是它的逆。"""
    c, s = math.cos(inputs["theta_rad"]), math.sin(inputs["theta_rad"])
    return {"alpha": inputs["d"] * c - inputs["q"] * s, "beta": inputs["d"] * s + inputs["q"] * c}


def law_current_pi(inputs: dict) -> dict:
    """`kp = max(2 wb L - R, 0)`, `ki = wb^2 L`, `v_lim = v_bus/2`.

    Substituting the plant pole back gives `s^2 + 2 wb s + wb^2`, i.e. critical damping and no overshoot, with every gain
    derived from the nameplate. The floor at zero is where the design rule would otherwise ask for a negative kp.
    代回被控对象极点得 `s² + 2·wb·s + wb²`，即临界阻尼、无超调，增益全由铭牌导出。零下限是设计规则本会要求负 kp 的地方。
    """
    wb, ld, rs = inputs["bandwidth_rad_s"], inputs["ld"], inputs["rs"]
    return {"kp": max((2.0 * wb * ld) - rs, 0.0), "ki": wb * wb * ld, "v_lim": inputs["v_bus"] / 2.0}


def law_speed_pi(inputs: dict) -> dict:
    """`kp = wb J / Kt`, `ki = wb B / Kt` -- cancels the mechanical pole `B/J`, so the loop collapses to `wb/s`.

    抵消机械极点 `B/J`，环路退化为 `wb/s`。
    """
    wb, kt = inputs["bandwidth_rad_s"], inputs["kt"]
    return {"kp": wb * inputs["j"] / kt, "ki": wb * inputs["b"] / kt}


def law_mtpa(inputs: dict) -> dict:
    """`i_d = (psi_f - sqrt(psi_f^2 + 8 (Lq-Ld)^2 iq^2)) / (4 (Lq-Ld))`, and the curve's magnitude.

    The maximum-torque-per-ampere condition is "the current vector is tangent to the torque contour"; solving it for a
    salient machine gives this root of a quadratic. 最大转矩每安培的条件是"电流矢量与转矩等值线相切"，对凸极机解这个二次式即得。
    """
    psi, ld, lq, iq = inputs["psi_f"], inputs["ld"], inputs["lq"], inputs["iq_abs"]
    saliency = lq - ld
    if abs(saliency) <= 1.0e-9:
        return {"i_d": 0.0, "magnitude": abs(iq)}
    root = math.sqrt((psi * psi) + (8.0 * saliency * saliency * iq * iq))
    d = (psi - root) / (4.0 * saliency)
    return {"i_d": d, "magnitude": math.hypot(d, iq)}


def law_mtpa_surface(inputs: dict) -> dict:
    """`Lq == Ld` implies `i_d = 0`: a surface machine has no reluctance term at all. 表贴机没有磁阻转矩项。"""
    if abs(inputs["lq"] - inputs["ld"]) > 1.0e-9:
        raise ValueError("this law only states the surface case; the pairs must be equal")
    return {"i_d": 0.0}


def law_pi_step(inputs: dict) -> dict:
    """One discrete PI step: integrate, re-clamp the integral against the current window, then clamp the output.

    The order matters and is the law: the integral is clamped **before** it is added to the proportional term, so a
    shrunken window releases stored output immediately rather than after the output clamp has already absorbed it.
    顺序就是定律：积分在**加到比例项之前**先被夹紧，于是窗口收窄时已存输出会立即释放，而不是等输出限幅先吸掉。
    """
    error = inputs["setpoint"] - inputs["feedback"]
    integral = inputs["integral"] + (inputs["ki"] * error * inputs["dt_s"])
    integral = min(max(integral, inputs["integral_min"]), inputs["integral_max"])
    raw = (inputs["kp"] * error) + integral
    return {"integral": integral, "output": min(max(raw, inputs["out_min"]), inputs["out_max"])}


def law_slew_step(inputs: dict) -> dict:
    """One slew step: `y <- clamp(target, y - rate*dt, y + rate*dt)`, and `y` holds when `dt <= 0`.

    The budget `rate * dt` is applied to the **state**, not to the error, which is what makes this a rate limit and not
    a first-order lag: the limiter keeps no memory of how far behind it was. 预算作用在**状态**上而非误差上，因此这是速率
    限制而不是一阶滞后：限制器不记得自己落后多少。
    """
    value, target, rate, dt = inputs["value"], inputs["target"], inputs["rate_per_s"], inputs["dt_s"]
    if dt <= 0.0:
        return {"value": value}
    delta = target - value
    budget = rate * dt
    return {"value": value + min(max(delta, -budget), budget)}


LAWS = {
    "clarke": (law_clarke, "alpha = 2/3 (u - (v+w)/2), beta = (v - w)/sqrt(3)"),
    "inv_clarke": (law_inv_clarke, "u = alpha, v = -alpha/2 + sqrt3/2 beta, w = -alpha/2 - sqrt3/2 beta"),
    "park": (law_park, "d = alpha cos + beta sin, q = -alpha sin + beta cos"),
    "inv_park": (law_inv_park, "alpha = d cos - q sin, beta = d sin + q cos"),
    "current_pi": (law_current_pi, "kp = max(2 wb L - R, 0), ki = wb^2 L"),
    "speed_pi": (law_speed_pi, "kp = wb J / Kt, ki = wb B / Kt"),
    "mtpa": (law_mtpa, "i_d = (psi_f - sqrt(psi_f^2 + 8 (Lq-Ld)^2 iq^2)) / (4 (Lq-Ld))"),
    "mtpa_surface": (law_mtpa_surface, "Lq == Ld -> i_d = 0"),
    "pi_step": (law_pi_step, "i <- clamp(i + ki e dt), u = clamp(kp e + i)"),
    "slew_step": (law_slew_step, "y <- clamp(target, y - rate dt, y + rate dt)"),
}


def test_cases() -> dict[str, Path]:
    """`{'Suite.Name': path}` for every `TEST(Suite, Name)` in the unit-test tree.

    Scanned rather than assumed: this gate's predecessor in another repo matched `"id"` against a file that declares
    `"name"`, found nothing, and reported a clean run over an empty set.
    扫出来而不是假定：本仓库另一个门禁曾用 `"id"` 去匹配一个声明 `"name"` 的文件，什么都没匹配到，却对空集报了个干净。
    """
    found: dict[str, Path] = {}
    if not TEST_DIR.exists():
        return found
    pattern = re.compile(r"^\s*TEST\s*\(\s*(\w+)\s*,\s*(\w+)\s*\)", re.MULTILINE)
    for path in sorted(TEST_DIR.glob("*.cpp")):
        for match in pattern.finditer(path.read_text(encoding="utf-8", errors="replace")):
            found[f"{match.group(1)}.{match.group(2)}"] = path
    return found


def registered_symbols() -> set[str]:
    if not SYMBOLS.exists():
        return set()
    table = json.loads(SYMBOLS.read_text(encoding="utf-8"))
    return {symbol.get("id") for symbol in table.get("symbols", [])}


def close(computed: float, recorded: float) -> bool:
    return abs(computed - recorded) <= (RELATIVE_TOLERANCE * max(abs(computed), abs(recorded), 1.0e-12))


def evaluate(ledger: dict, root: Path = ROOT) -> list[str]:
    problems: list[str] = []
    cases = test_cases()
    known_symbols = registered_symbols()
    seen: set[str] = set()
    for row in ledger.get("rows", []):
        row_id = row.get("id") or "<no id>"
        if row_id in seen:
            problems.append(f"{row_id}: declared twice")
        seen.add(row_id)
        if not row.get("latex"):
            problems.append(f"{row_id}: no latex, so there is nothing to typeset")
        law_name = row.get("law")
        if law_name not in LAWS:
            problems.append(f"{row_id}: unknown law '{law_name}' (known: {sorted(LAWS)})")
            continue
        impl = row.get("impl") or {}
        path = root / impl.get("file", "")
        if not impl.get("file") or not path.exists():
            problems.append(f"{row_id}: impl file '{impl.get('file')}' does not exist")
        elif not impl.get("symbol"):
            problems.append(f"{row_id}: impl names no symbol, so it points at a file rather than at the law")
        elif impl["symbol"] not in path.read_text(encoding="utf-8", errors="replace"):
            problems.append(f"{row_id}: impl symbol '{impl['symbol']}' is not in '{impl['file']}'")
        elif not any(line.lstrip().startswith("*") and row_id in line
                     for line in path.read_text(encoding="utf-8", errors="replace").splitlines()):
            # The registry exists so that a formula in the docs is a claim about the code rather than an
            # opinion; that only holds if the file the reader is in points at the row. The id has to be in a
            # doc-comment line, not merely somewhere in the file.
            problems.append(f"{row_id}: '{impl['file']}' never cites the row in a doc comment, so the reader of "
                            f"that code has no pointer to the law, its case or its worked evaluation")
        case = row.get("case")
        if not case:
            problems.append(f"{row_id}: names no case")
        elif case not in cases:
            problems.append(f"{row_id}: case '{case}' is not a TEST() in test_package/test/unit/")
        else:
            via = row.get("via")
            if via and via not in cases[case].read_text(encoding="utf-8", errors="replace"):
                problems.append(f"{row_id}: via '{via}' does not occur in {cases[case].name}, so the case does not "
                                f"reach the law that way")
        witness = row.get("witness") or ""
        if not witness:
            problems.append(f"{row_id}: no witness (write 'math' if there is no independent one)")
        elif witness != "math" and not witness.startswith(WITNESS_POINTERS):
            problems.append(f"{row_id}: witness '{witness}' is neither 'math' nor a case:/claim: pointer")
        elif witness.startswith("claim:"):
            target = root / witness[len("claim:"):]
            if not target.exists():
                problems.append(f"{row_id}: witness claims '{target}' which does not exist")
            elif target.name not in README.read_text(encoding="utf-8", errors="replace"):
                problems.append(f"{row_id}: '{target.name}' has no row in README.md, so the claim is not made there")
        for symbol_id in row.get("symbols", []):
            if symbol_id not in known_symbols:
                problems.append(f"{row_id}: symbol '{symbol_id}' is not in symbols.json")
        check = row.get("check") or {}
        if check.get("kind") != "numeric":
            problems.append(f"{row_id}: check.kind '{check.get('kind')}' is not 'numeric'")
            continue
        law, _ = LAWS[law_name]
        try:
            computed = law(check.get("inputs") or {})
        except (KeyError, ValueError, ZeroDivisionError) as error:
            problems.append(f"{row_id}: the law could not be evaluated from its inputs ({error})")
            continue
        expect = check.get("expect") or {}
        if not expect:
            problems.append(f"{row_id}: check records no expected value, so nothing is compared")
        for key, recorded in expect.items():
            if key not in computed:
                problems.append(f"{row_id}: expect names '{key}' which the law does not produce")
            elif not close(computed[key], recorded):
                problems.append(f"{row_id}: expect['{key}'] = {recorded!r} but the law gives {computed[key]!r}")
    for gap in ledger.get("gaps", []):
        if not gap.get("why", {}).get("en"):
            problems.append(f"gap '{gap.get('law')}': no reason, which is indistinguishable from an omission")
        impl = gap.get("impl") or {}
        if impl and not (root / impl.get("file", "")).exists():
            problems.append(f"gap '{gap.get('law')}': impl file '{impl.get('file')}' does not exist")
    return problems


def selftest(ledger: dict) -> int:
    """Seven mutations of the real ledger, each of which must be caught."""
    caught = 0
    trials: list[tuple[str, dict]] = []

    wrong = copy.deepcopy(ledger)
    for row in wrong["rows"]:
        for key in row["check"]["expect"]:
            row["check"]["expect"][key] = row["check"]["expect"][key] * 1.5 + 1.0
            break
        break
    trials.append(("an expected value the law does not give", wrong))

    ghost_case = copy.deepcopy(ledger)
    if ghost_case["rows"]:
        ghost_case["rows"][0]["case"] = "Transforms.ACaseNobodyWrote"
    trials.append(("a case that does not exist", ghost_case))

    uncited = copy.deepcopy(ledger)
    if uncited["rows"]:
        uncited["rows"][0]["id"] = "E-NOBODY-CITES-THIS"
        uncited["rows"][0]["check"]["expect"] = ledger["rows"][0]["check"]["expect"]
    trials.append(("a row whose implementing file never cites it", uncited))

    ghost_symbol = copy.deepcopy(ledger)
    if ghost_symbol["rows"]:
        ghost_symbol["rows"][0]["impl"]["symbol"] = "__not_in_this_file__"
    trials.append(("an impl symbol the file does not contain", ghost_symbol))

    level_in_witness = copy.deepcopy(ledger)
    if level_in_witness["rows"]:
        level_in_witness["rows"][0]["witness"] = "L3"
    trials.append(("a witness that is an evidence level instead of a pointer", level_in_witness))

    twice = copy.deepcopy(ledger)
    if twice["rows"]:
        twice["rows"].append(copy.deepcopy(twice["rows"][0]))
    trials.append(("the same equation declared twice", twice))

    foreign_symbol = copy.deepcopy(ledger)
    if foreign_symbol["rows"]:
        foreign_symbol["rows"][0]["symbols"] = ["__no_such_symbol__"]
    trials.append(("a symbol that is not in symbols.json", foreign_symbol))

    ghost_law = copy.deepcopy(ledger)
    if ghost_law["rows"]:
        ghost_law["rows"][0]["law"] = "__no_such_law__"
    trials.append(("a law this gate does not know", ghost_law))

    for label, mutated in trials:
        if evaluate(mutated):
            print(f"[equations-selftest] {label} -> caught")
            caught += 1
        else:
            print(f"[equations-selftest] {label} -> NOT CAUGHT", file=sys.stderr)
    return 0 if caught == len(trials) else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true", help="mutate the ledger and require each mutation to be caught")
    parser.add_argument("--list", action="store_true", help="print every problem")
    args = parser.parse_args(argv)
    ledger = json.loads(LEDGER.read_text(encoding="utf-8"))
    if args.selftest:
        return selftest(ledger)
    problems = evaluate(ledger)
    if args.list or problems:
        for problem in problems:
            print(f"[equations] {problem}")
    if problems:
        print(f"[equations] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        return 1
    rows = ledger["rows"]
    pointers = sum(1 for row in rows if (row.get("witness") or "").startswith(WITNESS_POINTERS))
    print(f"[equations] OK ({len(rows)} law(s), {pointers} with an independent witness, "
          f"{len(rows) - pointers} stating 'math', {len(ledger.get('gaps', []))} named gap(s))")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
