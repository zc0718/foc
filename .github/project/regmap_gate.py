#!/usr/bin/env python3
"""The register table against the C ABI it claims to expose.

F50's criterion, in one sentence: *every register must point at a C function that exists, and no readable C function may
exist that the table forgot.* Two directions, and they fail differently.

  * A register pointing at nothing is a protocol that answers with a number nobody computed. `rename a C function and
    forget the table` is the shape this catches -- the header is installed, the protocol still works, and the value is
    a stale copy of something else.
  * A readable C function missing from the table is a second vocabulary growing: the host reads what the table says,
    the firmware computes more, and the two drift until somebody measures one against the other.

The first direction is *also* a compile error, because `register_table()` stores the address of the function it names
(`read` is a function pointer, not a lookup key). This gate does not replace that; it makes the rename visible in the
diff review and in CI even when nobody builds, and it is the only half that can see the second direction.

What counts as "readable": a `foc_*` function declared in `foc_api.h` that returns a scalar (`uint32_t`, `int32_t`,
`float`) and takes a motor handle. Anything that returns a struct through an out-parameter is a *call*, not a register,
and is named by the registers that read fields out of it -- the table says which function each of those came from.

Usage: regmap_gate.py [--selftest]
"""
from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
API = ROOT / "include" / "foc" / "api" / "foc_api.h"
TABLE = ROOT / "include" / "foc" / "comm" / "register_map.hpp"
SCALARS = ("uint32_t", "int32_t", "float")

DECL = re.compile(r"^\s*(?!\*)([A-Za-z_][A-Za-z0-9_ \*]*?)\s+\**\b(foc_[a-z0-9_]+)\s*\(([^;]*)\)\s*;", re.MULTILINE)
ROW = re.compile(r"\{\s*RegisterId::(\w+)\s*,\s*RegisterKind::(\w+)\s*,\s*\"([^\"]+)\"\s*,")
ID = re.compile(r"^\s*(\w+)\s*=\s*(\d+)\s*,", re.MULTILINE)


def declarations() -> dict[str, tuple[str, str]]:
    """name -> (return type, parameter list) for every declaration in the installed C header."""
    out: dict[str, tuple[str, str]] = {}
    for match in DECL.finditer(API.read_text(encoding="utf-8", errors="replace")):
        ret, name, params = match.group(1).strip(), match.group(2), match.group(3)
        out[name] = (ret, params)
    return out


def table_rows() -> list[dict[str, str]]:
    text = TABLE.read_text(encoding="utf-8", errors="replace")
    return [{"id": m.group(1), "kind": m.group(2), "abi": m.group(3)} for m in ROW.finditer(text)]


def enum_ids() -> dict[str, int]:
    text = TABLE.read_text(encoding="utf-8", errors="replace")
    start = text.find("enum class RegisterId")
    if start < 0:
        return {}
    end = text.find("};", start)
    return {m.group(1): int(m.group(2)) for m in ID.finditer(text[start:end])}


def check(rows: list[dict[str, str]] | None = None, decls: dict | None = None,
          ids: dict[str, int] | None = None) -> list[str]:
    rows = table_rows() if rows is None else rows
    decls = declarations() if decls is None else decls
    ids = enum_ids() if ids is None else ids
    problems: list[str] = []

    if not rows:
        return ["the register table is empty -- a protocol with no registers is a protocol nobody wrote"]
    if not decls:
        return [f"no C declarations found in {API.relative_to(ROOT)} -- the parser or the header changed"]

    # 1. Every register points at something that exists.
    for row in rows:
        if row["abi"] not in decls:
            problems.append(f"register `{row['id']}` names `{row['abi']}`, which {API.relative_to(ROOT)} does not "
                            f"declare -- the protocol would answer with a number nobody computes")

    # 2. The address space is dense from 1 and the table is in that order.
    expected = 1
    for row in rows:
        value = ids.get(row["id"])
        if value is None:
            problems.append(f"register `{row['id']}` is in the table but not in `RegisterId`")
            continue
        if value != expected:
            problems.append(f"`RegisterId::{row['id']}` is {value}, expected {expected} -- the ids must be a dense "
                            f"range from 1 so a host can enumerate without a hole table")
        expected += 1
    for name, value in ids.items():
        if not any(row["id"] == name for row in rows):
            problems.append(f"`RegisterId::{name}` = {value} has no row in the table")

    # 3. No readable C function left out of the table.
    covered = {row["abi"] for row in rows}
    for name, (ret, params) in sorted(decls.items()):
        if ret not in SCALARS:
            continue
        if "foc_motor_t" not in params:
            continue
        if name not in covered:
            problems.append(f"`{name}` returns `{ret}` from a motor handle and no register names it -- the host would "
                            f"read what the table says while the firmware computes more")

    return problems


def selftest() -> int:
    """Cases where the answer must flip, so a gate that always says OK cannot be mistaken for a working one."""
    rows = table_rows()
    decls = declarations()
    ids = enum_ids()
    cases: list[tuple[str, list[str], dict, dict, bool]] = [
        ("a settled table", rows, decls, ids, True),
        ("a renamed C function", [{**rows[0], "abi": "foc_motor_get_statuss"}, *rows[1:]], decls, ids, False),
        ("a register deleted", rows[1:], decls, ids, False),
        ("the last register removed from table and enum", rows[:-1], decls,
         {k: v for k, v in ids.items() if k != "CurrentQ"}, True),
        ("a hole in the ids", [{**rows[0], "id": "State"}, *rows[1:]], decls, {**ids, "State": 2}, False),
        ("an empty table", [], decls, ids, False),
        ("a reader left out of the table",
         [r for r in rows if r["abi"] != "foc_motor_trace_dropped"], decls, ids, False),
    ]
    failures = 0
    for label, r, d, i, want_green in cases:
        got = check(r, d, i)
        ok = (not got) == want_green
        print(f"[regmap-selftest] {label:44s} -> {len(got)} problem(s), want "
              f"{'green' if want_green else 'red'} [{'ok' if ok else 'BROKEN'}]")
        failures += 0 if ok else 1
    return 1 if failures else 0


def main(argv: list[str]) -> int:
    if "--selftest" in argv:
        return selftest()
    problems = check()
    for problem in problems:
        print(f"[regmap] {problem}", file=sys.stderr, flush=True)
    if problems:
        print(f"[regmap] FAIL ({len(problems)} problem(s))")
        return 1
    rows = table_rows()
    print(f"[regmap] OK ({len(rows)} register(s), every one backed by a declaration in foc_api.h, and every readable "
          f"C function is in the table)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
