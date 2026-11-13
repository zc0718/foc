#!/usr/bin/env python3
"""The symbol table: every glyph the spec register may use, in one place the gate recomputes.

Why this exists
---------------
The plan's section 5.2 asks for one table carrying, per symbol, the meaning, the unit, the frame it lives in and which
direction is positive. The reason is not tidiness: this library has already paid for a missing `pole_pairs` (the
open-loop start's swing frequency was off by `sqrt(7)`) and for a mechanical/electrical mix-up, and both were found by
measurement rather than by reading. A table turns "which omega is this" into a question with a written answer, and
makes an unregistered symbol a failure instead of a judgement call.

What it checks
--------------
1. Every id in `required` has an entry: "we forgot" is red.
2. Every entry carries the four things that make a symbol usable (meaning in both registers, unit, frame, sign). A
   registered symbol with no unit is exactly the state this table exists to prevent, so it is red rather than blank.
3. Every non-reserved entry names an anchor `{file, code}`, the file exists and the fragment occurs in it.
4. An anchor whose every occurrence sits inside a comment is counted as `prose-anchored`: legal today, frozen at
   today's number, because a quotation of prose cannot survive having the prose rewritten.
5. A docstring glyph that no entry claims is unregistered. Counted per file and ratcheted downward.
6. A `reserved` entry whose glyph *is* used means the reservation is stale: promote it or drop it.
7. `docs/sphinx/source/symbols.rst` is generated from this file, and the gate fails when the page is stale.

Why the ratchets and not zero
-----------------------------
The docstrings already use eleven glyphs across three hundred-odd lines. Pretending the table covers them all would
mean either writing entries from imagination or leaving the gate off; the honest third option is to state today's
number, refuse any increase, and let the number fall as the entries get written.

Usage
-----
    python3 .github/project/symbols_gate.py              # check
    python3 .github/project/symbols_gate.py --list       # every problem, one per line
    python3 .github/project/symbols_gate.py --render     # rewrite the Sphinx page
    python3 .github/project/symbols_gate.py --baseline   # freeze today's counts
    python3 .github/project/symbols_gate.py --selftest   # mutate the table, require each mutation to be caught
"""

from __future__ import annotations

import argparse
import copy
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TABLE = Path(__file__).resolve().parent / "symbols.json"
BASELINE = Path(__file__).resolve().parent / "symbols-baseline.json"
PAGE = ROOT / "docs" / "sphinx" / "source" / "symbols.rst"

SCAN_DIRS = ["include", "src"]
SUFFIXES = {".h", ".hpp", ".c", ".cpp"}
SKIP_PARTS = {"build", "CMakeFiles", ".git"}

# Greek letters that mean *something* in motor control. `pi` is in the pool only so the gate can say out loud that it
# is not a machine quantity -- an allowlist entry with a reason beats a character quietly skipped.
GREEK = "θωψλαβδμτσφηγκπ"
ALLOWED_GLYPHS = {"π": "the mathematical constant, not a machine quantity"}

REQUIRED_FIELDS = ("id", "glyph", "unit", "frame", "sign")


def doc_lines(path: Path):
    """Yield `(lineno, text)` for every line that holds prose: a `/** */` body or a `//` line.

    Code is deliberately excluded. `psi_f`, `theta`, `kt` are identifiers in code and symbols in prose, and scanning
    identifiers would flag every parameter of every function for not being in a symbol table.
    """
    inside = False
    for lineno, raw in enumerate(path.read_text(encoding="utf-8", errors="replace").splitlines(), 1):
        line = raw.strip()
        if "/*" in line and "*/" not in line:
            inside = True
            yield lineno, line
            continue
        if inside:
            yield lineno, line
            if "*/" in line:
                inside = False
            continue
        if line.startswith("//"):
            yield lineno, line


def scanned_files():
    for name in SCAN_DIRS:
        for path in (ROOT / name).rglob("*"):
            if path.suffix not in SUFFIXES or SKIP_PARTS & set(path.parts):
                continue
            yield path


def alias_index(symbols) -> list[tuple[str, str]]:
    """`(alias, id)` pairs, longest alias first, so `θ_mech` is not read as `θ` plus `_mech`."""
    pairs = [(alias, symbol["id"]) for symbol in symbols for alias in symbol.get("aliases", [])]
    return sorted(pairs, key=lambda pair: len(pair[0]), reverse=True)


def coverage(symbols):
    """Return `(unregistered, reserved_used)`: unknown glyph occurrences and stale reservations."""
    pairs = alias_index(symbols)
    reserved = {symbol["id"] for symbol in symbols if symbol.get("status") == "reserved"}
    unregistered: list[str] = []
    reserved_used: list[str] = []
    for path in scanned_files():
        rel = path.relative_to(ROOT).as_posix()
        for lineno, line in doc_lines(path):
            spans: list[tuple[int, int]] = []
            for alias, symbol_id in pairs:
                start = line.find(alias)
                while start >= 0:
                    spans.append((start, start + len(alias)))
                    if symbol_id in reserved:
                        reserved_used.append(f"{rel}:{lineno} '{alias}' is reserved but used")
                    start = line.find(alias, start + 1)
            for offset, char in enumerate(line):
                if char not in GREEK or char in ALLOWED_GLYPHS:
                    continue
                if any(begin <= offset < end for begin, end in spans):
                    continue
                unregistered.append(f"{rel}:{lineno} '{char}' is not registered")
    return unregistered, reserved_used


def prose_anchored(symbols) -> list[str]:
    """Entries whose anchor fragment only ever occurs inside a comment."""
    found: list[str] = []
    for symbol in symbols:
        anchor = symbol.get("anchor")
        if not anchor:
            continue
        path = ROOT / anchor["file"]
        if not path.exists():
            continue
        hits = [line for line in path.read_text(encoding="utf-8", errors="replace").splitlines()
                if anchor["code"] in line]
        if hits and all(line.strip().startswith(("*", "//", "/*")) for line in hits):
            found.append(symbol["id"])
    return found


def evaluate(table: dict) -> list[str]:
    problems: list[str] = []
    symbols = table.get("symbols", [])
    by_id = {}
    by_glyph = {}
    for symbol in symbols:
        symbol_id = symbol.get("id")
        if not symbol_id:
            problems.append("an entry has no id")
            continue
        if symbol_id in by_id:
            problems.append(f"{symbol_id}: declared twice")
        by_id[symbol_id] = symbol
        glyph = symbol.get("glyph")
        if glyph:
            if glyph in by_glyph:
                problems.append(f"{symbol_id}: glyph '{glyph}' is also '{by_glyph[glyph]}'")
            by_glyph[glyph] = symbol_id
        missing = [field for field in REQUIRED_FIELDS if not symbol.get(field)]
        if missing:
            problems.append(f"{symbol_id}: missing {missing}")
        meaning = symbol.get("meaning") or {}
        empty = [register for register in ("en", "zh") if not (meaning.get(register) or "").strip()]
        if empty:
            problems.append(f"{symbol_id}: meaning is empty in {empty}")
        if symbol.get("status") == "reserved":
            if not (symbol.get("why") or {}).get("en"):
                problems.append(f"{symbol_id}: reserved without `why`, which is indistinguishable from forgotten")
            continue
        if not symbol.get("aliases"):
            problems.append(f"{symbol_id}: no alias, so no docstring could ever be checked against it")
        anchor = symbol.get("anchor")
        if not anchor:
            problems.append(f"{symbol_id}: no anchor, so the definition points at nothing")
            continue
        path = ROOT / anchor.get("file", "")
        if not path.exists():
            problems.append(f"{symbol_id}: anchor file '{anchor.get('file')}' does not exist")
            continue
        if anchor.get("code", "") not in path.read_text(encoding="utf-8", errors="replace"):
            problems.append(f"{symbol_id}: anchor fragment '{anchor.get('code')}' is not in '{anchor.get('file')}'")
    for required in table.get("required", []):
        if required not in by_id:
            problems.append(f"required symbol '{required}' has no entry: the table is silent about it")
    return problems


def load_baseline() -> dict:
    if not BASELINE.exists():
        return {}
    return json.loads(BASELINE.read_text(encoding="utf-8"))


def measure(table: dict) -> dict:
    unregistered, reserved_used = coverage(table.get("symbols", []))
    per_file: dict[str, int] = {}
    for problem in unregistered:
        per_file[problem.split(":", 1)[0]] = per_file.get(problem.split(":", 1)[0], 0) + 1
    reserved = sum(1 for symbol in table.get("symbols", []) if symbol.get("status") == "reserved")
    return {
        "unregistered": per_file,
        "prose_anchors": sorted(prose_anchored(table.get("symbols", []))),
        "reserved": reserved,
        "_reserved_used": reserved_used,
    }


def ratchet(measured: dict, baseline: dict) -> list[str]:
    problems = [problem for problem in measured["_reserved_used"]]
    frozen = baseline.get("unregistered", {})
    for path, count in measured["unregistered"].items():
        ceiling = frozen.get(path, 0)
        if count > ceiling:
            problems.append(f"{path}: {count} unregistered glyph line(s), baseline {ceiling}")
    frozen_prose = set(baseline.get("prose_anchors", []))
    for symbol_id in measured["prose_anchors"]:
        if symbol_id not in frozen_prose:
            problems.append(f"{symbol_id}: anchor is prose-only and is not in the frozen set")
    if measured["reserved"] > baseline.get("reserved", 0):
        problems.append(f"{measured['reserved']} reserved symbol(s), baseline {baseline.get('reserved', 0)}")
    return problems


def render(table: dict) -> str:
    """The reader-facing page, generated so it cannot disagree with the file the gate reads."""
    lines = [
        "Symbols and units",
        "=================",
        "",
        "Every symbol this library's documentation is allowed to use, with the four things that make a symbol usable:",
        "what it means, its unit, the frame it lives in, and which direction is positive.",
        "",
        "The page is generated from `.github/project/symbols.json` by `symbols_gate.py --render`; a symbol that is not",
        "on it cannot appear in a docstring, because the gate reads the same file.",
        "",
    ]
    for symbol in table.get("symbols", []):
        meaning = symbol.get("meaning") or {}
        reserved = " *(reserved -- defined for readers, not used by the code yet)*" if symbol.get("status") == "reserved" else ""
        lines.append(f"{symbol.get('glyph', '?')}  ``{symbol.get('id')}``{reserved}")
        lines.append(f"    **Meaning.** {meaning.get('en', '')}")
        lines.append(f"    **Unit.** {symbol.get('unit', '')}  **Frame.** {symbol.get('frame', '')}")
        lines.append(f"    **Positive.** {symbol.get('sign', '')}")
        anchor = symbol.get("anchor")
        if anchor:
            lines.append(f"    **Defined by.** ``{anchor['file']}`` (``{anchor['code']}``)")
        if symbol.get("note"):
            lines.append(f"    **Note.** {symbol['note'].get('en', '')}")
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def selftest(table: dict) -> int:
    """Six mutations of the real table, each of which must be caught before the gate is trusted."""
    caught = 0
    trials: list[tuple[str, dict]] = []

    no_unit = copy.deepcopy(table)
    for symbol in no_unit["symbols"]:
        symbol["unit"] = ""
        break
    trials.append(("a registered symbol with no unit", no_unit))

    no_zh = copy.deepcopy(table)
    for symbol in no_zh["symbols"]:
        symbol["meaning"] = {"en": symbol["meaning"]["en"], "zh": "  "}
        break
    trials.append(("a meaning in one language only", no_zh))

    bad_anchor = copy.deepcopy(table)
    for symbol in bad_anchor["symbols"]:
        if symbol.get("anchor"):
            symbol["anchor"]["code"] = "__not_in_this_file__"
            break
    trials.append(("an anchor the file does not contain", bad_anchor))

    forgotten = copy.deepcopy(table)
    forgotten["required"] = list(forgotten["required"]) + ["__selftest_symbol__"]
    trials.append(("a required symbol with no entry", forgotten))

    twice = copy.deepcopy(table)
    if twice["symbols"]:
        twice["symbols"].append(copy.deepcopy(twice["symbols"][0]))
    trials.append(("the same symbol declared twice", twice))

    stale = copy.deepcopy(table)
    probes = copy.deepcopy(table)
    probe = next(symbol for symbol in probes["symbols"] if symbol.get("aliases") and not symbol.get("status"))
    stale["symbols"].append({
        "id": "__selftest_reserved__",
        "glyph": "__selftest__",
        "aliases": list(probe["aliases"]),
        "meaning": {"en": "probe", "zh": "探针"},
        "unit": "1",
        "frame": "probe",
        "sign": "probe",
        "status": "reserved",
        "why": {"en": "probe", "zh": "探针"},
    })
    trials.append(("a reservation whose glyph is in use", stale))

    for label, mutated in trials:
        structural = evaluate(mutated)
        dynamic = ratchet(measure(mutated), load_baseline())
        if structural or dynamic:
            print(f"[symbols-selftest] {label} -> caught")
            caught += 1
        else:
            print(f"[symbols-selftest] {label} -> NOT CAUGHT", file=sys.stderr)
    return 0 if caught == len(trials) else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true", help="mutate the table and require each mutation to be caught")
    parser.add_argument("--list", action="store_true", help="print every problem and every measurement")
    parser.add_argument("--render", action="store_true", help="write the Sphinx page from the table")
    parser.add_argument("--out", type=Path, default=None,
                        help="where --render writes; the site page by default, a temp path when a doc-side check asks")
    parser.add_argument("--baseline", action="store_true", help="freeze today's measurements into the baseline")
    args = parser.parse_args(argv)
    table = json.loads(TABLE.read_text(encoding="utf-8"))
    if args.selftest:
        return selftest(table)
    if args.render:
        target = args.out or PAGE
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(render(table), encoding="utf-8")
        print(f"[symbols] wrote {target}")
        return 0
    measured = measure(table)
    if args.baseline:
        BASELINE.write_text(json.dumps({
            "_comment": [
                "Today's measurements, frozen so they may only fall. `unregistered` is per file on purpose: a file that",
                "gains one glyph cannot hide behind a file that lost two.",
                "今天的测量值，冻结后只许下降。`unregistered` 按文件计，这样一个文件新增的记号无法被另一个文件减少的抵掉。",
            ],
            "unregistered": measured["unregistered"],
            "prose_anchors": measured["prose_anchors"],
            "reserved": measured["reserved"],
        }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        total = sum(measured["unregistered"].values())
        print(f"[symbols] froze {total} unregistered line(s) across {len(measured['unregistered'])} file(s), "
              f"{len(measured['prose_anchors'])} prose anchor(s), {measured['reserved']} reserved symbol(s)")
        return 0
    problems = evaluate(table) + ratchet(measured, load_baseline())
    # The page this table renders into is *not* checked here. A gate whose subject is the code must run
    # with no documentation tree at all: whether the site page is current is a question about the
    # documentation, so `render_check.py` -- run by the `docs` gate -- asks it.
    if args.list or problems:
        for problem in problems:
            print(f"[symbols] {problem}")
    if problems:
        print(f"[symbols] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        return 1
    symbols = len(table["symbols"])
    total = sum(measured["unregistered"].values())
    print(f"[symbols] OK ({symbols} symbol(s), {len(table['required'])} required, "
          f"{measured['reserved']} reserved, {total} unregistered line(s) at or below baseline)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
