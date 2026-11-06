#!/usr/bin/env python3
"""`docshape` — docstrings use Doxygen's own vocabulary; they do not invent a section template.

Why this exists, and what it used to be
---------------------------------------
This gate started as a six-section skeleton (`**Model.**` `**Equation.**` `**Notation.**` `**Guarantees.**`
`**Refusals.**` `**Evidence.**`) with a ratchet over 195 blocks. That was a mistake, and the mistake is worth naming:
**the template already exists upstream**. `.github/skills/_shared/code-conventions.md` says the tag shape is "照
`fcpp/include/doctest.hpp`", and that header shows the real vocabulary -- a short `@brief` per language, untagged body
paragraphs underneath it, `\\f[...\\f]` for formulas, `@note`/`@warning` for caveats, `@see` for pointers. Doxygen
already has a field for everything those six sections were reaching for, so the skeleton was a private dialect layered
on a public one. Its cost appeared exactly where it was predicted: the `@brief` line stopped being brief, because a
section called "Model" has to carry a paragraph's worth of meaning.

What it checks now
------------------
1. **No invented section markers.** A body paragraph that opens with one of the six names the old template invented
   (`**Model.**` `**Equation.**` `**Notation.**` `**Guarantees.**` `**Refusals.**` `**Evidence.**`) is red -- otherwise
   the next person copies it. The rule names those six and nothing else: a bold lead-in such as `**The law.**` is
   pre-existing house prose in `min_pulse.hpp`, `mtpv.hpp` and `thermal_limit.hpp`, and banning every bold lead-in
   would flag good writing in order to prevent a mistake.
2. **`@brief` is a line, not a paragraph.** A brief that runs past `BRIEF_COLUMNS` display columns is red: the house
   example is `@brief vector add`, and a brief that needs a second line has already become the body that belongs
   underneath it. This leg has a ratchet, because the tree has long briefs today; the over-long count is frozen per
   file and may only fall.

What it deliberately does not check: which fields a block carries. A type with nothing to warn about should not hold an
empty `@warning`, so "which sections exist" is a judgement rather than a rule. `docfmt` already enforces the shape the
docs pipeline depends on (one tag per language group, untagged bodies, alignment, wrapping) and `docvoice` already
enforces the register. What is left here is the two things neither of those can see.
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASELINE = Path(__file__).resolve().parent / "docshape-baseline.json"
SCAN = ("include", "src", "api")
SUFFIXES = {".h", ".hpp", ".c", ".cpp"}
SKIP_PARTS = {"build", "CMakeFiles", ".git"}

# The six names of the private dialect. **Only these**: a bold lead-in such as `**The law.**` or `**Where it stops.**`
# is pre-existing house prose in `min_pulse.hpp`, `mtpv.hpp` and `thermal_limit.hpp`, and banning every bold lead-in
# would flag good writing to prevent a mistake. The rule is narrowed to the vocabulary that was invented here.
# 只禁这六个名字。`**The law.**` / `**Where it stops.**` 这类粗体引导句是 `min_pulse.hpp`、`mtpv.hpp`、
# `thermal_limit.hpp` 里既有的文风——把"所有粗体引导句"禁掉，是为了防一个错误而去误伤好文字。规则收窄到本处自创的那套词。
INVENTED = re.compile(r"^\s*\*\s+\*\*(Model|Equation|Notation|Guarantees|Refusals|Evidence)\.\*\*")
BRIEF = re.compile(r"^\s*\*\s*@brief (?:\[en\])?\s*(.*)$")
BRIEF_COLUMNS = 100


def blocks(text: str) -> list[tuple[int, str]]:
    out = []
    for match in re.finditer(r"/\*\*.*?\*/", text, re.DOTALL):
        out.append((text.count("\n", 0, match.start()) + 1, match.group(0)))
    return out


def files():
    for name in SCAN:
        root = ROOT / name
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*")):
            if path.suffix in SUFFIXES and path.is_file() and not (SKIP_PARTS & set(path.parts)):
                yield path


def scan_text(text: str) -> tuple[list[str], int]:
    """`(invented markers, over-long briefs)` for one file's worth of text."""
    invented: list[str] = []
    long_briefs = 0
    for start, block in blocks(text):
        for offset, line in enumerate(block.splitlines()):
            if INVENTED.match(line):
                invented.append(f"{start + offset}: invented section marker: {line.strip().lstrip('* ')}")
            brief = BRIEF.match(line)
            if brief and len(brief.group(1)) > BRIEF_COLUMNS:
                long_briefs += 1
    return invented, long_briefs


def measure() -> dict:
    invented: list[str] = []
    per_file: dict[str, int] = {}
    for path in files():
        rel = path.relative_to(ROOT).as_posix()
        found, long_briefs = scan_text(path.read_text(encoding="utf-8", errors="replace"))
        invented.extend(f"{rel}:{line}" for line in found)
        if long_briefs:
            per_file[rel] = long_briefs
    return {"invented": invented, "long_briefs": per_file}


def read_baseline() -> dict:
    if not BASELINE.is_file():
        return {}
    return json.loads(BASELINE.read_text(encoding="utf-8"))


def evaluate(measured: dict, baseline: dict) -> list[str]:
    problems = list(measured["invented"])
    frozen = baseline.get("long_briefs", {})
    for rel, count in measured["long_briefs"].items():
        ceiling = frozen.get(rel, 0)
        if count > ceiling:
            problems.append(f"{rel}: {count} brief(s) wider than {BRIEF_COLUMNS} columns, baseline {ceiling} — a "
                            f"brief that needs a second line is the body paragraph underneath it")
    return problems


def selftest() -> int:
    cases = [
        ("an invented section marker", "/**\n * @brief [en] x\n *\n *     **Equation.** e = mc^2\n */\nclass X {};\n",
         True),
        ("a brief that is a paragraph",
         "/**\n * @brief [en] " + ("word " * 30) + "\n */\nclass X {};\n", True),
        ("the house shape", "/**\n * @brief [en] vector add\n *\n *     Body paragraph, no tag.\n */\nclass X {};\n",
         False),
    ]
    caught = 0
    for label, text, expected in cases:
        invented, long_briefs = scan_text(text)
        got = bool(invented) or long_briefs > 0
        ok = got == expected
        print(f"[docshape-selftest] {'OK  ' if ok else 'MISS'} {label}: {'flagged' if got else 'clean'}")
        caught += 1 if ok else 0
    print(f"[docshape-selftest] {caught} / {len(cases)} judgement(s) as expected")
    return 0 if caught == len(cases) else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true", help="feed three blocks and require the verdicts")
    parser.add_argument("--list", action="store_true", help="print every problem")
    parser.add_argument("--baseline", action="store_true", help="freeze today's over-long brief count")
    args = parser.parse_args(argv)
    if args.selftest:
        return selftest()
    measured = measure()
    if args.baseline:
        BASELINE.write_text(json.dumps({
            "_comment": [
                "Over-long `@brief` lines, frozen per file so they may only fall. A `@brief` is a line, not a",
                "paragraph: the house example is `@brief vector add`.",
                "过长的 `@brief` 按文件冻结，只许下降。`@brief` 是一行，不是一段：本仓库的例子是 `@brief vector add`。",
            ],
            "long_briefs": measured["long_briefs"],
        }, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
        print(f"[docshape] froze {sum(measured['long_briefs'].values())} over-long brief(s) across "
              f"{len(measured['long_briefs'])} file(s)")
        return 0
    problems = evaluate(measured, read_baseline())
    if args.list or problems:
        for problem in problems:
            print(f"[docshape] {problem}")
    if problems:
        print(f"[docshape] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        return 1
    print(f"[docshape] OK (no invented section markers; "
          f"{sum(measured['long_briefs'].values())} over-long brief(s) at or below baseline)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
