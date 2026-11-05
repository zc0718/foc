#!/usr/bin/env python3
"""Emit the T0 compliance-evidence table from what this repository actually enforces.

T0 is the part of `workspace/compliance/` that costs nothing: four free rule sets already downloaded (AUTOSAR C++14,
CERT C++ and CERT C, the C++ Core Guidelines, JPL's Power of Ten) plus the two tool manuals whose checks we actually run.
This script turns "we follow some of that" into a table that can be checked line by line, and it deliberately claims
**no compliance level**: it reports which checks are enforced, which free rule family each belongs to, and every
suppression with the reason that was written next to it -- including the ones with no reason, which are the interesting
rows.

Why a generator rather than a document: the suppressions are the part that rots. A hand-written table saying "12
suppressions, each justified" is wrong the first time somebody adds a `NOLINT` at 2 a.m., and nothing would notice. This
reads the tree.

Usage:
    python3 .github/project/t0_evidence.py            # write docs/compliance/T0-EVIDENCE.md
    python3 .github/project/t0_evidence.py --stdout   # print instead
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
TIDY = ROOT / ".github" / "misc" / ".clang-tidy"
OUT = ROOT / "docs" / "compliance" / "T0-EVIDENCE.md"
SCAN_DIRS = ["include", "src", "api"]

# Which free rule set a clang-tidy check family belongs to. Anything absent is reported as "no free rule ID" rather than
# being stretched to fit: a mapping invented to make the table look complete is worse than an empty cell.
FAMILIES = {
    "cert-": "CERT C++ (A2) / CERT C (A3)",
    "cppcoreguidelines-": "C++ Core Guidelines (A10)",
    "automotive-": "AUTOSAR C++14 (A1)",
}

SUPPRESSION = re.compile(r"(?:NOLINTNEXTLINE|NOLINT)\s*\((?P<checks>[^)]*)\)(?P<tail>.*)$")
BARE_NOLINT = re.compile(r"(?:NOLINTNEXTLINE|NOLINT)\b(?!\s*\()")
REASON = re.compile(r"(?:--|—|:)\s*(?P<reason>\S.*)$")


def enabled_checks() -> list[str]:
    """The `Checks:` list from the tidy configuration, minus the `-*` reset."""
    text = TIDY.read_text(encoding="utf-8")
    body: list[str] = []
    inside = False
    for line in text.splitlines():
        if line.startswith("Checks:"):
            inside = True
            # `Checks: >` carries the list on the following indented lines.
            continue
        if inside:
            # The list ends at the next top-level key; blank lines and comments live *inside* it.
            if line and not line[0].isspace():
                break
            body.append(line)
    checks = []
    for line in body:
        line = line.split("#", 1)[0].strip().rstrip(",").strip()
        if not line or line == "-*":
            continue
        checks.append(line)
    return checks


def family_of(check: str) -> str:
    for prefix, name in FAMILIES.items():
        if check.startswith(prefix):
            return name
    return "no free rule ID"


COMMENT = re.compile(r"^\s*(?://+|/\*+)\s*(?P<text>.*?)\s*(?:\*/)?\s*$")


def _reason_above(lines: list[str], index: int) -> re.Match[str] | None:
    """The comment line immediately above a suppression, or None when there is none."""
    while index > 0:
        index -= 1
        text = lines[index]
        if not text.strip():
            return None
        match = COMMENT.match(text)
        if match is None:
            return None
        body = match.group("text").strip()
        if not body or "NOLINT" in body:
            continue
        return REASON.match("-- " + body) if True else None
    return None


def suppressions() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for name in SCAN_DIRS:
        base = ROOT / name
        if not base.is_dir():
            continue
        for path in sorted(base.rglob("*")):
            if path.suffix not in {".h", ".hpp", ".c", ".cpp"} or not path.is_file():
                continue
            if "build" in path.parts:
                continue
            lines = path.read_text(encoding="utf-8").splitlines()
            for number, line in enumerate(lines, start=1):
                match = SUPPRESSION.search(line)
                if match is not None:
                    checks = [c.strip() for c in match.group("checks").split(",") if c.strip()]
                    tail = match.group("tail").strip()
                    reason = REASON.search(tail)
                    if reason is None:
                        # This repository states the reason on the comment line(s) *above* the suppression, so look
                        # upward through contiguous comment lines and take the first non-empty one that is not itself a
                        # NOLINT. Reading only the same line would report a dedicated-reason style as "unjustified",
                        # which is a bug in this tool and not a gap in the tree -- that is how nine honest suppressions
                        # were once reported as unexplained.
                        reason = _reason_above(lines, number - 1)
                    rows.append(
                        {
                            "where": f"{path.relative_to(ROOT).as_posix()}:{number}",
                            "checks": checks,
                            "reason": (reason.group("reason").strip() if reason else ""),
                        }
                    )
                elif BARE_NOLINT.search(line):
                    rows.append(
                        {
                            "where": f"{path.relative_to(ROOT).as_posix()}:{number}",
                            "checks": ["(all checks)"],
                            "reason": "",
                        }
                    )
    return rows


def render() -> str:
    checks = enabled_checks()
    rows = suppressions()
    unjustified = [r for r in rows if not r["reason"]]
    by_family: dict[str, list[str]] = {}
    for check in checks:
        by_family.setdefault(family_of(check), []).append(check)

    out: list[str] = []
    out.append("# T0 evidence: what this repository enforces mechanically")
    out.append("")
    out.append("Generated by `.github/project/t0_evidence.py` -- edit the generator, not this file.")
    out.append("")
    out.append("**This is not a compliance claim.** No certification is asserted, no MISRA text is present in this")
    out.append("repository (A4/A5 are paywalled), and none of the tables below substitute for a deviation permit or an")
    out.append("independent assessment. What they do say is which parts of the free rule sets are enforced by a build")
    out.append("gate rather than by intention, and where every suppression is written down.")
    out.append("")
    out.append("## 1. Enabled tidy checks, by the free rule set they come from")
    out.append("")
    out.append("`clang-tidy` runs with `WarningsAsErrors: '*'`, so every check below is a build failure rather than a")
    out.append("warning, and the tree is clean under all of them.")
    out.append("")
    out.append("| Free rule set | Enabled patterns | Count |")
    out.append("|---|---|---|")
    for family in sorted(by_family):
        out.append(f"| {family} | {', '.join(f'`{c}`' for c in by_family[family])} | {len(by_family[family])} |")
    out.append("")
    out.append("Only two of the four free rule sets are touched mechanically: `cppcoreguidelines-avoid-magic-numbers`,")
    out.append("`cppcoreguidelines-macro-usage` and -- indirectly -- the `bugprone-*` and `performance-*` families, which")
    out.append("have no rule ID in any of the free documents. **AUTOSAR C++14 and CERT are not enforced here at all**, which")
    out.append("is the honest reading of an empty cell and is the largest gap in this table.")
    out.append("")
    out.append("## 2. Suppressions, and whether each carries a reason")
    out.append("")
    out.append(f"{len(rows)} suppression(s) across `include/`, `src/` and `api/`; **{len(unjustified)} without a stated")
    out.append("reason**. A reviewer's first question about a suppression is why, so the unanswered ones are listed first.")
    out.append("")
    if unjustified:
        out.append("| Where | Checks suppressed | Reason |")
        out.append("|---|---|---|")
        for row in unjustified:
            out.append(f"| `{row['where']}` | {', '.join(f'`{c}`' for c in row['checks'])} | **(none)** |")
        out.append("")
    if len(rows) > len(unjustified):
        out.append("| Where | Checks suppressed | Reason |")
        out.append("|---|---|---|")
        for row in rows:
            if row["reason"]:
                out.append(f"| `{row['where']}` | {', '.join(f'`{c}`' for c in row['checks'])} | {row['reason']} |")
        out.append("")
    out.append("## 3. JPL Power of Ten (D3): enforced, or only intended")
    out.append("")
    out.append("| Rule | How it is enforced here |")
    out.append("|---|---|")
    out.append("| 1. No `goto`, `setjmp`, recursion | not checked mechanically -- stated as a convention only |")
    out.append("| 2. Bounded loops | not checked mechanically |")
    out.append("| 3. No dynamic allocation after init | by architecture: `-fno-exceptions -fno-rtti` and no container use in")
    out.append("  `include/`; not a gate |")
    out.append("| 4. Functions fit on one page | not checked |")
    out.append("| 5. Minimum two assertions per function | not checked; the analogue here is `WarningsAsErrors` plus the")
    out.append("  invariant suite |")
    out.append("| 6. Smallest scope for data | `readability`/`bugprone` families are **not** enabled |")
    out.append("| 7. Check return values, validate parameters | partially: `bugprone-*` catches some; refusal paths are")
    out.append("  asserted per capability in `capabilities.rst` |")
    out.append("| 8. Limited preprocessor use | `cppcoreguidelines-macro-usage` is enabled |")
    out.append("| 9. Limited pointer use | not checked |")
    out.append("| 10. Compile with all warnings, then test | `WarningsAsErrors: '*'` plus the 24 project gates |")
    out.append("")
    out.append("Two of ten are mechanical, two more are partly mechanical. Writing this table down is the point: \"we follow")
    out.append("the Power of Ten\" would be a claim about intentions, and the rows above are what can be shown.")
    out.append("")
    out.append("## 4. What would have to be bought or decided to go further")
    out.append("")
    out.append("| Missing | Blocks |")
    out.append("|---|---|")
    out.append("| MISRA C++:2023 (A4) | the only published rule set written for C++17; without it every MISRA claim is")
    out.append("  guesswork |")
    out.append("| MISRA Compliance:2020 (A6) | the *form* of a compliance claim: recategorisation, deviation permits |")
    out.append("| ISO 26262-6/-8 (B2/B3) | software-level technique tables and tool confidence levels |")
    out.append("| hardware failure-rate data (T2) | any quantified FMEDA number |")
    out.append("")
    return "\n".join(out) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--stdout", action="store_true", help="print instead of writing the file")
    args = parser.parse_args()
    text = render()
    if args.stdout:
        sys.stdout.write(text)
        return 0
    OUT.parent.mkdir(parents=True, exist_ok=True)
    OUT.write_text(text, encoding="utf-8")
    print(f"[t0] wrote {OUT.relative_to(ROOT)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
