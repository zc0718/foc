#!/usr/bin/env python3
"""Keep the numbers in the documentation equal to the numbers the build produces.

`docs/sphinx/source/usage.rst` claims a verification result for each command it tells the reader
to run. Those claims drifted twice by hand -- 365 cases against a 367-case suite -- because a
number in prose has no owner. This gives them one: the case count comes from CTest's own discovery
(no tests are executed), the coverage and negative-compilation numbers from
`docs/verification-numbers.json`, which the gates that measure them write.

    python3 .github/misc/number_sheet.py --check      # what CI runs
    python3 .github/misc/number_sheet.py --record     # refresh the json from the artefacts

A missing artefact is reported, never assumed correct: a number nobody measured is exactly the
number that goes stale.
"""

from __future__ import annotations

import argparse
import json
import pathlib
import re
import subprocess
import sys

ROOT = pathlib.Path(__file__).resolve().parents[2]
USAGE = ROOT / "docs/sphinx/source/usage.rst"
NUMBERS = ROOT / "docs/verification-numbers.json"
BUILD_ROOTS = sorted((ROOT / "test_package/build").glob("*/CTestTestfile.cmake"))

CASES_ROW = re.compile(r"(\d+)\s+cases,\s*(\d+)\s+failed")
COVERAGE_ROW = re.compile(r"([\d.]+)\s*%\s*overall,\s*([\d.]+)\s*%\s*in\s*:code:`core/`")
NEGATIVE_ROW = re.compile(r"(\d+)\s*/\s*(\d+)\s*cases behave as specified")
TOTAL = re.compile(r"Total Tests:\s*(\d+)")


def measured_cases() -> int | None:
    """CTest's own count, from `ctest -N` -- discovery only, nothing is executed."""
    if not BUILD_ROOTS:
        return None
    build = BUILD_ROOTS[0].parent
    try:
        result = subprocess.run(["ctest", "-N"], cwd=build, capture_output=True, text=True, timeout=120)
    except (OSError, subprocess.SubprocessError):
        return None
    match = TOTAL.search(result.stdout)
    return int(match.group(1)) if match else None


def recorded() -> dict:
    if not NUMBERS.is_file():
        return {}
    return json.loads(NUMBERS.read_text(encoding="utf-8"))


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--check", action="store_true", help="compare the documentation with the artefacts")
    parser.add_argument("--record", action="store_true", help="write the measured numbers to the json")
    parser.add_argument("--cases", type=int, help="case count, when recording outside a build")
    args = parser.parse_args()

    text = USAGE.read_text(encoding="utf-8")
    cases = args.cases or measured_cases()
    stored = recorded()

    if args.record:
        payload = dict(stored)
        if cases is not None:
            payload["cases"] = cases
        NUMBERS.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        print(f"[numbers] recorded {payload}")
        return 0

    problems: list[str] = []
    skipped: list[str] = []

    documented = CASES_ROW.search(text)
    if cases is None:
        skipped.append("case count (no configured build under test_package/build)")
    elif documented is None:
        problems.append("the case-count row is gone from usage.rst")
    elif int(documented.group(1)) != cases:
        problems.append(f"usage.rst says {documented.group(1)} cases, CTest discovers {cases}")

    documented = COVERAGE_ROW.search(text)
    if "coverage_all" not in stored or "coverage_core" not in stored:
        skipped.append("coverage (docs/verification-numbers.json has no coverage entry)")
    elif documented is None:
        problems.append("the coverage row is gone from usage.rst")
    else:
        if float(documented.group(1)) != float(stored["coverage_all"]):
            problems.append(f"usage.rst says {documented.group(1)} % overall, recorded {stored['coverage_all']}")
        if float(documented.group(2)) != float(stored["coverage_core"]):
            problems.append(f"usage.rst says {documented.group(2)} % in core/, recorded {stored['coverage_core']}")

    documented = NEGATIVE_ROW.search(text)
    if "negative_cases" not in stored:
        skipped.append("negative compilation (not recorded; run negative_compile.py --record)")
    elif documented is None:
        problems.append("the negative-compilation row is gone from usage.rst")
    elif int(documented.group(1)) != int(stored["negative_cases"]) or int(documented.group(2)) != int(
        stored["negative_cases"]
    ):
        problems.append(f"usage.rst says {documented.group(1)}/{documented.group(2)}, recorded {stored['negative_cases']}")

    for note in skipped:
        print(f"[numbers] SKIPPED {note}")
    for problem in problems:
        print(f"[numbers] MISMATCH {problem}")
    if problems:
        print(f"[numbers] FAIL {len(problems)} claim(s) disagree with the artefacts")
        return 1
    verified = 3 - len(skipped)
    print(f"[numbers] OK ({verified} claim(s) match the artefacts, {len(skipped)} not measurable here)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
