#!/usr/bin/env python3
"""G7 coverage gate: enforce line-rate floors per scope from the recipe's coverage summary.

Why a scope, not one number
---------------------------
A single "library coverage" figure hides the thing that matters: the algorithms can be 100 % while
a 300-line peripheral drags the average down, or the opposite. The PRD therefore fixes two floors —
core algorithms >= 90 %, whole library >= 70 % — and this gate reports both, plus the per-file
breakdown so a regression is attributable instead of mysterious.

Where the numbers come from
---------------------------
`test_package/test/export/coverage/coverage_summary.json`, written by the coverage leg of
`conan create` (`.github/misc/coverage_gate.py` never instruments anything itself). Scope rates are
summed from per-file LF/LH counts, **never** averaged over per-file percentages: that would weight a
three-line header like a three-hundred-line source.

Usage
-----
    python3 .github/misc/coverage_gate.py                  # floors for this OS, from metadata.json
    python3 .github/misc/coverage_gate.py --core 95 --all 80
    python3 .github/misc/coverage_gate.py --verbose        # per-file table + margins
    python3 .github/misc/coverage_gate.py --list           # files in each scope

Exit codes: 0 pass, 1 a floor is violated, 2 the summary is missing or too old to judge.
"""
import argparse
import json
import pathlib
import platform
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SUMMARY = ROOT / "test_package" / "test" / "export" / "coverage" / "coverage_summary.json"

# Fallbacks, used only when metadata.json says nothing. The real values live there (`coverage.core_scopes`,
# `coverage.floors`, `coverage.default_floor`) so that this gate and the CI step assert the same numbers.
# 下面的值只是 metadata.json 缺席时的兜底；真值在那份文件里，好让本地与 CI 断言同一组数字。
DEFAULT_CORE_SCOPES = ("include/foc/core/", "src/core/")
DEFAULT_CORE_FLOOR = 90.0
DEFAULT_ALL_FLOOR = 70.0


def line_rate(hit, found):
    return round(100.0 * hit / found, 2) if found else 0.0


def scope_of(per_file, prefixes):
    """Sum the counts of every file under one of `prefixes`."""
    hit = found = 0
    files = []
    for path, counts in sorted(per_file.items()):
        if any(path.startswith(prefix) for prefix in prefixes):
            hit += counts.get("lines_hit", 0)
            found += counts.get("lines_found", 0)
            files.append(path)
    return {"hit": hit, "found": found, "percent": line_rate(hit, found), "files": files}


def policy():
    """The floors and the core scope, from `metadata.json` -- the same file CI reads.

    One source of truth on purpose: this gate and the workflow assert the *same* numbers, so a local green
    and a runner green mean the same thing. The floors are per-runner (`floors.<os>`), because gcov and
    llvm-cov do not count the same lines and the gap between them is the instrument, not the code; the
    runner this script is on picks its own row, and anything unlisted falls back to `default_floor`.
    """
    meta_path = ROOT / "metadata.json"
    try:
        coverage = json.loads(meta_path.read_text(encoding="utf-8")).get("coverage", {})
    except (OSError, json.JSONDecodeError):
        return (), {}, {}
    leg = {"Linux": "Linux", "Darwin": "macOS", "Windows": "Windows"}.get(platform.system())
    floors = coverage.get("floors", {}).get(leg) or coverage.get("default_floor", {})
    scopes = tuple(coverage.get("core_scopes", []))
    return scopes, floors, coverage


def main():
    scopes, floors, _coverage = policy()
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default=str(DEFAULT_SUMMARY), help="coverage_summary.json path")
    parser.add_argument("--core", type=float, default=float(floors.get("core", DEFAULT_CORE_FLOOR)),
                        help="core-scope line-rate floor in %% (default: metadata.json for this OS)")
    parser.add_argument("--all", type=float, default=float(floors.get("all", DEFAULT_ALL_FLOOR)),
                        help="whole-library line-rate floor in %% (default: metadata.json for this OS)")
    parser.add_argument("--core-scope", action="append", default=None,
                        help="path prefix counted as core (repeatable)")
    parser.add_argument("--verbose", action="store_true", help="print the per-file table")
    parser.add_argument("--list", action="store_true", help="list the files in each scope and exit")
    args = parser.parse_args()

    summary_path = Path(args.input)
    if not summary_path.is_file():
        print(f"[coverage] no summary at {summary_path}; run `conan create` with coverage ON first",
              file=sys.stderr)
        return 2

    summary = json.loads(summary_path.read_text(encoding="utf-8"))
    per_file = summary.get("per_file")
    if not per_file:
        print("[coverage] the summary has no per-file data (older recipe); regenerate it before "
              "judging a scope", file=sys.stderr)
        return 2

    core_scopes = tuple(args.core_scope) if args.core_scope else (scopes or DEFAULT_CORE_SCOPES)
    core = scope_of(per_file, core_scopes)
    overall = {"hit": summary["lines"]["hit"], "found": summary["lines"]["found"],
               "percent": summary["lines"]["percent"], "files": summary.get("files", [])}

    if args.list:
        print(f"core scope {core_scopes}:")
        for path in core["files"]:
            print(f"  {path}")
        print(f"all scope: {len(overall['files'])} file(s)")
        return 0

    if args.verbose:
        print(f"{'file':<44}{'lines':>7}{'hit':>7}{'rate':>8}")
        for path, counts in sorted(per_file.items()):
            rate = line_rate(counts.get("lines_hit", 0), counts.get("lines_found", 0))
            print(f"{path:<44}{counts.get('lines_found', 0):>7}{counts.get('lines_hit', 0):>7}"
                  f"{rate:>7.2f}%")
        print()

    failures = 0
    rows = (("core", core, args.core), ("all", overall, args.all))
    for name, stats, floor in rows:
        # Exact counts, never the rounded rate: the CI step compares against `100 * hit / found`, and a
        # rounded 95.70 that is really 95.6989 must not pass a 95.7 floor here and fail it there.
        rate = 100.0 * stats["hit"] / stats["found"] if stats["found"] else 0.0
        margin = rate - floor
        verdict = "ok" if margin >= 0.0 else "FAIL"
        print(f"[coverage] {verdict:<4} {name:<5} {rate:>6.2f}% "
              f"({stats['hit']}/{stats['found']} lines, floor {floor:.1f}%, margin {margin:+.2f})")
        if margin < 0.0:
            failures += 1

    # Name what to fix, worst file first: a bare "core is 89.4 %" is not actionable.
    if failures:
        worst = sorted(((line_rate(c.get("lines_hit", 0), c.get("lines_found", 0)), p)
                        for p, c in per_file.items() if c.get("lines_found", 0)),
                       key=lambda item: item[0])[:5]
        print("[coverage] worst files:", file=sys.stderr)
        for rate, path in worst:
            print(f"[coverage]   {rate:>6.2f}%  {path}", file=sys.stderr)

    # Record what was measured, so the numbers printed in the documentation can be checked
    # against something. A number nobody wrote down is the number that goes stale.
    # 把量到的记下来，好让文档里印的数字有东西可对。没人写下来的数字，就是会过期的数字。
    numbers = pathlib.Path(__file__).resolve().parents[2] / "docs/verification-numbers.json"
    payload = json.loads(numbers.read_text(encoding="utf-8")) if numbers.is_file() else {}
    payload["coverage_all"] = round(overall["percent"], 2)
    payload["coverage_core"] = round(core["percent"], 2)
    numbers.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")

    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
