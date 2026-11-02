#!/usr/bin/env python3
"""Modified condition/decision coverage for the safety subset.

Why this exists
---------------
The plan's stage A4 asked for "safety-subset MC/DC >= 90 %". The baseline refuted the threshold before any code
was written: the subset has **six** conditions in total, so one condition is worth 16.7 points and two
pass/fail results swing the "gate" by 33 -- a ratio nobody can act on. Worse, `llvm-cov report
--show-mcdc-summary` counts only *multi-condition* decisions, and almost every protection mechanism is a single
comparison, so a percentage over the whole subset cannot even see them.

What the measurement did show was worth keeping: two conditions had never been exercised, and neither was a
missing assertion. `protection.hpp`'s `lost` decision short-circuits on `!input.flux_valid`, and no test had ever
fed an invalid flux, so a machine that says "my flux estimate is not valid" reaching `LostSync` with no error
ratio at all was untested behaviour. `sm.cpp`'s `request` guards `to >= kStateCount`, and removing that term
does not merely weaken a check: `kLegal[from][4]` reads past the table, happens to be true, and the machine is
moved into state 4 -- a state that does not exist, with AddressSanitizer silent.

So the criterion this gate enforces is per decision, not per subset: **every multi-condition decision in the
safety subset must have every condition's independence pair covered**, and the subset must contain at least
`min_multi_condition_decisions` such decisions. The second half is the vacuity guard -- a subset that measures
nothing must not report 100 %.

`--selftest` feeds the parser four hand-written `llvm-cov show` excerpts (all covered, one pair missing, no
decisions at all, below the vacuity floor) and requires the verdicts to differ, because a checker that has
never said no is a checker nobody has any reason to trust.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONFIG = Path(__file__).resolve().parent / "mcdc.json"

_DECISION = re.compile(r"MC/DC Decision Region \((\d+):(\d+)\) to \((\d+):(\d+)\)")
_CONDITIONS = re.compile(r"Number of Conditions: (\d+)")
_PAIR = re.compile(r"(C\d+)-Pair: (covered|not covered)")
_PCT = re.compile(r"MC/DC Coverage for Decision: ([\d.]+)%")


def parse_decisions(text: str) -> list[dict]:
    """Every MC/DC decision block in an `llvm-cov show --show-mcdc` excerpt, in source order."""
    out: list[dict] = []
    cur: dict | None = None
    for line in text.splitlines():
        hit = _DECISION.search(line)
        if hit:
            cur = {"line": int(hit.group(1)), "conditions": 0, "pairs": {}, "percent": None}
            continue
        if cur is None:
            continue
        hit = _CONDITIONS.search(line)
        if hit:
            cur["conditions"] = int(hit.group(1))
            continue
        hit = _PAIR.search(line)
        if hit:
            cur["pairs"][hit.group(1)] = hit.group(2) == "covered"
            continue
        hit = _PCT.search(line)
        if hit:
            cur["percent"] = float(hit.group(1))
            out.append(cur)
            cur = None
    return out


def judge(name: str, decisions: list[dict], floor: int) -> list[str]:
    """The problems with one file's measurement, as sentences. Empty means the file is clean."""
    multi = [d for d in decisions if d["conditions"] >= 2]
    problems = []
    for d in multi:
        missing = sorted(cond for cond, covered in d["pairs"].items() if not covered)
        if missing:
            problems.append(f"{name}:{d['line']} has {d['conditions']} conditions and no independent pair for "
                            f"{', '.join(missing)} ({d['percent']} %)")
    if len(multi) < floor:
        problems.append(f"{name} contributes {len(multi)} multi-condition decision(s), which is below the floor of "
                        f"{floor}: the subset would report a perfect score for measuring nothing")
    return problems


def _tool(config: dict, key: str) -> str:
    """The first candidate that runs, so the gate names what it actually measured with."""
    for candidate in config[key]:
        try:
            done = subprocess.run([candidate, "--version"], capture_output=True, text=True, timeout=30)
        except (OSError, subprocess.TimeoutExpired):
            continue
        if done.returncode == 0:
            return candidate
    raise SystemExit(f"[mcdc] none of {config[key]} runs; the measurement cannot be made on this machine")


def _glob(patterns: list[str]) -> list[str]:
    seen: list[str] = []
    for pattern in patterns:
        for path in sorted(ROOT.glob(pattern)):
            rel = path.relative_to(ROOT).as_posix()
            if rel not in seen:
                seen.append(rel)
    return seen


def _packages() -> tuple[Path, Path]:
    """gtest and etl from the conan cache, so the gate needs no install of its own."""
    for base in (Path.home() / ".conan2" / "p", Path.home() / ".conan" / "data"):
        if not base.is_dir():
            continue
        gtest = next(iter(sorted(base.glob("*/p/lib/libgtest.a"))), None)
        etl = next(iter(sorted(base.glob("*/p/include/etl"))), None)
        if gtest is not None and etl is not None:
            return gtest.parents[1], etl.parents[1]
    raise SystemExit("[mcdc] gtest/etl not found in the conan cache; run a build once first")


def measure(config: dict, workdir: Path) -> tuple[list[dict], list[str]]:
    """Build the instrumented suite, run it, and read the per-decision pairs out of it."""
    cxx, cc = _tool(config, "cxx"), _tool(config, "cc")
    profdata, cov = _tool(config, "profdata"), _tool(config, "cov")
    gtest, etl = _packages()
    includes = [f"-I{(ROOT / d).as_posix()}" for d in config["include_dirs"]]
    includes += [f"-I{gtest / 'include'}", f"-I{etl / 'include'}"]
    defines = [f"-D{d}" for d in config["defines"]]
    common = [*config["cxx_flags"], *includes, *defines]

    excluded = set(config["excluded_tests"])
    tests = [rel for rel in _glob(config["tests"]) if Path(rel).name not in excluded]
    objects: list[str] = []
    for rel in [*_glob(config["sources_cxx"]), *tests]:
        obj = workdir / (rel.replace("/", "_") + ".o")
        done = subprocess.run([cxx, *common, "-c", rel, "-o", str(obj)], cwd=ROOT, capture_output=True, text=True)
        if done.returncode != 0:
            raise SystemExit(f"[mcdc] compiling {rel} failed:\n{done.stderr[-2000:]}")
        objects.append(str(obj))
    for rel in _glob(config["sources_c"]):
        obj = workdir / (rel.replace("/", "_") + ".o")
        done = subprocess.run([cc, *config["c_flags"], *includes, *defines, "-c", rel, "-o", str(obj)],
                              cwd=ROOT, capture_output=True, text=True)
        if done.returncode != 0:
            raise SystemExit(f"[mcdc] compiling {rel} failed:\n{done.stderr[-2000:]}")
        objects.append(str(obj))

    binary = workdir / "mcdc"
    done = subprocess.run([cxx, "-fprofile-instr-generate", *objects, "-o", str(binary), *config["link_flags"],
                           gtest / "lib" / "libgtest_main.a", gtest / "lib" / "libgtest.a", "-lpthread", "-lm"],
                          cwd=ROOT, capture_output=True, text=True)
    if done.returncode != 0:
        raise SystemExit(f"[mcdc] link failed:\n{done.stderr[-2000:]}")

    raw = workdir / "run.profraw"
    raw.unlink(missing_ok=True)
    env = {"LLVM_PROFILE_FILE": str(raw)}
    import os
    run = subprocess.run([str(binary), f'--gtest_filter={config["run_filter"]}'], cwd=ROOT, text=True,
                         capture_output=True, env={**os.environ, **env})
    if run.returncode != 0:
        tail = [line for line in run.stdout.splitlines() if line.startswith("[  FAILED  ]")][:5]
        raise SystemExit("[mcdc] the instrumented suite is red, so its coverage describes the wrong program:\n  "
                         + "\n  ".join(tail or [run.stdout[-800:]]))

    merged = workdir / "run.profdata"
    done = subprocess.run([profdata, "merge", "-sparse", str(raw), "-o", str(merged)], capture_output=True, text=True)
    if done.returncode != 0:
        raise SystemExit(f"[mcdc] llvm-profdata failed:\n{done.stderr[-2000:]}")

    rows, problems = [], []
    for entry in config["subset"]:
        target = ROOT / entry["file"]
        if not target.exists():
            problems.append(f"{entry['file']} is in the safety subset but does not exist")
            continue
        done = subprocess.run([cov, "show", "--show-mcdc", str(binary), f"-instr-profile={merged}",
                               str(target)], capture_output=True, text=True)
        decisions = parse_decisions(done.stdout)
        problems += judge(entry["file"], decisions, entry.get("min_multi_condition_decisions", 0))
        rows.append({"file": entry["file"], "why": entry["why"], "decisions": len(decisions),
                     "multi": [d for d in decisions if d["conditions"] >= 2]})
    total = sum(len(row["multi"]) for row in rows)
    if total < config["min_multi_condition_decisions"]:
        problems.append(f"the safety subset measured {total} multi-condition decision(s) in total, below the floor of "
                        f"{config['min_multi_condition_decisions']}")
    return rows, problems


SELFTEST_COVERED = """
  |---> MC/DC Decision Region (45:9) to (45:49)
  |  Number of Conditions: 2
  |  C1-Pair: covered: (1,2)
  |  C2-Pair: covered: (1,3)
  |  MC/DC Coverage for Decision: 100.00%
"""

SELFTEST_MISSING = """
  |---> MC/DC Decision Region (302:18) to (302:103)
  |  Number of Conditions: 2
  |  C1-Pair: not covered
  |  C2-Pair: covered: (1,3)
  |  MC/DC Coverage for Decision: 50.00%
"""

SELFTEST_EMPTY = "  |  just a covered line, no decision at all\n"


def selftest() -> int:
    """Four excerpts, four verdicts: a checker that cannot say no is not a checker."""
    cases = [
        ("all pairs covered", SELFTEST_COVERED, 1, 0, None),
        ("one pair missing", SELFTEST_MISSING, 1, 1, "C1"),
        ("nothing measured", SELFTEST_EMPTY, 1, 1, "below the floor"),
        ("nothing measured, no floor", SELFTEST_EMPTY, 0, 0, None),
    ]
    ok = True
    for name, text, floor, expect, needle in cases:
        problems = judge("f.hpp", parse_decisions(text), floor)
        good = len(problems) == expect and (needle is None or any(needle in p for p in problems))
        ok = ok and good
        print(f"[mcdc-selftest] {'OK  ' if good else 'BAD '} {name}: {problems or 'no problems'}", flush=True)
    if not ok:
        print("[mcdc-selftest] FAILED: the checker does not distinguish these cases", flush=True)
        return 1
    print(f"[mcdc-selftest] OK: {len(cases)} excerpt(s) judged as expected", flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true", help="judge hand-written excerpts instead of measuring")
    parser.add_argument("--workdir", type=Path, default=None, help="keep the instrumented objects here")
    args = parser.parse_args()
    if args.selftest:
        return selftest()

    config = json.loads(CONFIG.read_text(encoding="utf-8"))
    workdir = args.workdir or Path(tempfile.mkdtemp(prefix="mcdc-"))
    workdir.mkdir(parents=True, exist_ok=True)
    rows, problems = measure(config, workdir)

    for row in rows:
        covered = sum(1 for d in row["multi"] if all(d["pairs"].values()))
        print(f"[mcdc] {row['file']:<38} multi-condition decisions: {len(row['multi']):>2}  "
              f"fully covered: {covered:>2}  ({row['why']})", flush=True)
    if problems:
        print("[mcdc] FAILED:", flush=True)
        for problem in problems:
            print(f"  - {problem}", flush=True)
        return 1
    total = sum(len(row["multi"]) for row in rows)
    print(f"[mcdc] OK: every one of the {total} multi-condition decision(s) in the safety subset has all of its "
          f"condition pairs covered", flush=True)
    print("[mcdc] note: this is structural coverage of the subset, not a certification and not a claim about the "
          "code outside it", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
