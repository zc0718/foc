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
`conan create` (`.github/project/coverage_gate.py` never instruments anything itself). Scope rates are
summed from per-file LF/LH counts, **never** averaged over per-file percentages: that would weight a
three-line header like a three-hundred-line source.

Usage
-----
    python3 .github/project/coverage_gate.py                  # floors for this OS, from metadata.json
    python3 .github/project/coverage_gate.py --core 95 --all 80
    python3 .github/project/coverage_gate.py --verbose        # per-file table + margins
    python3 .github/project/coverage_gate.py --list           # files in each scope

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


def _code_mask(text: str) -> list:
    """True for each line that holds code: not blank, not inside a block comment, not a `//` line.

    Written as a two-state scan rather than a regex because `*` and `/` appear in expressions; a rule
    like "starts with `*`" would also swallow a dereference continuation and quietly shrink the
    denominator.
    两态扫描而不是正则：`*` 与 `/` 在表达式里也会出现，靠"以 `*` 开头"判断会连解引用的续行一起吞掉，
    静悄悄地缩小分母。
    """
    mask = []
    in_block = False
    for raw in text.splitlines():
        code = False
        i = 0
        while i < len(raw):
            if in_block:
                end = raw.find("*/", i)
                if end < 0:
                    i = len(raw)
                    break
                i = end + 2
                in_block = False
                continue
            pair = raw[i:i + 2]
            if pair == "//":
                break
            if pair == "/*":
                in_block = True
                i += 2
                continue
            if not raw[i].isspace():
                code = True
            i += 1
        mask.append(code)
    return mask


def recount_without_comment_lines(tracefile, per_file):
    """Drop tracefile records that land on a blank or comment-only line; return (per_file, dropped).

    llvm-cov's lcov export on the macOS toolchain writes `DA:<line>,0` for comment and blank lines,
    and gcov never does -- so the recipe, which counts *distinct DA line numbers*, charges macOS for
    every comment in every header. Measured: `include/foc/core/protection.hpp` reads **133** lines on
    macOS against **104** under clang 18 on Linux, and 31 of the 33 lines macOS calls uncovered are
    comments or blanks (the two real ones, 354 and 361, are uncovered on both runners). Nothing about
    the program differs, so the two must not be measured with different rulers; the fix is to use the
    ruler that counts code.

    Linux is unaffected: gcov emits no such records, so `dropped` is 0 there and every historical
    number stands.
    """
    root = pathlib.Path(__file__).resolve().parents[2]
    counts = {}
    masks = {}
    try:
        lines = tracefile.read_text(encoding="utf-8", errors="replace").splitlines()
    except OSError:
        return per_file, 0
    current = None
    dropped = 0
    for line in lines:
        if line.startswith("SF:"):
            raw = line[3:].strip()
            # The tracefile names the source through whichever build root compiled it -- `<repo>/b/include/...`
            # for the package's own objects, `<repo>/p/include/...` for the consumer's -- and neither exists as
            # written. Locate the project directory in the path and take it from there, which is the same
            # normalisation the recipe's own summary does with `_source_relpath`.
            # tracefile 用「哪个构建根把它编出来的」来命名源文件——包自己的对象是 `<repo>/b/include/...`，消费者
            # 的是 `<repo>/p/include/...`，两者按字面都不存在。在路径里找到项目目录、从那里截取，与 recipe 自己
            # 汇总时 `_source_relpath` 做的归一化一致。
            parts = pathlib.PurePosixPath(raw.replace("\\", "/")).parts
            key = raw
            for marker in ("include", "src", "api", "test_package", "docs"):
                if marker in parts:
                    start = len(parts) - 1 - parts[::-1].index(marker)
                    key = "/".join(parts[start:])
                    break
            current = key
            counts.setdefault(current, {})
            continue
        if current is None or not line.startswith("DA:"):
            continue
        number, _, rest = line[3:].strip().partition(",")
        count = rest.partition(",")[0]
        if not (number.isdigit() and count.isdigit()):
            continue
        number = int(number)
        if current not in masks:
            try:
                masks[current] = _code_mask((root / current).read_text(encoding="utf-8", errors="replace"))
            except OSError:
                masks[current] = None
        mask = masks[current]
        if mask is not None and 1 <= number <= len(mask) and not mask[number - 1]:
            dropped += 1            # a blank or comment line is not a line of code
            continue
        seen = counts[current]
        seen[number] = max(seen.get(number, 0), int(count))
    if not dropped:
        return per_file, 0
    rebuilt = {path: {"lines_hit": sum(1 for c in seen.values() if c > 0), "lines_found": len(seen)}
               for path, seen in counts.items() if seen}
    # Keep any file the tracefile did not mention, so a filter can never *remove* a file from the report.
    for path, entry in per_file.items():
        rebuilt.setdefault(path, entry)
    return rebuilt, dropped


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

    # One ruler for both runners: see `recount_without_comment_lines`. The recipe counts DA records and
    # the macOS toolchain puts comments in them; counting code lines here makes the two comparable, and
    # leaves Linux untouched because gcov never emits those records.
    # 两个 runner 用同一把尺子：见 `recount_without_comment_lines`。recipe 数的是 DA 记录，而 macOS 那套
    # 工具链会把注释写进去；这里改数代码行，两边才可比，且 gcov 从不产出那种记录，Linux 侧不受影响。
    _trace = summary_path.with_name("coverage_test.filtered.info")
    if not _trace.is_file():
        _candidates = sorted(summary_path.parent.glob("*.info"))
        _trace = _candidates[0] if _candidates else None
    if _trace is not None:
        _rebuilt, _dropped = recount_without_comment_lines(_trace, per_file)
        if _dropped:
            per_file = _rebuilt
            core = scope_of(per_file, core_scopes)
            _hit = sum(c.get("lines_hit", 0) for c in per_file.values())
            _found = sum(c.get("lines_found", 0) for c in per_file.values())
            overall = {"hit": _hit, "found": _found, "percent": line_rate(_hit, _found),
                       "files": summary.get("files", [])}
            print(f"[coverage] dropped {_dropped} record(s) that land on comment or blank lines "
                  f"({_trace.name} counts them, gcov does not)", flush=True)
            # Write the corrected counts back. The workflow's own coverage step reads this file next,
            # and if only this gate knew about the correction the two judges would disagree about the
            # same run -- which is the defect this fix is about, not its remedy. The raw numbers stay
            # in the payload so the correction is auditable rather than invisible.
            # 把校正后的计数写回去。workflow 自己那一步随后读的就是这个文件；如果只有本门禁知道这处校正，
            # 两个裁判会对同一次运行给出不同结论——那正是本修复针对的缺陷本身，而不是它的解法。原始数字
            # 留在 payload 里，使这次校正可审计而不是隐形的。
            _previous = {"lines": summary.get("lines"), "per_file": summary.get("per_file")}
            summary["lines"] = {"hit": _hit, "found": _found, "percent": overall["percent"]}
            summary["per_file"] = per_file
            summary["comment_line_filter"] = {"dropped_records": _dropped, "unfiltered": _previous}
            try:
                summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n",
                                        encoding="utf-8")
                print(f"[coverage] rewrote {summary_path.name} with the filtered counts", flush=True)
            except OSError as _exc:
                print(f"[coverage] cannot rewrite {summary_path}: {_exc}; the report is still judged "
                      f"with the filtered counts here", file=sys.stderr)

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
