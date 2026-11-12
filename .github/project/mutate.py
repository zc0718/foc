#!/usr/bin/env python3
"""Mutation score for the four modules whose numbers other capabilities quote.

Coverage answers "was this line executed"; it cannot answer "would a wrong line be noticed". That distinction is the
whole of G-B in the plan: a 95 % covered module with a test that only checks `is_finite()` has no teeth, and nothing in
the current gate set says so. This harness makes the teeth measurable: it edits one operator or constant at a time,
rebuilds the module's own tests, and counts how many wrong versions the tests reject.

How a number is produced (the rules are written here, not discovered afterwards):

  * **killed** -- the test binary exits non-zero. A crash, a timeout and an assertion failure all count: the test
    noticed. Timeouts are counted separately so a hanging mutant cannot hide.
  * **survived** -- the binary still passes. This is the interesting number; every survivor is printed with its file,
    line, original text and the replacement, because "survived" means either the tests are blind *or* the mutant is
    equivalent, and only a human can say which.
  * **invalid** -- the mutant does not compile. Excluded from the denominator: a compile error is not evidence about
    the tests. Counted anyway, because a high invalid rate means the operators are hitting declarations rather than
    arithmetic.
  * **score** = killed / (killed + survived).

Two refusals that keep the measurement honest. First, the unmutated build must pass before any mutant runs -- if the
baseline is red, every "killed" is meaningless and the harness stops. Second, `--check` fails when a module has no
recorded baseline: the plan's rule for stage 2 is that a baseline is *recorded*, not skipped because the number would
be ugly.

Nothing here is a threshold yet. Stage 2 records where the teeth are; the gate only refuses a *drop* of more than
`MAX_DROP_POINTS` against the recorded score, and refuses a module that quietly disappears.

    python3 .github/project/mutate.py --module transforms --record
    python3 .github/project/mutate.py --check                 # every module against the baseline
    python3 .github/project/mutate.py --module pi --list      # survivors only, for classification
    python3 .github/project/mutate.py --selftest              # prove the loop kills what it should
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
BASELINE = Path(__file__).resolve().with_name("mutation-baseline.json")
MAX_DROP_POINTS = 5.0
TIMEOUT_S = 20
GTEST_HINT = "gtest"

# What each module is allowed to claim through. `mutate` is where the arithmetic lives -- the header holds the inline
# policy code and the .cpp the out-of-line part, so a mutation in either is a mutation of the module.
# Everything the SIL `Motor` instantiates: `test_protection.cpp` drives the plant end to end, so its binary needs the
# observer, modulator, controller and driver stubs as well as the module under test. `src/version.c` is here because
# g++ compiles it as C++ by suffix and `version.h` declares its symbol without `extern "C"`.
_MOTOR_SOURCES = ["src/core/**/*.cpp", "src/motor/*.cpp", "src/driver/*.cpp", "src/version.c"]
MODULES = {
    "transforms": {
        "mutate": ["include/foc/core/transforms.hpp", "src/core/transforms.cpp"],
        "tests": ["test_package/test/unit/test_transforms.cpp"],
        "extra": [],
    },
    "svpwm": {
        "mutate": ["include/foc/core/svpwm.hpp", "src/core/svpwm.cpp"],
        "tests": ["test_package/test/unit/test_svpwm.cpp"],
        "extra": [],
    },
    "pi": {
        "mutate": ["include/foc/core/pi.hpp", "src/core/pi.cpp"],
        "tests": ["test_package/test/unit/test_pi.cpp"],
        "extra": [],
    },
    "observer": {
        "mutate": ["include/foc/core/observer/nonlinear_flux_observer.hpp",
                   "src/core/observer/nonlinear_flux_observer.cpp"],
        "tests": ["test_package/test/unit/test_nonlinear_flux_observer.cpp"],
        "extra": ["src/core/observer/back_emf_observer.cpp", "src/core/observer/ideal_observer.cpp",
                  "src/core/observer/hfi_observer.cpp", "src/core/filters.cpp"],
    },
    # The safety subset. It joins the harness for the reason `mcdc` exists but cannot answer: almost every
    # protection mechanism is a single comparison, so per-decision MC/DC reports 100 % for reaching it and says
    # nothing about whether a wrong threshold, a wrong inequality or a dropped latch would be noticed. The
    # harness measures exactly that, and the three modules deliberately use their *own* test files -- the number
    # is "the teeth of the unit tests", and a survivor that a consumer test would catch is recorded as such
    # during classification rather than hidden by widening the binary.
    # 安全子集。它加入变异基线，理由正是 `mcdc` 存在却回答不了的那件事：保护机理几乎全是单比较，逐判定
    # MC/DC 只能说“走到了”，说不出“阈值写错、不等号写反、锁存丢了”会不会被发现。这三个模块刻意只绑**自己**
    # 的测试文件——这个数是“单元测试的牙齿”，消费者能抓到的存活变异在分类时写明，而不是靠扩大二进制把它盖住。
    "protection": {
        "mutate": ["include/foc/core/protection.hpp"],
        "tests": ["test_package/test/unit/test_protection.cpp"],
        "extra": _MOTOR_SOURCES,
        "classify": True,
    },
    "sm": {
        "mutate": ["include/foc/core/sm.hpp", "src/core/sm.cpp"],
        "tests": ["test_package/test/unit/test_sm.cpp"],
        "extra": [],
        "classify": True,
    },
    "limits": {
        "mutate": ["include/foc/core/limits.hpp", "src/core/limits.cpp"],
        "tests": ["test_package/test/unit/test_limits.cpp"],
        "extra": [],
        "classify": True,
    },
    "commissioning_flow": {
        # Header-only, and its test needs nothing but gtest: the flow owns no estimators and touches no HAL, which is
        # also why it is the cheapest module in this table to mutate.
        # 纯头，且它的用例除 gtest 之外什么都不需要：流程不拥有估计器、也不碰 HAL，这也正是本表里变异代价最低的模块。
        "mutate": ["include/foc/core/commissioning_flow.hpp"],
        "tests": ["test_package/test/unit/test_commissioning_flow.cpp"],
        "extra": [],
    },
    # The three added on 2026-11-12, once `commissioning_flow` had proved the shape of a header-only entry. Until then
    # they had only ever been probed with three hand-picked mutations each -- which is an assertion about which
    # mutations I thought of, not about the teeth of their tests. The number that replaces it is below.
    # 2026-11-12 新增的三个，此时 `commissioning_flow` 已经验证了"纯头条目"的形态。在此之前它们只经过我手挑三个
    # 变异的探查——那是对"我想到了哪些变异"的断言，不是对用例牙齿的断言。下面那个数才是。
    "bus_current_limit": {
        # The limiter itself is header-only, but its test drives a whole axis (a plant, the current loop, the
        # modulator), so the motor's out-of-line half has to be in the link or nothing measures the limiter.
        # 限制器本身是纯头，但它的用例驱动一整根轴（被控对象、电流环、调制器），所以电机的带外实现必须在链接里，
        # 否则量到的不是限制器。
        "mutate": ["include/foc/core/bus_current_limit.hpp"],
        "tests": ["test_package/test/unit/test_bus_current_limit.cpp"],
        "extra": _MOTOR_SOURCES,
        "classify": True,
    },
    "move_trajectory": {
        # Same reason: the case that matters is the axis reaching the target, and that is a motor tick.
        # 同理：真正要紧的那条用例是"轴到达目标"，而那是电机的一拍。
        "mutate": ["include/foc/core/move_trajectory.hpp"],
        "tests": ["test_package/test/unit/test_move_trajectory.cpp"],
        "extra": _MOTOR_SOURCES,
        "classify": True,
    },
    "angle_source": {
        # This one is genuinely alone: its test feeds offers it builds itself and asserts the arbitration, so no
        # source file is needed to link it.
        # 这一个确实是孤立的：它的用例自己造候选、断言仲裁结果，所以链接时不需要任何源文件。
        "mutate": ["include/foc/core/angle_source.hpp"],
        "tests": ["test_package/test/unit/test_angle_source.cpp"],
        "extra": [],
        "classify": True,
    },
}

BINARY_OPS = {"+": "-", "-": "+", "*": "/", "/": "*", "<": "<=", ">": ">=", "<=": "<", ">=": ">",
              "==": "!=", "!=": "==", "&&": "||", "||": "&&"}
OP_RE = re.compile(r"(?<=\s)(<=|>=|==|!=|&&|\|\||[+\-*/<>])(?=\s)")
NUM_RE = re.compile(r"(?<![\w.])(\d+(?:\.\d+)?)([FU])?")
_LITERAL = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'')
_BLOCK = re.compile(r"/\*.*?\*/")


def _masked(raw: str) -> str:
    """The part of a line a mutation can change behaviour through, with the rest blanked to spaces.

    Length-preserving, so the columns the mutator reports still point at the original line. A constant inside
    `"kLegal's row 0 is Idle"` is not a constant: changing it changes a message that is printed only if the assertion
    fails, and it fails for nobody. Four such sites were being counted as survivors in `sm`, which is work for a human
    on mutants no test could ever distinguish -- the opposite of what this harness is for.
    """
    text = _LITERAL.sub(lambda match: " " * len(match.group(0)), raw)
    text = text.split("//", 1)[0]
    return _BLOCK.sub(lambda match: " " * len(match.group(0)), text)


def _gtest_root() -> Path | None:
    """The gtest package the conan cache already holds, so this needs no extra install."""
    for base in (Path.home() / ".conan2" / "p", Path.home() / ".conan" / "data"):
        if not base.is_dir():
            continue
        for candidate in sorted(base.glob("*/p/lib/libgtest.a")):
            return candidate.parents[1]
    return None


def _expand(patterns: list[str]) -> list[str]:
    """Glob patterns and plain paths alike, in the order given, without duplicates.

    `protection`'s test builds the whole `Motor`, so its binary needs every source the plant instantiates. Listing
    them by hand would rot silently the day a new source appears -- the link error would name a symbol, not a
    policy -- and a glob that overlaps the mutated module would define it twice, so the expansion dedupes.
    """
    out: list[str] = []
    for pattern in patterns:
        matches = sorted(ROOT.glob(pattern)) if any(ch in pattern for ch in "*?[") else [ROOT / pattern]
        for path in matches:
            rel = path.relative_to(ROOT).as_posix()
            if rel.endswith((".cpp", ".c")) and rel not in out:
                out.append(rel)
    return out


def _compile(module: str, gtest: Path, out: Path) -> subprocess.CompletedProcess:
    spec = MODULES[module]
    # The mutated .cpp files are part of the build as well as the mutation targets: a header-only module links without
    # them, but the out-of-line half does not, and a module silently built from headers alone would report kills for
    # code that was never compiled.
    sources = _expand([*spec["tests"], *spec["mutate"], *spec["extra"]])
    # Both sanitisers are part of the instrument, and the pair is not redundant -- it was measured:
    #
    #   * AddressSanitizer catches an overrun of a *standalone* object (`flags[8]` is a local array with a redzone,
    #     so `code <= 8` aborts). It does **not** catch an overrun of an array that is a member of a larger object,
    #     because the redzone only surrounds the object: `tripped_[8]` lands on the next member and reads it happily.
    #   * UndefinedBehaviorSanitizer's `array-bounds` checks the index against the declared bound wherever it is, so
    #     the intra-object case aborts too. `-fno-sanitize-recover` is what makes a report an exit code rather than a
    #     line of output on a still-green run.
    # A one-line probe of each (struct with `bool a[8]` and `a[8] = false`) reported 0 and 1 findings respectively,
    # which is why the harness uses both instead of assuming one covers the other.
    cmd = ["g++", "-std=c++17", "-O1", "-fsanitize=address,undefined", "-fno-sanitize-recover=all", "-g",
           "-I", "include", "-I", str(gtest / "include"),
           *sources, "-L", str(gtest / "lib"), "-lgtest", "-lgtest_main", "-lpthread",
           "-o", str(out)]
    return subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)


def _run(out: Path) -> tuple[str, int]:
    """('killed'|'survived', returncode). A timeout is a kill: the tests noticed.

    Leak detection is off on purpose: gtest keeps its registry alive until exit, so every run would report a leak and
    every mutant would be "killed" for a reason that has nothing to do with the mutant. What is wanted from the
    sanitiser is the *memory error* verdict, and an out-of-bounds read or write aborts with a non-zero code.
    """
    env = {**os.environ, "ASAN_OPTIONS": "detect_leaks=0:halt_on_error=1"}
    try:
        done = subprocess.run([str(out)], cwd=ROOT, capture_output=True, text=True, timeout=TIMEOUT_S, env=env)
        return ("survived" if done.returncode == 0 else "killed"), done.returncode
    except subprocess.TimeoutExpired:
        return "killed", -1


def _candidates(path: Path, limit: int) -> tuple[list[dict], bool]:
    """Operator swaps and constant perturbations in file order, on code lines only.

    Returns ``(sites, truncated)``. `truncated` matters: a limit that silently stops the scan turns "the score of the
    module" into "the score of the first N sites", and the tail of the file is then never measured. It is reported
    and recorded rather than left to be inferred from a round number."""
    sites: list[dict] = []
    lines = path.read_text(encoding="utf-8").splitlines()
    for lineno, raw in enumerate(lines, start=1):
        stripped = _masked(raw)
        if not stripped.strip() or stripped.lstrip().startswith(("#", "*", "/*", "///")):
            continue
        for match in OP_RE.finditer(stripped):
            sites.append({"line": lineno, "start": match.start(), "end": match.end(),
                          "original": match.group(1), "replacement": BINARY_OPS[match.group(1)],
                          "operator": "swap"})
        for match in NUM_RE.finditer(stripped):
            token = match.group(1)
            try:
                value = float(token) if "." in token else int(token)
            except ValueError:
                continue
            if "." in token:
                new = f"{value * 1.001:.6g}"
            else:
                new = str(value + 1)
            # A constant that keeps its value is not a mutant. `0.0 * 1.001` is `0.0`, so every `0.0F` in a module
            # produced a "survivor" that no test could ever kill -- twelve of them in `move_trajectory`, which is the
            # same failure `_masked` fixed for string literals: work for a human on mutants nothing can distinguish.
            # 保持原值的常数不是变异体。`0.0 * 1.001` 就是 `0.0`，于是每个 `0.0F` 都会产出一个任何用例都不可能杀掉的
            # "存活者"——`move_trajectory` 里就有十二个。这与 `_masked` 为字符串字面量修掉的是同一个毛病：
            # 让人去做一件没有东西能分辨出来的工作。
            try:
                if float(new) == float(value):
                    continue
            except ValueError:
                continue
            sites.append({"line": lineno, "start": match.start(1), "end": match.end(1),
                          "original": token, "replacement": new, "operator": "constant"})
        if len(sites) >= limit:
            return sites[:limit], True
    return sites, False


def measure(module: str, gtest: Path, limit: int, workdir: Path) -> dict:
    """Run every mutant of one module. Returns the record that goes into the baseline."""
    spec = MODULES[module]
    binary = workdir / f"mut_{module}"
    clean = _compile(module, gtest, binary)
    if clean.returncode != 0:
        raise SystemExit(f"[mutate] {module}: the UNMUTATED build does not compile, so no mutant can be judged:\n"
                         f"{clean.stderr[-800:]}")
    verdict, _code = _run(binary)
    if verdict != "survived":
        raise SystemExit(f"[mutate] {module}: the unmutated tests FAIL, so every kill would be meaningless")

    record = {"mutate": spec["mutate"], "tests": spec["tests"], "limit_per_file": limit,
              "killed": 0, "survived": 0, "invalid": 0, "timeout": 0, "survivors": [], "invalid_sites": 0,
              "truncated": []}
    started = time.time()
    for rel in spec["mutate"]:
        path = ROOT / rel
        original = path.read_text(encoding="utf-8")
        sites, truncated = _candidates(path, limit)
        if truncated:
            record["truncated"].append(rel)
        try:
            for site in sites:
                lines = original.splitlines(keepends=True)
                line = lines[site["line"] - 1]
                mutated_line = line[:site["start"]] + site["replacement"] + line[site["end"]:]
                if mutated_line == line:
                    continue
                lines[site["line"] - 1] = mutated_line
                path.write_text("".join(lines), encoding="utf-8")
                try:
                    built = _compile(module, gtest, binary)
                    if built.returncode != 0:
                        record["invalid"] += 1
                        record["invalid_sites"] += 1
                        continue
                    verdict, code = _run(binary)
                finally:
                    path.write_text(original, encoding="utf-8")
                if verdict == "killed":
                    record["killed"] += 1
                    if code == -1:
                        record["timeout"] += 1
                else:
                    record["survived"] += 1
                    record["survivors"].append({"file": rel, "line": site["line"], "col": site["start"] + 1,
                                                "original": site["original"],
                                                "replacement": site["replacement"],
                                                "operator": site["operator"]})
        finally:
            path.write_text(original, encoding="utf-8")
    judged = record["killed"] + record["survived"]
    record["judged"] = judged
    record["score"] = round(100.0 * record["killed"] / judged, 1) if judged else None
    record["seconds"] = round(time.time() - started, 1)
    return record


def _summary(module: str, rec: dict) -> str:
    scope = "" if not rec.get("truncated") else f"  [PARTIAL: {', '.join(rec['truncated'])} hit the limit]"
    return (f"[mutate] {module:11} killed {rec['killed']:3} / judged {rec['judged']:3} "
            f"= {rec['score']:5.1f}%  (survived {rec['survived']:2}, invalid {rec['invalid']:2}, "
            f"timeout {rec['timeout']}, {rec['seconds']}s){scope}")


def selftest() -> int:
    """The generator must transform code it is supposed to transform, and leave the rest alone.

    The cases encode the rules, not the implementation: a shift *count* is a constant like any other and mutating it
    changes what the code does, so `x <<= 2` yields exactly one constant mutant and no operator swap -- writing this
    case as "skip everything" was wrong, and the selftest caught it."""
    failures = []
    cases = [("        y = a + b;", "        y = a - b;", {"swap"}),
             ("        if (x < limit) {", "        if (x <= limit) {", {"swap"}),
             ("        return 0.5F * v;", "        return 0.5005F * v;", {"constant", "swap"}),
             ("    // a + b in a comment", None, set()),
             ("    static_assert(x == 1, \"row 0 is Idle\");", "    static_assert(x != 1, \"row 0 is Idle\");",
              {"swap", "constant"}),
             ("    log(\"value 0 of 8\");", None, set()),
             ("    const Foo* p = &foo;", None, set()),
             ("    x <<= 2;", "    x <<= 3;", {"constant"})]
    tmp = Path(tempfile.mkdtemp()) / "case.cpp"
    try:
        for text, expected, operators in cases:
            tmp.write_text(text + "\n", encoding="utf-8")
            sites, _truncated = _candidates(tmp, 10)
            if expected is None:
                if sites:
                    failures.append(f"{text.strip()!r} should have no mutant, got {sites[0]}")
                continue
            if {site["operator"] for site in sites} - operators:
                failures.append(f"{text.strip()!r} produced operators outside {operators}: {sites}")
                continue
            produced = []
            for site in sites:
                line = text + "\n"
                mutated = line[:site["start"]] + site["replacement"] + line[site["end"]:]
                produced.append(mutated.strip())
            if expected.strip() not in produced:
                failures.append(f"{text.strip()!r} did not produce {expected.strip()!r}; got {produced}")
    finally:
        shutil.rmtree(tmp.parent, ignore_errors=True)
    if failures:
        for failure in failures:
            print(f"[mutate] SELFTEST FAILED: {failure}", file=sys.stderr)
        return 1
    print(f"[mutate] selftest: {len(cases)} generator case(s) behave as specified")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--module", choices=sorted(MODULES), action="append", help="restrict to these modules")
    ap.add_argument("--record", action="store_true", help="write the measured scores into the baseline")
    ap.add_argument("--check", action="store_true", help="compare against the recorded baseline and fail on a drop")
    ap.add_argument("--list", action="store_true", help="print the survivors with their sites")
    ap.add_argument("--limit", type=int, default=200, help="mutants per file (default 200: whole file, no prefix)")
    ap.add_argument("--selftest", action="store_true", help="prove the generator and the loop work")
    args = ap.parse_args()

    if args.selftest:
        return selftest()

    gtest = _gtest_root()
    if gtest is None:
        message = "[mutate] no gtest package in the conan cache; the measurement cannot be made here"
        if args.check:
            print(message, file=sys.stderr)
            return 1
        raise SystemExit(message)

    modules = args.module or sorted(MODULES)
    baseline = json.loads(BASELINE.read_text(encoding="utf-8")) if BASELINE.is_file() else {"modules": {}}
    problems: list[str] = []
    with tempfile.TemporaryDirectory(prefix="mutate-") as tmp:
        workdir = Path(tmp)
        for module in modules:
            record = measure(module, gtest, args.limit, workdir)
            print(_summary(module, record), flush=True)
            if args.list:
                for survivor in record["survivors"]:
                    print(f"[mutate]   {survivor['file']}:{survivor['line']}:{survivor.get('col', '?')}  "
                          f"{survivor['original']!r} -> {survivor['replacement']!r}")
            old = (baseline.get("modules") or {}).get(module)
            if args.list:
                published = {f"{e['file']}:{e['line']}:{e['col']}" for e in (old or {}).get("equivalents", [])}
                for survivor in record["survivors"]:
                    key = f"{survivor['file']}:{survivor['line']}:{survivor['col']}"
                    if key not in published:
                        print(f"[mutate]   UNCLASSIFIED {key}  {survivor['original']!r} -> "
                              f"{survivor['replacement']!r}")
            if args.check:
                if old is None:
                    problems.append(f"{module}: no recorded baseline -- stage 2 requires the number to exist, "
                                    f"not to be skipped because it looks bad")
                elif record["score"] is not None and old.get("score") is not None and \
                        record["score"] < old["score"] - MAX_DROP_POINTS:
                    problems.append(f"{module}: {record['score']:.1f}% is more than {MAX_DROP_POINTS:.0f} points "
                                    f"below the recorded {old['score']:.1f}% -- the tests got blunter")
                if MODULES[module].get("classify") and old is not None:
                    published = {f"{e['file']}:{e['line']}:{e['col']}" for e in old.get("equivalents", [])}
                    for survivor in record["survivors"]:
                        key = f"{survivor['file']}:{survivor['line']}:{survivor['col']}"
                        if key not in published:
                            problems.append(f"{module}: {key} survives and is not classified -- a survivor is either a "
                                            f"blind spot or an equivalent mutant, and leaving it unlabelled means "
                                            f"nobody decided which")
                    if not old.get("equivalents") and record["survivors"]:
                        problems.append(f"{module}: {len(record['survivors'])} survivor(s) and no `equivalents` entry "
                                        f"at all")
            if args.record:
                kept = {k: v for k, v in (old or {}).items() if k in ("equivalents", "notes")}
                baseline.setdefault("modules", {})[module] = {**record, **kept,
                                                              "recorded": time.strftime("%Y-%m-%d")}

    if args.record:
        baseline["_comment"] = [
            "Mutation scores as measured by mutate.py, one entry per module, with the exact file set and limit used.",
            "Stage 2 of the maturity plan records these numbers without setting a target: a threshold chosen before",
            "the first measurement is a number picked to be met. The gate refuses a drop of more than",
            f"{MAX_DROP_POINTS:.0f} points and refuses a module with no entry at all.",
            "`survivors` is the work list: each one is either a blind spot in the tests or an equivalent mutant, and",
            "only a human can say which -- a survivor classified as equivalent is not deleted, it is explained.",
        ]
        BASELINE.write_text(json.dumps(baseline, indent=2) + "\n", encoding="utf-8")
        print(f"[mutate] recorded into {BASELINE.relative_to(ROOT)}")

    if problems:
        for problem in problems:
            print(f"[mutate] {problem}", file=sys.stderr)
        print(f"[mutate] FAIL ({len(problems)} problem(s))")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
