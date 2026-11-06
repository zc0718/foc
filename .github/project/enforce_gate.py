#!/usr/bin/env python3
"""The check families the pinned clang-tidy config does not enable, enforced without editing it.

Why this exists
---------------
`rules.tsv` recorded the CERT C++ family as `open` for one reason: the enablement point is
`.github/misc/.clang-tidy`, and that file is byte-pinned by the `scaffold` gate, so a local edit would make the next
upstream release unmergeable. The family sat there as a known gap for weeks, with the reason "we cannot turn it on"
written in a table.

That reason was wrong. `clang-tidy --checks` overrides the config's `Checks` key while the config's `CheckOptions`
and `HeaderFilterRegex` still apply, so the pinned file can stay byte-identical and the family can still be
enforced -- by this repository's own gate, which is exactly where `subset`, `symbols` and `equations` live. The
measurement that made it safe is in `enforce.json`: three sites, all `cert-dcl59-cpp`, all fixed.

What it checks
--------------
1. The checks string is **composed from the pinned config**, not retyped: the `Checks:` block is parsed out of
   `.github/misc/.clang-tidy` and the extra families are appended. A retyped list would drift from upstream silently,
   and the failure would look like "the family is enabled" when it is not.
2. Every family in `enforce.json` carries a `why`; a family nobody justified is a family that will be deleted by the
   first person who finds it inconvenient.
3. Every target compiles and produces **no diagnostic at all** -- the extra families are added to the pinned set, not
   substituted for it, so this is the pinned gate plus more rather than instead of.
4. `pending` entries need a `why` too. A family that is not enabled yet is a decision, and an unexplained decision is
   indistinguishable from an oversight.

Two measurement traps this gate is built around
-----------------------------------------------
**clang-tidy writes diagnostics to stdout on the pinned version**, not stderr. Reading stderr reports a clean sweep
over everything, which is how the first two counts of these families came back as "0 hits" and were wrong.
**A sweep with no positive control is not a measurement.** `--selftest` therefore builds a file that must be diagnosed
and requires the gate to see it, then removes the family and requires the same file to be clean -- so the selftest
fails if the family is not actually switched on.

Usage
-----
    python3 .github/project/enforce_gate.py            # check
    python3 .github/project/enforce_gate.py --list     # print the composed checks string and every problem
    python3 .github/project/enforce_gate.py --selftest # prove the family is really on
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
TABLE = Path(__file__).resolve().parent / "enforce.json"
PINNED = ROOT / ".github" / "misc" / ".clang-tidy"
DEVIATION_DIR = ROOT / ".github" / "project" / "compliance" / "deviation"
SCAN_DIRS = ("include", "src")
SUFFIXES = {".h", ".hpp", ".c", ".cpp"}
SKIP_PARTS = {"build", "CMakeFiles", ".git"}


def pinned_checks() -> list[str]:
    """The `Checks:` block of the pinned config, as check patterns.

    Parsed rather than retyped: a copy of upstream's list is a copy that stops matching the day upstream changes, and
    the resulting failure would read as "the extra family is on" while the pinned set had quietly shrunk.
    解析而不是抄写：抄一份上游清单，上游一改就不再匹配，而失败会读成"额外的族开着"，实际是钉住的集合悄悄少了。
    """
    lines = PINNED.read_text(encoding="utf-8").splitlines()
    out: list[str] = []
    inside = False
    for line in lines:
        if line.startswith("Checks:"):
            inside = True
            continue
        if inside and line and not line[0].isspace():
            break
        if not inside:
            continue
        text = line.split("#", 1)[0].strip()
        out.extend(part.strip() for part in text.replace("\n", " ").split(",") if part.strip())
    return out


def extra_families(table: dict) -> list[str]:
    return [entry["checks"] for entry in table.get("families", []) if entry.get("checks")]


def family_matcher(patterns: list[str]) -> re.Pattern[str]:
    """Match a diagnostic's `[check-name]` against the extra patterns only.

    The composed run enables the pinned families too, and this gate would then also report their diagnostics -- on a
    wider target set than `tidy` uses (header-standalone units). That is not enforcement, it is a second, stricter
    copy of another gate, and the first header it swept produced one: `cppcoreguidelines-avoid-magic-numbers` on an
    array bound (`orders[6]`) in `foc_types.h`, invisible to `tidy` because `tidy` only compiles `src/` units. That
    gap is real and worth fixing, but it is `tidy`'s gap and it must be fixed there, not absorbed here by a gate that
    was not asked to. So: this gate reports only what it was created to enforce.
    组合运行也开了钉住的那些族，于是本门会连它们的诊断一起报——而目标集比 `tidy` 更宽（头文件单独成 TU）。那不是强制，
    那是另一个门的一份更严的拷贝；它扫到的第一个头就冒出一条：`foc_types.h` 里数组边界 `orders[6]` 上的
    `cppcoreguidelines-avoid-magic-numbers`，`tidy` 看不见是因为 `tidy` 只编 `src/` 的 TU。那个缺口是真的、该修，但它是
    `tidy` 的缺口，得在那边修，不能在这里被一个没被要求做这件事的门默默吸收。所以：本门只报它被创建来强制的那部分。
    """
    names = []
    for pattern in patterns:
        # A family may name several patterns, because "one family, one check" is not how clang-tidy groups
        # things: `misc-*` minus one noisy member and "three readability checks worth having" are both
        # single decisions, and recording them as one entry keeps the reason next to the whole of it.
        # 一个族可以点名多个模式，因为 clang-tidy 的分组不是「一族一检查」：`misc-*` 去掉一个噪声成员、
        # 「值得要的三条 readability」，各自都是一个决定；写成一条记录，理由才跟着整件事走。
        for part in pattern.split(","):
            part = part.strip()
            # A `-glob` removes checks from the set; it diagnoses nothing, so it is not a thing this gate
            # can report. Skipping it is what makes "enable the family, minus one member" expressible.
            # `-glob` 是从集合里移除，不会产生诊断，本门无从报它。跳过它，「开整族、去掉一个成员」才写得出来。
            if not part or part.startswith("-"):
                continue
            names.append(re.escape(part[:-1]) if part.endswith("*") else re.escape(part))
    # Alternation, not a character class: `[cert-]` would match any single character of `cert-`, so every
    # `bugprone-` and `cppcoreguidelines-` diagnostic contains an `e` and would be reported by this gate.
    # 用择一而不是字符类：`[cert-]` 会匹配 `cert-` 里任意一个单字符，于是每条 `bugprone-` /
    # `cppcoreguidelines-` 诊断都因为含一个 `e` 而被本门报出来。
    return re.compile("|".join(f"(?:{name})" for name in names))


def excepted_checks(table: dict) -> list[str]:
    """Checks a family enables but does not report here, each with a signed deviation record.

    A family glob is coarse: `bugprone-*` is 41 checks and one of them asks every function with two
    adjacent floats to restructure its signature, which is the shape of this library's whole API.
    Turning the family off to avoid that one would trade 40 useful checks for tidiness; turning it
    on and ignoring the noise would trade the gate for a habit. So the family runs, the one check is
    named here, and `evaluate()` refuses unless a `deviation/<name>.md` exists for it -- the same
    rule the tree applies to `NOLINT`.
    """
    names = []
    for entry in table.get("families", []):
        names.extend(entry.get("except", []))
    return names


def is_excepted(line: str, excepted: list[str]) -> bool:
    match = re.search(r"\[([^\]]+)\]\s*$", line.strip())
    if match is None:
        return False
    tags = {tag.strip() for tag in match.group(1).split(",")}
    return any(name in tags for name in excepted)


def compose(table: dict) -> str:
    return ",".join([*pinned_checks(), *extra_families(table)])


def targets() -> list[Path]:
    found = []
    for name in SCAN_DIRS:
        for path in (ROOT / name).rglob("*"):
            if path.suffix in SUFFIXES and path.is_file() and not (SKIP_PARTS & set(path.parts)):
                found.append(path)
    return sorted(found)


def run_one(checks: str, work: Path, target: Path) -> list[str]:
    """Diagnostics for one target, read from **stdout** -- see the module docstring.

    A `.h` header here is a **C** header (`C_SUFFIXES` in `.github/misc/gates.py` is `('.c', '.h')`), and this
    tree has at least one that must compile as *both* languages: `foc_types.h` is included by the C++ core and by
    the C ABI, and its own deviation record says so. So a `.h` is compiled twice, once as each language, and the
    two diagnostic lists are concatenated. Treating every `.h` as C++ made `src/api/bridge.h` fail to compile, and
    because the family matcher only reports `cert-*`/`bugprone-*`, that failure was filtered out -- the gate
    reported a clean sweep over 124 targets while quietly skipping this one. A target that does not compile is now
    a failure in its own right, not an absence of findings.
    """
    if target.suffix not in (".h", ".hpp"):
        return _tidy(checks, work, target, "-std=c11" if target.suffix == ".c" else "-std=c++17",
                     [] if target.suffix == ".c" else ["-fno-exceptions", "-fno-rtti"])
    # Absolute, because a quoted include is searched next to the synthetic TU first: a private header
    # (`src/api/bridge.h`) is not on `-Iinclude` and would not be found by its relative path.
    lines: list[str] = []
    # **By language, not by directory**: `.c`/`.h` are the C family and are seen only as C; `.cpp`/`.hpp` only as
    # C++. That is the infrastructure layer's own rule (`C_SUFFIXES = ('.c', '.h')`), and it is also why a C header
    # must never be handed to the C++ view: measured on `foc_api.h`, the C++ view adds 17 diagnostics of exactly the
    # kind the project's scoping decision exists to avoid (`use-enum-class` 10, `avoid-c-arrays` 7 -- neither
    # construct exists in C11), and 0 in the families this gate enforces.
    # 按语言而不是按目录：`.c`/`.h` 只按 C 看，`.cpp`/`.hpp` 只按 C++ 看。这也是「C 头绝不交给 C++ 视图」的原因：
    # `foc_api.h` 的 C++ 视图多出 17 条诊断（`use-enum-class` 10、`avoid-c-arrays` 7——C11 里都不存在），
    # 而本门强制的两族一条都没多。
    languages = ((".c", "-std=c11", []), (".cpp", "-std=c++17", ["-fno-exceptions", "-fno-rtti"]))
    for suffix, std, cxx in (languages[:1] if target.suffix == ".h" else languages[1:]):
        tu = work / f"{target.stem}_{target.parent.name}{suffix}"
        tu.write_text(f'#include "{target.resolve().as_posix()}"\n', encoding="utf-8")
        lines.extend(_tidy(checks, work, tu, std, cxx))
    return lines


def _tidy(checks: str, work: Path, tu: Path, std: str, cxx: list[str]) -> list[str]:
    del work
    cmd = ["clang-tidy", f"--config-file={PINNED.relative_to(ROOT).as_posix()}", f"--checks={checks}",
           str(tu), "--", std, *cxx, "-Iinclude"]
    done = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    return [line for line in done.stdout.splitlines() if "warning:" in line or "error:" in line]


def evaluate(table: dict) -> list[str]:
    problems: list[str] = []
    if not PINNED.is_file():
        return [f"{PINNED.name} is missing; the pinned check set cannot be read"]
    base = pinned_checks()
    if not base:
        problems.append("the pinned config yields no checks, so composing from it would enforce nothing")
    if base[:1] != ["-*"]:
        problems.append("the pinned config does not start with '-*'; the composed set would add to whatever "
                        "clang-tidy enables by default instead of to a known-empty base")
    seen: set[str] = set()
    for entry in table.get("families", []):
        name = entry.get("id", "?")
        if name in seen:
            problems.append(f"{name}: declared twice")
        seen.add(name)
        if not entry.get("checks"):
            problems.append(f"{name}: has no `checks` pattern")
        if not (entry.get("why") or {}).get("en"):
            problems.append(f"{name}: enabled without a reason — that is how a family gets deleted by the next "
                            f"person who finds it inconvenient")
        for excepted in entry.get("except", []):
            covered = any(excepted.startswith(part[:-1]) if part.endswith("*") else excepted == part
                          for part in (p.strip() for p in entry.get("checks", "").split(","))
                          if part and not part.startswith("-"))
            if not covered:
                problems.append(f"{name}: exempts '{excepted}', which its own pattern '{pattern}' does not enable — "
                                f"an exemption for a check nobody runs is a note to nobody")
            record = DEVIATION_DIR / f"{excepted}.md"
            if not record.is_file():
                problems.append(f"{name}: exempts '{excepted}' with no {record.relative_to(ROOT).as_posix()} — a "
                                f"suppression without a record is not a deviation, it is a habit")
    for entry in table.get("pending", []):
        if not (entry.get("why") or {}).get("en"):
            problems.append(f"pending {entry.get('id', '?')}: no reason, which is indistinguishable from an oversight")
    problems.extend(relaxed_problems(table))
    return problems


def relaxed_problems(table: dict) -> list[str]:
    """The checks this project looked at and decided **not** to run, each with a reason and a count.

    Turning a check off is a decision with a price -- the defect it would have caught is not caught -- and a
    decision written only in a commit message is gone by the third refactor. So it lives here: what was
    relaxed, why its advice cannot indicate a defect in this domain, and the number that was measured before
    the decision. The `evidence` field is required for the same reason a deviation record is: a reason without
    a measurement is a preference, and the next person is entitled to overrule a preference silently.
    关掉一条检查是有代价的决定——它本该抓到的缺陷就没人抓了——只写在提交信息里的决定活不过第三次重构。
    所以它落在这里：关了哪条、为什么它的建议在本域内指不出缺陷、做决定前量到的数。`evidence` 是必填，
    理由和偏差单一样：没有测量的理由只是偏好，而偏好下一个人有权悄悄推翻。
    """
    problems: list[str] = []
    enabled = {part.strip() for entry in table.get("families", [])
               for part in entry.get("checks", "").split(",") if part.strip()}
    for entry in table.get("relaxed", []):
        checks = entry.get("checks", "")
        if not checks:
            problems.append("relaxed: an entry with no `checks` says nothing about what was turned off")
        if not (entry.get("why") or {}).get("en"):
            problems.append(f"relaxed {checks or '?'}: turned off without a reason — an omission is not a "
                            f"decision, and the next reader cannot tell them apart")
        if not entry.get("evidence"):
            problems.append(f"relaxed {checks or '?'}: no `evidence` — the count that justified the decision is "
                            f"the only thing that can reopen it")
        if checks in enabled:
            problems.append(f"relaxed {checks}: also enabled by a family — a check cannot be on and off")
        for family in table.get("families", []):
            if family.get("checks") == checks:
                problems.append(f"relaxed {checks}: identical to family '{family.get('id')}'")
    return problems


def probe(checks: str, work: Path) -> list[str]:
    """One header that must be diagnosed by the extra family: an unnamed namespace in a header.

    `--header-filter=.*` is passed here and only here: the probe lives outside the repository, so the config's
    `HeaderFilterRegex` (anchored to `^(include|src)/`) would hide the very diagnostic being looked for. This makes
    the probe answer "is the family enabled", which is the question; the real sweep keeps the config's filter.
    这里、且只有这里传 `--header-filter=.*`：探针住在仓库外，配置里的 `HeaderFilterRegex`（锚在 `^(include|src)/`）
    恰好会把要找的那条诊断滤掉。这样探针回答的是"族开着吗"——那才是问题；正式扫描仍用配置的过滤器。
    """
    probe_dir = work / "probe"
    probe_dir.mkdir(exist_ok=True)
    (probe_dir / "bad.hpp").write_text("namespace {\ninline auto f() noexcept -> int { return 1; }\n} // namespace\n",
                                       encoding="utf-8")
    tu = probe_dir / "use.cpp"
    tu.write_text('#include "bad.hpp"\n\nint main() { return f(); }\n', encoding="utf-8")
    cmd = ["clang-tidy", f"--config-file={PINNED.relative_to(ROOT).as_posix()}", f"--checks={checks}",
           "--header-filter=.*", str(tu), "--", "-std=c++17", f"-I{probe_dir}"]
    done = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
    return [line for line in done.stdout.splitlines() if "warning:" in line or "error:" in line]


def selftest(table: dict) -> int:
    """Two runs, opposite verdicts: the family must be on, and the probe must be what proves it."""
    full = compose(table)
    stripped = ",".join(pinned_checks())
    with tempfile.TemporaryDirectory(prefix="enforce-selftest-") as work:
        work_path = Path(work)
        on = probe(full, work_path)
        off = probe(stripped, work_path)
    caught = 0
    trials = [
        ("an unnamed namespace in a header is diagnosed by the extra family", bool(on)),
        ("the same file is clean once the extra family is removed, so the probe measures the family", not off),
    ]
    for label, ok in trials:
        print(f"[enforce-selftest] {'OK  ' if ok else 'MISS'} {label}")
        caught += 1 if ok else 0
    print(f"[enforce-selftest] {caught} / {len(trials)} judgement(s) as expected")
    return 0 if caught == len(trials) else 1


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--selftest", action="store_true", help="prove the extra families are actually switched on")
    parser.add_argument("--list", action="store_true", help="print the composed checks string and every problem")
    args = parser.parse_args(argv)
    table = json.loads(TABLE.read_text(encoding="utf-8"))
    if args.selftest:
        return selftest(table)
    problems = evaluate(table)
    checks = compose(table)
    if args.list:
        print(f"[enforce] checks = {checks}")
    if problems:
        for problem in problems:
            print(f"[enforce] {problem}", file=sys.stderr)
        print(f"[enforce] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        return 1
    files = targets()
    if not files:
        print("[enforce] FAIL: no targets; the gate would report a clean sweep over nothing", file=sys.stderr)
        return 1
    families = ", ".join(extra_families(table))
    matcher = family_matcher(extra_families(table))
    excepted = excepted_checks(table)
    if excepted:
        families += " (exempt: " + ", ".join(excepted) + ")"
    failures = 0
    with tempfile.TemporaryDirectory(prefix="enforce-") as work:
        work_path = Path(work)
        for target in files:
            lines = run_one(checks, work_path, target)
            broken = [line for line in lines if "clang-diagnostic-error" in line]
            if broken:
                failures += 1
                print(f"--- {target.relative_to(ROOT).as_posix()} does not compile")
                for line in broken[:3]:
                    print(f"    {line.strip()}")
            diagnostics = [line for line in lines
                           if matcher.search(line) and not is_excepted(line, excepted)]
            if diagnostics:
                failures += 1
                print(f"--- {target.relative_to(ROOT).as_posix()}")
                for line in diagnostics:
                    print(f"    {line.strip()}")
    if failures:
        print(f"[enforce] FAIL ({failures}/{len(files)} target(s))", file=sys.stderr)
        return 1
    relaxed = len(table.get("relaxed", []))
    print(f"[enforce] OK ({len(files)} target(s), `.c`/`.h` as C and `.cpp`/`.hpp` as C++; {families} on top "
          f"of the pinned set; {relaxed} check(s) deliberately relaxed to 0, see enforce.json)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
