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

3. **Each body paragraph reaches the build of its own language.** `docs/build.py` decides an untagged
   paragraph's language by script -- CJK goes to `zh`, everything else to the language that opened the
   chunk -- so a block that writes both briefs first and both bodies after them files the English body
   under `zh`: the English build loses it and the Chinese build shows it in English. This leg runs that
   same filter over every block and asks it for both languages, so the rule it enforces is the pipeline's
   own, not a second opinion about it. Measured before the repair: 88 blocks in 38 files.

What it deliberately does not check: which fields a block carries. A type with nothing to warn about should not hold an
empty `@warning`, so "which sections exist" is a judgement rather than a rule. `docfmt` already enforces the shape the
docs pipeline depends on (one tag per language group, untagged bodies, alignment, wrapping) and `docvoice` already
enforces the register. What is left here is the three things neither of those can see.
"""

from __future__ import annotations

import argparse
import importlib.util
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

# The language leg has to classify paragraphs the way `docs/build.py` does, so its two constants are copied
# from there: a different CJK range or a different word test would disagree with the pipeline it is checking.
CJK = re.compile(r"[\u3000-\u303f\u4e00-\u9fff\uff01-\uff60]")
LANG_MARKER = re.compile(r"\[(?:en|zh|jp)\]")
LANG_TAG = re.compile(r"^\s*\*\s*@\w+.*\[(?:en|zh|jp)\]")


def _english_words(text: str) -> int:
    """Words that are prose rather than LaTeX (`\\leftarrow` is not a word)."""
    text = re.sub(r"\\f\[.*?\\f\]", " ", text, flags=re.S)
    text = re.sub(r"\\f\$.*?\\f\$", " ", text, flags=re.S)
    text = re.sub(r"\\[A-Za-z]+", " ", text)
    return len(re.findall(r"[A-Za-z]{3,}", text))


def _plain(line: str) -> str:
    text = LANG_MARKER.sub(" ", line.strip().lstrip("*"))
    # A one-line block (`/** @brief x */`) puts the opener on the same line as the tag; without
    # stripping it the line reads as prose and the leg reports its own input.
    text = re.sub(r"^\s*/\*\*?<?|\*/\s*$", " ", text)
    return " ".join(text.split())


def _prose(lines: list[str]) -> list[str]:
    return [line for line in lines if _plain(line) and not _plain(line).startswith("@")]


def _load_pipeline():
    """The docs build's own language filter, imported rather than reimplemented."""
    spec = importlib.util.spec_from_file_location("foc_docs_build", ROOT / "docs" / "build.py")
    if spec is None or spec.loader is None:  # pragma: no cover - environment failure
        raise RuntimeError("docs/build.py is not loadable; the language leg cannot judge without it")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def blocks(text: str) -> list[tuple[int, str]]:
    out = []
    for match in re.finditer(r"/\*\*.*?\*/", text, re.DOTALL):
        out.append((text.count("\n", 0, match.start()) + 1, match.group(0)))
    return out


def documented_files():
    """The files the docs pipeline stages, taken from the pipeline's own configuration."""
    meta = json.loads((ROOT / "metadata.json").read_text(encoding="utf-8"))
    suffixes = {"." + suffix for suffix in meta["doc_doxygen_suffix"]}
    for folder in meta["doc_doxygen_folders"]:
        root = ROOT / folder
        if not root.is_dir():
            continue
        for path in sorted(root.rglob("*")):
            if path.suffix in suffixes and path.is_file() and not (SKIP_PARTS & set(path.parts)):
                yield path


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


def scan_language(text: str, pipeline) -> list[str]:
    """Blocks whose English body does not reach the English build, or does reach the Chinese one.

    Only the files the pipeline actually documents are judged: the folder list is read from
    `metadata.json` (`doc_doxygen_folders`) rather than repeated here, so a tree this leg skips
    is a tree the docs do not build either.
    """
    problems: list[str] = []
    for start, block in blocks(text):
        lines = block.splitlines()
        # The compact form `/** [en] x [zh] y */` carries no tag at all, so there is no tag line for a
        # body to belong to and both builds necessarily show the whole line. `docs/build.py` names that
        # shape as supported; this leg only judges blocks that do have per-language tags.
        if not any(LANG_TAG.match(line) for line in lines):
            continue
        source = _prose(lines)
        if len(source) < 2:
            continue
        english = [line for line in source if not CJK.search(line) and _english_words(line) >= 8]
        if not english:
            continue
        en = " ".join(_plain(line) for line in _prose(pipeline._language_filter(lines, ["en", "zh"], "en")))
        zh = " ".join(_plain(line) for line in _prose(pipeline._language_filter(lines, ["en", "zh"], "zh")))
        lost = [line for line in english if _plain(line) not in en]
        leak = [line for line in english if _plain(line) in zh]
        if lost:
            problems.append(f"{start}: the English build loses {len(lost)} body line(s), e.g. "
                            f"{_plain(lost[0])[:60]!r} -- a body paragraph belongs under the brief of its "
                            f"own language")
        if leak:
            problems.append(f"{start}: {len(leak)} English body line(s) also appear in the Chinese build, "
                            f"e.g. {_plain(leak[0])[:60]!r}")
    return problems


def measure() -> dict:
    invented: list[str] = []
    language: list[str] = []
    per_file: dict[str, int] = {}
    pipeline = _load_pipeline()
    for path in files():
        rel = path.relative_to(ROOT).as_posix()
        text = path.read_text(encoding="utf-8", errors="replace")
        found, long_briefs = scan_text(text)
        invented.extend(f"{rel}:{line}" for line in found)
        if long_briefs:
            per_file[rel] = long_briefs
    for path in documented_files():
        rel = path.relative_to(ROOT).as_posix()
        text = path.read_text(encoding="utf-8", errors="replace")
        language.extend(f"{rel}:{line}" for line in scan_language(text, pipeline))
    return {"invented": invented, "language": language, "long_briefs": per_file}


def read_baseline() -> dict:
    if not BASELINE.is_file():
        return {}
    return json.loads(BASELINE.read_text(encoding="utf-8"))


def evaluate(measured: dict, baseline: dict) -> list[str]:
    problems = list(measured["invented"]) + list(measured["language"])
    frozen = baseline.get("long_briefs", {})
    for rel, count in measured["long_briefs"].items():
        ceiling = frozen.get(rel, 0)
        if count > ceiling:
            problems.append(f"{rel}: {count} brief(s) wider than {BRIEF_COLUMNS} columns, baseline {ceiling} — a "
                            f"brief that needs a second line is the body paragraph underneath it")
    return problems


def selftest() -> int:
    pipeline = _load_pipeline()
    body_en = "    The integral clamp is the anti-windup mechanism and the step never sees what the actuator did.\n"
    body_zh = "    积分钳位就是抗饱和机制，这一步从不去看执行器实际做了什么。\n"
    cases = [
        ("an invented section marker", "/**\n * @brief [en] x\n *\n *     **Equation.** e = mc^2\n */\nclass X {};\n",
         True, False),
        ("a brief that is a paragraph",
         "/**\n * @brief [en] " + ("word " * 30) + "\n */\nclass X {};\n", True, False),
        ("the house shape", "/**\n * @brief [en] vector add\n *\n *     Body paragraph, no tag.\n */\nclass X {};\n",
         False, False),
        ("both briefs first, both bodies after them",
         "/**\n * @brief [en] one line\n *\n * @brief [zh] 一行\n *\n *     " + body_en.strip()
         + "\n *\n *     " + body_zh.strip() + "\n */\nclass X {};\n", False, True),
        ("the compact one-line form",
         "/** [en] Rows in the report. Fixed so the report has no growth path to negotiate. [zh] "
         "报告的行数。固定，于是这份报告不需要为增长再谈一次。 */\nclass X {};\n", False, False),
        ("each body under its own brief",
         "/**\n * @brief [en] one line\n *\n *     " + body_en.strip()
         + "\n *\n * @brief [zh] 一行\n *\n *     " + body_zh.strip() + "\n */\nclass X {};\n", False, False),
    ]
    caught = 0
    for label, text, invented_expected, language_expected in cases:
        invented, long_briefs = scan_text(text)
        language = scan_language(text, pipeline)
        got = "invented" if (invented or long_briefs) else ("language" if language else "clean")
        want = "invented" if invented_expected else ("language" if language_expected else "clean")
        ok = got == want
        print(f"[docshape-selftest] {'OK  ' if ok else 'MISS'} {label}: {got} (expected {want})")
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
    print(f"[docshape] OK (no invented section markers; every body paragraph reaches the build of its "
          f"own language; {sum(measured['long_briefs'].values())} over-long brief(s) at or below baseline)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
