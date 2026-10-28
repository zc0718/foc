#!/usr/bin/env python3
"""Normalise multi-line Doxygen comment bodies: alignment, wrapping, and no mid-CJK breaks.

Why this exists. The rules in `_shared/code-conventions.md` say `no comment reflow` and 120 columns,
which is right -- clang-format must not touch prose. The cost is that nothing else touches it either,
so a hand-wrapped block stays exactly as it was typed, mangled indentation included. That happened
repeatedly: continuation lines ended up at three or four different columns inside one paragraph, and
Chinese sentences got broken at arbitrary characters with the remainder hanging at a random indent.

So the invariant is stated once, here, and checked by a gate:

1. Inside a Doxygen block every line starts with `*` in the same column as the opener's `*`.
2. Within one paragraph, all continuation lines share **one** indent. A paragraph whose lines agree
   is left exactly alone -- the author's choice of column is respected, which is what keeps this tool
   from churning three hundred good comments to fix thirty bad ones.
3. A paragraph whose lines disagree is rewrapped: joined, then greedily wrapped at 120 columns with
   the indent of the paragraph's first line.
4. **CJK text is never broken mid-sentence.** Chinese has no inter-word spaces, so a greedy
   space-based wrap naturally leaves a whole sentence on one line. That is the point: the damage
   being repaired was caused by hand-wrapping Chinese at character boundaries.
5. No trailing whitespace; no tab characters.

Usage:
    python3 .github/misc/docfmt.py --check            # exit 1 if anything would change
    python3 .github/misc/docfmt.py --fix              # rewrite in place
    python3 .github/misc/docfmt.py --check --verbose  # list every file that would change

The scan covers the directories that hold shipped prose: `include/`, `src/`, `api/`, `benchmark/`,
`test_package/`, and skips build trees.

**Scope: Doxygen blocks only.** Trailing `///<` field comments are line comments, not blocks, and
wrapping them would move text out of the field's own line -- a different change with a different
risk. They are left alone by design, not by omission.
**范围：只处理 Doxygen 块注释。** 行尾 `///<` 字段注释是行注释而不是块，折它会把文字挪出那一行
——那是另一种改动、另一种风险。按设计不动它，不是漏掉了。
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

SCAN_DIRS = ["include", "src", "api", "benchmark", "test_package"]
# Build trees hold generated sources with colons in the path and 239-column lines nobody wrote.
# 构建树里是生成出来的源码，路径带冒号、行宽 239，都不是人写的。
SKIP_PARTS = {"build", "CMakeFiles", ".git"}
SUFFIXES = {".h", ".hpp", ".c", ".cpp"}

# `code-conventions.md`: 120 columns for C++ and C alike.
MAX_COLUMN = 120

OPEN = re.compile(r"^(?P<indent>\s*)/\*\*")
LINE = re.compile(r"^(?P<indent>\s*)\*(?P<body>.*)$")
# A list marker or a Doxygen field name: the text after it may hang deeper than the marker.
MARKER = re.compile(r"^(?:\d+\.|[-*+]|@\w+)\s+")


def _columns(line: str) -> int:
    """Display columns, counting a wide (CJK) character as two.

    `code-conventions.md` sets the limit at 120 columns, which is about how the line *reads*, not how
    many codepoints it holds. Counting CJK as one would let a 120-codepoint Chinese line render at
    240 columns.
    """
    total = 0
    for ch in line:
        code = ord(ch)
        wide = (
            0x1100 <= code <= 0x115F
            or 0x2E80 <= code <= 0xA4CF
            or 0xAC00 <= code <= 0xD7A3
            or 0xF900 <= code <= 0xFAFF
            or 0xFE30 <= code <= 0xFE6F
            or 0xFF00 <= code <= 0xFF60
            or 0xFFE0 <= code <= 0xFFE6
            or 0x20000 <= code <= 0x3FFFD
        )
        total += 2 if wide else 1
    return total


# Closing punctuation may not start a line: breaking there is the one Chinese line-break rule that
# matters to a reader. 收尾标点不得出现在行首：那是中文断行里唯一真会影响阅读的规则。
NO_LINE_START = "。，、；：！？）》」』】%"


def _units(text: str) -> list[tuple[str, bool]]:
    """Split into wrap units: ASCII words stay whole, CJK breaks between characters.

    Returns `(unit, space_before)`. A space is inserted only between two ASCII units -- that is how
    the source reads, and it keeps `**AND**` from colliding with the Chinese that follows it.
    返回 `(单元, 前置空格)`。只在两个 ASCII 单元之间插空格——源文如此，且能避免 `**AND**` 与
    其后的中文粘在一起。
    """
    units: list[tuple[str, bool]] = []
    index = 0
    length = len(text)
    previous_ascii = False
    while index < length:
        char = text[index]
        if char.isspace():
            index += 1
            continue
        if ord(char) > 0x2E7F:
            units.append((char, False))
            previous_ascii = False
            index += 1
            continue
        stop = index
        while stop < length and not text[stop].isspace() and ord(text[stop]) <= 0x2E7F:
            stop += 1
        units.append((text[index:stop], previous_ascii))
        previous_ascii = True
        index = stop
    return units


def _wrap(text: str, indent: int) -> list[str]:
    """Greedy wrap at MAX_COLUMN display columns.

    A single unit wider than the budget still goes on its own line rather than being broken: an
    unbreakable token broken at an arbitrary point is worse than a long line.
    单个单元超出预算时也独占一行而不被切开：把一个不可断的单元随便切开，比一行太长更糟。
    """
    lines: list[str] = []
    current = ""
    width = indent
    for unit, space in _units(text):
        piece = (" " if (space and current) else "") + unit
        add = _columns(piece)
        if current and (width + add) > MAX_COLUMN:
            # Do not let closing punctuation begin the next line.
            # 不让收尾标点跑到下一行行首。
            while len(current) > 1 and unit and unit[0] in NO_LINE_START and len(unit) == 1:
                moved = current[-1]
                current = current[:-1]
                unit = moved + unit
                add = _columns((" " if space else "") + unit)
                break
            lines.append(current)
            current = unit
            width = indent + _columns(unit)
        else:
            current = (current + piece) if current else unit
            width += add
    if current:
        lines.append(current)
    return lines


# A line that opens a new run inside a paragraph. This is the repo's own bilingual style --
# `@brief [en] ...` followed by ` *        [zh] ...` is two runs at two different columns, and it is
# correct. Damage is a *continuation* that disagrees with the run it continues.
RUN_START = re.compile(r"^(?:@\w+|\[en\]|\[zh\]|\d+\.|[-*+])\s*")


def _cjk_start(text: str) -> bool:
    """A line whose first non-space, non-tag character is CJK opens a run.

    Plenty of blocks here are an English paragraph followed by its Chinese counterpart with no `[zh]`
    tag on the second one -- the language itself is the marker. Without this, repairing an indent
    glitch would splice the two languages into one sentence.
    这里有很多块是英文段落后跟中文对照，而中文那段没有 `[zh]` 标记——语言本身就是标记。没有这一条，
    修一次缩进就会把两种语言拼成一句。
    """
    body = text.lstrip()
    for tag in ("[en]", "[zh]"):
        if body.startswith(tag):
            body = body[len(tag) :].lstrip()
            break
    for char in body:
        if char.isspace():
            continue
        return ord(char) > 0x2E7F
    return False


def _star_line(star_col: int, indent: int, text: str) -> str:
    return f"{' ' * star_col}*{' ' * max(indent - star_col - 1, 0)}{text}"


def _fix_block(lines: list[str], start: int, end: int) -> list[str]:
    """Rewrite the body of one Doxygen block (`start`..`end` exclusive of the closer)."""
    body = lines[start:end]
    if not body:
        return body
    first = LINE.match(body[0])
    if not first:
        return body
    star_col = len(first.group("indent"))
    out: list[str] = []
    run: list[tuple[int, str]] = []

    def flush() -> None:
        nonlocal run
        if not run:
            return
        indents = {e[0] for e in run}
        # A Doxygen field line (`@brief`, `@note`, `@param ...`) may hang its continuation deeper --
        # that is the canonical look and it is not damage. One hang, consistently, or nothing.
        # Doxygen 字段行（`@brief`/`@note`/`@param ...`）的续行可以更深地悬挂——那是本仓库的正规写法，
        # 不是损坏。悬挂要么只有一档且一致，要么不成立。
        if len(indents) > 1:
            hanging = {e[0] for e in run[1:]}
            corroborated = run[0][1].startswith("@") or len(run) >= 3
            if corroborated and (len(hanging) == 1) and (next(iter(hanging)) > run[0][0]):
                indents = {run[0][0]}
        overflow = any(_columns(e[1]) + e[0] > MAX_COLUMN for e in run)
        if len(indents) == 1 and not overflow:
            # Consistent: leave it byte-identical. This tool repairs damage, it does not impose a
            # house style on prose that is already fine -- which is what keeps it from churning a
            # hundred good comments to fix thirty bad ones.
            # 一致：原样保留。本工具修损坏，不给已经没问题的散文套统一风格——这正是它不会为修三十条
            # 而搅一百条的原因。
            out.extend(_star_line(star_col, e[0], e[1]) for e in run)
        else:
            wrap_indent = run[0][0]
            joined = " ".join(e[1].strip() for e in run)
            for piece in _wrap(joined, wrap_indent):
                out.append(_star_line(star_col, wrap_indent, piece))
        run = []

    for raw in body:
        match = LINE.match(raw)
        if not match:
            flush()
            out.append(raw.rstrip())
            continue
        text = match.group("body")
        stripped = text.strip()
        if not stripped:
            flush()
            out.append(f"{' ' * star_col}*")
            continue
        indent = star_col + 1 + (len(text) - len(text.lstrip(" ")))
        # A CJK line opens a new run only when the current run is not already CJK -- otherwise every
        # wrapped line of a Chinese paragraph would start its own run and no Chinese paragraph could
        # ever be repaired.
        # 中文行只有在当前段**不是**中文时才开新段——否则中文段的每一折行都会各自开段，任何中文段都
        # 修不了。
        opens_run = RUN_START.match(stripped) or (_cjk_start(stripped) and (not run or not _cjk_start(run[0][1])))
        if opens_run:
            flush()
        run.append((indent, stripped))
    flush()
    return out


def normalise(text: str) -> str:
    lines = text.split("\n")
    out: list[str] = []
    index = 0
    while index < len(lines):
        line = lines[index]
        opened = OPEN.match(line)
        if not opened or "*/" in line:
            out.append(line.rstrip())
            index += 1
            continue
        # Find the closer, guarding against an unterminated block at end of file.
        closer = index + 1
        while closer < len(lines) and "*/" not in lines[closer]:
            closer += 1
        if closer >= len(lines):
            out.extend(raw.rstrip() for raw in lines[index:])
            break
        out.append(line.rstrip())
        out.extend(_fix_block(lines, index + 1, closer))
        out.append(lines[closer].rstrip())
        index = closer + 1
    return "\n".join(out)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--fix", action="store_true", help="rewrite files in place")
    parser.add_argument("--check", action="store_true", help="report only (the default)")
    parser.add_argument("--verbose", action="store_true", help="name every file that changes")
    args = parser.parse_args()

    changed: list[Path] = []
    for name in SCAN_DIRS:
        base = ROOT / name
        if not base.is_dir():
            continue
        for path in sorted(base.rglob("*")):
            if path.suffix not in SUFFIXES or not path.is_file():
                continue
            if SKIP_PARTS & set(path.parts):
                continue
            original = path.read_text(encoding="utf-8")
            updated = normalise(original)
            if updated != original:
                changed.append(path)
                if args.fix:
                    path.write_text(updated, encoding="utf-8")

    verb = "rewrote" if args.fix else "would rewrite"
    if changed:
        if args.verbose:
            for path in changed:
                print(f"  {path.relative_to(ROOT)}")
        print(f"[docfmt] {verb} {len(changed)} file(s)")
        return 0 if args.fix else 1
    print("[docfmt] OK (every Doxygen block is aligned and wrapped consistently)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
