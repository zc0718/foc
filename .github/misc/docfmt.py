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

    Returns `(unit, space_before)`. The source's own word boundaries are kept -- a token is
    whatever sat between two spaces, and the space goes in front of that token and nowhere else.
    Splitting on `isspace()` and then forgetting which side of the space we came from is exactly
    how `任一 C API` loses a space and starts reading `任一C API`.
    返回 `(单元, 前置空格)`。**保留原文的词边界**：一个 token 就是两个空格之间的东西，
    空格只加在这个 token 之前。先按空白切分、之后忘掉自己在空格的哪一边，正是
    `任一 C API` 丢掉空格、变成 `任一C API` 的原因。
    """
    units: list[tuple[str, bool]] = []
    first_token = True
    for token in text.split():
        space = not first_token
        first_token = False
        index = 0
        length = len(token)
        while index < length:
            if ord(token[index]) > 0x2E7F:
                units.append((token[index], space))
                space = False
                index += 1
                continue
            stop = index
            while stop < length and not token[stop].isspace() and ord(token[stop]) <= 0x2E7F:
                stop += 1
            units.append((token[index:stop], space))
            space = False
            index = stop
    return units


def _wrap(text: str, indent: int) -> list[str]:
    """Greedy wrap at MAX_COLUMN display columns.

    Two invariants. A unit is never split: an unbreakable token broken at an arbitrary point is
    worse than a long line. And a closing punctuation mark never starts a line -- when one would,
    the unit in front of it moves down *with* it rather than the last character alone, because
    pulling one character down cuts numbers and words in half (`±20 %` -> `±2` / `0%`).
    两条不变量：单元绝不切开（把一个不可断的 token 随便切开，比一行太长更糟）；收尾标点绝不
    出现在行首——需要时把它**前面整个单元**一起下移，而不是只拽下最后一个字符，因为只拽一个
    字符会把数字和单词拦腰截断（`±20 %` → `±2` / `0%`）。
    """
    lines: list[str] = []
    current = ""
    width = indent
    last_piece = ""
    for unit, space in _units(text):
        piece = (" " if (space and current) else "") + unit
        add = _columns(piece)
        if current and (width + add) > MAX_COLUMN:
            if len(unit) == 1 and unit[0] in NO_LINE_START and last_piece:
                remainder = current[: len(current) - len(last_piece)]
                moved = last_piece.lstrip() + (" " if space else "") + unit
                if remainder.strip():
                    lines.append(remainder)
                current = moved
                width = indent + _columns(moved)
                last_piece = moved
                continue
            lines.append(current)
            current = unit
            width = indent + _columns(unit)
            last_piece = unit
        else:
            current = (current + piece) if current else unit
            width += add
            last_piece = piece
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


def _prose_characters(text: str) -> dict[str, int]:
    """Characters that carry meaning: no whitespace, no stars, no `@field` names, no tags.

    Line-stars and field names are the tool's own markup, and repeating a field once per language
    group is the intended shape, so both are excluded. Every letter, digit, punctuation mark,
    backtick and CJK character must survive.
    有意义的字符：不含空白、星号、`@field` 名与语言标签。星号和字段名是本工具自己的标记，
    而"每个语言组重复一次字段名"正是要的形状，所以两者都排除；其余的字母、数字、标点、
    反引号与中文都必须原样保留。
    """
    clean = FIELD_IN_PROSE.sub(" ", text)
    counts: dict[str, int] = {}
    for char in clean:
        if char.isspace() or char == "*":
            continue
        counts[char] = counts.get(char, 0) + 1
    return counts


def _prose_drift(before: str, after: str) -> dict[str, int]:
    """Characters present before the rewrite and missing after it."""
    old, new = _prose_characters(before), _prose_characters(after)
    return {char: n - new.get(char, 0) for char, n in old.items() if n > new.get(char, 0)}


# ------------------------------------------------------------------------------------------------
# Language groups.
#
# The template this project follows (`fcpp`, `include/doctest.hpp`) writes one tag line per language
# group and leaves the body paragraphs untagged:
#
#     * @brief [en] one-liner
#     *
#     *     a second English paragraph, no tag
#     *
#     * @brief [zh] 一句话
#     *
#     *     第二段中文，无标记
#
# Three shapes break that, and every one of them was present here:
#   1. a body line that *starts* with a tag -- the doc pipeline then assigns that line to the wrong
#      language group (English docs show Chinese, Chinese docs lose the text outright),
#   2. a single line carrying both tags, which cannot be split by language at all,
#   3. the same field tagged twice in one language, so a `@brief` ends up with two `[en]` groups.
# The repair is to regroup: per field, all paragraphs of one language become one group, one tag line,
# bodies untagged.
# ------------------------------------------------------------------------------------------------
TAG = re.compile(r"^\[(?P<lang>en|zh)\]\s*(?P<rest>.*)$")
FIELD = re.compile(r"^(?P<field>@\w+)(?:\s+(?P<args>[A-Za-z_]\w*))?(?P<tail>.*)$")
# Only these fields own a name argument. Everything else (`@brief bench_entry.cpp`, `@par 修改日志:`)
# keeps its whole tail as text, which is why the argument pattern cannot be applied to every field.
# 只有这些字段带"名字"参数；其余字段（`@brief bench_entry.cpp`）整条尾巴都是文字。
FIELDS_WITH_ARGS = {"@param", "@tparam", "@retval", "@throws", "@exception", "@sa"}
INLINE_TAGS = re.compile(r"\[(?:en|zh)\]")
# A body line that opens a list item. It starts a paragraph of its own.
# 以列表项开头的正文行：它自己起一段。
LIST_ITEM = re.compile(r"^(?:\d+[.)]|\u2022)\s")
# A dash is far more often a formula fragment (`- R` the loop is ...`) or emphasis than a bullet.
# It only counts as a list item when the paragraph it follows ends with a colon, which is what
# introduces a list.
# 短横线在本文档里更常是公式片段或强调，而不是列表项。只有上段以冒号结尾（列表的引子）时，
# 才把它当列表项。
BULLET_AFTER_COLON = re.compile(r"^[-*+]\s")
# An ASCII rule or banner (`=====`, `-----`): its own line, never joined with what follows.
# ASCII 分隔/横幅线（`=====`、`-----`）：独占一行，绝不与后文相连。
RULE_LINE = re.compile(r"^[=\-_*~]{4,}$")
# Markup the tool is allowed to add or remove: the language tags, the field names, and the inline
# doc markers it may promote out of existence. Everything else is prose.
# 工具允许增删的标记：语言标签、字段名、以及可能被提升掉的尾部注释标记。其余都是正文。
FIELD_IN_PROSE = re.compile(r"///<|/\*\*<|\[(?:en|zh)\]|@\w+(?:\s+[A-Za-z_]\w*)?")


def _split_inline(body: str) -> list[tuple[str | None, str]]:
    """`'a [en] b [zh] c'` -> [(lang, text)] with `None` for the leading untagged part."""
    marks = list(re.finditer(r"\[(en|zh)\]", body))
    if not marks:
        return [(None, body)]
    out: list[tuple[str | None, str]] = []
    lead = body[: marks[0].start()].strip()
    if lead:
        out.append((None, lead))
    for index, mark in enumerate(marks):
        end = marks[index + 1].start() if index + 1 < len(marks) else len(body)
        out.append((mark.group(1), body[mark.end() : end].strip()))
    return out


def _body_line(star_spaces: str, indent: int, text: str) -> str:
    """A continuation line: the star stays, the text keeps its hanging column."""
    return f"{star_spaces}*{' ' * max(indent - len(star_spaces) - 1, 0)}{text}".rstrip()


class _Entry:
    """One field's content in one language: a list of paragraphs."""

    def __init__(self, field: str, args: str, lang: str | None) -> None:
        self.field = field
        self.args = args
        self.lang = lang
        self.paragraphs: list[list[str]] = [[]]

    def add(self, text: str) -> None:
        text = text.strip()
        if text:
            self.paragraphs[-1].append(text)

    def new_paragraph(self) -> None:
        if self.paragraphs[-1]:
            self.paragraphs.append([])


def _canonical_language(lines: list[str]) -> list[str]:
    """Regroup a Doxygen block body so each field has one group per language."""
    for raw in lines:
        plain = LINE.match(raw)
        text = plain.group("body") if plain is not None else raw
        # A block that carries a code listing is left alone: joining code lines into paragraphs
        # would rewrite the listing, and a listing is the one thing a doc block cannot lose.
        # 带代码清单的块一律不动：把代码行并成段落等于改写清单，而清单是文档块唯一不能丢的东西。
        if "```" in text or "@code" in text or "@dot" in text or "<table" in text:
            return lines
    star_spaces = ""
    for raw in lines:
        match = LINE.match(raw)
        if match is not None and match.group("body").strip():
            star_spaces = match.group("indent")
            break

    order: list[tuple[str, str, str | None]] = []
    entries: dict[tuple[str, str, str | None], _Entry] = {}
    current: _Entry | None = None
    blank_pending = False

    def entry_for(field: str, args: str, lang: str | None) -> _Entry:
        key = (field, args, lang)
        if key not in entries:
            entries[key] = _Entry(field, args, lang)
            order.append(key)
        return entries[key]

    for raw in lines:
        match = LINE.match(raw)
        if match is None:
            continue
        body = match.group("body").strip()
        if not body:
            blank_pending = True
            if current is not None:
                current.new_paragraph()
            continue

        field = FIELD.match(body)
        if field and field.group("field") not in {"@en", "@zh"}:
            name = field.group("field")
            args = (field.group("args") or "") if name in FIELDS_WITH_ARGS else ""
            tail = field.group("tail").strip()
            pieces = _split_inline(tail)
            langs = [lang for lang, text in pieces if lang is not None and text]
            if not langs:
                # An untagged field (`@since`, `@context`, `@exporter`): keep it in place, whole.
                current = entry_for(name, args, None)
                current.add(tail)
            else:
                lead = " ".join(text for lang, text in pieces if lang is None).strip()
                first = True
                for lang, text in pieces:
                    if lang is None:
                        continue
                    current = entry_for(name, args, lang)
                    if first and lead:
                        # `@param[in] freq_hz [en] ...`: the bracketed qualifier precedes the first
                        # tag, so it joins that group's text instead of vanishing.
                        # `@param[in] freq_hz [en] ...`：方括号限定词在首个标签之前，归入该组。
                        current.add(lead)
                        first = False
                    current.add(text)
            blank_pending = False
            continue

        pieces = _split_inline(body)
        tagged = [(lang, text) for lang, text in pieces if lang is not None]
        if tagged:
            # A body line that carries a tag is a language switch: repeat the field.
            field_name = current.field if current is not None else ""
            field_args = current.args if current is not None else ""
            for lang, text in pieces:
                if lang is None:
                    # Text before the tag on this line continues whatever language is already
                    # open. This is the common case -- a sentence wrapped across a line boundary
                    # with the `[zh]` tag sitting at the end of it -- and skipping it truncates
                    # the sentence in the rendered docs.
                    # 标签之前的文字属于**已经打开**的那个语言组。这是最常见的情形——一句话折行、
                    # `[zh]` 落在行尾——跳过它就是让渲染出来的文档少半句话。
                    if current is not None and text:
                        current.add(text)
                    continue
                target = entry_for(field_name, field_args, lang)
                if blank_pending and target is not current:
                    target.new_paragraph()
                current = target
                current.add(text)
            blank_pending = False
            continue

        if current is None:
            # Body text that no field introduced stays body text. Giving it a field name would be
            # this script writing documentation on the author's behalf.
            # 没有字段引入的正文就还是正文。给它安一个字段名，等于这个脚本替作者写文档。
            current = entry_for("", "", None)
        previous = " ".join(current.paragraphs[-1]) if current.paragraphs[-1] else ""
        starts_item = bool(LIST_ITEM.match(body)) or (
            bool(BULLET_AFTER_COLON.match(body)) and previous.rstrip().endswith(":"))
        if (starts_item or RULE_LINE.match(body)) and current.paragraphs[-1]:
            # Lists are structure, not prose: each item keeps its own line.
            # 列表是结构而非散文：每一项各自成行。
            current.new_paragraph()
        current.add(body)
        blank_pending = False

    out: list[str] = []
    previous_had_bodies = False
    first_group = True
    for key in order:
        entry = entries[key]
        paragraphs = [p for p in entry.paragraphs if p]
        joined = [(" ".join(p)).strip() for p in paragraphs]
        if entry.lang is None:
            # An untagged entry: `@since 0.1.0`, `@exporter`, or plain body text with no field.
            # Every paragraph is emitted -- emitting only the first one is how a whole block of
            # prose disappears without a trace.
            # 无语言组的条目：`@since 0.1.0`、`@exporter`，或者没有字段的纯正文。
            # 每一段都要发出来——只发第一段，正是一整块正文无声消失的方式。
            head = f"{entry.field}{(' ' + entry.args) if entry.args else ''}".strip()
            block_indent = len(star_spaces) + 6
            if joined:
                first_line = f"{head} {joined[0]}".strip()
                out.append(f"{star_spaces}* {first_line}".rstrip())
            elif head:
                out.append(f"{star_spaces}* {head}".rstrip())
            for paragraph in joined[1:]:
                out.append(f"{star_spaces}*")
                for piece in _wrap(paragraph, block_indent):
                    out.append(_body_line(star_spaces, block_indent, piece))
            previous_had_bodies = len(joined) > 1
            first_group = False
            continue
        prefix = f"{star_spaces}* {entry.field}{(' ' + entry.args) if entry.args else ''} [{entry.lang}] "
        # Two columns, as in the template this follows. A wrapped continuation of the tag line aligns
        # under the field's own text; a *separate* body paragraph gets a fixed block indent five
        # columns in from the star, which is what makes a long body read as a block rather than as a
        # hanging line.
        # 两个列，与所遵循的模板一致：标签行的折行续行对齐到字段自身的文字之下；而**独立**的正文
        # 段落用星号后固定五格的块缩进——那正是让长正文读起来像一块而不是悬挂行的原因。
        indent = len(prefix)
        body_indent = len(star_spaces) + 6
        has_bodies = len(joined) > 1
        if not first_group and (previous_had_bodies or has_bodies):
            out.append(f"{star_spaces}*")
        head_lines = _wrap(joined[0], indent) if joined else [""]
        out.append((prefix + head_lines[0]).rstrip())
        for extra in head_lines[1:]:
            out.append(_body_line(star_spaces, indent, extra))
        for paragraph in joined[1:]:
            out.append(f"{star_spaces}*")
            for piece in _wrap(paragraph, body_indent):
                out.append(_body_line(star_spaces, body_indent, piece))
        previous_had_bodies = has_bodies
        first_group = False
    return out


# ------------------------------------------------------------------------------------------------
# Inline docs.
#
# A trailing `///<` may say what a field is -- unit, sign, range. Once it needs a second line it has
# become a paragraph, and a paragraph in the trailing position is where rationale accumulates and
# where it reads worst:
#
#     float probe_v[6];  ///< [en] the d-axis voltages to sweep, V; six or more,
#                        ///< because the `Rs` fit needs them. [zh] ...
#
# The repair is to promote the whole doc into a block above the declaration, in the same shape as
# every other doc block here -- which is also where a reader looks before deciding to use the type.
# ------------------------------------------------------------------------------------------------
INLINE_DOC = re.compile(r"^(?P<decl>.*?)\s*///<(?P<doc>.*)$")
DECL_END = (";", "{", "}", ":")


def _inline_group(lines: list[str], start: int) -> tuple[list[str], int]:
    """The doc text of the `///<` group that begins on `start`, and the line after it."""
    match = INLINE_DOC.match(lines[start])
    pieces = [match.group("doc")]
    index = start + 1
    while index < len(lines):
        following = INLINE_DOC.match(lines[index])
        if following is None or following.group("decl").strip():
            break
        pieces.append(following.group("doc"))
        index += 1
    return pieces, index


def _declaration_start(lines: list[str], end: int) -> int:
    """Walk back from `end` to the first line of the declaration it terminates."""
    index = end
    while index > 0:
        previous = lines[index - 1].strip()
        if not previous or previous.endswith(DECL_END) or "///" in previous or "/*" in previous:
            break
        index -= 1
    return index


def _promote_inline_docs(lines: list[str]) -> list[str]:
    """Move every multi-line `///<` doc to a block above its declaration."""
    out: list[str] = []
    index = 0
    while index < len(lines):
        match = INLINE_DOC.match(lines[index])
        if match is None or not match.group("decl").strip():
            out.append(lines[index])
            index += 1
            continue
        pieces, after = _inline_group(lines, index)
        if after == index + 1:
            # One line: a field description, which is what `///<` is for.
            out.append(lines[index])
            index += 1
            continue
        first = _declaration_start(lines, index)
        indent = lines[first][: len(lines[first]) - len(lines[first].lstrip())]
        declaration = " ".join(
            part.strip() for part in list(lines[first:index]) + [match.group("decl")]
        )
        text = _wrap_inline_text(pieces)
        out.extend(_block_from_text(indent, text))
        out.append(f"{indent}{declaration}")
        index = after
    return out


def _wrap_inline_text(pieces: list[str]) -> list[str]:
    """Rebuild the doc text as `<star> @brief [en] ...` style lines for the canonicaliser."""
    joined = " ".join(piece.strip() for piece in pieces if piece.strip())
    segments = _split_inline(joined)
    tagged = [(lang, text) for lang, text in segments if lang is not None and text]
    lead = " ".join(text for lang, text in segments if lang is None and text).strip()
    lines: list[str] = []
    if lead and not tagged:
        lines.append(f" * @brief {lead}")
    for lang, text in tagged:
        lines.append(f" * @brief [{lang}] {text}")
    return lines


def _block_from_text(indent: str, body: list[str]) -> list[str]:
    """Wrap `<indent>/**`, the given star-lines, and the closer."""
    out = [f"{indent}/**"]
    out.extend(f"{indent}{line}" for line in body)
    out.append(f"{indent} */")
    return out


def normalise(text: str) -> str:
    # Promotions come first: a multi-line `///<` becomes a block, and then it is just another block
    # for the loop below to align and wrap.
    # 先做提升：多行 `///<` 变成块，于是它只是下面那个循环里的又一个块。
    lines = _promote_inline_docs(text.split("\n"))
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
        out.extend(_canonical_language(_fix_block(lines, index + 1, closer)))
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
    corrupt: list[tuple[Path, dict[str, int]]] = []
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
            if updated == original:
                continue
            drift = _prose_drift(original, updated)
            if drift:
                # Refuse to write. A file whose prose changed is a bug in this tool, not a
                # formatting difference, and writing it would bury the evidence in a 99-file diff.
                # 拒绝写入。改动了文字的文件是本工具的 bug，不是排版差异；写下去只会把证据
                # 埋进一个 99 文件的 diff 里。
                corrupt.append((path, drift))
                continue
            changed.append(path)
            if args.fix:
                path.write_text(updated, encoding="utf-8")

    if corrupt:
        for path, drift in corrupt:
            print(f"[docfmt] REFUSED {path.relative_to(ROOT)}: prose characters would change {drift}")
        print(f"[docfmt] FAIL {len(corrupt)} file(s) would lose text; nothing was written for them")
        return 2

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
