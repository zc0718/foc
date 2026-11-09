#!/usr/bin/env python3
"""No doc block may sit inside a function body.

Why this is a gate: `EXTRACT_ALL = YES` publishes a `/** */` block wherever it is written, so a block
that documents a *statement* -- `thermal.continuous()` carried one -- is not a syntax error, not a
Doxygen warning, and not visible in review. It reaches the site as prose attached to nothing, or (in a
`.dox`) as a page-level paragraph that has lost the declaration it was meant to describe. A block
belongs at file scope (a `@file` block, an `@cond`) or inside a type body (a member). Inside a
function it is red.

The measurement is comment-aware on purpose. A regex over brace characters lies here: doc prose
carries braces (`{0, 1}`, `f(x) { ... }`), so a naive counter invents and loses scopes -- that version
reported 333 blocks inside function bodies, all phantom. This strips `//`, `/* ... */` with state
across lines, string and character literals, and treats the opening line of a new doc block as not
code, so the scope it is attributed to is the scope a human would name.

    python3 .github/project/docscope_gate.py            # the whole tree
    python3 .github/project/docscope_gate.py --selftest
"""
from __future__ import annotations

import argparse
import pathlib
import re
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]
TREES = ('include', 'src')
SUFFIXES = ('h', 'hpp', 'c', 'cpp')
TYPE_OPEN = re.compile(r'^\s*(?:template\s*<[^>]*>\s*)?(?:class|struct|union|enum|namespace)\b')
CTRL = re.compile(r'^\s*(?:if|for|while|switch|else|do|try|catch)\b')
DOC_OPEN = ('/**', '/*!')


def strip_code(line: str, in_block: bool, in_str: bool):
    """Return (code-only text, in_block, in_str) for one line."""
    out = []
    i = 0
    while i < len(line):
        ch = line[i]
        if in_block:
            if line.startswith('*/', i):
                in_block = False
                i += 2
                continue
            i += 1
            continue
        if in_str:
            if ch == '\\':
                i += 2
                continue
            if ch == '"':
                in_str = False
            i += 1
            continue
        if line.startswith('//', i):
            break
        if line.startswith('/*', i):
            in_block = True
            i += 2
            continue
        if ch == '"':
            in_str = True
            i += 1
            continue
        if ch == "'":
            # a character literal, unless it is a digit separator inside a number
            j = line.find("'", i + 1)
            if j > i:
                i = j + 1
                continue
        out.append(ch)
        i += 1
    return ''.join(out), in_block, in_str


def scopes(lines: list[str]) -> list[str]:
    """`type`, `func` or '' for each line: the innermost scope kind that owns it."""
    stack: list[str] = []
    out: list[str] = []
    in_block = in_str = False
    for line in lines:
        s = line.strip()
        starts_doc = s.startswith(DOC_OPEN)
        code, in_block, in_str = strip_code(line, in_block, in_str)
        if starts_doc:
            out.append(stack[-1] if stack else '')
            continue
        # Walk the characters in order: `{}` on one line is a closed empty body, not a pop followed by a
        # push. Counting the braces first and popping first lost the enclosing class and turned every
        # member block into a "function body" block.
        for ch in code:
            if ch == '}':
                if stack:
                    stack.pop()
            elif ch == '{':
                if TYPE_OPEN.match(code):
                    kind = 'type'
                elif ')' in code and not CTRL.match(code) and '=' not in code.split('(')[0]:
                    kind = 'func'
                else:
                    kind = stack[-1] if stack else ''
                stack.append(kind)
        out.append(stack[-1] if stack else '')
    return out


def violations(root: pathlib.Path) -> list[str]:
    """Every doc block that sits inside a function body, as `path:line -> what follows`."""
    found = []
    for tree in TREES:
        for suffix in SUFFIXES:
            for path in sorted((root / tree).rglob(f'*.{suffix}')):
                lines = path.read_text(encoding='utf-8').splitlines()
                kinds = scopes(lines)
                for i, line in enumerate(lines):
                    if not line.strip().startswith(DOC_OPEN):
                        continue
                    if kinds[i] != 'func':
                        continue
                    j = i
                    while j + 1 < len(lines) and lines[j].strip() not in ('*/', '**/'):
                        j += 1
                    nxt = lines[j + 1].strip() if j + 1 < len(lines) else ''
                    found.append(f'{path.relative_to(root)}:{i + 1} -> {nxt[:70]}')
    return found


GOOD = '''// Conan::ImportStart
#pragma once
// Conan::ImportEnd

/*!
 * @file thing.cpp
 *
 * @brief [en] A thing.
 * @since 0.1.0
 */

namespace foc {

/** @cond */
struct Hidden {
    int a;  /**< field */
};
/** @endcond */

class Thing {
public:
    /** @brief [en] A member. @since 0.1.0 */
    void run(int n) {
        // a plain comment inside the body is fine; a doc block is not
        for (int i = 0; i < n; ++i) {
            this->a_ += i;
        }
    }

private:
    int a_ = 0;
};

}  // namespace foc
'''

BAD = GOOD.replace(
    '        for (int i = 0; i < n; ++i) {',
    '        /** @brief [en] One step of the loop. */\n'
    '        for (int i = 0; i < n; ++i) {',
)

BAD_ONE_LINE = GOOD.replace(
    '        for (int i = 0; i < n; ++i) {',
    '        /** @cond */\n        for (int i = 0; i < n; ++i) {',
)


def _fixture(root: pathlib.Path, text: str):
    (root / 'src').mkdir(parents=True, exist_ok=True)
    (root / 'include').mkdir(parents=True, exist_ok=True)
    (root / 'src' / 'thing.cpp').write_text(text, encoding='utf-8')


def selftest() -> int:
    """A block in a function body must be caught; the same file without one must pass."""
    failures = []
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        _fixture(root, BAD)
        found = violations(root)
        if not found:
            failures.append('a doc block inside a function body was not caught')
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        _fixture(root, BAD_ONE_LINE)
        found = violations(root)
        if not found:
            failures.append('a one-line doc block inside a function body was not caught')
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        _fixture(root, GOOD)
        found = violations(root)
        if found:
            failures.append(f'a clean file is reported red: {found}')

    # The counter must not be fooled by the two shapes that made the first version lie: braces in doc
    # prose, and an empty body written on one line inside a class.
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        _fixture(root, GOOD.replace(
            ' * @brief [en] A thing.',
            ' * @brief [en] A thing, in the set {0, 1}, whose prose mentions f(x) { ... }.').replace(
            '    int a_ = 0;',
            '    void empty() {}\n\n    /** @brief [en] After an empty body. */\n    int a_ = 0;'))
        found = violations(root)
        if found:
            failures.append(f'braces in prose or a one-line empty body confused the counter: {found}')

    for failure in failures:
        print(f'[docscope] selftest FAIL: {failure}', file=sys.stderr, flush=True)
    if failures:
        return 1
    print('[docscope] selftest OK (a block in a function body is caught in both shapes, braces in prose '
          'and an empty one-line body do not move the counter, and the clean file passes)', flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selftest', action='store_true', help='prove the rule can fail')
    args = parser.parse_args()
    if args.selftest:
        return selftest()

    found = violations(ROOT)
    for item in found:
        print(f'[docscope] {item}', file=sys.stderr, flush=True)
    if found:
        print(f'[docscope] FAIL ({len(found)} doc block(s) inside a function body: a block belongs at '
              f'file scope or in a type body, elsewhere it documents nothing)', flush=True)
        return 1
    files = sum(len(list((ROOT / tree).rglob(f'*.{s}'))) for tree in TREES for s in SUFFIXES)
    print(f'[docscope] OK ({files} file(s): every doc block sits at file scope or inside a type body)',
          flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
