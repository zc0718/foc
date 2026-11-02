#!/usr/bin/env python3
"""Shape check (and one-off repair) for the rst list-tables in the documentation.

Why this exists: ``gates.py docs`` ran docfmt and the number sheet and nothing else, so
``docs/sphinx/source/foundation-log.rst`` carried 25 rows whose bullet count differed from the header
(one row had 19 items against a 6-column header) and the gate stayed green. Sphinx prints
``ERROR: Error parsing content block for the "list-table" directive`` and *drops the whole table*, and
CI greps for ``ERROR: `` and fails -- so the same tree was green locally and red in CI, with the
evidence table invisible in the rendered page in both cases. This is the "absence reads as success"
class: a table that does not parse renders as nothing, and nothing looks like a page with no rows.

The check needs no docutils, so it cannot silently skip on a machine without Sphinx.

    rst_table.py check      # exit 1 listing every ragged row
    rst_table.py fix        # rewrite ragged rows losslessly, then re-check
    rst_table.py --selftest # prove the check can go red

Repair rule, lossless: a log row is ``* - ID`` followed by five ``-`` items (class / root cause / how it
surfaced / blast radius / commit). Surplus paragraphs are merged into the "how it surfaced" cell as
extra paragraphs; missing cells are left empty. Every word is preserved, which ``fix`` asserts by
comparing the whole file's word sequence before and after, and refusing to write if it changed.
"""
import re
import sys

PATHS = ["docs/sphinx/source/foundation-log.rst"]
HEAD = re.compile(r"^(\s*)\* - (.*)$")
ITEM = re.compile(r"^(\s*)- (.*)$")


def tables(lines):
    """Yield (start, end, header_cells) for every list-table directive in the file."""
    i = 0
    while i < len(lines):
        if lines[i].strip().startswith(".. list-table::"):
            start = i
            i += 1
            while i < len(lines) and (not lines[i].strip() or lines[i].startswith("   ")):
                i += 1
            end = i
            body = lines[start:end]
            hidx = next((n for n, l in enumerate(body) if HEAD.match(l)), None)
            header = 0
            if hidx is not None:
                # the header row's cell count fixes the width of the table
                n = hidx + 1
                header = 1
                while n < len(body) and ITEM.match(body[n]):
                    header += 1
                    n += 1
            yield start, end, header
        else:
            i += 1


def rows_of(lines, start, end):
    body = lines[start:end]
    hidx = [n for n, l in enumerate(body) if HEAD.match(l)]
    out = []
    for k, i in enumerate(hidx):
        j = hidx[k + 1] if k + 1 < len(hidx) else len(body)
        block = body[i:j]
        out.append(
            {
                "line": start + i + 1,
                "i": i,
                "block": block,
                "items": 1 + sum(1 for l in block if ITEM.match(l)),
                "id": HEAD.match(block[0]).group(2).strip(),
            }
        )
    return out


def check(paths=PATHS, quiet=False):
    bad = []
    for path in paths:
        lines = open(path).read().splitlines()
        for start, end, width in tables(lines):
            if not width:
                continue
            for row in rows_of(lines, start, end):
                if row["items"] != width:
                    bad.append((path, row["line"], row["id"], row["items"], width))
    if bad:
        for path, line, rid, n, width in bad:
            print(f"  {path}:{line}: row {rid} has {n} item(s), header has {width}")
        print(f"[rst-table] {len(bad)} ragged row(s) -- a list-table like this does not parse, and Sphinx drops it whole")
        return 1
    if not quiet:
        print(f"[rst-table] OK (every row matches its header)")
    return 0


def fix_one(block, width):
    """Return the block rewritten to exactly `width` items, preserving every word."""
    head = block[0]
    starts = [n for n, l in enumerate(block) if ITEM.match(l)]
    segs = []
    for n, s in enumerate(starts):
        e = starts[n + 1] if n + 1 < len(starts) else len(block)
        seg = block[s:e]
        while seg and not seg[-1].strip():
            seg.pop()
        segs.append(seg)
    trailing = block[starts[-1] + len(segs[-1]):]
    cls, commit, narr = segs[0], segs[-1], segs[1:-1]
    target = width - 3  # narrative columns: ID and class on the left, commit on the right
    assert target >= 1, f"a {width}-column table has no room to merge into"
    if len(narr) == target:
        cols = narr
    elif len(narr) > target:
        cols = narr[: target - 1]
        for extra in narr[target - 1: -1]:
            cols[-1] = cols[-1] + [""] + para(extra)
        cols.append(narr[-1])
    else:
        cols = list(narr) + [[] for _ in range(target - len(narr))]
    out = [head] + dedent(cls)
    for c in cols:
        out.extend(dedent(c) if c else ["     - "])
    out.extend(dedent(commit))
    out.extend(trailing)
    return out


def para(seg):
    """Same, but as a continuation paragraph of the cell above it: no bullet, seven-space indent."""
    first = ITEM.match(seg[0]).group(2)
    return ["       " + first] + list(seg[1:])


def dedent(seg):
    """Strip the leading bullet from the first line, keeping the rest byte-identical."""
    first = ITEM.match(seg[0]).group(2)
    return ["     - " + first] + list(seg[1:])


def words(text):
    """Every real word, with the list bullets dropped: the markers may move, the words may not."""
    return [w for w in text.split() if w not in ("-", "*")]


def fix(paths=PATHS):
    changed = 0
    for path in paths:
        text = open(path).read()
        lines = text.splitlines()
        words_before = words(text)
        out = list(lines)
        for start, end, width in reversed(list(tables(lines))):
            if not width:
                continue
            new_body = []
            pos = start
            for row in rows_of(lines, start, end):
                new_body.extend(lines[pos: start + row["i"]])
                new_body.extend(fix_one(row["block"], width) if row["items"] != width else row["block"])
                pos = start + row["i"] + len(row["block"])
                if row["items"] != width:
                    changed += 1
            new_body.extend(lines[pos:end])
            out[start:end] = new_body
        words_after = words("\n".join(out))
        if words_before != words_after:
            print(f"REFUSED {path}: rewriting would change the text ({len(words_before)} -> {len(words_after)} words)")
            return 1
        if out != text.splitlines():
            open(path, "w").write("\n".join(out) + "\n")
    print(f"[rst-table] repaired {changed} row(s); word sequence unchanged")
    return 0


def selftest():
    """The log table has six columns; the selftest uses the same shape, not a convenient one."""
    def table(n_items):
        head = [".. list-table::", "   :header-rows: 1", "", "   * - ID", "     - Class",
                "     - Root cause", "     - How it surfaced", "     - Blast radius", "     - Commit"]
        cells = ["one", "two", "three", "four", "five", "six", "seven", "eight"]
        return head + ["   * - X-1"] + ["     - " + c for c in cells[:n_items]]

    cases = [("well formed", table(5), 0), ("one item too many", table(6), 1), ("one item too few", table(4), 1)]
    ok = True
    for name, src, want in cases:
        path = "/tmp/rst_table_selftest_" + name.replace(" ", "_") + ".rst"
        open(path, "w").write("\n".join(src) + "\n")
        import contextlib
        import io

        buf = io.StringIO()
        with contextlib.redirect_stdout(buf):
            got = check([path])
        if got != want:
            ok = False
        print(f"  selftest: {name}: expected {want}, got {got} -> {'ok' if got == want else 'WRONG'}")
        if want == 1:
            words_ = words(open(path).read())
            with contextlib.redirect_stdout(io.StringIO()):
                fix([path])
                after = check([path], quiet=True)
            same = words(open(path).read()) == words_
            if after != 0 or not same:
                ok = False
            print(f"  selftest: {name}: after fix, check={after}, words preserved={same}"
                  f" -> {'ok' if after == 0 and same else 'WRONG'}")
    print("[rst-table] selftest OK" if ok else "[rst-table] selftest FAILED")
    return 0 if ok else 1


def main(argv):
    if "--selftest" in argv:
        return selftest()
    mode = next((a for a in argv[1:] if not a.startswith("-")), "check")
    if mode == "check":
        return check()
    if mode == "fix":
        return fix()
    print(f"unknown mode {mode!r}; use check|fix|--selftest")
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))
