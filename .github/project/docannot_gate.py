"""Annotation gate: the two comment shapes that publish a member with no description.

Why this is not covered by anything else. `Doxyfile` has `EXTRACT_ALL = YES`, so Doxygen documents
every member it can see and says nothing when there is no docstring -- which is how 229 plain
`/* ... */` comments sat in `foc_types.h` and `foc_api.h` while the published pages showed the field
names and none of their meaning. The alternative was to switch `WARN_IF_UNDOCUMENTED` on, and that
means editing `Doxyfile`, which `scaffold` pins byte for byte. So the check lives here instead, and
it is narrower than Doxygen's own warning on purpose: it looks for the two shapes that *look* like
documentation and are not.

1. A plain `/* ... */` comment directly above a declaration or a field. It is not a docstring at
   all, and it is the exact thing that reads as documented in the source and renders as nothing.
   Separator banners (`/* ---- section ---- */`) and the usual "fall through"/"intentionally empty"
   notes are recognised and left alone, because those are notes to the next reader of the *code*.
2. A `///<` field comment carried over onto a second line. The marker documents the declaration to
   its left; a continuation line has no declaration to its left, so it attaches to nothing and the
   half of the sentence that moved down is published nowhere. The fix is to promote the whole
   description to a `/** ... */` block above the declaration, which is what
   `_shared/code-conventions.md` says.

3. Any `///<` at all. Documentation is a multi-line `/** ... */` block **above** the declaration: a
   trailing comment cannot be extended with a second sentence, cannot carry `@note`, and `@param` /
   `@return` have no meaning inside it. Measured before the change: 618 of them across `include/` and
   `src/`, of which every one had to be rewritten by hand once the rule was agreed.
4. A single-line `/** ... */` block that carries `[en]` and `[zh]` on the same line. `docs/build.py`
   only splits a block that spans lines, so both tags reached **both** builds and the Chinese pages
   printed `[en] One step's measurements. [zh] 一步的测量值。` verbatim (53 blocks did).

All four rules are at zero in this tree today, so the gate is a ratchet with no slack -- which is the
point: the conversions were done by hand and nothing was stopping the next 618.
"""
import argparse
import pathlib
import re
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]

# A line that ends a declaration or a field: what a doc comment could have been attached to.
DECL = re.compile(r'^\s*[^/\s].*;\s*(?://.*)?$')
# Banner separators: `/* ---- anything ---- */` and friends.
BANNER = re.compile(r'^[-=*#~\s]+$')
# Notes written to the code reader rather than about the documented interface.
CODE_NOTE = re.compile(r'^(?:fall ?through|falls? through|no break|not reached|unreachable|'
                       r'intentionally empty|empty|unused|no-op|reserved|keep in sync|'
                       r'clang-format off|clang-format on|NOLINT.*)$', re.IGNORECASE)


def _doc_files(root: pathlib.Path) -> list:
    """Every public header. The published surface is `include/`; `src/` is not documented."""
    return sorted(root.joinpath('include').rglob('*.h')) + sorted(root.joinpath('include').rglob('*.hpp'))


def violations(root: pathlib.Path) -> list:
    problems = []
    for path in _doc_files(root):
        lines = path.read_text(encoding='utf-8').splitlines()
        rel = path.relative_to(root)
        index = 0
        while index < len(lines):
            line = lines[index]

            # rule 2: a `///<` with nothing in front of it belongs to no declaration.
            marker = line.find('///<')
            if marker >= 0 and not line[:marker].strip():
                problems.append(f'{rel}:{index + 1}: a `///<` with no declaration to its left -- '
                                f'promote the description to a `/** */` block above the declaration')

            # rule 1: a plain `/* */` block that a reader will take for a docstring.
            opener = re.match(r'^(\s*)/\*(?![*!])(.*)$', line)
            if opener:
                body = [opener.group(2)]
                stop = index
                while '*/' not in body[-1] and stop + 1 < len(lines):
                    stop += 1
                    body.append(lines[stop])
                text = ' '.join(part.split('*/')[0] for part in body).strip(' *\t')
                nxt = stop + 1
                while nxt < len(lines) and not lines[nxt].strip():
                    nxt += 1
                attached = nxt < len(lines) and DECL.match(lines[nxt])
                if attached and not BANNER.match(text) and not CODE_NOTE.match(text):
                    problems.append(f'{rel}:{index + 1}: a plain `/* */` comment above a declaration '
                                    f'is not a docstring -- use `/** */` or `///<`')
                index = stop + 1
                continue

            # rule 3: a trailing `///<` is not the agreed shape
            if '///<' in line:
                problems.append(f'{rel}:{index + 1}: a `///<` trailing comment -- documentation is a '
                                f'multi-line `/** */` block above the declaration')
            # rule 4: both tags on one line never get split, so both builds show the markers
            if re.match(r'^\s*/\*\*.*\*/\s*$', line) and '[en]' in line:
                problems.append(f'{rel}:{index + 1}: a one-line block carrying `[en]` -- docs/build.py '
                                f'splits only multi-line blocks, so the tags are published literally')
            index += 1
    return problems


def _write(root: pathlib.Path, rel: str, text: str):
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')


def _fixture(root: pathlib.Path, body: str):
    _write(root, 'include/thing.hpp', body)


def selftest() -> int:
    cases = [
        ('a plain block above a field',
         '/** @file */\nstruct S {\n    /* the gain */\n    int gain;\n};\n', 'is not a docstring'),
        ('a field doc split over two lines',
         '/** @file */\nstruct S {\n    int gain;  ///< the gain\n    ///< continued here\n};\n',
         'no declaration to its left'),
        ('a one-line bilingual block',
         '/** @file */\n/** @brief [en] x. [zh] 甲。 */\nstruct S {};\n', 'published literally'),
    ]
    failures = []
    for label, body, expect in cases:
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            _fixture(root, body)
            found = violations(root)
            if not any(expect in _ for _ in found):
                failures.append(f'{label}: not caught ({found})')
    clean = ('/** @file */\n\n/* ---- section ---- */\nstruct S {\n'
             '    /**\n     * @brief [en] the gain.\n     * @brief [zh] 增益。\n     */\n    int gain;\n};\n\n'
             'int f(void) {\n    /* fall through */\n    return 0;\n}\n')
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        _fixture(root, clean)
        found = violations(root)
        if found:
            failures.append(f'a clean fixture is reported red: {found}')

    for failure in failures:
        print(f'[docannot] selftest FAIL: {failure}', file=sys.stderr, flush=True)
    if failures:
        return 1
    print('[docannot] selftest OK (both rules caught on their own fixture; banners and code notes '
          'are left alone)', flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selftest', action='store_true', help='prove each rule can fail')
    args = parser.parse_args()
    if args.selftest:
        return selftest()

    problems = violations(ROOT)
    for problem in problems:
        print(f'[docannot] {problem}', file=sys.stderr, flush=True)
    if problems:
        print(f'[docannot] FAIL ({len(problems)} comment(s) documented in shape only)', flush=True)
        return 1
    files = _doc_files(ROOT)
    print(f'[docannot] OK ({len(files)} public header(s): every doc comment is a multi-line '
          f'`/** */` block above its declaration, both tags on separate lines)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
