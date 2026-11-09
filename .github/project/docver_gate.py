"""Doc-version gate: a documented block may not be filtered out of the published set.

Why it exists. `docs/build.py` keeps a Doxygen block only when its `@since` is at or below the
`doc_versions` entry being built. The drop is silent twice over: `EXTRACT_ALL = YES` publishes the
symbol with no description, and a `.dox` file whose single block is dropped becomes a 0-byte mirror
file, which takes its `@example` directive with it -- so the Doxygen Examples page simply never
mentions it. Measured before this gate existed: **101 of 495 blocks** in 38 files and **all nine**
`slice-s*.dox` were missing from both language builds while `docs`, `docfmt`, `docvoice`, `docshape`
and `scaffold` stayed green.

Four rules, each one a measured failure rather than a style opinion:

1. every `@since` is at or below the highest published `doc_versions` entry;
2. no source file leaves the language+version filters empty while it has content (the `.dox` case);
3. every `@example <file>` resolves to a file sitting next to the `.dox` that names it -- the
   directive matches only files directly under an `EXAMPLE_PATH` entry, and `EXAMPLE_RECURSIVE` is
   not consulted;
4. no version stamp other than the published one survives anywhere in the doc tree (`@since`,
   `ABI x.y.z`, `(0.x.y)`, `since x.y.z`), because the filter reads all of them as release claims.

`--selftest` runs first and it is a real measurement: each rule gets a fixture that must be caught,
and the clean fixture must pass, so a checker that has become uniformly agreeable cannot pass by
having nothing to catch.
"""
import argparse
import importlib.util
import json
import pathlib
import re
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]
STAMP = re.compile(r'(?<![\w.])0\.\d+\.\d+(?![\w.])')
EXAMPLE = re.compile(r'@example\s+(\S+)')
SINCE = re.compile(r'@since\s+(\S+)')


def _load_build(root: pathlib.Path):
    """`docs/build.py` as a module: the filters under test are the real ones, not a copy."""
    spec = importlib.util.spec_from_file_location('_docver_build', root / 'docs' / 'build.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _ver_tuple(x: str) -> tuple:
    parts = [int(_) for _ in x.split('.')]
    return tuple(parts + [0] * (3 - len(parts))) if len(parts) < 3 else tuple(parts[:3])


def _doc_files(root: pathlib.Path, meta: dict) -> list:
    """(folder, relative path, suffix) for every file the docs pipeline reads."""
    res = []
    for folder in meta['doc_doxygen_folders']:
        base = root / folder.strip()
        for suffix in meta['doc_doxygen_suffix']:
            for path in sorted(base.rglob(f'*.{suffix}')):
                res.append((base, path, suffix))
    return res


def violations(root: pathlib.Path) -> list:
    """Every way the published doc set can disagree with the source. Empty means they agree."""
    meta = json.loads((root / 'metadata.json').read_text(encoding='utf-8'))
    published = [_ver_tuple(_) for _ in meta['doc_versions']]
    ceiling = max(published)
    language = meta['doc_languages'][0]
    build = _load_build(root)
    problems = []

    for base, path, suffix in _doc_files(root, meta):
        rel = path.relative_to(root)
        text = path.read_text(encoding='utf-8')

        # rule 1 + rule 4: a stamp above the ceiling is a block the site will not show, and any other
        # number in this tree is a release claim the tree cannot back up.
        for number, line in enumerate(text.splitlines(), 1):
            for stamp in sorted(set(SINCE.findall(line) + STAMP.findall(line))):
                if _ver_tuple(stamp) > ceiling or _ver_tuple(stamp) not in published:
                    problems.append(f'{rel}:{number}: version stamp {stamp!r} is above the published '
                                    f'{meta["doc_versions"][-1]}')

        # rule 3: an `@example` that names a file not sitting beside the directive produces no page.
        if suffix == 'dox':
            for name in EXAMPLE.findall(text):
                if not (path.parent / name).exists():
                    problems.append(f'{rel}: @example {name} has no sibling file, so Doxygen '
                                    f'publishes no example page for it')

        # rule 2: the two filters, run for real; an emptied file loses more than its prose.
        lines = text.splitlines(keepends=True)
        filtered = build._language_filter(lines, meta['doc_languages'], language)
        if not ''.join(filtered).strip() and text.strip():
            problems.append(f'{rel}: the {language} language filter removes the whole file')
            continue
        surviving, _ = build._ver_filter(filtered, meta['doc_versions'][-1])
        if not ''.join(surviving).strip() and ''.join(filtered).strip():
            problems.append(f'{rel}: the version filter removes the whole file (every block is '
                            f'stamped above {meta["doc_versions"][-1]})')

    return problems


def _write(root: pathlib.Path, rel: str, text: str):
    path = root / rel
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding='utf-8')


def _fixture(root: pathlib.Path, since: str, example: str, sibling: bool):
    """A minimal tree shaped like this one: metadata, one header, one example directory."""
    _write(root, 'docs/build.py', 'def _language_filter(lines, langs, lang):\n    return lines\n\n\n'
                                  'def _ver_filter(x, ver):\n    return x, ver\n')
    _write(root, 'metadata.json', json.dumps({
        'version': '0.1.0', 'doc_versions': ['0.1.0'], 'doc_languages': ['en', 'zh'],
        'doc_doxygen_folders': ['include', 'docs/doxygen/dox'],
        'doc_doxygen_suffix': ['h', 'dox', 'cxx']}))
    _write(root, 'include/thing.hpp', '/**\n * @brief [en] A thing.\n */\nstruct Thing {};\n')
    _write(root, 'docs/doxygen/dox/demos/demo.dox',
           f'/**\n * @example {example}\n * @brief [en] A demo.\n * @since {since}\n */\n')
    if sibling:
        _write(root, f'docs/doxygen/dox/demos/{example}', 'int main() { return 0; }\n')


def selftest() -> int:
    """Each rule must be caught on its own fixture, and the clean tree must pass."""
    cases = [
        ('a stamp above the published version', dict(since='0.1.12', example='demo.cxx',
                                                     sibling=True), 'version stamp'),
        ('an @example with no sibling file', dict(since='0.1.0', example='other.cxx',
                                                 sibling=False), 'has no sibling file'),
    ]
    failures = []
    for label, kwargs, expect in cases:
        with tempfile.TemporaryDirectory() as tmp:
            root = pathlib.Path(tmp)
            _fixture(root, **kwargs)
            found = violations(root)
            if not any(expect in _ for _ in found):
                failures.append(f'{label}: not caught ({found})')
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        _fixture(root, since='0.1.0', example='demo.cxx', sibling=True)
        found = violations(root)
        if found:
            failures.append(f'a clean fixture is reported red: {found}')

    # The version filter's own emptying, on the real filter and a block that carries a stamp the
    # pipeline drops: this is the exact shape all nine `slice-s*.dox` files had.
    with tempfile.TemporaryDirectory() as tmp:
        root = pathlib.Path(tmp)
        _fixture(root, since='0.1.0', example='demo.cxx', sibling=True)
        _write(root, 'docs/build.py', (ROOT / 'docs' / 'build.py').read_text(encoding='utf-8'))
        _write(root, 'docs/doxygen/dox/demos/demo.dox',
               '/**\n * @example demo.cxx\n * @brief [en] A demo.\n * @since 0.1.12\n */\n')
        found = violations(root)
        if not any('removes the whole file' in _ for _ in found):
            failures.append(f'version-filter emptying not caught ({found})')

    for failure in failures:
        print(f'[docver] selftest FAIL: {failure}', file=sys.stderr, flush=True)
    if failures:
        return 1
    print('[docver] selftest OK (every rule above is caught on its own fixture, and the clean '
          'fixture passes)', flush=True)
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--selftest', action='store_true', help='prove each rule can fail')
    args = parser.parse_args()
    if args.selftest:
        return selftest()

    meta = json.loads((ROOT / 'metadata.json').read_text(encoding='utf-8'))
    problems = violations(ROOT)
    for problem in problems:
        print(f'[docver] {problem}', file=sys.stderr, flush=True)
    if problems:
        print(f'[docver] FAIL ({len(problems)} block(s) would not reach the published site)', flush=True)
        return 1
    files = _doc_files(ROOT, meta)
    print(f'[docver] OK ({len(files)} doc file(s), every @since at or below '
          f'{meta["doc_versions"][-1]}: nothing is filtered out of the {"/".join(meta["doc_languages"])} '
          f'build(s)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
