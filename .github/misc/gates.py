#!/usr/bin/env python3
"""Native quality gates: one implementation behind CI, pre-commit and the editor task panel."""
import argparse
import fnmatch
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

# The gate bodies used to be copy-pasted into the workflow, the pre-commit config and
# .vscode/tasks.json, and the copies drifted (the editor one dropped the eigen/ETL
# include paths and pinned -std=c++17 by hand). One implementation, three callers:
#   CI          .github/workflows/security-linters.yml
#   pre-commit  .github/misc/.pre-commit-config.yaml
#   VS Code     .vscode/tasks.json
# Values the user can edit in metadata.json must never be re-typed here, so the C/C++
# standards are read from metadata.json instead of being hardcoded.
# Usage:
#   python3 .github/misc/gates.py format [FILE ...]   # clang-format --dry-run --Werror
#   python3 .github/misc/gates.py tidy   [FILE ...]   # clang-tidy (WarningsAsErrors)
#   python3 .github/misc/gates.py guard  [FILE ...]   # forbidden #include, rules in .include-guard.txt
#   python3 .github/misc/gates.py strict [FILE ...]   # every public header under -fno-exceptions/-fno-rtti
#   python3 .github/misc/gates.py all
# format covers include/ + src/. tidy covers src/ translation units; when metadata says
# is_header (or src/ holds no TU) it instead synthesizes one TU per public header, which
# also proves each header is self-contained. Otherwise a header-only package would have
# nothing to check and the gate would pass vacuously.
# tidy include roots: -Iinclude, then EIGEN_INC (default /usr/include/eigen3), then
# GATES_INCLUDE (os.pathsep-separated) and -isystem $ETL_INC.
# Exit codes: 0 clean, 1 gate failed, 2 setup/usage error.

ROOT = Path(__file__).resolve().parents[2]
GATE_DIRS = ('include', 'src')
TIDY_DIRS = ('src',)
C_SUFFIXES = ('.c', '.h')
CPP_SUFFIXES = ('.cpp', '.hpp')
FORMAT_C = '.github/misc/.clang-format-c'
FORMAT_CPP = '.github/misc/.clang-format-cpp'
TIDY_CONFIG = '.github/misc/.clang-tidy'
GUARD_CONFIG = '.github/misc/.include-guard.txt'
LICENSE_CONFIG = '.github/misc/.license-guard.txt'

_META = None


def _die(message):
    """Setup error: never report this as a gate verdict."""
    print(f'[gates] error: {message}', file=sys.stderr, flush=True)
    raise SystemExit(2)


def _rel(path):
    """Repo-relative posix path, so output is stable on every platform."""
    try:
        return path.relative_to(ROOT).as_posix()
    except ValueError:
        return path.as_posix()


def _meta(key, default=None):
    """One metadata.json field, read once: the single source, never re-typed here."""
    global _META
    if _META is None:
        try:
            _META = json.loads((ROOT / 'metadata.json').read_text(encoding='utf-8'))
        except (OSError, ValueError) as exc:
            _die(f'cannot read metadata.json: {exc}')
        if not isinstance(_META, dict):
            _die('metadata.json is not a JSON object')
    return _META.get(key, default)


def _standards():
    """C/C++ standards from metadata.json."""
    return str(_meta('build_cstd', '11')), str(_meta('build_cppstd', '17'))


def _require(tool, hint):
    exe = shutil.which(tool)
    if not exe:
        _die(f'{tool} not found on PATH; {hint}')
    return exe


def _version(exe):
    out = subprocess.run([exe, '--version'], capture_output=True, text=True)
    lines = (out.stdout or out.stderr).strip().splitlines()
    return lines[0] if lines else 'unknown'


def _gated_files(explicit, dirs):
    """C/C++ files under the given gate dirs, or just the given paths that live in them."""
    candidates = []
    if explicit:
        for item in explicit:
            path = Path(item)
            if not path.is_absolute():
                path = ROOT / path
            if path.is_file():
                candidates.append(path.resolve())
    else:
        for name in dirs:
            base = ROOT / name
            if base.is_dir():
                candidates.extend(p.resolve() for p in base.rglob('*') if p.is_file())
    files = []
    for path in candidates:
        if path.suffix not in C_SUFFIXES + CPP_SUFFIXES:
            continue
        try:
            if path.relative_to(ROOT).parts[0] not in dirs:
                continue
        except ValueError:
            continue
        files.append(path)
    return sorted(set(files))


def _empty(explicit, gate, dirs):
    """A whole-tree run must never pass vacuously; an explicit file list may be a no-op."""
    scope = ', '.join(f'{name}/' for name in dirs)
    if explicit:
        print(f'[gates] {gate}: nothing to check in the given file list (scope: {scope})', flush=True)
        return 0
    print(f'[gates] {gate}: FAIL -- no C/C++ file found under {scope}', file=sys.stderr, flush=True)
    return 1


def _format_cmd(exe, path, fix):
    """.c/.h route to the C config, .cpp/.hpp to the C++ one (clang-format maps .h to C++)."""
    style = FORMAT_C if path.suffix in C_SUFFIXES else FORMAT_CPP
    cmd = [exe, f'--style=file:{style}']
    cmd += ['-i'] if fix else ['--dry-run', '--Werror']
    cmd.append(_rel(path))
    return cmd


def cmd_format(explicit, fix=False, jobs=1):
    files = _gated_files(explicit, GATE_DIRS)
    if not files:
        return _empty(explicit, 'format', GATE_DIRS)
    exe = _require('clang-format', 'install it with `pip install clang-format==23.1.0` (>= 19 needed)')
    print(f'[gates] format({"fix" if fix else "check"}): {len(files)} file(s), {_version(exe)}', flush=True)
    with ThreadPoolExecutor(max_workers=max(jobs, 1)) as pool:
        results = list(pool.map(
            lambda p: (p, subprocess.run(_format_cmd(exe, p, fix), cwd=ROOT, capture_output=True, text=True)),
            files,
        ))
    failed = []
    for path, done in results:
        if done.returncode != 0:
            failed.append(path)
            print(f'--- {_rel(path)}', flush=True)
            print((done.stdout + done.stderr).strip(), flush=True)
    if failed:
        print(f'[gates] format: FAIL ({len(failed)}/{len(files)} file(s))', flush=True)
        return 1
    print(f'[gates] format: OK ({len(files)} file(s))', flush=True)
    return 0


def _tidy_flags(extra_include, extra_isystem):
    """Header roots src/ needs that no -I in the repo provides (CI exports ETL_INC)."""
    includes = ['include']
    eigen = os.environ.get('EIGEN_INC') or '/usr/include/eigen3'
    generic = os.environ.get('GATES_INCLUDE', '').split(os.pathsep)
    for path in [eigen] + generic + list(extra_include):
        if path and Path(path).is_dir() and path not in includes:
            includes.append(path)
    etl = os.environ.get('ETL_INC')
    if etl and not Path(etl).is_dir():
        print(f'[gates] warning: ETL_INC={etl} is not a directory', file=sys.stderr, flush=True)
    isystems = []
    for path in [etl] + list(extra_isystem):
        if path and Path(path).is_dir() and path not in isystems:
            isystems.append(path)
    return includes, isystems


def _tidy_cmd(exe, path, cstd, cppstd, includes, isystems):
    std = f'-std=c{cstd}' if path.suffix in C_SUFFIXES else f'-std=c++{cppstd}'
    cmd = [exe, f'--config-file={TIDY_CONFIG}', _rel(path), '--', std]
    cmd += [f'-I{d}' for d in includes]
    for dir_path in isystems:
        cmd += ['-isystem', dir_path]
    return cmd


def _header_units(headers, workdir):
    """One synthetic TU per public header; compiling a header alone also proves self-containment."""
    units = []
    for index, header in enumerate(headers):
        suffix = '.c' if header.suffix in C_SUFFIXES else '.cpp'
        tu = Path(workdir) / f'tu_{index:03d}{suffix}'
        # include through the -Iinclude root so diagnostics name the real header path
        rel = header.relative_to(ROOT / 'include').as_posix()
        tu.write_text(f'#include "{rel}"\n', encoding='utf-8')
        units.append((header, tu))
    return units


def _tidy_run(exe, pairs, cstd, cppstd, includes, isystems):
    """pairs = (label, file to compile); returns the labels that failed."""
    failed = []
    for label, path in pairs:
        done = subprocess.run(
            _tidy_cmd(exe, path, cstd, cppstd, includes, isystems),
            cwd=ROOT, capture_output=True, text=True,
        )
        if done.returncode != 0:
            failed.append(label)
            print(f'--- {_rel(label)}', flush=True)
            print((done.stdout + done.stderr).strip(), flush=True)
    return failed


def cmd_tidy(explicit, extra_include=(), extra_isystem=()):
    units = _gated_files(explicit, TIDY_DIRS)
    headers = _gated_files(explicit, ('include',))
    if explicit:
        plan_units, plan_headers = units, headers
    elif _meta('is_header') or not units:
        plan_units, plan_headers = [], headers
    else:
        plan_units, plan_headers = units, []
    if not plan_units and not plan_headers:
        return _empty(explicit, 'tidy', TIDY_DIRS + ('include',))
    exe = _require('clang-tidy', 'install it with `apt-get install clang-tidy`')
    cstd, cppstd = _standards()
    includes, isystems = _tidy_flags(extra_include, extra_isystem)
    total = len(plan_units) + len(plan_headers)
    print(f'[gates] tidy: {total} target(s), {_version(exe)}, -std=c{cstd} / -std=c++{cppstd}', flush=True)
    failed = _tidy_run(exe, [(p, p) for p in plan_units], cstd, cppstd, includes, isystems)
    if plan_headers:
        # header-only package (or a header passed explicitly): no TU of its own to inspect
        print(f'[gates] tidy: header mode -- {len(plan_headers)} synthetic TU(s), one per public header', flush=True)
        with tempfile.TemporaryDirectory(prefix='gates-tidy-') as workdir:
            pairs = _header_units(plan_headers, workdir)
            failed += _tidy_run(exe, pairs, cstd, cppstd, includes, isystems)
    if failed:
        print(f'[gates] tidy: FAIL ({len(failed)}/{total} file(s))', flush=True)
        return 1
    print(f'[gates] tidy: OK ({total} file(s))', flush=True)
    return 0


def _scoped_rules(config_name, flags=0):
    """(scope glob, compiled regex) pairs from a rule file: "<scope glob> <regex>" per line,
    `#` comments allowed. One loader for every scoped gate, so a new gate is a new rule file rather
    than a new parser."""
    config = ROOT / config_name
    if not config.is_file():
        _die(f'{config_name} not found; it lists "<scope glob> <forbidden regex>" per line')
    rules = []
    for lineno, raw in enumerate(config.read_text(encoding='utf-8').splitlines(), start=1):
        line = raw.split('#', 1)[0].strip()
        if not line:
            continue
        parts = line.split(None, 1)
        if len(parts) != 2:
            _die(f'{config_name}:{lineno}: expected "<scope glob> <regex>", got {raw!r}')
        scope, pattern = parts
        try:
            rules.append((scope, re.compile(pattern, flags)))
        except re.error as exc:
            _die(f'{config_name}:{lineno}: bad regex {pattern!r}: {exc}')
    if not rules:
        _die(f'{config_name} defines no rule')
    return rules


def _guard_rules():
    """Include-layering rules (see .include-guard.txt)."""
    return _scoped_rules(GUARD_CONFIG)
    return rules


def _scoped_name(path):
    """Path relative to include/ or src/ -- what the guard scopes are written against."""
    for name in GATE_DIRS:
        try:
            return path.resolve().relative_to(ROOT / name).as_posix()
        except ValueError:
            continue
    return path.as_posix()


def _include_directives(path):
    """(line number, included path) for every #include directive in the file."""
    pattern = re.compile(r'^\s*#\s*include\s*[<"]([^>"]+)[>"]')
    found = []
    with open(path, encoding='utf-8', errors='replace') as handle:
        for lineno, line in enumerate(handle, start=1):
            match = pattern.match(line)
            if match:
                found.append((lineno, match.group(1)))
    return found


def cmd_guard(explicit):
    """Layering guard: a scope may not include something its rules forbid (e.g. core must stay
    hardware-agnostic). Rules live in .github/misc/.include-guard.txt, one per line, so a derived
    project states its own layering instead of editing this file."""
    files = _gated_files(explicit, GATE_DIRS)
    if not files:
        return _empty(explicit, 'guard', GATE_DIRS)
    rules = _guard_rules()
    print(f'[gates] guard: {len(files)} file(s), {len(rules)} rule(s) from {GUARD_CONFIG}', flush=True)
    violations = 0
    for path in files:
        scoped = _scoped_name(path)
        active = [regex for scope, regex in rules if fnmatch.fnmatch(scoped, scope)]
        for lineno, included in _include_directives(path) if active else []:
            for regex in active:
                if regex.search(included):
                    violations += 1
                    print(f'{_rel(path)}:{lineno}: forbidden include <{included}> ({regex.pattern})', flush=True)
    if violations:
        print(f'[gates] guard: FAIL ({violations} violation(s))', flush=True)
        return 1
    print(f'[gates] guard: OK ({len(files)} file(s))', flush=True)
    return 0


def cmd_license(explicit):
    """Copyleft gate: shipped sources may not carry a copyleft or non-free licence marker.

    Rules live in .github/misc/.license-guard.txt, one "<scope glob> <regex>" per line, because a
    derived project may legitimately vendor a GPL subtree under its own scope while the library
    itself stays permissive."""
    files = _gated_files(explicit, GATE_DIRS)
    if not files:
        return _empty(explicit, 'license', GATE_DIRS)
    rules = _scoped_rules(LICENSE_CONFIG, re.IGNORECASE)
    print(f'[gates] license: {len(files)} file(s), {len(rules)} rule(s) from {LICENSE_CONFIG}', flush=True)
    violations = 0
    for path in files:
        scoped = _scoped_name(path)
        active = [regex for scope, regex in rules if fnmatch.fnmatch(scoped, scope)]
        if not active:
            continue
        for lineno, line in enumerate(path.read_text(encoding='utf-8', errors='replace').splitlines(), start=1):
            for regex in active:
                if regex.search(line):
                    violations += 1
                    print(f'{_rel(path)}:{lineno}: forbidden licence marker ({regex.pattern})', flush=True)
                    print(f'    {line.strip()[:100]}', flush=True)
    if violations:
        print(f'[gates] license: FAIL ({violations} violation(s))', flush=True)
        return 1
    print(f'[gates] license: OK ({len(files)} file(s))', flush=True)
    return 0


def cmd_strict(explicit):
    """Every public header must compile standalone under the project's hard constraints
    (-fno-exceptions -fno-rtti for C++, -pedantic for C). ADR-level promises become testable here."""
    headers = _gated_files(explicit, ('include',))
    if not headers:
        return _empty(explicit, 'strict', ('include',))
    cstd, cppstd = _standards()
    compilers = {
        '.h': (_require('gcc', 'install it with `apt-get install gcc`'), [f'-std=c{cstd}', '-pedantic']),
        '.hpp': (_require('g++', 'install it with `apt-get install g++`'),
                 [f'-std=c++{cppstd}', '-fno-exceptions', '-fno-rtti']),
    }
    print(f'[gates] strict: {len(headers)} header(s), -std=c{cstd} / -std=c++{cppstd} '
          f'-fno-exceptions -fno-rtti', flush=True)
    failed = []
    with tempfile.TemporaryDirectory(prefix='gates-strict-') as workdir:
        for header in headers:
            exe, flags = compilers[header.suffix]
            tu = Path(workdir) / (header.stem + ('.c' if header.suffix == '.h' else '.cpp'))
            tu.write_text(f'#include "{header.relative_to(ROOT / "include").as_posix()}"\n'
                          '\nint main(void) { return 0; }\n', encoding='utf-8')
            cmd = [exe, *flags, '-Wall', '-Wextra', '-Werror', '-I', str(ROOT / 'include'),
                   '-fsyntax-only', str(tu)]
            done = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True)
            if done.returncode != 0:
                failed.append(header)
                print(f'--- {_rel(header)}', flush=True)
                print((done.stdout + done.stderr).strip(), flush=True)
    if failed:
        print(f'[gates] strict: FAIL ({len(failed)}/{len(headers)} header(s))', flush=True)
        return 1
    print(f'[gates] strict: OK ({len(headers)} header(s))', flush=True)
    return 0


def cmd_claims(explicit):
    """Every installed header must carry a claim row with a level and evidence that exists.
    Delegates to claim_gate.py so the rule has one implementation.

    With `--impact <path|identifier>` it answers a different question -- which claims rest on this thing,
    and which cases must be re-run -- by reading the basis graph in `claim-basis.json`. That is the
    query the foundation log's blast-radius column is filled from.
    """
    args = [sys.executable, str(ROOT / '.github' / 'misc' / 'claim_gate.py'), *explicit]
    done = subprocess.run(args, cwd=ROOT, text=True)
    return done.returncode


def cmd_docs(_explicit):
    """Doxygen block comments: aligned star column, consistent continuation indent, 120 columns,
    no mid-CJK breaks, one tag line per language group, no multi-line `///<`. Delegates to docfmt.py
    so the rule has one implementation. Then checks the numbers the documentation claims against the
    artefacts the build produced, because a number in prose with no owner drifts."""
    done = subprocess.run([sys.executable, str(ROOT / '.github' / 'misc' / 'docfmt.py'), '--check'],
                          cwd=ROOT, text=True)
    if done.returncode != 0:
        return done.returncode
    numbers = subprocess.run([sys.executable, str(ROOT / '.github' / 'misc' / 'number_sheet.py'), '--check'],
                             cwd=ROOT, text=True)
    return numbers.returncode


def cmd_boundaries(_explicit):
    """Every statement on the boundaries page names the case that pins it, and that case exists.

    The page is only worth having if it cannot rot: a limit documented next to a case that was renamed, deleted or never
    written is worse than no page, because it reads as a guarantee. So each row carries `Suite.Case`, and this gate
    refuses a reference for which no `TEST(Suite, Case)` exists in the test tree. It also refuses a page that has lost its
    rows -- an emptied table would satisfy a per-row check trivially."""
    page = ROOT / 'docs' / 'sphinx' / 'source' / 'boundaries.rst'
    if not page.is_file():
        _die(f'missing {_rel(page)}')
    text = page.read_text(encoding='utf-8')
    refs = re.findall(r'``([A-Z][A-Za-z0-9_]*)\.([A-Za-z0-9_]+)``', text)
    sources = '\n'.join(path.read_text(encoding='utf-8', errors='replace')
                         for path in (ROOT / 'test_package' / 'test').rglob('*.cpp'))
    missing = [f'{suite}.{case}' for suite, case in refs if f'TEST({suite}, {case})' not in sources]
    minimum = 8
    if len(refs) < minimum:
        _die(f'{_rel(page)}: {len(refs)} pinned statement(s), expected at least {minimum}')
    if missing:
        for name in missing:
            print(f'[boundaries] no TEST() pins {name}', file=sys.stderr, flush=True)
        _die(f'{_rel(page)}: {len(missing)} of {len(refs)} statement(s) point at a case that does not exist')
    print(f'[boundaries] OK ({len(refs)} statement(s), each pinned by a case that exists)')
    return 0


def _fmpy_python():
    """An interpreter that can `import fmpy`, or None.

    Same optional-material convention as the FMI headers: an explicit environment variable first, then the materials
    checkout this box happens to have, then the interpreter running the gate."""
    candidates = []
    explicit = os.environ.get('FOC_FMPY_PYTHON')
    if explicit:
        candidates.append(explicit)
    candidates.append(str(ROOT / 'workspace' / 'materials' / '.venv' / 'bin' / 'python'))
    candidates.append(sys.executable)
    for exe in candidates:
        if not Path(exe).is_file():
            continue
        try:
            done = subprocess.run([exe, '-c', 'import fmpy'], capture_output=True, text=True, timeout=300)
        except OSError:
            continue
        if done.returncode == 0:
            return exe
    return None


def cmd_fmu(_explicit, tolerant=False):
    """The FMU is a claim about a third-party master, so the check is that a third-party master loads it.

    Build it with `api/fmu/build.py` (the FMI headers are the standard ones FMPy ships, unmodified), then drive it with
    `api/fmu/smoke.py`, which instantiates the block, moves the rotor from the master side and asserts on what comes
    back. Both halves need optional material -- a Python that can import fmpy, and the C archives a `conan create`
    produced. When they are absent this prints one SKIPPED line and stops: loudly, because a silent skip would turn
    "the FMU loads in a standard master" into a claim nobody has checked."""
    python = _fmpy_python()
    archives = sorted(Path.home().glob('.conan2/p/b/foc*/p/lib/libfoc_c.a'))
    reason = None
    if python is None:
        reason = 'no interpreter that can import fmpy (set FOC_FMPY_PYTHON to one)'
    elif not archives:
        reason = 'no libfoc_c.a in the Conan cache (run `conan create .` first)'
    if reason is not None:
        if not tolerant:
            _die(f'the FMU check cannot run: {reason}')
        print(f'[fmu] SKIPPED: {reason}', flush=True)
        return 0
    built = subprocess.run([python, str(ROOT / 'api' / 'fmu' / 'build.py')], cwd=ROOT, text=True)
    if built.returncode != 0:
        return built.returncode
    checked = subprocess.run([python, str(ROOT / 'api' / 'fmu' / 'smoke.py')], cwd=ROOT, text=True)
    return checked.returncode


def main():
    parser = argparse.ArgumentParser(description='Native quality gates (one implementation, three callers).')
    sub = parser.add_subparsers(dest='gate', required=True)
    helps = {
        'format': 'clang-format --dry-run --Werror over include/ + src/',
        'tidy': 'clang-tidy (WarningsAsErrors) over src/ TUs; header-only: one synthetic TU per header',
        'guard': 'forbidden #include directives, scoped rules from .github/misc/.include-guard.txt',
        'license': 'copyleft / non-free licence markers, scoped rules from .github/misc/.license-guard.txt',
        'strict': 'every public header compiles standalone with -fno-exceptions -fno-rtti (C: -pedantic)',
        'claims': 'every installed header has a README claim row with a level and existing evidence; '
                  '`claims --impact <path|name>` lists what rests on it',
        'docs': 'Doxygen blocks: star alignment, continuation indent, 120 columns, no mid-CJK breaks',
        'boundaries': 'every statement on the boundaries page names a test case that exists',
        'fmu': 'build the FMU and load it in a standard FMI master (FMPy); SKIPPED when its material is absent',
        'all': 'format + tidy + guard + license + strict + claims + docs + boundaries + fmu',
    }
    for name, text in helps.items():
        leaf = sub.add_parser(name, help=text)
        leaf.add_argument('files', nargs='*', help='check only these files (default: every gated file)')
        if name == 'format':
            leaf.add_argument('--fix', action='store_true', help='rewrite files instead of --dry-run')
            leaf.add_argument('--jobs', type=int, default=os.cpu_count() or 1, help='parallel clang-format workers')
        if name == 'tidy':
            leaf.add_argument('--include', action='append', default=[], metavar='DIR', help='extra -I (repeatable)')
            leaf.add_argument('--isystem', action='append', default=[], metavar='DIR', help='extra -isystem (repeatable)')
        if name == 'claims':
            leaf.add_argument('--impact', metavar='TERM',
                              help='list the claims that rest on this path or identifier, with the cases to re-run')
    args = parser.parse_args()

    if args.gate == 'format':
        return cmd_format(args.files, args.fix, args.jobs)
    if args.gate == 'tidy':
        return cmd_tidy(args.files, args.include, args.isystem)
    if args.gate == 'guard':
        return cmd_guard(args.files)
    if args.gate == 'license':
        return cmd_license(args.files)
    if args.gate == 'strict':
        return cmd_strict(args.files)
    if args.gate == 'claims':
        return cmd_claims([*args.files, *(['--impact', args.impact] if args.impact else [])])
    if args.gate == 'docs':
        return cmd_docs(args.files)
    if args.gate == 'boundaries':
        return cmd_boundaries(args.files)
    if args.gate == 'fmu':
        return cmd_fmu(args.files)
    return 1 if (cmd_format([]) or cmd_tidy([]) or cmd_guard([]) or cmd_license([]) or cmd_strict([]) or
                 cmd_claims([]) or cmd_docs([]) or cmd_boundaries([]) or cmd_fmu([], tolerant=True)) else 0


if __name__ == '__main__':
    sys.exit(main())
