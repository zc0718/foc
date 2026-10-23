#!/usr/bin/env python3
"""Native quality gates: one implementation behind CI, pre-commit and the editor task panel."""
import argparse
import json
import os
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


def main():
    parser = argparse.ArgumentParser(description='Native quality gates (one implementation, three callers).')
    sub = parser.add_subparsers(dest='gate', required=True)
    helps = {
        'format': 'clang-format --dry-run --Werror over include/ + src/',
        'tidy': 'clang-tidy (WarningsAsErrors) over src/ TUs; header-only: one synthetic TU per header',
        'all': 'format + tidy',
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
    args = parser.parse_args()

    if args.gate == 'format':
        return cmd_format(args.files, args.fix, args.jobs)
    if args.gate == 'tidy':
        return cmd_tidy(args.files, args.include, args.isystem)
    return 1 if (cmd_format([]) or cmd_tidy([])) else 0


if __name__ == '__main__':
    sys.exit(main())
