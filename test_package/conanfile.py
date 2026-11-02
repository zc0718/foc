from conan import ConanFile
from conan.tools.cmake import CMake, cmake_layout, CMakeToolchain
from conan.tools.build import can_run, cross_building
from conan.tools.files import copy
from conan.tools.env import VirtualRunEnv, VirtualBuildEnv
from pathlib import Path
import subprocess
import shutil
import json
import yaml
import re
import sys
import os
sep = os.path.sep
MAIN_CPP = 'main.cpp'
_get_file_name = (lambda x: x.split(sep)[-1])


def _lcov_ignore_errors() -> str:
    """The error classes to continue past, named for the lcov in use: geninfo 2.x renamed the line-range ones from
    `gcov` to `mismatch`/`inconsistent` and **validates the list**, while geninfo 1.x rejects a name it does not know
    (measured: 1.16 exits 2 on `mismatch`), so one fixed list cannot serve both.
    """
    try:
        _version = subprocess.run(['lcov', '--version'], check=True, capture_output=True, text=True).stdout
        _major = int(re.search(r'(\d+)\.', _version).group(1))
    except (OSError, AttributeError, ValueError, subprocess.CalledProcessError):
        return 'mismatch,source,inconsistent'
    return 'mismatch,source,inconsistent' if _major >= 2 else 'gcov,source'


def _clear_test_build():
    _build = sep.join(__file__.split(sep)[:-1] + ['build'])
    _presets = sep.join(__file__.split(sep)[:-1] + ['CMakeUserPresets.json'])
    if os.path.exists(_build):
        shutil.rmtree(_build)
    if os.path.exists(_presets):
        os.remove(_presets)


def _is_within(path: str, folder: str) -> bool:
    """True when `path` lives inside `folder` (both resolved). Used to keep foreign executables
    — interpreters, system tools — out of the coverage export."""
    _p, _f = os.path.abspath(os.path.realpath(path)), os.path.abspath(os.path.realpath(folder))
    try:
        return os.path.commonpath([_p, _f]) == _f
    except ValueError:  # different drives on Windows
        return False


def _recursive_find(root: str, obj_files: list[str]):
    for f in os.listdir(root):
        _f = root + sep + f
        if not os.path.isdir(_f):
            if f in obj_files:
                yield _f
        else:
            yield from _recursive_find(_f, obj_files)


def _entry_lists() -> list[str]:
    return ['#include <gtest/gtest.h>\n',
            '\n',
            '\n',
            'int main(int argc, char **argv) {\n',
            '    ::testing::InitGoogleTest(&argc, argv);\n',
            '    return RUN_ALL_TESTS();\n',
            '}\n']


def _source_relpath(tracefile_path: str) -> str:
    """Trim a tracefile's absolute path to the project-relative part, so two platforms can compare file sets."""
    _normalised = tracefile_path.replace('\\', '/')
    for _root in ('/src/', '/include/', '/api/'):
        _at = _normalised.rfind(_root)
        if _at != -1:
            return _normalised[_at + 1:]
    return _normalised.rsplit('/', 1)[-1]


class PackageTestConan(ConanFile):
    settings = "os", "compiler", "build_type", "arch"
    export_sources = "resources/*"
    generators = "CMakeDeps"

    conandata, metadata = None, None

    def init(self):
        conandata_path = Path(self.recipe_folder).parent / "conandata.yml"
        # explicit utf-8: read_text() defaults to the locale encoding (cp1252/cp936 on Windows)
        self.conandata = yaml.safe_load(conandata_path.read_text(encoding='utf-8'))
        metadata_path = Path(self.recipe_folder).parent / "metadata.json"
        self.metadata = yaml.safe_load(metadata_path.read_text(encoding='utf-8'))

    def build_requirements(self):
        # escape: HET_CMAKE_BUILD_REQUIRE=none also skips the test package's 45 MB cmake
        _override = (os.environ.get('HET_CMAKE_BUILD_REQUIRE') or '').strip()
        if _override.lower() == 'none':
            return
        self.build_requires(f"cmake/{_override or self.metadata.get('cmake_version')}")

    def requirements(self):
        self.requires(self.tested_reference_str)
        for req in self.conandata.get("requirements", []):
            self.requires(req)

    def _tests_enabled(self):
        """Whether this run builds and runs the suite; `-c user.het:run_tests=False` is the build-only leg."""
        return bool(self.metadata.get('trigger_tests')) and \
            self.conf.get('user.het:run_tests', default=True, check_type=bool)

    def generate(self):
        self._add_entries()
        build_env, run_env = VirtualBuildEnv(self), VirtualRunEnv(self)
        build_env.generate()
        run_env.generate(scope="run")

        tc = CMakeToolchain(self)
        lib_name = self.tested_reference_str.split("/")[0]
        tc.variables["LIB_NAME"] = lib_name
        tc.variables["CXX_DEPS"] = self._get_targets()
        tc.variables["TRIGGER_TESTS"] = self._tests_enabled()
        # coverage reads the gcda files the instrumented tests leave behind, so it rides the same switch
        tc.variables['ENABLE_COVERAGE'] = self._tests_enabled() and self.metadata.get('activate_code_coverage')
        tc.variables["MAIN_LIB_TARGET"] = [_a := self.metadata.get('target'),
                                           f'{lib_name}::{lib_name}' if _a == 'auto' else _a][-1]
        tc.variables["RESOURCES_PATH"] = os.path.join(self.build_folder, "resources").replace("\\", "/")
        tc.generate()

        src_folder = os.path.join(self.recipe_folder, "resources").replace("\\", "/")
        dst_folder = os.path.join(self.build_folder, "resources").replace("\\", "/")
        os.makedirs(dst_folder, exist_ok=True)
        copy(self, "*", src=src_folder, dst=dst_folder, keep_path=True)

    def _preparing_deps_links(self):
        _common, _c, _cpp, _infra = [self.metadata.get('dependencies').get(_) for _ in ['common', 'c', 'cpp', 'infra']]
        _c = {k: v if k not in _common.keys() else list(set(v).union(set(_common.get(k)))) for k, v in _c.items()}
        _cpp = {k: v if k not in _common.keys() else list(set(v).union(set(_common.get(k)))) for k, v in _cpp.items()}

        if not self.metadata.get('enable_python_bindings'):
            _infra.pop("pybind11", None)
        
        _infra_deps = [f"{k}@{' '.join(v)}" for k, v in _infra.items()]
        _c_deps = [f"{k}@{' '.join(v)}" for k, v in {**_common, **_c}.items()]
        _cpp_deps = [f"{k}@{' '.join(v)}" for k, v in {**_common, **_cpp}.items()]
        return list(set(_c_deps).union(set(_cpp_deps)).union(set(_infra_deps)))

    def _get_targets(self):
        _targets, _name = self.metadata.get('target'), self.metadata.get('name')
        if _targets is None or _targets == 'auto':
            _targets = [f'{_name}@{_name}::{_name}']
        else:
            _targets = [f'{_name}@{_targets}']
        _targets.extend(self._preparing_deps_links())
        return _targets

    def build(self):
        cmake = CMake(self)
        cmake.configure()
        cmake.build()

    def layout(self):
        cmake_layout(self)

    def configure(self):
        supported_compilers = {"gcc", "msvc", "clang", "apple-clang", }  # no support for 'Visual Studio' in Conan1.0
        compiler = getattr(self.settings, 'compiler')
        if compiler.__str__() in supported_compilers:
            _build_std = self.metadata.get("build_cppstd")
            _build_std = "17" if _build_std not in {"17", "20", "23"} else _build_std  # fallback
            compiler.cppstd = _build_std

    def test(self):

        # defensive logic for cross_building
        if cross_building(self):
            self.output.info("Cross-compilation detect. Skipping test execution.")
            return

        # set LLVM_PROFILE_FILE before main/ctest: the runtime picks its path at process start
        if self._is_llvm_coverage():
            self._prepare_llvm_profile_env()

        # scripting in test_package/main.cpp
        if can_run(self):
            cmd = os.path.join(self.cpp.build.bindirs[0], "main")
            self.run(cmd, env="conanrun")

        # test cases in test_pacakge/test/*.cpp
        if self._tests_enabled():
            # The report is written before a red suite is raised, never instead of raising it.
            suite_error = None
            try:
                self._run_ctest_suite()
            except Exception as err:  # noqa: BLE001 -- re-raised below, once the report exists
                suite_error = err

            coverage_error = None
            if self.metadata.get('activate_code_coverage'):
                try:
                    self._code_coverage_auto()
                except Exception as err:  # noqa: BLE001 -- reported, but after the suite's own verdict
                    coverage_error = err

            # The suite's failure is the verdict; raising the coverage error instead would hide which case failed.
            if suite_error is not None:
                raise suite_error from coverage_error
            if coverage_error is not None:
                raise coverage_error

    def _run_ctest_suite(self):
        # ctest prints a failed case only on request, and CMake.test has no `ctest_args`; Popen inherits os.environ.
        os.environ['CTEST_OUTPUT_ON_FAILURE'] = '1'
        try:
            if can_run(self):
                cmake = CMake(self)
                cmake.test()
        finally:
            target_folder = self.recipe_folder + sep + 'test' + sep + 'export'

            if self.metadata.get('saving_tests_log'):
                obj_folder = self.recipe_folder + sep + 'build'
                report = next(iter(_recursive_find(obj_folder, ['LastTest.log'])))
                with open(report, 'r', encoding='utf-8') as f:
                    _content = f.readlines()
                if not os.path.exists(target_folder):
                    os.mkdir(target_folder)
                with open(target_folder + sep + 'TestResult.log', 'w', encoding='utf-8') as f:
                    f.write(''.join(_content))
            elif os.path.exists(_f := target_folder + sep + 'TestResult.log'):
                os.remove(_f)

            self._remove_entries()

    def _compiler_name(self):
        return getattr(self.settings, 'compiler').__str__()

    def _is_llvm_coverage(self):
        """True when coverage is on and the toolchain writes LLVM raw profiles."""
        return bool(self.metadata.get('activate_code_coverage')) and \
            self._compiler_name() in ('clang', 'apple-clang')

    def _profraw_folder(self):
        return self.recipe_folder + sep + 'build' + sep + 'profraw'

    def _prepare_llvm_profile_env(self):
        """Send every LLVM raw profile to one folder, one file per process.

        The runtime only writes where it is told at process start, and two
        processes sharing a name would clobber each other, hence `%p`. Setting
        it in os.environ is enough: conan runs the binaries through
        ``Popen(..., shell=True)`` without an explicit ``env``, so children
        inherit the environment we set here.
        """
        _folder = self._profraw_folder()
        if os.path.exists(_folder):
            shutil.rmtree(_folder)
        os.makedirs(_folder)
        os.environ['LLVM_PROFILE_FILE'] = _folder + sep + 'het-%p.profraw'
        self.output.info(f"[coverage:llvm] LLVM_PROFILE_FILE={os.environ['LLVM_PROFILE_FILE']}")

    def _coverage_folder(self):
        """`<recipe>/test/export/coverage`, recreated empty.

        Its `coverage_report/` subfolder is the artifact downstream parses, so
        the layout is a contract (C3) -- both coverage paths must use this.
        """
        target_folder = self.recipe_folder + sep + 'test' + sep + 'export'
        coverage_folder = target_folder + sep + 'coverage'
        if not os.path.exists(target_folder):
            os.mkdir(target_folder)
        else:
            if os.path.exists(coverage_folder):
                shutil.rmtree(coverage_folder)
        os.mkdir(coverage_folder)
        return coverage_folder

    def _package_root(self):
        """Root of the tested package's cache entry, i.e. the parent of its `p/` folder.

        Asking `conan list` used to return whichever package was printed first, so a cross build of
        the same version (armv8 next to x86_64) pointed the coverage filter at the wrong tree and
        every record was excluded. The dependency itself knows its own folder.
        """
        _name = self.tested_reference_str.split('/')[0]
        try:
            _dep = self.dependencies[_name]
        except KeyError:
            _dep = None
        _pkg_folder = str(_dep.package_folder) if _dep and _dep.package_folder else ''
        if not _pkg_folder:
            raise RuntimeError(f'coverage: cannot resolve the package folder of {_name}; '
                               f'the coverage report cannot be scoped')
        return str(Path(_pkg_folder).parent)

    def _instrumented_binaries(self, build_folder):
        """The instrumented executables of this test package.

        `llvm-cov export` consumes binaries (the gcc path can consume
        .gcda/.gcno instead), and which executables exist is only known after
        the test package is configured, so derive them from ctest's own
        metadata rather than hard-coding names.

        Two sources are needed, and taking only the first is a trap:
          * every target gtest discovery ran, i.e. each `<target>[<n>]_*.cmake`
            -- this is the only way to reach `main`, which defines no TEST()
            and therefore never shows up in an `add_test` command, yet is the
            executable that links the library and so carries the coverage we
            actually want;
          * the `add_test` commands, which give the executables ctest runs.
        """
        _add_test = (r'add_test\s*\(\s*(?:\[=\[.*?\]=\]|"[^"]*"|\S+)\s+'
                     r'(?:\[=\[.*?\]=\]|"([^"]+)"|(\S+))')
        _discovered = r'([^/\\"]+)\[\d+\]_(?:tests|include)\.cmake'

        _paths, _names = [], set()
        for _cmake_file in sorted(Path(build_folder).rglob('CTestTestfile.cmake')) + \
                sorted(Path(build_folder).rglob('*_tests.cmake')):
            for _m in re.finditer(_discovered, _cmake_file.name):
                _names.add(_m.group(1))
            _text = _cmake_file.read_text(encoding='utf-8', errors='replace')
            for _m in re.finditer(_discovered, _text):
                _names.add(_m.group(1))
            for _m in re.finditer(_add_test, _text):
                _bin = _m.group(1) or _m.group(2)
                # Only binaries we built carry our profiles; foreign executables must never reach llvm-cov.
                if _bin and os.path.isfile(_bin) and _is_within(_bin, build_folder):
                    _paths.append(_bin)

        for _name in sorted(_names):
            _direct = os.path.join(build_folder, _name)   # single-config layout
            if os.path.isfile(_direct):
                _paths.append(_direct)
                continue
            _hits = [str(_) for _ in Path(build_folder).rglob(_name) if _.is_file()]
            _paths.extend(sorted(_hits))

        _found, _seen = [], set()
        for _p in _paths:
            _key = os.path.realpath(_p)      # add_test paths and the joined
            if _key not in _seen:            # ones may be different spellings
                _seen.add(_key)              # of the same executable
                _found.append(_p)
        return _found

    def _clang_coverage_tools(self):
        """Resolve llvm-profdata / llvm-cov plus the lcov renderer.

        Apple ships the LLVM tools with the Xcode command line tools but usually
        keeps them off PATH, so `xcrun -f` is the supported way to find them.
        Missing tools are reported loudly with the command that fixes it (C2).
        """
        _tools = {}
        for _name in ('llvm-profdata', 'llvm-cov'):
            _path = shutil.which(_name)
            if _path is None and sys.platform == 'darwin':
                _probe = subprocess.run(['xcrun', '-f', _name], capture_output=True, text=True)
                _path = _probe.stdout.strip() if _probe.returncode == 0 else None
            if not _path or not os.path.isfile(_path):
                raise RuntimeError(f'Code coverage: cannot locate `{_name}`, which is '
                                   f'required to turn LLVM raw profiles into a report. '
                                   f'On macOS install the Xcode command line tools '
                                   f'(`xcode-select --install`); elsewhere install an '
                                   f'LLVM toolchain that ships it.')
            _tools[_name] = _path
        for _name in ('lcov', 'genhtml'):
            if shutil.which(_name) is None:
                raise RuntimeError(f'Code coverage: cannot locate `{_name}`. '
                                   f'On macOS install it with `brew install lcov`.')
        return _tools['llvm-profdata'], _tools['llvm-cov']

    def _coverage_totals(self, info_file):
        """Merge the tracefile per source file, then sum, so every total is the sum of its own breakdown.

        ``_source_relpath`` is the identity of a file here: two records that normalise to the same path are the
        same source seen from two build roots and are merged line by line. A project that genuinely ships two
        different files at the same relative path under different roots would see them treated as one.
        """
        _lines, _meta, _files = {}, {}, set()
        _current = None
        with open(info_file, 'r', encoding='utf-8') as f:
            for _line in f:
                if _line.startswith('SF:'):
                    _current = _source_relpath(_line[3:].strip())
                    _files.add(_current)
                    _lines.setdefault(_current, {})
                    _meta.setdefault(_current, {})
                    continue
                if _current is None:
                    continue
                if _line.startswith('DA:'):
                    _number, _, _rest = _line[3:].strip().partition(',')
                    _count = _rest.partition(',')[0]
                    if _number.isdigit() and _count.isdigit():
                        _seen = _lines[_current]
                        _seen[int(_number)] = max(_seen.get(int(_number), 0), int(_count))
                    continue
                _key, _, _value = _line.strip().partition(':')
                if _key in ('FNF', 'FNH', 'BRF', 'BRH') and _value.isdigit():
                    # no line key to merge on: the larger record stands for the file, never the sum of two.
                    _meta[_current][_key] = max(_meta[_current].get(_key, 0), int(_value))
        _per_file = {_p: {'lines_hit': sum(1 for _c in _d.values() if _c > 0), 'lines_found': len(_d)}
                     for _p, _d in _lines.items()}
        _sum = lambda key: sum(_v.get(key, 0) for _v in _meta.values())
        _rate = lambda hit, found: round(100.0 * hit / found, 2) if found else 0.0
        _counts = [('lines', sum(_v['lines_hit'] for _v in _per_file.values()),
                    sum(_v['lines_found'] for _v in _per_file.values())),
                   ('functions', _sum('FNH'), _sum('FNF')),
                   ('branches', _sum('BRH'), _sum('BRF'))]
        _totals = {_what: {'hit': _hit, 'found': _found, 'percent': _rate(_hit, _found)}
                   for _what, _hit, _found in _counts}
        # the file set is what two platforms can compare; the rates cannot match (see _code_coverage_clang)
        _totals['files'] = sorted(_files)
        # Per-file counts, not percentages: averaging would weight a 3-line header like a 300-line source.
        _totals['per_file'] = {_p: _v for _p, _v in sorted(_per_file.items()) if _v['lines_found']}
        return _totals

    def _merge_tracefile(self, info_file, merged_file):
        """Rewrite ``SF:`` to one existing path per normalised key, then let lcov add the records.

        One header is compiled into the package's objects and again into the consumer's, so lcov reports it once
        per directory: ``lines`` counts the same source twice and every rate is computed on an inflated
        denominator. Two records of one file only merge once they carry the same path, and the path kept has to be
        one that exists -- ``genhtml`` reads the source through it to draw the annotated view. Counts are summed by
        lcov here; that does not affect the rate, which asks whether a line ran at all, not how often.
        """
        _raw, _candidates = [], {}
        with open(info_file, 'r', encoding='utf-8') as f:
            for _line in f:
                _raw.append(_line)
                if _line.startswith('SF:'):
                    _path = _line[3:].strip()
                    _candidates.setdefault(_source_relpath(_path), []).append(_path)

        _chosen = {_key: next((_p for _p in _paths if os.path.exists(_p)), _paths[0])
                   for _key, _paths in _candidates.items()}
        _rewritten = merged_file + '.rewritten'
        with open(_rewritten, 'w', encoding='utf-8') as f:
            for _line in _raw:
                if _line.startswith('SF:'):
                    _line = 'SF:' + _chosen[_source_relpath(_line[3:].strip())] + '\n'
                f.write(_line)

        subprocess.run(['lcov', '--ignore-errors', _lcov_ignore_errors(),
                        '-a', _rewritten, '--output-file', merged_file], check=True)
        os.remove(_rewritten)
        return merged_file

    def _render_html_report(self, coverage_folder, info_file, pkg_root):
        """Filter the tracefile down to OUR entry, merge duplicates, render the HTML, clean up.

        Shared verbatim by the gcc and clang paths so the two cannot drift: the artifact path produced here is
        what downstream parses.
        """
        # scope the report to this package's derived cache root; a non-matching filter makes lcov 2.x abort
        cmd2 = ['lcov', '--extract', info_file,
                pkg_root.replace(sep, '/') + '/*', '--output-file',
                os.path.join(coverage_folder, 'coverage_test.filtered.info')]
        subprocess.run(cmd2, check=True)

        # one tracefile for the HTML and the numbers: two artefacts with two rates is worse than either being wrong.
        _merged = self._merge_tracefile(os.path.join(coverage_folder, 'coverage_test.filtered.info'),
                                        os.path.join(coverage_folder, 'coverage_test.merged.info'))
        cmd3 = ['genhtml', _merged,
                '--output-directory', os.path.join(coverage_folder, 'coverage_report')]
        subprocess.run(cmd3, check=True)

        _totals = self._coverage_totals(_merged)

        # remove intermediate files
        for _f in os.listdir(coverage_folder):
            _full_name = coverage_folder + sep + _f
            if not os.path.isdir(_full_name):
                os.remove(_full_name)

        # the CI gate reads this, not genhtml's HTML: the tracefile is upstream, the HTML is a template
        with open(os.path.join(coverage_folder, 'coverage_summary.json'), 'w', encoding='utf-8') as f:
            json.dump(_totals, f, indent=2, sort_keys=True)
            f.write('\n')

    def _code_coverage_auto(self):
        compiler = self._compiler_name()
        if compiler == 'gcc':
            self._code_coverage_gcc()
        elif compiler in ('clang', 'apple-clang'):
            self._code_coverage_clang()
        else:
            raise NotImplementedError(f'Compiler {compiler} is not supported.')

    def _code_coverage_clang(self):
        """Apple/LLVM clang coverage: profraw -> profdata -> lcov -> html.

        GNU lcov/gcov cannot read LLVM's raw profiles, so the front of this
        pipeline necessarily differs from the gcc one. The tail -- filter to our
        cache entry, render, clean up -- is shared verbatim so the two can never
        drift, which is what keeps the artifact path a stable contract.

        Difference in the numbers is expected and is not a bug: llvm-cov counts
        lines and functions its own way, and it also reports inline code from
        headers that gcc/gcov attributes to the caller.
        """
        _profdata_bin, _cov_bin = self._clang_coverage_tools()
        coverage_folder = self._coverage_folder()

        _profraw = sorted(str(_) for _ in Path(self._profraw_folder()).glob('*.profraw'))
        if not _profraw:
            raise RuntimeError(f'Code coverage: no LLVM raw profile (*.profraw) found in '
                               f'{self._profraw_folder()}. The instrumented binaries did '
                               f'not run, or they did not inherit LLVM_PROFILE_FILE.')
        self.output.info(f'[coverage:llvm] merging {len(_profraw)} raw profile(s)')

        _profdata = coverage_folder + sep + 'coverage_test.profdata'
        cmd1 = [_profdata_bin, 'merge', '-sparse', *_profraw, '-o', _profdata]
        subprocess.run(cmd1, check=True)

        _bins = self._instrumented_binaries(self.build_folder)
        if not _bins:
            raise RuntimeError(f'Code coverage: ctest registered no runnable executable '
                               f'under {self.build_folder}. `llvm-cov export` needs the '
                               f'instrumented binaries themselves, not their object files.')
        self.output.info(f'[coverage:llvm] exporting {len(_bins)} instrumented binary(ies)')

        # llvm-cov export reports only the FIRST object, so export per binary and concatenate
        _info = os.path.join(coverage_folder, 'coverage_test.info')
        with open(_info, 'w', encoding='utf-8') as _handle:
            for _bin in _bins:
                cmd2 = [_cov_bin, 'export', f'-instr-profile={_profdata}',
                        '-format=lcov', _bin]
                subprocess.run(cmd2, check=True, stdout=_handle)

        self._render_html_report(coverage_folder, _info, self._package_root())

    def _code_coverage_gcc(self):

        # get conan build folder
        _pkg_root = self._package_root()
        _main_pkg_build_fd = _pkg_root + sep + 'b' + sep + 'build'

        # collect from both halves: a header-only module is instantiated by whoever includes it, often only a test.
        _gcda = [str(_) for _ in Path(_main_pkg_build_fd).rglob('*.gcda')]
        _gcda += [str(_) for _ in Path(self.build_folder).rglob('*.gcda')]
        _gcno = [_[:-4] + 'gcno' for _ in _gcda]

        coverage_folder = self._coverage_folder()

        for v1, v2 in zip(_gcda, _gcno):
            shutil.copy2(v1, coverage_folder + sep + _get_file_name(v1))
            shutil.copy2(v2, coverage_folder + sep + _get_file_name(v2))

        # `source` = deleted ucov_ copies; the line-range classes are named per lcov release (see _lcov_ignore_errors).
        cmd1 = ['lcov', '--directory', coverage_folder, '--capture', '--output-file',
                os.path.join(coverage_folder, 'coverage_test.info'), '--rc', 'geninfo_auto_base=1',
                '--ignore-errors', _lcov_ignore_errors()]
        subprocess.run(cmd1, check=True)

        self._render_html_report(coverage_folder,
                                 os.path.join(coverage_folder, 'coverage_test.info'),
                                 _pkg_root)

    def _add_entries(self):
        if self._tests_enabled():

            _f_stress = self.recipe_folder + sep + 'test' + sep + 'stress'
            if not os.path.exists(_m := _f_stress + sep + MAIN_CPP):
                with open(_m, 'w', encoding='utf-8') as f:
                    f.write(''.join(_entry_lists()))

            _f_unit = self.recipe_folder + sep + 'test' + sep + 'unit'
            if self.metadata.get('activate_code_coverage'):
                # never re-wrap a leftover ucov_ file: a failed run leaves them behind
                _files = [str(_) for _ in Path(_f_unit).rglob('*.cpp') if not _.name.startswith('ucov_')]
                _cache = [[_a := _.split(sep), (sep.join(_a[:-1]), _a[-1])][-1] for _ in _files]
                for _test_src, _test_ucov in zip(_files, _cache):
                    with open(_test_src, 'r', encoding='utf-8') as f:
                        _tmp = f.readlines()
                    _tmp.extend(_entry_lists()[1:])
                    with open(_test_ucov[0] + sep + 'ucov_' + _test_ucov[1], 'w', encoding='utf-8') as f:
                        f.write(''.join(_tmp))
            else:
                if not os.path.exists(_m := _f_unit + sep + MAIN_CPP):
                    with open(_m, 'w', encoding='utf-8') as f:
                        f.write(''.join(_entry_lists()))

    def _remove_entries(self):
        if not self._tests_enabled():
            return

        stress_main = self.recipe_folder + sep + 'test' + sep + 'stress' + sep + MAIN_CPP
        if os.path.exists(stress_main):
            os.remove(stress_main)

        _f_unit = self.recipe_folder + sep + 'test' + sep + 'unit'
        if self.metadata.get('activate_code_coverage'):
            self._remove_ucov_files(_f_unit)
        elif os.path.exists(_f3 := _f_unit + sep + MAIN_CPP):
            os.remove(_f3)

    def _remove_ucov_files(self, _f_unit):
        for _m in Path(_f_unit).rglob('*.cpp'):
            if str(_m).split(sep)[-1].startswith('ucov_'):
                os.remove(str(_m))


if __name__ == '__main__':
    _clear_test_build()
