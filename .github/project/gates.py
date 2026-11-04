#!/usr/bin/env python3
"""This project's own gates: the checks that follow the library, not the template.

**Why this file exists separately.** The scaffold ships `.github/misc/gates.py` with `format`, `tidy` and `guard`.
Those files are the contract with upstream: after a re-scaffold they must be byte-identical to the version they
were taken from, or the next upstream release cannot be merged without replaying private code by hand. So the
project's gates live here and the scaffold's file is not touched. Nothing in this file is copied from upstream --
the helpers are imported from it, so a fix to `_die`, `_gated_files` or `_meta` reaches both.

**What is in here.** `license`, `strict`, `claims`, `docs`, `boundaries`, `capabilities`, `ci-policy`, `assume`,
`anchor`, `fmu`, `pil` -- checks whose *content* is this library's own (a claim table, a capability matrix, a
parameter registry). `gates.py <gate>` runs one; `gates.py all` runs all of them in the order below.

    python3 .github/project/gates.py all
"""
from __future__ import annotations

import argparse
import fnmatch
import importlib.util
import os
import re
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCAFFOLD_GATES = ROOT / '.github' / 'misc' / 'gates.py'
LICENSE_CONFIG = '.github/misc/.license-guard.txt'


def _scaffold():
    """The scaffold's gates.py as a module object, so its helpers are reused rather than re-implemented."""
    spec = importlib.util.spec_from_file_location('scaffold_gates', SCAFFOLD_GATES)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


_S = _scaffold()
GATE_DIRS = _S.GATE_DIRS
_die = _S._die
_rel = _S._rel
_meta = _S._meta
_gated_files = _S._gated_files
_empty = _S._empty
_standards = _S._standards
_scoped_name = _S._scoped_name
_require = _S._require
_version = _S._version


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




# Gates that rebuild or emulate something expensive and are deliberately kept off the cloud runners.
# The reason is runtime, not principle: `pil` builds the library twice and runs the target binary under
# qemu, which is minutes per push for evidence that only changes when the control loop does. It runs on
# a developer machine and before a release. `cmd_guard` fails the build if a workflow calls one of these,
# so the constraint is enforced rather than remembered -- the failure message names the reason and where
# the gate is supposed to run.
LOCAL_ONLY = {'pil', 'fmu', 'mutants', 'plantref', 'mcdc'}


def _ci_policy_violations():
    """Workflows must not call a LOCAL_ONLY gate. Comments are ignored: they are where the rule is explained."""
    problems = []
    workflows = ROOT / '.github' / 'workflows'
    if not workflows.is_dir():
        return problems
    for path in sorted(workflows.glob('*.y*ml')):
        for lineno, line in enumerate(path.read_text(encoding='utf-8').splitlines(), start=1):
            stripped = line.strip()
            if not stripped or stripped.startswith('#'):
                continue
            for gate in LOCAL_ONLY:
                if re.search(rf'gates\.py\s+{gate}\b', stripped):
                    problems.append(f'{path.name}:{lineno}: calls `gates.py {gate}`, which is local-only -- '
                                    f'it rebuilds and emulates for minutes per push; run it locally and '
                                    f'before a release instead')
    return problems


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
    args = [sys.executable, str(ROOT / '.github' / 'project' / 'claim_gate.py'), *explicit]
    done = subprocess.run(args, cwd=ROOT, text=True)
    return done.returncode


def cmd_capabilities(_explicit):
    """Every capability states its diagnostic channel, admission, boundary and invariant participation.
    Delegates to capability_gate.py so the rule has one implementation."""
    done = subprocess.run([sys.executable, str(ROOT / '.github' / 'project' / 'capability_gate.py')],
                          cwd=ROOT, text=True)
    return done.returncode


def cmd_docs(_explicit):
    """Doxygen block comments: aligned star column, consistent continuation indent, 120 columns,
    no mid-CJK breaks, one tag line per language group, no multi-line `///<`. Delegates to docfmt.py
    so the rule has one implementation. Then checks the numbers the documentation claims against the
    artefacts the build produced, because a number in prose with no owner drifts."""
    done = subprocess.run([sys.executable, str(ROOT / '.github' / 'project' / 'docfmt.py'), '--check'],
                          cwd=ROOT, text=True)
    if done.returncode != 0:
        return done.returncode
    # A list-table whose rows disagree with its header is dropped whole by Sphinx, so the page renders as if
    # those rows had never been written -- and `docs-build.yml` greps the build log for `ERROR: ` and fails.
    # This gate ran neither docutils nor any substitute, so 25 ragged rows sat behind a green light.
    shape = subprocess.run([sys.executable, str(ROOT / '.github' / 'project' / 'rst_table.py'), 'check'],
                           cwd=ROOT, text=True)
    if shape.returncode != 0:
        return shape.returncode
    numbers = subprocess.run([sys.executable, str(ROOT / '.github' / 'project' / 'number_sheet.py'), '--check'],
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
    # CONAN_HOME first: CI sets it explicitly, and assuming `~/.conan2` there would work by accident until
    # somebody moves the cache and the gate reports "no archive" instead of a real answer.
    conan_home = Path(os.environ.get('CONAN_HOME') or (Path.home() / '.conan2')).expanduser()
    archives = sorted(conan_home.glob('p/b/foc*/p/lib/libfoc_c.a'))
    reason = None
    if python is None:
        reason = 'no interpreter that can import fmpy (set FOC_FMPY_PYTHON to one)'
    elif not archives:
        reason = f'no libfoc_c.a in the Conan cache at {conan_home} (run `conan create .` first)'
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


def cmd_pil(_explicit, tolerant=False):
    """The target ISA must compute what the host computes, not merely compile for it.

    `benchmark/script/run_pil.py` builds the same scenario twice -- once for the host, once for aarch64 -- runs both,
    and compares the metrics. A cross-compile leg answers "does it build"; this answers "does it compute the same
    thing", which is the question a different `float` implementation, a different FMA contraction or a different
    `-ffast-math` can answer "no" to.

    Optional material, same convention as the FMU: a cross compiler and qemu-user. Absent, this prints one SKIPPED line
    and stops when it is one in a chain of gates, and fails when it was asked for by name -- a gate that skips quietly
    is not a gate. The measurement this gate would produce has therefore *not* been made on a machine without qemu, and
    the skip line says so rather than implying a pass."""
    script = ROOT / 'benchmark' / 'script' / 'run_pil.py'
    done = subprocess.run([sys.executable, str(script), '--require-target'], cwd=ROOT, text=True)
    if done.returncode == 2:
        reason = 'no aarch64 cross compiler or qemu-user on this machine'
        if tolerant:
            print(f'[pil] SKIPPED: {reason}; the target-ISA comparison was NOT run', flush=True)
            return 0
        _die(f'the PIL comparison cannot run: {reason}')
    return done.returncode


def cmd_assume(_explicit):
    """Every assumption the bench rests on states how it would be caught being wrong, and the gap count only falls.

    A silent assumption is invisible: nothing in a green run says "this held because a constant was chosen". So each
    entry carries the statement, what breaks `if_wrong`, and a falsifier that must be *usable* -- a test that exists or
    a gate that exists. `gaps` are assumptions nobody has a falsifier for yet; the count is a ratchet, so writing a
    new gap is a decision that needs the baseline raised rather than a line quietly added."""
    script = ROOT / '.github' / 'project' / 'assume_gate.py'
    return subprocess.run([sys.executable, str(script)], cwd=ROOT, text=True).returncode


def cmd_anchor(_explicit):
    """A parameter table may only witness the claims its own arithmetic can support.

    `anchors.json` records which machine each claim is anchored to; this gate recomputes saliency, the characteristic
    ratio and the MTPA gain from that table and refuses a machine that cannot show the effect at all. It also runs its
    own reject case, so a registry that has become uniformly agreeable cannot pass by having nothing to catch."""
    script = ROOT / '.github' / 'project' / 'anchor_check.py'
    return subprocess.run([sys.executable, str(script), '--selftest'], cwd=ROOT, text=True).returncode


def cmd_ci_policy(_explicit):
    """A workflow may not call a gate whose cost belongs on a developer machine.

    `pil` builds the library twice and runs the target binary under qemu; `fmu` loads the model in an FMI master.
    Both are minutes per push for evidence that only changes when the control loop or the interface does, so they run
    locally and before a release. The list lives here rather than in a comment because a comment cannot fail a build.
    The scaffold's `gates.py` is not touched: this checks *this* project's workflows, which is a project fact."""
    problems = _ci_policy_violations()
    for problem in problems:
        print(f'[ci-policy] {problem}', file=sys.stderr, flush=True)
    if problems:
        print(f'[ci-policy] FAIL ({len(problems)} violation(s))', flush=True)
        return 1
    workflows = sorted((ROOT / '.github' / 'workflows').glob('*.y*ml'))
    print(f'[ci-policy] OK ({len(workflows)} workflow(s), no local-only gate in CI, local-only: '
          f'{", ".join(sorted(LOCAL_ONLY))})', flush=True)
    return 0


PROJECT_GATES = ('license', 'strict', 'claims', 'docs', 'boundaries', 'capabilities', 'ci-policy', 'assume', 'anchor',
                 'scaffold', 'budget', 'clocks', 'observability', 'combinations', 'domain', 'mutants', 'plantref', 'fmu',
                 'pil', 'mcdc', 'blast', 'wired', 'smoke')


def cmd_observability(_explicit):
    """A new observation capability obliges a re-read of the evidence chain, and that obligation is data, not habit.

    The rule is the one this session ended on: when the foundation layer gains the ability to observe something that
    could not be observed before, go back and ask which existing claim rows it re-opens. As a paragraph, that rule is
    followed until someone forgets and then silently stops -- and nothing goes red, because the failure is an absence.
    So the upgrade, the rows it re-opened and the verdict for each are kept in `observability.json`, and this gate
    refuses a registry that is inconsistent with README or that lets an implementation-level leg promote anything.
    Delegates to `observability_gate.py` so the rule has one implementation.
    """
    done = subprocess.run([sys.executable, str(ROOT / '.github' / 'project' / 'observability_gate.py'), '--selftest'],
                          cwd=ROOT, text=True)
    return done.returncode


def cmd_blast(_explicit):
    """When the foundation moves, every conclusion that rested on it is re-read before the commit lands.

    The blast-radius column in `foundation-log.rst` has always been written by hand, and `claims --impact`
    asks the graph one edge deep, so the mechanism asked the right question and never insisted on an
    answer. `claim-basis.json` already holds the graph; what was missing is the half that recomputes the
    radius instead of trusting the entry, and refuses a diff that changes the foundation and walks away.

    The shape this prevents is A-110: a rig whose `config.dual_inductance` was never filled in, a member
    running on an all-zero configuration, a reading that was a constant -- and a conclusion drawn from it
    that looked exactly like a conclusion. Nothing went red, because the ledger was open and the account
    was empty. So the selftest runs nine shapes against the real checker first, that one included.
    """
    script = ROOT / '.github' / 'project' / 'blast_gate.py'
    for _args in (['--selftest'], []):
        _rc = subprocess.run([sys.executable, str(script), *_args], cwd=ROOT, text=True).returncode
        if _rc:
            return _rc
    return 0


def cmd_wired(_explicit):
    """Does the released package contain the capability, or only the header?

    A capability can be finished, documented, covered and mutation-tested and still be unreachable from
    the thing a user links against. This repository has produced that shape repeatedly -- the v8 audit
    named it once ("implemented is not shipped"), it was true, then it stopped being true for four
    modules, then it became true again for three others. A fact that turns over three times is not a
    fact anybody should be asked to remember.

    So each installed header declares which consumer it must reach (`motor-owned` -> something under
    `src/`, `diagnostic` -> a C ABI function, `contract` -> an implementation under `driver/`, `policy`
    -> an example, `reference` -> a test that exercises it), and the debt is a ratchet: a header that
    is `motor-owned` and unreached must be listed with the batch that will wire it, and the day it
    becomes reachable that note has to go. The selftest runs eleven shapes first, including both
    ratchets in the stale direction.
    """
    script = ROOT / '.github' / 'project' / 'wired_gate.py'
    for _args in (['--selftest'], []):
        _rc = subprocess.run([sys.executable, str(script), *_args], cwd=ROOT, text=True).returncode
        if _rc:
            return _rc
    return 0


def cmd_smoke(_explicit):
    """Before a long run, prove the pathway is alive -- with an artefact, not an exit code.

    FEMM cost five runs of about 44 minutes each and not one of them failed: a B-H curve attached
    before its material was silently discarded, an inert parameter carried the current direction, a
    shadowed `--tag` wrote to a file called `-.fem`, and after a reboot `wine` returned 0 while doing
    nothing. All of them looked like success, and a longer run of the same thing would have looked
    like it too. The rule that came out is one sentence -- an exit code and a job number are not
    evidence, only an artefact with a new timestamp is -- and this is where it has an implementation.

    Two halves, and they are deliberately split: the static half (every bench script has a decision
    about whether it can be smoked, and a claimed `--tier` is actually parsed) runs in the aggregate;
    tier 0 (stale binaries, a cross-built package in the cache) only runs on `--preflight`, because CI
    builds from clean and a check that cannot fail there is worse than no check. Tiers 1 and 2 belong
    to the scripts, exposed as `--tier`: a closed-form scenario in seconds, and the counterfactual
    where switching the thing off must move the output.
    """
    script = ROOT / '.github' / 'project' / 'smoke_gate.py'
    for _args in (['--selftest'], list(_explicit)):
        _rc = subprocess.run([sys.executable, str(script), *_args], cwd=ROOT, text=True).returncode
        if _rc:
            return _rc
    return 0


def cmd_domain(explicit):
    """The domain in which C7 and C8 hold, as a table the gate recomputes.
    Section 0 of the plan splits "high" into the capability existing, the number being credible, and the **domain**
    being known. The first two have gates. The third was prose in five places, which is why two L3 capabilities still
    read as "low": nobody could see at a glance that no machine in this repository is inside either domain.
    """
    from domain_gate import main as domain_main

    # The selftest runs first, every time: a gate that has never been red is a gate nobody has a reason to trust, and
    # this family proves it on each invocation rather than on request.
    # 自检每次都先跑：一个从未红过的门禁没有理由被信任，而这一族是把这件事做成每次调用都证明，而不是按需证明。
    if domain_main(['--selftest']) != 0:
        return 1
    return domain_main([])


def cmd_combinations(_explicit):
    """Two slot policies combined by a template parameter compile whatever they are, so the pairing needs a register.

    `Motor<Observer, ..., RefGenerator, ...>` takes both as types: MTPA against an observer whose flux model has no
    d-axis term compiles, runs, and produces a plausible angle that is wrong -- and the library's own measurement
    (`test_daxis_observer_limit.cpp`) says the safe d-axis current is a *rate*, not a constant. §C8 asks for the
    known boundaries to become a decidable refusal; this gate is the decidable half. Every (reference x observer)
    pair discovered in the headers must have a verdict, `conditional` must say what enforces it, `forbid` must
    name a case that can go red, and `unmeasured` must say what would have to be measured -- because "we did not
    think about it" and "we decided it is fine" must not look the same in a file.
    Delegates to `combinations_gate.py` so the rule has one implementation.
    """
    done = subprocess.run([sys.executable, str(ROOT / '.github' / 'project' / 'combinations_gate.py'), '--selftest'],
                          cwd=ROOT, text=True)
    return done.returncode


def cmd_mcdc(_explicit):
    """Structural coverage of the decisions that switch the bridge off, counted per decision rather than in aggregate.

    The plan asked for "safety-subset MC/DC >= 90 %" and the baseline refused it: six conditions in the whole subset,
    so one condition is worth 16.7 points -- a ratio nobody can act on -- while `report --show-mcdc-summary` counts
    only multi-condition decisions and the protection logic is almost entirely single comparisons. What the baseline
    did show was two conditions nobody had ever exercised, and neither was a missing assertion: `protection.hpp`'s
    `lost` term never saw an invalid flux, and `sm.cpp`'s range term once removed lets `kLegal[from][4]` read past the
    table and move the machine into a state that does not exist, with AddressSanitizer silent.

    So the criterion here is per decision: every multi-condition decision in the safety subset must have all of its
    independence pairs covered, and the subset must contain enough of them that a perfect score is not a perfect
    score for measuring nothing. Both halves are proved able to go red; the second one matters because the first is
    trivially satisfied by an empty subset. Local-only: the instrumented build takes about two minutes and needs
    clang 18, which is why it is not in CI.
    Runs the checker's own selftest first and the real measurement second: a selftest-only wiring would be a gate
    that reports 100 % while touching nothing, which is the failure this gate exists to refuse.
    Delegates to `mcdc_gate.py` so the rule has one implementation.
    """
    script = str(ROOT / '.github' / 'project' / 'mcdc_gate.py')
    selftest = subprocess.run([sys.executable, script, '--selftest'], cwd=ROOT, text=True)
    if selftest.returncode != 0:
        return selftest.returncode
    return subprocess.run([sys.executable, script], cwd=ROOT, text=True).returncode


def cmd_plantref(_explicit):
    """The plant is the instrument every other number is measured with, so it gets an instrument of its own.

    F-40 is why. The plant integrated its mechanical angle in float32, a step smaller than half an ulp was rounded away,
    and the axis stopped while its speed still read non-zero. The measurements that rested on it were stable, repeatable
    and wrong in one direction -- and no amount of repetition *inside the same suite* could see it, because the defect
    and the tests shared an instrument. What found it was a different one.

    So the check is deliberately foreign: numpy float64 instead of C++ float32/double, scipy's adaptive DOP853 instead
    of a fixed-step RK4, and the right-hand side re-derived from the header rather than linked from it. It runs open
    loop at a held voltage, because a full cascade would add controller detail whose disagreement is hard to attribute
    and the question here is narrower: does the plant integrate *its own stated equations*? A 0.1 % error in the
    reference's `psi_f` turns it red, which is the evidence that it can see anything at all.
    """
    import subprocess
    import sys

    script = ROOT / '.github' / 'project' / 'plant_reference.py'
    # `--selftest` perturbs a good trajectory three ways and requires each audit to notice, so the gate proves it can
    # go red in the same run that it reports green. Without it, "this instrument would have caught F-40" would be a
    # claim about the instrument rather than a demonstration of it.
    proc = subprocess.run([sys.executable, str(script), '--selftest'], capture_output=True, text=True, check=False)
    sys.stdout.write(proc.stdout)
    if proc.returncode != 0:
        sys.stdout.write(proc.stderr)
        print('[plantref] FAIL: the C++ plant disagrees with an independent float64 reference', flush=True)
        return 1
    return 0


def cmd_budget(_explicit):
    """An L3 claim rests on a sensitivity table, and a table is a snapshot until something keeps it current.

    The knobs are enumerated from the plant and the HAL rather than from a list I maintain, because the failure
    this prevents is exactly the one a maintained list cannot see: a new physical effect is added, the table keeps
    passing, and the claim goes on being published as if the new effect had been covered. `--selftest` runs first,
    so a classifier that has lost the ability to say "unclassified" cannot pass by having nothing to classify."""
    script = ROOT / '.github' / 'project' / 'budget_gate.py'
    for _args in (['--selftest'], []):
        _rc = subprocess.run([sys.executable, str(script), *_args], cwd=ROOT, text=True).returncode
        if _rc:
            return _rc
    return 0


def cmd_clocks(_explicit):
    """A duration that only grows is not a float, and this is not a style rule.

    `HomingRoutine` decided its timeout on `elapsed_s_ += dt_s` in float: the sum reached "512 s" after 446.664 s of
    real time (12.8 % fast) and then could not advance at all, so a 520 s recipe never timed out at all and the axis
    would have crept without end. The same line existed in four more modules with limits small enough to hide it. The
    rule is mechanical so that the next timer cannot reintroduce it: every `float` accumulated with a `dt` term is a
    defect unless it is bounded by construction and named on the allow list with its bound written down.
    `--selftest` runs first, so a classifier that no longer finds anything cannot pass by finding nothing."""
    script = ROOT / '.github' / 'project' / 'clock_gate.py'
    for _args in (['--selftest'], []):
        _rc = subprocess.run([sys.executable, str(script), *_args], cwd=ROOT, text=True).returncode
        if _rc:
            return _rc
    return 0


def cmd_mutants(_explicit):
    """Would a wrong line be noticed? Coverage cannot answer that; a mutation score can.

    Builds each core module's tests against one deliberately broken version of the module at a time and counts how many
    the tests reject. Needs gtest from the conan cache, so it is local-only: the point is a number a human reads when
    the tests change, not a per-push cost. `--check` compares against the recorded baseline and fails on a drop."""
    script = ROOT / '.github' / 'project' / 'mutate.py'
    return subprocess.run([sys.executable, str(script), '--check'], cwd=ROOT, text=True).returncode


def cmd_scaffold(_explicit):
    """The zero-modification contract: the scaffold's build and CI files are byte-identical to the pinned release.

    Read as the cost of the *next* upstream release. A private edit to `.github/misc/gates.py` is invisible today and
    turns every future re-scaffold into a manual merge of code that upstream has never seen. `scaffold.json` pins the
    bytes and lists the honest exceptions with an exit plan; `--status` prints that list for a human."""
    script = ROOT / '.github' / 'project' / 'scaffold_gate.py'
    # Both halves, because they fail for different reasons: `--selftest` proves the classifier can still tell a
    # cosmetic fork from real work, and the plain run proves this tree satisfies the contract today. A gate that
    # only ever runs the second one can pass by having lost the ability to fail.
    for _args in (['--selftest'], []):
        _rc = subprocess.run([sys.executable, str(script), *_args], cwd=ROOT, text=True).returncode
        if _rc:
            return _rc
    return 0


def main():
    parser = argparse.ArgumentParser(description="This project's gates (the scaffold's live in .github/misc/gates.py).")
    sub = parser.add_subparsers(dest='gate', required=True)
    helps = {
        'license': 'copyleft / non-free licence markers, scoped rules from .github/misc/.license-guard.txt',
        'strict': 'every public header compiles standalone with -fno-exceptions -fno-rtti (C: -pedantic)',
        'claims': 'every installed header has a README claim row with a level and existing evidence; '
                  '`claims --impact <path|name>` lists what rests on it',
        'docs': 'Doxygen blocks: star alignment, continuation indent, 120 columns, no mid-CJK breaks',
        'boundaries': 'every statement on the boundaries page names a test case that exists',
        'capabilities': 'every capability states its diagnostic channel, admission, boundary and invariant participation',
        'ci-policy': 'no workflow calls a gate whose cost belongs on a developer machine (pil, fmu)',
        'assume': 'every assumption states how it would be caught being wrong; the unproven-gap count only falls',
        'anchor': 'every parameter table can support the claims anchored to it (saliency, leverage, magnitude)',
        'scaffold': 'the scaffold\'s build and CI files are byte-identical to the pinned release (zero-modification)',
        'budget': 'every injectable model error source is either covered by a sensitivity table or excluded with a '
                  'reason -- so a new knob cannot arrive silently',
        'clocks': 'no float accumulates a duration that only grows: a timeout is a core::ElapsedClock (F-105)',
        'mutants': 'mutation score per core module: would a wrong operator or constant be noticed by its tests?',
        'plantref': 'the SIL plant reproduces its own stated equations under an independent float64/scipy solver',
        'observability': 'every new observation capability has re-read the evidence chain, and a port never promotes',
        'combinations': 'every (reference x observer) pairing has a verdict, and a forbidden one names a case that can go red',
        'domain': 'the domain in which C7 and C8 hold is a table the gate recomputes, not prose in five places',
        'mcdc': 'every multi-condition decision in the safety subset has all of its independence pairs covered, and the '
                'subset is big enough for that to mean something',
        'blast': 'a change to the foundation is settled: every claim row that rests on it carries a verdict, and a '
                 'refuted claim is not left holding up a dependent',
        'wired': 'every installed header reaches the consumer its class promises (motor-owned / diagnostic / contract / '
                 'policy / reference), so "implemented" and "shipped" stop being the same word',
        'smoke': 'every bench script has a decision about whether it can be smoked; `--preflight` also checks tier 0 '
                 '(stale binaries, a cross-built package in the cache)',
        'fmu': 'build the FMU and load it in a standard FMI master (FMPy); SKIPPED when its material is absent',
        'pil': 'run the speed-step scenario on the target ISA under qemu and compare with the host; SKIPPED when its '
               'material is absent',
        'all': ' + '.join(PROJECT_GATES),
    }
    for name, text in helps.items():
        leaf = sub.add_parser(name, help=text)
        leaf.add_argument('files', nargs='*', help='check only these files (default: every gated file)')
        if name == 'claims':
            leaf.add_argument('--impact', metavar='TERM',
                              help='list the claims that rest on this path or identifier, with the cases to re-run')
        if name == 'smoke':
            leaf.add_argument('--preflight', action='store_true',
                              help='also run tier 0: stale binaries and a cross-built package in the cache')
    args = parser.parse_args()

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
    if args.gate == 'capabilities':
        return cmd_capabilities(args.files)
    if args.gate == 'ci-policy':
        return cmd_ci_policy(args.files)
    if args.gate == 'assume':
        return cmd_assume(args.files)
    if args.gate == 'anchor':
        return cmd_anchor(args.files)
    if args.gate == 'scaffold':
        return cmd_scaffold(args.files)
    if args.gate == 'budget':
        return cmd_budget(args.files)
    if args.gate == 'clocks':
        return cmd_clocks(args.files)
    if args.gate == 'mutants':
        return cmd_mutants(args.files)
    if args.gate == 'fmu':
        return cmd_fmu(args.files)
    if args.gate == 'pil':
        return cmd_pil(args.files)
    # Without this branch `plantref` fell through to the aggregate below and silently ran every gate: a single-gate
    # invocation took four minutes and reported "all 15", which is the sort of failure that gets read as success.
    # 没有这一支，`plantref` 会落到下面的聚合分支，静默地跑完全部门禁：单跑一条要四分钟，还打印"all 15"——这种故障
    # 最容易被当成成功读过去。
    if args.gate == 'plantref':
        return cmd_plantref(args.files)
    if args.gate == 'observability':
        return cmd_observability(args.files)
    if args.gate == 'combinations':
        return cmd_combinations(args.files)
    if args.gate == 'mcdc':
        return cmd_mcdc(args.files)
    if args.gate == 'blast':
        return cmd_blast(args.files)
    if args.gate == 'wired':
        return cmd_wired(args.files)
    if args.gate == 'smoke':
        return cmd_smoke(['--preflight'] if getattr(args, 'preflight', False) else [])
    if args.gate == 'domain':
        return cmd_domain([])
    # The chain above is the only thing standing between a single-gate invocation and the aggregate below, and it
    # has now been forgotten twice (`plantref` before, `combinations` and `mcdc` this time). A missing branch does
    # not fail: it runs *every* gate and prints "all N", which reads as success. So the fall-through is refused
    # explicitly -- an unknown name is an error, not a request for everything.
    # 上面那条链是单跑一条门禁与下面的聚合分支之间唯一的屏障，而它已经被忘掉两次（先是 `plantref`，这次是
    # `combinations` 与 `mcdc`）。漏一支不会报错：它会跑**所有**门禁并打印 "all N"，读起来像成功。所以这里显式
    # 拒绝落空——不认识的名字是错误，不是"要全部"。
    if args.gate != 'all':
        print(f'[project-gates] unknown gate {args.gate!r}: no branch handles it, so nothing was run', flush=True)
        return 1
    failed = [name for name in PROJECT_GATES
              if {'license': cmd_license, 'strict': cmd_strict, 'claims': lambda _f: cmd_claims([]),
                  'docs': cmd_docs, 'boundaries': cmd_boundaries, 'capabilities': cmd_capabilities,
                  'ci-policy': cmd_ci_policy, 'assume': cmd_assume, 'anchor': cmd_anchor,
                  'scaffold': cmd_scaffold, 'budget': cmd_budget, 'clocks': cmd_clocks, 'mutants': cmd_mutants,
                  'plantref': cmd_plantref, 'observability': cmd_observability, 'combinations': cmd_combinations,
                  'domain': cmd_domain,
                  'mcdc': cmd_mcdc,
                  'blast': cmd_blast,
                  'wired': cmd_wired,
                  'smoke': cmd_smoke,
                  'fmu': lambda _f: cmd_fmu([], tolerant=True),
                  'pil': lambda _f: cmd_pil([], tolerant=True)}[name]([]) ]
    if failed:
        print(f'[project-gates] FAIL: {", ".join(failed)}', flush=True)
        return 1
    print(f'[project-gates] OK: all {len(PROJECT_GATES)} project gate(s)', flush=True)
    return 0


if __name__ == '__main__':
    sys.exit(main())
