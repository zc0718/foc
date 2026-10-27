#!/usr/bin/env python3
"""PIL leg (Processor-In-the-Loop): run the S2 scenario on the *target ISA*, not just compile it.

What it proves
--------------
The cross-compile legs answer "does it build for the target". This script answers "does the target
binary compute the same thing", by running the very same scenario the host tests run, on an
emulated target CPU, and comparing the two.

Why qemu-user and not the board
------------------------------
PIL only needs the instruction set, not the peripherals. `qemu-user` gives the target ISA on the
host, deterministically, with no flashing step -- which is what makes it usable as a CI gate. It
cannot replace HIL: no real inverter, no real sampling, no timing.

Layout
------
  benchmark/pil/pil_main.cpp   the entry (borrows the target slot, see its header comment)
  this script                  build host + target, run both, compare

Exit codes
----------
  0  ok  (or: the target toolchain is absent and --require-target was not given)
  1  the comparison or the S2 acceptance failed
  2  --require-target was given but a tool is missing (CI uses this)

Usage
-----
    python3 benchmark/script/run_pil.py
    python3 benchmark/script/run_pil.py --require-target
    python3 benchmark/script/run_pil.py --cxx aarch64-linux-gnu-g++ --qemu qemu-aarch64-static
"""

import argparse
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
ENTRY = 'benchmark/pil/pil_main.cpp'

# The library's non-template translation units. Listed explicitly on purpose: version.cpp and
# motor_inst.cpp pull in the driver stubs and the C version symbol, which PIL does not need.
SOURCES = [
    'src/core/pi.cpp',
    'src/core/svpwm.cpp',
    'src/core/sm.cpp',
    'src/core/limits.cpp',
    'src/core/transforms.cpp',
    'src/core/observer/back_emf_observer.cpp',
    'src/core/feedforward.cpp',
    'src/core/motion_profile.cpp',
    'src/motor/motor_config.cpp',
    'src/motor/foc_loop.cpp',
]

METRICS = ['overshoot', 'steady_error_fraction', 'tail_peak_to_peak', 'settling_s', 'speed_final', 'peak_iq']

# PRD 8.4: S2 must not exceed 10 % overshoot and must settle within 1 % steady-state error.
S2_LIMITS = {'overshoot': 0.10, 'steady_error_fraction': 0.01}

# host vs target: the same algorithm on a different FPU. FMA contraction differs between x86-64
# (no FMA in the baseline instruction set) and aarch64 (FMA is baseline), so equality is not the
# right test -- agreement to a sane tolerance is.
RELATIVE_TOLERANCE = 1e-3
SETTLING_TOLERANCE_S = 1e-3


def cppstd():
    """metadata.json owns the language standard; never retype it here."""
    return str(json.loads((ROOT / 'metadata.json').read_text(encoding='utf-8'))['build_cppstd'])


def build(compiler, output, extra):
    command = [compiler, f'-std=c++{cppstd()}', '-O2', '-Iinclude', '-Itest_package/test/unit', ENTRY]
    command += SOURCES
    command += ['-o', str(output)]
    command += list(extra)
    print(f'[pil] build {output.name}: {compiler} ...', flush=True)
    done = subprocess.run(command, cwd=ROOT, capture_output=True, text=True, check=False)
    if done.returncode != 0:
        print(done.stdout)
        print(done.stderr, file=sys.stderr)
        raise SystemExit(f'[pil] build failed ({output.name})')


def run_binary(prefix):
    done = subprocess.run(prefix, cwd=ROOT, capture_output=True, text=True, check=False)
    if done.returncode != 0:
        print(done.stderr, file=sys.stderr)
        raise SystemExit(f'[pil] run failed: {" ".join(prefix)}')
    values = {}
    for line in done.stdout.splitlines():
        if '=' in line:
            key, _, raw = line.partition('=')
            try:
                values[key.strip()] = float(raw)
            except ValueError:
                continue
    missing = [name for name in METRICS if name not in values]
    if missing:
        raise SystemExit(f'[pil] the binary printed no value for: {", ".join(missing)}')
    return values


def check_s2(name, values):
    failures = 0
    for key, limit in S2_LIMITS.items():
        value = values[key]
        ok = value <= limit
        print(f'[pil] {name:<6} {key:<24} {value:.6g}  (limit {limit:g})  {"ok" if ok else "FAIL"}')
        failures += 0 if ok else 1
    return failures


def compare(host, target, name):
    failures = 0
    for key in METRICS:
        difference = abs(target[key] - host[key])
        if key == 'settling_s':
            ok = difference <= SETTLING_TOLERANCE_S
            detail = f'abs diff {difference:.3g} s (limit {SETTLING_TOLERANCE_S:g})'
        else:
            scale = max(abs(host[key]), 1.0)
            ok = (difference / scale) <= RELATIVE_TOLERANCE
            detail = f'rel diff {difference / scale:.3g} (limit {RELATIVE_TOLERANCE:g})'
        print(f'[pil] {name:<6} {key:<24} host {host[key]:.9g} target {target[key]:.9g}  {detail}  '
              f'{"ok" if ok else "FAIL"}')
        failures += 0 if ok else 1
    return failures


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--cxx', default='aarch64-linux-gnu-g++', help='target C++ compiler')
    parser.add_argument('--qemu', default=None, help='qemu-user binary (auto-detected)')
    parser.add_argument('--sysroot', default='/usr/aarch64-linux-gnu', help='target sysroot passed to qemu -L')
    parser.add_argument('--require-target', action='store_true',
                        help='a missing target toolchain is an error instead of a skip (CI uses this)')
    parser.add_argument('--keep', action='store_true', help='keep the build directory')
    args = parser.parse_args()

    qemu = args.qemu or next((candidate for candidate in ('qemu-aarch64', 'qemu-aarch64-static')
                              if shutil.which(candidate)), None)
    have_target = bool(shutil.which(args.cxx)) and (qemu is not None)

    with tempfile.TemporaryDirectory(prefix='foc-pil-') as workdir:
        work = Path(workdir)

        # The host leg always runs: it also proves the standalone build (no GTest, no Conan) works,
        # which is exactly what gets cross-compiled.
        host_binary = work / 'pil_host'
        build('c++', host_binary, [])
        host = run_binary([str(host_binary)])
        subject = check_s2('host', host)

        if not have_target:
            message = (f'target toolchain missing ({args.cxx}) or qemu-user not installed; '
                       f'host leg only')
            if args.require_target:
                print(f'[pil] error: {message}', file=sys.stderr)
                return 2
            print(f'[pil] skip: {message}')
            print('[pil] hint: sudo apt install g++-aarch64-linux-gnu qemu-user-static')
            return 1 if subject else 0

        target_binary = work / 'pil_aarch64'
        build(args.cxx, target_binary, [])
        target = run_binary([qemu, '-L', args.sysroot, str(target_binary)])
        subject += check_s2('target', target)
        subject += compare(host, target, 'diff')

        if args.keep:
            kept = ROOT / 'benchmark/pil/build'
            kept.mkdir(parents=True, exist_ok=True)
            for binary in (host_binary, target_binary):
                shutil.copy2(binary, kept / binary.name)
            print(f'[pil] kept binaries in {kept}')

    if subject:
        print(f'[pil] FAIL ({subject} check(s))')
        return 1
    print('[pil] OK: the target ISA reproduces the host result and meets S2')
    return 0


if __name__ == '__main__':
    sys.exit(main())
