#!/usr/bin/env python3
"""T1.5 strict compile gate: every public header must stand on its own.

Rules enforced (nothing is linked, so only declarations are checked):
  * `.hpp` -> `g++ -std=c++17 -fno-exceptions -fno-rtti -fsyntax-only`
  * `.h`   -> `gcc -std=c11   -pedantic -fsyntax-only`

Why a script and not a CMake target: `CMakeLists.txt` is a framework file; adding a build target
for a lint check would diverge from it, while this gate is a local/CI-invokable step.

Usage:
  python3 tools/strict_compile.py            # check every header under include/
  python3 tools/strict_compile.py -v         # print each compile command
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
INCLUDE = ROOT / "include"

CPP_FLAGS = ["-std=c++17", "-fno-exceptions", "-fno-rtti", "-Wall", "-Wextra", "-Werror"]
C_FLAGS = ["-std=c11", "-pedantic", "-Wall", "-Wextra", "-Werror"]


def compiler_for(suffix: str) -> str:
    return "g++" if suffix == ".hpp" else "gcc"


def header_flags(suffix: str) -> list[str]:
    return CPP_FLAGS if suffix == ".hpp" else C_FLAGS


def check_one(header: Path, verbose: bool) -> tuple[bool, str]:
    suffix = header.suffix
    rel = header.relative_to(INCLUDE).as_posix()
    with tempfile.TemporaryDirectory() as tmp:
        # The TU only includes the header, so a missing include or a non-self-contained header fails.
        tu = Path(tmp) / ("probe" + (".cpp" if suffix == ".hpp" else ".c"))
        tu.write_text(f'#include "{rel}"\n\nint main(void) {{ return 0; }}\n', encoding="utf-8")
        cmd = [compiler_for(suffix), *header_flags(suffix), "-I", str(INCLUDE), "-fsyntax-only", str(tu)]
        if verbose:
            print("  $", " ".join(cmd[:-1]), rel)
        proc = subprocess.run(cmd, capture_output=True, text=True)
        return proc.returncode == 0, proc.stderr.strip()


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("-v", "--verbose", action="store_true", help="print every compile command")
    args = parser.parse_args()

    headers = sorted(p for p in INCLUDE.rglob("*") if p.suffix in (".h", ".hpp"))
    if not headers:
        print("[strict] no headers found under include/")
        return 1

    failures = 0
    for header in headers:
        ok, message = check_one(header, args.verbose)
        status = "OK  " if ok else "FAIL"
        print(f"[strict] {status} {header.relative_to(ROOT).as_posix()}")
        if not ok:
            failures += 1
            print(message)

    print(f"[strict] {len(headers) - failures}/{len(headers)} header(s) compiled clean")
    return 0 if failures == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
