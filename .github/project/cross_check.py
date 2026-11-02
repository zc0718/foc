#!/usr/bin/env python3
"""Run the scaffold's cross-compile check under a C locale, because this box's binutils speaks Chinese.

The scaffold's `.github/misc/cross_compile_check.py` parses `readelf -h` output with

    re.search(r'^\\s*Machine:\\s*(.+?)\\s*$', header, re.M)

On a machine with a non-English locale, GNU binutils translates its own output: this one prints `ELF 头：`,
`类 别:`, `机器:` instead of `ELF Header:`, `Class:`, `Machine:`. The regex then matches nothing, `machine` comes out
empty, and the script reports

    libfoc_cpp.a: expected Machine AArch64, got

for an archive that is perfectly correct (`file` says `ELF 64-bit LSB relocatable, ARM aarch64`). That cost a real
investigation: the check was believed to be red, and the archive was blamed before the instrument was.

Two things follow, and only the second is a code change:

1. The scaffold file stays untouched. It is upstream's, it is correct for CI (GitHub runners are English), and the
   project's rule is that upstream files are not edited -- so the fix cannot be "add LC_ALL to the regex call".
2. The local invocation gets one entry point that cannot be got wrong. `.github/project/` is the only place this
   project may add files, so the wrapper lives here.

    python3 .github/project/cross_check.py --target arm-linux-a53 --out /tmp/xcheck

Any argument is passed through, so this is a drop-in for the scaffold script. The environment is *forced* rather
than defaulted, because a developer with `LC_ALL=zh_CN.UTF-8` exported should get the same numbers as CI without
having to remember why.
"""
from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCAFFOLD = ROOT / ".github" / "misc" / "cross_compile_check.py"


def main() -> int:
    if not SCAFFOLD.is_file():
        print(f"[cross-check] {SCAFFOLD.relative_to(ROOT)} is missing -- the scaffold ships it; "
              f"`gates.py scaffold` would be red too", file=sys.stderr)
        return 1
    env = dict(os.environ)
    # Forced, not defaulted: the point is that the answer does not depend on the caller's shell.
    env["LC_ALL"] = "C"
    env["LANG"] = "C"
    return subprocess.run([sys.executable, str(SCAFFOLD), *sys.argv[1:]], cwd=ROOT, env=env).returncode


if __name__ == "__main__":
    sys.exit(main())
