#!/usr/bin/env python3
"""Every example the documentation shows carries a build command, and that command works.

Why this gate exists
--------------------
The guide pages promise two things about the programs they print: that the listing **is** the
file (`literalinclude` of the example itself, so the two cannot drift) and that the example
*compiles*. The manual repeats the promise -- `check.py` there verifies each listing is a
subsequence of its `%% SOURCE:` file, so an example that does not build is a chapter that
cannot be trusted either.

Measured when this gate was written: **nothing in the repository compiled those files.** No
CMake target, no conan recipe, no CI leg, no gate. Nine of the ten carried a build command in
their header, and running those nine commands as written gave **four failures** -- two named a
source file that does not exist (`src/core/transform.cpp`, `src/core/protection.cpp`), two
omitted the sources their link needs, and the tenth (`servo-loop.cxx`) had no command at all.
One example did not compile at all: `-Werror=unused-but-set-variable` on a variable whose only
purpose was to be read.

That is how a wrong constant survives in a published example: an example nobody builds is prose
with error messages removed. A checker that ran its *own* command would have left all of those
instructions wrong, so this one runs **the command the page documents** -- the two cannot drift
for the same reason the listing cannot drift from the file.

What it checks
--------------
For each `docs/doxygen/dox/demos/*.cxx`:

  1. the header block carries a `g++` command (a demo without one fails: a reader who cannot
     reproduce it is being shown code, not given a program);
  2. running that command, verbatim and through a shell so its globs expand, produces an
     executable. The `-o` target is redirected into a temporary directory so a run leaves
     nothing behind and two runs cannot collide.

Usage
-----
  python3 .github/project/demo_gate.py             # check
  python3 .github/project/demo_gate.py --selftest  # prove it can fail
  python3 .github/project/demo_gate.py --verbose   # name every example that built

Exit codes: 0 all built, 1 at least one did not, 2 setup problem.
"""
from __future__ import annotations

import argparse
import pathlib
import re
import shlex
import subprocess
import sys
import tempfile

ROOT = pathlib.Path(__file__).resolve().parents[2]
DEMOS = ROOT / "docs" / "doxygen" / "dox" / "demos"

# The command lives in the file's own header, indented three spaces after `//`.
COMMAND_LINE = re.compile(r"^//\s{3}(?P<text>\S.*)$")


def documented_command(text: str) -> list[str] | None:
    """The `g++` invocation the example's header block carries, as shell words.

    It is contiguous: a run of `//   ` lines that contains `g++` and ends before the first line
    that is not `//   `. Joining them is what a reader does when pasting the block.
    """
    run: list[str] = []
    for raw in text.splitlines()[:24]:
        match = COMMAND_LINE.match(raw)
        if match:
            run.append(match.group("text").strip())
            continue
        if run:
            break
    if not run or not any(word.startswith("g++") for word in run):
        return None
    joined = " ".join(run)
    joined = re.sub(r"^\s*g\+\+", "g++", joined)
    try:
        return shlex.split(joined)
    except ValueError:
        return None


def build(example: pathlib.Path, workdir: pathlib.Path) -> tuple[bool, str]:
    """Run one example's documented command with its output redirected into `workdir`."""
    command = documented_command(example.read_text(encoding="utf-8"))
    if command is None:
        return False, "no g++ command in the header block"
    # The command has to name **this** file. A header copied from a neighbour builds the
    # neighbour and says nothing about the file it sits in -- which is what the first version of
    # this check did, and it reported a deliberately broken fixture as building fine.
    if example.name not in " ".join(command):
        return False, f"the documented command does not name {example.name}"
    if "-o" in command:
        command[command.index("-o") + 1] = str(workdir / example.stem)
    else:
        command += ["-o", str(workdir / example.stem)]
    # Through a shell, because the documented commands carry globs (`src/core/*.cpp`) and a
    # reader pastes them into one. The words are *not* re-quoted: quoting would stop the shell
    # from expanding those globs, which is how the first version of this gate reported ten of ten
    # examples as broken while the commands themselves were fine.
    done = subprocess.run(" ".join(command), shell=True, cwd=ROOT, capture_output=True, text=True)
    if done.returncode != 0:
        detail = (done.stderr or done.stdout).strip().splitlines()
        return False, " | ".join(detail[:2]) if detail else f"exit {done.returncode}"
    return True, ""


def check(verbose: bool) -> int:
    if not DEMOS.is_dir():
        print(f"[demo] FAIL: {DEMOS.relative_to(ROOT)} is missing", file=sys.stderr)
        return 2
    examples = sorted(DEMOS.glob("*.cxx"))
    if not examples:
        print(f"[demo] FAIL: no example under {DEMOS.relative_to(ROOT)}", file=sys.stderr)
        return 2

    failed: list[tuple[pathlib.Path, str]] = []
    with tempfile.TemporaryDirectory(prefix="foc-demo-gate-") as tmp:
        workdir = pathlib.Path(tmp)
        for example in examples:
            ok, detail = build(example, workdir)
            if ok:
                if verbose:
                    print(f"  {example.name} -> built", flush=True)
            else:
                failed.append((example, detail))
                print(f"--- {example.relative_to(ROOT)}", flush=True)
                print(detail, flush=True)

    if failed:
        print(f"[demo] FAIL ({len(failed)}/{len(examples)} example(s) did not build from their "
              f"own documented command)", flush=True)
        return 1
    print(f"[demo] OK ({len(examples)} example(s) built from the command each one documents)",
          flush=True)
    return 0


def selftest() -> int:
    """Three cases: a good example, a broken one, and one with no command at all."""
    failures = 0
    good = DEMOS / "servo-loop.cxx"
    if not good.is_file():
        print("[demo] selftest SKIP: docs/doxygen/dox/demos/servo-loop.cxx is missing",
              file=sys.stderr)
        return 2
    text = good.read_text(encoding="utf-8")

    with tempfile.TemporaryDirectory(prefix="foc-demo-selftest-") as tmp:
        fixture_dir = pathlib.Path(tmp) / "demos"
        fixture_dir.mkdir()
        workdir = pathlib.Path(tmp) / "out"
        workdir.mkdir()

        cases: list[tuple[str, str, bool]] = [
            ("the example as written", text, True),
            # A statement that cannot parse: the documented command must catch it.
            ("a body that does not compile", text + "\nint broken = ;\n", False),
            # No command at all: a reader who cannot reproduce it is being shown code, not given a
            # program, so this is a failure rather than a skip.
            ("a header with no command", re.sub(r"^//\s{3}g\+\+.*(\n//\s{3}.*)*", "", text,
                                                count=1, flags=re.MULTILINE), False),
        ]
        for index, (name, body, want) in enumerate(cases):
            example = fixture_dir / f"case{index}.cxx"
            # The header names its own file, so a fixture has to rename itself in the command --
            # otherwise the case would compile the original and report it as the fixture.
            body = body.replace(f"docs/doxygen/dox/demos/{good.name}", str(example))
            example.write_text(body, encoding="utf-8")
            ok, detail = build(example, workdir)
            good_run = ok == want
            failures += 0 if good_run else 1
            print(f"[demo-selftest] {'ok  ' if good_run else 'MISS'} {name}: "
                  f"{'built' if ok else 'failed'} (wanted {'built' if want else 'failed'})"
                  f"{'' if ok else ' -- ' + detail[:70]}")

    print(f"[demo-selftest] {len(cases) - failures} / {len(cases)} judgement(s) as expected")
    return 1 if failures else 0


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--selftest", action="store_true", help="prove the check can fail")
    parser.add_argument("--verbose", action="store_true", help="name every example that built")
    args = parser.parse_args(argv[1:])
    if args.selftest:
        return selftest()
    return check(args.verbose)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
