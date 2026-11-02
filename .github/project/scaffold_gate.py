#!/usr/bin/env python3
"""The zero-modification contract, checked instead of remembered.

The promise was: this library enters a project on the *unmodified* v0.3.3 scaffold, so that the next scaffold
release can be merged without replaying private code by hand. A promise like that survives only if something
fails when it is broken -- and it is broken by the ordinary act of fixing a bug in place, which never looks
like a decision at the time.

What is checked is `scaffold.json`'s `checked` list: the build and CI plumbing upstream ships (`.github/misc/**`,
`.github/workflows/**`, `conanfile.py`, `conandata.yml`, `conan_provider.cmake`, the two `CMakeLists.txt`, the
two `test_package` recipes, `Doxyfile`, `docs/build.py`). Each file is compared byte-for-byte against the pinned
hash of the release it was taken from.

Three ways to fail, each catching a different mistake:

  * **a new divergence** -- the file changed and nobody wrote down why. This is the one that matters: it is how
    private logic gets into a shared file.
  * **a missing file** -- the scaffold ships it, this tree does not. A deleted contract is not a cleaner tree.
  * **a stale tolerance** -- `tolerated` lists a file that has since become identical. Left in place it hides
    the next divergence behind an excuse that already expired, so the entry has to go when the fork closes.
  * **a cosmetic fork** -- the tolerated file carries no functional change: strip comments and blank lines and
    the two files hold the same lines (possibly reordered). Such a fork buys nothing and costs a manual merge at
    every release, so the right move is to take upstream's bytes and propose the wording upstream instead.
  * **an exit plan that is only a sentence** -- a tolerance that claims an upstream patch must carry a `patch`
    path, and applying that patch to the pinned baseline must reproduce this tree's file byte for byte. Without
    this, "proposed upstream" is a claim nobody can check, and the patch rots the moment the file is edited by
    hand or the proposal stops matching what the project actually needs.

Why the fourth check exists: a byte-level gate can only say *that* a file differs, never *how much*. Two forks
survived review with reasons that read like real work -- one carried a 17-line comment rewrite, the other a pair
of code blocks moved up and down the file -- and both were pure cosmetics. Counting the functional lines is how
that was found, so it is what the gate now counts.

`tolerated` is a ratchet against `baseline_divergences`: closing a fork lowers both, opening one needs the
baseline raised in the same commit, which is a reviewable act rather than a silent edit.

    python3 .github/project/scaffold_gate.py            # check
    python3 .github/project/scaffold_gate.py --status    # print the fork list and the exit plans, always exit 0
    python3 .github/project/scaffold_gate.py --selftest  # prove the four failure modes still go red
"""
from __future__ import annotations

import argparse
import hashlib
import json
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = Path(__file__).resolve().with_name("scaffold.json")
BASELINE = Path(__file__).resolve().with_name("scaffold-baseline")


def _digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _functional_lines(text: str) -> list[str]:
    """The lines that do something, order preserved, whitespace and comments dropped.

    `#` covers yaml, python and cmake; `//` and `/*` cover the C-family files that can appear in `checked`.
    A line-level filter is enough for the question this answers: are two files the same program spelled
    differently? A change that matters survives the filter, and a change that does not is exactly what the
    gate is looking for.
    """
    out: list[str] = []
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith(("#", "//", "/*", "*")):
            continue
        out.append(line)
    return out


def _is_cosmetic(mine: str, upstream: str) -> bool:
    """True when the only difference is comments, blank lines, or the order of the code."""
    return sorted(_functional_lines(mine)) == sorted(_functional_lines(upstream))


def _selftest() -> int:
    """Prove each classifier still recognises the case it exists for.

    A check that cannot be shown to fail is a check that passes for the wrong reason. These are the four
    shapes the gate must separate, with real examples from this repository's own history.
    """
    cases = [
        # comment-only rewrite: the test_package/CMakeLists.txt fork, closed by taking upstream
        ("comment-only", "a = 1\n# old wording\n", "a = 1\n# new, longer wording\n# which spans two lines\n", True),
        # pure reordering: the root CMakeLists.txt fork, same two blocks moved
        ("reordered", "set(X 1)\nfunction(f)\n  a\nendfunction()\n", "function(f)\n  a\nendfunction()\nset(X 1)\n", True),
        # real work: the docs/build.py fork
        ("functional", "if a:\n  b\n", "if a and closes[i]:\n  b\n", False),
        # a value changed but nothing else: still functional
        ("one argument", 'CPP_ARGS: ""\n', 'CPP_ARGS: "--check-level=exhaustive"\n', False),
        # KNOWN LIMIT, asserted so it stays a limit and not a surprise: a docstring-only rewrite counts as
        # functional, because a docstring is a statement. Catching it needs a parser, not a line filter.
        ("docstring-only", 'def f():\n  """a"""\n  return 1\n', 'def f():\n  """b"""\n  return 1\n', False),
    ]
    failed = 0
    for name, upstream, mine, want_cosmetic in cases:
        got = _is_cosmetic(mine, upstream)
        ok = got == want_cosmetic
        failed += 0 if ok else 1
        print(f"[scaffold-selftest] {'ok  ' if ok else 'FAIL'} {name}: cosmetic={got} (want {want_cosmetic})")
    print(f"[scaffold-selftest] {'OK' if not failed else 'FAIL'}: {len(cases) - failed}/{len(cases)}")
    return 0 if not failed else 1


def _patch_reproduces(patch: Path, baseline: Path, actual: Path, rel: str) -> str:
    """Apply `patch` to the pinned baseline in a scratch tree; return '' on success or the reason it failed.

    The patch is the only artifact that says what this project needs upstream to change. Checking it here means the
    proposal cannot silently stop matching the code it was cut from -- which is the same failure the foundation log
    keeps recording in other forms: a document that describes yesterday.
    """
    if not patch.is_file():
        return f"the declared patch {patch.relative_to(ROOT)} does not exist"
    with tempfile.TemporaryDirectory() as scratch:
        root = Path(scratch)
        target = root / rel
        target.parent.mkdir(parents=True, exist_ok=True)
        shutil.copyfile(baseline, target)
        done = subprocess.run(["patch", "-p1", "--silent", "-i", str(patch)], cwd=root,
                              capture_output=True, text=True)
        if done.returncode != 0:
            return f"{patch.relative_to(ROOT)} does not apply to the pinned baseline: {done.stderr.strip() or done.stdout.strip()}"
        if target.read_bytes() != actual.read_bytes():
            return (f"{patch.relative_to(ROOT)} applies but does not reproduce {rel} -- the patch and the file have "
                    f"drifted apart, so the proposal no longer describes this fork")
    return ""


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--status", action="store_true", help="list the tolerated forks with their exit plans")
    ap.add_argument("--selftest", action="store_true", help="prove the cosmetic-fork classifier still separates")
    args = ap.parse_args()
    if args.selftest:
        return _selftest()

    data = json.loads(MANIFEST.read_text(encoding="utf-8"))
    pinned, tolerated = data["sha256"], data.get("tolerated") or {}
    problems: list[str] = []
    forked: list[str] = []
    identical = 0

    for rel in data["checked"]:
        path = ROOT / rel
        if not path.is_file():
            problems.append(f"{rel}: the scaffold ships this file and this tree does not have it")
            continue
        if _digest(path) == pinned[rel]:
            identical += 1
            if rel in tolerated:
                problems.append(f"{rel}: listed as tolerated but it is identical to v{data['pinned']['version']} "
                                f"-- remove the entry, a stale tolerance hides the next divergence")
            continue
        if rel in tolerated:
            forked.append(rel)
            # A tolerance has to buy something. The pinned bytes of every tolerated file are kept under
            # scaffold-baseline/ so "how much" is answerable without a network fetch of the old release.
            baseline = BASELINE / rel
            if not baseline.is_file():
                problems.append(f"{rel}: tolerated but {baseline.relative_to(ROOT)} is missing -- a tolerance "
                                f"cannot be audited without the bytes it diverged from")
            elif _is_cosmetic(path.read_text(encoding="utf-8", errors="replace"),
                              baseline.read_text(encoding="utf-8", errors="replace")):
                problems.append(f"{rel}: tolerated, but every functional line is unchanged (only comments, blank "
                                f"lines or the order differ) -- take v{data['pinned']['version']}'s bytes and "
                                f"propose the wording upstream instead of carrying a merge cost for it")
            else:
                _declared = tolerated[rel].get("patch")
                if not _declared:
                    problems.append(f"{rel}: tolerated without a `patch` -- an exit plan that is only a sentence "
                                    f"cannot be checked, and this gate exists because such sentences expire quietly")
                else:
                    _why = _patch_reproduces(ROOT / _declared, baseline, path, rel)
                    if _why:
                        problems.append(f"{rel}: {_why}")
        else:
            problems.append(f"{rel}: differs from v{data['pinned']['version']} and is not in `tolerated` -- "
                            f"a fork of the scaffold needs a reason and an exit plan, not a quiet edit")

    if len(tolerated) > data["baseline_divergences"]:
        problems.append(f"tolerated is {len(tolerated)} but the baseline is {data['baseline_divergences']}: the "
                        f"ratchet only goes down; raising it is a deliberate edit to scaffold.json")

    # A byte-check of the files upstream ships cannot see a directory that has quietly grown extras, and that is how
    # the private code got in the first time: eight project scripts lived in .github/misc/ while every one of the
    # files upstream ships was identical. So the file *set* is compared too.
    for rel, rule in (data.get("owned_dirs") or {}).items():
        directory = ROOT / rel
        allowed = set(rule["upstream_file_set"]) | set(rule["declared_additions"])
        present = {p.name for p in directory.iterdir() if p.is_file()} if directory.is_dir() else set()
        for extra in sorted(present - allowed):
            problems.append(f"{rel}/{extra}: not a file the scaffold ships and not a declared addition -- "
                            f"project logic in the scaffold's directory is the modification this contract forbids")
        for name, why in rule["declared_additions"].items():
            if name not in present:
                problems.append(f"{rel}/{name}: declared as a project addition but missing -- remove the declaration")

    if args.status:
        print(f"[scaffold] checked {len(data['checked'])} file(s) against {data['pinned']['project']} "
              f"v{data['pinned']['version']} ({data['pinned']['commit']}): {identical} identical, "
              f"{len(forked)} forked")
        for rel in forked:
            print(f"[scaffold]   {rel}")
            print(f"[scaffold]     why : {tolerated[rel]['why']}")
            print(f"[scaffold]     exit: {tolerated[rel]['exit']}")
        return 0

    for problem in problems:
        print(f"[scaffold] {problem}", file=sys.stderr, flush=True)
    if problems:
        print(f"[scaffold] FAIL ({len(problems)} problem(s); {len(forked)} tolerated fork(s))", flush=True)
        return 1
    print(f"[scaffold] OK ({len(data['checked'])} file(s) checked, {identical} identical to "
          f"v{data['pinned']['version']}, {len(forked)} tolerated fork(s) with an exit plan)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
