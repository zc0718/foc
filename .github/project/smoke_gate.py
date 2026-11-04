#!/usr/bin/env python3
"""Before a long run, prove the pathway is alive -- and prove it with an artefact, not an exit code.

FEMM cost this project five runs of about 44 minutes each, and not one of them failed: the B-H curve
was attached before the material existed and was silently discarded, the current direction rode on an
inert parameter, a variable shadowed `--tag` so the save went to a file called `-.fem`, and after a
reboot `wine` returned 0 while doing nothing at all. Every one of those looked like success, and a
longer run of the same thing would have looked like it too.

The rule that came out of it is one sentence: **an exit code and a job number are not evidence; only an
artefact with a new timestamp is.** This gate is where that sentence either has an implementation or
does not.

  * `smoke` (static) -- every script under `benchmark/script/` is registered with a decision about
    whether it can be smoked, a script that claims `--tier` actually parses it, and the number of
    scripts that cannot be smoked yet is compared against `no_tier_baseline`, which may only fall.
    A new long scan arriving without a decision fails here.
  * `smoke --preflight` -- tier 0, the cheap state check: the test binaries must be newer than the
    headers and sources they were built from, and the conan cache must hold no cross-built `foc`
    package. Deliberately not part of the aggregate `all`: CI builds from clean, so it has nothing
    stale to find, and a gate that cannot fail there is worse than no gate.

Tiers 1 (a closed-form scenario in seconds) and 2 (switch the thing off, the output must move) belong
to the scripts themselves, exposed as `--tier`. This gate only insists that the decision exists.

The boundary, stated rather than implied: passing the preflight says every link produced something.
It says nothing about whether the numbers are right. Those are two different claims and mixing them is
how the FEMM sessions were spent.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SCRIPTS = ROOT / "benchmark" / "script"
BUILD = ROOT / "test_package" / "build"
REGISTER = Path(__file__).resolve().with_name("smoke.json")
CONAN_CACHE = Path.home() / ".conan2" / "p" / "b"


def registered_scripts() -> list[str]:
    return sorted(path.relative_to(ROOT).as_posix() for path in SCRIPTS.glob("*.py"))


def check(register: dict, present: list[str], read) -> list[str]:
    """Every problem with the registry. `read` returns a script's text, injected for the selftest."""
    problems: list[str] = []
    scans: dict = register.get("scans", {}) or {}
    baseline = int(register.get("no_tier_baseline", 0))

    for rel in present:
        if rel not in scans:
            problems.append(f"`{rel}` is a bench script with no smoke decision -- register it with a "
                            f"`tier` (or `null` and a reason), because the failure this prevents is a "
                            f"long scan arriving quietly")
    for rel in sorted(scans):
        if rel not in present:
            problems.append(f"`{rel}` is registered but no longer exists -- a rotted entry hides the "
                            f"next real one")
            continue
        spec = scans[rel] or {}
        if not str(spec.get("why") or "").strip():
            problems.append(f"`{rel}`: no `why` -- a registry entry that does not say what costs the "
                            f"time cannot be argued with")
        tier = spec.get("tier")
        if tier is not None:
            text = read(rel)
            if "--tier" not in text:
                problems.append(f"`{rel}` claims tier `{tier}` but does not parse `--tier` -- the claim "
                                f"is not implemented")

    unsmoked = [rel for rel, spec in scans.items() if (spec or {}).get("tier") is None]
    if len(unsmoked) > baseline:
        problems.append(f"{len(unsmoked)} script(s) cannot be smoked against a baseline of {baseline} "
                        f"-- the ratchet only falls; add `--tier` to the one you are about to run")
    return problems


def freshness() -> list[str]:
    """Tier 0: are the test binaries newer than the code they were built from?

    mtime alone cannot answer this. `git checkout` and every formatter in the world rewrite a file
    without changing a byte, which sets its mtime and makes every binary look stale -- the first
    version of this check reported 792 stale binaries on a clean tree for exactly that reason. So the
    question is asked the other way round: *is there anything uncommitted under `include/` or `src/`?*
    If not, the binaries match the tree no matter what the clocks say; if there is, they must be newer
    than the newest uncommitted source. git is the content oracle here, and the timestamp is only used
    inside the set git already narrowed.
    """
    problems: list[str] = []
    binaries = [path for path in BUILD.rglob("ucov_*") if path.is_file()]
    if not binaries:
        print("[smoke] preflight: no `ucov_*` binary found -- build first, then smoke "
              "(`conan create . -s build_type=Debug --build=missing`)")
        return problems
    proc = subprocess.run(["git", "-C", str(ROOT), "status", "--porcelain", "--", "include", "src"],
                          capture_output=True, text=True)
    if proc.returncode != 0:
        print("[smoke] preflight: git cannot report the working tree -- freshness not checked")
        return problems
    dirty = [line[3:].strip() for line in proc.stdout.splitlines() if line.strip()]
    print(f"[smoke] preflight: {len(binaries)} binary/binary-dir entry(ies), "
          f"{len(dirty)} uncommitted source(s)")
    if not dirty:
        return problems
    newest = max((ROOT / rel for rel in dirty if (ROOT / rel).is_file()),
                 key=lambda path: path.stat().st_mtime)
    stale = [path for path in binaries if path.stat().st_mtime < newest.stat().st_mtime]
    if stale:
        problems.append(f"{len(stale)} test binary(ies) are older than the uncommitted "
                        f"`{newest.relative_to(ROOT).as_posix()}` -- any number they produce belongs "
                        f"to the previous build")
    return problems


def cross_built() -> list[str]:
    """Tier 0: a leftover cross package poisons every later link, with an unreadable error."""
    problems: list[str] = []
    if not CONAN_CACHE.is_dir():
        return problems
    archives = sorted(CONAN_CACHE.glob("foc*/p/lib/*.a"))
    foreign = []
    for archive in archives:
        # `readelf` is localised on this box; the machine name is only stable under LC_ALL=C.
        proc = subprocess.run(["readelf", "-h", str(archive)], capture_output=True, text=True,
                              env={"LC_ALL": "C", "PATH": "/usr/bin:/bin"})
        if "AArch64" in proc.stdout:
            foreign.append(archive)
    print(f"[smoke] preflight: {len(archives)} archive(s) in the conan cache, "
          f"{len(foreign)} cross-built")
    if foreign:
        problems.append(f"{len(foreign)} cross-built archive(s) in the conan cache "
                        f"(e.g. `{foreign[0].name}`) -- run `conan remove \"foc/*\" --confirm` and "
                        f"rebuild, or the next link takes whichever archive prints first")
    return problems


def selftest() -> bool:
    """Five shapes the registry check must classify correctly."""
    texts = {"benchmark/script/good.py": "p.add_argument('--tier')",
             "benchmark/script/liar.py": "p.add_argument('--mode')"}
    present = ["benchmark/script/good.py", "benchmark/script/liar.py"]
    read = lambda rel: texts[rel]  # noqa: E731

    def entry(tier, why="takes minutes"):
        return {"why": why, "tier": tier}

    clean = {"no_tier_baseline": 1,
             "scans": {"benchmark/script/good.py": entry("--tier"), "benchmark/script/liar.py": entry(None)}}
    cases = [
        ("a clean registry", check(clean, present, read), False),
        ("a script with no decision", check({"no_tier_baseline": 1, "scans": {
            "benchmark/script/good.py": entry("--tier")}}, present, read), True),
        ("a registered script that is gone", check({"no_tier_baseline": 1, "scans": {
            **clean["scans"], "benchmark/script/rotted.py": entry(None)}}, present, read), True),
        ("a tier claim that is not implemented", check({"no_tier_baseline": 1, "scans": {
            "benchmark/script/good.py": entry("--tier"), "benchmark/script/liar.py": entry("--tier")}},
            present, read), True),
        ("the unsmoked count above its baseline", check({"no_tier_baseline": 0, "scans": clean["scans"]},
                                                        present, read), True),
        ("an entry with no reason", check({"no_tier_baseline": 1, "scans": {
            "benchmark/script/good.py": entry("--tier"), "benchmark/script/liar.py": entry(None, why="")}},
            present, read), True),
    ]
    ok = True
    for label, problems, want_red in cases:
        red = bool(problems)
        mark = "ok" if red == want_red else "WRONG"
        print(f"[smoke-selftest] {label:44s} -> {len(problems)} problem(s), want "
              f"{'red' if want_red else 'green'} [{mark}]")
        if red != want_red:
            ok = False
            for problem in problems:
                print(f"                  {problem}")
    return ok


def main(argv: list[str]) -> int:
    if "--selftest" in argv:
        if not selftest():
            print("[smoke] FAIL (selftest)", file=sys.stderr)
            return 1

    register = json.loads(REGISTER.read_text(encoding="utf-8")) if REGISTER.is_file() else {}
    present = registered_scripts()
    read = lambda rel: (ROOT / rel).read_text(encoding="utf-8", errors="replace")  # noqa: E731
    problems = check(register, present, read)

    scans = register.get("scans", {}) or {}
    unsmoked = [rel for rel, spec in scans.items() if (spec or {}).get("tier") is None]
    print(f"[smoke] {len(present)} bench script(s), {len(scans)} registered, {len(unsmoked)} without a "
          f"smoke yet (baseline {register.get('no_tier_baseline', 0)})")

    if "--preflight" in argv:
        problems.extend(freshness())
        problems.extend(cross_built())
    else:
        print("[smoke] tier 0 not run (pass `--preflight` before starting a long scan)")

    if problems:
        for problem in problems:
            print(f"[smoke] {problem}", file=sys.stderr)
        print(f"[smoke] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        return 1
    print("[smoke] OK (every bench script has a smoke decision; tier 0 is " +
          ("clean)" if "--preflight" in argv else "not inspected)"))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
