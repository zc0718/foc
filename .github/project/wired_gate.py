#!/usr/bin/env python3
"""Does the released package contain the capability, or only the header?

A capability can be finished, documented, covered, mutation-tested, and still be unreachable from the
thing a user actually links against. This repository has produced that shape repeatedly and each time
it looked like progress: `stall_start.hpp` is L3 with a measured 0.0300 N m hand-over and nothing in
`src/` includes it; `hall.hpp` decodes three pins with a measured mount-error table and has no input
to read them from; `machine_map.hpp` interpolates parameters and has no example of it being assembled
into a `Motor`. The v8 audit said this in one line -- implemented is not shipped -- and the line was
true, then stopped being true for four modules, then became true again for three others. A fact that
changes direction three times is not a fact anybody should be asked to remember; it is a gate.

Four classes, because they fail differently:

  * `motor-owned`  the library's own loop must contain it, so it must be reachable from a production
                   translation unit under `src/`. Tests do not count -- a header whose only consumer is
                   a test is the A-110 shape at one remove.
  * `diagnostic`   only useful if something can read it out, so each `abi` name must exist in the C
                   surface (and, through it, in the Python bindings, which see nothing else).
  * `contract`     a port is a promise; `stub` names the reference implementation that shows the
                   promise is implementable.
  * `policy`       composed by the host, so reachability from `src/` is *not* expected. What it owes
                   instead is a runnable example. `example: null` is allowed and counted, because
                   "no example yet" and "nobody thought about it" must not look the same in a file.
  * `reference`    an implementation of a contract (`driver/*`). It owes neither a production consumer
                   nor an example -- it *is* the example; what it owes is being exercised by the tests,
                   so an unused stub is dead weight rather than an asset.

`todo` exists so the ratchet can start before the classification is finished; its count is compared
against `todo_baseline` on every run and may only fall.

Two ratchets, both learned from `scaffold`:

  * `known_unshipped` lists the `motor-owned` headers that nothing in `src/` reaches, each with the
    batch that fixes it. An entry whose header *has* become reachable is itself a failure, because a
    stale exception hides the next real one.
  * a catalogue entry for a header that no longer exists is a failure too, for the same reason.

This is the STATIC half of the rule. The dynamic half -- "switch the thing off and the output must
move" -- is a test convention, not a gate: `wired` answers "is the wire there", never "is the thing on
the other end right".

`--selftest` runs eleven shapes through the real checker, including both ratchets in the stale direction.
"""

from __future__ import annotations

import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
INCLUDE = ROOT / "include" / "foc"
DRIVER = INCLUDE / "driver"
API = INCLUDE / "api" / "foc_api.h"
REGISTER = Path(__file__).resolve().with_name("wired.json")

CLASSES = ("motor-owned", "diagnostic", "contract", "policy", "reference", "todo")
INCLUDE_RE = re.compile(r'#include\s*[<"]foc/([^">]+)[">]')
ABI_RE = re.compile(r"\bfoc_[a-z0-9_]+\s*\(")


def header_keys() -> list[str]:
    """Every installed header, relative to `include/foc/`."""
    return sorted(str(path.relative_to(INCLUDE)).replace("\\", "/")
                  for path in INCLUDE.rglob("*") if path.suffix in {".h", ".hpp"})


def includes_of(path: Path) -> set[str]:
    return set(INCLUDE_RE.findall(path.read_text(encoding="utf-8", errors="replace")))


def world() -> dict:
    """The five facts the checker needs, gathered from the tree."""
    keys = header_keys()
    graph = {key: includes_of(INCLUDE / key) for key in keys}
    reachable: set[str] = set()
    frontier: set[str] = set()
    for path in sorted((ROOT / "src").rglob("*")):
        if path.suffix in {".c", ".cpp"}:
            frontier |= includes_of(path)
    while frontier:
        nxt: set[str] = set()
        for key in frontier:
            if key in reachable:
                continue
            reachable.add(key)
            nxt |= graph.get(key, set())
        frontier = nxt
    abi = set(ABI_RE.findall(API.read_text(encoding="utf-8", errors="replace"))) if API.is_file() else set()
    abi = {name.rstrip("(").strip() for name in abi}
    implementers: dict[str, set[str]] = {}
    for path in sorted(DRIVER.glob("*.hpp")):
        for key in includes_of(path):
            implementers.setdefault(key, set()).add(path.name)
    from_tests: set[str] = set()
    for path in sorted((ROOT / "test_package" / "test").rglob("*")):
        if path.suffix in {".c", ".cpp", ".hpp"}:
            from_tests |= includes_of(path)
    return {"headers": set(keys), "reachable": reachable, "abi": abi, "implementers": implementers,
            "from_tests": from_tests, "exists": lambda rel: (ROOT / rel).is_file()}


def check(catalogue: dict, facts: dict, register: dict) -> list[str]:
    """Every problem with the catalogue, given the tree. Pure, so the selftest runs the real rules."""
    problems: list[str] = []
    headers, reachable = facts["headers"], facts["reachable"]
    known = register.get("known_unshipped", {}) or {}
    baseline = int(register.get("todo_baseline", 0))

    for key in sorted(headers - set(catalogue)):
        problems.append(f"`{key}` is installed but has no class -- every public header must be one of "
                        f"{', '.join(CLASSES)}, so 'nobody decided' cannot look like 'fine'")
    for key in sorted(set(catalogue) - headers):
        problems.append(f"`{key}` is catalogued but no longer exists -- a rotted entry hides the next "
                        f"real one")

    todos = 0
    for key in sorted(set(catalogue) & headers):
        spec = catalogue[key] or {}
        klass = spec.get("class")
        if klass not in CLASSES:
            problems.append(f"`{key}`: class {klass!r} is not one of {', '.join(CLASSES)}")
            continue
        if klass == "motor-owned":
            if key in reachable:
                if key in known:
                    problems.append(f"`{key}` is listed in `known_unshipped` but something in `src/` "
                                    f"reaches it now -- remove the entry in the same commit that wired it")
            elif key not in known:
                problems.append(f"`{key}` is `motor-owned` and nothing under `src/` reaches it: the "
                                f"header ships, the tests are green, and a user of the package cannot "
                                f"get at it (wire it, or list it in `known_unshipped` with the batch "
                                f"that will)")
        elif klass == "diagnostic":
            names = spec.get("abi") or []
            if not names:
                problems.append(f"`{key}` is `diagnostic` with no `abi` names -- a recorder nobody can "
                                f"read is a recorder nobody has")
            for name in names:
                if name not in facts["abi"]:
                    problems.append(f"`{key}`: `{name}` is not a function in `foc_api.h`, so the "
                                    f"diagnostic channel does not exist")
        elif klass == "contract":
            stub = spec.get("stub")
            if not stub:
                problems.append(f"`{key}` is a `contract` with no `stub` -- nothing shows the promise is "
                                f"implementable")
            else:
                implementers = facts["implementers"].get(key, set())
                if not implementers:
                    problems.append(f"`{key}` is a `contract` that no file under `driver/` implements")
        elif klass == "policy":
            if "example" not in spec:
                problems.append(f"`{key}` is a `policy` with no `example` field -- the field is required, "
                                f"and `null` is a legitimate value as long as the reason is written down")
            elif spec.get("example") is None:
                if not str(spec.get("why") or "").strip():
                    problems.append(f"`{key}` is a `policy` with no example and no `why`")
            elif not facts["exists"](str(spec["example"])):
                problems.append(f"`{key}`: example `{spec['example']}` does not exist")
        elif klass == "reference":
            if key not in facts["from_tests"]:
                problems.append(f"`{key}` is a reference implementation that no test includes -- an "
                                f"unexercised stub is dead weight, not an asset")
        else:  # todo
            todos += 1
            if not str(spec.get("why") or "").strip():
                problems.append(f"`{key}` is `todo` with no `why` -- a placeholder without a reason is a "
                                f"decision nobody made")

    if todos > baseline:
        problems.append(f"{todos} header(s) are `todo` against a baseline of {baseline} -- the ratchet "
                        f"only falls; classify them, or raise the baseline in a commit that says why")
    for key in sorted(known):
        if key not in catalogue:
            problems.append(f"`known_unshipped` names `{key}`, which is not catalogued")
    return problems


def selftest() -> bool:
    """Nine shapes the checker must classify correctly, run through the real `check`."""
    files = {"examples/a.cpp"}
    facts = {
        "headers": {"core/owned.hpp", "core/lost.hpp", "core/diag.hpp", "core/port.hpp", "core/pol.hpp",
                    "core/tbd.hpp", "core/unr.hpp"},
        "reachable": {"core/owned.hpp", "core/diag.hpp", "core/port.hpp", "core/pol.hpp", "core/tbd.hpp"},
        "abi": {"foc_read"},
        "implementers": {"core/port.hpp": {"stub_port.hpp"}},
        "from_tests": {"core/owned.hpp", "core/diag.hpp", "core/port.hpp", "core/pol.hpp", "core/tbd.hpp"},
        "exists": lambda rel: rel in files,
    }
    base = {
        "core/owned.hpp": {"class": "motor-owned"},
        "core/lost.hpp": {"class": "motor-owned"},
        "core/diag.hpp": {"class": "diagnostic", "abi": ["foc_read"]},
        "core/port.hpp": {"class": "contract", "stub": "stub_port.hpp"},
        "core/pol.hpp": {"class": "policy", "example": "examples/a.cpp"},
        "core/tbd.hpp": {"class": "todo", "why": "later"},
        "core/unr.hpp": {"class": "motor-owned"},
    }
    register = {"todo_baseline": 1, "known_unshipped": {"core/unr.hpp": "batch 9", "core/lost.hpp": "batch 7"}}

    def run(catalogue=None, reg=None, f=None):
        return check(catalogue if catalogue is not None else base, f or facts, reg or register)

    def mutate(key, **over):
        out = json.loads(json.dumps(base))
        out[key].update(over)
        return out

    cases = [
        ("a clean catalogue", run(), False),
        # 1. the finding this gate exists for: motor-owned, shipped header, no production consumer.
        ("a motor-owned header nothing reaches", run(reg={"todo_baseline": 1, "known_unshipped": {
            "core/unr.hpp": "batch 9"}}), True),
        # 2. A-110 at one remove: the only reachable consumer is a test, so it is not reachable at all.
        ("a production consumer that is only a test", run(
            f={**facts, "reachable": facts["reachable"] - {"core/owned.hpp"}}), True),
        ("a diagnostic whose ABI name is missing", run(mutate("core/diag.hpp", abi=["foc_gone"])), True),
        ("a contract with no implementer", run(mutate("core/port.hpp"), f={**facts, "implementers": {}}), True),
        ("a policy with no example field", run(mutate("core/pol.hpp", example="examples/missing.cpp")), True),
        ("an unclassified header", run({k: v for k, v in base.items() if k != "core/lost.hpp"}), True),
        ("a todo count above the baseline", run(mutate("core/tbd.hpp"), reg={"todo_baseline": 0,
                                                                            "known_unshipped": register["known_unshipped"]}), True),
        # 3. both ratchets, in the direction that says "the debt was paid, delete the note".
        ("a stale known_unshipped entry", run(reg={"todo_baseline": 1, "known_unshipped": {
            "core/unr.hpp": "batch 9", "core/owned.hpp": "batch 9"}}), True),
        ("a catalogued header that no longer exists", run({**base, "core/gone.hpp": {"class": "motor-owned"}}),
         True),
        ("a reference implementation nothing exercises", run({**base, "driver/stub.hpp": {"class": "reference"}}),
         True),
    ]
    ok = True
    for label, problems, want_red in cases:
        red = bool(problems)
        mark = "ok" if red == want_red else "WRONG"
        print(f"[wired-selftest] {label:44s} -> {len(problems)} problem(s), want "
              f"{'red' if want_red else 'green'} [{mark}]")
        if red != want_red:
            ok = False
            for problem in problems:
                print(f"                  {problem}")
    return ok


def main(argv: list[str]) -> int:
    if "--selftest" in argv:
        if not selftest():
            print("[wired] FAIL (selftest)", file=sys.stderr)
            return 1

    register = json.loads(REGISTER.read_text(encoding="utf-8")) if REGISTER.is_file() else {}
    catalogue = register.get("catalogue", {}) or {}
    facts = world()
    problems = check(catalogue, facts, register)

    classes: dict[str, int] = {}
    for spec in catalogue.values():
        classes[str((spec or {}).get("class"))] = classes.get(str((spec or {}).get("class")), 0) + 1
    summary = ", ".join(f"{count} {name}" for name, count in sorted(classes.items()))
    no_example = sorted(key for key, spec in catalogue.items()
                        if (spec or {}).get("class") == "policy" and (spec or {}).get("example") is None)
    known = register.get("known_unshipped", {}) or {}
    print(f"[wired] {len(facts['headers'])} header(s): {summary or 'none catalogued'}; "
          f"{len(known)} known-unshipped, {len(no_example)} policy without an example")
    if known:
        for key in sorted(known):
            print(f"[wired] known unshipped: `{key}` -- {known[key]}")
    if no_example:
        for key in no_example:
            print(f"[wired] policy without an example: `{key}` -- {catalogue[key].get('why')}")
    if problems:
        for problem in problems:
            print(f"[wired] {problem}", file=sys.stderr)
        print(f"[wired] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        return 1
    print("[wired] OK (every installed header reaches the consumer its class promises, or is on the "
          "debt list with the batch that pays it)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
