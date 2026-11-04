#!/usr/bin/env python3
"""When the foundation moves, every conclusion that rested on it must be re-read -- and the blast
radius is computed, not remembered.

`docs/sphinx/source/foundation-log.rst` has carried a blast-radius column since it was written, and
`claim-basis.json` has carried the dependency graph that column is queried from. What neither had was
a mechanism that *insists*: the column is filled in by whoever writes the row, the graph is only ever
queried one edge deep by `claims --impact`, and nothing refuses a commit that changes the foundation
and walks away.

That gap is the expensive one. A wrong number does not stay wrong in one place: it is written into a
comparison, then into a golden trace, and from then on every later check endorses it. The measurement
is stable, repeatable, self-consistent, and wrong -- F-40 and A-110 are both that shape. The cost is
not one number, it is every conclusion built on it, which is why this gate exists before any new
capability work: 地基一变就清账.

The rule, in five steps (plan §2.2.2):

  1. TRIGGER. A changed file under `include/foc/`, `src/` or the SIL instruments needs a ledger entry
     when at least `fan_in_threshold` claim rows rest on it, when it is listed in `watched`, or when it
     itself rests on at least `fan_out_threshold` other modules. The third clause is not decoration: the
     first version of this gate keyed only on fan-in and therefore let a change to `motor.hpp` through --
     `motor.hpp` consumes seventeen modules and is consumed by one, and it is exactly where the loop's
     ordering, the reference path and the fault checks live. A module that rests on many others is
     upstream of all of them.
  2. RADIUS. The affected set is the transitive closure of `claim-basis.json` from the entry's seeds.
     It is recomputed here every time and never read from the entry: a hand-written radius that
     disagrees with the graph is exactly the failure this gate is for.
  3. VERDICT. Every affected row carries `holds` (and why), `retest` (with the new number and the
     evidence that produced it) or `refuted` (with the level the README was downgraded to).
  4. CLEAR. `retest` rows have a number; `refuted` rows have a lower README level, and no dependent
     row may be left as `holds` under the same entry -- that is the "the conclusion is still standing
     while the foundation beneath it changed" state this gate exists to refuse.
  5. CONVERGE. No affected row without a verdict; no verdict for a row that rests on nothing in play;
     an instrument with no claim row must name what to re-run by hand.

`--selftest` runs ten synthetic cases against the real checker, including the A-110 shape (an entry that
exists and adjudicates nothing) and the stale-radius shape, so a checker that has stopped being able to
fail cannot pass by finding nothing.
"""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))

from claim_gate import claim_rows, closure, load_basis  # noqa: E402  (one graph, one closure)

ROOT = Path(__file__).resolve().parents[2]
REGISTER = Path(__file__).resolve().with_name("blast.json")

# The foundation, for trigger purposes. `.github/`, `docs/`, `README.md` and the ledger itself are
# excluded: a ledger cannot require a ledger entry for its own bookkeeping.
LAYER_S = ("include/foc/", "src/", "test_package/test/unit/", "test_package/test/support/")

VERDICTS = ("holds", "retest", "refuted")
RANK = {"L1": 1, "L2": 2, "L3": 3, "L4": 4}


def key_of(path: str) -> str:
    """Map a repo-relative path into the namespace `claim-basis.json` uses (`include/foc/` stripped)."""
    path = path.replace("\\", "/")
    return path[len("include/foc/"):] if path.startswith("include/foc/") else path


def readme_levels() -> dict[str, str]:
    """Module -> the level its README claim row currently carries."""
    return {module: level for modules, level, _claim, _evidence in claim_rows() for module in modules}


def git_changed(base: str) -> list[str] | None:
    """Files that differ from `base`, plus anything staged. `None` when git cannot answer."""
    out: list[str] = []
    for extra in (["diff", "--name-only", base], ["diff", "--name-only", "--cached"]):
        proc = subprocess.run(["git", "-C", str(ROOT), *extra], capture_output=True, text=True)
        if proc.returncode != 0:
            return None
        out.extend(line.strip() for line in proc.stdout.splitlines() if line.strip())
    return sorted(set(out))


def evaluate(mapped: dict[str, list[str]], register: dict, levels: dict[str, str],
             changed: list[str], exists) -> list[str]:
    """Every problem with the ledger, given the graph, the ledger, the README and a diff.

    Pure: `exists` is injected so the selftest can run the real checker on a synthetic tree.
    """
    problems: list[str] = []
    watched: dict[str, str] = register.get("watched", {}) or {}
    threshold = int(register.get("fan_in_threshold", 2))
    fan_out = int(register.get("fan_out_threshold", 3))
    entries: list[dict] = register.get("entries", []) or []

    for rel in sorted(watched):
        if not exists(rel):
            problems.append(f"watched `{rel}` no longer exists -- remove the entry (a rotted watch list is "
                            f"how the next real instrument goes unnoticed)")
        if not str(watched[rel]).strip():
            problems.append(f"watched `{rel}` carries no reason -- the entry is a claim about what a change to "
                            f"it re-bases, so it has to say what that is")

    resting_on: dict[str, set[str]] = {}
    for module, items in mapped.items():
        for item in items:
            resting_on.setdefault(item, set()).add(module)

    available = {row: {item for item in mapped.get(row, []) if item in mapped} for row in mapped}

    # --- schema -----------------------------------------------------------------------------------
    seen_ids: set[str] = set()
    for entry in entries:
        ident = str(entry.get("id", "?"))
        if ident in seen_ids:
            problems.append(f"{ident}: duplicate id -- the ledger is append-only, so ids must be unique")
        seen_ids.add(ident)
        for field in ("date", "what_changed", "seeds"):
            if not entry.get(field):
                problems.append(f"{ident}: `{field}` is empty -- an entry that does not say what changed "
                                f"cannot be checked")
        for item in entry.get("affected", []) or []:
            verdict = item.get("verdict")
            row = str(item.get("row", "?"))
            if verdict not in VERDICTS:
                problems.append(f"{ident}: `{row}` has verdict {verdict!r}, not one of {', '.join(VERDICTS)}")
                continue
            if verdict == "holds":
                if not item.get("why"):
                    problems.append(f"{ident}: `{row}` is `holds` without a reason -- 'it is fine' is the "
                                    f"answer this gate exists to make uncheckable")
            elif verdict == "retest":
                if not item.get("new_number"):
                    problems.append(f"{ident}: `{row}` is `retest` without a new number -- a re-run that "
                                    f"reports nothing is indistinguishable from no re-run")
                evidence = item.get("evidence")
                if not evidence or not exists(str(evidence)):
                    problems.append(f"{ident}: `{row}` is `retest` but its evidence {evidence!r} does not "
                                    f"exist")
            else:  # refuted
                if item.get("removed"):
                    if row in levels:
                        problems.append(f"{ident}: `{row}` is `refuted` and marked removed, but the README "
                                        f"still carries an {levels[row]} row for it")
                else:
                    was, now = item.get("was_level"), item.get("now_level")
                    if was not in RANK or now not in RANK or RANK[now] >= RANK[was]:
                        problems.append(f"{ident}: `{row}` is `refuted` without a strictly lower level "
                                        f"(was={was!r} now={now!r})")
                    elif levels.get(row) != now:
                        problems.append(f"{ident}: `{row}` is `refuted` at {now} but the README carries "
                                        f"{levels.get(row, 'no row')} -- the retirement was recorded, not made")

    # --- trigger ----------------------------------------------------------------------------------
    required: list[tuple[str, str]] = []
    for path in changed:
        if not path.startswith(LAYER_S):
            continue
        if path in watched:
            required.append((path, "listed in `watched`: a layer-S instrument"))
            continue
        fan = len(resting_on.get(key_of(path), ()))
        if fan >= threshold:
            required.append((path, f"{fan} claim row(s) rest on it"))
            continue
        rests_on = len([item for item in mapped.get(key_of(path), ()) if item in mapped])
        if rests_on >= fan_out:
            required.append((path, f"it rests on {rests_on} other claim modules, so it is upstream of them"))

    covered: set[str] = set()
    in_play = []
    for entry in entries:
        seeds = [str(seed) for seed in entry.get("seeds", []) or []]
        hit = {path for path, _why in required if path in seeds or key_of(path) in {key_of(s) for s in seeds}}
        if hit:
            in_play.append((entry, seeds))
            covered |= hit
    for path, why in required:
        if path not in covered:
            problems.append(f"{path}: {why}, but no ledger entry covers it -- open one and settle every row "
                            f"that rests on it (地基一变就清账)")

    # --- radius and convergence, for the entries this diff touches only ---------------------------
    for entry, seeds in in_play:
        ident = str(entry.get("id", "?"))
        radius = closure([key_of(seed) for seed in seeds], mapped)
        verdicts: dict[str, str] = {}
        for item in entry.get("affected", []) or []:
            verdicts.setdefault(str(item.get("row")), str(item.get("verdict")))
        manual = {str(seed) for seed in entry.get("manual_seed", []) or []}

        for row in sorted(radius - set(verdicts)):
            problems.append(f"{ident}: `{row}` rests on the seeds but carries no verdict -- that is the "
                            f"A-110 shape: the ledger is open, the account is empty")
        for row in sorted(set(verdicts) - radius):
            if row in manual:
                continue
            problems.append(f"{ident}: `{row}` carries a verdict but does not rest on any seed -- either "
                            f"the seed list is short or the row list is padded")

        if not radius:
            hand = entry.get("hand_rerun", []) or []
            if not hand:
                problems.append(f"{ident}: no claim row rests on the seeds, so the entry must name what to "
                                f"re-run by hand (`hand_rerun`)")
            else:
                for item in hand:
                    if not exists(str(item)):
                        problems.append(f"{ident}: `hand_rerun` entry {item!r} does not exist")

        for row, verdict in verdicts.items():
            if verdict != "refuted":
                continue
            for dependent, items in available.items():
                if row in items and verdicts.get(dependent) == "holds":
                    problems.append(f"{ident}: `{dependent}` rests on the refuted `{row}` and was left as "
                                    f"`holds` -- the conclusion would go on standing on a foundation that "
                                    f"moved")
    return problems


def selftest() -> bool:
    """Seven shapes the checker must classify correctly, run through the real `evaluate`."""
    mapped = {
        "core/a.hpp": ["core/a.hpp"],
        "core/b.hpp": ["core/a.hpp", "src/b.cpp"],
        "motor/m.hpp": ["core/b.hpp"],
        "core/d.hpp": ["core/b.hpp", "src/d.cpp"],
        "core/c.hpp": ["core/types.hpp"],
    }
    levels = {"core/a.hpp": "L3", "core/b.hpp": "L3", "motor/m.hpp": "L3", "core/d.hpp": "L3",
              "core/c.hpp": "L1"}
    files = {"test_package/test/unit/test_a.cpp", "test_package/test/unit/sil_abc_plant.hpp"}
    exists = lambda rel: rel in files  # noqa: E731  (a one-line stub is clearer than a class here)
    watched = {"test_package/test/unit/sil_abc_plant.hpp": "the second plant"}
    seed = "include/foc/core/a.hpp"

    def entry(**over):
        base = {"id": "B-xx", "date": "2026-11-04", "what_changed": "a", "seeds": [seed], "affected": []}
        base.update(over)
        return base

    def run(register, changed=None, lv=None):
        return evaluate(mapped, register, lv if lv is not None else levels,
                        changed if changed is not None else [seed], exists)

    held = {
        "holds": {"row": "motor/m.hpp", "verdict": "holds", "why": "does not quote a"},
        "retest": {"row": "core/b.hpp", "verdict": "retest", "new_number": "0.1234",
                   "evidence": "test_package/test/unit/test_a.cpp"},
    }

    cases = []
    # 1. a foundation file changes and nobody opens the ledger.
    cases.append(("no ledger entry", run({"watched": watched, "entries": []}), True))
    # 2. the ledger is opened and adjudicates nothing -- the A-110 shape.
    cases.append(("ledger open, account empty", run({"watched": watched, "entries": [entry()]}), True))
    # 3. one row left without a verdict.
    cases.append(("a dependent row left unadjudicated",
                  run({"watched": watched, "entries": [entry(affected=[held["retest"]])]}), True))
    # 4. a verdict for a row that rests on nothing in play.
    cases.append(("a padded verdict", run({"watched": watched, "entries": [entry(
        affected=[held["holds"], held["retest"], {"row": "core/c.hpp", "verdict": "holds", "why": "x"}])]}), True))
    # 5. refuted while the README still carries the old level.
    cases.append(("retired in the ledger, alive in the README", run({"watched": watched, "entries": [entry(
        affected=[held["holds"], held["retest"],
                  {"row": "core/a.hpp", "verdict": "refuted", "was_level": "L3", "now_level": "L2"}])]}), True))
    # 6. a re-run that reports no number.
    cases.append(("retest with no number", run({"watched": watched, "entries": [entry(
        affected=[held["holds"],
                  {"row": "core/b.hpp", "verdict": "retest",
                   "evidence": "test_package/test/unit/test_a.cpp"},
                  {"row": "core/a.hpp", "verdict": "refuted", "was_level": "L3", "now_level": "L2"}])]},
        lv={"core/a.hpp": "L2", "core/b.hpp": "L3", "motor/m.hpp": "L3"}), True))
    # 7. a clean entry: every row settled, the retirement made, and the one row that does not rest on
    #    the refuted claim is allowed to stay `holds` -- so the rule above is a rule, not a blanket ban.
    clean_levels = {"core/a.hpp": "L2", "core/b.hpp": "L3", "motor/m.hpp": "L3", "core/d.hpp": "L3"}
    cases.append(("a settled account", run({"watched": watched, "entries": [entry(
        affected=[held["holds"], held["retest"],
                  {"row": "core/d.hpp", "verdict": "holds", "why": "rests on b only, not on a"},
                  {"row": "core/a.hpp", "verdict": "refuted", "was_level": "L3", "now_level": "L2"}])]},
        lv=clean_levels), False))
    # 7b. the same account with the dependent left standing -- refused.
    cases.append(("a dependent left standing on a refuted claim", run({"watched": watched, "entries": [entry(
        affected=[{"row": "core/b.hpp", "verdict": "holds", "why": "looks fine"},
                  {"row": "motor/m.hpp", "verdict": "holds", "why": "does not quote a"},
                  {"row": "core/a.hpp", "verdict": "refuted", "was_level": "L3", "now_level": "L2"}])]},
        lv=clean_levels), True))
    # 8. an instrument with no claim row must name its hand re-run.
    instrument = "test_package/test/unit/sil_abc_plant.hpp"
    cases.append(("an instrument with nothing to adjudicate", run(
        {"watched": watched, "entries": [{"id": "B-yy", "date": "2026-11-04", "what_changed": "pl ant",
                                          "seeds": [instrument], "affected": []}]}, changed=[instrument]), True))
    cases.append(("the same, with a hand re-run", run(
        {"watched": watched, "entries": [{"id": "B-yy", "date": "2026-11-04", "what_changed": "plant",
                                          "seeds": [instrument], "affected": [],
                                          "hand_rerun": ["test_package/test/unit/test_a.cpp"]}]},
        changed=[instrument]), False))

    ok = True
    for label, problems, want_red in cases:
        red = bool(problems)
        mark = "ok" if red == want_red else "WRONG"
        print(f"[blast-selftest] {label:44s} -> {len(problems)} problem(s), want {'red' if want_red else 'green'}"
              f" [{mark}]")
        if red != want_red:
            ok = False
            for problem in problems:
                print(f"                  {problem}")
    return ok


def main(argv: list[str]) -> int:
    if "--selftest" in argv:
        if not selftest():
            print("[blast] FAIL (selftest)", file=sys.stderr)
            return 1

    register = json.loads(REGISTER.read_text(encoding="utf-8")) if REGISTER.is_file() else {}
    mapped, _unmapped = load_basis()

    if "--changed" in argv:
        raw = argv[argv.index("--changed") + 1] if argv.index("--changed") + 1 < len(argv) else ""
        changed = [item.strip() for item in raw.split(",") if item.strip()]
        mode = "--changed (explicit)"
    else:
        base = argv[argv.index("--base") + 1] if "--base" in argv else "HEAD"
        answer = git_changed(base)
        if answer is None:
            print(f"[blast] git cannot diff against {base!r}: the trigger check did not run", file=sys.stderr)
            return 1
        changed = answer
        mode = f"git diff --name-only {base}"

    problems = evaluate(mapped, register, readme_levels(), changed, lambda rel: (ROOT / rel).exists())
    layer = [path for path in changed if path.startswith(LAYER_S)]
    print(f"[blast] {mode}: {len(changed)} changed, {len(layer)} in the foundation; "
          f"{len(register.get('entries', []) or [])} ledger entr(y/ies), "
          f"{len(register.get('watched', {}) or {})} watched instrument(s)")
    if not layer:
        # Not a failure: most commits do not touch the foundation. But say it out loud, because a
        # green line from a run that had nothing to inspect reads exactly like a green line from a
        # run that inspected everything.
        print("[blast] nothing in the foundation changed, so the trigger had nothing to inspect; "
              "the ledger's own consistency was still checked")
    if problems:
        for problem in problems:
            print(f"[blast] {problem}", file=sys.stderr)
        print(f"[blast] FAIL ({len(problems)} problem(s))", file=sys.stderr)
        return 1
    print("[blast] OK (every foundation change in this diff is covered by a cleared ledger entry, or "
          "nothing changed)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
