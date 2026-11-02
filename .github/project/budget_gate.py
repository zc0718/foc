#!/usr/bin/env python3
"""A sensitivity table is a snapshot; this gate is what stops it from becoming a museum piece.

The README's L3 rows rest on tables of the form "this error source moves the number by this much". A table like
that answers yesterday's question. The question it cannot answer is the one that matters later: *someone adds a
knob to the plant or the HAL, and the table keeps passing.* Nothing in the repository would notice, and the claim
would go on being published at L3 as if the new knob had been covered.

So the knobs are enumerated from the code (`void set_<name>(` in the files listed by `error-sources.json`), and each
one must appear either as covered -- in which case the field it drives has to exist in the covering test, which is
what makes the mapping checkable rather than a declaration -- or as explicitly not an error source, with a reason
written next to it.

Two ways to fail:

  * **an unclassified knob** -- a setter in neither list. This is the one that matters: a new physical effect
    arrived and nobody decided anything about it.
  * **a rotted mapping** -- a setter listed as covered whose `StartupEffects` field is gone from the covering test.
    The claim would then cite a row that no longer measures it.

    python3 .github/project/budget_gate.py             # check
    python3 .github/project/budget_gate.py --selftest  # prove the classifier still separates
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = Path(__file__).resolve().with_name("error-sources.json")

# `void set_x(`, `void set_x()`, `auto set_x(` -- setters are how this bench exposes a model parameter.
_SETTER = re.compile(r"\bvoid\s+(set_[a-z0-9_]+)\s*\(", re.IGNORECASE)
# `enable_mechanics` is a setter in spirit but named differently; it is listed here so the list of names that
# introduce a physical parameter stays visible in one place instead of relying on a naming convention.
_EXTRA_SETTERS = ("enable_mechanics",)


def _setters(path: Path) -> set[str]:
    text = path.read_text(encoding="utf-8", errors="replace")
    found = set(_SETTER.findall(text))
    found.update(name for name in _EXTRA_SETTERS if f"void {name}(" in text)
    return found


def _classify(name: str, manifest: dict, covering_text: str) -> tuple[str, str]:
    """Return ('covered'|'excluded'|'unclassified'|'rotted', detail).

    `covered_in` lets one knob be owned by a different table than the default `covered_by`. Both are tables under the
    same rule -- "the field it drives must appear in a test that measures it" -- so an override cannot weaken the
    check, and without it a knob covered elsewhere would have to be declared as not-an-error-source, which would be a
    lie.
    """
    covered = manifest.get("covered") or {}
    covered_in = manifest.get("covered_in") or {}
    excluded = manifest.get("not_an_error_source") or {}
    if name in covered:
        field = covered[name]
        target = covered_in.get(name)
        if target is not None:
            other = (ROOT / target)
            if not other.is_file():
                return "rotted", f"{target} (missing)"
            covering_text = other.read_text(encoding="utf-8", errors="replace")
        if field in covering_text:
            return "covered", field
        return "rotted", field
    if name in excluded:
        reason = excluded[name]
        if not reason or len(reason) < 10:
            return "unclassified", "excluded without a reason"
        return "excluded", reason
    return "unclassified", ""


def _selftest() -> int:
    """The three states must stay distinguishable: that is the whole contract."""
    manifest = {
        "covered": {"set_a": "field_a", "set_gone": "field_missing"},
        "not_an_error_source": {"set_b": "a stated reason long enough to pass", "set_c": ""},
    }
    covering = "struct Effects { float field_a; };"
    cases = [
        ("covered", "set_a", "covered"),
        ("excluded", "set_b", "excluded"),
        ("reason missing", "set_c", "unclassified"),
        ("rotted mapping", "set_gone", "rotted"),
        ("unknown knob", "set_new", "unclassified"),
    ]
    failed = 0
    for label, name, want in cases:
        got, _detail = _classify(name, manifest, covering)
        ok = got == want
        failed += 0 if ok else 1
        print(f"[budget-selftest] {'ok  ' if ok else 'FAIL'} {label}: {got} (want {want})")
    print(f"[budget-selftest] {'OK' if not failed else 'FAIL'}: {len(cases) - failed}/{len(cases)}")
    return 0 if not failed else 1


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true", help="prove the classifier distinguishes the four states")
    args = ap.parse_args()
    if args.selftest:
        return _selftest()

    manifest = json.loads(MANIFEST.read_text(encoding="utf-8"))
    covering_path = ROOT / manifest["covered_by"]
    if not covering_path.is_file():
        print(f"[budget] {manifest['covered_by']}: the covering table does not exist", file=sys.stderr)
        return 1
    covering_text = covering_path.read_text(encoding="utf-8", errors="replace")

    problems: list[str] = []
    counts = {"covered": 0, "excluded": 0}
    for rel in manifest["files"]:
        path = ROOT / rel
        if not path.is_file():
            problems.append(f"{rel}: listed in error-sources.json but missing")
            continue
        for name in sorted(_setters(path)):
            state, detail = _classify(name, manifest, covering_text)
            if state == "unclassified":
                problems.append(f"{rel}: {name}() is a model knob that no sensitivity table covers and no reason "
                                f"excludes -- a new error source arrived and nobody decided anything about it")
            elif state == "rotted":
                problems.append(f"{rel}: {name}() is listed as covered through `{detail}`, but `{detail}` is not in "
                                f"{manifest['covered_by']} -- the table no longer measures what the registry claims")
            else:
                counts[state] += 1

    for problem in problems:
        print(f"[budget] {problem}", file=sys.stderr, flush=True)
    if problems:
        print(f"[budget] FAIL ({len(problems)} problem(s))", flush=True)
        return 1
    _overrides = len(manifest.get("covered_in") or {})
    _tables = f"{manifest['covered_by']}" + (f" and {_overrides} other table(s)" if _overrides else "")
    print(f"[budget] OK ({counts['covered']} knob(s) covered by {_tables}, "
          f"{counts['excluded']} excluded with a reason)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
