# .github/project/compliance — the language-subset evidence package

**What this directory is.** Three deliverables plus the gate that keeps them honest. Unlike the rest
of `tools/`, the files here are meant to leave the repository: a customer's software-quality
questionnaire asks for exactly these three things.

| File | What it is | Generated? |
|---|---|---|
| `rules.tsv` | the rule table: one row per rule, with its status and the mechanism that enforces it | **yes** — from the two inputs below |
| `rules-manual.json` | the hand-maintained half: rules that are not clang-tidy checks, plus every judgement (`status`, `why`) | no |
| `tool-checks.txt` | a snapshot of the check names the pinned `.github/misc/.clang-tidy` enables (101 today) | **yes** — `gen_subset.py --snapshot` |
| `deviation/*.md` | one permit per suppressed rule, each carrying the five elements | no |
| `subset-baseline.json` | the frozen `open` count, so it can only fall | `subset_gate.py --baseline` |
| `COMPLIANCE-SUMMARY.md` | the four counts, rendered for a delivery package | **yes** — `subset_gate.py --summary` |

The page a reader should start with is `docs/sphinx/source/compliance.rst`, which says which sentence
may be put in front of a customer today and which may not.

## Commands

```bash
python3 .github/project/compliance/gen_subset.py --snapshot   # re-derive the tool snapshot, then rewrite rules.tsv
python3 .github/project/compliance/gen_subset.py --check      # is rules.tsv stale?
python3 .github/project/compliance/subset_gate.py             # the whole check, plus the four counts
python3 .github/project/compliance/subset_gate.py --selftest  # prove each leg can go red
python3 .github/project/compliance/subset_gate.py --summary   # rewrite COMPLIANCE-SUMMARY.md
python3 .github/project/compliance/subset_gate.py --baseline  # re-freeze the open count (a review event)
python3 .github/project/gates.py subset             # the same gate, through the project runner
```

## Why the code is here and not under `.github/`

Two reasons, and the second one is the load-bearing one.

1. `.github/misc/**` is byte-pinned by the `scaffold` gate (upstream template). `.github/project/**`
   is this repository's *gate layer* — machinery, not deliverables.
2. The rule table and the deviation permits **are** deliverables. They belong somewhere a release
   can pick them up without also picking up CI machinery, and somewhere their diff reads like a
   document rather than like a build-tool change.

The gate itself is still registered in `.github/project/gates.py`, so `gates.py all` and the
`ci-policy` gate see it like any other.

## The three states a rule can be in, and why they are not interchangeable

- **`implemented`** — something enforces it. The `check` column must name the mechanism in one of
  four forms: `tidy:<check>` (enabled by the pinned config), `gate:<name>` (a gate that exists),
  `test:<file>` (a test file that exists), `case:<id>` (a case in the negative-compilation set).
  A row whose `check` names nothing that exists fails the build. This is the only meaning the word
  has, and the gate refuses the alternative.
- **`deviation`** — we do not follow it, on purpose, and `deviation/<rule>.md` says why: reason,
  impact, compensating control, approver, review date. Every `NOLINT` in the shipped tree needs one,
  and a permit with no suppression is also a failure — a stale argument that looks current.
- **`open`** — we intend to have it and do not. The count may only fall. Three of today's five are
  blocked by the *same* fact: enablement lives in a byte-pinned file, so the fix is an upstream
  patch, not a code change. That is the most useful sentence in the table.

`not-applicable` splits in two by the `delta` column: `none` for a unit with no behaviour, `vacuous`
for a rule about a language feature the build rejects outright (`-fno-exceptions`, `-fno-rtti`).

## What this package does *not* claim

- Not a MISRA C++:2023 compliance statement: the tool-check → MISRA-rule-number mapping does not
  exist yet (`open`: `misra-rule-mapping`). The four *categories* are MISRA Compliance:2020's, and
  the permits already carry its five elements, but N/M/K cannot be filled in.
- Not any ASIL / SIL / Class B level — those come from a whole machine's hazard analysis.
- Not independent verification — only reproducibility: a third party who clones the repository and
  runs the gates gets the same numbers.
