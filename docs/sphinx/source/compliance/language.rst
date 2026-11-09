_`Language compliance`
======================

This page is the **language half** of the claim: the rule table, its deviations,
and what each of them can support. The four sentences that no tier and no
industry change are on :doc:`index`; the tiers themselves are on :doc:`tier`; the
industry-by-industry answers are on :doc:`industries`.

It answers one question honestly: **which sentence about language
compliance may this project put in front of a customer today, and which may it
not put in front of anybody yet.**

It is deliberately short, and every claim on it points at a file or a gate that
can be re-run. A claim that cannot be re-run is not evidence, it is a
recollection -- and the whole point of the report is that the two differ.

.. _subset-table:

The rule table
--------------

``.github/project/compliance/rules.tsv`` is the claim, one rule per line. It is
**generated**, from two inputs:

- ``.github/project/compliance/tool-checks.txt`` -- a snapshot of the check names the
  pinned ``.github/misc/.clang-tidy`` **names** in its ``Checks:`` block. The
  snapshot exists because a glob quietly matching one more check adds a rule to
  the subset with nobody deciding to. It records what that block names, not what
  the tool resolves it to -- see the next section for why those are two different
  questions.
- ``.github/project/compliance/rules-manual.json`` -- everything a tool cannot decide:
  what the compiler flags enforce, what this repository's own gates and tests
  enforce, which units have no behaviour to check, which rules are deviated and
  which are open.

Regenerate with ``python3 .github/project/compliance/gen_subset.py``; ``--check`` reports
drift instead of writing.

The two lists, and why neither can answer alone
-----------------------------------------------

The pinned ``.github/misc/.clang-tidy`` is a **loose baseline of intent, by
design**. Its ``Checks:`` value is a folded scalar whose ``#`` lines are
literal, so ``clang-tidy --dump-config`` resolves the list to ``-*``: the
scaffold's own ``tidy`` gate compiles every source in ``src/`` and constrains
nothing. That is the template's posture rather than a defect, and it is not a
patch this project sends upstream: the private-domain syntax of this
repository (``// Conan::ImportStart`` and the rest) is governed by the IaC
layer -- the shared conventions, the codegen starter and the skills -- so the
linter is deliberately kept out of that argument and stays compatible with
whatever an instance decides.

What follows from that is the part that matters to a reader of this page:
**the pinned file cannot be the answer to "what is enforced".** Two facts are
true at once, and the page would be misleading if it stated either alone:

- the pinned configuration **names** ``bugprone-*``, ``performance-*``,
  ``cert-*`` and two ``cppcoreguidelines-*`` checks, which is what
  ``tool-checks.txt`` records;
- the pinned configuration **runs** none of them, which is why ``enforce``
  exists.

``.github/project/enforce.json`` is where this project declares its own
strictness. The gate composes a check set from **outside** the pinned file --
``clang-tidy --checks`` overrides the config's ``Checks`` key while its
``CheckOptions`` and ``HeaderFilterRegex`` still apply -- so the pinned file
stays byte-identical and the families still run over every target. The same
file carries three things a reader needs: the families that run, the checks
exempted from a family (each with a signed deviation permit, or the gate
fails), and the checks this project has looked at and decided **not** to run.
That third list is a record with a count and a reason per entry; an entry with
no reason or no evidence fails the gate, because an omission and a decision
are otherwise indistinguishable.

Two consequences worth stating plainly:

- **The word "implemented" in the rule table means "a named mechanism runs it",
  and the mechanism for a lint rule is now the family table.** A check that the
  pinned file names while nothing executes it is not an implemented rule; it is a
  sentence about a file. The ``tidy:`` rows therefore describe the pinned
  intent, and the rows whose enforcement comes from ``enforce`` say so.
- **"This check is on" is a claim about a run, not about a configuration.** The
  witness is ``python3 .github/project/gates.py enforce``, which fails if any
  target does not compile, reports any diagnostic from the extra families, and
  runs a positive control: a scratch file that must be diagnosed with the
  families on and must be clean with them off.

The four counts, and what each one means
----------------------------------------

The ``subset`` gate (``python3 .github/project/gates.py subset``) prints the
same four numbers and refuses to pass unless they add up to the number of rows,
so a rule that matched no status surfaces as an arithmetic failure rather than
as a line nobody notices is missing.

.. list-table::
   :header-rows: 1
   :widths: 20 16 64

   * - Status
     - Means
     - What makes it legal
   * - ``implemented``
     - something enforces it
     - the ``check`` column names the mechanism, in one of four forms:
       ``tidy:<check>`` (a check the pinned config enables), ``gate:<name>``
       (a gate that exists), ``test:<file>`` (a test file that exists) or
       ``case:<id>`` (a case in the negative-compilation set). A row whose
       ``check`` names nothing that exists fails the build.
   * - ``not-applicable``
     - there is nothing to enforce
     - two sub-cases, told apart by the ``delta`` column. ``delta: none``
       means the unit has no behaviour: an interface, a lookup table or a test
       double -- the thirteen terminal ``L1`` modules of the claim table in the
       project ``README.md``. ``delta: vacuous`` means the language feature is
       rejected by the build (exceptions, RTTI), so a rule about it has no
       operand.
   * - ``deviation``
     - we do not follow it, on purpose
     - ``.github/project/compliance/deviation/<rule>.md`` exists and carries all five
       elements: reason, impact, compensating control, approver, review date.
       Missing one element fails the build, and so does a ``NOLINT`` in the
       shipped tree with no record -- and a record with no suppression.
   * - ``open``
     - we intend to have it and do not
     - this count may only fall. ``.github/project/compliance/subset-baseline.json``
       freezes it; raising it means re-freezing the baseline in the same
       commit, which is a review event rather than an accident.

``.github/project/compliance/COMPLIANCE-SUMMARY.md`` renders the same four counts for a
delivery package, plus the per-source breakdown.

What can be said today
----------------------

.. code-block:: text

   Language baseline C++17 (N4659) / C11 (N1570), enforced per public header by
   `gates.py strict`. Of the N rules registered in the subset, N1 are enforced
   by a named mechanism, N2 do not apply and carry a reason, N3 are submitted
   as deviation permits with their five elements, N4 are open.

The numbers are not written here on purpose: they are the output of the gate and
of ``COMPLIANCE-SUMMARY.md``, and a number copied into prose is a second source
of truth with a shelf life. Read them from the gate.

Two sentences that follow directly, because each has a re-runnable witness:

- **Ignoring a C ABI error code is a build failure.** The witness is a real
  compilation, not a syntax check: ``python3 .github/project/negative_compile.py``
  runs ``c_abi_dropped_error_code`` with ``-Werror`` and requires the
  diagnostic. It also pins the counter-intuitive part: a ``(void)`` cast does
  **not** silence ``warn_unused_result`` on GCC, while binding the value first
  does.
- **The rule set cannot drift from the pinned tool configuration.** The witness
  is ``python3 .github/project/gates.py subset``, which compares the enabled
  check set against ``tool-checks.txt`` and refuses a row that names a check the
  block does not name.
- **The CERT C++ family runs, and the pinned file did not need to change.** The
  witness is ``python3 .github/project/gates.py enforce``, which composes the
  check set from that block and from the family table and requires every target
  to compile and to produce no diagnostic from the extra families. The family
  was measured at zero on the source before it was switched on; the three sites
  it found on the way were fixed rather than suppressed.

What cannot be said yet
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 38 62

   * - Sentence
     - Why not, and what would close it
   * - "Organised per MISRA C++:2023: N implemented / M not applicable /
       K deviated"
     - The four categories are exactly the ones MISRA Compliance:2020 asks for,
       and the deviation permits already carry its five elements -- but the
       mapping from tool check to MISRA rule number does not exist, so N/M/K
       cannot be filled in. That is the ``misra-rule-mapping`` row of the
       ``open`` register, and it is the difference between the two lowest tiers
       on :doc:`tier`.
   * - "Compliant with MISRA C++:2023", or any ASIL / SIL / Class B level
     - Levels are derived from the whole machine's hazard analysis and safety
       goals. A library has no such context, so it can supply evidence about its
       own behaviour and nothing about the level of the system it lands in. See
       :ref:`honest limits <honest-limits>`.
   * - "Independently verified"
     - One author plus tooling can produce reproducibility, which is what this
       repository does produce: a third party who clones it and runs the gates
       gets the same numbers. It cannot produce independence.
   * - "Every check in the pinned configuration is enforced"
     - The configuration states an intent and the resolver discards it, so the
       two are separate questions and only the second one is evidence. What this
       repo enforces is in the family table, and the family table is not the
       whole of that block: the checks it names but no gate executes are recorded
       as ``open`` in the rule table, not as ``implemented``.

Where the evidence lives
------------------------

.. list-table::
   :header-rows: 1
   :widths: 42 58

   * - Artefact
     - Re-run with
   * - ``.github/project/compliance/rules.tsv``
     - ``python3 .github/project/compliance/gen_subset.py --check`` (stale means exit 1)
   * - ``.github/project/compliance/COMPLIANCE-SUMMARY.md``
     - ``python3 .github/project/compliance/subset_gate.py --summary``
   * - ``.github/project/compliance/deviation/*.md``
     - ``python3 .github/project/gates.py subset`` (five elements, and the
       NOLINT bijection)
   * - ``.github/project/compliance/subset-baseline.json``
     - ``python3 .github/project/compliance/subset_gate.py --baseline`` (a review event,
       not a reflex)
   * - ``.github/project/enforce.json``
     - ``python3 .github/project/gates.py enforce`` (every target compiles, no
       diagnostic from a family, and the positive control still fires) and
       ``python3 .github/project/enforce_gate.py --selftest`` (the families are
       really switched on)
   * - ``.github/project/compliance/tool-checks.txt``
     - ``python3 .github/project/compliance/gen_subset.py --check``; the snapshot
       and the pinned block are compared by the ``subset`` gate
   * - The materials the citations come from
     - the internal procurement ledger (not part of the delivery package): it records which
       edition of each document was obtained, the sha256 of the copy, and which claim it supports

Known gaps
----------

Three things are unresolved on purpose, and each costs the sentence it blocks:

- **The snapshot can age.** ``tool-checks.txt`` is a copy, and the tool
  installed here resolves more check names than the copy lists. Re-snapshotting
  would file rows for checks no gate runs, which is the defect this page exists
  to prevent, so the enforced set is read from the family table and the
  snapshot keeps its older meaning. The decision to re-snapshot is open.
- **No mapping from a check to a MISRA rule number.** Every count on this page
  is a count of checks or of units, not of rules in a named standard. The rule
  numbers are the ``misra-rule-mapping`` row of the ``open`` register.
- **Two delivery forms exist and are not claimed.** ``.github/project/forms.json``
  registers the FMU and the Python module with ``claim_status: gap`` and writes
  the gap out: no claim row describes the FMU (it has a runnable demonstration --
  a standard master loads it and drives the loop -- but no level, no boundary
  statement and no "what it does not claim"), and Python appears only as an
  evidence path on three rows, never as a claim that the binding behaves. The
  page therefore says "a form exists" and not "a form is supported".

.. note::

   The procurement ledger is internal by design and is not shipped. The
   consequence is deliberate: what a customer receives is the rule table, the
   deviation permits and the summary, not the list of documents this project
   bought to write them.
