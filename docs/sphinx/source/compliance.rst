_`Language compliance: what can be claimed today`
=================================================

This page answers one question honestly: **which sentence about language
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
  pinned ``.github/misc/.clang-tidy`` enables (101 of them: ``bugprone-*``,
  ``performance-*`` and two ``cppcoreguidelines-*``). The snapshot exists
  because a glob quietly matching one more check adds a rule to the subset with
  nobody deciding to.
- ``.github/project/compliance/rules-manual.json`` -- everything a tool cannot decide:
  what the compiler flags enforce, what this repository's own gates and tests
  enforce, which units have no behaviour to check, which rules are deviated and
  which are open.

Regenerate with ``python3 .github/project/compliance/gen_subset.py``; ``--check`` reports
drift instead of writing.

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
  config no longer enables.

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
       ``open`` register, and it is the difference between T0 and T1.
   * - "Compliant with MISRA C++:2023", or any ASIL / SIL / Class B level
     - Levels are derived from the whole machine's hazard analysis and safety
       goals. A library has no such context, so it can supply evidence about its
       own behaviour and nothing about the level of the system it lands in. See
       :ref:`honest limits <honest-limits>`.
   * - "CERT C++ rules are implemented"
     - The family is not enabled. Enablement lives in ``.github/misc/.clang-tidy``,
       which the ``scaffold`` gate byte-pins, so the change is an upstream patch
       rather than a local one. Measured when it was tried: zero hits on the
       current source, so the patch is cheap -- but it has not been made, and an
       unenabled rule is not an implemented one.
   * - "Independently verified"
     - One author plus tooling can produce reproducibility, which is what this
       repository does produce: a third party who clones it and runs the gates
       gets the same numbers. It cannot produce independence.

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
   * - The materials the citations come from
     - ``workspace/compliance/`` -- the ledger, the sha256 of every copy, and
       which tier each one unlocks

.. note::

   ``workspace/`` is not in version control by design: it holds internal
   material. The consequence is that the ledger is not part of what a customer
   receives -- but the rule table, the deviation permits and the summary are,
   because they live in ``.github/project/compliance/``.
