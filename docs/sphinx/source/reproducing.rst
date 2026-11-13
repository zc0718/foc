_`Reproducing these numbers`
============================

Every figure in this documentation is produced by a command in the tree, and this page is the list.
The point is not that the numbers are large; it is that a reader who does not trust them can make
them again instead of taking them on faith.

Two rules make the list worth keeping:

- **A number is only a measurement if the run that produced it is the run you are reading about.** The
  coverage figure comes from a build, and the case count from the same build, so the two are taken
  back to back -- ``coverage_gate.py`` first, ``number_sheet.py --record`` second. Reversing them
  records yesterday's coverage next to today's case count, which is exactly the kind of pair that
  looks plausible forever.
- **This page names paths, and a check fails if one of them stops existing.** A reproduction
  instruction that points at a renamed script is worse than none: it tells the reader the fault is in
  their environment.

What the numbers come from
--------------------------

.. code-block:: bash

   conan create . -s build_type=Debug --build=missing      # builds, and runs the whole suite
   python3 .github/project/coverage_gate.py                # the two coverage floors
   python3 .github/project/number_sheet.py --record        # writes docs/verification-numbers.json

``number_sheet.py`` reads the artefacts, not the prose: the case count from the test run, the
coverage figures from the ``.gcov`` output. ``gates.py docs`` then compares the two numbers that
appear on the :doc:`usage` page against that file, so a stale figure fails the build rather than
aging quietly.

The gates
---------

.. code-block:: bash

   python3 .github/project/gates.py all                    # every project gate, 15-20 minutes

Before a commit, the same list without the five that need a build of the library itself:

.. code-block:: bash

   python3 .github/project/gates.py license strict claims docs deps form boundaries \
       capabilities ci-policy assume anchor scaffold budget clocks observability \
       combinations domain blast wired smoke regmap subset docvoice symbols equations \
       enforce docshape docver docannot docscope compliance demo ports matrix

Three of those are worth understanding before running them, because their failure modes are
confusing:

- ``docs`` compiles nothing, but it re-renders the pages that carry numbers and fails when a quoted
  figure disagrees with ``verification-numbers.json``. A red ``docs`` after a green test run means the
  prose is stale, not the code.
- ``demo`` builds every example from the command **in that example's own header**. It is the only
  check that keeps the instructions on the pages honest, and it is why a renamed source turns red
  instead of leaving a reader with a broken line.
- ``mutation`` (``mutate.py``) rewrites files in this tree and restores them. It must never be run
  under ``timeout``: a kill in the middle leaves a mutant behind, and the next red looks like a
  physics failure. After any interrupted run, check ``git status`` before believing anything.

The local-only gates
--------------------

Five gates are excluded from the list above because they need a full library build, and they are the
ones that answer "does the claim still hold" rather than "does it compile": ``mutants``,
``plantref``, ``mcdc``, ``pil`` and ``fmu``. They run before a release:

.. code-block:: bash

   python3 .github/project/mutate.py --check               # mutation scores, one module at a time
   python3 .github/project/gates.py plantref mcdc pil fmu

CI does not run them, by design -- they mutate the tree and take minutes per module. That is a
statement about where the evidence lives, not a hidden exemption: this page is how a reader gets the
same answers on their own machine.

What this page does not do
--------------------------

It does not make the results portable. Everything here is produced on x86-64 Linux with gcc 13; a
different compiler will produce different floating-point results at the last bit, and several cases
compare trajectories bit for bit. When those disagree on your machine, the first question is which
compiler and which optimisation level, not which side is wrong -- see
:ref:`what has actually been run <platform-evidence>`.
