_`One machine, one shift`
=========================

The nine task slices answer one question each. This page runs them **in the order an axis
meets them**, on a single nameplate, from reading the plate to latching a trip — and each stage
contributes one number to a single table.

Every number on this page is printed by
``docs/sphinx/scripts/walkthrough.cxx``, which this page includes rather than quotes: the
listing is the program, and the program is what ran. If a line of the table and a line of the
program ever disagree, the program is right and the page is stale.

The machine
-----------

One nameplate is used all the way through — the same one the slices use, so a figure or a
number from another page can be compared with this run without converting anything:

.. list-table::
   :header-rows: 1
   :widths: 30 18 52

   * - Field
     - Value
     - Where it comes from
   * - Pole pairs
     - 7
     - the machine, not the library: every electrical number on this page is divided by it
   * - Bus voltage
     - 24 V
     - the SIL bench default; the modulator's ceiling follows from it
   * - Control period
     - 50 µs (20 kHz)
     - the bench default, and the rate the slices step their sequences at
   * - Nameplate ``Rs`` / ``Ld`` / ``Lq`` / ``psi_f``
     - 0.5 Ω / 4 mH / 8 mH / 0.01 Wb
     - a salient machine: ``Lq`` is twice ``Ld``, which is what makes injection and MTPA mean
       anything at all
   * - Current ceiling
     - 3 A
     - the protection rating below is built on it

1. Read the nameplate
---------------------

Nothing is derived from the machine here except the torque constant, and that one converts a
flux into a torque per amp — it does not measure anything:

.. literalinclude:: ../../scripts/walkthrough.cxx
   :language: cpp
   :start-at: void stage_nameplate()
   :end-before: // Stage 2

Two things are worth noticing. ``Kt = 1.5 * pole_pairs * psi_f`` is the conversion the field
loses most often (drop the 1.5, or use the electrical pole count); and the second half shows the
same nameplate as a **file** that the library can read back, because a configuration typed into
one program and re-typed into another is two machines. The round-trip verdict is ``same`` only
when the reader accepted what the writer produced.

Explained further in :doc:`../symbols`.

2. Check the board
------------------

Before the board is allowed to make torque, twelve items are measured against limits, and the
report is not a list of failures — it names **one cause**:

.. literalinclude:: ../../scripts/walkthrough.cxx
   :language: cpp
   :start-at: void stage_board()
   :end-before: // Stage 3

The trap this stage exists for: health is a *pattern* of signs, not "all positive". A positive
d-axis drive puts positive current on one phase and negative on the other two, so two of the
three polarity items are expected to be negative. Invert them and the count of failures stays
small while the report's cause changes — a reader who only counts failures sees a healthy board
twice.

Explained further in :doc:`current-sampling`.

3. Align, pull, hand over
-------------------------

The start-up routine decides an angle, a current and the moment to hand over. It is asked
twice: once with a recipe that can never finish, once with the real one:

.. literalinclude:: ../../scripts/walkthrough.cxx
   :language: cpp
   :start-at: void stage_startup()
   :end-before: // Stage 4

The refusal comes first on purpose. A hand-over speed of zero means "no observer will ever take
over", and the routine says so at validation rather than starting and stalling. What the table
then reports is the routine's own arithmetic: the hold lasts the settling time it was given, and
the pull takes as long as its acceleration needs to reach the hand-over speed. The
**mechanical** hand-over speed is the electrical one divided by the pole pairs — the conversion
that decides whether the observer is even in its working range.

What this stage cannot show is whether the rotor came along. There is no plant in this part of
the program, and the routine does not know: that judgement belongs to the hand-over check one
window later.

Explained further in :doc:`startup-sensorless`.

4. Take a torque step
---------------------

The first closed-loop stage, on the same SIL bench the test suite asserts on, with the same
metric helpers the ``Measured responses`` page uses:

.. literalinclude:: ../../scripts/walkthrough.cxx
   :language: cpp
   :start-at: void stage_torque_step()
   :end-before: // Stage 5

This is the page's internal consistency check: the settling time, overshoot and steady-state
error printed here are the same three numbers as the ``current step`` metrics table on
:doc:`../demos/index`, because they are computed from the same run shape by the same functions.
If those two ever disagree, one of them is measuring something else.

Explained further in :doc:`torque-reference`.

5. Run past base speed
----------------------

Above base speed the voltage ellipse, not the current ceiling, is what binds — and the speed at
which that happens is not ``psi_f`` divided by anything:

.. literalinclude:: ../../scripts/walkthrough.cxx
   :language: cpp
   :start-at: void stage_high_speed()
   :end-before: // Stage 6

Two readings matter. The base speed here comes from the flux **at the MTPA point**, which on a
salient machine is several times lower than ``psi_f / v_max`` would suggest — a drive sized on
the second number finds the real ceiling when its current loop saturates. And the first speed
that answers ``voltage_limited`` is not exactly base speed: it is the first grid point past it,
which is why this stage prints the ratio it swept rather than claiming the crossing.

Explained further in :doc:`high-speed-field-weakening`.

6. Position the axis
--------------------

The move is shaped by the profile and closed by the library's speed loop on the plant; the soft
limit is asked first, so a target behind a wall never becomes a reference:

.. literalinclude:: ../../scripts/walkthrough.cxx
   :language: cpp
   :start-at: void stage_position()
   :end-before: // Stage 7

Note what the limit returns: not a clamped target but the **previous** target with ``Param``.
Nothing downstream ever sees a position it would drive into a wall, and a caller that ignores
the status goes on commanding the old position — which looks exactly like an axis that arrived.
The residual error at the end of the move is printed with enough digits to show that it is a
plant tail, not a rounding artefact.

Explained further in :doc:`positioning`.

7. Fail safely
--------------

Three questions in one stage: how long an overload may last, how much current the derating
curve allows, and who may clear a trip:

.. literalinclude:: ../../scripts/walkthrough.cxx
   :language: cpp
   :start-at: void stage_protection()
   :end-before: } // namespace

The last three lines are the ones that get an installation into trouble. ``reset()`` clears the
latched fault and **deliberately keeps the i2t memory** — the winding is still hot — so the axis
stays blocked after a reset even though the current is healthy again. Only ``reset_cold()`` lets
it run, and that one is for a machine that has been off long enough to have cooled. Two answers
that look identical from the outside ("still blocked") have two different meanings, and the
table prints them on separate lines so that they cannot be confused.

Explained further in :doc:`protection-and-derating`.

What one run prints
-------------------

The complete output of the script, copied once — this block is the single source of every value
this page mentions:

.. literalinclude:: ../../scripts/walkthrough.expected
   :language: text

It is checked rather than remembered: the file next to the script holds the output of the run
the site was built from, and re-running the script and diffing against it is the whole test.

What this page is not
---------------------

- **Not a tuning guide.** Gains come from the library's own helpers; where they come from is on
  each slice page, and how to re-tune the outer loop on a new machine is :doc:`../sop`.
- **Not a hardware claim.** Stages 4 and 6 run on the SIL plant, which is a model. What the
  plant cannot represent is listed item by item under :ref:`honest limits <honest-limits>`.
- **Not a substitute for the slices.** Each stage states one number and hands over; the failure
  modes that compile cleanly live on the slice page it links to.
