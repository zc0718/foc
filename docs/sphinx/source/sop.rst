_`Outer-loop self-tuning`
=========================

This page is the standard operating procedure for one process-shaped capability:
tuning a speed loop on a machine the library has never seen. It is not a tutorial
on how to tune a motor. It is the answer to two other questions -- **what makes
this procedure valid on the next machine**, and **when it must refuse to hand back
a number** -- and it is written down because a procedure that exists only in
someone's habits cannot be replayed, reviewed, or contradicted.

The executable form of everything below is
``test_package/test/unit/test_sop_outer_loop.cpp``, with the shared measurement
bench in ``sil_sop_bench.hpp``. Where this page and that driver disagree, the
driver is right and the page is a bug.

Why a procedure rather than an instance
---------------------------------------

The library cannot ship gains for your machine, and shipping a table of gains for
*one* machine would be worse than shipping none: it would look like coverage.
What it can ship is a sequence of steps in which every number comes from the
machine in front of you, plus the refusal semantics for the case where the data
does not describe that machine at all.

Four phases, in one order
-------------------------

.. list-table::
   :header-rows: 1
   :widths: 14 46 40

   * - Phase
     - What happens
     - Library entry point
   * - **identify**
     - Run a current staircase on *this* machine and feed the drive's own
       ``(dt, iq, omega)`` into an accumulator; identify the two ratios the
       speed loop needs -- torque constant over inertia, and damping over inertia.
     - ``SpeedTuner::accumulate()``, ``SpeedTuner::finish()``
   * - **propose**
     - Turn the ratios into gains with the same law the shipped configuration
       uses, so a tuned loop and a configured loop are the same kind of object.
     - ``SpeedTuneResult::kp_a_per_rad_s``, ``ki_a_per_rad_s2``
   * - **measure**
     - Under those gains, *measure* the closed-loop -3 dB point (sine injection
       plus bisection) and report the delivered ratio: measured over requested.
     - ``sil_sop_bench.hpp::minus3db_over_requested()``
   * - **accept or refuse**
     - Accept when the delivered ratio lands inside the band; report
       ``valid == false`` with a reason and **no usable gains** otherwise.
     - ``SpeedTuneResult::valid``, ``.reason``, ``.tolerance``

**The order is not a style choice.** Proposing before measuring is accepting your
own model with your own model; measuring before proposing lets the criterion drift
with the measurement. What makes the sequence legitimate is that the proposal uses
only this machine's data while the acceptance test is applied where the machine is
not the one that produced the gains -- see the replay criterion below.

The three criteria, and what each one can falsify
-------------------------------------------------

**1. The procedure replays on a second machine, and the criterion moves with it.**
Tune on machine A, apply to machine B, and the delivered ratio leaves the band;
tune on B and apply to B and it is back inside. The gains themselves scale with
the machine's own ratio, which is the check that the procedure read the machine
rather than memorising a number. The falsifying run: replace B with a copy of A and
both the strict inequality and the gain ratio go red at once. A criterion that
cannot be made red is a criterion that is not testing anything.

**2. Data that does not describe this machine yields no number.** On a machine far
outside the plausible range the result is ``valid == false`` with a non-empty
reason, and the gains handed back are *bit for bit* the nameplate ones the caller
would have used anyway -- so a caller that applies whatever it is given changes
nothing. The measured ratio is still reported, and it is reported as a statement
**about the measurement**, not as a gain.

**3. The objective is distance to failure, not smallness of error.** The search
maximises the margin between the identified ratio and the edge of the acceptance
band, and that margin crosses zero exactly where the library's own refusal starts.
An objective written as "smaller residual" would be a different quantity with a
different winner: on the outer loop's own grid the smallest residual and the best
held-out margin are not achieved by the same setting, and the residual wins on the
wrong side. That is the argument for the margin, and it is measurable rather than
aesthetic.

Where the acceptance band comes from
------------------------------------

The band's upper edge is the geometric mean of two measured quantities, and
neither of them is chosen by taste:

- the largest residual the gain law itself leaves across a decade of bandwidth
  (measured in the motor configuration's own cases), because a band narrower than
  the law's own spread would fail a correctly identified loop;
- the smallest nameplate error this repository already treats as operational
  (the inertia tolerance), because a band wider than that would pass a machine
  nobody has described.

Both factors and their product live in the claim row for ``core/auto_tune.hpp``.
**``tolerance`` itself is the caller's number**, not a measured bound: ``valid``
is judged against whatever the caller passed, so a caller who widens it owns the
consequence. And the delivered bandwidth is **not** 1.0 by construction -- the
band accepts *this law*, not "unity".

The second cut: search process parameters only
----------------------------------------------

A hyper-parameter search over this procedure is only meaningful if the things
being searched actually move the result, and that has to be checked before it is
searched, not after. Two facts from that check:

- **The window length is a real parameter, and residual and margin disagree about
  it.** The best residual is not the best held-out margin; the two orderings
  differ, which is the whole reason criterion 3 exists. The difference is small on
  a clean two-parameter plant and the point is the ordering, not the magnitude.
- **The injection amplitude is not a parameter at all.** Scaling it leaves the
  identified ratio unchanged, because the fit is scale invariant. Searching it
  walks a flat objective, and a flat objective makes any sampler look patient.

A deterministic grid is used deliberately. The criterion is a margin on a held-out
set; a grid makes the ranking reversal reproducible and CI-able, and swapping in a
sampler that draws from a distribution would not change a single conclusion.

What this procedure does not claim
----------------------------------

- **Only the outer loop.** The current loop is assumed fast and is not identified.
- **``tolerance`` is not a measured bound.** It is the caller's acceptance number
  (see above).
- **The delivered bandwidth is not unity.** The law is what the band accepts.
- **Not the sensorless low-speed case.** That capability's replay needs a bench
  that can be moved to a second machine (changing flux linkage or dead time moves
  its floor with them), and until that bench exists the low-speed side has a
  **falsification** rather than a procedure: an early estimate of its speed floor
  as a ratio of dead-time volts to flux linkage was measured to be a lower bound,
  and a magnet change predicted to lower it made the measured error worse at every
  speed. No page in this documentation may present that number as a boundary.
- **Not an optimiser.** The search is a grid over process parameters, and its
  output is a setting, not a model of your machine.

Reproduce it
------------

The driver builds against the conan-cached gtest and runs standalone:

.. code-block:: console

   G=$(ls -d ~/.conan2/p/gtest*/p/include | head -1)
   L=$(ls -d ~/.conan2/p/gtest*/p/lib | head -1)
   g++ -std=c++17 -O1 -fno-exceptions -fno-rtti -I include -I "$G" -I test_package/test/unit \
       test_package/test/unit/test_sop_outer_loop.cpp $(ls src/*/*.cpp src/*/*/*.cpp) \
       -x c++ src/version.c -x none "$L/libgtest.a" "$L/libgtest_main.a" -lpthread -o /tmp/sop
   /tmp/sop

It runs as part of ``ctest`` as well; the case count is in
``docs/verification-numbers.json`` so this page does not have to carry it.
