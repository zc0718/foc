_`Foundation corrections`
=========================

Every claim in this documentation rests on a measurement, and every measurement rests on code that could be wrong. This
page is the record of the times it was: what broke, how it surfaced, what it invalidated, and what was re-run afterwards.

It is **append-only**. A row is never deleted or softened, including the rows that are unflattering -- a wave's
proportion of "the library was wrong" to "I was wrong" is information about the library, and erasing it would destroy
the only evidence there is.

Why a page rather than discipline: the failure this guards against is not forgetting a bug, it is a wrong number
becoming *reference data*. Once a figure is written into a comparison or a golden trace, every later check endorses it,
and the error stops looking like an error. A row here is what lets a reader ask "was this measured before or after that
was fixed".

Three classes, and they are deliberately not merged
---------------------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 12 88

   * - Class
     - What it means and what it costs
   * - **F**
     - The library was wrong: a computation, a contract, a constant that does not exist. These earn unbounded effort,
       because a wrong foundation multiplies -- every capability built on it inherits the error and the evidence for it.
   * - **C**
     - The criterion was wrong: it did not bind to the physics it claimed to test, or the tolerance was chosen after
       seeing the number. Fixing this is re-deriving the tolerance and writing down where it came from, which is slower
       than widening it.
   * - **A**
     - The analysis was wrong: the measurement was fine and the reading was not. The fix is to the method, never to the
       code -- patching code because a reading was misread damages a foundation that had nothing wrong with it.

The distinction is the whole point. Treat an **A** as an **F** and the library collects defensive patches it does not
need; treat an **F** as an **A** and a real defect is recorded as a personal slip and left in place.

How the blast-radius column is produced
---------------------------------------

Not by memory, and not by reading the diff and guessing. The command is

.. code-block:: console

   python3 .github/misc/gates.py claims --impact <path or identifier>

which reads the basis graph in `.github/misc/claim-basis.json` and prints every claim that rests on that thing, together
with the cases to re-run. `evidence` in the README answers "what checks this claim"; `basis` answers "what would
invalidate it", and the two are different -- a claim can have evidence that passes while resting on a plant model that is
wrong, which is the W7-c failure exactly.

Twelve of the forty-one claim rows have a basis so far. The other twenty-nine are listed explicitly as unmapped, the gate
prints that count on every run, and a row that is in neither list fails the build -- so the graph cannot fall behind the
table in silence. The graph is also checked for rot: a basis entry naming a file that no longer exists, or an identifier
that appears nowhere in `include/` or `src/`, fails the build, because a decayed graph returns a confident and wrong
blast radius and that is worse than returning nothing.

The log
-------

.. list-table::
   :header-rows: 1
   :widths: 6 5 30 27 20 12

   * - ID
     - Class
     - What was wrong (root cause)
     - How it surfaced, and what would have shown it first
     - Blast radius and re-run
     - Commit
   * - F-1
     - F
     - `foc_bridge_hfi_max_inject_hz` was declared in the wrong place twice: once outside the `extern "C"` block, so
       the name was mangled, and once inside `#ifdef __cplusplus`, so a C translation unit could not see it at all.
     - The pure-C consumer `test_package/c_api_validation.c` failed to link. That file exists precisely to catch this
       class, and it did -- after three attempts, because each fix was written before the reason for the failure was.
     - None beyond the build.
     - ``4c46351``
   * - F-2
     - F
     - The SIL bench called `reset()` before every step, which threw away the intra-period high-frequency content --
       the injection carrier. The observer was being handed a frozen carrier and reported it as a measurement.
     - The envelope bench produced three mutually inconsistent readings in a row (a speed-PI artefact, a pi-mirror
       lock, a degenerate standstill) and each was explained by a different cause. The real cause was one line in the
       harness; the fix was `set_mechanical_speed()`, which writes the rotor state without resetting.
     - The entire low-speed envelope table. Everything in it was re-measured.
     - ``0e01269``
   * - F-3
     - F
     - Nine build-output files under `build/` were tracked, so the tree was permanently dirty and a real change could
       hide inside a phantom diff.
     - `git status` showed modifications with no source edit. Untracked, and `/build/` added to `.gitignore`.
     - None; no claim depended on it.
     - ``bbbce46``
   * - F-4
     - F
     - The new `fmu` gate called `pathlib.Path` with `pathlib` never imported, so the gate died with `NameError` on the
       path that should have verified the FMU.
     - Running the gate. A gate that has never been seen to pass is not a gate.
     - None.
     - ``bbbce46``
   * - C-1
     - C
     - "418 Hz" was reported as a crossover frequency by counting zero crossings of a signal that had three tones in
       it. A crossing counter has no meaning for a multi-tone signal; the number was arithmetic, not measurement.
     - The tone decomposition, done exactly with a six-bin Goertzel, showed the carrier at 2000 Hz dominant at 0.04177
       published / 0.03835 rotor, and two smaller components.
     - That conclusion only. Re-measured with the exact transform.
     - ``16e672f``
   * - C-2
     - C
     - Case C4 compared a sensored arm against an injecting one, but the sensored arm also removed the injection -- and an
       injection observer that is not stepped freezes its carrier into DC. Two things differed, so nothing was isolated.
     - Re-reading the arm construction. The criterion asked "does removing the injection change the ripple", and the
       arm changed two inputs to that question.
     - C4 withdrawn rather than re-interpreted.
     - ``6bc2dfe``
   * - C-3
     - C
     - Case C12's first knob scaled `inject_v` with `lq` to hold the ratio :math:`V/(\omega L)` constant. But the
       injected current flows on the *d* axis, which is set by `Ld` -- so the knob moved the current *and* the saliency,
       and tested neither.
     - Nothing: the case "passed". It was caught by asking what the knob was actually varying, which is the question
       this page exists to keep being asked.
     - C12 redone with one knob (same volts, half the saliency): ripple indifferent, 0.97 of the original.
     - ``16e672f``
   * - A-1
     - A
     - I "corrected" a correct case count (455) to an incorrect one (454) by counting `^TEST(` macros, which misses the
       two consumer targets.
     - "N cases" means what `ctest` reports as its Total. Verified with `ctest -N`, which prints 455 = 453 gtest cases
       plus `c_api_validation` and `python_bindings`.
     - One sentence of documentation.
     - ``bbbce46``
   * - A-2
     - A
     - The breadth plan said the Python face exposed "29 of 34 ABI functions, missing `set_observer`, `*_pi_gains`,
       `axis_deadtime_v`". All three of the named functions had been bound for two commits.
     - Counting short names instead of counting what is reachable: `create`/`destroy`/`set_*`/`get_state` are reached
       through the `Motor` object, not by a module-level function of the same name. Measured properly: 36 functions, and
       exactly two unreachable (`foc_motor_set_observer`, `foc_fault_name`).
     - The scope of the increment. Two bindings instead of four, and a correction in the plan rather than four days of
       work on a problem that did not exist.
     - ``be3a14c``
   * - A-3
     - A
     - Filtering the HFI loop's proportional path inside the PLL removed the loop's damping. Every cutoff from 50 to 400
       Hz ran the axis away, -115 to -170 rad/s.
     - The sweep itself: twelve points, all unstable. Moving the filter to the publication boundary (after the
       integrator, on the published speed only) left the loop stable at every cutoff and cut the carrier tone 10.1x.
     - One failed design attempt, now recorded as falsified rather than silently replaced.
     - ``0d739e4``
   * - A-4
     - A
     - I wrote 768 lines of build script, wrapper and model description for an FMU export that had already shipped in
       ``2feea62``, because I did not look before starting.
     - Diffing the two wrappers: the difference was comment wording. Deleted rather than merged, since it added nothing.
     - Half a day. The path is now ignored so the same mistake cannot recur silently.
     - ``bbbce46``, ``73ef1ce``
   * - A-5
     - A
     - The scaling invariant was stated as "the electrical parameters scale": `Rs`, `Ld`, `Lq`, `kt` and `v_bus` were
       multiplied by `k` while `J` and `B` were left alone. Torque then scaled and inertia did not, so the speed the
       RK4 stages carry *within* a period stopped being proportional, and the back-EMF term with it.
     - The invariant itself: the trajectories differed by up to 3.2e-3 A at `k = 25`, growing with `k`. Scaling `J` and
       `B` too collapsed the residual to float resolution. The symmetry is not "the electrical parameters scale", it is
       "everything with a dimension scales".
     - None -- found before the case was committed, and the criterion was corrected rather than widened.
     - ``pending``
   * - A-6
     - A
     - Two counter-examples of mine were never actually exercised, so they "passed" while proving nothing: one compared
       only the settled current (an integrator absorbs a gain error completely by steady state; the measured difference
       was 4.8e-7, indistinguishable from exact), and one swept the input up to the clamp limit but never past it, so the
       branch it was meant to catch never fired.
     - Writing the counter-example down and asking what it would look like if it failed. Both were fixed by moving the
       comparison into the transient (periods 1, 5 and 25) and by letting the sweep cross the limit.
     - None. But the lesson is load-bearing: a control that is never exercised looks exactly like a control that passes.
     - ``pending``

Two kinds of row that are currently empty
-----------------------------------------

The page is honest about its own coverage too. There are two kinds of row this log expects and does not yet contain, and
their absence is a fact about the work so far, not a guarantee that the categories are empty:

- **A claim that shrank.** When a defect cannot be found within three attempts, the rule is to stop and lower the claim
  (an L3 becomes an L2, or becomes "not claimed") instead of shipping a number nobody can explain. That has not happened
  yet -- which means no claim here has been tested to destruction.
- **An unexplained pass.** A case that is green for a reason the author cannot state is treated as red. Nobody has
  written one down yet.

Both will be rows when they happen, with the same weight as the defects.
