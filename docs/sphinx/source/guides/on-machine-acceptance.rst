_`Acceptance on your machine`
=============================

The task: you have a board, this library, and no interest in trusting either one on faith. This page
is the procedure that decides whether the pair works. It is written for the person who will run it,
not for the person who wrote the library.

Three things are true before you start, and they are worth reading twice:

- **This procedure is ours; the result will be yours.** Nothing on this page has been run on
  hardware by this project -- see :ref:`what has actually been run <platform-evidence>`. What the
  library ships is host-and-SIL evidence plus this list. When you run it, the board-level evidence
  becomes yours, and it is worth the paper it is written on in a way ours could not be.
- **Every number below is an expectation from the simulation bench, not a fact about your machine.**
  A different motor is a different machine. Where a number appears, it is there so you know what
  "roughly right" looks like; where it conflicts with your datasheet, your datasheet wins and the
  number is the thing that needs explaining.
- **The library judges, you paste the verdict.** Every estimator behind these cards answers with a
  `valid` flag and a cause. A card passes when the code says it passed. "The waveform looked smooth"
  is how an expert reaches a conclusion; it is not available here, and it is not needed.

Do them in order
----------------

H0 comes first because everything after it assumes the board tells the truth. A card that fails
invalidates the ones below it, not the ones above it.

.. list-table::
   :header-rows: 1
   :widths: 8 26 34 32

   * - Card
     - What you do
     - It passes when
     - If it fails, look here first
   * - **H0**
     - Wire up, power at 1 A, run ``board_check.hpp``'s self-test over every item, print
       ``board_cause_name()``.
     - Every item reports ok and the cause is none.
     - The cause names one thing on purpose: a reversed current polarity is a swapped lead or sensor
       direction; a current offset means doing H4 first; a dead-time complaint means the PWM setup
       or the bus scaling; a sampling-instant complaint means the ADC trigger is not at the centre
       of the zero vector.
   * - **H1**
     - Lock the shaft, sweep six d-axis voltages through the resistance estimator, fit the line.
     - Positive resistance within 30 % of the nameplate, and the intercept (the volt-seconds the
       dead time eats) between 0.45 V and 0.75 V. The bench expects 0.55–0.65 V at a 12 V bus and
       1 µs.
     - A resistance that is negative or three times out means the bus scaling is wrong. An intercept
       near zero means the dead time never made it into the bridge.
   * - **H2**
     - Turn the rotor by hand to eight positions, step the d-axis at each, fit inductance against
       angle.
     - Both inductances positive and the sinusoidal fit within 10 % residual. The bench expects the
       ratio between 1.0 (surface magnet) and 1.8 (interior magnet).
     - A poor fit means the eight angles were uneven or the current had not settled. Too small an
       inductance means the step was shorter than the estimator needs.
   * - **H3**
     - Force rotation with a fixed voltage vector, no current loop and no observer, and fit the
       flux linkage from the back-EMF at two speeds.
     - The two speeds agree within 5 %, and the result is within 10 % of what the torque constant
       implies.
     - A linkage that reads low by a fifth means the speeds were too low: the back-EMF has to stand
       clear of the dead-time plateau to be measurable.
   * - **H4**
     - With the shaft locked, compare the library's current readings against the supply's own
       display at two or more points, and fit gain and offset.
     - Gain between 0.9 and 1.1, offset under 100 mA.
     - A gain that is off invalidates **every current number you have measured so far**, including
       H1's resistance. Redo them.
   * - **H6**
     - Lock the shaft and sweep q-axis current from 0.2 A to the drive's maximum, measuring
       incremental inductance at each point.
     - A monotone fall, with more than 5 % between the ends.
     - No fall is not a failure: it says this motor does not saturate and the nominal model is
       enough. Write it down and skip the saturation features.
   * - **H7**
     - Lock the shaft and sweep 20 mA to 1 A in sixteen points, taking the difference between
       commanded and resistive voltage at each.
     - The curve flattens toward zero at low current and saturates by the drive's dead time above
       roughly 0.1 A -- the shape the bench measured, not a straight ramp.
     - A genuinely wide linear ramp contradicts the bench's model, and the compensation constant the
       simulation chose for this shape would be the thing to revisit. Do not adjust it by eye.
   * - **H5**
     - Optional and last: current step, speed step, then compare the sensorless angle against an
       encoder as truth.
     - Steady-state error sign agrees with the bench and the magnitude is within a factor of two.
     - This is the card a non-specialist is most likely to get stuck on. The first action is to
       re-check that H0–H4 are green rather than to tune anything.

Three rules that keep the results usable
----------------------------------------

1. **One variable per card.** The most expensive mistake is a card that quietly changed two things --
   a rotor that found a new working point while you thought you were holding it still. It happened
   to us on the bench, and a real motor does it more readily.
2. **Write the criterion before you run.** Each card above has a pass condition and a first thing to
   check; if you change either, change it *before* the measurement, not after you have seen the
   number.
3. **Believe the verdict, not the waveform.** The estimators answer with a flag and a cause. A card
   whose numbers look plausible and whose flag says invalid has failed.

What this procedure does not reach
----------------------------------

Four things stay out of reach even with a board and these cards; they are on
:doc:`../boundaries` as well, because a limit that lives only on a procedure page is a limit people
will meet in the field:

- Above roughly 800 rad/s electrical with the small motors this was designed around: the machine
  cannot get there, so the high-speed conclusions stay where they are.
- Intermittent faults: they need hours-to-days of reproduction, and a person who has not seen the
  failure before cannot be asked to arrange it.
- Thermal, EMC and long-term ageing: no chamber, no instruments, no years.
- The absolute parameters of one particular industrial motor: what H6 gives you is the *shape* of
  saturation, which transfers. The numbers do not.

What you do have afterwards
---------------------------

Six assertions that were host-and-SIL claims become statements about your machine: the resistance
estimate, the dead-time volt-seconds, both inductances and their ratio, the flux linkage, the current
sensor's gain and offset, and the shape of the dead-time curve. The observer's accuracy -- the single
most load-bearing claim in the library -- becomes yours only after H5.

None of that changes a claim row in this documentation, and it should not: the tree can only speak
for what it has measured. What changes is your own evidence, and the moment you have it, you know
something about this library that we do not.
