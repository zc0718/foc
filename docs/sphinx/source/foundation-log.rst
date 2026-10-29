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

Twelve of the forty-one claim rows had a basis when the graph was first built; the other twenty-nine were listed
explicitly as unmapped and the gate printed that count on every run. It is now complete -- 51 of 51 installed headers are
mapped -- and the count is still printed, because the mechanism that filled it is the same one that has to keep it full:
a row in neither list fails the build, and a new header arrives with either a basis or a stated reason it has none.

The graph is generated from the code rather than from recollection. An edge is either a compile edge (a `foc/` header
the module or its own `.cpp` includes directly) or a semantic one (another module whose value the claim quotes without
including it -- `core/pi.hpp`'s claim quotes numbers that `motor/motor_config.hpp` computes, and if that formula moves
the claim is stale while every test still passes). Six headers are roots with no incoming edge and carry a self-edge; the
asymmetry is stated rather than smoothed over.

The graph is also checked for rot: a basis entry naming a file that no longer exists, or an identifier that appears
nowhere in `include/` or `src/`, fails the build, because a decayed graph returns a confident and wrong blast radius and
that is worse than returning nothing. It caught two while being filled -- `src/core/protection.hpp`, which is a header,
and three entries that resolved to nothing.

One answer worth knowing before the next wave: **33 of the 41 claims rest on `core/types.hpp`**. That is not a surprise,
and it is the reason a change to the value layer is not a small change no matter how mechanical it looks.

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
   * - F-5
     - F
     - `benchmark/script/run_pil.py` lists the library's non-template translation units by hand, and never gained
       `src/core/position_reference.cpp` when W7-a turned `PositionReference` into a non-template. The PIL leg has not
       linked since that commit.
     - Wiring the `pil` gate and running it: two undefined references at link time. Nothing had executed the script since
       W7-a, so "the PIL leg exists" was true of the file and false of the build -- a distinction no amount of reading
       would have settled.
     - The whole PIL leg. Rebuilt and re-run: 17 metrics compared, the target reproduces the host, worst relative
       difference 5.96e-7 against a 1e-3 limit.
     - ``pending``
   * - A-7
     - A
     - The breadth plan said the PIL leg "exists, and compares host with qemu point by point", because the two files
       (`run_pil.py`, `pil_main.cpp`) were present. It had not compiled for three waves.
     - Running it instead of listing it. The plan's Phase 0 wording changed from "put PIL in a gate" to "fix PIL, then
       put it in a gate", which is what the work actually was.
     - One plan sentence, and the next row's whole reason for existing.
     - ``pending``

   * - A-8
     - A
     - I sized the trace record at 32 bytes in the header comment and in the case that pins its size. It is 36: four
       bytes of timestamp, **seven** floats, two bytes of state. The wrong number came from counting six floats in a list
       of seven, and it had already been copied into two files before the assertion noticed.
     - The assertion itself, `EXPECT_EQ(sizeof(TraceSample), 32U)`, which reported 36 on its first run. Nothing else
       would have: a struct that is four bytes larger than documented changes how many periods fit in 2 KiB, and no other
       case cares.
     - Two comments and one test constant. Which is the point of pinning a size at all -- the claim "2 KiB holds N
       periods" is now arithmetic nobody has to remember.
     - ``pending``

   * - F-6
     - F
     - `foc_bridge_get_setup` copied seventeen fields of the setup and silently dropped the six that had been appended to
       `foc_motor_setup_t` over three waves: `observer`, `inject_v`, `inject_hz`, `blend_start_rad_s`,
       `blend_end_rad_s`, `handover_lpf_hz`. `Motor.setup()` -- the first place a support engineer looks -- therefore
       reported the *defaults* for exactly the fields that choose the observer, while `status()["observer"]` reported the
       truth from the live object.
     - Building the first golden window: the readback said `observer=BACK_EMF, inject_v=0` while `status()` said `HFI`
       for the same motor. No test caught it because every readback case asserted only pre-append fields -- the test
       asked whether the fields it knew about came back, never whether the struct came back.
     - One function, plus a case that sets every field and demands all of them back, so the next appended field is
       caught in the same place it is added.
     - ``pending``
   * - F-7
     - F
     - Selecting the injection observer through `Motor` applied **no carrier at all**, on any axis whose HAL supplies an
       angle. `tick()` took the sensor branch, so the observer was never stepped; an observer that is never stepped never
       advances its carrier phase, and a carrier frozen at phase 0 sits at a zero crossing, so `injection_ab()` returned
       exactly 0.0 V. The mode was admitted, reported as ``HFI`` by `status()`, and inert -- every surface agreed with the
       caller except the inverter. The C host path always supplies an angle (`read_angle_mech_rad` is required by the HAL
       contract), so this was the whole Python face, not an edge case.
     - Trying to record a golden window for the injection path: the duty swing over 3000 periods was 0.00000 while
       `status()["observer"]` said `HFI`. Instrumentation, not inference, then narrowed it: the `SwitchableObserver`
       standalone injects ±9.56 V (so the slot forwards), the composed `HfiObserver::update` **never printed** (so it was
       never called), and `injection_ab()` read 0 because `phase_applied_` had never moved. No existing case caught it
       because every ABI case asserted admission and the reported mode, and the W7-c SIL work drove `HfiObserver`
       directly -- the composed path was covered by nothing.
     - Two gates in `tick()`, plus one case that asserts the carrier reaches the inverter on a sensored axis and one that
       keeps asserting it on a sensorless one. The observer is now stepped whenever it injects
       (``speed_from_observer || injecting_now()``), and the carrier is added on the same condition. The sensor keeps the
       commutation angle and the published speed, so an encoder rig gets a hybrid drive rather than losing its angle to a
       carrier.
     - ``pending``
   * - F-7a
     - A
     - The first fix attempt widened the carrier gate alone (``speed_from_observer || injecting_now()``) and the case
       passed -- which proved nothing, because the observer's phase starts at 0 and the injection was zero either way.
       Reverting the fix left the case green: an unfalsifiable test standing in for a fixed defect.
     - Reverting the fix and re-running it. The criterion has to be able to go red; the way to know is to make it.
     - The second attempt. Then the two halves were shown to be independently load-bearing: reverting the *step* gate
       alone (carrier gate left open) gives a swing of 0.0000, and reverting the *carrier* gate alone gives 0.0000 as
       well. Both were needed, and neither was detectable from the diff.
     - ``pending``
   * - F-7b
     - A
     - The second fix attempt refused the mode at `init()` when the observer type could inject -- using
       ``has_injection_v<Observer>``, which is a property of the **type**, not of the selected mode. Every motor whose
       observer could switch modes was refused, and 38 cases went red. Narrowing the probe to ``injecting()`` (the mode)
       brought it to 6, still the wrong answer: the refusal was treating an inert carrier as correct by refusing to run.
     - The test suite itself, in the same run. The trait answered "can this type ever inject", the question was "is this
       mode injecting now", and only the test count separated the two.
     - The refusal, removed. A mode that is admitted must run; refusing admission is how a defect becomes a documented
       limitation.
     - ``pending``
   * - A-9
     - A
     - The probe that first measured F-7 ran with `pwm_freq_hz = 0` in the setup, which drives the carrier phase
       increment to zero on its own. The 0.00000 it reported was therefore guaranteed before the defect was consulted --
       and the same rig, after the fix, still read 0.00000 at 6 kHz and would have looked like a failed fix.
     - Sweeping the carrier rate against an explicit `pwm_freq_hz`: the same code, one knob changed, went from 0.00000
       at 1/2/4/5 kHz to 0.60/0.65/0.66/0.67. A probe is only worth its reading once it has been shown able to produce a
       non-zero one.
     - The F-7 verdict, re-measured. Pre-fix 0.00000 at all four rates with `observer = HFI`; post-fix 0.45-0.67 across
       eight rate/PWM combinations. Same window, one knob, both directions.
     - ``pending``

   * - F-8
     - F
     - `benchmark/script/foc_sil_plant.py` added the **mechanical** acceleration `torque / j` to `omega_e`, which is an
       **electrical** speed, and then divided by the pole-pair count when reporting mechanical speed. Every mechanical
       transient in the Python plant was therefore `pole_pairs` times too slow -- measured **7.002x** at seven pole
       pairs on an open-loop acceleration test against `T / J`.
     - The analysis layer's analytic criterion, on its first run. The fitted bandwidth of a speed step on the Python
       face read **17.78 rad/s against the 250 rad/s the setup asked for**. Varying one knob at a time separated the
       suspects: the fitted bandwidth scaled exactly as `1 / kt` and exactly as `j`, which is the gain law behaving
       correctly on a plant that was too slow to respond, not a gain law that was wrong. A two-line open-loop test then
       produced the number 7.002.
     - Every mechanical quantity ever derived from that plant. The golden ledger's `speed_step_100` settled in 0.2 s
       (the whole window) and now settles in 0.0211 s; its "overshoot" was a 5.6 % *undershoot* of a loop still
       crawling, and is now +0.005 %. The three windows were re-recorded, and every figure that had been quoted from
       them replaced.
     - ``pending``
   * - F-9
     - F
     - The same plant integrated a fixed 50 us (`DT`) per `write_duty` call, while `golden.py` ticks at 100 us. The
       model advanced half a period per call, so simulated time ran at **half speed** -- an exact factor of two on top
       of F-8's seven.
     - Reading the constant after F-8's factor of seven did not explain the whole 14x: 250/7 = 35.7, and the
       measurement said 17.8. `grep DT` found `50e-6` against the caller's `1e-4`.
     - The plant now takes `dt_s`, `golden.py` carries one `DT_S` that both the loop and the plant use, and the two
       development scripts that already ticked at 50 us are unaffected. Together F-8 and F-9 account for 14.05x, and
       the corrected measurement is 243.57 rad/s against 250 (-2.57 %).
     - ``pending``
   * - A-10
     - A
     - The ledger could not have found either defect, and it was not asked to: it compares the Python plant against a
       recording *of the same Python plant*. A self-referential baseline detects regressions and is silent about initial
       errors, however long it is stared at.
     - The analysis layer's second opinion -- the gain law's analytic bandwidth on one side, the C++ bench's independent
       plant on the other. Neither shares a line of model code with the Python plant, and the disagreement was 14x.
     - The reason 1.3 was worth doing before more scenarios were added: a ledger with one implementation can only say
       "unchanged", and "unchanged" is how a wrong model becomes a specification.
     - ``pending``

   * - A-11
     - A
     - The absolute 10 % criterion was written into the plan, met at 9.15 %, and looked like a passing gate. Scaling the
       speed loop's proportional gain by 0.8 -- a 20 % error in the gain law the criterion exists to check -- moved the
       reading to 225.31 rad/s, i.e. **-9.88 %, still inside the band**. The estimator's own +9.15 % bias was the same
       size as the defect.
     - Perturbing the thing the criterion is about, rather than trusting it because it went green. A tolerance met by
       an estimator whose bias is comparable to the effect has almost no discriminating power, and no amount of reading
       the tolerance reveals that -- you have to break the system and watch.
     - A second criterion that cannot be hidden by the bias: request 125 and 250 rad/s and demand the measurements move
       by the same factor (measured 134.66 and 272.87 rad/s, ratio 2.026). It catches a loop that ignores the request,
       not a scale error -- the frequency sweep that catches scale errors is the next increment, and this row is why it
       is the next increment.
     - ``pending``

   * - A-12
     - A
     - The first version of the sweep found the -3 dB crossing by interpolating in log-log space between 0.5 and 2
       times the bandwidth, and reported **180 rad/s for a loop whose answer is 235** -- a 28 % error, made entirely of
       interpolation. Interpolating a first-order magnitude between two points that far apart is not a straight line:
       reduced per point (wb = w / sqrt(1/g^2 - 1)) the same four measurements give 236.4, 235.8, 236.4 and 238.3.
     - Reducing every measured point individually instead of trusting the interpolant. The four values agreeing to 1 %
       is what proved the loop is first-order at the Python face and the error was the arithmetic between the points.
     - The -3 dB definition: the sweep now bisects, which assumes nothing about shape. The C++ bench turns out *not*
       to be first-order (its per-point equivalents fall 269.5 to 188.8), which the interpolation had been hiding.
     - ``pending``
   * - A-11b
     - A
     - A-11 said the frequency sweep would catch what the absolute step band cannot, and left it as a claim about a
       future instrument. It is now measured on the same defect: scaling the speed loop's proportional gain by 0.8
       moves the sweep's crossing from 234.91 to **210.79 rad/s (-15.68 %, outside the 10 % band)** while the step fit
       moves from +9.15 % to -9.88 %, still inside its own. Same defect, two instruments, one blind and one not.
     - Running both criteria against the same perturbation, which is the only way to know which one is carrying the
       claim.
     - The sweep is now the criterion that carries it; the step response stays because it says something different
       (how the servo behaves, not where its corner is).
     - ``pending``

   * - A-13
     - A
     - The first Monte-Carlo criterion -- "with `sigma = 0` the harness must reproduce the nominal measurement bit for
       bit" -- called one trial twice and compared the two. It was green, and it could not fail: it never went through
       the sampler, so the bug it exists to catch (a sampler that perturbs by a hair when asked not to) was invisible
       to it. Drifting the sampler to add 0.001 to sigma left the case green.
     - Drifting every criterion and watching which ones stay green. Three of the four drifted cases went red; this one
       did not, which is how the hole was found.
     - The criterion now runs the *stream*, so `draw()` is in the path: the same 0.001 drift produces 244.4909 against
       244.4273 and the case fails. Third instance of one family -- A-6 ("a control that is never exercised"), A-11 ("a
       band the defect fits inside"), A-13 ("a criterion that bypasses the code it is about"). The common cause is that
       a green test is evidence of nothing until it has been made to go red.
     - ``pending``

   * - A-14
     - A
     - The first version of the torque map's arithmetic case asserted that a positive `id` *raises* the torque on an
       `ld < lq` machine. It lowers it: the reluctance term is `1.5*pp*(ld-lq)*id*iq`, so the sign follows `ld - lq`
       and the usual interior machine has `ld < lq`. The map was right and the case was wrong.
     - The case failing against arithmetic that was already correct, and the analytic expression it was comparing with.
       Nothing in the map had to change.
     - Two assertions, and one more reason the case states the expression instead of quoting a number: a quoted number
       would have been edited to match the wrong sign.
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
