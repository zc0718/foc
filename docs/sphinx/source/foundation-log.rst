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

   python3 .github/project/gates.py claims --impact <path or identifier>

which reads the basis graph in `.github/project/claim-basis.json` and prints every claim that rests on that thing, together
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

One answer worth knowing before the next wave: **156 of the 169 claims rest on `core/types.hpp`**. That is not a surprise,
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

   * - C-4
     - C
     - The graded protection ladder had a case for the derating curve (`derated_limit(T)`, a pure function) and a case
       for the trip, and nothing in between: no case ever stepped the protector with a temperature *inside* the band, and
       no case anywhere asserted `active_mask`, so the enum-order ranking the header documents was unexercised too.
     - Writing the two cases the plan's own criterion named -- "each rung has a case, and an implementation that trips
       without derating must be red" -- and then building exactly that wrong implementation: replacing `t_trip_c` with
       `t_derate_start_c` in the over-temperature condition left **seven of eight** protection cases green. Only the new
       ladder case failed.
     - Two cases, and the `todo` in the capability matrix's boundary column that had been standing in for them is now a
       case name. The rule this row is about: a capability with a curve test and a trip test can still have no test of
       the thing it is for.
     - ``pending``

   * - A-15
     - A
     - Building the stiction rig I wrote a "current source" that nulled `diq` **at the current the plant happened to
       have**. That is a free integrator, not a source: it holds the current wherever the transient left it. The case's
       own arithmetic caught it twice -- 37.85 rad/s against a promised 40.00 with the d current drifting, then 35.49
       with the d current pinned, because `iq` had settled at 0.90988 A against a 1.0 A setpoint.
     - Printing what the plant actually ended with (`id 0.00700 iq 0.90988`) instead of reasoning about the voltage.
       Two fixes were guesses at the voltage; the third read the state and the answer was in the current.
     - The source now feeds the **setpoints** with the d-q cross-coupling: steady speed 39.997 against 40.000, `iq`
       1.00000, torque 0.050000. The lesson: regulating a derivative is not regulating a value, and a case whose
       arithmetic is exact will say so.
     - ``pending``

   * - C-5
     - C
     - The first version of the breakaway case could not tell "it started" from "it is still being held": the plant's
       stiction band was 0.5 rad/s and the sequence's release threshold was also 0.5 rad/s, so the two thresholds
       described the same speed and "last torque at rest" and "first torque moving" printed the same number. Separately
       the handover value carried 5.4 ms of ramp -- 2.7 mN*m -- because the rotor has to accelerate to the release speed,
       which was larger than the quantity the case was checking.
     - Printing both numbers and noticing they were identical, then asking what each threshold means: the band is a
       numerical necessity at the zero crossing, the release speed is a physical claim about motion. They must differ by
       a decade, and the handover's lag must be stated rather than absorbed by the tolerance.
     - The case now separates them (band 0.01, release 0.05 for the plant run) and states the observation lag instead
       of hiding it: 1 mN*m of slack, measured at 0.5 mN*m.
     - ``pending``

   * - A-16
     - A
     - Editing the plan I located the 3.2 row with `line.startswith('| 3.2 |')`. That pattern also matched the **ROI
       audit table's** own `| 3.2 | 1.00 | 1.50 | 低估 |` row, which sits two hundred lines earlier, so the two new
       rows landed inside the audit table and the real 3.2 row in the Phase 3 table was never touched. The script
       printed "3.2 closed" and I believed it; the plan still said the work was undone, and the user found it, not me.
     - Reading the artifact back instead of the tool's own success message. `grep '^| 3.2'` on the finished file would
       have shown two rows in one place and none in the other.
     - The two rows moved to where they belong, the audit table's row restored, and the plan now states the ordering
       policy it was silently applying. The rule: **a tool's "done" is a claim about the tool, not about the file** --
       the same shape as A-4 (did not look before starting) and A-13 (a criterion that bypassed its own subject), and
       the same fix: look at the artifact.
     - ``pending``

   * - C-6
     - C
     - The criterion for "a dead Hall sensor" was written as "somewhere two bits change at once (6 -> 4), which is the
       trace of the missing sensor". Measurement says otherwise: the dead bit is **frozen**, so every surviving
       transition still flips exactly one bit and the code walk degenerates onto a smaller Gray cycle instead of
       breaking the rule. The criterion would have gone red on a *correct* model, and the natural repair for a red
       case is to change the model -- which would have broken a model that was right.
     - Measuring the model in a throwaway script before the first compile: the flip counts for the stuck case read
       `[1,1,1,1]`, not `[1,2,...]`. Reasoning would have kept the wrong signature, because "a sensor is missing so
       more than one thing must be wrong" is a plausible-sounding sentence.
     - The criterion, replaced by three facts measured instead: one 60-degree illegal window per revolution (one
       sixth of the time), runs of 120 degrees (the edge interval doubles), and three distinct legal codes left. Plus
       the assertion that adjacency **survives** -- the useful negative result, since it says an adjacency check cannot
       be the fault detector.
     - ``pending``
   * - C-7
     - C
     - The quantisation case assumed the sector layout instead of reading it: it took `theta = 0..60 deg` to be one
       sector. Sensors at 0/120/240 degrees put their edges at 30/90/150/..., so **no** sector begins at zero; the case
       straddled an edge and found two codes 59.96 degrees apart, then reported it as a model error.
     - The case went red only after the sweep bug (A-17) was fixed; before that every case failed on the count and the
       layout assumption stayed hidden behind it.
     - The sector is now taken out of the sweep (the widest measured run), so the case tests the sensor and not the
       arrangement. A model whose edges are somewhere else now passes it, which is the point.
     - ``pending``
   * - A-17
     - A
     - The sweep that measures the sectors ran **two** revolutions: it computed `samples = (2.0F * kTwoPiRad) / kStep`,
       and `kTwoPiRad` is already 2*pi, so every count in the file was double six edge crossings instead of six, twelve
       sectors instead of six, and a wrap-merge that had nothing to do.
     - The very first assertion: `runs.size()` read 12 against an expected 6. Five of the six cases in the file failed
       at once, and the sixth (no illegal code anywhere) passed -- which localised it to the counting rather than to the
       model, since the illegal-code sweep does not depend on where the revolution ends.
     - One factor removed. The lesson recorded here is why the count was written as an absolute number: a `>= 6` would
       have been satisfied by twelve and the whole file would have read double forever.
     - ``pending``
   * - F-10
     - F
     - `HallTracker` stored the first sample's timestamp as if it were an edge. The rotor can be anywhere inside a
       sector when the first sample is taken, so the interval from that sample to the first real edge is short by an
       unknown amount -- and it became the reference every later interval was judged against.
     - A perfectly healthy sensor set reported `State` on its second sector: 23 samples of a 1280-sample run at
       120 rad/s. The case that found it was the one asserting `stale_fraction == 0` on a healthy run, which is the
       negative half of the fault criterion and the only reason the startup path was exercised at all.
     - Intervals count only when both of their ends were edges.
     - ``pending``
   * - F-11
     - F
     - The reference period was updated from any interval that arrived, including one that was evidence of a fault.
       Two instances, opposite directions: a **doubled** interval adopted as the new period let a dead sensor become the
       tracker's idea of normal (36 % of a revolution reporting `Ok` with an error up to 120 degrees), and a **short**
       one -- six samples of a sliver-thin but genuine sector just after a fault cleared (375 us against a 3.5 ms true
       period) -- poisoned the reference so that every real interval afterwards looked late, i.e. `State` forever.
     - Measuring the stuck-sensor case rather than assuming the fault channels covered it, and tracing the recovery
       path sample by sample when the status refused to return to `Ok` after the fault cleared.
     - The interval test is now a two-sided band around the last period, and an interval outside it is refused instead
       of adopted. Measured effect on the dead-sensor run: the undetected fraction fell from 36 % to 8.4 % without the
       false alarm on a healthy set changing at all (0 of 1280).
     - ``pending``
   * - F-12
     - F
     - Two things about a transition that crosses a stretch of illegal reads. The crossing happened without being
       seen, yet the interval spanning it was spent as a measurement; and the direction was taken from the shorter arc
       of the code jump, which for a four-sector gap reads as two sectors backwards.
     - The dead-sensor trace: the recovery edge was 119 degrees from the truth while the status said `Ok`.
     - A crossing during an illegal read is now marked unobserved and its interval refused; and when the jump is too
       far for the short arc to be meaningful, the direction falls back on the sign of the last known speed. Worst
       undetected error 120 -> 59 degrees on the low-stuck sensor.
     - ``pending``
   * - F-13
     - F
     - The transition that carried a refused interval reported `Ok`. The verdict was stored, and then the very sample
       that produced it returned `Ok` unconditionally, so the next period said `State` and the one sample in between --
       the one holding the largest error of the run, 119 degrees -- claimed to be a measurement.
     - Reading the trace rather than the aggregate: the worst undetected sample sat exactly on a transition, which is
       where the status assignment and the verdict were separated by one line.
     - The status of a transition now follows its own interval verdict.
     - ``pending``
   * - C-8
     - C
     - Three claims about `HallTracker` were wrong before any code shipped: that the angle "freezes" during an illegal
       read (it free-runs and is *clamped*, because the angle is a function of time); that the two fault channels
       "cover" a dead sensor (they cover roughly two thirds of a revolution); and that while `State` is up the angle is
       "still good to a few degrees" (at 20000 rad/s^2 it is 42 degrees out, which is what makes the flag load-bearing
       rather than decorative).
     - Each one failed its own case on the first run, in the direction of the case rather than of the code -- the
       signature of a claim written from expectation instead of from the artefact.
     - The three cases now assert the measured behaviour and quote the numbers, including the 42 degrees, which is the
       evidence for "a loop on this estimate must stop when the status says so".
     - ``pending``
   * - A-18
     - A
     - The first rig that measured the dead band was scaled so that a 20 mrad step demanded 1 N*m against a 3 A current
       limit: the loop saturated on the first period and limit-cycled at 125 mrad, and the file measured that instead of
       the friction. The module was never wrong; the instrument was.
     - The trajectory print rather than the summary: `theta` swinging between -26 and +125 mrad while the demanded torque
       sat on the clamp for most of every cycle. Two symptoms pointed the same way -- the "compensated" run did not settle
       either, and the ripple was 1.14 N*m when the term being added was 0.010.
     - A velocity damping term (as any real position loop has) and a step that stays inside the current limit. The dead
       band is set by the proportional path at rest, where the damping term is zero, so the arithmetic being checked did
       not change: 4.003 and 2.021 mrad against 4.0 and 2.0.
     - ``pending``
   * - A-19
     - A
     - I predicted that an over-estimate past the breakaway would park the axis `(coulomb - static)/kp` on the *wrong
       side* of the target. It does not: the latch turns around as soon as the demand passes the term, so there is no such
       equilibrium -- with the reference held at zero the axis simply walks away, and the size of the walk is set by where
       the loop's own demand finally balances the compensation.
     - Measuring it instead of deriving it: 114 mrad of excursion in the first attempt and 5.625 mrad of net travel in
       the second, against a predicted 2 mrad in the wrong direction.
     - The case asserts the measured behaviour. The point survives and is sharper than the prediction: an estimate above
       the breakaway is a motor, and `validate()` can only catch the version of it that shows up in the numbers.
     - ``pending``
   * - C-9
     - C
     - The first criterion for "the residual drops" was a single step's final position. It is not reproducible: where
       inside the dead band the trajectory happens to stop depends on the step size and the ringing, and the same three
       mechanisms read 2.9, 1.1 and 0.24 mrad at 20, 20 and 200 mrad steps -- a criterion that would have been "fixed" by
       choosing the step that made it look right.
     - Measuring the same mechanism at three step sizes and getting three answers, which is a property of the criterion
       and not of the mechanism.
     - The dead band itself is measured instead, by bisection on the reference step: a threshold, not a snapshot of a
       trajectory, and it lands on the arithmetic to four digits.
     - ``pending``
   * - A-20
     - A
     - The soft-limit case asserted the *ideal* invariant `v^2 <= 2*a*d` on the plant's own state, and failed by 19 mm. The
       invariant is true of the **command** from the instant the ceiling bites; the axis needs a delay before the braking
       deceleration arrives, and during that delay it keeps travelling at `v`.
     - The failure message did not say "wrong criterion", it said "the shaped run reached the wall", and the trajectory
       showed the axis pinned at maximum braking from the moment the ceiling appeared -- i.e. the loop was doing
       everything it could.
     - 19 mm at 19.5 rad/s is 1 ms of travel, so the missing term was `v*dt`. With it the ceiling becomes
       `v^2 <= 2*a*(d - v*dt)` and the axis stops 1.04 mm short. The case now runs all three: with the term, without it,
       and with no shaper at all.
     - ``pending``
   * - A-21
     - A
     - The first synchronisation measurement compared a *step* response and read 58 mrad, which I attributed to the ramp lag
       `v*kd/kp`. It was not: a step has no ramp, and what the number contained was the slave loop's own ringing for the
       first 0.09 s.
     - Measuring a ramp instead -- and, separately, feeding the gear the ideal ramp rather than the master's measured
       position -- separated the three contributions: 25.6 mrad from the formula, +4 mrad from the master's own tracking,
       and the rest from the transient.
     - The case now asserts the formula on an ideal ramp, its two scalings (`v` and `kp`), and the residue the velocity
       feedforward leaves behind.
     - ``pending``
   * - C-10
     - C
     - The criterion for the geared pair was "sync error below 2 mrad", written before the mechanism was understood. A
       perfectly geared pair cannot meet it: the standing lag is `v*kd/kp`, which is 25 mrad at the configuration used. The
       criterion was wrong by an order of magnitude in the direction that would have had me raise `kp` until the number
       passed -- buying accuracy with the stability margin, and hiding the real lesson.
     - The measurement landing 25.6 mrad against a 2 mrad criterion, and the arithmetic explaining it exactly.
     - The criterion is the formula, in both of its arguments, plus the velocity feedforward as the only term that removes
       it. "Small" is no longer a criterion anywhere in that case.
     - ``pending``
   * - A-22
     - A
     - The first high-speed ceiling case asserted the *flux* ratio against the *ceiling* ratio -- the two are inverses -- and
       expected the weakened operating point to need *more* voltage at the unweakened ceiling, when the whole point of
       weakening is that it needs less (12.22 V against a 13.86 V ceiling).
     - Both assertions failed with numbers that contradicted the claim rather than the model: 1.2019 against 0.832, and
       12.22 V where `> v_max` was expected.
     - The ceiling is now bisected on the plant's own steady state, which produced the more interesting answer: the naive
       `v_max/psi_f` is 4 % optimistic and weakening lifts the ceiling by 1.188 rather than the 1.202 the flux ratio
       promises.
     - ``pending``
   * - C-11
     - C
     - The sampling-ratio case was written with a 0.02 rad bound on the observer's angle error at 40 samples per electrical
       cycle. The open-loop flux integrator, seeded and fed correctly, measures **0.121 rad at 200 rad/s and 0.080 at 600**
       -- a bias it carries whatever the sampling ratio, and the bound was invented rather than measured.
     - The measurement itself, once the flux was seeded: the error did not fall with the bound relaxed, it stayed where the
       bias lives.
     - The criterion is now the *ratio* effect that is actually there: speed-independent at a fixed 40 samples per cycle,
       and 4.7x worse at 8 (0.380 rad against 0.080). The bias is published as a boundary instead of being asserted away.
     - ``pending``
   * - A-23
     - A
     - I reported "12 gates green" as if the online CI had been exercised, then pushed 25 commits and read the run as
       "everything else passed". It had not run anything: build, tests, docs, security and cross-compile are each started
       by an emoji in the commit messages (`:building_construction:`, `:beer:`, `:book:`, `:shield:`,
       `:hammer_and_wrench:`), and `metadata.json` only grants *permission*. Every commit in the push carried `:sparkles:`,
       `:bug:`, `:memo:` or `:wrench:`, so the controller logged five `false` and the run went green in 20 seconds with
       every substantive job skipped -- a green that means "nothing ran", which is the same shape as A-13 (a criterion
       that bypassed its own subject).
     - Reading the Log Decisions job rather than the run's conclusion. The conclusion was `success`; the job said
       `Build: false, Tests: false, Docs: false, Security: false, Cross compile: false`.
     - The local gates are not a substitute for the pipelines, and the trigger set is the five emojis above. A commit that
       carries them is the only way to exercise the sweep, so that is what this row's own commit does.
     - ``pending``
   * - F-14
     - F
     - The FMU gate searched for the FMI headers in exactly one place, `workspace/materials/FMPy/src/fmpy/c-code`, a
       gitignored local checkout. CI installs FMPy with `pip`, and the wheel ships the same unmodified
       `fmi2Functions.h` inside `fmpy/c-code`; the gate meanwhile had already proved an importable FMPy existed, because
       `_fmpy_python()` imports it. So it failed with "fmi2Functions.h not found; pass --fmpy" on the one machine where
       the header was one `import` away, and passed everywhere the checkout happened to be.
     - Run 40814535913, job `Auto Testing (ubuntu-latest)`: `fmi2Functions.h not found` is that job's **only** error, while
       `gates.py fmu` is green locally where the checkout exists. Pointing `ROOT` at a path that does not exist and calling
       `find_fmpy_headers(None)` resolved to `<site-packages>/fmpy/c-code`, which is what proved the header was reachable
       the whole time.
     - The installed package's `c-code` directory joined the candidate list, the checkout still searched first, and the
       error now names `pip install fmpy` as the alternative. A gate's search path is part of its precondition: if it
       accepts a machine it must be able to run there.
     - ``pending``
   * - F-15
     - F
     - The optional python consumer imported `numpy` at module scope, and nothing declared it. The test job installed
       conan, then built and ran the suite; numpy arrived later in the same job, as a transitive dependency of
       `fmpy==0.3.32` installed for the FMU gate. On Ubuntu the `ModuleNotFoundError` cost the case 0.04 s and printed
       nothing, so it read like a broken binding rather than a missing wheel.
     - Two line numbers in one log: the failing case at 2415, the `pip install fmpy` that would have supplied numpy at
       2585. Locally the same case passes in 0.80 s, because this box has numpy. The import is not only the test's own:
       it also arrives through the project's reference analyser (`benchmark/script/analyze.py`, `montecarlo.py`), which
       imports numpy at module scope -- so the wheel is needed by the comparison against a second implementation, which
       is a choice this project made.
     - Declared by the project, installed by a file: `test_package/test/python/requirements.txt` carries `numpy>=2.0`,
       and the shared test job runs `pip install -r` on that path **if it exists**. The workflow names a file, never a
       package, so a derived project with no such file pays nothing and one with different needs writes its own list.
       The case itself now fails with a sentence that names the missing dependency, instead of a bare
       `ModuleNotFoundError` in 0.04 s. Two earlier fixes were withdrawn and are worth keeping: the first installed
       numpy from a step gated on a boolean that never fired (F-16), and both wrote a project's choice of reference
       implementation into a file every derived project shares. The dependency was never the defect; the *undeclared*
       dependency was.
     - ``pending``
   * - A-24
     - A
     - Asked whether deleting the FMU step would turn CI green, I answered "no, the tests run first and they fail". Both
       halves were wrong. The ctest failure never reached the job at all (A-25), so it could not have been the cause; and
       the FMU step was the job's **only** red. I had argued from the step order in the workflow file, which proves
       sequence and says nothing about causation.
     - The log has exactly one `##[error]`, at line 2659, inside the FMU step; the `python_bindings` failure at line 2415
       carries no error marker and the step that ran it exited 0. One grep over the whole job, instead of two reads of the
       YAML, would have settled it.
     - The question "which step turned this job red" is answered by the log's error markers, not by the file's step order.
       Same shape as A-23: a plausible story about the pipeline, believed without asking the pipeline.
     - ``pending``
   * - A-25
     - A
     - A failing case could not fail the pipeline. `_run_ctest_suite` caught every exception, printed `CTest Crashed:` and
       returned, so a suite of 526 cases with one red reported success to conan and the job inherited it. The gate could
       not go red, which means the green it produced was never evidence -- the same defect as A-13, one level up: a
       criterion that bypasses its own subject.
     - An injected `EXPECT_EQ(1, 2)` case: before the fix the step exited 0 with the failure recorded only in a log nobody
       reads; after it, exit 1, the case's own message in the output, and coverage still written (87.4 % of lines) before
       the exception left `test()`. Online, the same shape: 526 cases, one red, job green.
     - Archive `LastTest.log` in `finally`, collect coverage, then re-raise: evidence first, failure never swallowed.
       ctest prints a failed case only on request, and conan's `CMake.test` (2.31.2) forwards build-tool arguments but not
       ctest ones -- its docstring points at the `CTEST_` variables, and `Popen` inherits `os.environ`, so
       `CTEST_OUTPUT_ON_FAILURE=1` is set in the recipe.
     - ``pending``
   * - F-16
     - F
     - A step I added to the shared test workflow to install numpy decided whether to run with
       `enabled=$(python3 -c "... is True")` and then compared `"$enabled"` against the string `true`. Python prints
       `True`, so the comparison never matched and the step took the else branch on every machine, printing
       "enable_python_bindings is off" about a flag that was on. It was inert, and it looked healthy in the diff.
     - Run 40817097696, both test legs, line 226: `enable_python_bindings is off: the python consumer is not built,
       nothing to satisfy`, while `metadata.json` says `"enable_python_bindings": true`. The case failed 40 lines later
       with "numpy is missing from this interpreter", which is the only reason the gate was caught.
     - The numpy-named step is gone; what stands in its place is F-15's file-driven hook, where the package name does
       not appear in the workflow at all. The rule that outlives both: a boolean crossing from Python to the shell
       crosses as an **exit status**, not as a printed word -- `sys.exit(0 if ... else 1)` has no case to get wrong and
       no string to compare.
     - ``pending``
   * - F-17
     - F
     - `SilDeadtime.TheRampFadesTheCorrectionWhereTheSignIsNotYetInformation` asserted that the recorded `sign()` column
       is reproduced "bit-for-bit" within `2.0e-4`, and it cannot be: apple-clang reads 0.14336 at width 0.10 against the
       gcc-13 recording of 0.14364, a drift of 2.8e-4, while the other four widths agree to better than 1e-5. The
       criterion was a claim about compilers disguised as a claim about code, and the swallow in A-25 hid the counter-
       example for a full version.
     - Two runs of the same filter, same case, different toolchains: gcc gives `0.05791 / 0.13498 / 0.14364 / 0.16482 /
       0.17555`, apple-clang gives the same five numbers except the third, `0.14336`. The macOS leg reported it in run
       40817097696, the first run in which a failing case could reach a job.
     - The bound is now `1.0e-3`, and both sides of it are measured: 3.6x the observed cross-compiler drift, and 1/77 of
       the smallest change the assertion must still catch (one sweep row apart, 0.0579 -> 0.1350). The message no longer
       says "bit-for-bit"; it says what is true, which is that the step path reproduces the recorded column.
     - ``pending``
   * - A-26
     - A
     - I read the local `526/526` as the whole answer twice: first as evidence that the suite was green, then as
       evidence that "only `python_bindings` was red" online. The macOS leg had a second failing case the entire time --
       a real cross-compiler defect (F-17) -- and it was invisible because nothing that failed could fail a job. "All
       green locally" was true and said almost nothing about the two legs that matter.
     - The macOS log at line 1742 of run 40817097696, which is the first run where a red case produced an error marker at
       all. The next line is the assertion output, printed by `CTEST_OUTPUT_ON_FAILURE`.
     - A local suite says what the local compiler does. It cannot stand in for a second toolchain, and a green test leg
       only means something once a red case has been watched failing a job -- which is exactly what A-25 was about.
     - ``pending``
   * - F-18
     - F
     - `ci-orchestrator.yml` advertises a `workflow_dispatch` input named `commit_messages` ("Emoji trigger
       simulation"), and `metadata-controller.yml` requires `EVENT_NAME == "push"` in every one of its six trigger
       conditions. A dispatched sweep therefore cannot start a pipeline: it completes green in half a minute having run
       only the metadata, lint and skills jobs. The documented way to exercise the sweep without touching history is
       inert, which is A-23's shape again -- a green that means "nothing ran" -- except that this time the template
       itself offered the shortcut.
     - Run 40822813297, dispatched with all five emojis in `commit_messages`: `Log Decisions` prints `Build: false,
       Tests: false, Docs: false, Security: false, Cross compile: false`, every pipeline job is `skipped`, and the run's
       conclusion is `success`. The emoji string did reach the workflow; the `EVENT_NAME == "push"` guards stop it.
     - Recorded, not fixed: letting a dispatch trigger pipelines is a template design decision, because a dispatch has
       no commit to audit and that absence is the reason the push-emoji rule exists at all. The sweep is requested the
       supported way instead -- a push whose message carries the set.
     - ``pending``
   * - F-19
     - F
     - The Documentation job reports `success` while Sphinx prints `ERROR: Error parsing content block for the
       "list-table" directive: row 53 does not contain the same number of items as row 1 (7 vs 6)`. The step does not
       treat a Sphinx ERROR as a failure, so a page that cannot be parsed ships green; and the local `gates.py docs`
       gate checks Doxygen block shape and the claim numbers but never parses the RST itself, so there was no second
       line of defence either.
     - Run 40822980678, job `Documentation / Build Documentation`: `ERROR: Error parsing` sits three lines above
       `build succeeded, 1 warning.` in the same log, and the job's conclusion is `success`.
     - The row is fixed (F-15 had grown a seventh field, so its commit cell shifted out of the table) and so is the
       gap: the docs workflow now fails when the build log contains an `ERROR: ` marker, because Sphinx exits 0 on a
       parse failure and only prints one. The local stand-in for "the Documentation job said yes" is
       `sphinx-build -b dummy` over the page itself, which parses it instead of trusting a conclusion.
     - ``pending``
   * - A-27
     - A
     - I appended a seven-item row to a six-column list-table, pushed it, and read the green Documentation job as "the
       docs are fine". The table had been six columns for fifty rows; nothing about appending an entry made that fact
       visible, and I never compared the row I wrote against the header.
     - The push's own log: `row 53 ... (7 vs 6)` while the job concluded `success`. Re-rendering the page afterwards
       reproduced nothing, because by then the malformed row was gone.
     - Rows added since are written by copying a neighbouring row's shape and then parsing the page with
       `sphinx-build -b dummy source /tmp/out source/foundation-log.rst`, which is the local stand-in for "the
       Documentation job said yes". Same family as A-26: one instrument's green read as a statement about something
       else.
     - ``pending``
   * - F-20
     - F
     - The gcc coverage collector read only the *package's* build folder for every project that is not
       `is_header`, so any code whose only instantiation point is a test TU was invisible -- and a header-only
       module's only instantiation point is exactly that. Thirteen core files and 496 lines were missing from the
       table, 474 of them covered, so the omission *lowered* the reported rate: `core 85.82 %` against a 90 % floor,
       when the same files measured 93.18 % once they were collected. The clang path never had the blind spot -- it
       merges every profraw and exports from the test binaries -- so the two legs were also measuring different
       file sets.
     - Two artifacts of the same tree: `coverage_gate.py` reporting `FAIL core 85.82 % (1313/1530)` with four
       headers absent, and a hand-merged capture of the consumer side finding those files at 95.5 % (hall), 100 %
       (friction_comp, gear_ratio), 86.7 % (soft_limit).
     - Both halves are collected now, and the capture tolerates two things it must: `source` (the generated
       `ucov_*` copies are deleted before the capture runs) and `mismatch` (gcov's line range for gtest macros).
       Dropping files was the worse tolerance. `core` reads 93.18 % (2132/2288) and `all` 89.18 % (4279/4798).
     - ``pending``
   * - F-21
     - F
     - The tolerance list for the capture was written against this workstation's lcov 2.0-1, where gcov's line
       range disagreeing with a gtest macro is categorised `mismatch`. The runner has lcov 2.4-3, which files the
       same condition under `(inconsistent)` -- so the capture aborted on ubuntu while the identical command exited 0
       locally. A tolerance list is a contract with a **tool version**, and I verified it against the wrong one.
     - Run 40826192872, job `Auto Testing (ubuntu-latest)`: `526/526` cases passed, then `lcov: ERROR:
       (inconsistent) mismatched end line ... while capturing from .../ucov_test_deadtime.cpp.gcda`, then
       `CalledProcessError: Command '[... --ignore-errors mismatch,source]' returned non-zero exit status 1`.
       Same tree, same command, lcov 2.0-1: exit 0.
     - Both names are listed now (`mismatch,source,inconsistent`); a name lcov does not know is itself an error, so
       the list has to carry both. The comment in the recipe records which version files it under which name.
       Verified again on 2.0-1 after the change: `conan create` exit 0, `core 93.18 %`, `all 89.18 %`.
     - ``pending``
   * - A-28
     - A
     - I told you the python bindings were verified locally only, and that moving them into CI would need a
       scaffold change. They had been built, registered and executed in the online tests job all along, on both
       legs -- `-- Found Python: 3.12.15` -> `foc_python.cpython-312-*.so` -> `Test #526: python_bindings`. What was
       missing was not the coverage but the sight of it: the case failed on every push and the failure was swallowed
       (A-25). The 3.10-versus-3.12 worry I repeated as an obstacle was never one, because the extension is compiled
       by the same interpreter that imports it.
     - The logs of run 40822980678, both legs, plus `test_package/CMakeLists.txt`'s own comment ("reuse the
       interpreter pybind11 picked: a module built for 3.12 cannot be imported by 3.13").
     - A capability that is already covered should never be argued as a cost: before proposing work, check what the
       pipeline actually does. Same shape as A-23 -- a story about CI believed without reading CI.
     - ``pending``
   * - A-29
     - A
     - I reported `core 85.82 % vs a 90 % floor` as a coverage *shortfall*, and a plan to raise it. The number was
       measured over an incomplete file set (F-20): the four newest algorithm headers, and nine more files, were not
       counted at all. The library was above its floor the whole time.
     - The same tree measured twice: `85.82 % (1313/1530)` from the recipe's summary, `93.18 % (2132/2288)` after the
       collector was fixed. The thirteen omitted files had 474 of 496 lines covered, so their absence could only
       push the rate down.
     - Before treating a measurement as bad news, check that its scope contains the thing being judged. A gate that
       silently drops files is indistinguishable from a gate that passes them, and the number it prints is not the
       number it claims.
     - ``pending``
   * - C-12
     - C
     - A new case asserted that after `reset()` the unwrapped mechanical angle equals the mechanical angle it handed
       in -- 3.0 rad. It cannot: the estimator is handed an **electrical** angle, which pins the mechanical one only
       modulo 2*pi/poles, so 3.0 reads 0.3072. The case went red on its first run and the implementation was right;
       the criterion was written from the caller's mental model instead of from the quantity the API receives.
     - `The difference between reference.unwrapped_mech_rad() and kFarMechRad is 2.6928 ... where
       reference.unwrapped_mech_rad() evaluates to 0.30720621` -- and 0.3072 is exactly `3.0 mod (2*pi/7)`.
     - The criterion is the principal value now, and the case keeps the point it always had: after a reset there is
       no stale previous angle, and a 3 rad jump is not reported as a lost count because there is nothing to have
       lost. Recorded because a wrong expectation fails in the shape of an implementation bug, which is the most
       expensive shape to be wrong in.
     - ``pending``
   * - F-22
     - F
     - `security-linters.yml`, an upstream file, carried six project-private checks as bare steps -- `gates.py`
       strict, claims, docs, boundaries, capabilities and the negative-compile harness. Upstream's `gates.py` has
       four subcommands, so a project inheriting this workflow fails on its first push with `invalid choice:
       'strict'`. Same shape as the FMU step we put in the test workflow, except six at once, and invisible to us
       because our `gates.py` always recognises them.
     - `python3 <upstream>/.github/misc/gates.py --help` lists `format tidy guard all`; ours lists twelve. The six
       steps were written as `run: python3 .github/misc/gates.py <gate>` with nothing between them and the runner.
     - One guarded step now asks each gate for by name and skips what is absent: the checks still run everywhere
       they exist, and a derived project pays nothing. Verified both branches -- our help text matches all five
       names, upstream's matches none. **Closed 2026-10-30**: the two private ids were dropped and the file is now
       upstream's bytes verbatim. The risk had been written down but never priced -- `pre-commit` is not on PATH on
       this machine and `.git/hooks/pre-commit` does not exist, so the two hooks had never run even once. Both checks
       they wrapped (`strict`, `negative_compile`) already run in `project-gates.yml`, so the only thing the fork bought
       was a commit-time copy of a CI-time result nobody had installed.
     - ``pending``
   * - F-23
     - F
     - Collecting both halves (F-20) put every header into the tracefile **twice**: the library's units are compiled
       against the build-tree copy and the test units against the installed one, so `b/include/x.hpp` and
       `p/include/x.hpp` became two records for one file -- and not with the same instrumented line set. `_coverage_totals`
       summed every record into `lines` but keyed `per_file` by the normalised path, so the totals and the breakdown were
       computed over different bases: 4327 of 4798 lines against a per-file sum of 3577 of 3821, i.e. 750 lines counted
       twice. The scope floors read `per_file`, the published figure read `lines`.
     - The same tracefile, two aggregations: summing records gives `89.53 % (2617/2923)` for the core scope, keeping one
       record per file gives `93.18 % (2132/2288)` -- a 3.6 point spread decided by an implementation detail.
     - The tracefile is now merged per source file (line union by `max(count)`, larger record for functions and branches)
       and every total is the sum of its own breakdown. Re-measured on the same commit: `lines` and the per-file sum agree
       exactly (3630/3857), core 95.70 %, all 94.11 %.
     - ``pending``
   * - A-30
     - A
     - I published `core 95.24 % / all 90.16 %`, pinned the per-leg floors to those numbers, and put them in the audit
       report as evidence for DoD G7 -- on top of a denominator that counted 750 lines twice (F-23). The direction of the
       earlier finding was right (files were missing, F-20); the precision was invented.
     - The reviewer's counter-example, reproduced on our own tree: `lines 4327/4798` against `per_file` sum `3577/3821`, and
       two aggregations of one tracefile 3.6 points apart.
     - A rate is only as good as its denominator's self-consistency: the check `totals == sum(breakdown)` now runs on every
       summary, and any future aggregation change has to keep it true.
     - ``pending``
   * - F-24
     - F
     - The baremetal whitelist was compared case-sensitively in both places that read it: `requirements()` tested the
       conandata package name (`etl`) against the metadata list, and `_preparing_deps_links()` tested the metadata key
       (`Eigen3`) against that same list. `metadata-contract.md` states the rule -- compare lowercase -- and its own default
       value is `["etl","ArduinoJson"]`, an entry whose case can never match a conandata name. Both lines are upstream's.
     - `conan graph info . -pr:h <mcu profile>` with the whitelist set to `["ETL"]` resolved a graph of `cmake` only: etl was
       dropped. With `["etl"]` the graph was `cmake, etl`. Nothing failed -- a missing dependency in a baremetal build
       surfaces later, as a compile error in someone else's file.
     - Both call sites go through `_baremetal_whitelist()`, which lowercases once. Re-probed: `ETL`, `Etl` and `etl` all keep
       etl, and `ArduinoJson` still drops it, so the filter still filters.
     - ``pending``
   * - F-25
     - A
     - Run 40840909635 spent 489 s in the cross-compile legs' `apt-get install gcc-arm-none-eabi`, and the natural reading
       -- "the Conan cache is broken, so every leg reinstalls" -- was wrong twice over: all legs printed `Cache restored
       from key` and the Build leg's `conan create` took 1 s. The apt step's own log says `Fetched 607 MB in 7min 51s (1290
       kB/s)`: 377 s of the 489 s was one slow mirror, and unpacking 3.2 GB is ~10 s. Three later runs of the same 607 MB
       took 39 s, 10 s and 48 s, so the cost is mirror variance rather than a constant -- which bounds what caching can
       win: it removes the eight-minute tail, it does not make a fast mirror faster. The baseline had already dealt with
       this: `cross-compile.yml` in v0.3.1 caches `/var/cache/apt/archives`, and our copy was 62 lines behind it.
     - On the adopted file a re-run of the same job prints `Need to get 0 B/607 MB of archives` and finishes the step in
       19.6 s against 77.7 s cold (607 MB at 15.4 MB/s); the job drops from 88.4 s to 26.0 s. The hand-rolled version this
       started as measured 35.8 s on a hit, because it copied 573 MB in and out of the archive directory instead of
       restoring it in place.
     - The first attempt was such a hand-rolled download cache (`~/.cache/apt-archives` copied into and back out of
       `/var/cache/apt/archives`) and it was thrown away for the baseline file. What a rewrite loses is what someone else's
       incident paid for: `DEBIAN_FRONTEND=noninteractive` with `Dpkg::Options` (a prompt on a hosted runner is a hang,
       bounded only by the 30-minute step timeout), `restore-keys` so a workflow edit still gets a partial hit, and an
       `if: always()` chown standing between a root-owned `partial/` and a silently failed save. `git diff` against v0.3.1
       is now empty for that file.
     - ``pending``
   * - F-26
     - A
     - Two coverage artefacts reported two different denominators for one build: `coverage_summary.json` said
       3630/3857 (94.11%) while `coverage_report/index.html` said 4327/4799 (90.2%). The per-file merge (F-23) happened
       only in memory inside `_coverage_totals`; `genhtml` still read the unmerged `filtered.info`, where one header is
       recorded once per directory that compiled it.
     - A human reads the HTML and the gate reads the JSON, so the disagreement is not cosmetic: it decides which number a
       review argues about. Reproduced on the v0.3.1 baseline too -- the same `cpptest.hpp` twice (`md5` equal, 8 and 13
       lines, one under `b/`, one under `p/`) -- which is why an upstream reviewer raised it as the patch's main risk.
     - The merge moves into the tracefile: `SF:` is rewritten to one existing path per normalised key, then `lcov -a`
       adds the records, and `genhtml` and the totals both read `coverage_test.merged.info`. Verified: HTML `3630/3857`
       equals the JSON, 67 `*.gcov.html` pages for 67 files, and a header-only package is unchanged (`59/49`, both
       artefacts, before and after).
     - ``pending``
   * - F-27
     - A
     - The coverage step pinned its `--ignore-errors` list to lcov 2.x (`mismatch,source,inconsistent`). lcov's own
       `--summary` tolerates an unknown name -- 1.14, 1.15 and 1.16 all exit 0 -- so the breakage hides until geninfo
       runs, and then it is a different tool saying `unknown argument for --ignore-errors: mismatch` and exiting 2.
     - Reproduced by running lcov 1.16's perl scripts from PATH: `conan create` exited 1 and wrote no
       `coverage_summary.json`, on a machine where the same tree passes under 2.0-1. The failing name is `mismatch`,
       not the `inconsistent` that the first reading blamed.
     - `_lcov_ignore_errors()` picks the list by major version (`mismatch,source,inconsistent` for 2.x, `gcov,source`
       for 1.x) and both call sites -- `--capture` and the `lcov -a` merge -- use it. Under a 1.16 shim the build now
       exits 0 with `59/69`, identical to 2.0-1, which is unchanged.
     - ``pending``
   * - F-28
     - F
     - `anchor_check.py`'s leverage rule compared a dimensionless quantity with a current: `psi/(Ld*i_max)` is a pure
       ratio, and it was tested against `LEVERAGE_LIMIT * i_max`, which is in amps. The comparison therefore had no
       meaning, and on the repository's own machine it fired -- `1.32x i_max` -- against `i_max = 3 A`, rejecting a
       table for exceeding a bound its own units could never reach.
     - Surfaced by running the gate once, on the machine it had just been written about. A gate that is only ever
       observed passing would not have shown this: the defect is invisible in the code and obvious in the first
       number it prints.
     - The rule is now `k = psi_f/((Lq - Ld)*i_max)` against a dimensionless `LEVERAGE_LIMIT`, and the same probe
       gives `k = n/a, 1.00, 6.77, n/a, 1.77` for the five registered machines with exactly one reject, the table
       whose `Lq < Ld`. This also corrected a vocabulary point the repository had not noticed: `i_char = psi_f/Ld`
       equals the classic characteristic current only at `Lq/Ld = 2`, the saliency `test_mtpa.cpp` happens to
       measure at, so `Ld` alone cannot decide the trade for other machines.
     - ``pending``
   * - F-29
     - A
     - `anchors.json` registered this repository's nominal SIL machine as salient -- `Ld = 4e-4`, `Lq = 8e-4` -- and
       listed `mtpa` among the claims it witnesses. `sil_bench.hpp`'s `nominal_spec()` has `ld == lq == 4e-4`: it is
       a surface machine, and MTPA on it is worth exactly zero. The +29.2 % belongs to `test_mtpa.cpp`'s
       `salient_spec()`, measured with `i_max` set equal to `i_char` rather than at rated current.
     - The gate printed `mtpa = 0.0%` for a machine registered as an MTPA witness. Nothing was broken in the library:
       the parameter set was transcribed from a wrong recollection of it, which is why the row is an A and not an F.
     - The claim moves to a machine that can carry it: `mtpa-test` (`Ld 2e-4`, `Lq 4e-4`, `i_max = i_char =
       23.81 A`), where the gate's own torque scan gives 29.9 % against the 29.2 % the test reports. `SIL` keeps
       `observer`, `startup` and `position`, which it can support.
     - ``pending``
   * - F-30
     - F
     - The scaffold's `.github/misc/gates.py` had accumulated 321 private lines -- eleven project gates, a
       generalised rule loader, the local-only CI policy -- inside a file the project promised to keep
       byte-identical to upstream. Eight more project scripts (claim, capability, coverage, docfmt, number-sheet,
       offline-arbiter, assume, anchor) had collected in the same directory beside it. Every one of those files was
       individually justified; together they made the next scaffold release a manual merge of code upstream has
       never seen, which is the cost the promise existed to avoid.
     - A byte comparison against v0.3.3 measured 44 differing files, 12 of them in the build-and-CI layer. Nothing
       failed, because a forked gate still runs: the damage is entirely in the *future* release, and a green build is
       the last thing that would show it.
     - The private code moved to `.github/project/` (one entry point, helpers imported from the scaffold's module
       rather than copied), the scaffold's `gates.py` and `security-linters.yml` were restored to v0.3.3 bytes, and
       the project gates run from a new `project-gates.yml` that needs nothing from `ci-orchestrator.yml`. The
       contract is now checked: `project/gates.py scaffold` pins 34 files by hash, compares the `.github/misc` file
       set as well as its contents, and refuses a new fork, a missing file, or a tolerance that has become stale.
       Both failure modes were demonstrated before the gate was trusted. Twelve forks remain, each with a written
       exit plan and a ratchet that only goes down.
     - ``pending``
   * - A-31
     - A
     - The open-loop start's "steady slip" was treated as a steady state and compared against a closed form. It is
       not one: the rotor never tracks the ramp -- measured 49.6 rad/s mechanical against the 57.1 the ramp demands at
       400 rad/s^2 -- so the slip grows for the whole pull and the "last quarter" average is a property of the window,
       not of an operating point. That is why the first law missed by 1.7x to 2x and why the second one, with the
       unit mapping corrected and the viscous term added, still missed by 41 %.
     - Two attempts from different directions. The first used the electrical acceleration with the rotor's inertia and
       dropped damping; the second fixed both and still disagreed. What exposed the real cause was measuring the
       quantities the law assumes instead of the law itself: the forced-frame current is entirely on the d axis
       (id = 1.20 A, iq = 0), and the rotor's own speed in the window is 13 % below the ramp's.
     - No code changed; what changed is the claim. `startup.hpp`'s README row now states that a closed-form ramp
       limit is not claimed *and why*, with the two failed attempts and the measured reason. Three test cases that
       asserted the law were written, run red, and deleted rather than loosened -- a band widened until it passes
       teaches nothing. The measurable statements (worst slip, hand-over survival) stay asserted.
     - ``pending``
   * - A-32
     - A
     - The second attempt at the pull's load angle used a *second model* rather than a formula: the rotor equation
       `delta'' = alpha_m - (Kt i / J) sin(delta) + (b/J)(alpha_m t - delta')` integrated by hand. It did not
       reproduce the bench either, and this time the disagreement is a clean factor rather than a mystery --
       integrated last-quarter mean 0.178 rad against a measured 0.293 at 400 rad/s^2, and 0.134 against 0.197 at
       200, i.e. the bench sits 1.5x to 1.6x above the second model at both accelerations.
     - The first comparison was made against the *worst* slip and produced a ratio of 0.35, which looked like a
       broken model; it was a broken *comparison*. The worst slip belongs to the initial transient (the rotor is
       parked at an unknown angle, so the first sample is already 0.27 rad off), while the equation describes the
       late part of the pull. Pairing the integration's own last-quarter mean with the bench's last-quarter mean
       turned 0.35 into 0.61 -- still wrong, but wrong in a way that can be reasoned about.
     - No code changed. The integration was removed after it failed rather than tuned to pass, and the residual is
       recorded because a *systematic* factor of 1.5 across two accelerations is a property of the plant/loop
       coupling, not of one number: the next attempt has to read `sil_pmsm_plant.hpp`'s torque path (rotor-frame
       current, `effective_ld/lq`) and `PlantHal`'s electrical-to-mechanical mapping instead of assuming them.
     - ``pending``
   * - A-33
     - A
     - "Steady slip" is not a steady angle at all: it is a **lightly damped oscillation**. Measured slip over one pull
       changes sign repeatedly (+0.66 ... -0.31 ... +0.55 ... -0.08 rad) and the electromagnetic torque follows it in
       sign. The viscous damping in the bench machine gives a damping ratio of 2 % (`b/(2 J omega_n) = 0.02`), so the
       decay constant is `2J/b = 2.0 s` -- longer than the whole 1 s pull. The number the test averaged was therefore an
       oscillation amplitude set by the *initial* mismatch, not a load angle set by the acceleration.
     - Counting zero crossings instead of trusting the average. Over one pull at 400 rad/s^2: 20 crossings in 1.000 s,
       i.e. 62.8 rad/s, against `sqrt(Kt i/J cos delta*) = 24.4 rad/s` -- **2.6x apart**, so the linearised pendulum is
       still not the right model. The envelope does behave like the damped one: 0.8203 rad at the second crossing falling
       to 0.6194 at the last, a time constant of about 3.6 s against the predicted 2.0 s -- right order, wrong number.
     - No code changed and no claim moved: `startup.hpp` still does not claim a closed-form ramp limit. What this adds is
       a much narrower target -- the remaining error is in the *frequency* of the slip oscillation, not in its size, and
       the next step is a fine-grained trace of one period rather than another derivation. Three attempts (steady law,
       second model, linearised pendulum) have each been refuted by their own measurement.
     - ``pending``
   * - A-34
     - A
     - The linearised pendulum of A-33 was right in shape and wrong by `sqrt(p)`: the slip is an **electrical** angle
       while the inertia is mechanical, so the stiffness of the equation for the slip carries the pole pairs
       (`delta'' = alpha_electrical - (p Kt i / J) sin delta + (b/J)(omega_e - delta')`). Every attempt in A-31 to A-33
       wrote the mechanical equation and then compared it against an electrical measurement, which is exactly the
       `sqrt(7) = 2.65` gap that kept appearing.
     - The measured zero-crossing rate was 62.8 rad/s against the mechanical prediction 24.4 (ratio 2.57); `sqrt(p)`
       is 2.646. With the factor restored the same run gives 64.66 predicted against 62.82 measured, and the sweep at
       400 / 200 / 1200 rad/s^2 gives ratios 0.972 / 0.994 / 1.040.
     - The claim moved instead of the code: `startup.hpp`'s README row no longer refuses a closed form. It now states
       the oscillation frequency `omega_n = sqrt(p Kt i cos(delta*) / J)` about
       `sin(delta*) = (J alpha_electrical / p) / (Kt i)`, with the three measured points, and keeps the *amplitude* out
       of the claim because it is set by the initial mismatch. The test that asserts it also asserts that the
       no-pole-pairs variant misses, so the criterion is proved able to reject the mistake that was actually made.
     - ``pending``
   * - F-35
     - F
     - `FaultCode::LostSync` had a definition, a ranking entry and a name in the C API, and **no producer anywhere**: the
       drive could lose the rotor after a failed open-loop start and report nothing but a slowly falling speed. The
       start cannot see the rotor (it forces the angle and does not step the observer), so the evidence only exists one
       window after the hand-over -- and nothing was looking for it.
     - Writing the check put it in the observer branch first, where it never ran: the SIL rig is a *sensored* setup
       (`sensor_present_{true}`), so `speed_from_observer` is false and that branch is skipped entirely. The new test
       failed for that reason and not because the threshold was wrong -- a check placed where the code does not pass is
       the same defect as no check at all.
     - `Motor` now carries a cause (`fault(FaultCode)`, `fault_code()`), and 20 ms after a hand-over the **published**
       speed -- sensor or observer, whichever owns the angle -- is compared with half of the speed the sequence says it
       handed over at. `AlignOutput` gained that field, and the second attempt at the check is why: deriving it from the
       forced vector's own angle rate faults a *plain alignment*, which turns the vector to its target and then stops,
       so its rate says nothing about the rotor. Two SIL alignment cases failed and named it. The aligner is the only
       party that knows whether it dragged or merely held, so it states it -- 0 means there is nothing to lose sync
       with. Measured: 400 rad/s^2 hands over clean with code `None`; 3000 rad/s^2 -- which still reaches its hand-over
       speed because the routine only knows the angle it commands -- faults with `lost sync`. `clear_fault()` resets the
       cause with the controllers so a stale one cannot outlive its state.
     - ``pending``
   * - F-36
     - F
     - `FaultCode::Stall` was in the same state `LostSync` had been in (F-35): defined, ranked, named in the C API, and
       never raised. The load-step sweep proved what that costs -- at four times the rated torque the machine was pushed
       **backwards** to -895 rad/s while the loop held a +400 rad/s command and reported nothing at all.
     - The load-step test was written for gap 3 of the C2 plan and its own falsifier produced this: a band that is only
       ever observed passing cannot show that the over-range case is unhandled. 20 of 20 within +-50 % were held, so
       nothing in the sweep pointed at the hole; one run beyond it did.
     - The detector is narrow on purpose: full current (90 % of the ceiling), **and** the machine not moving the way it
       was told (reversed, or below a quarter of the command), **and** sustained for 100 ms. Any one of the three alone
       fires on a large speed step, on a light load, or on the step's own transient -- and a protection that trips
       wrongly gets disabled. Measured after: the 4x case faults with `stall` in 100 ms; the 20 in-range cases still
       pass untouched.
     - ``pending``
   * - F-37
     - F
     - The cross-compile leg had never actually been built. Our copy of `cross_compile_check.py` predated upstream's
       `--build=foc*`, so `conan create` was free to reuse a cached package and the archive inspection was a replay of
       whatever the last local build left behind -- a check that could not fail. Taking upstream's script turned it into a
       real build, and the real build failed on `pybind11` for the target: our `conanfile.py` was also older, missing the
       `cross_building(self)` guard on both the `requires()` and the `_preparing_deps_links()` sides.
     - Both were found by doing stage 0 (converging the scaffold forks) rather than by any test: the first run of the
       restored script printed `libfoc_cpp.a: expected Machine AArch64, got` and then Conan failed inside `build()`.
     - `conanfile.py` was taken from v0.3.3 verbatim after checking it contains nothing private -- F-24's
       `_baremetal_whitelist()` is *already* upstream, so the fork had no content of its own and was purely a stale copy.
       The cross leg now builds and reports `Class=ELF64 Machine=AArch64` for both archives.
     - ``pending``
   * - F-38
     - F
     - Two more forks were not project work at all. `test_package/CMakeLists.txt` had **zero functional lines** of
       difference -- 17 inserted and 4 deleted lines, every one of them a comment -- so the tolerance reason recorded for it
       ("project test targets") was simply wrong. `test_package/conanfile.py` had exactly one functional difference,
       `_package_root()`, and it was **ours that was worse**: our docstring explained that asking `conan list` returns
       whichever package was printed first, so a cross build of the same version points the coverage filter at the wrong
       tree -- and then the body went on doing exactly that. The fork had taken upstream's *words* and not upstream's
       *code*.
     - Both were found by counting the non-comment diff lines of each tolerated file instead of trusting the reason
       written next to it.
     - Both files were taken from v0.3.3 verbatim (forks 6 -> 4). The `conan list` scoping bug is now gone by
       construction: `self.dependencies[...]` cannot select the wrong package. It was reachable here too, because the
       cross-compile leg leaves an aarch64 `foc/0.1.0` in the cache next to the x86_64 one.
     - ``pending``
   * - A-35
     - A
     - The pole-pair law of A-34 was measured only against the model the claim rests on: the same d-q plant the pull
       is made of. A sign or factor error in that model moves prediction and measurement together, so the ratio would
       stay at 0.97 while both were wrong. The law now has a witness that shares nothing with it: the phase-variable
       plant integrates the three phase fluxes directly, derives torque from magnetic co-energy, never forms a d-q
       quantity and never uses `Kt`.
     - The recorded duty train of one start (400 rad/s^2) was replayed into `AbcPlant` and the slip crossings counted
       over the identical tick set. Measured 62.82 rad/s -- the same number to two decimals as the d-q run -- against
       64.66 predicted, so the ratio stays 0.972. Rotor travel 29.76 rad.
     - The cross-check is asserted, not just printed, and it carries its own two controls. **Liveness**: with the
       mechanical integration switched off the same loop returns 197.9 rad/s at a saturated 3.14 rad slip, i.e. it
       cannot produce the pendulum frequency without mechanics. **Boundary**: at double inertia the rotor completes
       only 3 crossings (9.42 rad/s against 45.40 predicted), so the law is a *locked-rotor* law and stops applying
       there -- pinned as a number in the test rather than as a sentence in a document.
     - ``pending``
   * - A-36
     - A
     - An L2 row was promoted to L3, which needs three things and only had two. The frequency claim had an analytic
       anchor and (since A-35) an independent witness, but no sensitivity table: the repository's own definition of L3
       is "L2 **and** the sensitivity of each error source **and** the phase-variable cross-check".
     - Eleven model error sources were injected one at a time into the same start, at the magnitudes
       `test_sil_sensitivity.cpp` already uses. The oscillation frequency stayed within 3.3 % of the clean run for
       every one; the worst slip stayed within +18.2 %. One source is material: **differential sensor gain**
       (`u+10 v-10 %`, +18.2 % slip), so it goes on the user-must-measure list. The `sources` order in the test is
       anchored **by name** rather than by index, so inserting a row cannot silently re-point an assertion.
     - The table needed a new knob to be honest -- current-sense gain and offset were private to
       `test_sil_sensitivity.cpp`'s own HAL, so the startup bench could not inject them at all. They are now
       `PlantCurrentSense::set_phase_gain/set_offset_a`, defaulting to identity. Two *analytic* anchors keep the
       table from measuring the harness: a common offset moves the slip by < 0.1 % (it cancels identically in
       Clarke), and a differential gain must move more than a uniform one (phantom negative sequence vs a loop-gain
       scale). Both hold.
     - ``pending``
   * - A-37
     - A
     - The first rule set for that table was mine and it was wrong. It declared "a source that moves the slip by
       2 %-20 %" a red state -- neither negligible nor actionable. It went red, on exactly the two sensor-gain rows
       (2.2 % and 18.2 %), which is the signature of a rule that describes where two thresholds were placed rather
       than the machine.
     - The red came from the first run of the rule, before any threshold was moved to accommodate it.
     - Replaced by two arithmetic anchors (A-36). A second instrument error was found in the same pass: the
       frequency was counted as `pi * crossings / pull_s`, quantised at 3.14 rad/s = 5 % of the value, which made
       every "unchanged" cell a statement about the counting. `slip_frequency_rad_s()` times the crossings instead,
       and the test asserts the timed and the counted value agree within 2 % so the two measures cannot drift apart.
     - ``pending``
   * - F-39
     - F
     - An origin could not exist through the C ABI at all. `PositionReference` is advanced only while the commutation
       angle comes from the *observer* (`speed_from_observer || injecting_now()`), and `CallbackPosition::has_sensor()`
       was hard-coded `true` with `velocity_rad_s()` returning `0.0` (not `NaN`) when the callback was absent. So
       `speed_from_observer` was always false on the host path: the reference never advanced, `homed()` could never
       become true, and "absolute position" was a concept the C surface could neither reach nor report. The gap was
       invisible because the C surface never mentioned it, and the C++ path (which the SIL tests use) has
       `PlantPosition::set_sensor_present(false)`.
     - Asking what a caller could *do* with the refusal I had just added: the answer was "nothing, forever". A query
       whose success branch cannot be reached is a deleted feature with extra steps, so the branch was traced back to
       the flag that keeps it false.
     - Three additions, all append-only and `struct_size`-gated: `foc_motor_setup_t::use_observer_angle`, the optional
       `position_has_index`/`position_index_pulse` HAL callbacks, and `HalCallbacks::valid_without_angle()` so a
       sensor-less host may omit the angle callback (a *second predicate* rather than a flag, because "sensored host
       without an angle" is a configuration mistake while "sensor-less host without one" is the normal case, and one
       predicate would spell them the same). The callbacks struct is an *input*, so its appended fields are gated by
       the caller's `struct_size` -- reading them unguarded would read an older caller's stack.
     - ``pending``
   * - A-38
     - A
     - The first version of the new rig returned the index as a **level** (`index_edges > 0`) instead of an edge. Every
       tick after the crossing then re-established the origin, pinning `referenced_mech_rad` at `index_mech_rad`; the
       measurement was 0.0000 -> 0.0000 rad across 10 ms of rotation at 300 rad/s.
     - The red came from the "it tracks the rotor" assertion, not from a compile error, and the fix was in the
       experiment rather than the library.
     - Made the pulse a one-shot consumed by the next `index_pulse()` call; the same measurement then gave
       0.0000 -> 2.9872 rad against 3.0 expected. Worth recording because `hal/position.hpp` had *already* warned
       about exactly this failure ("a level would count one pass thousands of times") -- a documented hazard is only
       a warning until someone reads it, and the rig is where that has to happen.
     - ``pending``
   * - A-39
     - A
     - The positioning error budget's first metric was wrong, and it printed evidence of that: an encoder of 2 mrad per
       count measured **-0.26 mrad**, i.e. the error source made the result *better*. The cause is that quantisation is
       **signed** -- where the residual lands between two counts decides whether it helps or hurts -- and the metric was
       "final error minus the clean run's final error" at a single target. A budget made of such deltas can be pushed
       negative by one lucky target, which is the same class of mistake as fitting a threshold to a run.
     - The first table's own output: baseline residual 1.252 mrad, "encoder contribution" -0.262 mrad, and a combined
       measurement *below* the baseline it was supposed to be an increase over.
     - The target is now swept across one count (8 points) and the worst case is taken, which turns every row into a
       non-negative worst-case increase and makes the half-count bound checkable. Measured after: baseline 1.261 mrad,
       encoder 0.975 mrad (97.5 % of the 1.0 mrad half-count bound), breakaway 0.199 mrad, dead time 0.014 mrad,
       combined 2.236 against a budget of 2.450 (ratio 1.10). The bracket is now `measured <= budget <= 1.5*measured`
       and dropping the dominant term breaks the lower half, so a missing contributor cannot pass.
     - ``pending``
   * - A-40
     - A
     - C3's third gap asked for "a closed-form bound on trajectory following error", and the answer is that this
       architecture has no trajectory to follow: `MotionProfile` shapes a *velocity* command out of the position error,
       it never generates a position reference. So `e = v/kp`, the textbook first-order following error on a ramp, has
       nothing to describe here.
     - The measurement refused the hypothesis twice over: the cruise gap came out at 0.496 rad against 0.100 predicted,
       and doubling the shaping gain left it at **0.496 as well**. The second number is the one that settles it -- a
       quantity that does not move when its supposed parameter doubles is not that parameter's.
     - The claim was replaced rather than widened: what the structure does bound is where the cruise ends, at the
       deceleration threshold `v^2/(2a)` = 0.200 rad, and the smallest cruise gap measures 0.173 rad against it
       (-13.7 %, asserted, with the gain-independence kept as an assertion so the refuted law cannot quietly return).
       The README row now says so instead of quoting a following-error law.
     - ``pending``
   * - F-40
     - F
     - The position-error budget's individual rows are not converged in the simulation's integration substeps, so they
       are spot values of one setting rather than measurements of the machine. The aggregate hid this: the bracket
       looked healthy in both settings, so nothing pointed at the rows.
     - At ``kSubsteps=4``: baseline 1.261, encoder 0.975, breakaway 0.199, dead time 0.014 mrad, combined 2.236
       against a budget of 2.450 (ratio 1.10). At ``kSubsteps=16``: 1.673, 0.573, 0.435, 0.053 mrad, combined 2.495
       against 2.734 (ratio 1.10 again). The encoder row was the sharpest witness because it had been quoted as
       **97.5 % of the arithmetic half-count bound** -- a number that reads like confirmation -- and refining the
       integrator by 4x moves it to 57 % of the same bound.

       Fixed, and the fix was one line of precision: the mechanical channel is integrated in double (`omega_mech`,
       `theta_mech`, `domega`, `dtheta` and the RK4 combination), while the HAL accessors still narrow to float at the
       boundary -- a sensor has a resolution, the physics does not. The root cause is arithmetic, not control: at
       theta = 1.2 rad the float32 ulp is 1.19e-7, and a settled speed of 0.004 rad/s over a 12.5 us substep is a
       5.0e-8 step, i.e. below half an ulp, so `static_cast<float>(theta + step)` returned `theta` unchanged.

       All three settings, 4 / 16 / 64 substeps: the no-friction settled offset went from 200.4 / 799.9 / 3197.4 urad
       to **0.0 / 0.0 / 0.0**, and the stiction reversal value from +233.4 / -339.7 / -3525.6 urad (sign included) to a
       constant **+31.7 / -31.7**. The budget's four rows are now stable to better than 1 % between 4 and 16 substeps.
     - The consequences were larger than the defect and are recorded rather than absorbed: six cases carried numbers
       that only held on the frozen plant -- two of them claims, not constants (A-42) -- and one had been measuring its
       ceiling against a position the API never receives (F-41). Each was re-derived or rewritten; **none was
       loosened**. That is the measured answer to "how much rests on this": six cases.
     - ``pending``
   * - A-41
     - A
     - C3's missing evidence level was pursued through an authoritative external framework instead of one more internal
       check: ISO 230-2 / VDI/DGQ 3441 define unidirectional accuracy *A*, repeatability *R* and reversal value *B*,
       and the procedure of approaching every target from both directions. The prediction was written before the
       measurement: *B* equals **twice** the one-sided stiction displacement, with "no stiction therefore *B* = 0" as
       the mechanism's own control.
     - The measurement refuted the attribution and then refuted the instrument. With **zero** friction the reversal
       value was still 0.40 mrad at 1.2-1.8 rad targets, so stiction was not the cause; and it was not a settling
       artefact either, because extending the run from 1.2 s to 4.8 s left it unchanged. Sweeping the integrator then
       showed what it actually was: the settled offset went 200 -> 800 -> 3197 urad at 4 / 16 / 64 substeps, and
       adding 0.2 mN*m of stiction flipped its sign (+233 -> -340 urad).
     - Adopting the standard paid off twice. First it proved the bench could not support the claim (F-40); then, once
       the plant's mechanical channel moved to double, the same measurement returned the prediction it had rejected
       first time: with no friction the reversal value is exactly **0.0**, and with 0.2 mN*m of stiction the two
       directions sit at **+31.7 and -31.7 urad, so B = 63.4 = 2x the one-sided displacement**, invariant across
       substeps. **Neither P1 nor P2 was wrong; the instrument was.** The test is still not in the suite: it belongs
       there once C3's own numbers are settled, and re-adding it is now a small job rather than a blocked one.
     - ``pending``
   * - F-41
     - F
     - The soft-limit ceiling check in `test_soft_limit.cpp` compared the shaped command against `kMaxRad - theta`
       computed in double, while the shaper it was checking had received `theta` narrowed to float. Near the wall that
       is catastrophic cancellation: `0.5 - theta` subtracts two nearly equal numbers, so a 6e-8 rounding of `theta` is
       6e-4 *relative* on the 1e-4 rad that is left.
     - Invisible until F-40, because both sides then computed the same float subtraction and were bit-identical. After
       it, one period in 4000 showed the idealised command **1.308e-3** over its own ceiling. The first reading of that
       was "float-boundary artefact" with a 1e-4 allowance -- wrong by a factor of 13, and the giveaway was that a
       boundary artefact of a float subtraction is 1e-7, not 1e-3.
     - The check now narrows the position once, to the value the shaper actually receives, and the count returns to
       zero. Recorded because the shape recurs and is not a plant defect: a checker that computes its reference more
       precisely than the code under test can receive it is measuring a machine that cannot be built.
     - ``pending``
   * - A-42
     - A
     - Fixing F-40 changed two published claims and one headline conclusion, all three of which had passed every gate
       and been argued for in prose. This is what carrying a recorded defect forward actually costs.
     - (1) The speed-ripple case concluded "the ripple is inertia-invariant, so the mechanical story is falsified",
       with a mechanism (gains derived from the inertia) and a number (5.9x q-ripple growth). On the corrected plant
       ten times the inertia buys **9.16x** less ripple and the q-ripple does not move at all (0.0201 A against
       0.0200 A) -- the mechanical chain holds and the earlier falsification described the simulator. (2) The
       published-speed filter case claimed it removes the carrier without moving the axis or costing accuracy; C10a
       still holds at **10.2x** and C10b is still falsified, but the steady bias moves to **1.87x** against a 1.2x
       budget, so the three-way trade does not exist at any swept cutoff. (3) The position budget's headline "the loop,
       not the sensor, is the accuracy limit" is a statement about the measurement window: the loop's share decays to
       **0.000 urad** (time constant about 1 s) while the encoder's half-count floor is permanent at 991 / 995 / 999
       urad at 1.2 / 6 / 24 s.
     - All three now read as what the tests assert, and each carries its old value in a comment. The lesson is in the
       arithmetic: a defect that **freezes** a state produces measurements that are stable, repeatable and wrong in one
       direction, so repetition inside the same suite cannot find it -- only a different instrument can. The relay that
       found it was a Python float64/scipy reference (`project/gates.py plantref`), which is now a gate for exactly
       that reason: it shares no language, no arithmetic and no solver with the code it checks, and a 0.1 % error in
       its own parameters turns it red.
     - ``pending``
   * - A-43
     - A
     - The plant now has an instrument of its own: `project/gates.py plantref` builds a scenario harness that dumps the
       state *and the applied phase voltages* every control period, and a float64/scipy reference replays it. Dumping
       the voltage is what makes the instrument scenario-agnostic -- the closed-loop case (PositionP -> PI speed -> PI
       current -> SVPWM) is audited without one line of controller re-implemented in Python, so a disagreement is
       unambiguously about the plant. Three audits: per-period replay, energy conservation, and kinematics.
     - Two of its design choices were forced by the instrument failing to catch what it was built to catch, and both
       lessons are about measurement rather than about the plant. (1) The per-period replay **cannot see F-40**: the
       self-test froze the angle and every sampled period still passed, because losing half an ulp per step is below any
       single-period tolerance and only shows up when it accumulates. The accumulating audit (`theta(T) - theta(0)`
       against `integral of omega dt`) had to be added *after* the demonstration, not before it. A local-error test and
       a drift test are different instruments. (2) Reported as a ratio of the travel, 200 urad on a one-radian move is
       2e-4 and passes any sane bound, while being twenty times the finest claim resting on that trajectory -- so the
       kinematics bound is **absolute radians derived from the evidence chain** (1e-5 rad, a hundredth of the mrad-level
       position budget), not a ratio of the measurement's own scale.
     - Measured: open loop currents 1.35e-7 of scale and angle step 0.03 park-angle counts; closed loop currents
       3.18e-6, angle step **0.14 counts** (one float32 rounding), energy closes to 0.65 %, kinematics 3.2e-9 rad. The
       self-test perturbs a good trajectory three ways -- a travel 200 urad short, a speed 1 % high, a phase voltage
       2 % high -- and requires each audit to go red; all three are caught, so "this instrument would have found F-40"
       is a demonstrated property rather than a claim. A third lesson came free: the energy audit first reported a 38 %
       imbalance that was **my formula's** -- the dq-side magnetic energy needs the 3/2 of an amplitude-invariant
       transform, `0.75*(Ld id^2 + Lq iq^2)`. Found by re-deriving on a case whose answer was known, not by loosening
       the bound. An audit is an experiment too.
     - ``pending``
   * - A-44
     - A
     - The position row reaches L3, and it reaches it the way the level is supposed to mean: measurement, per-source
       sensitivity, and a third leg that is an **independent model**. The third leg is two externally published
       frameworks driven from the command line harness rather than from the suite. ISO 230-2 measures the reversal value
       the way the standard specifies -- every target approached from both sides -- and the quantisation distribution is
       measured against Bennett's model.
     - ISO 230-2, three targets: the reversal value is **exactly twice** the one-sided stiction displacement (ratio
       1.00 at 0.6 / 1.0 / 1.4 rad), the same measurement at zero breakaway torque collapses to **0.42 urad** (0.11 % of
       B, which is the mechanism's own control), repeated identical moves differ by **exactly zero**, and
       bidirectional accuracy is **0.386 mrad** against the 2.45 mrad budget. Quantisation: a 2 mrad sensor's residual
       over a sweep spanning one count has sigma **558 urad against q/sqrt(12) = 577 urad, 0.966x**, with mean
       -51.8 urad inside the q/4 bound.

       Two of the criteria were **tightened after the first measurement**, and the distinction matters enough to write
       down. P1's band went from +-50 % to +-10 % and the quantisation band to +-25 %: the first because the model
       predicts *exactly* 2 and 1.00 was measured, so a +-50 % band was a criterion nothing within a factor of 1.5 could
       contradict; the second because a 16-point sample of a uniform distribution has a ~18 % relative standard error,
       so tighter would fail on sample luck and looser could not separate uniform from triangular (0.71x). Tightening a
       band when the model fixes the value is the opposite of fitting a threshold to a run; the tell is that the new
       band is derived from the model or the sampling statistics, never from the measurement.
     - The ISO 230-2 case itself was deleted in A-41 when it could only be measured on a frozen bench, and it comes back
       now as a **gate** rather than a test case, because its inputs are a scenario sweep rather than a fixture. That is
       the shape the second half of this entry is about: evidence that lives in a different language from the code it
       checks does not go stale the way a hand-maintained test does.
     - ``pending``
   * - A-45
     - A
     - The first execution of the new observability discipline (section 3.4 of the maturity plan, enforced by
       `project/gates.py observability`) asked the question the instrument was supposed to raise: now that any plant
       scenario can be replayed in float64, which existing claim rows does that re-open? Twelve L2 rows, and **none of
       them can be promoted** -- the leg the instrument supplies is implementation-level.
     - The twelve are registered in `observability.json` with what each still lacks. The number that matters is not the
       twelve, it is the zero: an instrument that replays the same equations in another language proves the code does
       what the code says and says nothing about whether the rule is right. Counting that as L3 would make every port
       an evidence upgrade, which is the cheapest available way to be wrong -- so the gate refuses it outright rather
       than trusting the judgement of whoever runs it next.
     - The audit was not free of information: it ranked the remaining work by cost, and the ranking disagrees with the
       order the plan had. `friction_comp` is the cheapest remaining promotion because the model it needs is already
       published and validated (Dahl / LuGre / Stribeck describe pre-sliding displacement, which is exactly what a
       friction compensator claims to undo), while the plan had `speed_from_angle` first on the grounds that the
       instrument could reach it. Being reachable and being cheap are different things, and only the registry shows both
       in one place.
     - ``pending``
   * - A-46
     - A
     - `friction_comp` reaches L3 through the observability discipline, and it is the discipline's own demonstration:
       O-1 re-opened this row and correctly refused to promote it, because replaying the plant in float64 is an
       implementation-level leg. What promoted it is O-2, a **model-level** upgrade -- the plant gained a Stribeck dip,
       the published mechanism a velocity-sign feedforward structurally cannot reach. Until that dip existed, the plant's
       friction *was* the compensator's assumption, so every check of the compensator was the model checking itself.
     - Three predictions, written first, all hold: cruising at 0.25 / 1 / 2 rad/s with a dip of width 1 rad/s costs an
       extra following error of **1.8789 / 0.7356 / 0.0364 mrad** against `(Fs-Fc)*exp(-(v/vs)^2)/kKp` = **1.8790 /
       0.7358 / 0.0367** (ratios 1.00 / 1.00 / 0.99); the compensation saves `(Fc + b*v)/kKp` measured **2.0500 / 2.2000
       / 2.3998** against **2.0500 / 2.2000 / 2.4000**; and the saving is **identical with and without the dip**, to
       1e-4 mrad. The last one is the claim: the law has no state, so it is blind to the dip, and the row now states its
       domain with a formula instead of a caveat.

       My own P2 was wrong the first time -- written as a flat `Fc/kKp` = 2.000 mrad -- because the plant applies
       `b*omega` as a term separate from the friction law, so cancelling the viscous share is part of the saving. The
       measurement said 2.05 / 2.20 / 2.40 and a re-derivation produced `(Fc + b*v)/kKp`, which reproduces all three to
       four digits. Third time this session that the prediction rather than the code was the thing to fix (A-40, A-42),
       and the reason the prediction is re-derived and not re-fitted is that a fitted one would have hidden the fact
       that the viscous term is part of the mechanism.

       One bug came with the feature and the suite caught it, which is the only reason it is a paragraph rather than a
       silent wrong number: the new field was inserted **between** `t_coulomb_nm` and `omega_eps_rad_s`, and
       `PlantParams` is aggregate-initialised positionally in a dozen places, so every one of them fed `0.05F` to the new
       field and left the at-rest band at 0. The budget test caught it as a breakaway row moving from 0.213 to 0.355
       mrad. Appending the field to the end restored bit-for-bit behaviour. **A field's position is part of its API when
       the struct is initialised positionally**, and nothing in the type system says so.
     - The `budget` gate went red on the new setter with exactly the sentence it exists to produce -- "a new error source
       arrived and nobody decided anything about it" -- and the knob is now covered by the friction table, making 11
       knobs across 3 tables.
     - ``pending``
   * - F-42
     - F
     - `speed_from_angle.hpp` told its readers that a least-squares slope over the same window would be quieter by
       `sqrt(12/N)` -- "a factor of 4.1 at N = 200". The factor is `sqrt(6N/((N+1)(N+2)))`, which is **5.82** at N = 200.
       The stated number was wrong by exactly `sqrt(2)`.
     - Two independent checks, neither of which trusts the other: the exact linear-algebra noise gain (both estimators
       are linear in the noise, so the norm of the weight row *is* the answer) gives 0.560112 / 0.171916 / 0.107937 at
       N = 16 / 200 / 512, and the closed form gives the same to six digits; the C++ case, run on a synthetic stream,
       measures 0.3249 / 0.1738 against 0.3363 / 0.1719. The `sqrt(12/N)` form predicts 0.4899 / 0.2449, which the
       measurement excludes by 46 % and 30 % -- so the case can tell the two formulas apart, which is the only reason
       its agreement means anything.
     - The origin is ordinary and worth naming: the difference carries a `sqrt(2)` of its own, and the ratio was formed
       as if it did not. The note's *first* formula has the `sqrt(2)` in it; the ratio two sentences later dropped it.
       Found by deriving both sides rather than by re-reading the prose, which is the same lesson as A-44's: the tool
       that finds a wrong number is a derivation, not a careful reading.
     - ``pending``
   * - A-47
     - A
     - `speed_from_angle` reaches L3 by the third execution of the observability discipline (O-3), and it is the first
       one whose leg is **analytic** rather than a second simulator or a new plant mechanism. The claim the row carries
       is now the closed form checked against the shipped estimator, plus the diagnosis that closes the loop: the row
       already recorded the rig measuring 6.5x worse than the model, and since the model is verified to 1-2 % when the
       angle error is independent, that 6.5x is evidence of a *correlated* angle error rather than of a wrong formula.
       The gap became a measurement instead of a caveat.
     - One prediction did not survive and is recorded rather than deleted: a quantised angle was asserted to give
       `q/sqrt(12)` at the output and gave **0.616x** of it. The independence assumption behind `q/sqrt(12)` does not
       hold on a deterministic staircase -- the phase at sample `k` fixes the error at sample `k`, so the two ends of a
       window are correlated and `Var(diff) = 2*sigma^2*(1-rho)` is strictly below the independent-sample value. What is
       asserted is that inequality, which is derived; the factor would need phase dithering to measure and is left
       unmeasured rather than fitted. Same lesson as A-39, one level up: quantisation statistics are a statement about
       the *phase relationship*, not only about the count.
     - The third thing this pass taught is about self-tests. `observability_gate.py --selftest` had one case that
       removed a named L2 row from the registry, and promoting that row made the case impossible to trigger -- which the
       gate reported as MISSED. A self-test pinned to a moving target stops testing anything the day the target moves,
       and it only announced its own uselessness because it was written to fail rather than to pass when the premise
       disappeared. The target is now selected from the data.
     - ``pending``
   * - F-43
     - F
     - `ElectronicGear` declared `kMaxStepRad = pi` with a comment saying that the shortest-arc decode assumes the master
       moves less than half a turn per sample, "written down because the gear has no other way to tell a fast reversal
       from a wrap". The constant had **no consumer**: nothing checked it, and `SpeedFromAngle` already flagged the same
       condition for the same reason. A documented assumption with a constant and no check reads exactly like a checked
       assumption.
     - The comment also names the limit honestly and the limit is the interesting part: with a *wrapped* input a step of
       half a turn or more cannot be detected at all -- a lost *whole revolution* decodes to a step of zero and is
       indistinguishable from standing still. What can be checked is the near-boundary case, which is what
       `SpeedFromAngle` does at `0.9*pi`, and that rule is now here too.
     - Adding the check exposed a second defect in the same function, caught by the new case rather than by reading:
       `GearOutput` was aggregate-initialised with `lost_count_` **before** the flag could be set, so the output told the
       caller nothing for exactly the sample that had tripped it. One line, and it would have been invisible -- the flag
       would have appeared one sample late, which looks like a one-sample delay rather than a bug.
     - ``pending``
   * - A-48
     - A
     - `gear_ratio` reaches L3 through O-4, and this pass is the first where **more than one of my own predictions was
       wrong before the measurement**: two of four, and both wrong in instructive ways.
     - What holds, all derived first: a quantised master costs at most `abs(ratio)*q` and does not grow with time
       because consecutive deltas telescope -- 1.366 mrad measured against a 2.500 mrad bound, 1.316 of it in the second
       half of a four-second run, so the bound is exercised and the structure is confirmed rather than assumed; a sample
       interval off by fraction `j` moves the velocity reference by `ratio*omega*j/(1+j)` (0.95240 of nominal measured
       against 0.95238); a count the encoder never counted costs exactly one count (2.49958 against 2.50000); and a
       *sample* that never arrives costs **nothing** (0.00e+00).

       The two corrections. (1) P4 first said a dropped *sample* costs a permanent step. It costs zero, because the next
       sample carries the full accumulated angle and the unwrap recovers it exactly -- the difference between information
       arriving late and information never existing. The prediction was rewritten around a lost *count*, and the dropped
       sample is now asserted as the control that makes the lost count meaningful. (2) P3's first form ran at 1 rad/s
       master, where the per-sample step is 6.25e-5 rad and `wrap_pi`'s `fmod(x + pi, 2*pi) - pi` loses about 1e-7 of it
       per sample -- a **larger** term than the jitter under test. The case now runs at 20 rad/s where the artefact is a
       hundred times smaller, and the resolution limit is written into the comment instead of being averaged over.
     - The unwrap's own numerics were checked against the F-40 failure mode and are sound: a perfect ramp drifts by
       nothing at 1 rad/s over 40 s (end error 0.000 mrad) and by 0.122 mrad at 20 rad/s, the same at 4 s and at 40 s.
       There is a transient of up to 1.373 mrad at low master speed, which is a measured term this row's 25.6 mrad claim
       does not separate out; it is recorded here rather than folded into a tolerance.
     - ``pending``
   * - F-44
     - F
     - `motion_profile.hpp`'s rationale promised "accel-limited motion with a stopping distance that
       matches the deceleration it is allowed", stated as a property of a move. It is a property of the
       *command*. The axis covers `v^2/(2a)` only if it tracks the command exactly; when the inner loop
       lags, the axis passes the threshold and keeps going.
     - The row was being promoted to L3 and the deficit had to be written down as a number rather than as
       a caveat: the axis passes the deceleration threshold at **86.3 % of it** (a 27.4 mrad deficit on a
       1 rad move). The documentation simply had no place to put that number, which is how it stayed
       unwritten. This is the same class as F-42 -- a shipped sentence making a quantitative promise
       without its domain -- and it was found the same way, by deriving what the claim should say instead
       of re-reading what it does say.
     - Header prose only. What it changes is that a reader can no longer take `v^2/(2a)` as a bound on a
       real axis; the L3 claim row carries the measured deficit.
     - ``pending``
   * - A-49
     - A
     - `motion_profile` reaches L3 through O-5. This is the first row where **no code defect was found**,
       and the analysis was wrong instead, three times, in three different ways.
     - What holds, each derived before it was measured. (1) The two structural landmarks are closed forms:
       the proportional ceiling meets the braking curve at `2a/g^2` (measured 0.0500 / 0.0125 / 0.1000 on
       three `(g,a)` pairs) and the velocity limit takes over at `v^2/(2a)` (bisection ratios 1.0000, with
       the `a^-1` and `v^2` exponents recovered as 0.5000 and 0.2500 rather than restated). (2) The
       shaping gain appears in neither, and it reaches the structure only through `g >= 2a/v` -- which is
       the missing reason behind A-40's null result: at `g = 20, a = 10, v = 2` the boundary is exactly 10,
       so the gain sat on the safe side and no change to it could move the exit. A-40 recorded a
       refutation; this turns it into a condition. (3) The header's "not time-optimal" caveat is narrower
       than it reads: ramp / cruise / constant-deceleration is the time-optimal trapezoid's own phase
       sequence, so a rest-to-rest move takes `D/v + v/a` -- 0.6480 s measured against 0.6553 predicted,
       and 3 rad further costs **1.5000 s against 3/v = 1.5000**, a difference that cancels the ramp and
       the brake and is therefore far sharper than either absolute time.

       The three things I got wrong. (1) The first arrival-time definition was "the first time the gap is
       inside a band", which the axis crosses *early* because it is going to overshoot 8.03 mrad -- so the
       number measured was a crossing on the way out, not an arrival. The fix was to predict the tail that
       is still to be covered at the crossing, and to record separately that the axis overshoots while the
       command cannot. (2) The discrimination probe for `sqrt(2ae)` versus `sqrt(ae)` was placed at
       `e = a/g^2` for one configuration -- which is the *wrong* formula's own crossover, where the two
       readings coincide by construction. The teeth check read 0 % and was right to fail. Probe points have
       to be chosen where the competing formulas are actually apart, which for this pair means above the
       proportional crossover only. (3) The deficit was first modelled as a bare onset error; it is a
       kinematic part plus a lag, `v*(0.05*v/a + tau)`, and `tau` has to be recovered from one
       configuration and used to predict the others (3.692 ms, held-out rows within 2.6 %) -- recovering it
       from all three would have measured nothing.
     - One row. The take-away recorded on purpose: a row that always finds a bug is a row whose tests are
       being written to fit. Here the structure was right and the prose was not, and saying so is part of
       the evidence.
     - ``pending``
   * - A-50
     - A
     - `soft_limit` reaches L3 through O-6. No code defect -- and the one hypothesis I went looking for
       was tested and refuted, which is worth recording as much as a find.
     - What holds. The invariant `v^2 <= 2*a*(d - v*delay)` was only ever *checked*, one period at a
       time, which leaves "how much speed does this distance buy me" unanswered. Solving the quadratic
       for `v` gives the cap's seam, `sqrt(a^2*tau^2 + 2*a*d) - a*tau`, and the seam is an **exact**
       observable: below it the cap returns the caller's float unchanged, above it returns something
       strictly smaller, so bisecting for it needs no tolerance -- 1.000000 on five `(d,a,tau)` pairs.
       From there: homogeneous of degree one in `(d,a)` jointly (k = 0.25 / 0.5 / 2 / 4 recovered
       exactly), and the three sensitivity coefficients differentiate to `a/(v+a*tau)`,
       `v^2/(2*a*(v+a*tau))`, `-a*v/(v+a*tau)` -- measured 69.7744/69.7750, 6.2008e-3/6.2008e-3,
       -1395.23/-1395.34 against the seam itself. On the plant the delay term turns out to buy exactly
       **one delay travel** -- closest approach shifts by `v*tau`, ratio 1.0000 at 20, 10 and 5 rad/s --
       and the cap starts biting at `v^2/(2a) + v*tau` within 1.7 % at all three speeds.

       The hypothesis I could not confirm. The margin term is computed from the **command**, so an axis
       moving faster than it is being commanded should lose protection: command 1 rad/s while the axis
       runs at 20 into a wall. I built the case before believing the reply, and it does not happen --
       closest approach stayed at +0.51 mm for three different switch-over distances, because the cap
       has already bounded the speed long before the command can lie about it. The coupling is
       self-consistent rather than lucky, and the boundary that *is* real (a window moved inward around
       an axis already inside the new margin) is the one the header already names as needing a homing
       switch.
     - The general lesson, written down for the third time in this log in a different costume: **an
       invariant that is only checked is half a specification.** The checked form answers "is this
       period safe"; the solved form answers "what may I ask for", and the second is what a user needs.
       The same sentence applies to `motion_profile`'s stopping distance (F-44) and to `gear_ratio`'s
       `kMaxStepRad` (F-43), which is why it is here rather than in a commit message.
     - ``pending``
   * - F-45
     - F
     - `PositionReference` guards one path against an impossible jump and not the other. A step at or
       beyond `0.9*pi` electrical is *reported* rather than resolved, because picking a direction gives
       an angle that looks fine and is a whole turn out. The index snap had no such rule: it applied
       `wrap_pi(residual)` unconditionally, so a spurious pulse on the index line moved the origin by
       up to **3.14 rad of mechanical angle** -- seven times the electrical step it was not guarding
       against -- and `lost_count()` stayed false.
     - Found while deriving the sensitivity table, not while running a test: the row's second source is
       "the mark measures exactly one turn", which forced the question "what else can move the origin",
       and the answer was the mark itself. Then measured before fixed: a pulse a quarter turn after the
       mark snapped 1.5689 rad, at half a turn 3.1405, and neither reported anything. The next real mark
       moved it back, so in the field this would have looked like a transient nobody could reproduce --
       which is why the case asserts the flag *immediately* after the refused pulse rather than at the
       end of the run.
     - The snap now takes the same rule in the same units: past `0.9*pi/p` of mechanical residual the
       two sources are not describing the same event, so the pulse is refused and flagged. Two things
       fell out. The refused pulse moves the angle by one sample instead of half a turn. And an integer
       pole-pair error became **detectable**: its residue per turn is `2*pi*abs(1 - p/p')`, at least
       `2.22*abs(p'-p)` times the resolution the channel declares, so `p'=6` and `p'=8` are both reported
       from the second mark on. That is a capability the row could not claim before, and it arrived as a
       consequence of the fix rather than as a feature request.
     - ``pending``
   * - A-51
     - A
     - `position_reference` reaches L3 through O-7, with two of my four measurement designs wrong before
       the run.
     - What holds, all derived first, and exact because a counting channel is a sum of integers and one
       division: wrap rate `p*s/(2*pi)` (10.500 / 22.750 / 14.000); reconstructed turn length `2*pi*p/p'`
       (7.3033 vs 7.3304, 5.4775 vs 5.4978); mark jitter displaces the origin by exactly the jitter
       (ratios 0.9975 / 0.9991 / 0.9990 for 1, 10 and 50 mrad); a missed mark costs no counts, only a
       turn of drift (7.09e-3 rad over five turns). Plus F-45, which came out of the derivation.

       The two measurement errors, both of the same kind -- measuring a *difference* where I thought I
       was measuring a *term*. (1) Mark jitter was first read off "the last snap", which is the drift:
       the first pass absorbs the jitter into the origin and later passes only remove drift, so the
       number was identical for 1, 10 and 50 mrad. Differencing a jittered run against an otherwise
       identical clean run isolates it, because both do the same arithmetic on the same samples and the
       drift cancels. (2) The same mistake in the boundary case, fixed the same way. A third, smaller
       one: with a 0.02 rad sample step the pulse can land anywhere in a 0.02 rad window, which is
       larger than the smallest jitter under test -- the grid was being measured, not the mark; the
       jitter cases now run on a 2e-4 rad step.
     - Two rows. The general lesson is the same one `soft_limit` produced one row earlier in a different
       costume: **a term and a difference are different measurements**, and the difference is what
       survives when the term is absorbed by state.
     - ``pending``
   * - F-46
     - F
     - The decimated outer loop does not run at the period it is asked for, and the second half of the gap
       is not inherent: the loops run on whole control ticks, so the period is quantised to
       `dt*ceil(period/dt)` -- and because `speed_elapsed_s_` accumulates `dt` in **float32** and compares
       with `>=`, the rounded sum can land just below the request and the loop waits one more tick. A
       requested 1.000 ms runs every **21** ticks (1.050 ms); 0.250 ms and 0.500 ms land exactly.
     - Found by measuring the period instead of restating it: the row's second source needed the loop's
       instants, which the trace channel already publishes as the spacing between current-reference
       updates. Six decimations then showed the pattern above, and 1.2 ticks and 2.0 ticks turning out to
       run *identically* is what made the quantisation visible at all.
     - Recorded and documented, **not** changed -- and the reason is written down rather than implied.
       Making the comparison epsilon-tolerant or the accumulator double would change which tick fires in
       every decimated loop in the library, with no failing case behind it; the deviation is bounded by
       one control tick and does not grow; and a caller who needs an exact ratio can now read the actual
       period instead of assuming it. `motor_config.hpp` says all of that where the field lives.
     - ``pending``
   * - A-52
     - A
     - `foc_loop` reaches L3 through O-8. The interesting part is that the row's second source turned out
       to be **sharper than the first**: the z-domain model predicts a number the code does not contain.
     - What holds. The period is a lower bound and the unit is the tick, measured from the trace channel
       (1.0 / 1.2 / 1.5 / 2.0 / 5 / 10 ticks request 1 / 2 / 2 / 2 / 5 / 10, and 1.2 and 2.0 run
       identically); the float32 accumulation adds one tick at 20, 50, 100 and 160; and the response is
       flat to 0.7 % from 1x to 140x decimation (settling 0.0690 against 0.0685 s) before breaking at the
       model's own marginal point -- `omega_b*T = 2`, i.e. **160x** at 250 rad/s and 50 us -- with the
       settling time going 0.0685 -> 0.1058 -> 0.4000 s at 140x / 160x / 180x and the overshoot
       1.73 -> 3.62 -> 7.28 %.

       The one thing I had wrong at the start of the row: I expected the 250 to be hertz. It is rad/s,
       which moves the boundary from 25x to 160x and changes the whole story -- at 25x the original case
       would have been sitting on the edge, at 160x it is comfortably inside. Reading the unit is not a
       measurement, but it decides what a measurement means.
     - The lesson this row adds: **a model that predicts a number the code does not contain is worth more
       than one that restates the code.** `omega_b*T = 2` appears nowhere in the source; it comes from the
       difference equation, and the plant agrees with it to the tick across a 180x sweep.
     - ``pending``
   * - F-47
     - F
     - `MagnetPolarityTest` rests on one premise and never checked it. The method is "the *same* volt-second
       pulse in two directions", which is only meaningful if both pulses start from the same current.
       Nothing verified that. A `settle_s` shorter than the winding's `L/R` leaves the second pulse starting
       on the first pulse's tail -- where the incremental inductance has already collapsed -- so the second
       rise is larger for a reason unrelated to the magnet, and the verdict comes out confidently wrong.
     - The angle sweep found it, and the number is what makes it an F rather than a caveat: at a zero-length
       settle the asymmetry is **-0.5404 A** for a machine whose magnet is `Aligned` -- the honest value is
       **+0.5651 A**. Same magnitude to 4 %, opposite sign. Nothing downstream can catch that, because the
       only thing separating the two is a sign the caller was never given a way to check.
     - `decide()` now records the current at the start of each pulse and refuses to answer when the two
       differ by more than `min_asymmetry_a` -- the caller's own declared resolution, which is the right
       yardstick because below it no verdict is supportable anyway. The crossing is a closed form: the
       residual decays as `exp(-t/tau)` with `tau = Ld/Rs = 571 us`, so the guard releases at
       `tau*ln(1.791/0.020) = 2.57 ms`; measured refused at 2 ms and answered at 3 ms, the first grid point
       above the prediction.
     - ``pending``
   * - A-53
     - A
     - `polarity` reaches L3 through O-9. Two of my predictions were wrong, and one of the two was wrong in
       a way that says something about fixtures rather than about the code.
     - What holds: the answer is attenuated by `cos^2` of the d-axis error (0.987 / 0.943 / 0.850 at 15 / 30
       / 45 degrees, against a law derived from two projection factors, with the saliency term common-mode so
       it attenuates without contaminating); half a turn out reproduces the magnitude to 0.04 % with the
       opposite sign; and the settle precondition is the closed form above.

       Wrong prediction 1: I expected the window to be set by the `cos^2` attenuation crossing
       `min_asymmetry_a`. Measured, the window closes *earlier* and for a different reason -- at 60 degrees
       the signal is still 4.6x the threshold and the answer is refused because the rotor has been pushed 14
       mrad and the two pulses no longer share a start. Lengthening the settle to 20 ms restores the answer,
       which is what identifies the binding limit. The derivation was right about the *attenuation* and blind
       to the *second* limit, and only an experiment separates them.

       Wrong prediction 2, and the more interesting one: I expected the wrong answer at a zero-length settle
       to be *larger* than the honest one (it is, on the plant I first used: 137 %), and asserted that. On the
       case's own winding it is 95.6 % -- same order, slightly smaller. The claim was machine-dependent and I
       had generalised it from one machine. What is machine-independent is the part worth keeping: the wrong
       answer has the *same magnitude*, so no magnitude check can filter it.

       The same case exposed a fixture that had been lying: `FakeDx` had no decay at all, i.e. a winding whose
       current never relaxes. Every unit case in that file had been exercising the degenerate path -- the one
       F-47 is about -- and getting the right label because the fake keyed inductance on the voltage sign
       rather than the current. The fixture now has a decay constant, `valid_config()` has a settle long
       enough for it, and the degenerate path has its own case asserting `Inconclusive`.
     - Three rows. The lesson: **a fixture that omits the physics the protocol depends on will pass, and pass
       for the wrong reason.** The fake was simpler than the machine in exactly the dimension the feature is
       about.
     - ``pending``
   * - A-54
     - A
     - `feedforward` reaches L3 through O-10. No defect in the feedforward: its q-axis term is the machine's own
       equation to 0.05 %, and the mismatch law is exact. What the pass produced instead is a **boundary it did
       not have**: the d axis is not a usable identification channel, and the reason is outside the module.
     - What holds, all derived first. The mismatch law: a configuration that is `(1+eps)` of the truth leaves
       `-eps` of the term on the PI, measured at 1.025 / 0.975 / 1.010 times the prediction for
       eps = +0.10 / -0.10 / +0.25, with the ratio of the last two at 2.4629 against 2.5 (tolerance 6 %, which
       is two 3 % measurements combined -- not a number chosen after the fact). The machine's steady-state
       equation read through the PI: `vq` matches `Rs*iq + omega*(Ld*id + psi_f)` to 0.05 % and 0.25 % at two
       operating points, recovering `Ld` as 2.0930e-4 against 2.0e-4. And the identifiability, as an
       experiment: at `id = 0` a configuration with four times the true inductance leaves a residual 2e-6
       different, because `Ld` and `psi_f` enter only as their sum.

       The finding, which is a boundary rather than a bug: the d axis reads `Rs*id - omega*Lq*iq + vq*delta`,
       where `delta` is the estimator's angle error, because the command is built in the estimated frame and
       applied in the plant's true frame. Solving the two axes gives `delta = -0.0136 rad (0.78 deg)`, and
       `vq*delta` comes out at **55.5 %** of the cross term the d axis was supposed to measure, while the same
       rotation adds 0.052 % to the q axis. So a one-point d-axis identification of `Lq` is off by 41 % on one
       machine and 55 % on this one, and the budget is explicit: the angle has to be known to
       `omega*Lq*iq/vq = 0.0245 rad` for the channel to work at all.

       Two of my own errors on the way, both in the model rather than in the code. (1) I predicted the d-axis
       voltage from the machine equation and got 41 % off, and my first instinct was a plant parameter; the
       measurement said the *q* axis was right to 0.06 %, which is what ruled out a global scale and pointed at
       a rotation. A discrepancy that is axis-selective is not a gain error. (2) The linearity assertion failed
       at 1.5 % and my first move was to widen it to 6 % -- which would have been exactly the sin this log is
       about. The tolerance is now *derived*: two residuals each known to 3 % give a ratio known to 6 %, and
       the runs are not at byte-identical operating points.
     - One row. The lesson: **an axis-selective discrepancy is a rotation, not a gain** -- and the first
       question about a channel that disagrees with theory should be "what else reaches this axis", not "which
       parameter is wrong".
     - ``pending``
   * - A-55
     - A
     - `blended_observer` reaches L3 through O-11. No defect and no negative result: the module was already
       right, and the pass found that its coefficients are **exact** rather than noisy, which is a property
       worth recording because it means a stub can stand in for the plant for this row.
     - What holds, measured with stub observers whose answers are set by hand. An error in the low source
       reaches the output at `1-w` and one in the high source at `w`, both to 1e-4; a miscalibrated decision
       speed moves the angle by `disagreement/span` per rad/s, 0.006000 predicted and 0.006000 measured; the
       weight is 0.25000 / 0.50000 / 0.75000 for 35 / 60 / 85 rad/s against a 10..110 band; and the
       interpolation is on the circle, so 3.0 and 3.2832 blend to 3.14159 rather than to 0.0.

       The one thing the row did not have and now does: a handover sweeps the published angle by the two
       observers' disagreement while neither of them moves -- 0.60000 rad swept for a 0.60000 rad disagreement
       -- and the published *speed* shows none of it, because it is the same convex combination. So an angle
       and a speed integrated from that speed disagree during a handover by exactly the disagreement. That is
       not a bug; it is the shape of a handover, and a consumer that reconstructs position by integrating the
       speed needs to know which one to believe. Recorded as a property rather than fixed, because fixing it
       would mean making the speed the derivative of the blended angle, which would feed the handover's own
       sweep into the speed and reintroduce the feedback path the design deliberately keeps out.
     - One row. The lesson is the cheapest one in this log and the easiest to forget: **when the algorithm is
       a convex combination, the sensitivity table is the algebra of a convex combination** -- there is nothing
       to fit, and the only work is choosing inputs where the coefficients are distinguishable.
     - ``pending``
   * - A-56
     - A
     - `hfi_observer` reaches L3 through O-12. No defect -- the carrier's physics was already documented and half of
       it already had cases -- and the pass produced two corrections **to claims I would otherwise have made**.
     - What holds, each against arithmetic rather than against a previous run. Injection amplitude: the demodulated
       amplitude is linear in `V_h` (1.0000 / 2.0000 / 4.0000 for 6 / 12 / 24 V), so the amplitude is a
       signal-to-noise decision and not a calibration. Saliency: the signal is the *admittance difference*
       `(Ld-Lq)/(Ld*Lq)`, measured 0.5168 and 0.2618 against 0.5000 and 0.2500 on machines scaled by 2 and 4. Sample
       timing: `max_inject_hz = control_hz / kMinSamplesPerCarrier`, exactly 8000.0 Hz at 20 kHz, accepted at the
       limit and refused with `Param` 5 % past it.

       Correction 1, and the interesting one. `hf_current_a` is **3.601630** times `predicted_gain_a` at every
       amplitude -- a constant, not a coincidence, because it is the demodulator's own scale (the multiplier's 1/2,
       the filter, the normalisation by `demod_gain_a`). My first assertion was that the ratio should be 1 within
       20 %, and it failed at 3.60. The honest test is that the factor is the **same** at every amplitude and every
       machine, which is what pins the formula's *shape*; asserting the value would have meant fitting a scale and
       calling it physics.

       Correction 2, in the choice of experiment. My saliency sweep included a machine at `Ld = 1e-4` against a
       0.35 ohm stator and the law was 9 % out there -- correctly, because the closed form ignores `Rs` and
       `w_h*L` is only 3.6 times `Rs` at that point. The neighbouring case in the same file already documents that
       degradation. A table that straddles the domain boundary tests neither side, so the machines now scale **up**
       (14 and 29 times `Rs`) and the domain is written into the case.
     - One row. The lesson: **verify a formula's shape where it is valid and its value where it is not** -- and
       when an instrument has a scale of its own, measure that scale and report it, because the alternative is a
       tolerance wide enough to hide a wrong coefficient.
     - ``pending``
   * - A-57
     - A
     - C8's licence matrix exists as the seventeenth project gate. It is an A rather than an F because nothing was
       wrong: the measurement had been done (`test_daxis_observer_limit.cpp`) and what was missing was a register
       that forces a verdict.
     - What it is. `Motor<Observer, ..., RefGenerator, ...>` takes both slot policies as template parameters, so every
       pairing compiles -- including MTPA against an observer whose flux model has no d-axis term. The matrix is
       `.github/project/combinations.json` with four verdicts (`allow`, `conditional`, `forbid`, `unmeasured`), and
       `combinations_gate.py` refuses: a pair discovered in the headers with no cell and no wildcard, an unknown
       verdict, a `conditional` with no `condition` or no `enforced_by`, a `forbid` whose `falsifier` is not a case
       that exists in the test tree, an `unmeasured` with no reason, and any `evidence` file that does not exist.
       Five mutations prove each of those can go red.

       The two decisions worth recording. (1) `forbid` needed a `falsifier` before any cell could use it, which is why
       the current matrix has **zero** forbidden cells: the one pairing that looked forbidden turned out to be
       conditional, because `MtpaReference::i_d_limit_a()` already implements the rate. Declaring it forbidden would
       have been a claim with nothing behind it. (2) `unmeasured` is a verdict, not a gap -- it has to say what would
       have to be measured -- because "we did not think about it" and "we decided it is fine" must not look the same in
       a file. The `BackEmfObserver` cell uses it, with the argument that it *should* be harmless (0.042 rad at
       `i_d = -1 A`, inside the 0.15 rad ceiling) written down as an argument rather than as a result.
     - Deliberately **not** registered in `observability.json`: that registry is for capabilities that re-open existing
       claim rows, and a gate that enforces a register re-opens none. Applying the rule means saying where it does not
       apply, not only where it does.
     - ``pending``
   * - A-58
     - A
     - Stage 4 closes, and it closes with a correction to itself: the plan listed "add `ld`/`lq` to the
       feedforward" as the next capability, and `DecouplingFeedforward` had used both since it was written --
       `v_d -= omega*Lq*iq`, `v_q += omega*Ld*id`, with `feedforward_config` taking both from the nameplate.
     - Two things were wrong and only one of them was the plan. The other was that the *dual-inductance*
       property had never been measured, only read off the source, so a single-`L` regression would have
       passed every existing case. On a machine with `Lq/Ld = 3` that regression is a factor of three in one
       of the two terms. The new case differences two runs that share every other detail -- exact feedforward
       against one with a single parameter scaled -- because the d axis carries a `vq*delta` confound measured
       at 55.5 % of the cross term in the case above, which would swallow a 10 % mismatch whole. Measured:
       `Lq` scaled leaves `+0.007851` on d (predicted `+0.007823`) and `+0.000000` on q; `Ld` scaled leaves
       `+0.014128` on q (predicted `+0.014126`) and `+0.000016` on d. Each inductance reaches its own axis and
       only its own axis.

       What is left of C8 is a genuine capability, not a document: the observers are single-`L`
       (`BackEmfObserver` takes one inductance, `NonlinearFluxObserver` uses `lq`), which is why a non-zero
       `i_d` can only be granted a *rate* rather than modelled. That belongs to PLAN G1's capability schedule,
       not to stage 4's "turn the known boundaries into a decidable refusal".
     - The lesson is the one this plan's own preamble predicts: **a plan is a draft and the code is the fact,
       and the plan that is never re-read invents work that is already done.** The feedforward item survived
       three re-readings because it read like a sensible next step.
     - ``pending``
   * - A-59
     - A
     - Stage 5 opens by refuting its own criterion. The plan asked for "safety-subset MC/DC >= 90 %"; the baseline
       says the safety subset (`protection`, `limits`, `sm`) contains **six** conditions in total, so one condition
       is worth 16.7 points and two pass/fail results swing the "gate" by 33. A threshold that a single test can
       move by a third is not a threshold, and `llvm-cov report --show-mcdc-summary` counts only *multi-condition*
       decisions, which almost none of the protection logic is.
     - What the measurement did buy. Two conditions had never been exercised, and neither was a missing assertion:
       `protection.hpp`'s `lost` decision short-circuits on `!flux_valid` and no case had ever fed an invalid flux,
       so "my flux estimate is not valid" reaching `LostSync` with no error ratio at all was untested behaviour;
       `sm.cpp`'s `request` guards `to >= kStateCount` and removing that term does not weaken a check but changes
       the answer -- `kLegal[from][4]` reads past the table, happens to be `true`, and `request(4)` returns `Ok`
       while moving the machine into a state that does not exist, with AddressSanitizer silent. Both now have a
       boundary pair differing in one input, and both pairs are proved able to go red (delete the flux term: the
       fault code falls back from `LostSync` to `None`; delete the range term: `request(4)` says `Ok`).
     - The criterion that replaces it is per decision, not per subset: every multi-condition decision in the safety
       subset must have all of its independence pairs covered, and the subset must contain a floor number of such
       decisions so a perfect score cannot be a perfect score for measuring nothing. `.github/project/mcdc_gate.py`
       enforces both and is the eighteenth project gate, local-only because the instrumented build takes two
       minutes. Measured: `protection.hpp` 2 decisions (4 conditions), `sm.cpp` 1 decision (2 conditions),
       `limits` none -- 3 decisions, all covered.
     - The proof that the gate is not decoration is a mutation the *whole suite cannot see*: writing the range
       comparison twice (`to >= kStateCount || to >= kStateCount || ...`) keeps every case green, because the
       duplicate is masked by the first term, and the gate goes red at 66.67 % with "no independent pair for C2".
       That is the one thing MC/DC buys over a passing suite, and it is worth exactly one gate.
   * - F-60
     - F
     - The hysteresis clearing bands were a claimed behaviour with no test. `Protection.hpp`'s class comment says
       "hysteresis on exactly the faults that self-clear: a bus hovering on the threshold would otherwise chatter the
       output enable at the control rate", and the mutation baseline says no case had ever made a tripped fault
       *stop* being tripped: **eleven** mutants in the four clearing expressions survived, every one of them
       arithmetic on `x*(1 -/+ h)`.
     - It took a dyadic configuration to catch them. The shipped `t_trip = 100, h = 0.05` puts the clearing
       threshold at 94.999999... in float32, a value no assertion can stand on, so the cases use `128*(1-0.5) = 64`,
       `32*(1-0.5) = 16` and `8*(1+0.5) = 12` and probe exactly on the threshold (still inside the band, because the
       clear is strict) and one step past it. The `I2t` band needed a third idea: its threshold lives in A^2*s and its
       state is an accumulator, so the case charges at 4 A (3 A only *approaches* the budget asymptotically and never
       crosses it -- the first version of that case failed its own trip assertion, which is the harness doing its
       job), then idles long enough to sit inside the band and longer again to fall below it.
     - `protection` went from **30.0 %** (15/50, and truncated at `--limit 60`) to **71.6 %** (48/67) over the same
       file set, with all 19 remaining survivors classified.
     - ``pending``
   * - F-61
     - F
     - `latched_mask` was written and never read by any test. Two mutants survived and they are the whole of that
       field: `= 0` turned into `= 1` on a healthy step, and the bit-OR `1U << code` doubled into `2U << code`.
       Every case reads `fault` or `active_mask`, so the field a caller uses to decide "this one will not clear on
       its own" had no witness -- including the negative half, that a self-clearing fault must *not* appear in it.
     - 
     - 
     - ``pending``
   * - F-62
     - F
     - The `i2t_cooling_s > 0` guard had never been taken. The mutant makes it `>=`, which turns `dt / 0` into
       `inf` and empties the thermal budget every step; nothing noticed, because every configuration in the suite
       supplies a positive thermal constant, so the branch that exists *for* the zero case was never executed. A
       zero cooling constant is a legitimate configuration ("no cooling"), and with it the budget is exactly
       `i^2*t`: 3 A for 32 steps of 1/64 s is 4.5 A^2*s, the budget itself, and the thirty-third step is the first
       that spends past it.
     - 
     - 
     - ``pending``
   * - F-63
     - F
     - Four constants in `sm.cpp` had no consumer: `kIdle`, `kAlign`, `kRun` and `kFault`. Changing `kIdle` from 0
       to 1 changed nothing, which is how mutation testing found them -- the same defect class as F-43's
       `kMaxStepRad`. They exist to say which row of `kLegal` is which state, and the table's rows are positional,
       so the claim lived only in a comment.
     - They are now `static_assert`s against `to_index(MotorState::...)`, which makes the comment a compile-time
       check: reordering `MotorState` stops compiling instead of silently permuting the transition table. The
       consequence for the measurement is worth stating rather than hiding -- `sm`'s denominator **shrank** from 13
       to 8 because five constant mutants became compile errors, so its score moved from 61.5 % to 100 % on a
       smaller set. A score is only comparable across the same instrument, and this changed the instrument.
     - 
     - ``pending``
   * - F-64
     - F
     - The slew limiter's error term could have its sign flipped and nothing would notice. Replacing `target -
       value_` with `target + value_` survived the whole suite, because every existing case either starts from zero
       or has a target large enough to dominate the current value -- the two expressions then agree on which branch
       they take, and the branch is all the output depends on. The new case starts at 5 and asks for 3: the
       difference is -2, twice the one-step budget, so the output must land on 4, while `target + value_` reads +8
       and would walk it *up* to 6, away from its own target. `limits` went from 14.3 % to 28.6 % (2/7).
     - 
     - 
     - ``pending``
   * - F-65
     - F
     - Two defects in the mutation harness itself, both of the kind that make a measurement lie. (1) The generator
       only stripped `//` comments, so it mutated constants **inside string literals**: four of `sm`'s five
       survivors were `"... row 0 is Idle"` and its siblings, i.e. mutants no test could ever distinguish, presented
       as work for a human. `_masked()` now blanks string and character literals and one-line block comments,
       length-preserving so reported columns still point at the original line, and the selftest grew a case that
       requires a message string to produce no mutant. (2) `_compile` passed the module's `extra` list to `g++`
       verbatim, so a module whose test instantiates the whole `Motor` could not be built without listing every
       source by hand -- and a hand-written list rots silently. Globs are now expanded and deduplicated.
     - 
     - 
     - A third thing was not a defect but a result worth keeping: the first `protection` run hit `--limit 60` and
       said so (`[PARTIAL: ... hit the limit]`), which is the harness reporting that a truncated scan is a score of
       a file prefix rather than of a module. The final numbers use the whole file.
   * - F-66
     - F
     - `reset()` and `reset_cold()` are public operator actions and had **no test that called them**. The class
       comment promises three things -- "clear the latched faults, an operator action, never automatic", "`I2t` is
       *not* cleared here: the winding is still hot", and `reset_cold()` is the one that forgets the heat -- and not
       one of the three had a witness. Line coverage was satisfied because `bridge.cpp` reaches `reset_cold()` from
       the C ABI, which is exactly the kind of green that means nothing: the module's own tests never walked the two
       loops.
     - Sanitisers found it, and the measurement that matters is that **one was not enough**. A one-line probe (a
       struct with `bool a[8]`, then `a[8] = false`) reports nothing under AddressSanitizer and one finding under
       UndefinedBehaviorSanitizer, because ASan's redzone surrounds an object and not the members inside it:
       `flags[8]` in `step()` is a standalone local and dies under ASan, while `tripped_[8]` is a member and only
       dies under UBSan's `array-bounds`. The harness now builds with `-fsanitize=address,undefined
       -fno-sanitize-recover=all` -- the last flag because UBSan otherwise prints a line and still exits 0, which
       the harness would have recorded as a survivor.

       Two of the six surviving overruns then turned out to be *equivalent* rather than invisible, and only the
       instrument could tell: `reset()`'s write is guarded by `fault_is_latched(code)`, which is false for the
       out-of-range code in the ninth iteration, so the overrun never executes. Recording them as "needs a memory
       checker" would have been a wrong label kept forever; being able to say "equivalent, measured" required the
       tool that can see the writes that do happen. `protection` closed at **77.6 %** (52/67) with 15 survivors,
       all classified, none of them invisible.
     - The general lesson is about coverage rather than about this block: a function reached only from another
       layer's tests is a function whose *own* contract is untested, and no percentage will say so.
     - ``pending``

   * - A-67
     - A
     - Stage 6 was opened by measuring whether it can be executed at all, and the answer is not yet: the third layer
       (a geometric model of the machine) has **no input** in this repository. `core/machine_params.hpp` carries four
       lumped electrical quantities -- `rs`, `ld`, `lq`, `psi_f` -- plus `pole_pairs`, and a search of every header,
       JSON, script and document finds no bore, stack length, slot count, air gap, magnet thickness or lamination.
       The parameters are quoted as numbers without the geometry they came from, so "compute `Ld`/`Lq`/`psi_f`
       independently from the drawing" has nothing to compute from.
     - The tools are the smaller half, and they were checked rather than assumed: `pyfemm` and `femm` are absent,
       there is no FEMM binary and no `wine`, and `getdp`, Elmer and FreeFEM are not installed either. What exists is
       `gmsh` (which meshes but does not solve a field) and numpy/scipy in `workspace/materials/.venv`.

       The temptation here is to invent a plausible geometry and then "independently" derive the numbers the code
       already quotes -- which would be circular in exactly the way §3.1 of the plan exists to prevent, and would be
       indistinguishable in the write-up from a real check. So the layer is recorded as blocked instead of faked, with
       two named unblocking conditions: a real machine's datasheet filed in `workspace/materials/` and registered in
       `error-sources.json` as an error source, or an explicit "reference design" whose correspondence to
       `ld`/`lq`/`psi_f` is stated to be defined rather than fitted. Tooling side, `pip install scikit-fem` into that
       venv is enough for a 2D magnetostatic solve (`A_z` weak form) with no sudo and no wine.
     - One thing this does **not** license is calling an algebraic re-derivation a third model. The same equations
       written twice are the same physics in different notation; that is what an L3's "model-level second source"
       already is, and it is why the geometric layer is the only one that could catch a wrong *assumption*. Until it
       exists, the L3 rows keep the basis they have and none of them claims geometric independence.
     - ``pending``

   * - A-68
     - A
     - The sufficiency question for stage 6 was answered by fetching rather than by recalling, and the answer splits
       in two. Geometry is not the problem: `pip download pyleecan` yields a 9.2 MB pure-Python wheel whose
       `Data/Machine/` directory ships complete 2D geometries for named machines -- `TESLA_S`, `AUDI_eTron`,
       `IPMSM_A`, `SPMSM_001..015`, `SynRM_001`, `SCIM_001/006`. `IPMSM_A` alone carries 48 slots, 4 pole pairs,
       83.82 mm of stack, stator radii 55.32/80.20 mm, rotor radii 80.95/134.62 mm and the full slot and magnet-hole
       dimensions, which is everything a 2D magnetostatic solve needs.
     - Reference *values* are the problem, and the same wheel is why. This version of pyleecan ships only
       `MagFEMM`/`FluxLinkFEMM`, and even the equivalent-circuit path (`EEC_PMSM.comp_parameters()`) reaches for
       `comp_fluxlinkage()` on top of FEMM. There is no FEMM binary here and no wine, so an authoritative geometry can
       be borrowed but no authoritative number comes with it.

       That changes what the third layer can honestly be. Not "borrow a geometry, read off `Ld`/`Lq`, compare" --
       that would be comparing against a value we produced ourselves and calling it external. Instead: write the 2D
       magnetostatic solver (`scikit-fem 12.0.2` is a pure-Python wheel, installable without sudo), first validate
       *the solver* on a case whose answer is closed-form and derived independently of this repository's model, and
       then put the validated solver on an authoritative geometry to measure what the lumped model's assumptions --
       sinusoidal air-gap flux, no slotting ripple, no cross-saturation -- are actually worth. The output is an error
       term for the L3 rows' budgets, which is the one thing a geometric model can supply and no algebraic
       re-derivation can.
     - The MCU question is separate and worth separating: anchoring on a specific part (STM32G4 was the earlier
       proposal) supplies no magnetics at all, because the MCU never enters the magnetic model. What it does supply is
       numbers for the *timing and quantisation* error sources the budget table currently assumes -- PWM duty
       resolution from the high-resolution timer, ADC LSB and sample jitter, dead-time granularity -- so it is a real
       gain for the hardware-adjacent rows and no gain at all for this one.
     - ``pending``

   * - A-69
     - A
     - The FEMM question was settled from the vendor's own FAQ rather than from memory, and the answer rules out the
       path that looked most appealing. "Is there a native Linux version of FEMM?" -- "**No.** However, FEMM has been
       tuned so that it runs properly on Linux machines via **Wine**", and FAQ entry 29 adds that the release is
       deliberately built with Visual Studio 2008 because newer builds misbehave under Wine. The source is an
       MFC/Win32 GUI project, so there is no "clone it and build it here" either: the only route is Wine.
     - What could be done without privileges was done and recorded. The official x64 release for 22 Oct 2023 was
       downloaded and its SHA256 matches the vendor's published file character for character
       (`fdc5047d2fccc4d759df2a82c4627e46408250c57967773fb5bd59cd9eb17253`), so the artefact is the real one; the
       installer was identified from its own bytes as Inno Setup 6.1.0, which fixes the silent-install flags in
       advance (`/VERYSILENT /SUPPRESSMSGBOXES /NORESTART /DIR=`) instead of discovering them by trial; and
       `xvfb-run`/`Xvfb` are already present, which is what a windowless GUI run needs.

       The single missing piece is Wine itself, and it is the one step that needs a privileged install: apt offers
       `wine`/`wine64`/`wine-binfmt`, docker exists but the account is not in the `docker` group, so the container
       route is closed too. Everything after that install is userland: a private `WINEPREFIX`, the silent installer,
       and pyFEMM 0.1.3's file-based message passing to drive FEMM headless.
     - Worth keeping in view for what this buys: a Wine-hosted Windows binary is perfectly good as a *third model*,
       because the physics does not care how the executable was built. What it must not become is an excuse to skip
       validating the case against a published answer -- a solver that runs is not yet a solver that is right.
     - ``pending``

   * - A-70
     - A
     - FEMM now runs on this machine and returns numbers, and the route there was five measured findings rather than
       a download. (1) The vendor has no native Linux build and the source is an MFC GUI project, so Wine is the only
       path -- but the *installer* is 32-bit while every shipped binary is `PE32+ x86-64`. That split is what makes
       `innoextract` the right tool: unpack the installer, never run it, and no i386 multiarch or `wine32` is needed.
     - (2) The binaries then fail on `mfc90.dll`/`msvcr90.dll`, which FEMM does not bundle. They are inside the
       vendor's own `vcredist_x64.exe`, whose cabinets are LZX -- not readable by the pure-Python CAB library -- so
       `cabextract` on the nested `vc_red.cab` yields the amd64 DLLs, which are deployed app-local, the manner the
       redistributable licence is for.

       (3) `PyPI pyfemm 0.1.3` is not the interface to use: its first line imports `win32com.client`, i.e. it drives
       FEMM through ActiveX and is Windows-only regardless of what the changelog says about non-Windows transports.
       The Lua entry point is the real one.

       (4) `csolv.exe` cannot be driven directly. It reads `<name>.node` and `<name>.ele`, which the GUI produces from
       a `.poly` it writes internally and hands to `triangle.exe`; a hand-built `.fem` has no mesh to load, and the
       failure is a modal "problem loading input file" box that looks like a hang. Photographing the Xvfb screen was
       what turned that silence into a sentence.

       (5) With `Xvfb` and `femm.exe -lua-script=... -windowhide`, FEMM's own `Force-vs-Position.lua` completes 21
       solves and writes a 3.37 MB `temp.ans`. A purpose-written probe then pushed a value out of the GUI:
       `block_integral_19 = 27.29711881055766`, exit 0, with `quit()` closing the process cleanly so the wrapper can
       be used from a script. The wrapper is `workspace/tools/femm/femmenv.sh`.
     - The discipline that still applies: this proves the toolchain *runs*, not that it *computes correctly*. The
       next step is the one that has always been next -- put the solver on a case with a published answer (FEMM ships
       `TEAMProblem28.fem` and thirteen other example problems) before trusting it on any geometry of ours.
     - ``pending``

   * - F-71
     - F
     - The first cross-check ran and **failed**, which is the most useful thing it could have done on its first
       outing. The case was chosen so that the answer is closed-form and the mechanism is the one `Ld`/`Lq` needs: a
       coaxial cable in 2D planar, current-driven region, flux linkage read back from the solution,
       `L = mu0/(2*pi)*ln(b/a) +` the internal term. FEMM returns **4.980675912391254e-08** against an expected order
       of 1e-7 to 4e-7, and probing the vector potential on a radial line gives `A = 0` at three radii.
     - Three of my own API misuses were found and corrected from the vendor's Octave wrappers (which carry the
       argument names, unlike the PDF manual, whose text is not machine-readable here): `mi_addmaterial` takes
       **two** permeabilities before the coercivity, so my `(1, 0, 0)` "fix" had set `muy = 0` and degenerate the
       material; `mi_addcircprop`'s third argument is the circuit type where 0 is current; and `mi_setblockprop`'s
       last argument is **turns**, where 0 turns means no ampere-turns at all and therefore exactly the `A = 0` that
       was observed.

       Correcting all three changed nothing: the same flux linkage to 16 significant figures, including after
       deleting `coax.ans` so the read could not be a cached solution. A deterministic identical answer across three
       materially different setups means the setup is systematically wrong in some way those three changes did not
       reach, and the report is not "the solver is wrong" -- it is "the case has not yet been shown to be set up
       correctly".

       One internal inconsistency narrows it: `A = 0` at every probe with a non-zero flux linkage cannot both be
       true, and the flux linkage is computed from `A`, so the suspect is the probe's return convention
       (`mo_getpointvalues` returns several values) rather than the field. The next test is therefore chosen to
       separate the two: open one of the vendor's own `.fem` files, analyse it, and read `A`/`B` at points. If the
       probe reports non-zero there, the probe is fine and the geometry is at fault; if it reports zero there too,
       the probe is at fault and no geometry conclusion was ever valid.

       The bisection finished, and it cleared the probe and convicted my geometry. On a vendor-authored `.fem` the
       probe returns non-zero `A` and `B` (2.6e-4 down to 5.3e-5 across the radius) with the force integral still at
       27.297, so `mo_getpointvalues` was never the problem. The coaxial model was: **the boundary condition had
       never been applied**. Every one of the four arc segments carried boundary index 0 in the written `.fem`;
       `mi_selectcircle(0, 0, b, 4)` selected nothing at all (its `editmode` argument does not mean what I assumed),
       and standing on each arc in turn (`mi_selectarcsegment` at the four quadrant points) is what put `bc = 1` on
       the outer pair. The moment it did, `A(b)` fell to 2e-10, `A(a)` rose to 3.215e-7 and the flux linkage became
       3.713170643894499e-07.

       Criterion half 1 **passes**: against the closed form written down before measuring, 3.7188758e-07, the error
       is **0.153 %**, inside the 0.5 % the criterion allowed. Criterion half 2 **fails**: the refinement sweep gives
       0.153 %, 0.220 %, 0.051 %, 0.856 % for 4, 8, 16 and 32 arc segments -- not monotone, and worse at 32 than at
       4. By the rule as written ("both or neither"), the solver is **not** validated yet.

       The failure is in the experiment, and it is instructive: `segments` is the number of straight arc segments
       that approximate each circle, so sweeping it changes the *shape* being solved (an inscribed polygon whose
       radius and conductor area both move) at the same time as the discretisation. The analytic value belongs to two
       true circles, so the sweep measures a mixture of two errors and can move in either direction. The corrected
       experiment keeps the geometry fixed at a high arc count and refines the **mesh** (`mi_probabl` precision, or
       the block mesh size), which is the discretisation the convergence claim is actually about.

       That corrected experiment was run, and it turned up two more knobs that silently do nothing --
       `-lua-var=SEGMENTS=8` never reached the script (the reported `segments` stayed 4 for every run, so a "flat"
       sweep was a harness artefact, not a result), and `meshsize` is ignored while `automesh = 1`, so four mesh
       settings spanning 8x returned byte-identical solutions. Both are the same failure shape: a control that
       changes nothing, which reads exactly like a converged answer.

       With the geometry fixed at 32 arc segments and the mesh swept for real, the flux linkage is stable at
       **3.750680513287153e-07** (0.01, 0.005, 0.0025 m) and moves only at the finest mesh, 3.753957692770008e-07 for
       0.00125 m. Against the closed form written down in advance, 3.7188758e-07, that is **+0.86 % and moving away
       from it**, so criterion half 2 still fails and the solver remains unvalidated.

       Where that leaves it, stated as a question rather than a conclusion: the discrepancy is either the arc-segment
       geometry or **my closed form for the convention FEMM actually reports**. There are two candidate expressions,
       `L_ext + mu0/(8*pi)` = 3.7189e-7 and `L_ext - mu0/(8*pi)` = 2.7189e-7, differing by the sign of the partial
       internal linkage, and the measurement sits 0.86 % above the first. Choosing between them by which one the
       measurement is closer to would be fitting the physics to the answer, so the derivation has to be done properly
       before the next run -- and until it is, no real machine geometry is touched.

       It was neither: the third knob was misread, and that is what the sweep was measuring. `mi_setarcsegmentprop`'s
       first argument is **`maxsegdeg`**, the maximum angle *in degrees* that one meshed segment may span, not a count
       of segments. So "4" was the finest setting and "128" the coarsest, and the whole refinement sweep ran
       backwards -- which is exactly why refining appeared to make the answer worse. Three controls in this exercise
       turned out to mean something other than what their names suggested (`-lua-var`, `automesh`, `maxsegdeg`);
       the vendor's `.m` wrappers name the arguments, and reading them is cheaper than one wrong sweep.

       With the knob swept in the direction it actually means, the case passes both halves of the criterion and the
       closed form is confirmed rather than rescued. Splitting the two terms first is what made it readable: the
       external flux is `A(a) - A(b)` straight from the probe, so the internal part is a measured difference rather
       than a subtraction of two conventions. -0.179 %, -0.110 %, -0.112 %, -0.102 %, -0.096 % on the external term
       for 8, 4, 2, 1 and 0.5 degrees per segment; **4.9924, 4.9877, 4.9916, 4.9930, 4.9911 e-08** for the internal
       term against `mu0/(8*pi) = 5.0e-08`, i.e. within 0.25 % at *every* setting rather than converging from
       somewhere wrong; total error -0.176, -0.129, -0.120, -0.107, -0.107 %, non-increasing and flat between the
       last two, which is what convergence looks like.

       So the magnetostatic solver, the current source and the flux-linkage extraction are now validated on a case
       whose answer was written down in advance, to **0.11 %** with the criterion at 0.5 %. What that does *not*
       validate is anything with a rotor, a magnet or a slot in it -- which is the next case, and the reason this one
       was chosen as a gate rather than as a result.

       The lesson recorded for the next reader: the cross-check criterion was written **before** the measurement
       (`|delta|/L <= 0.5 %` plus monotone convergence under refinement) and the measurement came out three orders
       of magnitude outside it, so nothing downstream could have been mistaken for validated. A check that cannot go
       red is decoration; this one went red immediately and stopped a real-geometry run that would have produced
       numbers nobody could defend.

       ``pending``
     - Two wiring defects found on the way, both of the "absence reads as success" kind. `gates.py combinations`
       and `gates.py mcdc` had no single-gate branch, so they fell through to the aggregate, ran *every* gate and
       printed "all N" -- the same failure the file already carries a comment about from `plantref`. The
       fall-through is now refused explicitly: an unknown gate name is an error. And `src/version.c` must be
       compiled as C++ by the instrumented build, because `include/foc/version.h` declares `foc_version_string`
       without `extern "C"` and the repository's own build hands the file to `g++`, which compiles a `.c` by
       suffix as C++; instrumenting it as C defines the unmangled symbol and leaves every caller unresolved.
     - ``pending``

   * - A-72
     - A
     - Two vocabularies inside one tool, and one signal that lies. `mi_probdef`'s third argument is `"axi"` in the
       Lua binding, while the `.fem` writer stores the same fact as `[ProblemType] = axisymmetric`. Passing the
       spelling the file uses -- `"axisymmetric"` or `"Axisymmetric"`, both tried -- is rejected outright: `error:
       Unknown problem type axisymmetric`. The accepted token appears nowhere in the shipped `.m` wrappers, whose
       comments are stripped, and exactly once in the whole distribution: `examples/demo2.m`, an Octave demo, as
       `mi_probdef(0, 'millimeters', 'axi', 1.e-8, 0, 30)`. The second vocabulary is the geometry one: a magnet may
       not be drawn as a rectangle nested inside the air box, because both then carry an edge on `r = 0` over
       `-L/2..L/2` and the two coincident segments read as an overlap to the mesher. Both regions are written out
       node by node here so that the air and the magnet share exactly three edges.
     - The first run "hung": 400 s, no `.fem`, no result file, and a log containing only Wine's harmless
       `wine32` warning. Photographing the Xvfb screen -- `import -window root` against a persistent display --
       turned that silence into a sentence in about a minute, and the sentence was a **modal dialog**. A modal
       dialog is indistinguishable from a hang when the only evidence is a timeout.
     - Then the same `exit=124` appeared again on a run that had *worked*, which is the second half of the trap:
       with `-windowhide` and no `quit()` in the script the GUI stays resident, so the timeout kills a script that
       finished long before. The artefact is the signal, not the exit code. The runner now decides on
       `pm_axis_result.txt` existing, and every sweep row reports `no result file` as its own failure.
     - ``pending``

   * - C-73
     - C
     - The case-B criterion was written before measuring, which is the rule, but its **points** were chosen without
       asking what they can carry. The closed form `Bz(z) = (mu0*M/2)*[(z+L/2)/sqrt((z+L/2)^2+a^2) -
       (z-L/2)/sqrt((z-L/2)^2+a^2)]` is a difference of two disc terms, and its conditioning
       `cond = (|t1|+|t2|)/|t1-t2|` is computable from the geometry alone: 1.0 up to `z = 10 mm`, 2.9 at 15 mm,
       6.9 at 20 mm, 24.6 at 30 mm, 210 at 60 mm. Worse, `z = L/2 = 10 mm` is the magnet's end face, where `Bz`
       has a kink, so the smoothed interpolant converges slowly there by construction. Demanding monotone
       convergence at `cond = 210` is demanding that the metric resolve one part in 210 of the terms it is built
       from -- a property of the *criterion*, not of the solver.
     - Measured, in this order. At the script's own first settings (`R = 0.2 m`, magnet mesh 0.5 mm, air mesh
       4 mm) the 2 % half failed at **2.143 %** at `z = 30 mm`. Halving the air mesh to 2 mm brought the worst
       case to **1.142 %**, so the 2 % half passed -- and the refinement half failed anyway: across the 3x3 mesh
       matrix the `z = 30 mm` error ran `+10.75, +5.48, -0.49, +1.65, +2.14, +0.44, +3.35, +2.75, +2.38 %`, and
       at `z = 10 mm` it *grew* monotonically under magnet refinement (`-0.79, -0.08, +0.74 %`). An error that
       changes sign with the mesh is the metric talking, not the solver.
     - So the case is recorded as **failed against its first criterion**, and the replacement is narrower rather
       than looser: points with `cond <= 3` and at least one magnet radius away from the end face -- `z = 0, 2,
       5 mm` -- tolerance unchanged at 2 %, and monotonicity required only where the metric can express it.
       That set measures `-0.30, -0.23, -0.06 %` and tightens to `-0.22, -0.27, -0.13 %` as the mesh is refined.
       Its discriminating power is proved rather than assumed: the same geometry solved as `planar` returns
       `+40.4, +42.1, +54.1 %`, a red two hundred times deeper than the pass margin, so the pass is not a
       knife-edge and what the criterion actually tests is the axisymmetric formulation.
     - ``pending``

   * - A-74
     - A
     - `mo_smooth("on")` was added as the fix for a hypothesised defect -- a field read as a per-element constant
       -- and changed nothing at all, because in this build smoothing is **already on by default**. The control
       was not ineffective; it was already in effect, and the code was written against a default nobody had
       checked.
     - Twelve runs bracketing every mesh in the matrix came back identical to the last printed digit before and
       after the line was added. The hypothesis was then tested the other way round, which is what a control that
       does nothing should have provoked immediately: `mo_smooth("off")` returned the *same* `Bz` at ten points
       spanning 0.9 mm, to 17 significant digits, while `on` returned a smooth 2.4 % ramp across the same span.
       FEMM's `B` is therefore a per-element constant and the smoother is what makes the read usable -- but that
       was never the explanation for the errors, which belong to C-73.
     - None in the library: it closed nothing and re-opened nothing. It is the fifth instance this session of one
       class -- `-lua-var` never reaching the script, `meshsize` ignored while `automesh = 1`,
       `mi_setarcsegmentprop`'s reversed `maxsegdeg`, `mi_selectcircle` selecting nothing, and now `mo_smooth`
       already on. The rule that catches it is cheap and universal: print the thing you claim to have changed.
     - ``pending``

   * - A-75
     - A
     - The third model was trusted before asking what the vendor's data actually contains. pyleecan's `IPMSM_A` is a
       real machine -- a Toyota Prius 2004 interior-magnet rotor -- but the machine JSON carries a *stripped*
       material: `mag.BH_curve` points at an empty `ImportMatrix`, so the lamination arrives linear
       (`mur_lin = 2500`) with no saturation anywhere. On this geometry that is not a detail. The magnet's outer
       edge sits at `r = 77.05 mm` and the rotor surface at `r = 80.20 mm`, so every V-pole is closed by a
       **3.15 mm solid iron bridge**, and a linear bridge has a reluctance two to three orders of magnitude below
       the 0.75 mm air gap. Almost all of the magnet flux therefore short-circuits through the bridge and the gap
       sees leakage. A real IPM works *because* that bridge saturates; a linear model never lets it.
     - Measured, and the measurement is what made it readable. The air-gap field came out at `abs(B_r)` peak
       **0.0692 T** against roughly 1 T from the series magnetic circuit (`Hc * 6.5 mm = 5864 A` across a 0.75 mm
       gap) -- *but* the pattern was right: 8 sign changes around the circumference and an order-4 fundamental
       carrying 0.0677 T of it. Right pattern with a 15x low magnitude is a short circuit, not a wrong direction,
       which is why the first two hypotheses (the V's two limbs magnetised diverging versus converging) were both
       wrong. Back-solving the series circuit from the measured value gives an equivalent air path of **117 mm**,
       which is the radius of the machine: the magnets were sitting in a magnetic open circuit.
     - The fix is in the data and is recorded as such. pyleecan ships the missing curve in its own library,
       `Data/Material/M400-50A.json`: 29 points from (0 A/m, 0 T) to (10000 A/m, 1.761 T). It is attached with
       `mi_addbhpoint("iron", B, H)` -- vendor data, not an invented curve. The generator may no longer be silent
       about its absence: with no curve it writes a warning into the model saying everything downstream is a
       modelling artefact. What this does *not* change is that no `Ld`/`Lq`/`psi_f` value from this geometry has
       been claimed, and none will be until the saturating solve is affordable (A-76).
     - ``pending``

   * - A-76
     - A
     - Two mechanisms were mistaken for slow physics. (1) **Smart mesh**: the `.fem` carries
       `[DoSmartMesh] = 1`, and with it on FEMM ignores the per-block `meshsize` this generator writes -- scaling
       every block mesh by 6 moved the answer file from 1,649,290 to 1,614,589 lines, i.e. 2 %. Setting the flag to
       0 collapsed it to **148,679 lines**, an 11x difference, and that is the lever that had been missing while
       four runs hit their timeout. (2) **`fkn.exe` is FEMM's solver process**, not `csolv.exe`: `timeout N wine
       ...` kills only the wine wrapper, so a killed run leaves `fkn` burning a core and holding the `.ans` -- one
       was found with 62 minutes of CPU time -- which made every later run look slower than the one before it.
     - Both surfaced as the same symptom, a run that neither finishes nor fails, and neither was visible from the
       wrapper's exit code. The mesh one needed the answer file's size and line count to expose it (76.99 MB /
       1.65 M lines at scale 1 versus 75.35 MB / 1.61 M at scale 6 -- the measurement that refutes "the mesh got
       coarser"), and the process one needed `ps aux | grep fkn`.
     - Cleanup is now `pkill -9 -f 'fkn.exe|femm.exe|csolv|triangle|wineserver'`, and the generator no longer
       trusts `meshsize` for cost control. Also recorded: the gap mesh must stay *finer* than the gap (0.25 mm in a
       0.75 mm gap). Scaling it up to the gap width makes the mesher try to fit 30-degree triangles into a sliver
       and takes longer, not shorter -- measured 6m22 at scale 1 against more than 13 minutes at scale 3. What is
       *not* yet affordable, and is recorded rather than worked around: with the mesh at 148,679 lines and the
       vendor B-H curve attached, the same pipeline that finishes a linear solve in 6m22 does not complete inside
       30 minutes, and the work is in the detached `fkn` process, so the wrapper's own CPU accounting reads 0.4 s
       while it waits. So the saturating case is measured to be out of budget at this resolution, not shown to be
       wrong.
     - ``pending``

   * - F-77
     - F
     - Four defects in the new C-step tooling, each of which returned a plausible number instead of failing.
       (1) `comp_connection_mat` already multiplies by `Ntcoil`, so multiplying again put 9x the ampere-turns in
       every slot (plus or minus 81 where it should be plus or minus 9); caught only because the printed turn count
       was wrong. (2) pyleecan returns a lamination as ONE surface whose `get_lines()` concatenates two independent
       closed loops (the outside circle, then the inside circle with the slot notches); walked as a single chain
       that invents a phantom edge across the lamination, and FEMM then says nothing more specific than "Material
       properties have not been defined for all regions". (3) `mo_getcircuitproperties(name)` returns a *table* in
       Lua, so destructuring it into three values left the flux linkage nil and the column silently empty.
       (4) `mo_getpointvalues` on a planar problem returns `(A, Bx, By)`, not `(A, B_r, B_t)`, so the first radial
       field numbers were the x-component misread as the radial one.
     - (2) was localised by rasterising the segments and flood-filling the free space: every connected region is a
       FEMM region, so listing the ones that contain no block label gives coordinates instead of a riddle. It named
       two regions (52 mm^2 and 11 mm^2 at theta about -2.3 deg) sitting at the loop's seam, which is what pointed
       at the chain split. (1) and (3) were caught by printing the number that was supposed to change, and (4) while
       writing the downstream analysis, before any number was reported.
     - All four are fixed, and two are now assertions rather than hopes: the export asserts that each winding
       matrix entry equals plus or minus `Ntcoil`, and `find_unlabelled.py` runs as a preflight in the sweep driver
       so that no solve is spent on a model that will stop at a modal dialog. Files:
       `workspace/tools/geom/{export_geom,gen_femm,find_unlabelled,analyze_ipmsm,run_ipmsm}.py`.
     - ``pending``

   * - A-78
     - A
     - A FEMM region with **no block label is not meshed at all**, and the only symptom is that queries inside it
       return nothing. I had assumed the opposite -- that an unlabelled region is meshed as air by default --
       because an earlier full-machine model had emitted only a warning ("Material properties have not been defined
       for all regions") and then solved anyway. In the one-pole-pitch sector model the solution file covered
       `r = 63 .. 115 mm` and nothing else: the magnets and the copper were meshed, the stator iron, the rotor iron
       and the air gap were not, so 43 of 45 gap probes came back as `nan` and the field inside the rotor read
       2e-8 T. The reason the labels were in the wrong places is the second half of the row: the sector is built by
       clipping each polygon to the wedge, and clipping **invalidates pyleecan's `point_ref`** -- the original
       centroid can end up outside the clipped region or exactly on the wedge edge. A label must be recomputed on
       the clipped polygon, and for a chain that is only a boundary (a lamination's outside circle) the clipped
       "closed loop" has a bogus interior that will happily swallow any point.
     - The measurement that exposed it was the extent of the mesh, not the field: reading the `[Solution]` block of
       the `.ans` gives the node coordinates, and `r: 63.07 .. 115.24 mm` is a sentence, whereas `nan` is a
       question. Before that, three hypotheses were tried and all were wrong (probe radius off a boundary,
       antiperiodic pairing, label positions), each costing a run.
     - The sector itself is worth keeping: 557 points instead of 4294, and a solve in **5 seconds** against 6m22
       for the full machine. But it is only usable once region-level polygons replace the chain-level clipping;
       that is the next piece of work, and until then the sector is a worked example of a model that solves
       quickly and answers nothing. The full 360-degree model is what produced every number in A-75.
     - ``pending``

   * - A-79
     - A
     - Three cost facts about this FEMM build, measured while trying to make the saturating case affordable, and
       each one refuting a plausible guess. (1) `[DoSmartMesh] = 1` makes FEMM ignore the per-block `meshsize`
       entirely: scaling every block mesh by 6 moved the answer file from 1,649,290 to 1,614,589 lines (2 %).
       Setting the flag to 0 gives **207,047 lines**, an 8x cut -- but the flag is not reachable from the Lua API,
       so it can only be set by rewriting the saved `.fem`. (2) A script that starts with `open("model.fem")` and
       then `mi_analyze()` will write a complete `.ans` and then **never finish**, while the same geometry built by
       `newdocument` + `mi_build...` in Lua solves and post-processes normally. Three attempts at the `open` path
       ended at 6m48 (killed), 30 minutes and 3 minutes without a result, all with `femm.exe` idle at 0.4 % CPU.
       (3) The mid-gap circle this generator added was innocent: removing it changed the mesh by 6 % and the answer
       by **0.0691 T against 0.0692 T**, i.e. not at all -- which incidentally is the mesh-independence check the
       number needed.
     - (1) surfaced from comparing answer-file sizes across settings, and (2) from `ps` showing the solver gone
       while the script still waited. (3) was run precisely to test whether my own addition was the driver, because
       it was the only part of the model that was mine rather than pyleecan's.
     - Consequence recorded rather than worked around: the saturating case now runs with an hour's budget instead of
       a quarter of it, and the sector would be the fix if region-level clipping existed (A-78). What has *not*
       changed is the claim: the linear model's 0.0692 T stands as measured, and the saturating number is still
       missing.
     - ``pending``

   * - F-80
     - F
     - Two more defects in the C-step tooling, both of the same shape -- a post-processing call that fails silently
       or loudly at the wrong moment. (1) `mo_getpointvalues` returns **nothing** for a point that lies exactly on a
       region boundary, and my generator's guard was written so that a single such point (the first probe sat on the
       antiperiodic edge, and after that on the mid-gap circle) turned *every* row into `nan`: 45 of 45. (2)
       `mo_getcircuitproperties` returns a **number**, not a table; the shipped Octave wrapper returns an array, so
       indexing the Lua result (`z[1]`) produced `attempt to index global z (a number value)` and killed a run
       after it had already paid for its solve.
     - (1) was found by adding three diagnostic points that must lie inside material (origin, rotor, stator) and
       printing `nil` explicitly instead of letting `write()` throw; that immediately separated "postprocessor has no
       solution" from "this point is bad". (2) was found by the modal dialog naming the exact line.
     - Both are fixed in a way that cannot regress quietly: the probe loop writes `nan` for a missing pair so the
       parser can count them, and the circuit extraction probes `type(a) == "table"` at run time and reports which
       convention it found instead of assuming either. The probe grid is also offset by half a step so no probe sits
       on an edge by construction.
     - ``pending``

   * - F-81
     - F
     - The generator attached the iron's B-H curve **before creating the material**: `mi_addbhpoint("iron", ...)`
       sat at lines 7-35 of the Lua while `mi_addmaterial("iron", ...)` came later, and FEMM discards
       `mi_addbhpoint` on a name that does not exist yet **without any message**. Every "saturating" run in this
       session therefore solved the *linear* problem, and the file that was supposed to test whether bridge
       saturation restores the gap field to 0.6-1.1 T was never actually nonlinear.
     - It was caught only because two runs were diffed against each other: the "nonlinear" result file was
       **bit-identical for its first 20 lines** to the linear one, including `diag rotor -0.008935911975379105`.
       Reading the new file alone would have looked completely normal. Confirmed from the `.fem` afterwards:
       `<BlockName> = "iron"` with `<BHPoints> = 0`, against 29 points asked for.
     - The emission order is fixed and the reason is written into the generator next to the code, because this is
       the same defect class as the rest of this session -- a control that changes nothing reads exactly like a
       converged answer. The general rule it adds: **after adding any material property, read it back out of the
       saved `.fem`**; a property that is absent there was never applied, no matter what the script said.
     - ``pending``

   * - A-82
     - A
     - The saturating solve landed, and **all three criteria written before it ran were met**: the gap `abs(B_r)`
       peak went **0.0692 T to 0.7873 T** with an order-4 fundamental of **0.8503 T** against a pre-registered
       **0.6-1.1 T**; every harmonic other than order 4 is below **1.3 %** with **8** sign changes around the
       circumference, so the 8-pole pattern never changed; and the lumped winding route gives
       **psi_f = 0.3858 Wb-turns** against a pre-registered **0.35-0.40**, with `Kt = 2.315 N*m/A` against a
       predicted 2.3. The `B_g1 = 0.85 T` assumed when psi_f was predicted came out **0.8503 T**.
     - This is the first number in the whole leg that was **registered before it was measured and then met**, so it
       is a stronger statement than the two closed-form checks: those showed the solver reproduces an answer that
       already existed, this one shows a model assumption was quantitatively wrong and that correcting it lands where
       the physics said it would. The wrong version is preserved rather than deleted -- 0.0720 T with the flux
       short-circuited inside the rotor is what a linear bridge assumption produces, and the ratio **11.8x** between
       the two is the size of that modelling error, which is precisely the quantity this layer exists to report.
     - **The scope of the psi_f number has to be stated or it becomes a different claim**: it comes from the measured
       gap fundamental fed through the winding formula, *not* from FEMM's flux linkage. The prediction named the
       flux-linkage fundamental, which needs a 45-degree electrical sweep, and a single non-linear angle costs
       44 minutes (linear 6m22), so that sweep is around nine hours and is **not done**. The difference between the
       two routes is the model-assumption error, i.e. a result in itself -- the sine assumption for the gap field is
       not being tuned away.
     - ``pending``

   * - F-83
     - F
     - The first loaded solve returned `exit=0` and a complete result file, and **the excitation had done
       nothing**: with `ia,ib,ic = 100,-50,-50` A and 9 turns in every one of the 48 slots -- a nominal 43,200
       ampere-turns against the magnets' 5,864 -- the air-gap fundamental moved by **0.0000 T** (peak by
       0.007 T), the stator-side `diag` points by under 0.2 %, and the phase flux-linkage readout was
       **6.5e-6 Wb-turns**, i.e. `L = 65 nH` where this winding cannot be below roughly a millihenry.
     - The gap field is the validated instrument here, so the conclusion does not depend on the circuit
       readout: a 43,200 A-turns excitation producing a zero change in the order-4 component is not a
       converged answer, it is an unapplied one. The near-miss matters more than the defect -- every number
       in that file is formatted exactly like the no-load file that passed, and `Lq = 6.2e-8 H` would have
       been computable, plausible-looking, and wrong by five orders of magnitude.
     - The block labels were then read back out of the saved `.fem` against an official wound example
       (`DoubleSidedLIM.fem`, whose label line shows `Turns = -10` in the same position), which confirmed the
       layout is `InCircuit, MagDir, Group, Turns` and that the model does carry `Turns = 9` with the correct
       per-slot phase pattern -- so the defect is **not** in the winding definition and the next step is a
       discriminator, not a guess: the same geometry solved **linear**, where `abs(B)` must scale exactly with
       the applied ampere-turns, separates "no current" from "current reduced by a factor" from "current right
       and the estimate wrong".
     - ``pending``

   * - F-84
     - F
     - The cause of F-83, found by measuring rather than reading: the **sign of a block's `turns` is what carries
       current direction, and the `turns` argument of `mi_setblockprop` is inert in this FEMM build**. On the
       already-validated coaxial model, `turns = 1` and `turns = 10` return a **bit-identical** flux linkage
       (`3.714904718532582e-007`), so the block stored `Turns = +/-9` in the `.fem` and the solver never saw it.
       Every `-9` slot therefore carried the same direction as the `+9` ones.
     - The winding's period is 12 slots and that was written into the file correctly; with the signs deleted it
       became 6 slots, and the measured signature says exactly that: the current-induced field was dominated by
       spatial **order 8** (0.0082 T) and order 16 (0.0024 T) while **order 4 was 4e-6 T** -- an MMF with eight
       pole pairs from a winding that has four. The magnitude was low by roughly fifty times for the same reason
       (partial cancellation), which is why the first read was "the current did nothing" rather than "the current
       is the wrong shape".
     - Fixed by moving direction onto **signed circuits** (`A+`, `A-`, ... `C-`) with `turns` fixed at 1 and the
       ampere-turns carried by the circuit current; the readout then has to be scaled by N to be in Wb-turns, which
       is verified against the no-load answer rather than assumed. The two defects are the same family as F-81 and
       as the `automesh`/`-lua-var`/`mi_setarcsegmentprop` mistakes in C-73: **an argument the API accepts and
       then ignores produces a model that looks built**. What catches it is not reading the script but comparing a
       run against a case whose answer is already known.
     - ``pending``

   * - A-85
     - A
     - The signed-circuit fix for F-84 is verified, and the criterion was written before the run: the
       current-induced gap field must be dominated by **spatial order 4**. It is -- **order 4 = 0.3125 T**
       against order 20 = 0.0178 T (eighteen times smaller), with order 8 gone entirely, and the change's rms
       went from 0.0063 T to **0.2237 T** (35x). The order-8 signature that diagnosed F-84 disappeared exactly
       as predicted once the `-9` slots were allowed to carry the opposite direction.
     - This is the first loaded result in the leg that is physically recognisable: 900 ampere-turns per slot now
       produce a four-pole-pair armature reaction for a winding whose pattern repeats every 12 slots, and the
       short-circuit-proof instrument (the gap field) is what says so.
     - The circuit flux-linkage readout is **still not usable and no longer needed**: 4.25e-4 at 900 A is some
       300x below the flux the measured gap field implies, so psi will be taken from the field and the winding
       function instead. The readout stays in the file anyway -- a number wrong by a constant factor is evidence
       about the instrument, not noise to drop. Two self-inflicted detours cost a turn each on the way: the fix
       first named the saved document "-" because a loop variable shadowed `--tag`, then hung for seven minutes
       on a modal dialog because post-processing still probed circuits `A`/`B`/`C` that no longer existed. Both
       were caught by reading the **generated** file rather than the patch, which is the F-81 rule one layer up.
     - ``pending``

   * - F-86
     - F
     - **A-82's psi_f cannot be counted as a verified number, and the reason is that the prediction and the
       measurement used the same formula.** The pre-registered band 0.35-0.40 was computed as
       `N_ph*k_w1*B_g1*L1*2R/p` with `B_g1 = 0.85 T`; the measured 0.3858 is that same expression fed the measured
       `B_g1 = 0.8503 T`. So it confirms that the field level was predicted correctly, not that psi_f was measured.
       Of A-82's three criteria the first two -- the gap peak rising into 0.6-1.1 T and the 8-pole pattern
       surviving -- are genuinely independent and stand; the psi_f line does not.
     - The independent route disagrees and is not yet resolved. Taking psi from the definition,
       `psi = L1 * sum_k N_k * A_z(theta_k)` with `A_z` integrated from the measured gap field and the slots read
       out of the `.fem` itself, gives **0.188** on the same no-load field where the lumped route gives 0.386.
       Feeding a synthetic pure sinusoid through the same two routes isolates the cause: **0.0998 against 0.3858,
       a factor of 3.87**, so it is not the field's harmonic content and not the integration. The circuit
       readout is not a tie-breaker either -- it is about 200x below what the measured field implies.
     - Deliberately not resolving it by picking the number that agrees with the pre-registered band. The way out
       is a case whose flux linkage can be computed by hand and compared against both routes, plus taking `A_z`
       from FEMM rather than integrating `B_r`; until that is done, **no psi-derived quantity (Ld, Lq, Kt) is
       quoted**. The loaded runs finished with `exit=0` and are kept, because the fields in them are good.
     - ``pending``

   * - A-87
     - A
     - **F-86's factor of 3.87 is resolved, and the answer is that the psi_f I reported was wrong by 2.05x.**
       Two errors sat in the same helper: `N_ph` was taken as `sum(abs(turns))` over the slots, which counts
       every turn **twice** because each turn passes through two slots (16 slots x 9 turns is 8 coils, i.e.
       **N_ph = 72**, not 144); and `k_w1` came from the machine JSON's `coil_pitch = 5`, which implies a chord
       factor of sin(75 deg) while the winding matrix actually spans **6 slots, a full pitch**, so `k_p = 1`.
       With both fixed the lumped route gives **psi_f = 0.18826 Wb-turns** against the definition-level sum's
       **0.188** -- **0.1 % apart**, where before they were 3.87x apart.
     - The identity that settles it needs neither coil pitch nor winding factor:
       `abs(sum_k N_k e^{j p theta_k}) == 2 * N_ph * k_w1`. Evaluated on the layout at 1 turn per slot it returns
       **15.455** while `2*8*0.966` is **15.456**, so N_ph = 72 and k_w1 = 0.966 are read out of the model
       instead of out of a JSON field that disagrees with it. That is what turned a factor into a number.
     - **The pre-registered band 0.35-0.40 is therefore falsified, not met**: 0.188 lies outside it, and it was
       only ever met because the prediction and the measurement shared the doubled turn count (F-86 said exactly
       that in advance). `Kt` moves from 2.315 to **1.13 N*m/A**. A-82's other two criteria are untouched --
       they are about the field, and the field is fine: the peak rose into 0.6-1.1 T and the 8-pole pattern
       survived. Standing rule reinforced: a prediction that shares a formula with its measurement is not a
       prediction, and the only way to find that out is a second route sharing nothing but the field.
     - ``pending``

   * - A-88
     - A
     - **The geometric route now produces axis inductances**: two loaded solves at 100 A with the current vector
       rotated 90 degrees apart (rotor and mesh unchanged) give, from the symmetric 2x2 fit,
       **Ldd = 0.193 mH, Lqq = 0.648 mH, Ldq = -0.004 mH** -- the cross term is **2.0 %** of Ldd, and
       **Lq/Ld = 3.36**. Both runs returned `exit=0`; psi comes from the field and the winding function, not from
       the circuit readout.
     - The interesting part is that the **d-axis stopped being an assumed quantity and became a testable one**.
       Three candidate definitions existed: the no-load flux vector (**148.47 deg**), the phase of the gap field's
       order-4 harmonic (**-90.00 deg**), and the angle that zeroes the perpendicular component of one run's
       delta-psi (**143.37 deg**). Solving the 2x2 fit at each separates them without appealing to any of them:
       cross term **2.0 % at 148.47**, **22.4 % at 143.37**, **38.2 % at -90.00**. So the flux-vector axis is
       the consistent one, and the "minimise the perpendicular component" scan -- which *defines* its own answer
       -- is circular and is recorded as such rather than used.
     - **What this is not**: the fit residual is **7.0 %**, so the linear symmetric 2x2 does not fully describe the
       two points. The likely cause is stated rather than tuned away -- `delta-psi` subtracts the no-load flux as
       if the magnet's operating point were unchanged, and saturation violates exactly that. So the numbers are
       recorded with the 7 % attached, **no claim in README moves** (this is `IPMSM_A`, not this kernel's
       machine), and a third operating point is what would separate cross-saturation from the subtraction error.
     - ``pending``

   * - F-89
     - F
     - A relaunch of the q-axis differential case returned **`exit=0` and produced no artefacts at all**. The
       cause was outside the model: the machine had rebooted, `/tmp` was cleared, and **Xvfb on `:82` was never
       restarted**, so wine logged `nodrv_CreateWindow ... Make sure your X server is running and that $DISPLAY
       is set correctly` and exited successfully without solving. The first attempt had also been killed by that
       same reboot, but that one left a `.fem` behind, so the two failures looked different when neither had run.
     - The judgement rule that caught it, and that should have been used from the start: **decide from the
       artefact, not from the exit code and not from the job/status file**. `nohup` survives SIGHUP and does not
       survive a reboot; `/tmp` does not survive one either, so a missing status file says nothing. A `.ans` with
       a fresh timestamp is the only thing that says the solver ran.
     - Same family as F-81/F-83/F-84 and the `automesh`/`-lua-var` cases in C-73 -- an accepted invocation that
       does nothing still exits 0 -- with one difference worth naming: those were inside one process, this one is
       the harness around it, so the guard belongs in the launcher. The session's own raw `nohup wine` invocations
       skipped the `preflight()` that `run_ipmsm.py` already has; that check should be the only way to start a
       solve, or the next reboot will cost another day.
     - ``pending``

   * - F-90
     - F
     - Built the cheap pre-flight the session had been missing (`workspace/tools/geom/smoke.py`, three tiers),
       measured it, and **proved it can fail**: with Xvfb stopped it returns red in **0.1 s** naming
       `Xvfb: no Xvfb process`, and with the display back it is green in **4.5 s**. Tiers 0+1+2 cost **3.1 s**
       together, versus the 44 minutes a real solve costs.
     - The tier split is forced, not chosen: **a full-machine solve cannot be made cheap** -- a *linear* full
       solve is 6m22 measured, and scaling the gap mesh up makes the mesher *slower* (it has to fit 30-degree
       triangles into a 0.75 mm sliver), so my first attempt at a "coarse everything" smoke ran into the same
       120 s timeout it was supposed to prevent. The answer is three gates: 0 preconditions (<0.1 s), 1 solver
       path against the coaxial closed form (2.9 s), 2 static assertions on the generated script (0.3 s), and
       only then 3 a coarse solve with its own budget.
     - **Regression found while doing it**: the sector model, which used to build and solve in 5 s (A-78), now
       hangs -- 150 s with 2.4 s of CPU and no artefacts. The prime suspect is the six-signed-circuit change
       (F-84): a 45-degree wedge may leave one of the six circuits with no blocks, which FEMM reports as a
       modal dialog and `-windowhide` makes invisible. Not investigated yet; recorded so the sector path is not
       believed to be one flag away.
     - ``pending``

   * - A-91
     - A
     - **Ld and Lq are measured, and by the route that needs no assumption about the magnet.** The differential
       inductance between two loaded points cancels the magnet flux in the subtraction, so nothing depends on
       "the magnet's operating point is unchanged" -- which saturation violates and which was the suspect behind
       the earlier residual. Result: **Ld(50->100 A) = 0.1899 mH, Lq(50->100 A) = 0.6406 mH, Lq/Ld = 3.37**,
       with the no-load-subtracted values (0.1932 / 0.6493) agreeing to **1.7 % and 1.4 %** -- both inside the
       2 % criterion written before the runs.
     - The agreement is the actual content of this row, not the numbers: it says the 7 % residual in A-88 was
       **not** caused by the magnet-flux subtraction, because removing that subtraction entirely leaves the
       same values. The joint symmetric 2x2 over all four points gives `Ldq = -0.0041 mH`, i.e. **2.1 %** of
       Ldd, and a residual of **1.37e-03 Wb-turns** -- which is twelve times the d-axis-only residual, so what
       the constant-`Ldq` model cannot describe is the cross structure, not either axis. Stated as a limit of the
       extraction, not tuned away.
     - Anchors: published figures for this machine (Toyota Prius 2004, 48 slots, 8 poles) are `Ld` about
       0.24 mH and `Lq` about 0.47 mH at rated current. Ours is lower on d and higher on q at 100 A, which is
       what a different operating point would do -- so this is an **anchor, not a validation**, and no claim in
       README moves: this is `IPMSM_A`, not this kernel's machine.
     - ``pending``

   * - A-92
     - A
     - **Four-point sweep done (2 h 56 min total, four nonlinear solves at 50/100/150/200 A on both axes):
       both inductances fall monotonically, which was criterion 2 written before the runs.** Differential
       **Ld = 0.1899 / 0.1772 / 0.1645 mH** and **Lq = 0.6406 / 0.6209 / 0.5890 mH** over 50-to-100, 100-to-150
       and 150-to-200 A; the no-load-subtracted secants fall too (Ld 0.1955/0.1927/0.1875/0.1818, Lq
       0.6552/0.6479/0.6389/0.6264). End to end Ld drops **13.4 %** and Lq **8.1 %**.
     - **The non-obvious result is that the saliency ratio rises with load, not falls: Lq/Ld goes 3.373 to
       3.581, +6.1 %.** The prediction I would have written (Lq sags faster than Ld, so MTPA gain shrinks with
       current) is **wrong for this machine**: Ld saturates faster, so the machine becomes *more* salient under
       load. That matters for C8 because `MtpaReference` currently carries a single `Lq/Ld = 2`; the direction
       of its error with load is now known rather than assumed.
     - **The number that matters for a torque map is the cost of a single-point parameter set**: using the
       50-100 A values across the whole 4x range is **13.4 % high on Ld and 8.1 % high on Lq at 200 A**.
       Honest scope on criterion 1: the "two routes agree within 2 %" check was defined at 100 A, where it held
       at 1.7 %/1.4 %. At 200 A the secant and the chord *must* diverge because saturation is real -- that
       divergence is the measurement, not a criterion failure, and it is stated rather than re-badged. The
       global constant-`Ldq` fit also degrades with range (residual 1.37e-3 to **2.99e-3** Wb-turns from four to
       eight points), so `Ldq/Ldd` about 1.5 % is a **bound over the range, not a per-current value**.
     - ``pending``

   * - F-93
     - F
     - **The entire evidence chain had no external truth in it.** Every number from A-78 through A-92 came
       out of one pipeline: the same geometry, the same B-H curve, the same magnet grade, the same mesher.
       A-87 had already proved that two such routes agreeing to 0.1 % is not evidence of correctness but
       of a shared assumption -- and nothing in the plan's third-layer leg was positioned to detect that.
     - Surfaced while answering a question about whether *public* motor data exists, on the theory that the
       answer would be no and the geometry leg would be closed. It is yes: the U.S. Department of Energy had
       this exact motor torn down and instrumented at Oak Ridge National Laboratory, and both reports are
       public with stable DOIs -- ORNL/TM-2006/423 (`doi:10.2172/947571`, measured electrical and thermal
       characteristics) and ORNL/TM-2004/137 (`doi:10.2172/885676`, design and manufacturing assessment).
       `IPMSM_A` *is* the 2004 Toyota Prius traction motor, so the measurement is of the same physical
       machine that the geometry describes, and no datasheet -- and no domain knowledge -- is required from
       anyone to use it.
     - The consequence is that the distinction the third layer was built to make was never being made: the
       project could say "our `psi_f` is 0.18826 Wb, confirmed twice" and had no way to say whether 0.18826
       was *right*. Blast radius is the honesty of every earlier row, not any one number. No claim row moves
       on this row alone -- it is the defect, not the repair; the repair is A-93.
     - ``pending``

   * - A-93
     - A
     - **First comparison against third-party measurement in this project, and it says our `psi_f` is
       8.9 % high.** ORNL/TM-2006/423 Table 6.2 states the measured motor voltage constant as **1.33
       Vrms/Hz** (terminal, line-to-line), which converts to **`psi_f` = 0.17283 Wb** for a four-pole-pair
       machine. Ours is 0.18826 Wb, i.e. **+8.93 %**.
     - **The source was made trustworthy before it was used.** Four independent checks on the transcription:
       `f_e/(rpm/60)` over the twelve Table 3.4 points lands in [3.942, 4.040] against the known 4 pole
       pairs; the least-squares slope through the origin is 1.3481 V/Hz against the report's own stated 1.33
       (+1.36 %); the fit predicts 539.2 Vrms at 400 Hz against Table 6.2's 540 Vrms at 6000 rpm (+0.14 %);
       and Vpeak/Vrms over the twelve points lands in [1.540, 1.786], every one of them above ``sqrt(2)``,
       so the rms and peak columns are demonstrably not confused. All four are asserted in
       `workspace/tools/geom/oracle_ornl.py`, which refuses to print a comparison if any of them fails, and
       `--selftest` detunes `psi_f` by 1.30x and 0.70x and requires both to go red.
     - **The ``+-10 %`` band was chosen after the number was seen, and is recorded as such.** It is a
       statement of measured accuracy, not a pre-registered prediction -- the one prediction that *was*
       pre-registered (0.35 to 0.40 Wb) was falsified in A-87 and stays falsified. Against the band the
       comparison passes at 10 % and fails at 5 %. The confounders named here are the ones that no amount of
       further meshing or further angle points can remove: a 2D solve has no end-region leakage and so reads
       high, the geometry is a reconstruction and not Toyota's drawing, the B-H is an assumed M400-50A grade,
       and the magnet is `Br` = 1.1906 T from the vendor `Hc`. **That is the reason deepening the FEMM leg
       further now has low marginal value**: the residual is dominated by terms a finer discretisation
       cannot reach, so the next useful move is upstream and downstream of the field solve, not inside it.
       No claim row is promoted: this is `IPMSM_A`, README is untouched, and what changed is that this
       project can now state an error bar instead of an agreement between two of its own calculations.
     - ``pending``

   * - F-94
     - F
     - **A "5 % band" that read like an accuracy statement about a motor and was in fact an accuracy
       statement about arithmetic.** `torque_map` converts a nameplate it is *handed*; nothing in the claim
       said so, and the L2 evidence could not have discovered it -- the mechanics comparison perturbs the
       map while the SIL plant keeps the true constants, so a nameplate error is structurally invisible to
       that test. The same blindness sits inside the existing falsifier: scaling the map's gain by 0.9 moves
       the measurement to -9.994 %, which measures the code against a plant that is right, not the map
       against a machine that might not be.
     - Surfaced by writing the L3 sensitivity leg, which is required to use error bars the project has
       **measured** rather than invented ones. There are now two: the externally measured `psi_f` deviation
       of **+8.93 %** (A-93, against the ORNL bench data) and the **13.4 % / 8.1 %** `Ld`/`Lq` saturation
       between a 50-100 A nameplate and a 200 A operating point (A-92). Propagating the first one alone moves
       the torque estimate **+9.749 %** -- above the map's own 5 % band.
     - Blast radius is the claim text only; no implementation change. It also explains why the earlier
       numbers looked so good: +0.006 % agreement is a statement that the code computes what it says, and
       this project had been reporting it in a column where readers expect a statement about a machine. The
       repair is A-95.
     - ``pending``

   * - A-95
     - A
     - **`torque_map`'s L3 evidence is complete, and the level is held at L2 anyway, because the
       repository's own suite is red (F-96).** The evidence is at least as much about what the level does
       *not* buy as about what it does. Sensitivity on measured bars: `psi_f` +8.93 % moves the estimate
       **+9.749 %** (amplified, because the reluctance term subtracts at that working point -- a `kt` error
       cannot be damped there), `Ld` -13.4 % moves it **-1.229 %**, `Lq` -8.1 % moves it **+1.486 %**, and
       all three together **+10.006 %**.
     - Phase-variable witness, reused from the `test_abc_plant.cpp` six-point recipe: the co-energy model,
       which never forms a d-q quantity and never uses `kt`, agrees with `torque_estimate` to **0.001 %** at
       six `id = 0` points (0/200/800 rad/s electrical x 1/2 A) against a 2 % criterion. A falsifier runs the
       same comparison against a map whose gain is 10 % low and reads **0.900000**, so the witness is proved
       able to reject an error of the size it claims to detect.
     - **The blind spot is measured rather than asserted away.** `AbcPlant` is non-salient by construction
       (its own documentation explains why a salient abc model would reintroduce the transform being
       checked), so it has no reluctance term to disagree with: at `id = +1 A` the two readings differ by
       **-8.40 %** of the torque, at `id = -1 A` by **+8.40 %**, and that difference is exactly the map's
       saliency term and nothing else (asserted to 1e-3 relative). So the saliency half of this claim rests
       on the sensitivity table alone. README says that in the row rather than presenting the cross-check as
       complete -- and the same convention already governs `reference.hpp`, whose `MtpaReference` saliency
       claim is likewise L3 through sensitivity, not through a second derivation. No code changed; what was
       added is evidence and text, and the 5 % band is now explicitly a statement about the code path rather
       than about a physical motor. **The promotion itself is one edit away and is deliberately not made**:
       README says L2 and says why, so that the level and the build agree.
     - ``pending``

   * - F-96
     - F
     - **The working tree could not be built**, and `main` could: `conan create` fails at its test step on
       the pending changes (**15 of 599 cases fail**), so the tree never reached the cache as a tested
       artefact. (The coverage figure first written on this row -- 91.43 % against a 94.0 % floor -- was
       **wrong and is retracted**: it came from running `coverage_gate.py` against the export directory a
       failed build leaves behind, not from the recipe's own measurement. The recipe reports 94.22 % all /
       95.74 % core, both above their floors. See F-99, which exists because of that mistake.)
     - Surfaced by running the full build as the last step of A-95 -- the promotion was the reason a green
       suite mattered, and there was none. The largest cluster is `Protection.*` (13 cases), and it is a real
       logic failure, not instrumentation: `EveryFaultIsClassifiedAsItselfAndNothingElse` prints "5" against
       `EXPECT_EQ(8, correct)`, i.e. three of eight rows of the fault-classification matrix are wrong. A
       boolean comparison is build-mode independent, so this fails in every configuration, coverage or not.
       `c_api_validation` and `python_bindings` fail as well; those two are the reason `src/api/foc_api.c`
       sits at 77.65 % and `bridge.cpp` at 82.08 %. Not yet diagnosed.
     - Blast radius is every claim in README: the suite that is supposed to back them did not pass, and
       nothing said so. Two lessons came with it. First, **`ctest -N` overwrites
       `Testing/Temporary/LastTest.log`**: the diagnostic used to read the failure list destroyed the record
       of what had failed, and the suite had to be re-run to recover it -- a read-only question answered by
       a write. Second, the failures are all in uncommitted work (`protection.hpp`, `foc_api.c` are both
       modified against HEAD), which is why no earlier row caught them: nothing had run the suite.
     - ``pending``

   * - F-97
     - F
     - **The thermal budget accumulated `i^2 / dt` instead of `i^2 * dt`, and that line existed only in the
       working tree.** `q` is an energy in A^2*s, so the increment over one period is `i^2 * dt`; dividing by
       `dt` made a machine running at its own nameplate spend **180000 A^2*s per 50 us period** against a
       **4.5 A^2*s** budget, i.e. the protection tripped I2t on the first healthy control period. This is
       one character and it accounted for **14 of the 15** failing cases in F-96: the 13 `Protection.*`
       cases and the three `c_api_validation` failures, which assert `FOC_FAULT_NONE` and `output_allowed`
       on a healthy step.
     - Surfaced by running the suite, not by reading the line. The injection matrix printed the diagnosis in
       one screen: `injected None -> I2t WRONG` (a healthy input producing a fault) and
       `injected Stall -> I2t WRONG` (a *different* mechanism's code winning the ranking because I2t also
       tripped). That second row is the instructive one -- the observable symptom was two wrong cells out of
       eight, but only one mechanism was broken, and the ranking hid it. A chain of `else if`s would have
       shown "Stall does not work"; the independent-flag matrix showed "I2t fires when nothing is wrong",
       which is the actual defect.
     - **The criterion already existed and was never executed against.** The test's own comment states the
       equilibrium of this model is `i_cont^2 * tau` = 3^2 * 0.5 = 4.5 A^2*s, and that budget is what the
       config sets. Only `i^2 * dt` reproduces that identity; the dimensional check is one line. So this row
       is not a missing test but an unrun one, and the blast radius in a real product is exactly the worst
       one available: a protection that trips on its rated current protects nothing and stops the machine.
       **`HEAD` already had `i^2 * dt`.** The division was uncommitted, so the repair was a revert of a
       regression that no commit ever contained -- and the diff says so unambiguously: after the repair the
       corrected line shows up as *context*, not as a change, and the only thing the file actually gained was
       the paragraph of comment above it. The lesson is not "someone made a typo" but that **a working tree
       can be red while `main` is green, and nothing in this repository noticed for as long as nobody ran
       it** -- which is what F-96 is about. The comment now pins the units and the `i^2 * tau` identity so
       the next edit in that direction has to argue with the paragraph first. Suite re-run after it: A-98.
     - ``pending``

   * - A-98
     - A
     - **Reverting the uncommitted divide-by-`dt` clears the `Protection` cluster: 16 of 16 cases pass**
       (previously 13 of them failed), and `c_api_validation`'s three failures were the same mechanism seen
       through the C ABI
       -- the fault surfaced in the C API exactly as it did in the C++ test, which is what makes a separate
       C consumer worth having. The last failure was not a defect at all: `python_bindings` was failing
       because `numpy` was missing from the interpreter CMake selects, so the second half of getting green
       was `uv pip install --python <that interpreter> numpy`, not a code change. `conan create` now exits
       **0** with **599 of 599 cases passing**, **94.22 % overall** (floor 94.0, margin +0.22) and
       **95.74 % in core/** (floor 95.6, margin +0.14). README needed no numerical edit for this.
     - Honest scope on what this proves. The 15 failures were **one** defect plus **one** environment gap,
       not fifteen, and finding them required running the suite rather than more evidence. It also cost a
       retraction: the coverage numbers quoted while the tree was red were measured wrongly (F-99), and the
       correct method is to read what the recipe prints, not to re-run a gate over a failed build's
       leftovers.
     - The consequence for A-95 is that the reason for holding `torque_map` at L2 has gone away, so the
       promotion is made in the same turn against a green tree, and `observability.json` O-16 records it as
       a level change rather than a fresh capability.
     - ``pending``

   * - F-99
     - F
     - **I measured coverage by running the gate over another process's half-written intermediate files, and
       reported the result as a fact for a whole round of work.** The number was **91.4 %**, the real one is
       **94.22 %**: a 2.8 pp error, larger than the 0.22 pp margin the floor actually has, invented entirely
       by how the number was obtained.
     - A whole hypothesis was built on it and then refuted by evidence: that `c_api_validation` and
       `python_bindings` do not reach the coverage report, argued from `foc_api.c` and `bridge.cpp` being
       reported uncovered on functions those consumers demonstrably call. That argument was sound *about the
       report I was reading* and wrong about the project: after a complete `conan create`, `coverage_gate.py`
       reads **94.22 % all / 95.74 % core**, and the 109 lines that were "missing" are exactly the C and
       python consumers' contribution, which is merged by the recipe and not by an ad-hoc re-run. The
       refutation came from re-running the gate immediately after the build finished, against the summary
       that build had just written.
     - Blast radius: `docs/verification-numbers.json` and `usage.rst` were written with the wrong figure,
       and F-96's headline quoted it, so two rows had to be corrected rather than added to. The rule this
       produces, and it is the same shape as F-89's "decide from the artefact": **a measurement tool is only
       a measurement when it owns the run that produced its input.** Running `coverage_gate.py` on its own,
       after a build that died before writing a complete export, measures the wreckage. The correct habit is
       to read the number the recipe printed, or to re-run the gate only after a build that ran to
       completion -- which is what happened here, and why the retraction is in the same log.
     - ``pending``

   * - F-100
     - F
     - **A claim in a shipped header was wider than the behaviour, and nothing could catch it because
       nothing computed the alternative.** `sinusoid_fit.hpp` said the normal-equation projection is
       "unbiased for one [window] that does not [contain whole cycles], which a plain DFT bin would not be".
       Measured: a plain rectangular-window bin is **also exact on half cycles** -- 2.50 and 3.50 cycles give
       **0.0000 %** error -- and is biased only on the other fractions (**+4.09 %** at 3.20 cycles,
       **+2.01 %** at 3.40). So the sentence was true of 3.2 cycles, false of 2.5, and had no test that
       could tell.
     - Surfaced by writing the L3 cross-check, which is the first time the alternative estimator was
       actually implemented. That is the general shape: **a comparison stated in prose but never computed is
       unfalsifiable**, and the log has no way to distinguish it from a comparison that was measured.
     - Blast radius is one sentence, and the conclusion it supports survives: the normal equations are still
       the right choice, and they are exact at *every* window length tested where the bin is exact at only
       some. The correction narrows the premise and puts the measured table where the prose used to be, so
       the next reader can check it without re-deriving the alternative.
     - ``pending``

   * - A-101
     - A
     - **`sinusoid_fit` promoted L2 to L3 on three measured sensitivities and one cross-check against an
       estimator that is supposed to differ.** Window length: the fit reads **0.0000 %** (1.2e-7) at 2.00,
       2.50, 3.00, 3.20, 3.40 and 3.50 cycles while the plain bin reads **4.09 %** and **2.01 %** at the
       last two of the non-exact ones. Frequency mismatch: 0 %, +-0.5 %, +-1 %, +-2 % and 5 % offsets, each
       against the analytic envelope `abs(sinc(dw*T/2))`, all **within 1 percentage point** (5 % offset:
       **5.159 %** measured, 4.687 % predicted). DC offset: **0.0000 %** shift with the constant column,
       **7.6 %** without it.
     - **Two of my own guesses were wrong and are recorded rather than quietly replaced.** First, I
       predicted the amplitude error would grow *monotonically* with the frequency offset; it does not --
       +0.5 % gives 0.095 % while -0.5 % gives 0.218 %, and +1 % gives less than either, because the error
       depends on how much of a cycle the mismatch accumulated, not on its size. The criterion was replaced
       with the envelope, which is a prediction instead of a shape. Second, I predicted the no-DC variant
       would be off by more than **50 %**; it is off by **7.6 %**, because without a constant column an
       offset only reaches the amplitude through the basis's non-orthogonality over a non-integer window.
       The threshold was corrected to the size the mechanism has, not widened until it passed.
     - Scope stated in the README row: the sinc envelope is a **bound, not an identity** -- the residual is
       one-sided at negative offsets and is reported as such. The cross-check leg is a real second
       estimator and a fair one (it is exact where the fit is exact), which is what makes the 4.09 %
       difference attributable to the window rather than to the comparison.
     - ``pending``

   * - A-102
     - A
     - **`protection` gains both kinds of L3 leg and deliberately stays L2, because the leg O-13 named is a
       different question and is still open.** Stepping the recurrence and solving the ODE agree to **1e-4**:
       1.0555 s against 1.0556 s at 3.2 A, 0.6635 against 0.6634 at 3.5 A, 0.4881 against 0.4881 at 3.8 A. That is a weak test by construction for a *correct* pair -- explicit Euler at a 50 us step
       over 0.66 s cannot miss by much -- so the case also asserts the measurement is **not** confusable with
       the naive `budget/i^2` (0.367 s), which it misses by more than 40 %. A comparison that cannot tell the
       right closed form from the wrong one is decoration.
     - **The sensitivity numbers are the reason this promotion is worth having.** A 10 % current-sense gain
       error moves the trip instant to **1.1877 s / 0.4672 s** from 0.6635 s -- **+79 % / -30 %**, asymmetric
       because the budget goes with the square of the current, and both land on the closed form to 0.00 %. A
       real loop period 10 % away from the one the code was told stretches the trip by **exactly that factor**
       (0.7298 s and 0.5971 s), which is a prediction the case checks rather than a shape it observes. A 10 K
       temperature-sensor offset is worth **0.6 A on a 3 A machine** -- 20 % of the current limit, the same
       order as the current-sense error the project treats as its dominant one, and unlike that one nothing
       calibrates it unless the caller does.

       Two limits are stated rather than glossed. The cross-check compares two solutions of **the same lumped
       model**, so it validates the discretisation and the closed form against each other and says nothing
       about whether `i^2*tau` describes a winding -- README's boundary row already says this is not a
       temperature, and that row did not move. And the derating path is exercised through
       `derated_limit()` directly, not through a thermal sensor, because there is no sensor model to inject
       into; the offset is applied to the number the caller would pass in.
     - **Why the level did not move, recorded because it is the rule working.** The first draft of this row
       said "promoted" and the observability gate refused it: O-13 had already re-opened this header and
       named the missing leg as *"a model-level second source for how two simultaneous faults are ranked; the
       current rank is the enum order, which is a design choice with no independent derivation"*. The legs
       added here are about the **trip instant**, not about the ranking, so the named leg is as open as it was
       -- and promoting on evidence from a neighbouring question is exactly the substitution this repository's
       levels exist to prevent. The row stays L2, the new evidence is in it, and the missing leg is unchanged
       and still named.
     - ``pending``

   * - F-103
     - F
     - **The gate that verifies every exit plan was reading an artifact under a git-ignored directory, so it
       passed locally and failed in CI on every push.** `scaffold.json` points each tolerated fork at the
       patch that proposes its upstream exit, and the gate applies that patch to the pinned baseline before
       accepting the tolerance. Both patches lived in `workspace/scaffold/`, which `.gitignore` line 10
       excludes -- so the file was present in the working tree and absent from every checkout.
     - Surfaced by reading CI instead of assuming: `Project gates` was red on every push, and the log line
       is unambiguous -- `the declared patch workspace/scaffold/0019-cppcheck-exhaustive.patch does not
       exist`. Twelve red runs were read as "CI is broken" before anyone read the one line that says why.
       The local run of the same gate was green throughout, and green is what got reported.

       Blast radius is not the gate, it is everything the gate is for: `Project gates` is the workflow that
       enforces `license strict claims docs boundaries capabilities ci-policy assume anchor scaffold budget`
       and the negative-compile probes, so a red workflow from its third step meant the remaining eight were
       **never verified in CI at all**. The artifacts now live in
       `.github/project/upstream-patches/`, next to the `scaffold-baseline/` copies the patches are diffed
       against, and the verification is done in a **fresh `git clone`** -- the only tree that looks like CI's
       -- where all eleven gates plus 27/27 negative cases now pass.
     - This is the third row of one shape, and the shape is worth naming once: F-89 decided from an exit code
       instead of an artefact, F-99 measured by running a tool over another process's leftovers, and this row
       verified a gate against a tree CI never sees. All three are **"the thing you looked at was not the
       thing you thought"**, and the local habit that prevents all three is the same: reproduce the
       condition, then report.
     - ``pending``

   * - F-104
     - F
     - **A case asserted on a floating-point rounding floor, so it failed on the platform where the code did
       better.** `TheFitIsUnbiasedOnAWindowThatIsNotAWholeNumberOfCycles` carried `ASSERT_GT(part_fit, 0.0F)`
       -- a guard written only so a ratio could be printed -- where `part_fit` is the fit's own rounding
       error: **1.2e-07 on Linux, exactly 0.0 on macOS**, because Apple's libm is more accurate. In other
       words the assertion encoded "this platform's trig must be at least as inaccurate as glibc's".
     - Surfaced by the platform matrix, not by review: `Tests / Auto Testing (macos-latest)` reported
       605 of 606 passing with that one case failing on `Expected: (part_fit) > (0.0F), actual: 0 vs 0`.
       The case had been reported green locally for a day, and locally it *is* green -- Linux is the platform
       where the noise is non-zero. A single-platform pass was never evidence about this assertion.

       Blast radius is small in files and large in kind: three of the assertions added in this session were
       comparative (`x > k * y`) where `y` sat at or near a noise floor, and the other two were removed in
       the same edit before they could fail. Both halves are now **absolute errors against the 1 % line** --
       the fit inside it on every window length, the plain DFT bin outside it at 3.20 and 3.40 cycles --
       which states the same discrimination without dividing by a platform constant.
     - The rule this produces: **a tolerance has to be written in the units the claim is about.** "This fit
       is exact" is a statement against a behavioural threshold, not against another estimator's rounding
       noise; and a quantity *at* the floor has no resolution, so comparing it to anything is comparing
       nothing. This is the second time a defect has turned out to be the criterion rather than the
       implementation (the first was A-39, a signed metric measured at a single target), and it is worth
       noting that both were found by a measurement refusing to agree, not by reading the test.
     - ``pending``

   * - F-105
     - F
     - **A timeout decided on a float sum of control periods, so it fired early and then stopped firing at all.**
       `HomingRoutine` kept `elapsed_s_ += dt_s` in float and compared it to the recipe's `timeout_s`. A float sum of
       equal terms does not lose precision slowly -- it changes speed and then freezes: once `ulp(t) > dt/2` each
       addition advances the sum by an ULP instead of by `dt`, and once `ulp(t) > dt` the addition is a no-op. At a
       25 us period the sum read "512 s" after **17 866 579** periods, i.e. after **446.664 s** of real time, and
       could not advance at all past that. So a 512 s recipe gave up 65 s early and a 520 s recipe **never gave up**,
       which is the exact opposite of what the failure path in that file promises: "giving up rather than turning
       forever". Measured at the freeze point: 512 s at 40 kHz, 2048 s at 10 kHz, 32768 s at 1 kHz -- `2^24*dt`
       rounded up to a power of two.
     - Surfaced by the L3 work on `core/homing.hpp`: the case that was supposed to show "the timeout is decided on
       the sample grid" printed `446.664 s` where the recipe said 512 s. The number was found by *not* accepting the
       first explanation -- "the caller's period is coarse" -- and instead asking where a sum of equal floats stops
       being a sum. A test that had only asserted "the search eventually stops" would have passed for a year: the
       freeze only shows up past a magnitude nobody exercises, which is why the 20 M-tick case that found it is now
       in `test_homing_routine.cpp` and the same sum is mirrored in `test_elapsed_clock.cpp`.

       **The same line was in four more modules** and one more accumulator: `startup.cpp`, `alignment.cpp`,
       `polarity.cpp`, `stall_start.hpp` (a hold timer) and `stall_start.hpp`'s torque ramp -- the last of which
       freezes at 4.19 N*m for a gentle 0.01 N*m/s recipe, i.e. *inside* its working range, because there the
       increment is `ramp*dt` rather than `dt`. Four of the five had limits small enough (milliseconds to a couple of
       seconds) that the floor was never reachable, which is precisely how a defect survives: not because it is
       invisible, but because the numbers used in every test sit below it. Finding the first one is what made the
       other five findable, and fixing only the first would have been the same defect one magnitude away.
     - Blast radius, from `gates.py claims --impact core/types.hpp`: **40 claim rows** rest on the header that now
       carries the clock, so the re-run is the whole core suite plus the SIL cases that own a timeout --
       `test_homing_routine.cpp`, `test_sil_homing.cpp`, `test_startup.cpp`, `test_sil_startup.cpp`,
       `test_alignment.cpp`, `test_polarity.cpp`, `test_stall_start.cpp`, `test_elapsed_clock.cpp`, all green, and
       the recipe ticks are now what the arithmetic says: 40 000 / 20 480 000 / 20 800 000 for 1 s / 512 s / 520 s at
       40 kHz.

       The rule this produces: **a duration that only grows is never accumulated in float.** It is not a style
       preference and the fix is not an `if` -- a growing sum of periods has a *speed* proportional to its own
       magnitude, so it has no valid range that a tolerance can be written against. `gates.py clocks` now scans
       `include/` and `src/` for a float accumulated with a `dt` term and fails on every one that is not on an
       allow list carrying its bound (`speed_elapsed_s_`, `position_elapsed_s_`, `stall_elapsed_s_` -- all bounded by
       construction, all cleared or faulted before their magnitude could matter). Its `--selftest` classifies four
       snippets, so a classifier that has stopped finding anything cannot pass by finding nothing. The deeper lesson
       is about the earlier L3 row itself: it **measured** this defect, wrote it into the README as a limitation, and
       moved on. A measured defect on the boundary of a published claim is not a limitation to record -- it is the
       claim being wrong, and recording it is what made it look handled.
     - ``pending``

   * - F-107
     - F
     - **The documented go/no-go rule for field weakening could never return `true`, and the branch that decided it
       was undocumented.** `is_field_weakening_worthwhile()` promised `worthwhile = ratio < 0.6 && speed_gain >= 1.5`,
       each half with a rationale. An earlier `if` returns `true` whenever `Ld*i_max >= psi_f` (the current limit can
       null the flux), so the surviving expression was reached only with `lambda = Ld*i_max/psi_f < 1`, where
       `ratio = 1/lambda > 1` and `gain = 1/(1-lambda) < 1.5` -- i.e. it was **unreachable-as-true**. The function's
       real rule was one comparison, `i_char <= i_max`, and it was written nowhere.
     - Surfaced by turning the advice into a selection table and asserting each verdict against the *documented* rule
       recomputed from the same nameplate. Four of five rows agreed -- they are all far below the line, where both
       readings say no -- and the disagreement only appears in `1 <= lambda < 5/3`, a band no real machine in the
       collection occupies. That is the shape of the defect worth remembering: **two pieces of code agreed on every
       input anyone had tried, because every input was far from the boundary.**
     - Blast radius: the verdict is exposed to C as `foc_field_weakening_worthwhile`, so the documentation of a public
       answer was wrong for a whole band of machines. No behaviour changed -- the code's answer (a step at
       `i_char = i_max`) is the defensible one, since below the line the flux cannot be nulled at all, and the
       docstring's own rationale ("the region is a sliver") describes exactly that. The fix is to the header: the rule
       is now stated as the one comparison, the two dead thresholds are removed, and `test_field_weakening.cpp` pins
       the step at `lambda = 1` (bisected to 1e-5) and asserts that neither old threshold could ever have decided.
       Re-run: `test_field_weakening.cpp`, `test_field_weakening_motor.cpp`, `test_api_c_smoke.cpp`, all green.

       The rule this produces: **a threshold that cannot bind is not a policy, it is decoration.** When two conditions
       are written in series, prove that each one can be the deciding test -- otherwise remove it and say what the
       remaining one means. And the corollary that made it invisible for so long: a shipped advisory should be run over
       **every machine you have parameters for**, not just the one on the bench.
     - ``pending``

   * - F-108
     - F
     - **The mode machine's documented transition rule omitted two of its seven edges, and the two it omitted are the
       aborts.** `sm.hpp` said "Idle -> Align -> Run, any state -> Fault, Fault -> Idle on an explicit clear". Measured,
       `Align -> Idle` and `Run -> Idle` are legal as well -- the table has seven edges, not five -- so the sentence
       told a reader that a running motor **cannot be commanded back to Idle**. A caller obeying the documentation
       would have had to fault the drive in order to stop it.
     - Surfaced by measuring the machine as a **graph** instead of as a table of cells: enumerating the relation on the
       code (four start states x four targets) gives out-degree `2 / 3 / 2 / 0` and in-degree `2 / 1 / 1 / 3`, and the
       seven edges only balance against the documentation if the two aborts exist. The cell-by-cell test that was
       already there could not catch it: it compared the table against a *copy of the table*, which is exactly the
       shape of check that cannot notice a sentence disagreeing with both.
     - Blast radius: the class documentation of a safety-subset module, and any caller that planned a stop. No code
       changed -- the behaviour is right and the abort is the useful edge. `test_sm.cpp` now enumerates the relation and
       computes the graph properties over it (every state reachable from `Idle`, `Fault` the only sink, no self-loop
       through `request()`), so the sentence is no longer load-bearing; the header states the seven edges and names
       the one thing that is easy to get wrong in the other direction (**`request(current)` is a refusal, not a
       no-op**, so a supervisor that re-asserts its target every cycle fills `illegal_transitions()`). Re-run:
       `test_sm.cpp`, 14 cases green, and `gates.py mcdc` still 100 % on the subset.

       The rule this produces: **a table checked against a copy of itself is not checked.** For a state machine, the
       check with information in it is a graph property computed from the *measured* relation -- reachability, sinks,
       degrees -- because those are the things a reader of the prose is actually relying on.
     - ``pending``
   * - A-109
     - A
     - **The first instrument for the saliency bias could not see it, and the negative result read like a finding about
       the code.** Measuring the new `DualInductanceObserver` against the shipped single-`L` observer on the switching
       rig gave 0.13 to 0.15 rad for *both* of them at every d-axis current -- i.e. "the dual-inductance observer is
       useless" -- while the algebra says the whole difference is a closed form. At `psi_f = 4.76 mWb` the 800 rad/s
       back-EMF is 3.8 V, so a few tenths of a volt of dead-time volt-seconds is already 0.1 rad of flux angle: the
       impairment was the size of the effect and common to both observers, so it cancelled nothing and buried
       everything.
     - Surfaced by a **zero-current probe**. With no current neither model has a saliency term to neglect, so whatever
       both still report is the instrument itself: 0.0024 / 0.0096 / 0.0196 rad at 100 / 400 / 800 rad/s, which is
       `w_e*dt/2`, the backward-Euler lag of the flux integrator, and the line goes through all three points. What would
       have shown it first is that same probe **before** the effect was measured -- it is one loop and it costs a second.
     - Blast radius: no existing claim moved, because the wrong reading was never written down; the new row
       (`core/observer/dual_inductance_observer.hpp`) and O-38 are its only consumers. The table was retaken on a
       speed-locked plant driven by the exact steady-state d-q voltage at 100 rad/s, where `w_e*dt/2` is 0.0025 rad and
       the bias under test is 0.021 to 0.105 rad, and the 800 rad/s runs are recorded inside the test as *not usable*
       rather than quoted. Re-run: `test_dual_inductance_observer.cpp`, 5 cases green.

       The rule this produces: **measure the floor before the effect, and measure it where the effect is zero by
       construction.** A floor subtracted from a table is an assumption with a number next to it; a floor measured at
       zero current is the instrument answering a question whose true answer is known.
     - ``pending``
   * - F-109
     - F
     - **Four gates had never run in CI, and the workflow's gate list was a second copy of the gate set.**
       `clocks`, `observability`, `combinations` and `domain` were each implemented, registered in `PROJECT_GATES`,
       given a dispatch branch in `gates.py`, and documented -- and the only workflow that runs project gates
       enumerates gate names by hand and stopped at `budget`. So they ran when a human typed them and never
       otherwise. Two of the four carry this session's evidence chain (`observability`, `combinations`), and one
       guards the F-105 timeout rule.
     - Surfaced by asking "does CI actually run the gate I just added" during a records-vs-implementation check:
       the records were consistent, and the wiring was not. What would have shown it first is a comparison inside
       `ci-policy`, which already knows the gate set -- it prints the local-only exclusions -- but never compares
       it with what the workflows actually enumerate. That comparison is now written into the workflow as a
       comment, and the loop carries every non-local gate.
     - Blast radius: no claim moved, because every one of the four was green locally; what was missing was the
       enforcement, not the check. Re-run: the widened loop, fifteen gates green on the runner, and `ci-policy`
       still refuses the five whose cost belongs on a developer machine (`fmu`, `mcdc`, `mutants`, `pil`,
       `plantref`).

       Third instance in one day of the same shape: **a second copy of a set that is not derived from the set.**
       The first was `foc_motor_set_observer`'s upper bound saying `FOC_OBSERVER_BLENDED` instead of the last
       enumerator, so a new mode was refused; the second was the bilingual note pair in `foc_types.h` where the
       Chinese half had been false for several releases while the English half was right; this is the third. The
       rule they produce: when a set already exists somewhere, the consumer must derive from it or a gate must
       compare the two, because a hand-maintained list of "everything" fails silently and the failure looks like
       success.
     - ``pending``
   * - A-110
     - A
     - **The instrument, not the library: the fifth observer member was never configured.** The row read "the
       dual-inductance observer is not hand-over-safe and the deficit is pi" -- 2.9834 rad 20 ms after the hand-over
       against the other high-speed member's 0.7244, from the same inherited 0.5611. The real cause is that
       `observer_config()` in the rig never assigned `config.dual_inductance`, so the member ran with an all-zero
       `Config{0, 0, 0, 0}`, `valid_config` refused it, `update()` held its state and the readout was a constant. With
       the member configured the two land **0.6543** and **0.7244** rad out -- the same inherited error, 9.7 % apart.
     - Surfaced by asking a criterion that had been withdrawn as *vacuous* (C2: "within 20 ms the angle is inside
       0.05 rad", withdrawn because the rig it was written for was sensored). That refutation **stands**: both members
       are far outside 0.05, and for the reason the rig can now name -- the *outgoing* observer hands over an estimate
       that is 0.56 rad out, and neither incoming member can forget it in 20 ms. What does not stand is the claim that
       the incoming member was broken.
     - The falsifier was in the case the whole time and pointed the wrong way: it asserted `max_error > 2.0` "so it
       goes red the day the member is repaired". A recorded failure asserts a *number*, so the first thing to check
       when the number is a constant is whether the member is running at all. `status()` returns `Param` for an invalid
       config, and a rig that never reads it cannot tell a held output from a wrong algorithm -- the two are identical
       at the port. The case now asserts faithfulness for both members (0.8x to 2.0x the inherited error) and that the
       two agree within 15 %, and every rig that configures a member it also selects now prints the status.
     - ``pending``
   * - F-111
     - F
     - **The flux-source correction is not a remedy for a wrong seed: it is a nearest-point search and it changes
       branch.** With an exact seed the corrected source is worth 7.71x under a nameplate `Rs` error (recorded in
       O-56's neighbourhood as an L2 result). With the seed 0.30 rad off it is 8x **worse** at 0.2 A and 3.7x worse
       at 3 A (mean error 1.545 / 1.530 / 1.570 rad for the corrected source against 0.195 / 0.222 / 0.424 for the
       pure integrator, exact `Rs`, 200 rad/s electrical, 2 s). So "hand over, then switch the correction on so the
       inherited error decays" does not work, and the flux source's measured domain is exactly *exact seed, nameplate
       error* -- nothing wider.
     - Surfaced while looking for a fix for F-110, i.e. before F-110 turned out to be the rig's. The same behaviour
       shows in the chain: with `FluxSource::Corrected` the dual-inductance member lands 2.9834 rad out at gain 2 and
       3.0830 at gain 0.2, against 0.6543 for the pure integrator.
     - The pull is a projection onto the flux curve, and an angular seed error moves the *nearest* point to the other
       branch; the branch is then kept by continuity and never revisited. Nothing in shipped code changes (the ABI
       leaves the member on `Integrator`); what changes is the declaration, which now carries the domain instead of
       selling the correction as an upgrade.
     - ``pending``




   * - A-111
     - A
     - **Upstream shipped both scaffold proposals, and nothing here would have noticed.** `fcpp` v0.3.4
       (`701a2452`) took the cppcheck argument (`f2519753`, extended by `98ea19a1` to the C descriptor as
       well -- `.h`/`.c` are linted by C, so the patch I proposed was incomplete) and the bilingual filter
       fixes (`88f796b3`, plus `2f17bedd` stating the three shapes). Our tree sat on v0.3.3 for two releases
       with two tolerated forks that no longer had anything to tolerate.
     - Surfaced by the user reading upstream's release, **not** by this repository: the `scaffold` gate compares
       the working tree against the *pinned* bytes, so a newer upstream release is invisible to it. That is the
       right design -- a gate that needs the network is not a gate -- but it means the pin is a manual fact, and
       three artefacts had already gone stale behind it: the `_comment` paragraph arguing the two forks "cannot
       be removed from the private side" (true only while upstream had not shipped), a `v0.3.3` literal in
       `scaffold_gate.py`'s own docstring, and a `scaffold-baseline/docs/build.py` copy of v0.3.3 bytes that
       nothing referenced once the tolerance disappeared.
     - Pin moved to v0.3.4 with all 33 checked files re-hashed from the release tree, `tolerated` emptied,
       `baseline_divergences` 2 -> **0**, the stale baseline copy deleted, and the docstring made
       version-agnostic. The adoption was verified rather than assumed: both `docs/build.py` versions were run
       over all 61 public headers in both languages and produce **122/122 identical** filtered outputs, so the
       merge changed no documentation content. The rule this adds: **a pinned byte-check detects drift from the
       pin, never the existence of a newer release** -- so the pin carries the date it was last reviewed, and a
       scaffold release is a scheduled task, not something a gate will remind anyone about.
     - ``pending``


   * - A-112
     - A
     - **A constant that had never been executed was wrong by 18 %, and it was the one the plan was about to
       trust.** `oracle_ornl.py` carried `LOCKED_ROTOR = [(75, 140), (150, 260), (250, 400)]`, transcribed from
       ORNL/TM-2006/423's *summary* Table 6.3, and nothing in the repository ever read it. The primary data is
       Table 3.2 (p.19): 7 currents x 23 shaft angles, whose column peaks are **74.0 / 117.7 / 158.0 / 199.0 /
       229.0 / 286.6 / 339.0 N.m** at 50..250 A. Every summary value is **1.14 to 1.19x** the primary peak at the
       same current (1.189 / 1.135 / 1.180), i.e. a constant normalisation difference rather than a transcription
       slip -- and using it as "the measurement" would have reported an 18 % error in a parameter set that is
       within 3 %.
     - Surfaced by finally using it, as the cheap pre-check for B-2's charter. The 1.80x gap between a 75 A
       point and an 82 N.m prediction was too large to be physics, so the table was re-read from the PDF with
       `-layout`: that also settled the unit -- the currents are **per-phase rms** (peak = sqrt(2) I), proved by
       the 50 A point landing at **+0.7 %** and 125 A at **+0.2 %** under that convention, against **-42 %** if
       the column value were taken as a peak.
     - No kernel claim moves: this is `IPMSM_A`, which A-91 already says is not this library's machine. What
       changes is the tool and the statement of what the data can do. `--torque` now carries Table 3.2, a
       transcription self-check (peaks monotone, mean below peak, the 6.3/3.2 ratio constant), and a
       **sensitivity scan printed on every run**: these seven points pin `psi_f` to **[0.93, 1.02]x** the ORNL
       value -- which rejects the FEMM route (0.18826 Wb, **1.089x**, out of band at 150 A) on a measurement that
       does not share its assumptions -- and pin `Lq - Ld` to essentially **nothing** (`k` in [0, 1.1] passes, and
       `k = 0` passes, because the reluctance term is a small share of the torque at these currents). The first
       version of this check claimed to validate `Lq - Ld`; its own sensitivity scan is what forced that sentence
       to be withdrawn, and it is printed every run so the same claim cannot creep back.
     - ``pending``


   * - A-113
     - A
     - **The same "count the file set" check was applied to one upstream directory and not the other.** The
       scaffold gate has compared the *set* of `.github/misc` since it was written -- "a byte-check of the files
       upstream ships cannot see a directory that has quietly grown extras" -- but `.github/workflows` got only the
       byte comparison of the ten files upstream ships. This project adds an eleventh, `project-gates.yml`, so for
       the whole life of the private gate suite there has been a CI file in the template's directory that nothing
       checked and nothing named. The claim itself was honest ("the scaffold's build and CI files are byte-identical
       to the pinned release"), which is exactly why nobody looked: a scoped claim that is true hides the unscoped
       question next to it.
     - Surfaced while answering "is this repository really 0-IaC-change on the template": the answer needed the
       inventory rather than the claim, and the inventory took one `ls` against the release tree. Nothing was
       misbehaving -- the private workflow is deliberate -- so the defect is entirely in what was *recorded*.
     - `.github/workflows` joined `owned_dirs` with `project-gates.yml` as a declared addition, and the check was
       shown to bite before being trusted: adding a stray `stray-test.yml` turns the gate red, removing it turns it
       green. `.github/skills` is still outside the byte check (agent guidance is project-specific by design) but its
       count is now recorded rather than implied: 39 upstream files, 39 tracked, **15 of them rewritten** here.
       The general form: **"zero modifications" is a claim about files upstream ships; the question next to it is
       "zero additions to directories upstream ships", and only a set comparison answers that.**
     - ``pending``


What ``pending`` means in the commit column
-------------------------------------------

A row cannot cite the hash of the commit that contains it, and these rows are written in the same commit as the fix, so
154 of the 167 say ``pending``. Both counts are grepped from the table, not counted by hand -- the previous version of this sentence was two rows out, which is exactly the kind of number that goes stale while reading as a fact, and it had gone stale again before this line was corrected again. That is a convention and it was, until this paragraph, an unstated one --
which is the same defect the rows themselves are about. The row is recoverable rather than lost:

.. code-block:: console

   git log --oneline -S 'A-16' -- docs/sphinx/source/foundation-log.rst

prints every commit that added or removed that string, and the earliest of them is the commit that introduced the row
(verified for A-16, A-13, C-5 and F-9). A hash in that column therefore means "the row was written later, and cites the
commit that carried the fix"; ``pending`` means "the fix and the row are the same commit, use the command above".

How the table's shape is kept
-----------------------------

A ``list-table`` whose rows disagree with its header does not render badly -- it renders as **nothing**. Sphinx emits
``ERROR: Error parsing content block for the "list-table" directive`` and drops the table whole, and
``docs-build.yml`` greps the build log for ``ERROR:`` and fails the job. So a ragged row is both invisible in the page
and fatal in CI, while ``gates.py docs`` -- which ran the Doxygen formatter and the number sheet and nothing that parses
rST -- stayed green. 25 rows were in that state, the widest of them carrying 19 items against a 6-column header, all of
them added while appending findings paragraph by paragraph with a ``-`` that starts a new cell.

The repair was mechanical and is asserted to be lossless: ``.github/project/rst_table.py fix`` merges the surplus
paragraphs into the "how it surfaced" cell as extra paragraphs and leaves columns empty where a row genuinely has nothing
for them, and it refuses to write unless the whole file's word sequence is unchanged. ``gates.py docs`` now runs its
``check`` mode, which needs no docutils, so the check cannot silently skip on a machine without Sphinx. The habit that
prevents a recurrence: a new row is six items -- ID, class, root cause, how it surfaced, blast radius, commit -- and a
second paragraph belongs inside a cell at seven spaces of indent, not behind another ``-``.

Two kinds of row that are currently empty
-----------------------------------------

The page is honest about its own coverage too. There are two kinds of row this log expects and does not yet
contain, and their absence is a fact about the work so far, not a guarantee that the categories are empty:

- **A claim that shrank.** When a defect cannot be found within three attempts, the rule is to stop and lower the claim
  (an L3 becomes an L2, or becomes "not claimed") instead of shipping a number nobody can explain. That has not happened
  yet -- which means no claim here has been tested to destruction.
- **An unexplained pass.** A case that is green for a reason the author cannot state is treated as red. Nobody has
  written one down yet.

Both will be rows when they happen, with the same weight as the defects.
