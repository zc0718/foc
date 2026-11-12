_`What each capability offers, and what it costs`
=================================================

The :ref:`boundaries page <What this library does not do>` says what this library cannot do. The README's claim table says
what is measured and how it is checked. Neither answers the question the person adding an algorithm actually has: *what
does a capability have to come with?*

This page is that answer, one row per capability. Five columns: the four things a new capability owes, plus the
case name that lets the row be audited.

1. **Diagnostic channel** -- how the capability says "I am not measuring", "I have not converged", "I am holding". A
   capability whose only output is a number cannot be debugged in the field, because zero and "no idea" look the same.
2. **Admission** -- where a contradictory configuration is refused: at construction or at the first step, never by
   silently falling back to a default.
3. **Boundary** -- the limit, pinned by a case. The cell grammar is fixed and the gate enforces it:
   a case name in double backticks; or ``none -- <reason>`` for "no limit is known", which therefore requires an **L3**
   claim behind it; or ``na -- <reason>`` for "this dimension does not apply"; or ``todo -- <reason>`` for a hole that is
   written down instead of papered over. The gate counts the `todo` rows on every run.
4. **Invariant participation** -- which case in `test_invariants.cpp` this capability must obey. ``na -- <reason>`` is
   allowed and the reason is required.

The `capabilities` gate checks every cell: a header under `core/` or `motor/` must be listed here or named in
`capability-scope.json` as deliberately unlisted; every case name must exist as a `TEST()`; every ``none`` must have an
L3 claim; every reason must actually be a reason. A row that cannot be filled honestly fails the build, which is the
point -- the alternative is a library that grows algorithms and forgets how they fail.

The three questions a criterion has to survive before it is written down
--------------------------------------------------------------------------

Not every one of them can be checked by a machine, which is why they live here as prose rather than
as a gate. Each is a defect this repository has actually shipped, and each was invisible to the
coverage numbers at the time:

1. **Is it vacuously true on the rig I am actually using?** The sharpest example: "after hand-over,
   the angle error is within 0.05 rad in 20 ms" was asserted on a *sensored* bench, where the loop
   uses the sensor's angle -- so the error against that sensor is identically zero. The criterion was
   green, the number was a statement about the bench rather than the capability, and the fix was to
   delete it and state one that can fail.
2. **Does it only have an upper bound?** "must be better than X" alone pushes the next reader to tune
   until it passes; the honest form also records where a design is *expected* to be worse. The
   dead-band ramp is the example: it is worse than a plain ``sign()`` step on an ideal step, and that
   has to be written into the table rather than quietly avoided when tuning.
3. **Does the probe point distinguish the competing formula?** Before claiming that a case can reject
   a wrong formula, evaluate both at that point. ``sqrt(2ae)`` against ``sqrt(ae)`` was probed at the
   one place where the two coincide, so the case had no teeth and nothing said so.

Capabilities that are a procedure rather than a formula
---------------------------------------------------------

Some of what this library offers is not a law but a *procedure*: identification of the machine
(``commissioning.hpp``), speed-loop tuning (``auto_tune.hpp``), learning the cogging table
(``cogging_learn.hpp``). Their criterion is not a constant that could be written into an assertion, so the row records
what the procedure owes instead:

1. **It says when it cannot answer.** ``valid == false`` plus a reason, never a number produced from data that does not
   describe this machine. A procedure that only answers on good data cannot be claimed for any data.
2. **It states the band it is valid in**, in the units the caller measures -- a speed band, a window length, a number of
   revolutions -- and the band is pinned by a case rather than described in prose.
3. **Its objective is a held-out margin, never accuracy.** A procedure optimised for "smaller error" produces a
   criterion fitted to the model it was scored on; the number worth publishing is the distance to failure, measured on
   machines that took no part in the fit.

Such a capability reaches **L2** when the procedure is reproducible and its refusals are reachable, and that is the
whole of what "a procedure" buys. **L3** still needs the same two things every other row needs -- item-by-item
sensitivity and an independent cross-check, e.g. the same procedure replayed on a second plant -- so a procedure is not
a shortcut past the evidence levels, and it never reaches L4 without hardware.

**The three obligations are executable rather than aspirational.** ``test_sop_outer_loop.cpp`` drives the outer-loop
procedure end to end -- identify, propose, measure, held-out acceptance -- on two machines of different inertia, and
records the objective as a **distance to failure** (``tolerance - |j_identified/j_nameplate - 1|``), a number that
changes sign exactly where the library refuses. A gain fitted on the wrong machine is caught in both directions by a
band whose two edges are themselves measured (the gain law's own residual, and the smallest nameplate error the repo
calls actionable). The three red proofs were run and are recorded with the numbers: replacing the held-out machine with
a copy of the fitted one, deleting the refusal path, and collapsing the parameter search to a single setting each turn
exactly the expected case red. With that, the replay-on-a-second-machine half of **L3** is in place for this procedure;
the sensitivity half is not, so the row stays **L2**.

.. list-table:: capability matrix
   :header-rows: 1
   :widths: 20 24 20 28 8

   * - Capability
     - Diagnostic channel
     - Admission
     - Boundary
     - Invariant
   * - ``core/transforms.hpp``
     - na -- stateless: there is no state that could hold a stale angle, and the caller owns the angle.
     - na -- no configuration; a wrong argument is a wrong answer, which the invariance case catches.
     - none -- Clarke is blind to the zero sequence, which is stated and asserted rather than corrected; nothing
       else is known to break.
     - TheAmplitudeConventionHoldsAndTheZeroSequenceIsLost, ARotationAndItsFrameComposeExactly
   * - ``core/limits.hpp``
     - na -- stateless: a clamp with no memory has nothing to report about itself.
     - na -- no configuration; a limit wider than the signal is a caller decision, not a contradiction.
     - ``Invariants.TheClampIsContinuousAndAJumpWouldShow``
     - TheClampIsContinuousAndAJumpWouldShow
   * - ``core/pi.hpp``
     - ``status()``, ``integral()``, ``output()`` -- the integrator state is readable because a wound-up integrator is
       the failure mode that looks like a tuning problem.
     - ``status() == ErrorCode::Param`` on a negative gain, an inverted clamp or a non-positive ``dt_s``; the output
       holds instead of moving.
     - na -- the loop is linear and every limit on it is a clamp or a gain; the limits that matter are the plant's.
     - ElapsedTimeDecidesTheIntegralNotTheCallCount, TheClampIsContinuousAndAJumpWouldShow
   * - ``core/filters.hpp``
     - ``configured()``, ``size()`` -- an unconfigured filter says so instead of passing its input through untouched.
     - ``configure()`` refuses a non-positive cutoff or step.
     - na -- a first-order filter is exact for its stated cutoff; the limit that matters belongs to the loop
       around it, not to the filter.
     - na -- linear and time-invariant; the invariants that bite are on the loop around it, not on it.
   * - ``core/protection.hpp``
     - ``thermal_budget()``, ``derated_limit()``, ``fault_is_latched()`` -- "fine", "derating" and "stopped" are three
       readable numbers rather than one code. The split is by definition and not by accident: `output_allowed` is
       `active_mask == 0`, so a fault in this library *means* "authority removed", and derating -- the axis still
       running at reduced authority -- is announced by `i_limit_a` falling below the configured `i_max_a`. `FaultCode`
       names the mechanism, and the newest one (`DriveFault`) is the only cause that does not come from the protector at
       all: it is the power stage telling on itself, read at the top of the tick and appended to the enum so every
       existing number keeps its meaning.
     - na -- the configuration is a set of numbers; a nonsensical one is refused by the constructor that consumes it.
     - ``FaultInjection.TheProtectorCodesReachTheFaultCodeThroughTheDocumentedHop``
     - na -- a supervisor over other capabilities, not an equation of its own.
   * - ``core/speed_from_angle.hpp``
     - ``ready()``, ``lost_count()`` -- differentiates an angle and says when the angle stopped arriving.
     - ``reset()`` is mandatory after a gap: the module will not publish a speed across one.
     - ``SilSpeedFromAngleClosedLoop.AWindowShortEnoughToBeOnTimeIsTheOnlyWayThisRouteWins``
     - ElapsedTimeDecidesTheIntegralNotTheCallCount
   * - ``core/trace.hpp``
     - ``size()``, ``dropped()``, ``latest()`` -- a full ring keeps recording and counts what it overwrote, so \"I lost
       the window\" is a number rather than an absence. ``t_us == 0`` on every sample means the platform has no clock,
       which is a fact about the HAL and not a claim that everything happened at once.
     - ``n/a`` -- nothing to configure: the capacity is the type, and reading past the retained window returns a zeroed
       sample instead of reaching out of bounds.
     - ``Trace.TheRecordAgreesWithTheScalarReadBitForBit``
     - na -- a recorder of other capabilities' arithmetic, with none of its own.
   * - ``core/soft_limit.hpp``
     - `at_wall`, and the `Param` that a refused target returns. A clamp would make an impossible command look plausible
       and the caller would never learn; the status is the only channel a programming fault has.
     - `Param` for a target outside the window (the previous one comes back), and for an inverted or non-finite window.
       A wall is *inclusive*: "move to the limit" is the one command a soft limit exists to make safe.
     - ``SoftLimit.TheDelayTermIsTheDifferenceBetweenNineteenMillimetresPastAndOneShort``
     - na -- no integration; the remembered target is the whole state.
   * - ``core/gear_ratio.hpp``
     - `master_unwrapped_rad` and `slave_velocity_rad_s`. The unwrapped master is the synchronisation error's reference --
       comparing against the wrapped one is how a perfect pair reports 6.28 rad once a revolution -- and the velocity is
       what makes the `v*kd/kp` lag removable at all.
     - `Param` for a zero, negative-infinite or NaN ratio, and for a non-finite master sample (the unwrap state is then
       untouched). A zero ratio is refused because a decoupled slave and a stuck slave look identical.
     - ``ElectronicGear.TheSyncErrorIsTheRampLagInBothOfItsTerms``
     - na -- one state variable (the accumulated master angle); the cases check that a ratio change does not move it.
   * - ``core/friction_comp.hpp``
     - ``at_rest`` and `stalled` -- and the fact that the output is a *constant* at rest rather than a square wave. A
       Coulomb term on `sign(measured speed)` ripples by twice the term where the stage is needed most; the latch is what
       turns that into a number an axis can hold. `stalled` separates "holding a load" from "going nowhere", which the
       position signal cannot.
     - `Param` when the estimate describes a machine that cannot exist -- kinetic friction above breakaway, a negative
       coefficient, a non-finite value -- and in that case nothing is added at all, rather than a smaller guess.
     - ``FrictionComp.AnEstimateAboveTheBreakawayWalksTheAxisWithNoCommandAtAll``
     - na -- no time constant and no integration; the only state is the latched direction, and the cases check that it
       survives a re-tune.
   * - ``core/hall.hpp``
     - ``status()`` -- three answers rather than one number: `Fault` when the code is 0 or 7 (the two a three-bit field
       leaves over), `State` when the interval between edges stopped looking like a sector, `Ok` otherwise -- plus
       `illegal_reads()`/`stale_reads()`, which say which channel fired. A code-only decoder cannot know how wrong it
       is; these say whether it is measuring at all.
     - `Param` on a non-finite timestamp; an illegal code answers a zero vector and `Fault` rather than a guessed
       vector, and an interval that leaves `[period/1.5, period*1.5]` is refused rather than adopted as the new
       reference -- adopting it is what a dead sensor needs to stay hidden (measured: 36 % of a revolution reporting
       `Ok` with a 120-degree error, against 8.4 % once refused).
     - ``HallTracker.AStuckSensorIsCoveredByTwoChannelsAndNeitherAloneIsEnough``
     - na -- a per-sample decoder; the only cross-frame state is the sector period, and a doubtful one is kept rather
       than taken.
   * - ``core/brake_chopper.hpp``
     - `BrakeChopperOutput`: `duty`, the **period-average** `current_a`, `i2t_a2s` (the integral of the **mean square**
       current, which is not the square of that average -- they differ by `1/duty_max`), `on`, and the latched
       `budget_exceeded`. In SIL the same quantities are readable a second way, from the bus: `r*i2t` and the energy the
       meter books against the resistor agree to 0.045 %.
     - `Param` for a chopper that cannot act (a zero resistor, an inverted band, a zero duty) and for a period
       of zero, in both cases **without** changing any state; a spent budget latches rather than resuming.
     - ``SilBrakeChopper.TheChopperHoldsTheBusAndTheBooksClose``
     - na -- the bounds and the current law are asserted in the capability's own case, swept across the bus range with the wrong law computed alongside; the invariants suite is for statements that must hold whatever composition is selected, and a chopper is not on a composition axis.
   * - ``core/min_pulse.hpp``
     - `MinPulseOutput` carries `duty` (what to command), `lost_vs` (the volt-seconds the bridge will not deliver,
       signed and per phase) and `narrowed`; the residue is readable through `carry()`, which is what lets a trace
       tell "the loop is asking for something the driver refuses" from "the loop is fine".
     - `Param` for a limit that cannot *be* a limit (`2*t_min > T`, a negative limit, a zero period), for a bus
       voltage that is not a voltage, and never by moving the residue: a refused call changes no state, so a bad
       configuration cannot quietly bias the average it is supposed to correct.
     - ``SilMinPulse.ThePlantTransmitsWhatTheDriverAllowsAndThePolicyKnowsThePrice``
     - na -- a per-period policy whose only state is one residue, bounded by one minimum pulse and asserted as
       such; it produces no continuous output for the invariants suite to hold.
   * - ``core/single_shunt.hpp``
     - `ShuntVerdict` has three answers, not a boolean: `Measurable`, `WindowTooNarrow` (this period cannot be read, the
       reference is near a sector boundary) and `NoActiveVector` (nothing flows through the shunt at all, so nothing is
       missed), plus `ShuntWindow.window_s` / `needed_s`, which say how far from the edge the period was. A refused
       reconstruction returns `valid == false` **and** zeros, so a caller that checks only the numbers sees a motor drawing
       no current -- which is why the flag exists.
     - `Param` for a configuration that can never read anything (`2*(t_settle+t_sample) >= T`), for a non-finite timing
       and for a duty outside `[0, 1]`; a period outside the window is **not** a `Param` -- it is `Ok` with
       `valid == false`, because a narrow window is the normal operating condition and not a configuration error.
     - ``SingleShunt.TheBlindSpotIsLowModulationNotTheVoltageLimit``
     - na -- stateless, one period at a time, and the thing it holds is a verdict rather than an accumulated value.
   * - ``core/mtpv.hpp``
     - `MtpvPoint` names which ceiling decided the point: `voltage_limited` is true only when the voltage ellipse moved the
       answer, and `current_a` is the margin to the current ceiling, so a host can tell "the voltage is what limits me"
       from "the current is". `base_speed_rad_s()` is derived (`v_max` over the flux magnitude at the current-limited
       point), not `v_max/psi_f` -- on the in-domain profile the two differ by 4x, 725 against 2520 rad/s. The
       boundary case is the band where *both* ceilings are active at once (725 to 1191 rad/s there): a
       single-constraint law reports that band as 30 % over the voltage ceiling, which is what the case asserts.
     - `Param` for a ceiling that is not positive, a non-positive inductance, a negative flux linkage, zero pole pairs and
       anything non-finite, and `Param` from `at_speed()` for a speed that is not finite. There is no "no answer" case
       above zero speed: the `i_q = 0` point on the ellipse is always inside the current circle, so the candidate set is
       never empty and a zero-torque point is returned rather than a failure.
     - ``Mtpv.StayingOnTheCurrentLimitThereWouldOvervoltTheDrive``
     - na -- stateless, one speed at a time, and what it produces is an operating point rather than an accumulated value.
   * - ``core/cogging_comp.hpp``
     - `CoggingCompOutput` separates the two ways a compensator can stop working: `gain` is the frequency fade ("we
       chose not to ask for this ripple") and `clipped` is the current ceiling ("we asked and it was refused"), with
       `torque_nm` carrying the cancelled torque before either limit. A compensator that reports only its output current
       cannot be told apart from one that is silently doing nothing.
     - `Param` for a machine that cannot produce torque at the `id` it is given (`dT/diq = 0` at the flux-nulling
       current, where no current can cancel a torque), for any non-finite input, and for a fade band that is inverted or
       non-finite. A table of nothing is `Ok` with a zero offset -- there is no table, so there is no decision to report.
     - ``CoggingComp.ATableWithTheWrongPhaseCancelsNothingAtAll``
     - na -- stateless, one period at a time, and what it produces is an offset rather than an accumulated value.
   * - ``core/cogging_learn.hpp``
     - `CoggingLearnReport` reports `revolutions` and the speed band on **every** exit, including the refused ones: a
       refusal that says nothing leaves the caller guessing whether to spin longer, slower, or steadier, and those are
       three different actions. A table it never saw is absent from the output rather than extrapolated, because a
       compensator acting on an invented harmonic injects a ripple of its own.
       **Reachable from C** (six `experimental` entries:
       `foc_cogging_learn_create/accumulate/ready/finish/reset/destroy`), so a script can learn a machine's table. What
       this answer is, is a **table**, so the C contract carries a capacity clause the C++ one does not need: `finish()`
       echoes `required` (how many entries the answer needs) and `capacity` (how many the caller offered) on **every**
       return, and given too few slots it refuses with `FOC_ERR_PARAM` instead of writing a shorter table -- a truncated
       table tells the caller it asked for fewer orders, and the harmonics it is missing are exactly the ones nobody kept
       a record of. Passing `out_table == NULL` asks for the report alone: `FOC_OK` with `count == 0` then means *the
       window is a measurement and there was nowhere to put it*, which the C++ signature cannot say, because there
       `nullptr` with `capacity == 0` is just another refusal. A short buffer is the caller's defect and answers
       `FOC_ERR_PARAM`; the window's own refusals stay `FOC_ERR_STATE`, and both are reported: the buffer verdict wins
       the return code because it is the one the caller can fix without changing the physics, and `result.status` still
       names the window's verdict underneath.
     - `State` for fewer revolutions than configured, `|omega|` outside `[min, max]` either side, a window whose speed
       spread exceeds `speed_tolerance`, a saturated sample anywhere in it, and a destination with no room for the table;
       `Param` for order 0 (a constant torque is a load, not cogging), an inverted or non-finite band, a non-finite
       tolerance, and unusable machine parameters. Nothing was written in any of those cases.
     - ``CoggingLearn.TheLearnedTableCancelsTheInjectionItWasLearnedFrom``
     - na -- the accumulators are sums over a finite window that is thrown away on `reset()`, and what leaves the class is
       a table rather than a per-tick output.
   * - ``core/thermal_limit.hpp``
     - `ThermalBudget` separates "this current can be held forever" from "this current can be held for five more seconds"
       instead of reporting one number for both: `crosses == false` with `seconds` infinite is the sustainable answer, and
       collapsing the two is how a scheduler throttles a machine that was never in trouble. `ThermalLimitReport` reports
       `settle_at_cont_c` so the assumption behind the continuous current is visible rather than implied.
     - `Param` for `rt <= 0` (no cooling path), `rs <= 0` (no copper to heat), a negative `alpha` (a winding that cools as
       it heats is not a model), and a reserve that is not above ambient (no current can heat the winding down to it). A
       missing `Ct` is *not* an error: the continuous current does not need it, and only the budget then reports that it
       cannot name a time.
     - ``ThermalLimit.TheNaiveLawReadsFifteenPercentHighAndThePlantSaysSo``
     - na -- stateless: every answer is a closed form of the configuration and its arguments, and nothing is accumulated.
   * - ``core/watchdog.hpp``
     - `WatchdogState` separates the four answers a feeder can leave behind: `Feeding` (inside the window), `Expired` (the
       window passed), `ClockLost` (the stamp went backwards) and `Unarmed` (nothing measured yet). The last two are the
       point: `Unarmed` is reported as a `State` error and never as `Ok`, because a watchdog nobody feeds protects nothing,
       and `ClockLost` is a separate state from `Expired` because the verdict is the same (stop) and the diagnosis is not. A
       latched report repeats the numbers that caused the latch instead of growing with the clock.
     - `Param` for a zero window (it would trip on every feeder and stop a drive for a reason nobody can act on) and for a
       kick before any window was accepted -- and a refused window changes nothing, including a latch. `State` for a poll
       or kick before the first kick, and for a kick after a latch (it does not re-arm and is not counted as a feed).
       `Fault` once expired or clock-lost.
     - ``Watchdog.TheDeadlineIsExactForEveryWholeNumberOfControlPeriods``
     - na -- it holds one timestamp and one latch, neither of which is a continuous-output invariant; the latch is pinned
       by `Watchdog.AFeederThatComesBackDoesNotClearTheTrip`, which is a sequence rather than a per-tick property.
   * - ``core/auto_tune.hpp``
     - `SpeedTuneResult` separates "here are gains measured from this machine" from "I could not measure this machine":
       `valid` is the bit, and on a refusal the gains it carries are the **nameplate** ones, so a caller that applies
       whatever it gets applies nothing new. `reason` names which of seven refusals fired, because "spin with a varying
       current" and "something is pulling on the axis" are different actions. The fit keeps an **intercept** so a constant
       unknown load cannot be attributed to the current.
       **Reachable from C** (five `experimental` entries: `foc_autotune_create/accumulate/finish/reset/destroy`),
       so a C or Python consumer can run the procedure. A session is its own opaque handle rather than a motor register --
       nothing is readable tick by tick, only a window to feed and an answer to ask for -- and `finish()` answering
       `FOC_OK` with `valid == 0` is a refusal, not a failure.
     - `Param` for a non-positive `kt`, `j_expected`, `bandwidth` or `i_max`, a negative `b_expected`, a non-positive
       tolerance or residual bound, and a window shorter than three samples. `State`-shaped refusals arrive as
       `valid == false` with a reason: too short a window, a saturated period, a current that never varied, a residual above
       the bound, a negative mechanical pole, and a measured inertia outside the tolerance.
     - ``AutoTune.ADoubledInertiaIsRefusedAndTheGainsComeBackUntouched``
     - na -- the window is a finite sum that is thrown away on `reset()`, and what leaves the class is a `Pi` config rather
       than a per-tick output.
   * - ``comm/frame.hpp``
     - `FrameStatus` names which check failed (start / length / checksum) instead of one generic failure, and
       `SequenceChecker` reports `missing` / `duplicate` / `out_of_order` separately. `missing` counts the frames
       between the one expected and the one that arrived, so a single skipped frame reports 1.
     - `encode_frame` returns 0 and writes nothing when the frame does not fit; `decode_frame` refuses a short
       buffer, a late start byte and an over-long length before touching the receiver.
     - ``Frame.NoSingleByteCorruptionIsAccepted``
     - na -- the codec holds no state; `SequenceChecker` holds one byte and one flag, and neither is a
       continuous-output invariant.
   * - ``core/thermistor.hpp``
     - `NtcDivider::temperature_c` answers `false` at both rails and leaves the receiver untouched, because 0 and full
       scale are what an open and a shorted thermistor produce and both are ordinary numbers on the curve;
       `valid_config()` reports a divider that cannot exist before the first reading is attempted.
     - `valid_config()` is asked at construction by whoever builds the converter, and every conversion re-checks it, so
       a typo in `series_ohm` cannot silently produce a temperature.
     - ``Temperature.TheInverseUndoesTheForwardModelAcrossTheWorkingBand``
     - na -- a conversion is a pure function of one code and its configuration.
   * - ``comm/commissioned_set.hpp``
     - `SetStatus` names the check that failed (short / marker / schema / reserved / check value / field) and
       `SetError.offset` names the byte, so a reader learns which number to re-measure rather than which byte to stare
       at.
     - `encode_commissioned_set` returns 0 and writes nothing for a short buffer or an unusable set;
       `decode_commissioned_set` refuses a short buffer, a foreign marker, an unread schema, a non-zero reserved byte
       and a failed check value before touching the receiver, and leaves it untouched on every refusal.
     - ``CommissionedSet.EverySingleByteChangeIsRefused``
     - na -- a codec is a pure function of its buffer; the only state is the receiver, which is written once on
       success.
   * - ``comm/encoder_stream.hpp``
     - `EncoderFrame` separates "this frame arrived and is not trustworthy" (`State` plus `crc_ok == false`, with the counts
       still filled in so a log has something to show) from "this buffer cannot be read at all" (`Param`, with no value
       handed back). `TurnAssembler::Step` separates a wrap from a gap: `jumped == true` means the travel was unknowable, and
       then `unwrapped` does **not** move.
     - `Param` for a null buffer, a stream whose length is not `frame_bits()`, a layout with 0 single-turn bits, more than 31
       single-turn bits, more than 32 data bits, or negative multi-turn bits; and for an assembler whose single-turn bits are
       outside `1..31` or whose declared maximum step is 0 or above half a turn (above which the wrap question is
       undecidable). `State` for a CRC mismatch, a failed-CRC frame fed to the assembler, or a gap.
     - ``EncoderStream.TheLengthIsRefusedAndAShiftIsTheFramingLayersProblemNotTheCrc``
     - na -- the decoder is a pure function of one frame, and the assembler holds one previous count and one origin, neither of
       which is a continuous-output invariant.
   * - ``comm/register_map.hpp``
     - `read_register()` answers one 32-bit word per register; `RegisterKind` says whether to read it as an
       unsigned count, a signed code, or IEEE-754 bits.
     - a NULL receiver, an id with no row, and any register whose C function refuses all return `false`
       **with the word untouched** -- except the two count registers, whose C functions document NULL as 0.
     - ``RegisterMap.ARefusedReadLeavesTheWordAloneAndTheTwoKindsOfAnswerStayDifferent``
     - na -- read-only and idempotent; nothing here accumulates.
   * - ``core/trace_parse.hpp``
     - `missing` (inferred inside the window), `lost` (declared before it), `gaps`, `reversals`, `segments`, and
       `complete` only when nothing is missing either way. `consistent` is the parser's own check: `missing` is
       counted twice, by summing the per-pair skips and from the two endpoint timestamps.
     - `Param` for a zero period or a span whose samples pointer does not match its size; a window with no clock
       reports `timed == false` rather than a fabricated consistency.
     - ``TraceParse.AWrapIsOnePeriodAndNotAHole``
     - na -- a host-side tool, not a control-path component; it holds no state between calls.
   * - ``core/six_step.hpp``
     - `sector`, `changed`, `holding` and `skipped`, plus `rejected()`/`illegal()` counters. `holding` and
       `skipped` are different things and the duty alone cannot tell them apart: one means the pins named no
       position, the other means they named one the rotor cannot have reached.
     - `Param` for an amplitude outside `(0, 1]` or a step allowance outside `1..5` -- a table driven at zero
       amplitude produces no torque while looking like it is running.
     - ``SixStep.WalkingTheSectorsTurnsTheVectorSixtyDegreesEach``
     - na -- each call maps a code to a vector; the state it keeps is the last accepted sector, which is the
       boundary case above rather than a continuous-output invariant.
   * - ``core/command_source.hpp``
     - `analog`: `in_deadband`; `ppm`: `valid`; `step`: `pulses`, `delta_rad` and `jump`. A command source whose only
       output is a number cannot be told apart from a dead link, and `jump` is the difference between "the axis moved"
       and "the axis moved further than one period of motion allows, so a count was lost".
     - `Param` when the interface cannot be mapped exactly (`pulses_per_rev == 0`), when the analogue band is not below
       full scale, or when the pulse-width window is empty or inverted. An interface that cannot be mapped exactly is
       refused rather than approximated, because the approximation is a position error that never goes away.
     - ``CommandSource.AWholeTurnIsExactlyAWholeTurn``
     - na -- each call is a pure map from its input to a command; there is no state to hold invariant across time.
   * - ``core/step_metrics.hpp``
     - ``status``, and it carries weight: ``Ok`` means a first-order decay was fitted, ``State`` means the window carried
       none -- and the settling numbers stay valid in that case, because a response can settle without being fittable.
       A bandwidth of 0 is impossible under ``Ok``, so zero cannot be mistaken for "not measured".
     - ``Param`` for a window that cannot describe a step: fewer than three samples, a non-positive ``dt_s``, a negative
       band, or a window that does not move at all (``y[0] == target``).
     - ``StepMetrics.AWindowWithoutADecayReportsStateRatherThanAConfidentNumber``
     - na -- it measures other capabilities' arithmetic and owns none of its own.
   * - ``core/sinusoid_fit.hpp``
     - ``status``: ``Ok`` for a conditioned fit, ``State`` when the window is shorter than one period of the frequency
       asked about -- where a number would be produced but would not be the tone's amplitude.
     - ``Param`` for fewer than six samples, a non-positive ``dt_s`` or frequency, or a frequency at or above Nyquist,
       where the projection aliases and reports the tone at a frequency the signal does not have.
     - ``BodeSweep.TheThreeDecibelPointIsTheBandwidthTheGainLawWasAskedFor``
     - na -- it measures other capabilities' arithmetic and owns none of its own.
   * - ``motor/torque_map.hpp``
     - ``status`` for the map and ``limited`` for every command. The second one is the point: a clipped command and an
       obeyed one are otherwise identical, and a caller who cannot tell them apart reports a saturating tensioner as
       holding.
     - ``Param`` when the nameplate produces no torque per amp (`kt <= 0`), has no pole pairs, or when the limits do not
       describe an interval; and ``Param`` from `torque_command` when the d-axis current cancels the torque constant and
       no current can deliver the request -- a refusal, because clamping would report a torque that cannot be produced.
     - ``TorqueMap.TheLimitsAreNarrowedToWhatTheInverterCanDeliver``
     - na -- algebraic, no state to keep invariant across frames or time scales.
   * - ``core/stall_start.hpp``
     - ``phase()`` and `limited`. "starting", "released" and "cannot start" are three states rather than one number, and
       `Failed` is a first-class answer: a load the drive cannot move is a fact about the machine, and a sequence that
       ramps forever instead of saying so leaves the operator watching a silent axis. Through `Motor` the same two
       facts are `stall_start_phase()` and `stall_start_torque_nm()`, and they have to be read together: the phase says
       whether the torque beside it is still the sequence's or the caller's again.
     - ``Param`` from `validate()`/`begin()` when the limit is not above the breakaway point -- a sequence whose ceiling
       is under the breakaway can only ever produce `Failed`, so accepting it would accept a recipe known in advance
       not to work -- and when the ramp, release speed or grace period is not positive. `Motor::begin_stall_start()`
       adds the admission the sequence cannot make for itself: `State` when the axis is not running or is still
       aligning, because a torque source has no business running during a forced angle. Both refusals leave the phase
       `Idle` rather than half-way into a ramp.
     - ``StallStart.AStuckAxisSaysFailedInsteadOfRampingForever``
     - na -- a sequencer over other capabilities, not an equation of its own.
   * - ``core/observer/observer_policy.hpp``
     - na -- concepts: it names what an observer must provide and provides nothing itself.
     - na -- a type either satisfies the concept or fails to compile, which is the strongest admission there is.
     - na -- there is no runtime behaviour to bound.
     - na -- there is no arithmetic here to be invariant about.
   * - ``core/observer/ideal_observer.hpp``
     - ``reset()``, ``set_truth()`` -- a test double told the answer; it has no failure mode to report.
     - na -- it accepts whatever truth it is given, which is the whole point of it.
     - na -- only the SIL harness uses it; the harness is not part of the shipped library.
     - na -- no arithmetic beyond assigning the truth it was handed.
   * - ``core/observer/back_emf_observer.hpp``
     - ``state()``, ``flux()`` -- and deliberately no ``locked()``: there is no channel that could withhold a verdict,
       so reporting one would be a fiction.
     - na -- parameters arrive through ``apply_params()``; there is no configuration to contradict.
     - todo -- standstill is the obvious limit: there is no signal there. No case pins it yet, and writing the row
       without the case is the folklore the boundaries gate exists to prevent.
     - na -- a flux integration, not an algebraic identity.
   * - ``core/observer/dual_inductance_observer.hpp``
     - ``state()``, ``flux()``, ``root_gap_rad()`` -- the last is a diagnostic and not an estimate: it says how far
       apart the two rotor positions consistent with the measured flux and current are.
     - ``FluxSource::Corrected`` is admitted only with ``correction_gain`` in (0, 1]; ``Integrator`` never reads the
       gain, so leaving it at zero is not refused for a value it does not use.
     - none -- measured, and the hole closes the other way round: the second root of the quadratic is never a rotor
       position (modulus 20 to 68 against 1 across `i_d` = -0.5 to -2.5 A, its angle pi away), so there is no physical
       branch for continuity to pick wrongly; and a seed no angle explains is *not* corrected -- 0.1128 rad after one
       step and 0.1128 after 200 -- which is why `seed_at()` takes an angle rather than a guess. See
       ``DualInductanceObserver.TheSecondRootIsNotARotorPositionAndAWrongSeedIsPermanent``.
     - na -- one complex square root and one quadratic, evaluated pointwise.
   * - ``core/observer/nonlinear_flux_observer.hpp``
     - ``locked()``, ``state()``, ``flux()``
     - na -- parameters arrive through ``apply_params()``; saliency enters the flux law, not a switch.
     - none -- the mode is admitted on facts about the machine, and its estimate's sensitivity to those facts is
       measured rather than assumed.
     - na -- a flux law with a saturation term, evaluated pointwise.
   * - ``core/observer/hfi_observer.hpp``
     - ``locked()``, ``error()``, ``hf_current_a()``, ``status()`` -- the error signal is the demodulated quantity, so
       "no measurement" and "zero error" are distinguishable numbers.
     - ``status() == ErrorCode::Param`` for a surface machine, an absent carrier, or a carrier the control rate cannot
       resolve; ``update()`` refuses the last one too, on the path that bypasses the bridge.
     - ``HfiObserver.ACarrierThisControlRateCannotResolveIsRefusedNotFrozen``
     - na -- a demodulation with a carrier; its algebra is the PLL's, which is stateful and nonlinear.
   * - ``core/observer/blended_observer.hpp``
     - ``locked()``, ``blend()`` -- the mix is readable, so "which observer is carrying the estimate right now" is a
       number rather than an inference.
     - ``ErrorCode::Param`` when the hand-over band is inverted or empty.
     - ``SilHfiEnvelope.AcquiringARotorAlreadyRunningNeedsASeed``
     - na -- a schedule over two stateful observers.
   * - ``core/observer/switchable_observer.hpp``
     - ``kind()``, ``locked()``, ``error()``, ``injection_ab()`` -- the slot reports which mode is really running and
       what that mode is measuring, so a refused request cannot masquerade as a switch.
     - ``hand_over_to()`` answers ``ErrorCode::State`` outside ``Run``, and ``ErrorCode::Param`` when the setup in force
       cannot support the requested mode; ``set_kind()`` is the compile-time-only path.
     - ``SwitchableObserver.TheSelectedObserversLockVerdictIsReachableThroughTheSlot``
     - na -- a dispatcher; the invariants belong to the modes it dispatches to.
   * - ``motor/motor.hpp``
     - ``status()`` (mode, fault, observer, locked), ``error()``, ``trace()`` -- the composed loop answers "is it
       running, what is it commutating on, what has it recorded", so a wrong angle is a named mode rather than a
       symptom. ``trace()`` is what makes the loop's own decisions auditable after the fact.
     - ``init()`` answers ``ErrorCode::Param`` on a contradictory configuration and ``set_observer()`` refuses a mode
       the setup in force cannot support -- on facts (no carrier, no saliency), not on a switch.
     - ``MotorInjection.ASensoredAxisInjectsWithoutSurrenderingTheAngle``
     - ScalingImpedanceAndVoltageTogetherChangesNothing, TheClampIsContinuousAndAJumpWouldShow

Twenty-three headers under ``core/`` and ``motor/`` are not listed yet: alignment, board_check, commissioning, deadtime,
feedforward, field_weakening, homing, machine_map, machine_map_text, machine_params, motion_profile, polarity, position,
position_reference, reference, sm, startup, svpwm, traits, types, version, and the two remaining ``motor/`` headers. They
are named in ``capability-scope.json`` so the count is visible, and so that a *new* header cannot join them silently.

.. _evidence-levels:

_`The four evidence levels`
---------------------------

Every quantitative statement on this site carries one of four levels, and the level is part of the statement
rather than a footnote: an L2 number is not a weaker L4 number, it is a **different kind of claim**. The
table below is the vocabulary. The worked example for each level is a claim row in the project
``README.md``, and it stays there -- copying one here would give that number a second home with a shelf
life, which is the failure this page's matrix exists to prevent.

.. list-table::
   :header-rows: 1
   :widths: 7 30 27 36

   * - Level
     - What it means
     - How it is checked
     - What it does not say
   * - **L1**
     - Asserted by construction
     - A compiler flag, a type, or a gate that fails the build
     - That the behaviour was ever observed at run time
   * - **L2**
     - Measured against the model in this repository, with the criterion written before the
       measurement and the number recorded
     - A unit test that prints the number and asserts the criterion
     - Anything about your machine: it is true of the plant in this repository
   * - **L3**
     - L2 plus a **measured sensitivity** -- how far the number moves when the model is wrong in the
       ways it is known to be wrong
     - ``test_sil_sensitivity.cpp``, plus a phase-variable cross-check of the model itself
     - That the model omits nothing that matters to you; it bounds only what is known to be omitted
   * - **L4**
     - Measured on physical hardware, with the condition recorded
     - not available in this repository
     - --

**No page here makes an L4 claim**: at the time of writing there is no motor, no inverter and no current
probe on this project. A row may state the minimum of the levels its inputs depend on and no more, and
where a page quotes a number it names the level it is quoting.

An evidence level answers "how well is this supported". What may be said *to a customer* about a
capability — the sentence, the tier behind it, and the industry it is said in — is
:doc:`compliance/index`.
