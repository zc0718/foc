_`What each capability offers, and what it costs`
=================================================

The :ref:`boundaries page <What this library does not do>` says what this library cannot do. The README's claim table says
what is measured and how it is checked. Neither answers the question the person adding an algorithm actually has: *what
does a capability have to come with?*

This page is that answer, one row per capability. Six columns: the four things a new capability owes, plus the two links
that let the row be audited.

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
5. **Ledger row** -- when a capability forced a foundation correction, `foundation-log.rst` says which and what it cost.

The `capabilities` gate checks every cell: a header under `core/` or `motor/` must be listed here or named in
`capability-scope.json` as deliberately unlisted; every case name must exist as a `TEST()`; every ``none`` must have an
L3 claim; every reason must actually be a reason. A row that cannot be filled honestly fails the build, which is the
point -- the alternative is a library that grows algorithms and forgets how they fail.

.. list-table:: capability matrix
   :header-rows: 1
   :widths: 20 22 20 24 8 6

   * - Capability
     - Diagnostic channel
     - Admission
     - Boundary
     - Invariant
     - Ledger
   * - ``core/transforms.hpp``
     - na -- stateless: there is no state that could hold a stale angle, and the caller owns the angle.
     - na -- no configuration; a wrong argument is a wrong answer, which the invariance case catches.
     - none -- Clarke is blind to the zero sequence, which is stated and asserted rather than corrected; nothing
       else is known to break.
     - TheAmplitudeConventionHoldsAndTheZeroSequenceIsLost, ARotationAndItsFrameComposeExactly
     - --
   * - ``core/limits.hpp``
     - na -- stateless: a clamp with no memory has nothing to report about itself.
     - na -- no configuration; a limit wider than the signal is a caller decision, not a contradiction.
     - ``Invariants.TheClampIsContinuousAndAJumpWouldShow``
     - TheClampIsContinuousAndAJumpWouldShow
     - --
   * - ``core/pi.hpp``
     - ``status()``, ``integral()``, ``output()`` -- the integrator state is readable because a wound-up integrator is
       the failure mode that looks like a tuning problem.
     - ``status() == ErrorCode::Param`` on a negative gain, an inverted clamp or a non-positive ``dt_s``; the output
       holds instead of moving.
     - na -- the loop is linear and every limit on it is a clamp or a gain; the limits that matter are the plant's.
     - ElapsedTimeDecidesTheIntegralNotTheCallCount, TheClampIsContinuousAndAJumpWouldShow
     - --
   * - ``core/filters.hpp``
     - ``configured()``, ``size()`` -- an unconfigured filter says so instead of passing its input through untouched.
     - ``configure()`` refuses a non-positive cutoff or step.
     - na -- a first-order filter is exact for its stated cutoff; the limit that matters belongs to the loop
       around it, not to the filter.
     - na -- linear and time-invariant; the invariants that bite are on the loop around it, not on it.
     - --
   * - ``core/protection.hpp``
     - ``thermal_budget()``, ``derated_limit()``, ``fault_is_latched()`` -- "fine", "derating" and "stopped" are three
       readable numbers rather than one code. The split is by definition and not by accident: `output_allowed` is
       `active_mask == 0`, so a fault in this library *means* "authority removed", and derating -- the axis still
       running at reduced authority -- is announced by `i_limit_a` falling below the configured `i_max_a`.
     - na -- the configuration is a set of numbers; a nonsensical one is refused by the constructor that consumes it.
     - ``Protection.TheTemperatureLadderIsGradedRatherThanACliff``
     - na -- a supervisor over other capabilities, not an equation of its own.
     - --
   * - ``core/speed_from_angle.hpp``
     - ``ready()``, ``lost_count()`` -- differentiates an angle and says when the angle stopped arriving.
     - ``reset()`` is mandatory after a gap: the module will not publish a speed across one.
     - ``SilSpeedFromAngleClosedLoop.AWindowShortEnoughToBeOnTimeIsTheOnlyWayThisRouteWins``
     - ElapsedTimeDecidesTheIntegralNotTheCallCount
     - A-3
   * - ``core/trace.hpp``
     - ``size()``, ``dropped()``, ``latest()`` -- a full ring keeps recording and counts what it overwrote, so \"I lost
       the window\" is a number rather than an absence. ``t_us == 0`` on every sample means the platform has no clock,
       which is a fact about the HAL and not a claim that everything happened at once.
     - ``n/a`` -- nothing to configure: the capacity is the type, and reading past the retained window returns a zeroed
       sample instead of reaching out of bounds.
     - ``Trace.TheRecordAgreesWithTheScalarReadBitForBit``
     - na -- a recorder of other capabilities' arithmetic, with none of its own.
     - A-8
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
     - F-10, F-11, F-12, F-13, C-8
   * - ``core/step_metrics.hpp``
     - ``status``, and it carries weight: ``Ok`` means a first-order decay was fitted, ``State`` means the window carried
       none -- and the settling numbers stay valid in that case, because a response can settle without being fittable.
       A bandwidth of 0 is impossible under ``Ok``, so zero cannot be mistaken for "not measured".
     - ``Param`` for a window that cannot describe a step: fewer than three samples, a non-positive ``dt_s``, a negative
       band, or a window that does not move at all (``y[0] == target``).
     - ``StepMetrics.AWindowWithoutADecayReportsStateRatherThanAConfidentNumber``
     - na -- it measures other capabilities' arithmetic and owns none of its own.
     - F-8, F-9
   * - ``core/sinusoid_fit.hpp``
     - ``status``: ``Ok`` for a conditioned fit, ``State`` when the window is shorter than one period of the frequency
       asked about -- where a number would be produced but would not be the tone's amplitude.
     - ``Param`` for fewer than six samples, a non-positive ``dt_s`` or frequency, or a frequency at or above Nyquist,
       where the projection aliases and reports the tone at a frequency the signal does not have.
     - ``BodeSweep.TheThreeDecibelPointIsTheBandwidthTheGainLawWasAskedFor``
     - na -- it measures other capabilities' arithmetic and owns none of its own.
     - A-12
   * - ``motor/torque_map.hpp``
     - ``status`` for the map and ``limited`` for every command. The second one is the point: a clipped command and an
       obeyed one are otherwise identical, and a caller who cannot tell them apart reports a saturating tensioner as
       holding.
     - ``Param`` when the nameplate produces no torque per amp (`kt <= 0`), has no pole pairs, or when the limits do not
       describe an interval; and ``Param`` from `torque_command` when the d-axis current cancels the torque constant and
       no current can deliver the request -- a refusal, because clamping would report a torque that cannot be produced.
     - ``TorqueMap.TheLimitsAreNarrowedToWhatTheInverterCanDeliver``
     - na -- algebraic, no state to keep invariant across frames or time scales.
     - A-14
   * - ``core/stall_start.hpp``
     - ``phase()`` and `limited`. "starting", "released" and "cannot start" are three states rather than one number, and
       `Failed` is a first-class answer: a load the drive cannot move is a fact about the machine, and a sequence that
       ramps forever instead of saying so leaves the operator watching a silent axis.
     - ``Param`` from `validate()`/`begin()` when the limit is not above the breakaway point -- a sequence whose ceiling
       is under the breakaway can only ever produce `Failed`, so accepting it would accept a recipe known in advance
       not to work -- and when the ramp, release speed or grace period is not positive.
     - ``StallStart.AStuckAxisSaysFailedInsteadOfRampingForever``
     - na -- a sequencer over other capabilities, not an equation of its own.
     - C-5
   * - ``core/observer/observer_policy.hpp``
     - na -- concepts: it names what an observer must provide and provides nothing itself.
     - na -- a type either satisfies the concept or fails to compile, which is the strongest admission there is.
     - na -- there is no runtime behaviour to bound.
     - na -- there is no arithmetic here to be invariant about.
     - --
   * - ``core/observer/ideal_observer.hpp``
     - ``reset()``, ``set_truth()`` -- a test double told the answer; it has no failure mode to report.
     - na -- it accepts whatever truth it is given, which is the whole point of it.
     - na -- only the SIL harness uses it; the harness is not part of the shipped library.
     - na -- no arithmetic beyond assigning the truth it was handed.
     - --
   * - ``core/observer/back_emf_observer.hpp``
     - ``state()``, ``flux()`` -- and deliberately no ``locked()``: there is no channel that could withhold a verdict,
       so reporting one would be a fiction.
     - na -- parameters arrive through ``apply_params()``; there is no configuration to contradict.
     - todo -- standstill is the obvious limit: there is no signal there. No case pins it yet, and writing the row
       without the case is the folklore the boundaries gate exists to prevent.
     - na -- a flux integration, not an algebraic identity.
     - --
   * - ``core/observer/nonlinear_flux_observer.hpp``
     - ``locked()``, ``state()``, ``flux()``
     - na -- parameters arrive through ``apply_params()``; saliency enters the flux law, not a switch.
     - none -- the mode is admitted on facts about the machine, and its estimate's sensitivity to those facts is
       measured rather than assumed.
     - na -- a flux law with a saturation term, evaluated pointwise.
     - --
   * - ``core/observer/hfi_observer.hpp``
     - ``locked()``, ``error()``, ``hf_current_a()``, ``status()`` -- the error signal is the demodulated quantity, so
       "no measurement" and "zero error" are distinguishable numbers.
     - ``status() == ErrorCode::Param`` for a surface machine, an absent carrier, or a carrier the control rate cannot
       resolve; ``update()`` refuses the last one too, on the path that bypasses the bridge.
     - ``HfiObserver.ACarrierThisControlRateCannotResolveIsRefusedNotFrozen``
     - na -- a demodulation with a carrier; its algebra is the PLL's, which is stateful and nonlinear.
     - F-2
   * - ``core/observer/blended_observer.hpp``
     - ``locked()``, ``blend()`` -- the mix is readable, so "which observer is carrying the estimate right now" is a
       number rather than an inference.
     - ``ErrorCode::Param`` when the hand-over band is inverted or empty.
     - ``SilHfiEnvelope.AcquiringARotorAlreadyRunningNeedsASeed``
     - na -- a schedule over two stateful observers.
     - --
   * - ``core/observer/switchable_observer.hpp``
     - ``kind()``, ``locked()``, ``error()``, ``injection_ab()`` -- the slot reports which mode is really running and
       what that mode is measuring, so a refused request cannot masquerade as a switch.
     - ``hand_over_to()`` answers ``ErrorCode::State`` outside ``Run``, and ``ErrorCode::Param`` when the setup in force
       cannot support the requested mode; ``set_kind()`` is the compile-time-only path.
     - ``SwitchableObserver.TheSelectedObserversLockVerdictIsReachableThroughTheSlot``
     - na -- a dispatcher; the invariants belong to the modes it dispatches to.
     - F-1
   * - ``motor/motor.hpp``
     - ``status()`` (mode, fault, observer, locked), ``error()``, ``trace()`` -- the composed loop answers "is it
       running, what is it commutating on, what has it recorded", so a wrong angle is a named mode rather than a
       symptom. ``trace()`` is what makes the loop's own decisions auditable after the fact.
     - ``init()`` answers ``ErrorCode::Param`` on a contradictory configuration and ``set_observer()`` refuses a mode
       the setup in force cannot support -- on facts (no carrier, no saliency), not on a switch.
     - ``MotorInjection.ASensoredAxisInjectsWithoutSurrenderingTheAngle``
     - ScalingImpedanceAndVoltageTogetherChangesNothing, TheClampIsContinuousAndAJumpWouldShow
     - F-6, F-7

Twenty-three headers under ``core/`` and ``motor/`` are not listed yet: alignment, board_check, commissioning, deadtime,
feedforward, field_weakening, homing, machine_map, machine_map_text, machine_params, motion_profile, polarity, position,
position_reference, reference, sm, startup, svpwm, traits, types, version, and the two remaining ``motor/`` headers. They
are named in ``capability-scope.json`` so the count is visible, and so that a *new* header cannot join them silently.
