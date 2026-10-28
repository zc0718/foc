_`What this library does not do`
======================================

Every row below is a limit that will cost somebody a day if they meet it in the field instead of here. The point of the
page is not humility: a control library whose boundaries are unstated gets used outside them, and the failure then looks
like a bug in the library rather than a fact about the domain.

This page is about what the *library* cannot do. It is a different list from :ref:`honest limits <honest-limits>`, which
is about what software verification cannot reach and is deferred to hardware -- dead-time, ADC timing, EMI, ISR worst
case. Both are needed: one is a property of the method, the other of the evidence.

Each row names the case that **pins** it, and the `boundaries` gate checks that every named case still exists. A
statement whose case was renamed or deleted therefore fails the build rather than quietly becoming folklore. That is the
only reason this page can be trusted after the person who wrote it has moved on.

.. list-table::
   :header-rows: 1
   :widths: 30 40 30

   * - What it cannot do
     - What that means for you
     - Pinned by
   * - **Resolve the magnet polarity.**
     - The injection observer converges to `dtheta = pi` as happily as to zero: the d axis is right and the magnet is
       backwards, so the torque sign is inverted. Telling them apart needs a separate pulse or saturation test, which is
       `MagnetPolarityTest` applied once at commissioning -- not something the observer does for you.
     - ``SilHfi.OutsideTheHalfTurnWindowItLocksToTheMirror``
   * - **Survive a wrong saliency sign.**
     - A machine with `Ld > Lq` needs `demod_gain_a` positive; the wrong sign does not give a worse estimate, it drives
       the estimate to the unstable equilibrium at `+-pi/2` and leaves it there.
     - ``SilHfi.AWrongSaliencyDirectionRunsAwayInsteadOfLocking``
   * - **Say "I do not know" through `ObsState`.**
     - The angle has no validity channel. An observer that is not measuring holds its last angle and reports zero
       speed -- indistinguishable from a stopped machine by those two numbers alone. Use `locked()` (or
       `status()["observer_locked"]` from Python) to tell them apart.
     - ``SwitchableObserver.TheSelectedObserversLockVerdictIsReachableThroughTheSlot``
   * - **Run an injection carrier the control rate cannot resolve.**
     - Past `hfi_max_inject_hz(control_hz)` the observer stops: the demodulated amplitude collapses, the lock drops out,
       the angle freezes, the speed reads zero, and a loop closing on it runs away (measured: -19.5 rad/s of speed
       bias). The request is refused at setup, and by `update()` for the paths that do not go through the bridge.
     - ``HfiObserver.ACarrierThisControlRateCannotResolveIsRefusedNotFrozen``
   * - **Use one carrier frequency at every control rate.**
     - The ceiling is a sampling fact, not a machine fact: 9200 Hz is refused at 20 kHz and accepted at 40 kHz. Sizing a
       carrier for one rate and reusing it at another is the mistake this row exists to prevent.
     - ``HfiObserver.TheCarrierCeilingScalesWithTheControlRate``
   * - **Run the injection observer on a surface machine.**
     - `predicted_gain_a()` is proportional to `(lq - ld)`, so with `lq == ld` the demodulated signal is identically
       zero and no follower can lock. The mode is refused rather than admitted and left to fail later.
     - ``ApiObserverSelector.TheHfiModeRefusesWithoutACarrierOrWithoutSaliency``
   * - **Acquire a rotor that is already turning.**
     - The loop pulls the angle only inside `+-pi/2` and has no way to pull a frequency it has never seen. A rotor handed
       to it mid-spin either gets caught or lands on the mirror, and which one is a property of the speed. A hand-over
       must seed it (`seed_angle`), which is what the blending observer does while the carrier is wound down.
     - ``SilHfiEnvelope.AcquiringARotorAlreadyRunningNeedsASeed``
   * - **Reduce the low-speed servo's residual ripple by improving the angle.**
     - The steady bias at 20 rad/s is 0.0039 rad/s (0.02 %); what remains is a ripple at the carrier frequency, and it is
       the injected current cross-coupling into torque on a turning machine -- linear in `inject_v` and in speed,
       indifferent to the saliency. Improving the angle by 13x does not move it. `inject_v` is the knob.
     - ``SilSpeedRipple.TheFigureSurvivesSubstepResolution``
   * - **Reconstruct what the hardware actually did from a trace.**
     - The trace records what the loop *believed*: the currents the PI acted on and the voltages it applied before
       modulation. A trace that looks perfect is therefore entirely consistent with a miscalibrated current sensor -- the
       loop is faithful to a wrong measurement. Use it to explain the loop's decisions, not to measure the phases.
     - ``Trace.TheRecordAgreesWithTheScalarReadBitForBit``
   * - **Improve a sensored axis's angle by adding the carrier.**
     - The sensor owns the commutation angle, so the injection observer now runs for its own demodulation only: the
       carrier does reach the inverter (measured swing 0.45--0.67 across eight rate/PWM combinations), but the sensor
       keeps the angle and the published speed. `status()["observer"]` naming `HFI` therefore does *not* mean the loop
       is commutating on the injected estimate -- compare the two explicitly if that is what you need.
     - ``MotorInjection.ASensoredAxisInjectsWithoutSurrenderingTheAngle``
   * - **Get a quiet speed estimate out of the injection observer for free.**
     - `omega_e = omega_int + kp*error`, and `error` carries the carrier, so the published speed holds 0.4180 rad/s at
       2000 Hz in a 20 rad/s servo unless `speed_lpf_hz` is set. The library's own path sets it (200 Hz); a caller that
       constructs the observer directly gets the raw value.
     - ``SilSpeedRipple.ThePublishedSpeedCanBeCleanedAndTheRotorDoesNotCare``
   * - **Close a servo on a ten-millisecond-old speed.**
     - Differencing the angle is quieter than the PLL and worse in a loop: at a 200 rad/s speed-loop bandwidth the window
       is transport delay in the feedback path, and no window length from 500 us to 10 ms wins on both settled accuracy
       and peak. The route is closed by measurement, not by taste.
     - ``SilSpeedFromAngleClosedLoop.AWindowShortEnoughToBeOnTimeIsTheOnlyWayThisRouteWins``

What is *not* on this page, because it is enforced by a gate rather than by a test case: no exceptions, no RTTI, no
`new`/`delete`, no `std::vector`/`std::string`, C++17 and C11 only (`strict`), every installed header carrying a README
claim row (`claims`), and every number in the documentation matching an artefact the build produced (`docs`).
