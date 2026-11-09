_`core`
=======

The algorithms. Nothing in this group touches a pin: every stage takes numbers in, returns
numbers out, and is written as a policy so the loop can inline it.

Reading this page
-----------------

Each section below is one job the core does. The bullet lists name the headers — the name is the
entry point you include; the sentence is what the header is for. Where a stage is something you
call in a loop rather than a value you construct, the call sequence is spelled out.

Frames, types and clamps
------------------------

- ``types.hpp`` — the error codes and clock helpers shared by the C++ path and the C API.
- ``transforms.hpp`` — Clarke and Park and their inverses, in the amplitude-invariant convention
  the rest of the library assumes.
- ``limits.hpp`` — the clamps every stage shares: a symmetric clamp and a slew limiter with
  explicit rates.
- ``traits.hpp`` — compile-time checks that a policy really implements the interfaces the loop
  calls, so a wrong policy is a diagnostic at instantiation, not a crash at 20 kHz.
- ``version.hpp`` — the version query the core answers, from the string the C API reports.

Regulators and the current-loop furniture
-----------------------------------------

- ``pi.hpp`` — the proportional-integral regulator, with per-call gain scaling: how hard the
  drive resists, for this call only.
- ``feedforward.hpp`` — the feedforward stage's input, its null form, and the decoupling
  implementation that consumes it.
- ``filters.hpp`` — the first-order low pass the loop is filtered with, and why one pole is
  enough.
- ``step_metrics.hpp`` — what a recorded step response actually did, in numbers a tuning decision
  can rest on (settling time, overshoot, steady-state error).

Reference policies
------------------

- ``reference.hpp`` — the reference generator's input, its null form, and the MTPA
  implementation. The null form commands ``id = 0``.
- ``mtpv.hpp`` — the maximum-torque-per-volt point, and the two ceilings that bound a machine's
  operating point.
- ``field_weakening.hpp`` — whether field weakening is worth building for *this* machine, as
  advice, and the reference stage that acts on it.
- ``command_source.hpp`` — the command sources a drive can be driven by: step and analog, each
  with its own configuration and output.
- ``position_reference.hpp`` — a continuous mechanical angle built from an electrical estimate,
  with an index mark to fix its origin.
- ``friction_comp.hpp`` — the friction a drive is told to cancel, and the speeds that decide when
  the axis counts as at rest.
- ``cogging_comp.hpp`` — the cogging-torque compensator: one harmonic's coefficients, the input
  it reads, and what it adds to the reference.

Observers
---------

Every observer implements ``observer/observer_policy.hpp``: what it is given and what it has to
return. The implementations differ in what they can see, not in what they promise.

.. list-table::
   :header-rows: 1
   :widths: 34 26 40

   * - Header
     - Works from
     - Where it stops working
   * - ``observer/back_emf_observer.hpp``
     - voltage and current
     - below the speed at which back EMF is measurable
   * - ``observer/nonlinear_flux_observer.hpp``
     - the magnetisation map
     - machines with no map
   * - ``observer/dual_inductance_observer.hpp``
     - two current models
     - the region where neither model holds
   * - ``observer/hfi_observer.hpp``
     - an injected carrier and saliency
     - surface-mount machines, which have no saliency to read
   * - ``observer/blended_observer.hpp``
     - two estimates, shortest arc
     - never on its own: it blends what it is given
   * - ``observer/switchable_observer.hpp``
     - the run-time choice among the above
     - when nobody states which one is active
   * - ``observer/ideal_observer.hpp``
     - the plant's true angle
     - SIL only, by design

Machine data
------------

- ``machine_params.hpp`` — the electrical facts about a machine that its algorithms need, in SI
  units.
- ``machine_map.hpp`` — the magnetisation map: rows of current, inductance and flux that the
  observer interpolates.
- ``machine_map_text.hpp`` — reads a machine document from text, and says precisely why when it
  cannot. Unknown keys are counted and ignored on purpose.
- ``thermal_limit.hpp`` — the thermal facts a current limit is derived from, and the limit
  itself.

Modulation and the power stage
------------------------------

- ``svpwm.hpp`` — the modulator: one evaluation's result, the input it is given, and the SVPWM
  that produces both.
- ``min_pulse.hpp`` — the gate driver's own pulse-width limit, and the duty that survives once a
  narrow pulse is dropped.
- ``six_step.hpp`` — what a six-step commutation needs to know, and what it decides each period.
- ``single_shunt.hpp`` — what a single shunt can measure in one period, and the reconstruction
  built on that answer. A refused sample is an answer, not an error.
- ``deadtime.hpp`` — the dead-time estimate a compensator needs, and how it fades as the current
  leaves zero.
- ``hall.hpp`` — the six-step direction table, and the tracker that holds a direction across a
  sensor code change.
- ``brake_chopper.hpp`` — the brake chopper's duties and the state it falls back to.

Start-up and commissioning
--------------------------

- ``startup.hpp`` — the start-up sequence as a state machine: its states, configuration and the
  routine that advances them.
- ``alignment.hpp`` — the rotor-alignment sequence as a state machine, and its null form.
- ``polarity.hpp`` — the magnet-polarity pulse test: what a pulse pair concluded, and the test
  that runs it.
- ``stall_start.hpp`` — the breakaway sequence: its phases, configuration and the output of one
  step.
- ``commissioning.hpp`` — the standstill estimators used to commission an axis: resistance,
  flux, current-sense offset and incremental inductance.
- ``auto_tune.hpp`` — the speed-loop tuner's contract: what it is asked for, what it reports,
  and what it is allowed to trust.
- ``cogging_learn.hpp`` — the cogging learner's contract: which orders, over how many
  revolutions, and what it measured when it finished.
- ``sinusoid_fit.hpp`` — a single tone measured out of a recorded window, and the quality of the
  fit it came from.

Motion and position
-------------------

- ``motion_profile.hpp`` — the profile that shapes a velocity command into something the
  mechanics can follow.
- ``position.hpp`` — the position slot's input, its null form, and the proportional position
  controller.
- ``soft_limit.hpp`` — the window an axis is allowed to occupy, and how the command is bent as
  it reaches the edge. An out-of-range target comes back as the **previous** target.
- ``homing.hpp`` — the homing search: its states, failures, configuration and the routine that
  advances them.
- ``gear_ratio.hpp`` — electronic gearing: the ratio between motor and load, and where the output
  angle is folded.
- ``speed_from_angle.hpp`` — mechanical speed from an angle, by differencing it across a fixed
  delay.

Health, faults and records
--------------------------

- ``sm.hpp`` — the motor state machine: the states an instance may be in, and the transitions
  between them.
- ``protection.hpp`` — the fault vocabulary and the protector that chooses between them: one
  code per physical mechanism, and a latch that survives the cause.
- ``watchdog.hpp`` — the one number a watchdog is sized by: how long the feeder may stay silent,
  and what that made it conclude.
- ``board_check.hpp`` — board checks as numbers: what was measured, what the limits are, and what
  each item concluded — then one cause, not a list of failures.
- ``trace.hpp`` — the trace sink interface: one sample at a time, into a ring that may be the
  null sink.
- ``trace_parse.hpp`` — a window of trace samples as the host got it out of the ring, and the
  samples that window lost.

The stages the slices use
-------------------------

.. list-table::
   :header-rows: 1
   :widths: 34 62

   * - You are doing
     - Call
   * - sensorless start-up
     - ``StartupConfig`` → ``StartupRoutine::begin(pole_pairs)`` → ``step(dt, measured_rad)``
       until ``finished``
   * - deciding whether to weaken
     - ``MtpvConfig`` → ``Mtpv::validate()`` → ``base_speed_rad_s()`` / ``at_speed(omega_e)``
   * - a torque command
     - ``ReferenceInput`` → ``MtpaReference`` (or ``NullReferenceGenerator`` for ``id = 0``)
   * - a move
     - ``SoftLimit::apply(target)`` → ``MotionProfile::step(target, feedback, dt)`` → speed
       command
   * - overload and trip
     - ``Protector::step(input, dt)`` → read ``fault``; ``derated_limit(temperature_c)``;
       ``reset()`` or ``reset_cold()``
   * - a board self-test
     - ``BoardMeasurements`` + ``BoardLimits`` → ``check_board()`` → ``BoardReport``

Related
-------

- the tasks that use these, in order: :doc:`../guides/index`
- what a policy costs at run time: :doc:`../cost-model`
- the C face of the same stages: :doc:`c-api`
