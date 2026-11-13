_`Porting the library to a board`
================================

The task: the control law is chosen and the numbers are known; what is missing is
the code that touches *your* silicon. This page is the list of what the library
asks of a board, which of those it can live without, and what it does when one is
missing.

Read it before writing a single adapter, because the expensive mistake here is not
an adapter that fails to compile. It is an adapter that compiles -- a temperature
channel that returns zero when nothing is wired, a bus current that reports 0 A
because the sensor was never fitted -- and then a derating curve that looks
healthy on a machine that is cooking.

What you need
-------------

- ``foc/hal/*.hpp`` -- the ten contracts. Five are required and five are optional;
  the table below is the whole inventory, and the ``wired`` gate fails if a
  catalogued contract is missing from it or a row names a contract that does not
  exist.
- ``foc/core/traits.hpp`` -- the predicates that decide "is this member there".
  ``is_pwm_v``, ``is_clock_v``, ``is_temperature_v``, ``is_bus_current_v`` and
  ``is_mechanical_brake_v`` are compile-time questions, not runtime flags.
- ``foc/driver/stub_*.hpp`` -- a working implementation of every contract. Copy
  the shape from the stub rather than from memory: the predicate is the
  specification, and a member whose signature differs by one ``const`` is simply
  absent.

The ten contracts
-----------------

``required`` means the loop dereferences it: a missing member is a compile error,
not a degraded mode. ``optional`` means the member is detected, and its absence has
the behaviour written in the fourth column -- a behaviour you are choosing, so it is
worth reading before deciding you do not need the channel.

``hal/pwm.hpp``
   **required**. ``configure``, ``write_duty``, ``enable``, ``disable``, ``fault``.
   Under a driver that cannot switch, there is nothing for this library to do: the
   whole loop ends in a duty vector. Reference: ``driver/stub_pwm.hpp``.

``hal/current_sense.hpp``
   **required**. ``read_uvw``, ``calibrate_offset``, ``sample_rate_hz``. Note the
   third one: it is what lets a caller notice that its sampling instant does not fit
   the modulator's period, which is a compile-time-clean way to fly a wrong machine.
   Reference: ``driver/stub_current_sense.hpp``.

``hal/position.hpp``
   **required**. ``angle_mech_rad``, ``velocity_rad_s``, ``has_sensor``. A
   sensorless axis still implements it and answers ``has_sensor() == false``; the
   library then uses the observer. Reference: ``driver/sim_position.hpp``.

``hal/dc_link.hpp``
   **required**. ``voltage_v``. Cheaper than it looks -- the voltage is a slow signal
   and the loop reads it directly -- but it is read every tick, so a blocking ADC
   conversion here is a period spent in the wrong place. Reference:
   ``driver/stub_dc_link.hpp``.

``hal/fault_input.hpp``
   **required**, and the one contract the library may not default: "the power stage
   reports no fault" is not an answer a library can invent for a driver. An
   implementation that always answers "no fault" is a decision you are making
   explicitly. Reference: ``driver/stub_fault_input.hpp``.

``hal/clock.hpp``
   **optional**. Absent means the library measures each period from the ``dt`` you
   pass to ``tick`` and never reads a hardware timer, so your ``dt`` *is* the time
   base -- a fact worth knowing when a control loop drifts under load. Present means
   ``now_us`` and the trace timestamps come from the hardware clock. Reference:
   ``driver/callback_hal.hpp``.

``hal/temperature.hpp``
   **optional**. Absent means every conversion reports ``valid == false``, thermal
   derating reports "unavailable" and does not derate, and over-temperature never
   trips. That is honest and it is also unprotected: a machine that only ever sees
   the missing-channel path will run at full current until something else stops it.
   Reference: ``driver/stub_temperature.hpp``.

``read_bus_current_a`` (a member of *your* DC-link type, not of the contract)
   **optional**. ``foc/hal/dc_link.hpp`` deliberately does not carry it: a link fed
   by a bench supply has no return path to measure, and requiring the member would
   have ruled that hardware out of the library. A concrete link that has the member
   is asked ``is_bus_current_v``; absent means the current limiter is told "not
   measured" and relaxes at its configured rate instead of pretending zero.
   Reference: ``driver/stub_dc_link.hpp``.

``hal/mechanical_brake.hpp``
   **optional**. ``engage``, ``release`` and ``engaged`` -- the third one is in the
   shape on purpose: "I called engage" and "the shaft is held" are different
   sentences. Absent means the supervisor skips the brake step and never invents a
   state for a device that is not there. Reference:
   ``driver/stub_mechanical_brake.hpp``.

``hal/hall.hpp``
   **optional, and not really a port**. The decoder ships (``core/hall`` and
   ``core/six_step``); what is missing is a HAL input for three pins, so a host
   writes its own reader. Reference: ``driver/stub_hall.hpp``.

``hal/brake_switch.hpp``
   **optional**. ``core/brake_chopper`` decides; this contract is the switch it
   drives. Under a chopper that is a relay or a separate controller, you implement
   it; under one that is not fitted, you do not. Reference:
   ``driver/stub_brake_switch.hpp``.

Where the numbers come from
---------------------------

None of the ten says what a number *is*: they are the channels, not the values. The
values come from your nameplate or your own measurement, and the places they belong
are ``foc/motor/motor_config.hpp`` (the machine) and ``foc_motor_setup_t`` (the
drive). Two habits worth keeping:

- A gain recomputed at run time from a stored record is one machine; a gain retyped
  into a header is a second one, and the difference shows up as a current loop that
  behaves differently on the bench and in the field.
- A limit you have not measured is not a limit. Write the datasheet value with its
  source, or leave the feature off.

How it goes wrong
-----------------

- **Compiles, behaves as if the sensor is there.** A ``read_uvw`` that returns zeros,
  a temperature that returns 0 °C, a bus current that returns 0 A. Every one of them
  is a valid number to the library and a wrong one to the machine. Return the
  "invalid" shape the contract defines and let the consumer report it.
- **A signature off by one qualifier.** The predicate is exact: ``float
  velocity_rad_s()`` non-const, ``bool has_sensor() const``. A member that does not
  match is not "nearly there", it is absent -- and the failure is usually a confusing
  error about a missing member somewhere else.
- **A blocking adapter inside the tick.** ``voltage_v`` and ``read_uvw`` are called
  every period; a conversion that waits for a fresh sample spends the period budget
  and shows up as jitter, not as an error.
- **An optional channel that is silently switched off.** Derating that never runs
  looks exactly like a machine that never gets hot, until it does.

Evidence
--------

The contracts themselves are catalogued in ``.github/project/wired.json`` (class
``contract``, with the stub that shows each promise is implementable), and
``gates.py wired`` fails if this page and that catalogue disagree. What this page
does **not** claim: no adapter here has run on hardware. Every number in the tree
comes from the host and from the SIL plant; a board is where a port is proved, and
that procedure is the one you write, on your silicon.

.. note::

   The predicates live in ``foc/core/traits.hpp`` and are the specification of each
   member's signature. This page names the members so you know what you are writing;
   when the two differ, the predicate is right and this page is stale -- which is why
   the gate checks the list and not the signatures.
