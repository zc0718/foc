_`hal`
======

This is the corner of the library **you** implement. Everything else is the library's; these are
the board's, and the library never guesses any of them.

Ten interfaces, one per physical fact
-------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 26 44 30

   * - Interface
     - The fact it carries
     - If you get it wrong
   * - ``pwm.hpp``
     - how a duty reaches the switches, and what the switch promises in return
     - the modulator is perfect and the output is not
   * - ``current_sense.hpp``
     - three phase currents, **including what to do while the shunt cannot see the current**
     - the loop closes on a number the hardware never measured
   * - ``position.hpp``
     - the rotor angle and velocity the observers are corrected against
     - every observer converges to the wrong angle, confidently
   * - ``dc_link.hpp``
     - the bus voltage the modulator scales its duties by
     - duties are right in per-unit and wrong in volts
   * - ``fault_input.hpp``
     - the power stage's one-bit fault output
     - a hardware trip is invisible until the next command
   * - ``hall.hpp``
     - the three hall sensors as one three-bit field
     - six-step commutation starts a sector late
   * - ``brake_switch.hpp``
     - the brake resistor's switch, whose safe state is **open**
     - a braking resistor stays connected at power-up
   * - ``clock.hpp``
     - the monotonic microsecond clock the loop is timed against
     - ``max_period_us`` reports something that is not the tick interval
   * - ``temperature.hpp``
     - the winding temperature a derating limit is derived from — and whether the reading is a
       measurement at all
     - a derating schedule that quietly assumes a temperature nobody measured
   * - ``mechanical_brake.hpp``
     - the shaft's holding brake: engage, release, and whether it is **actually** holding
     - the drive cuts the bridge believing the shaft is held, and the load drops

Three of them are optional — the clock, the temperature and the mechanical brake — and
``traits.hpp`` exposes ``has_temperature_v``, ``has_mechanical_brake_v`` and the clock probe so a
consumer can ask. An absent member means "there is nothing to sequence", never "assume the safe
value": the eight without a default are the ones where no honest default exists.

Every one of them is a template policy, not a virtual base: the motor takes the HAL as a type
parameter and calls it directly, so an interface that is implemented wrong is a compile error
(:doc:`core`'s ``traits.hpp``) rather than a jump through a null pointer at 20 kHz.

The C-callback form
-------------------

A host that is not ready to write a policy supplies plain C functions instead. Five are
required, and the rest change what the loop is allowed to know:

.. code-block:: c

   typedef struct foc_hal_callbacks_t {
       void  (*pwm_write_duty)(void* user, const float duty_uvw[3]);
       void  (*read_current)(void* user, float uvw_a[3]);
       float (*read_angle_mech_rad)(void* user);
       float (*read_voltage_v)(void* user);
       int32_t (*read_drive_fault)(void* user);
       float (*read_velocity_rad_s)(void* user);   /* optional */
       uint64_t (*now_us)(void* user);             /* optional */
       /* appended fields -- see struct_size */
   } foc_hal_callbacks_t;

The two optional callbacks are absent-by-meaning, not by accident:

- **no ``read_velocity_rad_s``** means the host cannot measure speed, and the library derives it
  from the angle instead. That is a different controller, not a degraded one;
- **no ``now_us``** means the caller trusts the ``dt`` it passes to ``foc_tick()``.

``struct_size`` is what makes appended fields visible at all: the library reads only what the
caller's struct covers, so an older caller keeps working and a newer field is *not read* rather
than read out of somebody's stack.

A missing ``read_drive_fault`` is answered with ``FOC_ERR_NOT_IMPL``, not ``FOC_ERR_PARAM``. The
two mean different things — "this host cannot tell you about faults" is not "you passed a bad
argument" — and the library refuses to let the first read like the second.

What the HAL is *not* asked to do
---------------------------------

- **It does not decide when to sample.** The trigger belongs to the timer chain; the HAL reports
  what the ADC saw, and :doc:`core` decides whether that is usable.
- **It does not convert units.** Currents in amps, angles in radians, voltages in volts. A HAL
  that scales is a HAL that hides a factor of two.
- **It does not allocate.** Every callback receives a ``user`` pointer chosen at construction;
  nothing in the interface can allocate, because nothing in the interface is called with a
  context the library owns.

Related
-------

- reference implementations for benches and tests: :doc:`driver`
- the first page a porting engineer should read: :doc:`../guides/startup-sensorless`
- the sampling decision the HAL only reports: :doc:`../guides/current-sampling`
