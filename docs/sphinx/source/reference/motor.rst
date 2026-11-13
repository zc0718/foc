_`motor`
========

The instance that owns a control loop. It is one type, and the loop is a template: the observer,
the modulator, the regulator and the HAL are type parameters, so the whole tick inlines into
your translation unit and nothing in the control path is an indirect call.

The shape
---------

.. code-block:: cpp

   template <typename Observer, typename Modulator, typename Controller, typename HalPolicy,
             typename RefGenerator   = core::NullReferenceGenerator,
             typename FeedForward    = core::NullFeedforward,
             typename PositionControl = core::NullPositionControl,
             typename Aligner        = core::NullAlignment,
             typename Trace          = core::NullTrace>
   class Motor;

The four leading parameters have no default, because a drive that has not chosen an observer has
not chosen how it knows the angle. Everything after them is an inert default: present in the
type system, costing nothing, replaceable one at a time.

``Policies`` is how those parameters arrive at run time — one instance per control axis plus the
speed loop:

.. code-block:: cpp

   struct Policies {
       Observer observer;
       Modulator modulator;
       Controller current_d;
       Controller current_q;
       Controller speed;
       RefGenerator reference{};
       FeedForward feed_forward{};
       PositionControl position{};
       Aligner align{};
       /* Trace */ trace{};
   };

Two regulators for the current and one for the speed is not an accident of naming: the two
current axes have different plants once decoupling is in place, and a shared integrator would
make the d-axis disturbance visible on the q axis.

Lifecycle
---------

.. list-table::
   :header-rows: 1
   :widths: 30 26 44

   * - Call
     - Returns
     - What it means
   * - ``init()``
     - ``ErrorCode``
     - policies are checked and the HAL is validated; nothing has moved yet
   * - ``start()``
     - ``ErrorCode``
     - the state machine leaves ``Idle``: from here on, ``tick()`` is expected every period
   * - ``tick(dt_s)``
     - ``ErrorCode``
     - one control period: read, observe, regulate, modulate, write
   * - ``stop()``
     - ``ErrorCode``
     - back to ``Idle``; the output is safe before it returns
   * - ``fault()`` / ``fault(code)``
     - —
     - trip deliberately (the HAL or your supervisor saw something)
   * - ``clear_fault()``
     - ``ErrorCode``
     - the operator action; a latched fault does not clear because the cause went away

``tick()`` is the only function that must be called at a fixed rate, and it is the only one that
touches the HAL. Everything else may be called from a slower context.

Commands
--------

.. list-table::
   :header-rows: 1
   :widths: 30 34 36

   * - Call
     - Sets
     - Note
   * - ``set_torque(iq_a)``
     - a q-axis current reference directly
     - the reference policy is bypassed; ``id`` stays at what the policy says
   * - ``set_speed(rad_s)``
     - a mechanical speed reference
     - closed by the speed regulator, decimated by ``speed_loop_period_s``
   * - ``set_position(rad)``
     - a mechanical position reference
     - the outer loop; the speed reference it produces is what the speed loop sees
   * - ``set_command(const Command&)``
     - mode, reference and feed-forward in one call
     - the only one that can change the mode and the reference in the same period
   * - ``set_feedforward_current(iq_a)``
     - an additive iq term
     - set once and it stays; ``set_speed()`` deliberately does not clear it
   * - ``begin_stall_start(recipe)`` / ``cancel_stall_start()``
     - the breakaway sequence
     - the caller owns the axis again as soon as the sequence releases it

The one rule that costs integrators a day: after a start-up sequence releases the axis, the
caller must command something **in the same control period**, or the start-up torque keeps being
applied.

Readback
--------

- ``telemetry()`` — ``speed_mech_rad_s``, ``id_a``, ``iq_a`` and ``max_period_us``. That last
  field is the worst tick interval the target itself has seen: the number belongs to the chip,
  which is why it is measured here rather than published anywhere.
- ``state()`` — the state-machine state; ``status()`` — the first error since the last clear.
- ``mode()`` — which reference is in force.
- ``fault_code()`` — the fault in the library's own vocabulary.

Policies are reachable after construction — ``observer()``, ``reference()``,
``feed_forward()``, ``position()``, ``aligner()`` — because a drive that cannot ask its observer
what it thinks at run time cannot be commissioned.

Configuration
-------------

``MotorConfig`` is the machine plus the setup decisions that go with it: ``rs``, ``ld``, ``lq``,
``kt``, ``j``, ``b`` (the nameplate, in SI units), ``pole_pairs``, then the limits and timing that
belong to the *installation* — ``i_max_a``, ``pwm_freq_hz``, ``deadtime_ns``, and the decimation
periods of the two outer loops.

Two helpers turn that into gains without ever hiding a number:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Helper
     - What it derives, and from what
   * - ``current_pi_config(config, bandwidth_rad_s, v_bus)``
     - current-loop gains from ``rs``/``ld`` and the requested bandwidth; the output clamp comes
       from ``v_bus``, not from a constant
   * - ``speed_pi_config(config, bandwidth_rad_s)``
     - speed-loop gains from ``kt`` and ``j`` — the two numbers that decide how fast the axis can
       be, and therefore how fast it may be told to be
   * - ``position_p_config(config, bandwidth, velocity_limit, accel_limit)``
     - the outer position gain plus the two limits the profile needs
   * - ``psi_f_wb(config)``
     - the flux linkage behind ``kt``: ``kt / (1.5 * pole_pairs)``. Using ``kt`` directly in a
       voltage equation is dimensionally wrong

The whole tick, in one object
-----------------------------

``drive_supervisor.hpp`` is the assembly around the parts: one tick that reads the bus, runs the
protection, drives the chopper and only then lets the control loop have its turn. It exists because
the *order* is the claim — a protection evaluated after the loop has already commanded is a
protection that arrives one tick late — and because the status it returns has to name a cause:
the derated limit, the temperature that produced it (``NaN`` when no channel answered), the bus
voltage, the chopper's duty and the tick number.

Related
-------

- the interfaces a board must satisfy: :doc:`hal`
- reference implementations for benches: :doc:`driver`
- the loop assembled by hand, both doors: :doc:`../guides/c-and-cpp-callers`
