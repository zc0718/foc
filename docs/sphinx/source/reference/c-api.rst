_`c-api`
========

The frozen C11 surface, in ``include/foc/api/foc_api.h``. It is not a second implementation: every
entry point runs the same core as the C++ templates, and :doc:`../guides/c-and-cpp-callers`
proves that two doors give the same numbers, tick by tick.

What "frozen" means here, concretely
------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 24 76

   * - Rule
     - Consequence for you
   * - Every fallible function returns ``foc_err_t`` and is ``FOC_WARN_UNUSED``
     - ignoring a status is a compiler warning, and the project's own build treats it as an
       error. ``(void)`` does **not** silence it on GCC; bind first (``foc_err_t e = …``) — the
       escape exists and is deliberately the more typing
   * - Handles are opaque and generation-checked
     - a stale handle answers ``FOC_ERR_PARAM`` instead of dereferencing freed memory. A
       use-after-free becomes a return value
   * - Structs carry ``struct_size`` and fields are only appended
     - an older caller keeps working; a newer field is *not read* rather than read out of
       somebody's stack. A field that cannot be left unset answers ``FOC_ERR_NOT_IMPL``
   * - The pool is static and small
     - ``FOC_ERR_NO_RESOURCE`` is a real answer, not a memory-exhaustion symptom. You are told
       when the last slot is in use
   * - ``_f64`` variants exist for the commands
     - a ``double`` command from a script keeps its bits; the ``float`` form is the one an
       embedded caller wants

Status codes
------------

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Code
     - Meaning, and what to do about it
   * - ``FOC_OK``
     - done
   * - ``FOC_ERR_PARAM``
     - a null argument, an uncovered ``struct_size``, an unconfigured policy, or a request the
       machine cannot satisfy. Fix the caller
   * - ``FOC_ERR_NO_RESOURCE``
     - the handle pool is exhausted. Release something
   * - ``FOC_ERR_STATE``
     - the call does not apply to this state (command before ``enable``, for instance). Read the
       state machine, not the code
   * - ``FOC_ERR_NOT_IMPL``
     - this host cannot answer that — a missing callback, or a field this build does not read.
       *Not* a caller error, and the distinction is deliberate
   * - ``FOC_ERR_FAULT``
     - a latched fault is blocking the call. Diagnose, then clear

Motor handles
-------------

.. list-table::
   :header-rows: 1
   :widths: 44 56

   * - Functions
     - What they are for
   * - ``foc_motor_create``, ``foc_motor_create_ex``
     - a handle from a nameplate, with the setup struct covering limits, carrier and dead time
   * - ``foc_motor_create_host``
     - the same, with the HAL supplied as C callbacks (see :doc:`hal`) — the form scripted hosts
       and the Python module use
   * - ``foc_motor_get_setup``
     - what was **actually applied**, so a caller never has to guess which defaults it got: zero
       means "the library default"
   * - ``foc_motor_set_observer``
     - choose the observer at run time among the implemented kinds
   * - ``foc_motor_destroy``
     - release the slot; the handle goes stale rather than dangling

Enable, run, read
-----------------

.. list-table::
   :header-rows: 1
   :widths: 44 56

   * - Functions
     - What they are for
   * - ``foc_motor_enable``, ``foc_motor_disable``
     - output on and off. ``disable`` leaves the stage safe before it returns
   * - ``foc_motor_clear_fault``
     - the operator action — the C face of ``reset()``, which keeps the thermal memory
   * - ``foc_tick(motor, dt_s)``
     - one control period. The only function that must be called at a fixed rate
   * - ``foc_get_state``
     - speed, ``id``, ``iq`` — the three numbers a supervisor reads every period
   * - ``foc_motor_get_status``
     - the state machine, the fault vocabulary and the mode in one struct
   * - ``foc_motor_get_position_absolute``
     - a continuous mechanical angle, including the turns the encoder alone cannot see

Commands
--------

``foc_set_speed`` / ``foc_set_speed_f64``, ``foc_set_torque`` / ``foc_set_torque_f64``,
``foc_set_position`` / ``foc_set_position_f64``, and ``foc_set_command`` for the case where the
mode and the reference must change in the same period.

Breakaway, tracing and tuning sessions
--------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Session
     - Functions
   * - stall start
     - ``foc_motor_begin_stall_start``, ``foc_motor_get_stall_start``,
       ``foc_motor_cancel_stall_start``
   * - trace ring
     - ``foc_motor_set_trace``, ``foc_motor_trace_read``, ``foc_motor_trace_reset``
   * - speed-loop tuning
     - ``foc_autotune_create``, ``foc_autotune_accumulate``, ``foc_autotune_finish``,
       ``foc_autotune_reset``, ``foc_autotune_destroy``
   * - cogging learning
     - ``foc_cogging_learn_create``, ``foc_cogging_learn_accumulate``,
       ``foc_cogging_learn_ready``, ``foc_cogging_learn_finish``, ``foc_cogging_learn_reset``,
       ``foc_cogging_learn_destroy``
   * - protector
     - ``foc_protector_create``, ``foc_protector_step``, ``foc_protector_reset`` (with a cold
       flag), ``foc_protector_destroy``

The session shapes are the same in all of them — create, feed, ask, finish, destroy — and the
handles are the same kind: a slot with a generation, so a session that was not destroyed is
detectable rather than invisible.

Functions that need no motor
----------------------------

These take numbers and return numbers; they are the C face of the stages on :doc:`core`, and the
reason a script can plan a motion without owning a handle:

.. list-table::
   :header-rows: 1
   :widths: 40 60

   * - Function
     - Answers
   * - ``foc_mtpa_reference``
     - the ``(id, iq)`` pair for a torque request at a current ceiling
   * - ``foc_field_weakening_worthwhile``
     - whether this machine has anywhere to weaken into, and why
   * - ``foc_current_pi_gains``, ``foc_speed_pi_gains``
     - gains from the nameplate and a requested bandwidth
   * - ``foc_rs_estimate``, ``foc_flux_estimate``
     - the two standstill identifications
   * - ``foc_board_check``
     - the twelve board items and the one cause behind the failures
   * - ``foc_torque_map_build``, ``foc_torque_command``
     - the nameplate restated in newton-metres, and a command through it
   * - ``foc_step_metrics``
     - settling time, overshoot and steady-state error out of a recorded step
   * - ``foc_sinusoid_fit``
     - one tone out of a window, and how good the fit was
   * - ``foc_hfi_size``
     - the carrier amplitude and frequency this machine can resolve

Linking it
----------

The ABI is C, the implementation is C++. Set ``LINKER_LANGUAGE CXX`` (or add one empty ``.cpp``),
otherwise the link fails on symbols named ``__cxa_``. :doc:`../install` states this as a rule and
:ref:`U8 <honest-limits>` states the stronger version: a C consumer *without* the C++ runtime is
not supported.

Related
-------

- the C++ surface these call: :doc:`core`, :doc:`motor`
- the same experiment through both doors: :doc:`../guides/c-and-cpp-callers`
- the Python module, which binds this API and nothing else: :doc:`../quickstart`
