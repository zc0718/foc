_`driver`
=========

Implementations of :doc:`hal` that are not boards. Two families: the **callback HAL**, which
turns plain C functions into a policy so a host can implement the interface without writing C++,
and the **stubs**, which record instead of driving and let a test state what the loop saw.

Callback HAL
------------

- ``callback_hal.hpp`` — the five HAL operations a host supplies as plain C calls, and the policy
  that turns them into the interfaces the loop wants.

Use it when: the integration is C, or the code that reads the ADC lives in a vendor SDK whose
headers are C. The policy holds the callback table and the ``user`` pointer, so the loop's
indirect calls happen inside the HAL — one place, stated — rather than distributed through the
control code.

The stubs
---------

.. list-table::
   :header-rows: 1
   :widths: 34 66

   * - Header
     - What it does, and what a test uses it for
   * - ``stub_hal.hpp``
     - a complete HAL built from the rest: the one to instantiate when you want the loop to run
       and want to be able to lie to it
   * - ``stub_pwm.hpp``
     - records the duty instead of driving gates; the recording is the assertion
   * - ``scripted_current.hpp``
     - replays a scripted current sequence, so a test can say exactly what the sensor saw
   * - ``sim_position.hpp``
     - a position sensor the SIL plant writes into, so the loop reads an angle it did not invent
   * - ``stub_dc_link.hpp``
     - a bus voltage settable from the outside — a sag is a test, not a fault
   * - ``stub_fault_input.hpp``
     - reports exactly what it was told, and nothing else
   * - ``stub_hall.hpp``
     - a hall code settable from the outside, including the illegal ones
   * - ``stub_brake_switch.hpp``
     - records the brake duty instead of switching a resistor
   * - ``stub_temperature.hpp``
     - a temperature source a test sets, **including the state where the sensor is gone** — the
       invalid reading is the case a derating path has to survive, so it is one assignment away
   * - ``stub_mechanical_brake.hpp``
     - a holding brake a test drives, including the states where a call fails — a swallowed
       refusal is the failure this one exists to make reachable

The difference between ``scripted_current`` and ``sim_position`` is the difference between an
open and a closed loop, and it decides what a test can claim:

.. list-table::
   :header-rows: 1
   :widths: 30 34 36

   * - You are testing
     - Use
     - Because
   * - the modulator, the regulators, the policies
     - ``scripted_current``
     - the input is stated, so the output is a function of the code under test
   * - the closed loop
     - the SIL plant (``test_package/test/unit/sil_bench.hpp``)
     - the current has to *respond*; a script cannot be wrong in the way a plant can

The Python module and the C API run on the stub HAL. That is why they cannot produce a closed
loop on their own — the figures on :doc:`../demos/index` come from the C++ path with the plant,
and the page says so rather than showing a flat line.

What a stub is allowed to do that a HAL is not
----------------------------------------------

- **Lie deterministically.** A stub may return a value nobody measured; that is the point. The
  interfaces forbid it only because a real board has nothing to gain from it.
- **Be inspected.** Every stub exposes what it recorded. A policy never does: the loop's output
  is the trace, not the internals.

Related
-------

- the interface these implement: :doc:`hal`
- the plant a closed-loop test needs: :doc:`../guides/torque-reference`
- the C face that runs on stubs: :doc:`c-api`
