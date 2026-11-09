_`foc Project Documentation`
============================

_`Introduction`
---------------

:code:`foc` is a field-oriented-control library for three-phase PMSM drives. One engineering
constraint drives every design choice: **the control loop must be predictable** — no heap, no
exceptions, no virtual calls, no hidden allocation in the code path that runs every PWM period.

Three consumer faces share one implementation:

.. list-table:: One core, three faces
   :header-rows: 1
   :widths: 12 32 56

   * - Face
     - Header / module
     - Who it is for
   * - C++17
     - :code:`foc/motor/motor.hpp`
     - Board-level users: inject your own :code:`HalPolicy` and own every gain.
   * - C11
     - :code:`foc/api/foc_api.h`
     - Non-C++ projects, glue code and scripted tests: a frozen ABI over the same core.
   * - Python
     - :code:`import foc_python` (optional)
     - Host-side scripts and data analysis, built on the C API.

_`What V1 does`
---------------

- Clarke / Park and their inverses, space-vector modulation, PI controllers, a motor state
  machine and a back-EMF observer.
- **Policy-composed motor**: observer, modulator, controller and HAL are template parameters,
  so the whole current loop inlines into one translation unit.
- Simulation-grade HAL stubs (PWM duty history, scripted current, simulated position), so the
  complete control chain can be exercised without hardware.
- A C API with generation-checked opaque handles: a stale handle is *detected*, never
  dereferenced.
- An optional pybind11 module that binds the **C API**, not the C++ templates.

All of this is verified in software only — see :ref:`honest limits <honest-limits>`.

_`Where to start`
-----------------

The site is organised by **what you are trying to do**, not by what the project did. Four
doors, one per reader:

.. list-table::
   :header-rows: 1
   :widths: 26 34 40

   * - If you are…
     - Read
     - Because
   * - New to this library
     - :doc:`quickstart`
     - four steps from an empty directory to a running loop, ending in a real, executed example
   * - Holding a task
     - :doc:`guides/index`
     - one page per task: what to wire, what the numbers are, and how it goes wrong
   * - Deciding whether to use it
     - :doc:`architecture`, :doc:`boundaries`, :doc:`capabilities`
     - the shape, the limits, and what each capability owes before it may be sold
   * - Auditing a claim
     - :doc:`usage`, :doc:`compliance`, :doc:`foundation-log`
     - the claim rows, the language guarantees, and the record of what was wrong before

_`Project structure`
--------------------

::

   foc/
   ├── include/foc/     # public headers: core / hal / driver / motor / api
   ├── src/             # implementation, paired one-to-one with the headers
   ├── api/             # pybind11 bindings (compiled only when enabled)
   ├── test_package/    # Conan test package: unit, stress, pure-C and Python consumers
   ├── metadata.json    # single source of truth for the build switches
   └── docs/            # Doxygen + Sphinx pipeline (this site)

.. _honest-limits:

_`Honest limits`
----------------

V1 does **not** claim, and does not pretend to verify, the following. They are deferred to the
hardware stage. (For the limits that are *not* deferred -- the ones that are properties of the method and are pinned by
test cases -- see :doc:`what this library does not do <boundaries>`.)

.. list-table::
   :header-rows: 1
   :widths: 8 40 52

   * - #
     - Item
     - Why it cannot be verified yet
   * - U1
     - Dead-time compensation in practice
     - Needs the real inverter non-linearity.
   * - U2
     - ADC sampling aligned with PWM timing
     - Depends on the MCU timer and trigger chain.
   * - U3
     - Observer robustness against real parameter drift
     - The perturbation used in the SIL model is idealised.
   * - U4
     - EMI / EMC behaviour
     - Not observable in software.
   * - U5
     - Acoustic noise and vibration
     - Needs a mechanical load.
   * - U6
     - ISR worst-case execution time on the target MCU
     - No target profile is shipped.
   * - U7
     - Flash / RAM footprint against a chip budget
     - No target constraint to measure against.
   * - U8
     - A pure-C consumer **without** the C++ runtime
     - Would need a stripped link script and hand-written stubs; V1 explicitly requires C
       users to link the C++ runtime.

_`Contents`
-----------

Six chapters, in reading order. Nothing in the first three chapters is a changelog: the project's
own correction history sits in the last one, where a reader goes on purpose rather than by
accident.

.. toctree::
   :caption: Start here
   :numbered:

   Quick start <quickstart>

.. toctree::
   :caption: Guides
   :numbered:

   Task slices <guides/index>
   Measured responses <demos/index>
   Outer-loop self-tuning <sop>

.. toctree::
   :caption: Concepts
   :numbered:

   Architecture <architecture>
   Symbols and units <symbols>
   Convention alignment <conventions>

.. toctree::
   :caption: Evidence and limits
   :numbered:

   What this library does not do <boundaries>
   What each capability offers <capabilities>
   Usage and verification <usage>
   Language compliance <compliance>

.. toctree::
   :caption: Development and provenance
   :numbered:

   Foundation corrections <foundation-log>

_`License`
----------

This project is licensed under the Apache-2.0 License. See the LICENSE file for details.
