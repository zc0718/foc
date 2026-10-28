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

_`Quick start`
--------------

.. code-block:: python
   :caption: conanfile.py

   def requirements(self):
       self.requires("foc/0.1.0")

.. code-block:: cmake
   :caption: CMakeLists.txt

   find_package(foc REQUIRED)
   add_executable(app main.cpp)          # C users write main.c, the link rule is identical
   target_link_libraries(app PRIVATE foc::foc)

One rule to remember: link the aggregate target :code:`foc::foc`. It carries both
:code:`foc::foc_c` and :code:`foc::foc_cpp`, and the package metadata guarantees the order.

.. note::

   C consumers still link the C++ runtime. The C ABI is stable, the implementation is C++:
   set :code:`LINKER_LANGUAGE CXX` (or add one empty :code:`.cpp`), otherwise you will see
   :code:`undefined reference to __cxa_*`. This boundary is documented, not papered over.

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

_`Detailed settings`
--------------------

.. toctree::
   :caption: Using the library
   :numbered:

   Quick start <quickstart>
   What this library does not do <boundaries>
   What each capability offers <capabilities>
   Usage and verification <usage>
   Foundation corrections <foundation-log>
   Demos <demos/index>

.. toctree::
   :caption: Reference
   :numbered:

   Convention alignment <conventions>

_`License`
----------

This project is licensed under the Apache-2.0 License. See the LICENSE file for details.
