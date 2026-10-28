_`Quick start`
==============

From an empty directory to a running control loop, in four steps. The deep reference — all three
faces, the model, the verification matrix — lives in :doc:`usage`.

_`1. What you need`
-------------------

.. list-table::
   :header-rows: 1
   :widths: 22 78

   * - Item
     - Requirement
   * - Dependency manager
     - Conan 2.x (:code:`conan --version`)
   * - Build system
     - CMake; the exact floor is :code:`metadata.json` → :code:`cmake_version`
   * - Compiler
     - C++17 and C11 (GCC / Clang / MSVC)
   * - Library dependencies
     - :code:`etl` (fixed-capacity containers, header-only); :code:`GTest` for tests (host only);
       :code:`pybind11` for the Python module (host only, skipped on cross builds)

A consumer of the package does **not** need Sphinx or Doxygen — those build this documentation.

_`2. Consume the package`
-------------------------

.. code-block:: python
   :caption: conanfile.py

   def requirements(self):
       self.requires("foc/0.1.0")

.. code-block:: cmake
   :caption: CMakeLists.txt

   find_package(foc REQUIRED)
   add_executable(app main.cpp)          # C users write main.c, the link rule is identical
   target_link_libraries(app PRIVATE foc::foc)

There is exactly one link rule: the aggregate target :code:`foc::foc`. It carries both
:code:`foc::foc_c` and :code:`foc::foc_cpp`, and the package metadata guarantees the order.

.. warning::

   A C consumer still links the C++ runtime. The ABI is stable, the implementation is C++: set
   :code:`LINKER_LANGUAGE CXX` (or add one empty :code:`.cpp`), otherwise the link fails on
   :code:`__cxa_*`. This is a documented boundary, not an accident.

_`3. Pick a face`
-----------------

- **C++17** — board-level work, your own :code:`HalPolicy`, your own gains: see
  :ref:`the C++ path <cpp-control-step>`.
- **C11** — non-C++ projects and glue code over the frozen ABI: see
  :ref:`the C path <c-control-step>`.
- **Python** — host-side scripts on top of the C API: see
  :ref:`the Python path <python-control-step>`.

Each snippet on that page is complete enough to compile, and the C and Python faces run on the
stub HAL with library defaults (20 kHz, 500 ns dead time, 5 A limit, 24 V bus).

_`4. From zero to a servo, in one file`
---------------------------------------

The file below is the shortest complete example: a dq machine in Python behind the HAL callback protocol, the library's
own speed loop closed on it, one speed step, and an assertion on where the axis landed. It is not a copy of an example --
it *is* the example, and the test suite executes it, so this page cannot drift from working code:

.. literalinclude:: ../../../test_package/test/python/from_zero_to_servo.py
   :language: python
   :linenos:

Run it yourself with the module on the path:

.. code-block:: bash

   PYTHONPATH=$(dirname <the built foc_python*.so>) python3 test_package/test/python/from_zero_to_servo.py

It prints what the axis did. Measured on this machine: it lands 0.007 rad/s short of the 20 rad/s command, peaks 1.3 %
high, and stays inside a 0.14 rad/s band over the last 100 ms -- the residual being mechanical ripple rather than a
control error, which is one of the :doc:`documented limits <boundaries>`.

What it deliberately is *not*: a sensorless example (the plant reports a velocity, so the library takes the angle and
the speed from it), and a proof of accuracy on your machine. Both are covered where they belong -- the injection
observer and its boundaries, and the SIL tests that measure the loop against a plant.

_`5. Prove it yourself`
-----------------------

.. code-block:: bash

   # build the package and run every test (unit, SIL, stress, pure-C and Python consumers)
   conan create . -s build_type=Debug --build=missing

   # quality gates: clang-format, clang-tidy, layering, standalone headers
   python3 .github/misc/gates.py format
   python3 .github/misc/gates.py tidy
   python3 .github/misc/gates.py guard
   python3 .github/misc/gates.py strict

   # negative compilation: a fake HAL must NOT compile
   python3 .github/misc/negative_compile.py

   # coverage gate (reads the summary the recipe exports)
   python3 .github/misc/coverage_gate.py

   # bilingual documentation (needs sphinx and a loadable doxygen)
   python3 docs/build.py

What is deliberately **not** verified — dead-time compensation, ADC timing, EMI, ISR worst-case
time, flash/RAM footprint — is listed item by item under :ref:`honest limits <honest-limits>`.
