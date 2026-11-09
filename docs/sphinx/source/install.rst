_`Installing and toolchains`
============================

:doc:`quickstart` is the short path: four steps to a running loop. This page is the reference
behind step 1 and step 2 — the matrix, the three ways to consume the package, and the link
errors that mean you have it almost right.

Nothing on this page needs Sphinx or Doxygen: those build **this documentation**, not the
library.

What the build needs
--------------------

The floors are in ``metadata.json``, not in this page — the table repeats them so that a reader
can see the whole picture at once, and ``metadata.json`` stays the one place they are declared:

.. list-table::
   :header-rows: 1
   :widths: 26 20 54

   * - Tool
     - Floor
     - Notes
   * - Conan
     - 2.x
     - the package is built and consumed through Conan; ``conan --version`` is the check
   * - CMake
     - ``metadata.json`` → ``cmake_version``
     - a consumer's own CMake may be older; the floor applies to building the package
   * - C++ compiler
     - C++17
     - GCC, Clang or MSVC. The library builds with ``-fno-exceptions -fno-rtti`` — a compiler
       that cannot do that cannot build it
   * - C compiler
     - C11
     - only for the C ABI consumer; the implementation is still C++
   * - Cross toolchain
     - any
     - ``benchmark/profiles/cortex-a7.profile`` is the worked example: a Cortex-A Linux target,
       no Python bindings, no tests in the target graph

Library dependencies come in three buckets, and only one of them reaches a device build:

- **core** — ``etl`` (fixed-capacity containers, header-only). This is the only dependency on
  the bare-metal whitelist.
- **test** — ``GTest``. Host only.
- **bindings** — ``pybind11``. Host only, and skipped automatically on a cross build.

Three ways to consume it
------------------------

Two of them end at the same link line, which is the point: one aggregate target carries both
component targets and the package metadata guarantees their order.

.. list-table::
   :header-rows: 1
   :widths: 22 40 38

   * - You have
     - Do
     - Then
   * - Conan
     - ``self.requires("foc/0.1.0")`` in ``requirements()``
     - ``find_package(foc REQUIRED)`` + ``target_link_libraries(app PRIVATE foc::foc)``
   * - A prebuilt or locally installed package
     - put the Conan generator folder (or the install prefix) on ``CMAKE_PREFIX_PATH``
     - the same ``find_package`` and the same ``foc::foc`` target
   * - The repository itself
     - ``add_subdirectory(foc)``
     - the in-tree targets are ``foc_c``, ``foc_cpp`` and the aggregate ``foc``. The ``foc::``
       namespace is added by the **exported** package config, so it does not exist in this case

Link the aggregate target and not ``foc_c`` alone unless you know why: the aggregate is what
the package's own tests use, and it is the only one whose ordering is guaranteed.

The C consumer's link failure
-----------------------------

A C program can use the C ABI and still has to link the C++ runtime:

.. code-block:: cmake

   add_executable(app main.c)
   set_target_properties(app PROPERTIES LINKER_LANGUAGE CXX)   # without this: __cxa_*
   target_link_libraries(app PRIVATE foc::foc)

Without ``LINKER_LANGUAGE CXX`` (or one empty ``.cpp`` in the target) the link fails with
symbols named ``__cxa_`` — the C++ runtime's own entry points. That is not a packaging accident: the ABI is C, the
implementation is C++, and the library says so instead of hiding a C++ runtime behind a C
header. :ref:`The pure-C row in honest limits <honest-limits>` is the stronger version of the same
statement — a C consumer *without* the C++ runtime is explicitly not supported.

Common failures, and what each one means
----------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 42 58

   * - Symptom
     - What it is
   * - a link error naming ``__cxa_`` symbols
     - the C target is being linked as C; set ``LINKER_LANGUAGE CXX``
   * - ``foc::foc target not found``
     - the Conan generator folder is not on ``CMAKE_PREFIX_PATH``, or ``find_package`` was
       called before ``conan install`` ran
   * - ``import foc_python`` fails
     - the bindings are host-only and off on cross builds; check
       ``metadata.json`` → ``enable_python_bindings``, and put the built module's directory on
       ``PYTHONPATH``
   * - the library compiles but a policy does not
     - the motor is policy-composed: an observer or modulator that does not model the required
       concept fails at instantiation, with the concept name in the error
   * - ``-Werror`` fails inside the library headers
     - the headers are built with the project's own warning set; a consumer's stricter set can
       differ. Open an issue rather than silencing it — the headers are meant to be clean
   * - the doc build fails on ``doxygen``
     - documentation tooling is separate from the library; the site needs a loadable
       ``doxygen`` and ``sphinx`` but a consumer never does

Cross builds
------------

Two things change on a cross build, and both are by design:

- **the Python bindings are not built** — a target with no Python cannot load them, and
  pretending otherwise produces a module nobody can import;
- **the tests are not run on the target** — they are host tests over the same sources. What
  cannot be verified without hardware is listed under :ref:`honest limits <honest-limits>`.

The worked example is ``benchmark/profiles/cortex-a7.profile``; the same two switches are what
a Cortex-M bare-metal profile sets, with ``etl`` as the only dependency left in the graph.

Next
----

- the four-step path: :doc:`quickstart`
- what "predictable" costs, and what it does not cover: :doc:`cost-model`
- the API, one page per group: :doc:`reference/index`
