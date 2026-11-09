_`API index`
============

This page is the door to the API reference. The reference itself is the **Doxygen** site built
from the same sources: every public header, every file page, every group. It is not duplicated
here, because two copies of an API reference are two chances to be wrong.

How to find a symbol
--------------------

#. You know the task: start at :doc:`../guides/index`, which maps tasks to headers.
#. You know the header: open its file page in the reference (each page states what the file is).
#. You know a name: use the reference's own index (``Symbols``), which lists every type and
   function alphabetically.

The six groups
--------------

Everything public is in one of six groups. If a symbol is not in one of them, it is internal —
an anonymous-namespace helper or a detail header — and it is not part of the contract.

.. list-table::
   :header-rows: 1
   :widths: 16 40 44

   * - Group
     - What is in it
     - When you open it
   * - `Core <../../../../doxygen/build/en/v0.1.0/build_sub/html/group__group__core.html>`_
     - transforms, PI, limits, reference policies, observers, modulation, protection, motion
     - when you are choosing the algorithm, not the board
   * - `Motor <../../../../doxygen/build/en/v0.1.0/build_sub/html/group__group__motor.html>`_
     - ``Motor<...>`` and the policy composition that inlines the loop
     - when you are wiring the loop on a board with your own HAL
   * - `HAL <../../../../doxygen/build/en/v0.1.0/build_sub/html/group__group__hal.html>`_
     - the callbacks a board has to provide (current, angle, duties) and the stub
     - when you are porting: this is the only set of functions *you* implement
   * - `Driver <../../../../doxygen/build/en/v0.1.0/build_sub/html/group__group__driver.html>`_
     - encoder/hall handling, position and speed extraction
     - when the board's feedback is not a clean angle
   * - `Comm <../../../../doxygen/build/en/v0.1.0/build_sub/html/group__group__comm.html>`_
     - frames, register map, encoder stream
     - when the axis is driven from another device
   * - `C API <../../../../doxygen/build/en/v0.1.0/build_sub/html/group__group__c__api.html>`_
     - the frozen ABI: opaque handles, status codes, the servo façade
     - when you are not writing C++

Three faces, one implementation
-------------------------------

The same core is reachable three ways, and the reference keeps them separate on purpose:

.. list-table::
   :header-rows: 1
   :widths: 18 34 48

   * - Face
     - Entry point
     - Read this if
   * - C++17
     - ``foc/motor/motor.hpp`` — `Motor group <../../../../doxygen/build/en/v0.1.0/build_sub/html/group__group__motor.html>`_
     - you are on the board: own the gains, own the HAL, keep the loop inlined
   * - C11
     - ``foc/api/foc_api.h`` — `C API group <../../../../doxygen/build/en/v0.1.0/build_sub/html/group__group__c__api.html>`_
     - the surrounding project is not C++, or the consumer is glue code
   * - Python
     - ``import foc_python`` (host only)
     - you are scripting, logging or analysing — it binds the **C API**, not the templates

:doc:`../guides/c-and-cpp-callers` proves the first two doors agree: the same experiment through
the C handle path and the C++ template path, subtracted tick by tick.

Indexes worth bookmarking
-------------------------

- `Every file, with one line saying what it is <../../../../doxygen/build/en/v0.1.0/build_sub/html/files.html>`_
- `Every example, one page each <../../../../doxygen/build/en/v0.1.0/build_sub/html/examples.html>`_
- `The reference's front page <../../../../doxygen/build/en/v0.1.0/build_sub/html/index.html>`_

About these links
-----------------

The targets above point at the tree ``python docs/build.py`` produces, relative to this site:
``doxygen/build/en/v0.1.0/build_sub/html/``. Two things follow, and both are honest
consequences of a documentation pipeline that pins its output paths:

- the version directory is literal; after a version bump these links are one edit behind, and
  the gate that catches it is "the link 404s";
- the language is literal too — swap ``en`` for ``zh`` to land in the Chinese reference tree.

When the two sites are served from a common root (the repository root, which is what
``docs/build.py`` leaves behind), the links above work as written. Open the tree from anywhere
else and they will not, which is why this page is one hop and not the whole reference.
