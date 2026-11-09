_`Using the API`
================

This section is the **contract**, organised by what you are doing rather than by file. Six groups,
one page each: what you call, what it gives back, and what it will not do for you.

What is promised here, and what is not
--------------------------------------

Every public declaration in this library is grouped — ``core``, ``motor``, ``hal``, ``driver``,
``comm`` and the C API — and this section documents each group at the level a user needs: the
entry points you call, the shape of the data they exchange, and the failure they leave you with.

It deliberately does **not** enumerate every declaration. A complete per-header listing is a
different artefact: the project builds it internally for review, and it is not published,
because an exhaustive dump of the surface is not documentation — it is a map that tells a reader
what exists without telling them what to do, and it tells a competitor where every seam is
without telling them anything about why. What is published is the part that answers "how do I
use this", page by page.

Two consequences, stated plainly:

- **Missing entry point = missing documentation.** If you need something this section does not
  mention, the honest reading is that the documentation owes you a page, not that the library
  lacks a seam. That is an issue against this section.
- **Names are stable, ordering is not.** The names on these pages are the contract (the C API's
  are frozen; the C++ ones are the ones the slices use). Anything else — helper namespaces,
  detail headers, file layout — is free to move, and no page promises otherwise.

The six groups
--------------

.. toctree::
   :maxdepth: 1
   :hidden:

   core
   motor
   hal
   driver
   comm
   c-api

.. list-table::
   :header-rows: 1
   :widths: 22 40 38

   * - Group
     - What is in it
     - Read it when
   * - :doc:`core`
     - frames and transforms, regulators, observers, reference policies, modulation, motion,
       protection, machine data
     - you are choosing an algorithm, not wiring a board
   * - :doc:`motor`
     - the motor instance, its configuration, telemetry and the command it accepts
     - you are assembling the loop on your own hardware
   * - :doc:`hal`
     - the eight interfaces a board must satisfy — this is the set *you* implement
     - you are porting to a new inverter
   * - :doc:`driver`
     - the reference implementations of those interfaces: callback and stub HALs
     - you are on a host, a test bench, or bridging plain C callbacks
   * - :doc:`comm`
     - frames, register map, encoder stream — the wire contracts
     - the axis is driven by another device
   * - :doc:`c-api`
     - the frozen C11 surface: handles, status codes, the servo façade
     - you are not writing C++

Three faces, one implementation
-------------------------------

.. list-table::
   :header-rows: 1
   :widths: 16 30 54

   * - Face
     - Entry point
     - What it is for
   * - C++17
     - ``foc::motor::Motor<Observer, Modulator, CurrentPi, Hal>``
     - the whole loop, inlined into your translation unit; you own the gains and the HAL
   * - C11
     - ``foc_motor_t`` handle + ``foc_*`` functions
     - a frozen ABI over the same core, for projects that are not C++ and for scripted tests
   * - Python
     - ``import foc_python`` (host only)
     - the same C surface behind pybind11 — scripting, logging, analysis

The three doors are not three implementations: :doc:`../guides/c-and-cpp-callers` runs the same
experiment through the C handle path and the C++ template path and subtracts them tick by tick.

How to find something
---------------------

.. list-table::
   :header-rows: 1
   :widths: 24 34 42

   * - You know
     - Go to
     - Because
   * - the task
     - :doc:`../guides/index`
     - one page per task: what to wire, what the numbers are, how it fails
   * - a name
     - :doc:`../symbols`
     - every symbol the documentation may use, with its unit and its frame
   * - a word
     - :doc:`../glossary`
     - defined once, with where it is used *in this library*
   * - a group
     - the table above
     - one page per group, entry points first

Naming rules worth knowing before you read any page
---------------------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 26 74

   * - Rule
     - What it means
   * - ``foc::core::X<Y>``
     - a policy: a template parameter, not a base class. The loop takes it by value and inlines
       it, which is why there are no virtual calls in the control path
   * - ``Config`` / ``Input`` / ``Output``
     - the aggregate a stage is configured with, fed, and gives back. All are
       aggregate-initialised, so ``{}`` is a valid input with documented meaning
   * - ``Null*``
     - an inert implementation of the same interface: a stage that is present in the type
       system and costs nothing
   * - ``foc_*``
     - the C ABI. Every function that can fail returns ``foc_err_t`` and is
       ``FOC_WARN_UNUSED`` — ignoring a status is a build failure by design
   * - ``foc_motor_t``
     - an opaque handle with a generation counter: a stale handle is *detected*, never
       dereferenced

What this section will not do
-----------------------------

- **It will not replace the slice page.** Entry points tell you what exists; the slice tells you
  the order, the numbers, and the way it goes wrong.
- **It will not list internals.** Anonymous-namespace helpers, detail headers and file layout are
  not part of the contract and are not documented here.
- **It will not promise cycle counts.** Those belong to a chip, not to this page — see
  :doc:`../cost-model`.
