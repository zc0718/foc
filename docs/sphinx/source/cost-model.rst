_`What "predictable" costs`
===========================

The library's first sentence is that the control loop must be predictable. This page says what
that word buys, what it costs, and — more importantly — which parts of it are **measured** and
which are simply **not present**.

What the constraint excludes
----------------------------

Four exclusions, and each one is a decision with a price:

.. list-table::
   :header-rows: 1
   :widths: 20 30 50

   * - Excluded
     - Because
     - What it costs
   * - Heap allocation
     - ``malloc`` has a worst case nobody can bound on a target with a fragmented pool, and the
       tick runs 20 000 times a second
     - fixed capacities everywhere: ``etl`` vectors with a compile-time size, and a motor
       handle pool that can be exhausted (the C API says so instead of growing)
   * - Exceptions
     - ``throw`` in an ISR has no meaning, and the unwinder's cost is not a number the loop can
       carry; the library is built ``-fno-exceptions``
     - every fallible function returns a status, and callers must check it. The C API makes
       ignoring one a build failure
   * - RTTI
     - ``-fno-rtti``; nothing in the loop asks "what type is this" at run time
     - dispatch is a template parameter or an explicit function pointer, never a ``dynamic_cast``
   * - Virtual calls
     - an indirect call's cost depends on cache state, and "usually fast" is not a bound
     - the motor is policy-composed: observer, modulator, current controllers and the HAL are
       template parameters, and the whole loop inlines into one translation unit

The last row is the reason the *shape* of the library looks unusual (``Motor<Observer,
Modulator, PI, Hal>``): it is not a preference for templates, it is what remains once indirect
calls are excluded.

What the tick is made of
------------------------

One control period, in order:

#. read the HAL's current sample and the electrical angle (the HAL owns the ADC and the timer);
#. run the observer policy on those inputs;
#. run the reference policy (``d = 0``, MTPA, MTPV or a caller-supplied one) and the limits;
#. run the two current PI controllers plus the feed-forward and decoupling terms;
#. run the modulator, which produces three duties;
#. write the duties back through the HAL.

Which of those steps is worst depends on the policies chosen, so the library does not publish a
single cycle count: it publishes the property that makes a bound meaningful — **no step in that
list allocates, throws, or calls through a pointer it did not receive at construction**.

What is measured, and what is not
---------------------------------

.. list-table::
   :header-rows: 1
   :widths: 44 56

   * - Claim
     - Status
   * - the loop closes on a plant model, and the response is asserted per scenario
     - measured, in the SIL suite and on :doc:`../demos/index`
   * - the same experiment through the C door and the C++ door differs by nothing
     - measured, tick by tick, on :doc:`../guides/c-and-cpp-callers`
   * - no allocation in the control path
     - verified by tools that report allocations, and by the fact that the control path calls
       no allocator
   * - worst-case execution time on *your* MCU
     - **not measured**: no target profile is shipped. :ref:`U6 <honest-limits>`
   * - flash and RAM footprint against *your* budget
     - **not measured**: :ref:`U7 <honest-limits>`
   * - behaviour under an ISR that is itself interrupted
     - **not measured**: no target, no interrupt controller

The pattern is worth naming: the properties that follow from the *code* are verified; the
properties that require a *chip* are listed as limits. ``telemetry()`` reports
``max_period_us`` — the worst tick interval the target itself has seen — precisely because that
number belongs to the target, not to this page.

Why not simply measure it?
--------------------------

Because a number measured on one board is a claim about that board. The library ships no target
profile, so a cycle count from the author's machine would be read as a property of the library
and would be wrong on the next chip. What it ships instead is a shape in which the measurement
is *possible*: one inlined translation unit per policy set, no indirect calls, no allocator.

Where this shows up in the API
------------------------------

- ``foc_motor_create_*`` returns a handle or an error — never throws, never allocates beyond the
  static pool.
- Every ``foc_*`` function that can fail is ``FOC_WARN_UNUSED``; ignoring a status is a
  compiler warning turned into an error by the build.
- A stale handle is **detected** (generation counter), not dereferenced: the failure mode of a
  use-after-free is replaced by a return code.
- ``Config`` structs are aggregate-initialised, so a zeroed configuration is always a valid
  input, with documented meaning ("zero means the library default").

Related
-------

- the boundaries that are *not* deferred to hardware: :doc:`boundaries`
- what each capability owes before it may be claimed: :doc:`capabilities`
- the honest limits table: :ref:`honest limits <honest-limits>`
