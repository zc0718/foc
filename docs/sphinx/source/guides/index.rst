_`Guides by task`
=================

The API reference answers "what does this function take". This section answers
the other question: **"I have a task; which pieces do I need, and how do they go
together."** One page per task, not per header.

Every page in this section has the same seven parts, so one you have read teaches you
where to look on the next one:

1. **The task** -- the question in the words a user would use.
2. **What you need** -- the headers, and what each one is for.
3. **A minimal program** -- compiled with this repository's own flags, in
   ``docs/doxygen/dox/demos/``.
4. **Where the numbers come from** -- which nameplate field or helper produces
   each constant, because a retyped gain is a second machine.
5. **How it goes wrong** -- the failure modes that compile cleanly.
6. **Evidence** -- which claim rows cover the page, and what the page does not
   claim.
7. **Figure** -- produced by ``docs/demos/make_figures.py``, never hand-drawn.

Two rules apply to every page and are worth stating once:

- **The listing *is* the file.** Part 3 is a ``literalinclude`` of the example in
  ``docs/doxygen/dox/demos/``, not a copy of it: what the page shows is the translation unit
  that produced part 7's figure, so the two cannot drift. The one compile command lives in
  :doc:`../demos/index` instead of being repeated on every page.
- **The evidence level is the minimum of the page's claim rows**, and the level
  itself lives in the claim table in the project ``README.md``. No page in this
  section repeats a level or a measured number; a number copied into a page is a
  second source of truth with a shelf life.
- **The figure shows what it shows.** Where a page's headline number is not
  visible in the figure, the page says so rather than choosing a figure that
  looks like it agrees.

The nine pages
--------------

.. list-table::
   :header-rows: 1
   :widths: 44 40 16

   * - The task
     - Pieces
     - Example
   * - Bring a sensorless machine up, and know when to trust the observer
     - ``alignment`` ``polarity`` ``startup`` ``stall_start``
       ``observer/blended_observer``
     - :doc:`startup-sensorless`
   * - Produce torque at zero and low speed, and know when this machine cannot
     - ``observer/hfi_observer`` ``observer/dual_inductance_observer``
     - :doc:`low-speed-injection`
   * - Turn a torque request into a dq current reference and close both axes
     - ``reference`` ``limits`` ``pi`` ``feedforward``
     - :doc:`torque-reference`
   * - Keep producing torque above base speed, and decide whether it is worth it
     - ``field_weakening`` ``mtpv`` ``machine_map``
     - :doc:`high-speed-field-weakening`
   * - Modulate the voltage: overmodulation, minimum pulse, six-step
     - ``svpwm`` ``min_pulse`` ``six_step``
     - :doc:`modulation-limits`
   * - Build a position loop, home the axis, stay inside soft limits
     - ``position`` ``position_reference`` ``motion_profile`` ``homing``
       ``soft_limit`` ``gear_ratio``
     - :doc:`positioning`
   * - Sample current accurately: dead time, one shunt, the sampling instant
     - ``deadtime`` ``single_shunt`` ``trace`` ``board_check``
     - :doc:`current-sampling`
   * - Know when it has failed, and where the parameters came from
     - ``protection`` ``watchdog`` ``thermal_limit`` ``brake_chopper``
       ``auto_tune`` ``commissioning``
     - :doc:`protection-and-derating`
   * - Consume the library from C, Python or a simulation master
     - ``api/foc_api.h`` ``comm/frame`` ``comm/register_map``
       ``comm/encoder_stream`` ``api/python_bindings.cpp``
     - :doc:`c-and-cpp-callers`

Every page in the list carries all seven parts and its example compiles, which
makes the list above the boundary of what this section promises rather than a
forecast: a task that is not in it is not covered here, and the pieces column
says what it would take to add it.

.. toctree::
   :maxdepth: 1
   :hidden:

   Bringing a sensorless machine up <startup-sensorless>
   Torque at zero speed <low-speed-injection>
   Torque below base speed <torque-reference>
   Above base speed <high-speed-field-weakening>
   High speed and modulation <modulation-limits>
   Consuming the library from outside C++ <c-and-cpp-callers>
   Positioning an axis <positioning>
   Sampling and the power stage <current-sampling>
   Health and protection <protection-and-derating>

The figures already on the site
-------------------------------

:doc:`../demos/index` holds the SIL traces that the figure step draws on. They
are scenario pages rather than task pages: a scenario says "watch the loop do
this", a task page says "here is what to wire on your machine".

Reading the pages in order
--------------------------

The nine pages below are independent on purpose, but a machine does not meet them
independently: it is read, checked, aligned, given a torque command, run past base
speed, positioned and tripped. :doc:`walkthrough` does exactly that, in that order,
on one nameplate, and states one number per stage — every one printed by a script
that page includes.
