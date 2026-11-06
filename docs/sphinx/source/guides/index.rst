_`Task slices`
==============

The API reference answers "what does this function take". This section answers
the other question: **"I have a task; which pieces do I need, and how do they go
together."** One page per task, not per header.

Every slice page has the same seven parts, so a page you have read once teaches
you where to look on the next one:

1. **The task** -- the question in the words a user would use.
2. **What you need** -- the headers, and what each one is for.
3. **A minimal program** -- compiled with this repository's own flags, in
   ``docs/doxygen/dox/demos/``.
4. **Where the numbers come from** -- which nameplate field or helper produces
   each constant, because a retyped gain is a second machine.
5. **How it goes wrong** -- the failure modes that compile cleanly.
6. **Evidence** -- which claim rows cover the slice, and what the slice does not
   claim.
7. **Figure** -- produced by ``docs/demos/make_figures.py``, never hand-drawn.

Two rules apply to every page and are worth stating once:

- **The evidence level is the minimum of the slice's claim rows**, and the level
  itself lives in the claim table in the project ``README.md``. No page in this
  section repeats a level or a measured number; a number copied into a page is a
  second source of truth with a shelf life.
- **The figure shows what it shows.** Where a slice's headline number is not
  visible in the figure, the page says so rather than choosing a figure that
  looks like it agrees.

The slices
----------

.. list-table::
   :header-rows: 1
   :widths: 6 30 40 12 12

   * - #
     - The task
     - Pieces
     - Example
     - State
   * - S1
     - Bring a sensorless machine up, and know when to trust the observer
     - ``alignment`` ``polarity`` ``startup`` ``stall_start``
       ``observer/blended_observer``
     - :doc:`slice-s1-startup`
     - written
   * - S2
     - Produce torque at zero and low speed, and know when this machine cannot
     - ``observer/hfi_observer`` ``observer/dual_inductance_observer``
     - planned
     - planned
   * - S3
     - Turn a torque request into a dq current reference and close both axes
     - ``reference`` ``limits`` ``pi`` ``feedforward``
     - :doc:`slice-s3-torque`
     - written
   * - S4
     - Keep producing torque above base speed, and decide whether it is worth it
     - ``field_weakening`` ``mtpv`` ``machine_map``
     - planned
     - planned
   * - S5
     - Modulate the voltage: overmodulation, minimum pulse, six-step
     - ``svpwm`` ``min_pulse`` ``six_step``
     - :doc:`slice-s5-modulation`
     - written
   * - S6
     - Build a position loop, home the axis, stay inside soft limits
     - ``position`` ``position_reference`` ``motion_profile`` ``homing``
       ``soft_limit`` ``gear_ratio``
     - :doc:`slice-s6-position`
     - written
   * - S7
     - Sample current accurately: dead time, one shunt, the sampling instant
     - ``deadtime`` ``single_shunt`` ``trace`` ``board_check``
     - planned
     - planned
   * - S8
     - Know when it has failed, and where the parameters came from
     - ``protection`` ``watchdog`` ``thermal_limit`` ``brake_chopper``
       ``auto_tune`` ``commissioning``
     - planned
     - planned
   * - S9
     - Consume the library from C, Python or a simulation master
     - ``api/foc_api.h`` ``comm/frame`` ``comm/register_map``
       ``comm/encoder_stream`` ``api/python_bindings.cpp``
     - :doc:`slice-s9-c-abi`
     - written

A slice is ``written`` when its page carries all seven parts and its example
compiles. The remaining four are listed with their pieces so the boundary of
what this section promises stays visible: a task page that does not exist yet is
a plan, not a gap to be discovered by a reader.

.. toctree::
   :maxdepth: 1
   :hidden:

   Bringing a sensorless machine up <slice-s1-startup>
   Torque below base speed <slice-s3-torque>
   High speed and modulation <slice-s5-modulation>
   Consuming the library from outside C++ <slice-s9-c-abi>
   Positioning an axis <slice-s6-position>

The figures already on the site
-------------------------------

:doc:`../demos/index` holds the SIL traces that the figure step draws on. They
are scenario pages rather than task pages: a scenario says "watch the loop do
this", a slice says "here is what to wire on your machine".
