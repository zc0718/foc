_`Demos`
========

Every figure below is produced by running this library — no mock-up, no hand-drawn curve. The
pipeline is:

#. :code:`docs/demos/sil_export.cpp` drives the SIL bench through one scenario and writes a trace
   and a metrics CSV;
#. :code:`docs/demos/make_figures.py` compiles it against the repository sources, runs it, and
   plots the result;
#. :code:`python docs/build.py` copies the figures into the site.

.. code-block:: bash

   source ~/venv/build/bin/activate        # matplotlib + numpy
   python3 docs/demos/make_figures.py      # regenerate figures and tables

The scenarios mirror the SIL acceptance suite (S1–S3), so what you see here is the same closed
loop the test suite asserts on.

.. toctree::
   Current step <current_step>
   Speed step <speed_step>
   Sine tracking <sine_tracking>

.. note::

   These traces come from the **C++** path with the SIL plant. The C API — and therefore the
   Python module — runs on the stub HAL, whose current feedback is a fixed script, so it cannot
   produce a closed loop on its own.
