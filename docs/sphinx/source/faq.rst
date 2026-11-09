_`FAQ`
======

Eighteen questions that come back often, each answered with where the answer comes from. The
answers are short on purpose; the page that owns the topic is linked.

Linking and building
--------------------

**A C program fails to link, with an undefined C++ ABI symbol.**

The error names ``__cxa_`` symbols. The ABI is C, the implementation is C++: set
``LINKER_LANGUAGE CXX`` on the target (or add one empty ``.cpp``). This is documented rather
than worked around — see :ref:`U8 <honest-limits>` and :doc:`install`.

**Which target should I link?**

``foc::foc``. It carries both components, and the package metadata guarantees their order, so
``foc::foc_c`` on its own leaves the C++ symbols unresolved.

**The library will not compile with my warning set.**

The headers are built with the project's own ``-Wall -Wextra -Werror`` and there is a gate that
compiles each header standalone. If a stricter set of yours fails inside a header, that is worth
an issue: silencing it locally hides a real fragility.

**Why can't I use exceptions or RTTI in my policy?**

Because the whole translation unit is built ``-fno-exceptions -fno-rtti`` — the control path
must not unwind or ``dynamic_cast``. The reasons, and what it costs, are in
:doc:`cost-model`.

Current and torque
------------------

**The q-axis current overshoots on a step, and the integral term looks huge.**

Check anti-windup before the gains: while the modulator is saturated the integrator must stop
accumulating. The measured step on :doc:`../demos/index` has no overshoot at all, so an overshoot
is a difference from a known answer, not a matter of taste.

**The d-axis current is not zero even when I ask for zero torque.**

Two candidates, and they are different problems: the decoupling term
(``omega * Lq * iq``) is not being applied, or the angle is wrong so real d-axis current is
appearing in the q-axis reading. Torque at standstill with a wrong angle gives a characteristic
picture: torque, but less than commanded.

**Why does the default reference policy refuse to command a d-axis current?**

Because a d-axis current is only safe once somebody has stated how the angle is measured. On a
surface-mount machine the best use of the current ceiling *is* ``id = 0``, so the aggressive
choice is the one that needs an explicit opt-in.

**My torque request above the current ceiling comes back slightly low.**

That is the reference policy walking *down the MTPA curve* instead of scaling both components.
Scaling both would leave the curve and lose more torque than necessary. The behaviour is
asserted; see :doc:`../guides/torque-reference`.

Observer and speed
------------------

**The observer never locks at low speed.**

A back-EMF observer has nothing to observe when there is no back EMF. Below its working range the
options are: bring the machine up with :doc:`../guides/startup-sensorless`, use injection on a
salient machine (:doc:`../guides/low-speed-injection`), or fit a position sensor. The library
does not pretend the first two work on a surface-mount machine at standstill.

**The hand-over looks fine but the axis stalls immediately after it.**

``finished`` means the sequence ran, not that the rotor came with the vector. Seed the observer
with the angle **and** the speed the sequence ended at, and treat the hand-over speed as a floor.

**Why is my base speed lower than the flux over the voltage predicts?**

Because base speed is computed from the flux **at the MTPA point**, which on a salient machine is
several times lower than ``psi_f`` alone suggests. A drive sized on the second number discovers
the real ceiling when its current loop saturates.

Power stage and sampling
------------------------

**Overmodulation clamps at an amplitude I did not expect.**

The clamp is at the inscribed circle of the hexagon, at a stated amplitude, and the region
between the inscribed and the circumscribed circle is the overmodulation region. If you asked for
more than the circle allows, the answer is a clamp, not a scale — :doc:`../guides/modulation-limits`
shows the two clamp points.

**One-shunt sampling refuses some samples. What do I do with a refusal?**

Treat it as "the current is unknown this cycle" — hold the previous command or fall back to a
modulation state with a long enough window. What you must not do is use the number: a refused
sample is the library telling you it does not know.

**The measured ripple is much larger than the switching period predicts.**

The sample is landing off-centre of the zero vector. That is what the board check's
sampling-instant item measures, and :doc:`../guides/current-sampling` walks through it.

Protection
----------

**The drive trips, I clear the fault, and it trips again immediately.**

``reset()`` clears the latched fault and keeps the i2t memory, because the winding is still hot.
Use ``reset_cold()`` only for a machine that has been off long enough to have cooled. The three
"blocked" answers and their meanings are printed side by side at the end of
:doc:`../guides/walkthrough`.

**The derating curve allows current at the trip temperature — is the curve wrong?**

No: the curve and the trip are two different decisions. A controller that only reads the curve
never trips; one that only reads the trip runs at full current right up to it. You need both.

Python and tooling
------------------

**Importing the Python module fails.**

The bindings are host-only and skipped on cross builds. Check
``metadata.json`` → ``enable_python_bindings``, and put the directory containing the built module
on ``PYTHONPATH``. The module binds the **C API**, so it can do everything the C door can — and
nothing that needs the C++ templates.

**The documentation build fails on doxygen.**

Documentation tooling is separate from the library: it needs a loadable ``doxygen``, ``sphinx``
and ``graphviz``. A consumer never needs any of them.

Still stuck?
------------

- the API, per group: :doc:`reference/index`
- what the library does not do, item by item: :doc:`boundaries`
- where the numbers on any page come from: :doc:`../capabilities`
