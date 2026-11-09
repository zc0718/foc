_`Glossary`
===========

Thirty-four words this documentation uses without re-defining them, in the order a reader meets them.
Every entry says where it is used *in this library*, because the same word means slightly
different things in different drives — and a symbol, once it is used in an equation, must be
the one on :doc:`symbols`.

Machines and frames
-------------------

.. glossary::

   PMSM
      Permanent-magnet synchronous machine. Surface-mount (``Ld ≈ Lq``) or interior
      (``Lq > Ld``). The difference decides whether injection and reluctance torque exist at
      all; the nameplate's ``Ld``/``Lq`` is where this page's claims start.

   pole pairs
      Electrical revolutions per mechanical revolution. Every "electrical" number in this
      documentation is a mechanical number times this count — hand-over speeds, base speed and
      observer bandwidths included. Halving or doubling it silently moves every one of them.

   dq frame
      The rotor-aligned frame: ``d`` along the magnet flux, ``q`` in quadrature. A quantity
      that is not in this frame is ambiguous in this library, so the frame is part of the
      symbol, not of the context.

   Clarke / Park
      Frame changes: Clarke takes the three phase currents to a stationary two-axis frame (αβ),
      Park rotates that into the rotor frame (dq). Built as ``clarke``/``park`` and their
      inverses in ``include/foc/core/transforms.hpp``.

Control
-------

.. glossary::

   FOC
      Field-oriented control: control current in the rotor frame so torque and flux become
      separate, one-dimensional problems.

   MTPA
      Maximum torque per ampere: the ``(id, iq)`` combination that produces the most torque
      inside a current ceiling. On a surface-mount machine it collapses to ``id = 0``, which is
      why the default reference policy refuses ``id`` until the caller says otherwise.

   MTPV
      Maximum torque per volt: above base speed the voltage ellipse, not the current ceiling,
      is what binds, and the best use of the remaining voltage is a curve, not a point.

   field weakening
      Deliberately commanding negative ``id`` to reduce the flux the inverter has to overcome,
      trading torque per amp for speed. The quantity that decides whether it is possible at all
      is the direction of ``lambda = Ld * i_max / psi_f``: above 1 there is headroom, below 1
      there is not.

   base speed
      The speed at which the machine can no longer hold its MTPA point within the voltage
      ceiling. Computed from the flux **at** the MTPA point, not from ``psi_f`` alone.

   characteristic current
      ``psi_f / Ld``: the current that would cancel the magnet flux entirely. It is the
      asymptote field weakening runs towards, and the ceiling that decides whether it is worth
      running towards at all.

   HFI
      High-frequency injection: an observer that injects a small carrier and reads the machine's
      saliency to learn the angle at standstill, where a back-EMF observer has nothing to work
      with. A surface-mount machine gives it no signal — that is a property of the machine, not
      of the observer.

   back-EMF observer
      Estimates the angle and the speed from the voltage equation's back-EMF term. Needs speed:
      it has nothing to observe at zero speed, which is what makes the hand-over speed a floor.

   anti-windup
      What stops a PI integrator from accumulating while its output is saturated. Without it a
      current step comes back with an overshoot that looks like a tuning problem.

   feed-forward / decoupling
      Terms added to the voltage so the two current axes stop disturbing each other:
      ``omega * Lq * iq`` on the d axis and ``omega * (Ld * id + psi_f)`` on the q axis.

   dead time
      The interval both switches of a half-bridge are off, so a leg never shorts the bus.
      Necessary, and it corrupts the commanded voltage near zero current — the corruption is a
      function of current *sign*, which is why a compensation that ignores the sign can make
      things worse than none.

   single shunt
      Sensing the bus current through one shunt and reconstructing three phase currents from the
      switching state. Some modulation states leave the window too short to sample, and the
      library **refuses** those samples rather than returning a plausible number.

Power stage
-----------

.. glossary::

   SVPWM
      Space-vector pulse-width modulation: chooses the two adjacent active vectors and the zero
      vectors so the average voltage vector matches the request.

   overmodulation
      Asking for a vector outside the inscribed circle. The library clamps at a stated
      amplitude rather than silently scaling, and the region between the inscribed and the
      circumscribed circle is where linear modulation ends.

   six-step
      The limit of overmodulation: the inverter is switched as a six-step commutator and the
      amplitude is no longer controllable. ``include/foc/core/six_step.hpp`` is the policy that
      walks it.

   minimum pulse
      A pulse narrower than the gate driver can reproduce is not a pulse: it is either dropped
      or widened. Widening changes the volt-seconds, so the choice is a policy and it is
      documented.

   i2t
      The overload budget: ``current^2`` integrated over time, against a budget the continuous
      rating implies. Below the hard trip the question it answers is "how long", not "whether".

   derating
      The continuous current allowed as a function of temperature. A curve, not a threshold —
      and deliberately not the same number as the trip threshold.

   brake chopper
      A switch and a resistor that burn off energy the bus cannot absorb during regeneration.

Motion and safety
-----------------

.. glossary::

   soft limit
      A software wall that refuses an out-of-range target by returning the **previous** target
      with a status. Downstream never sees a position it would drive into the wall.

   homing
      Establishing the mechanical zero: a defined search, not an assumption about where the
      axis was at power-up.

   electronic gear
      A ratio between a master angle and a slave's position target, with the velocity the ratio
      implies. The velocity output is not a convenience: a position loop told only where to be
      lags a moving master.

   motion profile
      The shaping between a target and a velocity command: a gain, a velocity limit and an
      acceleration limit. It owns where the axis *should* be, not whether it is there.

   watchdog
      An external check that the loop is still running, with a window — early is as wrong as
      late, because the first means the loop is running at the wrong rate.

   latch
      A fault that does not clear because the cause went away. It clears on an operator action,
      and the thermal memory survives even that: the winding is still hot.

Tools and artefacts
-------------------

.. glossary::

   claim row
      One line of the table in the project ``README.md``: what is claimed, at which evidence
      level, and which test case measures it. A capability without a claim row is not a claim.

   evidence level
      How a statement is supported (measured on the SIL plant, asserted by a test, verified by a
      gate, or deferred to hardware). Pages may state **the minimum of the levels they depend
      on**, and no more.

   SIL
      Software-in-the-loop: the library's control loop running against a plant *model* on the
      host. What it can show is in :doc:`../demos/index`; what it cannot is
      :ref:`what is deferred to hardware <honest-limits>`.

   task page
      One page per task, in :doc:`../guides/index`. Not per header: a task page is what a reader
      is trying to do, and the headers are listed inside it.

   gate
      A check that fails a build: formatting, layering, standalone headers, negative
      compilation, coverage floors, documentation shape. A rule that only lives in prose is not
      a gate.

Next
----

- the symbol table an equation may use: :doc:`symbols`
- the API per group: :doc:`reference/index`
- where each term is *used* rather than defined: :doc:`quickstart` and :doc:`../guides/index`
