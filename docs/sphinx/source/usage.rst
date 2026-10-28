_`Usage and verification`
=========================

This page is the part a user actually needs: the three faces of the library, the model behind
them, and the evidence that they work.

_`C++ path (recommended for board-level work)`
----------------------------------------------

Every policy is a template parameter: the observer, the modulator, the controllers and the HAL.
Nothing is virtual, and swapping a policy costs nothing at run time.

.. code-block:: cpp
   :caption: one motor, one control step
   :name: cpp-control-step

   #include <foc/core/observer/back_emf_observer.hpp>
   #include <foc/core/pi.hpp>
   #include <foc/core/svpwm.hpp>
   #include <foc/driver/stub_hal.hpp>      // swap for your own HalPolicy on real hardware
   #include <foc/motor/motor.hpp>
   #include <foc/motor/motor_config.hpp>

   using Motor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm,
                                   foc::core::Pi, foc::driver::StubHal>;

   foc::driver::StubHal hal;                       // or your board's HAL
   const foc::motor::MotorConfig cfg = foc::motor::MotorConfig{
       0.5F, 4e-4F, 4e-4F, 0.05F, 1e-5F, 1e-6F, 7U, 5.0F, 20000.0F, 500.0F};
   const auto current = foc::current_pi_config(cfg, 2000.0F, 24.0F);

   Motor motor(cfg, Motor::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{},
                                    foc::core::Pi{current}, foc::core::Pi{current},
                                    foc::core::Pi{foc::speed_pi_config(cfg, 200.0F)}},
               hal);

   motor.init();
   motor.start();
   motor.set_torque(1.0F);                         // q-axis current in A
   motor.tick(1e-4F);                              // call this from your ISR

:code:`tick()` is noexcept, allocates nothing and does no I/O — it is the ISR entry point.
The tuning helpers (:code:`current_pi_config`, :code:`speed_pi_config`) live in the library so
the C path and the SIL bench use exactly the same numbers.

_`C path (non-C++ projects, glue code)`
---------------------------------------

.. code-block:: c
   :caption: the same motor through the frozen ABI
   :name: c-control-step

   #include <foc/api/foc_api.h>

   foc_motor_params_t params = {.rs = 0.5F, .ld = 4e-4F, .lq = 4e-4F,
                                .kt = 0.05F, .j = 1e-5F, .b = 1e-6F, .pole_pairs = 7};
   foc_motor_t motor = NULL;
   if (foc_motor_create(&params, &motor) != FOC_OK) { /* pool exhausted */ }

   foc_motor_enable(motor);
   foc_set_torque_g(motor, 1.0);                   // _Generic: float or double, your call
   foc_tick(motor, 1e-4F);

   float speed = 0.0F, id = 0.0F, iq = 0.0F;
   foc_get_state(motor, &speed, &id, &iq);
   foc_motor_destroy(motor);

Handles are opaque tokens carrying a generation counter: after :code:`foc_motor_destroy()` an
old handle answers :code:`FOC_ERR_PARAM` instead of touching freed memory. The C path runs on
the stub HAL with library defaults (20 kHz, 500 ns dead time, 5 A limit, 24 V bus); for real
hardware use the C++ path.

_`Python path (host-side scripts)`
----------------------------------

.. code-block:: python
   :caption: the same motor from Python
   :name: python-control-step

   import foc_python as foc

   with foc.Motor.from_parts() as motor:          # the C handle dies with the with-block
       motor.enable()
       motor.set_torque(1.0)
       for _ in range(1000):
           motor.tick(1e-4)
       print(motor.telemetry())                   # speed, id and iq of the last step

The module binds the **C API**. Failures raise :code:`foc_python.FocError` whose
:code:`code` attribute is the very :code:`ErrorCode` the C function returned — Python gets
exceptions, you keep the raw code.

_`The model behind the loop`
----------------------------

The controller sees a dq-frame PMSM. The plant used by the SIL tests integrates exactly this,
with fixed-step RK4 at 10 µs against a 100 µs control step:

.. math::
   :label: pmsm dq model

   \begin{aligned}
   \frac{di_d}{dt} &= \frac{v_d - R_s i_d + \omega_e L_q i_q}{L_d} \\
   \frac{di_q}{dt} &= \frac{v_q - R_s i_q - \omega_e L_d i_d - \omega_e \psi_f}{L_q} \\
   \frac{d\omega_m}{dt} &= \frac{T_e - B \omega_m - T_{load}}{J}, \qquad
   T_e = \tfrac{3}{2} p \left(\psi_f i_q + (L_d - L_q) i_d i_q\right)
   \end{aligned}

with :math:`\psi_f = K_t / (1.5 p)` and :math:`\omega_e = p\,\omega_m`. The torque constant is
*knot* the back-EMF constant: conflating them is the classic way to be wrong by 1.5 p.

_`Sizing the injection (HFI)`
-----------------------------------

The injection observer buys an angle where back-EMF has nothing to say, and it is not free. Every number below was
measured on the SIL envelope fixture — a loaded machine held at a given speed on a dyno, the observer as a passenger, so
a cell is a property of the observer and not of a loop closing on it.

**The carrier has a ceiling set by the control rate.** Ask for it rather than guess:

.. code-block:: python

   import foc_python as foc

   ceiling_hz = foc.hfi_max_inject_hz(20000.0)   # 8000.0 Hz: 2.5 control periods per carrier period
   assert setup.inject_hz <= ceiling_hz

The same question from C is :code:`foc_hfi_max_inject_hz(pwm_freq_hz)`. Past the ceiling the observer does not get
noisy, it stops: the demodulated amplitude collapses, the lock detector drops out, the angle freezes and the speed reads
zero, which a loop cannot tell from a stalled rotor — measured as a -19.5 rad/s speed bias before the failure was
understood. :code:`foc_motor_create_ex()` refuses such a request, and the cases it cannot catch are reported by
:code:`observer_locked`.

The same admission test runs when the observer is changed on a running motor: :code:`Motor.set_observer()` --
:code:`foc_motor_set_observer()` from C -- validates the request against the setup already in force, so a motor built
without a carrier can run on back-EMF but cannot be handed over to HFI later. Outside the Run state it answers
:code:`STATE` instead, because a hand-over carries an estimate and an observer that has not been stepped has none to
carry. Both refusals arrive as :code:`FocError`, never as a silently unchanged :code:`status()["observer"]`.

Within the ceiling, higher is better on both axes at once, because the angle bias falls faster than the injected current:

.. list-table:: 20 rad/s mechanical, 200 Hz demodulation, 12 V
   :header-rows: 1

   * - carrier
     - angle bias
     - HF current
   * - 2000 Hz
     - 0.0060 rad
     - 4.04 A
   * - 4000 Hz
     - 0.0012 rad
     - 1.89 A
   * - 8000 Hz
     - 0.00018 rad
     - 0.38 A

**Ask for the whole configuration, rather than assembling it.** :code:`foc.hfi_size()` -- :code:`foc_hfi_size()` from C --
takes the machine and the control rate and returns the carrier, the amplitude, and both cutoffs:

.. code-block:: python

   sized = foc.hfi_size(params, pwm_freq_hz=20000.0, hf_current_a=0.5)
   setup.observer, setup.inject_v, setup.inject_hz = foc.Observer.HFI, sized["inject_v"], sized["inject_hz"]

The amplitude is the volts that draw the HF current *you* budget ($V = I_{hf}\,\omega_h L_d$), so the injection becomes a
decision about current -- which is what sizes the torque ripple and the extra losses -- instead of a voltage copied from an
example. And the result is a configuration the library accepts: that is what the tests assert, because a sizing tool that
recommends something the library then refuses would be worse than no tool.

**The demodulation window is a fixed choice, not a scheduled one.** Across standstill to 50 rad/s mechanical it changes
the angle error by under 0.5 %, and at standstill the *narrowest* window is best by 14x. What looked like "a narrow
window breaks at speed" is a start-up transient, which is what the blanking time is for.

**What is left at low speed is the injection's own price.** In a closed servo at 20 rad/s the steady bias is
0.0039 rad/s — 0.02 % — and the remainder is a ripple at the carrier frequency. It is not the estimator, the angle, the
window, the rotor's inertia or the demodulator's clamp: it scales with the injected current (a third of the volts, 0.36
of the tone) and with speed (4.0x the speed, 4.0x the tone), and it is indifferent to the saliency. It is the injected
current cross-coupling into torque on a turning machine, and :code:`inject_v` is the knob that sizes it.

**Watch it rather than assume it.** :code:`status()` carries the observer's own verdict:

.. code-block:: python

   status = motor.status()
   if not status["observer_locked"]:
       ...   # the angle is a hold, not a measurement; speed reads zero
   carrier_ripple_a = status["observer_error"]   # ~ -sin(2*angle_error) for HFI, 0 for the others

:code:`observer_error` is exactly zero for observers that publish no error signal, so read it together with
:code:`observer_locked` and :code:`observer`.

**Record the window, rather than watching a scalar.** A status report answers "what is true now"; a fault is usually a
question about the last few milliseconds. The trace channel is a ninth policy on the motor, so it is chosen at compile
time and costs nothing when it is not chosen:

.. code-block:: cpp

   using Traced = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi,
                                    MyHal, foc::core::NullReferenceGenerator, foc::core::NullFeedforward,
                                    foc::core::NullPositionControl, foc::core::NullAlignment,
                                    foc::core::RingTrace<1024>>;   // 1024 periods = 36 KiB at 36 B/sample

   Traced motor(config, policies, hal);
   ...
   const auto sample = motor.trace().latest();       // the period that just ran
   const auto dropped = motor.trace().dropped();     // 0 unless the ring wrapped

One sample per period, six numbers and two states: the measured currents the PI acted on, the currents it was asked for,
the voltage it applied, the mechanical speed, and the mode and status the period ended in. With the default
:code:`foc::core::NullTrace` the type is empty -- no member, no branch, no pointer check -- so "off costs nothing" is a
property of the type system rather than a promise about the optimiser.

A full ring keeps recording and counts what it overwrote, because forgetting with a counter is reversible and stopping is
not: a trace that freezes when it fills loses exactly the window a fault is interesting in. Timestamps come from the
platform clock, truncated to 32 bits (71.6 minutes); with no clock every :code:`t_us` is zero, which means "no timestamp"
rather than "everything happened at once".

From C the same window is caller-owned storage and a run-time capacity, because the ABI is one type for every caller:

.. code-block:: c

   static foc_trace_sample_t window[1024];
   foc_motor_set_trace(motor, window, 1024U);   /* NULL, 0 detaches */
   ...
   uint32_t size = foc_motor_trace_size(motor);
   uint32_t lost = foc_motor_trace_dropped(motor);   /* 0 unless the ring wrapped */
   foc_trace_sample_t sample;
   foc_motor_trace_read(motor, size - 1U, &sample);

and from Python the storage belongs to the object, so there is nothing to keep alive by hand:

.. code-block:: python

   trace = foc.Trace(1024)
   motor.set_trace(trace)                 # or set_trace(None) to detach
   ...
   rows = trace.samples(motor)            # oldest first, a list of dicts
   rows[-1]["i_q"] == motor.telemetry()["iq_a"]   # exactly, not approximately

The C path pays one null check per period for that run-time capacity -- predictable, and nothing when detached. The
compiled-in C++ path pays nothing because the capacity is the type. Measured cost of the whole feature where it lands: the
Debug archive grew 8.6 KB (56.9 to 65.6 KB), of which the code is 1.5 KB and the ten new functions themselves about
0.65 KB; the rest is DWARF for two new types, which a Release artifact does not carry. A Release ROM figure has **not**
been measured, and is not claimed.

Three things this channel is measured to do, and one it cannot. Measured: a sample agrees with
:code:`telemetry()`/:code:`state()`/:code:`status()` **bit for bit** in the same period; the null channel adds no storage
and no branch (host, best of three rounds: 158.0 ns/period off against 161.9 on over 20 000 periods -- the +3.9 ns difference is at the resolution of that measurement, and one round of it reported recording as *faster*, which is how a noise figure announces itself); and a sample survives a :code:`%.9g` text
round trip bit for bit, so a trace written to a file is still the same numbers when it is read back. What it cannot do is
tell you what the *hardware* did -- it records what the loop believed, so a perfect-looking trace is entirely consistent
with a miscalibrated current sensor. Per-period cost on a target is also unmeasured here: the host figure above says
nothing about a 100 MHz core, and a budget for that is a claim somebody would have to take with a scope.

_`Verification: what is actually checked`
-----------------------------------------

Every number below is produced by the commands in the same column, on the same commit.

The rightmost column is the **evidence level** (defined in the project README): L1 is enforced by
construction, L2 is measured against the model in this repository with the criterion written first,
L3 adds a measured sensitivity to the model being wrong, L4 would be measured on real hardware.
**There are no L4 claims** -- no motor has been attached to any of this. Read every number below as
"true of the documented model", and use :code:`BoardSelfTest` / :code:`check_board()` to find out
whether your hardware matches that model.

.. list-table:: evidence, not adjectives
   :header-rows: 1
   :widths: 26 34 26 14

   * - What
     - How
     - Result
     - Level
   * - Unit, SIL and stress cases
     - :code:`conan create . -s build_type=Debug --build=missing`
     - 465 cases, 0 failed
     - L1/L2
   * - Line coverage
     - :code:`python3 .github/misc/coverage_gate.py`
     - 89.45 % overall, 92.44 % in :code:`core/`
     - L1
   * - Layering and standalone headers
     - :code:`gates.py guard` / :code:`gates.py strict`
     - 0 violations
     - L1
   * - Negative compilation (fake HAL must not compile)
     - :code:`python3 .github/misc/negative_compile.py`
     - 27 / 27 cases behave as specified
     - L1
   * - The dq model itself
     - :code:`test_abc_plant.cpp` -- an independently derived phase-variable plant
     - 6 operating points agree on :math:`(i_d, i_q)` within 0.02 A, torque within 0.3 %
     - L3
   * - Unmodelled effects (q-axis saturation, iron loss, sensor gain/offset)
     - :code:`test_sil_sensitivity.cpp`
     - 8 rows; the worst single effect doubles the angle error, all stay inside the ceiling
     - L3
   * - Three hosts and two cross targets
     - CI (ubuntu / macOS / Windows; arm-linux-a53, mcu-m4)
     - 17 / 17 jobs green
     - L1

Two things this table deliberately does *not* say. First, "0 failed" is a statement about the
scenarios someone wrote, not a proof of correctness. Second, the SIL bench's own validity rests on
the phase-variable cross-check above -- which is why that cross-check exists: with no motor expert
to review the derivation, a second derivation is the only thing that can disagree.


The SIL bench is layered on purpose: an analytic reference (L1), an independent offline
arbiter (L2), and the scenario tests (current step, speed step, sine tracking, saturation
limit, parameter perturbation). The SguanFOC project is a *third* reference, not the judge.
