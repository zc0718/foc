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

**Compare a window instead of remembering a number.** A status report answers "what is true now" and a trace answers "what
the loop did"; neither answers "did this change". Three recorded windows do, and they are compared on demand:

.. code-block:: console

   python3 benchmark/script/golden.py --record   # (re)write the three baselines from this tree
   python3 benchmark/script/golden.py --check    # compare now against them; prints, exits 0

Each window is plain text: a header naming the scenario and the module hash it came from, then one line per sample. It is
readable in a diff, which is the only reason a stored number can be reviewed instead of trusted. The report names the
**trace**, the **quantity** and the **difference**, and classifies it:

.. list-table:: what a difference means
   :header-rows: 1
   :widths: 22 78

   * - band
     - reading
   * - ``<= 1e-5``
     - same -- arithmetic noise between builds, not a behaviour change
   * - ``<= 30 %``
     - drift -- worth a look, not evidence of anything by itself
   * - ``1.3x .. 2.0x``
     - **attribute** -- the signature of a missing term rather than of noise; this is where the W7-c investigation
       landed (0.1356 predicted against 0.0900 measured, 1.51x) and the term that explained it was real
   * - ``> 2.0x`` or a sign flip
     - changed -- expected after an algorithm change, a surprise otherwise

The ledger **reports and does not block**, deliberately: a ledger that gates a build gets its tolerance widened the first
time it is inconvenient, and then the reference value means nothing. ``--strict`` exists for a release gate, where a human
has already decided that this build and this baseline are comparable.

A golden is a recording from one machine and one build, not a specification: a different compiler produces different
floats in the last digits. The header carries the module hash so the report can say when a baseline came from another
build, and the bands above are what keep that honest -- a foreign baseline shows up as drift, not as a silent pass.

**Measure the loop instead of eyeballing the plot.** A window tells you what happened; `foc/core/step_metrics.hpp`
turns it into the numbers a tuning decision rests on. The library's own speed loop is designed as a first-order closed
loop with time constant `1 / wb` (`speed_pi_config()` cancels the mechanical pole `B/J` and leaves `wb / s`), so the
bandwidth it reports can be checked against the bandwidth that was *asked for* -- a measurement of the gain law, not of
the fit.

.. code-block:: cpp

   // y: the recorded window, oldest first, starting at or before the step.
   // y：录制窗口，最早的在前，从阶跃前或阶跃处开始。
   const foc::core::StepMetrics m = foc::core::step_metrics(y, n, dt_s, target, band);
   if (m.status == foc::ErrorCode::Ok) { use(m.bandwidth_rad_s, m.settling_s); }

.. code-block:: python

   # The same metric, and the same window, from the other side of the C ABI.
   # 同一个指标、同一个窗口，从 C ABI 的另一侧。
   import analyze, foc_python as foc
   c = foc.step_metrics(speed, dt, target, band)          # C++
   p = analyze.step_metrics(speed, dt, target, band)      # numpy
   assert abs(p.bandwidth_rad_s - c['bandwidth_rad_s']) / c['bandwidth_rad_s'] <= 0.01

Three criteria, each of which was watched going red before it was believed:

- **fitted vs analytic, <= 10 %.** A 10 rad/s step on the SIL bench fits to **243.57 rad/s against the 250 rad/s
  requested (-2.57 %)** through the Python plant, and **+9.15 %** through the C++ bench. This band is weak and the
  weakness is measured: a 20 % error in the gain law moves the reading only to -9.88 %, still inside it, because the
  estimator's bias is the same size as the defect. The companion criterion -- request 125 and 250 rad/s and demand the
  measurements move by the same factor (134.66 and 272.87 rad/s, ratio 2.026) -- is what notices a loop that ignores
  the request it was given. Attribution, one knob at a
  time: raising that bench's inner current bandwidth 2000 to 4000 rad/s moves the number from +9.4 % to +5.8 %, halving
  the control period moves it 0.07 points, and giving the window its pre-step sample moves it 0.30. The inner loop is
  the story; the rest is a first-order model applied to a cascade.
- **C++ vs Python on the same samples, <= 1 %.** Two implementations of one definition, compared in one process --
  measured agreement is 0.003 % at 400 rad/s, so the band is a tripwire for drift rather than a tolerance.
- **replay, error 0.** The settling time is an *index* decision, not a fitted number, so both sides must pick the same
  sample: the analysis layer recomputes it from the window and the comparison is on the index, exactly.

A sweep closes the gap the band above leaves open. Drive the reference with `A*cos(w t)`, fit both the command and the
response at that frequency (`foc/core/sinusoid_fit.hpp`), divide: the -3 dB point needs no model and moves *with* an
error instead of carrying a bias of its own. Measured: **234.91 rad/s against 250 requested (-6.04 %)** on the C++
bench, **236.18 rad/s (-5.53 %)** through the Python plant, two independent machines agreeing to 0.5 %. The same 20 %
gain error that the step band could not see moves this crossing to **210.79 rad/s (-15.68 %)**, outside the band.

.. list-table:: the same defect, two instruments
   :header-rows: 1
   :widths: 34 22 22 22

   * - criterion
     - no defect
     - gain law x0.8
     - verdict
   * - step fit vs analytic, 10 % band
     - +9.15 %
     - -9.88 %
     - blind: passes either way
   * - step fit ratio 125 / 250
     - 2.026
     - 2.026
     - blind: a ratio cannot see a scale error
   * - sweep -3 dB, 10 % band
     - -6.04 %
     - **-15.68 %**
     - fails, as it should

The analysis layer also carries a spectrum and a THD (`analyze.spectrum`, `analyze.thd`), checked against a signal whose
answer is arithmetic rather than a modelling opinion: a unit tone reads 1.000 under a Hann window with the usual 2/sum
correction, and a known 11.2 % distortion made of a 10 % third harmonic and a 5 % fifth reads 0.1118 with each harmonic
in its own bin. Neither has a C-side counterpart, and neither needs one: they measure the window, not the loop.

**Break a stuck axis loose on purpose.** A speed PI does eventually break a stuck axis free -- its integral ramps the
current until the torque passes the breakaway point -- and that is the problem: by then the integral has stored the
whole windup, so the moment the rotor lets go the stored torque drives it past the setpoint. `core/stall_start.hpp`
does the same thing deliberately: a known torque ramp, a report the instant the rotor moves, and the torque it moved on
as the handover value.

.. code-block:: cpp

   foc::core::StallStart sequence(recipe);   // breakaway 0.030 N*m, limit 0.200, ramp 0.5 N*m/s
   if (sequence.begin()) {
       const auto out = sequence.step(dt_s, speed_mech);
       if (out.phase == foc::core::StallStartPhase::Released) { /* take over from out.torque_nm */ }
       if (out.phase == foc::core::StallStartPhase::Failed) { /* this load cannot be started here */ }
   }

Measured: 0.030 N*m of breakaway at 0.5 N*m/s is reached and released at **60.1 ms**; against a real stuck plant
(stiction 0.020 N*m, load 0.010 N*m) the sequence releases at **0.0307 N*m after 61.3 ms**, and a 0.9x torque -- a
sequence asking for less than it believes -- moves that to 68.0 ms and the case fails. A jam at the limit reports
`Failed` with zero torque after 400 ms of ramp plus a 200 ms grace.

**Turn a standing error into arithmetic.** A proportional position loop on a machine with friction has a *dead band*, not
a residual: below a certain error the torque it can still ask for is under the breakaway and the axis does not move at
all. `core/friction_comp.hpp` adds the Coulomb term that moves that wall, and refuses the estimates that would turn it
into a motor.

.. code-block:: cpp

   foc::core::FrictionComp friction(recipe);   // coulomb 0.010 N*m, static 0.020, viscous 1e-3, band 0.05 rad/s
   const auto out = friction.step(speed_ref, speed_meas, demand_torque);
   const float torque = demand_torque + out.torque_nm;   // and check out.stalled before trusting either

The numbers are the capability. The band is `static / kp` -- **4.003 mrad measured against 4.0 predicted** at
`kp = 5 N*m/rad` -- and with the compensation it is `(static - coulomb) / kp`, **2.021 against 2.0**. It follows the
estimate, which is why the estimate has to come from a measurement: 1.007 mrad with a 50 % over-estimate, and above the
breakaway there is no band at all -- a `(0.030, 0.030)` estimate walks the axis **5.625 mrad with the reference held at
zero and no command at all**.

The latch is not decoration. At rest the measured speed is zero-mean noise, so a Coulomb term on `sign(speed)` is a
square wave of amplitude twice the term: measured with +-0.01 rad/s of encoder noise, the naive law ripples **20.03
mN*m** peak to peak and the latched one **0.036**, 550 times quieter, while both park the axis in the same place.

**Run a motor from three Hall sensors.** `core/hall.hpp` carries the two things a Hall drive needs and nothing else:
the six-step table, and an electrical angle interpolated from edge timing.

.. code-block:: cpp

   foc::core::HallTracker tracker;
   // every control period, with the three GPIOs as a code:
   const auto status = tracker.update(now_s, hall_code);
   const float theta_e = tracker.angle_elec_rad(now_s);   // use only when status == Ok
   const auto duty = foc::core::six_step_duty(hall_code, amplitude, foc::core::HallDirection::Forward);

The numbers that decide whether that is enough. Held at one commutation state, the torque is the cosine of where the
rotor sits in the sector: maximum at the centre, **0.866 at both edges, never negative** -- a 14.0 % peak-to-peak ripple
that six-step cannot avoid, and the argument for interpolating an angle at all. Interpolating buys a factor of **52 at
120 rad/s electrical** (0.338 degrees RMS against the sector centre's 17.52), and the factor shrinks to 7.5 at 900 rad/s
because what is left is one *sampling period* of rotation (2.30 degrees at 62.5 us) rather than the 60-degree sector.

What it cannot do is in the same header: with a sensor stuck, `Fault` covers 15.5 % of a revolution and `State` another
53.9 %, and **8.4 % still reads `Ok` while wrong by up to 59 degrees**. The status is the whole admission story -- an
estimate that is not being measured says so rather than reporting a sector centre as if it were a measurement.

**A fault is not a temperature.** Protection is graded -- derate, then trip -- and the two decisions are separate
fields rather than one code: `output_allowed` is defined as `active_mask == 0`, so a **fault means authority was
removed**, while a **derating** drive is still running and says so through `i_limit_a` falling below the `i_max_a` the
caller configured. Measured on the ladder case: 3.00 A at 45 C, **1.80 A at 80 C with output still allowed and no
fault**, then 0 A and no output at 105 C. When two faults are active at once the lower-ranked one is reported and both
bits stay visible in `active_mask` (0x006 for current plus voltage, 0x018 for voltage plus heat).

**Command a torque, not an amperage.** `set_torque()` takes amps and leaves the N*m conversion to the caller -- the
same arithmetic, with the same sign convention and the same limit, in every product. `foc/motor/torque_map.hpp` is that
arithmetic once:

.. code-block:: cpp

   const foc::TorqueMap map = foc::torque_map(config, /*min_nm=*/0.0F, /*max_nm=*/1.0F);   // one-way
   const foc::TorqueCommand cmd = foc::torque_command(map, 0.2F, /*id_a=*/0.0F);
   motor.set_torque(cmd.iq_a);          // 0.2 N*m -> 4 A at kt = 0.05 N*m/A
   if (cmd.limited) { /* the request was clipped, and someone should know */ }

The estimate is `kt*iq + 1.5*pp*(ld-lq)*id*iq`, which is the expression the plant integrates -- so it is checked against
something that shares no line with it: the mechanical torque `J*dw/dt + B*w` from the recorded speed. The two agree to
**+0.006 % at 2 A** and **+0.007 % at 20 A**, where the loop is saturated; the plan's 5 % band is three orders of
magnitude wider than the measured residual, which is the forward-difference error in `dw/dt`. Scaling the map's gain by
10 % moves the same measurement to **-9.994 %**, so the band is a gate rather than decoration.

**Ask what a wrong nameplate costs.** Every gain here is derived from the nameplate, so the design question is how wrong
it can be before the claim stops holding. `benchmark/script/montecarlo.py` draws machines, runs the loop on each and
reports the spread:

.. code-block:: console

   python3 benchmark/script/montecarlo.py --trials 40 --sigma 0.10           # nameplate error, machine nominal
   python3 benchmark/script/montecarlo.py --trials 40 --sigma 0.10 --matched # each machine correctly described

With a 10 % standard deviation on six nameplate parameters, a **matched** family -- every drawn machine described
correctly by its own nameplate -- spreads **2.92 %** in bandwidth and 97.5 % of it stays inside the 10 % band, while the
**mismatched** family spreads **19.18 %** and 42.5 % of it does. At 20 % the same pair reads 6.21 % against 29.74 %
(yields 92.5 % and 12.5 %). That factor of about six and a half is what deriving every gain from the nameplate buys, and
it is the only number on this page that states it as a quantity rather than a design intention. All four figures are
`--trials 40 --seed 11`, the seed the suite uses.

A single parameter moves predictably, which is the arithmetic behind `kp = wb*J/Kt`: a nameplate `J` 10 % high raises
the bandwidth 8.46 % (the 1.40 % shortfall is the mechanical pole `B/J` only partly cancelled, since `ki = wb*B/Kt` does
not move with `J`), and a nameplate `kt` 10 % high lowers it 9.28 % (prediction 9.09 %, within 0.2 %).

Read back through the CLI, the ledger's own `speed_step_100` window measures **197.59 rad/s** against the 200 rad/s the
library defaults to (-1.2 %) -- on a window decimated to one sample per millisecond against a four-millisecond time
constant. That is the analysis layer replaying a stored file rather than the object that produced it.

What it cannot do: a first-order fit on a response that is not first-order reports the first-order *equivalent*, which
is a real number and not the system's bandwidth -- which is why the band above is 10 % and is attributed rather than
assumed.

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
     - 514 cases, 0 failed
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
