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
     - 451 cases, 0 failed
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
