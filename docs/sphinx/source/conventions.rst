_`Convention alignment`
=======================

This page is the **precondition** for comparing anything against an external reference — another
FOC implementation, a bench measurement, or an offline model. Angle frames, rotation direction,
units and clamping style must agree *before* any RMSE or overshoot number is computed; otherwise
the comparison measures the conventions, not the algorithm.

The conventions below are not a preference: they are whatever :code:`include/foc/` already
implements. Any reference has to align to them, never the other way round.

_`1. What this library fixes`
-----------------------------

.. list-table::
   :header-rows: 1
   :widths: 18 46 36

   * - Item
     - Convention
     - Evidence (the code is the spec)
   * - Angle unit
     - radians
     - :code:`hal::PositionSensor::angle_mech_rad()` returns rad
   * - Mechanical zero
     - zero when the rotor d-axis coincides with phase A; the sensor must remove its own mounting
       offset
     - :code:`hal::PositionSensor` contract comment
   * - Electrical angle
     - :math:`\theta_e = p\,\theta_m`, **not normalised** (:code:`park` / :code:`inv_park` use
       :math:`\cos/\sin`, so periodicity is automatic)
     - :code:`motor::Motor::tick()`
   * - Rotation sense
     - **counter-clockwise is positive**: :code:`park(v, θ)` and :code:`inv_park(v, θ)` are exact
       inverses, and increasing θ rotates the vector forward
     - :code:`core::park` / :code:`core::inv_park`
   * - Clarke transform
     - **amplitude invariant**: :math:`\alpha = (2a-b-c)/3`, :math:`\beta = (b-c)/\sqrt{3}`; a
       three-phase sine of peak :math:`I` gives :math:`|i_{\alpha\beta}| = I`
     - :code:`core::clarke`, case :code:`Transforms.ClarkeIsAmplitudeInvariant`
   * - dq convention
     - **motor convention**: :math:`i_q > 0` produces positive electromagnetic torque; the d-axis
       is aligned with the rotor flux
     - the torque equation in :code:`sil_pmsm_plant.hpp` and the sign of :code:`iq_ref`
   * - Torque constant
     - :math:`K_t` [N·m/A] and flux :math:`\psi_f` [Wb] are related by
       :math:`\psi_f = K_t / (1.5\,p)`
     - :code:`foc::psi_f_wb()`
   * - Units
     - **SI throughout**, no per-unit: V, A, mechanical rad/s, N·m, H, kg·m²
     - :code:`motor::MotorConfig` field comments
   * - Current direction
     - positive means current flowing into the motor
     - :code:`hal::CurrentSense::read_uvw()` contract
   * - SVPWM
     - symmetric seven-segment with mid-point (min-max) injection;
       :code:`duty = clamp(0.5 + (v_phase − v_mid)/v_bus, 0, 1)`; the linear-modulation limit is
       :math:`|v_{\alpha\beta}| = v_{bus}/\sqrt{3}`
     - :code:`core::Svpwm::modulate()`
   * - DC link
     - read once per control period via :code:`hal::DcLink::voltage_v()`; SVPWM normalises with it,
       and :code:`v_bus <= 0` yields 50 % duty with :code:`Param`
     - :code:`core::Svpwm::modulate()`
   * - PI form
     - positional (parallel): :code:`I += ki·e·dt` (forward Euler) → clamp the integral →
       :code:`out = clamp(kp·e + I)`; **anti-windup is static integral clamping only**, no
       back-calculation
     - :code:`core::Pi::step()`
   * - Loop rates
     - current loop at the carrier frequency (SIL bench: 20 kHz / 50 µs); the speed loop runs at the
       same rate
     - :code:`sil_bench.hpp:standard_bench_config()`
   * - Plant integrator (SIL)
     - C++ side: hand-written RK4, four sub-steps per control period, voltage vector held
       zero-order in the **stationary** frame; arbiter side: scipy :code:`solve_ivp` (RK45,
       rtol 1e-9) — the two paths are deliberately not the same code
     - :code:`sil_plant_hal.hpp`, :code:`offline_arbiter.py`
   * - Comparison slice
     - mechanical speed (rad/s), :math:`i_d` / :math:`i_q` (A), time measured from the step
       instant; samples aligned to the control period (the L2 reference keeps every fifth row)
     - :code:`test_sil_l2_reference.cpp`

_`2. The four traps that actually bite`
---------------------------------------

#. **Mechanical vs electrical angle.** A missing :code:`pole_pairs` makes the whole dq frame rotate
   at :math:`1/p` of the right speed; the symptom is "the current loop never catches up".
   :code:`PlantHal::advance()` and :code:`PlantPosition` must use one conversion, not two.
#. **Amplitude-invariant vs power-invariant Clarke.** A power-invariant Clarke (with
   :math:`\sqrt{2/3}`) scales :math:`|i_{\alpha\beta}|` by :math:`\sqrt{3/2}` — "1 A commanded,
   1.22 A delivered". A closed loop hides this: the integrator quietly absorbs the gain error, so
   only an open-loop comparison exposes it.
#. **Stationary vs rotating zero-order hold.** A real inverter pins the voltage vector in the
   *stationary* frame for the whole PWM period. Holding it in the dq frame (re-parking every
   sub-step) adds a spurious zero-order-hold error at speed. Both the C++ plant and the arbiter
   use the stationary frame.
#. **The ramp trap in steady-state error.** On a speed ramp the current loop always lags by
   :math:`e = \dot{E}MF / k_i` (about 1 % here). That is not a defect: before judging
   "steady-state error", confirm the run is on a constant-speed segment.

_`3. Extra checks before comparing with a third party`
------------------------------------------------------

.. list-table::
   :header-rows: 1
   :widths: 26 74

   * - Item
     - What to confirm
   * - dq convention
     - motor convention or generator convention (:math:`i_q` changes sign)
   * - Rotation sense
     - clockwise or counter-clockwise positive (:math:`\theta` changes sign)
   * - Units
     - per-unit or SI; if per-unit, what are :math:`V_{base}`, :math:`I_{base}`, :math:`\omega_{base}`
   * - PI anti-windup
     - present or not, and how: clamping / back-calculation / conditional integration
   * - SVPWM
     - seven-segment or five-segment; over-modulation or not; per-phase or vector clamping
   * - Sampling and delay
     - one-period computation delay? mid-point sampling? loop rate versus PWM frequency?

_`4. Status of the SguanFOC reference`
--------------------------------------

- **Not vendored, not built, not in any CI path.**
- Upstream is a single-maintainer project whose licence and long-term maintenance were never
  audited.
- Vendoring a snapshot (pinned commit) means putting third-party code into this repository —
  licence, size and maintenance are all affected, so it is an architecture-level decision that
  belongs to the repository owner. Until such a decision exists, sections 1 and 3 above are the
  only valid reference.
- Even if it were vendored, it would only be **read** for specific ideas (LADRC, HFI). It produces
  no comparison data and enters no test. When it disagrees with this page, the default is to
  distrust SguanFOC.

_`5. Change rule`
-----------------

#. Any change to a convention above (introducing per-unit, switching to generator convention,
   adding back-calculation to the PI) must change **this page first**, then the code, then the L2
   reference traces (regenerated with :code:`offline_arbiter.py`) — and only then may it be
   committed. Reversing that order yields gates that are green while the convention is wrong.
#. A new comparison scenario appends a row to the *comparison slice* in section 1 and gets its own
   decision record in the PRD.
