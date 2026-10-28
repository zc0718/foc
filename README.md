# foc — field-oriented control for three-phase PMSM

A field-oriented-control library for three-phase PMSM drives, built on the fcpp scaffold. One
constraint drives every design choice: **the control loop must be predictable** — no heap, no
exceptions, no virtual calls and no hidden allocation on the path that runs every PWM period.

The same implementation is reachable three ways: the C++17 template core for board-level work, a
frozen C11 ABI for non-C++ hosts, and an optional pybind11 module for host-side scripts. Start with
`docs/sphinx/source/quickstart.rst`, or the generated site under `docs/sphinx/build/html` (Chinese)
and `docs/sphinx/build/html_en` (English).

## Badges

![License](https://img.shields.io/github/license/zc0718/foc?color=blue&label=license)
![C++](https://img.shields.io/badge/C%2B%2B-17-blue?logo=c%2B%2B&logoColor=white)
![C](https://img.shields.io/badge/C-11-blue?logo=c&logoColor=white)
![CI](https://github.com/zc0718/foc/actions/workflows/ci-orchestrator.yml/badge.svg)
![Release](https://github.com/zc0718/foc/actions/workflows/semver-release.yml/badge.svg)
![CMake](https://img.shields.io/badge/cmake-4.2%2B-orange?logo=cmake)
![Python](https://img.shields.io/badge/python-3.10%2B-blue?logo=python)
![Doxygen](https://img.shields.io/badge/docs-Doxygen-green?logo=doxygen&logoColor=white)
![Sphinx](https://img.shields.io/badge/docs-Sphinx-blue?logo=readthedocs&logoColor=white)

## Project Overview

- **Language**: C/C++
- **Library Build System**: Python with Conan 2.0
- **File Build System**: Doxygen, Graphviz, Sphinx
- **Module Support**: Optionally activated, when C++ standard ≥ 23
- **Metadata-Driven**: `metadata.json` is the single source of truth (build, deps, docs, CI)
- **Skills & Agent**: harness-agnostic Agent Skills + one routing agent (canonical `.github/skills/`,
  default `.agents/skills` view; optional `.claude`/`.codex` views)
- **Component Structure**:
    - pairwise header and source assumption
    - suffix distinguishment, (.h, .c) for C part, and (.hpp, .cpp) for C++ part
    - documenting system uses .dox for pure docstring, .cxx for examples codes; both are
      documentation-only and live exclusively under `docs/doxygen/dox/`, never in `include/`/`src/`

## Features

- Field-oriented control: Clarke/Park and inverses, SVPWM, PI controllers, state machine, back-EMF
  observer — all hardware-agnostic in `foc/core`
- **Policy-composed motor**: observer, modulator, controllers and HAL are template parameters, so the
  current loop inlines into one translation unit (no virtual calls on the ISR path)
- **Frozen C ABI** (`foc/api/foc_api.h`) over the same core, with generation-checked opaque handles
- Optional **pybind11 module** (`foc_python`) bound to the C API, skipped automatically on cross builds
- **SIL bench** with a dq-frame PMSM plant, used by both the test suite and the documentation figures
- **Commissioning** (`foc/core/commissioning.hpp`): `Rs` from a two-point locked-rotor fit that
  separates the dead-time offset from the resistance, flux linkage from a no-load ramp, current-channel
  gain/offset, and incremental inductance from a flux integral
- **Protection** (`foc/core/protection.hpp`): over-current, I²t, over/under-voltage, over-temperature
  with derating, stall and lost-sync, with latching and hysteresis
- **Board self-test** (`foc/core/board_check.hpp`): a five-call bench concept that measures offsets,
  polarity, current sum, `Rs` and dead time, and a checklist that names the single cause which explains
  the most failures
- **Field weakening** (`foc/core/field_weakening.hpp`): exact quadratic operating-point solve with `Rs`
  retained, plus a "is this motor worth field weakening at all" test
- **MTPA** (`foc/core/reference.hpp`, `MtpaReference`): the d-axis current for maximum torque per
  amp, walked down the curve when the request does not fit, +29 % torque at `Lq/Ld = 2` where
  `i_char = i_max`
- Conan-based modern dependency management
- Supports C++ standards 17, 20, and 23
- Automatic module file generation (`.ixx`/`.cppm`) from headers/sources
- Cross-platform compatibility (Windows, Linux, macOS)
- Dual C and C++ interfaces with separate linkage targets
- Doxygen annotation support for object exporting (`@exporter`, `@attacher`)
- Automation documenting system via Doxygen and Sphinx
- Importation, derivation and call relationship illustration through Graphviz
- `metadata.json` as the single source of truth (build / deps / docs / CI)
- Harness-agnostic Agent Skills (`.github/skills/`, 15 task skills + one routing agent) with a default
  `.agents/skills` view and optional `.claude`/`.codex` views
- Metadata + gitmoji driven CI/CD orchestration (GitHub Actions)
- Cross-compilation & on-board benchmark framework (`benchmark/`)
- Agentic Coding workspace (`/workspace/`, git-ignored)

## Evidence Levels

Every quantitative claim here carries one of four levels. The level is part of the claim, not a
footnote: an L2 number is not a weaker L4 number, it is a **different kind of statement**.

| Level | What it means | How it is checked | Example |
| --- | --- | --- | --- |
| **L1** | Asserted by construction | A compiler flag, a type, or a gate that fails the build | "no exceptions" — `-fno-exceptions` plus `gates.py strict`, 27 negative-compile probes |
| **L2** | Measured against the model in this repository, with a criterion written before the measurement and the number recorded | A unit test that prints the number and asserts the criterion | "angle error 0.0124 rad at 200 rad/s" — true of `sil_pmsm_plant.hpp`, not of your motor |
| **L3** | L2 plus a **measured sensitivity**: how far the number moves when the model is wrong in the ways it is known to be wrong | `test_sil_sensitivity.cpp`, plus a phase-variable cross-check of the model itself | "…and 2.0× worse with a 10 % current-sense gain error" |
| **L4** | Measured on physical hardware, with the condition recorded | Not available here | — |

**This project currently makes no L4 claims.** There is no hardware in the loop at the time of
writing: no motor, no inverter, no current probe. Everything below the L1 line is a statement about a
model, and the model's own validity is exactly what L3 tries to bound.

The honest reading of an L2/L3 number is therefore: *the code is correct with respect to a documented
model, and that model has been checked for self-consistency (against a second, independently derived
phase-variable model) and for sensitivity (against the effects it omits)*. Whether **your** motor
matches the model is a separate question, and the library gives you the instruments to answer it —
`BoardSelfTest`, `RsEstimator`, `FluxEstimator`, `check_board()` — which are themselves only L3.

### What each module claims

Every installed header is listed here exactly once, with the level of what is claimed about it and
the thing that checks it. `.github/misc/claim_gate.py` enforces that: a header added without a row
fails the build, a row naming a test that does not exist fails the build, and **an `L4` row is
refused outright** because this project has no hardware to measure it on. Paths are relative to
`include/`.

| Module | Level | The claim, in one line | Evidence |
| --- | --- | --- | --- |
| `foc/api/foc_api.h`, `foc/api/foc_types.h` | L1 | The C11 surface compiles as C, links and runs; every misuse is an error code; handles carry their *kind*, so a motor handle cannot be read as a protector. `foc_motor_set_observer()` is the run-time hand-over, refused outside `Run` and refused for a mode this machine cannot run, so a C caller never has to reproduce the seeding rule itself | `test_api_c_smoke.cpp`, `c_api_validation.c` |
| `foc/core/types.hpp` | L1 | Value types with no hidden arithmetic | `test_transforms.cpp` |
| `foc/core/limits.hpp` | L1 | `clamp`/`sanitize` behave at the boundaries and at NaN | `test_limits.cpp` |
| `foc/core/version.hpp`, `foc/version.h` | L1 | One version string, identical from C and C++ | `test_version.cpp` |
| `foc/core/traits.hpp` | L1 | The policy concepts: a wrong policy fails to compile, and the failure is the one intended | `test_traits_contract.cpp`, `negative_compile.py` |
| `foc/core/transforms.hpp` | L3 | Clarke/Park and inverses; the dq derivation is cross-checked against an independently derived phase-variable model at six operating points (currents within 0.02 A, torque within 0.3 %) | `test_transforms.cpp`, `test_abc_plant.cpp` |
| `foc/core/svpwm.hpp` | L1 | Duty generation, clamped to the linear range | `test_svpwm.cpp` |
| `foc/core/pi.hpp` | L1 | `error = setpoint - feedback`; the integral is re-clamped every step so a shrunken window releases torque at once | `test_pi.cpp` |
| `foc/core/filters.hpp` | L1 | Discrete filters at their declared cutoff | `test_filters.cpp` |
| `foc/core/sm.hpp` | L1 | The state machine's reachable set and its refusals | `test_sm.cpp` |
| `foc/core/machine_params.hpp` | L1 | Nameplate → derived `psi_f`, one definition | `test_machine_params.cpp` |
| `foc/core/machine_map.hpp` | L1 | A current-dependent machine: rows reproduced exactly, linear between them, **clamped rather than extrapolated** outside the measured range, and an unsorted or degenerate map rejected. It says nothing about where the rows came from — that is the caller's claim | `test_machine_map.cpp` |
| `foc/core/startup.hpp` | L2 | The open-loop start runs inside `Motor`'s Align state -- the one state where the angle is forced and the observer is not stepped -- so the hand-over is exactly one tick. With a slow enough ramp the rotor is caught: measured steady slip 0.29 rad at 400 rad/s^2 and 0.20 rad at 200, and the hand-over keeps the machine within 5 % of its speed with no current spike. **Not** claimed: a closed-form limit on the ramp acceleration -- the load-angle expression tried here over-predicts by 1.7x to 2x and is reported, not asserted | `test_sil_startup.cpp` |
| `foc/core/position_reference.hpp` | L2 | Turns an electrical estimate into a position a loop can close on: unwrapped, tracking the truth to **0.000309 rad over five mechanical turns**; an index mark **starts the count** rather than pinning it to a single-turn value (6.2600 rad one turn after the mark); a repeat pass removes drift and does not rewind (6.4028 -> 6.2832 rad for 0.1429 rad of injected drift); and a step too large to unwrap is reported rather than guessed, cleared by the next mark. **Not** claimed: any absolute position before the first index -- there is none, and `homed()` is how a caller finds out | `test_position_reference.cpp` |
| `foc/core/homing.hpp` | L2 | The search that turns slowly until the index mark arrives, as a decision the caller applies rather than an actuation. An already-homed axis finishes before it starts; the command goes to zero on the exact tick the origin exists; a mark that never arrives times out on its own recipe (crossed at tick 5001 against a 5000-tick recipe) and stops commanding. Driving a sensor-less motor through it end to end finds the mark after **0.172 s** at truth **0.5004 rad** (mark at 0.5000) and then holds that place to **0.0121 rad** over the last 0.2 s. **Not** claimed: that it will find a mark outside the commanded direction's travel -- a search that never sees its mark is a machine that needs a person, not more current | `test_homing_routine.cpp`, `test_sil_homing.cpp` |
| `foc/core/speed_from_angle.hpp` | L2 | Speed by differencing the angle across a fixed delay, as an alternative to an observer's own loop state. Measured at 20 rad/s mechanical on the sensor-less rig: **0.1960 rad/s worst error against the injection observer's own 0.7995**, a 4.1x improvement -- and it clears the 4x bar only just. The noise model `sigma_theta*sqrt(2)/(N*dt)` predicts 0.030 rad/s; the measurement is 6.5x worse than that point value while still inside a factor-of-10 band, so the angle's error carries a correlated component that differencing does not average away. **Closed-loop result, and it is negative**: closing the library's own speed PI on the differenced estimate settles to 0.1982 rad/s against the observer's 0.1717 -- slightly worse -- and peaks at 62 % against 1.9 %. A quieter estimate that is ten milliseconds late is not a better estimate inside a loop: the window is transport delay, and ten milliseconds is about 115 degrees of phase at the loop's 32 Hz bandwidth. This is why it was measured before being wired into `Motor` | `test_sil_speed_from_angle.cpp`, `test_sil_speed_from_angle_closed_loop.cpp` |
| `foc/core/machine_map_text.hpp` | L1 | A machine document that survives a round trip bit for bit, an unknown key counted instead of rejected (a reader that refuses newer files stops the format growing), and a failed read that leaves the receiver untouched. It defines no vocabulary for where the numbers came from | `test_machine_map_text.cpp` |
| `foc/core/deadtime.hpp` | L3 | The axis projection: 4/3 on d, 2/√3 on q, 4/π averaged over a rotation. Using the d number on q costs 15.7 % on `Lq` | `test_deadtime.cpp` |
| `foc/core/feedforward.hpp` | L2 | Cross-coupling and back-EMF terms, on the SIL plant | `test_sil_feedforward.cpp` |
| `foc/core/reference.hpp` | L3 | `d = 0` default plus `MtpaReference`: +29.2 % torque at `Lq/Ld = 2`, and exactly `i_d = 0` on a surface machine | `test_mtpa.cpp` |
| `foc/core/field_weakening.hpp` | L3 | Matches the analytic voltage/current envelope to four digits; the "is it worth it" rule is a gate, not a suggestion; the policy engages end to end through `Motor` (`i_d = -1.93 A` 2 ms in) | `test_field_weakening.cpp`, `test_field_weakening_motor.cpp` |
| `foc/core/commissioning.hpp` | L3 | `Rs` +0.01 %, `psi_f` +0.51 %, `Ld` +0.14 %, `Lq` +0.23 % on the SIL plant, each with the condition it needs | `test_commissioning.cpp` |
| `foc/core/protection.hpp` | L2 | I²t budget `i_cont²·tau`; a trip removes the *command* in one period and the current then decays with `L/R` (0.80 ms) -- it cannot be clamped in one period, and the block says so | `test_protection.cpp` |
| `foc/core/trace.hpp` | L2 | One 36-byte sample per period, agreeing with `telemetry()`/`state()`/`status()` bit for bit in the same period; the null channel adds no storage and no branch (host, best of three: 158.0 ns/period off vs 161.9 on, and the +3.9 ns is at the resolution of the measurement); a sample survives a `%.9g` text round trip bit for bit | `test_trace.cpp` |
| `foc/core/board_check.hpp` | L3 | Names the correct cause on three seeded faults; measures the ADC trigger position when the bench can move it, and reports the optimum rather than assuming 0.5 | `test_board_check.cpp` |
| `foc/core/alignment.hpp` | L2 | The align/verify sequence and its failure modes | `test_alignment.cpp` |
| `foc/core/polarity.hpp` | L2 | Polarity detection and its refusal when the answer is not decisive | `test_polarity.cpp`, `test_sil_polarity.cpp` |
| `foc/core/position.hpp` | L2 | The position loop with velocity and acceleration limits | `test_position_p.cpp` |
| `foc/core/motion_profile.hpp` | L2 | Trapezoidal profile timing and its endpoints | `test_motion_profile.cpp` |
| `foc/core/observer/observer_policy.hpp` | L1 | The observer contract (`theta_e`, `omega_e`, one `update`) | `test_traits_contract.cpp` |
| `foc/core/observer/ideal_observer.hpp` | L2 | The reference every other observer is compared against, on a clean plant | `test_ideal_observer.cpp` |
| `foc/core/observer/back_emf_observer.hpp` | L2 | Angle and speed on the SIL plant | `test_back_emf_observer.cpp` |
| `foc/core/observer/nonlinear_flux_observer.hpp` | L3 | 0.1044 rad at 200 rad/s with `i_d = 0`; 2.03× worse with a 10 % current-gain error; **floor near 50 rad/s**; and **it requires `i_d = 0`** -- see the row below | `test_nonlinear_flux_observer.cpp`, `test_sil_sensitivity.cpp`, `test_sil_admission.cpp`, `test_salient_observer.cpp` |
| `foc/core/observer/hfi_observer.hpp` | L2 | Injection-based angle: the error signal is proportional to `sin(2δ)` and the method needs saliency | `test_hfi_observer.cpp`, `test_sil_hfi.cpp` |
| `foc/core/observer/blended_observer.hpp` | L2 | HFI/high-speed handover, with the measured floor (~50 rad/s) stated rather than tuned around | `test_blended_observer.cpp`, `test_sil_observer_blend.cpp` |
| `foc/core/observer/switchable_observer.hpp` | L1 | One observer slot holding the back-EMF, non-linear-flux, injection and blended observers, forwarding to the selected one: its output is reproduced **bit-identically**, a `seed_flux` reaches every observer that takes one, `injection_ab()` is the **selected** observer's carrier (zero for the two that do not inject) so no idle carrier phase advances, and a switch neither resets nor advances anything idle, `hand_over_to()` is the switching call that carries the outgoing estimate into the incoming one (measured: 0.0044 rad against 0.2768 rad for an unseeded `set_kind()`) -- so a hand-over must seed the incoming observer explicitly rather than assume it was tracking | `test_switchable_observer.cpp` |
| `foc/hal/clock.hpp`, `foc/hal/current_sense.hpp`, `foc/hal/dc_link.hpp`, `foc/hal/position.hpp`, `foc/hal/pwm.hpp` | L1 | Abstract interfaces only: no behaviour to claim, and the concepts reject an incomplete implementation | `test_traits_contract.cpp` |
| `foc/driver/stub_hal.hpp`, `foc/driver/stub_pwm.hpp`, `foc/driver/stub_dc_link.hpp`, `foc/driver/sim_position.hpp`, `foc/driver/scripted_current.hpp` | L1 | Host-side stand-ins for the real drivers; they prove the *interfaces* are usable, nothing about the hardware | `test_driver_stubs.cpp` |
| `foc/driver/callback_hal.hpp` | L1 | The same control loop with its measurements coming from a host: 200 duty writes and 200 current reads over 200 ticks, so the loop goes through the caller once per control step rather than through the stub. It says nothing about any particular host's accuracy | `test_host_hal.cpp` |
| `foc/motor/motor.hpp` | L3 | Policy composition, mode transitions, and the reference generator actually reaching the current loop. A position command is admitted when an **absolute angle** exists, which a sensor provides or a turn counter provides once its index origin is fixed -- previously the check was `has_sensor`, which was a consequence of `angle_mech` being a constant zero without one rather than a rule about what position control needs | `test_motor.cpp`, `test_field_weakening_motor.cpp`, `test_position_index.cpp` |
| `foc/motor/foc_loop.hpp` | L2 | Multi-rate loop timing at the SIL plant | `test_sil_multirate.cpp` |
| `foc/motor/motor_config.hpp` | L2 | Loop gains derived from nameplate `J`, `b`, `kt`, with the current loop's bandwidth the only free parameter | `test_motor.cpp` |

Two entries that used to be in this table are deliberately gone. `smo.hpp` never existed as a
shipped header -- the SMO is a **test-harness candidate** (`test_sil_admission.cpp`), and it was
measured to beat the incumbent on the mean-value plant and lose on a switching one, which is why it
does not ship. `benchmark/` is not an installed header and its numbers are timing, not control
performance.

### Operating range, not a single number

Because the low-speed floor is structural, claims are stated as a range:

| Speed | What is claimed |
| --- | --- |
| < 50 rad/s (≈ 480 rpm) | **No angle claim.** The nonlinear-flux observer reads 0.16 rad and above on an ideal plant; use the aligner plus open-loop start |
| 50–800 rad/s | L3 numbers above, with the sensitivity table attached |
| 800 rad/s – base speed (2910 rad/s) | L2/L3, sensitivity growing: iron loss stops being negligible at light load |
| Above base speed | Only with `FieldWeakeningReference`; L3 on a plant whose `Ld/Lq` is 1.0 |
| **Any speed, with `i_d != 0`** | Below ~1000 rad/s, using d-axis current costs angle: 0.1044 rad becomes **0.5079 rad** with `i_d = -1 A` at 200 rad/s, on a *surface* machine with no saliency involved. Above ~3000 rad/s the back-EMF dominates and it is harmless (0.0705 rad at 3200 rad/s with `i_d = -2.98 A`). So MTPA and this observer are mutually exclusive **at low speed**, and the mechanism is the observer's constant `psi_f` target, not the saliency ratio |

### Model reconciliation with a third party (manual, not in CI)

The SIL plant is our own model, so "the plant and the controller agree" is a statement about one model. `motulator`
(Aalto Electric Drives) provides an independent one, laboratory-validated, and `benchmark/script/reconcile_model.py`
drives both with the same stator voltage and the same speed and compares the current vector step by step:

| Quantity | Worst deviation over 400 steps | Limit |
|---|---|---|
| stator current magnitude | **0.0067 %** | 1 % |
| stator current angle | **3.0e-5 rad** | 1e-2 rad |

Both integrate forward Euler at 50 us, so with the same equations they should agree to rounding, and they do. This is
not in CI: it needs `pip install motulator`, and a dependency that large does not belong in the gate path. It is
recorded here because the first run **disagreed by 31 %**, and the cause was the harness — `motulator` takes the
stator voltage in stator coordinates while our model works in the rotor frame, so the command had to be rotated out
first. A harness that mixes frames produces a convincing wrong answer; that is worth knowing before trusting any
three-way table.

### Control reconciliation with a third party (manual, not in CI)

The table above reconciles the *machine model*. This one reconciles the *control loop*: two controllers drive
`motulator`'s synchronous machine at four prescribed speeds and accelerations, on a 300 V bus with a 200 Hz current
bandwidth. Ours is `foc.Motor` -- the C ABI through the host HAL, not a Python re-implementation; theirs is
`CurrentVectorControl`. `benchmark/script/reconcile_controller.py`.

| Phase | Acceleration | iq ours | iq motulator | Predicted | Gap |
|---|---|---|---|---|---|
| standstill | 0 | 0.5000 A | 0.5000 A | 0.5000 A | 0.00 % |
| ramp 0 -> 400 rad/s | 2800 rad/s^2 | 0.2890 A | 0.2851 A | 0.2879 A | 0.79 % |
| hold 400 rad/s | 0 | 0.4999 A | 0.4980 A | 0.5000 A | 0.39 % |
| ramp 400 -> 800 rad/s | 2800 rad/s^2 | 0.2889 A | 0.3189 A | 0.2879 A | 6.01 % |
| hold 800 rad/s | 0 | 0.4996 A | 0.4844 A | 0.5000 A | 3.03 % |

The command is 0.5 A. The ramp rows are not 0.5 A and should not be: a PI current loop has no steady-state error
against a step disturbance but a finite one against a ramp, since the back-EMF changes at `psi_f * alpha` and the loop
has finite gain below the break frequency,

    delta_i = psi_f * alpha / k_i,    k_i = alpha_c * R_s

which at 2800 rad/s^2 gives 0.212 A, i.e. 0.288 A. That number was written down before the run and both loops land on
it -- ours to 0.2 %, theirs to 1.3 %, and within 0.8 % of each other.

Two harness defects had to be cleared before that sentence could be written, and both had the same shape as a finding
about *their* controller:

* An earlier version integrated the mechanical equations instead of prescribing the speed, and reported iq at 0.66 of
  the command. That is the ramp error above seen through a different lens -- with the machine free to accelerate it
  looks like a gain error -- so a criterion of "must equal the command" would have condemned a correct loop.
* Their loop's feedback voltage has to be what their PWM reports as *realized*, delay compensation and the two-step
  average included. Feeding a reconstruction of the duty instead left a steady d-axis error of 0.39 A that looked
  exactly like a defect on their side.

### The residual d-axis question, answered by their own examples (manual, not in CI)

Their **d** axis carries an offset that appears while the speed is changing (0.010 A when the speed is held at
400 rad/s, 0.127 A while ramping to 800 rad/s, i.e. 0.39 rad of misalignment), while ours stays inside 0.04 rad. The
hand-built adapter could not say whether that was their control law or another artefact of the plumbing, so the question
was put to *their* code: motulator publishes ready-to-run examples -- 29 scripts and notebooks, in `examples/` in the
repository and as a zip on the documentation site -- and `benchmark/script/official_cvc_check.py` runs our operating
point through `model.Drive` + `model.Simulation`, torque mode, free acceleration, with none of this project's code in
the loop.

| Quantity | Their machinery | Closed form |
|---|---|---|
| q-axis current, mean over the second half | 0.3228 A | 0.3245 A |
| d-axis current, mean / at 450 rad/s | -0.0412 A / -0.0537 A | -- |
| misalignment | -0.127 rad | -- |

Two things follow. The q axis lands on the same PI ramp law through a completely different code path, which is the
reconciliation the table above was after. And the d offset is theirs: it appears with no adapter in the path. What the
adapter does *not* reproduce is its magnitude -- smaller under free acceleration, larger under the prescribed ramp --
so the table's d column stays indicative rather than asserted. motulator 0.19 reproduces the same numbers (0.3227 A /
-0.0533 A), so this is not an artefact of the 0.5.0 that these scripts pin.

### The two paths into the loop agree exactly (in CI)

`foc_motor_create_host()` and the compiled-in `sil::Bench` reach the same control loop two ways: the first through the
C ABI with `foc_hal_callbacks_t`, the second through `PlantHal` as a compile-time policy. `test_sil_api_equivalence.cpp`
runs both against the same plant, with the same command, and compares all 4000 samples:

| Quantity | Worst gap over 4000 samples | Bound |
|---|---|---|
| q-axis current | **0.000e+00 A** | 1e-4 A |
| d-axis current | **0.000e+00 A** | 1e-4 A |
| speed | **0.000e+00 rad/s** | 1e-3 rad/s |

Bit-identical, which is what "the seam is transparent" means. Two things that this test found while being written are
worth knowing before writing another host:

* `pwm_enable` / `pwm_disable` are **optional** callbacks, and a plant that models the inverter's gate enable needs
  them. Omitting them leaves the gate disabled, `advance()` integrates zero voltage, and the loop reports `Run` with no
  fault while producing no torque at all -- healthy-looking and completely dead.
* `pwm_enable` answers an `int32_t` (0 = ok) while `pwm_disable` answers `void`: the library can be told that a gate
  refused, but not that it failed to open.

### Tuning and identification from outside the library (in CI)

Four C entry points exist so a tool that is not written in C++ can still reproduce, rather than re-derive, what the
library does: `foc_current_pi_gains()`, `foc_speed_pi_gains()`, `foc_rs_estimate()` and `foc_flux_estimate()`. The gain
queries answer all six numbers a PI loop needs -- `kp`, `ki` and the four clamps -- from the same resolver the motor
constructor uses, so a tuning tool and the shipped loop cannot drift apart. Both are bound to Python and covered by
`test_api_identification.cpp` and the binding cases.

Writing those tests produced a general fact about the tuning rule that is worth more than the entry points themselves.
`current_pi_config` cancels the electrical pole with `kp = 2*w*Ld - Rs`, which goes to zero at `w = Rs/(2*Ld)` and is
floored there. Below that floor the loop is integral-only and does **not** deliver the bandwidth it was asked for:

| Requested bandwidth | kp | ki | Measured 63 % rise | 1/bandwidth |
|---|---|---|---|---|
| 200 rad/s | 0.0000 | 16.00 | 31.30 ms | 5.00 ms |
| 625 rad/s | 0.0000 | 156.25 | 3.50 ms | 1.60 ms |
| 1000 rad/s | 0.3000 | 400.00 | 1.30 ms | 1.00 ms |
| 2000 rad/s | 1.1000 | 1600.00 | 0.40 ms | 0.50 ms |

On the nominal machine `Rs/(2*Ld)` is 625 rad/s, and asking for 200 rad/s buys a loop six times slower than the request.
The test therefore asserts the design rule exactly, asserts that the request is honoured in the middle of the range, and
asserts that it is *not* honoured below the floor -- so the next reader finds the boundary instead of a surprise. The
warning is also in the Doxygen for `foc_current_pi_gains()`.

### What a commissioning sweep can actually see (manual, not in CI)

`benchmark/script/identify_parameters.py` runs the identification chain end to end: it produces the settled sweeps a
commissioning routine would record, identifies the machine two independent ways -- `foc.rs_estimate()` /
`foc.flux_estimate()` from the C ABI, and a `scipy.optimize.least_squares` fit of the same samples -- and reports where
the two must agree and where neither can help.

| Experiment | Injected | Library | scipy |
|---|---|---|---|
| standstill sweep, `Rs` | 0.500000 ohm | 0.500000 | 0.500000 |
| standstill sweep, constant loss | 0.6400 V | 0.6400 | 0.6400 |
| zero-current sweep, `psi_f` | 0.004762 Wb | 0.004762 | 0.004762 |
| zero-current sweep, offset | 0.3000 V | 0.3000 | 0.3000 |

The more useful half is what the sweep cannot see.

**A rotor-angle error scales both parameters.** At standstill, an unmodelled rotor offset of 10 degrees biases `Rs` by
+1.54 % and `L` by +1.61 % -- `1/cos(10 deg) = 1.0154` on both, because the fit only sees a smaller excitation and
cannot tell that from a larger resistance. It does **not** mix `Ld` with `Lq`: a salient machine (`Lq = 2*Ld`) gives
exactly the same numbers, since the axes are decoupled at zero speed. Separating the two inductances needs speed or a
second excitation axis, not a better standstill fit.

**A short capture separates `Rs` from `L` only weakly, and noiseless data hides it.** Over ten milliseconds the
`Rs`/`L` correlation is -0.32; over one time constant the condition number rises from 210 to 4620, yet the noiseless fit
still lands 0.3 % from the truth. With 20 mA rms of the noise a shunt and an ADC actually have, eight repeats give:

| Capture | `Rs` spread | `Ld` spread |
|---|---|---|
| one time constant | 1.35 % | 0.76 % |
| ten time constants | 0.12 % | 0.30 % |

So the practical rule the numbers support: record at least ten electrical time constants, and treat any estimate from a
shorter capture as a statement about `Rs/L` rather than about either one.

### Auto-tuning is a measurement, not a preference (manual, not in CI)

`benchmark/script/autotune_current_loop.py` steps the q-axis current through the real loop (`foc.Motor` on its host HAL)
across the bandwidth range and reports what the drive actually does. The first column pair is the reason the script
exists: what was requested is not what is delivered, and the two disagree at **both** ends of the range.

| Requested | kp | Overshoot | 2 % settling | Achieved | Ratio |
|---|---|---|---|---|---|
| 700 rad/s | 0.06 | 0.0 % | 163 ticks | 364 rad/s | **0.52** |
| 1000 rad/s | 0.30 | 0.0 % | 96 ticks | 769 rad/s | 0.77 |
| 1500 rad/s | 0.70 | 0.0 % | 44 ticks | 1667 rad/s | 1.11 |
| 2200 rad/s | 1.26 | 1.1 % | 17 ticks | 2857 rad/s | 1.30 |
| 2600 rad/s | 1.58 | 2.3 % | 25 ticks | 4000 rad/s | **1.54** |

The lower edge is the `Rs/(2*Ld)` floor from the tuning section above: just above it `kp` is still small enough that the
request is under-delivered. The upper edge is the sampling rate: at 2600 rad/s the 63 % crossing falls between two
control periods, the ratio jumps, and the settling time gets *worse* than at 2200. So "deliverable" is a band, not a
half-open interval, and the script now asserts its two edges instead of averaging across them.

Under parameter error -- the controller tuned from the nameplate, the plant the truth -- the loop is forgiving:

| Requested | Worst overshoot over `Rs`/`L` each wrong by 50 % |
|---|---|
| 1000 rad/s | 4.7 % |
| 1500 rad/s | 4.0 % |
| 2000 rad/s | 4.8 % |

Which yields the recommendation the script prints rather than assumes: **2000 rad/s**, 3.2x the floor and 10 control
periods per time constant, 0.5 % overshoot with the right parameters and 4.8 % with both wrong by 50 %. The rule it
encodes is worth more than the number: pick the highest bandwidth that still meets the overshoot bound *with the plant
50 % wrong*, and never accept a bandwidth the loop cannot deliver.

### The commissioning chain, end to end (manual, not in CI)

`benchmark/script/commission.py` runs identify -> tune -> verify with one rule: **the controller never sees the truth.**
It identifies `Rs`, the constant loss and `psi_f` from the sweeps, tunes the loop from *those* numbers, and then steps
the real loop against a plant that kept the truth. So whatever the identification got wrong shows up as a closed-loop
number instead of staying in a table.

| Rotor-angle error | Identified `Rs` | Error | Tuning floor | Bandwidth chosen | Overshoot | vs. truth-tuned |
|---|---|---|---|---|---|---|
| 0 deg | 0.50000 ohm | 0.00 % | 625 rad/s | 2000 rad/s | 0.5 % | +0.00 pp |
| 5 deg | 0.50191 ohm | 0.38 % | 627 rad/s | 2008 rad/s | 0.5 % | +0.03 pp |
| 10 deg | 0.50771 ohm | 1.54 % | 635 rad/s | 2031 rad/s | 0.6 % | +0.14 pp |

A 1.54 % identification error costs 0.14 percentage points of overshoot and no settling time at all. That inverts the
usual priority: the loop tolerates 50 % parameter error (`Rs` and `L` both wrong) and still overshoots under 5 %, so
identification accuracy is **not** the binding constraint -- what matters is that the sweep is one the estimator will
accept. Which is why the chain refuses rather than continues: a flat sweep comes back `valid == False` and the script
stops instead of tuning from a number the estimator already called meaningless.

The inductances are the honest gap: they cannot be separated at standstill (see the previous section), so the chain
takes them from the nameplate and says so, rather than presenting a standstill-derived `Ld` as if it meant something.

### Which observer, and where its ceiling is (in CI)

`test_observer_screening.cpp` measures both single-`L` observers on the same plant, on the two axes that decide the
answer. The criterion written before that run ("the shipping observer stays inside 0.15 rad at every speed with
`i_d = 0`") was wrong, and the way it was wrong is the finding: it described `NonlinearFluxObserver`, while the
observer that actually ships -- in `sil::Bench` and in the C ABI -- is `BackEmfObserver`. Surface machine, `iq = 0.5 A`,
angle error in rad, ceiling 0.15:

| Speed | `BackEmfObserver` | `NonlinearFluxObserver` | `BackEmfObserver`, `i_d = -1 A` | `NLFO`, `i_d = -1 A` |
|---|---|---|---|---|
| 20 rad/s | 0.9668 | 0.2874 | 1.5142 | 1.5317 |
| 50 rad/s | 0.4569 | 0.1813 | 1.1075 | 1.1382 |
| 200 rad/s | 0.1224 | 0.1044 | 0.4186 | **0.5079** |
| 800 rad/s | 0.0306 | 0.0568 | 0.1178 | 0.1531 |
| 3200 rad/s | 0.0968 | 0.0128 | 0.0291 | 0.0058 |

What the table settles:

* **The screening rule is one sentence.** D-axis current is admissible only above a speed: for the shipping observer,
  200 rad/s unloaded but 800 rad/s with `i_d = -1 A`. The same current at 3200 rad/s costs 0.0291 rad. Saliency is not
  a screening axis at all -- at `i_d = 0` a 2.5x change in `Lq/Ld` moves the shipping observer by 16 % and `NLFO` not at
  all, because the model error is `(Ld - Lq)*i_d`.
* **The two observers are not ranked.** `NLFO` is far better at 20 rad/s (0.2874 against 0.9668) and at 3200 rad/s
  (0.0128 against 0.0968), and worse at 800 rad/s unloaded (0.0568 against 0.0306). Neither dominates.
* **The recorded 0.5079 rad is `NLFO`'s**, reproduced here to four digits, which is also what confirms this rig matches
  the one those records came from. The plan notes that quote observer limits were describing the non-shipping observer.

### Claims deliberately not made

These are the claims the evidence levels do not support, listed so nobody has to guess whether they
were overlooked or rejected:

| Claim | Why it is not made |
| --- | --- |
| "Efficient" / "high efficiency" | Nothing here measures input power against shaft power. A modulator and a control loop set the *shape* of the voltages, and efficiency lives in the switching devices, the winding and the modulation index, none of which this project models or measures. Even the SIL plant's loss model is copper plus an optional iron-loss shunt, and the loss numbers are L2 estimates on a plant that has never been current-probed. |
| "Thermal protection" in the sense of knowing the winding temperature | `protection.hpp` integrates `i^2 - q/tau` in A^2*s. That is a lumped copper-loss budget with one time constant; it is not a temperature. `t_derate_*` thresholds are supplied by the caller. Without a thermistor or a thermal observer, this cannot tell you how hot the motor is. |
| "Works down to a few milliamps" | The current-loop evidence is at 0.1 A and above. Below that the resolution of the *model* is the limit, not the ADC: 12-bit quantisation over a +/-10 A range was measured and found to have no effect, but that says nothing about a real sensor's noise, offset drift and amplifier settling. |
| "A sensor-less low-speed speed servo is as good as a sensored one" | Not claimed, and now measured. At 20 rad/s mechanical (140 rad/s electrical) the sensored servo holds 0.000057 rad/s while the sensor-less one holds 0.168 -- a factor of 3000. The angle is **not** the reason: the injection observer reads 0.0015 rad there, which costs one part in a million of torque. The reason is that the speed loop closes on `omega_e / pole_pairs`, and that estimate is itself off by up to 0.7995 rad/s at this speed. The loop does not add to it (0.168 < 0.7995, so it filters rather than amplifies), but it cannot remove it either. Improving low-speed servo accuracy means improving the low-speed **speed** estimate, not the loop. And tuning that estimate's PLL is not enough either: over a 15-point (bandwidth, damping) grid the best point beats the default by **2.75x** (0.2903 against 0.7995 rad/s), the usable band is 10-40 Hz, and 80 Hz is unstable (angle 0.104-0.444 rad, estimate error up to 102 rad/s). The limit is the estimator's structure, not its parameters |
| "MTPA works with the shipped observer" | Not below ~1000 rad/s, and the library now refuses rather than trusting the user to know. MTPA's whole point is `i_d < 0`, and the observer's flux target is a constant `psi_f`; the measured allowance for the 0.15 rad ceiling is 0.04 A at 100 rad/s, 0.12 A at 200, 0.29 A at 400 and 0.92 A at 800, and at `i_d = -1 A` the angle reads 0.5079 rad at 200 rad/s. `MtpaReference::Config::i_d_per_speed_a_s_per_rad` defaults to **0**, so no d-axis current is applied at any speed until the caller states a rate; the same rate buys the full curve at high speed and nothing at standstill. A machine with a position sensor can raise it without limit. |
| "The single-`L` observer has a saliency limit" | Withdrawn, because it was measured to be false as stated. With `i_d = 0` the error is flat in `Lq/Ld` (0.1044 / 0.1048 / 0.1035 at ratios 1 / 2 / 2.5) -- a single-`L` observer's model error is `(Ld - Lq)*i_d`, so zero d-axis current hides all of it. The limit is `i_d`, not the ratio. |
| "The speed loop is validated past base speed" | Not established. Field weakening itself does engage end to end through `Motor` -- 2 ms after parking at 3200 rad/s with the setpoint above it, `i_d` is -1.93 A while the drive is still at 3178 rad/s. But at that operating point the **q-axis current settles negative** (-1.54 A) while the speed error is positive, which the PI's contract says cannot happen. That is an open measurement, not a resolved one: the reference generator and the voltage budget are validated, the outer loop in that regime is not. |
| Anything about a specific motor | Every number is a property of `sil_pmsm_plant.hpp` with the nominal `MachineSpec`, or of a plant varied deliberately. `BoardSelfTest` and `check_board()` exist to tell you how far your hardware is from that model, and they are L3 -- themselves unproven on metal. |

### Not a safety component

This library is not certified, not redundant, and not fail-safe. `protection.hpp` is a *supervisory*
layer that assumes the MCU, the gate driver and the sensing are already working; it cannot protect
against a failed current sensor, a shorted bridge, or a lost PWM. Do not use it in any application
where a wrong torque command can injure someone. The `latching` behaviour is there to make a fault
inspectable, not to make it safe.


## Agentic Coding Workspace

The template reserves `/workspace/` (declared in `.gitignore`) for agent-generated work products produced
during Agentic Coding sessions — e.g., implementation plans, test contracts, audit reports, and other
intermediate artifacts. Keep such working files under `/workspace/` so they never pollute the tracked
source tree.

## Skills & Agent (any harness)

The template ships a library-level Agent Skills system — **15 task skills plus one routing agent**
(`het-agent`), canonical under `.github/skills/`. The same bodies are exposed to each harness through
generated views, never copies. Default `sync` creates **one root directory** (`.agents/`); Claude Code
and Codex views are optional:

| Harness | Native view | How to invoke | Default? |
|---|---|---|---|
| DSH | `.agents/skills` | `skill` tool with the exact name, or `/het-build` | yes |
| Cursor | `.agents/skills` | `/het-build` or `@het-build` | yes |
| Gemini CLI | `.agents/skills` alias | automatic `activate_skill` | yes |
| GitHub Copilot | `.github/skills` (VS 2026 also `.agents`) | `/het-build` in chat | canonical |
| Claude Code | `.claude/skills` | `/het-build` | optional (`sync --harness claude`) |
| Codex | `.codex/skills` | `/use het-build` | optional (`sync --harness codex`) |
| Any other harness | direct read | read `.github/skills/<name>/SKILL.md` | — |

In this template repository the canonical tree is `.github/skills/` and the view is a
symlink, so materialize it once after cloning:

```bash
python .github/skills/_shared/skillkit/cli.py sync
python .github/skills/_shared/skillkit/cli.py validate --strict
```

A repository **created from this template** needs none of that: the `Template Bootstrap`
workflow commits a byte-identical `.agents/skills` view right after instantiation, so
`git clone` → open in DSH/Cursor/Gemini works immediately.

`skillkit` refuses to let a view drift: a link must point at the canonical tree and a
copy must be byte-identical, so `.github/skills/` stays the only body you edit.
There are **no root-level `AGENTS.md`/`CLAUDE.md`/`GEMINI.md` files**: the only default root addition is
the git-ignored `.agents/` directory.
Examples: `het-guide` for onboarding, `het-testgen` to generate tests, `het-agent` for composite tasks
(e.g. *"add a module, test it, and commit"*). See `.github/skills/README.md`,
`.github/skills/_shared/routing.md`, and `.github/skills/_shared/harness-adapters.md`.

## Build Requirements

- Python 3.10+ (Conan tooling)
- Conan 2.0+
- Compatible C/C++ compiler:
    - GCC
    - Clang
    - MSVC
- CMake (auto-installed by Conan)

## Documenting Requirements

- Doxygen
- Graphviz
- sphinx
- sphinx-intl
- sphinx-rtd-theme

## Test Requirements

- GTest

## CI/CD (metadata-driven)

All GitHub Actions workflows are orchestrated from `metadata.json` (entry: `ci-orchestrator.yml`, decision:
`metadata-controller.yml`). Each pipeline is gated by a `workflow_triggers.*` switch and triggered by a
gitmoji in the commit message. The canonical commit form is `<type>(<emoji>): <description>` — the emoji
sits in the parentheses right after the commit word (e.g. `feat(:fire:): ...`, `chore(:package:): ...`);
as a **soft rule** the emoji also triggers from anywhere in the message:

| Pipeline | gitmoji in commit message | Gate (`metadata.json`) |
|----------|---------------------------|------------------------|
| Build | `:building_construction:` | `workflow_triggers.build` |
| Tests + coverage | `:beer:` | `trigger_tests` / `activate_code_coverage` |
| Release | `(:package:):` | `workflow_triggers.release` + the gitmoji in the pushed commits |
| Docs | `:book:` | `workflow_triggers.docs` |
| Security / lint | `:shield:` | `workflow_triggers.security_scan` (also runs on every PR) |
| Online cross-compile | `:hammer_and_wrench:` | `workflow_triggers.cross_compile` (Conan cross-build from a generated profile; decoupled: the emoji starts only this one) |
| Board cross-build | `:fire:` (or `🔥`) | hetai self-hosted runner |

> **Note**: `workflow_triggers.build` / `.tests` / `.docs` / `.security_scan` / `.cross_compile` are
> enabled by default (commit-lint & schema gates always run on push/PR; build/tests/security
> shift-left on PRs).
> `release` needs both halves: the switch is the standing permission, the gitmoji is this push's
> intent. `build_type` is the project's default build type and gates nothing.

## Crash Course of Build

### 0. Use the library (C++ path / C path)

See [docs/sphinx/source/quickstart.rst](docs/sphinx/source/quickstart.rst) and
[docs/sphinx/source/usage.rst](docs/sphinx/source/usage.rst): both entry points, the link rules
(`foc::foc`, C users still link the C++ runtime), the isolation rules, and the honest list of what
this library does **not** verify yet (conventions and alignment traps are in
[docs/sphinx/source/conventions.rst](docs/sphinx/source/conventions.rst)).

### 1. Build then test your library

inplace build and test

```bash
conan create . -s build_type=Debug --build=missing
```

cross-build to host device (assume toolchain and profile are ready):

```bash
conan create . -pr:b=default -pr:h=arm_profile -s build_type=Debug --build=missing -tf=""
```

### 2. Build documentations

```bash
python ./docs/build.py
```

### 3. Full local pass

Steps 1-2 end to end, then drop the local package. `build_type` is the one `metadata.json` declares,
and the package name is yours — there is nothing to edit here:

```bash
conan create . -pr:b=default -pr:h=default -s build_type=Debug --build=missing
python ./docs/build.py
python ./test_package/conanfile.py
conan remove "<your-package>/*" --confirm
```

### 4. Add requirements

Add your required libraries in *conandata.yml* where dependency graph is automatically computed from, then 
modify the **dependencies** field in *metadata.json* to config proper package names and associated targets to 
link (no need modification on *CMakeLists.txt*).

Requirements for your project can be the package archived on [Conan Center](https://conan.io/center), or user 
built ones. If the later one, at least you need a locale Conan server for managing your libraries.

### 5. Run MegaLinter locally (blocking SAST)

MegaLinter mirrors the CI `megalinter` job (`.github/workflows/security-linters.yml`): same config
(`.github/misc/.mega-linter.yml`), same full image, and the same version (`v8.8.0`, pinned to match the
CI action). It runs the SAST layer — gitleaks / semgrep / checkov / devskim, plus clang-format
& cppcheck. The **required** gates are the native `quality-gates` job of the same workflow: format, tidy,
guard (layering + CI policy), strict, license, claims (including the `--impact` dependency query), docs,
boundaries and capabilities — all with pinned tool versions, all runnable locally through
`python3 .github/misc/gates.py <gate>`. Two more are real but not in this job: `fmu` runs on the Ubuntu
leg of the test matrix because it needs the archive that build produced, and `pil` is a **local** gate —
it rebuilds the library for aarch64 and runs the target binary under qemu, which is minutes per push for
evidence that only changes when the control loop does. `gates.py all` runs everything, and `guard` fails
the build if a workflow ever calls a local-only gate.

Prerequisites: Docker (or Podman) + Node.js ≥ 20.

```bash
bash .github/misc/run-megalinter.sh             # lint the whole codebase
bash .github/misc/run-megalinter.sh --fix       # auto-apply fixes (e.g. clang-format)
bash .github/misc/run-megalinter.sh src/foo.c   # lint selected files only
CONTAINER_ENGINE=podman bash .github/misc/run-megalinter.sh   # Podman users
```

- The first run pulls the `v8.8.0` image (a few GB — cached on later runs).
- Reports are written to `megalinter-reports/` (git-ignored); per-linter details are in
  `megalinter-reports/linters_logs/ERROR-*.log`.
- The runner (`mega-linter-runner`) is a devDependency — `npm ci` installs it, or it is fetched
  automatically on first use.

## All-in-one Project Structure

```
project-root/
├── conanfile.py              # Conan recipe
├── CMakeLists.txt            # CMake build framework
├── metadata.json             # Project metadata configuration (single source of truth)
├── conandata.yml             # Dependency specifications, Conan plugin support
├── package.json              # Node-side tooling deps (commitlint / semantic-release / ajv)
├── CHANGELOG.md              # Auto-generated by semantic-release (init tag required)
├── build                     # One-liner local build script
├── LICENSE                   # Apache v2 Project license
├── NOTICE                    # Notice file of Apache v2
├── .vscode/                  # Recommended editor config (extensions / settings / tasks)
├── .agents/skills            # Default generated view (git-ignored) → .github/skills
├── .claude/skills            # Optional generated view (git-ignored) → .github/skills
├── .codex/skills             # Optional generated view (git-ignored) → .github/skills
├── .devcontainer/            # Reproducible dev environment (Dockerfile)
├── .github/                  # CI/CD + canonical agent skills
│   ├── workflows/            # ci-orchestrator / metadata-controller / build / test / docs / release / security
│   ├── skills/               # canonical: 15 task skills + het-agent + _shared + manifest.json
│   │   └── _shared/skillkit/ # validate / sync / catalog tool
│   ├── misc/                 # clang-format(-c/cpp) / clang-tidy / gitleaks / releaserc / commitlint /
│   │                         # metadata.schema / mega-linter / pre-commit / labels / Codegen-Starter.txt
│   ├── CODEOWNERS            # Required reviewers
│   ├── CONTRIBUTING.md       # Contribution & commit conventions
│   ├── dependabot.yml        # Grouped monthly dependency updates
│   └── PULL_REQUEST_TEMPLATE.md
├── api/                      # Interface to advanced programming language
│    └── python_bindings.cpp  # Python bindings interface
├── include/                  # Public headers
│   ├── *.h                   # C interface headers
│   └── *.hpp                 # C++ interface headers
├── src/                      # Implementation files
│   ├── *.c                   # C sources
│   ├── *.cpp                 # C++ sources
│   └── *.ixx/*.cppm          # Auto-generated Module files (in experimental)
├── benchmark/                # Cross-compile & on-board benchmark framework (Cortex-M / Cortex-A)
├── .hetai/                   # hetai package matrix (cross-compile targets)
├── wokspace/                 # Agentic Coding work products (git-ignored)
├── docs/                     # Documentations root
│   ├── doxygen/              # Doxygen system main root
│   │   ├── dox/              # Pure documentations' folder (the only home of .dox/.cxx)
│   │   │   ├── demos/        # Examples catalogue (hand-written, standalone)
│   │   │   │   ├── *.dox     # Documenting docstring
│   │   │   │   └── *.cxx     # Example codes
│   │   │   └── *.dox         # Main pages and etc
│   │   └── ...
│   ├── sphinx/               # Sphinx system main root
│   │   ├── source/           # Source files of sphinx system
│   │   ├── locales/          # Pot files for internalization
│   │   └── ...
│   └── images/               # Static images for doxygen/sphinx system
└── test_package/             # Test project
    ├── export/               # Log for testing results
    ├── resources/            # Test resources for test_package/ programs
    ├── stress/
    │   └── *.cpp             # Scripts for stress testing
    ├── unit/
    │   └── *.cpp             # Scripts for unit testing
    ├── main.cpp              # Validation program for package
    ├── conanfile.py          # Conan recipe for test_package
    └── CMakeLists.txt        # CMake build workflow for test_package
```

> **Suffix rule**: `.dox` and `.cxx` are **documentation-only**. They live exclusively under
> `docs/doxygen/dox/` as hand-written standalone files (`mainpage.dox`, `demos/*.dox`,
> `demos/*.cxx`) and are never written as a docstring inside `include/` or `src/`, which carry
> `.h/.c/.hpp/.cpp` only. A complete doc set is a landing page plus a tutorial; example code is
> optional (0..N `.cxx` per `.dox`, no pairing) and `docs/build.py` fails the build when any of
> those properties breaks.

## Module Generation (experimental)

When `generate_modules_inplace` is enabled in `metadata.json`:

1. Header/source pairs automatically generate module files
2. `#include` directives are converted to `import` statements
3. Doxygen annotations control symbol visibility:
    - `@exporter`: Exports symbols in modules
    - `@attacher`: Attaches symbols to modules

This feature is experimental now, however, the specific syntax can make the existing project a ease 
migration to fit the future C++ standard.

## Compiler Support Matrix

This states what CI **actually builds and runs**, not what the language permits. The runner labels live
in `.github/workflows/`; `build_type` / `build_cppstd` / `build_cstd` live in `metadata.json`.

| Feature          | MSVC (`windows-latest`) | Apple-Clang (`macos-latest`) | GCC (`ubuntu-latest`) |
|------------------|-------------------------|------------------------------|-----------------------|
| Build, link, run | ✓                       | ✓                            | ✓                     |
| C Compatibility  | ✓                       | ✓                            | ✓                     |
| Automatic Export | ✓                       | ✓                            | ✓                     |

**C++ Modules are experimental and are not exercised by CI**: `generate_modules_inplace` is `false`, so
no leg compiles a module interface. Treat the feature as unverified until a leg covers it.

## Platforms Support

- **Desktop**: Windows, Linux, macOS — each is a build-and-run CI leg
- **Embedded / cross-compiled**: `arm-linux-a53` (Cortex-A, aarch64 Linux) and `mcu-m4` (Cortex-M
  baremetal, `arm-none-eabi`) — see `.github/workflows/cross-compile.yml`
- Any other target works through a generated Conan profile, but is **not covered by CI** until a leg is
  added for it.

## Downstream Contract (HeT DevTools)

This template is also built by the **HeT DevTools** VS Code extension in a *managed lane*: it creates its
own private venv / `CONAN_HOME` and a **generated** Conan profile instead of probing the user's machine.
The interfaces below are **backward compatible** — by default the behaviour is byte-for-byte the CI one.

| Interface | Value | Default | Meaning |
|---|---|---|---|
| `HET_CMAKE_MIN` | version string (e.g. `3.15`) | unset | Overrides the `cmake_minimum_required` floor (CI baseline is 3.28) |
| `HET_CMAKE_BUILD_REQUIRE` | `none` or a version | unset | Skips / replaces the ConanCenter `cmake/<metadata.cmake_version>` (read by **both** recipes) |
| `metadata.graphviz_bin` | directory path | **absent** | Absent = use `dot` from `PATH`. This is a **machine-specific** field and normally should not be committed |
| `conancenter` remote name | keep the standard name | — | A downstream private `CONAN_HOME` may point it at a corporate mirror — never assume its URL |
| Coverage artifact | `test_package/test/export/coverage/coverage_summary.json` | — | **Parse the JSON, not the HTML.** The `coverage_report/index.html` path stays a stable artifact, but its layout belongs to genhtml and may be restyled at any release. CI publishes the lcov leg as the canonical `Coverage-report` artifact and keeps `Coverage-report-<os>-<ts>` as per-leg evidence |
| `user.het:run_tests` | conan conf (`-c user.het:run_tests=False`) | `True` | Narrows `metadata.trigger_tests` for one run and leaves the library uninstrumented — what the build-only CI legs do. Namespaced to the template, not to the package, so a rename never has to touch it |

(The implementations live in `CMakeLists.txt`, `conanfile.py`, `test_package/conanfile.py` and `docs/build.py`.)

## Benchmark (on-board)

`benchmark/` cross-compiles the library to real hardware and measures performance on-board:

- **Cortex-M (baremetal)**: flash via JLink / OpenOCD / PyOCD, timing via SYSTICK
- **Cortex-A (Linux)**: deploy via ADB / SSH, timing via `clock_gettime`

Edit `benchmark/bench_config.json` (board parameters) and `benchmark/bench_entry.c` (algorithm cases),
then run `python benchmark/script/run_bench.py` (add `--no-flash` to build only). Results follow the
`BENCHMARK_START / RESULT|name|cycles / BENCHMARK_END` protocol. See `benchmark/README.md`.

> Role split: `test_package/` runs **host-side** GTest verification (desktop/x86_64, even when the library
> is cross-compiled to baremetal), while `benchmark/` runs **target-side** on-board verification.

## Commit Convention & Versioning

Commit style uses the [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/)
specification, extended with the template's private **emoji superset** (see the CI/CD table above) so a
single message both bumps the version and triggers the right pipeline.

**Canonical commit form** — the emoji is placed in the parentheses right after the commit word:

```
<type>(<emoji>): <description>
```

Examples: `feat(:fire:): cross-compile support`, `test(:beer:): vector add cases`,
`chore(:package:): prepare release`. As a soft rule, the emoji anywhere in the message also triggers the
pipeline, but the parenthesized placement is the recommended convention.

Versioning is driven by **semantic-release** (`semver-release.yml` + `.github/misc/.releaserc.json`): the commit prefix
decides the jump — `feat` → minor, `fix`/`perf` → patch, `BREAKING CHANGE`/`!` → major. A release
generates `CHANGELOG.md` and rewrites the `version` in `metadata.json`. (The legacy
commit-base-versioning mechanism has been removed.)

## To Do Things

- MSVC has no coverage backend configured, so the test + coverage chain runs on `ubuntu-latest` and
  `macos-latest` only (`.github/workflows/full-test-automation.yml`). A Windows leg needs MSVC
  instrumentation first.

## License

[Apache-2.0] - See included LICENSE file for details.

