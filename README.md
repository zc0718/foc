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

**The L1 rows that remain are a conclusion, not a to-do.** Each of them was asked the same
question -- what number would an L2 leg even measure -- and the answer was "none that the code owns":

* **A contract with no behaviour** (`core/traits.hpp`, `core/observer/observer_policy.hpp`,
  `hal/clock.hpp`, `hal/current_sense.hpp`, `hal/dc_link.hpp`, `hal/position.hpp`, `hal/pwm.hpp`) --
  the claim is that a wrong implementation does not compile, and the strongest instrument for that is
  a negative-compile probe, which is exactly what is used. There is nothing left to measure: a
  concept either admits a type or it does not.
* **One formula, no physics** (`core/machine_params.hpp`) -- nameplate to `psi_f` is one definition
  checked by one test. The *number* is the caller's nameplate, so there is no model to measure it
  against; measuring it would measure the input.
* **Dispatch, whose only error mode is binary** (`core/observer/switchable_observer.hpp`) -- output
  reproduced bit-identically, the seed reaching every observer that takes one, the carrier belonging
  to the selected observer. Exact reproduction has no *quantity* whose sensitivity could be measured,
  which is why the row states exactness instead of a tolerance.
* **Host stand-ins and the ABI surface** (`driver/stub_hal.hpp`, `driver/stub_pwm.hpp`,
  `driver/stub_dc_link.hpp`, `driver/sim_position.hpp`, `driver/scripted_current.hpp`,
  `driver/callback_hal.hpp`, `api/foc_api.h`, `api/foc_types.h`, `core/version.hpp`,
  `foc/version.h`) -- they prove the interfaces are usable and the error codes are the ones promised,
  and by construction they say nothing about any particular host's hardware or accuracy. An L2 leg
  here would measure the host, not the drive.

A future L1 row is therefore expected to arrive with the same sentence: *what number would be
measured, and what would the criterion be*. If that sentence can be written, the row does not belong
in this list.

### What each module claims

Every installed header is listed here exactly once, with the level of what is claimed about it and
the thing that checks it. `.github/project/claim_gate.py` enforces that: a header added without a row
fails the build, a row naming a test that does not exist fails the build, and **an `L4` row is
refused outright** because this project has no hardware to measure it on. Paths are relative to
`include/`.

| Module | Level | The claim, in one line | Evidence |
| --- | --- | --- | --- |
| `foc/api/foc_api.h`, `foc/api/foc_types.h` | L1 | The C11 surface compiles as C, links and runs; every misuse is an error code; handles carry their *kind*, so a motor handle cannot be read as a protector. `foc_motor_set_observer()` is the run-time hand-over, refused outside `Run` and refused for a mode this machine cannot run, so a C caller never has to reproduce the seeding rule itself | `test_api_c_smoke.cpp`, `c_api_validation.c` |
| `foc/core/types.hpp` | L3 | The value types the rest of the library is written in, plus the one piece of arithmetic they carry: `ElapsedClock`, an integer nanosecond clock. **Its law is exact and that is the point** -- 20 480 000 periods of 25 us is **512 s to the nanosecond**, reached on tick 20 480 000, where the float sum this replaced reached the same reading on tick **17 866 579** (446.664 s of real time, **12.8 % fast**) and then could not advance at all. So the error source is measured, not argued: a float accumulator of periods changes *speed* with its own magnitude (`ulp(t)/dt` per tick) and freezes once `ulp(t) > dt`, and the quantisation that remains is the conversion of one period, worth **1 ns** against a 25 us tick. Cross-checked against the float version it replaces, which is the falsifier: if a float sum were good enough the two columns would agree. Conversion is split into whole seconds and a fraction, because `seconds * 1e9F` in one float multiply needs 39 bits and left a 520 s recipe 16 us long -- one tick late | `test_transforms.cpp`, `test_elapsed_clock.cpp` |
| `foc/core/limits.hpp` | L3 | `clamp`/`sanitize` behave at the boundaries and at NaN, and the slew limiter is measured as a **step count** rather than as a wave shape. The law is one ceiling, `steps = ceil(travel / (rate * dt))`, checked on **14** (rate, period) combinations from 0.25 to 4 per second against 0.125 to 0.5 s, every one a power of two so the arithmetic is exact and the assertion can be an **equality**: a budget of 1/32 reaches a travel of 1.0 in exactly 32 steps, 1/16 in 16, 1/8 in 8, and the last grid point -- a budget of 2.0 for a travel of 1.0 -- is what shows the ceiling is a ceiling: **one** step, landing on the target rather than overshooting it. **Both factors enter one-for-one, so there is no leverage to hide in**: halving the budget doubles the count (32 -> 16 -> 8), and an error of eps in `rate` or in `dt` moves the count by eps times N and by nothing worse, which is why a ramp can be sized from the nameplate alone. The two sides of the `min` are measured at the boundary itself, where a `>=` written for a `>` would show: inside the budget the output **is** the target on that same call (0.05), exactly at the budget it lands on the target (0.15, not one step short), one step beyond is one budget (0.25, not the target), and a non-positive period changes **nothing** (0.25 for a target of 9.0). **Not** claimed: any filter dynamics or anti-windup property -- this row is the arithmetic of a rate limit, and where the limit sits inside a loop is the loop's claim | `test_limits.cpp` |
| `foc/core/version.hpp`, `foc/version.h` | L1 | One version string, identical from C and C++ | `test_version.cpp` |
| `foc/core/traits.hpp` | L1 | The policy concepts: a wrong policy fails to compile, and the failure is the one intended | `test_traits_contract.cpp`, `negative_compile.py` |
| `foc/core/transforms.hpp` | L3 | Clarke/Park and inverses; the dq derivation is cross-checked against an independently derived phase-variable model at six operating points (currents within 0.02 A, torque within 0.3 %) | `test_transforms.cpp`, `test_abc_plant.cpp` |
| `foc/core/svpwm.hpp` | L3 | "Clamped to the linear range" reads like one number. **It is a hexagon**, and the boundary is the geometry rather than a fit: bisecting the modulator at six angles gives **16.00000 V at a sector centre** (`2/3*v_bus`) and **13.85641 V at the six flat sides** (`v_bus/sqrt(3)`), matching `(2/3*v_bus)*cos(30)/cos(theta' - 30)` to **1e-5 relative at every angle** -- so the amplitude a rotating reference is *guaranteed* is the inscribed circle, and the extra 15 % that reaches 16 V is available only at six angles. Swept over a full turn: at **0.999x** the circle **0 of 72** angles saturate, at exactly the circle **6** do (the six where the circle *is* the floor), and at `2/3*v_bus` **66 of 72** do. **What comes out of the inverter is the vector that went in**: the duties are read back through `(duty - mean)*v_bus` and Clarked, and the result equals the request to **<= 1e-6 V** at seven angles across all sectors -- this modulator has **no gain error** inside its range, the common-mode offset being exactly what the reconstruction removes. **A wrong bus voltage comes out as exactly that error, the wrong way round**: duties are `0.5 + (v - offset)/v_bus_assumed`, so telling the modulator the bus is **low** makes it deliver **more** -- `1/k` measured as **1.111111 / 1.052632 / 0.952381 / 0.909091** for `k = 0.90/0.95/1.05/1.10`, five digits against the law. **Not** claimed: any behaviour *outside* the hexagon beyond "it saturates and clamps" -- over-modulation has no stated gain law here, and the `saturated` flag is the whole contract | `test_svpwm.cpp` |
| `foc/core/pi.hpp` | L3 | `error = setpoint - feedback`; the integral is re-clamped every step so a shrunken window releases torque at once. **The droop law is exact and has no free parameter**: with `ki = 0` a constant disturbance leaves exactly `d/kp` of standing error -- measured **0.100000 / 0.200000 / 0.500000** for three disturbances at `kp = 0.01` -- and with `ki = 1` it leaves `<= 1e-6`. **The textbook second-order law is wrong for this loop and the case proves it with a second model**: on a first-order plant the closed loop is `(kp*s + ki)/(J*s^2 + kp*s + ki)`, which carries a **zero at `-ki/kp`** because the proportional term acts on the reference too, so `zeta = kp/(2*sqrt(ki*J))` mispredicts the overshoot badly -- it says **16.3 %** where the loop does **29.8 %** (`kp = 0.01`), and **0 %** (critical damping) where it does **13.5 %** (`kp = 0.02`). The same loop written as two ODEs and integrated at 10 ns -- which contains the zero by construction -- agrees with the sampled loop to **0.06 / 0.04 / 0.02 %** at `kp = 0.01/0.02/0.03`. **The period error a caller will actually make moves the loop by half of it, the other way**: the integral integrates `ki*e*dt`, so a period `k` times wrong makes the effective `ki` `k` times wrong and the time to first peak goes as `1/sqrt(k)` -- measured **1.0415 / 1.0166 / 0.9793 / 0.9627** against **1.0541 / 1.0260 / 0.9759 / 0.9535** for `k = 0.90/0.95/1.05/1.10`, inside 2 %. **Not** claimed: that the zero is harmful -- it is the setpoint response a step command wants, and taming it is what setpoint weighting would be for, which this controller does not have | `test_pi.cpp` |
| `foc/core/filters.hpp` | L3 | **The declared cutoff is not where the corner is, and the shortfall has a measured law.** `configure()` pre-warps with `alpha = w/(1+w)`, which is faster than the analogue prototype, so the real -3 dB point sits *below* the request and the gap grows with `fc*dt`: **0.9847 / 0.9305 / 0.8751 / 0.7918 / 0.6518** x the request at 100 / 500 / 1000 / 2000 / 5000 Hz on a 20 kHz grid -- a 5 kHz request delivers 3.26 kHz, a third short. **The transfer function itself is verified by sweep against an instrument that shares no code with it**: a sine is driven through the filter, the response and the command are both fit with the least-squares estimator in `sinusoid_fit.hpp`, and the quotient agrees with `norm(H(e^{jw}))` and `arg H` of the recurrence to **0.000 %** and **0.0001 rad** at seven frequencies, with the corner phase at **-0.7051 rad** where the analogue prototype would put `-pi/4 = -0.7854`. **The period error a caller will actually have moves the corner by about that much**: a `dt` a tenth wrong moves it to **0.9105 / 0.9555 / 1.0441 / 1.0877** of nominal for `k = 0.90/0.95/1.05/1.10`, i.e. a coefficient near 1 because `alpha` is built from `fc*dt`. **The moving average has three closed forms and all three are exact**: DC gain 1 to the bit, the first null at `fs/N` measured at **3.1e-17**, a group delay of exactly **(N-1)/2 = 3.5000 samples** read off the phase slope, and a magnitude between the nulls equal to the Dirichlet kernel (**0.90176** both ways). **Not** claimed: that the cutoff shortfall is a defect -- it is the price of a one-multiply recurrence, and the row states it so a user can pick `fc` knowing what they get | `test_filters.cpp` |
| `foc/core/sm.hpp` | L3 | The mode machine, measured as the **graph** its transition table makes rather than as a table of cells. Enumerating the relation on the code (four start states x four targets) and computing over it: **seven legal edges**, out-degree **2 / 3 / 2 / 0**, in-degree **2 / 1 / 1 / 3**, every state reachable from `Idle` by breadth-first search, and `Fault` the **only sink** -- `clear_fault()` owns the edge out of it, which is exactly why that edge is not in the table. **The refusal is a measured zero**: of the sixteen cells, **nine** refuse, and every one of them leaves the state where it was and moves `illegal_transitions()` by **exactly one**, including targets outside the enum (`4`, `200`) -- which is the property that makes the counter a usable diagnostic and not a guess. **The sharp edge is stated as one**: `request(current)` is a **refusal, not a no-op** -- re-asserting a target three times accumulates three illegal transitions -- so a supervisor that re-asserts its command every cycle fills the counter with attempts that mean nothing. **Not** claimed: any timing or safety property of the supervised sequence -- this row is about the machine's own alphabet, and the sequence it drives is `startup`'s claim | `test_sm.cpp` |
| `foc/core/machine_params.hpp` | L1 | Nameplate → derived `psi_f`, one definition | `test_machine_params.cpp` |
| `foc/core/machine_map.hpp` | L3 | A current-dependent machine: rows reproduced exactly, linear between them, **clamped rather than extrapolated** outside the measured range, and an unsorted or degenerate map rejected. It says nothing about where the rows came from -- that is the caller's claim. **The error between rows is the chord error, and it goes as the square of the spacing**, measured against the analytic saturation `L0 / (1 + (i/ik)^2)` with `L0 = 4e-4 H`, `ik = 120 A` and 60 A held as the centre of a chord at every spacing: the interpolation reads **low** by 1.538e-6 / 3.630e-7 / 8.936e-8 H for rows every 40 / 20 / 10 A, against the chord law `f'' h^2 / 8` taken in magnitude (f'' < 0 here) = 1.422e-6 / 3.556e-7 / 8.889e-8, giving measured-to-law ratios of **1.082 / 1.021 / 1.005** and a halving ratio of **4.062** where the law says 4.0. The direction is forced by the curve: a chord lies **below** a concave one, so a table reads the inductance *under*-estimated between its rows, and the sign is checked rather than assumed. **The spacing, not the row count, is what sets the size**: halving it buys a factor of four, which is exactly the number a caller needs to size a table from the accuracy the loop requires. **Not** claimed: any agreement with a real machine between the rows -- the law is measured against a curve whose second derivative is known, which is what makes the law measurable at all, and a real saturation curve has to be measured to be known | `test_machine_map.cpp` |
| `foc/core/startup.hpp` | L3 | The open-loop start runs inside `Motor`'s Align state -- the one state where the angle is forced and the observer is not stepped -- so the hand-over is exactly one tick. With a slow enough ramp the rotor is caught: measured steady slip 0.29 rad at 400 rad/s^2 and 0.20 rad at 200, and the hand-over keeps the machine within 5 % of its speed with no current spike. **Claimed instead** (after three refuted models, see foundation-log A-31/A-32/A-33): the slip is not a steady angle but a lightly damped oscillation. The slip is an electrical angle while the inertia is mechanical, so the rotor equation carries the pole pairs in its stiffness, `delta'' = alpha_electrical - (p*Kt*i/J) sin(delta) + (b/J)(omega_e - delta')`, and the oscillation frequency is `omega_n = sqrt(p*Kt*i*cos(delta*)/J)` about `sin(delta*) = (J*alpha_electrical/p)/(Kt*i)`. Measured 62.8 / 64.4 / 66.0 rad/s against 64.7 / 64.8 / 63.4 predicted at 400 / 200 / 1200 rad/s^2, and dropping the pole pairs from the stiffness misses by sqrt(p) = 2.65x, which the same test rejects. **Not** claimed: the slip *amplitude*, which is set by the initial rotor-to-vector mismatch rather than by the ramp (decay constant `2J/b` is 2 s here, longer than the pull). A start that never caught the rotor is **reported**, not tolerated: one window (20 ms) after the hand-over the published speed is compared with the forced vector's own rate, and less than half of it latches `FaultCode::LostSync` -- the first producer that code has ever had. Measured: 400 rad/s^2 hands over clean, 3000 rad/s^2 faults with `lost sync`. After the hand-over the drive holds a load step: 20 cases (4 levels x 5 instants, up to +-50 % of the torque the pull itself made) are all held within 10 % of the commanded speed with no fault and no silent failure. Beyond the loop's authority the drive **stops with a cause**: `Stall` latches when the current sits at 90 % of its ceiling and the machine is not moving the way it was told to (reversed, or below a quarter of the command) for 100 ms. Measured: 4x the rated torque used to reverse the machine to -895 rad/s in silence; it now faults with `stall` after 100 ms, and the 20 load-step cases above still pass without tripping it. **Independently witnessed**: the recorded duty train of one start, replayed into the phase-variable plant (`sil_abc_plant.hpp` -- three phase fluxes integrated directly, torque from magnetic co-energy, never a d-q quantity and never `Kt`), reproduces the *same* 62.82 rad/s against 64.66 predicted; with the mechanical integration switched off the identical replay returns 197.9 rad/s at a saturated 3.14 rad slip, so the number belongs to the machine and not to the harness. Measured domain boundary: at double inertia the rotor completes 3 crossings instead of 20 (9.4 rad/s against 45.4 predicted), so this is a **locked-rotor** law and is not quoted outside that. **Sensitivity, which is what L3 adds**: eleven model error sources were injected one at a time at the magnitudes `test_sil_sensitivity.cpp` already uses -- d/q saturation 30 % at 1 A, iron loss at 2000 and 200 ohm, Rs +31 % via 100 degC, sensor gain +10 % uniform and u+10/v-10 %, 50 mA common offset, 0.1 A noise, 0.48 V of dead time, 0.02 rad of encoder mounting error. The oscillation frequency stays inside 3.3 % of the clean run for every one of them, and the worst slip stays inside +18.2 %: the one material source is **differential sensor gain**, which the user must therefore measure on their own hardware. The table carries its own two anchors so that 'no effect' means something -- a common offset moves the start by less than 0.1 %, because it cancels identically in Clarke, and a differential gain must do more than a uniform one, because it creates a phantom negative-sequence component. The load-step series below is a pass/fail count rather than a modelled quantity, so no error coefficient applies to it. | `test_sil_startup.cpp` |
| `foc/core/position_reference.hpp` | L3 | Turns an electrical estimate into a position a loop can close on: unwrapped, tracking the truth to **0.000309 rad over five mechanical turns**; an index mark **starts the count** rather than pinning it to a single-turn value (6.2600 rad one turn after the mark); a repeat pass removes drift and does not rewind (6.4028 -> 6.2832 rad for 0.1429 rad of injected drift); and a step too large to unwrap is reported rather than guessed, cleared by the next mark. **Not** claimed: any absolute position before the first index -- there is none. That used to be a sentence a caller had to remember; it is now an API that refuses: `foc_motor_get_position_absolute()` answers `FOC_ERR_STATE` and leaves its receiver untouched until an index has been seen, because the raw field answers 0 before the first mark and 0 is a legal mechanical angle -- "never homed" and "homed, parked at the origin" would be indistinguishable. Measured through the C ABI on a plant-backed, sensor-less host: the origin appears 2.4 ms after the rotor starts turning, the call then answers with the angle and it tracks the rotor (0.0000 -> 2.9872 rad over 10 ms against 3.0 expected). Reaching that second half at all required a capability the ABI did not have: `PositionReference` only advances while the commutation angle comes from the observer, and the callback HAL used to report a sensor unconditionally, so an origin could not exist through C at all -- `use_observer_angle` in the setup is what makes it exist. **The third leg is four identities, because a counting channel is a sum of integers and one division and every coefficient is therefore exact**: the wrap rate is `p*s/(2*pi)` electrical turns per `s` mechanical rad (10.500 / 22.750 / 14.000, exact); the reconstruction's **turn length is `2*pi*p/p'`**, not `2*pi` -- measured 7.3033 against 7.3304 at `p'=6` and 5.4775 against 5.4978 at `p'=8`, so a pole-pair error is a *scale* error whose residue at the next mark is `2*pi*abs(1 - p/p')`, at least `2.22*abs(p'-p)` times the resolution the channel declares, which is why an integer pole error is now **detected at the second mark instead of silently sawn back once per turn**; a mark that fires `j` late displaces the origin by exactly `j`, isolated by differencing a jittered run against a clean one (ratios **0.9975 / 0.9991 / 0.9990** for `j` = 1, 10 and 50 mrad, because the first pass absorbs it into the offset and later passes only remove drift); and a **missed** mark costs **no counts at all** -- the count is carried by the unwrap and the mark only removes drift, measured at 7.09e-3 rad over five turns against a lost count, which costs a whole turn. **Found and fixed (F-45)**: the index snap had no plausibility guard, so a spurious pulse moved the origin by up to half a mechanical turn -- 3.12 rad measured -- with `lost_count()` still false, while the electrical path already reported the same class of impossible jump; the snap now takes the same `0.9*pi/p` rule, and the refused pulse moves the angle by one sample instead. **Stated boundary**: below that threshold the pulse is believed (0.100 rad late moves the angle by 0.09990 and is not reported), because at that size nothing in the channel can tell it from drift | `test_position_reference.cpp`, `test_api_position_absolute.cpp` |
| `foc/core/homing.hpp` | L3 | The search that turns slowly until the index mark arrives, as a decision the caller applies rather than an actuation. An already-homed axis finishes before it starts; the command goes to zero on the exact tick the origin exists; a mark that never arrives times out on its own recipe (crossed at tick 5001 against a 5000-tick recipe) and stops commanding. Driving a sensor-less motor through it end to end finds the mark after **0.172 s** at truth **0.5004 rad** (mark at 0.5000) and then holds that place to **0.0121 rad** over the last 0.2 s. **The budget is set in samples, not in seconds**: 1 rad/s for 1 s buys **0.9999 rad** of travel, not 1.0, because the tick that gives up commands zero speed and the last sample of the recipe is therefore never spent -- a mark exactly at `creep*timeout` is **missed** while one sample inside it is found (1 rad/s: 0.9990 found, 0.9999 missed, 1.0 missed; 10 rad/s: 9.99 found, 10.0 missed), and the travel is exactly `(ticks - 1)*creep*dt`. **And the clock it gives up on used to be a float32 sum, which this row's own measurement condemned**: `elapsed_s_ += dt_s` reached "512 s" after **446.664 s** of real time (`12.8 %` fast, 17 866 579 ticks) and past `2^24*dt` rounded up to a power of two -- 512 s at 40 kHz, 2048 s at 10 kHz, 32768 s at 1 kHz -- could not advance at all, so a 520 s recipe was still creeping after 40 M ticks. It is now a `core::ElapsedClock` (integer nanoseconds, `foc/core/types.hpp`), and the same three recipes fire on the tick their own arithmetic says: **40 000 ticks for 1 s, 20 480 000 for 512 s, 20 800 000 for 520 s**. The finding is recorded as F-105 and the class it belongs to -- a duration that only grows, accumulated in float -- is now refused by `gates.py clocks` in every module, not just this one. **Not** claimed: that it will find a mark outside the commanded direction's travel -- a search that never sees its mark is a machine that needs a person, not more current. **And the origin error is the pulse delay measured in creep speed**: the zero cannot be better than where the axis was when the index was honoured, so a jitter of 0 / 1 / 5 / 20 ticks puts it off by **1.00 / 2.00 / 6.00 / 21.00 samples** of travel at 1 rad/s (`(jitter + 1)*creep*dt`, the extra sample being the crossing detection) and the ratio to `creep*dt` stays **21.000** at 0.5 / 1 / 2 / 4 rad/s -- one-for-one in the creep speed. Since the search *time* goes as `1/creep`, `error x time` is a constant: slowing down buys accuracy in exactly the proportion it costs time, so a recipe cannot be made both fast and accurate, only re-balanced | `test_homing_routine.cpp`, `test_sil_homing.cpp` |
| `foc/core/speed_from_angle.hpp` | L3 | Speed by differencing the angle across a fixed delay, as an alternative to an observer's own loop state. Measured at 20 rad/s mechanical on the sensor-less rig: **0.1960 rad/s worst error against the injection observer's own 0.7995**, a 4.1x improvement -- and it clears the 4x bar only just. The noise model `sigma_theta*sqrt(2)/(N*dt)` predicts 0.030 rad/s; the measurement is 6.5x worse than that point value while still inside a factor-of-10 band, so the angle's error carries a correlated component that differencing does not average away. **Closed-loop result, and it is negative**: closing the library's own speed PI on the differenced estimate settles to 0.1982 rad/s against the observer's 0.1717 -- slightly worse -- and peaks at 62 % against 1.9 %. A quieter estimate that is ten milliseconds late is not a better estimate inside a loop: the window is transport delay, and ten milliseconds is about 115 degrees of phase at the loop's 32 Hz bandwidth. This is why it was measured before being wired into `Motor`. **The third leg is the closed form itself**, and it is now checked against the shipped estimator rather than asserted: on a synthetic stream with independent angle noise the output standard deviation is **0.57887 / 0.14297 rad/s against sqrt(2)*sigma/(N*dt) = 0.56569 / 0.14142** at N = 50 / 200 (ratios 1.023 / 1.011), halving the angle noise halves the output **exactly 2.0000x**, and a least-squares slope over the same window is **0.3249 / 0.1738** of the difference against **0.3363 / 0.1719** predicted. That last pair is the one with teeth: the measurement separates the correct closed form from the `sqrt(12/N)` = 0.4899 / 0.2449 this header claimed until F-42, by 46 % and 30 %. So the 6.5x gap on the rig above is not the formula being wrong -- it is the formula working, since it holds to 1-2 % exactly when the angle error is independent, and a 6.5x excess therefore says the rig's angle error is correlated. One prediction did not survive: a quantised angle gave 0.616x of the `q/sqrt(12)` form, because a deterministic staircase correlates the two ends of a window; what is asserted is the derived inequality (positive correlation can only make the difference quieter), and the factor is left unmeasured rather than fitted. Domain: the estimator given angle samples -- it says nothing about whether the angle is right | `test_sil_speed_from_angle.cpp`, `test_sil_speed_from_angle_closed_loop.cpp` |
| `foc/core/machine_map_text.hpp` | L3 | A machine document that survives a round trip bit for bit, an unknown key counted instead of rejected (a reader that refuses newer files stops the format growing), and a failed read that leaves the receiver untouched. It defines no vocabulary for where the numbers came from. **The precision is `FLT_DECIMAL_DIG`, and the alternative is measured failing rather than argued about**: on 4000 deterministic samples drawn from the map's own range, the writer's `%.9g` round-trips **4000/4000** bit-exactly while `%.8g` round-trips **3993/4000** -- one digit shorter loses the last bit on 7 of them. So 9 is not a round number chosen for looks: it is the shortest width that is *sufficient*, and 8 is provably not, which pins the width at the standard's answer with no room left for a taste argument. The comparison is on the **bits** and not on the decimal text, because "the same number written two ways" is not a model of equality that a float has. **Not** claimed: any stability of the format for a reader that refuses what it does not understand -- that is the ignored-key count's job and it is measured separately | `test_machine_map_text.cpp` |
| `foc/core/deadtime.hpp` | L3 | The axis projection: 4/3 on d, 2/√3 on q, 4/π averaged over a rotation. Using the d number on q costs 15.7 % on `Lq` | `test_deadtime.cpp` |
| `foc/core/feedforward.hpp` | L3 | The two classical PMSM voltage feedforwards, axis cross-coupling and back-EMF, added after the current PI so the PI keeps its own-axis job. **The sensitivity table is the mismatch law**, and it is a one-liner with a number attached: the feedforward removes `omega*(Ld*id + psi_f)`, so a configuration that is `(1+eps)` of the truth leaves `-eps` of the term on the PI -- measured at eps = +0.10 / -0.10 / +0.25 the residuals are **1.025 / 0.975 / 1.010 times** the prediction, and the ratio of the last two halves the mismatch to **2.4629 against 2.5**. **The second leg is the machine's own steady-state equation**, read through the PI rather than from the code: with no feedforward the q-axis voltage matches `Rs*iq + omega*(Ld*id + psi_f)` to **0.05 % and 0.25 %** at two operating points, and the pair `(Ld, psi_f)` is then recovered as **2.0930e-4 against 2.0e-4** and 0.004550 against 0.004562. **What is not separable is worth stating**: `Ld` and `psi_f` enter only as `Ld*id + psi_f`, so at `id = 0` a configuration with four times the true inductance leaves a residual **2e-6 different** -- identical -- and only a second point at `id = -1 A` splits them (the shift there matches the product to 1e-5 relative). **The d axis is not a usable identification channel and this is measured**: the command is built in the estimated frame and applied in the plant's true frame, so the d voltage is `Rs*id - omega*Lq*iq + vq*delta` with `delta` the estimator's angle error; solving the two axes gives `delta = -0.0136 rad (0.78 deg)`, and `vq*delta` is **55.5 % of the cross term it was supposed to measure**, while the same rotation adds only 0.052 % to the q axis. The budget is therefore explicit: identifying `Lq` from d needs the angle known to `omega*Lq*iq/vq = 0.0245 rad`, which is why commissioning identifies inductance on q | `test_sil_feedforward.cpp` |
| `foc/core/reference.hpp` | L3 | `d = 0` default plus `MtpaReference`: +29.2 % torque at `Lq/Ld = 2`, and exactly `i_d = 0` on a surface machine | `test_mtpa.cpp` |
| `foc/core/field_weakening.hpp` | L3 | Matches the analytic voltage/current envelope to four digits; the "is it worth it" rule is a gate, not a suggestion; the policy engages end to end through `Motor` (`i_d = -1.93 A` 2 ms in). **The gate is one comparison, and that is now stated and pinned**: the verdict steps exactly at `i_char = i_max` (bisected to 1e-5 on the function itself, `speed_gain` infinite on the accepted side), because below that line the flux cannot be nulled at all. The two thresholds the header used to advertise -- `ratio < 0.6` and `speed_gain >= 1.5` -- are both **implied** by it, so the expression they formed was unreachable-as-true and they were removed rather than left as decoration (F-107). **Run as a selection function over every machine this project has parameters for**, the answer is no to all four of them: the SIL machine at `lambda = Ld*i_max/psi_f = 0.252`, a published SPMSM-class drive at 0.115, a published IPM at 0.235 (after the Ld/Lq correction its own table needs) and the **2004 Prius traction motor at 0.253 -- a machine designed for exactly this, 3.3x short of the line** -- so field weakening is a niche feature for low-flux, high-inductance designs and claiming it generally would be claiming it for machines that are not here. **The ellipse sensitivity is measured and it is asymmetric**, which a single coefficient would have hidden: the `Rs`-free law `d i_d/d psi_f = -1/Ld` is an upper bound never reached (the point sits on the current circle at **every** speed in the weakened region, `norm(i) = i_max` to four decimals), and a `psi_f` a tenth high moves `i_d` by **-0.795 A at 2910 rad/s but -0.016 A at 3800** -- while a tenth low still moves it +0.49 A there. Asymmetry **1.31x at the base speed, 30.3x at the top**, so an over-read flux is the dangerous sign and it is dangerous exactly where the feature is working. A 20 % `Rs` error is at most **21.5 %** of the flux swing and falls to 3.6 %, which is what makes the policy's `Rs`-free ellipse defensible. **Not** claimed: that a machine just inside the line is a better product than one just outside -- the model's ceiling is a step there, and a step is where a closed form stops having an answer | `test_field_weakening.cpp`, `test_field_weakening_motor.cpp` |
| `foc/core/commissioning.hpp` | L3 | `Rs` +0.01 %, `psi_f` +0.51 %, `Ld` +0.14 %, `Lq` +0.23 % on the SIL plant, each with the condition it needs | `test_commissioning.cpp` |
| `foc/core/protection.hpp` | L3 | The grade is real and graded: 3.00 A at 45 C, **1.80 A at 80 C with output still allowed**, 0 A and no output at 105 C, and a build that trips at the derating start fails the ladder case while the curve case stays green. Two faults at once report the lower-ranked code with both bits set (0x006 for current+voltage, 0x018 for voltage+heat). I²t budget `i_cont²·tau`; a trip removes the *command* in one period and the current then decays with `L/R` (0.80 ms) -- it cannot be clamped in one period, and the block says so. Cross-check: stepping the recurrence and solving the ODE agree to **1e-4** -- 1.0555 s against 1.0556 s at 3.2 A, 0.6635 against 0.6634 at 3.5 A, 0.4881 against 0.4881 at 3.8 A -- and the measurement is by construction not confusable with the naive `budget/i²` (0.367 s), which it misses by more than 40 %. Sensitivity: a **10 % current-sense gain error is worth +79 % / -30 %** on the trip instant (1.1877 s / 0.4672 s against 0.6635 s), asymmetric because the budget goes with the square; a real loop period 10 % away from the one the code was told stretches the trip by exactly that factor (0.7298 / 0.5971 s); and a **10 K temperature-sensor offset is worth 20 % of the current limit** (0.6 A on a 3 A machine), which nothing calibrates unless the caller does. **The ranking now has the derivation that exists, and the place where it stops is measured rather than asserted**: four of the seven codes are *threshold* detectors and every one of them asserts after **one step** (measured at a 1 % overshoot), so the machine supplies no order among them at all, while the two *duration* codes can only assert later -- `I2t` at **13 269 steps = 0.663 s** and `Stall` at **10 000 steps = 0.500 s** against the same 20 kHz grid. The enum is a linear extension of exactly that one statement: all four thresholds before both durations. And the property a fault log actually relies on, path independence, is measured over every physically possible pair of threshold codes -- raising A then B reports the same code and the same mask as raising B then A, winning by rank and not by arrival (0x006, 0x00a, 0x012, 0x014, 0x018: both bits always visible, the lower-ranked code always reported). **Not** claimed: any severity order *among* the four threshold codes, or between `I2t` and `Stall`. The first has no evidence available -- the detectors are simultaneous -- and the second compares a physics-driven integral against a configured window, which carries no severity information. Those two choices are what the enum *is*; they cannot be renumbered because the values are mirrored in `foc_api.h`, and the row says so instead of inventing a derivation | `test_protection.cpp` |
| `foc/core/trace.hpp` | L2 | One 36-byte sample per period, agreeing with `telemetry()`/`state()`/`status()` bit for bit in the same period; the null channel adds no storage and no branch (host, best of three: 158.0 ns/period off vs 161.9 on, and the +3.9 ns is at the resolution of the measurement); a sample survives a `%.9g` text round trip bit for bit; the same record crosses the C ABI (layout asserted at compile time, not commented) and is readable from Python. **L2 is the terminal level for this row**: a recorder has no error source to be sensitive to -- it stores what it is given -- so the leg L3 asks for cannot exist for it, and what can go wrong instead (losing a sample, the wrap, the null channel) is measured above. See O-21 | `test_trace.cpp`, `c_api_validation.c`, `test/python/test_python_bindings.py` |
| `foc/core/sinusoid_fit.hpp` | L3 | The moving part of a frequency sweep, and the criterion that carries the bandwidth claim: the loop's -3 dB point measures **234.91 rad/s against 250 requested (-6.04 %)** on the C++ bench and **236.18 rad/s (-5.53 %)** through the Python plant, two independent machines agreeing to 0.5 %. It is sharp where the step-response band is blunt: a 20 % error in the speed loop's gain moves this number 15.68 % (outside its 10 % band) against 9.9 % for the step fit (inside its own). **L3 sensitivity, on the ways this function is actually wrong**: window length, frequency mismatch and DC offset, each compared against an estimator that should differ. Window: the fit reads **0.0000 %** error at 2.00/2.50/3.00/3.20/3.40/3.50 cycles while a plain rectangular-window DFT bin -- exact on whole **and half** cycles -- reads **+4.09 % and +2.01 %** at 3.20 and 3.40. Frequency: the amplitude error is **not** monotone in the offset (+0.5 % gives 0.095 %, -0.5 % gives 0.218 %), so the criterion is the analytic envelope `abs(sinc(dw*T/2))`, which predicts all eight points to within **1 percentage point** (5 % offset: 5.159 % measured against 4.687 % predicted). DC: a +5 V offset on a 3 V tone moves the amplitude **0.0000 %** with the constant column and **7.6 %** without it, which is measured rather than asserted -- the falsifier is the same projection with that column removed. Stated limit: the envelope is a bound and not an identity, and the residual one-sidedness at negative offsets is reported rather than fitted away | `test_bode_sweep.cpp`, `test/python/test_python_bindings.py` |
| `foc/motor/torque_map.hpp` | L3 | Newton-metres in, amps out, with the limits an application needs. The estimate agrees with the mechanics it does not share a line with -- `J*dw/dt + B*w` from the recorded speed -- to **+0.006 % at 2 A and +0.007 % at 20 A** (which saturates the loop); scaling the map's gain by 0.9 moves the same measurement to **-9.994 %**, so the 5 % band is a real gate. One-way limits are reported through `limited` rather than obeyed silently, and a nameplate with `kt = 0` is refused instead of becoming a map of zeros. **L3 sensitivity on measured bars, not invented ones**: the externally measured `psi_f` deviation of **+8.93 %** moves the estimate **+9.749 %** (amplified, because the reluctance term subtracts at that point), and a 50-100 A nameplate used at 200 A moves it **+10.006 %** together with the **13.4 %/8.1 %** `Ld`/`Lq` saturation. **A phase-variable witness** that never forms a d-q quantity and never uses `kt` agrees to **0.001 %** at six `id = 0` points, and a falsifier reads **0.900000** for a 10 % low gain, so the witness is proved able to reject an error of the size it claims to detect. Two limits are stated rather than glossed: the 5 % band is a statement about the **code path**, not about a physical motor, and the witness **cannot see the saliency term at all** -- its share is measured at **8.40 %** of the torque at `id = 1 A`, so that half rests on the sensitivity table alone. Tree under this row: 599 of 599 cases, 94.22 % overall. See foundation-log F-94, A-95, F-99 | `test_torque_map.cpp`, `test/python/test_python_bindings.py` |
| `foc/core/soft_limit.hpp` | L3 | Refuse the targets outside a window and slow the axis down near the walls, with the invariant `v^2 <= 2*a*(d - v*dt)` as the claim. The `v*dt` term is the difference between working and not: at 20 rad/s toward a wall the axis crosses by **19 mm** without it and stops **1.04 mm short** with it, and 19 mm is exactly the 1 ms this drive takes to deliver a braking current. Unshaped, it goes through the wall and keeps going. **The third leg is the invariant solved rather than only checked**: the largest speed the cap passes at distance `d` is `sqrt(a^2*tau^2 + 2*a*d) - a*tau` -- measured through the cap's own seam, which is exact because below it the cap returns the caller's float unchanged and above it returns something smaller, so the observable needs no tolerance: **1.000000 on five `(d,a,tau)` pairs**. That seam is homogeneous of degree one in `(d,a)` jointly (k = 0.25/0.5/2/4 recovered exactly), so a machine twice as strong over twice the distance is twice as fast and not `sqrt(2)` times, and each error source has a coefficient measured against the seam itself: **a/(v+a*tau) = 69.7744 vs 69.7750** per radian of wall error, **v^2/(2a(v+a*tau)) = 6.2008e-3 vs 6.2008e-3** per rad/s^2 of braking capability, **-a*v/(v+a*tau) = -1395.23 vs -1395.34** per second of delay. The plant settles the second leg in the form a user needs, which one speed cannot: **the delay term shifts the closest approach by exactly one delay travel**, ratio **1.0000 at 20, 10 and 5 rad/s**, and the cap begins to bite at `v^2/(2a) + v*tau` (0.15321 vs 0.15333, 0.04279 vs 0.04333, 0.01310 vs 0.01333, all within 1.7 %). The delay-free reading of the same invariant is only 7.2 % away at the millisecond scale -- which is why it survived -- and 58.1 % away at ten milliseconds, so the seam test can tell them apart. **Not** claimed: any protection when the axis is already past a wall, or when the window moves inward around an axis that is already inside the new margin (both need a homing switch, and both are the caller's precondition that the command is one the axis is tracking) | `test_soft_limit.cpp` |
| `foc/core/gear_ratio.hpp` | L3 | Electronic gearing whose whole difficulty is that an encoder reads `[0, 2pi)`: three wrapped revolutions become 18.85 rad of continuous reference instead of two 6.28 rad jumps, and the synchronisation error is `v*kd/kp` -- **25.0 predicted, 25.6 mrad measured** on an ideal ramp, linear in the rate (0.51x at half) and inverse in the gain (10x at a tenth), 29.7 through a real master axis. The velocity output removes it (**1.3 mrad** ideal, 8.0 through a measured master), because a gear that emits only a position is a lag generator with a ratio. **The third leg is four closed forms, one per master error source**, each derived before it was measured: a quantised master costs at most **`abs(ratio)*q`** and does *not* grow with time because consecutive deltas telescope -- measured **1.366 mrad** against a 2.500 mrad bound, 1.316 of it in the second half of a four-second run, so the bound is exercised and the structure holds; a sample interval off by fraction `j` moves the velocity reference by `ratio*omega*j/(1+j)` -- measured **0.95240 of nominal against 0.95238**; a count the encoder never counted costs exactly one count, **2.49958 mrad against 2.50000**; and a *sample* that never arrives costs **nothing at all (0.00e+00)**, because the next sample carries the full accumulated angle. The last two are the pair that matters: a dropped sample is recovered and a lost count is not, and the first version of this case asserted the opposite. The header's stated assumption is now checked as well -- `kMaxStepRad` had a comment promising a check and no consumer until F-43, so a master step at or above `0.9*kMaxStepRad` is reported rather than decoded, with the honest limit written down: a lost *whole revolution* decodes to a zero step and cannot be detected from a wrapped angle | `test_gear_ratio.cpp` |
| `foc/core/friction_comp.hpp` | L3 | Turn a standing position error into two arithmetic numbers. A proportional loop stops where its torque falls under the static friction, so the residual band is `static/kp` -- **4.003 mrad measured against 4.0 predicted** -- and a Coulomb term latched onto the direction of travel moves that wall by its own size: **2.021 mrad**, and 1.007 with a 50 % over-estimate. The latch is what keeps it from being a square wave: at rest with +-0.01 rad/s of encoder noise the naive `sign(measured)` law ripples **20.03 mN*m** peak to peak and the latched one **0.036**, 550x quieter. An estimate above the breakaway is a motor: with the reference held at zero it walks the axis **5.625 mrad** with no command at all. **The third leg is a published friction model that the compensator does not implement**, and it is what makes this row L3 rather than a set of measurements of its own assumption: until now the plant's friction was Coulomb-plus-viscous, i.e. exactly the law under test, so the check was the model checking itself. With a Stribeck dip of width `vs` in the plant, cruising at `v` costs an extra following error of **`(Fs-Fc)*exp(-(v/vs)^2)/kKp`** -- measured **1.8789 / 0.7356 / 0.0364 mrad** at 0.25 / 1 / 2 rad/s against **1.8790 / 0.7358 / 0.0367** predicted, ratios 1.00 / 1.00 / 0.99 -- and the compensation saves `(Fc + b*v)/kKp`, measured **2.0500 / 2.2000 / 2.3998** against **2.0500 / 2.2000 / 2.4000**. The decisive number is the third row of the table the test prints: the saving is **identical with and without the dip**, to 1e-4 mrad, because a velocity-sign feedforward has no state and no velocity dependence beyond the viscous term. So the domain is stated with a formula rather than a caveat: **this compensator removes Coulomb and viscous friction and cannot remove anything inside a Stribeck dip**, and the residual there is the expression above, not a tuned error margin | `test_friction_comp.cpp` |
| `foc/core/hall.hpp` | L3 | Three Hall sensors decoded without a live angle. The six-step table is a rotation at `90 + k*60` degrees, and held at one state the torque is `cos(position in sector)` -- **0.866 at both edges, never negative**, the 14.0 % ripple six-step cannot avoid (measured against the plant, and the same number falls out of the integral of `cos`). Interpolating between edges beats the sector centre by **52x at 120 rad/s electrical (0.338 deg RMS against 17.52)** and 7.5x at 900 (2.30 against 17.32), where the residual is one sampling period of rotation and not the 60-degree sector. **The tracker's own input error is a table, not a hope**: a rigid mounting offset of all three sensors displaces the worst-case angle error **one-for-one** (maxima minus base at 2/4/8 deg are 1.988/3.976/8.002 deg, i.e. **0.994/0.994/1.000**), while **one** misplaced sensor carries about three quarters of it in RMS (0.679/0.752/0.751) and gives a **worse worst case -- 5.105 against 3.199 deg at 2 deg, 1.9-2.6x the offset** -- so a bracket is specified by the single-sensor case, which a rigid-only table understates by more than 2x. A non-finite timestamp is refused without being adopted (two trackers, one fed a NaN every 37th tick, stay bit-identical), and the clamp is symmetric: the lower boundary, which only a reversal visits, holds. Boundaries stated rather than glossed: a dead sensor is declared for **two thirds of a revolution** (`Fault` 15.5 %, `State` 53.9 %) and the remaining 8.4 % reads `Ok` while wrong by up to 59 degrees -- asserted, because that number is the boundary; and `State` is a state rather than a fault precisely because a deceleration that changes the speed by a third inside one sector is refused too | `test_hall_decoder.cpp`, `test_hall_physics.cpp` |
| `foc/core/stall_start.hpp` | L3 | Deliberate breakaway with a clean handover: the ramp releases the axis the moment it moves and reports the *handover* torque -- 0.0300 N*m at 60.1 ms for a 0.030 N*m breakaway at 0.5 N*m/s -- instead of a wound-up integrator. Against a real stuck plant (stiction 0.020, load 0.010) it releases at **0.0307 N*m after 61.3 ms**, and a limit under the breakaway point is refused rather than ramped into a wall. **The handover torque is a measurement and not a setting**, and the way that is proved is by separating the two numbers it could have come from: a **50 % error in the recipe's breakaway moves it by 0.0000 N*m** (recipes at 0.015/0.030/0.045 on a 0.030 N*m machine all release at 0.0307 N*m, 61.3 ms), while the same sweep on a **heavier plant** moves it by 10 mN*m (0.0405 N*m at 81.0 ms) -- a configuration-driven value could not pass the second sweep. **The ramp rate sets when, not how much**: 0.25/0.50/1.00 N*m/s release at 121.3/61.3/31.3 ms (the time halves to within 2 %) while the torque moves **1 mN*m over the 4x change**, which is observation lag rather than physics. Both instants are closed-form and asserted against arithmetic: a jam reports `Failed` at **exactly `max/rate + give_up`** -- 6000 ticks for a 0.2 s grace and 8001 for 0.4 s, the extra tick being the float32 accumulator in `hold_s_`, recorded rather than tuned away. A step before `begin()` answers `State` and commands nothing, so an un-started sequence cannot push a torque into the axis | `test_stall_start.cpp` |
| `foc/core/step_metrics.hpp` | L3 | The loop measured against its own gain law: a 10 rad/s step on the SIL bench fits to **243.57 rad/s against the 250 rad/s that was asked for (-2.57 %)** through the Python plant and +9.15 % through the C++ bench, whose inner current loop is the difference, with the ratio of a 125 and a 250 rad/s request measured at 2.026 against an ideal 2; a synthetic decay with a known time constant is recovered to 1 %, a known 25.7 % overshoot is measured to 1 % *and* kept out of the fit (bit-equal to the truncated window), and "never settled" is -1 rather than 0. **The sensitivity table is mostly zeros, and those are the interesting rows** because each says what kind of estimator this is: sample rate 0.2->2 ms (10x) gives a tau ratio **1.000000 -> 0.9999995**, a 10->40 ms window gives **1.000000 -> 0.999999** (every sample is on the model's line, so more samples add information and not bias), and a **DC offset of +-1 and +5 on a 10 A step gives 1.000000** (the offset sits in `y[0]` and `y[i]` and cancels in `progress` -- which is why a biased current sensor does not corrupt a bandwidth measurement). **The error that really costs is a target the loop does not reach**: a final value 5 % below the passed target (a P-only loop's droop) inflates tau to **1.438404x**, 5 % above shrinks it to **0.735124x**, and the map is asymmetric by a measured **1.65x**; noise is an order of magnitude cheaper at **0.987336** for sigma = 1 % of the step (200 seeds) and 0.982475 at 2 %. Settling is pinned against arithmetic rather than memory: **18.5000 ms measured against `tau*ln(step/band)` = 18.4207 ms**, with `0` for a response that never leaves the band and `-1` for one that never reaches it. Stated boundary: the fit's `!(slope < 0)` guard is **unreachable** (every usable sample has `0 < w < 1`, so the through-origin slope is a sum of negative terms), and the long-window limit is the **float32 sample format**, not the algebra -- 20 time constants read 1.002148 | `test_step_metrics.cpp`, `test/python/test_python_bindings.py` |
| `foc/core/board_check.hpp` | L3 | Names the correct cause on three seeded faults; measures the ADC trigger position when the bench can move it, and reports the optimum rather than assuming 0.5 | `test_board_check.cpp` |
| `foc/core/alignment.hpp` | L3 | The align/verify sequence, in the currency a commissioning engineer spends: **what passing the tolerance costs**. The capture is a circular mean over the hold window, so a rotor still creeping at `ω` is captured at the **middle of the arc** -- the offset is wrong by `ω*T/2`, and the readings spread over that arc as `ω*T/sqrt(12)`; both are the same sweep seen twice, so their ratio is `sqrt(3) = 1.7321` **at every rate**. Measured at `hold = 50 ms` over four creep rates (0.5/1/2/4 rad/s): offsets **0.012475 / 0.024950 / 0.049900 / 0.099800**, spreads **0.007217 / 0.014434 / 0.028869 / 0.057745** against `ω*T/sqrt(12)` = 0.007217 / 0.014434 / 0.028868 / 0.057735 (four digits), and ratios **1.7286 / 1.7286 / 1.7285 / 1.7283** -- constant to four digits across an eightfold range of creep, and 0.2 % below `sqrt(3)` because the *circular* mean shrinks second-order against the arc's midpoint (that is why the band is 0.3 % and not an equality). **The consequence is the number to commission against**: a capture accepted at exactly `settle_tolerance_rad` has an offset error of **`sqrt(3)*tol`, 1.73 times the number that was relaxed** -- so the tolerance is a budget for how wrong the offset may be, not a knob for making a sticky machine pass. A still rotor is captured **exactly** (0.000000 error, 0.000000 spread), which is the anchor the coefficients are measured against. Two slot contracts are pinned rather than assumed: a rejected alignment **keeps reporting `failed` on every following tick** until `reset()` (a poller must not see it once and then silence), and a plain alignment hands over **at rest** -- `handover_speed_e_rad_s` is 0 on every tick, which `Motor` reads as "this hand-over cannot be judged as caught". Stated boundary: `capture_mean()`'s zero-sample branch is **unreachable** (both phases accumulate before they capture), so two lines of it are defensive rather than covered | `test_alignment.cpp` |
| `foc/core/polarity.hpp` | L3 | Polarity detection and its refusal when the answer is not decisive. **The sensitivity table is the angle law**: both pulses are applied along the *estimate*, so the d-axis volt-seconds are cut by `cos(delta)` and the measured projection cuts them again -- two factors, hence `cos^2` -- while the saliency term is common to both pulses and cancels in the difference, so saliency attenuates the answer without contaminating it. Measured ratios against `cos^2`: **0.987 at 15 deg, 0.943 at 30, 0.850 at 45, 0.658 at 60**, the last one showing where the small-signal law stops holding (a smaller pulse engages less of the inductance collapse, so the real attenuation is faster). **The window is closed by something the derivation did not predict**: at 60 deg the asymmetry is 0.093 A, still **4.6x the 0.02 A threshold**, and the answer is refused because the pulse now has a large q component -- the rotor moves 14 mrad electrical and the two pulses no longer start from one operating point; a 20 ms settle brings the answer back, which is what identifies the binding limit. Half a turn out gives the same magnitude with the opposite sign to 0.04 %. **Found and fixed (F-47)**: the method's premise is "the same volt-second pulse in two directions", and nothing checked that both pulses started from the same current, so a settle shorter than the winding's `L/R` produced a confident **reversed** verdict for an `Aligned` magnet (asymmetry -0.5404 A against the honest +0.5651 A -- same magnitude, opposite sign, invisible to any magnitude check). `decide()` now refuses when the two start points differ by more than `min_asymmetry_a`, and the crossing is the closed form: the residual decays as `exp(-t/tau)` with `tau = Ld/Rs = 571 us`, so the guard releases at `tau*ln(1.791/0.020) = 2.57 ms` -- measured refused at 2 ms, answered at 3 ms | `test_polarity.cpp`, `test_sil_polarity.cpp` |
| `foc/core/position.hpp` | L3 | The position loop with velocity and acceleration limits, and **a measured error budget** rather than one more pass/fail number: at 20 rad/s loop bandwidth a 1.0 rad step settles with a **1.08 mrad** residual of its own at a 1.2 s window, an encoder of **2 mrad per count** adds **0.913 mrad**, a **0.2 mN*m** breakaway adds **0.213 mrad**, and **0.48 V** of dead time adds **0.027 mrad** because a constant voltage offset is integrated away at steady state -- a prediction the table checks instead of assuming. All three together measure **2.01 mrad** against a budget of **2.23 mrad** (ratio 1.11, inside the 1.5x the plan set), and dropping the dominant term breaks the budget, so the table cannot pass while missing a contributor. **The rows are converged now, and getting there changed what they mean.** They used to move by 1.3-3.8x with the simulator's integration step, because the plant integrated its mechanical angle in float32 and rounded away any step smaller than half an ulp -- the axis froze while its speed still read non-zero (F-40). With the angle in double every row is stable to better than 1 % between 4 and 16 substeps. Two consequences, and the first one **reverses what this row claimed before**: with an ideal sensor and no friction the residual decays to **zero** (8.8 urad at 6 s, 0.000 at 12 s, time constant about 1 s), so "the loop is the accuracy limit" was a statement about a window rather than a floor -- the loop's share is a **settling tail** that keeps shrinking, while the encoder's **half-count floor is permanent** (991 / 995 / 999 urad at 1.2 / 6 / 24 s: it does not decay at all). Which term dominates therefore depends on how long you wait, and at this table's 1.2 s window it is still the tail. The second consequence is unchanged: the target is swept across one count before the worst case is taken, because a quantisation error is signed and one lucky target measured the effect as *negative*. **The third leg -- an independent model -- is now in place, and it is the one that can actually falsify.** ISO 230-2's reversal value, measured the way the standard says (each target approached from both sides), comes out at **exactly twice** the one-sided stiction displacement -- 1.00 at three targets, so a model that says 2 rather than "about 2"; the same measurement at zero breakaway torque collapses to **0.42 urad**, 0.11 % of B, which is the mechanism's own control; repeated identical moves differ by **exactly zero**; and bidirectional accuracy is **0.386 mrad** against the 2.45 mrad budget. Separately, a 2 mrad sensor's residual over a sweep that spans one count has a standard deviation of **558 urad against Bennett's q/sqrt(12) = 577 urad (0.966x)** -- a question the budget never asked and could not: a worst-case half-count bound cannot tell a uniform quantiser from a biased one. Both are measured by a command-line harness rather than by the suite (`project/gates.py plantref`), so the evidence shares no language, no arithmetic and no solver with the code it checks, and its own self-test perturbs a good trajectory three ways to prove it can go red. Domain, stated rather than implied: **sensored, quasi-static, on the SIL plant** -- not L4. **Not** in the budget, said rather than omitted: torsional elasticity (needs a two-mass plant; this one is single-inertia) and gear-ratio error (a frame transformation the rig does not model) | `test_position_p.cpp`, `test_sil_position_budget.cpp`, `plant_reference.py` |
| `foc/core/motion_profile.hpp` | L3 | Trapezoidal profile timing and its endpoints, and **what the profile's own structure bounds**: it shapes a *velocity* command from the position error, so there is no position reference and therefore no ramp-following error -- the textbook `e = v/kp` does **not** apply here, which was measured rather than argued (cruise gap 0.496 rad against 0.100 predicted, and doubling the shaping gain left it at 0.496: a number that does not move when its supposed parameter doubles is not that parameter's). What is bounded is where the cruise ends: the profile decelerates at `v^2 / (2a)`, and the smallest cruise gap measures **0.173 rad against the 0.200 rad threshold** (-13.7 %, asserted), with the cruise window non-empty. **Not** claimed: any following-error law against a reference trajectory, because this module does not generate one. **The third leg is the two switch points in closed form, plus the move's own kinematics.** The three ceilings cross where the algebra says: the proportional term meets the braking curve at `2a/g^2` (measured exactly at 0.0500 / 0.0125 / 0.1000 for three `(g,a)` pairs) and the velocity limit takes over from both at `v^2/(2a)` (found by bisection, ratios 1.0000, exponents `a^-1` = 0.5000 and `v^2` = 0.2500 recovered rather than restated). Two consequences are the real content. First, **the shaping gain is absent from both landmarks** -- it reaches the structure only through the inequality `g >= 2a/v`, and at `g = 20, a = 10, v = 2` the boundary is exactly 10, so *no* change to the gain can move the cruise exit. That is the reason behind A-40's null result, which had been recorded as a refutation without one. Second, the header's "not time-optimal" caveat is narrower than it reads: for a rest-to-rest move, ramp / cruise / constant-deceleration makes the total `D/v + v/a`, the time-optimal trapezoid's own number -- predicted 0.6553 s against 0.6480 measured, and moving the target 3 rad further costs **1.5000 s against 3/v = 1.5000**. What the profile does **not** bound is the axis: the deficit has a kinematic part plus a lag that scales with `v` alone, `v*(0.05*v/a + tau)` with `tau = 3.692 ms` recovered on one configuration and predicting two held-out ones to 2.6 %, so the axis passes the deceleration threshold at **86.3 % of it** while the command cannot overshoot at all | `test_motion_profile.cpp`, `test_motion_profile_landmarks.cpp`, `test_sil_motion_profile.cpp` |
| `foc/core/observer/observer_policy.hpp` | L1 | The observer contract (`theta_e`, `omega_e`, one `update`) | `test_traits_contract.cpp` |
| `foc/core/observer/ideal_observer.hpp` | L2 | The reference every other observer is compared against, and the three properties that let it be one, each asserted against an input chosen to break it: **zero lag** (the value set this period is the value reported this period), **total independence from the electrical inputs** (NaN, infinities, a 1e9 V command, a zero period and a negative period all leave the reported truth bit-identical), and **reset to zeros**. The falsifier is a real observer fed the same two opposite voltages, which answers differently -- so "the output does not move with the inputs" is a property of this class and not of the interface. **L2 on purpose, and it is the terminal level**: L3 asks for a sensitivity to an error source, and this class has no model to be wrong -- it reports the plant's truth by construction, which is precisely what makes it the reference. Its only failure mode is ceasing to be a cheat channel: a filter, a one-period delay or a plausible-looking "improvement" that reads the electrical inputs would turn every comparison that uses it into a comparison between two estimators, and that is what the case pins | `test_ideal_observer.cpp` |
| `foc/core/observer/back_emf_observer.hpp` | L3 | Angle and speed from pure integration, and the honest question for a loop with **no correction term** is which input errors accumulate. Measured on the exact steady state of a PMSM (`Rs = 0.5`, `L = 8e-4`, `psi_f = 4.7619e-3`, `dt = 50 us`): an **exact model leaves exactly one step of rotation** -- 5.2e-4 rad at 20 rad/s, 5.3e-3 at 100, 1.06e-2 at 200, i.e. `omega*dt`. A **resistance error is a bounded bias, not a runaway**: 10 % wrong `Rs` costs **0.191 rad at 200 rad/s** and 0.091 at 400, against the small-error law `eps*Rs*iq/(omega*psi_f)` = 0.1575 / 0.0788 (ratios **1.21 / 1.16**), and the case records where that law dies -- at 50 rad/s the estimate is **3.14 rad** out, a full `pi`, because the perturbation is no longer small. An **inductance error is bounded too** (0.191 / 0.212 rad of excursion at +-20 %). **The one error that runs away is a DC offset on the measured current**: it enters as a constant voltage error every step and never reverses -- 50 mA puts the estimate **0.711 rad out in 100 ms** and 1.014 in 200, against `Rs*off/psi_f = 5.25 rad/s` as the right order. So a current-sense offset has to be calibrated out before this observer is trusted, while a wrong `Rs` merely biases it. `apply_params()` had never been called by anything and is now pinned behaviourally: it takes the **q-axis** inductance on this single-L model (an observer re-configured with `ld = Lq/4` answers bit-identically to one built with `lq`, and 0.68 rad away from one built with `ld`), which is why `i_d != 0` is handled by a ratio budget in `combinations.json` rather than by the model; an impossible parameter set puts the observer in `Param` and it then holds | `test_back_emf_observer.cpp` |
| `foc/core/observer/nonlinear_flux_observer.hpp` | L3 | 0.1044 rad at 200 rad/s with `i_d = 0`; 2.03× worse with a 10 % current-gain error; **floor near 50 rad/s**; and **it requires `i_d = 0`** -- see the row below | `test_nonlinear_flux_observer.cpp`, `test_sil_sensitivity.cpp`, `test_sil_admission.cpp`, `test_salient_observer.cpp` |
| `foc/core/observer/hfi_observer.hpp` | L3 | Saliency tracking on a high-frequency carrier, with the polarity the carrier cannot supply left to `MagnetPolarityTest`. **The sensitivity table is three closed forms, each checked against arithmetic rather than against a previous run.** Injection amplitude: the demodulated amplitude is linear in `V_h` -- **1.0000 / 2.0000 / 4.0000** for 6 / 12 / 24 V -- so a smaller amplitude is only a signal-to-noise decision and never a recalibration. Saliency: the signal is the **admittance difference** `(Ld-Lq)/(Ld*Lq)`, so the same ratio with a scaled machine halves for each doubling -- measured **0.5168 and 0.2618 against 0.5000 and 0.2500**, and the machines are scaled *up* on purpose because the formula ignores `Rs`: at `Ld = 1e-4` against a 0.35 ohm stator the same law is already 9 % out, which is the degradation the neighbouring case documents. Sample timing: `max_inject_hz = control_hz / kMinSamplesPerCarrier`, exactly **8000.0 Hz at 20 kHz**; that carrier is accepted and 5 % past it is refused with `Param` once the first period supplies the rate, rather than being left to collapse. **What the demodulator contributes is a scale and is stated as one**: `hf_current_a / predicted_gain_a` is **3.601630 at every amplitude**, to 1e-6, so the formula is verified in shape and the constant belongs to the multiplier and the filter -- claiming it should be 1 would have been a fitted claim, not a measured one. The `sin(2*dtheta)` error law, the half-turn window and the no-carrier hold each already had a case | `test_sil_hfi.cpp` |
| `foc/core/observer/blended_observer.hpp` | L3 | Two observers crossfaded across a speed band, with the weight taken from the high-speed source so a hung handover cannot hold itself. **The sensitivity table is the convex combination, measured with stub observers whose answers are set by hand** -- so nothing in it depends on a plant: an error in the low source reaches the output at `1-w`, one in the high source at `w` (both asserted to **1e-4**), and a *miscalibrated decision speed* moves the angle by `disagreement/span` per rad/s, which comes out at **0.006000 rad predicted and 0.006000 measured**. The weight itself is exact: **0.25000 / 0.50000 / 0.75000** for 35 / 60 / 85 rad/s against a 10..110 band, which is `(speed-start)/span` to the last digit a float carries. **The second leg is that the interpolation is on the circle**: at 3.0 and 3.2832 rad the output is the short-arc midpoint **3.14159**, where a plain subtraction gives 0.0 -- a whole turn away, and exactly at the boundary the two sources disagree about. **One property stated rather than hidden**: a handover sweeps the published angle by the observers' disagreement while neither of them moves (**0.60000 rad swept for a 0.60000 rad disagreement**), and the published *speed* shows none of it, because it is the same convex combination -- so an angle and a speed reconstructed by integrating it disagree during a handover by exactly that amount. The angle is the better estimate and the sweep is how a handover works | `test_blended_observer.cpp`, `test_sil_observer_blend.cpp` |
| `foc/core/observer/dual_inductance_observer.hpp` | L3 | The same integrator as the back-EMF observer with the rotor angle recovered from the **saliency** instead of from a single `L`: `psi = psi_f*e^(j*theta) + L0*i + L1*conj(i)*e^(j*2*theta)` is a quadratic in `x = e^(j*theta)`, so the answer is closed form -- no iteration, no tuning, no allocation. **The algebra is exact**: seeded with the flux a salient machine has at a known angle and handed a voltage that leaves that flux alone, it returns the angle to **4.768e-07 rad** over 48 (angle, `i_d`, `i_q`) combinations, and on `Ld == Lq` it reduces to the shipped observer's `arg(psi - L*i)` **bit for bit** (gap 0.000e+00 over 36 combinations) -- which is the cross-check of the new algebra on a machine where nothing is ambiguous. **The bias it removes is `atan((ld - lq)*i_d/psi_f) and it is speed-independent**: measured on this repository's plant (speed-locked, exact steady-state d-q voltage, `Lq/Ld = 3`, `psi_f = 4.76 mWb`, 100 rad/s, `i_q = 0.5 A`), the single-`L` observer reads 0.0273 / 0.0433 / 0.0604 / 0.0773 / 0.0939 rad at `i_d` = -0.5 / -1 / -1.5 / -2 / -2.5 A against the law's 0.0210 / 0.0420 / 0.0629 / 0.0838 / 0.1046, so the law holds to within **30 %** and the residual does not respond to current. **The dual-inductance answer is flat**: 0.0145 to 0.0157 rad over the same 5x current range (spread 1.08), and **6.5x better** at 2.5 A -- a bias that grows with `i_d` is exactly what `MtpaReference`, which is *defined* by its `i_d`, cannot tolerate, and this is the first observer in the library that does not have one. **Measured floor, stated rather than hidden**: a backward-Euler flux integrator lags by about `w_e*dt/2`, which is 0.0025 rad at 100 rad/s and 0.02 at 800, so the table is taken at 100 rad/s and the 800 rad/s numbers are recorded as not usable -- at that speed the harness is part of the answer. **Not** claimed: anything about the switching inverter (a few tenths of a volt of dead time is already 0.1 rad of flux angle here, which is why the switching rig reads both observers at 0.13 to 0.15 rad and buries the difference), and no integrated nameplate-error leg -- that one is measured **statically** instead, where there is no floor, and it refuted the law I derived first: an `Lq` error moves the **angle** by `dLq*i_q/psi_f` (2.871e-3 against 3.150e-3, ratio 0.91) while an `Ld` error moves the **radius** by `dLd*i_d/psi_f` (3.888e-3 against 4.200e-3, 0.93) and the angle by only 7.2e-5, 58x less than the same argument predicted. So the two inductances are separable in two different observables, and that is what `radius_error()` is for: it is where an `Ld` error shows at all. **`FluxSource::{Integrator, Corrected}` is the class's one `method=`**, and the two are measured to fail differently: with the machine's own `Rs` the corrected source is **7.71x better** (0.002580 against 0.019895 rad over 2 s at 200 rad/s, from the numerical drift of a 40 000-step integration) and with `Rs` 20 % high it is **18.4x worse** (3.080 against 0.167 rad), because the projection locks onto a self-consistent but wrong state -- so the integrator stays the default and the switch lets a caller pick which failure they can afford. That signal also makes a wrong seed **reportable** rather than silent -- consistent 0.0, a seed half a turn out 9.0e-02, a seed built from another current 3.4e-02, and a consistent state stays at 0.0 over 2000 steps -- with the measured limit stated: a seed a tenth of a turn out reads 5.8e-03 and **passes** while its angle is 0.28 rad wrong, so it is a gross-fault detector and not an accuracy guarantee | `test_dual_inductance_observer.cpp` |
| `foc/core/observer/switchable_observer.hpp` | L1 | One observer slot holding the back-EMF, non-linear-flux, injection and blended observers, forwarding to the selected one: its output is reproduced **bit-identically**, a `seed_flux` reaches every observer that takes one, `injection_ab()` is the **selected** observer's carrier (zero for the two that do not inject) so no idle carrier phase advances, and a switch neither resets nor advances anything idle, `hand_over_to()` is the switching call that carries the outgoing estimate into the incoming one (measured: 0.0044 rad against 0.2768 rad for an unseeded `set_kind()`) -- so a hand-over must seed the incoming observer explicitly rather than assume it was tracking **Fifth member since 0.1.11**: `DualInductanceObserver` is in the slot and selectable through C (`FOC_OBSERVER_DUAL_INDUCTANCE`). It is the only member whose state is the **stator** flux, so the one-argument hand-over was not enough: seeding `psi_f*e^(j*theta)` leaves the current's own flux out of the integration constant and a pure integrator never forgets -- the A/B measures **0.005330 rad after 50 steps** at 0.1 A, permanently, while `hand_over_to(kind, i_ab)` seeds that member with the flux it holds (to 1e-3 rad at every current, against a 0.02 rad offset at 2.5 A if the current is left out). | `test_switchable_observer.cpp` |
| `foc/hal/clock.hpp`, `foc/hal/current_sense.hpp`, `foc/hal/dc_link.hpp`, `foc/hal/position.hpp`, `foc/hal/pwm.hpp` | L1 | Abstract interfaces only: no behaviour to claim, and the concepts reject an incomplete implementation | `test_traits_contract.cpp` |
| `foc/driver/stub_hal.hpp`, `foc/driver/stub_pwm.hpp`, `foc/driver/stub_dc_link.hpp`, `foc/driver/sim_position.hpp`, `foc/driver/scripted_current.hpp` | L1 | Host-side stand-ins for the real drivers; they prove the *interfaces* are usable, nothing about the hardware | `test_driver_stubs.cpp` |
| `foc/driver/callback_hal.hpp` | L1 | The same control loop with its measurements coming from a host: 200 duty writes and 200 current reads over 200 ticks, so the loop goes through the caller once per control step rather than through the stub. It says nothing about any particular host's accuracy | `test_host_hal.cpp` |
| `foc/motor/motor.hpp` | L3 | Policy composition, mode transitions, and the reference generator actually reaching the current loop. A position command is admitted when an **absolute angle** exists, which a sensor provides or a turn counter provides once its index origin is fixed -- previously the check was `has_sensor`, which was a consequence of `angle_mech` being a constant zero without one rather than a rule about what position control needs | `test_motor.cpp`, `test_field_weakening_motor.cpp`, `test_position_index.cpp` |
| `foc/motor/foc_loop.hpp` | L3 | Clarke + Park and inverse Park + SVPWM as the current loop's two ends, and the decimated outer loop that runs through them. **The period is a lower bound, not a period**: the loop runs on whole control ticks, so the actual period is `dt*ceil(T/dt)` -- asking for 1.2 ticks gets 2, measured from the trace channel as the spacing between current-reference updates (1.0, 1.2, 1.5, 2.0, 5 and 10 ticks request 1, **2**, **2**, 2, 5 and 10; 1.2 ticks and 2.0 ticks run identically, so the finer request buys nothing). Because the elapsed time accumulates in float32 the rounded sum can fall just short and add one more tick: 20, 50, 100 and 160 ticks all run a tick longer (1.000 ms becomes 1.050 ms), while 5 and 10 land exactly. **The second leg is the z-domain model, and it is independent of the code**: with the plant reduced to its inertia the slow root sits at `z = 1 - omega_b*T`, a real-time time constant of `1/omega_b` **independent of the sampling**, which is the whole reason decimation is allowed to be coarse; and the fast root reaches `z = -1` at `omega_b*T = 2`, which at 250 rad/s and 50 us is **160x**. Measured: the settling time is flat to 0.7 % from 1x to 140x (0.0690 against 0.0685 s, overshoot 1.08 % against 1.73 %), and breaks exactly at the predicted point -- 0.1058 s at 160x and 0.4000 s at 180x, with the overshoot going 1.73 % -> 3.62 % -> 7.28 %. The boundary is not a tuning constant anywhere in the code | `test_sil_multirate.cpp` |
| `foc/motor/motor_config.hpp` | L3 | Loop gains derived from nameplate `J`, `b`, `kt`, with the current loop's bandwidth the only free parameter -- and the closed loop it actually produces, measured on an **ideal current loop** so that what is left belongs to the speed law alone. The -3 dB lands at **1.02516 / 1.02661 / 1.02948 / 1.03462 x** the request for 50 / 100 / 200 / 400 rad/s, so the law is good to **3.5 % across a decade** and the residual tracks `wb*dt` (0.0025 at 50 rad/s, 0.02 at 400) -- that is the 20 kHz sampling of a continuous-time design, not an approximate cancellation. **The two nameplate terms are not equal partners**: at 200 rad/s an inertia error of -20 % / +20 % / +50 % moves the bandwidth by **0.799 / 1.201 / 1.504** of the error-free row, i.e. **one for one** -- the bandwidth you get is the bandwidth you asked for times `J_hat/J` -- while a friction error of -50 % / +50 % / +200 % (at a non-negligible `b`) costs only **0.0537 / 0.0543 / 0.0523** per unit, **18x less**, because `kp = wb*J/kt` sets the first-order bandwidth and `ki` only trims the friction term. That is the actionable half: commission `J` before `b`, and a 20 % inertia error is a 20 % bandwidth error. It also bounds what `step_metrics` can attribute to this row: on an ideal current loop the law is within 3.5 %, so the +9.15 % the C++ bench reports is its inner current loop and its plant, not the gain law | `test_motor.cpp` |

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
* **This rig cannot separate the two observer families, and that is recorded as a negative result.** A fourth
  column was measured for `DualInductanceObserver` on the same switching rig (`Lq/Ld = 2.5`, 800 rad/s): at
  `i_d = -2 A` it reads 0.1699 rad against the single-`L` 0.1708, while the model difference between them is
  0.105 rad. A few tenths of a volt of dead time is already 0.1 rad of flux angle at this `psi_f`, so both columns
  are measuring the inverter. The separation is measured on the plant instead (see the row for
  `dual_inductance_observer.hpp`), and no other column in this table should be read as if the rig were finer than
  it is.
* **The `i_d` limit is the single-`L` model's, not MTPA's.** On the same plant with `Lq/Ld = 3` the single-`L`
  bias is `atan((Ld - Lq)*i_d/psi_f)` -- 0.0273 / 0.0433 / 0.0604 / 0.0773 / 0.0939 rad at `i_d` = -0.5 to
  -2.5 A against the law's 0.0210 / 0.0420 / 0.0629 / 0.0838 / 0.1046 -- and it does not fade with speed, while
  `DualInductanceObserver` is flat at 0.0145 to 0.0157 rad over the same range (6.5x better at 2.5 A). The
  pairing matrix therefore allows `MtpaReference` with it, and refuses it only for the single-`L` ones.
* **The two observers are not ranked.** `NLFO` is far better at 20 rad/s (0.2874 against 0.9668) and at 3200 rad/s
  (0.0128 against 0.0968), and worse at 800 rad/s unloaded (0.0568 against 0.0306). Neither dominates.
* **The recorded 0.5079 rad is `NLFO`'s**, reproduced here to four digits, which is also what confirms this rig matches
  the one those records came from. The plan notes that quote observer limits were describing the non-shipping observer.

### Applicability domain of C7 and C8

Both capabilities the strategic map still calls "low" are **L3 headers with measured numbers**; what is narrow is the
region in which they are worth running, and that region used to be prose in five places. It is now a table the gate
recomputes -- `.github/project/domain.json`, checked by `gates.py domain`, whose arithmetic is tied to the implementation
by the case each row names:

| Capability | Domain law | Threshold | Where this repository's machines sit |
| --- | --- | --- | --- |
| C7 field weakening | `lambda = Ld*i_max/psi_f` | `lambda >= 1` | 0.252 (nominal), 0.126 (`Lq/Ld = 2`), 0.101 (2.5), 0.063 (salient test machine) -- **all outside**, the best by 3.9x, so a closed-loop case would be measuring the wrong machine |
| C8 MTPA | `i_char = psi_f/(Lq - Ld)` | `i_char <= i_max` | 19.8 to 23.8 A against a 3 A limit on every salient machine, and infinite (no d-axis term) on the surface one -- **all outside**; the reference is correct, the machine is not a candidate |

A row whose machines all fall outside must carry a note saying what that means, and the gate refuses a silent one.
Nothing here is a claim about a real motor: it is the arithmetic that says which of these two capabilities these
machines can even exercise.

### Claims deliberately not made

These are the claims the evidence levels do not support, listed so nobody has to guess whether they
were overlooked or rejected:

| Claim | Why it is not made |
| --- | --- |
| "Efficient" / "high efficiency" | Nothing here measures input power against shaft power. A modulator and a control loop set the *shape* of the voltages, and efficiency lives in the switching devices, the winding and the modulation index, none of which this project models or measures. Even the SIL plant's loss model is copper plus an optional iron-loss shunt, and the loss numbers are L2 estimates on a plant that has never been current-probed. |
| "Thermal protection" in the sense of knowing the winding temperature | `protection.hpp` integrates `i^2 - q/tau` in A^2*s. That is a lumped copper-loss budget with one time constant; it is not a temperature. `t_derate_*` thresholds are supplied by the caller. Without a thermistor or a thermal observer, this cannot tell you how hot the motor is. |
| "Works down to a few milliamps" | The current-loop evidence is at 0.1 A and above. Below that the resolution of the *model* is the limit, not the ADC: 12-bit quantisation over a +/-10 A range was measured and found to have no effect, but that says nothing about a real sensor's noise, offset drift and amplifier settling. |
| "A sensor-less low-speed speed servo is as good as a sensored one" | Not claimed, and now measured. At 20 rad/s mechanical (140 rad/s electrical) the sensored servo holds 0.000057 rad/s while the sensor-less one holds 0.168 -- a factor of 3000. The angle is **not** the reason: the injection observer reads 0.0015 rad there, which costs one part in a million of torque. The reason is that the speed loop closes on `omega_e / pole_pairs`, and that estimate is itself off by up to 0.7995 rad/s at this speed. The loop does not add to it (0.168 < 0.7995, so it filters rather than amplifies), but it cannot remove it either. Improving low-speed servo accuracy means improving the low-speed **speed** estimate, not the loop. And tuning that estimate's PLL is not enough either: over a 15-point (bandwidth, damping) grid the best point beats the default by **2.75x** (0.2903 against 0.7995 rad/s), the usable band is 10-40 Hz, and 80 Hz is unstable (angle 0.104-0.444 rad, estimate error up to 102 rad/s). The limit is the estimator's structure, not its parameters |
| "MTPA works with the shipped observer" | Not below ~1000 rad/s, and the library now refuses rather than trusting the user to know. MTPA's whole point is `i_d < 0`, and the observer's flux target is a constant `psi_f`; the measured allowance for the 0.15 rad ceiling is 0.04 A at 100 rad/s, 0.12 A at 200, 0.29 A at 400 and 0.92 A at 800, and at `i_d = -1 A` the angle reads 0.5079 rad at 200 rad/s. `MtpaReference::Config::i_d_per_speed_a_s_per_rad` defaults to **0**, so no d-axis current is applied at any speed until the caller states a rate; the same rate buys the full curve at high speed and nothing at standstill. A machine with a position sensor can raise it without limit.  **That limit now has a counterpart**: `DualInductanceObserver` carries both inductances and its angle is flat in `i_d` (0.0145 to 0.0157 rad from 0.5 to 2.5 A), so the pairing matrix marks `MtpaReference` with it as **allow** — the refusal applies to the single-`L` observers, not to MTPA itself. |
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
`python3 .github/misc/gates.py <format|tidy|guard>` for the three the template ships, and
`python3 .github/project/gates.py <gate>` for this project's own (license, strict, claims, docs, boundaries,
capabilities, ci-policy, assume, anchor, scaffold). The split is deliberate: the scaffold's file must stay
byte-identical to the release it came from, so private gates live beside it rather than inside it, and
`scaffold` is the gate that checks that. Two gates are real but not in this job: `fmu` runs on the Ubuntu
leg of the test matrix because it needs the archive that build produced, and `pil` is a **local** gate --
it rebuilds the library for aarch64 and runs the target binary under qemu, which is minutes per push for
evidence that only changes when the control loop does. `project/gates.py all` runs every project gate, and
`ci-policy` fails the build if a workflow ever calls a local-only one.

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

