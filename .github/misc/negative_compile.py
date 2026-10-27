#!/usr/bin/env python3
"""Negative compile gate (G3 / PRD 8.6,载体 D9): a stub that does not satisfy a contract must fail
to compile, and the diagnostic must point at the contract it violates.

Why a separate script instead of `try_compile`
----------------------------------------------
`try_compile` can only report "some error"; it cannot tell whether the compiler failed *because of
the contract* or because of a typo in the fixture. Feeding one small translation unit per case to
the compiler directly lets the gate assert three things at once:

  1. the compiler exits non-zero (the contract really is enforced);
  2. the diagnostic names the contract header (the failure is attributable);
  3. the *conforming* twin of the same case compiles cleanly (the check is not vacuous).

Point 3 is the important one: without a positive control a gate can pass by breaking everything.

Usage
-----
    python3 .github/misc/negative_compile.py            # run every case
    python3 .github/misc/negative_compile.py --list     # list case names
    python3 .github/misc/negative_compile.py --verbose  # echo the compiler diagnostics
    python3 .github/misc/negative_compile.py -I extra/include

Exit code: 0 when every negative case fails for the right reason *and* every positive control
compiles; otherwise the number of failing cases (capped at 100).
"""
import argparse
import os
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# Each case: name, contract header, trait (in namespace hal::concepts or core::concepts), the type
# name, the member the case is about, the body of the fake type, and whether it must FAIL.
CASES = [
    {
        "name": "pwm_missing_fault",
        "header": "foc/hal/pwm.hpp",
        "trait": "foc::hal::concepts::is_pwm_v",
        "type": "FakePwm",
        "member": "fault",
        "must_fail": True,
        "body": """
class FakePwm {
  public:
    auto configure(float, float) noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
    void write_duty(foc::core::Uvw<float>) noexcept {}
    auto enable() noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
    void disable() noexcept {}
    // fault() is missing on purpose
};
""",
    },
    {
        "name": "pwm_wrong_write_duty_signature",
        "header": "foc/hal/pwm.hpp",
        "trait": "foc::hal::concepts::is_pwm_v",
        "type": "WrongPwm",
        "member": "write_duty",
        "must_fail": True,
        "body": """
class WrongPwm {
  public:
    auto configure(float, float) noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
    void write_duty(int) noexcept {}  // wrong parameter type
    auto enable() noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
    void disable() noexcept {}
    auto fault() const noexcept -> bool { return false; }
};
""",
    },
    {
        "name": "current_sense_missing_sample_rate",
        "header": "foc/hal/current_sense.hpp",
        "trait": "foc::hal::concepts::is_current_sense_v",
        "type": "FakeSense",
        "member": "sample_rate_hz",
        "must_fail": True,
        "body": """
class FakeSense {
  public:
    auto read_uvw() noexcept -> foc::core::Uvw<float> { return foc::core::Uvw<float>{}; }
    auto calibrate_offset() noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
    // sample_rate_hz() is missing on purpose
};
""",
    },
    {
        "name": "position_missing_has_sensor",
        "header": "foc/hal/position.hpp",
        "trait": "foc::hal::concepts::is_position_sensor_v",
        "type": "FakePosition",
        "member": "has_sensor",
        "must_fail": True,
        "body": """
class FakePosition {
  public:
    auto angle_mech_rad() noexcept -> float { return 0.0F; }
    auto velocity_rad_s() noexcept -> float { return 0.0F; }
    // has_sensor() is missing on purpose
};
""",
    },
    {
        "name": "dc_link_missing_voltage",
        "header": "foc/hal/dc_link.hpp",
        "trait": "foc::hal::concepts::is_dc_link_v",
        "type": "FakeDcLink",
        "member": "voltage_v",
        "must_fail": True,
        "body": """
class FakeDcLink {
  public:
    // voltage_v() is missing on purpose
};
""",
    },
    {
        "name": "clock_missing_now_us",
        "header": "foc/hal/clock.hpp",
        "trait": "foc::hal::concepts::is_clock_v",
        "type": "FakeClock",
        "member": "now_us",
        "must_fail": True,
        "body": """
class FakeClock {
  public:
    // now_us() is missing on purpose
};
""",
    },
    {
        "name": "observer_missing_update",
        "header": "foc/core/observer/observer_policy.hpp",
        "trait": "foc::core::concepts::is_observer_v",
        "type": "FakeObserver",
        "member": "update",
        "must_fail": True,
        "body": """
class FakeObserver {
  public:
    // update() is missing on purpose
};
""",
    },
    {
        "name": "observer_wrong_return_type",
        "header": "foc/core/observer/observer_policy.hpp",
        "trait": "foc::core::concepts::is_observer_v",
        "type": "WrongObserver",
        "member": "update",
        "must_fail": True,
        "body": """
class WrongObserver {
  public:
    auto update(foc::core::Ab<float>, foc::core::Ab<float>, float) noexcept -> float { return 0.0F; }
};
""",
    },
    {
        "name": "reference_generator_missing_generate",
        "header": "foc/core/reference.hpp",
        "trait": "foc::core::concepts::is_reference_generator_v",
        "type": "FakeReference",
        "member": "generate",
        "must_fail": True,
        "body": """
class FakeReference {
  public:
    // generate() is missing on purpose
};
""",
    },
    {
        "name": "feedforward_missing_generate",
        "header": "foc/core/feedforward.hpp",
        "trait": "foc::core::concepts::is_feedforward_v",
        "type": "FakeFeedforward",
        "member": "generate",
        "must_fail": True,
        "body": """
class FakeFeedforward {
  public:
    // generate() is missing on purpose
};
""",
    },
    {
        "name": "controller_missing_reset",
        "header": "foc/core/position.hpp",
        "trait": "foc::core::concepts::is_controller_v",
        "type": "FakeController",
        "member": "reset",
        "must_fail": True,
        "body": """
class FakeController {
  public:
    auto step(float, float, float) noexcept -> float { return 0.0F; }
    // reset() is missing on purpose
};
""",
    },
    # Positive controls: the same fixtures, complete. They must compile.
    {
        "name": "control_pwm_conforms",
        "header": "foc/hal/pwm.hpp",
        "trait": "foc::hal::concepts::is_pwm_v",
        "type": "GoodPwm",
        "member": "fault",
        "must_fail": False,
        "body": """
class GoodPwm {
  public:
    auto configure(float, float) noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
    void write_duty(foc::core::Uvw<float>) noexcept {}
    auto enable() noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
    void disable() noexcept {}
    auto fault() const noexcept -> bool { return false; }
};
""",
    },
    {
        "name": "control_observer_conforms",
        "header": "foc/core/observer/observer_policy.hpp",
        "trait": "foc::core::concepts::is_observer_v",
        "type": "GoodObserver",
        "member": "update",
        "must_fail": False,
        "body": """
class GoodObserver {
  public:
    auto update(foc::core::Ab<float>, foc::core::Ab<float>, float) noexcept -> foc::core::ObsState {
        return foc::core::ObsState{};
    }
};
""",
    },
    {
        "name": "control_reference_conforms",
        "header": "foc/core/reference.hpp",
        "trait": "foc::core::concepts::is_reference_generator_v",
        "type": "GoodReference",
        "member": "generate",
        "must_fail": False,
        "body": """
class GoodReference {
  public:
    auto generate(const foc::core::ReferenceInput& in) noexcept -> foc::core::Dq<float> {
        return foc::core::Dq<float>{0.0F, in.iq_command_a};
    }
};
""",
    },
    {
        "name": "control_feedforward_conforms",
        "header": "foc/core/feedforward.hpp",
        "trait": "foc::core::concepts::is_feedforward_v",
        "type": "GoodFeedforward",
        "member": "generate",
        "must_fail": False,
        "body": """
class GoodFeedforward {
  public:
    auto generate(const foc::core::FeedforwardInput&) noexcept -> foc::core::Dq<float> {
        return foc::core::Dq<float>{};
    }
};
""",
    },
    {
        "name": "modulator_two_arg_only",
        "header": "foc/core/svpwm.hpp",
        "trait": "foc::core::concepts::is_modulator_v",
        "type": "NarrowModulator",
        "member": "modulate",
        "must_fail": True,
        "body": """
class NarrowModulator {
  public:
    // The pre-widening shape: a voltage and a bus, and therefore no way to linearise the inverter.
    // 加宽之前的形状：一个电压与一个母线，因此无法线性化逆变器。
    [[nodiscard]] auto modulate(foc::core::Ab<float>, float) noexcept -> foc::core::SvpwmResult;
};
""",
    },
    {
        "name": "control_modulator_conforms",
        "header": "foc/core/svpwm.hpp",
        "trait": "foc::core::concepts::is_modulator_v",
        "type": "GoodModulator",
        "member": "modulate",
        "must_fail": False,
        "body": """
class GoodModulator {
  public:
    [[nodiscard]] auto modulate(const foc::core::ModulatorInput& request) noexcept -> foc::core::SvpwmResult {
        return foc::core::Svpwm::modulate(request);
    }
};
""",
    },
    {
        "name": "position_control_with_three_arg_step",
        "header": "foc/core/position.hpp",
        "trait": "foc::core::concepts::is_position_control_v",
        "type": "OldPositionController",
        "member": "step",
        "must_fail": True,
        "body": """
class OldPositionController {
  public:
    // The pre-widening shape: a setpoint, a measurement, a step -- and no room for the move's own
    // limits. It must not be accepted silently, because a position slot that ignores per-move
    // limits would look configured and behave unconstrained.
    // 加宽之前的形状：给定、实测、步长——没有位置留给这次运动自己的限制。它不得被静默接受，
    // 因为忽略按次限制的位置槽位会"看起来配置好了"，实际行为却毫无约束。
    auto step(float, float, float) noexcept -> float { return 0.0F; }
    void reset() noexcept {}
};
""",
    },
    {
        "name": "control_position_control_conforms",
        "header": "foc/core/position.hpp",
        "trait": "foc::core::concepts::is_position_control_v",
        "type": "GoodPositionController",
        "member": "step",
        "must_fail": False,
        "body": """
class GoodPositionController {
  public:
    auto step(const foc::core::PositionInput&, float, float) noexcept -> float { return 0.0F; }
    void reset() noexcept {}
};
""",
    },
    {
        "name": "aligner_missing_step",
        "header": "foc/core/alignment.hpp",
        "trait": "foc::core::concepts::is_aligner_v",
        "type": "FakeAligner",
        "member": "step",
        "must_fail": True,
        "body": """
class FakeAligner {
  public:
    auto validate() const noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
    auto begin(std::uint32_t) noexcept -> bool { return true; }
    void reset() noexcept {}
    // step() is missing on purpose
};
""",
    },
    {
        "name": "aligner_wrong_step_return_type",
        "header": "foc/core/alignment.hpp",
        "trait": "foc::core::concepts::is_aligner_v",
        "type": "WrongAligner",
        "member": "step",
        "must_fail": True,
        "body": """
class WrongAligner {
  public:
    auto validate() const noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
    auto begin(std::uint32_t) noexcept -> bool { return true; }
    // Returns the current only: no forced angle, so the rotor cannot be aligned.
    auto step(float, float) noexcept -> float { return 0.0F; }
    void reset() noexcept {}
};
""",
    },
    {
        "name": "control_aligner_conforms",
        "header": "foc/core/alignment.hpp",
        "trait": "foc::core::concepts::is_aligner_v",
        "type": "GoodAligner",
        "member": "step",
        "must_fail": False,
        "body": """
class GoodAligner {
  public:
    auto validate() const noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
    auto begin(std::uint32_t) noexcept -> bool { return true; }
    auto step(float, float) noexcept -> foc::core::AlignOutput {
        return foc::core::AlignOutput{0.0F, 0.0F, false, false};
    }
    void reset() noexcept {}
};
""",
    },
]

# A probe is a direct use of the member under test. The trait detects the missing member
# silently, so the probe is what makes the *compiler* name the offending member instead of the
# gate trusting its own assertion message.
PROBES = {
    "pwm_missing_fault": "void probe(FakePwm& p) {{ (void)p.fault(); }}",
    "pwm_wrong_write_duty_signature":
        "void probe(WrongPwm& p) {{ p.write_duty(foc::core::Uvw<float>{{}}); }}",
    "current_sense_missing_sample_rate": "void probe(FakeSense& p) {{ (void)p.sample_rate_hz(); }}",
    "position_missing_has_sensor": "void probe(FakePosition& p) {{ (void)p.has_sensor(); }}",
    "dc_link_missing_voltage": "void probe(FakeDcLink& p) {{ (void)p.voltage_v(); }}",
    "clock_missing_now_us": "void probe(FakeClock& p) {{ (void)p.now_us(); }}",
    "observer_missing_update":
        "void probe(FakeObserver& p) {{ p.update(foc::core::Ab<float>{{}}, foc::core::Ab<float>{{}}, 0.0F); }}",
    "observer_wrong_return_type":
        "void probe(WrongObserver& p) {{ foc::core::ObsState s = p.update(foc::core::Ab<float>{{}}, "
        "foc::core::Ab<float>{{}}, 0.0F); (void)s; }}",
    "control_pwm_conforms": "void probe(GoodPwm& p) {{ (void)p.fault(); }}",
    "control_observer_conforms":
        "void probe(GoodObserver& p) {{ foc::core::ObsState s = p.update(foc::core::Ab<float>{{}}, "
        "foc::core::Ab<float>{{}}, 0.0F); (void)s; }}",
    "reference_generator_missing_generate":
        "void probe(FakeReference& p) {{ foc::core::Dq<float> r = p.generate(foc::core::ReferenceInput{{}}); "
        "(void)r; }}",
    "control_reference_conforms":
        "void probe(GoodReference& p) {{ foc::core::Dq<float> r = p.generate(foc::core::ReferenceInput{{}}); "
        "(void)r; }}",
    "feedforward_missing_generate":
        "void probe(FakeFeedforward& p) {{ foc::core::Dq<float> r = p.generate(foc::core::FeedforwardInput{{}}); "
        "(void)r; }}",
    "control_feedforward_conforms":
        "void probe(GoodFeedforward& p) {{ foc::core::Dq<float> r = p.generate(foc::core::FeedforwardInput{{}}); "
        "(void)r; }}",
    "modulator_two_arg_only":
        "void probe(NarrowModulator& p) {{ foc::core::SvpwmResult r = p.modulate(foc::core::ModulatorInput{{}}); "
        "(void)r; }}",
    "control_modulator_conforms":
        "void probe(GoodModulator& p) {{ foc::core::SvpwmResult r = p.modulate(foc::core::ModulatorInput{{}}); "
        "(void)r; }}",
    "position_control_with_three_arg_step":
        "void probe(OldPositionController& p) {{ float v = p.step(foc::core::PositionInput{{}}, 0.0F, 0.0F); "
        "(void)v; }}",
    "control_position_control_conforms":
        "void probe(GoodPositionController& p) {{ float v = p.step(foc::core::PositionInput{{}}, 0.0F, 0.0F); "
        "(void)v; }}",
    "aligner_missing_step": "void probe(FakeAligner& p) {{ (void)p.step(0.0F, 0.0F); }}",
    "aligner_wrong_step_return_type":
        "void probe(WrongAligner& p) {{ foc::core::AlignOutput s = p.step(0.0F, 0.0F); (void)s; }}",
    "control_aligner_conforms":
        "void probe(GoodAligner& p) {{ foc::core::AlignOutput s = p.step(0.0F, 0.0F); (void)s; }}",
    "controller_missing_reset": "void probe(FakeController& p) {{ p.reset(); }}",
}


TEMPLATE = """// Generated by .github/misc/negative_compile.py -- {name}
// Case: {member} must be {verdict} for the {header} contract.
#include <foc/core/traits.hpp>
#include <{header}>

namespace {{

{body}

{probe}

}} // namespace

static_assert({trait}<{type}>, "{type} must satisfy the contract in {header} ({member})");
"""


def build_tu(case):
    return TEMPLATE.format(
        name=case["name"],
        member=case["member"],
        verdict="required" if case["must_fail"] else "present",
        header=case["header"],
        body=case["body"].strip("\n"),
        trait=case["trait"],
        type=case["type"],
        probe=PROBES[case["name"]],
    )


def run_case(case, compiler, include_dirs, std, verbose):
    """Compile one case; returns (ok, detail)."""
    with tempfile.TemporaryDirectory(prefix="foc-negative-") as tmp:
        source = Path(tmp) / f"{case['name']}.cpp"
        source.write_text(build_tu(case), encoding="utf-8")
        command = [compiler, f"-std={std}", "-fsyntax-only", "-Wall", "-Wextra"]
        command += [f"-I{d}" for d in include_dirs]
        command.append(str(source))
        completed = subprocess.run(command, capture_output=True, text=True, check=False)

    output = (completed.stdout or "") + (completed.stderr or "")
    if verbose:
        print(f"--- {case['name']} (exit {completed.returncode})")
        print(output.strip()[:2000])

    if case["must_fail"]:
        if completed.returncode == 0:
            return False, "compiled, but this stub must be rejected"
        # Attribution: the diagnostic must name the contract header. g++ prints the
        # static_assert message, which carries the path by construction.
        if case["header"] not in output:
            return False, f"failed without naming {case['header']}"
        # The compiler itself must name the member (via the probe), so a green gate is not just
        # the gate reading back its own message.
        if case["member"] not in output:
            return False, f"failed without naming the member {case['member']}"
        return True, "rejected as required"

    if completed.returncode != 0:
        return False, "the conforming control failed to compile"
    return True, "accepted"


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="list the case names and exit")
    parser.add_argument("--verbose", action="store_true", help="echo compiler diagnostics")
    parser.add_argument("-I", "--include", action="append", default=[], help="extra include dir")
    parser.add_argument("--compiler", default=os.environ.get("CXX", "c++"), help="compiler to use")
    args = parser.parse_args()

    if args.list:
        for case in CASES:
            verdict = "must FAIL" if case["must_fail"] else "must PASS"
            print(f"{case['name']:<38} {verdict:<10} {case['header']} [{case['member']}]")
        return 0

    include_dirs = [str(ROOT / "include")] + [str(Path(p).resolve()) for p in args.include]
    std = "c++17"
    failures = 0
    for case in CASES:
        ok, detail = run_case(case, args.compiler, include_dirs, std, args.verbose)
        status = "ok  " if ok else "FAIL"
        print(f"[negative] {status} {case['name']:<38} {detail}")
        if not ok:
            failures += 1

    total = len(CASES)
    print(f"[negative] {total - failures}/{total} cases behaved as specified ({args.compiler}, -std={std})")
    return min(failures, 100)


if __name__ == "__main__":
    sys.exit(main())
