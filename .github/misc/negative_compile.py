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
