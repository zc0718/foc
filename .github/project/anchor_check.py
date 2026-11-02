#!/usr/bin/env python3
"""Check that a parameter table can support the claims it is used for, before any number is quoted from it.

Three checks, in the order that matters:

  1. **saliency** -- a machine claimed to witness MTPA or field weakening must have ``Lq > Ld``. A table where it
     does not is either a surface machine described as salient or two columns printed the wrong way round; both
     happen in the literature, so this is measured rather than trusted.
  2. **leverage** -- the classic characteristic ratio ``k = psi_f / ((Lq - Ld) * i_max)``. At ``k = 1`` the
     reluctance torque is the same size as the magnet torque at rated current and MTPA is worth ~30 %; at
     ``k = 4`` it is worth ~3 %. Beyond ``LEVERAGE_LIMIT`` the machine is a poor witness: the claim would be true
     and about nothing. Note the arithmetic: the repository's own ``i_char = psi_f / Ld`` equals the classic
     characteristic current only when ``Lq/Ld = 2``, so ``Ld`` alone cannot decide this for other saliencies --
     ``test_mtpa.cpp`` measures at ``Lq/Ld = 2``, where the two coincide, which is why the difference is easy to
     miss.
  3. **magnitude** -- the MTPA gain computed from the same torque law has to land within ``MAGNITUDE_ORDER`` of the
     number the source reports. A larger disagreement means the formula or the table is wrong, and this gate
     refuses to guess which.

    python3 .github/misc/anchor_check.py            # every machine in anchors.json
    python3 .github/misc/anchor_check.py --selftest # prove the gate can fail, then pass
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REGISTRY = Path(__file__).resolve().with_name("anchors.json")
# Policy, not physics: at k = 2 MTPA is worth ~10 % (the repository's own table in PLAN 0.1), which is about the
# last point where quoting a gain is worth a reader's attention.
LEVERAGE_LIMIT = 2.0
MAGNITUDE_ORDER = 3.0
SALIENCY_CLAIMS = ("mtpa", "field_weakening")


def mtpa_gain(m: dict) -> tuple[float, float]:
    """(peak gain over i_d = 0 in percent, i_d* in A) by scanning the torque surface inside the current circle."""
    ld, lq, psi, pp, imax = m["Ld_H"], m["Lq_H"], m["psi_Wb"], m["pole_pairs"], m["i_max_A"]
    if lq <= ld:
        return 0.0, 0.0
    torque = lambda iq, idv: 1.5 * pp * (psi * iq + (ld - lq) * idv * iq)
    base = torque(imax, 0.0)
    steps = 400
    best, best_id = float("-inf"), 0.0
    for k in range(1, steps + 1):
        iq = imax * k / steps
        for j in range(0, steps + 1):
            idv = -imax * j / steps
            if iq * iq + idv * idv > imax * imax:
                break
            t = torque(iq, idv)
            if t > best:
                best, best_id = t, idv
    return (100.0 * (best / base - 1.0) if base else 0.0), best_id


def check(m: dict) -> tuple[list[str], dict]:
    """Return (problems, measurements)."""
    problems: list[str] = []
    ld, lq, psi, imax = m["Ld_H"], m["Lq_H"], m["psi_Wb"], m["i_max_A"]
    claims = m.get("claims") or []
    dl = lq - ld
    i_char_a = psi / ld if ld else 0.0                # the repository's convention
    ratio_repo = i_char_a / imax if imax else 0.0     # what README and PLAN print as `i_char/i_max`
    k_classic = psi / (dl * imax) if dl > 0 and imax else float("inf")
    gain, _id = mtpa_gain(m)
    measurements = {"Lq/Ld": lq / ld if ld else 0.0, "i_char_A": i_char_a, "ratio_repo": ratio_repo,
                    "k_classic": k_classic, "gain": gain}

    if any(c in SALIENCY_CLAIMS for c in claims) and dl <= 0:
        problems.append(f"Lq/Ld = {lq / ld:.2f} <= 1 contradicts the salient premise its claims "
                        f"({', '.join(claims)}) require")
    if "mtpa" in claims and dl > 0 and k_classic > LEVERAGE_LIMIT:
        problems.append(f"k = psi/(dL*i_max) = {k_classic:.2f} > {LEVERAGE_LIMIT:.1f}: MTPA is worth {gain:.1f}% "
                        f"here, a true statement about a machine nobody would quote it for")
    reported = m.get("reported_mtpa_gain_percent")
    if reported and gain:
        ratio = max(gain / reported, reported / gain)
        if ratio > MAGNITUDE_ORDER:
            problems.append(f"computed MTPA gain {gain:.1f}% is {ratio:.1f}x away from the {reported:.1f}% the "
                            f"source reports -- the formula or the table is wrong and this gate will not guess")
    return problems, measurements


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--selftest", action="store_true", help="also assert the as-published table IS rejected")
    args = ap.parse_args()

    data = json.loads(REGISTRY.read_text(encoding="utf-8"))
    failures = 0
    for m in data.get("machines") or []:
        problems, x = check(m)
        expect = m.get("expect", "accept")
        verdict = "REJECT" if problems else "accept"
        ok = (verdict == "accept") == (expect == "accept")
        k_text = "  n/a" if x["k_classic"] == float("inf") else f"{x['k_classic']:6.2f}"
        print(f"[anchor] {'ok  ' if ok else 'FAIL'} {m['id']:20} Lq/Ld={x['Lq/Ld']:5.2f} "
              f"i_char={x['i_char_A']:7.2f}A (repo ratio {x['ratio_repo']:5.2f}) k={k_text} "
              f"mtpa={x['gain']:6.1f}%  {verdict}")
        for p in problems:
            print(f"[anchor]        {p}")
        if not ok:
            failures += 1
            print(f"[anchor]        expected {expect}: "
                  f"{m.get('reject_reason', 'the gate and the registry disagree')}")

    if args.selftest:
        bad = next((m for m in data["machines"] if m.get("expect") == "reject"), None)
        if bad is None or not check(bad)[0]:
            print("[anchor] SELFTEST FAILED: no table was rejected, so this gate cannot fail")
            failures += 1
        else:
            print("[anchor] selftest: the as-published table is rejected, so this gate can fail")

    if failures:
        print(f"[anchor] FAIL: {failures} machine(s) cannot support the claims listed for them")
        return 1
    print(f"[anchor] OK ({len(data['machines'])} machine(s) checked against the claims they witness)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
