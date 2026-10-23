// S5 - parameter perturbation: the plant changes by +-30 % on Rs, Ld, Lq while the controller and
// observer keep the nameplate model. S2 is then replayed.
// S5 - 参数摄动：plant 的 Rs/Ld/Lq 各 +-30%，控制器与观测器仍用铭牌模型，随后重跑 S2。

#include <cmath>

#include <gtest/gtest.h>

#include "sil_bench.hpp"
#include "sil_s2_case.hpp"



namespace {

constexpr float kTargetRadS = 100.0F;

struct Perturbation final {
    const char* name;
    int field; // 0 = Rs, 1 = Ld, 2 = Lq
    float scale;
};

auto perturbed(const Perturbation& p) -> foc::sil::PlantParams {
    const foc::sil::PlantParams nominal = foc::sil::nominal_plant_params();
    foc::sil::PlantParams out = nominal;
    if (p.field == 0) {
        out.rs = nominal.rs * p.scale;
    } else if (p.field == 1) {
        out.ld = nominal.ld * p.scale;
    } else {
        out.lq = nominal.lq * p.scale;
    }
    return out;
}

} // namespace



TEST(SilS5Perturbation, StaysStableAndAccurateWithinPlusMinusThirtyPercent) {
    const Perturbation cases[] = {{"rs +30%", 0, 1.3F}, {"rs -30%", 0, 0.7F}, {"ld +30%", 1, 1.3F},
                                  {"ld -30%", 1, 0.7F}, {"lq +30%", 2, 1.3F}, {"lq -30%", 2, 0.7F}};

    for (const Perturbation& c : cases) {
        const foc::sil::S2Result r = foc::sil::run_s2(perturbed(c), kTargetRadS);

        // PRD 8.4: still stable, no divergence or sustained oscillation.
        EXPECT_GE(r.settling_s, 0.0F) << c.name << ": speed never settled";
        EXPECT_LT(r.peak_iq, 2.0F * foc::sil::nominal_spec().i_max_a) << c.name << ": current diverged";
        EXPECT_LT(r.tail_peak_to_peak, 0.05F * kTargetRadS) << c.name << ": sustained oscillation";

        // PRD 8.4: steady-state error <= 5 %.
        EXPECT_LE(r.steady_error_fraction, 0.05F) << c.name << ": steady-state error too large";
    }
}



TEST(SilS5Perturbation, OvershootStaysBounded) {
    const Perturbation cases[] = {{"ld +30%", 1, 1.3F}, {"lq -30%", 2, 0.7F}};

    for (const Perturbation& c : cases) {
        const foc::sil::S2Result r = foc::sil::run_s2(perturbed(c), kTargetRadS);
        // Mismatched inductances desynchronise the current loop; the speed loop must still not ring.
        EXPECT_LE(r.overshoot, 0.20F) << c.name << ": overshoot out of bounds";
    }
}
