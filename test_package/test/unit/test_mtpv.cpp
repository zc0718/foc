// F32 - maximum torque per volt. Above the base speed the voltage ceiling, not the current ceiling, decides the
// operating point, and the best point on that ceiling is where the torque hyperbola is tangent to it. This file pins
// the law three ways: the tangency itself, a brute-force scan of the ellipse, and the three regions the drive
// actually lives in. F32 -
// 每伏最大转矩。基速以上决定工作点的是电压上限而不是电流上限，那个上限上最好的点是转矩双曲线与它相切之处。
// 本文件用三种方式钉住这条律：相切条件本身、对椭圆的暴力扫描、以及驱动器真正所处的三个区间。

#include <cmath>
#include <vector>

#include <gtest/gtest.h>

#include <foc/core/mtpv.hpp>

#include "sil_bench.hpp"

namespace {

using foc::core::MachineParams;
using foc::core::Mtpv;
using foc::core::MtpvConfig;
using foc::core::MtpvPoint;

// The bench's own machine and the bench's own ceiling: the SIL bench runs a 24 V bus, and the most a SVPWM modulator
// can put on the terminals inside its linear range is `v_bus/sqrt(3)`. Taking both from the bench instead of writing
// numbers here is what keeps this file and the simulated drive describing the same hardware.
// 台架自己的机器、台架自己的上限：SIL 台架跑 24 V 母线，而 SVPWM 调制器在线性区内能加到端子上的最大幅值就是
// `v_bus/√3`。两者都取自台架而不是在这里写数字，是让本文件与仿真的驱动器描述同一套硬件的原因。
auto bench_machine(const foc::sil::MachineSpec& spec) -> MtpvConfig {
    const foc::sil::BenchConfig bench = foc::sil::standard_bench_config();
    MtpvConfig config{};
    config.params.rs = spec.rs;
    config.params.ld = spec.ld;
    config.params.lq = spec.lq;
    // The profile carries `kt`, not `psi_f`: `psi_f = kt/(1.5*pole_pairs)` is the one conversion that keeps the voltage
    // equation dimensionally right, and it is the same conversion `motor_config.hpp` makes. 档案里带的是 `kt` 而不是
    // `psi_f`：`ψf = kt/(1.5·极对数)` 是让电压方程量纲唯一正确的换算，也是 `motor_config.hpp` 做的那一次换算。
    config.params.psi_f = spec.kt / (1.5F * static_cast<float>(spec.pole_pairs));
    config.v_max_v = bench.v_bus_v / std::sqrt(3.0F);
    config.i_max_a = spec.i_max_a;
    config.pole_pairs = spec.pole_pairs;
    return config;
}

auto in_domain_config() -> MtpvConfig { return bench_machine(foc::sil::in_domain_spec()); }

auto nominal_config() -> MtpvConfig { return bench_machine(foc::sil::nominal_spec()); }

auto torque_of(const MtpvConfig& config, float id, float iq) -> float {
    const MachineParams& m = config.params;
    return 1.5F * static_cast<float>(config.pole_pairs) * ((m.psi_f * iq) + ((m.ld - m.lq) * id * iq));
}

auto flux_of(const MtpvConfig& config, float id, float iq) -> float {
    const MachineParams& m = config.params;
    return std::hypot((m.ld * id) + m.psi_f, m.lq * iq);
}

/** @brief The tangent-angle residual: `|grad T x grad |psi|| / (|grad T| |grad
 * |psi||)`, 0 at the MTPV point. */
auto tangency_residual(const MtpvConfig& config, float id, float iq) -> float {
    const MachineParams& m = config.params;
    const float k = 1.5F * static_cast<float>(config.pole_pairs);
    const float grad_t_d = k * (m.ld - m.lq) * iq;
    const float grad_t_q = k * (m.psi_f + ((m.ld - m.lq) * id));
    const float grad_f_d = 2.0F * m.ld * ((m.ld * id) + m.psi_f);
    const float grad_f_q = 2.0F * m.lq * m.lq * iq;
    const float cross = (grad_t_d * grad_f_q) - (grad_t_q * grad_f_d);
    const float scale = std::hypot(grad_t_d, grad_t_q) * std::hypot(grad_f_d, grad_f_q);
    return (scale > 0.0F) ? std::fabs(cross) / scale : 0.0F;
}

/** @brief The best torque findable on the voltage ellipse by sampling it -- the
 * falsifier for the closed form. */
auto best_on_ellipse(const MtpvConfig& config, float omega_e, float step_rad, float* id_out, float* iq_out) -> float {
    const MachineParams& m = config.params;
    const float v = config.v_max_v / omega_e;
    float best = -INFINITY;
    for (int n = 0; n <= 20000; ++n) {
        const float t = -3.14159265F + (static_cast<float>(n) * step_rad);
        const float id = ((v * std::cos(t)) - m.psi_f) / m.ld;
        const float iq = (v * std::sin(t)) / m.lq;
        const float torque = torque_of(config, id, iq);
        if (torque > best) {
            best = torque;
            if (id_out != nullptr) {
                *id_out = id;
            }
            if (iq_out != nullptr) {
                *iq_out = iq;
            }
        }
    }
    return best;
}

/** @brief The best torque findable on the current circle by sampling it. */
auto best_on_circle(const MtpvConfig& config, float step_rad) -> float {
    float best = -INFINITY;
    for (int n = 0; n <= 20000; ++n) {
        const float t = -3.14159265F + (static_cast<float>(n) * step_rad);
        const float torque = torque_of(config, config.i_max_a * std::cos(t), config.i_max_a * std::sin(t));
        if (torque > best) {
            best = torque;
        }
    }
    return best;
}

} // namespace

// The law on the constraint it is defined on: the current circle. Sampling the circle is a measurement the code
// cannot influence, so agreement here is evidence that the quadratic in `cos t` is the maximum and not merely a
// stationary point. 律在它被定义的那个约束上：电流圆。对圆采样是代码影响不到的测量，所以这里的一致就是"`cos t`
// 的二次方程给的是 最大值而不是仅仅一个驻点"的证据。
TEST(Mtpv, TheCurrentLimitedPointIsTheMaximumOnTheCircle) {
    const MtpvConfig config = in_domain_config();
    const Mtpv stage{config};
    const MtpvPoint mtpa = stage.mtpa_point();
    ASSERT_EQ(foc::ErrorCode::Ok, mtpa.status);

    EXPECT_NEAR(config.i_max_a, mtpa.current_a, 1.0e-5F);
    const float brute = best_on_circle(config, 1.0e-4F);
    EXPECT_GE(mtpa.torque_nm, brute * (1.0F - 1.0e-5F)) << "closed form " << mtpa.torque_nm << " vs scan " << brute;

    // On the circle the two gradients are not parallel -- the current ceiling is not a curve the torque is tangent to --
    // so the MTPA point must NOT pass the tangency test. Without this the test above could be satisfied by a point that
    // happens to be stationary for the voltage ellipse. 在圆上两梯度并不平行——电流上限不是转矩与之相切的曲线——所以 MTPA
    // 点必须**不能**通过相切检验。没有这一条， 上面那条测试可能被"恰好对电压椭圆驻定"的点满足。
    EXPECT_GT(tangency_residual(config, mtpa.id_a, mtpa.iq_a), 1.0e-3F);
}

// The base speed is *derived*: `v_max` over the flux magnitude at the current-limited point. The check is that the
// point that decides it really does sit on the ellipse at that speed, i.e. the formula and the geometry agree.
// 基速是**算出来的**：`v_max` 除以电流受限点处的磁链幅值。检验的是"决定它的那个点在那个转速下真的落在椭圆上"，
// 即公式与几何一致。
TEST(Mtpv, TheBaseSpeedIsWhereTheCurrentLimitedPointTouchesTheVoltageEllipse) {
    const MtpvConfig config = in_domain_config();
    const Mtpv stage{config};
    const float base = stage.base_speed_rad_s();
    ASSERT_GT(base, 0.0F);
    ASSERT_TRUE(std::isfinite(base));

    const MtpvPoint mtpa = stage.mtpa_point();
    const float v_at_base = config.v_max_v / base;
    const float flux = flux_of(config, mtpa.id_a, mtpa.iq_a);
    EXPECT_NEAR(1.0F, flux / v_at_base, 1.0e-5F);

    // And it is a real speed for this machine, not a number off in the noise: the profile's own mechanical base speed is
    // the electrical one over the pair count.
    // 而且它是这台机器上的一个真实转速，不是噪声里的一个数：档案自己的机械基速就是电基速除以极对数。
    EXPECT_GT(base / static_cast<float>(config.pole_pairs), 10.0F);
}

// The map has three regions and all three are reachable on this machine: current-limited (MTPA), both constraints
// active (the knee riding the current circle), and voltage-limited only (MTPV proper, current coming down). The first
// version of this module had only the first and the third and jumped from one to the other, which left the whole
// middle region -- the one a drive spends its time in -- returning a point that violated the voltage ceiling.
// Counting the regions is what caught that.
// 地图有三个区间，而且这台机器上三个都到达得了：电流受限（MTPA）、两个约束同时生效（拐点贴着电流圆）、只有电压受限
// （真正的 MTPV，电流在往下走）。本模块的第一版只有第一和第三，并且从一个跳到另一个，于是中间整段——驱动器真正待着的
// 那一段——返回了一个违反电压上限的点。数区间这件事把它抓了出来。
TEST(Mtpv, TheMapHasThreeRegionsInThisOrder) {
    const MtpvConfig config = in_domain_config();
    const Mtpv stage{config};
    const float base = stage.base_speed_rad_s();
    const float pinned = config.i_max_a * (1.0F - 1.0e-3F);

    int current_limited = 0;
    int both_active = 0;
    int voltage_only = 0;
    float last_current_limited = 0.0F;
    float first_voltage_only = INFINITY;
    for (int n = 1; n <= 400; ++n) {
        const float omega = base * static_cast<float>(n) / 10.0F;
        const MtpvPoint point = stage.at_speed(omega);
        ASSERT_EQ(foc::ErrorCode::Ok, point.status) << "omega " << omega;
        if (!point.voltage_limited) {
            ++current_limited;
            last_current_limited = omega;
        } else if (point.current_a > pinned) {
            ++both_active;
        } else {
            ++voltage_only;
            if (first_voltage_only > omega) {
                first_voltage_only = omega;
            }
        }
    }
    EXPECT_GT(current_limited, 0);
    EXPECT_GT(both_active, 0);
    EXPECT_GT(voltage_only, 0);
    // In this order: the current-limited region ends first, the voltage-only region begins after it.
    // 按这个顺序：电流受限区先结束，只有电压受限区在它之后才开始。
    EXPECT_LT(last_current_limited, first_voltage_only);
    EXPECT_NEAR(base, last_current_limited, base * 0.11F);
}

// Feasibility, on a grid: no returned point may exceed either ceiling. A policy that reports an unreachable operating
// point is worse than one that reports none, because the current loop will chase it into saturation.
// 可行性，在网格上：返回点不得超出任何一个上限。报告一个够不到的工作点，比报告没有工作点更糟——电流环会追着它跑进饱和。
TEST(Mtpv, EveryReturnedPointIsInsideBothCeilings) {
    const MtpvConfig config = in_domain_config();
    const Mtpv stage{config};
    const float base = stage.base_speed_rad_s();
    for (int n = 0; n <= 800; ++n) {
        const float omega = base * static_cast<float>(n) / 20.0F;
        const MtpvPoint point = stage.at_speed(omega);
        ASSERT_EQ(foc::ErrorCode::Ok, point.status) << "omega " << omega;
        EXPECT_LE(point.current_a, config.i_max_a * (1.0F + 1.0e-5F)) << "omega " << omega;
        const float v_available = (omega > 0.0F) ? (config.v_max_v / omega) : INFINITY;
        EXPECT_LE(flux_of(config, point.id_a, point.iq_a), v_available * (1.0F + 1.0e-5F)) << "omega " << omega;
        // And the reported current is the norm of the reported point, not a separate number that can drift from it.
        // 而且报告的电流是报告点自身的模，不是另一个可能与之漂开的值。
        EXPECT_NEAR(std::hypot(point.id_a, point.iq_a), point.current_a, 1.0e-6F) << "omega " << omega;
    }
}

// The criterion the capability is named after, stated without a tolerance that could hide a wrong law: at the MTPV
// point the torque gradient is parallel to the voltage-ellipse gradient. It is checked on the *returned* point,
// against gradients computed here from the same nameplate -- so a sign error, a swapped `ld`/`lq`, or a missed
// reluctance term all break it. 这项能力以之命名的那条判据，用无法藏住错误规律的容差写出来：在 MTPV
// 点上转矩梯度与电压椭圆梯度平行。它检验的是 **返回的**点，梯度在这里由同一套铭牌独立算出——所以符号错误、`ld`/`lq`
// 写反、漏掉磁阻项，都会让它红。
TEST(Mtpv, InTheVoltageOnlyRegionTheTwoGradientsAreParallel) {
    const MtpvConfig config = in_domain_config();
    const Mtpv stage{config};
    const float base = stage.base_speed_rad_s();
    int checked = 0;
    for (int n = 20; n <= 200; ++n) {
        const float omega = base * static_cast<float>(n) / 2.0F; // 10x .. 100x base
        const MtpvPoint point = stage.at_speed(omega);
        ASSERT_EQ(foc::ErrorCode::Ok, point.status) << "omega " << omega;
        ASSERT_TRUE(point.voltage_limited) << "omega " << omega;
        ASSERT_LT(point.current_a, config.i_max_a * 0.999F) << "omega " << omega; // really the voltage-only region
        EXPECT_LT(tangency_residual(config, point.id_a, point.iq_a), 1.0e-5F) << "omega " << omega;
        EXPECT_NEAR(flux_of(config, point.id_a, point.iq_a), config.v_max_v / omega, config.v_max_v / omega * 1.0e-4F)
            << "omega " << omega;
        ++checked;
    }
    EXPECT_GT(checked, 100);
}

// The closed form against a scan of the same ellipse. Agreement to 1e-5 of the torque means the quadratic in `cos t`
// is not merely stationary but the maximum, and that no grid point does better.
// 闭式解对着同一条椭圆的扫描。转矩一致到 1e-5 意味着 `cos t` 的二次方程不只是驻点而是最大值，而且没有哪个网格点更好。
TEST(Mtpv, TheClosedFormMatchesABruteForceScanOfTheEllipse) {
    const MtpvConfig config = in_domain_config();
    const Mtpv stage{config};
    const float base = stage.base_speed_rad_s();
    for (const float multiple : {5.0F, 12.0F, 30.0F, 90.0F}) {
        const float omega = base * multiple;
        const MtpvPoint point = stage.at_speed(omega);
        ASSERT_EQ(foc::ErrorCode::Ok, point.status);
        float brute_id = 0.0F;
        float brute_iq = 0.0F;
        const float brute = best_on_ellipse(config, omega, 2.0F * 3.14159265F / 20000.0F, &brute_id, &brute_iq);
        EXPECT_GE(point.torque_nm, brute * (1.0F - 1.0e-5F))
            << "omega " << omega << " returned " << point.torque_nm << " scan " << brute << " at id " << brute_id
            << " iq " << brute_iq;
        EXPECT_LT(brute, point.torque_nm * (1.0F + 1.0e-4F)) << "omega " << omega << ": the scan beat the closed form";
    }
}

// In the middle region both ceilings are active at once, so the returned point has to sit on the current circle with
// the voltage exactly used up. That is a property no single-constraint law can produce, which is why it is asserted
// separately from the tangency above.
// 在中间区两个上限同时生效，于是返回点必须坐在电流圆上、同时把电压正好用完。这是任何单约束规律都产不出的性质，
// 所以它与上面那条相切断言分开写。
TEST(Mtpv, InTheMiddleRegionBothCeilingsAreActiveAtOnce) {
    const MtpvConfig config = in_domain_config();
    const Mtpv stage{config};
    const float base = stage.base_speed_rad_s();
    int checked = 0;
    for (int n = 11; n <= 20; ++n) {
        const float omega = base * (1.0F + (static_cast<float>(n) / 100.0F)); // base + 11 % .. base + 20 %
        const MtpvPoint point = stage.at_speed(omega);
        ASSERT_EQ(foc::ErrorCode::Ok, point.status);
        ASSERT_TRUE(point.voltage_limited) << "omega " << omega;
        EXPECT_NEAR(point.current_a, config.i_max_a, config.i_max_a * 1.0e-5F) << "omega " << omega;
        EXPECT_NEAR(flux_of(config, point.id_a, point.iq_a), config.v_max_v / omega, config.v_max_v / omega * 1.0e-4F)
            << "omega " << omega;
        ++checked;
    }
    EXPECT_GT(checked, 0);
}

// The property a speed loop relies on: available torque never rises with speed. A map that is not monotone here is a
// map whose inverse is not a function, and every speed-limiting scheme built on it inherits the ambiguity.
// 速度环依赖的性质：可用转矩从不随转速上升。这里不单调的地图，其逆不是函数，所有建立在它之上的限速方案都继承那份歧义。
TEST(Mtpv, AvailableTorqueNeverRisesWithSpeed) {
    const MtpvConfig config = in_domain_config();
    const Mtpv stage{config};
    const float base = stage.base_speed_rad_s();
    float previous = INFINITY;
    for (int n = 0; n <= 2000; ++n) {
        const float omega = base * static_cast<float>(n) / 25.0F;
        const MtpvPoint point = stage.at_speed(omega);
        ASSERT_EQ(foc::ErrorCode::Ok, point.status);
        EXPECT_LE(point.torque_nm, previous * (1.0F + 1.0e-6F) + 1.0e-9F) << "omega " << omega;
        previous = point.torque_nm;
    }
}

// And the property an integrator upstream relies on: no step in the reference. The region boundaries are where a
// branch-on-speed implementation would produce one.
// 以及上游积分器依赖的性质：参考里没有台阶。区间边界正是"按转速分支"的实现会产生台阶的地方。
TEST(Mtpv, TheReferenceHasNoStepAcrossTheRegionBoundaries) {
    const MtpvConfig config = in_domain_config();
    const Mtpv stage{config};
    const float base = stage.base_speed_rad_s();
    float previous_id = 0.0F;
    float previous_iq = 0.0F;
    float worst = 0.0F;
    float worst_at = 0.0F;
    for (int n = 1; n <= 4000; ++n) {
        const float omega = base * static_cast<float>(n) / 100.0F; // 0 .. 40x base in 1 % steps
        const MtpvPoint point = stage.at_speed(omega);
        ASSERT_EQ(foc::ErrorCode::Ok, point.status);
        const float step = std::hypot(point.id_a - previous_id, point.iq_a - previous_iq);
        if (n > 1 && step > worst) {
            worst = step;
            worst_at = omega;
        }
        previous_id = point.id_a;
        previous_iq = point.iq_a;
    }
    // A 1 % step of the base speed moves the point by at most a tenth of an amp; a branch would show up as an order of
    // magnitude more. Measured worst on this machine is 0.021 A, at the knee. 基速 1 %
    // 的步长最多把点挪动十分之一安培；分支会表现为再大一个量级。这台机器实测最差 0.021 A，在拐点处。
    EXPECT_LT(worst, 0.10F) << "worst step " << worst << " A at omega " << worst_at;
}

// The falsifier for the middle region: the point that a drive would hold if it only ever looked at the current limit
// is not merely suboptimal there, it is *unreachable*. This is the mistake the first version made, written down so
// that removing the knee cannot be done quietly.
// 中间区的反证：一台只看电流上限的驱动器在那里守住的点不只是次优，而是**够不到**。这正是第一版犯的错误，写在这里，
// 好让删掉拐点这件事无法悄悄发生。
TEST(Mtpv, StayingOnTheCurrentLimitThereWouldOvervoltTheDrive) {
    const MtpvConfig config = in_domain_config();
    const Mtpv stage{config};
    const float base = stage.base_speed_rad_s();
    const MtpvPoint mtpa = stage.mtpa_point();
    float worst = 0.0F;
    for (int n = 11; n <= 40; ++n) {
        const float omega = base * (1.0F + (static_cast<float>(n) / 100.0F));
        const float v_available = config.v_max_v / omega;
        const float required = flux_of(config, mtpa.id_a, mtpa.iq_a);
        worst = std::fmax(worst, (required / v_available) - 1.0F);
    }
    EXPECT_GT(worst, 0.10F);
}

// A synchronous reluctance machine has no magnet, so the magnet term vanishes and the reluctance solution has to come
// out on its own. With `psi_f = 0` the torque is `(ld-lq)*id*iq` and the constraint is `(ld*id)^2 + (lq*iq)^2 = V^2`,
// so the maximum is at equal *flux* on the two axes -- `|ld*id| = |lq*iq| = V/sqrt(2)` -- which is `45` degrees in
// flux space and, because the inductances differ, **not** `45` degrees in current space. The assertion is on the
// flux, which is the statement the law actually makes; asserting equal currents would be asserting the wrong physics.
// 同步磁阻机没有磁钢，磁钢项消失，磁阻解必须自己出来。`psi_f = 0` 时转矩是 `(Ld−Lq)·id·iq`、约束是 `(Ld·id)² +
// (Lq·iq)² = V²`，所以极大值在**磁链**上两轴相等处——`|Ld·id| = |Lq·iq| = V/√2`——那是磁链空间的 45°，
// 而由于电感不等，它在电流空间里**不是** 45°。断言写在磁链上，那才是这条律真正说的；断言电流相等是在断言错的物理。
TEST(Mtpv, ASynchronousReluctanceMachineSplitsItsFluxEvenly) {
    MtpvConfig config = in_domain_config();
    config.params.psi_f = 0.0F;
    const Mtpv stage{config};
    ASSERT_EQ(foc::ErrorCode::Ok, stage.validate());
    const float omega = stage.base_speed_rad_s() * 20.0F;
    const MtpvPoint point = stage.at_speed(omega);
    ASSERT_EQ(foc::ErrorCode::Ok, point.status);
    ASSERT_TRUE(point.voltage_limited);
    const float v = config.v_max_v / omega;
    EXPECT_NEAR(v / std::sqrt(2.0F), std::fabs(config.params.ld * point.id_a), v * 1.0e-4F);
    EXPECT_NEAR(v / std::sqrt(2.0F), std::fabs(config.params.lq * point.iq_a), v * 1.0e-4F);
    // And the torque is the reluctance term alone, on the signs that make it positive.
    // 而转矩只有磁阻那一项，符号取使它为正的那一组。
    EXPECT_GT(point.torque_nm, 0.0F);
    const float expected =
        1.5F * static_cast<float>(config.pole_pairs) * (config.params.ld - config.params.lq) * point.id_a * point.iq_a;
    EXPECT_NEAR(expected, point.torque_nm, point.torque_nm * 1.0e-5F);
}

// A surface machine has no saliency, so there is no reluctance torque to trade against the voltage: the answer must
// collapse to flux-nulling on d and everything the ellipse allows on q. This is also the degenerate branch of the
// quadratic in `cos t` (its leading coefficient vanishes), which is where a naive root would divide by zero.
// 表贴机没有凸极，没有磁阻转矩可与电压交换：答案必须塌缩成 d 轴削磁、q 轴拿到椭圆允许的全部。这也是 `cos t` 二次方程
// 的退化分支（二次项系数消失），天真地取根会在这里除零。
TEST(Mtpv, ASurfaceMachineCollapsesToFluxNullingOnD) {
    MtpvConfig config = in_domain_config();
    config.params.lq = config.params.ld;
    const Mtpv stage{config};
    ASSERT_EQ(foc::ErrorCode::Ok, stage.validate());
    const float omega = stage.base_speed_rad_s() * 30.0F;
    const MtpvPoint point = stage.at_speed(omega);
    ASSERT_EQ(foc::ErrorCode::Ok, point.status);
    ASSERT_TRUE(point.voltage_limited);
    EXPECT_NEAR(-config.params.psi_f / config.params.ld, point.id_a, std::fabs(point.id_a) * 1.0e-4F);
    const float v = config.v_max_v / omega;
    EXPECT_NEAR(v / config.params.lq, point.iq_a, point.iq_a * 1.0e-4F);
    EXPECT_NEAR(1.5F * static_cast<float>(config.pole_pairs) * config.params.psi_f * point.iq_a, point.torque_nm,
                point.torque_nm * 1.0e-5F);
}

// The profile this capability is verified on, checked against the two numbers `domain.json` records for it: `lambda`
// and `i_char`. The two tables are written by different languages and read by different gates; this is the line that
// stops them from drifting apart. 这项能力被验证于其上的档案，对着 `domain.json` 为它记录的两个数核对：`lambda` 与
// `i_char`。两张表由不同语言写出、 被不同的门禁读取；这一行是阻止它们各自漂走的那一行。
TEST(Mtpv, TheInDomainProfileIsTheMachineThisLawNeedsAndTheNominalOneIsNot) {
    const MtpvConfig config = in_domain_config();
    const MachineParams& m = config.params;
    const float lambda = (m.ld * config.i_max_a) / m.psi_f;
    const float i_char = m.psi_f / m.ld;
    EXPECT_NEAR(2.52F, lambda, 0.01F);
    EXPECT_NEAR(1.190476F, i_char, 0.001F);
    // `lambda > 1` is exactly "the current limit can null the flux", which is what makes an MTPV point exist at all.
    // `lambda > 1` 正是"电流限幅能把磁链削到零"，也就是让 MTPV 点得以存在的东西。
    EXPECT_GT(lambda, 1.0F);

    // The nominal machine is the counter-example the domain table also records: its characteristic current is far above
    // the current limit, so the flux can never be nulled and there is no voltage-only region to verify anything on.
    // 标称机是域表同样记录的对照：它的特征电流远高于电流限幅，磁链永远削不到零，也就没有哪个"只有电压受限"的区可用来验证。
    const MtpvConfig nominal = nominal_config();
    const float nominal_lambda = (nominal.params.ld * nominal.i_max_a) / nominal.params.psi_f;
    EXPECT_LT(nominal_lambda, 1.0F);
    EXPECT_GT(nominal.params.psi_f / nominal.params.ld, nominal.i_max_a);
}

// Refusals, one line each, and the speed sign: the map is symmetric in the direction of rotation because no `Rs`
// enters it, so a negative speed must return the mirror of the positive one rather than a different point.
// 拒绝用例各一行，外加转速符号：本规律不含 `Rs`，地图对转向对称，所以负转速必须返回正转速的镜像而不是另一个点。
TEST(Mtpv, RefusesWhatItCannotComputeAndMirrorsTheSpeedSign) {
    const MtpvConfig good = in_domain_config();
    EXPECT_EQ(foc::ErrorCode::Ok, Mtpv{good}.validate());

    MtpvConfig bad = good;
    bad.v_max_v = 0.0F;
    EXPECT_EQ(foc::ErrorCode::Param, Mtpv{bad}.validate());
    bad = good;
    bad.i_max_a = -1.0F;
    EXPECT_EQ(foc::ErrorCode::Param, Mtpv{bad}.validate());
    bad = good;
    bad.params.ld = 0.0F;
    EXPECT_EQ(foc::ErrorCode::Param, Mtpv{bad}.validate());
    bad = good;
    bad.params.psi_f = -0.001F;
    EXPECT_EQ(foc::ErrorCode::Param, Mtpv{bad}.validate());
    bad = good;
    bad.pole_pairs = 0U;
    EXPECT_EQ(foc::ErrorCode::Param, Mtpv{bad}.validate());
    bad = good;
    bad.v_max_v = NAN;
    EXPECT_EQ(foc::ErrorCode::Param, Mtpv{bad}.validate());

    const Mtpv stage{good};
    EXPECT_EQ(foc::ErrorCode::Param, stage.at_speed(NAN).status);
    EXPECT_EQ(foc::ErrorCode::Param, stage.at_speed(INFINITY).status);
    EXPECT_EQ(0.0F, Mtpv{bad}.base_speed_rad_s());

    const MtpvPoint forward = stage.at_speed(stage.base_speed_rad_s() * 10.0F);
    const MtpvPoint backward = stage.at_speed(-stage.base_speed_rad_s() * 10.0F);
    EXPECT_FLOAT_EQ(forward.id_a, backward.id_a);
    EXPECT_FLOAT_EQ(forward.iq_a, backward.iq_a);
    EXPECT_EQ(forward.voltage_limited, backward.voltage_limited);

    // Standstill: the voltage ceiling costs nothing, so the current limit is the whole answer and nothing is limited by
    // the ellipse. 静止：电压上限不花钱，于是电流限幅就是全部答案，没有任何东西被椭圆限制。
    const MtpvPoint standstill = stage.at_speed(0.0F);
    EXPECT_EQ(foc::ErrorCode::Ok, standstill.status);
    EXPECT_FALSE(standstill.voltage_limited);
    EXPECT_FLOAT_EQ(stage.mtpa_point().torque_nm, standstill.torque_nm);
}
