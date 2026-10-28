// The pairing constraint, measured rather than assumed: the shipped single-L observer and any
// non-zero d-axis current are compatible only above a speed that nothing in this library states.
// This file holds the table so the limit can be quoted instead of guessed.
// 配对约束，量出来的而不是假设的：出厂的单 L 观测器与任何非零 d 轴电流，只在某个速度以上共存，
// 而这个速度库里没有任何地方写出来。本文件把表留在原处，好让这件事可以被引用而不是靠猜。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/nonlinear_flux_observer.hpp>
#include <foc/core/reference.hpp>

#include "sil_bench.hpp"
#include "sil_switching_rig.hpp"

namespace foc {

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

/**
 * @brief [en] The d-axis current the shipped observer can tolerate, as a function of speed.
 *
 *     MTPA asks for a negative `i_d` by definition; field weakening asks for a larger one. The shipped observer's flux
 *     target is the constant `psi_f`, so any `i_d` moves the flux it believes it is tracking, and at low speed the
 *     back-EMF is too small to outvote that error. The published boundary is 0.15 rad.
 *
 *     What the table below says. At `i_d = 0` the error is inside 0.15 rad from 100 rad/s upward; below that it is not,
 *     which is the same floor the README already states ("< 50 rad/s: no angle claim", and 0.16 rad and above at 50).
 *     At `i_d = -1 A` the same observer reads three times the ceiling at 200 rad/s and is harmless at 3200 rad/s -- so
 *     the constraint is **not** "no d-axis current", it is "no d-axis current below a speed", and the safe `|i_d|`
 *     grows with speed. That is why the limit this library enforces is a rate (A per rad/s) rather than a constant, and
 *     why the default rate is zero: a constant cannot be right at both ends.
 *
 * @brief [zh] 出厂观测器能容忍的 d 轴电流，作为转速的函数。
 *
 *     MTPA 按定义就要负 `i_d`，弱磁要得更多。出厂观测器的磁链目标是常数 `ψf`，所以任何 `i_d` 都会挪动它以为自己 在跟踪
 *     的那个磁链，而低速下反电势太小，压不住这个误差。公开边界是 0.15 rad。
 *
 *     下表说的。`i_d = 0` 时从 100 rad/s 往上都在 0.15 rad 以内；再低就不在了，而这正是 README 已经写下的那条 地板（"<
 *     50 rad/s 不给角度主张"，50 处读到 0.16 rad 以上）。`i_d = −1 A` 时同一个观测器在 200 rad/s 读到 三倍天花板，在
 *     3200 rad/s 却无害——所以约束**不是**"不许有 d 轴电流"，而是"某个转速以下不许有"，且安全的 `|i_d|` 随转速增大。这就
 *     是本库强制的是一条**比率**（A 每 rad/s）而不是常数的原因，也是默认比率为零的原因： 一个常数不可能在两端都对。
 */
constexpr float kCeilingRad = 0.15F;
constexpr float kIqA = 0.5F;

auto psi_f_of(const sil::MachineSpec &s) -> float {
  return s.kt / (1.5F * static_cast<float>(s.pole_pairs));
}

/**
 * @brief [en] Angle error of the shipped observer on the nominal machine.
 * @brief [zh] 出厂观测器在标称机上的角度误差。
 */
auto error_at(float omega_e, float id_a) -> float {
  const sil::MachineSpec s = sil::nominal_spec();
  core::NonlinearFluxObserver observer(core::NonlinearFluxObserver::Config{
      1.05F * s.rs, s.lq, psi_f_of(s), 0.3F, 3.0F, 0.5F * psi_f_of(s)});
  observer.seed_flux(core::Ab<float>{psi_f_of(s), 0.0F});
  sil::SwitchingRigConfig config{};
  config.spec = s;
  config.omega_e = omega_e;
  config.iq_a = kIqA;
  config.id_a = id_a;
  return sil::run_switched(
      config, [&observer](sil::SwitchingRig &rig) {
        return observer.update(rig.command(), rig.current(),
                               sil::SwitchingRig::kControlPeriod);
      });
}

} // namespace

TEST(DaxisObserverLimit, TheZeroDaxisClaimHoldsOnlyAboveTheDocumentedFloor) {
  // Written before the table was measured, against the contract the README already states:
  // "< 50 rad/s (approx. 480 rpm): no angle claim". So the 0.15 rad ceiling is asserted from
  // 100 rad/s upward, and the absence of the claim is asserted below it -- both directions are
  // the finding, and asserting only the good half would quietly extend the claim.
  // 表量出来之前就写下的判据，对着 README 已经写下的合同："< 50 rad/s（约 480 rpm）不给角度主张"。
  // 所以 0.15 rad 天花板从 100 rad/s 往上断言，而在它之下断言**没有**这个主张——两个方向都是结论，
  // 只断言好看的那一半，等于悄悄把主张扩大。
  std::printf("\n[d-axis] single-L observer, nominal machine, iq = %.1f A\n",
              static_cast<double>(kIqA));
  std::printf("     %-12s", "omega rad/s");
  const float speeds[] = {20.0F, 50.0F, 100.0F, 200.0F, 400.0F, 800.0F, 1600.0F, 3200.0F};
  for (const float omega : speeds) {
    std::printf(" %10.0f", static_cast<double>(omega));
  }
  std::printf("\n");
  const float currents[] = {0.0F, -0.25F, -0.5F, -0.75F, -1.0F};
  for (const float id : currents) {
    std::printf("     i_d %-8.2f", static_cast<double>(id));
    for (const float omega : speeds) {
      std::printf(" %10.4f", static_cast<double>(error_at(omega, id)));
    }
    std::printf("\n");
  }
  for (const float omega : speeds) {
    if (omega >= 100.0F) {
      EXPECT_LE(error_at(omega, 0.0F), kCeilingRad)
          << "omega = " << omega << " rad/s, i_d = 0";
    } else {
      EXPECT_GT(error_at(omega, 0.0F), kCeilingRad)
          << "omega = " << omega
          << " rad/s: the README says no claim here, so the number must not look like one";
    }
  }
}

TEST(DaxisObserverLimit, OneAmpOfDaxisCurrentIsThreeTimesTheCeilingAtLowSpeed) {
  // The constraint exists, and it is a speed constraint: the same current is harmless at high
  // speed. Recorded so that "no d-axis current at all" cannot be mistaken for the finding.
  // 约束确实存在，而且是转速约束：同样的电流在高速无害。记下来，免得把结论记成"完全不许有
  // d 轴电流"。
  EXPECT_GT(error_at(200.0F, -1.0F), 2.0F * kCeilingRad);
  EXPECT_LT(error_at(3200.0F, -1.0F), kCeilingRad);
}

TEST(DaxisObserverLimit, TheSafeDaxisCurrentAtLowSpeedIsBelowWhatMtpaAsksFor) {
  // MTPA on a typical interior machine asks for a large fraction of the current limit. Measure the
  // largest `|i_d|` that still holds the ceiling at 200 rad/s; if that is a small number, then MTPA
  // and this observer are mutually exclusive at low speed and a library that allows both is wrong.
  // 典型内嵌机上 MTPA 会要掉电流限幅的一大部分。量出 200 rad/s 下仍守住天花板的 `|i_d|` 上限；
  // 如果它很小，那么低速下 MTPA 与这个观测器互斥，允许两者同时存在的库就是错的。
  float lo = 0.0F;
  float hi = 1.0F;
  for (int k = 0; k < 12; ++k) {
    const float mid = static_cast<float>(0.5 * (lo + hi));
    if (error_at(200.0F, -mid) <= kCeilingRad) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  std::printf("[d-axis] at 200 rad/s the ceiling holds up to |i_d| = %.4f A\n",
              static_cast<double>(lo));
  EXPECT_LT(lo, 0.5F);
}

TEST(DaxisObserverLimit, ThePolicyRefusesTheCurrentTheObserverCannotSee) {
  // The mechanism follows from the table. `MtpaReference` defaults to an allowance of zero, and
  // when a caller states a rate the allowance is `rate * |omega|`, so the same policy gives the
  // full curve at high speed and nothing at all at standstill. Writing this as an assertion is the
  // difference between "the user has to know" and "the library will not do it".
  // 机制由上表推出。`MtpaReference` 默认许可量为零；调用方给出比率时许可量是 `rate * |omega|`，
  // 于是同一个策略在高速给整条曲线、在堵转什么都不给。把它写成断言，就是"用户得知道"与
  // "库不让错"之间的区别。
  const sil::MachineSpec s = sil::nominal_spec();
  constexpr float ld = 2.0e-4F;
  constexpr float lq = 4.0e-4F;
  constexpr float i_max = 3.0F;
  constexpr float iq = 1.0F;
  const float psi_f = psi_f_of(s);

  core::MtpaReference unguarded(core::MtpaReference::Config{ld, lq, psi_f, i_max});
  core::ReferenceInput in{};
  in.iq_command_a = iq;
  in.omega_e_rad_s = 3200.0F;
  EXPECT_EQ(unguarded.i_d_limit_a(3200.0F), 0.0F);
  EXPECT_EQ(unguarded.generate(in).d, 0.0F)
      << "the default allowance is zero, so no d-axis current is applied at any speed";

  core::MtpaReference gated(
      core::MtpaReference::Config{ld, lq, psi_f, i_max, 1.0e-4F});
  core::ReferenceInput slow{};
  slow.iq_command_a = iq;
  slow.omega_e_rad_s = 200.0F;
  const float allowed_slow = gated.i_d_limit_a(200.0F);
  EXPECT_NEAR(allowed_slow, 0.02F, 1.0e-6F);
  EXPECT_GE(gated.generate(slow).d, -allowed_slow)
      << "the allowance is a boundary, not a suggestion";

  core::ReferenceInput fast{};
  fast.iq_command_a = iq;
  fast.omega_e_rad_s = 3200.0F;
  EXPECT_LT(gated.generate(fast).d, gated.generate(slow).d)
      << "the same policy must spend more d-axis current where the back-EMF carries the observer";
}

} // namespace foc
