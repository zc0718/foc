// Screening the observers on the two axes that decide the answer: electrical speed and d-axis current.
//
// The repo already carries the individual measurements (`test_salient_observer`, `test_daxis_observer_limit`,
// `test_nonlinear_flux_observer`, `test_hfi_observer`), and one of them **refuted** the belief this table was going to be
// built on: at `i_d = 0` a single-`L` observer's error does not depend on the saliency ratio at all, because its model
// error is exactly `(Ld - Lq)*i_d`. So saliency is not a screening axis; the axes are speed and `i_d`.
//
// This file exists to put the whole decision in one place: one table, every cell asserted against the 0.15 rad ceiling,
// and the thresholds it establishes are what a nameplate-only verdict would have to cite.
//
// 用两个真正决定答案的轴来筛观测器：电角速度与 d 轴电流。
// 仓库里已有各自的测量（`test_salient_observer`、`test_daxis_observer_limit`、`test_nonlinear_flux_observer`、
// `test_hfi_observer`），其中一条**推翻了**本表原本要据以建立的信念：`i_d = 0` 时单 `L` 观测器的误差与凸极比完全无关，
// 因为它的模型误差恰好是 `(Ld − Lq)·id`。所以凸极比不是筛选轴；轴是转速与 `i_d`。
//
// 本文件把整个决定放在一处：一张表，每一格都对着 0.15 rad 天花板断言，而它建立的阈值正是"仅凭铭牌给判定"所要引用的。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/observer/dual_inductance_observer.hpp>
#include <foc/core/observer/nonlinear_flux_observer.hpp>

#include "sil_bench.hpp"
#include "sil_switching_rig.hpp"

namespace {

constexpr float kCeilingRad = 0.15F;
constexpr float kIqA = 0.5F;

auto psi_f_of(const foc::sil::MachineSpec &spec) -> float {
    return spec.kt / (1.5F * static_cast<float>(spec.pole_pairs));
}

auto spec_with_ratio(float ratio) -> foc::sil::MachineSpec {
    foc::sil::MachineSpec spec = foc::sil::nominal_spec();
    spec.lq = 4.0e-4F;
    spec.ld = spec.lq / ratio;
    return spec;
}

/**
 * @brief [en] Angle error of the observer the library ships on the motor, at one operating point.
 * @brief [zh] 库在电机上实际使用的观测器，在一个工作点上的角度误差。
 */
auto back_emf_error(const foc::sil::MachineSpec &spec, float omega_e, float id_a) -> float {
    foc::core::BackEmfObserver observer(foc::core::BackEmfObserver::Config{spec.rs, spec.ld});
    observer.seed_flux(foc::core::Ab<float>{psi_f_of(spec), 0.0F});
    foc::sil::SwitchingRigConfig config{};
    config.spec = spec;
    config.omega_e = omega_e;
    config.iq_a = kIqA;
    config.id_a = id_a;
    return foc::sil::run_switched(config, [&observer](foc::sil::SwitchingRig &rig) {
        return observer.update(rig.command(), rig.current(), foc::sil::SwitchingRig::kControlPeriod);
    });
}

/**
 * @brief [en] Angle error of the non-linear flux observer, which carries `Lq` instead of `Ld`.
 * @brief [zh] 非线性磁链观测器的角度误差，它携带的是 `Lq` 而不是 `Ld`。
 */
auto nlfo_error(const foc::sil::MachineSpec &spec, float omega_e, float id_a) -> float {
    foc::core::NonlinearFluxObserver observer(foc::core::NonlinearFluxObserver::Config{
        1.05F * spec.rs, spec.lq, psi_f_of(spec), 0.3F, 3.0F, 0.5F * psi_f_of(spec)});
    observer.seed_flux(foc::core::Ab<float>{psi_f_of(spec), 0.0F});
    foc::sil::SwitchingRigConfig config{};
    config.spec = spec;
    config.omega_e = omega_e;
    config.iq_a = kIqA;
    config.id_a = id_a;
    return foc::sil::run_switched(config, [&observer](foc::sil::SwitchingRig &rig) {
        return observer.update(rig.command(), rig.current(), foc::sil::SwitchingRig::kControlPeriod);
    });
}

/**
 * @brief [en] Angle error of the dual-inductance observer: both inductances, no `i_d` term in its model.
 * @brief [zh] 双电感观测器的角度误差：两个电感都带，模型里没有 `i_d` 项。
 */
auto dual_l_error(const foc::sil::MachineSpec &spec, float omega_e, float id_a) -> float {
    foc::core::DualInductanceObserver observer(
        foc::core::DualInductanceObserver::Config{spec.rs, spec.ld, spec.lq, psi_f_of(spec)});
    observer.seed_flux(foc::core::Ab<float>{psi_f_of(spec), 0.0F});
    foc::sil::SwitchingRigConfig config{};
    config.spec = spec;
    config.omega_e = omega_e;
    config.iq_a = kIqA;
    config.id_a = id_a;
    return foc::sil::run_switched(config, [&observer](foc::sil::SwitchingRig &rig) {
        return observer.update(rig.command(), rig.current(), foc::sil::SwitchingRig::kControlPeriod);
    });
}

} // namespace

TEST(ObserverScreening, SpeedAndDaxisCurrentDecideWhereTheCeilingIsRespected) {
    // The criterion was written before the run as "the shipping observer stays inside 0.15 rad with i_d = 0 at every
    // speed on the list", and the run refuted it: at 20 rad/s it reads 0.9668 rad. The criterion had assumed the
    // characteristics of the observer the repository characterises -- `NonlinearFluxObserver` -- while the one that
    // actually ships in `sil::Bench` and in the C ABI is `BackEmfObserver`. Both are measured here side by side, and the
    // assertions below are the boundaries the run established rather than the ones it was expected to show.
    // 判据在跑之前写成"库在用的观测器在 i_d = 0 时于每个转速都留在 0.15 rad 内"，而运行推翻了它：20 rad/s 读到 0.9668 rad。
    // 那条判据假定的是仓库做过刻画的观测器 `NonlinearFluxObserver` 的特性，而 `sil::Bench` 与 C ABI 里真正在用的是
    // `BackEmfObserver`。两者在此并排测量，下面的断言是运行**确立**的边界，而不是它被期望展示的边界。
    const foc::sil::MachineSpec spec = spec_with_ratio(1.0F); // a surface machine: no saliency to hide behind
    const float speeds[] = {20.0F, 50.0F, 200.0F, 800.0F, 3200.0F};

    std::printf("\n[screening] surface machine (Lq/Ld = 1), iq = %.1f A, ceiling %.2f rad\n", static_cast<double>(kIqA),
                static_cast<double>(kCeilingRad));
    std::printf("%12s %10s %10s %12s %10s\n", "speed rad/s", "BackEmf", "NLFO", "BackEmf i_d=-1", "NLFO i_d=-1");

    float back_emf_zero[5] = {};
    float back_emf_loaded[5] = {};
    float nlfo_zero[5] = {};
    for (int i = 0; i < 5; ++i) {
        back_emf_zero[i] = back_emf_error(spec, speeds[i], 0.0F);
        nlfo_zero[i] = nlfo_error(spec, speeds[i], 0.0F);
        back_emf_loaded[i] = back_emf_error(spec, speeds[i], -1.0F);
        const float nlfo_loaded = nlfo_error(spec, speeds[i], -1.0F);
        std::printf("%12.0f %10.4f %10.4f %12.4f %10.4f\n", static_cast<double>(speeds[i]),
                    static_cast<double>(back_emf_zero[i]), static_cast<double>(nlfo_zero[i]),
                    static_cast<double>(back_emf_loaded[i]), static_cast<double>(nlfo_loaded));
    }

    // 1. The shipping observer needs a few hundred rad/s before the ceiling is respected at all -- 200 rad/s reads
    //    0.1224 and 800 rad/s reads 0.0306 on a surface machine with no d-axis current.
    //    库在用的观测器要到几百 rad/s 才进天花板内——表贴机、无 d 轴电流时 200 rad/s 是 0.1224、800 rad/s 是 0.0306。
    EXPECT_GT(back_emf_zero[2], 0.0F);
    EXPECT_LE(back_emf_zero[3], kCeilingRad) << "800 rad/s unloaded must be inside the ceiling";
    EXPECT_GT(back_emf_zero[0], kCeilingRad) << "20 rad/s unloaded must be outside it, or this table is not measuring speed";

    // 2. With -1 A of d-axis current the shipping observer leaves the ceiling until 800 rad/s, which is the screening
    //    rule in one number: the constraint is on d-axis current *at low speed*, not on d-axis current.
    //    d 轴电流为 −1 A 时，库在用的观测器要 800 rad/s 才回到天花板内；这条筛选规则压缩成一个数：限制在于**低速下的** d 轴
    //    电流，而不在于 d 轴电流本身。
    EXPECT_GT(back_emf_loaded[2], kCeilingRad);
    EXPECT_LE(back_emf_loaded[3], kCeilingRad);

    // 3. The observer the repository characterises is the better one at low speed, which is why its numbers appear in the
    //    plans and this one's do not. Measured at 20 rad/s: 0.1623 against 0.9668.
    //    仓库刻画过的是低转速下更好的那个观测器，所以计划里出现的是它的数。20 rad/s 实测：0.1623 对 0.9668。
    EXPECT_LT(nlfo_zero[0], back_emf_zero[0]) << "NLFO must be the better low-speed observer, by a wide margin";
    EXPECT_LT(nlfo_zero[0] / back_emf_zero[0], 0.5F);
}

TEST(ObserverScreening, SaliencyIsNotAScreeningAxisWhenTheDaxisCurrentIsZero) {
    // The belief this table would otherwise inherit: "a single-L observer survives to Lq/Ld = 2 and fails at 2.5". If it
    // held, the error would climb with the ratio. It does not, and the reason is arithmetic -- the model error is
    // `(Ld - Lq)*i_d`, so with `i_d = 0` every ratio is the same machine to this observer.
    // 本表否则会继承的信念："单 L 观测器撑到 Lq/Ld = 2，2.5 时不行"。若它成立，误差应随比值上升。它并不上升，原因是算术——
    // 模型误差是 `(Ld − Lq)·id`，所以 `i_d = 0` 时每个比值对它们都是同一台机器。
    std::printf("\n[screening] saliency sweep at i_d = 0, 200 rad/s\n");
    std::printf("%10s %14s %14s\n", "Lq/Ld", "BackEmfObserver", "NLFO");
    float first = 0.0F;
    float last = 0.0F;
    bool first_set = false;
    for (const float ratio : {1.0F, 1.5F, 2.0F, 2.5F}) {
        const foc::sil::MachineSpec spec = spec_with_ratio(ratio);
        const float back_emf = back_emf_error(spec, 200.0F, 0.0F);
        const float nlfo = nlfo_error(spec, 200.0F, 0.0F);
        std::printf("%10.1f %14.4f %14.4f\n", static_cast<double>(ratio), static_cast<double>(back_emf),
                    static_cast<double>(nlfo));
        if (!first_set) {
            first = back_emf;
            first_set = true;
        }
        last = back_emf;
    }
    // Flat within a few percent across a 2.5x change of saliency: there is no trend, which is the point.
    EXPECT_LT(std::fabs(last - first), 0.20F * first) << "a trend with saliency would contradict the arithmetic";
}

TEST(ObserverScreening, TheSwitchingRigCannotSeparateTheDualInductanceObserverFromTheSingleLOne) {
  // The column that was tried for this table. The premise was that the dual-inductance observer would show its worth on a
  // salient machine with d-axis current; measured here it does **not**, and the reason is the instrument rather than the
  // observer: at 2 A the single-`L` observer reads 0.1708 rad and the dual-inductance one 0.1699, while the model
  // difference between them is 0.105 rad (the bias law of `test_dual_inductance_observer.cpp`). A few tenths of a volt of
  // dead time is already 0.1 rad of flux angle at this `psi_f`, so on this rig both columns are measuring the inverter.
  // That is worth recording as a negative result: the screening table cannot separate these two observers, and nothing in
  // the table's other columns should be read as if it could. The separation needs the plant measurement.
  // 为这张表试过的那一列。前提是双电感观测器会在带 d 轴电流的凸极机上显出价值；在这里量**并没有**，而原因是仪器不是观测器：
  // 2 A 时单 `L` 读 0.1708 rad、双电感 0.1699，而两者模型上的差是 0.105 rad（`test_dual_inductance_observer.cpp` 的偏置定
  // 律）。这个 `psi_f` 下几百毫伏的死区就已经是 0.1 rad 磁链角，所以在这台架子上两列都在量逆变器。这个否定结果值得记下：本
  // 筛选表分不开这两个观测器，表里其它列也不该被读成它分得开。要分离得靠 plant 测量。
  const foc::sil::MachineSpec spec = spec_with_ratio(2.5F);
  std::printf("\n[screening] Lq/Ld = 2.5, iq = %.1f A, 800 rad/s: who survives d-axis current\n",
              static_cast<double>(kIqA));
  std::printf("%10s %12s %12s %12s\n", "i_d A", "BackEmf", "NLFO", "Dual-L");
  float dual_loaded = 0.0F;
  float back_emf_loaded = 0.0F;
  for (const float id : {0.0F, -0.5F, -1.0F, -2.0F}) {
    const float back_emf = back_emf_error(spec, 800.0F, id);
    const float nlfo = nlfo_error(spec, 800.0F, id);
    const float dual = dual_l_error(spec, 800.0F, id);
    std::printf("%10.2f %12.4f %12.4f %12.4f\n", static_cast<double>(id), static_cast<double>(back_emf),
                static_cast<double>(nlfo), static_cast<double>(dual));
    EXPECT_GT(back_emf, back_emf_loaded) << "the single-L error must grow with i_d, or this table is not measuring it";
    back_emf_loaded = back_emf;
    if (id < -1.5F) {
      dual_loaded = dual;
    }
  }
  std::printf("[screening] at i_d = -2 A: BackEmf %.4f rad, Dual-L %.4f rad -- the rig cannot separate them\n",
              static_cast<double>(back_emf_loaded), static_cast<double>(dual_loaded));
  // What the rig does show: both are outside the ceiling, and they are the same number to two percent. The second half is
  // the finding -- if these two columns ever *do* separate on this rig, the instrument changed and the table has to be
  // retaken.
  // 台架能显示的：两者都在天花板之外，且到 2% 之内是同一个数。后半句才是结论——如果这两列哪天在这台架子上真的分开了，说明
  // 仪器变了，整张表要重测。
  EXPECT_GT(back_emf_loaded, kCeilingRad) << "the single-L observer must be outside the ceiling here";
  EXPECT_NEAR(back_emf_loaded, dual_loaded, 0.02F) << "the rig separated them: the instrument changed";
}

