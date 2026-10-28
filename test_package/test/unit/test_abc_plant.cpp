// Conan::ImportStart
#include <gtest/gtest.h>

#include <cmath>
#include <cstdio>

#include <foc/core/transforms.hpp>

#include "sil_abc_plant.hpp"
#include "sil_bench.hpp"
// Conan::ImportEnd

namespace foc {

namespace {

namespace core = foc::core;
namespace sil = foc::sil;

/// [en] One operating point to check both plants at. [zh] 两台 plant
/// 共用的一个工作点。
struct Point final {
  float omega_e;
  float i_d;
  float i_q;
};

/// [en] The dq voltage the dq model says this point needs -- the *prediction
/// under test*.
///      [zh] dq 模型说这个点需要的 dq 电压——即**被测的预测**。
auto dq_voltage(const Point &p) -> core::Dq<float> {
  const sil::MachineSpec s = sil::nominal_spec();
  const float psi = s.kt / (1.5F * static_cast<float>(s.pole_pairs));
  return core::Dq<float>{(s.rs * p.i_d) - (p.omega_e * s.lq * p.i_q),
                         (s.rs * p.i_q) + (p.omega_e * s.ld * p.i_d) +
                             (p.omega_e * psi)};
}

auto abc_params() -> sil::AbcPlant::Params {
  const sil::MachineSpec s = sil::nominal_spec();
  return sil::AbcPlant::Params{s.rs, s.ld,
                               s.kt / (1.5F * static_cast<float>(s.pole_pairs)),
                               s.pole_pairs};
}

} // namespace

TEST(AbcPlantCrossCheck,
     ThePhaseVariableModelReproducesTheDqOperatingPoints) {
  // T2: two independently derived plants, six operating points. This is the
  // substitute for the motor-expert review this project does not have -- a sign
  // or a factor error in the dq derivation would make every SIL conclusion
  // wrong in the same direction while staying internally consistent, and
  // nothing inside the project could see it. The abc model never forms a dq
  // quantity. T2：两台独立推导的
  // plant，六个工作点。这是本项目没有的"电机专家评审"的替代——dq 推导里一个
  // 符号或系数错误会让所有 SIL 结论朝同一方向错且内部自洽，项目内部看不见。abc
  // 模型从不构造 dq 量。
  const Point points[6] = {{0.0F, 1.0F, 0.0F},   {0.0F, 0.0F, 1.0F},
                           {200.0F, 0.0F, 1.0F}, {200.0F, -1.0F, 1.0F},
                           {800.0F, 0.0F, 1.0F}, {800.0F, -1.0F, 1.0F}};
  std::printf("\n[T2] phase-variable plant vs dq plant\n");
  std::printf("        %8s %8s %8s | %10s %10s\n", "w_e", "i_d", "i_q",
              "torque abc", "torque dq");

  for (const Point &p : points) {
    // The abc plant is driven with the inverse-Park of the dq prediction, then
    // read back.
    sil::AbcPlant plant(abc_params());
    plant.set_state(0.0F, p.omega_e);
    const core::Dq<float> v_dq = dq_voltage(p);
    const float dt = 5.0e-6F;
    // Settle long enough for the electrical time constant; the frame is frozen,
    // so the angle only advances and no Park transform is used anywhere inside
    // the plant. 稳定到电气时间常数之后；坐标 frozen，角度自己前进，plant
    // 内部任何地方都不用 Park。
    const int periods = static_cast<int>((0.02F / dt) + 0.5F);
    for (int k = 0; k < periods; ++k) {
      plant.set_voltage(core::inv_clarke(
          core::inv_park(v_dq, plant.omega_e() * static_cast<float>(k) * dt)));
      plant.advance(dt);
    }
    // Recover the dq currents with Park, at the angle the plant finished on.
    // 用 Park 在 plant 结束时的角度上还原 dq 电流。
    const float theta_end = plant.omega_e() * static_cast<float>(periods) * dt;
    const core::Ab<float> ab = core::clarke(plant.current());
    const core::Dq<float> i_dq = core::park(ab, theta_end);

    // The dq model's torque at the *same* point, from the formula the library
    // uses. dq 模型在**同一个**点上的转矩，用库里那条公式。
    const sil::MachineSpec s = sil::nominal_spec();
    const float psi = s.kt / (1.5F * static_cast<float>(s.pole_pairs));
    const float torque_dq =
        1.5F * static_cast<float>(s.pole_pairs) * psi * i_dq.q;

    std::printf("        %8.0f %8.2f %8.2f | %10.6f %10.6f\n",
                static_cast<double>(p.omega_e), static_cast<double>(p.i_d),
                static_cast<double>(p.i_q), static_cast<double>(plant.torque()),
                static_cast<double>(torque_dq));

    // 1. The phase currents the abc plant settles on are the ones the dq
    // prediction asked for.
    //    A wrong sign in the dq voltage equations shows up here immediately.
    // 1. abc plant 稳定到的相电流就是 dq 预测要的那一对。dq
    // 电压方程里的符号错会立刻在这里现形。
    EXPECT_NEAR(p.i_d, i_dq.d, 0.02F + (0.02F * std::fabs(p.i_d)))
        << "omega " << p.omega_e;
    EXPECT_NEAR(p.i_q, i_dq.q, 0.02F + (0.02F * std::fabs(p.i_q)))
        << "omega " << p.omega_e;
    // 2. Torque by co-energy equals torque by `1.5*p*psi_f*i_q`, which checks
    // the 3/2 factor.
    // 2. 磁共能算的转矩等于 `1.5·p·ψf·i_q` 算的，这检查了 3/2 那个系数。
    EXPECT_NEAR(torque_dq, plant.torque(),
                0.02F * std::fabs(torque_dq) + 1.0e-5F)
        << "omega " << p.omega_e;
  }
}

TEST(AbcPlantCrossCheck, TheTwoModelsShareTheSameElectricalTimeConstant) {
  // Dynamics, not just the operating point: a d-axis step in each plant must
  // decay with the same `L/R`. If the dq derivation had the inductance in the
  // wrong place, steady state could still agree while the transient did not.
  // 动态，而不只是工作点：两台 plant 的 d 轴阶跃必须按同一个 `L/R` 衰减。如果
  // dq 推导把电感放错了 位置，稳态仍可能一致而暂态不一致。
  const sil::MachineSpec s = sil::nominal_spec();
  const float tau = s.ld / s.rs;

  // abc plant: step the phase voltage and watch |i| grow.
  // abc plant：阶跃相电压，看 |i| 怎么长。
  sil::AbcPlant abc(abc_params());
  abc.set_state(0.0F, 0.0F);
  const core::Dq<float> step{dq_voltage(Point{0.0F, 1.0F, 0.0F}).d, 0.0F};
  const float dt = 2.0e-6F;
  auto run_to = [&](float seconds) {
    const int steps = static_cast<int>((seconds / dt) + 0.5F);
    for (int k = 0; k < steps; ++k) {
      abc.set_voltage(core::inv_clarke(core::inv_park(step, 0.0F)));
      abc.advance(dt);
    }
    const core::Uvw<float> i = abc.current();
    return std::fabs(i.u);
  };
  const float at_half = run_to(0.5F * tau);
  const float at_full = run_to(0.5F * tau);

  // The closed form for a first-order rise.
  // 一阶上升的闭式。
  const float steady = step.d / s.rs;
  const float expected_half = steady * (1.0F - std::exp(-0.5F));
  const float expected_full = steady * (1.0F - std::exp(-1.0F));
  std::printf(
      "[T2] tau %.4f ms: |i| at 0.5tau %.5f A (expect %.5f), at 1.0tau %.5f A "
      "(expect %.5f)\n",
      static_cast<double>(1.0e3F * tau), static_cast<double>(at_half),
      static_cast<double>(expected_half), static_cast<double>(at_full),
      static_cast<double>(expected_full));

  EXPECT_NEAR(expected_half, at_half, 0.02F * expected_half);
  EXPECT_NEAR(expected_full, at_full, 0.02F * expected_full);
}

} // namespace foc
