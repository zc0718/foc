// Torque -- torque below base speed: the reference policy, the two PIs, the
// feedforward, and where the voltage ceiling belongs. No hardware, no plant:
// the point of the example is the wiring a board shares with the SIL bench.
//
// Build with the flags the `strict` gate uses for a C++17 translation unit (one
// line, because a trailing backslash inside a `//` comment is a warning):
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude
//   docs/doxygen/dox/demos/torque-reference.cxx -o /tmp/torque-reference
//
// 转矩 —— 基速以下转矩：参考策略、两个 PI、前馈，以及电压天花板该放在哪。
// 不需要硬件也不需要被控对象：这个例子要展示的是"板子与 SIL
// 台架共用的那段接线"。

#include <cmath>
#include <cstdio>

#include <foc/core/feedforward.hpp>
#include <foc/core/limits.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/reference.hpp>
#include <foc/core/types.hpp>

namespace {

constexpr float kTwoPi = 6.28318530718F;

// One salient machine, named once. `lq = 2 * ld` is the shape MTPA exists for:
// on a surface machine the d axis earns nothing and the two policies coincide.
// 一台凸极机，只在这里写一次。`lq = 2 * ld` 正是 MTPA 存在的形状：表贴机上 d 轴
// 没有额外收益，两种策略给出同一个答案。
constexpr float kLd = 4.0e-4F;
constexpr float kLq = 8.0e-4F;
constexpr float kPsiF = 0.01F;
constexpr float kIMaxA = 3.0F;

constexpr float kRs = 0.5F;
constexpr float kVBusV = 24.0F;
constexpr float kDtS = 5.0e-5F; // 20 kHz current loop

// A current loop closed at 1 kHz. The gains are `L * omega` and `R * omega`,
// the pole-zero cancellation every board uses, written here so the example does
// not depend on a tuning helper the reader has not read yet. 1 kHz
// 闭环的电流环。增益是 `L * omega` 与 `R * omega`，即各处都在用的零极点对消；
// 写出来是为了让例子不依赖读者还没读过的整定辅助函数。
constexpr float kLoopOmega = kTwoPi * 1000.0F;

/// One current-loop tick on one axis: PI plus the axis's feedforward term.
/// 一个电流环周期上的单轴：PI 加该轴的前馈项。
auto tick(foc::core::Pi &pi, float reference, float measured, float ff,
          float ceiling) noexcept -> float {
  return foc::core::clamp(pi.step(reference, measured, kDtS) + ff, -ceiling,
                          ceiling);
}

} // namespace

auto main() -> int {
  // Two instances of the same policy, and the difference between them is the
  // whole point of this example: `i_d_per_speed_a_s_per_rad` defaults to 0,
  // which *refuses* d-axis current outright, because a d-axis current is only
  // safe if somebody has said how the angle is measured. Left at 0, MTPA
  // answers exactly what the d=0 policy answers.
  // 同一个策略的两个实例，它们的差别就是本片的要点：`i_d_per_speed_a_s_per_rad`
  // 默认是 0，即**完全不施加** d 轴电流——因为只有在"角度怎么测"有交代之后，
  // d 轴电流才是安全的。留在 0，MTPA 给出的答案与 d=0 策略完全相同。
  const foc::core::MtpaReference::Config shipped{kLd, kLq, kPsiF, kIMaxA};
  // A large allowance is safe only for a machine whose angle is measured (a
  // position sensor): the rate exists to bound how much a *wrong* angle can
  // cost, so with a sensor the bound is not needed.
  // 大的许可值只对"角度是被测出来的"机器才是安全的（带位置传感器）：这个比率
  // 存在是为了限制**错的**角度能造成多大代价，有传感器时那道界不需要。
  const foc::core::MtpaReference::Config with_allowance{kLd, kLq, kPsiF, kIMaxA,
                                                        1.0F};
  foc::core::MtpaReference mtpa_shipped(shipped);
  foc::core::MtpaReference mtpa(with_allowance);
  foc::core::NullReferenceGenerator d_zero{};

  const float v_ceiling = 0.5F * kVBusV; // the SVPWM linear range, not the rail
  const float kp = kLd * kLoopOmega;
  const float ki = kRs * kLoopOmega;
  foc::core::Pi pi_d(foc::core::Pi::Config{kp, ki, -v_ceiling, v_ceiling,
                                           -v_ceiling, v_ceiling});
  foc::core::Pi pi_q(foc::core::Pi::Config{kp, ki, -v_ceiling, v_ceiling,
                                           -v_ceiling, v_ceiling});
  foc::core::DecouplingFeedforward feedforward(
      foc::core::DecouplingFeedforward::Config{
          kLd, kLq, kPsiF,
          foc::core::DecouplingFeedforward::Terms::CrossAndBemf});

  // A standstill measurement: the loop is asked to move the current away from
  // zero, which is the case where the integral term does all the work.
  // 静止时的实测值：让环路把电流从零推出去，这正是积分项承担全部工作的情况。
  const foc::core::Dq<float> measured{0.0F, 0.0F};
  const float omega_e = 20.0F;

  std::printf("%7s | %12s %8s | %12s %8s | %8s %8s\n", "iq cmd", "mtpa(0) id",
              "iq", "mtpa(allow) id", "iq", "d=0 id", "iq");
  for (float iq_command = 0.5F; iq_command <= kIMaxA + 0.01F;
       iq_command += 0.5F) {
    const foc::core::ReferenceInput in{iq_command, omega_e, kVBusV, measured};
    const foc::core::Dq<float> as_shipped = mtpa_shipped.generate(in);
    const foc::core::Dq<float> allowed = mtpa.generate(in);
    const foc::core::Dq<float> with_d_zero = d_zero.generate(in);
    std::printf(
        "%7.2f | %12.4f %8.4f | %12.4f %8.4f | %8.4f %8.4f\n",
        static_cast<double>(iq_command), static_cast<double>(as_shipped.d),
        static_cast<double>(as_shipped.q), static_cast<double>(allowed.d),
        static_cast<double>(allowed.q), static_cast<double>(with_d_zero.d),
        static_cast<double>(with_d_zero.q));
  }

  // One tick at the top of the sweep, to show the voltage the two axes ask for
  // and that the ceiling is applied after the feedforward, not inside the PI.
  // 走到扫描顶端的那一拍，用来展示两轴要的电压，以及天花板夹在前馈**之后**、
  // 不是夹在 PI 内部。
  const foc::core::ReferenceInput top{kIMaxA, omega_e, kVBusV, measured};
  const foc::core::Dq<float> i_ref = mtpa.generate(top);
  const foc::core::Dq<float> v_ff =
      feedforward.generate(foc::core::FeedforwardInput{omega_e, measured});
  const float v_d = tick(pi_d, i_ref.d, measured.d, v_ff.d, v_ceiling);
  const float v_q = tick(pi_q, i_ref.q, measured.q, v_ff.q, v_ceiling);
  std::printf(
      "\none tick at iq = %.2f A, omega_e = %.1f rad/s, v_bus = %.1f V\n",
      static_cast<double>(top.iq_command_a), static_cast<double>(omega_e),
      static_cast<double>(kVBusV));
  std::printf("  reference   id = %+8.4f A  iq = %+8.4f A\n",
              static_cast<double>(i_ref.d), static_cast<double>(i_ref.q));
  std::printf("  feedforward vd = %+8.4f V  vq = %+8.4f V\n",
              static_cast<double>(v_ff.d), static_cast<double>(v_ff.q));
  std::printf("  command     vd = %+8.4f V  vq = %+8.4f V  (ceiling %.1f V)\n",
              static_cast<double>(v_d), static_cast<double>(v_q),
              static_cast<double>(v_ceiling));
  std::printf("  status      d = %d  q = %d  (0 = Ok)\n",
              static_cast<int>(pi_d.status()), static_cast<int>(pi_q.status()));
  return 0;
}
