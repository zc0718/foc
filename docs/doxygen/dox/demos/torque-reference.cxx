// Torque -- torque below base speed: the reference policy, the two PIs, the
// feedforward, and where the voltage ceiling belongs. No hardware, no plant:
// the point of the example is the wiring a board shares with the SIL bench.
//
// Build it the way the `demo` gate does -- the example calls into the library, so the
// sources are part of the command, and `-Iinclude` alone fails at the link. A renamed
// source turns that gate red instead of leaving a stale instruction here:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude
//   docs/doxygen/dox/demos/torque-reference.cxx src/core/*.cpp
//   src/core/observer/*.cpp src/motor/*.cpp src/driver/*.cpp src/api/foc_api.c
//   src/api/bridge.cpp src/version.c
//   -o /tmp/torque-reference
//
// 转矩 —— 基速以下转矩：参考策略、两个 PI、前馈，以及电压天花板该放在哪。
// 不需要硬件也不需要被控对象：这个例子要展示的是"板子与 SIL
// 台架共用的那段接线"。

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <foc/core/feedforward.hpp>
#include <foc/core/limits.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/reference.hpp>
#include <foc/core/types.hpp>
#include <foc/motor/motor_config.hpp>

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

// The nameplate, once. Six of these fields never reach the current loop -- they
// are here because `current_pi_config()` takes the configuration a caller already
// has, instead of the two numbers it happens to read.
// 铭牌，只写一次。其中六个字段根本进不到电流环——它们在这里是因为
// `current_pi_config()` 要的是调用方本来就有的那份配置，而不是它恰好读的那两个数。
constexpr foc::motor::MotorConfig kMachine{
    kRs, kLd, kLq, 0.08F, 1.0e-4F, 1.0e-4F, std::uint8_t{4}, kIMaxA, 20.0e3F,
    200.0F};

// A current loop closed at 1 kHz. The gains come from the library's own rule
// (`kp = 2*wb*Ld - Rs`, `ki = wb^2*Ld`), not retyped here: the equation table
// in the documentation carries that rule with a pointer to this function, and a
// second copy in an example is a second law that can drift from it.
// 1 kHz 闭环的电流环。增益来自库自己的规则（`kp = 2*wb*Ld - Rs`、`ki = wb^2*Ld`），
// 不在这里重抄：文档里的方程表带着指向这个函数的指针，而示例里的第二份就是一条
// 会与它走散的第二定律。
constexpr float kLoopOmega = kTwoPi * 1000.0F;

// The ceiling is a **vector** ceiling: `v_bus/sqrt(3)`, the inscribed circle of
// the hexagon, which is where `svpwm.hpp` reports `saturated`. It is not
// `v_bus/2` -- that is the per-axis clamp inside `Pi::Config`, a different
// object, and a pair of axes both sitting on it asks for `0.707*v_bus`, which
// the bridge cannot deliver linearly.
// 天花板是**矢量**天花板：`v_bus/sqrt(3)`，六边形的内切圆，也就是 `svpwm.hpp`
// 报 `saturated` 的位置。不是 `v_bus/2`——那是 `Pi::Config` 里的每轴钳位，是两个
// 不同的对象；两轴同时贴着它，合成矢量就是 `0.707*v_bus`，桥臂无法线性交付。
constexpr float kVCeilingV = kVBusV / 1.7320508F;

/// One current-loop tick on one axis: PI plus that axis's feedforward term, with
/// no ceiling applied here -- the ceiling belongs to the pair, see
/// `limit_vector()`.
/// 一个电流环周期上的单轴：PI 加该轴的前馈项，**不**在这里钳位——天花板属于那对
/// 数，见 `limit_vector()`。
auto tick(foc::core::Pi &pi, float reference, float measured, float ff) noexcept
    -> float {
  return pi.step(reference, measured, kDtS) + ff;
}

/// Limit the **pair**, not each axis. Clamping them separately keeps each inside
/// its own bound and still lets the vector out; scaling both keeps the angle the
/// loop asked for, which is the one thing the modulator cannot restore.
/// 限的是**那对数**，不是每个轴。分开钳位只能保证各自不越界，合成矢量仍然会出去；
/// 两边同比例缩，才不会改变环路要的角度——而角度是调制器无法恢复的东西。
auto limit_vector(foc::core::Dq<float> v, float ceiling) noexcept
    -> foc::core::Dq<float> {
  const float magnitude = std::sqrt((v.d * v.d) + (v.q * v.q));
  // `ceiling` is a positive constant, so a zero vector takes the early return and
  // the division below never divides by zero.
  if (magnitude <= ceiling) {
    return v;
  }
  const float scale = ceiling / magnitude;
  return foc::core::Dq<float>{v.d * scale, v.q * scale};
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

  // One law, one place. `current_pi_config()` returns the gains *and* the
  // per-axis output clamp, which is why nothing here recomputes either.
  // 一条定律，一个位置。`current_pi_config()` 同时返回增益与**每轴**输出钳位，
  // 所以这里两边都不重算。
  const foc::core::Pi::Config gains =
      foc::current_pi_config(kMachine, kLoopOmega, kVBusV);
  foc::core::Pi pi_d(gains);
  foc::core::Pi pi_q(gains);
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
  const float v_d_raw = tick(pi_d, i_ref.d, measured.d, v_ff.d);
  const float v_q_raw = tick(pi_q, i_ref.q, measured.q, v_ff.q);
  const foc::core::Dq<float> v =
      limit_vector(foc::core::Dq<float>{v_d_raw, v_q_raw}, kVCeilingV);
  std::printf(
      "\none tick at iq = %.2f A, omega_e = %.1f rad/s, v_bus = %.1f V\n",
      static_cast<double>(top.iq_command_a), static_cast<double>(omega_e),
      static_cast<double>(kVBusV));
  std::printf("  reference   id = %+8.4f A  iq = %+8.4f A\n",
              static_cast<double>(i_ref.d), static_cast<double>(i_ref.q));
  std::printf("  feedforward vd = %+8.4f V  vq = %+8.4f V\n",
              static_cast<double>(v_ff.d), static_cast<double>(v_ff.q));
  std::printf(
      "  asked       vd = %+8.4f V  vq = %+8.4f V  (|v| = %.4f V)\n",
      static_cast<double>(v_d_raw), static_cast<double>(v_q_raw),
      static_cast<double>(std::sqrt((v_d_raw * v_d_raw) + (v_q_raw * v_q_raw))));
  std::printf(
      "  commanded   vd = %+8.4f V  vq = %+8.4f V  (|v| = %.4f V, ceiling "
      "%.4f V)\n",
      static_cast<double>(v.d), static_cast<double>(v.q),
      static_cast<double>(std::sqrt((v.d * v.d) + (v.q * v.q))),
      static_cast<double>(kVCeilingV));
  std::printf("  status      d = %d  q = %d  (0 = Ok)\n",
              static_cast<int>(pi_d.status()), static_cast<int>(pi_q.status()));

  // The limiter, shown on the pair that needs it. Two axes each sitting on the
  // per-axis clamp `current_pi_config()` puts in `Pi::Config` ask for
  // `v_bus/2*sqrt(2)`, which is past the inscribed circle -- and the scaled pair
  // keeps the angle, which is the whole reason for scaling rather than clamping.
  // 限幅器，用真正需要它的那对数展示。两轴各贴着 `current_pi_config()` 写进
  // `Pi::Config` 的每轴钳位，合成矢量就是 `v_bus/2*sqrt(2)`，已经出了内切圆——
  // 而按比例缩放后的那对数保住了角度，这就是"缩放而不是钳位"的全部理由。
  const float per_axis = 0.5F * kVBusV;
  const foc::core::Dq<float> over{per_axis, per_axis};
  const foc::core::Dq<float> held = limit_vector(over, kVCeilingV);
  std::printf(
      "\nthe vector ceiling, on a pair that reaches it:\n  asked       |v| = "
      "%.4f V  angle = %+.1f deg\n  commanded   |v| = %.4f V  angle = %+.1f "
      "deg\n",
      static_cast<double>(std::sqrt((over.d * over.d) + (over.q * over.q))),
      static_cast<double>(std::atan2(over.q, over.d) * 57.29577951308232),
      static_cast<double>(std::sqrt((held.d * held.d) + (held.q * held.q))),
      static_cast<double>(std::atan2(held.q, held.d) * 57.29577951308232));
  return 0;
}
