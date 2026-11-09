// Conan::ImportStart
#pragma once
// Conan::ImportEnd

// One copy of the two measurements a procedure-type capability is judged by, so the SOP driver and the `MotorConfig`
// sensitivity row cannot drift apart. Both were written for the F35 row in `test_motor.cpp`; they live here now because
// a *procedure* has to be replayable on a second machine, and two implementations of "the bandwidth this loop actually
// delivered" would make that replay a comparison of two measurements rather than of two machines.
// 流程型能力判据所需的两项测量只此一份，好让 SOP 驱动与 `MotorConfig` 敏感度那一行不会各走一条路。两者原本是为
// `test_motor.cpp` 的 F35 那行写的；搬到这里是因为 **流程** 必须能在第二台机器上重放，而"这个环实际交付的带宽"有两份实现，
// 那次重放就成了两个测量的比较而不是两台机器的比较。

#include <cmath>
#include <cstdint>

#include <foc/core/pi.hpp>
#include <foc/motor/motor_config.hpp>

namespace foc {

namespace sil {

/**
 * @brief [en] The rate the tuning is used at: 20 kHz. @brief
 * @brief [zh] 整定被使用的速率：20 kHz。
 */
constexpr float kSopGainDt = 5e-5F;

/**
 * @brief [en] Amplitude gain of the sampled speed loop at frequency `f`, with an ideal current loop.
 * @brief [zh] 采样速度环在频率 `f` 处的幅值增益，电流环视为理想。
 * @param bandwidth_rad_s [en] the bandwidth the gains were designed for.
 * @param bandwidth_rad_s [zh] 增益所按设计的那条带宽。
 * @param j_true [en] the machine's inertia.
 * @param j_true [zh] 机器真实的惯量。
 * @param b_true [en] the machine's viscous coefficient.
 * @param b_true [zh] 机器真实的黏性系数。
 * @param j_hat [en] the inertia the gains were computed from.
 * @param j_hat [zh] 计算增益所用的惯量。
 * @param b_hat [en] the viscous coefficient the gains were computed from.
 * @param b_hat [zh] 计算增益所用的黏性系数。
 * @param kt [en] torque constant.
 * @param kt [zh] 转矩常数。
 * @param f_hz [en] measurement frequency, Hz.
 * @param f_hz [zh] 测量频率，Hz。
 * @param with_ki [en] false to keep only the proportional term.
 * @param with_ki [zh] 为假时只保留比例项。
 * @return [en] `rms(omega) / 0.7071`, i.e. the gain against a unit sine.
 * @return [zh] `rms(ω)/0.7071`，即对单位正弦的增益。
 * @note [en] `j_hat`/`b_hat` go into the gains and `j_true`/`b_true` into the machine, which is the whole point: a
 *            nameplate error is exactly the difference between those two pairs. The current loop is skipped (torque =
 *            kt*i in the same period), so whatever this measures belongs to the **speed** law and not to a cascade.
 * @note [zh] `j_hat`/`b_hat` 进增益、`j_true`/`b_true` 进机器，这正是要点：铭牌误差就是这两对参数之差。电流环被跳过（力
 *            矩在同一周期内就是 kt·i），所以这里量到的一切属于 **速度** 律而不属于任何级联。
 * @since 0.1.9
 */
inline auto speed_loop_gain(float bandwidth_rad_s, float j_true, float b_true, float j_hat, float b_hat, float kt,
                            double f_hz, bool with_ki) -> double {
    foc::motor::MotorConfig config{0.5F, 4.0e-4F, 8.0e-4F, kt, j_hat, b_hat, 4U, 3.0F, 20000.0F, 500.0F};
    foc::core::Pi::Config pi_config = foc::speed_pi_config(config, bandwidth_rad_s);
    if (!with_ki) {
        pi_config.ki = 0.0F;
    }
    foc::core::Pi pi(pi_config);
    const auto settle = static_cast<int>((40.0 / f_hz) / kSopGainDt);
    const auto measure = static_cast<int>((20.0 / f_hz) / kSopGainDt);
    float omega = 0.0F;
    double sum_squares = 0.0;
    int counted = 0;
    for (int k = 0; k < (settle + measure); ++k) {
        const auto t = static_cast<float>(k) * kSopGainDt;
        const auto command = static_cast<float>(std::sin(f_hz * static_cast<double>(t)));
        const float current = pi.step(command, omega, kSopGainDt);
        omega += (((kt * current) - (b_true * omega)) / j_true) * kSopGainDt;
        if (k >= settle) {
            sum_squares += static_cast<double>(omega) * omega;
            ++counted;
        }
    }
    const double rms_out = std::sqrt(sum_squares / static_cast<double>(counted));
    return rms_out / 0.7071067811865476; // the command is a unit sine 激励是单位正弦
}

/**
 * @brief [en] The frequency where the loop's amplitude gain crosses `1/sqrt(2)`, as a multiple of the requested
 *             bandwidth.
 * @brief [zh] 环路幅值增益越过 `1/√2` 的频率，以请求带宽的倍数表示。
 * @return [en] measured −3 dB / requested.
 * @return [zh] 实测 −3 dB / 请求值。
 * @note [en] Bisection, not a sweep: the gain is monotone in frequency, so the crossing can be located to any precision
 *            and a case does not have to argue about which grid point was closest.
 * @note [zh] 二分而不是扫描：增益对频率单调，所以穿越点能定位到任意精度，用例不必为"哪个栅格点最近"争论。
 * @since 0.1.9
 */
inline auto minus3db_over_requested(float bandwidth_rad_s, float j_true, float b_true, float j_hat, float b_hat,
                                    float kt) -> double {
    const double target = 0.7071067811865476;
    double low = 0.05 * static_cast<double>(bandwidth_rad_s);
    double high = 8.0 * static_cast<double>(bandwidth_rad_s);
    for (int i = 0; i < 40; ++i) {
        const double mid = std::sqrt(low * high);
        if (speed_loop_gain(bandwidth_rad_s, j_true, b_true, j_hat, b_hat, kt, mid, true) >= target) {
            low = mid;
        } else {
            high = mid;
        }
    }
    return std::sqrt(low * high) / static_cast<double>(bandwidth_rad_s);
}

} // namespace sil

} // namespace foc
