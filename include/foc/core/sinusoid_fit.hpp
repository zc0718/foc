// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstddef>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] One tone measured out of a recorded window: amplitude, phase, and the status that says whether there was
 *             a tone to measure.
 *
 * @brief [zh] 从一段录制窗口里测出的一个单音：幅值、相位，以及"到底有没有音可测"的状态。
 *
 *     Two sine quantities and a status. `status` is the diagnostic channel and not decoration: `Ok` means the fit is
 *     conditioned, `State` means the window is shorter than one period of the frequency asked about (a fit over a
 *     fraction of a cycle is a number, but not the tone's amplitude), `Param` means the arguments cannot describe a
 *     sinusoid at all.
 *
 *     两个正弦量与一个状态。`status` 是诊断通道，不是装饰：`Ok` 表示拟合良态，`State` 表示窗口短于所问频率的一个周期（
 *     在不足一个周期的窗口上拟合也能出数，但那不是这个音的幅值），`Param` 表示参数根本不是一段正弦的描述。
 * @since 0.1.8
 * @exporter
 */
struct SinusoidFit {
    ErrorCode
        status; ///< [en] Ok / State (window shorter than a period) / Param. [zh] Ok / State（窗口不足一周期）/ Param。
    float
        amplitude; ///< [en] peak amplitude of the fitted tone, same unit as the samples. [zh] 拟合出的音峰值，与样本同单位。
    float phase_rad; ///< [en] phase of `+A*cos(w*t + phase)`, rad. [zh] `+A*cos(w*t + phase)` 的相位，rad。
};


/**
 * @brief [en] Least-squares fit of one sinusoid at a known frequency to a recorded window.
 *
 * @brief [zh] 在已知频率上，对一段录制窗口做单音最小二乘拟合。
 *
 *     Why this exists. The step-response bandwidth in `step_metrics.hpp` is an *estimate*: it fits a first-order model
 *     to a response that is not quite first-order, and it carries a bias of the same size as the defects it is meant to
 *     catch (measured: a 20 % error in the speed loop's gain law moved it by 9.9 %, inside its own 10 % band). The
 *     frequency-domain magnitude at a known frequency is not an estimate of that kind: it is defined for any linear
 *     system, needs no model, and a 20 % gain error moves it by about 20 %. That is what makes a sweep the instrument
 *     for scale errors, and this function is the sweep's only moving part.
 *
 *     Exists so the loop can measure its own frequency response without a spectrum analyser: drive the reference with
 *     `A*cos(w*t)`, record the response, fit both, divide. The arithmetic is a projection onto `{1, cos, sin}`, solved
 *     as the normal equations -- exact for a window that contains whole cycles and unbiased for one that does not,
 *     which a plain DFT bin would not be.
 *
 *     为什么有这个函数。`step_metrics.hpp` 里的阶跃带宽是一个**估计**：它把一阶模型套在一个并不完全是一阶的响应上，其偏
 *     差与被测缺陷同样大小（实测：速度环增益律 20% 的错误只让它挪动 9.9%，仍在自己 10% 的带内）。而在已知频率上的频域幅
 *     值不是这类估计：它对任何线性系统都有定义、不需要模型，20% 的增益错误就让它挪动约 20%。这正是扫频成为"比例错误"之
 *     仪器的原因，而本函数是扫频唯一的运动部件。
 *
 *     存在的意义是让环路不需要频谱仪就能测自己的频率响应：用 `A*cos(w*t)` 驱动给定，录下响应，两边都拟合，相除。算术是
 *     向 `{1, cos, sin}` 的投影，用正规方程求解——对整周期窗口精确，对非整周期窗口无偏，而单纯取 DFT 的一个 bin 做不到后
 *     者。
 *
 * @param y [en] response samples, oldest first, SI units.
 * @param y [zh] 响应样本，最早的在前，SI 单位。
 * @param n [en] sample count; at least six.
 * @param n [zh] 样本数；至少六个。
 * @param dt_s [en] spacing between samples, s (> 0).
 * @param dt_s [zh] 相邻样本间隔，s（> 0）。
 * @param omega_rad_s [en] the frequency asked about, rad/s (> 0, and below Nyquist: `omega * dt < pi`).
 * @param omega_rad_s [zh] 所问的角频率，rad/s（> 0，且低于 Nyquist：`omega * dt < pi`）。
 * @return [en] the fit; `amplitude` and `phase_rad` are written even when the status is not `Ok`.
 * @return [zh] 拟合结果；即使状态不是 `Ok`，`amplitude` 与 `phase_rad` 也会被写入。
 * @context [en] Host-side analysis over a recorded window: noexcept, no allocation, O(n), no function pointer.
 * @context [zh] 对录制窗口的主机侧分析：noexcept、不分配、O(n)、无函数指针。
 * @since 0.1.8
 * @exporter
 */
inline auto sinusoid_fit(const float* y, std::size_t n, float dt_s, float omega_rad_s) noexcept -> SinusoidFit {
    SinusoidFit out{};
    out.status = ErrorCode::Param;
    out.amplitude = 0.0F;
    out.phase_rad = 0.0F;

    if ((y == nullptr) || (n < 6U) || !(dt_s > 0.0F) || !(omega_rad_s > 0.0F)) {
        return out;
    }
    const double omega = static_cast<double>(omega_rad_s);
    const double dt = static_cast<double>(dt_s);
    // Above Nyquist the projection aliases: a tone would be reported at a frequency it does not have.
    // 越过 Nyquist 后投影会混叠：一个音会被报到它并不具有的频率上。
    if (!((omega * dt) < 3.14159265358979323846)) {
        return out;
    }
    // A window shorter than one period cannot carry the tone's amplitude -- that is `State`, not a number.
    // 短于一个周期的窗口承载不了这个音的幅值——这是 `State`，不是一个数。
    if ((static_cast<double>(n) * dt) < (6.28318530717958647692 / omega)) {
        out.status = ErrorCode::State;
        return out;
    }

    // Normal equations for y ~ c0 + c1*cos + c2*sin. Accumulated in double: a 2000-sample window at 10 kHz loses the
    // basis products to float rounding long before the sine does.
    // y ~ c0 + c1*cos + c2*sin 的正规方程。用 double 累加：10 kHz 下 2000 个样本的窗口，基函数的乘积早于正弦本身
    // 就在 float 舍入里丢了。
    double m[3][4] = {{0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0}};
    for (std::size_t i = 0U; i < n; ++i) {
        const double t = static_cast<double>(i) * dt;
        const double c = std::cos(omega * t);
        const double s = std::sin(omega * t);
        const double basis[3] = {1.0, c, s};
        const double value = static_cast<double>(y[i]);
        for (std::size_t row = 0U; row < 3U; ++row) {
            for (std::size_t col = 0U; col < 3U; ++col) {
                m[row][col] += basis[row] * basis[col];
            }
            m[row][3] += basis[row] * value;
        }
    }

    // Gaussian elimination with partial pivoting; a singular system means the window cannot separate the basis.
    // 带主元选取的高斯消元；奇异系统意味着这个窗口分不开这组基。
    for (std::size_t col = 0U; col < 3U; ++col) {
        std::size_t pivot = col;
        for (std::size_t row = col + 1U; row < 3U; ++row) {
            const double a = std::fabs(m[row][col]);
            const double b = std::fabs(m[pivot][col]);
            if (a > b) {
                pivot = row;
            }
        }
        if (!(std::fabs(m[pivot][col]) > 1.0e-12)) {
            out.status = ErrorCode::State;
            return out;
        }
        if (pivot != col) {
            for (std::size_t k = 0U; k < 4U; ++k) {
                const double swap = m[col][k];
                m[col][k] = m[pivot][k];
                m[pivot][k] = swap;
            }
        }
        for (std::size_t row = col + 1U; row < 3U; ++row) {
            const double factor = m[row][col] / m[col][col];
            for (std::size_t k = col; k < 4U; ++k) {
                m[row][k] -= factor * m[col][k];
            }
        }
    }
    double c[3] = {0.0, 0.0, 0.0};
    for (std::size_t back = 3U; back > 0U; --back) {
        const std::size_t row = back - 1U;
        double acc = m[row][3];
        for (std::size_t k = row + 1U; k < 3U; ++k) {
            acc -= m[row][k] * c[k];
        }
        c[row] = acc / m[row][row];
    }

    // y = A*cos(w t + phi) = (A cos phi) cos + (-A sin phi) sin.
    // y = A*cos(w t + phi) = (A cos phi) cos + (-A sin phi) sin。
    out.amplitude = static_cast<float>(std::hypot(c[1], c[2]));
    out.phase_rad = static_cast<float>(std::atan2(-c[2], c[1]));
    out.status = ErrorCode::Ok;
    return out;
}


} // namespace core

} // namespace foc
