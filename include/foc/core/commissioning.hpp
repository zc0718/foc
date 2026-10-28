// Conan::ImportStart
#pragma once
#include <cmath>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] The dead-time voltage an **axis** sees, from the per-phase step and the angle of the current in the rotor
 *             frame.
 *
 *     Why this is not the per-phase number. The inverter takes `deadtime_v * sign(i_phase)` from each phase, and
 *     projecting three of those onto an axis depends on where the current is: with the current on the d axis the phases
 *     carry `(+i, -i/2, -i/2)`, and the projection is `(2/3)*(1 + 1/2 + 1/2)*deadtime_v = (4/3)*deadtime_v`. With the
 *     current on the q axis they carry `(0, +i*sqrt(3)/2, -i*sqrt(3)/2)`: one phase carries nothing, and the projection
 *     is `(2/3)*(sqrt(3)/2 + sqrt(3)/2) = 2/sqrt(3)` times -- 0.554 V instead of 0.64 V at 24 V. That 13 % difference
 *     is not cosmetic: using the d-axis number on a q-axis step leaves `Lq` reading 16 % low (SIL the two-point sweep).
 *
 * @brief [zh] 由每相阶跃和电流在转子系里的角度，算出**一个轴**看到的死区电压。
 *
 *     为什么这不是每相的数值。逆变器从每相拿走 `deadtime_v·sign(i_phase)`， 三个这样的量投影到一个轴上，取决于电流在哪
 *     里：电流在 d 轴上时相电流是 `(+i, −i/2, −i/2)`，投影是 `(2/3)·(1 + 1/2 + 1/2)·deadtime_v = (4/3)·deadtime_v`； 电
 *     流在 q 轴上时是 `(0, +i·√3/2, −i·√3/2)`：有一相根本不带电流，投影是 `(2/3)·(√3/2 + √3/2) = 2/√3` 倍——24 V 下是
 *     0.554 V 而不是 0.64 V。这 13 % 不是 无关紧要：在 q 轴阶跃上用 d 轴的数，会让 `Lq` 读低 16 %（两点扫描实测）。
 *
 * @param deadtime_v [en] (t_dead/T_pwm)*v_bus per phase, V.
 * @param deadtime_v [zh] (t_dead/T_pwm)*v_bus，每相，V。
 * @param angle_rad [en] the current's angle from the d axis, rad; 0 = d, pi/2 = q.
 * @param angle_rad [zh] 电流相对 d 轴的角度，rad；0 = d，π/2 = q。
 * @return [en] the axis-projected loss, V.
 * @return [zh] 投影到轴上的损失，V。
 * @note [en] A phase whose current is exactly zero contributes nothing -- `sign(0) == 0` is the physical choice, not a
 *            convenience.
 * @note [zh] 电流正好为零的那一相不贡献任何东西——`sign(0) == 0` 是物理选择，不是图方便。
 * @since 0.1.0
 * @exporter
 */
[[nodiscard]] inline auto axis_deadtime_v(float deadtime_v, float angle_rad) noexcept -> float {
    // `std::cos` is not constexpr in C++17, so this is `inline` rather than
    // `constexpr`: the projection depends on a runtime angle anyway.
    // C++17 里 `std::cos` 不是 constexpr，所以这里是 `inline` 而不是 `constexpr`：
    // 投影本来就依赖运行时角度。
    const float c0 = std::cos(angle_rad);
    const float c1 = std::cos(angle_rad - 2.0943951F); // -120 deg
    const float c2 = std::cos(angle_rad + 2.0943951F); // +120 deg
    // The projection is `(2/3) * sum |cos(theta_p - angle)|`: the loss on a
    // phase is `deadtime_v * sign(i_p)` and `i_p` is proportional to
    // `cos(theta_p - angle)`, so `sign(i_p) * cos(theta_p - angle)` is its
    // magnitude. On d that sum is `1 + 1/2 + 1/2 = 2`; on q it is
    // `0 + sqrt(3)/2 + sqrt(3)/2 = sqrt(3)`; averaged over all angles it is
    // `3 * 2/pi = 1.91`, which is where the `4/pi` rotating-frame factor comes
    // from. One formula, three numbers, all of them the same physics.
    // 投影是 `(2/3)·Σ|cos(θ_p − angle)|`：一相的损失是 `deadtime_v·sign(i_p)`，而
    // `i_p` 正比于 `cos(θ_p − angle)`，所以 `sign(i_p)·cos(θ_p − angle)` 就是它的
    // 大小。d 轴上这个和是 `1 + 1/2 + 1/2 = 2`；q 轴上是
    // `0 + √3/2 + √3/2 = √3`；对全部角度取平均是 `3·2/π = 1.91`，那个 `4/π`
    // 的旋转系系数正是从这里来的。一个公式、三个数，都是同一份物理。
    const float projection = std::fabs(c0) + std::fabs(c1) + std::fabs(c2);
    return (0.6666667F * deadtime_v) * projection;
}

/**
 * @brief [en] Recovers stator resistance **and** the inverter's dead-time voltage from a settled DC current sweep, by
 *             separating the two terms the voltage equation contains.
 *
 *     Why two unknowns and not one. At standstill with a settled current, the command obeys `v = rs*i + deadtime_v`:
 *     the winding drops `rs*i` and the inverter eats `deadtime_v` regardless of current. Measuring at a single current
 *     cannot tell them apart -- a 10 % error in the dead-time voltage looks exactly like a 10 % error in `rs` at that
 *     one point. Two currents can: the slope in `i` is `rs`, the intercept is `deadtime_v`.
 *
 *     Two conditions it cannot work without, both physical rather than numerical. (1) **Currents above the fade band**:
 *     below a few tens of milliamps the loss is not yet a step (measured: it goes from 2.7 % to 97 % of
 *     `(4/pi)*deadtime_v` between 21 mA and 86 mA), so a low-current sample measures the fade, not the step, and biases
 *     `rs` downwards. (2) **One polarity**: `sign(i)` has to be constant across the sweep, or the intercept flips sign
 *     half way through and the fit means nothing.
 *
 * @brief [zh] 从一个稳定的直流电流扫描里同时把定子电阻**和**逆变器死区电压辨识出来——办法是把 电压方程里那两项分开。
 *
 *     为什么是两个未知数而不是一个。堵转、电流稳定时，指令满足 `v = rs·i + deadtime_v`：绕组吃掉 `rs·i`，逆变器不管电流
 *     大小都吃掉 `deadtime_v`。 在**一个**电流上测是分不开这两项的——死区电压差 10 % 和在那个点上 `rs` 差 10 % 完全等
 *     价。两个电流就能分开：随 `i` 的斜率是 `rs`，截距是 `deadtime_v`。
 *
 *     两个它离不开的条件，都是物理的而不是数值的。（1）**电流要在淡出带以上**： 几十毫安以下损失还不是阶跃（实测它在 21
 *     mA 到 86 mA 之间从 `(4/π)·deadtime_v` 的 2.7 % 走到 97 %），所以小电流样本量的是淡出、不是阶跃， 会把 `rs` 往下
 *     带。（2）**单极性**：`sign(i)` 在整个扫描里必须不变，否则截距中途 变号，拟合没有意义。
 *
 * @note [en] `rs` is measured at **whatever temperature the winding is at**; the plant does not report it. That is the
 *            point of measuring instead of reading a nameplate: copper moves about 0.4 %/K, so a 20 K error in the
 *            assumed temperature is an 8 % error in `rs`, which is larger than the whole tolerance of the observers
 *            built on it.
 * @note [zh] `rs` 是在**绕组当时的温度**下测出来的；plant 不会告诉你温度。这正是"测"而不是 "抄铭牌"的理由：铜的温度系数
 *            约 0.4 %/K，假设温度差 20 K 就是 `rs` 差 8 %，比建立 在它上面的观测器的整个容差还大。
 * @context [en] Commissioning, standstill, once per board.
 * @context [zh] 整定阶段，堵转，每块板一次。
 * @since 0.1.0
 * @exporter
 */
class RsEstimator final {
  public:
    /**
     * @brief [en] Which samples are allowed in.
     * @brief [zh] 允许哪些样本进来。
     * @since 0.1.0
     */
    struct Config final {
        /**
         * @brief [en] below this |i| the loss is not a step yet, A; 0 disables the floor.
         * @brief [zh] |i| 低于此值时损失还不是阶跃， A；0 关闭下限。
         */
        float i_min_a;
        /**
         * @brief [en] above this |i| the winding heats during the sweep, A; 0 disables the ceiling.
         * @brief [zh] |i| 高于此值时绕组 会在扫描中发热，A；0 关闭上限。
         */
        float i_max_a;
    };

    /**
     * @brief [en] What the two-point separation produced.
     * @brief [zh] 两点分离的结果。
     * @since 0.1.0
     */
    struct Result final {
        float rs; ///< [en] stator resistance in ohm. [zh] 定子电阻，Ω。
        /**
         * @brief [en] the loss the fit implies, V.
         * @brief [zh] 拟合隐含的损失电压，V。
         */
        float deadtime_v;
        /**
         * @brief [en] rms of what `rs*i + deadtime_v` could not explain, V.
         * @brief [zh] `rs·i + deadtime_v` 解释不掉的 部分的均方根，V。
         */
        float residual_v;
        /**
         * @brief [en] enough samples, two enough-spread currents, and a fit with a positive slope.
         * @brief [zh] 样本够、电流跨度够、且斜率是正的。
         */
        bool valid;
    };

    /// [en] Fewer settled points than this is not an identification. [zh]
    /// 稳定点的数量少于这个就不算辨识。
    static constexpr int kMinSamples = 6;

    /// [en] Highest / lowest current in the sweep must reach this ratio, or the
    /// slope is buried in the noise. [zh]
    /// 扫描里最高与最低电流之比必须达到这个值，否则斜率埋在噪声里。
    static constexpr float kMinSpanRatio = 1.6F;

    explicit RsEstimator(const Config& config) noexcept : config_(config) {}

    /**
     * @brief [en] Feed one settled (command, current) pair.
     * @brief [zh] 送入一个已稳定的（指令，电流）对。
     * @param v_command [en] the voltage magnitude that was commanded, V.
     * @param v_command [zh] 当时命令的电压幅值，V。
     * @param i_current [en] the settled current **magnitude** in A; it must be the same component `v_command` refers to
     *                       (the projection along the current), because that is the pair the voltage equation joins.
     * @param i_current [zh] 稳定后的电流**幅值**，A；它必须与 `v_command` 指的是同一个分量（沿电流方向的 投影），因为电
     *                       压方程连接的是这一对。
     * @return [en] true if the sample was used.
     * @return [zh] 该采样是否被采用。
     * @since 0.1.0
     */
    auto add(float v_command, float i_current) noexcept -> bool {
        const float current = std::fabs(i_current);
        if (!std::isfinite(v_command) || !std::isfinite(current)) {
            return false;
        }
        if (!(current > 0.0F)) {
            return false;
        }
        if ((config_.i_min_a > 0.0F) && (current < config_.i_min_a)) {
            return false;
        }
        if ((config_.i_max_a > 0.0F) && (current > config_.i_max_a)) {
            return false;
        }
        sum_i_ = static_cast<float>(sum_i_ + current);
        sum_v_ = static_cast<float>(sum_v_ + v_command);
        sum_ii_ = static_cast<float>(sum_ii_ + (current * current));
        sum_iv_ = static_cast<float>(sum_iv_ + (current * v_command));
        sum_vv_ = static_cast<float>(sum_vv_ + (v_command * v_command));
        lowest_ = ((samples_ == 0) || (current < lowest_)) ? current : lowest_;
        highest_ = ((samples_ == 0) || (current > highest_)) ? current : highest_;
        ++samples_;
        return true;
    }

    /// [en] Samples that were actually used. [zh] 实际采用的样本数。
    [[nodiscard]] auto samples() const noexcept -> int { return samples_; }

    /**
     * @brief [en] `rs` and `deadtime_v`, or `valid == false` with no number invented.
     * @brief [zh] `rs` 与 `deadtime_v`；给不出来时 `valid == false`，不编数。
     * @since 0.1.0
     */
    [[nodiscard]] auto solve() const noexcept -> Result {
        if (samples_ < kMinSamples) {
            return Result{0.0F, 0.0F, 0.0F, false};
        }
        const float n = static_cast<float>(samples_);
        const float determinant = (n * sum_ii_) - (sum_i_ * sum_i_);
        // The determinant is `n^2 * var(i)`: exactly the thing that goes to zero
        // when every sample sits at one current. Scaling it by `n^2 * mean(i^2)`
        // makes the check dimensionless, so the same threshold works for a 0.4 A
        // sweep and a 4 A one.
        // 行列式就是 `n²·var(i)`：所有样本挤在同一个电流上时它正好趋于零。把它按
        // `n²·mean(i²)` 归一化，检查就变成无量纲的，同一个阈值对 0.4 A 和 4 A 的
        // 扫描都适用。
        const float scale = n * sum_ii_;
        if (!(determinant > (kMinCondition * scale))) {
            return Result{0.0F, 0.0F, 0.0F, false};
        }
        if ((lowest_ > 0.0F) && ((highest_ / lowest_) < kMinSpanRatio)) {
            return Result{0.0F, 0.0F, 0.0F, false};
        }
        const float slope = (((n * sum_iv_) - (sum_i_ * sum_v_)) / determinant);
        const float intercept = (((sum_v_ * sum_ii_) - (sum_i_ * sum_iv_)) / determinant);
        if (!(slope > 0.0F)) {
            return Result{0.0F, 0.0F, 0.0F, false};
        }
        // SSE straight from the normal equations: no second pass, no storage.
        // 从正规方程直接算 SSE：不用第二遍，也不存样本。
        const float sse = (sum_vv_ - (2.0F * slope * sum_iv_)) - (2.0F * intercept * sum_v_) +
                          (slope * slope * sum_ii_) + (2.0F * slope * intercept * sum_i_) + (n * intercept * intercept);
        const float rms = std::sqrt((sse > 0.0F) ? (sse / n) : 0.0F);
        return Result{slope, intercept, rms, true};
    }

  private:
    /// [en] Sampling noise is unavoidable; a fit that cannot beat it is not a
    /// fit. [zh] 采样噪声无法避免；连它都打不过的拟合不算拟合。
    static constexpr float kMinCondition = 1.0e-3F;

    Config config_{};
    int samples_{0};
    float sum_i_{0.0F};
    float sum_v_{0.0F};
    float sum_ii_{0.0F};
    float sum_iv_{0.0F};
    float sum_vv_{0.0F};
    float lowest_{0.0F};
    float highest_{0.0F};
};

/**
 * @brief [en] Recovers the rotor flux linkage from the voltage a **zero-current** spin needs, at two or more speeds.
 *
 *     Why zero current, and why two speeds. With the current held at zero the voltage equation collapses to `v =
 *     we*psi_f` -- there is no `rs*i` term and no dead-time term, because a leg carrying no current has no loss to
 *     hide. One speed would give `psi_f` if `v` were exact; two give it as a *slope*, which absorbs whatever constant
 *     offset the measurement chain has (sensor gain, a residual current, an inverter drop that survives at zero
 *     current). The offset is reported rather than assumed away.
 *
 * @brief [zh] 从**零电流**旋转所需电压在两个以上转速下的值，把转子磁链辨识出来。
 *
 *     为什么零电流、为什么两个转速。电流保持为零时电压方程塌成 `v = ω_e·ψf`—— 没有 `rs·i` 项、也没有死区项，因为不过电
 *     流的桥臂没有损失可藏。如果 `v` 是准的， 一个转速就够；两个转速给的是**斜率**，它把测量链上任何固定偏置都吸收掉（
 *     传感器 增益、残余电流、零电流下依然存在的逆变器压降）。偏置是**报出来**的，不是假设掉 的。
 *
 * @note [en] The caller must actually hold the current at zero; that is not something this estimator can check, and it
 *            is the whole assumption. What it *can* check is that the samples it is handed are within `i_zero_max_a` of
 *            zero, which catches the common failure of sweeping speed before the current loop has settled.
 * @note [zh] 调用方必须真的把电流保持为零——这不是本估计器能检查的事，却是整个假设。它能检查 的是交给它的样本都在
 *            `i_zero_max_a` 以内，这能抓住常见的失误：电流环还没稳就开始 扫转速。
 * @context [en] Commissioning, spinning, once per board.
 * @context [zh] 整定阶段，旋转，每块板一次。
 * @since 0.1.0
 * @exporter
 */
class FluxEstimator final {
  public:
    /**
     * @brief [en] What the fit needs to know.
     * @brief [zh] 拟合需要知道的东西。
     * @since 0.1.0
     */
    struct Config final {
        /**
         * @brief [en] samples with a larger |i| are ignored, A.
         * @brief [zh] |i| 超过此值的样本被忽略，A。
         */
        float i_zero_max_a;
        /**
         * @brief [en] below this electrical speed the back-EMF is too small to measure, rad/s.
         * @brief [zh] 电角速度低于此值时反电动势太小，rad/s。
         */
        float omega_min_rad_s;
    };

    /**
     * @brief [en] What the fit produced.
     * @brief [zh] 拟合结果。
     * @since 0.1.0
     */
    struct Result final {
        float psi_f; ///< [en] rotor flux linkage in Wb. [zh] 转子磁链，Wb。
        /**
         * @brief [en] the constant part of the measurement, V.
         * @brief [zh] 测量里的固定偏置，V。
         */
        float offset_v;
        /**
         * @brief [en] rms of what a line could not explain, V.
         * @brief [zh] 一条直线解释不掉的部分的均方根，V。
         */
        float residual_v;
        /**
         * @brief [en] enough speeds, spread widely enough, positive slope.
         * @brief [zh] 转速够、跨度够、斜率为正。
         */
        bool valid;
    };

    /// [en] Two points define a line; four let the residual say something. [zh]
    /// 两点定一条直线；四点才能让残差说出点什么。
    static constexpr int kMinSamples = 4;

    /// [en] Fastest / slowest speed must reach this ratio. [zh]
    /// 最快与最慢转速之比必须达到这个值。
    static constexpr float kMinSpanRatio = 1.6F;

    explicit FluxEstimator(const Config& config) noexcept : config_(config) {}

    /**
     * @brief [en] Feed one settled zero-current operating point.
     * @brief [zh] 送入一个已稳定的零电流工作点。
     * @param omega_e [en] electrical speed in rad/s.
     * @param omega_e [zh] 电角速度，rad/s。
     * @param v_magnitude [en] the voltage the inverter delivered, V.
     * @param v_magnitude [zh] 逆变器交付的电压幅值，V。
     * @param i_magnitude [en] the current that was flowing, A; used only to police the zero-current assumption.
     * @param i_magnitude [zh] 当时流过的电流，A；只用来守住 "零电流" 这个假设。
     * @return [en] true if the sample was used.
     * @return [zh] 该采样是否被采用。
     * @since 0.1.0
     */
    auto add(float omega_e, float v_magnitude, float i_magnitude) noexcept -> bool {
        const float speed = std::fabs(omega_e);
        if (!std::isfinite(v_magnitude) || !std::isfinite(i_magnitude)) {
            return false;
        }
        if (!(speed > 0.0F)) {
            return false;
        }
        if ((config_.omega_min_rad_s > 0.0F) && (speed < config_.omega_min_rad_s)) {
            return false;
        }
        if ((config_.i_zero_max_a > 0.0F) && (std::fabs(i_magnitude) > config_.i_zero_max_a)) {
            return false;
        }
        sum_x_ = static_cast<float>(sum_x_ + speed);
        sum_y_ = static_cast<float>(sum_y_ + v_magnitude);
        sum_xx_ = static_cast<float>(sum_xx_ + (speed * speed));
        sum_xy_ = static_cast<float>(sum_xy_ + (speed * v_magnitude));
        sum_yy_ = static_cast<float>(sum_yy_ + (v_magnitude * v_magnitude));
        lowest_ = ((samples_ == 0) || (speed < lowest_)) ? speed : lowest_;
        highest_ = ((samples_ == 0) || (speed > highest_)) ? speed : highest_;
        ++samples_;
        return true;
    }

    /// [en] Samples that were actually used. [zh] 实际采用的样本数。
    [[nodiscard]] auto samples() const noexcept -> int { return samples_; }

    /**
     * @brief [en] The flux linkage, or `valid == false`.
     * @brief [zh] 磁链；给不出来时 `valid == false`。
     * @since 0.1.0
     */
    [[nodiscard]] auto solve() const noexcept -> Result {
        if (samples_ < kMinSamples) {
            return Result{0.0F, 0.0F, 0.0F, false};
        }
        const float n = static_cast<float>(samples_);
        const float determinant = (n * sum_xx_) - (sum_x_ * sum_x_);
        if (!(determinant > (kMinCondition * n * sum_xx_))) {
            return Result{0.0F, 0.0F, 0.0F, false};
        }
        if ((lowest_ > 0.0F) && ((highest_ / lowest_) < kMinSpanRatio)) {
            return Result{0.0F, 0.0F, 0.0F, false};
        }
        const float slope = (((n * sum_xy_) - (sum_x_ * sum_y_)) / determinant);
        const float intercept = (((sum_y_ * sum_xx_) - (sum_x_ * sum_xy_)) / determinant);
        if (!(slope > 0.0F)) {
            return Result{0.0F, 0.0F, 0.0F, false};
        }
        const float sse = (sum_yy_ - (2.0F * slope * sum_xy_)) - (2.0F * intercept * sum_y_) +
                          (slope * slope * sum_xx_) + (2.0F * slope * intercept * sum_x_) + (n * intercept * intercept);
        const float rms = std::sqrt((sse > 0.0F) ? (sse / n) : 0.0F);
        return Result{slope, intercept, rms, true};
    }

  private:
    /// [en] Same conditioning floor as the resistance fit. [zh]
    /// 与电阻拟合相同的条件数下限。
    static constexpr float kMinCondition = 1.0e-3F;

    Config config_{};
    int samples_{0};
    float sum_x_{0.0F};
    float sum_y_{0.0F};
    float sum_xx_{0.0F};
    float sum_xy_{0.0F};
    float sum_yy_{0.0F};
    float lowest_{0.0F};
    float highest_{0.0F};
};

/**
 * @brief [en] Recovers one current channel's **offset** without a reference and its **gain** with one.
 *
 *     Why the two are not symmetric, and why that is the whole point. An offset has a free reference: command zero
 *     current, and whatever a channel reads *is* its offset, because the true value is known to be zero. A gain has
 *     none -- `reading = gain*truth + offset` with `truth` unknown is underdetermined at any current, so a gain needs
 *     an external truth (a clamp meter, a series shunt, or a second channel that has already been calibrated). The
 *     estimator therefore reports the two separately and refuses the one it cannot support, rather than fitting a slope
 *     to noise and calling it a gain.
 *
 * @brief [zh] 不需要基准就能得到一条电流通道的**零漂**，有基准才能得到它的**增益**。
 *
 *     为什么两者不对称，以及为什么这正是要点。零漂有一个免费的基准：命令零电流，通道读到的 **就是**它的零漂，因为真值已
 *     知为零。增益没有：`reading = gain·truth + offset` 而 `truth` 未知时，在**任何**电流上都是欠定的，所以增益需要一个
 *     外部真值（钳形表、串联 分流器，或另一条已经校正过的通道）。于是估计器把两者分开报，并且拒绝它支持不了的那个， 而
 *     不是拿噪声拟合一条斜率再叫它增益。
 *
 * @note [en] The three-wire constraint is a diagnostic, not a calibration. The true phase currents sum to zero, so the
 *            **readings** sum to the sum of the offsets: `sum(offset)` is measurable with no reference at all. But it
 *            is one equation for three unknowns -- it can tell you that something is wrong, and not which phase is
 *            wrong. Splitting it needs the per-channel `add_zero()`.
 * @note [zh] 三线约束是**诊断**，不是标定。真实相电流之和为零，所以**读数**之和就是 零漂之和：`Σoffset` 完全不需要基准
 *            就能测出来。但那是三个未知数的一个方程——它能告诉你 "有问题"，不能告诉你"哪一相"。要拆开就得用逐通道的
 *            `add_zero()`。
 * @context [en] Commissioning, first, before any angle alignment: an offset corrupts the alignment (
 * @context [zh] 整定阶段，**最先做**，在任何角度对齐之前：零漂会污染 对齐全过程).
 * @since 0.1.0
 * @exporter
 */
class CurrentSenseCalibrator final {
  public:
    /**
     * @brief [en] What the fit has to refuse.
     * @brief [zh] 拟合必须拒绝什么。
     * @since 0.1.0
     */
    struct Config final {
        /**
         * @brief [en] the reference must swing this far for a gain, A; 0 disables.
         * @brief [zh] 求增益时基准至少要摆动这么远，A； 0 关闭。
         */
        float span_min_a;
        /**
         * @brief [en] the zero-current readings may differ by this much, A; 0 disables.
         * @brief [zh] 零电流读数之间 最多允许差这么多，A；0 关闭。
         */
        float zero_spread_max_a;
    };

    /**
     * @brief [en] What was measured.
     * @brief [zh] 量到了什么。
     * @since 0.1.0
     */
    struct Result final {
        /**
         * @brief [en] reading = gain*truth + offset.
         * @brief [zh] 读数 = gain·真值 + 零漂。
         */
        float gain;
        float offset_a; ///< [en] the fitted intercept, A. [zh] 拟合截距，A。
        /**
         * @brief [en] rms of what the line could not explain, A.
         * @brief [zh] 直线解释不掉的部分的均方根，A。
         */
        float residual_a;
        /**
         * @brief [en] mean reading of the zero-current samples, A.
         * @brief [zh] 零电流样本的平均读数，A。
         */
        float zero_offset_a;
        /**
         * @brief [en] max minus min of those samples, A.
         * @brief [zh] 那些样本的极差，A。
         */
        float zero_spread_a;
        /**
         * @brief [en] enough pairs spread far enough.
         * @brief [zh] 样本对够、跨度够。
         */
        bool gain_valid;
        /**
         * @brief [en] enough zero-current samples, tightly clustered.
         * @brief [zh] 零电流样本够、够集中。
         */
        bool offset_valid;
    };

    /// [en] Fewer pairs than this and the slope is noise. [zh]
    /// 样本对少于这个时斜率就是噪声。
    static constexpr int kMinPairs = 6;

    /// [en] Fewer zero-current samples than this and the mean is noise. [zh]
    /// 零电流样本少于这个时均值就是噪声。
    static constexpr int kMinZeroSamples = 8;

    /// [en] Reference span (max/min) the gain fit needs. [zh]
    /// 求增益所需的基准跨度（最大/最小）。
    static constexpr float kMinSpanRatio = 2.0F;

    explicit CurrentSenseCalibrator(const Config& config) noexcept : config_(config) {}

    /**
     * @brief [en] Feed a reading whose true value is known to be zero.
     * @brief [zh] 送入一个真值已知为零的读数。
     * @param reading_a [en] what the channel reported, A.
     * @param reading_a [zh] 通道报出的值，A。
     * @return [en] true if the sample was used.
     * @return [zh] 该采样是否被采用。
     * @since 0.1.0
     */
    auto add_zero(float reading_a) noexcept -> bool {
        if (!std::isfinite(reading_a)) {
            return false;
        }
        sum_zero_ = static_cast<float>(sum_zero_ + reading_a);
        lowest_zero_ = ((zero_samples_ == 0) || (reading_a < lowest_zero_)) ? reading_a : lowest_zero_;
        highest_zero_ = ((zero_samples_ == 0) || (reading_a > highest_zero_)) ? reading_a : highest_zero_;
        ++zero_samples_;
        return true;
    }

    /**
     * @brief [en] Feed a reading together with an external reference.
     * @brief [zh] 送入一个读数以及一个外部基准。
     * @param reading_a [en] what the channel reported, A.
     * @param reading_a [zh] 通道报出的值，A。
     * @param reference_a [en] the true current, A -- from a clamp meter, a series shunt or an already-calibrated
     *                         channel.
     * @param reference_a [zh] 真实电流，A——来自钳形 表、串联分流器或已校正的通道。
     * @return [en] true if the sample was used.
     * @return [zh] 该采样是否被采用。
     * @since 0.1.0
     */
    auto add(float reading_a, float reference_a) noexcept -> bool {
        if (!std::isfinite(reading_a) || !std::isfinite(reference_a)) {
            return false;
        }
        const float magnitude = std::fabs(reference_a);
        sum_r_ = static_cast<float>(sum_r_ + reference_a);
        sum_y_ = static_cast<float>(sum_y_ + reading_a);
        sum_rr_ = static_cast<float>(sum_rr_ + (reference_a * reference_a));
        sum_ry_ = static_cast<float>(sum_ry_ + (reference_a * reading_a));
        sum_yy_ = static_cast<float>(sum_yy_ + (reading_a * reading_a));
        lowest_r_ = ((pairs_ == 0) || (magnitude < lowest_r_)) ? magnitude : lowest_r_;
        highest_r_ = ((pairs_ == 0) || (magnitude > highest_r_)) ? magnitude : highest_r_;
        ++pairs_;
        return true;
    }

    /// [en] Pairs actually used. [zh] 实际采用的样本对数。
    [[nodiscard]] auto pairs() const noexcept -> int { return pairs_; }

    /// [en] Zero-current samples actually used. [zh] 实际采用的零电流样本数。
    [[nodiscard]] auto zero_samples() const noexcept -> int { return zero_samples_; }

    /**
     * @brief [en] Offset and gain, each with its own validity.
     * @brief [zh] 零漂与增益，各自带自己的有效性。
     * @since 0.1.0
     */
    [[nodiscard]] auto solve() const noexcept -> Result {
        Result result{1.0F, 0.0F, 0.0F, 0.0F, 0.0F, false, false};

        if (zero_samples_ >= kMinZeroSamples) {
            result.zero_offset_a = sum_zero_ / static_cast<float>(zero_samples_);
            result.zero_spread_a = static_cast<float>(highest_zero_ - lowest_zero_);
            const bool tight =
                !(config_.zero_spread_max_a > 0.0F) || (result.zero_spread_a <= config_.zero_spread_max_a);
            result.offset_valid = tight;
        }

        if (pairs_ < kMinPairs) {
            return result;
        }
        const float n = static_cast<float>(pairs_);
        const float determinant = (n * sum_rr_) - (sum_r_ * sum_r_);
        if (!(determinant > (kMinCondition * n * sum_rr_))) {
            return result;
        }
        if ((lowest_r_ > 0.0F) && ((highest_r_ / lowest_r_) < kMinSpanRatio)) {
            return result;
        }
        const float span = static_cast<float>(highest_r_ - lowest_r_);
        if ((config_.span_min_a > 0.0F) && (span < config_.span_min_a)) {
            return result;
        }
        const float slope = (((n * sum_ry_) - (sum_r_ * sum_y_)) / determinant);
        const float intercept = (((sum_y_ * sum_rr_) - (sum_r_ * sum_ry_)) / determinant);
        if (!(slope > 0.0F)) {
            return result;
        }
        const float sse = (sum_yy_ - (2.0F * slope * sum_ry_)) - (2.0F * intercept * sum_y_) +
                          (slope * slope * sum_rr_) + (2.0F * slope * intercept * sum_r_) + (n * intercept * intercept);
        result.gain = slope;
        result.offset_a = intercept;
        result.residual_a = std::sqrt((sse > 0.0F) ? (sse / n) : 0.0F);
        result.gain_valid = true;
        return result;
    }

  private:
    /// [en] Same conditioning floor as the other fits in this header. [zh]
    /// 与本文件其它拟合相同的条件数下限。
    static constexpr float kMinCondition = 1.0e-3F;

    Config config_{};
    int pairs_{0};
    float sum_r_{0.0F};
    float sum_y_{0.0F};
    float sum_rr_{0.0F};
    float sum_ry_{0.0F};
    float sum_yy_{0.0F};
    float lowest_r_{0.0F};
    float highest_r_{0.0F};
    int zero_samples_{0};
    float sum_zero_{0.0F};
    float lowest_zero_{0.0F};
    float highest_zero_{0.0F};
};

/**
 * @brief [en] Recovers an incremental inductance from the **flux** a voltage step delivers, `L = integral(v -
 *             rs*i)/delta_i`, instead of from a differentiated current.
 *
 *     Why integrate. `di/dt` from a 20 kHz current sensor is a difference of two nearly equal noisy numbers scaled by
 *     1/dt: it amplifies the noise by `1/dt` and needs a filter whose group delay then biases the result. The integral
 *     form uses every sample once, weights nothing, and needs no filter: `integral(v_delivered - rs*i) dt = L *
 *     delta_i` exactly, because the `L*di/dt` term is the derivative of `L*i`.
 *
 *     What `v_delivered` is, and the error that follows. The inverter delivers `v_command - deadtime_v*sign(i)`, which
 *     is what this integrator subtracts using the `deadtime_v` measured by `RsEstimator`. While the current crosses
 *     zero the true loss is *less* than `deadtime_v` (that is the fade band), so the integrator over-subtracts for the
 *     crossing time `t_cross` and the flux comes out low by about `deadtime_v * t_cross`. At 0.48 V and 20 us against
 *     `L*delta_i = 4e-4 V*s` for a 1 A step, that is 2.4 % -- inside the tolerance, but it is a *known direction*, so
 *     it is documented rather than hoped away.
 *
 * @brief [zh] 从电压阶跃交付的**磁链**求增量电感，`L = ∫(v − rs·i)/Δi`，而不是靠微分电流。
 *
 *     为什么用积分。20 kHz 电流传感器的 `di/dt` 是两个几乎相等的带噪数之差再乘 `1/dt`： 噪声被放大 `1/dt` 倍，要有滤波
 *     器，而滤波器的群延迟又会带来偏差。积分形式每个样本 用一次、不加权、不用滤波：`∫(v_delivered − rs·i)dt = L·Δi` 是
 *     **精确**的，因为 `L·di/dt` 这一项就是 `L·i` 的导数。
 *
 *     `v_delivered` 是什么，以及随之而来的误差。逆变器交付 `v_cmd − deadtime_v·sign(i)`， 本积分器用 `RsEstimator` 量到
 *     的 `deadtime_v` 把它减掉。电流过零期间真实损失**小** 于 `deadtime_v`（那就是淡出带），所以积分在穿越时间
 *     `t_cross` 内减多了， 磁链偏低约 `deadtime_v·t_cross`。0.48 V、20 µs，对 1 A 阶跃的 `L·Δi = 4e-4 V·s` 是 2.4 %——在
 *     容差内，但**方向是已知的**，所以要写下来而不是 指望它不存在。
 *
 * @note [en] `begin()` is what makes this a measurement rather than a running average: the rotor frame has to be
 *            aligned, the axis has to be named (d or q is chosen by which voltage component the caller steps), and the
 *            current has to be at a known starting value. Calling `add()` without `begin()` is a programming error and
 *            simply produces `valid == false`.
 * @note [zh] `begin()` 是它成为一次测量而不是滑动平均的原因：转子系要对准、轴要指明（d 还是 q 由调用方在哪一个电压分量
 *            上加阶跃决定）、电流要从已知值出发。没调 `begin()` 就 `add()` 是编程错误，结果只是 `valid == false`。
 * @context [en] Commissioning, standstill, once per board and per axis.
 * @context [zh] 整定阶段，堵转，每块板、每个轴一次。
 * @since 0.1.0
 * @exporter
 */
class IncrementalInductanceEstimator final {
  public:
    /**
     * @brief [en] What the integrator has to subtract, and what it must refuse.
     * @brief [zh] 积分器要减掉什么，以及它必须拒绝什么。
     * @since 0.1.0
     */
    struct Config final {
        float rs; ///< [en] stator resistance in ohm. [zh] 定子电阻，Ω。
        /**
         * @brief [en] (t_dead/T_pwm)*v_bus in V; 0 = ignore.
         * @brief [zh] (t_dead/T_pwm)*v_bus，V；0 = 不计。
         */
        float deadtime_v;
        /**
         * @brief [en] the step must move the current at least this far, A.
         * @brief [zh] 阶跃至少要推动电流这么多，A。
         */
        float i_span_min_a;
    };

    /**
     * @brief [en] What the integral produced.
     * @brief [zh] 积分结果。
     * @since 0.1.0
     */
    struct Result final {
        float l_h; ///< [en] incremental inductance in H. [zh] 增量电感，H。
        /**
         * @brief [en] the flux the step delivered, Wb.
         * @brief [zh] 阶跃交付的磁链，Wb。
         */
        float flux_wb;
        /**
         * @brief [en] how far the current moved, A.
         * @brief [zh] 电流移动了多少，A。
         */
        float delta_i_a;
        /**
         * @brief [en] begun, enough samples, and a current span wide enough to divide by.
         * @brief [zh] 已开始、样本够、电流跨度足够做除法。
         */
        bool valid;
    };

    /// [en] Fewer integration steps than this and `dt` dominates. [zh]
    /// 积分步数少于这个时 `dt` 的误差占主导。
    static constexpr int kMinSamples = 3;

    explicit IncrementalInductanceEstimator(const Config& config) noexcept : config_(config) {}

    /**
     * @brief [en] Start a step from a known current.
     * @brief [zh] 从一个已知电流开始一次阶跃。
     * @param i_start_a [en] the current before the step, A.
     * @param i_start_a [zh] 阶跃前的电流，A。
     * @since 0.1.0
     */
    void begin(float i_start_a) noexcept {
        start_i_ = i_start_a;
        current_ = i_start_a;
        flux_ = 0.0F;
        samples_ = 0;
        started_ = std::isfinite(i_start_a);
    }

    /**
     * @brief [en] Integrate one complete switching period.
     * @brief [zh] 积分一个完整的开关周期。
     * @param v_command [en] the voltage commanded on the axis being stepped, V.
     * @param v_command [zh] 被阶跃的那个轴上的电压指令，V。
     * @param i_mean [en] the current on that axis **averaged over the period**, A -- not an instantaneous sample.
     * @param i_mean [zh] 该轴上的电流在该周期上的**平均值**， A——不是瞬时采样。
     * @param dt_s [en] the length of that period, s.
     * @param dt_s [zh] 该周期的长度，s。
     * @return [en] true if the sample was used.
     * @return [zh] 该采样是否被采用。
     * @note [en] Why the mean and not the sample. Over one switching period the current swings by `v_bus/(4*L*f_sw)` --
     *            0.75 A peak-to-peak here, against a step of 1 A. Integrating instantaneous samples would measure the
     *            ripple and call it the transient, and reading `delta_i` from two instantaneous values would be wrong
     *            by up to a full ripple. Averaging over a whole period cancels the ripple exactly, because its mean
     *            over a period is what `L*di/dt` integrates to.
     * @note [zh] 为什么要均值而不是采样点。一个开关周期内电流摆动 `v_bus/(4·L·f_sw)`——本机 0.75 A 峰峰，而阶跃才 1 A。
     *            积分瞬时采样等于在量纹波 并把它当成暂态，而用两个瞬时值算 `Δi` 会错整整一个纹波。对整周期取均值能把纹
     *            波 精确消掉，因为 `L·di/dt` 积分出来正是它在一个周期上的均值。
     * @since 0.1.0
     */
    auto add(float v_command, float i_mean, float dt_s) noexcept -> bool {
        if (!started_ || !std::isfinite(v_command) || !std::isfinite(i_mean) || !(dt_s > 0.0F)) {
            return false;
        }
        // `sign(0) == 0` is the physically right choice here: a leg carrying no
        // current really does drop nothing. It is also the source of the known
        // `deadtime_v * t_cross` bias documented above.
        // 这里 `sign(0) == 0` 是物理上正确的：不过电流的桥臂确实不掉电压。它也正是
        // 上面那条已知 `deadtime_v·t_cross` 偏差的来源。
        flux_ = static_cast<float>(flux_ + ((v_command - loss_of(i_mean) - (config_.rs * i_mean)) * dt_s));
        current_ = i_mean;
        ++samples_;
        return true;
    }

    /// [en] Integration steps that were actually used. [zh] 实际采用的积分步数。
    [[nodiscard]] auto samples() const noexcept -> int { return samples_; }

    /**
     * @brief [en] The inductance, or `valid == false`.
     * @brief [zh] 电感；给不出来时 `valid == false`。
     * @since 0.1.0
     */
    [[nodiscard]] auto solve() const noexcept -> Result {
        const float delta = static_cast<float>(current_ - start_i_);
        if (!started_ || (samples_ < kMinSamples)) {
            return Result{0.0F, 0.0F, delta, false};
        }
        if (!(std::fabs(delta) >= config_.i_span_min_a) || !(config_.i_span_min_a > 0.0F)) {
            return Result{0.0F, flux_, delta, false};
        }
        const float l = flux_ / delta;
        return Result{l, flux_, delta, l > 0.0F};
    }

  private:
    /// [en] The loss the inverter takes from this leg at this current. [zh]
    /// 逆变器在这条臂、这个电流下吃掉的电压。
    [[nodiscard]] auto loss_of(float current_a) const noexcept -> float {
        if (!(config_.deadtime_v > 0.0F)) {
            return 0.0F;
        }
        if (current_a > 0.0F) {
            return config_.deadtime_v;
        }
        return (current_a < 0.0F) ? -config_.deadtime_v : 0.0F;
    }

    Config config_{};
    bool started_{false};
    int samples_{0};
    float start_i_{0.0F};
    float current_{0.0F};
    float flux_{0.0F};
};

} // namespace core

} // namespace foc
