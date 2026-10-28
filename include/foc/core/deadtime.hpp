// Conan::ImportStart
#pragma once
#include <cmath>
#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] Adds back what the inverter's dead time takes away, then hands the request to a real modulator.
 *
 *     Dead time clamps each phase to the rail its current is coming from for `t_dead`, so the inverter delivers `v_cmd
 *     - v_dead * sign(i_phase)` per phase *when the current is large*. Commanding `v_desired + v_dead * sign(i_phase)`
 *     therefore lands on `v_desired`, provided the stage can see the phase currents -- which is exactly why
 *     `ModulatorInput` carries them.
 *
 *     It is a *decorator*, not a flag on `Svpwm`: space-vector modulation and inverter linearisation are two different
 *     jobs, and any modulator can be compensated this way.
 *
 * @brief [zh] 把逆变器死区吃掉的电压加回去，再把请求交给真正的调制器。
 *
 *     死区在每个 `t_dead` 内把该相钳位到电流来自的那条母线，因此在电流**较大**时逆变器每相交付 `v_cmd −
 *     v_dead·sign(i_phase)`。于是命令 `v_desired + v_dead·sign(i_phase)` 就能落到 `v_desired`——前提是这一级看得到相电
 *     流，这正是 `ModulatorInput` 带着它们的原因。
 *
 *     它是**装饰器**而不是 `Svpwm` 上的一个开关：空间矢量调制与逆变器线性化是两件事，而且任何 调制器都能用这种方式被补
 *     偿。
 *
 * @note [en] The correction is applied in the **phase** domain (inverse Clarke, per-phase shape, Clarke back), because
 *            that is where the physical mechanism lives: the loss belongs to each phase, not to the vector. Applying it
 *            to the vector magnitude would compensate a different machine. What it cannot do: the loss does not switch
 *            polarity at a zero crossing the way `sign()` says. It fades over a current range, and inside that range a
 *            `sign()` step *over-corrects* -- measured on a switching plant, the step is worse than no compensation at
 *            all near zero (measured on the switching plant).
 *
 * @note [zh] 修正在**相域**进行（逆 Clarke、按相整形、再 Clarke 回去），因为物理机制就在那里：损失属于 每一相，而不属于
 *            矢量。按矢量幅值补偿等于在补偿另一台电机。
 *
 *     它做不到的：损失不会像 `sign()` 说的那样在过零处瞬间换极性，而是在一段电流范围内淡出； 在这个范围内 `sign()` 会**
 *     过修**——开关 plant 上实测，近零处阶跃补偿比不补偿还差。
 *
 * @context [en] Current-loop ISR: noexcept, no allocation, one branch when disabled.
 * @context [zh] 电流环 ISR：noexcept、不分配、未启用时只有一个分支。
 * @since 0.1.0
 * @exporter
 */
template <typename Modulator> class DeadTimeCompensator final {
  public:
    /**
   * @brief [en] The numbers this stage needs.
   * @brief [zh] 这一级需要的数。
   * @note [en] `deadtime_v == 0` disables the correction entirely (the request passes through untouched), which is how
   *            the same build can be measured with and without it.
   * @note [zh] `deadtime_v == 0` 完全关闭修正（请求原样透传），同一个构建因此可以用它开/关来测量。
   * @since 0.1.0
   * @exporter
   */
    struct Config final {
        float deadtime_v;    ///< [en] (t_dead/T_pwm)*v_bus, V; 0 = off. [zh]
                             ///< (t_dead/T_pwm)*v_bus，V；0 = 关闭。
        float min_current_a; ///< [en] below this |i| the sign is not trusted, A.
                             ///< [zh] |i| 低于此值时不信任符号，A。
        /**
     * @brief [en] The current at which the correction saturates -- i.e. the **width of the fade** in A; 0 keeps the
     *             legacy `sign()` step.
     * @brief [zh] 修正饱和处的电流——也就是**淡出宽度**，A；0 保持旧的 `sign()` 阶跃。
     * @note [en] This is *not* the switching ripple: measured on a switching plant the two differ by more than an order
     *            of magnitude (0.375 A vs about 0.03 A here), and feeding the ripple makes the correction far too weak
     *            at intermediate currents (6.6x the step's error at 0.3 A, on the switching plant). It is identifiable
     *            -- see `DeadTimeFadeEstimator`.
     * @note [zh] 这**不是**开关纹波：开关 plant 上实测两者差一个数量级以上（本机 0.375 A vs 约 0.03 A），填纹波会让修正
     *            在中间电流处过弱得多（0.3 A 处误差是阶跃的 6.6 倍，开关 plant 实测）。它是**可辨识**的——见
     *            `DeadTimeFadeEstimator`。
     * @warning [en] **Not recommended.** The width can be identified (measured: 0.0355 A at 24 V, and it halves exactly
     *               when the bus halves), but the identified number still loses to the plain `sign()` step: 1.16x its
     *               error at 0.3 A and 1.58x at 20 mA. The reason is a wrong *shape*, not a wrong number -- on the
     *               plant the loss goes from 2.7 % to 97 % of `(4/pi)*deadtime_v` between |i| = 21 mA and 86 mA, which
     *               is a soft step, and a linear clamp spread over `If` cannot represent something that steep. So leave
     *               this at 0 unless a measurement on *your* hardware says the transition really is broad; and note
     *               that a ramp's advantage, when it exists, is at low current, where the PI already holds the mean.
     * @warning [zh] **不建议使用。** 宽度是可辨识的（实测：24 V 下 0.0355 A，且母线 减半时它正好减半），但辨识出来的数
     *               仍然输给朴素的 `sign()` 阶跃：0.3 A 处 误差是它的 1.16 倍，20 mA 处 1.58 倍。原因是**形状**错而不是
     *               数错——plant 上 损失在 |i| 从 21 mA 到 86 mA 之间从 `(4/π)·deadtime_v` 的 2.7 % 跳到 97 %， 那是一个
     *               软阶跃，铺在 `If` 上的线性斜坡表示不了这么陡的东西。所以除非**你的** 硬件上量出来过渡确实很宽，这一
     *               项保持 0；而且要注意斜坡的优势（如果存在）都在 小电流处，而那里 PI 本来就已经把均值守住了。
     * @since 0.1.0
     */
        float i_fade_a;
    };

    /**
   * @brief [en] An upper bound on the switching ripple, from numbers the board already knows.
   * @brief [zh] 开关纹波的上界，由板子本来就知道的数算出来。
   * @param v_bus [en] DC link voltage in V.
   * @param v_bus [zh] 母线电压，V。
   * @param dt_s [en] switching (not control) period in s.
   * @param dt_s [zh] 开关周期（不是控制周期），s。
   * @param l_h [en] inductance the ripple flows through in H.
   * @param l_h [zh] 纹波流过的电感，H。
   * @return [en] peak ripple bound in A, i.e. half of `v_bus*dt/(4L)`.
   * @return [zh] 纹波峰值上界，A，即 `v_bus·dt/(4L)` 的一半。
   * @warning [en] **Not a value for `i_fade_a`.** It bounds the ripple of one leg against the rails; the fade is
   *               narrower (the dead-time windows sample the ripple at particular instants) and a three-phase machine
   *               sees less than a single leg would. It is kept because its *scaling* is right: measure the fade once
   *               and this tells you how it moves with bus voltage and inductance.
   * @warning [zh] **不是 `i_fade_a` 的取值。** 它是单条桥臂对母线的纹波上界；淡出更窄（死区窗口只在特定 时刻对纹波取
   *               样），而三相电机看到的比单条腿更少。保留它只是因为它的**缩放关系**对：量过一次 之后，它告诉你淡出宽度
   *               随母线电压与电感怎么变。
   * @since 0.1.0
   */
    [[nodiscard]] static constexpr auto predicted_ripple_a(float v_bus, float dt_s, float l_h) noexcept -> float {
        return ((v_bus > 0.0F) && (dt_s > 0.0F) && (l_h > 0.0F)) ? ((v_bus * dt_s) / (8.0F * l_h)) : 0.0F;
    }

    DeadTimeCompensator() noexcept = default;

    explicit DeadTimeCompensator(const Config& config) noexcept : config_(config) {}

    /**
   * @brief [en] Compensated duties for this step.
   * @brief [zh] 本步补偿后的占空比。
   * @param request [en] desired stator voltage, bus and measured currents.
   * @param request [zh] 期望定子电压、母线与实测电流。
   * @return [en] duties plus saturation and status, from the wrapped modulator.
   * @return [zh] 占空比与饱和/状态， 来自被包装的调制器。
   * @since 0.1.0
   */
    [[nodiscard]] auto modulate(const ModulatorInput& request) noexcept -> SvpwmResult {
        if (!(config_.deadtime_v > 0.0F)) {
            return Modulator::modulate(request);
        }

        // Per phase: the sign is the phase's own, and below `min_current_a` it is
        // noise rather than information -- commanding a correction there would
        // inject a square wave into a signal that is mostly offset error.
        // 按相：符号属于该相自己；|i| 低于 `min_current_a`
        // 时它是噪声而不是信息——在那里施加修正等于往一个
        // 基本只有偏置误差的信号里注入方波。
        const Uvw<float> v_uvw = inv_clarke(request.v_ab);
        const float gain = config_.deadtime_v;

        // With a fade width the correction is the ramp the *averaged* error
        // follows: across the dead-time windows the phase current sits somewhere
        // inside its own ripple, so the volt-seconds left behind are `deadtime_v *
        // i/i_fade` while |i| < i_fade, and `deadtime_v * sign(i)` beyond it. The
        // step is only the limit of that ramp at zero width -- which is why it
        // over-corrects wherever the current is comparable to the fade.
        // 有了淡出宽度，修正就是**平均**误差所服从的那条斜坡：死区窗口内相电流落在自身纹波里的某个位置，
        // 因此留下的伏秒在 |i| < i_fade 时是 `deadtime_v·i/i_fade`，超出后是
        // `deadtime_v·sign(i)`。
        // 阶跃只是这条斜坡在宽度趋于零时的极限——这就是为什么凡是电流与淡出可比的地方它都会过修。
        const float fade = config_.i_fade_a;
        const float scale = (fade > config_.min_current_a) ? fade : config_.min_current_a;
        const Uvw<float> corrected{static_cast<float>(v_uvw.u + (gain * shape_of(request.i_uvw.u, scale, fade))),
                                   static_cast<float>(v_uvw.v + (gain * shape_of(request.i_uvw.v, scale, fade))),
                                   static_cast<float>(v_uvw.w + (gain * shape_of(request.i_uvw.w, scale, fade)))};

        ModulatorInput forwarded = request;
        forwarded.v_ab = clarke(corrected);
        return Modulator::modulate(forwarded);
    }

    /// [en] The configuration in force. [zh] 当前生效的配置。
    [[nodiscard]] auto config() const noexcept -> const Config& { return config_; }

    /// [en] Change the configuration (dead time belongs to the board, so it is a
    /// boot-time value).
    ///      [zh] 修改配置（死区属于板子，因此是启动期取值）。
    void configure(const Config& config) noexcept { config_ = config; }

  private:
    /// [en] +1 / -1 by sign, 0 inside the dead band. [zh] 按符号取 ±1，死区内取
    /// 0。
    [[nodiscard]] static auto sign_of(float value, float deadband) noexcept -> float {
        if (!(value > deadband)) {
            if (!(value < -deadband)) {
                return 0.0F;
            }
            return -1.0F;
        }
        return 1.0F;
    }

    /// [en] The ramp when a fade width is known, the legacy step when it is not.
    /// [zh] 知道淡出宽度时用斜坡， 不知道时用旧的阶跃。
    [[nodiscard]] static auto shape_of(float value, float scale, float fade) noexcept -> float {
        if (!(fade > 0.0F)) {
            return sign_of(value, scale);
        }
        const float normalized = value / ((scale > 0.0F) ? scale : fade);
        if (!(normalized > -1.0F)) {
            return -1.0F;
        }
        if (!(normalized < 1.0F)) {
            return 1.0F;
        }
        return normalized;
    }

    Config config_{};
};

/**
 * @brief [en] Recovers the dead-time **fade width** from a locked-rotor rotating-current sweep, by fitting one
 *             parameter.
 *
 *     Why this exists: the width is the one number the compensation stage needs and the one number nobody can compute
 *     from the nameplate (`predicted_ripple_a()` is an upper bound that is wrong by an order of magnitude). It is,
 *     however, *measurable*, and cheaply: hold the rotor, sweep the amplitude of a slow rotating current, and watch how
 *     the current responds. If the loss were the ideal step, `|v|` would be exactly `rs*|i| + deadtime_v`; whatever
 *     departs from that is the fade.
 *
 *     Why **rotating**, not a DC d-axis sweep. With a DC command the loss is a relay: the dead time pins the current at
 *     zero until the command exceeds `(4/pi)*deadtime_v`, so the sweep never enters the region where a fade is visible
 *     -- the first version of this estimator did exactly that and came back `valid == false` on a switching plant,
 *     correctly. A rotating current has an *amplitude* rather than a sign, and its loss is the fundamental
 *     `(4/pi)*deadtime_v*clamp(|I|/If)`: again one equation per sample and one unknown, but in the region the machine
 *     actually operates in.
 *
 * @brief [zh] 从锁定转子的旋转电流扫描里拟合一个参数，把死区的**淡出宽度**辨识出来。
 *
 *     存在的理由：这个宽度是补偿级唯一需要的数，也是唯一没人能从铭牌算出来的数 （`predicted_ripple_a()` 是差一个数量级
 *     的上界）。但它**可测**，而且很便宜： 定住转子、扫一个缓慢旋转电流的幅值、看电流怎么响应。若损失是理想阶跃， `|v|`
 *     就该正好是 `rs·|i| + deadtime_v`；偏离它的那部分就是淡出。
 *
 *     为什么是**旋转**而不是 d 轴直流扫描。直流指令下损失是一个继电器：死区把电流 钉在零，直到指令超过
 *     `(4/π)·deadtime_v`，所以扫描根本没进到能看见淡出的 区域——本估计器的第一版正是这么做的，在开关 plant 上返回 `valid
 *     == false`， 而且是对的。旋转电流有的是**幅值**而不是符号，它的损失是基波 `(4/π)·deadtime_v·clamp(|I|/If)`：仍然每
 *     样本一个方程、一个未知数，但落在 机器真正工作的区域里。
 *
 * @note [en] Implementation: a fixed grid of candidate widths, each keeping a running sum of squared residuals, so the
 *            estimator is *streaming* -- it never stores samples and never allocates. The winner is refined by a
 *            parabola through its two neighbours in log space. Deterministic: the same samples give bit-identical
 *            results. What it needs and what it cannot do. It needs `rs`; a wrong `rs` is absorbed into `If` (a bias
 *            that `residual_v` reports). It needs currents *above* the noise floor, so it cannot resolve a fade
 *            narrower than the current sensor's noise -- in that case `valid` stays false rather than inventing a
 *            number, which is the honest outcome: use the `sign()` step there.
 *
 * @note [zh] 实现：一组固定的候选宽度，各自累加残差平方和，因此估计器是**流式**的——不存样本、不分配。 胜出者用对数空间
 *            里相邻三点做抛物线细化。确定性：同样的样本给出逐位相同的结果。
 *
 *     它需要什么、做不到什么。它需要 `rs`；错的 `rs` 会被吸收进 `If`（这个偏差由 `residual_v` 报告）。 它需要高于噪声地
 *     板的电流，所以分辨不出比电流传感器噪声更窄的淡出——那种情况下 `valid` 保持 false， 而不是编一个数出来，这才是诚实
 *     的结果：那里就用 `sign()` 阶跃。
 *
 * @context [en] Commissioning, host- or target-side, once per board.
 * @context [zh] 整定阶段，主/目标侧，每块板一次。
 * @since 0.1.0
 * @exporter
 */
class DeadTimeFadeEstimator final {
  public:
    /**
   * @brief [en] What the fit needs to know.
   * @brief [zh] 拟合需要知道的东西。
   * @since 0.1.0
   */
    struct Config final {
        float rs;         ///< [en] stator resistance in ohm. [zh] 定子电阻，Ω。
        float deadtime_v; ///< [en] (t_dead/T_pwm)*v_bus in V. [zh]
                          ///< (t_dead/T_pwm)*v_bus，V。
        float i_limit_a;  ///< [en] samples with |i| above this are ignored, A. [zh]
                          ///< |i| 超过此值的采样被忽略，A。
    };

    /**
   * @brief [en] Outcome of the fit.
   * @brief [zh] 拟合结果。
   * @since 0.1.0
   */
    struct Result final {
        float fade_a;     ///< [en] the width, A; meaningless unless `valid`. [zh]
                          ///< 宽度，A；`valid` 为假时无意义。
        float residual_v; ///< [en] rms of what the model could not explain, V. [zh]
                          ///< 模型解释不掉的部分的均方根，V。
        bool valid;       ///< [en] enough usable samples, and a fit that explains the
                          ///< data. [zh] 样本够且拟合解释了数据。
    };

    /// [en] Candidate widths, log-spaced over four decades. [zh]
    /// 候选宽度，跨四个数量级对数分布。
    static constexpr int kCandidates = 25;

    /// [en] Fewer usable samples than this is not an identification. [zh]
    /// 可用样本少于这个数就不算辨识。
    static constexpr int kMinSamples = 8;

    explicit DeadTimeFadeEstimator(const Config& config) noexcept : config_(config) {}

    /**
   * @brief [en] Feed one settled operating point.
   * @brief [zh] 送入一个已稳定的工作点。
   * @param v_magnitude [en] the *amplitude* of the rotating voltage vector, V; this is the projection of the command
   *                         onto the current, `v . i/|i|`, which is the component the loss acts on (the `wL` term is
   *                         perpendicular to it and cancels exactly).
   * @param v_magnitude [zh] 旋转电压矢量的**幅值**，V；这里是命令在电流方向 上的投影 `v·i/|i|`，即损失作用的那个分量（
   *                         `ωL` 项垂直于它，正好抵消）。
   * @param i_magnitude [en] the *fundamental* amplitude of the current, A, with the switching ripple removed -- a
   *                         lock-in at the sweep frequency, not a peak of the instantaneous samples, or the ripple is
   *                         mistaken for signal.
   * @param i_magnitude [zh] 电流的**基波**幅值，A，已去掉开关纹波——要用在扫描频率上的锁相， 而不是瞬时采样的峰值，否则
   *                         纹波会被当成信号。
   * @return [en] true if the sample was used.
   * @return [zh] 该采样是否被采用。
   * @note [en] "Settled" matters: the model has no `di/dt` term, so a sample taken during a transient biases the fit.
   *            Only the caller knows when it settled.
   * @note [zh] "已稳定"很要紧：模型没有 `di/dt` 项，暂态里取的样本会偏。什么时候稳了，只有调用方知道。
   * @since 0.1.0
   */
    auto add(float v_magnitude, float i_magnitude) noexcept -> bool {
        if (!usable()) {
            return false;
        }
        const float current = std::fabs(i_magnitude);
        if (!std::isfinite(v_magnitude) || !std::isfinite(current)) {
            return false;
        }
        if ((config_.i_limit_a > 0.0F) && (std::fabs(current) > config_.i_limit_a)) {
            return false;
        }
        for (int k = 0; k < kCandidates; ++k) {
            const float model =
                static_cast<float>(config_.rs * current) + loss_magnitude(current, candidate(k), config_.deadtime_v);
            const float residual = static_cast<float>(v_magnitude - model);
            cost_[k] = static_cast<float>(cost_[k] + (residual * residual));
        }
        ++samples_;
        return true;
    }

    /// [en] Samples that were actually used. [zh] 实际采用的样本数。
    [[nodiscard]] auto samples() const noexcept -> int { return samples_; }

    /**
   * @brief [en] The best width, and how well the model explained the data.
   * @brief [zh] 最好的宽度，以及模型解释数据的程度。
   * @since 0.1.0
   */
    [[nodiscard]] auto solve() const noexcept -> Result {
        if ((samples_ < kMinSamples) || !usable()) {
            return Result{0.0F, 0.0F, false};
        }
        int best = 0;
        for (int k = 1; k < kCandidates; ++k) {
            if (cost_[k] < cost_[best]) {
                best = k;
            }
        }
        // At the nearest grid point a 1.47x step can sit 16 % off the truth, so the
        // residual worth reporting is the one at the parabola vertex, not at the
        // grid point -- otherwise the grid's resolution is blamed on the model.
        // 最近网格点可以偏离真值 16
        // %，所以值得报告的残差是抛物线顶点处的，而不是网格点处的——否则网格
        // 分辨率会被算到模型头上。
        float cost_at_min = cost_[best];

        // Parabola through the winner and its neighbours, in log space -- that is
        // the space the grid is uniform in, so the offset is a fraction of one grid
        // step.
        // 过胜出点与左右邻居的抛物线，在对数空间里做——网格在那里是均匀的，所以偏移是一个步长的分数。
        float step_fraction = 0.0F;
        if ((best > 0) && (best < (kCandidates - 1))) {
            const float left = cost_[best - 1];
            const float mid = cost_[best];
            const float right = cost_[best + 1];
            const float denominator = (left - (2.0F * mid)) + right;
            if (denominator > 0.0F) {
                step_fraction = (0.5F * (left - right)) / denominator;
                step_fraction = (step_fraction > 0.5F) ? 0.5F : step_fraction;
                step_fraction = (step_fraction < -0.5F) ? -0.5F : step_fraction;
                const float span = left - right;
                const float vertex = mid - ((span * span) / (8.0F * denominator));
                cost_at_min = (vertex > 0.0F) ? vertex : 0.0F;
            }
        }
        const float rms = std::sqrt(cost_at_min / static_cast<float>(samples_));
        const float exponent =
            ((static_cast<float>(best) + step_fraction) / static_cast<float>(kCandidates - 1)) * kDecades;
        const float fade = static_cast<float>(kMinFadeA * std::pow(10.0F, exponent));

        // The fit is worth reporting only if it removed most of the dead-time
        // voltage from the residual.
        // 只有当拟合把死区电压的大部分从残差里拿走时，这个结果才值得报告。
        const bool good = rms < (kMaxResidualFraction * config_.deadtime_v);
        return Result{fade, rms, good};
    }

  private:
    [[nodiscard]] auto usable() const noexcept -> bool {
        return (config_.deadtime_v > 0.0F) && (config_.rs > 0.0F) && std::isfinite(config_.rs);
    }

    /// [en] Candidate `k`, log-spaced. [zh] 第 `k` 个候选，对数分布。
    [[nodiscard]] static auto candidate(int index) noexcept -> float {
        const float fraction = static_cast<float>(index) / static_cast<float>(kCandidates - 1);
        return static_cast<float>(kMinFadeA * std::pow(10.0F, fraction * kDecades));
    }

    /// [en] The dead-time loss seen by a *rotating* current of amplitude `i_mag`:
    /// the fundamental of
    ///      `deadtime_v*sign(i)` is `(4/pi)*deadtime_v` along the current, faded
    ///      by `clamp(i_mag/i_fade)`. [zh] **旋转**电流（幅值
    ///      `i_mag`）看到的死区损失：`deadtime_v·sign(i)` 的基波是沿电流方向的
    ///      `(4/π)·deadtime_v`，再乘上 `clamp(i_mag/i_fade)` 的淡出。
    [[nodiscard]] static auto loss_magnitude(float i_mag, float fade, float deadtime_v) noexcept -> float {
        if (!(fade > 0.0F)) {
            return 0.0F;
        }
        return (static_cast<float>(4.0F / kPi<float>) * deadtime_v) * clamp_unit(i_mag / fade);
    }

    [[nodiscard]] static auto clamp_unit(float value) noexcept -> float {
        if (!(value > -1.0F)) {
            return -1.0F;
        }
        if (!(value < 1.0F)) {
            return 1.0F;
        }
        return value;
    }

    /// [en] The grid: 0.5 mA to 5 A, four decades, 25 points. [zh] 网格：0.5 mA
    /// 到 5 A，四个数量级，25 点。
    static constexpr float kMinFadeA = 5.0e-4F;
    static constexpr float kDecades = 4.0F;
    static constexpr float kMaxResidualFraction = 0.25F;

    Config config_{};
    float cost_[kCandidates]{};
    int samples_{0};
};

} // namespace core

} // namespace foc
