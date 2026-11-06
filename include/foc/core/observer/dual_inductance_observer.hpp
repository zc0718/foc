// Conan::ImportStart
#pragma once
#include <cstdint>

#include <foc/core/machine_params.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Dual-inductance flux observer.
 *
 *     The same integrator as `BackEmfObserver`, but the rotor angle is recovered from the saliency instead of from a
 *     single `L`.
 *
 *     The stator flux of a salient machine is a **position-dependent** quantity, not "rotor flux plus `L*i`":
 *
 *     psi = psi_f * e^(j*theta) + L0 * i + L1 * conj(i) * e^(j*2*theta), L0 = (Ld + Lq) / 2, L1 = (Ld - Lq) / 2
 *
 *     `BackEmfObserver` assumes `L1 = 0` and answers `theta = arg(psi - L*i)`, which is **exact only for Ld == Lq**. On
 *     a salient machine the neglected `L1` term is a second harmonic of the electrical angle whose amplitude is
 *     `|L1|*|i|` -- 8.5 % of the rotor flux at 100 A with the Prius pair (`Ld = 0.13 mH`, `Lq = 0.29 mH`, `psi_f =
 *     0.0947 Wb`) -- so the single-`L` answer is wrong by an angle that grows with current, and with it the whole
 *     reason `MtpaReference` may not be paired with a single-`L` observer. This observer keeps the same integrator (`d
 *     psi / dt = v - Rs * i`, with no inductance term at all) and inverts the relation above exactly: with `x =
 *     e^(j*theta)` the relation is a **quadratic in `x`**, so the answer is closed form and needs no iteration, no
 *     allocation and no tuning.
 *
 * @brief [zh] 双电感磁链观测器：与 `BackEmfObserver` 相同的积分器，但转子角从凸极性反解，而不是靠单一 `L` 减掉电流磁
 *
 *     链。
 *
 *     凸极电机的定子磁链是**与位置相关**的量，不是"转子磁链加上 `L·i`"：
 *
 *     psi = psi_f·e^(jθ) + L0·i + L1·conj(i)·e^(j2θ)，L0 = (Ld+Lq)/2，L1 = (Ld−Lq)/2
 *
 *     `BackEmfObserver` 假设 `L1 = 0`，给出 `θ = arg(psi − L·i)`，这只在 `Ld == Lq` 时精确。凸极机上被丢掉的 `L1` 项是
 *     电角度二次谐波，幅值 `|L1|·|i|`——用 Prius 那对电感（`Ld = 0.13 mH`、`Lq = 0.29 mH`、`psi_f = 0.0947 Wb`）在 100 A
 *     时达到转子磁链的 8.5%——所以单 `L` 的答案会有一个随电流增长的角度误差，而"`MtpaReference` 不能与单 `L` 观测器配对"
 *     正是因为这一点。本观测器保留同一个积分器（`dψ/dt = v − Rs·i`，完全不含电感项），并精确反解上式：令 `x = e^(jθ)`，
 *     上式就是关于 `x` 的**二次方程**，因此答案是闭式的，不需要迭代、不分配、也没有可调增益。
 *
 * @note [en] **There is no `method=` here, and that is a decision rather than an omission.** A second way to invert the
 *            relation exists -- minimising the residual of the same expression instead of solving the quadratic -- and
 *            it is kept where it belongs, in the tests, because it returns the same number from the same inputs:
 *            measured agreement **2.384e-07 rad** over 60 (angle, `i_d`, `i_q`) samples. A public switch whose options
 *            differ only in which code path rounds differently is dead weight, the same kind this project removed twice
 *            (a threshold that could never be true, a constant with no consumer). The rule it follows: **ship a
 *            `method=` when the choice changes behaviour, not when it changes the spelling.** The switch this class
 *            would grow is over the *flux source* -- the present pure integrator, which needs a seed and never forgets
 *            a bad one, against a corrected flux that needs no seed and carries a gain -- because that choice changes
 *            the failure modes (start-up from an unknown angle becomes possible; low-speed behaviour and the tuning
 *            surface change with it). That is a different observer with its own measurements, not a flag on this one.
 * @note [zh] **这里没有 `method=`，那是决定而不是遗漏。** 反解同一个关系确实还有第二种做法——不去解二次方程，而是最小化
 *            同一个表达式的残差——而它被留在该在的地方：测试里。因为它从同样的输入给出同一个数：60 组 (角度, `i_d`,
 *            `i_q`) 样本上实测一致到 **2.384e-07 rad**。一个公开开关，若它的选项只在"哪条代码路径舍入得不同"上有别，就
 *            是死重量—— 本项目已经删过两次同类东西（一个永远不可能为真的阈值、一个没有消费者的常量）。它遵循的规则是：
 *            **当选择改变行为时才提供 `method=`，当选择只改变写法时不提供。** 这个类真正会长出来的开关是关于**磁链来源
 *            **的——现在的纯积分器需要种子、且永不忘记坏种子，对上一种不需要种子、带修正增益的磁链——因为那个选择改变失效
 *            模式（从未知角度起动成为可能；低速行为与整定面随之改变）。那是一个需要自己测量表的另一个观测器，不是本类上
 *            的一个标志位。
 * @warning [en] Like the open-loop integrator it generalises, the **absolute** angle needs a seed: a pure integrator
 *               cannot know its integration constant. `seed_flux()` takes the full stator flux (not the rotor flux),
 *               and the two roots of the quadratic are disambiguated by continuity with the previous estimate -- on the
 *               first step after a seed, with the angle the seed implies.
 * @warning [zh] 与它所推广的开环积分器一样，**绝对角**需要播种：纯积分器无从知道积分常数。`seed_flux()` 接收的是完整定
 *               子磁链（不是转子磁链），二次方程的两个根靠"与上一次估计连续"来区分——播种后的第一步，则与种子隐含的角度
 *               连续。
 * @context [en] Current-loop ISR after construction: noexcept, no allocation, no iteration count that depends on data.
 * @context [zh] 构造之后的电流环 ISR：noexcept、不分配、迭代次数不依赖数据。
 * @since 0.1.11
 * @exporter
 */
class DualInductanceObserver final {
  public:
    /**
     * @brief [en] Electrical parameters, with the two inductances kept apart and the rotor flux made explicit.
     * @brief [zh] 电气参数：两个电感分开保留，转子磁链显式给出。
     * @since 0.1.11
     */
    /**
     * @brief [en] Where the flux state comes from -- the one choice on this class that changes *behaviour*.
     * @brief [zh] 磁链状态从哪来——本类上唯一改变**行为**的选择。
     * @note [en] Both options integrate `v - Rs*i`; what differs is whether the state is pulled back onto the flux
     *            relation that the algebra is about to invert. `Integrator` leaves it alone, so a nameplate error in
     *            `Rs` accumulates in the state for as long as the drive runs. `Corrected` closes a loop on the residual
     *            and holds the state on the manifold, at the cost of one gain. This is the shape a `method=` should
     *            have: **Measured both ways**: with the machine's own `Rs` the corrected source is 7.71x better
     *            (0.002580 against 0.019895 rad over 2 s at 200 rad/s), because a state that integrates for forty
     *            thousand steps carries its own numerical error; with `Rs` 20 % high it is 18.4x **worse** (3.080
     *            against 0.167 rad) -- it locks onto a self-consistent state that is wrong, which is what a projection
     *            onto the manifold can do. So the trade is real and the integrator stays the default. same interface,
     *            same inputs, **different failure modes** (see the note on the class for why the other candidate -- a
     *            second inversion -- is not shipped).
     * @note [zh] 两个选项都积分 `v − Rs·i`；差别在于是否把状态拉回到代数即将反解的那个磁链关系上。`Integrator` 不管它，
     *            于是 `Rs` 的铭牌误差会随运行时间在状态里累积。`Corrected` 在残差上闭合一个环、把状态按住在流形上，代价
     *            是一个增益。这才是 `method=` 该有的形状：同接口、同输入、**失效模式不同**（另一个候选——第二种反解—— 为
     *            什么不发布，见类上的注释）。
     * @since 0.1.11
     */
    enum class FluxSource : std::uint8_t {
        /**
         * @brief [en] pure `integral(v - Rs*i)`: exact, and it never forgets a bad seed. @brief
         * @brief [zh] 纯积分：精确，且永不忘记坏种子。
         */
        Integrator = 0,
        /**
         * @brief [en] the same integral, pulled back onto the flux relation each step -- **only from a faithful seed**.
         *             The pull is a nearest-point search on the flux curve, so an angular seed error moves it to the
         *             other branch (measured: 1.5 rad mean error from a 0.30 rad seed error, against 0.20 rad for the
         *             pure integrator).
         * @brief [zh] 同一个积分，但每步被拉回磁链关系上——**只限种子忠实**。这把"拉"是磁链曲线上的一次最近点搜索，所以
         *             角度上的种子误差会把它挪到另一支（实测：0.30 rad 的种子误差下平均误差 1.5 rad，纯积分器是 0.20
         *             rad）。
         */
        Corrected = 1,
    };

    struct Config final {
        float rs;                                  ///< [en] stator resistance in ohm, >= 0. [zh] 定子电阻，Ω，>= 0。
        float ld;                                  ///< [en] d-axis inductance in H, > 0. [zh] d 轴电感，H，> 0。
        float lq;                                  ///< [en] q-axis inductance in H, > 0. [zh] q 轴电感，H，> 0。
        float psi_f;                               ///< [en] rotor flux linkage in Wb, > 0. [zh] 转子磁链，Wb，> 0。
        FluxSource source{FluxSource::Integrator}; ///< [en] see FluxSource. [zh] 见 FluxSource。
        float correction_gain{
            0.2F}; ///< [en] fraction of the residual removed per step, (0, 1]; unused by `Integrator`. [zh] 每步消掉的残差比例，(0, 1]；`Integrator` 不用。
    };

    /**
     * @brief [en] Construct with the machine parameters.
     * @brief [zh] 用电机参数构造。
     * @param config [en] invalid values put the observer in Param status (update then holds 0).
     * @param config [zh] 非法值会让观测器进入 Param 状态（之后 update 保持 0）。
     * @since 0.1.11
     */
    explicit DualInductanceObserver(Config config) noexcept;

    /**
     * @brief [en] Seed the **stator** flux estimate.
     * @brief [zh] 播种**定子**磁链估计。
     * @param psi_ab [en] the full stator flux in Wb at the current instant -- `psi_f*e^(j*theta) + L0*i + L1*conj(i)
     *                    *e^(j*2*theta)`, not the rotor flux alone.
     * @param psi_ab [zh] 当前时刻的**完整**定子磁链，Wb——`psi_f·e^(jθ) + L0·i + L1·conj(i)·e^(j2θ)`，不是单独的转子磁
     *                    链。
     * @note [en] The seed does not need to be exact: it only breaks the tie between the two roots of the quadratic, and
     *            a wrong seed costs a fixed angle offset exactly as it does in the open-loop observer.
     * @note [zh] 种子不必精确：它只用于在二次方程的两个根之间定夺，种子有偏时带来的就是这个固定角度偏移——与开环观测器完
     *            全相同。
     * @since 0.1.11
     */
    void seed_flux(Ab<float> psi_ab) noexcept;

    /**
     * @brief [en] Seed from the rotor angle and the current at that instant, computing the stator flux this class
     *             holds.
     * @brief [zh] 由转子角与该时刻的电流播种，算出本类真正持有的那个定子磁链。
     * @param theta_e [en] electrical rotor angle in rad.
     * @param theta_e [zh] 电角度转子角，rad。
     * @param i_ab [en] stator current in A at the same instant.
     * @param i_ab [zh] 同一时刻的定子电流，A。
     * @note [en] `psi_f*e^(j*theta) + L0*i + L1*conj(i)*e^(j*2*theta)`, with this observer's own `ld`, `lq` and
     *            `psi_f`. It exists so a hand-over does not have to know that this observer's state is the **stator**
     *            flux: a caller that seeds it with the rotor flux alone leaves the current's own flux out of the
     *            integration constant, and a pure integrator never forgets -- the cost would be a permanent angle
     *            offset of about `atan(|L0*i| / psi_f)`, not a transient.
     * @note [zh] `psi_f·e^(jθ) + L0·i + L1·conj(i)·e^(j2θ)`，用本观测器自己的 `ld`、`lq`、`psi_f` 算。它存在的意义是交
     *            接方不必知道本观测器的状态是**定子**磁链：只拿转子磁链播种，会把电流自身的磁链漏在积分常数之外，而纯积
     *            分器不会遗忘——代价是一个约 `atan(|L0·i|/psi_f)` 的**永久**角度偏移，不是暂态。
     * @since 0.1.11
     */
    void seed_at(float theta_e, Ab<float> i_ab) noexcept;

    /**
     * @brief [en] Take an updated machine description.
     * @brief [zh] 接受更新后的机器描述。
     * @param params [en] the machine as it actually is.
     * @param params [zh] 机器实际的样子。
     * @note [en] `ld`, `lq` and `psi_f` all come from `params`; this is the observer that has no "which inductance?"
     *            question to answer, which is the whole difference from `BackEmfObserver::apply_params`.
     * @note [zh] `ld`、`lq`、`psi_f` 都取自 `params`；这个观测器没有"用哪个电感"这个问题要回答，这正是它与
     *            `BackEmfObserver::apply_params` 的全部区别。
     * @since 0.1.11
     */
    void apply_params(const MachineParams& params) noexcept;

    /**
     * @brief [en] Advance the observer by one current-loop step.
     * @brief [zh] 推进一个电流环步。
     * @param v_ab [en] applied stator voltage in V, same sample instant as `i_ab`.
     * @param v_ab [zh] 施加的定子电压，V，与 `i_ab` 同一采样时刻。
     * @param i_ab [en] measured stator current in A.
     * @param i_ab [zh] 实测定子电流，A。
     * @param dt_s [en] step length in s; <= 0 is rejected (status Param, state holds).
     * @param dt_s [zh] 步长，s；<= 0 被拒绝（状态置 Param，输出保持）。
     * @return [en] estimated electrical angle and speed.
     * @return [zh] 估计的电角度与电角速度。
     * @note [en] `v_ab` must come from the *same* sample instant as `i_ab`, for the reason documented on
     *            `BackEmfObserver::update`.
     * @note [zh] `v_ab` 必须与 `i_ab` 取自同一采样时刻，理由见 `BackEmfObserver::update`。
     * @since 0.1.11
     */
    auto update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState;

    /**
     * @brief [en] What the last `update()` reported, without stepping again.
     * @brief [zh] 上一次 `update()` 报出的状态，不推进。
     * @return [en] The last `ObsState`; the initial one before any update.
     * @return [zh] 上一次的 `ObsState`；未推进过时是初始值。
     * @since 0.1.11
     */
    [[nodiscard]] auto state() const noexcept -> ObsState { return state_; }

    /**
     * @brief [en] Clear the integrator, the speed estimate and the root continuity.
     * @brief [zh] 清除积分器、速度估计与选根连续性。
     * @since 0.1.11
     */
    void reset() noexcept;

    /**
     * @brief [en] Ok, or Param after an invalid configuration or a non-positive dt.
     * @brief [zh] Ok，或非法配置 / 非正 dt 后的 Param。
     * @since 0.1.11
     */
    [[nodiscard]] auto status() const noexcept -> ErrorCode;

    /**
     * @brief [en] Current stator flux estimate in Wb (diagnostics).
     * @brief [zh] 当前定子磁链估计，Wb（诊断用）。
     * @since 0.1.11
     */
    [[nodiscard]] auto flux() const noexcept -> Ab<float>;

    /**
     * @brief [en] Which flux source this instance runs, as configured.
     * @brief [zh] 这个实例跑的是哪种磁链来源（按配置）。
     * @since 0.1.11
     */
    [[nodiscard]] auto source() const noexcept -> FluxSource { return config_.source; }

    /**
     * @brief [en] How the last step chose between the two roots of the quadratic.
     * @brief [zh] 上一步在二次方程的两个根之间是如何选择的。
     * @return [en] Angle gap in rad between the two candidates; large means the choice was not close.
     * @return [zh] 两个候选之间的角度间隔，rad；很大意味着这次选择并不"接近"。
     * @note [en] A diagnostic, not part of the estimate: the two roots are the two rotor positions consistent with the
     *            measured flux and current, and the gap between them is how much of the answer came from continuity
     *            rather than from the measurement. Near the q axis (where `i_d ~ 0` and the saliency carries almost no
     *            information) the gap is smallest, which is exactly where a caller should trust the angle least.
     * @note [zh] 这是诊断量，不是估计的一部分：两个根是"与实测磁链、电流一致的两个转子位置"，它们的间隔说明这个答案有多
     *            少来自连续性、多少来自测量。q 轴附近（`i_d ≈ 0`、凸极几乎不携带信息）间隔最小，而那正是调用方最不该相
     *            信这个角的地方。
     * @since 0.1.11
     */
    [[nodiscard]] auto root_gap_rad() const noexcept -> float { return root_gap_rad_; }

    /**
     * @brief [en] How far the chosen root is from being a unit vector: `norm(x) - 1`.
     * @brief [zh] 被选中的根离单位矢量有多远：`norm(x) − 1`。
     * @return [en] Zero when the flux state and the current are consistent with each other, which is the only case in
     *              which a rotor position exists at all.
     * @return [zh] 磁链状态与电流彼此自洽时为零——而那是存在转子位置的唯一情形。
     * @note [en] This is the **only** signal that can tell a caller its seed was wrong. A flux integrator has no
     *            feedback, so a bad seed is permanent (measured: 0.1128 rad, unchanged after 200 steps); but the
     *            algebra still says something, because the quadratic has a root of unit modulus **iff** the state came
     *            from a real rotor position. Measured: 4.4e-16 for a consistent state and 3.4e-02 for a seed built from
     *            another current -- twelve orders of magnitude apart, which is why a threshold can be stated at all.
     * @note [zh] 这是**唯一**能告诉调用方"种子错了"的信号。磁链积分器没有反馈，所以坏种子是永久的（实测 0.1128 rad，
     *            200 步后不变）；但代数仍然说了话——二次方程**当且仅当**状态来自真实转子位置时才有一个模为 1 的根。实
     *            测：自洽状态 4.4e-16，用另一个电流构造的种子 3.4e-02——相差十二个数量级，这才使阈值可以被写下来。
     * @since 0.1.11
     */
    [[nodiscard]] auto radius_error() const noexcept -> float { return radius_error_; }

    /**
     * @brief [en] Whether the last step landed on a state that a rotor position can explain.
     * @brief [zh] 上一步是否落在一个能被转子位置解释的状态上。
     * @return [en] `true` when `abs(radius_error()) <= kRadiusTolerance`, i.e. the estimate is self-consistent.
     * @return [zh] `abs(radius_error()) <= kRadiusTolerance` 时为 `true`，即估计自洽。
     * @note [en] The tolerance is 1e-2 and it is chosen from the measurement, not for looks: on this machine the
     *            consistent case reads 4.4e-16 and an inconsistent seed 3.4e-02, so anything between the two separates
     *            them; 1e-2 keeps a margin of three orders on the good side and half an order on the bad one. It is
     *            *not* an angle tolerance -- it is a statement that the state can be explained at all.
     * @note [zh] 容差取 1e-2，取自测量而不是为了好看：这台机器上自洽情形读 4.4e-16、不一致的种子读 3.4e-02，两者之间任
     *            何值都能分开它们；1e-2 在好的一侧留三个数量级、坏的一侧留半个数量级。它**不是**角度容差——它说的是 "这
     *            个状态到底能不能被解释"。
     * @since 0.1.11
     */
    [[nodiscard]] auto flux_plausible() const noexcept -> bool {
        return !(radius_error_ > kRadiusTolerance) && !(radius_error_ < -kRadiusTolerance);
    }

    /**
     * @brief [en] Boundary of `flux_plausible()`; see the note there for how it was chosen.
     * @brief [zh] `flux_plausible()` 的边界；取值依据见那里的注释。
     * @since 0.1.11
     */
    static constexpr float kRadiusTolerance = 1.0e-2F;

  private:
    Config config_;
    float l0_;      ///< [en] (ld + lq) / 2. [zh] (ld+lq)/2。
    float l1_;      ///< [en] (ld - lq) / 2; zero is the surface-mounted degeneracy. [zh] (ld−lq)/2；零即表贴式退化。
    Ab<float> psi_; ///< [en] stator flux estimate in Wb (the integrator state). [zh] 定子磁链估计，Wb（积分器状态）。
    float seed_theta_; ///< [en] angle the seed implied, for the first root choice. [zh] 种子隐含的角度，用于首次选根。
    ObsState state_;
    ErrorCode status_;
    bool theta_primed_;
    float root_gap_rad_;
    float radius_error_;
};

} // namespace core

} // namespace foc
