// Conan::ImportStart
#pragma once
#include <foc/core/filters.hpp>
#include <foc/core/observer/observer_policy.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace detail {


/**
 * @brief [en] Shortest-arc difference, wrapped to (-pi, pi].
 * @brief [zh] 最短弧差，归一化到 (-π, π]。
 * @since 0.1.0
 */
inline auto shortest_arc(float to, float from) noexcept -> float {
    float delta = to - from;
    while (delta <= -kPi<float>) {
        delta += kTwoPi<float>;
    }
    while (delta > kPi<float>) {
        delta -= kTwoPi<float>;
    }
    return delta;
}


/**
 * @brief [en] Wrap to [0, 2*pi).
 * @brief [zh] 归一化到 [0, 2π)。
 * @since 0.1.0
 */
inline auto wrap_full_turn(float angle) noexcept -> float {
    while (angle < 0.0F) {
        angle += kTwoPi<float>;
    }
    while (angle >= kTwoPi<float>) {
        angle -= kTwoPi<float>;
    }
    return angle;
}

} // namespace detail


/**
 * @brief [en] One angle estimate from two observers, each valid in a different speed regime.
 *
 * @brief [zh] 由两个各自在某个速度区间有效的观测器合成的单一角度估计。
 *
 *     Why neither observer alone is enough: a saliency loop measures the magnetic circuit, which does not care about
 *     speed, so it works at standstill and drowns in the fundamental current as speed rises. A back-EMF observer
 *     measures the voltage the rotation generates, so it needs rotation and its accuracy improves with speed -- a flux
 *     integrator with a 30 % Rs error was measured at 2.7 rad of angle error near standstill and 0.02 rad at 400 rad/s
 *     electrical, on the same fixture where the saliency loop went the other way. The two curves cross, and this class
 *     is that crossing. 为什么单用哪个都不够：凸极环测的是磁路，与转速无关，因此静止可用，转速升高后被基频电流淹没。 反
 *     电动势观测器测的是旋转产生的电压，必须有转速，且精度随转速提高——同一台架上，Rs 偏 30% 的磁链 积分器在近静止处实测
 *     角度误差 2.7 rad，400 rad/s 电角速度处 0.02 rad，而凸极环的趋势正好相反。 两条曲线交叉，本类就是这个交叉。
 *
 *     The weight is a linear ramp between two speeds, not a switch, and the band is required to have width. A hard
 *     switch driven by the very speed being estimated is a limit cycle: it switches, the speed reading changes, it
 *     switches back. The band also has to *end before the low-speed observer stops working*, which is a measured
 *     property of the machine, not a universal constant -- hence a configuration field rather than a number inside this
 *     class. 权重是两个转速之间的线性斜坡而不是开关，且要求带宽不为零。用被估计的转速去驱动的硬开关就是一个 极限环：切
 *     换 → 转速读数变化 → 切回来。此外带宽必须**在低速观测器失效之前结束**，而那是被控对象 的实测属性、不是通用常数——所
 *     以它是配置字段，而不是本类里的某个数字。
 *
 * @note [en] The angle blend is a shortest-arc interpolation, never a plain mix of two numbers in [0, 2*pi): the two
 *            estimates sit on opposite sides of the wrap every time one of them crosses zero, and averaging those raw
 *            values produces a jump of a whole turn at exactly the wrong moment. The injection is scaled by `1 -
 *            weight`, so the carrier winds down as the handover completes instead of being paid for forever. At full
 *            handover it is exactly zero. What the handover costs the high-speed half: while the carrier is being wound
 *            down it is still being delivered, and a model-based observer cannot tell it from back-EMF. An open-loop
 *            flux integrator keeps what it is given, so the crossing leaves it biased by roughly one carrier period of
 *            volt-seconds -- measured 0.031 rad at 2 V / 2 kHz, against 0.033 predicted by `inject_v /
 *            (2*pi*inject_hz*psi_f)`. Filtering the voltage handed to the high-speed observer would remove it and cost
 *            far more: a pole low enough to matter at the carrier frequency lags the fundamental by tens of degrees at
 *            speed. The usable answers are to size the carrier for the bias you can afford, or to make the high-speed
 *            half a loop that tracks the flux instead of integrating it open-loop. Where the *speed* noise comes from,
 *            because it decides what the fix is: not the numerical differentiation. A flux-plus-PLL observer was built
 *            and measured against the shipped one, on the same signals, expecting the loop's smooth speed to make the
 *            handover reproducible. It does not: both report the same speed ripple (+-3 rad/s at 20 rad/s electrical,
 *            +-12 at 100..200) and the loop's *angle* is 2-3x worse at speed (0.030 vs 0.011 rad at 400, 0.060 vs 0.020
 *            at 800). The reason is that the ripple is in the flux estimate itself -- the integrator is fed `v - Rs*i`
 *            and swallows the current's switching ripple -- so any estimator of the angle from that flux inherits it,
 *            and the loop's proportional path turns it straight back into speed. A loop on the *same* flux is therefore
 *            not the fix. What would be is an observer that closes a loop on the currents (Luenberger / sliding mode),
 *            so the ripple is rejected before it ever reaches the flux.
 *
 * @note [zh] 角度混合是**最短弧插值**，绝不是把 [0, 2π) 里的两个数字直接按权重相加：只要其中一个跨过零点， 两个估计就分
 *            处环绕的两侧，直接平均会在最不该跳的时刻产生整整一圈的跳变。
 *
 *     注入按 `1 − 权重` 缩放，于是载波随交接完成而收掉，而不是永远付着这份代价。完全交接时恰为零。
 *
 *     交接对高速那一半的代价：载波在收掉的过程中仍在送达，而基于模型的观测器分不清它与反电动势。 开环磁链积分器给什么留
 *     什么，因此穿越会给它留下约一个载波周期伏秒的偏差——实测 2 V / 2 kHz 下 0.031 rad，`inject_v / (2π·inject_hz·ψf)`
 *     预测 0.033。给高速观测器的电压做滤波可以消掉它，代价 却更大：低到能在载波频率上起作用的极点，在高速时会把基频滞后
 *     几十度。可用的答案是按你能承受的 偏差去定容载波。
 *
 *     **转速**噪声的来源，因为它决定了修法是什么：不是数值微分。本库试过并实测过一个 "磁链 + PLL" 观测器，期待环路的平
 *     滑转速能让交接可复现。结果是不行：两者的转速纹波完全相同 （电角速度 20 rad/s 处 ±3 rad/s，100–200 处 ±12），而环
 *     路的**角度**在高速反而差 2–3 倍 （400 处 0.030 对 0.011，800 处 0.060 对 0.020）。原因是纹波本来就在磁链估计里——
 *     积分器的输入是 `v − Rs·i`，它把电流的开关纹波一并吞了进去——于是任何从该磁链导出的角度都继承它，而环路的比例 通道
 *     又把它直接变回转速。所以"对同一条磁链加环"不是修法；修法是让观测器**对电流闭环** （Luenberger / 滑模），在纹波到
 *     达磁链之前就把它压掉。
 *
 * @context [en] Current-loop ISR after construction: noexcept, no allocation.
 * @context [zh] 构造之后的电流环 ISR： noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
template <typename LowSpeed, typename HighSpeed> class BlendedObserver final {
  public:
    static_assert(concepts::is_observer_v<LowSpeed>, "the low-speed slot must satisfy the observer contract");
    static_assert(concepts::is_observer_v<HighSpeed>, "the high-speed slot must satisfy the observer contract");

    /**
     * @brief [en] The handover band, in electrical rad/s.
     * @brief [zh] 交接带宽，电角速度 rad/s。
     * @note [en] `blend_end_rad_s` must be strictly greater than `blend_start_rad_s`: a zero-width band is a hard
     *            switch, which is rejected rather than supported. `handover_lpf_hz` exists because the weight must not
     *            be computed from a raw differentiated angle: measured on the handover fixture, the back-EMF observer's
     *            speed carried a +-12 rad/s ripple around 100-200 rad/s (the `-L*i` term in its flux estimate ripples
     *            with the current), which on a 60 rad/s wide band swings the weight by +-20 % and makes the output jump
     *            between two estimates that are not the same quantity.
     *
     * @note [zh] `blend_end_rad_s` 必须严格大于 `blend_start_rad_s`：零带宽就是硬开关，会被拒绝而不是被 支持。
     *
     *     `handover_lpf_hz` 存在的理由：权重不能由未经平滑的微分角度算出。台架实测， 反电动势观测器的转速在 100–200
     *     rad/s 附近带 ±12 rad/s 的纹波（其磁链估计里的 `−L·i` 项随 电流纹波），在 60 rad/s 宽的带内就会把权重摆动
     *     ±20%，使输出在两个并非同一物理量的估计之间 跳变。
     * @since 0.1.0
     */
    struct Config final {
        float blend_start_rad_s; ///< [en] below this, only the low-speed observer. [zh] 低于此值只用低速观测器。
        float blend_end_rad_s;   ///< [en] above this, only the high-speed observer. [zh] 高于此值只用高速观测器。
        float
            handover_lpf_hz; ///< [en] smoothing of the speed the handover is decided on, Hz, > 0. [zh] 交接判据所用转速的平滑截止，Hz，> 0。
    };

    /**
     * @brief [en] Take both observers and the band.
     * @brief [zh] 取两个观测器与交接带。
     * @param low [en] the observer that works without rotation.
     * @param low [zh] 无需旋转即可工作的观测器。
     * @param high [en] the observer that needs rotation.
     * @param high [zh] 需要旋转才成立的观测器。
     * @param config [en] invalid values put the object in Param status; update() then holds.
     * @param config [zh] 非法值让对象进入 Param 状态；此后 update() 保持。
     * @since 0.1.0
     */
    BlendedObserver(LowSpeed low, HighSpeed high, Config config) noexcept
        : low_(low), high_(high), config_(config), status_(valid(config) ? ErrorCode::Ok : ErrorCode::Param),
          speed_lpf_(), state_{0.0F, 0.0F}, speed_(0.0F), weight_(0.0F), dt_configured_(0.0F) {}

    /**
     * @brief [en] Advance both observers and publish the fused estimate.
     * @brief [zh] 推进两个观测器并发布融合后的估计。
     * @param v_ab [en] applied stator voltage in V.
     * @param v_ab [zh] 施加的定子电压，V。
     * @param i_ab [en] measured stator current in A.
     * @param i_ab [zh] 实测定子电流，A。
     * @param dt_s [en] step length in s; <= 0 is rejected (status Param).
     * @param dt_s [zh] 步长，s；<= 0 被拒绝（状态 Param）。
     * @return [en] fused electrical angle and speed.
     * @return [zh] 融合后的电角度与电角速度。
     * @since 0.1.0
     */
    auto update(Ab<float> v_ab, Ab<float> i_ab, float dt_s) noexcept -> ObsState {
        if (!(dt_s > 0.0F)) {
            status_ = ErrorCode::Param;
            return state_;
        }
        if (status_ == ErrorCode::Param) {
            return state_;
        }

        if (dt_s != dt_configured_) {
            if (speed_lpf_.configure(config_.handover_lpf_hz, dt_s) != ErrorCode::Ok) {
                status_ = ErrorCode::Param;
                return state_;
            }
            dt_configured_ = dt_s;
        }

        const ObsState low = low_.update(v_ab, i_ab, dt_s);
        const ObsState high = high_.update(v_ab, i_ab, dt_s);

        // The weight is decided on the *high-speed* observer's own speed, filtered. Using its speed and
        // not the fused one keeps the decision out of the feedback path: a handover that reacted to its
        // own output could hold itself anywhere. Using the high-speed speed and not the low-speed one
        // matters for the failure that actually happens -- a saliency loop that locks to the mirror
        // reports zero speed, and a weight computed from that would hand back to the broken observer.
        // 权重由**高速**观测器自己的转速经滤波后决定。用它而不是融合转速，使这个决定不在反馈回路里：
        // 对自己输出做出反应的交接可以把自己停在任意位置。用高速而不是低速的转速，则针对真正会发生的失效
        // ——锁到镜像的凸极环报零速，以它算出的权重会把控制权交还给坏掉的那个观测器。
        //
        // Filter first, then rectify. The other order is a trap that was measured here: at standstill the
        // high-speed observer's speed is pure noise around zero, and |noise| has a positive mean (about
        // 0.64 of its amplitude), so rectifying first hands over to a source that has nothing to say --
        // the blend reported 0.30 rad of error at standstill until the order was fixed.
        // 先滤波再取模。反过来是个陷阱，此处实测过：静止时高速观测器的转速是绕零的纯噪声，而 |噪声| 有
        // 正均值（约为幅值的 0.64），先取模就会把控制权交给一个无话可说的源——修正顺序之前，本混合在
        // 静止处报出 0.30 rad 的误差。
        speed_ = std::fabs(speed_lpf_.step(high.omega_e));
        const float speed = speed_;
        const float span = config_.blend_end_rad_s - config_.blend_start_rad_s;
        weight_ = (speed <= config_.blend_start_rad_s)
                      ? 0.0F
                      : ((speed >= config_.blend_end_rad_s) ? 1.0F : ((speed - config_.blend_start_rad_s) / span));

        // A blind observer contributes nothing. Above the start of the band, if the low-speed observer
        // says it has no signal, its share goes to zero and the other one carries the estimate whole --
        // including the case that made this necessary: a carrier scaled down by the handover itself
        // drops out of lock, and mixing in an estimate from an observer that has just declared itself
        // blind is how a measured 0.23 rad got into the output while the flux observer was at 0.03.
        // 看不见的观测器不贡献任何东西。在带起点之上，若低速观测器说自己没有信号，它的份额归零、由另一个
        // 独自承担——包括让这条成为必要的那个情形：被交接自己收掉的载波掉出锁定，而把一个刚宣布自己看不见
        // 的观测器的估计混进来，正是输出里出现实测 0.23 rad 的来源，而那时反电动势观测器只有 0.03。
        if constexpr (concepts::has_locked_v<LowSpeed>) {
            if (!low_.locked() && (speed_ >= config_.blend_start_rad_s)) {
                weight_ = 1.0F;
            }
        }

        const float delta = detail::shortest_arc(high.theta_e, low.theta_e);
        state_.theta_e = detail::wrap_full_turn(low.theta_e + (weight_ * delta));
        state_.omega_e = static_cast<float>(low.omega_e + (weight_ * (high.omega_e - low.omega_e)));

        // While the low-speed observer is fully wound down it is not tracking, so its estimate goes stale
        // -- and a saliency loop only pulls in inside +-pi/2, which a rotor that kept turning has long
        // left. Keep it told where the rotor is, so bringing the carrier back is a resume and not a
        // re-acquisition that can land on the mirror.
        // 低速观测器完全收掉期间并不在跟踪，它的估计会过时——而凸极环只在 ±π/2 内牵引，一直转着的转子
        // 早已离开那个窗口。持续告诉它转子在哪，于是载波回来时是"继续"而不是一次可能落到镜像上的重新捕获。
        if constexpr (concepts::has_seed_angle_v<LowSpeed>) {
            // Two situations, one action: fully dormant, or not yet re-acquired. In both the low-speed
            // observer is not measuring, so leaving it with whatever angle it froze at is how it ends up
            // outside its own +-pi/2 pull-in window and comes back on the mirror. While it *is* locked it
            // is left alone -- then it is the one doing the measuring, and seeding it from a blend it
            // contributes to would be feeding it its own output.
            // 两种情形、一个动作：完全休眠，或尚未重新捕获。两者中低速观测器都没有在测量，若让它停在冻结
            // 的那个角度上，它就会跑到自己的 ±π/2 牵引窗口之外、回来时落到镜像上。它**锁定**时则不动它
            // ——那时它才是测量的那一方，用一个它自己也参与合成的结果去播种，等于喂它自己的输出。
            bool measuring = false;
            if constexpr (concepts::has_locked_v<LowSpeed>) {
                measuring = low_.locked();
            }
            if (!measuring || (weight_ >= 1.0F)) {
                low_.seed_angle(state_.theta_e, state_.omega_e);
            }
        }
        return state_;
    }

    /**
     * @brief [en] The carrier to add to the voltage command, wound down by the handover.
     * @brief [zh] 要叠加到电压指令上的载波，随交接进度收掉。
     * @since 0.1.0
     */
    [[nodiscard]] auto injection_ab() const noexcept -> Ab<float> {
        if constexpr (concepts::has_injection_v<LowSpeed>) {
            if (status_ == ErrorCode::Ok) {
                const Ab<float> v = low_.injection_ab();
                const float scale = static_cast<float>(1.0F - weight_);
                return Ab<float>{static_cast<float>(v.alpha * scale), static_cast<float>(v.beta * scale)};
            }
        }
        return Ab<float>{0.0F, 0.0F};
    }

    /**
     * @brief [en] Forward a flux seed to the high-speed observer when it takes one.
     * @brief [zh] 高速观测器需要播种时转发磁链种子。
     * @since 0.1.0
     */
    template <typename T = HighSpeed> void seed_flux(Ab<float> psi_ab) noexcept {
        if constexpr (concepts::has_seed_flux_v<T>) {
            high_.seed_flux(psi_ab);
        } else {
            static_cast<void>(psi_ab);
        }
    }

    /**
     * @brief [en] Forward updated machine parameters to whichever halves can take them.
     * @brief [zh] 把更新后的机器参数转发给能接受的那一半。
     * @param params [en] the machine as it actually is.
     * @param params [zh] 机器实际的样子。
     * @since 0.1.0
     */
    void apply_params(const MachineParams& params) noexcept {
        if constexpr (concepts::has_apply_params_v<LowSpeed>) {
            low_.apply_params(params);
        }
        if constexpr (concepts::has_apply_params_v<HighSpeed>) {
            high_.apply_params(params);
        }
    }

    /**
     * @brief [en] Clear both observers and the handover state.
     * @brief [zh] 清除两个观测器与交接状态。
     * @since 0.1.0
     */
    void reset() noexcept {
        low_.reset();
        high_.reset();
        speed_lpf_.reset();
        state_ = ObsState{0.0F, 0.0F};
        speed_ = 0.0F;
        weight_ = 0.0F;
        dt_configured_ = 0.0F;
        status_ = valid(config_) ? ErrorCode::Ok : ErrorCode::Param;
    }

    /**
     * @brief [en] Ok, or Param after an invalid band or non-positive dt.
     * @brief [zh] Ok，或非法交接带 / 非正 dt 后的 Param。
     * @since 0.1.0
     */
    [[nodiscard]] auto status() const noexcept -> ErrorCode { return status_; }

    /**
     * @brief [en] Whether the fused estimate currently means anything.
     * @brief [zh] 融合估计当前是否有意义。
     * @note [en] Below the band this is the low-speed observer's own answer, so a machine that cannot answer (no
     *            saliency, no saturation) still reports that honestly. Once the handover is complete it is `true` by
     *            definition: the high-speed observer is in the regime it is valid in, and whether it is right is not
     *            something this class can observe.
     * @note [zh] 带内取低速观测器自己的答案，于是"答不了"的机器（无凸极、无饱和）仍然诚实上报。一旦交接 完成，按定义就
     *            是 `true`：高速观测器处在它有效的工况里，而它是否正确不是本类能观测的事。
     * @since 0.1.0
     */
    [[nodiscard]] auto locked() const noexcept -> bool {
        if (status_ != ErrorCode::Ok) {
            return false;
        }
        // Above the start of the band the high-speed observer is in the regime the band declares it
        // usable in, so that alone is the answer. Checking the *weight* instead would make this flicker
        // at the end of the handover: by then the carrier has been wound down, the low-speed observer
        // honestly reports that it is blind, and a weight of 0.99 would still fall through to it.
        // 到了带起点之上，高速观测器就处在带宽声明它可用的工况里，单凭这一点即有答案。若改看**权重**，
        // 交接末尾就会抖动：那时载波已收掉、低速观测器诚实地报告自己看不见，而 0.99 的权重仍会落到它那里。
        if (speed_ >= config_.blend_start_rad_s) {
            return true;
        }
        if constexpr (concepts::has_locked_v<LowSpeed>) {
            return low_.locked();
        }
        return true;
    }

    /**
     * @brief [en] The fused estimate as last published (diagnostics).
     * @brief [zh] 最近一次发布的融合估计（诊断用）。
     */
    [[nodiscard]] auto state() const noexcept -> ObsState { return state_; }

    /**
     * @brief [en] Current weight of the high-speed observer, 0..1 (diagnostics).
     * @brief [zh] 高速观测器当前权重，0..1（诊断用）。
     */
    [[nodiscard]] auto blend() const noexcept -> float { return weight_; }

    /**
     * @brief [en] The low-speed observer (diagnostics and commissioning).
     * @brief [zh] 低速观测器（诊断与调试用）。
     */
    [[nodiscard]] auto low() noexcept -> LowSpeed& { return low_; }

    /**
     * @brief [en] The high-speed observer (diagnostics).
     * @brief [zh] 高速观测器（诊断用）。
     */
    [[nodiscard]] auto high() noexcept -> HighSpeed& { return high_; }

  private:
    static constexpr auto valid(const Config& config) noexcept -> bool {
        return (config.blend_start_rad_s >= 0.0F) && (config.blend_end_rad_s > config.blend_start_rad_s) &&
               (config.handover_lpf_hz > 0.0F);
    }

    LowSpeed low_;
    HighSpeed high_;
    Config config_;
    ErrorCode status_;
    FirstOrderLowPass speed_lpf_;
    ObsState state_;
    float speed_;  ///< [en] filtered |high-speed speed|, the handover criterion. [zh] 滤波后的高速转速模值，交接判据。
    float weight_; ///< [en] share of the high-speed observer, 0..1. [zh] 高速观测器的占比，0..1。
    float dt_configured_; ///< [en] period the handover filter was built for. [zh] 交接滤波器配置所用的周期。
};


} // namespace core

} // namespace foc
