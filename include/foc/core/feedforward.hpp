// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] Everything a feedforward stage may look at, filled once per current-loop step.
 *        [zh] 前馈阶段可以看的全部输入，每个电流环周期填一次。
 *
 * @context [en] Current-loop ISR, read-only for the policy. [zh] 电流环 ISR；对策略只读。
 * @since 0.1.0
 * @exporter
 */
struct FeedforwardInput final {
    float omega_e_rad_s; ///< [en] electrical angular speed in rad/s. [zh] 电角速度，rad/s。
    Dq<float> i_meas;    ///< [en] measured dq current in A. [zh] 实测 dq 电流，A。
};


/**
 * @brief [en] The default feedforward: contributes nothing, so the current loop is exactly the PI.
 *        [zh] 默认前馈：不贡献任何电压，电流环就是纯 PI。
 *
 * @since 0.1.0
 * @exporter
 */
class NullFeedforward final {
  public:
    /**
     * @brief [en] No feedforward term.
     *        [zh] 无前馈项。
     * @since 0.1.0
     */
    [[nodiscard]] auto generate(const FeedforwardInput&) noexcept -> Dq<float> { return Dq<float>{0.0F, 0.0F}; }
};


/**
 * @brief [en] The two classical voltage feedforwards of a PMSM: axis cross-coupling and back-EMF.
 *        [zh] PMSM 的两项经典电压前馈：轴间交叉耦合与反电动势。
 *
 *        [en] It is applied *after* the current PI, as a summation node. The PI keeps its own-axis
 *        job (resistance and inductance); the feedforward removes what couples the two axes.
 *        Putting the terms inside the PI would force the `step(ref, fb, dt)` contract to carry the
 *        other axis, the speed and the flux -- three inputs it deliberately does not have.
 *        [zh] 它加在电流 PI **之后**的求和节点上。PI 继续负责本轴的电阻与电感，前馈负责剥掉两轴
 *        之间的耦合。若把这几项塞进 PI，`step(ref, fb, dt)` 契约就必须携带另一轴电流、转速与
 *        磁链——这三个输入是它特意没有的。
 *
 *        [en] The measured current is used rather than the reference: what has to be cancelled is
 *        the coupling that actually exists in the machine this step.
 *        [zh] 使用实测电流而非给定：需要抵消的是本步电机里**真实存在**的耦合。
 *
 * @since 0.1.0
 * @exporter
 */
class DecouplingFeedforward final {
  public:
    /**
     * @brief [en] Which terms to include. Kept as a value, not as `#ifdef`s, so one build can
     *        compare the terms against each other in the same SIL run.
     *        [zh] 包含哪些项。它是取值而不是 `#ifdef`，因此同一次 SIL 运行里就能互相比较。
     *
     * @since 0.1.0
     */
    enum class Terms : std::uint8_t {
        Cross = 0,        ///< [en] `-we*Lq*iq` on d and `+we*Ld*id` on q. [zh] d 轴 `-we*Lq*iq`、q 轴 `+we*Ld*id`。
        Bemf = 1,         ///< [en] `+we*psi_f` on q only. [zh] 只在 q 轴加 `+we*psi_f`。
        CrossAndBemf = 2, ///< [en] both, the usual choice. [zh] 两者都加，通常选这个。
    };

    /**
     * @brief [en] The machine parameters the terms are derived from.
     *        [zh] 前馈项所依赖的电机参数。
     *
     * @note [en] A zeroed configuration yields zero feedforward, which is the physically
     *       consistent answer for a machine with no inductance and no flux. Build it with
     *       `foc::feedforward_config(MotorConfig)` so the nameplate is read once instead of retyped.
     *       [zh] 全零配置产生零前馈——对一台没有电感、没有磁链的电机，这正是物理上正确的答案。
     *       请用 `foc::feedforward_config(MotorConfig)` 构造，让铭牌只被读一次而不是被重抄一遍。
     * @since 0.1.0
     * @exporter
     */
    struct Config final {
        float ld;       ///< [en] d-axis inductance in H. [zh] d 轴电感，H。
        float lq;       ///< [en] q-axis inductance in H. [zh] q 轴电感，H。
        float psi_f_wb; ///< [en] rotor flux linkage in Wb. [zh] 转子磁链，Wb。
        Terms terms;    ///< [en] which terms to apply. [zh] 应用哪些项。
    };

    DecouplingFeedforward() noexcept = default;

    explicit DecouplingFeedforward(const Config& config) noexcept;

    /**
     * @brief [en] Voltage to add to the PI output, in V.
     *        [zh] 需要加到 PI 输出上的电压，V。
     *
     * @param in [en] this step's electrical speed and measured current. [zh] 本步的电角速度与实测电流。
     * @return [en] the feedforward voltage in the dq frame. [zh] dq 坐标系下的前馈电压。
     * @since 0.1.0
     */
    [[nodiscard]] auto generate(const FeedforwardInput& in) noexcept -> Dq<float>;

    /**
     * @brief [en] The configuration in force.
     *        [zh] 当前生效的配置。
     * @since 0.1.0
     */
    [[nodiscard]] auto config() const noexcept -> const Config&;

  private:
    Config config_{};
};

} // namespace core

} // namespace foc
