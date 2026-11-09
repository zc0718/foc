// Conan::ImportStart
#pragma once

#include <cmath>
#include <cstdint>

#include <foc/core/types.hpp>

#include "sil_pmsm_plant.hpp"
// Conan::ImportEnd

namespace foc {

namespace sil {


/**
 * @brief [en] The bus as an energy account: a capacitance, the currents that cross it, and the books that have to
 *             balance.
 * @brief [zh] 把母线当成一本能量账：一个电容、几条穿过它的电流，以及必须对得上的账。
 * @details [en]
 *
 *     Until this file existed the bus was a **constant**: `PlantDcLink` returned whatever had been set, and
 *     `DiscreteInverter::begin_period()` latched it. Nothing in the repository ever asked "how much charge left the bus
 *     this period", so nothing could observe the bus sagging, and nothing could observe a brake chopper working. The
 *     missing quantity was not a board, it was a **port**: the inverter knew its pole voltages and the plant knew its
 *     currents, and nobody multiplied them.
 *
 *     Sign convention, stated once because every criterion below depends on it: `i_dc` is positive
 *     **into the inverter** (motoring). The node equation is
 *
 *     C * dv/dt = i_source - i_dc - i_load - i_chopper
 *
 *     so a motoring drive discharges the capacitor and a regenerating one charges it. `i_load` is an auxiliary draw and
 *     `i_chopper` is the brake resistor, both positive-into-the-device. With `capacitance_f <= 0` the bus is an
 *     **ideal source**: the voltage stays where it was put and the books still run, which is what makes the
 *     counterfactual measurable -- the energy that should have moved the voltage shows up as a residual instead.
 *
 * @details [zh] 在这个文件存在之前，母线是一个 **常数** ：`PlantDcLink` 返回被设进去的那个值，
 *               `DiscreteInverter::begin_period()` 把它锁存。仓库里没有任何东西问过「这一拍有多少电荷离开了母线」，于是
 *               母线跌落不可观测，斩波器工不工作也不可观测。缺的不是板子，是一个 **端口** ：逆变器知道自己的极电压，
 *               plant 知道自己的电流，而没有人把它们乘起来。
 *
 *   符号约定只说一次，因为下面每条判据都依赖它：`i_dc` 为正表示 **流入逆变器** （电动）。节点方程是
 *
 *   C * dv/dt = i_source - i_dc - i_load - i_chopper
 *
 *   所以电动时电容放电、回馈时电容充电。`i_load` 是辅助负载电流，`i_chopper` 是制动电阻，两者都是"流入器件为正"。
 *   `capacitance_f <= 0` 时母线是 **理想源** ：电压停在原处，账照记——这正是让反事实可测的原因：本该把电压推动的那份能
 *   量，会以残差的形式露出来。
 * @since 0.1.9
 * @exporter
 */
/**
 * @brief [en] The DC current that a d-q voltage/current pair draws from the bus: `1.5*(v_d*i_d + v_q*i_q) / v_bus`.
 * @brief [zh] 一对 dq 电压/电流从母线抽取的直流电流：`1.5*(v_d*i_d + v_q*i_q) / v_bus`。
 * @param v_dq [en] applied voltage in the rotor frame, V.
 * @param v_dq [zh] 施加在转子坐标系上的电压，V。
 * @param i_dq [en] current at the same instant, A.
 * @param i_dq [zh] 同一时刻的电流，A。
 * @param v_bus [en] bus voltage, V; `<= 0` answers 0 rather than a division.
 * @param v_bus [zh] 母线电压，V；`<= 0` 时答 0 而不是做除法。
 * @return [en] current into the inverter, A, positive when motoring.
 * @return [zh] 流入逆变器的电流，A，电动为正。
 * @note [en] The `3/2` is the amplitude-invariant transform's power factor and not a fudge: this repository's `clarke`
 *            and `park` keep amplitudes, so the three-phase instantaneous power is `1.5*(v_alpha*i_alpha +
 *            v_beta*i_beta)` and the same `3/2` survives the rotation into d-q. A model built on power-invariant
 *            transforms would have no factor at all -- which is why the factor is derived here once instead of being
 *            sprinkled at call sites where it could be forgotten in one of them.
 * @note [zh] `3/2` 是幅值不变型变换的功率系数，不是凑出来的：本仓库的 `clarke` 与 `park` 保持幅值，所以三相瞬时功率是
 *            `1.5*(v_alpha*i_alpha + v_beta*i_beta)`，转到 dq 之后同一个 `3/2` 仍在。用功率不变型变换建模的话根本没有这
 *            个系数——这正是把它在这里推导一次、而不是撒在各调用点的原因：撒出去就会在其中一个点上被忘掉。
 * @since 0.1.9
 * @exporter
 */
inline auto bus_current_a(core::Dq<float> v_dq, core::Dq<float> i_dq, float v_bus) noexcept -> double {
    if (!(v_bus > 0.0F)) {
        return 0.0;
    }
    return (1.5 * ((static_cast<double>(v_dq.d) * static_cast<double>(i_dq.d)) +
                   (static_cast<double>(v_dq.q) * static_cast<double>(i_dq.q)))) /
           static_cast<double>(v_bus);
}


struct BusParams final {
    float capacitance_f;    ///< [en] bus capacitance, F; <= 0 = an ideal source. [zh] 母线电容，F；<= 0 表示理想源。
    float v_initial;        ///< [en] starting voltage, V. [zh] 初始电压，V。
    float load_current_a;   ///< [en] auxiliary draw, A, positive into the load. [zh] 辅助负载电流，A，流入负载为正。
    float source_current_a; ///< [en] supply current, A, positive into the bus. [zh] 供电电流，A，流入母线为正。
    float
        resistor_ohm; ///< [en] brake resistor, ohm; <= 0 = none fitted, and the chopper current stays zero. [zh] 制动电阻，Ω；<= 0 表示没装，斩波电流恒为 0。
};


/**
 * @brief [en] What the bus books say over a run.
 * @brief [zh] 一段运行之后，母线的账面上写着什么。
 * @note [en] `residual_j` is the whole point: it is what the conservation law does **not** explain. With a capacitance
 *            it is the integration error and must be tiny; with an ideal source it is the energy a real capacitor would
 *            have absorbed, and a run that reports `residual_j == 0` against an ideal source has not measured anything
 *            -- it has an account that cannot go wrong, which is the shape this repository keeps finding. `accountable`
 *            says which of the two this run is, so a reader does not have to infer it.
 * @note [zh] `residual_j` 才是重点：它是守恒律 **解释不了** 的那部分。有电容时它是积分误差，必须很小；理想源时它是一个
 *            真实电容本该吸收的能量，而一段对理想源报出 `residual_j == 0` 的运行什么都没有测到——它只有一本 **不可能出错
 *            ** 的账，而这正是本仓库反复发现的形状。`accountable` 直接说明这次运行属于哪一种，读者不必去推断。
 * @since 0.1.9
 * @exporter
 */
struct BusReport final {
    double
        drawn_j; ///< [en] energy into the inverter, integral of `v * i_dc`. [zh] 流入逆变器的能量，`v * i_dc` 的积分。
    double load_j;         ///< [en] energy into the auxiliary load. [zh] 流入辅助负载的能量。
    double chopper_j;      ///< [en] energy into the brake resistor. [zh] 流入制动电阻的能量。
    double source_j;       ///< [en] energy delivered by the supply. [zh] 供电提供的能量。
    double stored_delta_j; ///< [en] change in `0.5 * C * v^2`, J. [zh] `0.5 * C * v^2` 的变化量，J。
    double
        residual_j; ///< [en] `stored_delta + drawn + load + chopper - source`: what the law leaves over. [zh] 守恒律剩下的部分。
    float v_min;         ///< [en] lowest bus voltage seen, V. [zh] 见过的最低母线电压，V。
    float v_max;         ///< [en] highest bus voltage seen, V. [zh] 见过的最高母线电压，V。
    std::uint32_t steps; ///< [en] integration sub-steps booked. [zh] 记账的积分子步数。
    bool
        accountable; ///< [en] a capacitance was given, so the books can be wrong. [zh] 给了电容，因此这本账 **可能** 出错。
};


/**
 * @brief [en] Integrate the bus node equation alongside the plant, and keep the books.
 * @brief [zh] 与 plant 并行积分母线节点方程，并记账。
 * @note [en] Energy is accumulated with the **trapezoid** (the mean of the voltages at the two ends of a sub-step)
 *            while the state itself advances explicitly. That asymmetry is deliberate and it is what makes the residual
 *            a usable number: the two schemes differ by O(h^2), so the conservation check measures the quadrature error
 *            rather than the model, and a defect of any size stands out against it.
 * @note [zh] 能量用 **梯形** 累加（子步两端电压的平均），而状态本身是显式推进的。这个不对称是有意的，也正是让残差可用的
 *            原因：两种格式相差 O(h²)，于是守恒检查量到的是求积误差而不是模型，任何量级的缺陷都会在它上面显出来。
 * @since 0.1.9
 * @exporter
 */
class BusMeter final {
  public:
    BusMeter() noexcept = default;
    explicit BusMeter(const BusParams& params) noexcept { configure(params); }

    /**
     * @brief [en] Set the parameters and zero the books.
     * @brief [zh] 设定参数并清零账面。
     */
    void configure(const BusParams& params) noexcept {
        params_ = params;
        voltage_ = static_cast<double>(params.v_initial);
        v_min_ = params.v_initial;
        v_max_ = params.v_initial;
        drawn_j_ = 0.0;
        load_j_ = 0.0;
        chopper_j_ = 0.0;
        source_j_ = 0.0;
        stored0_ = stored_j();
        steps_ = 0U;
    }

    /**
     * @brief [en] Book one integration sub-step.
     * @brief [zh] 记一个积分子步。
     * @param i_dc_a [en] current into the inverter over this sub-step, A.
     * @param i_dc_a [zh] 本子步流入逆变器的电流，A。
     * @param i_chopper_a [en] current into the brake resistor, A.
     * @param i_chopper_a [zh] 流入制动电阻的电流，A。
     * @param h_s [en] sub-step length, s.
     * @param h_s [zh] 子步长度，s。
     */
    void book(double i_dc_a, double i_chopper_a, double h_s) noexcept {
        if (!(h_s > 0.0)) {
            return;
        }
        const double v_before = voltage_;
        const double net_a = static_cast<double>(params_.source_current_a) - i_dc_a -
                             static_cast<double>(params_.load_current_a) - i_chopper_a;
        if (params_.capacitance_f > 0.0F) {
            voltage_ += (net_a * h_s) / static_cast<double>(params_.capacitance_f);
        }
        // Trapezoid on the energy, so the quadrature error is O(h^2) and the residual stays a *measurement*.
        // 能量用梯形，于是求积误差是 O(h²)，残差才是一个 **测量** 。
        const double v_mean = 0.5 * (v_before + voltage_);
        drawn_j_ += v_mean * i_dc_a * h_s;
        load_j_ += v_mean * static_cast<double>(params_.load_current_a) * h_s;
        chopper_j_ += v_mean * i_chopper_a * h_s;
        source_j_ += v_mean * static_cast<double>(params_.source_current_a) * h_s;
        v_min_ = (voltage_ < v_min_) ? static_cast<float>(voltage_) : v_min_;
        v_max_ = (voltage_ > v_max_) ? static_cast<float>(voltage_) : v_max_;
        ++steps_;
    }

    [[nodiscard]] auto voltage_v() const noexcept -> float { return static_cast<float>(voltage_); }

    /**
     * @brief [en] The current a brake resistor draws at this duty, A.
     * @brief [zh] 制动电阻在这个占空比下抽取的电流，A。
     * @param duty [en] fraction of the period the switch is closed, clamped to [0, 1] here.
     * @param duty [zh] 开关在一个周期里闭合的比例，这里限幅到 [0, 1]。
     * @return [en] `duty * v / r`; 0 when no resistor is fitted.
     * @return [zh] `duty * v / r`；未装电阻时为 0。
     * @note [en] This is the physical law of one switch and one resistor, and it lives here rather than in the chopper
     *            because the chopper does not know the bus voltage -- it is told one. Writing it in two places would
     *            let a policy that reports an average current and a plant that injects a different one disagree
     *            quietly.
     * @note [zh] 这是一个开关加一个电阻的物理律，放在这里而不是斩波器里，因为斩波器不知道母线电压——是别人告诉它的。在两
     *            处各写一遍，会让「策略报告的电流」与「plant 注入的电流」安静地不一致。
     * @since 0.1.9
     */
    [[nodiscard]] auto brake_current_a(float duty) const noexcept -> double {
        if (!(params_.resistor_ohm > 0.0F)) {
            return 0.0;
        }
        // A switch cannot conduct for more than the whole period, and a negative duty is not a thing: clamp rather
        // than propagate, and let the policy's own bounds check be the thing that reports a real error.
        // 开关不可能导通超过整个周期，负占空比也不是东西：在这里限幅而不是把错值传下去，真正的错误由策略自己的边界
        // 检查来报。
        const double d = (duty > 1.0F) ? 1.0 : ((duty > 0.0F) ? static_cast<double>(duty) : 0.0);
        return (d * voltage_) / static_cast<double>(params_.resistor_ohm);
    }

    [[nodiscard]] auto stored_j() const noexcept -> double {
        return (params_.capacitance_f > 0.0F) ? (0.5 * static_cast<double>(params_.capacitance_f) * voltage_ * voltage_)
                                              : 0.0;
    }
    [[nodiscard]] auto params() const noexcept -> const BusParams& { return params_; }
    [[nodiscard]] auto has_capacitance() const noexcept -> bool { return params_.capacitance_f > 0.0F; }

    /** @brief [en] The books, including what they fail to explain. [zh] 账面，以及它解释不了的那部分。 */
    [[nodiscard]] auto report() const noexcept -> BusReport {
        BusReport out{};
        out.drawn_j = drawn_j_;
        out.load_j = load_j_;
        out.chopper_j = chopper_j_;
        out.source_j = source_j_;
        out.stored_delta_j = stored_j() - stored0_;
        out.residual_j = out.stored_delta_j + drawn_j_ + load_j_ + chopper_j_ - source_j_;
        out.v_min = v_min_;
        out.v_max = v_max_;
        out.steps = steps_;
        out.accountable = has_capacitance();
        return out;
    }

  private:
    BusParams params_{};
    double voltage_{0.0};
    double stored0_{0.0};
    double drawn_j_{0.0};
    double load_j_{0.0};
    double chopper_j_{0.0};
    double source_j_{0.0};
    float v_min_{0.0F};
    float v_max_{0.0F};
    std::uint32_t steps_{0U};
};


} // namespace sil

} // namespace foc
