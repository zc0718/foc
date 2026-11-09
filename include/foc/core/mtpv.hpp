// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <foc/core/machine_params.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

/*!
 * @ingroup group_core
 * @file mtpv.hpp
 *
 * @brief [en] The maximum-torque-per-volt point, and the two ceilings that bound a machine's operating point.
 * @brief [zh] 每伏最大转矩工作点，以及约束电机工作点的两个上限。
 * @since 0.1.0
 */

namespace foc {

namespace core {

/**
 * @brief [en] The machine and the two ceilings that bound its operating point.
 *
 *   The voltage the drive can put on the terminals, and the current it is allowed to push through them. `pole_pairs` is
 *   here because torque is not the quantity this module is about -- the *locus* is -- but the caller needs torque to
 *   decide whether a point is worth going to, and a torque in N.m requires the pair count. The voltage ceiling is a
 *   magnitude at the terminals, not a bus ratio: whoever builds this from a bus voltage owns the modulation limit, and
 *   this module does not guess it.
 *
 * @brief [zh] 机器，以及限制其工作点的两个上限。
 *
 *   驱动器能加到端子上的电压，和允许推过去的电流。 `pole_pairs` 在这里是因为转矩不是本模块的主题（ **轨迹** 才是），但
 *   调用方需要转矩来判断一个点值不值得去，而 N·m 的转矩需要极对数。电压上限是端子上的幅值而不是母线比例：从母线电压构造
 *   它的人拥有调制上限，本模块不去猜它。
 * @since 0.1.0
 * @exporter
 */
struct MtpvConfig final {
    /**
     * @brief [en] `rs` unused; `ld`/`lq`/`psi_f` used.
     * @brief [zh] 不用 `rs`；用 `ld`/`lq`/`psi_f`。
     */
    MachineParams params;
    /**
     * @brief [en] stator voltage ceiling, V.
     * @brief [zh] 定子电压上限，V。
     */
    float v_max_v;
    /**
     * @brief [en] current ceiling, A.
     * @brief [zh] 电流上限，A。
     */
    float i_max_a;
    /**
     * @brief [en] used for torque only.
     * @brief [zh] 仅用于转矩。
     */
    std::uint8_t pole_pairs;
};

/**
 * @brief [en] One point on the maximum-torque map.
 *
 *   What the drive should hold at this speed, and whether the voltage or the current is what decided it.
 *   `voltage_limited` is the honest half of the answer: below the base speed the optimum is the current-limited MTPA
 *   point and the voltage ceiling is not what stops the drive -- reporting an MTPV point there would be reporting a
 *   number computed from a constraint that is not active, which is how a policy ends up "working" on a machine it does
 *   not describe.
 *
 * @brief [zh] 最大转矩地图上的一个点。
 *
 *   这个转速下驱动器该守住什么，以及是电压还是电流决定了它。 `voltage_limited` 是这个答案里诚实的那一半：基速以下最优点
 *   是被电流限幅的 MTPA 点，电压上限并非拦住驱动器的东西——在那里报一个 MTPV 点，就是用一条 **没有生效** 的约束算出来的
 *   数，而那正是一条策略"在一台它并不描述的机器上也能跑通"的来路。
 * @since 0.1.0
 * @exporter
 */
struct MtpvPoint final {
    /**
     * @brief [en] `Ok`, or why not.
     * @brief [zh] `Ok`，或为什么不是。
     */
    ErrorCode status;
    /**
     * @brief [en] the voltage ellipse binds: see above.
     * @brief [zh] 生效的是电压椭圆：见上。
     */
    bool voltage_limited;
    /**
     * @brief [en] d-axis current, A.
     * @brief [zh] d 轴电流，A。
     */
    float id_a;
    /**
     * @brief [en] q-axis current, A.
     * @brief [zh] q 轴电流，A。
     */
    float iq_a;
    /**
     * @brief [en] the same expression the plant uses, N.m.
     * @brief [zh] 与 plant 同一表达式，N·m。
     */
    float torque_nm;
    /**
     * @brief [en] `|i|`, A: margin to the ceiling.
     * @brief [zh] `|i|`，A：到上限的余量。
     */
    float current_a;
};

/**
 * @brief [en] Maximum torque per volt, and the base speed that says where it begins -- both as closed forms on the
 *             voltage ellipse.
 *
 *   **The law.** With `Rs` dropped (verified against the full equation elsewhere, and the error is `Rs*i` against
 *   `omega*L*i` in the region that matters) the steady-state voltage ceiling is the ellipse
 *   `(Ld*i_d + psi_f)^2 + (Lq*i_q)^2 = V^2` with `V = v_max/w`. Parametrising it by the angle `t` --
 *   `i_d = (V*cos t - psi_f)/Ld`, `i_q = V*sin t/Lq` -- the torque along the ellipse becomes
 *
 *   `T(t) / (1.5*p) = (V/Lq) * [ p_r*sin t + q_r*sin 2t ]`, `p_r = psi_f*Lq/Ld`, `q_r = (Ld-Lq)*V/(2*Ld)`,
 *
 *   **which is a single sinusoid in `t` plus its second harmonic, so its stationary points are a quadratic in `cos t`**
 *   : `4*q_r*c^2 + p_r*c - 2*q_r = 0`. Two roots, one of each sign; the torque-maximising one is the one whose `sin t`
 *   carries the sign that makes the reluctance term add to the magnet term. No iteration, and the **same** quadratic on
 *   the current circle gives MTPA with `p_r = psi_f`, `q_r = (Ld-Lq)*i_max/2` -- the two candidates of the whole
 *   constant-torque map are the same three lines of arithmetic.
 *
 *   The maximum of `T` along the ellipse **is** the tangency point of the torque hyperbola with the ellipse, so the
 *   criterion "the two gradients are parallel" is not an extra check bolted on: it is what the quadratic solves.
 *   `test_mtpv.cpp` verifies it against gradients computed independently, and against a brute-force scan.
 *
 *   **Why this returns MTPA below the base speed instead of an MTPV point.** The ellipse grows without bound as
 *   `w -> 0`, so its interior maximum runs off to `i_d -> -inf`. Below the base speed the current circle is what
 *   actually binds, the optimum is the MTPA point, and this class returns it with `voltage_limited = false`. At exactly
 *   the base speed the ellipse passes through that point, so the two branches agree and the reference is continuous in
 *   speed -- which is the property field weakening also had to have, for the same reason: an integrator upstream of a
 *   step pays for it in overshoot.
 *
 * @brief [zh] 每伏最大转矩，以及说明它从哪开始的基速——两者都是电压椭圆上的闭式解。
 *
 *   **规律。** 丢掉 `Rs`（别处已对着完整方程验证过，且在真正要紧的区域 `Rs·i` 相对 `ω_e·L·i` 很小），稳态电压上限就是椭
 *   圆 `(Ld·i_d + ψf)² + (Lq·i_q)² = V²`， `V = v_max/ω_e`。用角度 `t` 参数化—— `i_d = (V·cos t − ψf)/Ld`、
 *   `i_q = V·sin t/Lq` ——椭圆上的转矩变成
 *
 *   `T(t) / (1.5·p) = (V/Lq) · [ p_r·sin t + q_r·sin 2t ]`， `p_r = ψf·Lq/Ld`、 `q_r = (Ld−Lq)·V/(2·Ld)`，
 *
 *   **即一个正弦项加它的二次谐波，于是极值点是 `cos t` 的二次方程** ： `4·q_r·c² + p_r·c − 2·q_r = 0`。两个根一正一负；
 *   极大值那个根是让 `sin t` 的符号使磁阻项与磁钢项同向相加的那个。无需迭代，而且 **同一个二次方程** 在电流圆上给出的就
 *   是 MTPA，只需 `p_r = ψf`、 `q_r = (Ld−Lq)·i_max/2` ——整张等转矩地图的两个候选点是同三行算术。
 *
 *   椭圆上 `T` 的极大值 **就是** 转矩双曲线与椭圆的切点，所以"两梯度平行"这条判据不是外加的检查：它正是那个二次方程在解
 *   的东西。 `test_mtpv.cpp` 拿独立算出的梯度去验证它，也拿暴力扫描去验证它。
 *
 *   **为什么基速以下返回 MTPA 而不是 MTPV 点。** `ω_e → 0` 时椭圆无界增长，其内部极大值跑到 `i_d → −∞`。基速以下真正生
 *   效的是电流圆，最优点是 MTPA 点，本类返回它并置 `voltage_limited = false`。恰在基速处椭圆穿过那个点，于是两支重合、
 *   参考对转速连续——这也是弱磁当初必须具备的性质，理由相同：积分器上游的一个台阶，要用过冲来偿还。
 *
 * @note [en] `psi_f = 0` is a supported machine (synchronous reluctance): the magnet term vanishes and the quadratic
 *            keeps its reluctance solution at `cos t = 1/sqrt(2)`.
 * @note [zh] `psi_f = 0` 是被支持的（同步磁阻机）：磁钢项消失，二次方程保留它 `cos t = 1/√2` 的磁阻解。
 * @since 0.1.0
 * @exporter
 */
class Mtpv final {
  public:
    Mtpv() noexcept = default;

    explicit Mtpv(const MtpvConfig& config) noexcept : config_(config) {}

    /**
   * @brief [en] Whether the config can produce a point at all.
   * @brief [zh] 配置能不能算出点来。
   * @return [en] `Ok`, or `Param` for a ceiling that is not positive, a negative inductance, a negative flux linkage,
   *              zero pole pairs, or a non-finite number anywhere.
   * @return [zh] `Ok`；上限非正、电感为负、磁链为负、极对数为零、或任何非有限数时为 `Param`。
   * @since 0.1.0
   */
    [[nodiscard]] auto validate() const noexcept -> ErrorCode;

    /**
   * @brief [en] The speed at which the voltage ceiling starts to bind: `v_max` divided by the flux magnitude at the
   *             current-limited maximum-torque point.
   * @brief [zh] 电压上限开始生效的转速： `v_max` 除以电流受限最大转矩点处的磁链幅值。
   * @return [en] electrical rad/s, `INFINITY` when the flux magnitude there is zero (a machine that needs no voltage to
   *              hold torque), `0` when the config is invalid.
   * @return [zh] 电 rad/s；该处磁链幅值为零（不需要电压就能维持转矩的机器）时为 `INFINITY`，配置非法时为 `0`。
   * @note [en] This is *not* `v_max/psi_f`: that is the speed the drive would reach at `i_d = i_q = 0`, and a machine
   *            carrying current needs more voltage than its rotor flux suggests. On the synthetic profile the two
   *            differ by 4x (`2520` against `628` rad/s), because `Lq*i_q` dominates the linkage there.
   * @note [zh] 这 **不是** `v_max/ψf`：那是 `i_d = i_q = 0` 时驱动器能达到的转速，而带电流的机器需要的电压比转子磁链暗
   *            示的更多。合成机上两者差 4 倍（ `2520` 对 `628` rad/s），因为那里 `Lq·i_q` 主导磁链。
   * @since 0.1.0
   */
    [[nodiscard]] auto base_speed_rad_s() const noexcept -> float;

    /**
   * @brief [en] The current-limited maximum-torque point, and the whole answer below the base speed.
   * @brief [zh] 电流受限的最大转矩点，也就是基速以下的全部答案。
   * @since 0.1.0
   */
    [[nodiscard]] auto mtpa_point() const noexcept -> MtpvPoint;

    /**
   * @brief [en] The maximum-torque point at `omega_e_rad_s`: MTPA below the base speed, on the voltage ellipse above
   *             it.
   * @brief [zh] `omega_e_rad_s` 处的最大转矩点：基速以下是 MTPA，以上在电压椭圆上。
   * @param omega_e_rad_s [en] electrical speed, rad/s; a negative value is treated as its magnitude (the map is
   *                           symmetric in the direction of rotation, as it is for every dq law that does not use `Rs`
   *                           ).
   * @param omega_e_rad_s [zh] 电转速，rad/s；负值取其幅值（这张地图对转向对称，凡是不用 `Rs` 的 dq 规律都如此）。
   * @return [en] the point, with `voltage_limited` saying which constraint decided it.
   * @return [zh] 该点， `voltage_limited` 说明是哪个约束决定的。
   * @since 0.1.0
   */
    [[nodiscard]] auto at_speed(float omega_e_rad_s) const noexcept -> MtpvPoint;

  private:
    MtpvConfig config_{};
};

} // namespace core

} // namespace foc
