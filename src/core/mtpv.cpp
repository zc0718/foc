// Conan::ImportStart
#include <foc/core/mtpv.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {

/**
 * @brief [en] The `t` that maximises `p*sin t + q*sin 2t`.
 *
 *   Which is the whole of both laws in this file: the ellipse case and the current-circle case differ only in what `p`
 *   and `q` are.
 *
 * @brief [zh] 使 `p·sin t + q·sin 2t` 最大的 `t`。本文件里的两条规律全在这里：椭圆情形与电流圆情形只差 `p`、`q` 取什
 *             么。
 * @param p [en] the coefficient of the fundamental.
 * @param p [zh] 基波系数。
 * @param q [en] the coefficient of the second harmonic.
 * @param q [zh] 二次谐波系数。
 * @return [en] `t` in radians, on the branch the objective itself selects.
 * @return [zh] 弧度制的 `t`，分支由目标函数自己挑。
 * @note [en] Stationarity is `p*cos t + 2*q*cos 2t = 0`, i.e. `4*q*c^2 + p*c - 2*q = 0` in `c = cos t`. The two roots
 *            are one of each sign, and which one is the *maximum* depends on the signs of `p` and `q`, so instead of
 *            reasoning about branches the four candidate points (`c = 0` and both roots, each with `sin t` of both
 *            signs) are evaluated in the objective and the best is taken. A smooth function on a circle has no
 *            boundary, so its maximum is always among them -- and `c = 0` is included so that the degenerate `q -> 0`
 *            machine (no saliency: pure q-axis current) needs no special case, its roots going off to infinity and
 *            being dropped by the range check.
 * @note [zh] 驻点条件是 `p·cos t + 2·q·cos 2t = 0`，即 `c = cos t` 的 `4·q·c² + p·c − 2·q = 0`。两根一正一负，哪一个是
 *            **极大**取决于 `p`、`q` 的符号，所以这里不去推理分支，而是把四个候选点（`c = 0` 与两个根，各自配上 `sin t`
 *            的两种符号）都代进目标函数取最好。圆上的光滑函数没有边界，最大值必在其中——把 `c = 0` 也放进去，是为了让退
 *            化的 `q → 0` 机器（没有凸极：纯 q 轴电流）不需要特例，它的根跑到无穷远，被范围检查丢掉。
 * @attacher
 * @since 0.1.9
 */
auto max_torque_angle(float p, float q) noexcept -> float {
    float best_value = -INFINITY;
    float best_c = 0.0F;
    float best_s = 1.0F;
    const auto consider = [&](float c, float s) noexcept -> void {
        const float value = (p * s) + (2.0F * q * s * c);
        if (value > best_value) {
            best_value = value;
            best_c = c;
            best_s = s;
        }
    };
    // `c = 0` (`t = +-pi/2`) is always a candidate: it is the answer for the degenerate no-saliency machine, and it is
    // the floor the root search falls back to when `q -> 0` sends both roots off the circle. `c = 0`（`t =
    // ±π/2`）永远是候选：它是退化的无凸极机器的答案，也是 `q → 0` 把两个根都送出圆外时根搜索退回的地板。
    consider(0.0F, 1.0F);
    consider(0.0F, -1.0F);
    const float eight_q = 8.0F * q;
    if (eight_q != 0.0F) {
        const float discriminant = std::sqrt((p * p) + (32.0F * q * q));
        for (int root = 0; root < 2; ++root) {
            const float c = (root == 0) ? ((-p + discriminant) / eight_q) : ((-p - discriminant) / eight_q);
            if (std::fabs(c) <= 1.0F) {
                const float s = std::sqrt(std::fmax(0.0F, 1.0F - (c * c)));
                consider(c, s);
                consider(c, -s);
            }
        }
    }
    return std::atan2(best_s, best_c);
}

} // namespace

auto Mtpv::validate() const noexcept -> ErrorCode {
    const MachineParams& m = config_.params;
    if (!std::isfinite(m.rs) || !std::isfinite(m.psi_f) || !(m.ld > 0.0F) || !(m.lq > 0.0F) || (m.psi_f < 0.0F)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.v_max_v) || !(config_.v_max_v > 0.0F)) {
        return ErrorCode::Param;
    }
    if (!std::isfinite(config_.i_max_a) || !(config_.i_max_a > 0.0F)) {
        return ErrorCode::Param;
    }
    if (config_.pole_pairs == 0U) {
        return ErrorCode::Param;
    }
    return ErrorCode::Ok;
}

namespace {

/**
 * @brief [en] Torque from a dq current, in N.m -- the same expression the plant uses, so a comparison between this
 *             module's points and a simulated trajectory means the same thing.
 * @brief [zh] 由 dq 电流算转矩，N·m——与 plant 用的同一表达式，于是本模块的点与仿真轨迹之间的比较是同一件事。
 * @attacher
 * @since 0.1.9
 */
auto torque_nm(const MtpvConfig& config, float id, float iq) noexcept -> float {
    const MachineParams& m = config.params;
    const float saliency = m.ld - m.lq;
    return 1.5F * static_cast<float>(config.pole_pairs) * ((m.psi_f * iq) + (saliency * id * iq));
}

auto make_point(const MtpvConfig& config, float id, float iq, bool voltage_limited) noexcept -> MtpvPoint {
    return MtpvPoint{ErrorCode::Ok, voltage_limited, id, iq, torque_nm(config, id, iq), std::hypot(id, iq)};
}

constexpr MtpvPoint kInvalidPoint{ErrorCode::Param, false, 0.0F, 0.0F, 0.0F, 0.0F};

} // namespace

auto Mtpv::mtpa_point() const noexcept -> MtpvPoint {
    if (validate() != ErrorCode::Ok) {
        return kInvalidPoint;
    }
    const MachineParams& m = config_.params;
    // On the current circle `i_d = i_max*cos t`, `i_q = i_max*sin t`, so the objective is `i_max * [psi_f*sin t +
    // (ld-lq)*i_max/2 * sin 2t]` -- the same shape, with the circle's own `p` and `q`. 在电流圆上 `i_d = i_max·cos
    // t`、`i_q = i_max·sin t`，于是目标是 `i_max · [ψf·sin t + (Ld−Lq)·i_max/2 · sin 2t]`——同一个形状，用圆自己的
    // `p`、`q`。
    const float i_max = config_.i_max_a;
    const float t = max_torque_angle(m.psi_f, 0.5F * (m.ld - m.lq) * i_max);
    return make_point(config_, i_max * std::cos(t), i_max * std::sin(t), false);
}

auto Mtpv::base_speed_rad_s() const noexcept -> float {
    if (validate() != ErrorCode::Ok) {
        return 0.0F;
    }
    const MtpvPoint mtpa = mtpa_point();
    const MachineParams& m = config_.params;
    const float flux_d = (m.ld * mtpa.id_a) + m.psi_f;
    const float flux_q = m.lq * mtpa.iq_a;
    const float flux = std::hypot(flux_d, flux_q);
    if (!(flux > 0.0F)) {
        // A machine whose current-limited maximum-torque point needs no voltage: no base speed to report.
        // 电流受限最大转矩点不需要电压的机器：没有基速可报。
        return INFINITY;
    }
    return config_.v_max_v / flux;
}

auto Mtpv::at_speed(float omega_e_rad_s) const noexcept -> MtpvPoint {
    if (validate() != ErrorCode::Ok || !std::isfinite(omega_e_rad_s)) {
        return kInvalidPoint;
    }
    const MachineParams& m = config_.params;
    const float i_max = config_.i_max_a;
    const float omega = std::fabs(omega_e_rad_s);
    if (!(omega > 0.0F)) {
        // At standstill the voltage ceiling costs nothing: the current limit is the whole answer.
        // 静止时电压上限不花钱：电流限幅就是全部答案。
        return mtpa_point();
    }

    const float v = config_.v_max_v / omega;
    const float v_sq = v * v;
    const float i_sq = i_max * i_max;
    // The feasibility test carries a relative slack because every candidate below is computed from a square root, and a
    // point that is on a constraint to within float noise is on it. The slack is relative, so it cannot hide a real
    // violation at any size of machine.
    // 可行性判定带一个相对松弛：下面每个候选点都是经开方算出来的，而在浮点噪声意义上贴着约束的点就是贴着约束。
    // 松弛是相对的，所以它藏不住任何真实越界，与机器的量级无关。
    constexpr float kSlack = 1.0e-5F;
    // Every candidate is a point, scored by torque; the point returned is the best **feasible** one. Writing it this way
    // instead of as a branch on speed is what keeps the three regions straight: the same code covers current-limited,
    // both-constraints-active, and voltage-limited, and the region boundaries come out of the feasibility tests rather
    // than out of a formula someone has to get right.
    // 每个候选点都是一个点，按转矩打分；返回的是其中**可行**的最好者。这样写而不是按转速分支，是三个区间保持正确的
    // 原因：同一段代码覆盖电流受限、两个约束同时生效、以及电压受限，而区间边界是从可行性判定里自然出来的，不是从一条
    // 必须有人写对的公式里出来的。
    const MtpvPoint mtpa = mtpa_point();
    MtpvPoint best = kInvalidPoint;
    float best_torque = -INFINITY;
    const auto consider = [&](float id, float iq, bool on_voltage_limit) noexcept -> void {
        const float flux_d = (m.ld * id) + m.psi_f;
        const float flux_q = m.lq * iq;
        const bool fits_current = ((id * id) + (iq * iq)) <= (i_sq * (1.0F + kSlack));
        const bool fits_voltage = ((flux_d * flux_d) + (flux_q * flux_q)) <= (v_sq * (1.0F + kSlack));
        if (!fits_current || !fits_voltage) {
            return;
        }
        const float torque = torque_nm(config_, id, iq);
        if (torque > best_torque) {
            best_torque = torque;
            best = MtpvPoint{ErrorCode::Ok, on_voltage_limit, id, iq, torque, std::hypot(id, iq)};
        }
    };

    // 1. The current-limited maximum-torque point. It is the answer exactly while it also fits the ellipse. 1.
    // 电流受限的最大转矩点。只要它同时装得进椭圆，它就是答案。
    consider(mtpa.id_a, mtpa.iq_a, false);

    // 2. The interior maximum of the ellipse -- the MTPV point proper. Above the MTPV entry speed the ellipse is small
    // enough that this fits the current circle, and it is then the best point there is. 2. 椭圆的内部极大值——真正的 MTPV
    // 点。高于 MTPV 进入转速后椭圆小到让它装进电流圆，那时它就是能拿到的最好点。
    const float a = (m.ld - m.lq) / m.ld;
    const float t_ellipse = max_torque_angle(m.psi_f * (1.0F - a), 0.5F * a * v);
    consider(((v * std::cos(t_ellipse)) - m.psi_f) / m.ld, (v * std::sin(t_ellipse)) / m.lq, true);

    // 3. The knee: where the current circle crosses the ellipse. This is the whole of the band between the base speed and
    // the MTPV entry speed, and it is the region a drive actually spends its time in above base speed -- torque falls as
    // `V/Lq` there, before MTPV allows the current to come down. `(Ld*im*c + psi_f)^2 + (Lq*im)^2*(1-c^2) = V^2` is a
    // quadratic in `c = cos t` on the circle. 3. 拐点：电流圆与椭圆的交点。基速到 MTPV
    // 进入转速之间的整条带子就是它，而那是驱动器在基速以上真正待着的区域—— 那里转矩按 `V/Lq` 下降，直到 MTPV
    // 允许电流降下来。 圆上的 `(Ld·im·c + ψf)² + (Lq·im)²·(1−c²) = V²` 是 `c = cos t` 的二次方程。
    const float q_a = ((m.ld * m.ld) - (m.lq * m.lq)) * i_sq;
    const float q_b = 2.0F * m.ld * i_max * m.psi_f;
    const float q_c = (m.psi_f * m.psi_f) + ((m.lq * m.lq) * i_sq) - v_sq;
    if (std::fabs(q_a) > 0.0F) {
        const float discriminant = (q_b * q_b) - (4.0F * q_a * q_c);
        if (discriminant >= 0.0F) {
            const float root = std::sqrt(discriminant);
            for (int branch = 0; branch < 2; ++branch) {
                const float c = (branch == 0) ? ((-q_b + root) / (2.0F * q_a)) : ((-q_b - root) / (2.0F * q_a));
                if (std::fabs(c) <= 1.0F) {
                    const float s = std::sqrt(std::fmax(0.0F, 1.0F - (c * c)));
                    consider(i_max * c, i_max * s, true);
                    consider(i_max * c, -i_max * s, true);
                }
            }
        }
    } else {
        // `ld == lq`: the circle and the ellipse meet where the quadratic's leading coefficient vanished, `c = -q_c/q_b`. `ld
        // == lq`：圆与椭圆相交于二次项系数消失之处，`c = −q_c/q_b`。
        if (std::fabs(q_b) > 0.0F) {
            const float c = -q_c / q_b;
            if (std::fabs(c) <= 1.0F) {
                const float s = std::sqrt(std::fmax(0.0F, 1.0F - (c * c)));
                consider(i_max * c, i_max * s, true);
                consider(i_max * c, -i_max * s, true);
            }
        }
    }

    // 4. The floor: `i_q = 0` on the ellipse, i.e. the current that leaves the drive with no torque but inside its
    // voltage ceiling. It can only win when every other candidate is out of reach, and then zero is the honest answer. 4.
    // 地板：椭圆上 `i_q = 0`，即让驱动器没有转矩但不超压的那个电流。只有其它候选全都够不到时它才会赢，那时零就是
    // 诚实的答案。
    consider((v - m.psi_f) / m.ld, 0.0F, true);
    consider((-v - m.psi_f) / m.ld, 0.0F, true);
    return best;
}

} // namespace core

} // namespace foc
