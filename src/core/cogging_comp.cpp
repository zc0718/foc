// Conan::ImportStart
#include <foc/core/cogging_comp.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

namespace {

constexpr float kTwoPi = 6.28318530718F;

/**
 * @brief [en] The torque a table asks for at a mechanical angle, N*m.
 * @brief [zh] 一张表在给定机械角处要求的转矩，N·m。
 * @note [en] The same expression the SIL plants use to evaluate their injection, written here rather than shared
 *            because `core/` may not include a test header -- so the test that wires this compensator to that plant is
 *            what keeps the two honest about the convention, and it asserts the cancellation residue, which is the only
 *            thing that can tell them apart if they drift.
 * @note [zh] 与 SIL 被控对象求值注入时用的同一表达式；这里自带一份而不共享，是因为 `core/` 不能 include 测试头——所以把
 *            本补偿器接到那台 plant 上的用例，就是让两者在约定上保持诚实的东西，它断言抵消后的残差，而若两者漂开，那是
 *            唯一能把它们区分开的量。
 * @attacher
 * @since 0.1.0
 */
auto table_torque_nm(const CoggingCompHarmonic* table, int count, float theta_mech) noexcept -> float {
    float sum = 0.0F;
    for (int k = 0; k < count; ++k) {
        const float angle = (static_cast<float>(table[k].order) * theta_mech) + table[k].phase_rad;
        sum = static_cast<float>(sum + (table[k].amplitude_nm * std::sin(angle)));
    }
    return sum;
}

/**
 * @brief [en] The highest order in the table, which is what sets the ripple frequency.
 * @brief [zh] 表里的最高次数——它决定纹波频率。
 * @attacher
 * @since 0.1.0
 */
auto highest_order(const CoggingCompHarmonic* table, int count) noexcept -> int {
    int order = 0;
    for (int k = 0; k < count; ++k) {
        if (table[k].order > order) {
            order = table[k].order;
        }
    }
    return order;
}

} // namespace

void CoggingCompensation::set_table(const CoggingCompHarmonic* harmonics, int count) noexcept {
    count_ = 0;
    if (harmonics == nullptr || count <= 0) {
        return;
    }
    const int usable = (count < kMaxHarmonics) ? count : kMaxHarmonics;
    for (int k = 0; k < usable; ++k) {
        if (harmonics[k].order == 0U) {
            continue; // a constant torque is a load, not cogging 恒定转矩是负载，不是齿槽
        }
        table_[count_] = harmonics[k];
        ++count_;
    }
}

auto CoggingCompensation::validate() const noexcept -> ErrorCode {
    const MachineParams& m = config_.params;
    if (!std::isfinite(m.psi_f) || !(m.ld > 0.0F) || !(m.lq > 0.0F) || (m.psi_f < 0.0F)) {
        return ErrorCode::Param;
    }
    if (config_.pole_pairs == 0U) {
        return ErrorCode::Param;
    }
    const float from = config_.fade_from_hz;
    const float to = config_.fade_to_hz;
    if (!std::isfinite(from) || !std::isfinite(to) || (from < 0.0F)) {
        return ErrorCode::Param;
    }
    if ((from > 0.0F) && !(to > from)) {
        return ErrorCode::Param;
    }
    return ErrorCode::Ok;
}

auto CoggingCompensation::offset(const CoggingCompInput& in) const noexcept -> CoggingCompOutput {
    CoggingCompOutput out{ErrorCode::Param, 0.0F, 0.0F, 1.0F, false};
    if (validate() != ErrorCode::Ok) {
        return out;
    }
    if (!std::isfinite(in.theta_mech_rad) || !std::isfinite(in.omega_mech_rad_s) || !std::isfinite(in.id_a) ||
        !std::isfinite(in.iq_command_a) || !std::isfinite(in.i_max_a)) {
        return out;
    }
    if (count_ == 0) {
        // Nothing to cancel: not an error, and deliberately not a "gain of zero" either -- there is no
        // table, so there is no decision to report.
        // 没有可抵消的东西：不是错误，也故意不报成「增益为零」——没有表，就没有决定可报。
        return CoggingCompOutput{ErrorCode::Ok, 0.0F, 0.0F, 1.0F, false};
    }

    const MachineParams& m = config_.params;
    // Torque per amp on the q axis, at the `id` actually flowing: `dT/diq = 1.5*p*(psi_f + (ld-lq)*id)`.
    // q 轴每安培的转矩，在 **实际流动的** `id` 处： `dT/diq = 1.5·p·(ψf + (Ld−Lq)·id)`。
    const float kt_eff = 1.5F * static_cast<float>(config_.pole_pairs) * (m.psi_f + ((m.ld - m.lq) * in.id_a));
    // "No torque per amp" has to be judged **against the machine's own scale**, not against an absolute
    // number. The flux-nulling `id` is `-psi_f/(ld-lq)`, so evaluating `kt_eff` there is a *cancellation*
    // and its residue is one rounding of `1.5*p*psi_f` -- measured 1.3e-9 N*m/A on this machine, which is
    // 2.6e-8 of that scale. A bare `1e-9` sits inside the rounding, so whether the refusal fired depended
    // on the compiler: `g++` landed the residue at 8e-10 and refused, `clang` on macOS landed it at 1.1e-9
    // and answered `Ok`. A millionth of the scale is above every rounding and below any real torque per amp
    // by orders of magnitude, so the refusal no longer depends on the last bit of two different orders.
    // 「每安培没有转矩」必须对着 **机器自身的尺度** 判定，而不是一个绝对数。削磁 `id` 就是
    // `-psi_f/(ld-lq)`，于是在那里求 `kt_eff` 是一次 **相消** ，其残差是 `1.5·p·ψf` 的一个舍入——本机上实测
    // 1.3e-9 N·m/A，即该尺度的 2.6e-8。写死的 `1e-9` 就落在舍入里， 于是拒绝是否触发取决于编译器： `g++` 的
    // 残差落在 8e-10 并拒绝了，macOS 上的 `clang` 落在 1.1e-9 并答了 `Ok`。取尺度的百万分之一 既高于任何舍
    // 入、又低于任何真实的每安培转矩好几个数量级，于是这个拒绝不再取决于两种运算顺序的最后一位。
    const float torque_scale = 1.5F * static_cast<float>(config_.pole_pairs) * std::fabs(m.psi_f);
    if (!(std::fabs(kt_eff) > (torque_scale * 1.0e-6F))) {
        // The machine cannot produce torque at this `id` (it is at the flux-nulling current), so no current
        // can cancel a torque here. Refusing is the honest answer; dividing would produce an infinity that
        // the caller would clamp.
        // 机器在该 `id` 处出不了转矩（正处在削磁电流），所以这里没有任何电流能抵消转矩。拒绝是诚实的答案；
        // 相除会产生一个 调用方随后会钳掉的无穷。
        return CoggingCompOutput{ErrorCode::Param, 0.0F, 0.0F, 1.0F, false};
    }

    // The frequency fade, on the highest order in the table because that is the fastest thing being asked
    // for.
    // 频率淡出，按表里的最高次数算，因为那是被要求的最快的成分。
    float gain = 1.0F;
    if (config_.fade_from_hz > 0.0F) {
        const float omega = std::fabs(in.omega_mech_rad_s);
        const float ripple_hz = (static_cast<float>(highest_order(table_, count_)) * omega) / kTwoPi;
        if (ripple_hz >= config_.fade_to_hz) {
            gain = 0.0F;
        } else if (ripple_hz > config_.fade_from_hz) {
            gain = (config_.fade_to_hz - ripple_hz) / (config_.fade_to_hz - config_.fade_from_hz);
        }
    }

    const float cancelled = table_torque_nm(table_, count_, in.theta_mech_rad);
    // The offset is the current that produces `-cancelled`, scaled by the fade. `kt_eff` carries its sign,
    // so a salient machine with `id` beyond the flux-nulling point is compensated with the opposite current
    // -- which is what the physics says, and it is the reason the fade and the sign are both computed
    // rather than taken from `Kt`.
    // 偏置就是产生 `-cancelled` 的电流，再乘上淡出系数。 `kt_eff` 带符号，所以 `id` 越过削磁点的凸极机会用
    // 相反的电流补偿—— 物理就是这么说的，这也是淡出与符号都要计算、而不从 `Kt` 取值的原因。
    const float wanted = ((-cancelled) / kt_eff) * gain;

    // The ceiling applies to the sum, because the sum is what the current loop sees.
    // 上限作用在 **和** 上，因为电流环看到的就是那个和。
    const float total = in.iq_command_a + wanted;
    const float limit = (in.i_max_a > 0.0F) ? in.i_max_a : 0.0F;
    float bounded = total;
    if (total > limit) {
        bounded = limit;
    } else if (total < -limit) {
        bounded = -limit;
    }
    out.status = ErrorCode::Ok;
    out.torque_nm = cancelled;
    out.gain = gain;
    out.iq_a = bounded - in.iq_command_a;
    out.clipped = (bounded != total);
    return out;
}

} // namespace core

} // namespace foc
