// Conan::ImportStart
#pragma once
#include <cmath>
#include <foc/core/types.hpp>
#include <foc/motor/motor_config.hpp>
// Conan::ImportEnd

namespace foc {


/**
 * @brief [en] The nameplate restated in the unit a customer thinks in: N*m per amp, plus the two limits a torque
 *             command has to respect.
 *
 * @brief [zh] 把铭牌换算成客户真正使用的单位：每安培多少 N·m，外加一条力矩指令必须遵守的两个限幅。
 *
 *     Why this exists. `set_torque()` takes **amps**, so every caller that wants a torque in N*m has to do the
 *     conversion above the library -- and do it identically, with the same sign convention and the same limit, in every
 *     product. That is the kind of arithmetic that is wrong in exactly one product. This type is that arithmetic once.
 *
 *     `status` is the diagnostic channel: `Ok` when the nameplate can produce a torque at all, `Param` when it cannot
 *     (`kt <= 0` says the machine produces no torque per amp, which is not a machine). A map that failed still has
 *     every field written, so a caller that ignores the status gets zeros rather than an uninitialised limit.
 *
 *     为什么有这个类型。`set_torque()` 收的是**安培**，于是每个想以 N·m 下指令的调用方都得在库之上自己做换算，而且要在
 *     每个产品里做得一模一样——同一个符号约定、同一条限幅。这正是"只在一个产品里错掉"的那类算术。本类型把它只写一次。
 *
 *     `status` 是诊断通道：铭牌能产生力矩时为 `Ok`，不能时为 `Param`（`kt <= 0` 表示这台机器每安培不产生力矩，那不是一
 *     个机器）。失败的地图仍然写满全部字段，因此忽略状态的调用方拿到的是零，而不是未初始化的限幅。
 * @since 0.1.8
 * @exporter
 */
struct TorqueMap {
    ErrorCode status; ///< [en] Ok / Param (nameplate cannot produce a torque). [zh] Ok / Param（铭牌不能产生力矩）。
    /**
     * @brief [en] torque per q-axis amp, N*m/A; this is the nameplate's `kt`.
     * @brief [zh] 每安培 q 轴电流的力矩，N·m/A；即铭牌 `kt`。
     */
    float nm_per_a;
    /**
     * @brief [en] `1.5 * pole_pairs * (ld - lq)`, N*m/A^2: the saliency term that makes the torque a function of `id`
     *             as well as `iq`.
     * @brief [zh] `1.5·极对数·(ld − lq)`，N·m/A²：使力矩同时依赖 `id` 与 `iq` 的凸极项。
     */
    float reluctance;
    float i_limit_a; ///< [en] per-axis current limit the command is clamped to. [zh] 指令被钳制到的每轴电流限幅。
    float min_nm;    ///< [en] lower torque limit, N*m (equal to max for a one-way command). [zh] 力矩下限，N·m。
    float max_nm;    ///< [en] upper torque limit, N*m. [zh] 力矩上限，N·m。
};


/**
 * @brief [en] What a torque command actually became.
 *
 * @brief [zh] 一条力矩指令实际变成了什么。
 *
 *     `limited` is the reason this is a struct rather than a number: a clipped command and an obeyed one differ in
 *     nothing else, and a caller that cannot tell them apart will report a tensioner as holding when it is saturating.
 *     这条指令是否被裁剪，正是这里用结构体而不是一个数的原因：被裁剪的指令与被执行的指令在其他方面毫无差别，分不清二者
 *     的调用方会在张力控制已经饱和时报"正在保持"。
 * @since 0.1.8
 * @exporter
 */
struct TorqueCommand {
    ErrorCode status; ///< [en] Ok / Param (the map is unusable). [zh] Ok / Param（地图不可用）。
    float torque_nm;  ///< [en] the torque actually commanded after both limits, N*m. [zh] 双侧限幅之后实际下发的力矩。
    float iq_a;       ///< [en] the q-axis current that delivers it, A. [zh] 交付该力矩的 q 轴电流，A。
    bool limited;     ///< [en] true when the request was clipped. [zh] 请求被裁剪时为真。
};


/**
 * @brief [en] Build the torque map from the nameplate, with the two limits the caller will work inside.
 * @brief [zh] 由铭牌建立力矩地图，并给出调用方将在其中工作的两个限幅。
 * @param config [en] the machine; `kt` and `pole_pairs` must describe a machine that produces torque, and the current
 *                    limit must be positive.
 * @param config [zh] 机器；`kt` 与 `pole_pairs` 必须描述一台能产生力矩的机器，且电流限幅必须为正。
 * @param min_nm [en] lower torque limit, N*m. Pass 0 with a positive `max_nm` for a **one-way** command: a tensioner or
 *                    a hold that must never pull backwards.
 * @param min_nm [zh] 力矩下限，N·m。与正的 `max_nm` 一起传 0 即**单向**指令：张力控制或保持，绝不允许反向拉。
 * @param max_nm [en] upper torque limit, N*m; must be >= `min_nm`.
 * @param max_nm [zh] 力矩上限，N·m；必须 >= `min_nm`。
 * @return [en] the map; every field is written even when the status is `Param`.
 * @return [zh] 地图；即使状态为 `Param`，每个字段也被写入。
 * @note [en] The current limit still applies on top of the torque limits: the map is clamped to whichever is smaller,
 *            `min/max_nm` or `kt * i_limit_a`. A torque limit wider than the inverter can deliver would otherwise be a
 *            promise the map cannot keep.
 * @note [zh] 电流限幅仍然叠加在力矩限幅之上：地图被钳到 `min/max_nm` 与 `kt·i_limit_a` 中较小的那个。否则一个比逆变器能
 *            力更宽的力矩限幅，就是地图兑现不了的承诺。
 * @since 0.1.8
 * @exporter
 */
inline auto torque_map(const motor::MotorConfig& config, float min_nm, float max_nm) noexcept -> TorqueMap {
    TorqueMap out{};
    out.status = ErrorCode::Param;
    out.nm_per_a = 0.0F;
    out.reluctance = 0.0F;
    out.i_limit_a = 0.0F;
    out.min_nm = 0.0F;
    out.max_nm = 0.0F;

    if ((config.pole_pairs == 0U) || !(config.kt > 0.0F) || !(config.i_max_a > 0.0F) || !std::isfinite(min_nm) ||
        !std::isfinite(max_nm) || (max_nm < min_nm)) {
        return out;
    }

    out.nm_per_a = config.kt;
    out.reluctance = 1.5F * static_cast<float>(config.pole_pairs) * (config.ld - config.lq);
    out.i_limit_a = config.i_max_a;

    // What the inverter can actually deliver, then the caller's limits on top of it.
    // 逆变器真正能交付的，再把调用方的限幅叠加其上。
    const float deliverable = config.kt * config.i_max_a;
    out.min_nm = (min_nm > -deliverable) ? min_nm : -deliverable;
    out.max_nm = (max_nm < deliverable) ? max_nm : deliverable;
    out.status = ErrorCode::Ok;
    return out;
}


/**
 * @brief [en] The torque a measured (or commanded) dq current pair is producing: `kt*iq + 1.5*pp*(ld-lq)*id*iq`.
 *
 * @brief [zh] 一对实测（或指令）dq 电流正在产生的力矩：`kt·iq + 1.5·pp·(ld − lq)·id·iq`。
 *
 *     This is the same expression the SIL plant integrates, which is what makes it checkable against the mechanics:
 *     comparing an estimate derived from the currents with one derived from `J*dw/dt + B*w` compares two independent
 *     measurements of one quantity rather than two restatements of one formula. 这与 SIL plant 积分的是同一个表达式，也
 *     正因如此它可以对着力学被检验：由电流得到的估计与由 `J·dω/dt + B·ω` 得到的估计相比，是两个独立测量相比，而不是同一
 *     个公式的两次复述。
 *
 * @param map [en] a map with status `Ok`; an unusable map estimates 0.
 * @param map [zh] 状态为 `Ok` 的地图；不可用的地图估计出 0。
 * @param id_a [en] d-axis current, A.
 * @param id_a [zh] d 轴电流，A。
 * @param iq_a [en] q-axis current, A.
 * @param iq_a [zh] q 轴电流，A。
 * @return [en] torque in N*m.
 * @return [zh] 力矩，N·m。
 * @since 0.1.8
 * @exporter
 */
inline auto torque_estimate(const TorqueMap& map, float id_a, float iq_a) noexcept -> float {
    if (map.status != ErrorCode::Ok) {
        return 0.0F;
    }
    return (map.nm_per_a * iq_a) + (map.reluctance * id_a * iq_a);
}


/**
 * @brief [en] Turn a requested torque into the current that delivers it, after both limits.
 *
 * @brief [zh] 把请求的力矩变成交付它的电流，并施加双侧限幅。
 *
 *     The inverse is exact, not approximate: with a known `id`, `iq = T / (kt + 1.5*pp*(ld-lq)*id)`. At `id = 0` --
 *     what the loop commands in torque mode -- that is the familiar `T / kt`. The closed form matters because the
 *     estimate above accounts for the reluctance term, and an inverse that ignored it would make a commanded torque and
 *     a measured torque disagree by exactly the term the estimate includes. 逆运算是精确的、不是近似的：`id` 已知时 `iq
 *     = T / (kt + 1.5·pp·(ld−lq)·id)`。在 `id = 0`——力矩模式下环路所下发的那个值——它就是熟悉的 `T / kt`。闭式解之所以重
 *     要，是因为上面的估计包含凸极项，而忽略它的逆运算会让"指令力矩"与 "实测力矩"恰好相差估计里包含的那一项。
 *
 * @param map [en] a map with status `Ok`.
 * @param map [zh] 状态为 `Ok` 的地图。
 * @param requested_nm [en] the torque asked for, N*m; clipped to `[min_nm, max_nm]`.
 * @param requested_nm [zh] 请求的力矩，N·m；被钳到 `[min_nm, max_nm]`。
 * @param id_a [en] the d-axis current the command will be applied at, A.
 * @param id_a [zh] 指令将在其上下发的 d 轴电流，A。
 * @return [en] the command; `limited` says whether anything was clipped.
 * @return [zh] 指令；`limited` 表示是否有内容被裁剪。
 * @since 0.1.8
 * @exporter
 */
inline auto torque_command(const TorqueMap& map, float requested_nm, float id_a) noexcept -> TorqueCommand {
    TorqueCommand out{};
    out.status = map.status;
    out.torque_nm = 0.0F;
    out.iq_a = 0.0F;
    out.limited = false;
    if (map.status != ErrorCode::Ok) {
        return out;
    }

    float wanted = requested_nm;
    if (wanted > map.max_nm) {
        wanted = map.max_nm;
        out.limited = true;
    } else if (wanted < map.min_nm) {
        wanted = map.min_nm;
        out.limited = true;
    }

    const float slope = map.nm_per_a + (map.reluctance * id_a);
    if (!(std::fabs(slope) > 1.0e-9F)) {
        // A d-axis current that cancels the torque constant leaves no current able to deliver this torque. That is a
        // refusal, not a clamp: clamping would report a torque that cannot be produced.
        // 抵消了转矩常数的 d 轴电流，没有任何电流能交付这个力矩。这是拒绝而不是钳制：钳制会报出一个产生不出来的力矩。
        out.status = ErrorCode::Param;
        return out;
    }

    out.torque_nm = wanted;
    out.iq_a = wanted / slope;
    // The torque limits were already narrowed by what the inverter can deliver, so this second clamp only catches
    // rounding; it is here because `iq` is what leaves the library and it must never exceed the stated limit.
    // 力矩限幅已按逆变器能力收窄过，因此这第二道钳制只兜住舍入；它存在是因为离开库的是 `iq`，而它绝不能超过写明的限幅。
    if (out.iq_a > map.i_limit_a) {
        out.iq_a = map.i_limit_a;
        out.torque_nm = torque_estimate(map, id_a, out.iq_a);
        out.limited = true;
    } else if (out.iq_a < -map.i_limit_a) {
        out.iq_a = -map.i_limit_a;
        out.torque_nm = torque_estimate(map, id_a, out.iq_a);
        out.limited = true;
    }
    return out;
}


} // namespace foc
