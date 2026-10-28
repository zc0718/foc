// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstddef>

#include <foc/core/machine_params.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] One row of a magnetisation map: at `|i|` amperes, these are the inductances and the flux.
 * @brief [zh] 磁化表的一行：在 `|i|` 安培处，这些是电感与磁链。
 * @details [en] A single `Ld`/`Lq` is a statement about one operating point. Interior machines saturate well before
 *               their current limit, so a scale model or a finite-element run gives a table instead, and every consumer
 *               that wants to be right away from the nominal point needs that table.
 * @details [zh] 单个 `Ld`/`Lq` 是关于一个工作点的陈述。内嵌机在远未到电流限幅时就已饱和，所以缩比模型或有限元给出的是一
 *               张表；任何想在标称点之外也正确的使用者都需要这张表。
 * @since 0.1.2
 * @exporter
 */
struct MachineMapPoint final {
    float i_a;      ///< [en] current magnitude the row was measured at, A, >= 0. [zh] 该行的电流幅值，A，>= 0。
    float ld_h;     ///< [en] d-axis inductance, H, > 0. [zh] d 轴电感，H，> 0。
    float lq_h;     ///< [en] q-axis inductance, H, > 0. [zh] q 轴电感，H，> 0。
    float psi_f_wb; ///< [en] rotor flux linkage, Wb, > 0. [zh] 转子磁链，Wb，> 0。
};


/**
 * @brief [en] What a map answers at one current: the inductances and the flux, but not `Rs`.
 * @brief [zh] 表在某个电流处给出的答案：电感与磁链，但不含 `Rs`。
 * @note [en] `Rs` is deliberately absent. It varies with temperature, not with current, so folding it into a current
 *            map would hide the one parameter whose error grows during a run.
 * @note [zh] 刻意不含 `Rs`。它随温度而非电流变化；把它折进电流表会掩盖那个在运行中误差不断增长的参数。
 * @since 0.1.2
 */
struct MachinePoint final {
    float ld_h;
    float lq_h;
    float psi_f_wb;

    /**
     * @brief [en] The same point as a full parameter set, with the caller's `rs`.
     * @brief [zh] 带上调用方的 `rs` 的完整参数集。
     */
    [[nodiscard]] auto with_rs(float rs) const noexcept -> MachineParams {
        return MachineParams{rs, ld_h, lq_h, psi_f_wb};
    }
};


/**
 * @brief [en] A current-dependent machine model, interpolated linearly between measured rows.
 * @brief [zh] 随电流变化的机器模型，在实测行之间线性插值。
 *
 * @details [en] Capacity is static: this lives in the control path, where a table that can grow is a table that can
 *               fail to allocate. Nine rows is enough for a saturation knee plus the ends, and the format keeps the
 *               rows sorted so interpolation is a scan of a handful of floats.
 *
 *     Outside the measured range the answer is **clamped to the end row**, not extrapolated. Extrapolating a saturation
 *     curve past the last measurement is how a model invents behaviour nobody measured.
 *
 * @details [zh] 容量是静态的：它住在控制路径上，而"能增长的表格"就是"可能分配失败的表格"。九行足以描述一个饱和拐点加两
 *               端，格式要求行按电流升序，于是插值只是扫几个 float。
 *
 *     超出实测范围时**钳到端行**，不做外推。把饱和曲线外推到最后一个测点之外，正是模型凭空发明没人测过的行为的做法。
 *
 * @note [en] A one-row map is legal and means "constant, this is all I know" -- the honest representation of a
 *            nameplate parameter set.
 * @note [zh] 单行表是合法的，含义是"它是常数，我只知道这些"——也就是铭牌参数集的诚实表示。
 * @since 0.1.2
 * @exporter
 */
struct MachineMap final {
    static constexpr std::size_t kCapacity = 9U;

    MachineMapPoint rows[kCapacity]{};
    std::size_t count{0U};

    /**
     * @brief [en] Whether this map can be trusted to answer at all.
     * @brief [zh] 这张表是否可信到可以回答。
     * @return [en] true when there is at least one row, every field is finite and positive, and the currents strictly
     *              increase.
     * @return [zh] 至少一行、每个字段有限且为正、电流严格递增时为 true。
     */
    [[nodiscard]] auto valid() const noexcept -> bool {
        if ((count == 0U) || (count > kCapacity)) {
            return false;
        }
        float previous = -1.0F;
        for (std::size_t i = 0U; i < count; ++i) {
            const MachineMapPoint& row = rows[i];
            if (!std::isfinite(row.i_a) || !std::isfinite(row.ld_h) || !std::isfinite(row.lq_h) ||
                !std::isfinite(row.psi_f_wb)) {
                return false;
            }
            if (!(row.ld_h > 0.0F) || !(row.lq_h > 0.0F) || !(row.psi_f_wb > 0.0F) || (row.i_a < 0.0F)) {
                return false;
            }
            if (!(row.i_a > previous)) {
                return false;
            }
            previous = row.i_a;
        }
        return true;
    }

    /**
     * @brief [en] The machine at that current magnitude.
     * @brief [zh] 该电流幅值处的机器。
     * @param i_a [en] current magnitude in A; the sign does not matter, the map is even in current.
     * @param i_a [zh] 电流幅值，A；符号无关，表对电流是偶函数。
     * @return [en] the interpolated point, clamped to the end rows outside the measured range.
     * @return [zh] 插值后的点；超出实测范围时钳到端行。
     * @pre [en] `valid()`; an invalid map returns row 0 unchanged, which the caller can detect by validating first.
     * @pre [zh] 需满足 `valid()`；无效的表原样返回第 0 行，调用方应先校验。
     */
    [[nodiscard]] auto at(float i_a) const noexcept -> MachinePoint {
        const float x = std::fabs(std::isfinite(i_a) ? i_a : 0.0F);
        const MachineMapPoint& first = rows[0];
        if ((count <= 1U) || (x <= first.i_a)) {
            return MachinePoint{first.ld_h, first.lq_h, first.psi_f_wb};
        }
        const MachineMapPoint& last = rows[count - 1U];
        if (x >= last.i_a) {
            return MachinePoint{last.ld_h, last.lq_h, last.psi_f_wb};
        }
        for (std::size_t i = 1U; i < count; ++i) {
            const MachineMapPoint& hi = rows[i];
            if (x > hi.i_a) {
                continue;
            }
            const MachineMapPoint& lo = rows[i - 1U];
            const float span = hi.i_a - lo.i_a;
            const float t = (span > 0.0F) ? ((x - lo.i_a) / span) : 0.0F;
            return MachinePoint{lo.ld_h + (t * (hi.ld_h - lo.ld_h)), lo.lq_h + (t * (hi.lq_h - lo.lq_h)),
                                lo.psi_f_wb + (t * (hi.psi_f_wb - lo.psi_f_wb))};
        }
        // Unreachable for a valid map; the last row is the only honest answer if it happens.
        // 对合法的表不可达；真发生时，最后一行是唯一诚实的答案。
        return MachinePoint{last.ld_h, last.lq_h, last.psi_f_wb};
    }

    /**
     * @brief [en] Build a one-row map from nameplate numbers.
     * @brief [zh] 用铭牌数字构造单行表。
     * @param params [en] the constant parameter set.
     * @param params [zh] 常数参数集。
     * @param i_a [en] the current the numbers are quoted at, A.
     * @param i_a [zh] 这些数字所对应的电流，A。
     * @return [en] a map that answers the same thing everywhere.
     * @return [zh] 在处处给出相同答案的表。
     */
    [[nodiscard]] static auto from_params(const MachineParams& params, float i_a) noexcept -> MachineMap {
        MachineMap map{};
        map.rows[0] = MachineMapPoint{i_a, params.ld, params.lq, params.psi_f};
        map.count = 1U;
        return map;
    }
};

} // namespace core

} // namespace foc
