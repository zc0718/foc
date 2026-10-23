// Conan::ImportStart
#pragma once

#include <cstdint>

#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] Result of one SVPWM evaluation.
 *        [zh] 一次 SVPWM 计算的结果。
 *
 * @since 0.1.0
 */
struct SvpwmResult final {
    Uvw<float> duty;  ///< [en] duty ratios in [0, 1]. [zh] 占空比，范围 [0, 1]。
    bool saturated;   ///< [en] true when over-modulation clamping was applied. [zh] 发生过调制钳位时为 true。
    ErrorCode status; ///< [en] Ok, or Param when the DC link voltage was not positive. [zh] Ok；母线电压非正时 Param。
};

/**
 * @brief [en] Space-vector modulator (symmetric seven-segment equivalent).
 *        [zh] 空间矢量调制器（对称七段式的等价实现）。
 *
 *        Implementation note: the symmetric seven-segment pattern is obtained by centring the
 *        phase voltages (midpoint / min-max injection), which yields exactly the same duties as
 *        the classic sector + T1/T2 formulation without a sector branch. Sector classification is
 *        still exposed as `svpwm_sector()` for telemetry and tests.
 *        实现说明：对称七段式通过把相电压居中（中点/min-max 注入）得到，与经典「扇区 + T1/T2」
 *        公式产生的占空比完全一致，但不需要扇区分支。扇区分类仍以 `svpwm_sector()` 提供，
 *        用于遥测与测试。
 *
 * @context [en] Current-loop ISR: noexcept, no allocation, no division by zero.
 *           [zh] 电流环 ISR：noexcept、不分配、不会除零。
 * @since 0.1.0
 */
class Svpwm final {
  public:
    /**
     * @brief [en] Evaluate the duties of one PWM period.
     *        [zh] 计算一个 PWM 周期的占空比。
     *
     * @param v_ab [en] stationary-frame voltage vector in V (amplitude invariant).
     *             [zh] 静止坐标系电压矢量，单位 V（幅值不变式）。
     * @param v_bus [en] DC link voltage in V.
     *              [zh] 直流母线电压，单位 V。
     * @return [en] duties clamped to [0, 1]; when `v_bus <= 0` the duties are 50% and the status is
     *         Param (no crash, no NaN).
     *         [zh] 限幅到 [0, 1] 的占空比；`v_bus <= 0` 时输出 50% 且状态为 Param（不崩溃、不产生 NaN）。
     * @post [en] Linear modulation up to |v_ab| = v_bus / sqrt(3); beyond that `saturated` is true.
     *       [zh] 在 |v_ab| = v_bus / sqrt(3) 以内为线性调制；超出后 `saturated` 为 true。
     * @since 0.1.0
     */
    [[nodiscard]] static auto modulate(Ab<float> v_ab, float v_bus) noexcept -> SvpwmResult;
};

/**
 * @brief [en] Sector of a stationary voltage vector, 1..6, or 0 for (near) zero.
 *        Sector 1 starts at 0 rad and each sector spans 60 degrees counter-clockwise.
 *        [zh] 静止电压矢量所在扇区，1..6；(近) 零矢量返回 0。扇区 1 从 0 rad 开始，每扇区 60°。
 *
 * @param v_ab [en] stationary-frame vector. [zh] 静止坐标系矢量。
 * @param epsilon [en] magnitude below which the vector counts as zero.
 *                [zh] 小于该幅值时视为零矢量。
 * @return [en] sector index, see above. [zh] 扇区编号，见上。
 * @context [en] Diagnostics only, not used by modulate(). [zh] 仅诊断用途，modulate() 不使用。
 * @since 0.1.0
 */
[[nodiscard]] auto svpwm_sector(Ab<float> v_ab, float epsilon) noexcept -> std::uint8_t;

} // namespace core

} // namespace foc
