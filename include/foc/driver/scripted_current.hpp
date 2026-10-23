// Conan::ImportStart
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include <foc/core/types.hpp>
#include <foc/hal/current_sense.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {

/**
 * @brief [en] Reference `foc::hal::CurrentSense` implementation: replays a scripted sequence of
 *        samples. Once the script is exhausted the last sample is held (documented, not undefined).
 *        [zh] `foc::hal::CurrentSense` 的参考实现：回放脚本化的采样序列。脚本用尽后保持最后一个
 *        采样（这是规定行为，不是未定义）。
 *
 * @context [en] Same context as the contract it implements. [zh] 与其实现的契约同上下文。
 * @since 0.1.0
 */
class ScriptedCurrent final : public hal::CurrentSense {
  public:
    /// [en] Maximum number of scripted samples. [zh] 脚本最大采样数。
    static constexpr std::size_t kCapacity = 64U;

    ScriptedCurrent() noexcept = default;

    /**
     * @brief [en] Load a sample sequence (copied, no allocation).
     *        [zh] 载入采样序列（复制，不分配）。
     *
     * @param samples [en] non-NULL array of samples. [zh] 非 NULL 的采样数组。
     * @param count [en] 1..kCapacity. [zh] 1..kCapacity。
     * @return [en] Ok, FOC-style Param on NULL/oversized, NoResource is not used here.
     *         [zh] Ok；NULL 或超容量返回 Param。
     * @since 0.1.0
     */
    auto load(const core::Uvw<float>* samples, std::size_t count) noexcept -> ErrorCode;

    /**
     * @brief [en] Next sample of the script; the last one repeats once exhausted.
     *        [zh] 脚本的下一采样；用尽后重复最后一个。
     * @return [en] phase currents in A. [zh] 三相电流，A。
     * @since 0.1.0
     */
    auto read_uvw() noexcept -> core::Uvw<float> override;

    /**
     * @brief [en] Record that a calibration happened.
     *        [zh] 记录已完成校准。
     * @return [en] Ok, or State when no script is loaded. [zh] Ok；未载入脚本时 State。
     * @since 0.1.0
     */
    auto calibrate_offset() noexcept -> ErrorCode override;

    /**
     * @brief [en] Configured sampling rate.
     *        [zh] 配置的采样率。
     * @since 0.1.0
     */
    auto sample_rate_hz() const noexcept -> float override;

    /**
     * @brief [en] Set the reported sampling rate.
     *        [zh] 设置上报的采样率。
     * @since 0.1.0
     */
    void set_sample_rate_hz(float rate_hz) noexcept;

    /**
     * @brief [en] Drop the script and rewind.
     *        [zh] 卸载脚本并回到起点。
     * @since 0.1.0
     */
    void reset() noexcept;

    /**
     * @brief [en] Whether the script has been fully consumed.
     *        [zh] 脚本是否已全部消费。
     * @since 0.1.0
     */
    [[nodiscard]] auto exhausted() const noexcept -> bool;

    /**
     * @brief [en] Whether calibrate_offset() ran since the last load().
     *        [zh] 自上次 load() 以来是否执行过 calibrate_offset()。
     * @since 0.1.0
     */
    [[nodiscard]] auto calibrated() const noexcept -> bool;

  private:
    std::array<core::Uvw<float>, kCapacity> samples_;
    std::size_t count_{0};
    std::size_t index_{0};
    float rate_hz_{0.0F};
    bool calibrated_{false};
};

} // namespace driver

} // namespace foc
