// Conan::ImportStart
#pragma once
#include <array>
#include <cstddef>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] First-order low-pass filter, forward Euler discretisation. `alpha = 2*pi*fc*dt / (1 + 2*pi*fc*dt)`; one
 *             multiply-add per step.
 * @brief [zh] 一阶低通滤波器，前向欧拉离散化。 `alpha = 2*pi*fc*dt / (1 + 2*pi*fc*dt)`；每步一次乘加。
 * @context [en] Current-loop ISR after configure(): noexcept, no allocation.
 * @context [zh] configure() 之后的电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
class FirstOrderLowPass final {
  public:
    FirstOrderLowPass() noexcept = default;

    /**
     * @brief [en] Configure the cutoff frequency for a given control period.
     * @brief [zh] 按给定控制周期配置截止频率。
     * @param cutoff_hz [en] cutoff frequency in Hz, must satisfy 0 < fc < 1/(2*dt).
     * @param cutoff_hz [zh] 截止频率，单位 Hz，必须满足 0 < fc < 1/(2*dt)。
     * @param dt_s [en] control period in s, must be > 0.
     * @param dt_s [zh] 控制周期，单位 s，必须 > 0。
     * @return [en] Ok, or Param when the cutoff is above Nyquist or the period is not positive.
     * @return [zh] Ok；截止频率超过 Nyquist 或周期非正时返回 Param。
     * @since 0.1.0
     */
    auto configure(float cutoff_hz, float dt_s) noexcept -> ErrorCode;

    /**
     * @brief [en] Feed one sample.
     * @brief [zh] 输入一个采样。
     * @param input [en] new sample.
     * @param input [zh] 新采样。
     * @return [en] filtered value; returns the input unchanged while unconfigured.
     * @return [zh] 滤波后的值；未配置时原样返回输入。
     * @context [en] Current-loop ISR.
     * @context [zh] 电流环 ISR。
     * @since 0.1.0
     */
    auto step(float input) noexcept -> float;

    /**
     * @brief [en] Clear the internal state (next step re-seeds from the input).
     * @brief [zh] 清除内部状态（下一步用输入重新播种）。
     * @since 0.1.0
     */
    void reset() noexcept;

    /**
     * @brief [en] Whether configure() succeeded.
     * @brief [zh] configure() 是否成功。
     * @since 0.1.0
     */
    [[nodiscard]] auto configured() const noexcept -> bool;

  private:
    float alpha_{0.0F};
    float state_{0.0F};
    bool configured_{false};
    bool primed_{false};
};


/**
 * @brief [en] Fixed-capacity moving average. Capacity is part of the type: no allocation, no growth, deterministic
 *             cost. Uses `std::array` (ADR-001 allows fixed-capacity `std::array`).
 * @brief [zh] 固定容量滑动平均。容量是类型的一部分：不分配、不扩容、代价确定。 用 `std::array`（ADR-001 明确允许固定容
 *             量 `std::array`）。
 * @tparam T [en] sample type.
 * @tparam T [zh] 采样类型。
 * @tparam N [en] window length, must be > 0.
 * @tparam N [zh] 窗口长度，必须 > 0。
 * @context [en] Current-loop ISR: noexcept, no allocation.
 * @context [zh] 电流环 ISR：noexcept、不分配。
 * @since 0.1.0
 * @exporter
 */
template <typename T, std::size_t N> class MovingAverage final {
    static_assert(N > 0, "MovingAverage needs a non-empty window");

  public:
    /**
     * @brief [en] Feed one sample and return the current average.
     * @brief [zh] 输入一个采样并返回当前平均值。
     * @param sample [en] new sample.
     * @param sample [zh] 新采样。
     * @return [en] average over the samples seen so far (at most N).
     * @return [zh] 目前为止（最多 N 个）采样的平均值。
     * @since 0.1.0
     */
    auto step(T sample) noexcept -> T {
        if (count_ == N) {
            sum_ = static_cast<T>(sum_ - buffer_[index_]);
        } else {
            ++count_;
        }

        buffer_[index_] = sample;
        sum_ = static_cast<T>(sum_ + sample);
        index_ = (index_ + 1U) % N;

        return static_cast<T>(sum_ / static_cast<T>(count_));
    }

    /**
     * @brief [en] Clear the window.
     * @brief [zh] 清空窗口。
     * @since 0.1.0
     */
    void reset() noexcept {
        buffer_.fill(T{});
        sum_ = T{};
        index_ = 0U;
        count_ = 0U;
    }

    /**
     * @brief [en] Number of samples currently in the window.
     * @brief [zh] 当前窗口内的采样数量。
     * @since 0.1.0
     */
    [[nodiscard]] auto size() const noexcept -> std::size_t { return count_; }

  private:
    std::array<T, N> buffer_{};
    T sum_{};
    std::size_t index_{0};
    std::size_t count_{0};
};

} // namespace core

} // namespace foc
