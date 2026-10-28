// Conan::ImportStart
#pragma once
#include <array>
#include <cstdint>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {


/**
 * @brief [en] One period of the control loop, as the loop itself saw it.
 * @brief [zh] 控制环的一个周期，以控制环自己的视角记录。
 * @note [en] These are the numbers the loop computed, not what the hardware did. `i_d`/`i_q` are the measured currents
 *            the PI acted on, `i_ref_*` what it was asked for, `v_*` what was applied before modulation. A trace cannot
 *            tell you the current in the phases -- it tells you the current the loop believed, which is the only thing
 *            that explains its decisions. `t_us` is the platform clock truncated to 32 bits, so the record stays a
 *            fixed 36 bytes and wraps every 71.6 minutes. It is 0 for every sample when the HAL has no clock -- a
 *            constant zero means "no timestamp", which is a fact about the platform rather than a claim that everything
 *            happened at once.
 * @note [zh] 这些是控制环算出来的数，不是硬件实际做了什么。`i_d`/`i_q` 是 PI 作用其上的实测电流，`i_ref_*` 是被要求的
 *            值，`v_*` 是调制前施加的电压。轨迹不能告诉你各相里真实的电流——它告诉你环**相信**的电流，而那是唯一能解释它
 *            决策的东西。 `t_us` 是平台时钟截断到 32 位，于是每条记录保持 32 字节，每 71.6 分钟回绕一次。HAL 没有时钟时
 *            每条都是 0 ——恒为 0 意味着"没有时间戳"，这是关于平台的事实，而不是"所有事件同时发生"的声明。
 * @since 0.1.8
 * @exporter
 */
struct TraceSample final {
    std::uint32_t t_us;  ///< [en] platform clock, low 32 bits. [zh] 平台时钟低 32 位。
    float i_d;           ///< [en] measured d-axis current, A. [zh] 实测 d 轴电流，A。
    float i_q;           ///< [en] measured q-axis current, A. [zh] 实测 q 轴电流，A。
    float i_ref_d;       ///< [en] demanded d-axis current, A. [zh] 给定的 d 轴电流，A。
    float i_ref_q;       ///< [en] demanded q-axis current, A. [zh] 给定的 q 轴电流，A。
    float v_d;           ///< [en] applied d-axis voltage, V. [zh] 施加的 d 轴电压，V。
    float v_q;           ///< [en] applied q-axis voltage, V. [zh] 施加的 q 轴电压，V。
    float speed_mech;    ///< [en] mechanical speed, rad/s. [zh] 机械转速，rad/s。
    std::uint8_t mode;   ///< [en] `MotorState` at the end of the period. [zh] 周期结束时的 `MotorState`。
    std::uint8_t status; ///< [en] `ErrorCode` at the end of the period. [zh] 周期结束时的 `ErrorCode`。
    std::uint16_t
        pad; ///< [en] keeps the record a fixed size (36 bytes with these seven floats). [zh] 使记录为固定 36 字节。
};


/**
 * @brief [en] The trace channel switched off: a type, no data.
 * @brief [zh] 关闭状态的记录通道：一个类型，没有数据。
 * @note [en] This is the default, and it is what makes "off costs nothing" a structural fact rather than a hope: the
 *            type is empty, every call is a no-op that the optimiser removes, and `sizeof()` of the motor does not
 *            change. A runtime flag would be cheaper to reason about and more expensive to run -- one branch per
 *            period, forever, in the path that must be predictable.
 * @note [zh] 这是默认值，也是让"关掉不花代价"成为结构性事实而非愿望的原因：类型为空，每个调用都是优化器会删掉的空操作，
 *            电机的 `sizeof()` 不变。运行时开关更好推理，但更贵——每周期一个分支，永远，就在那条必须可预测的路径上。
 * @since 0.1.8
 * @exporter
 */
struct NullTrace final {
    [[nodiscard]] static constexpr auto capacity() noexcept -> std::uint16_t { return 0; }
    [[nodiscard]] static constexpr auto size() noexcept -> std::uint16_t { return 0; }
    [[nodiscard]] static constexpr auto dropped() noexcept -> std::uint16_t { return 0; }
    constexpr auto push(const TraceSample& /*sample*/) noexcept -> bool { return false; }
    constexpr void reset() noexcept {}
};


/**
 * @brief [en] A fixed-capacity ring of trace samples: no allocation, no growth, no surprises.
 * @brief [zh] 固定容量的轨迹环形缓冲：不分配、不扩容、没有意外。
 * @tparam Capacity [en] samples retained; the most recent `Capacity` periods.
 * @tparam Capacity [zh] 保留的样本数；即最近 `Capacity` 个周期。
 * @note [en] When it fills, the oldest sample is overwritten and `dropped()` counts the loss. That is deliberate: a
 *            trace that stops recording when full loses exactly the window a fault is interesting in. Forgetting with a
 *            counter is reversible (you know how much you lost); stopping is not. `push()` returns `true` when the ring
 *            is about to overwrite a sample that was never read. A caller that treats this as an error has a bug; it is
 *            information.
 * @note [zh] 写满时覆盖最旧的样本，并由 `dropped()` 计数。这是故意的：写满就停的记录，恰好丢掉故障真正有意思的那段窗
 *            口。带着计数器的遗忘是可反推的（你知道丢了多少），停止不是。 `push()` 在即将覆盖一个从未被读过的样本时返回
 *            `true`。把这个当错误处理的调用方是错的：这是信息。
 * @since 0.1.8
 * @exporter
 */
template <std::uint16_t Capacity> class RingTrace final {
    static_assert(Capacity > 0, "a RingTrace of zero capacity is NullTrace");

  public:
    /**
     * @brief [en] Record one period. Returns `true` when an unread sample was overwritten.
     * @brief [zh] 记录一个周期。覆盖了未读样本时返回 `true`。
     */
    auto push(const TraceSample& sample) noexcept -> bool {
        const bool overwrote = (count_ == Capacity);
        samples_[head_] = sample;
        head_ = static_cast<std::uint16_t>((head_ + 1U) % Capacity);
        if (overwrote) {
            ++dropped_;
        } else {
            ++count_;
        }
        return overwrote;
    }

    [[nodiscard]] static constexpr auto capacity() noexcept -> std::uint16_t { return Capacity; }
    [[nodiscard]] auto size() const noexcept -> std::uint16_t { return count_; }
    [[nodiscard]] auto dropped() const noexcept -> std::uint16_t { return dropped_; }

    /**
     * @brief [en] Sample `index` from the oldest retained, 0-based.
     * @brief [zh] 自最旧保留样本起第 `index` 个，从 0 开始。
     * @return [en] the sample, or a zeroed one when `index` is out of range.
     * @return [zh] 该样本；`index` 越界时返回全零样本。
     */
    [[nodiscard]] auto at(std::uint16_t index) const noexcept -> TraceSample {
        if (index >= count_) {
            return TraceSample{};
        }
        const std::uint16_t oldest = static_cast<std::uint16_t>((head_ + Capacity - count_) % Capacity);
        return samples_[static_cast<std::uint16_t>((oldest + index) % Capacity)];
    }

    /**
     * @brief [en] The most recent sample; zeroed when nothing has been recorded.
     * @brief [zh] 最近一个样本；从未记录时返回全零。
     */
    [[nodiscard]] auto latest() const noexcept -> TraceSample {
        return (count_ == 0U) ? TraceSample{} : at(static_cast<std::uint16_t>(count_ - 1U));
    }

    /**
     * @brief [en] Forget the samples; the drop counter is kept, because it is the record of what was already lost.
     * @brief [zh] 丢弃样本；丢帧计数保留，因为它记录的是**已经**丢掉的东西。
     */
    void reset() noexcept {
        head_ = 0;
        count_ = 0;
    }

  private:
    std::array<TraceSample, Capacity> samples_{};
    std::uint16_t head_{};
    std::uint16_t count_{};
    std::uint16_t dropped_{};
};


/**
 * @brief [en] The same ring over storage somebody else owns, with the capacity known at run time.
 * @brief [zh] 同一个环，但存储由别人持有，容量在运行时给定。
 * @note [en] Two users, and both are shaped by a constraint rather than a preference. The C ABI cannot carry a capacity
 *            as a template argument, because the ABI is one type for every caller; and a firmware author who wants to
 *            put the trace in a linker section or in a memory pool should not have to give the library a `static` array
 *            it cannot see. So the samples live where the caller put them. Detached (`attach(nullptr, 0)`, or the
 *            default-constructed object) every call is a no-op and nothing is counted: there is no window to have lost.
 *            Attached, the cost is one null check per period -- which is why the compiled-in path uses `RingTrace`
 *            instead, where the check cannot exist.
 * @note [zh] 两个使用方，都是被约束而不是被偏好塑造的。C ABI 无法把容量当模板参数携带，因为 ABI 对所有调用方都是一个类
 *            型；而想把轨迹放进链接段或内存池的固件作者，不该被迫让库持有一块它看不见的 `static` 数组。因此样本住在调用
 *            方放置它们的地方。未挂载时（`attach(nullptr, 0)` 或默认构造）每个调用都是空操作，什么也不计数：没有窗口可
 *            谈丢失。挂载后每周期一个空指针检查——这也正是编译期路径用 `RingTrace` 的原因：那里连这个检查都不存在。
 * @since 0.1.8
 * @exporter
 */
class SpanTrace final {
  public:
    SpanTrace() noexcept = default;

    SpanTrace(TraceSample* samples, std::uint32_t capacity) noexcept : samples_(samples), capacity_(capacity) {}

    /**
     * @brief [en] Point the ring at caller storage. `(nullptr, 0)` detaches; the counters start over.
     * @brief [zh] 把环指向调用方存储。`(nullptr, 0)` 即卸载；计数器从头开始。
     */
    void attach(TraceSample* samples, std::uint32_t capacity) noexcept {
        samples_ = ((samples != nullptr) && (capacity > 0U)) ? samples : nullptr;
        capacity_ = (samples_ != nullptr) ? capacity : 0U;
        head_ = 0U;
        count_ = 0U;
        dropped_ = 0U;
    }

    void detach() noexcept { attach(nullptr, 0U); }

    [[nodiscard]] explicit operator bool() const noexcept { return samples_ != nullptr; }

    [[nodiscard]] auto capacity() const noexcept -> std::uint32_t { return capacity_; }
    [[nodiscard]] auto size() const noexcept -> std::uint32_t { return count_; }
    [[nodiscard]] auto dropped() const noexcept -> std::uint32_t { return dropped_; }

    /**
     * @brief [en] Record one period. Returns `true` when an unread sample was overwritten, `false` when detached.
     * @brief [zh] 记录一个周期。覆盖了未读样本返回 `true`；未挂载时返回 `false`。
     */
    auto push(const TraceSample& sample) noexcept -> bool {
        if (samples_ == nullptr) {
            return false;
        }
        const bool overwrote = (count_ == capacity_);
        samples_[head_] = sample;
        head_ = (head_ + 1U) % capacity_;
        if (overwrote) {
            ++dropped_;
        } else {
            ++count_;
        }
        return overwrote;
    }

    [[nodiscard]] auto at(std::uint32_t index) const noexcept -> TraceSample {
        if ((samples_ == nullptr) || (index >= count_)) {
            return TraceSample{};
        }
        const std::uint32_t oldest = (head_ + capacity_ - count_) % capacity_;
        return samples_[(oldest + index) % capacity_];
    }

    [[nodiscard]] auto latest() const noexcept -> TraceSample {
        return (count_ == 0U) ? TraceSample{} : at(count_ - 1U);
    }

    /**
     * @brief [en] Forget the samples; the drop counter is kept, as in `RingTrace`.
     * @brief [zh] 丢弃样本；与 `RingTrace` 一样保留丢帧计数。
     */
    void reset() noexcept {
        head_ = 0U;
        count_ = 0U;
    }

  private:
    TraceSample* samples_{nullptr};
    std::uint32_t capacity_{0U};
    std::uint32_t head_{0U};
    std::uint32_t count_{0U};
    std::uint32_t dropped_{0U};
};


} // namespace core

} // namespace foc
