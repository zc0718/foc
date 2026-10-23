// Conan::ImportStart
#pragma once

#include <cstdint>
// Conan::ImportEnd

namespace foc {

namespace hal {

/**
 * @brief [en] Monotonic microsecond clock used by the control loop.
 *        [zh] 控制环使用的单调微秒时钟。
 *
 * @since 0.1.0
 */
class Clock {
  public:
    virtual ~Clock() = default;

    /**
     * @brief [en] Monotonic tick counter.
     *        [zh] 单调递增的 tick 计数。
     *
     * @return [en] Microseconds since an arbitrary epoch; never decreasing.
     *         [zh] 自任意起点的微秒数；单调不减。
     * @context [en] Any context, must not block. [zh] 任意上下文，不得阻塞。
     * @since 0.1.0
     */
    virtual auto now_us() noexcept -> std::uint64_t = 0;
};

} // namespace hal

} // namespace foc
