// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
// Conan::ImportEnd

// The type the library's compensator cancels with, so the plants and the compensator share one convention.
// 库里的补偿器用来抵消的类型，于是被控对象与补偿器共用一套约定。
#include <foc/core/cogging_comp.hpp>

namespace foc {

namespace sil {

/**
 * @brief [en] The harmonic type, taken from the library rather than declared again here.
 *
 * @brief [zh] 谐波类型，取自库而不是在这里重新声明一遍。
 *
 *     One type for the whole chain -- planted, injected and cancelled -- so a table cannot be written by one part and
 *     read by another under a different convention. The `order` is **per mechanical revolution**, the quantity a
 *     designer reads off `lcm(slots, poles)`; mixing it with an electrical order is a factor of `pole_pairs`, which is
 *     exactly the kind of error that makes a compensation look like it does not work. 整条链——被控对象注入的、以及补偿
 *     器抵消的——共用这一个类型，于是一张表不可能由一方按一种约定写出、由另一方按另一种约定读入。`order` 是**每机械转**
 *     的次数，即设计师从 `lcm(slots, poles)` 读到的那个量；把它与电次数混用会差一个极对数，而那正是让补偿「看起来没生
 *     效」的那类错误。
 * @since 0.1.9
 */
using CoggingHarmonic = ::foc::core::CoggingCompHarmonic;

/** @brief [en] How many harmonics a table may hold. [zh] 一张表最多能装多少条谐波。 */
inline constexpr int kMaxCoggingHarmonics = 6;

/**
 * @brief [en] The cogging torque a table asks for at a mechanical angle, N*m.
 * @brief [zh] 给定机械角处，一张表要求的齿槽转矩，N·m。
 * @param table [en] the harmonics; may be null when `count` is 0.
 * @param table [zh] 谐波表；`count` 为 0 时可以为空指针。
 * @param count [en] how many entries are valid.
 * @param count [zh] 有效条数。
 * @param theta_mech [en] mechanical rotor angle, rad.
 * @param theta_mech [zh] 机械转子角，rad。
 * @since 0.1.9
 */
[[nodiscard]] inline auto evaluate_cogging(const CoggingHarmonic* table, int count, float theta_mech) noexcept
    -> float {
    if (table == nullptr) {
        return 0.0F;
    }
    float sum = 0.0F;
    for (int k = 0; k < count; ++k) {
        const float angle = (static_cast<float>(table[k].order) * theta_mech) + table[k].phase_rad;
        sum = static_cast<float>(sum + (table[k].amplitude_nm * std::sin(angle)));
    }
    return sum;
}

/**
 * @brief [en] Copy a caller's table into fixed storage, dropping what cannot be an injection, and say how many
 *             survived.
 * @brief [zh] 把调用方的表拷进定长存储，丢掉不可能是注入的条目，并报出活下来几条。
 * @param out [en] destination, `capacity` entries.
 * @param out [zh] 目标，`capacity` 条。
 * @param capacity [en] how many entries `out` holds.
 * @param capacity [zh] `out` 能装几条。
 * @param in [en] the caller's table; null clears.
 * @param in [zh] 调用方的表；空指针表示清空。
 * @param count [en] how many entries `in` holds.
 * @param count [zh] `in` 有几条。
 * @return [en] how many entries were kept.
 * @return [zh] 保留了几条。
 * @note [en] Two decisions, both deliberate. `count` above `capacity` is **clamped, not refused**: this is a test
 *            instrument, and "it silently used the first six" is visible in the accessor rather than fatal. And an
 *            order of 0 is **dropped, not rejected**: a constant torque is `set_load_torque()`'s job, and accepting it
 *            here would give a load two ways in.
 * @note [zh] 两个决定都是有意的。`count` 超过 `capacity` 时**截断而不拒绝**：这是测试仪器，"它悄悄用了前六个"在访问器里
 *            看得见，不必做成致命错误。而 0 次**被丢弃而不报错**：恒定转矩是 `set_load_torque()` 的活，在这里接受它就等
 *            于给负载开了两个入口。
 * @since 0.1.9
 */
inline auto store_cogging(CoggingHarmonic* out, int capacity, const CoggingHarmonic* in, int count) noexcept -> int {
    if (out == nullptr || capacity <= 0) {
        return 0;
    }
    if (in == nullptr || count <= 0) {
        return 0;
    }
    const int usable = (count < capacity) ? count : capacity;
    int kept = 0;
    for (int k = 0; k < usable; ++k) {
        if (in[k].order == 0U) {
            continue;
        }
        out[kept] = in[k];
        ++kept;
    }
    return kept;
}

} // namespace sil

} // namespace foc
