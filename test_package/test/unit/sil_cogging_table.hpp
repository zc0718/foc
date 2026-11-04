// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
// Conan::ImportEnd

namespace foc {

namespace sil {

/**
 * @brief [en] One harmonic of a cogging torque: `amplitude * sin(order*theta_mech + phase)`, in N*m.
 *
 * @brief [zh] 齿槽转矩的一个谐波：`amplitude · sin(order·θ_mech + phase)`，N·m。
 *
 *     `order` is **per mechanical revolution** -- the quantity a designer reads off `lcm(slots, poles)`, and the
 *     quantity a compensator's table is indexed by. It is not an electrical order: mixing the two is a factor of
 *     `pole_pairs`, which is exactly the kind of error that makes a compensation look like it does not work. `order` 是
 *     **每机械转**的次数——设计师从 `lcm(slots, poles)` 读到的那个量，也是补偿器表被索引的那个量。它不是电次数：两者混用
 *     会差一个极对数，而那正是让补偿「看起来没生效」的那类错误。
 *
 * @note [en] This type and the two functions below live in their own header because **two** plants inject it: the d-q
 *            plant, which is the one the closed loop runs on, and the phase-variable plant, which is the independent
 *            witness. Two copies of this arithmetic would be two places for a factor of `pole_pairs` to hide, and the
 *            whole point of the second plant is that it does not share the first one's mistakes.
 * @note [zh] 这个类型与下面两个函数单独成一个头文件，是因为**两台**被控对象都注入它：dq plant（闭环跑的那台）与相变量
 *            plant（独立证人）。这份算术若有两份拷贝，就多两个地方可以藏住一个极对数；而第二台 plant 存在的全部意义，正
 *            是它不共享第一台的错误。
 * @since 0.1.9
 */
struct CoggingHarmonic final {
    std::uint8_t order; ///< [en] per mechanical revolution. [zh] 每机械转的次数。
    float amplitude_nm; ///< [en] peak of this harmonic, N*m. [zh] 该谐波的峰值，N·m。
    float phase_rad;    ///< [en] phase at `theta_mech = 0`, rad. [zh] `θ_mech = 0` 处的相位，rad。
};

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
