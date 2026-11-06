// Conan::ImportStart
#pragma once
#include <cstdint>
// Conan::ImportEnd

namespace foc {


/**
 * @brief [en] Error codes shared by the C++ and the C path.
 * @warning [en] The numeric values are the contract: they mirror `foc_err_t` in `<foc/api/foc_types.h>`. Never renumber
 *               them, and only ever append a new code.
 * @brief [zh] C++ 与 C 路径共用的错误码。
 * @warning [zh] 数值本身就是契约：与 `<foc/api/foc_types.h>` 中的 `foc_err_t` 一一对应。不得重新编号，新增错误码只能追
 *               加。
 * @since 0.1.0
 * @exporter
 */
enum class ErrorCode : std::int8_t {
    Ok = 0,
    Param = -1,
    NoResource = -2,
    State = -3,
    NotImpl = -4,
    Fault = -5, ///< [en] latched inverter fault. [zh] 锁存的逆变故障。
};


/**
 * @brief [en] Core math types shared by both paths.
 *
 *     Fixed layout, trivially copyable, no allocation. Units follow the project-wide SI rule (A, V, rad, rad/s, s,
 *     N*m).
 *
 * @brief [zh] 两条路径共享的核心数学类型。
 *
 *     布局固定、可平凡复制、无内存分配。单位遵循全项目 SI 约定（A、V、rad、rad/s、s、N·m）。
 * @since 0.1.0
 */
namespace core {


/**
 * @brief [en] Stationary two-phase frame vector (Clarke output). Angle in rad, no unit tag needed.
 * @brief [zh] 静止两相坐标系向量（Clarke 输出）。角度单位为 rad。
 * @since 0.1.0
 * @exporter
 */
template <typename T> struct Ab final {
    T alpha{};
    T beta{};
};


/**
 * @brief [en] Rotating two-phase frame vector (Park output): d = flux axis, q = torque axis.
 * @brief [zh] 旋转两相坐标系向量（Park 输出）：d = 磁链轴，q = 转矩轴。
 * @since 0.1.0
 * @exporter
 */
template <typename T> struct Dq final {
    T d{};
    T q{};
};


/**
 * @brief [en] Three-phase quantity in SI units (A for currents, V for voltages, ratio for duty).
 * @brief [zh] 三相量，SI 单位（电流 A、电压 V、占空比为比值）。
 * @since 0.1.0
 * @exporter
 */
template <typename T> struct Uvw final {
    T u{};
    T v{};
    T w{};
};

/**
 * @brief [en] Elapsed time accumulated from control periods, counted in whole nanoseconds.
 * @brief [zh] 由控制周期累加出的已用时间，以整纳秒计数。
 * @note [en] Why an integer and not a float, which is how every clock in this library was written first: a float
 *            accumulator `t += dt` stops advancing at all once `ulp(t) > dt`, and before that it advances in ulp-sized
 *            jumps rather than in `dt` -- so a timer built on it drifts, then races, then freezes, and the three happen
 *            at magnitudes nobody writes down. Measured on the float version of this class at a 25 us period: **12.8 %
 *            fast at 512 s and frozen from there on**, which turned a 512 s timeout into one that fired at 446.7 s and
 *            a 520 s timeout into one that never fired. The integer version is exact to 1 ns and cannot stop: the count
 *            is a `uint64_t` that saturates at 1e18 ns (31.7 years) instead of wrapping, because a wrap would turn a
 *            long timeout into an immediate one. A control period is quantised to 1 ns on the way in, and a positive
 *            period always advances the count by at least 1 ns -- a sub-nanosecond period is not realistic, but a clock
 *            that can be handed a period it silently ignores is the exact defect this class exists to remove.
 * @note [zh] 为什么用整数而不用浮点（本库所有计时器最初都是浮点写法）：浮点累加器 `t += dt` 一旦 `ulp(t) > dt` 就完全不
 *            再前进，而在那之前它以 ulp 为步长而不是以 `dt` 前进——于是建立在其上的定时器先漂移、再狂奔、最后冻结，而这
 *            三件事发生在没人写下来的量级上。本类的浮点版在 25 us 周期下实测：**512 s 时快 12.8 %，并从那里开始冻结**，
 *            把一个 512 s 的超时变成 446.7 s 触发、把 520 s 的超时变成永不触发。整数版准到 1 ns 且不会停：计数是
 *            `uint64_t`，到 1e18 ns（31.7 年）饱和而不是回绕，因为回绕会把一个长超时变成立刻超时。控制周期在入口被量化
 *            到 1 ns，且正周期至少让计数前进 1 ns——亚纳秒周期不现实，但一个会静默忽略传入周期的时钟正是本类要消灭的那种
 *            缺陷。
 * @since 0.1.6
 * @exporter
 */
class ElapsedClock final {
  public:
    static constexpr float kNanosPerSecond = 1.0e9F;                   ///< [en] ns per s. [zh] 每秒纳秒数。
    static constexpr std::uint64_t kMaxNanos = 1000000000000000000ULL; ///< [en] saturation point, ns. [zh] 饱和点，ns。

    /** @brief [en] Back to zero. [zh] 归零。 */
    void reset() noexcept { nanos_ = 0U; }

    /**
     * @brief [en] Add one control period.
     * @brief [zh] 累加一个控制周期。
     * @param dt_s [en] the period in s; non-positive adds nothing.
     * @param dt_s [zh] 周期，s；非正值不累加。
     */
    void add_sample(float dt_s) noexcept {
        std::uint64_t step = to_nanos(dt_s);
        if (step == 0U && dt_s > 0.0F) {
            step = 1U;
        }
        nanos_ = ((nanos_ > (kMaxNanos - step)) ? kMaxNanos : (nanos_ + step));
    }

    /** @brief [en] Nanoseconds accumulated. [zh] 已累加的纳秒数。 */
    [[nodiscard]] auto nanos() const noexcept -> std::uint64_t { return nanos_; }

    /** @brief [en] Seconds accumulated. [zh] 已累加的秒数。 */
    [[nodiscard]] auto seconds() const noexcept -> float {
        return static_cast<float>(nanos_) * (1.0F / kNanosPerSecond);
    }

    /** @brief [en] Whether the count has reached `limit_s`. [zh] 计数是否已达到 `limit_s`。 */
    [[nodiscard]] auto reached(float limit_s) const noexcept -> bool { return nanos_ >= to_nanos(limit_s); }

    /**
     * @brief [en] Seconds to whole nanoseconds: nearest and saturating.
     *
     *     Anything not positive converts to zero rather than wrapping, NaN included.
     *
     * @brief [zh] 秒转整纳秒：四舍五入、饱和。
     *
     *     非正的一律转为零而不是回绕，NaN 也在内。
     *
     * @param seconds [en] a duration in s.
     * @param seconds [zh] 以秒计的时长。
     * @return [en] the equivalent count.
     * @return [zh] 等价的计数。
     * @note [en] Split into whole seconds and a fraction on purpose. `seconds * 1e9F` in one step is a float multiply
     *            whose result needs 39 bits, so a 520 s recipe came out 16 us long and fired one tick late -- a
     *            quantisation error that is invisible until it is more than a period. The integer part stays an integer
     *            and the fraction (below 1) multiplies into a range where a float still has sub-100 ns resolution.
     * @note [zh] 故意拆成整秒与小数两部分。一步做 `seconds * 1e9F` 是一次需要 39 位的浮点乘法，于是一个 520 s 的配方算
     *            出长了 16 us、晚一个 tick 触发——这种量化误差在超过一个周期之前是看不见的。整数部分保持整数，小数部分（
     *            小于 1）乘进一个浮点仍有百纳秒以内分辨率的范围。
     */
    [[nodiscard]] static auto to_nanos(float seconds) noexcept -> std::uint64_t {
        if (!(seconds > 0.0F)) {
            return 0U;
        }
        // 1e9 s is exactly kMaxNanos; anything above it saturates rather than wrapping.
        // 1e9 s 恰好是 kMaxNanos；超过它的饱和而不是回绕。
        if (!(seconds < 1.0e9F)) {
            return kMaxNanos;
        }
        const auto whole = static_cast<std::uint64_t>(seconds);
        const float fraction = seconds - static_cast<float>(whole);
        const float scaled = fraction * kNanosPerSecond;
        const auto truncated = static_cast<std::uint64_t>(scaled);
        const bool round_up = (scaled - static_cast<float>(truncated)) >= 0.5F;
        const auto sub = truncated + (round_up ? 1U : 0U);
        return (whole * 1000000000ULL) + ((sub < 1000000000ULL) ? sub : 999999999ULL);
    }

  private:
    std::uint64_t nanos_{0U};
};

} // namespace core

} // namespace foc
