// Conan::ImportStart
#pragma once
#include <cstdint>
#include <foc/core/protection.hpp>
#include <foc/core/types.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

/**
 * @brief [en] The one number a watchdog is sized by: how long the feeder may be silent.
 *
 *     `window_ns` is in **nanoseconds**, not seconds, and that is the whole point of the type. A window expressed in
 *     seconds is a float, a float deadline compared against a float elapsed time is two roundings meeting, and the
 *     repository already carries the receipt for that mistake: the float timer this library used to have ran **12.8 %
 *     fast at 512 s and stopped advancing from there**, turning a 512 s timeout into 446.7 s and a 520 s one into one
 *     that never fired (`ElapsedClock`'s note). A watchdog is the last thing that may inherit that class of error --
 *     its job is to stop the drive when a number stops moving, so a deadline that drifts is a deadline that stops
 *     protecting in a way nobody can see.
 *
 * @brief [zh] 看门狗只由这一个数来定尺寸：喂狗方可以沉默多久。
 *
 *     `window_ns` 用**纳秒**而不是秒，这正是本类型存在的理由。用秒表达的窗口是一个浮点数，浮点截止时刻去比浮点已用时间
 *     就是两次舍入相遇，而本仓库已经收着那次错误的收据：这个库原先的浮点计时器**在 512 s 处快 12.8 %、并从那里开始完全
 *     不再前进**，把一个 512 s 的超时变成 446.7 s 触发、把 520 s 的变成永不触发（见 `ElapsedClock` 的说明）。看门狗是最
 *     不该继承这一类误差的东西——它的职责就是在一个数停止前进时把驱动器停下来，于是一个会漂移的截止时刻，是一种没人看得
 *     见的失效。
 * @since 0.1.9
 * @exporter
 */
struct WatchdogConfig final {
    std::uint64_t window_ns{
        0U}; ///< [en] the longest silence allowed, ns; 0 is a configuration error. [zh] 允许的最长沉默，ns；0 是配置错误。
};

/**
 * @brief [en] Where a watchdog stands: the four states a feeder can leave it in.
 *
 *     `Unarmed` is not the healthy state and must not read as one. A watchdog that was never started is protecting
 *     nothing, and the failure it invites is the quiet kind: the host forgets one `kick()` at bring-up, the drive runs
 *     forever, and every report says `Ok` because nothing was ever measured. It is therefore reported as `State` -- the
 *     caller is in the wrong state -- and not as `Feeding`.
 *
 * @brief [zh] 看门狗处在什么位置：喂狗方能把它留在的四个状态。
 *
 *     `Unarmed` 不是健康状态，也不许读成健康状态。一个从未启动的看门狗没有保护任何东西，而它招来的失效是最安静的那种：
 *     主控方在调试时漏了一个 `kick()`，驱动器就一直跑下去，而所有报告都说 `Ok`，因为从来没有量过任何东西。所以它报
 *     `State`—— 调用方处在错误的状态——而不是报 `Feeding`。
 * @since 0.1.9
 * @exporter
 */
enum class WatchdogState : std::uint8_t {
    Unarmed = 0, ///< [en] no kick has arrived yet, so nothing is measured. [zh] 还没有任何一次喂狗，因此什么都没在量。
    Feeding,     ///< [en] the last kick is inside the window. [zh] 最近一次喂狗在窗口之内。
    Expired,     ///< [en] the window passed with no kick; latched. [zh] 窗口过去了而没有喂狗；已锁存。
    ClockLost,   ///< [en] the timestamp went backwards; latched. [zh] 时间戳倒退了；已锁存。
};

/**
 * @brief [en] What one answer says: the state, the step of the clock that decided it, and the time left.
 *
 *     `step_ns` is the **signed** step of the clock, carried in an unsigned field with its direction named by the
 *     state: forward steps are how long the feeder was silent, and a backwards step is how far the clock went back. One
 *     field rather than two, because a report where only one of two numbers is meaningful is a report whose other half
 *     gets read anyway.
 *
 *     A latched report repeats **the numbers that caused the latch**, not a growing timer. A caller that polls an
 *     already-tripped watchdog an hour later must see the trip, not an hour -- otherwise the first question ("how late
 *     was the feeder") has an answer that changes with when it is asked.
 *
 * @brief [zh] 一次回答说了什么：状态、决定它的那一跳时钟、以及剩余时间。
 *
 *     `step_ns` 是时钟那一跳的**带符号**步长，用无符号字段承载，方向由状态命名：正步长是喂狗方沉默了多久，负步长是时钟
 *     倒退了多远。一个字段而不是两个，因为一个"两个数里只有一个有意义"的报告，另一半照样会被读。
 *
 *     锁存后的报告重复**造成锁存的那个数**，而不是一个不断增长的计时器。一个小时后才去轮询一个已经跳闸的看门狗，必须看
 *     到那次跳闸，而不是"一小时"——否则"喂狗方迟了多久"这个头号问题的答案会随提问时间而变。
 * @since 0.1.9
 * @exporter
 */
struct WatchdogReport final {
    ErrorCode
        status; ///< [en] `Ok` feeding, `State` unarmed or fed after a latch, `Fault` tripped, `Param` bad window. [zh] `Ok` 正在喂、`State` 未武装或锁存后再喂、`Fault` 已跳闸、`Param` 窗口非法。
    WatchdogState state; ///< [en] the state this answer is about. [zh] 本回答所处的状态。
    std::uint64_t
        step_ns; ///< [en] forward silence, or backwards distance; 0 when unarmed. [zh] 前向沉默时长，或倒退距离；未武装时为 0。
    std::uint64_t
        remaining_ns; ///< [en] time left before the deadline while feeding, else 0. [zh] 正在喂时距截止的剩余时间，否则 0。
};

/**
 * @brief [en] Judge one timestamp against a window, and latch the first answer that says the drive must stop.
 *
 *     **The law is one subtraction.** `elapsed = now - last_kick`, and the verdict is `elapsed > window`. The
 *     comparison is *strict*, so a kick that arrives exactly on the deadline is on time: a window of `k * 25 us` is
 *     judged exactly at `k * 25 us + 1 ns`, one nanosecond late rather than one control period late, and with a stamp
 *     the caller supplies the arithmetic adds no error of its own. What the caller **does** add is the resolution of
 *     that stamp: with a microsecond clock the trip is decided to the microsecond, and a window shorter than the
 *     clock's own resolution cannot be judged at all. That is a property of the host's clock, not of this class, and it
 *     is stated here so "exact" is not read as a claim about somebody else's timer.
 *
 *     **A backwards clock is not a very long silence.** `now - last_kick` in `uint64_t` turns a 5 ms step backwards
 *     into `1.8e19 ns` -- 584 years of silence -- and a report that says "the feeder starved" about a clock that
 *     jumped. Both answers stop the drive, which is the safe part, but the first question afterwards is *which*
 *     happened, and this class distinguishes them: backwards motion latches `ClockLost`, not `Expired`. A **forward**
 *     jump larger than the window is *not* distinguished, because nothing can distinguish it -- a one-hour step forward
 *     and a feeder that stopped for an hour are the same two numbers -- and inventing a diagnosis the evidence does not
 *     support is how a fault word becomes a rumour. It is reported as the expiry it is.
 *
 *     **The latch is the feature.** Once `Expired` or `ClockLost`, the state does not move until `reset()`, and a
 *     `kick()` after the latch answers `State` instead of re-arming. A feeder that comes back is the *expected* thing
 *     after a trip -- the host is still running, only late -- so a watchdog that re-armed on the next kick would
 *     un-trip itself and its fault word would describe something that happened while nobody was looking. `reset()` is
 *     the one deliberate act.
 *
 *     **What it does not do.** It does not call `Motor::fault()`: the drive belongs to the caller, and a library that
 *     reached into it would be making the host's control flow decision. It knows the cause, in `fault_code()`, and the
 *     two-line chain is the host's: `if (watchdog.poll(now).status == ErrorCode::Fault) {
 *     motor.fault(watchdog.fault_code()); }`. It does not arm a hardware watchdog, does not read a register, and does
 *     not decide how often it should be polled.
 *
 * @brief [zh] 把一个时间戳对窗口判一次，并把第一个"驱动器必须停"的答案锁存下来。
 *
 *     **律只有一次减法。** `elapsed = now - last_kick`，判定是 `elapsed > window`。比较是**严格**的，所以在截止时刻上恰
 *     好到达的一次喂狗算准时：`k × 25 µs` 的窗口在 `k × 25 µs + 1 ns` 处被判为过期，迟一纳秒而不是迟一个控制周期；而时
 *     间戳由调用方提供，这套算术自己不引入误差。调用方**确实**引入的是那个时间戳的分辨率：用微秒钟时跳闸判定精确到微
 *     秒，比时钟自身分辨率还短的窗口根本无法判定。那是宿主时钟的性质，不是本类的性质，写在这里是为了"精确"不被读成对别
 *     人家计时器的声明。
 *
 *     **倒退的时钟不是一次很长的沉默。** `uint64_t` 里 `now - last_kick` 把倒退 5 ms 变成 `1.8e19 ns`——584 年的沉默——于
 *     是一份"喂狗方饿死了"的报告说的其实是一个跳变了的时钟。两个答案都会停下驱动器，那是安全的那一半；但事后第一个问题
 *     是 **到底哪一个**，而本类把二者分开：倒退锁存 `ClockLost` 而不是 `Expired`。**前向**跳跃大于窗口则不区分，因为没
 *     人能区分 ——向前跳一小时与喂狗方停了一小时是同样的两个数——而编造证据支撑不了的诊断，正是故障字变成传闻的方式。它按
 *     过期报。
 *
 *     **锁存就是功能本身。** 一旦 `Expired` 或 `ClockLost`，状态直到 `reset()` 都不会动，而锁存之后的一次 `kick()` 回答
 *     `State` 而不是重新武装。跳闸之后喂狗方恢复是**预期**的事——宿主还在跑，只是迟了——所以一个下一次喂狗就重新武装的看
 *     门狗会自己撤销跳闸，而它的故障字描述的将是"没人看着时发生的事"。`reset()` 是那唯一一次有意的动作。
 *
 *     **它不做什么。** 它不调用 `Motor::fault()`：驱动器属于调用方，库若伸手进去就是在替宿主做控制流决定。它知道成因，
 *     在 `fault_code()` 里，而那条两行的链是宿主的。它不武装硬件看门狗、不读寄存器、也不决定自己该被多频繁地轮询。
 * @since 0.1.9
 * @exporter
 */
class Watchdog final {
  public:
    /**
     * @brief [en] Whether a window can be judged at all.
     * @brief [zh] 这个窗口是否可判。
     * @param config [en] the window to check.
     * @param config [zh] 要检查的窗口。
     * @return [en] `Ok`, or `Param` when the window is zero.
     * @return [zh] `Ok`；窗口为零时 `Param`。
     * @note [en] A zero window is not a tight watchdog: it expires on the first stamp that is not the kick's own, so it
     *            would say "the feeder is late" about every feeder and stop a drive for a reason nobody can act on. It
     *            is refused rather than clamped, because a clamped window is a deadline the caller did not choose.
     * @note [zh] 零窗口不是严格的看门狗：它在第一个不等于喂狗时刻的时间戳上就过期，于是对每一个喂狗方都说"你迟了"，为一
     *            个没人能处置的理由停下驱动器。这里拒绝而不是夹紧，因为被夹紧的窗口是调用方没有选择过的截止时刻。
     */
    [[nodiscard]] static auto validate(const WatchdogConfig& config) noexcept -> ErrorCode;

    /**
     * @brief [en] Take a window, and forget everything measured under the previous one.
     * @brief [zh] 收下一个窗口，并忘掉在旧窗口下量到的一切。
     * @param config [en] the window to judge against.
     * @param config [zh] 用来判定的窗口。
     * @return [en] `Ok`, or `Param` with the previous configuration and state left untouched.
     * @return [zh] `Ok`；窗口非法时 `Param`，且原有配置与状态原封不动。
     * @note [en] A refused window changes nothing, including the latch: a caller that mistypes a window while
     *            diagnosing a trip must not thereby clear it.
     * @note [zh] 被拒绝的窗口什么都不改，包括锁存：在排查一次跳闸时把窗口敲错的调用方，不得因此把跳闸清掉。
     */
    auto setup(const WatchdogConfig& config) noexcept -> ErrorCode;

    /** @brief [en] The window in force. [zh] 当前生效的窗口。 */
    [[nodiscard]] auto config() const noexcept -> const WatchdogConfig& { return config_; }

    /**
     * @brief [en] Back to `Unarmed`: forget the latch, the stamp and the count.
     * @brief [zh] 回到 `Unarmed`：忘掉锁存、时间戳与计数。
     * @note [en] The only way out of a latch, on purpose -- see the class note. The window is kept, because a caller
     *            that resets after a trip is restarting a drive, not choosing a new deadline.
     * @note [zh] 故意做成离开锁存的唯一途径——见类说明。窗口保留，因为跳闸之后复位的调用方是在重启一台驱动器，不是在选一
     *            个新的截止时刻。
     */
    void reset() noexcept;

    /**
     * @brief [en] Feed the watchdog: this is the stamp that says the feeder is alive.
     * @brief [zh] 喂狗：这是"喂狗方还活着"的那个时间戳。
     * @param now_ns [en] the caller's monotonic timestamp, ns.
     * @param now_ns [zh] 调用方的单调时间戳，ns。
     * @return [en] the report for this instant.
     * @return [zh] 这一刻的报告。
     * @note [en] The first kick arms it. After that a kick is also a poll: arriving past the deadline trips the drive
     *            rather than quietly moving the deadline -- a watchdog that forgives a late kick is a watchdog that
     *            only measures the kicks it likes.
     * @note [zh] 第一次喂狗即武装。之后喂狗也是一次轮询：迟到的喂狗会让驱动器跳闸，而不是悄悄把截止时刻往后挪——一个宽容
     *            迟到喂狗的看门狗，只测量它喜欢的那些喂狗。
     */
    auto kick(std::uint64_t now_ns) noexcept -> WatchdogReport;

    /**
     * @brief [en] Ask without feeding, and latch the first answer that says the drive must stop.
     * @brief [zh] 只问不喂，并把第一个"驱动器必须停"的答案锁存下来。
     * @param now_ns [en] the caller's monotonic timestamp, ns.
     * @param now_ns [zh] 调用方的单调时间戳，ns。
     * @return [en] the report for this instant.
     * @return [zh] 这一刻的报告。
     */
    auto poll(std::uint64_t now_ns) noexcept -> WatchdogReport;

    /** @brief [en] The latched state, without asking the clock anything. [zh] 锁存的状态，不问时钟任何问题。 */
    [[nodiscard]] auto state() const noexcept -> WatchdogState { return state_; }

    /**
     * @brief [en] The cause this watchdog would contribute to the drive's fault word.
     * @brief [zh] 本看门狗会向驱动器的故障字贡献的成因。
     * @return [en] `None` unless tripped; `Watchdog` for a silence, `ClockLost` for a backwards clock.
     * @return [zh] 未跳闸时为 `None`；沉默为 `Watchdog`，时钟倒退为 `ClockLost`。
     * @note [en] `Unarmed` answers `None`: a watchdog nobody feeds must not accuse the drive of a fault it never saw.
     *            The missing feeder is a `State` problem, and `poll()` says so in its status.
     * @note [zh] `Unarmed` 回答 `None`：一个没人喂的看门狗不得指控驱动器犯了它从未见过的故障。缺的是喂狗方，那是
     *            `State` 问题，`poll()` 在状态字段里说了。
     */
    [[nodiscard]] auto fault_code() const noexcept -> FaultCode;

    /** @brief [en] The stamp of the last accepted kick; 0 when unarmed. [zh] 最后一次被接受的喂狗时间戳；未武装时为 0。 */
    [[nodiscard]] auto last_kick_ns() const noexcept -> std::uint64_t { return last_kick_ns_; }

    /** @brief [en] How many kicks have been accepted; a late kick is not one. [zh] 已被接受的喂狗次数；迟到的喂狗不算。 */
    [[nodiscard]] auto kicks() const noexcept -> std::uint32_t { return kicks_; }

  private:
    [[nodiscard]] auto latched() const noexcept -> bool {
        return (state_ == WatchdogState::Expired) || (state_ == WatchdogState::ClockLost);
    }
    [[nodiscard]] auto report(std::uint64_t now_ns) const noexcept -> WatchdogReport;
    [[nodiscard]] auto expire(std::uint64_t now_ns) noexcept -> WatchdogReport;
    [[nodiscard]] auto lose_clock(std::uint64_t now_ns) noexcept -> WatchdogReport;

    WatchdogConfig config_{};
    WatchdogState state_{WatchdogState::Unarmed};
    std::uint64_t last_kick_ns_{0U};
    std::uint64_t latched_step_ns_{0U};
    std::uint32_t kicks_{0U};
};

} // namespace core

} // namespace foc
