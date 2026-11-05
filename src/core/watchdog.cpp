// Conan::ImportStart
#include <foc/core/watchdog.hpp>
// Conan::ImportEnd

namespace foc {

namespace core {

auto Watchdog::validate(const WatchdogConfig& config) noexcept -> ErrorCode {
    return (config.window_ns == 0U) ? ErrorCode::Param : ErrorCode::Ok;
}

auto Watchdog::setup(const WatchdogConfig& config) noexcept -> ErrorCode {
    // Validate before touching anything: a caller who mistypes a window while diagnosing a trip must not thereby clear it.
    // 先校验再改动任何东西：在排查一次跳闸时把窗口敲错的调用方，不得因此把跳闸清掉。
    const ErrorCode status = validate(config);
    if (status != ErrorCode::Ok) {
        return status;
    }
    config_ = config;
    reset();
    return ErrorCode::Ok;
}

void Watchdog::reset() noexcept {
    state_ = WatchdogState::Unarmed;
    last_kick_ns_ = 0U;
    latched_step_ns_ = 0U;
    kicks_ = 0U;
}

auto Watchdog::report(std::uint64_t now_ns) const noexcept -> WatchdogReport {
    if (state_ == WatchdogState::Unarmed) {
        // Not a fault and not "healthy" either: nothing has been measured, so the status says the caller is in the wrong
        // state rather than answering `Ok` about a watchdog that is protecting nothing.
        // 既不是故障也不是"健康"：什么都没量过，于是状态说调用方处在错误的状态，而不是对一个毫无保护作用的看门狗回答 `Ok`。
        return WatchdogReport{ErrorCode::State, WatchdogState::Unarmed, 0U, 0U};
    }
    if (latched()) {
        // Latched: the numbers that caused the latch, not a growing timer.
        // 已锁存：重复造成锁存的那个数，而不是一个不断增长的计时器。
        return WatchdogReport{ErrorCode::Fault, state_, latched_step_ns_, 0U};
    }
    // Feeding means `step <= window`, so the subtraction below cannot wrap and the remaining time cannot underflow.
    // `Feeding` 意味着 `step <= window`，所以下面的减法既不会回绕，剩余时间也不会下溢。
    const std::uint64_t step = now_ns - last_kick_ns_;
    return WatchdogReport{ErrorCode::Ok, WatchdogState::Feeding, step, config_.window_ns - step};
}

auto Watchdog::expire(std::uint64_t now_ns) noexcept -> WatchdogReport {
    state_ = WatchdogState::Expired;
    // The overshoot, not the window: "how much later than allowed" is the number a reader wants.
    // 记的是超出量而不是窗口本身：读者要的是"比允许的晚了多少"。
    latched_step_ns_ = now_ns - last_kick_ns_;
    return report(now_ns);
}

auto Watchdog::lose_clock(std::uint64_t now_ns) noexcept -> WatchdogReport {
    state_ = WatchdogState::ClockLost;
    // How far the clock went back. `now_ns < last_kick_ns_` is the caller's guarantee here, so this cannot wrap either.
    // 时钟倒退了多远。此处由调用条件保证 `now_ns < last_kick_ns_`，所以这也回绕不了。
    latched_step_ns_ = last_kick_ns_ - now_ns;
    return report(now_ns);
}

auto Watchdog::kick(std::uint64_t now_ns) noexcept -> WatchdogReport {
    if (state_ == WatchdogState::Unarmed) {
        if (validate(config_) != ErrorCode::Ok) {
            WatchdogReport refused = report(now_ns);
            refused.status = ErrorCode::Param;
            return refused;
        }
        state_ = WatchdogState::Feeding;
        last_kick_ns_ = now_ns;
        kicks_ = 1U;
        return report(now_ns);
    }
    if (latched()) {
        // A kick does not clear a latch, and it is not counted as a feed. See the class note: after a trip the feeder
        // coming back is the expected thing, and a watchdog that re-armed on it would un-trip itself.
        // 一次喂狗不清除锁存，也不计入喂狗次数。见类说明：跳闸之后喂狗方恢复是预期的事，而一个因此重新武装的看门狗会自己
        // 撤销跳闸。
        WatchdogReport held = report(now_ns);
        held.status = ErrorCode::State;
        return held;
    }
    if (now_ns < last_kick_ns_) {
        return lose_clock(now_ns);
    }
    if ((now_ns - last_kick_ns_) > config_.window_ns) {
        // A late kick is a trip, not a new deadline: forgiving it would measure only the kicks the feeder chose to send.
        // 迟到的喂狗是一次跳闸，不是新的截止时刻：宽容它等于只测量喂狗方选择发出的那些喂狗。
        return expire(now_ns);
    }
    last_kick_ns_ = now_ns;
    ++kicks_;
    return report(now_ns);
}

auto Watchdog::poll(std::uint64_t now_ns) noexcept -> WatchdogReport {
    if ((state_ == WatchdogState::Unarmed) || latched()) {
        return report(now_ns);
    }
    if (now_ns < last_kick_ns_) {
        return lose_clock(now_ns);
    }
    if ((now_ns - last_kick_ns_) > config_.window_ns) {
        return expire(now_ns);
    }
    return report(now_ns);
}

auto Watchdog::fault_code() const noexcept -> FaultCode {
    // `Unarmed` and `Feeding` both answer `None`: an unarmed watchdog must not accuse the drive of a fault it never saw. A
    // `switch` here would need a trailing return that can never run, and an uncoverable line is a line nobody has tested.
    // `Unarmed` 与 `Feeding` 都回答 `None`：未武装的看门狗不得指控驱动器犯了它从未见过的故障。这里若用 `switch`，就得多一
    // 条永远不会执行到的收尾 `return`，而覆盖不到的行就是没人测过的行。
    if (state_ == WatchdogState::Expired) {
        return FaultCode::Watchdog;
    }
    return (state_ == WatchdogState::ClockLost) ? FaultCode::ClockLost : FaultCode::None;
}

} // namespace core

} // namespace foc
