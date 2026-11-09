// Conan::ImportStart
#include <cstdint>

#include <gtest/gtest.h>

#include <foc/core/types.hpp>
#include <foc/core/watchdog.hpp>
// Conan::ImportEnd

namespace {

using foc::ErrorCode;
using foc::core::ElapsedClock;
using foc::core::FaultCode;
using foc::core::Watchdog;
using foc::core::WatchdogConfig;
using foc::core::WatchdogState;

// F41: the watchdog and the clock it judges.
// F41：看门狗与它所判的时钟。
//
// The plan's criterion is "a feed interval past the window enters `Fault` with the bridge off, and a window that is a
// whole number of 25 us periods must be judged **exactly** -- no float accumulation". Both halves are assertions about
// numbers here, and the second one is the half that is easy to get wrong: a watchdog built on `float t += dt` inherits
// the drift `ElapsedClock`'s own note measures (12.8 % fast at 512 s, frozen after), which turns a 25 us deadline into a
// deadline nobody can state. These cases pin the arithmetic instead of the intention.
// 计划里的判据是"喂狗间隔超过窗口则进入 `Fault` 且桥关断；窗口是 25 µs 整数倍时必须 **精确** 判定——不允许浮点累加"。两半
// 在这里都是关于数字的断言，而第二半正是容易做错的那一半：建立在 `float t += dt` 上的看门狗会继承 `ElapsedClock` 自己记下的
// 那点漂移（512 s 处快 12.8 %、之后冻结），于是一个 25 µs 的截止时刻变成一个谁也说不清的时刻。这些用例锁的是算术，不是意图。

constexpr std::uint64_t kPeriodNs = 25000U; // 25 us, the period `ElapsedClock`'s note is measured at 25 µs 周期

// 1. Exactness, over every whole number of control periods up to 1 ms.
// 1. 精确性：从 1 个到 40 个控制周期（1 ms）。
TEST(Watchdog, TheDeadlineIsExactForEveryWholeNumberOfControlPeriods) {
    for (std::uint64_t k = 1U; k <= 40U; ++k) {
        // The seconds-facing route the caller actually writes: `25e-6F * k` must land on `k * 25000` ns exactly, or the
        // window is not the one it says it is before the watchdog is even involved.
        // 调用方真正会写的那条按秒的路：`25e-6F * k` 必须精确落在 `k * 25000` ns 上，否则窗口在看门狗还没介入之前就已经不是
        // 它自称的那个窗口了。
        EXPECT_EQ(k * kPeriodNs, ElapsedClock::to_nanos(25.0e-6F * static_cast<float>(k))) << "k " << k;

        const std::uint64_t window = k * kPeriodNs;
        Watchdog dog;
        ASSERT_EQ(ErrorCode::Ok, dog.setup(WatchdogConfig{window})) << "k " << k;
        ASSERT_EQ(WatchdogState::Feeding, dog.kick(0U).state) << "k " << k;

        // Exactly on the deadline: still feeding, zero left. The comparison is strict, so the window is inclusive and a
        // feeder that is exactly on time is not tripped by its own period.
        // 恰好在截止时刻：仍在喂，剩余为零。比较是严格的，于是窗口是闭区间，恰好准时的喂狗方不会被自己的周期绊倒。
        const auto on_time = dog.poll(window);
        EXPECT_EQ(WatchdogState::Feeding, on_time.state) << "k " << k;
        EXPECT_EQ(ErrorCode::Ok, on_time.status) << "k " << k;
        EXPECT_EQ(0U, on_time.remaining_ns) << "k " << k;

        // One nanosecond late: tripped, and the reported overshoot is exactly 1 ns -- not one period, and not a number
        // that drifts with the magnitude of the window.
        // 迟一纳秒：跳闸，且报出的超出量正好是 1 ns——不是一个周期，也不是一个随窗口大小漂移的数。
        const auto late = dog.poll(window + 1U);
        EXPECT_EQ(WatchdogState::Expired, late.state) << "k " << k;
        EXPECT_EQ(ErrorCode::Fault, late.status) << "k " << k;
        EXPECT_EQ(window + 1U, late.step_ns) << "k " << k;
        EXPECT_EQ(FaultCode::Watchdog, dog.fault_code()) << "k " << k;
    }
}

// 2. The latch: a feeder that comes back does not clear the trip.
// 2. 锁存：喂狗方回来并不能清除跳闸。
//
// This is the case that makes "stop feeding -> the status word changes" red if the module ever stops reporting: the drive
// is faulted, the feeder resumes, and the *only* thing that may clear it is an explicit `reset()`.
// 这个用例让"停止喂狗 ⇒ 状态字要变"在模块不再报告时变红：驱动器已故障、喂狗方恢复，而唯一能清掉它的是一次显式的
// `reset()`。
TEST(Watchdog, AFeederThatComesBackDoesNotClearTheTrip) {
    const std::uint64_t window = 10U * kPeriodNs; // 250 us
    Watchdog dog;
    ASSERT_EQ(ErrorCode::Ok, dog.setup(WatchdogConfig{window}));

    // Feed at a healthy 4 kHz for a while: 25 kicks, all inside the window.
    // 先按健康的 4 kHz 喂一段：25 次，每次都在窗口内。
    std::uint64_t now = 0U;
    for (int n = 0; n < 25; ++n) {
        ASSERT_EQ(ErrorCode::Ok, dog.kick(now).status);
        now += kPeriodNs;
    }
    EXPECT_EQ(25U, dog.kicks());

    // Then the feeder stops. One period late is still fine; the deadline itself is still fine; one nanosecond past it is
    // not. The loop left `now` one period past the last accepted kick, so the deadline is `window - one period` away and
    // the case says so in those terms rather than by adjusting a literal.
    // 然后喂狗方停了。迟一个周期仍然没事；截止时刻本身仍然没事；越过它一纳秒就不行。循环结束时 `now` 已经比最后一次被接受的
    // 喂狗晚了一个周期，所以截止时刻在 `window − 一个周期` 之外，本用例就这么写，而不是去调整一个字面量。
    const std::uint64_t last_kick = now - kPeriodNs;
    const std::uint64_t deadline = last_kick + window;
    ASSERT_EQ(WatchdogState::Feeding, dog.poll(deadline).state)
        << "exactly at the deadline the drive is still allowed to run";
    const auto tripped = dog.poll(deadline + 1U);
    EXPECT_EQ(WatchdogState::Expired, tripped.state);
    EXPECT_EQ(ErrorCode::Fault, tripped.status);
    EXPECT_EQ(window + 1U, tripped.step_ns) << "the overshoot, measured from the last accepted kick";

    // The feeder comes back, late. This is the expected thing after a trip -- the host is still running, only slower.
    // 喂狗方回来了，但是迟了。跳闸之后这是预期的事——宿主还在跑，只是慢了。
    const auto late = dog.kick(deadline + 5U * window);
    EXPECT_EQ(ErrorCode::State, late.status) << "a late kick is a state error, not a silent re-arm";
    EXPECT_EQ(WatchdogState::Expired, late.state);
    EXPECT_EQ(25U, dog.kicks()) << "and it is not counted as a feed";
    EXPECT_EQ(FaultCode::Watchdog, dog.fault_code()) << "so the fault word still names the silence";
    // The latched report keeps the numbers that caused it rather than growing with the clock.
    // 锁存后的报告保留造成它的那个数，而不是随时钟增长。
    EXPECT_EQ(window + 1U, dog.poll(deadline + 1000U * window).step_ns);

    // Only an explicit reset re-arms, and it keeps the window: restarting a drive is not choosing a new deadline.
    // 只有显式复位才重新武装，而且它保留窗口：重启一台驱动器不是在选一个新的截止时刻。
    dog.reset();
    EXPECT_EQ(WatchdogState::Unarmed, dog.state());
    EXPECT_EQ(0U, dog.kicks());
    EXPECT_EQ(window, dog.config().window_ns) << "the window survives the reset";
    const auto again = dog.kick(deadline + 2000U * window);
    EXPECT_EQ(ErrorCode::Ok, again.status);
    EXPECT_EQ(WatchdogState::Feeding, again.state);
    EXPECT_EQ(FaultCode::None, dog.fault_code()) << "after a reset the watchdog no longer accuses the drive";
}

// 3. A backwards clock is its own diagnosis.
// 3. 倒退的时钟是它自己的诊断。
//
// The naive unsigned difference says "the feeder has been silent for 584 years", which is a true statement about the
// arithmetic and a false one about the drive. Both answers stop the bridge -- that is the safe half -- and the cause is
// what a reader needs next.
// 天真的无符号差说"喂狗方沉默了 584 年"，那是对算术为真、对驱动器为假的一句话。两个答案都会关桥——那是安全的那一半——而
// 读者接下来需要的是成因。
TEST(Watchdog, AClockThatGoesBackwardsIsAClockFaultNotAFiveHundredYearSilence) {
    Watchdog dog;
    ASSERT_EQ(ErrorCode::Ok, dog.setup(WatchdogConfig{kPeriodNs}));
    ASSERT_EQ(ErrorCode::Ok, dog.kick(10'000'000U).status); // 10 ms

    // The host's clock steps 5 ms backwards, e.g. a second time source took over.
    // 宿主的时钟倒退 5 ms，例如换了一个时间源。
    const std::uint64_t backwards = 5'000'000U;
    const std::uint64_t now = 10'000'000U - backwards;
    const auto report = dog.poll(now);
    EXPECT_EQ(WatchdogState::ClockLost, report.state) << "a backwards step is not a silence";
    EXPECT_EQ(ErrorCode::Fault, report.status);
    EXPECT_EQ(backwards, report.step_ns) << "and it reports how far back the clock went";
    EXPECT_EQ(FaultCode::ClockLost, dog.fault_code());

    // The number the unsigned subtraction would have produced, recorded so the distinction is measured rather than
    // asserted: 18446744073709551616 - 5000000 ns is 584 years of silence.
    // 无符号减法本来会产出的那个数，记下来是为了让这个区分被量到而不是被断言：18446744073709551616 − 5000000 ns 是 584 年。
    const std::uint64_t naive = std::uint64_t{0U} - backwards;
    RecordProperty("naive_silence_ns", static_cast<double>(naive));
    EXPECT_GT(static_cast<double>(naive) / 1.0e9 / 86400.0 / 365.25, 500.0) << "the naive answer is half a millennium";

    // And a keeper of the fault wants to know which of the two happened, so the codes differ.
    // 而保管故障的人想知道到底发生了哪一个，所以两个码不同。
    EXPECT_NE(static_cast<int>(FaultCode::Watchdog), static_cast<int>(FaultCode::ClockLost));
}

// 4. The limitation, asserted so it cannot quietly become a claim.
// 4. 那条限制，断言下来，免得它悄悄变成一句声称。
//
// A forward jump larger than the window is **not** a clock fault, because nothing can tell it apart from a feeder that
// starved for exactly that long: the two produce the same two numbers. Reporting it as a clock fault would be inventing
// a diagnosis, so it is reported as the silence it looks like.
// 前向跳跃大于窗口 **不是** 时钟故障，因为没有任何东西能把它与"喂狗方正好饿了那么久"区分开：两者给出同样的两个数。把它报成
// 时钟故障等于编造诊断，所以按它看起来的样子报成沉默。
TEST(Watchdog, AForwardJumpIsJudgedAsASilenceBecauseNothingCanTellItFromOne) {
    Watchdog dog;
    ASSERT_EQ(ErrorCode::Ok, dog.setup(WatchdogConfig{kPeriodNs}));
    ASSERT_EQ(ErrorCode::Ok, dog.kick(0U).status);

    const std::uint64_t hour_ns = 3600ULL * 1000000000ULL;
    const auto jumped = dog.poll(hour_ns);
    EXPECT_EQ(WatchdogState::Expired, jumped.state);
    EXPECT_EQ(FaultCode::Watchdog, dog.fault_code()) << "the cause stays 'no kick', not 'bad clock'";
    EXPECT_EQ(hour_ns, jumped.step_ns);
}

// 5. The two refusals, and the one answer that must not read as healthy.
// 5. 两条拒绝，以及一个不许读成健康的答案。
TEST(Watchdog, RefusesAZeroWindowAndCallsAnUnarmedWatchdogAStateError) {
    Watchdog dog;
    // A zero window expires on the first stamp that is not the kick's own, so it is not a tight deadline, it is a
    // configuration error.
    // 零窗口在第一个不等于喂狗时刻的时间戳上就过期，所以它不是严格的截止时刻，而是配置错误。
    EXPECT_EQ(ErrorCode::Param, dog.setup(WatchdogConfig{0U}));
    EXPECT_EQ(WatchdogState::Unarmed, dog.state());

    // A refused window changes nothing -- including a latch. Set one up, trip it, then try to "fix" the window with a
    // zero one.
    // 被拒绝的窗口什么都不改——包括锁存。先建一个、让它跳闸，然后试图用零窗口去"修"它。
    ASSERT_EQ(ErrorCode::Ok, dog.setup(WatchdogConfig{kPeriodNs}));
    ASSERT_EQ(ErrorCode::Ok, dog.kick(0U).status);
    ASSERT_EQ(WatchdogState::Expired, dog.poll(kPeriodNs + 1U).state);
    EXPECT_EQ(ErrorCode::Param, dog.setup(WatchdogConfig{0U}));
    EXPECT_EQ(WatchdogState::Expired, dog.state()) << "a mistyped window does not clear a trip";
    EXPECT_EQ(kPeriodNs, dog.config().window_ns);

    // Never armed: not a fault, and not "healthy" either. Answering `Ok` here is how a drive whose feeder was never
    // wired runs forever with a green status word.
    // 从未武装：不是故障，也不是"健康"。在这里回答 `Ok`，正是一台喂狗根本没接线的驱动器带着绿色状态字一直跑下去的方式。
    Watchdog never;
    ASSERT_EQ(ErrorCode::Ok, never.setup(WatchdogConfig{kPeriodNs}));
    const auto before = never.poll(0U);
    EXPECT_EQ(ErrorCode::State, before.status);
    EXPECT_EQ(WatchdogState::Unarmed, before.state);
    EXPECT_EQ(FaultCode::None, never.fault_code()) << "and it does not accuse the drive of a fault it never saw";

    // A watchdog whose configuration was never accepted refuses to arm at all, rather than arming with a window of 0.
    // 配置从未被接受的看门狗根本拒绝武装，而不是带着 0 的窗口武装起来。
    Watchdog unconfigured;
    const auto refused = unconfigured.kick(1U);
    EXPECT_EQ(ErrorCode::Param, refused.status);
    EXPECT_EQ(WatchdogState::Unarmed, refused.state);
}

// 6. A kick that is itself late, and a kick that sees the clock go backwards.
// 6. 本身就迟到的一次喂狗，以及一次看到时钟倒退的喂狗。
//
// `kick()` is also a poll, and these two paths are the reason: a feeder that arrives past the deadline trips the drive
// instead of quietly moving the deadline, and a feeder whose timestamp went backwards reports a clock fault rather than
// an eighteensextillion-nanosecond silence. Both are paths a real host reaches -- the feeder runs on the same broken clock
// as everything else -- and both are asserted through `kick()` rather than only through `poll()`.
// `kick()` 同时是一次轮询，而下面两条路径正是理由：越过截止时刻才到达的喂狗方会让驱动器跳闸，而不是悄悄把截止时刻往后挪；
// 时间戳倒退了的喂狗方报的是时钟故障，而不是一千八百亿亿纳秒的沉默。两条都是真实宿主会走到的路——喂狗方和别的东西跑在同一个
// 坏掉的时钟上——而两条都通过 `kick()` 断言，而不只是通过 `poll()`。
TEST(Watchdog, ALateKickTripsAndABackwardsKickIsAClockFault) {
    const std::uint64_t window = 4U * kPeriodNs; // 100 us
    Watchdog dog;
    ASSERT_EQ(ErrorCode::Ok, dog.setup(WatchdogConfig{window}));
    ASSERT_EQ(ErrorCode::Ok, dog.kick(0U).status);

    // No poll in between: the feeder itself is the thing that is late, and the trip has to happen on its arrival.
    // 中间没有轮询：迟到的是喂狗方本身，于是跳闸必须发生在它到达的那一刻。
    const auto late = dog.kick(window + 1U);
    EXPECT_EQ(ErrorCode::Fault, late.status) << "a kick past the deadline trips, it does not extend the deadline";
    EXPECT_EQ(WatchdogState::Expired, late.state);
    EXPECT_EQ(window + 1U, late.step_ns);
    EXPECT_EQ(1U, dog.kicks()) << "and the late kick is not counted as a feed";

    // After a reset, a feeder whose stamp went backwards. Same two numbers as the poll case, reached through `kick()`.
    // 复位之后，一个时间戳倒退了的喂狗方。与轮询用例同样的两个数，经由 `kick()` 到达。
    dog.reset();
    ASSERT_EQ(ErrorCode::Ok, dog.kick(1'000'000U).status);
    const auto backwards = dog.kick(999'999U);
    EXPECT_EQ(ErrorCode::Fault, backwards.status);
    EXPECT_EQ(WatchdogState::ClockLost, backwards.state);
    EXPECT_EQ(1U, backwards.step_ns) << "one nanosecond backwards, reported as a clock fault";
    EXPECT_EQ(FaultCode::ClockLost, dog.fault_code());
}

} // namespace
