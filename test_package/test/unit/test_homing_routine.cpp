// W7-a, increment 5: the search for the index mark, as a policy rather than as a few lines in whichever caller needs it.
//
// The routine decides; it does not actuate. That split is what lets the same code drive a real drive, a SIL rig or a
// bench, and it is why these cases can be decisive without a plant: the whole claim is about what the routine commands
// and when it stops.
//
// W7-a 增量 5：把"找索引标记"做成策略，而不是散在某个需要它的调用方里。
// 这个流程只做**决定**，不做驱动。正是这个切分让同一段代码能驱动真实驱动、SIL 台架或试验台，也是这些用例不必带 plant
// 就能有分辨力的原因：全部主张都在于它下达什么指令、以及在什么时候停下。
//
// The criterion, fixed before the numbers below were measured:
//   C1  an axis that is already homed is done before it starts: `begin()` reports finished, the state is `Done`, and
//       nothing is commanded. Turning a homed axis to look for a mark it is sitting on is a move nobody asked for.
//   C2  searching commands `direction * creep_speed` every period until the mark arrives, then commands exactly zero on
//       the tick it does, and reports finished. No overshoot in the command: the caller applies what it is given.
//   C3  a mark that never arrives ends in `TimedOut` with a zero command, and the timeout is crossed on the period it
//       says it is -- a search that keeps turning after announcing failure is worse than one that never started.
//   C4  an all-zero recipe means "no homing was asked for" (`NotImpl`), and `begin()` then reports finished rather than
//       creeping; a zero is not a malformed recipe, it is the absence of a request.
//   C5  a recipe that is present but impossible is `Param`: zero or negative creep, zero timeout, direction outside
//       +/-1. The two answers must stay distinct for the same reason the observer selector's three do.
//
// 判据在量之前定下：
//   C1 已经回零的轴在开始之前就结束：`begin()` 报已完成、状态 `Done`、不下达任何指令。让一个已回零的轴转起来去找它
//      已经坐着的标记，是一次没人要求过的运动。
//   C2 搜索期间每个周期都下达 `direction * creep_speed`，在标记到来的那一个周期**恰好**下达 0 并报完成。指令不超调：
//      调用方施加什么，就是这里给出的什么。
//   C3 永远不来的标记以 `TimedOut` 结束、指令为 0，且超时正是在它声称的那个周期被跨过——一个宣布失败后还在继续转的
//      搜索，比一个从未开始的更糟。
//   C4 全零配方表示"没有要求回零"（`NotImpl`），此时 `begin()` 报已完成而不是爬行；零不是写错的配方，是没有请求。
//   C5 存在但不可能执行的配方是 `Param`：零或负的爬行速度、零超时、方向不在 ±1。两种回答必须分开，理由与观测器
//      选择器那三种回答相同。

#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/homing.hpp>
#include <foc/core/position_reference.hpp>

namespace {

namespace core = foc::core;

constexpr float kCreep = 1.5F;
constexpr float kTimeoutS = 0.25F;
constexpr float kDt = 5.0e-5F;
constexpr std::uint8_t kPoles = 7U;

auto recipe(float creep, float timeout, int8_t direction) -> core::HomingConfig {
    return core::HomingConfig{creep, timeout, direction};
}

auto usable_recipe() -> core::HomingConfig { return recipe(kCreep, kTimeoutS, 1); }

auto make_reference() -> core::PositionReference {
    return core::PositionReference(core::PositionReference::Config{kPoles, 0.0F});
}

/** @brief [en] Drive the reference one tick, optionally with the index pulse. */
/** @brief [zh] 推进参考一个周期，可选携带索引脉冲。 */
void advance(core::PositionReference& reference, int step, bool pulse) {
    static_cast<void>(reference.update(0.01F * static_cast<float>(step), pulse));
}

} // namespace

TEST(HomingRoutine, AnAlreadyHomedAxisIsDoneBeforeItStarts) {
    // C1
    core::PositionReference reference = make_reference();
    advance(reference, 1, true); // the mark is here; the origin exists
    ASSERT_TRUE(reference.homed());

    core::HomingRoutine routine(usable_recipe());
    EXPECT_TRUE(routine.begin(reference)) << "a homed axis must not start a search";
    EXPECT_EQ(core::HomingState::Done, routine.state());
    const core::HomingRoutine::Output out = routine.step(reference, kDt);
    EXPECT_FLOAT_EQ(0.0F, out.speed_ref_rad_s) << "nothing may be commanded on an axis that is already referenced";
    EXPECT_TRUE(out.finished);
}

TEST(HomingRoutine, SearchingCommandsTheCreepSpeedAndStopsOnTheExactTickTheMarkArrives) {
    // C2
    for (const int8_t direction : {static_cast<int8_t>(1), static_cast<int8_t>(-1)}) {
        core::PositionReference reference = make_reference();
        core::HomingRoutine routine(recipe(kCreep, kTimeoutS, direction));
        ASSERT_FALSE(routine.begin(reference));
        ASSERT_EQ(core::HomingState::Creep, routine.state());

        // Creep for a while, then deliver the mark.
        // 先爬行一段，再送来标记。
        float commanded = 0.0F;
        for (int i = 1; i <= 100; ++i) {
            advance(reference, i, false);
            commanded = routine.step(reference, kDt).speed_ref_rad_s;
            ASSERT_FLOAT_EQ(static_cast<float>(direction) * kCreep, commanded) << "step " << i;
        }
        ASSERT_FALSE(reference.homed());

        advance(reference, 101, true);
        ASSERT_TRUE(reference.homed());
        const core::HomingRoutine::Output out = routine.step(reference, kDt);
        std::printf("\n[homing] direction %+d: after the mark, commanded %.1f rad/s, state %d, finished %d\n",
                    static_cast<int>(direction), static_cast<double>(out.speed_ref_rad_s),
                    static_cast<int>(out.state), static_cast<int>(out.finished));
        EXPECT_FLOAT_EQ(0.0F, out.speed_ref_rad_s) << "the command must be zero on the tick the origin exists";
        EXPECT_EQ(core::HomingState::Done, out.state);
        EXPECT_TRUE(out.finished);
        EXPECT_EQ(core::HomingFailure::None, routine.failure());
    }
}

TEST(HomingRoutine, AMarkThatNeverArrivesTimesOutAndStopsCommanding) {
    // C3
    core::PositionReference reference = make_reference();
    core::HomingRoutine routine(usable_recipe());
    ASSERT_FALSE(routine.begin(reference));

    const int ticks = static_cast<int>(kTimeoutS / kDt);
    int first_failed_tick = -1;
    for (int i = 1; i <= (ticks + 10); ++i) {
        advance(reference, i, false); // no mark, ever
        const core::HomingRoutine::Output out = routine.step(reference, kDt);
        if (out.finished && (first_failed_tick < 0)) {
            first_failed_tick = i;
            EXPECT_EQ(core::HomingState::Failed, out.state);
            EXPECT_EQ(core::HomingFailure::TimedOut, routine.failure());
            EXPECT_FLOAT_EQ(0.0F, out.speed_ref_rad_s) << "failure must not leave a command behind";
        }
        if (first_failed_tick > 0) {
            EXPECT_EQ(core::HomingState::Failed, out.state) << "the state must stay failed";
            EXPECT_FLOAT_EQ(0.0F, out.speed_ref_rad_s) << "a failed search must not keep turning";
        }
    }
    std::printf("[homing] the timeout was crossed on tick %d, expected %d (+/- 1)\n", first_failed_tick, ticks);
    ASSERT_GT(first_failed_tick, 0);
    EXPECT_LE(std::abs(first_failed_tick - ticks), 1) << "the search ran for longer or shorter than its own recipe";
}

TEST(HomingRoutine, AZeroRecipeMeansNoHomingWasAskedFor) {
    // C4
    core::PositionReference reference = make_reference();
    core::HomingRoutine routine(recipe(0.0F, 0.0F, 0));
    EXPECT_EQ(foc::ErrorCode::NotImpl, routine.validate());
    EXPECT_TRUE(routine.begin(reference)) << "a recipe that asks for nothing must finish rather than creep";
    EXPECT_FLOAT_EQ(0.0F, routine.step(reference, kDt).speed_ref_rad_s);
}

TEST(HomingRoutine, ARecipeThatIsPresentButImpossibleIsAParameterError) {
    // C5
    for (const core::HomingConfig& bad :
         {recipe(0.0F, kTimeoutS, 1), recipe(-1.0F, kTimeoutS, 1), recipe(kCreep, 0.0F, 1), recipe(kCreep, kTimeoutS, 0),
          recipe(kCreep, kTimeoutS, 2)}) {
        core::HomingRoutine routine(bad);
        EXPECT_EQ(foc::ErrorCode::Param, routine.validate())
            << "creep " << bad.creep_speed_rad_s << " timeout " << bad.timeout_s << " dir " << static_cast<int>(bad.direction);
        EXPECT_EQ(core::HomingState::Failed, routine.state());
        EXPECT_EQ(core::HomingFailure::NotConfigured, routine.failure());
    }
}

namespace {

// The recipe decides two things at once: how fast to creep and how long to try, and together they are a *travel
// budget*. The search can only ever find a mark that lies inside it -- there is no retry, no second direction, no
// slowing down on approach. So the budget is the first thing to measure, and it turns out to be one sample smaller than
// the arithmetic says, because the tick that gives up commands nothing.
// 配方同时决定两件事：爬行多快、试多久；两者合起来是一个**行程预算**。搜索只能找到落在其中的标记——没有重试、没有反向
// 再找一次、没有接近时减速。所以预算是要量的第一件事，而它比算术结果小一个采样，因为放弃的那个 tick 什么都不命令。
struct Sweep final {
    bool found_the_mark;
    double travelled_mech_rad;
    long ticks;
};

/** @brief [en] Drive a reference with the speed the routine actually commands, pulsing the index once travel passes `mark`. */
/** @brief [zh] 用流程实际命令的转速推进参考，行程越过 `mark` 时给出索引脉冲。 */
auto sweep(float creep, float timeout, float dt, double mark) -> Sweep {
    core::HomingRoutine routine(recipe(creep, timeout, 1));
    core::PositionReference reference = make_reference();
    routine.begin(reference);
    double mech = 0.0;
    long ticks = 0;
    while (ticks < 40000000L) {
        const core::HomingRoutine::Output out = routine.step(reference, dt);
        ++ticks;
        if (out.finished) {
            return Sweep{routine.state() == core::HomingState::Done, mech, ticks};
        }
        const double next = mech + (static_cast<double>(out.speed_ref_rad_s) * static_cast<double>(dt));
        static_cast<void>(reference.update(static_cast<float>(next), (mech < mark) && (next >= mark)));
        mech = next;
    }
    return Sweep{false, mech, ticks};
}

} // namespace

TEST(HomingRoutine, TheSearchCanOnlyFindAMarkInsideItsOwnTravelBudget) {
    // L3: the recipe as an envelope, and the envelope is set in samples, not in seconds.
    // L3：把配方当作包络来量，而包络是按采样定的，不是按秒定的。
    struct Case final {
        float creep;
        float timeout;
        float dt;
        double mark;
        bool found;
    };
    // creep 1 rad/s for 1 s buys 0.9999 rad of travel, not 1.0: the tick that gives up returns zero speed, so the last
    // sample of the budget is never spent. A mark one sample short of the full budget is found; a mark *at* it is not.
    // 1 rad/s 转 1 s 买到的是 0.9999 rad 而不是 1.0：放弃的那个 tick 返回零速，预算里最后一个采样永远花不掉。
    // 差一个采样的标记能找到，正好落在预算上的找不到。
    const Case cases[] = {{1.0F, 1.0F, 1.0e-4F, 0.9990, true},
                          {1.0F, 1.0F, 1.0e-4F, 0.9999, false},
                          {1.0F, 1.0F, 1.0e-4F, 1.0000, false},
                          {10.0F, 1.0F, 1.0e-4F, 9.9900, true},
                          {10.0F, 1.0F, 1.0e-4F, 10.0000, false}};
    for (const Case& c : cases) {
        const Sweep run = sweep(c.creep, c.timeout, c.dt, c.mark);
        const double budget = static_cast<double>(c.creep) * static_cast<double>(c.timeout);
        std::printf("[homing] creep %5.2f timeout %.4f dt %.5f mark %8.4f -> %s, travelled %.6f of a %.6f budget\n",
                    static_cast<double>(c.creep), static_cast<double>(c.timeout), static_cast<double>(c.dt), c.mark,
                    run.found_the_mark ? "found " : "missed", run.travelled_mech_rad, budget);
        EXPECT_EQ(c.found, run.found_the_mark)
            << "creep " << c.creep << " timeout " << c.timeout << " mark " << c.mark;
        // The law, not a percentage: the budget spent is `(ticks - 1) * creep * dt`, because the tick that gives up
        // commanded nothing. The tick *count* is not `timeout / dt` -- the next case shows why -- so the law is written
        // against the ticks actually taken.
        // 是定律不是百分比：花掉的预算是 `(ticks - 1) * creep * dt`，因为放弃的那一 tick 什么都没命令。tick **数**
        // 并不等于 `timeout / dt`——下一个用例会说明为什么——所以定律按实际经过的 tick 写。
        const double law = static_cast<double>(run.ticks - 1) * static_cast<double>(c.creep) * static_cast<double>(c.dt);
        EXPECT_NEAR(run.travelled_mech_rad, law, 1.0e-9) << "the spent budget is not `(ticks - 1) * creep * dt`";
        EXPECT_LT(run.travelled_mech_rad, budget) << "the search spent more than its recipe allowed";
        const double one_sample = static_cast<double>(c.creep) * static_cast<double>(c.dt);
        if (c.found) {
            // A search that found the mark stopped one sample after crossing it and not later.
            // 找到标记的搜索在越过它一个采样后停下，不会再晚。
            EXPECT_GE(run.travelled_mech_rad, c.mark) << "it reported a mark it had not reached";
            EXPECT_LT(run.travelled_mech_rad - c.mark, one_sample) << "it kept creeping after the mark";
        } else {
            // An exhausted budget is short by exactly one sample of travel, not by a percentage.
            // 耗尽的预算恰好短一个采样，不是短一个百分比。
            EXPECT_NEAR(budget - run.travelled_mech_rad, one_sample, one_sample * 0.5)
                << "the shortfall stopped being one sample of travel";
        }
    }
}

TEST(HomingRoutine, TheTimerItGivesUpOnIsAFloatSumNotAClock) {
    // L3: the timeout is decided on `elapsed_s_ += dt_s` in float32, so it is only a clock while the sum is small.
    // Two things follow, both measured here: the clock runs *fast* as the sum gets large, and past a fixed point it
    // stops entirely -- so a long recipe never times out at all, which is the opposite of what the routine promises.
    // L3：超时是靠 float32 的 `elapsed_s_ += dt_s` 判的，所以只有当和很小的时候它才是个钟。由此得到两件事，
    // 都实测了：和变大时钟会**走快**；越过某个定点后它彻底不动——于是一个很长的配方根本不会超时，与流程承诺相反。

    // A 512 s recipe at 40 kHz fires at 446.7 s of real time: the accumulated clock is 12.7 % fast by then.
    // 40 kHz 下 512 s 的配方在真实 446.7 s 就触发了：那时累计钟已经快了 12.7 %。
    {
        const Sweep run = sweep(1.0F, 512.0F, 2.5e-5F, 1.0e6); // a mark that is never reached
        const double real_elapsed_s = static_cast<double>(run.ticks) * 2.5e-5;
        std::printf("[homing] 512 s recipe at 40 kHz fired after %.3f s of real time (%.1f %% fast), %ld ticks\n",
                    real_elapsed_s, (100.0 * (1.0 - (real_elapsed_s / 512.0))), run.ticks);
        ASSERT_FALSE(run.found_the_mark);
        EXPECT_GT(real_elapsed_s, 440.0) << "the recipe did not last long enough to reach the float32 floor";
        EXPECT_LT(real_elapsed_s, 500.0) << "the clock was expected to run fast, not slow";
    }

    // Past that point the sum cannot advance at all: 25 M ticks is 625 s of real time, and a 520 s recipe is still
    // creeping. The header says "giving up rather than turning forever"; that holds only below the floor.
    // 越过那一点，和彻底加不动：2500 万 tick 是真实 625 s，而 520 s 的配方仍在爬行。头文件写的是"放弃而不是一直转"，
    // 那句话只在地板以下成立。
    {
        const Sweep run = sweep(1.0F, 520.0F, 2.5e-5F, 1.0e6);
        std::printf("[homing] 520 s recipe at 40 kHz: %ld ticks = %.1f s of real time, finished = %s\n", run.ticks,
                    static_cast<double>(run.ticks) * 2.5e-5, run.found_the_mark ? "yes" : "NO");
        EXPECT_EQ(40000000L, run.ticks) << "the routine stopped on its own past the float32 floor";
    }

    // The floor is `2^24 * dt` rounded up to the next power of two, because the sum sits at a power of two when its ULP
    // first exceeds half a tick. Mirrored here on a bare float so the three decades can be checked without 25 M ticks
    // each; it is the same one line as `elapsed_s_ += dt_s`.
    // 地板是 `2^24 * dt` 向上取到 2 的幂：当 ULP 首次超过半个 tick 时，和正停在 2 的幂上。这里用裸 float 复现同一行，
    // 好把三个数量级都查一遍而不必每个跑 2500 万 tick；它与 `elapsed_s_ += dt_s` 是同一行。
    struct Floor final {
        float dt;
        float measured_s;
    };
    for (const Floor& f : {Floor{2.5e-5F, 512.0F}, Floor{1.0e-4F, 2048.0F}, Floor{1.0e-3F, 32768.0F}}) {
        float sum = 0.0F;
        long ticks = 0;
        while (ticks < 40000000L) {
            const float before = sum;
            sum += f.dt;
            ++ticks;
            if (sum == before) {
                break;
            }
        }
        std::printf("[homing] dt %.7f: the clock freezes at %.1f s = 2^24*dt rounded up to a power of two\n",
                    static_cast<double>(f.dt), static_cast<double>(sum));
        EXPECT_FLOAT_EQ(f.measured_s, sum) << "the float32 floor moved; re-derive the envelope before trusting it";
        EXPECT_LE(static_cast<double>(f.dt) * 16777216.0, static_cast<double>(sum));
        EXPECT_GT(static_cast<double>(f.dt) * 16777216.0 * 2.0, static_cast<double>(sum));
    }
}

namespace {

/** @brief [en] Where the established zero sits, measured as the travel at the tick the index was honoured minus the true mark. */
/** @brief [zh] 建立的零点在哪：索引被采信那个 tick 的行程减去真实标记位置。 */
auto origin_offset(float creep, float timeout, float dt, double mark, long jitter_ticks) -> double {
    core::HomingRoutine routine(recipe(creep, timeout, 1));
    core::PositionReference reference = make_reference();
    routine.begin(reference);
    double mech = 0.0;
    long crossed_at = -1;
    long ticks = 0;
    while (ticks < 4000000L) {
        const core::HomingRoutine::Output out = routine.step(reference, dt);
        ++ticks;
        if (out.finished) {
            break;
        }
        const double next = mech + (static_cast<double>(out.speed_ref_rad_s) * static_cast<double>(dt));
        if (crossed_at < 0 && next >= mark) {
            crossed_at = ticks;
        }
        // A worn sensor, a slow comparator or an index edge with no conditioning all delay the pulse by some ticks.
        // 磨损的传感器、慢比较器、或没有整形的索引边沿，都会让脉冲晚几个 tick。
        const bool pulse = (crossed_at >= 0) && (ticks >= crossed_at + jitter_ticks);
        static_cast<void>(reference.update(static_cast<float>(next), pulse));
        mech = next;
        if (reference.homed()) {
            return mech - mark;
        }
    }
    return -1.0;
}

} // namespace

TEST(HomingRoutine, TheOriginErrorIsThePulseDelayMeasuredInCreepSpeed) {
    // L3: the third source O-1 asked for -- index jitter -- and it turns out to be the same knob as the other two.
    // The origin cannot be better than where the axis was when the pulse arrived, so the error is the pulse delay
    // *expressed as distance*, and the distance per tick is the creep speed. One sample is spent finding the crossing.
    // L3：O-1 要的第三个误差源——索引抖动——而它和前两个共用同一个旋钮。零点不可能比"脉冲到达时轴所在的位置"更准，
    // 所以误差就是脉冲延迟**折算成距离**，而每个 tick 的距离就是爬行转速。另有一个采样花在发现越界上。
    const float dt = 1.0e-4F;
    for (const long jitter : {0L, 1L, 5L, 20L}) {
        const double offset = origin_offset(1.0F, 1.0F, dt, 0.5, jitter);
        const double per_tick = 1.0 * static_cast<double>(dt);
        std::printf("[homing] index jitter %2ld ticks at 1 rad/s -> origin off by %+.6f rad = %+.2f samples "
                    "(law: %ld + 1)\n",
                    jitter, offset, offset / per_tick, jitter);
        EXPECT_NEAR(offset, static_cast<double>(jitter + 1L) * per_tick, 1.0e-6)
            << "the origin error is not (jitter + 1 detection sample) x creep x dt";
    }

    // And it is one-for-one in the creep speed, which is the whole trade: the search *time* goes as 1/creep, so
    // error x time is fixed. Slowing down buys accuracy in exactly the proportion it costs time.
    // 而且它对爬行转速是一比一的，这正是全部取舍：搜索**时间**按 1/creep 走，所以 误差 x 时间 是定值。
    // 减速买到的精度，与它花掉的时间完全成比例。
    double reference_offset = 0.0;
    for (const float creep : {0.5F, 1.0F, 2.0F, 4.0F}) {
        const double offset = origin_offset(creep, 2.0F, dt, 0.5, 20);
        if (creep == 1.0F) {
            reference_offset = offset;
        }
        const double ratio = offset / (static_cast<double>(creep) * static_cast<double>(dt));
        std::printf("[homing] creep %4.2f rad/s -> origin off by %+.6f rad = %.3f ticks of travel\n",
                    static_cast<double>(creep), offset, ratio);
        EXPECT_NEAR(21.0, ratio, 1.0e-3) << "the origin error stopped being a fixed number of ticks of travel";
    }
    // Creep 1 rad/s over 0.5 rad of travel takes 0.5 s and 21 samples of error; 4 rad/s takes 0.125 s and the same 21
    // samples *of its own travel*, so the error x time product is what a recipe actually buys.
    // 1 rad/s 走 0.5 rad 用 0.5 s、错 21 个采样；4 rad/s 用 0.125 s、错同样 21 个**它自己的**行程采样，
    // 所以配方真正买到的是 误差 x 时间 这个乘积。
    EXPECT_GT(reference_offset, 0.0) << "the 1 rad/s reference row was never measured";
}

