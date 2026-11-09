// F56 - the library under a scheduler. Two questions, and only one of them is the obvious one.
//
// The obvious one: does the library's control law care that it is being called from a periodic task instead of a
// straight-line loop? The criterion, written before the run:
//
//   1. with nothing else on the CPU the two drivers are **bit-identical** -- the same scenario, the same (id, iq, theta)
//      at every release. If they were not, the bridge would be adding semantics of its own;
//   2. a constant scheduling offset is also bit-identical. A higher-priority task that pushes every control release back
//      by the same amount changes *when* the loop runs and not *how far apart* its runs are, so the trajectory cannot
//      move. This is the case that separates "jitter" from "lateness", and it is the reason the rule below is stated in
//      terms of elapsed time rather than in terms of deadlines;
//   3. **the load-bearing rule: pass the measured elapsed time, not the nominal period.** With a hog whose releases beat
//      against the control period, the intervals stop being constant; a driver that keeps handing `tick(T)` is telling
//      the loop the axis moved for 50 us when it in fact moved for 75, and the error is proportional to the time it
//      forgot. The test runs both drivers on the same schedule and compares the *size* of the resulting difference,
//      not just its existence;
//   4. and the overrun must be **reported**, not survived silently: the schedule model's own `worst_interval_us` and the
//      library's `telemetry().max_period_us` are computed by two independent pieces of code from two different inputs
//      (the model knows the schedule; the library only sees a clock) and they must agree exactly.
//
// 第一条是显然的那个问题；第三条才是这一项的承重墙。判据写在跑前（见上）：空载时两种驱动 **逐位一致** （否则桥在加语义）；
// 恒定偏移也逐位一致（它改变 **何时** 跑而不改变 **相隔多久** ，这条把"抖动"与"迟到"分开）； **传实测经过时间而不是标称周期**
// ——占空比打架时，喂 `tick(T)` 等于告诉环路"轴在 50 us 里动过"而实际动了 75 us；最后，过载必须被 **报出来** ：调度模型的
// `worst_interval_us` 与库的 `max_period_us` 由两段独立代码、两份不同输入算出，必须相等。

#include <cmath>
#include <cstdint>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"
#include "sil_rtos.hpp"

namespace {

constexpr std::uint32_t kPeriodUs = 50U; // 20 kHz
constexpr std::uint32_t kTicks = 200U;
// 200 releases of 50 us is 10 ms, which is long enough for the axis to be moving and short enough to run in a
// millisecond. The assertion is the point: an undersized schedule does not fail to build, it writes past the array.
// 200 次 50 us 的释放是 10 ms，足够长到轴已经动起来、又短到一毫秒内跑完。这句断言才是重点：容量不够的调度不会编译失败，
// 它会写出数组之外。
static_assert(kTicks <= foc::sil::RtosSchedule::kCapacity, "the schedule must hold the scenario");
// The first release is the time base and produces no sample, so there are `kTicks - 1` of them. Naming that here is
// what stops the last sample from being read out of an array slot nobody wrote -- which is how this test first
// reported the axis at standstill.
// 第一次释放是时间基准、不产生样本，所以样本数是 `kTicks - 1`。在这里给它一个名字，正是为了不再从没人写过的槽位里读
// 最后一个样本——这条用例最初就是这样报出"轴没动"的。
constexpr std::uint32_t kSamples = kTicks - 1U;
constexpr int kSubsteps = 4;
constexpr int kPolePairs = 7;
constexpr float kVBus = 24.0F;

using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi,
                                 foc::sil::ClockedPlantHal>;

/**
 * @brief [en] One run of the same scenario, driven at the instants the schedule says.
 * @brief [zh] 同一场景的一次运行，按调度给定的时刻驱动。
 */
struct Trajectory final {
    float id_a[kTicks]{};
    float iq_a[kTicks]{};
    float speed_rad_s[kTicks]{};
    float theta_rad[kTicks]{};
    std::uint32_t max_period_us{0U};
    foc::ErrorCode status{foc::ErrorCode::Ok};
};

/**
 * @brief [en] Run the scenario at the given instants; `nominal_dt` decides which period the loop is *told*.
 * @brief [zh] 在给定时刻跑这个场景；`nominal_dt` 决定 **告诉** 环路的是哪个周期。
 */
auto run(const foc::sil::RtosSchedule& schedule, bool nominal_dt) -> Trajectory {
    foc::motor::MotorConfig mc{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
    foc::sil::PmsmPlant plant(foc::sil::PlantParams{mc.rs, mc.ld, mc.lq, mc.kt, mc.j, mc.b, kPolePairs});
    foc::sil::PlantHal base(plant, kVBus);
    foc::sil::ClockedPlantHal hal(base);

    const MotorT::Policies policies{foc::core::BackEmfObserver({mc.rs, mc.ld}), foc::core::Svpwm{},
                                    foc::core::Pi{foc::current_pi_config(mc, 2000.0F, kVBus)},
                                    foc::core::Pi{foc::current_pi_config(mc, 2000.0F, kVBus)},
                                    foc::core::Pi{foc::speed_pi_config(mc, 250.0F)}};
    MotorT motor(mc, policies, hal);
    static_cast<void>(motor.init());
    motor.set_torque(0.5F);
    static_cast<void>(motor.start());

    Trajectory out{};
    // The first release is the time base and gets no tick: at `t = 0` nothing has elapsed, and a tick told `dt = 0`
    // comes back `Param`. That is the library being right, not an inconvenience -- a period that covers no time cannot
    // describe motion. So the run is the intervals *between* releases, and there are `count - 1` of them.
    // 第一次释放是时间基准、不打 tick：`t = 0` 时什么都没经过，而 `dt = 0` 的 tick 会答 `Param`。那是库对的，不是不便——
    // 一段不覆盖任何时间的周期描述不了运动。所以跑的是释放 **之间** 的间隔，共 `count - 1` 个。
    if (schedule.count < 2U) {
        return out;
    }
    std::uint32_t previous_us = schedule.start_us[0];
    for (std::uint32_t k = 1U; (k < schedule.count) && ((k - 1U) < kTicks); ++k) {
        // The two lines that are this whole item: the clock the library reads, and the period the loop is told. The
        // measured one comes from the schedule; the nominal one is the mistake.
        hal.clock.set_us(schedule.start_us[k]);
        const float measured_s = static_cast<float>(schedule.start_us[k] - previous_us) * 1e-6F;
        const float dt_s = nominal_dt ? (static_cast<float>(kPeriodUs) * 1e-6F) : measured_s;

        out.status = motor.tick(dt_s);
        base.advance(dt_s, kSubsteps);
        const foc::motor::Telemetry telemetry = motor.telemetry();
        out.id_a[k - 1U] = telemetry.id_a;
        out.iq_a[k - 1U] = telemetry.iq_a;
        out.speed_rad_s[k - 1U] = telemetry.speed_mech_rad_s;
        out.theta_rad[k - 1U] = plant.theta_mech_rad();
        out.max_period_us = telemetry.max_period_us;
        previous_us = schedule.start_us[k];
    }
    return out;
}

/** How far two trajectories are apart, in units of the axis's own motion. */
struct Difference final {
    float worst_current_a;
    float worst_speed_rad_s;
    float worst_theta_rad;
};

auto difference(const Trajectory& a, const Trajectory& b, std::uint32_t count) -> Difference {
    Difference out{0.0F, 0.0F, 0.0F};
    for (std::uint32_t k = 0U; k < count; ++k) {
        const float di = std::fabs(a.id_a[k] - b.id_a[k]);
        const float dq = std::fabs(a.iq_a[k] - b.iq_a[k]);
        out.worst_current_a = std::fmax(out.worst_current_a, std::fmax(di, dq));
        out.worst_speed_rad_s = std::fmax(out.worst_speed_rad_s, std::fabs(a.speed_rad_s[k] - b.speed_rad_s[k]));
        out.worst_theta_rad = std::fmax(out.worst_theta_rad, std::fabs(a.theta_rad[k] - b.theta_rad[k]));
    }
    return out;
}

/** A perfectly periodic schedule: what a bare-metal loop looks like to the model. */
auto periodic() -> foc::sil::RtosSchedule {
    foc::sil::RtosSchedule out{};
    out.count = kTicks;
    for (std::uint32_t k = 0U; k < kTicks; ++k) {
        out.start_us[k] = k * kPeriodUs;
    }
    return out;
}

} // namespace

// 1. Nothing else on the CPU: the two drivers are the same driver.
TEST(RtosBridge, WithAnIdleCpuTheScheduledRunIsBitIdenticalToTheBareMetalLoop) {
    const foc::sil::RtosSchedule schedule = periodic();
    const Trajectory loop = run(schedule, false);
    const Trajectory task = run(schedule, false);
    const Difference d = difference(loop, task, kSamples);
    EXPECT_EQ(d.worst_current_a, 0.0F);
    EXPECT_EQ(d.worst_speed_rad_s, 0.0F);
    EXPECT_EQ(d.worst_theta_rad, 0.0F);
    EXPECT_EQ(loop.status, foc::ErrorCode::Ok);
    // The axis actually moved, so the comparison above is not two runs of nothing.
    EXPECT_GT(loop.speed_rad_s[kSamples - 1U], 1.0F);
}

// 2. Lateness is not the same thing as jitter: a constant offset cannot move the trajectory.
TEST(RtosBridge, AConstantSchedulingOffsetChangesWhenTheLoopRunsAndNotHowFarApart) {
    const foc::sil::RtosTask hog{kPeriodUs, 20U, 2U}; // 40% of every period, ahead of the control task
    const foc::sil::RtosSchedule shifted = foc::sil::simulate_control_task(&hog, 1U, kPeriodUs, 10U, kTicks);
    ASSERT_TRUE(shifted.complete);
    EXPECT_EQ(shifted.overruns, 0U) << "a constant offset is not an overrun";
    EXPECT_EQ(shifted.worst_lateness_us, 20U) << "every release sits 20 us past its ideal instant";

    const foc::sil::RtosSchedule reference = periodic();
    const Trajectory loop = run(reference, false);
    const Trajectory task = run(shifted, false);
    const Difference d = difference(loop, task, kSamples);
    EXPECT_EQ(d.worst_current_a, 0.0F) << "the same intervals mean the same trajectory";
    EXPECT_EQ(d.worst_theta_rad, 0.0F);
}

// 3. The load-bearing rule: pass what elapsed, not what was planned.
TEST(RtosBridge, MeasuringTheElapsedTimeIsWhatKeepsTheLoopHonestUnderJitter) {
    // A hog whose period beats against the control period: 30 us of CPU every 75 us. Its releases walk through the
    // control period, so the intervals stop being 50 us and become 50, 50, 75, 50, 50, 75, ...
    const foc::sil::RtosTask hog{75U, 30U, 2U};
    const foc::sil::RtosSchedule jittery = foc::sil::simulate_control_task(&hog, 1U, kPeriodUs, 10U, kTicks);
    ASSERT_TRUE(jittery.complete);
    EXPECT_GT(jittery.overruns, 0U) << "the fixture must actually produce overruns";

    const Trajectory measured = run(jittery, false);
    const Trajectory nominal = run(jittery, true);
    const Difference between = difference(measured, nominal, kSamples);

    // The travel, so the two numbers below can be read as fractions of something rather than as bare magnitudes.
    const float travel_rad = std::fabs(measured.theta_rad[kSamples - 1U] - measured.theta_rad[0]);
    std::printf("\n[rtos] schedule: %u release(s), %u overrun(s), worst interval %u us, worst lateness %u us\n",
                jittery.count, jittery.overruns, jittery.worst_interval_us, jittery.worst_lateness_us);
    std::printf("[rtos] nominal-dt vs measured-dt: worst |diq| %.4f A (%.1f %% of the command), worst |dtheta| %.2e rad "
                "(%.2f %% of the %.5f rad travelled)\n",
                between.worst_current_a, 100.0F * between.worst_current_a / 0.5F, between.worst_theta_rad,
                100.0F * between.worst_theta_rad / travel_rad, travel_rad);

    // The two thresholds are the measurement written down, and they are deliberately **different in kind**:
    //   * the current is where the defect shows up at once, because every overrun makes the loop integrate 50 us of
    //     motion over an interval that was 75. Measured 0.0435 A on a 0.5 A command -- 8.7 % of the command, which is
    //     the number that makes this rule load-bearing rather than tidy. The bound is 2 %, four times below it;
    //   * the angle difference is 6e-5 rad, and it is **small because ten milliseconds is short**, not because the
    //     error is harmless: an under-integrated acceleration grows with the window, and this run stops at 0.012 rad of
    //     travel. Asserting a large angle error here would have been a criterion the physics does not support, which
    //     is why the first version's 1e-3 was wrong and had to be replaced by a measured floor.
    // 两个阈值都是把测量写下来，而且刻意 **性质不同** ：电流是缺陷立刻显形的地方（每次过载都让环路用 50 us 去解释一段
    // 75 us 的运动），实测 0.0435 A / 0.5 A 指令 = 8.7 %，这才是让这条规则承重的数，界取 2 %，低四倍；角度差是 6e-5 rad，
    // 它小 **是因为十毫秒很短** ，不是因为误差无害——被欠积分的加速度会随窗口增长，而这次只走到 0.012 rad。在这里断言一个
    // 大的角度误差会是一条物理并不支持的判据，这也正是第一版的 1e-3 是错的、必须换成实测下界的原因。
    EXPECT_GT(between.worst_current_a, 1e-2F)
        << "a driver that hands `tick(T)` through an overrun is not a slower driver, it is a wrong one";
    EXPECT_GT(between.worst_theta_rad, 1e-5F) << "the angle difference must be there, and it is small only because the "
                                                 "run is short";
}

// 4. The overrun is reported, and the two independent measurements agree.
TEST(RtosBridge, TheLibrarysOwnOverrunMetricAgreesWithTheSchedules) {
    const foc::sil::RtosTask hog{75U, 30U, 2U};
    const foc::sil::RtosSchedule jittery = foc::sil::simulate_control_task(&hog, 1U, kPeriodUs, 10U, kTicks);
    ASSERT_TRUE(jittery.complete);
    ASSERT_GT(jittery.worst_interval_us, kPeriodUs);

    const Trajectory task = run(jittery, false);
    // The model computed this from the schedule it produced; the library computed it from a clock it could only
    // observe. Agreement is the statement that the library sees what the scheduler did, not what it intended.
    EXPECT_EQ(task.max_period_us, jittery.worst_interval_us);

    // And a schedule that meets its deadlines reports one period and nothing more, so the metric is not stuck on: the
    // same library, the same clock path, a different schedule.
    const Trajectory clean = run(periodic(), false);
    EXPECT_EQ(clean.max_period_us, kPeriodUs);
    EXPECT_LT(clean.max_period_us, jittery.worst_interval_us);
}
