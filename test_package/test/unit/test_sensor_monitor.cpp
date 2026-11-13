// Whether the position sensor's own readings hold together, judged without a second opinion.
// 位置传感器自己的读数是否自洽——不用第二个意见来判。
//
// The observer-disagreement monitor needs a second estimator and is only valid above a speed this library measured
// (101 rad/s on the nominal machine), which is above where a servo spends its life. This one is the complement: it
// needs nothing but the readings, works at standstill, and its bound is *derived* from the mechanism's own ceiling
// rather than picked. Every case below computes that bound itself, so a wrong bound fails here rather than agreeing
// with the code.
//
// 观测器分歧监视需要第二个估计器，且只在这台库量过的转速以上有效（标称机器 101 rad/s），而伺服的一生大多在那以下。本类
// 是它的补集：只需要读数、静止也能用，而它的界是**从机构自己的上限推出来的**，不是挑的。下面每条用例都自己算那个界，于是
// 界错了会在这里红，而不是和代码互相印证。

#include <cmath>

#include <gtest/gtest.h>

#include <foc/core/sensor_monitor.hpp>


namespace {

using foc::ErrorCode;
using foc::core::SensorFault;
using foc::core::SensorMonitor;
using foc::core::SensorMonitorConfig;
using foc::core::SensorVerdict;

constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kDt = 50e-6F;

/**
 * @brief [en] A 200 rad/s mechanism: the ceiling the bound is derived from, and a tolerance of a quarter of one sample
 *             at that speed -- a number with a meaning, not a magic constant.
 * @brief [zh] 200 rad/s 的机构：界由它推出，余量取该速度下一个采样的四分之一——有含义的数，不是魔法常数。
 */
auto config() -> SensorMonitorConfig { return SensorMonitorConfig{200.0F, 0.25F * 200.0F * kDt}; }

auto wrap(float angle_rad) -> float {
    float wrapped = std::fmod(angle_rad, kTwoPi);
    if (wrapped < 0.0F) {
        wrapped += kTwoPi;
    }
    return wrapped;
}

// 1. Nothing to judge is not a pass. The first sample has no predecessor, an unconfigured monitor has no bound, and a
// refused period has no step -- all three answer `Unobserved`, never `Ok`. A monitor that reported `Ok` here would make
// "nobody compared anything" indistinguishable from "the comparisons passed", which is the failure `fault_input`
// already refuses at the port.
// 1. 没有可判的东西不是通过。首个采样没有前一个，未配置的监视器没有界，被拒绝的周期没有步长——三者都答 `Unobserved`，
//    绝不答 `Ok`。在这里报 `Ok` 的监视器会让"没人比过"与"比过了都通过"无法区分，而这正是 `fault_input` 在端口上拒绝
//    的那种失败。
TEST(SensorMonitor, NothingToJudgeIsNotAPass) {
    {
        // A monitor nobody has configured or stepped reports the same nothing the first step does: a newly constructed
        // object must not claim it has already found a fault, and its count must start at zero rather than at whatever
        // the member initializer happened to say.
        // 还没配置、也没走过一拍时，报的"什么都没有"与首次采样相同：新建的对象不能声称已经发现过故障，计数也必须从零
        // 开始，而不是成员初始化里碰巧写的那个数。
        SensorMonitor fresh;
        const auto& idle = fresh.output();
        EXPECT_EQ(SensorVerdict::Unobserved, idle.verdict);
        EXPECT_EQ(SensorFault::None, idle.fault);
        EXPECT_EQ(0.0F, idle.step_rad);
        EXPECT_EQ(0.0F, idle.bound_rad);
        EXPECT_EQ(0U, idle.implausible_count) << "the history starts empty";
        EXPECT_FALSE(fresh.configured());
    }
    {
        SensorMonitor monitor;
        EXPECT_EQ(SensorVerdict::Unobserved, monitor.step(0.0F, 0.0F, kDt).verdict)
            << "unconfigured is not 'the sensor is fine'";
    }
    {
        SensorMonitor monitor;
        ASSERT_EQ(ErrorCode::Ok, monitor.configure(config()));
        EXPECT_EQ(SensorVerdict::Unobserved, monitor.step(1.0F, 10.0F, kDt).verdict) << "the first sample has no step";
        EXPECT_EQ(SensorVerdict::Ok, monitor.step(1.0F, 10.0F, kDt).verdict) << "and the second one does";
    }
    {
        SensorMonitor monitor;
        ASSERT_EQ(ErrorCode::Ok, monitor.configure(config()));
        ASSERT_EQ(SensorVerdict::Unobserved, monitor.step(0.0F, 0.0F, kDt).verdict);
        for (const float dt : {0.0F, -kDt, std::nanf("")}) {
            const auto& out = monitor.step(1.0F, 0.0F, dt);
            EXPECT_EQ(SensorVerdict::Unobserved, out.verdict) << dt;
            EXPECT_EQ(SensorFault::None, out.fault) << "a refused period is not a sensor fault";
        }
    }
}

// 2. A steady reading passes, and so does the ordinary crossing of the `2*pi` boundary -- which a naive difference
// reports as a full turn in one sample, i.e. as the worst possible jump.
// 2. 平稳读数通过，普通地跨过 `2*pi` 边界也通过——而朴素差分会把它报成一拍里整整一圈，也就是最严重的跳变。
TEST(SensorMonitor, TheOrdinaryWrapIsNotAJump) {
    SensorMonitor monitor;
    ASSERT_EQ(ErrorCode::Ok, monitor.configure(config()));
    // 200 rad/s at 50 us is 0.01 rad per sample: 700 samples take the reading through 7 rad and across the wrap.
    // 200 rad/s 配 50 us 是每采样 0.01 rad：700 个采样让它走过 7 rad 并跨过归一化边界。
    float angle = 6.0F;
    static_cast<void>(monitor.step(angle, 200.0F, kDt));
    for (int i = 0; i < 700; ++i) {
        angle = wrap(angle + (200.0F * kDt));
        const auto& out = monitor.step(angle, 200.0F, kDt);
        ASSERT_EQ(SensorVerdict::Ok, out.verdict) << "sample " << i << " at " << angle << " rad, step "
                                                  << out.step_rad << " against a bound of " << out.bound_rad;
        ASSERT_FALSE(std::isnan(out.step_rad));
    }
    EXPECT_EQ(0U, monitor.output().implausible_count);
}

// 3. A jump is called a jump, and the *bound* is what decides: exactly `max_speed*dt + tolerance` is allowed and one
// thousandth beyond it is not. The two directions are checked because a monitor that only looked at the sign would pass
// half of every wire failure.
// 3. 跳变就叫跳变，而决定它的是**界**：恰好 `max_speed*dt + 余量` 允许，再把一千分之一就越界。两个方向都查，因为只看符号
//    的监视器会放过一半的接线故障。
TEST(SensorMonitor, TheBoundItselfDecidesAndBothDirectionsCount) {
    const SensorMonitorConfig cfg = config();
    const float bound = (cfg.max_speed_rad_s * kDt) + cfg.jump_tolerance_rad;
    for (const float sign : {1.0F, -1.0F}) {
        {
            SensorMonitor monitor;
            ASSERT_EQ(ErrorCode::Ok, monitor.configure(cfg));
            const auto& first = monitor.step(0.0F, 0.0F, kDt);
            ASSERT_EQ(SensorVerdict::Unobserved, first.verdict);
            EXPECT_EQ(bound, first.bound_rad) << "the bound is this tick's ceiling times the period, plus the tolerance";
            // Exactly on the bound: legal.
            const auto& on_it = monitor.step(wrap(sign * bound), 0.0F, kDt);
            EXPECT_EQ(SensorVerdict::Ok, on_it.verdict) << "exactly the bound is inside it";
            // 1e-6 rad, not `EXPECT_FLOAT_EQ`: the step went through a wrap, and `fmod` near 0.0125 costs
            // about 50 ulp of the value. The bound is what is under test here, not the wrap's last bit.
            EXPECT_NEAR(bound, std::fabs(on_it.step_rad), 1.0e-6F);
        }
        {
            SensorMonitor monitor;
            ASSERT_EQ(ErrorCode::Ok, monitor.configure(cfg));
            static_cast<void>(monitor.step(0.0F, 0.0F, kDt));
            const auto& past = monitor.step(wrap(sign * (bound * 1.001F)), 0.0F, kDt);
            EXPECT_EQ(SensorVerdict::Implausible, past.verdict);
            EXPECT_EQ(SensorFault::AngleJump, past.fault);
            EXPECT_EQ(1U, past.implausible_count) << "the count is kept, because one glitch and many are different machines";
            // And the next good sample is `Ok` again: the verdict is about a reading, not a latch.
            // 而且下一个好采样又是 `Ok`：判决是关于一条读数的，不是锁存。
            const auto& back = monitor.step(wrap(sign * (bound * 1.001F) + (10.0F * kDt)), 0.0F, kDt);
            EXPECT_EQ(SensorVerdict::Ok, back.verdict);
            EXPECT_EQ(1U, back.implausible_count) << "and the history stays visible";
        }
    }
}

// 4. A speed the mechanism cannot produce is its own fault, not the same one as a jump: it says the pulse train is
// losing counts, where a jump says the wiring or the coupling moved. They are reported apart so the diagnosis stays
// usable.
// 4. 机构产生不出来的转速是它自己的故障，与跳变不是同一个：它说的是脉冲串在丢计数，而跳变说的是接线或联轴器动了。分开上报，
//    诊断才有用。
TEST(SensorMonitor, AnImpossibleSpeedIsItsOwnDiagnosis) {
    SensorMonitor monitor;
    ASSERT_EQ(ErrorCode::Ok, monitor.configure(config()));
    static_cast<void>(monitor.step(0.0F, 0.0F, kDt));
    const auto& out = monitor.step(10.0F * kDt, 400.0F, kDt); // 400 rad/s on a 200 rad/s mechanism, one legal step
    EXPECT_EQ(SensorVerdict::Implausible, out.verdict);
    EXPECT_EQ(SensorFault::ImplausibleSpeed, out.fault);
    EXPECT_FLOAT_EQ(0.0F, out.step_rad - (10.0F * kDt)) << "the step itself was fine; the speed is what failed";
}

// 5. A reading that is not a number *is* information -- the sensor said something impossible -- and it must not become
// the thing the next good sample is compared against.
// 5. 不是数的读数**是**信息——传感器说了不可能的东西——而且它不能成为下一个好采样用来比较的对象。
TEST(SensorMonitor, AReadingThatIsNotANumberIsAFaultAndIsNotAdopted) {
    SensorMonitor monitor;
    ASSERT_EQ(ErrorCode::Ok, monitor.configure(config()));
    static_cast<void>(monitor.step(1.0F, 0.0F, kDt));
    const auto& broken = monitor.step(std::nanf(""), 0.0F, kDt);
    EXPECT_EQ(SensorVerdict::Implausible, broken.verdict);
    EXPECT_EQ(SensorFault::NotANumber, broken.fault);

    // The next sample is compared against the last *good* one (1.0 rad), so a legal step from there is `Ok`.
    // 下一个采样是与上一个**好**读数（1.0 rad）比较，所以从那里出发的一个合法步长是 `Ok`。
    const auto& after = monitor.step(1.0F + (5.0F * kDt), 0.0F, kDt);
    EXPECT_EQ(SensorVerdict::Ok, after.verdict);
    EXPECT_NEAR(5.0F * kDt, after.step_rad, 1.0e-6F); // again: the step carries the wrap's rounding
}

// 6. `reset()` is the escape hatch for a move this sensor did not see: after it, the next sample is a first sample
// again, so a hand turn or a re-clamp is not reported as a fault for the rest of the shift.
// 6. `reset()` 是"这次移动本传感器没看见"的出口：之后下一个采样又是首个采样，于是一次手转或重新装夹不会在余下的班次里
//    一直报故障。
TEST(SensorMonitor, ResetMakesTheNextSampleAFirstSample) {
    SensorMonitor monitor;
    ASSERT_EQ(ErrorCode::Ok, monitor.configure(config()));
    static_cast<void>(monitor.step(0.0F, 0.0F, kDt));
    monitor.reset();
    EXPECT_EQ(SensorVerdict::Unobserved, monitor.step(3.0F, 0.0F, kDt).verdict) << "3 rad in one tick, but it is a "
                                                                                "first tick";
    EXPECT_EQ(0U, monitor.output().implausible_count) << "and reset clears the count with the sample";
    EXPECT_EQ(SensorVerdict::Ok, monitor.step(3.0F + (1.0F * kDt), 0.0F, kDt).verdict);
}

// 7. The two numbers a configuration cannot describe are refused before a tick runs, and a zero tolerance is not one of
// them -- "the ceiling is exact" is a position a caller is allowed to take.
// 7. 描述不了的两个数在跑一拍之前就被拒绝，而零余量不在其中——"上限是精确的"是调用方可以采取的立场。
TEST(SensorMonitor, TwoConfigurationsAreRefusedAndZeroToleranceIsNotOneOfThem) {
    SensorMonitor monitor;
    EXPECT_EQ(ErrorCode::Param, monitor.configure(SensorMonitorConfig{0.0F, 0.0F})) << "no ceiling, no bound";
    EXPECT_EQ(ErrorCode::Param, monitor.configure(SensorMonitorConfig{-1.0F, 0.0F}));
    EXPECT_EQ(ErrorCode::Param, monitor.configure(SensorMonitorConfig{std::nanf(""), 0.0F}));
    EXPECT_EQ(ErrorCode::Param, monitor.configure(SensorMonitorConfig{200.0F, -1e-3F})) << "a negative margin is a typo";
    EXPECT_EQ(ErrorCode::Param, monitor.configure(SensorMonitorConfig{200.0F, std::nanf("")}));
    EXPECT_EQ(ErrorCode::Ok, monitor.configure(SensorMonitorConfig{200.0F, 0.0F})) << "exact is legal";
    EXPECT_TRUE(monitor.configured());
    // And an exact ceiling still behaves: the bound is `max_speed*dt` and nothing else.
    // 精确的上限照样工作：界就是 `max_speed*dt`，别的什么都没有。
    static_cast<void>(monitor.step(0.0F, 0.0F, kDt));
    EXPECT_FLOAT_EQ(200.0F * kDt, monitor.step(200.0F * kDt * 1.5F, 0.0F, kDt).bound_rad);
}

} // namespace
