// 3.1b: the decoder, measured against the physics 3.1a established. Two capabilities in one header -- the six-step
// table, and an angle interpolated from edge timing -- and both criteria are about how *wrong* each one is, because
// "it turns the motor" is satisfied by a table with two entries swapped.
// 3.1b：解码器，对着 3.1a 立下的物理来测。一个头文件里两件能力——六步表，和由边沿时刻插值出来的角度——两条判据都
// 关于各自**错多少**，因为"电机转了"这件事，一张有两项写反的表也能满足。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>

#include <gtest/gtest.h>

#include <foc/core/hall.hpp>
#include <foc/core/transforms.hpp>

#include "sil_bench.hpp"
#include "sil_hall_sensor.hpp"
#include "sil_pmsm_plant.hpp"

namespace {

constexpr float kPi = 3.14159265358979323846F;
constexpr float kTwoPi = 6.28318530717958647692F;
constexpr float kSectorRad = kPi / 3.0F;
constexpr float kDegRad = kPi / 180.0F;

auto wrap_pi(float angle_rad) -> float {
    float wrapped = std::fmod(angle_rad + kPi, kTwoPi);
    if (wrapped < 0.0F) {
        wrapped += kTwoPi;
    }
    return wrapped - kPi;
}

auto deg(float rad) -> float { return rad / kDegRad; }

/**
 * @brief The code that names a sector, found by asking the header rather than by copying its table into the test. 指向某扇区的编码——问头文件要，而不是把它的表抄进测试里。
 */
auto code_of_sector(std::uint8_t wanted) -> std::uint8_t {
    for (std::uint8_t code = 0U; code < 8U; ++code) {
        if (foc::core::hall_sector(code) == wanted) {
            return code;
        }
    }
    return 0xFFU;
}

/**
 * @brief One run of the tracker fed from the sensor model, with the two error statistics it exists to compare. 跟踪器由传感器模型驱动的一次运行，连同它存在的意义所在的两个误差统计量。
 */
struct TrackerRun final {
    float rms_rad{0.0F};        ///< tracker error, rad
    float rms_centre_rad{0.0F}; ///< sector-centre baseline error, same window, rad
    float max_rad{0.0F};
    float fault_fraction{0.0F};
    float stale_fraction{0.0F};
    float ok_but_wrong_fraction{0.0F}; ///< status Ok and the error over 10 deg: the undetected part
    float worst_undetected_deg{0.0F};
    int samples{0};
};

/**
 * @brief Run the tracker against a rotor turning at `omega_elec` rad/s electrical.
 *
 *     The samples are taken at the control rate and nothing is fed to the tracker except the code and the time, which
 *     is what a real drive has. `setup` is the sensor set, so the same run measures a healthy set and a broken one. 以
 *     控制频率采样，除了编码与时间之外不喂给跟踪器任何东西——这正是真实驱动手上有的一切。`setup` 是传感器组，因此同一段
 *     代码既能测健康的组，也能测坏掉的组。
 */
auto run_tracker(float omega_elec, float dt_s, float seconds, const foc::sil::HallSetup &setup,
                 float start_angle_rad = 0.35F) -> TrackerRun {
    const foc::sil::HallSensorModel model{setup};
    foc::core::HallTracker tracker;
    TrackerRun run;
    double sum_tracker = 0.0;
    double sum_centre = 0.0;
    int counted = 0;
    int faults = 0;
    int stales = 0;
    int ok_wrong = 0;
    const int ticks = static_cast<int>(seconds / dt_s);
    for (int i = 0; i < ticks; ++i) {
        const float t = static_cast<float>(i) * dt_s;
        const float truth = start_angle_rad + (omega_elec * t);
        const std::uint8_t code = model.code(truth);
        tracker.update(t, code);
        // The first two sectors are the tracker's warm-up: it has no period until the second edge, and scoring it
        // before that would score the start-up rule rather than the interpolation.
        // 前两个扇区是跟踪器的预热：第二个边沿之前它没有周期，在那之前打分等于给启动规则打分，而不是给插值打分。
        if (t < 2.0F * kSectorRad / std::fabs(omega_elec)) {
            continue;
        }
        const float estimate = tracker.angle_elec_rad(t);
        const float truth_wrapped = std::fmod(truth, kTwoPi);
        const float error = wrap_pi(estimate - truth_wrapped);
        ++counted;
        sum_tracker += static_cast<double>(error) * static_cast<double>(error);
        run.max_rad = std::max(run.max_rad, std::fabs(error));
        const std::uint8_t sector = tracker.sector();
        if (sector != foc::core::kHallNoSector) {
            const float centre_error = wrap_pi(foc::core::hall_sector_centre_rad(sector) - truth_wrapped);
            sum_centre += static_cast<double>(centre_error) * static_cast<double>(centre_error);
        }
        const foc::ErrorCode status = tracker.status();
        if (status == foc::ErrorCode::Fault) {
            ++faults;
        } else if (status == foc::ErrorCode::State) {
            ++stales;
        } else if (std::fabs(error) > (10.0F * kDegRad)) {
            ++ok_wrong;
            run.worst_undetected_deg = std::max(run.worst_undetected_deg, deg(std::fabs(error)));
        }
    }
    run.samples = counted;
    if (counted > 0) {
        run.rms_rad = static_cast<float>(std::sqrt(sum_tracker / static_cast<double>(counted)));
        run.rms_centre_rad = static_cast<float>(std::sqrt(sum_centre / static_cast<double>(counted)));
        const float n = static_cast<float>(counted);
        run.fault_fraction = static_cast<float>(faults) / n;
        run.stale_fraction = static_cast<float>(stales) / n;
        run.ok_but_wrong_fraction = static_cast<float>(ok_wrong) / n;
    }
    return run;
}

/**
 * @brief A machine with the rotor frozen, so a held commutation state is measured as a held commutation state.
 *
 *     J of 1e6 is not a modelling choice, it is the instrument: with the nominal 1e-4 the torque produced during the 20
 *     ms settling time would rotate the rotor through the sector being measured (500 rad/s^2 over 20 ms is 0.1 rad
 *     mechanical, 40 degrees electrical, more than half the sector). Plant memory records the same trap from the HFI
 *     work. J 取 1e6 不是建模选择，而是仪器：用标称的 1e-4，20 ms 建立时间内力矩会把转子转过正在测量的那个扇区（500
 *     rad/s² × 20 ms 是 0.1 rad 机械角，即 40° 电角，超过半个扇区）。plant 的记忆里记着 HFI 那轮踩过同一个坑。
 */
auto frozen_plant() -> foc::sil::PlantParams {
    foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    params.j = 1.0e6F;
    params.b = 0.0F;
    return params;
}

/**
 * @brief Steady q current with a six-step state held while the rotor sits at `theta_e`.
 *
 *     The duties are turned into phase volts the way an inverter would, Clarke'd, and Parked at the rotor -- so the
 *     number that comes out is the torque the machine actually makes in that commutation state at that rotor position.
 *     把占空比按逆变器的方式变成相电压，Clarke，再按转子角 Park——于是出来的数是机器在那个换相状态、那个转子位置下**真的
 *     ** 产生的力矩。
 */
auto held_state_current(const foc::sil::PlantParams &params, std::uint8_t code, float theta_e_rad,
                        foc::core::HallDirection direction, float amplitude, float v_bus) -> foc::core::Dq<float> {
    foc::sil::PmsmPlant plant(params);
    plant.reset(foc::sil::PlantState{0.0F, 0.0F, 0.0F, theta_e_rad / static_cast<float>(params.pole_pairs)});
    const foc::core::Uvw<float> duty = foc::core::six_step_duty(code, amplitude, direction);
    const foc::core::Uvw<float> volts{duty.u - 0.5F, duty.v - 0.5F, duty.w - 0.5F};
    const foc::core::Ab<float> ab = foc::core::clarke(volts);
    const foc::core::Dq<float> dq = foc::core::park(foc::core::Ab<float>{ab.alpha * v_bus, ab.beta * v_bus},
                                                    theta_e_rad);
    const float dt = 1.0e-5F; // 10 us: 12 steps per RL time constant (0.8 ms)
    for (int i = 0; i < 2000; ++i) { // 20 ms, 25 time constants
        plant.set_voltage_dq(dq);
        plant.advance(dt);
    }
    const foc::sil::PlantState state = plant.state();
    return foc::core::Dq<float>{state.id_a, state.iq_a};
}

} // namespace

/**
 * @brief The six-step table is a rotation: `90 + k*60` degrees for sector `k`, exactly 60 degrees apart, one phase up, one down, one at mid-rail -- and reverse is the same table 180 degrees away.
 *
 *     This is arithmetic and it is checked as arithmetic, without a plant, because a swapped table entry would still
 *     produce torque and would still rotate the machine; what it would not do is keep the vectors 60 degrees apart. 六
 *     步表是一个旋转：扇区 `k` 在 `90 + k*60` 度，彼此恰好 60°，一相上、一相下、一相在中点——反向是同一张表转 180°。这是
 *     算术，所以按算术检查，不用 plant：写反一项的表照样出扭矩、照样转机器；它做不到的是让矢量保持 60° 间隔。
 */
TEST(HallSixStep, TheTableIsARotationAtNinetyDegreesPlusSixtyPerSector) {
    constexpr float kAmplitude = 1.0F;
    constexpr float kExpected[6] = {90.0F, 150.0F, 210.0F, 270.0F, 330.0F, 390.0F}; // 90 + 60k, unwrapped
    float previous_deg = 0.0F;
    for (std::uint8_t sector = 0U; sector < foc::core::kHallSectors; ++sector) {
        const std::uint8_t code = code_of_sector(sector);
        ASSERT_NE(code, 0xFFU);
        const foc::core::Uvw<float> d =
            foc::core::six_step_duty(code, kAmplitude, foc::core::HallDirection::Forward);

        // Exactly one phase at the top, one at the bottom, one at mid-rail: two phases driven, and never two vectors
        // at once (which would be a table with an entry that is not a six-step state at all).
        // 恰好一相在上、一相在下、一相在中点：两相驱动，且不会同时有两个矢量（那种表项根本不是六步状态）。
        int high = 0;
        int low = 0;
        int mid = 0;
        for (const float value : {d.u, d.v, d.w}) {
            if (std::fabs(value - 1.0F) < 1.0e-6F) {
                ++high;
            } else if (std::fabs(value) < 1.0e-6F) {
                ++low;
            } else if (std::fabs(value - 0.5F) < 1.0e-6F) {
                ++mid;
            }
        }
        EXPECT_EQ(high, 1) << "sector " << static_cast<int>(sector);
        EXPECT_EQ(low, 1) << "sector " << static_cast<int>(sector);
        EXPECT_EQ(mid, 1) << "sector " << static_cast<int>(sector);

        const foc::core::Ab<float> ab = foc::core::clarke(foc::core::Uvw<float>{d.u - 0.5F, d.v - 0.5F, d.w - 0.5F});
        // Compared as a *rotation*: the expected angle is unwrapped (it passes 360 at sector 4), and the difference is
        // wrapped to +-180 before it is judged. The first version wrapped the expected value itself and then subtracted
        // the previous wrapped value, which reported a 300-degree step at sector 2 and looked like a table fault.
        // 按**旋转**比较：期望角不折回（它在扇区 4 越过 360），差值先折到 ±180 再判。第一版把期望值本身折回了，
        // 又去减上一个已折回的值，于是在扇区 2 读出 300 度的步进，看起来像表的错。
        const float measured_deg = deg(std::atan2(ab.beta, ab.alpha));
        const float gap_deg = deg(wrap_pi(measured_deg * kDegRad - kExpected[sector] * kDegRad));
        EXPECT_NEAR(gap_deg, 0.0F, 0.01F) << "sector " << static_cast<int>(sector) << " reads " << measured_deg;

        if (sector > 0U) {
            const float step_deg = deg(wrap_pi((measured_deg - previous_deg) * kDegRad));
            EXPECT_NEAR(step_deg, 60.0F, 0.01F) << "sector " << static_cast<int>(sector) << " moved by " << step_deg
                                                << " deg, not 60";
        }
        previous_deg = measured_deg;

        // Reverse is the same table three places further round: exactly 180 degrees, for every sector.
        const foc::core::Uvw<float> r =
            foc::core::six_step_duty(code, kAmplitude, foc::core::HallDirection::Reverse);
        const foc::core::Ab<float> ab_r =
            foc::core::clarke(foc::core::Uvw<float>{r.u - 0.5F, r.v - 0.5F, r.w - 0.5F});
        const float reverse_gap = std::fabs(deg(wrap_pi(std::atan2(ab_r.beta, ab_r.alpha) - std::atan2(ab.beta, ab.alpha))));
        EXPECT_NEAR(reverse_gap, 180.0F, 0.01F) << "sector " << static_cast<int>(sector);
    }

    // An illegal code commands nothing, rather than a guess.
    for (const std::uint8_t bad : {0U, 7U}) {
        const foc::core::Uvw<float> zero = foc::core::six_step_duty(bad, kAmplitude, foc::core::HallDirection::Forward);
        EXPECT_NEAR(zero.u, 0.5F, 1.0e-6F);
        EXPECT_NEAR(zero.v, 0.5F, 1.0e-6F);
        EXPECT_NEAR(zero.w, 0.5F, 1.0e-6F);
    }
}

/**
 * @brief Held at one commutation state, the torque is `cos(error)` of the rotor's position in the sector: maximum at the centre, **0.866 at both edges**, never negative -- and that is the 14.0 % ripple six-step cannot avoid.
 *
 *     The number is the argument for interpolating an angle at all. `cos` over +-30 degrees has a mean of 0.9549 (the
 *     integral of cos over the sector divided by its width), so the peak-to-peak ripple is (1 - 0.866) / 0.9549 =
 *     14.0 % of the mean torque -- analytic here, measured below, and the same number from both routes. 保持在一个换相
 *     状态上，力矩是转子在扇区内位置的 `cos(误差)`：中心最大，**两个边沿 0.866**，永不为负——这就是六步消不掉的 14.0 %
 *     脉动。这个数是"为什么值得插值角度"的全部论据。`cos` 在 ±30° 上均值为 0.9549（扇区内 cos 的积分除以宽度），于是峰
 *     峰脉动是 (1 − 0.866) / 0.9549 = 14.0 % 的均值力矩——这里是解析，下面实测，两条路给出同一个数。
 */
TEST(HallSixStep, HeldAtOneStateTheTorqueIsTheCosineOfThePositionAndNeverGoesNegative) {
    constexpr float kAmplitude = 0.05F;
    constexpr float kVBus = 24.0F;
    const foc::sil::PlantParams params = frozen_plant();

    for (std::uint8_t sector = 0U; sector < foc::core::kHallSectors; ++sector) {
        const std::uint8_t code = code_of_sector(sector);
        const float centre = foc::core::hall_sector_centre_rad(sector);

        // The reference is the torque at the sector centre, measured the same way, so the ratio is free of every model
        // constant (kt, bus voltage, duty scale): what is left is the geometry.
        // 参考值是同一方法在扇区中心测到的力矩，因此比值里没有任何模型常数（kt、母线电压、占空比标度），剩下的只有几何。
        const float reference =
            params.kt * held_state_current(params, code, centre, foc::core::HallDirection::Forward, kAmplitude, kVBus)
                            .q;
        ASSERT_GT(reference, 1.0e-3F) << "sector " << static_cast<int>(sector) << " produces no torque at all";

        float min_ratio = 1.0e9F;
        float max_ratio = 0.0F;
        float worst_cos = 0.0F;
        for (int step = -5; step <= 5; ++step) {
            const float offset = static_cast<float>(step) * 6.0F * kDegRad; // -30 .. +30 deg in 6 deg steps
            const foc::core::Dq<float> iq =
                held_state_current(params, code, centre + offset, foc::core::HallDirection::Forward, kAmplitude, kVBus);
            const float measured = (params.kt * iq.q) / reference;
            const float predicted = std::cos(offset);
            EXPECT_GT(measured, 0.0F) << "sector " << static_cast<int>(sector) << " offset " << deg(offset)
                                      << " deg: six-step never reverses the torque inside a sector";
            worst_cos = std::max(worst_cos, std::fabs(measured - predicted));
            min_ratio = std::min(min_ratio, measured);
            max_ratio = std::max(max_ratio, measured);
        }

        // The measured ratio is cos(offset) to three digits -- the machine agrees with the geometry, not with a
        // constant someone tuned.
        // 实测比值与 cos(偏移) 相符到三位——机器同意的是几何，不是谁调出来的常数。
        EXPECT_LT(worst_cos, 0.01F) << "sector " << static_cast<int>(sector);
        EXPECT_NEAR(min_ratio, std::cos(30.0F * kDegRad), 0.01F) << "sector " << static_cast<int>(sector);
        EXPECT_NEAR(max_ratio, 1.0F, 0.01F) << "sector " << static_cast<int>(sector);

        // Reverse: the *same* rotor at the same sensor code, the opposite vector -- the torque changes sign and keeps
        // its magnitude. Using a different code here would have measured a different rotor position.
        // 反向：**同一个**转子、同一个传感器编码、相反的矢量——力矩变号且保持大小。这里换一个编码就等于在量另一个转子位置。
        const float backwards =
            params.kt * held_state_current(params, code, centre, foc::core::HallDirection::Reverse, kAmplitude, kVBus)
                            .q;
        EXPECT_LT(backwards, -1.0e-3F) << "sector " << static_cast<int>(sector) << " reverse must push, not idle";
        EXPECT_NEAR(backwards / reference, -1.0F, 0.01F) << "sector " << static_cast<int>(sector);
    }

    // The arithmetic of the ripple, from the same two numbers: 14.0 % of the mean, not of the peak.
    const float mean_ratio = 3.0F * std::sin(30.0F * kDegRad) / (kPi * 0.5F); // integral of cos over +-30 deg, / width
    const float ripple = (1.0F - std::cos(30.0F * kDegRad)) / mean_ratio;
    EXPECT_NEAR(mean_ratio, 0.9549F, 1.0e-3F);
    EXPECT_NEAR(ripple, 0.140F, 2.0e-3F);
}

/**
 * @brief Interpolating between edges beats the sector centre by more than an order of magnitude, measured in the same window on the same run.
 *
 *     Both numbers come out of one pass, which is the only way the comparison means anything: the sector centre is what
 *     the code alone can buy (17.3 degrees RMS by the arithmetic of a uniform distribution over +-30, and that is what
 *     the baseline reads), and the interpolation is what the edge timing adds. 边沿插值比扇区中心好一个数量级以上，且是
 *     在同一趟运行的同一个窗口里量的。两个数出自同一遍扫描——这是比较有意义的唯一方式：扇区中心是只看编码能买到的东西（
 *     ±30° 均匀分布的算术给出 17.3° RMS，基线实测正是这个数），插值是边沿时刻额外买到的。
 */
TEST(HallTracker, InterpolationBeatsTheSectorCentreByAnOrderOfMagnitude) {
    for (const float omega : {120.0F, 400.0F}) {
        const TrackerRun run = run_tracker(omega, 6.25e-5F, 0.08F, foc::sil::HallSetup{});
        SCOPED_TRACE(omega);
        ASSERT_GT(run.samples, 100);
        EXPECT_NEAR(deg(run.rms_centre_rad), 17.32F, 1.0F) << "the baseline is the quantisation, and it reads right";
        // Measured at these two speeds: 0.019 and 0.053. The bound is 0.15 because the factor is *not* constant -- it
        // is 0.133 at 900 rad/s, where one sampling period is 3.2 degrees and the interpolation has little left to win.
        // 这两个转速下实测 0.019 与 0.053。界取 0.15 是因为这个倍数**不是**常数——900 rad/s 下是 0.133，那里一个采样周期
        // 就是 3.2°，插值已经没什么可赢的。
        EXPECT_LT(run.rms_rad / run.rms_centre_rad, 0.15F)
            << "tracker " << deg(run.rms_rad) << " deg RMS against baseline " << deg(run.rms_centre_rad);
        EXPECT_EQ(run.fault_fraction, 0.0F);
        EXPECT_EQ(run.stale_fraction, 0.0F);
        EXPECT_EQ(run.ok_but_wrong_fraction, 0.0F) << "a healthy set must not be wrong while it says Ok";
    }
}

/**
 * @brief What is left of the error is the sampling period times the speed, not the sector: three speeds, and the residual scales with the speed.
 *
 *     This is the sentence the header claims, and it is the one that decides whether Hall-FOC is usable at a given
 *     control rate: at 900 rad/s electrical with a 62.5 us period the angle is good to about 3 degrees, and doubling
 *     the control rate halves that -- whereas the 30-degree quantisation of the code alone never moves. 剩下的误差是采
 *     样周期乘转速，不是扇区：三个转速，残差随转速缩放。这就是头文件声称的那句话，也是决定霍尔 FOC 在某个控制频率下能不
 *     能用的那句话：62.5 µs 周期、900 rad/s 电角下角度好到约 3°，控制频率翻倍它减半——而只看编码的 30° 量化永远不动。
 */
TEST(HallTracker, TheResidualErrorIsTheSamplingPeriodTimesTheSpeedAndNotTheSector) {
    const float dt = 6.25e-5F;
    const TrackerRun slow = run_tracker(100.0F, dt, 0.12F, foc::sil::HallSetup{});
    const TrackerRun mid = run_tracker(300.0F, dt, 0.04F, foc::sil::HallSetup{});
    const TrackerRun fast = run_tracker(900.0F, dt, 0.014F, foc::sil::HallSetup{});

    EXPECT_LT(mid.rms_rad / slow.rms_rad, 4.0F);
    EXPECT_GT(mid.rms_rad / slow.rms_rad, 2.4F);
    EXPECT_LT(fast.rms_rad / mid.rms_rad, 4.0F);
    EXPECT_GT(fast.rms_rad / mid.rms_rad, 2.4F);
    // The bands are wide on purpose: the residual is not a clean power of the speed, because it depends on where the
    // sample grid falls inside each sector (measured 2.57x and 3.50x for two equal 3x steps in speed). What is claimed
    // is growth with the speed and the bound below, not an exponent.
    // 带子故意宽：残差不是转速的干净幂次，它取决于采样栅格落在每个扇区内的位置（两个相等的 3 倍转速步实测 2.57 与
    // 3.50）。这里声称的是"随转速增长"和下面那个界，不是一个指数。

    // And it is bounded by one sampling period of rotation, at every speed: the edge can only be late by the interval
    // between two samples.
    // 并且在每个转速下都被"一个采样周期转过的角度"界住：边沿最多迟到两次采样之间的间隔。
    struct SpeedAndError final {
        float omega;
        float rms;
    };
    for (const SpeedAndError &pair : {SpeedAndError{100.0F, slow.rms_rad}, SpeedAndError{300.0F, mid.rms_rad},
                                      SpeedAndError{900.0F, fast.rms_rad}}) {
        const float bound = pair.omega * dt;
        EXPECT_LE(pair.rms, bound) << "at " << pair.omega << " rad/s: " << deg(pair.rms) << " deg against "
                                   << deg(bound);
    }
}

/**
 * @brief Reversing does not flip the angle: the rotor enters the new sector across the *upper* boundary, and getting that wrong is a 60-degree error, not a small one.
 *
 *     A forward-only interpolator that always starts from the lower boundary reads perfectly going one way and is a
 *     whole sector out going the other -- and "a whole sector out" is easy to mistake for a sensor problem. 反向不翻转
 *     角度：转子是跨**上**边界进入新扇区的，搞错那不是小误差，是 60°。一个永远从下边界起步的"只会正向"的插值器往一个方
 *     向走读数完美，往另一个方向走整整差一个扇区——而"差一个扇区"很容易被误当成传感器问题。
 */
TEST(HallTracker, ReversingEntersThroughTheOtherBoundaryAndDoesNotCostSixtyDegrees) {
    const float dt = 6.25e-5F;
    const TrackerRun forward = run_tracker(300.0F, dt, 0.06F, foc::sil::HallSetup{});
    const TrackerRun backward = run_tracker(-300.0F, dt, 0.06F, foc::sil::HallSetup{});
    EXPECT_LT(deg(forward.rms_rad), 2.0F);
    EXPECT_LT(deg(backward.rms_rad), 2.0F) << "reverse error " << deg(backward.rms_rad) << " deg";
    EXPECT_LT(backward.rms_rad / forward.rms_rad, 1.5F);
    EXPECT_GT(backward.rms_rad / forward.rms_rad, 0.67F);
}

/**
 * @brief A stuck sensor: the illegal window covers one sixth of the revolution, the interval check covers the rest, and the honest number is how long the tracker stays *Ok and wrong* in between.
 *
 *     3.1a measured that a dead sensor is invisible for five sixths of a turn. This case takes that seriously and
 *     measures what the two channels actually cover: the illegal-code channel only ever sees the one sector where the
 *     other two sensors agree, and the interval channel needs 1.5 periods before it can speak -- so there is a window
 *     at the start of each doubled sector where the tracker reports `Ok` with an error growing through 10 to 30
 *     degrees. Publishing that number is the point; a fault detector quoted as "catches a dead Hall" without it is a
 *     claim about a machine that does not exist. 卡死一路传感器：非法窗口覆盖六分之一圈，间隔检查覆盖其余的，而诚实的数
 *     字是跟踪器在中间"报 Ok 却错着"有多久。 3.1a 量到坏一路在五分之六圈里不可见。本用例当真地量两条通道**实际**覆盖了
 *     什么：非法编码通道只看得到另两路一致的那一个扇区，间隔通道需要 1.5 个周期才能开口——所以每个被拉长的扇区开头都有一
 *     小段，跟踪器报 `Ok` 而误差从 10° 长到 30°。把这个数公布出来是重点；一个只说"能抓坏霍尔"而不给这个数的故障检测器，
 *     是关于一台不存在的机器的声称。
 */
TEST(HallTracker, AStuckSensorIsCoveredByTwoChannelsAndNeitherAloneIsEnough) {
    foc::sil::HallSetup setup{};
    setup.stuck_sensor = 0U;
    setup.stuck_high = false;
    const TrackerRun run = run_tracker(300.0F, 6.25e-5F, 0.12F, setup);

    ASSERT_GT(run.samples, 100);
    // The wiring sees exactly one sixth.
    EXPECT_NEAR(run.fault_fraction, 1.0F / 6.0F, 0.03F) << "the illegal window is one sector in six";
    // The interval channel sees the doubled sectors, and between the two most of the revolution is *declared* rather
    // than silently wrong. Measured: 0.155 + 0.539 with the sensor stuck low, 0.185 + 0.482 stuck high.
    // 间隔通道看得到被拉长的扇区，两者合起来把大部分圈数**申报**出来，而不是沉默地错着。实测：低位卡死 0.155 + 0.539，
    // 高位卡死 0.185 + 0.482。
    EXPECT_GT(run.stale_fraction, 0.4F);
    EXPECT_GT(run.fault_fraction + run.stale_fraction, 0.6F);
    // What is left over is the part that is `Ok` and wrong: measured 8.4 % (stuck low) and 13.7 % (stuck high), with a
    // worst case of 59 and 120 degrees. That is the boundary -- from the code stream alone roughly a third of a
    // revolution is undeclared -- and it is asserted here so it cannot quietly become "the fault is caught".
    // 剩下的是"报 Ok 却错着"的那部分：实测 8.4%（低位卡死）与 13.7%（高位卡死），最坏 59° 与 120°。这就是边界——只看编码流
    // 大约三分之一的圈数没有被申报——在这里断言它，免得它悄悄变成"故障被抓到了"。
    EXPECT_LT(run.ok_but_wrong_fraction, 0.18F) << "undetected " << (100.0F * run.ok_but_wrong_fraction) << " %";
    EXPECT_GT(run.ok_but_wrong_fraction, 0.0F) << "if this ever reads zero, the run stopped exercising the fault";
    EXPECT_LT(run.worst_undetected_deg, 130.0F) << "worst undetected " << run.worst_undetected_deg << " deg";

    // The same run on a healthy set: neither channel fires. A detector that fires on a good machine is a nuisance
    // generator, so the negative half of this criterion is asserted, not assumed.
    // 同一段在健康组上跑：两条通道都不响。在好机器上就报警的检测器是噪声发生器，所以这条判据的反面是断言出来的，不是
    // 假设的。
    const TrackerRun healthy = run_tracker(300.0F, 6.25e-5F, 0.12F, foc::sil::HallSetup{});
    EXPECT_EQ(healthy.fault_fraction, 0.0F);
    EXPECT_EQ(healthy.stale_fraction, 0.0F);
    EXPECT_EQ(healthy.ok_but_wrong_fraction, 0.0F);
}

/**
 * @brief An illegal code is reported, the angle stops moving, and the tracker re-locks once the sensors speak again.
 *
 *     The re-lock matters as much as the fault: a tracker that latches forever turns one bad sample into a stopped
 *     axis, and one that carries on silently turns it into a wrong one. 重新锁定和报故障一样要紧：永远锁死的跟踪器把一
 *     次坏采样变成停住的轴，而当作没发生的跟踪器把它变成错的轴。
 */
TEST(HallTracker, AnIllegalCodeIsReportedAndTheTrackerRelocksWhenTheSensorsRecover) {
    const foc::sil::HallSensorModel model{};
    foc::core::HallTracker tracker;
    const float dt = 6.25e-5F;
    const float omega = 300.0F;
    bool saw_fault = false;
    bool relocked = false;
    std::uint8_t faulted_sector = foc::core::kHallNoSector;
    for (int i = 0; i < 2400; ++i) {
        const float t = static_cast<float>(i) * dt;
        const float truth = 0.3F + (omega * t);
        const bool broken = (i >= 600) && (i < 900); // 300 samples of a dead wiring loom
        const std::uint8_t code = broken ? 0x07U : model.code(truth);
        const foc::ErrorCode status = tracker.update(t, code);
        if (broken) {
            if (!saw_fault) {
                faulted_sector = tracker.sector();
            }
            EXPECT_EQ(status, foc::ErrorCode::Fault);
            // With no edge to work from the estimate free-runs and then stops at the sector boundary: it is *clamped*,
            // not frozen. The first version of this case asserted "frozen" and was wrong about its own subject -- the
            // angle function is a function of time, so it keeps moving until the clamp catches it, and that is the
            // behaviour worth having (a short glitch costs no accuracy; a long one costs at most half a sector).
            // 没有边沿可用时估计值会自由推进、然后在扇区边界停住：它是**被钳住**的，不是冻结的。本用例第一版断言"冻结"，
            // 那是对它自己的被测对象判断错了——角度是时间的函数，会一直走到钳位为止，而这正是值得要的行为（短毛刺不损失
            // 精度，长故障最多损失半个扇区）。
            const float from_centre =
                std::fabs(wrap_pi(tracker.angle_elec_rad(t) - foc::core::hall_sector_centre_rad(faulted_sector)));
            EXPECT_LE(from_centre, (30.0F * kDegRad) + 1.0e-4F) << "left the sector the sensors still name";
            EXPECT_EQ(tracker.sector(), faulted_sector) << "an illegal code must not move the sector";
            saw_fault = true;
        }
    }
    EXPECT_TRUE(saw_fault);
    EXPECT_EQ(tracker.illegal_reads(), 300U);
    // Two sectors after the wiring comes back, the estimate is inside the band again -- re-locked, not merely
    // "not faulted".
    // 线路恢复两个扇区之后，估计值回到带内——是重新锁定，不只是"不再报故障"。
    const float truth = 0.3F + (omega * (static_cast<float>(2400 - 1) * dt));
    const float error = wrap_pi(tracker.angle_elec_rad(static_cast<float>(2400 - 1) * dt) - std::fmod(truth, kTwoPi));
    relocked = std::fabs(error) < (10.0F * kDegRad);
    EXPECT_TRUE(relocked) << "error after recovery " << deg(error) << " deg";
    EXPECT_EQ(tracker.status(), foc::ErrorCode::Ok);
}

/**
 * @brief A deceleration that halves the speed inside one sector also looks like a missed edge -- and at 20000 rad/s^2 the estimate while flagged is 42 degrees out, so `State` is doing real work, not being conservative.
 *
 *     The threshold is a band on the interval, so any rotor that changes speed by a third within a sector leaves it.
 *     That is not a defect to tune away: it is why the header refuses to call `State` a sensor fault, and why anything
 *     closed around this estimate must stop integrating when the status says so. The measured 42 degrees is the number
 *     that turns "use the status" from advice into arithmetic -- at a third of that deceleration the flag fires and the
 *     angle is still good, which would have made the flag look like noise. 在一个扇区内把转速减半的减速，看起来同样像丢
 *     了边沿——而在 20000 rad/s² 下，被标记时的估计值本身就差 42°，所以 `State` 是在做实事，不是在保守。阈值是间隔上的一
 *     个带子，因此任何在扇区内改变三分之一转速的转子都会离开它。这不是该调掉的缺陷：这正是头文件拒绝把 `State` 叫成传感
 *     器故障的原因，也正是任何闭合在这个估计上的环路必须在状态说话时停止积分的原因。实测的 42° 就是把"看状态"从建议变成
 *     算术的那个数——如果只有那个减速度的三分之一，旗标仍会竖起而角度依然好用，那会让旗标看起来像噪声。
 */
TEST(HallTracker, AHardDecelerationAlsoLooksLikeAMissedEdgeWhichIsWhyStateIsNotAFault) {
    const float dt = 6.25e-5F;
    foc::core::HallTracker tracker;
    const foc::sil::HallSensorModel model{};
    // Constant deceleration from 600 rad/s at 20000 rad/s^2: it loses a third of its speed in 10 ms, and a sector at
    // 600 rad/s takes 1.7 ms -- so the very next interval is 1.7x the one before it.
    // 从 600 rad/s 以 20000 rad/s² 匀减速：10 ms 内掉三分之一转速，而 600 rad/s 下一个扇区只要 1.7 ms——于是紧接的
    // 那个间隔就是前一个的 1.7 倍。
    float omega = 600.0F;
    float theta = 0.25F;
    float worst_error_deg = 0.0F;
    bool stale_seen = false;
    for (int i = 0; i < 1200; ++i) {
        const float t = static_cast<float>(i) * dt;
        tracker.update(t, model.code(theta));
        if (tracker.status() == foc::ErrorCode::State && i > 100) {
            stale_seen = true;
            const float error = wrap_pi(tracker.angle_elec_rad(t) - std::fmod(theta, kTwoPi));
            worst_error_deg = std::max(worst_error_deg, deg(std::fabs(error)));
        }
        theta += omega * dt;
        omega -= 20000.0F * dt;
        if (omega <= 0.0F) {
            break;
        }
    }
    EXPECT_TRUE(stale_seen) << "the interval check must fire on this deceleration, or the band is wrong";
    // The flag is not decoration: while it is up the estimate really is bad, by up to a sector once the clamp is the
    // only thing holding it.
    // 旗标不是装饰：它竖起时估计值确实很糟，一旦只剩钳位在支撑，误差可以大到一个扇区。
    EXPECT_GT(worst_error_deg, 20.0F) << "if the flag fires while the angle is still good, the flag means nothing";
    EXPECT_LT(worst_error_deg, 60.0F) << "and it is still clamped inside a sector, so it cannot run away";
}

/**
 * @brief [en] Before the first edge there is no angle, and `0` is the only answer that says so.
 *
 * @brief [zh] 第一个边沿之前没有角度，而 `0` 是唯一说出这件事的答案。
 *
 *     `angle_elec_rad()` returns `0` while nothing has been seen, and that zero is deliberately the same value a rotor
 *     sitting at zero would produce -- the contract says the caller must read `status()` / `sector()` to tell the two
 *     apart, and the honest half of the contract is that the accessor never invents a plausible-looking angle. The
 *     other half is what one edge buys: one edge is a *sector*, not a speed, so the best available answer is the centre
 *     of that sector, and it must not be `0` (which is inside the sector only by coincidence, for sector 5).
 *
 *     The pair matters because this is exactly what a start-up routine reads first: a tracker that answered `0` for "no
 *     information" and for "the rotor is at 0" is fine, and one that answered `0` after an edge is 30 degrees out
 *     before the machine has even moved. That is not hypothetical: the flash calibration of this repository's own
 *     `hall_sector_centre_rad` starts at `90 + k*60`, so sector 2's centre is 210 degrees.
 *
 *     `angle_elec_rad()` 在什么都没看见时返回 `0`，而这个零是**故意**与"转子停在零位"同一个值——契约说调用方必须读
 *     `status()` / `sector()` 来区分两者，而契约诚实的那一半是这个访问器永远不编一个看起来合理的角度。另一半是"一个边沿
 *     "买到了什么：一个边沿是一个**扇区**，不是一个转速，所以能给出的最好答案是那个扇区的中心，而它不能是 `0` （只有扇
 *     区 5 的 `0` 才碰巧落在扇区里）。
 *
 *     这一对要紧，因为起动流程读到的第一个数就是这个：把"没有信息"和"转子在 0"都答成 `0` 的跟踪器是对的，而在边沿之后仍
 *     答 `0` 的那个，机器还没动就已经偏了 30°。这不是假设：本仓库自己的 `hall_sector_centre_rad` 从 `90 + k*60` 开始，
 *     所以扇区 2 的中心是 210°。
 */
TEST(HallTracker, BeforeTheFirstEdgeThereIsNoAngleAndAfterOneThereIsASector) {
    foc::core::HallTracker tracker;
    EXPECT_EQ(tracker.sector(), foc::core::kHallNoSector);
    EXPECT_EQ(tracker.status(), foc::ErrorCode::State);
    EXPECT_FLOAT_EQ(0.0F, tracker.speed_elec_rad_s());
    EXPECT_FLOAT_EQ(0.0F, tracker.period_s());
    EXPECT_FLOAT_EQ(0.0F, tracker.age_s());
    // Not "zero at t = 0": zero at *every* time, because a clock does not create information.
    // 不是"t=0 时为零"：在**任何**时刻都为零，因为时钟不会创造信息。
    EXPECT_FLOAT_EQ(0.0F, tracker.angle_elec_rad(0.0F));
    EXPECT_FLOAT_EQ(0.0F, tracker.angle_elec_rad(12.5F)) << "an angle appeared out of an empty tracker";

    const std::uint8_t code = code_of_sector(2U);
    ASSERT_NE(code, 0xFFU);
    EXPECT_EQ(tracker.update(0.0F, code), foc::ErrorCode::Ok);
    EXPECT_EQ(tracker.sector(), 2U);
    // One edge is a sector, not a speed.
    // 一个边沿是一个扇区，不是一个转速。
    EXPECT_FLOAT_EQ(0.0F, tracker.speed_elec_rad_s());
    EXPECT_FLOAT_EQ(0.0F, tracker.period_s());
    // And the angle is that sector's centre, not zero: the clamp pins it there because the speed is zero.
    // 而角度是那个扇区的中心，不是零：转速为零，所以钳位把它钉在那里。
    EXPECT_FLOAT_EQ(foc::core::hall_sector_centre_rad(2U), tracker.angle_elec_rad(0.7F))
        << "one edge must buy the sector centre, not an invented zero";
}

/**
 * @brief [en] A sample that is not a finite time is refused, and refusing it leaves the estimate bit-identical to a
 *             tracker that never saw it.
 *
 * @brief [zh] 一个不是有限时间的采样被拒绝，而拒绝它之后，估计值与从未见过它的跟踪器逐位相同。
 *
 *     The guard exists because `now_s` is `float` and the caller owns the clock: one `NaN` in the time base silently
 *     turns every subsequent `elapsed` into `NaN`, every comparison against the interval band into `false`, and the
 *     published angle into `NaN` -- which the loop then feeds to Park, where it becomes a `NaN` duty cycle. A guard
 *     that reports `Param` is only worth having if it also *does not adopt* the sample, and that is the part nothing
 *     was checking: two trackers, one fed a `NaN` every 37th tick plus the real sample, must stay bit-identical to each
 *     other in angle, speed and period.
 *
 *     The comparison is the criterion, and it can fail: a version that stored `now_s = NaN` before testing it would
 *     agree on the angle (the guard still returns early) but not on `age_s()`, which is built from the stored timestamp
 *     -- hence the explicit finiteness assertion at the end.
 *
 *     这条守卫存在，是因为 `now_s` 是 `float` 而时钟由调用方掌握：时基里一个 `NaN` 会悄悄把此后每个 `elapsed` 变成
 *     `NaN`、把每个与间隔带的比较变成 `false`、把发布的角度变成 `NaN`——环路再把它送进 Park，变成 `NaN` 占空比。一条报
 *     `Param` 的守卫只有在它同时**不采纳**该采样时才值得存在，而这正是没人检查的那一半：两个跟踪器，一个每隔 37 拍喂一
 *     次 `NaN` 再喂真实采样，两者在角度、转速、周期上必须**逐位**相同。
 *
 *     比较本身就是判据，而且它能红：一个先把 `now_s = NaN` 存下来再判断的版本会在角度上一致（守卫仍然提前返回），但不会
 *     在 `age_s()` 上一致——后者正是由存下的时间戳算出来的，所以最后那条有限性断言是它的钉子。
 */
TEST(HallTracker, ANonFiniteSampleIsRefusedAndDoesNotPoisonTheClock) {
    constexpr float kDt = 6.25e-5F;
    constexpr float kOmega = 300.0F;
    const float nan_v = std::numeric_limits<float>::quiet_NaN();
    const foc::sil::HallSensorModel model{};
    foc::core::HallTracker glitched;
    foc::core::HallTracker clean;
    int refused = 0;
    for (int i = 0; i < 400; ++i) {
        const float t = static_cast<float>(i) * kDt;
        const std::uint8_t code = model.code(0.35F + (kOmega * t));
        if ((i % 37) == 0) {
            EXPECT_EQ(glitched.update(nan_v, code), foc::ErrorCode::Param) << "a NaN time was accepted, tick " << i;
            ++refused;
        }
        const foc::ErrorCode glitched_status = glitched.update(t, code);
        const foc::ErrorCode clean_status = clean.update(t, code);
        EXPECT_EQ(glitched_status, clean_status) << "tick " << i;
        EXPECT_FLOAT_EQ(clean.angle_elec_rad(t), glitched.angle_elec_rad(t)) << "tick " << i;
        EXPECT_FLOAT_EQ(clean.speed_elec_rad_s(), glitched.speed_elec_rad_s()) << "tick " << i;
        EXPECT_FLOAT_EQ(clean.period_s(), glitched.period_s()) << "tick " << i;
    }
    ASSERT_GT(refused, 10) << "the run stopped exercising the guard";
    EXPECT_TRUE(std::isfinite(glitched.age_s())) << "the refused sample's time was adopted anyway";
    EXPECT_TRUE(std::isfinite(glitched.angle_elec_rad(400.0F * kDt)));
    // The NaN arrived after real edges too, so the guard was tested on a tracker that had something to lose.
    // NaN 也在真实边沿之后到过，所以这条守卫是在一个"有东西可丢"的跟踪器上被测的。
    EXPECT_GT(glitched.period_s(), 0.0F);
}

/**
 * @brief [en] The clamp holds on the lower boundary too, which only a reversing rotor visits.
 *
 * @brief [zh] 钳位在下边界上同样成立，而只有反向的转子会走到那里。
 *
 *     `angle_elec_rad()` clamps the interpolated line to the sector the sensors are still naming, and the two clamps
 *     are not the same code: entry is on the *lower* edge going forward and on the *upper* edge going backward, so the
 *     lower clamp only ever fires on a reversal. Every case until this one drove the rotor forwards, which means the
 *     lower clamp had never executed at all -- and a missing clamp on the reverse side is invisible in a forward-only
 *     run while being an unbounded angle error in the other direction (with the rotor stopped, the line keeps going at
 *     the last speed forever).
 *
 *     The falsifier is the linear prediction: it is asserted to be *below* the lower edge, so the equality below is a
 *     clamp that had work to do and not a value that happened to land there.
 *
 *     `angle_elec_rad()` 把插值直线钳在传感器仍在指名的那个扇区里，而两条钳位不是同一段代码：正向从**下**边界进入、反向
 *     从**上**边界进入，所以下边界那条只在反向时才会触发。此前每个用例都让转子正向转，也就是说下边界钳位 **从未执行过
 *     **——而反向那一侧缺一条钳位在只看正向的运行里不可见，在另一个方向上却是无界的角度误差（转子停住时，直线会以最后那
 *     个转速一直走下去）。
 *
 *     伪证是线性预测：断言它落在下边界**之下**，所以下面那个等号是一次"有活可干"的钳位，而不是碰巧落在那里。
 */
TEST(HallTracker, TheClampHoldsOnTheLowerBoundaryWhichOnlyAReversalVisits) {
    foc::core::HallTracker tracker;
    const float step_s = 0.010F;
    // Three edges, not two: the interval arithmetic needs the *previous* sample to have been an edge, so the first
    // transition only establishes the direction and the period appears on the second one. Walking the codes down
    // 5 -> 4 -> 3 is one sector backwards each time, which is the only reading the codes allow.
    // 三个边沿，不是两个：间隔算术要求**上一个**采样本身就是边沿，所以第一次跳变只确立方向，周期在第二次才出现。
    // 编码 5 -> 4 -> 3 每步倒退一格，这是编码所允许的唯一读法。
    ASSERT_EQ(tracker.update(0.0F, code_of_sector(5U)), foc::ErrorCode::Ok);
    ASSERT_EQ(tracker.update(step_s, code_of_sector(4U)), foc::ErrorCode::Ok);
    const float t_last = 2.0F * step_s;
    ASSERT_EQ(tracker.update(t_last, code_of_sector(3U)), foc::ErrorCode::Ok);
    ASSERT_LT(tracker.speed_elec_rad_s(), 0.0F) << "this case needs a reversing tracker, and it did not reverse";
    ASSERT_EQ(tracker.sector(), 3U);

    const float later = t_last + 100.0F;
    const float half = kSectorRad * 0.5F;
    const float centre = foc::core::hall_sector_centre_rad(3U);
    const float lower_edge = centre - half;
    // A reversing rotor entered this sector across its *upper* boundary.
    // 反向的转子是跨它的**上**边界进入这个扇区的。
    const float entry = centre + half;
    const float linear = entry + (tracker.speed_elec_rad_s() * (later - t_last));
    ASSERT_LT(linear, lower_edge) << "the linear prediction is inside the sector, so this case proves nothing";

    EXPECT_FLOAT_EQ(lower_edge, tracker.angle_elec_rad(later))
        << "the lower clamp did not hold: the estimate walked out of the sector the sensors still name";
    // And the clamped answer is inside the sector, while the line it was claiming is not.
    // 而钳位后的答案落在扇区内，而它原本声称的那条直线不落在里面。
    const float answer = tracker.angle_elec_rad(later);
    EXPECT_GE(answer, centre - half);
    EXPECT_LE(answer, centre + half);
}

/**
 * @brief [en] The sensitivity table for the tracker's own input error: a rigid mounting offset enters one-for-one, and
 *             the *worst* case for the same offset is one misplaced sensor rather than all three shifted.
 *
 * @brief [zh] 跟踪器自身输入误差的敏感度表：刚性安装偏移一比一进入，而同样偏移下**最坏**的情形是单只装歪，不是三只一起
 *             平移。
 *
 *     Everything measured so far perturbed the tracker's *inputs* -- speed, the sensor being dead -- but nothing moved
 *     the sensors themselves, which is the error a real installation actually has (three sensors fitted by hand on a
 *     bracket, each a few degrees from where the geometry says it is). `HallSetup::mount_error_rad` exists for exactly
 *     this and had never been set to anything but zero.
 *
 *     Measured, healthy set at 300 rad/s electrical, 6.25e-5 s samples, 120 ms window (base = 0.696 deg rms, 1.211 deg
 *     max):
 *
 *     | offset | all three (rms / max) | one sensor (rms / max) |
 *     |--------|----------------------|------------------------| | 2 deg | 2.576 / 3.199 | 2.054 / 5.105 | | 4 deg |
 *     4.575 / 5.187 | 3.704 / 8.822 | | 8 deg | 8.549 / 9.213 | 6.703 / 15.426 |
 *
 *     Two coefficients come out of it. **Rigid rotation: the worst-case error grows exactly one-for-one** -- the maxima
 *     minus the base maxima are 1.988 / 3.976 / 8.002 deg for 2 / 4 / 8 deg, i.e. 0.994 / 0.994 / 1.000. That is not a
 *     coincidence: rotating the sensor set rotates the coordinates, so the whole estimate is displaced by the same
 *     angle and the interpolation residual rides along with it. **One sensor: the rms enters at about three quarters**
 *     (0.679 / 0.752 / 0.751) because only the two boundaries that sensor owns move, leaving four of the six sectors
 *     where they were.
 *
 *     The counter-intuitive half is asserted too, because it is the number that decides how a bracket has to be made:
 *     the single-sensor worst case is **larger than the rigid one** at the same offset (5.105 against 3.199 deg at 2
 *     deg), reaching 1.9-2.6 times the offset. Inside the two affected sectors the interpolation is anchored to a
 *     boundary that is a whole offset wrong, so the error doubles before the next true boundary corrects it. A table
 *     that only listed the rigid case would understate the worst case by more than a factor of two.
 *
 *     The bands are wide because a maximum over a sampled window is not a smooth function of the offset (it depends on
 *     where the sample grid falls inside each sector); what is claimed is the coefficient, not a decimal.
 *     `fault_fraction` and `stale_fraction` are asserted to stay zero: a mounting offset is a calibration error, not a
 *     sensor failure, and a detector that reported one as the other would make every installation look broken.
 *
 *     到目前为此所有测量扰动的都是跟踪器的**输入**——转速、某一路卡死——但没有任何一次挪动传感器本身，而真实安装恰恰有这
 *     个误差（三个传感器手工装在一个支架上，各差几度）。`HallSetup::mount_error_rad` 就是为这个存在的，而它从来没被设成
 *     过非零值。
 *
 *     实测（健康组、300 rad/s 电角、6.25e-5 s 采样、120 ms 窗口，基线 0.696° rms / 1.211° 最大值）见上表。两个系数：**
 *     刚性旋转下最坏误差一比一增长**——最大值减基线在 2/4/8° 下是 1.988/3.976/8.002°，即 0.994/0.994/1.000，因为转动传感
 *     器组就是转动坐标；**单只装歪时 rms 约为四分之三**（0.679/0.752/0.751），因为只有它辖下两条边界动了、六个扇区里四
 *     个原地不动。
 *
 *     反直觉的那一半也断言，因为它决定支架该怎么造：同样偏移下单只装歪的**最坏**误差比刚性更大（2° 时 5.105° 对
 *     3.199°），达到偏移的 1.9–2.6 倍——受影响的两个扇区里插值锚在一条整体偏了一个偏移量的边界上，于是误差先翻倍，直到下
 *     一个真边界把它纠正过来。只列刚性一项的表会把最坏情形低估一半以上。
 *
 *     带子宽，是因为"采样窗口上的最大值"不是偏移的光滑函数（取决于采样栅格落在扇区里的位置）；这里声称的是系数，不是某
 *     一位小数。`fault_fraction` 与 `stale_fraction` 断言保持为零：安装偏移是标定误差、不是传感器失效，把一个当另一个报
 *     的检测器会让每一次安装都看起来是坏的。
 */
TEST(HallTracker, TheMountingErrorEntersOneToOneAndTheWorstCaseIsOneMisplacedSensor) {
    constexpr float kDt = 6.25e-5F;
    constexpr float kOmega = 300.0F;
    constexpr float kSeconds = 0.12F;
    const TrackerRun base = run_tracker(kOmega, kDt, kSeconds, foc::sil::HallSetup{});
    ASSERT_GT(base.max_rad, 0.0F) << "a base run with no error at all would make every coefficient below meaningless";

    for (const float offset_deg : {2.0F, 4.0F, 8.0F}) {
        const float offset = offset_deg * kDegRad;

        foc::sil::HallSetup uniform{};
        for (float &error : uniform.mount_error_rad) {
            error = offset;
        }
        foc::sil::HallSetup single{};
        single.mount_error_rad[0] = offset;

        const TrackerRun rigid = run_tracker(kOmega, kDt, kSeconds, uniform);
        const TrackerRun one = run_tracker(kOmega, kDt, kSeconds, single);

        // Row 1: a rigid rotation of the set displaces the whole estimate by the same angle -- 0.994 / 0.994 / 1.000.
        // 第 1 行：传感器组刚性旋转把整个估计平移同一个角度——实测 0.994 / 0.994 / 1.000。
        EXPECT_NEAR(1.0F, (rigid.max_rad - base.max_rad) / offset, 0.03F)
            << "at " << offset_deg << " deg the rigid case is off by "
            << ((rigid.max_rad - base.max_rad) / offset) << " x the offset";
        // Row 2: one sensor carries about three quarters of it, and its worst case is larger than the rigid one.
        // 第 2 行：单只装歪只带走大约四分之三，而它的最坏情形比刚性更大。
        const float rms_factor = (one.rms_rad - base.rms_rad) / offset;
        EXPECT_GT(rms_factor, 0.60F) << "at " << offset_deg << " deg: " << rms_factor;
        EXPECT_LT(rms_factor, 0.85F) << "at " << offset_deg << " deg: " << rms_factor;
        EXPECT_GT(one.max_rad, rigid.max_rad)
            << "at " << offset_deg << " deg the single-sensor case is not the worst case, which contradicts the table";
        const float worst_factor = one.max_rad / offset;
        EXPECT_GT(worst_factor, 1.8F) << "at " << offset_deg << " deg: " << worst_factor;
        EXPECT_LT(worst_factor, 2.7F) << "at " << offset_deg << " deg: " << worst_factor;

        // A mounting offset is a calibration error: neither channel may claim it is a fault or a stale edge.
        // 安装偏移是标定误差：两条通道都不许把它当成故障或迟到的边沿。
        EXPECT_EQ(rigid.fault_fraction, 0.0F);
        EXPECT_EQ(rigid.stale_fraction, 0.0F);
        EXPECT_EQ(one.fault_fraction, 0.0F);
        EXPECT_EQ(one.stale_fraction, 0.0F);

        std::printf("\n[hall] mount %4.1f deg: rigid rms %6.3f max %6.3f | one sensor rms %6.3f max %6.3f | base %6.3f\n",
                    static_cast<double>(offset_deg), static_cast<double>(deg(rigid.rms_rad)),
                    static_cast<double>(deg(rigid.max_rad)), static_cast<double>(deg(one.rms_rad)),
                    static_cast<double>(deg(one.max_rad)), static_cast<double>(deg(base.rms_rad)));
    }
}
