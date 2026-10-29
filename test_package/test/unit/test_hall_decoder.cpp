// 3.1b: the decoder, measured against the physics 3.1a established. Two capabilities in one header -- the six-step
// table, and an angle interpolated from edge timing -- and both criteria are about how *wrong* each one is, because
// "it turns the motor" is satisfied by a table with two entries swapped.
// 3.1b：解码器，对着 3.1a 立下的物理来测。一个头文件里两件能力——六步表，和由边沿时刻插值出来的角度——两条判据都
// 关于各自**错多少**，因为"电机转了"这件事，一张有两项写反的表也能满足。

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

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
