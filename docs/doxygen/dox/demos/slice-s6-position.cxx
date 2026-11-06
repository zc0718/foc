// Slice S6 -- positioning: admit a target, shape a velocity for it, and know
// where the axis may go. No plant: the "axis" below is a bare integrator, so what
// you are looking at is the profile's shape and the limit's refusals, not a motor.
//
// Build with the flags the `strict` gate uses for a C++17 translation unit:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude docs/doxygen/dox/demos/slice-s6-position.cxx src/core/motion_profile.cpp -o /tmp/slice-s6-position
//
// With `--csv <path>` it writes the same run as rows; that is how
// `docs/demos/make_figures.py` draws this slice's figure.
//
// 切片 S6 —— 定位：接纳一个目标、为它塑造一条速度、并知道轴能去哪。没有被控对象：下面的
// "轴"只是一个纯积分器，所以看到的是速度轮廓的形状与限位的拒绝，不是一台电机。
// 带 `--csv <path>` 时把同一次运行写成行；`docs/demos/make_figures.py` 正是这样画本片的图。

#include <cmath>
#include <cstdio>
#include <cstring>

#include <foc/core/gear_ratio.hpp>
#include <foc/core/motion_profile.hpp>
#include <foc/core/soft_limit.hpp>
#include <foc/core/types.hpp>

namespace {

constexpr float kDtS = 5.0e-5F; // 20 kHz, the loop this profile is shaped for
constexpr float kWallLowRad = -1.0F;
constexpr float kWallHighRad = 1.0F;

/// The shaping law's three numbers, named once. `shaping_gain` is velocity per
/// radian of error (1/s), which is why it has the dimension of a bandwidth: the
/// axis closes about that fast to a small error.
/// 塑造律的三个数只写一次。`shaping_gain` 是"每弧度误差对应的速度"（1/s），所以它的量纲
/// 就是带宽：轴以大约那个速度收敛到小误差。
auto shaping() noexcept -> foc::core::MotionProfile::Config {
    foc::core::MotionProfile::Config config{};
    config.shaping_gain_rad_s = 8.0F;
    config.velocity_limit_rad_s = 5.0F;
    config.accel_limit_rad_s2 = 20.0F;
    return config;
}

} // namespace

auto main(int argc, char** argv) -> int {
    const char* csv_path = nullptr;
    if (argc == 3 && std::strcmp(argv[1], "--csv") == 0) {
        csv_path = argv[2];
    } else if (argc != 1) {
        std::fprintf(stderr, "usage: %s [--csv <path>]\n", argv[0]);
        return 2;
    }

    foc::core::SoftLimit limit(foc::core::SoftLimitConfig{kWallLowRad, kWallHighRad});
    foc::core::MotionProfile profile(shaping());

    // Two refusals first, because they are what the limit exists for. An
    // out-of-range target comes back as the *previous* target with `Param`, so
    // nothing downstream ever sees a position it would drive into a wall.
    // 先看两处拒绝，因为限位存在的意义就在这里。越界目标回来的是**上一个**目标并带
    // `Param`，因此下游任何东西都不会看到一个会把它驱进墙里的位置。
    const foc::core::SoftLimitOutput inside = limit.apply(0.5F);
    const foc::core::SoftLimitOutput outside = limit.apply(1.5F);
    const foc::core::SoftLimitOutput on_wall = limit.apply(kWallHighRad);

    // The gear: one master angle in, the slave's target and its velocity
    // reference out. The velocity output is not a convenience -- a position loop
    // told only where to be lags a moving target, and this is the term that
    // cancels the lag.
    // 齿轮：一个主轴角度进，从轴目标与它的速度参考出。速度输出不是便利项——只被告知
    // "该在哪"的位置环跟随移动目标会滞后，而这一项正是抵消滞后的那个。
    // Two samples on purpose: the first one *defines* the origin, so its
    // velocity is zero by construction. Printing only that one would make a
    // working gear look broken.
    // 故意喂两个采样：第一个**定义**原点，所以它的速度按构造就是零。只打印它会让一个
    // 正常的齿轮看起来是坏的。
    foc::core::ElectronicGear gear(foc::core::GearConfig{2.0F, 0.1F});
    const foc::core::GearOutput origin = gear.apply(0.25F, kDtS);
    const foc::core::GearOutput geared = gear.apply(0.2505F, kDtS);
    (void)origin;

    FILE* csv = nullptr;
    float t_s = 0.0F;
    float theta_rad = 0.0F;
    int rows = 0;

    if (csv_path != nullptr) {
        csv = std::fopen(csv_path, "w");
        if (csv == nullptr) {
            std::perror("fopen");
            return 1;
        }
        std::fprintf(csv, "t_s,target_rad,velocity_rad_s,position_rad\n");
    } else {
        std::printf("soft limit [%.1f, %.1f] rad\n", static_cast<double>(kWallLowRad),
                    static_cast<double>(kWallHighRad));
        std::printf("  apply( 0.5) -> status %2d  target %+.3f rad\n", static_cast<int>(inside.status),
                    static_cast<double>(inside.target_rad));
        std::printf("  apply( 1.5) -> status %2d  target %+.3f rad  (refused, previous target returned)\n",
                    static_cast<int>(outside.status), static_cast<double>(outside.target_rad));
        std::printf("  apply( 1.0) -> status %2d  target %+.3f rad  at_wall=%s\n\n", static_cast<int>(on_wall.status),
                    static_cast<double>(on_wall.target_rad), on_wall.at_wall ? "true" : "false");
        std::printf("gear 2:1, offset 0.1: master 10 rad/s (0.5 mrad per period) -> slave %+.4f rad, "
                    "velocity %+.4f rad/s\n\n",
                    static_cast<double>(geared.slave_rad), static_cast<double>(geared.slave_velocity_rad_s));
        std::printf("%8s %10s %12s %12s\n", "t (ms)", "target", "v cmd (rad/s)", "position (rad)");
    }

    // One move: ask for 0.5 rad and let the profile shape the approach. The
    // integrator below is the whole "plant" -- it is a kinematic identity, not a
    // motor model, which is why the figure shows a shape and not a dynamic.
    // 一次运动：要 0.5 rad，让轮廓塑造整个接近过程。下面的积分器就是全部"被控对象"——
    // 它是运动学恒等式，不是电机模型，所以图上看到的是形状而不是动态。
    const float target_rad = inside.target_rad;
    while (t_s < 1.5F) {
        const float velocity = profile.step(target_rad, theta_rad, kDtS);
        theta_rad += velocity * kDtS;
        t_s += kDtS;
        if (csv != nullptr) {
            std::fprintf(csv, "%.6f,%.6f,%.6f,%.6f\n", static_cast<double>(t_s), static_cast<double>(target_rad),
                         static_cast<double>(velocity), static_cast<double>(theta_rad));
        } else if ((rows % 400) == 0) {
            std::printf("%8.2f %10.3f %12.4f %12.5f\n", static_cast<double>(1000.0F * t_s),
                        static_cast<double>(target_rad), static_cast<double>(velocity), static_cast<double>(theta_rad));
        }
        ++rows;
    }

    if (csv != nullptr) {
        std::fclose(csv);
        std::printf("wrote %s (%d rows)\n", csv_path, rows);
        return 0;
    }

    std::printf("\nfinal position %.5f rad after %.2f s, residual %.5f rad\n", static_cast<double>(theta_rad),
                static_cast<double>(t_s), static_cast<double>(target_rad - theta_rad));
    return (std::fabs(target_rad - theta_rad) < 1.0e-3F) ? 0 : 1;
}
