// Startup -- bringing a sensorless machine up: hold the rotor, drag the vector
// up to a speed, hand over. No plant and no hardware: the sequence itself is a
// policy, and what it decides (angle, current, when it is finished) is what this
// program prints.
//
// Build with the flags the `strict` gate uses for a C++17 translation unit:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude docs/doxygen/dox/demos/startup-sensorless.cxx -o /tmp/startup-sensorless
//
// With `--csv <path>` it writes the same run as rows instead of a table; that is
// how `docs/demos/make_figures.py` draws this slice's figure, so the figure and
// the snippet are one program.
//
// 起动 —— 无感起转：按住转子、把矢量拖到某个转速、交出控制权。没有硬件也没有被控对象：
// 这个序列本身就是一条策略，它决定了什么（角度、电流、什么时候结束）就是这个程序打印的东西。
// 带 `--csv <path>` 时把同一次运行写成行而不是表；`docs/demos/make_figures.py` 正是这样画本片
// 的图，于是图与片段是同一个程序。

#include <cmath>
#include <cstdio>
#include <cstring>

#include <foc/core/startup.hpp>
#include <foc/core/types.hpp>

namespace {

constexpr float kTwoPi = 6.28318530718F;
constexpr float kDtS = 5.0e-5F; // 20 kHz control loop
constexpr std::uint32_t kPolePairs = 7U;

/// The state as a short word. A name table, not a second state machine: every
/// entry is one enumerator of `StartupState`.
/// 状态的名字表，不是第二台状态机：每一项都是 `StartupState` 的一个枚举值。
auto state_name(foc::core::StartupState state) noexcept -> const char* {
    switch (state) {
    case foc::core::StartupState::Idle:
        return "idle";
    case foc::core::StartupState::Align:
        return "align";
    case foc::core::StartupState::Pull:
        return "pull";
    case foc::core::StartupState::Done:
        return "done";
    case foc::core::StartupState::Failed:
        return "failed";
    }
    return "?";
}

/// The recipe, named once. Everything here is a statement about the machine and
/// the load: the current that holds the rotor still, how long that is held, the
/// current while dragging, how fast the vector accelerates, and the speed at
/// which the observer is expected to take over. The routine validates the
/// combination -- see the two refusals below.
/// 配方只写一次。这里每一项都是关于机器与负载的陈述：按住转子所需的电流、按多久、拖动时的
/// 电流、矢量加速多快、以及在哪个转速上应当交给观测器。这个组合由例行自己校验——见下面
/// 两处拒绝。
auto recipe() noexcept -> foc::core::StartupConfig {
    foc::core::StartupConfig config{};
    config.align_current_a = 1.5F;
    config.align_hold_s = 0.02F;
    config.pull_current_a = 2.0F;
    config.pull_accel_rad_s2 = 40.0F;
    config.handover_speed_rad_s = 60.0F;
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

    // A recipe the routine will not run. The refusal is the point: a hand-over
    // speed of zero means "no observer will ever take over", and the routine
    // says so at validate() rather than starting and stalling.
    // 一个例行不会执行的配方。这次拒绝才是要点：交接转速为零意味着"永远不会有人接管"，
    // 例行在 validate() 处就说清楚，而不是启动了再卡住。
    foc::core::StartupConfig broken = recipe();
    broken.handover_speed_rad_s = 0.0F;
    const foc::core::StartupRoutine refused(broken);

    foc::core::StartupRoutine startup(recipe());
    const bool begun = startup.begin(kPolePairs);

    // Step until the routine says it is finished, or until the clock runs out.
    // The timeout is not a parameter of the library; it is this program refusing
    // to loop forever if the recipe and the machine disagree.
    // 一直步进到例行说结束，或者时间用完。超时不是库的参数；是这个程序在配方与机器不合时
    // 拒绝永远循环下去。
    const float limit_s = 3.0F;
    float t_s = 0.0F;
    float previous_handover_e = -1.0F;
    foc::core::AlignOutput out{};
    int rows = 0;

    FILE* csv = nullptr;
    if (csv_path != nullptr) {
        csv = std::fopen(csv_path, "w");
        if (csv == nullptr) {
            std::perror("fopen");
            return 1;
        }
        std::fprintf(csv, "t_s,state,id_a,theta_e_rad,handover_e_rad_s\n");
    } else {
        std::printf("recipe refused: validate() = %d  (0 = Ok, -1 = Param)\n", static_cast<int>(refused.validate()));
        std::printf("begin(%u pole pairs) = %s\n\n", static_cast<unsigned>(kPolePairs), begun ? "true" : "false");
        std::printf("%8s  %-7s %8s %10s %12s\n", "t (ms)", "state", "id (A)", "theta_e (rad)", "hand-over (rad/s)");
    }

    while (t_s < limit_s) {
        // The measured mechanical angle is an input, and during the open-loop
        // part there is nothing to measure it with -- that is what "open loop"
        // means. The routine does not use it to decide anything before the
        // hand-over; it is there for the policy that owns the axis afterwards.
        // 实测机械角是一个输入，而开环那一段没有东西能测它——"开环"就是这个意思。例行在交接
        // 之前不用它做任何决定；它是给之后拥有这条轴的策略用的。
        out = startup.step(kDtS, 0.0F);
        t_s += kDtS;
        const bool changed = (out.handover_speed_e_rad_s != previous_handover_e);
        previous_handover_e = out.handover_speed_e_rad_s;
        if (csv != nullptr) {
            std::fprintf(csv, "%.6f,%s,%.6f,%.6f,%.6f\n", static_cast<double>(t_s), state_name(startup.state()),
                         static_cast<double>(out.id_a), static_cast<double>(out.theta_e_rad),
                         static_cast<double>(out.handover_speed_e_rad_s));
        } else if (changed || (rows % 40) == 0) {
            std::printf("%8.2f  %-7s %8.3f %10.4f %12.2f\n", static_cast<double>(1000.0F * t_s),
                        state_name(startup.state()), static_cast<double>(out.id_a),
                        static_cast<double>(out.theta_e_rad), static_cast<double>(out.handover_speed_e_rad_s));
        }
        ++rows;
        if (out.finished) {
            break;
        }
    }

    if (csv != nullptr) {
        std::fclose(csv);
        std::printf("wrote %s (%d rows, %.3f s)\n", csv_path, rows, static_cast<double>(t_s));
        return out.finished ? 0 : 1;
    }

    // What the caller does with `theta_e_rad` when the routine finishes: the
    // observer is seeded with it, and `handover_speed_e_rad_s` is what the
    // hand-over check compares against afterwards. Printing both makes the
    // contract visible instead of implied.
    // 例行结束时调用方拿 `theta_e_rad` 做什么：观测器用它播种，而 `handover_speed_e_rad_s`
    // 是之后交接检查用来比较的量。两个都打印出来，契约就是看得见的而不是隐含的。
    std::printf("\nfinished=%s  state=%s  theta_e=%.4f rad (%.2f deg)  hand-over=%.2f rad/s (%.2f mech)\n",
                out.finished ? "true" : "false", state_name(startup.state()), static_cast<double>(out.theta_e_rad),
                static_cast<double>(out.theta_e_rad * 180.0F / kTwoPi), static_cast<double>(out.handover_speed_e_rad_s),
                static_cast<double>(out.handover_speed_e_rad_s / static_cast<float>(kPolePairs)));
    return out.finished ? 0 : 1;
}
