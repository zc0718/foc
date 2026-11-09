// Low speed -- zero and low speed: high-frequency injection, and the machines it
// refuses to work on.
//
// Build with the flags the `strict` gate uses for a C++17 translation unit:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude docs/doxygen/dox/demos/low-speed-injection.cxx src/core/observer/hfi_observer.cpp src/core/transform.cpp -o /tmp/low-speed-injection
//
// With `--csv <path>` it writes both sweeps as rows; `docs/demos/make_figures.py`
// draws this slice's figure from that file.
//
// 低速 —— 零速与低速：高频注入，以及它拒绝在哪些机器上工作。

#include <cmath>
#include <cstdio>
#include <cstring>

#include <foc/core/observer/hfi_observer.hpp>
#include <foc/core/types.hpp>

namespace {

constexpr float kTwoPi = 6.28318530718F;

// A standstill salient machine, small enough to read: two axes, no back EMF, no
// saturation. `lq_ratio == 1` is a surface-mount rotor -- the case this slice is
// about.
// 一台静止的凸极电机，小到能一眼读完：两个轴、没有反电动势、没有饱和。
// `lq_ratio == 1` 就是表贴式转子——本切片要讲的就是这一种。
struct SalientPlant final {
    float rs_ohm = 1.0F;
    float ld_h = 0.010F;
    float lq_h = 0.010F;
    float theta_rad = 0.7F; // where the rotor happens to be
    float i_alpha_a = 0.0F;
    float i_beta_a = 0.0F;

    void step(const foc::core::Ab<float>& v_ab, float dt_s) noexcept {
        const float c = std::cos(theta_rad);
        const float s = std::sin(theta_rad);
        const float v_d = (v_ab.alpha * c) + (v_ab.beta * s);
        const float v_q = (v_ab.beta * c) - (v_ab.alpha * s);
        const float i_d = (i_alpha_a * c) + (i_beta_a * s);
        const float i_q = (i_beta_a * c) - (i_alpha_a * s);
        const float i_d_next = i_d + (dt_s * (v_d - (rs_ohm * i_d)) / ld_h);
        const float i_q_next = i_q + (dt_s * (v_q - (rs_ohm * i_q)) / lq_h);
        i_alpha_a = (i_d_next * c) - (i_q_next * s);
        i_beta_a = (i_d_next * s) + (i_q_next * c);
    }
};

struct RunResult final {
    bool configured = false;
    bool locked_at_end = false;
    float final_angle_error_rad = 0.0F;
    float final_hf_current_a = 0.0F;
};

} // namespace

auto main(int argc, char **argv) -> int {
    const char *csv_path = nullptr;
    if (argc == 3 && std::strcmp(argv[1], "--csv") == 0) {
        csv_path = argv[2];
    } else if (argc != 1) {
        std::fprintf(stderr, "usage: %s [--csv <path>]\n", argv[0]);
        return 2;
    }

    FILE *csv = nullptr;
    if (csv_path != nullptr) {
        csv = std::fopen(csv_path, "w");
        if (csv == nullptr) {
            std::perror("fopen");
            return 1;
        }
        std::fprintf(csv, "kind,x,value,flag\n");
    }

    constexpr float kInjectV = 12.0F;
    constexpr float kInjectHz = 2000.0F;
    constexpr float kControlHz = 20000.0F;
    constexpr float kDt = 1.0F / kControlHz;
    constexpr int kSteps = 20000; // one second
    constexpr int kLogEvery = 20; // 1 ms

    // Three runs that between them answer the slice's question. The first is a
    // surface-mount rotor, the second an interior one, and the third is an interior
    // rotor observed by a controller that cannot resolve the carrier.
    // 三次运行合起来回答本切片的问题。第一次是表贴式转子，第二次是内置式，第三次是内置式
    // 转子，但控制器分辨不了这个载波。
    struct Case final {
        const char *name;
        float lq_ratio;
        float dt_s;
        const char *colour; // for the reader, not for the program
    };
    const Case cases[] = {
        {"spm", 1.0F, kDt, "never locks"},
        {"ipm", 1.4F, kDt, "locks"},
        {"slow-control", 1.4F, 1.0F / 3000.0F, "refused"},
    };

    std::printf("HFI at %.0f V / %.0f Hz, control %.0f Hz; max carrier this rate resolves: %.0f Hz\n",
                static_cast<double>(kInjectV), static_cast<double>(kInjectHz), static_cast<double>(kControlHz),
                static_cast<double>(foc::core::HfiObserver::max_inject_hz(kControlHz)));
    std::printf("%-14s %8s %10s %10s %8s %8s %6s\n", "case", "Lq/Ld", "gain (A)", "locks", "err (rad)",
                "|dtheta|", "status");

    float surface_mount_error_rad = 0.0F;
    for (const Case &test : cases) {
        SalientPlant plant;
        plant.lq_h = plant.ld_h * test.lq_ratio;
        const float gain = foc::core::HfiObserver::predicted_gain_a(kInjectV, kInjectHz, plant.ld_h, plant.lq_h);
        foc::core::HfiObserver observer(foc::core::HfiObserver::Config{kInjectV, kInjectHz, 100.0F, 200.0F,
                                                                      20.0F, 0.9F, 0.0F, gain, 0.001F});
        RunResult result;
        result.configured = observer.status() == foc::ErrorCode::Ok;

        // The estimate is what `update()` publishes; the truth is where the rotor
        // is. Keeping both is the whole point -- the observer's own `error()` is not
        // the angle error, it is the demodulator's residual.
        // 估计是 `update()` 发布的东西；真值是转子实际所在。两个都留着才是要点——观测器自己的
        // `error()` 不是角度误差，而是解调器的残差。
        for (int step = 0; step < kSteps; ++step) {
            const foc::core::Ab<float> injection = observer.injection_ab();
            plant.step(injection, test.dt_s);
            const foc::core::ObsState state =
                observer.update(injection, foc::core::Ab<float>{plant.i_alpha_a, plant.i_beta_a}, test.dt_s);
            const float angle_error = std::fabs(std::remainder(state.theta_e - plant.theta_rad, kTwoPi));
            if ((step % kLogEvery) == 0 && csv != nullptr) {
                std::fprintf(csv, "%s,%.4f,%.6f,%d\n", test.name, 1000.0F * static_cast<float>(step) * test.dt_s,
                             static_cast<double>(angle_error), observer.locked() ? 1 : 0);
            }
            result.final_angle_error_rad = angle_error;
            result.final_hf_current_a = observer.hf_current_a();
            result.locked_at_end = observer.locked();
        }

        std::printf("%-14s %8.2f %10.5f %10d %10.5f %8.5f %6d\n", test.name, static_cast<double>(test.lq_ratio),
                    static_cast<double>(gain), result.locked_at_end ? 1 : 0,
                    static_cast<double>(observer.error()), static_cast<double>(result.final_angle_error_rad),
                    static_cast<int>(observer.status()));
        if (test.lq_ratio == 1.0F) {
            surface_mount_error_rad = result.final_angle_error_rad;
        }
    }

    // The sensitivity itself, over the whole range: it is zero at ratio one, and a
    // zero is not a small number to be tuned around -- it is the statement that
    // this method has nothing to measure.
    // 灵敏度在整段上的取值：比值等于一时它是零，而零不是一个可以调参绕过的小数——它说的是
    // 这个方法没有东西可测。
    for (int step = 0; step <= 40; ++step) {
        const float ratio = 1.0F + (0.025F * static_cast<float>(step));
        const float gain = foc::core::HfiObserver::predicted_gain_a(kInjectV, kInjectHz, 0.010F, 0.010F * ratio);
        if (csv != nullptr) {
            std::fprintf(csv, "gain,%.4f,%.6f,0\n", static_cast<double>(ratio), static_cast<double>(gain));
        } else if ((step % 5) == 0) {
            std::printf("  Lq/Ld %.3f -> sensitivity %+.5f A\n", static_cast<double>(ratio),
                        static_cast<double>(gain));
        }
    }

    if (csv != nullptr) {
        std::fclose(csv);
        std::printf("wrote %s\n", csv_path);
        return 0;
    }

    std::printf("\nthe surface-mount run ends %.3f rad away from the truth and reports locked=0, error=0:\n"
                "a zero error signal is what a perfect lock looks like *and* what no signal at all looks like\n",
                static_cast<double>(surface_mount_error_rad));
    return 0;
}
