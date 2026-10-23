// Documentation asset exporter: runs the library's SIL bench and writes the traces the Sphinx demo
// pages plot.
//
// Why a C++ program instead of driving the Python module: the C API (and therefore `foc_python`)
// runs on the stub HAL, whose current sense is a fixed script — the traces would be flat lines.
// Real curves need the SIL plant, which lives on the C++ side. `docs/demos/make_figures.py`
// compiles this file against the repository sources, runs it, and plots the CSV it writes.
// 为什么用 C++ 而不是 Python 模块：C API（以及 foc_python）跑在桩 HAL 上，电流反馈是固定脚本，
// 画出来是直线；真实曲线需要 SIL plant，而它只存在于 C++ 侧。`docs/demos/make_figures.py` 会
// 编译本文件、运行它，并把输出的 CSV 画成图。
//
// Usage / 用法:
//     sil_export <trace.csv> <metrics.csv>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstddef>
#include <vector>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"



namespace {

constexpr float kTwoPi = 6.28318530718F;

struct Row final {
    float t_s;
    float speed;
    float id;
    float iq;
    float ref;
};

auto emit(FILE *trace, const char *scenario, const std::vector<Row> &rows) -> void {
    for (const Row &row : rows) {
        std::fprintf(trace, "%s,%.6f,%.6f,%.6f,%.6f,%.6f\n", scenario, static_cast<double>(row.t_s),
                     static_cast<double>(row.speed), static_cast<double>(row.id), static_cast<double>(row.iq),
                     static_cast<double>(row.ref));
    }
}

} // namespace



auto main(int argc, char **argv) -> int {
    if (argc < 3) {
        std::fprintf(stderr, "usage: %s <trace.csv> <metrics.csv>\n", argv[0]);
        return 2;
    }

    FILE *trace = std::fopen(argv[1], "w");
    FILE *metrics = std::fopen(argv[2], "w");
    if (trace == nullptr || metrics == nullptr) {
        std::perror("fopen");
        return 1;
    }
    std::fprintf(trace, "scenario,t_s,speed_rad_s,id_a,iq_a,reference\n");
    std::fprintf(metrics, "scenario,metric,value,unit\n");

    // ---------------------------------------------------------------- current step
    {
        foc::sil::Bench bench(foc::sil::standard_bench_config());
        if (bench.init_and_start() != foc::ErrorCode::Ok) {
            std::fprintf(stderr, "current_step: bench refused to start\n");
            return 1;
        }
        const float dt = bench.config().dt_ctrl_s;
        std::vector<Row> rows;
        std::vector<foc::sil::Sample> trace_c;
        std::vector<float> reference;
        auto record = [&](float ref) {
            bench.tick_once();
            const foc::sil::Sample sample = bench.sample();
            rows.push_back(Row{sample.t_s, sample.speed, sample.id_a, sample.iq_a, ref});
            trace_c.push_back(sample);
            reference.push_back(ref);
        };

        // idle, then +1 A, then -1 A: the worst case for an integrator that is not clamped
        const int settle = static_cast<int>((0.005F / dt) + 0.5F);
        const int hold = static_cast<int>((0.020F / dt) + 0.5F);
        for (int i = 0; i < settle; ++i) {
            record(0.0F);
        }
        const std::size_t step_index = trace_c.size();
        for (int i = 0; i < hold; ++i) {
            bench.motor().set_torque(1.0F);
            record(1.0F);
        }
        for (int i = 0; i < hold; ++i) {
            bench.motor().set_torque(-1.0F);
            record(-1.0F);
        }
        emit(trace, "current_step", rows);

        // metrics on the +1 A half only
        std::vector<foc::sil::Sample> positive(trace_c.begin() + static_cast<std::ptrdiff_t>(step_index),
                                               trace_c.begin() + static_cast<std::ptrdiff_t>(step_index) +
                                                   static_cast<std::ptrdiff_t>(hold));
        const std::vector<float> iq = foc::sil::series_of(positive, &foc::sil::Sample::iq_a);
        std::fprintf(metrics, "current_step,settling_time_5pct,%.4f,s\n",
                     static_cast<double>(foc::sil::settling_time_s(positive, &foc::sil::Sample::iq_a, 1.0F, 0.05F)));
        std::fprintf(metrics, "current_step,overshoot,%.2f,%%\n",
                     static_cast<double>(100.0F * foc::sil::overshoot_fraction(iq, 1.0F, 0.0F)));
        std::fprintf(metrics, "current_step,steady_state_error,%.4f,A\n",
                     static_cast<double>(1.0F - foc::sil::tail_mean(iq, 0.2F)));
    }

    // ---------------------------------------------------------------- speed step
    {
        foc::sil::Bench bench(foc::sil::standard_bench_config());
        bench.motor().set_speed(0.0F);
        if (bench.init_and_start() != foc::ErrorCode::Ok) {
            std::fprintf(stderr, "speed_step: bench refused to start\n");
            return 1;
        }
        const float dt = bench.config().dt_ctrl_s;
        std::vector<Row> rows;
        std::vector<foc::sil::Sample> samples;
        std::vector<float> reference;
        const int idle = static_cast<int>((0.005F / dt) + 0.5F);
        const int run = static_cast<int>((0.300F / dt) + 0.5F);
        for (int i = 0; i < idle; ++i) {
            bench.tick_once();
            const foc::sil::Sample s = bench.sample();
            rows.push_back(Row{s.t_s, s.speed, s.id_a, s.iq_a, 0.0F});
            samples.push_back(s);
            reference.push_back(0.0F);
        }
        const std::size_t step_index = samples.size();
        bench.motor().set_speed(100.0F);
        for (int i = 0; i < run; ++i) {
            bench.tick_once();
            const foc::sil::Sample s = bench.sample();
            rows.push_back(Row{s.t_s, s.speed, s.id_a, s.iq_a, 100.0F});
            samples.push_back(s);
            reference.push_back(100.0F);
        }
        emit(trace, "speed_step", rows);

        std::vector<foc::sil::Sample> step(samples.begin() + static_cast<std::ptrdiff_t>(step_index), samples.end());
        const std::vector<float> speed = foc::sil::series_of(step, &foc::sil::Sample::speed);
        std::fprintf(metrics, "speed_step,settling_time_2pct,%.4f,s\n",  // band = 2 % of 100 rad/s
                     static_cast<double>(foc::sil::settling_time_s(step, &foc::sil::Sample::speed, 100.0F, 2.0F)));
        std::fprintf(metrics, "speed_step,overshoot,%.2f,%%\n",
                     static_cast<double>(100.0F * foc::sil::overshoot_fraction(speed, 100.0F, 0.0F)));
        std::fprintf(metrics, "speed_step,steady_state_error,%.4f,rad/s\n",
                     static_cast<double>(100.0F - foc::sil::tail_mean(speed, 0.2F)));
    }

    // ---------------------------------------------------------------- sine tracking
    {
        foc::sil::Bench bench(foc::sil::standard_bench_config());
        bench.motor().set_speed(0.0F);
        if (bench.init_and_start() != foc::ErrorCode::Ok) {
            std::fprintf(stderr, "sine_tracking: bench refused to start\n");
            return 1;
        }
        const float dt = bench.config().dt_ctrl_s;
        std::vector<Row> rows;
        std::vector<foc::sil::Sample> samples;
        std::vector<float> reference;
        const int total = static_cast<int>((2.0F / dt) + 0.5F);
        const int skip = static_cast<int>((0.25F / dt) + 0.5F);
        for (int i = 0; i < total; ++i) {
            const float ref = 50.0F * std::sin(kTwoPi * 1.0F * bench.elapsed_s());
            bench.motor().set_speed(ref);
            bench.tick_once();
            const foc::sil::Sample s = bench.sample();
            if (i >= skip) {
                rows.push_back(Row{s.t_s, s.speed, s.id_a, s.iq_a, ref});
                samples.push_back(s);
                reference.push_back(ref);
            }
        }
        emit(trace, "sine_tracking", rows);

        float worst = 0.0F;
        for (std::size_t i = 0; i < samples.size(); ++i) {
            const float err = std::fabs(samples[i].speed - reference[i]);
            worst = (err > worst) ? err : worst;
        }
        std::fprintf(metrics, "sine_tracking,worst_error_pct_of_amplitude,%.2f,%%\n",
                     static_cast<double>(100.0F * worst / 50.0F));
        std::fprintf(metrics, "sine_tracking,peak_to_peak_speed,%.2f,rad/s\n",
                     static_cast<double>(foc::sil::tail_peak_to_peak(
                         foc::sil::series_of(samples, &foc::sil::Sample::speed), 0.5F)));
    }

    std::fclose(trace);
    std::fclose(metrics);
    return 0;
}
