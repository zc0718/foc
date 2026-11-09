// Modulation -- high speed and modulation: turn a voltage vector into duties, see
// where the linear range ends, and see what a real gate driver does to a pulse
// that is too short to transmit.
//
// Build with the flags the `strict` gate uses for a C++17 translation unit:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude docs/doxygen/dox/demos/modulation-limits.cxx src/core/svpwm.cpp src/core/min_pulse.cpp -o /tmp/modulation-limits
//
// With `--csv <path>` it writes the sweep as rows; `docs/demos/make_figures.py`
// draws this slice's figure from that file.
//
// 调制 —— 高速与调制：把一个电压矢量变成占空比、看清线性区的边界，以及真实驱动对
// "短到送不出去"的脉冲做了什么。
// 带 `--csv <path>` 时把扫描写成行；`docs/demos/make_figures.py` 用它画本片的图。

#include <cmath>
#include <cstdio>
#include <cstring>

#include <foc/core/min_pulse.hpp>
#include <foc/core/six_step.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/types.hpp>

namespace {

constexpr float kVBusV = 24.0F;
constexpr float kSwitchingPeriodS = 50.0e-6F; // 20 kHz
constexpr float kMinPulseS = 500.0e-9F;       // what the driver can actually send
constexpr float kTwoPi = 6.28318530718F;

auto span(const foc::core::Uvw<float>& duty) noexcept -> float {
    const float high = std::fmax(duty.u, std::fmax(duty.v, duty.w));
    const float low = std::fmin(duty.u, std::fmin(duty.v, duty.w));
    return high - low;
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

    FILE* csv = nullptr;
    if (csv_path != nullptr) {
        csv = std::fopen(csv_path, "w");
        if (csv == nullptr) {
            std::perror("fopen");
            return 1;
        }
        std::fprintf(csv, "angle_deg,v_pu,duty_span,saturated\n");
    } else {
        std::printf("v_bus = %.1f V, switching period %.1f us\n\n", static_cast<double>(kVBusV),
                    static_cast<double>(1.0e6F * kSwitchingPeriodS));
        std::printf("%6s %8s %10s %10s %12s %s\n", "angle", "|v| (V)", "|v|/v_bus", "duty span", "saturated",
                    "duty u / v / w");
    }

    // Sweep the commanded voltage at two angles. The modulator's limit is **not
    // a circle**: at a sector face the reachable amplitude is the inscribed
    // circle, at a sector centre it is larger. So the same sweep at 0 and at 30
    // electrical degrees clamps at two different amplitudes, and a reader who
    // assumes one radius for the whole rotation will be wrong at half of it.
    // 在两个角度上扫指令电压。调制器的边界**不是一个圆**：扇区边（面）上能达到的幅值是内切圆，
    // 扇区中心更大。所以同样一条扫描在 0 度与 30 电角度上会在两个不同幅值处钳位；把整个旋转
    // 当成一个半径的读者会在一半的位置上算错。
    const float angles[2] = {0.0F, 30.0F};
    for (const float angle_deg : angles) {
        const float angle_rad = angle_deg * kTwoPi / 360.0F;
        int previous_saturated = -1;
        for (int step = 0; step <= 90; ++step) {
            const float v_pu = 0.01F * static_cast<float>(step);
            const float v_mag = v_pu * kVBusV;
            const foc::core::Ab<float> request{v_mag * std::cos(angle_rad), v_mag * std::sin(angle_rad)};
            const foc::core::SvpwmResult result = foc::core::Svpwm::modulate(request, kVBusV);
            const int saturated = result.saturated ? 1 : 0;
            if (csv != nullptr) {
                std::fprintf(csv, "%.1f,%.4f,%.6f,%d\n", static_cast<double>(angle_deg), static_cast<double>(v_pu),
                             static_cast<double>(span(result.duty)), saturated);
            } else if (saturated != previous_saturated || (step % 20) == 0) {
                std::printf("%6.0f %8.2f %10.3f %10.4f %12s %7.4f %7.4f %7.4f\n", static_cast<double>(angle_deg),
                            static_cast<double>(v_mag), static_cast<double>(v_pu),
                            static_cast<double>(span(result.duty)), result.saturated ? "yes" : "no",
                            static_cast<double>(result.duty.u), static_cast<double>(result.duty.v),
                            static_cast<double>(result.duty.w));
                previous_saturated = saturated;
            }
        }
    }

    if (csv != nullptr) {
        std::fclose(csv);
        std::printf("wrote %s\n", csv_path);
        return 0;
    }

    // What the driver does to a narrow pulse. `lost_vs` is the volt-seconds the
    // bridge will not deliver this period: positive means the phase gets less
    // than it was asked for. A loop that does not know this number integrates an
    // error its own feedback cannot see, because the bridge is inside the loop
    // and the phase voltage is not measured.
    // 驱动对窄脉冲做了什么。`lost_vs` 是本周期桥臂**不会**交付的伏秒：正数表示那一相拿到的
    // 比要求少。不知道这个数的环路会积分一项它自己的反馈看不见的误差，因为桥臂在环内、相电压
    // 又没有被测。
    const foc::core::MinPulse min_pulse(foc::core::MinPulseConfig{kMinPulseS, kSwitchingPeriodS});
    std::printf("\nmin pulse %.0f ns of a %.1f us period; asking for one phase at 0.1 %% duty\n",
                static_cast<double>(1.0e9F * kMinPulseS), static_cast<double>(1.0e6F * kSwitchingPeriodS));
    const foc::core::MinPulseOutput shaped = min_pulse.apply(foc::core::Uvw<float>{0.001F, 0.5F, 0.999F}, kVBusV);
    std::printf("  status=%d narrowed=%s\n", static_cast<int>(shaped.status), shaped.narrowed ? "true" : "false");
    std::printf("  requested duty %.4f %.4f %.4f\n", 0.001, 0.5, 0.999);
    std::printf("  applied   duty %.4f %.4f %.4f\n", static_cast<double>(shaped.duty.u),
                static_cast<double>(shaped.duty.v), static_cast<double>(shaped.duty.w));
    std::printf("  lost          %.6f %.6f %.6f  V*s\n", static_cast<double>(shaped.lost_vs.u),
                static_cast<double>(shaped.lost_vs.v), static_cast<double>(shaped.lost_vs.w));

    // Six-step: the same modulator question with the answer taken from a table.
    // One hall code in, three duties out, and a rate limit on how fast the table
    // may be walked -- a single false edge must not jump the vector by 120
    // electrical degrees.
    // 六步：同一个调制问题，答案来自一张表。一个霍尔码进、三个占空比出，并对"表能被走多快"
    // 有速率限制——一个假的边沿不许把矢量一次跳过 120 电角度。
    foc::core::SixStepConfig six_config{};
    six_config.amplitude = 1.0F;
    six_config.direction = foc::core::HallDirection::Forward;
    six_config.max_step_sectors = 1U;
    foc::core::SixStepCommutation six_step(six_config);
    std::printf("\nsix-step, code 0b101 (sector 5):\n");
    for (int index = 0; index < 2; ++index) {
        const foc::core::SixStepOutput commuted = six_step.step(0b101U);
        std::printf("  tick %d: sector %u duty %.4f %.4f %.4f\n", index, static_cast<unsigned>(commuted.sector),
                    static_cast<double>(commuted.duty.u), static_cast<double>(commuted.duty.v),
                    static_cast<double>(commuted.duty.w));
    }
    std::printf("  rejected=%u illegal=%u\n", static_cast<unsigned>(six_step.rejected()),
                static_cast<unsigned>(six_step.illegal()));
    return 0;
}
