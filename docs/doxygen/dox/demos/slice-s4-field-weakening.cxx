// Slice S4 -- above base speed: the voltage ceiling starts to bind, and the
// currents have to leave the MTPA curve to keep producing torque.
//
// Build with the flags the `strict` gate uses for a C++17 translation unit:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude docs/doxygen/dox/demos/slice-s4-field-weakening.cxx src/core/mtpv.cpp -o /tmp/slice-s4-field-weakening
//
// With `--csv <path>` it writes the sweep as rows; `docs/demos/make_figures.py`
// draws this slice's figure from that file.
//
// 切片 S4 —— 基速以上：电压天花板开始约束，电流必须离开 MTPA 曲线才能继续出力。
// 带 `--csv <path>` 时把扫描写成行，供绘图脚本使用。

#include <cmath>
#include <cstdio>
#include <cstring>

#include <foc/core/machine_params.hpp>
#include <foc/core/mtpv.hpp>
#include <foc/core/types.hpp>

namespace {

constexpr float kVBusV = 24.0F;

/// The voltage the modulator can actually deliver over a whole rotation: the
/// inscribed circle of its hexagon, not the rail. S5 draws why those differ.
/// 调制器在整个旋转上都保证得住的电压：它六边形的**内切圆**，不是母线电压。两者为何不同见 S5。
constexpr float kVMaxV = kVBusV / 1.7320508F;

} // namespace

auto main(int argc, char** argv) -> int {
    const char* csv_path = nullptr;
    if (argc == 3 && std::strcmp(argv[1], "--csv") == 0) {
        csv_path = argv[2];
    } else if (argc != 1) {
        std::fprintf(stderr, "usage: %s [--csv <path>]\n", argv[0]);
        return 2;
    }

    // A salient machine whose characteristic current is well above its current
    // ceiling: `lambda = Ld * i_max / psi_f` above 1 is the domain where field
    // weakening has somewhere to go. The four nameplate machines of this
    // repository sit below it, which is why this one is synthetic and says so.
    // 一台凸极机，其特征电流远高于电流上限：`lambda = Ld * i_max / psi_f` 大于 1 才是有弱磁
    // 余地的区域。本仓库的四台铭牌机都在这个值以下，所以这一台是合成的，并且它自己说出来。
    const foc::core::MachineParams machine{0.5F, 4.0e-3F, 8.0e-3F, 0.01F};
    const foc::core::MtpvConfig config{machine, kVMaxV, 3.0F, 7U};
    foc::core::Mtpv solver(config);
    if (solver.validate() != foc::ErrorCode::Ok) {
        std::fprintf(stderr, "the machine/ceiling combination was refused\n");
        return 1;
    }

    const foc::core::MtpvPoint mtpa = solver.mtpa_point();
    const float base_speed = solver.base_speed_rad_s();

    FILE* csv = nullptr;
    if (csv_path != nullptr) {
        csv = std::fopen(csv_path, "w");
        if (csv == nullptr) {
            std::perror("fopen");
            return 1;
        }
        std::fprintf(csv, "speed_ratio,id_a,iq_a,torque_nm,voltage_limited\n");
    } else {
        std::printf("nameplate: Ld 4 mH, Lq 8 mH, psi_f 0.01 Wb, i_max 3 A\n");
        std::printf("voltage ceiling %.3f V (the modulator's inscribed circle)\n", static_cast<double>(kVMaxV));
        std::printf("base speed %.1f rad/s elec -- from the flux at the MTPA point, not from psi_f\n\n",
                    static_cast<double>(base_speed));
        std::printf("MTPA point: id %+.4f A  iq %+.4f A  torque %.4f N.m  |i| %.4f A\n\n",
                    static_cast<double>(mtpa.id_a), static_cast<double>(mtpa.iq_a), static_cast<double>(mtpa.torque_nm),
                    static_cast<double>(mtpa.current_a));
        std::printf("%8s %10s %10s %10s %12s\n", "w/w_base", "id (A)", "iq (A)", "torque (N.m)", "voltage binds");
    }

    // Sweep the electrical speed. Below base speed the answer does not move: the
    // current ceiling is the only one in force and MTPA is the best use of it.
    // Above it, the voltage ellipse starts to bind and the d current has to grow
    // negative -- trading torque per amp for headroom.
    // 扫电角速度。基速以下答案不动：只有电流上限生效，而 MTPA 是它的最好用法。基速以上电压
    // 椭圆开始约束，d 轴电流必须更负——用"每安培转矩"去换电压余量。
    int previous_limited = -1;
    for (int step = 0; step <= 200; ++step) {
        const float ratio = 0.1F * static_cast<float>(step);
        const float omega_e = ratio * base_speed;
        const foc::core::MtpvPoint point = solver.at_speed(omega_e);
        const int limited = point.voltage_limited ? 1 : 0;
        if (csv != nullptr) {
            std::fprintf(csv, "%.4f,%.6f,%.6f,%.6f,%d\n", static_cast<double>(ratio), static_cast<double>(point.id_a),
                         static_cast<double>(point.iq_a), static_cast<double>(point.torque_nm), limited);
        } else if (limited != previous_limited || (step % 20) == 0) {
            std::printf("%8.2f %10.4f %10.4f %10.5f %12s\n", static_cast<double>(ratio),
                        static_cast<double>(point.id_a), static_cast<double>(point.iq_a),
                        static_cast<double>(point.torque_nm), point.voltage_limited ? "yes" : "no");
            previous_limited = limited;
        }
    }

    if (csv != nullptr) {
        std::fclose(csv);
        std::printf("wrote %s\n", csv_path);
        return 0;
    }

    // The number that decides whether field weakening is worth implementing at
    // all, printed rather than asserted: `lambda`. The plan's domain gate uses the
    // same ratio, so a machine that lands outside it is a machine where the whole
    // slice is optional.
    // 决定"弱磁值不值得做"的那个数，打印出来而不是断言：`lambda`。计划的 domain 门用的是同一个
    // 比值，所以落在这个范围之外的机器，整片都是可选的。
    const float lambda = (machine.ld * config.i_max_a) / machine.psi_f;
    std::printf("\nlambda = Ld * i_max / psi_f = %.3f  (above 1: field weakening has room)\n",
                static_cast<double>(lambda));
    return 0;
}
