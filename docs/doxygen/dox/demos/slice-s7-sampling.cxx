// Slice S7 -- sampling and the power stage: when there is time to measure the
// current at all, and what the dead time does to the voltage the bridge delivers.
//
// Build with the flags the `strict` gate uses for a C++17 translation unit:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude docs/doxygen/dox/demos/slice-s7-sampling.cxx src/core/svpwm.cpp src/core/single_shunt.cpp -o /tmp/slice-s7-sampling
//
// With `--csv <path>` it writes both sweeps as rows; `docs/demos/make_figures.py`
// draws this slice's figure from that file.
//
// 切片 S7 —— 采样与功率级：到底有没有时间测电流，以及死区对桥臂交付电压做了什么。

#include <cmath>
#include <cstdio>
#include <cstring>

#include <foc/core/board_check.hpp>
#include <foc/core/deadtime.hpp>
#include <foc/core/single_shunt.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/transforms.hpp>
#include <foc/core/types.hpp>

namespace {

constexpr float kTwoPi = 6.28318530718F;
constexpr float kVBusV = 24.0F;
constexpr float kTpwmS = 50.0e-6F;  // 20 kHz
constexpr float kTDeadS = 1.0e-6F;  // the inverter's dead time
constexpr float kTSettleS = 0.3e-6F; // ringing + propagation after an edge
constexpr float kTSampleS = 0.2e-6F; // the ADC's aperture

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
        std::fprintf(csv, "kind,x,value,flag,limit\n");
    }

    // ------------------------------------------------------------------ shunt
    // One DC-link shunt sees the current only while a particular switch pattern
    // is in force, so "is there a measurement this period" is a question about
    // the duties, not about the ADC. The answer depends on **where** the vector
    // points as much as on how big it is, so this sweeps the modulation index at
    // three angles instead of one.
    // 单电阻只在某个开关图样成立时才看得到电流，所以"这个周期有没有测量"是关于占空比的
    // 问题，不是关于 ADC 的。答案既取决于矢量有多长，也取决于它指向**哪里**，所以这里
    // 在三个角度上扫调制比，而不是只扫一个。
    const foc::core::SingleShunt shunt(foc::core::SingleShunt::Config{kTSettleS, kTSampleS, kTpwmS});
    std::printf("single shunt: settle %.2f us + sample %.2f us of a %.1f us period\n",
                static_cast<double>(1.0e6F * kTSettleS), static_cast<double>(1.0e6F * kTSampleS),
                static_cast<double>(1.0e6F * kTpwmS));
    std::printf("%6s %8s %10s %12s  %s\n", "angle", "m", "window (us)", "needed (us)", "verdict");
    constexpr int kAngles[] = {0, 20, 30};
    for (const int angle_deg : kAngles) {
        const float angle_rad = (kTwoPi / 360.0F) * static_cast<float>(angle_deg);
        int previous_verdict = -1;
        for (int step = 0; step <= 100; ++step) {
            const float m = 0.01F * static_cast<float>(step);
            const float magnitude_v = m * kVBusV;
            const foc::core::SvpwmResult modulated = foc::core::Svpwm::modulate(
                foc::core::Ab<float>{magnitude_v * std::cos(angle_rad), magnitude_v * std::sin(angle_rad)}, kVBusV);
            const foc::core::ShuntWindow window = shunt.classify(modulated.duty);
            const int verdict = static_cast<int>(window.verdict);
            if (csv != nullptr) {
                std::fprintf(csv, "shunt%d,%.4f,%.4f,%d,%.4f\n", angle_deg, static_cast<double>(m),
                             static_cast<double>(1.0e6F * window.window_s), verdict,
                             static_cast<double>(1.0e6F * window.needed_s));
            } else if (verdict != previous_verdict || (step % 25) == 0) {
                static const char *names[] = {"measurable", "window too narrow", "no active vector"};
                std::printf("%6d %8.2f %10.3f %12.3f  %s\n", angle_deg, static_cast<double>(m),
                            static_cast<double>(1.0e6F * window.window_s),
                            static_cast<double>(1.0e6F * window.needed_s),
                            names[(verdict >= 0 && verdict <= 2) ? verdict : 0]);
                previous_verdict = verdict;
            }
        }
    }

    // --------------------------------------------------------------- dead time
    // The bridge cannot deliver what it is told: every edge costs `t_dead` of the
    // period on the phase whose current sign it contradicts. `deadtime_v` is that
    // loss expressed as a voltage, and the correction is per phase because the
    // loss is per phase.
    // 桥臂交付不了它被告知的电压：每个边沿在"电流符号与之相反"的那一相上吃掉一个
    // `t_dead`。`deadtime_v` 是这份损失折算成的电压，而修正在**相域**做，因为损失属于
    // 每一相。
    const float deadtime_v = (kTDeadS / kTpwmS) * kVBusV;
    constexpr float kMinCurrentA = 0.02F;
    constexpr float kFadeA = 0.0355F; // the width the estimator returns on 24 V

    // A modulator that only scales -- `duty = inv_clarke(v_ab)/v_bus + 0.5`, no
    // common-mode redistribution. Wrapping *this* makes the duty difference equal
    // the correction itself, measured in the phase where it is applied. With a real
    // SVPWM in between, moving one phase also moves the common mode, and the duty
    // you read back is only part of what was added -- which is exactly why the
    // correction belongs in the phase domain and has to be measured there.
    // 一个只做缩放的调制器：`duty = inv_clarke(v_ab)/v_bus + 0.5`，不做共模重分配。包在它
    // 外面，占空比之差**就是**修正本身，而且是在它被施加的那一相上量的。中间夹一个真的
    // SVPWM 时，抬一相也会抬共模，你读回来的占空比只是加进去的一部分——这正是修正在相域
    // 做、也必须在相域量的原因。
    struct PassThrough final {
        [[nodiscard]] static auto modulate(const foc::core::ModulatorInput &request) noexcept
            -> foc::core::SvpwmResult {
            const foc::core::Uvw<float> v_uvw = foc::core::inv_clarke(request.v_ab);
            const float per_volt = (request.v_bus > 0.0F) ? (1.0F / request.v_bus) : 0.0F;
            auto duty_of = [per_volt](float volt) noexcept -> float {
                const float duty = 0.5F + (volt * per_volt);
                if (!(duty > 0.0F)) {
                    return 0.0F;
                }
                if (!(duty < 1.0F)) {
                    return 1.0F;
                }
                return duty;
            };
            return foc::core::SvpwmResult{
                foc::core::Uvw<float>{duty_of(v_uvw.u), duty_of(v_uvw.v), duty_of(v_uvw.w)}, false,
                foc::ErrorCode::Ok};
        }
    };

    using PhaseDomain = foc::core::DeadTimeCompensator<PassThrough>;
    const PhaseDomain stepped(PhaseDomain::Config{deadtime_v, kMinCurrentA, 0.0F});
    const PhaseDomain ramped(PhaseDomain::Config{deadtime_v, kMinCurrentA, kFadeA});
    const PhaseDomain off(PhaseDomain::Config{0.0F, kMinCurrentA, 0.0F});

    const foc::core::Ab<float> request{6.0F, 2.0F};
    std::printf("\ndead time %.0f ns of a %.1f us period -> deadtime_v = %.3f V\n",
                static_cast<double>(1.0e9F * kTDeadS), static_cast<double>(1.0e6F * kTpwmS),
                static_cast<double>(deadtime_v));
    if (csv == nullptr) {
        std::printf("%10s %16s %16s\n", "|i_u| (A)", "step (V, a-b)", "ramp (V, a-b)");
    }
    // The two configurations only disagree inside one band -- between
    // `min_current_a` (below it neither corrects) and `i_fade_a` (above it both
    // saturate) -- so the sweep is dense there and coarse above it. That band *is*
    // the whole difference, and it is 15 mA wide on this hardware.
    // 两个配置只在一个带里不同：`min_current_a` 以下都不修，`i_fade_a` 以上都饱和。所以
    // 在那个带里加密、上面放宽。那个带**就是**全部差别，本机上是 15 mA 宽。
    for (int step = 0; step <= 200; ++step) {
        const float i_magnitude = (step <= 60) ? 0.001F * static_cast<float>(step) : 0.06F + 0.002F * static_cast<float>(step - 60);
        // The current on the phase being measured goes from 0 upwards; the other two
        // carry what is left, so the set stays balanced and the correction on each
        // phase is the one that phase's own sign asks for.
        // 被测那一相的电流从 0 往上；另两相带走剩下的，保持平衡，每相的修正就是它自己符号
        // 所要求的那个。
        const foc::core::Uvw<float> currents{i_magnitude, -0.5F * i_magnitude, -0.5F * i_magnitude};
        const foc::core::ModulatorInput input{request, kVBusV, currents};
        const foc::core::SvpwmResult base = off.modulate(input);
        const foc::core::SvpwmResult with_step = stepped.modulate(input);
        const foc::core::SvpwmResult with_ramp = ramped.modulate(input);
        const float step_v = std::fabs(with_step.duty.u - base.duty.u) * kVBusV;
        const float ramp_v = std::fabs(with_ramp.duty.u - base.duty.u) * kVBusV;
        if (csv != nullptr) {
            std::fprintf(csv, "dead,%.4f,%.6f,0,%.6f\n", static_cast<double>(i_magnitude),
                         static_cast<double>(step_v), static_cast<double>(deadtime_v));
            std::fprintf(csv, "ramp,%.4f,%.6f,0,%.6f\n", static_cast<double>(i_magnitude),
                         static_cast<double>(ramp_v), static_cast<double>(deadtime_v));
        } else if ((step % 20) == 0 || (step > 18 && step < 40 && (step % 2) == 0)) {
            std::printf("%10.3f %14.4f %14.4f\n", static_cast<double>(i_magnitude), static_cast<double>(step_v),
                        static_cast<double>(ramp_v));
        }
    }

    if (csv != nullptr) {
        std::fclose(csv);
        std::printf("wrote %s\n", csv_path);
        return 0;
    }

    // ---------------------------------------------------------- board check
    // A bench session produces numbers, not verdicts -- and "two items failed" is
    // not actionable. The measurand that points at *this* slice is the one the ADC
    // sees: if the sampled ripple is much larger than the switching period implies,
    // the sample is landing off the centre of the zero vector.
    // 一次台架测量给出的是数字，不是结论——而"两项失败了"没法处理。指向**本切片**的那个
    // 被测量就是 ADC 看到的东西：如果采到的纹波远大于开关周期应有的值，采样点就落在零矢量
    // 中心之外。
    foc::core::BoardMeasurements measured{};
    measured.sum_current_a = 0.004F;
    measured.offset_a[0] = 0.001F;
    measured.offset_a[1] = -0.002F;
    measured.offset_a[2] = 0.001F;
    measured.ripple_expected_a = 0.10F;
    measured.ripple_measured_a = 0.22F; // the one this slice is about
    measured.deadtime_v = deadtime_v;
    measured.deadtime_cmd_v = deadtime_v;
    measured.encoder_offset_rad = 0.003F;
    measured.rs_ohm = 1.02F;
    measured.rs_nameplate_ohm = 1.00F;
    measured.psi_f_wb = 0.0101F;
    measured.psi_f_nameplate_wb = 0.0100F;
    measured.observer_error_rad = 0.02F;

    // Two sessions from the same board. The first leaves one channel's sign
    // inverted, which is the failure the report names first *and* the one that
    // hides the rest: an inverted channel makes the ripple comparison meaningless.
    // Fix it and the next session names a different cause -- which is what a
    // diagnosis is for.
    // 同一块板子的两次测量。第一次留了一个通道符号反了，那是报告最先指出的一项，**也是**
    // 遮蔽其余项的那一项：符号反了之后纹波比较就没意义了。修好它，第二次会话就会指出
    // 另一个成因——诊断的用处就在这里。
    for (int session = 0; session < 2; ++session) {
        measured.response_sign[0] = 1.0F;
        measured.response_sign[1] = (session == 0) ? 1.0F : -1.0F;
        measured.response_sign[2] = (session == 0) ? 1.0F : -1.0F;
        const foc::core::BoardReport report = foc::core::check_board(measured, foc::core::BoardLimits{});
        std::printf("\nboard check, session %d: %d item(s) failed of %d\n", session + 1, report.failures, report.count);
        for (int index = 0; index < report.count; ++index) {
            const foc::core::BoardReading &reading = report.items[index];
            if (reading.pass) {
                continue;
            }
            std::printf("  %-18s %8.4f vs %8.4f  FAIL\n", foc::core::board_item_name(reading.item),
                        static_cast<double>(reading.value), static_cast<double>(reading.limit));
        }
        std::printf("  -> one thing to fix: %s\n", foc::core::board_cause_name(report.cause));
    }

    // The same correction read through the real modulator: one phase moves, the
    // common mode moves with it, and the duty difference is a third reading of the
    // same 0.48 V. Three frames, one number.
    // 同一个修正透过真调制器去读：一相在动，共模跟着动，占空比之差是同一个 0.48 V 的
    // 第三种读数。三个参照系，一个数。
    using ThroughSvpwm = foc::core::DeadTimeCompensator<foc::core::Svpwm>;
    const ThroughSvpwm svpwm_step(ThroughSvpwm::Config{deadtime_v, kMinCurrentA, 0.0F});
    const ThroughSvpwm svpwm_off(ThroughSvpwm::Config{0.0F, kMinCurrentA, 0.0F});
    const foc::core::ModulatorInput at_40ma{request, kVBusV, {0.04F, -0.02F, -0.02F}};
    const float alpha_beta_v =
        std::fabs(stepped.modulate(at_40ma).duty.u - off.modulate(at_40ma).duty.u) * kVBusV;
    const float duty_v =
        std::fabs(svpwm_step.modulate(at_40ma).duty.u - svpwm_off.modulate(at_40ma).duty.u) * kVBusV;
    std::printf("\nat |i_u| = 40 mA, same correction, three places to read it (V):\n");
    std::printf("  demanded on each phase     %.3f\n", static_cast<double>(deadtime_v));
    std::printf("  visible in alpha-beta      %.3f\n", static_cast<double>(alpha_beta_v));
    std::printf("  visible in U's duty (SVPWM) %.3f\n", static_cast<double>(duty_v));
    std::printf("when all three phases correct, the alpha-beta figure is 4/3 of the per-phase loss, "
                "because all three contribute along the same axis: %.3f V\n",
                static_cast<double>((4.0F / 3.0F) * deadtime_v));

    // What the two shapes do differently, in words that match the figure.
    // 两种形状的差别，用与图一致的说法。
    std::printf("step: 0 below %.0f mA, then full deadtime_v everywhere\n", static_cast<double>(1.0e3F * kMinCurrentA));
    std::printf("ramp: 0 below %.0f mA, linear to %.1f mA, then full\n", static_cast<double>(1.0e3F * kMinCurrentA),
                static_cast<double>(1.0e3F * kFadeA));
    std::printf("the step jumps twice, not once: U leaves its dead band at %.0f mA, V and W at %.0f mA, "
                "because they carry half the current\n",
                static_cast<double>(1.0e3F * kMinCurrentA), static_cast<double>(2.0e3F * kMinCurrentA));
    return 0;
}
