// Slice S8 -- health and protection: what a drive does when something is wrong,
// and how the answers differ from the ones that look like them.
//
// Build with the flags the `strict` gate uses for a C++17 translation unit:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude docs/doxygen/dox/demos/slice-s8-protection.cxx src/core/protection.cpp src/core/watchdog.cpp src/core/brake_chopper.cpp -o /tmp/slice-s8-protection
//
// With `--csv <path>` it writes the two sweeps and the latch timeline as rows;
// `docs/demos/make_figures.py` draws this slice's figure from that file.
//
// 切片 S8 —— 健康与保护：驱动器在出问题时会做什么，以及那些看起来一样的答案之间差在哪里。

#include <cmath>
#include <cstdio>
#include <cstring>

#include <foc/core/brake_chopper.hpp>
#include <foc/core/protection.hpp>
#include <foc/core/types.hpp>
#include <foc/core/watchdog.hpp>

namespace {

// The configuration the library's own protection tests use, retyped here on
// purpose: a protection review is a review of *these* numbers, and a demo with its
// own invented limits would be reviewing nothing.
// 库自己的保护测试所用的配置，这里故意照抄一遍：保护评审评审的就是**这些**数，而一个自己发明
// 限值的演示什么都没评审。
auto protection_config() -> foc::core::ProtectionConfig {
    foc::core::ProtectionConfig config{};
    config.i_hard_a = 4.0F;
    config.i_max_a = 3.0F;
    config.i2t_a2s = 4.5F; // 3 A squared times 0.5 s: the equilibrium the rating implies
    config.i2t_cooling_s = 0.5F;
    config.v_max_v = 30.0F;
    config.v_min_v = 18.0F;
    config.t_trip_c = 100.0F;
    config.t_derate_start_c = 60.0F;
    config.t_derate_end_c = 110.0F;
    config.stall_speed_rad_s = 5.0F;
    config.stall_current_a = 1.0F;
    config.stall_time_s = 0.5F;
    config.sync_error_ratio = 0.25F;
    config.hysteresis_ratio = 0.05F;
    return config;
}

// A healthy bus at 24 V, a valid flux estimate and a comfortable temperature, so
// that the one input under test is the only thing that can trip.
// 24 V 的健康母线、有效的磁链估计、舒适的温度，于是被测的那个输入是唯一可能跳闸的东西。
auto healthy_input(float current_a, float temperature_c) -> foc::core::ProtectionInput {
    foc::core::ProtectionInput input{};
    input.i_magnitude_a = current_a;
    input.v_bus_v = 24.0F;
    input.temperature_c = temperature_c;
    input.omega_e_rad_s = 100.0F;
    input.flux_error_ratio = 0.01F;
    input.flux_valid = true;
    return input;
}

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

    constexpr float kDt = 100.0e-6F; // 10 kHz
    const foc::core::ProtectionConfig config = protection_config();

    // ------------------------------------------------------------- i2t budget
    // The budget is `i_cont^2 * tau`, so the continuous rating sits *at* the
    // limit forever and only the excess spends the margin. Below the hard trip
    // (4 A) the question is therefore not "does it trip" but "how long", and that
    // time goes to infinity as the current approaches the rating.
    // 预算是 `i_cont²·τ`，所以连续额定永远**坐在**限值上，只有超出的部分花掉余量。在硬跳闸
    // （4 A）以下，问题不是"跳不跳"而是"多久"，而这个时长在电流逼近额定时趋于无穷。
    std::printf("protection: %.0f A hard, %.1f A continuous, budget %.1f A^2*s, %.1f s cooling\n",
                static_cast<double>(config.i_hard_a), static_cast<double>(config.i_max_a),
                static_cast<double>(config.i2t_a2s), static_cast<double>(config.i2t_cooling_s));
    std::printf("%10s %14s %s\n", "i (A)", "trip after (s)", "fault");
    for (int step = 0; step <= 24; ++step) {
        const float current_a = 3.02F + (0.04F * static_cast<float>(step));
        foc::core::Protector protector(config);
        float time_s = 0.0F;
        foc::core::ProtectionOutput output{};
        while (time_s < 60.0F) {
            output = protector.step(healthy_input(current_a, 40.0F), kDt);
            time_s += kDt;
            if (output.fault != foc::core::FaultCode::None) {
                break;
            }
        }
        const bool tripped = output.fault != foc::core::FaultCode::None;
        if (csv != nullptr) {
            std::fprintf(csv, "i2t,%.3f,%.5f,%d\n", static_cast<double>(current_a),
                         static_cast<double>(tripped ? time_s : 0.0F), tripped ? 1 : 0);
        } else if ((step % 4) == 0) {
            if (tripped) {
                std::printf("%10.3f %14.4f %s\n", static_cast<double>(current_a), static_cast<double>(time_s),
                            "I2t");
            } else {
                std::printf("%10.3f %14s %s\n", static_cast<double>(current_a), "never", "-");
            }
        }
    }

    // ------------------------------------------------------------ derating
    // The continuous limit is a curve in temperature; the trip is a threshold.
    // They are not two spellings of one decision: at the trip temperature the
    // curve still allows current, so a controller that only reads the curve never
    // trips, and a controller that only reads the trip runs at full current right
    // up to it.
    // 连续限幅是温度上的一条曲线；跳闸是一个阈值。它们不是同一个决定的两种写法：在跳闸温度上
    // 曲线仍然允许电流，所以只读曲线的控制器永不跳闸，而只读跳闸的控制器会把满电流一直用到那一点。
    std::printf("\n%-10s %10s %10s\n", "T (C)", "limit (A)", "trip?");
    for (int step = 0; step <= 120; step += 10) {
        const float temperature_c = static_cast<float>(step);
        const float limit_a = foc::core::Protector(config).derated_limit(temperature_c);
        const bool trip = temperature_c > config.t_trip_c;
        if (csv != nullptr) {
            std::fprintf(csv, "derate,%.1f,%.5f,%d\n", static_cast<double>(temperature_c),
                         static_cast<double>(limit_a), trip ? 1 : 0);
        } else {
            std::printf("%-10.0f %10.4f %10s\n", static_cast<double>(temperature_c), static_cast<double>(limit_a),
                        trip ? "trip" : "-");
        }
    }

    // The trip threshold on its own row: it is a configuration number, not a
    // sample of the curve, and a figure that drew it from the curve would be
    // showing the curve's zero crossing instead of the trip.
    // 跳闸阈值单独一行：它是配置里的数，不是曲线上的采样点；从曲线里取它来画，画出来的会是
    // 曲线的零点而不是跳闸点。
    if (csv != nullptr) {
        const float at_trip_a = foc::core::Protector(config).derated_limit(config.t_trip_c);
        std::fprintf(csv, "trip,%.1f,%.5f,0\n", static_cast<double>(config.t_trip_c), static_cast<double>(at_trip_a));
    }

    // --------------------------------------------------------------- latching
    // An over-current trip is latched: when the current comes back inside the
    // limits the output stays disabled, because the reason it tripped has not been
    // looked at yet. `reset()` is the only way out, and it is the caller's
    // decision -- a drive that cleared itself would restart into the same fault.
    // 过流跳闸是锁存的：电流回到限值以内之后输出仍然被禁，因为跳闸的原因还没有人看过。
    // `reset()` 是唯一的出路，而且是调用方的决定——自己复位的驱动器会一头撞回同一个故障。
    {
        foc::core::Protector protector(config);
        float time_s = 0.0F;
        foc::core::FaultCode previous_fault = foc::core::FaultCode::None;
        std::printf("\nlatch timeline (10 ms at 4.5 A, then healthy input):\n");
        for (int step = 0; step < 300; ++step) {
            const float current_a = (time_s < 0.010F) ? 4.5F : 1.0F;
            const foc::core::ProtectionOutput output = protector.step(healthy_input(current_a, 40.0F), kDt);
            time_s += kDt;
            if (output.fault != foc::core::FaultCode::None && previous_fault == foc::core::FaultCode::None) {
                std::printf("  4.5 A at %.1f ms -> fault, output_allowed %d\n", static_cast<double>(1000.0F * time_s),
                            output.output_allowed ? 1 : 0);
            }
            if (current_a < 2.0F && (step == 110 || step == 299)) {
                std::printf("  back to 1 A at %.1f ms -> output_allowed %d (latched)\n",
                            static_cast<double>(1000.0F * time_s), output.output_allowed ? 1 : 0);
            }
            previous_fault = output.fault;
            if ((step % 20) == 0 || step == 299) {
                if (csv != nullptr) {
                    std::fprintf(csv, "latch,%.3f,%.4f,%d\n", static_cast<double>(1000.0F * time_s),
                                 static_cast<double>(output.i_limit_a), output.output_allowed ? 1 : 0);
                }
            }
        }
        foc::core::ProtectionOutput after = protector.step(healthy_input(1.0F, 40.0F), kDt);
        std::printf("  healthy current, no reset   -> output_allowed %d, limit %.2f A\n", after.output_allowed ? 1 : 0,
                    static_cast<double>(after.i_limit_a));
        protector.reset();
        after = protector.step(healthy_input(1.0F, 40.0F), kDt);
        std::printf("  after reset()               -> output_allowed %d, limit %.2f A\n", after.output_allowed ? 1 : 0,
                    static_cast<double>(after.i_limit_a));
    }

    // -------------------------------------------------------------- watchdog
    // It measures silence, not progress. A loop that is alive but stuck still gets
    // kicked if the kick sits in the wrong place; a loop that is dead cannot kick.
    // 它量的是**沉默**，不是进展。活着但卡住的循环，如果喂狗写错了地方，照样会被喂；而死的循环
    // 喂不了。
    {
        constexpr std::uint64_t kWindowNs = 5'000'000ULL; // 5 ms
        foc::core::Watchdog watchdog;
        if (watchdog.setup(foc::core::WatchdogConfig{kWindowNs}) != foc::ErrorCode::Ok) {
            std::printf("  watchdog refused the window\n");
        }
        static const char *names[] = {"unarmed", "feeding", "expired", "clock lost"};
        for (std::uint64_t now_ns = 1'000'000ULL; now_ns <= 3'000'000ULL; now_ns += 1'000'000ULL) {
            const foc::core::WatchdogReport report = watchdog.kick(now_ns);
            std::printf("  watchdog kick at %llu ms -> %s, %llu ms left\n",
                        static_cast<unsigned long long>(now_ns / 1'000'000ULL), names[static_cast<int>(report.state)],
                        static_cast<unsigned long long>(report.remaining_ns / 1'000'000ULL));
        }
        // Asking is not kicking: `poll()` says what the state is at that instant
        // without moving the deadline, which is what a supervisor does on its own
        // schedule -- the loop that must be watched cannot be the one doing the
        // watching.
        // 问不是喂：`poll()` 说出那一刻的状态而不动截止时间，这正是监督者按自己的节奏做的事——
        // 被看的那个环路不能同时是看的那个。
        const foc::core::WatchdogReport expired = watchdog.poll(9'000'000ULL);
        std::printf("  watchdog asked at 9 ms (no kick since 3) -> %s, silence %llu ms\n",
                    names[static_cast<int>(expired.state)],
                    static_cast<unsigned long long>(expired.step_ns / 1'000'000ULL));
        const foc::core::WatchdogReport late = watchdog.kick(10'000'000ULL);
        std::printf("  a kick after the trip -> %s (it does not clear itself)\n", names[static_cast<int>(late.state)]);
    }

    // ---------------------------------------------------------- brake chopper
    // The resistor is a budget, not a switch: its thermal mass is what decides how
    // long a regen event may last, and a chopper that only follows the voltage can
    // burn itself out while holding the bus perfectly.
    // 电阻是一个预算，不是一个开关：它的热容决定一次回馈能持续多久，而一个只看电压的斩波器
    // 可以在把母线守得很好的同时把自己烧掉。
    {
        foc::core::BrakeChopper chopper(
            foc::core::BrakeChopperConfig{32.0F, 30.0F, 10.0F, 0.9F, 100.0e-6F, 100.0e-6F, 0.5F, 2.0F});
        std::printf("\nbrake chopper, bus ramps to 34 V then falls back:\n");
        for (int step = 0; step < 60; ++step) {
            const float v_bus = (step < 30) ? (28.0F + (0.2F * static_cast<float>(step)))
                                            : (34.0F - (0.2F * static_cast<float>(step - 30)));
            const foc::core::BrakeChopperOutput output = chopper.step(v_bus, 1.0e-3F);
            if ((step % 10) == 0 || (step == 15 && output.on)) {
                std::printf("  v_bus %5.1f V -> on %d, duty %.2f, i2t %.3f A^2*s\n", static_cast<double>(v_bus),
                            output.on ? 1 : 0, static_cast<double>(output.duty), static_cast<double>(output.i2t_a2s));
            }
        }
    }

    if (csv != nullptr) {
        std::fclose(csv);
        std::printf("wrote %s\n", csv_path);
        return 0;
    }
    return 0;
}
