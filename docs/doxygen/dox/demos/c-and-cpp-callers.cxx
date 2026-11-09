// Callers -- consuming the library from C: five function pointers in, a handle
// out, and every call after that is a plain `foc_*` function.
//
// Build with the flags the `strict` gate uses, plus the C translation unit the
// ABI lives in (the repository links `src/**/*.cpp` and `src/**/*.c`; a product
// links the aggregate target `foc::foc`):
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude docs/doxygen/dox/demos/c-and-cpp-callers.cxx src/api/foc_api.c src/api/bridge.cpp src/core/*.cpp src/motor/*.cpp src/comm/*.cpp src/driver/*.cpp src/core/observer/*.cpp -o /tmp/c-and-cpp-callers
//
// With `--csv <path>` it writes the run as rows; `docs/demos/make_figures.py`
// draws this example's figure from that file.
//
// 调用方 —— 从 C 侧使用本库：五个函数指针进去、一个句柄出来，之后每一次调用都是普通的
// `foc_*` 函数。带 `--csv <path>` 时把这次运行写成行，供绘图脚本使用。

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include <array>

#include <foc/api/foc_api.h>
#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/driver/callback_hal.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

namespace {

constexpr float kDtS = 5.0e-5F; // the loop the host ticks at
constexpr int kTicks = 1200;    // 60 ms: two current steps, the same for both paths
constexpr float kStepAtS = 0.03F;
constexpr float kTwoPi = 6.28318530718F;
constexpr std::uint8_t kPolePairs = 7U;

/**
 * @brief [en] The integrator's board, as plain C callbacks see it.
 *
 *     This is a **stand-in**, not a motor model: Ohm's law across the winding
 *     plus the winding's own `L/R`, so the current loop has something to regulate
 *     and the figure has something to show. What the example is actually
 *     demonstrating is the ABI -- five function pointers in, a handle out, and the
 *     same `foc_*` calls a pure C consumer would make.
 *
 * @brief [zh] 集成方的板子，按普通 C 回调看到的样子。
 *
 *     这是一个**替身**，不是电机模型：绕组上的欧姆定律加上绕组自身的 `L/R`，好让电流环有
 *     东西可调、图上有东西可看。例子真正演示的是 ABI——五个函数指针进去、一个句柄出来，以及
 *     一个纯 C 消费者会做的同一批 `foc_*` 调用。
 */
struct Board final {
    float duty[3]{0.5F, 0.5F, 0.5F};
    float current_a[3]{0.0F, 0.0F, 0.0F};
    float v_bus_v{24.0F};
    float angle_mech_rad{0.0F};
    float speed_mech_rad_s{0.0F};
    float rs{0.5F};       // the winding's resistance, from the same nameplate
    float tau_s{0.8e-3F}; // L/R of that winding: 0.4 mH over 0.5 ohm
    int32_t faults{0};
};

void write_duty(void* user, const float duty_uvw[3]) {
    auto* board = static_cast<Board*>(user);
    std::memcpy(board->duty, duty_uvw, sizeof(board->duty));
}

void read_current(void* user, float uvw_a[3]) {
    auto* board = static_cast<Board*>(user);
    // Ohm's law across the winding, with the star point floating: the common mode
    // of the duties drives nothing, so it is removed first. Then one first-order
    // lag with the winding's own `L/R`. Both are identities, not a controller --
    // this stand-in exists so the current loop has something to regulate, and the
    // page says so.
    // 绕组上的欧姆定律，星点浮空：占空比的共模不驱动任何东西，所以先去掉。再给它一个
    // 绕组自身的 `L/R` 一阶滞后。两者都是恒等式而不是控制器——这个替身存在的意义是让
    // 电流环有东西可调，页面会把这一点写明。
    const float mean = (board->duty[0] + board->duty[1] + board->duty[2]) / 3.0F;
    for (int phase = 0; phase < 3; ++phase) {
        const float target = ((board->duty[phase] - mean) * board->v_bus_v) / board->rs;
        board->current_a[phase] += (target - board->current_a[phase]) * (kDtS / board->tau_s);
        uvw_a[phase] = board->current_a[phase];
    }
}

auto read_angle(void* user) -> float { return static_cast<Board*>(user)->angle_mech_rad; }

auto read_v_bus(void* user) -> float { return static_cast<Board*>(user)->v_bus_v; }

auto read_fault(void* user) -> int32_t { return static_cast<Board*>(user)->faults; }

/**
 * @brief [en] The same experiment through the C++ API: the class, not the handle.
 *
 *     It shares the callbacks, the plant and the tick count with the C path above -- only the
 *     entry surface differs, which is the whole point of putting the two in one file.
 * @brief [zh] 同一个实验，走 C++ 接口：用类，而不是句柄。
 *
 *     回调、台架与拍数和上面的 C 路径共用，区别只在入口表面——把两条路放进一个文件，正是
 *     为了说明这一点。
 */
auto run_cpp_path(Board& board, std::array<float, kTicks>& span_out) -> bool {
    // The same four operations, wired to the same functions the C path hands over. `CallbackHal`
    // is the C++-side struct; the C struct is its spelling across the ABI.
    foc::driver::HalCallbacks callbacks{};
    callbacks.user = &board;
    callbacks.pwm_write_duty = &write_duty;
    callbacks.read_current = &read_current;
    callbacks.read_angle_mech_rad = &read_angle;
    callbacks.read_voltage_v = &read_v_bus;
    callbacks.pwm_fault = &read_fault;
    callbacks.read_drive_fault = &read_fault;
    // The numbers the C constructor resolves to with a null setup, written out: same nameplate,
    // same limits, same two gain helpers. Anything else would compare two different controllers.
    const foc::motor::MotorConfig config{0.5F, 4.0e-4F, 8.0e-4F, 0.01F, 1.0e-4F, 1.0e-4F, kPolePairs,
                                         5.0F, 20000.0F, 500.0F, 0.0F, 0.0F};
    using CppMotor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi,
                                       foc::driver::CallbackHal>;
    foc::driver::CallbackHal hal{callbacks};
    CppMotor motor(config,
                   CppMotor::Policies{foc::core::BackEmfObserver({0.5F, 4.0e-4F}), foc::core::Svpwm{},
                                      foc::core::Pi{foc::current_pi_config(config, 2000.0F, 24.0F)},
                                      foc::core::Pi{foc::current_pi_config(config, 2000.0F, 24.0F)},
                                      foc::core::Pi{foc::speed_pi_config(config, 200.0F)}},
                   hal);
    if (motor.init() != foc::ErrorCode::Ok || motor.start() != foc::ErrorCode::Ok) {
        return false;
    }
    for (int i = 0; i < kTicks; ++i) {
        const float iq_command_a = (static_cast<float>(i) * kDtS < kStepAtS) ? 1.5F : -1.5F;
        motor.set_torque(iq_command_a);
        if (motor.tick(kDtS) != foc::ErrorCode::Ok) {
            return false;
        }
        const float high = std::fmax(board.duty[0], std::fmax(board.duty[1], board.duty[2]));
        const float low = std::fmin(board.duty[0], std::fmin(board.duty[1], board.duty[2]));
        span_out[static_cast<std::size_t>(i)] = high - low;
    }
    return true;
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

    Board board{};
    foc_hal_callbacks_t callbacks{};
    callbacks.struct_size = sizeof(foc_hal_callbacks_t);
    callbacks.user = &board;
    callbacks.pwm_write_duty = &write_duty;
    callbacks.read_current = &read_current;
    callbacks.read_angle_mech_rad = &read_angle;
    callbacks.read_voltage_v = &read_v_bus;
    callbacks.pwm_fault = &read_fault;
    // The appended field, and the one whose absence has its own answer: without
    // it `create_host()` returns FOC_ERR_NOT_IMPL rather than FOC_ERR_PARAM,
    // because "we never asked" and "nothing is wrong" would otherwise look the
    // same. `struct_size` is what makes the field visible at all.
    // 追加的那个字段，也是"缺失时单独回答"的那个：没有它，`create_host()` 返回
    // FOC_ERR_NOT_IMPL 而不是 FOC_ERR_PARAM，否则"我们没问"与"什么都没坏"就长成一个
    // 样子。`struct_size` 才让这个字段可见。
    callbacks.read_drive_fault = &read_fault;

    // Nameplate as plain data. The order is the struct's, which is why the C
    // header says it in the docstring rather than leaving it to the reader.
    // 铭牌就是普通数据。顺序就是结构体的顺序——这也是 C 头在文档里把它写出来的原因。
    const foc_motor_params_t params{
        0.5F,    // rs
        4.0e-4F, // ld
        8.0e-4F, // lq -- salient, so the observer and MTPA have something to do
        0.01F,   // psi_f
        1.0e-4F, // j
        1.0e-4F, // b
        kPolePairs,
    };

    foc_motor_t motor = nullptr;
    if (foc_motor_create_host(&params, nullptr, &callbacks, &motor) != FOC_OK) {
        std::fprintf(stderr, "foc_motor_create_host refused\n");
        return 1;
    }
    if (foc_motor_enable(motor) != FOC_OK) {
        std::fprintf(stderr, "foc_motor_enable refused\n");
        static_cast<void>(foc_motor_destroy(motor));
        return 1;
    }

    FILE* csv = nullptr;
    if (csv_path != nullptr) {
        csv = std::fopen(csv_path, "w");
        if (csv == nullptr) {
            std::perror("fopen");
            static_cast<void>(foc_motor_destroy(motor));
            return 1;
        }
        std::fprintf(csv, "t_s,iq_command_a,iq_measured_a,duty_span\n");
    } else {
        std::printf("five callbacks in, handle out: create_host -> enable -> set_speed -> tick\n\n");
        std::printf("%8s %10s %10s %8s %8s\n", "t (ms)", "iq cmd (A)", "iq (A)", "id (A)", "span");
    }

    // Torque mode on purpose: this page is about what crosses the ABI, and a
    // speed loop would put a mechanical model in the way of that question. The
    // command is a current, the loop regulates it against the host's windings,
    // and what comes back out is the duty the host has to apply.
    // 故意用转矩模式：本页讲的是"什么东西穿过 ABI"，而速度环会把一个机械模型摆在那个问题
    // 前面。指令是电流，环路对着宿主的绕组把它调住，出来的是宿主必须施加的占空比。
    std::array<float, kTicks> span_c{};
    int rows = 0;

    for (int i = 0; i < kTicks; ++i) {
        const float t_s = static_cast<float>(i) * kDtS;
        // Two steps, so the picture has a shape: a duty trace from a constant
        // current would be a flat line, and a flat line teaches nothing about
        // what crosses the boundary.
        // 两个阶跃，图才有形状：恒定电流的占空比会是一条直线，而直线教不会你边界上过去了什么。
        const float iq_command_a = (t_s < kStepAtS) ? 1.5F : -1.5F;
        static_cast<void>(foc_set_torque(motor, iq_command_a));
        static_cast<void>(foc_tick(motor, kDtS));

        float speed_rad_s = 0.0F;
        float id_a = 0.0F;
        float iq_a = 0.0F;
        static_cast<void>(foc_get_state(motor, &speed_rad_s, &id_a, &iq_a));

        // The **span** rather than one phase: at some electrical angles a given
        // phase sits at exactly 0.5 whatever the loop is doing, and a column that
        // happens to be constant looks like a loop that is not working.
        // 记**跨度**而不是某一相：在有些电角度上，某一相无论环路在做什么都正好是 0.5，而一列
        // 恰好恒定的数看起来像环路没在工作。
        const float high = std::fmax(board.duty[0], std::fmax(board.duty[1], board.duty[2]));
        const float low = std::fmin(board.duty[0], std::fmin(board.duty[1], board.duty[2]));
        const float duty_span = high - low;
        span_c[static_cast<std::size_t>(i)] = duty_span;
        if (csv != nullptr) {
            std::fprintf(csv, "%.6f,%.4f,%.4f,%.4f\n", static_cast<double>(t_s), static_cast<double>(iq_command_a),
                         static_cast<double>(iq_a), static_cast<double>(duty_span));
        } else if ((rows % 200) == 0) {
            std::printf("%8.2f %10.3f %10.3f %8.3f %8.3f\n", static_cast<double>(1000.0F * t_s),
                        static_cast<double>(iq_command_a), static_cast<double>(iq_a), static_cast<double>(id_a),
                        static_cast<double>(duty_span));
        }
        ++rows;
    }

    foc_motor_status_t status{};
    status.struct_size = sizeof(foc_motor_status_t);
    const foc_err_t status_result = foc_motor_get_status(motor, &status);

    if (csv != nullptr) {
        std::fclose(csv);
        std::printf("wrote %s (%d rows)\n", csv_path, rows);
    } else {
        std::printf("\nfoc_motor_get_status -> %d (0 = FOC_OK)\n", static_cast<int>(status_result));
        std::printf("  mode_state=%d error=%d observer=%d locked=%d worst tick=%u us\n",
                    static_cast<int>(status.mode_state), static_cast<int>(status.error),
                    static_cast<int>(status.observer), static_cast<int>(status.observer_locked),
                    static_cast<unsigned>(status.max_period_us));
    }

    // The equivalence check: the same experiment through the C++ class, against a fresh copy of
    // the same plant. One number decides whether the two entry surfaces are one implementation.
    // 等效性检查：同一个实验改走 C++ 类，台架重新取一份拷贝。一个数就足以判断两个入口是不是
    // 同一份实现。
    Board board_cpp{};
    std::array<float, kTicks> span_cpp{};
    const bool cpp_ok = run_cpp_path(board_cpp, span_cpp);
    float worst = 0.0F;
    if (cpp_ok) {
        for (int i = 0; i < kTicks; ++i) {
            worst = std::fmax(worst, std::fabs(span_cpp[static_cast<std::size_t>(i)] -
                                               span_c[static_cast<std::size_t>(i)]));
        }
    }
    std::printf("\ncpp path: %s, worst |duty span difference| over %d ticks = %.3e\n",
                cpp_ok ? "ran" : "refused", kTicks, static_cast<double>(worst));

    // The handle owns a pool slot until it is destroyed: a program that leaks one
    // leaks the ability to create another motor.
    // 句柄在被销毁前占着一个池槽：泄漏一个，就泄漏"再创建一台电机"的能力。
    static_cast<void>(foc_motor_destroy(motor));
    return 0;
}
