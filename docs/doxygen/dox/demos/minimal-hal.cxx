// Porting to a board, minimally -- the five contracts the loop requires, written from scratch in this file, with no
// `driver/stub_*` involved. The point is the member set: the library detects each contract structurally, so an adapter
// is "these methods, with these signatures" and nothing else.
//
// Build it the way the `demo` gate does -- the example links the out-of-line observer, so `-Iinclude` alone fails at
// the link. A renamed source turns that gate red instead of leaving a stale instruction here:
//   g++ -std=c++17 -fno-exceptions -fno-rtti -Wall -Wextra -Werror -Iinclude
//   docs/doxygen/dox/demos/minimal-hal.cxx src/core/*.cpp src/core/observer/*.cpp src/motor/*.cpp
//   src/driver/*.cpp src/version.c -o /tmp/minimal-hal
//
// 移植到板子上的最小形态 —— 环需要的五个契约，全部在本文件里从零写起，不涉及任何 `driver/stub_*`。重点就是成员集：
// 库按结构探测每个契约，所以一个适配器就是"这些方法、这些签名"，再没有别的。

#include <cstdio>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/types.hpp>
#include <foc/motor/motor.hpp>

namespace {

constexpr float kDtS = 1.0e-4F;
constexpr float kBusV = 24.0F;
constexpr int kTicks = 200;

/**
 * @brief [en] A board: five required contracts and nothing else.
 * @brief [zh] 一块板子：五个必需契约，别的什么都没有。
 * @note [en] Three of the five optional contracts -- clock, temperature, mechanical brake -- are deliberately absent, and
 *            this file still compiles and runs: that is what "optional" means here. What it does *not* have is a
 *            declared absence of the temperature channel, so thermal derating reports "unavailable" and over-temperature
 *            never trips. An adapter that cares should read the porting page before borrowing this shape.
 * @note [zh] 五个可选契约里的三个——时钟、温度、机械抱闸——被故意省掉，而本文件照样编译、照样跑：这就是这里"可选"的含义。
 *            但它**没有**写明温度通道的缺席，于是热降额报 "unavailable"、过温永不动作。要长期上机的适配器，借这个形状之前
 *            先读移植指南那一页。
 */
struct Board final {
    /**
     * @brief [en] The switch. `configure`/`enable` return a code because a real bridge can refuse; `write_duty` does not
     *        because a duty written into a timer compare register cannot fail after the fact.
     * @brief [zh] 开关管那一侧。`configure`/`enable` 返回错误码，因为真实桥臂会拒绝；`write_duty` 不返回，因为写进比较
     *        寄存器的占空比事后不会失败。
     */
    struct Pwm final {
        float frequency_hz{0.0F};
        unsigned writes{0U};

        [[nodiscard]] auto configure(float freq_hz, float deadtime_ns) noexcept -> foc::ErrorCode {
            // A real adapter validates against the timer it has; this one only records what it was told.
            // 真实适配器会对着自己手上的定时器校验；这一个只记下被告知的数。
            (void)deadtime_ns;
            if (!(freq_hz > 0.0F)) {
                return foc::ErrorCode::Param;
            }
            frequency_hz = freq_hz;
            return foc::ErrorCode::Ok;
        }
        void write_duty(foc::core::Uvw<float> duty) noexcept {
            (void)duty;
            ++writes;
        }
        [[nodiscard]] auto enable() noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
        void disable() noexcept {}
        [[nodiscard]] auto fault() const noexcept -> bool { return false; }
    };

    /**
     * @brief [en] Phase currents. Zeros here, because this board has no shunt; a real one reads the ADC at the instant
     *        the modulator says, which is why `sample_rate_hz` is part of the contract.
     * @brief [zh] 相电流。这里返回零，因为这块板子没有分流器；真实实现按调制器指定的时刻读 ADC——这也是
     *        `sample_rate_hz` 属于契约的原因。
     */
    struct Current final {
        [[nodiscard]] auto read_uvw() noexcept -> foc::core::Uvw<float> {
            return foc::core::Uvw<float>{0.0F, 0.0F, 0.0F};
        }
        [[nodiscard]] auto calibrate_offset() noexcept -> foc::ErrorCode { return foc::ErrorCode::Ok; }
        [[nodiscard]] auto sample_rate_hz() const noexcept -> float { return 20000.0F; }
    };

    /**
     * @brief [en] Shaft angle from the encoder. `has_sensor() const` is a compile-time-shaped question answered at run
     *        time: a sensorless adapter keeps the member and answers `false`.
     * @brief [zh] 编码器给出的轴角度。`has_sensor() const` 是一个"形状上是编译期、答案在运行期"的问题：无感适配器保留这个
     *        成员并答 `false`。
     */
    struct Position final {
        // The state is named with a trailing underscore on purpose: `angle_mech_rad` and `velocity_rad_s` are the
        // *contract's* method names, and a field may not share one -- a shadowed method is not a missing member, it is
        // a compiler error thirty lines away inside the loop.
        // 状态带尾下划线是故意的：`angle_mech_rad` 与 `velocity_rad_s` 是**契约的**方法名，字段不能与它们同名——被遮蔽的
        // 方法不是"缺成员"，而是一个三十行之外的编译错误。
        float angle_{0.0F};
        float velocity_{100.0F};

        [[nodiscard]] auto angle_mech_rad() noexcept -> float { return angle_; }
        [[nodiscard]] auto velocity_rad_s() noexcept -> float { return velocity_; }
        [[nodiscard]] auto has_sensor() const noexcept -> bool { return true; }
    };

    /**
     * @brief [en] The DC link. One number, read every tick, so a blocking conversion here spends the period budget.
     * @brief [zh] 母线。一个数，每拍读一次，所以这里一次阻塞转换就花掉了整个周期预算。
     */
    struct DcLink final {
        [[nodiscard]] auto voltage_v() noexcept -> float { return kBusV; }
    };

    /**
     * @brief [en] The power stage's fault pin. Answering "quiet" always is a decision this adapter is making, which is
     *        why `hal/fault_input.hpp` is the one contract the library may not default.
     * @brief [zh] 功率级的故障引脚。永远答"安静"是**这个适配器**在做的一个决定——这也是 `hal/fault_input.hpp` 成为唯一
     *        一个库不许替驱动器默认的契约的原因。
     */
    struct FaultInput final {
        [[nodiscard]] auto faulted() noexcept -> bool { return false; }
    };

    Pwm pwm{};
    Current current_sense{};
    Position position{};
    DcLink dc_link{};
    FaultInput fault_input{};
};

using Motor = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, Board>;

auto config() -> foc::motor::MotorConfig {
    // rs, ld, lq, kt, j, b, pole_pairs, i_max, pwm_freq, deadtime
    return foc::motor::MotorConfig{0.5F, 4e-4F, 4e-4F, 0.05F, 1e-5F, 1e-6F, 7U, 5.0F, 20000.0F, 500.0F};
}

auto policies() -> Motor::Policies {
    const foc::core::Pi::Config current{1.0F, 0.0F, -24.0F, 24.0F, -24.0F, 24.0F};
    const foc::core::Pi::Config speed{0.01F, 0.001F, -5.0F, 5.0F, -5.0F, 5.0F};
    return Motor::Policies{foc::core::BackEmfObserver({0.5F, 4e-4F}), foc::core::Svpwm{}, foc::core::Pi{current},
                           foc::core::Pi{current}, foc::core::Pi{speed}};
}

} // namespace

int main() {
    Board board{};
    Motor motor(config(), policies(), board);

    if (motor.init() != foc::ErrorCode::Ok) {
        (void)std::printf("init refused\n");
        return 1;
    }
    if (motor.start() != foc::ErrorCode::Ok) {
        (void)std::printf("start refused\n");
        return 1;
    }
    motor.set_torque(0.5F);

    // The shaft turns because this board says it does: 100 rad/s mechanical, integrated here so the angle and the
    // reported speed stay consistent. An adapter reads both from the encoder instead.
    // 轴之所以在转，是因为这块板子这么说的：100 rad/s 机械，在这里积分，好让角度与报出的转速保持一致。真实适配器从编码器
    // 读这两个数。
    for (int i = 0; i < kTicks; ++i) {
        if (motor.tick(kDtS) != foc::ErrorCode::Ok) {
            (void)std::printf("tick %d refused\n", i);
            return 1;
        }
        board.position.angle_ += board.position.velocity_ * kDtS;
    }

    // What to check: the loop reached the power stage on every tick, and it is still running. A hand-written adapter
    // that answered the wrong member would have failed to compile; one that answered a *plausible* wrong number would
    // still look green here -- which is why the porting page spends most of its words on the channels that lie.
    // 要检查的：环在每一拍都走到了功率级，而且现在还在跑。把成员写错的适配器编译不过；把一个"看起来合理"的错数字答出来的
    // 适配器在这里照样是绿的——这也是移植指南把大部分篇幅花在"会说谎的通道"上的原因。
    if (board.pwm.writes != static_cast<unsigned>(kTicks)) {
        (void)std::printf("the loop wrote %u duties for %d ticks\n", board.pwm.writes, kTicks);
        return 1;
    }
    if (motor.state() != foc::core::MotorState::Run) {
        (void)std::printf("the loop stopped without being asked to\n");
        return 1;
    }

    (void)std::printf("minimal hal: %u duties written, %d ticks, still running at %.1f rad/s\n", board.pwm.writes,
                      kTicks, static_cast<double>(board.position.velocity_));
    return 0;
}
