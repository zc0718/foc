// Conan::ImportStart
#pragma once
#include <cmath>
#include <cstdint>
#include <foc/core/brake_chopper.hpp>
#include <foc/core/protection.hpp>
#include <foc/core/sm.hpp>
#include <foc/core/traits.hpp>
#include <foc/core/types.hpp>
#include <optional>
// Conan::ImportEnd

/*!
 * @ingroup group_motor
 * @file drive_supervisor.hpp
 *
 * @brief [en] One tick of the whole drive: bus, protection, chopper, then the control loop.
 * @brief [zh] 整个驱动器的一个 tick：母线、保护、斩波，然后才是控制环。
 * @since 0.1.0
 */

namespace foc {

namespace motor {

/**
 * @brief [en] What the supervisor concluded on the tick just run.
 * @brief [zh] 监督器在刚跑完的那个 tick 上的结论。
 * @note [en] `temperature_c` is `NaN` when no channel answered, which is a different statement from "0 C" and is the
 *            only way a caller can tell "no derating because it is cool" from "no derating because nobody measured".
 *            The two look identical in `i_limit_a`, and treating them as the same is how a hot machine keeps its full
 *            authority.
 * @note [zh] 没有通道回答时 `temperature_c` 是 `NaN`，这与"0 °C"是两句不同的话，也是调用方唯一能区分"因为凉所以不降额"
 *            与"因为没人测所以不降额"的地方。两者在 `i_limit_a` 上完全一样，而把它们当成一回事正是热机器保有全部权限的
 *            方式。
 * @since 0.1.0
 * @exporter
 */
struct SupervisorStatus final {
    /**
     * @brief [en] the highest-ranked active fault, `None` while everything is in order.
     * @brief [zh] 排名最高的活动故障；一切正常时为 `None`。
     */
    core::FaultCode fault;
    /**
     * @brief [en] bit `1 << code` for each active fault.
     * @brief [zh] 每个活动故障一位 `1 << code`。
     */
    std::uint16_t active_mask;
    /**
     * @brief [en] the same, for the faults that will not clear on their own.
     * @brief [zh] 同上，但只含不会自行清除的故障。
     */
    std::uint16_t latched_mask;
    /**
     * @brief [en] the current limit after derating, 0 while tripped.
     * @brief [zh] 降额后的电流限幅，跳闸时为 0。
     */
    float i_limit_a;
    /**
     * @brief [en] the winding temperature that was used, `NaN` when no channel answered.
     * @brief [zh] 实际用到的绕组温度；没有通道回答时为 `NaN`。
     */
    float temperature_c;
    /**
     * @brief [en] the bus voltage this tick was decided on, V.
     * @brief [zh] 本 tick 据以判定的母线电压，V。
     */
    float v_bus_v;
    /**
     * @brief [en] the brake chopper's duty this tick, 0 when it is not dissipating.
     * @brief [zh] 本 tick 制动斩波器的占空比，不耗能时为 0。
     */
    float chopper_duty;
    /**
     * @brief [en] whether the control loop was allowed to run.
     * @brief [zh] 控制环是否被允许运行。
     */
    bool output_allowed;
    /**
     * @brief [en] whether the shaft is being held by the mechanical brake; `false` on a drive that has none.
     * @brief [zh] 轴是否被机械抱闸抱住；没有抱闸的驱动上为 `false`。
     * @note [en] Kept next to the fault because it answers the second question a stop raises: not only "did it stop",
     *            but "is something holding it now".
     * @note [zh] 与故障并列，因为它回答了一次停机提出的第二个问题：不只是"停了没有"，还有"现在有东西抱着它吗"。
     */
    bool brake_engaged;
    /**
     * @brief [en] how many ticks have been stepped, for a caller that has to say when something happened.
     * @brief [zh] 已经走过的 tick 数，供需要说明"某事发生在何时"的调用方使用。
     */
    std::uint32_t tick;
};

/**
 * @brief [en] The drive's tick order, in one place: sense the bus, protect, keep the chopper alive, then switch.
 *
 *   Everything this type does is a statement about **order**, because each of these pieces is correct on its own and
 *   useless in the wrong sequence:
 *
 *   1. **The chopper runs first and unconditionally.** It acts on the DC link, not on the machine, so it keeps
 *   dissipating after the inverter is disabled -- cutting it off with the inverter is how a braking robot puts its
 *   energy into the capacitors instead of the resistor.
 *
 *   2. **Protection decides this tick, not the next one.** `Motor` already latches the power stage's own fault pin
 *   before it runs the controller; what was missing was the five causes nobody else could see (over-current,
 *   over/under-voltage, over-temperature, and the `I2t` budget). A protection that needs a crane to lift its own weight
 *   is a report, not a protection.
 *
 *   3. **PWM off before the fault is recorded.** `stop()` then `fault(code)`, never the other way: a state that says
 *   "protection acted" while the output is still switching is a lie the operator will act on.
 *
 *   4. **A stopped drive still has a bus.** The chopper keeps stepping while the motor is in `Fault`, so an
 *   over-voltage caused by a coasting load is still absorbed.
 *
 *   What it is **not**, said here so nobody assumes it: it does not own the current limit's *enforcement*. It publishes
 *   `i_limit_a` and stops on a trip, but a winding that is merely hot is not a fault and must not be treated as one --
 *   the limit has to reach the controller as a reference ceiling, which is the caller's `Command::max_current_a`.
 *   Derating that silently refused commands instead would look like a broken user.
 *
 * @brief [zh] 驱动器一拍的顺序，集中在一处：先看母线，再保护，再让斩波器活着，最后才开关。
 *
 *   本类型做的一切都是关于 **顺序** 的断言，因为这每一件单独看都对，放错先后就都没用：
 *
 *   1. **斩波器最先跑，且无条件跑。** 它作用于直流母线、不作用于电机，所以逆变器关掉之后它继续耗能——把斩波器和逆变器一
 *   起切掉，正是"制动机器人"把能量灌进电容而不是电阻的方式。
 *
 *   2. **保护在** 本拍 **决定，不是下一拍。** `Motor` 已经在跑控制器之前锁存功率级自己的故障引脚；缺的是别人看不见的五
 *   个成因（过流、过压/欠压、过温、 `I2t` 预算）。一个需要先把自己搬起来的保护是报告，不是保护。
 *
 *   3. **先关 PWM 再记故障。** 先 `stop()` 再 `fault(code)`，绝不反过来：一个说着"保护已动作"而输出还在开关的状态，是运
 *   维会照着做决定的谎。
 *
 *   4. **停下的驱动器仍然有母线。** 电机处于 `Fault` 时斩波器继续走，于是滑行负载造成的过压仍然被吸收。
 *
 *   它 **不是** 什么，写在这里免得有人假定：它不拥有电流限幅的 **执行** 。它发布 `i_limit_a`、跳闸时停机，但"只是有点热
 *   "不是故障，也绝不能当成故障处理——限值必须作为给定上界送到控制器，那属于调用方的 `Command::max_current_a`。用静默拒
 *   绝指令的方式去降额，看起来会像用户坏了。
 *
 * @context [en] Current-loop ISR, every step, before the control loop.
 * @context [zh] 电流环 ISR，每步，在控制环之前。
 * @tparam MotorType [en] a `foc::motor::Motor`.
 * @tparam MotorType [zh] 一个 `foc::motor::Motor`。
 * @tparam HalType [en] the same HalPolicy instance the motor was built on.
 * @tparam HalType [zh] 与电机所基于的同一个 HalPolicy 实例。
 * @tparam ProtectorType [en] the fault classifier; `core::Protector` unless a case substitutes one.
 * @tparam ProtectorType [zh] 故障分类器；除用例替身外为 `core::Protector`。
 * @tparam ChopperType [en] the bus dissipator; `core::BrakeChopper` unless a case substitutes one.
 * @tparam ChopperType [zh] 母线耗能器；除用例替身外为 `core::BrakeChopper`。
 * @since 0.1.0
 * @exporter
 */
template <typename MotorType, typename HalType, typename ProtectorType = core::Protector,
          typename ChopperType = core::BrakeChopper>
class DriveSupervisor final {
  public:
    /**
     * @brief [en] The numbers that are not measurements: the protection thresholds, the chopper's band, and how many
     *             electrical cycles one mechanical turn is worth.
     * @brief [zh] 不是测量值的那几个数：保护阈值、斩波器的区间，以及机械一圈折合多少个电周期。
     * @since 0.1.0
     */
    struct Config final {
        /**
         * @brief [en] the fault thresholds, including the derating band.
         * @brief [zh] 故障阈值，含降额区间。
         */
        core::ProtectionConfig protection{};
        /**
         * @brief [en] the chopper's bus band and resistor.
         * @brief [zh] 斩波器的母线区间与电阻。
         */
        core::BrakeChopperConfig chopper{};
        /**
         * @brief [en] electrical revolutions per mechanical revolution.
         * @brief [zh] 每机械圈对应的电周期数。
         */
        float pole_pairs{1.0F};
    };

    /**
     * @brief [en] Bind to the motor and the hardware it runs on.
     * @brief [zh] 绑定到电机以及它运行其上的硬件。
     * @param motor [en] the motor instance, by reference: this type owns neither.
     * @param motor [zh] 电机实例，按引用：本类型不拥有其中任何一个。
     * @param hal [en] the HalPolicy the motor was built on.
     * @param hal [zh] 电机所基于的 HalPolicy。
     * @since 0.1.0
     */
    DriveSupervisor(MotorType& motor, HalType& hal) noexcept : motor_(motor), hal_(hal) {}

    /**
     * @brief [en] Accept the configuration, or refuse it before a single tick runs.
     * @brief [zh] 接受这份配置，或者在还没跑一拍之前就拒绝它。
     * @param config [en] the thresholds; nothing is stored when any of them is refused.
     * @param config [zh] 阈值；其中任何一项被拒绝时什么都不保存。
     * @return [en] `Param` for a band that cannot be a band (a trip point under its own derating start, a chopper that
     *              switches on below where it switches off, a pole-pair count that is not positive).
     * @return [zh] 无法成立的区间返回 `Param`（跳闸点低于自己的降额起点、斩波器在低于关断点的地方开通、极对数不为正）。
     * @note [en] Checked here rather than on every tick because these are constants: a threshold that is wrong is wrong
     *            once, and finding out on tick 4000 is the expensive version of finding out now.
     * @note [zh] 在这里检查而不是每拍检查，因为这些是常数：错的阈值只错一次，而到第 4000 拍才发现是现在就能发现的那件事
     *            的昂贵版本。
     * @since 0.1.0
     */
    auto configure(const Config& config) noexcept -> ErrorCode {
        const core::ProtectionConfig& p = config.protection;
        const bool band_ok = (p.i_hard_a > 0.0F) && (p.i_max_a > 0.0F) && (p.v_max_v > p.v_min_v) &&
                             (p.t_trip_c > p.t_derate_end_c) && (p.t_derate_end_c > p.t_derate_start_c);
        const bool budget_ok = (p.i2t_a2s > 0.0F) && (p.i2t_cooling_s > 0.0F);
        const bool counts_ok = config.pole_pairs > 0.0F;
        if (!band_ok || !budget_ok || !counts_ok || !std::isfinite(config.pole_pairs)) {
            return ErrorCode::Param;
        }
        chopper_.set_config(config.chopper);
        if (chopper_.validate() != ErrorCode::Ok) {
            return ErrorCode::Param;
        }
        config_ = config;
        // Constructed, not assigned: `Protector` takes its thresholds through its only constructor, and that is the
        // right shape -- a classifier that can exist without thresholds can also be stepped without them.
        // 构造而不是赋值：`Protector` 只通过它唯一的构造函数接收阈值，而那个形状是对的——一个可以没有阈值就存在的
        // 分类器，也就可以没有阈值就被步进。
        protector_.emplace(config.protection);
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] One tick of the drive, in the order documented on the type.
     * @brief [zh] 驱动器的一拍，顺序见类型文档。
     * @param dt_s [en] step length in s; `<= 0` is refused as `Param` and changes nothing.
     * @param dt_s [zh] 步长，s； `<= 0` 被拒绝为 `Param` 且什么都不改。
     * @return [en] `Ok` when the loop ran, `Fault` when this tick was refused by protection or by the motor, `Param`
     *              for a step that is not a step, or whatever the motor reported.
     * @return [zh] 控制环跑完为 `Ok`；本拍被保护或电机拒绝为 `Fault`；步长非法为 `Param`；其余照电机所报。
     * @post [en] A tripped tick leaves `output_allowed == false`, the PWM disabled, and the motor's state at `Fault`.
     * @post [zh] 跳闸的一拍以 `output_allowed == false`、PWM 已关、电机状态为 `Fault` 结束。
     * @since 0.1.0
     */
    auto step(float dt_s) noexcept -> ErrorCode {
        if (!(dt_s > 0.0F)) {
            return ErrorCode::Param;
        }
        if (!protector_.has_value()) {
            // Not a convenience check. Without thresholds there is nothing to compare against, and a supervisor that
            // quietly runs an unprotected drive is the failure this whole type is supposed to prevent.
            // 不是顺手加的检查。没有阈值就没有可比较的东西，而一个默默让驱动器不设保护地跑起来的监督器，正是本类型存在的
            // 意义所在的反面。
            return ErrorCode::State;
        }
        ++status_.tick;

        // Bus first and unconditionally: the chopper acts on the link, not on the machine, so it must keep working
        // after the inverter is disabled. `Motor::tick()` reads the bus again for its own reference and modulation --
        // two reads of the same register in one period, which is what a real ADC does anyway; the alternative was a
        // supervisor that decides on a voltage the loop never saw.
        // 先看母线，且无条件：斩波器作用于母线、不作用于电机，所以逆变器关掉之后它必须继续工作。`Motor::tick()` 会为自己的
        // 给定与调制再读一次母线——同一周期读两次同一个寄存器，真 ADC 本来也是这样；另一条路是让监督器基于一个控制环从未
        // 见过的电压做判断。
        const float v_bus_v = hal_.dc_link.voltage_v();
        status_.v_bus_v = v_bus_v;
        const auto chopper_out = chopper_.step(v_bus_v, dt_s);
        status_.chopper_duty = chopper_out.duty;

        // The five causes nobody else in the library can see. `Motor` already latches the power stage's own pin at
        // the top of its tick, so this is not a duplicate of that check -- it is the other five codes.
        // 库中其他部分看不见的五个成因。`Motor` 已经在自己那拍的开头锁存功率级自己的引脚，所以这不是重复检查——这里管的是
        // 另外五个码。
        const core::ProtectionInput input = make_input();
        const core::ProtectionOutput out = protector_->step(input, dt_s);
        status_.fault = out.fault;
        status_.active_mask = out.active_mask;
        status_.latched_mask = out.latched_mask;
        status_.i_limit_a = out.i_limit_a;
        status_.output_allowed = out.output_allowed;
        record();

        if (!out.output_allowed) {
            // The brake is asked to hold *before* the output goes away, from the same rule `disable()` follows: a
            // stopped drive must not become a dropping load. Its answer does not delay the stop, though -- a protection
            // that waits for a brake is not a protection -- so a refusal is recorded rather than returned.
            // 抱闸在输出消失 *之前* 被要求抱住，规则与 `disable()` 相同：停下的驱动器不能变成一个下坠的负载。但它的回答不
            // 会推迟停机——等抱闸的保护不是保护——所以拒绝被记录下来而不是被返回。
            if constexpr (core::concepts::has_mechanical_brake_v<HalType>) {
                (void)hal_.mechanical_brake.engage();
                status_.brake_engaged = hal_.mechanical_brake.engaged();
            }
            // PWM off, *then* the state. `fault()` disables as well, so the order is not about who does the
            // disabling -- it is about never publishing "protection acted" for a period in which the loop still
            // switched. `stop()` first also means the refusal happens on a machine that is already quiet.
            // 先关 PWM，*再* 记状态。`fault()` 自己也会关，所以顺序不是关于谁来关——而是关于绝不为一个控制环还在开关的
            // 周期发布"保护已动作"。先 `stop()` 也意味着拒绝发生在一台已经安静下来的机器上。
            if (motor_.state() != core::MotorState::Fault) {
                (void)motor_.stop();
                motor_.fault(out.fault);
            }
            return ErrorCode::Fault;
        }
        return motor_.tick(dt_s);
    }

    /**
     * @brief [en] The verdict of the last tick.
     * @brief [zh] 最近一拍的结论。
     * @return [en] the status; untouched before the first tick except for `tick == 0`.
     * @return [zh] 状态；首拍之前除 `tick == 0` 外保持原样。
     * @since 0.1.0
     */
    [[nodiscard]] auto status() const noexcept -> const SupervisorStatus& { return status_; }

    /**
     * @brief [en] The fault classifier, for its thermal budget and stall timer.
     * @brief [zh] 故障分类器，用于读取它的热预算与堵转计时。
     * @since 0.1.0
     */
    [[nodiscard]] auto protector() noexcept -> ProtectorType& { return protector_.value(); }

    /**
     * @brief [en] The bus dissipator, for its resistor heat.
     * @brief [zh] 母线耗能器，用于读取它的电阻热量。
     * @since 0.1.0
     */
    [[nodiscard]] auto chopper() noexcept -> ChopperType& { return chopper_; }

    /**
     * @brief [en] Power the drive up in the one order that never leaves the load held by nothing.
     *
     *   Start the loop first, then let the brake go. The other order is a vertical axis taking a step down on every
     *   enable: the brake lets go of a shaft that nothing is holding yet. What makes this worth a call rather than a
     *   comment is the failure path -- if the brake refuses to release, the loop is stopped again and the error is
     *   returned, because a drive left running against a brake that thinks it is still applied is a stall with a
     *   current limit.
     *
     * @brief [zh] 用唯一一种不会让负载一时无人抱住的顺序把驱动器使能起来。
     *
     *   先启动控制环，再让抱闸松开。相反的顺序会让垂直轴每次使能都往下掉一格：抱闸松开一根还没有任何东西抱住的轴。让它
     *   成为一个调用而不是一句注释的，是失败路径——如果抱闸拒绝松开，控制环会被重新停掉并返回错误，因为一个对着"自认为仍
     *   夹紧"的抱闸继续跑的驱动器，是一次带着电流限幅的堵转。
     *
     * @return [en] `Ok`; whatever the motor said; or whatever the brake refused with, after stopping the motor again.
     * @return [zh] `Ok`；电机的回答；或抱闸拒绝的原因，并在那之前把电机重新停掉。
     * @note [en] A drive with no brake takes the same path with one step fewer. It does **not** get a state invented
     *            for a device that is not there.
     * @note [zh] 没有抱闸的驱动走同一条路，只是少一步。它 **不会** 为一个不存在的设备编一个状态出来。
     * @since 0.1.0
     */
    auto enable() noexcept -> ErrorCode {
        const ErrorCode started = motor_.start();
        if (started != ErrorCode::Ok) {
            return started;
        }
        if constexpr (core::concepts::has_mechanical_brake_v<HalType>) {
            const ErrorCode released = hal_.mechanical_brake.release();
            if (released != ErrorCode::Ok) {
                // The shaft is still held, which is the safe half of this; the unsafe half would be staying enabled
                // against a brake that thinks it is applied.
                // 轴仍被抱住，这是安全的那一半；不安全的那一半是对着一个"自认为仍夹紧"的抱闸保持使能。
                (void)motor_.stop();
                status_.brake_engaged = hal_.mechanical_brake.engaged();
                return released;
            }
            status_.brake_engaged = hal_.mechanical_brake.engaged();
        }
        return ErrorCode::Ok;
    }

    /**
     * @brief [en] Stop the drive in the one order that never leaves the load held by nothing.
     *
     *   Hold first, then stop switching. The opposite -- cut the inverter and then apply the brake -- is a load that
     *   free-falls for as long as the brake takes to grip, which is tens of milliseconds and therefore metres.
     *
     * @brief [zh] 用唯一一种不会让负载一时无人抱住的顺序把驱动器停下来。
     *
     *   先抱住，再停止开关。反过来——先关逆变器再抱闸——是负载在抱闸夹紧所需的那段时间里自由下落，那段时间是几十毫秒，因
     *   此也就是好几米。
     *
     * @return [en] `Ok`; whatever the brake refused with, **with the motor left running**; or whatever the motor said.
     * @return [zh] `Ok`；抱闸拒绝的原因（此时 **电机仍在运行** ）；或电机的回答。
     * @warning [en] A brake that refuses to engage does not stop the drive. Cutting the output anyway would turn a
     *               failed brake into a dropped load, so the error is returned with the loop still enabled and the
     *               caller is told to deal with it.
     * @warning [zh] 拒绝夹紧的抱闸不会让驱动器停下。照样切断输出会把一次抱闸失效变成一次负载坠落，所以返回错误时控制环
     *               仍在使能，并把这件事交给调用方处理。
     * @since 0.1.0
     */
    auto disable() noexcept -> ErrorCode {
        if constexpr (core::concepts::has_mechanical_brake_v<HalType>) {
            const ErrorCode held = hal_.mechanical_brake.engage();
            // Asked, not inferred: `engage()` returning `Ok` and the shaft being held are two different claims.
            // 去问，而不是推断：`engage()` 返回 `Ok` 与轴被抱住是两句不同的话。
            status_.brake_engaged = hal_.mechanical_brake.engaged();
            if ((held != ErrorCode::Ok) || !status_.brake_engaged) {
                return (held != ErrorCode::Ok) ? held : ErrorCode::State;
            }
        }
        return motor_.stop();
    }

    /**
     * @brief [en] How many verdicts the record keeps.
     * @brief [zh] 档案保留多少条结论。
     * @note [en] Sixteen at a 50 us period is 0.8 ms of history -- long enough to hold the bus sag that preceded an
     *            over-current on the same control frame, and short enough to stay in the core's own memory.
     * @note [zh] 50 us 周期下十六条是 0.8 ms 的历史——足够装下同一控制帧里过流之前的母线跌落，又短到能留在内核自己的内存
     *            里。
     * @since 0.1.0
     */
    static constexpr std::size_t kSnapshotCapacity = 16U;

    /**
     * @brief [en] Copy the recorded verdicts out, oldest first.
     * @brief [zh] 把记录下来的结论拷出来，最旧的在前。
     * @param out [en] non-NULL destination for up to `capacity` verdicts.
     * @param out [zh] 非 NULL 的目的缓冲区，最多 `capacity` 条。
     * @param capacity [en] how many entries `out` can take.
     * @param capacity [zh] `out` 能装多少条。
     * @return [en] how many were written; when the buffer is shorter than the record, the **newest** entries are the
     *              ones handed back, in chronological order among themselves.
     * @return [zh] 写入了多少条；缓冲区比档案短时，交回的是 **最新的** 那几条，它们之间仍按时间排序。
     * @note [en] A copy rather than a span over the ring, because a ring is not contiguous: an accessor that returned
     *            one would either have to unwrap on every read or hand back two disjoint ranges, and both invite the
     *            caller to read it backwards.
     * @note [zh] 拷贝而不是给环一个 span，因为环不连续：那样一个访问器要么每次读都展开，要么交回两段互不相连的区间，而
     *            两者都会招来"从后往前读"的调用方。
     * @since 0.1.0
     */
    auto snapshot(SupervisorStatus* out, std::size_t capacity) const noexcept -> std::size_t {
        if ((out == nullptr) || (capacity == 0U)) {
            return 0U;
        }
        const std::size_t take = (held_ < capacity) ? held_ : capacity;
        const std::size_t skip = held_ - take;
        for (std::size_t i = 0U; i < take; ++i) {
            // `skip` drops the oldest when the caller's buffer is shorter than the record.
            const std::size_t slot = ((head_ + kSnapshotCapacity - held_) + skip + i) % kSnapshotCapacity;
            out[i] = ring_[slot];
        }
        return take;
    }

    /**
     * @brief [en] How many verdicts are held.
     * @brief [zh] 档里存了多少条结论。
     * @return [en] the count, at most `kSnapshotCapacity`.
     * @return [zh] 条数，最多 `kSnapshotCapacity`。
     * @since 0.1.0
     */
    [[nodiscard]] auto snapshot_size() const noexcept -> std::size_t { return held_; }

    /**
     * @brief [en] The tick on which something was first wrong, `0` while nothing has been.
     * @brief [zh] 第一次出问题的那一 tick；一直没出过问题时为 `0`。
     * @return [en] the tick number, which is the same number `SupervisorStatus::tick` carries.
     * @return [zh] tick 号，与 `SupervisorStatus::tick` 是同一个编号。
     * @note [en] Not derived by scanning the record: the record is short, and the interesting tick is often the first
     *            one that was wrong rather than the first one still held.
     * @note [zh] 不去扫档案推出来：档案很短，而有意义的那一 tick 常常是第一个出问题的，不是第一个还留在档案里的。
     * @since 0.1.0
     */
    [[nodiscard]] auto fault_tick() const noexcept -> std::uint32_t { return fault_tick_; }

    /**
     * @brief [en] Clear the latched faults, in the right order: the motor leaves `Fault` first, then the classifier
     *             forgets.
     * @brief [zh] 清除锁存故障，顺序正确：电机先离开 `Fault`，分类器再忘记。
     * @return [en] whatever `Motor::clear_fault()` said.
     * @return [zh] `Motor::clear_fault()` 的回答。
     * @note [en] `Protector::reset()` deliberately keeps the `I2t` budget: the winding is still hot. Clearing it here
     *            would undo that decision from the outside. The record survives too, and for the same kind of reason:
     *            clearing a fault is a request to run again, not a request to forget what happened. `fault_tick()`
     *            still names the tick that went wrong.
     * @note [zh] `Protector::reset()` 是故意保留 `I2t` 预算的：绕组还热着。在这里把它清掉会从外部推翻那个决定。档案同样
     *            保留，理由属于同一类：清故障是要求重新跑，不是要求忘掉发生过什么。 `fault_tick()` 仍然指着出事的那一
     *            tick。
     * @since 0.1.0
     */
    auto clear() noexcept -> ErrorCode {
        const ErrorCode code = motor_.clear_fault();
        if (protector_.has_value()) {
            protector_->reset();
        }
        status_.fault = core::FaultCode::None;
        status_.active_mask = 0;
        status_.latched_mask = 0;
        status_.output_allowed = true;
        return code;
    }

  private:
    /**
     * @brief [en] Put this tick's verdict into the record.
     * @brief [zh] 把本拍的结论记进档案。
     * @note [en] Fixed size and no allocation, because this runs in the control loop; a record that can fail to
     *            allocate is a record that disappears exactly when memory is tight, which is when a fault is likely.
     * @note [zh] 固定容量、不分配，因为它在控制环里跑；会分配失败的档案，恰好在内存紧张时消失，而内存紧张也正是故障容易
     *            发生的时候。
     * @since 0.1.0
     */
    void record() noexcept {
        ring_[head_] = status_;
        head_ = (head_ + 1U) % kSnapshotCapacity;
        if (held_ < kSnapshotCapacity) {
            ++held_;
        }
        if ((status_.fault != core::FaultCode::None) && (fault_tick_ == 0U)) {
            // The first tick on which anything was wrong, kept until somebody asks for a new recording. Recording and
            // clearing are different actions: an operator who clears a fault is asking to run again, not asking to
            // forget what happened.
            // 第一个出问题的 tick，一直保留到有人要求重新记录。记录与清除是两件事：清故障的运维是在要求重新跑，不是在要求忘掉
            // 发生过什么。
            fault_tick_ = status_.tick;
        }
    }

    /**
     * @brief [en] Gather this tick's observations from the hardware and the loop's own telemetry.
     * @brief [zh] 从硬件与控制环自身的遥测里收集本拍的观测量。
     * @return [en] the protector's input, with every field accounted for.
     * @return [zh] 保护器的输入，每个字段都有交代。
     * @note [en] Two fields are deliberately not measurements. `temperature_c` falls back to `t_derate_start_c` when no
     *            channel answered, because the alternative is a `NaN` that poisons `i_limit_a` through the derating
     *            arithmetic -- the truthful record of "nobody measured" is `status().temperature_c`, and the caller can
     *            see it there. `flux_valid` is `true` with a zero error, because `Protector` reads `false` as "the
     *            observer has lost sync" and that judgement belongs to `Motor`, which has the observer. The current is
     *            the one the loop **already converted**, not a second read of the same shunt: two conversions per
     *            period is how a drive ends up with an aliased sample that disagrees with the one the controller acted
     *            on. So the price is one tick of latency, and it is stated rather than hidden -- an over-current
     *            applied during tick N reaches this input at tick N+1, and the output stops on that tick. The bus
     *            voltage is the exception, read here directly, because it is a slow signal and reading it twice costs
     *            nothing.
     * @note [zh] 有两个字段故意不是测量值。没有通道回答时 `temperature_c` 退到 `t_derate_start_c`，因为另一条路是一个
     *            `NaN`，它会顺着降额算术毒化 `i_limit_a`——"没人测过"的诚实记录在 `status().temperature_c` 里，调用方在
     *            那里看得见。 `flux_valid` 取 `true` 且误差为 0，因为 `Protector` 把 `false` 读作"观测器失锁"，而那个判
     *            断属于拥有观测器的 `Motor`。电流用的是控制环 **已经换算出** 的那一份，不是对同一路分流电阻的第二次采
     *            样：每周期两次采样正是驱动器拿到一个与控制器所用不一致的混叠样本的方式。所以代价是一拍延迟，而这一条写
     *            出来而不是藏起来——在第 N 拍施加的过流在这个输入上第 N+1 拍才可见，输出也在那一拍停下。母线电压是例外，
     *            在这里直接读，因为它是慢信号，读两次不花钱。
     * @since 0.1.0
     */
    [[nodiscard]] auto make_input() noexcept -> core::ProtectionInput {
        core::ProtectionInput input{};
        const auto telemetry = motor_.telemetry();
        input.i_magnitude_a = std::sqrt((telemetry.id_a * telemetry.id_a) + (telemetry.iq_a * telemetry.iq_a));
        input.v_bus_v = status_.v_bus_v;
        input.omega_e_rad_s = telemetry.speed_mech_rad_s * config_.pole_pairs;
        float temperature_c = std::nanf("");
        if constexpr (core::concepts::has_temperature_v<HalType>) {
            const auto reading = hal_.temperature.read();
            if (reading.valid) {
                temperature_c = reading.celsius;
            }
        }
        status_.temperature_c = temperature_c;
        input.temperature_c = std::isnan(temperature_c) ? config_.protection.t_derate_start_c : temperature_c;
        input.flux_valid = true;
        input.flux_error_ratio = 0.0F;
        return input;
    }

    MotorType& motor_;
    HalType& hal_;
    std::optional<ProtectorType> protector_{};
    ChopperType chopper_{};
    Config config_{};
    SupervisorStatus status_{};
    std::array<SupervisorStatus, kSnapshotCapacity> ring_{};
    std::size_t head_{0U};
    std::size_t held_{0U};
    std::uint32_t fault_tick_{0U};
};

} // namespace motor

} // namespace foc
