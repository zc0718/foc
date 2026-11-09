// Conan::ImportStart
#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/brake_chopper.hpp>

#include "sil_bench.hpp"
#include "sil_bus.hpp"
#include "sil_plant_hal.hpp"
// Conan::ImportEnd

namespace {

using foc::ErrorCode;
using foc::core::BrakeChopper;
using foc::core::BrakeChopperConfig;
using foc::sil::Bench;
using foc::sil::BenchConfig;
using foc::sil::BusMeter;
using foc::sil::BusParams;

// The scenario the chopper exists for: a machine held at speed by a prime mover and commanded to a **regenerating**
// torque, on a bus that has nothing but a capacitor to absorb it.
// 斩波器为之存在的场景：一台被原动机拖住在恒速、被命令去产生 **回馈** 转矩的电机，母线只有一个电容去吸收。
//
// Two fixture decisions are load-bearing and are stated rather than buried:
//
//   * `j = 1e6` kg*m^2. The shaft is dragged by something outside the model, so the mechanical energy comes from
//     outside the bus books and the electrical account has exactly one source to explain. It also makes the run
//     comparable: a real 1e-4 kg*m^2 rotor would decelerate at 3000 rad/s^2 and the operating point would not hold
//     still long enough for two runs to be compared against each other.
//   * the chopper is stepped **once per control period, between the controller and the plant**, which is where a bus
//     supervisor runs in firmware. It is not in `Motor`'s policy list because it does not act on the machine: it keeps
//     working when the drive is disabled, which is exactly when a regenerating machine needs it.
// 两个夹具决定是承重的，写出来而不是埋着：
//
//   * `j = 1e6` kg·m²。轴被模型之外的东西拖着，机械能来自母线账本之外，所以电账上只有一个来源要解释。它同时让运行
//     可比：真实的 1e-4 kg·m² 转子会以 3000 rad/s² 减速，工作点撑不到两次运行可以互相比对。
//   * 斩波器 **每控制周期被推进一次，位置在控制器与 plant 之间** ——固件里母线监督就运行在那里。它不在 `Motor` 的策略
//     列表里，因为它不作用于电机：驱动被关闭之后它还在工作，而那恰好是回馈中的电机最需要它的时候。

constexpr float kVBusInitial = 24.0F;
constexpr float kCapacitanceF = 2e-3F;
constexpr float kResistorOhm = 5.0F;
constexpr float kOmegaMechRadS = 300.0F;
constexpr float kRegenIqA = -1.5F;
constexpr float kMotoringIqA = 1.5F;
constexpr float kWindowS = 30e-3F;
constexpr float kBudget = 1.0F; // A^2*s, generous on purpose: this case is not about the latch
// No cooling in this fixture, and that is what makes the resistor's heat checkable: with cooling on, `r * i2t` is the
// energy dissipated *minus* what the resistor radiated away, so the two numbers would differ by a real physical term and
// the cross-check would have to be padded to accommodate it. The cooling law has its own closed-form case in
// `test_brake_chopper.cpp`; here the question is whether the policy and the bus agree about one resistor.
// 本夹具不冷却，这正是电阻热量可核对的原因：开着冷却时 `r·i²t` 是耗散掉的能量 **减去** 电阻辐射掉的那部分，两个数字会
// 相差一个真实的物理项，对账就必须放宽容差来迁就它。冷却律在 `test_brake_chopper.cpp` 里有自己的闭式用例；这里问的是
// 策略与母线对同一个电阻是否达成一致。
constexpr float kCoolTau = 0.0F; // s

auto plant_held_at_speed() -> foc::sil::PlantParams {
    foc::sil::PlantParams params = foc::sil::nominal_plant_params();
    params.j = 1e6F; // the prime mover, not the rotor
    return params;
}

auto bus_params() -> BusParams {
    return BusParams{kCapacitanceF, kVBusInitial, /*load*/ 0.0F, /*source*/ 0.0F, /*resistor*/ kResistorOhm};
}

auto chopper_config(float budget_a2s) -> BrakeChopperConfig {
    BrakeChopperConfig c{};
    c.v_on = 26.0F;
    c.v_off = 24.5F;
    c.r_ohm = kResistorOhm;
    c.duty_max = 0.35F;
    c.min_on_s = 0.0F; // the hysteresis already sets the limit-cycle period here; the dwell floor has its own case
    c.min_off_s = 0.0F;
    c.i2t_limit_a2s = budget_a2s;
    c.i2t_cool_tau_s = kCoolTau;
    return c;
}

// What one run measured, in one place, so the cases below read as claims rather than as bookkeeping.
// 一次运行量到了什么，集中在一处，于是下面的用例读起来是论断而不是记账。
struct RunResult final {
    double drawn_j;
    double chopper_j;
    double stored_delta_j;
    double residual_j;
    float v_min;
    float v_max;
    double i2t;
    bool latched;
    float iq_a;
};

/**
 * @brief [en] One rig, two runs: the only difference between them is whether a chopper is fitted.
 * @brief [zh] 一个台架，两次运行：两者唯一的差别是装没装斩波器。
 */
class RegenRig final {
  public:
    explicit RegenRig(float budget_a2s) noexcept
        : meter_(bus_params()), config_(foc::sil::standard_bench_config()), bench_(config_, plant_held_at_speed()),
          chopper_(chopper_config(budget_a2s)) {}

    RegenRig(const RegenRig&) = delete;
    RegenRig& operator=(const RegenRig&) = delete;
    RegenRig(RegenRig&&) = delete;
    RegenRig& operator=(RegenRig&&) = delete;

    /**
     * @brief [en] Bring the current loop up **at zero speed**, zero the books, and only then let the shaft turn.
     * @brief [zh] 在 **零速** 下把电流环带起来，清零账面，然后才让轴转起来。
     * @param iq_a [en] the current this run will hold.
     * @param iq_a [zh] 本次运行将保持的电流。
     * @note [en] The order matters and the reason is measured rather than argued. A PI starts with an empty integrator,
     *            so from a cold start at 300 rad/s the machine is effectively shorted through the bridge: it pulls -3.1
     *            A out of the bus before the loop has built the 9.25 V it needs, and charges the bus by 0.8 V in the
     *            first millisecond. That transient is indistinguishable from the effect under test -- it made the
     *            *motoring* case "rise 0.37 V" -- so it must not be inside the window. At zero speed there is no
     *            back-EMF, the loop converges for free (measured: 23.844 V, i.e. the copper loss of 1.5 A in 0.5 ohm),
     *            and stepping the shaft to 300 rad/s afterwards is the scenario itself: a prime mover brings the
     *            machine to speed and the machine starts pushing back.
     * @note [zh] 顺序是要紧的，而理由是量出来的、不是辩出来的。PI 的积分器从空开始，所以在 300 rad/s 上冷启动时，电机实
     *            际上经桥臂被短路：在环路建立起它需要的 9.25 V 之前，它从母线抽出 -3.1 A，并在第一毫秒里把母线充高 0.8
     *            V。那个暂态与待测效应无法区分——它让 **电动** 用例凭空"上升 0.37 V"——所以它不能待在窗口里。零速时没有反
     *            电势，环路白送就收敛了（实测 23.844 V，即 1.5 A 在 0.5 Ω 上的铜损），之后再把轴升到 300 rad/s，那就是
     *            场景本身：原动机把电机带到转速，电机开始往回推。
     */
    auto start(float iq_a) -> ErrorCode {
        bench_.hal().set_bus(&meter_);
        bench_.motor().set_torque(iq_a);
        const ErrorCode rc = bench_.init_and_start();
        if (rc != ErrorCode::Ok) {
            return rc;
        }
        bench_.plant().reset(foc::sil::PlantState{/*id*/ 0.0F, /*iq*/ iq_a, /*omega*/ 0.0, /*theta*/ 0.0});
        run(5e-3F, /*chopper fitted*/ false); // the loop converges at rest; nothing reaches the bus
        meter_.configure(bus_params());       // the books (and the bus) start at the nominal voltage
        chopper_.reset();                     // and the chopper starts cold, whatever the settle did
        bench_.plant().set_mechanical_speed(kOmegaMechRadS);
        return ErrorCode::Ok;
    }

    void run(float seconds, bool chopper_fitted) {
        const int steps = static_cast<int>((seconds / config_.dt_ctrl_s) + 0.5F);
        for (int i = 0; i < steps; ++i) {
            static_cast<void>(bench_.motor().tick(config_.dt_ctrl_s));
            if (chopper_fitted) {
                const auto out = chopper_.step(bench_.hal().dc_link.voltage_v(), config_.dt_ctrl_s);
                bench_.hal().brake.write_duty(out.duty);
            } else {
                bench_.hal().brake.disable();
            }
            bench_.hal().advance(config_.dt_ctrl_s, config_.substeps);
        }
    }

    [[nodiscard]] auto measure() const noexcept -> RunResult {
        const auto report = meter_.report();
        const auto sample = bench_.sample();
        return RunResult{report.drawn_j, report.chopper_j, report.stored_delta_j, report.residual_j,
                         report.v_min,   report.v_max,     chopper_.i2t_a2s(),    chopper_.budget_exceeded(),
                         sample.iq_a};
    }

    [[nodiscard]] auto chopper() noexcept -> BrakeChopper& { return chopper_; }
    [[nodiscard]] auto dt_s() const noexcept -> float { return config_.dt_ctrl_s; }

  private:
    BusMeter meter_;
    BenchConfig config_;
    Bench bench_;
    BrakeChopper chopper_;
};

void report(const char* what, const RunResult& r) {
    std::printf("\n[sil-chopper] %s: drawn %.4f J, resistor %.4f J, stored %.4f J, residual %.3e J\n", what, r.drawn_j,
                r.chopper_j, r.stored_delta_j, r.residual_j);
    std::printf("[sil-chopper]   v in [%.3f, %.3f] V, iq %.3f A, i2t %.4f A^2*s, latched %d\n",
                static_cast<double>(r.v_min), static_cast<double>(r.v_max), static_cast<double>(r.iq_a), r.i2t,
                static_cast<int>(r.latched));
}

// 1. The counterfactual, written first because it establishes the quantity the chopper is supposed to control.
//    Regenerating into a capacitor with nothing to burn the energy leaves it with no destination, and the bus voltage
//    is the only place that can show up. If it does not move here, the bus is not being integrated and the case below
//    would pass for the wrong reason.
// 1. 反事实，放在最前面写，因为它确立了斩波器要控制的那个量。回馈进一个没有地方消耗能量的电容，能量就没有去处，而
//    这件事只能在母线电压上露出来。如果这里它不动，说明母线根本没被积分，那么下面的用例会因为错误的原因通过。
TEST(SilBrakeChopper, WithoutAChopperTheBusRunsAway) {
    RegenRig rig(kBudget);
    ASSERT_EQ(rig.start(kRegenIqA), ErrorCode::Ok);
    rig.run(kWindowS, /*chopper fitted*/ false);
    const RunResult r = rig.measure();
    report("no chopper", r);

    ASSERT_LT(r.iq_a, -1.0F) << "the fixture must actually regenerate, or nothing below means anything";
    ASSERT_LT(r.drawn_j, 0.0) << "the inverter must push energy *into* the bus for this to be the chopper's problem";
    EXPECT_EQ(r.chopper_j, 0.0) << "nothing was fitted, so the resistor cannot have taken anything";
    // The bus rises and there is nowhere else for the energy to be: the capacitor's own books are the whole account.
    EXPECT_GT(r.v_max, 32.0F) << "a capacitor-only bus fed by a regenerating machine has to run away";
    EXPECT_LE(r.v_min, kVBusInitial + 0.01F) << "and it must not sag on the way: only regeneration is happening";
    EXPECT_NEAR(r.stored_delta_j, -r.drawn_j, 1e-6 * std::fabs(r.drawn_j))
        << "with no resistor fitted the stored energy *is* the regenerated energy";
}

// 2. The same run with a chopper fitted. Every number here is checked against another number rather than against a
//    constant: the voltage against a bound derived from the measured bus current, and the resistor's heat against the
//    bus meter's own account of the same energy.
// 2. 同样的运行，装上斩波器。这里每个数字都对另一个数字检查，而不是对一个常数：电压对着一个由实测母线电流推出来的
//    界，电阻的热对着母线 meter 对同一份能量的账。
TEST(SilBrakeChopper, TheChopperHoldsTheBusAndTheBooksClose) {
    RegenRig rig(kBudget);
    ASSERT_EQ(rig.start(kRegenIqA), ErrorCode::Ok);
    rig.run(kWindowS, /*chopper fitted*/ true);
    const RunResult r = rig.measure();
    report("with chopper", r);

    ASSERT_LT(r.drawn_j, 0.0) << "same fixture as the counterfactual: it must still regenerate";
    EXPECT_GT(r.chopper_j, 0.0) << "the resistor has to be taking the energy, not the capacitor";

    // The upper limit, with a bound derived from this run rather than guessed: the chopper acts on the voltage it sees
    // at the start of a period, so it can be at most one period's rise late. That rise is `(i/C)*dt`, with `i` the
    // average bus current this run measured -- conservative, because the current *falls* as the voltage rises.
    const double mean_bus_a =
        std::fabs(r.drawn_j) / (static_cast<double>(kWindowS) * static_cast<double>(kVBusInitial));
    const double rise_per_period_v =
        (mean_bus_a / static_cast<double>(kCapacitanceF)) * static_cast<double>(rig.dt_s());
    std::printf("[sil-chopper]   mean bus current %.3f A -> at most %.4f V of rise per control period\n", mean_bus_a,
                rise_per_period_v);
    EXPECT_LE(r.v_max, 26.0F + (2.0 * rise_per_period_v))
        << "the chopper turns on at v_on and must not be late by more";
    // The lower limit is `v_off`, but the run *starts* at 24 V, below it -- so the statement is that the chopper never
    // took the bus below where it was allowed to take it: `min(v_start, v_off)`, minus one period's fall. A chopper with
    // no hysteresis would keep dumping past `v_off` and this is the line that says so.
    const double floor_v = (kVBusInitial < 24.5F) ? static_cast<double>(kVBusInitial) : 24.5;
    EXPECT_GE(r.v_min, floor_v - (2.0 * rise_per_period_v)) << "and it must not pull the bus below v_off either";

    // Energy closure: the capacitor changed, the resistor took something, and the inverter delivered it. The residual
    // is what the law does not explain. Derived, not picked: the current is constant over each integration sub-step and
    // the bus voltage is linear in time across it, so the trapezoid this book uses is *exact* -- the residual is
    // round-off, which puts it at 1e-12 of the energies involved and not at "a few percent".
    const double scale = std::fabs(r.drawn_j) + std::fabs(r.chopper_j) + std::fabs(r.stored_delta_j);
    EXPECT_LT(std::fabs(r.residual_j), 1e-9 * scale) << "the bus books have to close, or the port is fiction";
    EXPECT_NEAR(r.stored_delta_j, -(r.drawn_j + r.chopper_j), 1e-9 * scale);

    // The resistor's heat, computed twice: by the policy (I2t) and by the meter (the energy that crossed the node).
    // They agree up to the zero-order hold the policy works under -- it uses the voltage at the start of the period
    // while the meter uses the sub-step mean -- which is one period's drift, ~0.1 % here.
    const double from_policy_j = rig.chopper().i2t_a2s() * static_cast<double>(kResistorOhm);
    std::printf("[sil-chopper]   resistor energy: meter %.6f J, policy r*i2t %.6f J (%.4f %% apart)\n", r.chopper_j,
                from_policy_j, 100.0 * (from_policy_j - r.chopper_j) / r.chopper_j);
    EXPECT_NEAR(from_policy_j, r.chopper_j, 0.005 * r.chopper_j);

    // And the budget was respected without being spent: this run must not be the latch case.
    EXPECT_FALSE(r.latched) << "1 A^2*s over 30 ms of chopping is meant to be comfortable";
    EXPECT_LE(r.i2t, static_cast<double>(kBudget));
}

// 3. The sign half of the same claim. The chopper's duty is only ever non-zero when the bus is being charged by the
//    machine; a chopper that fired while motoring would burn the energy the supply just delivered, and every number in
//    case 2 would still look fine.
// 3. 同一论断的符号那一半。斩波器的占空比只有在母线正被电机充电时才非零；一个在电动时也动作的斩波器会烧掉电源刚送来的
//    能量，而用例 2 里的每个数字看起来仍然正常。
TEST(SilBrakeChopper, TheChopperDoesNotFireWhileTheMachineMotors) {
    RegenRig rig(kBudget);
    ASSERT_EQ(rig.start(kMotoringIqA), ErrorCode::Ok);
    rig.run(5e-3F, /*chopper fitted*/ true); // 5 ms: the sag stays inside the modulation headroom
    const RunResult r = rig.measure();
    report("motoring", r);

    ASSERT_GT(r.iq_a, 1.0F) << "the fixture must actually motor";
    ASSERT_GT(r.drawn_j, 0.0) << "motoring means energy flows out of the bus";
    EXPECT_EQ(r.chopper_j, 0.0) << "a brake resistor that takes energy while motoring is a heater, not a brake";
    EXPECT_EQ(rig.chopper().i2t_a2s(), 0.0);
    // The reason it did not fire, stated so that "it did not fire" cannot be true because the chopper was broken: the
    // bus never went anywhere near the threshold. It does rise 0.74 V at the start -- the current loop taking up the
    // speed step -- and that transient is left visible rather than trimmed away, because it is the closest this run gets
    // to the band and the case is stronger for showing the distance than for claiming the bus only ever fell.
    EXPECT_LT(r.v_max, 26.0F) << "the bus must stay below v_on, or this case stops being about the sign";
    EXPECT_LT(r.v_min, kVBusInitial - 0.3F) << "and it must have sagged, or the port is not running here at all";
}

// 4. The latch's consequence, measured on the bus instead of on a flag. A spent budget is allowed to be a failure --
//    what is not allowed is for it to be a *quiet* one, and the honest failure is a bus that keeps climbing.
// 4. 锁存的后果，在母线上量而不是在一个标志上量。预算是允许花完的——不允许的是它 **安静地** 花完，而诚实的失败就是一条
//    继续上升的母线。
TEST(SilBrakeChopper, ASpentBudgetIsVisibleOnTheBus) {
    RegenRig rig(/*budget*/ 0.02F); // trips after about 2 ms of chopping, well inside the window
    ASSERT_EQ(rig.start(kRegenIqA), ErrorCode::Ok);
    rig.run(kWindowS, /*chopper fitted*/ true);
    const RunResult r = rig.measure();
    report("spent budget", r);

    ASSERT_TRUE(r.latched) << "0.02 A^2*s must not survive 30 ms of a 5 ohm resistor on a 25 V bus";
    EXPECT_LE(r.i2t, static_cast<double>(0.02F) * 1.05)
        << "the latch fires on the first period past the budget, so the overshoot is one period's worth";
    // The visible failure: past the latch the chopper is a switch that never closes, so the bus is the counterfactual's
    // bus again -- and it has the rest of the window to say so.
    EXPECT_GT(r.v_max, 26.0F + 2.0F) << "a latched chopper leaves the bus with nowhere to put the energy";
    EXPECT_LT(r.chopper_j, 0.5) << "and it can only have taken what it took before the latch";
}

} // namespace
