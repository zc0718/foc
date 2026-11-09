// One machine, one shift -- the walkthrough behind `guides/walkthrough.rst`.
//
// 一台电机、一个班次——`guides/walkthrough.rst` 背后的贯通脚本。
//
// Seven stages in the order an axis meets them: read the nameplate, check the board,
// align and hand over, take a torque step, run past base speed, position, and fail
// safely. Every number the page states is printed by this program, and the page
// includes this file instead of quoting it, so the two cannot drift.
//
// 按一条轴实际遇到的顺序走七步：读铭牌、自检、对齐交接、转矩阶跃、过基速、定位、安全失效。
// 页面上出现的每个数都由本程序打印；页面内联本文件而不是转述它，于是两者无法漂移。
//
// Build (same flags as the figure step; `sil_bench.hpp` is the SIL plant the test
// suite itself uses, so stages 4 and 6 run on the same model the tests assert on):
// 编译（与画图步骤同参数；`sil_bench.hpp` 就是测试套件自己用的 SIL 被控对象，因此第 4、6 步
// 跑在与测试断言相同的模型上）：
//
//   g++ -std=c++17 -O2 -Iinclude -Itest_package/test/unit docs/sphinx/scripts/walkthrough.cxx
//       $(find src -name '*.cpp' -o -name '*.c' | grep -v '/version\.') -o /tmp/walkthrough
//   /tmp/walkthrough

#include <cmath>
#include <cstdio>
#include <cstddef>
#include <vector>

#include <foc/core/board_check.hpp>
#include <foc/core/machine_map_text.hpp>
#include <foc/core/machine_params.hpp>
#include <foc/core/motion_profile.hpp>
#include <foc/core/mtpv.hpp>
#include <foc/core/protection.hpp>
#include <foc/core/soft_limit.hpp>
#include <foc/core/startup.hpp>
#include <foc/core/types.hpp>

#include "sil_bench.hpp"
#include "sil_metrics.hpp"

namespace {

constexpr float kDtS = 5.0e-5F;       // 20 kHz control loop / 控制周期
constexpr std::uint32_t kPolePairs = 7U;
constexpr float kVMaxV = 12.0F;       // the modulator's inscribed circle at 24 V DC
constexpr float kTwoPi = 6.28318530718F;

/// One printed row. The page's table is this output, copied once: a quantity, its
/// value and its unit, in the order the stages produce them.
/// 一行打印结果。页面上的表就是这段输出，只抄一次：量、值、单位，按各步产生的顺序。
void row(int stage, const char* quantity, double value, const char* unit) {
    std::printf("  %2d  %-38s %12.5g  %s\n", stage, quantity, value, unit);
}

/// Same shape, for a conclusion rather than a number (a verdict, a fault name).
/// 同一种行，用于结论而不是数字（判定、故障名）。
void verdict(int stage, const char* quantity, const char* value) {
    std::printf("  %2d  %-38s %12s  -\n", stage, quantity, value);
}

// ---------------------------------------------------------------------------
// Stage 1 -- the nameplate. Data in, data out: nothing here is derived from the
// machine except `Kt`, and `Kt` is derived from the flux, not the other way round.
// 第 1 步 —— 铭牌。数据进、数据出：除了 `Kt`，这里没有任何东西是从机器推出来的，而 `Kt`
// 是从磁链推出来的，不是反过来。
// ---------------------------------------------------------------------------
void stage_nameplate() {
    const foc::core::MachineParams nameplate{0.5F, 4.0e-3F, 8.0e-3F, 0.01F};
    row(1, "Rs", static_cast<double>(nameplate.rs), "ohm");
    row(1, "Ld", static_cast<double>(nameplate.ld), "H");
    row(1, "Lq", static_cast<double>(nameplate.lq), "H");
    row(1, "psi_f", static_cast<double>(nameplate.psi_f), "Wb");
    // `Kt = 1.5 * p * psi_f`: the one conversion this page performs, and the one
    // place where a factor of 1.5 or a pole-pair count gets lost in the field.
    // `Kt = 1.5 * p * psi_f`：本页唯一的换算，也是现场最容易丢掉 1.5 或极对数的地方。
    row(1, "Kt = 1.5 * pole_pairs * psi_f", 1.5 * static_cast<double>(kPolePairs) * nameplate.psi_f, "N.m/A");

    // The same nameplate as a file. A document without map rows carries only `rs`:
    // the per-current parameters live in the rows, so a round trip that is worth
    // anything has to include one.
    // 同一份铭牌当作文件。没有映射行的文档只带 `rs`：逐电流的参数活在行里，所以有意义的往返
    // 必须带一行。
    foc::core::MachineDocument document{};
    document.params = nameplate;
    document.map = foc::core::MachineMap::from_params(nameplate, 3.0F);
    char text[512]{};
    const std::size_t written = foc::core::write_document(document, text, sizeof(text));
    foc::core::MachineDocument readback{};
    foc::core::TextError error{};
    const bool ok = (written > 0U) && foc::core::read_document(text, written, &readback, &error);
    row(1, "nameplate file: size", static_cast<double>(written), "B");
    row(1, "nameplate file: map rows", static_cast<double>(readback.map.count), "rows");
    verdict(1, "nameplate round trip",
            (ok && (readback.params.rs == nameplate.rs) && (readback.map.count == document.map.count)) ? "same"
                                                                                                    : "differs");
}

// ---------------------------------------------------------------------------
// Stage 2 -- the board, before it is allowed to make torque. Eight items, then a
// single cause, because "two items failed" is not something a person can act on.
// 第 2 步 —— 出力之前的自检。八项，然后给出一个成因，因为"两项失败"不是人能处理的东西。
// ---------------------------------------------------------------------------
void stage_board() {
    foc::core::BoardMeasurements measurements{};
    measurements.sum_current_a = 0.004F;
    measurements.offset_a[0] = 0.001F;
    measurements.offset_a[1] = -0.002F;
    measurements.offset_a[2] = 0.001F;
    // A positive d-axis drive puts `+i` on u and `-i/2` on v and w, so health is a
    // *pattern* of signs and not "all positive". The inverted case below is the one
    // a reader who only counts failures cannot tell from this one.
    // 正 d 轴驱动在 u 上给 `+i`、在 v 与 w 上给 `−i/2`，所以健康是一组**符号模式**，不是
    // “全为正”。下面那个反相案例，正是只数失败项的人分不出来的那一种。
    measurements.response_sign[0] = 1.0F;
    measurements.response_sign[1] = -1.0F;
    measurements.response_sign[2] = -1.0F;
    measurements.ripple_expected_a = 0.10F;
    measurements.ripple_measured_a = 0.11F;
    measurements.deadtime_v = 0.30F;
    measurements.deadtime_cmd_v = 0.30F;
    measurements.encoder_offset_rad = 0.003F;
    measurements.rs_ohm = 1.02F;
    measurements.rs_nameplate_ohm = 1.00F;
    measurements.psi_f_wb = 0.0101F;
    measurements.psi_f_nameplate_wb = 0.0100F;
    measurements.observer_error_rad = 0.02F;

    const foc::core::BoardLimits limits{};
    const foc::core::BoardReport good = foc::core::check_board(measurements, limits);
    row(2, "board items checked", static_cast<double>(good.count), "items");
    row(2, "items failed (healthy board)", static_cast<double>(good.failures), "items");
    verdict(2, "cause on a healthy board", foc::core::board_cause_name(good.cause));

    // One channel inverted: the failure that hides the others, because a flipped
    // sign makes the ripple comparison meaningless. It is named first for a reason.
    // 一个通道符号反了：这一项会遮住其余项，因为符号翻转之后纹波比较没有意义。它被第一个
    // 报出来是有原因的。
    foc::core::BoardMeasurements inverted = measurements;
    inverted.response_sign[1] = 1.0F;
    inverted.response_sign[2] = 1.0F;
    const foc::core::BoardReport bad = foc::core::check_board(inverted, limits);
    row(2, "items failed (one sign inverted)", static_cast<double>(bad.failures), "items");
    verdict(2, "cause named", foc::core::board_cause_name(bad.cause));
}

// ---------------------------------------------------------------------------
// Stage 3 -- align, pull, hand over. The routine decides an angle, a current and a
// moment; it does not know whether the rotor came along, and this stage does not
// pretend otherwise (that is what the hand-over check one window later is for).
// 第 3 步 —— 对齐、拖动、交接。例行决定一个角度、一个电流、一个时刻；它不知道转子有没有跟上，
// 本步也不假装知道（那是一个窗口之后的交接检查负责的事）。
// ---------------------------------------------------------------------------
void stage_startup() {
    foc::core::StartupConfig recipe{};
    recipe.align_current_a = 1.5F;
    recipe.align_hold_s = 0.02F;
    recipe.pull_current_a = 2.0F;
    recipe.pull_accel_rad_s2 = 40.0F;
    recipe.handover_speed_rad_s = 60.0F;

    // The refusal first: a hand-over speed of zero is a recipe that can never end.
    // 先看拒绝：交接转速为零等于一个永远不会结束的配方。
    foc::core::StartupConfig broken = recipe;
    broken.handover_speed_rad_s = 0.0F;
    verdict(3, "recipe with hand-over 0 rad/s", (foc::core::StartupRoutine{broken}.validate() == foc::ErrorCode::Ok)
                                                   ? "accepted"
                                                   : "refused");

    foc::core::StartupRoutine startup{recipe};
    if (!startup.begin(kPolePairs)) {
        verdict(3, "begin()", "refused");
        return;
    }
    float t_s = 0.0F;
    foc::core::AlignOutput out{};
    float hold_s = 0.0F;
    while (t_s < 3.0F) {
        out = startup.step(kDtS, 0.0F);
        t_s += kDtS;
        if (startup.state() == foc::core::StartupState::Align) {
            hold_s = t_s;
        }
        if (out.finished) {
            break;
        }
    }
    row(3, "hold (align) lasts", static_cast<double>(hold_s), "s");
    row(3, "pull + hold, to hand-over", static_cast<double>(t_s), "s");
    row(3, "hand-over speed (electrical)", static_cast<double>(out.handover_speed_e_rad_s), "rad/s");
    row(3, "hand-over speed (mechanical)",
        static_cast<double>(out.handover_speed_e_rad_s / static_cast<float>(kPolePairs)), "rad/s");
    row(3, "commanded angle at hand-over", static_cast<double>(out.theta_e_rad), "rad");
    verdict(3, "sequence finished", out.finished ? "yes" : "no");
}

// ---------------------------------------------------------------------------
// Stage 4 -- a torque step on the closed loop. Same bench, same metric helpers as
// the figure step, so this number and the metrics table on the `Measured
// responses` page are computed by the same code.
// 第 4 步 —— 闭环上的转矩阶跃。同一个台架、与画图步骤同一套指标函数，所以这个数与
// "实测响应"页上的指标表由同一段代码算出。
// ---------------------------------------------------------------------------
void stage_torque_step() {
    foc::sil::Bench bench(foc::sil::standard_bench_config());
    if (bench.init_and_start() != foc::ErrorCode::Ok) {
        verdict(4, "bench", "refused to start");
        return;
    }
    const int idle = static_cast<int>((0.005F / kDtS) + 0.5F);
    const int hold = static_cast<int>((0.020F / kDtS) + 0.5F);
    for (int i = 0; i < idle; ++i) {
        bench.tick_once();
    }
    std::vector<foc::sil::Sample> step;
    step.reserve(static_cast<std::size_t>(hold));
    for (int i = 0; i < hold; ++i) {
        bench.motor().set_torque(1.0F);
        bench.tick_once();
        step.push_back(bench.sample());
    }
    const std::vector<float> iq = foc::sil::series_of(step, &foc::sil::Sample::iq_a);
    row(4, "iq step: settling to 5 %", static_cast<double>(foc::sil::settling_time_s(step, &foc::sil::Sample::iq_a, 1.0F, 0.05F)), "s");
    row(4, "iq step: overshoot", static_cast<double>(100.0F * foc::sil::overshoot_fraction(iq, 1.0F, 0.0F)), "%");
    row(4, "iq step: steady-state error", static_cast<double>(1.0F - foc::sil::tail_mean(iq, 0.2F)), "A");
}

// ---------------------------------------------------------------------------
// Stage 5 -- past base speed. The solver answers where the voltage ceiling starts
// to bind, which is the only honest way to say "this machine can go faster".
// 第 5 步 —— 过基速。求解器回答电压上限从哪里开始约束，这是"这台机器能更快"唯一诚实的说法。
// ---------------------------------------------------------------------------
void stage_high_speed() {
    const foc::core::MachineParams machine{0.5F, 4.0e-3F, 8.0e-3F, 0.01F};
    const foc::core::MtpvConfig config{machine, kVMaxV, 3.0F, kPolePairs};
    foc::core::Mtpv solver(config);
    if (solver.validate() != foc::ErrorCode::Ok) {
        verdict(5, "machine / ceiling", "refused");
        return;
    }
    const foc::core::MtpvPoint mtpa = solver.mtpa_point();
    const float base = solver.base_speed_rad_s();
    row(5, "base speed (electrical)", static_cast<double>(base), "rad/s");
    row(5, "base speed (mechanical)", static_cast<double>(base / static_cast<float>(kPolePairs)), "rad/s");
    row(5, "MTPA id at the current ceiling", static_cast<double>(mtpa.id_a), "A");
    row(5, "MTPA iq at the current ceiling", static_cast<double>(mtpa.iq_a), "A");

    // Where the ellipse takes over: sweep and report the first speed that is
    // voltage-limited, then say what the d axis had to give up to get there.
    // 椭圆从哪里接管：扫一遍，报出第一个受电压约束的转速，再说为了到那里 d 轴付出了什么。
    float ratio = 0.0F;
    for (int i = 1; i <= 200; ++i) {
        const float candidate = 0.05F * static_cast<float>(i);
        if (solver.at_speed(candidate * base).voltage_limited) {
            ratio = candidate;
            break;
        }
    }
    row(5, "voltage binds first at", static_cast<double>(ratio), "x base");
    const foc::core::MtpvPoint at_twice = solver.at_speed(2.0F * base);
    row(5, "id at 2x base", static_cast<double>(at_twice.id_a), "A");
    row(5, "torque at 2x base relative to MTPA",
        static_cast<double>(at_twice.torque_nm / ((mtpa.torque_nm > 0.0F) ? mtpa.torque_nm : 1.0F)), "x");
    verdict(5, "past base speed", "weakened");
}

// ---------------------------------------------------------------------------
// Stage 6 -- a move. The profile shapes the velocity, the library's speed loop
// closes it on the SIL plant, and the soft limit is asked first so a target
// behind a wall never becomes a reference.
// 第 6 步 —— 一次移动。型线给出速度，库自带的速度环在 SIL 被控对象上闭合，软限位先被问一次，
// 于是墙后的目标永远不会变成参考。
// ---------------------------------------------------------------------------
void stage_position() {
    foc::core::SoftLimit limit(foc::core::SoftLimitConfig{-1.0F, 1.0F});
    const foc::core::SoftLimitOutput inside = limit.apply(0.5F);
    const foc::core::SoftLimitOutput beyond = limit.apply(1.5F);
    row(6, "target 0.5 accepted as", static_cast<double>(inside.target_rad), "rad");
    verdict(6, "target 1.5 rad against wall 1.0", (beyond.status == foc::ErrorCode::Ok) ? "accepted" : "refused");
    row(6, "previous target returned instead", static_cast<double>(beyond.target_rad), "rad");

    foc::core::MotionProfile::Config shaping{};
    shaping.shaping_gain_rad_s = 8.0F;
    shaping.velocity_limit_rad_s = 5.0F;
    shaping.accel_limit_rad_s2 = 20.0F;
    foc::core::MotionProfile profile{shaping};

    foc::sil::Bench bench(foc::sil::standard_bench_config());
    if (bench.init_and_start() != foc::ErrorCode::Ok) {
        verdict(6, "bench", "refused to start");
        return;
    }
    const float target = limit.apply(0.8F).target_rad;
    float t_s = 0.0F;
    float worst_velocity = 0.0F;
    while (t_s < 1.5F) {
        const double theta = bench.plant().state().theta_mech;
        const float velocity = profile.step(target, static_cast<float>(theta), kDtS);
        worst_velocity = (std::fabs(velocity) > worst_velocity) ? std::fabs(velocity) : worst_velocity;
        bench.motor().set_speed(velocity);
        bench.tick_once();
        t_s += kDtS;
    }
    const double landed = bench.plant().state().theta_mech;
    row(6, "peak velocity commanded", static_cast<double>(worst_velocity), "rad/s");
    row(6, "move duration", static_cast<double>(t_s), "s");
    row(6, "residual error at 1.5 s", std::fabs(static_cast<double>(target) - landed), "rad");
    row(6, "landed at", landed, "rad");
}

// ---------------------------------------------------------------------------
// Stage 7 -- fail safely. Below the hard trip the question is "how long"; above
// the derating start the question is "how much"; after a trip the question is
// "who may clear it".
// 第 7 步 —— 安全失效。硬跳闸以下问"多久"；降额起点以上问"多少"；跳闸之后问"谁可以清"。
// ---------------------------------------------------------------------------
void stage_protection() {
    foc::core::ProtectionConfig config{};
    config.i_hard_a = 4.0F;
    config.i_max_a = 3.0F;
    config.i2t_a2s = 4.5F;
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

    const auto healthy = [](float current_a, float temperature_c) {
        foc::core::ProtectionInput input{};
        input.i_magnitude_a = current_a;
        input.v_bus_v = 24.0F;
        input.temperature_c = temperature_c;
        input.omega_e_rad_s = 100.0F;
        input.flux_error_ratio = 0.01F;
        input.flux_valid = true;
        return input;
    };

    // 3.4 A: above the continuous rating, below the hard trip. The budget is
    // `i_cont^2 * tau`, so only the excess spends it -- the time is finite and
    // it is long, which is exactly what makes an i2t limit different from a fuse.
    // 3.4 A：高于连续额定、低于硬跳闸。预算是 `i_cont²·τ`，所以只有超出的部分花余量——
    // 时间是有限的，而且很长，这正是 i2t 与保险丝的区别。
    constexpr float kDt = 100.0e-6F;
    foc::core::Protector overloaded{config};
    float trip_s = 0.0F;
    foc::core::ProtectionOutput output{};
    while (trip_s < 30.0F) {
        output = overloaded.step(healthy(3.4F, 40.0F), kDt);
        trip_s += kDt;
        if (output.fault != foc::core::FaultCode::None) {
            break;
        }
    }
    row(7, "3.4 A continuous: trip after", static_cast<double>(trip_s), "s");
    verdict(7, "fault latched", output.output_allowed ? "no" : "yes");
    row(7, "derated limit at 80 C", static_cast<double>(foc::core::Protector(config).derated_limit(80.0F)), "A");
    row(7, "derated limit at 105 C", static_cast<double>(foc::core::Protector(config).derated_limit(105.0F)), "A");

    // The latch is the point of a trip: it does not clear because the current came
    // back. `reset()` clears the latched faults and deliberately keeps the i2t
    // memory -- the winding is still hot -- so the axis is *still* blocked. Only
    // `reset_cold()` lets it run again, and that one is for a machine that has been
    // off long enough to have cooled.
    // 锁存才是跳闸的意义：电流回来了它也不清。 `reset()` 清掉锁存故障，却有意保留 i2t
    // 记忆——绕组还热着——所以轴**依然**被拦住。只有 `reset_cold()` 能让它重新跑，而那个是
    // 给已经断电够久、确实凉了的机器用的。
    const foc::core::ProtectionOutput still_tripped = overloaded.step(healthy(0.5F, 40.0F), kDt);
    verdict(7, "healthy input after the trip", still_tripped.output_allowed ? "allowed" : "still blocked");
    overloaded.reset();
    const foc::core::ProtectionOutput after_reset = overloaded.step(healthy(0.5F, 40.0F), kDt);
    verdict(7, "after reset()", after_reset.output_allowed ? "allowed" : "still blocked");
    overloaded.reset_cold();
    const foc::core::ProtectionOutput after_cold = overloaded.step(healthy(0.5F, 40.0F), kDt);
    verdict(7, "after reset_cold()", after_cold.output_allowed ? "allowed" : "still blocked");
}

} // namespace

auto main() -> int {
    std::printf("one machine, one shift -- %u pole pairs, 24 V bus, %.0f kHz control loop\n\n",
                static_cast<unsigned>(kPolePairs), static_cast<double>((1.0F / kDtS) / 1000.0F));
    std::printf(" stage  %-38s %12s  %s\n", "quantity", "value", "unit");

    stage_nameplate();
    stage_board();
    stage_startup();
    stage_torque_step();
    stage_high_speed();
    stage_position();
    stage_protection();

    return 0;
}
