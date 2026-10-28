// PIL entry point: the *same* speed-step scenario the host GTest runs, compiled for the target ISA.
//
// Why this file lives under benchmark/ and not under test_package/: `test-conventions.md` gives
// `test_package` to host-side GTest and `benchmark/` to the target side. PIL is the target side, so
// it borrows the target slot instead of inventing a test directory the template does not have.
// The scenario itself is *not* copied: the include below reaches the single definition the unit
// tests use, so a change to the plant or the bench reaches PIL automatically.
//
// PIL 入口：与宿主 GTest **同一个**速度阶跃场景，按目标 ISA 编译。
// 放在 benchmark/ 而不是 test_package/：test-conventions.md 把 test_package 给宿主 GTest、
// 把 benchmark/ 给目标侧。PIL 属于目标侧，所以借目标槽位，而不是发明一个模板里没有的测试目录。
// 场景本身**不复制**：下面的 include 直接取单元测试用的那一份定义，plant 或台架改了，PIL 自动跟上。

#include <cstdio>

#include <foc/core/board_check.hpp>
#include <foc/core/commissioning.hpp>
#include <foc/core/field_weakening.hpp>
#include <foc/core/protection.hpp>
#include <foc/core/reference.hpp>

#include "../../test_package/test/unit/sil_speed_step_case.hpp"



int main() {
    const foc::sil::SpeedStepResult result = foc::sil::run_speed_step(foc::sil::nominal_plant_params(), 100.0F, 0.5F);

    // One key=value per line: the harness parses this, and a human can read it.
    // 每行一个 key=value：脚本解析它，人也能直接读。
    std::printf("overshoot=%.9g\n", static_cast<double>(result.overshoot));
    std::printf("steady_error_fraction=%.9g\n", static_cast<double>(result.steady_error_fraction));
    std::printf("tail_peak_to_peak=%.9g\n", static_cast<double>(result.tail_peak_to_peak));
    std::printf("settling_s=%.9g\n", static_cast<double>(result.settling_s));
    std::printf("speed_final=%.9g\n", static_cast<double>(result.speed_final));
    std::printf("peak_iq=%.9g\n", static_cast<double>(result.peak_iq));

    // --- the modules added after the speed-step scenario, on the same target ISA ---
    // --- 速度阶跃场景之后新增的模块，在同一个目标 ISA 上 ---
    //
    // Why these belong in PIL rather than only in the host tests: the target is a different FPU with
    // a different libm, and every one of these modules either takes a square root, a tangent or a
    // cosine, or integrates a state across thousands of steps. A host-only green build says nothing
    // about whether the target agrees -- the speed-step comparison exists because that difference is real.
    // 为什么这些该进 PIL 而不只是宿主测试：目标机是另一个 FPU、另一套 libm，而这些模块要么开方、
    // 要么取正切、要么取余弦，要么把状态积分几千步。只在宿主上绿，对"目标是否给出同一个答案"
    // 什么也没说——速度阶跃的比对本就是因为这个差别是真实存在的。

    const foc::core::MachineParams machine{0.5F, 4.0e-4F, 4.0e-4F, 4.7619048e-3F};
    const auto verdict = foc::core::is_field_weakening_worthwhile(machine, 3.0F);
    std::printf("fw_i_char_a=%.9g\n", static_cast<double>(verdict.i_char_a));
    std::printf("fw_ratio=%.9g\n", static_cast<double>(verdict.ratio));
    std::printf("fw_speed_gain=%.9g\n", static_cast<double>(verdict.speed_gain));

    // MTPA on a salient machine: the sqrt is what could differ between libms.
    // The allowance is stated here rather than defaulted: this case is about the arithmetic of the
    // curve, not about whether a sensorless observer could see the result.
    // 凸极机上的 MTPA：可能因 libm 不同而不同的就是那个开方。许可量在这里写明而不用默认值：这个
    // 用例考的是曲线的算术，不是无感观测器是否看得见结果。
    foc::core::MtpaReference mtpa(
        foc::core::MtpaReference::Config{2.0e-4F, 4.0e-4F, 4.7619048e-3F, 7.0F, 1.0F});
    foc::core::ReferenceInput ref{};
    ref.iq_command_a = 5.0F;
    ref.omega_e_rad_s = 3200.0F;
    const foc::core::Dq<float> mtpa_i = mtpa.generate(ref);
    std::printf("mtpa_id_a=%.9g\n", static_cast<double>(mtpa_i.d));
    std::printf("mtpa_iq_a=%.9g\n", static_cast<double>(mtpa_i.q));

    // The field-weakening operating point at 3200 rad/s with 1 A of torque.
    // 3200 rad/s、1 A 转矩下的弱磁工作点。
    foc::core::FieldWeakeningReference fw(foc::core::FieldWeakeningReference::Config{
        0.5F, 4.0e-4F, 4.7619048e-3F, 3.0F, 0.57735027F});
    foc::core::ReferenceInput fw_in{};
    fw_in.iq_command_a = 1.0F;
    fw_in.omega_e_rad_s = 3200.0F;
    fw_in.v_bus_v = 24.0F;
    const foc::core::Dq<float> fw_i = fw.generate(fw_in);
    std::printf("fw_id_a=%.9g\n", static_cast<double>(fw_i.d));
    std::printf("fw_iq_a=%.9g\n", static_cast<double>(fw_i.q));

    // The dead-time axis projection at three angles: cos in three places.
    // 三个角度上的死区轴投影：三处余弦。
    std::printf("deadtime_d=%.9g\n", static_cast<double>(foc::core::axis_deadtime_v(0.48F, 0.0F)));
    std::printf("deadtime_q=%.9g\n",
                static_cast<double>(foc::core::axis_deadtime_v(0.48F, 1.57079632F)));

    // The protector integrates for 2000 steps, so a rounding difference compounds instead of hiding.
    // 保护器积分 2000 步，于是舍入差别会累积而不是藏起来。
    foc::core::ProtectionConfig pc{};
    pc.i_hard_a = 4.0F;
    pc.i_max_a = 3.0F;
    pc.i2t_a2s = 4.5F;
    pc.i2t_cooling_s = 0.5F;
    pc.v_max_v = 30.0F;
    pc.v_min_v = 18.0F;
    pc.t_trip_c = 100.0F;
    pc.t_derate_start_c = 60.0F;
    pc.t_derate_end_c = 110.0F;
    pc.stall_speed_rad_s = 5.0F;
    pc.stall_current_a = 1.0F;
    pc.stall_time_s = 0.5F;
    pc.sync_error_ratio = 0.25F;
    pc.hysteresis_ratio = 0.05F;
    foc::core::Protector protector(pc);
    foc::core::ProtectionInput pin{};
    pin.v_bus_v = 24.0F;
    pin.temperature_c = 70.0F; // in the derating band, so the limit moves
    pin.flux_valid = true;
    pin.i_magnitude_a = 3.2F;
    for (int k = 0; k < 2000; ++k) {
        static_cast<void>(protector.step(pin, 5.0e-5F));
    }
    std::printf("protector_budget=%.9g\n", static_cast<double>(protector.thermal_budget()));

    // The board checklist: a fixed input, so any disagreement is arithmetic rather than a scenario.
    // 板级检查表：固定输入，于是任何不一致都是算术而不是场景。
    foc::core::BoardMeasurements measured{};
    measured.response_sign[0] = 1.0F;
    measured.response_sign[1] = -1.0F;
    measured.response_sign[2] = -1.0F;
    measured.ripple_measured_a = 0.30F;
    measured.ripple_expected_a = 0.375F;
    measured.deadtime_v = 0.640F;
    measured.deadtime_cmd_v = 0.640F;
    measured.rs_ohm = 0.500F;
    measured.rs_nameplate_ohm = 0.500F;
    measured.psi_f_wb = 4.7619048e-3F;
    measured.psi_f_nameplate_wb = 4.7619048e-3F;
    measured.observer_error_rad = 0.04F;
    measured.encoder_offset_rad = 0.005F;
    const foc::core::BoardReport board = foc::core::check_board(measured, foc::core::BoardLimits{});
    std::printf("board_failures=%.9g\n", static_cast<double>(board.failures));
    std::printf("board_cause=%d\n", static_cast<int>(board.cause));

    return 0;
}
