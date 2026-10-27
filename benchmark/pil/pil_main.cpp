// PIL entry point: the *same* S2 scenario the host GTest runs, compiled for the target ISA.
//
// Why this file lives under benchmark/ and not under test_package/: `test-conventions.md` gives
// `test_package` to host-side GTest and `benchmark/` to the target side. PIL is the target side, so
// it borrows the target slot instead of inventing a test directory the template does not have.
// The scenario itself is *not* copied: the include below reaches the single definition the unit
// tests use, so a change to the plant or the bench reaches PIL automatically.
//
// PIL 入口：与宿主 GTest **同一个** S2 场景，按目标 ISA 编译。
// 放在 benchmark/ 而不是 test_package/：test-conventions.md 把 test_package 给宿主 GTest、
// 把 benchmark/ 给目标侧。PIL 属于目标侧，所以借目标槽位，而不是发明一个模板里没有的测试目录。
// 场景本身**不复制**：下面的 include 直接取单元测试用的那一份定义，plant 或台架改了，PIL 自动跟上。

#include <cstdio>

#include "../../test_package/test/unit/sil_s2_case.hpp"



int main() {
    const foc::sil::S2Result result = foc::sil::run_s2(foc::sil::nominal_plant_params(), 100.0F, 0.5F);

    // One key=value per line: the harness parses this, and a human can read it.
    // 每行一个 key=value：脚本解析它，人也能直接读。
    std::printf("overshoot=%.9g\n", static_cast<double>(result.overshoot));
    std::printf("steady_error_fraction=%.9g\n", static_cast<double>(result.steady_error_fraction));
    std::printf("tail_peak_to_peak=%.9g\n", static_cast<double>(result.tail_peak_to_peak));
    std::printf("settling_s=%.9g\n", static_cast<double>(result.settling_s));
    std::printf("speed_final=%.9g\n", static_cast<double>(result.speed_final));
    std::printf("peak_iq=%.9g\n", static_cast<double>(result.peak_iq));
    return 0;
}
