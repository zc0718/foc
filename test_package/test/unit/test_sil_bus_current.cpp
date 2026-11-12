// Conan::ImportStart
#include <cmath>
#include <cstdio>

#include <gtest/gtest.h>

#include <foc/core/observer/back_emf_observer.hpp>
#include <foc/core/pi.hpp>
#include <foc/core/svpwm.hpp>
#include <foc/core/traits.hpp>
#include <foc/driver/stub_hal.hpp>
#include <foc/motor/motor.hpp>
#include <foc/motor/motor_config.hpp>

#include "sil_bus.hpp"
#include "sil_plant_hal.hpp"
#include "sil_pmsm_plant.hpp"
// Conan::ImportEnd

namespace {

// The bus current, on the bench where it can actually be measured.
//
// The library can read a bus current from a DC link that knows its own (`test_bus_current.cpp`), but until this port had
// a producer in the SIL bench the only numbers it could be checked against were the ones a case made up. Now the plant
// publishes what the bus node carried, and the cases here ask the two questions that are cheap to get wrong: does the
// link hand back the number the plant computed (not a scaled, inverted or mis-averaged version of it), and does the
// number move when the load moves.
//
// What these cases do **not** establish, said here rather than left to be assumed: that the number is physically right.
// The energy identity below compares the link's current against the meter's book, and both are fed by the same
// computation, so it proves the plumbing and not the physics. The physics of this current is already covered next door:
// `test_sil_bus.cpp` holds the d-q power form against the phase form, and the same file closes the meter's energy books.
// Trying to prove it here too would mean two places asserting one thing and drifting apart.
//
// 母线电流，在真能量到它的台架上。
//
// 库能从"知道自己电流"的直流母线上读母线电流（见 `test_bus_current.cpp`），但在 SIL 台架里还没有生产者之前，能拿来对账的
// 数字都是用例自己编的。现在 plant 公布母线节点流过的电流，而这里的用例只问两个最容易被做错的问题：link 交回的 **是不是**
// plant 算出的那个数（而不是它的缩放、取反或错误平均），以及负载变化时这个数会不会跟着变。
//
// 这些用例 **不** 证明什么，写在这里而不是留给读者假定：不证明这个数在物理上是对的。下面的能量恒等式比较的是 link 的电流
// 与 meter 的账，而两者由同一段计算供给，所以它证明的是管线而不是物理。这个电流的物理已经在隔壁覆盖了：
// `test_sil_bus.cpp` 把 d-q 功率形式对着相形式检查，并在同一个文件里把 meter 的能量账做平。在这里再证一遍，等于让两个地方
// 断言同一件事，然后各自漂走。

constexpr float kDt = 50e-6F;
constexpr int kSubsteps = 4;
constexpr int kPolePairs = 7;
constexpr float kVBus = 24.0F;

using MotorT = foc::motor::Motor<foc::core::BackEmfObserver, foc::core::Svpwm, foc::core::Pi, foc::sil::PlantHal>;

auto motor_config() -> foc::motor::MotorConfig {
    return foc::motor::MotorConfig{0.5F, 4.0e-4F, 4.0e-4F, 0.05F, 1.0e-4F, 1.0e-4F, kPolePairs, 3.0F, 20000.0F, 0.0F};
}

struct Bench final {
    explicit Bench(float v_bus)
        : plant(foc::sil::PlantParams{motor_config().rs, motor_config().ld, motor_config().lq, motor_config().kt,
                                      motor_config().j, motor_config().b, kPolePairs}),
          hal(plant, v_bus),
          motor(motor_config(),
                MotorT::Policies{foc::core::BackEmfObserver({motor_config().rs, motor_config().ld}), foc::core::Svpwm{},
                                 foc::core::Pi{foc::current_pi_config(motor_config(), 2000.0F, v_bus)},
                                 foc::core::Pi{foc::current_pi_config(motor_config(), 2000.0F, v_bus)},
                                 foc::core::Pi{foc::speed_pi_config(motor_config(), 250.0F)}},
                hal) {}

    [[nodiscard]] auto bus_current_a() const -> float {
        float amps = 0.0F;
        const bool ok = const_cast<foc::sil::PlantDcLink&>(hal.dc_link).read_bus_current_a(amps);
        EXPECT_TRUE(ok);
        return amps;
    }

    foc::sil::PmsmPlant plant;
    foc::sil::PlantHal hal;
    MotorT motor;
};

// 1. The bench has the capability, and it says "no measurement" until the plant has actually integrated a period.
TEST(SilBusCurrent, TheLinkAnswersOnlyAfterThePlantHasRunAPeriod) {
    static_assert(foc::core::concepts::has_bus_current_v<foc::sil::PlantHal>,
                  "the bench's HalPolicy has to be detected as having a bus current, or nothing below means anything");
    // The shipped stub is detected too, and that is not a contradiction: its link carries the member so a case can drive
    // it, while the bench's link is driven by the plant. What separates them is not the shape but where the number comes
    // from, which is what the next two cases are about.
    // 出厂桩件也被识别，这不矛盾：它的母线带着这个成员是为了让用例去驱动它，而台架的母线由 plant 驱动。区分两者的不是形状，
    // 而是数字从哪来——下一两条用例讲的正是这件事。
    static_assert(foc::core::concepts::has_bus_current_v<foc::driver::StubHal>,
                  "the stub answers the same shape, by design");

    foc::sil::BusMeter meter(foc::sil::BusParams{0.0F, kVBus, 0.0F, 0.0F, 0.0F});
    Bench bench(kVBus);
    bench.hal.set_bus(&meter);

    float amps = 0.0F;
    EXPECT_FALSE(bench.hal.dc_link.read_bus_current_a(amps)) << "nothing has been integrated yet, so there is nothing";

    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.init());
    bench.motor.set_torque(0.5F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.start());
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.tick(kDt));
    bench.hal.advance(kDt, kSubsteps);

    EXPECT_TRUE(bench.hal.dc_link.read_bus_current_a(amps)) << "one period is enough to have measured one";
    // Positive torque in the direction of rotation: current leaves the bus.
    // 转矩为正且与旋转同向：电流流出母线。
    EXPECT_GT(amps, 0.0F);
}

// 2. The link's current integrates into the energy the meter booked: one quantity, two accumulations.
TEST(SilBusCurrent, TheReportedCurrentIntegratesIntoTheEnergyTheMeterBooked) {
    // An ideal source, so the node voltage is constant and the two accumulations differ only in where they happen.
    // 理想源，于是节点电压恒定，两次累加只在"发生在哪里"上不同。
    foc::sil::BusMeter meter(foc::sil::BusParams{0.0F, kVBus, 0.0F, 0.0F, 0.0F});
    Bench bench(kVBus);
    bench.hal.set_bus(&meter);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.init());
    bench.motor.set_torque(0.6F);
    ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.start());

    double from_link_j = 0.0;
    constexpr int kTicks = 200;
    for (int k = 0; k < kTicks; ++k) {
        ASSERT_EQ(foc::ErrorCode::Ok, bench.motor.tick(kDt));
        bench.hal.advance(kDt, kSubsteps);
        from_link_j += static_cast<double>(kVBus) * static_cast<double>(bench.bus_current_a()) * kDt;
    }

    const foc::sil::BusReport report = meter.report();
    std::printf("\n[bus current] link %.6f J, meter %.6f J\n", from_link_j, report.drawn_j);
    ASSERT_GT(report.drawn_j, 0.0) << "a motoring machine has to draw something, or the two numbers agree on zero";
    const double relative = std::fabs(from_link_j - report.drawn_j) / report.drawn_j;
    EXPECT_LT(relative, 2e-4)
        << "the link averages the current it was given and the meter integrates it into energy; a disagreement means one "
           "of them is not the number the plant computed";
}

// 3. It follows the load -- a reading that did not would still pass the case above.
TEST(SilBusCurrent, TheReadingFollowsTheLoadRatherThanSittingAtAConstant) {
    const auto mean_over = [](float iq) {
        foc::sil::BusMeter meter(foc::sil::BusParams{0.0F, kVBus, 0.0F, 0.0F, 0.0F});
        Bench bench(kVBus);
        bench.hal.set_bus(&meter);
        EXPECT_EQ(foc::ErrorCode::Ok, bench.motor.init());
        bench.motor.set_torque(iq);
        EXPECT_EQ(foc::ErrorCode::Ok, bench.motor.start());
        double sum = 0.0;
        constexpr int kTicks = 200;
        for (int k = 0; k < kTicks; ++k) {
            EXPECT_EQ(foc::ErrorCode::Ok, bench.motor.tick(kDt));
            bench.hal.advance(kDt, kSubsteps);
            sum += static_cast<double>(bench.bus_current_a());
        }
        return sum / kTicks;
    };

    const double light = mean_over(0.3F);
    const double heavy = mean_over(0.9F);
    std::printf("[bus current] light %.4f A, heavy %.4f A\n", light, heavy);
    EXPECT_GT(light, 0.0);
    EXPECT_GT(heavy, light) << "three times the torque has to carry more current, not the same number";
}

} // namespace
