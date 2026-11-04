// `comm::register_table`, measured: is every register backed by a C function that really answers, is the table the only
// place a register number is written, and does a refused read leave the caller's word alone?
//
// The criteria, written before the run:
//
//   1. the ids are exactly 1..kRegisterCount, each once: a host may enumerate without a hole table, and a duplicate is
//      the one defect that makes two registers answer each other's numbers;
//   2. a real read through a real motor agrees with the C entry point it names -- `State` is `Run` after `enable()`,
//      and `FloatBits` decodes to the *same float* `foc_get_state` returns, bit for bit, not to a number that prints
//      the same;
//   3. the two kinds of C function behave differently on a NULL handle and the table must not paper over it:
//      `foc_motor_get_status` refuses (so the register read fails and the word is untouched), while
//      `foc_motor_trace_size` documents NULL as 0 (so the register read succeeds with 0). A layer that normalised both
//      into one answer would be inventing a third behaviour;
//   4. an id that is not in the table is refused rather than guessed at.
//
// 判据写在跑前（见上）：编号恰为 1..kRegisterCount 各一次；真实电机上的读取与它指名的 C 入口一致（`enable()` 之后
// `State` 是 `Run`，`FloatBits` 解出来的是 `foc_get_state` 返回的**同一个** float，逐位相同而不是打印相同）；两类 C 函数对
// NULL 句柄的行为不同而表不得抹平（`foc_motor_get_status` 拒绝，于是读取失败且字不被触碰；`foc_motor_trace_size` 文档里
// NULL 就是 0，于是读取成功且得 0）——把两者归一成一个答案就是发明了第三种行为；表里没有的编号被拒绝，不猜。

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <set>
#include <string>

#include <gtest/gtest.h>

#include <foc/comm/register_map.hpp>

namespace {

namespace comm = foc::comm;

using comm::kRegisterCount;
using comm::read_register;
using comm::RegisterEntry;
using comm::RegisterId;
using comm::RegisterKind;
using comm::register_table;

struct Host final {
    float current_a[3]{0.0F, 0.0F, 0.0F};
    float angle_rad{0.0F};
    float v_bus_v{24.0F};
};

auto on_write_duty(void* user, const float /*duty_uvw*/[3]) -> void { static_cast<Host*>(user)->v_bus_v += 0.0F; }

auto on_read_current(void* user, float uvw_a[3]) -> void {
    const auto* host = static_cast<Host*>(user);
    for (int i = 0; i < 3; ++i) {
        uvw_a[i] = host->current_a[i];
    }
}

auto on_read_angle(void* user) -> float { return static_cast<Host*>(user)->angle_rad; }

auto on_read_voltage(void* user) -> float { return static_cast<Host*>(user)->v_bus_v; }

// The power stage's fault output: this host models a healthy inverter. Required by the library (see
// `foc_hal_callbacks_t::read_drive_fault`), so it is answered here, once, rather than left NULL.
// 功率级的故障输出：本主控方建模的是一个健康逆变器。库要求它在（见 `foc_hal_callbacks_t::read_drive_fault`），
// 所以在这里回答一次，而不是留成 NULL。
auto on_read_drive_fault(void* /*user*/) -> int32_t { return 0; }


auto callbacks_for(Host& host) -> foc_hal_callbacks_t {
    foc_hal_callbacks_t callbacks{};
    callbacks.struct_size = sizeof(foc_hal_callbacks_t);
    callbacks.user = &host;
    callbacks.pwm_write_duty = &on_write_duty;
    callbacks.read_current = &on_read_current;
    callbacks.read_angle_mech_rad = &on_read_angle;
    callbacks.read_voltage_v = &on_read_voltage;
    callbacks.read_drive_fault = &on_read_drive_fault;
    return callbacks;
}

auto nominal_params() -> foc_motor_params_t {
    foc_motor_params_t params{};
    params.rs = 0.5F;
    params.ld = 2.0e-4F;
    params.lq = 4.0e-4F;
    params.kt = 0.02F;
    params.j = 1.0e-5F;
    params.b = 1.0e-6F;
    params.pole_pairs = 1U;
    return params;
}

// 1. The address space is dense from 1 and every entry names a function.
TEST(RegisterMap, TheAddressSpaceIsDenseAndEveryRegisterNamesAFunction) {
    const RegisterEntry* table = register_table();
    std::set<std::uint16_t> ids;
    std::set<std::string> functions;
    for (std::size_t i = 0U; i < kRegisterCount; ++i) {
        const std::uint16_t id = static_cast<std::uint16_t>(table[i].id);
        EXPECT_TRUE(ids.insert(id).second) << "register " << id << " appears twice";
        EXPECT_GE(id, 1U);
        EXPECT_LE(id, kRegisterCount) << "the ids must be 1..kRegisterCount with no holes";
        ASSERT_NE(table[i].abi, nullptr);
        EXPECT_GT(std::strlen(table[i].abi), 3U) << "a register with no function behind it is a number, not a register";
        EXPECT_NE(table[i].read, nullptr);
        functions.insert(table[i].abi);
        // The ids must arrive in ascending order too, so a host may binary-search or walk without sorting.
        if (i > 0U) {
            EXPECT_EQ(static_cast<std::uint16_t>(table[i].id),
                      static_cast<std::uint16_t>(table[i - 1U].id) + 1U)
                << "the table is walked, so it has to be ordered";
        }
    }
    EXPECT_EQ(ids.size(), kRegisterCount);
    std::printf("\n[register map] %zu register(s) over %zu distinct C function(s)\n", static_cast<std::size_t>(kRegisterCount),
                functions.size());
    // Four functions for ten registers, and the sharing is not a coincidence to be tidied away: five of the registers
    // are fields of one struct and three are out-parameters of one call, so reading each group once is what keeps one
    // frame internally consistent. `foc_motor_trace_size` and `foc_motor_trace_dropped` are separate functions and
    // separate registers, which is the case that proves the mapping is not "one function per struct".
    EXPECT_EQ(functions.size(), 4U);
}

// 2 and 3. A real motor, a real read, and the two kinds of refusal.
TEST(RegisterMap, ARealReadAgreesWithTheEntryPointItNames) {
    Host host{};
    const foc_hal_callbacks_t callbacks = callbacks_for(host);
    const foc_motor_params_t params = nominal_params();
    foc_motor_t motor{};
    ASSERT_EQ(foc_motor_create_host(&params, nullptr, &callbacks, &motor), FOC_OK);
    ASSERT_EQ(foc_motor_enable(motor), FOC_OK);
    ASSERT_EQ(foc_set_torque(motor, 0.5F), FOC_OK);
    for (int i = 0; i < 50; ++i) {
        host.angle_rad = 0.1F * static_cast<float>(i);
        ASSERT_EQ(foc_tick(motor, 1.0F / 20000.0F), FOC_OK);
    }

    std::uint32_t word = 0U;
    ASSERT_TRUE(read_register(motor, RegisterId::State, &word));
    EXPECT_EQ(word, static_cast<std::uint32_t>(FOC_MOTOR_RUN));

    ASSERT_TRUE(read_register(motor, RegisterId::Error, &word));
    EXPECT_EQ(word, 0U) << "no fault was raised, so the sticky error is FOC_OK";

    // The float registers must be bit-exact copies of what the C entry point returns. Read both ways in the same instant.
    float speed_rad_s = 0.0F;
    float id_a = 0.0F;
    float iq_a = 0.0F;
    ASSERT_EQ(foc_get_state(motor, &speed_rad_s, &id_a, &iq_a), FOC_OK);
    std::uint32_t by_register = 0U;
    ASSERT_TRUE(read_register(motor, RegisterId::CurrentQ, &by_register));
    std::uint32_t by_call = 0U;
    std::memcpy(&by_call, &iq_a, sizeof(by_call));
    EXPECT_EQ(by_register, by_call) << "the register carries the word, not a rounded version of the number";

    // A count that cannot fail: no trace attached reads 0 and the read succeeds.
    ASSERT_TRUE(read_register(motor, RegisterId::TraceSize, &word));
    EXPECT_EQ(word, 0U);
    ASSERT_TRUE(read_register(motor, RegisterId::TraceDropped, &word));
    EXPECT_EQ(word, 0U);

    ASSERT_EQ(foc_motor_destroy(motor), FOC_OK);
}

TEST(RegisterMap, ARefusedReadLeavesTheWordAloneAndTheTwoKindsOfAnswerStayDifferent) {
    std::uint32_t word = 0xA5A5A5A5U;

    // `foc_motor_get_status` has an error channel, and a NULL handle is an error: the register read fails.
    EXPECT_FALSE(read_register(nullptr, RegisterId::State, &word));
    EXPECT_EQ(word, 0xA5A5A5A5U) << "a stale value paired with a fresh failure is worse than no value";

    // `foc_motor_trace_size` documents NULL as 0 and has no error channel: the read succeeds and says 0. A layer that
    // reported failure here would be contradicting the function it claims to be a register for.
    EXPECT_TRUE(read_register(nullptr, RegisterId::TraceSize, &word));
    EXPECT_EQ(word, 0U);

    // 4. An id with no entry is refused, and nothing is written.
    word = 0xA5A5A5A5U;
    EXPECT_FALSE(read_register(nullptr, static_cast<RegisterId>(9999), &word));
    EXPECT_EQ(word, 0xA5A5A5A5U);

    // A NULL receiver is refused rather than dereferenced.
    EXPECT_FALSE(read_register(nullptr, RegisterId::TraceSize, nullptr));
}

} // namespace
