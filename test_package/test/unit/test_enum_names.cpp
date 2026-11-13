// Does every value the C ABI can hand a caller also have a name to compare against, and a name to print?
//
// The failure this file exists for is not a wrong number, it is a *missing* one: an enumerator is added to the C++
// side, the switch that names it is not updated, and the ABI answers "?" -- which is a perfectly valid string, so
// every downstream comparison keeps working and nothing goes red. The plan proposed buying a reflection library
// (`magic_enum`) for this. Measurement said otherwise, twice:
//
//   1. `-Wall -Werror` already makes a missing `case` a **compile error**: the naming switch has no `default:` label,
//      so `-Wswitch` fires. Measured 2026-11-06 by adding `InjectedGap` to `FaultCode` -- `bridge.cpp` refused to
//      compile with `error: enumeration value 'InjectedGap' not handled in switch [-Werror=switch]`.
//   2. The surface that actually needs checking is not the C++ enum, it is the **C enum** that mirrors it -- and no
//      reflection library can see that mirror, because it has no relationship to the C++ type except a comment.
//
// Point 2 was not hypothetical: `foc_fault_t` stopped at `FOC_FAULT_LOST_SYNC = 7` while `FaultCode` carried
// `DriveFault`, `Watchdog` and `ClockLost` (8, 9, 10). All three have producers (the nFAULT pin, the watchdog) and all
// three reach the caller through `bridge.cpp`'s `out->fault = static_cast<std::int32_t>(result.fault)`, so a C client
// could *receive* 8 and could not *name* it. `-Wswitch` cannot see this (the naming switch was complete), and a
// reflection library could not see it either. A test that walks the shipped ABI can.
//
// 这个文件针对的不是一个错的数，而是一个 **缺失** 的数：C++ 侧加了枚举值、给它命名的 switch 没跟上，ABI 就回答 "?" ——
// 而 "?" 是一个完全合法的字符串，于是下游所有比较照旧工作，什么都不会红。计划里为此准备引入一个反射库
// （`magic_enum`）。实测两次否掉了它：
//
//   1. `-Wall -Werror` 里已经让"漏一个 `case`"成为 **编译错误** ：命名 switch 没有 `default:` 标签，`-Wswitch` 会开火。
//      2026-11-06 实测：给 `FaultCode` 加一个 `InjectedGap`，`bridge.cpp` 拒绝编译，报
//      `error: enumeration value 'InjectedGap' not handled in switch [-Werror=switch]`。
//   2. 真正需要检查的面不是 C++ 枚举，而是 **镜像它的那个 C 枚举** —— 而任何反射库都看不见那个镜像，因为它与 C++ 类型
//      之间唯一的联系是一条注释。
//
// 第 2 点不是假想：`foc_fault_t` 停在 `FOC_FAULT_LOST_SYNC = 7`，而 `FaultCode` 已经带上了 `DriveFault`、
// `Watchdog`、`ClockLost`（8、9、10）。三个都有生产者（nFAULT 引脚、看门狗），三个都经
// `bridge.cpp` 的 `out->fault = static_cast<std::int32_t>(result.fault)` 交到调用方手里 —— 也就是说一个 C 客户端
// **收得到** 8，却 **叫不出** 它。`-Wswitch` 看不见这件事（命名 switch 是完整的），反射库同样看不见。只有穿过
// 发布 ABI 走一遍的用例能看见。

#include <cstdint>
#include <cstring>
#include <set>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include <foc/api/foc_api.h>
#include <foc/core/board_check.hpp>
#include <foc/core/protection.hpp>

namespace {

using NameFn = const char* (*)(std::int32_t);

/**
 * @brief [en] The names the ABI returns for `0..last`, and whether they form one contiguous, distinct, real set.
 * @brief [zh] ABI 对 `0..last` 返回的名字，以及它们是否构成一个连续、互不相同、且真实存在的集合。
 */
struct Walk final {
    std::vector<std::string> names;  ///< one per value, in order
    std::vector<std::string> faults; ///< what went wrong, empty when the walk is clean
    [[nodiscard]] auto clean() const -> bool { return faults.empty(); }
};

auto walk(NameFn name, std::int32_t last) -> Walk {
    Walk result;
    std::set<std::string> seen;
    for (std::int32_t value = 0; value <= last; ++value) {
        const char* spelt = name(value);
        if (spelt == nullptr) {
            result.faults.push_back("value " + std::to_string(value) + " has a null name");
            result.names.emplace_back();
            continue;
        }
        result.names.emplace_back(spelt);
        if (std::strcmp(spelt, "?") == 0) {
            result.faults.push_back("value " + std::to_string(value) + " is inside the declared domain but unnamed");
        }
        if (!seen.insert(spelt).second) {
            result.faults.push_back("value " + std::to_string(value) + " is called \"" + spelt +
                                    "\", which another value already uses -- a caller cannot tell them apart");
        }
    }
    // One past the last declared value: the domain has to end exactly where the header says it does, or a caller
    // iterating `for (v = 0; v <= LAST; ++v)` silently skips a real value.
    // 声明的最后一个值再往后一格：取值域必须恰好停在头文件说的地方，否则按 `for (v = 0; v <= LAST; ++v)` 迭代的调用方
    // 会静默漏掉一个真实存在的值。
    const char* beyond = name(last + 1);
    if ((beyond != nullptr) && (std::strcmp(beyond, "?") != 0)) {
        result.faults.push_back("value " + std::to_string(last + 1) + " is named \"" + beyond +
                                "\" but is outside the declared domain");
    }
    const char* negative = name(-1);
    if ((negative != nullptr) && (std::strcmp(negative, "?") != 0)) {
        result.faults.push_back("value -1 is named \"" + std::string(negative) + "\"; an out-of-range code must be '?'");
    }
    return result;
}

auto report(const char* what, const Walk& result) -> std::string {
    std::string text;
    for (const auto& fault : result.faults) {
        text += what;
        text += ": ";
        text += fault;
        text += "\n";
    }
    return text;
}

} // namespace

/**
 * @brief [en] Every fault code the ABI can return has its own name, and the C enum mirrors the C++ one value for value.
 *
 * @brief [zh] ABI 能返回的每个故障码都有自己的名字，而 C 枚举与 C++ 枚举逐值对应。
 *
 *   Three legs, and the third is the one that was missing from the plans. (a) The naming switch is complete for every
 *   value in `0..FOC_FAULT_CLOCK_LOST` and every name is distinct -- distinctness matters because that is what a caller
 *   does with the string: prints it, or diffs it into a log. (b) `FOC_FAULT_CLOCK_LOST + 1` answers `"?"`, so the
 *   declared domain and the switch agree at the boundary as well as in the middle. (c) Each C macro equals the C++
 *   enumerator it mirrors. Leg (c) is the one `-Wswitch` and every reflection library are blind to, and it was failing
 *   on 2026-11-06 for `DriveFault`/`Watchdog`/`ClockLost`, whose macros did not exist.
 *
 *   **Not claimed**: a value appended after the last enumerator *and* a `default:` label added to the switch *and* the
 *   range check in `foc_bridge_fault_name` widened would still pass this case. That combination cannot be caught
 *   without reflection, and the two-and-a-half of the three steps it needs are each caught by the compiler; the cost of
 *   closing it is a dependency that makes every consumer compile 277 extra headers (see `.github/project/deps.json`),
 *   which is not a trade this repository makes for the remaining corner.
 *
 *   三条腿，而第三条正是计划里缺的那条。(a) 命名 switch 对 `0..FOC_FAULT_CLOCK_LOST` 每个值都完整，且名字互不相同 —— 不
 *   重复是要紧的，因为调用方拿这个字符串去打印、去和日志做差异。(b) `FOC_FAULT_CLOCK_LOST + 1` 回答 `"?"`，于是声明的取
 *   值域与 switch 在边界上和在中间都一致。(c) 每个 C 宏等于它镜像的 C++ 枚举值。第 (c) 条正是 `-Wswitch` 与任何反射库都
 *   看不见的那条，而它在 2026-11-06 对 `DriveFault`/`Watchdog`/`ClockLost` 是 **不成立** 的：那三个宏根本不存在。
 *
 *   **不宣称** ：一个追加在末值之后的新枚举值， **并且** 给 switch 加了 `default:` 标签， **并且** 把
 *   `foc_bridge_fault_name` 里的范围检查放宽——这三件事凑齐了仍能过本用例。没有反射就抓不到这种组合，而它需要的三步里有
 *   两步半都被编译器盯着；而关闭这个角落的代价是一个让每个消费者多编 277 个头的依赖（见 `.github/project/deps.json`
 *   ），这个仓库不为剩下的那个角付这个价。
 */
TEST(EnumNames, EveryFaultTheAbiCanReturnHasItsOwnNameAndItsOwnMacro) {
    // (c) first: the mirror. These are the assertions that failed before the fix, so they also document the gap.
    // (c) 先来：镜像。这批断言在修复前是失败的，因此它们同时记录了这个缺口。
    EXPECT_EQ(static_cast<std::int32_t>(foc::core::FaultCode::None), FOC_FAULT_NONE);
    EXPECT_EQ(static_cast<std::int32_t>(foc::core::FaultCode::OverCurrent), FOC_FAULT_OVER_CURRENT);
    EXPECT_EQ(static_cast<std::int32_t>(foc::core::FaultCode::OverVoltage), FOC_FAULT_OVER_VOLTAGE);
    EXPECT_EQ(static_cast<std::int32_t>(foc::core::FaultCode::UnderVoltage), FOC_FAULT_UNDER_VOLTAGE);
    EXPECT_EQ(static_cast<std::int32_t>(foc::core::FaultCode::OverTemperature), FOC_FAULT_OVER_TEMPERATURE);
    EXPECT_EQ(static_cast<std::int32_t>(foc::core::FaultCode::I2t), FOC_FAULT_I2T);
    EXPECT_EQ(static_cast<std::int32_t>(foc::core::FaultCode::Stall), FOC_FAULT_STALL);
    EXPECT_EQ(static_cast<std::int32_t>(foc::core::FaultCode::LostSync), FOC_FAULT_LOST_SYNC);
    EXPECT_EQ(static_cast<std::int32_t>(foc::core::FaultCode::DriveFault), FOC_FAULT_DRIVE_FAULT);
    EXPECT_EQ(static_cast<std::int32_t>(foc::core::FaultCode::Watchdog), FOC_FAULT_WATCHDOG);
    EXPECT_EQ(static_cast<std::int32_t>(foc::core::FaultCode::ClockLost), FOC_FAULT_CLOCK_LOST);

    // (a) and (b): the shipped naming function, walked over its whole declared domain.
    // (a) 与 (b)：发布出去的那个命名函数，把它的整个声明取值域走一遍。
    const Walk faults = walk(&foc_fault_name, static_cast<std::int32_t>(foc::core::FaultCode::ClockLost));
    ASSERT_TRUE(faults.clean()) << report("fault names", faults);
    EXPECT_EQ(faults.names.size(), static_cast<std::size_t>(FOC_FAULT_CLOCK_LOST) + 1U);

    // The same walk over the two board reports. They are in this file because they are the same promise: a C caller
    // that got a number out of a report struct must be able to print it without knowing the enum.
    // 同样走一遍两份板级报告。它们放在本文件里，因为它们许的是同一个诺言：一个从报告结构里拿到数字的 C 调用方，必须
    // 能在不知道枚举的情况下把它打印出来。
    const Walk items = walk(&foc_board_item_name, static_cast<std::int32_t>(foc::core::BoardItem::ObserverHealth));
    ASSERT_TRUE(items.clean()) << report("board item names", items);

    const Walk causes = walk(&foc_board_cause_name, static_cast<std::int32_t>(foc::core::BoardCause::ObserverUnhealthy));
    ASSERT_TRUE(causes.clean()) << report("board cause names", causes);

    std::printf("\n[enum-names] faults %zu, board items %zu, board causes %zu: every value named, none shared\n",
                faults.names.size(), items.names.size(), causes.names.size());
}
