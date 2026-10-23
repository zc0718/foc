// Conan::ImportStart
#pragma once

#include <foc/driver/scripted_current.hpp>
#include <foc/driver/sim_position.hpp>
#include <foc/driver/stub_dc_link.hpp>
#include <foc/driver/stub_pwm.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {

/**
 * @brief [en] A complete HalPolicy built from the reference stubs: the default for SIL and tests.
 *        Motor binds to it through the member names `pwm`, `current_sense`, `position`, `dc_link`.
 *        [zh] 由参考桩拼成的完整 HalPolicy：SIL 与测试的默认实现。Motor 通过成员名
 *        `pwm`、`current_sense`、`position`、`dc_link` 绑定它。
 *
 * @since 0.1.0
 */
struct StubHal final {
    using Pwm = StubPwm;
    using CurrentSense = ScriptedCurrent;
    using Position = SimPosition;
    using DcLink = StubDcLink;

    Pwm pwm;
    CurrentSense current_sense;
    Position position;
    DcLink dc_link;

    /**
     * @brief [en] Reset every member to its power-on state.
     *        [zh] 把所有成员复位到上电状态。
     * @since 0.1.0
     */
    void reset() noexcept;
};

} // namespace driver

} // namespace foc
