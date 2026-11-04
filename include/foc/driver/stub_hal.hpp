// Conan::ImportStart
#pragma once
#include <foc/driver/scripted_current.hpp>
#include <foc/driver/sim_position.hpp>
#include <foc/driver/stub_fault_input.hpp>
#include <foc/driver/stub_dc_link.hpp>
#include <foc/driver/stub_pwm.hpp>
// Conan::ImportEnd

namespace foc {

namespace driver {


/**
 * @brief [en] A complete HalPolicy built from the reference stubs: the default for SIL and tests. Motor binds to it
 *             through the member names `pwm`, `current_sense`, `position`, `dc_link`.
 * @brief [zh] 由参考桩拼成的完整 HalPolicy：SIL 与测试的默认实现。Motor 通过成员名 `pwm`、`current_sense`、`position`、
 *             `dc_link` 绑定它。
 * @since 0.1.0
 * @exporter
 */
struct StubHal final {
    using Pwm = StubPwm;
    using CurrentSense = ScriptedCurrent;
    using Position = SimPosition;
    using DcLink = StubDcLink;
    using FaultInput = StubFaultInput;

    Pwm pwm;
    CurrentSense current_sense;
    Position position;
    DcLink dc_link;
    /**
     * @brief [en] The power stage's fault output: present and quiet, and injection is what a fault case does.
     * @brief [zh] 功率级的故障输出：在、且安静，注入是故障用例要做的事。
     * @note [en] Present because it is **required**: a HalPolicy without it cannot start a motor (`Motor::start()`
     *            answers `NotImpl`). Quiet because a stub power stage has nothing to report -- which is a statement
     *            this struct makes explicitly rather than a default the library assumed.
     * @note [zh] 它在，是因为它是**必需的**：没有它的 HalPolicy 无法启动电机（`Motor::start()` 答 `NotImpl`）。它安静，
     *            是因为桩功率级没有东西可报——这句话由这个结构**明说**，而不是由库默认。
     * @since 0.1.9
     */
    FaultInput fault_input;

    /**
     * @brief [en] Reset every member to its power-on state.
     * @brief [zh] 把所有成员复位到上电状态。
     * @since 0.1.0
     */
    void reset() noexcept;
};

} // namespace driver

} // namespace foc
